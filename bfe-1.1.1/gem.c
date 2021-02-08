/*
 *  gem.c: general ethernet mac driver framework v1.1
 *
 * Copyright (c) 2002-2006 Masayuki Murayama.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * 3. Neither the name of the author nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#pragma ident	"@(#)gem.c	1.77 06/05/27"

/*
 Change log
 04/08/2003  gem_get_packet_default fixed for sparc
 05/18/2003  no interrupt option implemented
 10/06/2003  link watcher rewriten
 04/01/2004  license changed
 11/04/2004  threshold of tx desc changed; 2*desc => 3*desc
 12/10/2004  txthr 256 -> 1024
 01/04/2005  txthr 1024->ETHERMAX(1514)
 05/25/2005  mii power down and isolate removed
 02/01/2005  property (nointr, flowcontrol) fixed
 02/04/2006  address of bounce buffer with rx header fixed
 */

/*
 TODO:
	separete allocation descriptors and bufferes
	txbuf allocation in attach
	fix rx_desc_write/tx_desc_write interface
	programable mtu, txmaxdma, rxmaxdma, txthr, rxthr
 */

/*
 * System Header files.
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strlog.h>
#include <sys/kmem.h>
#include <sys/stat.h>
#include <sys/kstat.h>
#include <sys/vtrace.h>
#include <sys/dlpi.h>
#include <sys/strsun.h>
#include <sys/ethernet.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/dditypes.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#include <sys/pci.h>

#include "mii.h"
#include "gem.h"

#if defined(GEM_CONFIG_CKSUM_OFFLOAD) && !defined(SOLARIS10)
/* GEM_CONFIG_CKSUM_OFFLOAD requires solaris10 or later*/
# define SOLARIS10
#endif
#if defined(GEM_CONFIG_VLAN) && !defined(SOLARIS10)
/* GEM_CONFIG_VLAN requires solaris10 or later */
# define SOLARIS10
#endif

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
#include <sys/pattr.h>
#include <netinet/in.h>
#endif

#ifdef MODULE
char	ident[] = "general ethernet mac driver v" VERSION;
char	_depends_on[] = {"misc/gld"};
#else
# ifdef GEM_COMPAT
  char	ident[] = "general ethernet mac driver v" VERSION;
# else
  extern char ident[];
# endif
#endif

#ifndef GEM_GIGA
#  define GEM_GIGA
#endif

/* Debugging support */
#ifdef GEM_DEBUG_LEVEL
static int gem_debug = GEM_DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (gem_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#undef ASSERT
#define ASSERT(x)
#endif

#define	IOC_LINESIZE	0x40	/* Is it right for amd64? */

/*
 * Useful macros and typedefs
 */
#define	FALSE	(0)
#define	TRUE	(!FALSE)
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))
#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))
#define	TXINTR_SCHEDULED(dp)	((dp)->tx_desc_intr - (dp)->tx_desc_head > 0)

#define	GET_NET16(p)	((((uint8_t *)(p))[0] << 8)| ((uint8_t *)(p))[1])

#define	GET_ETHERTYPE(p)	GET_NET16(((uint8_t *)(p)) + ETHERADDRL*2)

#define	GET_IPTYPE(p)	(((uint8_t *)(p))[sizeof(struct ether_header) + 9])

#ifndef INT32_MAX
# define INT32_MAX	0x7fffffff
#endif

/*
 * configuration parameters
 */
#ifdef GEM_CONFIG_VLAN
#define	MAXPKTLEN(dp)	((dp)->mtu + sizeof(struct ether_header) + VTAG_SIZE)
#else
#define	MAXPKTLEN(dp)	((dp)->mtu + sizeof(struct ether_header))
#endif
#define	MAXPKTBUF(dp)	(MAXPKTLEN(dp) + ETHERFCSL)

#define	WATCH_INTERVAL_FAST	drv_usectohz(100*1000)

/*
 * Macros to distinct chip generation.
 */

/*
 * Private functions
 */
static void gem_mii_start(struct gem_dev *);
static void gem_mii_stop(struct gem_dev *);

/* local buffer management */
/* static struct rxbuf *gem_get_rxbuf(struct gem_dev *, int); */
/* static void gem_free_rxbuf(struct rxbuf *); */
static int gem_alloc_memory(struct gem_dev *);
static void gem_free_memory(struct gem_dev *);
static void gem_init_rx_desc(struct gem_dev *);
static void gem_init_tx_desc(struct gem_dev *);
static void gem_append_rxbuf(struct gem_dev *, struct rxbuf *, int);

static void gem_tx_start_unit(struct gem_dev *dp);

static void gem_tx_timeout(struct gem_dev *);

/* GLD interfaces */
static int gem_gld_reset(gld_mac_info_t *);
static int gem_gld_start(gld_mac_info_t *);
static int gem_gld_stop(gld_mac_info_t *);
static int gem_gld_set_mac_address(gld_mac_info_t *, uint8_t *);
static int gem_gld_set_multicast(gld_mac_info_t *, uint8_t *, int);
static int gem_gld_set_promiscuous(gld_mac_info_t *, int);
static int gem_gld_get_stats(gld_mac_info_t *, struct gld_stats *);
static int gem_gld_send(gld_mac_info_t *, mblk_t *);
static int gem_gld_send_tagged(gld_mac_info_t *, mblk_t *, uint32_t);
static u_int gem_gld_intr(gld_mac_info_t *);

static	struct ether_addr	gem_etherbroadcastaddr = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

int gem_speed_value[] = {10, 100, 1000};

static ddi_dma_attr_t gem_dma_attr_get_paddr = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x7fffffffffffffffull,	/* dma_attr_addr_hi */
	0x7fffffffffffffffull,	/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	8,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

#ifdef MODULE
extern struct mod_ops mod_miscops;

static struct modlmisc modlmisc = {
	&mod_miscops,
	"gem v" VERSION,
};

static struct modlinkage modlinkage = {
	MODREV_1, &modlmisc, NULL
};

/*
 * _init : done
 */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, "!gem: _init: called"));
	status = mod_install(&modlinkage);

	return (status);
}

/*
 * _fini : done
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!gem: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
#endif /* MODULE */

/* ============================================================== */
/*
 * gcc3 runtime routines
 */
/* ============================================================== */
#ifdef GEM_GCC_RUNTIME
int
memcmp(const void *s1, const void *s2, size_t n)
{
	int	i;
	int	ret;

	ret = 0;
	for (i = 0; i < n; i++) {
		ret = (int)((uint8_t *)s1)[i] - (int)((uint8_t *)s2)[i];
		if (ret != 0) {
			return (ret);
		}
	}
	return (0);
}

void *
memset(void *s, int c, size_t n)
{
	if ((c & 0xff) == 0) {
		bzero(s, n);
	}
	else {
		while (n--) {
			((uint8_t *)s)[n] = c;
		}
	}
	return (s);
}

void *
memcpy(void *s1, const void *s2, size_t n)
{
	bcopy(s2, s1, n);
	return (s1);
}
#endif /* GEM_GCC_RUNTIME */

static int
gem_prop_get_int(struct gem_dev *dp, char *prop_template, int def_val)
{
	char	propname[32];

	sprintf(propname, prop_template, dp->name);

	return (ddi_prop_get_int(DDI_DEV_T_ANY, dp->dip,
				DDI_PROP_DONTPASS, propname, def_val));
}

/* ============================================================== */
/*
 * Misc runtime routines
 */
/* ============================================================== */
/*
 * Ether CRC calculation according to 21143 data sheet
 */
#define	CRC32_POLY_LE	0xedb88320
uint32_t
gem_ether_crc_le(uint8_t *addr)
{
	int		idx;
	int		bit;
	u_int		data;
	uint32_t	crc = 0xffffffff;

	crc = 0xffffffff;
	for (idx = 0; idx < ETHERADDRL; idx++) {
		for (data = *addr++, bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc >> 1)
			    ^ (((crc ^ data) & 1) ? CRC32_POLY_LE : 0);
		}
	}
	return (crc);
}

#define	CRC32_POLY_BE	0x04c11db7
uint32_t
gem_ether_crc_be(uint8_t *addr)
{
	int		idx;
	int		bit;
	u_int		data;
	uint32_t	crc;

	crc = 0xffffffff;
	for (idx = 0; idx < ETHERADDRL; idx++) {
                for (data = *addr++, bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc << 1)
			    ^ ((((crc >> 31) ^ data) & 1) ? CRC32_POLY_BE : 0);
		}
	}
	return (crc);
}

/* ============================================================== */
/*
 * Buffer management
 */
/* ============================================================== */

/*
 * gem_get_rxbuf: supply a receive buffer which have been mapped into
 * DMA space.
 */
struct rxbuf *
gem_get_rxbuf(struct gem_dev *dp, int cansleep)
{
	struct rxbuf		*rbp;
	ddi_dma_cookie_t	*dma;
	uint_t			count = 0;
	int			i;
	int			err;
	int			align;

	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(3, (CE_CONT, "!gem_get_rxbuf: called freecnt:%d",
		dp->rx_buf_freecnt));
	/*
	 * Get rx buffer management structure
	 */
	if ((rbp = dp->rx_buf_freelist) != NULL) {
		/* get one from the recycle list */
		ASSERT(dp->rx_buf_freecnt > 0);

		dp->rx_buf_freelist = rbp->rxb_next;
		dp->rx_buf_freecnt--;

		if (rbp->rxb_mp != NULL) {
			/*
			 * All required resources have been prepared,
			 * just return.
			 */
			ASSERT(rbp->rxb_nfrags > 0);
			return (rbp);
		}

		goto allocate_buffer;
	}

	if (dp->rx_buf_allocated > dp->gc.gc_rx_buf_max) {
		/*
		 * the number of allocated rx buffers was exceeded the limit.
		 */
		return (NULL);
	}

	/*
	 * Allocate a rx buffer management structure
	 */
	if ((rbp = (struct rxbuf *) kmem_zalloc(sizeof(struct rxbuf),
			cansleep ? KM_SLEEP : KM_NOSLEEP)) == NULL) {
		/* no memory */
		return (NULL);
	}

#ifdef SANITY
	rbp->rxb_buf = NULL;
	rbp->rxb_mp  = NULL;
	rbp->rxb_bah = NULL;
#endif
	/*
	 * Prepare a back pointer to the device structure which will be
	 * needed on freeing the buffer.
	 */
	rbp->rxb_devp = dp;

	/* allocate a dma handle for a receive buffer */
	if ((err = ddi_dma_alloc_handle(dp->dip,
#if defined(sun4u)
			&dp->gc.gc_dma_attr_rxbuf,
#else
			&gem_dma_attr_get_paddr,
#endif
			cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT,
			NULL, &rbp->rxb_dh)) != DDI_SUCCESS) {

		cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_alloc_handle:1: failed, err=%d",
			dp->name, __func__, err);

		kmem_free(rbp, sizeof(struct rxbuf));
		return (NULL);
	}

	/* allocate a dma handle for a rx bounce buffer */
	if ((err = ddi_dma_alloc_handle(dp->dip,
			&dp->gc.gc_dma_attr_rxbuf,
			cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT,
			NULL, &rbp->rxb_bdh)) != DDI_SUCCESS) {

		cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_alloc_handle:2: failed, err=%d",
			dp->name, __func__, err);

		ddi_dma_free_handle(&rbp->rxb_dh);
		kmem_free(rbp, sizeof(struct rxbuf));
		return (NULL);
	}


	/* allocate a bounce buffer for rx */
	if ((err = ddi_dma_mem_alloc(rbp->rxb_bdh,
#ifdef BUG_MEM_ALLOC	/* obsoluleted in Solaris10 FCS */
		ROUNDUP(dp->rx_buf_len, PAGESIZE),
#else
		ROUNDUP(dp->rx_buf_len, IOC_LINESIZE),
#endif
		&dp->gc.gc_buf_attr,
		/*
		 * if the nic requires a header at the top of receive buffers,
		 * it may access the rx buffer randomly.
		 */
		(dp->gc.gc_rx_header_len > 0) ? 0 : DDI_DMA_STREAMING,
		cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT,
		NULL,
		&rbp->rxb_bbuf, &rbp->rxb_bbuf_len,
		&rbp->rxb_bah)) != DDI_SUCCESS) {

		cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_mem_alloc: failed, err=%d",
			dp->name, __func__, err);

		ddi_dma_free_handle(&rbp->rxb_dh);
		ddi_dma_free_handle(&rbp->rxb_bdh);
		kmem_free(rbp, sizeof(struct rxbuf));
		return (NULL);
	}

	/* Now we successfully prepared the management section of rx buffer */
	dp->rx_buf_allocated++;

allocate_buffer:
	if (dp->gc.gc_dma_attr_rxbuf.dma_attr_align > 8) {
		/* this alignment includes IOC_LINESIZE */
		align = dp->gc.gc_dma_attr_rxbuf.dma_attr_align;
	} else {
		/*
		 * As allocb() will return an 8-byte-aligned buffer,
		 * no need to manipulate starting addresss of it.
		 */
		align = 1;
	}
	if ((rbp->rxb_mp = allocb(ROUNDUP(dp->rx_buf_len, IOC_LINESIZE)
							+ align - 1,
							BPRI_MED)) == NULL) {
		gem_free_rxbuf(rbp);
		return (NULL);
	}
	rbp->rxb_buf = (caddr_t) ROUNDUP((long)rbp->rxb_mp->b_rptr, align);
	rbp->rxb_buf_len = dp->rx_buf_len;
	rbp->rxb_mp->b_rptr = (uchar_t *)rbp->rxb_buf + dp->gc.gc_rx_header_len;
	rbp->rxb_flags &= ~RXB_FLAGS_BOUNCE;

	if (MAXPKTBUF(dp) <= dp->gc.gc_rx_copy_thresh) {
use_bounce_buffer:
		/*
		 * Copy case: try to map the bounce buffer into the DMA space,
		 * instead of the mblk.
		 */
		DPRINTF(4, (CE_CONT,
			"!%s: mapping rx bounce buffer", dp->name));
		rbp->rxb_buf     = rbp->rxb_bbuf;
		rbp->rxb_buf_len = rbp->rxb_bbuf_len;
		rbp->rxb_flags  |= RXB_FLAGS_BOUNCE;
#ifdef GEM_DEBUG_LEVEL
		dp->stats.norcvbuf++;
#endif
	}

	/* Mapin the data buffer into the DMA space */
	if ((err = ddi_dma_addr_bind_handle(rbp->rxb_dh,
				NULL, rbp->rxb_buf, dp->rx_buf_len,
				((dp->gc.gc_rx_header_len > 0)
					? DDI_DMA_RDWR
					:(DDI_DMA_READ | DDI_DMA_STREAMING)) |
				  DDI_DMA_PARTIAL,
				cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT,
				NULL,
				rbp->rxb_dmacookie,
				&count)) != DDI_DMA_MAPPED) {

		ASSERT(err != DDI_DMA_INUSE);
		DPRINTF(0, (CE_NOTE,
			"!%s:switch to rx bounce buffer: "
			"ddi_dma_addr_bind_handle: failed, err=%d",
			dp->name, err));

		if (err == DDI_DMA_PARTIAL_MAP) {
			/* need to free partially allocated dma resources. */
			ddi_dma_unbind_handle(rbp->rxb_dh);
		}

		if ((rbp->rxb_flags & RXB_FLAGS_BOUNCE) != 0) {
			/*
			 * failed to allocate a dma resource
			 * for the rx bounce buffer.
			 */
			freemsg(rbp->rxb_mp);
			rbp->rxb_mp      = NULL;
			rbp->rxb_buf     = NULL;
			rbp->rxb_buf_len = 0;
			gem_free_rxbuf(rbp);
			return (NULL);
		}
		/*
		 * change current configuration for rx to choose "copy" 
		 */
		dp->gc.gc_rx_copy_thresh = MAXPKTBUF(dp);
		goto use_bounce_buffer;
	}

	if (count > dp->gc.gc_rx_max_frags) {
		/*
		 * Current configuration for rx isn't suitable for the
		 * DMA capability of the nic.
		 */
		DPRINTF(0, (CE_NOTE,
			"!%s: switch to rx bounce buffer: "
			"too many rx fragments",
			dp->name));
#ifdef SANITY
		for (i = 1; i < count; i++) {
			ddi_dma_nextcookie(rbp->rxb_dh, &rbp->rxb_dmacookie[0]);
		}
#endif
		/* free allocated DMA resources */
		ddi_dma_unbind_handle(rbp->rxb_dh);

		/* let the driver to choose "copy" on rx. */
		dp->gc.gc_rx_copy_thresh = MAXPKTBUF(dp);

		ASSERT((rbp->rxb_flags & RXB_FLAGS_BOUNCE) == 0);
		goto use_bounce_buffer;
	}

	/* correct the rest of the DMA mapping */
	for (i = 1; i < count; i++) {
		ddi_dma_nextcookie(rbp->rxb_dh, &rbp->rxb_dmacookie[i]);
	}
	rbp->rxb_nfrags = count;

	/*
	 * XXX - Check if the dma range is valid because
	 * ddi_dma_addr_bind_handle() returns successfully even if
	 * the physical address range of mblk isn't suitable for the dma
	 * capability of the nic.
	 */
	for (i = 0, dma = rbp->rxb_dmacookie; i < count; i++, dma++) {

		DPRINTF(3, (CE_CONT,
			"! frag:%dth: dma_addr:0x%llx len:0x%x",
			i, dma->dmac_laddress, dma->dmac_size));

		if (dma->dmac_laddress <
				dp->gc.gc_dma_attr_rxbuf.dma_attr_addr_lo
		   || dma->dmac_laddress + (dma->dmac_size - 1) >
				dp->gc.gc_dma_attr_rxbuf.dma_attr_addr_hi) {
			/*
			 * release the dma mapping, and change current
			 * configuration to use rx bounce buffers.
			 */
			ddi_dma_unbind_handle(rbp->rxb_dh);
			dp->gc.gc_rx_copy_thresh = MAXPKTBUF(dp);
			DPRINTF(0, (CE_NOTE,
				"!%s: switch to rx bounce buffer: "
				"the dma range is not suitabble for the nic",
				dp->name));

			ASSERT((rbp->rxb_flags & RXB_FLAGS_BOUNCE) == 0);
			goto use_bounce_buffer;
		}
	}

	return (rbp);
}

void
gem_free_rxbuf(struct rxbuf *rbp)
{
	struct gem_dev	*dp;

	dp = rbp->rxb_devp;

	ASSERT(mutex_owned(&dp->intrlock));

	ASSERT(rbp->rxb_mp != NULL && rbp->rxb_nfrags ||
		rbp->rxb_mp == NULL && rbp->rxb_nfrags == 0);

	rbp->rxb_next = dp->rx_buf_freelist;
	dp->rx_buf_freelist = rbp;
	dp->rx_buf_freecnt++;
}

/* ============================================================== */
/*
 * memory resource management
 */
/* ============================================================== */
static int
gem_alloc_memory(struct gem_dev* dp)
{
	caddr_t			ring;
	caddr_t			buf;
	size_t			req_size;
	size_t			ring_len;
	size_t			buf_len;
	ddi_dma_cookie_t	ring_cookie;
	ddi_dma_cookie_t	buf_cookie;
	uint_t			count;
	int			i;
	int			j;
	int			err;
	struct rxbuf		*rbp, *head;
	struct txbuf		*tbp;
	const int		tx_ring_size = dp->gc.gc_tx_ring_size;
	const int		tx_buf_size  = dp->gc.gc_tx_buf_size;
	const int		rx_ring_size = dp->gc.gc_rx_ring_size;
	int			tx_buf_len;
	ddi_dma_attr_t		dma_attr_txbounce;

	dp->desc_dma_handle = NULL;
	req_size = dp->gc.gc_rx_desc_size + dp->gc.gc_tx_desc_size;
	if (req_size > 0) {
		/*
		 * Alloc RX/TX descriptors and a tx padding buffer.
		 */
		if ((err = ddi_dma_alloc_handle(dp->dip,
				&dp->gc.gc_dma_attr_desc,
				DDI_DMA_SLEEP,NULL,
				&dp->desc_dma_handle)) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
				"!%s: %s: ddi_dma_alloc_handle failed: %d",
				dp->name, __func__, err);
			return (ENOMEM);
		}

		if ((err = ddi_dma_mem_alloc(dp->desc_dma_handle,
				req_size, &dp->gc.gc_desc_attr,
				DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
				&ring, &ring_len,
				&dp->desc_acc_handle)) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
				"!%s: %s: ddi_dma_mem_alloc failed: "
				"ret %d, request size: %d",
				dp->name, __func__, err, req_size);
			ddi_dma_free_handle(&dp->desc_dma_handle);
			return (ENOMEM);
		}

		if ((err = ddi_dma_addr_bind_handle(dp->desc_dma_handle,
				NULL, ring, ring_len,
				DDI_DMA_RDWR | DDI_DMA_CONSISTENT,
				DDI_DMA_SLEEP, NULL,
				&ring_cookie, &count)) != DDI_SUCCESS) {
			ASSERT(err != DDI_DMA_INUSE);
			cmn_err(CE_WARN,
				"!%s: %s: ddi_dma_addr_bind_handle failed: %d",
				dp->name, __func__, err);
			ddi_dma_mem_free(&dp->desc_acc_handle);
			ddi_dma_free_handle(&dp->desc_dma_handle);
			return (ENOMEM);
		}
		ASSERT(count == 1);

		/* set base of rx descriptor ring */
		if (dp->gc.gc_rx_desc_size > 0) {
			dp->rx_ring     = ring;
			dp->rx_ring_dma = ring_cookie.dmac_laddress;
		}
		/* set base of tx descriptor ring */
		if (dp->gc.gc_tx_desc_size > 0) {
			dp->tx_ring     = ring + dp->gc.gc_rx_desc_size;
			dp->tx_ring_dma = ring_cookie.dmac_laddress
					+ dp->gc.gc_rx_desc_size;
		}
	}

#ifdef notyet
	/* allocate rx ring buffer */
#endif
	if (dp->gc.gc_tx_max_frags > 0) {
		/*
		 * Prepare tx buffers and bounce buffers.
		 */

		/* special dma attribute for tx bounce buffer */
		STRUCT_COPY(dma_attr_txbounce, dp->gc.gc_dma_attr_txbuf);
		dma_attr_txbounce.dma_attr_sgllen = 1;
		dma_attr_txbounce.dma_attr_align =
			max(dma_attr_txbounce.dma_attr_align, IOC_LINESIZE);

		/* Size for tx bounce buffers must be max of tx packet size. */
		tx_buf_len = MAXPKTBUF(dp);
#ifdef BUG_MEM_ALLOC	/* obsoluleted in Solaris10 FCS */
		tx_buf_len = ROUNDUP(tx_buf_len, PAGESIZE);
#else
		tx_buf_len = ROUNDUP(tx_buf_len, IOC_LINESIZE);
#endif
		ASSERT(tx_buf_len >= ETHERMAX+ETHERFCSL);
		for (i = 0, tbp = dp->tx_buf; i < tx_buf_size; i++, tbp++) {
			/*
			 * Allocate dma handles for each fragment of
			 * buffers for direct transmittion.
			 */
			for (j = 0; j < GEM_MAXTXSEGS; j++) {
				if (ddi_dma_alloc_handle(dp->dip,
#if defined(sun4u)
					&dp->gc.gc_dma_attr_txbuf,
#else
					&gem_dma_attr_get_paddr,
#endif
					DDI_DMA_SLEEP, NULL,
					&tbp->txb_dh[j]) != DDI_SUCCESS) {

					cmn_err(CE_WARN,
					"!%s: %s: ddi_dma_alloc_handle failed.",
						dp->name, __func__);
					goto err_alloc_dh;
				}
			}

			/* setup bounce buffers for tx */
			/* new way; allocate a bounce buffer for each txbuf */
			if (ddi_dma_alloc_handle(dp->dip,
					&dma_attr_txbounce,
					DDI_DMA_SLEEP, NULL,
					&tbp->txb_bdh) != DDI_SUCCESS) {

				cmn_err(CE_WARN,
		"!%s: %s: ddi_dma_alloc_handle for bounce buffer failed.",
					dp->name, __func__);
				goto err_alloc_dh;
			}

			if ((err = ddi_dma_mem_alloc(tbp->txb_bdh,
					tx_buf_len,
					&dp->gc.gc_buf_attr,
					DDI_DMA_STREAMING, DDI_DMA_SLEEP, NULL,
					&buf, &buf_len,
					&tbp->txb_bah)) != DDI_SUCCESS) {
				cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_mem_alloc for bouce buffer failed: "
					"ret %d, request size %d",
					dp->name, __func__, err, tx_buf_len);
				ddi_dma_free_handle(&tbp->txb_bdh);
				goto err_alloc_dh;
			}

			if ((err = ddi_dma_addr_bind_handle(tbp->txb_bdh,
					NULL, buf, buf_len,
					DDI_DMA_WRITE | DDI_DMA_STREAMING,
					DDI_DMA_SLEEP, NULL,
					&buf_cookie, &count)) != DDI_SUCCESS) {
				ASSERT(err != DDI_DMA_INUSE);
				cmn_err(CE_WARN,
	"!%s: %s: ddi_dma_addr_bind_handle for bounce buffer failed: %d",
					dp->name, __func__, err);
				ddi_dma_mem_free(&tbp->txb_bah);
				ddi_dma_free_handle(&tbp->txb_bdh);
				goto err_alloc_dh;
			}
			ASSERT(count == 1);
			tbp->txb_buf     = buf;
			tbp->txb_buf_dma = buf_cookie.dmac_laddress;
		}
	}
	return (0);

err_alloc_dh:
	if (dp->gc.gc_tx_max_descs_per_pkt > 0) {
		while (j-- > 0) {
			ddi_dma_free_handle(&dp->tx_buf[i].txb_dh[j]);
		}
		while (i-- > 0) {
			ddi_dma_unbind_handle(dp->tx_buf[i].txb_bdh);
			ddi_dma_mem_free(&dp->tx_buf[i].txb_bah);
			ddi_dma_free_handle(&dp->tx_buf[i].txb_bdh);
			j = GEM_MAXTXSEGS;
			while (j-- > 0) {
				ddi_dma_free_handle(&dp->tx_buf[i].txb_dh[j]);
			}
		}
	}
err:
	if (dp->desc_dma_handle) {
		err = ddi_dma_unbind_handle(dp->desc_dma_handle);
		ASSERT(err == DDI_SUCCESS);
		ddi_dma_mem_free(&dp->desc_acc_handle);
		ddi_dma_free_handle(&dp->desc_dma_handle);
	}

	return (ENOMEM);
}

static void
gem_free_memory(struct gem_dev* dp)
{
	int		i;
	int		j;
	int		err;
	struct rxbuf	*rbp;
	struct txbuf	*tbp;

	/* Free TX/RX descriptors and tx padding buffer */
	if (dp->desc_dma_handle != 0) {
		err = ddi_dma_unbind_handle(dp->desc_dma_handle);
		ASSERT(err == DDI_SUCCESS);

		ddi_dma_mem_free(&dp->desc_acc_handle);
		ddi_dma_free_handle(&dp->desc_dma_handle);
	}

	/* Free dma handles for Tx */
	if (dp->gc.gc_tx_max_frags > 0) {
		for (i = dp->gc.gc_tx_buf_size, tbp = dp->tx_buf; i--; tbp++) {
			for (j = 0; j < GEM_MAXTXSEGS; j++) {
				ddi_dma_free_handle(&tbp->txb_dh[j]);
			}
			/* Free bounce buffer associated to each txbuf */
			ddi_dma_unbind_handle(tbp->txb_bdh);
			ddi_dma_mem_free(&tbp->txb_bah);
			ddi_dma_free_handle(&tbp->txb_bdh);
		}
	}

	/* Free rx buffer */
	while (rbp = dp->rx_buf_freelist) {

		ASSERT(dp->rx_buf_freecnt > 0);

		dp->rx_buf_freelist = rbp->rxb_next;
		dp->rx_buf_freecnt--;

		/* release DMA mapping */
		ASSERT(rbp->rxb_dh != NULL);

		/* free dma handles for rx bbuf */
		if (rbp->rxb_nfrags > 0) {
			/* it still have a dma mapping */
			err = ddi_dma_unbind_handle(rbp->rxb_dh);
			ASSERT(err == DDI_SUCCESS);
		}

		if (rbp->rxb_mp != NULL) {
			freemsg(rbp->rxb_mp);
		}

		/* free the associated bounce buffer */
		ASSERT(rbp->rxb_bah != NULL);
		ddi_dma_mem_free(&rbp->rxb_bah);

		/* free the associated dma handle */
		ddi_dma_free_handle(&rbp->rxb_dh);
		ddi_dma_free_handle(&rbp->rxb_bdh);

		/* free the base memory of rx buffer management */
		kmem_free(rbp, sizeof(struct rxbuf));
	}
}

/* ============================================================== */
/*
 * Rx/Tx descriptor management
 */
/* ============================================================== */
static void
gem_init_rx_desc(struct gem_dev *dp)
{
	int	i;

	DPRINTF(2, (CE_CONT, "!%s: %s: ring_size:%d, buf_size:%d",
		dp->name, __func__,
		dp->gc.gc_rx_ring_size, dp->gc.gc_rx_buf_size));

	ASSERT(dp->gc.gc_rx_ring_size == 0 || dp->rx_ring != NULL);

	dp->rx_desc_head = (seqnum_t) 0;
	dp->rx_desc_tail = (seqnum_t) 0;

	/* make physical chain of rx descriptors */
	for (i = 0; i < dp->gc.gc_rx_ring_size; i++) {
		GEM_RX_DESC_INIT(dp, i);
	}

	ASSERT(dp->rx_buf_head == NULL);
	ASSERT(dp->rx_buf_tail == NULL);
}

static void
gem_clean_rx_ring(struct gem_dev* dp)
{
	int		i;
	struct rxbuf	*rbp;
	mblk_t		*mp;
	const int	rx_ring_size = dp->gc.gc_rx_ring_size;
#ifdef GEM_DEBUG_LEVEL
	int		total;
#endif
	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(!dp->nic_active);

	DPRINTF(2, (CE_CONT, "!%s: gem_clean_rx_ring: %d buffers are free",
		dp->name, dp->rx_buf_freecnt));

	for (i = 0; i < rx_ring_size; i++) {
		GEM_RX_DESC_CLEAN(dp, i);
	}

#ifdef GEM_DEBUG_LEVEL
	total = 0;
#endif
	while ((rbp = dp->rx_buf_head) != NULL) {
#ifdef GEM_DEBUG_LEVEL
		total++;
#endif
		if ((dp->rx_buf_head = rbp->rxb_next) == NULL) {
			dp->rx_buf_tail = NULL;
		}
		
		/* release DMA resource */
		ASSERT (rbp->rxb_nfrags > 0);
		ddi_dma_unbind_handle(rbp->rxb_dh);
		rbp->rxb_nfrags = 0;

		/* release mblk */
		ASSERT(rbp->rxb_mp != NULL);
		mp = rbp->rxb_mp;
		rbp->rxb_mp = NULL;
		freemsg(mp);

		gem_free_rxbuf(rbp);
	}
	dp->rx_buf_tail = NULL;

	DPRINTF(2, (CE_CONT, "!%s: %s: %d buffers freeed, total: %d free",
		dp->name, __func__, total, dp->rx_buf_freecnt));
}

static void
gem_init_tx_desc(struct gem_dev* dp)
{
	int		i;

	DPRINTF(2, (CE_CONT, "!%s: %s: ring_size:%d, buf_size:%d",
		dp->name, __func__,
		dp->gc.gc_tx_ring_size, dp->gc.gc_tx_buf_size));

	ASSERT(mutex_owned(&dp->xmitlock));

	/* reset positions in tx descriptor ring */
	dp->tx_desc_head = (seqnum_t) 0;
	dp->tx_desc_tail = (seqnum_t) 0;
	dp->tx_desc_intr = (seqnum_t) 0;
	dp->tx_desc_rsvd = (seqnum_t) 0;

	for (i = 0; i < dp->gc.gc_tx_ring_size; i++) {
		GEM_TX_DESC_INIT(dp, i);
	}

	/* keep softq */
	ASSERT(dp->tx_buf_head == dp->tx_softq_head);
}

static void
gem_clean_tx_ring(struct gem_dev *dp, boolean_t keep_buffers)
{
	int		i;
	struct txbuf	*tbp;

	ASSERT(mutex_owned(&dp->xmitlock));

	for (i = 0; i < dp->gc.gc_tx_ring_size; i++) {
		GEM_TX_DESC_CLEAN(dp, i);
	}

	if (keep_buffers) {
		/* dont free untransmitted tx packets */
		dp->tx_softq_head = dp->tx_buf_head;
		return;
	}
		
	while (tbp = dp->tx_buf_head) {

		/* dequeue one from tx list */
		if ((dp->tx_buf_head = tbp->txb_next) == NULL) {
			dp->tx_buf_tail = NULL;
		}

		if (tbp->txb_mp != NULL) {

			ASSERT(tbp->txb_dh_used > 0);

			i = tbp->txb_dh_used;
			while (i--) {
				/* release dma resource */
				ddi_dma_unbind_handle(tbp->txb_dh[i]);
			}

			/* free direct tx buffer */
			freemsg(tbp->txb_mp);
			tbp->txb_mp = NULL;
		}

		/* free descriptor */
		dp->tx_desc_head += tbp->txb_ndesc;
#ifdef SANITY
		tbp->txb_ndesc = 0;
#endif
		dp->stats.errxmt++;

		/* recycle txbuf */
		tbp->txb_next = dp->tx_buf_freelist;
		dp->tx_buf_freelist = tbp;
		dp->tx_buf_freecnt++;
	}

	ASSERT(dp->tx_buf_freecnt == dp->gc.gc_tx_buf_size);
	ASSERT(dp->tx_desc_head == dp->tx_desc_tail);
}

static void
gem_mac_init(struct gem_dev *dp)
{
	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->xmitlock));

	dp->nic_active  = FALSE;

	gem_init_rx_desc(dp);
	gem_init_tx_desc(dp);

	/* reset tx thread state */
	dp->tx_blocked  = FALSE;
	dp->tx_busy     = 0;

#ifdef GEM_CONFIG_POLLING
	dp->rx_pkt_cnt    = 0;
	dp->rx_intr_cnt   = 0;
	dp->poll_intr_cnt = 0;
	dp->tx_pkt_cnt    = 0;
	dp->poll_interval = 0;
#endif
	GEM_INIT_CHIP(dp);
}

static int
gem_mac_start(struct gem_dev *dp)
{
	int		i;
	struct rxbuf	*rbp;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(mutex_owned(&dp->xmitlock));
#ifdef GEM_DELAYED_START
	ASSERT(dp->nic_online);
#endif
	ASSERT(!dp->nic_active);

	/* prepare rx buffers */
	for (i = 0; i < dp->gc.gc_rx_buf_size; i++) {
		if ((rbp = gem_get_rxbuf(dp, TRUE)) == NULL) {
			break;
		}
		gem_append_rxbuf(dp, rbp, TRUE);
	}

	/* enable tx and rx */
	dp->nic_active = TRUE;
	GEM_START_CHIP(dp);

	/* issue preloaded tx buffers */
	gem_tx_start_unit(dp);
}

static int
gem_mac_stop(struct gem_dev *dp, boolean_t keep_buffers)
{
	int		i;
	int		wait_time;	/* in uS */
	clock_t		now;
	timeout_id_t	old_id;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, rx_buf_free:%d",
		dp->name, __func__, dp->rx_buf_freecnt));

	ASSERT(mutex_owned(&dp->xmitlock));

	dp->nic_active = FALSE;

	/* block transmits */
	while (dp->tx_busy > 0) {
		cv_wait(&dp->drain_cv, &dp->xmitlock);
	}

	/*
	 * Wait for all tx buffers sent.
	 */
	wait_time = 2 * (8 * MAXPKTBUF(dp) / gem_speed_value[dp->speed]) *
			(dp->tx_desc_tail - dp->tx_desc_head);

	DPRINTF(0, (CE_CONT, "%s: %s: max drain time: %d uS",
			dp->name, __func__, wait_time));
	i = 0;
	now = ddi_get_lbolt();
	while (dp->tx_buf_head != NULL) {
		if (i > wait_time) {
			/* timeout */
			cmn_err(CE_NOTE, "%s: %s timeout", dp->name, __func__);
			break;
		}
		(void) gem_reclaim_txbuf(dp);
		drv_usecwait(100);
		i += 100;
	}

	DPRINTF(0, (CE_NOTE,
		"!%s: %s: the nic have drained in %d uS, real %d mS",
		dp->name, __func__,
		i, 10*((int)(ddi_get_lbolt() - now))));

	/*
	 * Now we can stop the nic safely
	 */
	if (GEM_STOP_CHIP(dp) != GEM_SUCCESS) {
		cmn_err(CE_NOTE, "%s: %s: resetting the chip to stop it",
			dp->name, __func__);
		if (GEM_RESET_CHIP(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
			/* not implemented yet */
			;
#endif
		}
	}

	/* clear all rx buffers */
	if (keep_buffers) {
		mutex_exit(&dp->xmitlock);
		gem_receive(dp);
		mutex_enter(&dp->xmitlock);
	}
	gem_clean_rx_ring(dp);

	/* clear pended tx packets */
	gem_clean_tx_ring(dp, keep_buffers);

#ifdef GEM_DEBUG_LEVEL
	if (dp->rx_buf_allocated !=  dp->rx_buf_freecnt) {
		cmn_err(CE_CONT, "!%s: %s: rxbuf alloced:%d, free: %d\n",
			dp->name, __func__,
			dp->rx_buf_allocated, dp->rx_buf_freecnt);
	}
#endif
	return (GEM_SUCCESS);
}

void
gem_restart_nic(struct gem_dev *dp, boolean_t keep_tx_buf)
{
	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(mutex_owned(&dp->xmitlock));

	DPRINTF(1, (CE_CONT, "!%s: %s: called: tx_desc:%d %d %d",
		dp->name, __func__,
		dp->tx_desc_head, dp->tx_desc_tail, dp->tx_desc_intr));

	/* stop dma activity */
	(void) gem_mac_stop(dp, keep_tx_buf);

	/* Reset the chip. */
	if (GEM_RESET_CHIP(dp) != GEM_SUCCESS) {
		cmn_err(CE_WARN, "%s: %s: failed to reset chip",
			dp->name, __func__);
	}

	/* inititalize tx and rx rings */
	gem_mac_init(dp);

	/* set mac address and rx mode */
	GEM_SET_RX_FILTER(dp);

	/* restart the nic */
	gem_mac_start(dp);
}

/* ============================================================== */
/*
 * GLD interface
 */
/* ============================================================== */
static int
gem_gld_reset(gld_mac_info_t *macinfo)
{
	struct gem_dev *dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
#ifdef GEM_DELAYED_START
	dp->nic_online  = TRUE;
#endif
	gem_mac_init(dp);

	/* reset rx filter state */
	dp->mc_count     = 0;
	dp->mc_count_req = 0;

#ifdef GEM_DELAYED_START
	if (dp->mii_state == MII_STATE_LINKUP) {
		GEM_SET_MEDIA(dp);
	}
#endif
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_start(gld_mac_info_t *macinfo)
{
	struct gem_dev	*dp = (struct gem_dev *) macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
#ifdef GEM_DELAYED_START
	if (dp->mii_state == MII_STATE_LINKUP)
#endif
	{
		gem_mac_start(dp);
	}
	mutex_exit(&dp->xmitlock);

	dp->timeout_id = timeout((void (*)(void *))gem_tx_timeout,
				(void *)dp, dp->gc.gc_tx_timeout_interval);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_stop(gld_mac_info_t *macinfo)
{
	timeout_id_t	old_id;
	struct gem_dev	*dp = (struct gem_dev *) macinfo->gldm_private;

	/* stop tx timeout watcher */
	if (dp->timeout_id) {
		do {
			untimeout(old_id = dp->timeout_id);
		} while (dp->timeout_id != old_id);	
		dp->timeout_id = 0;
	}

	/* make the nic state inactive */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
#ifdef GEM_DELAYED_START
	dp->nic_online = FALSE;
#endif
	(void) gem_mac_stop(dp, FALSE);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_set_multicast(gld_mac_info_t *macinfo, uint8_t *ep, int flag)
{
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;
	size_t		len;
	int		i;
	int		cnt;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (flag == GLD_MULTI_ENABLE) {
		if (dp->mc_count_req++ < GEM_MAXMC) {
			/* append the new address at the end of the mclist */
			cnt = dp->mc_count;
			bcopy(ep, dp->mc_list[cnt].addr.ether_addr_octet,
				ETHERADDRL);
			if (dp->gc.gc_multicast_hash != NULL) {
				dp->mc_list[cnt].hash =
						GEM_MULTICAST_HASH(dp, ep);
			}
			dp->mc_count = cnt + 1;
		}
	}
	else {
		dp->mc_count_req--;
		cnt = dp->mc_count;
		for (i = 0; i < cnt; i++) {
			if (bcmp(ep, &dp->mc_list[i].addr, ETHERADDRL) != 0) {
				continue;
			}
			/* shrink the mclist by copying forward */
			len = (cnt - (i + 1)) * sizeof(*dp->mc_list);
			if (len > 0) {
				bcopy(&dp->mc_list[i+1], &dp->mc_list[i], len);
			}
			dp->mc_count--;
			break;
		}
	}
x:
	if (dp->mc_count_req != dp->mc_count) {
		/* multicast address list overflow */
		dp->rxmode |= RXMODE_MULTI_OVF;
	}
	else {
		dp->rxmode &= ~RXMODE_MULTI_OVF;
	}
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
	GEM_SET_RX_FILTER(dp);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_set_promiscuous(gld_mac_info_t *macinfo, int flag)
{
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (flag == GLD_MAC_PROMISC_NONE) {
		dp->rxmode &= ~(RXMODE_PROMISC | RXMODE_ALLMULTI_REQ);
	}
	else if (flag == GLD_MAC_PROMISC_MULTI) {
		dp->rxmode |= RXMODE_ALLMULTI_REQ;
	}
	else if (flag == GLD_MAC_PROMISC_PHYS) {
		dp->rxmode |= RXMODE_PROMISC;
	}
	else {
		/* mode unchanged */
		return (GLD_SUCCESS);
	}
x:
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
	GEM_SET_RX_FILTER(dp);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_set_mac_address(gld_mac_info_t *macinfo, uint8_t *mac)
{
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	bcopy(mac, dp->cur_addr.ether_addr_octet, ETHERADDRL);
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
	GEM_SET_RX_FILTER(dp);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_get_stats(gld_mac_info_t *macinfo, struct gld_stats *gs)
{
	struct gem_dev		*dp = (struct gem_dev *)macinfo->gldm_private;
	struct gem_stats	*vs;

	ASSERT(!mutex_owned(&dp->intrlock));

	GEM_GET_STATS_CHIP(dp);

	vs = &dp->stats;

	gs->glds_errxmt    = vs->errxmt;
	gs->glds_errrcv    = vs->errrcv;
	gs->glds_collisions= vs->collisions;

	gs->glds_excoll    = vs->excoll;
	gs->glds_defer     = vs->defer;
	gs->glds_frame     = vs->frame;
	gs->glds_crc       = vs->crc;

	gs->glds_overflow  = vs->overflow; /* fifo err,underrun,rbufovf*/
	gs->glds_underflow = vs->underflow;
	gs->glds_short     = vs->runt;
	gs->glds_missed    = vs->missed; /* missed pkts while rbuf ovf */
	gs->glds_xmtlatecoll = vs->xmtlatecoll;
	gs->glds_nocarrier = vs->nocarrier;
	gs->glds_norcvbuf  = vs->norcvbuf;	/* OS resource exaust */
	gs->glds_intr      = vs->intr;

	/* all before here must be kept in place for v0 compatibility */
	gs->glds_speed     = gem_speed_value[dp->speed] * 1000000;
	gs->glds_media     = GLDM_PHYMII;
	gs->glds_duplex    = dp->full_duplex
				? GLD_DUPLEX_FULL : GLD_DUPLEX_HALF;

	/* gs->glds_media_specific */
	gs->glds_dot3_first_coll     = vs->first_coll;
	gs->glds_dot3_multi_coll     = vs->multi_coll;
	gs->glds_dot3_sqe_error	     = 0;
	gs->glds_dot3_mac_xmt_error  = 0;
	gs->glds_dot3_mac_rcv_error  = 0;
	gs->glds_dot3_frame_too_long = vs->frame_too_long;

	return (GLD_SUCCESS);
}

/*==========================================================*/
/*
 * Start transmission.
 * Return zero on success,
 */
/*==========================================================*/
int
gem_reclaim_txbuf(struct gem_dev *dp)
{
	struct txbuf		*tbp;
	int			i;
	uint_t			txstat;
	int			err;
#if GEM_DEBUG_LEVEL > 4
	clock_t			now = ddi_get_lbolt();
#endif
	ASSERT(mutex_owned(&dp->xmitlock));

	err = GEM_SUCCESS;
	while ((tbp = dp->tx_buf_head) != NULL) {

		if (tbp == dp->tx_softq_head) {
			break;
		}

		txstat = GEM_TX_DESC_STAT(dp, tbp->txb_slot, tbp->txb_ndesc);

		if (txstat == 0) {
			/* not transmitted yet */
			break;
		}

		if ((txstat & GEM_TX_ERR) != 0) {
			err = GEM_FAILURE;
		}
#if GEM_DEBUG_LEVEL > 4
		if (now - tbp->txb_stime >= 50) {
			cmn_err(CE_WARN, "!%s: tx delay while %d mS",
			dp->name, (now - tbp->txb_stime)*10);
		}
#endif
		ASSERT((txstat & ~(GEM_TX_DONE | GEM_TX_ERR)) == 0);

		/* Dequeue the tx buffer from the tx queue */
		if ((dp->tx_buf_head = tbp->txb_next) == NULL) {
			dp->tx_buf_tail = NULL;
		}
		dp->tx_desc_head += tbp->txb_ndesc;

		/* Update the next interrupted position */
		if (!TXINTR_SCHEDULED(dp)) {
			dp->tx_desc_intr = dp->tx_desc_head;
		}
#ifdef SANITY
		tbp->txb_ndesc = 0;
#endif
		/* Free dma mapping resources of each fragment */
		if (tbp->txb_mp) {
			i = tbp->txb_dh_used;
			while (i--) {
				ddi_dma_unbind_handle(tbp->txb_dh[i]);
			}
			tbp->txb_nfrags  = 0;
#ifdef SANITY
			tbp->txb_dh_used = 0;
#endif
			/* Free the original mblk. */
			freemsg(tbp->txb_mp);
#ifdef SANITY
			tbp->txb_mp = NULL;
#endif
		}

		/* Recycle the txbuf */
		tbp->txb_next = dp->tx_buf_freelist;
		dp->tx_buf_freelist = tbp;
		dp->tx_buf_freecnt++;
	}

	return (err);
}

static void
gem_tx_start_unit(struct gem_dev *dp)
{
	struct txbuf	*tbp;
	uint32_t	intreq;
	seqnum_t	curr;
	int		dfree;
	const int	tx_ring_size = dp->gc.gc_tx_ring_size;
	const int	descs = dp->gc.gc_tx_max_descs_per_pkt;

	ASSERT(mutex_owned(&dp->xmitlock));

	for (tbp = dp->tx_softq_head; tbp != NULL;
			dp->tx_softq_head = tbp = tbp->txb_next) {

		dfree = tx_ring_size - SUB(dp->tx_desc_tail, dp->tx_desc_head)
			- dp->tx_desc_rsvd;
		if (dfree < descs) {
			break;
		}

		/* allocate hw descriptor and link it to software structure */
		tbp->txb_slot = SLOT(dp->tx_desc_tail, tx_ring_size);
		curr = dp->tx_desc_tail;

		/*
		 * Guarantee to have a chance to be interrupted before
		 * tx thread will be blocked.
		 */
		intreq = 0;
		if ((!TXINTR_SCHEDULED(dp) &&
			(dfree < 2*descs
			/*
			 * Schedule a tx-done interrupt before free tx
			 * descriptors will run out. To ensure we can issue
			 * a tx packet on the next time, we needs at least
			 * 2*desc here.
			 * This is the last chance to schedule tx interrupt.
			 */
			 || dp->tx_buf_freecnt < 1))
			/*
			 * When a tx interrupt have not been scheduled and the
			 * number of free tx buffers is 0, we'll be blocked for
			 * ever on the next tx request.
			 * This is the last chance to schedule tx interrupt.
			 */

		 || (tx_ring_size + dp->tx_desc_intr) - dp->tx_desc_tail < descs
			/*
			 * We also need at least 1*desc to continue
			 * transmitting after we will get a tx interrupt.
			 */
		) {

			intreq = GEM_TXFLAG_INTR;
		}
#if GEM_DEBUG_LEVEL > 1
		if (dp->tx_cnt < 100) {
			dp->tx_cnt++;
			intreq = GEM_TXFLAG_INTR;
		}
#endif
		/*
		 * Write tx descriptor(s)
		 */
		dp->tx_desc_tail += tbp->txb_ndesc =
		GEM_TX_DESC_WRITE(dp, tbp->txb_slot, tbp->txb_dmacookie,
			tbp->txb_nfrags,
			intreq | tbp->txb_flag);
		tbp->txb_stime = dp->tx_start_time = ddi_get_lbolt();

		/* update the next interrupted position */
		if (intreq) {
			dp->tx_desc_intr = dp->tx_desc_tail;
		}
#ifdef GEM_CONFIG_POLLING
		dp->tx_pkt_cnt++;
#endif
	}
#if GEM_DEBUG_LEVEL > 3
	if (dp->tx_softq_head != NULL) {
		int		i;
		struct txbuf *tbp;

		cmn_err(CE_CONT, "!%s: %s: hw descriptor full",
			dp->name, __func__);
		i = 0;
		for (tbp = dp->tx_buf_head; tbp; tbp = tbp->txb_next) {
			cmn_err(CE_CONT, "!%d: tbp:%p slot:%d ndesc:%d %s",
				i, tbp, tbp->txb_slot, tbp->txb_ndesc,
				(tbp==dp->tx_softq_head ? "(softq_head)": ""));
			i++;
		}
	}
#endif
#if GEM_DEBUG_LEVEL > 0
	ASSERT(dp->tx_desc_rsvd > 0 || dp->tx_softq_head == NULL || TXINTR_SCHEDULED(dp));
#endif
}

#ifdef TXTIMEOUT_TEST
static int gem_send_cnt = 0;
#endif

/*
 * gem_send_common is external function because hw depend routines can
 * use it for sending control frames like setup frame for 2114x chips.
 */
#define	TXBUF_LOWAT(dp)		2
#define	TXDESC_LOWAT(dp)	(3*(dp)->gc.gc_tx_max_descs_per_pkt)

#define	GLD_SUCCESS_COPIED	(-2)

int
gem_send_common(struct gem_dev *dp, mblk_t *mp, uint32_t vtag, uint32_t flags)
{
	mblk_t			*tp;
	ddi_dma_cookie_t	*dcp;
	ddi_dma_cookie_t	*dma;
	int			i;
	int			dh_used;
	size_t			len;
	int			err;
	struct txbuf		*tbp;
	boolean_t		copy;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	uint16_t		ether_proto;
	uint16_t		ip_proto;
#endif

	copy = (flags & (GEM_SEND_COPY | GEM_SEND_CTRL)) != 0;

	DPRINTF(3, (CE_CONT, "!%s: called", __func__));

	ASSERT(mutex_owned(&dp->xmitlock));
	
	DPRINTF(3, (CE_CONT, "!%s: free txbufs:%d",
		__func__, dp->tx_buf_freecnt));

	if (dp->tx_buf_freecnt == 0) {
		ASSERT(dp->tx_buf_freelist == NULL);
		dp->tx_blocked = TRUE;
		DPRINTF(4, (CE_CONT, "!%s: %s: tx blocked",
			dp->name, __func__));
#ifdef GEM_DEBUG_LEVEL
		dp->stats.nocarrier++;
#endif
		return (GLD_NORESOURCES);
	}

	/*
	 * Reserve required resources
	 */
	/* dequeue the tx buffer from the free list */
	ASSERT(dp->tx_buf_freelist != NULL);
	tbp = dp->tx_buf_freelist;
	dp->tx_buf_freelist = tbp->txb_next;
	dp->tx_buf_freecnt--;

	dp->tx_blocked = FALSE;
	dp->tx_busy++;

	mutex_exit(&dp->xmitlock);
copy:
	dcp = tbp->txb_dmacookie;
	dh_used = 0;

	if (copy) {
		caddr_t	bp;

		/*
		 * copy case
		 */
		bp = tbp->txb_buf;
#ifdef GEM_CONFIG_VLAN
		if ((dp->misc_flag & GEM_VLAN_SOFT) != 0 &&
		    (vtag & VLAN_VID_MASK) != 0) {
			/*
			 * Underlying hardware doesn't capability to
			 * insert vlan tci and tag.
			 * So we insert it manually after copying the packet
			 * into the bounce buffer.
			 */
			bp += VTAG_SIZE;
		}
#endif
		for (tp = mp; tp; tp = tp->b_cont) {
			size_t	nbyte;
			nbyte = tp->b_wptr - tp->b_rptr;
			bcopy(tp->b_rptr, bp, nbyte);
			bp += nbyte;
		}

		len = bp - tbp->txb_buf;

		if (len < ETHERMIN && !dp->gc.gc_tx_auto_pad) {
			bzero(bp, ETHERMIN - len);
			len = ETHERMIN;
		}

#ifdef GEM_CONFIG_VLAN
		if ((dp->misc_flag & GEM_VLAN_SOFT) != 0 &&
		    (vtag & VLAN_VID_MASK) != 0) {
			/*
			 * Insert vlan tag
			 */
			bp = tbp->txb_buf;
			bcopy(bp + VTAG_SIZE, bp, ETHERADDRL*2);

			/*
			 * XXX - we must use byte operations to avoid
			 * segmentation violations
			 */
			bp[ETHERADDRL*2 + 0] = vtag >> 24;
			bp[ETHERADDRL*2 + 1] = vtag >> 16;
			bp[ETHERADDRL*2 + 2] = vtag >> 8;
			bp[ETHERADDRL*2 + 3] = vtag;
#   ifdef GEM_CONFIG_CKSUM_OFFLOAD
			/* we must check packet type before inserting vlan tag*/
			ether_proto = GET_ETHERTYPE(tbp->txb_buf + VTAG_SIZE);
			ip_proto    = GET_IPTYPE(tbp->txb_buf + VTAG_SIZE);
#   endif
		}
		else
#endif
		{
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
			/* we must check packet type before inserting vlan tag*/
			ether_proto = GET_ETHERTYPE(tbp->txb_buf);
			ip_proto    = GET_IPTYPE(tbp->txb_buf);
#endif
		}
		ddi_dma_sync(tbp->txb_bdh, (off_t)0, len, DDI_DMA_SYNC_FORDEV);

		ASSERT((tbp->txb_buf_dma & dp->gc.gc_tx_buf_align) == 0);
		dcp->dmac_laddress = tbp->txb_buf_dma;
		dcp->dmac_size    = len;
		DPRINTF(2, (CE_CONT, "!%s: copy: addr:0x%llx len:0x%x",
			__func__, dcp->dmac_laddress, dcp->dmac_size));
		dcp++;

		/* prepare to destroy mp */
		mp  = NULL;
		err = GLD_SUCCESS_COPIED;
	}
	else {
		/*
		 * prepare dma resources for direct transmittion.
		 */
		int	frags;

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
		ether_proto = GET_ETHERTYPE(mp->b_rptr);
		ip_proto    = GET_IPTYPE(mp->b_rptr);
#endif
		frags = 0;
		for (tp = mp; tp; tp = tp->b_cont) {
			ddi_dma_handle_t	dma_handle;
			uint_t			count;
			size_t			len;

			len = tp->b_wptr - tp->b_rptr;
			dma_handle = tbp->txb_dh[dh_used++];

			if (frags + 1 > dp->gc.gc_tx_max_frags) {
				/* no room for saving entire fragment list */
				goto err_dma_range;
			}

			if ((err = ddi_dma_addr_bind_handle(
				dma_handle, NULL,
				(caddr_t)tp->b_rptr, len,
				DDI_DMA_WRITE | DDI_DMA_PARTIAL |
					((len >= 512) ? DDI_DMA_STREAMING : 0),
				DDI_DMA_DONTWAIT, NULL,
				dcp++, &count))!= DDI_DMA_MAPPED) {

				DPRINTF(0, (CE_CONT,
				"!%s: %s: dma_bind error: %d",
					dp->name, __func__, err));

				/* failed to bind dma resource to the mblk */
				dh_used--;

				goto err_dma_range;
			}

			frags += count;
			if (frags > dp->gc.gc_tx_max_frags) {
				/* no room for saving entire fragment list */
				goto err_dma_range;
			}

			/* collect dma fragment list */
			while (--count > 0) {
				ddi_dma_nextcookie(dma_handle, dcp++);
			}
		}

		DPRINTF(2, (CE_CONT,
			"!%s: %s: mp:0x%p, tbp:0x%p, dma frags:%d",
			dp->name, __func__, mp, tbp, dcp-tbp->txb_dmacookie));

		/*
		 * XXX - check if the dma range is valid because of solaris bug.
		 * ddi_dma_addr_bind_handle() returns successfully even if
		 * the physical address range of mblk isn't suitable for the dma
		 * capability of the nic.
		 */
		for (i = 0, dma = tbp->txb_dmacookie; i < frags; i++, dma++) {

			DPRINTF(2, (CE_CONT,
				"! frag:%dth: dma_addr:0x%llx len:0x%x",
				i, dma->dmac_laddress, dma->dmac_size));

			if (!(dma->dmac_laddress >=
				dp->gc.gc_dma_attr_txbuf.dma_attr_addr_lo &&
			   (uint64_t)(dma->dmac_laddress+(dma->dmac_size-1)) <=
				dp->gc.gc_dma_attr_txbuf.dma_attr_addr_hi)) {

				goto err_dma_range;
			}
		}
		err = GLD_SUCCESS;
		goto dma_map_ok;

err_dma_range:
		/*
		 * This will happen when the physcal
		 * address range of the installed memory exceeds
		 * the dma addressing capability of the nic.
		 * From the view point of performance, we
		 * should switch to use the copy method
		 * always.
		 */
		DPRINTF(0, (CE_NOTE,
"!%s: %s: switch to tx copy mode, frags:%d, dh_used:%d, b_rptr:%p len:%lld",
			dp->name, __func__, frags, dh_used,
			mp->b_rptr, mp->b_wptr - mp->b_rptr));
		dp->gc.gc_tx_copy_thresh = MAXPKTBUF(dp);

		/* free partially allocated dma resources */
		while (dh_used--) {
			ddi_dma_unbind_handle(tbp->txb_dh[dh_used]);
		}

		/* force to copy */
		copy = TRUE;
		goto copy;
	}
dma_map_ok:
	/* save misc info */
	tbp->txb_mp      = mp;
	tbp->txb_dh_used = dh_used;
	tbp->txb_nfrags  = dcp - tbp->txb_dmacookie;
	tbp->txb_ndesc   = 0;
	tbp->txb_flag    = (flags & GEM_SEND_CTRL) << GEM_TXFLAG_PRIVATE_SHIFT;

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if (ether_proto == ETHERTYPE_IP) {
		tbp->txb_flag |= GEM_TXFLAG_IPv4;
		if (ip_proto == IPPROTO_TCP) {
			tbp->txb_flag |= GEM_TXFLAG_TCP;
		}
		if (ip_proto == IPPROTO_UDP) {
			tbp->txb_flag |= GEM_TXFLAG_UDP;
		}
	}
#endif

	mutex_enter(&dp->xmitlock);
	dp->tx_busy--;

	if (!dp->nic_active && (flags & GEM_SEND_CTRL)== 0) {
		/* device status has changed while we were preparing tx buf */
		if (dp->tx_busy == 0) {
			/*
			 * we are last one that make tx_busy.
			 * wake up someone who may wait for us.
			 */
			cv_broadcast(&dp->drain_cv);
		}
#ifdef SOLARIS10
		err = GLD_NOLINK;
#else
		err = GLD_FAILURE;
#endif
		return (err);
	}

	/* Append tbp at the tail of the active tx buffer list */
	tbp->txb_next = NULL;
	if (dp->tx_buf_head == NULL) {
		dp->tx_buf_head = tbp;
		dp->tx_buf_tail = tbp;
	} else {
		dp->tx_buf_tail->txb_next = tbp;
		dp->tx_buf_tail = tbp;
	}
	if (dp->tx_softq_head == NULL) {
		dp->tx_softq_head = tbp;
	}

	if (dp->tx_busy == 0) {
		/*
		 * load tx buffer(s) into hardware descriptors.
		 */
		gem_tx_start_unit(dp);
	}

	return (err);
}

/*
 * gem_gld_send is used only for sending data packets into ethernet wire.
 */
static int
gem_gld_send_tagged(gld_mac_info_t *macinfo, mblk_t *mp, uint32_t vtag)
{
	struct gem_dev	*dp;
	mblk_t		*tp;
	uint8_t		*bp;
	int		ret;
	int		segs;
#ifdef i86pc
	int		frags;
#endif
	long		align;
	size_t		len;
	boolean_t	copy;

	dp = (struct gem_dev *)macinfo->gldm_private;

	copy = FALSE;

#ifdef GEM_CONFIG_VLAN
	if ((dp->misc_flag & GEM_VLAN_SOFT) != 0 &&
	    (vtag & VLAN_VID_MASK) != 0) {
		copy = TRUE;
	}
#endif
	/*
	 * check the packet length and alignment of each fragment of
	 * the packet
	 */
	align = 0;
	len   = 0;
	segs  = 0;
#ifdef i86pc
	frags = 0;
#endif
	for (tp = mp; tp != NULL; tp = tp->b_cont) {
		int	diff; 

		len  +=  (diff = tp->b_wptr - tp->b_rptr);
		if (diff < 8) {
			/*
			 * the fragment is too short.
			 */
			copy = TRUE;
		}

		align |= (long)tp->b_rptr;
		if (tp->b_cont != NULL) {
			align |= (long)tp->b_wptr;
		}
		segs++;
#ifdef i86pc
		frags += ((PAGEOFFSET & (long) tp->b_rptr) + diff + PAGEOFFSET)
				>> PAGESHIFT;
#endif
	}

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	/*
	 * first fragment must be bigger than ether header + ip header
	 * + tcp header
	 */
	if (segs > 1) {
		if (mp->b_wptr - mp->b_rptr <
				sizeof(struct ether_header) + 120) {
			copy = TRUE;
		}
	}
#endif
	if (len > dp->mtu + sizeof(struct ether_header)) {
		cmn_err(CE_CONT, "!%s: %s: msg too big: %d",
			dp->name, __func__, len);
		/* Do not free mp here, GLD will free it */
		return (GLD_NOTSUPPORTED);
	}

	if ((align & dp->gc.gc_tx_buf_align) != 0 ||
	    segs > GEM_MAXTXSEGS ||
#ifdef i86pc
	    frags > dp->gc.gc_tx_max_frags ||
#else
	    segs > dp->gc.gc_tx_max_frags ||
#endif
	    len <= dp->gc.gc_tx_copy_thresh) {
		copy = TRUE;
	}

	/*
	 * Some nics hate to send packets while the link is down.
	 */
	mutex_enter(&dp->xmitlock);

	if (!dp->nic_active  || dp->mii_state != MII_STATE_LINKUP) {
		mutex_exit(&dp->xmitlock);
#ifdef SOLARIS10
		return (GLD_NOLINK);
#else
		return (GLD_FAILURE);
#endif
	}

	ret = gem_send_common(dp, mp, vtag, copy ? GEM_SEND_COPY : 0);

	mutex_exit(&dp->xmitlock);

	if (ret == GLD_SUCCESS_COPIED) {
		freemsg(mp);
		ret = GLD_SUCCESS;
	}

	return (ret);
}

static int
gem_gld_send(gld_mac_info_t *macinfo, mblk_t *mp)
{
	return (gem_gld_send_tagged(macinfo, mp, 0));
}

static void
gem_tx_timeout(struct gem_dev *dp)
{
	clock_t		now;
	boolean_t	tx_sched;
	struct txbuf	*tbp;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock); 

	tx_sched = FALSE;
	now = ddi_get_lbolt();

	if (!dp->nic_active || dp->mii_state != MII_STATE_LINKUP) {
		goto schedule_next;
	}

	/* reclaim transmitted buffers to check the trasmitter hangs or not. */
	if (gem_reclaim_txbuf(dp) != GEM_SUCCESS) {
		/* tx error happended, reset transmitter in the chip */
		gem_restart_nic(dp, TRUE);
		tx_sched = dp->tx_blocked;
		goto schedule_next;
	}

	/* check if the transmitter is stuck */
	if (dp->tx_buf_head == dp->tx_softq_head) {
		/* no tx buffer is loaded to the nic */
		goto schedule_next;
	}
	if (now - dp->tx_buf_head->txb_stime < dp->gc.gc_tx_timeout) {
		goto schedule_next;
	}

	cmn_err(CE_WARN,
"!%s: tx timeout: tx_desc_head: %d[%d] %d[%d], softq:%x, starttime:%d, %d",
		dp->name,
		dp->tx_desc_head, SLOT(dp->tx_desc_head,dp->gc.gc_tx_ring_size),
		dp->tx_desc_tail, SLOT(dp->tx_desc_tail,dp->gc.gc_tx_ring_size),
		dp->tx_softq_head,
		dp->tx_buf_head->txb_stime - now,
		dp->tx_start_time - now);

	/* discard untransmitted packet and restart tx.  */
	gem_restart_nic(dp, FALSE);
	tx_sched = dp->tx_blocked;

#ifdef TXTIMEOUT_TEST
	gem_send_cnt = 0;
#endif

schedule_next:
	mutex_exit(&dp->xmitlock); 
	mutex_exit(&dp->intrlock);

	/* restart the downstream if needed */
	if (tx_sched) {
		gld_sched(dp->macinfo);
	}

	DPRINTF(4, (CE_CONT,
	"!%s: blocked:%d txbuf_cnt:%d desc_head:%d desc_tail:%d desc_intr:%d",
		dp->name, dp->tx_blocked, dp->tx_buf_freecnt,
		dp->tx_desc_head, dp->tx_desc_tail, dp->tx_desc_intr));
	dp->timeout_id =
		timeout((void (*)(void *))gem_tx_timeout,
			(void *)dp, dp->gc.gc_tx_timeout_interval);
}

/*==================================================================*/
/*
 * Interrupt handler
 */
/*==================================================================*/
static void
gem_append_rxbuf(struct gem_dev *dp, struct rxbuf *rbp, int cansleep)
{
	int		err;
	mblk_t		*mp;
	const int	rx_ring_size = dp->gc.gc_rx_ring_size;
	const int	rx_max_frags = dp->gc.gc_rx_max_frags;


	ASSERT(rbp != NULL);
	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(3, (CE_CONT,
		"!%s: gem_append_rxbuf: desc_head:%d, desc_tail:%d",
		dp->name, dp->rx_desc_head, dp->rx_desc_tail));

	if (rx_ring_size - SUB(dp->rx_desc_tail, dp->rx_desc_head)
			< dp->gc.gc_rx_max_descs_per_pkt) {
		/*
		 * As the number of free rx descriptors is too short,
		 * we may fail to append the rx buffer to the rx list.
		 */

		/* release allocated DMA resources */
		ASSERT(rbp->rxb_nfrags > 0);
		ddi_dma_unbind_handle(rbp->rxb_dh);
		rbp->rxb_nfrags = 0;

		ASSERT(rbp->rxb_mp != NULL);
		mp = rbp->rxb_mp;
		rbp->rxb_mp = NULL;
		freemsg(mp);

		gem_free_rxbuf(rbp);

		return;
	}

	/*
	 * Add it into active rx buffer list
	 */	
	rbp->rxb_slot = SLOT(dp->rx_desc_tail, rx_ring_size);
	rbp->rxb_next = NULL;
	if (dp->rx_buf_head == NULL) {
		dp->rx_buf_head = rbp;
		dp->rx_buf_tail = rbp;
	}
	else {
		dp->rx_buf_tail->rxb_next = rbp;
		dp->rx_buf_tail = rbp;
	}

	dp->rx_desc_tail += rbp->rxb_ndesc =
		GEM_RX_DESC_WRITE(dp, rbp->rxb_slot, rbp->rxb_dmacookie,
				rbp->rxb_nfrags);

	return;
}

mblk_t *
gem_get_packet_default(struct gem_dev *dp, struct rxbuf *rbp, size_t len)
{
	mblk_t	*mp;

	/* allocate a new small mblk */
	if ((mp = allocb(len, BPRI_MED)) != NULL) {
		mp->b_wptr = mp->b_rptr + len;
		ddi_dma_sync(rbp->rxb_dh, dp->gc.gc_rx_header_len,
				len, DDI_DMA_SYNC_FORKERNEL);
		bcopy(rbp->rxb_buf + dp->gc.gc_rx_header_len,
				mp->b_rptr, len);
	}
	return (mp);
}

#ifdef GEM_DEBUG_LEVEL
uint_t	gem_rx_pkts[17];
#endif

int
gem_receive(struct gem_dev *dp)
{
	int		len;
	struct rxbuf	*rbp;
	struct rxbuf	*nrbp;
	int		limit;
	mblk_t		*mp;
	uint64_t	rxstat;
	int		cnt;
#ifdef GEM_CONFIG_VLAN
	uint32_t	vtag;
#endif
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	uint32_t	flags;
#endif

	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(3, (CE_CONT, "!%s: gem_receive: rx_buf_head:%p",
			dp->name, dp->rx_buf_head));

	limit = max(dp->gc.gc_rx_buf_size, 100);
	for (cnt = 0; (rbp = dp->rx_buf_head) != NULL && cnt < limit; cnt++) {

		if (((rxstat = GEM_RX_DESC_STAT(dp,
				rbp->rxb_slot, rbp->rxb_ndesc))
					& (GEM_RX_DONE | GEM_RX_ERR)) == 0) {
			/* not received yet */
			break;
		}

		/* Remove the head of the rx buffer list */
		if ((dp->rx_buf_head = rbp->rxb_next) == NULL) {
			dp->rx_buf_tail = NULL;
		}
		dp->rx_desc_head += rbp->rxb_ndesc;

		/* first of all, invalidate pointer to new rx buffer */
		nrbp = NULL;

		if ((rxstat & GEM_RX_ERR) != 0) {
			nrbp = rbp;
			goto next;
		}

		len = (int16_t)(rxstat & GEM_RX_LEN);
#ifdef GEM_CONFIG_VLAN
		vtag = (rxstat & GEM_RX_VTAG) >> GEM_RX_VTAG_SHIFT;
#endif
		DPRINTF(2, (CE_CONT, "!%s: %s: rxstat:0x%x, len:0x%x",
				dp->name, __func__, rxstat, len));

		if (len > MAXPKTLEN(dp)) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
			nrbp = rbp;
			goto next;
		}

		if (len < ETHERMIN) {
			dp->stats.errrcv++;
			dp->stats.runt++;
			nrbp = rbp;
			goto next;
		}

		if (len > dp->gc.gc_rx_copy_thresh) {
			/*
			 * try to allocate a new rx buffer
			 */
			nrbp = gem_get_rxbuf(dp, FALSE);
			if (nrbp == NULL) {
				/* no rx buffer */
				dp->stats.norcvbuf++;
			}
		}

		if (nrbp == NULL) {
			/*
			 * Copy the packet
			 */
			/* reuse this rbp */
			nrbp = rbp;
			/* allocate a new small mblk */
			mp = (*dp->gc.gc_get_packet)(dp, rbp, len);
			if (mp == NULL) {
				/* no memory, discard the packet */
				dp->stats.norcvbuf++;
				goto next;
			}
		}
		else {
			/*
			 * Direct reception
			 */
			ASSERT(rbp->rxb_nfrags > 0);

			ddi_dma_unbind_handle(rbp->rxb_dh);
			rbp->rxb_nfrags = 0;

			/* remove the associated mblk from the rbp */
			mp = rbp->rxb_mp;
			rbp->rxb_mp = NULL;
			mp->b_wptr = mp->b_rptr + len;

			if ((rbp->rxb_flags & RXB_FLAGS_BOUNCE) != 0) {
				/*
				 * copy the received packet from
				 * the bounce buffer
				 */
				bcopy(rbp->rxb_buf + dp->gc.gc_rx_header_len,
						mp->b_rptr, len);
			}

			gem_free_rxbuf(rbp);
		}

		mutex_exit(&dp->intrlock);
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
		flags = 0;
		if ((rxstat & (GEM_RX_CKSUM_TCP | GEM_RX_CKSUM_UDP)) != 0) {
			flags |= HCK_FULLCKSUM;
		}
		if ((rxstat & GEM_RX_CKSUM_IPv4) != 0) {
			flags |= HCK_IPV4_HDRCKSUM;
		}
		flags |= HCK_FULLCKSUM_OK;
		flags = 0;
		if (flags != 0) {
			hcksum_assoc(mp, NULL, NULL, 0, 0, 0, 0, flags, 0);
		}
#endif
#ifdef GEM_CONFIG_VLAN
		if ((dp->misc_flag & GEM_VLAN_SOFT) != 0 &&
		     GET_ETHERTYPE(mp->b_rptr) == VLAN_TPID) {
			/*
			 * We got vlan tagged packet, extract the tag id.
			 */
			vtag = GET_ETHERTYPE(mp->b_rptr + 2);
			bcopy(mp->b_rptr, mp->b_rptr + VTAG_SIZE, ETHERADDRL*2);
			mp->b_rptr += VTAG_SIZE;
		}
		gld_recv_tagged(dp->macinfo, mp,
			(vtag != 0) ? GLD_MAKE_VTAG(vtag) : VLAN_VTAG_NONE);
#else
		gld_recv(dp->macinfo, mp);
#endif
		mutex_enter(&dp->intrlock);
next:
		ASSERT(nrbp != NULL);
		gem_append_rxbuf(dp, nrbp, FALSE);
	}
#ifdef GEM_CONFIG_POLLING
	/* update rx statistics for polling */
	dp->rx_pkt_cnt += cnt;
	if (cnt > 0) {
		/* count number of valid rx interrupts */
		dp->rx_intr_cnt++;
	}
#endif

#ifdef GEM_DEBUG_LEVEL
	gem_rx_pkts[min(cnt, sizeof(gem_rx_pkts)/sizeof(uint_t)-1)]++;
#endif
	return (cnt);
}

boolean_t
gem_tx_done(struct gem_dev *dp)
{
	boolean_t	tx_sched = FALSE;

	mutex_enter(&dp->xmitlock);

	if (gem_reclaim_txbuf(dp) != GEM_SUCCESS) {
		gem_restart_nic(dp, TRUE);
		DPRINTF(2, (CE_CONT, "!%s: gem_tx_done: tx_desc: %d %d",
			dp->name, dp->tx_desc_head, dp->tx_desc_tail));
	}
	else if (dp->tx_softq_head != NULL) {
		gem_tx_start_unit(dp);
	}

	DPRINTF(3, (CE_CONT, "!%s: gem_tx_done: ret: blocked:%d",
		dp->name, dp->tx_blocked));

	tx_sched = dp->tx_blocked;

	mutex_exit(&dp->xmitlock);

	return (tx_sched);
}

static u_int
gem_gld_intr(gld_mac_info_t *macinfo)
{
	struct gem_dev	*dp;
	u_int		ret;
#ifdef GEM_CONFIG_POLLING
	clock_t		now;
#endif
	dp = (struct gem_dev *)(macinfo->gldm_private);

	mutex_enter(&dp->intrlock);
	dp->intr_busy = TRUE;

	ret = GEM_INTERRUPT(dp);

	if (ret == DDI_INTR_UNCLAIMED) {
		dp->intr_busy = FALSE;
		mutex_exit(&dp->intrlock);
		return (ret);
	}

	if (!dp->nic_active) {
		cv_broadcast(&dp->drain_cv);
	}

#ifdef GEM_CONFIG_POLLING
{
	int	pkts;

	now = ddi_get_lbolt()/drv_usectohz(10*1000);
	if (now != dp->last_intr_time) {
		/*
		 * It's time to check tx and rx statistics
		 */
		if (dp->poll_pkt_delay > 0) {
			int     speed;
			/*
			 * Polling mode support
			 */
			speed = gem_speed_value[dp->speed] * (1000 * 1000 / 8);

			/* maximum packets in 10 mS */
			dp->poll_pkt_hiwat =
				speed / (dp->mtu * 100 * dp->poll_pkt_delay);
		}
		else {
			dp->poll_pkt_hiwat = INT32_MAX;
		}

		mutex_enter(&dp->xmitlock);

		pkts = max(dp->tx_pkt_cnt, dp->rx_pkt_cnt);

		if (now == dp->last_intr_time + 1 &&
		    pkts > dp->poll_pkt_hiwat) {
			/*
			 * calculate recommended polling interval
			 */
			dp->poll_interval = 10000 / dp->poll_pkt_hiwat;
								/* in 1uS */
		}
		else {
			/* normal mode */
			dp->poll_interval = 0;
		}

		/* reset tx and rx packet counter */
		dp->rx_pkt_cnt  = 0;
		dp->rx_intr_cnt = 0;
		dp->tx_pkt_cnt  = 0;
		dp->poll_intr_cnt = 0;
		mutex_exit(&dp->xmitlock);
	}
	dp->last_intr_time = now;
	dp->poll_intr_cnt++;
}
#endif
	dp->stats.intr++;
	dp->intr_busy = FALSE;

	mutex_exit(&dp->intrlock);

	if ((ret & INTR_RESTART_TX) != 0) {
		DPRINTF(4, (CE_CONT, "!%s: calling gld_sched", dp->name));
		gld_sched(macinfo);
	}

	return (ret & ~INTR_RESTART_TX);
}

static void
gem_intr_watcher(gld_mac_info_t *macinfo)
{
	struct gem_dev	*dp;

	dp = (struct gem_dev *)macinfo->gldm_private;

	(macinfo->gldm_intr)(macinfo);
next:
	/* schedule next call of tu_intr_watcher */
	dp->intr_watcher_id =
		timeout((void (*)(void *))gem_intr_watcher, (void *)macinfo, 1);
}

/* ======================================================================== */
/*
 * MII support routines
 */
/* ======================================================================== */
static uint16_t
gem_mii_read(struct gem_dev *dp, uint_t reg)
{
	if (!dp->no_preamble) {
		GEM_MII_SYNC(dp);
	}
	return (GEM_MII_READ(dp, reg));
}

static void
gem_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	if (!dp->no_preamble) {
		GEM_MII_SYNC(dp);
	}
	GEM_MII_WRITE(dp, reg, val);
}

#define	fc_cap_decode(x)	\
	((((x) & MII_ABILITY_PAUSE) != 0 ? 1 : 0) |	\
	 (((x) & MII_ABILITY_ASM_DIR) != 0 ? 2 : 0))

int
gem_mii_config_default(struct gem_dev *dp)
{
	uint16_t	status;
	uint16_t	xstatus;
	uint16_t	val;
	static uint16_t fc_cap_encode[4] = {
		/* none */		0,
		/* symmetric */		MII_ABILITY_PAUSE,
		/* tx */		MII_ABILITY_ASM_DIR,
		/* rx-symmetric */	MII_ABILITY_PAUSE | MII_ABILITY_ASM_DIR,
	};

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * Set advertisement register
	 */
	status = gem_mii_read(dp, MII_STATUS);
	DPRINTF(1, (CE_CONT, "!%s: %s: MII_STATUS reg:%b",
		dp->name, __func__, status, MII_STATUS_BITS));

	if ((status & MII_STATUS_ABILITY) == 0) {
		/* it's funny */
		cmn_err(CE_WARN, "!%s: wrong ability bits: status:%b",
			dp->name, status, MII_STATUS_BITS);
		return (GEM_FAILURE);
	}

	/* Do not change bits except ability bits in advert reg */
	val = gem_mii_read(dp, MII_AN_ADVERT) & ~MII_ABILITY;

	DPRINTF(1, (CE_CONT,
		"!%s: %s: 100T4:%d 100F:%d 100H:%d 10F:%d 10H:%d",
		dp->name, __func__,
		dp->anadv_100t4, dp->anadv_100fdx, dp->anadv_100hdx,
		dp->anadv_10fdx, dp->anadv_10hdx));

	if ((status & MII_STATUS_100_BASE_T4) != 0 && dp->anadv_100t4) {
		val |= MII_ABILITY_100BASE_T4;
	}
	if ((status & MII_STATUS_100_BASEX_FD) != 0 && dp->anadv_100fdx) {
		val |= MII_ABILITY_100BASE_TX_FD;
	}
	if ((status & MII_STATUS_100_BASEX) != 0 && dp->anadv_100hdx) {
		val |= MII_ABILITY_100BASE_TX;
	}
	if ((status & MII_STATUS_10_FD) != 0 && dp->anadv_10fdx) {
		val |= MII_ABILITY_10BASE_T_FD;
	}
	if ((status & MII_STATUS_10) != 0 && dp->anadv_10hdx) {
		val |= MII_ABILITY_10BASE_T;
	}

	/* set flow control capability */
	val |= fc_cap_encode[dp->gc.gc_flow_control];

	DPRINTF(1, (CE_CONT,
		"!%s: %s: setting MII_AN_ADVERT reg:%b, mii_mode:%d, fc:%d",
		dp->name, __func__, val, MII_ABILITY_BITS, dp->gc.gc_mii_mode,
		dp->gc.gc_flow_control));

	gem_mii_write(dp, MII_AN_ADVERT, val);

	if ((status & MII_STATUS_XSTATUS) != 0) {
		/*
		 * 1000Base-T GMII support
		 */
		val = gem_mii_read(dp, MII_1000TC) &
				~(MII_1000TC_ADV_FULL | MII_1000TC_ADV_HALF);
		xstatus = gem_mii_read(dp, MII_XSTATUS);
		if ((xstatus & MII_XSTATUS_1000BASET_FD) != 0 &&
		     dp->anadv_1000fdx) {
			val |= MII_1000TC_ADV_FULL;
		}
		if ((xstatus & MII_XSTATUS_1000BASET) != 0 &&
		     dp->anadv_1000hdx) {
			val |= MII_1000TC_ADV_HALF;
		}

		DPRINTF(1, (CE_CONT,
			"!%s: %s: setting MII_1000TC reg:%b",
			dp->name, __func__, val, MII_1000TC_BITS));

		gem_mii_write(dp, MII_1000TC, val);
	}

	return (GEM_SUCCESS);
}

clock_t
gem_mii_link_check(struct gem_dev *dp)
{
	uint16_t	status;
	uint16_t	advert;
	uint16_t	lpable;
	uint16_t	exp;
#ifdef GEM_GIGA
	uint16_t	ctl1000;
	uint16_t	stat1000;
#endif
	uint16_t	val;
	clock_t		now;
	clock_t		diff;
	int		linkdown_action;

	now = ddi_get_lbolt();
	diff = now - dp->mii_last_check;
	dp->mii_last_check = now;

	DPRINTF(3, (CE_CONT, "!%s: mii_link_check: time:%d state:%d",
				dp->name, now, dp->mii_state));

next_nowait:
	switch (dp->mii_state) {
	case MII_STATE_UNKNOWN:
		goto reset_phy;

	case MII_STATE_RESETTING:
		dp->mii_timer -= diff;
		if (dp->mii_timer > 0) {
			/* wait for time-up */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/*
		 * Timer expired, ensure reset bit is not set, or the phy
		 * was enabled
		 */
		GEM_MII_SYNC(dp);
		if ((gem_mii_read(dp, MII_CONTROL)
			 & (MII_CONTROL_RESET | MII_CONTROL_ISOLATE |
			    MII_CONTROL_PWRDN)) != 0) {
			/* As the phy is still reset, or disabled, enable it */
			gem_mii_write(dp, MII_CONTROL, 0);
		}

		/* Configure phy registers */
		if (GEM_MII_CONFIG(dp) != 0) {
			goto reset_phy;
		}

		if (dp->mii_fixedmode) {
			dp->mii_state = MII_STATE_MEDIA_SETUP;
			dp->mii_timer = 0;
			dp->mii_interval = 0;
			goto next_nowait;
		}

		/* Issue auto-negotiation command */
		goto autonego;

	case MII_STATE_AUTONEGOTIATING:
		/*
		 * Autonegotiation in progress
		 */
		dp->mii_timer -= diff;
		if (dp->mii_timer -
		      (dp->gc.gc_mii_an_timeout - dp->gc.gc_mii_an_wait) > 0) {
			/* wait for minimum time (2.3 - 2.5 sec) for rhine */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/* read PHY status */
		status = gem_mii_read(dp, MII_STATUS);
		DPRINTF(4, (CE_CONT,
			"!%s: gem_mii_link_check: called: "
			"mii_state:%d MII_STATUS reg:%b",
			dp->name, dp->mii_state,
			status, MII_STATUS_BITS));

		if ((status & MII_STATUS_REMFAULT) != 0) {
			/*
			 * The link parnert told me something wrong happend.
			 * What do we do ?
			 */
			cmn_err(CE_CONT,
				"!%s: auto-negotiation failed: remote fault",
				dp->name);
			goto autonego;
		}

		if ((status & MII_STATUS_ANDONE) == 0) {
			if (dp->mii_timer <= 0) {
				/*
				 * Auto-negotiation timed out,
				 * Reset PHY and try again.
				 */
				if (!dp->mii_supress_msg) {
					cmn_err(CE_WARN,
					"!%s: auto-negotiation failed: timeout",
					dp->name);
					dp->mii_supress_msg = TRUE;
				}
#ifdef notdef
				/* stop auto negotiation */
				val = gem_mii_read(dp, MII_CONTROL);
				gem_mii_write(dp, MII_CONTROL,
				   val & ~(MII_CONTROL_RSAN | MII_CONTROL_ANE));
#endif
				goto autonego;
			}
			/*
			 * Auto-negotiation is in progress. Wait.
			 */
			dp->mii_interval = dp->gc.gc_mii_an_watch_interval;
			goto next;
		}

		/*
		 * Auto-negotiation have done.
		 * Assume linkdown and fall through.
		 */
		DPRINTF(2, (CE_CONT, "!%s: auto-negotiation done", dp->name));
		dp->mii_supress_msg = FALSE;
		dp->mii_state = MII_STATE_AN_DONE;
		dp->mii_timer = 0;
		diff = 0;
		goto next_nowait;

	case MII_STATE_AN_DONE:
		/*
		 * Auto-negotiation have done.
		 * Now we can set up media.
		 */
		dp->mii_timer -= diff;
		if (dp->mii_timer > 0) {
			/*
			 * wait for a while
			 */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/*
		 * set up the result of auto negotiation 
		 */

		/* determine full/half and 1000Mbps/100Mbps/10Mbps */
		advert = gem_mii_read(dp, MII_AN_ADVERT);
		lpable = gem_mii_read(dp, MII_AN_LPABLE);
		exp    = gem_mii_read(dp, MII_AN_EXPANSION);

		cmn_err(CE_CONT,
		"!%s: auto-negotiation done, advert:%b, lpable:%b, exp:%b",
			dp->name,
			advert, MII_ABILITY_BITS,
			lpable, MII_ABILITY_BITS,
			exp, MII_AN_EXP_BITS);
		/*
		 * configure link mode according to AN priority.
		 */	
		val = advert & lpable;
#ifdef GEM_GIGA
		ctl1000  = 0;
		stat1000 = 0;
		if (dp->gc.gc_mii_mode == GEM_MODE_1000BASET) {
			ctl1000  = gem_mii_read(dp, MII_1000TC);
			stat1000 = gem_mii_read(dp, MII_1000TS);
			DPRINTF(0, (CE_CONT,
				"!%s: MII_1000TC reg:%b, MII_1000TS reg:%b",
				dp->name,
				ctl1000, MII_1000TC_BITS,
				stat1000, MII_1000TS_BITS));
		}

		if ((ctl1000 & MII_1000TC_ADV_FULL) != 0 &&
		    (stat1000 & MII_1000TS_LP_FULL) != 0) {
			/* 1000BaseT & full duplex */
			dp->speed	 = GEM_SPD_1000;
			dp->full_duplex  = TRUE;
		}
		else if ((ctl1000 & MII_1000TC_ADV_HALF) != 0 &&
			 (stat1000 & MII_1000TS_LP_HALF) != 0) {
			/* 1000BaseT & half duplex */
			dp->speed	 = GEM_SPD_1000;
			dp->full_duplex  = FALSE;
		}
		else 
#endif /* GEM_GIGA */
		if ((val & MII_ABILITY_100BASE_TX_FD) != 0) {
			/* 100BaseTx & fullduplex */
			dp->speed	 = GEM_SPD_100;
			dp->full_duplex  = TRUE;
		}
		else if ((val & MII_ABILITY_100BASE_TX) != 0) {
			/* 100BaseTx & half duplex */
			dp->speed	 = GEM_SPD_100;
			dp->full_duplex  = FALSE;
		}
		else if ((val & MII_ABILITY_10BASE_T_FD) != 0) {
			/* 10BaseT & full duplex */
			dp->speed	 = GEM_SPD_10;
			dp->full_duplex  = TRUE;
		}
		else if ((val & MII_ABILITY_10BASE_T) != 0) {
			/* 10BaseT & half duplex */
			dp->speed	 = GEM_SPD_10;
			dp->full_duplex  = FALSE;
		}
		else {
			/*
			 * It seem that the link partnar don't have
			 * auto-negotiation capability and our PHY
			 * could not report the correct current mode.
			 * This case happened on VT86C100A chip so
			 * that we guess current mode by mii_control
			 * register.
			 */

			val = gem_mii_read(dp, MII_CONTROL);

			cmn_err(CE_NOTE,
				"!%s: auto-negotiation done but "
				"common ability not found.\n"
				"PHY state: control:%b advert:%b lpable:%b\n"
				"guessing the following mode...",
				dp->name,
				val, MII_CONTROL_BITS,
				advert, MII_ABILITY_BITS,
				lpable, MII_ABILITY_BITS);

			/* select 100m full or 10m half */
			dp->speed = ((val & MII_CONTROL_100MB) != 0)
						? GEM_SPD_100 : GEM_SPD_10;
			dp->full_duplex = dp->speed != GEM_SPD_10;
		}

		if (dp->full_duplex) {
			static uint8_t fc_result[4/*my cap*/][4/*lp cap*/] = {
			/*	 none	symm	tx	rx/symm */
			/* none */
				{FLOW_CONTROL_NONE,
					FLOW_CONTROL_NONE,
						FLOW_CONTROL_NONE,
							FLOW_CONTROL_NONE},
			/* sym */
				{FLOW_CONTROL_NONE,
					FLOW_CONTROL_SYMMETRIC,
						FLOW_CONTROL_NONE,
							FLOW_CONTROL_SYMMETRIC},
			/* tx */
				{FLOW_CONTROL_NONE,
					FLOW_CONTROL_NONE,
						FLOW_CONTROL_NONE,
							FLOW_CONTROL_TX_PAUSE},
			/* rx/symm */
				{FLOW_CONTROL_NONE,
					FLOW_CONTROL_SYMMETRIC,
						FLOW_CONTROL_RX_PAUSE,
							FLOW_CONTROL_SYMMETRIC},
			};
			dp->flow_control =
			fc_result[fc_cap_decode(advert)][fc_cap_decode(lpable)];
		}
		else {
			dp->flow_control = FLOW_CONTROL_NONE;
		}
		dp->mii_state = MII_STATE_MEDIA_SETUP;

		if (dp->gc.gc_mii_an_delay > 0) {
			/*
			 * Some nics which detect media mode automatically
			 * take a little time to know the mode.
			 */
			dp->mii_timer = dp->gc.gc_mii_an_delay;
			dp->mii_interval = drv_usectohz(20*1000);
			goto next;
		}
		goto next_nowait;

	case MII_STATE_MEDIA_SETUP:
		if (dp->mii_fixedmode || dp->gc.gc_mii_an_oneshot) {
			/*
			 * write the result of auto negotiation back.
			 * XXX - This code didn't work on AN983B.
			 */
			val = gem_mii_read(dp, MII_CONTROL);
			val &= ~(MII_CONTROL_SPEED | MII_CONTROL_FDUPLEX |
				 MII_CONTROL_ANE   | MII_CONTROL_RSAN);

			if (dp->full_duplex) {
				val |= MII_CONTROL_FDUPLEX;
			}

			switch (dp->speed) {
			case GEM_SPD_1000:
				val |= MII_CONTROL_1000MB;
				break;

			case GEM_SPD_100:
				val |= MII_CONTROL_100MB;
				break;

			default:
				/* for GEM_SPD_10, do nothing */
				break;
			}

			gem_mii_write(dp, MII_CONTROL, val);
		}
#ifdef GEM_DELAYED_START
		if (dp->nic_online)
#endif
		{
			mutex_enter(&dp->xmitlock);
			GEM_SET_MEDIA(dp);
			mutex_exit(&dp->xmitlock);
		}

		if ((void *)(dp->gc.gc_mii_tune_phy) != NULL) {
			/* for built-in sis900 */
			GEM_MII_TUNE_PHY(dp);
		}

		dp->mii_state = MII_STATE_LINKDOWN;
		dp->mii_timer = drv_usectohz(1*1000*1000);

		DPRINTF(2, (CE_CONT, "!%s: setup midia mode done", dp->name));
		dp->mii_supress_msg = FALSE;

		/* use short interval */
		dp->mii_interval = WATCH_INTERVAL_FAST;
		goto next_nowait;

	case MII_STATE_LINKDOWN:
		status = gem_mii_read(dp, MII_STATUS);
		if ((status & MII_STATUS_LINKUP) != 0) {
			static char *fc_type[] = {
				"without",
				"with symmetric",
				"with tx",
				"with rx",
			};
			/*
			 * Link going up
			 */
			dp->mii_state = MII_STATE_LINKUP;
			dp->mii_supress_msg = FALSE;

			DPRINTF(0, (CE_CONT,
			"!%s: link up detected: mii_control:%b, status:%b",
				dp->name,
				gem_mii_read(dp, MII_CONTROL), MII_CONTROL_BITS,
				status, MII_STATUS_BITS));

			/* restart tx watchdog */
			dp->tx_start_time = ddi_get_lbolt();

			/*
			 * MII_CONTROL_100MB and  MII_CONTROL_FDUPLEX are
			 * ignored when MII_CONTROL_ANE is set.
			 */
			cmn_err(CE_CONT,
			    "!%s: Link up: %d Mbps %s duplex %s flow control",
				dp->name,
				gem_speed_value[dp->speed],
				dp->full_duplex ? "full" : "half",
				fc_type[dp->flow_control]);

			dp->mii_interval = dp->gc.gc_mii_link_watch_interval;
#ifdef GEM_DELAYED_START
			if (dp->nic_online && !dp->nic_active) {
				/*
				 * as it is first link-up after online,
				 * we should start tx and rx here.
				 */
				mutex_enter(&dp->xmitlock);
				gem_mac_start(dp);
				mutex_exit(&dp->xmitlock);
			}
#endif /* GEM_DELAYED_START */
#ifdef SOLARIS10
			gld_linkstate(dp->macinfo, GLD_LINKSTATE_UP);
#endif
			goto next;
		}

		dp->mii_supress_msg = TRUE;
		if (!dp->mii_fixedmode) {
			dp->mii_timer -= diff;
			if (dp->mii_timer <= 0) {
				/*
				 * link down timer expired.
				 * need to restart auto-negotiation.
				 */
				linkdown_action =
					dp->gc.gc_mii_linkdown_timeout_action;
				goto restart_autonego;
			}
		}
		/* don't change mii_state */
		break;

	case MII_STATE_LINKUP:
		status = gem_mii_read(dp, MII_STATUS);
		if ((status & MII_STATUS_LINKUP) == 0) {
			/*
			 * Link going down
			 */
			cmn_err(CE_NOTE,
				"!%s: link down detected: status:%b",
				dp->name, status, MII_STATUS_BITS);
#ifdef SOLARIS10
			gld_linkstate(dp->macinfo, GLD_LINKSTATE_DOWN);
#endif
			if (!dp->mii_fixedmode) {
				/* need to restart auto-negotiation */
				linkdown_action = dp->gc.gc_mii_linkdown_action;
				goto restart_autonego;
			}

			dp->mii_state = MII_STATE_LINKDOWN;
			dp->mii_timer = dp->gc.gc_mii_linkdown_timeout;

			if ((void *)(dp->gc.gc_mii_tune_phy) != NULL) {
				/* for built-in sis900 */
				GEM_MII_TUNE_PHY(dp);
			}
		}
		/* don't change mii_state */
		break;
	}
	dp->mii_interval = dp->gc.gc_mii_link_watch_interval;
	goto next;

	/* Actions on the end of state routine */

restart_autonego:
	switch (linkdown_action) {
	case MII_ACTION_RESET:
		if (!dp->mii_supress_msg) {
			cmn_err(CE_CONT, "!%s: resetting PHY", dp->name);
		}
		dp->mii_supress_msg = TRUE;
		goto reset_phy;

	case MII_ACTION_NONE:
		dp->mii_supress_msg = TRUE;
		if (dp->gc.gc_mii_an_oneshot) {
			goto autonego;
		}
		/* PHY will restart autonego automatically */
		dp->mii_state = MII_STATE_AUTONEGOTIATING;
		dp->mii_timer = dp->gc.gc_mii_an_timeout;
		dp->mii_interval = dp->gc.gc_mii_an_watch_interval;
		goto next;

	case MII_ACTION_RSA:
		if (!dp->mii_supress_msg) {
			cmn_err(CE_CONT, "!%s: restarting auto-negotiation",
				dp->name);
		}
		dp->mii_supress_msg = TRUE;
		goto autonego;

	default:
		cmn_err(CE_WARN, "!%s: unknowm linkdown action: %d",
			dp->name, dp->gc.gc_mii_linkdown_action);
		dp->mii_supress_msg = TRUE;
	}
	/* NOTREACHED */

reset_phy:
#ifdef SOLARIS10
	gld_linkstate(dp->macinfo, GLD_LINKSTATE_DOWN);
#endif
	if (!dp->mii_supress_msg) {
		cmn_err(CE_CONT, "!%s: resetting PHY", dp->name);
	}
	dp->mii_state = MII_STATE_RESETTING;
	dp->mii_timer = dp->gc.gc_mii_reset_timeout;
	if (!dp->gc.gc_mii_dont_reset) {
		gem_mii_write(dp, MII_CONTROL, MII_CONTROL_RESET);
	}
	dp->mii_interval = WATCH_INTERVAL_FAST;
	goto next;

autonego:
#ifdef SOLARIS10
	gld_linkstate(dp->macinfo, GLD_LINKSTATE_DOWN);
#endif
	if (!dp->mii_supress_msg) {
		cmn_err(CE_CONT, "!%s: auto-negotiation started", dp->name);
	}
	dp->mii_state = MII_STATE_AUTONEGOTIATING;
	dp->mii_timer = dp->gc.gc_mii_an_timeout;

	/* start/restart auto nego */
	val = gem_mii_read(dp, MII_CONTROL);
	gem_mii_write(dp, MII_CONTROL,
			val | MII_CONTROL_ANE | MII_CONTROL_RSAN);

	dp->mii_interval = dp->gc.gc_mii_an_watch_interval;

next:
	return (dp->mii_interval);
}

static void
gem_mii_link_watcher(struct gem_dev *dp)
{
	clock_t		next;
	int		old_mii_state;
	boolean_t	tx_sched;


	mutex_enter(&dp->intrlock);
	old_mii_state = dp->mii_state;
	next = gem_mii_link_check(dp);
	tx_sched = (old_mii_state != MII_STATE_LINKUP) &&
		   (dp->mii_state == MII_STATE_LINKUP) && dp->tx_blocked;
	mutex_exit(&dp->intrlock);

	if (tx_sched) {
		/* kick potentially stopped downstream */
		gld_sched(dp->macinfo);
	}

	if (next != (clock_t)0) {
		dp->link_watcher_id =
			timeout((void (*)(void *))& gem_mii_link_watcher,
				(void *)dp, next);
	}

	return;
}

int
gem_mii_init_default(struct gem_dev *dp)
{
	int		phy;
	uint16_t	status;
	uint16_t	Xstatus;

	DPRINTF(3, (CE_CONT, "!%s: gem_mii_init: called", dp->name));

	/*
	 * Scan PHY
	 */
	dp->no_preamble = FALSE;
	GEM_MII_SYNC(dp);

	/* Try default phy first */
	if (dp->mii_phy_addr != 0) {
		status = gem_mii_read(dp, MII_STATUS);
		if (status != 0xffff && status != 0x0000) {
			/* ensure the phy found is enabled */
			gem_mii_write(dp, MII_CONTROL, 0);
			goto PHY_found;
		}

		DPRINTF(0, (CE_NOTE, "!%s: %s: writing 0 to mii control",
				dp->name, __func__));

		if (dp->mii_phy_addr < 0) {
			cmn_err(CE_NOTE,
			"!%s: failed to probe default internal/non-MII PHY",
				dp->name);
			return (GEM_FAILURE);
		}

		cmn_err(CE_NOTE,
			"!%s: failed to probe default MII PHY at %d",
			dp->name, dp->mii_phy_addr);
	}

	/* Try all possible address */
	for (phy = dp->gc.gc_mii_addr_min; phy < 32; phy++) {
		dp->mii_phy_addr = phy;
		status = gem_mii_read(dp, MII_STATUS);

		if (status != 0xffff && status != 0x0000) {
			/* ensure the phy found is enabled */
			gem_mii_write(dp, MII_CONTROL, 0);
			goto PHY_found;
		}
	}

	for (phy = dp->gc.gc_mii_addr_min; phy < 32; phy++) {
		dp->mii_phy_addr = phy;
		/* try to enable the phy */
		gem_mii_write(dp, MII_CONTROL, 0);
		status = gem_mii_read(dp, MII_STATUS);

		if (status != 0xffff && status != 0x0000) {
			goto PHY_found;
		}
	}

	cmn_err(CE_NOTE, "!%s: no MII PHY found", dp->name);
	dp->mii_phy_addr = -1;

	return (GEM_FAILURE);

PHY_found:
	dp->mii_phy_id  = gem_mii_read(dp, MII_PHYIDH) << 16;
	dp->mii_phy_id |= gem_mii_read(dp, MII_PHYIDL);

	dp->no_preamble = (status & MII_STATUS_MFPRMBLSUPR) != 0;

	if (dp->mii_phy_addr < 0) {
		cmn_err(CE_CONT, "!%s: using internal/non-MII PHY(0x%08x)",
			dp->name, dp->mii_phy_id);
	} else {
		cmn_err(CE_CONT, "!%s: MII PHY (0x%08x) found at %d",
			dp->name, dp->mii_phy_id, dp->mii_phy_addr);
	}

	cmn_err(CE_CONT, "!%s: PHY control:%b, status:%b, advert:%b, lpar:%b",
		dp->name,
		gem_mii_read(dp, MII_CONTROL), MII_CONTROL_BITS,
		status, MII_STATUS_BITS,
		gem_mii_read(dp, MII_AN_ADVERT), MII_ABILITY_BITS,
		gem_mii_read(dp, MII_AN_LPABLE), MII_ABILITY_BITS);
#ifdef GEM_GIGA
	if (dp->gc.gc_mii_mode == GEM_MODE_1000BASET) {
		cmn_err(CE_CONT, "!%s: xstatus:%b",
			dp->name,
			gem_mii_read(dp, MII_XSTATUS), MII_XSTATUS_BITS);
	}
#endif
	if (!dp->mii_fixedmode && (status & MII_STATUS_CANAUTONEG) == 0) {
		dp->mii_fixedmode = TRUE;
		/* fix speed and duplex mode. check half duplex first */
		if ((status & MII_STATUS_10) != 0) {
			dp->speed	= GEM_SPD_10;
			dp->full_duplex = FALSE;
		}
		else if ((status & MII_STATUS_100_BASEX) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = FALSE;
		}
		else if ((status & MII_STATUS_10_FD) != 0) {
			dp->speed	= GEM_SPD_10;
			dp->full_duplex = TRUE;
		}
		else if ((status & MII_STATUS_100_BASEX_FD) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = TRUE;
		}
		else if ((status & MII_STATUS_100_BASE_T4) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = FALSE;
		}
	}

	return (GEM_SUCCESS);
}

static void
gem_mii_start(struct gem_dev *dp)
{
	/*
	 * Synchronize MII state.
	 * on resume case, mii_init won't be called.
	 */
	GEM_MII_SYNC(dp);

	/* reset PHY */
	dp->mii_state = MII_STATE_UNKNOWN;
	gem_mii_link_watcher(dp);
}

static void
gem_mii_stop(struct gem_dev *dp)
{
	timeout_id_t	old_id;

	/* Ensure timer routine stopped */
	if (dp->link_watcher_id != 0) {
		do {
			untimeout(old_id = dp->link_watcher_id);
		} while (old_id != dp->link_watcher_id);
		dp->link_watcher_id = 0;
	}
#ifdef NEVER
	/*
	 * XXX - Don't powedown phy, AMD NetPHY cannot be recovered from
	 * powerdown mode, if it once become to power-down.
	 */
	/* Disable the phy */
	gem_mii_write(dp, MII_CONTROL,
			MII_CONTROL_ISOLATE | MII_CONTROL_PWRDN);
#endif
}

void
gem_generate_macaddr(struct gem_dev *dp, uint8_t *mac)
{
	int		i;
	uint_t		val;

	cmn_err(CE_CONT,
	"!%s: using temporal ether address, do not use this for long time",
		dp->name);
	val = ddi_get_lbolt() ^ ddi_get_time();

	/* generate a local address */
	mac[0] = 0x02;
	mac[1] = 0x00;
	mac[2] = val >> 24;
	mac[3] = val >> 16;
	mac[4] = val >> 8;
	mac[5] = val;
}

boolean_t
gem_get_mac_addr_conf(struct gem_dev *dp)
{
	char		propname[32];
	char		*valstr;
	uint8_t		mac[ETHERADDRL];
	char		*cp;
	int		c;
	int		i;
	int		j;
	int		v;
	int		d;
	int		ored;

	/*
	 * Get ethernet address from .conf file
	 */
	sprintf(propname, "%s-mac-addr", dp->name);
	if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
			DDI_PROP_DONTPASS, propname, &valstr
			)) != DDI_PROP_SUCCESS) {
#ifdef notdef
		cmn_err(CE_CONT,
			"!%s: trying to read %s property from .conf: undefined",
			dp->name, propname);
#endif
		return (FALSE);
	}

	if (strlen(valstr) != ETHERADDRL*3-1) {
		goto syntax_err;
	}

	cp = valstr;
	j  = 0;
	ored = 0;
	while (1) {
		v = 0;
		for (i = 0; i < 2; i++) {
			c = *cp++;

			if (c >= 'a' && c <= 'f') {
				d = c - 'a' + 10;
			}
			else if (c >= 'A' && c <= 'F') {
				d = c - 'A' + 10;
			}
			else if (c >= '0' && c <= '9') {
				d = c - '0';
			}
			else {
				goto syntax_err;
			}
			v = (v << 4) | d;
		}

		mac[j++] = v;
		ored |= v;
		if (j == ETHERADDRL) {
			/* done */
			break;
		}

		c = *cp++;
		if (c != ':') {
			goto syntax_err;
		}
	}

	if (ored == 0) {
		gem_generate_macaddr(dp, mac);
	}

	for (i = 0; i < ETHERADDRL; i++) {
		dp->dev_addr.ether_addr_octet[i] = mac[i];
	}
	ddi_prop_free(valstr);
	return (TRUE);

syntax_err:
	cmn_err(CE_CONT,
		"!%s: read mac addr: trying .conf: syntax err %s",
		dp->name, valstr);
	ddi_prop_free(valstr);
	return (FALSE);
}

/* ======================================================================== */
/*
 * attach/detatch support
 */
/* ======================================================================== */
static void
gem_read_conf(struct gem_dev *dp)
{
	char			propname[32];
	char			*valstr;
	int			val;

	/*
	 * Get media mode infomation from .conf file
	 */

	sprintf(propname, "%s-duplex", dp->name);
	if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
			DDI_PROP_DONTPASS, propname, &valstr
			)) == DDI_PROP_SUCCESS) {
		dp->mii_fixedmode = TRUE;
		if (strcmp(valstr, "full") == 0) {
			dp->full_duplex = TRUE;
		}
		else if (strcmp(valstr, "half") == 0) {
			dp->full_duplex = FALSE;
		}
		else {
			cmn_err(CE_WARN,
				"!%s: property %s: illegal value (%s)",
				dp->name, propname, valstr);
			dp->mii_fixedmode = FALSE;
		}
		ddi_prop_free(valstr);
	}

	if ((val = gem_prop_get_int(dp, "%s-speed", 0)) > 0) {
		dp->mii_fixedmode = TRUE;
		switch (val) {
		case 1000:
			dp->speed = GEM_SPD_1000;
			break;
		case 100:
			dp->speed = GEM_SPD_100;
			break;
		case 10:
			dp->speed = GEM_SPD_10;
			break;
		default:
			cmn_err(CE_WARN,
				"!%s: property %s: illegal value:%d",
				dp->name, propname, val);
			dp->mii_fixedmode = FALSE;
			break;
		}
	}

	val = gem_prop_get_int(dp, "%s-flow-control", dp->gc.gc_flow_control);
	if (val > FLOW_CONTROL_RX_PAUSE || val < FLOW_CONTROL_NONE) {
		cmn_err(CE_WARN,
			"!%s: property %s: illegal value:%d",
			dp->name, propname, val);
	}
	else {
		dp->gc.gc_flow_control = val;
	}

	dp->anadv_1000fdx = gem_prop_get_int(dp, "%s-anadv-1000fdx", 1) != 0;
	dp->anadv_1000hdx = gem_prop_get_int(dp, "%s-anadv-1000hdx", 1) != 0;
	dp->anadv_100t4   = gem_prop_get_int(dp, "%s-anadv-100t4", 1) != 0;
	dp->anadv_100fdx  = gem_prop_get_int(dp, "%s-anadv-100fdx", 1) != 0;
	dp->anadv_100hdx  = gem_prop_get_int(dp, "%s-anadv-100hdx", 1) != 0;
	dp->anadv_10fdx   = gem_prop_get_int(dp, "%s-anadv-10fdx", 1) != 0;
	dp->anadv_10hdx   = gem_prop_get_int(dp, "%s-anadv-10hdx", 1) != 0;

	if (gem_prop_get_int(dp, "%s-nointr", 0) != 0) {
		dp->misc_flag |= GEM_NOINTR;
		cmn_err(CE_NOTE, "!%s: polling mode enabled", dp->name);
	}

	dp->mtu = gem_prop_get_int(dp, "%s-mtu", dp->mtu);
	dp->txthr = gem_prop_get_int(dp, "%s-txthr", dp->txthr);
	dp->rxthr = gem_prop_get_int(dp, "%s-rxthr", dp->rxthr);
	dp->txmaxdma = gem_prop_get_int(dp, "%s-txmaxdma", dp->txmaxdma);
	dp->rxmaxdma = gem_prop_get_int(dp, "%s-rxmaxdma", dp->rxmaxdma);
	dp->poll_pkt_delay =
		gem_prop_get_int(dp, "%s-pkt-delay", dp->poll_pkt_delay);
}

#define	GEM_LOCAL_DATA_SIZE(gc)	\
	(sizeof(struct gem_dev) + GEM_MCALLOC + \
	sizeof(struct txbuf)*((gc)->gc_tx_buf_size))

struct gem_dev *
gem_do_attach(dev_info_t *dip,
	struct gem_conf *gc, void *base, ddi_acc_handle_t *regs_handlep,
	void *lp, int lmsize)
{
	struct gem_dev		*dp;
	int			i;
	ddi_iblock_cookie_t	c;
	gld_mac_info_t		*macinfo;
	int			ret;
	int			unit;

	unit = ddi_get_instance(dip);

	DPRINTF(2, (CE_CONT, "!gem%d: gem_do_attach: called cmd:ATTACH",
		unit));

	macinfo = gld_mac_alloc(dip);
	if (macinfo == NULL) {
		return (NULL);
	}

	/*
	 * Allocate soft data structure
	 */
	dp = (struct gem_dev *)
		kmem_zalloc(GEM_LOCAL_DATA_SIZE(gc), KM_SLEEP);
	if (dp == NULL) {
		gld_mac_free(macinfo);
		return (NULL);
	}
	/* link to private area */
	dp->private   = lp;
	dp->priv_size = lmsize;

	dp->mc_count  = 0;
	dp->mc_count_req = 0;
	dp->mc_list = (struct mcast_addr *) &dp[1];

	dp->dip = dip;
	dp->macinfo = macinfo;
	bcopy(gc->gc_name, dp->name, GEM_NAME_LEN);

	/*
	 * Get iblock cookie
	 */
	if (ddi_get_iblock_cookie(dip, 0, &c) != DDI_SUCCESS) {
		cmn_err(CE_CONT,
			"!%s: gem_do_attach: ddi_get_iblock_cookie: failed",
			dp->name);
		goto err_free_private;
	}
	dp->iblock_cookie = c;

	/*
	 * Initialize mutex's for this device.
	 */
	mutex_init(&dp->intrlock, NULL, MUTEX_DRIVER, (void *)c);
	mutex_init(&dp->xmitlock, NULL, MUTEX_DRIVER, (void *)c);
	cv_init(&dp->drain_cv, NULL, CV_DRIVER, NULL);

	/*
	 * configure gem parameter
	 */
	dp->base_addr   = base;
	dp->regs_handle = *regs_handlep;
#ifdef notdef
	/* XXX - avoid that gcc generates calling memcpy */
	dp->gc = *gc;
#else
	STRUCT_COPY(dp->gc, *gc);
#endif
	gc     = &dp->gc;

	/* fix max frags */
	dp->gc.gc_tx_max_frags = min(dp->gc.gc_tx_max_frags, GEM_MAXTXSEGS);
	dp->gc.gc_rx_max_frags = min(dp->gc.gc_rx_max_frags, GEM_MAXRXSEGS);

	/* fix copy threadsholds */
#ifdef BUG_MEM_ALLOC	/* obsoluleted in Solaris10 FCS */
	dp->gc.gc_tx_copy_thresh = max(PAGESIZE, dp->gc.gc_tx_copy_thresh);
	dp->gc.gc_rx_copy_thresh = max(PAGESIZE, dp->gc.gc_rx_copy_thresh);
#else
	dp->gc.gc_tx_copy_thresh = max(ETHERMIN, dp->gc.gc_tx_copy_thresh);
	dp->gc.gc_rx_copy_thresh = max(ETHERMIN, dp->gc.gc_rx_copy_thresh);
#endif
	/* fix rx buffer boundary for iocache line size */
	ASSERT(gc->gc_dma_attr_txbuf.dma_attr_align-1==dp->gc.gc_tx_buf_align);
	ASSERT(gc->gc_dma_attr_rxbuf.dma_attr_align-1==dp->gc.gc_rx_buf_align);
	dp->gc.gc_rx_buf_align = max(dp->gc.gc_rx_buf_align, IOC_LINESIZE-1); 
	dp->gc.gc_dma_attr_rxbuf.dma_attr_align = dp->gc.gc_rx_buf_align + 1;

	/* fix get_packet method */
	if (dp->gc.gc_get_packet == NULL) {
		dp->gc.gc_get_packet = &gem_get_packet_default;
	}

	dp->mtu		= ETHERMTU;
	dp->tx_buf	= (struct txbuf *)((caddr_t)(&dp[1])+GEM_MCALLOC);
	/* make txbuf free list */
{
	struct txbuf	*head = NULL;
	struct txbuf	*tbp;
	for (i = 0, tbp = dp->tx_buf; i < dp->gc.gc_tx_buf_size; i++, tbp++) {
		tbp->txb_next = head;
		head = tbp;
		tbp->txb_nfrags = 0; /* that means no dma mapping */
#ifdef SANITY
		tbp->txb_ndesc  = 0;
		tbp->txb_mp      = NULL;
		tbp->txb_dh_used = NULL;
#endif
	}
	dp->tx_buf_freelist = head;
	dp->tx_buf_freecnt  = dp->gc.gc_tx_buf_size;
}
	dp->rxmode	= 0;
	dp->mii_fixedmode = FALSE;	/* default is autoneg */
	dp->speed	  = GEM_SPD_10;	/* default is 10Mbps */
	dp->full_duplex   = FALSE;	/* default is half */
	dp->flow_control  = FLOW_CONTROL_NONE;
	dp->poll_pkt_delay = 6;
	dp->poll_pkt_hiwat = INT32_MAX;

        /* performance tuning parameters */
	dp->txthr    = ETHERMAX;	/* tx fifo threshold */
	dp->txmaxdma = 16*4;		/* tx max dma burst size */
	dp->rxthr    = 128;		/* rx fifo threshold */
	dp->rxmaxdma = 16*4;		/* rx max dma burst size */

	/*
	 * Get media mode information from .conf file
	 */
	gem_read_conf(dp);

	/* rx_buf_len is required buffer length without padding for alignment */
	dp->rx_buf_len = MAXPKTBUF(dp) + dp->gc.gc_rx_header_len;

	/*
	 * Reset the chip
	 */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
#ifdef GEM_DELAYED_START
	dp->nic_online = FALSE;
#endif
	ret = GEM_RESET_CHIP(dp);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);
	if (ret != GEM_SUCCESS) {
		goto err_free_regs;
	}

	/*
	 * HW dependant paremeter initialization
	 */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
	ret = GEM_ATTACH_CHIP(dp);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);
	if (ret != GEM_SUCCESS) {
		goto err_free_regs;
	}

	/*
	 * configure GLD 
	 */
	macinfo->gldm_devinfo	   = dp->dip;
	macinfo->gldm_private	   = (caddr_t)dp;
	macinfo->gldm_cookie       = dp->iblock_cookie;
	macinfo->gldm_reset        = &gem_gld_reset;
	macinfo->gldm_start        = &gem_gld_start;
	macinfo->gldm_stop         = &gem_gld_stop;
	macinfo->gldm_set_mac_addr = &gem_gld_set_mac_address;
	macinfo->gldm_send         = &gem_gld_send;
#ifdef GEM_CONFIG_VLAN
	if ((dp->misc_flag & GEM_VLAN) != 0) {
		macinfo->gldm_send_tagged = &gem_gld_send_tagged;
	}
#endif
	macinfo->gldm_set_promiscuous = &gem_gld_set_promiscuous;
	macinfo->gldm_get_stats    = &gem_gld_get_stats;
	macinfo->gldm_ioctl        = NULL; 
	macinfo->gldm_set_multicast= &gem_gld_set_multicast;
	macinfo->gldm_intr         = &gem_gld_intr;
	macinfo->gldm_mctl         = NULL;
#ifdef GEM_COMPAT
	macinfo->gldm_ident   = (char *)ddi_driver_name(dip);
#else
	macinfo->gldm_ident   = ident;
#endif
	macinfo->gldm_type    = DL_ETHER;
	macinfo->gldm_minpkt  = 0;
	macinfo->gldm_maxpkt  = dp->mtu;
	macinfo->gldm_addrlen = ETHERADDRL;
	macinfo->gldm_saplen  = -2;
	macinfo->gldm_ppa     = unit;

#ifdef SOLARIS10
	macinfo->gldm_capabilities = GLD_CAP_LINKSTATE;
#  ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if ((dp->misc_flag & (GEM_CKSUM_IPv4 | GEM_CKSUM_TCP | GEM_CKSUM_UDP))
			 == (GEM_CKSUM_IPv4 | GEM_CKSUM_TCP | GEM_CKSUM_UDP)) {
		macinfo->gldm_capabilities |= 
			(GLD_CAP_CKSUM_IPHDR | GLD_CAP_CKSUM_FULL_V4);
	}
#  endif
#endif
	/* allocate tx and rx resources */
	if (gem_alloc_memory(dp) != 0) {
		goto err_free_regs;
	}

	DPRINTF(0, (CE_CONT,
	"!%s: at 0x%x, %02x:%02x:%02x:%02x:%02x:%02x",
		dp->name, (long)dp->base_addr,
		dp->dev_addr.ether_addr_octet[0],
		dp->dev_addr.ether_addr_octet[1],
		dp->dev_addr.ether_addr_octet[2],
		dp->dev_addr.ether_addr_octet[3],
		dp->dev_addr.ether_addr_octet[4],
		dp->dev_addr.ether_addr_octet[5]));

	/* copy mac address */
	dp->cur_addr = dp->dev_addr;
	macinfo->gldm_vendor_addr =
			dp->dev_addr.ether_addr_octet;
	macinfo->gldm_broadcast_addr =
			gem_etherbroadcastaddr.ether_addr_octet;

	/* Init MII (scan phy)*/
	if (GEM_MII_INIT(dp) != 0) {
		goto err_free_ring;
	}

	/* reset_mii and start mii link watcher */
	gem_mii_start(dp);

	/*
	 * Add interrupt to system.
	 */
	if (gld_register(dip,
			(char *)ddi_driver_name(dip), macinfo) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: gld_register failed", dp->name);
		goto err_stop_mii;
	}
#ifdef GEM_CONFIG_JUMBO_PACKET	/* experimental workaround */
{
	gld_mac_pvt_t		*gpp;
	gld_interface_t		*gifp;
	static gld_interface_t	my_if;

	gpp  = (gld_mac_pvt_t *)(macinfo->gldm_mac_pvt);
	gifp = gpp->interfacep;

	bcopy(gifp, &my_if, sizeof(my_if));

	DPRINTF(0, (CE_CONT, "%s: gld interface: mtu_size:%d",
		dp->name, gifp->mtu_size));

	my_if.mtu_size = dp->mtu;
	gpp->interfacep =  &my_if;
	macinfo->gldm_maxpkt  = dp->mtu;
}
#endif /* JUMBO_PACKET */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		if (ddi_add_intr(dip, 0, NULL, NULL, gld_intr,
					(caddr_t)macinfo) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s: ddi_add_intr failed", dp->name);
			goto err_unregister;
		}
	}
	else {
		/*
		 * Dont use interrupt.
		 * schedule first call of gem_intr_watcher
		 */
		dp->intr_watcher_id =
			timeout((void (*)(void *))gem_intr_watcher,
				(void *)macinfo, drv_usectohz(3*1000000));
	}

	DPRINTF(2, (CE_CONT, "!gem_do_attach: return: success"));
	return (dp);

err_unregister:
	gld_unregister(macinfo);
err_stop_mii:
	gem_mii_stop(dp);
err_free_ring:
	gem_free_memory(dp);
err_free_regs:
	ddi_regs_map_free(&dp->regs_handle);
err_free_locks:
	mutex_destroy(&dp->xmitlock);
	mutex_destroy(&dp->intrlock);
	cv_destroy(&dp->drain_cv);
err_free_private:
	kmem_free((caddr_t)dp, GEM_LOCAL_DATA_SIZE(gc));
	gld_mac_free(macinfo);

	return (NULL);
}

int
gem_do_detach(dev_info_t *dip)
{
	struct gem_dev	*dp;
	gld_mac_info_t	*macinfo;
	timeout_id_t	old_id;

	macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
	dp = (struct gem_dev *)macinfo->gldm_private;

	/* check if all rx buffers are freed */

	if (dp->rx_buf_allocated != dp->rx_buf_freecnt) {
		/*
		 * resource is busy
		 */
		cmn_err(CE_NOTE,
		"!%s: gem_do_detach: rxbuf is busy: allocated:%d, freecnt:%d",
			dp->name, dp->rx_buf_allocated, dp->rx_buf_freecnt);
#ifdef GEM_DEBUG_LEVEL
		return (DDI_FAILURE);
#endif
	}

	/*
	 * stop the device
	 */

	/* stop interrupt watcher */
	if ((dp->misc_flag & GEM_NOINTR) != 0 &&
	    dp->intr_watcher_id != 0) {
		do {
			untimeout(old_id = dp->intr_watcher_id);
		} while (old_id != dp->intr_watcher_id);
		dp->intr_watcher_id = 0;
	}

	gem_mii_stop(dp);

	/*
	 * unregister interrupt handler
	 */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		ddi_remove_intr(dip, 0, dp->iblock_cookie);
	}

	(void)gld_unregister(macinfo);

	gem_free_memory(dp);

	/*
	 * Release mapping resources
	 */
	ddi_regs_map_free(&dp->regs_handle);

	mutex_destroy(&dp->xmitlock);
	mutex_destroy(&dp->intrlock);
	cv_destroy(&dp->drain_cv);

	/*
	 * Release memory resources
	 */
	kmem_free((caddr_t)(dp->private), dp->priv_size);
	kmem_free((caddr_t)dp, GEM_LOCAL_DATA_SIZE(&dp->gc));
	gld_mac_free(macinfo);
	DPRINTF(2, (CE_CONT, "!%s%d: gem_do_detach: return: success",
			ddi_driver_name(dip), ddi_get_instance(dip)));

	return (DDI_SUCCESS);
}

int
gem_suspend(dev_info_t *dip)
{
	struct gem_dev	*dp;
	gld_mac_info_t	*macinfo;
	timeout_id_t	old_id;

	macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
	dp = (struct gem_dev *)macinfo->gldm_private;

	/*
	 * stop the device
	 */

	/* stop interrupt watcher */
	if ((dp->misc_flag & GEM_NOINTR) != 0 &&
	    dp->intr_watcher_id != 0) {
		do {
			untimeout(old_id = dp->intr_watcher_id);
		} while (old_id != dp->intr_watcher_id);
		dp->intr_watcher_id = 0;
	}

	/* stop link watcher */
	gem_mii_stop(dp);

	/* stop tx timeout watcher */
	if (dp->timeout_id) {
		do {
			untimeout(old_id = dp->timeout_id);
		} while (dp->timeout_id != old_id);	
		dp->timeout_id = 0;
	}

	/* make the nic state inactive */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
#ifdef GEM_DELAYED_START
	dp->nic_online = FALSE;
#endif
	(void) gem_mac_stop(dp, FALSE);
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (DDI_SUCCESS);
}

int
gem_resume(dev_info_t *dip)
{
	struct gem_dev	*dp;
	gld_mac_info_t	*macinfo;

	macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
	dp = (struct gem_dev *)macinfo->gldm_private;

	/*
	 * restart the device
	 */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	gem_restart_nic(dp, FALSE);
	gem_mii_start(dp);
	mutex_exit(&dp->xmitlock);

	/* restart tx timetou watcher */
	dp->timeout_id = timeout((void (*)(void *))gem_tx_timeout,
				(void *)dp, dp->gc.gc_tx_timeout_interval);

	if ((dp->misc_flag & GEM_NOINTR) != 0) {
		/*
		 * Dont use interrupt.
		 * schedule first call of gem_intr_watcher
		 */
		dp->intr_watcher_id =
			timeout((void (*)(void *))gem_intr_watcher,
				(void *)macinfo, drv_usectohz(3*1000000));
	}
	mutex_exit(&dp->intrlock);

	return (DDI_SUCCESS);
err:
	return (DDI_FAILURE);
}

/*
 * misc routines for PCI
 */
int
gem_pci_set_power_state(dev_info_t *dip,
		ddi_acc_handle_t conf_handle, uint_t new_mode)
{
	uint8_t		pci_cap_ptr;
	uint32_t	pci_cap;
	uint32_t	pmcsr;
	uint_t		unit;
	const char	*drv_name;

	ASSERT(new_mode < 4);

        unit =  ddi_get_instance(dip);
        drv_name = ddi_driver_name(dip);

	/* search power management capablities */
	pci_cap_ptr = pci_config_get8(conf_handle, PCI_CONF_CAP_PTR);
	while (pci_cap_ptr != 0) {
		/* read pci capability header */
		pci_cap = pci_config_get32(conf_handle, pci_cap_ptr);
		if ((pci_cap & 0xff) == PCI_CAP_ID_PM) {
			/*
			 * power management capability found
			 */
			break;
		}
		/* get next_ptr */
		pci_cap_ptr = (pci_cap >> 8) & 0xff;
	}
	if (pci_cap_ptr == 0) {
		cmn_err(CE_CONT,
			"!%s%d: doesn't have pci power management capability",
			drv_name, unit);
		return (GEM_FAILURE);
	}

	/* read power management capabilities */
	pmcsr = pci_config_get32(conf_handle, pci_cap_ptr + PCI_PMCSR);

	DPRINTF(0, (CE_CONT,
		"!%s%d: pmc found at 0x%x: cap: 0x%08x, pmcsr: 0x%08x",
		drv_name, unit, pci_cap_ptr, pci_cap, pmcsr));

	/*
	 * Is the resuested power mode supported?
	 */
	/* not yet */

	/*
	 * move to new mode
	 */
	pmcsr = (pmcsr & ~PCI_PMCSR_STATE_MASK) | new_mode;
	pci_config_put32(conf_handle, pci_cap_ptr + PCI_PMCSR, pmcsr);

	return (GEM_SUCCESS);
}

int
gem_pci_regs_map_setup(dev_info_t *dip, uint32_t which,
	struct ddi_device_acc_attr *attrp,
	caddr_t *basep, ddi_acc_handle_t *hp)
{
	struct pci_phys_spec	*regs;
	uint_t		len;
	uint_t		unit;
	uint_t		n;
	uint_t		i;
	int		ret;
	const char	*drv_name;
	const char	*space_names[] = {"CONFIG", "IO", "MEM32", "MEM64"};

        unit     = ddi_get_instance(dip);
        drv_name = ddi_driver_name(dip);

	/* Search IO-range or memory-range to be mapped */
	regs = NULL;
	len  = 0;

	if (ddi_prop_lookup_int_array(
		DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		"reg", (int **)&regs, &len) != DDI_PROP_SUCCESS) {
		cmn_err(CE_WARN,
			"!%s%d: failed to get reg property (ret:%d)",
				drv_name, unit, ret);
		return (ret);
	}
	n = len / (sizeof(struct pci_phys_spec) / sizeof(int));

	ASSERT(regs != NULL && len > 0);

#if DEBUG_LEVEL > 0
	for (i = 0; i < n; i++) {
		cmn_err(CE_CONT,
			"!%s%d: regs[%d]: %08x.%08x.%08x.%08x.%08x",
			drv_name, unit, i,
			regs[i].pci_phys_hi,
			regs[i].pci_phys_mid,
			regs[i].pci_phys_low,
			regs[i].pci_size_hi,
			regs[i].pci_size_low);
	}
#endif
	for (i = 0; i < n; i++) {
		if ((regs[i].pci_phys_hi & PCI_REG_ADDR_M) == which) {
			/* it's the requested space */
			ddi_prop_free(regs);
			goto address_range_found;
		}
	}
	cmn_err(CE_WARN, "!%s%d: failed to find %s space",
		drv_name, unit, space_names[PCI_REG_ADDR_G(which)]);
	ddi_prop_free(regs);
	return (GEM_FAILURE);

address_range_found:
	if ((ret = ddi_regs_map_setup(dip, i, basep, 0, 0, attrp, hp))
			!= DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed (ret:%d)",
			drv_name, unit, ret);
	}

	return (ret);
}
