/*
 *  gem.c: general ethernet mac driver framework version 2.4
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

#pragma ident	"@(#)gem.c	1.94 07/02/11"

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
--- gem v2---
 01/15/2006  fcs
 06/18/2006  gem_restart_nic fixed to call xxx_reset_chip first on resume case.
 08/06/2006  kstat support
 10/01/2006  performance of gem_receive and gem_reclaim_txbuf were improved
 11/12/2006  runtime supports for gcc were  added as weak symbols.
	     mii detection improved
	     gem_send_common loop limited by the number of allocated buffers
 */

/*
 TODO:
	implement rx drain in gem_mac_stop
	improve tx drain in gem_mac_stop
 */

/*
 * System Header files.
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strsubr.h>
#include <sys/strlog.h>
#include <sys/kmem.h>
#include <sys/stat.h>
#include <sys/kstat.h>
#include <sys/vtrace.h>
#include <sys/dlpi.h>
#include <sys/strsun.h>
#include <sys/ethernet.h>
#include <sys/modctl.h>
#include <sys/kstat.h>
#include <sys/errno.h>
#include <sys/dditypes.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
#include <sys/pattr.h>
#endif

#include <sys/pci.h>

#ifdef GEM_CONFIG_GLDv3	/* XXXX - from bge in opensolaris */
/*
 * Reconfiguring the network devices requires the net_config privilege
 * in Solaris 10+.  Prior to this, root privilege is required.  In order
 * that the driver binary can run on both S10+ and earlier versions, we
 * make the decisiion as to which to use at runtime.  These declarations
 * allow for either (or both) to exist ...
 */
extern int secpolicy_net_config(const cred_t *, boolean_t);
extern int drv_priv(cred_t *);
#pragma weak    secpolicy_net_config
#pragma weak    drv_priv
#endif /* GEM_CONFIG_GLDv3 */

#include <sys/note.h>

#include "mii.h"
#include "gem.h"

#if defined(GEM_CONFIG_CKSUM_OFFLOAD) && !defined(SOLARIS10)
/* GEM_CONFIG_CKSUM_OFFLOAD requires solaris10 or later*/
# define SOLARIS10
#endif

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
#include <netinet/in.h>
#endif

#ifdef MODULE
char	ident[] = "general ethernet mac driver v2" 
#else
extern char ident[];
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
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))
#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

#define	GET_NET16(p)	((((uint8_t *)(p))[0] << 8)| ((uint8_t *)(p))[1])
#define	GET_ETHERTYPE(p)	GET_NET16(((uint8_t *)(p)) + ETHERADDRL*2)

#define	GET_IPTYPEv4(p)	(((uint8_t *)(p))[sizeof(struct ether_header) + 9])
#define	GET_IPTYPEv6(p)	(((uint8_t *)(p))[sizeof(struct ether_header) + 6])

#define	VTAG_OFF		(ETHERADDRL*2)

#ifndef INT32_MAX
# define INT32_MAX      0x7fffffff
#endif
#ifndef VTAG_SIZE
# define	VTAG_SIZE	4
#endif
#ifndef VTAG_TPID
# define	VTAG_TPID	0x8100u
#endif

#define	DEL_VTAG(mp)	{	\
	uint8_t	*bp = (uint8_t *)((mp)->b_rptr);	\
	switch (3ull & (long) bp) {	\
	case 3:	\
		bp[VTAG_OFF - 3 + 4] = bp[VTAG_OFF - 3];	\
	case 2:	\
		bp[VTAG_OFF - 2 + 4] = bp[VTAG_OFF - 2];	\
	case 1:	\
		bp[VTAG_OFF - 1 + 4] = bp[VTAG_OFF - 1];	\
	}	\
	bp = (void *) (long) ((~3ull) & (long)bp);	\
	((uint32_t *)(bp))[3] = ((uint32_t *)(bp))[2];	\
	((uint32_t *)(bp))[2] = ((uint32_t *)(bp))[1];	\
	((uint32_t *)(bp))[1] = ((uint32_t *)(bp))[0];	\
	(mp)->b_rptr += VTAG_SIZE;	\
}

#define	ADD_VTAG(mp, vtag)	{	\
	uint8_t	*bp;	\
	\
	(mp)->b_rptr -= VTAG_SIZE;	\
	bp = (uint8_t *)((mp)->b_rptr);	\
	\
	switch (3ull & (long) bp) {	\
	case 3:	\
		bp[VTAG_OFF - 3] = bp[VTAG_OFF - 3 + 4];	\
	case 2:	\
		bp[VTAG_OFF - 2] = bp[VTAG_OFF - 2 + 4];	\
	case 1:	\
		bp[VTAG_OFF - 1] = bp[VTAG_OFF - 1 + 4];	\
	}	\
	bp[VTAG_OFF + 0] = (uint8_t)(VTAG_TPID >> 8);	\
	bp[VTAG_OFF + 1] = (uint8_t) VTAG_TPID;	\
	bp[VTAG_OFF + 2] = (uint8_t)((vtag) >> 8);	\
	bp[VTAG_OFF + 3] = (uint8_t) (vtag);	\
	bp = (void *) (long) ((~3ull) & (long)bp);	\
	((uint32_t *)(bp))[0] =  ((uint32_t *)(bp))[1];	\
	((uint32_t *)(bp))[1] =  ((uint32_t *)(bp))[2];	\
	((uint32_t *)(bp))[2] =  ((uint32_t *)(bp))[3];	\
}

#define	GET_TXBUF(dp, sn)	\
	&(dp)->tx_buf[SLOT((dp)->tx_slots_base + (sn), (dp)->gc.gc_tx_buf_size)]

/*
 * configuration parameters
 */
#define	MAXPKTBUF(dp)	\
	((dp)->mtu + sizeof(struct ether_header) + VTAG_SIZE + ETHERFCSL)

#define	WATCH_INTERVAL_FAST	drv_usectohz(100*1000)	/* 100mS */

#define	HIWAT_INTR(x)	(3*((x)/4))

/*
 * Macros to distinct chip generation.
 */

/*
 * Private functions
 */
static void gem_mii_start(struct gem_dev *);
static void gem_mii_stop(struct gem_dev *);

/* local buffer management */
static int gem_alloc_memory(struct gem_dev *);
static void gem_free_memory(struct gem_dev *);
static void gem_init_rx_ring(struct gem_dev *);
static void gem_prepare_rx_buffer(struct gem_dev *);
static void gem_init_tx_ring(struct gem_dev *);
__INLINE__ static void gem_append_rxbuf(struct gem_dev *, struct rxbuf *);

static void gem_tx_timeout(struct gem_dev *);
static void gem_mii_link_watcher(struct gem_dev *dp);
static int gem_mac_init(struct gem_dev *dp);
static int gem_mac_start(struct gem_dev *dp);
static int gem_mac_stop(struct gem_dev *dp, uint_t flags);

static	struct ether_addr	gem_etherbroadcastaddr = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

int gem_speed_value[] = {10, 100, 1000};

/* prevent i86pc rootnex from using copybuffer for dma */
static ddi_dma_attr_t gem_dma_attr_get_paddr = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x7fffffffffffffffull,	/* dma_attr_addr_hi */
	0x7fffffffffffffffull,	/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x7fffffffffffffffull,	/* dma_attr_maxxfer */
	0x7fffffffffffffffull,	/* dma_attr_seg */
	GEM_MAXTXFRAGS,		/* dma_attr_sgllen */
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

	return status;
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

	return status;
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
#endif /* MODULE */

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
gem_ether_crc_le(uint8_t *addr, int len)
{
	int		idx;
	int		bit;
	u_int		data;
	uint32_t	crc = 0xffffffff;

	crc = 0xffffffff;
	for (idx = 0; idx < len; idx++) {
		for (data = *addr++, bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc >> 1)
			    ^ (((crc ^ data) & 1) ? CRC32_POLY_LE : 0);
		}
	}
	return (crc);
}

#define	CRC32_POLY_BE	0x04c11db7
uint32_t
gem_ether_crc_be(uint8_t *addr, int len)
{
	int		idx;
	int		bit;
	u_int		data;
	uint32_t	crc;

	crc = 0xffffffff;
	for (idx = 0; idx < len; idx++) {
                for (data = *addr++, bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc << 1)
			    ^ ((((crc >> 31) ^ data) & 1) ? CRC32_POLY_BE : 0);
		}
	}
	return (crc);
}

int
gem_prop_get_int(struct gem_dev *dp, char *prop_template, int def_val)
{
	char	propname[32];

	sprintf(propname, prop_template, dp->name);

	return (ddi_prop_get_int(DDI_DEV_T_ANY, dp->dip,
				DDI_PROP_DONTPASS, propname, def_val));
}

static int
gem_population(uint32_t x)
{
	int	i;
	int	cnt;

	cnt = 0;
	for (i = 0; i < 32; i++) {
		if ((x & (1 << i)) != 0) {
			cnt++;
		}
	}
	
	return (cnt);
}

/*
 * gcc3 runtime routines
 */
#pragma weak memcmp
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

#pragma weak memset
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

#pragma weak memcpy
void *
memcpy(void *s1, const void *s2, size_t n)
{
	bcopy(s2, s1, n);
	return (s1);
}

/* ============================================================== */
/*
 * IO cache flush
 */
/* ============================================================== */
void
gem_rx_desc_dma_sync(struct gem_dev *dp, int head, int nslot, int how)
{
	int	n;
	int	m;
	int	rx_desc_unit_shift = dp->gc.gc_rx_desc_unit_shift;

	/* sync active descriptors */
	if (rx_desc_unit_shift < 0 || nslot == 0) {
		/* no rx descriptor ring */
		return;
	}

	n = dp->gc.gc_rx_ring_size - head;
	if ((m = nslot - n) > 0) {
		ddi_dma_sync(dp->desc_dma_handle,
			(off_t) 0,
			(size_t) (m << rx_desc_unit_shift),
			 how);
		nslot = n;
	}

	(void) ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(head << rx_desc_unit_shift),
		(size_t)(nslot << rx_desc_unit_shift),
		how);
}

void
gem_tx_desc_dma_sync(struct gem_dev *dp, int head, int nslot, int how)
{
	int	n;
	int	m;
	int	tx_desc_unit_shift = dp->gc.gc_tx_desc_unit_shift;

	/* sync active descriptors */
	if (tx_desc_unit_shift < 0 || nslot == 0) {
		/* no tx descriptor ring */
		return;
	}

	n = dp->gc.gc_tx_ring_size - head;
	if ((m = nslot - n) > 0) {
		ddi_dma_sync(dp->desc_dma_handle,
			(off_t) (dp->tx_ring_dma - dp->rx_ring_dma),
			(size_t) (m << tx_desc_unit_shift),
			 how);
		nslot = n;
	}

	(void) ddi_dma_sync(dp->desc_dma_handle,
		(off_t) ((head << tx_desc_unit_shift)
			+ (dp->tx_ring_dma - dp->rx_ring_dma)),
		(size_t) (nslot << tx_desc_unit_shift),
		how);
}

/* ============================================================== */
/*
 * Buffer management
 */
/* ============================================================== */
static void
gem_dump_txbuf(struct gem_dev *dp, int level, const char *title)
{
	cmn_err(level,
	"!%s: %s: tx_active: %d[%d] %d[%d] (+%d), "
	"tx_softq: %d[%d] %d[%d] (+%d), "
	"tx_free: %d[%d] %d[%d] (+%d), "
	"tx_desc: %d[%d] %d[%d] (+%d), "
	"intr: %d[%d] (+%d)",
		dp->name, title,
		dp->tx_active_head,
			SLOT(dp->tx_active_head, dp->gc.gc_tx_buf_size),
		dp->tx_active_tail,
			SLOT(dp->tx_active_tail, dp->gc.gc_tx_buf_size),
			dp->tx_active_tail - dp->tx_active_head,
		dp->tx_softq_head,
			SLOT(dp->tx_softq_head, dp->gc.gc_tx_buf_size),
		dp->tx_softq_tail,
			SLOT(dp->tx_softq_tail, dp->gc.gc_tx_buf_size),
			dp->tx_softq_tail - dp->tx_softq_head,
		dp->tx_free_head,
			SLOT(dp->tx_free_head, dp->gc.gc_tx_buf_size),
		dp->tx_free_tail,
			SLOT(dp->tx_free_tail, dp->gc.gc_tx_buf_size),
			dp->tx_free_tail - dp->tx_free_head,
		dp->tx_desc_head,
			SLOT(dp->tx_desc_head, dp->gc.gc_tx_ring_size),
		dp->tx_desc_tail,
			SLOT(dp->tx_desc_tail, dp->gc.gc_tx_ring_size),
			dp->tx_desc_tail - dp->tx_desc_head,
		dp->tx_desc_intr,
			SLOT(dp->tx_desc_intr, dp->gc.gc_tx_ring_size),
			dp->tx_desc_intr - dp->tx_desc_head);
}

#define	GEM_FREE_RXBUF(rbp)	\
{	\
	struct gem_dev	*dp;	\
	\
	dp = (rbp)->rxb_devp;	\
	ASSERT(mutex_owned(&dp->intrlock));	\
	(rbp)->rxb_next = dp->rx_buf_freelist;	\
	dp->rx_buf_freelist = (rbp);	\
	dp->rx_buf_freecnt++;	\
}

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
		rbp->rxb_next = NULL;

		if (rbp->rxb_mp != NULL) {
			/*
			 * All required resources have been prepared,
			 * just return.
			 */
			ASSERT(rbp->rxb_nfrags > 0);
			return rbp;
		}

		goto allocate_buffer;
	}

	/*
	 * Allocate a rx buffer management structure
	 */
	if ((rbp = (struct rxbuf *)
			kmem_zalloc(sizeof(struct rxbuf),
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
	 * refered on freeing the buffer later.
	 */
	rbp->rxb_devp = dp;

	/* allocate a dma handle for rx direct buffer */
	if ((err = ddi_dma_alloc_handle(dp->dip,
#if defined(sun4u)
			&dp->gc.gc_dma_attr_rxbuf,
#else
			&gem_dma_attr_get_paddr,
#endif
			(cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT),
			NULL, &rbp->rxb_dh)) != DDI_SUCCESS) {

		cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_alloc_handle:1 failed, err=%d",
			dp->name, __func__, err);

		kmem_free(rbp, sizeof(struct rxbuf));
		return (NULL);
	}

	/* allocate a dma handle for rx bounce buffer */
	if ((err = ddi_dma_alloc_handle(dp->dip,
			&dp->gc.gc_dma_attr_rxbuf,
			cansleep ? DDI_DMA_SLEEP : DDI_DMA_DONTWAIT,
			NULL, &rbp->rxb_bdh)) != DDI_SUCCESS) {

		cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_alloc_handle:2 failed, err=%d",
			dp->name, __func__, err);

		ddi_dma_free_handle(&rbp->rxb_dh);
		kmem_free(rbp, sizeof(struct rxbuf));
		return (NULL);
	}


	/* allocate a bounce buffer for rx */
	if ((err = ddi_dma_mem_alloc(rbp->rxb_bdh,
		ROUNDUP(dp->rx_buf_len, IOC_LINESIZE),
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
#ifdef notdef
	if (dp->gc.gc_dma_attr_rxbuf.dma_attr_align > 8) {
		/* this alignment includes IOC_LINESIZE */
		align = dp->gc.gc_dma_attr_rxbuf.dma_attr_align;
	} else {
		/*
		 * As allocb() returns an 8-byte-aligned buffer,
		 * no need to manipulate starting addresss of it.
		 */
		align = 1;
	}
#else/* VTAG */
	align = dp->gc.gc_dma_attr_rxbuf.dma_attr_align;
#endif
	if ((rbp->rxb_mp = allocb(ROUNDUP(dp->rx_buf_len, IOC_LINESIZE) +
				  (align - 1) + ROUNDUP(VTAG_SIZE, align),
							BPRI_MED)) == NULL) {
		GEM_FREE_RXBUF(rbp);
		return (NULL);
	}
	rbp->rxb_buf = (caddr_t) ROUNDUP(
			((long)rbp->rxb_mp->b_rptr) + VTAG_SIZE, align);
	rbp->rxb_buf_len = dp->rx_buf_len;
	rbp->rxb_mp->b_rptr = (uchar_t *)rbp->rxb_buf + dp->gc.gc_rx_header_len;
	rbp->rxb_flags &= ~RXB_FLAGS_BOUNCE;

	if (MAXPKTBUF(dp) <= dp->gc.gc_rx_copy_thresh) {
use_bounce_buffer:
		/*
		 * Copy case: try to map the bounce buffer into the DMA space,
		 * instead of the mblk.
		 */
		DPRINTF(4, (CE_CONT, "!%s: %s: mapping rx bounce buffer",
			dp->name, __func__));
		rbp->rxb_buf     = rbp->rxb_bbuf;
		rbp->rxb_buf_len = rbp->rxb_bbuf_len;
		rbp->rxb_flags  |= RXB_FLAGS_BOUNCE;
#ifdef notdef /* GEM_DEBUG_LEVEL */
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
			"!%s: %s: switch to rx bounce buffer: "
			"ddi_dma_addr_bind_handle: failed, err=%d",
			dp->name, __func__, err));

		if (err == DDI_DMA_PARTIAL_MAP) {
			/* we need to free partially allocated dma resources. */
			ddi_dma_unbind_handle(rbp->rxb_dh);
		}

		if ((rbp->rxb_flags & RXB_FLAGS_BOUNCE) != 0) {
			/*
			 * we failed to allocate a dma resource
			 * for the rx bounce buffer.
			 */
			freemsg(rbp->rxb_mp);
			rbp->rxb_mp = NULL;
			rbp->rxb_buf = NULL;
			rbp->rxb_buf_len = 0;
			GEM_FREE_RXBUF(rbp);
			return (NULL);
		}
		/*
		 * change current configuration for rx to choose "copy" 
		 */
		dp->gc.gc_rx_copy_thresh = MAXPKTBUF(dp);
		goto use_bounce_buffer;
	}
#ifdef NEVER	/* solaris nv only */
	ASSERT((((ddi_dma_impl_t *) rbp->rxb_dh)->dmai_rflags & DMP_NOSYNC)
				== DMP_NOSYNC);
#endif
	if (count > dp->gc.gc_rx_max_frags) {
		/*
		 * Current configuration for rx isn't suitable for the
		 * DMA capability of the nic.
		 */
		DPRINTF(0, (CE_NOTE,
		"!%s: %s: switch to rx bounce buffer: too many rx fragments",
			dp->name, __func__));
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
				"!%s: %s: switch to rx bounce buffer: "
				"the dma range is not suitabble for the nic",
				dp->name, __func__));

			ASSERT((rbp->rxb_flags & RXB_FLAGS_BOUNCE) == 0);
			goto use_bounce_buffer;
		}
	}

	return (rbp);
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
	int			tx_buf_size = dp->gc.gc_tx_buf_size;
	int			tx_buf_len;
	ddi_dma_attr_t		dma_attr_txbounce;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	dp->desc_dma_handle = NULL;
	req_size = dp->rx_desc_size + dp->tx_desc_size + dp->gc.gc_io_area_size;

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
#ifdef NEVER	/* solaris nv only */
	ASSERT((((ddi_dma_impl_t *) dp->desc_dma_handle)->dmai_rflags & DMP_NOSYNC)
				== DMP_NOSYNC);
#endif

		/* set base of rx descriptor ring */
		dp->rx_ring     = ring;
		dp->rx_ring_dma = ring_cookie.dmac_laddress;

		/* set base of tx descriptor ring */
		dp->tx_ring     = dp->rx_ring + dp->rx_desc_size;
		dp->tx_ring_dma = dp->rx_ring_dma + dp->rx_desc_size;

		/* set base of io area */
		dp->io_area     = dp->tx_ring + dp->tx_desc_size;
		dp->io_area_dma = dp->tx_ring_dma + dp->tx_desc_size;
	}

	if (dp->gc.gc_tx_buf_size > 0) {
		/*
		 * Prepare tx buffers and bounce buffers.
		 */

		/* special dma attribute for tx bounce buffers */
		STRUCT_COPY(dma_attr_txbounce, dp->gc.gc_dma_attr_txbuf);
		dma_attr_txbounce.dma_attr_sgllen = 1;
		dma_attr_txbounce.dma_attr_align =
			max(dma_attr_txbounce.dma_attr_align, IOC_LINESIZE);

		/* Size for tx bounce buffers must be max tx packet size. */
		tx_buf_len = MAXPKTBUF(dp);
		tx_buf_len = ROUNDUP(tx_buf_len, IOC_LINESIZE);

		ASSERT(tx_buf_len >= ETHERMAX+ETHERFCSL);

		for (i = 0, tbp = dp->tx_buf; i < tx_buf_size; i++, tbp++) {
			/*
			 * Allocate dma handles for each fragment of
			 * buffers for zerocopy transmittion.
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
			/* new way: allocate a bounce buffer for each txbuf */
			if ((err = ddi_dma_alloc_handle(dp->dip,
					&dma_attr_txbounce,
					DDI_DMA_SLEEP, NULL,
					&tbp->txb_bdh)) != DDI_SUCCESS) {

				cmn_err(CE_WARN,
			"!%s: %s ddi_dma_alloc_handle for bounce buffer failed:"
			" err=%d, i=%d",
					dp->name, __func__, err, i);
				goto err_alloc_dh;
			}

			if ((err = ddi_dma_mem_alloc(tbp->txb_bdh,
					tx_buf_len,
					&dp->gc.gc_buf_attr,
					DDI_DMA_STREAMING, DDI_DMA_SLEEP, NULL,
					&buf, &buf_len,
					&tbp->txb_bah)) != DDI_SUCCESS) {
				cmn_err(CE_WARN,
			"!%s: %s: ddi_dma_mem_alloc for bounce buffer failed"
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
#ifdef NEVER	/* solaris nv only */
	ASSERT((((ddi_dma_impl_t *) tbp->txb_bdh)->dmai_rflags & DMP_NOSYNC)
				== DMP_NOSYNC);
#endif
		}
	}
	return (0);

err_alloc_dh:
	if (dp->gc.gc_tx_buf_size > 0) {
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
	if (dp->desc_dma_handle != NULL) {
		err = ddi_dma_unbind_handle(dp->desc_dma_handle);
		ASSERT(err == DDI_SUCCESS);
		ddi_dma_mem_free(&dp->desc_acc_handle);
		ddi_dma_free_handle(&dp->desc_dma_handle);
		dp->desc_dma_handle = NULL;
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

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

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
	while ((rbp = dp->rx_buf_freelist) != NULL) {

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
 * Rx/Tx descriptor slot management
 */
/* ============================================================== */
/*
 * Initialize an empty rx ring.
 */
static void
gem_init_rx_ring(struct gem_dev *dp)
{
	int		i;
	int		rx_ring_size = dp->gc.gc_rx_ring_size;
	struct rxbuf	*rbp;

	DPRINTF(1, (CE_CONT, "!%s: %s ring_size:%d, buf_max:%d",
		dp->name, __func__,
		rx_ring_size, dp->gc.gc_rx_buf_max));

	/* make a physical chain of rx descriptors */
	for (i = 0; i < rx_ring_size; i++) {
		(*dp->gc.gc_rx_desc_init)(dp, i);
	}
	gem_rx_desc_dma_sync(dp, 0, rx_ring_size, DDI_DMA_SYNC_FORDEV);

	dp->rx_active_head = (seqnum_t) 0;
	dp->rx_active_tail = (seqnum_t) 0;

	ASSERT(dp->rx_buf_head == (struct rxbuf *) NULL);
	ASSERT(dp->rx_buf_tail == (struct rxbuf *) NULL);
}

/*
 * Prepare rx buffers and put them into the rx buffer/descriptor ring.
 */
static void
gem_prepare_rx_buf(struct gem_dev *dp)
{
	int		i;
	int		nrbuf;
	struct rxbuf	*rbp;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Now we have no active buffers in rx ring */

	nrbuf = min(dp->gc.gc_rx_ring_size, dp->gc.gc_rx_buf_max);
	for (i = 0; i < nrbuf; i++) {
		if ((rbp = gem_get_rxbuf(dp, B_TRUE)) == NULL) {
			break;
		}
		gem_append_rxbuf(dp, rbp);
	}

	gem_rx_desc_dma_sync(dp,
		0, dp->gc.gc_rx_ring_size, DDI_DMA_SYNC_FORDEV);
}

/*
 * Reclaim active rx buffers in rx buffer ring.
 */
static void
gem_clean_rx_buf(struct gem_dev* dp)
{
	int		i;
	int		ret;
	struct rxbuf	*rbp;
	int		rx_ring_size = dp->gc.gc_rx_ring_size;
#ifdef GEM_DEBUG_LEVEL
	int		total;
#endif
	ASSERT(mutex_owned(&dp->intrlock));
#if 0
	ASSERT(!dp->mac_active);
#endif
	DPRINTF(2, (CE_CONT, "!%s: %s: %d buffers are free",
		dp->name, __func__, dp->rx_buf_freecnt));
	/*
	 * clean up HW descriptors
	 */
	for (i = 0; i < rx_ring_size; i++) {
		(*dp->gc.gc_rx_desc_clean)(dp, i);
	}
	gem_rx_desc_dma_sync(dp, 0, rx_ring_size, DDI_DMA_SYNC_FORDEV);

#ifdef GEM_DEBUG_LEVEL
	total = 0;
#endif
	/*
	 * Reclaim allocated rx buffers
	 */
	while ((rbp = dp->rx_buf_head) != (struct rxbuf *) NULL) {
#ifdef GEM_DEBUG_LEVEL
		total++;
#endif
		/* remove the first one from rx buffer list */
		dp->rx_buf_head = rbp->rxb_next;
		
		/* release associated DMA resources with the rxbuf */
		ASSERT(rbp->rxb_nfrags > 0);
		ret = ddi_dma_unbind_handle(rbp->rxb_dh);
		ASSERT(ret == DDI_SUCCESS);
		rbp->rxb_nfrags = 0;

		/* release the associated mblk */
		ASSERT(rbp->rxb_mp != NULL);
		freemsg(rbp->rxb_mp);
		rbp->rxb_mp = NULL;

		/* recycle the rxbuf */
		GEM_FREE_RXBUF(rbp);
	}
	dp->rx_buf_tail = (struct rxbuf *) NULL;

	DPRINTF(2, (CE_CONT,
		"!%s: %s: %d buffers freeed, total: %d free",
		dp->name, __func__, total, dp->rx_buf_freecnt));
}

/*
 * Initialize an empty transmit buffer/descriptor ring
 */
static void
gem_init_tx_ring(struct gem_dev* dp)
{
	int		i;
	struct txbuf	*tbp;
	int		tx_buf_size = dp->gc.gc_tx_buf_size;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;

	DPRINTF(2, (CE_CONT, "!%s: %s: ring_size:%d, buf_size:%d",
		dp->name, __func__,
		dp->gc.gc_tx_ring_size, dp->gc.gc_tx_buf_size));

	ASSERT(!dp->mac_active);

	/* initialize active list and free list */
	dp->tx_slots_base =
		SLOT(dp->tx_slots_base + dp->tx_softq_head, tx_buf_size);
	dp->tx_softq_tail -= dp->tx_softq_head; 
	dp->tx_softq_head = (seqnum_t) 0;

	dp->tx_active_head = dp->tx_softq_head;
	dp->tx_active_tail = dp->tx_softq_head;

	dp->tx_free_head   = dp->tx_softq_tail;
	dp->tx_free_tail   = dp->gc.gc_tx_buf_limit;

	dp->tx_desc_head = (seqnum_t) 0;
	dp->tx_desc_tail = (seqnum_t) 0;
	dp->tx_desc_intr = (seqnum_t) 0;

	for (i = 0; i < tx_ring_size; i++) {
		(*dp->gc.gc_tx_desc_init)(dp, i);
	}
	gem_tx_desc_dma_sync(dp, 0, tx_ring_size, DDI_DMA_SYNC_FORDEV);
}
#if 0
#define	TXBUF_FREE_DMA_RESOURCES(tbp) {	\
	if ((tbp)->txb_mp != NULL) {	\
		int	i;	\
		ASSERT((tbp)->txb_dh_used > 0);	\
		i = (tbp)->txb_dh_used;	\
		while (i--) {	\
			int	ret;	\
			ret = ddi_dma_unbind_handle((tbp)->txb_dh[i]);	\
			ASSERT(ret == DDI_SUCCESS);	\
		}	\
		freemsg((tbp)->txb_mp);	\
		(tbp)->txb_mp = NULL;	\
		(tbp)->txb_dh_used = 0;	\
	}	\
	(tbp)->txb_nfrags = 0;	\
}
#else
#define	TXBUF_FREE_DMA_RESOURCES(tbp) {	\
	if ((tbp)->txb_dh_used > 0) {	\
		int	i;	\
		i = (tbp)->txb_dh_used;	\
		while (i--) {	\
			int	ret;	\
			ret = ddi_dma_unbind_handle((tbp)->txb_dh[i]);	\
			ASSERT(ret == DDI_SUCCESS);	\
		}	\
		(tbp)->txb_dh_used = 0;	\
	}	\
	if ((tbp)->txb_mp != NULL) {	\
		freemsg((tbp)->txb_mp);	\
		(tbp)->txb_mp = NULL;	\
	}	\
	(tbp)->txb_nfrags = 0;	\
}
#endif
/*
 * reclaim active tx buffers and reset positions in tx rings.
 */
static void
gem_clean_tx_buf(struct gem_dev *dp)
{
	int		i;
	int		slot;
	seqnum_t	head;
	seqnum_t	tail;
	seqnum_t	sn;
	struct txbuf	*tbp;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;
	int		err = 0;

	ASSERT(!dp->mac_active);
	ASSERT(dp->tx_busy == 0);
#ifdef notdef
	ASSERT(mutex_owned(&dp->xmitlock));
#endif
	ASSERT(dp->tx_softq_tail == dp->tx_free_head);

	/*
	 * clean up all HW descriptors
	 */
	for (i = 0; i < tx_ring_size; i++) {
		(*dp->gc.gc_tx_desc_clean)(dp, i);
	}
	gem_tx_desc_dma_sync(dp, 0, tx_ring_size, DDI_DMA_SYNC_FORDEV);

	/* dequeue all active and loaded buffers */
	head = dp->tx_active_head;
	tail = dp->tx_softq_tail;

	ASSERT(dp->tx_free_head - head >= 0);
	tbp = GET_TXBUF(dp, head);
	for (sn = head; sn != tail; sn++) {
		TXBUF_FREE_DMA_RESOURCES(tbp);
		ASSERT(tbp->txb_mp == NULL);
		dp->stats.errxmt++;
		tbp = tbp->txb_next;
	}

#ifdef GEM_DEBUG_LEVEL
	/* ensure no dma resources for tx are not in use now */
	err = 0;
	while (sn != head + dp->gc.gc_tx_buf_size) {
		if (tbp->txb_mp != NULL || tbp->txb_nfrags != 0) {
			DPRINTF(0, (CE_CONT,
				"%s: %s: sn:%d[%d] mp:%p nfrags:%d",
				dp->name, __func__,
				sn, SLOT(sn, dp->gc.gc_tx_buf_size),
				 tbp->txb_mp, tbp->txb_nfrags));
			err = 1;
		}
		sn++;
		tbp = tbp->txb_next;
	}

	if (err) {
		gem_dump_txbuf(dp, CE_WARN,
			"gem_clean_tx_buf: tbp->txb_mp != NULL");
	}
#endif
	/* recycle buffers, now no active tx buffers in the ring */
	dp->tx_free_tail += tail - head;
	ASSERT(dp->tx_free_tail == dp->tx_free_head + dp->gc.gc_tx_buf_limit);

	/* fix positions in tx buffer rings */
	dp->tx_active_head = dp->tx_free_head;
	dp->tx_active_tail = dp->tx_free_head;
	dp->tx_softq_head  = dp->tx_free_head;
	dp->tx_softq_tail  = dp->tx_free_head;
}

/*
 * Reclaim transmitted buffers from tx buffer/descriptor ring.
 */
__INLINE__ int
gem_reclaim_txbuf(struct gem_dev *dp)
{
	struct txbuf	*tbp;
	uint_t		txstat;
	int		err = GEM_SUCCESS;
	seqnum_t	head;
	seqnum_t	tail;
	seqnum_t	sn;
	seqnum_t	desc_head;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;
	int		tx_buf_size = dp->gc.gc_tx_buf_size;
	uint_t		(*tx_desc_stat)(struct gem_dev *dp,
					int slot, int ndesc);
#if GEM_DEBUG_LEVEL > 4
	clock_t			now = ddi_get_lbolt();
#endif
	ASSERT(mutex_owned(&dp->intrlock));

	mutex_enter(&dp->xmitlock);

	head = dp->tx_active_head;
	tail = dp->tx_active_tail;

	if (head == tail) {
		mutex_exit(&dp->xmitlock);
		/* no tx list */
		return (GEM_SUCCESS);
	}

	DPRINTF(2, (CE_CONT,
		"!%s: %s: testing active_head:%d[%d], active_tail:%d[%d]",
		dp->name, __func__,
		head, SLOT(head, tx_buf_size),
		tail, SLOT(tail, tx_buf_size)));
#ifdef GEM_DEBUG_LEVEL
	if (dp->tx_reclaim_busy == 0) {
		ASSERT(dp->tx_free_tail - dp->tx_active_head
				== dp->gc.gc_tx_buf_limit);
	}
#endif
	dp->tx_reclaim_busy++;

	/* sync all active HW descriptors */
	gem_tx_desc_dma_sync(dp,
			SLOT(dp->tx_desc_head, tx_ring_size),
			dp->tx_desc_tail - dp->tx_desc_head,
			DDI_DMA_SYNC_FORKERNEL);

	tx_desc_stat = dp->gc.gc_tx_desc_stat;
	tbp = GET_TXBUF(dp, head);
	desc_head = dp->tx_desc_head;
	for (sn = head; sn != tail;
			dp->tx_active_head = (++sn), tbp = tbp->txb_next) {
		int	ndescs;

		ASSERT(tbp->txb_desc == desc_head);

		ndescs = tbp->txb_ndescs;
		txstat = (*tx_desc_stat)(dp,
				SLOT(tbp->txb_desc, tx_ring_size), ndescs);

		if (txstat == 0) {
			/* not transmitted yet */
			break;
		}

		ASSERT((txstat & (GEM_TX_DONE | GEM_TX_ERR)) != 0);

		if ((txstat & GEM_TX_ERR) != 0) {
			err = GEM_FAILURE;
			cmn_err(CE_WARN, "!%s: tx error at desc %d[%d]",
				dp->name, sn, SLOT(sn, tx_ring_size));
		}
#if GEM_DEBUG_LEVEL > 4
		if (now - tbp->txb_stime >= 50) {
			cmn_err(CE_WARN, "!%s: tx delay while %d mS",
			dp->name, (now - tbp->txb_stime)*10);
		}
#endif
		/* free descriptors */
		desc_head += ndescs;
	}
	dp->tx_desc_head = desc_head;

	/* If we passed the next interrupt position, update it */
	if (dp->tx_desc_head - dp->tx_desc_intr > 0) {
		dp->tx_desc_intr = dp->tx_desc_head;
	}
	mutex_exit(&dp->xmitlock);

	/* free dma mapping resources associated with each tx buffer */
	tbp = GET_TXBUF(dp, head);
	tail = sn;
	DPRINTF(2, (CE_CONT, "%s: freeing head:%d[%d], tail:%d[%d]",
		__func__,
		head, SLOT(head, tx_buf_size),
		tail, SLOT(tail, tx_buf_size)));
	for (sn = head; sn != tail; sn++, tbp = tbp->txb_next) {
		TXBUF_FREE_DMA_RESOURCES(tbp);
	}

	/* recycle the txbuf */
	mutex_enter(&dp->xmitlock);
	if (--dp->tx_reclaim_busy == 0) {
#if GEM_DEBUG_LEVEL > 4
		tail = dp->tx_free_tail;
		tbp = GET_TXBUF(dp, tail);
		sn = 0;
		while (tail != dp->tx_active_head + dp->gc.gc_tx_buf_limit) {
			if (tbp->txb_nfrags != 0) {
				break;
			}
			ASSERT(tbp->txb_mp == NULL);
			tbp = tbp->txb_next;
			tail++;
			sn++;
		}
		dp->tx_free_tail = tail;
		ASSERT(dp->tx_active_head + dp->gc.gc_tx_buf_limit
				== dp->tx_free_tail);
#else
		dp->tx_free_tail =
			dp->tx_active_head + dp->gc.gc_tx_buf_limit;
#endif
	}
	if (!dp->mac_active) {
		cv_broadcast(&dp->tx_drain_cv);
	}

	DPRINTF(2, (CE_CONT,
		"!%s: %s: called, free_head:%d free_tail:%d(+%d) added:%d",
		dp->name, __func__,
		dp->tx_free_head, dp->tx_free_tail,
		dp->tx_free_tail - dp->tx_free_head, sn));

	mutex_exit(&dp->xmitlock);

	return (err);
}
#pragma inline (gem_reclaim_txbuf)

/*
 * Make tx descriptors
 */
static seqnum_t
gem_tx_load_descs(struct gem_dev *dp,
	seqnum_t start_slot, seqnum_t end_slot)
{
	seqnum_t	sn;
	uint32_t	flags;
	struct txbuf	*tbp;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;
	int		tx_ring_limit = dp->gc.gc_tx_ring_limit;
	int		tx_buf_limit = dp->gc.gc_tx_buf_limit;
	int		descs = dp->gc.gc_tx_max_descs_per_pkt;
	int		(*tx_desc_write)(struct gem_dev *dp, int slot,
			ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag);

	DPRINTF(1, (CE_CONT, "!%s: %s: called, slots:%d, %d, ring_limit:%d",
		dp->name, __func__, start_slot, end_slot, tx_ring_limit));

	tx_desc_write = dp->gc.gc_tx_desc_write;

	tbp = GET_TXBUF(dp, start_slot);
	flags = GEM_TXFLAG_HEAD;
	for (sn = start_slot; sn != end_slot; sn++, tbp = tbp->txb_next) {

		DPRINTF(1, (CE_CONT,
		"%s: %s: desc:%d[%d],%d[%d], descs:%d",
			dp->name, __func__,
			dp->tx_desc_head,
			SLOT(dp->tx_desc_head, tx_ring_size),
			dp->tx_desc_tail,
			SLOT(dp->tx_desc_tail, tx_ring_size),
			descs));

		if ((dp->tx_desc_tail + descs)
			- (dp->tx_desc_head + tx_ring_limit) > 0) {
			/* we don't have enough free descriptors */
			DPRINTF(1, (CE_CONT,
			"%s: %s: no resources, desc:%d[%d],%d[%d], descs:%d",
				dp->name, __func__,
				dp->tx_desc_head,
				SLOT(dp->tx_desc_head, tx_ring_size),
				dp->tx_desc_tail,
				SLOT(dp->tx_desc_tail, tx_ring_size),
				descs));
			break;
		}

		/*
		 * Guarantee to have a chance to be interrupted before
		 * tx thread will be blocked.
		 */
		if ((dp->tx_desc_intr == dp->tx_desc_head) && (
		    (HIWAT_INTR(tx_ring_limit) +
				dp->tx_desc_head - dp->tx_desc_tail < 2*descs)
			/*
			 * Schedule a tx-done interrupt before free tx
			 * descriptors will run out. To ensure we can issue
			 * a tx packet on the next time, we needs at least
			 * 2*desc here.
			 * This is the last chance to schedule tx interrupt.
			 */

		   || (HIWAT_INTR(tx_ring_limit) +
				dp->tx_desc_intr - dp->tx_desc_tail < descs)
			/*
			 * We also need at least 1*desc to continue
			 * transmitting after we will get a tx interrupt.
			 */

		   || (dp->tx_free_tail - dp->tx_free_head
				<= tx_buf_limit - HIWAT_INTR(tx_buf_limit)) )) {
			/*
			 * When a tx interrupt have not been scheduled and the
			 * number of available tx buffers is 0, we'll be
			 * blocked forever waiting for  the next tx request.
			 * This is the last chance to schedule tx interrupt.
			 */
			flags |= GEM_TXFLAG_INTR;
		}
#if GEM_DEBUG_LEVEL > 1
		if (dp->tx_cnt < 100) {
			dp->tx_cnt++;
			flags |= GEM_TXFLAG_INTR;
		}
#endif
		if (sn == end_slot - 1) {
			flags |= GEM_TXFLAG_TAIL;
		}

		/*
		 * Write tx descriptor(s)
		 */
		tbp->txb_desc = dp->tx_desc_tail;
		dp->tx_desc_tail += tbp->txb_ndescs =
			(*tx_desc_write)(dp,
					SLOT(tbp->txb_desc, tx_ring_size),
					tbp->txb_dmacookie,
					tbp->txb_nfrags, flags | tbp->txb_flag);
		if ((flags & GEM_TXFLAG_INTR) != 0) {
			dp->tx_desc_intr = dp->tx_desc_tail;
		}
		tbp->txb_stime = ddi_get_lbolt();
		flags = 0;
#ifdef GEM_CONFIG_POLLING
		dp->tx_pkt_cnt++;
#endif
	}

	return (sn);
}

/*
 * Make tx descriptors in out-of-order manner
 */
static void
gem_tx_load_descs_oo(struct gem_dev *dp,
	seqnum_t start_slot, seqnum_t end_slot, seqnum_t intr_slot,
	uint32_t flags)
{
	seqnum_t	sn;
	struct txbuf	*tbp;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;
	int		(*tx_desc_write)
				(struct gem_dev *dp, int slot,
				ddi_dma_cookie_t *dmacookie,
				int frags, uint32_t flag);
	clock_t		now = ddi_get_lbolt();

	tx_desc_write = dp->gc.gc_tx_desc_write;

	sn = start_slot;
	tbp = GET_TXBUF(dp, sn);
	do {
		if (sn == intr_slot) {
			flags |= GEM_TXFLAG_INTR;
		}

#if GEM_DEBUG_LEVEL > 1
		if (dp->tx_cnt < 100) {
			dp->tx_cnt++;
			flags |= GEM_TXFLAG_INTR;
		}
#endif
		/*
		 * Write tx descriptor(s)
		 */
		tbp->txb_desc = sn;
		tbp->txb_ndescs = (*tx_desc_write)(dp,
					SLOT(sn, tx_ring_size),
					tbp->txb_dmacookie,
					tbp->txb_nfrags, flags | tbp->txb_flag);

		tbp->txb_stime = now;
#ifdef GEM_CONFIG_POLLING
		dp->tx_pkt_cnt++;
#endif
		flags = 0;
		sn++;
		tbp = tbp->txb_next;
	} while (sn != end_slot);
}

__INLINE__
static boolean_t
gem_setup_txbuf_zerocopy(struct gem_dev *dp, mblk_t *mp, struct txbuf *tbp)
{
	mblk_t			*tp;
	ddi_dma_cookie_t	*dcp;
	int			i;
	int			dh_used;
	int			frags;

	ASSERT(tbp->txb_mp == NULL);
	/*
	 * prepare dma resources for zerocopy transmittion.
	 */
	frags   = 0;
	dh_used = 0;
	dcp     = tbp->txb_dmacookie;

	for (tp = mp; tp; tp = tp->b_cont) {
		ddi_dma_handle_t	dma_handle;
		uint_t			count;
		size_t			len;
		int			err;

		len = tp->b_wptr - tp->b_rptr;
		dma_handle = tbp->txb_dh[dh_used++];

		if (frags >= dp->gc.gc_tx_max_frags) {
			/* no room for saving entire fragment list */
			goto err_dma_range;
		}

		if ((err = ddi_dma_addr_bind_handle(
			dma_handle, NULL,
			(caddr_t)tp->b_rptr, len,
			DDI_DMA_WRITE | DDI_DMA_PARTIAL |
				((len >= 512) ? DDI_DMA_STREAMING : 0),
			DDI_DMA_DONTWAIT, NULL,
			&dcp[frags++], &count)) != DDI_DMA_MAPPED) {

			/* failed to bind dma resource to the mblk */
			if (err != DDI_DMA_PARTIAL_MAP) {
				dh_used--;
			}

			DPRINTF(0, (CE_NOTE,
				"!%s: %s: dma_bind error: %d at %dth mblk,"
				" current configuration is not suitable for tx",
				dp->name, __func__, err, dh_used));

			goto err_dma_range;
		}
#ifdef NEVER	/* solaris nv only */
		ASSERT((((ddi_dma_impl_t *) dma_handle)->dmai_rflags & DMP_NOSYNC)
				== DMP_NOSYNC);
#endif
		if (frags + count >= dp->gc.gc_tx_max_frags) {
			/* no room for saving entire fragment list */
			DPRINTF(0, (CE_NOTE,
				"!%s: %s: tx dma cookies(%d) exceeds max(%d),"
				" current configuration is not suitable for tx",
				dp->name, __func__,
				frags + count, dp->gc.gc_tx_max_frags));
			goto err_dma_range;
		}

		/* collect dma fragment list */
		while (--count > 0) {
			ddi_dma_nextcookie(dma_handle, &dcp[frags++]);
		}
	}

	DPRINTF(3, (CE_CONT,
		"!%s: %s: mp:0x%p, tbp:0x%p, dma frags:%d",
		dp->name, __func__, mp, tbp, frags));

	/*
	 * XXX - check if the assigned dma range is valid.
	 */
	for (i = 0; i < frags; i++) {

		DPRINTF(3, (CE_CONT,
			"! frag:%dth: dma_addr:0x%llx len:0x%x",
			i, dcp[i].dmac_laddress, dcp[i].dmac_size));

		if (!(dcp[i].dmac_laddress >=
			dp->gc.gc_dma_attr_txbuf.dma_attr_addr_lo &&
		   (uint64_t)(dcp[i].dmac_laddress+(dcp[i].dmac_size-1)) <=
			dp->gc.gc_dma_attr_txbuf.dma_attr_addr_hi)) {

			goto err_dma_range;
		}
	}

	/* save misc info */
	tbp->txb_mp	 = mp;
	tbp->txb_dh_used = dh_used;
	tbp->txb_nfrags  = frags;
#ifdef SANITY
	tbp->txb_ndescs  = 0;
#endif
	return (B_TRUE);

err_dma_range:
	/*
	 * This will happen when the physcal
	 * address range of the installed memory exceeds
	 * the dma addressing capability of the nic.
	 * From the view point of performance, we
	 * should switch to use the copy method
	 * always.
	 */
	dp->gc.gc_tx_copy_thresh = MAXPKTBUF(dp);

	/* free partially allocated dma resources */
	while (dh_used--) {
		ddi_dma_unbind_handle(tbp->txb_dh[dh_used]);
	}

	/* force to copy */
	return (B_FALSE);
}
#pragma inline(gem_setup_txbuf_zerocopy)

#define	FLG_VTAG	0x0000ffff
#define	FLG_DEL_VTAG	0x00010000
__INLINE__
static void
gem_setup_txbuf_copy(struct gem_dev *dp, mblk_t *mp, struct txbuf *tbp,
	uint_t flag)
{
	mblk_t			*tp;
#ifdef GEM_CONFIG_VLAN
	size_t			len;
	size_t			rest;
#endif
	size_t			off;
	size_t			min_pkt;
	caddr_t			bp;
	boolean_t		tagged = B_FALSE;

	ASSERT(tbp->txb_mp == NULL);

	/* we use bounce buffer for the packet */
	min_pkt = ETHERMIN;
	bp      = tbp->txb_buf;
	off     = 0;

#ifdef GEM_CONFIG_VLAN
	/* copy until VTAG offset */
	rest = VTAG_OFF + VTAG_SIZE;
	for (tp = mp; tp != NULL; tp = tp->b_cont) {
		len = min(tp->b_wptr - tp->b_rptr, rest);
		bcopy(tp->b_rptr, &bp[off], len);
		off  += len;
		rest -= len;
		if (rest == 0) {
			tp->b_rptr += len;
			break;
		}
	}
	ASSERT(tp != NULL);
	ASSERT(rest == 0);

	/* we have just copied vtag, see it. */
	if (GET_NET16(&bp[off - VTAG_SIZE]) == VTAG_TPID) {
		/* the packet has a vtag alreay */
		tagged = B_TRUE;
		min_pkt += VTAG_SIZE;
	}

	if ((flag & FLG_DEL_VTAG) != 0) {
		ASSERT(tagged);
		off -= VTAG_SIZE;
		ASSERT(tp != NULL);
		ASSERT(rest == 0);
		min_pkt -= VTAG_SIZE;
	}
	else if ((flag & FLG_VTAG) != 0) {
		/* insert the tag */
		ASSERT(!tagged);
		tagged = B_TRUE;
		bp[off + 0] = bp[off - VTAG_SIZE + 0];
		bp[off + 1] = bp[off - VTAG_SIZE + 1];
		bp[off + 2] = bp[off - VTAG_SIZE + 2];
		bp[off + 3] = bp[off - VTAG_SIZE + 3];

		bp[off - VTAG_SIZE + 0] = (uint8_t)(VTAG_TPID >> 8);
		bp[off - VTAG_SIZE + 1] = (uint8_t) VTAG_TPID;
		bp[off - VTAG_SIZE + 2] = (uint8_t)(flag >> 8);
		bp[off - VTAG_SIZE + 3] = (uint8_t) flag;
		off += VTAG_SIZE;
		min_pkt += VTAG_SIZE;
	}

	/* copy the rest */
	for (; tp != NULL; tp = tp->b_cont) {
		if ((len = tp->b_wptr - tp->b_rptr) > 0) {
			bcopy(tp->b_rptr, &bp[off], len);
			off += len;
		}
	}
#else
	for (tp = mp; tp != NULL; tp = tp->b_cont) {
		size_t	nbyte;
		nbyte = tp->b_wptr - tp->b_rptr;
		bcopy(tp->b_rptr, &bp[off], nbyte);
		off += nbyte;
	}
#endif

	if (off < min_pkt && (!dp->gc.gc_tx_auto_pad || tagged) ) {
		/* we extend explicitly the packet to minimum packet size. */
		bzero(&bp[off], min_pkt - off);
		off = min_pkt;
	}

	ddi_dma_sync(tbp->txb_bdh, (off_t)0, off, DDI_DMA_SYNC_FORDEV);

	tbp->txb_dmacookie[0].dmac_laddress = tbp->txb_buf_dma;
	tbp->txb_dmacookie[0].dmac_size     = off;

	DPRINTF(2, (CE_CONT,
		"!%s: %s: copy: addr:0x%llx len:0x%x, tadded:%d, min_pkt:%x",
		dp->name, __func__,
		tbp->txb_dmacookie[0].dmac_laddress,
		tbp->txb_dmacookie[0].dmac_size, tagged, min_pkt));

	/* save misc info */
	tbp->txb_mp	 = mp;
	tbp->txb_nfrags  = 1;
#ifdef SANITY
	tbp->txb_dh_used = 0;
	tbp->txb_ndescs  = 0;
#endif
}
#pragma inline(gem_setup_txbuf_copy)

__INLINE__
static void
gem_tx_start_unit(struct gem_dev *dp)
{
	seqnum_t	head;
	seqnum_t	tail;
	struct txbuf	*tbp_head;
	struct txbuf	*tbp_tail;
	int		tx_ring_size = dp->gc.gc_tx_ring_size;
	int		tx_buf_size = dp->gc.gc_tx_buf_size;

	/* update HW descriptors from soft queue */
	ASSERT(mutex_owned(&dp->xmitlock));
	ASSERT(dp->tx_softq_head == dp->tx_active_tail);

	head = dp->tx_softq_head;
	tail = dp->tx_softq_tail;

	DPRINTF(1, (CE_CONT,
		"%s: %s: called, softq %d %d[+%d], desc %d %d[+%d]",
		dp->name, __func__, head, tail, tail - head,
		dp->tx_desc_head, dp->tx_desc_tail,
		dp->tx_desc_tail - dp->tx_desc_head));

	if (head == tail) {
		DPRINTF(2, (CE_CONT,
			"%s: %s: no softq", dp->name, __func__));
		return;
	}

	if (!dp->gc.gc_tx_desc_write_oo) {
		tail = gem_tx_load_descs(dp, head, tail);
		if (head == tail) {
			DPRINTF(2, (CE_CONT,
				"%s: %s: failed to load new tx HW descs",
				dp->name, __func__));
			return;
		}
	}
	else {
		seqnum_t	intr;

		/* update only descriptor position */
#if DEBUG_LEVEL > 5
		seqnum_t 	sn;
		struct txbuf	*tbp;

		for (sn = head; sn != tail; sn++) {
			tbp = GET_TXBUF(dp, sn);
			ASSERT(tbp->txb_ndescs == 1);
			ASSERT(tbp->txb_desc == sn);
		}
#endif
		dp->tx_desc_tail = tail;
		intr = dp->tx_desc_intr +
			HIWAT_INTR(dp->gc.gc_tx_ring_limit);
		if (tail - intr >= 0) {
			dp->tx_desc_intr = intr;
		}
	}

	tbp_head = GET_TXBUF(dp, head);
	tbp_tail = GET_TXBUF(dp, tail - 1);
	ASSERT(tbp_tail->txb_desc + tbp_tail->txb_ndescs == dp->tx_desc_tail);
	dp->gc.gc_tx_start(dp,
		SLOT(tbp_head->txb_desc, tx_ring_size),
		tbp_tail->txb_desc + tbp_tail->txb_ndescs - tbp_head->txb_desc);

	/* advance softq head and active tail */
	dp->tx_softq_head = dp->tx_active_tail = tail;

#ifdef notdef
	ASSERT(dp->tx_desc_intr - dp->tx_desc_head > 0 ||
		dp->tx_softq_head == dp->tx_softq_tail);
#endif
}
#pragma inline(gem_tx_start_unit)

#ifdef GEM_DEBUG_LEVEL
static int gem_send_cnt[10];
#endif

#ifdef GEM_CONFIG_VLAN
__INLINE__
static int
gem_tx_add_vtag(struct gem_dev *dp, mblk_t **mp_headp, uint16_t vtag)
{
	mblk_t	*mp = *mp_headp;
	mblk_t	*nmp;
	uint8_t	*bp;

	ASSERT(vtag != 0);

	if (mp->b_rptr - mp->b_datap->db_base >= VTAG_SIZE) {
		ADD_VTAG(mp, vtag);
		return (GEM_SUCCESS);
	}

	if (mp->b_wptr - mp->b_rptr < VTAG_OFF + 8) {
		/* first fragment is too small to divide. */
		DPRINTF(2, (CE_CONT, "%s: pkt is small", dp->name));
		return (GEM_FAILURE);
	}

	nmp = allocb(VTAG_OFF + VTAG_SIZE, BPRI_MED);
	if (nmp == NULL) {
		/* no memory resource */
		return (GEM_FAILURE);
	}
	nmp->b_wptr = nmp->b_rptr + VTAG_OFF + VTAG_SIZE;

	bcopy(mp->b_rptr, nmp->b_rptr, VTAG_OFF);
	mp->b_rptr += VTAG_OFF;

	bp = ((uint8_t *) nmp->b_rptr) + VTAG_OFF;
	bp[0] = (uint8_t)(VTAG_TPID >> 8);
	bp[1] = (uint8_t) VTAG_TPID;
	bp[2] = (uint8_t)(vtag >> 8);
	bp[3] = (uint8_t) vtag;

	nmp->b_cont = mp;
	*mp_headp = nmp;

	DPRINTF(2, (CE_CONT, "%s: vtag added", dp->name));

	return (GEM_SUCCESS);
}
#pragma inline(gem_tx_add_vtag)
#endif /* GEM_CONFIG_VLAN */

/*
 * gem_send_common is an external function because hw depend routines can
 * use it for sending control frames like setup frame for 2114x chips.
 */
mblk_t *
gem_send_common(struct gem_dev *dp, mblk_t *mp_head, uint32_t flags)
{
	int			nmblk;
	int			avail;
	mblk_t			*tp;
	mblk_t			*mp;
	int			i;
	struct txbuf		*tbp;
	seqnum_t		head;
	seqnum_t		intr;
	uint32_t		pflags;
	uint32_t		load_flags;
	uint64_t		len_total = 0;
	uint64_t		packets = 0;
	uint32_t		vtag = 0;
	uint32_t		copy_flag;

	ASSERT(mp_head != NULL);

	mp = mp_head;
	nmblk = 1;
	while ((mp = mp->b_next) != NULL) {
		nmblk++;
	}
#ifdef GEM_DEBUG_LEVEL
	gem_send_cnt[0]++;
	gem_send_cnt[min(nmblk, 9)]++;
#endif
	/*
	 * Aquire resources
	 */
	mutex_enter(&dp->xmitlock);

	if (!dp->mac_active && (flags & GEM_SEND_CTRL) == 0) {
		/* don't send data packets while mac isn't active */
		mutex_exit(&dp->xmitlock);
		return (mp_head);
	}

	/* allocate free slots */
	head  = dp->tx_free_head;
	avail = dp->tx_free_tail - head;

	DPRINTF(2, (CE_CONT,
		"!%s: %s: called, free_head:%d free_tail:%d(+%d) req:%d",
		dp->name, __func__,
		dp->tx_free_head, dp->tx_free_tail, avail, nmblk));

	if (nmblk > avail) {
		if (avail == 0) {
			/* no resources; short cut */
			DPRINTF(2, (CE_CONT, "!%s: no resources", __func__));
			goto done;
		}
		nmblk = avail;
	}

	dp->tx_free_head = head + nmblk;
	load_flags = ((dp->tx_busy++) == 0) ? GEM_TXFLAG_HEAD : 0;

	/* next interrupt position for OO */
	intr = dp->tx_desc_intr + HIWAT_INTR(dp->gc.gc_tx_buf_limit);
	mutex_exit(&dp->xmitlock);

	tbp = GET_TXBUF(dp, head);

	i = nmblk;
	do {
		size_t		len;
		int		segs;
#ifdef i86pc
		int		frags;
#endif
		long		align;
		uint8_t		*bp;
#define	PKT_MIN_SIZE	(sizeof(struct ether_header) + 10 + VTAG_SIZE)

		/* remove one from the mblk list */
		ASSERT(mp_head != NULL);
		mp = mp_head;
		mp_head = mp_head->b_next,
		mp->b_next = NULL;

		/* save misc info */
		tbp->txb_flag =
			(flags & GEM_SEND_CTRL) << GEM_TXFLAG_PRIVATE_SHIFT;

		/*
		 * check ether packet type and ip protocol
		 */
		if (mp->b_wptr - mp->b_rptr < PKT_MIN_SIZE) {
			int 	off;
			int	len;

			/* we use bounce buffer for the packet */
			bp = (uint8_t *) tbp->txb_buf;
			for (tp = mp, off = 0;
				(tp != NULL) && (off < PKT_MIN_SIZE);
					tp = tp->b_cont, off += len) {
				len = min(tp->b_wptr - tp->b_rptr,
							PKT_MIN_SIZE - off);
				bcopy(tp->b_rptr, &bp[off], len);
			}
		}
		else {
			bp = mp->b_rptr;
		}
#undef PKT_MIN_SIZE

		copy_flag = 0;
#ifdef GEM_CONFIG_VLAN
		if (GET_NET16(&bp[VTAG_OFF]) == VTAG_TPID) {
			vtag = GET_NET16(&bp[VTAG_OFF + 2]);
			ASSERT(vtag != 0);
			if ((dp->misc_flag & GEM_VLAN_HARD) != 0) {
				tbp->txb_flag |= vtag << GEM_TXFLAG_VTAG_SHIFT;
				copy_flag |= FLG_DEL_VTAG;
			}
			/* skip vtag */
			bp += VTAG_SIZE;
			vtag = 0;
		}
		else {
			vtag = (flags & GEM_SEND_VTAG) >> GEM_SEND_VTAG_SHIFT;
			if ((dp->misc_flag & GEM_VLAN_HARD) != 0) {
				tbp->txb_flag |= vtag << GEM_TXFLAG_VTAG_SHIFT;
				vtag = 0;
			}
		}
#endif /* GEM_CONFIG_VLAN */
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
		hcksum_retrieve(mp, NULL, NULL, NULL,
					NULL, NULL, NULL, &pflags);

		if ((pflags & (HCK_PARTIALCKSUM | HCK_FULLCKSUM)) != 0) {
			int	ipproto;

			switch (GET_ETHERTYPE(bp)) {
			case ETHERTYPE_IP:
				ipproto = GET_IPTYPEv4(bp);
				if ((pflags & HCK_IPV4_HDRCKSUM) != 0) {
					tbp->txb_flag |= GEM_TXFLAG_IPv4;
				}
				break;

			case ETHERTYPE_IPV6:
				ipproto = GET_IPTYPEv6(bp);
				break;

			default:
				ipproto = IPPROTO_IP;
				break;
			}

			switch (ipproto) {
			case IPPROTO_TCP:
				tbp->txb_flag |= GEM_TXFLAG_TCP;
				break;

			case IPPROTO_UDP:
				tbp->txb_flag |= GEM_TXFLAG_UDP;
				break;
			}
		}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */

		/*
		 * determin if we should copy the packet or not.
		 */
#ifdef GEM_CONFIG_VLAN
		if ((copy_flag & FLG_DEL_VTAG) != 0) {
			ASSERT(vtag == 0);
			if (mp->b_wptr - mp->b_rptr < VTAG_OFF + 8) {
				/* use bounce buffer to remove tag */
				goto copy;
			}
			/* remove vtag directly */
			DEL_VTAG(mp);

			copy_flag &= ~FLG_DEL_VTAG;
		}

		if (vtag != 0) {
			/* try to add a new fragment to the head of mp */
			if (gem_tx_add_vtag(dp, &mp, vtag) != GEM_SUCCESS) {
				/* failed, we should use copy method */
				goto copy;
			}
			/* vtag was added successfully */
			vtag = 0;
		}
#endif /* GEM_CONFIG_VLAN */

		if ((flags & (GEM_SEND_COPY | GEM_SEND_CTRL)) != 0) {
			goto copy;
		}

		/*
		 * check the packet length and alignment for each fragment in
		 * the packet
		 */
		align = 0;
		len   = 0;
		segs  = 0;
#ifdef i86pc
		frags = 0;
#endif
		tp = mp;
		do {
			int	diff; 

			len  +=  (diff = tp->b_wptr - tp->b_rptr);
			if (diff < 8) {
				/*
				 * the fragment is too short.
				 * some nics including sis900, have
				 * the restriction.
				 */
				goto copy;
			}

			align |= (long)tp->b_rptr;
			if (tp->b_cont != NULL) {
				align |= (long)tp->b_wptr;
			}
			segs++;
#ifdef i86pc
			frags += ((PAGEOFFSET & (intptr_t)tp->b_rptr)
					+ diff + PAGEOFFSET) >> PAGESHIFT;
#endif
		} while ((tp = tp->b_cont) != NULL);

		len_total += len;
		packets++;

		if ((align & dp->gc.gc_tx_buf_align) != 0 ||
		    segs > GEM_MAXTXSEGS ||
#ifdef i86pc
		    frags > dp->gc.gc_tx_max_frags ||
#else
		    segs > dp->gc.gc_tx_max_frags ||
#endif
		    len <= dp->gc.gc_tx_copy_thresh) {
			goto copy;
		}

		/*
		 * we can use zerocopy sending method
		 */
		DPRINTF(2, (CE_CONT, "%s: %s: calling txbuf_zerocopy txbuf:%d",
			dp->name, __func__, head + i));
		if (gem_setup_txbuf_zerocopy(dp, mp, tbp)) {
			goto next;
		}

copy:
		/*
		 * we must copy the payload
		 */
		DPRINTF(2, (CE_CONT,
	"%s: %s: calling txbuf_copy txbuf:%d, flag:0x%x, orig flags:0x%x",
			dp->name, __func__, head + i, copy_flag | vtag, flags));
		gem_setup_txbuf_copy(dp, mp, tbp, copy_flag | vtag);
#ifdef NEVER
		freemsg(mp);
#endif
next:
		tbp = tbp->txb_next;
	} while (--i > 0);

	if (dp->gc.gc_tx_desc_write_oo) {
		/*
		 * we can update hw descriptors in out of order manner.
		 * XXX - tx_buf_max must equal to tx_ring_size*tx_desc_unit
		 */
		(void)gem_tx_load_descs_oo(dp,
				head, head + nmblk, intr - 1, load_flags);
	}

	/* Append the tbp at the tail of the active tx buffer list */
	mutex_enter(&dp->xmitlock);

	if ((--dp->tx_busy) == 0) {
		/* update softq positions which have been ready. */
		dp->tx_softq_tail = dp->tx_free_head;

		if (!dp->mac_active && (flags & GEM_SEND_CTRL) == 0) {
			/*
			 * The device status has changed while we were
			 * preparing tx buf.
			 * we are last one that make tx_busy.
			 * wake up someone who may wait for us.
			 */
			cv_broadcast(&dp->tx_drain_cv);
		}
		else if (!dp->tx_coalesce) {
			gem_tx_start_unit(dp);
		}
	}
	dp->stats.obytes += len_total;
	dp->stats.opackets += packets;

done:
	if (mp_head != NULL) {
		dp->tx_blocked = B_TRUE;
	}
	mutex_exit(&dp->xmitlock);

	return (mp_head);
}

/*==========================================================*/
/*
 * error detection and restart routines
 */
/*==========================================================*/
int
gem_restart_nic(struct gem_dev *dp, uint_t flags)
{
	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(1, (CE_CONT, "!%s: %s: called: tx_desc:%d %d %d",
		dp->name, __func__,
		dp->tx_active_head, dp->tx_active_tail, dp->tx_desc_intr));

	/*
	 * We should avoid calling any routines except xxx_chip_reset
	 * when we are resuming the system.
	 */
	if (dp->mac_active) {
		if ((flags & GEM_RESTART_KEEP_BUF) != 0) {
			/* stop rx gracefully */
			dp->rxmode &= ~RXMODE_ENABLE;
			(void) (*dp->gc.gc_set_rx_filter)(dp);
		}
		(void) gem_mac_stop(dp, flags);
	}

	/* reset the chip. */
	if ((*dp->gc.gc_reset_chip)(dp) != GEM_SUCCESS) {
		cmn_err(CE_WARN, "%s: %s: failed to reset chip",
			dp->name, __func__);
		goto err;
	}

	if (gem_mac_init(dp) != GEM_SUCCESS) {
		goto err;
	}

	/* enable mac address and rx filter */
	dp->rxmode |= RXMODE_ENABLE;
	if ((*dp->gc.gc_set_rx_filter)(dp) != GEM_SUCCESS) {
		goto err;
	}

	/*
	 * XXX - a panic happended because of linkdown.
	 * We must check mii_state here, because the link can be down just
	 * before the restart event happen. If the link is down now,
	 * gem_mac_start() will be called from gem_mii_link_check() when
	 * the link become up later.
	 */
	if (dp->mii_state == MII_STATE_LINKUP) {
		/* restart the nic */
		ASSERT(!dp->mac_active);
		gem_mac_start(dp);
	}
	return (GEM_SUCCESS);
err:
#ifdef GEM_CONFIG_FMA
	ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
	return (GEM_FAILURE);
}


static void
gem_tx_timeout(struct gem_dev *dp)
{
	clock_t		now;
	boolean_t	tx_sched;
	struct txbuf	*tbp;

	mutex_enter(&dp->intrlock);

	tx_sched = B_FALSE;
	now = ddi_get_lbolt();

	mutex_enter(&dp->xmitlock); 
	if (!dp->mac_active || dp->mii_state != MII_STATE_LINKUP) {
		mutex_exit(&dp->xmitlock); 
		goto schedule_next;
	}
	mutex_exit(&dp->xmitlock); 

	/* reclaim transmitted buffers to check the trasmitter hangs or not. */
	if (gem_reclaim_txbuf(dp) != GEM_SUCCESS) {
		/* tx error happended, reset transmitter in the chip */
		gem_restart_nic(dp, B_FALSE);
		tx_sched = B_TRUE;
		dp->tx_blocked = B_FALSE;

		goto schedule_next;
	}

	mutex_enter(&dp->xmitlock); 
	/* check if the transmitter is stuck */
	if (dp->tx_active_head == dp->tx_active_tail) {
		/* no tx buffer is loaded to the nic */
		mutex_exit(&dp->xmitlock); 
		goto schedule_next;
	}

	tbp = GET_TXBUF(dp, dp->tx_active_head);
	if (now - tbp->txb_stime < dp->gc.gc_tx_timeout) {
		mutex_exit(&dp->xmitlock); 
		goto schedule_next;
	}
	mutex_exit(&dp->xmitlock); 

	gem_dump_txbuf(dp, CE_WARN, __func__);

	/* discard untransmitted packet and restart tx.  */
	gem_restart_nic(dp, B_FALSE);
	tx_sched = B_TRUE;
	dp->tx_blocked = B_FALSE;

schedule_next:
	mutex_exit(&dp->intrlock);

	/* restart the downstream if needed */
	if (tx_sched) {
#ifdef GEM_CONFIG_GLDv3
		mac_tx_update(dp->mh);
#else
		gld_sched(dp->macinfo);
#endif
	}

	DPRINTF(4, (CE_CONT,
		"!%s: blocked:%d desc_head:%d desc_tail:%d desc_intr:%d",
		dp->name, dp->tx_blocked,
		dp->tx_active_head, dp->tx_active_tail, dp->tx_desc_intr));
	dp->timeout_id =
		timeout((void (*)(void *))gem_tx_timeout,
			(void *)dp, dp->gc.gc_tx_timeout_interval);
}

/*==================================================================*/
/*
 * Interrupt handler
 */
/*==================================================================*/
__INLINE__
static void
gem_append_rxbuf(struct gem_dev *dp, struct rxbuf *rbp_head)
{
	struct rxbuf	*rbp;
	seqnum_t	tail;
	int		rx_ring_size = dp->gc.gc_rx_ring_size;

	ASSERT(rbp_head != NULL);
	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(3, (CE_CONT, "!%s: %s: slot_head:%d, slot_tail:%d",
		dp->name, __func__, dp->rx_active_head, dp->rx_active_tail));

	/*
	 * Add new buffers into active rx buffer list
	 */	
	if (dp->rx_buf_head == NULL) {
		dp->rx_buf_head = rbp_head;
		ASSERT(dp->rx_buf_tail == NULL);
	}
	else {
		dp->rx_buf_tail->rxb_next = rbp_head;
	}

	tail = dp->rx_active_tail;
	for (rbp = rbp_head; rbp != NULL; rbp = rbp->rxb_next) {
		/* need to notify the tail for the lower layer */
		dp->rx_buf_tail = rbp;

		dp->gc.gc_rx_desc_write(dp,
			SLOT(tail, rx_ring_size),
			rbp->rxb_dmacookie,
			rbp->rxb_nfrags);

		dp->rx_active_tail = tail = tail + 1;
	}
}
#pragma inline(gem_append_rxbuf)

mblk_t *
gem_get_packet_default(struct gem_dev *dp, struct rxbuf *rbp, size_t len)
{
	int		rx_header_len = dp->gc.gc_rx_header_len;
	uint8_t		*bp;
	mblk_t		*mp;

	/* allocate a new mblk */
	if ((mp = allocb(len + VTAG_SIZE, BPRI_MED)) != NULL) {
		ASSERT(mp->b_next == NULL);
		ASSERT(mp->b_cont == NULL);

		mp->b_rptr += VTAG_SIZE;
		bp = mp->b_rptr;
		mp->b_wptr = bp + len;

		ddi_dma_sync(rbp->rxb_dh, rx_header_len,
				len, DDI_DMA_SYNC_FORKERNEL);

		bcopy(rbp->rxb_buf + rx_header_len, bp, len);
	}
	return (mp);
}

#ifdef GEM_DEBUG_LEVEL
uint_t	gem_rx_pkts[17];
#endif

int
gem_receive(struct gem_dev *dp)
{
	uint64_t	len_total = 0;
	struct rxbuf	*rbp;
	struct rxbuf	*nrbp;
	mblk_t		*mp;
	int		cnt = 0;
	uint64_t	rxstat;
	struct rxbuf	*newbufs;
	struct rxbuf	**newbufs_tailp;
	mblk_t		*rx_head;
	mblk_t 		**rx_tailp;
	int		rx_ring_size = dp->gc.gc_rx_ring_size;
	seqnum_t	active_head;
	uint64_t	(*rx_desc_stat)(struct gem_dev *dp,
					int slot, int ndesc);
	int		copy_thresh = dp->gc.gc_rx_copy_thresh;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	uint32_t	flags;
#endif
	uint16_t	vtag;
	int		ethermin = ETHERMIN;
	int		ethermax = dp->mtu + sizeof(struct ether_header);

	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(3, (CE_CONT, "!%s: gem_receive: rx_buf_head:%p",
			dp->name, dp->rx_buf_head));

	rx_desc_stat  = dp->gc.gc_rx_desc_stat;
	newbufs_tailp = &newbufs;
	rx_tailp      = &rx_head;
	for (active_head = dp->rx_active_head;
			(rbp = dp->rx_buf_head) != NULL; active_head++) {
		int	len;

		if (cnt == 0) {
			cnt = max(dp->poll_pkt_delay*2, 10);
			cnt = min(cnt,
				dp->rx_active_tail - active_head);
			gem_rx_desc_dma_sync(dp,
					SLOT(active_head, rx_ring_size),
					cnt,
					DDI_DMA_SYNC_FORKERNEL);
		}
		if (((rxstat = (*rx_desc_stat)(dp,
				SLOT(active_head, rx_ring_size),
				rbp->rxb_nfrags))
					& (GEM_RX_DONE | GEM_RX_ERR)) == 0) {
			/* not received yet */
			break;
		}

		/* Remove the head of the rx buffer list */
		dp->rx_buf_head = rbp->rxb_next;
		cnt--;

		/* first of all, invalidate pointer to new rx buffer */
		nrbp = NULL;

		if ((rxstat & GEM_RX_ERR) != 0) {
			nrbp = rbp;
			goto next;
		}

		len = rxstat & GEM_RX_LEN;

		DPRINTF(3, (CE_CONT, "!%s: %s: rxstat:0x%x, len:0x%x",
				dp->name, __func__, rxstat, len));

		if (len > copy_thresh) {
			/*
			 * try to allocate a new rx buffer
			 */
			nrbp = gem_get_rxbuf(dp, B_FALSE);
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
			if ((mp = dp->gc.gc_get_packet(dp, rbp, len)) == NULL) {
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
			GEM_FREE_RXBUF(rbp);
		}

#ifdef GEM_CONFIG_VLAN
		/*
		 * Process VLAN tag
		 */
		ethermin = ETHERMIN;
		ethermax = dp->mtu + sizeof(struct ether_header);
		vtag = (rxstat & GEM_RX_VTAG) >> GEM_RX_VTAG_SHIFT;
#  ifdef GEM_CONFIG_GLDv3
		if (vtag != 0) {
			/* insert vlan vtag extracted by the hardware */
			ADD_VTAG(mp, vtag);
			len += VTAG_SIZE;
			ethermax += VTAG_SIZE;
		}
		else if (GET_NET16(mp->b_rptr + VTAG_OFF) == VTAG_TPID) {
			ethermax += VTAG_SIZE;
		}
#  else /* GLDv2 */
		if (vtag != 0) {
			/* vtag has been removed by nic hardware. */
			ethermin -= VTAG_SIZE;
		}
		else if (GET_NET16(mp->b_rptr + VTAG_OFF) == VTAG_TPID) {
			/* remove vtag from the packet */
			vtag = GET_NET16(mp->b_rptr + VTAG_OFF + 2);
			DEL_VTAG(mp);
			len -= VTAG_SIZE;
			ethermin -= VTAG_SIZE;
		}
		/* save vtag at the head of the packet */
		((uint16_t *) mp->b_rptr)[-1] = vtag;
#  endif /* GEM_CONFIG_GLDv3 */
#endif /* GEM_CONFIG_VLAN */

		/* check packet size */
		if (len < ethermin) {
			dp->stats.errrcv++;
			dp->stats.runt++;
			freemsg(mp);
			goto next;
		}

		if (len > ethermax) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
			freemsg(mp);
			goto next;
		}

		len_total += len;

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
		flags = 0;
		if ((rxstat & (GEM_RX_CKSUM_TCP | GEM_RX_CKSUM_UDP)) != 0) {
			flags |= HCK_FULLCKSUM | HCK_FULLCKSUM_OK;
		}
		if ((rxstat & GEM_RX_CKSUM_IPv4) != 0) {
			flags |= HCK_IPV4_HDRCKSUM ;
		}
		if (flags != 0) {
			hcksum_assoc(mp, NULL, NULL, 0, 0, 0, 0, flags, 0);
		}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
		/* append received packet to temporaly rx buffer list */
		*rx_tailp = mp;
		rx_tailp  = &mp->b_next;
next:
		ASSERT(nrbp != NULL);

		/* append new one to temporal new buffer list */
		*newbufs_tailp = nrbp;
		newbufs_tailp  = &nrbp->rxb_next;
	}

	/* advance rx_active_head */
	if ((cnt = active_head - dp->rx_active_head) > 0) {
#ifdef GEM_CONFIG_POLLING
		/* update rx statistics for polling */
		dp->rx_pkt_cnt += cnt;

		/* count number of valid rx interrupts */
		dp->rx_intr_cnt++;
#endif
		dp->stats.rbytes += len_total;
		dp->stats.rpackets += cnt;
	}
	dp->rx_active_head = active_head;

	/* terminate the working lists */
	*newbufs_tailp = NULL;
	*rx_tailp = NULL;

	if (dp->rx_buf_head == NULL) {
		dp->rx_buf_tail = NULL;
	}

	DPRINTF(4, (CE_CONT, "%s: %s: cnt:%d, rx_head:%p",
		dp->name, __func__, cnt, rx_head));

	if (newbufs != NULL) {
		/*
		 * fillfull rx list with new buffers
		 */
		seqnum_t	head;

		/* save current tail */
		head = dp->rx_active_tail;
		gem_append_rxbuf(dp, newbufs);

		/* call hw depend start routine if we have. */
		if (dp->gc.gc_rx_start != NULL) {
			dp->gc.gc_rx_start(dp,
					SLOT(head, rx_ring_size),
					dp->rx_active_tail - head);
		}
		else {
			gem_rx_desc_dma_sync(dp,
				SLOT(head, rx_ring_size),
				dp->rx_active_tail - head,
				DDI_DMA_SYNC_FORDEV);
		}
	}

	if (rx_head != NULL) {
		/*
		 * send up received packets
		 */
		mutex_exit(&dp->intrlock);
#ifdef GEM_CONFIG_GLDv3
		mac_rx(dp->mh, dp->mac_rx_ring_ha, rx_head);
#else
		while ((mp = rx_head) != NULL) {
			rx_head = rx_head->b_next;
			mp->b_next = NULL;
# ifdef GEM_CONFIG_VLAN
			gld_recv_tagged(dp->macinfo, mp,
			    (VTAG_TPID << 16) | ((uint16_t *) mp->b_rptr)[-1]);
# else
			gld_recv(dp->macinfo, mp);
# endif
		}
#endif /* GEM_CONFIG_GLDv3 */
		mutex_enter(&dp->intrlock);
	}

#ifdef GEM_DEBUG_LEVEL
	gem_rx_pkts[min(cnt, sizeof(gem_rx_pkts)/sizeof(uint_t)-1)]++;
#endif
	return (cnt);
}

boolean_t
gem_tx_done(struct gem_dev *dp)
{
	struct txbuf		*tbp;
	boolean_t		tx_sched = B_FALSE;
#if 0
	boolean_t		intr_sched = B_FALSE;

	mutex_enter(&dp->xmitlock);
	if (dp->tx_desc_intr - dp->tx_desc_head > 0) {
		intr_sched = B_TRUE;
	}
	mutex_exit(&dp->xmitlock);
#endif
	if (gem_reclaim_txbuf(dp) != GEM_SUCCESS) {
		gem_restart_nic(dp, B_TRUE);
		DPRINTF(2, (CE_CONT, "!%s: gem_tx_done: tx_desc: %d %d",
			dp->name, dp->tx_active_head, dp->tx_active_tail));
		tx_sched = B_TRUE;
		goto x;
	}

	mutex_enter(&dp->xmitlock);

	/* fill new tx requests */
	gem_tx_start_unit(dp);

	/*
	 * if we won't have chance to get more free tx buffers, and blocked,
	 * it is worth to reschedule the downstream i.e. tx side.
	 */
#if 0
	/* XXX - tx hang under netperf UDP_STREAM tests. */
	if (dp->tx_blocked && intr_sched &&
	    (dp->tx_desc_intr == dp->tx_desc_head)) {
		tx_sched = B_TRUE;
		dp->tx_blocked = B_FALSE;
	}
#else
	if (dp->tx_blocked && (dp->tx_desc_intr == dp->tx_desc_head)) {
		tx_sched = B_TRUE;
		dp->tx_blocked = B_FALSE;
	}
#endif
	mutex_exit(&dp->xmitlock);

	DPRINTF(3, (CE_CONT, "!%s: gem_tx_done: ret: blocked:%d",
		dp->name, dp->tx_blocked));
x:
	return (tx_sched);
}

static u_int
gem_intr(struct gem_dev	*dp)
{
	u_int		ret;
#ifdef GEM_CONFIG_POLLING
	clock_t		now;
#endif

	mutex_enter(&dp->intrlock);
	dp->intr_busy = B_TRUE;

	ret = (*dp->gc.gc_interrupt)(dp);

	if (ret == DDI_INTR_UNCLAIMED) {
		dp->intr_busy = B_FALSE;
		mutex_exit(&dp->intrlock);
		return (ret);
	}

	if (!dp->mac_active) {
		cv_broadcast(&dp->tx_drain_cv);
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

		if ((dp->misc_flag & GEM_POLL_RXONLY) != 0) {
			pkts = dp->rx_pkt_cnt;
		} else {
			pkts = max(dp->tx_pkt_cnt, dp->rx_pkt_cnt);
		}

		if (now == dp->last_intr_time + 1 &&
		    pkts > dp->poll_pkt_hiwat) {
			/*
			 * calculate recommended polling interval
			 */
			dp->poll_interval = 10000 / dp->poll_pkt_hiwat;
								/* in 1uS */
			DPRINTF(2, (CE_CONT,
				"%s: now polling changed to %d uS",
				dp->name, dp->poll_interval));
		}
		else {
			/* normal mode */
			dp->poll_interval = 0;
			dp->tx_coalesce = B_FALSE;
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
	dp->intr_busy = B_FALSE;

	mutex_exit(&dp->intrlock);

#ifdef GEM_CONFIG_GLDv3
	if ((ret & INTR_RESTART_TX) != 0) {
		DPRINTF(4, (CE_CONT, "!%s: calling mac_tx_update", dp->name));
		mac_tx_update(dp->mh);
		ret &= ~INTR_RESTART_TX;
	}
#endif
	return (ret);
}

static void
gem_intr_watcher(struct gem_dev *dp)
{
#ifdef GEM_CONFIG_GLDv3
	gem_intr(dp);
#else
	(*dp->macinfo->gldm_intr)(dp->macinfo);
#endif
next:
	/* schedule next call of tu_intr_watcher */
	dp->intr_watcher_id =
		timeout((void (*)(void *))gem_intr_watcher, (void *)dp, 1);
}

/* ======================================================================== */
/*
 * MII support routines
 */
/* ======================================================================== */
uint16_t
gem_mii_read(struct gem_dev *dp, uint_t reg)
{
	if (!dp->no_preamble) {
		(*dp->gc.gc_mii_sync)(dp);
	}
	return ((*dp->gc.gc_mii_read)(dp, reg));
}

void
gem_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	if (!dp->no_preamble) {
		(*dp->gc.gc_mii_sync)(dp);
	}
	(*dp->gc.gc_mii_write)(dp, reg, val);
}

#define	fc_cap_decode(x)	\
	((((x) & MII_ABILITY_PAUSE) != 0 ? 1 : 0) |	\
	 (((x) & MII_ABILITY_ASM_DIR) != 0 ? 2 : 0))

int
gem_mii_config_default(struct gem_dev *dp)
{
	uint16_t	mii_stat;
	uint16_t	xstat;
	uint16_t	val;
	static uint16_t fc_cap_encode[4] = {
		/* none */		0,
		/* symmetric */		MII_ABILITY_PAUSE,
		/* tx */		MII_ABILITY_ASM_DIR,
		/* rx-symmetric */	MII_ABILITY_PAUSE | MII_ABILITY_ASM_DIR,
	};

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * Set advertisement register
	 */
	mii_stat = gem_mii_read(dp, MII_STATUS);

	DPRINTF(1, (CE_CONT, "!%s: gem_mii_config: MII_STATUS reg:%b",
		dp->name, mii_stat, MII_STATUS_BITS));

	if ((mii_stat & MII_STATUS_ABILITY) == 0) {
		/* it's funny */
		cmn_err(CE_WARN, "!%s: wrong ability bits: mii_status:%b",
			dp->name, mii_stat, MII_STATUS_BITS);
		return (GEM_FAILURE);
	}

	/* Do not change the rest of ability bits in advert reg */
	val = gem_mii_read(dp, MII_AN_ADVERT) & ~MII_ABILITY;

	DPRINTF(1, (CE_CONT,
		"!%s: %s: 100T4:%d 100F:%d 100H:%d 10F:%d 10H:%d",
		dp->name, __func__,
		dp->anadv_100t4, dp->anadv_100fdx, dp->anadv_100hdx,
		dp->anadv_10fdx, dp->anadv_10hdx));

	if ((mii_stat & MII_STATUS_100_BASE_T4) != 0 && dp->anadv_100t4) {
		val |= MII_ABILITY_100BASE_T4;
	}
	if ((mii_stat & MII_STATUS_100_BASEX_FD) != 0 && dp->anadv_100fdx) {
		val |= MII_ABILITY_100BASE_TX_FD;
	}
	if ((mii_stat & MII_STATUS_100_BASEX) != 0 && dp->anadv_100hdx) {
		val |= MII_ABILITY_100BASE_TX;
	}
	if ((mii_stat & MII_STATUS_10_FD) != 0 && dp->anadv_10fdx) {
		val |= MII_ABILITY_10BASE_T_FD;
	}
	if ((mii_stat & MII_STATUS_10) != 0 && dp->anadv_10hdx) {
		val |= MII_ABILITY_10BASE_T;
	}

	/* set flow control capability */
	val |= fc_cap_encode[dp->gc.gc_flow_control];

	DPRINTF(1, (CE_CONT,
		"!%s: %s: setting MII_AN_ADVERT reg:%b, mii_mode:%d, fc:%d",
		dp->name, __func__, val, MII_ABILITY_BITS, dp->gc.gc_mii_mode,
		dp->gc.gc_flow_control));

	gem_mii_write(dp, MII_AN_ADVERT, val);

	if ((mii_stat & MII_STATUS_XSTATUS) != 0) {
		/*
		 * 1000Base-T GMII support
		 */
		val = gem_mii_read(dp, MII_1000TC) &
				~(MII_1000TC_ADV_FULL | MII_1000TC_ADV_HALF);
		xstat = gem_mii_read(dp, MII_XSTATUS);
		dp->mii_xstatus = xstat;
		if ((xstat & MII_XSTATUS_1000BASET_FD) != 0 &&
		     dp->anadv_1000fdx) {
			val |= MII_1000TC_ADV_FULL;
		}
		if ((xstat & MII_XSTATUS_1000BASET) != 0 &&
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

#ifdef GEM_CONFIG_GLDv3
#define	GEM_LINKUP(dp)		mac_link_update((dp)->mh, LINK_STATE_UP)
#define	GEM_LINKDOWN(dp)	mac_link_update((dp)->mh, LINK_STATE_DOWN)
#elif defined(SOLARIS10)
#define	GEM_LINKUP(dp)		gld_linkstate((dp)->macinfo, GLD_LINKSTATE_UP)
#define	GEM_LINKDOWN(dp)	gld_linkstate((dp)->macinfo, GLD_LINKSTATE_DOWN)
#else
#define	GEM_LINKUP(dp)
#define	GEM_LINKDOWN(dp)
#endif
	
boolean_t
gem_mii_link_check(struct gem_dev *dp)
{
	clock_t		next;
	int		old_mii_state;
	boolean_t	tx_sched;
	uint16_t	status;
	uint16_t	advert;
	uint16_t	lpable;
	uint16_t	exp;
	uint16_t	ctl1000;
	uint16_t	stat1000;
	uint16_t	val;
	clock_t		now;
	clock_t		diff;
	int		linkdown_action;
	boolean_t	fix_phy = B_FALSE;

	now = ddi_get_lbolt();

	DPRINTF(3, (CE_CONT, "!%s: %s: time:%d state:%d",
			dp->name, __func__, now, dp->mii_state));

	diff = now - dp->mii_last_check;
	dp->mii_last_check = now;
	old_mii_state = dp->mii_state;

next_nowait:
	switch (dp->mii_state) {
	case MII_STATE_UNKNOWN:
		/* power-up, DP83840 requires 32 sync bits */
		(*dp->gc.gc_mii_sync)(dp);
		goto reset_phy;

	case MII_STATE_RESETTING:
		dp->mii_timer -= diff;
		if (dp->mii_timer > 0) {
			/* don't read phy registers in resetting */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/* Timer expired, ensure reset bit is not set */

		if (dp->no_preamble) {
			/* some phys need sync bits after reset */
			(*dp->gc.gc_mii_sync)(dp);
		}
		if (((val = gem_mii_read(dp, MII_CONTROL)) & MII_CONTROL_RESET)
				!= 0) {
			cmn_err(CE_NOTE,
				"!%s: time:%d resetting phy not complete."
				" mii_control:0x%b",
				dp->name, ddi_get_lbolt(),
				val, MII_CONTROL_BITS);
		}

		/* ensure neither isolated nor pwrdown nor auto-nego mode */
		/* XXX -- this operation is required for NS DP83840A. */
		gem_mii_write(dp, MII_CONTROL, 0);

#ifdef GEM_CONFIG_GLDv3
		mac_link_update(dp->mh, LINK_STATE_DOWN);
#endif
		/* As resetting PHY has completed, configure PHY registers */
		if ((*dp->gc.gc_mii_config)(dp) != 0) {
			/* we failed to configure PHY. */
			goto reset_phy;
		}

		if (dp->mii_fixedmode) {
			/* skip auto-negotiation phase */
			dp->mii_state = MII_STATE_MEDIA_SETUP;
			dp->mii_timer = 0;
			dp->mii_interval = 0;
			goto next_nowait;
		}

		/* Issue auto-negotiation command */
		goto autonego;

	case MII_STATE_AUTONEGOTIATING:
		/*
		 * Autonegotiation is in progress
		 */
		dp->mii_timer -= diff;
		if (dp->mii_timer -
		      (dp->gc.gc_mii_an_timeout - dp->gc.gc_mii_an_wait) > 0) {
			/*
			 * wait for a while, typically autonegotiation
			 * completes in 2.3 - 2.5 sec.
			 */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/* read PHY status */
		status = gem_mii_read(dp, MII_STATUS);
		DPRINTF(4, (CE_CONT,
			"!%s: %s: called: mii_state:%d MII_STATUS reg:%b",
			dp->name, __func__, dp->mii_state,
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
				 * Auto-negotiation was timed out,
				 * try again w/o resetting phy.
				 */
				if (!dp->mii_supress_msg) {
					cmn_err(CE_WARN,
					"!%s: auto-negotiation failed: timeout",
					dp->name);
					dp->mii_supress_msg = B_TRUE;
				}
				goto autonego;
			}
			/*
			 * Auto-negotiation is in progress. Wait.
			 */
			dp->mii_interval = dp->gc.gc_mii_an_watch_interval;
			goto next;
		}

		/*
		 * Auto-negotiation have completed.
		 * Assume linkdown and fall through.
		 */
		dp->mii_supress_msg = B_FALSE;
		dp->mii_state = MII_STATE_AN_DONE;
		DPRINTF(0, (CE_CONT,
			"!%s: auto-negotiation completed, MII_STATUS:%b",
			dp->name, status, MII_STATUS_BITS));

		if (dp->gc.gc_mii_an_delay > 0) {
			dp->mii_timer = dp->gc.gc_mii_an_delay;
			dp->mii_interval = drv_usectohz(20*1000);
			goto next;
		}

		dp->mii_timer = 0;
		diff = 0;
		goto next_nowait;

	case MII_STATE_AN_DONE:
		/*
		 * Auto-negotiation have done. Now we can set up media.
		 */
		dp->mii_timer -= diff;
		if (dp->mii_timer > 0) {
			/* wait for a while */
			dp->mii_interval = WATCH_INTERVAL_FAST;
			goto next;
		}

		/*
		 * set up the result of auto negotiation 
		 */

		/*
		 * Read registers required to determin current
		 * duplex mode and media speed.
		 */
		if (dp->gc.gc_mii_an_delay > 0) {
			/*
			 * As the link watcher context has been suspended,
			 * 'status' is invalid. We must status register here
			 */
			status = gem_mii_read(dp, MII_STATUS);
		}
		advert   = gem_mii_read(dp, MII_AN_ADVERT);
		lpable   = gem_mii_read(dp, MII_AN_LPABLE);
		exp      = gem_mii_read(dp, MII_AN_EXPANSION);
		if (exp == 0xffff) {
			/* some phys don't have exp register */
			exp = 0;
		}
		ctl1000  = 0;
		stat1000 = 0;
		if ((status & MII_STATUS_XSTATUS) != 0) {
			ctl1000  = gem_mii_read(dp, MII_1000TC);
			stat1000 = gem_mii_read(dp, MII_1000TS);
		}
		dp->mii_lpable   = lpable;
		dp->mii_advert   = advert;
		dp->mii_exp      = exp;
		dp->mii_ctl1000  = ctl1000;
		dp->mii_stat1000 = stat1000;

		cmn_err(CE_CONT,
		"!%s: auto-negotiation done, advert:%b, lpable:%b, exp:%b",
			dp->name,
			advert, MII_ABILITY_BITS,
			lpable, MII_ABILITY_BITS,
			exp, MII_AN_EXP_BITS);

		if ((status & MII_STATUS_XSTATUS) != 0) {
			cmn_err(CE_CONT,
				"! MII_1000TC:%b, MII_1000TS:%b",
				ctl1000, MII_1000TC_BITS,
				stat1000, MII_1000TS_BITS);
		}

		if (gem_population(lpable) <= 1 &&
		   (exp & MII_AN_EXP_LPCANAN) == 0) {
			if ((advert & MII_ABILITY) != lpable) {
				cmn_err(CE_WARN,
	"!%s: but the link partnar doesn't seem to have auto-negotiation "
	"capability. please check the link configuration.",
					dp->name);
			}
			/*
			 * it should be result of pararell detection, which
			 * cannot detect duplex mode.
			 */
			if ((lpable & MII_ABILITY_100BASE_TX) != 0) {
				/*
				 * we prefer full duplex mode for 100Mbps
				 * connection, if we can.
				 */
				lpable |= advert & MII_ABILITY_100BASE_TX_FD;
			}

			if ((advert & lpable) == 0 &&
			    (lpable & MII_ABILITY_10BASE_T) != 0) {
				lpable |= advert & MII_ABILITY_10BASE_T_FD;
			}
			/*
			 * as the link partnar isn't auto-negotiatable, use
		 	 * fixed mode temporally.
			 */
			fix_phy = B_TRUE;
		}
		else if (lpable == 0) {
			cmn_err(CE_WARN, "!%s: wrong lpable.", dp->name);
			goto reset_phy;
		}

		/*
		 * configure current link mode according to AN priority.
		 */	
		val = advert & lpable;
		if ((ctl1000 & MII_1000TC_ADV_FULL) != 0 &&
		    (stat1000 & MII_1000TS_LP_FULL) != 0) {
			/* 1000BaseT & full duplex */
			dp->speed	 = GEM_SPD_1000;
			dp->full_duplex  = B_TRUE;
		}
		else if ((ctl1000 & MII_1000TC_ADV_HALF) != 0 &&
			 (stat1000 & MII_1000TS_LP_HALF) != 0) {
			/* 1000BaseT & half duplex */
			dp->speed	 = GEM_SPD_1000;
			dp->full_duplex  = B_FALSE;
		}
		/* 100BaseTX & full duplex: not supported */
		else if ((val & MII_ABILITY_100BASE_TX_FD) != 0) {
			/* 100BaseTx & full duplex */
			dp->speed	 = GEM_SPD_100;
			dp->full_duplex  = B_TRUE;
		}
		else if ((val & MII_ABILITY_100BASE_T4) != 0) {
			/* 100BaseT4 & full duplex */
			dp->speed	 = GEM_SPD_100;
			dp->full_duplex  = B_TRUE;
		}
		/* 100BaseTX & half duplex: not supported */
		else if ((val & MII_ABILITY_100BASE_TX) != 0) {
			/* 100BaseTx & half duplex */
			dp->speed	 = GEM_SPD_100;
			dp->full_duplex  = B_FALSE;
		}
		else if ((val & MII_ABILITY_10BASE_T_FD) != 0) {
			/* 10BaseT & full duplex */
			dp->speed	 = GEM_SPD_10;
			dp->full_duplex  = B_TRUE;
		}
		else if ((val & MII_ABILITY_10BASE_T) != 0) {
			/* 10BaseT & half duplex */
			dp->speed	 = GEM_SPD_10;
			dp->full_duplex  = B_FALSE;
		}
		else {
			/*
			 * It seems that the link partnar doesn't have
			 * auto-negotiation capability and our PHY
			 * could not report the correct current mode.
			 * We guess current mode by mii_control register.
			 */
			val = gem_mii_read(dp, MII_CONTROL);

			/* select 100m full or 10m half */
			dp->speed = ((val & MII_CONTROL_100MB) != 0)
						? GEM_SPD_100 : GEM_SPD_10;
			dp->full_duplex = dp->speed != GEM_SPD_10;
			fix_phy = B_TRUE;

			cmn_err(CE_NOTE,
				"!%s: auto-negotiation done but "
				"common ability not found.\n"
				"PHY state: control:%b advert:%b lpable:%b\n"
				"guessing %d Mbps %s duplex mode",
				dp->name,
				val, MII_CONTROL_BITS,
				advert, MII_ABILITY_BITS,
				lpable, MII_ABILITY_BITS,
				gem_speed_value[dp->speed],
				dp->full_duplex ? "full" : "half");
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
		/* fall down */

	case MII_STATE_MEDIA_SETUP:
		if (dp->mii_fixedmode || dp->gc.gc_mii_an_oneshot || fix_phy) {
			/*
			 * write specified mode to phy.
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

		if (dp->nic_state >= NIC_STATE_INITIALIZED) {
			/* notify the result of auto-negotiation to mac */
			(*dp->gc.gc_set_media)(dp);
		}

		if ((void *)(dp->gc.gc_mii_tune_phy) != NULL) {
			/* for built-in sis900 */
			/* XXX - this code should be removed.  */
			(*dp->gc.gc_mii_tune_phy)(dp);
		}

		dp->mii_state = MII_STATE_LINKDOWN;
		dp->mii_timer = dp->gc.gc_mii_linkdown_timeout;
		DPRINTF(2, (CE_CONT, "!%s: setup midia mode done", dp->name));
		dp->mii_supress_msg = B_FALSE;

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
			dp->mii_supress_msg = B_FALSE;

			DPRINTF(0, (CE_CONT,
				"!%s: link up detected: mii_stat:%b",
				dp->name, status, MII_STATUS_BITS));

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

			/* XXX - we need other timer to watch statictics */
			if (dp->gc.gc_mii_hw_link_detection &&
			    dp->nic_state == NIC_STATE_ONLINE) {
				dp->mii_interval = 0;
			}

			if (dp->nic_state == NIC_STATE_ONLINE &&
			    !dp->mac_active) {
				gem_mac_start(dp);
			}

			GEM_LINKUP(dp);
			goto next;
		}
		GEM_LINKDOWN(dp);

		dp->mii_supress_msg = B_TRUE;
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
				"!%s: link down detected: mii_stat:%b",
				dp->name, status, MII_STATUS_BITS);

			if (dp->nic_state == NIC_STATE_ONLINE &&
			    dp->mac_active &&
			    dp->gc.gc_mii_stop_mac_on_linkdown) {
				(void) gem_mac_stop(dp, 0);
			}

			GEM_LINKDOWN(dp);

			if (!dp->mii_fixedmode) {
				/* need to restart auto-negotiation */
				linkdown_action = dp->gc.gc_mii_linkdown_action;
				goto restart_autonego;
			}

			dp->mii_state = MII_STATE_LINKDOWN;
			dp->mii_timer = dp->gc.gc_mii_linkdown_timeout;

			if ((void *)(dp->gc.gc_mii_tune_phy) != NULL) {
				/* for built-in sis900 */
				(*dp->gc.gc_mii_tune_phy)(dp);
			}
			dp->mii_interval = dp->gc.gc_mii_link_watch_interval;
			goto next;
		}
		GEM_LINKUP(dp);
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
		dp->mii_supress_msg = B_TRUE;
		goto reset_phy;

	case MII_ACTION_NONE:
		dp->mii_supress_msg = B_TRUE;
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
		dp->mii_supress_msg = B_TRUE;
		goto autonego;

	default:
		cmn_err(CE_WARN, "!%s: unknowm linkdown action: %d",
			dp->name, dp->gc.gc_mii_linkdown_action);
		dp->mii_supress_msg = B_TRUE;
	}
	/* NOTREACHED */

reset_phy:
	if (dp->nic_state == NIC_STATE_ONLINE) {
		GEM_LINKDOWN(dp);
	}
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
	if (dp->nic_state == NIC_STATE_ONLINE) {
		GEM_LINKDOWN(dp);
	}
	if (!dp->mii_supress_msg) {
		cmn_err(CE_CONT, "!%s: auto-negotiation started", dp->name);
	}
	dp->mii_state = MII_STATE_AUTONEGOTIATING;
	dp->mii_timer = dp->gc.gc_mii_an_timeout;

	/* start/restart auto nego */
	val = gem_mii_read(dp, MII_CONTROL) &
		~(MII_CONTROL_ISOLATE | MII_CONTROL_PWRDN | MII_CONTROL_RESET);
#if 1
	if ((val & MII_CONTROL_ANE) != 0) {
		/* restart auto nego */
		gem_mii_write(dp, MII_CONTROL, val | MII_CONTROL_RSAN);
	}
	else {
		/* enable auto nego */
		/* XXX - it doesn't worf for mx98315 */
		gem_mii_write(dp, MII_CONTROL, val | MII_CONTROL_ANE);
	}
#else
	gem_mii_write(dp, MII_CONTROL,
			val | MII_CONTROL_ANE | MII_CONTROL_RSAN);
#endif
	dp->mii_interval = dp->gc.gc_mii_an_watch_interval;

next:
	if (dp->link_watcher_id == 0 && dp->mii_interval != 0) {
		/* we must schedule next mii_watcher */
		dp->link_watcher_id =
			timeout((void (*)(void *))& gem_mii_link_watcher,
				(void *)dp, dp->mii_interval);
	}

	if (tx_sched = (old_mii_state != MII_STATE_LINKUP) &&
		   (dp->mii_state == MII_STATE_LINKUP) && dp->tx_blocked) {
		dp->tx_blocked = B_FALSE;
	}

	return (tx_sched);
}

static void
gem_mii_link_watcher(struct gem_dev *dp)
{
	boolean_t	tx_sched;

	mutex_enter(&dp->intrlock);

	dp->link_watcher_id = 0;
	tx_sched = gem_mii_link_check(dp);
#if GEM_DEBUG_LEVEL > 2
	if (dp->link_watcher_id == 0) {
		cmn_err(CE_CONT, "%s: link watcher stopped", dp->name);
	}
#endif
	mutex_exit(&dp->intrlock);

	if (tx_sched) {
		/* kick potentially stopped downstream */
#ifdef GEM_CONFIG_GLDv3
		mac_tx_update(dp->mh);
#else
		gld_sched(dp->macinfo);
#endif
	}
}

int
gem_mii_init_default(struct gem_dev *dp)
{
	int		phy;
	uint16_t	status;
	uint16_t	Xstatus;

	DPRINTF(3, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * Scan PHY
	 */
	dp->no_preamble = B_FALSE;

	/* Try default phy first */
	if (dp->mii_phy_addr != 0) {
		status = gem_mii_read(dp, MII_STATUS);
		if (status != 0xffff && status != 0x0000) {
			gem_mii_write(dp, MII_CONTROL, 0);
			goto PHY_found;
		}

		if (dp->mii_phy_addr < 0) {
			cmn_err(CE_NOTE,
		"!%s: failed to probe default internal and/or non-MII PHY",
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
			gem_mii_write(dp, MII_CONTROL, 0);
			goto PHY_found;
		}
	}

	for (phy = dp->gc.gc_mii_addr_min; phy < 32; phy++) {
		dp->mii_phy_addr = phy;
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
	dp->mii_status = status;
	dp->mii_phy_id  = (gem_mii_read(dp, MII_PHYIDH) << 16)
			|  gem_mii_read(dp, MII_PHYIDL);

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

	if ((status & MII_STATUS_XSTATUS) != 0) {
		cmn_err(CE_CONT, "!%s: xstatus:%b",
			dp->name,
			gem_mii_read(dp, MII_XSTATUS), MII_XSTATUS_BITS);
	}

	if (!dp->mii_fixedmode && (status & MII_STATUS_CANAUTONEG) == 0) {
		dp->mii_fixedmode = B_TRUE;
		/* fix speed and duplex mode. check half duplex first */
		if ((status & MII_STATUS_10) != 0) {
			dp->speed	= GEM_SPD_10;
			dp->full_duplex = B_FALSE;
		}
		else if ((status & MII_STATUS_100_BASEX) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = B_FALSE;
		}
		else if ((status & MII_STATUS_10_FD) != 0) {
			dp->speed	= GEM_SPD_10;
			dp->full_duplex = B_TRUE;
		}
		else if ((status & MII_STATUS_100_BASEX_FD) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = B_TRUE;
		}
		else if ((status & MII_STATUS_100_BASE_T4) != 0) {
			dp->speed	= GEM_SPD_100;
			dp->full_duplex = B_FALSE;
		}
	}

	return (GEM_SUCCESS);
}

static void
gem_mii_start(struct gem_dev *dp)
{
	DPRINTF(3, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* make a first call of check link */
	dp->mii_state = MII_STATE_UNKNOWN;
	gem_mii_link_watcher(dp);
}

static void
gem_mii_stop(struct gem_dev *dp)
{
	timeout_id_t	old_id;

	DPRINTF(3, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Ensure timer routine stopped */
	mutex_enter(&dp->intrlock);
	if (dp->link_watcher_id != 0) {
		do {
			old_id = dp->link_watcher_id;
			mutex_exit(&dp->intrlock);
			untimeout(old_id);
			mutex_enter(&dp->intrlock);
		} while (old_id != dp->link_watcher_id);
		dp->link_watcher_id = 0;
	}
	mutex_exit(&dp->intrlock);
#ifdef NEVER
	/* XXX - Davicom PHY won't come up if it is in powerdown mode once */
	gem_mii_write(dp, MII_CONTROL,
			MII_CONTROL_ISOLATE | MII_CONTROL_PWRDN);
#endif
}

void
gem_generate_macaddr(struct gem_dev *dp, uint8_t *mac)
{
	extern char	hw_serial[];
	char		*hw_serial_p;
	int		i;
	uint64_t	val;
	uint64_t	key;

	cmn_err(CE_NOTE,
	"!%s: using temporal ether address, do not use this for long time",
		dp->name);

	/* prefer a fixed address for DHCP */
	hw_serial_p = &hw_serial[0];
	val = stoi(&hw_serial_p);

	key = 0;
	for (i = 0; i < GEM_NAME_LEN; i++) {
		if (dp->name[i] == 0) {
			break;
		}
		key ^= dp->name[i];
	}
	key ^= ddi_get_instance(dp->dip);
	val ^= key << 32;

	/* generate a local address */
	mac[0] = 0x02;
	mac[1] = (uint8_t) (val >> 32);
	mac[2] = (uint8_t) (val >> 24);
	mac[3] = (uint8_t) (val >> 16);
	mac[4] = (uint8_t) (val >> 8);
	mac[5] = (uint8_t) val;
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

	DPRINTF(3, (CE_CONT, "!%s: %s: called", dp->name, __func__));
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
		return (B_FALSE);
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
	return (B_TRUE);

syntax_err:
	cmn_err(CE_CONT,
		"!%s: read mac addr: trying .conf: syntax err %s",
		dp->name, valstr);
	ddi_prop_free(valstr);

	return (B_FALSE);
}


/* ============================================================== */
/*
 * internal start/stop interface
 */
/* ============================================================== */
/*
 * gem_mac_init: cold start
 */
static int
gem_mac_init(struct gem_dev *dp)
{
	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	dp->mac_active = B_FALSE;

	gem_init_rx_ring(dp);
	gem_init_tx_ring(dp);

	gem_prepare_rx_buf(dp);

	/* reset transmitter state */
	dp->tx_blocked  = B_FALSE;
	dp->tx_busy     = 0;
	dp->tx_reclaim_busy = 0;

#ifdef GEM_CONFIG_POLLING
	dp->rx_pkt_cnt    = 0;
	dp->rx_intr_cnt   = 0;
	dp->poll_intr_cnt = 0;
	dp->tx_pkt_cnt    = 0;
	dp->poll_interval = 0;
	dp->tx_coalesce   = B_FALSE;
#endif
	if ((*dp->gc.gc_init_chip)(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FME
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		return (GEM_FAILURE);
	}

	return (GEM_SUCCESS);
}
/*
 * gem_mac_start: warm start
 */
static int
gem_mac_start(struct gem_dev *dp)
{
	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(dp->nic_state == NIC_STATE_ONLINE);
	ASSERT(dp->mii_state ==  MII_STATE_LINKUP);

	/* enable tx and rx */
	mutex_enter(&dp->xmitlock);
	dp->mac_active = B_TRUE;
	mutex_exit(&dp->xmitlock);

	if ((*dp->gc.gc_start_chip)(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
		/* FIXME - the nic should go to an error state. */
		dp->nic_state = NIC_STATE_STOPPED;
#endif
		cmn_err(CE_WARN, "%s: %s: start_chip: failed",
			dp->name, __func__);
		return (GEM_FAILURE);
	}

	dp->nic_state = NIC_STATE_ONLINE;

	/* kick rx */
	if (dp->gc.gc_rx_start != NULL) {
		dp->gc.gc_rx_start(dp,
			SLOT(dp->rx_active_head, dp->gc.gc_rx_ring_size),
			dp->rx_active_tail - dp->rx_active_head);
	}

	mutex_enter(&dp->xmitlock);

	/* load untranmitted packets to the nic */
	if (dp->tx_softq_head != dp->tx_softq_tail &&
	    dp->gc.gc_tx_desc_write_oo) {
		gem_tx_load_descs_oo(dp,
			dp->tx_softq_head, dp->tx_softq_tail,
			HIWAT_INTR(dp->gc.gc_tx_buf_limit) - 1,
			GEM_TXFLAG_HEAD);
	}

	/* issue preloaded tx buffers */
	gem_tx_start_unit(dp);

	mutex_exit(&dp->xmitlock);

	return (GEM_SUCCESS);
}

static int
gem_mac_stop(struct gem_dev *dp, uint_t flags)
{
	int		i;
	int		wait_time; /* in uS */
	clock_t		now;
	timeout_id_t	old_id;
	int		ret = GEM_SUCCESS;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, rx_buf_free:%d",
		dp->name, __func__, dp->rx_buf_freecnt));

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(!mutex_owned(&dp->xmitlock));

	dp->mac_active = B_FALSE;

	/*
	 * Block transmits
	 */
	mutex_enter(&dp->xmitlock);
	while (dp->tx_busy > 0) {
		cv_wait(&dp->tx_drain_cv, &dp->xmitlock);
	}
	mutex_exit(&dp->xmitlock);

	if ((flags & GEM_RESTART_NOWAIT) == 0) {
		/*
		 * Wait for all tx buffers sent.
		 */
		wait_time =
			2 * (8 * MAXPKTBUF(dp) / gem_speed_value[dp->speed]) *
				(dp->tx_active_tail - dp->tx_active_head);

		DPRINTF(0, (CE_CONT, "%s: %s: max drain time: %d uS",
				dp->name, __func__, wait_time));
		i = 0;
		now = ddi_get_lbolt();
		while (dp->tx_active_tail != dp->tx_active_head) {
			if (i > wait_time) {
				/* timeout */
				cmn_err(CE_NOTE, "%s: %s timeout: tx drain",
					dp->name, __func__);
				break;
			}
			(void) gem_reclaim_txbuf(dp);
			drv_usecwait(100);
			i += 100;
		}
		DPRINTF(0, (CE_NOTE,
			"!%s: %s: the nic have drained in %d uS, real %d mS",
			dp->name, __func__, i,
			10*((int)(ddi_get_lbolt() - now))));
	}

	/*
	 * Now we can stop the nic safely.
	 */
	if ((*dp->gc.gc_stop_chip)(dp) != GEM_SUCCESS) {
		cmn_err(CE_NOTE, "%s: %s: resetting the chip to stop it",
			dp->name, __func__);
		if ((*dp->gc.gc_reset_chip)(dp) != GEM_SUCCESS) {
			cmn_err(CE_WARN, "%s: %s: failed to reset chip",
				dp->name, __func__);
#ifdef GEM_CONFIG_FMA
			dp->mac_state = MAC_STATE_UNKNOWN;
			ret = GEM_FAILURE;
#endif
		}
#ifdef GEM_CONFIG_FMA
		dp->mac_state = MAC_STATE_RESET;
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_UNAFFECTED);
#endif
	}

	/*
	 * Clear all rx buffers
	 */
	if ((flags & GEM_RESTART_KEEP_BUF) != 0) {
		(void) gem_receive(dp);
	}
	gem_clean_rx_buf(dp);

	(*dp->gc.gc_get_stats)(dp);

	/*
	 * Clear all pended tx packets
	 */
	ASSERT(dp->tx_active_tail == dp->tx_softq_head);
	ASSERT(dp->tx_softq_tail == dp->tx_free_head);
	if ((flags & GEM_RESTART_KEEP_BUF) != 0) {
		/* restore active tx buffers */
		dp->tx_active_tail = dp->tx_active_head;
		dp->tx_softq_head  = dp->tx_active_head;
	} else {
		gem_clean_tx_buf(dp);
	}

	return (ret);
}

static int
gem_add_multicast(struct gem_dev *dp, const uint8_t *ep)
{
	int		i;
	int		cnt;
	int		err;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);

	if (dp->mc_count_req++ < GEM_MAXMC) {
		/* append the new address at the end of the mclist */
		cnt = dp->mc_count;
		bcopy(ep, dp->mc_list[cnt].addr.ether_addr_octet,
			ETHERADDRL);
		if (dp->gc.gc_multicast_hash != NULL) {
			dp->mc_list[cnt].hash =
				(*dp->gc.gc_multicast_hash)(dp, (uint8_t *)ep);
		}
		dp->mc_count = cnt + 1;
	}

	if (dp->mc_count_req != dp->mc_count) {
		/* multicast address list overflow */
		dp->rxmode |= RXMODE_MULTI_OVF;
	}
	else {
		dp->rxmode &= ~RXMODE_MULTI_OVF;
	}
	/* In gem v2, don't hold xmitlock on calling set_rx_filter */
	/* FIXME - we should check the return code of set_rx_filter */
	err = (*dp->gc.gc_set_rx_filter)(dp);

	mutex_exit(&dp->intrlock);

	return (err);
}

static int
gem_remove_multicast(struct gem_dev *dp, const uint8_t *ep)
{
	size_t		len;
	int		i;
	int		cnt;
	int		err;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);

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

	if (dp->mc_count_req != dp->mc_count) {
		/* multicast address list overflow */
		dp->rxmode |= RXMODE_MULTI_OVF;
	}
	else {
		dp->rxmode &= ~RXMODE_MULTI_OVF;
	}
	/* In gem v2, don't hold xmitlock on calling set_rx_filter */
	err = (*dp->gc.gc_set_rx_filter)(dp);

	mutex_exit(&dp->intrlock);

	return (err);
}

/* ============================================================== */
/*
 * GLD/MAC interface
 */
/* ============================================================== */
#ifdef GEM_CONFIG_GLDv3
/* GLDv3 interfaces */
static int		gem_mc_getstat(void *, uint_t, uint64_t *);
static int		gem_mc_start(void *);
static void		gem_mc_stop(void *);
static int		gem_mc_setpromisc(void *, boolean_t);
static int		gem_mc_multicst(void *, boolean_t, const uint8_t *);
static int		gem_mc_unicst(void *, const uint8_t *);
static mblk_t		*gem_mc_tx(void *, mblk_t *);
static void		gem_mc_resources(void *);
static void		gem_mc_ioctl(void *, queue_t *, mblk_t *);
static boolean_t	gem_mc_getcapab(void *, mac_capab_t, void *);

#define	GEM_M_CALLBACK_FLAGS	(MC_RESOURCES | MC_IOCTL | MC_GETCAPAB)

static mac_callbacks_t gem_m_callbacks = {
	GEM_M_CALLBACK_FLAGS,
	gem_mc_getstat,
	gem_mc_start,
	gem_mc_stop,
	gem_mc_setpromisc,
	gem_mc_multicst,
	gem_mc_unicst,
	gem_mc_tx,
	gem_mc_resources,
	gem_mc_ioctl,
	gem_mc_getcapab,
};

static int
gem_mc_start(void *arg)
{
	int		err = 0;
	struct gem_dev *dp = arg;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	if (gem_mac_init(dp) != GEM_SUCCESS) {
		err = EIO;
		goto x;
	}
	dp->nic_state = NIC_STATE_INITIALIZED;

	/* reset rx filter state */
	dp->mc_count     = 0;
	dp->mc_count_req = 0;

	/* setup initial rx filter */
	bcopy(dp->dev_addr.ether_addr_octet,
		dp->cur_addr.ether_addr_octet, ETHERADDRL);
	dp->rxmode |= RXMODE_ENABLE;

	if ((*dp->gc.gc_set_rx_filter)(dp) != GEM_SUCCESS) {
		err = EIO;
		goto x;
	}

	dp->nic_state = NIC_STATE_ONLINE;
	if (dp->mii_state == MII_STATE_LINKUP) {
		if (gem_mac_start(dp) != GEM_SUCCESS) {
			err = EIO;
			goto x;
		}
	}

	dp->timeout_id = timeout((void (*)(void *))gem_tx_timeout,
				(void *)dp, dp->gc.gc_tx_timeout_interval);
	mutex_exit(&dp->intrlock);

	return (0);
x:
	dp->nic_state = NIC_STATE_STOPPED;
	mutex_exit(&dp->intrlock);
	return (err);
}

static void
gem_mc_stop(void *arg)
{
	struct gem_dev	*dp = arg;
	timeout_id_t	old_id;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* stop rx */
	mutex_enter(&dp->intrlock);
	dp->rxmode &= ~RXMODE_ENABLE;
	(*dp->gc.gc_set_rx_filter)(dp);
	mutex_exit(&dp->intrlock);

	/* stop tx timeout watcher */
	if (dp->timeout_id) {
		do {
			untimeout(old_id = dp->timeout_id);
		} while (dp->timeout_id != old_id);	
		dp->timeout_id = 0;
	}

	/* make the nic state inactive */
	mutex_enter(&dp->intrlock);
	dp->nic_state = NIC_STATE_STOPPED;

	/* we need deassert mac_active due to block interrupt handler */
	mutex_enter(&dp->xmitlock);
	dp->mac_active = B_FALSE;
	mutex_exit(&dp->xmitlock);

	/* block interrupts */
	while (dp->intr_busy) {
		cv_wait(&dp->tx_drain_cv, &dp->intrlock);
	}
	(void) gem_mac_stop(dp, 0);
	mutex_exit(&dp->intrlock);
}

static int
gem_mc_multicst(void *arg, boolean_t add, const uint8_t *ep)
{
	int		err;
	struct gem_dev	*dp = arg;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (add) {
		err = gem_add_multicast(dp, ep);
	}
	else {
		err = gem_remove_multicast(dp, ep);
	}

	if (err != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		err = EIO;
	}

	return (err);
}

static int
gem_mc_setpromisc(void *arg, boolean_t on)
{
	int		err = 0;	/* no error */
	struct gem_dev	*dp = arg;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	if (on) {
		dp->rxmode |= RXMODE_PROMISC;
	} else {
		dp->rxmode &= ~RXMODE_PROMISC;
	}

	if ((*dp->gc.gc_set_rx_filter)(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		err = EIO;
	}
	mutex_exit(&dp->intrlock);

	return (err);
}

int
gem_mc_getstat(void *arg, uint_t stat, uint64_t *valp)
{
	struct gem_dev		*dp = arg;
	struct gem_stats	*gstp = &dp->stats;
	uint64_t		val = 0;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if ((*dp->gc.gc_get_stats)(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		return (EIO);
	}

	switch (stat) {
	case MAC_STAT_IFSPEED:
		val = gem_speed_value[dp->speed] *1000000ull;
		break;

	case MAC_STAT_MULTIRCV:
		val = gstp->rmcast;
		break;

	case MAC_STAT_BRDCSTRCV:
		val = gstp->rbcast;
		break;

	case MAC_STAT_MULTIXMT:
		val = gstp->omcast;
		break;

	case MAC_STAT_BRDCSTXMT:
		val = gstp->obcast;
		break;

	case MAC_STAT_NORCVBUF:
		val = gstp->norcvbuf + gstp->missed;
		break;

	case MAC_STAT_IERRORS:
		val = gstp->errrcv;
		break;

	case MAC_STAT_NOXMTBUF:
		val = gstp->noxmtbuf;
		break;

	case MAC_STAT_OERRORS:
		val = gstp->errxmt;
		break;

	case MAC_STAT_COLLISIONS:
		val = gstp->collisions;
		break;

	case MAC_STAT_RBYTES:
		val = gstp->rbytes;
		break;

	case MAC_STAT_IPACKETS:
		val = gstp->rpackets;
		break;

	case MAC_STAT_OBYTES:
		val = gstp->obytes;
		break;

	case MAC_STAT_OPACKETS:
		val = gstp->opackets;
		break;

	case ETHER_STAT_ALIGN_ERRORS:
		val = gstp->frame;
		break;

	case ETHER_STAT_FCS_ERRORS:
		val = gstp->crc;
		break;

	case ETHER_STAT_FIRST_COLLISIONS:
		val = gstp->first_coll;
		break;

	case ETHER_STAT_MULTI_COLLISIONS:
		val = gstp->multi_coll;
		break;
#if 0
	case ETHER_STAT_SQE_ERRORS:
		val = gstp->crc;
		break;
#endif
	case ETHER_STAT_DEFER_XMTS:
		val = gstp->defer;
		break;

	case ETHER_STAT_TX_LATE_COLLISIONS:
		val = gstp->xmtlatecoll;
		break;

	case ETHER_STAT_EX_COLLISIONS:
		val = gstp->excoll;
		break;

	case ETHER_STAT_MACXMT_ERRORS:
		val = gstp->xmit_internal_err;
		break;

	case ETHER_STAT_CARRIER_ERRORS:
		val = gstp->nocarrier;
		break;

	case ETHER_STAT_TOOLONG_ERRORS:
		val = gstp->frame_too_long;
		break;
#if 0
	case ETHER_STAT_MACRCV_ERRORS
		val = dp->mii_phy_addr;
		break;
#endif
	case ETHER_STAT_XCVR_ADDR:
		val = dp->mii_phy_addr;
		break;

	case ETHER_STAT_XCVR_ID:
		val = dp->mii_phy_id;
		break;

	case ETHER_STAT_XCVR_INUSE:
		/* FIXME - the name should be changed to mii_xcvr */
		switch (dp->gc.gc_mii_mode) {
		default:
		case GEM_MODE_100BASETX:
			val = XCVR_100X;
			break;

		case GEM_MODE_1000BASET:
			val = XCVR_1000T;
			break;

		case GEM_MODE_1000BASETX:
			val = XCVR_1000X;
			break;
		}
	/* FIXME - should be calculated from MII_STAT */
	case ETHER_STAT_CAP_1000FDX:
		val = dp->anadv_1000fdx;
		break;

	case ETHER_STAT_CAP_1000HDX:
		val = dp->anadv_1000hdx;
		break;

	case ETHER_STAT_CAP_100FDX:
		val = dp->anadv_100fdx;
		break;

	case ETHER_STAT_CAP_100HDX:
		val = dp->anadv_100hdx;
		break;

	case ETHER_STAT_CAP_10FDX:
		val = dp->anadv_10fdx;
		break;

	case ETHER_STAT_CAP_10HDX:
		val = dp->anadv_10hdx;
		break;

	case ETHER_STAT_CAP_ASMPAUSE:
		val = (dp->mii_advert & MII_ABILITY_ASM_DIR) != 0;
		break;

	case ETHER_STAT_CAP_PAUSE:
		val = (dp->mii_advert & MII_ABILITY_PAUSE) != 0;
		break;

	case ETHER_STAT_CAP_AUTONEG:
		val = !dp->mii_fixedmode;
		break;

	case ETHER_STAT_ADV_CAP_1000FDX:
		val = (dp->mii_ctl1000 & MII_1000TC_ADV_FULL) != 0;
		break;

	case ETHER_STAT_ADV_CAP_1000HDX:
		val = (dp->mii_ctl1000 & MII_1000TC_ADV_HALF) != 0;
		break;

	case ETHER_STAT_ADV_CAP_100FDX:
		val = (dp->mii_advert & MII_ABILITY_100BASE_TX_FD) != 0;
		break;

	case ETHER_STAT_ADV_CAP_100HDX:
		val = (dp->mii_advert & MII_ABILITY_100BASE_TX) != 0;
		break;

	case ETHER_STAT_ADV_CAP_10FDX:
		val = (dp->mii_advert & MII_ABILITY_10BASE_T_FD) != 0;
		break;

	case ETHER_STAT_ADV_CAP_10HDX:
		val = (dp->mii_advert & MII_ABILITY_10BASE_T) != 0;
		break;

	case ETHER_STAT_ADV_CAP_ASMPAUSE:
		val = (dp->mii_advert & MII_ABILITY_ASM_DIR) != 0;
		break;

	case ETHER_STAT_ADV_CAP_PAUSE:
		val = (dp->mii_advert & MII_ABILITY_PAUSE) != 0;
		break;

	case ETHER_STAT_ADV_CAP_AUTONEG:
		val = (dp->mii_status & MII_STATUS_CANAUTONEG) != 0;
		break;

	case ETHER_STAT_LP_CAP_1000FDX:
		val = (dp->mii_stat1000 & MII_1000TS_LP_FULL) != 0;
		break;

	case ETHER_STAT_LP_CAP_1000HDX:
		val = (dp->mii_stat1000 & MII_1000TS_LP_HALF) != 0;
		break;

	case ETHER_STAT_LP_CAP_100FDX:
		val = (dp->mii_lpable & MII_ABILITY_100BASE_TX_FD) != 0;
		break;

	case ETHER_STAT_LP_CAP_100HDX:
		val = (dp->mii_lpable & MII_ABILITY_100BASE_TX) != 0;
		break;

	case ETHER_STAT_LP_CAP_10FDX:
		val = (dp->mii_lpable & MII_ABILITY_10BASE_T_FD) != 0;
		break;

	case ETHER_STAT_LP_CAP_10HDX:
		val = (dp->mii_lpable & MII_ABILITY_10BASE_T) != 0;
		break;

	case ETHER_STAT_LP_CAP_ASMPAUSE:
		val = (dp->mii_lpable & MII_ABILITY_ASM_DIR) != 0;
		break;

	case ETHER_STAT_LP_CAP_PAUSE:
		val = (dp->mii_lpable & MII_ABILITY_PAUSE) != 0;
		break;

	case ETHER_STAT_LP_CAP_AUTONEG:
		val = (dp->mii_exp & MII_AN_EXP_LPCANAN) != 0;
		break;

	case ETHER_STAT_LINK_ASMPAUSE:
		val = dp->flow_control > FLOW_CONTROL_SYMMETRIC;
		break;

	case ETHER_STAT_LINK_PAUSE:
		val = dp->flow_control == FLOW_CONTROL_SYMMETRIC;
		break;

	case ETHER_STAT_LINK_AUTONEG:
		/* FIXME -- see also the link partnar capability */
		val = !dp->mii_fixedmode;
		break;

	case ETHER_STAT_LINK_DUPLEX:
		val = (dp->mii_state == MII_STATE_LINKUP) ?
					(dp->full_duplex ? 2 : 1) : 0;
		break;

	default:
#if GEM_DEBUG_LEVEL > 2
		cmn_err(CE_WARN,
		    "%s: unrecognized parameter value = %d",
		    __func__, stat);
#endif
		 return (ENOTSUP);
	}

	*valp = val;

	return (0);
}

static int
gem_mc_unicst(void *arg, const uint8_t *mac)
{
	int		err = 0;
	struct gem_dev	*dp = arg;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	bcopy(mac, dp->cur_addr.ether_addr_octet, ETHERADDRL);
	dp->rxmode |= RXMODE_ENABLE;

	if ((*dp->gc.gc_set_rx_filter)(dp) != GEM_SUCCESS) {
		err = EIO;
	}
	mutex_exit(&dp->intrlock);

	return (err);
}

/*
 * gem_m_tx is used only for sending data packets into ethernet wire.
 */
static mblk_t *
gem_mc_tx(void *arg, mblk_t *mp)
{
	struct gem_dev	*dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(dp->nic_state == NIC_STATE_ONLINE);

	if (dp->mii_state != MII_STATE_LINKUP) {
		/* Some nics hate to send packets when the link is down. */
		/* XXX - solaris hung on returning non-null mp at boot time. */
		mblk_t		*tp;

		while (mp != NULL) {
			tp = mp->b_next;
			mp->b_next = NULL;
			freemsg(mp);
			mp = tp;
		}
		return (mp);
	}

	return (gem_send_common(dp, mp, 0));
}

static void
gem_mc_ioctl(void *arg, queue_t *wq, mblk_t *mp)
{
	struct gem_dev	*dp = arg;
	struct iocblk	*iocp;
#ifdef notdef
	enum ioc_reply	status;
#else
	int 		status;
#endif
	boolean_t	need_privilege;
	int		err;
	int		cmd;

	/*
	 * Validate the command before bothering with the mutex ...
	 */
	iocp = (struct iocblk *)mp->b_rptr;
	iocp->ioc_error = 0;
	need_privilege = B_TRUE;
	cmd = iocp->ioc_cmd;

	DPRINTF(0, (CE_CONT, "%s: %s cmd:0x%x", dp->name, __func__, cmd));

	switch (cmd) {
	default:
		miocnak(wq, mp, 0, EINVAL);
		return;
#ifdef notdef
	case GEM_MII_READ:
	case GEM_MII_WRITE:
	case GEM_SEE_READ:
	case GEM_SEE_WRITE:
	case GEM_DIAG:
	case GEM_PEEK:
	case GEM_POKE:
	case GEM_PHY_RESET:
	case GEM_SOFT_RESET:
	case GEM_HARD_RESET:
		break;
#endif
#ifdef notdef
	case LB_GET_INFO_SIZE:
	case LB_GET_INFO:
	case LB_GET_MODE:
		need_privilege = B_FALSE;
		/* FALLTHRU */
	case LB_SET_MODE:
		break;
#endif
#ifdef notdef
	case ND_GET:
		need_privilege = B_FALSE;
		/* FALLTHRU */
	case ND_SET:
		break;
#endif
	}
#ifdef notyet
	if (need_privilege) {
		/*
		 * Check for specific net_config privilege on Solaris 10+.
		 * Otherwise just check for root access ...
		 */
		if (secpolicy_net_config != NULL)
			err = secpolicy_net_config(iocp->ioc_cr, B_FALSE);
		else
			err = drv_priv(iocp->ioc_cr);
		if (err != 0) {
			miocnak(wq, mp, 0, err);
			return;
		}
	}

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	switch (cmd) {
	default:
		_NOTE(NOTREACHED)
#ifdef notdef
		status = IOC_INVAL;
#endif
		break;
#ifdef notdef
	case GEM_MII_READ:
	case GEM_MII_WRITE:
	case GEM_SEE_READ:
	case GEM_SEE_WRITE:
	case GEM_DIAG:
	case GEM_PEEK:
	case GEM_POKE:
	case GEM_PHY_RESET:
	case GEM_SOFT_RESET:
	case GEM_HARD_RESET:
		break;
#endif
#ifdef notdef
	case LB_GET_INFO_SIZE:
	case LB_GET_INFO:
	case LB_GET_MODE:
	case LB_SET_MODE:
		status = gem_loop_ioctl(dp, wq, mp, iocp);
		break;
#endif

#ifdef notdef
	case ND_GET:
	case ND_SET:
		status = gem_nd_ioctl(dp, wq, mp, iocp);
		break;
#endif
	}

#ifdef notdef
	switch (status) {
	case IOC_RESTART_REPLY:
	case IOC_RESTART_ACK:
	}
#endif
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	/*
	 * Finally, decide how to reply
	 */
	switch (status) {
	default:
#ifdef notdef
	case IOC_INVAL:
#endif
		/*
		 * Error, reply with a NAK and EINVAL or the specified error
		 */
		miocnak(wq, mp, 0, iocp->ioc_error == 0 ?
			EINVAL : iocp->ioc_error);
		break;
#ifdef notdef
	case IOC_DONE:
		/*
		 * OK, reply already sent
		 */
		break;

	case IOC_RESTART_ACK:
	case IOC_ACK:
		/*
		 * OK, reply with an ACK
		 */
		miocack(wq, mp, 0, 0);
		break;

	case IOC_RESTART_REPLY:
	case IOC_REPLY:
		/*
		 * OK, send prepared reply as ACK or NAK
		 */
		mp->b_datap->db_type = iocp->ioc_error == 0 ?
			M_IOCACK : M_IOCNAK;
		qreply(wq, mp);
		break;
#endif
	}
#endif /* notyet */
}

static void
gem_set_coalease(void *arg, time_t ticks, uint_t count)
{
	struct gem_dev *dp = arg;
	DPRINTF(2, (CE_CONT, "%s: %s: ticks:%d count:%d",
		dp->name, __func__, ticks, count));
#if 1
	mutex_enter(&dp->intrlock);
	dp->poll_pkt_delay = count;
	mutex_exit(&dp->intrlock);
#endif
}

static void
gem_mc_resources(void *arg)
{
	struct gem_dev		*dp = arg;
	mac_rx_fifo_t		mrf;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	/*
	 * Register Rx rings as resources and save mac
	 * resource id for future reference
	 */
	mrf.mrf_type = MAC_RX_FIFO;
	mrf.mrf_blank = gem_set_coalease;
	mrf.mrf_arg = (void *)dp;
	mrf.mrf_normal_blank_time = 128; /* in uS */
	mrf.mrf_normal_pkt_count = dp->poll_pkt_delay;

	dp->mac_rx_ring_ha = mac_resource_add(dp->mh, (mac_resource_t *)&mrf);

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);
}

static boolean_t
gem_mc_getcapab(void *arg, mac_capab_t cap, void *cap_data)
{
	struct gem_dev	*dp = arg;
	uint32_t	cksum;
	boolean_t	ret;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ret = B_FALSE;
	switch (cap) {
	case MAC_CAPAB_HCKSUM:
		cksum = 0;
		if ((dp->misc_flag & GEM_CKSUM_FULL_IPv4) != 0) {
			cksum |= HCKSUM_INET_FULL_V4;
		}
		if ((dp->misc_flag & GEM_CKSUM_FULL_IPv6) != 0) {
			cksum |= HCKSUM_INET_FULL_V6;
		}
		if ((dp->misc_flag & GEM_CKSUM_HEADER_IPv4) != 0) {
			cksum |= HCKSUM_IPHDRCKSUM;
		}
		if ((dp->misc_flag & GEM_CKSUM_PARTIAL) != 0) {
			cksum |= HCKSUM_INET_PARTIAL;
		}

		*(uint32_t *)cap_data = cksum;
		ret = B_TRUE;
		break;

	case MAC_CAPAB_POLL:
		ret = B_TRUE;
		break;
	}
	return (ret);
}

static void
gem_gld3_init(struct gem_dev *dp, mac_register_t *macp)
{
        macp->m_type_ident = MAC_PLUGIN_IDENT_ETHER;
        macp->m_driver = dp;
        macp->m_dip = dp->dip;
#if 0
        macp->m_instance = 0;
#endif
        macp->m_src_addr = dp->dev_addr.ether_addr_octet;
#if 0
        macp->m_dst_addr = NULL;
#endif
        macp->m_callbacks = &gem_m_callbacks;
        macp->m_min_sdu = 0;
        macp->m_max_sdu = dp->mtu;
#if 0
        macp->m_pdata = NULL;
        macp->m_pdata_size = 0;
#endif
}
#else
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

static int
gem_gld_reset(gld_mac_info_t *macinfo)
{
	struct gem_dev *dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	gem_mac_init(dp);
	dp->nic_state = NIC_STATE_INITIALIZED;

	/* reset rx filter state */
	dp->mc_count     = 0;
	dp->mc_count_req = 0;

	/* setup media mode if the link have been up */
	if (dp->mii_state == MII_STATE_LINKUP) {
		(dp->gc.gc_set_media)(dp);
	}
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_start(gld_mac_info_t *macinfo)
{
	struct gem_dev *dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	dp->nic_state = NIC_STATE_ONLINE;
	if (dp->mii_state == MII_STATE_LINKUP) {
		gem_mac_start(dp);
	}
#ifdef notdef
	/*
	 * XXX - don't call gld_linkstate in gld call back routines,
	 * otherwise it cause recursive mutex call.
	 */
#ifdef SOLARIS10
	gld_linkstate(dp->macinfo, GLD_LINKSTATE_UP);
#endif
#endif
	dp->timeout_id = timeout((void (*)(void *))gem_tx_timeout,
				(void *)dp, dp->gc.gc_tx_timeout_interval);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_stop(gld_mac_info_t *macinfo)
{
	timeout_id_t	old_id;
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;

	/* stop rx */
	mutex_enter(&dp->intrlock);
	dp->rxmode &= ~RXMODE_ENABLE;
	(*dp->gc.gc_set_rx_filter)(dp);
	mutex_exit(&dp->intrlock);

	/* stop tx timeout watcher */
	if (dp->timeout_id) {
		do {
			untimeout(old_id = dp->timeout_id);
		} while (dp->timeout_id != old_id);	
		dp->timeout_id = 0;
	}

	/* make the nic state inactive */
	mutex_enter(&dp->intrlock);
	dp->nic_state = NIC_STATE_STOPPED;

	/* we need deassert mac_active due to block interrupt handler */
	mutex_enter(&dp->xmitlock);
	dp->mac_active = B_FALSE;
	mutex_exit(&dp->xmitlock);

	/* block interrupts */
	while (dp->intr_busy) {
		cv_wait(&dp->tx_drain_cv, &dp->intrlock);
	}
	(void) gem_mac_stop(dp, 0);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_set_multicast(gld_mac_info_t *macinfo, uint8_t *ep, int flag)
{
	struct gem_dev	*dp;

	dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (flag == GLD_MULTI_ENABLE) {
		gem_add_multicast(dp, ep);
	}
	else {
		gem_remove_multicast(dp, ep);
	}

	return (GLD_SUCCESS);
}

static int
gem_gld_set_promiscuous(gld_mac_info_t *macinfo, int flag)
{
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
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

	(*dp->gc.gc_set_rx_filter)(dp);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static int
gem_gld_set_mac_address(gld_mac_info_t *macinfo, uint8_t *mac)
{
	struct gem_dev	*dp = (struct gem_dev *)macinfo->gldm_private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->intrlock);
	bcopy(mac, dp->cur_addr.ether_addr_octet, ETHERADDRL);
	dp->rxmode |= RXMODE_ENABLE;

	(*dp->gc.gc_set_rx_filter)(dp);
	mutex_exit(&dp->intrlock);

	return (GLD_SUCCESS);
}

static	int
gem_gld_get_stats(gld_mac_info_t *macinfo, struct gld_stats *gs)
{
	struct gem_dev		*dp = (struct gem_dev *)macinfo->gldm_private;
	struct gem_stats	*vs;

	if ((*dp->gc.gc_get_stats)(dp) != GEM_SUCCESS) {
#ifdef GEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		return (GEM_FAILURE);
	}

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

/*
 * gem_gld_send is used only for sending data packets into ethernet wire.
 */
static int
gem_gld_send(gld_mac_info_t *macinfo, mblk_t *mp)
{
	struct gem_dev	*dp	= (struct gem_dev *) macinfo->gldm_private;
	uint32_t	flags	= 0;

	ASSERT(dp->nic_state == NIC_STATE_ONLINE);

	if (dp->mii_state != MII_STATE_LINKUP) {
		/* Some nics hate to send packets while the link is down. */
		/* we dicard the untransmitted packets silently */
		freemsg(mp);
#ifdef GEM_CONFIG_FMA
		/* FIXME - should we ignore the error? */
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		return (GLD_SUCCESS);
	}

	if (dp->gc.gc_tx_copy_thresh >= MAXPKTBUF(dp)) {
		flags |= GEM_SEND_COPY;
	}

	return ((gem_send_common(dp, mp, flags) == NULL)
					? GLD_SUCCESS : GLD_NORESOURCES);
}

/*
 * gem_gld_send is used only for sending data packets into ethernet wire.
 */
static int
gem_gld_send_tagged(gld_mac_info_t *macinfo, mblk_t *mp, uint32_t vtag)
{
	struct gem_dev	*dp = (struct gem_dev *) macinfo->gldm_private;
	uint32_t	flags;

	/*
	 * Some nics hate to send packets while the link is down.
	 */
	if (dp->mii_state != MII_STATE_LINKUP) {
		/* we dicard the untransmitted packets silently */
		freemsg(mp);
#ifdef GEM_CONFIG_FMA
		/* FIXME - should we ignore the error? */
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_UNAFFECTED);
#endif
		return (GLD_SUCCESS);
	}

	flags = GLD_VTAG_TCI(vtag) << GEM_SEND_VTAG_SHIFT;
	if (dp->gc.gc_tx_copy_thresh >= MAXPKTBUF(dp)) {
		flags |= GEM_SEND_COPY;
	}

	return ((gem_send_common(dp, mp, flags) == NULL)
			? GLD_SUCCESS : GLD_NORESOURCES);
}

static u_int
gem_gld_intr(gld_mac_info_t *macinfo)
{
	struct gem_dev	*dp = (struct gem_dev *)(macinfo->gldm_private);
	u_int		ret;

	ret = gem_intr(dp);

	if ((ret & INTR_RESTART_TX) != 0) {
		DPRINTF(4, (CE_CONT, "!%s: calling gld_sched", dp->name));
		gld_sched(macinfo);

		ret &= ~INTR_RESTART_TX;
	}

	return (ret);
}


static void
gem_gld_init(struct gem_dev *dp, gld_mac_info_t *macinfo, char *ident)
{
	/*
	 * configure GLD 
	 */
	macinfo->gldm_devinfo		= dp->dip;
	macinfo->gldm_private		= (caddr_t)dp;
	macinfo->gldm_cookie		= dp->iblock_cookie;
	macinfo->gldm_reset		= &gem_gld_reset;
	macinfo->gldm_start		= &gem_gld_start;
	macinfo->gldm_stop		= &gem_gld_stop;
	macinfo->gldm_set_mac_addr	= &gem_gld_set_mac_address;
	macinfo->gldm_send		= &gem_gld_send;
#ifdef GEM_CONFIG_VLAN
	if ((dp->misc_flag & GEM_VLAN) != 0) {
		macinfo->gldm_send_tagged = &gem_gld_send_tagged;
	}
#endif
	macinfo->gldm_set_promiscuous	= &gem_gld_set_promiscuous;
	macinfo->gldm_get_stats		= &gem_gld_get_stats;
	macinfo->gldm_ioctl		= NULL; 
	macinfo->gldm_set_multicast	= &gem_gld_set_multicast;
	macinfo->gldm_intr		= &gem_gld_intr;
	macinfo->gldm_mctl		= NULL;
	macinfo->gldm_ident		= ident;
	macinfo->gldm_type		= DL_ETHER;
	macinfo->gldm_minpkt		= 0;
	macinfo->gldm_maxpkt		= dp->mtu;
	macinfo->gldm_addrlen		= ETHERADDRL;
	macinfo->gldm_saplen		= -2;
	macinfo->gldm_ppa		= ddi_get_instance(dp->dip);
#ifdef SOLARIS10
	macinfo->gldm_capabilities	= GLD_CAP_LINKSTATE;
#endif
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if ((dp->misc_flag & GEM_CKSUM_FULL_IPv4) != 0) {
		macinfo->gldm_capabilities |= GLD_CAP_CKSUM_FULL_V4;
	}
#ifdef GLD_CAP_CKSUM_FULL_V6
	if ((dp->misc_flag & GEM_CKSUM_FULL_IPv6) != 0) {
		macinfo->gldm_capabilities |= GLD_CAP_CKSUM_FULL_V6;
	}
#endif
	if ((dp->misc_flag & GEM_CKSUM_HEADER_IPv4) != 0) {
		macinfo->gldm_capabilities |= GLD_CAP_CKSUM_IPHDR;
	}
	if ((dp->misc_flag & GEM_CKSUM_PARTIAL) != 0) {
		macinfo->gldm_capabilities |= GLD_CAP_CKSUM_PARTIAL;
	}
#endif
	macinfo->gldm_vendor_addr =
			dp->dev_addr.ether_addr_octet;
	macinfo->gldm_broadcast_addr =
			gem_etherbroadcastaddr.ether_addr_octet;
}
#endif /* GEM_CONFIG_GLDv3 */

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

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * Get media mode infomation from .conf file
	 */
	sprintf(propname, "%s-duplex", dp->name);
	if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
			DDI_PROP_DONTPASS, propname, &valstr
			)) == DDI_PROP_SUCCESS) {
		dp->mii_fixedmode = B_TRUE;
		if (strcmp(valstr, "full") == 0) {
			dp->full_duplex = B_TRUE;
		}
		else if (strcmp(valstr, "half") == 0) {
			dp->full_duplex = B_FALSE;
		}
		else {
			cmn_err(CE_WARN,
				"!%s: property %s: illegal value (%s)",
				dp->name, propname, valstr);
			dp->mii_fixedmode = B_FALSE;
		}
		ddi_prop_free(valstr);
	}

	if ((val = gem_prop_get_int(dp, "%s-speed", 0)) > 0) {
		dp->mii_fixedmode = B_TRUE;
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
			dp->mii_fixedmode = B_FALSE;
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


/*
 * Gem kstat support
 */

/* kstat items based of dmfe driver */

struct gem_kstat_named {
	struct kstat_named	ks_xcvr_addr;
	struct kstat_named	ks_xcvr_id;
	struct kstat_named	ks_xcvr_inuse;
	struct kstat_named	ks_link_up;
	struct kstat_named	ks_link_duplex;	/* 0:unknwon, 1:half, 2:full */
	struct kstat_named	ks_cap_1000fdx;
	struct kstat_named	ks_cap_1000hdx;
	struct kstat_named	ks_cap_100fdx;
	struct kstat_named	ks_cap_100hdx;
	struct kstat_named	ks_cap_10fdx;
	struct kstat_named	ks_cap_10hdx;
#ifdef notdef
	/* XXX - no make sense */
	struct kstat_named	ks_cap_remfault;
#endif
	struct kstat_named	ks_cap_autoneg;

	struct kstat_named	ks_adv_cap_1000fdx;
	struct kstat_named	ks_adv_cap_1000hdx;
	struct kstat_named	ks_adv_cap_100fdx;
	struct kstat_named	ks_adv_cap_100hdx;
	struct kstat_named	ks_adv_cap_10fdx;
	struct kstat_named	ks_adv_cap_10hdx;
	struct kstat_named	ks_adv_cap_remfault;
	struct kstat_named	ks_adv_cap_autoneg;
	struct kstat_named	ks_lp_cap_1000fdx;
	struct kstat_named	ks_lp_cap_1000hdx;
	struct kstat_named	ks_lp_cap_100fdx;
	struct kstat_named	ks_lp_cap_100hdx;
	struct kstat_named	ks_lp_cap_10fdx;
	struct kstat_named	ks_lp_cap_10hdx;
	struct kstat_named	ks_lp_cap_remfault;
	struct kstat_named	ks_lp_cap_autoneg;
};

static int
gem_kstat_update(kstat_t *ksp, int rw)
{
	struct gem_kstat_named	*knp;
	struct gem_dev		*dp = (struct gem_dev *) ksp->ks_private;

	if (rw != KSTAT_READ) {
		return (0);
	}

	knp = (struct gem_kstat_named *) ksp->ks_data;

	knp->ks_xcvr_addr.value.ul	= dp->mii_phy_addr;
	knp->ks_xcvr_id.value.ul	= dp->mii_phy_id;
	knp->ks_xcvr_inuse.value.ul	= 1;
	knp->ks_link_up.value.ul = dp->mii_state == MII_STATE_LINKUP;
	knp->ks_link_duplex.value.ul =
		(dp->mii_state == MII_STATE_LINKUP) ?
					(dp->full_duplex ? 2 : 1) : 0;

	if ((dp->mii_status & MII_STATUS_XSTATUS) != 0) {
		knp->ks_cap_1000fdx.value.ul =
			(dp->mii_xstatus & MII_XSTATUS_1000BASET_FD) != 0;
		knp->ks_cap_1000hdx.value.ul =
			(dp->mii_xstatus & MII_XSTATUS_1000BASET) != 0;
	}
	else {
		knp->ks_cap_1000fdx.value.ul = 0;
		knp->ks_cap_1000hdx.value.ul = 0;
	}

	knp->ks_cap_100fdx.value.ul =
			(dp->mii_status & MII_STATUS_100_BASEX_FD) != 0;
	knp->ks_cap_100hdx.value.ul =
			(dp->mii_status & MII_STATUS_100_BASEX) != 0;
	knp->ks_cap_10fdx.value.ul =
			(dp->mii_status & MII_STATUS_10_FD) != 0;
	knp->ks_cap_10hdx.value.ul =
			(dp->mii_status & MII_STATUS_10) != 0;
#ifdef notdef
	/* XXX - no make sense */
	knp->ks_cap_remfault.value.ul =
			(dp->mii_status & MII_STATUS_REMFAULT) != 0;
#endif
	knp->ks_cap_autoneg.value.ul =
			(dp->mii_status & MII_STATUS_CANAUTONEG) != 0;

	if ((dp->mii_status & MII_STATUS_XSTATUS) != 0) {
		knp->ks_adv_cap_1000fdx.value.ul = dp->anadv_1000fdx;
		knp->ks_adv_cap_1000hdx.value.ul = dp->anadv_1000hdx;
	}

	knp->ks_adv_cap_100fdx.value.ul	= dp->anadv_100fdx;
	knp->ks_adv_cap_100hdx.value.ul	= dp->anadv_100hdx;
	knp->ks_adv_cap_10fdx.value.ul	= dp->anadv_10fdx;
	knp->ks_adv_cap_10hdx.value.ul	= dp->anadv_10hdx;
	knp->ks_adv_cap_remfault.value.ul = 0;
	knp->ks_adv_cap_autoneg.value.ul = !dp->mii_fixedmode;

	if ((dp->mii_status & MII_STATUS_XSTATUS) != 0) {
		knp->ks_lp_cap_1000fdx.value.ul =
			(dp->mii_stat1000 & MII_1000TS_LP_FULL) != 0;
		knp->ks_lp_cap_1000hdx.value.ul =
			(dp->mii_stat1000 & MII_1000TS_LP_HALF) != 0;
	}
	else {
		knp->ks_lp_cap_1000fdx.value.ul = 0;
		knp->ks_lp_cap_1000hdx.value.ul = 0;
	}

	knp->ks_lp_cap_100fdx.value.ul =
			(dp->mii_lpable & MII_ABILITY_100BASE_TX_FD) != 0;
	knp->ks_lp_cap_100hdx.value.ul =
			(dp->mii_lpable & MII_ABILITY_100BASE_TX) != 0;
	knp->ks_lp_cap_10fdx.value.ul =
			(dp->mii_lpable & MII_ABILITY_10BASE_T_FD) != 0;
	knp->ks_lp_cap_10hdx.value.ul =
			(dp->mii_lpable & MII_ABILITY_10BASE_T) != 0;
	knp->ks_lp_cap_remfault.value.ul =
			(dp->mii_exp & MII_AN_EXP_PARFAULT) != 0;
	knp->ks_lp_cap_autoneg.value.ul =
			(dp->mii_exp & MII_AN_EXP_LPCANAN) != 0;

	return (0);
}

#ifndef GEM_CONFIG_GLDv3
static int
gem_kstat_init(struct gem_dev *dp)
{
	int			i;
	kstat_t			*ksp;
	struct gem_kstat_named	*knp;

	ksp = kstat_create(
		(char *)ddi_driver_name(dp->dip), ddi_get_instance(dp->dip),
		"mii", "net", KSTAT_TYPE_NAMED,
		sizeof(*knp) / sizeof(knp->ks_xcvr_addr), 0);

	if (ksp == NULL) {
		cmn_err(CE_WARN, "%s: %s() for mii failed",	
			dp->name, __func__);
		return (GEM_FAILURE);
	}

	knp = (struct gem_kstat_named *) ksp->ks_data;

	kstat_named_init(&knp->ks_xcvr_addr,		"xcvr_addr",
		KSTAT_DATA_INT32);
	kstat_named_init(&knp->ks_xcvr_id,		"xcvr_id",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_xcvr_inuse,		"xcvr_inuse",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_link_up,		"link_up",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_link_duplex,		"link_duplex",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_1000fdx,		"cap_1000fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_1000hdx,		"cap_1000hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_100fdx,		"cap_100fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_100hdx,		"cap_100hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_10fdx,		"cap_10fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_cap_10hdx,		"cap_10hdx",
		KSTAT_DATA_UINT32);
#ifdef notdef
	kstat_named_init(&knp->ks_cap_remfault,		"cap_rem_fault",
		KSTAT_DATA_UINT32);
#endif
	kstat_named_init(&knp->ks_cap_autoneg,		"cap_autoneg",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_1000fdx,	"adv_cap_1000fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_1000hdx,	"adv_cap_1000hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_100fdx,	"adv_cap_100fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_100hdx,	"adv_cap_100hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_10fdx,	"adv_cap_10fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_10hdx,	"adv_cap_10hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_remfault,	"adv_rem_fault",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_adv_cap_autoneg,	"adv_cap_autoneg",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_1000fdx, "lp_cap_1000fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_1000hdx, "lp_cap_1000hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_100fdx,	"lp_cap_100fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_100hdx,	"lp_cap_100hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_10fdx,		"lp_cap_10fdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_10hdx,		"lp_cap_10hdx",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_remfault,	"lp_cap_rem_fault",
		KSTAT_DATA_UINT32);
	kstat_named_init(&knp->ks_lp_cap_autoneg,	"lp_cap_autoneg",
		KSTAT_DATA_UINT32);

	ksp->ks_private = (void *) dp;
	ksp->ks_update = gem_kstat_update;
	dp->ksp = ksp;

	kstat_install(ksp);

	return (GEM_SUCCESS);
}
#endif

#define	GEM_LOCAL_DATA_SIZE(gc)	\
	(sizeof(struct gem_dev) + \
	sizeof(struct mcast_addr) * GEM_MAXMC + \
	sizeof(struct txbuf) * ((gc)->gc_tx_buf_size) + \
	sizeof(void *) * ((gc)->gc_tx_buf_size))

struct gem_dev *
gem_do_attach(dev_info_t *dip,
	struct gem_conf *gc, void *base, ddi_acc_handle_t *regs_handlep,
	void *lp, int lmsize)
{
	struct gem_dev		*dp;
	int			i;
	ddi_iblock_cookie_t	c;
#ifdef GEM_CONFIG_GLDv3
	mac_register_t		*macp = NULL;
#else
	gld_mac_info_t		*macinfo;
#endif
	int			ret;
	int			unit;

	unit = ddi_get_instance(dip);

	DPRINTF(2, (CE_CONT, "!gem%d: gem_do_attach: called cmd:ATTACH",
		unit));


	/*
	 * Allocate soft data structure
	 */
	dp = (struct gem_dev *) kmem_zalloc(GEM_LOCAL_DATA_SIZE(gc), KM_SLEEP);

#ifdef GEM_CONFIG_GLDv3
	if ((macp = mac_alloc(MAC_VERSION)) == NULL) {
		cmn_err(CE_WARN, "!gem%d: %s: mac_alloc failed",
			unit, __func__);
		return (NULL);
	}
	ddi_set_driver_private(dip, dp);
#else
	macinfo = gld_mac_alloc(dip);
	dp->macinfo = macinfo;
#endif

	/* link to private area */
	dp->private   = lp;
	dp->priv_size = lmsize;
	dp->mc_list = (struct mcast_addr *) &dp[1];

	dp->dip = dip;
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
	cv_init(&dp->tx_drain_cv, NULL, CV_DRIVER, NULL);

	/*
	 * configure gem parameter
	 */
	dp->base_addr   = base;
	dp->regs_handle = *regs_handlep;
	STRUCT_COPY(dp->gc, *gc);
	gc     = &dp->gc;

	/* fix copy threadsholds */
	dp->gc.gc_tx_copy_thresh = max(ETHERMIN, dp->gc.gc_tx_copy_thresh);
	dp->gc.gc_rx_copy_thresh = max(ETHERMIN, dp->gc.gc_rx_copy_thresh);

	/* fix rx buffer boundary for iocache line size */
	ASSERT(gc->gc_dma_attr_txbuf.dma_attr_align-1 == gc->gc_tx_buf_align);
	ASSERT(gc->gc_dma_attr_rxbuf.dma_attr_align-1 == gc->gc_rx_buf_align);
	gc->gc_rx_buf_align = max(gc->gc_rx_buf_align, IOC_LINESIZE - 1); 
	gc->gc_dma_attr_rxbuf.dma_attr_align = gc->gc_rx_buf_align + 1;

	/* fix get_packet method */
	if (gc->gc_get_packet == NULL) {
		gc->gc_get_packet = &gem_get_packet_default;
	}

	/* calculate descriptor area */
	if (gc->gc_rx_desc_unit_shift >= 0) {
		dp->rx_desc_size =
		    ROUNDUP(gc->gc_rx_ring_size << gc->gc_rx_desc_unit_shift,
			gc->gc_dma_attr_desc.dma_attr_align);
	}
	if (gc->gc_tx_desc_unit_shift >= 0) {
		dp->tx_desc_size =
		    ROUNDUP(gc->gc_tx_ring_size << gc->gc_tx_desc_unit_shift,
			gc->gc_dma_attr_desc.dma_attr_align);
	}

	dp->mtu		   = ETHERMTU;
	dp->tx_buf	   = (struct txbuf *) &dp->mc_list[GEM_MAXMC];
	/* link tx buffers */
	for (i = 0; i < dp->gc.gc_tx_buf_size; i++) {
		dp->tx_buf[i].txb_next =
			&dp->tx_buf[SLOT(i + 1, dp->gc.gc_tx_buf_size)];
	}

	dp->rxmode	   = 0;
	dp->mii_fixedmode  = B_FALSE;		/* default is autoneg */
	dp->speed	   = GEM_SPD_10;	/* default is 10Mbps */
	dp->full_duplex    = B_FALSE;		/* default is half */
	dp->flow_control   = FLOW_CONTROL_NONE;
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
	dp->nic_state = NIC_STATE_STOPPED;
	ret = (*dp->gc.gc_reset_chip)(dp);
	mutex_exit(&dp->intrlock);
	if (ret != GEM_SUCCESS) {
		goto err_free_regs;
	}

	/*
	 * HW dependant paremeter initialization
	 */
	mutex_enter(&dp->intrlock);
	ret = (*dp->gc.gc_attach_chip)(dp);
	mutex_exit(&dp->intrlock);
	if (ret != GEM_SUCCESS) {
		goto err_free_regs;
	}

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

#ifdef GEM_CONFIG_GLDv3
	gem_gld3_init(dp, macp);
#else
	gem_gld_init(dp, macinfo, ident);
#endif

	/* Init MII (scan phy)*/
	if (dp->gc.gc_mii_init != NULL) {
		if ((*dp->gc.gc_mii_init)(dp) != 0) {
			goto err_free_ring;
		}
	}

	/*
	 * initialize kstats including mii statistics
	 */
#ifndef GEM_CONFIG_GLDv3
	if (gem_kstat_init(dp) != GEM_SUCCESS) {
		goto err_free_ring;
	}
#endif
	/* reset_mii and start mii link watcher */
	gem_mii_start(dp);

	/*
	 * Add interrupt to system.
	 */
#ifdef GEM_CONFIG_GLDv3
	if ((ret = mac_register(macp, &dp->mh)) != 0) {
		cmn_err(CE_WARN, "!%s: mac_register failed, error:%d",
			dp->name, ret);
		goto err_stop_mii;
	}
	mac_free(macp);
	macp = NULL;
#else
	if (gld_register(dip,
			(char *)ddi_driver_name(dip), macinfo) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: gld_register failed", dp->name);
		goto err_stop_mii;
	}
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
#ifdef GEM_CONFIG_GLDv3
		if (ddi_add_intr(dip, 0, NULL, NULL,
					(u_int (*)(caddr_t))gem_intr,
					(caddr_t)dp) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s: ddi_add_intr failed", dp->name);
			goto err_unregister;
		}
#else
		if (ddi_add_intr(dip, 0, NULL, NULL, gld_intr,
					(caddr_t)macinfo) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s: ddi_add_intr failed", dp->name);
			goto err_unregister;
		}
#endif
	}
	else {
		/*
		 * Dont use interrupt.
		 * schedule first call of gem_intr_watcher
		 */
		dp->intr_watcher_id =
			timeout((void (*)(void *))gem_intr_watcher,
				(void *)dp, drv_usectohz(3*1000000));
	}

	DPRINTF(2, (CE_CONT, "!gem_do_attach: return: success"));
	return (dp);

err_unregister:
#ifdef GEM_CONFIG_GLDv3
	mac_unregister(dp->mh);
#else
	gld_unregister(macinfo);
#endif
err_stop_mii:
	gem_mii_stop(dp);

#ifndef GEM_CONFIG_GLDv3
	kstat_delete(dp->ksp);
#endif

err_free_ring:
	gem_free_memory(dp);
err_free_regs:
	ddi_regs_map_free(&dp->regs_handle);
err_free_locks:
	mutex_destroy(&dp->xmitlock);
	mutex_destroy(&dp->intrlock);
	cv_destroy(&dp->tx_drain_cv);
err_free_private:
#ifdef GEM_CONFIG_GLDv3
	if (macp != NULL) {
		mac_free(macp);
	}
#else
	gld_mac_free(macinfo);
#endif
	kmem_free((caddr_t)dp, GEM_LOCAL_DATA_SIZE(gc));

	return (NULL);
}

int
gem_do_detach(dev_info_t *dip)
{
	struct gem_dev	*dp;
	timeout_id_t	old_id;

	dp = GEM_GET_DEV(dip);

	if (dp == NULL) {
		/* gem_do_attach has been failed */
		return DDI_SUCCESS;
	}

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

#ifndef GEM_CONFIG_GLDv3
	/*
	 * destroy kstat objects
	 */
	kstat_delete(dp->ksp);
#endif
	/*
	 * unregister interrupt handler
	 */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		ddi_remove_intr(dip, 0, dp->iblock_cookie);
	}
#ifdef GEM_CONFIG_GLDv3
	(void)mac_unregister(dp->mh);
#else
	(void)gld_unregister(dp->macinfo);
#endif
	gem_free_memory(dp);

	/*
	 * Release mapping resources
	 */
	ddi_regs_map_free(&dp->regs_handle);

	mutex_destroy(&dp->xmitlock);
	mutex_destroy(&dp->intrlock);
	cv_destroy(&dp->tx_drain_cv);

	/*
	 * Release memory resources
	 */
	kmem_free((caddr_t)(dp->private), dp->priv_size);
#ifndef GEM_CONFIG_GLDv3
	gld_mac_free(dp->macinfo);
#endif
	kmem_free((caddr_t)dp, GEM_LOCAL_DATA_SIZE(&dp->gc));


	DPRINTF(2, (CE_CONT, "!%s%d: gem_do_detach: return: success",
			ddi_driver_name(dip), ddi_get_instance(dip)));

	return (DDI_SUCCESS);
}

int
gem_suspend(dev_info_t *dip)
{
	struct gem_dev	*dp;
	timeout_id_t	old_id;

	dp = GEM_GET_DEV(dip);

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

	/* stop tx timeout watcher */
	if (dp->timeout_id) {
		do {
			untimeout(old_id = dp->timeout_id);
		} while (dp->timeout_id != old_id);	
		dp->timeout_id = 0;
	}

	/* make the nic state inactive */
	mutex_enter(&dp->intrlock);
	(void) gem_mac_stop(dp, 0);
	mutex_exit(&dp->intrlock);

	return (DDI_SUCCESS);
}

int
gem_resume(dev_info_t *dip)
{
	struct gem_dev	*dp;

	dp = GEM_GET_DEV(dip);

	/*
	 * restart the device
	 */
	mutex_enter(&dp->intrlock);

	/* restart mac */
	gem_restart_nic(dp, B_FALSE);

	/* restart mii watcher */
	gem_mii_start(dp);

	/* restart tx timeout watcher */
	dp->timeout_id = timeout((void (*)(void *))gem_tx_timeout,
				(void *)dp, dp->gc.gc_tx_timeout_interval);

	if ((dp->misc_flag & GEM_NOINTR) != 0) {
		/*
		 * Dont use interrupt.
		 * schedule first call of gem_intr_watcher
		 */
		dp->intr_watcher_id =
			timeout((void (*)(void *))gem_intr_watcher,
				(void *)dp, drv_usectohz(3*1000000));
	}
	mutex_exit(&dp->intrlock);

	return (DDI_SUCCESS);
err:
#ifdef GEM_CONFIG_FMA
	ddi_fm_service_impact(dip, DDI_SERVICE_DEGRADED);
#endif
	return (DDI_FAILURE);
}

/*
 * misc routines for PCI
 */
static uint8_t
gem_search_pci_cap(dev_info_t *dip,
		ddi_acc_handle_t conf_handle, uint8_t target)
{
	uint8_t		pci_cap_ptr;
	uint32_t	pci_cap;

	/* search power management capablities */
	pci_cap_ptr = pci_config_get8(conf_handle, PCI_CONF_CAP_PTR);
	while (pci_cap_ptr != 0) {
		/* read pci capability header */
		pci_cap = pci_config_get32(conf_handle, pci_cap_ptr);
		if ((pci_cap & 0xff) == target) {
			/* found */
			break;
		}
		/* get next_ptr */
		pci_cap_ptr = (pci_cap >> 8) & 0xff;
	}
	return (pci_cap_ptr);
}

int
gem_pci_set_power_state(dev_info_t *dip,
		ddi_acc_handle_t conf_handle, uint_t new_mode)
{
	uint8_t		pci_cap_ptr;
	uint32_t	pmcsr;
	uint_t		unit;
	const char	*drv_name;

	ASSERT(new_mode < 4);

        unit =  ddi_get_instance(dip);
        drv_name = ddi_driver_name(dip);

	/* search power management capablities */
	pci_cap_ptr = gem_search_pci_cap(dip, conf_handle, PCI_CAP_ID_PM);

	if (pci_cap_ptr == 0) {
		cmn_err(CE_CONT,
			"!%s%d: doesn't have pci power management capability",
			drv_name, unit);
		return DDI_FAILURE;
	}

	/* read power management capabilities */
	pmcsr = pci_config_get32(conf_handle, pci_cap_ptr + PCI_PMCSR);

	DPRINTF(0, (CE_CONT,
		"!%s%d: pmc found at 0x%x: pmcsr: 0x%08x",
		drv_name, unit, pci_cap_ptr, pmcsr));

	/*
	 * Is the resuested power mode supported?
	 */
	/* not yet */

	/*
	 * move to new mode
	 */
	pmcsr = (pmcsr & ~PCI_PMCSR_STATE_MASK) | new_mode;
	pci_config_put32(conf_handle, pci_cap_ptr + PCI_PMCSR, pmcsr);

	return (DDI_SUCCESS);
}

boolean_t
gem_is_pcie(dev_info_t *dip, ddi_acc_handle_t conf_handle)
{
	return gem_search_pci_cap(dip, conf_handle, 0x09) != 0;
}

int
gem_pci_regs_map_setup(dev_info_t *dip, uint32_t which, uint32_t mask,
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
		return ret;
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
		if ((regs[i].pci_phys_hi & mask) == which) {
			/* it's the requested space */
			ddi_prop_free(regs);
			goto address_range_found;
		}
	}
	ddi_prop_free(regs);
	return (DDI_FAILURE);

address_range_found:
	if ((ret = ddi_regs_map_setup(dip, i, basep, 0, 0, attrp, hp))
			!= DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed (ret:%d)",
			drv_name, unit, ret);
	}

	return (ret);
}

void
gem_mod_init(struct dev_ops *dop, char *name)
{
#ifdef GEM_CONFIG_GLDv3
	mac_init_ops(dop, name);
#endif
}

void
gem_mod_fini(struct dev_ops *dop)
{
#ifdef GEM_CONFIG_GLDv3
	mac_fini_ops(dop);
#endif
}
