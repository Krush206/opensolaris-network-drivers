/*
 * gani_gem.c : The Realtek RTL8169 Gigabit Ethernet Driver for Solaris
 *
 * Copyright (c) 2004, 2005 Masayuki Murayama.  All rights reserved.
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

#pragma	ident	"@(#)gani_gem.c	1.17 05/08/27"

/*
 *  Changelog:

 2005/02/26
	gani_set_rx_filter was fixed to use ALLMULTI mode when the number
	of multicast addresses was greater than or equal to 32, instead of
	promiscious mode.

	the performance of gani_set_rx_filter was improved. 
	ID registers had been updated on every call of it.

 2005/04/23
	rx buffer setup problem in gani_init_chip & gani_start_chip fixed.

 2005/05/09
	0.8.16

 2005/05/09
	changing mac address did not work
	compile error on __FUNCTION__ with Sun CC
	0.8.17

 2005/05/25
	lp->mac updated after loading EEPROM contents
 2005/06/11
	0.8.18

 2005/08/09
	0.8.19

 */

/*
 * TODO
	flow control: done
	hang/panic at boot time on sparc
   BUGS
	Don't enable hardware checksum offloading, it's slow.
 */

/* ======================================================= */

/*
 * Solaris system header files and macros
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
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/ddi_impldefs.h>

#include <sys/pci.h>

#include "gem.h"
#include "mii.h"
#include "rtl8169reg.h"

char	ident[] = "rtl8169 nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/*
 * Useful macros
 */
#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))
#define	FALSE	(0)
#define	TRUE	(!FALSE)

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

#define	ONESEC			(drv_usectohz(1*1000000))

#define	DESC_BASE_ALIGN	256	/* it should be defined in rtl8169reg.h */

#define	TALLY_OFFSET	\
	(ROUNDUP2(sizeof(struct rx_desc) * RX_RING_SIZE, DESC_BASE_ALIGN) + \
	 ROUNDUP2(sizeof(struct tx_desc) * TX_RING_SIZE, DESC_BASE_ALIGN))
#define	TALLY_VADDR(dp)	((dp)->rx_ring + TALLY_OFFSET)
#define	TALLY_DMA(dp)	((dp)->rx_ring_dma + TALLY_OFFSET)

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif

#define CONFIG_EEPROM_IO

/*
 * Experimental: polling mode support
 */
#define	GANI_IS_POLLING(lp)		(((lp)->imr & INTR_ROK) == 0)

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int gani_debug = DEBUG_LEVEL;
#define DPRINTF(n, args)	if (gani_debug > (n)) cmn_err args
#else
#define DPRINTF(n, args)
#endif

#define	DUMP_ETHER(n, dp, m) \
	DPRINTF(n, (CE_CONT, "!%s: %s: mac: %02x:%02x:%02x:%02x:%02x:%02x", \
		(dp)->name, __func__, \
		(m)[0], (m)[1], (m)[2], (m)[3], (m)[4], (m)[5]));
/*
 * Our configration for rtl8169
 */
#ifndef	TX_MAX_FRAGS
# define	TX_MAX_FRAGS	1
#endif

#ifdef TEST_TXDESC_FULL
# define TX_RING_SIZE	(DESC_BASE_ALIGN / sizeof(struct tx_desc)) /**/
/*# define TX_RING_SIZE	8	/**/
#endif
#ifdef TEST_RX_EMPTY
# define RX_BUF_SIZE	1
#endif

#ifndef	TX_BUF_SIZE
# define TX_BUF_SIZE	256
#endif
#ifndef	TX_RING_SIZE
# define TX_RING_SIZE	(TX_BUF_SIZE*4)
#endif

#ifndef	RX_BUF_SIZE
# define RX_BUF_SIZE	256
#endif
#define	RX_RING_SIZE	1024		/* was RX_BUF_SIZE */

#define	OUR_INTR_MASK	\
	(INTR_SERR | INTR_SWInt | INTR_FOVW | INTR_RDU | \
	 INTR_TDU | INTR_TER | INTR_RER | INTR_ROK)
#if defined(sun4u)
static int	gani_tx_copy_thresh = INT32_MAX;
static int	gani_rx_copy_thresh = INT32_MAX;
#else
static int	gani_tx_copy_thresh = 256;
static int	gani_rx_copy_thresh = INT32_MAX;
#endif
/*
 * Local device definitions
 */
struct chip_info {
	uint32_t	tcr_val; /* from RTL8169 docs */
	int		flags;
	char		*name;
	int		type;
};

#define	MAC_VER_B	0
#define	MAC_VER_D	1
#define	MAC_VER_E	2
#define	MAC_VER_F	3

struct chip_info chiptbl_8169[] = {
	{TCR_MACVER_B,	0,	"rtl8169 ver.B",	MAC_VER_B},
	{TCR_MACVER_D,	0,	"rtl8169s/8110s ver.D",	MAC_VER_D},
	{TCR_MACVER_E,	0,	"rtl8169s/8110s ver.E",	MAC_VER_E},
	{TCR_MACVER_F,	0,	"rtl8169s/8110s ver.F",	MAC_VER_F},
};

#define	CHIPTABLESIZE	(sizeof(chiptbl_8169)/sizeof(struct chip_info))

struct gani_dev {
	/*
	 * Misc HW information
	 */
	struct chip_info	*chip;

	uint8_t			mac[ETHERADDRL];
	uint8_t			cfg2;

	uint32_t		rcr;
	boolean_t		need_to_reset;
	uint32_t		imr;
	uint32_t		imr_hw;
	uint32_t		imr_disabled;

	clock_t			last_stats_time;

	int			tx_list_len;
	boolean_t		tx_active;
	boolean_t		tx_active_pended;
#ifdef CONFIG_POLLING
	int			last_poll_interval; /* polling interval in uS */
#endif
	struct rtl8169_tally_counters	last_stat;
};

/*
 * private functions
 */
 
/* mii operations */
static void  gani_mii_sync(struct gem_dev *);
static uint16_t gani_mii_read(struct gem_dev *, uint_t);
static void gani_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int gani_attach_chip(struct gem_dev *);
static int gani_reset_chip(struct gem_dev *);
static void gani_init_chip(struct gem_dev *);
static void gani_start_chip(struct gem_dev *);
static void gani_stop_chip(struct gem_dev *);
static void gani_set_media(struct gem_dev *);
static void gani_set_rx_filter(struct gem_dev *);
static void gani_get_stats(struct gem_dev *);

/* descriptor operations */
static int gani_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq);
static int gani_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t gani_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t gani_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void gani_tx_desc_init(struct gem_dev *dp, int slot);
static void gani_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int gani_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr gani_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr gani_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t gani_dma_attr_buf32 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t gani_dma_attr_buf64 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffffffull,	/* dma_attr_addr_hi */
	0xffffffffffffull,	/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t gani_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	DESC_BASE_ALIGN,	/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* =============================================================== */
/*
 * Debugging support
 */
/* =============================================================== */
#ifdef DEBUG_LEVEL
static void
gani_dump_regs(struct gem_dev *dp, char *m)
{
	int		i;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	cmn_err(CE_CONT, "!%s: register dump (%s)", dp->name, m);

	for (i = 0; i < 256; i += 8) {
		cmn_err(CE_CONT,
			"!%02x: %02x %02x %02x %02x %02x %02x %02x %02x",
			i,
			INB(dp, i+0), INB(dp, i+1),
			INB(dp, i+2), INB(dp, i+3),
			INB(dp, i+4), INB(dp, i+5),
			INB(dp, i+6), INB(dp, i+7));
	}
}
#endif
/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
gani_reset_chip(struct gem_dev *dp)
{
	int		i;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: gani_reset_chip: called", dp->name));
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "reset_chip 1");
#endif
	/* disable interrupts */
	OUTW(dp, IMR, lp->imr_hw = 0);

	OUTB(dp, CR, CR_RST);
	drv_usecwait(1000);

	for (i = 0; (INB(dp, CR) & CR_RST) != 0; i++) {
		if (i > 100) {
			/* time out */
			cmn_err(CE_WARN, "!%s: failed to reset: timeout",
					dp->name);
			return -1;
		}
		drv_usecwait(10);
	}

	/* undocumented patch */
	OUTB(dp, 0x82, 0x01);

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "reset_chip 2");
#endif
	return 0;
}

/*
 * Setup rtl8169
 */
static void
gani_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: gani_init_chip: called", dp->name));

	/* XXX - disable interrupts */
	OUTW(dp, IMR, lp->imr_hw = 0);

	/* ID registers will be set later by gani_set_rx_filter */

	/* Multicast hash registers will be  set later by gani_set_rx_filter */

	/* make configuration registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	/* CpCR */
	val = INW(dp, CpCR);
	OUTW(dp, CpCR, val);	/* XXX */
	if (lp->chip->type == MAC_VER_D || lp->chip->type == MAC_VER_E) {
		/* what is this ? */
		val |= 0x4000;
	}
#ifdef CONFIG_VLAN
	val |= CpCR_RxVLAN;
#endif
	OUTW(dp, CpCR, val | CpCR_MulRW);

	/* XXX - undocumented */
	OUTW(dp, 0xe2, 0);

	/* Transmit Normal Priority Descriptors */
	DPRINTF(4 , (CE_CONT, "!%s: TNPDS 0x%llx", dp->name, dp->tx_ring_dma));
	OUTL(dp, TNPDSL, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, TNPDSH, (uint32_t)(dp->tx_ring_dma >> 32));


	/* Transmit High Priority Descriptors: Do nothing */
	OUTL(dp, THPDSL, 0);
	OUTL(dp, THPDSH, 0);

	/* Receive Descriptor Start Address register */
	OUTL(dp, RDSARL, (uint32_t)dp->rx_ring_dma);
	OUTL(dp, RDSARH, (uint32_t)(dp->rx_ring_dma >> 32));
	DPRINTF(4 , (CE_CONT, "!%s: RDSAR 0x%llx", dp->name, dp->rx_ring_dma));

	/* make config registers read only */
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	drv_usecwait(10);

	/* Dump Tally Count Register : */
	OUTL(dp, DTCCRH, (uint32_t)(TALLY_DMA(dp) >> 32));
#ifdef SANITY
	OUTL(dp, DTCCRL, 0);
#endif
	bzero(&lp->last_stat, sizeof(struct rtl8169_tally_counters));

	/* Flash memory read/wirte register: no need to touch */

	/* Interrupt status register */
	OUTW(dp, ISR, 0xffff);

	/* TimerInt Register */
	OUTL(dp, TimerInt, 0);

	/* Timer count register: test count rate */
	OUTL(dp, TCTR, 0);
#if DEBUG_LEVEL > 3
	drv_usecwait(1000);
	cmn_err(CE_CONT, "!%s: TCTR:%d in 1mS", dp->name, INL(dp, TCTR));
#endif
	/*
	 * Receive control register: don't touch here
	 *
	 * XXX - Keep undocumented bits.
	 * This also implies to disable receiving packets.
	 */
	lp->rcr = INL(dp, RCR) & ~RCR_MASK;

	lp->tx_list_len = 0;
	lp->tx_active = FALSE;
	lp->tx_active_pended = FALSE;

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "init_chip done");
#endif
}

static uint_t
gani_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return gem_ether_crc_be(addr);
}

static void
gani_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	uint32_t	mhash[2];
	int		i;
	uint8_t		*m;
	uint8_t		reg;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: gani_set_rx_mode: called", dp->name));

	mode = RCR_AB	/* accept broadcast */
	     | RCR_APM 	/* accept physical match  */
	     | RCR_AER | RCR_AR | RCR_RER8; /* accept error and runt packets */

	mhash[0] = 0;
	mhash[1] = 0;

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		/* promiscious mode implies all multicast and all physical */
		mode |= RCR_AM | RCR_AAP;
		mhash[0] = 0xffffffff;
		mhash[1] = 0xffffffff;
	}	
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 || dp->mc_count > 32) {
		/* accept all multicast packets */
		mode |= RCR_AM;
		mhash[0] = 0xffffffff;
		mhash[1] = 0xffffffff;
	}
	else if (dp->mc_count > 0) {
		mode |= RCR_AM;
		/*
		 * make hash table to select interresting
		 * multicast address only.
		 */
		for (i = 0; i < dp->mc_count; i++) {
			uint_t	h;
			/* hash table is 64 = 2^6 bit width */
			h = dp->mc_list[i].hash >> (32 - 6);
			mhash[h / 32] |= 1 << (h % 32);
		}
	}	

	lp->rcr &= ~(RCR_ACCEPT_MODE | RCR_RER8);
	if (dp->nic_active) {
		/*
		 * disable all kind of rx filters to stop rx before
		 * changing mac address and/or multicast hash table.
		 */
		OUTL(dp, RCR, lp->rcr);
		FLSHL(dp, RCR);
	}

	m = &dp->cur_addr.ether_addr_octet[0];
	if (bcmp(m, lp->mac, ETHERADDRL) != 0) {
		/*
		 * XXX - make config registers writable before
		 * changing IDRs. (undocumented)
		 */
		reg = INB(dp, CR9346);
		OUTB(dp, CR9346, CR9346_EEM_WE);

		/* set mac address */	
		OUTL(dp, IDR + 0,
			(m[3] << 24) | (m[2] << 16) | (m[1] << 8) | m[0]);
		OUTL(dp, IDR + 4, (m[5] <<  8) | m[4]);

		OUTB(dp, CR9346, reg);
		drv_usecwait(10);
	}

	if ((mode & RCR_AM) != 0) {
		/* need to set up multicast hash table */
		OUTL(dp, MAR + 0, mhash[0]);
		OUTL(dp, MAR + 4, mhash[1]);
	}

	/* update rcr */
	lp->rcr |= mode;
	if (dp->nic_active) {
		OUTL(dp, RCR, lp->rcr);
	}

	DPRINTF(4, (CE_CONT, "!gani_set_rx_filter: returned"));
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "set_rxfilter done");
#endif
}

static void
gani_set_media(struct gem_dev *dp)
{
	struct gani_dev	*lp = (struct gani_dev *)(dp)->private;

	DPRINTF(4, (CE_CONT,
		"!gani_set_media: phys: %b, nic_active: %d",
		INB(dp, PHYS), PHYS_BITS, dp->nic_active));

	/* do nothing */
}

static void
gani_start_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: gani_start_chip: called", dp->name));

	ASSERT(dp->mii_state == MII_STATE_LINKUP ||
	       dp->mii_state == MII_STATE_LINKDOWN);

	/*
	 * Kick Rx and Tx
	 */

	/* Command register : Enable Tx and Rx before writing TCR and RCR */
	OUTB(dp, CR, CR_RE | CR_TE);

	/*
	 * Rx packet Maxmum size:
	 * XXX - I think actual that max packet size will be limited by
	 * tx fifo size, aka  8K-1.
	 */
	OUTW(dp, RMS, sizeof(struct ether_header) + dp->mtu + ETHERFCSL + 4);
	DPRINTF(2 , (CE_CONT, "!%s: RMS 0x%x", dp->name, INW(dp, RMS)));

	/* Receive control register: write an initial value */
	val = 0;
	if (dp->rxmaxdma <= 64) {
		val |= RCR_MXDMA_64;
	} else if (dp->rxmaxdma<= 128) {
		val |= RCR_MXDMA_128;
	} else if (dp->rxmaxdma<= 256) {
		val |= RCR_MXDMA_256;
	} else if (dp->rxmaxdma<= 512) {
		val |= RCR_MXDMA_512;
	} else if (dp->rxmaxdma<= 1024) {
		val |= RCR_MXDMA_1024;
	} else {
		val |= RCR_MXDMA_UNLIMITED;
	}
	if (dp->rxthr <= 64) {
		val |= RCR_RXFTH_64;
	} else if (dp->rxthr <= 128) {
		val |= RCR_RXFTH_128;
	} else if (dp->rxthr <= 256) {
		val |= RCR_RXFTH_256;
	} else if (dp->rxthr <= 512) {
		val |= RCR_RXFTH_512;
	} else if (dp->rxthr <= 1024) {
		val |= RCR_RXFTH_1024;
	} else {
		val |= RCR_RXFTH_NONE;	/* store and forward */
	}
	lp->rcr |= val;
	OUTL(dp, RCR, lp->rcr);

	/* Early Tx register */
	OUTB(dp, ETThR, min(dp->txthr / ETThR_UNIT, ETThR_MASK));
	DPRINTF(2 , (CE_CONT, "!%s: ETThR 0x%x", dp->name, INB(dp, ETThR)));

	/* Transmit configration register : */
	if (dp->txmaxdma <= 16) {
		val = TCR_MXDMA_16;
	} else if (dp->txmaxdma <= 32) {
		val = TCR_MXDMA_32;
	} else if (dp->txmaxdma <= 64) {
		val = TCR_MXDMA_64;
	} else if (dp->txmaxdma <= 128) {
		val = TCR_MXDMA_128;
	} else if (dp->txmaxdma <= 256) {
		val = TCR_MXDMA_256;
	} else if (dp->txmaxdma <= 512) {
		val = TCR_MXDMA_512;
	} else if (dp->txmaxdma <= 1024) {
		val = TCR_MXDMA_1024;
	} else {
		val = TCR_MXDMA_UNLIMITED;
	}
	OUTL(dp, TCR, TCR_IFG_802_3 | TCR_LBK_NORMAL | val);

	/* Missed packet counter: clear it */
	OUTL(dp, MPC, 0);

	/* clear tally counter area */
	DPRINTF(2, (CE_CONT, "!%s: tally base: vaddr:0x%p, dma addr:0x%p",
		dp->name, TALLY_VADDR(dp), TALLY_DMA(dp)));
	bzero(TALLY_VADDR(dp), TC_SIZE);
	lp->last_stats_time = ddi_get_lbolt();

	/* MulInt Register */
	OUTW(dp, MULINT, INW(dp, MULINT) & 0xf000);

	/* Enable interrupts */
	lp->imr = OUR_INTR_MASK;	/* logical imterrupt mask */
	lp->imr_disabled = 0;

#ifdef CONFIG_POLLING
	lp->imr |= INTR_TimeOut;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IMR, lp->imr_hw = lp->imr);
	}
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "start_chip done");
#endif
}

static void
gani_stop_chip(struct gem_dev *dp)
{
	int		i;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: gani_stop_chip: called", dp->name));

	/* wait for dumping tally counters stops. */
	for (i = 0; (INL(dp, DTCCRL) & DTCCR_CMD) != 0; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: timeout: dumping tally counters",
				dp->name);
			break;
		}
		drv_usecwait(10);
	}

	/* disable interrupts */
	lp->imr = 0;
	OUTW(dp, IMR, lp->imr_hw = lp->imr);

	/* disable rx filter for preparing the next gani_start_chip() */
	lp->rcr &= ~(RCR_ACCEPT_MODE | RCR_RER8);
	OUTL(dp, RCR, lp->rcr);

	/* disable RX and TX */
	OUTB(dp, CR, 0);

	/* Clear pended interrupt */
	OUTW(dp, ISR, 0xffff); FLSHW(dp, ISR);
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "stop_chip done");
#endif
}

static void
gani_get_stats(struct gem_dev *dp)
{
	uint32_t			x;
	struct rtl8169_tally_counters	*tbp;
	struct rtl8169_tally_counters	*ls;
	struct rtl8169_tally_counters	new;
	struct gani_dev			*lp = (struct gani_dev *)dp->private;
	ddi_acc_handle_t		h = dp->desc_acc_handle;

	DPRINTF(4, (CE_CONT, "!%s: gani_get_stats: called", dp->name));

	if (!dp->nic_active) {
		/* nic isn't running yet */
		return;
	}

	lp->last_stats_time = ddi_get_lbolt();

	x = INL(dp, DTCCRL);

	if ((x & DTCCR_CMD) != 0) {
		/* Now dumping tally counters, do nothing */
		return;
	}

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)TALLY_OFFSET, (size_t)TC_SIZE, DDI_DMA_SYNC_FORKERNEL);

	tbp = (struct rtl8169_tally_counters *)TALLY_VADDR(dp);

	new.tc_txok    = ddi_get64(h, &tbp->tc_txok);		/* 8byte */
	new.tc_rxok    = ddi_get64(h, &tbp->tc_rxok);		/* 8byte */
	new.tc_txer    = ddi_get64(h, &tbp->tc_txer);		/* 8byte */
	new.tc_rxer    = ddi_get32(h, &tbp->tc_rxer);		/* 4byte */
	new.tc_misspkt = ddi_get16(h, &tbp->tc_misspkt);	/* 2byte */
	new.tc_fae     = ddi_get16(h, &tbp->tc_fae);		/* 2byte */
	new.tc_tx1col  = ddi_get32(h, &tbp->tc_tx1col);		/* 4byte */
	new.tc_txmcol  = ddi_get32(h, &tbp->tc_txmcol);		/* 4byte */
	new.tc_rxokphy = ddi_get64(h, &tbp->tc_rxokphy);	/* 8byte */
	new.tc_rxokbrd = ddi_get64(h, &tbp->tc_rxokbrd);	/* 8byte */
	new.tc_rxokmu  = ddi_get32(h, &tbp->tc_rxokmu);		/* 4byte */
	new.tc_txabt   = ddi_get16(h, &tbp->tc_txabt);		/* 2byte */
	new.tc_txundrn = ddi_get16(h, &tbp->tc_txundrn);	/* 2byte */

	ls = &lp->last_stat;
	dp->stats.errxmt	+= new.tc_txer       - ls->tc_txer;
	dp->stats.errrcv	+= new.tc_rxer       - ls->tc_rxer;
	dp->stats.missed	+= new.tc_misspkt    - ls->tc_misspkt;
	dp->stats.frame		+= new.tc_fae        - ls->tc_fae;
	dp->stats.first_coll	+= x = new.tc_tx1col - ls->tc_tx1col;
	dp->stats.collisions += x;
	dp->stats.multi_coll	+= x = new.tc_txmcol - ls->tc_txmcol;
	dp->stats.collisions += x*2;
	dp->stats.excoll	+= new.tc_txabt        - ls->tc_txabt;
	dp->stats.underflow	+= new.tc_txundrn    - ls->tc_txundrn;

	STRUCT_COPY(lp->last_stat, new);

	/* issue Dump_Tally_Counters_Cmd for the next call */
	OUTL(dp, DTCCRL, DTCCR_CMD | (uint32_t) TALLY_DMA(dp));
}

/*
 * discriptor  manupiration
 */
static int
gani_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag)
{
	int			i;
	struct tx_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	uint_t			vlan_tag;
	struct gani_dev		*lp = (struct gani_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;
	int			curslot;
	int			pad = 0;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"!%s: %s: seqnum %d, slot %d, frags %d flag 0x%x",
		dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr 0x%x, len 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TXD0_OWN | TXD0_LS;

#if defined(WA_TX_DESC_NO_WRAP)
#  if DEBUG_LEVEL > 0
	if (slot + frags > TX_RING_SIZE) {
		cmn_err(CE_WARN, "%s: tx desc w/ wrap", dp->name);
	}
#  endif
	if (TX_RING_SIZE - slot < 2*TX_MAX_FRAGS) {
		pad = TX_RING_SIZE - slot - frags;
		mark |= TXD0_EOR;
		if ((flag & GEM_TXFLAG_INTR) == 0) {
			cmn_err(CE_WARN, "%s: missing tx intr", dp->name);
		}
	}
#endif /* WA_TX_DESC_WRAP */

#if TX_MAX_FRAGS > 1
	for (i = frags - 1, dcp = &dmacookie[frags - 1]; i > 0; i--, dcp--) {

		curslot = SLOT(slot + i, TX_RING_SIZE);

		/*
		 * specify descriptor control flags
		 */
#  ifndef WA_TX_DESC_NO_WRAP
		if (curslot == TX_RING_SIZE - 1) {
			mark |= TXD0_EOR;
		}
#  endif
		tdp = &((struct tx_desc *)dp->tx_ring)[curslot];
		ddi_put32(h, &tdp->txd3, dcp->dmac_laddress >> 32);
		ddi_put32(h, &tdp->txd2, dcp->dmac_laddress);
		ddi_put32(h, &tdp->txd1, 0);
		ddi_put32(h, &tdp->txd0, mark | dcp->dmac_size);

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);

		mark = TXD0_OWN;
	}
#endif
	/*
	 * specify descriptor control flags for first fragment.
	 */
	mark |= TXD0_FS;

#ifndef WA_TX_DESC_NO_WRAP
	if (slot == TX_RING_SIZE - 1) {
		mark |= TXD0_EOR;
	}
#endif

#if DEBUG_LEVEL > 0 && defined(WA_TPQ_NO_WRAP)
	if ((mark & TXD0_EOR) != 0) {
		/* here is at the end of the ring */
		if (SLOT(dp->tx_desc_head + TX_RING_SIZE - dp->tx_desc_rsvd,
				TX_RING_SIZE) != 0) {
			cmn_err(CE_WARN,
			"%s: wrong tx desc limit, tx desc %d[%d], %d[%d], %d",
				dp->name,
				dp->tx_desc_head,
				SLOT(dp->tx_desc_head, TX_RING_SIZE),
				dp->tx_desc_tail,
				SLOT(dp->tx_desc_tail, TX_RING_SIZE),
				dp->tx_desc_rsvd);
		}
	}
#endif

#ifdef CONFIG_CKSUM_OFFLOAD
	if ((flag & GEM_TXFLAG_IPv4) != 0) {
		mark |= TXD0_IPCS;
	}
	if ((flag & GEM_TXFLAG_UDP) != 0) {
		mark |= TXD0_UDPCS;
	}
	if ((flag & GEM_TXFLAG_TCP) != 0) {
		mark |= TXD0_TCPCS;
	}
#endif
	vlan_tag = 0;
#ifdef CONFIG_VLAN
	if ((flag & GEM_TXFLAG_VID) != 0) {
		uint_t	vid;
		vid = (flag & GEM_TXFLAG_VID) >> GEM_TXFLAG_VID_SHIFT;
		vlan_tag |= TXD1_TAGC | TXD1_VID(vid);
	}

	if ((flag & GEM_TXFLAG_PRI) != 0) {
		uint_t	pri;
		pri = (flag & GEM_TXFLAG_PRI) >> GEM_TXFLAG_PRI_SHIFT;
		vlan_tag |= pri << TXD1_PRI_SHIFT;
	}
#endif
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];
	ddi_put32(h, &tdp->txd3, dmacookie->dmac_laddress >> 32);
	ddi_put32(h, &tdp->txd2, dmacookie->dmac_laddress);
	ddi_put32(h, &tdp->txd1, vlan_tag);
	ddi_put32(h, &tdp->txd0, mark | dmacookie->dmac_size);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);

	if (dp->nic_active) {
		/* kick Tx engine */
#if DEBUG_LEVEL > 0 && defined(WA_TPQ_NO_WRAP)
		if (slot == 0 && lp->tx_active) {
			cmn_err(CE_WARN, "%s: TPPollNPQ w/ wrap", dp->name);
		}
#endif
		if (!lp->tx_active) {
			OUTB(dp, TPPoll, TPPoll_NPQ);
			lp->tx_active= TRUE;
			lp->tx_active_pended = FALSE;
		}
		else {
			lp->tx_active_pended = TRUE;
		}
	}

	lp->tx_list_len++;
	return frags + pad;
}

static int
gani_rx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t		mark;
	struct rx_desc		*rdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	mark = RXD0_OWN;
	if (slot == RX_RING_SIZE - 1) {
		mark |= RXD0_EOR;
	}

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	ddi_put32(h, &rdp->rxd3, dmacookie->dmac_laddress >> 32);
	ddi_put32(h, &rdp->rxd2, dmacookie->dmac_laddress);
	ddi_put32(h, &rdp->rxd1, 0);
	ddi_put32(h, &rdp->rxd0, mark | dmacookie->dmac_size);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);

	return 1;
}

static uint_t
gani_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	int		i;
	uint32_t	tsr;
	struct tx_desc	*tdp;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;
	int		frags = dp->tx_buf_head->txb_nfrags;

	/* XXX - we must check last descripor of the packet. */
	tdp = &((struct tx_desc *)dp->tx_ring)[
					SLOT(slot + frags - 1, TX_RING_SIZE)];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORKERNEL);

	tsr = ddi_get32(dp->desc_acc_handle, &tdp->txd0);

	DPRINTF(2, (CE_CONT,
		"!%s: gani_tx_desc_stat: slot:%d ndesc:%d isr:%b tsr:%b",
		dp->name, slot, ndesc,
		INW(dp, ISR), INTR_BITS, tsr, TXD0_BITS));

	if ((tsr & TXD0_OWN) != 0) {
		/* not transmitted yet */
		return 0;
	}
	lp->tx_list_len--;

#ifdef WA_TPQ_NO_WRAP
	if (slot == 0 || dp->tx_desc_rsvd > 0) {
		/* prevent the transmitted descriptors recycled for a while. */
		dp->tx_desc_rsvd += ndesc;
	}

	if (dp->tx_desc_intr - (dp->tx_desc_head + ndesc) <= 0) {
		/* no need further TOK interrupt */
		lp->imr &= ~INTR_TOK;
	}
#endif

	if (slot + ndesc != TX_RING_SIZE && (tsr & TXD0_EOR) != 0) {
		/* the descriptor is corrupted */
		cmn_err(CE_WARN,
		"%s: %s: tx descriptor corrupted, slot:%d ndesc:%d tsr:%b",
			dp->name, __func__, slot, ndesc, tsr, TXD0_BITS);
		return GEM_TX_DONE | GEM_TX_ERR;
	}

	return GEM_TX_DONE;
}

static uint64_t
gani_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint_t		len;
	uint32_t	rsr;
	struct rx_desc	*rdp;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;
#ifdef CONFIG_VLAN
	uint32_t	vtag;
#endif
	uint64_t	pflags = 0;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];
#ifndef NEW
	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORKERNEL);
#endif
	rsr = ddi_get32(dp->desc_acc_handle, &rdp->rxd0);
#ifdef CONFIG_VLAN
	vtag= ddi_get32(dp->desc_acc_handle, &rdp->rxd1);
#endif
	DPRINTF(2, (CE_CONT,
		"!%s: gani_rx_desc_stat: slot:%d rxd0:0x%b",
		dp->name, slot, rsr, RXD0_BITS));

	if ((rsr & RXD0_OWN) != 0) {
		/* not transmitted yet */
		return 0;
	}

	if ((rsr & RXD0_RES) != 0) {
		return GEM_RX_ERR;
	}

	if ((rsr & (RXD0_LS | RXD0_FS)) != (RXD0_LS | RXD0_FS)) {
		/* received frame is too long */
		if ((rsr & RXD0_FS) != 0) {
			dp->stats.frame_too_long++;
		}
		return GEM_RX_ERR;
	}

	len = rsr & RXD0_FRAMELEN;

	if (len < ETHERFCSL) {
		return GEM_RX_ERR;
	}
	len -= ETHERFCSL;

#ifdef CONFIG_VLAN
	if ((vtag & RXD1_TAVA) != 0) {
		/* fix vlan tag format */
		return ((vtag & 0xff) << 24) | ((vtag & 0xff00) << 8)
			| GEM_RX_DONE | len;
	}
#endif
#ifdef CONFIG_CKSUM_OFFLOAD
	switch ((rsr & RXD0_PID) >> RXD0_PID_SHIFT) {
	case RXD0_PID_IP >> RXD0_PID_SHIFT:
		if ((rsr & RXD0_IPF) == 0) {
			pflags |= GEM_RX_CKSUM_IPv4;
		}
		break;

	case RXD0_PID_TCP >> RXD0_PID_SHIFT:
		if ((rsr & RXD0_TCPF) == 0) {
			pflags |= GEM_RX_CKSUM_TCP;
		}
		break;

	case RXD0_PID_UDP >> RXD0_PID_SHIFT:
		if ((rsr & RXD0_UDPF) == 0) {
			pflags |= GEM_RX_CKSUM_UDP;
		}
		break;
	}
#endif
	return pflags | GEM_RX_DONE | len;
}

static void
gani_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct tx_desc	*tdp;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	ddi_put32(dp->desc_acc_handle,
		&tdp->txd0, (slot == TX_RING_SIZE - 1) ? TXD0_EOR : 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);
}

static void
gani_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct rx_desc	*rdp;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	ddi_put32(dp->desc_acc_handle,
		&rdp->rxd0, (slot == RX_RING_SIZE - 1) ? RXD0_EOR : 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);
}

/*
 * Device depend interrupt handler
 */
static u_int
gani_interrupt(struct gem_dev *dp)
{
	uint16_t	isr;
	u_int		restart_tx = 0;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	isr = INW(dp, ISR);

	DPRINTF(4, (CE_CONT,"!%s: time:%d gani_interrupt: isr:%b",
		dp->name, ddi_get_lbolt(), isr, INTR_BITS));

	if ((isr & lp->imr_hw) == 0) {
		/* not for us */
		return DDI_INTR_UNCLAIMED;
	}

	DPRINTF(2, (CE_CONT,"!%s: time:%d gani_interrupt: isr:%b imr:%b",
		dp->name, ddi_get_lbolt(), isr, INTR_BITS,
		INW(dp, IMR), INTR_BITS));

	/* disable interrupts */
	OUTW(dp, IMR, lp->imr_hw = 0);

	if (!dp->nic_active) {
		/* inhibit interrupt */
		lp->imr = 0;

		/* ack to all interrupts */
		OUTW(dp, ISR, 0xffff);
		FLSHW(dp, ISR);
		return DDI_INTR_CLAIMED;
	}

	/* clear interrupt sources explicitly */
	OUTW(dp, ISR, isr);

	if ((isr & INTR_TDU) != 0) {
		/*
		 * RTL8169 have loaded all valid tx descriptors.
		 * request.
		 */
		mutex_enter(&dp->xmitlock);
		lp->tx_active = FALSE;
		if (lp->tx_active_pended) {
			/* kick Tx engine again */
			lp->tx_active = TRUE;
			OUTB(dp, TPPoll, TPPoll_NPQ);
			lp->tx_active_pended = FALSE;
		}
		else {
#ifdef WA_TPQ_NO_WRAP
			if (SLOT(dp->tx_desc_tail, TX_RING_SIZE) == 0) {
				/*
				 * As no pended NPQ requests, release all of
				 * reserved descriptors.
				 */
				dp->tx_desc_rsvd = 0;

				/* make new tx requests into tx ring */
				isr |= INTR_TOK;
			}
#endif
			if (dp->tx_desc_intr - dp->tx_desc_head > 0) {
				/* need to enable TOK interrupt */
				lp->imr |= INTR_TOK;
			}
		}
		mutex_exit(&dp->xmitlock);
	}

#ifdef CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval != 0) {
			/* polling mode */
#if 1
			lp->imr_disabled = INTR_ROK | INTR_TDU | INTR_TOK;
#else
			/* performance improved, but high cpu usage */
			lp->imr_disabled = INTR_ROK | INTR_TOK;
#endif
			OUTL(dp, TimerInt,
				((lp->cfg2 & CFG2_PCICLKF) != 0)
					? dp->poll_interval * 66
					: dp->poll_interval * 33);

			if (lp->last_poll_interval == 0) {
				/*
				 * To schedule the next timer interrupt,
				 * we pretend as we were interrupted from
				 * polling timer
				 */
				isr |= INTR_TimeOut;
			}
		}
		else {
			/* normal mode */
			lp->imr_disabled = 0;
			OUTL(dp, TimerInt, 0);
		}

		lp->last_poll_interval = dp->poll_interval;
	}

	if ((isr & INTR_TimeOut) != 0) {
		/*
		 * Reset PCI clock counter to schedule the next polling
		 * timer interrupt.
		 */
		OUTL(dp, TCTR, 0);

		/* force to process ROK and TOK */
		isr |= INTR_ROK | INTR_TOK;
	}
#endif /* CONFIG_POLLING */

	if ((isr & INTR_PUN) != 0) {
		/*
		 * Link or PHY status has changed
		 */
		DPRINTF(4, (CE_CONT, "!%s: isr:%b", dp->name, isr, INTR_BITS));
	}

	if ((isr & (INTR_ROK | INTR_RER | INTR_FOVW | INTR_RDU)) != 0) {
		(void) gem_receive(dp);
		if ((isr & INTR_FOVW) != 0) {
			cmn_err(CE_WARN,
				"!%s: gani_interrupt: rx fifo overflow, isr %b",
				dp->name, isr, INTR_BITS);
			/*
			 * we should clear rx fifo
			 * and increase rxdma size;
			 */
			dp->rxmaxdma = min(dp->rxmaxdma + 256, 1024);
			lp->need_to_reset = TRUE;
			dp->stats.overflow++;
		}

		if ((isr & INTR_RDU) != 0 && dp->poll_interval != 0) {
			/*
			 * current polling interval is too long. recompute it.
			 */
			if (dp->poll_pkt_delay > 0) {
				dp->poll_pkt_delay--;
				DPRINTF(0, (CE_CONT,
					"!%s: pkt-delay decreased to %d",
					dp->name, dp->poll_pkt_delay));
			}
			dp->stats.norcvbuf++;
		}
	}

	if ((isr & (INTR_TER | INTR_TOK | INTR_TimeOut)) != 0) {
		/* Need to kick potentially blocked down stream */
		if (gem_tx_done(dp)) {
			restart_tx |= INTR_RESTART_TX;
		}

		if ((isr & INTR_TER) != 0) {
			cmn_err(CE_WARN, "!%s: Tx error, isr:%b",
				dp->name, isr, INTR_BITS);
		}
	}

	if ((isr & INTR_SERR) != 0) {
		cmn_err(
#ifdef NEVER /* DEBUG_LEVEL */
			CE_PANIC,
#else
			CE_WARN,
#endif
			"!%s: unexpected interrupt: isr:%b",
			dp->name, isr, INTR_BITS);
		lp->need_to_reset = TRUE;
	}

	/* barrier for interrupt state */
	FLSHW(dp, ISR);

	if (lp->need_to_reset) {
		mutex_enter(&dp->xmitlock);
		gani_stop_chip(dp);
		gem_restart_nic(dp, TRUE);
		mutex_exit(&dp->xmitlock);

		restart_tx = INTR_RESTART_TX;
		lp->need_to_reset = FALSE;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* enable interrupts again */
		OUTW(dp, IMR, lp->imr_hw = lp->imr & ~lp->imr_disabled);
	}

	return DDI_INTR_CLAIMED | restart_tx;
}

/* 
 * MII Interfaces
 */
#define	MII_DELAY(dp, r)	{INL(dp, (r)); INL(dp, (r))}

static void
gani_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
gani_mii_read(struct gem_dev *dp, uint_t index)
{
	int		i;
	uint32_t	ret;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	if (dp->nic_active &&
	   (ddi_get_lbolt() - lp->last_stats_time) >= ONESEC) {
		gani_get_stats(dp);
	}

	OUTL(dp, PHYAR, index << PHYAR_REGADDR_SHIFT);
	drv_usecwait(20);

	for (i = 0; ((ret = INL(dp, PHYAR)) & PHYAR_FLAG) == 0; i++) {
		if (i > 1000) {
			cmn_err(CE_WARN, "%s: %s: timeout",
				dp->name, __func__);
			return 0xffff;
		}
		drv_usecwait(20);
	}
	return ret & PHYAR_DATA;
}

static void
gani_mii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int		i;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	OUTL(dp, PHYAR,
		PHYAR_FLAG | (index << PHYAR_REGADDR_SHIFT) | val);
	drv_usecwait(20);

	for (i = 0; (INL(dp, PHYAR) & PHYAR_FLAG) != 0; i++) {
		if (i > 1000) {
			cmn_err(CE_WARN, "%s: %s: timeout",
				dp->name, __func__);
			break;
		}
		drv_usecwait(20);
	}
}
#undef	MII_DELAY

static int
gani_mii_patch(struct gem_dev *dp)
{
	struct gani_dev		*lp = (struct gani_dev *)dp->private;

	if (lp->chip->type == MAC_VER_F /* M4 */) {
		gani_mii_write(dp, 0x1F, 0x0001);
		gani_mii_write(dp, 0x09, 0x273a);
		gani_mii_write(dp, 0x0e, 0x7bfb);
		gani_mii_write(dp, 0x1b, 0x841e);

		gani_mii_write(dp, 0x1F, 0x0002);
		gani_mii_write(dp, 0x01, 0x90D0);
		gani_mii_write(dp, 0x1F, 0x0000);
	}

	if (lp->chip->type == MAC_VER_D /* M2 */) {
		OUTB(dp, 0x82, 0x01);
		gani_mii_write(dp, 0xb, 0x0000);
	}
}

static int
gani_mii_init(struct gem_dev *dp)
{
	int			ret;
	struct gani_dev		*lp = (struct gani_dev *)dp->private;

	ret = gem_mii_init_default(dp);

	gani_mii_patch(dp);

	if (lp->chip->type >= MAC_VER_D/*M2*/ &&
	    dp->mii_phy_id >= 0 && (dp->mii_phy_id & 0xf) <= 1) {
		/* fix linkdown timeout action */
		dp->gc.gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
	}

	return ret;
}

/* ======================================================== */
/*
 * OS depend (device driver kernel interface) routine
 */
/* ======================================================== */
#ifdef CONFIG_EEPROM_IO
#define	gani_eeprom_delay(dp)	{INB(dp, CR9346); INB(dp, CR9346);}

static uint16_t
gani_read_eeprom(struct gem_dev *dp, int addr)
{
	int		i;
	int		addr_bits;
	uint_t		cmd;
	uint8_t		chip_select;
	uint8_t		di;
	uint16_t	ret;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	addr_bits = ((INL(dp, RCR) & RCR_9356SEL) != 0) ? 8 : 6;

	DPRINTF(2, (CE_CONT, "!%s: gani_read_eeprom: called: addr_bits:%d",
			dp->name, addr_bits));

	/* make command bits */
	cmd = (6 << addr_bits) | addr;

	/* enable eeprom interface register */
	chip_select = CR9346_EEM_PROGRAM;
	OUTB(dp, CR9346, chip_select);

	chip_select |= CR9346_EECS;
	OUTB(dp, CR9346, chip_select);
	gani_eeprom_delay(dp);

	/* output eeprom command */
	for (i = 4 + addr_bits; i >= 0; i--) {
		di = ((cmd >> i) << CR9346_EEDI_SHIFT) & CR9346_EEDI;
		OUTB(dp, CR9346, chip_select | di);
		gani_eeprom_delay(dp);

		OUTB(dp, CR9346, chip_select | di | CR9346_EESK);
		gani_eeprom_delay(dp);
	}

	/* release clock but keep chip_select asserted */
	OUTB(dp, CR9346, chip_select);
	gani_eeprom_delay(dp);

	/* get returned value */
	ret = 0;
	for (i = 16; i > 0; i--) {
		/* get 1 bit */
		OUTB(dp, CR9346, chip_select | CR9346_EESK);
		gani_eeprom_delay(dp);

		ret = (ret << 1)
		    | ((INB(dp, CR9346) >> CR9346_EEDO_SHIFT) & 1);

		OUTB(dp, CR9346, chip_select);
		gani_eeprom_delay(dp);
	}

	/* Terminate the EEPROM access. */
	OUTB(dp, CR9346, CR9346_EEM_PROGRAM);
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	gani_eeprom_delay(dp);

	DPRINTF(2, (CE_CONT, "!gani_read_eeprom: returned 0x%x", ret));

	return ret;
}
static void
gani_eeprom_dump(struct gem_dev *dp, int size)
{
	int		i;

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < size; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x", 
			i, gani_read_eeprom(dp, i),
			gani_read_eeprom(dp, i + 1),
			gani_read_eeprom(dp, i + 2),
			gani_read_eeprom(dp, i + 3));
	}
}
#endif

static int
gani_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	val;
	uint8_t		*m;
	struct gani_dev	*lp = (struct gani_dev *)dp->private;

	/*
	 * reload default mac address from EEPROM
	 */
	m = &dp->dev_addr.ether_addr_octet[0];
#ifdef CONFIG_EEPROM_IO
#  if DEBUG_LEVEL > 2
	gani_eeprom_dump(dp, 0x40);
#  endif
	for (i = 0; i < ETHERADDRL/2; i++) {
		val = gani_read_eeprom(dp, i + 7);
		m[i*2  ] = (uint8_t) val;
		m[i*2+1] = (uint8_t)(val >> 8);
	}
#else
	OUTB(dp, CR9346, CR9346_EEM_AUTOLD);

	/*
	 * XXX - the old datasheet says it takes 2mS typically, but
	 * it still caused bus timeouts for Netgear cardbus products
	 * even if we waited for 10mS. Now we wait for 100mS.
	 */
	drv_usecwait(1000000);

	for (i = 0; i < ETHERADDRL; i++) {
		m[i] = INB(dp, IDR + i);
	}
#endif /* CONFIG_EEPROM_IO */
	/* initialize soft copy of mac registers */
	bcopy(m, lp->mac, ETHERADDRL);

	/* no need to scan phy */
	dp->mii_phy_addr = -1;

	/* mtu: max packet size is 8kbyte - 1 */
	dp->mtu = min(dp->mtu,
		8192 - 1 - sizeof(struct ether_header)
			- ETHERFCSL - 4/*VTAG_SIZE*/);

	/* increase tx dma burst size for performance */
	dp->txmaxdma = max(4*64, dp->txmaxdma);		/* default is 64 */

	/* rx priority should be higher than tx */
	dp->rxmaxdma = dp->txmaxdma * 2;

	/* store & foward for tx */
	dp->txthr = sizeof(struct ether_header) + dp->mtu + ETHERFCSL + 4;

	/* enable early dma for rx */
	dp->rxthr = (3 * dp->rxmaxdma) / 4;
#ifdef notdef
	/* undocumented patch */
	OUTB(dp, 0x82, 0x01);
#endif
	/* rtl8169 and rtl8110 don't support 1000Tx half duplex mode */
	dp->anadv_1000hdx = FALSE;

	/* fix rx buffer length, it must have additional 4 byte */
	dp->rx_buf_len = dp->mtu + sizeof(struct ether_header) + ETHERFCSL + 4;

	if (dp->mii_fixedmode) {
		cmn_err(CE_WARN,
			"!%s: rtl8169 does not support fixed mode. "
			"limited auto negotiation capability is used "
			"instead of specified fixed mode.",
			dp->name);
		dp->anadv_1000fdx = FALSE;
		dp->anadv_1000hdx = FALSE;
		dp->anadv_100t4   = FALSE;
		dp->anadv_100fdx  = FALSE;
		dp->anadv_100hdx  = FALSE;
		dp->anadv_10fdx   = FALSE;
		dp->anadv_10hdx   = FALSE;

		switch(dp->speed) {
		case GEM_SPD_1000:
			if (!dp->full_duplex) {
				cmn_err(CE_WARN,
	"!%s: 1G half duplex isn't supported. using full duplex instead of.",
				dp->name);
			}
			dp->anadv_100fdx = TRUE;
			dp->full_duplex  = TRUE;
			break;

		case GEM_SPD_100:
			dp->anadv_100fdx = dp->full_duplex;
			dp->anadv_100hdx =!dp->full_duplex;
			break;

		case GEM_SPD_10:
			dp->anadv_10fdx = dp->full_duplex;
			dp->anadv_10hdx =!dp->full_duplex;
			break;
		}

		dp->mii_fixedmode = FALSE;
	}

#ifdef CONFIG_CKSUM_OFFLOAD
	dp->misc_flag |= (GEM_CKSUM_IPv4 | GEM_CKSUM_TCP | GEM_CKSUM_UDP);
#endif
#ifdef CONFIG_VLAN
	dp->misc_flag |= GEM_VLAN_HARD;
#endif

	gani_rx_copy_thresh = 
	gani_tx_copy_thresh = dp->mtu + sizeof(struct ether_header) + 4;

	return 0;	/* currently return code is not used. */
}

static int
ganiattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_acc_handle_t	conf_handle;
	int			vid;
	int			did;
	int			revid;
	int			unit;
	struct chip_info	*p;
	uint32_t		tcr;
	const char		*drv_name;
	struct gem_dev		*dp;
	void			*base;
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	struct gani_dev		*lp;
	uint32_t		ilr;

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);
	DPRINTF(3, (CE_CONT, "!%s%d: ganiattach: called", drv_name, unit));

	/*
	 * Common routine after power-on
	 */

	/* fix config registers */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
			drv_name, unit);
		goto err;
	}

	pci_config_put16(conf_handle, PCI_CONF_COMM,
		pci_config_get16(conf_handle, PCI_CONF_COMM)
			| PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	/* set pci latency timer to 0x40 */
	pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 0x40);

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME: {
		struct gem_dev  *dp;
		gld_mac_info_t  *macinfo;

		macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
		dp = (struct gem_dev *)macinfo->gldm_private;

		gani_mii_patch(dp);
		return gem_resume(dip);
	}
	case DDI_ATTACH:
		/*
		 * Map in the device registers.
		 */
		if (gem_pci_regs_map_setup(dip, PCI_ADDR_MEM32, &gani_dev_attr,
			(caddr_t *)&base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * Check hardware revision
		 */
		tcr = ddi_get32(regs_handle,
			(uint32_t *)((caddr_t)base + TCR)) & TCR_MACVER;

		for (p = chiptbl_8169, i = 0; i < CHIPTABLESIZE; p++, i++) {
			if (p->tcr_val == tcr) {
				/* found */
				goto chip_found;
			}
		}

		cmn_err(CE_WARN,
			"!%s%d: attach: unknown mac version: tcr:0x%08x",
			drv_name, unit, tcr);

		p = &chiptbl_8169[CHIPTABLESIZE-1];	/* MAC ver F */

chip_found:
		cmn_err(CE_CONT, "!%s%d: chip is %s rev:0x%02x tcr:0x%08x",
			drv_name, unit, p->name, revid, tcr);

		if ((ddi_get8(regs_handle,
			(uint8_t *)((caddr_t)base + PHYS)) & PHYS_EnTBI) != 0) {
			cmn_err(CE_CONT,
				"!%s%d: tbi interfaces isn't supported",
				drv_name, unit);
			goto err;	
		}

		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_tx_max_frags = TX_MAX_FRAGS;
		gcp->gc_tx_desc_size =
			ROUNDUP2(sizeof(struct tx_desc)*TX_RING_SIZE,
				DESC_BASE_ALIGN) + TC_SIZE,
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_auto_pad  = TRUE;
		gcp->gc_tx_copy_thresh = gani_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_size =
			ROUNDUP2(sizeof(struct rx_desc) * RX_RING_SIZE,
				DESC_BASE_ALIGN);
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;
		gcp->gc_rx_max_descs_per_pkt = gcp->gc_rx_max_frags;
		gcp->gc_rx_copy_thresh = gani_rx_copy_thresh;
		gcp->gc_rx_buf_max  = gcp->gc_rx_buf_size + 1;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, gani_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, gani_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, gani_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, gani_dma_attr_desc);
		STRUCT_COPY(gcp->gc_dma_attr_txbuf, gani_dma_attr_buf32);
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, gani_dma_attr_buf32);
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* mii mode */
		gcp->gc_mii_mode = GEM_MODE_1000BASET;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/10;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout  = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait	= 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* rtl8169 seems to delay to recognize PHY status */
		gcp->gc_mii_an_delay	    = ONESEC/10;	/* 100mS */
		gcp->gc_mii_linkdown_action = MII_ACTION_NONE;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_NONE;
		gcp->gc_mii_dont_reset      = FALSE;
		gcp->gc_mii_an_oneshot      = FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &gani_attach_chip;
		gcp->gc_reset_chip  = &gani_reset_chip;
		gcp->gc_init_chip   = &gani_init_chip;
		gcp->gc_start_chip  = &gani_start_chip;
		gcp->gc_stop_chip   = &gani_stop_chip;
		gcp->gc_multicast_hash = &gani_mcast_hash;
		gcp->gc_set_rx_filter = &gani_set_rx_filter;
		gcp->gc_set_media   = &gani_set_media;
		gcp->gc_get_stats   = &gani_get_stats;
		gcp->gc_interrupt   = &gani_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &gani_tx_desc_write;
		gcp->gc_rx_desc_write = &gani_rx_desc_write;
		gcp->gc_tx_desc_stat = &gani_tx_desc_stat;
		gcp->gc_rx_desc_stat = &gani_rx_desc_stat;
		gcp->gc_tx_desc_init = &gani_tx_desc_init;
		gcp->gc_rx_desc_init = &gani_rx_desc_init;
		gcp->gc_tx_desc_clean = &gani_tx_desc_init;
		gcp->gc_rx_desc_clean = &gani_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_init   = &gani_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync  = &gani_mii_sync;
		gcp->gc_mii_read  = &gani_mii_read;
		gcp->gc_mii_write = &gani_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = (struct gani_dev *)
				kmem_zalloc(sizeof(struct gani_dev), KM_SLEEP);
		lp->chip = p;

		/* read config2 register */
		lp->cfg2 = ddi_get8(regs_handle,
				(uint8_t *)((caddr_t)base + CFG2));
		DPRINTF(2, (CE_CONT, "!%s%d: cfg2:0x%02x",
			drv_name, unit, lp->cfg2));
		dp = gem_do_attach(dip, gcp, base, &regs_handle,
					lp, sizeof(struct gani_dev));
		kmem_free(gcp, sizeof(*gcp));

		if (dp == NULL) {
			goto err_free_mem;
		}

		return DDI_SUCCESS;

err_free_mem:
		kmem_free(lp, sizeof(struct gani_dev));
err:
		return DDI_FAILURE;
	}

	return DDI_FAILURE;
}

static int
ganidetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct gem_dev  *dp;
	gld_mac_info_t  *macinfo;

	macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
	dp = (struct gem_dev *)macinfo->gldm_private;

	switch (cmd) {
	case DDI_DETACH:
		return  gem_do_detach(dip);

	case DDI_SUSPEND:
		return gem_suspend(dip);
	}

	return DDI_FAILURE;
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info ganiminfo = {
	0,			/* mi_idnum */
	"gani",			/* mi_idname */
	0,			/* mi_minpsz */
	8192,			/* mi_maxpsz */
	TX_BUF_SIZE*ETHERMAX,	/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit ganirinit = {
	(int (*)()) NULL,	/* qi_putp */
	gld_rsrv,		/* qi_srvp */
	gld_open,		/* qi_qopen */
	gld_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ganiminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit ganiwinit = {
	gld_wput,		/* qi_putp */
	gld_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ganiminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	gani_info = {
	&ganirinit,	/* st_rdinit */
	&ganiwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_gani_ops = {
	nulldev,	/* cb_open */
	nulldev,	/* cb_close */
	nodev,		/* cb_strategy */
	nodev,		/* cb_print */
	nodev,		/* cb_dump */
	nodev,		/* cb_read */
	nodev,		/* cb_write */
	nodev,		/* cb_ioctl */
	nodev,		/* cb_devmap */
	nodev,		/* cb_mmap */
	nodev,		/* cb_segmap */
	nochpoll,	/* cb_chpoll */
	ddi_prop_op,	/* cb_prop_op */
	&gani_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops gani_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gld_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	ganiattach,	/* devo_attach */
	ganidetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_gani_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&gani_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/* ======================================================== */
/*
 * _init : done
 */
/* ======================================================== */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, "!gani: _init: called"));
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

	DPRINTF(2, (CE_CONT, "!gani: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
