/*
 * jmge: Jmicrom JMC250/260 PCI-E ethernet controller driver
 *
 * Copyright (c) 2008-2012 Masayuki Murayama. All rights reserved.
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

/*
 * Acknowledgement:
 * I thank to Mr.Yamamoto and Mr.Haraguchi of Tokyo OpenSolaris Study Group
 * for testing with Shuttle XS series and low cost 1G swiching hubs.
 */

#pragma	ident	"@(#)jmge_gem.c 1.6     12/06/23"

/*
 * System Header files.
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/byteorder.h>
#include <sys/ethernet.h>
#include <sys/pci.h>

#include "gem_mii.h"
#include "gem.h"
#include "jmc250reg.h"

char	ident[] = "jmc250/260 driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int jmge_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (jmge_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)

#define	FM_ECO(x, y)		((y) << 4 | (x))
#define	REV_JMC250A2		FM_ECO(1, 1)
#define	REV_JMC260		FM_ECO(2, 0)
#define	FULL_MASK_REV(x)	((x) & 0xf)
#define	ECO_REV(x)		(((x) >> 4) & 0xf)

#define	PCI_DEVID_JMC250	0x250
#define	PCI_DEVID_JMC260	0x260
#define	PCI_VENID_JMC		0x197b

#ifdef GEM3
#define	mac_active	mac_state
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif

/*
 * Our configuration
 */
#ifdef CONFIG_POLLING
#define	OUR_INTR_BITS \
	(IEVE_SW | IEVE_TM | IEVE_LINK \
	| IEVE_PCCRX0TO | IEVE_PCCRX0 | IEVE_RX0EMP \
	| IEVE_PCCTXTO | IEVE_PCCTX)
#else
#define	OUR_INTR_BITS	\
	(IEVE_SW | IEVE_TM | IEVE_LINK | IEVE_RX0EMP | IEVE_RX0 | IEVE_TX0)
#endif

#ifdef TEST_TXDESC_FULL
#undef	TX_BUF_SIZE
#define	TX_BUF_SIZE	16
#endif
#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	(TX_BUF_SIZE * 8)
#endif


#ifdef TEST_RX_EMPTY
#undef	RX_BUF_SIZE
#define	RX_BUF_SIZE	32
#endif
#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#define	CONFIG_TX_SINGLE_QUEUE

#ifdef CONFIG_TX_COPY
static int	jmge_tx_copy_thresh = INT32_MAX;
#else
static int	jmge_tx_copy_thresh = 256;
#endif
#ifdef CONFIG_RX_COPY
static int	jmge_rx_copy_thresh = INT32_MAX;
#else
static int	jmge_rx_copy_thresh = 256;
#endif

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info jmge_chiptbl[] = {
	{PCI_VENID_JMC, PCI_DEVID_JMC250, "JMC250"},
	{PCI_VENID_JMC, PCI_DEVID_JMC260, "JMC260"},
};
#define	CHIPTABLESIZE   (sizeof (jmge_chiptbl) / sizeof (struct chip_info))

/*
 * jmgeine chip state
 */
struct jmge_dev {
	uint8_t			mac_addr[ETHERADDRL];
	uint32_t		rxmcs;
	uint32_t		txmcs;
	uint32_t		rxcs;
	uint32_t		txcs;

	uint8_t			revid;	/* chip revision id */
	uint_t			maxrdreq;
	uint_t			maxpayload;
	uint32_t		ien;
#ifdef RESET_TEST
	int			reset_test;
#endif

	/* device type */
	uint_t			fpga;
	uint_t			chiprev;

	boolean_t		need_to_reset;

	struct chip_info	*chip;

	/* tunable phy parameter */
	int	phy_cable_length;
	int	phy_flp_gap;
};

/*
 * Macros to identify chip generation.
 */

/* ======================================================== */

/* mii operations */
static void  jmge_mii_sync(struct gem_dev *);
static uint16_t  jmge_mii_read(struct gem_dev *, uint_t);
static void jmge_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int jmge_attach_chip(struct gem_dev *);
static int jmge_reset_chip(struct gem_dev *);
static int jmge_init_chip(struct gem_dev *);
static int jmge_start_chip(struct gem_dev *);
static int jmge_stop_chip(struct gem_dev *);
static int jmge_set_media(struct gem_dev *);
static int jmge_set_rx_filter(struct gem_dev *);
static int jmge_get_stats(struct gem_dev *);

/* descriptor operations */
static int jmge_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void jmge_tx_start(struct gem_dev *dp, int slot, int frags);
static void jmge_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t jmge_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static void jmge_rx_start(struct gem_dev *dp, int slot, int frags);
static uint64_t jmge_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void jmge_tx_desc_init(struct gem_dev *dp, int slot);
static void jmge_rx_desc_init(struct gem_dev *dp, int slot);
static void jmge_tx_desc_clean(struct gem_dev *dp, int slot);
static void jmge_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t jmge_interrupt(struct gem_dev *dp);

static void jmge_tx_desc_dump(struct gem_dev *dp, int slot);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr jmge_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr jmge_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t jmge_dma_attr_buf_250 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x0000ffffffffffffull,	/* dma_attr_addr_hi */
	0x0000ffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x0000ffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000000000000ffffull,	/* dma_attr_maxxfer */
	0x0000ffffffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t jmge_dma_attr_buf_260 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x00000000ffffffffull,	/* dma_attr_addr_hi */
	0x0000ffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x0000ffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000000000000ffffull,	/* dma_attr_maxxfer */
	0x00000000ffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t jmge_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xf,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */
__INLINE__ static void
jmge_ioarea_dma_sync(struct gem_dev *dp, off_t start, size_t len, int how)
{
	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(dp->io_area_dma - dp->rx_ring_dma + start),
	    len, how);
}

static void
jmge_turn_off_tx_clock(struct gem_dev *dp)
{
	int	i;
	struct jmge_dev	*lp = dp->private;

	OUTL(dp, GHC, INL(dp, GHC) & ~(GHC_TXCKSRC | GHC_TCPCKSRC));
	FLSHL(dp, GHC);
}

static void
jmge_turn_on_tx_clock(struct gem_dev *dp)
{
	uint32_t	val;

	val = INL(dp, GHC) & ~(GHC_TXCKSRC | GHC_TCPCKSRC);
	if ((val & GHC_SPD) == GHC_SPD_1000M) {
		val |= GHC_TXCKSRC_GPHY | GHC_TCPCKSRC_GPHY;
	} else {
		val |= GHC_TXCKSRC_PCIE | GHC_TCPCKSRC_PCIE;
	}
	OUTL(dp, GHC, val);
	FLSHL(dp, GHC);
}

static void
jmge_turn_off_rx_clock(struct gem_dev *dp)
{
	int	i;
	struct jmge_dev	*lp = dp->private;

	OUTL(dp, GPREG1, INL(dp, GPREG1) | GPREG1_dis_clk_rx);
	FLSHL(dp, GPREG1);
}

static void
jmge_turn_on_rx_clock(struct gem_dev *dp)
{
	OUTL(dp, GPREG1, INL(dp, GPREG1) & ~GPREG1_dis_clk_rx);
	FLSHL(dp, GPREG1);
}

static int
jmge_reset_chip(struct gem_dev *dp)
{
	uint32_t	uval;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* clear cached mac address */
	bzero(lp->mac_addr, ETHERADDRL);

	/* reset ghc speed and duplex */
	OUTL(dp, GHC, INL(dp, GHC) & ~(GHC_SPD | GHC_DPX));

	/* reset 250A2 workaround */
	OUTL(dp, GHC, INL(dp, GPREG1) &
	    ~(GPREG1_HALFMODEPATCH | GPREG1_RSSPATCH));

	/* clear interrutp mask */
	OUTL(dp, IENC, 0xffffffffU);
	FLSHL(dp, IENC);

	/* ensure rx and tx are disabled */
	uval = INL(dp, RXCS);
	if (uval & RXCS_EN) {
		OUTL(dp, RXCS, uval & ~RXCS_EN);
		FLSHL(dp, RXCS);
	}

	uval = INL(dp, TXCS);
	if (uval & TXCS_EN) {
		OUTL(dp, TXCS, uval & ~TXCS_EN);
		FLSHL(dp, TXCS);
	}

#ifdef WA_RXFIFO
	/* turn on rx clock */
	jmge_turn_on_rx_clock(dp);

	/* turn on tx clock */
	jmge_turn_on_tx_clock(dp);
	drv_usecwait(1);
#endif
	/* force to reset the hardware */
	OUTL(dp, GHC, INL(dp, GHC) | GHC_SWRST);
	FLSHL(dp, GHC);
	drv_usecwait(2);

#ifdef WA_RXFIFO
	/* turn off rx clock */
	jmge_turn_off_rx_clock(dp);

	/* turn off tx clock */
	jmge_turn_off_tx_clock(dp);
	drv_usecwait(1);
#endif
	/* clear software reset bit */
	OUTL(dp, GHC, INL(dp, GHC) & ~GHC_SWRST);
	FLSHL(dp, GHC);
	drv_usecwait(1);

	jmge_turn_on_rx_clock(dp);
#ifdef WA_RXFIFO
	/* turn on tx clock */
	jmge_turn_on_tx_clock(dp);
	drv_usecwait(1);

	/*
	 * XXX - don't turn off tx/rx clocks.
	 */
#ifdef NEVER	/* NG */
	/* turn off rx clock */
	jmge_turn_off_rx_clock(dp);

	/* turn off tx clock */
	jmge_turn_off_tx_clock(dp);
	drv_usecwait(1);
#endif
	OUTL(dp, RXDBAL, 0x00000000);
	OUTL(dp, RXDBAH, 0x00000000);
	OUTL(dp, RXQDC, 0x00000000);
	OUTL(dp, RXNDA, 0x00000000);
	OUTL(dp, TXDBAL, 0x00000000);
	OUTL(dp, TXDBAH, 0x00000000);
	OUTL(dp, TXQDC, 0x00000000);
	OUTL(dp, TXNDA, 0x00000000);

	OUTL(dp, RXMCHT + 0, 0x00000000);
	OUTL(dp, RXMCHT + 4, 0x00000000);
#endif

	/* setup GPREG0 */
	uval = GPREG0_PCIRLMT_4 | GPREG0_PCCTU_1uS | 1;
	if (lp->fpga) {
		uval |= GPREG0_LINTS;	/* select polling mode */
	}
	OUTL(dp, GPREG0, uval);

	return (GEM_SUCCESS);
}

static int
jmge_init_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	uval;
	uint_t		packet_time;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/*
	 * GIGA mac operation registers
	 */
	/* TXCS: shoule be setup before tx ring base address */
	lp->txcs = 0 << TXCS_QW_SHIFT | 0 << TXCS_QSEL_SHIFT
	    | TXCS_FT_4QW | TXCS_BR;
	switch (lp->maxrdreq) {
	case 0:
		lp->txcs |= TXCS_DS_128B;
		break;

	case 1:
		lp->txcs |= TXCS_DS_256B;
		break;

	default:
		lp->txcs |= TXCS_DS_512B;
		break;
	}
	OUTL(dp, TXCS, lp->txcs);

	/* tx desc base addr */
	OUTL(dp, TXDBAL, dp->tx_ring_dma);
	OUTL(dp, TXDBAH, dp->tx_ring_dma >> 32);
	OUTL(dp, TXQDC, dp->gc.gc_tx_ring_size);

	/* TXMCS: set later */	/* ok L */
	lp->txmcs = TXMCS_IFG2_8_5 | TXMCS_IFG1_16_8
	    | TXMCS_THOLD_SF | TXMCS_DEFEN | TXMCS_CRCEN | TXMCS_PADEN;

	/* TXPCS: set later */
	/* RXCS: should be setup before setup rx ring base address */
	/* new recommended configuration from JMicron */
	lp->rxcs = RXCS_FF_128T
	    | RXCS_FT_16QW
	    | RXCS_DRS_128B
	    | (0 << RXCS_QSEL_SHIFT) | RXCS_RG_256nS |  RXCS_RCn(32);
#ifdef WA_SHORT_PACKETS
	/* not required ? */
	lp->rxcs |= RXCS_SPREN;
#endif
	OUTL(dp, RXCS, lp->rxcs);

	/* rx desc base addr */
	OUTL(dp, RXDBAL, dp->rx_ring_dma);
	OUTL(dp, RXDBAH, dp->rx_ring_dma >> 32);
	/*
	 * we use two descriptors for a receive buffer to support
	 * 64bit physical memory space.
	 */
	OUTL(dp, RXQDC, dp->gc.gc_rx_ring_size * 2);

	/* RXMCS */
#ifdef CHIP_BUG_RX_CKSUM /* CONFIG_CKSUM_OFFLOAD */
	/*
	 * XXX - Don't enable RXMCS_CSEN. When RXMCS_CSEN is set, the chip
	 * won't receive the last fragment in fragmented IP packets, which
	 * ethernet frame length is shorter than 66 byte.
	 * For example, "ping -s hostname 1503" will be failed.
	 */
	lp->rxmcs = RXMCS_CSEN;
#else
	lp->rxmcs = 0;
#endif

	/* RXUMA: set later in set_rx_filter */
	/* RXMCHT: set later in set_rx_filter */
	/* WFODP: */
	/* WFOFI: */
	/* SMI: no need to be initialized */
	/* GHC: has been initialized */
	/* PMCS: disable power managements */
	OUTL(dp, PMCS, 0xffff0000U);

	/*
	 * MISC operation registers
	 */
	/* TMCSR: */
	/* GPREG0: has been initialized */
	/* GPREG1: has been initialized */

	/* MSINUM */
	/* IEVE: clear */
	OUTL(dp, IEVE, 0xffffffffU);

	/* IREQ: no need to initialize */
	/* IENS/IENC: no need to initialize */
	/* PCCRXn: set later in jmge_set_media() */
	/* PCCTX: set later in jmge_set_media() */
	/* shadow: enable */
	OUTL(dp, SHBALO, (dp->io_area_dma & SHBALO_LO) | SHBALO_POSTEN);

	lp->need_to_reset = B_FALSE;

	return (GEM_SUCCESS);
}

static uint_t
jmge_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is the lowest 6 bits of big endian crc */
	return (gem_ether_crc_be(addr, ETHERADDRL) % 64);
}

static int
jmge_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	int		i;
	uint64_t	mhash;
	uint8_t		*mac;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d, rxmode:%b mc_count:%d",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, dp->rxmode, RXMODE_BITS,
	    dp->mc_count));

	mode = RXMCS_BF | RXMCS_UF | RXMCS_MF | RXMCS_FFHTEN;
	mhash = 0ULL;

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* accept all frames */
		mode |= RXMCS_AF;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* accept all multicast frames */
		mhash = ~0ULL;
	} else if (dp->mc_count > 0) {
		for (i = 0; i < dp->mc_count; i++) {
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	/*
	 * make rx filter mode promiscious not to lose any
	 * packets while we are changing the filter mode.
	 */
	OUTL(dp, RXMCS, lp->rxmcs | RXMCS_AF);
	FLSHL(dp, RXMCS);

	/* set my mac address */
	mac = &dp->cur_addr.ether_addr_octet[0];

	if (bcmp(lp->mac_addr, mac, ETHERADDRL) != 0) {
		ddi_acc_handle_t	hdl;
#ifdef DEBUG_LEVEL
		uint32_t	uval0;
		uint32_t	uval1;
#endif
		if (pci_config_setup(dp->dip, &hdl) == DDI_SUCCESS) {
			for (i = 0; i < ETHERADDRL; i++) {
				pci_config_put32(hdl, 0xe8,
				    0x81000000 |
				    (0x38 + i) << 16 | mac[i] << 8);
			}
			pci_config_teardown(&hdl);
		}
#if DEBUG_LEVEL > 2
		uval0 = INL(dp, RXUMA);
		uval1 = INL(dp, RXUMA + 4);
		DPRINTF(0, (CE_CONT, "!%s: %s: %02x:%02x:%02x:%02x:%02x:%02x",
		    dp->name, __func__,
		    uval0 & 0xff, (uval0 >> 8) & 0xff,
		    (uval0 >> 16) & 0xff, (uval0 >> 24) & 0xff,
		    uval1 & 0xff, (uval1 >> 8) & 0xff));
#endif
		bcopy(mac, lp->mac_addr, ETHERADDRL);
	}

	/* setup multicast hash table */
	OUTL(dp, RXMCHT, mhash);
	OUTL(dp, RXMCHT + 4, mhash >> 32);

	/* install new rx filter mode */
	lp->rxmcs &=
	    ~(RXMCS_AF | RXMCS_BF | RXMCS_UF | RXMCS_MF | RXMCS_FFHTEN);
	lp->rxmcs |= mode;
	OUTL(dp, RXMCS, lp->rxmcs);

	return (GEM_SUCCESS);
}

static int
jmge_set_media(struct gem_dev *dp)
{
	uint32_t	old_ghc;
	uint32_t	ghc;
	uint32_t	txpfc;
	uint_t		pkttime;
	int		i;
	uint32_t	uval;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, active:%d",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active));

	/*
	 * select speed and duplex mode
	 */
	old_ghc = INL(dp, GHC);
	ghc = old_ghc &
	    ~(GHC_DPX | GHC_SPD | GHC_LSPEN | GHC_TCPCKSRC | GHC_TXCKSRC);

	switch (dp->speed) {
	case GEM_SPD_10:
#ifdef NEVER
		pkttime = 1200;
#else
		/* workaround for JMC261 */
		pkttime = 120;
#endif
		ghc |= GHC_SPD_10M | GHC_TCPCKSRC_PCIE | GHC_TXCKSRC_PCIE;
		break;

	case GEM_SPD_100:
		pkttime = 120;
		ghc |= GHC_SPD_100M | GHC_TCPCKSRC_PCIE | GHC_TXCKSRC_PCIE;
		break;

	case GEM_SPD_1000:
		pkttime = 12;
		ghc |= GHC_SPD_1000M | GHC_TCPCKSRC_GPHY | GHC_TXCKSRC_GPHY;
		break;
	}

	if (lp->fpga && !(dp->speed == GEM_SPD_100 && dp->full_duplex)) {
		ghc |= GHC_LSPEN;
	}

	if (dp->full_duplex) {
		ghc |= GHC_DPX;
		OUTL(dp, TXMCS, lp->txmcs);
		OUTL(dp, TXTRHD, 0);
	} else {
		OUTL(dp, TXMCS,
		    lp->txmcs | TXMCS_BKFEN | TXMCS_CRSEN | TXMCS_COLEN);
		OUTL(dp, TXTRHD,
		    TXTRHD_TXPEN | (0x2000 << TXTRHD_TXP_SHIFT)
		    | TXTRHD_TXREN | 15);
	}

	if (old_ghc != ghc) {
		OUTL(dp, GHC, ghc);
	}

	/* flow control */
	switch (dp->flow_control) {
	case FLOW_CONTROL_SYMMETRIC:
		txpfc = TXPFC_PFEN;
		lp->rxmcs |= RXMCS_FCEN;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		txpfc = TXPFC_PFEN;
		lp->rxmcs &= ~RXMCS_FCEN;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		txpfc = 0;
		lp->rxmcs |= RXMCS_FCEN;
		break;

	case FLOW_CONTROL_NONE:
	default:
		txpfc = 0;
		lp->rxmcs &= ~RXMCS_FCEN;
		break;
	}

	OUTL(dp, TXPFC, txpfc);
	OUTL(dp, RXMCS, lp->rxmcs);

	/* workaround for crc errors at 100mbps */
	if (lp->chiprev == REV_JMC250A2) {
		if (dp->speed == GEM_SPD_100) {
			/* fifo 5 level depth */
			jmge_mii_write(dp, 27, 0x0004);
		} else {
			/* fifo 8 level depth */
			jmge_mii_write(dp, 27, 0x0000);
		}
	}

	/* setup interrupt coalescing parameters */
	/* PCCRX */
#define	N_RXQUEUE	4
	for (i = 0; i < N_RXQUEUE; i++) {
		uval = dp->poll_pkt_delay;
#ifdef NEVER
		/* XXX - doesn't work for JMC261 */
		if (dp->speed == GEM_SPD_10) {
			uval = 1;
		}
#endif
		OUTL(dp, PCCRX(i),
		    ((uval * pkttime) << PCCRX_TO_SHIFT)
		    | (uval << PCCRX_CNT_SHIFT));
	}

	/* PCCTX */
	if (dp->speed == GEM_SPD_10) {
#ifdef NEVER
		/* XXX - 0 doesn't work for JMC261 */
		uval = 0;
#else
		uval = TX_BUF_SIZE / 8;
#endif
	} else {
		uval = TX_BUF_SIZE / 8;
	}
	OUTL(dp, PCCTX,
	    ((uval * pkttime) << PCCTX_TO_SHIFT)
	    | (uval << PCCTX_CNT_SHIFT)
	    | PCCTX_EN(0));	/* only for tx0 */

	return (GEM_SUCCESS);
}

static int
jmge_start_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	ghc;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

#ifdef WA_RXFIFO
	/*
	 * Before enabling tx and rx, disable tx and rx clock source
	 */
	/* turn off tx clock */
	ghc = INL(dp, GHC);
	OUTL(dp, GHC, ghc & ~(GHC_TXCKSRC | GHC_TCPCKSRC));

	/* turn off rx clock */
	OUTL(dp, GPREG1, INL(dp, GPREG1) | GPREG1_dis_clk_rx);
#endif
	/* kick tx */
	OUTL(dp, TXCS, lp->txcs | TXCS_EN);
	FLSHL(dp, TXCS);

	/* kick rx */
	OUTL(dp, RXCS, lp->rxcs | RXCS_QST | RXCS_EN);
	FLSHL(dp, RXCS);

#ifdef WA_RXFIFO
	/*
	 * enable tx and rx clock source again
	 */
	/* restore tx clock */
	OUTL(dp, GHC, ghc);

	/* turn on rx clock */
	OUTL(dp, GPREG1, INL(dp, GPREG1) & ~GPREG1_dis_clk_rx);
#endif
	lp->ien = OUR_INTR_BITS;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* enable interrupt mask */
		OUTL(dp, IENC, ~0U);
		FLSHL(dp, IENC);
		OUTL(dp, IENS, lp->ien);
		FLSHL(dp, IENS);
	}

	return (GEM_SUCCESS);
}

static int
jmge_stop_chip(struct gem_dev *dp)
{
	int	i;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* Disable interrupts by clearing the interrupt mask */
		OUTL(dp, IENC, ~0U);
		FLSHL(dp, IENC);
	}

	/* Stop Tx and Rx processes in the chip. */
	/* XXX - don't stop rx ant tx clock, otherwise the chip hang */
	OUTL(dp, RXCS, lp->rxcs | (0 << RXCS_QSEL_SHIFT));
	for (i = 0; (INL(dp, RXCS) & RXCS_EN); i++) {
		if (i >= 50) {
			/* timeout */
			cmn_err(CE_NOTE,
			    "!%s: %s: timeout: tx didn't stopped",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(1*1000);
	}

	OUTL(dp, TXCS, lp->txcs);
	for (i = 0; (INL(dp, TXCS) & TXCS_EN); i++) {
		if (i >= 50) {
			/* timeout */
			cmn_err(CE_NOTE,
			    "!%s: %s: timeout: tx didn't stopped",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(1*1000);
	}

	return (jmge_reset_chip(dp));
}

static void
jmge_fixup_params(struct gem_dev *dp)
{
	struct jmge_dev	*lp = dp->private;

	dp->misc_flag &=
	    ~(GEM_CKSUM_NOFULL_IPv4 | GEM_LSO_IPv4_PHCKSUM);

#ifdef CONFIG_CKSUM_OFFLOAD
	/* jmc250/260 requires psuedo header checksum */
	if (dp->mtu <= ETHERMTU) {
		dp->misc_flag |= GEM_CKSUM_NOFULL_IPv4;
#ifdef CONFIG_LSO
		dp->misc_flag |= GEM_LSO_IPv4_PHCKSUM;
#endif
	}
#endif /* CONFIG_CKSUM_OFFLOAD */

#ifdef CONFIG_VLAN_HW
	dp->misc_flag |= GEM_VLAN_HARD;
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	/* round up receive buffer size to the next 8 byte boundary */
	dp->rx_buf_len = ROUNDUP(dp->rx_buf_len, 8);
}

static int
jmge_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	uval;
	uint8_t		*mac;
	static uint8_t	zero6[] = {0, 0, 0, 0, 0, 0};
	struct jmge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	mac = &dp->dev_addr.ether_addr_octet[0];

	/* reload mac address from non-volatile memory */
	uval = INL(dp, SMBCSR);
	if (uval & SMBCSR_EEPROMD) {
		/* reload mac from eeprom */
		DPRINTF(0, (CE_CONT, "!%s: %s: eeprom exists",
		    dp->name, __func__));

		OUTL(dp, SMBCSR, uval | SMBCSR_CNACK);
		OUTL(dp, SMBCSR, uval | SMBCSR_CNACK | SMBCSR_RELOAD);

		/* wait about 12ms */
		delay(drv_usectohz(12000));

		for (i = 0; i < 200; i++) {
			if ((INL(dp, SMBCSR) & SMBCSR_RELOAD) == 0) {
				DPRINTF(0, (CE_CONT,
				    "!%s: %s: eeprom has been reloaded %x, %x",
				    dp->name, __func__,
				    INL(dp, RXUMA), INL(dp, RXUMA + 4)));
				goto reloaded;
			}
			/* reloading in progress */
			delay(drv_usectohz(10000));
		}
#ifdef CONFIG_EFUSE
	} else if (FULL_MASK_REV(lp->chiprev) >= 5) {
		ddi_acc_handle_t	hdl;

		/* reload mac from eFuse */
		if (pci_config_setup(dp->dip, &hdl) != DDI_SUCCESS) {
			goto failed;
		}
		uval = pci_config_get32(hdl, PCIREG_PRIV_EFUSE1);
		if ((uval &
		    (PCICMD_EFUSE1_INITLDDONE | PCICMD_EFUSE1_AUTOLDERRFLG))
		    == PCICMD_EFUSE1_INITLDDONE) {
			DPRINTF(0, (CE_CONT, "!%s: %s: reloading eFuse",
			    dp->name, __func__));

			/* reset eFuse controller */

			uval = pci_config_get32(hdl, PCIREG_PRIV_EFUSE2);
			pci_config_put32(hdl, PCIREG_PRIV_EFUSE2,
			    uval | PCICMD_EFUSE2_RESET);

			uval = pci_config_get32(hdl, PCIREG_PRIV_EFUSE2);
			pci_config_put32(hdl, PCIREG_PRIV_EFUSE2,
			    uval & ~PCICMD_EFUSE2_RESET);

			/*
			 * issue autoload command to load mac address from
			 * eFuse to Mac controller
			 */
			uval = pci_config_get32(hdl, PCIREG_PRIV_EFUSE1) &
			    ~PCICMD_EFUSE1_CMDSELMSK;
			pci_config_put32(hdl, PCIREG_PRIV_EFUSE1,
			    PCICMD_EFUSE1_CMDSELAUTOLD |
			    PCICMD_EFUSE1_CMDREQ |
			    uval);

			for (i = 0; i < 30; i++) {
				if ((pci_config_get32(hdl, PCIREG_PRIV_EFUSE1)
				    & PCICMD_EFUSE1_CMDREQ) == 0) {
					/* done */
					DPRINTF(0, (CE_CONT,
				    "!%s: %s: eFuse has been reloaded %x, %x",
					    dp->name, __func__,
					    INL(dp, RXUMA),
					    INL(dp, RXUMA + 4)));
					break;
				}
				drv_usecwait(10);
			}
		}
		pci_config_teardown(&hdl);
#else
	} else {
		goto failed;
#endif /* CONFIG_EFUSE */
	}
reloaded:
	uval = INL(dp, RXUMA);
	mac[0] = (uint8_t)uval;
	mac[1] = (uint8_t)(uval >> 8);
	mac[2] = (uint8_t)(uval >> 16);
	mac[3] = (uint8_t)(uval >> 24);
	uval = INL(dp, RXUMA + 4);
	mac[4] = (uint8_t)uval;
	mac[5] = (uint8_t)(uval >> 8);

failed:
	if ((mac[0] & 1) || bcmp(mac, zero6, ETHERADDRL) == 0) {
		gem_generate_macaddr(dp, mac);
	}

	/* we use eeprom autoload function to get mac address */
	gem_get_mac_addr_conf(dp);

	if (lp->fpga == 0) {
		dp->mii_phy_addr = 1;
	}

	lp->phy_cable_length = gem_prop_get_int(dp, "cable_length", 0);
	lp->phy_flp_gap = gem_prop_get_int(dp, "flp_gap", 3);

#ifndef GEM_CONFIG_JUMBO_FRAME
	jmge_fixup_params(dp);
#endif
	return (GEM_SUCCESS);
}

static int
jmge_get_stats(struct gem_dev *dp)
{
	/* not implemented */
	return (GEM_SUCCESS);
}

/*
 * discriptor manupiration
 */
static int
jmge_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	uint32_t		mark;
	uint32_t		total_size;
	struct tx_desc		*tdp;
#ifdef CONFIG_CKSUM_OFFLOAD
	uint32_t		mss;
#endif
	ddi_dma_cookie_t	*dcp;
#ifdef CONFIG_VLAN_HW
	uint32_t		vtag;
#endif
	struct jmge_dev		*lp = dp->private;

#if DEBUG_LEVEL > 3
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags: 0x%llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif

	/*
	 * write tx descriptor(s)
	 */
	total_size = 0;
	dcp = dmacookie;
	frags++;
	for (i = 1; i < frags; i++, dcp++) {
		uint32_t	len;
		uint64_t	tmp;
		uint32_t	addrh;
		uint32_t	addrl;

		tdp = &((struct tx_desc *)dp->tx_ring)[
		    SLOT(slot + i, dp->gc.gc_tx_ring_size)];

		len = dcp->dmac_size;
		total_size += len;
		tmp = dcp->dmac_laddress;
		addrh = (uint32_t)(tmp >> 32);
		addrl = (uint32_t)tmp;
		tdp->td0 = addrh ?
		    LE_32(TD0_OWN | TD0_64BIT) : LE_32(TD0_OWN);
		tdp->td1 = LE_32(len);
		tdp->td2 = LE_32(addrh);	/* high addr */
		tdp->td3 = LE_32(addrl);	/* low addr */
	}

	/* for the first fragment */
	mark = TD0_INT;
	if ((flags & GEM_TXFLAG_HEAD) == 0) {
		mark |= TD0_OWN;
	}
#ifdef CONFIG_CKSUM_OFFLOAD
	mss = (flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;
	if (mss) {
		mark |= TD0_LSEN;
		mss <<= TD1_MSS_SHIFT;

	} else if (flags & GEM_TXFLAG_TCP) {
		mark |= TD0_TCPCS;

	} else if (flags & GEM_TXFLAG_UDP) {
		mark |= TD0_UDPCS;

	} else if (flags & GEM_TXFLAG_IPv4) {
		mark |= TD0_IPCS;
	}
#endif /* CONFIG_CKSUM_OFFLOAD */

#ifdef CONFIG_VLAN_HW
	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		mark |= TD0_TAGON | vtag;
	}
#endif

	tdp = &((struct tx_desc *)dp->tx_ring)[
	    SLOT(slot, dp->gc.gc_tx_ring_size)];
	tdp->td0 = LE_32(mark);
#ifdef CONFIG_CKSUM_OFFLOAD
	tdp->td1 = LE_32(mss);
#else
	tdp->td1 = 0;
#endif
	tdp->td2 = LE_32(total_size);
	tdp->td3 = 0;

	return (frags);
}

static void
jmge_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint_t	tail;
	struct jmge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called, mac_active:%x, ien:%b",
	    dp->name, __func__, dp->mac_active, lp->ien, IEVE_BITS));

	/*
	 * Append an additional not-OWNed descriptor to terminate
	 * the list of current active tx descriptors.
	 * The nic will clear OWN bit in the first desciptor for tx
	 * buffers on completion of transmit, but OWN bits in secondary
	 * descriptors are left.
	 * So, descriptors after transmit, possibly have *garbage*
	 * OWN bit. We must clear them by ourselves.
	 */
	tail = SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size);
	((struct tx_desc *)dp->tx_ring)[tail].td0 = 0;

	/* flush tx descriptors we made including the termination slot  */
	gem_tx_desc_dma_sync(dp,
	    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size), nslot,
	    DDI_DMA_SYNC_FORDEV);

	((struct tx_desc *)dp->tx_ring)[start_slot].td0 |= LE_32(TD0_OWN);
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	OUTL(dp, TXCS, lp->txcs | TXCS_QnS(0) | TXCS_EN);
}

static void
jmge_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rx_desc		*rdp;
	uint32_t		addrl;
	uint32_t		addrh;
	uint32_t		len;
	uint64_t		tmp;
#if DEBUG_LEVEL > 3
	int			i;

	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif

	/*
	 * write a RX descriptor
	 */
	rdp = &((struct rx_desc *)dp->rx_ring)[slot * 2];
	len = dmacookie->dmac_size;
	tmp = dmacookie->dmac_laddress;
	addrh = (uint32_t)(tmp >> 32);
	addrl = (uint32_t)tmp;

	rdp[1].rd3 = LE_32(addrl);
	rdp[1].rd2 = LE_32(addrh);
	rdp[1].rd1 = LE_32(len);
	rdp[1].rd0 = addrh ?
	    LE_32(RD0_OWN | RD0_64BIT) : LE_32(RD0_OWN);
	rdp[0].rd3 = 0;
	rdp[0].rd2 = 0;
	rdp[0].rd1 = 0;
	rdp[0].rd0 = LE_32(RD0_OWN | RD0_INT);
}

static void
jmge_tx_desc_dump(struct gem_dev *dp, int slot)
{
	struct tx_desc		*tdp;

	gem_tx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);

	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	cmn_err(CE_CONT,
	    "!%s: %s: slot:%d td:%b %x %x %x",
	    dp->name, __func__, slot,
	    LE_32(tdp->td0), TD0_BITS,
	    LE_32(tdp->td1),
	    LE_32(tdp->td2),
	    LE_32(tdp->td3));
}

static uint_t
jmge_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_desc	*tdp;
	uint32_t	td0;
	struct jmge_dev	*lp = dp->private;
	uint_t		ret;

	/*
	 * check the first descriptor of the packet
	 */
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	td0 = tdp->td0;
	td0 = LE_32(td0);

	if (td0 & TD0_OWN) {
		ret = 0;
	}

	return (ret);
}

static uint64_t
jmge_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_desc	*rdp;
	uint32_t	rd0;
	uint32_t	rd1;
	uint64_t	ret;
	struct jmge_dev	*lp = dp->private;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot * 2];

	rd0 = rdp->rd0;
	rd0 = LE_32(rd0);
	rd1 = rdp->rd1;
	rd1 = LE_32(rd1);

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d %b %b",
	    dp->name, __func__, slot,
	    rd0, RD0_BITS, rd1, RD1_BITS));

	if ((rd0 & RD0_OWN) || (rd1 & RD1_WBCPL) == 0) {
		/* it isn't received yet */
		return (0);
	}

#define	RD1_ERRS	\
	(RD1_LIMIT | RD1_MIIERR | RD1_NIBON | RD1_COLON |	\
	RD1_ABORT | RD1_SHORT | RD1_OVERUN | RD1_CRCERR)

#ifdef WA_SHORT_PACKETS
	if ((rd1 & RD1_SHORT) && (rd1 & RD1_RECVFS) >= ETHERMIN) {
		rd1 &= ~RD1_SHORT;
	}
#endif
#ifdef WA_CRC_ERROR
	if ((rd1 & RD1_CRCERR) && dp->speed == GEM_SPD_1000) {
		rd1 &= ~RD1_CRCERR;
	}
#endif
	if (((rd1 & RD1_DCNT) >> RD1_DCNT_SHIFT) != 2 ||
	    (rd1 & RD1_ERRS)) {

		/* errored packet */
		dp->stats.errrcv++;
#ifdef notdef
		if ((rd1 & RD1_LIMIT) ||
		    ((rd1 & RD1_DCNT) >> RD1_DCNT_SHIFT) > 2) {
			dp->stats.frame_too_long++;
		} else if (rd1 & RD1_MIIERR) {
			dp->stats.rcv_internal_err++;
		} else if (rd1 & RD1_NIBON) {
			dp->stats.crc++;
		} else if (rd1 & RD1_COLON) {
			dp->stats.missed++;
		} else if (rd1 & RD1_ABORT) {
			dp->stats.missed++;
		} else if (rd1 & RD1_SHORT) {
			dp->stats.runt++;
		} else if (rd1 & RD1_OVERUN) {
			dp->stats.overflow++;
		} else if (rd1 & RD1_CRCERR) {
			dp->stats.crc++;
		}
#else
		if (((rd1 & RD1_DCNT) >> RD1_DCNT_SHIFT) != 2) {
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: slot:%d, error:%b, len:%d",
			    dp->name, __func__,
			    slot, rd1, RD1_BITS, rd1 & RD1_RECVFS));
			lp->need_to_reset = B_TRUE;
		}
		if ((rd1 & RD1_LIMIT) ||
		    ((rd1 & RD1_DCNT) >> RD1_DCNT_SHIFT) > 2) {
			dp->stats.frame_too_long++;
		}
		if (rd1 & RD1_MIIERR) {
			dp->stats.rcv_internal_err++;
		}
		if (rd1 & RD1_NIBON) {
			dp->stats.crc++;
		}
		if (rd1 & RD1_COLON) {
			dp->stats.missed++;
		}
		if (rd1 & RD1_ABORT) {
			dp->stats.missed++;
		}
		if (rd1 & RD1_SHORT) {
			dp->stats.runt++;
		}
		if (rd1 & RD1_OVERUN) {
			dp->stats.overflow++;
		}
		if (rd1 & RD1_CRCERR) {
			dp->stats.crc++;
		}
#endif
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/*
	 * if POEN bit in PRXMCS is set, recieved packets have
	 * additional 10 byte header for alignment.
	 */
ok:
	ret = rd1 & RD1_RECVFS;
#ifdef CONFIG_CKSUM_OFFLOAD
	if (rd0 & RD0_TCPCS) {
		ret |= GEM_RX_CKSUM_TCP;
	}
	if (rd0 & RD0_UDPCS) {
		ret |= GEM_RX_CKSUM_UDP;
	}
	if (rd0 & RD0_IPCS) {
		if (rd0 & RD0_IPV4) {
			ret |= GEM_RX_CKSUM_IPv4;
		}
		if (rd0 & RD0_IPV6) {
			ret |= GEM_RX_CKSUM_IPv6;
		}
	}
#endif
	return (GEM_RX_DONE | ret);
}

static void
jmge_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct tx_desc *)dp->tx_ring)[slot],
	    sizeof (struct tx_desc));
}

static void
jmge_rx_desc_init(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct rx_desc *)dp->rx_ring)[slot * 2],
	    sizeof (struct rx_desc) * 2);
}

static void
jmge_tx_desc_clean(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct tx_desc *)dp->tx_ring)[slot],
	    sizeof (struct tx_desc));
}

static void
jmge_rx_desc_clean(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct rx_desc *)dp->rx_ring)[slot * 2],
	    sizeof (struct rx_desc) * 2);
}

/*
 * Device depend interrupt handler
 */
static uint_t
jmge_interrupt(struct gem_dev *dp)
{
	uint32_t	ieve;
	uint32_t	ieve_org;
	uint_t		flag = 0;
	struct jmge_dev	*lp = dp->private;
#ifdef NEVER
	ieve_org = ieve = INL(dp, IEVE);
#else
	jmge_ioarea_dma_sync(dp, 0, dp->gc.gc_io_area_size,
	    DDI_DMA_SYNC_FORKERNEL);
	ieve_org = ieve = ((uint32_t *)dp->io_area)[0];
#endif
	if ((ieve & lp->ien) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(4, (CE_CONT, "!%s: Interrupt, ieve: %b, active:%x",
	    dp->name, ieve, IEVE_BITS, dp->mac_active));

	/* clear interrupt resource */
#ifdef CONFIG_POLLING
	/* XXX - don't clear interrupt source until coalesced events happen */
	ieve &= ~(IEVE_RX0 | IEVE_TX0);

	if (ieve & (IEVE_PCCRX0TO | IEVE_PCCRX0)) {
		/*
		 * We need clear rx coalescing timer/counter and
		 * rx interrupt source.
		 */
		ieve |= IEVE_PCCRX0TO | IEVE_PCCRX0 | IEVE_RX0;
	}
	if (ieve & (IEVE_PCCTXTO | IEVE_PCCTX)) {
		/*
		 * We need clear tx coalescing timer/counter and
		 * tx interrupt source.
		 */
		ieve |= IEVE_PCCTXTO | IEVE_PCCTX | IEVE_TX0;
	}
#endif
	OUTL(dp, IENC, lp->ien);
	OUTL(dp, IEVE, ieve);

	if (!IS_MAC_ONLINE(dp)) {
		/* the device is not active, no more interrupts */
		lp->ien = 0;

		/* disable interrupt */
		OUTL(dp, IENC, ~lp->ien);
		FLSHL(dp, IENC);

		((uint32_t *)dp->io_area)[0] = 0;
		jmge_ioarea_dma_sync(dp, 0, dp->gc.gc_io_area_size,
		    DDI_DMA_SYNC_FORDEV);

		return (DDI_INTR_CLAIMED);
	}
#if defined(RESET_TEST) && (RESET_TEST >= 1000)
	lp->reset_test++;
	if ((lp->reset_test % RESET_TEST) == RESET_TEST-1) {
		lp->need_to_reset = B_TRUE;
	}
#endif
	ieve &= lp->ien;

	/* barrier to commit the interrupt status */
	FLSHL(dp, IEVE);

	((uint32_t *)dp->io_area)[0] &= ~ieve;
	jmge_ioarea_dma_sync(dp, 0, dp->gc.gc_io_area_size,
	    DDI_DMA_SYNC_FORDEV);

	if (ieve & (IEVE_PCCRX0TO | IEVE_PCCRX0 | IEVE_RX0EMP | IEVE_RX0)) {
		/*
		 * One or more packets were received, or receive error
		 * happened
		 */
		(void) gem_receive(dp);

		if (ieve & IEVE_RX0EMP) {
			DPRINTF(0, (CE_CONT, "!%s: no rx desc", dp->name));
			dp->stats.norcvbuf++;
			/* Kick RX again */
			OUTL(dp, RXCS,
			    lp->rxcs | RXCS_QST | RXCS_EN);
		}
	}

	if (ieve & (IEVE_PCCTXTO | IEVE_PCCTX | IEVE_TX0)) {
		/*
		 * One or more packets were transmitted or transmit error
		 * happened
		 */
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

	if (ieve & IEVE_LINK) {
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			flag |= INTR_RESTART_TX;
		}
#endif
	}

	if (lp->need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flag |= INTR_RESTART_TX;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IENS, lp->ien);
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routines
 */
static void
jmge_mii_sync(struct gem_dev *dp)
{
	/* EMPTY */
}

static uint16_t
jmge_mii_read(struct gem_dev *dp, uint_t reg)	/* ok */
{
	int		i;
	uint32_t	val;

	OUTL(dp, SMI,
	    reg << SMI_REGADDR_SHIFT
	    | dp->mii_phy_addr << SMI_PHYADDR_SHIFT | SMI_REQ);
	drv_usecwait(20);

	i = 0;
	while ((val = INL(dp, SMI)) & SMI_REQ) {
		if (++i > 5000) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(20);
	}
	return ((uint16_t)((val & SMI_DATA) >> SMI_DATA_SHIFT));
}

static void
jmge_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)	/* ok */
{
	int	i;

	OUTL(dp, SMI,
	    ((uint32_t)val) << SMI_DATA_SHIFT
	    | reg << SMI_REGADDR_SHIFT
	    | dp->mii_phy_addr << SMI_PHYADDR_SHIFT | SMI_WR | SMI_REQ);
	drv_usecwait(20);

	i = 0;
	while (INL(dp, SMI) & SMI_REQ) {
		if (++i > 5000) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(20);
	}
}

static uint16_t
jmge_phy_specreg_read(struct gem_dev *dp, uint_t reg)
{
	jmge_mii_write(dp, PHY_SPEC_ADDR_REG, PHY_SPEC_REG_READ | reg);
	return (jmge_mii_read(dp, PHY_SPEC_DATA_REG));
}

static void
jmge_phy_specreg_write(struct gem_dev *dp, uint_t reg, uint16_t data)
{
	jmge_mii_write(dp, PHY_SPEC_DATA_REG, data);
	jmge_mii_write(dp, PHY_SPEC_ADDR_REG, PHY_SPEC_REG_WRITE | reg);
}

static void
jmge_phy_off(struct gem_dev *dp)
{
	uint_t	reg;
	ddi_acc_handle_t	hdl;
	struct jmge_dev	*lp = dp->private;

	jmge_mii_write(dp, MII_CONTROL,
	    jmge_mii_read(dp, MII_CONTROL) | MII_CONTROL_PWRDN);

	if (FULL_MASK_REV(lp->chiprev) >= 5) {
		reg = INL(dp, PHYPWR);
		reg |= PHYPWR_DOWN1_SEL | PHYPWR_DOWN1_SW |
		    PHYPWR_DOWN2 | PHYPWR_CLK_SEL;
		OUTL(dp, PHYPWR, reg);

		if (pci_config_setup(dp->dip, &hdl) == DDI_SUCCESS) {
			reg = pci_config_get32(hdl, PCI_PRIV_PE1);
			reg &= ~PE1_GPREG0_PBG;
			reg |= PE1_GPREG0_PDD3COLD;
			pci_config_put32(hdl, PCI_PRIV_PE1, reg);
			pci_config_teardown(&hdl);
		}
	}
}

static void
jmge_phy_on(struct gem_dev *dp)
{
	uint_t	reg;
	ddi_acc_handle_t	hdl;
	struct jmge_dev	*lp = dp->private;

	jmge_mii_write(dp, MII_CONTROL,
	    jmge_mii_read(dp, MII_CONTROL) & ~MII_CONTROL_PWRDN);

	if (FULL_MASK_REV(lp->chiprev) >= 5) {
		reg = INL(dp, PHYPWR);
		reg &= ~(PHYPWR_DOWN1_SEL | PHYPWR_DOWN1_SW |
		    PHYPWR_DOWN2 | PHYPWR_CLK_SEL);
		OUTL(dp, PHYPWR, reg);

		if (pci_config_setup(dp->dip, &hdl) == DDI_SUCCESS) {
			reg = pci_config_get32(hdl, PCI_PRIV_PE1);
			reg &= ~PE1_GPREG0_PBG;
			reg |= PE1_GPREG0_ENBG;
			pci_config_put32(hdl, PCI_PRIV_PE1, reg);
			pci_config_teardown(&hdl);
		}
	}
}

static int
jmge_mii_init(struct gem_dev *dp)
{
	int		ret = GEM_SUCCESS;
	uint16_t	phy_data;
	uint16_t	ctrl1000;
	ddi_acc_handle_t	hdl;
	uint8_t	nic_ctrl;
	struct jmge_dev	*lp = dp->private;

	/*
	 * enable phy
	 */
	if (lp->fpga == 0) {
		jmge_mii_write(dp, 26,
		    jmge_mii_read(dp, 26) & ~0x1000);
	}
	jmge_phy_off(dp);
	jmge_phy_on(dp);

#define	MII_1000TC_TESTMODE_SHIFT	13

	jmge_mii_write(dp, MII_CONTROL,
	    jmge_mii_read(dp, MII_CONTROL) | MII_CONTROL_RESET);
	drv_usecwait(20*1000);

	/* enabel PHY test mode 1 */
	ctrl1000 = jmge_mii_read(dp, MII_1000TC);
	ctrl1000 &= ~MII_1000TC_TESTMODE;
	ctrl1000 |= 1U << MII_1000TC_TESTMODE_SHIFT;
	jmge_mii_write(dp, MII_1000TC, ctrl1000);

	phy_data = jmge_phy_specreg_read(dp, PHY_EXT_COMM_2_REG);
	phy_data &= ~PHY_EXT_COMM_2_CALI_MODE_0;
	phy_data |= PHY_EXT_COMM_2_CALI_LATCH |
	    PHY_EXT_COMM_2_CALI_ENABLE;
	jmge_phy_specreg_write(dp, PHY_EXT_COMM_2_REG, phy_data);

	delay(drv_usectohz(20*1000));

	phy_data = jmge_phy_specreg_read(dp, PHY_EXT_COMM_2_REG);
	phy_data &= ~(PHY_EXT_COMM_2_CALI_ENABLE |
	    PHY_EXT_COMM_2_CALI_MODE_0 |
	    PHY_EXT_COMM_2_CALI_LATCH);
	jmge_phy_specreg_write(dp, PHY_EXT_COMM_2_REG, phy_data);

	/* disable PHY test mode */
	ctrl1000 = jmge_mii_read(dp, MII_1000TC);
	ctrl1000 &= ~MII_1000TC_TESTMODE;
	jmge_mii_write(dp, MII_1000TC, ctrl1000);

#undef	MII_1000TC_TESTMODE_SHIFT

	if (pci_config_setup(dp->dip, &hdl) != DDI_SUCCESS) {
		goto x;
	}
	nic_ctrl = pci_config_get8(hdl, PCI_PRIV_SHARE_NICCTRL);
	pci_config_teardown(&hdl);

	if ((nic_ctrl & 0x3) != FLAG_PHYEA_ENABLE) {
		uint16_t	phy_comm0 = 0;
		uint16_t	phy_comm1 = 0;

		switch (lp->chip->devid) {
		case PCI_DEVID_JMC250:
			if (lp->chiprev == FM_ECO(5, 0) ||
			    lp->chiprev == FM_ECO(5, 1) ||
			    lp->chiprev == FM_ECO(5, 3) ||
			    FULL_MASK_REV(lp->chiprev) >= 6) {
				phy_comm0 = 0x008a;
				phy_comm1 = 0x4109;
			}
			if (lp->chiprev == FM_ECO(3, 1) ||
			    lp->chiprev == FM_ECO(3, 2)) {
				phy_comm0 = 0xe088;
			}
			break;

		case PCI_DEVID_JMC260:
			if (lp->chiprev == FM_ECO(5, 0) ||
			    lp->chiprev == FM_ECO(5, 1) ||
			    lp->chiprev == FM_ECO(5, 3) ||
			    FULL_MASK_REV(lp->chiprev) >= 6) {
				phy_comm0 = 0x008a;
				phy_comm1 = 0x4109;
			}
			if (lp->chiprev == FM_ECO(3, 1) ||
			    lp->chiprev == FM_ECO(3, 2)) {
				phy_comm0 = 0xe088;
			}
			if (lp->chiprev == FM_ECO(2, 0)) {
				phy_comm0 = 0x608a;
			}
			if (lp->chiprev == FM_ECO(2, 2)) {
				phy_comm0 = 0x408a;
			}
			break;
		}

		if (phy_comm0) {
			jmge_phy_specreg_write(dp,
			    PHY_EXT_COMM_0_REG, phy_comm0);
		}
		if (phy_comm1) {
			jmge_phy_specreg_write(dp,
			    PHY_EXT_COMM_1_REG, phy_comm1);
		}
	}
x:
	/* tune phy for speed up */
	jmge_mii_write(dp, 23,
	    lp->phy_cable_length << 12 |
	    lp->phy_cable_length << 8 |
	    lp->phy_cable_length << 4 |
	    lp->phy_cable_length);

	phy_data = jmge_mii_read(dp, 24);
	jmge_mii_write(dp, 24,
	    (phy_data & ~0x1f00) | lp->phy_flp_gap << 8);

	DPRINTF(0, (CE_CONT, "!%s: %s: reg23:%x, reg24: %x->%x",
	    dp->name, __func__, jmge_mii_read(dp, 23),
	    phy_data, jmge_mii_read(dp, 24)));

	return (ret);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
jmgeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	int			ret;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			revid;
	int			unit;
	struct chip_info	*p;
	int			len;
	struct pci_phys_spec	*regs;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct jmge_dev	*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint32_t		val;
	uint32_t		cap_ptr;
	uint32_t		cap;
	uint32_t		ps;
	uint32_t		ilr;
	uint_t			pcie_cap;
	uint32_t		devcap;
	uint32_t		devcsr = 0;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: %s: called at time:%d (%s)",
	    drv_name, unit, __func__, ddi_get_lbolt(), ident));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
		    drv_name, unit);
		goto err;
	}
#if DEBUG_LEVEL >= 10
	if (cmd == DDI_RESUME) {
		OUTL(GEM_GET_DEV(dip), PMCS, 0xffff0000U);
	}
#endif
	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);
#if DEBUG_LEVEL >= 10
	if (cmd == DDI_RESUME) {
		jmge_reset_chip(GEM_GET_DEV(dip));
	}
#endif
#if DEBUG_LEVEL >= 10
	for (i = 0; i < 16; i++) {
		cmn_err(CE_CONT, "!%s%d: %x: %08x",
		    drv_name, unit,
		    i*4, pci_config_get32(conf_handle, i*4));
	}
#endif
	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);

	DPRINTF(0, (CE_CONT,
	    "!%s%d: ilr 0x%08x, latency_timer:0x%02x",
	    drv_name, unit,
	    pci_config_get32(conf_handle, PCI_CONF_ILINE),
	    pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER)));

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* check bus type */
	pcie_cap = gem_search_pci_cap(dip, conf_handle, PCI_CAP_ID_PCI_E);

	if (pcie_cap == 0) {
		cmn_err(CE_WARN, "!%s%d: no pcie cap",
		    drv_name, unit);
		goto err;
	}
	devcap = pci_config_get32(conf_handle, pcie_cap + 4);
	devcsr = pci_config_get32(conf_handle, pcie_cap + 8);

	DPRINTF(0, (CE_CONT,
	    "!%s: pcie cap:%x, dev_cap:%x, dev_csr:%x",
	    __func__,
	    pci_config_get32(conf_handle, pcie_cap),
	    devcap, devcsr));

	pci_config_teardown(&conf_handle);
	conf_handle = NULL;

	switch (cmd) {
	case DDI_RESUME:
#if 0
		OUTL(GEM_GET_DEV(dip), PMCS, 0xffff0000U);
		jmge_reset_chip(GEM_GET_DEV(dip));
		jmge_attach_chip(GEM_GET_DEV(dip));
		gem_mii_probe_default(GEM_GET_DEV(dip));
#endif
		return (gem_resume(dip));

	case DDI_ATTACH:
		/*
		 * Check if the chip is supported.
		 */
		for (i = 0, p = jmge_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (p->venid == vid && p->devid == did) {
				/* found */
				cmn_err(CE_CONT,
				    "!%s%d: %s (vid: 0x%04x, did: 0x%04x,"
				    " revid: 0x%02x)",
				    drv_name, unit, p->name, vid, did, revid);
				goto chip_found;
			}
		}
		/* Not found */
		cmn_err(CE_WARN,
		    "!%s: jmge_attach: wrong PCI venid/devid (0x%x, 0x%x)",
		    drv_name, vid, did);

		/* assume JMC250 */
		p = jmge_chiptbl;
chip_found:
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
		    PCI_CONF_BASE0, 0xff, &jmge_dev_attr, &base, &regs_ha)
		    != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: gem_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		lp = kmem_zalloc(sizeof (struct jmge_dev), KM_SLEEP);
		lp->revid = revid;
		lp->maxrdreq = (devcsr >> 12) & 7;
		lp->chip = p;

		ddi_put32(regs_ha, (uint32_t *)(base + PMCS), 0xffff0000U);

		val = ddi_get32(regs_ha, (uint32_t *)(base + CHIPMODE));
		lp->fpga = (val & CHIPMODE_FPGA);
		lp->chiprev = (val & CHIPMODE_REV) >> CHIPMODE_REV_SHIFT;
		DPRINTF(0, (CE_CONT,
		    "!%s%d: fpga_rev:0x%x, fm_rev:0x%x, eco_rev:0x%x",
		    drv_name, unit, lp->fpga,
		    FULL_MASK_REV(lp->chiprev), ECO_REV(lp->chiprev)));

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* configuration on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = GEM_MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
		gcp->gc_tx_desc_unit_shift = 4; /* 16 byte */
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		/*
		 * We require an additional blank descriptor to terminate the
		 * list of the valid tx descripters, because the nic doesn't
		 * clear OWN bit automatically on completion of transmit.
		 */
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = jmge_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint64_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 5;	/* 16*2 byte */
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_copy_thresh = jmge_rx_copy_thresh;

		/* prepare io area for shadow status */
		gcp->gc_io_area_size = 32;

		/* map attributes */
		gcp->gc_dev_attr = jmge_dev_attr;
		gcp->gc_buf_attr = jmge_buf_attr;
		gcp->gc_desc_attr = jmge_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = jmge_dma_attr_desc;
		if (lp->chip->devid == PCI_DEVID_JMC260) {
			gcp->gc_dma_attr_txbuf = jmge_dma_attr_buf_260;
		} else {
			gcp->gc_dma_attr_txbuf = jmge_dma_attr_buf_250;
		}
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		if (lp->chip->devid == PCI_DEVID_JMC260) {
			gcp->gc_dma_attr_rxbuf = jmge_dma_attr_buf_260;
		} else {
			gcp->gc_dma_attr_rxbuf = jmge_dma_attr_buf_250;
		}
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* timeout parameters */
		gcp->gc_tx_timeout = ONESEC*3;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		if (lp->chip->devid == PCI_DEVID_JMC260) {
			gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
		} else {
			gcp->gc_flow_control = FLOW_CONTROL_NONE;
		}

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* workaround for PHY */
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RSA;
		gcp->gc_mii_dont_reset = B_TRUE;
		gcp->gc_mii_hw_link_detection = B_TRUE;
		gcp->gc_mii_stop_mac_on_linkdown = B_TRUE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &jmge_attach_chip;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &jmge_fixup_params;
#endif
		gcp->gc_reset_chip = &jmge_reset_chip;
		gcp->gc_init_chip = &jmge_init_chip;
		gcp->gc_start_chip = &jmge_start_chip;
		gcp->gc_stop_chip = &jmge_stop_chip;
		gcp->gc_multicast_hash = &jmge_mcast_hash;
		gcp->gc_set_rx_filter = &jmge_set_rx_filter;
		gcp->gc_set_media = &jmge_set_media;
		gcp->gc_get_stats = &jmge_get_stats;
		gcp->gc_interrupt = &jmge_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &jmge_tx_desc_write;
		gcp->gc_rx_desc_write = &jmge_rx_desc_write;
		gcp->gc_tx_start = &jmge_tx_start;
		gcp->gc_rx_start = NULL;

		gcp->gc_tx_desc_stat = &jmge_tx_desc_stat;
		gcp->gc_rx_desc_stat = &jmge_rx_desc_stat;
		gcp->gc_tx_desc_init = &jmge_tx_desc_init;
		gcp->gc_rx_desc_init = &jmge_rx_desc_init;
		gcp->gc_tx_desc_clean = &jmge_tx_desc_clean;
		gcp->gc_rx_desc_clean = &jmge_rx_desc_clean;
		gcp->gc_get_packet = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = &jmge_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &jmge_mii_sync;
		gcp->gc_mii_read = &jmge_mii_read;
		gcp->gc_mii_write = &jmge_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* MSI/MSIX interrupts */
		gcp->gc_nintrs_req = 1;

		/*
		 * offload and jumbo frame:
		 * I won't support jumbo frames (max 9kbyte) because
		 * the chipset cannot offload checksum for jumbo frames.
		 */
		gcp->gc_max_lso = 64 * 1024 - 1;
		gcp->gc_max_mtu = 9*1024;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));
		if (dp == NULL) {
			cmn_err(CE_WARN, "!%s%d: gem_do_attach failed",
			    drv_name, unit);
			goto err_free_mem;
		}

		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof (struct jmge_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
jmgedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;
	ddi_acc_handle_t	conf_handle;

	switch (cmd) {
	case DDI_SUSPEND:
		ret = gem_suspend(dip);
#if 0
		if (pci_config_setup(dip, &conf_handle) == DDI_SUCCESS) {
			(void) gem_pci_set_power_state(dip,
			    conf_handle, PCI_PMCSR_D3HOT);
			pci_config_teardown(&conf_handle);
		}
#endif
		return (ret);

	case DDI_DETACH:
		return (gem_do_detach(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef GEM_CONFIG_GLDv3
GEM_STREAM_OPS(jmge_ops, jmgeattach, jmgedetach);
#else
static	struct module_info jmgeminfo = {
	0,			/* mi_idnum */
	"jmge",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit jmgerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&jmgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit jmgewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&jmgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	jmge_info = {
	&jmgerinit,	/* st_rdinit */
	&jmgewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_jmge_ops = {
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
	&jmge_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops jmge_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	jmgeattach,	/* devo_attach */
	jmgedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_jmge_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce,	/* devo_quiesce */
#endif
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&jmge_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/* ======================================================== */
/*
 * _init :
 */
/* ======================================================== */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, "!jmge: _init: called"));
	status = gem_mod_init(&jmge_ops, "jmge");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&jmge_ops);
	}
	return (status);
}

/*
 * _fini :
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!jmge: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&jmge_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
