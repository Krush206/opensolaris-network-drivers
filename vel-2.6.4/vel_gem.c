/*
 * vel: VIA Technology Velocity series Gigabit Ethernet MAC driver
 *
 * Copyright (c) 2002-2008 Masayuki Murayama.  All rights reserved.
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

#pragma	ident	"@(#)vel_gem.c 1.11     08/12/31"

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
#include "vt612xreg.h"

char	ident[] = "via vt612x nic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int vel_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (vel_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else /* MAP_MEM */
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif /* MAP_MEM */

/*
 * Our configuration
 */
#define	OUR_INTR_BITS	\
	(ISR_TXSTL | ISR_RXSTL | ISR_MIBF | ISR_SHDN | ISR_PHY | \
	ISR_TMR1 | ISR_TMR0 | ISR_SRC | ISR_LSTE | ISR_OVF | \
	ISR_RACE | ISR_TXWB1 | ISR_TXWB0 | ISR_PTX)

#ifdef TEST_TXDESC_FULL
#undef	TX_BUF_SIZE
#define	TX_BUF_SIZE	16
#endif
#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	256		/* hardware max is 256 */
#endif

#ifdef TEST_RX_EMPTY
#undef	RX_BUF_SIZE
#define	RX_BUF_SIZE	32
#endif
#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256		/* hardware max is 256 */
#endif

#define	CONFIG_TX_SINGLE_QUEUE

#ifdef CONFIG_TX_COPY
static int	vel_tx_copy_thresh = INT32_MAX;
#else
static int	vel_tx_copy_thresh = 256;
#endif
#ifdef CONFIG_RX_COPY
static int	vel_rx_copy_thresh = INT32_MAX;
#else
static int	vel_rx_copy_thresh = 256;
#endif

/*
 * veline chip state
 */
struct vel_dev {
	uint8_t			mac_addr[ETHERADDRL];
	uint8_t			revid;	/* chip revision id */
	uint16_t		rft;	/* rx fifo threshold */
	uint16_t		maxdma;
	volatile uint32_t	imr;
#ifdef CONFIG_POLLING
	int			last_poll_interval;
#endif
#ifdef RESET_TEST
	int			reset_test;
#endif
	int			rx_tail;

	/* device type */
	boolean_t		pcie;
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info vel_chiptbl[] = {
	{VID_VIA, DID_VT6122, "VT6122"},
};
#define	CHIPTABLESIZE   (sizeof (vel_chiptbl) / sizeof (struct chip_info))

/*
 * Macros to identify chip generation.
 */

/* ======================================================== */

/* mii operations */
static void  vel_mii_sync(struct gem_dev *);
static uint16_t  vel_mii_read(struct gem_dev *, uint_t);
static void vel_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int vel_attach_chip(struct gem_dev *);
static int vel_reset_chip(struct gem_dev *);
static int vel_init_chip(struct gem_dev *);
static int vel_start_chip(struct gem_dev *);
static int vel_stop_chip(struct gem_dev *);
static int vel_set_media(struct gem_dev *);
static int vel_set_rx_filter(struct gem_dev *);
static int vel_get_stats(struct gem_dev *);

/* descriptor operations */
static int vel_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void vel_tx_start(struct gem_dev *dp, int slot, int frags);
static void vel_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t vel_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static void vel_rx_start(struct gem_dev *dp, int slot, int frags);
static uint64_t vel_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void vel_tx_desc_init(struct gem_dev *dp, int slot);
static void vel_rx_desc_init(struct gem_dev *dp, int slot);
static void vel_tx_desc_clean(struct gem_dev *dp, int slot);
static void vel_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t vel_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr vel_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr vel_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

#ifdef CONFIG_DAC
static ddi_dma_attr_t vel_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x0000ffffffffffffull,	/* dma_attr_addr_hi */
	0x00003fffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003fff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000000000003fffull,	/* dma_attr_maxxfer */
	0x0000ffffffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};
#else
static ddi_dma_attr_t vel_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x00003fffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003fff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x00003fffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};
#endif
static ddi_dma_attr_t vel_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	64,			/* dma_attr_align */
	0x1f,			/* dma_attr_burstsizes */
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
static int
vel_reset_chip(struct gem_dev *dp)
{
	int		i;
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, time:%d",
	    dp->name, __func__, ddi_get_lbolt()));
#ifdef notdef
	/* white a dummy mac address to check eeprom autoloading works */
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(dp, PAR + i, i);
		lp->mac_addr[i] = i;
	}
#endif
	/* inhibit interrupts */
	OUTL(dp, IMR, 0);
	OUTL(dp, CR_CLR, CR_GINTMSK1);

	/* Reset the nic. */
	/* XXX - reset doesn't change mac address */
	OUTL(dp, CR_SET, CR_SRST);

	drv_usecwait(10);
	for (i = 0; INL(dp, CR_SET) & CR_SRST; i++) {
		drv_usecwait(10);
		if (i > 1000) {
			cmn_err(CE_NOTE, "!%s: %s: timeout",
			    dp->name, __func__);
			/* Use force software reset bit instead of */
			OUTL(dp, CR_SET, CR_FORSRST);
			drv_usecwait(2000);
			break;
		}
	}
	drv_usecwait(5000);

	/* clear ISR */
	OUTL(dp, ISR, 0xffffffffU);

	DPRINTF(4, (CE_CONT, "!%s: %s: mac[5]:%x",
	    dp->name, __func__, INB(dp, PAR+5)));

	/* synchronzie read cache for mac address */
	for (i = 0; i < ETHERADDRL; i++) {
		lp->mac_addr[i] = INB(dp, PAR + i);
	}

	return (GEM_SUCCESS);
}

static int
vel_init_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* PAR: call setmedia later */

	/* clear PreACPI bit */
	OUTB(dp, CFGA, INB(dp, CFGA) & ~CFGA_PACPI);

	/* CFGD */
	DPRINTF(2, (CE_CONT, "!%s: %s: CFGD:0x%02x",
	    dp->name, __func__, INB(dp, CFGD)));
	OUTB(dp, CFGD, INB(dp, CFGD) | CFGD_CFGDACEN | CFGD_PCI64EN);

	/* TCR */
	val = INB(dp, TCR) & ~TCR_LB;
	OUTB(dp, TCR, val | TCR_LB_NORMAL);

	/* MCFG: rx threshold and tag etc, don't set MCFG_VIDFR */
	val = INW(dp, MCFG0) & ~(MCFG_RFT | MCFG_VTAG);
	val |= (lp->rft << MCFG_RFT_SHIFT);
#ifdef CONFIG_VLAN_HW
	val |= (VTAG_OPT2 << MCFG_VTAG_SHIFT);
#endif
	OUTW(dp, MCFG0, val);
	DPRINTF(0, (CE_CONT, "!%s: mcfg:%b", dp->name, val, MCFG_BITS));

	/* DCFG: dma length */
	val = INW(dp, DCFG) & ~(DCFG_XMRL | DCFG_PERDIS | DCFG_DMA);
#ifdef CONFIG_MRM
	val |= DCFG_XMRL;	/* disable MRL */
#endif
#ifdef CONFIG_LATMEN
	val |= DCFG_LATMEN;
#endif
	val |= lp->maxdma << DCFG_DMA_SHIFT;
	OUTW(dp, DCFG, val);
	DPRINTF(0, (CE_CONT, "!%s: DCFG:%b",
	    dp->name, INW(dp, DCFG), DCFG_BITS));

	/* WOL */
	OUTB(dp, WOLCFG_SET, WOLCFG_SAM | WOLCFG_SAB);

	/* CFGB */
	val = INB(dp, CFGB);
	OUTB(dp, CFGB, val | CFGB_CRANDOM | CFGB_CAP | CFGB_MBA | CFGB_BAKOPT);

	/* Rx Desc Addr */
	OUTL(dp, RDBASE_LO, (uint32_t)dp->rx_ring_dma);
	OUTW(dp, RDINDEX, 0);
	OUTW(dp, RDCSIZE, dp->gc.gc_rx_ring_size - 1);
	OUTB(dp, RDCSR_SET, RDCSR_RUN);
	OUTB(dp, RDCSR_SET, RDCSR_WAK);	/* activate rx dma engine */

	/* Tx Desc Addr: it has 4 transmit queues */
	OUTW(dp, TDCSIZE, dp->gc.gc_tx_ring_size - 1);

#ifdef CONFIG_TX_SINGLE_QUEUE
	OUTL(dp, TDBASE_LO, (uint32_t)dp->tx_ring_dma);
	OUTW(dp, TDCSR_SET, TDCSR_RUN(0));
	for (i = 1; i < NUM_TX_QUEUES; i++) {
		OUTL(dp, TDBASE_LO + sizeof (uint32_t)*i, 0);
		OUTW(dp, TDCSR_CLR, TDCSR_RUN(i));
	}
#else
	/* XXX - not implemented */
	for (i = 0; i < NUM_TX_QUEUES; i++) {
		OUTL(dp, TDBASE_LO + sizeof (uint32_t)*i,
		    ((uint32_t)dp->tx_ring_dma) +
		    dp->gc.gc_tx_ring_size * sizeof (struct tx_desc) * i);
		OUTW(dp, TDCSR_SET, TDCSR_RUN(i));
	}
#endif
	/* init cam filter: later in set_rx_filter */

	/* initialize vlan cam mask */
	OUTB(dp, CAMCR, CAMCR_SEL_MASK);
	OUTB(dp, CAMADDR, CAMADDR_EN | CAMADDR_VCAMSL);
	for (i = 0; i < 8; i++) {
		OUTB(dp, MAR + i, 0);
	}
	OUTB(dp, CAMADDR, 0);
	OUTB(dp, CAMCR, CAMCR_SEL_MAR);

	/* init flow control */
	OUTL(dp, CR_CLR, CR_TXLOWAT | CR_TXHIWAT);
	OUTL(dp, CR_SET, CR_XONEN | CR_TXLOWAT_24 | CR_TXHIWAT_48);
	OUTW(dp, PAUSE_TIMER, 0xffffU);

	/* set software timer resolution to 1uS */
	val = INB(dp, GMCR) | GMCR_TMR0US | GMCR_TMR1US;
	OUTB(dp, GMCR, val);

	/* init adaptive interrupts */
	/* Set Tx Interrupt Suppression Threshold */
	OUTB(dp, CAMCR, CAMCR_SEL_MASK);
	OUTW(dp, ISRCTL, 31);	/* originally 31 */

	/* Set Rx Interrupt Suppression Threshold */
	/* XXX - 15 improved rx performance in my experience, but why ? */
	OUTB(dp, CAMCR, CAMCR_SEL_DATA);
	OUTW(dp, ISRCTL, 31);	/* originally 31 */
	OUTB(dp, CAMCR, 0);

	lp->rx_tail = 0;

	return (GEM_SUCCESS);
}

static uint_t
vel_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is higher 6 bits of msb in crc */
	return (gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6));
}

static int
vel_set_rx_filter(struct gem_dev *dp)
{
	uint8_t		mode;
	int		i, j;
	uint32_t	mhash[2];
	uint8_t		mcmask[MULTICAST_CAM_SIZE/8];
	uint8_t		*mac;
	static uint8_t	invalid_mac[ETHERADDRL] = {0, 0, 0, 0, 0, 0};
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d, rxmode:%b mc_count:%d",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, dp->rxmode, RXMODE_BITS,
	    dp->mc_count));

	/*
	 * temporally make rx filter promiscious mode not to lose any
	 * packets while changing it.
	 */
	OUTL(dp, MAR + 0, 0xffffffffU);
	OUTL(dp, MAR + 4, 0xffffffffU);
	OUTB(dp, RCR, RCR_SEP | RCR_AR | RCR_AB | RCR_AM | RCR_PRO);
	FLSHB(dp, RCR);

	mode = RCR_SEP | RCR_AR | RCR_AB;	/* broadcast */
	mode |= RCR_AL;	/* for long packets and soft vlan */

	mhash[0] = mhash[1] = 0;
	bzero(mcmask, sizeof (mcmask));

	if (dp->rxmode & RXMODE_PROMISC) {
		mode |= RCR_PRO	/* promiscous */
		    | RCR_AM;	/* muticast */
		mhash[0] = mhash[1] = 0xffffffffU;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) ||
	    dp->mc_count > MULTICAST_CAM_SIZE) {
		mode |= RCR_AM;	/* multicast */
		mhash[0] = mhash[1] = 0xffffffffU;
	} else if (dp->mc_count > 0) {
#ifndef USE_MCAST_CAM
		mode |= RCR_AM;	/* multicast */

		for (i = 0; i < dp->mc_count; i++) {
			j = dp->mc_list[i].hash;
			mhash[j / 32] |= 1 << (j % 32);
		}
#else
		/*
		 * Use multicast cam. I guess multicast hash should be zero.
		 * VT612x has 64 entry CAMs for multicast address filtering.
		 */

		/* RCR_AP bit must be set to enable the CAM filter. */
		mode |= RCR_AP;

		/* enable cam port for writing multicast addresses */
		OUTB(dp, CAMCR, CAMCR_SEL_DATA);

		for (i = 0; i < dp->mc_count; i++) {
			/* write cam index */
			OUTB(dp, CAMADDR, CAMADDR_EN | i);

			/* write a multicast address into cam */
			mac = dp->mc_list[i].addr.ether_addr_octet;
			for (j = 0; j < ETHERADDRL; j++) {
				OUTB(dp, MAR + j, mac[j]);
			}
			mcmask[i / 8] |= 1 << (i % 8);

			/* issue a cam write command */
			OUTB(dp, CAMCR, CAMCR_SEL_DATA | CAMCR_WR);
			FLSHB(dp, CAMCR);
			drv_usecwait(10);
#if DEBUG_LEVEL > 3
			/* issue a cam read command */
			OUTB(dp, CAMCR, CAMCR_SEL_DATA | CAMCR_RD);
			FLSHB(dp, CAMCR);
			drv_usecwait(10);

			cmn_err(CE_CONT,
			    "!%s: rx_fitler:"
			    " mcast: %02x:%02x:%02x:%02x:%02x:%02x added",
			    dp->name,
			    INB(dp, MAR + 0), INB(dp, MAR + 1),
			    INB(dp, MAR + 2), INB(dp, MAR + 3),
			    INB(dp, MAR + 4), INB(dp, MAR + 5));
#endif
		}
		/* close to access multicast CAM */
		OUTB(dp, CAMADDR, 0);
		OUTB(dp, CAMCR, CAMCR_SEL_MAR);
#endif /* USE_MCAST_CAM */
	}

	/* set my mac address */
	mac = &dp->cur_addr.ether_addr_octet[0];
	if (bcmp(lp->mac_addr, mac, ETHERADDRL)) {
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, mac[i]);
			lp->mac_addr[i] = mac[i];
		}
	}

	if (mode & RCR_AP) {
		/* setup multicast cam mask */
		OUTB(dp, CAMCR, CAMCR_SEL_MASK);
		OUTB(dp, CAMADDR, CAMADDR_EN);
		for (i = 0; i < 8; i++) {
			OUTB(dp, MAR + i, mcmask[i]);
		}
		OUTB(dp, CAMADDR, 0);
		OUTB(dp, CAMCR, CAMCR_SEL_MAR);
	}

	if (mode & RCR_AM) {
		/* setup the multicast hash table */
		for (i = 0; i < 2; i++) {
			OUTL(dp, MAR + i*4, mhash[i]);
		}
	}

	/* install new rx filter mode */
	OUTB(dp, RCR, mode);

	return (GEM_SUCCESS);
}

static int
vel_set_media(struct gem_dev *dp)
{
	uint32_t	val;
	int		speed;
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d, REVID:%02x, TCR:%02x, GMCR:%b",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active,
	    INB(dp, REVID), INB(dp, TCR), INB(dp, GMCR), GMCR_BITS));

	val = INB(dp, GMCR) | GMCR_FCMODE;

	/* select PHY, aka, speed */
	val &= ~GMCR_FCGMII;
	if (dp->speed == GEM_SPD_1000) {
		val |= GMCR_FCGMII;
	}

	/* select duplex */
	val &= ~GMCR_FCFDX;
	if (dp->full_duplex) {
		val |= GMCR_FCFDX;
	}

	OUTB(dp, GMCR, val);

	/* flow control */
	switch (dp->flow_control) {
	case FLOW_CONTROL_SYMMETRIC:
		val = CR_FDXTFCEN | CR_FDXRFCEN;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		val = CR_FDXTFCEN;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		val = CR_FDXRFCEN;
		break;

	case FLOW_CONTROL_NONE:
	default:
		val = 0;
		break;
	}

	OUTL(dp, CR_CLR, (CR_FDXTFCEN | CR_FDXRFCEN | CR_HDXFCEN) & ~val);
	OUTL(dp, CR_SET, val);

	/* select heartbeat for 10m half */
	val = INB(dp, TESTCFG) | TESTCFG_HBDIS;
	if ((!dp->full_duplex) && dp->speed == GEM_SPD_10) {
		val &= ~TESTCFG_HBDIS;
	}
	OUTB(dp, TESTCFG, val);

	/* workaround for older revisions */
	if (INB(dp, REVID) < REVID_VT3216_A0) {
		val = INB(dp, TCR) & ~TCR_TB2BDIS;
		if (dp->full_duplex) {
			val |= TCR_TB2BDIS;
		}
		OUTB(dp, TCR, val);
	}

	if (lp->revid >= REVID_VT3216_A0) {
		/* XXX - adaptive interrupt doesn't seem to work */
#if 0
		if (dp->speed == GEM_SPD_1000 || dp->speed == GEM_SPD_100) {
			OUTB(dp, TQETMR, 0x59); /* 100us */
			OUTB(dp, TQETMR, 0x14);	/* 20us */
		} else {
			OUTB(dp, TQETMR, 0x00);
			OUTB(dp, TQETMR, 0x00);
		}
#else
		OUTB(dp, TQETMR, 0x00);
		OUTB(dp, TQETMR, 0x00);
#endif
	}
	return (GEM_SUCCESS);
}

static int
vel_start_chip(struct gem_dev *dp)
{
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* Kick TX and RX */
	OUTL(dp, CR_CLR, CR_STOP);
	OUTL(dp, CR_SET, CR_TXON | CR_RXON | CR_DPOLL | CR_STRT);

	/* enable interrupt mask */
	lp->imr = OUR_INTR_BITS;
#ifdef CONFIG_POLLING
	lp->imr |= ISR_PRX;
#else
	lp->imr |= ISR_PPRX;
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: imr:%b",
	    dp->name, __func__, lp->imr, ISR_BITS));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IMR, lp->imr);
		OUTL(dp, CR_SET, CR_GINTMSK1);
	}

	return (GEM_SUCCESS);
}

static int
vel_stop_chip(struct gem_dev *dp)
{
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* Disable interrupts by clearing the interrupt mask */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, CR_CLR, CR_GINTMSK1);
		OUTL(dp, IMR, 0);
	}

	/* Stop Tx and Rx processes in the chip. */
	OUTL(dp, CR_SET, CR_STOP);
	OUTW(dp, TDCSR_CLR, 0xffffU);
	OUTB(dp, RDCSR_CLR, 0xffU);

	return (GEM_SUCCESS);
}

#ifdef DEBUG_LEVEL
static uint16_t
vel_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;
	uint16_t	ret;
	uint8_t		chip_select;
	uint8_t		di;
	uint8_t		cfgc_saved;

#define	VEL_EEPROM_DELAY(dp)	{ \
	volatile uint32_t x; x = INB(dp, EECSR); x = INB(dp, EECSR); \
}
#define	EE93C46_READ	6

	/* enable eeprom direct programing */
	cfgc_saved = INB(dp, CFGC);
	OUTB(dp, CFGC, cfgc_saved | CFGC_EELOAD);
	VEL_EEPROM_DELAY(dp);

	/* ensure de-assert chip select */
	chip_select = EECSR_DPM;
	OUTB(dp, EECSR, chip_select);
	VEL_EEPROM_DELAY(dp);

	/* assert chip select */
	chip_select |= EECSR_ECS;
	OUTB(dp, EECSR, chip_select);
	VEL_EEPROM_DELAY(dp);

	/* make a read command for eeprom */
	offset = (offset & 0x3f) | (EE93C46_READ << 6);

	for (i = 10; i >= 0; i--) {
		/* send 1 bit */
		di = ((offset >> i) & 1) << EECSR_EDI_SHIFT;

		OUTB(dp, EECSR, chip_select | di);
		VEL_EEPROM_DELAY(dp);

		OUTB(dp, EECSR, chip_select | di | EECSR_ECK);
		VEL_EEPROM_DELAY(dp);
	}

	OUTB(dp, EECSR, chip_select);
	VEL_EEPROM_DELAY(dp);

	/* get the reply and construct a 16bit value */
	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTB(dp, EECSR, chip_select | EECSR_ECK);
		VEL_EEPROM_DELAY(dp);

		ret = (ret << 1)
		    | ((INB(dp, EECSR) >> EECSR_EDO_SHIFT) & 1);

		OUTB(dp, EECSR, chip_select);
		VEL_EEPROM_DELAY(dp);
	}

	/* negate chip_select */
	OUTB(dp, EECSR, EECSR_DPM);
	VEL_EEPROM_DELAY(dp);

	OUTB(dp, EECSR, 0);
	VEL_EEPROM_DELAY(dp);

	/* disable eeprom direct programming */
	OUTB(dp, CFGC, cfgc_saved);
	VEL_EEPROM_DELAY(dp);

	return (ret);
}

static void
vel_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	prom[0x10];

	for (i = 0; i < 0x10; i++) {
		prom[i] = vel_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 0x10; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
}
#endif /* DEBUG_LEVEL */

static int
vel_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint_t		val;
	struct vel_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	DPRINTF(4, (CE_CONT, "!%s: %d.%09d",
	    dp->name, hrestime.tv_sec, hrestime.tv_nsec));

	/* we use eeprom autoload function to get mac address */
	OUTB(dp, EECSR, INB(dp, EECSR) | EECSR_AUTOLD);

	/*
	 * XXX - Don't access velocity for 20mS.
	 * It doesn't respond to any PCI transactions while it
	 * loads eeprom. On sparc, it may cause bus error.
	 */
	delay(drv_usectohz(20000));

	DPRINTF(4, (CE_CONT, "!%s: %d.%09d",
	    dp->name, hrestime.tv_sec, hrestime.tv_nsec));

	for (i = 0; INB(dp, EECSR) & EECSR_AUTOLD; i++) {
		if (i > 4000) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout: initializing the nic",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}
	DPRINTF(4, (CE_CONT, "!%s: %d.%09d",
	    dp->name, hrestime.tv_sec, hrestime.tv_nsec));

	/* get factory mac address */
	for (i = 0; i < ETHERADDRL; i++) {
		dp->dev_addr.ether_addr_octet[i] = INB(dp, PAR + i);
	}
	bcopy(dp->dev_addr.ether_addr_octet, lp->mac_addr, ETHERADDRL);

#if DEBUG_LEVEL > 2
	vel_eeprom_dump(dp);
#endif /* DEBUG_LEVEL */

	/* read default address of PHY in eeprom */
	dp->mii_phy_addr = INB(dp, MIIADR) & MIIADR_PHYAD;
	DPRINTF(2, (CE_CONT, "!%s: default PHY address:%d",
	    dp->name, dp->mii_phy_addr));

	val = dp->mtu;	/* vendor recommendation: no limit */

	if (val <= 32) {
		lp->maxdma = DMA_32;
	} else if (val <= 64) {
		lp->maxdma = DMA_64;
	} else if (val <= 128) {
		lp->maxdma = DMA_128;
	} else if (val <= 256) {
		lp->maxdma = DMA_256;
	} else if (val <= 512) {
		lp->maxdma = DMA_512;
	} else if (val <= 1024) {
		lp->maxdma = DMA_1024;
	} else {
		lp->maxdma = DMA_NOLIMIT;
	}

	if (dp->rxthr <= 128) {
		lp->rft = RFT_128;	/* vendor recommendation */
	} else if (dp->rxthr <= 512) {
		lp->rft = RFT_512;
	} else if (dp->rxthr <= 1024) {
		lp->rft = RFT_1024;
	} else {
		lp->rft = RFT_SF;
	}

	dp->misc_flag |= GEM_POLL_RXONLY;
#ifdef CONFIG_CKSUM_OFFLOAD
	dp->misc_flag |= (GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4);
#endif
#ifdef CONFIG_VLAN_HW
	dp->misc_flag |= GEM_VLAN_HARD;
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif

	return (GEM_SUCCESS);
}

static int
vel_get_stats(struct gem_dev *dp)
{
	/* not implemented */
	return (GEM_SUCCESS);
}

/*
 * discriptor manupiration
 */
static int
vel_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	uint32_t		tdes1;
	uint64_t		total_size;
	struct tx_desc		*tdp;
	uint64_t		tmp;
#ifdef CONFIG_VLAN_HW
	uint64_t		vtag;
#endif
	struct vel_dev		*lp = dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags: 0x%llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 2
	flags |= GEM_TXFLAG_INTR;
#endif
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	/*
	 * write tx descriptor(s)
	 */
	total_size = 0;
	i = frags - 1;
	do {
		uint64_t	len;

		len = dmacookie[i].dmac_size;
		total_size += len;
		tmp = (len << TBDES_LEN_SHIFT) | dmacookie[i].dmac_laddress;
		tdp->tbdes[i] = LE_64(tmp);
	} while (--i >= 0);

	tdp->tbdes[0] |= LE_64(TBDES_QUEUE);

	tdes1 = TDES1_TCPLS_NORMAL | ((frags + 1) << TDES1_CMDZ_SHIFT);
	if (flags & GEM_TXFLAG_INTR) {
		tdes1 |= TDES1_TCR_TIC;
	}
	if (total_size > ETHERMAX) {
		tdes1 |= TDES1_TCR_JUMBO;
	}

#ifdef CONFIG_CKSUM_OFFLOAD
#if (TDES1_TXFLAG_IPv4 == (TDES1_TXFLAG_UDP << 1)) && \
	(TDES1_TXFLAG_IPv4 == (TDES1_TXFLAG_TCP << 2))
	tdes1 |=
	    ((flags & (GEM_TXFLAG_IPv4 | GEM_TXFLAG_UDP | GEM_TXFLAG_TCP))
	    >> GEM_TXFLAG_TCP_SHIFT) << TDES1_TCR_TCPCK_SHIFT;
#else
	if (flags & GEM_TXFLAG_IPv4) {
		tdes1 |= TDES1_TCR_IPCK;
	}
	if (flags & GEM_TXFLAG_TCP) {
		tdes1 |= TDES1_TCR_TCPCK;
	} else if (flags & GEM_TXFLAG_UDP) {
		tdes1 |= TDES1_TCR_UDPCK;
	}
#endif
#endif /* CONFIG_CKSUM_OFFLOAD */

#ifdef CONFIG_VLAN_HW
	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		tdes1 |= TDES1_TCR_VETAG | vtag;
	}
#endif

	tdp->tdes1 = LE_32(tdes1);
	tmp = TDES0_OWN | (total_size << TDES0_LEN_SHIFT);
	tdp->tdes0 = LE_32(tmp);

	return (1);
}

static void
vel_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int		slot;
	uint64_t	le64_queue;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct vel_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, mac_active:%x, imr:%b",
	    dp->name, __func__, dp->mac_active, lp->imr, ISR_BITS));

	le64_queue = LE_64(TBDES_QUEUE);

	/* remove QUEUE bit from the last descriptor */
	slot = SLOT(start_slot + nslot - 1, tx_ring_size);
	((struct tx_desc *)dp->tx_ring)[slot].tbdes[0] &= ~le64_queue;

	/* flush tx descriptors we made */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	/*
	 * kick Tx engine
	 *  (1) set QUEUE bit to td buffer[0] in the previous descriptor.
	 *  (2) wake up the tx queue.
	 */
#ifdef TXTIMEOUT_TEST
	if ((vel_send_cnt++ % 100) == 99) {
		OUTL(dp, CR_CLR, CR_TXON);
	}
#endif
	slot = SLOT(start_slot - 1, tx_ring_size);
	((struct tx_desc *)dp->tx_ring)[slot].tbdes[0] |= le64_queue;
	gem_tx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);

	OUTW(dp, TDCSR_SET, TDCSR_WAK(0));
}

static void
vel_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rx_desc		*rdp;
	uint64_t		daddr;
	uint32_t		tmp;
#if DEBUG_LEVEL > 2
	int			i;

	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif

	/*
	 * write a RX descriptor
	 */
	rdp = &((struct rx_desc *)dp->rx_ring)[slot];
	daddr = dmacookie->dmac_laddress;

	rdp->rdes1 = 0;
	rdp->rdes2 = LE_32((uint32_t)daddr);
	tmp = (((uint32_t)dmacookie->dmac_size) << RDES3_RBS_SHIFT)
	    |  RDES3_INTR
	    |  ((uint32_t)(daddr >> 32));
	rdp->rdes3 = LE_32(tmp);
#ifdef SANITY
	rdp->rdes0 = 0;
#endif
}

static void
vel_tx_desc_dump(struct gem_dev *dp, int slot)
{
	int			i;
	int			frags;
	uint32_t		tdes0;
	uint32_t		tdes1;
	struct tx_desc		*tdp;

	gem_tx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);

	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	tdes0 = tdp->tdes0;
	tdes0 = LE_32(tdes0);
	tdes1 = tdp->tdes1;
	tdes1 = LE_32(tdes1);
	frags = ((tdes1 & TDES1_CMDZ) >> TDES1_CMDZ_SHIFT) - 1;

	cmn_err(CE_CONT,
	    "!%s: %s: time: %d slot:%d frags:%d tsr:%b txdesc1:%b",
	    dp->name, __func__, ddi_get_lbolt(), slot, frags,
	    tdes0, TDES0_BITS, tdes1, TDES1_BITS);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: tbdes: %xll", i, tdp->tbdes[i]);
	}
}

static uint_t
vel_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_desc		*tdp;
	uint32_t		tsr;
#ifdef DEBUG_LEVEL
	int			frags;
	uint32_t		tdes1;
#endif
	struct vel_dev		*lp = dp->private;

	/*
	 * check the descriptor
	 */
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	tsr = tdp->tdes0;
	tsr = LE_32(tsr);
#if DEBUG_LEVEL > 2
	tdes1 = LE_32(tdp->tdes1);
	frags = ((tdes1 & TDES1_CMDZ) >> TDES1_CMDZ_SHIFT) - 1;

	cmn_err(CE_CONT,
	    "!%s: %s: time: %d slot:%d frags:%d tsr:%b txdesc1:%b",
	    dp->name, __func__, ddi_get_lbolt(), slot, frags,
	    LE_32(tdp->tdes0), TDES0_BITS,
	    LE_32(tdp->tdes1), TDES1_BITS);
{
	int	i;

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: tb: %xll", i, tdp->tbdes[i]);
	}
}
#endif
	if (!(tsr & (TSR_TXERR | TDES0_OWN))) {
#ifdef notdef
		int	cols;

		if (!dp->full_duplex && (cols = (tsr & TSR_NCR)) > 0) {
			dp->stats.collisions += cols;
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* if (cols > 1) */ {
				dp->stats.multi_coll++;
			}
		}
#endif
		/* short cut for normal case */
		return (GEM_TX_DONE);
	}

	if (tsr & TDES0_OWN) {
		/* not transmitted yet */
		return (0);
	}

	DPRINTF(2, (CE_CONT, "!%s: tx error: tsr: %b  desc:%d",
	    dp->name, tsr, TDES0_BITS, dp->tx_desc_head));
	dp->stats.errxmt++;

	if (tsr & TSR_CRS) {
		dp->stats.nocarrier++;
	} else if (tsr & TSR_OWC) {
		dp->stats.xmtlatecoll++;
	} else if (tsr & TSR_ABT) {
		if (!dp->full_duplex) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		}
#ifdef notdef
		vel_tx_desc_dump(dp, slot);
#endif
	}

	return (GEM_TX_DONE);
}

static void
vel_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int		new_tail;
	int		n;
	int		slot;
	uint32_t	own;
	uint_t		rx_ring_size = dp->gc.gc_rx_ring_size;
	struct vel_dev	*lp = dp->private;

	/*
	 * Rx side of vt612x stops when descriptors exhausted.
	 * To restart rx dma engine correctly, it must stop at 4N
	 * boundary of rx descriptor. So, we add rx descriptors by 4.
	 */
	new_tail = SLOT((start_slot + nslot) & ~3, rx_ring_size);
	n = SLOT(new_tail - lp->rx_tail, rx_ring_size);
	if (n == 0) {
		/* only queue the new rx buffers */
		return;
	}

	own = LE_32(RDES0_OWN);
	if (n > 1) {
		slot = SLOT(lp->rx_tail + 1, rx_ring_size);
		while (slot != new_tail) {
			((struct rx_desc *)dp->rx_ring)[slot].rdes0 = own;
			slot = SLOT(slot + 1, rx_ring_size);
		}
		gem_rx_desc_dma_sync(dp,
		    SLOT(lp->rx_tail + 1, rx_ring_size),
		    n - 1, DDI_DMA_SYNC_FORDEV);
	}

	((struct rx_desc *)dp->rx_ring)[lp->rx_tail].rdes0 = own;
	gem_rx_desc_dma_sync(dp, lp->rx_tail, 1, DDI_DMA_SYNC_FORDEV);

	OUTW(dp, RBRDU, n);

	/* save the new tail */
	lp->rx_tail = new_tail;
}

static uint64_t
vel_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_desc		*rdp;
	uint32_t		rsr;
	uint32_t		rdes1;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	rsr = rdp->rdes0;
	rsr = LE_32(rsr);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d dma len/addr:0x%x.%x rsr 0x%b, rdes1:%x",
	    dp->name, __func__, slot,
	    LE_32(rdp->rdes3), LE_32(rdp->rdes2),
	    LE_32(rdp->rdes0), RSR_BITS, LE_32(rdp->rdes1)));

#define	RSR_ERRS	(RSR_RXER | RSR_LONG | RSR_FAE | RSR_CRC)

	if (!(rsr & (RDES0_OWN | RSR_STP | RSR_EDP | RSR_ERRS))) {
		uint64_t	pflags = 0;
#ifdef CONFIG_CKSUM_OFFLOAD
		uint32_t	rdes1;

		rdes1 = rdp->rdes1;
		rdes1 = LE_32(rdes1);

		if ((rdes1 & (RDES1_IPKT | RDES1_IPOK)) ==
		    (RDES1_IPKT | RDES1_IPOK)) {
			pflags |= GEM_RX_CKSUM_IPv4;
		}

		if ((rdes1 & (RDES1_TCPKT | RDES1_TUPOK)) ==
		    (RDES1_TCPKT | RDES1_TUPOK)) {
			pflags |= GEM_RX_CKSUM_TCP;
		} else if ((rdes1 & (RDES1_UDPKT | RDES1_TUPOK)) ==
		    (RDES1_UDPKT | RDES1_TUPOK)) {
			pflags |= GEM_RX_CKSUM_UDP;
		}
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef CONFIG_VLAN_HW
		if (rsr & RSR_VTAG) {
			uint64_t	vtag;
			/* extract vtag */
			vtag = BSWAP_16(rdes1 & RDES1_VTAG);
			pflags |= vtag << GEM_RX_VTAG_SHIFT;
		}
#endif /* CONFIG_VLAN_HW */
		DPRINTF(2, (CE_CONT,
		    "!%s: %s: slot:%d dma len/addr:0x%x.%x rsr 0x%b, rdes1:%x",
		    dp->name, __func__, slot,
		    LE_32(rdp->rdes3), LE_32(rdp->rdes2),
		    LE_32(rdp->rdes0), RSR_BITS, LE_32(rdp->rdes1)));

		return (GEM_RX_DONE | pflags |
		    (((rsr >> RDES0_LEN_SHIFT) - ETHERFCSL) & RDES0_LEN_MASK));
	}

	if (rsr & RDES0_OWN) {
		/* it isn't received yet */
		return (0);
	}

	if (rsr & (RSR_STP | RSR_EDP)) {
		/* big packet, ignore this fragment */
		if (rsr & RSR_STP) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
		}
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/* errored packet */
	dp->stats.errrcv++;
	if (rsr & RSR_LONG) {
		dp->stats.frame_too_long++;
	} else if (rsr & RSR_FAE) {
		dp->stats.frame++;
	} else if (rsr & RSR_CRC) {
		dp->stats.crc++;
	}

	return (GEM_RX_DONE | GEM_RX_ERR);
}

static void
vel_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct tx_desc *)dp->tx_ring)[slot],
	    sizeof (struct tx_desc));
}

static void
vel_rx_desc_init(struct gem_dev *dp, int slot)
{
	/* invalidate this descriptor */
	bzero(&((struct rx_desc *)dp->rx_ring)[slot],
	    sizeof (struct rx_desc));
}

/*
 * Device depend interrupt handler
 */
static uint_t
vel_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint32_t	isr_org;
	uint_t		flag = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct vel_dev	*lp = dp->private;

	isr_org = isr = INL(dp, ISR);

	if ((isr & lp->imr) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: Interrupt, isr: %b, active:%x",
	    dp->name, isr, ISR_BITS, dp->mac_active));

	/* clear interrupts */
	OUTL(dp, ISR, isr);

	if (!dp->mac_active) {
		/* the device is not active, no more interrupts */
		lp->imr = 0;
		/* disable interrupt */
		OUTL(dp, CR_CLR, CR_GINTMSK1);
		FLSHL(dp, CR_CLR);

		return (DDI_INTR_CLAIMED);
	}
#if defined(RESET_TEST) && (RESET_TEST >= 1000)
	lp->reset_test++;
	if ((lp->reset_test % RESET_TEST) == RESET_TEST-1) {
		need_to_reset = B_TRUE;
	}
#endif

#ifdef CONFIG_POLLING
#define	INTR_MASK	ISR_PRX

	isr &= lp->imr | INTR_MASK;

	if (dp->poll_interval != lp->last_poll_interval) {
		/*
		 * It's time to change polling rate.
		 */
		if (dp->poll_interval) {
			/* polling mode */
			lp->imr &= ~INTR_MASK;
			OUTW(dp, SOFT_TMR1, dp->poll_interval/1000);
			OUTL(dp, CR_SET, CR_TMR1EN);
		} else {
			/* normal mode */
			lp->imr |= INTR_MASK;
			OUTL(dp, CR_CLR, CR_TMR1EN);
		}
		OUTL(dp, IMR, lp->imr);

		lp->last_poll_interval = dp->poll_interval;
	}
#else
	isr &= lp->imr;
#endif /* CONFIG_POLLING */

	/* barrier to commit the interrupt status */
	FLSHL(dp, ISR);

	if (isr & (ISR_PRX | ISR_PPRX | ISR_LSTE)) {
		/* packet was received, or receive error happened */
		(void) gem_receive(dp);

		if (isr & ISR_LSTE) {
#if DEBUG_LEVEL > 3
			if ((INW(dp, RDINDEX) & 3) != 0) {
				cmn_err(CE_CONT,
				    "!%s: no rx desc: isr:%b rdcsr:%x:%x "
				    "rbrdu:%x rdindex:%d slot:%d tail:%d",
				    dp->name, isr_org, ISR_BITS,
				    INB(dp, RDCSR_SET), INB(dp, RDCSR_CLR),
				    INW(dp, RBRDU), INW(dp, RDINDEX),
				    SLOT(dp->rx_active_head, RX_BUF_SIZE),
				    lp->rx_tail);
			}
#endif
			dp->stats.norcvbuf++;
			/* Kick TX and RX */
			OUTB(dp, RDCSR_SET, RDCSR_WAK);
		}
	}

	if (isr & (ISR_PTX | ISR_PPTX)) {
		/* packets have been transmitted or transmit error happened */
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}
#ifdef notdef
	if (isr & ISR_RACE) {
		OUTW(dp, TDCSR_SET, TDCSR_WAK(0));
	}
#endif
	if (isr & ~(ISR_SHDN | ISR_TMR1 | ISR_PRX | ISR_PPRX |
	    ISR_PTX | ISR_PPTX | ISR_LSTE | ISR_LSTPE | ISR_RACE)) {
		/*
		 * handle unexpected interrupt
		 */
		cmn_err(CE_WARN,
		    "!%s: unexpected interrupt: isr:%b",
		    dp->name, isr_org, ISR_BITS);

		if (isr & ISR_TXSTL) {
#ifdef notdef
			int	i;
#endif
			/* clear interrupt reason */
			OUTB(dp, TXESR, INB(dp, TXESR) | TXESR_TDSTR);

			/* stop tx queue */
			OUTW(dp, TDCSR_CLR, TDCSR_RUN(0));

			cmn_err(CE_WARN,
			    "!%s: tx stopped at descriptor index:%d",
			    dp->name, INW(dp, TDINDEX));
#ifdef notdef
			for (i = 0; i < dp->gc.gc_tx_ring_size; i++) {
				vel_tx_desc_dump(dp, i);
			}
#endif
#ifdef notdef
			need_to_reset = B_TRUE;
#else
			OUTW(dp, TDCSR_SET, TDCSR_WAK(0));
#endif
		}
#ifdef notdef
		if (isr & (ISR_SRC | ISR_PHY)) {
			/* link status changed */
			/* XXX - doesn't work */
			DPRINTF(0, (CE_CONT,
			    "!%s: isr:%b", dp->name, isr, ISR_BITS));
			if (gem_mii_link_check(dp)) {
				flag |= INTR_RESTART_TX;
			}
		}
#endif
		if (isr & ISR_MIBF) {
			vel_get_stats(dp);
		}
	}

	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flag |= INTR_RESTART_TX;
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routines
 */
static void
vel_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
vel_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint16_t	val;

	OUTB(dp, MIIADR, reg);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_RCMD);

	for (i = 0; INB(dp, MIICR) & MIICR_RCMD; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(10);
	}
	val = INW(dp, MIIDATA);

	return (val);
}

static void
vel_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int	i;

	OUTB(dp, MIIADR, reg);
	OUTW(dp, MIIDATA, val);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_WCMD);

	for (i = 0; INB(dp, MIICR) & MIICR_WCMD; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return;
		}
		drv_usecwait(10);
	}
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
velattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct vel_dev	*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint32_t		val;
	uint32_t		cap_ptr;
	uint32_t		cap;
	uint32_t		ps;
	uint32_t		ilr;
	uint32_t		pcie_cap;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: velattach: called at time:%d (%s)",
	    drv_name, unit, ddi_get_lbolt(), ident));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
		    drv_name, unit);
		goto err;
	}

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

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	/* check bus type */
	pcie_cap = gem_search_pci_cap(dip, conf_handle, PCI_CAP_ID_PCI_E);
	if (pcie_cap) {
		uint32_t	devcap;
		uint32_t	devcsr;

		devcap = pci_config_get32(conf_handle, pcie_cap + 4);
		devcsr = pci_config_get32(conf_handle, pcie_cap + 8);

		DPRINTF(0, (CE_CONT,
		    "!%s: pcie cap:%x, dev_cap:%x, dev_csr:%x", 
		    __func__,
		    pci_config_get32(conf_handle, pcie_cap),
		    devcap, devcsr));
		/*
		 * increase max read request 2:512, 3:1024, 4:2048
		 */
		if ((devcsr & (7 << 12)) < (3 << 12)) {
			devcsr = (devcsr & ~(7 << 12)) | (3 << 12);
			pci_config_put32(conf_handle, pcie_cap + 8, devcsr);
		}
	}

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/*
		 * Check if the chip is supported.
		 */
		for (i = 0, p = vel_chiptbl; i < CHIPTABLESIZE; i++, p++) {
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
		    "!%s: vel_attach: wrong PCI venid/devid (0x%x, 0x%x)",
		    drv_name, vid, did);
chip_found:
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_CONF_BASE1, 0xff,
#else
		    PCI_CONF_BASE0, 0xff,
#endif
		    &vel_dev_attr, (caddr_t *)&base, &regs_ha)
		    != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* configuration on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = NTDBUFS;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift = 6; /* 64 byte */
		gcp->gc_tx_ring_size = TX_BUF_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_buf_size = gcp->gc_tx_ring_size;
		gcp->gc_tx_buf_limit = gcp->gc_tx_ring_limit;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = vel_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4;	/* 16 byte */
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_copy_thresh = vel_rx_copy_thresh;

		/* map attributes */
		gcp->gc_dev_attr = vel_dev_attr;
		gcp->gc_buf_attr = vel_buf_attr;
		gcp->gc_desc_attr = vel_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = vel_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = vel_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = vel_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* timeout parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* workaround for PHY */
		gcp->gc_mii_an_delay = ONESEC/10;
		/* need to reset PHY for autonegotiating 1000Mbps connection */
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_hw_link_detection = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &vel_attach_chip;
		gcp->gc_reset_chip = &vel_reset_chip;
		gcp->gc_init_chip = &vel_init_chip;
		gcp->gc_start_chip = &vel_start_chip;
		gcp->gc_stop_chip = &vel_stop_chip;
		gcp->gc_multicast_hash = &vel_mcast_hash;
		gcp->gc_set_rx_filter = &vel_set_rx_filter;
		gcp->gc_set_media = &vel_set_media;
		gcp->gc_get_stats = &vel_get_stats;
		gcp->gc_interrupt = &vel_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &vel_tx_desc_write;
		gcp->gc_rx_desc_write = &vel_rx_desc_write;
		gcp->gc_tx_start = &vel_tx_start;
		gcp->gc_rx_start = &vel_rx_start;

		gcp->gc_tx_desc_stat = &vel_tx_desc_stat;
		gcp->gc_rx_desc_stat = &vel_rx_desc_stat;
		gcp->gc_tx_desc_init = &vel_tx_desc_init;
		gcp->gc_rx_desc_init = &vel_rx_desc_init;
		gcp->gc_tx_desc_clean = &vel_tx_desc_init;
		gcp->gc_rx_desc_clean = &vel_rx_desc_init;
		gcp->gc_get_packet = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &vel_mii_sync;
		gcp->gc_mii_read = &vel_mii_read;
		gcp->gc_mii_write = &vel_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof (struct vel_dev), KM_SLEEP);
		lp->revid = revid;
		lp->pcie = pcie_cap != 0;

		/* tursn off SWPTAG right after leaving power down mode */
		val = ddi_get8(regs_ha, (uint8_t *)(base + STICKHW));
		ddi_put8(regs_ha, (uint8_t *)(base + STICKHW),
		    val & ~STICKHW_SWPTAG);
		/* clear sticky bit */
		val = ddi_get8(regs_ha, (uint8_t *)(base + STICKHW));
		ddi_put8(regs_ha, ((uint8_t *)base) + STICKHW,
		    val & ~(STICKHW_DS1 | STICKHW_DS0));

		/* disable force PME-enable */
		ddi_put8(regs_ha,
		    (uint8_t *)(base + WOLCFG_CLR), WOLCFG_PMEOVR);

		/* disable power-event config bit */
		ddi_put16(regs_ha, (uint16_t *)(base + WOLCR_CLR), 0xffffU);

		/* clear power status */
		ddi_put16(regs_ha, (uint16_t *)(base + PWRCSR_CLR), 0xffffU);

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
		kmem_free(lp, sizeof (struct vel_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
veldetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	switch (cmd) {
	case DDI_SUSPEND:
		return (gem_suspend(dip));

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
GEM_STREAM_OPS(vel_ops, velattach, veldetach);
#else
static	struct module_info velminfo = {
	0,			/* mi_idnum */
	"vel",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit velrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&velminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit velwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&velminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	vel_info = {
	&velrinit,	/* st_rdinit */
	&velwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_vel_ops = {
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
	&vel_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops vel_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	velattach,	/* devo_attach */
	veldetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_vel_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&vel_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!vel: _init: called"));
	gem_mod_init(&vel_ops, "vel");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&vel_ops);
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

	DPRINTF(2, (CE_CONT, "!vel: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&vel_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
