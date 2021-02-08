/*
 *  bfe: Broadcom 440x Fast Ethernet MAC driver for Solaris
 *
 * Copyright (c) 2003-2011 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)bfe_gem.c 1.7     11/09/19"

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
#include "bcm4400reg.h"

char	ident[] = "bcm4400 nic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int bfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (bfe_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	RX_HEAD_ROOM	(RX_HEADER_SIZE + 2)
#define	BFE_DESC(p)	((struct bfe_desc *)(void *)(p))

#define	FLUSH(dp, reg)	(void) INL(dp, reg)

#define	TXSTAT_DESC_INDEX(dp)	\
	((INL(dp, TXSTAT) & TXSTAT_CURRDSCR) / sizeof (struct bfe_desc))

#define	RXSTAT_DESC_INDEX(dp)	\
	((INL(dp, RXSTAT) & RXSTAT_CURRDSCR) / sizeof (struct bfe_desc))

#ifndef INT32_MAX
#define	INT32_MAX	0x7fffffff
#endif

#define	INSIDE(slot, head, tail)	\
	(((head) <= (tail)) ?	\
		((head) <= (slot) && (slot) < (tail)) :	\
		((slot) < (tail) || (head) <= (slot)))

/*
 * Our configuration
 */
#ifdef TEST_RX_EMPTY
#define	RX_BUF_SIZE	1
#endif

#define	OUR_INTR_BITS	\
	(INT_RX | INT_TX | INT_DESCERR | INT_DATAERR | INT_DESCPROT | \
	INT_RXDESCEMPTY | INT_RXOF | INT_TXUF | INT_MAC)


#ifdef GEM_CONFIG_TX_DIRECT
#define	MAXTXFRAGS	(min(8, GEM_MAXTXFRAGS))
#else
#define	MAXTXFRAGS	1
#endif
#define	MAXRXFRAGS	1

#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef	TX_RING_SIZE
#if MAXTXFRAGS == 1
#define	TX_RING_SIZE	TX_BUF_SIZE
#else
#define	TX_RING_SIZE	(TX_BUF_SIZE * 4)
#endif
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#if TX_RING_SIZE > 512
#error TX_RING_SIZE must be less than or equal to 512
#endif
#if RX_BUF_SIZE > 512
#error RX_BUF_SIZE must be less than or equal to 512
#endif

#ifdef DEBUG_COPY_PACKET
static int	bfe_tx_copy_thresh = INT32_MAX;
static int	bfe_rx_copy_thresh = INT32_MAX;
#else
static int	bfe_tx_copy_thresh = 256;
static int	bfe_rx_copy_thresh = 256;
#endif

/*
 * bcm440x chip state
 */
struct bfe_dev {
	/* HW/FW revision numbers */
	int		coreunit;
	uint_t		revid;

	/* rx control */
	int		rx_rest;
	int		rx_head;
	int		rx_tail;

	/* tx control */
	int		tx_head;
	int		tx_tail;

	/* interrupt control */
	uint32_t	intmask;
#ifdef CONFIG_POLLING
	int		last_poll_interval;
#endif

	/* reset control */
	boolean_t	need_to_reset;
	boolean_t	init_done;
	boolean_t	nic_active;

	/* MIB data */
	boolean_t	mibvalid;
	uint32_t	mib[MIB_SIZE];

	uint32_t	prom_data[EEPROM_SIZE];
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info bfe_chiptbl[] = {
	{0x14e4, 0x4401, "BCM4401"},
	{0x14e4, 0x4402, "BCM4402"},
	{0x14e4, 0x170c, "BCM4401B0"},
};
#define	CHIPTABLESIZE   (sizeof (bfe_chiptbl)/sizeof (struct chip_info))

/* ======================================================== */

/* mii operations */
static void  bfe_mii_sync(struct gem_dev *);
static uint16_t  bfe_mii_read(struct gem_dev *, uint_t);
static void bfe_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int bfe_attach_chip(struct gem_dev *);
static int bfe_reset_chip(struct gem_dev *);
static int bfe_init_chip(struct gem_dev *);
static int bfe_set_rx_filter(struct gem_dev *);
static int bfe_set_media(struct gem_dev *);
static int bfe_start_chip(struct gem_dev *);
static int bfe_stop_chip(struct gem_dev *);
static int bfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int bfe_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void bfe_tx_start(struct gem_dev *dp, int startslot, int nslot);
static void bfe_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void bfe_rx_start(struct gem_dev *dp, int startslot, int nslot);
static uint_t bfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t bfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void bfe_tx_desc_init(struct gem_dev *dp, int slot);
static void bfe_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t bfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr bfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, buffers should be native endianness */
static struct ddi_device_acc_attr bfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

#ifdef TEST_SMALL_DMA_RANGE
#undef	SB_PCIDMA_SIZE
#define	SB_PCIDMA_SIZE	(128*1024*1024)
#endif

static ddi_dma_attr_t bfe_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_addr_hi */
	0x1fff,			/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x1fff,			/* dma_attr_maxxfer */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t bfe_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	SB_PCIDMA_SIZE - 1, 	/* dma_attr_addr_hi */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_count_max */
	DESC_BASE_ALIGN,	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_maxxfer */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * debug routines
 */
/* ======================================================== */
#ifdef DEBUG_DUMP_REGS
#include "bfe_reg_dump.c"
#endif

/* ======================================================== */
/*
 * HW manipulation routines
 */
/* ======================================================== */
static int
bfe_reset_chip(struct gem_dev *dp)
{
	struct bfe_dev		*lp = dp->private;
	uint32_t		bar0win;
	uint32_t		tmp;
	ddi_acc_handle_t	conf_handle;
	uint32_t		val;
	uint32_t		phyctl;
	int			i;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#ifdef DEBUG_DUMP_REGS
	if (!lp->init_done) {
		bfe_dump_allregs(dp, "before reset");
	}
#endif
	lp->nic_active = B_FALSE;
	if (lp->init_done) {
		/* inhibit all interrupts before resetting nic */
		OUTL(dp, INTMASK, 0); FLUSH(dp, INTMASK);
		DPRINTF(2, (CE_CONT, "!%s: %s: sb_intmask:0x%x",
		    dp->name, __func__, INL(dp, SB_INTMASK)));
	}

#if DEBUG_LEVEL > 4
	if ((INL(dp, SB_TMSTATEL) &
	    (SB_TMSTATEL_RESET | SB_TMSTATEL_REJ | SB_TMSTATEL_CLK))
	    != SB_TMSTATEL_CLK) {
		/* the core isn't up now */
		cmn_err(CE_CONT, "!%s: %s: core isn't up", dp->name, __func__);
	}
#endif

	if ((INL(dp, SB_TMSTATEL) &
	    (SB_TMSTATEL_RESET | SB_TMSTATEL_REJ |
	    SB_TMSTATEL_CLK)) != SB_TMSTATEL_CLK ||
	    !lp->init_done) {

		if (pci_config_setup(dp->dip, &conf_handle) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s: %s: pci_config_setup failed",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}

		/*
		 * Change bar0window to map sbtopci registers
		 */
		bar0win = pci_config_get32(conf_handle, PCI_BAR0_WIN);
		DPRINTF(2, (CE_CONT, "!%s: %s: bar0win:0x%x",
		    dp->name, __func__, bar0win));
		pci_config_put32(conf_handle, PCI_BAR0_WIN, SB_PCI_BASE);

		/*
		 * Ensure if the map is correct.
		 */
		if (SB_IDH_CORE_PCI != (INL(dp, SB_IDH) & SB_IDH_CORE)) {
			cmn_err(CE_WARN,
			    "!%s: %s: failed to map sbtopci registers",
			    dp->name, __func__);
			/*
			 * XXX - we must restore bar0window mapping,
			 * otherwise the hardware will be hung.
			 */
			pci_config_put32(conf_handle, PCI_BAR0_WIN, bar0win);

			pci_config_teardown(&conf_handle);

			return (GEM_FAILURE);
		}

		val = INL(dp, SB_INTMASK) | SB_INTMASK_ENET0;
		OUTL(dp, SB_INTMASK, val);
#ifdef DEBUG_NOBURST
		val = INL(dp, SBTOPCI2) & ~(SBTOPCI_PREF | SBTOPCI_BURST);
#else
		val = INL(dp, SBTOPCI2) | SBTOPCI_PREF | SBTOPCI_BURST;
#endif
		OUTL(dp, SBTOPCI2, val);

		/* restore bar0window mapping */
		pci_config_put32(conf_handle, PCI_BAR0_WIN, bar0win);

		pci_config_teardown(&conf_handle);
	} else {
		/*
		 * the core have been up,
		 * we try to stop all activities gracefully.
		 */
		DPRINTF(0, (CE_CONT, "!%s: %s: core is up",
		    dp->name, __func__));

		/* do soft reset */
		OUTL(dp, ENETCTL, ENETCTL_SOFTRESET);
	}

	if ((INL(dp, SB_TMSTATEL) & SB_TMSTATEL_RESET) == 0) {

		DPRINTF(0, (CE_CONT, "!%s: %s: resetting the core",
		    dp->name, __func__));
		OUTL(dp, SB_TMSTATEL, SB_TMSTATEL_CLK | SB_TMSTATEL_REJ);

		for (i = 0;
		    (INL(dp, SB_TMSTATEL) & SB_TMSTATEL_REJ) == 0; i++) {
			if (i > 100) {
				cmn_err(CE_NOTE, "!%s: %s: timeout:"
				    " waiting for SB_TMSTATE_REJ",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}

		for (i = 0; INL(dp, SB_TMSTATEH) & SB_TMSTATEH_BUSY; i++) {
			if (i > 100) {
				cmn_err(CE_NOTE, "!%s: %s: timeout:"
				    " waiting for SB_TMSTATH_BUSY",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}

		OUTL(dp, SB_TMSTATEL,
		    SB_TMSTATEL_FGC | SB_TMSTATEL_CLK |
		    SB_TMSTATEL_REJ | SB_TMSTATEL_RESET);
		FLUSH(dp, SB_TMSTATEL);
		drv_usecwait(10);

		OUTL(dp, SB_TMSTATEL,
		    SB_TMSTATEL_REJ | SB_TMSTATEL_RESET);
		FLUSH(dp, SB_TMSTATEL);
		drv_usecwait(1);
#ifdef DEBUG_LEVEL
	} else {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: no need to reset the core",
		    dp->name, __func__));
#endif
	}

	OUTL(dp, SB_TMSTATEL,
	    SB_TMSTATEL_FGC | SB_TMSTATEL_CLK | SB_TMSTATEL_RESET);
	FLUSH(dp, SB_TMSTATEL);
	drv_usecwait(1);

	if (INL(dp, SB_TMSTATEH) & SB_TMSTATEH_SERR) {
		OUTL(dp, SB_TMSTATEH, 0);
		cmn_err(CE_WARN, "!%s: %s: SB_TMSTATEH_SERR still set",
		    dp->name, __func__);
		return (GEM_FAILURE);
	}

	if ((tmp = INL(dp, SB_IMSTATE)) & (SB_IMSTATE_IBE | SB_IMSTATE_TO)) {
		OUTL(dp, SB_IMSTATE, tmp & ~(SB_IMSTATE_IBE | SB_IMSTATE_TO));
		cmn_err(CE_WARN, "!%s: %s: sbimstate is not clear",
		    dp->name, __func__);
		return (GEM_FAILURE);
	}

	/* clear reset and allow it to propagate throughout the core */
	OUTL(dp, SB_TMSTATEL, SB_TMSTATEL_FGC | SB_TMSTATEL_CLK);
	FLUSH(dp, SB_TMSTATEL);
	drv_usecwait(1);

	/* clock must be enabled */
	OUTL(dp, SB_TMSTATEL, SB_TMSTATEL_CLK);
	FLUSH(dp, SB_TMSTATEL);
	drv_usecwait(1);

	/* setup PHY */
	OUTL(dp, MIICTL, MIICTL_PREAMBLE | 13);

	phyctl = INL(dp, PHYCTL);
	if ((phyctl & PHYCTL_INTERNAL) == 0) {
		if ((ENETCTL_EXTERNALPHY & INL(dp, ENETCTL)) == 0) {
			/* synchronize with mii_state */
			dp->mii_state = MII_STATE_RESETTING;
		}

		/* enable external phy */
		OUTL(dp, ENETCTL, ENETCTL_EXTERNALPHY);
	} else {
		if (phyctl & PHYCTL_RESET) {
			/* release reset signal for internal PHY */
			OUTL(dp, PHYCTL, phyctl & ~PHYCTL_RESET);
			drv_usecwait(100);

			/* synchronize with mii_state */
			dp->mii_state = MII_STATE_RESETTING;
		}
	}
#ifdef DEBUG_DUMP_REGS
	if (!lp->init_done) {
		bfe_dump_allregs(dp, "after reset");
	}
#endif
	lp->init_done = B_TRUE;

	/* mask unexpected interrupts */
	OUTL(dp, INTMASK, 0); FLUSH(dp, INTMASK);
	if (lp->intmask == 0) {
		OUTL(dp, INTSTAT, 0xffffffff); FLUSH(dp, INTSTAT);
	}

	/* workaround for resetting nic on reception of long packets */
	lp->rx_rest = 0;

	lp->tx_head = 0;
	lp->tx_tail = 0;
	lp->rx_head = 0;
	lp->rx_tail = 0;
	lp->nic_active = B_FALSE;

	return (GEM_SUCCESS);
}

#define	RX_TIMEOUT_10u	625		/* 10uS */
#define	RX_FRAMECNT	2

static int
bfe_init_chip(struct gem_dev *dp)
{
	struct bfe_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT((dp->rx_ring_dma & (DESC_BASE_ALIGN - 1)) == 0);
	ASSERT((dp->tx_ring_dma & (DESC_BASE_ALIGN - 1)) == 0);

	/* 000 PHYCTL: has been configured */

	/* 00c BISTSTAT: ignore */

	/* 010 WAKEUPLEN: ignore */

	/* 020 INTSTAT: ignore */

	/* 024 INTMASK: clear */
	OUTL(dp, INTMASK, 0);
	lp->intmask = 0;

	/* 028 GPTIMER: clear */
	OUTL(dp, GPTIMER, 0);
#if DEBUG_LEVEL > 10
{
	clock_t	now;

	OUTL(dp, GPTIMER, 100*1000*1000);
	now = ddi_get_lbolt();
	while ((INL(dp, INTSTAT) & INT_GPTIMER) == 0) {
		drv_usecwait(10);
	}

	cmn_err(CE_CONT, "!%s: %s: %d mS",
	    dp->name, __func__,  (ddi_get_lbolt() - now)*10);

	OUTL(dp, GPTIMER, 0);
}
#endif

	/* 0a8 MACCTL enable crc generation */
	OUTL(dp, MACCTL, MACCTL_EN_LED | MACCTL_EN_CRC);

	/* 100 RXINTDELAY: */
	OUTL(dp, RXINTDELAY, (1 << RXINTDELAY_FRAMECNT_SHIFT));


	/* 400 RXCFG: don't touch */

	/* 404 RXMAXLEN: */
	OUTL(dp, RXMAXLEN, dp->mtu +
	    sizeof (struct ether_header) + ETHERFCSL + RX_HEAD_ROOM + 4);

	/* 408 TXMAXLEN: */
	OUTL(dp, TXMAXLEN, dp->mtu + sizeof (struct ether_header) + 4 + 32);

	/* 418 MACINTMASK: don't touch */

	/* 42c ENETCTL: don't touch */

	/* 430 TXCFG: don't touch */

	/* 434 TXHIWAT: must be 56 */
#if DEBUG_LEVEL > 10
	OUTL(dp, TXHIWAT, 1);
	DPRINTF(0, (CE_CONT, "%s: %s: txhiwat: %x",
	    dp->name, __func__, INL(dp, TXHIWAT)));
#else
	OUTL(dp, TXHIWAT, min(56, 0x3f));	/* default 56 */
#endif
	/* 438 MIBCTL */
	OUTL(dp, MIBCTL, MIBCTL_AUTOCLR);

	/* 500 mibdata: clear by reading all mib counter registers */
	if (!lp->mibvalid) {
		ddi_rep_get32(dp->regs_handle, lp->mib,
			(uint32_t *)(((long)dp->base_addr) + MIBDATA),
			MIB_SIZE, DDI_DEV_AUTOINCR);
		/* discard mib data */
		bzero(lp->mib, sizeof (lp->mib));
		lp->mibvalid = B_TRUE;
	}

	/*
	 * finally, we setup dma engine.
	 */
	/* for tx dma channel */
#ifdef TEST_BURST
	OUTL(dp, TXMAXBURSTLEN, 0xffffffff);
	OUTL(dp, RXMAXBURSTLEN, 0xffffffff);
	DPRINTF(0, (CE_CONT, "%s: %s: txmaxburst:%x, rxmaxburst:%x",
	    dp->name, __func__,
	    INL(dp, TXMAXBURSTLEN), INL(dp, RXMAXBURSTLEN)));
#endif
#ifdef notdef
	OUTL(dp, TXMAXBURSTLEN, min(0x80, dp->txmaxdma/4));
	OUTL(dp, RXMAXBURSTLEN, min(0x80, dp->rxmaxdma/4));
#endif
	DPRINTF(1, (CE_CONT, "%s: %s: txmaxburst:%x, rxmaxburst:%x",
	    dp->name, __func__,
	    INL(dp, TXMAXBURSTLEN), INL(dp, RXMAXBURSTLEN)));
#ifdef TEST_FAIRSCHED
	OUTL(dp, TXCTL, TXCTL_FPRI | TXCTL_EN);
#else
	OUTL(dp, TXCTL, TXCTL_EN);
#endif
	OUTL(dp, TXDESCBASE, dp->tx_ring_dma | SB_PCIDMA_BASE);

	/* for rx dma channel */
	OUTL(dp, RXCTL,
	    (RX_HEAD_ROOM << RXCTL_OFFSET_SHIFT) | RXCTL_EN);
	OUTL(dp, RXDESCBASE, dp->rx_ring_dma | SB_PCIDMA_BASE);

	return (GEM_SUCCESS);
}

#ifdef DEBUG_LEVEL
static int
bfe_dump_cam(struct gem_dev *dp)
{
	int		i;
	int		ix;
	uint32_t	hi;
	uint32_t	lo;

	for (ix = 0; ix < 64; ix++) {
		OUTL(dp, CAMCTL, (ix << CAMCTL_INDEX_SHIFT) | CAMCTL_READ);

		for (i = 0; INL(dp, CAMCTL) & CAMCTL_BUSY; i++) {
			if (i > 10) {
				cmn_err(CE_WARN,
				    "!%s: dump_cam: timeout on reading cam",
				    dp->name);
				return (GEM_FAILURE);
			}
			drv_usecwait(10);
		}
		hi = INL(dp, CAMHI);
		lo = INL(dp, CAMLOW);
		cmn_err(CE_CONT, "!%s: %s: (%d) %02x:%02x:%02x:%02x:%02x:%02x",
		    dp->name, __func__, ix,
		    (hi >> 8)  & 0xff, (hi >> 0)  & 0xff,
		    (lo >> 24) & 0xff, (lo >> 16) & 0xff,
		    (lo >> 8)  & 0xff, (lo >> 0)  & 0xff);
	}
	return (GEM_SUCCESS);
}
#endif

static int
bfe_load_addr_into_cam(struct gem_dev *dp, uint8_t *mac, uint_t ix)
{
	int		i;

	DPRINTF(3, (CE_CONT,
	    "!%s: %s: (%d) %02x:%02x:%02x:%02x:%02x:%02x",
	    dp->name, __func__, ix,
	    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));

	OUTL(dp, CAMLOW,
	    (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5]);
	OUTL(dp, CAMHI, CAMHI_VALID | (mac[0] << 8) | mac[1]);

	OUTL(dp, CAMCTL, (ix << CAMCTL_INDEX_SHIFT) | CAMCTL_WRITE);

	for (i = 0; INL(dp, CAMCTL) & CAMCTL_BUSY; i++) {
		if (i > 10) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout on writing to cam",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}
	return (GEM_SUCCESS);
}

static int
bfe_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	rxcfg;
	int		i;
	static uint8_t	mac_invalid[ETHERADDRL] = {0, 0, 0, 0, 0, 0};

	DPRINTF(0, (CE_CONT, "!%s: %s: called, mc_count:%d, mode:0x%b",
	    dp->name, __func__, dp->mc_count, dp->rxmode, RXMODE_BITS));
#if DEBUG_LEVEL > 0
	for (i = 0; i < dp->mc_count; i++) {
		cmn_err(CE_CONT,
		    "!%s: adding mcast(%d) %02x:%02x:%02x:%02x:%02x:%02x",
		    dp->name, i,
		    dp->mc_list[i].addr.ether_addr_octet[0],
		    dp->mc_list[i].addr.ether_addr_octet[1],
		    dp->mc_list[i].addr.ether_addr_octet[2],
		    dp->mc_list[i].addr.ether_addr_octet[3],
		    dp->mc_list[i].addr.ether_addr_octet[4],
		    dp->mc_list[i].addr.ether_addr_octet[5]);
	}
#endif
	rxcfg = INL(dp, RXCFG) & ~(RXCFG_PROMISC | RXCFG_ALLMULT);

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* disable the cam */
		OUTL(dp, CAMCTL, 0);
	} else if (dp->rxmode & RXMODE_PROMISC) {
		rxcfg |= RXCFG_PROMISC;
	} else {
		if (dp->mac_active) {
			/*
			 * XXX - receive all packets for a while
			 * to avoid dropping packets which must be received.
			 */
			OUTL(dp, RXCFG, rxcfg | RXCFG_PROMISC | RXCFG_ALLMULT);
			FLUSH(dp, RXCFG);
		}

		/* disable the cam for writing */
		OUTL(dp, CAMCTL, 0);
		FLUSH(dp, CAMCTL);

		/* reset cam index counter */
		i = 0;

		/* next, append multicast addresses */
		if ((dp->rxmode & RXMODE_ALLMULTI) ||
		    /* XXX - we must reserve one for our station address */
		    dp->mc_count > CAM_ENTRY_SIZE - 1) {
			rxcfg |= RXCFG_ALLMULT;
		} else {
			for (; i < dp->mc_count; i++) {
				(void) bfe_load_addr_into_cam(dp,
				    dp->mc_list[i].addr.ether_addr_octet, i);
			}
		}

		/* XXX - invalidate unused CAM entries. 2004/12/19 */
		for (; i < CAM_ENTRY_SIZE - 1; i++) {
			(void) bfe_load_addr_into_cam(dp, mac_invalid, i);
		}

		/* finally,  we add our station address into the cam */
		(void) bfe_load_addr_into_cam(dp,
		    dp->cur_addr.ether_addr_octet, i);

#if DEBUG_LEVEL > 3
		bfe_dump_cam(dp);
#endif
		/* finally enable the cam */
		OUTL(dp, CAMCTL, INL(dp, CAMCTL) | CAMCTL_EN);
		FLUSH(dp, CAMCTL);
	}
	OUTL(dp, RXCFG, rxcfg);
	FLUSH(dp, RXCFG);

	return (GEM_SUCCESS);
}

static int
bfe_set_media(struct gem_dev *dp)
{
	uint32_t	rx;
	uint32_t	tx;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(dp->nic_state == NIC_STATE_INITIALIZED ||
	    dp->nic_state == NIC_STATE_ONLINE);

	/*
	 * Notify current duplex mode to mac
	 */
	tx = INL(dp, TXCFG) & ~TXCFG_FULLDPX;
	if (dp->full_duplex) {
		tx |= TXCFG_FULLDPX;
	}
	OUTL(dp, TXCFG, tx);

	/*
	 * set up flow control
	 */
	rx = INL(dp, RXCFG) & ~RXCFG_EN_FLOWCTL;
	tx = INL(dp, FLOWCTL) & ~(FLOWCTL_ENABLE | FLOWCTL_RXHIWAT);
	switch (dp->flow_control) {
	case FLOW_CONTROL_RX_PAUSE:
		rx |= RXCFG_EN_FLOWCTL;
		break;

	case FLOW_CONTROL_TX_PAUSE:
#define	RXFIFO_HI	((256/8)*6)
		tx |= FLOWCTL_ENABLE | (RXFIFO_HI & FLOWCTL_RXHIWAT);
		break;

	case FLOW_CONTROL_SYMMETRIC:
		rx |= RXCFG_EN_FLOWCTL;
		tx |= FLOWCTL_ENABLE | (RXFIFO_HI & FLOWCTL_RXHIWAT);
		break;
#undef	RXFIFO_HI
	}
	OUTL(dp, RXCFG, rx);
	OUTL(dp, FLOWCTL, tx);

	return (GEM_SUCCESS);
}

static int
bfe_start_chip(struct gem_dev *dp)
{
	struct bfe_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Kick TX and RX */
	lp->nic_active = B_TRUE;
	OUTL(dp, ENETCTL, INL(dp, ENETCTL) | ENETCTL_ENABLE);

	/* enable interrupt */
	lp->intmask = OUR_INTR_BITS;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, INTMASK, lp->intmask); FLUSH(dp, INTMASK);
#ifdef DEBUG_FLOW
		OUTL(dp, MACINTMASK, MACINT_FLOW);
#endif
	}

	return (GEM_SUCCESS);
}

static int
bfe_stop_chip(struct gem_dev *dp)
{
	int		i;
	struct bfe_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Disable interrupts by clearing the interrupt mask */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, INTMASK, 0); FLUSH(dp, INTMASK);
		OUTL(dp, MACINTMASK, 0);
	}

	/* inhibit rx interrupt */
	OUTL(dp, RXINTDELAY, 0);

	/* stop tx */
	/* XXX - last chance to get correct tx_head */
	lp->tx_head = TXSTAT_DESC_INDEX(dp);
	OUTL(dp, TXCTL, 0);

	/* wait until tx and rx finish */
	lp->nic_active = B_FALSE;
	OUTL(dp, ENETCTL, ENETCTL_DISABLE);
	for (i = 0; INL(dp, ENETCTL) & ENETCTL_DISABLE; i++) {
		if (i > 100) {
			cmn_err(CE_NOTE,
			    "!%s: %s: timeout: stopping mac",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}

	/* wait until rx dma status become idle */
	if ((INL(dp, RXSTAT) & RXSTAT_ERR) == RXSTAT_ERR_NONE) {
		DPRINTF(0, (CE_CONT, "!%s: %s: rxstat_err_none",
		    dp->name, __func__));
		i = 0;
		while ((INL(dp, RXSTAT) & RXSTAT_STATE)
		    == RXSTAT_STATE_ACTIVE) {
			if (i++ > 200) {
				/*
				 * XXX - it seems that rx state is
				 * always active.
				 */
				DPRINTF(1, (CE_CONT,
				    "!%s: %s: timeout: stopping rx dma",
				    dp->name, __func__));
				break;
			}
			drv_usecwait(10);
		}
	}

	DPRINTF(0, (CE_CONT, "%s: %s: rxstat:%x",
	    dp->name, __func__, INL(dp, RXSTAT)));

	/* XXX - last chance to get valid rx_head */
	lp->rx_head = RXSTAT_DESC_INDEX(dp);
	/* stop rx dma engine */
	OUTL(dp, RXCTL, 0);

	if (lp->mibvalid) {
		/* read mib counters before resetting the chip */
		(void) bfe_get_stats(dp);
	}

	return (GEM_SUCCESS);
}

static void
bfe_eeprom_dump(struct gem_dev *dp, uint32_t *prom)
{
	int		i;

	cmn_err(CE_CONT, "!%s: %s:", dp->name, __func__);
	for (i = 0; i < EEPROM_SIZE; i += 4) {
		cmn_err(CE_CONT, "!0x%02x(%d): 0x%08x 0x%08x 0x%08x 0x%08x",
		    i*4, i*4, prom[i], prom[i+1], prom[i+2], prom[i+3]);
	}
}

static int
bfe_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint8_t		*mac;
	struct bfe_dev	*lp = dp->private;
	uint8_t		*prom = (uint8_t *)lp->prom_data;

	for (i = 0; i < EEPROM_SIZE; i++) {
		lp->prom_data[i] = INL(dp, EEPROM_BASE + i*sizeof (uint32_t));
	}
#if DEBUG_LEVEL > 0
	bfe_eeprom_dump(dp, lp->prom_data);
#endif
	if (prom[EEPROM_MAGIC] != 0x01 &&
	    prom[EEPROM_MAGIC] != 0x10) {
		cmn_err(CE_WARN,
		    "!%s: %s: contents in eeprom is corrupted",
		    dp->name, __func__);
		bfe_eeprom_dump(dp, lp->prom_data);
		return (GEM_FAILURE);
	}

	mac = dp->dev_addr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL; i += 2) {
		/* XXX - the core is running in big endian */
		*mac++ = prom[EEPROM_NODEADDR + i + 1];
		*mac++ = prom[EEPROM_NODEADDR + i];
	}

	/* get phy address from eeprom */
	dp->mii_phy_addr = prom[EEPROM_PHYADDR] & 0x1f;

	/* fix rx buffer length, it must have additional 4 byte and rx_header */
	dp->rx_buf_len = ETHERMTU + sizeof (struct ether_header) +
	    ETHERFCSL + 4 + RX_HEAD_ROOM;

	DPRINTF(2, (CE_CONT, "!%s: %s:"
	    " phy_addr:%d, macintmask:0x%x,"
	    " txcfg:0x%x, txmaxburst:0x%x,"
	    " rxmaxburst:0x%x, txhiwat:0x%x",
	    dp->name, __func__,
	    dp->mii_phy_addr,
	    INL(dp, MACINTMASK), INL(dp, TXCFG),
	    INL(dp, TXMAXBURSTLEN), INL(dp, RXMAXBURSTLEN),
	    INL(dp, TXHIWAT)));

	/* mib counters are not initialized yet. */
	lp->mibvalid = B_FALSE;

#ifdef CONFIG_POLLING
#ifdef CONFIG_NEW_POLLING
	/* EMPTY */
#else
	dp->misc_flag |= GEM_POLL_RXONLY;
#endif
#endif
#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	return (GEM_SUCCESS);
}


static int
bfe_get_stats(struct gem_dev *dp)
{
	int	txerr;
	int	rxerr;
	int	x;
	struct bfe_dev	*lp = dp->private;

	if (!lp->mibvalid) {
		return (GEM_SUCCESS);
	}

	ddi_rep_get32(dp->regs_handle, lp->mib,
	    (void *)(((caddr_t)dp->base_addr) + MIBDATA),
	    MIB_SIZE, DDI_DEV_AUTOINCR);

	txerr = 0;
	rxerr = 0;

	dp->stats.collisions	+= lp->mib[0x40/4];
	dp->stats.first_coll	+= lp->mib[0x44/4];
	dp->stats.multi_coll	+= lp->mib[0x48/4];
	dp->stats.excoll	+= x = lp->mib[0x4c/4]; txerr += x;
	/*
	 * XXX - no carrier counter seems to be broken
	 * dp->stats.nocarrier	+= x = lp->mib[0x58/4]; txerr += x;
	 */
	dp->stats.defer		+= lp->mib[0x54/4];
	dp->stats.underflow	+= x = lp->mib[0x3c/4]; txerr += x;
	dp->stats.xmtlatecoll	+= x = lp->mib[0x50/4]; txerr += x;

	dp->stats.frame_too_long += x = lp->mib[0xb4/4]; rxerr += x;
	dp->stats.missed	+= x = lp->mib[0xbc/4]; rxerr += x;
	dp->stats.runt		+= x = lp->mib[0xc4/4]; rxerr += x;
	dp->stats.crc		+= x = lp->mib[0xc8/4]; rxerr += x;
	dp->stats.frame	+= x = lp->mib[0xcc/4] + lp->mib[0xd0/4];
	rxerr += x;
	dp->stats.errxmt += txerr;
	dp->stats.errrcv += rxerr;

	return (GEM_SUCCESS);
}

/*
 * descriptor manipulation
 */
static int
bfe_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
#ifdef GEM_CONFIG_TX_DIRECT
	int			i;
	int			ix;
#endif
	struct bfe_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	uint32_t		addr;
	uint32_t		tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d flags: %ld",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
	cmn_err(CE_CONT, "!          isr: %b", INL(dp, INTSTAT), INT_BITS);
}
#endif
#if DEBUG_LEVEL > 2
	if ((slot & 7) == 0) {
		flag |= GEM_TXFLAG_INTR;
	}
#endif
	/*
	 * write tx descriptor(s)
	 */
	mark = CTL_EOF;
#if defined(CONFIG_POLLING) && defined(CONFIG_NEW_POLLING)
	mark |= CTL_IOC;
#else
	if (flag & GEM_TXFLAG_INTR) {
		mark |= CTL_IOC;
	}
#endif
	dcp = &dmacookie[frags - 1];

#ifdef GEM_CONFIG_TX_DIRECT
	for (i = frags - 1; i > 0; i--, dcp--) {
		ix = SLOT(slot + i, tx_ring_size);
		tdp = &BFE_DESC(dp->tx_ring)[ix];

		if (ix == tx_ring_size - 1) {
			mark |= CTL_EOT;
		}
		mark |= dcp->dmac_size;
		addr = dcp->dmac_address | SB_PCIDMA_BASE;
		tdp->desc_databufptr = LE_32(addr);
		tdp->desc_ctl = LE_32(mark);
		mark = 0;
	}
#endif
	/* make first fragment */
	tdp = &BFE_DESC(dp->tx_ring)[slot];
	mark |= (dcp->dmac_size | CTL_SOF);
	if (slot == tx_ring_size - 1) {
		mark |= CTL_EOT;
	}
	addr = dcp->dmac_address | SB_PCIDMA_BASE;
	tdp->desc_databufptr = LE_32(addr);
	tdp->desc_ctl = LE_32(mark);

	return (frags);
}

static void
bfe_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct bfe_dev	*lp = dp->private;

	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	/* set new tail */
	lp->tx_tail = SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size);
	OUTL(dp, TXTAIL, lp->tx_tail * sizeof (struct bfe_desc));
}

static void
bfe_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t	ctl;
	uint32_t	addr;
	int		rx_ring_size = dp->gc.gc_rx_ring_size;
	struct bfe_desc	*rdp;
	struct bfe_dev	*lp = dp->private;

#if DEBUG_LEVEL > 2
{
	int		i;

	cmn_err(CE_CONT,
	    "!%s: bfe_rx_desc_write seqnum: %d, slot %d, frags: %d",
	    dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x(%d)",
		    i, dmacookie[i].dmac_laddress,
		    dmacookie[i].dmac_size, dmacookie[i].dmac_size);
	}
}
#endif
	ASSERT(frags == 1);

	/*
	 * write a RX descriptor
	 */
	ctl = (dmacookie->dmac_size & CTL_BUFCOUNT);
	if (slot == rx_ring_size - 1) {
		ctl |= CTL_EOT;
	}
	rdp = &BFE_DESC(dp->rx_ring)[slot];

	addr = dmacookie->dmac_address | SB_PCIDMA_BASE;
	rdp->desc_databufptr = LE_32(addr);
	rdp->desc_ctl = LE_32(ctl);

	/* clear rx header area to check reception of big packets */
	bzero(dp->rx_buf_tail->rxb_buf, RX_HEAD_ROOM);
	(void) ddi_dma_sync(dp->rx_buf_tail->rxb_dh,
	    0, RX_HEAD_ROOM, DDI_DMA_SYNC_FORDEV);

	lp->rx_tail = SLOT(slot + frags, rx_ring_size);
}

static void
bfe_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct bfe_dev	*lp = dp->private;

	gem_rx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	/* set new tail */
	OUTL(dp, RXTAIL, lp->rx_tail * sizeof (struct bfe_desc));
}

#ifdef DEBUG_LEVEL
static void
bfe_tx_desc_dump(struct gem_dev *dp, seqnum_t head, int ndesc)
{
	int		i;
	uint32_t	tx_ring_size = dp->gc.gc_tx_ring_size;
	struct bfe_desc	*tdp;

	cmn_err(CE_CONT, "!%s: tx_desc_dump:  slot:%d, seqnum:%d",
	    dp->name, SLOT(head, tx_ring_size), head);

	for (i = 0; i < ndesc; i++) {
		tdp = &BFE_DESC(dp->tx_ring)[SLOT(head + i, tx_ring_size)];
		(void) ddi_dma_sync(dp->desc_dma_handle,
		    (off_t)(((caddr_t)tdp) - dp->rx_ring),
		    sizeof (struct bfe_desc), DDI_DMA_SYNC_FORKERNEL);
		cmn_err(CE_CONT, "! %d: ctrl:%b, addr:0x%08x",
		    i, LE_32(tdp->desc_ctl), CTL_BITS,
		    LE_32(tdp->desc_databufptr));
	}
}
#endif

#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
bfe_tx_desc_head(struct gem_dev *dp)
{
	struct bfe_dev	*lp = dp->private;

	if (lp->nic_active) {
		lp->tx_head = TXSTAT_DESC_INDEX(dp);
	}
	return (lp->tx_head);
}
#else
static uint_t
bfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct bfe_dev	*lp = dp->private;

	if (lp->nic_active) {
		lp->tx_head = TXSTAT_DESC_INDEX(dp);
	}
	slot = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(slot, lp->tx_head, lp->tx_tail)) {
		/* not yet */
		return (0);
	}
	return (GEM_TX_DONE);
}
#endif

#ifdef DEBUG_LEVEL
static void
bfe_packet_dump(struct gem_dev *dp, uint8_t *bp)
{
	cmn_err(CE_CONT, "!%s: "
	    "%02x %02x %02x %02x %02x %02x %02x %02x "
	    "%02x %02x %02x %02x %02x %02x",
	    dp->name,
	    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7],
	    bp[8], bp[9], bp[10], bp[11], bp[12], bp[13]);
}
#endif
static uint64_t
bfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rxbuf		*rbp = dp->rx_buf_head;
	volatile uint8_t	*hdp;
	uint16_t		rxf;
	uint_t			len;
	int			i;
	uint_t			ret;
	struct bfe_dev		*lp = dp->private;

	/* compare the slot with current rx slot index */
	if (lp->nic_active) {
		lp->rx_head = RXSTAT_DESC_INDEX(dp);
	}
	i = SLOT(slot + ndesc - 1, dp->gc.gc_rx_ring_size);
	if (INSIDE(i, lp->rx_head, lp->rx_tail)) {
		/* not yet */
		return (0);
	}

	if (lp->rx_rest > 0) {
		/*
		 * Ignore this fragment as it is a part of the previous
		 * jumbo packet.
		 */
		lp->rx_rest -= min(rbp->rxb_buf_len, lp->rx_rest);
		return (GEM_RX_ERR);
	}

	/*
	 * Get rx header which is leading received data area.
	 */
	hdp = (volatile uint8_t *)rbp->rxb_buf;

	/*
	 * Need to wait until the length field in the rx header at
	 * the beginning of the receive buffer is updated in case of
	 * the received packet exceeds the length of the buffer.
	 */
	for (i = 0; i < 100; i++) {
		/* sync rx header area in the rx buffer */
		(void) ddi_dma_sync(rbp->rxb_dh,
		    0, RX_HEAD_ROOM, DDI_DMA_SYNC_FORKERNEL);

		len = (hdp[RXHD_LEN + 1] << 8) | hdp[RXHD_LEN];
		rxf = (hdp[RXHD_RXF + 1] << 8) | hdp[RXHD_RXF];
		if (len) {
			break;
		}
		DPRINTF(3, (CE_CONT,
		    "!%s: %s: length field was not updated yet",
		    dp->name, __func__));
		drv_usecwait(10);
	}

	if (len < ETHERMIN + ETHERFCSL) {
		dp->stats.errrcv++;
		dp->stats.runt++;
		cmn_err(CE_CONT,
		    "!%s: %s: runt error: slot:%d len:%d rxf 0x%b",
		    dp->name, __func__, slot, len, rxf, RXF_BITS);
		return (GEM_RX_ERR);
	}

	ret = GEM_RX_DONE;

	if (rxf & RXF_ERRORS) {
		/* errored packet */
#ifdef notdef
		if (rxf & RXF_NIBBLEODD) {
			/* The nic seems to stop */
			lp->need_to_reset = B_TRUE;
		}
#endif
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: rx error: slot:%d len:%d rxf 0x%b",
		    dp->name, __func__, slot, len, rxf, RXF_BITS));
		ret |= GEM_RX_ERR;
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d len:%d rxf 0x%b",
	    dp->name, __func__, slot, len, rxf, RXF_BITS));
#if DEBUG_LEVEL > 2
	bfe_packet_dump(dp, (uint8_t *)hdp);
#endif

	/* Is the packet too big ? */
	if (len > rbp->rxb_buf_len - RX_HEAD_ROOM) {
		lp->rx_rest = len - (rbp->rxb_buf_len - RX_HEAD_ROOM);
		dp->stats.errrcv++;
		dp->stats.frame_too_long++;
		cmn_err(CE_CONT,
		    "!%s: %s: too_long error: slot:%d len:%d rxf 0x%b",
		    dp->name, __func__, slot, len, rxf, RXF_BITS);
		ret |= GEM_RX_ERR;
	}

	return (ret | ((len - ETHERFCSL) & GEM_RX_LEN));
}

static void
bfe_tx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	ctl;

	/* invalidate the descriptor */
	ctl = (slot == dp->gc.gc_tx_ring_size - 1) ? CTL_EOT : 0;
	BFE_DESC(dp->tx_ring)[slot].desc_ctl = LE_32(ctl);
}

static void
bfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	ctl;

	/* invalidate the descriptor */
	ctl = (slot == dp->gc.gc_rx_ring_size - 1) ? CTL_EOT : 0;
	BFE_DESC(dp->rx_ring)[slot].desc_ctl = LE_32(ctl);
}

/*
 * Device depend interrupt handler
 */
static uint_t
bfe_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flag = 0;
	struct bfe_dev	*lp = dp->private;

	isr = INL(dp, INTSTAT);

	if ((isr & lp->intmask) == 0) {
		/* Not for us */
		if (isr) {
			OUTL(dp, INTSTAT, isr); FLUSH(dp, INTSTAT);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: %s, isr: 0x%b",
	    dp->name, __func__, isr, INT_BITS));

	if (!dp->mac_active) {
		/* the device is not active */
		if (isr) {
			OUTL(dp, INTSTAT, isr); FLUSH(dp, INTSTAT);
		}
		/* inhibit interrupts gracefully */
		lp->intmask = 0;
		OUTL(dp, INTMASK, lp->intmask); FLUSH(dp, INTMASK);
		return (DDI_INTR_CLAIMED);
	}

	OUTL(dp, INTSTAT, isr);

#ifdef CONFIG_POLLING
#ifdef CONFIG_NEW_POLLING
	isr &= lp->intmask | INT_RX | INT_TX;
#else
	isr &= lp->intmask | INT_RX;
#endif
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval) {
			/* polling mode */
#ifdef CONFIG_NEW_POLLING
			lp->intmask &= ~(INT_RX | INT_TX);
#else
			lp->intmask &= ~INT_RX;
#endif
			lp->intmask |= INT_GPTIMER;
		} else {
			/* normal mode */
#ifdef CONFIG_NEW_POLLING
			lp->intmask |= INT_RX | INT_TX;
#else
			lp->intmask |= INT_RX;
#endif
			lp->intmask &= ~INT_GPTIMER;
		}
		OUTL(dp, GPTIMER, dp->poll_interval/(10000 / RX_TIMEOUT_10u));
		lp->last_poll_interval = dp->poll_interval;

		if ((dp->misc_flag & GEM_NOINTR) == 0) {
			/*
			 * XXX - it seems pended interrupts are cleared
			 * on changing INTMSK register
			 */
			isr |= INT_RX | INT_TX;
			OUTL(dp, INTMASK, lp->intmask);
		}
	}
#else
	isr &= lp->intmask;
#endif /* CONFIG_POLLING */

	/* a barrier to commit the interrupts state to be cleared */
	FLUSH(dp, INTSTAT);

	if (isr & (INT_RX | INT_RXDESCEMPTY)) {
		(void) gem_receive(dp);

		if (isr & INT_RXDESCEMPTY) {
			/* RX descriptor is empty */
			DPRINTF(2, (CE_CONT,
			    "!%s: %s: rxctl:%x rxtail:%x rxstat:%x",
			    dp->name, __func__,
			    INL(dp, RXCTL), INL(dp, RXTAIL),
			    INL(dp, RXSTAT)));
			dp->stats.norcvbuf++;
		}
	}

	if (isr & INT_TX) {
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

	if (isr & INT_MAC) {
		uint32_t	macint;

		macint = INL(dp, MACINTSTAT);
		cmn_err(CE_NOTE, "!%s: mac interrupt: macint:0x%b",
		    dp->name, macint, MACINT_BITS);
		OUTL(dp, MACINTSTAT, macint);
	}

	if (isr & (INT_DESCERR | INT_DATAERR |
	    INT_DESCPROT | INT_RXOF | INT_TXUF)) {
		cmn_err(CE_NOTE, "!%s: unexpected interrupt: isr:%b",
		    dp->name, isr, INT_BITS);

		if (isr & INT_RXOF) {
			dp->stats.errrcv++;
			dp->stats.overflow++;
		}
		if (isr & INT_TXUF) {
			dp->stats.errxmt++;
			dp->stats.underflow++;
		}

		lp->need_to_reset = B_TRUE;
	}

	if (lp->need_to_reset) {
		cmn_err(CE_NOTE, "!%s: %s: isr:%b, resetting the chip ...",
		    dp->name, __func__, isr, INT_BITS);

		(void) gem_restart_nic(dp,
		    GEM_RESTART_KEEP_BUF | GEM_RESTART_NOWAIT);

		flag |= INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routine
 */
static int
bfe_mii_config(struct gem_dev *dp)
{
	uint16_t	val;

	if (gem_mii_config_default(dp) != GEM_SUCCESS) {
		return (GEM_FAILURE);
	}

	/* enable LED */
	val = gem_mii_read(dp, 0x1a);
	gem_mii_write(dp, 0x1a, val & ~0x8000);

	val = gem_mii_read(dp, 0x1b);
	gem_mii_write(dp, 0x1b, val & 0x0040);

	return (GEM_SUCCESS);
}

static void
bfe_mii_sync(struct gem_dev *dp)
{
	/* nothing */
}

static uint16_t
bfe_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint16_t	val;

	OUTL(dp, MACINTSTAT, MACINT_MII);

	if (INL(dp, MACINTSTAT) & MACINT_MII) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to clear MII bit in MACINTSTAT register",
		    dp->name, __func__);
		return (0);
	}

	OUTL(dp, MIIDATA, MII_READ_CMD(dp->mii_phy_addr, reg));

	/* wait until done */
	for (i = 0; (INL(dp, MACINTSTAT) & MACINT_MII) == 0; i++) {
		if (i > 100) {
			val = INL(dp, MIIDATA);
			cmn_err(CE_WARN, "!%s: %s: timeout: 0x%b",
			    dp->name, __func__, val, MII_STATUS_BITS);
			return (0);
		}
		drv_usecwait(10);
	}
	val = INL(dp, MIIDATA);

	DPRINTF(4, (CE_CONT, "!%s: %s: done: 0x%04x", dp->name, __func__, val));

	return (val);
}

static void
bfe_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int	i;

	OUTL(dp, MACINTSTAT, MACINT_MII);

	if (INL(dp, MACINTSTAT) & MACINT_MII) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to clear MII bit in MACINTSTAT register",
		    dp->name, __func__);
		return;
	}

	OUTL(dp, MIIDATA, MII_WRITE_CMD(dp->mii_phy_addr, reg, val));

	for (i = 0; (INL(dp, MACINTSTAT) & MACINT_MII) == 0; i++) {
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
bfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			ret;
	ddi_acc_handle_t	conf_handle;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			revid;
#ifdef DEBUG_LEVEL
	uint32_t		ilr;
	uint8_t			cacheline;
	uint8_t			latencytimer;
#endif
	int			unit;
	struct chip_info	*p;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct bfe_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: %s: called %s",
	    drv_name, unit, __func__, ident));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

	/*
	 * XXX - fix command register in pci config space.
	 * some versions of bios in DELL Inspiron 5160 don't set
	 * MAE bit. Thank to Jamie for reporting and testing.
	 */
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);

#ifdef DEBUG_LEVEL
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);

	/* XXX - BCM440x doesn't have cache line register */
	cacheline = pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ);
	latencytimer = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
	if (bfe_debug > 0) {
		cmn_err(CE_CONT,
		    "!%s%d: ilr: 0x%08x, cache line size: 0x%02x,"
		    " latency timer: 0x%02x, stat: 0x%04x",
		    drv_name, unit, ilr, cacheline, latencytimer,
		    pci_config_get16(conf_handle, PCI_CONF_STAT));
	}
#endif /* DEBUG_LEVEL */
	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		/* force to bring up the nic core after power up */
		lp->init_done = B_FALSE;
		return (gem_resume(dip));

	case DDI_ATTACH:
		for (i = 0, p = bfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
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
		    "!%s%d: %s: unknown PCI venid/devid (0x%x, 0x%x)",
		    drv_name, unit, __func__, vid, did);
		/* fall down */
chip_found:
		/*
		 * Map in the device registers.
		 */
		ret = gem_pci_regs_map_setup(dip,
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
		    &bfe_dev_attr, (void *)&base, &regs_ha);
		if (ret != DDI_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s%d: gem_pci_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configuration
		 */
		gcp = kmem_zalloc(sizeof (struct gem_conf), KM_SLEEP);

		/* name */
		(void) sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;

		gcp->gc_tx_desc_unit_shift = 3;	/* 8byte */
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = TX_RING_SIZE - 1;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = bfe_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 3;
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_header_len = RX_HEAD_ROOM;
		gcp->gc_rx_copy_thresh = bfe_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = bfe_dev_attr;
		gcp->gc_buf_attr = bfe_buf_attr;
		gcp->gc_desc_attr = bfe_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = bfe_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = bfe_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = bfe_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
#ifdef TEST_RX_OVERFLOW
		/* XXX - test for rx overflow */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
#else
		/*
		 * XXX - don't enable flow control. it seems to cause
		 * tx performance problem.
		 */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;
#endif
		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = GEM_LINK_WATCH_INTERVAL/10;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* workaround for PHY */
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &bfe_attach_chip;
		gcp->gc_reset_chip = &bfe_reset_chip;
		gcp->gc_init_chip = &bfe_init_chip;
		gcp->gc_start_chip = &bfe_start_chip;
		gcp->gc_stop_chip = &bfe_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &bfe_set_rx_filter;
		gcp->gc_set_media = &bfe_set_media;
		gcp->gc_get_stats = &bfe_get_stats;
		gcp->gc_interrupt = &bfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &bfe_tx_desc_write;
		gcp->gc_tx_start = &bfe_tx_start;
		gcp->gc_rx_desc_write = &bfe_rx_desc_write;
		gcp->gc_rx_start = &bfe_rx_start;
#ifdef GEM_CONFIG_TX_HEAD_PTR
		gcp->gc_tx_desc_stat = NULL;
		gcp->gc_tx_desc_head = &bfe_tx_desc_head;
#else
		gcp->gc_tx_desc_stat = &bfe_tx_desc_stat;
#endif
		gcp->gc_rx_desc_stat = &bfe_rx_desc_stat;
		gcp->gc_tx_desc_init = &bfe_tx_desc_init;
		gcp->gc_rx_desc_init = &bfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &bfe_tx_desc_init;
		gcp->gc_rx_desc_clean = &bfe_rx_desc_init;
		gcp->gc_get_packet = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &bfe_mii_config;
		gcp->gc_mii_sync = &bfe_mii_sync;
		gcp->gc_mii_read = &bfe_mii_read;
		gcp->gc_mii_write = &bfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_max_mtu = ETHERMTU;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN;
#endif
		lp = kmem_zalloc(sizeof (struct bfe_dev), KM_SLEEP);
		lp->revid = revid;
		lp->need_to_reset = B_FALSE;
		lp->init_done = B_FALSE;

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (struct gem_conf));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof (struct bfe_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
bfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(bfe_ops, bfeattach, bfedetach);
#else
static	struct module_info bfeminfo = {
	0,			/* mi_idnum */
	"bfe",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit bferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&bfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit bfewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&bfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	bfe_info = {
	&bferinit,	/* st_rdinit */
	&bfewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_bfe_ops = {
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
	&bfe_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops bfe_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	bfeattach,	/* devo_attach */
	bfedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_bfe_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&bfe_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!bfe: _init: called"));
	gem_mod_init(&bfe_ops, "bfe");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&bfe_ops);
	}

	return (status);
}

/*
 * _fini : done
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!bfe: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&bfe_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
