/*
 *  bfe_gem.c: Broadcom 440x Fast Ethernet MAC driver for Solaris
 *
 * Copyright (c) 2003-2006 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)bfe_gem.c	1.1 06/05/21"

/*
 Change log
 0.8.0  09/21/2003
	first release
 0.8.1  09/21/2002
	flow control disabled because of tx performance problem.
	macro definitions are fixed
 0.8.2
 	09/21/2003
	flow control bug fixed.
	Status LED enabled.
	promiscous problem in set_rx_filter fixed
	register operation changed to use load/store directly

 	10/10/2003
	rx_desc_stat: retrying reading receive length.
	flow control disabled as it is still funny.
 0.8.3
	10/12/2003
	licensing changed to the BSD license
	source code clean up.
 0.8.4  05/05/2004
	dd_getlongprop removed for DDI/DKI complience.
	wrong attribute in ddi_dma_alloc_handle() for tx.
 0.8.7  11/20/2004
	hiwat of bfeminfo was changed.
	tx bounce buffer corruption in gem fixed.
 0.8.8  11/23/2004
	MAE bit in command register of pci config space fixed for DELL
	Inspillons.

	behaviour when no rx buffer fixed.

 	07/21/2005
	resume/suspend supported but not tested yet.
	behaviour when physical address exceeds the hardware ability tested
	 (use TEST_SMALL_DMA_RANGE to test)
 0.8.15 07/22/2005

	09/29/2005
	restart autonego iff the phy have been reset on bfe_reset_chip.
	debug reports on cache line size and latency timer in pci cfg added
	[backports from 3.0.13]
	 don't use RXINTDELAY
	 don't clear RXCFG at initialization.
	 use single fragment for tx
 0.8.17t0 09/29/2005
	 don't clear TXCFG at initialization.
	 txmaxbuffer changed to 1518+32
	 rxmaxbuffer changed to 1518+32
	 tx and rx dma engine initialazation was moved to tha end of
		 bfe_init_chip.
	 bfe_reset_chip() fixed
 0.8.17t4 02/01/2005
	hang by bogus interrupt fixed. (bfe_reset_chip and bfe_interrupt)

 0.8.17 02/04/2005
 */

/*
 TODO:
	mii_read/write might fail.  should retry on timeout error?
 */

/*
 * System Header files.
 */
#include <values.h>
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
#include "bcm4400reg.h"

char	ident[] = "bcm4400 nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int bfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (bfe_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1000*1000)

#define	RX_HEAD_ROOM	(RX_HEADER_SIZE + 2)

#ifdef NOFLUSH
#define	FLUSH(dp, reg)
#else
#define	FLUSH(dp, reg)	INL(dp, reg)
#endif

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

#define	TXSTAT_DESC_INDEX(dp)	\
	((INL(dp, TXSTAT) & TXSTAT_DESCOFFSET) / sizeof(struct bfe_desc))

#define	RXSTAT_DESC_INDEX(dp)	\
	((INL(dp, RXSTAT) & RXSTAT_DESCOFFSET) / sizeof(struct bfe_desc))

#ifndef INT32_MAX
# define INT32_MAX	0x7fffffff
#endif

/*
 * Our configuration
 */
#ifdef TEST_RX_EMPTY
# define RX_BUF_SIZE	1
#endif

#define	OUR_INTR_BITS	\
	(INT_RX | INT_TX | INT_DESCERR | INT_DATAERR | INT_DESCPROT | \
	 INT_RXDESCEMPTY | INT_RXOF | INT_TXUF)

#define TX_RING_SIZE	(DESC_BASE_ALIGN / 8)
#define TX_BUF_SIZE	64
#define RX_RING_SIZE	(DESC_BASE_ALIGN / 8)
#ifndef RX_BUF_SIZE
# define RX_BUF_SIZE	64
#endif

#if TX_BUF_SIZE >= TX_RING_SIZE
# error TX_BUF_SIZE must be less than TX_RING_SIZE
#endif

#ifdef CONFIG_COPY_PACKET
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
	int		rx_tail;

	/* interrupt control */
	uint32_t	intmask;
	int		tx_tail;
#ifdef GEM_CONFIG_POLLING
	int		last_poll_interval;
#endif

	/* reset control */
	boolean_t	need_to_reset;
	boolean_t	init_done;

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
	{0x14e4, 0x4402, "BCM4401B0"},
	{0x14e4, 0x170c, "BCM4401B0"},
};
#define CHIPTABLESIZE   (sizeof(bfe_chiptbl)/sizeof(struct chip_info))

/* ======================================================== */
 
/* mii operations */
static void  bfe_mii_sync(struct gem_dev *);
static uint16_t  bfe_mii_read(struct gem_dev *, uint_t);
static void bfe_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int bfe_attach_chip(struct gem_dev *);
static int bfe_reset_chip(struct gem_dev *);
static void bfe_init_chip(struct gem_dev *);
static void bfe_set_rx_filter(struct gem_dev *);
static void bfe_set_media(struct gem_dev *);
static void bfe_start_chip(struct gem_dev *);
static int bfe_stop_chip(struct gem_dev *);
static void bfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int bfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq);
static int bfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t bfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t bfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void bfe_tx_desc_init(struct gem_dev *dp, int slot);
static void bfe_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int bfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr bfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr bfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

#ifdef TEST_SMALL_DMA_RANGE
#  undef	SB_PCIDMA_SIZE
#  define	SB_PCIDMA_SIZE	(128*1024*1024)
#endif

static ddi_dma_attr_t bfe_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_addr_hi */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_count_max */
	0,/* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_maxxfer */
	SB_PCIDMA_SIZE - 1,	/* dma_attr_seg */
	0,/* patched later */	/* dma_attr_sgllen */
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
 * HW manupilation routines
 */
/* ======================================================== */

static int
bfe_reset_chip(struct gem_dev *dp)
{
	struct bfe_dev		*lp = (struct bfe_dev *)dp->private;
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

	if (lp->init_done) {
		/* inhibit all interrrupts before resetting nic */
		OUTL(dp, INTMASK, lp->intmask = 0); FLUSH(dp, INTMASK);
		DPRINTF(2, (CE_CONT, "!%s: %s: sb_intmask:0x%x",
				dp->name, __func__, INL(dp, SB_INTMASK)));
	}

#if DEBUG_LEVEL > 4
	if ((INL(dp, SB_TMSTATEL) &
		(SB_TMSTATEL_RESET | SB_TMSTATEL_REJ | SB_TMSTATEL_CLK)) != SB_TMSTATEL_CLK) {
		/* the core isn't up now */
		cmn_err(CE_CONT, "!%s: %s: core isn't up", dp->name, __func__);
	}
#endif

	if ((INL(dp, SB_TMSTATEL) &
		(SB_TMSTATEL_RESET | SB_TMSTATEL_REJ | SB_TMSTATEL_CLK)) != SB_TMSTATEL_CLK ||
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
#ifdef DBG_NOBURST
		val = INL(dp, SBTOPCI2) & ~(SBTOPCI_PREF | SBTOPCI_BURST);
#else
		val = INL(dp, SBTOPCI2) | SBTOPCI_PREF | SBTOPCI_BURST;
#endif
		OUTL(dp, SBTOPCI2, val);

		/* restore bar0window mapping */
		pci_config_put32(conf_handle, PCI_BAR0_WIN, bar0win);

		pci_config_teardown(&conf_handle);
	}
	else {
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

		for (i = 0; (INL(dp, SB_TMSTATEL) & SB_TMSTATEL_REJ)==0; i++) {
			if (i > 100) {
				cmn_err(CE_NOTE,
				"!%s: %s: timeout: waiting for SB_TMSTATE_REJ",
					dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}

		for (i = 0; (INL(dp, SB_TMSTATEH) & SB_TMSTATEH_BUSY)!=0; i++) {
			if (i > 100) {
				cmn_err(CE_NOTE,
				"!%s: %s: timeout: waiting for SB_TMSTATH_BUSY",
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
	}
	else {
		DPRINTF(0, (CE_CONT, "!%s: %s: no need to reset the core",
				dp->name, __func__));
	}

	OUTL(dp, SB_TMSTATEL,
		SB_TMSTATEL_FGC | SB_TMSTATEL_CLK | SB_TMSTATEL_RESET);
	FLUSH(dp, SB_TMSTATEL);
	drv_usecwait(1);

	if ((INL(dp, SB_TMSTATEH) & SB_TMSTATEH_SERR) != 0) {
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
	}
	else {
		if ((phyctl & PHYCTL_RESET) != 0) {
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

	/* 1/13/2006 clear and mask unexpected interrupts */
	OUTL(dp, INTMASK, lp->intmask = 0); FLUSH(dp, INTMASK);
	OUTL(dp, INTSTAT, 0xffffffff); FLUSH(dp, INTSTAT);

	return (GEM_SUCCESS);
}

#define	RX_TIMEOUT_10u	625		/* 10uS */
#define	RX_FRAMECNT	2

static void
bfe_init_chip(struct gem_dev *dp)
{
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;
	int		rx_delay;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT((dp->rx_ring_dma & 0xfff) == 0);
	ASSERT((dp->tx_ring_dma & 0xfff) == 0);

	/* 000 PHYCTL: have been configured */

	/* 00c BISTSTAT: ignore */

	/* 010 WAKEUPLEN: ignore */

	/* 020 INTSTAT: ignore */

	/* 024 INTMASK: clear */
	OUTL(dp, INTMASK, 0);

	/* 028 GPTIMER: clear */
	OUTL(dp, GPTIMER, 0);
#if 0
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
	rx_delay = 1;
	OUTL(dp, RXINTDELAY,
		(rx_delay << RXINTDELAY_FRAMECNT_SHIFT) |
		(15*RX_TIMEOUT_10u * (rx_delay - 1)));

	/* 400 RXCFG: don't touch */

	/* 404 RXMAXLEN: must be 1518+32 */
	OUTL(dp, RXMAXLEN, dp->mtu +
		sizeof(struct ether_header) + ETHERFCSL + RX_HEAD_ROOM + 2);
	if (dp->mtu + sizeof(struct ether_header) +
				ETHERFCSL + RX_HEAD_ROOM + 2 != 1518+32) {
		cmn_err(CE_WARN, "!%s: rxmaxlen isn't 1518+32 (%d)",
			dp->name,
			dp->mtu + sizeof(struct ether_header) +
				ETHERFCSL + RX_HEAD_ROOM + 2);
	}

	/* 408 TXMAXLEN: must be 1518+32 */
	OUTL(dp, TXMAXLEN, dp->mtu + sizeof(struct ether_header) + ETHERFCSL + 32);
	if (dp->mtu + sizeof(struct ether_header) + ETHERFCSL + 32 != 1518+32) {
		cmn_err(CE_WARN, "!%s: txmaxlen isn't 1518+32 (%d)",
			dp->name,
			dp->mtu + sizeof(struct ether_header) + ETHERFCSL + 32);
	}

	/* 418 MACINTMASK: don't touch */

	/* 42c ENETCTL: don't touch */

	/* 430 TXCFG: don't touch */

	/* 434 TXHIWAT: must be 56 */
	OUTL(dp, TXHIWAT, 56);	/* 56 */

	/* 438 MIBCTL */
	OUTL(dp, MIBCTL, MIBCTL_AUTOCLR);

	/* 500 mibdata: clear by reading all mib counter registers */
	if (!lp->mibvalid) {
		ddi_rep_get32(dp->regs_handle, lp->mib, 
			(uint32_t *)(((long)dp->base_addr) + MIBDATA),
			MIB_SIZE, DDI_DEV_AUTOINCR);
		/* discard mib data */
		bzero(lp->mib, sizeof(lp->mib));
		lp->mibvalid = B_TRUE;
	}

	/*
	 * finally, we setup dma engine.
	 */
	/* for tx dma channel */
#ifdef DBG_RXOF
	OUTL(dp, TXMAXBURSTLEN, 0x10); /**/
	OUTL(dp, RXMAXBURSTLEN, 0x10); /**/
	OUTL(dp, TXCTL, TXCTL_FPRI | TXCTL_EN);
#else
	OUTL(dp, TXCTL, TXCTL_EN);
#endif
	OUTL(dp, TXDESCBASE, dp->tx_ring_dma | SB_PCIDMA_BASE);

	/* for rx dma channel */
	OUTL(dp, RXCTL,
		(RX_HEAD_ROOM << RXCTL_OFFSET_SHIFT) | RXCTL_EN);
	OUTL(dp, RXDESCBASE, dp->rx_ring_dma | SB_PCIDMA_BASE);

	/* workaround for reseting nic on reception of long packets */
	lp->rx_rest = 0;

	lp->tx_tail = 0;
	lp->rx_tail = 0;
}

#ifdef DEBUG_LEVEL
static boolean_t
bfe_dump_cam(struct gem_dev *dp)
{
	int		i;
	int		ix;

	for (ix = 0; ix < 64; ix++) {
		OUTL(dp, CAMCTL, (ix << CAMCTL_INDEX_SHIFT) | CAMCTL_READ);

		for (i = 0; (INL(dp, CAMCTL) & CAMCTL_BUSY) != 0; i++) {
			if (i > 10) {
				cmn_err(CE_WARN,
				"!%s: dump_cam: timeout on reading cam",
					dp->name);
				return;
			}
			drv_usecwait(10);
		}
		cmn_err(CE_CONT, "!cam[%d]: %08x, %08x",
				ix, INL(dp, CAMHI), INL(dp, CAMLOW));
	}
}
#endif

static boolean_t
bfe_load_addr_into_cam(struct gem_dev *dp, uint8_t *mac, uint_t ix)
{
	int		i;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	OUTL(dp, CAMLOW,
		(mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5]);
	OUTL(dp, CAMHI, CAMHI_VALID | (mac[0] << 8) | mac[1]);

	OUTL(dp, CAMCTL, (ix << CAMCTL_INDEX_SHIFT) | CAMCTL_WRITE);

	for (i = 0; (INL(dp, CAMCTL) & CAMCTL_BUSY) != 0; i++) {
		if (i > 10) {
			cmn_err(CE_WARN,
				"!%s: %s: timeout on writing to cam",
				dp->name, __func__);
			return (B_FALSE);
		}
		drv_usecwait(10);
	}
	return (B_TRUE);
}

static void
bfe_set_rx_filter(struct gem_dev *dp)	
{
	uint32_t	rxcfg;
	int		i;
	static uint8_t	mac_invalid[ETHERADDRL] = {0,0,0,0,0,0};
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rxcfg = INL(dp, RXCFG) & ~(RXCFG_PROMISC | RXCFG_ALLMULT);

	if (dp->nic_active) {
		/* XXX - should disable all rx filters for a while ? */
	}

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		rxcfg |= RXCFG_PROMISC;
	}
	else {
		/* disable the cam for writing */
		OUTL(dp, CAMCTL, 0); FLUSH(dp, CAMCTL);

		i = 0;

		/* first, we add our station address into the cam */
		bfe_load_addr_into_cam(dp, dp->cur_addr.ether_addr_octet, i++);

		/* next, append multicast addresses */
		if ((dp->rxmode & RXMODE_ALLMULTI) != 0 ||
		     /* XXX - we must reserve one for our station address */
		     dp->mc_count > CAM_ENTRY_SIZE - 1) {
			rxcfg |= RXCFG_ALLMULT;
		}
		else if (dp->mc_count > 0) {
			for (; i < dp->mc_count; i++) {
				bfe_load_addr_into_cam(dp,
				    dp->mc_list[i].addr.ether_addr_octet, i);
			}
		}

		/* XXX - invalidate unused CAM entries. 2004/12/19 */
		for (; i < CAM_ENTRY_SIZE; i++) {
			bfe_load_addr_into_cam(dp, mac_invalid, i);
		}
#if DEBUG_LEVEL > 4
		bfe_dump_cam(dp);
#endif
		OUTL(dp, CAMCTL, INL(dp, CAMCTL) | CAMCTL_EN);
	}
	OUTL(dp, RXCFG, rxcfg);
}

static void
bfe_set_media(struct gem_dev *dp)
{
	uint32_t	rx;
	uint32_t	tx;
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#ifdef GEM_DELAYED_START
	ASSERT(dp->nic_online);
#else
	if (!dp->nic_active) {
		/* the chip is not initialized properly yet. */
		return;
	}
#endif
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
		tx |= FLOWCTL_ENABLE | (192 & FLOWCTL_RXHIWAT);
		break;

	case FLOW_CONTROL_SYMMETRIC:
		rx |= RXCFG_EN_FLOWCTL;
		tx |= FLOWCTL_ENABLE | (192 & FLOWCTL_RXHIWAT);
		break;
	}
	OUTL(dp, RXCFG, rx);
	OUTL(dp, FLOWCTL, tx);
}

static void
bfe_start_chip(struct gem_dev *dp)
{
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#ifndef GEM_DELAYED_START
	bfe_set_media(dp);
#endif
	/* Kick TX and RX */
	OUTL(dp, ENETCTL, INL(dp, ENETCTL) | ENETCTL_ENABLE);

	/* notify mac that rx buffers have been prepared */
	OUTL(dp, RXTAIL, lp->rx_tail * sizeof(struct bfe_desc));

	/* enable interrupt */
	lp->intmask = OUR_INTR_BITS;
#ifdef GEM_CONFIG_POLLING
	lp->intmask |= INT_GPTIMER;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, INTMASK, lp->intmask); FLUSH(dp, INTMASK);
	}
}

static int
bfe_stop_chip(struct gem_dev *dp)
{
	int		i;
	int		wait_time;
	struct txbuf	*tbp;
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;
	extern int	gem_speed_value[];

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Disable interrupts by clearing the interrupt mask */
	lp->intmask = 0;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, INTMASK, lp->intmask); FLUSH(dp, INTMASK);
	}

	OUTL(dp, RXINTDELAY, 0);

	if ((INL(dp, TXSTAT) & TXSTAT_ERR) == TXSTAT_ERR_NONE) {
		/* wait until all tx buffer have been transmitted */
		wait_time =
			(TX_BUF_SIZE * dp->mtu) / gem_speed_value[dp->speed];
		for (i = 0; TXSTAT_DESC_INDEX(dp) != lp->tx_tail; i++) {
			if (i > wait_time) {
				cmn_err(CE_NOTE,
					"!%s: %s: timeout: stopping tx dma",
					dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}
		DPRINTF(4, (CE_CONT, "%s: %s: the nic stopped in %d uS",
				dp->name, __func__, i*10));
	}

	/* tx fifo drain time (guess) */
	drv_usecwait(210);

	/* stop tx dma engine */
	OUTL(dp, TXCTL, 0);

	/* Stop Tx and Rx processes in the chip. */
	OUTL(dp, ENETCTL, ENETCTL_DISABLE);
	for (i = 0; (INL(dp, ENETCTL) & ENETCTL_DISABLE) != 0; i++) {
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
		for (i = 0;
			(INL(dp, RXSTAT) & RXSTAT_STATE) == RXSTAT_STATE_ACTIVE;
				i++) {
			if (i > 200) {
				cmn_err(CE_WARN,
					"!%s: %s: timeout: stopping rx dma",
					dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}
	}

	/* stop rx dma engine */
	OUTL(dp, RXCTL, 0);

	if (lp->mibvalid) {
		/* read mib counters before resettting the chip */
		bfe_get_stats(dp);
	}

	return (GEM_SUCCESS);
}

static void
bfe_eeprom_dump(struct gem_dev *dp, uint32_t *prom)
{
	int		i;

	cmn_err(CE_CONT, "!%s: bfe_eeprom dump:", dp->name);
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
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;
	uint8_t		*prom = (uint8_t *)lp->prom_data;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	for (i = 0; i < EEPROM_SIZE; i++) {
		lp->prom_data[i] = INL(dp, EEPROM_BASE + i*sizeof(uint32_t));
	}
#if DEBUG_LEVEL > 0
	bfe_eeprom_dump(dp, lp->prom_data);
#endif
	if (prom[EEPROM_MAGIC] != 0x01 &&
	    prom[EEPROM_MAGIC] != 0x10) {
		cmn_err(CE_WARN,
		"!%s: %s: contents in eeprom is corrupted", dp->name, __func__);
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
	dp->mii_phy_addr =prom[EEPROM_PHYADDR] & 0x1f;

	/* fix rx buffer length, it must have additional 4 byte and rx_header */
	dp->rx_buf_len = dp->mtu + sizeof(struct ether_header)
			+ ETHERFCSL + 4 + RX_HEAD_ROOM;

	DPRINTF(2, (CE_CONT, "!%s: %s:"
		" phy_addr:%d, macintmask:0x%x,"
		" txcfg:0x%x, txmaxburst:0x%x,"
		" rxmaxburst:0x%x, txhiwat:0x%x",
			dp->name, __func__,
			dp->mii_phy_addr,
			INL(dp, MACINTMASK), INL(dp, TXCFG),
			INL(dp, TXMAXBURSTLEN), INL(dp, RXMAXBURSTLEN),
			INL(dp, TXHIWAT)));

	/* mib counters are not initialized */
	lp->mibvalid = B_FALSE;

	return (GEM_SUCCESS);
}


static void
bfe_get_stats(struct gem_dev *dp)
{
	int	txerr;
	int	rxerr;

	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;

	if (!lp->mibvalid) {
		return;
	}

	ddi_rep_get32(dp->regs_handle, lp->mib, 
		(uint32_t *)(((caddr_t)dp->base_addr) + MIBDATA),
		MIB_SIZE, DDI_DEV_AUTOINCR);

	txerr = 0;
	rxerr = 0;

	dp->stats.collisions	+= lp->mib[0x40/4];
	dp->stats.first_coll	+= lp->mib[0x44/4];
	dp->stats.multi_coll	+= lp->mib[0x48/4];
	dp->stats.excoll	+= lp->mib[0x4c/4]; txerr += lp->mib[0x4c/4];
	dp->stats.nocarrier	+= lp->mib[0x58/4]; txerr += lp->mib[0x58/4];
	dp->stats.defer		+= lp->mib[0x54/4];
	dp->stats.underflow	+= lp->mib[0x3c/4]; txerr += lp->mib[0x3c/4];
	dp->stats.xmtlatecoll	+= lp->mib[0x50/4]; txerr += lp->mib[0x50/4];

	dp->stats.frame_too_long += lp->mib[0xb4/4]; rxerr += lp->mib[0xb4/4];
	dp->stats.missed	+= lp->mib[0xbc/4]; rxerr += lp->mib[0xbc/4];
	dp->stats.runt		+= lp->mib[0xc4/4]; rxerr += lp->mib[0xc4/4];
	dp->stats.crc		+= lp->mib[0xc8/4]; rxerr += lp->mib[0xc8/4];
	dp->stats.frame	+= lp->mib[0xcc/4] + lp->mib[0xd0/4];
				rxerr += lp->mib[0xcc/4] + lp->mib[0xd0/4];
	dp->stats.errxmt += txerr;
	dp->stats.errrcv += rxerr;
}

/*
 * discriptor manupiration
 */
static int
bfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq)
{
	int			i;
	int			ix;
	struct bfe_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	struct bfe_dev		*lp = (struct bfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	"!%s: bfe_tx_desc_write seqnum: %d, slot %d, frags: %d flags: %d",
		dp->name, dp->tx_desc_tail, slot, frags, intreq);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
			i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
	cmn_err(CE_CONT, "!          isr: %b", INL(dp, INTSTAT), INT_BITS);
#endif
#if DEBUG_LEVEL > 2
	intreq |= GEM_TXFLAG_INTR;
#endif
	/*
	 * write tx descriptor(s)
	 */
	mark = CTL_STARTF;
	dcp = dmacookie;
	ix = slot;
#ifndef CONFIG_TX_SINGLE
	for (i = frags; --i > 0; ix = SLOT(ix + 1, TX_RING_SIZE), dcp++) {
		tdp = &((struct bfe_desc *)dp->tx_ring)[ix];
		ddi_put32(h, &tdp->desc_addr,
				dcp->dmac_address | SB_PCIDMA_BASE);
		ddi_put32(h, &tdp->desc_ctl,
				(dcp->dmac_size & CTL_LENGTH) | mark
				 | ((ix == TX_RING_SIZE - 1) ? CTL_ENDTBL : 0));
		mark = 0;

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct bfe_desc), DDI_DMA_SYNC_FORDEV);
	}
#endif /* !TX_SINGLE */
	/* make last fragment */
	tdp = &((struct bfe_desc *)dp->tx_ring)[ix];
	ddi_put32(h, &tdp->desc_addr, dcp->dmac_address | SB_PCIDMA_BASE);
	ddi_put32(h, &tdp->desc_ctl,
			(dcp->dmac_size & CTL_LENGTH) | mark | CTL_ENDF
			| (intreq ? CTL_INTREQ : 0)
			| ((ix == TX_RING_SIZE - 1) ? CTL_ENDTBL : 0));

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct bfe_desc), DDI_DMA_SYNC_FORDEV);

	/* set new tail */
	lp->tx_tail = SLOT(slot + frags, TX_RING_SIZE);
	if ((lp->intmask & INT_TX) != 0) {
		OUTL(dp, TXTAIL, lp->tx_tail*sizeof(struct bfe_desc));
	}

	return (frags);
}

static int
bfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	struct bfe_desc		*rdp;
	struct bfe_dev		*lp = (struct bfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

#if DEBUG_LEVEL > 3
	cmn_err(CE_CONT,
		"!%s: bfe_rx_desc_write seqnum: %d, slot %d, frags: %d",
		dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x(%d)",
			i, dmacookie[i].dmac_laddress,
			dmacookie[i].dmac_size, dmacookie[i].dmac_size);
	}
#endif
	ASSERT(frags == 1);

	/*
	 * write a RX descriptor
	 */
	rdp = &((struct bfe_desc *)dp->rx_ring)[slot];

	ddi_put32(h, &rdp->desc_addr, dmacookie->dmac_address | SB_PCIDMA_BASE);
	ddi_put32(h, &rdp->desc_ctl,
			(dmacookie->dmac_size & CTL_LENGTH)
		        | ((slot == RX_RING_SIZE - 1) ? CTL_ENDTBL : 0));

	/* clear rx header area to check reception of big packets */
	bzero(dp->rx_buf_tail->rxb_buf, RX_HEAD_ROOM);
	ddi_dma_sync(dp->rx_buf_tail->rxb_dh,
			0, RX_HEAD_ROOM, DDI_DMA_SYNC_FORDEV);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct bfe_desc), DDI_DMA_SYNC_FORDEV);

	lp->rx_tail = SLOT(slot + 1, RX_RING_SIZE);
#ifndef RXTAIL_COALEASE
	if (dp->nic_active) {
		/* set new tail */
		OUTL(dp, RXTAIL,
			lp->rx_tail * sizeof(struct bfe_desc));
	}
#endif
	return (frags);
}

static void
bfe_tx_desc_dump(struct gem_dev *dp, seqnum_t head, int ndesc)
{
	int			i;
	struct bfe_desc		*tdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	cmn_err(CE_CONT, "!%s: tx_desc_dump:  slot:%d, seqnum:%d",
		dp->name, SLOT(head, TX_RING_SIZE), head);

	for (i = 0; i < ndesc; i++) {
		tdp = &((struct bfe_desc *)dp->tx_ring)[
				SLOT(head + i, TX_RING_SIZE)];
		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct bfe_desc), DDI_DMA_SYNC_FORKERNEL);

		cmn_err(CE_CONT, "! %d: ctrl:%b, addr:0x%08x",
			i, ddi_get32(h, &tdp->desc_ctl), CTL_BITS,
			ddi_get32(h, &tdp->desc_addr));
	}
}

static uint_t
bfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	if (TXSTAT_DESC_INDEX(dp) == slot) {
		/* not yet */
		return (0);
	}
	return (GEM_TX_DONE);
}

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

static uint64_t
bfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rxbuf		*rbp = dp->rx_buf_head;
	volatile uint8_t	*hdp;
	uint16_t		rxf;
	uint_t			len;
	int			i;
	uint_t			ret;
	struct bfe_dev		*lp = (struct bfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	/* compare the slot with current rx slot index */
	if (RXSTAT_DESC_INDEX(dp) == slot) {
		/* not received yet */
		return (0);
	}

	if (lp->rx_rest > 0) {
		/*
		 * Ignore this fragment as it is a part of previous jumbo
		 * packet.
		 */
		lp->rx_rest -= min(rbp->rxb_buf_len, lp->rx_rest);
		return (GEM_RX_ERR);
	}

	/*
	 * Get rx header which is leading received data area.
	 */
	hdp = (volatile uint8_t *) rbp->rxb_buf;

	/*
	 * Need to wait until the length field at the begining
	 * of the receive buffer updated in case of received packet
	 * exceeds the buffer.
	 */
	for (i = 0; i < 100; i++) {
		/* sync rx header area in the rx buffer */
		ddi_dma_sync(rbp->rxb_dh,
			0, RX_HEAD_ROOM, DDI_DMA_SYNC_FORKERNEL);

		len = (hdp[RXHD_LEN + 1] << 8) | hdp[RXHD_LEN];
		rxf = (hdp[RXHD_RXF + 1] << 8) | hdp[RXHD_RXF];
		if (len != 0) {
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

	if ((rxf & RXF_ERRORS) != 0) {
		/* errored packet */
#ifdef notdef
		if ((rxf & RXF_NIBBLEODD) != 0) {
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
	struct bfe_desc		*tdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tdp = &((struct bfe_desc *)dp->tx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &tdp->desc_ctl,
			(slot == TX_RING_SIZE - 1) ? CTL_ENDTBL : 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct bfe_desc), DDI_DMA_SYNC_FORDEV);
}

static void
bfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct bfe_desc		*rdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct bfe_desc *)dp->rx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &rdp->desc_ctl,
			(slot == RX_RING_SIZE - 1) ? CTL_ENDTBL : 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct bfe_desc), DDI_DMA_SYNC_FORDEV);
}

/*
 * Device depend interrupt handler
 */
static u_int
bfe_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flag = 0;
	struct bfe_dev	*lp = (struct bfe_dev *)dp->private;

	isr = INL(dp, INTSTAT);

	if ((isr & lp->intmask) == 0) {
		/* Not for us */
		if (isr != 0) {
			OUTL(dp, INTSTAT, isr);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: Interrupt, isr: 0x%b",
			   dp->name, isr, INT_BITS));

	if (!dp->nic_active) {
		/* the device is not active */
		if (isr != 0) {
			OUTL(dp, INTSTAT, isr);
		}
		return (DDI_INTR_CLAIMED);
	}

	OUTL(dp, INTMASK, 0);
	OUTL(dp, INTSTAT, isr);

	/*
	 * No need to flush INTSTAT because bfe_tx_desc_stat() and
	 * bfe_rx_desc_stat() routines imply reading registers in the nic.
	 */

#ifdef GEM_CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval != 0) {
			/* polling mode */
			lp->intmask &= ~(INT_RX | INT_TX);
			OUTL(dp, GPTIMER, dp->poll_interval * 62);

			if (lp->last_poll_interval == 0) {
				/*
				 * To schedule the next timer interrupt,
				 * we pretend as we were interrupted from
				 * polling timer
				 */
				isr |= INT_GPTIMER;
			}
		}
		else {
			/* normal mode */
			lp->intmask |= INT_RX | INT_TX;
			OUTL(dp, GPTIMER, 0);
		}

		lp->last_poll_interval = dp->poll_interval;
	}

	if ((isr & INT_GPTIMER) != 0) {
		/* force to process RX and TX */
		isr |= INT_RX | INT_TX;
	}
#else
	isr &= lp->intmask;
#endif /* GEM_CONFIG_POLLING */

	if ((isr & (INT_RX | INT_RXDESCEMPTY)) != 0) {
		if (gem_receive(dp) > 0) {
#ifdef RXTAIL_COALEASE
			OUTL(dp, RXTAIL, lp->rx_tail * sizeof(struct bfe_desc));
#else
			;
#endif
		}

		if ((isr & INT_RXDESCEMPTY) != 0) {
			/* RX descriptor is empty */
			DPRINTF(2, (CE_CONT,
				"!%s: %s: rxctl:%x rxtail:%x rxstat:%x",
				dp->name, __func__,
				INL(dp, RXCTL), INL(dp, RXTAIL),
				INL(dp, RXSTAT)));
			dp->stats.norcvbuf++;
		}
	}

	if ((isr & INT_TX) != 0) {
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
#ifdef GEM_CONFIG_POLLING
		if ((isr & INT_GPTIMER) != 0) {
			/* force to process RX and TX */
			OUTL(dp, TXTAIL, lp->tx_tail*sizeof(struct bfe_desc));
		}
#endif
	}

	if ((isr & (INT_DESCERR | INT_DATAERR |
		    INT_DESCPROT | INT_RXOF | INT_TXUF)) != 0) {
		FLUSH(dp, INTSTAT);
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b",
			dp->name, isr, INT_BITS);

		if ((isr & INT_RXOF) != 0) {
			dp->stats.overflow++;
		}
		if ((isr & INT_TXUF) != 0) {
			dp->stats.underflow++;
		}

		lp->need_to_reset = B_TRUE;
	}

        if (lp->need_to_reset) {
		cmn_err(CE_WARN, "!%s: %s: isr:%b, resetting the chip ...",
			dp->name, __func__, isr, INT_BITS);

                mutex_enter(&dp->xmitlock);
#if 0
		/* XXX - workaround for receiving jumbo packet: reset twice */
		bfe_reset_chip(dp);
#endif
                gem_restart_nic(dp, B_TRUE);
                mutex_exit(&dp->xmitlock);
		flag |= INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
        }

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* restore interrupt mask */
		OUTL(dp, INTMASK, lp->intmask);
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

	if (gem_mii_config_default(dp) < 0) {
		return (GEM_FAILURE);
	}

	/* enable LED */
	val = GEM_MII_READ(dp, 0x1a);
	GEM_MII_WRITE(dp, 0x1a, val & ~0x8000);

	val = GEM_MII_READ(dp, 0x1b);
	GEM_MII_WRITE(dp, 0x1b, val & 0x0040);

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

	if ((INL(dp, MACINTSTAT) & MACINT_MII) != 0) {
		cmn_err(CE_WARN,
		"!%s: %s: failed to clear MII bit in MACINTSTAT register",
			dp->name, __func__);
		return (0xffff);
	}

	OUTL(dp, MIIDATA, MII_READ_CMD(dp->mii_phy_addr, reg));

	/* wait until done */
	for (i = 0; (INL(dp, MACINTSTAT) & MACINT_MII) == 0; i++) {
		if (i > 100) {
			val = INL(dp, MIIDATA);
			cmn_err(CE_WARN, "!%s: %s: timeout: 0x%b",
				dp->name, __func__, val, MII_STATUS_BITS);
			return (0xffff);
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

	if ((INL(dp, MACINTSTAT) & MACINT_MII) != 0) {
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
	ddi_acc_handle_t	conf_handle;
	int			ret;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			revid;
	uint32_t		ilr;
	uint8_t			cacheline;
	uint8_t			latencytimer;
	int			unit;
	struct chip_info	*p;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct bfe_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: bfeattach: called %s",
		drv_name, unit, ident));

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

	vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid= pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	/* XXX - BCM440x does't have cache line register */
	cacheline = pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ);
	latencytimer = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
#if 0
	pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 0x40); /* XXX */
#endif
	DPRINTF(0, (CE_CONT,
	"!%s%d: cache line size: 0x%02x, latency timer: 0x%02x, stat: 0x%04x",
		drv_name, unit, cacheline, latencytimer,
		pci_config_get16(conf_handle, PCI_CONF_STAT)));

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		for (i = 0, p = bfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (p->venid == vid && p->devid == did) {
				/* found */
				cmn_err(CE_CONT,
			"!%s%d: %s (vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
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
		if (gem_pci_regs_map_setup(dip,
			PCI_ADDR_MEM32,
			&bfe_dev_attr, (caddr_t *)&base, &regs_ha) !=
								DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
				drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *)
			kmem_zalloc(sizeof(struct gem_conf), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
#ifdef CONFIG_TX_SINGLE
		gcp->gc_tx_max_frags = 1;
#else
		gcp->gc_tx_max_frags = GEM_MAXTXFRAGS;
#endif
		gcp->gc_tx_desc_size = DESC_BASE_ALIGN;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_auto_pad  = B_FALSE;
		gcp->gc_tx_copy_thresh = bfe_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_size = DESC_BASE_ALIGN;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;
		gcp->gc_rx_max_descs_per_pkt = gcp->gc_rx_max_frags;
		gcp->gc_rx_copy_thresh = bfe_rx_copy_thresh;
		gcp->gc_rx_buf_max   = gcp->gc_rx_buf_size + 1;
		gcp->gc_rx_header_len= RX_HEAD_ROOM;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, bfe_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, bfe_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, bfe_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, bfe_dma_attr_desc);

		STRUCT_COPY(gcp->gc_dma_attr_txbuf, bfe_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_align  = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, bfe_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align  = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		/* XXX - dont use as it seems to cause tx performance problem */
		gcp->gc_flow_control = B_FALSE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval   = ONESEC/10;
		gcp->gc_mii_reset_timeout       = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout          = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait		= 0;
		gcp->gc_mii_linkdown_timeout    = MII_LINKDOWN_TIMEOUT; 

		/* workaround for PHY */
		gcp->gc_mii_an_delay	    = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset      = B_FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &bfe_attach_chip;
		gcp->gc_reset_chip = &bfe_reset_chip;
		gcp->gc_init_chip  = &bfe_init_chip;
		gcp->gc_start_chip = &bfe_start_chip;
		gcp->gc_stop_chip  = &bfe_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &bfe_set_rx_filter;
		gcp->gc_set_media = &bfe_set_media;
		gcp->gc_get_stats = &bfe_get_stats;
		gcp->gc_interrupt = &bfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &bfe_tx_desc_write;
		gcp->gc_rx_desc_write = &bfe_rx_desc_write;

		gcp->gc_tx_desc_stat  = &bfe_tx_desc_stat;
		gcp->gc_rx_desc_stat  = &bfe_rx_desc_stat;
		gcp->gc_tx_desc_init  = &bfe_tx_desc_init;
		gcp->gc_rx_desc_init  = &bfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &bfe_tx_desc_init;
		gcp->gc_rx_desc_clean = &bfe_rx_desc_init;
		gcp->gc_get_packet    = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_init   = &gem_mii_init_default;
		gcp->gc_mii_config = &bfe_mii_config;
		gcp->gc_mii_sync   = &bfe_mii_sync;
		gcp->gc_mii_read   = &bfe_mii_read;
		gcp->gc_mii_write  = &bfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = (struct bfe_dev *)
			kmem_zalloc(sizeof(struct bfe_dev), KM_SLEEP);
		lp->revid         = revid;
		lp->need_to_reset = B_FALSE;
		lp->init_done     = B_FALSE;

		dp = gem_do_attach(dip, gcp, base, &regs_ha, lp, sizeof(*lp));

		kmem_free(gcp, sizeof(struct gem_conf));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof(struct bfe_dev));
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
static	struct module_info bfeminfo = {
	0,			/* mi_idnum */
	"bfe",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	TX_BUF_SIZE*ETHERMAX,	/* mi_hiwat */
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
	ddi_power	/* devo_power */
};

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

	DPRINTF(2, (CE_CONT, "!bfe: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
