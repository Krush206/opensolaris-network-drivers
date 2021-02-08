/*
 *  tcfe: 3Com Fast Ethernet MAC driver for Solaris
 *  @(#)tcfe_gem.c	1.7 11/09/19
 *
 * Copyright (c) 2006-2011 Masayuki Murayama.  All rights reserved.
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
#ifdef notdef
    Change log
	2006/12/24 multicast hash calculation fixed
	2007/02/10 cleanup
	2007/02/10 2.4.0
#endif

/*
 * TODO:
 */

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
#include "3c90xreg.h"

char	ident[] = "3com 3c90x driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int tcfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (tcfe_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ROUNDUP(x, a)		(((x) + (a) - 1) & ~((a) - 1))

#define	EXEC_CMD(dp, cmd)	{ \
	mutex_enter(&((struct tcfe_dev *)(dp)->private)->cmdlock); \
	OUTW((dp), WxCommand, (cmd)); \
	mutex_exit(&((struct tcfe_dev *)(dp)->private)->cmdlock); \
}

#define	EXEC_CMD_WAIT(dp, cmd)	{ \
	mutex_enter(&((struct tcfe_dev *)(dp)->private)->cmdlock); \
	tcfe_exec_cmd_wait((dp), (cmd)); \
	mutex_exit(&((struct tcfe_dev *)(dp)->private)->cmdlock); \
}

#define	SET_WIN(dp, n)		EXEC_CMD(dp, CmdSelectRegisterWindow | (n))

#define	REG_LOCK(lp)		mutex_enter(&lp->reglock)
#define	REG_UNLOCK(lp)		mutex_exit(&lp->reglock)

#define	DN_LIST_SIZE_SHIFT	6
#define	DN_LIST_SIZE		(1 << DN_LIST_SIZE_SHIFT)
#define	MAXTXFRAGS	\
	((DN_LIST_SIZE - sizeof (uint32_t) * 2) / sizeof (struct DnFrag))

#define	UP_LIST_SIZE_SHIFT	5
#define	UP_LIST_SIZE		(1 << UP_LIST_SIZE_SHIFT)
#define	MAXRXFRAGS	\
	((UP_LIST_SIZE - sizeof (uint32_t) * 2) / sizeof (struct UpFrag))

/*
 * Our configuration
 */
#define	OUR_INTR_BITS	\
	(IS_hostError | IS_txComplete | IS_intRequested | \
	IS_updateStats | IS_upComplete | IS_dnComplete | IS_interruptLatch)

#define	TXFIFOSIZE	(2*1024)
#define	RXFIFOSIZE	(2*1024)

#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	TX_BUF_SIZE
#endif
#ifndef RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	ONESEC		(drv_usectohz(1*1000000))

static int	tcfe_tx_copy_thresh = 256;
static int	tcfe_rx_copy_thresh = 256;

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
	uint_t		flags;
#define	CHIP_TYPE	0x0007
#define	CHIP_TYPE_59X	0x0000
#define	CHIP_TYPE_90X	0x0001
#define	CHIP_TYPE_90XB	0x0002
#define	CHIP_TYPE_90XC	0x0003
#define	EEADDRSIZE_8	0x4000
#define	CHIP_CARDBUS	0x8000
	uint16_t	ee_offset;
	uint16_t	reset_opt;
};

#define	IS_3C59X(lp)	(((lp)->chip->flags & CHIP_TYPE) == CHIP_TYPE_59X)
#define	IS_3C90X(lp)	(((lp)->chip->flags & CHIP_TYPE) == CHIP_TYPE_90X)
#define	IS_3C90XB(lp)	(((lp)->chip->flags & CHIP_TYPE) == CHIP_TYPE_90XB)
#define	IS_3C90XC(lp)	(((lp)->chip->flags & CHIP_TYPE) == CHIP_TYPE_90XC)
#define	IS_3C90XB_OR_LATER(lp)	\
			(((lp)->chip->flags & CHIP_TYPE) >= CHIP_TYPE_90XB)
#define	IS_CARDBUS(lp)	(((lp)->chip->flags & CHIP_CARDBUS) != 0)

#define	ASIC_40_0502_00X(lp)	(IS_3C90XB(lp) && ((lp)->rev_id >> 5) == 0)
#define	ASIC_40_0483_00X(lp)	(IS_3C90XB(lp) && ((lp)->rev_id >> 5) == 1)
#define	ASIC_40_0476_001(lp)	(IS_3C90XB(lp) && ((lp)->rev_id >> 5) == 3)

static struct chip_info tcfe_chiptbl[] = {
#ifdef CONFIG_3C59X
	{0x10b7, 0x5900, "3C590", CHIP_TYPE_59X, 0, 0},
	{0x10b7, 0x5920, "3C592", CHIP_TYPE_59X, 0, 0},
	{0x10b7, 0x5970, "3C597", CHIP_TYPE_59X, 0, 0},
	{0x10b7, 0x5950, "3C595", CHIP_TYPE_59X, 0, 0},
	{0x10b7, 0x5951, "3C595", CHIP_TYPE_59X, 0, 0},
	{0x10b7, 0x5952, "3C595", CHIP_TYPE_59X, 0, 0},
#endif
/* boomerang 10M */
	{0x10b7, 0x9000, "3C900 10baseT", CHIP_TYPE_90X, 0, 0},
	{0x10b7, 0x9001, "3C900 10Mbps combo", CHIP_TYPE_90X, 0, 0},
/* cyclone 10M */
	{0x10b7, 0x9004, "3C900 10Mbps TPO", CHIP_TYPE_90XB, 0, 0},
	{0x10b7, 0x9005, "3C900 10Mbps combo", CHIP_TYPE_90XB, 0, 0},
/* tornade 10M */
	{0x10b7, 0x9006, "3C900 10Mbps TPC", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x900a, "3C900B-FL 10base-FL", CHIP_TYPE_90XC, 0, 0},
/* boomerang 100M */
	{0x10b7, 0x9050, "3C905 100baseTx", CHIP_TYPE_90X, 0, 0},
	{0x10b7, 0x9051, "3C905 100baseT4", CHIP_TYPE_90X, 0, 0},
/* cyclone 100M */
	{0x10b7, 0x9055, "3C905B 100baseTx", CHIP_TYPE_90XB, 0, 0},
	{0x10b7, 0x9058, "3C905B 10/100/BNC", CHIP_TYPE_90XB, 0, 0},
	{0x10b7, 0x905a, "3C905B-FX 100baseFx", CHIP_TYPE_90XB, 0, 0},
/* tornade 100M */
	{0x10b7, 0x9200, "3C905C", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x9202, "3C920B-EMB-WNM (ATI)", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x9201, "3C920", CHIP_TYPE_90XC, 0, 0},
/* 980 series cyclone */
	{0x10b7, 0x9800, "3C980", CHIP_TYPE_90XB, 0, 0},
	{0x10b7, 0x9805, "3C980C Python", CHIP_TYPE_90XB, 0, 0},
/* SOHO series tornade */
	{0x10b7, 0x7646, "3CSOHO100-TX", CHIP_TYPE_90XC, 0, 0},
/* mini-pci 555/556w/modem series */
	{0x10b7, 0x5055, "3C555", CHIP_TYPE_90XB | EEADDRSIZE_8, 0x30, 0},
	{0x10b7, 0x6055, "3C556",
		CHIP_TYPE_90XC | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertMIIPwr},
	{0x10b7, 0x6056, "3C556B",
		CHIP_TYPE_90XC | CHIP_CARDBUS,
			0x30, RO_invertMIIPwr},
/* cardbus 575 series */
	{0x10b7, 0x5b57, "3C575", CHIP_TYPE_90X | EEADDRSIZE_8, 0x30, 0},
	{0x10b7, 0x5057, "3C575", CHIP_TYPE_90X | EEADDRSIZE_8, 0x30, 0},
	{0x10b7, 0x5157, "3C575BT",
		CHIP_TYPE_90XB | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertLEDPwr},
	{0x10b7, 0x5257, "3C575CT",
		CHIP_TYPE_90XC | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertMIIPwr},
/* cardbus 656 series */
	{0x10b7, 0x6560, "3C656",
		CHIP_TYPE_90XB | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertMIIPwr | RO_invertLEDPwr},
	{0x10b7, 0x6562, "3C656B",
		CHIP_TYPE_90XB | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertMIIPwr | RO_invertLEDPwr},
	{0x10b7, 0x6564, "3C656C",
		CHIP_TYPE_90XC | CHIP_CARDBUS | EEADDRSIZE_8,
			0x30, RO_invertMIIPwr},
	{0x10b7, 0x4500, "3C450", CHIP_TYPE_90XC, 0, 0},
/* tornade chipset */
	{0x10b7, 0x1201, "3C982", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x1202, "3C982", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x9056, "3C905BT4", CHIP_TYPE_90XC, 0, 0},
	{0x10b7, 0x9210, "3C920B-EMB-WNM", CHIP_TYPE_90XC, 0, 0},
	{0xffff, 0xffff, "unknown 3com chipset", CHIP_TYPE_90XC, 0},
};
#define	CHIPTABLESIZE   (sizeof (tcfe_chiptbl)/sizeof (struct chip_info))

struct tcfe_dev {
	uint8_t			rev_id;
	struct chip_info	*chip;

	kmutex_t		reglock;
	kmutex_t		cmdlock;

	boolean_t		dnlistptr_loaded;
	uint32_t		intenable;

	/* cardbus staff */
	ddi_acc_handle_t	base2_ha;
	void			*base2;

	/* configuration */
	int			media;
	uint16_t		bmcr;
	uint16_t		bmsr;
	uint16_t		adv;

	/* polling mode */
#ifdef GEM_CONFIG_POLLING
	int			last_poll_interval;
#endif
	/* multicast hash */
	uint8_t			mhash[256/8];

	/* rx list mngt */
	int			rx_last_slot;

	/* global error recovery */
	boolean_t		need_to_reset;

	/* configuration */
	uint_t			ic;	/* internal config */
	uint_t			ms;	/* media status */
	boolean_t		mii_initialized;

	uint_t			tx_last_slot;
};

/* ======================================================== */

/* mii operations */
static void  tcfe_mii_sync(struct gem_dev *);
static uint16_t  tcfe_mii_read(struct gem_dev *, uint_t);
static void tcfe_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int tcfe_reset_chip(struct gem_dev *);
static int tcfe_attach_chip(struct gem_dev *);
static int tcfe_init_chip(struct gem_dev *);
static int tcfe_start_chip(struct gem_dev *);
static int tcfe_stop_chip(struct gem_dev *);
static int tcfe_set_media(struct gem_dev *);
static int tcfe_set_rx_filter(struct gem_dev *);
static int tcfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int tcfe_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void tcfe_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void tcfe_tx_start(struct gem_dev *dp, int slot, int nslot);
static void tcfe_rx_start(struct gem_dev *dp, int slot, int nslot);
static uint_t tcfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t tcfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void tcfe_tx_desc_init(struct gem_dev *dp, int slot);
static void tcfe_rx_desc_init(struct gem_dev *dp, int slot);
static void tcfe_tx_desc_clean(struct gem_dev *dp, int slot);
static void tcfe_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t tcfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr tcfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* Tx and rx buffers should have native endianness for sparc platforms. */
static struct ddi_device_acc_attr tcfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

#define	TCFE_MAX_PACKET_SIZE	(0x1fff * (min(MAXRXFRAGS, MAXTXFRAGS) - 2) + 2)
static ddi_dma_attr_t tcfe_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x1fffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	TCFE_MAX_PACKET_SIZE,	/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t tcfe_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	16,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
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
static void
tcfe_exec_cmd_wait(struct gem_dev *dp, int cmd)
{
	int		i;
	struct tcfe_dev	*lp = dp->private;

	OUTW(dp, WxCommand, cmd);

	for (i = 0; INW(dp, WxIntStatus) & IS_cmdInProgress; i++) {
		if (i > 200) {
			cmn_err(CE_WARN,
			    "%s: cmd timeout error: cmd:0x%04",
			    dp->name, cmd);
			lp->need_to_reset = B_TRUE;
			break;
		}
		drv_usecwait(10);
	}
}

static int
tcfe_reset_chip(struct gem_dev *dp)
{
	int	i;
	uint32_t	val;
	struct tcfe_dev	*lp = dp->private;

	if (!lp->mii_initialized) {
		EXEC_CMD_WAIT(dp, CmdGlobalReset);
		delay(drv_usectohz(10000));
	}

	if (IS_CARDBUS(lp)) {
		REG_LOCK(lp);

		SET_WIN(dp, 2);
		val = INW(dp, W2ResetOptions) &
		    ~(RO_invertLEDPwr | RO_invertMIIPwr);
		OUTW(dp, W2ResetOptions, val | lp->chip->reset_opt);

		REG_UNLOCK(lp);

		DPRINTF(0, (CE_CONT, "%s: %s reset option:%b",
		    dp->name, __func__, val, ResetOptionsBits));
		/* enable cardbus interrupt */
		ddi_put32(lp->base2_ha,
		    (void *)(((caddr_t)lp->base2) + 4), 0x8000); /* X */

		/*
		 * XXX - need to enable tranceiver power by writing
		 * 0x0800 into reg 0 in window 0
		 */
	}

	/* Reset transmitter */
	EXEC_CMD_WAIT(dp, CmdTxReset);

	/* Reset receiver */
	EXEC_CMD_WAIT(dp, CmdRxReset);

	lp->need_to_reset = B_FALSE;

	return (GEM_SUCCESS);
}

static int
tcfe_init_chip(struct gem_dev *dp)
{
	int	i;
	uint32_t	val;
	int	pktsize;
	struct tcfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	pktsize = sizeof (struct ether_header) + 4 + dp->mtu;

	/* Set InterruptMask */
	lp->intenable = 0;
	EXEC_CMD(dp, CmdSetInterruptEnable | lp->intenable);
	EXEC_CMD(dp, CmdAcknowledgeInterrupt | AllAck);

	/* Set IndicationEnable */
	EXEC_CMD(dp, CmdSetIndicationEnable | AllIndications);

	/* setup transmitter */
	EXEC_CMD_WAIT(dp, CmdSetTxStartThresh |
	    min(TxStartThreshMask, dp->txthr / TxStartThreshUnit));

	if (IS_3C90XB_OR_LATER(lp)) {
		/* XXX - use default */
		EXEC_CMD(dp, CmdSetTxReclaimThresh | 8);
	}

	/* setup rx */
	val = ROUNDUP(pktsize, RxEarlyThreshUnit) / RxEarlyThreshUnit;
	EXEC_CMD(dp, CmdSetRxEarlyThresh | min(RxEarlyThreshMask, val));

	/* common and extended registers */

	/* WxTxPktId 0x18 90xB ro */
	/* WxTimer		0x1a */
	/* WxTxStatus		0x1b */
	/* WxIntStatus_1e	0x1e */
	/* WxIntStatusAuto	0x1e */
	/* WxCommand_1e		0x1e */
	/* WxDmaCtrl		0x20 */
	val = INL(dp, WxDmaCtrl);
	val |= DC_upRxEarlyEnable;	/* rw */
	val |= DC_counterSpeed;		/* rw 0:3.2uS, 1:320nS */
	val &= ~DC_countdownMode;	/* rw 0:from now, 1:from intr */
	if (IS_3C90XB_OR_LATER(lp)) {
		val &= ~DC_upAltSeqDisable;	/* rw 90xB/C */
		val &= ~DC_defeatMWI;		/* rw 90xB/C */
		val &= ~DC_defeatMRL;		/* rw 90xB/C */
		val &= ~DC_upOverDiscDisable;	/* rw 90xB/C */
	}
	OUTL(dp, WxDmaCtrl, val);

	/* WxDnListPtr 0x24 */
	EXEC_CMD_WAIT(dp, CmdDnStall);
	OUTL(dp, WxDnListPtr, 0);
	lp->dnlistptr_loaded = B_FALSE;

	/* WxDnBurstThresh 0x2a 90XB/C */
	if (IS_3C90XB_OR_LATER(lp)) {
		val = dp->txmaxdma/DnBurstThreshUnit;
		val = max(1, min(val, DnBurstThreshMask));
		OUTB(dp, WxDnBurstThresh, val);
	}

	/* WxDnPriorityThreash 0x2c */
	if (IS_3C90XB_OR_LATER(lp)) {
		val = TXFIFOSIZE/8/DnPriorityThreshUnit;
		val = max(1, min(val, DnPriorityThreshMask));
		OUTB(dp, WxDnPriorityThresh, val);
	}

	/* WxDnPoll 0x2d */
	if (IS_3C90XB_OR_LATER(lp)) {
#ifdef CONFIG_TX_AUTO_POLL
		OUTB(dp, WxDnPoll, 0x7f);	/* in 320uS */
#else
		OUTB(dp, WxDnPoll, 0);
#endif
	}
	/* WxTxFreeThresh 0x2f (3c90x) */
	if (IS_3C90X(lp)) {
		/*
		 * XXX - 3C90X nics must have enough space in tx fifo
		 * when it starts DMAing a packet, otherwise it may
		 * hang frequently during downloading.
		 */
		val = ROUNDUP(pktsize, TxFreeThreshUnit) / TxFreeThreshUnit;
		OUTB(dp, WxTxFreeThresh, min(TxFreeThreshMask, val));
	}

	/* WxUpPktStatus 0x30, ro */
	/* WxFreeTimer 0x34 */
	/* WxCountdown 0x36, stop */
	OUTW(dp, WxCountdown, 0);

	/*
	 * WxUpListPtr 0x38
	 */
	EXEC_CMD_WAIT(dp, CmdUpStall);

	/* WxUnPriorityThreash 0x3c */
	if (IS_3C90XB_OR_LATER(lp)) {
		val = (RXFIFOSIZE/8)/UpPriorityThreshUnit;
		val = max(1, min(val, UpPriorityThreshMask));
		OUTB(dp, WxUpPriorityThresh, val);
	}

	/* WxUpPoll 0x3d */
	if (IS_3C90XB_OR_LATER(lp)) {
#ifdef CONFIG_RX_AUTO_POLL
		OUTB(dp, WxUpPoll, 0x7f);
#else
		OUTB(dp, WxUpPoll, 0);
#endif
	}

	/* WxUpBurstThresh 0x3e */
	if (IS_3C90XB_OR_LATER(lp)) {
		val = dp->rxmaxdma/UpBurstThreshUnit;
		val = max(1, min(val, UpBurstThreshMask));
		OUTB(dp, WxUpBurstThresh, val);
	}

	/* WxRealTimeCnt 0x40 */
	/* WxDebugData 0x70 */
	/* WxDebugControl 0x74 */
	/* WxDnMaxBurst 0x78, rw, no need to setup */
	/* WxPowerMgmtCtrl 0x7a */

	REG_LOCK(lp);

	/* Window 0 */
	SET_WIN(dp, 0);
	/* W0EepromCommand:0xa, nothing */
	/* W0EepromData:0xc, nothing */

	/* Window 1 */
	SET_WIN(dp, 1);
	/* XXX - all registers in window 1 are obsolute */

	/* Window 2 */
	SET_WIN(dp, 2);
	/* W2StationAddress 0x0, set in set_rx_filter */
	/* W2StationMask 0x6 */
	for (i = 0; i < ETHERADDRL; i += 2) {
		OUTW(dp, W2StationMask + i, 0);
	}
	/* W2ResetOptions 0xc (90xB/C) */
	/* XXX - ResetOptions register will be configured in mii_init. */

	/* Window 3 */
	SET_WIN(dp, 3);
	/* W3InternalConfig offset 0x0 dw, set in reset_chip */
	/* W3MaxPktSize 0x4 w 90XB/C */
	if (IS_3C90XB_OR_LATER(lp)) {
		OUTW(dp, W3MaxPktSize,
		    sizeof (struct ether_header) + 4 + dp->mtu);
	}

	/* W3MacControl 0x6 word */
	val = INW(dp, W3MacControl);
	if (IS_3C90X(lp)) {
		val |= MC_allowLargePackets;
	}
#ifdef NEVER
	if (IS_3C90XB_OR_LATER(lp)) {
		/* we don't use 3com vlan tagging. */
		val |= MC_vltEnable;
	}
#endif
	OUTW(dp, W3MacControl, val);

	/* W3ResetOptions_old 0x8 w, obsolute, 90x */
	/* W3MediaOptions 0x8 w, will be refered in mii_init */
	/* W3RxFree 0xa	w ro */
	/* W3TxFree 0xc	w ro */

	/* Window 4 */
	SET_WIN(dp, 4);
	/* W4FifoDiagnostic 0x4	*/
	/* W4NetworkDiagnostic 0x6 */
	OUTW(dp, W4NetworkDiagnostic,
	    INW(dp, W4NetworkDiagnostic) | ND_upperBytesEnable);

	/* W4PhysicalMgmt 0x8, do nothing */
	/* W4MediaStatus 0xa, configured in mii_init */
	/* W4BadSSD 0xc */
	/* W4UpperBytesOk 0xd, do nothing */

	/* Window 5 */
	SET_WIN(dp, 5);
	/* no registers to manupilate in window 5 */

	/* Window 7 */
	SET_WIN(dp, 7);
	if (IS_3C90XB_OR_LATER(lp)) {
		OUTW(dp, W7VlanEtherType, 0x8100);	/* setup TPID */
	}

	REG_UNLOCK(lp);

	if (IS_3C90XB_OR_LATER(lp)) {
		int	hashsize = ASIC_40_0502_00X(lp) ? 64 : 256;
		/* clear entire multicast hash table */
		for (i = 0; i < hashsize; i++) {
			EXEC_CMD(dp, CmdSetHashFilterBits | i);
		}
		bzero(&lp->mhash[0], sizeof (lp->mhash));
	}

	/* end of hardware initialization */

#ifdef GEM_CONFIG_POLLING
	lp->last_poll_interval = 0;
#endif
	lp->rx_last_slot = 0;
	lp->tx_last_slot = 0;

	return (GEM_SUCCESS);
}

static uint32_t
tcfe_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL));
}

static int
tcfe_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint8_t		*mac;
	uint8_t		mhash[256/8];
	uint16_t	operand;
	int		hashsize;
	uint_t		h;
	struct tcfe_dev	*lp = dp->private;

#if DEBUG_LEVEL > 3
	for (i = 0; i < dp->mc_count; i++) {
		cmn_err(CE_CONT,
		    "!%s: adding mcast(%d)"
		    " %02x:%02x:%02x:%02x:%02x:%02x",
		    dp->name, i,
		    dp->mc_list[i].addr.ether_addr_octet[0],
		    dp->mc_list[i].addr.ether_addr_octet[1],
		    dp->mc_list[i].addr.ether_addr_octet[2],
		    dp->mc_list[i].addr.ether_addr_octet[3],
		    dp->mc_list[i].addr.ether_addr_octet[4],
		    dp->mc_list[i].addr.ether_addr_octet[5]);
	}
#endif

	/* Normal RX operation mode */
	operand = rxFilterIndividual | rxFilterBroadcast;

	if (IS_3C90XB_OR_LATER(lp)) {
		/* we can use a 64 or 256bit-width hardware multicast filter */
		hashsize = ASIC_40_0502_00X(lp) ? 64 : 256;
		bzero(mhash, hashsize/8);
	} else {
		hashsize = 0;
	}

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		operand = 0;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* Promiscous mode */
		operand |= rxFilterAllMulticast | rxFilterPromiscuous;
	} else if (dp->mc_count > hashsize / 2 ||
	    (dp->rxmode & RXMODE_ALLMULTI)) {
		/* All Multicast mode */
		operand |= rxFilterAllMulticast;
	} else if (dp->mc_count > 0) {
		/*
		 * make hash table to select interresting
		 * multicast address only.
		 */
		operand |= rxFilterMulticastHash;
		for (i = 0; i < dp->mc_count; i++) {
			h = dp->mc_list[i].hash & (hashsize - 1);
			mhash[h / 8] |= 1 << (h % 8);
		}
	}

	REG_LOCK(lp);
	SET_WIN(dp, 2);
	mac = &dp->cur_addr.ether_addr_octet[0];
	for (i = 0; i < ETHERADDRL; i += 2) {
		OUTW(dp, W2StationAddress + i, mac[i + 1] << 8 | mac[i]);
	}
	REG_UNLOCK(lp);

	if (hashsize > 0) {
		/* update hardware multicast hash filter */
		for (i = 0; i < hashsize; i++) {
			h = (mhash[i/8] >> (i%8)) & 1;

			if (((lp->mhash[i/8] >> (i%8)) & 1) == h) {
				/* no need to change the bit in hash filter */
				continue;
			}

			EXEC_CMD(dp, CmdSetHashFilterBits | h << 10 | i);
		}
		bcopy(&mhash[0], &lp->mhash[0], hashsize/8);
	}

	EXEC_CMD(dp, CmdSetRxFilter | operand);

	return (GEM_SUCCESS);
}

static int
tcfe_set_media(struct gem_dev *dp)
{
	uint16_t	old;
	uint16_t	new;
	uint32_t	ic;
	struct tcfe_dev	*lp = dp->private;
	extern int	gem_speed_value[];

	DPRINTF(0, (CE_CONT, "!%s: %s speed:%d duplex:%s",
	    dp->name, __func__,
	    gem_speed_value[dp->speed],
	    dp->full_duplex ? "full" : "half"));

	REG_LOCK(lp);
	SET_WIN(dp, 3);

	old = INW(dp, W3MacControl);
	new = old & ~(MC_fullDuplexEnable | MC_flowControlEnable);

	/* duplex mode */
	if (dp->full_duplex) {
		new |= MC_fullDuplexEnable;
	}

	/* flow control */
	if (dp->flow_control == FLOW_CONTROL_SYMMETRIC) {
		new |= MC_flowControlEnable;
	}

	if (new != old) {
		OUTW(dp, W3MacControl, new);
	}
#ifdef WA_905B_10M
	if (lp->media == xcvrSelectAN) {
		/* fix tranceiver selection */
		ic = INL(dp, W3InternalConfig) & ~IC_xcvrSelect;
		if (dp->speed == GEM_SPD_100) {
			ic |= xcvrSelect100BASETX << IC_xcvrSelectShift;
		} else {
			ic |= xcvrSelect10BASET << IC_xcvrSelectShift;
		}
		OUTL(dp, W3InternalConfig, ic);
	}
#endif
	REG_UNLOCK(lp);

	return (GEM_SUCCESS);
}

static int
tcfe_start_chip(struct gem_dev *dp)
{
	struct tcfe_dev	*lp = dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Enable statistics */
	EXEC_CMD(dp, CmdStatisticsEnable);

	/* Enable interrupts */
	lp->intenable = OUR_INTR_BITS;
	EXEC_CMD(dp, CmdSetInterruptEnable | lp->intenable);

	/* Enable transmitter */
	EXEC_CMD(dp, CmdTxEnable);

	/* Enable receiver */
	EXEC_CMD(dp, CmdRxEnable);

	/* Start Dnload DMA engine */
	EXEC_CMD(dp, CmdDnUnstall);

	/* Start Upload DMA engine */
	OUTL(dp, WxUpListPtr, (uint32_t)dp->rx_ring_dma);
	EXEC_CMD(dp, CmdUpUnstall);

	return (GEM_SUCCESS);
}

static int
tcfe_stop_chip(struct gem_dev *dp)
{
	struct tcfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* inhibit interrupt */
	EXEC_CMD(dp, CmdSetInterruptEnable | 0);

	/* no more statistics */
	EXEC_CMD(dp, CmdStatisticsDisable);

	/* stop receiver, then stop rx dma */
	EXEC_CMD(dp, CmdRxDisable);
	EXEC_CMD_WAIT(dp, CmdUpStall);

	/* stop tx dma, then stop transmitter */
	EXEC_CMD_WAIT(dp, CmdDnStall);
	EXEC_CMD(dp, CmdTxDisable);

	/* wait DMA completes */
	drv_usecwait(dp->speed == GEM_SPD_100 ? dp->mtu/10 : dp->mtu);

	if (lp->media == xcvrSelect10BASE2) {
		EXEC_CMD(dp, CmdDisableDcConverter);
		drv_usecwait(800);
	}

	EXEC_CMD_WAIT(dp, CmdRxReset);
	EXEC_CMD_WAIT(dp, CmdTxReset);

	return (GEM_SUCCESS);
}

static int
tcfe_get_stats(struct gem_dev *dp)
{
	int		first_coll;
	int		multi_coll;
	volatile int	x;
	struct tcfe_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	EXEC_CMD(dp, CmdStatisticsDisable);

	REG_LOCK(lp);
	SET_WIN(dp, 6);
	dp->stats.nocarrier 	+= INB(dp, W6CarrierLost);
	x /* heartbeat err */	 = INB(dp, W6SqeErrors);
	multi_coll		 = INB(dp, W6MultipleCollisions);
	dp->stats.multi_coll	+= multi_coll;
	first_coll		 = INB(dp, W6SingleCollisions);
	dp->stats.first_coll	+= first_coll;
	dp->stats.xmtlatecoll	+= INB(dp, W6LateCollisions);
	dp->stats.missed	+= INB(dp, W6RxOverruns);
	x /* dp->stats.opackets */ += INB(dp, W6FramesXmittedOk);
	x /* dp->stats.rpackets */ += INB(dp, W6FramesRcvdOk);
	dp->stats.defer  	+= INB(dp, W6FramesDeferred);
	x /* dp->stats.rbytes */ += INW(dp, W6BytesRcvdOk);
	x /* dp->stats.obytes */ += INW(dp, W6BytesXmittedOk);

	x /* read extended statistics */ = INB(dp, W6UpperFramesOk);
#ifdef notdef
	dp->stats.rpackets	+=
	    ((x & UF_upperFramesRcvdOk) >> UF_upperFramesRcvdOkShift) << 8;
	dp->stats.opackets	+=
	    ((x & UF_upperFramesXmittedOk) >> UF_upperFramesXmittedOkShift)
	    << 8;
#endif
	SET_WIN(dp, 4);
	x			 = INB(dp, W4BadSSD);
	x			 = INW(dp, W4UpperBytesOk);
#ifdef notdef
	dp->stats.rpackets	+=
	    ((x & UF_upperFramesRcvdOk) >> UF_upperFramesRcvdOkShift)
	    << 16;
	dp->stats.opackets	+=
	    ((x & UF_upperFramesXmittedOk) >> UF_upperFramesXmittedOkShift)
	    << 16;
#endif
	REG_UNLOCK(lp);

	/*
	 * Guess total collisions
	 */
	dp->stats.collisions += first_coll + multi_coll*2;

	EXEC_CMD(dp, CmdStatisticsEnable);


	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
tcfe_tx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
	struct DPD_type0	*tfdp;
	struct DnFrag		*tfp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	uint32_t		total;
	struct tcfe_dev		*lp = dp->private;

#if DEBUG_LEVEL > 20
	/* force to cause interrupt upon tx completion */
	flag |= GEM_TXFLAG_INTR;
#endif
	tfdp = (void *)(dp->tx_ring + DN_LIST_SIZE * slot);
#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d flag: %llx",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "%d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif

	/* copy fragment list */
	total = 0;
	mark = DF_dnFragLast; /* last fragment */
	dcp = &dmacookie[frags - 1];
	tfp = &tfdp->frags[frags - 1];
	for (i = frags-1; i >= 0; dcp--, tfp--, i--, mark = 0) {
		uint32_t	len;
		uint32_t	addr;

		total += dcp->dmac_size;
		len = dcp->dmac_size | mark;
		addr = dcp->dmac_address;
		tfp->DnFragLen = LE_32(len);
		tfp->DnFragAddr = LE_32(addr);
	}

	tfdp->DnNextPtr = LE_32(dp->tx_ring_dma +
	    DN_LIST_SIZE * SLOT(slot + 1, TX_RING_SIZE));

	if (flag & GEM_TXFLAG_INTR) {
		total |= FS_dnIndicate;
	}

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if (IS_3C90XB_OR_LATER(lp)) {
		if (flag & GEM_TXFLAG_TCP) {
			total |= FS_addTcpChecksum | FS_addIpChecksum;
		} else if (flag & GEM_TXFLAG_UDP) {
			total |= FS_addUdpChecksum | FS_addIpChecksum;
		} else if (flag & GEM_TXFLAG_IPv4) {
			total |= FS_addIpChecksum;
		}
	}
#endif
	ASSERT((total & FS_dnComplete) == 0);
	tfdp->FrameStartHeader = LE_32(total);

	return (1);
}

static void
tcfe_tx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	struct DPD_type0	*tfdp;
	int			prev_slot;
	uint32_t		head_dma;
	boolean_t		need_restart;
	struct tcfe_dev		*lp = dp->private;
	int			tx_ring_size = dp->gc.gc_tx_ring_size;

	/* fix next pointer in the last descriptor of the new tx list */
	tfdp = (void *)(dp->tx_ring +
	    DN_LIST_SIZE * SLOT(start_slot + nslots - 1, tx_ring_size));
	tfdp->DnNextPtr = 0;
	gem_tx_desc_dma_sync(dp, start_slot, nslots, DDI_DMA_SYNC_FORDEV);

	/* append the new tx descriptors at the tail of current tx list */
	prev_slot = SLOT(start_slot - 1, tx_ring_size);
	tfdp = (void *)(dp->tx_ring + DN_LIST_SIZE * prev_slot);

	head_dma = ((uint32_t)dp->tx_ring_dma) + DN_LIST_SIZE * start_slot;
	tfdp->DnNextPtr = LE_32(head_dma);
	gem_tx_desc_dma_sync(dp, prev_slot, 1, DDI_DMA_SYNC_FORDEV);

	if (!lp->dnlistptr_loaded) {
		/* stop the download engine */
		EXEC_CMD_WAIT(dp, CmdDnStall);

		/* update DnListPtr if download engine has finished */
		if (INL(dp, WxDnListPtr) == 0) {
			OUTL(dp, WxDnListPtr, head_dma);
#ifdef CONFIG_TX_AUTO_POLL
			if (IS_3C90XB_OR_LATER(lp)) {
				lp->dnlistptr_loaded = B_TRUE;
			}
#endif
		}
	}

	/* activate the download engine */
	EXEC_CMD(dp, CmdDnUnstall);

	lp->tx_last_slot = SLOT(start_slot + nslots, tx_ring_size);
}

static void
tcfe_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	struct UPD		*rfdp;
	struct UpFrag		*rfp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		len;
	uint32_t		n;
	uint32_t		addr;
	struct tcfe_dev		*lp = dp->private;

	rfdp = (void *)(dp->rx_ring + UP_LIST_SIZE * slot);

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif

	/* copy fragment list */
	len = 0;
	dcp = dmacookie;
	rfp = rfdp->frags;
	for (i = frags - 1; i--; dcp++, rfp++) {
		len += n = (uint32_t)dcp->dmac_size;
		addr = (uint32_t)dcp->dmac_address;
		rfp->UpFragLen = LE_32(n);
		rfp->UpFragAddr = LE_32(addr);
	}

	/* for last fragment */
	len += n = (uint32_t)dcp->dmac_size;
	n |= UF_upFragLast;
	addr = (uint32_t)dcp->dmac_address;
	rfp->UpFragLen = LE_32(n);
	rfp->UpFragAddr = LE_32(addr);

	/* make an upload header */
	/* XXX - this storing action must be the last for auto polling */
	if (lp->rx_last_slot != slot) {
		rfdp->UpPktStatus = 0;
	}
}

static void
tcfe_rx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	int		i;
	struct UPD	*rfdp;
	struct tcfe_dev	*lp = dp->private;

	if (nslots > 1) {
		gem_rx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, RX_RING_SIZE), nslots - 1,
		    DDI_DMA_SYNC_FORDEV);
	}

	/* enable the first descriptor of the new list */
	rfdp = (void *)(dp->rx_ring + UP_LIST_SIZE * start_slot);
	lp->rx_last_slot = SLOT(start_slot + nslots, RX_RING_SIZE);

	rfdp->UpPktStatus = 0;
	gem_rx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

#ifdef CONFIG_RX_AUTO_POLL
	if (IS_3C90XB_OR_LATER(lp)) {
		/* no need to restart upload engine explicitly */
		return;
	}
#endif
#ifdef NEVER
	/*
	 * XXX - don't stall upload engine. it will make packets dropped.
	 */
	EXEC_CMD_WAIT(dp, CmdUpStall);
	for (i = 0; INW(dp, WxIntStatus) & IS_cmdInProgress; i++) {
		if (i > 200) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout error: upLoad busy",
			    dp->name, __func__);
			lp->need_to_reset = B_TRUE;
			break;
		}
		drv_usecwait(10);
	}
#endif
	/* start possibly stopped upload engine */
	EXEC_CMD(dp, CmdUpUnstall);
}

#if defined(GEM_CONFIG_TX_HEAD_PTR) && defined(CONFIG_TX_AUTO_POLL)
static uint_t
tcfe_tx_desc_head(struct gem_dev *dp)
{
	uint_t		slot;
	uint32_t	cur_pos;
	struct tcfe_dev	*lp = dp->private;

	cur_pos = INL(dp, WxDnListPtr);
	if (cur_pos == 0) {
		return (lp->tx_last_slot);
	}

	slot = (cur_pos - (uint32_t)dp->tx_ring_dma) >> DN_LIST_SIZE_SHIFT;

	/* sanity check */
	if (slot < 0 || slot >= dp->gc.gc_tx_ring_size) {
		lp->need_to_reset = B_TRUE;
		return (lp->tx_last_slot);
	}

	return (slot);
}
#endif /* GEM_CONFIG_TX_HEAD_PTR */
static uint_t
tcfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	int			ret;
	struct DPD_type0	*tfdp;
	uint32_t		header;
	struct tcfe_dev		*lp = dp->private;

#ifdef CONFIG_TX_AUTO_POLL
	if (IS_3C90XB_OR_LATER(lp)) {
		tfdp = (void *)(dp->tx_ring + DN_LIST_SIZE * slot);
		header = tfdp->FrameStartHeader;
		header = LE_32(header);
		return ((header & FS_dnComplete) ? GEM_TX_DONE : 0);
	}
#endif
	ret = 0;
#ifdef CONFIG_STALL_ON_TXSTAT
	EXEC_CMD_WAIT(dp, CmdDnStall);
#endif
	if (INL(dp, WxDnListPtr) !=
	    ((uint32_t)dp->tx_ring_dma) + DN_LIST_SIZE * slot) {
		ret = GEM_TX_DONE;
	}
#ifdef CONFIG_STALL_ON_TXSTAT
	EXEC_CMD(dp, CmdDnUnstall);
#endif
	return (ret);
}

static uint64_t
tcfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct UPD	*rfdp;
	uint32_t	rfs;
	uint64_t	len;
	uint64_t	flag = 0;
	struct tcfe_dev	*lp = dp->private;

	rfdp = (void *)(dp->rx_ring + UP_LIST_SIZE * slot);

	rfs = rfdp->UpPktStatus;
	rfs = LE_32(rfs);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, rfs:0x%b",
	    dp->name, __func__, slot, rfs, UpPktStatusBits));

	if ((rfs & UPS_upComplete) == 0) {
		/* not received */
		return (flag);
	}

	flag = GEM_RX_DONE | (rfs & UPS_upPktLen);

#define	UPS_upErrorVlan	\
	    (UPS_alignmentError | UPS_upOverflow | UPS_runtFrame | \
	    UPS_crcError | UPS_upOverrun)

	if (rfs & UPS_upErrorVlan) {
		/* error packet */

		DPRINTF(4, (CE_CONT,
		    "!%s: %s: slot:%d, rfs:0x%b",
		    dp->name, __func__, slot, rfs, UpPktStatusBits));

		dp->stats.errrcv++;
		if (rfs & UPS_alignmentError) {
			dp->stats.frame++;
		} else if (rfs & UPS_upOverflow) {
			dp->stats.overflow++;
		} else if (rfs & UPS_upOverrun) {
			dp->stats.overflow++;
		} else if (rfs & UPS_runtFrame) {
			dp->stats.runt++;
		} else if (rfs & UPS_crcError) {
			dp->stats.crc++;
		} else if (rfs & UPS_oversizedFrame) {
			dp->stats.frame_too_long++;
		} else {
			dp->stats.rcv_internal_err++;
		}

		flag = GEM_RX_ERR;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	} else if ((rfs & (UPS_tcpChecksumChecked | UPS_tcpChecksumError))
	    == UPS_tcpChecksumChecked) {
		flag |= GEM_RX_CKSUM_TCP;
	} else if ((rfs & (UPS_udpChecksumChecked | UPS_udpChecksumError))
	    == UPS_udpChecksumChecked) {
		flag |= GEM_RX_CKSUM_UDP;
	} else if ((rfs & (UPS_ipChecksumChecked | UPS_ipChecksumError))
	    == UPS_ipChecksumChecked) {
		flag |= GEM_RX_CKSUM_IPv4;
#endif
	}

	return (flag);
}

static void
tcfe_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct DPD_type0	*tfdp;

	tfdp = (void *)(dp->tx_ring + DN_LIST_SIZE * slot);
	bzero(tfdp, DN_LIST_SIZE);
}

static void
tcfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct UPD	*rfdp;
	uint32_t	next_dma;

	next_dma = ((uint32_t)dp->rx_ring_dma) +
	    UP_LIST_SIZE * SLOT(slot + 1, RX_RING_SIZE);
	rfdp = (void *)(dp->rx_ring + UP_LIST_SIZE * slot);

	rfdp->UpPktStatus = LE_32(UPS_upComplete);
	rfdp->UpNextPtr = LE_32(next_dma);
}

static void
tcfe_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct DPD_type0	*tfdp;

	tfdp = (void *)(dp->tx_ring + DN_LIST_SIZE * slot);
	bzero(tfdp, DN_LIST_SIZE);
}

static void
tcfe_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct UPD	*rfdp;

	rfdp = (void *)(dp->rx_ring + UP_LIST_SIZE * slot);

	rfdp->UpPktStatus = LE_32(UPS_upComplete);
}

/*
 * Device depend interrupt handler
 */
static void
tcfe_get_tx_status(struct gem_dev  *dp)
{
	int		error = 0;
	uint8_t		tx_status;
	struct tcfe_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "%s: %s: called", dp->name, __func__));

	/* Clear Tx status stack. */
	while (INW(dp, WxIntStatus) & IS_txComplete) {

		tx_status = INB(dp, WxTxStatus);
		OUTB(dp, WxTxStatus, 0);

		if (tx_status & TS_txStatusOverflow) {
			cmn_err(CE_WARN,
			    "%s: tx status overflow: tx_status: %b",
			    dp->name, tx_status, TxStatusBits);
			dp->stats.xmit_internal_err++;
			dp->stats.errxmt++;
		}

		/* update stats */
		if (tx_status & TS_txUnderrun) {
			/* fifo underflow */
			dp->stats.underflow++;
			dp->stats.errxmt++;
		}

		if (tx_status & TS_maxCollisions) {
			/* exceed maximum collisions */
			dp->stats.excoll++;
			dp->stats.collisions += 16;
			dp->stats.errxmt++;
		}

		if (tx_status & TS_txJabber) {
			dp->stats.xmit_internal_err++;
			dp->stats.errxmt++;
		}

		/* collect status bits */
		error |= (tx_status &
		    (TS_txJabber | TS_txUnderrun | TS_maxCollisions));
	}

	if (error) {
		if (error & (TS_txUnderrun | TS_txJabber)) {
			/*
			 * XXX - not implemented completely
			 */
			EXEC_CMD_WAIT(dp, CmdTxReset);
			lp->need_to_reset = B_TRUE;
		}

		if (error & TS_maxCollisions) {
			EXEC_CMD(dp, CmdStatisticsEnable);
		}

		EXEC_CMD(dp, CmdTxEnable);
	}
}

static uint_t
tcfe_interrupt(struct gem_dev *dp)
{
	uint16_t	status;
	uint_t		tx_sched = 0;
	struct tcfe_dev	*lp = dp->private;

	/* Read interrupt status */
	status = INW(dp, WxIntStatus);
	DPRINTF(3, (CE_CONT, "%s: %s, status: %b",
	    dp->name, __func__, status, IntStatusBits));

	if ((status & lp->intenable) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "%s: %s, status: %b",
	    dp->name, __func__, status, IntStatusBits));

	/*
	 * Clear all interrupt mask and ack interrupt,
	 */
	EXEC_CMD(dp, CmdAcknowledgeInterrupt | (status & AllAck));

	if (!dp->mac_active) {
		/*
		 * the device is not active.
		 * side effect: left interrupt masked.
		 */
		lp->intenable = 0;
		EXEC_CMD(dp, CmdSetInterruptEnable | lp->intenable);
		return (DDI_INTR_CLAIMED);
	}

	status &= OUR_INTR_BITS;

#ifdef GEM_CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval) {
			/* move to polling mode */
			lp->intenable &= ~IS_upComplete;
		} else {
			/* return to normal mode */
			lp->intenable |= IS_upComplete;
		}

		/*
		 * To schedule the next timer interrupt,
		 * we pretend as we were interrupted from
		 * polling timer
		 */
		status |= IS_intRequested;

		lp->last_poll_interval = dp->poll_interval;

		if ((dp->misc_flag & GEM_NOINTR) == 0) {
			/* change interrupt mask */
			EXEC_CMD(dp, CmdSetInterruptEnable | lp->intenable);
		}
	}

	if (status & IS_intRequested) {
		/*
		 * Reset timer counter to schedule the next polling
		 * timer interrupt.
		 */
		OUTW(dp, WxCountdown, dp->poll_interval / 320);
	}
#endif /* GEM_CONFIG_POLLING */

	if (status & IS_rxEarly) {
		cmn_err(CE_WARN, "%s: unexpected RX_EARLY, status: %b",
		    dp->name, status, IntStatusBits);
	}

	if (status & IS_upComplete) {
		/* Receive Complete */
		(void) gem_receive(dp);
	}

	if (status & IS_dnComplete) {
		if (gem_tx_done(dp)) {
			tx_sched = INTR_RESTART_TX;
		}
	}

	if (status & IS_txComplete) {
		tcfe_get_tx_status(dp);
	}

	if (status & IS_updateStats) {
		/* Update Statistics */
		tcfe_get_stats(dp);
	}

	if (status & IS_hostError) {
		lp->need_to_reset = B_TRUE;
	}

	/*
	 * Postamble for interrupt thread
	 */
x:
	if (lp->need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		tx_sched = INTR_RESTART_TX;
	}

	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "!%s: %s done: status: %b",
	    dp->name, __func__, INW(dp, WxIntStatus), IntStatusBits));

	return (DDI_INTR_CLAIMED | tx_sched);
}
/*
 * HW depend MII routine
 */
#define	MDIO_DELAY(dp)	{INW(dp, W4PhysicalMgmt); INW(dp, W4PhysicalMgmt); }
static void
tcfe_mii_sync(struct gem_dev *dp)
{
	int		i;
	struct tcfe_dev	*lp = dp->private;

	/* output 32 ones */
	REG_LOCK(lp);
	SET_WIN(dp, 4);
	for (i = 0; i < 32; i++) {
		OUTW(dp, W4PhysicalMgmt, PM_mgmtDir | PM_mgmtData);
		MDIO_DELAY(dp);
		OUTW(dp, W4PhysicalMgmt,
		    PM_mgmtDir | PM_mgmtData | PM_mgmtClk);
		MDIO_DELAY(dp);
	}
	REG_UNLOCK(lp);
}

static uint16_t
tcfe_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint16_t	data;
	boolean_t	error = B_FALSE;
	struct tcfe_dev	*lp = dp->private;

	cmd = MII_READ_CMD(dp->mii_phy_addr, reg);

	REG_LOCK(lp);
	SET_WIN(dp, 4);

	for (i = 31; i >= 18; i--) {
		data = ((cmd >> i) & 1) <<  PM_mgmtDataShift;
		OUTW(dp, W4PhysicalMgmt, data | PM_mgmtDir);
		MDIO_DELAY(dp);
		OUTW(dp, W4PhysicalMgmt, data | PM_mgmtDir | PM_mgmtClk);
		MDIO_DELAY(dp);
	}
	/* turn around */
	OUTW(dp, W4PhysicalMgmt, 0);
	MDIO_DELAY(dp);

	/* get response from PHY */
	OUTW(dp, W4PhysicalMgmt, PM_mgmtClk);
	MDIO_DELAY(dp);
	OUTW(dp, W4PhysicalMgmt, 0);
	if (INW(dp, W4PhysicalMgmt) & PM_mgmtData) {
		DPRINTF(2, (CE_CONT, "%s: no response from phy@%d",
		    dp->name, dp->mii_phy_addr));
		error = B_TRUE;
	}

	OUTW(dp, W4PhysicalMgmt, PM_mgmtClk);
	MDIO_DELAY(dp);

	for (i = 16; i > 0; i--) {
		OUTW(dp, W4PhysicalMgmt, 0);
		ret = (ret << 1) |
		    ((INW(dp, W4PhysicalMgmt) >> PM_mgmtDataShift) & 1);
		OUTW(dp, W4PhysicalMgmt, PM_mgmtClk);
		MDIO_DELAY(dp);
	}

	/* send 2 ones to phy */
	for (i = 0; i < 2; i++) {
		OUTW(dp, W4PhysicalMgmt,
		    PM_mgmtDir | PM_mgmtData);
		MDIO_DELAY(dp);
		OUTW(dp, W4PhysicalMgmt,
		    PM_mgmtDir | PM_mgmtData | PM_mgmtClk);
		MDIO_DELAY(dp);
	}

	if (lp->media == xcvrSelectAN && reg == MII_STATUS) {
		if (INW(dp, W4MediaStatus) & MS_linkDetect) {
			ret |= MII_STATUS_LINKUP;
		}
	}

	REG_UNLOCK(lp);

	if (error) {
		ret = 0;
	}
	DPRINTF(10, (CE_CONT, "%s:%s: reg:%d: val:0x%04x",
	    dp->name, __func__, reg, ret));
	return (ret);
}

static void
tcfe_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;
	uint32_t	cmd;
	uint16_t	data;
	struct tcfe_dev	*lp = dp->private;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, reg, val);

	REG_LOCK(lp);
#ifdef WA_905B_10M
	if (lp->media == xcvrSelectAN && reg == MII_CONTROL) {
		uint32_t	ic;

		/* fix tranceiver selection */
		SET_WIN(dp, 3);
		ic = INL(dp, W3InternalConfig) & ~IC_xcvrSelect;
		ic |= xcvrSelectAN << IC_xcvrSelectShift;
		OUTL(dp, W3InternalConfig, ic);
	}
#endif
	SET_WIN(dp, 4);
	for (i = 31; i >= 0; i--) {
		data = ((cmd >> i) & 1) << PM_mgmtDataShift;
		OUTW(dp, W4PhysicalMgmt,
		    data | PM_mgmtDir);
		MDIO_DELAY(dp);
		OUTW(dp, W4PhysicalMgmt,
		    data | PM_mgmtDir | PM_mgmtClk);
		MDIO_DELAY(dp);
	}

	/* send 2 ones to phy */
	for (i = 0; i < 2; i++) {
		OUTW(dp, W4PhysicalMgmt,
		    PM_mgmtDir | PM_mgmtData);
		MDIO_DELAY(dp);
		OUTW(dp, W4PhysicalMgmt,
		    PM_mgmtDir | PM_mgmtData | PM_mgmtClk);
		MDIO_DELAY(dp);
	}
	REG_UNLOCK(lp);
}
#undef MDIO_DELAY

static void
tcfe_nomii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
tcfe_nomii_read(struct gem_dev *dp, uint_t index)
{
	uint16_t	ret;
	struct tcfe_dev	*lp = dp->private;

	ret = 0;
	switch (index) {
	case MII_CONTROL:
		ret = lp->bmcr;
		break;

	case MII_STATUS:
		/* TODO: should return proper link status */
		/* setup capability */
		ret = lp->bmsr;

		switch (lp->media) {
		case xcvrSelectAUI:
		case xcvrSelect10BASE2:
			/* no way to detect link up */
			ret |= MII_STATUS_LINKUP;
			break;

		default:
			REG_LOCK(lp);
			SET_WIN(dp, 4);
			if (INW(dp, W4MediaStatus) & MS_linkDetect) {
				ret |= MII_STATUS_LINKUP;
			}
			REG_UNLOCK(lp);
			break;
		}
		break;

	case MII_AN_ADVERT:
		ret = lp->adv;
		break;
	}
	return (ret);
}

static void
tcfe_nomii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	struct tcfe_dev    *lp = dp->private;

	switch (index) {
	case MII_CONTROL:
		lp->bmcr = val & ~MII_CONTROL_RESET;
		break;

	case MII_AN_ADVERT:
		lp->adv = val;
		break;

	default:
		cmn_err(CE_WARN,
		    "!%s: %s: writing to register %d in phy isn't permitted.",
		    dp->name, __func__, index);
		break;
	}
}

static char *tcfe_media_name[] = {
	"10BASET",		/* 0 */
	"AUI",			/* 1 */
	"(reserved)",		/* 2 */
	"10BASE2",		/* 3 */
	"100BASETX",		/* 4 */
	"100BASEFX",		/* 5 */
	"MII",			/* 6 */
	"(reserved)",		/* 7 */
	"10/100Mbps w/ AN",	/* 8 */
	"external MII",		/* 9 */
	"(reserved)",		/* 10 */
	"(reserved)",		/* 11 */
	"(reserved)",		/* 12 */
	"(reserved)",		/* 13 */
	"(reserved)",		/* 14 */
	"(reserved)",		/* 15 */
};

static int
tcfe_mii_probe(struct gem_dev *dp)
{
	uint32_t	ic;
	uint16_t	mopt;
	uint16_t	ms;
	uint32_t	port;
	struct tcfe_dev	*lp = dp->private;

	/*
	 * select media here
	 */

	/* read internal config register */
	REG_LOCK(lp);

	/*
	 * choose media port
	 */
	SET_WIN(dp, 3);
	mopt = INW(dp, W3MediaOptions);
	cmn_err(CE_CONT, "!%s: media options:%b",
	    dp->name, mopt,
	    IS_3C90X(lp) ? MediaOptionsBits : MediaOptionsBits_BC);

	ic = INL(dp, W3InternalConfig);

	DPRINTF(0, (CE_CONT, "!%s: %s: internal_config:%b",
	    dp->name, __func__, ic, InternalConfigBits));

	if (ic & IC_autoSelect) {
		/* scan available media */
		if (mopt & MO_baseT4Available) {
			lp->media = xcvrSelectMII;
			/* XXX - should we use 100M full forced mode ? */
		} else
		if (IS_3C90XB_OR_LATER(lp) &&
		    (mopt & (MO_baseTxAvailable | MO_10bTAvailable))) {
			lp->media = xcvrSelectAN;
		} else if (mopt & MO_baseTxAvailable) {
			lp->media = xcvrSelect100BASETX;
		} else if (mopt & MO_baseFxAvailable) {
			lp->media = xcvrSelect100BASEFX;
		} else if ((mopt & MO_10BaseFL) && IS_3C90XB(lp)) {
			lp->media = xcvrSelectAUI;
		} else if (mopt & MO_10bTAvailable) {
			lp->media = xcvrSelect10BASET;
		} else if (mopt & MO_miiDevice) {
			lp->media = xcvrSelectMII;
		} else {
			lp->media = xcvrSelect10BASET;
		}

		/* fix tranceiver selection */
		ic = (ic & ~IC_xcvrSelect) |
		    (lp->media << IC_xcvrSelectShift);
	} else {
		lp->media = (ic & IC_xcvrSelect) >> IC_xcvrSelectShift;
	}

	OUTL(dp, W3InternalConfig, ic);
	lp->ic = ic;

	cmn_err(CE_CONT, "!%s: media %s[%d] selected, ic:0x%b",
	    dp->name, tcfe_media_name[lp->media], lp->media,
	    ic, InternalConfigBits);

	/*
	 * setup media
	 */
	SET_WIN(dp, 4);
	ms = INW(dp, W4MediaStatus);

	DPRINTF(0, (CE_CONT,
	    "!%s: ms:0x%b", dp->name, ms, MediaStatusBits));

	ms &= ~(MS_enableSqeStats | MS_jabberGuardEnable | MS_linkBeatEnale);
	switch (lp->media) {
	case xcvrSelectMII:
	case xcvrSelectAN:
		/* use MII with autonegotiation capability */
		dp->mii_phy_addr = 0;
		if (lp->media == xcvrSelectAN) {
			dp->mii_phy_addr = 24;
		}
		if (IS_3C90XC(lp)) {
			ms |= MS_linkBeatEnale;
		}
		break;

	case xcvrSelect10BASET:
		ms |= MS_jabberGuardEnable | MS_linkBeatEnale;
		lp->bmsr = MII_STATUS_10 | MII_STATUS_10_FD;
		goto non_mii;

	case xcvrSelectAUI:
		ms |= MS_enableSqeStats;
		lp->bmsr = MII_STATUS_10;
		goto non_mii;

	case xcvrSelect10BASE2:
		lp->bmsr = MII_STATUS_10;
		goto non_mii;

	case xcvrSelect100BASETX:
	case xcvrSelect100BASEFX:
		ms |= MS_linkBeatEnale;
		lp->bmsr = MII_STATUS_100_BASEX | MII_STATUS_100_BASEX_FD;
		goto non_mii;

	non_mii:
		dp->gc.gc_mii_sync = &tcfe_nomii_sync;
		dp->gc.gc_mii_read = &tcfe_nomii_read;
		dp->gc.gc_mii_write = &tcfe_nomii_write;

		/* no need to scan phy */
		dp->mii_phy_addr = -1;
		break;

	default:
	case xcvrSelectMIIExt:
		REG_UNLOCK(lp);
		cmn_err(CE_WARN,
		    "!%s: unknown media in InternalConfig: media:%d",
		    dp->name, lp->media);
		return (GEM_FAILURE);
	}

	SET_WIN(dp, 4);
	OUTW(dp, W4MediaStatus, ms);
	lp->ms = ms;

	REG_UNLOCK(lp);

	lp->mii_initialized = B_TRUE;

	return (gem_mii_probe_default(dp));
}

static int
tcfe_mii_init(struct gem_dev *dp)
{
	struct tcfe_dev	*lp = dp->private;

	if (lp->mii_initialized) {
		return (GEM_SUCCESS);
	}

	REG_LOCK(lp);

	SET_WIN(dp, 3);
	OUTL(dp, W3InternalConfig, lp->ic);

	SET_WIN(dp, 4);
	OUTW(dp, W4MediaStatus, lp->ms);

	REG_UNLOCK(lp);

	return (GEM_SUCCESS);
}

static int
tcfe_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;
	uint16_t	eecmd;
	uint16_t	val;
	struct tcfe_dev	*lp = dp->private;

	/* window should be write-locked */
	eecmd = offset;

	if (lp->chip->flags & EEADDRSIZE_8) {
		eecmd |= (EE_ReadRegister << (8 - 2));
	} else {
		eecmd |= (EE_ReadRegister << (6 - 2));
	}

	REG_LOCK(lp);
	SET_WIN(dp, 0);
	OUTW(dp, W0EepromCommand, eecmd);
	drv_usecwait(162);

	for (i = 0; (INW(dp, W0EepromCommand) & EC_eepromBusy); i++) {
		if (i > 20) {
			REG_UNLOCK(lp);
			cmn_err(CE_WARN,
			    "!%s: %s: timeout, offset 0x%x, flags 0x%x",
			    dp->name, __func__, offset, lp->chip->flags);
			return (0);
		}
		drv_usecwait(10);
	}

	val = INW(dp, W0EepromData);
	REG_UNLOCK(lp);

	return (val);
}

#ifdef DEBUG_LEVEL
static void
tcfe_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	*prom;
	uint8_t		sum;
	int		epsize;
	struct tcfe_dev	*lp = dp->private;

	epsize = 0x21;
	if (IS_CARDBUS(lp)) {
		epsize = 0x80;
	}
	/* allocate temporary buffer for reading eeprom contents */
	prom = kmem_alloc(epsize * sizeof (uint16_t), KM_SLEEP);

	for (i = 0; i < epsize; i++) {
		prom[i] = tcfe_read_eeprom(dp, i);
	}
	/* calculate checksum for eeprom area */
	sum = 0;
	for (i = 0; i < 0x20; i++) {
		uint16_t	val;
		val = prom[i + lp->chip->ee_offset];
		sum ^= (val >> 8) ^ val;
		DPRINTF(10, (CE_CONT, "!%s: %x: sum:%x", __func__, i, sum));
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < epsize; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
	cmn_err(CE_CONT, "!sum[00-1f]: 0x%04x", sum);

	kmem_free(prom, epsize * sizeof (uint16_t));
}
#endif /* DEBUG_LEVEL */

#if 0
#include "tcfe_eeprom_wr.c"
#endif

static int
tcfe_attach_chip(struct gem_dev *dp)
{
	int		i;
	int		val;
	uint8_t		*mac;
	struct tcfe_dev	*lp = dp->private;
	static uint8_t	zero6[] = {0, 0, 0, 0, 0, 0,};
	static uint8_t	bcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff,};

#if 0
	tcfe_fix_eeprom905b(dp);
#endif

#if DEBUG_LEVEL > 1
	tcfe_eeprom_dump(dp);
#endif

#ifdef NEVER
	/* fix eeprom contents */
	val = tcfe_read_eeprom(dp,
	    EO_SoftwareIntormation + lp->chip->ee_offset);

	tcfe_exec_eeprom_cmd(dp, EE_WriteEnable, 0, 0);
	tcfe_exec_eeprom_cmd(dp, EE_EraseRegister,
	    EO_SoftwareIntormation + lp->chip->ee_offset, 0);

	tcfe_exec_eeprom_cmd(dp, EE_WriteEnable, 0, 0);
	tcfe_exec_eeprom_cmd(dp, EE_WriteRegister,
	    EO_SoftwareIntormation + lp->chip->ee_offset,
	    val | 0x8000);

	tcfe_exec_eeprom_cmd(dp, EE_WriteDisable, 0, 0);
#endif /* NEVER */
#if DEBUG_LEVEL > -1
	tcfe_eeprom_dump(dp);
#endif

	mac = &dp->dev_addr.ether_addr_octet[0];
	for (i = 0; i < ETHERADDRL; i += 2) {
		val = tcfe_read_eeprom(dp,
		    EO_OEMNodeAddress + i/2 + lp->chip->ee_offset);
		if (val < 0) {
			return (GEM_FAILURE);
		}
		/* the station address is stored in network byteorder */
		mac[i + 0] = (uint8_t)(val >> 8);
		mac[i + 1] = (uint8_t)val;
	}

	if (bcmp(mac, zero6, ETHERADDRL) == 0 || (mac[0] & 0x1) ||
	    bcmp(mac, bcast, ETHERADDRL) == 0) {
		gem_generate_macaddr(dp, mac);
	}

	dp->txmaxdma = max(dp->txmaxdma, 512);
	dp->rxmaxdma = max(dp->rxmaxdma, 512);

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if (IS_3C90XB_OR_LATER(lp)) {
		dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4;
	}
#endif
#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	dp->misc_flag |= GEM_POLL_RXONLY;

	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
tcfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			n;
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	uint16_t		comm;
	int			ret;
	int			vid;
	int			did;
	int			unit;
	struct chip_info	*p;
	int			val;
	int			len;
	struct pci_phys_spec	*regs;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct tcfe_dev		*lp;
	caddr_t			base;
	caddr_t			base2;
	ddi_acc_handle_t	regs_handle;
	ddi_acc_handle_t	regs2_handle;
	struct gem_conf		*gcp;
	uint8_t			revid;
	uint8_t			lat;
	uint32_t		iline;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "%s%d: tcfeattach: called", drv_name, unit));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "%s: ddi_regs_map_setup failed", drv_name);
		goto err;
	}
	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	comm = pci_config_get16(conf_handle, PCI_CONF_COMM);
	lat = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
	iline = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	comm |= PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE;
	pci_config_put16(conf_handle, PCI_CONF_COMM, comm);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		dp = GEM_GET_DEV(dip);
		lp = dp->private;
		lp->mii_initialized = B_FALSE;
		return (gem_resume(dip));

	case DDI_ATTACH:
		for (i = 0, p = tcfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (p->venid == vid && p->devid == did) {
				/* found */
				cmn_err(CE_CONT,
				    "!%s%d: %s (vid: 0x%04x, did: 0x%04x,"
				    " revid: 0x%02x, latency timer: 0x%02x)",
				    drv_name, unit, p->name,
				    vid, did, revid, lat);
				goto chip_found;
			}
		}

		/* Not found */
		cmn_err(CE_WARN,
		    "%s: tcfe_attach: wrong PCI venid/devid (0x%x, 0x%x)",
		    drv_name, vid, did);
		p--;
chip_found:
		DPRINTF(2, (CE_CONT,
		    "%s: %s: iline %08x", drv_name, __func__, iline));
		/*
		 * Map in the device registers.
		 */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_IO, PCI_ADDR_MASK,
		    &tcfe_dev_attr,
		    &base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		regs2_handle = NULL;
		if (p->flags & CHIP_CARDBUS) {
			if (gem_pci_regs_map_setup(dip,
			    PCI_CONF_BASE2, PCI_REG_REG_M,
			    &tcfe_dev_attr,
			    &base2, &regs2_handle)
			    != DDI_SUCCESS) {
				ddi_regs_map_free(&regs_handle);
				goto err;
			}
		}

		lp = kmem_zalloc(sizeof (struct tcfe_dev), KM_SLEEP);
		lp->rev_id = revid;
		lp->chip = p;
		lp->base2 = base2;
		lp->base2_ha = regs2_handle;

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift = DN_LIST_SIZE_SHIFT; /* 64 byte */
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = tcfe_tx_copy_thresh;
#ifndef GEM_CONFIG_TX_DIRECT
		gcp->gc_tx_desc_write_oo = (TX_BUF_SIZE == TX_RING_SIZE);
#endif

		gcp->gc_rx_buf_align	= sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags	= MAXRXFRAGS;
		gcp->gc_rx_desc_unit_shift = UP_LIST_SIZE_SHIFT; /* 32 byte */
		gcp->gc_rx_ring_size	= RX_RING_SIZE;
		gcp->gc_rx_buf_max	= gcp->gc_rx_ring_size;
		gcp->gc_rx_copy_thresh	= tcfe_rx_copy_thresh;
		gcp->gc_rx_header_len	= 0;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = tcfe_dev_attr;
		gcp->gc_buf_attr = tcfe_buf_attr;
		gcp->gc_desc_attr = tcfe_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = tcfe_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = tcfe_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		gcp->gc_dma_attr_rxbuf = tcfe_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
#ifdef WA_905B_10M
		/* not worked, link up but ping failed */
		gcp->gc_mii_an_oneshot = B_TRUE;
#endif

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &tcfe_attach_chip;
		gcp->gc_reset_chip = &tcfe_reset_chip;
		gcp->gc_init_chip = &tcfe_init_chip;
		gcp->gc_start_chip = &tcfe_start_chip;
		gcp->gc_stop_chip = &tcfe_stop_chip;
		gcp->gc_multicast_hash = &tcfe_mcast_hash;
		gcp->gc_set_rx_filter = &tcfe_set_rx_filter;
		gcp->gc_set_media = &tcfe_set_media;
		gcp->gc_get_stats = &tcfe_get_stats;
		gcp->gc_interrupt = &tcfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &tcfe_tx_desc_write;
		gcp->gc_tx_start = &tcfe_tx_start;
		gcp->gc_rx_desc_write = &tcfe_rx_desc_write;
		gcp->gc_rx_start = &tcfe_rx_start;

		gcp->gc_tx_desc_init = &tcfe_tx_desc_init;
		gcp->gc_rx_desc_init = &tcfe_rx_desc_init;
#if defined(GEM_CONFIG_TX_HEAD_PTR) && defined(CONFIG_TX_AUTO_POLL)
		if (IS_3C90XB_OR_LATER(lp)) {
			gcp->gc_tx_desc_stat = &tcfe_tx_desc_stat;
		} else {
			gcp->gc_tx_desc_head = &tcfe_tx_desc_head;
		}
#else
		gcp->gc_tx_desc_stat = &tcfe_tx_desc_stat;
#endif
		gcp->gc_rx_desc_stat = &tcfe_rx_desc_stat;
		gcp->gc_tx_desc_clean = &tcfe_tx_desc_clean;
		gcp->gc_rx_desc_clean = &tcfe_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_probe = &tcfe_mii_probe;
		gcp->gc_mii_init = &tcfe_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &tcfe_mii_sync;
		gcp->gc_mii_read = &tcfe_mii_read;
		gcp->gc_mii_write = &tcfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		mutex_init(&lp->reglock, NULL, MUTEX_DRIVER, NULL);
		mutex_init(&lp->cmdlock, NULL, MUTEX_DRIVER, NULL);

		/* offload and jumbo frame */
		gcp->gc_max_lso = 0;
		gcp->gc_max_mtu = TCFE_MAX_PACKET_SIZE -
		    (sizeof (struct ether_header) + ETHERFCSL + 4);
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip,
		    0, gcp, base, &regs_handle, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

		mutex_destroy(&lp->reglock);
		mutex_destroy(&lp->cmdlock);
err_free_mem:
		kmem_free(lp, sizeof (struct tcfe_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
tcfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;
	struct gem_dev  *dp;
	struct tcfe_dev   *lp;

	dp = GEM_GET_DEV(dip);
	lp = dp->private;

	switch (cmd) {
	case DDI_DETACH:
		if (lp->base2_ha != NULL) {
			ddi_regs_map_free(&lp->base2_ha);
		}
		mutex_destroy(&lp->reglock);
		mutex_destroy(&lp->cmdlock);
		return (gem_do_detach(dip));

	case DDI_SUSPEND:
		return (gem_suspend(dip));
	}

	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef GEM_CONFIG_GLDv3
GEM_STREAM_OPS(tcfe_ops, tcfeattach, tcfedetach);
#else
static	struct module_info tcfeminfo = {
	0,			/* mi_idnum */
	"tcfe",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit tcferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&tcfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit tcfewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&tcfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	tcfe_info = {
	&tcferinit,	/* st_rdinit */
	&tcfewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_tcfe_ops = {
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
	&tcfe_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops tcfe_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	tcfeattach,	/* devo_attach */
	tcfedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_tcfe_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,
	ident,
	&tcfe_ops,
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

	DPRINTF(2, (CE_CONT, "!tcfe: _init: called"));

	gem_mod_init(&tcfe_ops, "tcfe");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&tcfe_ops);
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

	DPRINTF(2, (CE_CONT, "!tcfe: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&tcfe_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
