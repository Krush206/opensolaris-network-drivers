/*
 *  icpt: IC Plus IP1000A GbE MAC driver for Solaris
 *  @(#)icpt_gem.c	1.4 07/02/19
 *
 * Copyright (c) 2005-2007 Masayuki Murayama.  All rights reserved.
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
 Change log
03/20/2006
	macro nests releated on USE_FRAMEID fixed
	it caused compile time error with old gcc version.
11/04/2006
	changes for gem 2.4.x framework

02/18/2007
	2.4.0 released
 */

/*
 TODO:
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
#include "ip1000reg.h"

char	ident[] = "ip1000a nic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int icpt_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (icpt_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	OUTL_HI(dp, reg, val)	OUTW(dp, ((reg)+2), (((uint32_t)(val)) >> 16))
#define	OUTL_LOW(dp, reg, val)	OUTW(dp, reg, ((uint16_t)(val)))

#if defined(i86pc)
#define	LE64(x)	(x)
#else
#define	LE64(x)	ddi_swap64(x)
#endif

/*
 * configuration on IP1000A TFD and RFD structures.
 */

#ifndef IPGE_MAXTXFRAGS
#  define	IPGE_MAXTXFRAGS	6
#endif

struct icpt_tfd {
	volatile uint64_t	TxDMANextPtr;
	volatile uint64_t	TxFrameControl;
#if IPGE_MAXTXFRAGS <= 2
	volatile uint64_t	TxDMAFrag[2];
#elif IPGE_MAXTXFRAGS <= 6
	volatile uint64_t	TxDMAFrag[6];
#else
# error IPGE_MAXTXFRAGS must be less than or equal to 6.
#endif
};

struct icpt_rfd {
	volatile uint64_t	RxDMANextPtr;
	volatile uint64_t	RxFrameStatus;
	volatile uint64_t 	RxDMAFrag;
	volatile uint64_t	pad;	/* software padding for gem v2 */
};

struct icpt_dev {
	boolean_t	need_to_reset;
	boolean_t	tx_list_loaded;
	uint32_t	asic_ctrl;
	uint8_t		rev_id;
	uint16_t	intmask;
	uint32_t	countdown;
	int		rx_tail;
#ifdef USE_FRAMEID
	uint_t		frameid_tail;
#endif
#ifdef GEM_CONFIG_POLLING
	int		last_poll_interval;
#endif
#ifdef TEST_TXTIMEOUT
	int		send_cnt;
#endif
};

/*
 * Our configuration
 */
#define	OUR_INTR_BITS \
	(INT_HostError | INT_TxComplete | INT_UpdateStats | \
	 INT_TxDMAComplete | INT_RxDMAComplete | INT_RFDListEnd | \
	 INT_LinkEvent)

#ifndef TX_RING_SIZE
# define TX_RING_SIZE	256
#endif
#ifndef TX_BUF_SIZE
# define TX_BUF_SIZE	TX_RING_SIZE
#endif

#ifndef RX_RING_SIZE
# define RX_RING_SIZE	64
#endif
#ifndef RX_BUF_SIZE
# define RX_BUF_SIZE	RX_RING_SIZE
#endif

#define	ONESEC			(drv_usectohz(1*1000000))

static int	icpt_tx_copy_thresh = 256;
static int	icpt_rx_copy_thresh = 256;

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info icpt_chiptbl[] = {
	{PCI_VID_DLINK,		0x4020,	"D-Link DL-2000"},
	{PCI_VID_SUNDANCE,	0x1023,	"IC Plus ip1000a"},
};
#define CHIPTABLESIZE   (sizeof(icpt_chiptbl)/sizeof(struct chip_info))

/* ======================================================== */
 
/* mii operations */
static void  icpt_mii_sync(struct gem_dev *);
static uint16_t icpt_mii_read(struct gem_dev *, uint_t);
static void icpt_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int icpt_attach_chip(struct gem_dev *);
static int icpt_reset_chip(struct gem_dev *);
static int icpt_init_chip(struct gem_dev *);
static int icpt_start_chip(struct gem_dev *);
static int icpt_stop_chip(struct gem_dev *);
static int icpt_set_media(struct gem_dev *);
static int icpt_set_rx_filter(struct gem_dev *);
static int icpt_get_stats(struct gem_dev *);

/* descriptor operations */
static int icpt_tx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag);
static void icpt_tx_start(struct gem_dev *dp, int slot, int nslot);
static void icpt_rx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t icpt_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t icpt_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void icpt_tx_desc_init(struct gem_dev *dp, int slot);
static void icpt_rx_desc_init(struct gem_dev *dp, int slot);
static void icpt_tx_desc_clean(struct gem_dev *dp, int slot);
static void icpt_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int icpt_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr icpt_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr icpt_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t icpt_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x000000ffffffffffull,	/* dma_attr_addr_hi */
	0x000000000000ffffull,	/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0x000007fc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000000000000ffffull,	/* dma_attr_maxxfer */
	0x000000ffffffffffull,	/* dma_attr_seg */
	0,/* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t icpt_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	64,			/* dma_attr_addr_lo */
	0x000000ffffffffffull,	/* dma_attr_addr_hi */
	0x00000000ffffffffull,	/* dma_attr_count_max */
	64,			/* dma_attr_align */
	0x000007fc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x00000000ffffffffull,	/* dma_attr_maxxfer */
	0x000000ffffffffffull,	/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */

static boolean_t
icpt_reset_asic(struct gem_dev *dp, uint32_t cmd, int timeo)
{
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	/* Reset the chip. */
	OUTL_HI(dp, AsicCtrl, cmd | lp->asic_ctrl);

	for (; (INL(dp, AsicCtrl) & AC_ResetBusy) != 0; timeo -= 10) {
		if (timeo < 0) {
			return (B_FALSE);
		}
		drv_usecwait(10);
	}

	return (B_TRUE);
}

static int
icpt_reset_chip(struct gem_dev *dp)
{
	struct icpt_dev	*lp = (struct icpt_dev *) dp->private;
	uint32_t	cmd;

	/* Disable all interrupts */
	OUTW(dp, IntEnable, lp->intmask = 0);

	/* Reset the chip. */
	cmd = AC_GlobalReset | AC_RxReset | AC_TxReset | AC_DMA
	    | AC_FIFO | AC_Network | AC_Host | AC_AutoInit;

	if (!lp->tx_list_loaded) {
		 /* This is first call after initialization. Reset phy too. */
		cmd |= AC_RstOut;
	}

	if (!icpt_reset_asic(dp, cmd, 2000)) {
		cmn_err(CE_WARN, "%s: %s: timeout", dp->name, __func__);
		return (GEM_FAILURE);
	}

	return (GEM_SUCCESS);
}

static int
icpt_init_chip(struct gem_dev *dp)
{
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(lp->intmask == 0);

	/* tx list ptr */
	OUTL(dp, TFDListPtr + 0, 0);
	OUTL(dp, TFDListPtr + 4, 0);
	lp->tx_list_loaded = B_FALSE;

	/* rx list ptr */
	OUTL(dp, RFDListPtr + 0, 0);
	OUTL(dp, RFDListPtr + 4, 0);

	/* AsicCtrl : assume the chip stopped */
	DPRINTF(3, (CE_CONT, "!%s: %s: AsicCtrl: %b", dp->name, __func__,
		INL(dp, AsicCtrl), AC_BITS));

	/* keep LED control bits and rd/wr bits */
	lp->asic_ctrl =
		(lp->asic_ctrl & 0xffff0000) | (INL(dp, AsicCtrl) & 0xffff);
	OUTL(dp, AsicCtrl, lp->asic_ctrl);

	/* MACCtrl: */
	DPRINTF(3, (CE_CONT, "!%s %s called, mc: %b",
		dp->name, __func__, INL(dp, MACCtrl), MC_BITS));

	/* Ensure all functions are disabled */
	OUTL_HI(dp, MACCtrl,
			MC_StatisticsDisable | MC_TxDisable | MC_RxDisable);
#ifdef	CONFIG_RCV_FCS
	OUTL_LOW(dp, MACCtrl, MC_RcvFCS | MC_IFSSelect802_3);
#else
	OUTL_LOW(dp, MACCtrl, MC_IFSSelect802_3);
#endif

#define	RxFIFOSIZE	(32*1024)			/* 32Kbyte */
#define	RxFIFO_HIWAT	(RxFIFOSIZE*16/32)		/* 16Kbyte */
#define	RxFIFO_LOWAT	(RxFIFOSIZE*1/32)		/*  1Kbyte */

	OUTW(dp, MaxFrameSize, dp->mtu + sizeof(struct ether_header));
	OUTW(dp, RxEarlyThresh, RxEarlyThreshMax);

	OUTW(dp, TxStartThresh,
		min((dp->txthr + 4) / TxStartThreshUnit, TxStartThreshMax));

	/* disable hardware rx interrupt coaleasing */
	OUTL(dp, RxDMAIntCtrl, RIC_RxDMAWaitTime | RIC_PriorityThreshMax | 1);

	OUTB(dp, RxDMAPollPeriod, RxDMAPollPeriodMin);
	OUTB(dp, RxDMAUrgentThresh,
		max(min((RxFIFOSIZE - RxFIFO_HIWAT) / RxDMAUrgentThreshUnit,
			RxDMAUrgentThreshMax), RxDMAUrgentThreshMin));
	OUTB(dp, RxDMABurstThresh,
		max(min(dp->rxthr / RxDMABurstThreshUnit,
			RxDMABurstThreshMax), RxDMABurstThreshMin));

	OUTB(dp, TxDMAPollPeriod, TxDMAPollPeriodMax); /* 81.6 uS */

	/* TxDMAUrgentThreash : 1/16 of TxStartThresh */
	OUTB(dp, TxDMAUrgentThresh,
		max(min((dp->txthr / 16) / TxDMAUrgentThreshUnit,
			TxDMAUrgentThreshMax), TxDMAUrgentThreshMin));

	OUTB(dp, TxDMABurstThresh,
		max(min(dp->txmaxdma / TxDMABurstThreshUnit,
			TxDMABurstThreshMax), TxDMABurstThreshMin));

	/* DMACtrl: */
#ifdef USE_FRAMEID
	OUTL_LOW(dp, DMACtrl, DC_TxWriteBackDisable | DC_TxBurstLimitMax);
#else
	OUTL_LOW(dp, DMACtrl, DC_TxBurstLimitMax);
#endif

	/* Tx flow control threshold */
	OUTW(dp, FlowOnThresh, RxFIFO_HIWAT / FlowThreshUnit);
	OUTW(dp, FlowOffThresh, RxFIFO_LOWAT / FlowThreshUnit);

	/* DebugCtrl: workarounds */
	OUTW(dp, DebugCtrl, 0x0200 | INL(dp, DebugCtrl));
	OUTW(dp, DebugCtrl, 0x0010 | INL(dp, DebugCtrl));
	OUTW(dp, DebugCtrl, 0x0020 | INL(dp, DebugCtrl));

	/* XXX - clear Countdown timer */
#if 0
	DPRINTF(0, (CE_CONT, "!%s: Countdown: 0x%x",
		dp->name, INL(dp, Countdown)));
	OUTL(dp, Countdown, 0);
#endif
	/* Setup statistics masks */
	OUTL(dp, RMONStatisticsMask, 0);
	OUTL(dp, StatisticsMask, 0);
#ifdef USE_FRAMEID
	lp->frameid_tail = 0;
#endif
	lp->rx_tail = 0;

	DPRINTF(2, (CE_CONT, "!%s: %s: mc: %b, ac: %b",
		dp->name, __func__,
		INL(dp, MACCtrl), MC_BITS, INL(dp, AsicCtrl), AC_BITS));

	return (GEM_SUCCESS);
}

static int
icpt_start_chip(struct gem_dev *dp)
{
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Enable statistics */
	OUTL_HI(dp, MACCtrl, MC_StatisticsEnable);

	/* Set media mode */
	icpt_set_media(dp);

	ASSERT((dp->rx_ring_dma & 7) == 0);
	OUTL(dp, RFDListPtr + 0, (uint32_t) dp->rx_ring_dma);
	OUTL(dp, RFDListPtr + 4, (uint32_t) (dp->rx_ring_dma >> 32));

	/* Enable transmitter and receiver */
	OUTL_HI(dp, MACCtrl, MC_TxEnable | MC_RxEnable);

	/* Set InterruptMask */
	lp->intmask = OUR_INTR_BITS;

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IntEnable, lp->intmask);
	}

	DPRINTF(0, (CE_CONT, "!%s: %s: mc: %b",
		dp->name, __func__, INL(dp, MACCtrl), MC_BITS));
	DPRINTF(0, (CE_CONT, "!%s: %s: dmactrl: %b",
		dp->name, __func__, INL(dp, DMACtrl), DC_BITS));

	return (GEM_SUCCESS);
}

static int
icpt_stop_chip(struct gem_dev *dp)
{
	int		i;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: icpt_stop_chip: called", dp->name));

	/* inhibit interrupt */
	OUTW(dp, IntEnable, lp->intmask = 0);

	/* no more statistics */
	OUTL_HI(dp, MACCtrl, MC_StatisticsDisable);

	/* stop receiver and transceiver */
	OUTL_HI(dp, MACCtrl, MC_RxDisable | MC_TxDisable);
	drv_usecwait(2000);

	/* reset DMA pointers */
	if (!icpt_reset_asic(dp, AC_RxReset | AC_TxReset | AC_DMA, 2000)) {
		cmn_err(CE_WARN, "%s: %s: timeout: resetting tx/rx",
			dp->name, __func__);
	}

	return (GEM_SUCCESS);
}

static uint16_t
icpt_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;

	OUTW(dp, EepromCtrl, EEC_OpcodeRd | (offset & EEC_Address));
	drv_usecwait(100);

	for (i = 0; (INW(dp, EepromCtrl) & EEC_Busy) != 0; i++) {
		if (i > 30) {
			cmn_err(CE_WARN,
				"%s: %s: timeout", dp->name, __func__);
			return (0xffff);
		}
		drv_usecwait(10);
	}

	DPRINTF(4, (CE_CONT, "%s: %s: took %d uS", dp->name, __func__, i*10));

	return (INW(dp, EepromData));
}

#ifdef DEBUG_LEVEL
static void
icpt_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	prom[0x14];

	for (i = 0; i < 0x14; i++) {
		prom[i] = icpt_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 0x14; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x", 
			i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
}
#endif /* DEBUG_LEVEL */

static int
icpt_attach_chip(struct gem_dev *dp)	
{
	int		i;
	uint_t		val;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	for (i = 0; i < ETHERADDRL; i += 2) {
		val = icpt_read_eeprom(dp, 0x10 + i/2);

		dp->dev_addr.ether_addr_octet[i + 0] = (uint8_t) val;
		dp->dev_addr.ether_addr_octet[i + 1] = (uint8_t) (val >> 8);

		OUTW(dp, StationAddress + i, val);
	}

	/* setup LED control bits in offset 6 of AsicCtrl */
	lp->asic_ctrl = 0;
	val = icpt_read_eeprom(dp, 6);
	if ((val & 0x01) != 0) {
		lp->asic_ctrl |= AC_LEDMode;
	}
	if ((val & 0x02) != 0) {
		lp->asic_ctrl |= AC_LEDMode1;
	}
	if ((val & 0x08) != 0) {
		lp->asic_ctrl |= AC_LEDSpeed;
	}

#if DEBUG_LEVEL > 4
	icpt_eeprom_dump(dp);
#endif

	/* Minimum burst size is 256 byte */
	dp->txmaxdma = max(dp->txmaxdma,
				TxDMABurstThreshMin * TxDMABurstThreshUnit);
	dp->rxthr = dp->rxmaxdma =
		max(max(dp->rxmaxdma, dp->rxthr),
				RxDMABurstThreshMin * RxDMABurstThreshUnit);
#ifdef TXUNDERRUNTEST
	dp->txthr = min(256, dp->txthr);
#endif
#ifdef CONFIG_VLAN
        dp->misc_flag |= GEM_VLAN_SOFT;
#endif
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
        dp->misc_flag |= (GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4);
#endif
	return (GEM_SUCCESS);
}

static uint32_t
icpt_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL));
}

static int
icpt_set_rx_filter(struct gem_dev *dp)	
{
	int		i;
	uint16_t	mode;
	uint8_t		*mac;
	uint32_t	mhash[2];

	ASSERT(mutex_owned(&dp->intrlock));
	bzero(mhash, sizeof(mhash));

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	}
	else if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		/* promiscous */
		mode = RM_ReceiveAllFrames;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 || dp->mc_count > 32) {
		mode = RM_ReceiveUnicast
		     | RM_ReceiveBroadcast
		     | RM_ReceiveMulticast;
	}
	else {
		/* Normal mode */
		mode = RM_ReceiveUnicast | RM_ReceiveBroadcast;
		if (dp->mc_count > 0) {
			mode |= RM_ReceiveMulticastHash;
		}

		/* Make multicast hash table */
		for (i = 0; i < dp->mc_count; i++) {
			uint_t	k;
			/* hash key is 6 bits of LSB in be-crc */
			k = dp->mc_list[i].hash & ((1 << 6) - 1);
			mhash[k / 32] |= 1 << (k % 32);
		}
	}

	/* set rx mode */
	DPRINTF(2, (CE_CONT, "!%s: %s: setting rx mode: 0x%b",
		dp->name, __func__, mode, RM_BITS));

	/* stop rx temporary */
	if (dp->mac_active) {
		OUTW(dp, ReceiveMode, (mode != 0) ? RM_ReceiveAllFrames : 0);
	}

	if ((mode & RM_ReceiveMulticastHash) != 0) {
		/* set multicast hash table up */
		for (i = 0; i < sizeof(mhash)/sizeof(uint32_t); i++) {
			OUTL(dp, HashTable + i*sizeof(uint32_t), mhash[i]);
		}
	}

	/* set station address */
	mac = dp->cur_addr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL; i += 2) {
		OUTW(dp, StationAddress + i, (mac[i + 1] << 8) | mac[i]);
	}

	OUTW(dp, ReceiveMode, mode);

	return (GEM_SUCCESS);
}

static int
icpt_set_media(struct gem_dev *dp)
{
	uint16_t	old;
	uint16_t	new;
	int		i;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/*
	 * Notify current duplex mode to MAC
	 */
	old = INW(dp, MACCtrl);
	new = old & ~(MC_DuplexSelect |
		      MC_TxFlowControlEnable | MC_RxFlowControlEnable);

	if (dp->full_duplex) {
		new |= MC_DuplexSelect;

		switch (dp->flow_control) {
		case FLOW_CONTROL_SYMMETRIC:
			new |= MC_TxFlowControlEnable | MC_RxFlowControlEnable;
			break;

		case FLOW_CONTROL_TX_PAUSE:
			new |= MC_TxFlowControlEnable;
			break;

		case FLOW_CONTROL_RX_PAUSE:
			new |= MC_RxFlowControlEnable;
			break;
		}
	}

	if (old != new) {
		OUTL_LOW(dp, MACCtrl, new);
	}

	if (((old ^ new) & MC_DuplexSelect) != 0) {
		/*
		 * Duplex mode was changed. Need to reset Tx/Rx.
		 */
		DPRINTF(1, (CE_CONT,
			"!%s: %s: resetting tx and rx to change duplex mode",
			dp->name, __func__));

		if (!icpt_reset_asic(dp, AC_TxReset | AC_RxReset, 2000)) {
			/* Time out */
			cmn_err(CE_WARN, "%s: %s: timeout: resetting tx/rx",
				dp->name, __func__);
		}

		OUTL_HI(dp, MACCtrl, MC_TxEnable | MC_RxEnable);
	}

	return (GEM_SUCCESS);
}

static int
icpt_get_stats(struct gem_dev *dp)
{
	int		first_coll;
	int		multi_coll;
	int		c;
	int		l;
	volatile int	x;

	DPRINTF(4, (CE_CONT, "!%s: icpt_get_stats: called", dp->name));
#ifdef NEVER
	ASSERT(mutex_owned(&dp->intrlock));
#endif
	OUTL_HI(dp, MACCtrl, MC_StatisticsDisable);

				x= INL(dp, OctetRcvdOk);
				x= INL(dp, McstOctetRcvdOk);
				x= INL(dp, BcstOctetRcvdOk);
				x= INL(dp, FramesRcvdOk);
				x= INL(dp, McstFramesRcvdOk);

				x= INW(dp, BcstFramesRcvdOk);
				x= INW(dp, MacControlFramesRcvd);
				x= INW(dp, FrameTooLongErrors);
				x= INW(dp, InRangeLengthErrors);
				x= INW(dp, FramesCheckSeqErrors);
	dp->stats.missed        += INW(dp, FramesLostRxErrors);

				x= INL(dp, OctetXmtOk);
				x= INL(dp, McstOctetXmtOk);
				x= INL(dp, BcstOctetXmtOk);
				x= INL(dp, FramesXmtdOk);
				x= INL(dp, McstFramesXmtdOk);
	dp->stats.defer         += INL(dp, FramesWDeferredXmt);
	dp->stats.xmtlatecoll += l = INL(dp, LateCollisions);
	multi_coll               = INL(dp, MultiColFrames);
	first_coll               = INL(dp, SingleColFrames);

				x= INW(dp, BcstFramesXmtdOk);
	dp->stats.nocarrier += c = INW(dp, CarrierSenseErrors);
				x= INW(dp, MacControlFramesXmtd);
				x= INW(dp, FramesAbortXSColls);
				x= INW(dp, FramesWEXDeferral);

	dp->stats.multi_coll    += multi_coll;
	dp->stats.first_coll    += first_coll;

	/*
	 * Guess total collisions
	 */
	dp->stats.collisions += first_coll + multi_coll*2;

	dp->stats.errxmt += c + l;

	OUTL_HI(dp, MACCtrl, MC_StatisticsEnable);

	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
icpt_tx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag)
{
	int			i;
	uint64_t		mark;
	uint64_t		vtag;
	struct icpt_tfd		*tfdp;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

#if DEBUG_LEVEL > 2
	/* force to cause interrupt upon tx completion */
	flag |= GEM_TXFLAG_INTR;
#endif
	tfdp = &((struct icpt_tfd *) dp->tx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"%s: %s: seqnum: %d, slot %d, frags: %d flag: %d",
		dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "%d: addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	tfdp->TxDMANextPtr = dp->tx_ring_dma +
			sizeof(struct icpt_tfd) * SLOT(i + 1, TX_RING_SIZE);

	/* make fragment list */
	for (i = 0; i < frags; i++) {
		tfdp->TxDMAFrag[i] = LE64(
		    (((uint64_t) dmacookie[i].dmac_size) << TFI_FragLenShift) |
		    dmacookie[i].dmac_laddress);
	}

	mark = (uint64_t) slot
	     | (((uint64_t) frags) << TFC_FragCountShift)
	     | TFC_WordAlignDisable;

	if ((flag & GEM_TXFLAG_INTR) != 0) {
#ifdef USE_FRAMEID
		mark |= TFC_TxIndicate;
#else
		mark |= TFC_TxDMAIndicate;
#endif
#ifdef TEST_CRCERR
		mark |= TFC_FcsAppendDisable;
#endif
	}

#ifdef CONFIG_VLAN
	/* XXX - it's funny */
	if ((vtag = ((flag & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT)) != 0) {
		mark |= TFC_VLANTagInsert | (vtag << 32);
	}
#endif

#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if ((flag & GEM_TXFLAG_IPv4) != 0) {
		mark |= TFC_IPCksumEn;
	}
	if ((flag & GEM_TXFLAG_TCP) != 0) {
		mark |= TFC_TCPCksumEn;
	}
	else if ((flag & GEM_TXFLAG_UDP) != 0) {
		mark |= TFC_UDPCksumEn;
	}
#endif
	tfdp->TxFrameControl = LE64(mark);

	return (1);
}

static void
icpt_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int		last;
	uint64_t	tfd_dma;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	/* write zero into  NextPtr in the last slot to terminate tx list */
	last = SLOT(start_slot + nslot - 1, TX_RING_SIZE);
	((struct icpt_tfd *) dp->tx_ring) [last].TxDMANextPtr = 0ULL;

	/* flush tx descriptors we have made */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

#ifdef TEST_TXTIMEOUT
	if ((lp->send_cnt++ % 10000) == 9999) {
		OUTL_HI(dp, MACCtrl, MC_TxDisable);
	}
#endif
	tfd_dma = dp->tx_ring_dma + sizeof(struct icpt_tfd) * start_slot;

	if (lp->tx_list_loaded)  {
		/*
		 * Link this to the previous slot. The previous isn't in use,
		 * because the number of tx buffers is smaller than the
		 * number of tx descriptors.
		 */
		last = SLOT(start_slot - 1, TX_RING_SIZE);
		((struct icpt_tfd *)dp->tx_ring)[
				last].TxDMANextPtr = LE64(tfd_dma);

		/* flush tx descriptors we have made */
		gem_tx_desc_dma_sync(dp, last, 1, DDI_DMA_SYNC_FORDEV);

		/* Make the nic poll tx immediately */
#ifdef GEM_CONFIG_POLLING
		if (dp->poll_interval == 0)
#endif
		{
			/* kick tx engine right now */
#ifdef USE_FRAMEID
			OUTL_LOW(dp, DMACtrl,
				DC_TxWriteBackDisable |
				DC_TxBurstLimitMax | DC_TxDMAPollNow);
#else
			OUTL_LOW(dp, DMACtrl,
				DC_TxBurstLimitMax | DC_TxDMAPollNow);
#endif
		}
	}
	else {
		/*
		 * This is first call after initialization.
		 * Need to tell the head of tx list to the nic.
		 */
		OUTL_LOW(dp, DMACtrl, DC_TxBurstLimitMax);

		OUTL(dp, TFDListPtr + 0, (uint32_t) tfd_dma);
		OUTL(dp, TFDListPtr + 4, (uint32_t) (tfd_dma >> 32));
		lp->tx_list_loaded = B_TRUE;
	}
}

static void
icpt_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int		i;
	struct icpt_rfd	*rfdp;
	struct icpt_dev	*lp = (struct icpt_dev *) dp->private;

	rfdp = &((struct icpt_rfd *)dp->rx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT, "%s: %s seqnum: %d, slot %d, frags: %d",
		dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "  frag: %d addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	ASSERT(frags == 1);

	rfdp->RxDMAFrag = LE64(
		(((uint64_t) dmacookie->dmac_size) << RFI_FragLenShift) |
		dmacookie->dmac_address);

	if (slot != lp->rx_tail) {
		rfdp->RxFrameStatus = 0ULL;
	}
}

static void
icpt_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct icpt_dev	*lp = (struct icpt_dev *) dp->private;

	if (nslot > 1) {
		gem_rx_desc_dma_sync(dp, SLOT(start_slot + 1, RX_RING_SIZE),
				nslot, DDI_DMA_SYNC_FORDEV);
	}

	((struct icpt_rfd *)
		dp->rx_ring)[start_slot].RxFrameStatus = 0ULL;

	gem_rx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	lp->rx_tail = start_slot + nslot;
}

static uint_t
icpt_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
#ifdef USE_FRAMEID
	uint32_t	tx_status;
	struct icpt_dev	*lp = (struct icpt_dev *) dp->private;

	/* first, we try to compare the slot with the last frame id read. */
	if (slot != lp->frameid_tail) {
		return (GEM_TX_DONE);
	}

	/* update frame id */
	tx_status = INL(dp, TxStatus);
	lp->frameid_tail =
		SLOT(((tx_status & TS_TxFrameId) >> TS_TxFrameIdShift) + 1,
				TX_RING_SIZE);

	return ((slot != lp->frameid_tail) ? GEM_TX_DONE : 0);
#else
	uint64_t	tfc;

	tfc = LE64(((struct icpt_tfd *) dp->tx_ring)[slot].TxFrameControl);

	DPRINTF(2, (CE_CONT, "%s: %s: slot:%d, tfs:0x%b",
		dp->name, __func__, slot, (uint32_t) tfc, TFC_BITS));

	if ((tfc & TFC_TFDDone) == 0) {
		/* not transmitted yet */
		return (0);
	}
	return (GEM_TX_DONE);
#endif
}

static uint64_t
icpt_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint64_t		rfs;
	uint64_t		len;
	uint64_t		flag;

	rfs  = LE64(((struct icpt_rfd *) dp->rx_ring)[slot].RxFrameStatus);
	len  = rfs & RFS_RxFrameLen;
	flag = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT, "%s: %s: slot:%d, rfs:0x%b",
		dp->name, __func__, slot, (uint32_t) rfs, RFS_BITS));

	if ((rfs & RFS_RFDDone) == 0) {
		/* not received */
		return (0);
	}

	/* XXX - RuntFrame bit unreliable */
#define	RFS_ERROR_BITS	\
	(RFS_RxFIFOOverrun |/*RFS_RxRuntFrame |*/RFS_RxAlignmentError |	\
	 RFS_RxFCSError | RFS_RxOversizedFrame | RFS_RxLengthError |	\
	 RFS_FrameStart | RFS_FrameEnd)

	if ((rfs & RFS_ERROR_BITS) != (RFS_FrameStart | RFS_FrameEnd)) {
		/* error packet */
		dp->stats.errrcv++;
		if ((rfs & RFS_RxAlignmentError) != 0) {
			dp->stats.frame++;
		}
		if ((rfs & RFS_RxFIFOOverrun) != 0) {
			dp->stats.overflow++;
		}
#ifdef notdef
		if ((rfs & RFS_RxRuntFrame) != 0) {
			dp->stats.runt++;
		}
#endif
		if ((rfs & RFS_RxFCSError) != 0) {
			dp->stats.crc++;
		}
		if ((rfs & (RFS_RxLengthError | RFS_RxOversizedFrame)) != 0) {
			dp->stats.frame_too_long++;
		}

		flag = GEM_RX_ERR;
	}
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if ((rfs & (RFS_TCPDetected | RFS_TCPError)) == RFS_TCPDetected) {
		flag |= GEM_RX_CKSUM_TCP;
	}
	if ((rfs & (RFS_UDPDetected | RFS_UDPError)) == RFS_UDPDetected) {
		flag |= GEM_RX_CKSUM_UDP;
	}
	if ((rfs & (RFS_IPDetected | RFS_IPError)) == RFS_IPDetected) {
		flag |= GEM_RX_CKSUM_IPv4;
	}
#endif
#ifdef notdef /* CONFIG_VLAN */
	/* the hardware doesn't seem to implement hardware vlan detection */
	if ((rfs & RFS_VLANDetected) != 0) {
		flag |= (rfs & RFS_TCI) >> (RFS_TCIShift - GEM_RX_VTAG_SHIFT);
	}
#endif
#ifdef CONFIG_RCV_FCS
	len = max(len - ETHERFCSL, 0);
#endif
	return (flag | len);
}

static void
icpt_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* invalidate the tfd */
	((struct icpt_tfd *)
		dp->tx_ring)[slot].TxFrameControl = LE64(TFC_TFDDone);
}

static void
icpt_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct icpt_rfd		*rfdp;

	/* invalidate this rfd */
	((struct icpt_rfd *)
		dp->rx_ring)[slot].RxFrameStatus = LE64(RFS_RFDDone);

	/* link it to the previous rfd */
	rfdp = &((struct icpt_rfd *)dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)];
	rfdp->RxDMANextPtr =
		LE64(dp->rx_ring_dma + slot*sizeof(struct icpt_rfd));
}

static void
icpt_tx_desc_clean(struct gem_dev *dp, int slot)
{
	/* invalidate the tfd */
	((struct icpt_tfd *)
		dp->tx_ring)[slot].TxFrameControl = LE64(TFC_TFDDone);
}

static void
icpt_rx_desc_clean(struct gem_dev *dp, int slot)
{
	/* invalidate the rfd */
	((struct icpt_rfd *)
		dp->rx_ring)[slot].RxFrameStatus = LE64(RFS_RFDDone);
}

/*
 * Device depend interrupt handler
 */
static void
icpt_get_tx_status(struct gem_dev  *dp)
{
	int		i;
	uint32_t	tx_status;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Pop up all status from the Tx status stack. */
	if (((tx_status = INL(dp, TxStatus)) & TS_TxComplete) == 0) {
		/* tx status is invalid */
		return;
	}

	/* update statistics */

	if ((tx_status & TS_TxError) != 0) {
		dp->stats.errxmt++;
	}

	if ((tx_status & TS_TxUnderrun) != 0) {
		/* fifo underflow. TxReset and TxEnable are required */
		cmn_err(CE_WARN,
		    "%s: tx underrun error: tx_status: %b",
		    dp->name, tx_status, TS_BITS);
		dp->stats.underflow++;
	}

	if ((tx_status & TS_MaxCollisions) != 0) {
		/* exceed maximum collisions */
		dp->stats.excoll++;
		dp->stats.collisions += 16;
	}

	if ((tx_status & TS_LateCollision) != 0) {
		/* Out of Window collision detected */
		dp->stats.xmtlatecoll++;
	}

	mutex_enter(&dp->xmitlock);
#ifdef USE_FRAMEID
	lp->frameid_tail =
		SLOT(((tx_status & TS_TxFrameId) >> TS_TxFrameIdShift) + 1,
			TX_RING_SIZE);
#endif
	if ((tx_status &
	    (TS_TxUnderrun | TS_MaxCollisions | TS_LateCollision)) != 0) {
		if ((tx_status & TS_TxUnderrun) != 0) {
			/* a fatal error */
#ifdef notdef
			/* XXX - the nic has stopped */
			OUTL_HI(dp, MACCtrl, MC_TxDisable);
#endif
			lp->need_to_reset = B_TRUE;
			/* increase tx threashold by 128 byte */
			dp->txthr = min(dp->txthr + 128,
					dp->mtu + sizeof(struct ether_header));
			cmn_err(CE_NOTE, "%s: tx error, 0x%b",
				dp->name, tx_status, TS_BITS);
		}
		else {
			OUTL_HI(dp, MACCtrl, MC_TxEnable); 
		}
	}
	mutex_exit(&dp->xmitlock);
}

static u_int
icpt_interrupt(struct gem_dev *dp)
{
	uint16_t	int_status;
	u_int		tx_sched = 0;
	struct icpt_dev	*lp = (struct icpt_dev *)dp->private;

	/*
	 * Read and reset interrupt status.
	 */
	int_status = INW(dp, IntStatusAck);

	if ((int_status & lp->intmask) == 0) {
		/* Not for us */
		if (lp->intmask != 0 && (dp->misc_flag & GEM_NOINTR) == 0) {
			/*
			 * As reading IntStatusAck also reset IntEnable,
			 * we need to enable it again.
			 */
			OUTW(dp, IntEnable, lp->intmask);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "%s: %s, int_status: %b, intmask: %b",
		dp->name, __func__,
		int_status, INT_BITS, lp->intmask, INT_BITS));

	if (!dp->mac_active) {
		/*
		 * the device is not active.
		 * side effect: left interrupt masked.
		 */
		return (DDI_INTR_CLAIMED);
	}

	int_status &= lp->intmask;

#ifdef GEM_CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		uint32_t	wait;
		uint32_t	rxcnt;

		if (dp->poll_interval != 0) {
			rxcnt = dp->poll_pkt_delay;
			wait = min(dp->poll_interval * RxDMAWaitTime_1uS,
					0xffff) << RIC_RxDMAWaitTimeShift;
		}
		else {
			rxcnt = 1;
			wait = RIC_RxDMAWaitTime;
		}

		OUTL(dp, RxDMAIntCtrl, wait | RIC_PriorityThreshMax | rxcnt);

		lp->last_poll_interval = dp->poll_interval;
	}
#endif

	if ((int_status &
	    (INT_RxDMAComplete | INT_MACControlFrame | INT_RFDListEnd)) != 0) {
		/*
		 * packet was received, or receive error happened
		 */
		(void) gem_receive(dp);
		if ((int_status & INT_RFDListEnd) != 0) {
			/* kick rx dma engine again */
			OUTL_LOW(dp, DMACtrl, DC_RxDMAPollNow);
		}
	}

	if ((int_status &
		(INT_TxComplete | INT_TxDMAComplete | INT_IntRequested)) != 0) {

		if ((int_status & INT_TxComplete) != 0) {
			/*
			 * Need to pull up TxStatus before calling tx_desc_stat
			 */
			icpt_get_tx_status(dp);
		}

		/*
		 * Packets was transfered into TxFIFO or
		 * an error happened.
		 */
		if (gem_tx_done(dp)) {
			tx_sched = INTR_RESTART_TX;
		}
	}

	if ((int_status & (INT_UpdateStats | INT_LinkEvent |
			   INT_IntRequested | INT_HostError)) != 0) {

		if ((int_status & INT_UpdateStats) != 0) {
			/*
			 * Statistics counter overflow
			 */
			icpt_get_stats(dp);
			DPRINTF(0, (CE_CONT, "%s: %s UpdateStats",
				dp->name, __func__));
		}

		if ((int_status & INT_LinkEvent) != 0) {
			DPRINTF(0, (CE_CONT, "%s: %s link event",
				dp->name, __func__));
			if (gem_mii_link_check(dp)) {
				tx_sched = INTR_RESTART_TX;
			}
		}

		if ((int_status & (INT_IntRequested | INT_HostError)) != 0) {
			cmn_err(CE_WARN, "%s: unexpected interrupt: %b",
				dp->name, int_status, INT_BITS);
		}
	}

	if (lp->need_to_reset) { 
		DPRINTF(0, (CE_CONT, "%s: resetting...", dp->name));
		gem_restart_nic(dp, B_TRUE);
		tx_sched = INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "%s: %s done: int_status: %b",
		   dp->name, __func__, INW(dp, IntStatus), INT_BITS));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IntEnable, lp->intmask);
	}

	return (DDI_INTR_CLAIMED | tx_sched);
}

/*
 * HW depend MII routine
 */
#define MDIO_DELAY(dp)    {INB(dp, PhyCtrl); INB(dp, PhyCtrl);}

static void
icpt_mii_sync(struct gem_dev *dp)
{
	int	i;

	/* output 32 ones */
	for (i = 0; i < 32; i++) {
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
	OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
}

static uint16_t
icpt_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint8_t		data;

	cmd = MII_READ_CMD(dp->mii_phy_addr, reg);

	for (i = 31; i >= 18; i--) {
		data = ((cmd >> i) & 1) <<  PC_MgmtDataShift;
		OUTB(dp, PhyCtrl, data | PC_MgmtDir);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, data | PC_MgmtDir | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
	/* turn around */
	OUTB(dp, PhyCtrl, 0);
	MDIO_DELAY(dp);

	/* get response from PHY */
	OUTB(dp, PhyCtrl, PC_MgmtClk);
	MDIO_DELAY(dp);
	OUTB(dp, PhyCtrl, 0);
	if ((INB(dp, PhyCtrl) & PC_MgmtData) != 0) {
		DPRINTF(1, (CE_CONT, "%s: no response from phy@%d",
			dp->name, dp->mii_phy_addr));
	}

	OUTB(dp, PhyCtrl, PC_MgmtClk);
	MDIO_DELAY(dp);

	for (i = 16; i > 0; i--) {
		OUTB(dp, PhyCtrl, 0);
		ret = (ret << 1) | ((INB(dp, PhyCtrl) >> PC_MgmtDataShift) & 1);
		OUTB(dp, PhyCtrl, PC_MgmtClk);
		MDIO_DELAY(dp);
	}

	/* send two 1s to phy */
	for (i = 0; i < 2; i++) {
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
	OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);

	return (ret);
}

static void
icpt_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;
	uint32_t	cmd;
	uint8_t		data;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, reg, val);

	for (i = 31; i >= 0; i--) {
		data = ((cmd >> i) & 1) << PC_MgmtDataShift;
		OUTB(dp, PhyCtrl, data | PC_MgmtDir);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, data | PC_MgmtDir | PC_MgmtClk);
		MDIO_DELAY(dp);
	}

	/* send two 1s to phy */
	for (i = 0; i < 2; i++) {
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, PC_MgmtDir  | PC_MgmtData | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
	OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
}
#undef MDIO_DELAY

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
icptattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			n;
	ddi_acc_handle_t	conf_handle;
	int			vid;
	int			did;
	uint8_t			revid;
	uint8_t			lat;
	int			unit;
	const char		*drv_name;
	struct chip_info	*p;
	struct gem_dev		*dp;
	struct icpt_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "%s%d: icptattach: called %s",
		drv_name, unit, ident));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "%s: ddi_regs_map_setup failed",
			drv_name);
		goto err;
	}

	vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid= pci_config_get8(conf_handle, PCI_CONF_REVID);
	lat  = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);

	for (i = 0, p = icpt_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_WARN,
		"%s: icpt_attach: wrong PCI venid/devid (0x%x, 0x%x)",
		drv_name, vid, did);
	pci_config_teardown(&conf_handle);
	goto err;

chip_found:
	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_put16(conf_handle, PCI_CONF_COMM,
		PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
			pci_config_get16(conf_handle, PCI_CONF_COMM));

	pci_config_teardown(&conf_handle);

	cmn_err(CE_CONT,
"!%s%d: %s (vid: 0x%04x, did: 0x%04x, revid: 0x%02x, latency timer: 0x%02x)",
		drv_name, unit, p->name, vid, did, revid, lat);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
			PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
			PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
			&icpt_dev_attr, (caddr_t *)&base, &regs_ha)
					!= DDI_SUCCESS) {
			goto err;
		}

		/*
		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* configuration on tx and rx rings and buffers */
		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_tx_max_frags = IPGE_MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = 1;
#if IPGE_MAXTXFRAGS <= 2
		gcp->gc_tx_desc_unit_shift = 5; /* 32 byte */
#elif IPGE_MAXTXFRAGS <= 6
		gcp->gc_tx_desc_unit_shift = 6; /* 64 byte */
#else
# error IPGE_MAXTXFRAGS must be 2 or 6
#endif
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size - 1;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad  = B_FALSE;
		gcp->gc_tx_copy_thresh = icpt_tx_copy_thresh;
		gcp->gc_tx_desc_write_oo = TX_RING_SIZE == TX_BUF_SIZE;

		gcp->gc_rx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 5;		/* 32 byte */
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max   = min(RX_BUF_SIZE, RX_RING_SIZE -1);
		gcp->gc_rx_copy_thresh = icpt_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = icpt_dev_attr;
		gcp->gc_buf_attr = icpt_buf_attr;
		gcp->gc_desc_attr = icpt_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = icpt_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = icpt_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = icpt_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* mii mode */
		gcp->gc_mii_mode = GEM_MODE_1000BASET;
		gcp->gc_mii_hw_link_detection = B_TRUE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval   = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout    = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait       = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &icpt_attach_chip;
		gcp->gc_reset_chip  = &icpt_reset_chip;
		gcp->gc_init_chip   = &icpt_init_chip;
		gcp->gc_start_chip  = &icpt_start_chip;
		gcp->gc_stop_chip   = &icpt_stop_chip;
		gcp->gc_multicast_hash = &icpt_mcast_hash;
		gcp->gc_set_rx_filter = &icpt_set_rx_filter;
		gcp->gc_set_media   = &icpt_set_media;
		gcp->gc_get_stats   = &icpt_get_stats;
		gcp->gc_interrupt   = &icpt_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &icpt_tx_desc_write;
		gcp->gc_rx_desc_write = &icpt_rx_desc_write;
		gcp->gc_tx_start      = &icpt_tx_start;
		gcp->gc_rx_start      = &icpt_rx_start;

		gcp->gc_tx_desc_init = &icpt_tx_desc_init;
		gcp->gc_rx_desc_init = &icpt_rx_desc_init;
		gcp->gc_tx_desc_stat = &icpt_tx_desc_stat;
		gcp->gc_rx_desc_stat = &icpt_rx_desc_stat;
		gcp->gc_tx_desc_clean = &icpt_tx_desc_clean;
		gcp->gc_rx_desc_clean = &icpt_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_init  = &gem_mii_init_default;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync  = &icpt_mii_sync;
		gcp->gc_mii_read  = &icpt_mii_read;
		gcp->gc_mii_write = &icpt_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof(struct icpt_dev), KM_SLEEP);
		lp->rev_id = revid;
		dp = gem_do_attach(dip, gcp, base, &regs_ha,
					lp, sizeof(*lp));

		kmem_free(gcp, sizeof(*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof(struct icpt_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
icptdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	switch (cmd) {
	case DDI_SUSPEND:
		return  (gem_suspend(dip));

	case DDI_DETACH:
		return  (gem_do_detach(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info icptminfo = {
	0,			/* mi_idnum */
	"icpt",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit icptrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&icptminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit icptwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&icptminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	icpt_info = {
	&icptrinit,	/* st_rdinit */
	&icptwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_icpt_ops = {
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
	&icpt_info,	/* cb_str */
	D_MP,		/* cb_flag */
#ifdef notdef
	CB_REV,		/* cb_rev */
	nodev,		/* cb_aread */
	nodev,		/* cb_awrite */
#endif
};

static	struct dev_ops icpt_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	icptattach,	/* devo_attach */
	icptdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_icpt_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&icpt_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "icpt: _init: called"));
	gem_mod_init(&icpt_ops, "icpt");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&icpt_ops);
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

	DPRINTF(2, (CE_CONT, "icpt: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&icpt_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
