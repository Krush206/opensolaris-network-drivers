/*
 *  alta: Sundance Technology ST201 Fast Ethernet MAC driver for Solaris
 *  @(#)alta_gem.c	1.22 11/09/04
 *
 * Copyright (c) 2002-2011 Masayuki Murayama.  All rights reserved.
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

#pragma	ident	"@(#)alta_gem.c 1.22     11/09/04"

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
#include "st201reg.h"

char	ident[] = "st201/ip100a driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int alta_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (alta_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	(drv_usectohz(1*1000000))
#define	ALTA_TFD(p)	((struct alta_tfd *)(void *)(p))
#define	ALTA_RFD(p)	((struct alta_rfd *)(void *)(p))

/*
 * ST201 TFD and RFD structure
 */
struct alta_fragment {
	volatile uint32_t	Addr;
	volatile uint32_t	Len;
};

#define	MAXTXFRAGS	min(GEM_MAXTXFRAGS, 7)	/* alta_tfd is power of 2 */

struct alta_tfd {
	volatile uint32_t	TxDMANextPtr;
	volatile uint32_t	TxFrameControl;
	struct alta_fragment	TxDMAFrag[MAXTXFRAGS];
};

#define	MAXRXFRAGS	1

struct alta_rfd {
	volatile uint32_t	RxDMANextPtr;
	volatile uint32_t	RxFrameStatus;
	struct alta_fragment	RxDMAFrag[MAXRXFRAGS];
};

struct alta_dev {
	boolean_t	need_to_reset;
	boolean_t	tx_list_loaded;
	uint8_t		rev_id;
	uint32_t	intmask;
	uint32_t	countdown;
	uint8_t		mac_addr[ETHERADDRL];
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
#define	OUR_INTR_BITS	\
	(INT_HostError | INT_TxComplete | INT_UpdateStats | \
	INT_LinkEvent | INT_TxDMAComplete | INT_RxDMAComplete)

#ifdef TEST_RX_EMPTY
#define	RX_RING_SIZE	1
#endif
#ifdef TEST_TXDESC_FULL
#define	TX_RING_SIZE	4
#define	TX_BUF_SIZE	64
#endif

#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif

#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	TX_BUF_SIZE
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif
#ifndef RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

static int	alta_tx_copy_thresh = 256;
static int	alta_rx_copy_thresh = 256;

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info alta_chiptbl[] = {
	{PCI_VID_SUNDANCE,	PCI_DID_ST201,	"ST201/IP100"},
	{PCI_VID_DLINK,		PCI_DID_DFE550, "DL10050"},
	{PCI_VID_SUNDANCE,	PCI_DID_IP100A,	"IP100A"},
};
#define	CHIPTABLESIZE   (sizeof (alta_chiptbl)/sizeof (struct chip_info))

/* ======================================================== */

/* mii operations */
static void  alta_mii_sync(struct gem_dev *);
static uint16_t alta_mii_read(struct gem_dev *, uint_t);
static void alta_mii_write(struct gem_dev *, uint_t, uint16_t);
static int  alta_mii_probe(struct gem_dev *);

/* nic operations */
static int alta_attach_chip(struct gem_dev *);
static int alta_reset_chip(struct gem_dev *);
static int alta_init_chip(struct gem_dev *);
static int alta_start_chip(struct gem_dev *);
static int alta_stop_chip(struct gem_dev *);
static int alta_set_media(struct gem_dev *);
static int alta_set_rx_filter(struct gem_dev *);
static int alta_get_stats(struct gem_dev *);

/* descriptor operations */
static int alta_tx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void alta_tx_start(struct gem_dev *dp, int slot, int nslot);
static void alta_rx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t alta_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t alta_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void alta_tx_desc_init(struct gem_dev *dp, int slot);
static void alta_rx_desc_init(struct gem_dev *dp, int slot);
static void alta_tx_desc_clean(struct gem_dev *dp, int slot);
static void alta_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t alta_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr alta_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr alta_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

#define	MAX_FRAG_LEN	0x7ff
#define	ALTA_MAX_PACKET_SIZE	2040
static ddi_dma_attr_t alta_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	MAX_FRAG_LEN,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	MAX_FRAG_LEN,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	ALTA_MAX_PACKET_SIZE,	/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t alta_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	8,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	8,			/* dma_attr_align */
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
 * misc routines
 */
/* ======================================================== */
static int
alta_log2(uint32_t val)
{
	int	i;

	for (i = 0; i < 32; i++) {
		if ((1 << i) >= val) {
			break;
		}
	}

	return (i);
}

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */

static int
alta_reset_chip(struct gem_dev *dp)
{
	int		i;
	struct alta_dev	*lp = dp->private;

	/* Reset the chip. */
	bzero(lp->mac_addr, ETHERADDRL);

	OUTW(dp, IntEnable, 0);

	/* XXX - should not reset AC_AutoInit nor AC_RstOut */
	OUTL(dp, AsicCtrl,
	    AC_GlobalReset | AC_RxReset | AC_TxReset
	    | AC_DMA | AC_FIFO | AC_Network | AC_Host);

	i = 0;
	while (INL(dp, AsicCtrl) & AC_ResetBusy) {
		drv_usecwait(10);
		if (i++ > 200) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout", dp->name, __func__);
			return (GEM_FAILURE);
		}
	}

	DPRINTF(1, (CE_CONT, "!%s: %s: took %d uS", dp->name, __func__, i*10));

	return (GEM_SUCCESS);
}

static int
alta_init_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));

	/* AsicCtrl : assume the chip stopped */

	DPRINTF(0, (CE_CONT, "!%s %s called, mc0: %b, mc1:%b, ac:%b, pc:%b",
	    dp->name, __func__,
	    INW(dp, MACCtrl0), MC0_BITS,
	    INW(dp, MACCtrl1), MC1_BITS,
	    INL(dp, AsicCtrl), AC_BITS,
	    INB(dp, PhyCtrl), PC_BITS));
#ifdef notdef
	/* MACCtrl1: Ensure all functions are disabled */
	OUTW(dp, MACCtrl1,
	    MC1_StatisticsDisable | MC1_TxDisable | MC1_RxDisable);
#endif
	/* MACCtrl0: Ensure normal operational mode */
	OUTW(dp, MACCtrl0, MC0_IFSSelect802_3);
#ifdef GEM_CONFIG_GLDv3
	OUTW(dp, MaxFrameSize, dp->mtu + sizeof (struct ether_header) + 4);
#else
	OUTW(dp, MaxFrameSize, dp->mtu + sizeof (struct ether_header));
#endif
	/* TxDMA */
	OUTL(dp, TxDMAListPtr, 0);
	OUTB(dp, TxDMAPollPeriod, TxDMAPollPeriodMax);
	lp->tx_list_loaded = B_FALSE;

	/* RxDMA */
	OUTL(dp, RxDMAListPtr, 0);
	OUTB(dp, RxDMAPollPeriod, RxDMAPollPeriodMax);

	OUTB(dp, RxDMABurstThresh, dp->rxthr/RxDMABurstThreshUnit);

	/* TxStartThreash : depends on txthr */
#ifdef TXUNDERRUNTEST
	dp->txmaxdma = min(256, dp->txthr);
#endif
	OUTW(dp, TxStartThresh, dp->txthr & ~(TxStartThreshUnit-1));
	OUTB(dp, TxDMABurstThresh,
	    max(dp->txmaxdma / TxDMABurstThreshUnit, 1));

	/* TxDMAUrgentThreash : half of txthr */
	OUTB(dp, TxDMAUrgentThresh,
	    max((dp->txthr / 2) / TxDMAUrgentThreshUnit, 1));

	/* TxReleaseThreash : 128 byte */
	OUTB(dp, TxReleaseThresh, 128/TxReleaseThreshUnit);

	/* RxEarlyThreth: disable Eerly Rx */
	OUTW(dp, RxEarlyThresh, RxEarlyThreshDisable);

	DPRINTF(2, (CE_CONT, "!%s: %s: mc0: %b, mc1: %b, ac: %b",
	    dp->name, __func__,
	    INW(dp, MACCtrl0), MC0_BITS,
	    INW(dp, MACCtrl1), MC1_BITS,
	    INL(dp, AsicCtrl), AC_BITS));

	return (GEM_SUCCESS);
}

static int
alta_start_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Enable statistics */
	OUTW(dp, MACCtrl1, MC1_StatisticsEnable);

	if (lp->rev_id >= 0x14) {
		/* XXX - undocumented */
		OUTB(dp, DebugCtrl1, 0x01);
	}

	ASSERT((dp->rx_ring_dma & 7) == 0);
	OUTL(dp, RxDMAListPtr, dp->rx_ring_dma);

	/* Enable transmitter and receiver */
	OUTW(dp, MACCtrl1, MC1_TxEnable | MC1_RxEnable);

	/* Set InterruptMask */
	lp->intmask = OUR_INTR_BITS;
#ifdef GEM_CONFIG_POLLING
	lp->intmask |= INT_IntRequest;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IntEnable, lp->intmask);
	}
	/* disable interval timer */
	lp->countdown = 0;
	OUTW(dp, Countdown, lp->countdown);

	OUTL(dp, DMACtrl, 0);
	DPRINTF(0, (CE_CONT, "!%s: %s: mc0: %b, mc1: %b, dmactrl: %b",
	    dp->name, __func__,
	    INW(dp, MACCtrl0), MC0_BITS,
	    INW(dp, MACCtrl1), MC1_BITS,
	    INL(dp, DMACtrl), DC_BITS));

	return (GEM_SUCCESS);
}

static int
alta_stop_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* inhibit interrupt */
	/* XXX - don't clear lp->intmask to avoid bogus interrupts */
	OUTW(dp, IntEnable, 0);

	/* no more statistics */
	OUTW(dp, MACCtrl1, MC1_StatisticsDisable);

	/* stop receiver and transceiver */
	OUTW(dp, MACCtrl1, MC1_RxDisable | MC1_TxDisable);
	if (dp->speed == GEM_SPD_10) {
		drv_usecwait(2000);
	} else {
		drv_usecwait(200);
	}

	/* reset DMA pointers */
	OUTL(dp, AsicCtrl, AC_RxReset | AC_TxReset | AC_DMA);

	return (GEM_SUCCESS);
}


static uint16_t
alta_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;

	OUTW(dp, EepromCtrl, EEC_OpcodeRd | (offset & EEC_Address));
	drv_usecwait(100);

	for (i = 0; INW(dp, EepromCtrl) & EEC_Busy; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s timeout", dp->name, __func__);
			return (0xffff);
		}
		drv_usecwait(10);
	}

	DPRINTF(4, (CE_CONT, "!%s: %s: took %d uS", dp->name, __func__, i*10));

	return (INW(dp, EepromData));
}

#ifdef DEBUG_LEVEL
static void
alta_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	prom[0x14];

	for (i = 0; i < 0x14; i++) {
		prom[i] = alta_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 0x14; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
}
#endif /* DEBUG_LEVEL */

static int
alta_attach_chip(struct gem_dev *dp)
{
	int			i;
	uint16_t		val;

	for (i = 0; i < ETHERADDRL; i += 2) {
		val = alta_read_eeprom(dp, 0x10 + i/2);

		dp->dev_addr.ether_addr_octet[i + 0] = (uint8_t)val;
		dp->dev_addr.ether_addr_octet[i + 1] = (uint8_t)(val >> 8);
	}

#if DEBUG_LEVEL > 4
	alta_eeprom_dump(dp);
#endif
#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	dp->misc_flag |= GEM_POLL_RXONLY;

	return (GEM_SUCCESS);
}

static uint32_t
alta_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is 6 bits of LSB in be-crc */
	return (gem_ether_crc_be(addr, ETHERADDRL) % 64);
}

static int
alta_set_rx_filter(struct gem_dev *dp)
{
	uint8_t		mode;
	uint8_t		*newmac;
	int		i;
	uint64_t	mhash;
	struct alta_dev	*lp = dp->private;

	ASSERT(mutex_owned(&dp->intrlock));
	mhash = 0;

	mode = RM_ReceiveUnicast | RM_ReceiveBroadcast;
	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* disabled */
		mode = 0;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscous */
		mode |= RM_ReceiveAllFrames;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* allmulti */
		mode |= RM_ReceiveMulticast;
	} else if (dp->mc_count > 0) {
		/* Normal mode with multicast */
		mode |= RM_ReceiveMulticastHash;

		/* make hash table */
		for (i = 0; i < dp->mc_count; i++) {
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	/* set rx mode */
	DPRINTF(2, (CE_CONT, "!%s: %s: setting mode: %b",
	    dp->name, __func__, mode, RM_BITS));

	if (mode) {
		OUTB(dp, ReceiveMode, RM_ReceiveAllFrames);
	}

	/* set multicast hash table up */
	for (i = 0; i < 4; i++) {
		OUTW(dp, HashTable + i*2, mhash >> (i*16));
	}

	newmac = dp->cur_addr.ether_addr_octet;
	if (bcmp(newmac, lp->mac_addr, ETHERADDRL)) {
		/* need to update the station address */
		for (i = 0; i < ETHERADDRL; i += 2) {
			OUTW(dp, StationAddress + i,
			    (newmac[i+1] << 8) | newmac[i]);
		}
		bcopy(newmac, lp->mac_addr, ETHERADDRL);
	}

	OUTB(dp, ReceiveMode, mode);

	return (GEM_SUCCESS);
}

static int
alta_set_media(struct gem_dev *dp)
{
	uint16_t	old;
	uint16_t	new;
	int		i;

	ASSERT(mutex_owned(&dp->intrlock));

	DPRINTF(0, (CE_CONT, "!%s %s called, mac_active;%d, pc:%b",
	    dp->name, __func__, dp->mac_active, INB(dp, PhyCtrl), PC_BITS));

	/*
	 * Notify current duplex mode to MAC
	 */
	old = INW(dp, MACCtrl0);
	new = old & ~(MC0_FullDuplexEnable | MC0_FlowControlEnable);

	if (dp->full_duplex) {
		new |= MC0_FullDuplexEnable;

		switch (dp->flow_control) {
		case FLOW_CONTROL_SYMMETRIC:
		case FLOW_CONTROL_TX_PAUSE:
			new |= MC0_FlowControlEnable;
			break;
		}
	}

	OUTW(dp, MACCtrl0, new);

	if ((old ^ new) & MC0_FullDuplexEnable) {
		/*
		 * Duplex mode is changed. Need to reset Tx/Rx.
		 */
		DPRINTF(1, (CE_CONT,
		    "!%s: resetting tx and rx to change duplex mode",
		    dp->name));

		OUTL(dp, AsicCtrl, AC_TxReset | AC_RxReset);

		i = 0;
		while (INL(dp, AsicCtrl) & AC_ResetBusy) {
			if (i++ > 200) {
				/* Time out */
				cmn_err(CE_NOTE, "!%s: %s: tx/rx reset timeout",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}

		OUTW(dp, MACCtrl1, MC1_TxEnable | MC1_RxEnable);
	}

	return (GEM_SUCCESS);
}

static int
alta_get_stats(struct gem_dev *dp)
{
	int		first_coll;
	int		multi_coll;
	int		c;
	int		l;
	volatile int	x;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#ifdef notdef
	ASSERT(mutex_owned(&dp->intrlock));
#endif
	OUTW(dp, MACCtrl1, MC1_StatisticsDisable);

	x =			INL(dp, OctetsRecevedOK);
	x =			INL(dp, OctetsTransmittedOK);

	x =			INW(dp, FramesTransmittedOK);
	x =			INW(dp, FramesReceivedOK);

	dp->stats.nocarrier += c = INB(dp, CarrierSenseErrors);
	dp->stats.xmtlatecoll += l = INB(dp, LateCollisionss);
	multi_coll =		INB(dp, MultipleColFrames);
	first_coll =		INB(dp, SingleColFrames);
	dp->stats.defer +=	INB(dp, FramesWDeferedXmt);
	dp->stats.missed +=	INB(dp, FramesLostRxErrors);
	x =			INB(dp, FramesWEXDeferral);
	x =			INB(dp, FramesAbortXSClls);
	x =			INB(dp, BcstFramesXmtdOK);
	x =			INB(dp, BcstFramesRcvdOK);
	x =			INB(dp, McstFramesXmtdOK);
	x =			INB(dp, McstFramesRcvdOK);

	dp->stats.multi_coll +=	multi_coll;
	dp->stats.first_coll +=	first_coll;

	/*
	 * Guess total collisions
	 */
	dp->stats.collisions +=	first_coll + multi_coll*2;

	dp->stats.errxmt += c + l;

	OUTW(dp, MACCtrl1, MC1_StatisticsEnable);

	return (GEM_SUCCESS);
}

/*
 * descriptor manipulation
 */
static int
alta_tx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
	struct alta_tfd		*tfdp;
	struct alta_fragment	*tfp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	uint32_t		addr;
	struct alta_dev		*lp = dp->private;
	const uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 2
	/* force to cause interrupt upon tx completion */
	flag |= GEM_TXFLAG_INTR;
#endif
	tfdp = &ALTA_TFD(dp->tx_ring)[slot];
#if DEBUG_LEVEL > 10
	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d flag: %llx",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
#if 0
	tfdp->TxDMANextPtr = 0;
#else
	tfdp->TxDMANextPtr = dp->tx_ring_dma +
	    sizeof (struct alta_tfd) * SLOT(slot + 1, tx_ring_size);
#endif

	mark = 0;
	if (flag & GEM_TXFLAG_INTR) {
#ifdef USE_FRAMEID
		mark = TFC_TxIndicate;
#else
		mark = TFC_TxDMAIndicate;
#endif
	}
	mark |=
#ifdef USE_FRAMEID
	    (TFC_FrameId & (slot << TFC_FrameIdShift)) |
#endif
	    TFC_WordAlignDisable;
	tfdp->TxFrameControl = LE_32(mark);

	/* copy fragment list */
	mark = TFD_TxDMAFragLast; /* last fragment */
	i = frags - 1;
	dcp = &dmacookie[i];
	tfp = &tfdp->TxDMAFrag[i];
	for (; i >= 0; dcp--, tfp--, i--) {
		mark |= dcp->dmac_size;
		tfp->Len = LE_32(mark);
		addr = dcp->dmac_address;
		tfp->Addr = LE_32(addr);
		mark = 0;
	}

	return (1);
}

static void
alta_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int		slot;
	uint32_t	tfd_dma;
	struct alta_dev	*lp = dp->private;
	const uint_t	tx_ring_size = dp->gc.gc_tx_ring_size;

	slot = SLOT(start_slot + nslot - 1, tx_ring_size);
	tfd_dma = dp->tx_ring_dma + sizeof (struct alta_tfd) * slot;

	/* make new tx list */
#if 0
	while (slot != start_slot) {
		/* get the index of the previous slot */
		slot = SLOT(slot - 1, tx_ring_size);

		/* link me to the previous */
		ALTA_TFD(dp->tx_ring)[slot].TxDMANextPtr = LE_32(tfd_dma);

		/* update current descriptor address in dma space */
		tfd_dma = dp->tx_ring_dma + sizeof (struct alta_tfd) * slot;
	}
#else
	/* terminate the tx list */
	slot = SLOT(start_slot + nslot - 1, tx_ring_size);
	ALTA_TFD(dp->tx_ring)[slot].TxDMANextPtr = 0;
#endif
	/* flush tx descriptors in the tx list we have made */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

#ifdef TEST_TXTIMEOUT
	if ((lp->send_cnt++ % 10000) == 9999) {
		OUTW(dp, MACCtrl1, MC1_TxDisable);
	}
#endif
	if (lp->tx_list_loaded)  {
		/*
		 * Link the new list to the tail of current tx descriptor.
		 */
		slot = SLOT(start_slot - 1, tx_ring_size);
		ALTA_TFD(dp->tx_ring)[slot].TxDMANextPtr = LE_32(tfd_dma);
		gem_tx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);
	} else {
		/*
		 * This is first call after initialization.
		 * Need to tell the head of tx list to the nic.
		 */
		OUTL(dp, TxDMAListPtr, tfd_dma);
		lp->tx_list_loaded = B_TRUE;
	}
}

static void
alta_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	struct alta_rfd		*rfdp;
	struct alta_fragment	*rfp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		len;
	uint32_t		n;
	uint32_t		mark;
	uint32_t		addr;

	rfdp = &ALTA_RFD(dp->rx_ring)[slot];

#if DEBUG_LEVEL > 10
	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif

	/* copy fragment list in reversed order */
	len = 0;
	i = frags - 1;
	dcp = &dmacookie[i];
	rfp = &rfdp->RxDMAFrag[i];
	mark = RFD_RxDMALastFrag;
	for (; i >= 0; i--, dcp--, rfp--) {
		n = (uint32_t)dcp->dmac_size;
		len += n;
		mark |= n;
		addr = dcp->dmac_address;
		rfp->Addr = LE_32(addr);
		rfp->Len = LE_32(mark);

		mark = 0;
	}
	rfdp->RxFrameStatus = LE_32(len);
}

static uint_t
alta_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct alta_tfd		*tfdp;
	uint32_t		frameid;
	uint32_t		tfc;

	tfdp = &ALTA_TFD(dp->tx_ring)[slot];
#ifdef USE_FRAMEID
	frameid = (INW(dp, TxStatus) & TS_TxFrameId) >> TS_TxFrameIdShift;
	if (slot == frameid + 1) {
		return (0);
	}
	if (slot == frameid) {
		tfc = tfdp->TxFrameControl;
		tfc = LE_32(tfc);

		if ((tfc & TFC_TxDMAComplete) == 0) {
			/* not transmitted */
			return (0);
		}
	}
#else
	tfc = tfdp->TxFrameControl;
	tfc = LE_32(tfc);

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d, tfs:0x%b",
	    dp->name, __func__, slot, tfc, TFC_BITS));

	if ((tfc & TFC_TxDMAComplete) == 0) {
		/* not transmitted */
		return (0);
	}
#endif
	return (GEM_TX_DONE);
}

static uint64_t
alta_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct alta_rfd		*rfdp;
	uint32_t		rfs;
	uint32_t		len;
	uint32_t		flag;

	rfdp = &ALTA_RFD(dp->rx_ring)[slot];

	rfs = rfdp->RxFrameStatus;
	rfs = LE_32(rfs);
	len = rfs & RFS_RxDMAFrameLen;
	flag = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d, rfs:0x%b",
	    dp->name, __func__, slot, rfs, RFS_BITS));

	if ((rfs & RFS_RxDMAComplete) == 0) {
		/* not received */
		return (0);
	}

	if (rfs & RFS_ERRORS) {
		/* error packet */
		dp->stats.errrcv++;
		if (rfs & (RFS_RxAlignmentError | RFS_RxFrameError)) {
			dp->stats.frame++;
		}
		if (rfs & RFS_RxFIFOOverrun) {
			dp->stats.overflow++;
		}
		if (rfs & RFS_RxRuntFrame) {
			dp->stats.runt++;
		}
		if (rfs & RFS_RxFCSError) {
			dp->stats.crc++;
		}
		if (rfs & (RFS_RxDMAOverflow | RFS_RxOversizedFrame)) {
			dp->stats.frame_too_long++;
		}

		flag = GEM_RX_ERR;
	}

	return (flag | len);
}

static void
alta_tx_desc_init(struct gem_dev *dp, int slot)
{
	ALTA_TFD(dp->tx_ring)[slot].TxFrameControl = LE_32(TFC_TxDMAComplete);
}

static void
alta_rx_desc_init(struct gem_dev *dp, int slot)
{
	int		i;
	uint32_t	ptr;
	struct alta_rfd		*rfdp;

	rfdp = &ALTA_RFD(dp->rx_ring)[slot];

	/* invalidate this rfd */
	rfdp->RxFrameStatus = LE_32(RFS_RxDMAComplete);

	/* link it to the previous rfd */
	rfdp = &ALTA_RFD(dp->rx_ring)[SLOT(slot - 1, dp->gc.gc_rx_ring_size)];
	ptr = ((uint32_t)dp->rx_ring_dma) + slot * sizeof (struct alta_rfd);
	rfdp->RxDMANextPtr = LE_32(ptr);
}

static void
alta_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct alta_tfd		*tfdp;

	tfdp = &ALTA_TFD(dp->tx_ring)[slot];

	tfdp->TxDMANextPtr = 0;
	tfdp->TxFrameControl = LE_32(TFC_TxDMAComplete);
}

static void
alta_rx_desc_clean(struct gem_dev *dp, int slot)
{
	/* invalidate this rfd */
	ALTA_RFD(dp->rx_ring)[slot].RxFrameStatus = LE_32(RFS_RxDMAComplete);
}

/*
 * Device depend interrupt handler
 */
static void
alta_get_tx_status(struct gem_dev  *dp)
{
	int		error = 0;
	int		restart_xmit;
	uint16_t	tx_status;
	int		i;
	struct alta_dev	*lp = dp->private;

	/* Pop all status from the Tx status stack. */
	while ((tx_status = INW(dp, TxStatus)) & TS_TxComplete) {

		/* Discard one */
		OUTW(dp, TxStatus, 0);

		if (tx_status & TS_TxReleaseError) {
			cmn_err(CE_WARN,
			    "!%s: tx release error: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
		}

		if (tx_status & TS_TxStatusOverflow) {
			cmn_err(CE_WARN,
			    "!%s: tx status overflow: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
		}

		/* update statistics */
		if (tx_status & TS_TxUnderrun) {
			/* fifo underflow. TxReset and TxEnable are required */
			cmn_err(CE_WARN,
			    "!%s: tx underrun error: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
			dp->stats.underflow++;
			dp->stats.errxmt++;
		}

		if (tx_status & TS_MaxCollisions) {
			/* exceed maximum collisions */
			dp->stats.excoll++;
			dp->stats.collisions += 16;
			dp->stats.errxmt++;
		}

		/* collect status bits */
		error |= (tx_status &
		    (TS_TxReleaseError | TS_TxUnderrun | TS_MaxCollisions));
	}

	mutex_enter(&dp->xmitlock);

	if (error) {
		if (error & (TS_TxReleaseError | TS_TxUnderrun)) {
			/* FATAL error */
#ifdef notdef
			/* XXX - the nic has stopped */
			OUTW(dp, MACCtrl1, MC1_TxDisable);
#endif
			lp->need_to_reset = B_TRUE;
			if (error & TS_TxUnderrun) {
				/* increase tx threashold by 64 byte */
				dp->txthr = min(dp->txthr+64, ETHERMAX);
			}
		} else {
			if (error & TS_MaxCollisions) {
				OUTW(dp, MACCtrl1, MC1_StatisticsEnable);
			}

			OUTW(dp, MACCtrl1, MC1_TxEnable);
		}
	}

	mutex_exit(&dp->xmitlock);
}


static uint_t
alta_interrupt(struct gem_dev *dp)
{
	uint16_t	int_status;
	uint_t		tx_sched = 0;
	struct alta_dev	*lp = dp->private;

	/*
	 * Read interrupt status
	 * Do not use IntStatusAck here. It clears IntEnable too.
	 */
	int_status = INW(dp, IntStatus);

	if ((int_status & lp->intmask) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	/*
	 * Acknowledge to the interrupt.
	 * Reading IntStatusAck clears both of IntStatus and IntEnable
	 */
	int_status = INW(dp, IntStatusAck);
	DPRINTF(3, (CE_CONT, "!%s: %s, int_status: %b",
	    dp->name, __func__, int_status, INT_BITS));
	if (!dp->mac_active) {
		/*
		 * the device is not active.
		 * side effect: left interrupt masked.
		 */
		lp->intmask = 0;
		return (DDI_INTR_CLAIMED);
	}

#ifdef GEM_CONFIG_POLLING
	int_status &= lp->intmask | INT_RxDMAComplete;

	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval) {
			/* move to polling mode */
			lp->intmask &= ~INT_RxDMAComplete;
			lp->countdown = min(dp->poll_interval/1250, 0xffff);
		} else {
			/* move to normal mode */
			lp->intmask |= INT_RxDMAComplete;

			/* use countdown timer for tx timeout */
			lp->countdown = 0;
		}
		/*
		 * To schedule the next timer interrupt,
		 * we pretend as we were interrupted from
		 * polling timer
		 */
		int_status |= INT_IntRequest;
	}
	lp->last_poll_interval = dp->poll_interval;
#else
	int_status &= lp->intmask;
#endif /* GEM_CONFIG_POLLING */

	if (int_status & INT_IntRequest) {
		/*
		 * re-load Countdown timer to schedule the next polling
		 * timer interrupt.
		 */
		OUTW(dp, Countdown, lp->countdown);
	}

	if (int_status & INT_UpdateStats) {
		/*
		 * Statistics counter overflow
		 */
		alta_get_stats(dp);
	}

	if (int_status & (INT_RxDMAComplete | INT_MACControlFrame)) {
		/*
		 * packet was received, or receive error happened
		 */
		(void) gem_receive(dp);
	}

	if (int_status & INT_TxComplete) {
		/*
		 * Pull up TxStatus
		 */
		(void) alta_get_tx_status(dp);
	}

	if (int_status & (INT_TxComplete | INT_TxDMAComplete)) {
		/*
		 * Packets was transfered into TxFIFO or
		 * an error happened.
		 */
		if (gem_tx_done(dp)) {
			tx_sched |= INTR_RESTART_TX;
		}
	}

	if (int_status & INT_LinkEvent) {
		DPRINTF(0, (CE_CONT, "!%s: %s link event",
		    dp->name, __func__));
		if (gem_mii_link_check(dp)) {
			tx_sched |= INTR_RESTART_TX;
		}
	}

	if (int_status & (INT_RxEarly |
#ifndef GEM_CONFIG_POLLING
	    INT_IntRequest |
#endif
	    INT_HostError)) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: %b",
		    dp->name, int_status, INT_BITS);
	}

	if (lp->need_to_reset) {
		gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		tx_sched = INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "!%s: %s done: int_status: %b",
	    dp->name, __func__, INW(dp, IntStatus), INT_BITS));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IntEnable, lp->intmask);
	}

	return (DDI_INTR_CLAIMED | tx_sched);
}

/*
 * HW depend MII routine
 */
#define	MDIO_DELAY(dp)    {INB(dp, PhyCtrl); INB(dp, PhyCtrl); }

static void
alta_mii_sync(struct gem_dev *dp)
{
	int	i;

	/* output 32 ones */
	for (i = 0; i < 32; i++) {
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
}

static uint16_t
alta_mii_read(struct gem_dev *dp, uint_t reg)
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
	if (INB(dp, PhyCtrl) & PC_MgmtData) {
		DPRINTF(1, (CE_CONT, "!%s: no response from phy@%d",
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

	return (ret);
}

static void
alta_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint32_t	cmd;
	int		i;
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
}
#undef MDIO_DELAY

static int
alta_mii_probe(struct gem_dev *dp)
{
	int	ret;
	uint32_t	val;

	ret = gem_mii_probe_default(dp);
	if (ret == GEM_SUCCESS) {
		val = INL(dp, AsicCtrl);
		if (val & AC_PhyMedia) {
			/*
			 * workaround for fibre interface.
			 * we need to disable autonego by default.
			 */
			dp->mii_status &= ~MII_STATUS_CANAUTONEG;
			dp->mii_status |= MII_STATUS_100_BASEX_FD;
		}
	}
	return (ret);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
altaattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct alta_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: ddi_regs_map_setup failed",
		    drv_name);
		goto err;
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	lat = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);

	for (i = 0, p = alta_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_NOTE, "!%s%d: %s: unknown PCI venid/devid (0x%x, 0x%x)",
	    drv_name, unit, __func__, vid, did);
	/* assume ST201/IP100 chipset */
	p = &alta_chiptbl[0];
	/* fall down */

chip_found:
	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	pci_config_teardown(&conf_handle);

	cmn_err(CE_CONT,
	    "!%s%d: %s (vid: 0x%04x, did: 0x%04x,"
	    " revid: 0x%02x, latency timer: 0x%02x)",
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
		    &alta_dev_attr, (void *)&base, &regs_ha)
		    != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_desc_unit_shift =
		    alta_log2(sizeof (struct alta_tfd));
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = alta_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_unit_shift =
		    alta_log2(sizeof (struct alta_rfd));
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = alta_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = alta_dev_attr;
		gcp->gc_buf_attr = alta_buf_attr;
		gcp->gc_desc_attr = alta_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = alta_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = alta_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		gcp->gc_dma_attr_rxbuf = alta_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
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

		/* hw link event doesn't work */
		gcp->gc_mii_hw_link_detection = B_FALSE;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &alta_attach_chip;
		gcp->gc_reset_chip = &alta_reset_chip;
		gcp->gc_init_chip = &alta_init_chip;
		gcp->gc_start_chip = &alta_start_chip;
		gcp->gc_stop_chip = &alta_stop_chip;
		gcp->gc_multicast_hash = &alta_mcast_hash;
		gcp->gc_set_rx_filter = &alta_set_rx_filter;
		gcp->gc_set_media = &alta_set_media;
		gcp->gc_get_stats = &alta_get_stats;
		gcp->gc_interrupt = &alta_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &alta_tx_desc_write;
		gcp->gc_rx_desc_write = &alta_rx_desc_write;
		gcp->gc_tx_start = &alta_tx_start;

		gcp->gc_tx_desc_init = &alta_tx_desc_init;
		gcp->gc_rx_desc_init = &alta_rx_desc_init;
		gcp->gc_tx_desc_stat = &alta_tx_desc_stat;
		gcp->gc_rx_desc_stat = &alta_rx_desc_stat;
		gcp->gc_tx_desc_clean = &alta_tx_desc_clean;
		gcp->gc_rx_desc_clean = &alta_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_probe = &alta_mii_probe;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &alta_mii_sync;
		gcp->gc_mii_read = &alta_mii_read;
		gcp->gc_mii_write = &alta_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof (struct alta_dev), KM_SLEEP);
		lp->rev_id = revid;

		/* offload and jumbo frame */
		gcp->gc_max_lso = 0;
		gcp->gc_max_mtu = 0x7f8 - sizeof (struct ether_header) - 4 - 4;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof (struct alta_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
altadetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

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
GEM_STREAM_OPS(alta_ops, altaattach, altadetach);
#else
static	struct module_info altaminfo = {
	0,			/* mi_idnum */
	"alta",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit altarinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&altaminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit altawinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&altaminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	alta_info = {
	&altarinit,	/* st_rdinit */
	&altawinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_alta_ops = {
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
	&alta_info,	/* cb_stream */
	D_MP,		/* cb_flag */
#ifdef notdef
	CB_REV,		/* cb_rev */
	nodev,		/* cb_aread */
	nodev,		/* cb_awrite */
#endif
};

static	struct dev_ops alta_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	altaattach,	/* devo_attach */
	altadetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_alta_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&alta_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!alta: _init: called"));
	gem_mod_init(&alta_ops, "alta");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&alta_ops);
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

	DPRINTF(2, (CE_CONT, "!alta: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&alta_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
