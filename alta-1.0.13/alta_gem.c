/*
 *  alta: Sundance Technology ST201 Fast Ethernet MAC driver for Solaris
 *  @(#)alta_gem.c	1.19 07/08/23
 *
 * Copyright (c) 2002-2007 Masayuki Murayama.  All rights reserved.
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

02/09/2003  0.9.0 release. tested under s9 and s8 
02/10/2003  alta_get_stats fixed. count no carrier and late collision
		into totai xmit error.
02/10/2003  0.9.1 release.
03/13/2003  fixed for sparc
04/10/2003  ddi_dma_sync added when accessing tx/rx descriptors
04/11/2003  alta_mii_init added to scan from location 1 instead of 0
04/20/2003  0.9.4 release (updated for DL10050B)
10/19/2003  alta_mii_init removed, get_packet removed
	    gld entries removed.
            1.0.0 release
03/01/2004  1.0.2 release
04/11/2004  1.0.3 release; license changed
12/06/2004  tx list was potentially corrupted. Number of tx bufs must be
	    smaller tban number of tx descriptors. fixed.
03/23/2006  ddytypes.h added, ddi_impldefs.h removed because of compile time
	    error.
	    accessing registers via PCI memory space doesn't work for
	    DFE-580 quad port ethernet card. Thank to Vladimir very much for
	    testing.
	    alta_mii_read_raw was removed because this workaround is duplicated
	    with that of gem layer.
	    Now I ensured alta worked with DFE-580.
03/24/2006  1.0.12 release
07/13/2007  IP100A supported (thank to Pedro Alejandro Lopez Valencia)
07/13/2007  1.0.13 release

 */

/*
 TODO:
	separete allocation descriptors and bufferes
	txbuf allocation in attach
	fix rx_desc_write/tx_desc_write interface
	clear txlistloaded on reset
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
#include "st201reg.h"

#ifdef sun4u
# define	IOCACHE
# define	IOMMU
#endif

char	ident[] = "st201 nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int alta_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (alta_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	FALSE	(0)
#define	TRUE	(!FALSE)

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

/*
 * ST201 TFD and RFD structure
 */
struct alta_fragment {
	uint32_t	Addr;
	uint32_t	Len;
};

#define	MAXTXFRAGS	GEM_MAXTXFRAGS		/* was 7 */

struct alta_tfd{
	uint32_t		TxDMANextPtr;
	uint32_t		TxFrameControl;
	struct alta_fragment	TxDMAFrag[MAXTXFRAGS];
};


#define	MAXRXFRAGS	GEM_MAXRXFRAGS		/* was 3 */

struct alta_rfd {
	uint32_t		RxDMANextPtr;
	uint32_t		RxFrameStatus;
	struct alta_fragment	RxDMAFrag[MAXRXFRAGS];
};

struct alta_dev {
	boolean_t	need_to_reset;
	boolean_t	tx_list_loaded;
	uint8_t		rev_id;
	uint32_t	intmask;
	int		countdown;
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
#define	OUR_INTR_BITS \
	(INT_HostError | INT_TxComplete | INT_UpdateStats | \
	 INT_LinkEvent | INT_TxDMAComplete | INT_RxDMAComplete)

#ifdef TEST_RX_EMPTY
# define RX_RING_SIZE	1
#endif
#ifdef TEST_TXDESC_FULL
# define TX_RING_SIZE	4
# define TX_BUF_SIZE	64
#endif

#ifndef TX_RING_SIZE
# define TX_RING_SIZE	64
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
	{PCI_VID_DLINK,		PCI_DID_DFE550, "DL10050"},
	{PCI_VID_SUNDANCE,	PCI_DID_ST201,	"ST201"},
	{PCI_VID_SUNDANCE,	PCI_DID_IP100A,	"IP100A"},
};
#define CHIPTABLESIZE   ((int)(sizeof(alta_chiptbl)/sizeof(struct chip_info)))

/* ======================================================== */
 
/* mii operations */
static void  alta_mii_sync(struct gem_dev *);
static uint16_t alta_mii_read(struct gem_dev *, uint_t);
static void alta_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int alta_attach_chip(struct gem_dev *);
static int alta_reset_chip(struct gem_dev *);
static void alta_init_chip(struct gem_dev *);
static void alta_start_chip(struct gem_dev *);
static int alta_stop_chip(struct gem_dev *);
static void alta_set_media(struct gem_dev *);
static void alta_set_rx_filter(struct gem_dev *);
static void alta_get_stats(struct gem_dev *);

/* descriptor operations */
static int alta_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag);
static int alta_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t alta_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t alta_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void alta_tx_desc_init(struct gem_dev *dp, int slot);
static void alta_rx_desc_init(struct gem_dev *dp, int slot);
static void alta_tx_desc_clean(struct gem_dev *dp, int slot);
static void alta_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int alta_interrupt(struct gem_dev *dp);

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

static ddi_dma_attr_t alta_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0,/* patched later */	/* dma_attr_sgllen */
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
 * HW manupilation routines
 */
/* ======================================================== */

static int
alta_reset_chip(struct gem_dev *dp)
{
	int			i;
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	/* Reset the chip. */
	lp->intmask = 0;
	OUTW(dp, IntEnable, 0);
	OUTL(dp, AsicCtrl,
		AC_GlobalReset | AC_RxReset | AC_TxReset
		| AC_DMA | AC_FIFO | AC_Network
		| AC_Host /* | AC_AutoInit | AC_RstOut*/);
	i = 0;
	while ((INL(dp, AsicCtrl) & AC_ResetBusy) != 0) {
		drv_usecwait(10);
		if (i++ > 200) {
			cmn_err(CE_WARN, "%s: alta_reset_chip: timeout",
				dp->name);
			return -1;
		}
	}

	/* clear register shadows */
	bzero(lp->mac_addr, ETHERADDRL);

	DPRINTF(1, (CE_CONT, "!%s: alta_reset_chip: took %d uS",
				dp->name, i*10));

	return 0;
}

static void
alta_init_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: alta_init_chip: called", dp->name));

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(lp->intmask == 0);

	/* AsicCtrl : assume the chip stopped */

	DPRINTF(3, (CE_CONT, "!%s alta_init_chip called, mc0: %b",
		dp->name, INW(dp, MACCtrl0), MC0_BITS));
	DPRINTF(3, (CE_CONT, "!%s: mc1: %b",
		dp->name, INW(dp, MACCtrl1), MC1_BITS));
	DPRINTF(3, (CE_CONT, "!%s: ac: %b", dp->name,
		INL(dp, AsicCtrl), AC_BITS));
#ifdef notdef
	/* MACCtrl1: Ensure all functions are disabled */
	OUTW(dp, MACCtrl1,
		MC1_StatisticsDisable | MC1_TxDisable | MC1_RxDisable);
#endif
	/* MACCtrl0: Ensure normal operational mode*/
	OUTW(dp, MACCtrl0, MC0_IFSSelect802_3);

	OUTW(dp, MaxFrameSize, dp->mtu + sizeof(struct ether_header));

	/* TxDMA */
	OUTL(dp, TxDMAListPtr, 0);
	OUTB(dp, TxDMAPollPeriod, TxDMAPollPeriodMax);
	lp->tx_list_loaded = FALSE;

	/* RxDMA */
	OUTL(dp, RxDMAListPtr, 0);
	OUTB(dp, RxDMAPollPeriod, RxDMAPollPeriodMax);

	OUTB(dp, RxDMABurstThresh, dp->rxthr/RxDMABurstThreshUnit);

	/* TxStartThreash : depends on txthr */
#ifdef TXUNDERRUNTEST
	dp->txmaxdma = min(256, dp->txthr);
#endif
	OUTW(dp, TxStartThresh, dp->txthr & ~(TxStartThreshUnit-1));
	OUTB(dp, TxDMABurstThresh, max(dp->txmaxdma/TxDMABurstThreshUnit, 1));

	/* TxDMAUrgentThreash : half of txthr */
	OUTB(dp, TxDMAUrgentThresh, max((dp->txthr/2)/TxDMAUrgentThreshUnit,1));

	/* TxReleaseThreash : 128 byte */
	OUTB(dp, TxReleaseThresh, 128/TxReleaseThreshUnit);

	/* RxEarlyThreth: disable Eerly Rx */
	OUTW(dp, RxEarlyThresh, RxEarlyThreshDisable);

	DPRINTF(2, (CE_CONT, "!%s: alta_init_chip: mc0: %b, mc1: %b, ac: %b",
			dp->name,
			INW(dp, MACCtrl0), MC0_BITS,
			INW(dp, MACCtrl1), MC1_BITS,
			INL(dp, AsicCtrl), AC_BITS));
}

static void
alta_start_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Enable statistics */
	OUTW(dp, MACCtrl1, MC1_StatisticsEnable);
#ifndef GEM_DELAYED_START
	/* Set media mode */
	alta_set_media(dp);
#endif
	if (lp->rev_id >= 0x14) {
		OUTB(dp, DebugCtrl1, 0x01);
	}

	ASSERT((dp->rx_ring_dma & 7) == 0);
	OUTL(dp, RxDMAListPtr, dp->rx_ring_dma);

	/* Enable transmitter and receiver */
	OUTW(dp, MACCtrl1, MC1_TxEnable | MC1_RxEnable);

	/* Set InterruptMask */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		lp->intmask = OUR_INTR_BITS;
#ifdef GEM_CONFIG_POLLING
		lp->intmask |= INT_IntRequest;
#endif
		OUTW(dp, IntEnable, lp->intmask);
	}

	lp->countdown = 0;
	OUTW(dp, Countdown, lp->countdown);

	DPRINTF(0, (CE_CONT, "!%s: alta_start_chip: mc0: %b, mc1: %b",
		dp->name, INW(dp, MACCtrl0), MC0_BITS,
		INW(dp, MACCtrl1), MC1_BITS));
	DPRINTF(0, (CE_CONT, "!%s: alta_start_chip: dmactrl: %b",
		dp->name, INL(dp, DMACtrl), DC_BITS));
}

static int
alta_stop_chip(struct gem_dev *dp)
{
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: alta_stop_chip: called", dp->name));

	/* inhibit interrupt */
	OUTW(dp, IntEnable, lp->intmask = 0);

	/* no more statistics */
	OUTW(dp, MACCtrl1, MC1_StatisticsDisable);

	/* stop receiver and transceiver */
	OUTW(dp, MACCtrl1, MC1_RxDisable | MC1_TxDisable);
	drv_usecwait(1000);

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

	i = 0;
	while (++i<100) {
		if ((INW(dp, EepromCtrl) & EEC_Busy) == 0) {
			/* done */
			DPRINTF(4, (CE_CONT, "%s eeprom_read took %d uS",
				dp->name, i*10));
			return INW(dp, EepromData);
		}
		drv_usecwait(10);
	}

	cmn_err(CE_CONT, "%s eeprom_read timeout", dp->name);

	return 0xffff;
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
		val =  alta_read_eeprom(dp, 0x10 + i/2);

		dp->dev_addr.ether_addr_octet[i    ] = val;
		dp->dev_addr.ether_addr_octet[i + 1] = val >> 8;

		OUTW(dp, StationAddress + i, val);
	}
#ifdef notdef
	dp->txthr = ETHERMAX;	/* now 1024 */
#endif
#if DEBUG_LEVEL > 4
	alta_eeprom_dump(dp);
#endif
	if ((!dp->mii_fixedmode) && (INL(dp, AsicCtrl) & AC_PhyMedia) != 0) {
		/* fibre mode, no autonegotiation */
		dp->speed         = GEM_SPD_100;
		dp->full_duplex   = TRUE;
		dp->mii_fixedmode = FALSE;
        }

	return 0;
}

static uint32_t
alta_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return gem_ether_crc_be(addr);
}

static void
alta_set_rx_filter(struct gem_dev *dp)	
{
	uint8_t		mode;
	uint8_t		*newmac;
	int		i;
	uint16_t	mhash[4];
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));
	mhash[0] = mhash[1] = mhash[2] = mhash[3] = 0;

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		/* promiscous */
		mode = RM_ReceiveAllFrames;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0) {
		mode = RM_ReceiveUnicast
		     | RM_ReceiveBroadcast
		     | RM_ReceiveMulticast;
	}
	else {
		/* Normal mode */
		mode = RM_ReceiveUnicast | RM_ReceiveBroadcast;

		/* Make multicast hash table */
		if (dp->mc_count == 0) {
			/* no multicast, do nothing */
		}
		else if (dp->mc_count <= 100) {
			mode |= RM_ReceiveMulticastHash;
			/* make hash table */
			for (i = 0; i < dp->mc_count; i++) {
				uint_t	k;
				/* hash key is 6 bits of LSB in be-crc */
				k = dp->mc_list[i].hash & (4*16 - 1);
				mhash[k / 16] |= 1 << (k % 16);
			}
		}
		else {
			mode |= RM_ReceiveMulticast;
		}
	}

	/* set rx mode */
	DPRINTF(2, (CE_CONT, "!%s: set_rx_filter: setting mode: %b",
		dp->name, mode, RM_BITS));
#ifdef notdef
	/*
	 * XXX - don't disable receiving while changing the rx filter
	 * to avoid dropping packets. ipv6 will offten change multicast
	 * fitler in normal operation.
	 */
	/* stop rx temporary */
	if (dp->nic_active) {
		OUTW(dp, MACCtrl1, MC1_RxDisable);
	}
#endif
	/* set multicast hash table up */
	for (i = 0; i < 4; i++) {
		OUTW(dp, HashTable + i*sizeof(uint16_t), mhash[i]);
	}

	newmac = dp->cur_addr.ether_addr_octet;
	if (bcmp(lp->mac_addr, newmac, ETHERADDRL) != 0) {
		/* need to update the station address */
		for (i = 0; i < ETHERADDRL; i += 2) {
			OUTW(dp, StationAddress + i,
					(newmac[i+1] << 8) | newmac[i]);
		}
		bcopy(newmac, lp->mac_addr, ETHERADDRL);
	}

	OUTB(dp, ReceiveMode, mode);
#ifdef notdef
	/* enable rx again */
	if (dp->nic_active) {
		OUTW(dp, MACCtrl1, MC1_RxEnable);
	}
#endif
}

static void
alta_set_media(struct gem_dev *dp)
{
	uint16_t	old;
	uint16_t	new;
	int		i;

	ASSERT(mutex_owned(&dp->intrlock));
#ifdef GEM_DELAYED_START
	ASSERT(dp->nic_online);
#else
	if (!dp->nic_active) {
		return;
	}
#endif
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
		"!%s: resetting tx and rx to change duplex mode", dp->name));

		OUTL(dp, AsicCtrl, AC_TxReset | AC_RxReset);
		i = 200;
		while (INL(dp, AsicCtrl) & AC_ResetBusy) {
			if (i-- == 0) {
				/* Time out */
				cmn_err(CE_WARN,
				"%s: alta_set_media: tx/rx reset timeout",
				dp->name);
				break;
			}
			drv_usecwait(10);
		}

		OUTW(dp, MACCtrl1, MC1_TxEnable | MC1_RxEnable);
	}
}

static void
alta_get_stats(struct gem_dev *dp)
{
	int		first_coll;
	int		multi_coll;
	int		c;
	int		l;
	volatile int	x;

	DPRINTF(4, (CE_CONT, "!%s: alta_get_stats: called", dp->name));
#ifdef notdef
	ASSERT(mutex_owned(&dp->intrlock));
#endif
	OUTW(dp, MACCtrl1, MC1_StatisticsDisable);

				x= INL(dp, OctetsRecevedOK);
				x= INL(dp, OctetsTransmittedOK);

				x= INW(dp, FramesTransmittedOK);
				x= INW(dp, FramesReceivedOK);

	dp->stats.nocarrier += c = INB(dp, CarrierSenseErrors);
	dp->stats.xmtlatecoll += l = INB(dp, LateCollisionss);
	multi_coll               = INB(dp, MultipleColFrames);
	first_coll               = INB(dp, SingleColFrames);
	dp->stats.defer         += INB(dp, FramesWDeferedXmt);
	dp->stats.missed        += INB(dp, FramesLostRxErrors);
				x= INB(dp, FramesWEXDeferral);
				x= INB(dp, FramesAbortXSClls);
				x= INB(dp, BcstFramesXmtdOK);
				x= INB(dp, BcstFramesRcvdOK);
				x= INB(dp, McstFramesXmtdOK);
				x= INB(dp, McstFramesRcvdOK);

	dp->stats.multi_coll    += multi_coll;
	dp->stats.first_coll    += first_coll;

	/*
	 * Guess total collisions
	 */
	dp->stats.collisions += first_coll + multi_coll*2;

	dp->stats.errxmt += c + l;

	OUTW(dp, MACCtrl1, MC1_StatisticsEnable);
}

/*
 * discriptor  manupiration
 */
static int
alta_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag)
{
	int			i;
	struct alta_tfd		*tfdp;
	struct alta_fragment	*tfp;
	uint32_t		tfd_dma;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	struct alta_dev	*lp = (struct alta_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

#if DEBUG_LEVEL > 3
	/* force to cause interrupt upon tx completion */
	flag |= GEM_TXFLAG_INTR;
#endif
	tfdp = &((struct alta_tfd *)dp->tx_ring)[slot];
#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	"%s: alta_tx_desc_write seqnum: %d, slot %d, frags: %d flag: %d",
		dp->name, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "%d: addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	ddi_put32(h, &tfdp->TxDMANextPtr, 0);
					/* means this is the last descriptor */
	ddi_put32(h, &tfdp->TxFrameControl,
#ifdef USE_FRAMEID
			((flag & GEM_TXFLAG_INTR) != 0 ? TFC_TxIndicate : 0) |
#else
			((flag & GEM_TXFLAG_INTR) != 0 ? TFC_TxDMAIndicate : 0)|
#endif
			(TFC_FrameId & (slot << TFC_FrameIdShift)) |
			TFC_WordAlignDisable);

	/* copy fragment list */
	mark = TFD_TxDMAFragLast; /* last fragment */
	for (i = frags-1, dcp = &dmacookie[i], tfp = &tfdp->TxDMAFrag[i];
			i >= 0;
				dcp--, tfp--, i--, mark = 0) {
		ddi_put32(h, &tfp->Len, dcp->dmac_size | mark);
		ddi_put32(h, &tfp->Addr, dcp->dmac_address);
	}

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tfdp) - dp->rx_ring),
		sizeof(struct alta_tfd), DDI_DMA_SYNC_FORDEV);

	/* link it into the hardware tx list */
	tfd_dma = dp->tx_ring_dma + ((caddr_t)tfdp - (caddr_t)dp->tx_ring);

	ASSERT((tfd_dma & 7) == 0);

#ifdef TEST_TXTIMEOUT
	if ((lp->send_cnt++ % 10000) == 9999) {
		OUTW(dp, MACCtrl1, MC1_TxDisable);
	}
#endif
	if (lp->tx_list_loaded)  {
		/*
		 * Link this to the previous slot. The previous isn't in use,
		 * because number of tx buffers is smaller than the number of
		 * tx descriptors.
		 */
		tfdp = &((struct alta_tfd *)dp->tx_ring)[
						SLOT(slot - 1, TX_RING_SIZE)];
		ddi_put32(h, &tfdp->TxDMANextPtr, tfd_dma);

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tfdp) - dp->rx_ring),
			sizeof(struct alta_tfd), DDI_DMA_SYNC_FORDEV);
	}
	else {
		/*
		 * This is first call after initialization.
		 * Need to tell the head of tx list to the nic.
		 */
		OUTL(dp, TxDMAListPtr, tfd_dma);
		lp->tx_list_loaded = TRUE;
	}

	return 1;
}

static int
alta_rx_desc_write(struct gem_dev *dp, uint_t slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	struct alta_rfd		*rfdp;
	struct alta_fragment	*rfp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		len;
	uint32_t		n;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rfdp = &((struct alta_rfd *)dp->rx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT, "%s: alta_rx_desc_write seqnum: %d, slot %d, frags: %d",
		dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "  frag: %d addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif

	/* copy fragment list */
	for (len = 0, i = frags-1, dcp = dmacookie, rfp = rfdp->RxDMAFrag;
			i--; dcp++, rfp++) {
		len += n = (uint32_t)dcp->dmac_size;
		ddi_put32(h, &rfp->Len, n);
		ddi_put32(h, &rfp->Addr, (uint32_t)dcp->dmac_address);
	}
	/* for last fragment */
	len += n = (uint32_t)dcp->dmac_size;
	ddi_put32(h, &rfp->Len, n | RFD_RxDMALastFrag);
	ddi_put32(h, &rfp->Addr, (uint32_t)dcp->dmac_address);

	ddi_put32(h, &rfdp->RxFrameStatus, len);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORDEV);

	return 1;
}

static uint_t
alta_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct alta_tfd		*tfdp;
	int			frameid;
	uint32_t		tfc;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tfdp = &((struct alta_tfd *)dp->tx_ring)[slot];
#ifdef USE_FRAMEID
	frameid = (INW(dp, TxStatus) & TS_TxFrameId) >> TS_TxFrameIdShift;
	if (slot == frameid + 1) {
		return 0;
	}
	if (slot == frameid) {
		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tfdp) - dp->rx_ring),
			sizeof(struct alta_tfd), DDI_DMA_SYNC_FORKERNEL);
		tfc = ddi_get32(h, &tfdp->TxFrameControl);

		if ((tfc & TFC_TxDMAComplete) == 0) {
			/* not transmitted */
			return 0;
		}
	}
#else
	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tfdp) - dp->rx_ring),
		sizeof(struct alta_tfd), DDI_DMA_SYNC_FORKERNEL);

	tfc = ddi_get32(h, &tfdp->TxFrameControl);

	DPRINTF(2, (CE_CONT,
		"%s: alta_tx_desc_stat: slot:%d, tfs:0x%b",
		dp->name, slot, tfc, TFC_BITS));

	if ((tfc & TFC_TxDMAComplete) == 0) {
		/* not transmitted */
		return 0;
	}
#endif
	return GEM_TX_DONE;
}

static uint64_t
alta_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct alta_rfd		*rfdp;
	uint_t			rfs;
	uint_t			len;
	uint_t			flag;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rfdp = &((struct alta_rfd *)dp->rx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORKERNEL);

	rfs  = ddi_get32(h, &rfdp->RxFrameStatus);
	len  = rfs & RFS_RxDMAFrameLen;
	flag = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT,
		"%s: alta_rx_desc_stat: slot:%d, rfs:0x%b",
		dp->name, slot, rfs, RFS_BITS));

	if ((rfs & RFS_RxDMAComplete) == 0) {
		/* not received */
		return 0;
	}

	if ((rfs & RFS_ERRORS) != 0) {
		/* error packet */
		dp->stats.errrcv++;
		if ((rfs & (RFS_RxAlignmentError
				| RFS_RxFrameError)) != 0) {
			dp->stats.frame++;
		}
		if ((rfs & RFS_RxFIFOOverrun) != 0) {
			dp->stats.overflow++;
		}
		if ((rfs & RFS_RxRuntFrame) != 0) {
			dp->stats.runt++;
		}
		if ((rfs & RFS_RxFCSError) != 0) {
			dp->stats.crc++;
		}
		if ((rfs & (RFS_RxDMAOverflow | RFS_RxOversizedFrame))
								!= 0) {
			dp->stats.frame_too_long++;
		}

		flag = GEM_RX_ERR;
	}

	return flag | len;
}

static void
alta_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct alta_tfd		*tfdp;

	tfdp = &((struct alta_tfd *)dp->tx_ring)[slot];
}

static void
alta_rx_desc_init(struct gem_dev *dp, int slot)
{
	int			i;
	struct alta_rfd		*rfdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rfdp = &((struct alta_rfd *)dp->rx_ring)[slot];

	/* invalidate this rfd */
	ddi_put32(h, &rfdp->RxFrameStatus, RFS_RxDMAComplete);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORDEV);

	/* link it to the previous rfd */
	rfdp = &((struct alta_rfd *)dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)];
	ddi_put32(h, &rfdp->RxDMANextPtr,
		((uint32_t)dp->rx_ring_dma)+slot*sizeof(struct alta_rfd));

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORDEV);
}

static void
alta_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct alta_tfd		*tfdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tfdp = &((struct alta_tfd *)dp->tx_ring)[slot];

	/* do nothing */
	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tfdp) - dp->rx_ring),
		sizeof(struct alta_tfd), DDI_DMA_SYNC_FORKERNEL);
}

static void
alta_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct alta_rfd		*rfdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rfdp = &((struct alta_rfd *)dp->rx_ring)[slot];
#ifdef notdef
	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORKERNEL);
#endif
	/* invalidate this rfd */
	ddi_put32(h, &rfdp->RxFrameStatus, RFS_RxDMAComplete);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rfdp) - dp->rx_ring),
		sizeof(struct alta_rfd), DDI_DMA_SYNC_FORDEV);
}

/*
 * Device depend interrupt handler
 */
static boolean_t
alta_get_tx_status(struct gem_dev  *dp)
{
	int		error = 0;
	int		restart_xmit;
	uint16_t	tx_status;
	int		i;
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	/* Pop all status from  the Tx status stack. */
	while (((tx_status = INW(dp, TxStatus)) & TS_TxComplete) != 0) {

		/* Discard one */
		OUTW(dp, TxStatus, 0);

		if ((tx_status & TS_TxReleaseError) != 0) {
			cmn_err(CE_WARN,
			    "%s: tx release error: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
		}

		if ((tx_status & TS_TxStatusOverflow) != 0) {
			cmn_err(CE_WARN,
			    "%s: tx status overflow: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
		}

		/* update statistics */
		if ((tx_status & TS_TxUnderrun) != 0) {
			/* fifo underflow. TxReset and TxEnable are required */
			cmn_err(CE_WARN,
			    "%s: tx underrun error: tx_status: %b",
			    dp->name, tx_status, TS_BITS);
			dp->stats.underflow++;
			dp->stats.errxmt++;
		}

		if ((tx_status & TS_MaxCollisions) != 0) {
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

	if (error != 0) {
		if ((error & (TS_TxReleaseError | TS_TxUnderrun)) != 0) {
			/* FATAL error */
#ifdef notdef
			/* XXX - the nic has stopped */
			OUTW(dp, MACCtrl1, MC1_TxDisable);
#endif
			lp->need_to_reset = TRUE;
			if ((error & TS_TxUnderrun) != 0) {
				/* increase tx threashold by 64 byte */
				dp->txthr = min(dp->txthr+64, ETHERMAX);
			}
		}
		else {
			if ((error & TS_MaxCollisions) != 0) {
				OUTW(dp, MACCtrl1, MC1_StatisticsEnable);
			}

			OUTW(dp, MACCtrl1, MC1_TxEnable); 
		}
	}

	/*
	 * kick someone who waits for enough space in TX fifo
	 */
	restart_xmit = dp->tx_blocked;
	mutex_exit(&dp->xmitlock);

	return restart_xmit;
}


static u_int
alta_interrupt(struct gem_dev *dp)
{
	uint16_t	int_status;
	u_int		tx_sched = 0;
	struct alta_dev	*lp = (struct alta_dev *)dp->private;

	/*
	 * Read interrupt status
	 * Do not use IntStatusAck here. It clears IntEnable too.
	 */
	int_status = INW(dp, IntStatus);

	if ((int_status & lp->intmask) == 0) {
		/* Not for us */
		return DDI_INTR_UNCLAIMED;
	}

	/*
	 * Acknowledge to the interrupt.
	 * Reading IntStatusAck clears both of IntStatus and IntEnable
	 */
	int_status = INW(dp, IntStatusAck);
	DPRINTF(2, (CE_CONT, "%s: alta_interrupt, int_status: %b",
			   dp->name, int_status, INT_BITS));
	if (!dp->nic_active) {
		/*
		 * the device is not active.
		 * side effect: left interrupt masked.
		 */
		return DDI_INTR_CLAIMED;
	}

#ifdef GEM_CONFIG_POLLING
	if (dp->speed == GEM_SPD_100 &&
	    dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval != 0) {
			/* move to polling mode */
			lp->intmask &= ~INT_RxDMAComplete;
			/* the unit of clock of countdown timer seems 1uS */
			lp->countdown = dp->poll_interval;
			lp->countdown = min(lp->countdown, 0xffff);

			if (lp->last_poll_interval == 0) {
				/*
				 * To schedule the next timer interrupt,
				 * we pretend as we were interrupted from
				 * polling timer
				 */
				int_status |= INT_IntRequest;
			}
		}
		else {
			/* move to normal mode */
			lp->intmask |= INT_RxDMAComplete;

			/* use countdown timer for tx timeout */
			lp->countdown = 0;
		}
	}
	lp->last_poll_interval = dp->poll_interval;
#endif /* GEM_CONFIG_POLLING */

	if ((int_status & INT_IntRequest) != 0) {
		/*
		 * re-load Countdown timer to schedule the next polling
		 * timer interrupt.
		 */
		OUTW(dp, Countdown, lp->countdown);
#ifdef GEM_CONFIG_POLLING
		if (dp->poll_interval > 0) {
			/* force to process rx */
			int_status |= INT_RxDMAComplete;
		}
#endif /* GEM_CONFIG_POLLING */
	}

	if ((int_status & INT_UpdateStats) != 0) {
		/*
		 * Statistics counter overflow
		 */
		alta_get_stats(dp);
	}

	if ((int_status & (INT_RxDMAComplete | INT_MACControlFrame)) != 0) {
		/*
		 * packet was received, or receive error happened
		 */
		gem_receive(dp);
	}

	if ((int_status & INT_TxComplete) != 0) {
		/*
		 * Pull up TxStatus
		 */
		(void) alta_get_tx_status(dp);
	}

	if ((int_status &
		(INT_TxComplete | INT_TxDMAComplete | INT_IntRequest)) != 0) {
		/*
		 * Packets was transfered into TxFIFO or
		 * an error happened.
		 */
		if (gem_tx_done(dp)) {
			tx_sched |= INTR_RESTART_TX;
		}
	}

	if ((int_status & INT_LinkEvent) != 0) {
		DPRINTF(0, (CE_CONT, "%s: %s link event",
			dp->name, __func__));
		(void) gem_mii_link_check(dp);
	}

	if ((int_status & (INT_RxEarly |
#ifndef GEM_CONFIG_POLLING
			   INT_IntRequest |
#endif
			   INT_HostError)) != 0) {
		cmn_err(CE_WARN, "%s: unexpected interrupt: %b",
			dp->name, int_status, INT_BITS);
	}

	if (lp->need_to_reset) {
		mutex_enter(&dp->xmitlock);
		gem_restart_nic(dp, TRUE);
		mutex_exit(&dp->xmitlock);
		tx_sched = INTR_RESTART_TX;
		lp->need_to_reset = FALSE;
	}

	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "%s: alta_inter done: int_status: %b",
			   dp->name, INW(dp, IntStatus), INT_BITS));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IntEnable, lp->intmask);
	}

	return DDI_INTR_CLAIMED | tx_sched;
}

/*
 * HW depend MII routine
 */
#define MDIO_DELAY(dp)    {INB(dp, PhyCtrl);INB(dp, PhyCtrl);}

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
	uint16_t	ret = 0;
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

	/* send 2 ones to phy */
	/* XXX - DL10050 rev. C worked with only 1 one. */
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

	/* send 2 ones to phy */
	for (i = 0; i < 2; i++) {
		OUTB(dp, PhyCtrl, PC_MgmtDir | PC_MgmtData);
		MDIO_DELAY(dp);
		OUTB(dp, PhyCtrl, PC_MgmtDir  | PC_MgmtData | PC_MgmtClk);
		MDIO_DELAY(dp);
	}
}
#undef MDIO_DELAY

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

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "%s%d: altaattach: called", drv_name, unit));

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

	for (i = 0, p = alta_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_WARN,
		"%s: alta_attach: wrong PCI venid/devid (0x%x, 0x%x)",
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
		return gem_resume(dip);

	case DDI_ATTACH:
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
			/* XXX - doesn't work with DFE-580 */
			PCI_ADDR_MEM32,
#else
			PCI_ADDR_IO,
#endif
			&alta_dev_attr, (caddr_t *)&base, &regs_ha)
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

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_desc_size = sizeof(struct alta_tfd) * TX_RING_SIZE;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;

		/* 
		 * XXX - bug fixed 2004/12/06
		 * We cannot use alltx descriptors at the same time.
		 * We must reserve a descriptor which points the head of tx
		 * descriptor list. This is tricky.
		 */ 
		gcp->gc_tx_max_descs_per_pkt = 2;

		gcp->gc_tx_auto_pad  = TRUE;
		gcp->gc_tx_copy_thresh = alta_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_size = sizeof(struct alta_rfd) * RX_RING_SIZE;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;

		/*
		 * XXX - no need to reserve one to point to the head of rx
		 * buffers bacause they can be consist of a ring. 
		 * RxDMAComplete bit in RCF take a role of valid bit.
		 */
		gcp->gc_rx_max_descs_per_pkt = 1;
		gcp->gc_rx_copy_thresh = alta_rx_copy_thresh;
		gcp->gc_rx_buf_max  = gcp->gc_rx_buf_size + 1;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, alta_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, alta_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, alta_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, alta_dma_attr_desc);

		STRUCT_COPY(gcp->gc_dma_attr_txbuf, alta_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, alta_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval   = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout    = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait       = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0; /* default PHY address seems 1 */
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_NONE;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RSA;
		gcp->gc_mii_dont_reset = FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &alta_attach_chip;
		gcp->gc_reset_chip  = &alta_reset_chip;
		gcp->gc_init_chip   = &alta_init_chip;
		gcp->gc_start_chip  = &alta_start_chip;
		gcp->gc_stop_chip   = &alta_stop_chip;
		gcp->gc_multicast_hash = &alta_mcast_hash;
		gcp->gc_set_rx_filter = &alta_set_rx_filter;
		gcp->gc_set_media   = &alta_set_media;
		gcp->gc_get_stats   = &alta_get_stats;
		gcp->gc_interrupt   = &alta_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &alta_tx_desc_write;
		gcp->gc_rx_desc_write = &alta_rx_desc_write;

		gcp->gc_tx_desc_init = &alta_tx_desc_init;
		gcp->gc_rx_desc_init = &alta_rx_desc_init;
		gcp->gc_tx_desc_stat = &alta_tx_desc_stat;
		gcp->gc_rx_desc_stat = &alta_rx_desc_stat;
		gcp->gc_tx_desc_clean = &alta_tx_desc_clean;
		gcp->gc_rx_desc_clean = &alta_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_init  = &gem_mii_init_default;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync  = &alta_mii_sync;
		gcp->gc_mii_read  = &alta_mii_read;
		gcp->gc_mii_write = &alta_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof(struct alta_dev), KM_SLEEP);
		lp->rev_id = revid;
		dp = gem_do_attach(dip, gcp, base, &regs_ha,
					lp, sizeof(*lp));

		kmem_free(gcp, sizeof(*gcp));

		if (dp != NULL) {
			return DDI_SUCCESS;
		}
err_free_mem:
		kmem_free(lp, sizeof(struct alta_dev));
err:
		return DDI_FAILURE;
	}
	return DDI_FAILURE;
}

static int
altadetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	switch (cmd) {
	case DDI_SUSPEND:
		return  gem_suspend(dip);

	case DDI_DETACH:
		return  gem_do_detach(dip);
	}
	return DDI_FAILURE;
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info altaminfo = {
	0,			/* mi_idnum */
	"alta",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	TX_RING_SIZE*ETHERMAX,	/* mi_hiwat */
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
	D_NEW|D_MP	/* cb_flag */
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
	ddi_power	/* devo_power */
};

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

	DPRINTF(2, (CE_CONT, "alta: _init: called"));
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

	DPRINTF(2, (CE_CONT, "alta: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

