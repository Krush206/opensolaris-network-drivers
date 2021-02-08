/*******************************************************************************
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright © 2005 Agere Systems Inc.
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 *
 * SOFTWARE LICENSE
 *
 * This software is provided subject to the following terms and conditions,
 * which you should read carefully before using the software.  Using this
 * software indicates your acceptance of these terms and conditions.  If you do
 * not agree with these terms and conditions, do not use the software.
 *
 * Copyright © 2005 Agere Systems Inc.
 * All rights reserved.
 *
 * Redistribution and use in source or binary forms, with or without
 * modifications, are permitted provided that the following conditions are met:
 *
 * . Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following Disclaimer as comments in the code as
 *    well as in the documentation and/or other materials provided with the
 *    distribution.
 *
 * . Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following Disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * . Neither the name of Agere Systems Inc. nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Disclaimer
 *
 * THIS SOFTWARE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, INFRINGEMENT AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  ANY
 * USE, MODIFICATION OR DISTRIBUTION OF THIS SOFTWARE IS SOLELY AT THE USERS OWN
 * RISK. IN NO EVENT SHALL AGERE SYSTEMS INC. OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, INCLUDING, BUT NOT LIMITED TO, CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ******************************************************************************/

#pragma	ident	"@(#)etge_gem.c 1.4     11/09/19"

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
#include "et1310reg.h"

char	ident[] = "et1310 nic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int etge_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (etge_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#define	INSIDE(slot, head, tail)	\
	(((head) <= (tail)) ?	\
	    ((head) <= (slot) && (slot) < (tail)) :	\
	    ((slot) < (tail) || (head) <= (slot)))

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else /* MAP_MEM */
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif /* MAP_MEM */

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#define	mac_active	mac_state
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif

/*
 * Our configuration
 */
#ifdef TEST_TXDESC_FULL
#undef	TX_BUF_SIZE
#define	TX_BUF_SIZE	16
#endif
#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	128
#endif
#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	512	/* hardware max is 1024, but didn't work */
#endif

#ifdef TEST_RX_EMPTY
#undef	RX_BUF_SIZE
#define	RX_BUF_SIZE	32
#endif
#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256		/* hardware max is 1024 */
#define	RX_RING_SIZE	512
#endif

#define	CONFIG_TX_SINGLE_QUEUE

#ifdef CONFIG_TX_COPY
static int	etge_tx_copy_thresh = INT32_MAX;
#else
static int	etge_tx_copy_thresh = 256;
#endif
#ifdef CONFIG_RX_COPY
static int	etge_rx_copy_thresh = INT32_MAX;
#else
static int	etge_rx_copy_thresh = 256;
#endif

static int etge_fbr0_len = 256;

#define	OUR_INTR_MASK	\
	(INT_PHY_INTERRUPT	\
	| INT_TXMAC_INTERRUPT	\
	| INT_RXDMA_FB_RING1_LOW	\
	| INT_RXDMA_FB_RING0_LOW	\
	| INT_RXDMA_XFR_DONE	\
	| INT_TXDMA_ISR)

/*
 * etgeine chip state
 */
struct etge_dev {
	uint32_t	our_intr_mask;
	uint32_t	rx_ctrl;
	uint32_t	pf_ctrl;
	uint32_t	RegistryRxMemEnd;
	uint8_t		factory_mac[ETHERADDRL];
	uint8_t		mac_addr[ETHERADDRL];

	uint32_t	tx_head;
	uint32_t	tx_tail;
	uint32_t	tx_req_wrap;

	uint32_t	fbr0_head;
	uint32_t	fbr0_tail;
	uint32_t	fbr0_wrap;

	uint32_t	fbr1_head;
	uint32_t	fbr1_tail;
	uint32_t	fbr1_wrap;

	uint32_t	psr_head;
	uint32_t	psr_tail;
	uint32_t	psr_wrap;

	/* io_area offset */
	offset_t	tx_status_offset;
	offset_t	rx_status_offset;
#ifdef notdef
	offset_t	psr_offset;	/* we use rx ring for psr descritors */
#endif
	offset_t	fbr0_offset;
	offset_t	fbr1_offset;

	boolean_t	need_to_reset;
};

/*
 * Macros to identify chip generation.
 */

/* ======================================================== */

/* mii operations */
static void  etge_mii_sync(struct gem_dev *);
static uint16_t  etge_mii_read(struct gem_dev *, uint_t);
static void etge_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int etge_attach_chip(struct gem_dev *);
static int etge_reset_chip(struct gem_dev *);
static int etge_init_chip(struct gem_dev *);
static int etge_start_chip(struct gem_dev *);
static int etge_stop_chip(struct gem_dev *);
static int etge_set_media(struct gem_dev *);
static int etge_set_rx_filter(struct gem_dev *);
static int etge_get_stats(struct gem_dev *);

/* descriptor operations */
static int etge_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void etge_tx_start(struct gem_dev *dp, int slot, int frags);
static void etge_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t etge_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static void etge_rx_start(struct gem_dev *dp, int slot, int frags);
static uint64_t etge_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void etge_tx_desc_init(struct gem_dev *dp, int slot);
static void etge_rx_desc_init(struct gem_dev *dp, int slot);
static void etge_tx_desc_clean(struct gem_dev *dp, int slot);
static void etge_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t etge_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr etge_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr etge_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t etge_dma_attr_buf = {
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

static ddi_dma_attr_t etge_dma_attr_desc = {
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
static void
etge_ioarea_dma_sync(struct gem_dev *dp, off_t start, size_t len, int how)
{
	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(dp->io_area_dma - dp->rx_ring_dma + start),
	    len, how);
}

static void
etge_fbr_desc_dma_sync(struct gem_dev *dp, offset_t offset, int head, int nslot)
{
	int	n;
	int	m;
	struct atge_dev	*lp = dp->private;

	/* sync active descriptors */
	if (nslot == 0) {
		/* no rx descriptor ring */
		return;
	}

	n = dp->gc.gc_rx_ring_size - head;
	if ((m = nslot - n) > 0) {
		(void) etge_ioarea_dma_sync(dp,
		    (off_t)offset,
		    (size_t)(m * sizeof (struct fbr_desc)),
		    DDI_DMA_SYNC_FORDEV);
		nslot = n;
	}

	(void) etge_ioarea_dma_sync(dp,
	    (off_t)(head * sizeof (struct fbr_desc) + offset),
	    (size_t)(nslot * sizeof (struct fbr_desc)),
	    DDI_DMA_SYNC_FORDEV);
}

static void
etge_power_up(struct gem_dev *dp)
{
	OUTL(dp, PM_CSR,
	    PM_CSR_PM_SYSCLK_GATE | PM_CSR_PM_TXCLK_GATE
	    | PM_CSR_PM_RXCLK_GATE);
}

static void
etge_power_down(struct gem_dev *dp)
{
}

static void
etge_disable_interrupts(struct gem_dev *dp)
{
	OUTL(dp, INT_MASK, ~0U);
}

static void
etge_enable_interrupts(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	OUTL(dp, INT_MASK, ~lp->our_intr_mask);
}

static int
etge_reset_chip(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable mac core */
	OUTL(dp, MAC_CFG1,
	    MAC_CFG1_SOFT_RESET | MAC_CFG1_SIM_RESET
	    | MAC_CFG1_RESET_RX_MC | MAC_CFG1_RESET_TX_MC
	    | MAC_CFG1_RESET_RX_FUN | MAC_CFG1_RESET_TX_FUN);

	/* set everything to a reset value */
	OUTL(dp, SW_RESET,
	    SW_RESET_MMC | SW_RESET_MAC_STAT | SW_RESET_MAC |
	    SW_RESET_RXMAC | SW_RESET_TXMAC | SW_RESET_RXDMA | SW_RESET_TXDMA);

	OUTL(dp, MAC_CFG1,
	    MAC_CFG1_RESET_RX_MC  |MAC_CFG1_RESET_TX_MC |
	    MAC_CFG1_RESET_RX_FUN | MAC_CFG1_RESET_TX_FUN);

	/* unreset */
	OUTL(dp, MAC_CFG1, 0);

	etge_disable_interrupts(dp);

	/* invalidate mac address cache */
	bzero(lp->mac_addr, ETHERADDRL);

	return (GEM_SUCCESS);
}

static void
etge_rx_dma_enable(struct gem_dev *dp)
{
	uint32_t	val = 0;

	val |= RXDMA_CSR_FBR1_ENABLE;
	if (dp->rx_buf_len <= 2048) {
		val |= RXDMA_CSR_FBR1_SIZE_2048;
	} else if (dp->rx_buf_len <= 4096) {
		val |= RXDMA_CSR_FBR1_SIZE_4096;
	} else if (dp->rx_buf_len <= 8192) {
		val |= RXDMA_CSR_FBR1_SIZE_8192;
	} else {
		val |= RXDMA_CSR_FBR1_SIZE_16384;
	}

#ifdef USE_FBR0
	val |= RXDMA_CSR_FBR0_ENABLE;
	if (etge_fbr0_len <= 128) {
		val |= RXDMA_CSR_FBR1_SIZE_128;
	} else if (etge_fbr0_len <= 256) {
		val |= RXDMA_CSR_FBR1_SIZE_256;
	} else if (etge_fbr0_len <= 512) {
		val |= RXDMA_CSR_FBR1_SIZE_512;
	} else {
		val |= RXDMA_CSR_FBR1_SIZE_1024;
	}
#endif
	OUTL(dp, RXDMA_CSR, val);

	val = INL(dp, RXDMA_CSR);
	if (val & RXDMA_CSR_HALT_STATUS) {
		drv_usecwait(5);
		val = INL(dp, RXDMA_CSR);
		if (val & RXDMA_CSR_HALT_STATUS) {
			cmn_err(CE_WARN,
			    "!%s: %s: RX Dma failed to exit halt state."
			    " rxdma_csr:0x%x",
			    dp->name, __func__, val);
		}
	}
}

static void
etge_rx_dma_disable(struct gem_dev *dp)
{
	/*
	 * Setup the receive dma configuration register
	 */
	OUTL(dp, RXDMA_CSR,
	    RXDMA_CSR_FBR1_ENABLE | RXDMA_CSR_HALT /* 0x00002001 */);

	if ((INL(dp, RXDMA_CSR) & RXDMA_CSR_HALT_STATUS) == 0) {
		drv_usecwait(5);
		if ((INL(dp, RXDMA_CSR) & RXDMA_CSR_HALT_STATUS) == 0) {
			cmn_err(CE_WARN, "!%s: %s:"
			    "RX Dma failed to enter halt state. CSR 0x%08x",
			    dp->name, __func__, INL(dp, RXDMA_CSR));
		}
	}
}

static void
etge_tx_dma_enable(struct gem_dev *dp)
{
	OUTL(dp, TXDMA_CSR, TXDMA_CSR_SNGL_EPKT_MODE);
}

static void
etge_tx_dma_disable(struct gem_dev *dp)
{
	OUTL(dp, TXDMA_CSR, TXDMA_CSR_SNGL_EPKT_MODE | TXDMA_CSR_HALT);
}

static void
ConfigGlobalRegs(struct gem_dev *dp, boolean_t loopback)
{
	uint32_t	rxq_start_addr;
	uint32_t	rxq_end_addr;
	uint32_t	txq_start_addr;
	uint32_t	txq_end_addr;
	struct etge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (loopback) {
		/*
		 * For PHY Line loopback, the memory is configured
		 * as if Tx and Rx both have all the memory.  This is
		 * because the RxMAC will write data into the space,
		 * and the TxMAC will read it out.
		 */
		rxq_start_addr = 0;
		rxq_end_addr = INTERNAL_MEM_SIZE - 1;
		txq_start_addr = 0;
		txq_end_addr = INTERNAL_MEM_SIZE - 1;

	} else if (dp->mtu < 2048) {
		/*
		 * Tx / RxDMA and Tx/Rx MAC interfaces have a 1k
		 * word block of RAM that the driver can split
		 * between Tx and Rx as it desires.  Our default
		 * is to split it 50/50:
		 */
		rxq_start_addr = 0;
		rxq_end_addr = lp->RegistryRxMemEnd;
		txq_start_addr = lp->RegistryRxMemEnd + 1;
		txq_end_addr = INTERNAL_MEM_SIZE - 1;

	} else if (dp->mtu < 8192) {
		/*
		 * For jumbo packets > 2k in length, but < 8k,
		 * split 50-50.
		 */
		rxq_start_addr = 0;
		rxq_end_addr = INTERNAL_MEM_RX_OFFSET;
		txq_start_addr = INTERNAL_MEM_RX_OFFSET + 1;
		txq_end_addr = INTERNAL_MEM_SIZE - 1;

	} else {
		/*
		 * 9216 is the only packet size greater than 8k that
		 * is available.
		 * The Tx buffer has to be big enough for one whole
		 * packet on the Tx side.
		 * We'll make the Tx 9408, and give the rest to Rx
		 */
		rxq_start_addr = 0x0000;
		rxq_end_addr = 0x01b3;
		txq_start_addr = 0x01b4;
		txq_end_addr = INTERNAL_MEM_SIZE - 1;
	}

	OUTL(dp, RXQ_START_ADDR, rxq_start_addr);
	OUTL(dp, RXQ_END_ADDR, rxq_end_addr);
	OUTL(dp, TXQ_START_ADDR, txq_start_addr);
	OUTL(dp, TXQ_END_ADDR, txq_end_addr);

	/* Initialize the loopback register.  Disable all loopbacks. */
	OUTL(dp, LOOP_BACK, loopback ? 1 : 0);

	/* MSI Register */
	OUTL(dp, MSI_CONFIG, 0);

	/*
	 * By default, disable the watchdog timer. It will be enabled
	 * when a packet is queued.
	 */
	OUTL(dp, WATCHDOG_TIMER, 0);

	DPRINTF(0, (CE_CONT, "!%s: %s: end", dp->name, __func__));
}

static void
ConfigMACRegs1(struct gem_dev *dp)
{
	uint8_t		*m;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * First we need to reset everything.  Write to MAC configuration
	 * register 1 to perform reset.
	 */
	OUTL(dp, MAC_CFG1,
	    MAC_CFG1_SOFT_RESET | MAC_CFG1_SIM_RESET
	    | MAC_CFG1_RESET_RX_MC | MAC_CFG1_RESET_TX_MC
	    | MAC_CFG1_RESET_RX_FUN | MAC_CFG1_RESET_TX_FUN); /* 0xc00f0000 */

	/* Next lets configure the MAC Inter-packet gap register */
	OUTL(dp, MAC_IPG,
	    (0x38 << MAC_IPG_NON_B2B_IPG_1_SHIFT)
	    | (0x58 << MAC_IPG_NON_B2B_IPG_2_SHIFT)
	    | (0x50 << MAC_IPG_MIN_IFG_ENFORCE_SHIFT)
	    | (0x60 << MAC_IPG_B2B_IPG_SHIFT));

	/* Next lets configure the MAC Half Duplex register */
	OUTL(dp, MAC_HFDP,
	    (10 << MAC_HFDP_ALT_BEB_TRUNC_SHIFT)
	    | MAC_HFDP_EXCESS_DEFER
	    | (15 << MAC_HFDP_REXMIT_MAX_SHIFT)
	    | (55 << MAC_HFDP_COLL_WINDOW_SHIFT));

	/* Next lets configure the MAC Interface Control register */
	OUTL(dp, MAC_IF_CTRL, 0);
#if 0
	/* following code were moved into etge_mii_init() */
	/* Let's move on to setting up the mii managment configuration */
	OUTL(dp, MII_MGMT_CFG, 7 << MII_MGMT_CFG_MGMT_CLK_RESET_SHIFT);
#endif
#if 0
	/*
	 * Next lets configure the MAC Station Address register. These
	 * values are read from the EEPROM during initialization and
	 * stored in the adapter structure.  We write what is stored in
	 * the adapter structure to the MAC Station Address registers
	 * high and low.  This station address is used for generating
	 * and checking pause control packets.
	 */
	m = dp->cur_addr.ether_addr_octet;
	OUTL(dp, MAC_STATION_ADDR1,
	    (m[5] << 24) | (m[4] << 16) | (m[3] << 8) | m[2]);
	OUTL(dp, MAC_STATION_ADDR2, (m[1] << 24) | (m[0] << 16));
#endif

	/*
	 * Max ethernet packet in bytes that will passed by the mac
	 * without being truncated.  Allow the MAC to pass 8 more than
	 * our max packet size.  This is 4 for the Ethernet CRC and 4
	 * for the VLAN ID.
	 *
	 * Packets larger than (RegistryJumboPacket) that do not
	 * contain a VLAN ID will be dropped by the Rx function.
	 */
#ifdef GEM_CONFIG_GLDv3
	OUTL(dp, MAC_MAX_FM_LEN,
	    sizeof (struct ether_header) + 4 + dp->mtu + ETHERFCSL);
#else
	OUTL(dp, MAC_MAX_FM_LEN,
	    sizeof (struct ether_header) + dp->mtu + ETHERFCSL);
#endif

	/* clear out MAC config reset */
	OUTL(dp, MAC_CFG1, 0);
}

static void
ConfigMMCRegs(struct gem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	OUTL(dp, MMC_CTRL, MMC_CTRL_MMC_ENABLE);
}

static void
ConfigRxMacRegs(struct gem_dev *dp)
{
	int	i;
	uint8_t	*m;
	uint32_t	val;
	struct etge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* initialize soft copies */
	lp->rx_ctrl = 0;
	lp->pf_ctrl = 0;

	/* Disable the MAC while it is being configured (also disable WOL) */
	lp->rx_ctrl |= RXMAC_CTRL_WOL_DISABLE; /* 0x8 */
	OUTL(dp, RXMAC_CTRL, lp->rx_ctrl);

	/* Initialize WOL to disabled. */
	OUTL(dp, RXMAC_WOL_CTL_CRC0, 0);
	OUTL(dp, RXMAC_WOL_CRC12, 0);
	OUTL(dp, RXMAC_WOL_CRC34, 0);

	/*
	 * We need to set the WOL mask0 - mask4 next. We initialize it to its
	 * default Values of 0x00000000 because there are not WOL masks as of
	 * this time.
	 */
	for (i = 0; i < 5; i++) {
		OUTL(dp, RXMAC_WOL_MASK + i * 16 + 0x0, 0);
		OUTL(dp, RXMAC_WOL_MASK + i * 16 + 0x4, 0);
		OUTL(dp, RXMAC_WOL_MASK + i * 16 + 0x8, 0);
		OUTL(dp, RXMAC_WOL_MASK + i * 16 + 0xc, 0);
	}

	/* Lets setup the WOL Source Address */
	m = dp->cur_addr.ether_addr_octet;
	OUTL(dp, RXMAC_WOL_SA_LO,
	    (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | (m[5] << 0));
	OUTL(dp, RXMAC_WOL_SA_HI, m[0] << 8 | m[1] << 0);

	/* Disable all Packet Filtering */
	OUTL(dp, RXMAC_PF_CTRL, 0);

#ifdef never
	/* Let's initialize the Unicast Packet filtering address */
	if (pAdapter->PacketFilter & ET131X_PACKET_TYPE_DIRECTED) {
		SetupDeviceForUnicast(pAdapter);
		pf_ctrl.bits.filter_uni_en = 1;
	} else {
		pRxMac->uni_pf_addr1.value = 0x00000000;
		pRxMac->uni_pf_addr2.value = 0x00000000;
		pRxMac->uni_pf_addr3.value = 0x00000000;
	}

	/* Let's initialize the Multicast hash */
	if (pAdapter->PacketFilter & ET131X_PACKET_TYPE_ALL_MULTICAST) {
		pf_ctrl.bits.filter_multi_en = 0;
	} else {
		pf_ctrl.bits.filter_multi_en = 1;
		SetupDeviceForMulticast(pAdapter);
	}
#endif

	/* Runt packet filtering.  Didn't work in version A silicon. */
	lp->pf_ctrl |=
	    (ETHERMIN + ETHERFCSL) << RXMAC_PF_CTRL_MIN_PKT_SIZE_SHIFT;
	lp->pf_ctrl |= RXMAC_PF_CTRL_FILTER_FRAG_EN;

	if (dp->mtu > 8192) {
		/* XXX - is mtu right? should we use packet size ? */
		/*
		 * In order to transmit jumbo packets greater than 8k, the FIFO
		 * between RxMAC and RxDMA needs to be reduced in size to (16k -
		 * Jumbo packet size).  In order to implement this, we must use
		 * "cut through" mode in the RxMAC, which chops packets down
		 * into segments which are (max_size * 16).  In this case we
		 * selected 256 bytes, since this is the size of the
		 * PCI-Express TLP's that the 1310 uses.
		 */
		val = RXMAC_MCIF_CTRL_MAX_SEG_SEG_EN
		    | (0x10 <<  RXMAC_MCIF_CTRL_MAX_SEG_MAX_SIZE_SHIFT);
	} else {
		val = 0x0;
	}
	OUTL(dp, RXMAC_MCIF_CTRL_MAX_SEG, val);

	/* Initialize the MCIF water marks */
	OUTL(dp, RXMAC_MCIF_WATER_MARK, 0);

	/* Initialize the MIF control */
	OUTL(dp, RXMAC_MIF_CTL, 0);

	/* Initialize the Space Available Register */
	OUTL(dp, RXMAC_SPACE_AVAIL, 0);

#ifdef notyet
	/*
	 * Initialize the the mif_ctrl register
	 * bit 3  - Receive code error. One or more nibbles were signaled
	 *	    as errors during the reception of the packet.  Clear
	 *	    this bit in Gigabit, set it in 100Mbit.  This was
	 *	    derived experimentally at UNH.
	 * bit 4  - Receive CRC error. The packet’s CRC did not match the
	 *	    internally generated CRC.
	 * bit 5  - Receive length check error. Indicates that frame length
	 *	    field value in the packet does not match the actual data
	 *	    byte length and is not a type field.
	 * bit 16 - Receive frame truncated.
	 * bit 17 - Drop packet enable
	 */
	if (pAdapter->uiLinkSpeed == TRUEPHY_SPEED_100MBPS) {
		val = 0x30038;
	} else {
		val = 0x30030;
	}
	OUTL(dp, RXMAC_MIF_CTL, val);
#endif

	/*
	 * Finally we initialize RxMac to be enabled & WOL disabled.
	 * Packet filter is always enabled since it is where the runt
	 * packets are supposed to be dropped.  For version A silicon,
	 * runt packet dropping doesn't work, so it is disabled in the
	 * pf_ctrl register, but we still leave the packet filter on.
	 */
	OUTL(dp, RXMAC_PF_CTRL, lp->pf_ctrl);

	lp->rx_ctrl |= RXMAC_CTRL_RXMAC_EN;	/* now 0x9 */
	OUTL(dp, RXMAC_CTRL, lp->rx_ctrl);
}

static void
ConfigTxMacRegs(struct gem_dev *dp)
{
	uint32_t	val;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * We need to update the Control Frame Parameters
	 * cfpt - control frame pause timer set to 64 (0x40)
	 * cfep - control frame extended pause timer set to 0x0
	 */
	if (dp->anadv_pause || dp->anadv_asmpause) {
		val = 64 << TXMAC_CF_PARAM_CFPT_SHIFT;
	} else {
		val = 0;
	}
	OUTL(dp, TXMAC_CF_PARAM, val);
}

static void
ConfigRxDmaRegs(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* Halt RXDMA to perform the reconfigure. */
	etge_rx_dma_disable(dp);

	/* Load the completion writeback physical address */
	OUTL(dp, RXDMA_DMA_WB_BASE_HI,
	    (uint32_t)((dp->io_area_dma + lp->rx_status_offset) >> 32));
	OUTL(dp, RXDMA_DMA_WB_BASE_LO,
	    (uint32_t)(dp->io_area_dma + lp->rx_status_offset));

	bzero(dp->io_area + lp->rx_status_offset, sizeof (uint32_t) * 2);

	/*
	 * Set the address and parameters of the packet status ring
	 * into the 1310's registers
	 */
	OUTL(dp, RXDMA_PSR_BASE_HI, (uint32_t)(dp->rx_ring_dma >> 32));
	OUTL(dp, RXDMA_PSR_BASE_LO, (uint32_t)(dp->rx_ring_dma));
	OUTL(dp, RXDMA_PSR_NUM_DES, dp->gc.gc_rx_ring_size - 1);
	OUTL(dp, RXDMA_PSR_FULL_OFFSET, 0);
	/* packet status descriptor low water mark is 15% */
	OUTL(dp, RXDMA_PSR_MIN_DES,
	    ((dp->gc.gc_rx_ring_size - 1) * 15) / 100);

	/*
	 * Set the address and parameters of Free buffer ring 1 (and 0
	 * if required) into the 1310's registers
	 */
	OUTL(dp, RXDMA_FBR1_BASE_HI,
	    (uint32_t)((dp->io_area_dma + lp->fbr1_offset) >> 32));
	OUTL(dp, RXDMA_FBR1_BASE_LO,
	    (uint32_t)(dp->io_area_dma + lp->fbr1_offset));
	OUTL(dp, RXDMA_FBR1_NUM_DES, dp->gc.gc_rx_ring_size - 1);
#if 0
	OUTL(dp, RXDMA_FBR1_FULL_OFFSET, RXDMA_FBR_FULL_OFFSET_WRAP); /* orig */
#else
	OUTL(dp, RXDMA_FBR1_FULL_OFFSET, 0);
#endif
	/* fbr1 descriptor low water mark is 15% */
	OUTL(dp, RXDMA_FBR1_MIN_DES,
	    ((dp->gc.gc_rx_ring_size * 15) / 100) - 1);

#ifdef USE_FBR0
	OUTL(dp, RXDMA_FBR0_BASE_HI,
	    (uint32_t)((dp->io_area_dma + lp->fbr0_offset) >> 32));
	OUTL(dp, RXDMA_FBR0_BASE_LO,
	    (uint32_t)(dp->io_area_dma + lp->fbr0_offset));
	OUTL(dp, RXDMA_FBR0_NUM_DES, dp->gc.gc_rx_ring_size - 1);
	OUTL(dp, RXDMA_FBR0_FULL_OFFSET, RXDMA_FBR_FULL_OFFSET__WRAP);
	/* fbr0 descriptor low water mark is 15% */
	OUTL(dp, RXDMA_FBR0_MIN_DES,
	    ((dp->gc.gc_rx_ring_size * 15) / 100) - 1);
#endif
#if 1
	/*
	 * Program the number of packets we will receive before generating
	 * an interrupt.
	 * For version B silicon, this value gets updated once autoneg is
	 * complete.
	 */
	OUTL(dp, RXDMA_NUM_PKT_DONE, dp->poll_pkt_delay);

	/*
	 * The "time_done" is not working correctly to coalesce interrupts
	 * after a given time period, but rather is giving us an interrupt
	 * regardless of whether we have received packets.
	 * This value gets updated once autoneg is complete.
	 */
	OUTL(dp, RXDMA_MAX_PKT_TIME, dp->poll_pkt_delay * 12 / 4);
#else
	OUTL(dp, RXDMA_NUM_PKT_DONE, 1);
	OUTL(dp, RXDMA_MAX_PKT_TIME, 0);
#endif
}

static void
ConfigTxDmaRegs(struct gem_dev *dp)
{
	uint64_t	tx_status_dma;
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* Load the hardware with the start of the transmit descriptor ring */
	OUTL(dp, TXDMA_PR_BASE_HI, (uint32_t)(dp->tx_ring_dma >> 32));
	OUTL(dp, TXDMA_PR_BASE_LO, (uint32_t)dp->tx_ring_dma);

	/* Initialise the transmit DMA engine */
	OUTL(dp, TXDMA_PR_NUM_DES, dp->gc.gc_tx_ring_size - 1);

	/* Load the completion writeback physical address */
	tx_status_dma = dp->io_area_dma + lp->tx_status_offset;
	OUTL(dp, TXDMA_DMA_WB_BASE_HI, (uint32_t)(tx_status_dma >> 32));
	OUTL(dp, TXDMA_DMA_WB_BASE_LO, (uint32_t)tx_status_dma);
	bzero(dp->io_area + lp->tx_status_offset, sizeof (uint32_t));

	OUTL(dp, TXDMA_SERVICE_REQUEST, 0);
}

static int
etge_init_chip(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* initialize jagcore */
	ConfigGlobalRegs(dp, B_FALSE);

	ConfigMACRegs1(dp);
	ConfigMMCRegs(dp);

	ConfigRxMacRegs(dp);
	ConfigTxMacRegs(dp);

	ConfigRxDmaRegs(dp);
	ConfigTxDmaRegs(dp);

	/* ConfigMacStatRegs(dp); */

	lp->tx_head = 0;
	lp->tx_tail = 0;
	lp->tx_req_wrap = 0;

#ifdef USE_FBR0
	lp->fbr0_head = 0;
	lp->fbr0_tail = 0;
	lp->fbr0_wrap = 0;
#endif
	lp->fbr1_head = 0;
	lp->fbr1_tail = 0;
	lp->fbr1_wrap = 0;

	lp->psr_head = 0;
	lp->psr_tail = 0;
	lp->psr_wrap = 0;

	lp->our_intr_mask = OUR_INTR_MASK;
	lp->need_to_reset = B_FALSE;

	return (GEM_SUCCESS);
}

static uint_t
etge_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is almost higher 7 bits of almost MSB in crc */
	return ((gem_ether_crc_be(addr, ETHERADDRL) >> 23) & 0x7f);
}

static int
etge_set_rx_filter(struct gem_dev *dp)
{
	int	h;
	int	i;
	uint8_t	*m;
	uint32_t	mhash[4];
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d, rxmode:%b mc_count:%d",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, dp->rxmode, RXMODE_BITS,
	    dp->mc_count));

	mhash[0] = mhash[1] = mhash[2] = mhash[3] = 0;
	lp->rx_ctrl &= ~RXMAC_CTRL_PKT_FILTER_DISABLE;
	lp->pf_ctrl &= ~(RXMAC_PF_CTRL_FILTER_BROAD_EN |
	    RXMAC_PF_CTRL_FILTER_MULTI_EN | RXMAC_PF_CTRL_FILTER_UNI_EN);

	if (dp->rxmode & RXMODE_PROMISC) {
		lp->rx_ctrl |= RXMAC_CTRL_PKT_FILTER_DISABLE;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count >= 128/2) {
		lp->pf_ctrl &= ~RXMAC_PF_CTRL_FILTER_MULTI_EN;
		lp->pf_ctrl |= RXMAC_PF_CTRL_FILTER_UNI_EN;
	} else if (dp->mc_count > 0) {
		/*
		 * make the hash table to select the interresting
		 * multicast address only.
		 */
		lp->pf_ctrl |= RXMAC_PF_CTRL_FILTER_MULTI_EN
		    | RXMAC_PF_CTRL_FILTER_UNI_EN;
		for (i = 0; i < dp->mc_count; i++) {
			/* hash table is 128 = 2^7 bit width */
			h = dp->mc_list[i].hash;
			mhash[h / 32] |= 1U << (h % 32);
		}
	}

	/* pass all packets while we are changing filter mode */
	OUTL(dp, RXMAC_CTRL, lp->rx_ctrl | RXMAC_CTRL_PKT_FILTER_DISABLE);

	/* update mac address */
	m = dp->cur_addr.ether_addr_octet;
	if (bcmp(m, lp->mac_addr, ETHERADDRL) != 0) {
		OUTL(dp, RXMAC_UNI_PF_ADDR1,
		    (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5]);
		OUTL(dp, RXMAC_UNI_PF_ADDR2,
		    (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5]);
		OUTL(dp, RXMAC_UNI_PF_ADDR3,
		    (m[0] << 24) | (m[1] << 16) | (m[0] << 8) | m[1]);
		bcopy(m, lp->mac_addr, ETHERADDRL);
	}

	/* update hardware multicast hash table */
	for (i = 0; i < 4; i++) {
		OUTL(dp, RXMAC_MULTI_HASH + sizeof (uint32_t) * i,
		    mhash[i]);
	}

	OUTL(dp, RXMAC_PF_CTRL, lp->pf_ctrl);
	OUTL(dp, RXMAC_CTRL, lp->rx_ctrl);

	return (GEM_SUCCESS);
}

static int
etge_set_media(struct gem_dev *dp)
{
	int		i;
	uint32_t	cfg1;
	uint32_t	cfg2;
	uint32_t	ifctrl;
	uint32_t	mif_ctrl;
	struct etge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active));

	cfg1 = INL(dp, MAC_CFG1);
	cfg2 = INL(dp, MAC_CFG2);
	ifctrl = INL(dp, MAC_IF_CTRL);

	/*
	 * Initialize the the mif_ctrl register
	 * bit 3  - Receive code error. One or more nibbles were signaled
	 *	    as errors during the reception of the packet.  Clear
	 *	    this bit in Gigabit, set it in 100Mbit.  This was
	 *	    derived experimentally at UNH.
	 * bit 4  - Receive CRC error. The packet’s CRC did not match the
	 *	    internally generated CRC.
	 * bit 5  - Receive length check error. Indicates that frame length
	 *	    field value in the packet does not match the actual data
	 *	    byte length and is not a type field.
	 * bit 16 - Receive frame truncated.
	 * bit 17 - Drop packet enable
	 */
	cfg2 &= ~MAC_CFG2_IF_MODE;
	if (dp->speed == GEM_SPD_1000) {
		cfg2 |= MAC_CFG2_IF_MODE_1000;
		ifctrl &= ~MAC_IF_CTRL_PHY_MODE;
		mif_ctrl = 0x30030;
	} else {
		cfg2 |= MAC_CFG2_IF_MODE_100_10;
		ifctrl |= MAC_IF_CTRL_PHY_MODE;
		mif_ctrl = 0x30038;
	}
#if 0
	/* XXX - following code were moved into etge_start_chip() */
	/*  We need to enable Rx/Tx */
	cfg1 |= MAC_CFG1_RX_ENABLE | MAC_CFG1_TX_ENABLE;
#endif
	/*
	 *  Set up flow control
	 */
	cfg1 |= MAC_CFG1_TX_FLOW;

	switch (dp->flow_control) {
	case FLOW_CONTROL_RX_PAUSE:
	case FLOW_CONTROL_SYMMETRIC:
		cfg1 |= MAC_CFG1_RX_FLOW;
		break;

	default:
		cfg1 &= ~MAC_CFG1_RX_FLOW;
		break;
	}

	cfg1 &= ~MAC_CFG1_LOOP_BACK;
	OUTL(dp, MAC_CFG1, cfg1);

	/*
	 * Now we need to initialize the MAC Configuration 2 register
	 */
	cfg2 = (cfg2 & ~MAC_CFG2_PREAMBLE_LEN) |
	    (7 << MAC_CFG2_PREAMBLE_LEN_SHIFT);

	cfg2 &= ~MAC_CFG2_HUGE_FRAME;

	/*
	 * LENGTH FIELD CHECKING bit4:
	 * Set this bit to cause the MAC to check the frame's length
	 * field to ensure it matches the actual data field length.
	 * Clear this bit if no length field checking is desired. Its
	 * default is ‘0’.
	 */
	cfg2 |= MAC_CFG2_LEN_CHECK;

	cfg2 |= MAC_CFG2_PAD_CRC | MAC_CFG2_CRC_ENABLE;

	/* setup duplex mode */
	if (dp->full_duplex) {
		cfg2 |= MAC_CFG2_FULL_DUPLEX;
		ifctrl &= ~MAC_IF_CTRL_GHD_MODE;
	} else {
		cfg2 &= ~MAC_CFG2_FULL_DUPLEX;
		ifctrl |= MAC_IF_CTRL_GHD_MODE;
	}

	OUTL(dp, MAC_IF_CTRL, ifctrl);
	OUTL(dp, MAC_CFG2, cfg2);
	OUTL(dp, RXMAC_MIF_CTL, mif_ctrl);

#if 0
	/* XXX - following code were moved into etge_start_chip() */
	for (i = 0; i < 100; i++) {
		if ((INL(dp, MAC_CFG1) &
		    (MAC_CFG1_SYNCD_RX_EN | MAC_CFG1_SYNCD_TX_EN)) ==
		    (MAC_CFG1_SYNCD_RX_EN | MAC_CFG1_SYNCD_TX_EN)) {
			goto mac_cfg1_write_ok;
		}
		drv_usecwait(10);
	}
	cmn_err(CE_WARN,
	    "!%s: %s: Syncd bits did not respond correctly cfg1 word 0x%08x",
	    dp->name, __func__, INL(dp, MAC_CFG1));

mac_cfg1_write_ok:

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: "
	    "speed %d, dup %d, CFG1 0x%08x, CFG2 0x%08x, if_ctrl 0x%08x",
	    dp->name, __func__,
	    dp->speed, dp->full_duplex,
	    INL(dp, MAC_CFG1),
	    INL(dp, MAC_CFG2),
	    INL(dp, MAC_IF_CTRL)));

	/* Enable TXMAC */
	OUTL(dp, TXMAC_CTL,
	    TXMAC_CTL_TXMAC_EN | TXMAC_CTL_FC_DISABLE | INL(dp, TXMAC_CTL));

	/* Ready to start the RXDMA/TXDMA engine */
	et131x_rx_dma_enable(dp);
	et131x_tx_dma_enable(dp);
#endif
	return (GEM_SUCCESS);
}

static int
etge_start_chip(struct gem_dev *dp)
{
	int	i;
	struct etge_dev	*lp = dp->private;

	/*  We need to enable Rx/Tx */
	OUTL(dp, MAC_CFG1,
	    MAC_CFG1_RX_ENABLE | MAC_CFG1_TX_ENABLE
	    | INL(dp, MAC_CFG1));

	for (i = 0; i < 100; i++) {
		if ((INL(dp, MAC_CFG1) &
		    (MAC_CFG1_SYNCD_RX_EN | MAC_CFG1_SYNCD_TX_EN)) ==
		    (MAC_CFG1_SYNCD_RX_EN | MAC_CFG1_SYNCD_TX_EN)) {
			goto mac_cfg1_write_ok;
		}
		drv_usecwait(10);
	}
	cmn_err(CE_WARN,
	    "!%s: %s: "
	    "Syncd bits did not respond correctly cfg1 word 0x%08x",
	    dp->name, __func__, INL(dp, MAC_CFG1));

mac_cfg1_write_ok:

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: "
	    "speed %d, dup %d, CFG1 %b, CFG2 %b, if_ctrl %b",
	    dp->name, __func__,
	    dp->speed, dp->full_duplex,
	    INL(dp, MAC_CFG1), CFG1_BITS,
	    INL(dp, MAC_CFG2), CFG2_BITS,
	    INL(dp, MAC_IF_CTRL), IF_CTRL_BITS));

	/* Enable TXMAC */
	OUTL(dp, TXMAC_CTL,
	    TXMAC_CTL_TXMAC_EN | TXMAC_CTL_FC_DISABLE |
	    INL(dp, TXMAC_CTL));

	/* Ready to start the RXDMA/TXDMA engine */
	etge_rx_dma_enable(dp);
	etge_tx_dma_enable(dp);

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		etge_enable_interrupts(dp);
	}

	return (GEM_SUCCESS);
}

static int
etge_stop_chip(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	etge_rx_dma_disable(dp);
	etge_tx_dma_disable(dp);

	etge_disable_interrupts(dp);

	(void) etge_reset_chip(dp);

	return (GEM_SUCCESS);
}

static int
etge_attach_chip(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	if (!gem_get_mac_addr_conf(dp)) {
		bcopy(lp->factory_mac, &dp->dev_addr, ETHERADDRL);
	}
	lp->RegistryRxMemEnd = 0x2bc;	/* 700 in decimal */

#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
#ifdef CONFIG_OFFLOAD	/* it didn't work */
	/* dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4; */
	dp->misc_flag |= GEM_CKSUM_FULL_IPv4;
	/* dp->misc_flag |= GEM_CKSUM_PARTIAL; */
#endif

	return (GEM_SUCCESS);
}

static int
etge_get_stats(struct gem_dev *dp)
{
	return (GEM_SUCCESS);
}

static int
etge_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	struct tx_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	int	i;
	uint32_t	mark;
	uint64_t	hiaddr;
	uint64_t	loaddr;
	uint32_t	vtag_len;
	const uint32_t	tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d flags: %ld",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
#if DEBUG_LEVEL > 20
	flags |= GEM_TXFLAG_INTR;
#endif
	/*
	 * write tx descriptor(s)
	 */
	mark = TXD3_F;
	if (flags & GEM_TXFLAG_INTR) {
		mark |= TXD3_IR;
	}
#ifdef CONFIG_OFFLOAD /* it didn't work */
	if (flags & GEM_TXFLAG_IPv4) {
		mark |= TXD3_IPA;
	}
	if (flags & GEM_TXFLAG_TCP) {
		mark |= TXD3_TCPA;
	} else if (flags & GEM_TXFLAG_UDP) {
		mark |= TXD3_UDPA;
	}
#endif
	dcp = &dmacookie[0];
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

#ifdef GEM_CONFIG_TX_DIRECT
	i = frags;
	while (--i > 0) {

		vtag_len = dcp->dmac_size;
		hiaddr = dcp->dmac_laddress;
		loaddr = (uint32_t)hiaddr;
		hiaddr >>= 32;
		tdp->txd0 = LE_32(hiaddr);
		tdp->txd1 = LE_32(loaddr);
		tdp->txd2 = LE_32(vtag_len);
		tdp->txd3 = LE_32(mark);

		/* prepare for the next */
		slot = SLOT(slot + 1, tx_ring_size);
		tdp = &((struct tx_desc *)dp->tx_ring)[slot];
		dcp++;
		mark &= ~TXD3_F;
	}

#endif
	/* make the last fragment */
	vtag_len = dcp->dmac_size;
	mark |= TXD3_L;
	hiaddr = dcp->dmac_laddress;
	loaddr = (uint32_t)hiaddr;
	hiaddr >>= 32;
	tdp->txd0 = LE_32(hiaddr);
	tdp->txd1 = LE_32(loaddr);
	tdp->txd2 = LE_32(vtag_len);
	tdp->txd3 = LE_32(mark);

	return (frags);
}
static void
etge_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	const uint32_t	tx_ring_size = dp->gc.gc_tx_ring_size;
	struct etge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, mac_active:%x, imr:%b",
	    dp->name, __func__, dp->mac_active,
	    lp->our_intr_mask, INT_BITS));

	/* flush tx descriptors we made */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	/* update tail index */
	lp->tx_tail = start_slot + nslot;
	if (lp->tx_tail >= tx_ring_size) {
		/* invert wrap bit */
		lp->tx_req_wrap ^= TXDMA_SERVICE_REQUEST_WRAP;
		lp->tx_tail -= tx_ring_size;
	}

	/* notify the new tail position to the nic */
	OUTL(dp, TXDMA_SERVICE_REQUEST, lp->tx_tail | lp->tx_req_wrap);
}
#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
etge_tx_desc_head(struct gem_dev *dp)
{
	struct etge_dev	*lp = dp->private;
#ifdef NEVER
	/* XXX - it sometimes caused tx timeout */
	etge_ioarea_dma_sync(dp,
	    lp->tx_status_offset, sizeof (uint32_t),
	    DDI_DMA_SYNC_FORKERNEL);
	lp->tx_head =
	    (*(uint32_t *)(dp->io_area + lp->tx_status_offset))
	    & TXDMA_SERVICE_COMPLETE_MASK;
#else
	lp->tx_head = INL(dp, TXDMA_NEW_SERVICE_COMPLETE)
	    & TXDMA_SERVICE_COMPLETE_MASK;
#endif
	return (lp->tx_head);
}
#else
static uint_t
etge_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct etge_dev	*lp = dp->private;

	slot = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(slot, lp->tx_head, lp->tx_tail)) {
		/* update tx_head */
#ifdef NEVER
		/* XXX - it sometimes caused tx timeout */
		etge_ioarea_dma_sync(dp,
		    lp->tx_status_offset, sizeof (uint32_t),
		    DDI_DMA_SYNC_FORKERNEL);
		lp->tx_head =
		    (*(uint32_t *)(dp->io_area + lp->tx_status_offset))
		    & TXDMA_SERVICE_COMPLETE_MASK;
#else
		lp->tx_head = INL(dp, TXDMA_NEW_SERVICE_COMPLETE)
		    & TXDMA_SERVICE_COMPLETE_MASK;
#endif
		/* test again */
		if (INSIDE(slot, lp->tx_head, lp->tx_tail)) {
			/* not yet */
			return (0);
		}
	}
	return (GEM_TX_DONE);
}
#endif /* GEM_CONFIG_TX_HEAD_PTR */

static void
etge_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	uint64_t	hiaddr;
	uint64_t	loaddr;
	struct		fbr_desc	*fdp;
	struct etge_dev	*lp = dp->private;
#if DEBUG_LEVEL > 2
	int		i;

	cmn_err(CE_CONT, "!%s: %s, slot %d, frags: %d",
	    dp->name, __func__, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif

	fdp = &((struct fbr_desc *)(dp->io_area + lp->fbr1_offset))[slot];

	/*
	 * write a RX descriptor
	 */
	hiaddr = dmacookie->dmac_laddress;
	loaddr = (uint32_t)hiaddr;
	hiaddr >>= 32;

	fdp->fbd0 = LE_32(loaddr);
	fdp->fbd1 = LE_32(hiaddr);
	fdp->fbd2 = LE_32(slot);	/* buffer index */
}

static void
etge_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint32_t	full;
	struct etge_dev	*lp = dp->private;

	(void) etge_fbr_desc_dma_sync(dp, lp->fbr1_offset, start_slot, nslot);
	full = start_slot + nslot;
	if (full >= dp->gc.gc_rx_ring_size) {
		full -= dp->gc.gc_rx_ring_size;
		lp->fbr1_wrap ^= RXDMA_FBR_FULL_OFFSET_WRAP;
		lp->psr_wrap ^= RXDMA_PSR_FULL_OFFSET_WRAP;
	}
	lp->fbr1_tail = full;
	lp->psr_tail = full;

	OUTL(dp, RXDMA_FBR1_FULL_OFFSET,  lp->fbr1_tail | lp->fbr1_wrap);
	OUTL(dp, RXDMA_PSR_FULL_OFFSET,  lp->psr_tail | lp->psr_wrap);
}

static uint64_t
etge_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	rxstat0;
	uint32_t	rxstat1;
	uint32_t	psrd0;
	uint32_t	psrd1;
	uint_t		ri;
	uint_t		bi;
	uint64_t	ret;
	uint64_t	len;
	struct	psr_desc	*pdp;
	struct etge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:[%d-%d]: psr:[%d-%d])",
	    dp->name, __func__,
	    slot, slot + ndesc, lp->psr_head, lp->psr_tail));

	ASSERT(ndesc == 1);

	/*
	 * check packet status descriptor
	 */
	if (INSIDE(slot, lp->psr_head, lp->psr_tail)) {
		etge_ioarea_dma_sync(dp,
		    lp->rx_status_offset, sizeof (uint32_t) * 2,
		    DDI_DMA_SYNC_FORKERNEL);

		rxstat0 = ((uint32_t *)(dp->io_area + lp->rx_status_offset))[0];
		rxstat1 = ((uint32_t *)(dp->io_area + lp->rx_status_offset))[1];

		/* decode rx status */
		lp->fbr0_head = (rxstat0 & RXSTAT_FBR0_OFFSET)
		    >> RXSTAT_FBR0_OFFSET_SHIFT;
		lp->fbr1_head = (rxstat0 & RXSTAT_FBR1_OFFSET)
		    >> RXSTAT_FBR1_OFFSET_SHIFT;
		lp->psr_head = (rxstat1 & RXSTAT_PSR_OFFSET)
		    >> RXSTAT_PSR_OFFSET_SHIFT;

		DPRINTF(2, (CE_CONT, "!%s: %s: rx_status:%08x %08x, "
		    "psr:[%d-%d], fbr0:[%d-%d], fbr1:[%d-%d]",
		    dp->name, __func__,
		    rxstat0, rxstat1,
		    lp->psr_head, lp->psr_tail,
		    lp->fbr0_head, lp->fbr0_tail,
		    lp->fbr1_head, lp->fbr1_tail));

		if (INSIDE(slot, lp->psr_head, lp->psr_tail)) {
			return (0);
		}
	}

	ret = GEM_RX_DONE;

	pdp = &((struct psr_desc *)dp->rx_ring)[slot];
	psrd0 = pdp->psrd0;
	psrd1 = pdp->psrd1;
	psrd0 = LE_32(psrd0);
	psrd1 = LE_32(psrd1);

	ri = (psrd1 & PSRD1_RI) >> PSRD1_RI_SHIFT;
	bi = (psrd1 & PSRD1_BI) >> PSRD1_BI_SHIFT;
	if (!(ri == 1 && bi == slot)) {
		/* fatal error happened */
		lp->need_to_reset = B_TRUE;
		cmn_err(CE_CONT, "!%s: %s: wrong ring/buffer: %x %x",
		    dp->name, __func__, psrd0, psrd1);
		ret |= GEM_RX_ERR;
	}

	if ((psrd0 & PSRD0_ASW_OK) == 0) {
		cmn_err(CE_CONT, "!%s: %s: corrupted packet %x %x",
		    dp->name, __func__, psrd0, psrd1);
		ret |= GEM_RX_ERR;
	}

	len = psrd1 & PSRD1_LENGTH;
	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}

	return (ret | len);
}

static void
etge_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

static void
etge_rx_desc_init(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

/*
 * Device depend interrupt handler
 */
static uint_t
etge_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flags = 0;
	struct etge_dev	*lp = dp->private;

	isr = INL(dp, INT_STATUS);

	if ((isr & lp->our_intr_mask) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: Interrupt, isr: %b, active:%x",
	    dp->name, isr, INT_BITS, dp->mac_active));

	isr &= lp->our_intr_mask;

	if (!IS_MAC_ONLINE(dp)) {
		/* the device is not active, no more interrupts */
#if 0
		lp->our_intr_mask = 0;
#endif
		etge_disable_interrupts(dp);
		return (DDI_INTR_CLAIMED);
	}
#ifdef GEM3

	if (isr & (INT_TXDMA_ISR | INT_TXMAC_INTERRUPT)) {
		/* packets have been transmitted or transmit error happened */
		if (gem_tx_done(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}
#endif

	if (isr & INT_RXDMA_XFR_DONE) {
		/* packet was received, or receive error happened */
		(void) gem_receive(dp);
	}
#ifndef GEM3

	if (isr & (INT_TXDMA_ISR | INT_TXMAC_INTERRUPT)) {
		/* packets have been transmitted or transmit error happened */
		if (gem_tx_done(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}
#endif

	if (lp->need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flags |= INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	return (DDI_INTR_CLAIMED | flags);
}

/*
 * HW depend MII routines
 */
static void
etge_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
etge_mii_read(struct gem_dev *dp, uint_t reg)
{
	int	i;
	uint16_t	val;
	uint32_t	addr_saved;
	uint32_t	cmd_saved;

	/* save original value to restore them later */
	addr_saved = INL(dp, MII_MGMT_ADDR);
	cmd_saved = INL(dp, MII_MGMT_CMD);

	/* stop the current operation */
	OUTL(dp, MII_MGMT_CMD, 0);

	OUTL(dp, MII_MGMT_ADDR,
	    (dp->mii_phy_addr << MII_MGMT_ADDR_PHY_ADDR_SHIFT)
	    | (reg << MII_MGMT_ADDR_REG_ADDR_SHIFT));

	OUTL(dp, MII_MGMT_CMD, MII_MGMT_CMD_READ_CYCLE);

	i = 0;
	while (INL(dp, MII_MGMT_INDICATOR) &
	    (MII_MGMT_INDICATOR_NOT_VALID | MII_MGMT_INDICATOR_BUSY)) {
		if (i++ > 100) {
			/* timeout */
			cmn_err(CE_WARN, "!%s %s: "
			    "reg 0x%08x could not be read, status:0x%08x",
			    dp->name, __func__,
			    reg, INL(dp, MII_MGMT_INDICATOR));
			val = 0;
			goto done;
		}
		drv_usecwait(50);
	}
	val = INL(dp, MII_MGMT_STAT) & MII_MGMT_STAT_MASK;

done:
	/* stop the read operation */
	OUTL(dp, MII_MGMT_CMD, 0);

	/* restore the original value */
	OUTL(dp, MII_MGMT_ADDR, addr_saved);
	OUTL(dp, MII_MGMT_CMD, cmd_saved);

	return (val);
}

static void
etge_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int	i;
	uint32_t	addr_saved;
	uint32_t	cmd_saved;

	/* save original value to restore them later */
	addr_saved = INL(dp, MII_MGMT_ADDR);
	cmd_saved = INL(dp, MII_MGMT_CMD);

	/* stop the current operation */
	OUTL(dp, MII_MGMT_CMD, 0);

	OUTL(dp, MII_MGMT_ADDR,
	    (dp->mii_phy_addr << MII_MGMT_ADDR_PHY_ADDR_SHIFT)
	    | (reg << MII_MGMT_ADDR_REG_ADDR_SHIFT));

	OUTL(dp, MII_MGMT_CTRL, val);

	i = 0;
	while (INL(dp, MII_MGMT_INDICATOR) & MII_MGMT_INDICATOR_BUSY) {
		if (i++ > 100) {
			/* timeout */
			cmn_err(CE_WARN, "!%s %s: "
			    "reg 0x%08x could not be written, "
			    "status:0x%08x",
			    dp->name, __func__,
			    reg, INL(dp, MII_MGMT_INDICATOR));
			break;
		}
		drv_usecwait(50);
	}

	/* stop the write operation */
	OUTL(dp, MII_MGMT_CMD, 0);

	/* restore the original value */
	OUTL(dp, MII_MGMT_ADDR, addr_saved);
	OUTL(dp, MII_MGMT_CMD, cmd_saved);
}

static void
etge_enable_phy(struct gem_dev *dp)
{
	/* mii managment configuration */
	OUTL(dp, MII_MGMT_CFG, 7 << MII_MGMT_CFG_MGMT_CLK_RESET_SHIFT);
}

static int
etge_mii_probe(struct gem_dev *dp)
{
	etge_enable_phy(dp);
	return (gem_mii_probe_default(dp));
}

static int
etge_mii_init(struct gem_dev *dp)
{
	etge_enable_phy(dp);
	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
etgeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct etge_dev	*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint32_t		val;
	uint32_t		cap_ptr;
	uint32_t		cap;
	uint32_t		ps;
	uint32_t		ilr;
	uint32_t		pcie_cap;
	offset_t		io_off;
	uint32_t		mac[2];

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: etgeattach: called at time:%d (%s)",
	    drv_name, unit, ddi_get_lbolt(), ident));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		goto err;
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);

	DPRINTF(0, (CE_CONT,
	    "!%s%d: ilr 0x%08x, latency_timer:0x%02x, cache line size:%x",
	    drv_name, unit,
	    pci_config_get32(conf_handle, PCI_CONF_ILINE),
	    pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER),
	    pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ)));

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* get factory mac address */
	mac[0] = pci_config_get32(conf_handle, 0xa4);
	mac[1] = pci_config_get16(conf_handle, 0xa8);

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
#ifdef notdef
		/*
		 * increase max read request 2:512, 3:1024, 4:2048
		 */
		if ((devcsr & (7 << 12)) < (3 << 12)) {
			devcsr = (devcsr & ~(7 << 12)) | (3 << 12);
			pci_config_put32(conf_handle, pcie_cap + 8, devcsr);
		}
#endif
	}

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/*
		 * Map in the device registers.
		 */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_MEM64, PCI_ADDR_MASK,
		    &etge_dev_attr, (caddr_t *)&base, &regs_ha)
		    != DDI_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s%d: gem_pci_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		lp = kmem_zalloc(sizeof (struct etge_dev), KM_SLEEP);

		/*
		 * ioarea allocation
		 */
		io_off = 0;

		/* tx status block */
		lp->tx_status_offset = io_off = ROUNDUP(io_off, 8);
		io_off += sizeof (uint32_t);
		DPRINTF(0, (CE_CONT, "!%s%d: tx_status_offset:0x%x",
		    drv_name, unit, lp->tx_status_offset));

		/* rx free buffer descriptors */
#ifdef USE_FBR0
		lp->fbr0_offset = io_off = ROUNDUP(io_off, 8);
		io_off += sizeof (struct fbr_desc) * RX_RING_SIZE;
		DPRINTF(0, (CE_CONT, "!%s%d: fbr0_offset:0x%x",
		    drv_name, unit,  lp->fbr0_offset));
#endif
		lp->fbr1_offset = io_off = ROUNDUP(io_off, 8);
		io_off += sizeof (struct fbr_desc) * RX_RING_SIZE;
		DPRINTF(0, (CE_CONT, "!%s%d: fbr1_offset:0x%x",
		    drv_name, unit,  lp->fbr1_offset));
#ifdef notdef
		/* packet status descriptors */
		lp->psr_offset = io_off = ROUNDUP(io_off, 8);
		io_off += sizeof (struct psr_desc) * RX_RING_SIZE;
		DPRINTF(0, (CE_CONT, "!%s%d: psr_offset:0x%x",
		    drv_name, unit,  lp->psr_offset));
#endif
		/* rx status block */
		lp->rx_status_offset = io_off = ROUNDUP(io_off, 8);
		io_off += sizeof (uint32_t) * 2;
		DPRINTF(0, (CE_CONT, "!%s%d: rx_status_offset:0x%x",
		    drv_name, unit,  lp->rx_status_offset));

		lp->factory_mac[0] = (uint8_t)mac[0];
		lp->factory_mac[1] = (uint8_t)(mac[0] >> 8);
		lp->factory_mac[2] = (uint8_t)(mac[0] >> 16);
		lp->factory_mac[3] = (uint8_t)(mac[0] >> 24);
		lp->factory_mac[4] = (uint8_t)mac[1];
		lp->factory_mac[5] = (uint8_t)(mac[2] >> 8);

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* configuration on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = min(8, GEM_MAXTXFRAGS);
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = 4; /* 16 byte */
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_buf_size = gcp->gc_tx_ring_size;
		gcp->gc_tx_buf_limit = gcp->gc_tx_ring_limit;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = etge_tx_copy_thresh;
		gcp->gc_tx_min_fraglen = 8;

#ifdef GEM_CONFIG_JUMBO_FRAME
		/* if rx_buf_len > 2048, rx align is 4096 */
		gcp->gc_rx_buf_align = 4096 - 1;
#else
		gcp->gc_rx_buf_align = 2048 - 1;
#endif
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 3;	/* 8 byte */
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = etge_rx_copy_thresh;

		/* io_area */
		gcp->gc_io_area_size = io_off;

		/* map attributes */
		gcp->gc_dev_attr = etge_dev_attr;
		gcp->gc_buf_attr = etge_buf_attr;
		gcp->gc_desc_attr = etge_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = etge_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = etge_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = etge_dma_attr_buf;
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
		/* need to reset PHY for autonegotiation over 1000Mbps */
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_hw_link_detection = B_FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &etge_attach_chip;
		gcp->gc_reset_chip = &etge_reset_chip;
		gcp->gc_init_chip = &etge_init_chip;
		gcp->gc_start_chip = &etge_start_chip;
		gcp->gc_stop_chip = &etge_stop_chip;
		gcp->gc_multicast_hash = &etge_mcast_hash;
		gcp->gc_set_rx_filter = &etge_set_rx_filter;
		gcp->gc_set_media = &etge_set_media;
		gcp->gc_get_stats = &etge_get_stats;
		gcp->gc_interrupt = &etge_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &etge_tx_desc_write;
		gcp->gc_rx_desc_write = &etge_rx_desc_write;
		gcp->gc_tx_start = &etge_tx_start;
		gcp->gc_rx_start = &etge_rx_start;

#ifdef GEM_CONFIG_TX_HEAD_PTR
		gcp->gc_tx_desc_stat = NULL;
		gcp->gc_tx_desc_head = &etge_tx_desc_head;
#else
		gcp->gc_tx_desc_stat = &etge_tx_desc_stat;
#endif
		gcp->gc_rx_desc_stat = &etge_rx_desc_stat;
		gcp->gc_tx_desc_init = &etge_tx_desc_init;
		gcp->gc_rx_desc_init = &etge_rx_desc_init;
		gcp->gc_tx_desc_clean = &etge_tx_desc_init;
		gcp->gc_rx_desc_clean = &etge_rx_desc_init;
		gcp->gc_get_packet = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_probe = &etge_mii_probe;
		gcp->gc_mii_init = &etge_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &etge_mii_sync;
		gcp->gc_mii_read = &etge_mii_read;
		gcp->gc_mii_write = &etge_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* MSI/MSIX interrupts */
#ifdef CONFIG_INTR_MSI
		gcp->gc_nintrs_req = 1;
#else
		gcp->gc_nintrs_req = 0;
#endif
		gcp->gc_max_mtu = 9216
		    - ETHERFCSL - sizeof(struct ether_header);
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof(struct ether_header);

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
		kmem_free(lp, sizeof (struct etge_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
etgedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(etge_ops, etgeattach, etgedetach);
#else
static	struct module_info etgeminfo = {
	0,			/* mi_idnum */
	"etge",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit etgerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&etgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit etgewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&etgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	etge_info = {
	&etgerinit,	/* st_rdinit */
	&etgewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_etge_ops = {
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
	&etge_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops etge_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	etgeattach,	/* devo_attach */
	etgedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_etge_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&etge_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!etge: _init: called"));
	gem_mod_init(&etge_ops, "etge");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&etge_ops);
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

	DPRINTF(2, (CE_CONT, "!etge: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&etge_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
