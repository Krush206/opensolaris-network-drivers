/*
 * sige_gem.c : SiS190/191 integrated fast/giga ethernet MAC driver for Solaris
 *
 * Copyright (c) 2005-2010 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)sige_gem.c 1.13     10/05/14"

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
#include "sis190reg.h"

char	ident[] = "sis190/191 driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int sige_debug = DEBUG_LEVEL;
#if DEBUG_LEVEL > 4
#define	CONS	"^"
#else
#define	CONS	"!"
#endif
#define	DPRINTF(n, args)	if (sige_debug > (n)) cmn_err args
#else
#define	CONS	"!"
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC			(drv_usectohz(1*1000000))

#define	ROUNDUP(x, a)		(((x) + (a) - 1) & ~((a) - 1))

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif /* MAP_MEM */

#define	PHY_ID_MARVELL88E1111	0x01410cc1
#define	PHY_ID_BCM5461_1	0x00206021

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif
/*
 * Our configuration
 */
#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef	TX_RING_SIZE
#define	TX_RING_SIZE	256
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif
#ifndef	RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	MAXTXFRAGS	(min(8, GEM_MAXTXFRAGS))
#define	MAXRXFRAGS	1

#define	OUR_INTR_BITS	( \
	INT_LINK | \
	INT_RXIDLE | INT_RXDONE | INT_TXDONE | \
	INT_TXHALT | INT_RXHALT \
)

static int	sige_tx_copy_thresh = 256;
static int	sige_rx_copy_thresh = 256;

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*chip_name;
};

/*
 * Chip dependant MAC state
 */
struct sige_dev {
	/* misc HW information */
	struct chip_info	*chip;
	boolean_t		phy_rgmii;
	boolean_t		mac_in_apc;

	uint32_t		tx_desc_flags;

	/* register shadows */
	uint32_t		our_intr_bits;
	uint32_t		tx_control;
	uint32_t		rx_control;
	uint8_t			mac_addr[ETHERADDRL];
	uint16_t		rx_mac_ctrl;

	boolean_t		tx_running;
};

/*
 * Hardware information
 */
struct chip_info sige_chiptbl[] = {
	0x1039,	0x0190,	"SiS190",
	0x1039,	0x0191,	"SiS191",
};
#define	CHIPTABLESIZE	(sizeof (sige_chiptbl)/sizeof (struct chip_info))

/* ======================================================== */

/* mii operations */
static void  sige_mii_sync(struct gem_dev *);
static uint16_t  sige_mii_read(struct gem_dev *, uint_t);
static void sige_mii_write(struct gem_dev *, uint_t, uint16_t);
/* nic operations */
static int sige_reset_chip(struct gem_dev *);
static int sige_init_chip(struct gem_dev *);
static int sige_start_chip(struct gem_dev *);
static int sige_stop_chip(struct gem_dev *);
static int sige_set_media(struct gem_dev *);
static int sige_set_rx_filter(struct gem_dev *);
static int sige_get_stats(struct gem_dev *);
static int sige_attach_chip(struct gem_dev *);

/* descriptor operations */
static int sige_tx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void sige_tx_start(struct gem_dev *dp, int slot, int frags);
static void sige_rx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags);
static uint_t sige_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t sige_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void sige_tx_desc_init(struct gem_dev *dp, int slot);
static void sige_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t sige_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr sige_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr sige_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t sige_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t sige_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
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
static uint16_t
sige_read_eeprom(struct gem_dev *dp, int offset)
{
	int		i;
	uint32_t	ret;
	ASSERT(offset <= EI_OFFSET);
	OUTL(dp, ROMCMD, EI_REQ | EI_OP_RD | (offset << EI_OFFSET_SHIFT));
	drv_usecwait(500);

	i = 0;
	while ((ret = INL(dp, ROMCMD)) & EI_REQ) {
		if (i++ > 1000) {
			/* timeout */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0xffff);
		}
		drv_usecwait(100);
	}

	return ((ret & EI_DATA) >> EI_DATA_SHIFT);
}

static int
sige_get_mac_addr_eeprom(struct gem_dev *dp)
{
	int		i;
	uint16_t	val;
	uint8_t		*mac;
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s called", dp->name, __func__));

	val = sige_read_eeprom(dp, EEPROMSignature);

	if (val == 0xffff || val == 0x0000) {
		cmn_err(CE_WARN, "!%s: invalid eeprom signature (%x)",
		    dp->name, val);
		return (GEM_FAILURE);
	}

	mac = dp->dev_addr.ether_addr_octet;

	for (i = 0; i < ETHERADDRL; i += 2) {
		val = sige_read_eeprom(dp, EEPROMMACAddr + i/2);
		mac[i + 0] = (uint8_t)val;
		mac[i + 1] = (uint8_t)(val >> 8);
	}

	/* check RGMII/GMII mode */
	lp->phy_rgmii = (sige_read_eeprom(dp, EEPROMInfo) & 0x80) != 0;

	return (GEM_SUCCESS);
}

static dev_info_t *
sige_search_pci_dev_subr(dev_info_t *cur_node,
    uint_t vendor_id, uint_t device_id)
{
	dev_info_t	*child_id;
	dev_info_t	*ret;
	uint_t		vid, did;

	ASSERT(cur_node != NULL);

	/* check brothers */
	do {
		vid = ddi_prop_get_int(DDI_DEV_T_ANY, cur_node,
		    DDI_PROP_DONTPASS, "vendor-id", -1);
		did = ddi_prop_get_int(DDI_DEV_T_ANY, cur_node,
		    DDI_PROP_DONTPASS, "device-id", -1);

		if (vid == vendor_id && did == device_id) {
			/* found */
			return (cur_node);
		}

		/* check children */
		if ((child_id = ddi_get_child(cur_node)) != NULL) {
			if ((ret = sige_search_pci_dev_subr(
			    child_id, vendor_id, device_id)) != NULL) {
				return (ret);
			}
		}

	} while ((cur_node = ddi_get_next_sibling(cur_node)) != NULL);

	/* not found */
	return (NULL);
}

static dev_info_t *
sige_search_pci_dev(int vendor_id, int device_id)
{
	return (sige_search_pci_dev_subr(
	    ddi_root_node(), vendor_id, device_id));
}

static int
sige_get_mac_addr_apc(struct gem_dev *dp)
{
	int		i;
	uint_t		reg;
	dev_info_t	*isa_bridge;
	ddi_acc_handle_t isa_handle;
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s called", dp->name, __func__));

	if (isa_bridge = sige_search_pci_dev(0x1039, 0x965)) {
		goto found;
	}
	if (isa_bridge = sige_search_pci_dev(0x1039, 0x966)) {
		goto found;
	}
	if (isa_bridge = sige_search_pci_dev(0x1039, 0x968)) {
		goto found;
	}

	cmn_err(CE_WARN, "!%s: %s: isa bridge not found",
	    dp->name, __func__);
	return (GEM_FAILURE);

found:
	if (pci_config_setup(isa_bridge, &isa_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: pci_config_setup failed",
		    dp->name, __func__);
		return (GEM_FAILURE);
	}

	/* enable port 0x78 & 0x79 to access APC registgers */
	reg = pci_config_get8(isa_handle, 0x48);
	pci_config_put8(isa_handle, 0x48, reg & ~0x02);
	drv_usecwait(50);
	(void) pci_config_get8(isa_handle, 0x48);

	/* get factory mac addresss */
	for (i = 0; i < ETHERADDRL; i++) {
		outb(0x78, 0x09 + i);
		dp->dev_addr.ether_addr_octet[i] = inb(0x79);
	}

	/* check MII/RGMII */
	outb(0x78, 0x12);
	lp->phy_rgmii = ((inb(0x79) & 0x80) != 0);

	/* close to access APC registers */
	pci_config_put8(isa_handle, 0x48, reg);
	pci_config_teardown(&isa_handle);

	return (GEM_SUCCESS);
}

static int
sige_reset_chip(struct gem_dev *dp)
{
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s called", dp->name, __func__));

	lp->our_intr_bits = 0;
	OUTL(dp, IMR, lp->our_intr_bits);
	OUTL(dp, ISR, 0xffffffff);

	/* soft reset */
	OUTL(dp, ICR, 0x8000);
	FLSHL(dp, ICR);
	drv_usecwait(100);
	OUTL(dp, ICR, 0);

	/* stop mac */
	lp->tx_control = 0x1a00;
	lp->rx_control = 0x1a00;
	OUTL(dp, TxCTL, lp->tx_control);
	OUTL(dp, RxCTL, lp->rx_control);

	OUTL(dp, IMR, lp->our_intr_bits);
	OUTL(dp, ISR, 0xffffffff);

	/* clear the register shadow for mac_addr */
	bzero(lp->mac_addr, sizeof (lp->mac_addr));

	OUTL(dp, GMIICTL, 0);
	if (lp->phy_rgmii) {
		OUTL(dp, RGMIIDelay, 0x0441);	/* was  0x4441 */
		OUTL(dp, RGMIIDelay, 0x0440);	/* was  0x4440 */
	}
#if 0
{
	clock_t	now;

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: TIMER:0x%x", dp->name, __func__, INL(dp, TIMER)));
	OUTL(dp, TIMER, 0xffffffff);
	OUTL(dp, ISR, INT_TIMER);
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: TIMER:%x, ISR:%b", dp->name, __func__,
	    INL(dp, TIMER), INL(dp, ISR), INT_BITS));
	drv_usecwait(1000);
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: TIMER:%x, ISR:%b", dp->name, __func__,
	    INL(dp, TIMER), INL(dp, ISR), INT_BITS));
	drv_usecwait(1000);
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: TIMER:%x, ISR:%b", dp->name, __func__,
	    INL(dp, TIMER), INL(dp, ISR), INT_BITS));
	OUTL(dp, TIMER, 1000);
	now = ddi_get_lbolt();
	OUTL(dp, ISR, INT_TIMER);
	while ((INL(dp, ISR) & INT_TIMER) == 0) {
		drv_usecwait(10);
	}

	cmn_err(CE_CONT, "!%s: %s: %d mS",
	    dp->name, __func__,  (ddi_get_lbolt() - now)*10);

	OUTL(dp, TIMER, 0);
}
#endif
	return (GEM_SUCCESS);
}

static int
sige_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s called", dp->name, __func__));

	OUTL(dp, TxDESC, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, RxDESC, (uint32_t)dp->rx_ring_dma);

	OUTL(dp, TxMacCTL, 0x60);
	lp->rx_mac_ctrl = 0x0002;
#ifdef CONFIG_RX_PAD
	lp->rx_mac_ctrl |= RM_PAD_ENB;
#endif
#ifdef CONFIG_CRC_STRIP
	lp->rx_mac_ctrl |= RM_STRIP_FCS;
#endif
#ifdef CONFIG_VLAN_HW
	lp->rx_mac_ctrl |= RM_STRIP_VLAN;
#endif
	OUTW(dp, RxMacCTL, lp->rx_mac_ctrl);

	OUTL(dp, 0x6c, 0);
	OUTL(dp, RxWOL, 0);
	OUTL(dp, RxWOLData, 0);

	lp->tx_running = B_FALSE;

	return (GEM_SUCCESS);
}

static uint_t
sige_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL) & (64 - 1));
}

static int
sige_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint16_t	mode;
	uint64_t	hash_tbl;
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mode = 0;
	hash_tbl = 0ull;

	/* Set Receive filter control register */
	mode = RM_AB | RM_AM | RM_AP; /* broadcast, multicast, my physical */

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* accept all physcal address */
		mode |= RM_AAP;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* enable all multicast address */
		hash_tbl = ~0ull;
	} else {
		/* make hash table for selected multicast address */
		for (i = 0; i < dp->mc_count; i++) {
			hash_tbl |= 1ull << dp->mc_list[i].hash;
		}
	}

	/* Load station address if it has been changed */
	if (bcmp(dp->cur_addr.ether_addr_octet, lp->mac_addr, ETHERADDRL)) {
		/* temporally disable my station address before chaning it */
		OUTW(dp, RxMacCTL, lp->rx_mac_ctrl);
		FLSHW(dp, RxMacCTL);

		/* update my station address */
		bcopy(dp->cur_addr.ether_addr_octet, lp->mac_addr, ETHERADDRL);
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, RxMacAddr + i, lp->mac_addr[i]);
		}
	}

	/* Load Multicast hash table */
	OUTL(dp, MCASTHASH + 0, (uint32_t)hash_tbl);
	OUTL(dp, MCASTHASH + 4, (uint32_t)(hash_tbl >> 32));

	/* Load new rx filter mode */
	OUTW(dp, RxMacCTL, lp->rx_mac_ctrl | mode);

	return (GEM_SUCCESS);
}

static int
sige_start_chip(struct gem_dev *dp)
{
	struct sige_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* enable interrupt */
	lp->our_intr_bits = OUR_INTR_BITS;
#ifdef OPT_TxPOLL
	lp->our_intr_bits |= INT_TXIDLE;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IMR, lp->our_intr_bits);
	}

	/* enable tx */
	lp->tx_control |= TxCTL_EN;
	OUTL(dp, TxCTL, lp->tx_control);

	/* enable rx */
	lp->rx_control |= 0x0c | RxCTL_EN;
	OUTL(dp, RxCTL, lp->rx_control | RxCTL_POLL);

	return (GEM_SUCCESS);
}

static int
sige_stop_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct sige_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* stop tx and rx state machines */
	lp->tx_control = 0x1a00;
	lp->rx_control = 0x1a00;
	OUTL(dp, TxCTL, lp->tx_control);
	OUTL(dp, RxCTL, lp->rx_control);

	/* disable interrupts */
	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);
#ifdef notdef
	/* stop tx and rx state machines */
	val = INL(dp, ICR);
	OUTL(dp, ICR, val | 0x8000);
	FLSHL(dp, ICR);
	drv_usecwait(50);
	OUTL(dp, ICR, val & ~0x8000);
#endif
	drv_usecwait(2000);

	return (GEM_SUCCESS);
}

static int
sige_set_media(struct gem_dev *dp)
{
	uint32_t	val;
	struct sige_dev	*lp = dp->private;
	extern int gem_speed_value[];

	DPRINTF(0, (CE_CONT, "!%s: %s: %s duplex, %d Mbps, StationCTL:%x",
	    dp->name, __func__,
	    dp->full_duplex ? "full" : "half",
	    gem_speed_value[dp->speed], INL(dp, StationCTL)));

	val = INL(dp, StationCTL) & ~(0x0f000000 | SC_FDX | SC_SPEED);

	switch (dp->speed) {
	case GEM_SPD_1000:
		val |= SC_SPEED_1000;
		break;

	case GEM_SPD_100:
		val |= SC_SPEED_100;
		break;

	case GEM_SPD_10:
		val |= SC_SPEED_10;
		break;
	}

	if (dp->full_duplex) {
		val |= SC_FDX;
	}

	/* XXX: MII phy operation mode? */
	if (dp->speed == GEM_SPD_1000) {
		val |= 0x07000000;
	} else {
		val |= 0x04000000;
	}
	if (lp->phy_rgmii && dp->mii_phy_id == PHY_ID_BCM5461_1) {
		val |= 0x03000000;
	}
	OUTL(dp, StationCTL, val);

	DPRINTF(0, (CE_CONT, "!%s: %s: new StationCTL:%x",
	    dp->name, __func__, val));

	/*
	 * setup tx descriptor flags
	 */
	lp->tx_desc_flags = TDC_TXOWN | TDC_PADEN | TDC_CRCEN | TDC_DEFEN;
	if (dp->speed == GEM_SPD_1000) {
		lp->tx_desc_flags |= TDC_BSTEN;
	}
#ifdef notdef
	if (!dp->full_duplex) {
		lp->tx_desc_flags |= TDC_COLEN | TDC_BKFEN;
	}
#endif
	return (GEM_SUCCESS);
}

static int
sige_get_stats(struct gem_dev *dp)
{
	/* do nothing */
	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
sige_tx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	uint32_t		pktsize = 0;
	uint32_t		mark;
	struct sige_txdesc	*tdp;
	struct sige_dev		*lp = dp->private;
#ifdef CONFIG_CKSUM_OFFLOAD
	uint32_t		mss;
#endif

#if DEBUG_LEVEL > 10
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum:%d, slot%d, frags:%d flags:%llx",
	    dp->name, __func__,
	    dp->tx_desc_tail, slot, frags, flags);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr:0x%x, len:%d",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
	if (flags & GEM_TXFLAG_INTR) {
		mark = lp->tx_desc_flags | TDC_TXINT;
	} else {
		mark = lp->tx_desc_flags;
	}
#ifdef CONFIG_CKSUM_OFFLOAD
	/* only 1 bit in tcp, udp and ip, is allowed */
	mss = (flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;
	if (mss) {
		mark |= TDC_LSEN;
		pktsize |= mss << TX_MSS_SHIFT;
	} else if (flags & GEM_TXFLAG_UDP) {
		/* it requires psuedo header check sum */
		mark |= TDC_UDPCS;
	} else if (flags & GEM_TXFLAG_TCP) {
		/* it requires psuedo header check sum */
		mark |= TDC_TCPCS;
	}
	if (flags & GEM_TXFLAG_IPv4) {
		mark |= TDC_IPCS;
	}
#endif
	for (i = frags - 1; i > 0; i--) {
		tdp = &((struct sige_txdesc *)dp->tx_ring)[
		    SLOT(slot+i, dp->gc.gc_tx_ring_size)];

		pktsize += dmacookie[i].dmac_size;
		tdp->td_bufptr = LE_32(dmacookie[i].dmac_address);
		tdp->td_pktsize = 0;
		tdp->td_flags = (tdp->td_flags & LE_32(TX_RING_EOD))
		    | LE_32(dmacookie[i].dmac_size);
		tdp->td_cmdsts = LE_32(TDC_TXOWN);
	}
	pktsize += dmacookie[0].dmac_size;

	if (flags & GEM_TXFLAG_HEAD) {
		mark &= ~TDC_TXOWN;
	}
#ifdef CONFIG_VLAN_HW
	if (flags & GEM_TXFLAG_VTAG) {
		pktsize |= TX_INS_VTAG;
		mark |= (GEM_TXFLAG_VTAG & flags) >> GEM_TXFLAG_VTAG_SHIFT;
	}
#endif
	tdp = &((struct sige_txdesc *)dp->tx_ring)[slot];
	tdp->td_bufptr = LE_32(dmacookie[0].dmac_address);
	tdp->td_pktsize = LE_32(pktsize);
	tdp->td_flags = (tdp->td_flags & LE_32(TX_RING_EOD))
	    | LE_32(dmacookie[0].dmac_size);
	tdp->td_cmdsts = LE_32(mark);

	return (frags);
}

static void
sige_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct sige_dev		*lp = dp->private;

	/* add a NOT OWNed descriptor to terminate tx list */
	((struct sige_txdesc *)dp->tx_ring)[
	    SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size)].td_cmdsts = 0;
	gem_tx_desc_dma_sync(dp,
	    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
	    nslot, DDI_DMA_SYNC_FORDEV);

	/* activate the first discriptor of the tx list */
	((struct sige_txdesc *)dp->tx_ring)[start_slot].td_cmdsts |=
	    LE_32(TDC_TXOWN);
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/*
	 * Let the Transmit Buffer Manager Fill state machine active.
	 */
	if (!lp->tx_running) {
		OUTL(dp, TxCTL, lp->tx_control | TxCTL_POLL);
#ifdef OPT_TxPOLL
		lp->tx_running = B_TRUE;
#endif
	}
}

static void
sige_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct sige_rxdesc	*rdp;
	struct sige_dev		*lp = dp->private;
#if DEBUG_LEVEL > 2
	int			i;

	ASSERT(frags == 1);

	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/* for the last slot of the packet */
	rdp = &((struct sige_rxdesc *)dp->rx_ring)[slot];

	rdp->rd_bufptr = LE_32(dmacookie->dmac_address);
	rdp->rd_stssize = 0;
	rdp->rd_flags = (rdp->rd_flags & LE_32(RX_RING_EOD))
	    | LE_32(dmacookie->dmac_size);

#ifdef CONFIG_CKSUM_OFFLOAD
	rdp->rd_pktinfo = LE_32(
	    RDI_RXOWN | RDI_RXINT | RDI_TCPON | RDI_UDPON | RDI_IPON);
#else
	rdp->rd_pktinfo = LE_32(RDI_RXOWN | RDI_RXINT);
#endif
}

static uint_t
sige_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct sige_txdesc	*tdp;
	uint32_t		status;
	int			i;
	int			cols;
#ifdef DEBUG_LEVEL
	struct sige_dev		*lp = dp->private;
	clock_t			delay;
#endif
	/* check status of the first descriptor */
	tdp = &((struct sige_txdesc *)dp->tx_ring)[slot];

	status = tdp->td_cmdsts;
	status = LE_32(status);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: slot:%d, status:0x%b",
	    dp->name, ddi_get_lbolt(), __func__, slot, status, TDS_BITS));

	if (status & TDC_TXOWN) {
		/*
		 * not yet transmitted
		 */
		return (0);
	}
#if DEBUG_LEVEL > 3
	delay = (ddi_get_lbolt() - dp->tx_buf_head->txb_stime) * 10;
	if (delay >= 50) {
		DPRINTF(0, (CE_NOTE, "!%s: tx deferred %d mS: slot %d",
		    dp->name, delay, slot));
	}
#endif
	/*
	 *  collect statictics
	 */
	if (status & (TDS_OWC | TDS_FIFO | TDS_CRS | TDS_ABT)) {

		/* failed to transmit the packet */

		DPRINTF(2, (CE_CONT, "!%s: Transmit error, Tx status %b",
		    dp->name, status, TDS_BITS));

		dp->stats.errxmt++;

		if (status & TDS_FIFO) {
			dp->stats.underflow++;
		} else if (status & TDS_CRS) {
			dp->stats.nocarrier++;
		} else if (status & TDS_OWC) {
			dp->stats.xmtlatecoll++;
		} else if (!dp->full_duplex && (status & TDS_ABT)) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		} else {
			dp->stats.xmit_internal_err++;
		}
	} else if (!dp->full_duplex) {
		/* half duplex: update collision counters */
		cols = (status & TDS_COLLS) - 1;

		if (cols > 0) {
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* (cols > 1) */ {
				dp->stats.multi_coll++;
			}
			dp->stats.collisions += cols;
		}
	}

	return (GEM_TX_DONE);
}

static uint64_t
sige_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct sige_rxdesc	*rdp;
	uint64_t		len;
	uint64_t		flag;
	uint32_t		rxinfo;
	uint32_t		status;

	flag = GEM_RX_DONE;

	rdp = &((struct sige_rxdesc *)dp->rx_ring)[slot];

	rxinfo = rdp->rd_pktinfo;
	rxinfo = LE_32(rxinfo);
	status = LE_32(rdp->rd_stssize);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: slot:%d, rxinfo:0x%b, status:0x%b",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, rxinfo, RDI_BITS, status, RDS_BITS));

	if (rxinfo & RDI_RXOWN) {
		/*
		 * No more received packets because
		 * the buffer is owned by NIC.
		 */
		return (0);
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d"
	    " %s: slot:%d, rxinfo:0x%b, status:0x%b, bufptr:0x%x, flags:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, rxinfo, RDI_BITS, status, RDS_BITS,
	    LE_32(rdp->rd_bufptr), LE_32(rdp->rd_flags)));

#define	RX_ERR_BITS \
	(RDS_COLON | RDS_NIBON | RDS_OVRUN | RDS_MIIER | \
	RDS_LIMIT | RDS_SHORT | RDS_ABORT | RDS_CRCOK | RDS_DESCS)

	if ((status & RX_ERR_BITS) != (RDS_CRCOK | (1 << RDS_DESCS_SHIFT))) {
		/*
		 * we received a packet with error.
		 */
		DPRINTF(0, (CE_CONT,
		    "!%s: Corrupted packet received, buffer status: %b",
		    dp->name, status, RDS_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;
		if (status & RDS_OVRUN) {
			dp->stats.overflow++;
		} else if ((status & RDS_LIMIT) ||
		    ((status & RDS_DESCS) >> RDS_DESCS_SHIFT) > 1) {
			dp->stats.frame_too_long++;
		} else if (status & RDS_SHORT) {
			dp->stats.runt++;
		} else if (status & (RDS_NIBON | RDS_MIIER | RDS_COLON)) {
			dp->stats.frame++;
		} else if ((status & RDS_CRCOK) == 0) {
			dp->stats.crc++;
		} else {
			dp->stats.rcv_internal_err++;
		}

		return (flag | GEM_RX_ERR);
	}

	/*
	 * this packet was received without errors
	 */
#ifdef CONFIG_CKSUM_OFFLOAD
	if ((rxinfo & (RDI_IPON | RDI_IPOK)) == (RDI_IPON | RDI_IPOK)) {
		if ((rxinfo & (RDI_TCPON | RDI_TCPOK)) ==
		    (RDI_TCPON | RDI_TCPOK)) {
			flag |= GEM_RX_CKSUM_TCP;
		} else if ((rxinfo & (RDI_UDPON | RDI_UDPOK)) ==
		    (RDI_UDPON | RDI_UDPOK)) {
			flag |= GEM_RX_CKSUM_UDP;
		} else {
			flag |= GEM_RX_CKSUM_IPv4;
		}
	}
#endif
	len = status & RDS_SIZE;
	if ((rxinfo & RDI_CRCOFF) == 0 && len - ETHERFCSL > 0) {
		len -= ETHERFCSL;
	}
#ifdef CONFIG_RX_PAD
	if ((rxinfo & RDI_PREADD) && len - RM_PAD_LEN > 0) {
		len -= RM_PAD_LEN;
	}
#endif
#ifdef CONFIG_VLAN_HW
	if (status & RDS_TAGON) {
		flag |= ((uint64_t)rxinfo & RDI_VTAG) << GEM_RX_VTAG_SHIFT;
	}
#endif
#if DEBUG_LEVEL > 10
	{
		int	i;
		uint8_t	*bp = (void *)&dp->rx_buf_head->rxb_buf[0];

		cmn_err(CE_CONT, "!%s: len:%d (org:%d)",
		    dp->name, len, status & RDS_SIZE);

		for (i = 0; i < 20; i += 10) {
			cmn_err(CE_CONT,
			    "!%02x %02x %02x %02x %02x"
			    " %02x %02x %02x %02x %02x",
			    bp[0], bp[1], bp[2], bp[3], bp[4],
			    bp[5], bp[6], bp[7], bp[8], bp[9]);
			bp += 10;
		}
	}
#endif
	return (flag | (len & GEM_RX_LEN));
}

static void
sige_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct sige_txdesc	*tdp;

	tdp = &((struct sige_txdesc *)dp->tx_ring)[slot];

	tdp->td_pktsize = 0;
	tdp->td_cmdsts = 0;
	tdp->td_bufptr = 0;
	tdp->td_flags = (slot == dp->gc.gc_tx_ring_size - 1) ? TX_RING_EOD : 0;
}

static void
sige_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct sige_rxdesc	*rdp;

	rdp = &((struct sige_rxdesc *)dp->rx_ring)[slot];

	rdp->rd_stssize = 0;
	rdp->rd_pktinfo = 0;
	rdp->rd_bufptr = 0;
	rdp->rd_flags = (slot == RX_RING_SIZE - 1) ? RX_RING_EOD : 0;
}

/*
 * Device depend interrupt handler
 */
static uint_t
sige_interrupt(struct gem_dev *dp)
{
	uint32_t	intsrc;
	uint_t		flags = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct sige_dev	*lp = dp->private;

	/* read reason */
	intsrc = INL(dp, ISR);

	if ((intsrc & lp->our_intr_bits) == 0) {
		/* not for us, clear unused interrupts */
		if (intsrc) {
			OUTL(dp, ISR, intsrc);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	OUTL(dp, ISR, intsrc);
	FLSHL(dp, ISR);

	DPRINTF(1, (CE_CONT,
	    "!%s: time:%d %s: called: intsrc:0x%b rx_active_head: %d",
	    dp->name, ddi_get_lbolt(), __func__,
	    intsrc, INT_BITS, dp->rx_active_head));

	if (!IS_MAC_ONLINE(dp)) {
		/* the device is going to stop */
		lp->our_intr_bits = 0;
		OUTL(dp, IMR, 0);
		FLSHL(dp, IMR);
		return (DDI_INTR_CLAIMED);
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: time:%d %s: called: intsrc:0x%b rx_active_head: %d",
	    dp->name, ddi_get_lbolt(), __func__,
	    intsrc, INT_BITS, dp->rx_active_head));

	if (intsrc & INT_TXHALT) {
		DPRINTF(0, (CE_NOTE, "!%s: %s: tx halt", dp->name, __func__));
	}

	if (intsrc & INT_RXHALT) {
		DPRINTF(0, (CE_NOTE, "!%s: %s: rx halt", dp->name, __func__));
	}

#ifdef GEM3
	if (intsrc & (INT_TXDONE | INT_TXIDLE)) {
		/* need to relaim tx buffers */
		if (gem_tx_done(dp)) {
			DPRINTF(1, (CE_NOTE, "!%s: resched tx", dp->name));
			flags |= INTR_RESTART_TX;
		}
#ifdef OPT_TxPOLL
		if (intsrc & INT_TXIDLE) {
			ASSERT(lp->tx_running);
			mutex_enter(&dp->xmitlock);
			if (dp->tx_desc_head != dp->tx_desc_tail) {
				OUTL(dp, TxCTL, lp->tx_control | TxCTL_POLL);
			} else {
				lp->tx_running = B_FALSE;
			}
			mutex_exit(&dp->xmitlock);
		}
#endif
	}
#endif
	if (intsrc & (INT_RXDONE | INT_RXIDLE)) {
		(void) gem_receive(dp);
		if (intsrc & INT_RXIDLE) {
			/*
			 * we schedule to re-enable rx state machine
			 * when we add rx buffer next time.
			 */
			OUTL(dp, RxCTL, lp->rx_control | RxCTL_POLL);

			dp->stats.errrcv++;
			dp->stats.norcvbuf++;
		}
	}
#ifndef GEM3
	if (intsrc & (INT_TXDONE | INT_TXIDLE)) {
		/* need to relaim tx buffers */
		if (gem_tx_done(dp)) {
			DPRINTF(1, (CE_NOTE, "!%s: resched tx", dp->name));
			flags |= INTR_RESTART_TX;
		}
#ifdef OPT_TxPOLL
		if (intsrc & INT_TXIDLE) {
			ASSERT(lp->tx_running);
			mutex_enter(&dp->xmitlock);
			if (dp->tx_desc_head != dp->tx_desc_tail) {
				OUTL(dp, TxCTL, lp->tx_control | TxCTL_POLL);
			} else {
				lp->tx_running = B_FALSE;
			}
			mutex_exit(&dp->xmitlock);
		}
#endif
	}
#endif
	if (intsrc & INT_LINK) {
		DPRINTF(2, (CE_NOTE, "!%s: %s: link changed",
		    dp->name, __func__));
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			flags |= INTR_RESTART_TX;
		}
#endif
	}

	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flags |= INTR_RESTART_TX;
	}

	DPRINTF(5, (CE_CONT, "!%s: %s: return: isr: %b",
	    dp->name, __func__, intsrc, INT_BITS));

	return (DDI_INTR_CLAIMED | flags);
}

/* ======================================================== */
/*
 * HW depend MII routine
 */
/* ======================================================== */
static void
sige_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
sige_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint32_t	ret;

	OUTL(dp, GMIICTL,
	    (dp->mii_phy_addr << GMI_PHY_SHIFT) |
	    (reg << GMI_REG_SHIFT) |
	    GMI_OP_RD | GMI_REQ);

	drv_usecwait(10);

	for (i = 0; (ret = INL(dp, GMIICTL)) & GMI_REQ; i++) {
		if (i > 1000) {
			/* timeout error happened */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(10);
	}

	return ((ret & GMI_DATA) >> GMI_DATA_SHIFT);
}

static void
sige_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;

	/* workaround for starting auto-negotiation */
	if ((dp->mii_status & MII_STATUS_XSTATUS) &&
	    reg == MII_CONTROL && (val & MII_CONTROL_RSAN)) {
		val |= MII_CONTROL_RESET;
	}

	OUTL(dp, GMIICTL,
	    (((uint32_t)val) << GMI_DATA_SHIFT) |
	    (((uint32_t)dp->mii_phy_addr) << GMI_PHY_SHIFT) |
	    (reg << GMI_REG_SHIFT) |
	    GMI_OP_WR | GMI_REQ);

	drv_usecwait(10);

	for (i = 0; INL(dp, GMIICTL) & GMI_REQ; i++) {
		if (i > 1000) {
			/* timeout error happened */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}
}

static int
sige_mii_init(struct gem_dev *dp)
{
	struct sige_dev	*lp = dp->private;

	switch (dp->mii_phy_id) {
	case PHY_ID_MARVELL88E1111:
		if (lp->phy_rgmii) {
			sige_mii_write(dp, 0x1b, 0x808b);
			drv_usecwait(200);
			sige_mii_write(dp, 0x14, 0x0ce1);
			drv_usecwait(200);
		} else {
			sige_mii_write(dp, 0x1b, 0x808f);
			drv_usecwait(200);
			sige_mii_write(dp, 0x14, 0x0c60);
			drv_usecwait(200);
		}
		break;

	case PHY_ID_BCM5461_1:
		/* for Broadcom BCM5461 PHY, set Tx Delay in RGMII mode */
		if (lp->phy_rgmii) {
			sige_mii_write(dp, 0x18, 0xf1c7);
			drv_usecwait(200);
			sige_mii_write(dp, 0x1c, 0x8c00);
			drv_usecwait(200);
		}
		break;
	}

	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
sige_attach_chip(struct gem_dev *dp)
{
	int	ret;
	struct sige_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (lp->mac_in_apc) {
		ret = sige_get_mac_addr_apc(dp);
	} else {
		ret = sige_get_mac_addr_eeprom(dp);
	}

	if (ret != GEM_SUCCESS) {
		cmn_err(CE_WARN, "%s: %s: cannot read mac address from %s",
		    dp->name, __func__,
		    lp->mac_in_apc ? "apc registger" : "eeprom");
		if (!gem_get_mac_addr_conf(dp)) {
			gem_generate_macaddr(dp,
			    dp->dev_addr.ether_addr_octet);
		}
	}

	/* fix rx buffer length, it must be aligned by 8 */
	dp->rx_buf_len = ROUNDUP(dp->rx_buf_len, 8);
#ifdef CONFIG_VLAN_HW
	dp->misc_flag |= GEM_VLAN_HARD;
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
#ifdef CONFIG_CKSUM_OFFLOAD
#ifdef CONFIG_LSO
	dp->misc_flag |= GEM_LSO;
#endif
	dp->misc_flag |= GEM_CKSUM_NOFULL_IPv4;
#endif

	/* 1G half mode isn't supported */
	dp->anadv_1000hdx = B_FALSE;

	return (GEM_SUCCESS);
}

static int
sigeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			unit;
	const char		*drv_name;
	int			i;
	ddi_acc_handle_t	conf_handle;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			rev;
	uint8_t			cfg73;
#ifdef DEBUG_LEVEL
	uint32_t		iline;
	uint8_t			latim;
#endif
	struct chip_info	*p;
	struct gem_dev		*dp;
	struct sige_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called (%s)",
	    drv_name, unit, __func__, ident));

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
	rev = pci_config_get16(conf_handle, PCI_CONF_REVID);
#ifdef DEBUG_LEVEL
	iline = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	latim = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
#endif
	for (i = 0, p = sige_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_NOTE,
	    "!%s%d: %s: unknown PCI venid/devid (0x%x, 0x%x)",
	    drv_name, unit, __func__, vid, did);
	/* fall through */

chip_found:
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	cfg73 = pci_config_get8(conf_handle, 0x73);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:

		DPRINTF(0, (CE_CONT,
		    "!%s%d: ilr 0x%08x, latency_timer:0x%02x",
		    drv_name, unit, iline, latim));
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &sige_dev_attr, (void *)&base, &regs_ha) !=
		    DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *)kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = 4;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = sige_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint64_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = sige_rx_copy_thresh;
#ifdef CONFIG_RX_PAD
		gcp->gc_rx_header_len = RM_PAD_LEN;
#else
		gcp->gc_rx_header_len = 0;
#endif
		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = sige_dev_attr;
		gcp->gc_buf_attr = sige_buf_attr;
		gcp->gc_desc_attr = sige_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = sige_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = sige_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = sige_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* setting for general PHY */
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;

		/*
		 * XXX - we cannot enable hw_link_detection because a
		 * phy didn't generate interrupts on link down after
		 * once it became link up.
		 */
		gcp->gc_mii_hw_link_detection = B_FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &sige_attach_chip;
		gcp->gc_reset_chip = &sige_reset_chip;
		gcp->gc_init_chip = &sige_init_chip;
		gcp->gc_start_chip = &sige_start_chip;
		gcp->gc_stop_chip = &sige_stop_chip;
		gcp->gc_multicast_hash = &sige_mcast_hash;
		gcp->gc_set_rx_filter = &sige_set_rx_filter;
		gcp->gc_set_media = &sige_set_media;
		gcp->gc_get_stats = &sige_get_stats;
		gcp->gc_interrupt = &sige_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &sige_tx_desc_write;
		gcp->gc_rx_desc_write = &sige_rx_desc_write;
		gcp->gc_tx_start = &sige_tx_start;
		gcp->gc_rx_start = NULL;

		gcp->gc_tx_desc_stat = &sige_tx_desc_stat;
		gcp->gc_rx_desc_stat = &sige_rx_desc_stat;
		gcp->gc_tx_desc_init = &sige_tx_desc_init;
		gcp->gc_rx_desc_init = &sige_rx_desc_init;
		gcp->gc_tx_desc_clean = &sige_tx_desc_init;
		gcp->gc_rx_desc_clean = &sige_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = &sige_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &sige_mii_sync;
		gcp->gc_mii_read = &sige_mii_read;
		gcp->gc_mii_write = &sige_mii_write;
		gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;

		lp = kmem_zalloc(sizeof (struct sige_dev), KM_SLEEP);
		lp->chip = p;
		lp->mac_in_apc = (cfg73 & 0x01) != 0;

		DPRINTF(0, (CE_CONT,
		    "!%s%d: chip:%s rev:0x%02x, cfg73:0x%02x",
		    drv_name, unit, p->chip_name, rev, cfg73));

		/* offload and jumbo frame */
		gcp->gc_max_lso = 16 * 1024 - 1;
		gcp->gc_max_mtu = ETHERMTU;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip, 0, gcp, base, &regs_ha,
		    lp, sizeof (*lp));
		kmem_free(gcp, sizeof (*gcp));

		if (dp == NULL) {
			goto err_freelp;
		}

		return (DDI_SUCCESS);

err_freelp:
		kmem_free(lp, sizeof (struct sige_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
sigedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(sige_ops, sigeattach, sigedetach);
#else
static	struct module_info sigeminfo = {
	0,			/* mi_idnum */
	"sige",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit sigerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&sigeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit sigewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&sigeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	sige_info = {
	&sigerinit,	/* st_rdinit */
	&sigewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_sige_ops = {
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
	&sige_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops sige_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	sigeattach,	/* devo_attach */
	sigedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_sige_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
};
#endif

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&sige_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/* ======================================================== */
/*
 * Loadable module support
 */
/* ======================================================== */
int
_init(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!sige: _init: called"));

	gem_mod_init(&sige_ops, "sige");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&sige_ops);
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

	DPRINTF(2, (CE_CONT, "!sige: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&sige_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
