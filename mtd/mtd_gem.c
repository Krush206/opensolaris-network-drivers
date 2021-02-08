/*
 *  mtd_gem.c : Myson mtd80x fast ethernet MAC driver for Solaris
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
#pragma	ident	"@(#)mtd_gem.c 1.9     11/08/17"

/*
 * CHANGE LOG:
 *  03/23/2006  dditypes.h added, ddi_impledefs.h removed
 *  02/21/2009  nicdrv passed
 *
 */

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
#include "mtd800reg.h"

char	ident[] = "Myson mtd80x driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int mtd_debug = DEBUG_LEVEL;
#if DEBUG_LEVEL > 4
#define	CONS	"^"
#else
#define	CONS	"!"
#endif
#define	DPRINTF(n, args)	if (mtd_debug > (n)) cmn_err args
#else
#define	CONS	"!"
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC			(drv_usectohz(1*1000000))
#define	ROUNDUP2(x, a)		(((x) + (a) - 1) & ~((a) - 1))

#ifdef MAP_MEM
#define	FLSHB(dp, reg)	(void) INB(dp, reg)
#define	FLSHW(dp, reg)	(void) INW(dp, reg)
#define	FLSHL(dp, reg)	(void) INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif

#define	READ_RXTX_CFG(rc, tc) { \
	uint32_t	val;	\
	val = INL(dp, RCR);	\
	(rc) = (uint16_t)val;	\
	(tc) = (uint16_t)(val >> 16);	\
}

#define	WRITE_RXTX_CFG(rc, tc)	OUTL(dp, RCR, ((tc) << 16) | (rc))

#define	TXDESC(p)	((struct tx_desc *)(void *)(p))
#define	RXDESC(p)	((struct rx_desc *)(void *)(p))

/*
 * Our configuration
 */
#ifndef MAXTXFLAGS
#define	MAXTXFLAGS	min(GEM_MAXTXFRAGS, 8)
#endif
#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef	TX_RING_SIZE
#define	TX_RING_SIZE	(TX_BUF_SIZE * (MAXTXFLAGS >= 4 ? 4 : 1))
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif
#ifndef	RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	MAXRXFRAGS	1

#if defined(WA_TXINTR)
/*
 * sometimes INT_TI isn't raised. Use INT_TBU, instead of.
 */
#define	OUR_INTR_BITS	\
	(INT_LSC | INT_RI | INT_RxErr | INT_RBU | INT_TBU | \
	INT_CNTOVF | INT_ROVF | INT_TUNF | INT_FBE | INT_ANC)
#else
#define	OUR_INTR_BITS	\
	(INT_LSC | INT_RI | INT_RxErr | INT_RBU | INT_TI | \
	INT_CNTOVF | INT_ROVF | INT_TUNF | INT_FBE | INT_ANC)
#endif

static int	mtd_tx_copy_thresh = 256;
static int	mtd_rx_copy_thresh = 256;

/*
 * Chip dependant MAC state
 */
struct mtd_dev {
	/* register shadows */
	uint32_t	our_intr_bits;
	uint16_t	rx_config;
	uint16_t	tx_config;
	uint8_t		mac_addr[ETHERADDRL];
	uint_t		cls;
};

/* ======================================================== */

/* mii operations */
static void  mtd_mii_sync_internal(struct gem_dev *);
static uint16_t  mtd_mii_read_internal(struct gem_dev *, uint_t);
static void mtd_mii_write_internal(struct gem_dev *, uint_t, uint16_t);
/* nic operations */
static int mtd_reset_chip(struct gem_dev *);
static int mtd_init_chip(struct gem_dev *);
static int mtd_start_chip(struct gem_dev *);
static int mtd_stop_chip(struct gem_dev *);
static int mtd_set_media(struct gem_dev *);
static int mtd_set_rx_filter(struct gem_dev *);
static int mtd_get_stats(struct gem_dev *);
static int mtd_attach_chip(struct gem_dev *);

/* descriptor operations */
static int mtd_tx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void mtd_tx_start(struct gem_dev *dp, int slot, int frags);
static void mtd_rx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags);
static void mtd_rx_start(struct gem_dev *dp, int slot, int frags);
static uint_t mtd_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t mtd_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void mtd_tx_desc_init(struct gem_dev *dp, int slot);
static void mtd_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t mtd_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr mtd_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr mtd_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t mtd_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x000007ffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000007ffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t mtd_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xffffffffull,		/* dma_attr_burstsizes */
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
#define	MTD_EEPROM_DELAY(dp)	{ \
	INL(dp, SROM_CR); INL(dp, SROM_CR); \
	INL(dp, SROM_CR); INL(dp, SROM_CR); \
}
#define	EE_CMD_READ	6
#define	EE_CMD_SHIFT	6

static uint16_t
mtd_eeprom_read(struct gem_dev *dp, uint_t offset)
{
	uint32_t	eedi;
	uint16_t	ret;
	int		i;
	uint_t		cs = SROM_DPM | SROM_ECS;

	/* ensure de-assert chip select */
	OUTL(dp, SROM_CR, cs);
	MTD_EEPROM_DELAY(dp);

	/* assert chip select */
	offset |= EE_CMD_READ << EE_CMD_SHIFT;

	for (i = 10; i >= 0; i--) {
		/* send one bit */
		eedi = ((offset >> i) & 1) << SROM_EDI_SHIFT;

		/* send 1 bit */
		OUTL(dp, SROM_CR, cs | eedi);
		MTD_EEPROM_DELAY(dp);
		OUTL(dp, SROM_CR, cs | eedi | SROM_ECK);
		MTD_EEPROM_DELAY(dp);
	}

	OUTL(dp, SROM_CR, cs);

	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTL(dp, SROM_CR, cs);
		MTD_EEPROM_DELAY(dp);
		OUTL(dp, SROM_CR, cs | SROM_ECK);
		MTD_EEPROM_DELAY(dp);
		ret = (ret << 1) | ((INL(dp, SROM_CR) >> SROM_EDO_SHIFT) & 1);
	}

	OUTL(dp, SROM_CR, cs);
	OUTL(dp, SROM_CR, 0);
	MTD_EEPROM_DELAY(dp);

	return (ret);
}

static void
mtd_eeprom_dump(struct gem_dev *dp, int size)
{
	int		i;

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < size; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, mtd_eeprom_read(dp, i), mtd_eeprom_read(dp, i + 1),
		    mtd_eeprom_read(dp, i + 2), mtd_eeprom_read(dp, i + 3));
	}
}

static int
mtd_freeze_tx(struct gem_dev *dp)
{
	int		i;
	uint16_t	rcr;
	uint16_t	tcr;
	int		ret = GEM_SUCCESS;
	struct mtd_dev	*lp = dp->private;

	lp->tx_config &= ~TCR_TE;
	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);

	i = 0;
	while (1) {
		READ_RXTX_CFG(rcr, tcr);
		if ((tcr & TCR_TE) == 0) {
			break;
		}
		if (i++ > 200) {
			cmn_err(CE_WARN, "!%s: %s: failed to stop tx",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(10);
	}

	return (ret);
}

#ifdef CONFIG_EEPROM_WRITE
#include "mtd_eeprom_wr.c"
#endif

static int
mtd_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	rcr;
	uint16_t	tcr;
	int		ret = GEM_SUCCESS;
	struct mtd_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s called", dp->name, __func__));

	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);

	lp->rx_config &= ~RCR_RE;
	lp->tx_config &= ~TCR_TE;
	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);

	for (i = 0; B_TRUE; i++) {
		READ_RXTX_CFG(rcr, tcr);
		if ((rcr & RCR_RE) == 0 && (tcr & TCR_TE) == 0) {
			break;
		}
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: failed to stop rx",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}

	/* reset the nic */
	OUTL(dp, BCR, BCR_SWR);
	FLSHL(dp, BCR);
	for (i = 0; INL(dp, BCR) & BCR_SWR; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(10);
	}

	/* clear the register shadow for mac_addr */
	bzero(lp->mac_addr, sizeof (lp->mac_addr));

	return (ret);
}

static int
mtd_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	int		maxdma;
	struct mtd_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s called", dp->name, __func__));

	OUTL(dp, TxLBA, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, RxLBA, (uint32_t)dp->rx_ring_dma);

	/*
	 * accept all multicast packets because we cannot change
	 * multicast hash filter after once tx is enabled.
	 */
	OUTL(dp, MAR + 0, 0xffffffffU);
	OUTL(dp, MAR + 4, 0xffffffffU);

	/* configure the PCI bus bursts and FIFO thresholds */
	maxdma = dp->txmaxdma;
	if (maxdma <= 4) {
		val = BCR_PBL_1;
	} else if (maxdma <= 16) {
		val = BCR_PBL_4;
	} else if (maxdma <= 32) {
		val = BCR_PBL_8;
	} else if (maxdma <= 64) {
		val = BCR_PBL_16;
	} else if (maxdma <= 128) {
		val = BCR_PBL_32;
	} else if (maxdma <= 256) {
		val = BCR_PBL_64;
	} else if (maxdma <= 512) {
		val = BCR_PBL_128;
	} else {
		val = BCR_PBL_512;
	}
	/* XXX - RLE not worked, RME made performance degraded */
	if (lp->cls != 0) {
		val |= BCR_RLE | BCR_RME;
	}
	OUTL(dp, BCR, val);

	maxdma = dp->rxmaxdma;
	if (maxdma <= 4) {
		val = RCR_RPB_1;
	} else if (maxdma <= 16) {
		val = RCR_RPB_4;
	} else if (maxdma <= 32) {
		val = RCR_RPB_8;
	} else if (maxdma <= 64) {
		val = RCR_RPB_16;
	} else if (maxdma <= 128) {
		val = RCR_RPB_32;
	} else if (maxdma <= 256) {
		val = RCR_RPB_64;
	} else if (maxdma <= 512) {
		val = RCR_RPB_128;
	} else {
		val = RCR_RPB_512;
	}
	lp->rx_config |= RCR_RBLEN | val;

	if (dp->txthr <= 32) {
		val = TCR_TFT_32;
	} else if (dp->txthr <= 64) {
		val = TCR_TFT_64;
	} else if (dp->txthr <= 12) {
		val = TCR_TFT_128;
	} else if (dp->txthr <= 256) {
		val = TCR_TFT_256;
	} else if (dp->txthr <= 512) {
		val = TCR_TFT_512;
	} else if (dp->txthr <= 768) {
		val = TCR_TFT_768;
	} else if (dp->txthr <= 1024) {
		val = TCR_TFT_1024;
	} else {
		val = TCR_TFT_SF;
	}
	lp->tx_config = val | TCR_FBACK;

	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);

	DPRINTF(10, (CE_CONT, "!%s:%s: flow control addr :%x %x",
	    dp->name, __func__, INL(dp, FAR), INL(dp, FAR+4)));

	return (GEM_SUCCESS);
}

static int
mtd_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint16_t	mode;
	uint64_t	hash_tbl;
	uint8_t		*m;
	uint16_t	rcr, tcr;
	struct mtd_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Set Receive filter control register */
	mode = RCR_AB | RCR_AM | RCR_ARP | RCR_ALP | RCR_SEP;

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* accept all physcal address and multicast packets */
		mode |= RCR_PROM;
	}

	/* Load station address if it has been changed */
	m = dp->cur_addr.ether_addr_octet;
	if (bcmp(m, lp->mac_addr, ETHERADDRL) != 0) {

		/* update my station address */
		bcopy(m, lp->mac_addr, ETHERADDRL);
		OUTL(dp, PAR + 0,
		    m[3] << 24 | m[2] << 16 | m[1] << 8 | m[0]);
		OUTL(dp, PAR + 4, m[5] << 8 | m[4]);
		DPRINTF(1, (CE_CONT, "!%s: updating mac_addr", dp->name));
	}

	/* Load the new rx filter mode */
	lp->rx_config = (lp->rx_config & ~RxFilterMode) | mode;
	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);

	DPRINTF(1, (CE_CONT, "!%s: updating rcr:%b",
	    dp->name, lp->rx_config, RCR_BITS));

	return (GEM_SUCCESS);
}

static int
mtd_start_chip(struct gem_dev *dp)
{
	struct mtd_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* enable interrupt */
	lp->our_intr_bits = OUR_INTR_BITS;

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IMR, lp->our_intr_bits);
	}

	/* enable rx and tx */
	lp->rx_config |= RCR_RE;
	lp->tx_config |= TCR_TE;
	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);

	return (GEM_SUCCESS);
}

static int
mtd_stop_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	rcr;
	uint16_t	tcr;
	int		ret = GEM_SUCCESS;
	struct mtd_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable interrupts */
	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);

	/* stop rx and tx state machines */
	lp->rx_config &= ~RCR_RE;
	lp->tx_config &= ~TCR_TE;
	WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);
	FLSHW(dp, RCR);

	for (i = 0; B_TRUE; i++) {
		READ_RXTX_CFG(rcr, tcr);
		if ((rcr & RCR_RE) == 0 && (tcr & TCR_TE) == 0) {
			break;
		}
		if (i > 200) {
			cmn_err(CE_WARN, "!%s: %s: failed to stop rx",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(10);
	}

	return (ret);
}

static int
mtd_set_media(struct gem_dev *dp)
{
	uint16_t	new_tx;
	uint16_t	new_rx;
	struct mtd_dev	*lp = dp->private;
	extern int	gem_speed_value[];

	DPRINTF(0, (CE_CONT, "!%s: %s: %s duplex, %d Mbps",
	    dp->name, __func__,
	    dp->full_duplex ? "full" : "half",
	    gem_speed_value[dp->speed]));

	READ_RXTX_CFG(new_rx, new_tx);
	new_rx &= ~(RCR_RFCEN);
	new_tx &= ~(TCR_FD | TCR_PS | TCR_TFCEN);

	if (dp->full_duplex) {
		new_tx |= TCR_FD;
	}

	if (dp->speed == GEM_SPD_10) {
		new_tx |= TCR_PS;
	}

	/* flow control */
	switch (dp->flow_control) {
	case FLOW_CONTROL_SYMMETRIC:
		new_tx |= TCR_TFCEN;
		new_rx |= RCR_RFCEN;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		new_tx |= TCR_TFCEN;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		new_rx |= RCR_RFCEN;
		break;
	}

	if (lp->tx_config != new_tx || lp->rx_config != new_rx) {
		lp->rx_config = new_rx;
		lp->tx_config = new_tx;
		WRITE_RXTX_CFG(lp->rx_config, lp->tx_config);
	}

	return (GEM_SUCCESS);
}

static int
mtd_get_stats(struct gem_dev *dp)
{
	volatile uint32_t	x;

	dp->stats.crc += INW(dp, TC_CRC);
	dp->stats.missed += INW(dp, TC_MPA);

	x = INL(dp, TC_TSR);

	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
mtd_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int		i;
	uint_t		len;
	uint_t		total;
	uint32_t	mark;
	uint32_t	addr;
	uint32_t	own;
	struct tx_desc	*tdp;
	struct mtd_dev	*lp = dp->private;
	int		additional = 0;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 3
	cmn_err(CE_CONT,
	    "!%s: time:%d %s seqnum: %d, slot %d, frags: %d flags: %lld",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_desc_tail, slot, frags, flags);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%llx",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif

	/*
	 * write tx descriptor in reversed order.
	 */
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
	total = 0;
	own = LE_32(TDES0_OWN);
	mark = TDES1_LD | TDES1_CRC | TDES1_PAD;
	if (flags & GEM_TXFLAG_INTR) {
		mark |= TDES1_IC;
	}

#if defined(CONFIG_PCI_MRL) && defined(WA_PCI_MRL_HANG)
	/*
	 * XXX - a work around to avoid that received packets will be lost
	 * when we use pci read line or read multiple. Otherwise, we won't
	 * be able to get the response packets for ping -s HOSTNAME 220.
	 */
	i = frags - 1;
	addr = dmacookie[i].dmac_address + dmacookie[i].dmac_size;
	if (lp->cls != 0 && (dmacookie[i].dmac_size & (lp->cls - 1)) == 6) {
		len = 1;
		dmacookie[i].dmac_size -= len;
		mark |= len;
		addr -= len;
		tdp = &TXDESC(dp->tx_ring)[SLOT(slot + i + 1, tx_ring_size)];
		tdp->tdes2 = LE_32(addr);
		tdp->tdes1 = LE_32(mark);
		tdp->tdes0 = own;

		mark &= ~(TDES1_LD | TDES1_TBS);
		total += len;
		additional = 1;
	}
#endif

#if MAXTXFLAGS != 1
	for (i = frags - 1; i > 0; i--) {
		len = dmacookie[i].dmac_size;
		mark |= len;
		addr = dmacookie[i].dmac_address;
		tdp = &TXDESC(dp->tx_ring)[SLOT(slot + i, tx_ring_size)];
		tdp->tdes2 = LE_32(addr);
		tdp->tdes1 = LE_32(mark);
		tdp->tdes0 = own;

		mark &= ~(TDES1_LD | TDES1_TBS);
		total += len;
	}
#endif
	if (flags & GEM_TXFLAG_HEAD) {
		own = 0;
	}

	len = dmacookie[0].dmac_size;
	total += len;

	addr = dmacookie[0].dmac_address;
	mark |= TDES1_FD | total << TDES1_PKTS_SHIFT | len;
	tdp = &TXDESC(dp->tx_ring)[slot];
	tdp->tdes2 = LE_32(addr);
	tdp->tdes1 = LE_32(mark);
	tdp->tdes0 = own;

	return (frags + additional);
}

static void
mtd_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct mtd_dev	*lp = dp->private;

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	TXDESC(dp->tx_ring)[start_slot].tdes0 = LE_32(TDES0_OWN);
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/*
	 * Let the Transmit Buffer Manager Fill state machine active.
	 */
	OUTL(dp, TxPDR, 0);
}

static void
mtd_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t	addr;
	uint32_t	size;
	struct rx_desc	*rdp;
#if DEBUG_LEVEL > 3
	int		i;

	ASSERT(frags == 1);

	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	rdp = &RXDESC(dp->rx_ring)[slot];

	addr = dmacookie->dmac_address;
	size = dmacookie->dmac_size;
	rdp->rdes2 = LE_32(addr);
	rdp->rdes1 = LE_32(size);
	rdp->rdes0 = LE_32(RDES0_OWN);
}

static uint_t
mtd_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_desc	*tdp;
	uint32_t	status;
	int		i;
	int		cols;
	struct mtd_dev	*lp = dp->private;

	/* check status of the last descriptor */
#if MAXTXFLAGS == 1
	tdp = &TXDESC(dp->tx_ring)[SLOT(slot, dp->gc.gc_tx_ring_size)];
#else
	tdp = &TXDESC(dp->tx_ring)[SLOT(slot + ndesc - 1,
	    dp->gc.gc_tx_ring_size)];
#endif

	status = tdp->tdes0;
	status = LE_32(status);

#ifdef NEVER
	if (ndesc > 1) {
		for (i = 0; i < ndesc; i++) {
			tdp = &TXDESC(dp->tx_ring)[
			    SLOT(slot + i, dp->gc.gc_tx_ring_size)];
			DPRINTF(-1, (CE_CONT,
			    "!%s: time:%d %s:"
			    " slot:%d+%d, status:0x%b, ctrl:0x%b",
			    dp->name, ddi_get_lbolt(), __func__,
			    slot, i,
			    LE_32(tdp->tdes0), TDS_BITS,
			    LE_32(tdp->tdes1), TDES1_BITS));
		}
	}
#endif
	if (status & TDES0_OWN) {
		/*
		 * not yet transmitted
		 */
		return (0);
	}

	/*
	 *  collect statictics
	 */
	if (status &
	    (TDES0_ABORT | TDES0_CSL | TDES0_LC | TDES0_EC | TDES0_HF)) {

		/* failed to transmit the packet */

		DPRINTF(2, (CE_CONT, "!%s: Transmit error, Tx status %b",
		    dp->name, status, TDS_BITS));

		dp->stats.errxmt++;

		if (status & TDES0_CSL) {
			dp->stats.nocarrier++;
		} else if (status & TDES0_LC) {
			dp->stats.xmtlatecoll++;
		} else if ((!dp->full_duplex) && (status & TDES0_NCR) == 16) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		}
	} else if (!dp->full_duplex) {
		/* update collision counters in half duplex mode */
		cols = status & TDES0_NCR;

		if (cols > 0) {
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* (cols > 1) */ {
				dp->stats.multi_coll++;
			}
			dp->stats.collisions += cols;
		} else if (status & TDES0_DFR) {
			dp->stats.defer++;
		}
	}

	return (GEM_TX_DONE);
}

static uint64_t
mtd_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_desc		*rdp;
	uint_t			len;
	uint_t			flag;
	uint32_t		status;

	rdp = &RXDESC(dp->rx_ring)[slot];

	status = rdp->rdes0;
	status = LE_32(status);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: slot:%d, status:0x%b",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, status, RDS_BITS));

	if (status & RDES0_OWN) {
		/*
		 * No more received packets because
		 * this buffer is owned by NIC.
		 */
		return (0);
	}

#define	RX_ERR_BITS \
	(RDS_COLON | RDS_NIBON | RDS_OVRUN | RDS_MIIER | \
	RDS_LIMIT | RDS_SHORT | RDS_ABORT | RDS_CRCOK | RDS_DESCS)

	if ((status & (RDES0_FSD | RDES0_LSD | RDES0_ES))
	    != (RDES0_FSD | RDES0_LSD)) {
		/*
		 * we received a packet with error.
		 */
		DPRINTF(0, (CE_CONT, "!%s: Corrupted packet "
		    "received, buffer status: %b",
		    dp->name, status, RDS_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;

		if ((status & RDES0_LONG) ||
		    (status & (RDES0_FSD | RDES0_LSD)) == RDES0_FSD) {
			dp->stats.frame_too_long++;
		} else if (status & RDES0_RUNT) {
			dp->stats.runt++;
		} else if (status & (RDES0_FAE | RDES0_RXER)) {
			dp->stats.frame++;
		} else if (status & RDES0_CRC) {
#ifdef notdef
			dp->stats.crc++;
#else
			/* EMPTY */
#endif
		}

		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/*
	 * this packet was received without errors
	 */
	len = (status & RDES0_FLNG) >> RDES0_FLNG_SHIFT;
	if (len - ETHERFCSL > 0) {
		len -= ETHERFCSL;
	}

#ifdef notdef
{
	int	i;
	uint8_t	*bp = dp->rx_buf_head->rxb_buf;

	cmn_err(CE_CONT, "!%s: len:%d", dp->name, len);

	for (i = 0; i < 60; i += 10) {
		cmn_err(CE_CONT,
		    "!%02x %02x %02x %02x %02x"
		    " %02x %02x %02x %02x %02x",
		    bp[0], bp[1], bp[2], bp[3], bp[4],
		    bp[5], bp[6], bp[7], bp[8], bp[9]);
	}
	bp += 10;
}
#endif
	return (GEM_RX_DONE | (len & GEM_RX_LEN));
}

static void
mtd_tx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	here;

	TXDESC(dp->tx_ring)[slot].tdes0 = 0;

	/* make a link to this from the previous descriptor */
	here = ((uint32_t)dp->tx_ring_dma) + sizeof (struct tx_desc) * slot;

	TXDESC(dp->tx_ring)[SLOT(slot - 1, dp->gc.gc_tx_ring_size)].tdes3
	    = LE_32(here);
}

static void
mtd_rx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	here;

	RXDESC(dp->rx_ring)[slot].rdes0 = 0;

	/* make a link to this from the previous descriptor */
	here = ((uint32_t)dp->rx_ring_dma) + sizeof (struct rx_desc) * slot;

	RXDESC(dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)].rdes3 = LE_32(here);
}

static void
mtd_tx_desc_clean(struct gem_dev *dp, int slot)
{
	TXDESC(dp->tx_ring)[slot].tdes0 = 0;
}

static void
mtd_rx_desc_clean(struct gem_dev *dp, int slot)
{
	RXDESC(dp->rx_ring)[slot].rdes0 = 0;
}


/*
 * Device depend interrupt handler
 */
static uint_t
mtd_interrupt(struct gem_dev *dp)
{
	uint32_t	intsrc;
	uint_t		flags = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct mtd_dev	*lp = dp->private;

	/* read interrupt reason */
	intsrc = INL(dp, ISR);

	if ((intsrc & lp->our_intr_bits) == 0) {
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: called: intsrc:0x%b rx_active_head: %d",
	    dp->name, ddi_get_lbolt(), __func__,
	    intsrc, INT_BITS, dp->rx_active_head));

	/* clear the interrupt reasons */
	intsrc &= lp->our_intr_bits;
	OUTL(dp, ISR, intsrc);

	if (!dp->mac_active) {
		/* the device is going to stop */
		if (intsrc) {
			OUTL(dp, ISR, intsrc);
		}
		lp->our_intr_bits = 0;
		OUTL(dp, IMR, lp->our_intr_bits);
		FLSHL(dp, IMR);
		return (DDI_INTR_CLAIMED);
	}

	/* barrier for interrupt state */
	FLSHL(dp, ISR);

	if (intsrc & INT_CNTOVF) {
		mtd_get_stats(dp);
	}

	if (intsrc & (INT_RI | INT_RBU | INT_ROVF | INT_RxErr)) {
		(void) gem_receive(dp);

		if (intsrc & INT_ROVF) {
			dp->stats.overflow++;
			/* restart the nic */
#ifdef TEST_RESTART
			need_to_reset = B_TRUE;
			DPRINTF(0, (CE_NOTE,
			    "!%s: %s: rx overflow", dp->name, __func__));
#else
			OUTL(dp, RxPDR, 0);
#endif
		}

		if (intsrc & INT_RBU) {
			DPRINTF(0, (CE_NOTE,
			    "!%s: %s: rx empty", dp->name, __func__));
			dp->stats.norcvbuf++;
			/* restart the nic */
			OUTL(dp, RxPDR, 0);
		}
	}

	if (intsrc & (INT_TI | INT_TBU)) {
		/* need to relaim tx buffers */
		if (gem_tx_done(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}

	if (intsrc & INT_TUNF) {
		dp->stats.underflow++;
#ifdef TEST_RESTART
		need_to_reset = B_TRUE;
		DPRINTF(0, (CE_NOTE, "!%s: %s: time:%d tx underrun",
		    dp->name, __func__, ddi_get_lbolt()));
#else
		OUTL(dp, TxPDR, 0);
#endif
	}

	if (intsrc & (INT_LSC | INT_ANC)) {
		DPRINTF(2, (CE_NOTE, "!%s: %s: link status changed",
		    dp->name, __func__));
		if (gem_mii_link_check(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}

	if (intsrc & (INT_ERI | INT_ETI)) {
		cmn_err(CE_NOTE, "!%s: %s: unexpected interrupt: %b",
		    dp->name, __func__, intsrc, INT_BITS);
	}

	if (intsrc & INT_FBE) {
		cmn_err(CE_WARN, "!%s: %s: abnormal interrupt: %b",
		    dp->name, __func__, intsrc, INT_BITS);
		need_to_reset = B_TRUE;
	}

	if (need_to_reset) {
		gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
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
mtd_mii_sync_internal(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
mtd_mii_read_internal(struct gem_dev *dp, uint_t reg)
{
	uint16_t	val;

	switch (reg) {
	case MII_CONTROL:
	case MII_STATUS:
	case MII_PHYIDH:
	case MII_PHYIDL:
	case MII_AN_ADVERT:
	case MII_AN_LPABLE:
		val = INL(dp, PHYBA + reg * sizeof (uint16_t));
		break;

	default:
		val = 0;
		break;
	}

	DPRINTF(4, (CE_CONT, "!%s: %s: reg:%d val:0x%04x",
	    dp->name, __func__, reg, val));

	return (val);
}

static void
mtd_mii_write_internal(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	switch (reg) {
	case MII_CONTROL:
	case MII_STATUS:
	case MII_PHYIDH:
	case MII_PHYIDL:
	case MII_AN_ADVERT:
	case MII_AN_LPABLE:
		OUTL(dp, PHYBA + reg * sizeof (uint16_t), val);
		break;
	}
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
mtd_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	uint8_t		*m;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#if DEBUG_LEVEL > 4
	mtd_eeprom_dump(dp, 0x40);
#endif
#ifdef notdef
	/* fix eeprom contents */
	/* enable flow control */
	mtd_control_eeprom(dp, WEN);
	for (i = 0; i < 0x40; i++) {
		mtd_erase_eeprom(dp, i);
		delay(2);
	}
	mtd_control_eeprom(dp, WDS);
	mtd_eeprom_dump(dp, 0x40);
	mtd_control_eeprom(dp, WEN);
	for (i = 0; i < 0x40; i++) {
		mtd_write_eeprom(dp, i, mtd803_eeprom[i]);
		delay(2);
	}
	mtd_control_eeprom(dp, WDS);
	mtd_eeprom_dump(dp, 0x40);
#endif
	m = dp->dev_addr.ether_addr_octet;

	for (i = 0; i < ETHERADDRL; i += 2) {
		val = mtd_eeprom_read(dp, i/2 + 8);
		m[i + 0] = (uint8_t)val;
		m[i + 1] = (uint8_t)(val >> 8);
	}
#ifdef never
	/* mac address is incorrect */
	if (!gem_get_mac_addr_conf(dp)) {
		cmn_err(CE_WARN,
		    "!%s: %s: cannot read mac address from eeprom",
		    dp->name, __func__);
		gem_generate_macaddr(dp, dp->dev_addr.ether_addr_octet);
	}
#endif
	/* rx buffer length must be multiple of 4 */
	dp->rx_buf_len = ROUNDUP2(dp->rx_buf_len, 4);

	/* clear statistic counters */
	mtd_get_stats(dp);
	bzero(&dp->stats, sizeof (dp->stats));

#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	return (GEM_SUCCESS);
}

static int
mtdattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			unit;
	const char		*drv_name;
	int			i;
	ddi_acc_handle_t	conf_handle;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			rev;
#ifdef DEBUG_LEVEL
	uint32_t		iline;
	uint8_t			latim;
#endif
	uint8_t			cls;
	struct gem_dev		*dp;
	struct mtd_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: %s: called (%s)",
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
	iline =	pci_config_get32(conf_handle, PCI_CONF_ILINE);
	latim = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
#endif
#ifdef CONFIG_PCI_MRL
	if (pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ) == 0) {
		pci_config_put8(conf_handle, PCI_CONF_CACHE_LINESZ, 8);
	}
#else
	pci_config_put8(conf_handle, PCI_CONF_CACHE_LINESZ, 0);
#endif
	cls = pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ) * 4;
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));


	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:

		DPRINTF(0, (CE_CONT,
		    "!%s%d: ilr 0x%08x, latency_timer:0x%02x, "
		    "cache line size:%d double words",
		    drv_name, unit, iline, latim, cls/4));
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &mtd_dev_attr, (void *)&base, &regs_ha)
		    != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		lp = kmem_zalloc(sizeof (struct mtd_dev), KM_SLEEP);
		lp->cls = cls;

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_tx_max_frags = min(GEM_MAXTXFRAGS, MAXTXFLAGS);
#if defined(CONFIG_PCI_MRL) && defined(WA_PCI_MRL_HANG)
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
#else
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
#endif
		gcp->gc_tx_desc_unit_shift = 4;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = mtd_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = mtd_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = mtd_dev_attr;
		gcp->gc_buf_attr = mtd_buf_attr;
		gcp->gc_desc_attr = mtd_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = mtd_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = mtd_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = mtd_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

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
		gcp->gc_mii_hw_link_detection = B_TRUE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &mtd_attach_chip;
		gcp->gc_reset_chip = &mtd_reset_chip;
		gcp->gc_init_chip = &mtd_init_chip;
		gcp->gc_start_chip = &mtd_start_chip;
		gcp->gc_stop_chip = &mtd_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &mtd_set_rx_filter;
		gcp->gc_set_media = &mtd_set_media;
		gcp->gc_get_stats = &mtd_get_stats;
		gcp->gc_interrupt = &mtd_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &mtd_tx_desc_write;
		gcp->gc_rx_desc_write = &mtd_rx_desc_write;
		gcp->gc_tx_start = &mtd_tx_start;
		gcp->gc_rx_start = NULL;

		gcp->gc_tx_desc_stat = &mtd_tx_desc_stat;
		gcp->gc_rx_desc_stat = &mtd_rx_desc_stat;
		gcp->gc_tx_desc_init = &mtd_tx_desc_init;
		gcp->gc_rx_desc_init = &mtd_rx_desc_init;
		gcp->gc_tx_desc_clean = &mtd_tx_desc_clean;
		gcp->gc_rx_desc_clean = &mtd_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &mtd_mii_sync_internal;
		gcp->gc_mii_read = &mtd_mii_read_internal;
		gcp->gc_mii_write = &mtd_mii_write_internal;
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* offload and jumbo frame */
		gcp->gc_max_lso = 0;
		gcp->gc_max_mtu = 1920;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp == NULL) {
			goto err_freelp;
		}

		return (DDI_SUCCESS);

err_freelp:
		kmem_free(lp, sizeof (struct mtd_dev));
err:
		break;
	}
	return (DDI_FAILURE);
}

static int
mtddetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(mtd_ops, mtdattach, mtddetach);
#else
static	struct module_info mtdminfo = {
	0,			/* mi_idnum */
	"mtd",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit mtdrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&mtdminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit mtdwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&mtdminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	mtd_info = {
	&mtdrinit,	/* st_rdinit */
	&mtdwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_mtd_ops = {
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
	&mtd_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops mtd_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	mtdattach,	/* devo_attach */
	mtddetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_mtd_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&mtd_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!mtd: _init: called"));

	gem_mod_init(&mtd_ops, "mtd");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&mtd_ops);
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

	DPRINTF(2, (CE_CONT, "!mtd: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&mtd_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
