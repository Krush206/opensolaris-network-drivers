/*
 * epfe_gem.c : SMC EPIC II fast ethernet controler driver for Solaris.
 *
 * Copyright (c) 2002-2009 Masayuki Murayama.  All rights reserved.
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
#pragma ident	"@(#)epfe_gem.c	1.7 09/02/11"

/*
 * Change log
 *
 * 09/11/2003  0.0.0 starting coding.
 * 12/24/2006  2.4.0 multicast hash bit calculation fixed
 */

/*
 * TODO:
 *	tx underrun
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
#include "epicreg.h"

char	ident[] = "smsc epic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int epfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (epfe_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif

/*
 * EPIC Tx descriptor and Rx descriptor implementation
 */
#define	MAXTXFRAGS	5

struct epic_tfd {
	struct epic_desc	desc;		/* 16 byte */
#if MAXTXFRAGS > 1
	uint32_t		:32;		/* 4 byte */
	volatile uint32_t	nfrags;		/* 4 bytes */
	struct epic_frag	frag[MAXTXFRAGS];	/* 8 * MAXTXFRAGS */
#endif
};

#define	MAXRXFRAGS	1	/* 5 */

struct epic_rfd {
	struct epic_desc	desc;		/* 16 bytes */
#if MAXRXFRAGS > 1
	uint32_t		:32;		/* 4 byte */
	volatile uint32_t	nfrags;		/* 4 byte */
	struct epic_frag	frag[MAXRXFRAGS];
#endif
};

/*
 * Our configuration
 */
#if 1
#define	INTR_MASK_COMMON	\
	(INT_CNT | INT_HCC |	\
	INT_RXE | INT_OVW | INT_RQE | INT_RCC | \
	INT_TXU | INT_TXC)
#else
#define	INTR_MASK_COMMON	\
	(INT_CNT | INT_HCC |	\
	INT_RXE | INT_OVW | INT_RQE | INT_RCC | \
	INT_TXU | INT_TQE)
#endif
#define	INTR_BUSERR_170	\
	(INT_DPE_170 | INT_APE_170 | INT_PMA_170 | INT_PTA_170)
#define	INTR_BUSERR_171	INT_FATAL_INT

#ifdef TEST_RX_EMPTY
#define	RX_BUF_SIZE	4
#endif
#ifdef TEST_TXDESC_FULL
#define	TX_RING_SIZE	4
#define	TX_BUF_SIZE	64
#endif

#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	64
#endif
#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	TX_RING_SIZE
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#define	ONESEC			(drv_usectohz(1*1000000))
static int	epfe_tx_copy_thresh = 256;
static int	epfe_rx_copy_thresh = 256;

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	uint8_t		rev_min;
	uint8_t		rev_max;
	uint8_t		chip_type;
	char		*name;
};

#define	EPIC_83C170	0
#define	EPIC_83C171	1
#define	EPIC_83C172	2
#define	EPIC_83C175	3

static struct chip_info epfe_chiptbl[] = {
	{0x10b8, 0x0005, 0x00, 0x05, EPIC_83C170, "SMSC LAN83C170"},
	{0x10b8, 0x0005, 0x06, 0x07, EPIC_83C171, "SMSC LAN83C171"},
	{0x10b8, 0x0005, 0x08, 0xff, EPIC_83C172, "SMSC LAN83C172"},
	{0x10b8, 0x0006, 0x00, 0xff, EPIC_83C175, "SMSC LAN83C175"},
};

#define	CHIPTABLESIZE	(sizeof (epfe_chiptbl) / sizeof (struct chip_info))

struct epfe_dev {
	struct chip_info	*chip;

	/* copy of registers */
	uint16_t	genctl;
	uint16_t	rxcon;
	uint8_t		txcon;
	uint_t		need_to_reset;

	/* interrupt mask */
	uint32_t	our_intr_mask;
	uint32_t	intr_buserr;
	char		*int_bits;

#ifdef DEBUG_LEVEL
	seqnum_t	prev_rx_head;
	seqnum_t	prev_rx_tail;
#endif
};


/* ======================================================== */

/* mii operations */
static void  epfe_mii_sync(struct gem_dev *);
static uint16_t epfe_mii_read(struct gem_dev *, uint_t);
static void epfe_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int epfe_attach_chip(struct gem_dev *);
static int epfe_reset_chip(struct gem_dev *);
static int epfe_init_chip(struct gem_dev *);
static int epfe_start_chip(struct gem_dev *);
static int epfe_stop_chip(struct gem_dev *);
static int epfe_set_media(struct gem_dev *);
static int epfe_set_rx_filter(struct gem_dev *);
static int epfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int epfe_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void epfe_tx_start(struct gem_dev *dp, int slot, int frags);
static void epfe_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t epfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t epfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void epfe_tx_desc_init(struct gem_dev *dp, int slot);
static void epfe_rx_desc_init(struct gem_dev *dp, int slot);
static void epfe_tx_desc_clean(struct gem_dev *dp, int slot);
static void epfe_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t epfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr epfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr epfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t epfe_dma_attr_buf = {
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

static ddi_dma_attr_t epfe_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	4,			/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * misc runtime routines
 */
/* ======================================================== */
static int
epfe_log2(uint64_t val)
{
	int	i;

	if (val == 0) {
		return (0);
	}

	for (i = 0; i < sizeof (val)*8; i++) {
		if ((val & 1) != 0) {
			break;
		}
		val >>= 1;
	}
	return (i);
}

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */

static int
epfe_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));
	/*
	 * Reset the chip.
	 * XXX - don't reset PHY, otherwise it may not be detected.
	 */
	OUTL(dp, GENCTL, GC_SOFTRESET);

	/* it need 15 pci cycle */
	drv_usecwait(10);

	lp->genctl = 0;
	OUTL(dp, GENCTL, lp->genctl);

	for (i = 0; i < 16; i++) {
		OUTL(dp, TEST1, 0x0008);
	}

	/* enable phy */
	OUTL(dp, MIICFG, MIICFG_EN_SMI);

	/* exit power down mode */
	if (lp->chip->chip_type == EPIC_83C175) {
		val = INL(dp, NVCTL) &
		    ~(NVCTL_GPIO2| NVCTL_GPIO1| NVCTL_GPOE2 | NVCTL_GPOE1);
		OUTL(dp, NVCTL, val | NVCTL_FETPWRPHY | NVCTL_PHYPWRDOWN_N);

	}
	return (GEM_SUCCESS);
}

static int
epfe_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* 00 COMMAND: do nothing */

	/* 04 INTSTAT STATUS: do nothing */

	/* 08 INTMASK : set OUR_INTR_MASK later */
	OUTL(dp, INTMASK, lp->our_intr_mask);
	DPRINTF(2, (CE_CONT, "!%s: %s: intmask:%b",
	    dp->name, __func__, INL(dp, INTMASK), lp->int_bits));

	/* 0C GENERAL CONTROL */
	if (dp->rxthr <= 32) {
		val = GC_RXFIFO_32;
	} else if (dp->rxthr <= 64) {
		val = GC_RXFIFO_64;
	} else if (dp->rxthr <= 96) {
		val = GC_RXFIFO_96;
	} else {
		val = GC_RXFIFO_128;
	}
	/* XXX - tx underrun happens in case of GC_RXDMAPRI */
	lp->genctl = val | GC_ONECOPY | GC_MEMRD_LINE | GC_TXDMAPRI;
	OUTL(dp, GENCTL, lp->genctl);
	DPRINTF(0, (CE_CONT, "!%s: %s: genctl:%b",
	    dp->name, __func__, INL(dp, GENCTL), GENCTL_BITS));

	/* 10 NON-VOLATILE CONTROL : dont touch */

	/* 14 EEPROM CTL: release MA bus */
	OUTL(dp, EECTL, 0);
#if 0
	/* 18 PBLCNT(6bit width): shoud we limit max burst length = 128 byte */
	OUTL(dp, PBLCNT,
	    min(max(dp->txmaxdma, dp->rxmaxdma)/sizeof (uint32_t), 0x3f));
#endif
	/* 20 CRC ERROR COUNTER: clear later in get_stats */
	/* 24 FRAME ALIGNMENT ERROR COUNTER: clear later in get_stats */
	/* 28 MISSED PACKET COUNTER: clear later in get_stats */

	/* 30 MII control: do nothing */
	/* 34 MII data: do nothing */
	/* 38 MII configration: initialize in mii_init */

	/* 3C interpacket gap register : don't touch */
	DPRINTF(2, (CE_CONT, "!%s: %s: ipg:%x",
	    dp->name, __func__, INL(dp, IPG)));

	/* 40-4C LAN0-2: don't touch */
	/* 4C ID_CHK: don't touch */
	/* 50-5C MC0-3: don't touch */

	/* 60 RXCON: receive control register */
	lp->rxcon = RXCON_RUNT | RXCON_ERR;
	OUTL(dp, RXCON, lp->rxcon);
	DPRINTF(2, (CE_CONT, "!%s: %s: rxcon:%b",
	    dp->name, __func__, INL(dp, RXCON), RXCON_BITS));

	/* 70 TXCON: transmit control register, we don't use early_tx */
	lp->txcon = TXCON_SLOT(512) /* | TXCON_EARLY */;
	OUTL(dp, TXCON, lp->txcon);
	DPRINTF(0, (CE_CONT, "!%s: %s: txcon:%b",
	    dp->name, __func__, INL(dp, TXCON), TXCON_BITS));

	/* 74 TXTEST: clear tx test register */
	OUTL(dp, TXTEST, 0);

	/* 84 PRCDAR: PCI recieve current descriptor address */
	OUTL(dp, PRCDAR, dp->rx_ring_dma);

	/* A4 PRSTAT: clear PCI receive dma status */
	OUTL(dp, PRSTAT, 0);

	/* B0 PRCPTHR: receive copy threshold */
	OUTL(dp, PRCPTHR, 0);

	/* C4 PTCDAR: PCI transmit current descriptor address */
	OUTL(dp, PTCDAR, dp->tx_ring_dma);

	/* DC ETXTHR: tx threshold */
	OUTL(dp, ETXTHR, 0x100);

#ifdef DEBUG_LEVEL
	lp->prev_rx_head = -1;
	lp->prev_rx_tail = -1;
#endif
	return (GEM_SUCCESS);
}

static uint32_t
epfe_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is 6 bits of MSB in be-crc */
	return (gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6));
}

static int
epfe_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint8_t		mode;
	uint8_t		*mac;
	uint16_t	mhash[4];
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#if DEBUG_LEVEL > 2
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
	mode = RXCON_BR | RXCON_MULT;

	/* clear multicast hash table */
	bzero(mhash, sizeof (mhash));

	if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscous */
		mode |= RXCON_PROMISC;

		/* need to set all bit in the multcast hash table */
		for (i = 0; i < 4; i++) {
			mhash[i] = 0xffff;
		}
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 64/2) {
		for (i = 0; i < 4; i++) {
			mhash[i] = 0xffff;
		}
	} else if (dp->mc_count > 0) {
		/* Make multicast hash table */
		for (i = 0; i < dp->mc_count; i++) {
			uint_t	k;
			k = dp->mc_list[i].hash;
			mhash[k / 16] |= 1 << (k % 16);
		}
	}

	/* set rx mode */
	DPRINTF(3, (CE_CONT,
	    "!%s: %s(%x): "
	    "setting mode: %b mhash[3-0]: %04x %04x %04x %04x",
	    dp->name, __func__, dp->rxmode, mode | lp->rxcon, RXCON_BITS,
	    mhash[3], mhash[2], mhash[1], mhash[0]));

	/* stop rx temporary */
	if (dp->mac_active) {
		OUTL(dp, COMMAND, CMD_STOP_RX | CMD_STOP_RDMA);

		/* XXX - wait for rx become idle */
		for (i = 0; (INL(dp, INTSTAT) & INT_RXIDLE) == 0; i++) {
			if (i > 1000) {
				cmn_err(CE_CONT,
				    "!%s: %s: failed to stop the nic",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}
	}

	OUTL(dp, RXCON, lp->rxcon | mode);

	/* set multicast hash table up */
	for (i = 0; i < 4; i++) {
		OUTL(dp, MC0 + i*4, mhash[i]);
	}

	/* set station address */
	mac = dp->cur_addr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL/2; i++) {
		OUTL(dp, LAN0 + i*4, (mac[i*2+1] << 8) | mac[i*2]);
	}

	/* enable rx again */
	if (dp->mac_active) {
		OUTL(dp, COMMAND, CMD_RXQUEUED | CMD_START_RX);
	}

	return (GEM_SUCCESS);
}

static int
epfe_set_media(struct gem_dev *dp)
{
	int		i;
	uint8_t		old;
	extern int	gem_speed_value[];
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: setting %dM bps, %s duplex mode",
	    dp->name, __func__, gem_speed_value[dp->speed],
	    dp->full_duplex ? "full" : "half"));

	/*
	 * Notify current duplex mode to MAC
	 */
	old = lp->txcon;
	lp->txcon &= ~TXCON_LPBK;
	if (dp->full_duplex) {
		lp->txcon |= TXCON_LPBK_FDX;
	}

	if (old == lp->txcon) {
		/* no need to change media mode */
		return (GEM_SUCCESS);
	}

	/* stop tx and rx temporary */
	if (dp->mac_active) {
		OUTL(dp, COMMAND,
		    CMD_STOP_RX | CMD_STOP_RDMA | CMD_STOP_TDMA);

		/* XXX - wait for rx become idle */
		for (i = 0;
		    (INL(dp, INTSTAT) & (INT_TXIDLE | INT_RXIDLE))
		    != (INT_TXIDLE | INT_RXIDLE); i++) {
			if (i > 1000) {
				cmn_err(CE_CONT,
				    "!%s: %s: failed to stop the nic",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}
	}

	/* change mode */
	OUTL(dp, TXCON, lp->txcon);
	DPRINTF(0, (CE_CONT, "!%s: %s: txcon:%b",
	    dp->name, __func__, INL(dp, TXCON), TXCON_BITS));

	/* enable rx and tx again */
	if (dp->mac_active) {
		OUTL(dp, COMMAND,
		    CMD_TXQUEUED | CMD_RXQUEUED | CMD_START_RX);
	}

	return (GEM_SUCCESS);
}

static int
epfe_start_chip(struct gem_dev *dp)
{
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Enable receiver */
	OUTL(dp, COMMAND, CMD_START_RX | CMD_RXQUEUED);

	/* Enable interrupt mask */
	lp->genctl |= GC_INTEN;
	OUTL(dp, GENCTL, lp->genctl);

	return (GEM_SUCCESS);
}

static int
epfe_stop_chip(struct gem_dev *dp)
{
	struct epfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* inhibit interrupts */
	lp->genctl &= ~GC_INTEN;
	OUTL(dp, GENCTL, lp->genctl);

	/* stop RxDMA, TxDMA and receiver */
	OUTL(dp, COMMAND, CMD_STOP_RDMA | CMD_STOP_TDMA | CMD_STOP_RX);

	return (GEM_SUCCESS);
}


static uint16_t
epfe_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;
	uint16_t	ret;
	uint8_t		chip_select;
	uint8_t		di;
	uint8_t		cfga_saved;

#define	EPC_EEPROM_DELAY(dp)	{ INL(dp, EECTL); INL(dp, EECTL); }
#define	EE93C46_READ	6

	/* aquire MA bus and ensure de-assert chip select */
	chip_select = EECTL_EN;
	OUTL(dp, EECTL, chip_select);

	/* assert chip select */
	chip_select |= EECTL_EECS;
	OUTL(dp, EECTL, chip_select);
	EPC_EEPROM_DELAY(dp);

	/* make a read command for eeprom */
	offset = (offset & 0x3f) | (EE93C46_READ << 6);

	for (i = 10; i >= 0; i--) {
		/* send 1 bit */
		di = ((offset >> i) & 1) << EECTL_EEDI_SHIFT;

		OUTL(dp, EECTL, chip_select | di);
		EPC_EEPROM_DELAY(dp);

		OUTL(dp, EECTL, chip_select | di | EECTL_EESK);
		EPC_EEPROM_DELAY(dp);
	}

	OUTL(dp, EECTL, chip_select);
	EPC_EEPROM_DELAY(dp);

	/* get the reply and construct a 16bit value */
	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTL(dp, EECTL, chip_select | EECTL_EESK);
		EPC_EEPROM_DELAY(dp);

		ret = (ret << 1)
		    | ((INL(dp, EECTL) >> EECTL_EEDO_SHIFT) & 1);

		OUTL(dp, EECTL, chip_select);
		EPC_EEPROM_DELAY(dp);
	}

	/* negate chip_select */
	OUTL(dp, EECTL, EECTL_EN);
	EPC_EEPROM_DELAY(dp);

	OUTL(dp, EECTL, 0);

	return (ret);
}

#ifdef DEBUG_LEVEL
#define	EPIC_EEPROM_SIZE	128	/* in word */
static void
epfe_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	prom[EPIC_EEPROM_SIZE];

	for (i = 0; i < EPIC_EEPROM_SIZE; i++) {
		prom[i] = epfe_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < EPIC_EEPROM_SIZE; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
}
#endif /* DEBUG_LEVEL */

static int
epfe_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint8_t		*m;
	uint16_t	val;
	struct epfe_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: nvctl: %b, miicfg: %b",
	    dp->name, __func__, INL(dp,  NVCTL), NVCTL_BITS,
	    INL(dp, MIICFG), MIICFG_BITS));
#ifdef notdef
	OUTL(dp, NVCTL,
	    (INL(dp, NVCTL) & NVCTL_IPG_DLY) | NVCTL_GPOE1 | NVCTL_GPIO1);
#endif
	if (lp->chip->chip_type != EPIC_83C175) {
		OUTL(dp, NVCTL, (INL(dp, NVCTL) & ~0x003C) | 0x4800);
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: nvctl: %b",
	    dp->name, __func__, INL(dp,  NVCTL), NVCTL_BITS));

	m = &dp->dev_addr.ether_addr_octet[0];
	for (i = 0; i < ETHERADDRL/2; i++) {
		val = epfe_read_eeprom(dp, i);
		*m++ = (uint8_t)val;
		*m++ = (uint8_t)(val >> 8);
#ifdef notdef
		OUTL(dp, LAN0 + i*4, val);
#endif
	}
#if DEBUG_LEVEL > 0
	epfe_eeprom_dump(dp);
#endif

	/* clear statistics */
	(void) epfe_get_stats(dp);
	bzero(&dp->stats, sizeof (dp->stats));

	/* makeintr mask */
	if (lp->chip->chip_type == EPIC_83C170) {
		lp->intr_buserr = INTR_BUSERR_170;
		lp->int_bits = INT_BITS_170;
	} else {
		lp->intr_buserr = INTR_BUSERR_171;
		lp->int_bits = INT_BITS_171;
	}
	lp->our_intr_mask = INTR_MASK_COMMON | lp->intr_buserr;

	dp->misc_flag |= GEM_VLAN_SOFT;

	dp->rxthr = 128;

	return (GEM_SUCCESS);
}


static int
epfe_get_stats(struct gem_dev *dp)
{
	DPRINTF(4, (CE_CONT, "!%s:%s: called", dp->name, __func__));

	/* 20 CRC ERROR COUNTER: clear on read */
	(void) INB(dp, CRCCNT);

	/* 24 FRAME ALIGNMENT ERROR COUNTER: clear on read */
	(void) INB(dp, ALICNT);

	/* 28 MISSED PACKET COUNTER: clear on read */
	dp->stats.missed += INB(dp, MPCNT) & MPCNT_MASK;

	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
epfe_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
	uint_t			len;
	uint32_t		size;
	uint32_t		addr;
	uint32_t		csr;
	uint32_t		stat;
	struct epic_tfd		*tfdp;
	struct epic_frag	*tfp;
	ddi_dma_cookie_t	*dcp;
	struct epfe_dev		*lp = dp->private;

#if DEBUG_LEVEL > 0
	/* force to cause interrupt upon tx completion */
	flag |= GEM_TXFLAG_INTR;
#endif
	tfdp = &((struct epic_tfd *)dp->tx_ring)[slot];
#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d flag: %lld",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * Make fragment section
	 */
	if (flag & GEM_TXFLAG_INTR) {
		csr |= TD_CTL_IAF;
	} else {
		csr = 0;
	}
#if MAXTXFRAGS > 1
	len = 0;
	tfdp->nfrags = LE_32((uint32_t)frags);

	for (i = 0, dcp = dmacookie, tfp = &tfdp->frag[0];
	    i < frags; dcp++, tfp++, i++) {
		size = dcp->dmac_size;
		addr = dcp->dmac_address;
		len += size;
		tfp->frag_len = LE_32(size);
		tfp->frag_ptr = LE_32(addr);
	}
	/*
	 * Setup descriptor section
	 */
	/* XXX - length field must be zero */
	csr |= TD_CTL_FRAGLIST;
	/* keep tfdp->desc.desc_buf */
	/* keep tfdp->desc.desc_next */
#else
	csr |= TD_CTL_LASTDSCR;
	ASSERT(frags == 1);
	size = dmacookie->dmac_size;
	addr = dmacookie->dmac_address;
	tfdp->desc.desc_buf = LE_32(addr);
	/* keep rfdp->desc.desc_next */

	csr |= size;
	len = size;
#endif
	tfdp->desc.desc_ctl = LE_32(csr);

	/* finally set OWN bit in txstat */
	if (len < ETHERMIN) {
		len = ETHERMIN;
	}
	stat = len << TD_STAT_LEN_SHIFT;
	if ((flag & GEM_TXFLAG_HEAD) == 0) {
		stat |= TD_STAT_OWN;
	}
	tfdp->desc.desc_stat = LE_32(stat);

	DPRINTF(2, (CE_CONT, "  slot[%d]: %08x %08x %08x %08x",
	    slot, tfdp->desc.desc_stat, tfdp->desc.desc_buf,
	    tfdp->desc.desc_ctl, tfdp->desc.desc_next));

	return (1);
}

static void
epfe_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct epfe_dev		*lp = dp->private;

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    start_slot + 1, nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	((struct epic_tfd *)dp->tx_ring)[start_slot].desc.desc_stat |=
	    LE_32(TD_STAT_OWN);
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	OUTL(dp, COMMAND, CMD_TXQUEUED);
}

static void
epfe_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	uint32_t		size;
	uint32_t		addr;
	struct epic_rfd		*rfdp;
	struct epic_frag	*rfp;
	ddi_dma_cookie_t	*dcp;

	rfdp = &((struct epic_rfd *)dp->rx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * Setup fragment section
	 */
#if MAXRXFRAGS > 1
	ASSERT(frags <= MAXRXFRAGS);
	rfdp->nfrags = LE_32(frags);
	for (i = 0, dcp = dmacookie, rfp = &rfdp->frag[0];
	    i < frags; dcp++, rfp++, i++) {
		size = dcp->dmac_size;
		addr = dcp->dmac_address;
		rfp->frag_len = LE_32(size);
		rfp->frag_ptr = LE_32(addr);
	}

	/*
	 * Setup descriptor section
	 */
	rfdp->desc.desc_ctl = LE_32(RD_CTL_FRAGLIST);

	/* XXX - length field must be zero */
	/* keep rfdp->desc.desc_next */
	/* keep rfdp->desc.desc_buf */
#else
	ASSERT(frags == 1);
	size = dmacookie->dmac_size;
	addr = dmacookie->dmac_address;
	rfdp->desc.desc_ctl = LE_32(size);
	rfdp->desc.desc_buf = LE_32(addr);
	/* keep rfdp->desc.desc_next */
#endif

	/* finally set OWN bit in txstat */
	rfdp->desc.desc_stat = LE_32(RD_STAT_OWN);

	DPRINTF(2, (CE_CONT, "  slot[%d]: %08x %08x %08x %08x",
	    slot, rfdp->desc.desc_stat, rfdp->desc.desc_buf,
	    rfdp->desc.desc_ctl, rfdp->desc.desc_next));
}

static uint_t
epfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct epic_tfd		*tfdp;
	uint32_t		txstat;
	uint32_t		cc;
	struct epfe_dev		*lp = dp->private;

	tfdp = &((struct epic_tfd *)dp->tx_ring)[slot];
	DPRINTF(4, (CE_CONT,
	    "!%s: %s: slot:%d, txstat:0x%b intstat:0x%b",
	    dp->name, __func__, slot, tfdp->desc.desc_stat, TD_STAT_BITS,
	    INL(dp, INTSTAT), lp->int_bits));

	txstat = tfdp->desc.desc_stat;
	txstat = LE_32(txstat);
	if (txstat & TD_STAT_OWN) {
		/* not transmitted */
		return (0);
	}

	cc = (txstat & TXSTAT_CC) >> TXSTAT_CC_SHIFT;

	if (txstat & TXSTAT_PT) {
		dp->stats.collisions += cc;
		if (cc == 0) {
			if ((txstat & TXSTAT_ND) == 0) {
				dp->stats.defer++;
			}
		} else if (cc == 1) {
			dp->stats.first_coll++;
		} else {
			dp->stats.multi_coll++;
		}
	} else {
		dp->stats.errxmt++;
		DPRINTF(4, (CE_CONT, "!%s: %s: tx err: txstat:%b",
		    dp->name, __func__, txstat, TD_STAT_BITS));

		if ((txstat & TXSTAT_OWC) != 0) {
			/* late collision */
			dp->stats.xmtlatecoll++;
		}

		if ((txstat & TXSTAT_UN) != 0) {
			/* this must not happen here */
			dp->stats.underflow++;
			DPRINTF(1, (CE_WARN,
			    "!%s: unexpected tx fifo underflow happened",
			    dp->name));
		}

		if (txstat & TXSTAT_CSL) {
			dp->stats.nocarrier++;
		}

		if (cc >= 16) {
			/* max collision exceeded */
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		}
	}
	return (GEM_TX_DONE);
}

static uint64_t
epfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct epic_rfd	*rfdp;
	uint64_t	len;
	uint64_t	flag;
	uint32_t	rxstat;

	flag = GEM_RX_DONE;
	rfdp = &((struct epic_rfd *)dp->rx_ring)[slot];

	rxstat = rfdp->desc.desc_stat;
	rxstat = LE_32(rxstat);

	len = (rxstat & RD_STAT_LEN) >> RD_STAT_LEN_SHIFT;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, rxstat:0x%b",
	    dp->name, __func__, slot, rxstat, RD_STAT_BITS));

	if (rxstat & RD_STAT_OWN) {
		/* the nic still owns the buffer, packets are not received */
		return (0);
	}

	if (len > ETHERFCSL) {
		len -= ETHERFCSL;
	}

	if ((rxstat & RXSTAT_PRI) == 0) {
		/* error packet */
		dp->stats.errrcv++;
		if (rxstat & RXSTAT_FAE) {
			dp->stats.frame++;
		}
		if (rxstat & RXSTAT_CRC) {
			dp->stats.crc++;
		}
		if (rxstat & RD_STAT_FLE) {
			dp->stats.frame_too_long++;
		}

		flag = GEM_RX_ERR;
	}

	return (flag | len);
}

static void
epfe_tx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	addr;
	struct epic_tfd		*tfdp;

	tfdp = &((struct epic_tfd *)dp->tx_ring)[slot];

	/* invalidate this tfd */
	/* XXX - don't clear desc_next because it may have been initialized */
	tfdp->desc.desc_stat = 0;
	tfdp->desc.desc_ctl = 0;
#if MAXTXFRAGS > 1
	/* make a link to the fragment descriptor array */
	addr = ((uint32_t)dp->tx_ring_dma)
	    + slot * sizeof (struct epic_tfd)
	    + ((uint32_t)&((struct epic_tfd *)NULL)->nfrags);
	tfdp->desc.desc_buf = LE_32(addr);
#else
#ifdef SANITY
	tfdp->desc.desc_buf = 0;
#endif
#endif
	/* link it to the previous tfd */
	tfdp = &((struct epic_tfd *)
	    dp->tx_ring)[SLOT(slot - 1, dp->gc.gc_tx_ring_size)];
	addr = ((uint32_t)dp->tx_ring_dma) + slot*sizeof (struct epic_tfd);
	tfdp->desc.desc_next = LE_32(addr);
}

static void
epfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	int		i;
	uint32_t	addr;
	struct epic_rfd		*rfdp;

	rfdp = &((struct epic_rfd *)dp->rx_ring)[slot];

	/* invalidate this rfd */
	/* XXX - don't clear desc_next because it may have been initialized */
	rfdp->desc.desc_stat = 0;
	rfdp->desc.desc_ctl = 0;

#if MAXRXFRAGS > 1
	/* make a link to the fragment descriptor array */
	addr = ((uint32_t)dp->rx_ring_dma)
	    + slot*sizeof (struct epic_rfd)
	    + ((uint32_t)&((struct epic_rfd *)NULL)->nfrags);
	rfdp->desc.desc_buf = LE_32(addr);
#else
#ifdef SANITY
	rfdp->desc.desc_buf = 0;
#endif
#endif

	/* link it to the previous rfd */
	rfdp = &((struct epic_rfd *)
	    dp->rx_ring)[SLOT(slot - 1, dp->gc.gc_rx_ring_size)];
	addr = ((uint32_t)dp->rx_ring_dma) + slot*sizeof (struct epic_rfd);
	rfdp->desc.desc_next = LE_32(addr);
}

static void
epfe_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct epic_tfd		*tfdp;

	tfdp = &((struct epic_tfd *)dp->tx_ring)[slot];

	/* keep desc_addr and desc_next */
	tfdp->desc.desc_stat = 0;

#ifdef SANITY
	tfdp->desc.desc_ctl = 0;

#if MAXTXFRAGS > 1
	/* clear fragment list field */
	tfdp->nfrags = 0;
	bzero(&tfdp->frag[0], sizeof (tfdp->frag));
#endif
#endif
}

static void
epfe_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct epic_rfd		*rfdp;

	rfdp = &((struct epic_rfd *)dp->rx_ring)[slot];

	/* keep desc_addr and desc_next */
	rfdp->desc.desc_stat = 0;

#ifdef SANITY
	rfdp->desc.desc_ctl = 0;

#if MAXRXFRAGS > 1
	/* clear fragment list field */
	rfdp->nfrags = 0;
	bzero(&rfdp->frag[0], sizeof (rfdp->frag));
#endif
#endif
}

/*
 * Device depend interrupt handler
 */
static uint_t
epfe_interrupt(struct gem_dev *dp)
{
	int		i;
	uint32_t	intstat;
	uint_t		restart_tx = 0;
	boolean_t	print_warn = B_FALSE;
	struct epfe_dev	*lp = dp->private;

	/*
	 * Read interrupt status
	 * Do not use IntStatusAck here. It clears IntEnable too.
	 */
	intstat = INL(dp, INTSTAT);

	if ((intstat & INT_INT_ACTV) == 0) {
		/* Not for us */
		if (intstat) {
			OUTL(dp, INTSTAT, intstat);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(4, (CE_CONT, "%s: Interrupt, intstat: %b",
	    dp->name, intstat, lp->int_bits));
	if (!dp->mac_active) {
		/*
		 * the device is not active.
		 * side effect: left interrupt masked.
		 */
		OUTL(dp, INTSTAT, intstat);

		return (DDI_INTR_CLAIMED);
	}

	/* don't mask any interrupts reason, otherwise system may hang */
	OUTL(dp, INTSTAT, intstat);
	FLSHL(dp, INTSTAT);

	intstat &= lp->our_intr_mask;
#ifdef notdef
	/* ignore */
	/* INT_INT_ACTV	 : interrupt active */
	/* RCT: receive copy threshold crossed */
	/* INT_PREI */

	/* not interrupt */
	/* INT_RCTS :	  Receive copy threshold status */
	/* INT_TXIDLE :	 transmit idle */
	/* INT_RXIDLE :	 receive idle */
	/* INT_RSV :	 receive status valid */
	/* INT_TCIP :	 transmit copy in progress */
	/* INT_RCIP :	 receive copy in progress */
	/* INT_RBE : Receive buffer empty : ignore */
#endif
	/* counter overflow */
	if (intstat & INT_CNT) {
		/*
		 * as statistics counters will overflow soon, collect them.
		 */
		(void) epfe_get_stats(dp);
	}

	/* RCC : receive copy complete : normal */
	/* RXE : receive error : normal */
	if (intstat &
	    (INT_RCC  | INT_RCT_171 | INT_PREI | INT_RQE | INT_OVW)) {
		/*
		 * packets were received, or receive errors happened
		 */
#ifdef DEBUG_LEVEL
		lp->prev_rx_head = dp->rx_active_head;
		lp->prev_rx_tail = dp->rx_active_tail;
#endif
		(void) gem_receive(dp);
	}

	/* TCC: transmit chain complete : normal */
	/* TXC: transmit complete : normal */
	if (intstat & (INT_TCC | INT_TXC)) {
		if (gem_tx_done(dp)) {
			restart_tx = INTR_RESTART_TX;
		}
	}

	/* RQE : receive queue empty : warning only */
	/* OVW : receive buffer overflow : fatal */
	if (intstat & (INT_RQE | INT_OVW)) {
#ifdef DEBUG_LEVEL
		DPRINTF(2, (CE_WARN,
		    "!%s: %s int:%b rx_desc<%d %d><%d %d>",
		    dp->name,
		    (intstat & INT_RQE) ? "no rx queue" : "rx overflow",
		    intstat, lp->int_bits,
		    lp->prev_rx_head, lp->prev_rx_tail,
		    dp->rx_active_head, dp->rx_active_tail));
#endif
		if (intstat & INT_RQE) {
			dp->stats.norcvbuf++;
		} else if (intstat & INT_OVW) {
			dp->stats.overflow++;
		}
		OUTL(dp, COMMAND, CMD_RXQUEUED);
	}

	/* transmit underrun : fatal */
	if (intstat & INT_TXU) {
		/*
		 * Packets was transfered into TxFIFO or error happened
		 */
		/* recover from tx underrun */
		cmn_err(CE_WARN, "!%s: tx underrun", dp->name);
		OUTL(dp, COMMAND, CMD_TXUGO);
	}

	/* should not happen: unexpected */
	/* GPIO(2) is low */
	if (intstat & INT_GP2_INT) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: %b",
		    dp->name, intstat, lp->int_bits);
	}

	/* should not happen: bus error */
	/* PCI target abort */
	/* PCI master abort */
	/* PCI address parity error */
	/* PCI data parity error */
	/* INT_FATAL_INT : fatal error (27:24) occured */
	/* PME: power management event */
	if (intstat & lp->intr_buserr) {
		cmn_err(CE_WARN,
		    "!%s: PCI bus error intstat %b, resetting the chip...",
		    dp->name, intstat, lp->int_bits);

		/* restart the chip */
		lp->need_to_reset = B_TRUE;
		print_warn = B_TRUE;
		goto x;
	}

x:
	if (lp->need_to_reset) {
		DPRINTF(-1, (CE_CONT,
		    "%s: %s: isr:%b, resetting the chip ...",
		    dp->name, __func__, intstat, lp->int_bits));

		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);

		restart_tx = INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "%s: %s done: intstat: %b",
	    dp->name, __func__, INL(dp, INTSTAT), lp->int_bits));

	return (DDI_INTR_CLAIMED | restart_tx);
}

/*
 * HW depend MII routine
 */

static void
epfe_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
epfe_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint32_t	ctl;
	uint32_t	val;

	ctl = (dp->mii_phy_addr << MMCTL_PHYADDR_SHIFT)
	    | (reg << MMCTL_PHYREG_SHIFT)
	    | MMCTL_READ;

	DPRINTF(4, (CE_CONT, "%s: %s: addr:0x%x reg:0x%x ctl:0x%x",
	    dp->name, __func__, dp->mii_phy_addr, reg, ctl));

	OUTL(dp, MMCTL, ctl);
	drv_usecwait(10);

	for (i = 0; (INL(dp, MMCTL) & MMCTL_READ) != 0; i++) {
		/* wait */
		if (i > 100) {
			/* timeout */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(10);
	}

	val = INL(dp, MMDATA);
#if DEBUG_LEVEL > 4
	if (val == 0 || val == 0xffff) {
		cmn_err(CE_CONT, "%s: %s: val:0x%x", dp->name, __func__, val);
	}
#endif
	if (reg == MII_STATUS && val == 0xffff) {
		val = 0;
	}
#ifdef CONFIG_10M_ONLY
	if (reg == MII_STATUS) {
		val &= ~(MII_STATUS_100_BASEX | MII_STATUS_100_BASEX_FD);
	}
#endif
	return (val);
}

static void
epfe_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;
	uint32_t	ctl;

	ctl = (dp->mii_phy_addr << MMCTL_PHYADDR_SHIFT)
	    | (reg << MMCTL_PHYREG_SHIFT)
	    | MMCTL_WRITE;

	OUTL(dp, MMDATA, val);
	OUTL(dp, MMCTL, ctl);
	drv_usecwait(10);

	for (i = 0; (INL(dp, MMCTL) & MMCTL_WRITE) != 0; i++) {
		/* wait */
		if (i > 100) {
			/* timeout */
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
epfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_acc_handle_t	conf_handle;
	int			ret;
	int			vid;
	int			did;
	int			unit;
	struct chip_info	*p;
	int			val;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct epfe_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	uint8_t			revid;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "%s%d: %s: called", drv_name, unit, __func__));

	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		goto err;
	}

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	DPRINTF(0, (CE_CONT,
	    "!%s%d: ilr 0x%08x, latency_timer:0x%02x",
	    drv_name, unit,
	    pci_config_get32(conf_handle, PCI_CONF_ILINE),
	    pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER)));

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/*
		 * Check if chip is supported.
		 */
		for (i = 0, p = epfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (p->venid == vid && p->devid == did &&
			    p->rev_min <= revid && revid <= p->rev_max) {
				/* found */
				cmn_err(CE_CONT,
				    "!%s%d: %s "
				    "(vid:%04x, did:%04x, revid: 0x%02x)",
				    drv_name, unit, p->name, vid, did, revid);
				break;
			}
		}
		if (i >= CHIPTABLESIZE) {
			/* Not found */
			cmn_err(CE_WARN,
			    "!%s: %s: wrong PCI venid/devid (0x%x, 0x%x)",
			    drv_name, __func__, vid, did);

			/* assuming latest chipset */
			p = &epfe_chiptbl[CHIPTABLESIZE-1];
		}

		/* Map in the device registers. */
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &epfe_dev_attr,
		    (void *)&base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		/* construct gem configration */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift =
		    epfe_log2(sizeof (struct epic_tfd));
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;

		/* XXX - autopad doesn't work */
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = epfe_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_unit_shift =
		    epfe_log2(sizeof (struct epic_rfd));
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_copy_thresh = epfe_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = epfe_dev_attr;
		gcp->gc_buf_attr = epfe_buf_attr;
		gcp->gc_desc_attr = epfe_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = epfe_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = epfe_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = epfe_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/10;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_NONE;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RSA;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_an_oneshot = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &epfe_attach_chip;
		gcp->gc_reset_chip = &epfe_reset_chip;
		gcp->gc_init_chip = &epfe_init_chip;
		gcp->gc_start_chip = &epfe_start_chip;
		gcp->gc_stop_chip = &epfe_stop_chip;
		gcp->gc_multicast_hash = &epfe_mcast_hash;
		gcp->gc_set_rx_filter = &epfe_set_rx_filter;
		gcp->gc_set_media = &epfe_set_media;
		gcp->gc_get_stats = &epfe_get_stats;
		gcp->gc_interrupt = &epfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &epfe_tx_desc_write;
		gcp->gc_rx_desc_write = &epfe_rx_desc_write;
		gcp->gc_tx_start = &epfe_tx_start;
		gcp->gc_rx_start = NULL;
		gcp->gc_tx_desc_stat = &epfe_tx_desc_stat;
		gcp->gc_rx_desc_stat = &epfe_rx_desc_stat;
		gcp->gc_tx_desc_init = &epfe_tx_desc_init;
		gcp->gc_rx_desc_init = &epfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &epfe_tx_desc_clean;
		gcp->gc_rx_desc_clean = &epfe_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &epfe_mii_sync;
		gcp->gc_mii_read = &epfe_mii_read;
		gcp->gc_mii_write = &epfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof (struct epfe_dev), KM_SLEEP);
		lp->chip = p;

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_handle, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err:
		kmem_free(lp, sizeof (struct epfe_dev));
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
epfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(epfe_ops, epfeattach, epfedetach);
#else
static	struct module_info epfeminfo = {
	0,			/* mi_idnum */
	"epfe",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit epferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&epfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit epfewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&epfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	epfe_info = {
	&epferinit,	/* st_rdinit */
	&epfewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_epfe_ops = {
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
	&epfe_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops epfe_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	epfeattach,	/* devo_attach */
	epfedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_epfe_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};
#endif
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&epfe_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "epfe: _init: called"));
	gem_mod_init(&epfe_ops, "epfe");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&epfe_ops);
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

	DPRINTF(2, (CE_CONT, "epfe: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&epfe_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
