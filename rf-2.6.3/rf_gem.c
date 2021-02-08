/*
 * rf_gem.c : Realtek RTL8129/813x/810x Fast Ethernet Driver for Solaris
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
#pragma ident "@(#)rf_gem.c	1.7 11/09/25"

/*
 *  Changelog:
 01/05/2003 TX DMA burst size (TCR_MXDMAXXX) was fixed  (16byte->128byte)
 01/22/2003 TX DMA burst size (TCR_MXDMAXXX) was changed  (1024)
	0.9.2t release for michael
 01/24/2003 Inter Frame Gap in TCR is too short (8139c data sheet bug)
 01/24/2003 checking method on tx-done fixed according to realtel manual.
	    tx-timeout gone when loopback heatrun test .
 01/25/2003 interrupt processing manner was modified according to realtek manual
	0.9.3t release for michael
 02/02/2003 NOINTR_WORKAROUND added
 03/10/2003 Endianess problem fixed for sparc platform (gem also updated)
 03/22/2003 0.9.7 release
 04/12/2003 ddi_dma_sync added for sparc
 04/20/2003 0.9.8 release
 04/26/2003 FLSH* removed
 05/20/2003 rf_interrupt fixed for missing interrupt of "tx deffered"
 05/20/2003 NOINTR implemented officially
 05/22/2003 0.9.9 release
 05/31/2003 burstsizes in dma attr fixed for sparc
 10/19/2003 MII timeout parameters fixed, 1.0.0 release
 10/23/2003 chip info table modified, 1.0.0 release again
 05/30/2004 interrupts disabled while rf_interrupt().
 05/30/2005 mac address could not be changed.
 */

/*
 * TODO
 *   Rx error test (crc error):  done.
 *   Ensure D0 mode :
 */
/* ======================================================= */

/*
 * Solaris system header files and macros
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
#include "rtl8139reg.h"

char	ident[] = "rtl8129/39 nic driver v" VERSION;

/*
 * Useful macros
 */
#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int rf_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (rf_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Our configration for rtl8139
 */
/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))
#define	TIM_1uS		33

/*
 * RX/TX buffer size
 */
#define	OUR_RBLEN	RBLEN_64K
#define	OUR_RBLEN_SHIFT	16
#ifdef RX_BUF_NOWRAP
#define	RX_BUFFER_SIZE	\
	(min(RBLEN(OUR_RBLEN) + 2048 + RBLEN_PAD, 0x10000))
#else
#define	RX_BUFFER_SIZE	(RBLEN(OUR_RBLEN))
#endif

#ifndef NTXBUF
#define	NTXBUF	32
#endif
#define	TXBUF_LEN	ROUNDUP2(ETHERMAX, 4)
#define	RX_BUF_SIZE	128

#define	IFG	(12)

#define	OUR_INTR_MASK	\
	(INTR_LenChg | INTR_PUN |	\
	INTR_SERR | INTR_FOVF | INTR_RXOVW |	\
	INTR_TER | INTR_TOK | INTR_RER | INTR_ROK)

static int	rf_tx_copy_thresh = 256;
static int	rf_rx_copy_thresh = INT_MAX;

/*
 * Local device definitions
 */
struct chip_info {
	uint32_t	tcr_val; /* from RTL8139C docs */
	int		flags;
	char		*name;
	int		type;
#define	CHIP_8129	0
#define	CHIP_8139	1
#define	CHIP_8139K	2
#define	CHIP_8139A	3
#define	CHIP_8139A_G	4
#define	CHIP_8139B	5
#define	CHIP_8130	6
#define	CHIP_8139C	7
#define	CHIP_8139CP	8
#define	CHIP_8100	9
#define	CHIP_8100B	10
#define	CHIP_8101L	11
};

struct chip_info chiptbl_8129[] = {
	{0,		0,	"RTL8129",	CHIP_8129},
};

struct chip_info chiptbl_8139[] = {
	{HWREV_8139,	0,	"RTL8139",	CHIP_8139},
	{HWREV_8139K,	0,	"RTL8139 rev K", CHIP_8139K},
	{HWREV_8139A,	0,	"RTL8139A",	CHIP_8139A},
	{HWREV_8139A_G,	0,	"RTL8139A_G",	CHIP_8139A_G},
	{HWREV_8139B,	0,	"RTL8139B",	CHIP_8139B},
	{HWREV_8130,	0,	"RTL8130",	CHIP_8130},
	{HWREV_8139C,	0,	"RTL8139C",	CHIP_8139C},
	{HWREV_8100,	0,	"RTL8100",	CHIP_8100},
	{HWREV_8100B,	0,	"RTL8100B/8100C/8139D", CHIP_8100B},
	{HWREV_8139CP,	0,	"RTL8139C_Plus", CHIP_8139CP},
	{HWREV_8101L,	0,	"RTL8101L",	CHIP_8101L},
};

#define	CHIPTABLESIZE	(sizeof (chiptbl_8139) / sizeof (struct chip_info))

struct rf_dev {
	/*
	 * Misc HW information
	 */
	struct chip_info	*chip;
	uint32_t		rcr;
	uint16_t		rx_curpos;
	boolean_t		need_to_reset;
	uint32_t		imr;
	uint8_t			mac[ETHERADDRL];
	uint_t			tim_1us;
#define	HAVE_TIMER(lp)	((lp)->tim_1us != 0)

#ifdef CONFIG_TX_POLLING
	uint_t			tx_pkt_len[NTXDESC];
	uint_t			tx_remain;
#endif
#ifdef CONFIG_RX_POLLING
	int			last_poll_interval;
#endif
	uint32_t		tx_desc_addr[NTXDESC];
	uint32_t		tx_desc_len[NTXDESC];
};

/*
 * private functions
 */

/* mii operations */
static void  rf_mii_sync(struct gem_dev *);
static uint16_t  rf_mii_read(struct gem_dev *, uint_t);
static void rf_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int rf_attach_chip(struct gem_dev *);
static int rf_reset_chip(struct gem_dev *);
static int rf_init_chip(struct gem_dev *);
static int rf_start_chip(struct gem_dev *);
static int rf_stop_chip(struct gem_dev *);
static int rf_set_media(struct gem_dev *);
static int rf_set_rx_filter(struct gem_dev *);
static int rf_get_stats(struct gem_dev *);

/* descriptor operations */
static int rf_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags);
static void rf_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void rf_tx_start(struct gem_dev *dp, int start_slot, int nslot);
static uint_t rf_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t rf_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void rf_tx_desc_init(struct gem_dev *dp, int slot);
static void rf_rx_desc_init(struct gem_dev *dp, int slot);
static void rf_tx_desc_clean(struct gem_dev *dp, int slot);
static void rf_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t rf_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr rf_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr rf_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t rf_dma_attr_txbuf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	4,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t rf_dma_attr_rxbuf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
rf_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint8_t		*m;
	uint32_t	val;
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#ifdef notdef /* debug */
	OUTB(dp, CR9346, CR9346_EEM_WE);
	OUTL(dp, IDR + 0, 0x00888800);
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);

	for (i = 0; i < ETHERADDRL; i++) {
		lp->mac[i] = INB(dp, IDR + i);
	}
#endif
	OUTW(dp, IMR, 0);

	OUTB(dp, CR, CR_RST);

	for (i = 0; (INB(dp, CR) & CR_RST); i++) {
		if (i > 100) {
			/* time out */
			cmn_err(CE_WARN, "!%s: %s: failed to reset: timeout",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}

	return (GEM_SUCCESS);
}

/*
 * Setup rtl8139
 */
static int
rf_init_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* reset rx buffer position */
	lp->rx_curpos = 0;

	/* ID registers: set later in rf_set_rx_filter */

	/* Multicast registers: set later in rf_set_rx_filter */

	/* Interrupt status register: do nothing */

	/* Interrupt mask register */
	lp->imr = 0;
	OUTW(dp, IMR, lp->imr);

	/* rx buffer start address */
	OUTL(dp, RBSTART, dp->io_area_dma);

	/* Current address of packet read */
	OUTW(dp, CAPR, lp->rx_curpos - RBLEN_PAD);

	/* Current buffer address (RO) */
	ASSERT(INW(dp, CBR) == 0);

	/* Transmit status of descriptors: no need to touch */

	/* Transmit start address of descriptors: no need to touch */

	/* Timer count register: no need to touch */
#ifdef notdef /* moved to rf_attach_chip */
	OUTL(dp, TimInt, 0);
	OUTL(dp, TCTR, 0);
	drv_usecwait(1000);
	lp->tim_1us = INL(dp, TCTR)/1000;
	DPRINTF(1, (CE_CONT, "!%s: tim_1us:%d", dp->name, lp->tim_1us));
#endif
	/* Missed packet counter: clear it */
	OUTL(dp, MPC, 0);

	/* multiple interrupt register: no need to touch */

	/* make BMCR and CONFIG registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	/* Configuration register 0: no need to change */
	DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG0: 0x%02x",
	    dp->name, __func__, INB(dp, CONFIG0)));

	/* Configuration register 1: enable powermanagement ? */
	DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG1: 0x%02x",
	    dp->name, __func__, INB(dp, CONFIG1)));

	val = INB(dp, CONFIG1) | CONFIG1_DVRLOAD;
	if (lp->chip->type >= CHIP_8139A) {
		val &= ~CONFIG1_LWACT;
		val |= CONFIG1_PMEn;
		if (lp->chip->type >= CHIP_8139B) {
			val |= CONFIG1_VPD;
		}
	} else {
		/* for 8129 / 8139 :  wake the chip up */
		val &= ~(CONFIG1_SLEEP | CONFIG1_PWRDWN);
	}
	OUTB(dp, CONFIG1, val);

	/* Configuration register 3: */
	if (lp->chip->type >= CHIP_8139B) {
		/* Disable Magic packet */
		DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG3: 0x%02x",
		    dp->name, __func__, INB(dp, CONFIG3)));
		OUTB(dp, CONFIG3, INB(dp, CONFIG3) & ~CONFIG3_Magic);
	}

	/* Configuration register 4: default:RxFIFOAutoClr=1 */
	if (lp->chip->type >= CHIP_8139) {
		DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG4: 0x%02x",
		    dp->name, __func__, INB(dp, CONFIG4)));
		OUTB(dp, CONFIG4, INB(dp, CONFIG4) | CONFIG4_RxFIFOAutoClr);
	}

	/* Configuration register 5: */
	DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG5: 0x%02x",
	    dp->name, __func__, INB(dp, CONFIG5)));

	/* make BMCR and config registers read only */
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	drv_usecwait(10);

	/* Receive configuration register: don't touch here */
	lp->rcr = 0;

#ifdef CONFIG_TX_POLLING
	lp->tx_remain = 0;
#endif
	/* clear mac address cache */
	bzero(lp->mac, ETHERADDRL);

	return (GEM_SUCCESS);
}

static uint_t
rf_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6));
}

static int
rf_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	uint64_t	mhash;
	int		i;
	uint8_t		*m;
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mode = RCR_AB	/* accept broadcast */
	    | RCR_APM;	/* accept physical match */

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* accept all multicast packets */
		mode |= RCR_AM;
		mhash = ~0ULL;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscious mode implies all multicast and all physical */
		mode |= RCR_AM | RCR_AAP;
		mhash = ~0ULL;
	} else if (dp->mc_count > 0) {
		/*
		 * make hash table to select interresting
		 * multicast address only.
		 */
		mode |= RCR_AM;
		mhash = 0ULL;
		for (i = 0; i < dp->mc_count; i++) {
			/* hash table is 64 = 2^6 bit width */
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	lp->rcr &= ~(RCR_AB | RCR_APM | RCR_AM | RCR_AAP);
#ifdef notdef	/* debug */
	if (dp->mac_active) {
		/* enable all rx filters while changing */
		OUTL(dp, MAR + 0, ~0U);
		OUTL(dp, MAR + 4, ~0U);
		OUTL(dp, RCR, lp->rcr | RCR_AB | RCR_APM | RCR_AM | RCR_AAP);
	}
#endif
	/* setup mac address */
	m = &dp->cur_addr.ether_addr_octet[0];
	if (bcmp(m, lp->mac, ETHERADDRL) != 0) {
		/* make ID registers writable */
		OUTB(dp, CR9346, CR9346_EEM_WE);

		OUTL(dp, IDR + 0,
		    (m[3] << 24) | (m[2] << 16) | (m[1] << 8) | m[0]);
		OUTL(dp, IDR + 4, (m[5] <<  8) | m[4]);

		/* make ID registers read only */
		OUTB(dp, CR9346, CR9346_EEM_NORMAL);
		drv_usecwait(10);

		bcopy(m, lp->mac, ETHERADDRL);
	}

	if (mode & RCR_AM) {
		/* need to set up multicast hash table */
		OUTL(dp, MAR + 0, (uint32_t)mhash);
		OUTL(dp, MAR + 4, (uint32_t)(mhash >> 32));
	}

	lp->rcr |= mode;
	if (dp->mac_active) {
		/* update rcr */
		OUTL(dp, RCR, lp->rcr);
	}

	DPRINTF(2, (CE_CONT, "!%s: %s: returned", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
rf_set_media(struct gem_dev *dp)
{
	uint32_t	val;
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* select duplex */

	if (lp->chip->type == CHIP_8129) {
		/* make BMCR and CONFIG registers writable */
		OUTB(dp, CR9346, CR9346_EEM_WE);

		val = INB(dp, CONFIG1) & ~CONFIG1_FULL;
		if (dp->full_duplex) {
			val |= CONFIG1_FULL;
		}
		OUTB(dp, CONFIG1, val);

		OUTB(dp, CR9346, CR9346_EEM_NORMAL);
		drv_usecwait(10);
	}

	/* select speed: do nothing */

	/* flow control */
	val = INB(dp, MSR) & ~(MSR_TXFCE | MSR_RXFCE);

	switch (dp->flow_control) {
	case FLOW_CONTROL_RX_PAUSE:
		val |= MSR_RXFCE;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		val |= MSR_TXFCE;
		break;

	case FLOW_CONTROL_SYMMETRIC:
		val |= MSR_TXFCE | MSR_RXFCE;
		break;
	}

	OUTB(dp, MSR, val);

	DPRINTF(2, (CE_CONT, "!%s: %s: returned", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
rf_start_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Command register : Enable Tx and Rx before writing TCR and RCR */
	OUTB(dp, CR, CR_RE | CR_TE);

	/* Transmit configration register : */
	if (dp->txmaxdma <= 16) {
		val = TCR_MXDMA_16;
	} else if (dp->txmaxdma <= 32) {
		val = TCR_MXDMA_32;
	} else if (dp->txmaxdma <= 64) {
		val = TCR_MXDMA_64;
	} else if (dp->txmaxdma <= 128) {
		val = TCR_MXDMA_128;
	} else if (dp->txmaxdma <= 256) {
		val = TCR_MXDMA_256;
	} else if (dp->txmaxdma <= 512) {
		val = TCR_MXDMA_512;
	} else if (dp->txmaxdma <= 1024) {
		val = TCR_MXDMA_1024;
	} else {
		val = TCR_MXDMA_2048;
	}
	OUTL(dp, TCR, TCR_IFG_802_3 | TCR_LBK_NORMAL | val);

	/* Receive configuration register : */
	val = 0;
	if (dp->rxmaxdma <= 16) {
		val |= RCR_MXDMA_16;
	} else if (dp->rxmaxdma <= 32) {
		val |= RCR_MXDMA_32;
	} else if (dp->rxmaxdma <= 64) {
		val |= RCR_MXDMA_64;
	} else if (dp->rxmaxdma <= 128) {
		val |= RCR_MXDMA_128;
	} else if (dp->rxmaxdma <= 256) {
		val |= RCR_MXDMA_256;
	} else if (dp->rxmaxdma <= 512) {
		val |= RCR_MXDMA_512;
	} else if (dp->rxmaxdma <= 1024) {
		val |= RCR_MXDMA_1024;
	} else {
		val |= RCR_MXDMA_UNLIMIT;
	}
	if (dp->rxthr <= 16) {
		val |= RCR_RXFTH_16;
	} else if (dp->rxthr <= 32) {
		val |= RCR_RXFTH_32;
	} else if (dp->rxthr <= 64) {
		val |= RCR_RXFTH_64;
	} else if (dp->rxthr <= 128) {
		val |= RCR_RXFTH_128;
	} else if (dp->rxthr <= 256) {
		val |= RCR_RXFTH_256;
	} else if (dp->rxthr <= 512) {
		val |= RCR_RXFTH_512;
	} else if (dp->rxthr <= 1024) {
		val |= RCR_RXFTH_1024;
	} else {
		val |= RCR_RXFTH_NONE;
	}

	lp->rcr |= RCR_ERTH_NONE | RCR_RER8 | val
	    | (OUR_RBLEN << RCR_RBLEN_SHIFT)
#ifdef RX_BUF_NOWRAP
	    | RCR_WRAP
#endif
	    | RCR_AER | RCR_AR;

	OUTL(dp, RCR, lp->rcr);

	/* Enable interrupts */
	lp->imr = OUR_INTR_MASK;
#ifdef CONFIG_TX_POLLING
	if (HAVE_TIMER(lp)) {
		lp->imr &= ~INTR_TOK;
		lp->imr |= INTR_TIMEOUT;
	}
#endif
#ifdef CONFIG_RX_POLLING
	if (HAVE_TIMER(lp)) {
		lp->imr |= INTR_TIMEOUT;
	}
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IMR, lp->imr);
	}

	return (GEM_SUCCESS);
}

static int
rf_stop_chip(struct gem_dev *dp)
{
	struct rf_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable the rx filter completely */
	lp->rcr &= ~(RCR_AB | RCR_APM | RCR_AM | RCR_AAP);
	OUTL(dp, RCR, lp->rcr);

	/* disable interrupts */
	/*
	 * don't clear lp->imr and the interrupt source to process interrupts
	 * correctly, that we may have now.
	 */
#ifdef NEVER
	lp->imr = 0;
#endif
	OUTW(dp, IMR, 0);
#ifdef NEVER
	OUTW(dp, ISR, 0xffff);
#endif

	/* disable RX and TX */
	OUTB(dp, CR, 0);
	drv_usecwait(2000);

	return (GEM_SUCCESS);
}

static int
rf_get_stats(struct gem_dev *dp)
{
	/* read missed count */
	dp->stats.missed += INL(dp, MPC);

	/* clear missed count */
	OUTL(dp, MPC, 0);

	return (GEM_SUCCESS);
}


/*
 * discriptor  manupiration
 */
#ifdef CONFIG_TX_POLLING
__INLINE__ static void
rf_sched_tx_timer(struct gem_dev *dp)
{
	uint32_t	clks;
	struct rf_dev	*lp = (dp)->private;

	if (dp->speed == GEM_SPD_100) {
		clks = lp->tim_1us * (lp->tx_remain * 8) / 100;
	} else {
		clks = lp->tim_1us * (lp->tx_remain * 8) / 10;
	}
	clks = (clks/8) * 7;
	OUTL(dp, TimInt, max(clks, 1));
	OUTL(dp, TCTR, 0);
}
#pragma inline(rf_sched_tx_timer)
#endif
static int
rf_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int		len;
	int		i;
	struct rf_dev	*lp = dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: time:%d %s seqnum: %d, slot %d, frags: %d flags: %llx",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!    %d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	OUTL(dp, TSAD + slot*4, dmacookie->dmac_address);

	/* use early transmission for performance */
	len = dmacookie->dmac_size & TSR_SIZE;

	/* early tx does not seem to work in 10M half mode */
	OUTL(dp, TSD  + slot*4, TSR_ERTXTH(dp->txthr) | len);

#ifdef CONFIG_TX_POLLING
	if (HAVE_TIMER(lp)) {
		lp->tx_remain += lp->tx_pkt_len[slot] = len + IFG;
		if (flags & GEM_TXFLAG_INTR) {
			rf_sched_tx_timer(dp);
		}
	}
#endif
	return (1);
}

static void
rf_rx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	/* EMPTY */
}

static void
rf_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	/* nothing to do */
}

static uint_t
rf_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	tsr;
	uint32_t	val;
	uint32_t	errbits;
	struct rf_dev	*lp = dp->private;

	tsr = INL(dp, TSD + slot*4);

	DPRINTF(3, (CE_CONT,
	    "!%s: time:%d rf_tx_desc_stat: seqnum: %d, slot %d, tsr: %b",
	    dp->name, ddi_get_lbolt(),
	    dp->tx_desc_head, slot, tsr, TX_STATUS_BITS));

	if ((tsr & (TSR_TOK | TSR_TABT | TSR_TUN)) == 0) {
		/*
		 * The packet is under transmitting. Wait.
		 */
		return (0);
	}

	if (tsr & TSR_TUN) {
		/* this may happen when early tx */
		dp->stats.underflow++;
		dp->txthr = min(ETHERMAX, dp->txthr*2);
		DPRINTF(0, (CE_WARN,
		    "!%s: tx fifo underflow happened",
		    dp->name));
	}

	errbits = TSR_TABT | TSR_OWC | TSR_CRS;
	if (!dp->full_duplex) {
		/* CRS seems not reliable in 100mbps half duplex mode */
		errbits &= ~TSR_CRS;
	}

	if (tsr & errbits) {
		dp->stats.errxmt++;
		DPRINTF(4, (CE_CONT, "!%s: tx err: tsr:%b",
		    dp->name, tsr, TX_STATUS_BITS));

		if (tsr & TSR_CRS & errbits) {
			/* carrier lost */
			dp->stats.nocarrier++;
		}
		if (tsr & TSR_OWC & errbits) {
			/* late collision */
			dp->stats.xmtlatecoll++;
		}
		if (tsr & TSR_TABT & errbits) {
			/* max collision exceeded */
			dp->stats.excoll++;
			dp->stats.collisions += 16;
			DPRINTF(2, (CE_NOTE, "!%s: tx aborted", dp->name));
			OUTB(dp, TCR, (INB(dp, TCR) & TCR_TXRR) | TCR_CLRABT);
			/*
			 * Don't advance current tx pointer as we
			 * issued TCR_CLRABT above.
			 */
			return (0);
		}
	} else {
		if (val = (tsr & TSR_NCC)) {
			val >>= TSR_NCC_SHIFT;
			dp->stats.collisions += val;
			if (val == 1) {
				dp->stats.first_coll++;
			} else {
				dp->stats.multi_coll++;
			}
		}
	}
#ifdef CONFIG_TX_POLLING
	if (HAVE_TIMER(lp)) {
		lp->tx_remain -= lp->tx_pkt_len[slot];
	}
#endif
	return (GEM_TX_DONE);
}

static mblk_t *
rf_get_packet(struct gem_dev *dp, struct rxbuf *rbp, size_t len)
{
	uint16_t	rsr;
	uint16_t	*p;
	size_t		rest;
	uint16_t	offset;
	mblk_t		*mp = NULL;
	struct rf_dev	*lp = dp->private;

	/* get receive status and packet length */
	offset = lp->rx_curpos & (RBLEN(OUR_RBLEN) - 1);
	p = (uint16_t *)&dp->io_area[offset];

	/* read rx header atomicly */
	rsr = p[0];
	rsr = LE_16(rsr);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: status:%b len:%d at offset:0x%04x cur:0x%04x",
	    dp->name, ddi_get_lbolt(), __func__,
	    rsr, RSR_BITS, len, offset, lp->rx_curpos));

	/* read packet and advance current position */
	offset = (lp->rx_curpos + 4) & (RBLEN(OUR_RBLEN) - 1);

	/* update current position */
	lp->rx_curpos += ROUNDUP2(4 /* header size */ + len + ETHERFCSL, 4);

	/* check if any errors happened */
#ifdef GEM_CONFIG_GLDv3	/* for vlan */
#define	RSR_ERROR	(RSR_ISE | RSR_FAE | RSR_RUNT | RSR_CRC)
	if (rsr & (RSR_ERROR))
#else
	if ((rsr & (RSR_ROK)) == 0)
#endif
	{
		DPRINTF(0, (CE_CONT, "!%s: rsr:%b", dp->name, rsr, RSR_BITS));
		dp->stats.errrcv++;
		if (rsr & (RSR_ISE | RSR_FAE)) {
			dp->stats.frame++;
		} else if (rsr & RSR_RUNT) {
			dp->stats.runt++;
#ifndef GEM_CONFIG_GLDv3	/* for vlan */
		} else if (rsr & RSR_LONG) {
			dp->stats.frame_too_long++;
#endif
		} else if (rsr & RSR_CRC) {
			dp->stats.crc++;
		} else {
			dp->stats.rcv_internal_err++;
		}

		/*
		 * Need to reset the chip to recover from error state
		 */
		lp->need_to_reset = B_TRUE;
		goto done;
	}

	/* aquire packet buffer */
	if ((mp = allocb(len, BPRI_MED)) == NULL) {
		/*
		 * No receive buffer, OS resource exaust
		 */
		dp->stats.norcvbuf++;
		dp->stats.errrcv++;
	} else {
		mp->b_wptr = mp->b_rptr + len;
#if defined(RX_BUF_NOWRAP) && (OUR_RBLEN != RBLEN_64K)
		ddi_dma_sync(dp->desc_dma_handle,
		    (off_t)(offset + (dp->io_area - dp->rx_ring)), len,
		    DDI_DMA_SYNC_FORKERNEL);

		bcopy(dp->io_area + offset, mp->b_rptr, len);
#else
		rest = min(RBLEN(OUR_RBLEN) - offset, len);
		ddi_dma_sync(dp->desc_dma_handle,
		    (off_t)(offset + (dp->io_area - dp->rx_ring)), rest,
		    DDI_DMA_SYNC_FORKERNEL);

		bcopy(dp->io_area + offset, mp->b_rptr, rest);
		if ((len -= rest) > 0) {
			ddi_dma_sync(dp->desc_dma_handle, (off_t)0, len,
			    DDI_DMA_SYNC_FORKERNEL);

			bcopy(dp->io_area, mp->b_rptr + rest, len);
		}
#endif
	}
done:
	/* set new limit */
	OUTW(dp, CAPR, lp->rx_curpos - RBLEN_PAD);
	return (mp);
}

static uint64_t
rf_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	size_t		len;
	uint16_t	rsr;
	uint16_t	offset;
	uint16_t	*p;
	struct rf_dev	*lp = dp->private;

	if (INB(dp, CR) & CR_BUFE) {
		/* Rx buffer is empty */
		return (0);
	}

	offset = lp->rx_curpos & (RBLEN(OUR_RBLEN) - 1);
	ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(offset + (dp->io_area - dp->rx_ring)), 4,
	    DDI_DMA_SYNC_FORKERNEL);

	/* get receive status and packet length */
	p = (uint16_t *)&dp->io_area[offset];

	/* read rx header atomicly */
	rsr = p[0];
	rsr = LE_16(rsr);
	len = p[1];
	len = LE_16(len);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: status:%b len:%d at offset:0x%04x cur:0x%04x",
	    dp->name, ddi_get_lbolt(), __func__,
	    rsr, RSR_BITS, len, offset, lp->rx_curpos));

	if (len == 0xfff0) {
		/* 8139 is busy to copy a packet into memory */
		return (0);
	}

	if (len > ETHERFCSL) {
		/* remove crc code */
		len -= ETHERFCSL;
	}

	return (GEM_RX_DONE | (len & GEM_RX_LEN));
}

static void
rf_tx_desc_init(struct gem_dev *dp, int slot)
{
	OUTL(dp, TSD + slot*4, 0);
}

static void
rf_rx_desc_init(struct gem_dev *dp, int slot) { return; }

static void
rf_tx_desc_clean(struct gem_dev *dp, int slot)
{
	OUTL(dp, TSD + slot*4, 0);
}

static void
rf_rx_desc_clean(struct gem_dev *dp, int slot) { return; }

/*
 * Device depend interrupt handler
 */
static uint_t
rf_interrupt(struct gem_dev *dp)
{
	uint16_t	isr;
	uint16_t	imr;
	uint_t		restart_tx = 0;
	boolean_t	update_imr = B_FALSE;
	struct rf_dev	*lp = dp->private;

	/* XXX - reading ISR does not clear interrupt source */
	isr = INW(dp, ISR);
	imr = lp->imr;
	if ((isr & imr) == 0) {
		/* not for us */
#ifdef NEVER
		/*
		 * Don't clear TOK and ROK interrupt source bits,
		 * they may be serviced later, in case of TX/RX polling.
		 */
		if (isr) {
			OUTW(dp, ISR, isr);
		}
#endif
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: time:%d %s: isr:%b imr:%b",
	    dp->name, ddi_get_lbolt(), __func__, isr, INTR_BITS,
	    INW(dp, IMR), INTR_BITS));

	if (!dp->mac_active) {
		/* no futher interrupts */
		lp->imr = 0;
		OUTW(dp, IMR, lp->imr);

		/* ack to all interrupts */
		OUTW(dp, ISR, 0xffff);
		return (DDI_INTR_CLAIMED);
	}

#ifdef CONFIG_TX_POLLING
	if (isr & INTR_TIMEOUT) {
		imr |= INTR_TOK;
	}
#endif
#ifdef CONFIG_RX_POLLING
	if (isr & INTR_TIMEOUT) {
		imr |= INTR_ROK;
	}

	if (dp->speed == GEM_SPD_100 && HAVE_TIMER(lp) &&
	    dp->poll_interval != lp->last_poll_interval) {
		if (lp->last_poll_interval == 0) {
			/*
			 * To schedule the next timer interrupt,
			 * we pretend as we were interrupted from
			 * polling timer
			 */
			isr |= INTR_TIMEOUT;
		}
		lp->last_poll_interval = dp->poll_interval;
	}
#endif /* CONFIG_POLLING */

	/* clear interrupt sources explicitly */
	isr &= imr;
	OUTW(dp, ISR, isr);

	if (isr & (INTR_LenChg | INTR_PUN)) {
		/*
		 * Link or PHY status changed
		 */
		DPRINTF(2, (CE_CONT, "!%s: isr:%b",
		    dp->name, isr, INTR_BITS));
		if (gem_mii_link_check(dp)) {
			restart_tx = INTR_RESTART_TX;
		}
	}

	if (isr & (INTR_ROK | INTR_RER | INTR_FOVF | INTR_RXOVW)) {
		if (isr & (INTR_FOVF | INTR_RXOVW)) {
			if (isr & INTR_RXOVW) {
				dp->stats.norcvbuf++;
			} else {
				dp->stats.overflow++;
			}
			dp->stats.errrcv++;
			DPRINTF(1, (CE_CONT, "!%s: isr:%b",
			    dp->name, isr, INTR_BITS));
		}
		(void) gem_receive(dp);
	}

	if (isr & (INTR_TOK | INTR_TER)) {
		if (gem_tx_done(dp)) {
			restart_tx = INTR_RESTART_TX;
		}
	}

	if (isr & INTR_TIMEOUT) {
		mutex_enter(&dp->xmitlock);
#ifdef CONFIG_TX_POLLING
		if (dp->tx_desc_intr - dp->tx_desc_head > 0) {
			/* need to reschedule tx-intr again */
			rf_sched_tx_timer(dp);
		} else
#endif
#ifdef CONFIG_RX_POLLING
		if (dp->poll_interval) {
			uint_t	cnt;
			/* schedule the next rx polling interrupt */
			cnt = dp->poll_interval * lp->tim_1us / 1000;
			OUTL(dp, TimInt, max(cnt, lp->tim_1us));
			OUTL(dp, TCTR, 0);

			/* disble rx interrupt */
			if (lp->imr & INTR_ROK) {
				lp->imr &= ~INTR_ROK;
				update_imr = B_TRUE;
			}
		} else
#endif
		{
			/* disable timer interrupt */
			OUTL(dp, TimInt, 0);

			/* enable rx interrupt */
			lp->imr |= INTR_ROK;
			update_imr = B_TRUE;

		}
		mutex_exit(&dp->xmitlock);
	}

	if (isr & INTR_SERR) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b",
		    dp->name, isr, INTR_BITS);
		lp->need_to_reset = B_TRUE;
	}

	if (lp->need_to_reset) {
		gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		restart_tx = INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0 && update_imr) {
		/* change or enable imr */
		OUTW(dp, IMR, lp->imr);
	}

	return (DDI_INTR_CLAIMED | restart_tx);
}

/*
 * MII Interfaces
 */
/* for rtl8129 */
#define	MII_DELAY(dp, r)	{ INB(dp, r); INB(dp, r); }

static void
rf_mii_sync_external(struct gem_dev *dp)
{
	int		i;
	int		reg;
	struct rf_dev	*lp = dp->private;

	reg = (lp->chip->type == CHIP_8129) ? MII8129 : MII8130;

	for (i = 0; i < 32; i++) {
		OUTB(dp, reg, MII_MDM | MII_MDO);
		MII_DELAY(dp, reg);
		OUTB(dp, reg, MII_MDM | MII_MDO | MII_MDC);
		MII_DELAY(dp, reg);
	}
}

static uint16_t
rf_mii_read_external(struct gem_dev *dp, uint_t index)
{
	int		i;
	uint_t		data;
	uint16_t	ret;
	uint_t		cmd;
	int		reg;
	struct rf_dev	*lp = dp->private;

	reg = (lp->chip->type == CHIP_8129) ? MII8129 : MII8130;

	cmd = MII_READ_CMD(dp->mii_phy_addr, index);

	/* send read command */
	for (i = 31; i >= 18; i--) {
		data = ((cmd >> i) & 1) << MII_MDO_SHIFT;
		OUTB(dp, reg, MII_MDM | data);
		MII_DELAY(dp, reg);

		OUTB(dp, reg, MII_MDM | data | MII_MDC);
		MII_DELAY(dp, reg);
	}

	/* turn around */
	OUTB(dp, reg, 0);
	MII_DELAY(dp, reg);

	/* notify the phy to send ack */
	OUTB(dp, reg, MII_MDC);
	MII_DELAY(dp, reg);

	/* get response from the phy */
	OUTB(dp, reg, 0);
	if (INB(dp, reg) & MII_MDI) {
		DPRINTF(2, (CE_CONT, "!%s: phy@%d didn't respond",
		    dp->name, dp->mii_phy_addr));
	}

#ifdef SANITY
	ret = 0;
#endif
	for (i = 16; i > 0; i--) {
		OUTB(dp, reg, MII_MDC);
		MII_DELAY(dp, reg);

		OUTB(dp, reg, 0);
		ret = (ret << 1) | ((INB(dp, reg) >> MII_MDI_SHIFT) & 1);
	}

	/* revert the clock cycle and send idle bits */
	for (i = 0; i < 2; i++) {
		OUTB(dp, reg, MII_MDM | MII_MDO);
		MII_DELAY(dp, reg);
		OUTB(dp, reg, MII_MDM | MII_MDO | MII_MDC);
		MII_DELAY(dp, reg);
	}

	DPRINTF(5, (CE_CONT, "!%s: %s: ret: 0x%x",
	    dp->name, __func__, ret));

	return (ret);
}

static void
rf_mii_write_external(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int		i;
	uint32_t	cmd;
	int		data;
	int		reg;
	struct rf_dev	*lp = dp->private;

	reg = (lp->chip->type == CHIP_8129) ? MII8129 : MII8130;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, index, val);

	/* send write command and data */
	for (i = 31; i >= 0; i--) {
		data = ((cmd >> i) & 1) << MII_MDO_SHIFT;
		OUTB(dp, reg, data | MII_MDM);
		MII_DELAY(dp, reg);
		OUTB(dp, reg, data | MII_MDM | MII_MDC);
		MII_DELAY(dp, reg);
	}

	/*
	 * Send 2 idle clock cycles to ensure that the transmittion
	 * is terminated.
	 */
	for (i = 0; i < 2; i++) {
		OUTB(dp, reg, MII_MDM | MII_MDO);
		MII_DELAY(dp, reg);
		OUTB(dp, reg, MII_MDM | MII_MDO | MII_MDC);
		MII_DELAY(dp, reg);
	}
}
#undef	MII_DELAY

static void
rf_mii_sync_internal(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
rf_mii_read_internal(struct gem_dev *dp, uint_t index)
{
	int	reg;

	DPRINTF(5, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	switch (index) {
	case MII_CONTROL:
		reg = BMCR;
		break;

	case MII_STATUS:
		reg = BMSR;
		break;

	case MII_AN_ADVERT:
		reg = ANAR;
		break;

	case MII_AN_LPABLE:
		reg = ANLPAR;
		break;

	case MII_AN_EXPANSION:
		reg = ANER;
		break;

	default:
		return (0);
	}

	return (INW(dp, reg));
}

static void
rf_mii_write_internal(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int	reg;

	DPRINTF(5, (CE_CONT, "!%s: %s called, reg:0x%x val:0x%04x",
	    dp->name, __func__, index, val));

	switch (index) {
	case MII_CONTROL:
		reg = INB(dp, CR9346);
		OUTB(dp, CR9346, CR9346_EEM_WE);
		OUTW(dp, BMCR, val);
		OUTB(dp, CR9346, reg);
		return;

	case MII_STATUS:
		reg = BMSR;
		break;

	case MII_AN_ADVERT:
		reg = ANAR;
		break;

	case MII_AN_LPABLE:
		reg = ANLPAR;
		break;

	case MII_AN_EXPANSION:
		reg = ANER;
		break;

	default:
		return;
	}

	OUTW(dp, reg, val);
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
#define	rf_eeprom_delay(dp)	{INB(dp, CR9346); INB(dp, CR9346); }
#ifdef EEPROMIO
#include "rf_eeprom_wr.c"
#endif /* EEPROMIO */
static uint16_t
rf_read_eeprom(struct gem_dev *dp, int addr)
{
	int		i;
	int		addr_bits;
	uint_t		cmd;
	uint8_t		chip_select;
	uint8_t		di;
	uint16_t	ret;
	struct rf_dev	*lp = dp->private;

	if (lp->chip->type == CHIP_8139B || lp->chip->type == CHIP_8139C ||
	    lp->chip->type == CHIP_8139CP) {
		/*
		 * eeprom 93C46 or 93C56: 8139B, 8139C, 8139CP
		 */
		addr_bits = ((INL(dp, RCR) & RCR_9356SEL)) ? 8 : 6;
	} else {
		/*
		 * eeprom 93C46 : 8129, 8130, 8100, 8100B/8139D, 8101
		 */
		addr_bits = 6;
	}

	DPRINTF(2, (CE_CONT, "!%s: %s: called: addr_bits:%d",
	    dp->name, __func__, addr_bits));

	/* make command bits */
	cmd = (6 << addr_bits) | addr;

	/* enable eeprom interface register */
	chip_select = CR9346_EEM_PROGRAM;
	OUTB(dp, CR9346, chip_select);

	chip_select |= CR9346_EECS;
	OUTB(dp, CR9346, chip_select);
	rf_eeprom_delay(dp);

	/* output eeprom command */
	for (i = 4 + addr_bits; i >= 0; i--) {
		di = ((cmd >> i) << CR9346_EEDI_SHIFT) & CR9346_EEDI;
		OUTB(dp, CR9346, chip_select | di);
		rf_eeprom_delay(dp);

		OUTB(dp, CR9346, chip_select | di | CR9346_EESK);
		rf_eeprom_delay(dp);
	}

	/* release clock but keep chip_select asserted */
	OUTB(dp, CR9346, chip_select);
	rf_eeprom_delay(dp);

	/* get returned value */
	ret = 0;
	for (i = 16; i > 0; i--) {
		/* get 1 bit */
		OUTB(dp, CR9346, chip_select | CR9346_EESK);
		rf_eeprom_delay(dp);

		ret = (ret << 1)
		    | ((INB(dp, CR9346) >> CR9346_EEDO_SHIFT) & 1);

		OUTB(dp, CR9346, chip_select);
		rf_eeprom_delay(dp);
	}

	/* Terminate the EEPROM access. */
	OUTB(dp, CR9346, CR9346_EEM_PROGRAM);
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	rf_eeprom_delay(dp);

	DPRINTF(2, (CE_CONT, "!%s: %s: returned 0x%x",
	    dp->name, __func__, ret));

	return (ret);
}

static void
rf_eeprom_dump(struct gem_dev *dp, int size)
{
	int	i;

	cmn_err(CE_CONT, "!%s: %s dump:", dp->name, __func__);
	for (i = 0; i < size; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, rf_read_eeprom(dp, i), rf_read_eeprom(dp, i + 1),
		    rf_read_eeprom(dp, i + 2), rf_read_eeprom(dp, i + 3));
	}
}

static int
rf_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	uint8_t		*m;
	struct rf_dev	*lp = dp->private;

	/*
	 * reload default mac address from EEPROM
	 */
	m = dp->dev_addr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL; i += 2) {
		val = rf_read_eeprom(dp, (EPROM_EthernetID + i) / 2);
		m[i + 0] = (uint8_t)val;
		m[i + 1] = (uint8_t)(val >> 8);
	}
	bzero(lp->mac, ETHERADDRL);

#ifdef notdef
	if (lp->chip->type == CHIP_8139C) {
		/* enable flow control */
		rtl8139c_eeprom[0xc/2] &= ~0x0002;
		rf_control_eeprom(dp, WEN);
		for (i = 0; i < 0x30; i++) {
			rf_write_eeprom(dp, i, rtl8139c_eeprom[i]);
		}
		rf_control_eeprom(dp, WDS);
	}
	DPRINTF(0, (CE_CONT, "!%s: reg70:%04x, reg74:%04x",
	    dp->name, INW(dp, 0x70), INW(dp, 0x74)));
#endif

#ifdef notdef
	if (lp->chip->type == CHIP_8139B) {
		rf_control_eeprom(dp, WEN);
		for (i = 0; i < 0x40; i++) {
			rf_write_eeprom(dp, i, rtl8139b_eeprom[i]);
		}
		rf_control_eeprom(dp, WDS);
	}
	DPRINTF(0, (CE_CONT, "!%s: reg70:%04x, reg74:%04x",
	    dp->name, INW(dp, 0x70), INW(dp, 0x74)));
#endif
	rf_eeprom_dump(dp, 0x40);

	if (lp->chip->type != CHIP_8129) {
		/* no need to scan PHY */
		dp->mii_phy_addr = -1;
	}

#ifndef TX_UNDERRUN_TEST
	dp->txmaxdma = min(dp->txmaxdma, 256);
	dp->txmaxdma = max(dp->txmaxdma, 1024);
#endif
	dp->txmaxdma = min(dp->txmaxdma, 256);

	if (ddi_prop_exists(DDI_DEV_T_ANY,
	    dp->dip, DDI_PROP_DONTPASS, "txthr") == 0) {
		/* fix default value for txthr */
		dp->txthr = 256;
	}

	/* check if we have timer function */
#define	TEST_PERIOD	100
	lp->tim_1us = 0;
	OUTW(dp, IMR, 0);
	OUTL(dp, TimInt, TIM_1uS*TEST_PERIOD/2);
	OUTL(dp, TCTR, 0);
	OUTW(dp, ISR, 0xffffU);
	drv_usecwait(TEST_PERIOD);
	if (INW(dp, ISR) & INTR_TIMEOUT) {
		/* we got interrupted, the timer is alive. */
		OUTW(dp, ISR, 0);
		lp->tim_1us = INL(dp, TCTR)/TEST_PERIOD;
		cmn_err(CE_CONT, "!%s: timer resolution:%duS",
		    dp->name, lp->tim_1us);

		/* check the result, expected value is 33 */
		if (lp->tim_1us < TIM_1uS/2 || lp->tim_1us > TIM_1uS*3) {
			lp->tim_1us = 0;
		}
	}
	OUTL(dp, TimInt, 0);
#undef	TEST_PERIOD

	dp->misc_flag |= GEM_POLL_RXONLY | GEM_NORXBUF;
#ifdef GEM_CONFIG_GLDv3	/* for vlan */
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif

#if 0 /* DEBUG */
	dp->txmaxdma = 1024;
	dp->rxmaxdma = dp->mtu;
	dp->rxthr = dp->mtu;
#endif
	return (GEM_SUCCESS);
}

static int
rfattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_acc_handle_t	conf_handle;
	int			ret;
	int			vid;
	int			did;
	int			revid;
	int			unit;
	struct chip_info	*p;
	uint32_t		tcr;
	uint_t			len;
	const char		*drv_name;
	struct gem_dev		*dp;
	caddr_t			base;
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	struct rf_dev		*lp;
	uint32_t		ilr;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

	/* ensure I/O accesss and bus master */
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    pci_config_get16(conf_handle, PCI_CONF_COMM)
	    | PCI_COMM_IO | PCI_COMM_ME);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	/* fix latency timer */
	if (pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER) < 16) {
		pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 16);
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);

	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

#ifdef notdef
	/* 8139 doesn't have cacle_line_size register */
	pci_config_put8(conf_handle, PCI_CONF_CACHE_LINESZ, 64/4);
	DPRINTF(0, (CE_CONT, "!%s%d: cache line size: 0x%02x", drv_name, unit,
	    pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ)));
#endif

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/* Map in the device registers.  */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_IO, PCI_ADDR_MASK,
		    &rf_dev_attr,
		    &base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		/* Check the chip if it is really 8129/8139.  */
		tcr = ddi_get32(regs_handle, (uint32_t *)(base + TCR));

		/* Try rtl8129 first */
		if (vid == 0x10ec && did == 0x8129) {
			p = &chiptbl_8129[0];
			goto chip_found;
		}

		/* Try rtl8139 */
		for (i = 0; i < CHIPTABLESIZE; i++) {
			if (chiptbl_8139[i].tcr_val == (tcr & TCR_HWREV)) {
				/* found */
				p = &chiptbl_8139[i];
				goto chip_found;
			}
		}

		cmn_err(CE_WARN,
		    "!%s%d: unknown variant of rtl8139 chip: "
		    "venid:0x%04x devid:%04x tcr:0x%08x",
		    drv_name, unit, vid, did, tcr);

		/* assume latest variant of the 8139 */
		/* XXX - RTL8101L for now */
		p = &chiptbl_8139[CHIPTABLESIZE-1];
chip_found:
		cmn_err(CE_CONT, "!%s%d: %s rev:0x%02x tcr:0x%08x",
		    drv_name, unit, p->name, revid, tcr);

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_tx_max_frags = 1;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift = -1;
		gcp->gc_tx_buf_size = NTXBUF;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = NTXDESC;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = rf_tx_copy_thresh;
		gcp->gc_tx_desc_write_oo = B_FALSE;

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = -1;
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = rf_rx_copy_thresh;

		gcp->gc_io_area_size = RX_BUFFER_SIZE;

		/* map attributes (endianess) */
		gcp->gc_dev_attr = rf_dev_attr;		/* for registers */
		gcp->gc_buf_attr = rf_buf_attr;		/* for tx buffers */
		gcp->gc_desc_attr = rf_buf_attr;	/* for rx buffers */

		/* dma attributes */
		gcp->gc_dma_attr_desc = rf_dma_attr_rxbuf;
		gcp->gc_dma_attr_txbuf = rf_dma_attr_txbuf;
		gcp->gc_dma_attr_rxbuf = rf_dma_attr_rxbuf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = (25*ONESEC)/10;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		gcp->gc_mii_an_delay = ONESEC/10;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_hw_link_detection = B_TRUE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &rf_attach_chip;
		gcp->gc_reset_chip = &rf_reset_chip;
		gcp->gc_init_chip = &rf_init_chip;
		gcp->gc_start_chip = &rf_start_chip;
		gcp->gc_stop_chip = &rf_stop_chip;
		gcp->gc_multicast_hash = &rf_mcast_hash;
		gcp->gc_set_rx_filter = &rf_set_rx_filter;
		gcp->gc_set_media = &rf_set_media;
		gcp->gc_get_stats = &rf_get_stats;
		gcp->gc_interrupt = &rf_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &rf_tx_desc_write;
		gcp->gc_rx_desc_write = &rf_rx_desc_write;
		gcp->gc_tx_start = &rf_tx_start;
		gcp->gc_tx_desc_stat = &rf_tx_desc_stat;
		gcp->gc_rx_desc_stat = &rf_rx_desc_stat;
		gcp->gc_tx_desc_init = &rf_tx_desc_init;
		gcp->gc_rx_desc_init = &rf_rx_desc_init;
		gcp->gc_tx_desc_clean = &rf_tx_desc_clean;
		gcp->gc_rx_desc_clean = &rf_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		if (p->type == CHIP_8129) {
			/* rtl8129 */
			gcp->gc_mii_sync = &rf_mii_sync_external;
			gcp->gc_mii_read = &rf_mii_read_external;
			gcp->gc_mii_write = &rf_mii_write_external;
		} else {
			/* rtl8139 */
			gcp->gc_mii_sync = &rf_mii_sync_internal;
			gcp->gc_mii_read = &rf_mii_read_internal;
			gcp->gc_mii_write = &rf_mii_write_internal;
		}
		gcp->gc_mii_tune_phy = NULL;
		gcp->gc_get_packet = &rf_get_packet;

		/* offload and jumbo frame */
		gcp->gc_max_lso = 0;
		gcp->gc_max_mtu = ETHERMTU;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		lp = kmem_zalloc(sizeof (struct rf_dev), KM_SLEEP);
		lp->chip = p;

		dp = gem_do_attach(dip, 0, gcp, base, &regs_handle,
		    lp, sizeof (struct rf_dev));

		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof (struct rf_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
rfdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(rf_ops, rfattach, rfdetach);
#else
static	struct module_info rfminfo = {
	0,			/* mi_idnum */
	"rf",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit rfrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&rfminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit rfwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&rfminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	rf_info = {
	&rfrinit,	/* st_rdinit */
	&rfwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_rf_ops = {
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
	&rf_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops rf_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	rfattach,	/* devo_attach */
	rfdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_rf_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&rf_ops,	/* driver ops */
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

	DPRINTF(0, (CE_CONT, "!rf: _init: called"));

	gem_mod_init(&rf_ops, "rf");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&rf_ops);
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

	DPRINTF(0, (CE_CONT, "!rf: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&rf_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
