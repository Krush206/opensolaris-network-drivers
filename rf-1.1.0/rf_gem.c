/*
 * rf_gem.c : Realtek RTL8129/813x/810x Fast Ethernet Driver for Solaris
 *
 * Copyright (c) 2002-2005 Masayuki Murayama.  All rights reserved.
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
#pragma ident "%W% %E%"

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
 05/04/2006 mac address was wrong. chaned to get it without autoload function.
 07/16/2006 polling mode fixed
 */

/*
 * TODO
    Rx error test (crc error):  done.
    Ensure D0 mode :
 */
/* ======================================================= */

/*
 * Solaris system header files and macros
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

#include "gem.h"
#include "mii.h"
#include "rtl8139reg.h"

char	ident[] = "rtl8129/39 nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/*
 * Useful macros
 */
#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))
#define	FALSE	(0)
#define	TRUE	(!FALSE)
#ifndef	INT32_MAX
# define INT32_MAX	0x7fffffff
#endif

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int rf_debug = DEBUG_LEVEL;
#define DPRINTF(n, args)	if (rf_debug > (n)) cmn_err args
#else
#define DPRINTF(n, args)
#endif

/*
 * Our configration for rtl8139
 */
/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))
#define	TIM_1us		33

/*
 * RX/TX buffer size
 */
#define	OUR_RBLEN	RBLEN_64K
#ifdef RX_BUF_NOWRAP
#define	RX_BUFFER_SIZE	\
	(min(RBLEN(OUR_RBLEN) + ETHERMAX + ETHERFCSL, 0x10000) + RBLEN_PAD)
#else
#define	RX_BUFFER_SIZE	(RBLEN(OUR_RBLEN) + RBLEN_PAD)
#endif

#ifndef NTXBUF
#define NTXBUF	32
#endif
#define TXBUF_LEN	ROUNDUP2(ETHERMAX, 4)

#define	IFG	(12)

#ifdef TX_DELAYED_INTR
#define	OUR_INTR_MASK	\
	(INTR_LenChg | INTR_PUN |	\
	 INTR_SERR | INTR_TIMEOUT | INTR_FOVF | INTR_RXOVW |	\
	 INTR_TER | INTR_RER | INTR_ROK)
#else
#define	OUR_INTR_MASK	\
	(INTR_LenChg | INTR_PUN |	\
	 INTR_SERR | INTR_TIMEOUT | INTR_FOVF | INTR_RXOVW |	\
	 INTR_TER | INTR_TOK | INTR_RER | INTR_ROK)
#endif

static int	rf_tx_copy_thresh = 256;
static int	rf_rx_copy_thresh = INT32_MAX;

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

#define	CHIPTABLESIZE	(sizeof(chiptbl_8139)/sizeof(struct chip_info))

struct rf_dev {
	/*
	 * Misc HW information
	 */
	struct chip_info	*chip;
	uint32_t		rcr;
	uint16_t		rx_curpos;
	boolean_t		need_to_reset;
	uint32_t		imr;
	boolean_t		have_timer;
	uint8_t			mac[ETHERADDRL];
	uint_t			tim_1us;
#ifdef TX_DELAYED_INTR
	uint_t			tx_pkt_len[NTXDESC];
	uint_t			tx_remain;
#endif
#ifdef GEM_CONFIG_POLLING
	int			last_poll_interval;
#endif
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
static void rf_init_chip(struct gem_dev *);
static void rf_start_chip(struct gem_dev *);
static int rf_stop_chip(struct gem_dev *);
static void rf_set_media(struct gem_dev *);
static void rf_set_rx_filter(struct gem_dev *);
static void rf_get_stats(struct gem_dev *);

/* descriptor operations */
static int rf_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq);
static int rf_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t rf_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t rf_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void rf_tx_desc_init(struct gem_dev *dp, int slot);
static void rf_rx_desc_init(struct gem_dev *dp, int slot);
static void rf_tx_desc_clean(struct gem_dev *dp, int slot);
static void rf_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int rf_interrupt(struct gem_dev *dp);

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
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

#ifdef notdef /*  DEBUG_LEVEL > 0 */
	OUTB(dp, CR9346, CR9346_EEM_WE);
	OUTL(dp, IDR + 0, 0x00888800);
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);

	for (i = 0; i < ETHERADDRL; i++) {
		lp->mac[i] = INB(dp, IDR + i);
	}
#endif
	OUTB(dp, CR, CR_RST);

	for (i = 0; (INB(dp, CR) & CR_RST) != 0; i++) {
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
static void
rf_init_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* reset rx buffer position */
	lp->rx_curpos = 0;

	/* ID registers: set later by rf_set_rx_filter */

	/* Multicast registers: set later by rf_set_rx_filter */

	/* Interrupt status register: do nothing */

	/* Interrupt mask register */
	lp->imr = 0;
	OUTW(dp, IMR, lp->imr);

	/* rx buffer start address */
	OUTL(dp, RBSTART, dp->rx_ring_dma);

	/* Current address of packet read */
	OUTW(dp, CAPR, lp->rx_curpos - RBLEN_PAD);

	/* Current buffer address (RO) */
	ASSERT(INW(dp, CBR) == 0);

	/* Transmit status of descriptors: no need to touch */

	/* Transmit start address of descriptors: no need to touch */

	/* Timer count register: no need to touch */
	OUTL(dp, TimInt, 0);
	OUTL(dp, TCTR, 0);
	drv_usecwait(1000);
	lp->tim_1us = INL(dp, TCTR)/1000;
	DPRINTF(1, (CE_CONT, "!%s: tim_1us:%d", dp->name, lp->tim_1us));

	/* Missed packet counter: clear it */
	OUTL(dp, MPC, 0);

	/* multiple interrupt register: no need to touch */

	/* make BMCR and CONFIG registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	/* Configuration register 0: no need to change */
	DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG0: 0x%02x",
			dp->name, __func__, INB(dp, CONFIG0)));

	/* Configuration register 1: enable powermanagement ?*/
	DPRINTF(1, (CE_CONT, "!%s: %s: CONFIG1: 0x%02x",
				dp->name, __func__, INB(dp, CONFIG1)));

	val = INB(dp, CONFIG1) | CONFIG1_DVRLOAD;
	if (lp->chip->type >= CHIP_8139A) {
		val &= ~CONFIG1_LWACT;
		val |= CONFIG1_PMEn;
		if (lp->chip->type >= CHIP_8139B) {
			val |= CONFIG1_VPD;
		}
	}
	else {
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

	lp->have_timer = FALSE;
	if ((dp->misc_flag & GEM_NOINTR) == 0 &&
	    lp->tim_1us >= TIM_1us/2 &&
	    lp->chip->type >= CHIP_8139C) {
		lp->have_timer = TRUE;
	}
#ifdef TX_DELAYED_INTR
	lp->tx_remain  = 0;
#endif
}

static uint_t
rf_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr));
}

static void
rf_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	uint32_t	mhash[2];
	int		i;
	uint8_t		*m;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mode = RCR_AB	/* accept broadcast */
	     | RCR_APM;	/* accept physical match  */

	if ((dp->rxmode & RXMODE_ALLMULTI) != 0) {
		/* accept all multicast packets */
		mode |= RCR_AM;
		mhash[0] = 0xffffffff;
		mhash[1] = 0xffffffff;
	}	
	else if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		/* promiscious mode implies all multicast and all physical */
		mode |= RCR_AM | RCR_AAP;
		mhash[0] = 0xffffffff;
		mhash[1] = 0xffffffff;
	}
	else if (dp->mc_count > 0) {
		mode |= RCR_AM;
		if (dp->mc_count < 64/2) {
			/*
			 * make hash table to select interresting
			 * multicast address only.
			 */
			mhash[0] = 0;
			mhash[1] = 0;
			for (i = 0; i < dp->mc_count; i++) {
				uint_t	h;
				/* hash table is 64 = 2^6 bit width */
				h = dp->mc_list[i].hash >> (32 - 6);
				mhash[h / 32] |= 1 << (h % 32);
			}
		}
		else {
			/* too many multicast addresses */
			mhash[0] = 0xffffffff;
			mhash[1] = 0xffffffff;
		}
	}	

	lp->rcr &= ~(RCR_AB | RCR_APM | RCR_AM | RCR_AAP);
	if (dp->nic_active) {
		/* disable all rx filters before changing */
		OUTL(dp, RCR, lp->rcr);
	}

	/* set mac address up */	
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
	}

	if ((mode & RCR_AM) != 0) {
		/* need to set up multicast hash table */
		OUTL(dp, MAR + 0, mhash[0]);
		OUTL(dp, MAR + 4, mhash[1]);
	}

	lp->rcr |= mode;
	if (dp->nic_active) {
		/* update rcr */
		OUTL(dp, RCR, lp->rcr);
	}

	DPRINTF(2, (CE_CONT, "!%s: %s: returned", dp->name, __func__));
}

static void
rf_set_media(struct gem_dev *dp)
{
	uint32_t	val;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

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
}

static void
rf_start_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

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
	} else if (dp->rxmaxdma<= 128) {
		val |= RCR_MXDMA_128;
	} else if (dp->rxmaxdma<= 256) {
		val |= RCR_MXDMA_256;
	} else if (dp->rxmaxdma<= 512) {
		val |= RCR_MXDMA_512;
	} else if (dp->rxmaxdma<= 1024) {
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
#ifdef NOWRAP
		| RCR_WRAP
#endif
		| RCR_AER | RCR_AR;

	OUTL(dp, RCR, lp->rcr);

	/* Enable interrupts */
	lp->imr = OUR_INTR_MASK;
	if (!lp->have_timer) {
		lp->imr |= INTR_TOK;
		lp->imr &= ~INTR_TIMEOUT;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IMR, lp->imr);
	}
}

static int
rf_stop_chip(struct gem_dev *dp)
{
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable all rx filters */
	lp->rcr &= ~(RCR_AB | RCR_APM | RCR_AM | RCR_AAP);
	OUTL(dp, RCR, lp->rcr);

	/* disable interrupts */
	lp->imr = 0;
	OUTW(dp, IMR, lp->imr);

	/* Clear pended interrupt */
	OUTW(dp, ISR, 0xffff);

	/* disable RX and TX */
	OUTB(dp, CR, 0);
	drv_usecwait(2000);

	return (GEM_SUCCESS);
}

static void
rf_get_stats(struct gem_dev *dp)
{
	/* read missed count */
	dp->stats.missed += INL(dp, MPC);

	/* clear missed count */
	OUTL(dp, MPC, 0);
}


/*
 * discriptor  manupiration
 */
#define	rf_sched_timer(dp)	\
{	\
	uint32_t	clks;	\
	struct rf_dev	*lp = (struct rf_dev *)(dp)->private;	\
	\
	if (dp->speed == GEM_SPD_100) {	\
		clks = lp->tim_1us * (lp->tx_remain * 8) / 100;	\
	} else {	\
		clks = lp->tim_1us * (lp->tx_remain * 8) / 10;	\
	}	\
	clks = (clks/8) * 7;	\
	clks += INL(dp, TCTR);	\
	OUTL(dp, TimInt, clks ? clks : 1); \
}

static int
rf_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq)
{
	int		len;
	int		i;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
    "!%s: time:%d rf_tx_desc_write seqnum: %d, slot %d, frags: %d flags: %d",
		dp->name, ddi_get_lbolt(),
		dp->tx_desc_tail, slot, frags, intreq);
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

#ifdef TX_DELAYED_INTR
	if (lp->have_timer) {
		lp->tx_remain += lp->tx_pkt_len[slot] = len + IFG;
		if (intreq != 0) {
			rf_sched_timer(dp);
		}
	}
#endif
	return (1);
}

static int
rf_rx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags)
{
	/* do nothing, fake caller */
	return (1);
}

static uint_t
rf_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	tsr;
	uint32_t	val;
	uint32_t	errbits;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

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

	if ((tsr & TSR_TUN) != 0) {
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

	if ((tsr & errbits) != 0) {
		dp->stats.errxmt++;
		DPRINTF(4, (CE_CONT, "!%s: tx err: tsr:%b",
			dp->name, tsr, TX_STATUS_BITS));

		if ((tsr & TSR_CRS & errbits) != 0) {
			/* carrier lost */
			dp->stats.nocarrier++;
		} else if ((tsr & TSR_OWC & errbits) != 0) {
			/* late collision */
			dp->stats.xmtlatecoll++;
		} else {
			/* max collision exceeded */
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		}

		if ((tsr & TSR_TABT) != 0) {
			DPRINTF(2, (CE_WARN, "!%s: tx aborted", dp->name));
			OUTB(dp, TCR, (INB(dp, TCR) & TCR_TXRR) | TCR_CLRABT);

			/*
			 * Don't advance current tx pointer as we 
			 * issued TCR_CLRABT above.
			 */
			return (0);
		}
	}
	else {
		if ((val = (tsr & TSR_NCC)) != 0) {
			val >>= TSR_NCC_SHIFT;
			dp->stats.collisions += val;
			if (val == 1) {
				dp->stats.first_coll++;
			} else {
				dp->stats.multi_coll++;
			}
		}
	}
#ifdef TX_DELAYED_INTR
	if (lp->have_timer) {
		lp->tx_remain -= lp->tx_pkt_len[slot];
	}
#endif
	return (GEM_TX_DONE);
}

static mblk_t *
rf_get_packet(struct gem_dev *dp, struct rxbuf *rbp, size_t len)
{
	size_t		rest;
	uint16_t	offset;
	mblk_t		*mp;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	/* read packet and advance current position */
	offset = (lp->rx_curpos + 4) & (RBLEN(OUR_RBLEN) - 1);

	/* update current position */
	lp->rx_curpos += ROUNDUP2(4/* header size */ + len + ETHERFCSL, 4);

	/* aquire packet buffer */
	if ((mp = allocb(len, BPRI_MED)) == NULL) {
		/*
		 * No receive buffer, OS resource exaust
		 */
		dp->stats.norcvbuf++;
	}
	else {
		mp->b_wptr = mp->b_rptr + len;
#if defined(NOWRAP) && (OUR_RBLEN != RBLEN_64K)
		ddi_dma_sync(dp->desc_dma_handle, (off_t)offset, len,
			DDI_DMA_SYNC_FORKERNEL);

		bcopy(dp->rx_ring + offset, mp->b_rptr, len);
#else
		rest = min(RBLEN(OUR_RBLEN) - offset, len);
		ddi_dma_sync(dp->desc_dma_handle, (off_t)offset, rest,
			DDI_DMA_SYNC_FORKERNEL);

		bcopy(dp->rx_ring + offset, mp->b_rptr, rest);
		if ((len -= rest) > 0) {
			ddi_dma_sync(dp->desc_dma_handle, (off_t) 0, len,
				DDI_DMA_SYNC_FORKERNEL);

			bcopy(dp->rx_ring, mp->b_rptr + rest, len);
		}
#endif
	}

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
	uint8_t		*p;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	/* test receive status */
	if ((INB(dp, CR) & CR_BUFE) != 0) {
		/* Rx buffer is empty */
		return (0);
	}

	offset = lp->rx_curpos & (RBLEN(OUR_RBLEN) - 1);
	ddi_dma_sync(dp->desc_dma_handle, (off_t)offset, 4,
		DDI_DMA_SYNC_FORKERNEL);

	/* get receive status and packet length */
	p = (uint8_t *)&dp->rx_ring[offset];

	/* read Rx header */
	rsr = (p[1] << 8) | p[0];
	len = (p[3] << 8) | p[2];

	DPRINTF(2, (CE_CONT,
   "!%s: time:%d rf_rx_desc_stat: status:%b len:%d at offset:0x%04x cur:0x%04x",
		dp->name, ddi_get_lbolt(),
		rsr, RSR_BITS, len, offset, lp->rx_curpos));

	if (len == 0xfff0) {
		/* 8139 is busy to copy a packet into memory */
		return (0);
	}

	/* check if error happen */
	if ((rsr & (RSR_ROK)) == 0) {

		DPRINTF(0, (CE_CONT, "!%s: rsr:%b", dp->name, rsr, RSR_BITS));

		if ((rsr & (RSR_ISE | RSR_FAE)) != 0) {
			dp->stats.frame++;
		}
		if ((rsr & RSR_RUNT) != 0) {
			dp->stats.runt++;
		}
		if ((rsr & RSR_LONG) != 0) {
			dp->stats.frame_too_long++;
		}
		if ((rsr & RSR_CRC) != 0) {
			dp->stats.crc++;
		}
		dp->stats.errrcv++;

		/*
		 * Need to reset the chip to recover from error state
		 */
		lp->need_to_reset = TRUE;
		return (GEM_RX_ERR);
	}

	if (len > ETHERMAX + ETHERFCSL || len < ETHERMIN + ETHERFCSL) {
		if (len > ETHERMAX + ETHERFCSL) {
			dp->stats.frame_too_long++;
		} else {
			dp->stats.runt++;
		}

		/*
		 * Need to reset the chip to recover from error state
		 */
		lp->need_to_reset = TRUE;
		return (GEM_RX_ERR);
	}

	/* remove crc code */
	len -= ETHERFCSL;

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
static u_int
rf_interrupt(struct gem_dev *dp)
{
	uint16_t	isr;
	u_int		restart_tx = 0;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	/* XXX - reading ISR does not clear interrupt source */
	isr = INW(dp, ISR);
	if ((isr & lp->imr) == 0) {
		/* not for us */
		if (isr != 0) {
			OUTW(dp, ISR, isr);
		}
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT,"!%s: time:%d rf_interrupt: isr:%b imr:%b",
		dp->name, ddi_get_lbolt(), isr, INTR_BITS,
		INW(dp, IMR), INTR_BITS));

	/* disable interrupts */
	OUTW(dp, IMR, 0);

	if (!dp->nic_active) {
		/* inhibit interrupt for ever */
		lp->imr = 0;
		/* ack to all interrupts */
		OUTW(dp, ISR, 0xffff);
		return (DDI_INTR_CLAIMED);
	}

	OUTW(dp, ISR, isr);
#ifdef notdef
	/* clear interrupt sources explicitly */
	isr &= lp->imr;
#endif

#ifdef GEM_CONFIG_POLLING
	if (lp->have_timer && dp->speed == GEM_SPD_100 &&
	    dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval != 0) {
			/* polling mode */
			lp->imr = OUR_INTR_MASK & ~(INTR_ROK | INTR_TOK);

			if (lp->last_poll_interval == 0) {
				/*
				 * To schedule the next timer interrupt,
				 * we pretend as we were interrupted from
				 * polling timer
				 */
				isr |= INTR_TIMEOUT;
			}
		}
		else {
			/* normal mode */
			lp->imr = OUR_INTR_MASK;
		}
		lp->last_poll_interval = dp->poll_interval;
	}
#endif /* GEM_CONFIG_POLLING */

	if ((isr & (INTR_LenChg | INTR_PUN)) != 0) {
		/*
		 * Link or PHY status changed
		 */
		DPRINTF(2, (CE_CONT, "!%s: isr:%b",
			dp->name, isr, INTR_BITS));
		gem_mii_link_check(dp);
	}

	if ((isr & (INTR_ROK | INTR_RER | INTR_FOVF | INTR_RXOVW)) != 0) {
		if ((isr & (INTR_FOVF | INTR_RXOVW)) != 0) {
			dp->stats.overflow++;
			dp->stats.errrcv++;
			DPRINTF(0, (CE_CONT, "!%s: isr:%b",
				dp->name, isr, INTR_BITS));
		}
		gem_receive(dp);
	}

#if defined(TX_DELAYED_INTR) || defined(GEM_CONFIG_POLLING)
	if ((isr & (INTR_TOK | INTR_TER | INTR_TIMEOUT)) != 0)
#else
	if ((isr & (INTR_TOK | INTR_TER)) != 0)
#endif
	{
		if (gem_tx_done(dp)) {
			restart_tx = INTR_RESTART_TX;
		}
	}

	if ((isr & INTR_TIMEOUT) != 0) {
		mutex_enter(&dp->xmitlock);
#ifdef TX_DELAYED_INTR
		if (dp->tx_desc_intr - dp->tx_desc_head > 0) {
			/* need to reschedule tx-intr again */
			rf_sched_timer(dp);
		} else
#endif
#ifdef GEM_CONFIG_POLLING
		if (lp->last_poll_interval != 0) {
			/* schedule next polling interrupt */
			OUTL(dp, TCTR, 0);
			OUTL(dp, TimInt, dp->poll_interval * 33);
		} else
#endif /* GEM_CONFIG_POLLING */
		{
			/* disable timer interrupt */
			OUTL(dp, TimInt, 0);
		}
		mutex_exit(&dp->xmitlock);
	}

	if ((isr & INTR_SERR) != 0) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b",
			dp->name, isr, INTR_BITS);
			lp->need_to_reset = TRUE;
	}

	if (lp->need_to_reset) {
		mutex_enter(&dp->xmitlock);
		gem_restart_nic(dp, TRUE);
		mutex_exit(&dp->xmitlock);
		restart_tx = INTR_RESTART_TX;
		lp->need_to_reset = FALSE;
	}
	else if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* enable interrupts again */
		OUTW(dp, IMR, lp->imr);
	}

	return (DDI_INTR_CLAIMED | restart_tx);
}

/* 
 * MII Interfaces
 */
/* for rtl8129 */
#define	MII_DELAY(dp, r)	INB(dp, r)

static void
rf_mii_sync_external(struct gem_dev *dp)
{
	int	i;
	int	reg;
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

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
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

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
        if ((INB(dp, reg) & MII_MDI) != 0) {
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
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

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

	DPRINTF(5, (CE_CONT, "!%s: %s called", dp->name, __func__));

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
	return;
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
#define	rf_eeprom_delay(dp)	{INB(dp, CR9346); INB(dp, CR9346);}

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
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	if (lp->chip->type == CHIP_8139B || lp->chip->type == CHIP_8139C ||
	    lp->chip->type == CHIP_8139CP) {
		/*
		 * eeprom 93C46 or 93C56: 8139B, 8139C, 8139CP
		 */
		addr_bits = ((INL(dp, RCR) & RCR_9356SEL) != 0) ? 8 : 6;
	}
	else {
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
	int		i;

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
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
	char		propname[32];
	struct rf_dev	*lp = (struct rf_dev *)dp->private;

	/*
	 * reload default mac address from EEPROM
	 */
	m = dp->dev_addr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL; i += 2) {
		val = rf_read_eeprom(dp, (EPROM_EthernetID + i) / 2);
		m[i    ] = (uint8_t) val;
		m[i + 1] = (uint8_t) (val >> 8);
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

	sprintf(propname, "%s-txthr", dp->name);
	if (ddi_prop_exists(DDI_DEV_T_ANY,
			dp->dip, DDI_PROP_DONTPASS, propname) == 0) {
		/* fix default value for txthr */
		dp->txthr = 256;
	}
#ifdef GEM_CONFIG_POLLING
	sprintf(propname, "%s-pkt-delay", dp->name);
	if (ddi_prop_exists(DDI_DEV_T_ANY,
			dp->dip, DDI_PROP_DONTPASS, propname) == 0) {
		/* fix rfN-pkt-delay value when we use its default value */
		if (lp->chip->type >= CHIP_8139C) {
			dp->poll_pkt_delay = min(dp->poll_pkt_delay, NTXDESC-1);
		} else {
			dp->poll_pkt_delay = 0;
		}
	}
#endif /* GEM_CONFIG_POLLING */
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
	u_int			len;
	const char		*drv_name;
	struct gem_dev		*dp;
	void			*base;
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	struct rf_dev		*lp;
	uint32_t		ilr;

	unit     =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: rfattach: called", drv_name, unit));

	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
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
		if (gem_pci_regs_map_setup(dip, PCI_ADDR_IO, &rf_dev_attr,
			(caddr_t *)&base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		/* Check the chip if it is really 8129/8139.  */
		tcr = ddi_get32(regs_handle, (uint32_t *)((caddr_t)base + TCR));

		/* Try rtl8129 first*/
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
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof(uint32_t) - 1;
		gcp->gc_tx_max_frags = 1;
		gcp->gc_tx_desc_size = 0;
		gcp->gc_tx_ring_size = NTXDESC;
		gcp->gc_tx_buf_size  = NTXBUF;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_auto_pad  = FALSE;
		gcp->gc_tx_copy_thresh = rf_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;	/* a dummy descriptor for Rx */
		gcp->gc_rx_desc_size = RX_BUFFER_SIZE; /* Rx ring buffer */
		gcp->gc_rx_ring_size = 0; /* prepare a dummy */
		gcp->gc_rx_buf_size  = 64;
		gcp->gc_rx_copy_thresh = rf_rx_copy_thresh;
		gcp->gc_rx_buf_max  = gcp->gc_rx_buf_size + 1;

		/* map attributes (endianess) */
		STRUCT_COPY(gcp->gc_dev_attr, rf_dev_attr); /* for registers */
		STRUCT_COPY(gcp->gc_buf_attr, rf_buf_attr); /* for tx */
		STRUCT_COPY(gcp->gc_desc_attr, rf_buf_attr);/* for rx buffers */

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, rf_dma_attr_rxbuf);
		STRUCT_COPY(gcp->gc_dma_attr_txbuf, rf_dma_attr_txbuf);
		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, rf_dma_attr_rxbuf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align + 1;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout    = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = (25*ONESEC)/10;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		gcp->gc_mii_an_delay   = ONESEC/10;
		gcp->gc_mii_linkdown_action         = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &rf_attach_chip;
		gcp->gc_reset_chip  = &rf_reset_chip;
		gcp->gc_init_chip   = &rf_init_chip;
		gcp->gc_start_chip  = &rf_start_chip;
		gcp->gc_stop_chip   = &rf_stop_chip;
		gcp->gc_multicast_hash = &rf_mcast_hash;
		gcp->gc_set_rx_filter = &rf_set_rx_filter;
		gcp->gc_set_media   = &rf_set_media;
		gcp->gc_get_stats   = &rf_get_stats;
		gcp->gc_interrupt   = &rf_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &rf_tx_desc_write;
		gcp->gc_rx_desc_write = &rf_rx_desc_write;
		gcp->gc_tx_desc_stat  = &rf_tx_desc_stat;
		gcp->gc_rx_desc_stat  = &rf_rx_desc_stat;
		gcp->gc_tx_desc_init  = &rf_tx_desc_init;
		gcp->gc_rx_desc_init  = &rf_rx_desc_init;
		gcp->gc_tx_desc_clean = &rf_tx_desc_clean;
		gcp->gc_rx_desc_clean = &rf_rx_desc_clean;

		/* mii operations */
		gcp->gc_mii_init   = &gem_mii_init_default;
		gcp->gc_mii_config = &gem_mii_config_default;
		if (p->type == CHIP_8129) {
			/* rtl8129 */
			gcp->gc_mii_sync  = &rf_mii_sync_external;
			gcp->gc_mii_read  = &rf_mii_read_external;
			gcp->gc_mii_write = &rf_mii_write_external;
		}
		else {
			/* rtl8139 */
			gcp->gc_mii_sync  = &rf_mii_sync_internal;
			gcp->gc_mii_read  = &rf_mii_read_internal;
			gcp->gc_mii_write = &rf_mii_write_internal;
		}
		gcp->gc_mii_tune_phy = NULL;
		gcp->gc_get_packet = &rf_get_packet;

		lp = (struct rf_dev *)
				kmem_zalloc(sizeof(struct rf_dev), KM_SLEEP);
		lp->chip = p;

		dp = gem_do_attach(dip, gcp, base, &regs_handle,
					lp, sizeof(struct rf_dev));

		kmem_free(gcp, sizeof(*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof(struct rf_dev));
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
static	struct module_info rfminfo = {
	0,			/* mi_idnum */
	"rf",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
#ifdef notdef
	INT32_MAX,		/* mi_hiwat */
#else
	NTXBUF*ETHERMAX,	/* mi_hiwat */
#endif
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
	D_NEW|D_MP	/* cb_flag */
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
	ddi_power	/* devo_power */
};

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

	DPRINTF(2, (CE_CONT, "!rf: _init: called"));
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

	DPRINTF(2, (CE_CONT, "!rf: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
