/*
 * vfe.c: VIA Technology RHINE seris Fast Ethernet MAC driver
 *
 * Copyright (c) 2002-2008 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)vfe_gem.c 1.11     05/01/01"

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

#include "rhinereg.h"

char	ident[] = "via rhine nic driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int vfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (vfe_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)

#ifdef MAP_MEM
#define	FLSHL(dp, r)	(void)INL(dp, r)
#define	FLSHW(dp, r)	(void)INW(dp, r)
#define	FLSHB(dp, r)	(void)INB(dp, r)
#else
#ifdef __sparc
#define	FLSHL(dp, r)	(void)INL(dp, r)
#define	FLSHW(dp, r)	(void)INW(dp, r)
#define	FLSHB(dp, r)	(void)INB(dp, r)
#else
#define	FLSHL(dp, r)
#define	FLSHW(dp, r)
#define	FLSHB(dp, r)
#endif /* __sparc */
#endif /* MAP_MEM */

#define	VFE_WRITE_IMR(dp, imr)	{	\
	OUTW((dp), IMR, (imr));	\
	if (((struct vfe_dev *)(dp)->private)->revid > 0x40) {	\
		OUTB((dp), MIMR, (imr) >> 16);	\
	}	\
}

#define	VFE_WRITE_ISR(dp, isr)	{	\
	OUTW((dp), ISR, (isr));	\
	if (((struct vfe_dev *)(dp)->private)->revid > 0x40) {	\
		OUTB((dp), MISR, (isr) >> 16);	\
	}	\
}

#define	VFE_READ_ISR(dp, isr)	{	\
	(isr) = INW((dp), ISR);	\
	if (((struct vfe_dev *)(dp)->private)->revid > 0x40) {	\
		(isr) |= INB((dp), MISR) << 16;	\
	}	\
}

#define	VFE_KICK_TX(dp, cr)	{	\
	if (IS_RHINE3M(((struct vfe_dev *)(dp)->private)->revid)) { \
		OUTB((dp), TXQW, 1 << 7);	\
	}	\
	OUTB((dp), CR+1, ((CR_TDMD1 | (cr)) >> 8));	\
}

#define	VFE_TXDESC(p)	((struct tx_desc *)(void *)(p))
#define	VFE_RXDESC(p)	((struct rx_desc *)(void *)(p))

#define	VFE_KICK_RX(dp, cr)	{ \
	OUTB((dp), CR+1, ((CR_RDMD1 | (cr)) >> 8));	\
}

/* workaround for TDWB problem */
#define	TSR_TDWB	TSR_JAB	/* this bit isn't defined for 6102 */

/*
 * Our configuration
 */
#define	OUR_INTR_BITS	\
	(ISR_ABTI | ISR_NORBF | ISR_PKRACE | ISR_OVFI |	\
	ISR_CNT | ISR_BE | ISR_RU | ISR_TU |	\
	ISR_RXE | ISR_TXE | ISR_PTX | ISR_PRX)

#ifdef GEM_CONFIG_TX_DIRECT
#define	MAXTXFRAGS	(min(8, GEM_MAXTXFRAGS))
#else
#define	MAXTXFRAGS	1
#endif

#ifdef TEST_RX_EMPTY
#define	RX_BUF_SIZE	1
#endif
#ifdef TEST_TXDESC_FULL
#define	TX_BUF_SIZE	4
#define	TX_RING_SIZE	8
#endif

#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif

#ifndef TX_RING_SIZE
#if MAXTXFRAGS == 1
#define	TX_RING_SIZE	TX_BUF_SIZE
#else
#define	TX_RING_SIZE	(TX_BUF_SIZE*4)
#endif
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

static int	vfe_tx_copy_thresh = 256;
static int	vfe_rx_copy_thresh = 256;

/*
 * rhine chip state
 */
struct vfe_dev {
	uint8_t		mac_addr[ETHERADDRL];
	uint16_t	cr;	/* soft copy of command reg. */
	uint8_t		revid;	/* chip revision id */
	uint8_t		tft;	/* tx fifo threshold */
	uint8_t		rft;	/* rx fifo threshold */
	uint8_t		maxdma;
	uint32_t	imr;
#ifdef WA_SPURIOUS_INTR
	uint32_t	isr_pended;
#endif
	int		last_phy_addr;
#ifdef RESET_TEST
	int		reset_test;
#endif
#ifdef TEST_TDWB
	int		tdwb_cnt;
#endif
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
};

static struct chip_info vfe_chiptbl[] = {
	{VID_VIA, DID_VT3043, "DL10030/VT86C100A Rhine"},
	{VID_VIA, DID_VT6102, "VT6102 Rhine-II"},
	{VID_VIA, DID_VT6105, "VT6105/VT6107 Rhine-III"},
	{VID_VIA, DID_VT6105M, "VT6105M Rhine-III"},
};
#define	CHIPTABLESIZE   (sizeof (vfe_chiptbl)/sizeof (struct chip_info))

/*
 * Macros to identify chip generation.
 */
#define	IS_RHINE1_VT86C100A_E(r)	((r) <= 0x4)
#define	IS_RHINE1_VT86C100A(r)		((r) < 0x20)
#define	IS_RHINE1(r)			((r) < 0x40)
#define	IS_RHINE2(r)			(0x40 <= (r) && (r) < 0x80)
#define	IS_RHINE2_OR_LATER(r)		(0x40 <= (r))
#define	IS_RHINE3(r)			(0x80 <= (r) && (r) < 0x90)
#define	IS_RHINE3_OR_LATER(r)		(0x80 <= (r))
#define	IS_RHINE3LOM(r)			(0x8a <= (r) && (r) < 0x90)
#define	IS_RHINE3M(r)			(0x90 <= (r))
#define	IS_VT6107A1(r)			(0x8d <= (r) && (r) < 0x90)

#define	IS_VT6103(phyid)	\
	(((phyid) & 0xfffffff0) == 0x01018f20 && ((phyid) & 0xf) > 4)
/* ======================================================== */

/* mii operations */
static void  vfe_mii_sync(struct gem_dev *);
static uint16_t  vfe_mii_read(struct gem_dev *, uint_t);
static void vfe_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int vfe_attach_chip(struct gem_dev *);
static int vfe_reset_chip(struct gem_dev *);
static int vfe_init_chip(struct gem_dev *);
static int vfe_start_chip(struct gem_dev *);
static int vfe_stop_chip(struct gem_dev *);
static int vfe_set_media(struct gem_dev *);
static int vfe_set_rx_filter(struct gem_dev *);
static int vfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int vfe_tx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags);
static void vfe_tx_start(struct gem_dev *dp, int startslot, int nslot);
static void vfe_rx_desc_write(struct gem_dev *dp, int slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static void vfe_rx_start(struct gem_dev *dp, int startslot, int nslot);
static uint_t vfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t vfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void vfe_tx_desc_init(struct gem_dev *dp, int slot);
static void vfe_rx_desc_init(struct gem_dev *dp, int slot);
static void vfe_tx_desc_clean(struct gem_dev *dp, int slot);
static void vfe_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t vfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr vfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr vfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t vfe_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x000007ffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003ffc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000007ffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t vfe_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xfffffffc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * HW manipulation routines
 */
/* ======================================================== */
static void
vfe_stop_mauto(struct gem_dev *dp)
{
	int		i;
	struct vfe_dev	*lp = dp->private;

	/*
	 * Initialize MII logic
	 */
	OUTB(dp, MIICR, 0);

	/* wait for midle */
	if (IS_RHINE1_VT86C100A(lp->revid)) {
		/*
		 * turn off MSRCEN
		 */
		OUTB(dp, MIIADR, MII_STATUS);
		drv_usecwait(1000);

		/* turn on MAUTO */
		OUTB(dp, MIICR, MIICR_MAUTO);

		for (i = 0; (INB(dp, MIIADR) & MIIADR_MDONE) == 0; i++) {
			if (i > 100000) {
				break;
			}
			drv_usecwait(10);
		}

		/*
		 * as soon as MDONE is on,
		 * this is the right time to turn off MAUTO
		 */
		OUTB(dp, MIICR, 0);
	} else {
		for (i = 0; (INB(dp, MIIADR) & MIIADR_MIDLE) == 0; i++) {
			if (i > 1000) {
				cmn_err(CE_WARN, "!%s: %s: timeout",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}
		DPRINTF(2, (CE_CONT, "%s: %s: nic became MIDLE in %duS",
		    dp->name, __func__, i*10));
	}
	DPRINTF(2, (CE_CONT, "!%s: mauto stopped in %d uS", __func__, i*10));
}

static void
vfe_start_mauto(struct gem_dev *dp)
{
	int	i;

	OUTB(dp, MIICR, 0);
	OUTB(dp, MIIADR, MII_STATUS);
	OUTB(dp, MIICR, MIICR_MAUTO);

	for (i = 0; (INB(dp, MIIADR) & MIIADR_MDONE) == 0; i++) {
		if (i > 1000) {
			cmn_err(CE_WARN, "%s: %s: timeout",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}

	DPRINTF(2, (CE_CONT, "%s: %s: nic became MAUTO in %duS",
	    dp->name, __func__, i*10));

	OUTB(dp, MIIADR, MIIADR_MSRCEN | MII_STATUS);
}

static int
vfe_reset_chip(struct gem_dev *dp)
{
	int		i;
	int		ret;
#ifdef WA_SPURIOUS_INTR
	uint32_t	isr;
#endif
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_reset_chip: called, time:%ld, cr:%b",
	    dp->name, ddi_get_lbolt(),
	    lp->cr, CR_BITS));

	ret = GEM_SUCCESS;
#if DEBUG_LEVEL > 10
	/* white a dummy mac address to ensure eeprom autoloading works */
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(dp, PAR + i, i);
	}
#endif
	/* invalidate a software cache for current mac address */
	bzero(lp->mac_addr, sizeof (lp->mac_addr));

	/* clear interrupt mask, but don't clear interrupt causes */
	VFE_WRITE_IMR(dp, 0);
#ifdef WA_SPURIOUS_INTR
	/* save interrupt status */
	VFE_READ_ISR(dp, isr);
	lp->isr_pended |= isr;
#endif

	/* reset the chip. */
	lp->cr = 0;
	OUTB(dp, CR + 1, (CR_SRST >> 8) | INB(dp, CR + 1));
	drv_usecwait(10);

	if (IS_RHINE1(lp->revid)) {
		/* extra wait time for Rhine-I chips */
		drv_usecwait(100);
	}

	for (i = 0; INB(dp, CR + 1) & (CR_SRST >> 8); i++) {
		if (i > 2000) {
			cmn_err(CE_CONT, "!%s: %s: timeout",
			    dp->name, __func__);

			if (IS_RHINE1_VT86C100A(lp->revid)) {
				ret = GEM_FAILURE;
				break;
			}

			/* Use force_software_reset bit in MISC register */
			OUTB(dp, MISC + 1,
			    (MISC_FORSRST >> 8) | INB(dp, MISC + 1));
			drv_usecwait(2000);
			break;
		}
		drv_usecwait(10);
	}

	delay(drv_usectohz(5000));

	/* clear interrupt mask again */
	VFE_WRITE_IMR(dp, 0);

	/* initialize cached value for PHY address */
	lp->last_phy_addr = INB(dp, MPHY) & PHYADR_PHYAD;
	/* XXX - write it back */
	OUTB(dp, MPHY, lp->last_phy_addr);

	vfe_stop_mauto(dp);

	return (ret);
}

static int
vfe_init_chip(struct gem_dev *dp)
{
	int		i;
	uint_t		val;
	ddi_acc_handle_t	conf_handle;
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

	if (pci_config_setup(dp->dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: ddi_regs_map_setup failed",
		    dp->name);
		return (GEM_FAILURE);
	}

	/* Undocumented dances and songs for PCI config registers */
	if (IS_RHINE2_OR_LATER(lp->revid)) {
		/* PCEROPT */
		pci_config_put8(conf_handle, 0x52,
		    pci_config_get8(conf_handle, 0x52) | 0x80);
		/* MII ON */
		pci_config_put8(conf_handle, 0x53,
		    pci_config_get8(conf_handle, 0x53) | 0x04);
	}
	if (IS_RHINE1(lp->revid) || IS_RHINE3LOM(lp->revid)) {
		/* enable MODE10T */
		pci_config_put8(conf_handle, 0x52,
		    pci_config_get8(conf_handle, 0x52) | 0x02);
	}
	if (IS_VT6107A1(lp->revid)) {
		/* enable MRM */
		pci_config_put8(conf_handle, 0x52,
		    pci_config_get8(conf_handle, 0x52) | 0x08);
	}
	pci_config_teardown(&conf_handle);

	/* PAR: don't touch */

	/* RCR: disable all rx filter */
	OUTB(dp, RCR, lp->rft << RCR_RRFT_SHIFT);

	/* CFGA */
	if (IS_RHINE2_OR_LATER(lp->revid)) {
		OUTB(dp, CFGA, INB(dp, CFGA) & ~0x01);
	}

	/* TCR */
	OUTB(dp, TCR,
	    (lp->tft << TCR_RTFT_SHIFT) | TCR_LB_NORMAL | TCR_OFSET);

	/* CR: don't touch  */

	/* ISR: don't touch */

	/* IMR & MIMR */
	lp->imr = 0;
	VFE_WRITE_IMR(dp, lp->imr);

	/* Curr Rx Desc Addr */
	/* This also seems to reset Rx Desc Pointer register. */
	OUTL(dp, CRDA, (uint32_t)dp->rx_ring_dma);

	/* Curr Tx Desc Addr */
	/* This also seems to reset Tx Desc Pointer register. */
	OUTL(dp, CTDA, (uint32_t)dp->tx_ring_dma);
	if (IS_RHINE3M(lp->revid)) {
		/* It has 8 transmit queues, but we use only the first one. */
		bzero(dp->io_area, dp->gc.gc_io_area_size);
		for (i = 1; i < 8; i++) {
			OUTL(dp, CTDA + i*4, dp->io_area_dma);
		}
	}

	/* BCR */
	OUTW(dp, BCR,
	    (lp->tft << BCR_CTFT_SHIFT) |
	    (lp->rft << BCR_CRFT_SHIFT) |
	    lp->maxdma);

	/* CFGB */
	val = INB(dp, CFGB);
	if (IS_RHINE1(lp->revid)) {
		/* Disable queuing packets */
		val |= CFGB_QPKTDIS;
	} else {
		/* Enable queuing packets */
		val &= ~CFGB_QPKTDIS;
		OUTB(dp, WOLCG_SET, WOLCG_SAM | WOLCG_SAB);
	}
	OUTB(dp, CFGB, val);

	/* CFGD */
	val = INB(dp, CFGD);
	val &= ~(CFGD_CRANDOM | CFGD_CAP | CFGD_MBA | CFGD_BAKOPT);
	if (IS_RHINE1(lp->revid)) {
		/* Disable MII status change interrupt as it doesn't work */
		val &= ~CFGD_GPIOEN;
	}
	OUTB(dp, CFGD, val);

	if (IS_RHINE3_OR_LATER(lp->revid)) {
		/* setup RhineIII flow control */
		OUTB(dp, FCR1,
		    FCR1_TXLOWAT_24 | FCR1_TXHIWAT_48 | FCR1_XONOFF_EN);
		OUTW(dp, PAUSE_TIMER, 0xffff);
	}

	DPRINTF(1, (CE_CONT, "!%s: txthr:%d rxthr:%d maxdma:%d",
	    dp->name, lp->tft, lp->rft, lp->maxdma));

	return (GEM_SUCCESS);
}

static uint_t
vfe_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is most significant 6 bits of crc in big endian manner */
	return (gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6));
}

static int
vfe_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	int		i, j;
	uint64_t	mhash;
	uint8_t		*mac;
	static uint8_t	invalid_mac[ETHERADDRL] = {0, 0, 0, 0, 0, 0};
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld, "
	    "active:%d, rxmode:%b, cr:%b",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, dp->rxmode, RXMODE_BITS, lp->cr, CR_BITS));

	mode = RCR_SEP | RCR_AR | RCR_AB /* error packets and broadcast */
	    | RCR_AM;			/* enable multicast packet reception */
	mhash = 0ULL;
	mac = &dp->cur_addr.ether_addr_octet[0];

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;	/* disable all filters */
		mac = invalid_mac;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		mode |= RCR_PRO;	/* promiscuous */
		mhash = ~0ULL;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) ||
	    dp->mc_count > MULTICAST_CAM_SIZE) {
		mhash = ~0ULL;
	} else if (!IS_RHINE3M(lp->revid)) {
		/* make a hash table */
		for (i = 0; i < dp->mc_count; i++) {
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	/* update rx filter related registers */
	if (bcmp(lp->mac_addr, mac, ETHERADDRL) != 0) {
		/* update station address */
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, mac[i]);
			lp->mac_addr[i] = mac[i];
		}
	}

	if (IS_RHINE3M(lp->revid)) {
		uint32_t	cammask;
		/*
		 * VT6105M has 32 entry cams for filtering muticast packets.
		 */
		cammask = 0;

		/* enable cam port for writing multicast address */
		OUTB(dp, CAMCTRL, CAMCTRL_EN);

		if (dp->mc_count <= MULTICAST_CAM_SIZE) {
			for (i = 0; i < dp->mc_count; i++) {
				/* write cam index */
				OUTB(dp, CAMADDR, i);

				/* write the multicast address into cam */
				mac = dp->mc_list[i].addr.ether_addr_octet;
				for (j = 0; j < ETHERADDRL; j++) {
					OUTB(dp, MAR + j, mac[j]);
				}
				drv_usecwait(10);

				/* issue a cam write command */
				OUTB(dp, CAMCTRL, CAMCTRL_EN | CAMCTRL_WR);
				drv_usecwait(10);

				cammask |= 1 << i;
			}
		}

		/* setup multicast cam mask */
		OUTL(dp, CAMMASK, cammask);
		OUTB(dp, CAMCTRL, 0);

		/* we don't use VLAN cam */
		OUTB(dp, CAMCTRL, CAMCTRL_EN | CAMCTRL_VLAN);
		OUTL(dp, CAMMASK, 0);
		OUTB(dp, CAMCTRL, 0);
	}

	if (mode & RCR_AM) {
		/* update multicast hash table */
		for (i = 0; i < 2; i++) {
			OUTL(dp, MAR + i*4, (uint32_t)mhash);
			mhash >>= 32;
		}
	}

	/* update rx filter mode */
	OUTB(dp, RCR, (lp->rft << RCR_RRFT_SHIFT) | mode);

	return (GEM_SUCCESS);
}

static int
vfe_set_media(struct gem_dev *dp)
{
	uint_t		val;
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld, "
	    "active:%d, cr:%b, misc:%04x",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, lp->cr, CR_BITS, INW(dp, MISC)));

	/*
	 * Notify current duplex mode to mac
	 */
	/* XXX - don't stop the nic, it isn't recoverable. */
	if (dp->full_duplex) {
		lp->cr |= CR_FDX;
	} else {
		lp->cr &= ~CR_FDX;
	}
	OUTB(dp, CR+1, lp->cr >> 8);

	/* Set up flow control register */
	if (IS_RHINE2(lp->revid)) {
		/* VT6102 has partial flow control */
		val = INW(dp, MISC) &
		    ~(MISC_FDXTFEN | MISC_FDXRFEN | MISC_HDXFEN);
		switch (dp->flow_control) {
		case FLOW_CONTROL_SYMMETRIC:
		case FLOW_CONTROL_RX_PAUSE:
			val |= MISC_FDXRFEN;
			break;
		}
		OUTW(dp, MISC, val);
	} else if (IS_RHINE3_OR_LATER(lp->revid)) {
		val = INB(dp, FCR1) &
		    ~(FCR1_FDFCTX_EN | FCR1_FDFCRX_EN | FCR1_HDFC_EN);

		switch (dp->flow_control) {
		case FLOW_CONTROL_RX_PAUSE:
			val |= FCR1_FDFCRX_EN;
			break;

		case FLOW_CONTROL_TX_PAUSE:
			val |= FCR1_FDFCTX_EN;
			break;

		case FLOW_CONTROL_SYMMETRIC:
			val |= FCR1_FDFCTX_EN | FCR1_FDFCRX_EN;
			break;
		}
		OUTB(dp, FCR1, val);
	}

	return (GEM_SUCCESS);
}

static int
vfe_start_chip(struct gem_dev *dp)
{
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

	/* enable interrupt */
	lp->imr = OUR_INTR_BITS;
#ifdef CONFIG_MAUTO
	if (IS_RHINE2_OR_LATER(lp->revid)) {
		lp->imr |= ISR_SRCI;
	}
#endif
	if (IS_RHINE2(lp->revid) || IS_RHINE3(lp->revid)) {
		lp->imr |= MISR_TDWBI << 16;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		VFE_WRITE_IMR(dp, lp->imr);
	}

	/* Kick TX and RX */
	lp->cr |= CR_TXON | CR_RXON | CR_DPOLL | CR_STRT;
	OUTW(dp, CR, lp->cr);

#ifdef CONFIG_MAUTO
	if (IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_start_mauto(dp);
	}
#endif
	return (GEM_SUCCESS);
}

static int
vfe_stop_chip(struct gem_dev *dp)
{
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

#ifdef CONFIG_MAUTO
	if (IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_stop_mauto(dp);
	}
#endif
	/* Disable interrupts by clearing the interrupt mask */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* XXX - don't clear imr to avoid bogus interrupts */
		VFE_WRITE_IMR(dp, lp->imr);
	}

	/* Stop Tx and Rx processes in the chip. */
	lp->cr &= ~(CR_TXON | CR_RXON | CR_DPOLL | CR_STRT);
	OUTW(dp, CR, lp->cr | CR_STOP);

	/*
	 * XXX -  sometimes rx doesn't restart after stop.
	 */
	(void) vfe_reset_chip(dp);

	return (GEM_SUCCESS);
}

#ifdef DEBUG_LEVEL
static uint16_t
vfe_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int		i;
	uint16_t	ret;
	uint8_t		chip_select;
	uint8_t		di;
	uint8_t		cfga_saved;

#define	VFE_EEPROM_DELAY(dp)	{INB(dp, EECSR); INB(dp, EECSR); }
#define	EE93C46_READ	6

	/* enable eeprom direct programming */
	cfga_saved = INB(dp, CFGA);
	OUTB(dp, CFGA, cfga_saved | CFGA_EELOAD);
	VFE_EEPROM_DELAY(dp);

	/* ensure de-assert chip select */
	chip_select = EECSR_DPM;
	OUTB(dp, EECSR, chip_select);
	VFE_EEPROM_DELAY(dp);

	/* assert chip select */
	chip_select |= EECSR_ECS;
	OUTB(dp, EECSR, chip_select);
	VFE_EEPROM_DELAY(dp);

	/* make a read command for eeprom */
	offset = (offset & 0x3f) | (EE93C46_READ << 6);

	for (i = 10; i >= 0; i--) {
		/* send 1 bit */
		di = ((offset >> i) & 1) << EECSR_EDI_SHIFT;

		OUTB(dp, EECSR, chip_select | di);
		VFE_EEPROM_DELAY(dp);

		OUTB(dp, EECSR, chip_select | di | EECSR_ECK);
		VFE_EEPROM_DELAY(dp);
	}

	OUTB(dp, EECSR, chip_select);
	VFE_EEPROM_DELAY(dp);

	/* get the reply and construct a 16bit value */
	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTB(dp, EECSR, chip_select | EECSR_ECK);
		VFE_EEPROM_DELAY(dp);

		ret = (ret << 1)
		    | ((INB(dp, EECSR) >> EECSR_EDO_SHIFT) & 1);

		OUTB(dp, EECSR, chip_select);
		VFE_EEPROM_DELAY(dp);
	}

	/* negate chip_select */
	OUTB(dp, EECSR, EECSR_DPM);
	VFE_EEPROM_DELAY(dp);

	OUTB(dp, EECSR, 0);
	VFE_EEPROM_DELAY(dp);

	/* disable eeprom direct programming */
	OUTB(dp, CFGA, cfga_saved);
	VFE_EEPROM_DELAY(dp);

	return (ret);
}

static void
vfe_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	uint16_t	prom[0x10];

	for (i = 0; i < 0x10; i++) {
		prom[i] = vfe_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 0x10; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
}
#endif /* DEBUG_LEVEL */

static int
vfe_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint8_t		*mac;
	struct vfe_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

	mac = dp->dev_addr.ether_addr_octet;

	/*
	 * XXX - Don't read a mac address from eeprom, it doesn't
	 * work for built-in rhine II core in VT8235 south bridge.
	 * We can read a factory mac address from PAR register,
	 *  only after the eeprom contents have been auto-loaded.
	 */
	for (i = 0; i < ETHERADDRL; i++) {
		mac[i] = INB(dp, PAR + i);
	}

	if (gem_get_mac_addr_conf(dp)) {
		/* Use a mac address specified in vfe.conf. */
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, mac[i]);
		}
	}

	if (mac[0] & 1) {
		/* it seems the mac address is corrupted, generate new one. */
		cmn_err(CE_NOTE,
		    "!%s: mac address %x:%x:%x:%x:%x:%x is corrupted.",
		    dp->name,
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#ifdef OS_PUTBACK
		return (GEM_FAILURE);
#else
		gem_generate_macaddr(dp, mac);

		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, mac[i]);
		}
#endif
	}

	/* initialize soft copy of mac address registers */
	bcopy(mac, lp->mac_addr, ETHERADDRL);

#if DEBUG_LEVEL > 4
	vfe_eeprom_dump(dp);
#endif /* DEBUG_LEVEL */

	/* read default address of PHY in eeprom */
	dp->mii_phy_addr = INB(dp, MPHY) & PHYADR_PHYAD;

	if (dp->rxthr <= 32) {
		lp->rft = RRFT_32;
	} else if (dp->rxthr <= 64) {
		lp->rft = RRFT_64;
	} else if (dp->rxthr <= 128) {
		lp->rft = RRFT_128;
	} else if (dp->rxthr <= 256) {
		lp->rft = RRFT_256;
	} else if (dp->rxthr <= 512) {
		lp->rft = RRFT_512;
	} else if (dp->rxthr <= 768) {
		lp->rft = RRFT_768;
	} else if (dp->rxthr <= 1024) {
		lp->rft = RRFT_1024;
	} else {
		lp->rft = RRFT_SF;
	}

	/* setup initial tft */
#ifdef notdef
	/* XXX - not required, now txthr is ETHERMAX */
	/* XXX - smaller tx threshold will cause performance issue. */
	if (IS_RHINE1(lp->revid)) {
		dp->txthr = max(dp->txthr, ETHERMAX); /* vfe-1.0.0 default */
	} else if (IS_RHINE2(lp->revid)) {
		dp->txthr = max(dp->txthr, 512);
	}
#endif
	if (dp->txthr <= 128) {
		lp->tft = RTFT_128;
	} else if (dp->txthr <= 256) {
		lp->tft = RTFT_256;
	} else if (dp->txthr <= 512) {
		lp->tft = RTFT_512;
	} else if (dp->txthr <= 1024) {
		lp->tft = RTFT_1024;
	} else {
		lp->tft = RTFT_SF;
	}

	/* XXX - smaller txmaxdma will cause performance issue. */
	dp->txmaxdma = dp->mtu;

	if (dp->txmaxdma < 32) {
		lp->maxdma = BCR_DMA_32;
	} else if (dp->txmaxdma < 64) {
		lp->maxdma = BCR_DMA_64;
	} else if (dp->txmaxdma < 128) {
		lp->maxdma = BCR_DMA_128;
	} else if (dp->txmaxdma < 256) {
		lp->maxdma = BCR_DMA_256;
	} else if (dp->txmaxdma < 512) {
		lp->maxdma = BCR_DMA_512;
	} else if (dp->txmaxdma < 1024) {
		lp->maxdma = BCR_DMA_1024;
	} else {
		lp->maxdma = BCR_DMA_NOLIMIT;
	}
#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif

#if DEBUG_LEVEL > 10
	/* test for timer function */
{
	clock_t		tim;
	int		i;

#define	TEST_COUNT	300
	tim = ddi_get_lbolt();

	/* OUTB(dp, GMCR, INB(dp, GMCR) | GMCR_TMR1US); */

	OUTW(dp, SOFT_TMR0, TEST_COUNT);
	/* OUTB(dp, MIMR, INB(dp, MIMR) | MIMR_TIM0IM); */
	OUTW(dp, MISC, INW(dp, MISC) | MISC_Tm0EN);
	for (i = 100000; (INB(dp, MISR) & MISR_TIM0I) == 0; i--) {
		if (i == 0) {
			cmn_err(CE_CONT, "!%s: soft_timer0 timeout", dp->name);
		}
		drv_usecwait(100);
	}
	tim = ddi_get_lbolt() - tim;

	if (i > 0) {
		cmn_err(CE_CONT, "!%s: soft_timer0 resolution in 10mS: %d",
		    dp->name, tim > 0 ? TEST_COUNT/tim : 9999999);
	}
	/* OUTL(dp, CR_CLR, CR_TM0EN); */

}
#endif
	return (GEM_SUCCESS);
}

static int
vfe_get_stats(struct gem_dev *dp)
{
	volatile int	x;
	struct vfe_dev	*lp = dp->private;

	if (IS_RHINE2_OR_LATER(lp->revid)) {
		dp->stats.missed += INW(dp, MPAC);
	} else {
		/* MPAC is unreliable on rhine I */
		x = INW(dp, MPAC);
	}
	x = INW(dp, CRCC);
	OUTW(dp, MPAC, 0);
#ifdef lint
	x = x;
#endif
	return (GEM_SUCCESS);
}

/*
 * tx/rx descriptor manipurations
 */
#ifdef DEBUG_LEVEL
static int vfe_tx_frags[MAXTXFRAGS];
#endif
static int
vfe_tx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	struct tx_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	uint32_t		tmp;
#ifdef GEM_CONFIG_TX_DIRECT
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;
#endif
	struct vfe_dev		*lp = dp->private;

#ifdef lint
	i = 0;
	i = i;
#endif
#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags: %llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 2
	if (flags & GEM_TXFLAG_INTR) {
		cmn_err(CE_CONT, "!%s: %s time: %d,"
		    " seqnum: %d, slot %d, frags: %d flags: %llx",
		    dp->name, __func__, ddi_get_lbolt(),
		    dp->tx_desc_tail, slot, frags, flags);
	}
#endif
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
#ifdef DEBUG_LEVEL
	vfe_tx_frags[min(frags, MAXTXFRAGS) - 1]++;
#endif
	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TDES1_EDP | TDES1_C;
	dcp = &dmacookie[frags - 1];
#ifdef GEM_CONFIG_TX_DIRECT
	for (i = frags - 1; i > 0; i--, dcp--) {

		tdp = &VFE_TXDESC(dp->tx_ring)[SLOT(slot + i, tx_ring_size)];
		mark |= dcp->dmac_size;
		tdp->td_length = LE_32(mark);
		tmp = dcp->dmac_address;
		tdp->td_addr = LE_32(tmp);
		tdp->td_csr = 0;

		/*
		 * XXX - The datasheets says that the flags for intermediate
		 * fragments are STP=1 and EDP=0, but it didn't work.
		 * The right flags are STP=0 and EDP=0 in my experience.
		 */
		mark = TDES1_C;
	}
#else
	ASSERT(frags == 1);
#endif

	/* for the first fragment */
	tdp = &VFE_TXDESC(dp->tx_ring)[slot];
	mark |= dcp->dmac_size | TDES1_STP;
	tdp->td_length = LE_32(mark);
	tmp = dcp->dmac_address;
	tdp->td_addr = LE_32(tmp);

	if ((flags & GEM_TXFLAG_INTR) == 0 && IS_RHINE2_OR_LATER(lp->revid)) {
		/* set suppress-interrupt request for the packet */
		tdp->td_next |= LE_32(TDES3_TDCTL);
	}

	if ((flags & GEM_TXFLAG_HEAD) == 0) {
		tdp->td_csr = LE_32(TDES0_OWN);
	}

	return (frags);
}

static void
vfe_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct vfe_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	VFE_TXDESC(dp->tx_ring)[start_slot].td_csr = LE_32(TDES0_OWN);
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/* kick Tx engine */
#ifdef TEST_TXTIMEOUT
	if ((vfe_send_cnt++ % 100) == 99) {
		lp->cr &= ~CR_TXON;
		OUTW(dp, CR, lp->cr);
	}
#endif
	VFE_KICK_TX(dp, lp->cr);
}

static void
vfe_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t	tmp0;
	uint32_t	tmp1;
	struct rx_desc	*rdp;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT,
	    "!%s: vfe_rx_desc_write seqnum: %d, slot %d, frags: %d",
	    dp->name, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
	/*
	 * write a RX descriptor
	 */
	rdp = &VFE_RXDESC(dp->rx_ring)[slot];

	tmp0 = dmacookie->dmac_address;
	rdp->rd_addr = LE_32(tmp0);
	tmp1 = dmacookie->dmac_size;
	rdp->rd_length = LE_32(tmp1);
	rdp->rd_status = LE_32(RDES0_OWN);
}

static void
vfe_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	struct vfe_dev	*lp = dp->private;

	gem_rx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	if (IS_RHINE3_OR_LATER(lp->revid)) {
		OUTB(dp, FCR0, nslot);
	}
}

#ifdef DEBUG_LEVEL
static void
vfe_tx_desc_dump(struct gem_dev *dp, seqnum_t head, int ndesc)
{
	int		i;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct tx_desc	*tdp;

	cmn_err(CE_CONT, "!%s: %s:  slot:%d, seqnum:%d",
	    dp->name, __func__, SLOT(head, tx_ring_size), head);

	for (i = 0; i < ndesc; i++) {
		tdp = &VFE_TXDESC(dp->tx_ring)[SLOT(head + i, tx_ring_size)];

		(void) ddi_dma_sync(dp->desc_dma_handle,
		    (off_t)(((caddr_t)tdp) - (intptr_t)dp->rx_ring),
		    sizeof (struct tx_desc), DDI_DMA_SYNC_FORKERNEL);

		cmn_err(CE_CONT, "! %d: %b, %b, 0x%08x",
		    i, LE_32(tdp->td_csr), TSR_BITS,
		    LE_32(tdp->td_length), TDES1_BITS,
		    LE_32(tdp->td_addr));
	}
}
#endif /* DEBUG LEVEL */
static uint_t
vfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_desc	*tdp;
	uint32_t	csr;
	uint_t		ret;
	int		cols;
	int		i;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct vfe_dev	*lp = dp->private;

	/*
	 * check the first descriptor of the packet
	 */
	tdp = &VFE_TXDESC(dp->tx_ring)[slot];

	csr = tdp->td_csr;
	csr = LE_32(csr);

#if DEBUG_LEVEL > 2
	if (ndesc > 0) {
		int		i;
		uint32_t	isr;

		VFE_READ_ISR(dp, isr);
		cmn_err(CE_CONT, "!%s: %s: time: %d "
		    "slot:%d ndesc:%d isr:%b misr:%b",
		    dp->name, __func__, ddi_get_lbolt(), slot, ndesc,
		    isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS);
		for (i = 0; i < ndesc; i++) {
			struct tx_desc	*tp;

			tp = &VFE_TXDESC(dp->tx_ring)[
			    SLOT(slot + i, tx_ring_size)];
			cmn_err(CE_CONT, "!%d: tsr: %b txdesc1: %b",
			    i,
			    LE_32(tp->td_csr), TSR_BITS,
			    LE_32(tp->td_length), TDES1_BITS);
		}
	}
#endif
	if (csr & TDES0_OWN) {
		/* not transmitted yet */
		return (0);
	}

	ret = GEM_TX_DONE;

#define	TSR_ERRS	\
	(TSR_SERR | TSR_TBUFF | TSR_UDF | TSR_CRS | \
	TSR_OWC | TSR_ABT | TSR_CDH | TSR_TDWB)

	if ((csr & TSR_TXERR) == 0) {
		if (!dp->full_duplex && (cols = (csr & TSR_NCR)) > 0) {
			dp->stats.collisions += cols;
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* if (cols > 1) */ {
				dp->stats.multi_coll++;
			}
		}
		/* short cut for normal case */
		goto x;
	}

	DPRINTF(2, (CE_CONT, "!%s: tx error: tsr: %b  desc:%d",
	    dp->name, csr, TSR_BITS, dp->tx_desc_head));

	dp->stats.errxmt++;

	if (csr & TSR_CRS) {
		dp->stats.nocarrier++;
	} else if (csr & TSR_OWC) {
		dp->stats.xmtlatecoll++;
	} else if (csr & TSR_UDF) {
		dp->stats.underflow++;

		lp->tft = min(lp->tft+1, RTFT_SF);

		OUTB(dp, TCR,
		    (lp->tft << TCR_RTFT_SHIFT) |
		    TCR_LB_NORMAL | TCR_OFSET);

		OUTW(dp, BCR,
		    (lp->tft << BCR_CTFT_SHIFT) |
		    (lp->rft << BCR_CRFT_SHIFT) |
		    lp->maxdma);

		/* transmitter halted, resend the packet */
		cmn_err(CE_CONT, "!%s: transmitter halted, resend the packet",
		    dp->name);
		ndesc = 0;
		ret |= GEM_TX_ERR;
	} else if (csr & TSR_ABT) {
		if (!dp->full_duplex) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		} else {
			dp->stats.xmit_internal_err++;
		}
#if DEBUG_LEVEL > 10
		vfe_tx_desc_dump(dp, dp->tx_desc_head, ndesc);
#endif
		/* transmitter halted, drop the packet */
		ret |= GEM_TX_ERR;
	} else {
		/* unknown error */
		dp->stats.xmit_internal_err++;
	}

	if ((ret & GEM_TX_ERR) == 0) {
		/* no need to recover from fatal error */
		goto x;
	}

	/*
	 * Try to recover the transmitter state without resetting the chip.
	 */
	/* wait until the transmitter becomes idle */
	for (i = 0; INW(dp, CR) & CR_TXON; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "!%s: %s: timeout: stopping tx",
			    dp->name, __func__);
			goto x;
		}
		drv_usecwait(10);
	}
	DPRINTF(2, (CE_CONT, "!%s: tx stopped in %d uS", dp->name, i*10));

	/* fix Current Tx Desctriptor Address register */
	OUTL(dp, CTDA, ((uint32_t)dp->tx_ring_dma) +
	    SLOT(slot + ndesc, tx_ring_size) * sizeof (struct tx_desc));

	/* fix the next action to do. */
	ret = (ndesc > 0) ? GEM_TX_DONE : 0;

	/* enable the transmitter again. */
	OUTW(dp, CR, lp->cr);

	/* kick tx polling */
	VFE_KICK_TX(dp, lp->cr);
x:
	/* reset suppress-intr bit */
	if (ret) {
		/*
		 * As tx was finished, update the interrupt mask.
		 */
		if (!IS_RHINE1(lp->revid)) {
			/*
			 * For Rhine II or later:
			 * Ensure that there is no suppress interrupt request
			 * in TD3.
			 */
			tdp->td_next &= LE_32(TDES3_NEXT);
		}
	}
	return (ret);
}

static uint64_t
vfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_desc		*rdp;
	uint32_t		rsr;
	uint_t			ret;
	uint_t			len;

	rdp = &VFE_RXDESC(dp->rx_ring)[slot];

	rsr = rdp->rd_status;
	rsr = LE_32(rsr);
	len = (rsr >> RDES0_LEN_SHIFT) & RDES0_LEN_MASK;
	ret = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d dma_addr:0x%x len:%d rsr 0x%b",
	    dp->name, __func__, slot,
	    LE_32(rdp->rd_addr),
	    LE_32(rdp->rd_length),
	    LE_32(rdp->rd_status), RSR_BITS));

	if (rsr & RDES0_OWN) {
		/* packets aren't received yet. */
		return (0);
	}

	if ((rsr & (RSR_STP | RSR_EDP)) != (RSR_STP | RSR_EDP)) {
		/* big packet, ignore this fragment */
		if (rsr & RSR_STP) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
		}
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	if ((rsr & RSR_RXOK) == 0) {
		/* errored packet */
		dp->stats.errrcv++;
		if (rsr & RSR_LONG) {
			dp->stats.frame_too_long++;
		} else if (rsr & RSR_RUNT) {
			dp->stats.runt++;
			DPRINTF(2, (CE_CONT,
			    "!%s: %s: slot:%d dma_addr:0x%x"
			    " len:%d rsr 0x%b",
			    dp->name, __func__, slot,
			    LE_32(rdp->rd_addr),
			    LE_32(rdp->rd_length),
			    LE_32(rdp->rd_status), RSR_BITS));
		} else if (rsr & (RSR_SERR | RSR_BUFF)) {
			dp->stats.overflow++;
		} else if (rsr & RSR_FAE) {
			dp->stats.frame++;
		} else if (rsr & RSR_CRC) {
			dp->stats.crc++;
		} else {
			dp->stats.rcv_internal_err++;
		}

		ret |= GEM_RX_ERR;
	}

	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}

	return (ret | (len & GEM_RX_LEN));
}

static void
vfe_tx_desc_init(struct gem_dev *dp, int slot)
{
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	uint32_t	tmp;
	struct tx_desc	*tdp;

	tdp = &VFE_TXDESC(dp->tx_ring)[slot];

	/* invalidate this descriptor */
	tdp->td_csr = 0;

	/* link to previous descriptor */
	tdp = &VFE_TXDESC(dp->tx_ring)[SLOT(slot - 1, tx_ring_size)];
	tmp = dp->tx_ring_dma + slot*sizeof (struct tx_desc);
	tdp->td_next = LE_32(tmp);
}

static void
vfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	uint32_t	tmp;
	struct rx_desc	*rdp;

	rdp = &VFE_RXDESC(dp->rx_ring)[slot];

	/* invalidate this descriptor */
	rdp->rd_status = 0;

	/* link this to its previous descriptor */
	rdp = &VFE_RXDESC(dp->rx_ring)[SLOT(slot - 1, dp->gc.gc_rx_ring_size)];
	tmp = dp->rx_ring_dma + slot*sizeof (struct rx_desc);
	rdp->rd_next = LE_32(tmp);
}

static void
vfe_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct tx_desc		*tdp;

	tdp = &VFE_TXDESC(dp->tx_ring)[slot];

	/* invalidate this descriptor */
	tdp->td_csr = 0;
	tdp->td_next &= LE_32(TDES3_NEXT);
}

static void
vfe_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct rx_desc		*rdp;

	rdp = &VFE_RXDESC(dp->rx_ring)[slot];

	/* invalidate this descriptor */
	rdp->rd_status = 0;
}

/*
 * Device depend interrupt handler
 */
static uint_t
vfe_interrupt(struct gem_dev *dp)
{
	seqnum_t	s;
	uint32_t	isr;
#ifdef WA_SPURIOUS_INTR
	uint32_t	bogus_isr;
#endif
	uint_t		flag = 0;
	boolean_t	need_to_reset = B_FALSE;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct vfe_dev	*lp = dp->private;

	VFE_READ_ISR(dp, isr);

#ifdef WA_SPURIOUS_INTR
	bogus_isr = lp->isr_pended;
	lp->isr_pended = 0;
	if (((isr | bogus_isr) & lp->imr) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}
#else
	if ((isr & lp->imr) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}
#endif

	DPRINTF(2, (CE_CONT,
	    "!%s: Interrupt, time:%ld isr: %b, misr: %b",
	    dp->name, ddi_get_lbolt(),
	    isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS));

	if (!dp->mac_active) {
		/* the device is not active, no more interrupts */
		lp->imr = 0;

		/* disable interrupt */
		VFE_WRITE_IMR(dp, 0);

		VFE_WRITE_ISR(dp, isr);

		return (DDI_INTR_CLAIMED);
	}
#ifdef TEST_RESET_ON_ERRROR
	lp->reset_test++;
	if ((lp->reset_test % 10000) == 9999) {
		need_to_reset = B_TRUE;
	}
#endif

	/* clear interrupts */
	VFE_WRITE_ISR(dp, isr);

	/* barrier to commit interrupt status */
	FLSHW(dp, ISR);

	isr &= lp->imr;

	if (isr & ISR_CNT) {
		/* statics counter overflow */
		(void) vfe_get_stats(dp);
	}

	if (isr & (ISR_PRX | ISR_RXE | ISR_PKRACE | ISR_RU | ISR_NORBF)) {
		/* A packet was received, or receive error happened */
		/* RU: receive buffer unavailable */
		(void) gem_receive(dp);

		if (isr & (ISR_RU | ISR_NORBF)) {
			dp->stats.errrcv++;
			dp->stats.norcvbuf++;

			DPRINTF(2, (CE_CONT,
			    "!%s: rx buffer ran out: isr:%b misr:%b cr:%b",
			    dp->name, isr & 0xffff, ISR_BITS,
			    (isr >> 16), MISR_BITS, lp->cr, CR_BITS));
		}
	}

	if (isr & ISR_OVFI) {
		/*
		 * receive FIFO overflow
		 */
		dp->stats.overflow++;
		goto x;
	}

	if (isr & ISR_BE) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b misr:%b",
		    dp->name, isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS);
		need_to_reset = B_TRUE;
		goto x;
	}
#ifdef TEST_TDWB
	if ((++(lp->tdwb_cnt) % 10000) == 0) {
		isr |= MISR_TDWBI << 16;
		OUTW(dp, CR, lp->cr & ~CR_TXON);
	}
#endif
	if (isr & (MISR_TDWBI << 16)) {
		int	cnt;

		DPRINTF(0, (CE_WARN, "!%s: TDWBI interrupt: isr:%b misr:%b",
		    dp->name, isr & 0xffff, ISR_BITS,
		    isr >> 16, MISR_BITS));

		mutex_enter(&dp->xmitlock);

		/* wait a while until tx side stops */
		for (cnt = 0; (INW(dp, CR) & CR_TXON); cnt++) {
			if (cnt > 10) {
				/* timeout: tx side is still active */
				need_to_reset = B_TRUE;
				mutex_exit(&dp->xmitlock);
				goto x;
			}
			drv_usecwait(10);
		}

		DPRINTF(2, (CE_CONT, "!%s: tx stopped on TDWBI in %d uS",
		    dp->name, cnt*10));

		/* force to reset OWN bit in all current tx descriptors. */
		for (s = dp->tx_desc_head; s != dp->tx_desc_tail; s++) {
			/*
			 * XXX - we use non-fatal TSR_TDWB (JAB) error bit
			 * to show the error in statistics.
			 */
			VFE_TXDESC(dp->tx_ring)[
			    SLOT(s, tx_ring_size)].td_csr = LE_32(TSR_TDWB);
		}
		gem_tx_desc_dma_sync(dp,
		    SLOT(dp->tx_desc_head, tx_ring_size),
		    dp->tx_desc_tail - dp->tx_desc_head,
		    DDI_DMA_SYNC_FORDEV);

		/*
		 * fix Current Tx Desctriptor to point the tail of the tx list.
		 */
		OUTL(dp, CTDA,
		    ((uint32_t)dp->tx_ring_dma) +
		    SLOT(dp->tx_desc_tail, tx_ring_size)
		    * sizeof (struct tx_desc));

		/* force to call gem_tx_done() later. */
		isr |= ISR_TXE;

		/* restart Tx side again. */
		OUTW(dp, CR, lp->cr);
		VFE_KICK_TX(dp, lp->cr);

		/* we have done all dirty things. */
		mutex_exit(&dp->xmitlock);
	}

	if (isr & (ISR_PTX | ISR_TXE | ISR_TU | ISR_ABTI)) {
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

#ifdef CONFIG_MAUTO
	if (isr & ISR_SRCI) {
		/*
		 * PHY port status changed.
		 */
		DPRINTF(0, (CE_CONT,
		    "!%s: port status changed: isr:0x%b misr:0x%b",
		    dp->name,
		    isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS));
		if (gem_mii_link_check(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}
#endif
	if (isr & (ISR_TU | ISR_ETI | ISR_KEYI)) {
		/*
		 * unexpected interrupt
		 */
		/* BE: PCI bus error */
		/* TU: Transmit buffer underflow */
		/* ETI: Transmit descriptor underflow */
		/* KEYI : magic packet received / gpio  */

		cmn_err(CE_WARN,
		    "!%s: unexpected interrupt: isr:%b misr:%b",
		    dp->name,
		    isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS);

		need_to_reset = B_TRUE;
	}
x:
	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flag |= INTR_RESTART_TX;
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routines
 */

/* vendor specific registers in VT6103 based PHY */
#define	MII_PHYCFG1	0x10
#define	MII_PHYCFG1_PHYADDR	0xf800u
#define	MII_PHYCFG1_PHYADDR_SHIFT	11
#define	MII_PHYCFG1_FIBRE	0x0400u
#define	MII_PHYCFG1_SIP		0x0200u
#define	MII_PHYCFG1_FORCE_LINK	0x0100u
#define	MII_PHYCFG1_SQUELCH	0x0080u
#define	MII_PHYCFG1_LED		0x0060u
#define	MII_PHYCFG1_REPEATER	0x0010u
#define	MII_PHYCFG1_PHYINT	0x0008u
#define	MII_PHYCFG1_SYMBOL	0x0004u

static void
vfe_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
vfe_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint16_t	val;
	struct vfe_dev	*lp = dp->private;

	DPRINTF(3, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

#ifdef CONFIG_MAUTO
	if (dp->mac_active && IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_stop_mauto(dp);
	}
#endif
	if (lp->last_phy_addr != dp->mii_phy_addr) {
		OUTB(dp, MPHY, dp->mii_phy_addr);
		lp->last_phy_addr = dp->mii_phy_addr;
	}
	OUTB(dp, MIIADR, reg);
	FLSHB(dp, MIIADR);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_RCMD);

	for (i = 0; INB(dp, MIICR) & MIICR_RCMD; i++) {
		if (i > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout", dp->name, __func__);
			val = 0;
			goto x;
		}
		drv_usecwait(10);
	}

	val = INW(dp, MIIDATA);

x:
#ifdef CONFIG_MAUTO
	if (dp->mac_active && IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_start_mauto(dp);
	}
#endif
	return (val);
}

static void
vfe_mii_write_raw(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;
	struct vfe_dev	*lp = dp->private;

#ifdef CONFIG_MAUTO
	if (dp->mac_active && IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_stop_mauto(dp);
	}
#endif
	if (lp->last_phy_addr != dp->mii_phy_addr) {
		OUTB(dp, MPHY, dp->mii_phy_addr);
		lp->last_phy_addr = dp->mii_phy_addr;
	}
	OUTB(dp, MIIADR, reg);
	FLSHB(dp, MIIADR);
	OUTW(dp, MIIDATA, val);
	FLSHW(dp, MIIDATA);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_WCMD);
	for (i = 0; INB(dp, MIICR) & MIICR_WCMD; i++) {
		if (i > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout", dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}
#ifdef CONFIG_MAUTO
	if (dp->mac_active && IS_RHINE2_OR_LATER(lp->revid)) {
		vfe_start_mauto(dp);
	}
#endif
}

static void
vfe_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint16_t	ret;
	struct vfe_dev	*lp = dp->private;

	if (reg == MII_CONTROL &&
	    (IS_RHINE3_OR_LATER(lp->revid) ||
	    (IS_RHINE2_OR_LATER(lp->revid) && IS_VT6103(dp->mii_phy_id)))) {

		/* fix MII_PHYCFG1 register in VT6103 based PHY */

		ret = vfe_mii_read(dp, MII_PHYCFG1);
		if (val & MII_CONTROL_ANE) {
			ret &= ~1;
		} else {
			ret |= 1;
		}
		vfe_mii_write_raw(dp, MII_PHYCFG1, ret);
	}

	vfe_mii_write_raw(dp, reg, val);
}

static int
vfe_mii_probe(struct gem_dev *dp)
{
	int		ret;
	uint16_t	adv;
	uint16_t	adv_org;
	struct vfe_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: MIISR:%x",
	    dp->name, __func__, INB(dp, MIISR)));

	if ((ret = gem_mii_probe_default(dp)) != GEM_SUCCESS) {
		return (ret);
	}

	/* check if the phy can advertise pause abilities */
	adv_org = vfe_mii_read(dp, MII_AN_ADVERT);
	vfe_mii_write(dp,
	    MII_AN_ADVERT,
	    MII_ABILITY_PAUSE | MII_ABILITY_ASM_DIR);
	adv = vfe_mii_read(dp, MII_AN_ADVERT);
	if ((adv & MII_ABILITY_PAUSE) == 0) {
		dp->gc.gc_flow_control &= ~1;
	}
	if ((adv & MII_ABILITY_ASM_DIR) == 0) {
		dp->gc.gc_flow_control &= ~2;
	}
	vfe_mii_write(dp, MII_AN_ADVERT, adv_org);

	/*
	 * Fibre mode support for VT6105M and VT6103.
	 * The default bits in mii status register are still
	 * normal mii mode even if LED port was strapped for fibre mode.
	 * Therefore, we need to check fibre mode bit in PHY config1
	 * register by ourselves.
	 */
	if (IS_RHINE3M(lp->revid) || IS_VT6103(dp->mii_phy_id)) {
		/* check fibre mode */
		if (vfe_mii_read(dp, MII_PHYCFG1) & MII_PHYCFG1_FIBRE) {

			/* fix capabilities in status register */

			dp->mii_status &=
			    ~(MII_STATUS_10_FD | MII_STATUS_10 |
			    MII_STATUS_CANAUTONEG);
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
vfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
#ifdef notdef
	int			n;
	ddi_iblock_cookie_t	c;
#endif
	ddi_acc_handle_t	conf_handle;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			revid;
	int			unit;
	struct chip_info	*p;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct vfe_dev		*lp;
	uint8_t			*base;
	ddi_acc_handle_t	reg_ha;
	struct gem_conf		*gcp;
	uint32_t		ilr;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(1, (CE_CONT, "!%s%d: %s: called at time:%ld",
	    drv_name, unit, __func__, ddi_get_lbolt()));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

	/* ensure we can access the registers through IO space. */
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
#ifdef lint
	ilr = ilr;
#endif
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);
	pci_config_teardown(&conf_handle);

	/*
	 * Misc setup for rhine registers
	 */
	if (gem_pci_regs_map_setup(dip, PCI_ADDR_IO, PCI_ADDR_MASK,
	    &vfe_dev_attr, (void *)&base, &reg_ha) != DDI_SUCCESS) {
		goto err;
	}

	/*
	 * we use eeprom autoload function to initialize registers including
	 * mac address
	 */
#ifdef DEBUG_LEVEL
	/* write a dummy address to ensure eeprom autoloading works. */
	for (i = 0; i < ETHERADDRL; i++) {
		ddi_put8(reg_ha, base + PAR + i, i);
	}
#endif
	ddi_put8(reg_ha, base + EECSR,
	    EECSR_AUTOLD | ddi_get8(reg_ha, base + EECSR));

	/*
	 * XXX - Don't access rhine nics for 10mS after autoloading.
	 * Rhine doesn't respond to any PCI transactions
	 * while it is loading eeprom. On sparc platforms, it caused
	 * to panic the system with PCI bus timeout errors.
	 * We must wait for 2.5 mS at least in experience.
	 */
	delay(drv_usectohz(10000));

	/* disable WOL */
	if (IS_RHINE2_OR_LATER(revid)) {
		ddi_put8(reg_ha, base + STICKHW,
		    ddi_get8(reg_ha, base + STICKHW) &
		    ~(STICKHW_DS1 | STICKHW_DS0));
		ddi_put8(reg_ha, base + WOLCG_CLR, WOLCG_PMEOVR);
		ddi_put8(reg_ha, base + WOLCR_CLR, 0xff);
		if (revid > 0x83 /* VT6105B0 */) {
			ddi_put8(reg_ha, base + TSTREG_CLR, 0x03);
		}
		ddi_put8(reg_ha, base + PWRCSR_CLR, 0xff);
		if (revid > 0x83 /* VT6105B0 */) {
			ddi_put8(reg_ha, base + PWRCSR1_CLR, 0x03);
		}
	}

	ddi_regs_map_free(&reg_ha);

	/* check chip revision and report it */
	for (i = 0, p = vfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			cmn_err(CE_CONT, "!%s%d: %s"
			    " (vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
			    drv_name, unit, p->name, vid, did, revid);
			break;
		}
	}
	if (i >= CHIPTABLESIZE) {
		/* Not found */
		cmn_err(CE_NOTE,
		    "!%s%d: %s: unknown PCI venid/devid (0x%x, 0x%x)",
		    drv_name, unit, __func__, vid, did);
	}

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/* XXX - memory mapped io doesn't work for sparc platforms. */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_IO, PCI_ADDR_MASK,
		    &vfe_dev_attr, (void *)&base, &reg_ha)
		    != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * construct gem configuration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		(void) sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = IS_RHINE2_OR_LATER(revid)
		    ? (sizeof (uint8_t) - 1)
		    : (sizeof (uint32_t) - 1);
		/* rhine I supports only a fragment per tx packet */
		gcp->gc_tx_max_frags =
		    IS_RHINE2_OR_LATER(revid) ? MAXTXFRAGS : 1;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = 4; /* 16byte */
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = vfe_tx_copy_thresh;
#ifndef GEM_CONFIG_TX_DIRECT
		gcp->gc_tx_desc_write_oo = B_TRUE;
#endif

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4; /* 16byte */
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = min(RX_BUF_SIZE, 255);
		gcp->gc_rx_copy_thresh = vfe_rx_copy_thresh;

		gcp->gc_io_area_size = 16; /* a dummy tx desc */

		/* map attributes */
		gcp->gc_dev_attr = vfe_dev_attr;
		gcp->gc_buf_attr = vfe_buf_attr;
		gcp->gc_desc_attr = vfe_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = vfe_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = vfe_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		gcp->gc_dma_attr_rxbuf = vfe_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;


		/* timeout parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;
		if (IS_RHINE2_OR_LATER(revid)) {
			/* rx_pause implies symmetric */
			gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
		}

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0; /* was (25*ONESEC)/10 */
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;
#ifdef CONFIG_MAUTO
		gcp->gc_mii_hw_link_detection = IS_RHINE2_OR_LATER(revid);
#else
		gcp->gc_mii_hw_link_detection = B_FALSE;
#endif

		/* workaround for DAVICOM PHY */
		gcp->gc_mii_an_delay = ONESEC/10;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &vfe_attach_chip;
		gcp->gc_reset_chip = &vfe_reset_chip;
		gcp->gc_init_chip = &vfe_init_chip;
		gcp->gc_start_chip = &vfe_start_chip;
		gcp->gc_stop_chip = &vfe_stop_chip;
		gcp->gc_multicast_hash = &vfe_mcast_hash;
		gcp->gc_set_rx_filter = &vfe_set_rx_filter;
		gcp->gc_set_media = &vfe_set_media;
		gcp->gc_get_stats = &vfe_get_stats;
		gcp->gc_interrupt = &vfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &vfe_tx_desc_write;
		gcp->gc_rx_desc_write = &vfe_rx_desc_write;
		gcp->gc_tx_start = &vfe_tx_start;
		gcp->gc_rx_start = &vfe_rx_start;
		gcp->gc_tx_desc_stat = &vfe_tx_desc_stat;
		gcp->gc_rx_desc_stat = &vfe_rx_desc_stat;
		gcp->gc_tx_desc_init = &vfe_tx_desc_init;
		gcp->gc_rx_desc_init = &vfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &vfe_tx_desc_clean;
		gcp->gc_rx_desc_clean = &vfe_rx_desc_clean;
		gcp->gc_get_packet = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_probe = &vfe_mii_probe;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &vfe_mii_sync;
		gcp->gc_mii_read = &vfe_mii_read;
		gcp->gc_mii_write = &vfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof (struct vfe_dev), KM_SLEEP);
		lp->revid = revid;
		dp = gem_do_attach(dip, 0,
		    gcp, base, &reg_ha, lp, sizeof (*lp));
		kmem_free(gcp, sizeof (*gcp));
		if (dp == NULL) {
			goto err_free_mem;
		}

		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof (struct vfe_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
vfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	switch (cmd) {
	case DDI_DETACH:
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
GEM_STREAM_OPS(vfe_ops, vfeattach, vfedetach);
#else
static	struct module_info vfeminfo = {
	0,			/* mi_idnum */
	"vfe",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64 * 1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit vferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&vfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit vfewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&vfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	vfe_info = {
	&vferinit,	/* st_rdinit */
	&vfewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_vfe_ops = {
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
	&vfe_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops vfe_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	vfeattach,	/* devo_attach */
	vfedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_vfe_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&vfe_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!vfe: _init: called"));
	gem_mod_init(&vfe_ops, "vfe");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&vfe_ops);
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

	DPRINTF(2, (CE_CONT, "!vfe: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&vfe_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
