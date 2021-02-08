/*
 * nfo: nforce integrated ethernet mac driver
 *
 * Copyright (c) 2005-2011 Masayuki Murayama.  All rights reserved.
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
 * Acknowledgement:
 * I thank and respect the authors and the contributors of forcedeth driver
 * in Linux. As NVIDIA doesn't publish datasheets of the ethernet core
 * in nForce chepset, I made this driver by experience on re-engineering
 * the source code of forcedeth driver.
 */

#pragma	ident	"%Z%%M% %I%     %E%"

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
#include "nvenetreg.h"

char	ident[] = "nForce ethernet v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int nfo_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (nfo_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC		drv_usectohz(1*1000000)
#define	ARRAY_SIZE(a)	(sizeof (a) / sizeof ((a)[0]))

#define	LE32PACKED(a)	\
	(((a)[3] << 24) | ((a)[2] << 16) | ((a)[1] << 8) | (a)[0])
#define	LE16PACKED(a)	\
	(((a)[1] << 8) | (a)[0])

#define	FLSHB(dp, reg)	(void) INB(dp, reg)
#define	FLSHW(dp, reg)	(void) INW(dp, reg)
#define	FLSHL(dp, reg)	(void) INL(dp, reg)

#define	IS_DFMT1(dmactrl)	\
	(((dmactrl) & DMACTRL_DFMT) == DMACTRL_DFMT_1)

#define	IS_DFMT2(dmactrl)	\
	(((dmactrl) & DMACTRL_DFMT) == DMACTRL_DFMT_2)

#define	IS_DFMT3(dmactrl)	\
	(((dmactrl) & DMACTRL_DFMT) == DMACTRL_DFMT_3)

#define	OUI_CICADA	0x03f1U
#define	OUI_VITESSE	0x01c1U
#define	OUI_MARVELL	0x5043U
#define	OUI_REALTEK	0x0732U
#define	OUI_REALTEK_2	0x0020U

#define	MODEL_MARVELL_E3016	0x0220
#define	MODEL_REALTEK_8211	0x0110
#define	REV_REALTEK_8211B	0x0000
#define	REV_REALTEK_8211C	0x0001
#define	MODEL_REALTEK_8201	0x0200

#define	OUI_CMP(dp, oui)	\
	((((dp)->mii_phy_id >> 10) ^ (oui)) & 0xffff)
#define	IS_PHY(dp, oui, model)	\
	(OUI_CMP(dp, oui) == 0 && (model) == ((dp)->mii_phy_id & 0x03f0))
#define	IS_PHY_REV(dp, oui, model, rev)	\
	(OUI_CMP(dp, oui) == 0 && \
	((model) | (rev)) == ((dp)->mii_phy_id & 0x03ff))

#define	TXDESC32(dp)	((struct txdesc32 *)((void *)(dp)->tx_ring))
#define	TXDESC64(dp)	((struct txdesc64 *)((void *)(dp)->tx_ring))
#define	RXDESC32(dp)	((struct rxdesc32 *)((void *)(dp)->rx_ring))
#define	RXDESC64(dp)	((struct rxdesc64 *)((void *)(dp)->rx_ring))

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif
/*
 * Our configuration
 */
#define	OUR_INTR_BITS	\
	(INTR_RXERR_FATAL | INTR_TXDESC | INTR_RXDESC | INTR_LSC | \
	INTR_TXERR | INTR_RDU | INTR_RXOK | INTR_RXERR)

#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#ifdef DEBUG_TX_SINGLE
static int	nfo_tx_copy_thresh = INT32_MAX;
#else
static int	nfo_tx_copy_thresh = 256;
#endif
static int	nfo_rx_copy_thresh = 256;

/* supported chipsets */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
	uint_t		hwcap;
#define	CAP_JUMBO	0x00000001
#define	CAP_CKSUM	0x00000002
#define	CAP_64BIT	0x00000004
#define	CAP_VLAN	0x00000008
#define	CAP_MSI		0x00000010
#define	CAP_MSIX	0x00000020
#define	CAP_PM		0x00000040
#define	CAP_TXPAUSE1	0x00000080
#define	CAP_TXPAUSE2	0x00000100
#define	CAP_TXPAUSE3	0x00000200
#define	CAP_STAT	0x00000400
#define	CAP_TEST	0x00000800
#define	CAP_MGMT	0x00001000
#define	CAP_RESET	0x00002000
#define	CAP_CORRECT_MAC	0x00004000
#define	CAP_WA_TX16	0x00008000
#define	CAP_WA_TX16_2	0x00010000
#define	CAP_COLFIX	0x00020000
#define	CAP_NEWBACKOFF	0x00040000
#define	CAP_PHY_WA	0x00080000
#define	CAP_MSI_WA	0x00100000
#define	CAP_WA_NO_LSO	0x00200000
};


/* private driver state */
struct nfo_dev {
	uint8_t			mac_addr[ETHERADDRL];
	uint32_t		factory_mac_addr[2];
	struct chip_info	*chip;
	uint_t			hwcap;	/* copy of hwcap in chip_info */

	uint32_t		imr;
	uint32_t		dmac;

	/* statstics support */
	clock_t			last_stats_time;
#ifdef CONFIG_POLLING
	int			last_poll_interval;
#endif

	boolean_t		mac_in_use;
	boolean_t		phy_initialized;
	boolean_t		mac_initialized;

	uint8_t			revid;

	uint16_t		mii_led_stats[3];
};

/*
 * Macros to identify chip generation.
 */
static struct chip_info nfo_chiptbl[] = {
0x10de,	0x01c3,	"type1 (nForce MCP)",		0,
0x10de,	0x0066,	"type2 (nForce MCP-T)",		CAP_RESET,
0x10de,	0x00d6,	"type3 (nForce3 150)",		CAP_RESET,
0x10de,	0x0086,	"type4 (nForce2 MCP-R)",	CAP_JUMBO | CAP_CKSUM,
0x10de,	0x008c,	"type5 (nForce2 MCP-Gb)",	CAP_JUMBO | CAP_CKSUM,
0x10de,	0x00e6,	"type6 (nForce3 250Gb)",	CAP_JUMBO | CAP_CKSUM,
0x10de,	0x00df,	"type7 (nForce3 250)",		CAP_JUMBO | CAP_CKSUM,

#define	CAP_CK804	(CAP_JUMBO | CAP_CKSUM | CAP_64BIT | CAP_WA_TX16)
0x10de,	0x0056,	"type8 (nForce4 CK804)",	CAP_CK804,
0x10de,	0x0057,	"type9 (nForce4 Ultra CK804)",	CAP_CK804,

#define	CAP_MCP04	(CAP_JUMBO | CAP_CKSUM | CAP_64BIT | CAP_WA_TX16)
0x10de,	0x0037,	"type10 (nForce MCP04)",	CAP_MCP04,
0x10de,	0x0038,	"type11 (nForce MCP04 Ultra)",	CAP_MCP04,

#define	CAP_MCP51	(CAP_64BIT | CAP_PM)
0x10de, 0x0268, "type12 (nForce MCP51)",	CAP_MCP51,
0x10de, 0x0269, "type13 (GF6150/NF430,GF6100/NF410 MCP51)", CAP_MCP51,

#define	CAP_MCP55	\
	(CAP_JUMBO | CAP_CKSUM | CAP_64BIT | CAP_VLAN | CAP_MSI | \
	CAP_MSIX | CAP_PM | CAP_TXPAUSE1 | CAP_STAT | CAP_TEST | CAP_MGMT | \
	CAP_WA_TX16 | CAP_MSI_WA)
0x10de, 0x0372, "type14 (nForce590 MCP55)",	CAP_MCP55,
0x10de, 0x0373, "type15 (nForce570 Ultra, 680a, MCP55)",	CAP_MCP55,

#define	CAP_MCP61	\
	(CAP_64BIT | CAP_PM | CAP_MSI | CAP_TXPAUSE1 | CAP_STAT | \
	CAP_TEST | CAP_MGMT | CAP_CORRECT_MAC | CAP_MSI_WA)
0x10de, 0x03e5, "type16 (nForce MCP61)",	CAP_MCP61,
0x10de, 0x03e6, "type17 (nForce MCP61)",	CAP_MCP61,
0x10de, 0x03ee, "type18 (nForce MCP61)",	CAP_MCP61,
0x10de, 0x03ef, "type19 (GF6100/nF430, GF6100/nF405, MCP61)", CAP_MCP61,

#define	CAP_MCP65	(CAP_MCP61 | CAP_JUMBO | CAP_CKSUM | CAP_WA_TX16)
0x10de, 0x0450, "type20 (nForce MCP65)",	CAP_MCP65,
0x10de, 0x0451, "type21 (nForce MCP65)",	CAP_MCP65,
0x10de, 0x0452, "type22 (nForce MCP65)",	CAP_MCP65,
0x10de, 0x0453, "type23 (nForce MCP65)",	CAP_MCP65,

#define	CAP_MCP67	CAP_MCP61
0x10de, 0x054c, "type24 (GF7000M/nF610M MCP67-MV)",	CAP_MCP67,
0x10de, 0x054d, "type25 (nForce MCP67)",	CAP_MCP67,
0x10de, 0x054e, "type26 (nForce MCP67)",	CAP_MCP67,
0x10de, 0x054f, "type27 (nForce MCP67)",	CAP_MCP67,

#define	CAP_MCP73	\
	(CAP_64BIT | CAP_PM | CAP_MSI | CAP_TXPAUSE1 | CAP_STAT | \
	CAP_TEST | CAP_MGMT | CAP_CORRECT_MAC | CAP_COLFIX | CAP_MSI_WA)
0x10de, 0x07dc, "type28 (nForce MCP73)",	CAP_MCP73,
0x10de, 0x07dd, "type29 (nForce MCP73)",	CAP_MCP73,
0x10de, 0x07de, "type30 (nForce MCP73)",	CAP_MCP73,
0x10de, 0x07df, "type31 (nForce MCP73)",	CAP_MCP73,

#define	CAP_MCP77	\
	(CAP_CKSUM | CAP_64BIT | CAP_PM | CAP_MSI | CAP_TXPAUSE2 | CAP_STAT | \
	CAP_TEST | CAP_MGMT | CAP_CORRECT_MAC | CAP_WA_TX16_2 | CAP_COLFIX | \
	CAP_PHY_WA | CAP_MSI_WA)
0x10de, 0x0760, "type32 (GF8200, MCP77)",	CAP_MCP77,
0x10de, 0x0761, "type33 (nForce MCP77)",	CAP_MCP77,
0x10de, 0x0762, "type34 (nForce MCP77)",	CAP_MCP77,
0x10de, 0x0763, "type35 (nForce MCP77)",	CAP_MCP77,

#define	CAP_MCP79	\
	(CAP_JUMBO | CAP_CKSUM | CAP_64BIT | CAP_PM | CAP_MSI | \
	CAP_TXPAUSE3 | CAP_STAT | CAP_TEST | CAP_MGMT | CAP_CORRECT_MAC | \
	CAP_WA_TX16_2 | CAP_COLFIX | CAP_PHY_WA | CAP_MSI_WA)
0x10de, 0x0ab0, "type36 (nForce MCP79)",	CAP_MCP79,
0x10de, 0x0ab1, "type37 (nForce MCP79)",	CAP_MCP79,
0x10de, 0x0ab2, "type38 (nForce MCP79)",	CAP_MCP79,
0x10de, 0x0ab3, "type39 (nForce MCP79)",	CAP_MCP79,

#define	CAP_MCP89	\
	(CAP_CKSUM | CAP_64BIT | CAP_PM | CAP_MSI | CAP_TXPAUSE3 | CAP_STAT | \
	CAP_TEST | CAP_MGMT | CAP_CORRECT_MAC | CAP_COLFIX | CAP_PHY_WA)
0x10de, 0x0d7d, "type40 (nForce MCP89)",	CAP_MCP89,
};
#define	CHIPTABLESIZE	(sizeof (nfo_chiptbl) / sizeof (struct chip_info))

#define	IS_TYPE4_OR_LATER(lp)	((lp)->hwcap & (CAP_JUMBO | CAP_64BIT))

/* ======================================================== */
/* local functions */

/* mii operations */
static void  nfo_mii_sync(struct gem_dev *);
static uint16_t  nfo_mii_read(struct gem_dev *, uint_t);
static void nfo_mii_write_raw(struct gem_dev *, uint_t, uint16_t);
static void nfo_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int nfo_reset_chip(struct gem_dev *);
static int nfo_init_chip(struct gem_dev *);
static int nfo_start_chip(struct gem_dev *);
static int nfo_stop_chip(struct gem_dev *);
static int nfo_set_media(struct gem_dev *);
static int nfo_set_rx_filter(struct gem_dev *);
static int nfo_get_stats(struct gem_dev *);

/* descriptor operations */
static int nfo_tx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static int nfo_tx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static uint_t nfo_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t nfo_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static void nfo_rx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void nfo_rx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);

static void nfo_tx_desc_dump32(struct gem_dev *);
static void nfo_tx_desc_dump64(struct gem_dev *);

static void nfo_tx_desc_init32(struct gem_dev *dp, int slot);
static void nfo_tx_desc_init64(struct gem_dev *dp, int slot);
static void nfo_rx_desc_init32(struct gem_dev *dp, int slot);
static void nfo_rx_desc_init64(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t nfo_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr nfo_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr nfo_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t nfo_dma_attr_buf32 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x00004000ull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003fff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x00004000ull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t nfo_dma_attr_buf64 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x000000ffffffffffull,	/* dma_attr_addr_hi (40bit) */
	0x0000000000004000ull,	/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003fff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000000000004000ull,	/* dma_attr_maxxfer */
	0x000000ffffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t nfo_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
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
 *	Debug routines
 */
/* ======================================================== */
#ifdef DEBUG_LEVEL
static void
nfo_dump_regs(struct gem_dev *dp, char *title)
{
	int		i;
	ssize_t		len = 0x400;

	cmn_err(CE_CONT, "!%s", title);

	for (i = 0; i < len; i += 4*4) {
		cmn_err(CE_CONT, "!%04x: %08x %08x %08x %08x",
		    i, INL(dp, i), INL(dp, i + 4*1),
		    INL(dp, i + 4*2), INL(dp, i + 4*3));
	}
}
#endif

/* ======================================================== */
/*
 * HW manipulation routines
 */
/* ======================================================== */
static int
nfo_reset_chip(struct gem_dev *dp)
{
	int		ret;
	uint32_t	miiauto;
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called, time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	ret = GEM_SUCCESS;
#if DEBUG_LEVEL > 0
	nfo_dump_regs(dp, (void *)__func__);
#endif
	/*
	 * Disable MIIAUTO
	 */
	miiauto = INL(dp, MIIAUTO);
	if (miiauto & MIIAUTO_EN) {
		OUTL(dp, MIIAUTO, miiauto & ~MIIAUTO_EN);
	}

	/*
	 * Inhibit interrupts
	 */
	OUTL(dp, MIIIMR, 0);
	FLSHL(dp, MIIIMR);
	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);

	if (!lp->mac_in_use) {
		OUTL(dp, RxPOLL, 0);
		OUTL(dp, TxPOLL, INL(dp, TxPOLL) & TxPOLL_MACADDR_REV);
	}

	/* halt dma */
	OUTL(dp, DMACTRL, DMACTRL_RESET | DMACTRL_STOP | lp->dmac);
	drv_usecwait(20);

	if ((lp->hwcap & CAP_PM) && !lp->mac_initialized) {
		/* reset mac */
		OUTL(dp, MACRESET, 0xf3);
		FLSHL(dp, MACRESET);
		drv_usecwait(100);

		OUTL(dp, MACRESET, 0);
		FLSHL(dp, MACRESET);
		drv_usecwait(100);
	}
#ifdef NEVER
	/* XXX - not required */
	if (lp->chip->hwcap & CAP_RESET)  {
		uint32_t	mac0_save;
		uint32_t	mac1_save;

		mac1_save = INL(dp, MACADDR + 4);
		mac0_save = INL(dp, MACADDR);

		OUTL(dp, MACRESET, INL(dp, MACRESET) | 0xff);
		drv_usecwait(10);

		OUTL(dp, MACRESET, INL(dp, MACRESET) & ~0xff);
		drv_usecwait(10);

		OUTL(dp, MACADDR + 4, mac1_save);
		OUTL(dp, MACADDR, mac0_save);
	}
#endif
	OUTL(dp, DMACTRL, DMACTRL_STOP | lp->dmac);
	drv_usecwait(20);

	/* disable timer */
	OUTL(dp, TIMERCTL, 0);

	if (!lp->mac_initialized) {
		/* clear MII error */
		OUTL(dp, MIIISR, MIIISR_ALL);

		/* clear bugus interrupts */
		OUTL(dp, ISR, INTR_ALL);
	}

	if (!IS_TYPE4_OR_LATER(lp)) {
		/* old chipset have only MII interface for 100Mbps */
		OUTL(dp, PHYIF, INL(dp, PHYIF) & ~PHYIF_RGMII);
#ifdef NEVER
	} else {
		/* recent chipset have RGMII interface for 1Gbps */
		OUTL(dp, PHYIF, INL(dp, PHYIF) | PHYIF_RGMII);
#endif
	}

	if (lp->mac_initialized && (miiauto & MIIAUTO_EN)) {
		/*
		 * restore MIIAUTO
		 */
		OUTL(dp, MIIAUTO, miiauto);
	}

	lp->mac_initialized = B_TRUE;

	return (ret);
}

static int
nfo_init_chip(struct gem_dev *dp)
{
	int		i;
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* setup tx and rx descriptor rings */
	OUTL(dp, RxDESCBASE, (uint32_t)dp->rx_ring_dma);
	OUTL(dp, TxDESCBASE, (uint32_t)dp->tx_ring_dma);
	if (IS_DFMT3(lp->dmac)) {
		OUTL(dp, RxDESCBASE_HI, (uint32_t)(dp->rx_ring_dma >> 32));
		OUTL(dp, TxDESCBASE_HI, (uint32_t)(dp->tx_ring_dma >> 32));
	}
	OUTL(dp, RINGCFG,
	    ((dp->gc.gc_rx_ring_size - 1) << RINGCFG_RX_SHIFT)
	    | (dp->gc.gc_tx_ring_size - 1));

	OUTL(dp, RxPOLL, RxPOLL_EN | RxPOLL_ITV_10M);

	/* unreset DMAC */
	OUTL(dp, DMACTRL, lp->dmac);

	/* start rx side of dma engine */
	OUTL(dp, DMACTRL, DMACTRL_RxPOLL | lp->dmac);
	for (i = 0; (INL(dp, RxDMASTAT) & RxDMASTAT_RUNNING) == 0; i++) {
		if (i > 1000) {
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: failed to start rx side of dma engine",
			    dp->name, __func__));
			break;
		}
		drv_usecwait(10);
	}

	OUTL(dp, TxSTAT, INL(dp, TxSTAT));

	OUTL(dp, MAXBUFSIZE,
	    dp->mtu + sizeof (struct ether_header) + ETHERFCSL + 4);

	OUTL(dp, RxSTAT, INL(dp, RxSTAT));

	(void) nfo_set_media(dp);

	OUTL(dp, RxDEFER, 0x16);
#ifdef CONFIG_VLAN_HW
	if (lp->hwcap & CAP_VLAN) {
		OUTL(dp, VLANCTRL, VLANCTRL_EN);
	}
#endif
#ifdef CONFIG_POLLING
	lp->last_poll_interval = 0;
#endif

	DPRINTF(2, (CE_CONT, "!%s: %s: out", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
nfo_start_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	val = INL(dp, PWRSTAT);
	if ((val & PWRSTAT_POWEREDUP) == 0) {
		OUTL(dp, PWRSTAT, PWRSTAT_POWEREDUP | val);
	}
	OUTL(dp, PWRSTAT, INL(dp, PWRSTAT) | PWRSTAT_VALID);

	lp->imr = OUR_INTR_BITS;
#if DEBUG_LEVEL >= 5
	lp->imr |= INTR_TXOK;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, MIIIMR, MIIISR_LSC);
		FLSHL(dp, MIIIMR);
		OUTL(dp, IMR, lp->imr);
		FLSHL(dp, IMR);
#ifdef CONFIG_POLLING
		OUTL(dp, TIMERCTL, 3);
		FLSHL(dp, TIMERCTL);
#endif
	}

	/* start rx */
	val = INL(dp, RxCTRL) | RxCTRL_START;
	if (lp->mac_in_use) {
		/* take rx path from management unit */
		val &= ~RxCTRL_RxPATH;
	}
	OUTL(dp, RxCTRL, val);

	/* start tx activety */
	val = INL(dp, TxCTRL) | TxCTRL_START;
	if (lp->mac_in_use) {
		/* take tx path from management unit */
		val &= ~TxCTRL_TxPATH;
	}
	OUTL(dp, TxCTRL, val);

	DPRINTF(2, (CE_CONT, "!%s: %s: return", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
nfo_stop_chip(struct gem_dev *dp)
{
	int		i;
	int		ret;
	uint32_t	val;
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));
#if DEBUG_LEVEL > 0
	if (dp->tx_desc_head != dp->tx_desc_tail) {
		if (IS_DFMT3(lp->dmac)) {
			nfo_tx_desc_dump64(dp);
		} else {
			nfo_tx_desc_dump32(dp);
		}
	}
#endif

	ret = GEM_SUCCESS;
#ifdef notdef
	/* wait until dma engine becomes idle */
	OUTL(dp, DMACTRL, DMACTRL_STOP | lp->dmac);

	for (i = 0; (INL(dp, DMACTRL) & DMACTRL_IDLE) == 0; i++) {
		if (i > 10000) {
			cmn_err(CE_WARN, "!%s: %s: timeout - DMACTRL busy",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(50);
	}
#endif
	/* stop tx activety */
	val = INL(dp, TxCTRL);
	if (!lp->mac_in_use) {
		/* stop tx completely */
		val &= ~TxCTRL_START;
	} else {
		/* return tx path for management unit */
		val |= TxCTRL_TxPATH;
	}
	OUTL(dp, TxCTRL, val);

	for (i = 0; (INL(dp, TxSTAT) & TxSTAT_RUNNING); i++) {
		if (i > 500) {
			cmn_err(CE_WARN, "!%s: %s tx engine is still running",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(10);
	}
	drv_usecwait(100);

	/* stop rx activety */
	val = INL(dp, RxCTRL);
	if (!lp->mac_in_use) {
		/* stop rx completely */
		val &= ~RxCTRL_START;
	} else {
		/* return rx path for management unit */
		val |= RxCTRL_RxPATH;
	}
	OUTL(dp, RxCTRL, val);

	for (i = 0; (INL(dp, RxSTAT) & RxSTAT_RUNNING); i++) {
		if (i > 500) {
			cmn_err(CE_WARN, "!%s: %s: rx engine is still running",
			    dp->name, __func__);
			ret = GEM_FAILURE;
			break;
		}
		drv_usecwait(10);
	}
	drv_usecwait(100);

	/* inhibit interrupts */
	OUTL(dp, MIIIMR, 0);
	FLSHL(dp, MIIIMR);
	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);

	/* reset dma engine */
	OUTL(dp, DMACTRL, DMACTRL_RESET | DMACTRL_STOP | lp->dmac);
	FLSHL(dp, DMACTRL);
	drv_usecwait(20);

	OUTL(dp, DMACTRL, DMACTRL_STOP | lp->dmac);
	FLSHL(dp, DMACTRL);
	drv_usecwait(20);

	/* disable timer */
	OUTL(dp, TIMERCTL, 0);
	FLSHL(dp, TIMERCTL);

	return (ret);
}

static int
nfo_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint32_t	mode;
	uint8_t		*m;
	uint64_t	mac;
	uint64_t	oraddr;
	uint64_t	andaddr;
	uint64_t	MulticastMask;
	uint64_t	MulticastAddress;
	struct nfo_dev	*lp = dp->private;

	m = dp->cur_addr.ether_addr_octet;
#ifdef GEM3
	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "state:%d, rxmode:%b, %02x:%02x:%02x:%02x:%02x:%02x",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_state, dp->rxmode, RXMODE_BITS,
	    m[0], m[1], m[2], m[3], m[4], m[5]));
#else
	DPRINTF(1, (CE_CONT, "!%s: %s: called at time:%d, "
	    "active:%d, rxmode:%b, %02x:%02x:%02x:%02x:%02x:%02x",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, dp->rxmode, RXMODE_BITS,
	    m[0], m[1], m[2], m[3], m[4], m[5]));
#endif

	/*
	 * Initialize filter
	 */
	bzero(&oraddr, sizeof (oraddr));
	bzero(&andaddr, sizeof (andaddr));
	mode = RxPF_MBO | (INL(dp, RxPF) & RxPF_RxPAUSE);

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		MulticastMask = 0xffffffffffffull;
		MulticastAddress = 0x000000000000ull;
	} else if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		mode |= RxPF_PROMISC;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 || dp->mc_count > 32) {
		mode |= RxPF_UNICAST;
		MulticastMask = 0x000000000001ull;
		MulticastAddress = 0x000000000001ull;
	} else if (dp->mc_count > 0) {
		/*
		 * Process multicast address list
		 */
		mode |= RxPF_UNICAST;

		andaddr = 0xffffffffffffull;
		oraddr = 0x000000000000ull;

		for (i = 0; i < dp->mc_count; i++) {
			m = dp->mc_list[i].addr.ether_addr_octet;
			mac = (((uint64_t)LE16PACKED(m+4)) << 32)
			    | ((uint64_t)LE32PACKED(m));

			andaddr &= mac;
			oraddr |= mac;
		}

		/*
		 * Write 1s to mask where all addresses have 0 bit or 1 bit.
		 * Compute mask and address value.
		 */
		MulticastAddress = (andaddr & oraddr) | 1ull;
		MulticastMask = (andaddr | ~oraddr) | 1ull;
	} else {
		mode |= RxPF_UNICAST;
	}

	OUTL(dp, RxPF, mode);
	if ((dp->rxmode & RXMODE_ENABLE) != 0) {
		OUTL(dp, MCASTMASK + 0, 1);
		OUTL(dp, MCASTMASK + 4, 0);
	}
	OUTL(dp, MCASTADDR + 0, (uint32_t)MulticastAddress);
	OUTL(dp, MCASTADDR + 4, (uint32_t)(MulticastAddress >> 32));
	OUTL(dp, MCASTMASK + 0, (uint32_t)MulticastMask);
	OUTL(dp, MCASTMASK + 4, (uint32_t)(MulticastMask >> 32));

	m = dp->cur_addr.ether_addr_octet;
	if (bcmp(m, lp->mac_addr, ETHERADDRL) != 0) {
		/* Set new node address */
		bcopy(m, lp->mac_addr, ETHERADDRL);
		OUTL(dp, MACADDR + 0, LE32PACKED(m));
		OUTL(dp, MACADDR + 4, LE16PACKED(m+4));
	}

	DPRINTF(2, (CE_CONT, "!%s: %s return", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
nfo_set_media(struct gem_dev *dp)
{
	uint32_t	rxpoll;
	uint32_t	txcfg;
	uint32_t	backoff;
	uint32_t	rxpf;
	uint32_t	phyif;
	uint32_t	txdefer;
	uint32_t	txwm;
	struct nfo_dev	*lp = dp->private;

#ifdef GEM3
	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%ld mac_state:%d",
	    dp->name, __func__, ddi_get_lbolt(), dp->mac_state));
#else
	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%ld mac_active:%d",
	    dp->name, __func__, ddi_get_lbolt(), dp->mac_active));
#endif

	/*
	 * Set speed and duplex settings depend on the hardware
	 */

	/* tx config register */
	txcfg = TxCFG_DEFAULT;
	/*
	 * XXX - HD bit is required for legacy chipset, i.e. MCP1 family,
	 * but it seems no effect for recent chipset.
	 */
	if (!dp->full_duplex) {
		txcfg |= TxCFG_HD;
	}
	if (lp->hwcap & (CAP_TXPAUSE1 | CAP_TXPAUSE2 | CAP_TXPAUSE3)) {
		switch (dp->flow_control) {
		case FLOW_CONTROL_SYMMETRIC:
		case FLOW_CONTROL_TX_PAUSE:
			txcfg |= TxCFG_TxPAUSE;
			break;

		default:
			txcfg &= ~TxCFG_TxPAUSE;
			break;
		}
	}

	DPRINTF(1, (CE_CONT, "!%s: %s TxCFG:%x -> %x",
	    dp->name, __func__, INL(dp, TxCFG), txcfg));
	OUTL(dp, TxCFG, txcfg);
	FLSHL(dp, TxCFG);

	/* setup rx flow control */
	rxpf = INL(dp, RxPF);

	switch (dp->flow_control) {
	case FLOW_CONTROL_SYMMETRIC:
	case FLOW_CONTROL_RX_PAUSE:
		rxpf |= RxPF_RxPAUSE;
		break;

	case FLOW_CONTROL_TX_PAUSE:
	case FLOW_CONTROL_NONE:
		rxpf &= ~RxPF_RxPAUSE;
		break;
	}

	DPRINTF(1, (CE_CONT, "!%s: %s RxPF:%x -> %x",
	    dp->name, __func__, INL(dp, RxPF), rxpf));
	OUTL(dp, RxPF, rxpf);
	FLSHL(dp, RxPF);

	/* tx watermark register */
	if (IS_DFMT1(lp->dmac)) {
		/* descriptor format 1 */
		txwm = 0x00200010;
	} else {
		/* descriptor format 2 or 3 */
		if (dp->speed == GEM_SPD_1000) {
			txwm = 0x0fe08000;
		} else {
			txwm = 0x01e08000;
		}
	}

	DPRINTF(1, (CE_CONT, "!%s: %s TXWM:%x -> %x",
	    dp->name, __func__, INL(dp, TXWM), txwm));
	OUTL(dp, TXWM, txwm);
	FLSHL(dp, TXWM);

	/* rx polling register */
	rxpoll = RxPOLL_EN;
	switch (dp->speed) {
	case GEM_SPD_1000:
		rxpoll |= RxPOLL_ITV_1G;
		break;

	case GEM_SPD_10:
		rxpoll |= RxPOLL_ITV_10M;
		break;

	case GEM_SPD_100:
		rxpoll |= RxPOLL_ITV_100M;
		break;

	}
	DPRINTF(1, (CE_CONT, "!%s: %s RxPOLL:%x -> %x",
	    dp->name, __func__, INL(dp, RxPOLL), rxpoll));
	OUTL(dp, RxPOLL, rxpoll);
	FLSHL(dp, RxPOLL);

	/* phy interface register */
	phyif = INL(dp, PHYIF) & ~(PHYIF_HALF | PHYIF_SPEED);
	/*
	 * XXX - duplex and speed bits are required for recent chipset,
	 * but it seems no effect for legacy chipset.
	 */
	if (!dp->full_duplex) {
		phyif |= PHYIF_HALF;
	}
	switch (dp->speed) {
	case GEM_SPD_10:
		phyif |= PHYIF_SPEED_10M;
		break;

	case GEM_SPD_100:
		phyif |= PHYIF_SPEED_100M;
		break;

	case GEM_SPD_1000:
		phyif |= PHYIF_SPEED_1G;
		break;
	}

	DPRINTF(1, (CE_CONT, "!%s: %s PHYIF:%x -> %x",
	    dp->name, __func__, INL(dp, PHYIF), phyif));
	OUTL(dp, PHYIF, phyif);

	/* backoff and tx defer register */
#if 0
	backoff = 8;
#else
	if (lp->hwcap & CAP_NEWBACKOFF) {
		/* not yet */
		backoff = 0;
		/* load random number table via backoff control register */
	} else {
		backoff = BACKOFF_SEED & ddi_get_lbolt();
		if (!IS_DFMT1(lp->dmac)) {
			backoff |= BACKOFF_LEGACY;
		}
	}
#endif
	if (phyif & PHYIF_RGMII) {
		switch (dp->speed) {
		case GEM_SPD_10:
			backoff |= BACKOFF_SPEED_10M;
			if ((dp->mii_exp & MII_AN_EXP_LPCANAN) == 0 &&
			    (!dp->full_duplex) && (lp->hwcap & CAP_COLFIX)) {
				txdefer = 0x0016190f;
			} else {
				txdefer = 0x0016070f;
			}
			break;

		case GEM_SPD_100:
			backoff |= BACKOFF_SPEED_100M;
			if ((dp->mii_exp & MII_AN_EXP_LPCANAN) == 0 &&
			    (!dp->full_duplex) && (lp->hwcap & CAP_COLFIX)) {
				txdefer = 0x0016300f;
			} else {
				txdefer = 0x0016070f;
			}
			break;

		case GEM_SPD_1000:
			backoff |= BACKOFF_SPEED_1G;
			txdefer = 0x0014050f;
			break;
		}
	} else {
		backoff |= BACKOFF_SPEED_MII;
		if ((dp->mii_exp & MII_AN_EXP_LPCANAN) == 0 &&
		    (!dp->full_duplex) && (lp->hwcap & CAP_COLFIX)) {
			txdefer = 0x152000;
		} else {
			txdefer = 0x15050f;
		}
	}

	DPRINTF(1, (CE_CONT, "!%s: %s BACKOFF:%x -> %x",
	    dp->name, __func__, INL(dp, BACKOFF), backoff));
	OUTL(dp, BACKOFF, backoff);
	FLSHL(dp, BACKOFF);

	DPRINTF(1, (CE_CONT, "!%s: %s TxDEFER:%x -> %x",
	    dp->name, __func__, INL(dp, TxDEFER), txdefer));
	OUTL(dp, TxDEFER, txdefer);
	FLSHL(dp, TxDEFER);

	/* tx pause regiseter */
	if (lp->hwcap & (CAP_TXPAUSE1 | CAP_TXPAUSE2 | CAP_TXPAUSE3)) {
		uint32_t	txpause;

		if ((txcfg & TxCFG_TxPAUSE) == 0) {
			txpause = TXPAUSE_DIS;
		} else if (lp->hwcap & CAP_TXPAUSE1) {
			txpause = TXPAUSE_EN1;
		} else if (lp->hwcap & CAP_TXPAUSE2) {
			txpause = TXPAUSE_EN2;
		} else {
			ASSERT(lp->hwcap & CAP_TXPAUSE3);
			txpause = TXPAUSE_EN3;
		}
		DPRINTF(1, (CE_CONT, "!%s: %s TXPAUSE:%x -> %x",
		    dp->name, __func__, INL(dp, TXPAUSE), txpause));
		OUTL(dp, TXPAUSE, txpause);
		FLSHL(dp, TXPAUSE);
	}
#if DEBUG_LEVEL > 2
	nfo_dump_regs(dp, (void *)__func__);
#endif
	return (GEM_SUCCESS);
}

static int
nfo_get_stats(struct gem_dev *dp)
{
	return (GEM_SUCCESS);
}

/*
 * descriptor manipuration
 */
static int
nfo_tx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	struct txdesc32		*tdp;
	uint32_t		mark;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mss;
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;
	struct nfo_dev		*lp = dp->private;

#if DEBUG_LEVEL > 10
	cmn_err(CE_CONT,
	    "!%s: %s: time: %d, seqnum: %d, slot %d, frags: %d flags: %llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 2
	flags |= GEM_TXFLAG_INTR;
#endif
	/*
	 * write tx descriptor(s) in reversed order
	 */
	if (IS_DFMT1(lp->dmac)) {
		/* format 1 */
		mark = TxD_LAST_1 | TxD_OWN;
		if (flags & GEM_TXFLAG_INTR) {
			mark |= TxD_INT_1_NF4 | TxD_INT_1;
		}
	} else {
		/* format 2 or later */
		mark = TxD_LAST_2 | TxD_OWN;
		if (flags & GEM_TXFLAG_INTR) {
			mark |= TxD_INT_2;
		}
	}

	dcp = &dmacookie[frags - 1];
	for (i = frags - 1; i > 0; i--) {
		int	curslot;

		curslot = SLOT(slot + i, tx_ring_size);
		tdp = &TXDESC32(dp)[curslot];

		tdp->txd0 = dcp->dmac_address;
		tdp->txd1 = mark | (dcp->dmac_size - 1);

		mark &= TxD_OWN;
		dcp--;
	}

#ifdef CONFIG_CKSUM_OFFLOAD
	if (mss = ((flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT)) {
		mark |= (mss << TxD_MSS_SHIFT_2) | TxD_TSO_2;
	} else if (flags & GEM_TXFLAG_IPv4) {
		mark |= TxD_CKSUM_IPv4_2;
		if (flags & (GEM_TXFLAG_TCP | GEM_TXFLAG_UDP)) {
			mark |= TxD_CKSUM_TCPUDP_2;
		}
	}
#endif
	if (flags & GEM_TXFLAG_HEAD) {
		/* remove own bit for the first fragment */
		mark &= ~TxD_OWN;
	}
	tdp = &TXDESC32(dp)[slot];

	tdp->txd0 = dcp->dmac_address;
	tdp->txd1 = mark | (dcp->dmac_size - 1);

	return (frags);
}

static int
nfo_tx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			i;
	uint64_t		phys;
	uint32_t		vtag;
	uint32_t		mark;
	struct txdesc64		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mss;
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 10
if ((flags & GEM_TXFLAG_TCP) && frags == 1 && dmacookie[0].dmac_size <= 60) {
	cmn_err(CE_CONT,
	    "!%s: %s: time: %d, seqnum: %d, slot %d, frags: %d flags: %llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
#if DEBUG_LEVEL > 2
	flags |= GEM_TXFLAG_INTR;
#endif
	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TxD_LAST_2 | TxD_OWN;
	if (flags & GEM_TXFLAG_INTR) {
		mark |= TxD_INT_2;
	}

	dcp = &dmacookie[frags - 1];
	for (i = frags - 1; i > 0; i--) {
		phys = dcp->dmac_laddress;
		tdp = &TXDESC64(dp)[SLOT(slot + i, tx_ring_size)];
		tdp->txd0 = (uint32_t)(phys >> 32);
		tdp->txd1 = (uint32_t)phys;
		tdp->txd2 = 0;
		tdp->txd3 = mark | (dcp->dmac_size - 1);

		mark &= TxD_OWN;
		dcp--;
	}

#ifdef CONFIG_CKSUM_OFFLOAD
	if (mss = ((flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT)) {
		mark |= (mss << TxD_MSS_SHIFT_2) | TxD_TSO_2;
	} else if (flags & GEM_TXFLAG_IPv4) {
		mark |= TxD_CKSUM_IPv4_2;
		if (flags & (GEM_TXFLAG_TCP | GEM_TXFLAG_UDP)) {
			mark |= TxD_CKSUM_TCPUDP_2;
		}
	}
#endif
	if (flags & GEM_TXFLAG_HEAD) {
		/* remove own bit for the first fragment */
		mark &= ~TxD_OWN;
	}

	tdp = &TXDESC64(dp)[slot];

	phys = dcp->dmac_laddress;
	tdp->txd0 = (uint32_t)(phys >> 32);
	tdp->txd1 = (uint32_t)phys;

	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		vtag |= TxD_VLAN_EN;
	}
	tdp->txd2 = vtag;

	tdp->txd3 = mark | (dcp->dmac_size - 1);

	return (frags);
}

static void
nfo_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint32_t	own;
	struct nfo_dev	*lp = dp->private;

	own = TxD_OWN;

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	if (IS_DFMT3(lp->dmac)) {
		TXDESC64(dp)[start_slot].txd3 |= own;
	} else {
		TXDESC32(dp)[start_slot].txd1 |= own;
	}

	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/* kick dma engine for tx */
	OUTL(dp, DMACTRL, DMACTRL_TxPOLL | lp->dmac);
}

static uint_t
nfo_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct nfo_dev	*lp = dp->private;
	uint_t		ret = GEM_TX_DONE;
	uint32_t	status;
	uint_t		curslot;
	int		col;

	/* check tx status flag in the last fragment */
	curslot = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);

	if (IS_DFMT3(lp->dmac)) {
		/* format 3 (64bit addressing) */
		status = TXDESC64(dp)[curslot].txd3;
	} else {
		/* format 1 or 2 (32bit addressing) */
		status = TXDESC32(dp)[curslot].txd1;
	}

	if (status & TxD_OWN) {
		return (0);
	}

	DPRINTF(10, (CE_CONT,
	    "!%s: %s slot:%d+%d, status:%08x",
	    dp->name, __func__, slot, ndesc, status));

	if (IS_DFMT1(lp->dmac)) {
#ifdef DEBUG_LEVEL
		if (status &
		    ~(TxD_COL_1 |TxD_LAST_1 | TxD_DEF_1 | 0x010007ff)) {
			cmn_err(CE_CONT,
			    "!%s: %s status_1:%08x", dp->name, __func__,
			    status);
		}
#endif
		if (status & (TxD_ABORT_1 | TxD_CRS_1 |
		    TxD_OWC_1 | TxD_UDF_1 | TxD_ERR_1)) {
			dp->stats.errxmt++;
			if (status & TxD_UDF_1) {
				dp->stats.underflow++;
			} else if (status & TxD_CRS_1) {
				dp->stats.nocarrier++;
			} else if (status & TxD_OWC_1) {
				dp->stats.xmtlatecoll++;
			} else if (status & TxD_ABORT_1) {
				dp->stats.excoll++;
				dp->stats.collisions += 16;
			} else {
				dp->stats.xmit_internal_err++;
			}
		} else if (!dp->full_duplex) {
			col = status & TxD_COL_1;
			if (col) {
				col >>= TxD_COL_SHIFT_1;
				dp->stats.collisions += col;
				if (col == 1) {
					dp->stats.first_coll++;
				} else {
					dp->stats.multi_coll++;
				}
			} else if (status & TxD_DEF_1) {
				dp->stats.defer++;
			}
		}
	} else {
#ifdef NEVER
		if (status & ~(TxD_LAST_2 | TxD_COL_2 |
		    TxD_DEF_2 | TxD_TOK_2 | 0x3fff)) {
			cmn_err(CE_CONT,
			    "!%s: %s status_2:%08x", dp->name, __func__,
			    status);
		}
#endif
		if (status & (TxD_ABORT_2 | TxD_CRS_2 | TxD_OWC_2 |
		    TxD_UDF_2 | TxD_COL_2 | TxD_DEF_2)) {

			if (status & (TxD_ABORT_2 | TxD_CRS_2 |
			    TxD_OWC_2 | TxD_UDF_2)) {
				dp->stats.errxmt++;
				if (status & TxD_UDF_2) {
					dp->stats.underflow++;
				} else if (status & TxD_CRS_2) {
					dp->stats.nocarrier++;
				} else if (status & TxD_OWC_2) {
					dp->stats.xmtlatecoll++;
				} else if (status & TxD_ABORT_2) {
					dp->stats.excoll++;
					dp->stats.collisions += 16;
				} else {
					dp->stats.xmit_internal_err++;
				}
			} else if (!dp->full_duplex) {
				col = status & TxD_COL_2;
				if (col) {
					col >>= TxD_COL_SHIFT_2;
					dp->stats.collisions += col;
					if (col == 1) {
						dp->stats.first_coll++;
					} else {
						dp->stats.multi_coll++;
					}
				} else if (status & TxD_DEF_2) {
					dp->stats.defer++;
				}
			}
		}
	}
	return (ret);
}

static void
nfo_rx_desc_write32(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rxdesc32	*rdp;

#if DEBUG_LEVEL > 2
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
	/* for the last slot of the packet */
	rdp = &RXDESC32(dp)[slot];

	rdp->rxd0 = dmacookie->dmac_address;
	rdp->rxd1 = RxD_OWN |
	    (dp->mtu + sizeof (struct ether_header) + ETHERFCSL + 8 - 1);
}

static void
nfo_rx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rxdesc64	*rdp;
#if DEBUG_LEVEL > 1
	int		i;

	ASSERT(frags == 1);

	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/* for the last slot of the packet */
	rdp = &RXDESC64(dp)[slot];

	rdp->rxd0 = (uint32_t)(dmacookie->dmac_laddress >> 32);
	rdp->rxd1 = (uint32_t)dmacookie->dmac_laddress;
	rdp->rxd2 = 0;
	rdp->rxd3 = RxD_OWN |
	    (dp->mtu + sizeof (struct ether_header) + ETHERFCSL + 8 - 1);
}

#ifdef DEBUG_LEVEL
static void
nfo_packet_dump(struct gem_dev *dp, uint8_t *bp)
{
	cmn_err(CE_CONT, "!%s: "
	    "%02x %02x %02x %02x %02x %02x %02x %02x "
	    "%02x %02x %02x %02x %02x %02x",
	    dp->name,
	    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7],
	    bp[8], bp[9], bp[10], bp[11], bp[12], bp[13]);
}

static void
nfo_tx_desc_dump32(struct gem_dev *dp)
{
	int		i;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct txdesc32	*tdp;

	cmn_err(CE_CONT,
	    "!%s: tx descriptor dump: current head:%d[%d], tail:%d[%d]",
	    dp->name,
	    dp->tx_desc_head, (int)SLOT(dp->tx_desc_head, tx_ring_size),
	    dp->tx_desc_tail, (int)SLOT(dp->tx_desc_tail, tx_ring_size));

	tdp = &TXDESC32(dp)[0];
	for (i = 0; i < tx_ring_size; i++, tdp++) {
		cmn_err(CE_CONT, "%d: %08x %08x",
		    i,
		    LE_32(tdp->txd0), LE_32(tdp->txd1));
	}
}

static void
nfo_tx_desc_dump64(struct gem_dev *dp)
{
	int		i;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct txdesc64	*tdp;

	cmn_err(CE_CONT,
	    "!%s: tx descriptor dump: current head:%d[%d], tail:%d[%d]",
	    dp->name,
	    dp->tx_desc_head, (int)SLOT(dp->tx_desc_head, tx_ring_size),
	    dp->tx_desc_tail, (int)SLOT(dp->tx_desc_tail, tx_ring_size));

	tdp = &TXDESC64(dp)[0];
	for (i = 0; i < tx_ring_size; i++, tdp++) {
		cmn_err(CE_CONT, "%d: %08x.%08x %x %x",
		    i,
		    LE_32(tdp->txd0),
		    LE_32(tdp->txd1),
		    LE_32(tdp->txd2),
		    LE_32(tdp->txd3));
	}
}
#endif /* DEBUG_LEVEL */
static uint64_t
nfo_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	status;
	uint64_t	vlan;
	uint64_t	ret;
	struct nfo_dev	*lp = dp->private;

	if (IS_DFMT3(lp->dmac)) {
		/* format 3 (64bit addressing) */
		status = RXDESC64(dp)[slot].rxd3;
	} else {
		/* format 1 or 2 (32bit addressing) */
		status = RXDESC32(dp)[slot].rxd1;
	}

	if (lp->hwcap & CAP_VLAN) {
		vlan = RXDESC64(dp)[slot].rxd1;
	} else {
		vlan = 0;
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: time:%d %s: slot:%d, status:0x%b, vlan:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, status,
	    (IS_DFMT1(lp->dmac) ? RxD_1_BITS : RxD_2_BITS), vlan));

	if ((status & (RxD_OWN | RxD_ERR)) == 0) {
		/*
		 * this packet was received without errors
		 */
#if DEBUG_LEVEL > 4
		nfo_packet_dump(dp, dp->rx_buf_head->rxb_buf);
#endif
		ret = GEM_RX_DONE;

		/* check result of hw checksum */
		if ((lp->hwcap & CAP_CKSUM) && !IS_DFMT1(lp->dmac)) {
			switch ((status >> RxD_CKSUMP_SHIFT_2) & CKSUMP_2) {
			case CKSUMP_TCP_2:
				ret |= GEM_RX_CKSUM_TCP | GEM_RX_CKSUM_IPv4;
				break;

			case CKSUMP_UDP_2:
				ret |= GEM_RX_CKSUM_UDP | GEM_RX_CKSUM_IPv4;
				break;

			case CKSUMP_IP_2:
				ret |= GEM_RX_CKSUM_IPv4;
				break;

			case CKSUMP_NONE_2:
				break;
#ifdef DEBUG_LEVEL
			default:
				cmn_err(CE_CONT,
				    "%s: %s: cksum error cksump:%d",
				    dp->name, __func__,
				    (status >> RxD_CKSUMP_SHIFT_2) &
				    CKSUMP_2);
				ret |= GEM_RX_CKSUM_ERR;
				break;
#endif
			}
		}

		if (vlan & RxD_VLAN) {
			/* extract stripped vlan tag */
			ret |= (vlan & RxD_VTAG) << GEM_RX_VTAG_SHIFT;
		}

		return (ret | (status & RxD_LEN & GEM_RX_LEN));
	}

	if (status & RxD_OWN) {
		/*
		 * No more received packets because
		 * this buffer is owned by NIC.
		 */
		return (0);
	}

	/*
	 * Packet with error received
	 */
	if (IS_DFMT1(lp->dmac)) {
		DPRINTF(0, (CE_CONT,
		    "!%s: Corrupted packet received, buffer status: %b",
		    dp->name, status, RxD_1_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;
		if (status & RxD_OVF_1) {
			dp->stats.overflow++;
		} else if (status & RxD_MISSED_1) {
			dp->stats.missed++;
		} else if (status & RxD_CRC_1) {
			dp->stats.crc++;
		} else if (status & RxD_FRAME_1) {
			dp->stats.frame++;
		} else {
			dp->stats.rcv_internal_err++;
		}
	} else {
		DPRINTF(0, (CE_CONT,
		    "!%s: Corrupted packet received, buffer status: %b",
		    dp->name, status, RxD_2_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;
		if (status & RxD_OVF_2) {
			dp->stats.overflow++;
		} else if (status & RxD_CRC_2) {
			dp->stats.crc++;
		} else if (status & RxD_FRAME_2) {
			dp->stats.frame++;
		} else {
			dp->stats.rcv_internal_err++;
		}
	}

	return (GEM_RX_ERR);
}

static void
nfo_tx_desc_init32(struct gem_dev *dp, int slot)
{
	TXDESC32(dp)[slot].txd1 = 0;
}

static void
nfo_tx_desc_init64(struct gem_dev *dp, int slot)
{
	TXDESC64(dp)[slot].txd3 = 0;
}

static void
nfo_rx_desc_init32(struct gem_dev *dp, int slot)
{
	RXDESC32(dp)[slot].rxd1 = 0;
}

static void
nfo_rx_desc_init64(struct gem_dev *dp, int slot)
{
	RXDESC64(dp)[slot].rxd3 = 0;
}

/*
 * Device depend interrupt handler
 */
static uint_t
nfo_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		restart_tx = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct nfo_dev	*lp = dp->private;

	isr = INL(dp, ISR);

	DPRINTF(2, (CE_CONT, "!%s: %s: time:%d isr:%b",
	    dp->name, __func__, ddi_get_lbolt(), isr, INTR_BITS));

	if ((isr & lp->imr) == 0) {
		/* not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(3, (CE_CONT, "!%s: %s: time:%d isr:%b imr:%b",
	    dp->name, __func__, ddi_get_lbolt(), isr, INTR_BITS,
	    INL(dp, IMR), INTR_BITS));

	if (!IS_MAC_ONLINE(dp)) {
		/* inhibit further interrupts */
		lp->imr = 0;

		/* ack to all interrupts */
		OUTL(dp, MIIISR, MIIISR_LSC);
		FLSHL(dp, MIIISR);
		OUTL(dp, ISR, INTR_ALL);
		FLSHL(dp, ISR);
		return (DDI_INTR_CLAIMED);
	}

	/*
	 * First of all, we must clear the reason of LSC, link status
	 * changed, interrupts to avoid bogus LSC interrupts.
	 */
	if (isr & INTR_LSC) {
		/*
		 * Link or PHY status has changed
		 */
		DPRINTF(0, (CE_CONT, "!%s: isr:%b, miistat:%b",
		    dp->name, isr, INTR_BITS,
		    INL(dp, MIIISR), MIIISR_BITS));

		OUTL(dp, MIIISR, MIIISR_LSC);
		FLSHL(dp, MIIISR);
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			restart_tx |= INTR_RESTART_TX;
		}
#endif
	}

	/* clear interrupt sources explicitly */
	OUTL(dp, ISR, isr);

#ifdef CONFIG_POLLING
	isr &= lp->imr | INTR_RXOK;

	if (dp->poll_interval != lp->last_poll_interval) {
		uint_t	rate;

		rate = dp->poll_interval/10300;
		if (rate) {
			/* polling mode */
			lp->imr &= ~INTR_RXOK;
			lp->imr |= INTR_TIMER;
		} else {
			/* normal mode */
			lp->imr |= INTR_RXOK;
			lp->imr &= ~INTR_TIMER;
		}
		OUTL(dp, TIMER, rate);

		lp->last_poll_interval = dp->poll_interval;

		if ((dp->misc_flag & GEM_NOINTR) == 0) {
			OUTL(dp, IMR, lp->imr);
		}
	}
#else
	isr &= lp->imr;
#endif /* CONFIG_POLLING */

	/* barrier for interrupt state */
	FLSHL(dp, ISR);

	if (isr & INTR_RXERR_FATAL) {
		cmn_err(CE_NOTE, "!%s: Rx fatal error, isr:%b",
		    dp->name, isr, INTR_BITS);
		need_to_reset = B_TRUE;
	}

#ifdef WA_TXINTR	/* withdrawn */
	/* XXX - Why do we also need to check rx interrupt bits for tx ? */
#define	NFO_TX_INTRS	\
	(INTR_TXOK | INTR_TXERR | INTR_TXDESC | INTR_RXOK | INTR_RXERR)
#else
#define	NFO_TX_INTRS	(INTR_TXOK | INTR_TXERR | INTR_TXDESC)
#endif
#ifdef GEM3
	if (isr & NFO_TX_INTRS) {
		gem_tx_done(dp);
#if DEBUG_LEVEL > 2
		if (isr & INTR_TXERR) {
			cmn_err(CE_WARN, "!%s: Tx error, isr:%b",
			    dp->name, isr, INTR_BITS);
		}
#endif
	}
#endif /* GEM3 */

	if (isr & (INTR_RXOK | INTR_RXDESC | INTR_RXERR | INTR_RDU)) {
		(void) gem_receive(dp);
		if (isr & INTR_RDU) {
			dp->stats.norcvbuf++;
		}
	}

#ifndef GEM3
	if (isr & NFO_TX_INTRS) {
		if (gem_tx_done(dp)) {
			/* Need to kick potentially blocked down stream */
			restart_tx |= INTR_RESTART_TX;
		}
#if DEBUG_LEVEL > 2
		if (isr & INTR_TXERR) {
			cmn_err(CE_WARN, "!%s: Tx error, isr:%b",
			    dp->name, isr, INTR_BITS);
		}
#endif
	}
#endif /* !GEM3 */

	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);

		restart_tx |= INTR_RESTART_TX;
	}

	return (DDI_INTR_CLAIMED | restart_tx);
}

/*
 * HW depend MII routines
 */
static void
nfo_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
nfo_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint32_t	val;
	uint32_t	stat;

#if DEBUG_LEVEL > 5
	if (INL(dp, MIICTL) & MIICTL_BUSY) {
		DPRINTF(0, (CE_NOTE,
		    "!%s: %s: miictl is busy", dp->name, __func__));
		OUTL(dp, MIICTL, MIICTL_BUSY);
		FLSHL(dp, MIICTL);
		drv_usecwait(10);
	}
#endif
	OUTL(dp, MIICTL,
	    (dp->mii_phy_addr << MIICTL_ADDR_SHIFT) | (uint32_t)reg);
	FLSHL(dp, MIICTL);
	drv_usecwait(20);

	for (i = 0; (INL(dp, MIICTL) & MIICTL_BUSY); i++) {
		if (i > 100) {
			/* time out */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);

			/* force to stop MII control */
			OUTL(dp, MIICTL, MIICTL_BUSY);
			FLSHL(dp, MIICTL);
			drv_usecwait(10);

			/* prepare to ack to MIIISR */
			stat = MIIISR_DONE | MIIISR_ERROR;
			goto x;
		}
		drv_usecwait(10);
	}

	/* XXX -  we should read MIIISR after MIICTL_BUSY is reset */
	stat = INL(dp, MIIISR);
	DPRINTF(5, (CE_CONT, "!%s: %s: MIIISR:%b (after reading)",
	    dp->name, __func__, stat, MIIISR_BITS));

x:
	if ((stat & MIIISR_ERROR) == 0) {
		val = INL(dp, MIIDATA);
	} else {
		if (dp->mii_state != MII_STATE_UNKNOWN) {
			/* suppress error messages under probing phys */
			cmn_err(CE_WARN,
			    "!%s: %s: error, phy addr:%d, MIIISR:%b",
			    dp->name, __func__,
			    dp->mii_phy_addr, stat, MIIISR_BITS);
		}
		val = 0;
	}

	/* ack to MII status */
	/* XXX - this enables the mii interface again. */
	OUTL(dp, MIIISR, stat);

	return ((uint16_t)val);
}

static void
nfo_mii_write_raw(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int		i;

#if DEBUG_LEVEL > 5
	if (INL(dp, MIICTL) & MIICTL_BUSY) {
		DPRINTF(0, (CE_WARN,
		    "!%s: %s: miictl is busy", dp->name, __func__));
		OUTL(dp, MIICTL, MIICTL_BUSY);
		FLSHL(dp, MIICTL);
		drv_usecwait(10);
	}
#endif
	OUTL(dp, MIIDATA, val);

	OUTL(dp, MIICTL,
	    MIICTL_WRITE |
	    (((uint32_t)dp->mii_phy_addr) << MIICTL_ADDR_SHIFT) |
	    (uint32_t)index);
	FLSHL(dp, MIICTL);
	drv_usecwait(20);

	for (i = 0; (INL(dp, MIICTL) & MIICTL_BUSY); i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);

			/* force to stop MII control */
			OUTL(dp, MIICTL, MIICTL_BUSY);
			FLSHL(dp, MIICTL);
			drv_usecwait(10);

			break;
		}
		drv_usecwait(10);
	}

#ifdef NEVER
	/* ack to MII status */
	/* XXX - no need to ack MIIISR for writing */
	OUTL(dp, MIIISR, INL(dp, MIIISR));
#endif
	DPRINTF(5, (CE_CONT, "!%s: %s: MIIISR:%b (after writing)",
	    dp->name, __func__, INL(dp, MIIISR), MIIISR_BITS));
}

static void
nfo_mii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int		i;

	if (IS_PHY(dp, OUI_MARVELL, MODEL_MARVELL_E3016) &&
	    index == MII_CONTROL &&
	    (val & MII_CONTROL_ANE) && (val & MII_CONTROL_RSAN)) {
		val |= MII_CONTROL_RESET;
	}

	nfo_mii_write_raw(dp, index, val);
}

static int
nfo_mii_config(struct gem_dev *dp)
{
	int		i;
	int		ret;
	uint32_t	val;
	struct nfo_dev	*lp = dp->private;

	if (OUI_CMP(dp, OUI_MARVELL) == 0) {
		/* restore LED stats */
		nfo_mii_write_raw(dp, 22, 3);
		drv_usecwait(5);
		for (i = 0; i < 3; i++) {
			nfo_mii_write_raw(dp, 16 + i, lp->mii_led_stats[i]);
		}
		nfo_mii_write_raw(dp, 22, 0);
		drv_usecwait(5);
	}

	if ((ret = gem_mii_config_default(dp)) != GEM_SUCCESS) {
		return (ret);
	}

	if (OUI_CMP(dp, OUI_CICADA) == 0) {
		cmn_err(CE_CONT,
		    "!%s: %s: CICADA phy (model:%x ver:%x) found",
		    dp->name, __func__,
		    (dp->mii_phy_id >> 4) & 0x3f, dp->mii_phy_id & 0xf);

		val = INL(dp, PHYIF);
		if (val & PHYIF_RGMII) {
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: RGMII phy found, PHYIF:%04x",
			    dp->name, __func__, val));
			nfo_mii_write_raw(dp, 0x17,
			    (nfo_mii_read(dp, 0x17) & ~(0xfe00)) | 0x1200);

			nfo_mii_write_raw(dp, 0x1c,
			    nfo_mii_read(dp, 0x1c) | 0x0004);
		}

		nfo_mii_write_raw(dp, 0x16, nfo_mii_read(dp, 0x16) | 0x2000);
	}

	if (OUI_CMP(dp, OUI_VITESSE) == 0) {
		cmn_err(CE_CONT,
		    "!%s: %s: VITESSE phy (model:%x ver:%x) found",
		    dp->name, __func__,
		    (dp->mii_phy_id >> 4) & 0x3f, dp->mii_phy_id & 0xf);

		nfo_mii_write_raw(dp, 0x1f, 0x52b5);
		nfo_mii_write_raw(dp, 0x10, 0xaf8a);
		nfo_mii_write_raw(dp, 0x12, nfo_mii_read(dp, 0x12));
		nfo_mii_write_raw(dp, 0x11,
		    (nfo_mii_read(dp, 0x11) & ~0xc) | 0x8);
		nfo_mii_write_raw(dp, 0x10, 0x8f8a);
		nfo_mii_write_raw(dp, 0x10, 0xaf86);
		nfo_mii_write_raw(dp, 0x12,
		    (nfo_mii_read(dp, 0x12) & ~0xc) | 0x8);
		nfo_mii_write_raw(dp, 0x11, nfo_mii_read(dp, 0x11));
		nfo_mii_write_raw(dp, 0x10, 0x8f86);
		nfo_mii_write_raw(dp, 0x10, 0xaf82);
		nfo_mii_write_raw(dp, 0x12, nfo_mii_read(dp, 0x12));
		nfo_mii_write_raw(dp, 0x11,
		    (nfo_mii_read(dp, 0x11) & ~0x0180) | 0x0100);
		nfo_mii_write_raw(dp, 0x10, 0x8f82);
		nfo_mii_write_raw(dp, 0x1f, 0x0);
	}

	if (OUI_CMP(dp, OUI_REALTEK) == 0) {
		cmn_err(CE_CONT, "!%s: %s:"
		    " REALTEK phy (model:%x ver:%x) found",
		    dp->name, __func__,
		    (dp->mii_phy_id >> 4) & 0x3f, dp->mii_phy_id & 0xf);

		nfo_mii_write_raw(dp, 0x1f, 0x0000);
		nfo_mii_write_raw(dp, 0x19, 0x8e00);
		nfo_mii_write_raw(dp, 0x1f, 0x0001);
		nfo_mii_write_raw(dp, 0x13, 0xad17);
		nfo_mii_write_raw(dp, 0x1f, 0x0000);
	}

	return (ret);
}

static boolean_t
nfo_mgmt_acq_sema(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;

	i = 0;
	while ((INL(dp, TxCTRL) & TxCTRL_MSEMA) != TxCTRL_MSEMA_FREE) {
		if (i++ > 10) {
			cmn_err(CE_WARN,
			    "!%s: %s: failed to acquire msema:"
			    " msema didn't become free",
			    dp->name, __func__);
			return (B_FALSE);
		}
		delay(drv_usectohz(500*1000));
	}

	for (i = 0; i < 2; i++) {
		/* try to acquire semaphore */
		OUTL(dp, TxCTRL, INL(dp, TxCTRL) | TxCTRL_HSEMA_ACQ);

		/* verify that semaphore was acquired */
		val = INL(dp, TxCTRL);
		if ((val & TxCTRL_HSEMA) == TxCTRL_HSEMA_ACQ &&
		    (val & TxCTRL_MSEMA) == TxCTRL_MSEMA_FREE) {
			/* we got !! */
			return (B_TRUE);
		}
		drv_usecwait(50);
	}

	cmn_err(CE_NOTE, "!%s: %s: failed to acuire msema: time over",
	    dp->name, __func__);

	return (B_FALSE);
}

static int
nfo_mii_probe(struct gem_dev *dp)
{
	int		i;
	int		ret;
	uint32_t	miiauto;
	struct nfo_dev	*lp = dp->private;

	/* clear phy state and temporarily halt phy interrupts */
	OUTL(dp, MIIIMR, 0);
	FLSHL(dp, MIIIMR);
	miiauto = INL(dp, MIIAUTO);
	if (miiauto & MIIAUTO_EN) {
		OUTL(dp, MIIAUTO, miiauto & ~MIIAUTO_EN);
	}
	OUTL(dp, MIIISR, MIIISR_ALL);
	FLSHL(dp, MIIISR);

	if ((lp->hwcap & CAP_MGMT) &&
	    (INL(dp, TxCTRL) & TxCTRL_SYNC_PHYINIT)) {
		lp->mac_in_use = (INL(dp, TxCTRL) & TxCTRL_MGMT_ST) != 0;

		DPRINTF(0, (CE_CONT, "!%s: %s: management is active: %d",
		    dp->name, __func__, lp->mac_in_use));

		if (nfo_mgmt_acq_sema(dp)) {
			lp->phy_initialized =
			    (INL(dp, TxCTRL) & TxCTRL_SYNC)
			    == TxCTRL_SYNC_PHYINIT;
		}
	}

	ret = gem_mii_probe_default(dp);

	if (ret != GEM_SUCCESS) {
		/* restore MIIAUTO */
		if (miiauto & MIIAUTO_EN) {
			OUTL(dp, MIIAUTO, miiauto);
		}
	}

	if (((dp->mii_phy_id >> 10) & 0xffff) == OUI_REALTEK_2) {
		dp->mii_phy_id = OUI_REALTEK << 10;
	}

	/* 1000Mbps half duplex isn't supported */
	dp->mii_xstatus &= ~(MII_XSTATUS_1000BASEX | MII_XSTATUS_1000BASET);
	dp->mii_xstatus_ro = ~dp->mii_xstatus;

	return (ret);
}

static int
nfo_mii_init(struct gem_dev *dp)
{
	int		i;
	int		ret = GEM_SUCCESS;
	uint32_t	val;
	struct nfo_dev	*lp = dp->private;

	/* setup mii interface */
	OUTL(dp, MIIISR, MIIISR_ALL);
	OUTL(dp, MIIAUTO,
	    (dp->mii_phy_addr << MIIAUTO_PHYADDR_SHIFT) | MIIAUTO_PHYVALID);
	OUTL(dp, MIIAUTO, INL(dp, MIIAUTO) | MIIAUTO_EN);

	/* tune mii interface speed */
	OUTL(dp, MIICFG, 0x105);

	if (!lp->phy_initialized) {
		if (IS_PHY(dp, OUI_MARVELL, MODEL_MARVELL_E3016)) {
			/* XXX -- workaround for 88E3016 */
			cmn_err(CE_CONT, "!%s: %s:"
			    " MARVELL 88E3016 phy (model:%x ver:%x) found",
			    dp->name, __func__,
			    (dp->mii_phy_id >> 4) & 0x3f,
			    dp->mii_phy_id & 0xf);
			nfo_mii_write_raw(dp, 0x1c,
			    nfo_mii_read(dp, 0x1c) & ~0x0300);
		}

		if (IS_PHY_REV(dp, OUI_REALTEK,
		    MODEL_REALTEK_8211, REV_REALTEK_8211B)) {
			nfo_mii_write_raw(dp, 0x1f, 0x0000);
			nfo_mii_write_raw(dp, 0x19, 0x8e00);
			nfo_mii_write_raw(dp, 0x1f, 0x0001);
			nfo_mii_write_raw(dp, 0x13, 0xad17);
			nfo_mii_write_raw(dp, 0x14, 0xfb54);
			nfo_mii_write_raw(dp, 0x1f, 0x0000);
		}

		if (IS_PHY_REV(dp, OUI_REALTEK,
		    MODEL_REALTEK_8211, REV_REALTEK_8211C)) {
			val = INL(dp, PWRSTAT2) & ~PWRSTAT2_PHY_RESET;
			OUTL(dp, PWRSTAT2, val | PWRSTAT2_PHY_RESET);
			delay(drv_usectohz(25000));
			OUTL(dp, PWRSTAT2, val);
			delay(drv_usectohz(25000));

			nfo_mii_write_raw(dp, 0x11,
			    nfo_mii_read(dp, 0x11) | 0x0008);
			nfo_mii_write_raw(dp, 0x1f, 0x0005);

			val = nfo_mii_read(dp, 0x01);
			if ((val & 0x0200) == 0) {
				nfo_mii_write(dp, 0x01, val | 0x0200);
			}
			nfo_mii_write_raw(dp, 0x1f, 0x0000);
		}

		if (IS_PHY(dp, OUI_REALTEK, MODEL_REALTEK_8201)) {
			if (lp->hwcap & CAP_PHY_WA) {
				nfo_mii_write_raw(dp, 0x11,
				    nfo_mii_read(dp, 0x11) | 0x1000);
			}
		}
	} else {
		/* don't reset phy setting by management unit */
		dp->gc.gc_mii_dont_reset = B_TRUE;
	}

	if (lp->mac_in_use) {
		/* XXX - indicate driverloaded flag */
		OUTL(dp, TxCTRL, INL(dp, TxCTRL) | TxCTRL_HSEMA_LOADED);
	}

	/* save LED status */
	nfo_mii_write_raw(dp, 22, 3);
	drv_usecwait(5);

	if (OUI_CMP(dp, OUI_MARVELL) == 0) {
		for (i = 0; i < 3; i++) {
			lp->mii_led_stats[i] = nfo_mii_read(dp, 16 + i);
		}

		DPRINTF(0, (CE_CONT, "!%s: %s: mii_led_stats:%x, %x, %x",
		    dp->name, __func__,
		    lp->mii_led_stats[0],
		    lp->mii_led_stats[1],
		    lp->mii_led_stats[2]));
	}
	nfo_mii_write_raw(dp, 22, 0);
	drv_usecwait(5);

	return (ret);
}

static void
nfo_power_up(struct gem_dev *dp)
{
	uint32_t	val;
	struct nfo_dev	*lp = dp->private;

	/* disable wol */
	OUTL(dp, WAKECTRL, 0);

	if (lp->hwcap & CAP_PM) {
		val = INL(dp, PWRSTAT2) & ~0x0f11;
		if ((lp->chip->devid == 0x0268 ||
		    lp->chip->devid == 0x0269) && lp->revid >= 0xa3) {
			/* workaround for MCP51 */
			val |= 1;
		}
		OUTL(dp, PWRSTAT2, val);
	}
}

static void
nfo_fixup_params(struct gem_dev *dp)
{
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* fix rx buffer length, it needs additional 48 + 64 bytes. */
	dp->rx_buf_len += 48 + 64;
}

static int
nfo_attach_chip(struct gem_dev *dp)
{
	uint32_t	val;
	uint32_t	*mac_addr;
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	nfo_power_up(dp);

	/* Save factory mac address */
	lp->factory_mac_addr[0] = INL(dp, MACADDR + 0);
	lp->factory_mac_addr[1] = INL(dp, MACADDR + 4);

	/*
	 * XXX - As the factory mac address is set in incorrect format,
	 * aka in reversed order after power-on, we must re-write it.
	 * But as there are no copies of the original mac address anywhere
	 * else, it is gone after we fix it.
	 */
	mac_addr = lp->factory_mac_addr;
	val = INL(dp, TxPOLL);
	DPRINTF(0, (CE_CONT, "!%s: %s: TxPOLL:%x", dp->name, __func__, val));

	if ((lp->chip->hwcap & CAP_CORRECT_MAC) == 0) {
		/* original format */
		dp->dev_addr.ether_addr_octet[0] = mac_addr[1] >> (1*8);
		dp->dev_addr.ether_addr_octet[1] = mac_addr[1] >> (0*8);
		dp->dev_addr.ether_addr_octet[2] = mac_addr[0] >> (3*8);
		dp->dev_addr.ether_addr_octet[3] = mac_addr[0] >> (2*8);
		dp->dev_addr.ether_addr_octet[4] = mac_addr[0] >> (1*8);
		dp->dev_addr.ether_addr_octet[5] = mac_addr[0] >> (0*8);
	} else {
		/* mac address format has been fixed */
		dp->dev_addr.ether_addr_octet[0] = mac_addr[0] >> (0*8);
		dp->dev_addr.ether_addr_octet[1] = mac_addr[0] >> (1*8);
		dp->dev_addr.ether_addr_octet[2] = mac_addr[0] >> (2*8);
		dp->dev_addr.ether_addr_octet[3] = mac_addr[0] >> (3*8);
		dp->dev_addr.ether_addr_octet[4] = mac_addr[1] >> (0*8);
		dp->dev_addr.ether_addr_octet[5] = mac_addr[1] >> (1*8);
	}

	dp->misc_flag |= GEM_POLL_RXONLY;
#ifdef CONFIG_CKSUM_OFFLOAD
	if ((lp->hwcap & CAP_CKSUM) && !IS_DFMT1(lp->dmac)) {
		dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4;
#ifdef CONFIG_LSO
		if ((lp->hwcap & CAP_WA_NO_LSO) == 0) {
			dp->misc_flag |= GEM_LSO;
		}
#endif /* CONFIG_LSO */
	}
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef CONFIG_VLAN_HW
	if (lp->hwcap & CAP_VLAN) {
		dp->misc_flag |= GEM_VLAN_HARD;
	} else {
		dp->misc_flag |= GEM_VLAN_SOFT;
	}
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif /* CONFIG_VLAN_HW */

#if DEBUG_LEVEL > 4
{
	uint8_t		*mac;

	mac = dp->dev_addr.ether_addr_octet;
	cmn_err(CE_CONT, "!%s: mac:%02x:%02x:%02x:%02x:%02x:%02x",
	    dp->name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
#endif
	if (IS_PHY(dp, OUI_MARVELL, 0x220)) {
		/* workaround for Marvell 88E3016 100/10 phy */
		dp->gc.gc_mii_linkdown_action = MII_ACTION_RESET;
	}

#ifndef GEM_CONFIG_JUMBO_FRAME
	nfo_fixup_params(dp);
#endif

	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
nfoattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_acc_handle_t	conf_handle;
	int			ret;
	uint16_t		vid;
	uint16_t		did;
	uint8_t			revid;
	int			unit;
	struct chip_info	*p;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct nfo_dev	*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint32_t		ilr;
	boolean_t		is64bit;
	boolean_t		tx16;
	uint_t			max_ring_size;
	uint_t			tx_ring_size;
#ifdef NEVER
	uint_t			pcie_cap;
#endif

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(2, (CE_CONT, "!%s%d: nfoattach: called at time:%d",
	    drv_name, unit, ddi_get_lbolt()));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
#ifdef TEST_NF3
	did = 0xd6;
#endif
	if (did == 0) {
		/* workaround for MCP compatible chipset */
		did = 0x1c3;
	}
	for (i = 0, p = nfo_chiptbl; i < ARRAY_SIZE(nfo_chiptbl); i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			cmn_err(CE_CONT,
			    "!%s%d: %s (vid: 0x%04x, did: 0x%04x,"
			    " revid: 0x%02x)",
			    drv_name, unit, p->name, vid, did, revid);
			goto chip_found;
		}
	}
	/* Not found */
	cmn_err(CE_WARN,
	    "!%s: %s: unsupported  PCI venid/devid (0x%x, 0x%x)",
	    drv_name, __func__, vid, did);
	pci_config_teardown(&conf_handle);
	return (DDI_FAILURE);

chip_found:
	/* ensure we can access the registers through IO and MEM space. */
	ret = pci_config_get16(conf_handle, PCI_CONF_COMM);
	ret |= PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE;
	pci_config_put16(conf_handle, PCI_CONF_COMM, ret);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));
#ifdef lint
	ilr = ilr;
#endif
#ifdef NEVER
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
#ifdef NEVER
		/*
		 * increase max read request 2:512, 3:1024, 4:2048
		 */
		if ((devcsr & (7 << 12)) < (3 << 12)) {
			devcsr = (devcsr & ~(7 << 12)) | (3 << 12);
			pci_config_put32(conf_handle,
			    pcie_cap + 8, devcsr);
		}
#endif
	}
#endif
	pci_config_teardown(&conf_handle);
	conf_handle = NULL;

	switch (cmd) {
	case DDI_ATTACH:
		/*
		 * Map the device registers in the kernel virtual space.
		 */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_MEM32, PCI_ADDR_MASK, &nfo_dev_attr,
		    (caddr_t *)&base, &regs_ha) != DDI_SUCCESS) {

			return (DDI_FAILURE);
		}

		/*
		 * construct gem configuration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);
		lp = kmem_zalloc(sizeof (struct nfo_dev), KM_SLEEP);

		lp->chip = p;
		lp->hwcap = p->hwcap;

		/* name */
		(void) sprintf(gcp->gc_name, "%s%d", drv_name, unit);

#if !defined(TEST_FMT1) && !defined(TEST_FMT2)
		is64bit = ((p->hwcap & CAP_64BIT) != 0);
#else
		is64bit = B_FALSE;
#endif
		/* tx ring depends on descriptor format */
		if (is64bit) {
			/* format 3 */
			max_ring_size = 16*1024;
#ifndef TEST_FMT1
		} else if (lp->hwcap & CAP_JUMBO) {
			/* format 2 */
			max_ring_size = 16*1024;
#endif
		} else {
			/* format 1 */
			max_ring_size = 1024;
		}
#ifdef GEM_CONFIG_TX_DIRECT
#define	MAXTXFRAGS	(min(GEM_MAXTXFRAGS, 16))
#else
#define	MAXTXFRAGS	1
#endif
		tx_ring_size = TX_BUF_SIZE * max(MAXTXFRAGS / 2, 1);
		tx_ring_size = max(min(tx_ring_size, max_ring_size), 64);

		/* workaround for tx hang */
		tx16 = B_FALSE;
		if ((lp->hwcap & CAP_WA_TX16) ||
		    ((lp->hwcap & CAP_WA_TX16_2) && revid < 0xa2)) {
			tx16 = B_TRUE;
		}

		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = is64bit ? 4 : 3;
#ifdef GEM_CONFIG_TX_DIRECT
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
#else
		gcp->gc_tx_buf_size = tx_ring_size;
#endif
		gcp->gc_tx_buf_limit = tx16 ? 16 : gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = tx_ring_size;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = nfo_tx_copy_thresh;
#ifdef WA_SHORT_FRAG /* withdrawn */
		gcp->gc_tx_min_fraglen = 64;
#endif

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = is64bit ? 4 : 3;
		gcp->gc_rx_ring_size = min(RX_BUF_SIZE, max_ring_size);
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size;
		gcp->gc_rx_copy_thresh = nfo_rx_copy_thresh;
		gcp->gc_rx_header_len = 0;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = nfo_dev_attr;
		gcp->gc_buf_attr = nfo_buf_attr;
		gcp->gc_desc_attr = nfo_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = nfo_dma_attr_desc;
		if (is64bit) {
			gcp->gc_dma_attr_txbuf = nfo_dma_attr_buf64;
		} else {
			gcp->gc_dma_attr_txbuf = nfo_dma_attr_buf32;
		}
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen =
		    gcp->gc_tx_max_frags;

		if (is64bit) {
			gcp->gc_dma_attr_rxbuf = nfo_dma_attr_buf64;
		} else {
			gcp->gc_dma_attr_rxbuf = nfo_dma_attr_buf32;
		}
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen =
		    gcp->gc_rx_max_frags;

		/* timeout parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/10;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* workaround for PHY */
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_an_oneshot = B_FALSE;
		gcp->gc_mii_hw_link_detection = B_TRUE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &nfo_attach_chip;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &nfo_fixup_params;
#endif
		gcp->gc_reset_chip = &nfo_reset_chip;
		gcp->gc_init_chip = &nfo_init_chip;
		gcp->gc_start_chip = &nfo_start_chip;
		gcp->gc_stop_chip = &nfo_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &nfo_set_rx_filter;
		gcp->gc_set_media = &nfo_set_media;
		gcp->gc_get_stats = &nfo_get_stats;
		gcp->gc_interrupt = &nfo_interrupt;

		/* descriptor operation */
		gcp->gc_tx_start = &nfo_tx_start;
		gcp->gc_rx_start = NULL;

		lp->dmac = 0;	/* was DMACTRL_TxRCMP_EN */

		if (!is64bit) {
			/* format 1 or 2 */
			gcp->gc_tx_desc_write = &nfo_tx_desc_write32;
			gcp->gc_rx_desc_write = &nfo_rx_desc_write32;
			gcp->gc_tx_desc_stat = &nfo_tx_desc_stat;
			gcp->gc_rx_desc_stat = &nfo_rx_desc_stat;
			gcp->gc_tx_desc_init = &nfo_tx_desc_init32;
			gcp->gc_rx_desc_init = &nfo_rx_desc_init32;
			gcp->gc_tx_desc_clean = &nfo_tx_desc_init32;
			gcp->gc_rx_desc_clean = &nfo_rx_desc_init32;
#ifndef TEST_FMT1
			if (p->hwcap & CAP_JUMBO) {
				/* format 2 */
				lp->dmac |= DMACTRL_DFMT_2 | DMACTRL_TxRCMP_EN;
				if (lp->hwcap & CAP_CKSUM) {
					lp->dmac |= DMACTRL_RXCHECK;
				}
			}
#endif
		} else {
			/* format 3 */
			gcp->gc_tx_desc_write = &nfo_tx_desc_write64;
			gcp->gc_rx_desc_write = &nfo_rx_desc_write64;
			gcp->gc_tx_desc_stat = &nfo_tx_desc_stat;
			gcp->gc_rx_desc_stat = &nfo_rx_desc_stat;
			gcp->gc_tx_desc_init = &nfo_tx_desc_init64;
			gcp->gc_rx_desc_init = &nfo_rx_desc_init64;
			gcp->gc_tx_desc_clean = &nfo_tx_desc_init64;
			gcp->gc_rx_desc_clean = &nfo_rx_desc_init64;

			lp->dmac |=
			    DMACTRL_DFMT_3 | DMACTRL_TxRCMP_EN | 0x00c00000;
			if (lp->hwcap & CAP_CKSUM) {
				lp->dmac |= DMACTRL_RXCHECK;
			}

			if (lp->hwcap & CAP_VLAN) {
				lp->dmac |= DMACTRL_VLANSTRIP | DMACTRL_VLANINS;
			}
		}

		gcp->gc_get_packet = NULL;

		/* mii operations */
		gcp->gc_mii_probe = &nfo_mii_probe;
		gcp->gc_mii_init = &nfo_mii_init;
		gcp->gc_mii_config = &nfo_mii_config;
		gcp->gc_mii_sync = &nfo_mii_sync;
		gcp->gc_mii_read = &nfo_mii_read;
		gcp->gc_mii_write = &nfo_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* MSI/MSIX interrupts */
		gcp->gc_nintrs_req = 1;

		/* jumbo frame and lso */
		gcp->gc_max_lso = 16 * 1024;
		gcp->gc_max_mtu = (p->hwcap & CAP_JUMBO) ? 9202 : ETHERMTU;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		lp->revid = revid;
		lp->mac_in_use = B_FALSE;
		lp->mac_initialized = B_FALSE;
		lp->phy_initialized = B_FALSE;

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
		kmem_free(lp, sizeof (struct nfo_dev));
err:
		return (DDI_FAILURE);

	case DDI_RESUME:
		nfo_power_up(GEM_GET_DEV(dip));
		return (gem_resume(dip));

	}
	return (DDI_FAILURE);
}

static int
nfodetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct gem_dev	*dp;
	struct nfo_dev	*lp;

	switch (cmd) {
	case DDI_DETACH:
		dp = GEM_GET_DEV(dip);
		lp = dp->private;

		if (lp->mac_in_use) {
			/* XXX - reset driverloaded flag */
			OUTL(dp, TxCTRL,
			    INL(dp, TxCTRL) & ~TxCTRL_HSEMA_LOADED);
		}

		/*
		 * XXX - We need to restore the original factory mac address.
		 * Once we changed the node addresss, there is no way
		 * to get the correct address.
		 */
		OUTL(dp, MACADDR + 0, lp->factory_mac_addr[0]);
		OUTL(dp, MACADDR + 4, lp->factory_mac_addr[1]);
		return (gem_do_detach(dip));

	case DDI_SUSPEND:
	case DDI_PM_SUSPEND:
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
GEM_STREAM_OPS(nfo_ops, nfoattach, nfodetach);
#else
static	struct module_info nfominfo = {
	0,			/* mi_idnum */
	"nfo",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit nforinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&nfominfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit nfowinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&nfominfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	nfo_info = {
	&nforinit,	/* st_rdinit */
	&nfowinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_nfo_ops = {
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
	&nfo_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops nfo_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	nfoattach,	/* devo_attach */
	nfodetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_nfo_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&nfo_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!nfo: _init: called"));

	gem_mod_init(&nfo_ops, "nfo");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&nfo_ops);
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

	DPRINTF(2, (CE_CONT, "!nfo: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&nfo_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
