/*
 * gani : The Realtek RTL816x Gigabit Ethernet Driver for Solaris
 *
 * Copyright (c) 2004-2012 Masayuki Murayama.  All rights reserved.
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

#pragma	ident	"%Z%%M% %I%     %E%"

/*
 * TODO
 *	offloading seem to cause tx timeout
 *	tx hang on 8169
 *	rx hang on 8169
 *
 *  BUGS
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
#include "rtl8169reg.h"

char	ident[] = "rtl816x nic driver v" VERSION;

/*
 * Useful macros
 */
#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))
#define	ETHERHEADERL	(ETHERMAX - ETHERMTU)
#define	VTAG_SIZE	4
#define	RMS_PCI(m)	(ETHERHEADERL + VTAG_SIZE + (m) + ETHERFCSL + 4)
#define	RMS_PCIE(m)	(ETHERHEADERL + VTAG_SIZE + (m) + ETHERFCSL + 1)
#define	ONESEC		(drv_usectohz(1*1000000))

#define	DESC_BASE_ALIGN	256	/* it should be defined in rtl8169reg.h */
#define	DESC_MAX_SIZE	(16*1024)	/* in byte */

#define	TALLY_OFFSET(dp)	\
	(((intptr_t)(dp)->io_area) - ((intptr_t)(dp)->rx_ring))
#define	TALLY_VADDR(dp)		((void *)((dp)->io_area))
#define	TALLY_DMA_ADDR(dp)	((dp)->io_area_dma)

#ifdef MAP_MEM
#define	FLSHB(dp, reg)	(void) INB(dp, reg)
#define	FLSHW(dp, reg)	(void) INW(dp, reg)
#define	FLSHL(dp, reg)	(void) INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif

#define	TXDESC(p)	((struct tx_desc *)(void *)(p))
#define	RXDESC(p)	((struct rx_desc *)(void *)(p))

/* additional PCI related macros for compatibility */
#ifndef PCI_CAP_ID_PCI_E
#define	PCI_CAP_ID_PCI_E	0x10
#endif

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif
/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int gani_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (gani_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

#define	DUMP_ETHER(n, dp, m) \
	DPRINTF(n, (CE_CONT, "!%s: %s: mac: %02x:%02x:%02x:%02x:%02x:%02x", \
		(dp)->name, __func__, \
		(m)[0], (m)[1], (m)[2], (m)[3], (m)[4], (m)[5]));
/*
 * Our configration for rtl816x
 */
static boolean_t gani_64bit_addr = B_FALSE;

#ifndef GEM_CONFIG_TX_DIRECT
#undef TX_MAX_FRAGS
#define	TX_MAX_FRAGS	1
#undef TX_BUF_SIZE
#define	TX_BUF_SIZE	256
#undef TX_RING_SIZE
#define	TX_RING_SIZE	TX_BUF_SIZE
#else
#define	TX_MAX_FRAGS	GEM_MAXTXFRAGS
#endif

#ifdef TEST_TXDESC_FULL
#define	TX_RING_SIZE	(DESC_BASE_ALIGN / sizeof (struct tx_desc))
#endif
#ifdef TEST_RX_EMPTY
#define	RX_BUF_SIZE	1
#endif

#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	256
#endif
#ifndef	TX_RING_SIZE
#define	TX_RING_SIZE	(DESC_MAX_SIZE / sizeof (struct tx_desc))
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif
#ifndef	RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	OUR_INTR_MASK	\
	(INTR_SERR | INTR_SWInt | INTR_FOVW | INTR_PUN | INTR_RDU | \
	INTR_TER | INTR_TOK | INTR_RER | INTR_ROK)

#if defined(sun4u) || defined(CONFIG_TX_COPY)
static int	gani_tx_copy_thresh = ETHERMAX + 4;
#else
static int	gani_tx_copy_thresh = 256;
#endif
#if defined(sun4u) || defined(CONFIG_RX_COPY)
static int	gani_rx_copy_thresh = ETHERMAX + 4;
#else
static int	gani_rx_copy_thresh = 256;
#endif
static int gani_pcie_req_max = 5;

/*
 * Local device definitions
 */
struct chip_info {
	uint32_t	tcr_val; /* from RTL8169/RTL8168 docs */
	uint32_t	tcr_mask;
	int		max_mtu;
#define	MTU4K	(4*1024 - (ETHERMAX - ETHERMTU) - ETHERFCSL - 4)
#define	MTU6K	(6*1024 - (ETHERMAX - ETHERMTU) - ETHERFCSL - 4)
#define	MTU7K	(7*1024 - (ETHERMAX - ETHERMTU) - ETHERFCSL - 4)
#define	MTU9K	(9*1024 - (ETHERMAX - ETHERMTU) - ETHERFCSL - 4)
	uint_t		type;
	char		*name;
};

#define	GANI_TYPE_MASK		0xf000U
#define	GANI_TYPE_PCI		0x0000U
#define	GANI_TYPE_PCIE_GIGA	0x1000U
#define	GANI_TYPE_PCIE_FAST	0x2000U

#define	R8169_1	(0 | GANI_TYPE_PCI)
#define	R8169_2	(1 | GANI_TYPE_PCI)
#define	R8169_3	(2 | GANI_TYPE_PCI)
#define	R8169_4	(3 | GANI_TYPE_PCI)
#define	R8169_5	(4 | GANI_TYPE_PCI)
#define	R8169_6	(5 | GANI_TYPE_PCI)

#define	R8168_1 (0 | GANI_TYPE_PCIE_GIGA)
#define	R8168_2 (1 | GANI_TYPE_PCIE_GIGA)
#define	R8168_3 (2 | GANI_TYPE_PCIE_GIGA)
#define	R8168_4 (3 | GANI_TYPE_PCIE_GIGA)
#define	R8168_5 (4 | GANI_TYPE_PCIE_GIGA)
#define	R8168_6 (5 | GANI_TYPE_PCIE_GIGA)
#define	R8168_7 (6 | GANI_TYPE_PCIE_GIGA)
#define	R8168_8 (7 | GANI_TYPE_PCIE_GIGA)
#define	R8168_9 (8 | GANI_TYPE_PCIE_GIGA)
#define	R8168_10 (9 | GANI_TYPE_PCIE_GIGA)
#define	R8168_11 (10 | GANI_TYPE_PCIE_GIGA)
#define	R8168_12 (11 | GANI_TYPE_PCIE_GIGA)
#define	R8168_13 (12 | GANI_TYPE_PCIE_GIGA)
#define	R8168_14 (13 | GANI_TYPE_PCIE_GIGA)
#define	R8168_15 (14 | GANI_TYPE_PCIE_GIGA)

#define	R8101_1	(0 | GANI_TYPE_PCIE_FAST)
#define	R8101_2	(1 | GANI_TYPE_PCIE_FAST)
#define	R8101_3	(2 | GANI_TYPE_PCIE_FAST)
#define	R8101_4	(3 | GANI_TYPE_PCIE_FAST)
#define	R8101_5	(4 | GANI_TYPE_PCIE_FAST)
#define	R8101_6	(5 | GANI_TYPE_PCIE_FAST)
#define	R8101_7	(6 | GANI_TYPE_PCIE_FAST)
#define	R8101_8	(7 | GANI_TYPE_PCIE_FAST)
#define	R8101_9	(8 | GANI_TYPE_PCIE_FAST)
#define	R8101_10 (9 | GANI_TYPE_PCIE_FAST)
#define	R8101_11 (10 | GANI_TYPE_PCIE_FAST)
#define	R8101_X	(11 | GANI_TYPE_PCIE_FAST)

#define	GANI_PCIE(x)	((x)->tcr & 0x20000000)
#define	GANI_PCI(x)	(!GANI_PCIE(x))

#define	GANI_PCIE_GIGA(x)	\
	(GANI_PCIE(x) && (((x)->tcr & 0x0f000000) != 0x04000000))

#define	GANI_PCIE_FAST(x)	\
	(GANI_PCIE(x) && (((x)->tcr & 0x0f000000) == 0x04000000))

static struct chip_info chiptbl_8169[] = {
/* pci gigabit ethernet devices */
/* 012 */
{ 0x00000000, 0xfc800000, MTU7K, R8169_1, "RTL8169", },
{ 0x00800000, 0xfc800000, MTU7K, R8169_2, "RTL8169S/8110S rev.D", },
{ 0x04000000, 0xfc800000, MTU7K, R8169_3, "RTL8169S/8110S rev.E", },
{ 0x10000000, 0xfc800000, MTU7K, R8169_4, "RTL8169SB/8110SB", },
{ 0x18000000, 0xfc800000, MTU7K, R8169_5, "RTL8169/8110SC rev.D", },
{ 0x98000000, 0xfc800000, MTU7K, R8169_6, "RTL8169/8110SC rev.E", },

/* pci express gigabit ethernet devices */
/* 012 */
{ 0x30000000, 0x7c800000, MTU4K, R8168_1, "RTL8168B/8111B rev.B", },

{ 0x38000000, 0x7cf00000, MTU4K, R8168_2, "RTL8168B/8111B rev.E", },
{ 0x38500000, 0x7cf00000, MTU4K, R8168_3, "RTL8168B/8111B rev.F", },
{ 0x38000000, 0x7c800000, MTU4K, R8168_3, "RTL8168B/8111B unknown", },

{ 0x3c000000, 0x7cf00000, MTU6K, R8168_4, "RTL8168C/8111C", },
{ 0x3c200000, 0x7cf00000, MTU6K, R8168_5, "RTL8168C/8111C rev.B", },
{ 0x3c400000, 0x7cf00000, MTU6K, R8168_6, "RTL8168C/8111C rev.C", },
{ 0x3c000000, 0x7c800000, MTU6K, R8168_6, "RTL8168C/8111C unknown", },

{ 0x3c900000, 0x7cf00000, MTU6K, R8168_7, "RTL8168CP/8111CP rev.B", },
{ 0x3cb00000, 0x7cf00000, MTU6K, R8168_8, "RTL8168CP/8111CP rev.C", },
{ 0x3c800000, 0x7c800000, MTU6K, R8168_8, "RTL8168CP/8111CP", },

{ 0x28100000, 0x7cf00000, MTU9K, R8168_9, "RTL8168D/8111D", },
{ 0x28300000, 0x7cf00000, MTU9K, R8168_10, "RTL8168D/8111D rev.C", },
{ 0x28000000, 0x7c800000, MTU9K, R8168_10, "RTL8168D/8111D unknown", },

{ 0x28800000, 0x7cf00000, MTU9K, R8168_11, "RTL8168DP/8111DP", },
{ 0x28a00000, 0x7cf00000, MTU9K, R8168_12, "RTL8168DP/8111DP rev.B", },
{ 0x28800000, 0x7c800000, MTU9K, R8168_13, "RTL8168DP/8111DP rev.C", },

{ 0x2c100000, 0x7cf00000, MTU9K, R8168_14, "RTL8168E/8111E", },
{ 0x2c200000, 0x7cf00000, MTU9K, R8168_15, "RTL8168E/8111E rev.B", },
{ 0x2c000000, 0x7c800000, MTU9K, R8168_15, "RTL8168E/8111E unknown", },

/* pci express fast ethernet devices */
{ 0x34000000, 0x7cf00000, 1500, R8101_1, "RTL8101E rev.B", },
{ 0x34200000, 0x7cf00000, 1500, R8101_2, "RTL8101E rev.E", },
{ 0x34300000, 0x7cf00000, 1500, R8101_3, "RTL8101E rev.F", },
{ 0x34000000, 0x7c800000, 1500, R8101_3, "RTL8101E unknown", },

{ 0x34900000, 0x7cf00000, 1500, R8101_4, "RTL8102E", },
{ 0x34a00000, 0x7cf00000, 1500, R8101_5, "RTL8102E rev.B", },
{ 0x34c00000, 0x7cf00000, 1500, R8101_6, "RTL8103E", },
{ 0x34d00000, 0x7cf00000, 1500, R8101_7, "RTL8103E", },
{ 0x34e00000, 0x7cf00000, 1500, R8101_8, "RTL8103E", },
{ 0x34800000, 0x7c800000, 1500, R8101_8, "RTL8103E", },

{ 0x24900000, 0x7cf00000, 1500, R8101_4, "RTL8102EL", },
{ 0x24a00000, 0x7cf00000, 1500, R8101_5, "RTL8102EL rev.B", },
{ 0x24c00000, 0x7cf00000, 1500, R8101_6, "RTL8103EL", },
{ 0x24d00000, 0x7cf00000, 1500, R8101_7, "RTL8103EL", },
{ 0x24e00000, 0x7cf00000, 1500, R8101_8, "RTL8103EL", },
{ 0x24800000, 0x7c800000, 1500, R8101_8, "RTL8103EL unknown", },

{ 0x24100000, 0x7cf00000, 1500, R8101_9, "RTL8401EL", },
{ 0x24000000, 0x7c800000, 1500, R8101_9, "RTL8401EL unknown", },

#ifdef CONFIG_RTL8105 /* not yet, it conflicts 8168 */
{ 0x2c100000, 0x7cf00000, 1500, R8101_10, "RTL8105EL", },
{ 0x2c000000, 0x7c800000, 1500, R8101_10, "RTL8105EL unknown", },

{ 0x40800000, 0x7c800000, 1500, R8101_11, "RTL8105EL unknown", },
#endif
};

#define	CHIPTABLESIZE	(sizeof (chiptbl_8169)/sizeof (struct chip_info))

struct gani_dev {
	/*
	 * Misc HW information
	 */
	struct chip_info	*chip;
	uint16_t		svid;
	uint16_t		sdid;
	uint8_t			mac[ETHERADDRL];
	uint8_t			busclk;
#define	BUSCLK_PCI33	0
#define	BUSCLK_PCI66	1
#define	BUSCLK_PCIE	2
	uint8_t			pcie_cap;

	uint32_t		rcr;
	uint32_t		tcr;
	boolean_t		need_to_reset;
	boolean_t		last_stats_valid;
	uint16_t		imr;
	uint16_t		isr_pended;
	boolean_t		msi;
	boolean_t		initialized;
#ifdef GEM_CONFIG_VLAN_HW
#ifdef CONFIG_CKSUM_OFFLOAD
	boolean_t		new_cksum;
#endif
#endif

#ifdef TEST_FOVW
	int			rx_pkt_cnt;
#endif
#ifdef CONFIG_POLLING
	int			last_poll_interval; /* polling interval in nS */
#endif

	struct rtl8169_tally_counters	last_stat;

	kmutex_t		stat_lock;
	timeout_id_t		stat_to_id;
#ifdef WA_NEW_TX_HANG
	int			tx_last_desc;
#endif
};

/*
 * private functions
 */

#ifdef DEBUG_LEVEL
/* debugging support */
static void gani_tx_desc_dump(struct gem_dev *dp);
#endif

/* mii operations */
static void  gani_mii_sync(struct gem_dev *);
static uint16_t gani_mii_read_raw(struct gem_dev *, uint_t);
static void gani_mii_write_raw(struct gem_dev *, uint_t, uint16_t);
static uint16_t gani_mii_read(struct gem_dev *, uint_t);
static void gani_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int gani_attach_chip(struct gem_dev *);
static int gani_reset_chip(struct gem_dev *);
static int gani_init_chip(struct gem_dev *);
static int gani_start_chip(struct gem_dev *);
static int gani_stop_chip(struct gem_dev *);
static int gani_set_media(struct gem_dev *);
static int gani_set_rx_filter(struct gem_dev *);
static int gani_get_stats(struct gem_dev *);

/* descriptor operations */
static int gani_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void gani_tx_start(struct gem_dev *dp, int slot, int frags);
static void gani_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t gani_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t gani_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void gani_tx_desc_init(struct gem_dev *dp, int slot);
static void gani_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t gani_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr gani_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr gani_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t gani_dma_attr_buf32 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffff,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000ffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0 /* pached later */,	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t gani_dma_attr_buf64 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffffffull,	/* dma_attr_addr_hi */
	0xffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffff,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x00000000ffffull,	/* dma_attr_maxxfer */
	0xffffffffffffull,	/* dma_attr_seg */
	0 /* pached later */,	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t gani_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	DESC_BASE_ALIGN,	/* dma_attr_align */
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
 * Debugging support
 */
/* =============================================================== */
#ifdef DEBUG_HANG
static void
gani_dump_regs(struct gem_dev *dp, char *m)
{
	int		i;

	cmn_err(CE_CONT, "!%s: register dump (%s)", dp->name, m);

	for (i = 0; i < 256; i += 8) {
		cmn_err(CE_CONT,
		    "!%02x: %02x %02x %02x %02x %02x %02x %02x %02x",
		    i,
		    INB(dp, i+0), INB(dp, i+1),
		    INB(dp, i+2), INB(dp, i+3),
		    INB(dp, i+4), INB(dp, i+5),
		    INB(dp, i+6), INB(dp, i+7));
	}
}
#endif
/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
/* IO routines for PCI-E phy */
static uint16_t
gani_ephy_read(struct gem_dev *dp, uint_t location)
{
	int	i;

	OUTL(dp, EPHYAR, location << EPHYAR_REGADDR_SHIFT);

	drv_usecwait(100);

	for (i = 0; (INL(dp, EPHYAR) & EPHYAR_FLAG) == 0; i++) {
		if (i > 10) {
			cmn_err(CE_CONT, "%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(100);
	}

	return ((uint16_t)(INL(dp, EPHYAR) & EPHYAR_DATA));
}

static void
gani_ephy_write(struct gem_dev *dp, uint_t location, uint16_t val)
{
	int	i;

	OUTL(dp, EPHYAR,
	    EPHYAR_FLAG | (location << EPHYAR_REGADDR_SHIFT) | val);

	for (i = 0; INL(dp, EPHYAR) & EPHYAR_FLAG; i++) {
		if (i > 10) {
			cmn_err(CE_CONT, "%s: %s: timeout",
			    dp->name, __func__);
			return;
		}
		drv_usecwait(100);
	}
}

static uint32_t
gani_csi_read(struct gem_dev *dp, uint_t addr)
{
	int	i;

	OUTL(dp, CSIAR, CSIAR_ByteEn | addr);

	/* Check if the RTL8101 has completed CSI read */
	for (i = 0; (INL(dp, CSIAR) & CSIAR_FLAG) == 0; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "%s: %s timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(100);
	}

	return (INL(dp, CSIDR));
}

static void
gani_csi_write(struct gem_dev *dp, uint_t addr, uint32_t val)
{
	int	i;

	OUTL(dp, CSIDR, val);
	OUTL(dp, CSIAR, CSIAR_FLAG | CSIAR_ByteEn | addr);

	for (i = 0; INL(dp, CSIAR) & CSIAR_FLAG; i++) {
		if (i > 10) {
			break;
		}
		drv_usecwait(100);
	}
}

static uint32_t
gani_efuse_read(struct gem_dev *dp, uint_t index)
{
	int	i;
	uint32_t	val;
	struct gani_dev	*lp = dp->private;

	OUTL(dp, EFUSEAR, EFUSE_READ | (index << EFUSE_REG_SHIFT));
	drv_usecwait(100);

	for (i = 0; (INL(dp, EFUSEAR) & EFUSE_READ_OK) == 0; i++) {
		if (i > 300) {
			/* time out */
			val = 0xff;
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			goto x;
		}
		drv_usecwait(100);
	}

	val = INL(dp, EFUSEAR) & EFUSE_DATA;
x:
	return (val);
}

static int
gani_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct gani_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "reset_chip 1");
#endif
	if (!lp->initialized) {
		OUTB(dp, CR9346, CR9346_EEM_WE);
		OUTB(dp, CFG1, INB(dp, CFG1) | CFG1_PMEn);
		OUTB(dp, CFG5, INB(dp, CFG5) & CFG5_PME_STS);
		OUTB(dp, CR9346, CR9346_EEM_NORMAL);

		if (lp->busclk == BUSCLK_PCI66) {
			if (lp->chip->type == R8169_5) {
				OUTL(dp, 0x7c, 0x000fffff);
			} else if (lp->chip->type == R8169_6) {
				OUTL(dp, 0x7c, 0x003fffff);
			}
		} else if (lp->busclk == BUSCLK_PCI33) {
			if (lp->chip->type == R8169_5) {
				OUTL(dp, 0x7c, 0x000fff00);
			} else if (lp->chip->type == R8169_6) {
				OUTL(dp, 0x7c, 0x003fff00);
			}
		}

		lp->initialized = B_TRUE;
	}

	val = INL(dp, RCR);
	if (val & RCR_ACCEPT_MODE) {
		OUTL(dp, RCR, val & ~RCR_ACCEPT_MODE);
	}

	if (GANI_PCIE(lp)) {
		switch (lp->chip->type) {
		/* 016 */
		case R8168_1: /* RTL8168B rev.B */
		case R8168_2: /* RTL8168B rev.E */
		case R8168_3: /* RTL8168B rev.F */

		case R8101_1: /* RTL8101E rev.B */
		case R8101_2: /* RTL8101E rev.E */
		case R8101_3: /* RTL8101E rev.F */
			/* EMPTY */
			break;

		/* 016 */
		case R8168_11: /* RTL8168DP */
		case R8168_12: /* RTL8168DP */
			for (i = 0;
			    (INB(dp, TPPoll) & TPPoll_NPQ) && i < 1000;
			    i++) {
				drv_usecwait(20);
			}
			break;

		default:
			OUTB(dp, CR, CR_STOP | CR_RE | CR_TE);
			drv_usecwait(100);
			break;
		}
	}

	/* disable interrupts */
	OUTW(dp, IMR, 0);
	FLSHW(dp, IMR);

	OUTB(dp, CR, CR_RST);

	for (i = 0; INB(dp, CR) & CR_RST; i++) {
		if (i > 10000) {
			/* timeout */
			cmn_err(CE_WARN, "!%s: failed to reset: timeout",
			    dp->name);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}

	/* 013 */
	if (lp->chip->type == R8168_11) {
		OUTL(dp, OCPDR, 0x00000001U);
		OUTL(dp, OCPAR, 0x80001038U);
		OUTL(dp, OCPDR, 0x00000001U);
		OUTL(dp, OCPAR, 0x00001030U);

		OUTL(dp, OCPAR, 0x00001034U);
		for (i = 0; (INL(dp, OCPDR) & 0xffffU); i++) {
			if (i > 1000) {
				cmn_err(CE_CONT, "!%s: %s: OCP timeout",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(100);
		}
	}

	/* clear bogus interrupts */
	lp->isr_pended = INW(dp, ISR);
	OUTW(dp, ISR, 0xffffU);
	FLSHW(dp, ISR);

	if (GANI_PCI(lp)) {
		OUTB(dp, 0x82, 0x01);

		switch (lp->chip->type) {
		case R8169_2: /* RTL8169S rev.D */
			OUTB(dp, 0x82, 0x01);
			break;
		}
	}

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "reset_chip 2");
#endif
	return (GEM_SUCCESS);
}

/*
 * Setup rtl8169
 */
static int
gani_init_8169(struct gem_dev *dp, ddi_acc_handle_t conf_ha)
{
	uint32_t		val;
	uint32_t		cpcmd;
	struct gani_dev		*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	OUTW(dp, RMS, RMS_PCI(dp->mtu));
	DPRINTF(0, (CE_CONT, "!%s: RMS %d", dp->name, INW(dp, RMS)));

	/* make configuration registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	OUTB(dp, ETThR, 0x3f);

	/* configure CPlus command register */
	cpcmd = INW(dp, CpCR);
	cpcmd &= ~(CpCR_RxVLAN | CpCR_RxChkSum);
	cpcmd |= CpCR_MulRW;
	if (gani_64bit_addr) {
		cpcmd |= CpCR_DAC;
	}
#ifdef GEM_CONFIG_VLAN_HW
	cpcmd |= CpCR_RxVLAN;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	cpcmd |= CpCR_RxChkSum;
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
#endif /* GEM_CONFIG_VLAN_HW */
	OUTW(dp, CpCR, cpcmd);

	/* fix pci configuration registers */
	pci_config_put8(conf_ha, PCI_CONF_CACHE_LINESZ, 8);

	if (lp->chip->type == R8169_2 /* RTL8169S rev.D */) {
		cpcmd |= CpCR_EnAnaPLL;
	} else {
		cpcmd &= ~CpCR_EnAnaPLL;
	}
	OUTW(dp, CpCR, cpcmd);

	pci_config_put8(conf_ha, PCI_CONF_LATENCY_TIMER, 0x40);

	/* XXX - undocumented */
	OUTW(dp, 0xe2, 0);

	/* Transmit Normal Priority Descriptors */
	DPRINTF(4, (CE_CONT, "!%s: TNPDS 0x%llx",
	    dp->name, (unsigned long long)dp->tx_ring_dma));
	OUTL(dp, TNPDSL, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, TNPDSH, (uint32_t)(dp->tx_ring_dma >> 32));

	/* Transmit High Priority Descriptors: Do nothing */
	OUTL(dp, THPDSL, 0);
	OUTL(dp, THPDSH, 0);

	/* Receive Descriptor Start Address register */
	OUTL(dp, RDSARL, (uint32_t)dp->rx_ring_dma);
	OUTL(dp, RDSARH, (uint32_t)(dp->rx_ring_dma >> 32));
	DPRINTF(4, (CE_CONT, "!%s: RDSAR 0x%llx",
	    dp->name, (unsigned long long)dp->rx_ring_dma));

	/* Missed packet counter: clear it */
	OUTL(dp, MPC, 0);

	/* MulInt Register */
	OUTW(dp, MULINT, INW(dp, MULINT) & 0xf000);

	/* Interrupt status register: clear bogus interrupts */
	OUTW(dp, ISR, 0xffffU);
	FLSHW(dp, ISR);

	/* enable tx and rx before writing rcr and tcr */
	OUTB(dp, CR, CR_RE | CR_TE);

	OUTL(dp, RCR, lp->rcr);
	OUTL(dp, TCR, lp->tcr);

	/* make config registers read only */
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	drv_usecwait(10);

	/* TimerInt Register */
	OUTL(dp, TimerInt, 0);

	/* Timer count register: test count rate */
	OUTL(dp, TCTR, 0);
#if DEBUG_LEVEL > 3
	drv_usecwait(1000);
	cmn_err(CE_CONT, "!%s: TCTR:%d in 1mS", dp->name, INL(dp, TCTR));
#endif
	return (GEM_SUCCESS);
}

static int
gani_init_8168(struct gem_dev *dp, ddi_acc_handle_t conf_ha)
{
	uint32_t		val;
	uint32_t		cpcmd;
	uint32_t		devcsr;
	struct gani_dev		*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	pci_config_put8(conf_ha, PCI_CONF_LATENCY_TIMER, 0x40);

	/* make configuration registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	/* reserved */
	OUTB(dp, ETThR, 0x3f);

	OUTW(dp, RMS, RMS_PCIE(dp->mtu));
	DPRINTF(0, (CE_CONT, "!%s: RMS %d", dp->name, INW(dp, RMS)));

	/* configure CPlus command register */
	cpcmd = INW(dp, CpCR);
	cpcmd |= CpCR_StatDis | CpCR_INTT_1;
	cpcmd &= ~(CpCR_RxVLAN | CpCR_RxChkSum);

#ifdef GEM_CONFIG_VLAN_HW
	/* common capability for CpCR */
	cpcmd |= CpCR_RxVLAN;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if (dp->mtu <= ETHERMTU) {
		cpcmd |= CpCR_RxChkSum;
	} else {
		cpcmd &= ~CpCR_RxChkSum;
	}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
#endif /* GEM_CONFIG_VLAN_HW */
	OUTW(dp, CpCR, cpcmd);

	/* XXX - undocumented */
	OUTW(dp, 0xe2, 0x5151);

	/* Transmit Normal Priority Descriptors */
	DPRINTF(2, (CE_CONT, "!%s: TNPDS 0x%llx",
	    dp->name, (uint64_t)dp->tx_ring_dma));
	OUTL(dp, TNPDSH, (uint32_t)(dp->tx_ring_dma >> 32));
	OUTL(dp, TNPDSL, (uint32_t)dp->tx_ring_dma);
#ifdef SANITY
	/* Transmit High Priority Descriptors: Do nothing */
	OUTL(dp, THPDSH, 0);
	OUTL(dp, THPDSL, 0);
#endif
	/* Receive Descriptor Start Address register */
	DPRINTF(2, (CE_CONT, "!%s: RDSAR 0x%llx",
	    dp->name, (uint64_t)dp->rx_ring_dma));
	OUTL(dp, RDSARH, (uint32_t)(dp->rx_ring_dma >> 32));
	OUTL(dp, RDSARL, (uint32_t)dp->rx_ring_dma);

	OUTL(dp, RCR, lp->rcr);
	OUTL(dp, TCR, lp->tcr);
	DPRINTF(0, (CE_CONT, "!%s: %s: rcr:%b, tcr:%x",
	    dp->name, __func__,
	    INL(dp, RCR), RCR_BITS, INL(dp, TCR)));

	/* Interrupt status register: clear bogus interrupts */
	OUTW(dp, ISR, 0xffffU);
	FLSHW(dp, ISR);

	devcsr = pci_config_get32(conf_ha, lp->pcie_cap + 8);
	DPRINTF(0, (CE_CONT, "!%s: %s: devcsr:0x%x",
	    dp->name, __func__, devcsr));
	devcsr &= ~(7 << 12);

	switch (lp->chip->type) {
	case R8168_11:
		devcsr |= gani_pcie_req_max << 12;
		break;
	default:
		if (dp->mtu > ETHERMTU) {
			devcsr |= 2 << 12;
		} else {
			devcsr |= gani_pcie_req_max << 12;
		}
		break;
	}

	switch (lp->chip->type) {
	case R8168_1:
	case R8168_2:
	case R8168_3:
		devcsr |= 0x800;
		break;
	}
	pci_config_put32(conf_ha, lp->pcie_cap + 8, devcsr);

	switch (lp->chip->type) {
	case R8168_4: /* RTL8168C */
		/* 012 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);

		OUTB(dp, DBGREG, 0xe0 | DBGREG_FIX_NAK_1 | DBGREG_FIX_NAK_2);

		gani_ephy_write(dp, 2,
		    (gani_ephy_read(dp, 2) & ~0x0800) | 0x1000);
		gani_ephy_write(dp, 3, gani_ephy_read(dp, 3) | 0x0002);
		gani_ephy_write(dp, 6, gani_ephy_read(dp, 6) & ~0x0080);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		pci_config_put8(conf_ha, 0x81, 0);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
			/* don't use cksum offload */
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_5: /* RTL8168C rev.B */
		/* 012 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);

		gani_ephy_write(dp, 1,
		    gani_ephy_read(dp, 1) | 0x0001);
		gani_ephy_write(dp, 3,
		    (gani_ephy_read(dp, 3) & ~0x0400) | 0x0220);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_6: /* RTL8168C rev.C */
		/* 013 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_7: /* RTL8168CP rev.B */
		/* 013 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);
#ifdef notyet
		gani_eri_write(dp, 0x1EC, 1, 0x07, ERIAR_ASF);
#endif
		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_8: /* RTL8168 rev.C */
		/* 013 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);
#ifdef notyet
		gani_eri_write(dp, 0x1EC, 1, 0x07, ERIAR_ASF);
#endif
		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);
		OUTB(dp, 0xD1, 0x20);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_9: /* RTL8168D */
		/* 016 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x13000000);

		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~0x10);
		OUTB(dp, DBGREG, INB(dp, DBGREG) | 0x82);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		gani_ephy_write(dp, 0x01, 0x7c7d);
		gani_ephy_write(dp, 0x02, 0x091f);
		gani_ephy_write(dp, 0x06, 0xb271);
		gani_ephy_write(dp, 0x07, 0xce00);
		break;

	case R8168_10: /* RTL8168D rev.C */
		/* 016 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x13000000);

		OUTB(dp, DBGREG, INB(dp, DBGREG) | 0x82);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		OUTB(dp, CFG1, (INB(dp, CFG1) & 0xc0) | 0x1F);

		gani_ephy_write(dp, 0x01, 0x6C7F);
		gani_ephy_write(dp, 0x02, 0x011F);
		gani_ephy_write(dp, 0x03, 0xC1B2);
		gani_ephy_write(dp, 0x1A, 0x0546);
		gani_ephy_write(dp, 0x1C, 0x80C4);
		gani_ephy_write(dp, 0x1D, 0x78E4);
		gani_ephy_write(dp, 0x0A, 0x8100);

		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		OUTB(dp, 0xf3, INB(dp, 0xf3) | 0x04);
		break;

	case R8168_11: /* RTL8168DP */
		/* 016 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
		}
		/* disable clock request */
		pci_config_put8(conf_ha, 0x81, 0);

		OUTB(dp, CFG1, (INB(dp, CFG1) & 0xc0) | 0x1F);
		break;

	case R8168_12: /* RTL8168DP */
		/* 016 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG3, INB(dp, CFG3) | CFG3_JUMBO);
		} else {
			OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_JUMBO);
		}
#ifdef notyet
		val = gani_ephy_read(dp, 0x0B);
		gani_ephy_write(dp, 0x03, val | 0x48);

		val = gani_ephy_read(dp, 0x19) & 0x20;
		gani_ephy_write(dp, 0x03, val | 0x50);

		val = gani_ephy_read(dp, 0x0C);
		gani_ephy_write(dp, 0x03, val | 0x20);
#endif
		pci_config_put8(conf_ha, 0x81, 1);

		OUTB(dp, CFG1, (INB(dp, CFG1) & 0xc0) | 0x1F);
		break;

	case R8168_1: /* RTL8168B rev.B */
		/* 012 */
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);
		break;

	case R8168_2: /* RTL8168B rev.E */
		/* 012 */
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;

	case R8168_3: /* RTL8168B rev.F */
		/* 012 */
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, ETThR, 0x3f);

		if (dp->mtu > ETHERMTU) {
			OUTB(dp, CFG4, INB(dp, CFG4) | CFG4_JUMBO);
		} else {
			OUTB(dp, CFG4, INB(dp, CFG4) & ~CFG4_JUMBO);
		}
		break;
	}

	OUTB(dp, CR, CR_RE | CR_TE);

	/* make config registers read only */
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	drv_usecwait(10);

	/* TimerInt Register */
	OUTL(dp, TimerInt, 0);

	/* Timer count register: test count rate */
	OUTL(dp, TCTR, 0);
#if DEBUG_LEVEL > 3
	drv_usecwait(1000);
	cmn_err(CE_CONT, "!%s: TCTR:%d in 1mS", dp->name, INL(dp, TCTR));
#endif
	return (GEM_SUCCESS);
}

static int
gani_init_8101(struct gem_dev *dp, ddi_acc_handle_t conf_ha)
{
	uint32_t		val;
	uint32_t		cpcmd;
	struct gani_dev		*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	OUTW(dp, RMS, RMS_PCIE(dp->mtu));
	DPRINTF(2, (CE_CONT, "!%s: RMS %d", dp->name, INW(dp, RMS)));

	/* configure CPlus command register */
	cpcmd = INW(dp, CpCR);
	cpcmd &= ~(CpCR_RxVLAN | CpCR_RxChkSum);
#ifdef GEM_CONFIG_VLAN_HW
	cpcmd &= ~CpCR_RxVLAN;
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	cpcmd |= CpCR_RxChkSum;
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
#endif /* GEM_CONFIG_VLAN_HW */
	OUTW(dp, CpCR, cpcmd);

	switch (lp->chip->type) {
	case R8101_4: /* RTL8102E */
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		/* XXX config2/4 upon link_control */

		OUTB(dp, CFG1, 0x0f);
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x7;
		OUTW(dp, CpCR, val);
		gani_ephy_write(dp, 0x03, 0xc2f9);
		break;

	case R8101_5: /* RTL8102E rev.B */
#ifdef gone
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);
#endif
		/* XXX config2/4 upon link_control */

		OUTB(dp, CFG1, 0x0f);
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x7;
		OUTW(dp, CpCR, val);
		gani_ephy_write(dp, 0x01, 0x6FE5);
		gani_ephy_write(dp, 0x03, 0xD7D9);
		break;

	case R8101_6: /* RTL8103E */
#ifdef gone
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		OUTB(dp, DBGREG, 0x10);
#endif
		/* XXX config2/4 upon link_control */

		OUTB(dp, CFG1, 0xdf);
		OUTB(dp, 0xf4, 0x01);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x7;
		OUTW(dp, CpCR, val);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		gani_ephy_write(dp, 0x06, 0xaf35);
		break;

	case R8101_7: /* RTL8103E */
	case R8101_8: /* RTL8103E */
#ifdef gone
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		OUTB(dp, DBGREG, 0x10);
#endif
		OUTB(dp, CFG1, (INB(dp, CFG1) & 0xc0) | 0x1f);
		OUTB(dp, 0xf4, 0x01);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x7;
		OUTW(dp, CpCR, val);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		OUTB(dp, 0xf5, INB(dp, 0xf5) | 0x04);
		gani_ephy_write(dp, 0x19, 0xec90);
		gani_ephy_write(dp, 0x01, 0x6fe5);
		gani_ephy_write(dp, 0x03, 0x05d9);
		gani_ephy_write(dp, 0x06, 0xaf35);
		break;

	case R8101_9: /* RTL8401 */
#ifdef gone
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x17000000);

		OUTB(dp, DBGREG, 0x18);
#endif
		OUTB(dp, CFG1, 0xdf);
		OUTB(dp, 0xf4, 0x01);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x7;
		OUTW(dp, CpCR, val);

		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);

		gani_ephy_write(dp, 0x06, 0xaf25);
		gani_ephy_write(dp, 0x07, 0x8e68);
		break;

#ifdef CONFIG_RTL8105
	case R8101_10:
		/* set PCI configuration space offset 0x70F to 0x17 */
		val = gani_csi_read(dp, 0x70c) & 0x00ffffff;
		gani_csi_write(dp, 0x70c, val | 0x27000000);

		OUTB(dp, ETThR, 0x0c);

		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, 0xf3, INB(dp, 0xf3) | 0x20);
		OUTB(dp, 0xf3, INB(dp, 0xf3) & ~0x20);

		OUTB(dp, CFG5, (INB(dp, CFG5) & ~0x08) | 0x01);
		OUTB(dp, CFG2, INB(dp, CFG2) | 0x80);
		OUTB(dp, CFG3, INB(dp, CFG3) & ~CFG3_BEACON_EN);
		break;

	case R8101_11:
		val = cpcmd;
		val &= CpCR_NORMAL | CpCR_RxVLAN | CpCR_RxChkSum | 0x3;
		OUTW(dp, CpCR, val);

		OUTB(dp, 0xf2, INB(dp, 0xf2) & ~0x01);
		OUTB(dp, 0xd0, INB(dp, 0xd0) | 0x40);
		OUTW(dp, 0xe0, INW(dp, 0xe0) & ~0xdf9c);
		break;
#endif
	}

	/* make configuration registers writable */
	OUTB(dp, CR9346, CR9346_EEM_WE);

	/* reserved */
	OUTB(dp, ETThR /* 0xec */, 0x3f);

	OUTW(dp, CpCR, cpcmd);

	/* XXX - undocumented */
	OUTW(dp, 0xe2, 0);	/* intermitigate */

	/* Transmit Normal Priority Descriptors */
	DPRINTF(4, (CE_CONT, "!%s: TNPDS 0x%llx",
	    dp->name, (unsigned long long)dp->tx_ring_dma));
	OUTL(dp, TNPDSL, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, TNPDSH, (uint32_t)(dp->tx_ring_dma >> 32U));

	/* Receive Descriptor Start Address register */
	OUTL(dp, RDSARL, (uint32_t)dp->rx_ring_dma);
	OUTL(dp, RDSARH, (uint32_t)(dp->rx_ring_dma >> 32U));
	DPRINTF(4, (CE_CONT, "!%s: RDSAR 0x%llx",
	    dp->name, (unsigned long long)dp->rx_ring_dma));

	/* need to enable tx and rx before writing rcr and tcr */
	OUTB(dp, CR, CR_RE | CR_TE);

	OUTL(dp, RCR, lp->rcr);
	OUTL(dp, TCR, lp->tcr);

	/* make config registers read only */
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);
	drv_usecwait(10);

	/* TimerInt Register */
	OUTL(dp, TimerInt, 0);

	/* Timer count register: test count rate */
	OUTL(dp, TCTR, 0);
#if DEBUG_LEVEL > 3
	drv_usecwait(1000);
	cmn_err(CE_CONT, "!%s: TCTR:%d in 1mS", dp->name, INL(dp, TCTR));
#endif
	return (GEM_SUCCESS);
}

static int
gani_init_chip(struct gem_dev *dp)
{
	int	ret;
	ddi_acc_handle_t	conf_ha;
	struct gani_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (pci_config_setup(dp->dip, &conf_ha) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: pci_config_setup failed", dp->name);
		return (GEM_FAILURE);
	}

	if (GANI_PCI(lp)) {
		ret = gani_init_8169(dp, conf_ha);
	} else if (GANI_PCIE_GIGA(lp)) {
		ret = gani_init_8168(dp, conf_ha);
	} else {
		ret = gani_init_8101(dp, conf_ha);
	}
	pci_config_teardown(&conf_ha);

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "init_chip done");
#endif
#ifdef CONFIG_POLLING
	lp->last_poll_interval = 0; /* polling interval in nS */
#endif
	lp->last_stats_valid = B_FALSE;

	return (ret);
}

static uint_t
gani_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	uint_t	h;
	struct gani_dev	*lp = dp->private;

	h = gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6);

	return (GANI_PCIE(lp) ? (h ^ 0x38) : h);
}

static int
gani_set_rx_filter(struct gem_dev *dp)
{
	uint32_t	mode;
	uint64_t	mhash;
	int		i;
	uint8_t		*m;
	struct gani_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mode = RCR_AB	/* accept broadcast */
	    | RCR_APM	/* accept physical match  */
	    | RCR_AM;	/* accept multicast */

	mhash = 0;

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;

	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscious mode implies all multicast and all physical */
		mode |= RCR_AAP;
		mhash = ~0ULL;

	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* accept all multicast packets */
		mhash = ~0ULL;

	} else if (dp->mc_count > 0) {
		/*
		 * make the hash table to select the interresting
		 * multicast address only.
		 */
		for (i = 0; i < dp->mc_count; i++) {
			/* hash table is 64 = 2^6 bit width */
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	/*
	 * Don't disable the rx filter not to drop packets
	 * while we are changing rx filer mode.
	 */

	m = &dp->cur_addr.ether_addr_octet[0];
	if (bcmp(m, lp->mac, ETHERADDRL)) {
		/*
		 * make config registers writable before
		 * changing IDRs. (undocumented)
		 */
		OUTB(dp, CR9346, CR9346_EEM_WE);

		/* set mac address */
		OUTL(dp, IDR + 0,
		    (m[3] << 24) | (m[2] << 16) | (m[1] << 8) | m[0]);
		OUTL(dp, IDR + 4, (m[5] <<  8) | m[4]);

		OUTB(dp, CR9346, CR9346_EEM_NORMAL);
		drv_usecwait(10);
	}

	if (mode & RCR_AM) {
		/* need to set up multicast hash table */
		OUTL(dp, MAR + 4, (uint32_t)(mhash >> 32U));
		OUTL(dp, MAR + 0, (uint32_t)mhash);
	}

	/* update rcr */
	OUTL(dp, RCR, lp->rcr | mode);

	DPRINTF(4, (CE_CONT, "!%s: returned", __func__));
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "set_rxfilter done");
#endif

	return (GEM_SUCCESS);
}


#define	MII_ABILITY_SPEED_DUPLEX	\
	(MII_ABILITY_100BASE_T4	|	\
	MII_ABILITY_100BASE_TX_FD |	\
	MII_ABILITY_100BASE_TX |	\
	MII_ABILITY_10BASE_T |	\
	MII_ABILITY_10BASE_T_FD)

static int
gani_set_media(struct gem_dev *dp)
{
	struct gani_dev	*lp = dp->private;
#ifdef GEM3
	DPRINTF(0, (CE_CONT, "!%s: %s: phys: %b, mac_state: %d",
	    dp->name, __func__, INB(dp, PHYS), PHYS_BITS, dp->mac_state));
#else
	DPRINTF(0, (CE_CONT, "!%s: %s: phys: %b, mac_active: %d",
	    dp->name, __func__, INB(dp, PHYS), PHYS_BITS, dp->mac_active));
#endif
	switch (lp->chip->type) {
	/* 011 */
	case R8169_5:
	case R8169_6:
		if (dp->full_duplex) {
			lp->tcr = (lp->tcr & ~TCR_IFG) | TCR_IFG_802_3;
		} else {
			lp->tcr = (lp->tcr & ~TCR_IFG) | TCR_IFG_HALF_8169;
		}
		OUTL(dp, TCR, lp->tcr);
		break;

	case R8101_4:
	case R8101_5:
		if (gani_csi_read(dp, 0x70c) != 0x27000000) {
			gani_csi_write(dp, 0x70c, 0x27000000);
		}
		break;

	case R8168_11:
		if (dp->speed == GEM_SPD_10) {
			gani_mii_write_raw(dp, 0x1f, 0x0000);
			gani_mii_write_raw(dp, 0x10, 0x04ee);
		} else {
			gani_mii_write_raw(dp, 0x1f, 0x0000);
			gani_mii_write_raw(dp, 0x10, 0x01ee);
		}
		break;
	}

	return (GEM_SUCCESS);
}

static int
gani_start_chip(struct gem_dev *dp)
{
	struct gani_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(dp->mii_state == MII_STATE_LINKUP ||
	    dp->mii_state == MII_STATE_LINKDOWN);

	/* prepare to dump Tally Count Registers */
	OUTL(dp, DTCCRH, (uint32_t)(TALLY_DMA_ADDR(dp) >> 32));
#ifdef SANITY
	OUTL(dp, DTCCRL, 0);
#endif
	/* clear tally counter area */
	DPRINTF(2, (CE_CONT, "!%s: tally base: vaddr:0x%p, dma addr:0x%llx",
	    dp->name, TALLY_VADDR(dp),
	    (unsigned long long)TALLY_DMA_ADDR(dp)));
	bzero(TALLY_VADDR(dp), TC_SIZE);

	/* Enable interrupts */
	lp->imr = OUR_INTR_MASK;
	if (GANI_PCIE(lp)) {
		lp->imr |= INTR_TDU;
	}
#ifdef CONFIG_POLLING
	lp->imr |= INTR_TimeOut;
#endif
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTW(dp, IMR, lp->imr);
		FLSHW(dp, IMR);
	}
#ifdef DEBUG_HANG
	gani_dump_regs(dp, "start_chip done");
#endif
	return (GEM_SUCCESS);
}

/*
 * gani_stop_chip: disable interrupts and ensure rx has stopped.
 */
static int
gani_stop_chip(struct gem_dev *dp)
{
	int		i;
	uint8_t		*m;
	struct gani_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#if DEBUG_LEVEL > 1
	if (dp->tx_desc_head != dp->tx_desc_tail) {
		gani_tx_desc_dump(dp);
	}
#endif
	/* wait until dumping tally counters stops. */
	for (i = 0; INL(dp, DTCCRL) & DTCCR_CMD; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: timeout: dumping tally counters",
			    dp->name);
			break;
		}
		drv_usecwait(10);
	}

	/* disable interrupts */
	/* XXX - don't clear pended interrupts */
	OUTW(dp, IMR, 0);
	FLSHW(dp, IMR);

	switch (lp->chip->type) {
	case R8101_4:
	case R8101_5:
		if (gani_csi_read(dp, 0x70c) != 0x17000000) {
			gani_csi_write(dp, 0x70c, 0x17000000);
		}
		break;
	}

	/* stop nic core */
	(void) (dp->gc.gc_reset_chip)(dp);

	/* restore factory mac address */
	OUTB(dp, CR9346, CR9346_EEM_WE);
	m = dp->dev_addr.ether_addr_octet;
	OUTL(dp, IDR + 0,
	    (m[3] << 24) | (m[2] << 16) | (m[1] << 8) | m[0]);
	OUTL(dp, IDR + 4, (m[5] << 8) | m[4]);
	OUTB(dp, CR9346, CR9346_EEM_NORMAL);

#ifdef DEBUG_HANG
	gani_dump_regs(dp, "stop_chip done");
#endif
	return (GEM_SUCCESS);
}

/* private function */
static boolean_t
gani_update_stats(struct gem_dev *dp)
{
	uint32_t			x;
	struct rtl8169_tally_counters	*tbp;
	struct rtl8169_tally_counters	*ls;
	struct rtl8169_tally_counters	new;
	struct gani_dev			*lp = dp->private;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&lp->stat_lock));

	if (!IS_MAC_ONLINE(dp)) {
		/* nic isn't running yet */
		return (B_FALSE);
	}

	if (INL(dp, DTCCRL) & DTCCR_CMD) {
		/* Now dumping tally counters, do nothing */
		return (B_FALSE);
	}

	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)TALLY_OFFSET(dp), (size_t)TC_SIZE, DDI_DMA_SYNC_FORKERNEL);

	tbp = TALLY_VADDR(dp);

	new.tc_txok = LE_64(tbp->tc_txok);		/* 8byte */
	new.tc_rxok = LE_64(tbp->tc_rxok);		/* 8byte */
	new.tc_txer = LE_64(tbp->tc_txer);		/* 8byte */
	new.tc_rxer = LE_32(tbp->tc_rxer);		/* 4byte */
	new.tc_misspkt = LE_16(tbp->tc_misspkt);	/* 2byte */
	new.tc_fae = LE_16(tbp->tc_fae);		/* 2byte */
	new.tc_tx1col = LE_32(tbp->tc_tx1col);		/* 4byte */
	new.tc_txmcol = LE_32(tbp->tc_txmcol);		/* 4byte */
	new.tc_rxokphy = LE_64(tbp->tc_rxokphy);	/* 8byte */
	new.tc_rxokbrd = LE_64(tbp->tc_rxokbrd);	/* 8byte */
	new.tc_rxokmu = LE_32(tbp->tc_rxokmu);		/* 4byte */
	new.tc_txabt = LE_16(tbp->tc_txabt);		/* 2byte */
	new.tc_txundrn = LE_16(tbp->tc_txundrn);	/* 2byte */

	if (lp->last_stats_valid) {
		struct gem_stats	*gsp = &dp->stats;
#if 0
		ls = &lp->last_stat;
		gsp->errxmt += new.tc_txer - ls->tc_txer;
		gsp->errrcv += new.tc_rxer - ls->tc_rxer;
		gsp->missed += new.tc_misspkt - ls->tc_misspkt;
		gsp->frame += new.tc_fae - ls->tc_fae;
		gsp->first_coll +=	x = new.tc_tx1col - ls->tc_tx1col;
		gsp->collisions +=	x;
		gsp->multi_coll +=	x = new.tc_txmcol - ls->tc_txmcol;
		gsp->collisions +=	x*2;
		gsp->excoll += new.tc_txabt - ls->tc_txabt;
		gsp->underflow += new.tc_txundrn - ls->tc_txundrn;
#else
		uint64_t	txer;
		uint64_t	txok;
		uint64_t	rxer;
		uint64_t	rxok;

		ls = &lp->last_stat;

		txer = new.tc_txer - ls->tc_txer;
		txok = new.tc_txok - ls->tc_txok;
		rxer = new.tc_rxer - ls->tc_rxer;
		rxok = new.tc_rxok - ls->tc_rxok;

		gsp->errrcv += rxer;
		gsp->frame += new.tc_fae - ls->tc_fae;
		gsp->missed += new.tc_misspkt - ls->tc_misspkt;

		gsp->errxmt += txer;
		gsp->first_coll += x = new.tc_tx1col - ls->tc_tx1col;
		gsp->collisions += x;
		gsp->multi_coll += x = new.tc_txmcol - ls->tc_txmcol;
		gsp->collisions += x*2;
		gsp->excoll += new.tc_txabt - ls->tc_txabt;

		gsp->underflow += new.tc_txundrn - ls->tc_txundrn;
#ifdef NEVER
		gsp->rpackets += rxok + rxer;
		gsp->opackets += txok + txer;
#endif
#endif
	}

	lp->last_stat = new;
	lp->last_stats_valid = B_TRUE;

	/* issue Dump_Tally_Counters_Cmd for the next call */
	bzero(tbp, sizeof (struct rtl8169_tally_counters));
	OUTL(dp, DTCCRL, DTCCR_CMD | (uint32_t)TALLY_DMA_ADDR(dp));

	return (B_TRUE);
}

/* private function */
static void
gani_stat_timeout(void *arg)
{
	struct gem_dev	*dp = arg;
	struct gani_dev	*lp = dp->private;

	mutex_enter(&lp->stat_lock);
	(void) gani_update_stats(dp);
	lp->stat_to_id = timeout(gani_stat_timeout, dp, ONESEC);
	mutex_exit(&lp->stat_lock);
}

static int
gani_get_stats(struct gem_dev *dp)
{
	struct gani_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&lp->stat_lock);
	(void) gani_update_stats(dp);
	mutex_exit(&lp->stat_lock);

	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
gani_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
	struct tx_desc		*tdp;
	uint32_t		mark;
	uint32_t		vlan_tag = 0;
	struct gani_dev		*lp = dp->private;
	int			curslot;
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;
#ifdef GEM_CONFIG_VLAN_HW
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	uint32_t		mss;
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
#endif /* GEM_CONFIG_VLAN_HW */
	uint32_t		tmp0;
	uint32_t		tmp1;
	uint32_t		tmp2;

	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TXD0_OWN | TXD0_LS;
#ifdef GEM_CONFIG_VLAN_HW
#ifdef CONFIG_CKSUM_OFFLOAD
	mss = (GEM_TXFLAG_MSS & flag) >> GEM_TXFLAG_MSS_SHIFT;
	if (mss) {
		mark |= (mss << TXD0_LGSMSS_SHIFT) | TXD0_LGSEN;
	} else if (!lp->new_cksum) {
		if (flag & GEM_TXFLAG_IPv4) {
			mark |= TXD0_IPCS;
		}
		if (flag & GEM_TXFLAG_TCP) {
			mark |= TXD0_TCPCS;
		} else if (flag & GEM_TXFLAG_UDP) {
			mark |= TXD0_UDPCS;
		}
	} else {
		if (flag & GEM_TXFLAG_IPv4) {
			vlan_tag |= TXD1_IPCS;
		}
		if (flag & GEM_TXFLAG_TCP) {
			vlan_tag |= TXD1_TCPCS;
		} else if (flag & GEM_TXFLAG_UDP) {
			vlan_tag |= TXD1_UDPCS;
		}
	}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
	if (flag & GEM_TXFLAG_VTAG) {
		uint32_t	vtag;
		vtag = (flag & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
		vlan_tag |= TXD1_TAGC | TXD1_VID(vtag);
	}
#endif /* GEM_CONFIG_VLAN_HW */

#if DEBUG_LEVEL > 100
if (flag & (GEM_TXFLAG_IPv4 | GEM_TXFLAG_UDP | GEM_TXFLAG_TCP)) {
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum %d, slot %d, frags %d flag 0x%llx, mss %d",
	    dp->name, __func__, dp->tx_active_tail, slot, frags,
	    (unsigned long long)flag, mss);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr 0x%llx, len 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif

#if TX_MAX_FRAGS > 1
	for (i = frags - 1; i > 0; i--) {

		curslot = SLOT(slot + i, tx_ring_size);

		/*
		 * specify descriptor control flags
		 */
		if (curslot == tx_ring_size - 1) {
			mark |= TXD0_EOR;
		}
		tdp = &TXDESC(dp->tx_ring)[curslot];
		tmp0 = (uint32_t)(dmacookie[i].dmac_laddress >> 32);
		tmp1 = (uint32_t)dmacookie[i].dmac_laddress;
		tmp2 = mark | (uint32_t)dmacookie[i].dmac_size;

		tdp->txd3 = LE_32(tmp0);
		tdp->txd2 = LE_32(tmp1);
		tdp->txd1 = LE_32(vlan_tag);
		tdp->txd0 = LE_32(tmp2);
		mark &= ~(TXD0_LS | TXD0_EOR);
	}
#endif /* TX_MAX_FRAGS > 1 */
	/*
	 * specify descriptor control flags for first fragment.
	 */
	mark |= TXD0_FS;
	if (flag & GEM_TXFLAG_HEAD) {
		mark &= ~TXD0_OWN;
	}
	if (slot == tx_ring_size - 1) {
		mark |= TXD0_EOR;
	}

	tdp = &TXDESC(dp->tx_ring)[slot];
	tmp0 = (uint32_t)(dmacookie[i].dmac_laddress >> 32);
	tmp1 = (uint32_t)dmacookie[i].dmac_laddress;
	tmp2 = mark | (uint32_t)dmacookie[i].dmac_size;

	tdp->txd3 = LE_32(tmp0);
	tdp->txd2 = LE_32(tmp1);
	tdp->txd1 = LE_32(vlan_tag);
	tdp->txd0 = LE_32(tmp2);

	return (frags);
}

static void
gani_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint32_t	own;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct gani_dev	*lp = dp->private;

	DPRINTF(5, (CE_CONT, "!%s: %s: start_slot %d, nslot %d",
	    dp->name, __func__, start_slot, nslot));

	ASSERT(nslot > 0);

	own = LE_32(TXD0_OWN);

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	TXDESC(dp->tx_ring)[start_slot].txd0 |= own;
	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);
#ifdef WA_NEW_TX_HANG
	lp->tx_last_desc = start_slot + nslot - 1;
#endif

	/* kick Tx engine */
	ASSERT(IS_MAC_ONLINE(dp));
	OUTB(dp, TPPoll, TPPoll_NPQ);
}

static void
gani_rx_desc_write(struct gem_dev *dp, int slot,
		ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t		mark;
	struct rx_desc		*rdp;
	uint32_t		tmp0;
	uint32_t		tmp1;
	uint32_t		tmp2;

#if DEBUG_LEVEL > 4
{
	int	i;
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum %d, slot %d, frags %d",
	    dp->name, __func__, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr 0x%llx, len 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
	mark = RXD0_OWN;
	if (slot == dp->gc.gc_rx_ring_size - 1) {
		mark = RXD0_OWN | RXD0_EOR;
	}

	rdp = &RXDESC(dp->rx_ring)[slot];

	ASSERT(dmacookie->dmac_size == dp->rx_buf_len);
	tmp0 = (uint32_t)(dmacookie->dmac_laddress >> 32);
	tmp1 = (uint32_t)dmacookie->dmac_laddress;
	tmp2 = mark | (uint32_t)dmacookie->dmac_size;

	rdp->rxd3 = LE_32(tmp0);
	rdp->rxd2 = LE_32(tmp1);
	rdp->rxd1 = 0;
	rdp->rxd0 = LE_32(tmp2);
}

static uint_t
gani_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	tsr;
	struct tx_desc	*tdp;
	struct gani_dev	*lp = dp->private;
	int		frags = ndesc;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

	/* XXX - we must check the last descripor of the packet. */
	tdp = &TXDESC(dp->tx_ring)[SLOT(slot + frags - 1, tx_ring_size)];

	tsr = tdp->txd0;
	tsr = LE_32(tsr);
#ifndef lint
	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d ndesc:%d isr:%b tsr:%b",
	    dp->name, __func__, slot, ndesc,
	    INW(dp, ISR), INTR_BITS, tsr, TXD0_BITS));
#endif
	if (tsr & TXD0_OWN) {
		/* not transmitted yet */
		if (GANI_PCIE(lp)) {
			/* XXX - restart potentially stopped tx engine. */
			OUTB(dp, TPPoll, TPPoll_NPQ);
		}
		return (0);
	}

#ifdef DEBUG_LEVEL
	if (slot + ndesc != tx_ring_size && (tsr & TXD0_EOR)) {
		/* the descriptor is corrupted */
#ifndef lint
		cmn_err(CE_WARN,
		    "!%s: %s: tx descriptor corrupted, slot:%d ndesc:%d tsr:%b",
		    dp->name, __func__, slot, ndesc, tsr, TXD0_BITS);
#endif
		gani_tx_desc_dump(dp);
		return (GEM_TX_DONE | GEM_TX_ERR);
	}
#endif /* DEBUG_LEVEL */
	return (GEM_TX_DONE);
}

#ifdef DEBUG_LEVEL
static void
gani_tx_desc_dump(struct gem_dev *dp)
{
	int		i;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;
	struct tx_desc	*tdp;

	cmn_err(CE_CONT,
	    "!%s: tx descriptor dump: current head:%d[%d], tail:%d[%d]",
	    dp->name,
	    dp->tx_desc_head, (int)SLOT(dp->tx_desc_head, tx_ring_size),
	    dp->tx_desc_tail, (int)SLOT(dp->tx_desc_tail, tx_ring_size));

	tdp = &TXDESC(dp->tx_ring)[0];
	for (i = 0; i < tx_ring_size; i++, tdp++) {
#ifdef lint
		cmn_err(CE_CONT, "%d: %x %x %x %x",
		    i,
		    LE_32(tdp->txd0),
		    LE_32(tdp->txd1),
		    LE_32(tdp->txd2),
		    LE_32(tdp->txd3));
#else
		cmn_err(CE_CONT, "%d: %b %x %x %x",
		    i,
		    LE_32(tdp->txd0), TXD0_BITS,
		    LE_32(tdp->txd1),
		    LE_32(tdp->txd2),
		    LE_32(tdp->txd3));
#endif /* lint */
	}
}
#endif /* DEBUG_LEVEL */

static uint64_t
gani_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint_t		len;
	uint32_t	rsr;
	struct rx_desc	*rdp;
	uint64_t	flags = GEM_RX_DONE;
#ifdef GEM_CONFIG_VLAN_HW
	uint64_t	vtag;
#endif
	struct gani_dev	*lp = dp->private;

	rdp = &RXDESC(dp->rx_ring)[slot];
	rsr = rdp->rxd0;
	rsr = LE_32(rsr);
#ifndef lint
	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d rxd0:0x%b, rxd1:0x%x",
	    dp->name, __func__, slot, rsr, RXD0_BITS, LE_32(rdp->rxd1)));
#endif
	if (rsr & RXD0_OWN) {
		/* not transmitted yet */
		return (0);
	}
#ifndef lint
	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d rxd0:0x%b, rxd1:0x%x",
	    dp->name, __func__, slot, rsr, RXD0_BITS, LE_32(rdp->rxd1)));
#endif
	if ((rsr & (RXD0_RES | RXD0_LS | RXD0_FS)) != (RXD0_LS | RXD0_FS)) {
		if (rsr & RXD0_RES) {
			if (rsr & RXD0_CRC) {
				dp->stats.crc++;
			}
			if (rsr & RXD0_RUNT) {
				dp->stats.runt++;
			}
			dp->stats.errrcv++;
		}

		if ((rsr & (RXD0_LS | RXD0_FS)) != (RXD0_LS | RXD0_FS)) {
			/* received frame is too long */
			if (rsr & RXD0_FS) {
				dp->stats.frame_too_long++;
				dp->stats.errrcv++;
			}
		}
		return (GEM_RX_ERR);
	}

	/* eliminate fcs */
	if ((len = rsr & RXD0_FRAMELEN) < ETHERFCSL) {
		return (GEM_RX_ERR);
	}
	len -= ETHERFCSL;

#ifdef GEM_CONFIG_VLAN_HW
	if ((vtag = LE_32(rdp->rxd1)) & RXD1_TAVA) {
		/* fix vlan tag format */
		flags |= ((uint64_t)BSWAP_16(vtag & 0xffff))
		    << GEM_RX_VTAG_SHIFT;
		DPRINTF(2, (CE_CONT, "%s: %s: vtag detected %llx",
		    dp->name, __func__, (unsigned long long)flags));
	}
#ifdef GEM_CONFIG_CKSUM_OFFLOAD
	if ((rsr & (RXD0_PID | RXD0_TCPF)) == RXD0_PID_TCP) {
		flags |= GEM_RX_CKSUM_TCP;
	} else if ((rsr & (RXD0_PID | RXD0_UDPF)) == RXD0_PID_UDP) {
		flags |= GEM_RX_CKSUM_UDP;
	}
	if (lp->new_cksum) {
		uint32_t	rsr1;
		rsr1 = rdp->rxd1;
		rsr1 = LE_32(rsr1);

		if ((rsr & (RXD0_PID | RXD0_IPF)) == RXD0_PID_IP_NEW &&
		    (rsr1 & RXD1_IPv4)) {
			flags |= GEM_RX_CKSUM_IPv4;
		}
	} else {
		if ((rsr & (RXD0_PID | RXD0_IPF)) == RXD0_PID_IP) {
			flags |= GEM_RX_CKSUM_IPv4;
		}
	}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
#endif /* GEM_CONFIG_VLAN_HW */
#ifdef notdef
{
	int	i;
	uint8_t	*bp = (uint8_t *)&dp->rx_buf_head->rxb_buf[0];

	cmn_err(CE_CONT, "!%s: len:%d", dp->name, len);

	for (i = 0; i < len; i += 10) {
		cmn_err(CE_CONT,
		    "!%02x %02x %02x %02x %02x"
		    " %02x %02x %02x %02x %02x",
		    bp[0], bp[1], bp[2], bp[3], bp[4],
		    bp[5], bp[6], bp[7], bp[8], bp[9]);
		bp += 10;
	}
}
#endif
	return (flags | GEM_RX_DONE | len);
}

static void
gani_tx_desc_init(struct gem_dev *dp, int slot)
{
	TXDESC(dp->tx_ring)[slot].txd0 = 0;
}

static void
gani_rx_desc_init(struct gem_dev *dp, int slot)
{
	RXDESC(dp->rx_ring)[slot].rxd0 = 0;
}

/*
 * Device depend interrupt handler
 */
static uint_t
gani_interrupt(struct gem_dev *dp)
{
	int		i;
	int		nread;
	uint16_t	isr;
	uint16_t	bogus_isr;
	uint16_t	imr_hw;
	uint_t		restart_tx = 0;
	struct gani_dev	*lp = dp->private;
	uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

	isr = INW(dp, ISR);
	bogus_isr = lp->isr_pended;
	lp->isr_pended = 0;

	DPRINTF(4, (CE_CONT, "!%s: time:%ld %s: isr:%b",
	    dp->name, ddi_get_lbolt(), __func__, isr, INTR_BITS));

	imr_hw = lp->imr;
	if (((isr | bogus_isr) & lp->imr) == 0) {
		/* not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: time:%ld %s: isr:%b imr:%b",
	    dp->name, ddi_get_lbolt(), __func__, isr, INTR_BITS,
	    INW(dp, IMR), INTR_BITS));

	if (!IS_MAC_ONLINE(dp)) {
		/* disable further interrupts */
		OUTW(dp, IMR, 0);
		FLSHW(dp, IMR);

		/* ack to all interrupts */
		OUTW(dp, ISR, 0xffff);
		FLSHW(dp, ISR);
		return (DDI_INTR_CLAIMED);
	}

	/* clear interrupt sources explicitly */
	if (lp->pcie_cap) {
		/* disable interrupts */
		OUTW(dp, IMR, imr_hw = 0);
	}
	OUTW(dp, ISR, isr);

#ifdef CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval) {
			/* polling mode */
			lp->imr &= ~(INTR_ROK | INTR_TDU | INTR_TOK);

			switch (lp->busclk) {
			case BUSCLK_PCIE:
				i = dp->poll_interval / (1000 / 125);
				break;

			case BUSCLK_PCI66:
			case BUSCLK_PCI33:
			default:
				i = dp->poll_interval / (1000 / 33);
				break;
			}

			/*
			 * To schedule the next timer interrupt correctly,
			 * we need to reset PCI clock counter. So, we pretend
			 * as we were interrupted from polling timer.
			 */
			isr |= INTR_TimeOut;

			DPRINTF(3, (CE_CONT,
			    "%s: poll_interval:%d", dp->name, i));
		} else {
			/* normal mode, restore interrupt mask */
			lp->imr |= INTR_ROK | INTR_TOK;
			if (GANI_PCIE(lp)) {
				lp->imr |= INTR_TDU;
			}
			i = 0;
		}
		OUTL(dp, TimerInt, i);

		lp->last_poll_interval = dp->poll_interval;
	}

	if (isr & INTR_TimeOut) {
		/*
		 * Reset PCI clock counter to schedule the next polling
		 * timer interrupt.
		 */
		OUTL(dp, TCTR, 0);
	}
#endif /* CONFIG_POLLING */

	/* PCI read/write barrier to ensure the interrupt is ack'ed. */
	/* XXX - it costs 5%-10% of cpu utilazation on 1.8GHz AMD athlon */
	FLSHW(dp, ISR);

	if (isr & INTR_PUN) {
		/*
		 * Link or PHY status has changed
		 */
		DPRINTF(1, (CE_CONT, "!%s: isr:%b", dp->name, isr, INTR_BITS));
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			restart_tx |= INTR_RESTART_TX;
		}
#endif
	}

#ifdef GEM3
	if (isr & INTR_TER) {
		cmn_err(CE_WARN, "!%s: Tx error, isr:%b",
		    dp->name, isr, INTR_BITS);
		lp->need_to_reset = B_TRUE;
	} else if ((GANI_PCIE(lp) && (isr & INTR_TDU)) ||
	    (isr & INTR_TOK) && dp->tx_free_bufs < dp->gc.gc_tx_buf_size / 8) {
		/* Need to kick potentially blocked downstream */
		gem_tx_done(dp);
	}
#endif /* !GEM3 */

	if (isr & (INTR_ROK | INTR_RER | INTR_RDU | INTR_FOVW)) {
		if (isr & INTR_FOVW) {
			if (lp->chip->type == R8168_1 /* RTL8168B rev.B */) {
				lp->need_to_reset = B_TRUE;
				lp->imr &= ~INTR_FOVW;
			}
			dp->stats.overflow++;
			dp->stats.errrcv++;
		}
		nread = gem_receive(dp);
#ifdef TEST_FOVW
		lp->rx_pkt_cnt += nread;
		if ((lp->rx_pkt_cnt % 100000) == 99999) {
			isr |= INTR_FOVW;
		}
#endif
#ifdef CONFIG_POLLING
		if (isr & INTR_RDU) {
			if (dp->poll_interval && dp->poll_pkt_delay > 1 &&
			    GANI_PCIE(lp)) {
				/*
				 * current polling interval is too long.
				 * recompute it.
				 */
				dp->poll_pkt_delay--;
				DPRINTF(-1, (CE_CONT,
				    "!%s: pkt-delay decreased to %d",
				    dp->name, dp->poll_pkt_delay));
			}
			dp->stats.errrcv++;
			dp->stats.norcvbuf++;
		}
#endif /* CONFIG_POLLING */
	}

#ifndef GEM3
	if (isr & INTR_TER) {
		cmn_err(CE_WARN, "!%s: Tx error, isr:%b",
		    dp->name, isr, INTR_BITS);
		lp->need_to_reset = B_TRUE;
	} else if ((GANI_PCIE(lp) && (isr & INTR_TDU)) ||
	    (isr & INTR_TOK)) {
		/* Need to kick potentially blocked down stream */
		if (gem_tx_done(dp)) {
			restart_tx |= INTR_RESTART_TX;
		}
	}
#endif /* !GEM3 */

	if (isr & INTR_SERR) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b",
		    dp->name, isr, INTR_BITS);
		lp->need_to_reset = B_TRUE;
	}

	if (lp->need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);

		restart_tx = INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0 && lp->imr != imr_hw) {
		/* enable interrupts again */
		OUTW(dp, IMR, lp->imr);
	}

	return (DDI_INTR_CLAIMED | restart_tx);
}

/*
 * MII Interfaces
 */
#define	MII_DELAY(dp, r) { (void) INL(dp, (r)); (void) INL(dp, (r)); }

static void
gani_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

#define	GANI_MII_MAX_RETRY	10

static uint16_t
gani_mii_read_raw(struct gem_dev *dp, uint_t index)
{
	int	i;
	int	retry = 0;
	uint_t	ret;
	struct gani_dev	*lp = dp->private;

	if (lp->chip->type == R8168_12 /* RTL8168DP */) {
		OUTL(dp, 0xD0, INL(dp, 0xD0) & ~0x00020000);
	}
again:
	OUTL(dp, PHYAR, index << PHYAR_REGADDR_SHIFT);
	drv_usecwait(20);

	for (i = 0; ((ret = INL(dp, PHYAR)) & PHYAR_FLAG) == 0; i++) {
		if (i > 50) {
			if (retry++ < GANI_MII_MAX_RETRY) {
				goto again;
			}
			cmn_err(CE_WARN, "!%s: %s: timeout, reg:%d",
			    dp->name, __func__, index);
			ret = 0;
			goto x;
		}
		drv_usecwait(20);
	}
	ret &= PHYAR_DATA;
x:
	drv_usecwait(100);
	if (lp->chip->type == R8168_12 /* RTL8168DP */) {
		OUTL(dp, 0xD0, INL(dp, 0xD0) | 0x00020000);
	}

	return ((uint16_t)ret);
}

static uint16_t
gani_phyio_read(struct gem_dev *dp, uint_t index)
{
	int	i;
	uint_t	ret;

	OUTL(dp, PHYIO, PHYIO_READ | (index << PHYIO_REG_SHIFT));
	drv_usecwait(100);

	for (i = 0; (INL(dp, PHYIO) & PHYIO_FLAG) == 0; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "!%s: %s: timeout, reg:%d",
			    dp->name, __func__, index);
			return (0);
		}
		drv_usecwait(100);
	}
	ret = INL(dp, PHYIO) & PHYIO_DATA;
	return ((uint16_t)ret);
}

static void
gani_mii_write_raw(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int	i;
	int	retry = 0;
	struct gani_dev	*lp = dp->private;

	if (lp->chip->type == R8168_12 /* RTL8168DP */) {
		OUTL(dp, 0xD0, INL(dp, 0xD0) & ~0x00020000);
	}
again:
	OUTL(dp, PHYAR,
	    PHYAR_FLAG | (index << PHYAR_REGADDR_SHIFT) | val);
	drv_usecwait(20);

	for (i = 0; (INL(dp, PHYAR) & PHYAR_FLAG) != 0; i++) {
		if (i > 50) {
			if (retry++ < GANI_MII_MAX_RETRY) {
				goto again;
			}
			cmn_err(CE_WARN, "!%s: %s: timeout, reg:%d",
			    dp->name, __func__, index);
			break;
		}
		drv_usecwait(20);
	}
	drv_usecwait(100);

	if (lp->chip->type == R8168_12 /* RTL8168DP */) {
		OUTL(dp, 0xD0, INL(dp, 0xD0) | 0x00020000);
	}
}
#undef	MII_DELAY

static void
gani_phyio_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int	i;

	OUTL(dp, PHYIO,
	    PHYIO_WRITE | (index << PHYIO_REG_SHIFT) | val);
	drv_usecwait(20);

	for (i = 0; INL(dp, PHYIO) & PHYIO_FLAG; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "!%s: %s: timeout, reg:%d",
			    dp->name, __func__, index);
			break;
		}
		drv_usecwait(200);
	}
	drv_usecwait(100);
}

static void
gani_mii_write_ocp(struct gem_dev *dp, uint_t index, uint16_t val)
{
	int	i;

	OUTL(dp, OCPDR, OCPDR_WRITE | (index << OCPDR_GPHY_REG_SHIFT) | val);
	OUTL(dp, OCPAR, OCPAR_GPHY_WRITE);
	OUTL(dp, EPHY_RXER_NUM, 0);

	for (i = 0; INL(dp, OCPAR) & OCPAR_FLAG; i++) {
		if (i > 100) {
			/* timeout */
			cmn_err(CE_WARN,
			    "!%s:%s: timeout", dp->name, __func__);
			break;
		}
		drv_usecwait(2000);
	}
}

static uint16_t
gani_mii_read_ocp(struct gem_dev *dp, uint_t index)
{
	int	i;
	uint16_t	val;

	OUTL(dp, OCPDR, OCPDR_READ | (index << OCPDR_GPHY_REG_SHIFT));
	OUTL(dp, OCPAR, OCPAR_GPHY_WRITE);
	OUTL(dp, EPHY_RXER_NUM, 0);

	for (i = 0; (INL(dp, OCPAR) & OCPAR_FLAG); i++) {
		if (i > 100) {
			/* timeout */
			cmn_err(CE_WARN,
			    "!%s:%s: timeout", dp->name, __func__);
			break;
		}
		drv_usecwait(2000);
	}

	OUTL(dp, OCPAR, OCPAR_GPHY_READ);
	val = INL(dp, OCPDR) & OCPDR_DATA;

	return (val);
}

static uint16_t
gani_mii_read(struct gem_dev *dp, uint_t index)
{
	return (gani_mii_read_raw(dp, index));
}

static void
gani_mii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	struct gani_dev	*lp = dp->private;

	if (index == MII_CONTROL && val == MII_CONTROL_RESET) {
		val |= gani_mii_read(dp, index) & MII_CONTROL_ANE;
	}
	if ((GANI_PCIE_GIGA(lp) || lp->chip->type == R8101_10) &&
	    index == MII_CONTROL && (val & MII_CONTROL_RSAN)) {
		val |= MII_CONTROL_RESET;
	}
	gani_mii_write_raw(dp, index, val);
}

struct gani_patch {
	uint_t	addr:8;
	uint_t	val:16;
};

static struct gani_patch gani_8169_m2_patch[] = {
	/* 012 */
	0x1f, 0x0001, 0x06, 0x006e, 0x08, 0x0708,
	0x15, 0x4000, 0x18, 0x65c7,

	0x1f, 0x0001, 0x03, 0x00a1, 0x02, 0x0008,
	0x01, 0x0120, 0x00, 0x1000, 0x04, 0x0800,
	0x04, 0x0000,

	0x03, 0xff41, 0x02, 0xdf60, 0x01, 0x0140,
	0x00, 0x0077, 0x04, 0x7800, 0x04, 0x7000,

	0x03, 0x802f, 0x02, 0x4f02, 0x01, 0x0409,
	0x00, 0xf0f9, 0x04, 0x9800, 0x04, 0x9000,

	0x03, 0xdf01, 0x02, 0xdf20, 0x01, 0xff95,
	0x00, 0xba00, 0x04, 0xa800, 0x04, 0xa000,

	0x03, 0xff41, 0x02, 0xdf20, 0x01, 0x0140,
	0x00, 0x00bb, 0x04, 0xb800, 0x04, 0xb000,

	0x03, 0xdf41, 0x02, 0xdc60, 0x01, 0x6340,
	0x00, 0x007d, 0x04, 0xd800, 0x04, 0xd000,

	0x03, 0xdf01, 0x02, 0xdf20, 0x01, 0x100a,
	0x00, 0xa0ff, 0x04, 0xf800, 0x04, 0xf000,

	0x1f, 0x0000, 0x0b, 0x0000, 0x00, 0x9200,
};

static struct gani_patch gani_8169_m4_patch[] = {
	/* 012 */
	0x1f, 0x0002, 0x01, 0x90d0, 0x1f, 0x0000,
};

static struct gani_patch gani_8169_m5_patch0[] = {
	/* 012 */
	0x1f, 0x0001, 0x04, 0x0000, 0x03, 0x00a1, 0x02, 0x0008,
	0x01, 0x0120, 0x00, 0x1000, 0x04, 0x0800, 0x04, 0x9000,
	0x03, 0x802f, 0x02, 0x4f02, 0x01, 0x0409, 0x00, 0xf099,
	0x04, 0x9800, 0x04, 0xa000, 0x03, 0xdf01, 0x02, 0xdf20,
	0x01, 0xff95, 0x00, 0xba00, 0x04, 0xa800, 0x04, 0xf000,
	0x03, 0xdf01, 0x02, 0xdf20, 0x01, 0x101a, 0x00, 0xa0ff,
	0x04, 0xf800, 0x04, 0x0000, 0x1f, 0x0000,

	0x1f, 0x0001, 0x10, 0xf41b, 0x14, 0xfb54, 0x18, 0xf5c7,
	0x1f, 0x0000,

	0x1f, 0x0001, 0x17, 0x0cc0, 0x1f, 0x0000,
};

static struct gani_patch gani_8169_m5_patch1[] = {
	/* 012 */
	0x1f, 0x0001, 0x10, 0xf01b, 0x1f, 0x0000,
};

static struct gani_patch gani_8169_m6_patch[] = {
	/* 009 */
	0x1f, 0x0001, 0x04, 0x0000, 0x03, 0x00a1,
	0x02, 0x0008, 0x01, 0x0120, 0x00, 0x1000,
	0x04, 0x0800, 0x04, 0x9000, 0x03, 0x802f,
	0x02, 0x4f02, 0x01, 0x0409, 0x00, 0xf099,
	0x04, 0x9800, 0x04, 0xa000, 0x03, 0xdf01,
	0x02, 0xdf20, 0x01, 0xff95, 0x00, 0xba00,
	0x04, 0xa800, 0x04, 0xf000, 0x03, 0xdf01,
	0x02, 0xdf20, 0x01, 0x101a, 0x00, 0xa0ff,
	0x04, 0xf800, 0x04, 0x0000, 0x1f, 0x0000,

	0x1f, 0x0001, 0x0b, 0x8480, 0x1f, 0x0000,

	0x1f, 0x0001, 0x18, 0x67c7, 0x04, 0x2000,
	0x03, 0x002f, 0x02, 0x4360, 0x01, 0x0109,
	0x00, 0x3022, 0x04, 0x2800, 0x1f, 0x0000,

	0x1f, 0x0001, 0x17, 0x0cc0, 0x1f, 0x0000,
};

static struct gani_patch gani_8168_m1_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x0B, 0x94B0,

	0x1F, 0x0003, 0x12, 0x6096, 0x1F, 0x0000,

	0x0D, 0xF8A0,
	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m2_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x0B, 0x94B0,

	0x1F, 0x0003, 0x12, 0x6096,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m3_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x0B, 0x94B0,

	0x1F, 0x0003, 0x12, 0x6096,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m4_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x12, 0x2300, 0x1F, 0x0000, 0x1F, 0x0003,
	0x16, 0x000A, 0x1F, 0x0000,

	0x1F, 0x0003, 0x12, 0xC096, 0x1F, 0x0000,

	0x1F, 0x0002, 0x00, 0x88DE, 0x01, 0x82B1, 0x1F, 0x0000,

	0x1F, 0x0002, 0x08, 0x9E30, 0x09, 0x01F0, 0x1F, 0x0000,

	0x1F, 0x0002, 0x0A, 0x5500, 0x1F, 0x0000,

	0x1F, 0x0002, 0x03, 0x7002, 0x1F, 0x0000,

	0x1F, 0x0002, 0x0C, 0x00C8, 0x1F, 0x0000,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m5_patch0[] = {
	/* 013 */
	0x1F, 0x0001, 0x12, 0x2300, 0x1F, 0x0003, 0x16, 0x0F0A,
	0x1F, 0x0000,

	0x1F, 0x0002, 0x00, 0x88DE, 0x01, 0x82B1, 0x1F, 0x0000,

	0x1F, 0x0002, 0x0C, 0x7EB8, 0x1F, 0x0000,

	0x1F, 0x0002, 0x06, 0x0761, 0x1F, 0x0000,

	0x1F, 0x0001, 0x03, 0x802F, 0x02, 0x4F02, 0x01, 0x0409,
	0x00, 0xF099, 0x04, 0x9800, 0x04, 0x9000, 0x1F, 0x0000,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m5_patch1[] = {
	/* 013 */
	0x1F, 0x0001, 0x1D, 0x3D98, 0x1F, 0x0000,

	0x1F, 0x0001, 0x17, 0x0CC0, 0x1F, 0x0000,
};

static struct gani_patch gani_8168_m6_patch0[] = {
	/* 013 */
	0x1F, 0x0001, 0x12, 0x2300, 0x1F, 0x0003, 0x16, 0x0F0A,
	0x1F, 0x0000,

	0x1F, 0x0002, 0x00, 0x88DE, 0x01, 0x82B1, 0x1F, 0x0000,

	0x1F, 0x0002, 0x0C, 0x7EB8, 0x1F, 0x0000,

	0x1F, 0x0002, 0x06, 0x0761, 0x1F, 0x0000,

	0x1F, 0x0002, 0x06, 0x5461, 0x1F, 0x0000,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m6_patch1[] = {
	/* 013 */
	0x1F, 0x0001, 0x1D, 0x3D98, 0x1F, 0x0000,

	0x1f, 0x0001, 0x17, 0x0CC0, 0x1F, 0x0000,
};

static struct gani_patch gani_8168_m7_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x1D, 0x3D98,

	0x1F, 0x0001, 0x14, 0xCAA3, 0x1C, 0x000A, 0x18, 0x65D0,

	0x1F, 0x0003, 0x17, 0xB580, 0x18, 0xFF54, 0x19, 0x3954,

	0x1F, 0x0002, 0x0D, 0x310C, 0x0E, 0x310C, 0x0F, 0x311C,
	0x06, 0x0761,

	0x1F, 0x0003, 0x18, 0xFF55, 0x19, 0x3955, 0x18, 0xFF54,
	0x19, 0x3954,

	0x1F, 0x0001, 0x17, 0x0CC0,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m8_patch[] = {
	/* 013 */
	0x1F, 0x0001, 0x14, 0xCAA3, 0x1C, 0x000A, 0x18, 0x65D0,

	0x1F, 0x0003, 0x17, 0xB580, 0x18, 0xFF54, 0x19, 0x3954,

	0x1F, 0x0002, 0x0D, 0x310C, 0x0E, 0x310C, 0x0F, 0x311C,
	0x06, 0x0761,

	0x1F, 0x0003, 0x18, 0xFF55, 0x19, 0x3955, 0x18, 0xFF54,
	0x19, 0x3954,

	0x1F, 0x0001, 0x17, 0x0CC0,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m9_patch0[] = {
	/* 013 */
	0x1F, 0x0001, 0x06, 0x4064, 0x07, 0x2863, 0x08, 0x059C,
	0x09, 0x26B4, 0x0A, 0x6A19, 0x0B, 0xDCC8, 0x10, 0xF06D,
	0x14, 0x7F68, 0x18, 0x7FD9, 0x1C, 0xF0FF, 0x1D, 0x3D9C,
	0x1F, 0x0003, 0x12, 0xF49F, 0x13, 0x070B, 0x1A, 0x05AD,
	0x14, 0x94C0,
};

static struct gani_patch gani_8168_m9_patch1[] = {
	/* 012 */
	0x1F, 0x0001, 0x17, 0x0CC0,

	0x1F, 0x0005, 0x05, 0xFFF6, 0x06, 0x0080, 0x05, 0x8000,

	0x06, 0xF8F9, 0x06, 0xFAEF, 0x06, 0x59EE, 0x06, 0xF8EA,
	0x06, 0x00EE, 0x06, 0xF8EB, 0x06, 0x00E0, 0x06, 0xF87C,
	0x06, 0xE1F8, 0x06, 0x7D59, 0x06, 0x0FEF, 0x06, 0x0139,
	0x06, 0x029E, 0x06, 0x06EF, 0x06, 0x1039, 0x06, 0x089F,
	0x06, 0x2AEE, 0x06, 0xF8EA, 0x06, 0x00EE, 0x06, 0xF8EB,
	0x06, 0x01E0, 0x06, 0xF87C,

	0x06, 0xE1F8, 0x06, 0x7D58, 0x06, 0x409E, 0x06, 0x0F39,
	0x06, 0x46AA, 0x06, 0x0BBF, 0x06, 0x8290, 0x06, 0xD682,
	0x06, 0x9802, 0x06, 0x014F, 0x06, 0xAE09, 0x06, 0xBF82,
	0x06, 0x98D6, 0x06, 0x82A0, 0x06, 0x0201, 0x06, 0x4FEF,
	0x06, 0x95FE, 0x06, 0xFDFC, 0x06, 0x05F8, 0x06, 0xF9FA,
	0x06, 0xEEF8, 0x06, 0xEA00,

	0x06, 0xEEF8, 0x06, 0xEB00, 0x06, 0xE2F8, 0x06, 0x7CE3,
	0x06, 0xF87D, 0x06, 0xA511, 0x06, 0x1112, 0x06, 0xD240,
	0x06, 0xD644, 0x06, 0x4402, 0x06, 0x8217, 0x06, 0xD2A0,
	0x06, 0xD6AA, 0x06, 0xAA02, 0x06, 0x8217, 0x06, 0xAE0F,
	0x06, 0xA544, 0x06, 0x4402, 0x06, 0xAE4D, 0x06, 0xA5AA,
	0x06, 0xAA02, 0x06, 0xAE47, 0x06, 0xAF82, 0x06, 0x13EE,
	0x06, 0x834E, 0x06, 0x00EE, 0x06, 0x834D, 0x06, 0x0FEE,
	0x06, 0x834C, 0x06, 0x0FEE, 0x06, 0x834F, 0x06, 0x00EE,
	0x06, 0x8351, 0x06, 0x00EE, 0x06, 0x834A, 0x06, 0xFFEE,
	0x06, 0x834B, 0x06, 0xFFE0, 0x06, 0x8330, 0x06, 0xE183,
	0x06, 0x3158, 0x06, 0xFEE4, 0x06, 0xF88A, 0x06, 0xE5F8,

	0x06, 0x8BE0, 0x06, 0x8332, 0x06, 0xE183, 0x06, 0x3359,
	0x06, 0x0FE2, 0x06, 0x834D, 0x06, 0x0C24, 0x06, 0x5AF0,
	0x06, 0x1E12, 0x06, 0xE4F8, 0x06, 0x8CE5, 0x06, 0xF88D,
	0x06, 0xAF82, 0x06, 0x13E0, 0x06, 0x834F, 0x06, 0x10E4,
	0x06, 0x834F, 0x06, 0xE083, 0x06, 0x4E78, 0x06, 0x009F,
	0x06, 0x0AE0, 0x06, 0x834F,

	0x06, 0xA010, 0x06, 0xA5EE, 0x06, 0x834E, 0x06, 0x01E0,
	0x06, 0x834E, 0x06, 0x7805, 0x06, 0x9E9A, 0x06, 0xE083,
	0x06, 0x4E78, 0x06, 0x049E, 0x06, 0x10E0, 0x06, 0x834E,
	0x06, 0x7803, 0x06, 0x9E0F, 0x06, 0xE083, 0x06, 0x4E78,
	0x06, 0x019E, 0x06, 0x05AE, 0x06, 0x0CAF, 0x06, 0x81F8,
	0x06, 0xAF81, 0x06, 0xA3AF,

	0x06, 0x81DC, 0x06, 0xAF82, 0x06, 0x13EE, 0x06, 0x8348,
	0x06, 0x00EE, 0x06, 0x8349, 0x06, 0x00E0, 0x06, 0x8351,
	0x06, 0x10E4, 0x06, 0x8351, 0x06, 0x5801, 0x06, 0x9FEA,
	0x06, 0xD000, 0x06, 0xD180, 0x06, 0x1F66, 0x06, 0xE2F8,
	0x06, 0xEAE3, 0x06, 0xF8EB, 0x06, 0x5AF8, 0x06, 0x1E20,
	0x06, 0xE6F8, 0x06, 0xEAE5,

	0x06, 0xF8EB, 0x06, 0xD302, 0x06, 0xB3FE, 0x06, 0xE2F8,
	0x06, 0x7CEF, 0x06, 0x325B, 0x06, 0x80E3, 0x06, 0xF87D,
	0x06, 0x9E03, 0x06, 0x7DFF, 0x06, 0xFF0D, 0x06, 0x581C,
	0x06, 0x551A, 0x06, 0x6511, 0x06, 0xA190, 0x06, 0xD3E2,
	0x06, 0x8348, 0x06, 0xE383, 0x06, 0x491B, 0x06, 0x56AB,
	0x06, 0x08EF, 0x06, 0x56E6,

	0x06, 0x8348, 0x06, 0xE783, 0x06, 0x4910, 0x06, 0xD180,
	0x06, 0x1F66, 0x06, 0xA004, 0x06, 0xB9E2, 0x06, 0x8348,
	0x06, 0xE383, 0x06, 0x49EF, 0x06, 0x65E2, 0x06, 0x834A,
	0x06, 0xE383, 0x06, 0x4B1B, 0x06, 0x56AA, 0x06, 0x0EEF,
	0x06, 0x56E6, 0x06, 0x834A, 0x06, 0xE783, 0x06, 0x4BE2,
	0x06, 0x834D, 0x06, 0xE683,

	0x06, 0x4CE0, 0x06, 0x834D, 0x06, 0xA000, 0x06, 0x0CAF,
	0x06, 0x81DC, 0x06, 0xE083, 0x06, 0x4D10, 0x06, 0xE483,
	0x06, 0x4DAE, 0x06, 0x0480, 0x06, 0xE483, 0x06, 0x4DE0,
	0x06, 0x834E, 0x06, 0x7803, 0x06, 0x9E0B, 0x06, 0xE083,
	0x06, 0x4E78, 0x06, 0x049E, 0x06, 0x04EE, 0x06, 0x834E,
	0x06, 0x02E0, 0x06, 0x8332,

	0x06, 0xE183, 0x06, 0x3359, 0x06, 0x0FE2, 0x06, 0x834D,
	0x06, 0x0C24, 0x06, 0x5AF0, 0x06, 0x1E12, 0x06, 0xE4F8,
	0x06, 0x8CE5, 0x06, 0xF88D, 0x06, 0xE083, 0x06, 0x30E1,
	0x06, 0x8331, 0x06, 0x6801, 0x06, 0xE4F8, 0x06, 0x8AE5,
	0x06, 0xF88B, 0x06, 0xAE37, 0x06, 0xEE83, 0x06, 0x4E03,
	0x06, 0xE083, 0x06, 0x4CE1,

	0x06, 0x834D, 0x06, 0x1B01, 0x06, 0x9E04, 0x06, 0xAAA1,
	0x06, 0xAEA8, 0x06, 0xEE83, 0x06, 0x4E04, 0x06, 0xEE83,
	0x06, 0x4F00, 0x06, 0xAEAB, 0x06, 0xE083, 0x06, 0x4F78,
	0x06, 0x039F, 0x06, 0x14EE, 0x06, 0x834E, 0x06, 0x05D2,
	0x06, 0x40D6, 0x06, 0x5554, 0x06, 0x0282, 0x06, 0x17D2,
	0x06, 0xA0D6, 0x06, 0xBA00,

	0x06, 0x0282, 0x06, 0x17FE, 0x06, 0xFDFC, 0x06, 0x05F8,
	0x06, 0xE0F8, 0x06, 0x60E1, 0x06, 0xF861, 0x06, 0x6802,
	0x06, 0xE4F8, 0x06, 0x60E5, 0x06, 0xF861, 0x06, 0xE0F8,
	0x06, 0x48E1, 0x06, 0xF849, 0x06, 0x580F, 0x06, 0x1E02,
	0x06, 0xE4F8, 0x06, 0x48E5, 0x06, 0xF849, 0x06, 0xD000,
	0x06, 0x0282, 0x06, 0x5BBF,

	0x06, 0x8350, 0x06, 0xEF46, 0x06, 0xDC19, 0x06, 0xDDD0,
	0x06, 0x0102, 0x06, 0x825B, 0x06, 0x0282, 0x06, 0x77E0,
	0x06, 0xF860, 0x06, 0xE1F8, 0x06, 0x6158, 0x06, 0xFDE4,
	0x06, 0xF860, 0x06, 0xE5F8, 0x06, 0x61FC, 0x06, 0x04F9,
	0x06, 0xFAFB, 0x06, 0xC6BF, 0x06, 0xF840, 0x06, 0xBE83,
	0x06, 0x50A0, 0x06, 0x0101,

	0x06, 0x071B, 0x06, 0x89CF, 0x06, 0xD208, 0x06, 0xEBDB,
	0x06, 0x19B2, 0x06, 0xFBFF, 0x06, 0xFEFD, 0x06, 0x04F8,
	0x06, 0xE0F8, 0x06, 0x48E1, 0x06, 0xF849, 0x06, 0x6808,
	0x06, 0xE4F8, 0x06, 0x48E5, 0x06, 0xF849, 0x06, 0x58F7,
	0x06, 0xE4F8, 0x06, 0x48E5, 0x06, 0xF849, 0x06, 0xFC04,
	0x06, 0x4D20, 0x06, 0x0002,

	0x06, 0x4E22, 0x06, 0x0002, 0x06, 0x4DDF, 0x06, 0xFF01,
	0x06, 0x4EDD, 0x06, 0xFF01, 0x06, 0xF8FA, 0x06, 0xFBEF,
	0x06, 0x79BF, 0x06, 0xF822, 0x06, 0xD819, 0x06, 0xD958,
	0x06, 0x849F, 0x06, 0x09BF, 0x06, 0x82BE, 0x06, 0xD682,
	0x06, 0xC602, 0x06, 0x014F, 0x06, 0xEF97, 0x06, 0xFFFE,
	0x06, 0xFC05, 0x06, 0x17FF,

	0x06, 0xFE01, 0x06, 0x1700, 0x06, 0x0102, 0x05, 0x83D8,
	0x06, 0x8051, 0x05, 0x83D6, 0x06, 0x82A0, 0x05, 0x83D4,
	0x06, 0x8000, 0x02, 0x2010, 0x03, 0xDC00, 0x1F, 0x0000,
	0x0B, 0x0600, 0x1F, 0x0005, 0x05, 0xFFF6, 0x06, 0x00FC,
	0x1F, 0x0000,

	0x1F, 0x0000, 0x0D, 0xF880, 0x1F, 0x0000,
};

static struct gani_patch gani_8168_m10_patch0[] = {
	/* 013 */
	0x1F, 0x0001, 0x06, 0x4064, 0x07, 0x2863, 0x08, 0x059C,
	0x09, 0x26B4, 0x0A, 0x6A19, 0x0B, 0xDCC8, 0x10, 0xF06D,
	0x14, 0x7F68, 0x18, 0x7FD9, 0x1C, 0xF0FF, 0x1D, 0x3D9C,
	0x1F, 0x0003, 0x12, 0xF49F, 0x13, 0x070B, 0x1A, 0x05AD,
	0x14, 0x94C0,

	0x1F, 0x0002, 0x06, 0x5561, 0x1F, 0x0005, 0x05, 0x8332,
	0x06, 0x5561,

	0x1F, 0x0000,
};

static struct gani_patch gani_8168_m10_patch1[] = {
	/* 013 */
	0x1F, 0x0005, 0x05, 0xFFC2, 0x1F, 0x0005, 0x05, 0x8000,
	0x06, 0xF8F9, 0x06, 0xFAEE, 0x06, 0xF8EA, 0x06, 0x00EE,
	0x06, 0xF8EB, 0x06, 0x00E2, 0x06, 0xF87C, 0x06, 0xE3F8,
	0x06, 0x7DA5, 0x06, 0x1111, 0x06, 0x12D2, 0x06, 0x40D6,
	0x06, 0x4444, 0x06, 0x0281, 0x06, 0xC6D2, 0x06, 0xA0D6,
	0x06, 0xAAAA, 0x06, 0x0281, 0x06, 0xC6AE, 0x06, 0x0FA5,
	0x06, 0x4444, 0x06, 0x02AE, 0x06, 0x4DA5, 0x06, 0xAAAA,
	0x06, 0x02AE, 0x06, 0x47AF, 0x06, 0x81C2, 0x06, 0xEE83,
	0x06, 0x4E00, 0x06, 0xEE83, 0x06, 0x4D0F, 0x06, 0xEE83,
	0x06, 0x4C0F, 0x06, 0xEE83, 0x06, 0x4F00, 0x06, 0xEE83,
	0x06, 0x5100, 0x06, 0xEE83, 0x06, 0x4AFF, 0x06, 0xEE83,
	0x06, 0x4BFF, 0x06, 0xE083, 0x06, 0x30E1, 0x06, 0x8331,
	0x06, 0x58FE, 0x06, 0xE4F8, 0x06, 0x8AE5, 0x06, 0xF88B,
	0x06, 0xE083, 0x06, 0x32E1, 0x06, 0x8333, 0x06, 0x590F,
	0x06, 0xE283, 0x06, 0x4D0C, 0x06, 0x245A, 0x06, 0xF01E,
	0x06, 0x12E4, 0x06, 0xF88C, 0x06, 0xE5F8, 0x06, 0x8DAF,
	0x06, 0x81C2, 0x06, 0xE083, 0x06, 0x4F10, 0x06, 0xE483,
	0x06, 0x4FE0, 0x06, 0x834E, 0x06, 0x7800, 0x06, 0x9F0A,
	0x06, 0xE083, 0x06, 0x4FA0, 0x06, 0x10A5, 0x06, 0xEE83,
	0x06, 0x4E01, 0x06, 0xE083, 0x06, 0x4E78, 0x06, 0x059E,
	0x06, 0x9AE0, 0x06, 0x834E, 0x06, 0x7804, 0x06, 0x9E10,
	0x06, 0xE083, 0x06, 0x4E78, 0x06, 0x039E, 0x06, 0x0FE0,
	0x06, 0x834E, 0x06, 0x7801, 0x06, 0x9E05, 0x06, 0xAE0C,
	0x06, 0xAF81, 0x06, 0xA7AF, 0x06, 0x8152, 0x06, 0xAF81,
	0x06, 0x8BAF, 0x06, 0x81C2, 0x06, 0xEE83, 0x06, 0x4800,
	0x06, 0xEE83, 0x06, 0x4900, 0x06, 0xE083, 0x06, 0x5110,
	0x06, 0xE483, 0x06, 0x5158, 0x06, 0x019F, 0x06, 0xEAD0,
	0x06, 0x00D1, 0x06, 0x801F, 0x06, 0x66E2, 0x06, 0xF8EA,
	0x06, 0xE3F8, 0x06, 0xEB5A, 0x06, 0xF81E, 0x06, 0x20E6,
	0x06, 0xF8EA, 0x06, 0xE5F8, 0x06, 0xEBD3, 0x06, 0x02B3,
	0x06, 0xFEE2, 0x06, 0xF87C, 0x06, 0xEF32, 0x06, 0x5B80,
	0x06, 0xE3F8, 0x06, 0x7D9E, 0x06, 0x037D, 0x06, 0xFFFF,
	0x06, 0x0D58, 0x06, 0x1C55, 0x06, 0x1A65, 0x06, 0x11A1,
	0x06, 0x90D3, 0x06, 0xE283, 0x06, 0x48E3, 0x06, 0x8349,
	0x06, 0x1B56, 0x06, 0xAB08, 0x06, 0xEF56, 0x06, 0xE683,
	0x06, 0x48E7, 0x06, 0x8349, 0x06, 0x10D1, 0x06, 0x801F,
	0x06, 0x66A0, 0x06, 0x04B9, 0x06, 0xE283, 0x06, 0x48E3,
	0x06, 0x8349, 0x06, 0xEF65, 0x06, 0xE283, 0x06, 0x4AE3,
	0x06, 0x834B, 0x06, 0x1B56, 0x06, 0xAA0E, 0x06, 0xEF56,
	0x06, 0xE683, 0x06, 0x4AE7, 0x06, 0x834B, 0x06, 0xE283,
	0x06, 0x4DE6, 0x06, 0x834C, 0x06, 0xE083, 0x06, 0x4DA0,
	0x06, 0x000C, 0x06, 0xAF81, 0x06, 0x8BE0, 0x06, 0x834D,
	0x06, 0x10E4, 0x06, 0x834D, 0x06, 0xAE04, 0x06, 0x80E4,
	0x06, 0x834D, 0x06, 0xE083, 0x06, 0x4E78, 0x06, 0x039E,
	0x06, 0x0BE0, 0x06, 0x834E, 0x06, 0x7804, 0x06, 0x9E04,
	0x06, 0xEE83, 0x06, 0x4E02, 0x06, 0xE083, 0x06, 0x32E1,
	0x06, 0x8333, 0x06, 0x590F, 0x06, 0xE283, 0x06, 0x4D0C,
	0x06, 0x245A, 0x06, 0xF01E, 0x06, 0x12E4, 0x06, 0xF88C,
	0x06, 0xE5F8, 0x06, 0x8DE0, 0x06, 0x8330, 0x06, 0xE183,
	0x06, 0x3168, 0x06, 0x01E4, 0x06, 0xF88A, 0x06, 0xE5F8,
	0x06, 0x8BAE, 0x06, 0x37EE, 0x06, 0x834E, 0x06, 0x03E0,
	0x06, 0x834C, 0x06, 0xE183, 0x06, 0x4D1B, 0x06, 0x019E,
	0x06, 0x04AA, 0x06, 0xA1AE, 0x06, 0xA8EE, 0x06, 0x834E,
	0x06, 0x04EE, 0x06, 0x834F, 0x06, 0x00AE, 0x06, 0xABE0,
	0x06, 0x834F, 0x06, 0x7803, 0x06, 0x9F14, 0x06, 0xEE83,
	0x06, 0x4E05, 0x06, 0xD240, 0x06, 0xD655, 0x06, 0x5402,
	0x06, 0x81C6, 0x06, 0xD2A0, 0x06, 0xD6BA, 0x06, 0x0002,
	0x06, 0x81C6, 0x06, 0xFEFD, 0x06, 0xFC05, 0x06, 0xF8E0,
	0x06, 0xF860, 0x06, 0xE1F8, 0x06, 0x6168, 0x06, 0x02E4,
	0x06, 0xF860, 0x06, 0xE5F8, 0x06, 0x61E0, 0x06, 0xF848,
	0x06, 0xE1F8, 0x06, 0x4958, 0x06, 0x0F1E, 0x06, 0x02E4,
	0x06, 0xF848, 0x06, 0xE5F8, 0x06, 0x49D0, 0x06, 0x0002,
	0x06, 0x820A, 0x06, 0xBF83, 0x06, 0x50EF, 0x06, 0x46DC,
	0x06, 0x19DD, 0x06, 0xD001, 0x06, 0x0282, 0x06, 0x0A02,
	0x06, 0x8226, 0x06, 0xE0F8, 0x06, 0x60E1, 0x06, 0xF861,
	0x06, 0x58FD, 0x06, 0xE4F8, 0x06, 0x60E5, 0x06, 0xF861,
	0x06, 0xFC04, 0x06, 0xF9FA, 0x06, 0xFBC6, 0x06, 0xBFF8,
	0x06, 0x40BE, 0x06, 0x8350, 0x06, 0xA001, 0x06, 0x0107,
	0x06, 0x1B89, 0x06, 0xCFD2, 0x06, 0x08EB, 0x06, 0xDB19,
	0x06, 0xB2FB, 0x06, 0xFFFE, 0x06, 0xFD04, 0x06, 0xF8E0,
	0x06, 0xF848, 0x06, 0xE1F8, 0x06, 0x4968, 0x06, 0x08E4,
	0x06, 0xF848, 0x06, 0xE5F8, 0x06, 0x4958, 0x06, 0xF7E4,
	0x06, 0xF848, 0x06, 0xE5F8, 0x06, 0x49FC, 0x06, 0x044D,
	0x06, 0x2000, 0x06, 0x024E, 0x06, 0x2200, 0x06, 0x024D,
	0x06, 0xDFFF, 0x06, 0x014E, 0x06, 0xDDFF, 0x06, 0x0100,
	0x05, 0x83D8, 0x06, 0x8000, 0x03, 0xDC00, 0x05, 0xFFF6,
	0x06, 0x00FC, 0x1F, 0x0000,

	0x1F, 0x0000, 0x0D, 0xF880, 0x1F, 0x0000,
};

static struct gani_patch gani_8168_m11_patch[] = {
	/* 013 */
	0x1F, 0x0002, 0x10, 0x0008, 0x0D, 0x006C,

	0x1F, 0x0000, 0x0D, 0xF880,

	0x1F, 0x0001, 0x17, 0x0CC0,

	0x1F, 0x0001, 0x0B, 0xA4D8, 0x09, 0x281C, 0x07, 0x2883,
	0x0A, 0x6B35, 0x1D, 0x3DA4, 0x1C, 0xEFFD, 0x14, 0x7F52,
	0x18, 0x7FC6, 0x08, 0x0601, 0x06, 0x4063, 0x10, 0xF074,
	0x1F, 0x0003, 0x13, 0x0789, 0x12, 0xF4BD, 0x1A, 0x04FD,
	0x14, 0x84B0, 0x1F, 0x0000, 0x00, 0x9200,

	0x1F, 0x0005, 0x01, 0x0340, 0x1F, 0x0001, 0x04, 0x4000,
	0x03, 0x1D21, 0x02, 0x0C32, 0x01, 0x0200, 0x00, 0x5554,
	0x04, 0x4800, 0x04, 0x4000, 0x04, 0xF000, 0x03, 0xDF01,
	0x02, 0xDF20, 0x01, 0x101A, 0x00, 0xA0FF, 0x04, 0xF800,
	0x04, 0xF000, 0x1F, 0x0000,

	0x1F, 0x0007, 0x1E, 0x0023, 0x16, 0x0000, 0x1F, 0x0000,
};

#define	GANI_PHY_PATCH(t)	{ \
	int	i;	\
	for (i = 0; i < sizeof (t) / sizeof ((t)[0]); i++) {	\
		gani_mii_write_raw(dp, (t)[i].addr, (t)[i].val);	\
	}	\
}

#define	GANI_PHY_PATCH_OCP(t)	{ \
	int	i;	\
	for (i = 0; i < sizeof (t) / sizeof ((t)[0]); i++) {	\
		gani_mii_write_ocp(dp, (t)[i].addr, (t)[i].val);	\
	}	\
}

static void
gani_patch_phy(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct gani_dev	*lp = dp->private;

	switch (lp->chip->type) {
	case R8169_2: /* RTL8169S rev.D */
	case R8169_3: /* RTL8169S rev.E */
		GANI_PHY_PATCH(gani_8169_m2_patch);
		break;

	case R8169_4: /* RTL8169SB */
		GANI_PHY_PATCH(gani_8169_m4_patch);
		break;

	case R8169_5: /* RTL8169SC rev.D */
		GANI_PHY_PATCH(gani_8169_m5_patch0);
		if (lp->svid == 0x1458 && lp->sdid == 0xe000) {
			GANI_PHY_PATCH(gani_8169_m5_patch1);
		}
		break;

	case R8169_6: /* RTL8169SC rev.E */
		GANI_PHY_PATCH(gani_8169_m6_patch);
		break;

	case R8168_1: /* RTL8168B rev.B */
		GANI_PHY_PATCH(gani_8168_m1_patch);
		break;

	case R8168_2: /* RTL8168B rev.E */
		GANI_PHY_PATCH(gani_8168_m2_patch);
		break;

	case R8168_3: /* RTL8168B rev.F */
		GANI_PHY_PATCH(gani_8168_m3_patch);
		break;

	case R8168_4: /* RTL8168C */
		/* 013 */
		GANI_PHY_PATCH(gani_8168_m4_patch);

		gani_mii_write_raw(dp, 0x14,
		    gani_mii_read_raw(dp, 0x14) | 0x0020);
		gani_mii_write_raw(dp, 0x0d,
		    gani_mii_read_raw(dp, 0x0d) & ~0x0020);
		break;

	case R8168_5: /* RTL8168C rev.B */
		GANI_PHY_PATCH(gani_8168_m5_patch0);

		gani_mii_write_raw(dp, 0x16,
		    gani_mii_read_raw(dp, 0x16) | 0x0001);

		gani_mii_write_raw(dp, 0x1F, 0x0000);
		gani_mii_write_raw(dp, 0x14,
		    gani_mii_read_raw(dp, 0x14) | 0x0020);
		gani_mii_write_raw(dp, 0x0D,
		    gani_mii_read_raw(dp, 0x0D) & ~0x0020);

		GANI_PHY_PATCH(gani_8168_m5_patch1);
		break;

	case R8168_6: /* RTL8168C rev.C */
		GANI_PHY_PATCH(gani_8168_m6_patch0);

		gani_mii_write_raw(dp, 0x16,
		    gani_mii_read_raw(dp, 0x16) | 0x0001);

		gani_mii_write_raw(dp, 0x1F, 0x0000);
		gani_mii_write_raw(dp, 0x14,
		    gani_mii_read_raw(dp, 0x14) | 0x0020);
		gani_mii_write_raw(dp, 0x0D,
		    gani_mii_read_raw(dp, 0x0D) & ~0x0020);

		GANI_PHY_PATCH(gani_8168_m6_patch1);
		break;

	case R8168_7: /* RTL8168CP rev.B */
		gani_mii_write_raw(dp, 0x1F, 0x0000);
		gani_mii_write_raw(dp, 0x14,
		    gani_mii_read_raw(dp, 0x14) | 0x0020);
		gani_mii_write_raw(dp, 0x0D,
		    gani_mii_read_raw(dp, 0x0D) & ~0x0020);

		GANI_PHY_PATCH(gani_8168_m7_patch);
		break;

	case R8168_8: /* RTL8168CP rev.C */
		gani_mii_write_raw(dp, 0x1F, 0x0000);
		gani_mii_write_raw(dp, 0x14,
		    gani_mii_read_raw(dp, 0x14) | 0x0020);
		gani_mii_write_raw(dp, 0x0D,
		    gani_mii_read_raw(dp, 0x0D) & ~0x0020);

		GANI_PHY_PATCH(gani_8168_m8_patch);

		gani_mii_write_raw(dp, 0x16,
		    gani_mii_read_raw(dp, 0x16) | 0x0001);
		gani_mii_write_raw(dp, 0x1F, 0x0000);
		break;

	case R8168_9: /* RTL8168D */
#if 1 /* defined(CONFIG_PATCH_8168D) tested */
		GANI_PHY_PATCH(gani_8168_m9_patch0);

		gani_mii_write_raw(dp, 0x1F, 0x0002);
		val = gani_mii_read_raw(dp, 0x0B) & 0xFF00;
		val |= 0x10;
		gani_mii_write_raw(dp, 0x0B, val);
		val = gani_mii_read_raw(dp, 0x0C) & 0x00FF;
		val |= 0xA200;
		gani_mii_write_raw(dp, 0x0C, val);

		/* 013 */
		gani_mii_write_raw(dp, 0x1F, 0x0002);
		gani_mii_write_raw(dp, 0x06, 0x5561);
		gani_mii_write_raw(dp, 0x1F, 0x0005);
		gani_mii_write_raw(dp, 0x05, 0x8332);
		gani_mii_write_raw(dp, 0x06, 0x5561);

		if (gani_efuse_read(dp, 0x01) == 0xb1) {
			/* 013 */
			gani_mii_write_raw(dp, 0x1F, 0x0002);
			gani_mii_write_raw(dp, 0x05, 0x669A);
			gani_mii_write_raw(dp, 0x1F, 0x0005);
			gani_mii_write_raw(dp, 0x05, 0x8330);
			gani_mii_write_raw(dp, 0x06, 0x669A);

			gani_mii_write_raw(dp, 0x1F, 0x0002);
			val = gani_mii_read_raw(dp, 0x0D);
			if ((val & 0x00FF) != 0x006C) {
				/* 013 */
				val &= 0xFF00;
				gani_mii_write_raw(dp, 0x1F, 0x0002);
				gani_mii_write_raw(dp, 0x0D, val | 0x0065);
				gani_mii_write_raw(dp, 0x0D, val | 0x0066);
				gani_mii_write_raw(dp, 0x0D, val | 0x0067);
				gani_mii_write_raw(dp, 0x0D, val | 0x0068);
				gani_mii_write_raw(dp, 0x0D, val | 0x0069);
				gani_mii_write_raw(dp, 0x0D, val | 0x006A);
				gani_mii_write_raw(dp, 0x0D, val | 0x006B);
				gani_mii_write_raw(dp, 0x0D, val | 0x006C);
			}
		} else {
			/* 013 */
			gani_mii_write_raw(dp, 0x1F, 0x0002);
			gani_mii_write_raw(dp, 0x05, 0x6662);
			gani_mii_write_raw(dp, 0x1F, 0x0005);
			gani_mii_write_raw(dp, 0x05, 0x8330);
			gani_mii_write_raw(dp, 0x06, 0x6662);
		}

		gani_mii_write_raw(dp, 0x1F, 0x0002);
		val = gani_mii_read_raw(dp, 0x0D);
		val |= 0x0300;
		gani_mii_write_raw(dp, 0x0D, val);
		val = gani_mii_read_raw(dp, 0x0F);
		val |= 0x0010;
		gani_mii_write_raw(dp, 0x0F, val);

		gani_mii_write_raw(dp, 0x1F, 0x0002);
		val = gani_mii_read_raw(dp, 0x02);
		val &= ~0x0600;
		val |= 0x0100;
		gani_mii_write_raw(dp, 0x02, val);
		val = gani_mii_read_raw(dp, 0x03);
		val &= ~0xe000;
		gani_mii_write_raw(dp, 0x03, val);

		GANI_PHY_PATCH(gani_8168_m9_patch1);
#endif /* CONFIG_PATCH_8168D */
		break;

	case R8168_10: /* RTL8168D rev.C */
#ifdef CONFIG_PATCH_8168D_C
		GANI_PHY_PATCH(gani_8168_m10_patch0);

		if (gani_efuse_read(dp, 0x01) == 0xb1) {
			/* 013 */
			gani_mii_write_raw(dp, 0x1F, 0x0002);
			gani_mii_write_raw(dp, 0x05, 0x669A);
			gani_mii_write_raw(dp, 0x1F, 0x0005);
			gani_mii_write_raw(dp, 0x05, 0x8330);
			gani_mii_write_raw(dp, 0x06, 0x669A);

			gani_mii_write_raw(dp, 0x1F, 0x0002);
			val = gani_mii_read_raw(dp, 0x0D);
			if ((val & 0x00FF) != 0x006C) {
				/* 013 */
				val &= 0xFF00;
				gani_mii_write_raw(dp, 0x1F, 0x0002);
				gani_mii_write_raw(dp, 0x0D, val | 0x0065);
				gani_mii_write_raw(dp, 0x0D, val | 0x0066);
				gani_mii_write_raw(dp, 0x0D, val | 0x0067);
				gani_mii_write_raw(dp, 0x0D, val | 0x0068);
				gani_mii_write_raw(dp, 0x0D, val | 0x0069);
				gani_mii_write_raw(dp, 0x0D, val | 0x006A);
				gani_mii_write_raw(dp, 0x0D, val | 0x006B);
				gani_mii_write_raw(dp, 0x0D, val | 0x006C);
			}
		} else {
			/* 013 */
			gani_mii_write_raw(dp, 0x1F, 0x0002);
			gani_mii_write_raw(dp, 0x05, 0x2642);
			gani_mii_write_raw(dp, 0x1F, 0x0005);
			gani_mii_write_raw(dp, 0x05, 0x8330);
			gani_mii_write_raw(dp, 0x06, 0x2642);
		}

		/* 013 */
		gani_mii_write_raw(dp, 0x1F, 0x0002);
		val = gani_mii_read_raw(dp, 0x02);
		val &= ~0x0600;
		val |= 0x0100;
		gani_mii_write_raw(dp, 0x02, val);
		val = gani_mii_read_raw(dp, 0x03);
		val &= ~0xe000;
		gani_mii_write_raw(dp, 0x03, val);

		gani_mii_write_raw(dp, 0x1F, 0x0001);
		gani_mii_write_raw(dp, 0x17, 0x0CC0);

		gani_mii_write_raw(dp, 0x1F, 0x0002);
		val = gani_mii_read_raw(dp, 0x0F);
		val |= 0x0017;
		gani_mii_write_raw(dp, 0x0F, val);

		GANI_PHY_PATCH(gani_8168_m10_patch1);
#endif /* CONFIG_PATCH_8168D_C */
		break;

	case R8168_11: /* RTL8168DP */
#ifdef CONFIG_PATCH_8168DP
		/* 013 */
		GANI_PHY_PATCH_OCP(gani_8168_m11_patch);
#endif
		break;

	case R8168_12:
		break;

	case R8168_13:
		break;

	case R8168_14:
		break;

	case R8101_4: /* RTL8102E */
	case R8101_5: /* RTL8102E rev.B */
		/* 015 */
		gani_mii_write_raw(dp, 0x1f, 0x0000);
		gani_mii_write_raw(dp, 0x11,
		    gani_mii_read_raw(dp, 0x11) | 0x1000);
		gani_mii_write_raw(dp, 0x19,
		    gani_mii_read_raw(dp, 0x19) | 0x2000);
		gani_mii_write_raw(dp, 0x10,
		    gani_mii_read_raw(dp, 0x10) | 0x8000);

		gani_mii_write_raw(dp, 0x1f, 0x0003);
		gani_mii_write_raw(dp, 0x08, 0x441D);
		gani_mii_write_raw(dp, 0x01, 0x9100);

		gani_mii_write_raw(dp, 0x1f, 0x0000);
		break;

	case R8101_6: /* RTL8103E */
		/* 015 */
		gani_mii_write_raw(dp, 0x1f, 0x0000);
		gani_mii_write_raw(dp, 0x11,
		    gani_mii_read_raw(dp, 0x11) | 0x1000);
		gani_mii_write_raw(dp, 0x19,
		    gani_mii_read_raw(dp, 0x19) | 0x2000);
		gani_mii_write_raw(dp, 0x10,
		    gani_mii_read_raw(dp, 0x10) | 0x8000);

		gani_mii_write_raw(dp, 0x1f, 0x0003);
		gani_mii_write_raw(dp, 0x08, 0x441D);

		gani_mii_write_raw(dp, 0x1f, 0x0000);
		break;

	case R8101_8: /* RTL8401E */
		gani_phyio_write(dp, 0x0e, 0x0068);
		gani_phyio_write(dp, 0x0e, 0x0069);
		gani_phyio_write(dp, 0x0e, 0x006a);
		gani_phyio_write(dp, 0x0e, 0x006b);
		gani_phyio_write(dp, 0x0e, 0x000c);
		break;
	}
}

static int
gani_mii_probe(struct gem_dev *dp)
{
	uint_t	val;
	int	ret = GEM_SUCCESS;

	/* select page 0 */
	sema_p(&dp->hal_op_lock);
	gani_mii_write_raw(dp, 0x1f, 0x0000);

	dp->mii_status = gani_mii_read_raw(dp, MII_STATUS);
	if (dp->mii_status == 0xffff || dp->mii_status == 0) {
		cmn_err(CE_NOTE,
		    "!%s: failed to probe default internal and/or non-MII PHY",
		    dp->name);
		ret = GEM_FAILURE;
		goto x;
	}

	/* ensure phy is powered up */
	val = gani_mii_read_raw(dp, MII_CONTROL);
	gani_mii_write_raw(dp, MII_CONTROL,
	    val & (MII_CONTROL_SPEED | MII_CONTROL_ANE));

	dp->mii_status_ro = ~dp->mii_status;
	dp->mii_phy_id  = (gani_mii_read_raw(dp, MII_PHYIDH) << 16) |
	    gani_mii_read_raw(dp, MII_PHYIDL);

	cmn_err(CE_CONT, "!%s: using internal/non-MII PHY(0x%08x)",
	    dp->name, dp->mii_phy_id);

	cmn_err(CE_CONT,
	    "!%s: PHY control:%b, status:%b, advert:%b, lpar:%b, exp:%b",
	    dp->name,
	    gani_mii_read_raw(dp, MII_CONTROL), MII_CONTROL_BITS,
	    dp->mii_status, MII_STATUS_BITS,
	    gani_mii_read_raw(dp, MII_AN_ADVERT), MII_ABILITY_BITS,
	    gani_mii_read_raw(dp, MII_AN_LPABLE), MII_ABILITY_BITS,
	    gani_mii_read_raw(dp, MII_AN_EXPANSION), MII_AN_EXP_BITS);

	dp->mii_xstatus = 0;
	if (dp->mii_status & MII_STATUS_XSTATUS) {
		dp->mii_xstatus = gani_mii_read_raw(dp, MII_XSTATUS);

		cmn_err(CE_CONT, "!%s: xstatus:%b",
		    dp->name, dp->mii_xstatus, MII_XSTATUS_BITS);

		dp->mii_xstatus &=
		    ~ (MII_XSTATUS_1000BASET | MII_XSTATUS_1000BASEX);
	}
	dp->mii_xstatus_ro = ~dp->mii_xstatus;
x:
	sema_v(&dp->hal_op_lock);
	return (GEM_SUCCESS);
}

static int
gani_mii_init(struct gem_dev *dp)
{
	struct gani_dev		*lp = dp->private;

	/* install latest patch for phy */
	sema_p(&dp->hal_op_lock);
	gani_patch_phy(dp);

	/* select page 0 */
	gani_mii_write_raw(dp, 0x1f, 0x0000);

	if (GANI_PCI(lp)) {
		OUTB(dp, 0x82, 1);
		/* XXX - set pci latency timer to 0x40 */
		if (lp->chip->type == R8169_2 /* RTL8169S rev.D */) {
			OUTB(dp, 0x82, 1);
			gani_mii_write_raw(dp, 0x0b, 0x0000);
		}

		/* power up */
		gani_mii_write_raw(dp, 0x0e, 0x0000);
	}

	if (GANI_PCIE_GIGA(lp)) {
		switch (lp->chip->type) {
		case R8168_9:
		case R8168_10:
		case R8168_12:
		case R8168_14:
		case R8168_15:
			OUTB(dp, PMCH, INB(dp, PMCH) | 0x80);
		}

		/* power up */
		switch (lp->chip->type) {
		case R8168_14:
		case R8168_15:
			break;
		default:
			gani_mii_write_raw(dp, 0x0e, 0x0000);
			break;
		}
		gani_mii_write_raw(dp, MII_CONTROL, MII_CONTROL_ANE);
	}
	sema_v(&dp->hal_op_lock);

	return (GEM_SUCCESS);
}

static int
gani_mii_config(struct gem_dev *dp)
{
	int		ret;
	uint16_t	val;
	uint_t		anadv_1000hdx_org;
	struct gani_dev	*lp = dp->private;

	/*
	 * rtl816x nics seem to have a problem in forced mode. The link
	 * never go up again after once it become down from up.
	 */
	if ((!dp->anadv_autoneg) && dp->speed == GEM_SPD_1000) {
		cmn_err(CE_NOTE,
		    "!%s: rtl816x series does not support fixed mode for "
		    "1Gbps connection. "
		    "limited auto negotiation capability is used "
		    "instead of given forced mode.",
		    dp->name);
		dp->anadv_1000fdx = B_TRUE;
		dp->anadv_1000hdx = B_FALSE;
		dp->anadv_100fdx = B_TRUE;
		dp->anadv_100hdx = B_TRUE;
		dp->anadv_10fdx = B_TRUE;
		dp->anadv_10hdx = B_TRUE;
#ifdef SANITY
		dp->full_duplex = B_TRUE;
#endif
		dp->anadv_autoneg = B_TRUE;
	}

	/*
	 * the datasheet says rtl816x doesn't support 1Gbps half duplex
	 * mode, but the phy unit in rtl8168 seem to require 1000hdx bit
	 * in 1000TC register set for 1G bps negotiation.
	 */
	anadv_1000hdx_org = dp->anadv_1000hdx;
	dp->anadv_1000hdx = dp->anadv_1000fdx;

	if (GANI_PCI(lp) &&
	    dp->anadv_autoneg && dp->anadv_1000fdx &&
	    (dp->anadv_100fdx | dp->anadv_100hdx |
	    dp->anadv_10fdx | dp->anadv_10hdx) == B_FALSE) {
		/*
		 * At least one capability in 100Mbps or 10Mbps should be
		 * enabled. Otherwise autonegotion will timeout.
		 */
		dp->anadv_100fdx = B_TRUE;
		dp->anadv_100hdx = B_TRUE;
		dp->anadv_10fdx = B_TRUE;
		dp->anadv_10hdx = B_TRUE;
	}

	/* select page 0 */
	sema_p(&dp->hal_op_lock);
	gani_mii_write_raw(dp, 0x1f, 0x0000);

	if (GANI_PCIE_GIGA(lp) && dp->anadv_autoneg) {
		/* power up phy */
		switch (lp->chip->type) {
		case R8168_14:
		case R8168_15:
			break;
		default:
			gani_mii_write_raw(dp, 0x0e, 0x0000);
			break;
		}
		gani_mii_write_raw(dp, MII_CONTROL, MII_CONTROL_ANE);

	} else if (GANI_PCIE_FAST(lp) && dp->anadv_autoneg) {
		switch (lp->chip->type) {
		case R8101_1: /* RTL8101E rev.B */
		case R8101_2: /* RTL8101E rev.E */
		case R8101_3: /* RTL8101E rev.F */
			if (dp->anadv_100fdx || dp->anadv_100hdx) {
				break;
			}
			/* FALL THROU */

		case R8101_4: /* RTL8102E */
		case R8101_5: /* RTL8102E rev.B */
			gani_mii_write_raw(dp,
			    MII_CONTROL, MII_CONTROL_RESET);
			drv_usecwait(100);
			gani_patch_phy(dp);

			/* select page 0 */
			gani_mii_write_raw(dp, 0x1f, 0x0000);
			break;
		}
	}
	sema_v(&dp->hal_op_lock);
	ret = gem_mii_config_default(dp);
	sema_p(&dp->hal_op_lock);

	dp->anadv_1000hdx = anadv_1000hdx_org;

	if (lp->chip->type == R8169_2 /* RTL8169S rev.D */ ||
	    lp->chip->type == R8169_3 /* RTL8169S rev.E */) {
		if ((!dp->anadv_autoneg) && dp->speed == GEM_SPD_100) {
			gani_mii_write_raw(dp, 0x17, 0x2138);
			gani_mii_write_raw(dp, 0x0e, 0x0260);
		} else {
			gani_mii_write_raw(dp, 0x17, 0x2108);
			gani_mii_write_raw(dp, 0x0e, 0x0000);
		}
	}
	sema_v(&dp->hal_op_lock);
	return (ret);
}

/* ======================================================== */
/*
 * OS depend (device driver kernel interface) routine
 */
/* ======================================================== */
#define	gani_eeprom_delay(dp)	\
	{ (void) INB(dp, CR9346); drv_usecwait(3); }

static uint16_t
gani_read_eeprom(struct gem_dev *dp, int addr)
{
	int		i;
	int		addr_bits;
	uint_t		cmd;
	uint8_t		chip_select;
	uint8_t		di;
	uint16_t	ret;

	addr_bits = (INL(dp, RCR) & RCR_9356SEL) ? 8 : 6;

	DPRINTF(4, (CE_CONT, "!%s: %s: called: addr_bits:%d",
	    dp->name, __func__, addr_bits));

	/* make command bits */
	cmd = (6 << addr_bits) | addr;

	/* enable eeprom interface register */
	chip_select = INB(dp, CR9346);
	chip_select &= ~(CR9346_EEDI | CR9346_EEDO | CR9346_EEM | CR9346_EESK);
	chip_select |= CR9346_EEM_PROGRAM | CR9346_EECS;
	OUTB(dp, CR9346, chip_select);
	gani_eeprom_delay(dp);

	/* output eeprom command */
	for (i = 3 + addr_bits - 1; i >= 0; i--) {
		di = ((cmd >> i) << CR9346_EEDI_SHIFT) & CR9346_EEDI;
		OUTB(dp, CR9346, chip_select | di);
		gani_eeprom_delay(dp);

		OUTB(dp, CR9346, chip_select | di | CR9346_EESK);
		gani_eeprom_delay(dp);

		OUTB(dp, CR9346, chip_select | di);
		gani_eeprom_delay(dp);
	}

	/* get returned value */
#ifdef lint
	ret = ret;
#endif
	for (i = 16; i > 0; i--) {
		/* get 1 bit */
		OUTB(dp, CR9346, chip_select | CR9346_EESK);
		gani_eeprom_delay(dp);

		ret = (ret << 1) |
		    ((INB(dp, CR9346) >> CR9346_EEDO_SHIFT) & 1);

		OUTB(dp, CR9346, chip_select);
		gani_eeprom_delay(dp);
	}

	/* Terminate the EEPROM access. */
	chip_select &= ~CR9346_EECS;
	OUTB(dp, CR9346, chip_select | CR9346_EESK);
	gani_eeprom_delay(dp);
	OUTB(dp, CR9346, chip_select);
	gani_eeprom_delay(dp);

	chip_select &= ~CR9346_EEM;	/* i.e. CR9346_EEM_NORMAL */
	OUTB(dp, CR9346, chip_select);

	DPRINTF(4, (CE_CONT, "!%s: %s: returned 0x%x",
	    dp->name, __func__, ret));

	return (ret);
}

#if DEBUG_LEVEL > 0
static void
gani_eeprom_dump(struct gem_dev *dp, int size)
{
	int		i;
	uint16_t	x0, x1, x2, x3;

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < size; i += 4) {
		x0 = gani_read_eeprom(dp, i + 0);
		x1 = gani_read_eeprom(dp, i + 1);
		x2 = gani_read_eeprom(dp, i + 2);
		x3 = gani_read_eeprom(dp, i + 3);
		cmn_err(CE_CONT,
		    "!0x%02x: %02x %02x %02x %02x %02x %02x %02x %02x",
		    i*2,
		    (uint8_t)x0, (uint8_t)(x0 >> 8),
		    (uint8_t)x1, (uint8_t)(x1 >> 8),
		    (uint8_t)x2, (uint8_t)(x2 >> 8),
		    (uint8_t)x3, (uint8_t)(x3 >> 8));
	}
}
#endif

static void
gani_fixup_params(struct gem_dev *dp)
{
	int	rxmaxsize;
	struct gani_dev	*lp = dp->private;

	/*
	 * Fix rx buffer length:
	 * It must have additional space depend on chip model,
	 * and its size must be multiple of 8.
	 */
	rxmaxsize = GANI_PCI(lp) ? RMS_PCI(dp->mtu) : RMS_PCIE(dp->mtu);
	dp->rx_buf_len = ROUNDUP2(rxmaxsize, 8);

#ifdef GEM_CONFIG_VLAN_HW
	/*
	 * RTL816x series will corrupt vlan-tagged outgoing packets
	 * when hw checksum is enabled and hw vlan tagging is disabled.
	 */
	dp->misc_flag |= GEM_VLAN_HARD;
#ifdef CONFIG_CKSUM_OFFLOAD
	lp->new_cksum = !(GANI_PCI(lp) ||
	    lp->chip->type == R8168_1 /* RTL8168B rev.B */ ||
	    lp->chip->type == R8168_2 /* RTL8168B rev.E */ ||
	    lp->chip->type == R8168_3 /* RTL8168B rev.F */ ||
	    lp->chip->type == R8101_1 /* RTL8101E rev.B */ ||
	    lp->chip->type == R8101_2 /* RTL8101E rev.E */ ||
	    lp->chip->type == R8101_3 /* RTL8191E rev.h */);

	if (GANI_PCIE_GIGA(lp) && dp->mtu > ETHERMTU) {
		/*
		 * checksum offload doesn't work for jumbo packets.
		 * clear previous setting, it's garbage.
		 */
		dp->misc_flag &=
		    ~(GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4 | GEM_LSO);
	} else {
		/* XXX - ok for 8169 and 8101 */
		dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4;
#ifdef CONFIG_LSO
		dp->misc_flag |= GEM_LSO;
#endif
	}
#endif /* CONFIG_CKSUM_OFFLOAD */
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif /* GEM_CONFIG_VLAN_HW */
}

static int
gani_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	uint8_t		*m;
	struct gani_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	m = &dp->dev_addr.ether_addr_octet[0];
	if (GANI_PCIE_GIGA(lp) &&
	    (gani_read_eeprom(dp, 0) == 0x8128 ||
	    gani_read_eeprom(dp, 0) == 0x8129)) {
		/*
		 * read factory mac address from EEPROM
		 */
#if DEBUG_LEVEL > 10
		gani_eeprom_dump(dp, 0x40);
#endif
		for (i = 0; i < ETHERADDRL/2; i++) {
			val = gani_read_eeprom(dp, 7 + i);
			m[i*2 + 0] = (uint8_t)val;
			m[i*2 + 1] = (uint8_t)(val >> 8);
		}
	} else {
		/*
		 * no eeprom
		 * read factory mac address from IDR register
		 */
		for (i = 0; i < ETHERADDRL; i++) {
			m[i] = INB(dp, IDR + i);
		}
	}
	/* invalidate soft copy of mac registers */
	bzero(lp->mac, ETHERADDRL);

	/* no need to scan phy */
	dp->mii_phy_addr = -1;

#ifndef GEM_CONFIG_JUMBO_FRAME
	gani_fixup_params(dp);
#endif
	/* RTL816x series support only store & forword for tx and rx */
	dp->txthr = INT32_MAX;
	dp->rxthr = INT32_MAX;

	/*
	 * XXX - workaround for SiS965L
	 * txmaxdma and rxmaxdma must be unlimited otherwise the system
	 * will hang.
	 */
	dp->txmaxdma = max(dp->mtu, 1024+1);
	if (lp->chip->type == R8168_1 /* RTL8168B rev.B */) {
		dp->txmaxdma = min(dp->txmaxdma,  512);
	}

	dp->rxmaxdma = dp->mtu;

	/* Receive control register: write an initial value */
	val = 0;
	if (dp->rxmaxdma <= 64) {
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
		val |= RCR_MXDMA_UNLIMITED;
	}

	if (GANI_PCIE_FAST(lp)) {
		val |= RCR_RXFTH_SF;	/* reserved */
	} else if (GANI_PCIE_GIGA(lp)) {
		switch (lp->chip->type) {
		case R8168_1: /* RTL8168B rev.B  */
		case R8168_2: /* RTL8168B rev.E */
		case R8168_3: /* RTL8168B rev.F */
			val |= 7 << 13;	/* reserved */
			break;

		case R8168_4: /* RTL8168C */
		case R8168_5: /* RTL8168C rev.B */
		case R8168_6: /* RTL8168C rev.C */
		case R8168_7: /* RTL8168CP rev.B */
		case R8168_8: /* RTL8168CP rev.C */
			val |= RCR_128_INT | RCR_FET_MULTI;
			break;

		case R8168_9: /* RTL8168D */
		case R8168_10: /* RTL8168D rev.B */
		case R8168_11: /* RTL8168D rev.C */
		default:
			val |= RCR_128_INT;
			break;
		}
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
		val |= RCR_RXFTH_SF;	/* store and forward */
	}

	/* keep undocumented bits in RCR  */
	lp->rcr = val | (INL(dp, RCR) & ~RCR_MASK);
	DPRINTF(10, (CE_CONT, "!%s: %s: RCR_MASK:0x%x",
	    dp->name, __func__, ~RCR_MASK));

	/* Transmit configration register : */
	val = 0;
	if (dp->txmaxdma <= 16) {
		val |= TCR_MXDMA_16;
	} else if (dp->txmaxdma <= 32) {
		val |= TCR_MXDMA_32;
	} else if (dp->txmaxdma <= 64) {
		val |= TCR_MXDMA_64;
	} else if (dp->txmaxdma <= 128) {
		val |= TCR_MXDMA_128;
	} else if (dp->txmaxdma <= 256) {
		val |= TCR_MXDMA_256;
	} else if (dp->txmaxdma <= 512) {
		val |= TCR_MXDMA_512;
	} else if (dp->txmaxdma <= 1024) {
		/* vendor choice for 8101 */
		val |= TCR_MXDMA_1024;
	} else {
		val |= TCR_MXDMA_UNLIMITED;
	}
	lp->tcr |= TCR_IFG_802_3 | TCR_LBK_NORMAL | val;

	mutex_init(&lp->stat_lock, NULL, MUTEX_DRIVER, dp->iblock_cookie);
	lp->stat_to_id = (timeout_id_t)0;

	if (GANI_PCIE_GIGA(lp)) {
		/* disable flow control by default for pci-e GbE */
		if (!ddi_prop_exists(DDI_DEV_T_ANY, dp->dip,
		    DDI_PROP_DONTPASS, "adv_pause")) {
			dp->anadv_pause = 0;
		}
		if (!ddi_prop_exists(DDI_DEV_T_ANY, dp->dip,
		    DDI_PROP_DONTPASS, "adv_asmpause")) {
			dp->anadv_asmpause = 0;
		}
	}

	return (GEM_SUCCESS);	/* currently return code is not used. */
}

static int
ganiattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_acc_handle_t	conf_ha;
	int			revid;
	int			unit;
	struct chip_info	*p;
	uint32_t		tcr;
	const char		*drv_name;
	struct gem_dev		*dp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	struct gani_dev		*lp;
	uint32_t		ilr;
	uint_t			pcie_cap;
	uint_t			msi_cap;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);
	DPRINTF(1, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	/*
	 * Common routine after power-on
	 */

	/* fix config registers */
	if (pci_config_setup(dip, &conf_ha) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		goto err;
	}

	pci_config_put16(conf_ha, PCI_CONF_COMM,
	    pci_config_get16(conf_ha, PCI_CONF_COMM)
	    | PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_ha, PCI_PMCSR_D0);

	revid = pci_config_get8(conf_ha, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_ha, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	pcie_cap = gem_search_pci_cap(dip, conf_ha, PCI_CAP_ID_PCI_E);
	msi_cap = gem_search_pci_cap(dip, conf_ha, PCI_CAP_ID_MSI);
	pci_config_teardown(&conf_ha);

	switch (cmd) {
	case DDI_RESUME:
		dp = GEM_GET_DEV(dip);

		if ((i = gem_resume(dip)) == DDI_SUCCESS) {
			gani_stat_timeout(dp);
		}
		return (i);

	case DDI_ATTACH:
		/*
		 * Map in the device registers. Try to map register
		 * in pci memory space first.
		 *  rnumber 1: i/o
		 *  rnumber 2: mem32(pci) or mem64(pcie)
		 */
#ifdef MAP_MEM
		if (ddi_regs_map_setup(dip, 2, &base, 0, 0,
		    &gani_dev_attr, &regs_ha) != DDI_SUCCESS) {
#endif
			if (ddi_regs_map_setup(dip, 1, &base, 0, 0,
			    &gani_dev_attr, &regs_ha) != DDI_SUCCESS) {
				goto err;
			}
#ifdef MAP_MEM
		}
#endif

		/*
		 * Check hardware revision
		 */
		tcr = ddi_get32(regs_ha, (void *)(base + TCR));

		for (p = chiptbl_8169, i = 0; i < CHIPTABLESIZE; p++, i++) {
			if (((p->tcr_val ^ tcr) & p->tcr_mask) == 0) {
				/* found */
				goto chip_found;
			}
		}

		cmn_err(CE_WARN,
		    "!%s%d: attach: unknown mac version: tcr:0x%08x",
		    drv_name, unit, tcr);

		/* choose the oldest 8169 chipset */
		p = &chiptbl_8169[0];

chip_found:
		cmn_err(CE_CONT,
		    "!%s%d: chip is %s rev:0x%02x tcr:0x%08x "
		    "pcie_cap:0x%02x msi_cap:0x%02x",
		    drv_name, unit, p->name, revid, tcr, pcie_cap, msi_cap);

		lp = kmem_zalloc(sizeof (struct gani_dev), KM_SLEEP);
		lp->chip = p;
		lp->pcie_cap = pcie_cap;
		lp->msi = msi_cap != 0;
		lp->isr_pended = 0;
		lp->initialized = B_FALSE;
		lp->tcr = tcr & p->tcr_mask;

		/* read config2 register */
		if (GANI_PCIE(lp)) {
			lp->busclk = BUSCLK_PCIE; /* PCI-E (125MHz) */
		} else {
			lp->busclk = ddi_get8(regs_ha,
			    (uint8_t *)(base + CFG2)) & CFG2_PCICLKF
			    ? BUSCLK_PCI66 : BUSCLK_PCI33;
		}
		DPRINTF(0, (CE_CONT, "!%s%d: busclk:%d, cfg2:0x%x",
		    drv_name, unit, lp->busclk,
		    ddi_get8(regs_ha, (uint8_t *)(base + CFG2))));

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		(void) sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = TX_MAX_FRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = 4;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = gani_tx_copy_thresh;

		gcp->gc_rx_buf_align = (GANI_PCIE(lp) ? 8 : 1) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = gani_rx_copy_thresh;
		gcp->gc_io_area_size = TC_SIZE;

		/* map attributes */
		gcp->gc_dev_attr = gani_dev_attr;
		gcp->gc_buf_attr = gani_buf_attr;
		gcp->gc_desc_attr = gani_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = gani_dma_attr_desc;
		if (gani_64bit_addr) {
			gcp->gc_dma_attr_txbuf = gani_dma_attr_buf64;
		} else {
			gcp->gc_dma_attr_txbuf = gani_dma_attr_buf32;
		}
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		if (gani_64bit_addr) {
			gcp->gc_dma_attr_rxbuf = gani_dma_attr_buf64;
		} else {
			gcp->gc_dma_attr_rxbuf = gani_dma_attr_buf32;
		}
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control capability */
		/* XXX - rx hangs for 8168 when flow control is enabled */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/10;
		gcp->gc_mii_reset_timeout = 25*(ONESEC/10);
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = ONESEC;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* rtl8169 seems to delay to recognize PHY status */
		gcp->gc_mii_an_delay = ONESEC/10;	/* 100mS */
		/* we must reset rtl8169 phy to update advert register */
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;

		/*
		 * XXX - MAC_VER_8169S or later chipset requires reset
		 * on link down timeout for 1Gbps connection
		 */
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_an_oneshot = B_FALSE;
		gcp->gc_mii_hw_link_detection = B_TRUE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &gani_attach_chip;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &gani_fixup_params;
#endif
		gcp->gc_reset_chip = &gani_reset_chip;
		gcp->gc_init_chip = &gani_init_chip;
		gcp->gc_start_chip = &gani_start_chip;
		gcp->gc_stop_chip = &gani_stop_chip;
		gcp->gc_multicast_hash = &gani_mcast_hash;
		gcp->gc_set_rx_filter = &gani_set_rx_filter;
		gcp->gc_set_media = &gani_set_media;
		gcp->gc_get_stats = &gani_get_stats;
		gcp->gc_interrupt = &gani_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &gani_tx_desc_write;
		gcp->gc_rx_desc_write = &gani_rx_desc_write;
		gcp->gc_tx_start = &gani_tx_start;
		gcp->gc_rx_start = NULL;
		gcp->gc_tx_desc_stat = &gani_tx_desc_stat;
		gcp->gc_rx_desc_stat = &gani_rx_desc_stat;
		gcp->gc_tx_desc_init = &gani_tx_desc_init;
		gcp->gc_rx_desc_init = &gani_rx_desc_init;
		gcp->gc_tx_desc_clean = &gani_tx_desc_init;
		gcp->gc_rx_desc_clean = &gani_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &gani_mii_probe;
		gcp->gc_mii_init = &gani_mii_init;
		gcp->gc_mii_config = &gani_mii_config;
		gcp->gc_mii_sync = &gani_mii_sync;
		gcp->gc_mii_read = &gani_mii_read;
		gcp->gc_mii_write = &gani_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* MSI/MSIX interrupts */
#ifdef CONFIG_INTR_MSI
		gcp->gc_nintrs_req = 1;
#else
		gcp->gc_nintrs_req = 0;
#endif

		/* offload and jumbo frame */
		gcp->gc_max_lso = 16 * 1024 - 1;
		gcp->gc_max_mtu = p->max_mtu;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		dp = gem_do_attach(dip, 0, gcp, base, &regs_ha,
		    lp, sizeof (struct gani_dev));
		kmem_free(gcp, sizeof (*gcp));

		if (dp == NULL) {
			goto err_free_mem;
		}

		gani_stat_timeout(dp);

		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof (struct gani_dev));
err:
		return (DDI_FAILURE);
	}

	return (DDI_FAILURE);
}

static int
ganidetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct gem_dev  *dp;
	struct gani_dev	*lp;

	dp = GEM_GET_DEV(dip);
	lp = dp->private;

	switch (cmd) {
	case DDI_DETACH:
	case DDI_SUSPEND:
		/* Ensure statistics timer routine stopped */
		mutex_enter(&lp->stat_lock);
		if (lp->stat_to_id != 0) {
			timeout_id_t	old_id;

			do {
				old_id = lp->stat_to_id;
				mutex_exit(&lp->stat_lock);
				(void) untimeout(old_id);
				mutex_enter(&lp->stat_lock);
			} while (old_id != lp->stat_to_id);
			lp->stat_to_id = 0;
		}
		mutex_exit(&lp->stat_lock);

		switch (cmd) {
		case DDI_DETACH:
			mutex_destroy(&lp->stat_lock);
			return (gem_do_detach(dip));

		case DDI_SUSPEND:
			return (gem_suspend(dip));
		}
	}

	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef GEM_CONFIG_GLDv3
GEM_STREAM_OPS(gani_ops, ganiattach, ganidetach);
#else
static	struct module_info ganiminfo = {
	0,			/* mi_idnum */
	"gani",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit ganirinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ganiminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit ganiwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ganiminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	gani_info = {
	&ganirinit,	/* st_rdinit */
	&ganiwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_gani_ops = {
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
	&gani_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops gani_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	ganiattach,	/* devo_attach */
	ganidetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_gani_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce,	/* devo_quiesce */
#endif
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&gani_ops,	/* driver ops */
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
	int	status;

	DPRINTF(2, (CE_CONT, "!gani: _init: called"));

	status = gem_mod_init(&gani_ops, "gani");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&gani_ops);
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

	DPRINTF(2, (CE_CONT, "!gani: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&gani_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
