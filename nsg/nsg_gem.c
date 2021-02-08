/*
 *  nsg_gem.c : DP83820 Gigabit Ethernet MAC driver for Solaris
 *
 * Copyright (c) 2005-2012 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"%W% %E%"

/*
 * Change log
 */

/*
 * TODO:
 *
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

#include <sys/byteorder.h>

#include <sys/pci.h>
#include "gem_mii.h"
#include "gem.h"
#include "dp83820reg.h"

char	ident[] = "NS DP83820 GbE driver " VERSION;

#ifndef GEM_CONFIG_GLDv3
char	_depends_on[] = {"misc/gld"};
#endif

/* Debugging support */
#ifdef DEBUG_LEVEL
static int nsg_debug = DEBUG_LEVEL;

#if DEBUG_LEVEL > 4
#define	CONS	"^"
#else
#define	CONS	"!"
#endif

#define	DPRINTF(n, args)	if (nsg_debug > (n)) cmn_err args

#else

#define	CONS	"!"
#define	DPRINTF(n, args)

#endif	/* DEBUG_LEVEL */

/*
 * Useful macros and typedefs
 */
#define	ONESEC			(drv_usectohz(1*1000000))

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif /* MAP_MEM */

/*
 * Our configuration
 */
#ifdef CONFIG_OO
#define	MAXTXFRAGS	1
#else
#define	MAXTXFRAGS	min(GEM_MAXTXFRAGS, 8)
#endif
#define	MAXRXFRAGS	1

#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	256
#endif
#ifndef	TX_RING_SIZE
#if MAXTXFRAGS == 1
#define	TX_RING_SIZE	TX_BUF_SIZE
#else
#define	TX_RING_SIZE	(TX_BUF_SIZE * 4)
#endif
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif
#ifndef	RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	OUR_INTR_BITS	\
	(ISR_SWI |	\
	ISR_DPERR | ISR_SSERR | ISR_RMABT | ISR_RTABT | ISR_RXSOVR |	\
	ISR_TXURN | ISR_TXDESC | ISR_TXERR |	\
	ISR_RXORN | ISR_RXIDLE | ISR_RXOK | ISR_RXERR | \
	ISR_PHY)

#define	USE_MULTICAST_HASHTBL

static int	nsg_tx_copy_thresh = 256;
static int	nsg_rx_copy_thresh = 256;
static int	nsg_use_dac = B_TRUE;

/* IEEE802.3x mac control frame for flow controi */
struct pause_frame {
	struct ether_addr	dest;
	struct ether_addr	src;
	uint16_t	type_len;
	uint16_t	pause_flag;
	uint16_t	pause_time;
};

#define	NSG_DESC_SIZE32_SHIFT	4
#define	NSG_DESC_SIZE64_SHIFT	5

#define	NSG_DESC_SIZE32	(1 << NSG_DESC_SIZE32_SHIFT)
#define	NSG_DESC_SIZE64	(1 << NSG_DESC_SIZE64_SHIFT)

#define	NSG_DESC_ADDR(dp, b, n)	\
	((b) + ((n) << (dp)->gc.gc_tx_desc_unit_shift))
#define	NSG_DESC_PTR(dp, b, n)	\
	((void *)(NSG_DESC_ADDR(dp, (uintptr_t)(b), n)))

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*chip_name;
	int		chip_type;
#define	CHIPTYPE_DP83820	0
};

/*
 * Chip dependant MAC state
 */
struct nsg_dev {
	/* misc HW information */
	struct chip_info	*chip;
	uint32_t		our_intr_bits;
	uint32_t		cr;
	int			tx_drain_threshold;
	int			tx_fill_threshold;
	int			rx_drain_threshold;
	int			rx_fill_threshold;
	uint8_t			revid;	/* revision from PCI configuration */
	int			last_poll_interval;
	boolean_t		pci_addr_64;
#ifdef USE_MULTICAST_HASHTBL
#define	MCAST_HASH_SIZE	(MCAST_HASH_BITS / 16)
	uint16_t		hash_tbl[MCAST_HASH_SIZE];
#endif
};

/*
 * Hardware information
 */
struct chip_info nsg_chiptbl[] = {
{
	0x100b,	0x0022,	"DP83820", CHIPTYPE_DP83820,
},
};
#define	CHIPTABLESIZE	(sizeof (nsg_chiptbl) / sizeof (struct chip_info))

/* ======================================================== */

/* mii operations */
static uint16_t  nsg_mii_read(struct gem_dev *, uint_t);
static void nsg_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int nsg_reset_chip(struct gem_dev *);
static int nsg_init_chip(struct gem_dev *);
static int nsg_start_chip(struct gem_dev *);
static int nsg_stop_chip(struct gem_dev *);
static int nsg_set_media(struct gem_dev *);
static int nsg_set_rx_filter(struct gem_dev *);
static int nsg_get_stats(struct gem_dev *);
static int nsg_attach_chip(struct gem_dev *);

/* descriptor operations */
static int nsg_tx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags);
static int nsg_tx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags);
static void nsg_tx_start64(struct gem_dev *dp, int startslot, int nslot);
static void nsg_tx_start32(struct gem_dev *dp, int startslot, int nslot);
static void nsg_rx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void nsg_rx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t nsg_tx_desc_stat64(struct gem_dev *dp, int slot, int ndesc);
static uint_t nsg_tx_desc_stat32(struct gem_dev *dp, int slot, int ndesc);
static uint64_t nsg_rx_desc_stat64(struct gem_dev *dp, int slot, int ndesc);
static uint64_t nsg_rx_desc_stat32(struct gem_dev *dp, int slot, int ndesc);

static void nsg_tx_desc_init64(struct gem_dev *dp, int slot);
static void nsg_tx_desc_init32(struct gem_dev *dp, int slot);
static void nsg_rx_desc_init64(struct gem_dev *dp, int slot);
static void nsg_rx_desc_init32(struct gem_dev *dp, int slot);
static void nsg_tx_desc_clean64(struct gem_dev *dp, int slot);
static void nsg_tx_desc_clean32(struct gem_dev *dp, int slot);
static void nsg_rx_desc_clean64(struct gem_dev *dp, int slot);
static void nsg_rx_desc_clean32(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t nsg_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr nsg_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr nsg_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t nsg_dma_attr_buf64 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	~0ull,			/* dma_attr_addr_hi */
	0x0000ffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000ffffull,		/* dma_attr_maxxfer */
	~0ull,			/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t nsg_dma_attr_desc64 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	16,			/* dma_attr_addr_lo */
	~0ull,			/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	~0ull,			/* dma_attr_maxxfer */
	~0ull,			/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t nsg_dma_attr_buf32 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x0000ffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000ffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t nsg_dma_attr_desc32 = {
	DMA_ATTR_V0,		/* dma_attr_version */
	16,			/* dma_attr_addr_lo */
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

#define	EEPROM_DELAY(dp)	{ INL(dp, MEAR); INL(dp, MEAR); }
#define	EE_CMD_READ	6
#define	EE_CMD_SHIFT	6

#define	EEPROM_CLOCK(dp, ctrl)	{ \
	OUTL(dp, MEAR, (ctrl));	\
	EEPROM_DELAY(dp);	\
	OUTL(dp, MEAR, (ctrl) | MEAR_EECLK);	\
	EEPROM_DELAY(dp);	\
}

static uint16_t
nsg_read_eeprom(struct gem_dev *dp, uint_t offset)
{
	int	i;
	uint_t	eedi;
	uint_t	ret;

	/* ensure de-assert chip select */
	EEPROM_CLOCK(dp, 0);

	/* assert chip select */
	offset |= EE_CMD_READ << EE_CMD_SHIFT;

	for (i = 8; i >= 0; i--) {
		/* make command */
		eedi = ((offset >> i) & 1) << MEAR_EEDI_SHIFT;

		/* send 1 bit */
		EEPROM_CLOCK(dp, MEAR_EESEL | eedi);
	}

	OUTL(dp, MEAR, MEAR_EESEL);

	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		EEPROM_CLOCK(dp, MEAR_EESEL);
		ret = (ret << 1) | ((INL(dp, MEAR) >> MEAR_EEDO_SHIFT) & 1);
	}

	OUTL(dp, MEAR, 0);
	EEPROM_DELAY(dp);

	return (ret);
}
#undef EEPROM_DELAY

static boolean_t
nsg_get_mac_addr(struct gem_dev *dp)
{
	uint8_t		*mac;
	uint_t		val;
	int		i;

	DPRINTF(4, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	mac = dp->dev_addr.ether_addr_octet;

	for (i = 0; i < ETHERADDRL; i += 2) {
		val = nsg_read_eeprom(dp, EEPROM_PMATCH0 - i/2);
		mac[i + 0] = (uint8_t)val;
		mac[i + 1] = (uint8_t)(val >> 8);
	}

	return (B_TRUE);
}

static int
nsg_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct nsg_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: %s called", dp->name, __func__));

	lp->cr = 0;
	OUTL(dp, IER, 0);
	OUTL(dp, IMR, 0);
	OUTL(dp, RFCR, 0);

	OUTL(dp, CR, CR_RST);
	drv_usecwait(10);

	for (i = 0; (INL(dp, CR) & CR_RST) != 0; i++) {
		if (i > 100) {
			cmn_err(CE_WARN,
			    "!%s: timeout: chip reset", dp->name);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}
#ifdef notdef
	OUTL(dp, CCSR, CCSR_PMESTS);
	OUTL(dp, CCSR, 0);
#endif
	/* Configration register: */
	val = INL(dp, CFG);
	DPRINTF(2, (CE_CONT, CONS "%s: cfg:%b",
	    dp->name, val, CFG_BITS));
#if 0
	/* don't care, or keep the current status */
	CFG_LNKSTS	/* current link status */
	CFG_SPDSTS	/* current speed */
	CFG_DUPSTS	/* current duplex */
	CFG_T64ADDR	/* Enable target 64bit address (ro) */
	CFG_PCI64_DET	/* PCI 64bit bus detected (ro) */
	CFG_TBI_EN	/* Enable Ten-Bit I/F */
#endif
	/* clear bits */
	val &= ~CFG_MODE_1000;	/* Enable 1000Mbps mode */
	val &= ~CFG_MRM_DIS;	/* disable memry read multiple */
	val &= ~CFG_MWI_DIS;	/* disable memry write invalidate */
	val &= ~CFG_PHY_RST;	/* external PHY reset */
	val &= ~CFG_PHY_DIS;	/* external PHY disable */
	val &= ~CFG_REQALG;	/* PCI Bus request algorithm */
	val &= ~CFG_SB;		/* Single backoff */
	val &= ~CFG_POW;	/* Program out of window timer */
	val &= ~CFG_EXD;	/* Excessive deferral timer disable */
	val &= ~CFG_PESEL;	/* Parity error detection action */
	val &= ~CFG_BROM_DIS;	/* BootRom disable */
	val &= ~CFG_EXT_125;	/* select external 125MHz reference */
	val &= ~CFG_BEM;	/* Big endian mode */

	/* set bits */
	val |= CFG_PINT_CTL;	/* PHY interrupt control */
	val |= CFG_TMRTEST;	/* Speeds up 100uS timer to 4uS */
	val |= CFG_EXTSTS_EN;	/* Enable extended status */

	/* depend on configuration */
	if (lp->pci_addr_64) {
		/* enable pci 64bit addressing for master */
		val |= CFG_M64ADDR;
	} else {
		val &= ~CFG_M64ADDR;
	}

	if (val & CFG_PCI64_DET) {
		/* enable pci 64bit data transfer for master */
		val |= CFG_DATA64_EN;
	} else {
		val &= ~CFG_DATA64_EN;
	}

	OUTL(dp, CFG, val);

	DPRINTF(0, (CE_CONT, CONS "%s: cfg:%b",
	    dp->name, INL(dp, CFG), CFG_BITS));
	return (GEM_SUCCESS);
}

static int
nsg_init_chip(struct gem_dev *dp)
{
	uint_t		val;
	struct nsg_dev	*lp = dp->private;

	/* PCI test control register: do nothing */

	/* Interrupt status register : clear pended interrupts */
	(void) INL(dp, ISR);

	/* Interrupt enable register: enable interrupt */
	OUTL(dp, IER, 0);

	/* Interrupt mask register: clear */
	lp->our_intr_bits = OUR_INTR_BITS;

	OUTL(dp, IMR, lp->our_intr_bits);

	/* Interrupt hold timer: reset */
	OUTL(dp, IHR, 0);
	OUTL(dp, IHR, IH_IHCTL | 1);
	OUTL(dp, IHR, 0);

	/* PQCR: disable priority queueing */
	OUTL(dp, PQCR, 0);

	/* WCSR: disable wake-on-lan */
	OUTL(dp, WCSR, 0);

	/* Transmit Descriptor Pointer register: base addr of TX ring */
	OUTL(dp, TXDP, (uint32_t)dp->tx_ring_dma);
	OUTL(dp, TXDP_HI, (uint32_t)(dp->tx_ring_dma >> 32));

	/* Receive descriptor pointer register: base addr of RX ring */
	OUTL(dp, RXDP, (uint32_t)dp->rx_ring_dma);
	OUTL(dp, RXDP_HI, (uint32_t)((dp->rx_ring_dma) >> 32));

#ifdef GEM_CONFIG_GLDv3
#ifdef CONFIG_CKSUM_OFFLOAD
	/*
	 * VTCT_GCK caused udp cksum errors. Use VTCR_PPCHK, instead of.
	 */
	OUTL(dp, VTCR, VTCR_PPCHK | VTCR_VPPTI);
	OUTL(dp, VRCR, VRCR_IPEN | VRCR_VTDEN);
#endif
#endif

	return (GEM_SUCCESS);
}

static uint_t
nsg_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL));
}

static int
nsg_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint32_t	mode;
	int		addr_shift;
	uint8_t		*mac;
	struct nsg_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	addr_shift = RFCR_RFADDR_SHIFT;
	mac = dp->cur_addr.ether_addr_octet;

	/* Set Receive filter control register */

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = 0;
	} else if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		mode = RFCR_AAB		/* all broadcast */
		    | RFCR_AAM		/* all multicast */
		    | RFCR_AAU;	/* all physcal */
	} else if ((dp->rxmode & RXMODE_ALLMULTI) != 0) {
		mode = RFCR_AAB		/* all broadcast */
		    | RFCR_AAM		/* all multicast */
		    | RFCR_APM;	/* physical for the chip */
	} else {
		mode = RFCR_AAB		/* all broadcast */
		    | RFCR_APM;	/* physical for the chip */

		if (dp->mc_count == 0) {
			/* no multicast, do nothing */
		} else if (dp->mc_count <= MCAST_HASH_BITS / 2) {
			uint_t	h;

#ifdef USE_MULTICAST_HASHTBL
			/* use multicast hash table */
			mode |= RFCR_MHEN;

			/* make hash table */
			bzero(lp->hash_tbl, sizeof (lp->hash_tbl));
#endif
			for (i = 0; i < dp->mc_count; i++) {
#ifdef USE_MULTICAST_PERFECT
				if (i < 4) {
					mode |= 1U << (RFCR_APAT_SHIFT + i);
					continue;
				}
#endif
#ifdef USE_MULTICAST_HASHTBL
				h = dp->mc_list[i].hash >> (32 - 11);
				lp->hash_tbl[h / 16] |= 1U << (h % 16);
#else
				mode |= RFCR_AAM;
				break;
#endif
			}
		} else {
			mode |= RFCR_AAM;
		}
	}

	/* Disable Rx filter and load mac address for the chip */
	for (i = 0; i < ETHERADDRL; i += 2) {
		OUTL(dp, RFCR, RFADDR_MAC+ i);
		OUTL(dp, RFDR, mac[i + 1] << 8 | mac[i]);
	}

#ifdef USE_MULTICAST_PERFECT
	if ((mode & RFCR_APAT) && (mode & RFCR_AAM) == 0) {
		int	j;
		int	n;
		static uint_t	rf_perfect_base[] = {
			RFADDR_PMATCH0, RFADDR_PMATCH1,
			RFADDR_PMATCH2, RFADDR_PMATCH3,
		}
		;

		/* setup perfect match patterns */
		n = min(dp->mc_count, 4);
		for (i = 0; i < n; i++) {
			mac = &dp->mc_list[i].addr.ether_addr_octet[0];
			for (j = 0; j < ETHERADDRL; j += 2) {
				OUTL(dp, RFCR, rf_perfect_base[i] + j);
				OUTL(dp, RFDR, mac[j + 1] << 8 | mac[j]);
			}
		}

		/* setup pattern count register */
		OUTL(dp, RFCR, RFADDR_PCOUNT01);
		OUTL(dp, RFDR, ETHERADDRL << 8 | ETHERADDRL);
		OUTL(dp, RFCR, RFADDR_PCOUNT23);
		OUTL(dp, RFDR, ETHERADDRL << 8 | ETHERADDRL);
	}
#endif
#ifdef USE_MULTICAST_HASHTBL
	if ((mode & RFCR_MHEN) && (mode & RFCR_AAM) == 0) {
		/* Load Multicast hash table */
		for (i = 0; i < MCAST_HASH_SIZE; i++) {
			/* For DP83820, index is in byte */
			OUTL(dp, RFCR, RFADDR_MULTICAST + i * 2);
			OUTL(dp, RFDR, lp->hash_tbl[i]);
		}
	}
#endif
	/* Set rx filter mode and enable rx filter */
	OUTL(dp, RFCR, RFCR_RFEN | mode);

	return (GEM_SUCCESS);
}

static int
nsg_start_chip(struct gem_dev *dp)
{
	struct nsg_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	/* enable interrupt */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IER, 1);
	}

	/* Kick RX */
	OUTL(dp, CR, lp->cr | CR_RXE);

	return (GEM_SUCCESS);
}

static int
nsg_stop_chip(struct gem_dev *dp)
{
	struct nsg_dev	*lp = dp->private;
	uint32_t	done;
	int		i;

	DPRINTF(4, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));
	/*
	 * Stop chip core
	 */
	/* Inhibit interrupt */
	OUTL(dp, IER, 0);
	FLSHL(dp, IER);

	/* stop TX and RX immediately */
	OUTL(dp, CR, lp->cr | CR_TXR | CR_RXR);

	done = 0;
	for (i = 0; done != (ISR_RXRCMP | ISR_TXRCMP); i++) {
		if (i > 1000) {
			cmn_err(CE_NOTE, "!%s: %s: Tx/Rx stop timeout",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}
		done |= INL(dp, ISR) & (ISR_RXRCMP | ISR_TXRCMP);
		drv_usecwait(10);
	}

	return (GEM_SUCCESS);
}

static int
nsg_set_media(struct gem_dev *dp)
{
	int		pktsize;
	uint32_t	txcfg;
	uint32_t	rxcfg;
	uint32_t	pcr;
	uint32_t	cfg;
	uint32_t	val;
	struct nsg_dev	*lp = dp->private;
	extern int	gem_speed_value[];

	DPRINTF(2, (CE_CONT, CONS "%s: %s: %s duplex, %d Mbps",
	    dp->name, __func__,
	    dp->full_duplex ? "full" : "half", gem_speed_value[dp->speed]));

	/* select speed and duplex */
	cfg = INL(dp, CFG);
	val = cfg & ~(CFG_MODE_1000 | CFG_SB);
	if (dp->speed == GEM_SPD_1000) {
		val |= CFG_MODE_1000;
	}
	if (dp->full_duplex) {
		val |= CFG_SB;
	}
	if (cfg != val) {
		OUTL(dp, CFG, val);
	}

	pktsize = dp->mtu + sizeof (struct ether_header) + ETHERFCSL;
#ifdef GEM_CONFIG_GLDv3
	pktsize += 4;
#endif
	/* tx low water mark */
	lp->tx_fill_threshold = max(3 * pktsize / 4, dp->txmaxdma);

	/* transmit starting threshold: S&F */
	lp->tx_drain_threshold = 0;

	/* receive starting threshold */
	lp->rx_drain_threshold = min(dp->rxthr, 31 * RXCFG_DRTH_UNIT);
#ifdef notdef
	ASSERT(lp->tx_drain_threshold < 64*TXCFG_DRTH_UNIT);
	ASSERT(lp->tx_fill_threshold < 64*TXCFG_FLTH_UNIT);
	ASSERT(lp->rx_drain_threshold < 32*RXCFG_DRTH_UNIT);
#endif
	txcfg =	TXCFG_ATP		/* auto padding */
	    | TXCFG_MXDMA_256
	    | (dp->full_duplex ? (TXCFG_CSI | TXCFG_HBI) : 0)
	    | ((lp->tx_fill_threshold/TXCFG_FLTH_UNIT) << TXCFG_FLTH_SHIFT)
	    | (lp->tx_drain_threshold/TXCFG_DRTH_UNIT);

	if (INL(dp, TXCFG) != txcfg) {
		OUTL(dp, TXCFG, txcfg);
	}

	rxcfg =	RXCFG_AEP | RXCFG_ARP
	    | (dp->full_duplex ? RXCFG_RX_FD : 0)
	    | RXCFG_MXDMA_1024
	    | ((lp->rx_drain_threshold/RXCFG_DRTH_UNIT) << RXCFG_DRTH_SHIFT);

	if (INL(dp, RXCFG) != rxcfg) {
		OUTL(dp, RXCFG, rxcfg);
	}

	DPRINTF(0, (CE_CONT, CONS "%s: %s: txcfg:%b rxcfg:%b",
	    dp->name, __func__,
	    txcfg, TXCFG_BITS, rxcfg, RXCFG_BITS));

	/* Flow control */
#define	PCR_TX_PAUSE	\
	(PCR_PS_STHI_8 | PCR_PS_STLO_4 | PCR_PS_FFHI_8 | \
	PCR_PS_FFLO_4 | 0xffffU)

#define	PCR_TX_PAUSE_NG	\
	(PCR_PS_STHI_DIS | PCR_PS_STLO_8 | PCR_PS_FFHI_DIS | \
	PCR_PS_FFLO_8 | 0xfffeU)

#define	PCR_RX_PAUSE	(PCR_PSEN | PCR_PS_DA | PCR_PS_MCAST)

	switch (dp->flow_control) {
	case FLOW_CONTROL_NONE:
		pcr = 0;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		pcr = PCR_TX_PAUSE;
		break;

	case FLOW_CONTROL_SYMMETRIC:
		pcr = PCR_TX_PAUSE | PCR_RX_PAUSE;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		pcr = PCR_RX_PAUSE;
		break;
	}
	if (INL(dp, PCR) != pcr) {
		OUTL(dp, PCR, pcr);
	}
#undef	PCR_TX_PAUSE
#undef	PCR_TX_PAUSE_NG
#undef	PCR_RX_PAUSE

	DPRINTF(2, (CE_CONT, CONS "%s: %s: PCR: %b",
	    dp->name, __func__, INL(dp, PCR), PCR_BITS));
	return (GEM_SUCCESS);
}

static int
nsg_get_stats(struct gem_dev *dp)
{
	/* do nothing */
	return (GEM_SUCCESS);
}

/*
 * discriptor  manipulation
 */
static int
nsg_tx_desc_write64(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			n;
	uint32_t		mark;
	uint64_t		tmp;
	uint32_t		tmp_lo;
	uint32_t		tmp_hi;
	uint32_t		extsts = 0;
	struct nsg_desc64	*tdp;
	struct nsg_dev		*lp = dp->private;
	const uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    CONS "%s: time:%d %s seqnum:%d, slot:%d, frags:%d flags:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_desc_tail, slot, frags, flags);

	for (n = 0; n < frags; n++) {
		cmn_err(CE_CONT, CONS "%d: addr: 0x%x, len: 0x%x",
		    n, dmacookie[n].dmac_address, dmacookie[n].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
	mark = CMDSTS_MORE;
	if ((flags & GEM_TXFLAG_HEAD) == 0) {
		mark |= CMDSTS_OWN;
	}
#ifdef CONFIG_CKSUM_OFFLOAD
	if (flags & GEM_TXFLAG_IPv4) {
		extsts |= LE_32(EXTSTS_IPPKT);
		if (flags & GEM_TXFLAG_TCP) {
			extsts |= LE_32(EXTSTS_TCPPKT);
		} else if (flags & GEM_TXFLAG_UDP) {
			extsts |= LE_32(EXTSTS_UDPPKT);
		}
	}

	if (flags & GEM_TXFLAG_VTAG) {
		uint32_t	vtag;
		vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
		vtag = EXTSTS_VPKT | BSWAP_16(vtag);
		extsts |= LE_32(vtag);
	}
#endif
	for (n = 0; n < frags; n++) {
		/* make a slot for n-th fragment */
		tdp = NSG_DESC_PTR(dp, dp->tx_ring,
		    SLOT(slot + n, tx_ring_size));

		tmp = (uint64_t)dmacookie[n].dmac_address;
		tmp_lo = tmp;
		tmp_hi = tmp >> 32;
		tdp->d_bufptr_lo = LE_32(tmp_lo);
		tdp->d_bufptr_hi = LE_32(tmp_hi);

		tdp->d_extsts = extsts;

		tmp_lo = mark | (uint32_t)dmacookie[n].dmac_size;
		tdp->d_cmdsts = LE_32(tmp_lo);

		mark |= CMDSTS_OWN;
	}

	/* fix flags of the last descriptor */
	tmp_lo = tdp->d_cmdsts & ~LE_32(CMDSTS_MORE);
	if (flags & GEM_TXFLAG_INTR) {
		tmp_lo |= LE_32(CMDSTS_INTR);
	}
	tdp->d_cmdsts = tmp_lo;

	return (frags);
}

static int
nsg_tx_desc_write32(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int			n;
	uint32_t		mark;
	uint32_t		tmp;
	uint32_t		extsts = 0;
	struct nsg_desc32	*tdp;
	struct nsg_dev		*lp = dp->private;
	const uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    CONS "%s: time:%d %s seqnum:%d, slot:%d, frags:%d flags:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_desc_tail, slot, frags, flags);

	for (n = 0; n < frags; n++) {
		cmn_err(CE_CONT, CONS "%d: addr: 0x%x, len: 0x%x",
		    n, dmacookie[n].dmac_address, dmacookie[n].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
	mark = CMDSTS_MORE;
	if ((flags & GEM_TXFLAG_HEAD) == 0) {
		mark |= CMDSTS_OWN;
	}
#ifdef CONFIG_CKSUM_OFFLOAD
	if (flags & GEM_TXFLAG_IPv4) {
		extsts |= LE_32(EXTSTS_IPPKT);
		if (flags & GEM_TXFLAG_TCP) {
			extsts |= LE_32(EXTSTS_TCPPKT);
		} else if (flags & GEM_TXFLAG_UDP) {
			extsts |= LE_32(EXTSTS_UDPPKT);
		}
	}
#endif
	for (n = 0; n < frags; n++) {
		/* make a slot for n-th fragment */
		tdp = NSG_DESC_PTR(dp, dp->tx_ring,
		    SLOT(slot + n, tx_ring_size));

		tmp = (uint32_t)dmacookie[n].dmac_address;
		tdp->d_bufptr = LE_32(tmp);

		tdp->d_extsts = extsts;

		tmp = mark | (uint32_t)dmacookie[n].dmac_size;
		tdp->d_cmdsts = LE_32(tmp);

		mark |= CMDSTS_OWN;
		extsts = 0;
	}

	/* fix flags of the last descriptor */
	tmp = tdp->d_cmdsts & ~LE_32(CMDSTS_MORE);
	if (flags & GEM_TXFLAG_INTR) {
		tmp |= LE_32(CMDSTS_INTR);
	}
	tdp->d_cmdsts = tmp;

	return (frags);
}

static void
nsg_tx_start64(struct gem_dev *dp, int start_slot, int nslot)
{
	struct nsg_desc64	*tdp;
	struct nsg_dev		*lp = dp->private;

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, start_slot);
	tdp->d_cmdsts |= LE_32(CMDSTS_OWN);

	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/*
	 * Let the Transmit Buffer Manager Fill state machine active.
	 */
	OUTL(dp, CR, lp->cr | CR_TXE);
}

static void
nsg_tx_start32(struct gem_dev *dp, int start_slot, int nslot)
{
	struct nsg_desc32	*tdp;
	struct nsg_dev		*lp = dp->private;

	if (nslot > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, dp->gc.gc_tx_ring_size),
		    nslot - 1, DDI_DMA_SYNC_FORDEV);
	}

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, start_slot);
	tdp->d_cmdsts |= LE_32(CMDSTS_OWN);

	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	/*
	 * Let the Transmit Buffer Manager Fill state machine active.
	 */
	OUTL(dp, CR, lp->cr | CR_TXE);
}

static void
nsg_rx_desc_write64(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct nsg_desc64	*rdp;
	uint64_t		tmp;
	uint32_t		tmp_lo;
	uint32_t		tmp_hi;

	ASSERT(frags == 1);

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT, CONS
	    "%s: %s: seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
}
#endif
	/* for the last slot of the packet */
	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);

	tmp = dmacookie->dmac_address;
	tmp_lo = tmp;
	tmp_hi = tmp >> 32;
	rdp->d_bufptr_lo = LE_32(tmp_lo);
	rdp->d_bufptr_hi = LE_32(tmp_hi);

	tmp_lo = CMDSTS_INTR | (uint32_t)dmacookie->dmac_size;
	rdp->d_cmdsts = LE_32(tmp_lo);
}

static void
nsg_rx_desc_write32(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct nsg_desc32	*rdp;
	uint32_t		tmp;

	ASSERT(frags == 1);

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT, CONS
	    "%s: %s: seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
}
#endif
	/* for the last slot of the packet */
	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);

	tmp = (uint32_t)dmacookie->dmac_address;
	rdp->d_bufptr = LE_32(tmp);

	tmp = CMDSTS_INTR | (uint32_t)dmacookie->dmac_size;
	rdp->d_cmdsts = LE_32(tmp);
}

static uint_t
nsg_tx_desc_stat_common(struct gem_dev *dp, uint32_t status)
{
	int	cols;

	if (status & CMDSTS_OWN) {
		/* not transmitted yet */
		return (0);
	}
	ASSERT((status & CMDSTS_MORE) == 0);

	/*
	 * collect statictics
	 */
	if ((status & CMDSTS_OK) == 0) {

		/* failed to transmit the packet */

		DPRINTF(2, (CE_CONT, CONS "%s: Transmit error, Tx status %b",
		    dp->name, status, TXSTAT_BITS));

		dp->stats.errxmt++;

		if (status & CMDSTS_TFU) {
			dp->stats.underflow++;
		}
		if (status & CMDSTS_CRS) {
			dp->stats.nocarrier++;
		}
		if (status & CMDSTS_OWC) {
			dp->stats.xmtlatecoll++;
		}
		if (!dp->full_duplex) {
			if (status & CMDSTS_EC) {
				dp->stats.excoll++;
				dp->stats.collisions += 16;
			}
		}
	} else if (!dp->full_duplex) {

		cols = (status >> CMDSTS_CCNT_SHIFT) & CCNT_MASK;

		if (cols > 0) {
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* (cols > 1) */ {
				dp->stats.multi_coll++;
			}
			dp->stats.collisions += cols;
		} else if ((status & CMDSTS_TD) != 0) {
			dp->stats.defer++;
		}
	}

	return (GEM_TX_DONE);
}

static uint_t
nsg_tx_desc_stat64(struct gem_dev *dp, int slot, int ndesc)
{
	struct nsg_desc64	*tdp;
	uint32_t		status;

	/* check status of the last descriptor */
	tdp = NSG_DESC_PTR(dp,
	    dp->tx_ring, SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size));

	status = tdp->d_cmdsts;
	status = LE_32(status);

	DPRINTF(2, (CE_CONT,
	    CONS "%s: time:%d %s: slot:%d, status:0x%b",
	    dp->name, __func__,
	    ddi_get_lbolt(), slot, status, TXSTAT_BITS));

	return (nsg_tx_desc_stat_common(dp, status));
}

static uint_t
nsg_tx_desc_stat32(struct gem_dev *dp, int slot, int ndesc)
{
	struct nsg_desc32	*tdp;
	uint32_t		status;

	/* check status of the last descriptor */
	tdp = NSG_DESC_PTR(dp,
	    dp->tx_ring, SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size));

	status = tdp->d_cmdsts;
	status = LE_32(status);

	DPRINTF(2, (CE_CONT,
	    CONS "%s: time:%d %s: slot:%d, status:0x%b",
	    dp->name, __func__,
	    ddi_get_lbolt(), slot, status, TXSTAT_BITS));

	return (nsg_tx_desc_stat_common(dp, status));
}

static uint64_t
nsg_rx_desc_stat_common(struct gem_dev *dp, uint32_t status, uint32_t extsts)
{
	uint32_t	len;
	uint32_t	flags = 0;

	if ((status & CMDSTS_OWN) == 0) {
		/*
		 * No more received packets because
		 * this buffer is owned by NIC.
		 */
		return (0);
	}

	if ((status & CMDSTS_OK) == 0) {
		/*
		 * Packet with error received
		 */
		DPRINTF(2, (CE_CONT,
		    "!%s: %s: Corrupted packet received, buffer status: %b",
		    dp->name, __func__, status, RXSTAT_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;
		if (status & CMDSTS_RXO) {
			dp->stats.overflow++;
		}
		if (status & (CMDSTS_LONG | CMDSTS_MORE)) {
			dp->stats.frame_too_long++;
		}
		if (status & CMDSTS_RUNT) {
			dp->stats.runt++;
		}
		if (status & (CMDSTS_ISE | CMDSTS_FAE)) {
			dp->stats.frame++;
		}
		if (status & CMDSTS_CRCE) {
			dp->stats.crc++;
		}

		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/*
	 * the packet was received without errors
	 */
	if ((len = (status & CMDSTS_SIZE) - ETHERFCSL) < 0) {
		len = 0;
	}

#if DEBUG_LEVEL > 10
	{
		int	i;
		uint8_t	*bp = dp->rx_buf_head->rxb_buf;

		cmn_err(CE_CONT, CONS "%s: %s: len:%d",
		    dp->name, __func__, len);

		for (i = 0; i < 60; i += 10) {
			cmn_err(CE_CONT, CONS
			    "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			    bp[0], bp[1], bp[2], bp[3], bp[4],
			    bp[5], bp[6], bp[7], bp[8], bp[9]);
		}
		bp += 10;
	}
#endif

#ifdef CONFIG_CKSUM_OFFLOAD
	if ((extsts & (EXTSTS_IPPKT | EXTSTS_IPERR)) == EXTSTS_IPPKT) {
		if ((extsts & (EXTSTS_TCPPKT | EXTSTS_TCPERR)) ==
		    EXTSTS_TCPPKT) {
			flags |= GEM_RX_CKSUM_TCP;
		} else if ((extsts & (EXTSTS_UDPPKT | EXTSTS_UDPERR)) ==
		    EXTSTS_UDPPKT) {
			flags |= GEM_RX_CKSUM_UDP;
		} else {
			flags |= GEM_RX_CKSUM_IPv4;
		}
		DPRINTF(10, (CE_CONT, "!%s: %s: extsts:%b",
		    dp->name, __func__, extsts, EXTSTS_BITS));
	}
#endif /* GEM_CONFIG_CKSUM_OFFLOAD */
	return (GEM_RX_DONE | flags | (len & GEM_RX_LEN));
}

static uint64_t
nsg_rx_desc_stat64(struct gem_dev *dp, int slot, int ndesc)
{
	struct nsg_desc64	*rdp;
	uint32_t		status;
	uint32_t		exsts;

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);

	status = rdp->d_cmdsts;
	exsts = rdp->d_extsts;
	status = LE_32(status);
	exsts = LE_32(exsts);

	DPRINTF(2, (CE_CONT, CONS "%s: %s: time:%d, slot:%d, status:0x%b",
	    dp->name, __func__,
	    ddi_get_lbolt(), slot, status, RXSTAT_BITS));

	return (nsg_rx_desc_stat_common(dp, status, exsts));
}

static uint64_t
nsg_rx_desc_stat32(struct gem_dev *dp, int slot, int ndesc)
{
	struct nsg_desc32	*rdp;
	uint32_t		status;
	uint32_t		exsts;

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);

	status = rdp->d_cmdsts;
	exsts = rdp->d_extsts;
	status = LE_32(status);
	exsts = LE_32(exsts);

	DPRINTF(2, (CE_CONT, CONS "%s: %s: time:%d, slot:%d, status:0x%b",
	    dp->name, __func__,
	    ddi_get_lbolt(), slot, status, RXSTAT_BITS));

	return (nsg_rx_desc_stat_common(dp, status, exsts));
}

static void
nsg_tx_desc_init64(struct gem_dev *dp, int slot)
{
	struct nsg_desc64	*tdp;
	uint64_t		here;
	uint32_t		here_lo;
	uint32_t		here_hi;
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, slot);

	tdp->d_cmdsts = 0;

	/* make a link to this from the previous descriptor */
	here = NSG_DESC_ADDR(dp, dp->tx_ring_dma, slot);
	here_lo = here;
	here_hi = here >> 32;

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, SLOT(slot - 1, tx_ring_size));
	tdp->d_link_lo = LE_32(here_lo);
	tdp->d_link_hi = LE_32(here_hi);
}

static void
nsg_tx_desc_init32(struct gem_dev *dp, int slot)
{
	struct nsg_desc32	*tdp;
	uint32_t		here;
	uint_t			tx_ring_size = dp->gc.gc_tx_ring_size;

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, slot);

	tdp->d_cmdsts = 0;

	/* make a link to this from the previous descriptor */
	here = NSG_DESC_ADDR(dp, dp->tx_ring_dma, slot);

	tdp = NSG_DESC_PTR(dp, dp->tx_ring, SLOT(slot - 1, tx_ring_size));
	tdp->d_link = LE_32(here);
}

static void
nsg_rx_desc_init64(struct gem_dev *dp, int slot)
{
	struct nsg_desc64	*rdp;
	uint64_t		here;
	uint32_t		here_lo;
	uint32_t		here_hi;

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);
	rdp->d_cmdsts = LE_32(CMDSTS_OWN);

	/* make a link to this from the previous descriptor */
	here = NSG_DESC_ADDR(dp, dp->rx_ring_dma, slot);
	here_lo = here;
	here_hi = here >> 32;

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, SLOT(slot - 1, RX_RING_SIZE));
	rdp->d_link_lo = LE_32(here_lo);
	rdp->d_link_hi = LE_32(here_hi);
}

static void
nsg_rx_desc_init32(struct gem_dev *dp, int slot)
{
	struct nsg_desc32	*rdp;
	uint32_t		here;

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, slot);
	rdp->d_cmdsts = LE_32(CMDSTS_OWN);

	/* make a link to this from the previous descriptor */
	here = NSG_DESC_ADDR(dp, dp->rx_ring_dma, slot);

	rdp = NSG_DESC_PTR(dp, dp->rx_ring, SLOT(slot - 1, RX_RING_SIZE));
	rdp->d_link = LE_32(here);
}

static void
nsg_tx_desc_clean64(struct gem_dev *dp, int slot)
{
	((struct nsg_desc64 *)
	    NSG_DESC_PTR(dp, dp->tx_ring, slot))->d_cmdsts = 0;
}

static void
nsg_tx_desc_clean32(struct gem_dev *dp, int slot)
{
	((struct nsg_desc32 *)
	    NSG_DESC_PTR(dp, dp->tx_ring, slot))->d_cmdsts = 0;
}

static void
nsg_rx_desc_clean64(struct gem_dev *dp, int slot)
{
	((struct nsg_desc64 *)
	    NSG_DESC_PTR(dp, dp->rx_ring, slot))->d_cmdsts = LE_32(CMDSTS_OWN);
}

static void
nsg_rx_desc_clean32(struct gem_dev *dp, int slot)
{
	((struct nsg_desc32 *)
	    NSG_DESC_PTR(dp, dp->rx_ring, slot))->d_cmdsts = LE_32(CMDSTS_OWN);
}

/*
 * Device depend interrupt handler
 */
static uint_t
nsg_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flags = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct nsg_dev	*lp = (struct nsg_dev *)dp->private;

	/* read reason and clear interrupts */
	isr = INL(dp, ISR);

	if ((isr & lp->our_intr_bits) == 0) {
		/* not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(3, (CE_CONT,
	    CONS "%s: time:%d %s:called: isr:0x%b rx_active_head: %d",
	    dp->name, ddi_get_lbolt(), __func__,
	    isr, INTR_BITS, dp->rx_active_head));

	if (!dp->mac_active) {
		/* the device is going to stop */
		return (DDI_INTR_CLAIMED);
	}

#ifdef CONFIG_POLLING
	if (dp->poll_interval != lp->last_poll_interval) {
		if (dp->poll_interval) {
			/* polling mode: inhibit interrupts from now */
			OUTL(dp, IHR,
			    min(IH_IH, dp->poll_interval/4000));
		} else {
			/* normal mode */
			OUTL(dp, IHR, 0);
		}

		lp->last_poll_interval = dp->poll_interval;
	}
#ifdef notdef
	/* It doesn't seem to work */
	if (dp->poll_interval > 0) {
		/* schedule next */
		OUTL(dp, CR, CR_SWI);
	}
	if ((isr & ISR_SWI) != 0) {
		/* polling timer expired */
		isr |= ISR_RXOK | ISR_TXOK;
	}
#endif
#endif /* CONFIG_POLLING */

	isr &= lp->our_intr_bits;

	if (isr & ISR_PHY) {
		/*
		 * Link or PHY status has changed
		 */
		DPRINTF(0, (CE_CONT, "!%s: %s: isr:%b",
		    dp->name, __func__, isr, INTR_BITS));
		if (gem_mii_link_check(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}

	if ((isr &
	    (ISR_RXORN | ISR_RXIDLE | ISR_RXERR |
	    ISR_RXDESC | ISR_RXOK)) != 0) {

		gem_receive(dp);
#if 0
		if ((isr & (ISR_RXIDLE | ISR_RXORN)) != 0) {
			DPRINTF(0, (CE_CONT,
			    "%s: rx error: isr %b.",
			    dp->name, isr, INTR_BITS));
			need_to_reset |= (isr & ISR_RXORN) != 0;
		}
#endif
		if ((isr & ISR_RXIDLE) != 0) {
			/*
			 * Make RXDP points the head of receive
			 * buffer list.
			 */
			OUTL(dp, RXDP,
			    NSG_DESC_ADDR(dp, dp->rx_ring_dma,
			    SLOT(dp->rx_active_head, dp->gc.gc_rx_ring_size)));

			/* Restart the receive engine */
			OUTL(dp, CR, lp->cr | CR_RXE);
		}
	}

	if ((isr & (ISR_TXURN | ISR_TXERR | ISR_TXDESC |
	    ISR_TXIDLE | ISR_TXOK)) != 0) {
		/* need to relaim tx buffers */
		if (gem_tx_done(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}

	if ((isr & (ISR_DPERR | ISR_SSERR | ISR_RMABT | ISR_RTABT)) != 0) {
		cmn_err(CE_WARN, "!%s: ERROR interrupt: isr %b.",
		    dp->name, isr, INTR_BITS);
		need_to_reset = B_TRUE;
	}

	if (need_to_reset) {
		gem_restart_nic(dp, B_TRUE);
		flags |= INTR_RESTART_TX;
	}

	DPRINTF(5, (CE_CONT, CONS "%s: %s: return: isr: %b",
	    dp->name, __func__, isr, INTR_BITS));

	return (DDI_INTR_CLAIMED | flags);
}

/* ======================================================== */
/*
 * HW depend MII routine
 */
/* ======================================================== */
#define	MDIO_DELAY(dp)	INL(dp, MEAR)
static void
nsg_mii_sync(struct gem_dev *dp)
{
	int	i;

	for (i = 0; i < 32; i++) {
		OUTL(dp, MEAR, MEAR_MDDIR | MEAR_MDIO);
		MDIO_DELAY(dp);
		OUTL(dp, MEAR, MEAR_MDDIR | MEAR_MDIO | MEAR_MDC);
		MDIO_DELAY(dp);
	}
}

static uint16_t
nsg_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint32_t	data;

	cmd = MII_READ_CMD(dp->mii_phy_addr, reg);

	for (i = 31; i >= 18; i--) {
		data = ((cmd >> i) & 1) <<  MEAR_MDIO_SHIFT;
		OUTL(dp, MEAR, data | MEAR_MDDIR);
		MDIO_DELAY(dp);
		OUTL(dp, MEAR, data | MEAR_MDDIR | MEAR_MDC);
		MDIO_DELAY(dp);
	}

	/* tern arould cycle */
	OUTL(dp, MEAR, data | MEAR_MDDIR);
	MDIO_DELAY(dp);

	/* get response from PHY */
	OUTL(dp, MEAR, MEAR_MDC);
	MDIO_DELAY(dp);
	OUTL(dp, MEAR, 0);
	if ((INL(dp, MEAR) & MEAR_MDIO) != 0) {
		DPRINTF(2, (CE_CONT, "!%s: PHY@%d not responded",
		    dp->name, dp->mii_phy_addr));
	}
	/* terminate response cycle */
	OUTL(dp, MEAR, MEAR_MDC);

	for (i = 16; i > 0; i--) {
		OUTL(dp, MEAR, 0);
		ret = (ret << 1) | ((INL(dp, MEAR) >> MEAR_MDIO_SHIFT) & 1);
		OUTL(dp, MEAR, MEAR_MDC);
		MDIO_DELAY(dp);
	}

	/* terminate data transmition from PHY */
	OUTL(dp, MEAR, 0);
	MDIO_DELAY(dp);
	OUTL(dp, MEAR, MEAR_MDC);
	MDIO_DELAY(dp);
#if 0
	if (reg == MII_STATUS) {
		ret |= MII_STATUS_LINKUP;
	}
#endif
#ifdef CONFIG_NO_1G
	if (reg == MII_STATUS && ret != 0 && ret != 0xffffU) {
		ret &= ~MII_STATUS_XSTATUS;
	}
#endif
	return (ret);
}

static void
nsg_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint32_t	cmd;
	int		i;
	uint32_t	data;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, reg, val);

	for (i = 31; i >= 0; i--) {
		data = ((cmd >> i) & 1) << MEAR_MDIO_SHIFT;
		OUTL(dp, MEAR, data | MEAR_MDDIR);
		MDIO_DELAY(dp);
		OUTL(dp, MEAR, data | MEAR_MDDIR | MEAR_MDC);
		MDIO_DELAY(dp);
	}

	/* send two 0s to terminate write cycle. */
	for (i = 0; i < 2; i++) {
		OUTL(dp, MEAR, MEAR_MDDIR);
		MDIO_DELAY(dp);
		OUTL(dp, MEAR, MEAR_MDDIR | MEAR_MDC);
		MDIO_DELAY(dp);
	}
	OUTL(dp, MEAR, MEAR_MDDIR);
	MDIO_DELAY(dp);
	OUTL(dp, MEAR, MEAR_MDC);
	MDIO_DELAY(dp);
}
#undef MDIO_DELAY

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
nsg_attach_chip(struct gem_dev *dp)
{
	struct nsg_dev	*lp = (struct nsg_dev *)dp->private;
	int		i;

	DPRINTF(4, (CE_CONT, CONS "%s: %s called", dp->name, __func__));
#if DEBUG_LEVEL > 10
	/* dump eeprom */
	for (i = 0; i < 16; i++) {
		cmn_err(CE_CONT, "[%d] %04x", i, nsg_read_eeprom(dp, i));
	}
#endif
	/* read MAC address */
	if (!gem_get_mac_addr_conf(dp)) {
		if (!nsg_get_mac_addr(dp)) {
			cmn_err(CE_WARN,
			    "!%s: %s: cannot get mac address",
			    dp->name, __func__);
			gem_generate_macaddr(dp, dp->dev_addr.ether_addr_octet);
		}
	}

	/* fix dma parameters */
	dp->txmaxdma = max(dp->txmaxdma, 1024);
	dp->rxmaxdma = max(dp->txmaxdma, 1024);

#ifdef GEM_CONFIG_GLDv3
#ifdef CONFIG_CKSUM_OFFLOAD
	dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_HEADER_IPv4;
	dp->misc_flag |= GEM_VLAN_HARD;
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
#endif


	return (GEM_SUCCESS);
}

static int
nsgattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	uint8_t			cachelinesz;
#endif
	struct chip_info	*p;
	struct gem_dev		*dp;
	struct nsg_dev		*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	boolean_t		pci64;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, CONS "%s%d: nsgattach: called", drv_name, unit));

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
	for (i = 0, p = nsg_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_WARN,
	    "!%s%d: nsg_attach: wrong PCI venid/devid (0x%x, 0x%x)",
	    drv_name, unit, vid, did);
	pci_config_teardown(&conf_handle);
	goto err;

chip_found:
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
#if 1
	    PCI_COMM_MEMWR_INVAL |
#endif
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	cachelinesz = pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ);
	if (cachelinesz < 16) {
		pci_config_put8(conf_handle, PCI_CONF_CACHE_LINESZ, 16);
		cachelinesz = pci_config_get8(conf_handle,
		    PCI_CONF_CACHE_LINESZ);
	}
#if 1
	pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 0x80); /* XXX */
#endif
	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:

		DPRINTF(0, (CE_CONT,
		    CONS "%s%d: "
		    "ilr 0x%08x, latency_timer:0x%02x, cachelinesz:0x%02x",
		    drv_name, unit, iline, latim, cachelinesz));
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &nsg_dev_attr, &base, &regs_ha) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!%s%d: gem_regs_map_setup failed",
			    drv_name, unit);
			goto err;
		}
		/* check if the pci bus supports 64bit */

		pci64 = (ddi_get32(regs_ha, (uint32_t *)(base + CFG)) &
		    CFG_PCI64_DET) || nsg_use_dac;
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
		gcp->gc_tx_desc_unit_shift =
		    pci64 ? NSG_DESC_SIZE64_SHIFT : NSG_DESC_SIZE32_SHIFT;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;

		/* auto pad isn't secure */
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = nsg_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_unit_shift =
		    pci64 ? NSG_DESC_SIZE64_SHIFT : NSG_DESC_SIZE32_SHIFT;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = nsg_rx_copy_thresh;

		/* map attributes */
		gcp->gc_dev_attr = nsg_dev_attr;
		gcp->gc_buf_attr = nsg_buf_attr;
		gcp->gc_desc_attr = nsg_dev_attr;

		/* dma attributes */
		if (pci64) {
			gcp->gc_dma_attr_desc = nsg_dma_attr_desc64;
		} else {
			gcp->gc_dma_attr_desc = nsg_dma_attr_desc32;
		}

		if (pci64) {
			gcp->gc_dma_attr_txbuf = nsg_dma_attr_buf64;
		} else {
			gcp->gc_dma_attr_txbuf = nsg_dma_attr_buf32;
		}
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		if (pci64) {
			gcp->gc_dma_attr_rxbuf = nsg_dma_attr_buf32;
		} else {
			gcp->gc_dma_attr_rxbuf = nsg_dma_attr_buf32;
		}
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC / 5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT * 2;	/* 10 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* setting for general PHY */
		gcp->gc_mii_an_delay =  0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &nsg_attach_chip;
		gcp->gc_reset_chip = &nsg_reset_chip;
		gcp->gc_init_chip = &nsg_init_chip;
		gcp->gc_start_chip = &nsg_start_chip;
		gcp->gc_stop_chip = &nsg_stop_chip;
#ifdef USE_MULTICAST_HASHTBL
		gcp->gc_multicast_hash = &nsg_mcast_hash;
#endif
		gcp->gc_set_rx_filter = &nsg_set_rx_filter;
		gcp->gc_set_media = &nsg_set_media;
		gcp->gc_get_stats = &nsg_get_stats;
		gcp->gc_interrupt = &nsg_interrupt;

		/* descriptor operation */
		if (pci64) {
			gcp->gc_tx_desc_write = &nsg_tx_desc_write64;
			gcp->gc_tx_start = &nsg_tx_start64;
			gcp->gc_rx_desc_write = &nsg_rx_desc_write64;

			gcp->gc_tx_desc_stat = &nsg_tx_desc_stat64;
			gcp->gc_rx_desc_stat = &nsg_rx_desc_stat64;
			gcp->gc_tx_desc_init = &nsg_tx_desc_init64;
			gcp->gc_rx_desc_init = &nsg_rx_desc_init64;
			gcp->gc_tx_desc_clean = &nsg_tx_desc_clean64;
			gcp->gc_rx_desc_clean = &nsg_rx_desc_clean64;
		} else {
			gcp->gc_tx_desc_write = &nsg_tx_desc_write32;
			gcp->gc_tx_start = &nsg_tx_start32;
			gcp->gc_rx_desc_write = &nsg_rx_desc_write32;

			gcp->gc_tx_desc_stat = &nsg_tx_desc_stat32;
			gcp->gc_rx_desc_stat = &nsg_rx_desc_stat32;
			gcp->gc_tx_desc_init = &nsg_tx_desc_init32;
			gcp->gc_rx_desc_init = &nsg_rx_desc_init32;
			gcp->gc_tx_desc_clean = &nsg_tx_desc_clean32;
			gcp->gc_rx_desc_clean = &nsg_rx_desc_clean32;
		}

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &nsg_mii_sync;
		gcp->gc_mii_read = &nsg_mii_read;
		gcp->gc_mii_write = &nsg_mii_write;
		gcp->gc_mii_tune_phy = NULL;
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
		gcp->gc_mii_hw_link_detection = B_TRUE;

		lp = (struct nsg_dev *)
		    kmem_zalloc(sizeof (struct nsg_dev), KM_SLEEP);

		lp->chip = p;
		lp->revid = rev;
		lp->cr = 0;
		lp->pci_addr_64 = pci64;

		DPRINTF(0, (CE_CONT, CONS "%s%d: chip:%s rev:0x%02x",
		    drv_name, unit, p->chip_name, rev));

		/* offload and jumbo frame */
		gcp->gc_max_lso = 0;
		gcp->gc_max_mtu = 1920;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);
		gcp->gc_nintrs_req = 1;

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));
		kmem_free(gcp, sizeof (*gcp));

		if (dp == NULL) {
			goto err_freelp;
		}

		return (DDI_SUCCESS);

err_freelp:
		kmem_free(lp, sizeof (struct nsg_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
nsgdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
GEM_STREAM_OPS(nsg_ops, nsgattach, nsgdetach);
#else
static	struct module_info nsgminfo = {
	0,			/* mi_idnum */
	"nsg",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit nsgrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&nsgminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit nsgwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&nsgminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	nsg_info = {
	&nsgrinit,	/* st_rdinit */
	&nsgwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_nsg_ops = {
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
	&nsg_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops nsg_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	nsgattach,	/* devo_attach */
	nsgdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_nsg_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce,     /* devo_quiesce */
#endif
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&nsg_ops,	/* driver ops */
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
	int 	status;

	DPRINTF(2, (CE_CONT, "!nsg: _init: called"));

	status = gem_mod_init(&nsg_ops, "nsg");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&nsg_ops);
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

	DPRINTF(2, (CE_CONT, "!: nsg_fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&nsg_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
