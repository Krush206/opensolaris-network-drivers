/*
 * tne_gem.c: TI ThunderLAN TNETE100A Fast Ethernet MAC driver
 *
 * Copyright (c) 2003-2011 Masayuki Murayama.  All rights reserved.
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

#pragma ident "@(#)tne_gem.c	1.15 11/09/19"

/*
 Change log
 0.8.5  05/10/2004
	clean up tne_interrupt().
        05/29/2004
	disabling interrupts while tne_interrupt()
	supported cards added
 0.8.6  06/19/2004
 ==============================
 2.4.0  12/29/2006
	reading mac address was fixed for Olicom products.
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

#define	TNE_MAXTXFRAGS	7
#define	TNE_MAXRXFRAGS	7

#include "tnete100areg.h"

char	ident[] = "TI ThunderLAN driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int tne_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (tne_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	OUTB_DIO(dp, reg, val)	OUTB(dp, DIO_DATA | ((reg) & 3), val)
#define	OUTW_DIO(dp, reg, val)	OUTW(dp, DIO_DATA | ((reg) & 3), val)
#define	OUTL_DIO(dp, reg, val)	OUTL(dp, DIO_DATA, val)
#define	INB_DIO(dp, reg)	INB(dp, DIO_DATA | ((reg) & 3))
#define	INW_DIO(dp, reg)	INW(dp, DIO_DATA | ((reg) & 3))
#define	INL_DIO(dp, reg)	INL(dp, DIO_DATA)
#define	FLSHL(dp, reg)		INL(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)

#define	FLSHB(dp, reg)	INB(dp, reg)
#define	FLSHW(dp, reg)	INW(dp, reg)
#define	FLSHB(dp, reg)	INB(dp, reg)
#define	FLSHW(dp, reg)	INW(dp, reg)
#define	FLSHL(dp, reg)	INL(dp, reg)

#define	DUAL_PHY(lp)	((lp)->is_dual_phy)

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif

/*
 * Our configuration
 */
#ifdef TEST_TXDESC_FULL
#define	TX_RING_SIZE	4
#define	TX_BUF_SIZE	64
#endif
#ifdef TEST_RX_EMPTY
#define	RX_BUF_SIZE	8
#endif

#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	TX_BUF_SIZE
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

#define	ONESEC		(drv_usectohz(1*1000000))

static int	tne_tx_copy_thresh = 256;
static int	tne_rx_copy_thresh = 256;

#ifndef INTR_DELAY
#define	INTR_DELAY	0
#endif

struct tne_dev {
	struct chip_info	*chip;

	boolean_t	tx_list_loaded;
	uint_t		tx_list_done;
	int		tx_list_len;
	int		tx_list_head;

	boolean_t	rx_list_loaded;
	uint_t		rx_list_done;
	int		rx_list_len;
	int		rx_list_head;

	boolean_t	is_dual_phy;
	int		curr_phy;

#ifdef CONFIG_POLLING
	int		last_poll_interval;
#endif
	uint8_t		netcmd;
	uint8_t		rev_id;
	uint8_t		prom[256];
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
} tne_chiptbl[] = {
	{0x0e11, 0xae32, "Compaq Netelligent 10/100 TX UTP"},
	{0x0e11, 0xae34, "Compaq Netelligent 10 T UTP"},
	{0x0e11, 0xae35, "Compaq Integrated NetFlex-3/P"},
	{0x0e11, 0xae40, "Compaq Netelligent Dual 10/100 TX UTP"},
	{0x0e11, 0xae43, "Compaq Netelligent Integrated 10/100 TX UTP"},
	{0x0e11, 0xb011, "Compaq Netelligent 10/100 TX Embedded UTP"},
	{0x0e11, 0xb012, "Compaq Netelligent 10 T/2 PCI UTP/Coax"},
	{0x0e11, 0xb030, "Compaq Netelligent 10/100 TX UTP"},
	{0x108d, 0x0012, "Olicom OC-2325"},
	{0x108d, 0x0013, "Olicom OC-2183/2185"},
	{0x108d, 0x0014, "Olicom OC-2326"},
	{0x104c, 0x0500, "TI ThunderLAN TNETE100A"},
};
#define	CHIPTABLESIZE	(sizeof (tne_chiptbl)/sizeof (struct chip_info))

/* ======================================================== */

/* mii operations */
static uint16_t tne_mii_read_raw(struct gem_dev *dp, uint_t phy, uint_t reg);
static void tne_mii_write_raw(struct gem_dev *dp,
    uint_t phy, uint_t reg, uint16_t val);
static void  tne_mii_sync(struct gem_dev *);
static uint16_t  tne_mii_read(struct gem_dev *, uint_t);
static void tne_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int tne_attach_chip(struct gem_dev *);
static int tne_reset_chip(struct gem_dev *);
static int tne_init_chip(struct gem_dev *);
static int tne_start_chip(struct gem_dev *);
static int tne_stop_chip(struct gem_dev *);
static int tne_set_media(struct gem_dev *);
static int tne_set_rx_filter(struct gem_dev *);
static int tne_get_stats(struct gem_dev *);

/* descriptor operations */
static int tne_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void tne_tx_start(struct gem_dev *dp, int start_slot, int nslots);
static void tne_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void tne_rx_start(struct gem_dev *dp, int start_slot, int nslots);
static uint_t tne_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t tne_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void tne_tx_desc_init(struct gem_dev *dp, int slot);
static void tne_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t tne_interrupt(struct gem_dev *dp);

static void tne_load_tx_list(struct gem_dev *dp);
static void tne_load_rx_list(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr tne_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr tne_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t tne_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t tne_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	8,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	8,			/* dma_attr_align */
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
 * HW manupilation routines
 */
/* ======================================================== */

static int
tne_reset_chip(struct gem_dev *dp)
{
	int	i;
	uint8_t	val;
	struct tne_dev	*lp = dp->private;

	/*
	 * Issue a reset command to ThunderLAN by asserting the AdRst bit
	 * in the HOST_CMD register
	 */
	OUTL(dp, HOST_CMD, HC_AdRst);

	OUTW(dp, DIO_ADR, NETCMD);
	lp->netcmd = INB_DIO(dp, NETCMD);

	/*
	 * unreset MII PHY
	 */
	OUTW(dp, DIO_ADR, NETSIO);
	OUTB_DIO(dp, NETSIO, (INB_DIO(dp, NETSIO) | NSIO_NMRST) & ~NSIO_MINTEN);

	/* synchronize with mii_state */
	dp->mii_state = MII_STATE_RESETTING;

	DPRINTF(2, (CE_CONT, "!%s: %s", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
tne_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct tne_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Disable interrupts by asserting the Ints_off bit int HOST_CMD */
	OUTL(dp, HOST_CMD, HC_IntsOff);

	/* Setup the NetConfig registers for the appropriate options */
	ASSERT((lp->netcmd & NCMD_NRESET) == 0);

	OUTW(dp, DIO_ADR, NETCFG);
	OUTW_DIO(dp, NETCFG,
	    (dp->mii_phy_addr == 31 || DUAL_PHY(lp))
	    ? (NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En)
	    : (NCFG_PEF | NCFG_ONEchn));

	/* Setup the BSIZEreg for the correct burst size */
	val = 0;
	if (dp->txmaxdma <= 32) {
		val |= BS_TX_16;
	} else if (dp->txmaxdma < 64) {
		val |= BS_TX_32;
	} else if (dp->txmaxdma < 128) {
		val |= BS_TX_64;
	} else if (dp->txmaxdma < 256) {
		val |= BS_TX_128;
	} else if (dp->txmaxdma < 512) {
		val |= BS_TX_256;
	} else {
		val |= BS_TX_512;
	}
	if (dp->rxmaxdma <= 32) {
		val |= BS_RX_16;
	} else if (dp->rxmaxdma < 64) {
		val |= BS_RX_32;
	} else if (dp->rxmaxdma < 128) {
		val |= BS_RX_64;
	} else if (dp->rxmaxdma < 256) {
		val |= BS_RX_128;
	} else if (dp->rxmaxdma < 512) {
		val |= BS_RX_256;
	} else {
		val |= BS_RX_512;
	}
	OUTW(dp, DIO_ADR, BSIZEreg);
	OUTB_DIO(dp, BSIZEreg, val);

	/* Setup the correct Tx commit level in the Acommit register */
	if (dp->txthr <= 64) {
		val = AC_TX_64;
	} else if (dp->txthr <= 128) {
		val = AC_TX_128;
	} else if (dp->txthr <= 256) {
		val = AC_TX_256;
	} else if (dp->txthr <= 512) {
		val = AC_TX_512;
	} else if (dp->txthr <= 1024) {
		val = AC_TX_1024;
	} else {
		val = AC_TX_SF;
	}
	OUTW(dp, DIO_ADR, Acommit);
	OUTB_DIO(dp, Acommit, val);
#if 0
	/* Load the appropriate interrupt pacing timer in Ld_Tmr in HOST_CMD */
	OUTL(dp, HOST_CMD, HC_LdTmr | 0);
#endif
	/* Load the appropriate Tx threshold value in Ld_Thr in HOST_CMD */
#ifdef CONFIG_POLLING
	OUTL(dp, HOST_CMD, HC_LdThr | 1);
#else
	OUTL(dp, HOST_CMD, HC_LdThr | 0);
#endif
#if 0
	/* Unreset the MII by asserting NMRDT in NetSio */

	/* Initialize PHY layer */
#endif
	/* Setup the network status mask register */
	OUTW(dp, DIO_ADR, NETMASK);
	OUTB_DIO(dp, NETMASK, 0);

	OUTW(dp, DIO_ADR, MaxRx);
#ifdef GEM_CONFIG_GLDv3
	val = min(0xffff, dp->mtu + sizeof (struct ether_header) + 4);
#else
	val = min(0xffff, dp->mtu + sizeof (struct ether_header));
#endif
	OUTW_DIO(dp, MaxRx, (uint16_t)val);

	/* enable NETCMD register */
	lp->netcmd |= NCMD_NRESET | NCMD_NWRAP;
	OUTW(dp, DIO_ADR, NETCMD);
	OUTB_DIO(dp, NETCMD, lp->netcmd);

	lp->tx_list_loaded = B_FALSE;
	lp->tx_list_len = 0;
	lp->tx_list_done = 0;
	lp->tx_list_head = 0;

	lp->rx_list_loaded = B_FALSE;
	lp->rx_list_len = 0;
	lp->rx_list_done = 0;
	lp->rx_list_head = 0;

	return (GEM_SUCCESS);
}

static uint32_t
tne_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	int		i;
	uint8_t		hash;
	uint32_t	addr0, addr1;

	/*
	 * According with my experience, the hash function must return
	 * following 6 bit-width value:
	 *   01:00:5e:00:00:01 -> 0x0e (001110)
	 *   01:00:5e:00:00:04 -> 0x0b (001011)
	 */
	hash = 0;
	addr0 = addr[0] << 16 | addr[1] << 8 | addr[2];
	addr1 = addr[3] << 16 | addr[4] << 8 | addr[5];

	for (i = 0; i < 3*8/6; i++) {
		hash ^= addr0 ^ addr1;
		/* shift 6 bits right */
		addr0 >>= 6;
		addr1 >>= 6;
	}
	return (hash & 0x3f);
}

static void
tne_load_cam(struct gem_dev *dp, int ix, uint8_t *ether_addr)
{
	int	i;
	int	offset;

	offset = Areg_0 + ix * ETHERADDRL;
	for (i = 0; i < ETHERADDRL; i++) {
		OUTW(dp, DIO_ADR, offset + i);
		OUTB_DIO(dp, offset + i, ether_addr[i]);
	}
}

static int
tne_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	int		cam_ix;
	uint8_t		mode;
	uint64_t	hash;
	uint16_t	addr;
	static uint8_t	ff6[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	static uint8_t	zero6[] = {0, 0, 0, 0, 0, 0};
	uint8_t		*filler = ff6;
	struct tne_dev	*lp = dp->private;

#if DEBUG_LEVEL > 3
	for (i = 0; i < dp->mc_count; i++) {
		cmn_err(CE_CONT,
		    "!%s: adding mcast(%d) %02x:%02x:%02x:%02x:%02x:%02x -> %d",
		    dp->name, i,
		    dp->mc_list[i].addr.ether_addr_octet[0],
		    dp->mc_list[i].addr.ether_addr_octet[1],
		    dp->mc_list[i].addr.ether_addr_octet[2],
		    dp->mc_list[i].addr.ether_addr_octet[3],
		    dp->mc_list[i].addr.ether_addr_octet[4],
		    dp->mc_list[i].addr.ether_addr_octet[5],
		    tne_mcast_hash(dp, dp->mc_list[i].addr.ether_addr_octet));
	}
#endif
	hash = 0ULL;
	cam_ix = 0;
	mode = 0;
#define	NCAMS	4

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* clear cam */
		filler = zero6;
		goto x;
	}

	/* accept all rx packets while we are changing rx filter */
	OUTW(dp, DIO_ADR, NETCMD);
	OUTB_DIO(dp, NETCMD, lp->netcmd | NCMD_CAF);

	/* write my station address into cam-entry #0 */
	tne_load_cam(dp, cam_ix++, &dp->cur_addr.ether_addr_octet[0]);

	if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscous */
		mode = NCMD_CAF;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) && dp->mc_count > 32) {
		/* allmulti */
		hash = ~0ULL;
	} else if (dp->mc_count > NCAMS - 1) {
		/* use multicast hash table */
		for (i = 0; i < dp->mc_count; i++) {
			hash |= 1ULL << dp->mc_list[i].hash;
		}
	} else {
		/* use CAM-based multicast filter */
		for (i = 0; i < dp->mc_count; i++) {
			tne_load_cam(dp, cam_ix++,
			    &dp->mc_list[i].addr.ether_addr_octet[0]);
		}
	}
x:
	/* fill rest of perfect match filter entries with ff:ff:ff:ff:ff:ff */
	for (; cam_ix < NCAMS; cam_ix++) {
		tne_load_cam(dp, cam_ix, filler);
	}

	/* update hash table */
	OUTW(dp, DIO_ADR, HASH1 + 0);
	OUTL_DIO(dp, HASH1, (uint32_t)hash);
	OUTW(dp, DIO_ADR, HASH1 + 4);
	OUTL_DIO(dp, HASH1, (uint32_t)(hash >> 32));

	/* update rx filter mode */
	lp->netcmd = (lp->netcmd & ~NCMD_CAF) | mode;
	OUTW(dp, DIO_ADR, NETCMD);
	OUTB_DIO(dp, NETCMD, lp->netcmd);

	return (GEM_SUCCESS);
}

static int
tne_set_media(struct gem_dev *dp)
{
	uint_t		old;
	struct tne_dev	*lp = dp->private;

	if (DUAL_PHY(lp)) {
		uint_t		right_phy;

		/* check current phy */
		if ((dp->anadv_100fdx || dp->anadv_100hdx) &&
		    (dp->mii_lpable &
		    (MII_ABILITY_100BASE_TX_FD | MII_ABILITY_100BASE_TX))) {
			/* we must use external phy device */
			right_phy = dp->mii_phy_addr;
		} else {
			/* we must use internal phy device */
			right_phy = 31;
		}

		if (lp->curr_phy != right_phy) {
			/* switch PHY */
			cmn_err(CE_NOTE,
			    "!%s: %s switching phy from @%d to @%d",
			    dp->name, __func__, lp->curr_phy, right_phy);

			tne_mii_sync(dp);
			tne_mii_write_raw(dp, lp->curr_phy,
			    MII_CONTROL,
			    MII_CONTROL_ISOLATE);

			lp->curr_phy = right_phy;

			tne_mii_sync(dp);
			tne_mii_write_raw(dp, lp->curr_phy,
			    MII_CONTROL,
			    (dp->anadv_autoneg ? MII_CONTROL_ANE : 0) |
			    (dp->full_duplex ? MII_CONTROL_FDUPLEX : 0));

			/*
			 * Force to start autonegotiation with the right phy.
			 * It will cause link-down.
			 */
			dp->mii_state = MII_STATE_UNKNOWN;

			return (GEM_SUCCESS);
		}
	}

	if (dp->mii_phy_addr == 31 ||
	    (DUAL_PHY(lp) && lp->curr_phy == 31)) {
		/* now we are using internal 10M phy */
		if (dp->anadv_autoneg) {
			/* we must configure current duplex mode explicitly */
			tne_mii_sync(dp);
			tne_mii_write_raw(dp,
			    lp->curr_phy,
			    MII_CONTROL,
			    MII_CONTROL_ANE |
			    (dp->full_duplex ? MII_CONTROL_FDUPLEX : 0));
		}
	}

	/*
	 * Notify current duplex mode to MAC
	 */
	old = lp->netcmd;
	lp->netcmd &= ~NCMD_DUPLEX;
	if (dp->full_duplex) {
		lp->netcmd |= NCMD_DUPLEX;
	}

	if (old != lp->netcmd) {
		OUTW(dp, DIO_ADR, NETCMD);
		OUTB_DIO(dp, NETCMD, lp->netcmd);
	}

	/* Load the appropriate interrupt pacing timer in Ld_Tmr in HOST_CMD */
	OUTL(dp, HOST_CMD, HC_LdTmr | 0);

	return (GEM_SUCCESS);
}

static int
tne_start_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct tne_dev	*lp = dp->private;

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* Enable interrupts by asserting the IntsOn bit in HOST_CMD */
		OUTL(dp, HOST_CMD, HC_IntsOn);
	}

	return (GEM_SUCCESS);
}

static int
tne_stop_chip(struct gem_dev *dp)
{
	uint8_t		val;
	uint16_t	stat;
	struct tne_dev	*lp = dp->private;

	OUTL(dp, HOST_CMD, HC_IntsOff);

	OUTL(dp, HOST_CMD, HC_TxSTOP(0));
	OUTL(dp, HOST_CMD, HC_RxSTOP(0));

	/* hw reset may be required */
	(void) tne_reset_chip(dp);

	return (GEM_SUCCESS);
}

/*
 * IIC EEPROM I/O routines
 */
#define	NETSIO_SET(dp, bit)	\
	OUTB_DIO(dp, NETSIO, INB_DIO(dp, NETSIO) | (bit))
#define	NETSIO_CLR(dp, bit)	\
	OUTB_DIO(dp, NETSIO, INB_DIO(dp, NETSIO) & ~(bit))

#define	TNE_EEPROM_DELAY(dp)	{ INB_DIO(dp, NETSIO); INB_DIO(dp, NETSIO); }

static void
tne_iic_start(struct gem_dev *dp)
{
	NETSIO_SET(dp, NSIO_EDATA);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);
	NETSIO_CLR(dp, NSIO_EDATA);
	TNE_EEPROM_DELAY(dp);
	NETSIO_CLR(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);
}

static void
tne_iic_stop(struct gem_dev *dp)
{
	/* send EEPROM stop access sequence */
	NETSIO_CLR(dp, NSIO_EDATA);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_EDATA);
	TNE_EEPROM_DELAY(dp);
	NETSIO_CLR(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
}

static boolean_t
tne_iic_ack(struct gem_dev *dp)
{
	boolean_t	ack;

	/* disable tx */
	NETSIO_CLR(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);

	/* get ack */
	ack = (INB_DIO(dp, NETSIO) & NSIO_EDATA) == 0;
	NETSIO_CLR(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);	/* XX */

#ifdef notdef
	NETSIO_SET(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
#endif
	return (ack);
}

static void
tne_iic_send(struct gem_dev *dp, uint8_t data)
{
	uint_t	i;

	for (i = 0x80; i; i >>= 1) {
		if (data & i) {
			/* send 1 */
			NETSIO_SET(dp, NSIO_EDATA);
		} else {
			/* send 0 */
			NETSIO_CLR(dp, NSIO_EDATA);
		}
		TNE_EEPROM_DELAY(dp);	/* XX */
		NETSIO_SET(dp, NSIO_ECLOK);
		TNE_EEPROM_DELAY(dp);
		NETSIO_CLR(dp, NSIO_ECLOK);
	}
	TNE_EEPROM_DELAY(dp);
	/* now clock is in low state */
}

static uint8_t
tne_iic_recv(struct gem_dev *dp, boolean_t last)
{
	uint_t	i;
	uint8_t	data;

	/* assume clock is in low state */
	NETSIO_CLR(dp, NSIO_ETXEN);

	/* clock bits in from EDATA and construct data in tmp */
	for (i = 8; i; i--) {
		NETSIO_SET(dp, NSIO_ECLOK);
		TNE_EEPROM_DELAY(dp);
		data = (data << 1)
		    | ((INB_DIO(dp, NETSIO) >> NSIO_EDATA_SHIFT) & 1);
		NETSIO_CLR(dp, NSIO_ECLOK);
		TNE_EEPROM_DELAY(dp);
	}

	/* send EEPROM ack */
	if (last) {
		NETSIO_SET(dp, NSIO_EDATA);
	} else {
		NETSIO_CLR(dp, NSIO_EDATA);
	}
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ETXEN);
	TNE_EEPROM_DELAY(dp);
	NETSIO_SET(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);
	NETSIO_CLR(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);
	/* now clock is low */

	return (data);
}

static int8_t
tne_read_eeprom(struct gem_dev *dp, uint8_t offset)
{
	int	i;
	int	tmp;

	/* select NETSIO register */
	OUTW(dp, DIO_ADR, NETSIO);

	/* let clock low */
	NETSIO_CLR(dp, NSIO_ECLOK);
	TNE_EEPROM_DELAY(dp);

	/* send EEPROM start sequence */
	tne_iic_start(dp);

	/*
	 * put EEPROM device identifier, address, and write
	 * command on bus
	 */
#define	EEPROM_DEVID	0xa
#define	EEPROM_WRITE(a)	(((EEPROM_DEVID << 4) | 0) | ((a) << 1))
#define	EEPROM_READ(a)	(((EEPROM_DEVID << 4) | 1) | ((a) << 1))

	tne_iic_send(dp, EEPROM_WRITE(0));

	/* EEPROM should have acked */
	if (!tne_iic_ack(dp)) {
		cmn_err(CE_WARN, "eeprom WRITE ack failed");
		return (0);
	}

	/* send address on EEPROM to read from */
	NETSIO_SET(dp, NSIO_ETXEN);
	tne_iic_send(dp, offset);

	/* EEPROM should have acke'd addres received */
	if (!tne_iic_ack(dp)) {
		cmn_err(CE_WARN, "eeprom addr ack failed");
		return (0);
	}

	/* send EEPROM start access sequence */
	tne_iic_start(dp);

	/*
	 * put EEPROM device identifier, address, and read
	 * command on bus
	 */
	tne_iic_send(dp, EEPROM_READ(0));

	/* EEPROM should have acked */
	if (!tne_iic_ack(dp)) {
		cmn_err(CE_WARN, "eeprom read ack failed");
		return (0);
	}
	/* receive a 8bit-date from EEPROM and send EEPROM nack */
	tmp = tne_iic_recv(dp, B_FALSE);

	/* send EEPROM stop access sequence */
	tne_iic_stop(dp);

	/* now data is high, clock is high */

	DPRINTF(4, (CE_CONT, "!%s: %s: ret:%02x", dp->name, __func__, tmp));

	return (tmp);
}

#ifdef DEBUG_LEVEL
static void
tne_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	struct tne_dev	*lp = dp->private;

	for (i = 0; i < 256; i++) {
		lp->prom[i] = tne_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 256; i += 8) {
		cmn_err(CE_CONT,
		    "!0x%02x: 0x%02x 0x%02x 0x%02x 0x%02x "
		    "0x%02x 0x%02x 0x%02x 0x%02x",
		    i,
		    lp->prom[i  ], lp->prom[i+1], lp->prom[i+2], lp->prom[i+3],
		    lp->prom[i+4], lp->prom[i+5], lp->prom[i+6], lp->prom[i+7]);
	}
}
#endif /* DEBUG_LEVEL */

static int
tne_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	val;
	int		mac_offset;
	struct tne_dev	*lp = dp->private;

#if DEBUG_LEVEL >= 0
	OUTW(dp, DIO_ADR, DEFAULT_REV);
	val = INB_DIO(dp, DEFAULT_REV);
	cmn_err(CE_CONT, "!%s: rev_id: 0x%02x", dp->name, val);
#endif
	/* get factory mac address */
	if (lp->chip->venid == 0x108d) {
		/*
		 * Olicom products have network byte ordered mac
		 * address at non-standard offset in eeprom.
		 */
		for (i = 0; i < ETHERADDRL; i++) {
			dp->dev_addr.ether_addr_octet[i] =
			    tne_read_eeprom(dp, 0xf8 + (i ^ 1));
		}
	} else {
		for (i = 0; i < ETHERADDRL; i++) {
			dp->dev_addr.ether_addr_octet[i] =
			    tne_read_eeprom(dp, 0x83 + i);
		}
	}
	/* check if the mac address is valid. */
	if ((dp->dev_addr.ether_addr_octet[0] & 0x1) ||
	    bcmp(dp->dev_addr.ether_addr_octet,
	    "\0\0\0\0\0\0", ETHERADDRL) == 0) {
		if (!gem_get_mac_addr_conf(dp)) {
			gem_generate_macaddr(dp, dp->dev_addr.ether_addr_octet);
		}
	}
#if DEBUG_LEVEL > 3
	tne_eeprom_dump(dp);
#endif
	/* Clear the statistics by reading the statistic registers */
	(void) tne_get_stats(dp);
	bzero(&dp->stats, sizeof (dp->stats));
#if 0
	/* now jumbo frame is handled by solais kernel */
	/* fix mtu */
#ifdef GEM_CONFIG_GLDv3
	dp->mtu = min(dp->mtu, 64*1024 - sizeof (struct ether_header) - 4);
#else
	dp->mtu = min(dp->mtu, 64*1024 - sizeof (struct ether_header));
#endif
#endif

#ifdef GEM_CONFIG_GLDv3
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif

	return (GEM_SUCCESS);
}

static int
tne_get_stats(struct gem_dev *dp)
{
	uint_t	x;
	uint_t	val;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	OUTW(dp, DIO_ADR, Tx_UN_GOOD);
	val = INL_DIO(dp, Tx_UN_GOOD);
	dp->stats.underflow += x = (uint8_t)(val >> 24);
	dp->stats.errxmt += x;

	OUTW(dp, DIO_ADR, Rx_OV_GOOD);
	val = INL_DIO(dp, Rx_OV_GOOD);
	dp->stats.overflow += x = (uint8_t)(val >> 24);
	dp->stats.errrcv += x;

	OUTW(dp, DIO_ADR, DeferredTx);
	val = INL_DIO(dp, DeferredTx);
	dp->stats.defer  += (uint16_t)(val);
	dp->stats.crc    += x = (uint8_t)(val >> 16);
	dp->stats.errrcv += x;
	dp->stats.frame  += x = (uint8_t)(val >> 24);
	dp->stats.errrcv += x;

	OUTW(dp, DIO_ADR, Multicollisions);
	val = INL_DIO(dp, Multicollisions);
	dp->stats.multi_coll += x = (uint16_t)(val);
	dp->stats.collisions += x*2;
	dp->stats.first_coll += x = (uint16_t)(val >> 16);
	dp->stats.collisions += x;

	OUTW(dp, DIO_ADR, ExcessiveCollisions);
	val = INL_DIO(dp, ExcessiveCollisions);
	dp->stats.excoll += x = (uint8_t)(val);
	dp->stats.errxmt += x;
	dp->stats.collisions += x*16;
	dp->stats.xmtlatecoll += x = (uint8_t)(val >> 8);
	dp->stats.errxmt += x;
	dp->stats.nocarrier += x = (uint8_t)(val >> 16);
	dp->stats.errxmt += x;

	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static void
tne_load_tx_list(struct gem_dev *dp)
{
	struct tne_dev	*lp = dp->private;

	ASSERT(!lp->tx_list_loaded);

	OUTL(dp, CH_PARM,
	    dp->tx_ring_dma + sizeof (struct tx_list) * lp->tx_list_head);

	OUTL(dp, HOST_CMD, HC_TxGO(0));

	lp->tx_list_loaded = B_TRUE;
}

#ifdef TXTIMEOUT_TEST
static int	tne_send_cnt;
#endif
static int
tne_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
	struct tx_list		*tlp;
	struct frag		*tfp;
	uint32_t		tlp_dma;
	uint32_t		mark;
	uint32_t		addr;
	ddi_dma_cookie_t	*dcp;
	uint_t			len;
	struct tne_dev		*lp = dp->private;
	const uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

	tlp = ((struct tx_list *)dp->tx_ring) + slot;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "%s: %s seqnum: %d, slot %d, frags: %d flag: %lld",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "%d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	tlp->forward_ptr = dp->tx_ring_dma +
	    (sizeof (struct tx_list)) * SLOT(slot + 1, tx_ring_size);

	/* copy fragment list */
	i = frags - 1;
	len = 0;
	tfp = &tlp->frags[i];
	dcp = &dmacookie[i];
	mark = DATACOUNT_LAST;
	for (; i-- >= 0; tfp--, dcp--) {
		len += dcp->dmac_size;
		mark |= dcp->dmac_size;
		tfp->DataCount = LE_32(mark);
		addr = dcp->dmac_address;
		tfp->DataAddress = LE_32(addr);
		mark = 0;
	}
	addr = (len << TC_FRAMESIZE_SHIFT) | TC_MBO;
	tlp->tx_cstat = LE_32(addr);

#ifdef TXTIMEOUT_TEST
	tne_send_cnt++;
	if (tne_send_cnt > 100) {
		tne_send_cnt = 0;
		return (1);
	}
#endif
	return (1);
}

static void
tne_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int			slot;
	uint32_t		addr;
	struct tx_list		*tlp;
	struct tx_list		*tlpp;
	struct tne_dev		*lp = dp->private;
	const uint_t		tx_ring_size = dp->gc.gc_tx_ring_size;

	/* terminate the new tx list */
	slot = SLOT(start_slot + nslot - 1, tx_ring_size);
	((struct tx_list *)dp->tx_ring)[slot].forward_ptr = 0;

	/* flush tx descriptors we made */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	if (lp->tx_list_len > 0) {
		/* link the tx list to the previous descriptor */
		slot = SLOT(start_slot - 1, tx_ring_size);
		addr = dp->tx_ring_dma + start_slot * sizeof (struct tx_list);
		((struct tx_list *)dp->tx_ring)[slot].forward_ptr = LE_32(addr);

		/* flush the previous descriptor */
		gem_tx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);
	}
	lp->tx_list_len += nslot;

	if (!lp->tx_list_loaded && IS_MAC_ONLINE(dp)) {
		/* Restart tx list */
		tne_load_tx_list(dp);
	}
}

static void
tne_load_rx_list(struct gem_dev *dp)
{
	struct tne_dev	*lp = dp->private;

	ASSERT(!lp->rx_list_loaded);
	ASSERT(lp->rx_list_len > 0);
	OUTL(dp, CH_PARM,
	    dp->rx_ring_dma + sizeof (struct rx_list) * lp->rx_list_head);

	OUTL(dp, HOST_CMD, HC_RxGO(0));

	lp->rx_list_loaded = B_TRUE;
}

static void
tne_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	uint32_t		addr;
	uint32_t		len;
	struct rx_list		*rlp;
	struct frag		*rfp;
	ddi_dma_cookie_t	*dcp;
	struct tne_dev		*lp = dp->private;
	const uint_t		rx_ring_size = dp->gc.gc_rx_ring_size;

	rlp = &((struct rx_list *)dp->rx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
#if 0
	rlp->forward_ptr = 0; /* means this is the last descriptor in rx list */
#else
	rlp->forward_ptr = dp->rx_ring_dma +
	    SLOT(slot + 1, rx_ring_size) * sizeof (struct rx_list);
#endif
	rlp->rx_cstat = LE_32(RC_MBO);

	/* copy fragment list */
	rfp = rlp->frags;
	dcp = dmacookie;
	for (i = frags; i--; rfp++, dcp++) {
		len = (uint32_t)dcp->dmac_size;
		rfp->DataCount = LE_32(len);
		addr = (uint32_t)dcp->dmac_address;
		rfp->DataAddress = LE_32(addr);
	}

	/*
	 * Add a zero length fragment to terminate
	 * the rx fragment list
	 */
	rfp->DataCount = 0;
}

static void
tne_rx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	int			i;
	int			slot;
	uint32_t		addr;
	const uint_t		rx_ring_size = dp->gc.gc_rx_ring_size;
	struct rx_list		*rlp;
	struct rx_list		*rlpp;
	struct tne_dev		*lp = dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s: start_slot %d, nslots: %d",
	    dp->name, __func__, start_slot, nslots);
#endif
	i = start_slot + nslots - 1;
	slot = SLOT(i, rx_ring_size);
	rlp = &((struct rx_list *)dp->rx_ring)[slot];
#if 0
	while (i-- != start_slot) {
		/* append this descriptor at the end of current rx list */
		slot = SLOT(i, rx_ring_size);
		rlpp = &((struct rx_list *)dp->rx_ring)[slot];
		addr = dp->rx_ring_dma + (((caddr_t)rlp) - dp->rx_ring);
		rlpp->forward_ptr = LE_32(addr);
		rlp = rlpp;
	}
#else
	/* terminate rx list */
	rlp->forward_ptr = 0;
#endif
	/* flush rx descriptors we made */
	gem_rx_desc_dma_sync(dp, start_slot, nslots, DDI_DMA_SYNC_FORDEV);

	if (lp->rx_list_len > 0)  {
		/* Link it to the previous slot */
		slot = SLOT(start_slot - 1, rx_ring_size);
		addr = dp->rx_ring_dma + start_slot * sizeof (struct rx_list);
		((struct rx_list *)dp->rx_ring)[slot].forward_ptr = LE_32(addr);

		gem_rx_desc_dma_sync(dp, slot, 1, DDI_DMA_SYNC_FORDEV);
	}

	lp->rx_list_len += nslots;

	if (!lp->rx_list_loaded && IS_MAC_ONLINE(dp)) {
		/* Restart rx list */
		tne_load_rx_list(dp);
	}
}

static uint_t
tne_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_list		*tlp;
	uint16_t		cstat;
	struct tne_dev		*lp = dp->private;

	tlp = &((struct tx_list *)dp->tx_ring)[slot];
	cstat = tlp->tx_cstat;
	cstat = LE_32(cstat);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, tx_cstat:0x%b",
	    dp->name, __func__, slot, tlp->tx_cstat, TC_BITS));

	if ((cstat & TC_FrmCmp) == 0) {
		/* not transmitted */
		return (0);
	}

	ASSERT(lp->tx_list_len > 0);
	lp->tx_list_len--;
	lp->tx_list_done++;
	lp->tx_list_head = SLOT(slot + 1, dp->gc.gc_tx_ring_size);

	return (GEM_TX_DONE);
}

#ifdef DEBUG_LEVEL
static void
tne_dump_packet(struct gem_dev *dp, uint8_t *bp, int n)
{
	int	i;

	for (i = 0; i < n; i += 8, bp += 8) {
		cmn_err(CE_CONT, "%02x %02x %02x %02x %02x %02x %02x %02x",
		    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7]);
	}
}
#endif
static uint64_t
tne_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_list		*rlp;
	uint_t			flag;
	uint_t			len;
	uint32_t		cstat;
	struct tne_dev		*lp = dp->private;

	rlp = &((struct rx_list *)dp->rx_ring)[slot];
	cstat = rlp->rx_cstat;
	cstat = LE_32(cstat);

	if ((cstat & RC_FrmCmp) == 0) {
		/* not received */
		return (0);
	}

	len = (cstat >> RC_FRAMESIZE_SHIFT) & RC_FRAMESIZE_MASK;
	flag = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, rx_cstat:0x%b",
	    dp->name, __func__, slot, cstat, RC_BITS));

	ASSERT(lp->rx_list_len > 0);
	lp->rx_list_len--;
	lp->rx_list_done++;
	lp->rx_list_head = SLOT(slot + 1, dp->gc.gc_rx_ring_size);

	if (cstat & RC_RxError) {
		DPRINTF(0, (CE_WARN,
		    "!%s: %s: slot:%d, rx_cstat:0x%b",
		    dp->name, __func__, slot, cstat, RC_BITS));
		flag |= GEM_RX_ERR;
	}

#if DEBUG_LEVEL > 3
	tne_dump_packet(dp, dp->rx_buf_head->rxb_buf, len);
#endif
	return (flag | len);
}

static void
tne_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

static void
tne_rx_desc_init(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

/*
 * Device depend interrupt handler
 */
static void
tne_adapter_check(struct gem_dev *dp)
{
	uint32_t	check_code;
	uint_t		channel;
	uint8_t		failure_code;
	static char	*failure_name[] = {
		"",
		"Data parity error",
		"Address parity error",
		"Master abort",
		"Target abort",
		"List error",
		"Acknowledge error",
		"Int overflow error",
	};

	check_code = INL(dp, CH_PARM);
	channel = (check_code & AdC_Channel) >> AdC_Channel_SHIFT;
	failure_code = check_code & AdC_FailureCode;

	cmn_err(CE_WARN, "%s: adaptor failure happened. "
	    "failure_code:%x (%s) %s %s %s",
	    dp->name,
	    failure_code,
	    (failure_code >= 1 && failure_code <= 7)
	    ? failure_name[failure_code] : "unknwon error",
	    (check_code & AdC_List) ? "List/EOC" : "Data/EOF",
	    (check_code & AdC_Rx) ? "Rx" : "Tx",
	    (check_code & AdC_Rd) ? "Read" : "Write");

	(void) tne_get_stats(dp);
}

#ifdef DEBUG_LEVEL
static char *tne_inttype_names[] = {
	"NoIntr",
	"TxEOF",
	"StatisticsOvf",
	"RxEOF",
	"Dummy",
	"TxEOC",
	"AdNet",
	"RxEOC",
};
#endif
static uint_t
tne_interrupt(struct gem_dev *dp)
{
	uint16_t	host_int;
	uint8_t		int_type;
	uint16_t	int_vec;
	boolean_t	tx_sched = B_FALSE;
	int		loop;
	boolean_t	need_to_reset = B_FALSE;
	uint16_t	netsts;
	struct tne_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	host_int = INW(dp, HOST_INT);
	int_type = (host_int & HI_IntType) >> HI_IntType_SHIFT;
	int_vec = (host_int & HI_IntVec) >> HI_IntVec_SHIFT;

	if (int_type == IntType_NoIntr) {
		/* not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	/* Disable all interrupts */
	OUTL(dp, HOST_CMD, HC_IntsOff);
#ifdef MAP_MEMx
	FLSHL(dp, HOST_CMD);
#endif

	if (!IS_MAC_ONLINE(dp)) {
		/* Ack */
		OUTW(dp, HOST_INT, host_int);
		return (DDI_INTR_CLAIMED);
	}

#ifdef CONFIG_POLLING
	if (dp->speed == GEM_SPD_100 &&
	    lp->last_poll_interval != dp->poll_interval) {
		/*
		 * update interrupt pacing timer
		 * we actually poll four times in expected polling interval.
		 */
		OUTL(dp, HOST_CMD,
		    HC_LdTmr | min(dp->poll_interval/6000, 255));
		lp->last_poll_interval = dp->poll_interval;
	}
#endif /* CONFIG_POLLING */

	/*
	 * Read interrupt status
	 */
	loop = 100;
	do {
		/* Ack */
		OUTW(dp, HOST_INT, host_int);

		DPRINTF(3, (CE_CONT, "!%s: %s: type:%s(%d) vec:0x%x",
		    dp->name, __func__,
		    tne_inttype_names[int_type], int_type, int_vec));

		switch (int_type) {
#ifdef DEBUG_LEVEL
		case IntType_NoIntr:
			/* Never happen */
			cmn_err(CE_PANIC, "%s: NoInter happened", dp->name);
			/* NOT REACHED */
			break;
#endif
		case IntType_TxEOF:
#ifdef GEM3
			gem_reclaim_txbuf(dp);
#endif
			if (gem_tx_done(dp)) {
				tx_sched = B_TRUE;
			}

			/* ack the number of transmitted frames */
			mutex_enter(&dp->xmitlock);
			OUTL(dp, HOST_CMD,
			    HC_Ack | (host_int << 16) | lp->tx_list_done);
			lp->tx_list_done = 0;
			mutex_exit(&dp->xmitlock);
			break;

		case IntType_StatisticsOvf:
			/*
			 * Statistics counter overflow
			 */
			(void) tne_get_stats(dp);

			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_RxEOF:
			/*
			 * packet was received, or receive error happened
			 */
			(void) gem_receive(dp);

			/* notify the number of received frames to nic */
			OUTL(dp, HOST_CMD,
			    HC_Ack | (host_int << 16) | lp->rx_list_done);
			lp->rx_list_done = 0;
			break;

		case IntType_Dummy:
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_TxEOC:
#ifndef CONFIG_POLLING
			if (gem_tx_done(dp)) {
				tx_sched = B_TRUE;
			}
#endif
			/* ack and start again */
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));

			mutex_enter(&dp->xmitlock);
			lp->tx_list_loaded = B_FALSE;

			if (lp->tx_list_len > 0) {
				/* start tx again */
				tne_load_tx_list(dp);
			}
			mutex_exit(&dp->xmitlock);
			break;

		case IntType_AdNet:
			if (int_vec) {
				tne_adapter_check(dp);
				need_to_reset = B_TRUE;
				goto x;
			}
			/* never happen */

			/* clear NetSTS register */
			netsts = INW_DIO(dp, NETSTS);
			OUTW_DIO(dp, NETSTS, netsts);
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_RxEOC:
			/*  try to allocate Rx buffers */
			/* ack and start again */
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			lp->rx_list_loaded = B_FALSE;
			dp->stats.norcvbuf++;

			if (lp->rx_list_len > 0) {
				/* start rx again */
				tne_load_rx_list(dp);
			} else {
				need_to_reset = B_TRUE;
				goto x;
			}
			break;
		}

		host_int = INW(dp, HOST_INT);
		int_type = (host_int & HI_IntType) >> HI_IntType_SHIFT;
		int_vec = (host_int & HI_IntVec) >> HI_IntVec_SHIFT;

	} while (int_type != IntType_NoIntr && --loop > 0);

x:
	if (loop <= 0) {
		need_to_reset = B_TRUE;
		cmn_err(CE_WARN,
		    "%s: interrupts maxmum exceeded, "
		    "type:%s(%d) vec:0x%x",
		    dp->name,
		    tne_inttype_names[int_type], int_type, int_vec);
	}

	if (need_to_reset) {
		/* Disable all interrupts */
		OUTL(dp, HOST_CMD, HC_IntsOff);

		/* Clear all interrupt */
		OUTW(dp, HOST_INT, INW(dp, HOST_INT));

		gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		tx_sched = B_TRUE;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0 && loop > 0) {
		/* Enable all interrupts again */
		OUTL(dp, HOST_CMD, HC_IntsOn);
	}
#if DEBUG_LEVEL > 2
	if (tx_sched) {
		cmn_err(CE_CONT, "!%s: rescheduling blocked tx", dp->name);
	}
#endif
	/*
	 * Recover Interrput Enable register
	 */
	DPRINTF(4, (CE_CONT, "!%s: %s done", dp->name, __func__));

	return (DDI_INTR_CLAIMED | (tx_sched ? INTR_RESTART_TX : 0));
}

/*
 * HW depend MII routine
 */
#define	MDIO_DELAY(dp)	{ INB_DIO(dp, NETSIO); INB_DIO(dp, NETSIO); }

static void
tne_mii_sync(struct gem_dev *dp)
{
	int		i;
	uint8_t		diodata;

	/* output 32 ones */
	OUTW(dp, DIO_ADR, NETSIO);

	diodata = INB_DIO(dp, NETSIO) | NSIO_MDATA | NSIO_MTXEN;
	diodata &= ~(NSIO_MINTEN | NSIO_MCLK);
	for (i = 0; i < 32; i++) {
		OUTB_DIO(dp, NETSIO, diodata);
		MDIO_DELAY(dp);
		OUTB_DIO(dp, NETSIO, diodata | NSIO_MCLK);
		MDIO_DELAY(dp);
	}
}

static uint16_t
tne_mii_read_raw(struct gem_dev *dp, uint_t phy, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint8_t		data;
	uint8_t		diodata;
	uint32_t	valid_mask = 0xffff;

	cmd = MII_READ_CMD(phy, reg);

	OUTW(dp, DIO_ADR, NETSIO);

	diodata = INB_DIO(dp, NETSIO);
	DPRINTF(4, (CE_CONT,
	    "!%s: %s: diodata: 0x%02x", dp->name, __func__, diodata));

	diodata &= ~(NSIO_MINTEN | NSIO_MDATA | NSIO_MTXEN | NSIO_MCLK);
	diodata |= NSIO_MTXEN;

	for (i = 31; i >= 18; i--) {
		/* notify the phy to send data */
		data = (((cmd >> i) & 1) <<  NSIO_MDATA_SHIFT) | diodata;
		OUTB_DIO(dp, NETSIO, data);
		MDIO_DELAY(dp);

		/* notify to the phy that data is stable */
		OUTB_DIO(dp, NETSIO, data | NSIO_MCLK);
		MDIO_DELAY(dp);
	}
	/* turn araund */
	diodata &= ~NSIO_MTXEN;
	OUTB_DIO(dp, NETSIO, diodata);
	MDIO_DELAY(dp);

	/* notify to the phy to send ack */
	OUTB_DIO(dp, NETSIO, diodata | NSIO_MCLK);
	MDIO_DELAY(dp);

	/* get response from the phy */
	OUTB_DIO(dp, NETSIO, diodata);
	if (INB_DIO(dp, NETSIO) & NSIO_MDATA) {
		DPRINTF(4, (CE_CONT, "!%s: %s: phy@%d didn't respond",
		    dp->name, __func__, phy));
		valid_mask = 0;
	}

	ret = 0;
	for (i = 16; i > 0; i--) {
		/* notify the phy to send data */
		OUTB_DIO(dp, NETSIO, diodata | NSIO_MCLK);
		MDIO_DELAY(dp);

		/* down the clock (actual latch timing) */
		OUTB_DIO(dp, NETSIO, diodata);

		/* read data before raising the clock */
		ret = (ret << 1)
		    | ((INB_DIO(dp, NETSIO) >> NSIO_MDATA_SHIFT) & 1);
	}

	/* revert the clock cycle and send idle bits */
	diodata |= NSIO_MDATA | NSIO_MTXEN;
	for (i = 0; i < 2; i++) {
		OUTB_DIO(dp, NETSIO, diodata);
		MDIO_DELAY(dp);
		OUTB_DIO(dp, NETSIO, diodata | NSIO_MCLK);
		MDIO_DELAY(dp);
	}

	return (ret & valid_mask);
}

static uint16_t
tne_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint16_t	val;
	uint_t		phy;
	struct tne_dev	*lp = dp->private;

	if (DUAL_PHY(lp)) {
		if ((dp->mii_status & MII_STATUS_MFPRMBLSUPR) &&
		    lp->curr_phy == 31) {
			/* internal phy requires sync bits before each access */
			tne_mii_sync(dp);
		}
		phy = lp->curr_phy;
	} else {
		phy = dp->mii_phy_addr;
	}

	val = tne_mii_read_raw(dp, phy, reg);

	if (DUAL_PHY(lp) && reg == MII_STATUS && (val & MII_STATUS_REMFAULT)) {
		uint16_t	ctrl;

		DPRINTF(0, (CE_CONT,
		    "!%s: %s: switching phy at %d, status:%b",
		    dp->name, __func__,
		    lp->curr_phy, val, MII_ABILITY_BITS));

		/* switch phy */
		tne_mii_sync(dp);
		tne_mii_write_raw(dp, lp->curr_phy,
		    MII_CONTROL,
		    MII_CONTROL_ISOLATE);
		if (lp->curr_phy == dp->mii_phy_addr) {
			lp->curr_phy = 31;
		} else {
			lp->curr_phy = dp->mii_phy_addr;
		}

		/* internal phy requires sync bits before each access */
		tne_mii_sync(dp);

		/* initialize control register */
		tne_mii_write_raw(dp, lp->curr_phy, MII_CONTROL, 0);
		dp->mii_state = MII_STATE_UNKNOWN;
	}

	if (reg == MII_AN_ADVERT) {
		val |= 1;
	}
	return (val);
}

static void
tne_mii_write_raw(struct gem_dev *dp, uint_t phy, uint_t reg, uint16_t val)
{
	uint32_t	cmd;
	int		i;
	uint8_t		data;
	uint8_t		diodata;

	DPRINTF(4, (CE_CONT,
	    "!%s: %s: time:%d %s: reg:%d, val: 0x%04x",
	    dp->name, __func__, ddi_get_lbolt(), __func__, reg, val));

	cmd = MII_WRITE_CMD(phy, reg, val);

	OUTW(dp, DIO_ADR, NETSIO);
	diodata = INB_DIO(dp, NETSIO);
	diodata &= ~(NSIO_MINTEN | NSIO_MDATA | NSIO_MTXEN | NSIO_MCLK);
	diodata |= NSIO_MTXEN;

	for (i = 31; i >= 0; i--) {
		/* output new data */
		data = (((cmd >> i) & 1) << NSIO_MDATA_SHIFT) | diodata;
		OUTB_DIO(dp, NETSIO, data);
		MDIO_DELAY(dp);

		/* notify to the phy that data is stable */
		OUTB_DIO(dp, NETSIO, data | NSIO_MCLK);
		MDIO_DELAY(dp);
	}

	/*
	 * Send 2 idle clock cycle to ensure that the transmittion
	 * is terminated.
	 */
	diodata |= NSIO_MDATA | NSIO_MTXEN;
	for (i = 0; i < 2; i++) {
		OUTB_DIO(dp, NETSIO, diodata);
		MDIO_DELAY(dp);
		OUTB_DIO(dp, NETSIO, diodata | NSIO_MCLK);
		MDIO_DELAY(dp);
	}
}

static void
tne_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint_t		phy;
	struct tne_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: time:%d %s: reg:%d, val: 0x%04x",
	    dp->name, __func__, ddi_get_lbolt(), __func__, reg, val));

	if (DUAL_PHY(lp)) {
		if ((dp->mii_status & MII_STATUS_MFPRMBLSUPR) &&
		    lp->curr_phy == 31) {
			/* internal phy requires sync bits for each access */
			tne_mii_sync(dp);
		}
		phy = lp->curr_phy;
	} else {
		phy = dp->mii_phy_addr;
	}

	tne_mii_write_raw(dp, phy, reg, val);
}

#undef MDIO_DELAY

static int
tne_mii_probe(struct gem_dev *dp)
{
	int	ret;
	struct tne_dev	*lp = dp->private;

	/* Setup the NetConfig register for external phy and internal phy */
	lp->is_dual_phy = B_FALSE;
	OUTW(dp, DIO_ADR, NETCFG);
	OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En);

	/* First, we try external PHY */
	ret = gem_mii_probe_default(dp);
	if (ret == GEM_SUCCESS) {
		DPRINTF(0, (CE_CONT, "!%s: %s: external phy selected",
		    dp->name, __func__));

		if (dp->mii_phy_id == 0 || dp->mii_phy_id == 0xffffffffU) {
			uint16_t	status;

			/* dual phy mode */
			lp->is_dual_phy = B_TRUE;
			lp->curr_phy = (uint_t)dp->mii_phy_addr;

			/* enable internal PHY */
			tne_mii_sync(dp);
			status = tne_mii_read_raw(dp, 31, MII_STATUS);
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: reading internal phy: status:%b",
			    dp->name, __func__, status, MII_STATUS_BITS));
			tne_mii_sync(dp);
			tne_mii_write_raw(dp, 31,
			    MII_CONTROL,
			    MII_CONTROL_ISOLATE);
		} else {
			/* disable internal PHY */
			OUTW(dp, DIO_ADR, NETCFG);
			OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn);
		}
		return (ret);
	}

	/* Next, we try internal PHY */
	/* Setup NetConfig register for the internal 10Mbps phy */
	dp->mii_phy_addr = 31;
	OUTW(dp, DIO_ADR, NETCFG);
	OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En);

	DPRINTF(0, (CE_CONT, "!%s: %s: internal phy selected",
	    dp->name, __func__));

	return (gem_mii_probe_default(dp));
}

static int
tne_mii_init(struct gem_dev *dp)
{
	struct tne_dev	*lp = dp->private;

	if (!DUAL_PHY(lp) && dp->mii_phy_addr != 31) {
		/* Setup NetConfig register only for external phy */
		OUTW(dp, DIO_ADR, NETCFG);
		OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn);
	} else {
		/* Setup NetConfig register for the internal 10Mbps phy */
		OUTW(dp, DIO_ADR, NETCFG);
		OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En);
		if (DUAL_PHY(lp)) {
			/* isolate and power down unused phy */
			tne_mii_sync(dp);
			tne_mii_write_raw(dp,
			    lp->curr_phy != 31 ? 31 : dp->mii_phy_addr,
			    MII_CONTROL,
			    MII_CONTROL_ISOLATE);
		}
	}
	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
tneattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			n;
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	int			ret;
	int			vid;
	int			did;
	int			unit;
	struct chip_info	*p;
	int			val;
	uint_t			len;
	struct pci_phys_spec	*regs;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct tne_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint8_t			revid;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "%s%d: pci_config_setup failed",
		    drv_name, unit);
		goto err;
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 0x20);

	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		for (i = 0, p = tne_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (p->venid == vid && p->devid == did) {
				/* found */
				cmn_err(CE_CONT,
			"!%s%d: %s (vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
				    drv_name, unit, p->name, vid, did, revid);
				goto chip_found;
			}
		}

		/* Not found */
		cmn_err(CE_WARN,
		    "!%s%d: %s: wrong PCI venid/devid (0x%x, 0x%x)",
		    drv_name, unit, __func__, vid, did);
		goto err;
chip_found:
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &tne_dev_attr, (void *)&base, &regs_ha)
		    != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = min(GEM_MAXTXFRAGS, TNE_MAXTXFRAGS);
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift = 6;	/* 64 byte */
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = tne_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
#ifdef RXSINGLE
		gcp->gc_rx_max_frags = 1;
#else
		gcp->gc_rx_max_frags = min(GEM_MAXRXFRAGS, TNE_MAXRXFRAGS - 1);
#endif
		gcp->gc_rx_desc_unit_shift = 6;	/* 64 byte */
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE - 1;
		gcp->gc_rx_copy_thresh = tne_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes (endianness) */
		gcp->gc_dev_attr = tne_dev_attr;
		gcp->gc_buf_attr = tne_buf_attr;
		gcp->gc_desc_attr = tne_buf_attr;

		/* dma attributes (boundary) */
		gcp->gc_dma_attr_desc = tne_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = tne_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = tne_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &tne_attach_chip;
		gcp->gc_reset_chip = &tne_reset_chip;
		gcp->gc_init_chip = &tne_init_chip;
		gcp->gc_start_chip = &tne_start_chip;
		gcp->gc_stop_chip = &tne_stop_chip;
		gcp->gc_multicast_hash = &tne_mcast_hash;
		gcp->gc_set_rx_filter = &tne_set_rx_filter;
		gcp->gc_set_media = &tne_set_media;
		gcp->gc_get_stats = &tne_get_stats;
		gcp->gc_interrupt = &tne_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &tne_tx_desc_write;
		gcp->gc_tx_start = &tne_tx_start;
		gcp->gc_rx_desc_write = &tne_rx_desc_write;
		gcp->gc_rx_start = &tne_rx_start;

		gcp->gc_tx_desc_init = &tne_tx_desc_init;
		gcp->gc_rx_desc_init = &tne_rx_desc_init;
		gcp->gc_tx_desc_stat = &tne_tx_desc_stat;
		gcp->gc_rx_desc_stat = &tne_rx_desc_stat;
		gcp->gc_tx_desc_clean = &tne_tx_desc_init;
		gcp->gc_rx_desc_clean = &tne_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &tne_mii_probe;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &tne_mii_sync;
		gcp->gc_mii_read = &tne_mii_read;
		gcp->gc_mii_write = &tne_mii_write;
		gcp->gc_mii_tune_phy = NULL;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_max_mtu = (gcp->gc_rx_max_frags - 1) * 4096 -
		    sizeof(struct ether_header) - 4;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN;
#endif
		lp = kmem_zalloc(sizeof (struct tne_dev), KM_SLEEP);
		lp->chip = p;
		lp->rev_id = revid;
		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}
err_free_mem:
		kmem_free(lp, sizeof (struct tne_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
tnedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

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
GEM_STREAM_OPS(tne_ops, tneattach, tnedetach);
#else
static	struct module_info tneminfo = {
	0,			/* mi_idnum */
	"tne",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit tnerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&tneminfo,		/* qi_minfo */
	NULL,			/* qi_mstat */
};

static	struct qinit tnewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&tneminfo,		/* qi_minfo */
	NULL,			/* qi_mstat */
};

static struct streamtab	tne_info = {
	&tnerinit,	/* st_rdinit */
	&tnewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL,		/* st_muxwrinit */
};

static	struct cb_ops cb_tne_ops = {
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
	&tne_info,	/* cb_stream */
	D_NEW|D_MP,	/* cb_flag */
};

static	struct dev_ops tne_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	tneattach,	/* devo_attach */
	tnedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_tne_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power,	/* devo_power */
};
#endif
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&tne_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!tne: _init: called"));
	gem_mod_init(&tne_ops, "tne");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&tne_ops);
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

	DPRINTF(2, (CE_CONT, "!tne: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&tne_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
