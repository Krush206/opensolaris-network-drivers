/*
 * tne_gem.c: TI ThunderLAN TNETE100A Fast Ethernet MAC driver
 *
 * Copyright (c) 2003-2004 Masayuki Murayama.  All rights reserved.
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

#pragma ident "@(#)tne_gem.c	1.12 05/05/09"

/*
 Change log
 0.8.5  05/10/2004
	clean up tne_interrupt().
        05/29/2004
	disabling interrupts while tne_interrupt()
	supported cards added
 0.8.6  06/19/2004
 */

/*
 TODO:
 mii sync/read/write worked
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
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/ddi_impldefs.h>

#include <sys/pci.h>
#include "mii.h"
#include "gem.h"
#include "tnete100areg.h"

#if GEM_MAXTXFRAGS > TNE_MAXTXFRAGS
# error GEM_MAXTXFRAGS must be less than or equals to 10.
#endif

#if GEM_MAXRXFRAGS >= TNE_MAXRXFRAGS
# error GEM_MAXRXFRAGS must be less than 10.
#endif

char	ident[] = "TI ThunderLAN driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int tne_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (tne_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	FALSE	(0)
#define	TRUE	(!FALSE)

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

#define	INT8(x)		(0xff & (x))
#define	INT16(x)	(0xffff & (x))

#define	OUTB_DIO(dp, reg, val)	OUTB(dp, DIO_DATA | ((reg) & 3), val)
#define	OUTW_DIO(dp, reg, val)	OUTW(dp, DIO_DATA | ((reg) & 3), val)
#define	OUTL_DIO(dp, reg, val)	OUTL(dp, DIO_DATA, val)
#define	INB_DIO(dp, reg)	INB(dp, DIO_DATA | ((reg) & 3))
#define	INW_DIO(dp, reg)	INW(dp, DIO_DATA | ((reg) & 3))
#define	INL_DIO(dp, reg)	INL(dp, DIO_DATA)
#define	FLSHL(dp, reg)		INL(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)

struct tne_dev {
	boolean_t	tx_last_intreq;

	boolean_t	tx_list_loaded;
	uint_t		tx_list_done;
	int		tx_list_len;
	int		tx_list_head;

	boolean_t	rx_list_loaded;
	uint_t		rx_list_done;
	int		rx_list_len;
	int		rx_list_head;

	boolean_t	which_phy;
#define	PHY_EXTERNAL_MII	0
#define	PHY_INTERNAL_MII	1

#ifdef CONFIG_POLLING
	int		last_poll_interval;
#endif
	uint8_t		netcmd;
	uint8_t		rev_id;
	uint8_t		prom[256];
};

/*
 * Our configuration
 */
#define	TNE_MAX_MTU	(64*1024 - 14 - 1)

#ifdef TEST_TXDESC_FULL
# define TX_RING_SIZE	4
# define TX_BUF_SIZE	64
#endif
#ifdef TEST_RX_EMPTY
# define RX_RING_SIZE	8
#endif

#ifndef TX_RING_SIZE
# define TX_RING_SIZE	64
#endif
#ifndef TX_BUF_SIZE
# define TX_BUF_SIZE	TX_RING_SIZE
#endif

#ifndef RX_RING_SIZE
# define RX_RING_SIZE	64
#endif
#define RX_BUF_SIZE	RX_RING_SIZE

#define	ONESEC			(drv_usectohz(1*1000000))

static int	tne_tx_copy_thresh = 256;
static int	tne_rx_copy_thresh = 256;

#ifndef INTR_DELAY
#   define	INTR_DELAY	0
#endif

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
	{0x108d, 0x0013, "Olicom OC-2183/2185"},
	{0x108d, 0x0014, "Olicom OC-2326"},
	{0x104c, 0x0500, "TI ThunderLAN TNETE100A"},
};
#define CHIPTABLESIZE   (sizeof(tne_chiptbl)/sizeof(struct chip_info))

/* ======================================================== */
 
/* mii operations */
static void  tne_mii_sync(struct gem_dev *);
static uint16_t  tne_mii_read(struct gem_dev *, uint_t);
static void tne_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int tne_attach_chip(struct gem_dev *);
static int tne_reset_chip(struct gem_dev *);
static void tne_init_chip(struct gem_dev *);
static void tne_start_chip(struct gem_dev *);
static void tne_stop_chip(struct gem_dev *);
static void tne_set_media(struct gem_dev *);
static void tne_set_rx_filter(struct gem_dev *);
static void tne_get_stats(struct gem_dev *);

/* descriptor operations */
static int tne_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag);
static int tne_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t tne_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint_t tne_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void tne_tx_desc_init(struct gem_dev *dp, int slot);
static void tne_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int tne_interrupt(struct gem_dev *dp);

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
	0xffffffffull,		/* dma_attr_count_max */
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
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

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

	DPRINTF(2, (CE_CONT, "!%s: tne_reset_chip", dp->name));

	return 0;
}

static void
tne_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: tne_init_chip: called", dp->name));

	/* Disable interrupts by asserting the Ints_off bit int HOST_CMD */
	OUTL(dp, HOST_CMD, HC_IntsOff);

	/* Setup the NetConfig registers for the appropriate options */
	ASSERT((lp->netcmd & NCMD_NRESET) == 0);
	OUTW(dp, DIO_ADR, NETCFG);
	if (lp->which_phy == PHY_INTERNAL_MII) {
		OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En);
	} else {
		OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn);
	}

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

	/* Load the appropriate interrupt pacing timer in Ld_Tmr in HOST_CMD */
	OUTL(dp, HOST_CMD, HC_LdTmr | 0);

	/* Load the appropriate Tx threshold value in Ld_Thr in HOST_CMD */
	OUTL(dp, HOST_CMD, HC_LdThr | 0);
#if 0
	/* Unreset the MII by asserting NMRDT in NetSio */

	/* Initialize PHY layer */
#endif
	/* Setup the network status mask register */
	OUTW(dp, DIO_ADR, NETMASK);
	OUTB_DIO(dp, NETMASK, 0);

	OUTW(dp, DIO_ADR, MaxRx);
	val = min(0xffff, dp->mtu + sizeof(struct ether_header));
	OUTW_DIO(dp, MaxRx, (uint16_t)val);

	/* enable NETCMD register */
	lp->netcmd |= NCMD_NRESET | NCMD_NWRAP;
	OUTW(dp, DIO_ADR, NETCMD);
	OUTB_DIO(dp, NETCMD, lp->netcmd);

	lp->tx_list_loaded = FALSE;
	lp->tx_list_len    = 0;
	lp->tx_list_done   = 0;
	lp->tx_list_head   = 0;

	lp->rx_list_loaded = FALSE;
	lp->rx_list_len    = 0;
	lp->rx_list_done   = 0;
	lp->rx_list_head   = 0;
}

static uint32_t
tne_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	int		i;
	uint8_t		hash;
	uint32_t	addr0, addr1;

	addr0 = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	addr1 = (addr[5] << 8) | addr[4];
	for (hash = 0, i = 0; i < 8; i++) {
		hash ^= addr0;
		/* shift 6 bits right */
		addr0 = (addr1 << (32-6)) | (addr0 >> 6);
		addr1 >>= 6;
	}

	return hash & ((1<<6)-1);
}

static void
tne_set_rx_filter(struct gem_dev *dp)	
{
	int		i, j;
	uint8_t		mode;
	uint8_t		old;
	uint8_t		reg;
	uint32_t	hash[2];
	uint8_t		*m;
	uint16_t	addr;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));
	bzero(hash, sizeof(hash));

	mode = 0;
	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		/* promiscous */
		mode = NCMD_CAF;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 && dp->mc_count > 32) {
		/* allmulti */
		hash[0] = 0xffffffff;
		hash[1] = 0xffffffff;
	}
	else {
		/* Normal mode */
		/* set CAM-based multicast filter */
		i = 0;
		addr = Areg_0;
		if (dp->mc_count <= 3) {
			for (; i < dp->mc_count; i++) {
				m = dp->mc_list[i].addr.ether_addr_octet;
				for (j = 0; j < ETHERADDRL; j++, addr++) {
					OUTW(dp, DIO_ADR, addr);
					OUTB_DIO(dp, addr, m[j]);
				}
			}
		}
		else {
			/* Make multicast hash table */
			for (j = 0; j < dp->mc_count; j++) {
				uint_t	k;
				k = dp->mc_list[j].hash;
				hash[k / 32] |= 1 << (k % 32);
			}
			OUTW(dp, DIO_ADR, HASH1);
			OUTL_DIO(dp, HASH1, hash[0]);
			OUTW(dp, DIO_ADR, HASH2);
			OUTL_DIO(dp, HASH2, hash[1]);
		}

		/* Set my addresss */
		for (; i < 4; i++) {
			m = dp->cur_addr.ether_addr_octet;
			for (j = 0; j < ETHERADDRL; j++, addr++) {
				OUTW(dp, DIO_ADR, addr);
				OUTB_DIO(dp, addr, m[j]);
			}
		}
	}

	old = lp->netcmd;
	lp->netcmd = (lp->netcmd & ~NCMD_CAF) | mode;
	if (old != lp->netcmd) {
		OUTW(dp, DIO_ADR, NETCMD);
		OUTB_DIO(dp, NETCMD, lp->netcmd);
	}
}

static void
tne_set_media(struct gem_dev *dp)
{
	uint_t		old;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

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
}

static void
tne_start_chip(struct gem_dev *dp)
{
	uint32_t	val;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	ASSERT(mutex_owned(&dp->intrlock));

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* Enable interrupts by asserting the IntsOn bit in HOST_CMD */
		OUTL(dp, HOST_CMD, HC_IntsOn);
	}

	tne_load_rx_list(dp);
}

static void
tne_stop_chip(struct gem_dev *dp)
{
	uint8_t		val;
	uint16_t	stat;

	OUTL(dp, HOST_CMD, HC_IntsOff);

	OUTL(dp, HOST_CMD, HC_TxSTOP(0));
	OUTL(dp, HOST_CMD, HC_RxSTOP(0));

	tne_reset_chip(dp);
}

/*
 * IIC EEPROM I/O routines
 */
#define	NETSIO_SET(dp, bit)	\
	OUTB_DIO(dp, NETSIO, INB_DIO(dp, NETSIO) | (bit))
#define	NETSIO_CLR(dp, bit)	\
	OUTB_DIO(dp, NETSIO, INB_DIO(dp, NETSIO) & ~(bit))

#define	TNE_EEPROM_DELAY(dp)	{INB_DIO(dp, NETSIO); INB_DIO(dp, NETSIO);}

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
	return ack;
}

static void
tne_iic_send(struct gem_dev *dp, uint8_t data)
{
	uint_t	i;

	for (i = 0x80; i != 0; i >>= 1) {
		if ((data & i) != 0) {
			/* send 1 */
			NETSIO_SET(dp, NSIO_EDATA);
		}
		else {
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

	return data;
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
		return 0;
	}

	/* send address on EEPROM to read from */
	NETSIO_SET(dp, NSIO_ETXEN);
	tne_iic_send(dp, offset);

	/* EEPROM should have acke'd addres received */
	if (!tne_iic_ack(dp)) {
		cmn_err(CE_WARN, "eeprom addr ack failed");
		return 0;
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
		return 0;
	}
	/* receive a 8bit-date from EEPROM and send EEPROM nack */
	tmp = tne_iic_recv(dp, FALSE);

	/* send EEPROM stop access sequence */
	tne_iic_stop(dp);

	/* now data is high, clock is high */

	DPRINTF(4, (CE_CONT, "!%s: tne_eeprom_read: ret:%02x", dp->name, tmp));

	return tmp;
}

#ifdef DEBUG_LEVEL
static void
tne_eeprom_dump(struct gem_dev *dp)
{
	int			i;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	for (i = 0; i < 256; i++) {
		lp->prom[i] = tne_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 256; i += 8) {
		cmn_err(CE_CONT,
			"!0x%02x: 0x%02x 0x%02x 0x%02x 0x%02x " 
				 "0x%02x 0x%02x 0x%02x 0x%02x", 
		i, lp->prom[i  ], lp->prom[i+1], lp->prom[i+2], lp->prom[i+3],
		   lp->prom[i+4], lp->prom[i+5], lp->prom[i+6], lp->prom[i+7]);
	}
}
#endif /* DEBUG_LEVEL */

static int
tne_attach_chip(struct gem_dev *dp)	
{
	int			i;
	uint16_t		val;

#if DEBUG_LEVEL >= 0
	OUTW(dp, DIO_ADR, DEFAULT_REV);
	val = INB_DIO(dp, DEFAULT_REV);
	cmn_err(CE_CONT, "!%s: rev_id: 0x%02x", dp->name, val);
#endif
	/* get factory mac address */
	for (i = 0; i < ETHERADDRL; i++) {
		dp->dev_addr.ether_addr_octet[i] = tne_read_eeprom(dp, 0x83+i);
	}
#if DEBUG_LEVEL > 4
	tne_eeprom_dump(dp);
#endif
	/* Clear the statistics by reading the statistic registers */
	tne_get_stats(dp);
	bzero(&dp->stats, sizeof(dp->stats));

	/* fix mtu */
	dp->mtu = min(dp->mtu, 64*1024 - 14);

	return 0;
}

static void
tne_get_stats(struct gem_dev *dp)
{
	uint_t	x;
	uint_t	val;

	DPRINTF(4, (CE_CONT, "!%s: tne_get_stats: called", dp->name));

	OUTW(dp, DIO_ADR, Tx_UN_GOOD);
	val = INL_DIO(dp, Tx_UN_GOOD);
	dp->stats.underflow += x = INT8(val >> 24);
	dp->stats.errxmt += x;

	OUTW(dp, DIO_ADR, Rx_OV_GOOD);
	val = INL_DIO(dp, Rx_OV_GOOD);
	dp->stats.overflow += x = INT8(val >> 24);
	dp->stats.errrcv += x;

	OUTW(dp, DIO_ADR, DeferredTx);
	val = INL_DIO(dp, DeferredTx);
	dp->stats.defer  += INT16(val);
	dp->stats.crc    += x = INT8(val >> 16);
	dp->stats.errrcv += x;
	dp->stats.frame  += x = INT8(val >> 24);
	dp->stats.errrcv += x;

	OUTW(dp, DIO_ADR, Multicollisions);
	val = INL_DIO(dp, Multicollisions);
	dp->stats.multi_coll += x = INT16(val);
	dp->stats.collisions += x*2;
	dp->stats.first_coll += x = INT16(val >> 16);
	dp->stats.collisions += x;

	OUTW(dp, DIO_ADR, ExcessiveCollisions);
	val = INL_DIO(dp, ExcessiveCollisions);
	dp->stats.excoll += x = INT8(val);
	dp->stats.errxmt += x;
	dp->stats.collisions += x*16;
	dp->stats.xmtlatecoll += x = INT8(val >> 8);
	dp->stats.errxmt += x;
	dp->stats.nocarrier += x = INT8(val >> 16);
	dp->stats.errxmt += x;
}

/*
 * discriptor  manupiration
 */
static void
tne_load_tx_list(struct gem_dev *dp)
{
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	ASSERT(!lp->tx_list_loaded);

	OUTL(dp, CH_PARM,
		dp->tx_ring_dma + sizeof(struct tx_list) * lp->tx_list_head);

	OUTL(dp, HOST_CMD, HC_TxGO(0));

	lp->tx_list_loaded = TRUE;
}

#ifdef TXTIMEOUT_TEST
static int	tne_send_cnt;
#endif
static int
tne_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag)
{
	int			i;
	struct tx_list		*tlp;
	struct frag		*tfp;
	uint32_t		tlp_dma;
	uint32_t		mark;
	ddi_dma_cookie_t	*dcp;
	uint_t			len;
	struct tne_dev		*lp = (struct tne_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tlp = ((struct tx_list *)dp->tx_ring) + slot;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"%s: tne_tx_desc_write "
		"seqnum: %d, slot %d, frags: %d flag: %d",
		dp->name, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "%d: addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
	ASSERT(frags <= TNE_MAXTXFRAGS);
#endif
	ddi_put32(h, &tlp->forward_ptr, 0);
					/* means this is the last descriptor */

	/* copy fragment list */
	i   = frags - 1;
	len = 0;
	tfp = &tlp->frags[i];
	dcp = &dmacookie[i];
	mark= DATACOUNT_LAST;
	for (; i-- >= 0; tfp--, dcp--) {
		len += dcp->dmac_size;
		ddi_put32(h, &tfp->DataCount, mark | dcp->dmac_size);
		ddi_put32(h, &tfp->DataAddress, dcp->dmac_address);
		mark = 0;
	}
	ddi_put32(h, &tlp->tx_cstat, (len << TC_FRAMESIZE_SHIFT) | TC_MBO);

#ifdef notdef
	if (frags < TNE_MAXTXFRAGS) {
		ddi_put32(h, &tlp->frags[frags].DataCount, 0);
	}
#endif
	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tlp) - dp->rx_ring),
		sizeof(struct tx_list), DDI_DMA_SYNC_FORDEV);

#ifdef TXTIMEOUT_TEST
	tne_send_cnt++;
	if (tne_send_cnt > 100) {
		tne_send_cnt = 0;
		return 1;
	}
#endif
	if (lp->tx_list_len > 0 && !lp->tx_last_intreq) {
		struct tx_list	*tlpp;
		/* append this descriptor at the end of current tx list */
		tlpp = ((struct tx_list *) dp->tx_ring) +
					SLOT(slot-1, TX_RING_SIZE);
		ddi_put32(h, &tlpp->forward_ptr,
			dp->tx_ring_dma + (((caddr_t)tlp) - dp->tx_ring));

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tlpp) - dp->rx_ring),
			sizeof(struct tx_list), DDI_DMA_SYNC_FORDEV);
	}
	lp->tx_list_len++;

	if (!lp->tx_list_loaded && dp->nic_active) {
		/* Restart tx list */
		tne_load_tx_list(dp);
	}
	lp->tx_last_intreq = flag;

	return 1;
}

static void
tne_load_rx_list(struct gem_dev *dp)
{
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	ASSERT(!lp->rx_list_loaded);
	ASSERT(lp->rx_list_len > 0);
	OUTL(dp, CH_PARM,
		dp->rx_ring_dma + sizeof(struct rx_list) * lp->rx_list_head);

	OUTL(dp, HOST_CMD, HC_RxGO(0));

	lp->rx_list_loaded = TRUE;
}

static int
tne_rx_desc_write(struct gem_dev *dp, uint_t slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int			i;
	struct rx_list		*rlp;
	struct frag		*rfp;
	ddi_dma_cookie_t	*dcp;
	struct tne_dev		*lp = (struct tne_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rlp = &((struct rx_list *)dp->rx_ring)[slot];

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"%s: tne_rx_desc_write seqnum: %d, slot %d, frags: %d",
		dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "  frag: %d addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	ASSERT(frags < TNE_MAXRXFRAGS);
	ddi_put32(h, &rlp->forward_ptr, 0);
			/* means this is the last descriptor in rx list */
	ddi_put32(h, &rlp->rx_cstat, RC_MBO);

	/* copy fragment list */
	rfp = rlp->frags;
	dcp = dmacookie;
	for (i = frags; i--; rfp++, dcp++) {
		ddi_put32(h, &rfp->DataCount, (uint32_t)dcp->dmac_size);
		ddi_put32(h, &rfp->DataAddress, (uint32_t)dcp->dmac_address);
	}
	/* Add a zero length fragment to terminate the rx fragment list */
	ddi_put32(h, &rfp->DataCount, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rlp) - dp->rx_ring),
		sizeof(struct rx_list), DDI_DMA_SYNC_FORDEV);

	if (lp->rx_list_len++ > 0)  {
		struct rx_list		*rlpp;

		/* Link it to the previous slot */
		rlpp = &((struct rx_list *)
				dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)];
		ddi_put32(h, &rlpp->forward_ptr,
			dp->rx_ring_dma+((caddr_t)rlp-(caddr_t)dp->rx_ring));

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)rlp) - dp->rx_ring),
			sizeof(struct rx_list), DDI_DMA_SYNC_FORDEV);
	}

	return 1;
}

static uint_t
tne_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_list		*tlp;
	uint16_t		cstat;
	struct tne_dev		*lp = (struct tne_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tlp = &((struct tx_list *)dp->tx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tlp) - dp->rx_ring),
		sizeof(struct tx_list), DDI_DMA_SYNC_FORKERNEL);

	cstat = ddi_get32(h, &tlp->tx_cstat);

	DPRINTF(2, (CE_CONT,
		"!%s: tne_tx_desc_stat: slot:%d, tx_cstat:0x%b",
		dp->name, slot, tlp->tx_cstat, TC_BITS));

	if ((cstat & TC_FrmCmp) == 0) {
		/* not transmitted */
		return 0;
	}

	ASSERT(lp->tx_list_len > 0);
	lp->tx_list_len--;
	lp->tx_list_done++;
	lp->tx_list_head = SLOT(slot + 1, TX_RING_SIZE);

	return GEM_TX_DONE;
}

#ifdef DEBUG_LEVEL
static void
tne_dump_packet(struct gem_dev *dp, uint8_t *bp, int n)
{
	int	i;

	for (i=0; i < n; i += 8, bp += 8) {
		cmn_err(CE_CONT, "%02x %02x %02x %02x %02x %02x %02x %02x",
		bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7]);
	}
}
#endif
static uint_t
tne_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct rx_list		*rlp;
	uint_t			flag;
	uint_t			len;
	uint32_t		cstat;
	struct tne_dev		*lp = (struct tne_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rlp = &((struct rx_list *)dp->rx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rlp) - dp->rx_ring),
		sizeof(struct rx_list), DDI_DMA_SYNC_FORKERNEL);

	cstat = ddi_get32(h, &rlp->rx_cstat);

	if ((cstat & RC_FrmCmp) == 0) {
		/* not received */
		return 0;
	}

	len  = (cstat >> RC_FRAMESIZE_SHIFT) & RC_FRAMESIZE_MASK;
	flag = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT,
		"!%s: tne_rx_desc_stat: slot:%d, rx_cstat:0x%b",
		dp->name, slot, cstat, RC_BITS));

	ASSERT(lp->rx_list_len > 0);
	lp->rx_list_len--;
	lp->rx_list_done++;
	lp->rx_list_head = SLOT(slot + 1, RX_RING_SIZE);

	if ((cstat & RC_RxError) != 0) {
		flag |= GEM_RX_ERR;
	}

#if DEBUG_LEVEL > 3
	tne_dump_packet(dp, dp->rx_buf_head->rxb_buf, len);
#endif
	return flag | len;
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

	tne_get_stats(dp);
}

static u_int
tne_interrupt(struct gem_dev *dp)
{
	uint16_t	host_int;
	uint8_t		int_type;
	uint16_t	int_vec;
	boolean_t	tx_sched = FALSE;
	int		loop;
	boolean_t	need_to_reset = FALSE;
	uint16_t	netsts;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	DPRINTF(3, (CE_CONT, "!%s: tne_interrupt: called", dp->name));

	host_int = INW(dp, HOST_INT);
	int_type = (host_int & HI_IntType) >> HI_IntType_SHIFT;
	int_vec  = (host_int & HI_IntVec) >> HI_IntVec_SHIFT;

	if (int_type == IntType_NoIntr) {
		/* not for us */
		return DDI_INTR_UNCLAIMED;
	}

	/* Disable all interrupts */
	OUTL(dp, HOST_CMD, HC_IntsOff);
#ifdef MAP_MEMx
	FLSHL(dp, HOST_CMD);
#endif

	if (!dp->nic_active) {
		/* Ack */
		OUTW(dp, HOST_INT, host_int);
		return DDI_INTR_CLAIMED;
	}

#ifdef CONFIG_POLLING
	if (dp->speed == GEM_SPD_100 &&
	    lp->last_poll_interval != dp->poll_interval) {
		/*
		 * update interrupt pacing timer
		 * we use half polling period expected.
		 */
		OUTL(dp, HOST_CMD, HC_LdTmr | min(dp->poll_interval/4, 255));
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
#ifdef MAP_MEMx
		FLSHW(dp, HOST_INT);
#endif
		DPRINTF(2, (CE_CONT, "!%s: tne_interrupt: type:0x%x vec:0x%x",
			dp->name, int_type, int_vec));

		switch (int_type) {
#ifdef DEBUG_LEVEL
		case IntType_NoIntr:
			/* Never happen */
			cmn_err(CE_PANIC, "%s: NoInter happened", dp->name);
			/* NOT REACHED */
			break;
#endif
		case IntType_TxEOF:
			if (gem_tx_done(dp)) {
				tx_sched = TRUE;
			}

			/* ack number of transmitted frames */
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
			tne_get_stats(dp);

			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_RxEOF:
			/*
			 * packet was received, or receive error happened
			 */
			gem_receive(dp);

			/* notify number of received frames to the nic */
			OUTL(dp, HOST_CMD,
				HC_Ack | (host_int << 16) | lp->rx_list_done);
			lp->rx_list_done = 0;
			break;

		case IntType_Dummy:
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_TxEOC:
			if (gem_tx_done(dp)) {
				tx_sched = TRUE;
			}

			/* ack and start again */
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));

			mutex_enter(&dp->xmitlock);
			lp->tx_list_loaded = FALSE;

			if (lp->tx_list_len > 0) {
				/* start tx again */
				tne_load_tx_list(dp);
			}
			mutex_exit(&dp->xmitlock);
			break;

		case IntType_AdNet:
			if (int_vec != 0) {
				tne_adapter_check(dp);
				need_to_reset = TRUE;
				goto x;
			}
			/* Never happen */

			/* clear the NetSTS register */
			netsts = INW_DIO(dp, NETSTS);
			OUTW_DIO(dp, NETSTS, netsts);
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			break;

		case IntType_RxEOC:
			/*  try to allocate Rx buffer */
			/* ack and start again */
			OUTL(dp, HOST_CMD, HC_Ack | (host_int << 16));
			lp->rx_list_loaded = FALSE;
			dp->stats.norcvbuf++;

			if (lp->rx_list_len > 0) {
				/* start rx again */
				tne_load_rx_list(dp);
			}
			else {
				need_to_reset = TRUE;
				goto x;
			}
			break;
		}
#ifdef MAP_MEMx
		FLSHL(dp, HOST_CMD);
#endif
		host_int = INW(dp, HOST_INT);
		int_type = (host_int & HI_IntType) >> HI_IntType_SHIFT;
		int_vec  = (host_int & HI_IntVec) >> HI_IntVec_SHIFT;

	} while (int_type != IntType_NoIntr && --loop > 0);

x:
	if (loop <= 0) {
		need_to_reset = TRUE;
		cmn_err(CE_WARN, "%s: interrupts maxmum exceeded", dp->name);
	}

	if (need_to_reset) {
		/* Disable all interrupts */
		OUTL(dp, HOST_CMD, HC_IntsOff);

		/* Clear all interrupt */
		OUTW(dp, HOST_INT, INW(dp, HOST_INT));

		mutex_enter(&dp->xmitlock);
		gem_restart_tx(dp);
		mutex_exit(&dp->xmitlock);
		tx_sched = TRUE;

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
	DPRINTF(4, (CE_CONT, "!%s: tne_inter done", dp->name));

	return DDI_INTR_CLAIMED | (tx_sched ? INTR_RESTART_TX : 0);
}

/*
 * HW depend MII routine
 */
#define MDIO_DELAY(dp)    {INB_DIO(dp, NETSIO);}

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
tne_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint8_t		data;
	uint8_t		diodata;

	cmd = MII_READ_CMD(dp->mii_phy_addr, reg);

	OUTW(dp, DIO_ADR, NETSIO);

	diodata = INB_DIO(dp, NETSIO);
	DPRINTF(4, (CE_CONT,
		"!%s: tne_mii_read: diodata: 0x%02x", dp->name, diodata));

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
	if ((INB_DIO(dp, NETSIO) & NSIO_MDATA) != 0) {
		DPRINTF(2, (CE_CONT, "!%s: phy@%d didn't respond",
			dp->name, dp->mii_phy_addr));
	}

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

	return ret;
}

static void
tne_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint32_t	cmd;
	int		i;
	uint8_t		data;
	uint8_t		diodata;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, reg, val);

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
#undef MDIO_DELAY

static int
tne_mii_init(struct gem_dev *dp)
{
	int		ret;
	struct tne_dev	*lp = (struct tne_dev *)dp->private;

	/* First, we try external PHY */
	/* Setup the NetConfig register for external phy */
	OUTW(dp, DIO_ADR, NETCFG);
	OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn);
	ret = gem_mii_init_default(dp);
	if (ret >= 0) {
		lp->which_phy = PHY_EXTERNAL_MII;
		return ret;
	}

	/* Next, we try internal PHY */
	/* Setup the NetConfig register for the internal 10Mbps phy */
	dp->mii_phy_addr = 31;
	lp->which_phy = PHY_INTERNAL_MII;
	OUTW(dp, DIO_ADR, NETCFG);
	OUTW_DIO(dp, NETCFG, NCFG_PEF | NCFG_ONEchn | NCFG_PHY_En);
	return gem_mii_init_default(dp);
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
	uint16_t		comm;
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
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	uint8_t			revid;

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: tneattach: called", drv_name, unit));

	if (cmd == DDI_ATTACH) {
		/*
		 * Check if chip is supported.
		 */
		if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "%s: ddi_regs_map_setup failed",
				drv_name);
			goto err;
		}
		vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
		did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
		revid= pci_config_get8(conf_handle, PCI_CONF_REVID);
		comm = pci_config_get16(conf_handle, PCI_CONF_COMM);
		comm |= PCI_COMM_IO | PCI_COMM_ME;
		pci_config_put16(conf_handle, PCI_CONF_COMM, comm);

		pci_config_put8(conf_handle, PCI_CONF_LATENCY_TIMER, 0x20);

		pci_config_teardown(&conf_handle);

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
			"%s: tne_attach: wrong PCI venid/devid (0x%x, 0x%x)",
			drv_name, vid, did);
		goto err;
chip_found:
		/*
		 * Map in the device registers.
		 */

		/* Search IO-range or memory-range to be mapped */
		regs = NULL;
		len  = 0;
		if (ddi_prop_lookup_int_array(
			DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
			"reg", (int **)&regs, &len) != DDI_PROP_SUCCESS) {

			cmn_err(CE_WARN,
			"%s%d: failed to get reg property", drv_name, unit);
			goto err;
		}
		n = len / (sizeof(struct pci_phys_spec) / sizeof(int));

		ASSERT(regs != NULL && len > 0);
#if DEBUG_LEVEL > 0
		for (i = 0; i < n; i++) {
			cmn_err(CE_CONT,
				"!%s%d: regs[%d]: %08x.%08x.%08x.%08x.%08x",
				drv_name, unit, i,
				regs[i].pci_phys_hi,
				regs[i].pci_phys_mid,
				regs[i].pci_phys_low,
				regs[i].pci_size_hi,
				regs[i].pci_size_low);
		}
#endif
		for (i = 0; i < n; i++) {
			if ((regs[i].pci_phys_hi & PCI_REG_ADDR_M) == 
#ifdef MAP_MEM
				PCI_ADDR_MEM32
#else
				PCI_ADDR_IO
#endif
			) {
				ddi_prop_free(regs);
				goto map_space_found;
			}
		}
		cmn_err(CE_WARN,
#ifdef MAP_MEM
			"%s%d: failed to find MEM32 space",
#else
			"%s%d: failed to find IO space",
#endif
			drv_name, unit);
		ddi_prop_free(regs);
		goto err;

map_space_found:
		if (ddi_regs_map_setup(dip, i, (caddr_t *)&base,
			0, 0, &tne_dev_attr, &regs_handle)) {
			cmn_err(CE_WARN, "%s: ddi_regs_map_setup failed",
				drv_name);
			goto err;
		}
		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = 0;
		gcp->gc_tx_max_frags = GEM_MAXTXFRAGS;
		gcp->gc_tx_desc_size = sizeof(struct tx_list) * TX_RING_SIZE;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_auto_pad  = FALSE;
		gcp->gc_tx_copy_thresh = tne_tx_copy_thresh;

		gcp->gc_rx_buf_align = 0;
#ifdef RXSINGLE
		gcp->gc_rx_max_frags = 1;
#else
		gcp->gc_rx_max_frags = GEM_MAXRXFRAGS;
#endif
		gcp->gc_rx_desc_size = sizeof(struct rx_list) * RX_RING_SIZE;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = tne_rx_copy_thresh;
		gcp->gc_rx_buf_max  = gcp->gc_rx_buf_size + 1;

		/* map attributes (endianness) */
		STRUCT_COPY(gcp->gc_dev_attr, tne_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, tne_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, tne_dev_attr);

		/* dma attributes (boundary) */
		STRUCT_COPY(gcp->gc_dma_attr_desc, tne_dma_attr_desc);

		STRUCT_COPY(gcp->gc_dma_attr_txbuf, tne_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, tne_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 5*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval   = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_NONE;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RSA;
		gcp->gc_mii_dont_reset = FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &tne_attach_chip;
		gcp->gc_reset_chip  = &tne_reset_chip;
		gcp->gc_init_chip   = &tne_init_chip;
		gcp->gc_start_chip  = &tne_start_chip;
		gcp->gc_stop_chip   = &tne_stop_chip;
		gcp->gc_multicast_hash = &tne_mcast_hash;
		gcp->gc_set_rx_filter = &tne_set_rx_filter;
		gcp->gc_set_media   = &tne_set_media;
		gcp->gc_get_stats   = &tne_get_stats;
		gcp->gc_interrupt   = &tne_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &tne_tx_desc_write;
		gcp->gc_rx_desc_write = &tne_rx_desc_write;

		gcp->gc_tx_desc_init = &tne_tx_desc_init;
		gcp->gc_rx_desc_init = &tne_rx_desc_init;
		gcp->gc_tx_desc_stat = &tne_tx_desc_stat;
		gcp->gc_rx_desc_stat = &tne_rx_desc_stat;
		gcp->gc_tx_desc_clean = &tne_tx_desc_init;
		gcp->gc_rx_desc_clean = &tne_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_init  = &gem_mii_init_default;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync  = &tne_mii_sync;
		gcp->gc_mii_read  = &tne_mii_read;
		gcp->gc_mii_write = &tne_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = kmem_zalloc(sizeof(struct tne_dev), KM_SLEEP);
		lp->rev_id = revid;
		dp = gem_do_attach(dip, gcp, base, &regs_handle,
					lp, sizeof(*lp));

		kmem_free(gcp, sizeof(*gcp));

		if (dp != NULL) {
			return DDI_SUCCESS;
		}
err_free_mem:
		kmem_free(lp, sizeof(struct tne_dev));
err:
		return DDI_FAILURE;
	}
	return DDI_FAILURE;
}

static int
tnedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	if (cmd == DDI_DETACH) {
		ret = gem_do_detach(dip);
		if (ret != DDI_SUCCESS) {
			return DDI_FAILURE;
		}
		return DDI_SUCCESS;
	}
	return DDI_FAILURE;
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info tneminfo = {
	0,			/* mi_idnum */
	"tne",			/* mi_idname */
	0,			/* mi_minpsz */
	TNE_MAX_MTU,		/* mi_maxpsz */
	TX_BUF_SIZE*ETHERMAX,	/* mi_hiwat */
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

	DPRINTF(2, (CE_CONT, "!tne: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
