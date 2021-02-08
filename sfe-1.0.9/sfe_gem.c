/*
 *  sfe_gem.c : DP83815/DP83816/SiS900 Fast Ethernet MAC driver for Solaris
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
#pragma	ident	"@(#)sfe_gem.c	1.28 05/08/31"

/*
 Change log
 02/11/2003	PHY register patch for 83815CVNG.
		NOINTR_WORKAROUND implemented
 02/11/2003	0.9.4 released
 02/16/2003	 trace for hung
 02/17/2003	0.9.5 released
 03/13/2003	fixed for sparc
 04/05/2003	0.9.6 released
 04/10/2003	sfe_reset_chip changed for sis96x
 04/20/2003	sfe_set_media: maxdma size fixed for builtin sis900
 04/20/2003	0.9.8 released
 04/20/2003	sfe_set_rx_filter_dp83815: perfect match bug fixed
 04/20/2003	0.9.9 released
 04/21/2003	sfe_tx_desc_write: type of intreq fixed for sparc
 04/22/2003	0.9.10 released
 05/26/2003	fixed for sparc
 05/27/2003	0.9.12 released
 05/31/2003	burstsizes in dma_attr fixed for sparc.
 09/08/2003	gld entries removed
 10/19/2003	get_packet removed
 10/19/2003	1.0.0 released
 11/18/2003	sfe_set_rx_filter_dp83815: perfect match bug fixed
 12/06/2003	sis900 mii routines were fixed.
 12/06/2003	1.0.1 released
 01/15/2004	TX_RING_SIZE, rx_copy_thresh changed.
 01/18/2004	Error recovery on HIB interrupt fixed
 01/18/2004	1.0.2 released
 01/24/2004	tx_timeout changed for fixed gem_tx_timeout (2sec->5sec) 
 01/24/2004	1.0.3 released
 */

/*
 TODO:
	need to customize mii_link_water and mii_config
	need to eliminate rx suspend/pause frame.
	100m loop back test
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

#include <sys/byteorder.h>

#include <sys/pci.h>
#include "mii.h"
#include "gem.h"
#include "sfereg.h"

char	ident[] = "sis900/dp83815 driver " VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int sfe_debug = DEBUG_LEVEL;
# if DEBUG_LEVEL > 4
#    define	CONS	"^"
#  else
#    define	CONS	"!"
#  endif
#define	DPRINTF(n, args)	if (sfe_debug>(n)) cmn_err args
#else
#define	CONS	"!"
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	FALSE	(0)
#define	TRUE	(!FALSE)
#define	ONESEC			(drv_usectohz(1*1000000))

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

#ifdef MAP_MEM
#define	FLSHB(dp,reg)		INB(dp, reg)
#define	FLSHW(dp,reg)		INW(dp, reg)
#define	FLSHL(dp,reg)		INL(dp, reg)
#else
#define	FLSHB(dp,reg)
#define	FLSHW(dp,reg)
#define	FLSHL(dp,reg)
#endif /* MAP_MEM */

/*
 * Our configuration
 */
#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif
#ifndef	TX_RING_SIZE
#define	TX_RING_SIZE	(TX_BUF_SIZE * 4)
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	64
#endif
#ifndef	RX_RING_SIZE
#define	RX_RING_SIZE	RX_BUF_SIZE
#endif

#define	MAXTXFRAGS	GEM_MAXTXFRAGS
#define	MAXRXFRAGS	1

#define	OUR_INTR_BITS	\
	(ISR_DPERR | ISR_SSERR | ISR_RMABT | ISR_RTABT | ISR_RXSOVR |	\
	 ISR_TXURN | ISR_TXDESC | ISR_TXERR |	\
	 ISR_RXORN | ISR_RXIDLE | ISR_RXOK | ISR_RXERR)

#define USE_MULTICAST_HASHTBL

static int	sfe_tx_copy_thresh = 256;
static int	sfe_rx_copy_thresh = 256;

/* special PHY registers for SIS900 */
#define	MII_CONFIG1	0x0010
#define	MII_CONFIG2	0x0011
#define	MII_MASK	0x0013
#define	MII_RESV	0x0014

#define	PHY_MASK		0xfffffff0
#define	PHY_SIS900_INTERNAL	0x001d8000
#define	PHY_ICS1893		0x0015f440

/* IEEE802.3x mac control frame for flow controi */
struct pause_frame {
	struct ether_addr	dest;
	struct ether_addr	src;
	uint16_t	type_len;
	uint16_t	pause_flag;
	uint16_t	pause_time;
};

static struct ether_addr	sfe_pause_mac_addr = {
	0x01, 0x80, 0xc2, 0x00, 0x00, 0x01,
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*chip_name;
	int		chip_type;
#define	CHIPTYPE_DP83815	0
#define	CHIPTYPE_SIS900		1
};

/*
 * Chip dependant MAC state
 */
struct sfe_dev {
	/* misc HW information */
	struct chip_info	*chip;
	uint32_t		our_intr_bits;
	uint32_t		cr;
	int			tx_drain_threshold;
	int			tx_fill_threshold;
	int			rx_drain_threshold;
	int			rx_fill_threshold;
	uint8_t			revid;	/* revision from PCI configuration */
	boolean_t		(*get_mac_addr)(struct gem_dev *);
#ifdef TUNE_PHY_630
	uint8_t			bridge_revid;
#endif
};

/*
 * Hardware information
 */
struct chip_info sfe_chiptbl[] = {
#ifdef CONFIG_SIS900
 {
	0x1039,	0x0900,	"SiS900", CHIPTYPE_SIS900,
 },
#endif
#ifdef CONFIG_DP83815
 {
	0x100b,	0x0020,	"DP83815/83816", CHIPTYPE_DP83815,
 },
#endif
#ifdef CONFIG_SIS7016
 {
	0x1039,	0x7016,	"SiS7016", CHIPTYPE_SIS900,
 },
#endif
};
#define	CHIPTABLESIZE	(sizeof(sfe_chiptbl)/sizeof(struct chip_info))

/* ======================================================== */
 
/* mii operations */
static void  sfe_mii_sync_dp83815(struct gem_dev *);
static void  sfe_mii_sync_sis900(struct gem_dev *);
static uint16_t  sfe_mii_read_dp83815(struct gem_dev *, uint_t);
static uint16_t  sfe_mii_read_sis900(struct gem_dev *, uint_t);
static void sfe_mii_write_dp83815(struct gem_dev *, uint_t, uint16_t);
static void sfe_mii_write_sis900(struct gem_dev *, uint_t, uint16_t);
#ifdef i86pc
static void sfe_set_eq_sis630(struct gem_dev *dp);
#endif
/* nic operations */
static int sfe_reset_chip_sis900(struct gem_dev *);
static int sfe_reset_chip_dp83815(struct gem_dev *);
static void sfe_init_chip(struct gem_dev *);
static void sfe_start_chip(struct gem_dev *);
static void sfe_stop_chip(struct gem_dev *);
static void sfe_set_media(struct gem_dev *);
static void sfe_set_rx_filter_dp83815(struct gem_dev *);
static void sfe_set_rx_filter_sis900(struct gem_dev *);
static void sfe_get_stats(struct gem_dev *);
static int sfe_attach_chip(struct gem_dev *);

/* descriptor operations */
static int sfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq);
static int sfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t sfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t sfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void sfe_tx_desc_init(struct gem_dev *dp, int slot);
static void sfe_rx_desc_init(struct gem_dev *dp, int slot);
static void sfe_tx_desc_clean(struct gem_dev *dp, int slot);
static void sfe_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int sfe_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr sfe_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr sfe_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t sfe_dma_attr_buf = {
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

static ddi_dma_attr_t sfe_dma_attr_desc = {
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

#define	SFE_EEPROM_DELAY(dp)	{INL(dp, EROMAR);INL(dp, EROMAR);}
#define	EE_CMD_READ	6
#define	EE_CMD_SHIFT	6

static uint16_t
sfe_read_eeprom(struct gem_dev *dp, int offset)
{
	int	eedi;
	int	i;
	int	ret;

	/* ensure de-assert chip select */
	OUTL(dp, EROMAR, 0);
	SFE_EEPROM_DELAY(dp);
	OUTL(dp, EROMAR, EROMAR_EESK);
	SFE_EEPROM_DELAY(dp);

	/* assert chip select */
	offset |= EE_CMD_READ << EE_CMD_SHIFT;

	for (i = 8; i >= 0; i--) {
		/* make command */
		eedi = ((offset >> i) & 1) << EROMAR_EEDI_SHIFT;

		/* send 1 bit */
		OUTL(dp, EROMAR, EROMAR_EECS | eedi);
		SFE_EEPROM_DELAY(dp);
		OUTL(dp, EROMAR, EROMAR_EECS | eedi | EROMAR_EESK);
		SFE_EEPROM_DELAY(dp);
	}

	OUTL(dp, EROMAR, EROMAR_EECS);

	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTL(dp, EROMAR, EROMAR_EECS);
		SFE_EEPROM_DELAY(dp);
		OUTL(dp, EROMAR, EROMAR_EECS | EROMAR_EESK);
		SFE_EEPROM_DELAY(dp);

		ret = (ret << 1) | ((INL(dp, EROMAR) >> EROMAR_EEDO_SHIFT) & 1);
	}

	OUTL(dp, EROMAR, 0);
	SFE_EEPROM_DELAY(dp);

	return ret;
}
#undef SFE_EEPROM_DELAY

static boolean_t
sfe_get_mac_addr_dp83815(struct gem_dev *dp)
{
	uint8_t		*mac;
	int		val;
	int		i;

#define	BITSET(p, ix, v)	(p)[(ix)/8] |= ((v) ? 1 : 0) << ((ix) & 0x7)
#define	BITTEST(v, pos)		((v) & (1 << (pos)))

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_get_mac_addr_dp83815: called",
		dp->name));

	mac = dp->dev_addr.ether_addr_octet;

	/* first of all, clear MAC address buffer */
	bzero(mac, ETHERADDRL);

	/* get bit 0 */
	val = sfe_read_eeprom(dp, 0x6);
	BITSET(mac, 0, val & 1);

	/* get bit 1 - 16 */
	val = sfe_read_eeprom(dp, 0x7);
	for (i = 0; i < 16; i++) {
		BITSET(mac, 1 + i, val & (1 << (15 - i)));
	}

	/* get bit 17 -  32 */
	val = sfe_read_eeprom(dp, 0x8);
	for (i = 0; i < 16; i++) {
		BITSET(mac, 17 + i, val & (1 << (15 - i)));
	}

	/* get bit 33 -  47 */
	val = sfe_read_eeprom(dp, 0x9);
	for (i = 0; i < 15; i++) {
		BITSET(mac, 33 + i, val & (1 << (15 - i)));
	}

	return TRUE;
#undef BITSET
#undef BITTEST
}

static boolean_t
sfe_get_mac_addr_sis900(struct gem_dev *dp)
{
	int		val;
	int		i;
	uint8_t		*mac;
	
	mac = dp->dev_addr.ether_addr_octet;

	for (i = 0; i < ETHERADDRL/2; i++) {
		val = sfe_read_eeprom(dp, 0x8 + i);
		*mac++ = val;
		*mac++ = val >> 8;
	}

	return TRUE;
}

static dev_info_t *
sfe_search_pci_dev_subr(dev_info_t *cur_node, int vendor_id, int device_id)
{
	dev_info_t	*child_id;
	dev_info_t	*ret;
	int		vid, did;

	if (cur_node == NULL) {
		return NULL;
	}

	/* check brothers */
	do {
		vid = ddi_getprop(DDI_DEV_T_ANY, cur_node,
					DDI_PROP_DONTPASS, "vendor-id", -1);
		did = ddi_getprop(DDI_DEV_T_ANY, cur_node,
					DDI_PROP_DONTPASS, "device-id", -1);

		if (vid == vendor_id && did == device_id) {
			/* found */
			return cur_node;
		}

		/* check children */
		if ((child_id = ddi_get_child(cur_node)) != NULL) {
			if((ret = sfe_search_pci_dev_subr(
				child_id, vendor_id, device_id)) != NULL) {
				return ret;
			}
		}

	} while ((cur_node = ddi_get_next_sibling(cur_node)) != NULL);

	/* not found */
	return NULL;
}

static dev_info_t *
sfe_search_pci_dev(int vendor_id, int device_id)
{
	return sfe_search_pci_dev_subr(ddi_root_node(), vendor_id, device_id);
}

static boolean_t
sfe_get_mac_addr_sis630e(struct gem_dev *dp)
{
#if defined(i86pc) && defined(GET_MAC_ADDR_SIS630E)
	uint32_t	rfcrSave;
	int		i;
	uint16_t	v;
        dev_info_t	*isa_bridge;
        ddi_acc_handle_t isa_handle;
        int		reg;

	if ((isa_bridge = sfe_search_pci_dev(0x1039, 0x8)) == NULL) {
		cmn_err(CE_WARN, "%s: failed to find isa-bridge pci1039,8",
				dp->name);
		return FALSE;
	}

	if (pci_config_setup(isa_bridge, &isa_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "%s: ddi_regs_map_setup failed",
			dp->name);
		return FALSE;
	}

	/* enable to access CMOS RAM */
	reg = pci_config_get8(isa_handle, 0x48);
	pci_config_put8(isa_handle, 0x48, reg | 0x40);

	for (i = 0; i < ETHERADDRL; i++) {
		outb(0x70, 0x09 + i);
		dp->dev_addr.ether_addr_octet[i] = inb(0x71); 
	}

	/* disable to access CMOS RAM */
	pci_config_put8(isa_handle, 0x48, reg);
	pci_config_teardown(&isa_handle);
x:

	return TRUE;
#else
	return FALSE;
#endif
}

static boolean_t
sfe_get_mac_addr_sis635(struct gem_dev *dp)
{
	int		i;
	uint32_t	rfcr;
	uint16_t	v;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(2, (CE_CONT, CONS "%s: sfe_get_mac_addr_sis635: called",
		dp->name));
	rfcr = INL(dp, RFCR);

	OUTL(dp, CR, lp->cr | CR_RELOAD);
	OUTL(dp, CR, lp->cr);

	/* disable packet filtering before reading filter */
	OUTL(dp, RFCR, rfcr & ~RFCR_RFEN);

	/* load MAC addr from filter data register */
	for (i = 0 ; i < ETHERADDRL ; i += 2) {
		OUTL(dp, RFCR,
		    (RFADDR_MAC_SIS900 + (i/2)) << RFCR_RFADDR_SHIFT_SIS900);
		v = INL(dp, RFDR);
		dp->dev_addr.ether_addr_octet[i]   = v;
		dp->dev_addr.ether_addr_octet[i+1] = v >> 8;
	}

	/* re-enable packet filitering */
	OUTL(dp, RFCR, rfcr | RFCR_RFEN);

	return TRUE;
}

#ifdef i86pc
static boolean_t
sfe_get_mac_addr_sis962(struct gem_dev *dp)
{
	int	ret;
	int	i;

	ret = FALSE;

	/* rise request signal */
	OUTL(dp, MEAR, EROMAR_EEREQ);
	for (i = 0; (INL(dp, MEAR) & EROMAR_EEGNT) == 0; i++) {
		if (i > 200) {
			/* failed to aquire eeprom */
			cmn_err(CE_NOTE,
				CONS "%s: failed to access eeprom", dp->name);
			goto x;
		}
		drv_usecwait(10);      
	}
	ret = sfe_get_mac_addr_sis900(dp);
x:
	/* release EEPROM */
	OUTL(dp, MEAR, EROMAR_EEDONE);

	return ret;
}
#endif /* i86pc */

static int
sfe_reset_chip_sis900(struct gem_dev *dp)
{
	int		i;
	uint32_t	done;
	uint32_t	val;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_reset_chip_sis900 called",
		dp->name));

	lp->cr = 0;
	OUTL(dp, IER, 0);
	OUTL(dp, IMR, 0);
	OUTL(dp, RFCR, 0);

	OUTL(dp, CR, CR_RST | CR_TXR | CR_RXR);
	drv_usecwait(10);

	done = 0;
	for (i = 0; done != (ISR_TXRCMP | ISR_RXRCMP); i++) {
		if (i > 1000) {
			cmn_err(CE_WARN, "%s: chip reset timeout", dp->name);
			return 1;	
		}
		done |= INL(dp, ISR) & (ISR_TXRCMP | ISR_RXRCMP);
		drv_usecwait(10);
	}
#ifdef i86pc
	if (lp->revid == SIS630ET_900_REV) {
		lp->cr |= CR_ACCESSMODE;
		OUTL(dp, CR, lp->cr | INL(dp, CR));
	}
#endif
	/* Configration register: PCI parity enable */
	DPRINTF(2, (CE_CONT, CONS "%s: cfg:%b",
			dp->name, INL(dp, CFG), CFG_BITS_SIS900));
	val = CFG_PESEL;
	if (lp->revid >= SIS635A_900_REV ||
	    lp->revid == SIS900B_900_REV) {
		/* what is this ? */
		val |= CFG_RND_CNT;
	}
	OUTL(dp, CFG, val);
	DPRINTF(2, (CE_CONT, CONS "%s: cfg:%b", dp->name,
		INL(dp, CFG), CFG_BITS_SIS900));

	return 0;
}

static int
sfe_reset_chip_dp83815(struct gem_dev *dp)
{
	int		i;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_reset_chip_dp83815 called",
		dp->name));

	lp->cr = 0;
	OUTL(dp, IER, 0);
	OUTL(dp, IMR, 0);
	OUTL(dp, RFCR, 0);

	OUTL(dp, CR, CR_RST);
	drv_usecwait(10);

	for (i = 0; (INL(dp, CR) & CR_RST) != 0; i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: chip reset timeout", dp->name);
			return 1;	
		}
		drv_usecwait(10);
	}

	OUTL(dp, CCSR, CCSR_PMESTS);
	OUTL(dp, CCSR, 0);

	/* Configration register: PCI parity enable */
	DPRINTF(2, (CE_CONT, CONS "%s: cfg:%b",
			dp->name, INL(dp, CFG), CFG_BITS_DP83815));
	OUTL(dp, CFG, CFG_PESEL | CFG_PAUSE_ADV);
	DPRINTF(2, (CE_CONT, CONS "%s: cfg:%b", dp->name,
		INL(dp, CFG), CFG_BITS_DP83815));

	return 0;
}

static void
sfe_init_chip(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	/* Configration register: have been set up in sfe_chip_reset */

	/* PCI test control register: do nothing */

	/* Interrupt status register : clear pended interrupts */
	(void) INL(dp, ISR);

	/* Interrupt enable register: enable interrupt */
	OUTL(dp, IER, 0);

	/* Interrupt mask register: clear */
	lp->our_intr_bits = OUR_INTR_BITS;
	OUTL(dp, IMR, lp->our_intr_bits);

	/* Enhanced PHY Access register (sis900): do nothing */

	/* Transmit Descriptor Pointer register: base addr of TX ring */
	OUTL(dp, TXDP, dp->tx_ring_dma);

	/* Receive descriptor pointer register: base addr of RX ring */
	ASSERT(dp->rx_ring_dma != 0);
	OUTL(dp, RXDP, dp->rx_ring_dma);
}

static uint_t
sfe_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return gem_ether_crc_be(addr);
}

static void
sfe_set_rx_filter_dp83815(struct gem_dev *dp)
{
	int		i;
	uint32_t	mode;
	int		addr_shift;
	uint8_t		*mac;
#ifdef USE_MULTICAST_HASHTBL
	uint16_t	hash_tbl[32];
#endif

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_set_rx_filter_dp83815: called",
		dp->name));

	addr_shift = RFCR_RFADDR_SHIFT_DP83815;
	mac = dp->cur_addr.ether_addr_octet;

	/* Set Receive filter control register */

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		mode = RFCR_AAB		/* all broadcast */
		     | RFCR_AAM		/* all multicast */
		     | RFCR_AAP;	/* all physcal */
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0) {
		mode = RFCR_AAB		/* all broadcast */
		     | RFCR_AAM		/* all multicast */
		     | RFCR_APM_DP83815;/* physical for the chip */
	}
	else {
		mode = RFCR_AAB		/* all broadcast */
		     | RFCR_APM_DP83815;/* physical for the chip */

		if (dp->mc_count == 0) {
			/* no multicast, do nothing */
		} else
#ifdef USE_MULTICAST_PERFECT
		if (dp->mc_count <= 4) {
			for (i = 0; i < dp->mc_count; i++) {
				mode |= 1 << (RFCR_APAT_SHIFT + i);
			}
		} else
#endif
#ifdef USE_MULTICAST_HASHTBL
		if (dp->mc_count <= 16*32/2) {
			/* use multicast hash table */
			/* enable mcast hash tble */
			mode |= RFCR_MHEN_DP83815;

			/* make hash table */
			bzero(hash_tbl, sizeof(hash_tbl));
			for (i = 0; i < dp->mc_count; i++) {
				uint_t	h;
				h = dp->mc_list[i].hash & (32*16 - 1);
				hash_tbl[h / 16] |= 1 << (h % 16);
			}
		} else
#endif
		{
			mode |= RFCR_AAM;
		}
	}

	/* Disable Rx filter and load mac address for the chip */
	for (i = 0; i < ETHERADDRL; i += 2) {
		OUTL(dp, RFCR, RFADDR_MAC_DP83815 + i);
		OUTL(dp, RFDR, (mac[i+1] << 8) | mac[i]);
	}

#ifdef USE_MULTICAST_PERFECT
	if (mode & RFCR_APAT_DP83815) {
		int	j;
		static uint_t	rf_perfect_base[]= {
			RFADDR_PMATCH0_DP83815, RFADDR_PMATCH1_DP83815,
			RFADDR_PMATCH2_DP83815, RFADDR_PMATCH3_DP83815};

		/* setup perfect match patterns*/
		for (j = 0; j < dp->mc_count; j++) {
			mac = &dp->mc_list[j].addr.ether_addr_octet[0];
			for (i = 0; i < ETHERADDRL; i += 2) {
				OUTL(dp, RFCR, rf_perfect_base[j] + i*2);
				OUTL(dp, RFDR, (mac[i+1] << 8) | mac[i]);
			}
		}

		/* setup pattern count register */
		OUTL(dp, RFCR, RFADDR_PCOUNT01_DP83815);
		OUTL(dp, RFDR, ETHERADDRL<<8 | ETHERADDRL);
		OUTL(dp, RFCR, RFADDR_PCOUNT23_DP83815);
		OUTL(dp, RFDR, ETHERADDRL<<8 | ETHERADDRL);
	}
#endif
#ifdef USE_MULTICAST_HASHTBL
	if (mode & RFCR_MHEN_DP83815) {
		/* Load Multicast hash table */
		for (i = 0; i < 32; i++ ) {
			/* For Dp83815, index is in byte */
			OUTL(dp, RFCR, RFADDR_MULTICAST_DP83815 + i*2);
			OUTL(dp, RFDR, hash_tbl[i]);
		}
	}
#endif
	/* Set rx filter mode and enable rx filter */
	OUTL(dp, RFCR, RFCR_RFEN | mode);
}

static void
sfe_set_rx_filter_sis900(struct gem_dev *dp)
{
	int		i;
	uint32_t	mode;
	uint16_t	hash_tbl[16];
	uint8_t		*mac;
	uint16_t	m[3];
	int		hash_size;
	int		hash_shift;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_set_rx_filter_sis900: called",
			dp->name));
	mac = dp->cur_addr.ether_addr_octet;

	/*
	 * determin hardware hash table size in word.
	 */
	hash_shift = 25;
	if ((lp->revid >= SIS635A_900_REV) ||
	    (lp->revid == SIS900B_900_REV)) {
		hash_shift = 24;
	}
	hash_size = (1 << (32 - hash_shift)) / 16;
	bzero(hash_tbl, sizeof(hash_tbl));

	/* Set Receive filter control register */

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		mode = RFCR_AAB | RFCR_AAM | RFCR_AAP;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0) {
		mode = RFCR_AAB | RFCR_AAM;
	}
	else {
		mode = RFCR_AAB;
		if (dp->mc_count == 0) {
			/* no multicast, do nothing */
		} else
#ifdef USE_MULTICAST_HASHTBL
		if (dp->mc_count < 100) {
			/* make hash table */
			for (i = 0; i < dp->mc_count; i++) {
				uint_t	h;
				h = dp->mc_list[i].hash >> hash_shift;
				hash_tbl[h / 16] |= 1 << (h % 16);
			}
		} else
#endif
		{
			mode |= RFCR_AAM;
		}
	}

	/* Disable Rx filter and load mac address */
	for (i = 0; i < ETHERADDRL/2; i++) {
		/* For sis900, index is in word */
		OUTL(dp, RFCR,
		    (RFADDR_MAC_SIS900 + i) << RFCR_RFADDR_SHIFT_SIS900);
		OUTL(dp, RFDR, (mac[i*2+1] << 8) | mac[i*2]);
	}

	/* Load Multicast hash table */
	for (i = 0; i < hash_size; i++) {
		/* For sis900, index is in word */
		OUTL(dp, RFCR,
		    (RFADDR_MULTICAST_SIS900 + i) << RFCR_RFADDR_SHIFT_SIS900);
		OUTL(dp, RFDR, hash_tbl[i]);
	}

	/* Load rx filter mode and enable rx filter */
	OUTL(dp, RFCR, RFCR_RFEN | mode);
}

static void
sfe_start_chip(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_start_chip: called", dp->name));

	/*
	 * Setup Tx and Rx configuration registers and
	 * flow/pause control register.
	 */
	sfe_set_media(dp);

	/* enable interrupt */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IER, 1);
	}

	/* Kick RX */
	OUTL(dp, CR, lp->cr | CR_RXE);
}

static void
sfe_stop_chip(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;
	uint32_t	done;
	int		i;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_stop_chip: called", dp->name));
        /*
	 * Stop chip core
	 */
	/* Inhibit interrupt */
	OUTL(dp, IER, 0);

	/* stop TX and RX immediately */
	OUTL(dp, CR, lp->cr | CR_TXR | CR_RXR);

	done = 0;
	for (i = 0; done != (ISR_RXRCMP | ISR_TXRCMP); i++) {
		if (i > 1000) {
			cmn_err(CE_NOTE,
				"!%s: sfe_stop_chip: Tx/Rx reset timeout",
				dp->name);
			break;
		}
		done |= INL(dp, ISR) & (ISR_RXRCMP | ISR_TXRCMP);
		drv_usecwait(10);
	}
}

static void
sfe_set_media(struct gem_dev *dp)
{
	uint32_t	txcfg;
	uint32_t	rxcfg;
	uint32_t	pcr;
	uint32_t	edb;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;
#ifdef DEBUG_LEVEL
	static int	speed[3] = {10, 100, 1000};
#endif
	if (!dp->nic_active) {
		return;
	}

	DPRINTF(2, (CE_CONT, CONS "%s: sfe_set_media: %s duplex, %d Mbps",
		dp->name,
		dp->full_duplex?"full":"half", speed[dp->speed]));

	edb = 0;
	if (lp->chip->chip_type == CHIPTYPE_SIS900) {
#ifdef SIS_EDB_TEST
		edb = CFG_EDB_MASTER_EN;
#else
		edb = INL(dp, CFG) & CFG_EDB_MASTER_EN;
#endif
	}

	/* tx low water mark */
	lp->tx_fill_threshold = TXFIFOSIZE / 4;			/* 1/4 FIFO */

	/* high water marks */
	if (dp->speed == GEM_SPD_100) {
		/* transmit starting threshold */
		lp->tx_drain_threshold = (TXFIFOSIZE/4)*3;	/* S&F */
	} else {
		/* transmit starting threshold */
		lp->tx_drain_threshold = (TXFIFOSIZE/4)*2;	/* 2/4 FIFO */
	}
	/* receive starting threshold */
	lp->rx_drain_threshold = 128;

	/* fix tx_drain_theshold */
	lp->tx_drain_threshold =
		min(lp->tx_drain_threshold, TXFIFOSIZE - lp->tx_fill_threshold);

	/* fix rx_drain_theshold; it have only 5bit-wide field */
	lp->rx_drain_threshold =
		min(lp->rx_drain_threshold, 31 * RXCFG_DRTH_UNIT);

	ASSERT(lp->tx_drain_threshold < 64*TXCFG_DRTH_UNIT);
	ASSERT(lp->tx_fill_threshold < 64*TXCFG_FLTH_UNIT);
	ASSERT(lp->rx_drain_threshold < 32*RXCFG_DRTH_UNIT);

	txcfg =	TXCFG_ATP |		/* auto padding */
		(edb ? TXCFG_MXDMA_64 : TXCFG_MXDMA_512) |
		(dp->full_duplex ? (TXCFG_CSI | TXCFG_HBI) : 0) |
		(lp->tx_fill_threshold/TXCFG_FLTH_UNIT) << TXCFG_FLTH_SHIFT |
		lp->tx_drain_threshold/TXCFG_DRTH_UNIT;
	OUTL(dp, TXCFG, txcfg);

	rxcfg =	RXCFG_AEP | RXCFG_ARP |
		(dp->full_duplex ? RXCFG_ATX : 0) |
		(edb ? RXCFG_MXDMA_64 : RXCFG_MXDMA_512) |
		(lp->rx_drain_threshold/RXCFG_DRTH_UNIT) << RXCFG_DRTH_SHIFT;
	OUTL(dp, RXCFG, rxcfg);

	DPRINTF(0, (CE_CONT, CONS "%s: sfe_set_media: txcfg:%b rxcfg:%b",
		dp->name,
		txcfg, TXCFG_BITS, rxcfg, RXCFG_BITS));

	/* Flow control */
	if (lp->chip->chip_type == CHIPTYPE_DP83815) {
		pcr = INL(dp, PCR);
		if (dp->flow_control) {
			OUTL(dp, PCR, pcr | PCR_PSEN | PCR_PS_MCAST);
		}
		else {
			OUTL(dp, PCR,
				pcr & ~(PCR_PSEN | PCR_PS_MCAST | PCR_PS_DA));
		}
		DPRINTF(2, (CE_CONT, CONS "%s: PCR: %b", dp->name,
				INL(dp, PCR), PCR_BITS));
	}
	else if (lp->chip->chip_type == CHIPTYPE_SIS900) {
 
		if (dp->flow_control) {
			OUTL(dp, FLOWCTL, FLOWCTL_FLOWEN);
		}
		else {
			OUTL(dp, FLOWCTL, 0);
		}
		DPRINTF(2, (CE_CONT, CONS "%s: FLOWCTL: %b",
			dp->name, INL(dp, FLOWCTL), FLOWCTL_BITS));
	}
}

static void
sfe_get_stats(struct gem_dev *dp)
{
	/* do nothing */
}

/*
 * discriptor  manupiration
 */
static int
sfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq)
{
	int			n;
	uint32_t		mark;
	struct sfe_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	struct sfe_dev		*lp = (struct sfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;
#if DEBUG_LEVEL > 2
	int			i;

	cmn_err(CE_CONT,
CONS "%s: time:%d sfe_tx_desc_write seqnum: %d, slot %d, frags: %d intreq: %d",
		dp->name, ddi_get_lbolt(),
		dp->tx_desc_tail, slot, frags, intreq);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "%d: addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
#if DEBUG_LEVEL > 3
	intreq |= GEM_TXFLAG_INTR;
#endif
	mark = ((intreq & GEM_TXFLAG_INTR) != 0)
			? (CMDSTS_OWN | CMDSTS_INTR) : CMDSTS_OWN;
	dcp = &dmacookie[frags];
	n = frags;
	while (n--) {
		/* make a slot for n-th fragment */
		dcp--;
		tdp = &((struct sfe_desc *)dp->tx_ring)[
				SLOT(slot + n, TX_RING_SIZE)];

		ddi_put32(h, &tdp->d_bufptr, (uint32_t)dcp->dmac_address);
		ddi_put32(h, &tdp->d_cmdsts, mark | (uint32_t)dcp->dmac_size);

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

		mark = CMDSTS_OWN | CMDSTS_MORE;
	}

	/*
	 * Let the Transmit Buffer Manager Fill state machine active.
	 */
	OUTL(dp, CR, lp->cr | CR_TXE);

	return frags;
}

static int
sfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct sfe_desc		*rdp;
	struct sfe_dev		*lp = (struct sfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;
#if DEBUG_LEVEL > 2
	int			i;

	ASSERT(frags == 1);

	cmn_err(CE_CONT, CONS
		"%s: sfe_rx_desc_write seqnum: %d, slot %d, frags: %d",
		dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "  frag: %d addr: 0x%x, len: 0x%x",
			i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/* for the last slot of the packet */
	rdp = &((struct sfe_desc *)dp->rx_ring)[slot];

	ddi_put32(h, &rdp->d_bufptr, (uint32_t)dmacookie->dmac_address);
	ddi_put32(h, &rdp->d_cmdsts,
				CMDSTS_INTR | (uint32_t)dmacookie->dmac_size);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

	return 1;
}

static uint_t
sfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct sfe_desc		*tdp;
	uint32_t		status;
	int			i;
	int			cols;
	ddi_acc_handle_t	h = dp->desc_acc_handle;
#ifdef DEBUG_LEVEL
	struct sfe_dev		*lp = (struct sfe_dev *)dp->private;
	clock_t			delay;
#endif
	/* check status of the last descriptor */
	tdp = &((struct sfe_desc *)dp->tx_ring)[
		SLOT(slot + ndesc - 1, TX_RING_SIZE)];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORKERNEL);

	status = ddi_get32(h, &tdp->d_cmdsts);

	DPRINTF(2, (CE_CONT,
		CONS "%s: time:%d sfe_tx_desc_stat: slot:%d, status:0x%b",
		dp->name, ddi_get_lbolt(), slot, status, TXSTAT_BITS));

	if ((status & CMDSTS_OWN) != 0) {
		/*
		 * not yet transmitted
		 */
		return 0;
	}
	ASSERT((status & CMDSTS_MORE) == 0);

#if DEBUG_LEVEL > 3
	delay = (ddi_get_lbolt() - dp->tx_buf_head->txb_stime) * 10;
	if (delay >= 50) {
		DPRINTF(0, (CE_NOTE, "%s: tx deferred %d mS: slot %d",
			dp->name, delay, slot));
	}
#endif

#if DEBUG_LEVEL > 3
	for (i = 0; i < ndesc-1; i++) {
		uint32_t	s;
		int		n;

		n = SLOT(slot + i, TX_RING_SIZE);
		s = ddi_get32(h, &((struct sfe_desc *)dp->tx_ring)[n].d_cmdsts);
		
		ASSERT((s & CMDSTS_MORE) != 0);
		ASSERT((s & CMDSTS_OWN) == 0);
	}
#endif

	/*
	 *  collect statictics
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
	}
	else if (!dp->full_duplex) {

		cols = (status >> CMDSTS_CCNT_SHIFT) & CCNT_MASK;

		if (cols > 0) {
			if (cols == 1) {
				dp->stats.first_coll++;
			} else /* (cols > 1) */ {
				dp->stats.multi_coll++;
			}
			dp->stats.collisions += cols;
		}
		else if ((status & CMDSTS_TD) != 0) {
			dp->stats.defer++;
		}
	}

	return GEM_TX_DONE;
}

static void
sfe_pause_received(struct gem_dev *dp, uint8_t *bp)
{
	uint16_t	type_len;
	uint16_t	pause_flag;
	uint16_t	pause_time;
	uint32_t	val;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	type_len = ntohs(((struct pause_frame *)bp)->type_len);
	pause_flag = ntohs(((struct pause_frame *)bp)->pause_flag);
	pause_time = ntohs(((struct pause_frame *)bp)->pause_time);

	DPRINTF(2, (CE_CONT,
	   CONS "%s: pause packet received: src:%02x:%02x:%02x:%02x:%02x:%02x "
		"type:0x%04x flag:0x%04x time:0x%04x",
		dp->name, bp[6], bp[7], bp[8], bp[9], bp[10], bp[11],
		type_len, pause_flag, pause_time));

	if (lp->chip->chip_type == CHIPTYPE_DP83815) {
		/*
		 * It seems that DP83815 doesn't pause tx automatically
		 * when it receives a IEEE 802.3x pause frame.
		 * For a workaround, we pause transmition manually.
		 * Is it right? But it seems to work.
		 */
		switch (pause_flag) {
		case 1:
			if (pause_time == 0) {
				/* release HW */
				/* cancel pause */
				val = INL(dp, PCR);
				OUTL(dp, PCR, val & ~PCR_PSEN);
				drv_usecwait(1);
				/* restore PCR */
				OUTL(dp, PCR, val);
			}
			else {
				/* issue pause command */
				val = INL(dp, PCR);
				if ((val & (PCR_PS_RCVD | PCR_PS_ACT)) == 0) {
					val &= ~PCR_PAUSE_CNT;
					val |=  pause_time | PCR_MLD_EN;
					OUTL(dp, PCR, val);
				}
			}
			break;

		default:
			cmn_err(CE_NOTE, CONS
				"%s: unknown mac control frame: flag:%d",
				dp->name, pause_flag);
			break;
		}
	}
}

static uint64_t
sfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct sfe_desc		*rdp;
	uint_t			len;
	uint_t			flag;
	uint32_t		status;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	flag = GEM_RX_DONE;

	/* Dont read ISR because we cannot ack only to rx interrupt. */

	rdp = &((struct sfe_desc *)dp->rx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORKERNEL);

	status = ddi_get32(h, &rdp->d_cmdsts);

	DPRINTF(2, (CE_CONT,
	CONS "%s: time:%d sfe_rx_desc_stat: slot:%d, status:0x%b",
		dp->name, ddi_get_lbolt(), slot, status, RXSTAT_BITS));

	if ((status & CMDSTS_OWN) == 0) {
		/*
		 * No more received packets because
		 * this buffer is owned by NIC.
		 */
		return 0;
	}

#define	RX_ERR_BITS \
	(CMDSTS_RXA | CMDSTS_RXO | CMDSTS_LONG | CMDSTS_RUNT | \
	 CMDSTS_ISE | CMDSTS_CRCE | CMDSTS_FAE | CMDSTS_MORE)

	if ((status & RX_ERR_BITS) != 0) {
		/*
		 * Packet with error received
		 */
		DPRINTF(0, (CE_CONT, CONS "%s: Corrupted packet "
			"received, buffer status: %b",
			dp->name, status, RXSTAT_BITS));

		/* collect statistics information */
		dp->stats.errrcv++;
		if ((status & CMDSTS_RXO) != 0) {
			dp->stats.overflow++;
		}
		if ((status & (CMDSTS_LONG | CMDSTS_MORE)) != 0) {
			dp->stats.frame_too_long++;
		}
		if ((status & CMDSTS_RUNT) != 0) {
			dp->stats.runt++;
		}
		if ((status & (CMDSTS_ISE | CMDSTS_FAE)) != 0) {
			dp->stats.frame++;
		}
		if ((status & CMDSTS_CRCE) != 0) {
			dp->stats.crc++;
		}

		return flag | GEM_RX_ERR;
	}

	/*
	 * this packet was received without errors 
	 */
	if ((len = (status & CMDSTS_SIZE) - ETHERFCSL) < 0) {
		len = 0;
	}
#ifdef RX_FLOW_CONTROL
	if (dp->flow_control && len == ETHERMIN &&
		bcmp(dp->rx_buf_head->rxb_buf,
		  sfe_pause_mac_addr.ether_addr_octet, ETHERADDRL) == 0) {

		sfe_pause_received(dp, (uint8_t *)dp->rx_buf_head->rxb_buf);

		/*
		 * For performance, we discard this packet
		 * if we are not in promiscous mode
		 */
		len = 0;
	}
#endif /* RX_FLOW_CONTROL */

#ifdef notdef
	if (1) {
		int	i;
		uint8_t	*bp = dp->rx_buf_head->rxb_buf;

		cmn_err(CE_CONT, CONS "%s: len:%d", dp->name, len);

		for (i = 0; i < 60; i += 10) {
			cmn_err(CE_CONT, CONS
			"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			bp[0], bp[1], bp[2], bp[3], bp[4],
			bp[5], bp[6], bp[7], bp[8], bp[9]);
		}
		bp += 10;
	}
#endif
	return flag | (len & GEM_RX_LEN);
}

static void
sfe_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct sfe_desc		*tdp;
	uint32_t		here;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tdp = &((struct sfe_desc *)dp->tx_ring)[slot];

	ddi_put32(h, &tdp->d_cmdsts, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

	/* make a link to this from the previous descriptor */
	here = ((uint32_t)dp->tx_ring_dma) + sizeof(struct sfe_desc)*slot;

	tdp = &((struct sfe_desc *)dp->tx_ring)[SLOT(slot - 1, TX_RING_SIZE)];
	ddi_put32(h, &tdp->d_link, here);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

	return;
}

static void
sfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct sfe_desc		*rdp;
	uint32_t		here;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct sfe_desc *)dp->rx_ring)[slot];

	ddi_put32(h, &rdp->d_cmdsts, CMDSTS_OWN);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

	/* make a link to this from the previous descriptor */
	here = ((uint32_t)dp->rx_ring_dma) + sizeof(struct sfe_desc)*slot;

	rdp = &((struct sfe_desc *)dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)];
	ddi_put32(h, &rdp->d_link, here);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);

	return;
}

static void
sfe_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct sfe_desc		*tdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tdp = &((struct sfe_desc *)dp->tx_ring)[slot];
	ddi_put32(h, &tdp->d_cmdsts, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);
}

static void
sfe_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct sfe_desc		*rdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct sfe_desc *)dp->rx_ring)[slot];
	ddi_put32(h, &rdp->d_cmdsts, CMDSTS_OWN);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct sfe_desc), DDI_DMA_SYNC_FORDEV);
}

/*
 * Device depend interrupt handler
 */
static u_int
sfe_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flags = 0;
	boolean_t	need_to_reset = FALSE;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	/* disable interrupts before clearing isr.*/
	OUTL(dp, IER, 0);

	/* read reason and clear interrupt */
	isr = INL(dp, ISR);

	if ((isr & lp->our_intr_bits) == 0) {
		/* not for us, enable interrupts again */
		OUTL(dp, IER, 1);
		return DDI_INTR_UNCLAIMED;
	}

	DPRINTF(3, (CE_CONT,
		CONS "%s: time:%d sfe_interrupt:called: "
		"isr:0x%b rx_desc_head: %d",
		dp->name, ddi_get_lbolt(), isr, INTR_BITS, dp->rx_desc_head));

	if (!dp->nic_active) {
		/* the device is going to stop */
		return DDI_INTR_CLAIMED;
	}

	isr &= lp->our_intr_bits;

	if ((isr & (ISR_RXORN | ISR_RXIDLE | ISR_RXERR |
		    ISR_RXDESC | ISR_RXOK)) != 0) {

		gem_receive(dp);

		if ((isr & ISR_RXIDLE) != 0) {
			cmn_err(CE_NOTE,
				"%s: rx buffer ran out: isr %b.",
				dp->name, isr, INTR_BITS);
			/*
			 * Ensure RXDP points the head of receive
			 * buffer list.
			 */
			OUTL(dp, RXDP, dp->rx_ring_dma +
				sizeof(struct sfe_desc) *
				SLOT(dp->rx_desc_head, RX_RING_SIZE));

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
		cmn_err(CE_WARN, "%s: ERROR interrupt: isr %b.",
			dp->name, isr, INTR_BITS);
		need_to_reset = TRUE;
	}

	if (need_to_reset) {
		mutex_enter(&dp->xmitlock);
		gem_restart_nic(dp, TRUE);
		mutex_exit(&dp->xmitlock);
		flags |= INTR_RESTART_TX;
	}

	/* enable interrupts again */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IER, 1);
	}

	DPRINTF(5, (CE_CONT, CONS "%s: sfe_interrupt: return: isr: %b",
	       dp->name, isr, INTR_BITS));

	return DDI_INTR_CLAIMED | flags;
}

/* ======================================================== */
/*
 * HW depend MII routine
 */
/* ======================================================== */

/*
 * MII routines for NS DP83815
 */
static void
sfe_mii_sync_dp83815(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
sfe_mii_read_dp83815(struct gem_dev *dp, uint_t offset)
{
	DPRINTF(4, (CE_CONT, CONS"%s: sfe_mii_read_dp83815: offset 0x%x",
	       dp->name, offset));
	return (uint16_t) INL(dp, MII_REGS_BASE + offset*4);
}

static void
sfe_mii_write_dp83815(struct gem_dev *dp, uint_t offset, uint16_t val)
{
	DPRINTF(4, (CE_CONT, CONS"%s: sfe_mii_write_dp83815: offset 0x%x 0x%x",
	       dp->name, offset, val));
	OUTL(dp, MII_REGS_BASE + offset*4, val);
}

static int
sfe_mii_init_dp83815(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	DPRINTF(3,(CE_CONT, CONS"%s: sfe_mii_init_dp83815: called", dp->name));

	dp->mii_phy_addr = -1;
	if (gem_mii_init_default(dp) < 0) {
		return -1;
	}

	/* workaround for 83815 internal PHY */
	dp->gc.gc_mii_linkdown_action = MII_ACTION_NONE;
	dp->gc.gc_mii_linkdown_timeout_action = MII_ACTION_NONE;
	dp->gc.gc_mii_dont_reset      = FALSE;

	return 0;
}

static int
sfe_mii_config_dp83815(struct gem_dev *dp)
{
	uint32_t	srr;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	srr = INL(dp, SRR) & SRR_REV;

	DPRINTF(0, (CE_CONT, CONS "%s: srr:0x%04x %04x %04x %04x %04x %04x",
		dp->name, srr,
		INW(dp, 0x00cc),	/* PGSEL */
		INW(dp, 0x00e4),	/* PMDCSR */
		INW(dp, 0x00fc),	/* TSTDAT */
		INW(dp, 0x00f4),	/* DSPCFG */
		INW(dp, 0x00f8))	/* SDCFG */
	);

	if (srr == SRR_REV_CVNG) {
		/*
		 * NS datasheet says that DP83815CVNG needs following
		 * registers to be patched for optimizing performance.
		 * A report said that CRC errors on RX was disappeared.
		 */
		OUTW(dp, 0x00cc, 0x0001);	/* PGSEL */
		OUTW(dp, 0x00e4, 0x189c);	/* PMDCSR */
		OUTW(dp, 0x00fc, 0x0000);	/* TSTDAT */
		OUTW(dp, 0x00f4, 0x5040);	/* DSPCFG */
		OUTW(dp, 0x00f8, 0x008c);	/* SDCFG */

		DPRINTF(0, (CE_CONT,
			CONS "%s: PHY patched %04x %04x %04x %04x %04x",
			dp->name,
			INW(dp, 0x00cc),	/* PGSEL */
			INW(dp, 0x00e4),	/* PMDCSR */
			INW(dp, 0x00fc),	/* TSTDAT */
			INW(dp, 0x00f4),	/* DSPCFG */
			INW(dp, 0x00f8))	/* SDCFG */
		);
	}

	return gem_mii_config_default(dp);
}


/*
 * MII routines for SiS900
 */
#define MDIO_DELAY(dp)    INL(dp, MEAR)
static void
sfe_mii_sync_sis900(struct gem_dev *dp)
{
	int	i;

	for (i = 0; i < 32; i++) {
		OUTL(dp, MEAR, MEAR_MDDIR | MEAR_MDIO);
		MDIO_DELAY(dp);
		OUTL(dp, MEAR, MEAR_MDDIR | MEAR_MDIO | MEAR_MDC);
		MDIO_DELAY(dp);
	}
}

static int
sfe_mii_init_sis900(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	if (gem_mii_init_default(dp) != 0) {
		return -1;
	}

	if ((dp->mii_phy_id & PHY_MASK) == PHY_SIS900_INTERNAL) {
		/* workaround for internal PHY */
		dp->gc.gc_mii_linkdown_action = MII_ACTION_RSA;
		dp->gc.gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		dp->gc.gc_mii_dont_reset      = FALSE;
	} 
	else {
		dp->gc.gc_mii_linkdown_action = MII_ACTION_NONE;
		dp->gc.gc_mii_linkdown_timeout_action = MII_ACTION_NONE;
		dp->gc.gc_mii_dont_reset      = FALSE;
	}

	return 0;
}

static int
sfe_mii_config_sis900(struct gem_dev *dp)
{
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	/* Do chip depend setup */
	if ((dp->mii_phy_id & PHY_MASK) == PHY_ICS1893) {
		/* workaround for ICS1893 PHY */ 
		GEM_MII_WRITE(dp, 0x0018, 0xD200);
	}
#ifdef i86pc
	if (lp->revid == SIS630E_900_REV) {
		/*
		 * SiS 630E has some bugs on default value
		 * of PHY registers
		 */
		GEM_MII_WRITE(dp, MII_AN_ADVERT, 0x05e1);
		GEM_MII_WRITE(dp, MII_CONFIG1, 0x0022);
		GEM_MII_WRITE(dp, MII_CONFIG2, 0xff00);
		GEM_MII_WRITE(dp, MII_MASK,    0xffc0);
	}
	sfe_set_eq_sis630(dp);
#endif
	return gem_mii_config_default(dp);
}

static uint16_t
sfe_mii_read_sis900_raw(struct gem_dev *dp, uint_t reg)
{
	uint32_t	cmd;
	uint16_t	ret;
	int		i;
	uint32_t	data;

	cmd = MII_READ_CMD(dp->mii_phy_addr, reg);

	sfe_mii_sync_sis900(dp);

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
		DPRINTF(0, (CE_WARN, "%s: PHY@%d not responded",
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

	return ret;
}

static uint16_t
sfe_mii_read_sis900(struct gem_dev *dp, uint_t reg)
{
	if (reg == MII_STATUS) {
		(void) sfe_mii_read_sis900_raw(dp, reg);
	}

	return sfe_mii_read_sis900_raw(dp, reg);
}

static void
sfe_mii_write_sis900(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint32_t	cmd;
	int		i;
	uint32_t	data;

	cmd = MII_WRITE_CMD(dp->mii_phy_addr, reg, val);

	sfe_mii_sync_sis900(dp);

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

#ifdef i86pc
static void
sfe_set_eq_sis630(struct gem_dev *dp)
{
	uint16_t	reg14h;
	uint16_t	eq_value;
	uint16_t	max_value;
	uint16_t	min_value;
	int		i;
	uint8_t		rev;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	rev = lp->revid;

	if (!(rev == SIS630E_900_REV || rev == SIS630EA1_900_REV ||
	      rev == SIS630A_900_REV || rev ==  SIS630ET_900_REV) ) {
		/* it dont have a internal PHY */
		return;
	}

#ifdef TUNE_PHY_630
	if (dp->mii_state == MII_STATE_LINKUP) {
		reg14h = GEM_MII_READ(dp, MII_RESV);
		GEM_MII_WRITE(dp, MII_RESV, (0x2200 | reg14h) & 0xBFFF);

		eq_value = (0x00f8 & GEM_MII_READ(dp, MII_RESV)) >> 3;
		max_value = min_value = eq_value;
		for (i = 1; i < 10; i++) {
			eq_value = (0x00f8 & GEM_MII_READ(dp, MII_RESV)) >> 3;
			max_value = max(eq_value, max_value);
			min_value = min(eq_value, min_value);
		}

		/* 630E rule to determine the equalizer value */
		if (rev == SIS630E_900_REV || rev == SIS630EA1_900_REV ||
		    rev == SIS630ET_900_REV) {
			if (max_value < 5) {
				eq_value = max_value;
			}
			else if (5 <= max_value && max_value < 15) {
				eq_value = max(max_value + 1, min_value + 2);
			}
			else if (15 <= max_value) {
				eq_value = max(max_value + 5, min_value + 6);
			}
		}
		/* 630B0&B1 rule to determine the equalizer value */
		else
		if (rev == SIS630A_900_REV && 
			(lp->bridge_revid == SIS630B0 ||
			 lp->bridge_revid == SIS630B1)) {

			if (max_value == 0) {
				eq_value = 3;
			}
			else {
				eq_value = (max_value + min_value + 1)/2;
			}
		}
		/* write equalizer value and setting */
		reg14h = GEM_MII_READ(dp, MII_RESV);
		reg14h = (reg14h & ~0x00f8) | ((eq_value << 3) & 0x00f8);
		reg14h = (reg14h | 0x6000) & ~0x0200;
		GEM_MII_WRITE(dp, MII_RESV, reg14h);
	}
	else {
		reg14h = GEM_MII_READ(dp, MII_RESV);
		if (rev== SIS630A_900_REV && 
			 (lp->bridge_revid == SIS630B0 ||
			  lp->bridge_revid == SIS630B1)) {

			GEM_MII_WRITE(dp, MII_RESV, (reg14h | 0x2200) & ~0x4000);
		}
		else {
			GEM_MII_WRITE(dp, MII_RESV, (reg14h | 0x2000) & ~0x4000);
		}
	}
#endif /* TUNE_PHY_630 */
	return;
}
#endif /* i86pc */
/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static void
sfe_chipinfo_init_sis900(struct gem_dev *dp)
{
	int		rev;
	struct sfe_dev	*lp = (struct sfe_dev *)dp->private;

	rev = lp->revid;
#ifdef i86pc
	if (rev == SIS630E_900_REV /* 0x81 */) {
		/* sis630E */
		lp->get_mac_addr = &sfe_get_mac_addr_sis630e;
	}
	else if (rev > 0x81 &&  rev <= 0x90) {
		/* 630S, 630EA1, 630ET, 635A */
		lp->get_mac_addr = &sfe_get_mac_addr_sis635;
	}
	else if (rev == SIS962_900_REV /* 0x91 */) {
		/* sis962 or later */
		lp->get_mac_addr = &sfe_get_mac_addr_sis962;
	} else
#endif
	{
		/* sis900 */
		lp->get_mac_addr = &sfe_get_mac_addr_sis900;
	}

#if defined(i86pc) && defined(TUNE_PHY_630)
	lp->bridge_revid = 0;

	if (rev == SIS630E_900_REV || rev == SIS630EA1_900_REV ||
	    rev == SIS630A_900_REV || rev ==  SIS630ET_900_REV) {
		/*
		 * read host bridge revision
		 */
		dev_info_t	*bridge;
		ddi_acc_handle_t bridge_handle;
		int		reg;

		if ((bridge = sfe_search_pci_dev(0x1039, 0x630) == NULL) {
			cmn_err(CE_WARN,
				"%s: cannot find host bridge (pci1039,630)",
				dp->name);
			goto x;
		}

		if (pci_config_setup(bridge, &bridge_handle) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "%s: pci_config_setup failed",
				dp->name);
			goto x;
		}

		lp->bridge_revid =
			pci_config_get8(bridge_handle, PCI_CONF_REVID);
		pci_config_teardown(bridge_handle);
	}
#endif /* i86pc && TUNE_PHY_630 */
x:
	return;
}

static int
sfe_attach_chip(struct gem_dev *dp)	
{
	struct sfe_dev		*lp = (struct sfe_dev *)dp->private;

	DPRINTF(4, (CE_CONT, CONS "%s: sfe_attach_chip called", dp->name));

	/* setup chip-depend get_mac_address function */
	if (lp->chip->chip_type == CHIPTYPE_SIS900) {
		sfe_chipinfo_init_sis900(dp);
	}
	else {
		lp->get_mac_addr = &sfe_get_mac_addr_dp83815;
	}

	/* read MAC address */
	if (!gem_get_mac_addr_conf(dp)) {
		if (!((lp->get_mac_addr)(dp))) {
			cmn_err(CE_WARN, 
				"%s: sfe_attach_chip: cannot get mac address",
				dp->name);
			gem_generate_macaddr(dp, dp->dev_addr.ether_addr_octet);
		}
	}

	return 0;
}

static int
sfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct chip_info	*p;
	struct gem_dev		*dp;
	struct sfe_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit     = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, CONS "%s%d: sfeattach: called", drv_name, unit));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "%s%d: ddi_regs_map_setup failed",
			drv_name, unit);
		goto err;
	}

	vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	rev  = pci_config_get16(conf_handle, PCI_CONF_REVID);
#ifdef DEBUG_LEVEL
	iline =	pci_config_get32(conf_handle, PCI_CONF_ILINE),
	latim = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
#endif
#if defined(i86pc) && defined(DEBUG_BUILT_IN_SIS900)
	rev  = SIS630E_900_REV;
	rev  = SIS630ET_900_REV;
#endif
	for (i = 0, p = sfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_WARN, 
		"%s%d: sfe_attach: wrong PCI venid/devid (0x%x, 0x%x)",
		drv_name, unit, vid, did);
	pci_config_teardown(&conf_handle);
	goto err;

chip_found:
	pci_config_put16(conf_handle, PCI_CONF_COMM,
		PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
			pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return gem_resume(dip);

	case DDI_ATTACH:

		DPRINTF(0, (CE_CONT,
			CONS "%s%d: ilr 0x%08x, latency_timer:0x%02x",
			drv_name, unit, iline, latim));
		/*
		 * Map in the device registers.
		 */

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
			PCI_ADDR_MEM32,
#else
			PCI_ADDR_IO,
#endif
			&sfe_dev_attr, (caddr_t *)&base, &regs_ha) !=
								DDI_SUCCESS) {
			cmn_err(CE_WARN, "%s%d: ddi_regs_map_setup failed",
				drv_name, unit);
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_desc_size = sizeof(struct sfe_desc) * TX_RING_SIZE;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_auto_pad  = TRUE;
		gcp->gc_tx_copy_thresh = sfe_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_size = sizeof(struct sfe_desc) * RX_RING_SIZE;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;
		gcp->gc_rx_max_descs_per_pkt = gcp->gc_rx_max_frags;
		gcp->gc_rx_copy_thresh = sfe_rx_copy_thresh;
		gcp->gc_rx_buf_max   = gcp->gc_rx_buf_size + 1;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, sfe_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, sfe_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, sfe_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, sfe_dma_attr_desc);

		STRUCT_COPY(gcp->gc_dma_attr_txbuf, sfe_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_align = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, sfe_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval   = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* setting for general PHY */
		gcp->gc_mii_an_delay	  =  0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset      = FALSE;


		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &sfe_attach_chip;
		if (p->chip_type == CHIPTYPE_DP83815) {
			gcp->gc_reset_chip = &sfe_reset_chip_dp83815;
		} else {
			gcp->gc_reset_chip = &sfe_reset_chip_sis900;
		}
		gcp->gc_init_chip  = &sfe_init_chip;
		gcp->gc_start_chip = &sfe_start_chip;
		gcp->gc_stop_chip  = &sfe_stop_chip;
#ifdef USE_MULTICAST_HASHTBL
		gcp->gc_multicast_hash = &sfe_mcast_hash;
#endif
		if (p->chip_type == CHIPTYPE_DP83815) {
			gcp->gc_set_rx_filter = &sfe_set_rx_filter_dp83815;
		} else {
			gcp->gc_set_rx_filter = &sfe_set_rx_filter_sis900;
		}
		gcp->gc_set_media = &sfe_set_media;
		gcp->gc_get_stats = &sfe_get_stats;
		gcp->gc_interrupt = &sfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &sfe_tx_desc_write;
		gcp->gc_rx_desc_write = &sfe_rx_desc_write;

		gcp->gc_tx_desc_stat = &sfe_tx_desc_stat;
		gcp->gc_rx_desc_stat = &sfe_rx_desc_stat;
		gcp->gc_tx_desc_init = &sfe_tx_desc_init;
		gcp->gc_rx_desc_init = &sfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &sfe_tx_desc_clean;
		gcp->gc_rx_desc_clean = &sfe_rx_desc_clean;

		/* mii operations */
		if (p->chip_type == CHIPTYPE_DP83815) {
			gcp->gc_mii_init  = &sfe_mii_init_dp83815;
			gcp->gc_mii_config= &sfe_mii_config_dp83815;
			gcp->gc_mii_sync  = &sfe_mii_sync_dp83815;
			gcp->gc_mii_read  = &sfe_mii_read_dp83815;
			gcp->gc_mii_write = &sfe_mii_write_dp83815;
			gcp->gc_mii_tune_phy = NULL;
			gcp->gc_flow_control = FLOW_CONTROL_NONE;
		}
		else {
			gcp->gc_mii_init  = &sfe_mii_init_sis900;
			gcp->gc_mii_config= &sfe_mii_config_sis900;
			gcp->gc_mii_sync  = &sfe_mii_sync_sis900;
			gcp->gc_mii_read  = &sfe_mii_read_sis900;
			gcp->gc_mii_write = &sfe_mii_write_sis900;
#ifdef i86pc
			gcp->gc_mii_tune_phy = &sfe_set_eq_sis630;
#endif
			gcp->gc_flow_control = FLOW_CONTROL_SYMMETRIC;
		}

		lp = (struct sfe_dev *)
			kmem_zalloc(sizeof(struct sfe_dev), KM_SLEEP);

		lp->chip  = p;
		lp->revid = rev;
		lp->cr    = 0;

		DPRINTF(0, (CE_CONT, CONS "%s%d: chip:%s rev:0x%02x",
			drv_name, unit, p->chip_name, rev));

		ddi_set_driver_private(dip, NULL);

		dp = gem_do_attach(dip, gcp, base, &regs_ha,
			lp, sizeof(*lp));
		kmem_free(gcp, sizeof(*gcp));

		if (dp == NULL) {
			goto err_freelp;
		}

		return DDI_SUCCESS;

err_freelp:
		kmem_free(lp, sizeof(struct sfe_dev));
err:
		return DDI_FAILURE;
	}
	return DDI_FAILURE;
}

static int
sfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	switch (cmd) {
	case DDI_SUSPEND:
		return  gem_suspend(dip);

	case DDI_DETACH:
		return  gem_do_detach(dip);
	}
	return DDI_FAILURE;
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info sfeminfo = {
	0,			/* mi_idnum */
	"sfe",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	TX_BUF_SIZE*ETHERMAX,	/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit sferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&sfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit sfewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&sfeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	sfe_info = {
	&sferinit,	/* st_rdinit */
	&sfewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_sfe_ops = {
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
	&sfe_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops sfe_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	sfeattach,	/* devo_attach */
	sfedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_sfe_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&sfe_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, CONS "sfe: _init: called"));
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

	DPRINTF(2, (CE_CONT, CONS "sfe: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
