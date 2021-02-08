/*
 * vfe_gem.c: VIA Technology RHINE seris Fast Ethernet MAC driver
 *
 * Copyright (c) 2002-2006 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)vfe_gem.c	1.1 06/05/21"

/*
 Change log
 0.9.0	11/23/2002
	first release of gem based vfe driver

 0.9.1  11/30/2002
	Added debugging code for rhine1 emulation on rhine2 chips.
	0.9.0 did not work for rhine1 chips because gem did not check
	tx buffer alignment. Gem was fixed.

 0.9.2  12/1/2002
	vfe_set_rx_filter(): multcast hash table was fixed
	Issue MII_RESET on attach

	02/16/2003
	struct gc cleared. gem_get_packet_default added.

	02/17/2003 vfe_get_stats fixed for rhine I.
 0.9.3  02/17/2003 release
 	04/12/2002 ddi_dma_sync added
 	04/26/2002 vfe_tx_desc_write was fixed for type mismactch.
 		   calling ddi_get/put8 in vfeattach fixed for type mismatch
		   multicast cam filtering added (not tested)
		   shape up source code
 0.9.4  04/27/2003 release
 0.9.5  05/18/2003 no interrupt implemented
 0.9.5  05/18/2003 release
 0.9.8
        11/10/2003 vfe_mii_init() removed
 1.0.0  11/10/2003 release
		   burst size tuning on tx underflow/rx overflow implemented.
 1.0.1  03/07/2004 release
		   TDWBI workaround implemented. Tx errors gone!
		   Licensing changed to the BSD license from LGPL.
 1.0.2  04/01/2004 release
		   a workaround for VT6103 implemented in vfe_mii_write.
 1.0.5  04/29/2004 release
		   tx interrput optimizatoin for rhine1
 1.0.7  05/29/2004 release
		   bus error while eeprom autoloading fixed.
		   RCR accessed in wrong size (long).
 1.0.8  06/06/2004 release
 1.0.12 08/09/2004 vfe_set_rx_filter fixed not to stop the nic core.
 1.0.18 04/12/2005 ddi_put/get are used for descriptor access
 1.0.25 10/31/2005 release
 	11/23/2005 fibre mode support for 6105M and 6103, but not tested.
 1.0.26 03/14/2006 release
 */

/*
 TODO:
	tuning on tx underflow.
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

#include <sys/pci.h>
#include "mii.h"
#include "gem.h"
#include "rhinereg.h"

char	ident[] = "via rhine nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int vfe_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (vfe_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)

#define	RH_WRITE_IMR(dp, imr)	{	\
	OUTW((dp), IMR, (imr));	\
	if (((struct vfe_dev *)dp->private)->revid > 0x40) {	\
		OUTB((dp), MIMR, (imr) >> 16);	\
	}	\
}

#define	RH_WRITE_ISR(dp, isr)	{	\
	OUTW((dp), ISR, (isr));	\
	if (((struct vfe_dev *)dp->private)->revid > 0x40) {	\
		OUTB((dp), MISR, (isr) >> 16);	\
	}	\
}

#define	RH_READ_ISR(dp, isr)	{	\
	isr = INW((dp), ISR);	\
	if (((struct vfe_dev *)dp->private)->revid > 0x40) {	\
		isr |= INB((dp), MISR) << 16;	\
	}	\
}

#define	RH_KICK_TX(dp, cr)	{	\
	if (IS_RHINE3M_OR_LATER(((struct vfe_dev *)(dp)->private)->revid)) { \
		OUTB((dp), TXQW, 1 << 7);	\
	}	\
	OUTB((dp), CR+1, ((CR_TDMD1 | (cr)) >> 8));	\
}

#ifdef MAP_MEM
# define	FLSHB(dp, reg)	INB(dp, reg)
# define	FLSHW(dp, reg)	INW(dp, reg)
# define	FLSHL(dp, reg)	INL(dp, reg)
#else
# define	FLSHB(dp, reg)
# define	FLSHW(dp, reg)
# define	FLSHL(dp, reg)
#endif

#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

/*
 * Our configuration
 */
#define	OUR_INTR_BITS	\
	(ISR_ABTI | ISR_NORBF | ISR_PKRACE | ISR_OVFI |	\
	 ISR_CNT | ISR_BE | ISR_RU | ISR_TU |	\
	 ISR_RXE | ISR_TXE | 	\
	 ISR_PTX | ISR_PRX)

#ifdef TEST_RX_EMPTY
# define RX_BUF_SIZE	1
#endif
#ifdef TEST_TXDESC_FULL
# define TX_BUF_SIZE	4
# define TX_RING_SIZE	8
#endif

#ifndef TX_BUF_SIZE
# define TX_BUF_SIZE	64
#endif
#ifndef TX_RING_SIZE
# define TX_RING_SIZE	(TX_BUF_SIZE * 4)
#endif

#ifndef RX_BUF_SIZE
# define RX_BUF_SIZE	64
#endif
#ifndef RX_RING_SIZE
# define RX_RING_SIZE	RX_BUF_SIZE
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
	volatile uint32_t	imr;
	volatile uint32_t	imr_hw;
	boolean_t	td_unreliable;
#ifdef RESET_TEST
	int		reset_test;
#endif
#ifdef OPT_TX_INTR
	ddi_softintr_t	soft_id;
	boolean_t	imr_change_req;
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
	{VID_VIA, DID_VT3043, "DL10030/VT3043 Rhine"},
	{VID_VIA, DID_VT6102, "VT6102 Rhine-II"},
	{VID_VIA, DID_VT6105, "VT6105 Rhine-III"},
	{VID_VIA, DID_VT6105M, "VT6105M Rhine-III"},
};
#define CHIPTABLESIZE   (sizeof(vfe_chiptbl)/sizeof(struct chip_info))

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
#define	IS_RHINE3M_OR_LATER(r)		(0x90 <= (r))

#define	IS_VT6103(phyid)	\
	(((phyid) & 0xfffffff0) == 0x01018f20 && ((phyid) & 0xf) > 4)
/* ======================================================== */
 
/* mii operations */
static void  vfe_mii_sync(struct gem_dev *);
static uint16_t  vfe_mii_read(struct gem_dev *, uint_t);
static void vfe_mii_write(struct gem_dev *, uint_t, uint16_t);
static int vfe_mii_init(struct gem_dev *);

/* nic operations */
static int vfe_attach_chip(struct gem_dev *);
static int vfe_reset_chip(struct gem_dev *);
static void vfe_init_chip(struct gem_dev *);
static void vfe_start_chip(struct gem_dev *);
static int vfe_stop_chip(struct gem_dev *);
static void vfe_set_media(struct gem_dev *);
static void vfe_set_rx_filter(struct gem_dev *);
static void vfe_get_stats(struct gem_dev *);

/* descriptor operations */
static int vfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t flags);
static int vfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t vfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t vfe_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void vfe_tx_desc_init(struct gem_dev *dp, int slot);
static void vfe_rx_desc_init(struct gem_dev *dp, int slot);
static void vfe_tx_desc_clean(struct gem_dev *dp, int slot);
static void vfe_rx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int vfe_interrupt(struct gem_dev *dp);

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
	0xffffffffull,		/* dma_attr_count_max */
	0,/* patched later */	/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0,/* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t vfe_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
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
static void
vfe_stop_mauto(struct gem_dev *dp)
{
	int		i;
	int		ret;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	/*
	 * Initialize MII logic
	 */
	OUTB(dp, MIICR, 0);

	/* wait for midle */
	if (IS_RHINE1_VT86C100A_E(lp->revid)) {
		/*
		 * turn off MSRCEN
		 */
		OUTB(dp, MIIADR, 1);
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
	}
	else {
		for (i = 0; (INB(dp, MIIADR) & MIIADR_MIDLE) == 0; i++) {
			if (i > 1000) {
				break;
			}
			drv_usecwait(10);
		}
	}
	DPRINTF(1, (CE_CONT, "!%s: mauto stopped in %d uS", __func__, i*10));
}

static int
vfe_reset_chip(struct gem_dev *dp)
{
	int		i;
	int		ret;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_reset_chip: called, time:%d, cr:%b",
		dp->name, ddi_get_lbolt(),
		lp->cr, CR_BITS));

	ret = GEM_SUCCESS;
#ifdef notdef /* DEBUG_LEVEL */
	/* white a dummy mac address to ensure eeprom autoloading works */
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(dp, PAR + i, i);
	}
#endif
	/* invalidate a software cache of current mac address */
	bzero(lp->mac_addr, sizeof(lp->mac_addr));

	/* clear interrupt mask */
	RH_WRITE_IMR(dp, 0);

	/* Reset the chip. */
	lp->cr = 0;
	OUTB(dp, CR + 1, (CR_SRST >> 8) | INB(dp, CR + 1));
	drv_usecwait(10);

	if (IS_RHINE1(lp->revid)) {
		/* extra wait time for Rhine-I chips */
		drv_usecwait(100);
	}

	for (i = 0; (INB(dp, CR + 1) & (CR_SRST >> 8)) != 0; i++) {
		if (i > 1000) {
			cmn_err(CE_WARN, "!%s: vfe_reset_chip: timeout",
				dp->name);

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

	drv_usecwait(5000);

	/* clear interrupt mask again */
	RH_WRITE_IMR(dp, 0);

	vfe_stop_mauto(dp);

	return (ret);
}


static void
vfe_init_chip(struct gem_dev *dp)
{
	int		i;
	u_int		val;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_init_chip: called at time:%d",
		dp->name, ddi_get_lbolt()));

	/* PAR: do nothing */

	/* RCR: disable all rx filter */
	OUTB(dp, RCR, lp->rft << RCR_RRFT_SHIFT);

	/* TCR */
	OUTB(dp, TCR, (lp->tft << TCR_RTFT_SHIFT) | TCR_LB_NORMAL | TCR_OFSET);

	/* CR: don't touch  */

	/* ISR: don't touch */

	/* IMR & MIMR */
	lp->imr = 0;
	lp->imr_hw = lp->imr;
	RH_WRITE_IMR(dp, lp->imr_hw);

	/* Curr Rx Desc Addr */
	/* This seems to reset rx desc pointer. */
	OUTL(dp, CRDA, (uint32_t)dp->rx_ring_dma);

	/* Curr Tx Desc Addr */
	/* This seems to reset tx desc pointer. */
	OUTL(dp, CTDA, (uint32_t)dp->tx_ring_dma);
	if (IS_RHINE3M_OR_LATER(lp->revid)) {
		/* It has 8 transmit queues, but we use only the first one. */
		for (i = 1; i < 8; i++) {
			OUTL(dp, CTDA + i*4,
				((uint32_t)dp->tx_ring_dma) +
				TX_RING_SIZE*sizeof(struct tx_desc)*i);
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
	}
	else {
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
}

static uint_t
vfe_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* hash key is most significant 6 bits of crc in big endian manner */
	return gem_ether_crc_be(addr) >> (32 - 6);
}

static void
vfe_set_rx_filter(struct gem_dev *dp)	
{
	uint32_t	mode;
	int		i, j;
	uint32_t	mhash[2];
	uint8_t		*mac;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_set_rx_filter: called at time:%d, "
		"active:%d, rxmode:%b, cr:%b",
		dp->name, ddi_get_lbolt(),
		dp->nic_active, dp->rxmode, "\020\002AllMulti\001Promisc",
		lp->cr, CR_BITS));

	mode = RCR_AB;	/* broadcast */
	mhash[0] = mhash[1] = 0;
	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		mode |=RCR_PRO	/* promiscous */
                     | RCR_AM;	/* muticast */
		mhash[0] = mhash[1] = 0xffffffff;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 ||
				dp->mc_count > MULTICAST_CAM_SIZE) {
		mode |= RCR_AM;	/* multicast */
		mhash[0] = mhash[1] = 0xffffffff;
	}
	else if (IS_RHINE3M_OR_LATER(lp->revid) && dp->mc_count > 0) {
		/*
		 * use multicast cam. The multicast hash table should be 0.
		 */
		mode |= RCR_AM;	/* enable multicast reception */
	}
	else if (dp->mc_count > 0) {
		mode |= RCR_AM;	/* enable multicast reception */

		/* make a hash table */
		for (i = 0; i < dp->mc_count; i++) {
			j = dp->mc_list[i].hash;
			mhash[j / 32] |= 1 << (j % 32);
		}
	}

	mac = &dp->cur_addr.ether_addr_octet[0];

	if (bcmp(lp->mac_addr, mac, ETHERADDRL) != 0) {
		/*
		 * XXX - as we cannot resume rx w/o resetting, don't stop rx.
		 */
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, lp->mac_addr[i] = mac[i]);
		}
	}

	if (IS_RHINE3M_OR_LATER(lp->revid)) {
		uint32_t	cammask;
		/*
		 * VT6105M has 32 entry cams for multicast filtering
		 */
		cammask = 0;

		/* enable cam port for writing multicast address*/
		OUTB(dp, CAMCTRL, CAMCTRL_EN);

		if (dp->mc_count <= MULTICAST_CAM_SIZE) {
			for (i = 0; i < dp->mc_count; i++) {
				/* write cam index */
				OUTB(dp, CAMADDR, i);

				/* write a multicast address into cam */
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

		/* initialize VLAN cam mask */
		OUTB(dp, CAMCTRL, CAMCTRL_EN | CAMCTRL_VLAN);
		OUTL(dp, CAMMASK, 0);
		OUTB(dp, CAMCTRL, 0);
	}

	if ((mode & RCR_AM) != 0) {
		/* need to set multicast hash table */
		for (i = 0; i < 2; i++) {
			OUTL(dp, MAR + i*4, mhash[i]);
		}
	}

	/* update new rx filter mode */
	OUTB(dp, RCR, (lp->rft << RCR_RRFT_SHIFT) | mode | RCR_SEP | RCR_AR);
}

static void
vfe_set_media(struct gem_dev *dp)
{
	u_int		val;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_set_media: called at time:%d, "
		"active:%d, cr:%b, misc:%04x",
		dp->name, ddi_get_lbolt(),
		dp->nic_active, lp->cr, CR_BITS, INW(dp, MISC)));

	/*
	 * Notify current duplex mode to mac
	 */
	/* XXX - don't stop the nic, it isn't recoverble. */
	if (dp->full_duplex) {
		lp->cr |= CR_FDX;
	}
	else {
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
#ifdef NEVER
		case FLOW_CONTROL_NONE:
			if (!dp->full_duplex) {
				/* XXX - don't use half duplex flow control */
				val |= MISC_HDXFEN;
			}
			break;
#endif
		}
		OUTW(dp, MISC, val);
	}	
	else if (IS_RHINE3_OR_LATER(lp->revid)) {
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
#ifdef NEVER
		case FLOW_CONTROL_NONE:
			if (!dp->full_duplex) {
				/* XXX - don't use half duplex flow control */
				val |=  FCR1_HDFC_EN;
			}
			break;
#endif
		}
		OUTB(dp, FCR1, val);
	}
}

static void
vfe_start_chip(struct gem_dev *dp)
{
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_start_chip: called at time:%d",
		dp->name, ddi_get_lbolt()));

	/* enable interrupt */
	lp->imr =
#ifdef OPT_TX_INTR
		  IS_RHINE1(lp->revid) ? (OUR_INTR_BITS & ~ISR_PTX) :
#endif
		  OUR_INTR_BITS;
	if (lp->revid > 0x40 && lp->revid < 0x90) {
		lp->imr |= MISR_TDWBI << 16;
	}

	lp->imr_hw = lp->imr;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		RH_WRITE_IMR(dp, lp->imr_hw);
	}

	/* Kick TX and RX */
	if (IS_RHINE3_OR_LATER(lp->revid)) {
		OUTB(dp, FCR0, RX_BUF_SIZE);
	}
	lp->cr |= CR_TXON | CR_RXON | CR_DPOLL | CR_STRT;
	OUTW(dp, CR, lp->cr);
}

static int
vfe_stop_chip(struct gem_dev *dp)
{
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_stop_chip: called at time:%d",
		dp->name, ddi_get_lbolt()));

	/* Disable interrupts by clearing the interrupt mask */
	lp->imr = 0;
	lp->imr_hw = lp->imr;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		RH_WRITE_IMR(dp, lp->imr_hw);
	}

	/* Stop Tx and Rx processes in the chip. */
	lp->cr &= ~(CR_TXON | CR_RXON | CR_DPOLL | CR_STRT);
	OUTW(dp, CR, lp->cr | CR_STOP);

	/* Note: No need to call vfe_reset_chip() to reset Tx/Rx desc pointers */
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

#define	RH_EEPROM_DELAY(dp)	{INB(dp, EECSR); INB(dp, EECSR);}
#define	EE93C46_READ	6

	/* enable eeprom direct programing */
	cfga_saved = INB(dp, CFGA);
	OUTB(dp, CFGA, cfga_saved | CFGA_EELOAD);
	RH_EEPROM_DELAY(dp);

	/* ensure de-assert chip select */
	chip_select = EECSR_DPM;
	OUTB(dp, EECSR, chip_select);
	RH_EEPROM_DELAY(dp);

	/* assert chip select */
	chip_select |= EECSR_ECS;
	OUTB(dp, EECSR, chip_select);
	RH_EEPROM_DELAY(dp);

	/* make a read command for eeprom */
	offset = (offset & 0x3f) | (EE93C46_READ << 6);

	for (i = 10; i >= 0; i--) {
		/* send 1 bit */
		di = ((offset >> i) & 1) << EECSR_EDI_SHIFT;

		OUTB(dp, EECSR, chip_select | di);
		RH_EEPROM_DELAY(dp);

		OUTB(dp, EECSR, chip_select | di | EECSR_ECK);
		RH_EEPROM_DELAY(dp);
	}

	OUTB(dp, EECSR, chip_select);
	RH_EEPROM_DELAY(dp);

	/* get the reply and construct a 16bit value */
	ret = 0;
	for (i = 0; i < 16; i++) {
		/* Get 1 bit */
		OUTB(dp, EECSR, chip_select | EECSR_ECK);
		RH_EEPROM_DELAY(dp);

		ret = (ret << 1)
		    | ((INB(dp, EECSR) >> EECSR_EDO_SHIFT) & 1);

		OUTB(dp, EECSR, chip_select);
		RH_EEPROM_DELAY(dp);
	}

	/* negate chip_select */
	OUTB(dp, EECSR, EECSR_DPM);
	RH_EEPROM_DELAY(dp);

	OUTB(dp, EECSR, 0);
	RH_EEPROM_DELAY(dp);

	/* disable eeprom direct programming */
	OUTB(dp, CFGA, cfga_saved);
	RH_EEPROM_DELAY(dp);

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
	uint_t		val;
	uint8_t		*mac;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: vfe_attach_chip: called at time:%d",
		dp->name, ddi_get_lbolt()));

	mac = dp->dev_addr.ether_addr_octet;

	if (gem_get_mac_addr_conf(dp)) {
		/* the mac address has been specified by vfe.conf */
		for (i = 0; i < ETHERADDRL; i++) {
			OUTB(dp, PAR + i, mac[i]);
		}
	}
	else {
		/*
		 * XXX - Don't read a mac address from eeprom, it doesn't
		 * work for built-in rhine II core in VT8235 south bridge.
		 * We can read a factory mac address from PAR reg after
		 * the eeprom contents are auto-loaded.
		 */
		for (i = 0; i < ETHERADDRL; i++) {
			mac[i]= INB(dp, PAR + i);
		}
	}

	/* initialize soft copy of mac address registers */
	bcopy(mac, lp->mac_addr, ETHERADDRL);

#ifdef DEBUG_LEVEL
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
	/* XXX - smaller tx threshold will cause performance issue. */
	if (IS_RHINE1(lp->revid)) {
		dp->txthr = max(dp->txthr, ETHERMAX); /* vfe-1.0.0 default */
	}
	else if (IS_RHINE2(lp->revid)) {
		dp->txthr = max(dp->txthr, 512);
	}

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
	dp->txmaxdma = max(128, dp->txmaxdma);

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

#if 0
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
	tim = (ddi_get_lbolt() - tim);

	if (i > 0) {
		cmn_err(CE_CONT, "!%s: soft_timer0 resolution in 10mS: %d",
			dp->name, tim > 0 ? TEST_COUNT/tim: 9999999);
	}
	/* OUTL(dp, CR_CLR, CR_TM0EN); */
	
}
#endif
	return (GEM_SUCCESS);
}

static void
vfe_get_stats(struct gem_dev *dp)
{
	volatile int	x;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	if (IS_RHINE2_OR_LATER(lp->revid)) {
		dp->stats.missed += INW(dp, MPAC);
	}
	else {
		/* they are unreliable on rhine I */
		x = INW(dp, MPAC);
	}
	x = INW(dp, CRCC);
	OUTW(dp, MPAC, 0);
}

/*
 * tx/rx discriptor manupirations
 */
#ifdef DEBUG_LEVEL
static int vfe_tx_frags[GEM_MAXTXFRAGS];
#endif
static int
vfe_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t flags)
{
	int			i;
	struct tx_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	struct vfe_dev		*lp = (struct vfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	"!%s: vfe_tx_desc_write time: %d, seqnum: %d, slot %d, frags: %d flags: %d",
		dp->name, ddi_get_lbolt(), dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
			i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 3
	flags |= GEM_TXFLAG_INTR;
#endif
#ifdef DEBUG_LEVEL
	vfe_tx_frags[min(frags, GEM_MAXTXFRAGS) - 1]++;
#endif
	/*
	 * To avoid condition race, enabme PTx interrupt before making
	 * descriptor(s) for the packet
	 */
#ifdef OPT_TX_INTR
	if ((dp->misc_flag & GEM_NOINTR) == 0 && IS_RHINE1(lp->revid) &&
	    (flags & GEM_TXFLAG_INTR) != 0 && (lp->imr & ISR_PTX) == 0) {
		/*
		 * enable PTX interrupt for the time being
		 */
		lp->imr |= ISR_PTX;
#if 0
		if (mutex_tryenter(&dp->intrlock)) {
			if (lp->imr_hw != 0) {
				lp->imr_hw = lp->imr;
				RH_WRITE_IMR(dp, lp->imr_hw);
				mutex_exit(&dp->intrlock);
			}
		} else
#endif
		{
			lp->imr_change_req = B_TRUE;
			ddi_trigger_softintr(lp->soft_id);
		}
	}
#endif /* OPT_TX_INTR */
	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TDES1_EDP | TDES1_C | ((flags & GEM_TXFLAG_INTR) ? TDES1_IC : 0);
	for (i = frags - 1, dcp = &dmacookie[frags - 1]; i > 0; i--, dcp--) {

		tdp = &((struct tx_desc *)dp->tx_ring)[
				SLOT(slot + i, TX_RING_SIZE)];
		ddi_put32(h, &tdp->td_length, dcp->dmac_size | mark);
		ddi_put32(h, &tdp->td_addr, dcp->dmac_address);
		ddi_put32(h, &tdp->td_csr, 0);
		mark = TDES1_C;

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);
	}

	/* for the first fragment */
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];
	ddi_put32(h, &tdp->td_length, dcp->dmac_size | mark | TDES1_STP);
	ddi_put32(h, &tdp->td_addr, dcp->dmac_address);

	if ((flags & GEM_TXFLAG_INTR) == 0 && IS_RHINE2_OR_LATER(lp->revid)) {
		/* set suppress-interrupt request for the packet */
		ddi_put32(h, &tdp->td_next,
			ddi_get32(h, &tdp->td_next) | TDES3_TDCTL);
	}
	ddi_put32(h, &tdp->td_csr, TDES0_OWN);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);

	/* kick Tx engine */
#ifdef TXTIMEOUT_TEST
	if ((vfe_send_cnt++ % 100) == 99) {
		lp->cr &= ~CR_TXON;
		OUTW(dp, CR, lp->cr);
	}
#endif
	RH_KICK_TX(dp, lp->cr);

	return frags;
}

static int
vfe_rx_desc_write(struct gem_dev *dp, uint_t slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rx_desc		*rdp;
	int			i;
	struct vfe_dev		*lp = (struct vfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"!%s: vfe_rx_desc_write seqnum: %d, slot %d, frags: %d",
		dp->name, dp->rx_desc_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
			i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write a RX descriptor
	 */
	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	ddi_put32(h, &rdp->rd_addr, dmacookie->dmac_address);
	ddi_put32(h, &rdp->rd_length, dmacookie->dmac_size);
	ddi_put32(h, &rdp->rd_status, RDES0_OWN);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);

	if (IS_RHINE3_OR_LATER(lp->revid) && dp->nic_active) {
		/* Tx flow control support for rhine III */
		OUTB(dp, FCR0, 1);
	}

	return frags;
}

static void
vfe_tx_desc_dump(struct gem_dev *dp, seqnum_t head, int ndesc)
{
	int			i;
	struct tx_desc		*tdp;
	struct vfe_dev		*lp = (struct vfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	cmn_err(CE_CONT, "!%s: tx_desc_dump:  slot:%d, seqnum:%d",
		dp->name,SLOT(head, TX_RING_SIZE), head);

	for (i = 0; i < ndesc; i++) {
		tdp = &((struct tx_desc *)dp->tx_ring)[
				SLOT(head + i, TX_RING_SIZE)];

		ddi_dma_sync(dp->desc_dma_handle,
			(off_t)(((caddr_t)tdp) - dp->rx_ring),
			sizeof(struct tx_desc), DDI_DMA_SYNC_FORKERNEL);

		cmn_err(CE_CONT, "! %d: %b, %b, 0x%08x",
			i, ddi_get32(h, &tdp->td_csr), TSR_BITS,
			ddi_get32(h, &tdp->td_length), TDES1_BITS,
			ddi_get32(h, &tdp->td_addr));
	}
}

static uint_t
vfe_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct tx_desc		*tdp;
	uint32_t		csr;
	uint_t			ret;
	int			cols;
	int			i;
	struct vfe_dev		*lp = (struct vfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	/*
	 * check the first descriptor of the packet
	 */
	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORKERNEL);

	csr = ddi_get32(h, &tdp->td_csr);

#if DEBUG_LEVEL > 2
	if (ndesc > 0) {
		int		i;
		uint32_t	isr;

		RH_READ_ISR(dp, isr);
		cmn_err(CE_CONT,
			"!%s: vfe_tx_desc_stat: time: %d "
			"slot:%d ndesc:%d isr:%b misr:%b",
			dp->name, ddi_get_lbolt(), slot, ndesc,
			isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS);
		for (i = 0; i < ndesc; i++) {
			struct tx_desc	*tp;

			tp = &((struct tx_desc *)dp->tx_ring)[
						SLOT(slot + i, TX_RING_SIZE)];
			cmn_err(CE_CONT, "!%d: tsr: %b txdesc1: %b",
			    i,
			    ddi_get32(h, &tp->td_csr), TSR_BITS,
			    ddi_get32(h, &tp->td_length), TDES1_BITS);
		}
	}
#endif
	if ((csr & TDES0_OWN) != 0) {
		/* not transmitted yet */
		return (0);
	}

	ret = GEM_TX_DONE;

	if (lp->td_unreliable) {
		goto x;
	}

#define	TSR_ERRS	\
	(TSR_SERR | TSR_TBUFF | TSR_UDF | TSR_CRS | TSR_OWC | TSR_ABT | TSR_CDH)
	if ((csr & TSR_TXERR) == 0) {
		if (!dp->full_duplex && (cols = (csr & TSR_NCR)) > 0) {
			dp->stats.collisions += cols;
			if (cols == 1) {
				dp->stats.first_coll++;
			}
			else /* if (cols > 1)*/ {
				dp->stats.multi_coll++;
			}
		}
		/* short cut for normal case */
		goto x;
	}

	DPRINTF(2, (CE_CONT, "!%s: tx error: tsr: %b  desc:%d",
		dp->name, csr, TSR_BITS, dp->tx_desc_head));

	dp->stats.errxmt++;

	if ((csr & TSR_CRS) != 0) {
		dp->stats.nocarrier++;
	}
	if ((csr & TSR_OWC) != 0) {
		dp->stats.xmtlatecoll++;
	}
	if ((csr & TSR_UDF) != 0) {
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
		ndesc = 0;
		ret |= GEM_TX_ERR;
	}
	if ((csr & TSR_ABT) != 0) {
		if (!dp->full_duplex) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		}
#ifdef notdef
		vfe_tx_desc_dump(dp, dp->tx_desc_head, ndesc);
#endif
		/* transmitter halted, drop the packet */
		ret |= GEM_TX_ERR;
	}

	if ((ret & GEM_TX_ERR) == 0) {
		/* no need to recover from a fatal error */
		goto x;
	}

	/*
	 * Try to recover the transmitter state without resetting the chip.
	 */
	/* wait until the transmitter becomes idle */
	for (i = 0; (INW(dp, CR) & CR_TXON) != 0; i++) {
		if (i > 10) {
			cmn_err(CE_WARN,
				"!%s: %s: timeout: stopping tx",
				dp->name, __func__);
			goto x;
		}
		drv_usecwait(10);
	}
	DPRINTF(2, (CE_CONT, "!%s: tx stopped in %d uS", dp->name, i*10));

	/* fix Current Tx Desctriptor Address */
	OUTL(dp, CTDA, ((uint32_t)dp->tx_ring_dma) +
		    SLOT(slot + ndesc, TX_RING_SIZE) * sizeof(struct tx_desc));

	/* fix the next actions to do. */
	ret = (ndesc > 0) ? GEM_TX_DONE : 0;

	/* enable the transmitter again. */
	OUTW(dp, CR, lp->cr);

	/* kick tx polling */
	RH_KICK_TX(dp, lp->cr);
x:
	/* reset supress-intr bit */
	if (ret != 0) {
		/*
		 * As tx was finished, update the interrupt mask.
		 */
		if (IS_RHINE1(lp->revid)) {
#ifdef OPT_TX_INTR
			if (dp->tx_desc_intr == dp->tx_desc_head + ndesc &&
			   (lp->imr & ISR_PTX) != 0) {
				/*
				 * XXX - Hardware IMR will be updated later.
				 */
				lp->imr &= ~ISR_PTX;
			}
#endif
		}
		else {
			/*
			 * For Rhine II or later:
			 * Ensure that there is no suppress interrupt request
			 * in TD3.
			 */
			ddi_put32(h, &tdp->td_next,
				ddi_get32(h, &tdp->td_next) & TDES3_NEXT);
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
	struct vfe_dev		*lp = (struct vfe_dev *)dp->private;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORKERNEL);

	rsr = ddi_get32(h, &rdp->rd_status);
	len = (rsr >> RDES0_LEN_SHIFT) & RDES0_LEN_MASK;
	ret = GEM_RX_DONE;

	DPRINTF(2, (CE_CONT,
		"!%s: vfe_rx_desc_stat: slot:%d dma_addr:0x%x len:%d rsr 0x%b",
		dp->name, slot,
		ddi_get32(h, &rdp->rd_addr),
		ddi_get32(h, &rdp->rd_length),
		ddi_get32(h, &rdp->rd_status), RSR_BITS));

	if ((rsr & RDES0_OWN) != 0) {
		/* it isn't received yet */
		return (0);
	}

	if ((rsr & (RSR_STP | RSR_EDP)) != (RSR_STP | RSR_EDP)) {
		/* big packet, ignore this fragment */
		if ((rsr & RSR_STP) != 0) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
		}
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	if ((rsr & RSR_RXOK) == 0) {
		/* errored packet */
		dp->stats.errrcv++;
		if ((rsr & RSR_LONG) != 0) {
			dp->stats.frame_too_long++;
		}
		if ((rsr & RSR_RUNT) != 0) {
			dp->stats.runt++;
			DPRINTF(2, (CE_CONT,
		"!%s: vfe_rx_desc_stat: slot:%d dma_addr:0x%x len:%d rsr 0x%b",
				dp->name, slot,
				ddi_get32(h, &rdp->rd_addr),
				ddi_get32(h, &rdp->rd_length),
				ddi_get32(h, &rdp->rd_status), RSR_BITS));
		}
		if ((rsr & (RSR_SERR | RSR_BUFF)) != 0) {
			dp->stats.overflow++;
		}
		if ((rsr & RSR_FAE) != 0) {
			dp->stats.frame++;
		}
		if ((rsr & RSR_CRC) != 0) {
			dp->stats.crc++;
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
	struct tx_desc		*tdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &tdp->td_csr, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);

	/* link to previous descriptor */
	tdp = &((struct tx_desc *)dp->tx_ring)[SLOT(slot - 1, TX_RING_SIZE)];
	ddi_put32(h, &tdp->td_next,
		((uint32_t)dp->tx_ring_dma) + slot*sizeof(struct tx_desc));

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);
}

static void
vfe_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct rx_desc		*rdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &rdp->rd_status, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);

	/* link this to its previous descriptor */
	rdp = &((struct rx_desc *)dp->rx_ring)[SLOT(slot - 1, RX_RING_SIZE)];
	ddi_put32(h, &rdp->rd_next,
		((uint32_t)dp->rx_ring_dma) + slot*sizeof(struct rx_desc));

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);
}

static void
vfe_tx_desc_clean(struct gem_dev *dp, int slot)
{
	struct tx_desc		*tdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	tdp = &((struct tx_desc *)dp->tx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &tdp->td_csr, 0);
	ddi_put32(h, &tdp->td_next, ddi_get32(h, &tdp->td_next) & TDES3_NEXT);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)tdp) - dp->rx_ring),
		sizeof(struct tx_desc), DDI_DMA_SYNC_FORDEV);
}

static void
vfe_rx_desc_clean(struct gem_dev *dp, int slot)
{
	struct rx_desc		*rdp;
	ddi_acc_handle_t	h = dp->desc_acc_handle;

	rdp = &((struct rx_desc *)dp->rx_ring)[slot];

	/* invalidate this descriptor */
	ddi_put32(h, &rdp->rd_status, 0);

	ddi_dma_sync(dp->desc_dma_handle,
		(off_t)(((caddr_t)rdp) - dp->rx_ring),
		sizeof(struct rx_desc), DDI_DMA_SYNC_FORDEV);
}

/*
 * Device depend interrupt handler
 */
#ifdef OPT_TX_INTR
static
uint_t vfe_update_imr(struct gem_dev *dp)
{
	uint_t		ret;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	ret = DDI_INTR_UNCLAIMED;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	if (lp->imr_change_req) {
		if ((dp->misc_flag & GEM_NOINTR) == 0 && lp->imr_hw != 0 &&
		     lp->imr_hw != lp->imr) {
			lp->imr_hw = lp->imr;
			RH_WRITE_IMR(dp, lp->imr_hw);
		}
		lp->imr_change_req = B_FALSE;
		ret = DDI_INTR_CLAIMED;
	}

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return (ret);
}
#endif /* OPT_TX_INTR */

static u_int
vfe_interrupt(struct gem_dev *dp)
{
	uint32_t	isr;
	uint_t		flag = 0;
	boolean_t	need_to_reset = B_FALSE;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	RH_READ_ISR(dp, isr);

	if ((isr & lp->imr_hw) == 0) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT, "!%s: Interrupt, isr: %b, misr: %b",
		dp->name, isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS));

	/* disable interrupt */
	RH_WRITE_IMR(dp, 0);

	if (!dp->nic_active) {
		/* the device is not active, no more interrupts */
		lp->imr = 0;
		lp->imr_hw = lp->imr;
		RH_WRITE_ISR(dp, isr);

		return (DDI_INTR_CLAIMED);
	}
#ifdef RESET_TEST
	lp->reset_test++;
	if ((lp->reset_test % 10000) == 9999) {
		need_to_reset = B_TRUE;
	}
#endif

	/* clear interrupts */
	RH_WRITE_ISR(dp, isr); FLSHW(dp, ISR);

	isr &= lp->imr_hw;

	if ((isr & ISR_CNT) != 0) {
		/* statics counter overflow */
		vfe_get_stats(dp);
	}

	if (isr & (ISR_PRX | ISR_RXE | ISR_PKRACE | ISR_RU | ISR_NORBF)) {

		/* packet was received, or receive error happened */
		/* RU: receive buffer unavailable */

		(void) gem_receive(dp);
		if ((isr & (ISR_RU | ISR_NORBF)) != 0) {
			dp->stats.norcvbuf++;
		}
	}

	if ((isr & ISR_OVFI) != 0) {
		/*
		 * receive FIFO overflow
		 */
		dp->stats.overflow++;
	}

	if ((isr & ISR_BE) != 0) {
		cmn_err(CE_WARN, "!%s: unexpected interrupt: isr:%b misr:%b",
			dp->name, isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS);
		need_to_reset = B_TRUE;
		goto x;
	}

	if ((isr & (MISR_TDWBI << 16)) != 0) {
		int	cnt;

		DPRINTF(2, (CE_WARN, "!%s: TDWBI interrupt: isr:%b misr:%b",
			dp->name, isr & 0xffff, ISR_BITS,
			isr >> 16, MISR_BITS));

		mutex_enter(&dp->xmitlock);

		/* XXX - stop Tx */
		for (cnt = 0; (INW(dp, CR) & CR_TXON) != 0; cnt++) {
			if (cnt == 10) {
				break;
			}
			drv_usecwait(10);
		}

		DPRINTF(2, (CE_CONT, "!%s: tx stopped on TDWBI in %d uS",
			dp->name, cnt*10));

		if (cnt == 10) {
			/* timeout: we failed to stop tx side */
			need_to_reset = B_TRUE;
			mutex_exit(&dp->xmitlock);
			goto x;
		}

		lp->td_unreliable = B_TRUE;
		(void)gem_reclaim_txbuf(dp);
		lp->td_unreliable = B_FALSE;

		/*
		 * Fix Current Tx Desctriptor to point
		 * the head of tx list.
		 */
		OUTL(dp, CTDA, ((uint32_t)dp->tx_ring_dma) +
			    SLOT(dp->tx_desc_head, TX_RING_SIZE)
				* sizeof(struct tx_desc));

		/* XXX - start Tx */
		OUTW(dp, CR, lp->cr);
		RH_KICK_TX(dp, lp->cr);

		mutex_exit(&dp->xmitlock);
	}

	if ((isr & ISR_PTX) != 0) {
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

	if ((isr & (ISR_TXE | ISR_TU | ISR_ABTI)) != 0) {
		/*
		 * Need to process transmitted buffer immediately.
		 */
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}
#ifdef notdef
	if ((isr & ISR_SRCI) != 0) {
		/*
		 * PHY port status changed.
		 */
		DPRINTF(0, (CE_WARN,
			"%s: port status changed: isr:%b misr:%b",
			dp->name,
			isr & 0xffff, ISR_BITS, isr >> 16, MISR_BITS));
		(void) gem_mii_link_check(dp);
	}
#endif
	if ((isr & (ISR_TU | ISR_ETI/*| ISR_KEYI*/)) != 0) {
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
	}
x:
	RH_READ_ISR(dp, isr);

	if (need_to_reset) {
		mutex_enter(&dp->xmitlock);
		gem_restart_nic(dp, B_TRUE);
		mutex_exit(&dp->xmitlock);
		flag |= INTR_RESTART_TX;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* enable interrupts again */
		if (lp->imr_hw != lp->imr) {
			mutex_enter(&dp->xmitlock);
			if (lp->imr_hw != lp->imr) {
				lp->imr_hw = lp->imr;
			}
			mutex_exit(&dp->xmitlock);
		}
		RH_WRITE_IMR(dp, lp->imr_hw);
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routines
 */
static void
vfe_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint16_t
vfe_mii_read(struct gem_dev *dp, uint_t reg)
{
	int	i;

	OUTB(dp, MIIADR, reg);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_RCMD);

	for (i = 0; (INB(dp, MIICR) & MIICR_RCMD) != 0; i++) {
		if  (i > 100) {
			cmn_err(CE_WARN,
				"!%s: %s: timeout", dp->name, __func__);
			return (0xffff);
		}
		drv_usecwait(10);
	}
	return (INW(dp, MIIDATA));
}

static void
vfe_mii_write_raw(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int	i;

	OUTB(dp, MIIADR, reg);
	OUTW(dp, MIIDATA, val);
	OUTB(dp, MIICR, INB(dp, MIICR) | MIICR_WCMD);
	for (i = 0; (INB(dp, MIICR) & MIICR_WCMD) != 0; i++) {
		if (i > 100) {
			cmn_err(CE_WARN,
				"!%s: %s: timeout", dp->name, __func__);
			return;
		}
		drv_usecwait(10);
	}
}

static void
vfe_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	uint16_t	ret;
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	if (reg == MII_CONTROL &&
	   (IS_RHINE3_OR_LATER(lp->revid)
	   || (IS_RHINE2_OR_LATER(lp->revid) && IS_VT6103(dp->mii_phy_id)))) {

		ret = vfe_mii_read(dp, 0x10);
		if ((val & MII_CONTROL_ANE) != 0) {
			ret &= ~1;
		} else {
			ret |= 1;
		}
		vfe_mii_write_raw(dp, 0x10, ret);
	}

	vfe_mii_write_raw(dp, reg, val);
}

static int
vfe_mii_init(struct gem_dev *dp)
{
	struct vfe_dev	*lp = (struct vfe_dev *)dp->private;

	if (gem_mii_init_default(dp) != GEM_SUCCESS) {
		return (GEM_FAILURE);
	}

	vfe_stop_mauto(dp);

	/*
	 * Fibre mode support for VT6105M and VT6103.
	 * The default bits in mii status register are still
	 * normal mii mode even if LED port waa strapped for fibre mode.
	 * Therefore, we need to check fibre mode bit in PHY configuration1
	 * register by ourselves.
	 */
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

	if (IS_RHINE3M_OR_LATER(lp->revid) || IS_VT6103(dp->mii_phy_id)) {
		/* check fibre mode */
		if ((vfe_mii_read(dp, MII_PHYCFG1) & MII_PHYCFG1_FIBRE) != 0
		     && !dp->mii_fixedmode) {
			/*
			 * force to use 100Mbps full duplex mode
			 */
			dp->mii_fixedmode = B_TRUE;
			dp->speed = GEM_SPD_100;
			dp->full_duplex = B_TRUE;

			cmn_err(CE_CONT,
	"%s: fibre mode detected, forcing to 100M full duplex fixed mode",
				dp->name);
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
vfeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			n;
	ddi_iblock_cookie_t	c;
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
	uint32_t		reg;
	uint8_t			cachelinesz;

	unit     = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(1, (CE_CONT, "!%s%d: vfeattach: called at time:%d",
		drv_name, unit, ddi_get_lbolt()));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
			drv_name, unit);
		return (DDI_FAILURE);
	}

	/* ensure we can access the registers through IO space. */
	cachelinesz = pci_config_get8(conf_handle, PCI_CONF_CACHE_LINESZ);
	reg = pci_config_get16(conf_handle, PCI_CONF_COMM);
	reg |= PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE;
#ifdef notdef
	if (cachelinesz > 0) {
		reg |= PCI_COMM_MEMWR_INVAL;
	}
#endif
	pci_config_put16(conf_handle, PCI_CONF_COMM, reg);
	reg = pci_config_get16(conf_handle, PCI_CONF_COMM);

	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid= pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: comm:%04x, ilr 0x%08x, cachline:%02x",
		drv_name, unit, reg, ilr, cachelinesz));

	/* Undocumented dances and songs for PCI config registers */
	if (IS_RHINE2_OR_LATER(revid)) {
		/* PCEROPT */
		pci_config_put8(conf_handle, 0x52,
			pci_config_get8(conf_handle, 0x52) | 0x80);
		/* MII On */
		pci_config_put8(conf_handle, 0x53, 
			pci_config_get8(conf_handle, 0x53) | 0x04);
	}
	if (IS_RHINE1(revid) || IS_RHINE3LOM(revid)) {
		pci_config_put8(conf_handle, 0x52,
			pci_config_get8(conf_handle, 0x52) | 0x02);
	}

	pci_config_teardown(&conf_handle);

	/*
	 * Misc setup for rhine registsers
	 */
	if (gem_pci_regs_map_setup(dip, PCI_ADDR_IO, &vfe_dev_attr,
		(caddr_t *)&base, &reg_ha) != DDI_SUCCESS) {
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
		ddi_get8(reg_ha, base + EECSR) | EECSR_AUTOLD);

	/*
	 * XXX - Don't access rhine for 10mS.
	 * Rhine doesn't respond to any PCI transactions
	 * while it is loading eeprom. On sparc, it caused
	 * a bus error panic. We must wait for 2.5 mS at least.
	 */
	drv_usecwait(10000);

	/* enable memory mapped I/O */
	/* XXX - reloading eeprom will disable MMIO */
	if (IS_RHINE1_VT86C100A(revid)) {
		ddi_put8(reg_ha, base + CFGA, 
			ddi_get8(reg_ha, base + CFGA) | CFGA_MMIOEN);
	}
	else {
		ddi_put8(reg_ha, base + CFGD,
			ddi_get8(reg_ha, base + CFGD) | CFGD_MMIOEN);
	}                                                        

	/* disable WOL */
	if (IS_RHINE2_OR_LATER(revid)) {
		ddi_put8(reg_ha, base + STICKHW,
			ddi_get8(reg_ha, base + STICKHW) &
				~(STICKHW_DS1 | STICKHW_DS0));
		ddi_put8(reg_ha, base + WOLCR_CLR, WOLCR_LinkOFF);
		ddi_put8(reg_ha, base + WOLCG_CLR, 0xff);
		ddi_put8(reg_ha, base + PWRCSR_CLR, 0xff);
	}

	ddi_regs_map_free(&reg_ha);


	/* check chip revision and report it */
	for (i = 0, p = vfe_chiptbl; i < CHIPTABLESIZE; i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			cmn_err(CE_CONT,
			"!%s%d: %s (vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
				drv_name, unit, p->name, vid, did, revid);
			break;
		}
	}
	if (i >= CHIPTABLESIZE) {
		/* Not found */
		cmn_err(CE_WARN,
			"!%s: vfe_attach: wrong PCI venid/devid (0x%x, 0x%x)",
			drv_name, vid, did);
	}

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/* XXX - memory mapped io doesn't work for sparc platforms */
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
			PCI_ADDR_MEM32,
#else
			PCI_ADDR_IO,
#endif
			&vfe_dev_attr, (caddr_t *)&base, &reg_ha)
					!= DDI_SUCCESS) {
			goto err;
		}

		/*
		 * construct gem configration
		 */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = 
			IS_RHINE2_OR_LATER(revid)
			? (sizeof(uint8_t) - 1)
			: (sizeof(uint32_t) - 1);
#ifdef PTX_TEST
		gcp->gc_tx_max_frags = 1;
#else
		/* rhine I supports only a fragment per tx packet */
		gcp->gc_tx_max_frags = 
			IS_RHINE2_OR_LATER(revid) ? GEM_MAXTXFRAGS : 1;
#endif
		gcp->gc_tx_desc_size =
			IS_RHINE3M_OR_LATER(revid)
			? sizeof(struct tx_desc) * TX_RING_SIZE * 8
			: sizeof(struct tx_desc) * TX_RING_SIZE;

		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_auto_pad  = B_FALSE;
		gcp->gc_tx_copy_thresh = vfe_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_size = sizeof(struct rx_desc) * RX_RING_SIZE;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_size  = RX_BUF_SIZE;
		gcp->gc_rx_max_descs_per_pkt = gcp->gc_rx_max_frags;
		gcp->gc_rx_copy_thresh = vfe_rx_copy_thresh;
		gcp->gc_rx_buf_max   = gcp->gc_rx_buf_size + 1;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, vfe_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, vfe_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, vfe_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, vfe_dma_attr_desc);
		STRUCT_COPY(gcp->gc_dma_attr_txbuf, vfe_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_align  = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, vfe_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align  = gcp->gc_rx_buf_align+1;
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
		gcp->gc_mii_reset_timeout    = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout       = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait          = 0; /* was (25*ONESEC)/10 */
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT; 

		/* workaround for DAVICOM PHY */
		gcp->gc_mii_an_delay        = ONESEC/10;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset      = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &vfe_attach_chip;
		gcp->gc_reset_chip = &vfe_reset_chip;
		gcp->gc_init_chip  = &vfe_init_chip;
		gcp->gc_start_chip = &vfe_start_chip;
		gcp->gc_stop_chip  = &vfe_stop_chip;
		gcp->gc_multicast_hash = &vfe_mcast_hash;
		gcp->gc_set_rx_filter = &vfe_set_rx_filter;
		gcp->gc_set_media = &vfe_set_media;
		gcp->gc_get_stats = &vfe_get_stats;
		gcp->gc_interrupt = &vfe_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &vfe_tx_desc_write;
		gcp->gc_rx_desc_write = &vfe_rx_desc_write;

		gcp->gc_tx_desc_stat = &vfe_tx_desc_stat;
		gcp->gc_rx_desc_stat = &vfe_rx_desc_stat;
		gcp->gc_tx_desc_init = &vfe_tx_desc_init;
		gcp->gc_rx_desc_init = &vfe_rx_desc_init;
		gcp->gc_tx_desc_clean = &vfe_tx_desc_clean;
		gcp->gc_rx_desc_clean = &vfe_rx_desc_clean;
		gcp->gc_get_packet    = &gem_get_packet_default;

		/* mii operations */
		gcp->gc_mii_init   = &vfe_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync   = &vfe_mii_sync;
		gcp->gc_mii_read   = &vfe_mii_read;
		gcp->gc_mii_write  = &vfe_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = (struct vfe_dev *)
			kmem_zalloc(sizeof(struct vfe_dev), KM_SLEEP);
		lp->revid = revid;

		dp = gem_do_attach(dip, gcp, base, &reg_ha, lp, sizeof(*lp));

		kmem_free(gcp, sizeof(*gcp));
		if (dp == NULL) {
			goto  err_free_mem;
		}

#ifdef OPT_TX_INTR
		if (IS_RHINE1(lp->revid)) {
			if (ddi_add_softintr(dp->dip,
					DDI_SOFTINT_LOW, &lp->soft_id,
					NULL, NULL,
					(uint_t (*)(caddr_t))&vfe_update_imr,
					(caddr_t)dp) != DDI_SUCCESS) {
				goto  err_free_mem;
			}
		}
#endif
		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof(struct vfe_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
vfedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
#ifdef OPT_TX_INTR
	struct gem_dev  *dp;
	gld_mac_info_t  *macinfo;
	struct vfe_dev	*lp;

	macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
	dp = (struct gem_dev *)macinfo->gldm_private;
	lp = (struct vfe_dev *)dp->private;
#endif
	switch (cmd) {
	case DDI_DETACH:
#ifdef OPT_TX_INTR
		if (IS_RHINE1(lp->revid)) {
			ddi_remove_softintr(lp->soft_id);
		}
#endif
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
static	struct module_info vfeminfo = {
	0,			/* mi_idnum */
	"vfe",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	TX_BUF_SIZE * ETHERMAX,	/* mi_hiwat */
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
	ddi_power	/* devo_power */
};

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

	DPRINTF(2, (CE_CONT, "!vfe: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
