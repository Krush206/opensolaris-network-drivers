/******************************************************************************
 *
 * Name   : sky2.c
 * Project: Gigabit Ethernet Driver for FreeBSD 5.x/6.x
 * Version: $Revision: 1.23 $
 * Date   : $Date: 2005/12/22 09:04:11 $
 * Purpose: Main driver source file
 *
 *****************************************************************************/

/******************************************************************************
 *
 *	LICENSE:
 *	Copyright (C) Marvell International Ltd. and/or its affiliates
 *
 *	The computer program files contained in this folder ("Files")
 *	are provided to you under the BSD-type license terms provided
 *	below, and any use of such Files and any derivative works
 *	thereof created by you shall be governed by the following terms
 *	and conditions:
 *
 *	- Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above
 *	  copyright notice, this list of conditions and the following
 *	  disclaimer in the documentation and/or other materials provided
 *	  with the distribution.
 *	- Neither the name of Marvell nor the names of its contributors
 *	  may be used to endorse or promote products derived from this
 *	  software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *	BUT NOT LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR SERVICES;
 *	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *	STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *	OF THE POSSIBILITY OF SUCH DAMAGE.
 *	/LICENSE
 *
 *****************************************************************************/

/*-
 * Copyright (c) 1997, 1998, 1999, 2000
 *	Bill Paul <wpaul@ctr.columbia.edu>.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/*-
 * Copyright (c) 2003 Nathan L. Binkert <binkertn@umich.edu>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Device driver for the Marvell Yukon II Ethernet controller.
 * Due to lack of documentation, this driver is based on the code from
 * sk(4) and Marvell's myk(4) driver for FreeBSD 5.x.
 */

/*
 * Change log
 */

/*
 * TODO:
 *	port reset and global reset.
 */

/*
 * CAUTION:
 *	Don't enable checksum offloading. It rarely corrupts UDP packets.
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
#include "mykreg.h"

char	ident[] = "yukon2 GbE driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static volatile int myk_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (myk_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
/* additional PCI related macros for compatibility */
#ifndef PCI_CAP_ID_PCIX
#define	PCI_CAP_ID_PCIX		0x7
#endif
#ifndef PCI_CAP_ID_PCI_E
#define	PCI_CAP_ID_PCI_E	0x10
#endif
#ifndef	PCI_PMCSR_D3COLD
#define	PCI_PMCSR_D3COLD	4
#endif

#define	FLSHB(dp, reg)	(void)INB(dp, reg)
#define	FLSHW(dp, reg)	(void)INW(dp, reg)
#define	FLSHL(dp, reg)	(void)INL(dp, reg)

#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))

#define	INSIDE(slot, head, tail)	\
	(((head) <= (tail)) ?	\
	((head) <= (slot) && (slot) < (tail)) :	\
	((slot) < (tail) || (head) <= (slot)))

#define	RAM_BUFFER(b, r)	SELECT_RAM_BUFFER(b, r)
#define	Y2_QADDR(q, r)		Y2_PREF_Q_ADDR(q, r)

#define	TXDESC(p)	((struct msk_tx_desc *)(void *)(p))
#define	RXDESC(p)	((struct msk_rx_desc *)(void *)(p))

#define	IS_YUKON2(id)	((id) >= CHIP_ID_YUKON_XL)

#define	HW_HAS_NEWER_PHY(id)	\
	((id) == CHIP_ID_YUKON_XL ||	\
	(id) == CHIP_ID_YUKON_EC_U ||	\
	(id) == CHIP_ID_YUKON_EX ||	\
	(id) >= CHIP_ID_YUKON_SUPR)

#define	HW_IS_RAM_IF_AVAIL(id)	\
	((id) == CHIP_ID_YUKON || \
	(id) == CHIP_ID_YUKON_LITE || \
	(id) == CHIP_ID_YUKON_XL || \
	(id) == CHIP_ID_YUKON_EC || \
	(id) == CHIP_ID_YUKON_FE)

/* for *bsd compatibility */
#define	ETHER_HDR_LEN	(sizeof (struct ether_header))
#define	ETHER_CRC_LEN	(ETHERFCSL)

/*
 * Our configuration
 */
#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	512
#endif
#ifndef TX_BUF_SIZE
#define	TX_BUF_SIZE	64
#endif

#ifndef RX_RING_SIZE
#define	RX_RING_SIZE	256
#endif
#define	RX_DESC_SIZE	(RX_RING_SIZE*2)

#define	DESC_ALIGN	(4096*8)

#define	ONESEC		(drv_usectohz(1*1000000))

/* I/O area confiuration */
#define	STATUS_DESC_SIZE	2048	/* 2 ports * (TX + RX) */
#define	STATUS_LE_BYTES		\
	ROUNDUP2(STATUS_DESC_SIZE * sizeof (struct msk_stat_desc), DESC_ALIGN)
#define	RX_DESC_BYTES	\
	ROUNDUP2(RX_DESC_SIZE * sizeof (struct msk_rx_desc), DESC_ALIGN)

#define	RX_DESC_BASE(dp)		((dp)->io_area)
#define	STATUS_DESC_BASE(dp)		((dp)->io_area + RX_DESC_BYTES)

#define	RX_DESC_BASE_DMA(dp)		((dp)->io_area_dma)
#define	STATUS_DESC_BASE_DMA(dp)	((dp)->io_area_dma + RX_DESC_BYTES)

/*
 * Marvell yukon2 driver for Solaris
 */
struct rx_soft_desc {
	uint32_t	rx_len;
	uint32_t	rx_stat;
	uint16_t	rx_tag;
	uint16_t	rx_cksum;
};

#define	PORT(dp)	((dp)->port)

struct myk_dev {
	int		initialized;
	kmutex_t	chip_lock;

	uint8_t		mac_id;
	uint8_t		chip_id;
	uint_t		chip_rev;
	uint8_t		chip_cap;
	uint8_t		ports;
	uint8_t		pmd_type;
	uint_t		pm_cap;
	boolean_t	is_pcie;
	boolean_t	tx_full_cksum;
	boolean_t	rx_full_cksum;
	boolean_t	asf_enabled;

	ddi_iblock_cookie_t   iblock_cookie;
#ifdef notdef
	int		last_poll_interval;
#endif
	/* status descriptor management */
	int		status_desc_head;

	struct myk_port {
		struct gem_dev	*dp;

		/* interrput mask management */
		uint32_t	imsk;
		uint32_t	hwe_imsk;
		uint32_t	isr;

		/* MII phy state management */
		uint16_t	bmcr;

		/* tx descriptor management */
		uint32_t	tx_phys32hi;
		uint32_t	tx_mss;
		uint32_t	tx_hck_offsets;
		uint32_t	tx_vtag;
		volatile uint_t	tx_head;
		uint_t		tx_tail;

		/* rx descriptor management */
		uint32_t	rx_phys32hi;
		seqnum_t	rx_desc_head;
		seqnum_t	rx_desc_tail;

		/* rx buffer management */
		int		rx_slot_tail;
		struct rx_soft_desc	rx_slot[RX_RING_SIZE];

		/* led control */
		uint32_t	blink_ctrl;
	} descset[2];
};

static int	myk_tx_copy_thresh = 256;
static int	myk_rx_copy_thresh = 256;

/* ======================================================== */
/* status descriptor operations */
static void myk_status_desc_dma_sync(struct gem_dev *, int, int, int);
static void myk_status_desc_init(struct myk_dev *);

/* mii operations */
static void  myk_mii_sync(struct gem_dev *);
static uint16_t myk_mii_read(struct gem_dev *, uint_t);
static void myk_mii_write_raw(struct gem_dev *, uint_t, uint16_t);
static void myk_mii_write(struct gem_dev *, uint_t, uint16_t);
static int myk_mii_init(struct gem_dev *);

/* nic operations */
static int myk_attach_port(struct gem_dev *);
static int myk_reset_port(struct gem_dev *);
static int myk_init_port(struct gem_dev *);
static int myk_start_port(struct gem_dev *);
static int myk_stop_port(struct gem_dev *);
static int myk_set_media(struct gem_dev *);
static int myk_set_rx_filter(struct gem_dev *);
static int myk_get_stats(struct gem_dev *);

/* descriptor operations */
static int myk_tx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void myk_tx_start(struct gem_dev *dp, int slot, int nslot);
static void myk_rx_start(struct gem_dev *dp, int slot, int nslot);
static void myk_rx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags);
static uint_t myk_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t myk_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void myk_tx_desc_init(struct gem_dev *dp, int slot);
static void myk_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t myk_soft_intr(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr myk_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr myk_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t myk_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x000000ffffffffffull,	/* dma_attr_addr_hi */
	0x000000000000ffffull,	/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0x0000fffc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x000000000000ffffull,	/* dma_attr_maxxfer */
	0x00000000ffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t myk_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x00000000ffffffffull,	/* dma_attr_addr_hi */
	0x00000000ffffffffull,	/* dma_attr_count_max */
	DESC_ALIGN,		/* dma_attr_align */
	0x000007fc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x00000000ffffffffull,	/* dma_attr_maxxfer */
	0x00000000ffffffffull,	/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

#include "mskreg.h"

static int
myk_is_copper(struct gem_dev *dp)
{
	boolean_t	ret;
	struct myk_dev	*lp = dp->private;

	switch (lp->pmd_type) {
	case 'L': case 'S': case 'P':
		ret = B_FALSE;
		break;

	default:
		ret = B_TRUE;
		break;
	}

	return (ret);
}

/* Yukon PHY related registers */
#define	GMAC_REG(port, reg)	\
	(BASE_GMAC_1 + (port) * (BASE_GMAC_2-BASE_GMAC_1) + (reg))

static void
myk_set_macaddr(struct gem_dev *dp, unsigned port, unsigned reg, uint8_t *addr)
{
	OUTW(dp, GMAC_REG(port, reg + 0), addr[0] | (addr[1] << 8));
	OUTW(dp, GMAC_REG(port, reg + 4), addr[2] | (addr[3] << 8));
	OUTW(dp, GMAC_REG(port, reg + 8), addr[4] | (addr[5] << 8));
}

/* PCI config space access */
static uint32_t
myk_pci_read32(struct gem_dev *dp, unsigned reg)
{
	return (INL(dp, Y2_CFG_SPC + reg));
}

static uint16_t
myk_pci_read16(struct gem_dev *dp, unsigned reg)
{
	return (INW(dp, Y2_CFG_SPC + reg));
}

static void
myk_pci_write32(struct gem_dev *dp, unsigned reg, uint32_t val)
{
	OUTL(dp, Y2_CFG_SPC + reg, val);
}

static void
myk_pci_write16(struct gem_dev *dp, unsigned reg, uint16_t val)
{
	OUTW(dp, Y2_CFG_SPC + reg, val);
}

/*
 * The Yukon II chipset takes 64 bit command blocks (called list elements)
 * that are organized into three (receive, transmit, status) different rings
 * similar to Tigon3. A transmit can require several elements;
 * a receive requires one (or two if using 64 bit dma).
 */

/* Avoid conditionals by using array */
#define	TXQADDR(p)	((p) == 0 ? Q_XA1 : Q_XA2)
#define	RXQADDR(p)	((p) == 0 ? Q_R1 : Q_R2)
#define	PORTIRQ_MSK(p)	((p) == 0 ? Y2_IS_PORT_A : Y2_IS_PORT_B)

/* This driver supports yukon2 chipset only */
static const char *yukon2_name[] = {
	"XL",		/* 0xb3 */
	"EC Ultra", 	/* 0xb4 */
	"EX",		/* 0xb5 */
	"EC",		/* 0xb6 */
	"FE",		/* 0xb7 */
	"FE+",		/* 0xb8 */
	"SUPR",		/* 0xb9 */
	"Ultra2",	/* 0xba */
	"unknown",	/* 0xbb */
	"Optima",	/* 0xbc */
};

static void
myk_set_phy_power_state(struct gem_dev *dp, unsigned state)
{
	uint16_t	power_cap;
#ifdef notdef
	uint16_t	power_control;
#endif
	uint32_t	our1;
	uint32_t	val;
	uint32_t	powerdownbit;
	int		i;
	int		vaux;
	struct myk_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s %d", dp->name, state));
#ifdef notdef
	/* enable Config Write */
	OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_ON);
#endif
	power_cap = myk_pci_read16(dp, lp->pm_cap + PCI_PMCAP);
	vaux = (INW(dp, B0_CTST) & Y2_VAUX_AVAIL) &&
	    (power_cap & PCI_PMCAP_D3COLD_PME);

#ifdef notdef
	power_control = myk_pci_read16(dp, lp->pm_cap + PCI_PMCSR);
	power_control |= PCI_PMCSR_PME_STAT;
	power_control &= ~(PCI_PMCSR_STATE_MASK);
#endif

	powerdownbit = PCI_Y2_PHY1_POWD | PCI_Y2_PHY2_POWD;

#define	CHIP_REV_YU_XL_A1	1

	switch (state) {
	case PCI_PMCSR_D0:
		/* switch power to VCC (WA for VAUX problem) */
		OUTB(dp, B0_POWER_CTRL,
		    PC_VAUX_ENA | PC_VCC_ENA | PC_VAUX_OFF | PC_VCC_ON);

		/* disable Core Clock Division, */
		OUTL(dp, B2_Y2_CLK_CTRL, Y2_CLK_DIV_DIS);

		val = 0;
		if (lp->chip_id == CHIP_ID_YUKON_XL &&
		    lp->chip_rev > CHIP_REV_YU_XL_A1) {
			/* Enable bits are inverted */
			val = Y2_PCI_CLK_LNK1_DIS | Y2_COR_CLK_LNK1_DIS |
			    Y2_CLK_GAT_LNK1_DIS | Y2_PCI_CLK_LNK2_DIS |
			    Y2_COR_CLK_LNK2_DIS | Y2_CLK_GAT_LNK2_DIS;
		}
		/*
		 * Enable PCI & Core Clock, enable clock gating for both Links.
		 */
		OUTB(dp, B2_Y2_CLK_GATE, val);

		/* Turn off phy power saving */
		our1 = myk_pci_read32(dp, PCI_OUR_REG_1);
		our1 &= ~powerdownbit;

		/* looks like this XL is back asswards .. */
		if (lp->chip_id == CHIP_ID_YUKON_XL &&
		    lp->chip_rev > CHIP_REV_YU_XL_A1) {
			/* deassert Low Power for 1st PHY */
			our1 |= PCI_Y2_PHY1_COMA;
			if (lp->ports > 1) {
				our1 |= PCI_Y2_PHY2_COMA;
			}
		} else if (lp->chip_id == CHIP_ID_YUKON_EC_U) {
			uint32_t	tmp32;

			/* enable HW WOL */
			OUTW(dp, B0_CTST, Y2_HW_WOL_ON);

			/* enable all clocks */
			myk_pci_write32(dp, PCI_OUR_REG_3, 0);
			tmp32 = myk_pci_read32(dp, PCI_OUR_REG_4);
			tmp32 &=
			    (P_FORCE_ASPM_REQUEST |
			    P_ASPM_GPHY_LINK_DOWN |
			    P_ASPM_INT_FIFO_EMPTY |
			    P_ASPM_CLKRUN_REQUEST);
			/* Set all bits to 0 except bits 15..12. */
			myk_pci_write32(dp, PCI_OUR_REG_4, tmp32);

			/* Set to default value. */
			tmp32 = myk_pci_read32(dp, PCI_OUR_REG_5);
			tmp32 &= P_CTL_TIM_VMAIN_AV_MSK;
			myk_pci_write32(dp, PCI_OUR_REG_5, tmp32);
		}

		/* release PHY from PowerDown/COMA Mode */
		myk_pci_write32(dp, PCI_OUR_REG_1, our1);

		if (!lp->asf_enabled) {
			for (i = 0; i < lp->ports; i++) {
				OUTB(dp, MR_ADDR(i, GMAC_LINK_CTRL),
				    (uint8_t)GMLC_RST_SET);
				OUTB(dp, MR_ADDR(i, GMAC_LINK_CTRL),
				    (uint8_t)GMLC_RST_CLR);
			}
		}
		break;

	case PCI_PMCSR_D3HOT:
	case PCI_PMCSR_D3COLD:
		/* Turn on phy power saving */
		val = myk_pci_read32(dp, PCI_OUR_REG_1);
		if (lp->chip_id == CHIP_ID_YUKON_XL &&
		    lp->chip_rev > CHIP_REV_YU_XL_A1) {
			val &= ~(PCI_Y2_PHY1_POWD | PCI_Y2_PHY2_POWD);
		} else {
			val |= (PCI_Y2_PHY1_POWD | PCI_Y2_PHY2_POWD);
		}
		myk_pci_write32(dp, PCI_OUR_REG_1, val);

		if (lp->chip_id == CHIP_ID_YUKON_XL &&
		    lp->chip_rev > CHIP_REV_YU_XL_A1) {
			OUTB(dp, B2_Y2_CLK_GATE, 0);
		} else {
			/* enable bits are inverted */
			OUTB(dp, B2_Y2_CLK_GATE,
			    Y2_PCI_CLK_LNK1_DIS | Y2_COR_CLK_LNK1_DIS |
			    Y2_CLK_GAT_LNK1_DIS | Y2_PCI_CLK_LNK2_DIS |
			    Y2_COR_CLK_LNK2_DIS | Y2_CLK_GAT_LNK2_DIS);
		}

		/* switch power to VAUX */
		if (vaux && state != PCI_PMCSR_D3COLD) {
			OUTB(dp, B0_POWER_CTRL,
			    PC_VAUX_ENA | PC_VCC_ENA |
			    PC_VAUX_ON | PC_VCC_OFF);
		}
		break;

	default:
		cmn_err(CE_WARN, "!%s: %s; Unknown power state %d",
		    dp->name, __func__, state);
		break;
	}
#ifdef notdef
	myk_pci_write16(dp, lp->pm_cap + PCI_PMCSR, power_control);
	OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_OFF);
#endif
}

/* Chip internal frequency for clock calculations */
static uint32_t
myk_mhz(struct gem_dev *dp)
{
	struct myk_dev	*lp = dp->private;

	switch (lp->chip_id) {
	case CHIP_ID_YUKON_EC:
	case CHIP_ID_YUKON_EC_U:
	case CHIP_ID_YUKON_EX:
	case CHIP_ID_YUKON_SUPR:
	case CHIP_ID_YUKON_UL_2:
#ifdef CONFIG_OPTIMA
	case CHIP_ID_YUKON_OPT:
#endif
		return (125);	/* 125 Mhz */

	case CHIP_ID_YUKON_FE:
		return (100);	/* 100 Mhz */

	case CHIP_ID_YUKON_FE_P:
		return (50);	/* 50 Mhz */

	case CHIP_ID_YUKON_XL:
		return (156);	/* 156 Mhz */

	default:
		cmn_err(CE_WARN, "%s: %s: unknown chip id (0x%x)",
		    dp->name, __func__, lp->chip_id);
		/* FALLTHRU */
	}

	return (100);	/* guess: 100 Mhz */
}

static uint32_t
myk_us2clk(struct gem_dev *dp, uint32_t us)
{
	return (myk_mhz(dp) * us);
}

#define	HW_MS_TO_TICKS(dp, ms)	myk_us2clk(dp, (ms)*1000)

/* ======================================================== */
/*
 * HW descriputor cache sync routines
 */
/* ======================================================== */
static void
myk_rx_desc_dma_sync(struct gem_dev *dp, int head, int nslot, int how)
{
	int	n;
	int	m;
	off_t	rx_desc_offset;

	/* sync active descriptors */
	rx_desc_offset = (off_t)(RX_DESC_BASE_DMA(dp) - dp->rx_ring_dma);
	n = RX_DESC_SIZE - head;
	if ((m = nslot - n) > 0) {
		(void) ddi_dma_sync(dp->desc_dma_handle, rx_desc_offset,
		    (size_t)(m * sizeof (struct msk_rx_desc)), how);
		nslot = n;
	}

	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(head * sizeof (struct msk_rx_desc) + rx_desc_offset),
	    (size_t)(nslot * sizeof (struct msk_rx_desc)),
	    how);
}

static void
myk_status_desc_dma_sync(struct gem_dev *dp, int head, int nslot, int how)
{
	int	n;
	int	m;
	off_t	status_desc_offset;

	/* sync active descriptors */
	status_desc_offset = STATUS_DESC_BASE_DMA(dp) - dp->rx_ring_dma;
	n = STATUS_DESC_SIZE - head;
	if ((m = nslot - n) > 0) {
		(void) ddi_dma_sync(dp->desc_dma_handle, status_desc_offset,
		    (size_t)(m * sizeof (struct msk_stat_desc)),
		    how);
		nslot = n;
	}

	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(head * sizeof (struct msk_stat_desc) + status_desc_offset),
	    (size_t)(nslot * sizeof (struct msk_stat_desc)),
	    how);
}

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */
#ifdef notdef
static boolean_t
myk_test_chip(struct gem_dev *dp, char *msg)
{
	uint32_t	val;
	boolean_t	ret;

#define	TEST_VAL	0x11335577
#define	B2_IRQM_INI	0x0140

	/* check if the adapter seems to be accessible */
	val = INL(dp, B2_IRQM_INI);
	OUTL(dp, B2_IRQM_INI, TEST_VAL);

	if ((ret = (INL(dp, B2_IRQM_INI)) == TEST_VAL)) {
		/* restore original value */
		cmn_err(CE_CONT, "!%s: test succeeded for %s",
		    dp->name, msg);
		OUTL(dp, B2_IRQM_INI, val);
	} else {
		cmn_err(CE_CONT, "!%s: test failed for %s",
		    dp->name, msg);
		OUTL(dp, B2_IRQM_INI, 0);
	}

	return (ret);
}
#endif /* notdef */
static int
myk_reset_port(struct gem_dev *dp)
{
	int		i;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called, GPHY:%08x, GMAC:%02x",
	    dp->name, __func__,
	    INL(dp, MR_ADDR(port, GPHY_CTRL)),
	    INB(dp, MR_ADDR(port, GMAC_CTRL))));

	/* setup back pointer to struct gem_dev */
	if (lp->descset[port].dp == NULL) {
		lp->descset[port].dp = dp;
	}

	if (lp->initialized <= 0) {
		/*
		 * SkGeInit1()
		 */

#ifdef notdef	/* should be moved into attach */
		myk_test_chip(dp, "reset_port@1");
#endif /* notdef */

		if (!lp->asf_enabled) {
			/*
			 * Since we disabled ASF, S/W reset is required
			 * for Power Management.
			 */
			OUTB(dp, B0_CTST, CS_RST_SET);	/* ok */
			OUTB(dp, B0_CTST, CS_RST_CLR);	/* ok */
		}

		/* make config registers writable */	/* ok */
		OUTB(dp, B2_TST_CTRL1,
		    (INB(dp, B2_TST_CTRL1) & ~TST_CFG_WRITE_OFF)
		    | TST_CFG_WRITE_ON);

		/* Clear all error bits in the PCI status register */ /* ok */
		myk_pci_write16(dp, PCI_CONF_STAT,
		    myk_pci_read16(dp, PCI_CONF_STAT) |
		    PCI_STAT_PERROR | PCI_STAT_S_SYSERR | PCI_STAT_R_MAST_AB |
		    PCI_STAT_R_TARG_AB | PCI_STAT_S_PERROR);

		OUTW(dp, B0_CTST, CS_MRST_CLR); /* ok */

		if (lp->is_pcie) {
			/* clear any PEX errors */
			myk_pci_write32(dp, PEX_UNC_ERR_STAT, ~0U); /* ok */
		}
		/* XXX - case PCI */
		/* XXX - case PCIX */

		/* Set PHY power state. */
		myk_set_phy_power_state(dp, PCI_PMCSR_D0);

		/* make config registers read only */
		OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_OFF);

		/* LED On. */
		OUTW(dp, B0_CTST, Y2_LED_STAT_ON);

		/* Clear TWSI IRQ. */
		OUTL(dp, B2_I2C_IRQ, 1 /* I2C_CLR_IR */);

		/* Turn off hardware timer. */
		OUTB(dp, B2_TI_CTRL, TIM_STOP);
		OUTB(dp, B2_TI_CTRL, TIM_CLR_IRQ);

		/* Turn off descriptor polling. */
		OUTB(dp, B28_DPT_CTRL, DPT_STOP);

		/* Turn off time stamps. */
		OUTB(dp, GMAC_TI_ST_CTRL, GMT_ST_STOP);
		OUTB(dp, GMAC_TI_ST_CTRL, GMT_ST_CLR_IRQ);

		/* inhibit all interrupts */
		OUTL(dp, B0_HWE_IMSK, 0);
		FLSHL(dp, B0_HWE_IMSK);
		OUTL(dp, B0_IMSK, 0);
		FLSHL(dp, B0_IMSK);

		for (i = 0; i < lp->ports; i++) {
			lp->descset[i].imsk = 0;
			lp->descset[i].hwe_imsk = 0;
		}

		/* XXX - need for workaround for PCI-X cards */
		/* XXX - need for workaround for PCIE cards */
#ifdef notdef /* sun4u */
		/* byte swap descriptors in hardware */
		myk_pci_write32(dp, PCI_OUR_REG_2,
		    myk_pci_read32(dp, PCI_OUR_REG_2) | PCI_REV_DESC);
#endif

		if (lp->initialized < 0) {
			myk_status_desc_init(lp);
		}
	}

	/* inhibit all interrupts */
	lp->descset[port].hwe_imsk = 0;
	OUTL(dp, B0_HWE_IMSK,
	    lp->descset[0].hwe_imsk | lp->descset[1].hwe_imsk);
	FLSHL(dp, B0_HWE_IMSK);

	lp->descset[port].imsk = 0;
	OUTL(dp, B0_IMSK, lp->descset[0].imsk | lp->descset[1].imsk);
	FLSHL(dp, B0_IMSK);

	/* Reset GPHY/GMAC Control */
	OUTB(dp, MR_ADDR(port, GPHY_CTRL), GPC_RST_SET);
	OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_RST_SET);

	OUTB(dp, MR_ADDR(port, GPHY_CTRL), GPC_RST_CLR);
	OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_RST_CLR);

	OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_F_LOOPB_OFF);
	if (lp->chip_id == CHIP_ID_YUKON_EX) {
		OUTW(dp, MR_ADDR(port, GMAC_CTRL),
		    GMC_BYP_MACSECRX_ON | GMC_BYP_MACSECTX_ON |
		    GMC_BYP_RETR_ON);
	}

	/*
	 * SeGeInit2()
	 */
#ifdef notyet
	/* descriptor polling timer */
#endif
	/* enable the Tx Arbiters */
	OUTB(dp, MR_ADDR(port, TXA_CTRL), TXA_ENA_ARB);

	/*
	 * From SkGeInitRamIface()	done
	 */
	if (HW_IS_RAM_IF_AVAIL(lp->chip_id)) {
		/* Enable the RAM Interface Arbiter. */
		OUTW(dp, RAM_BUFFER(port, B3_RI_CTRL), RI_RST_CLR);

		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_R1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_XA1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_XS1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_R1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_XA1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_XS1), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_R2), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_XA2), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_WTO_XS2), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_R2), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_XA2), MSK_RI_TO_53);
		OUTB(dp, RAM_BUFFER(port, B3_RI_RTO_XS2), MSK_RI_TO_53);
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: exit, GPHY:%08x, GMAC:%02x",
	    dp->name, __func__,
	    INL(dp, MR_ADDR(port, GPHY_CTRL)),
	    INB(dp, MR_ADDR(port, GMAC_CTRL))));

	if (lp->initialized <= 0) {
		/* make config registers writable */	/* ok */
		OUTB(dp, B2_TST_CTRL1,
		    (INB(dp, B2_TST_CTRL1) & ~TST_CFG_WRITE_OFF)
		    | TST_CFG_WRITE_ON);
#if 1
		if (lp->is_pcie) {
			uint16_t v;

			v = myk_pci_read16(dp, PEX_DEV_CTRL);

			/* check for HW-default value of 512 bytes */
			if ((v & PEX_DC_MAX_RRS_MSK) ==
			    PEX_DC_MAX_RD_RQ_SIZE(2)) {
				/*
				 * change Max. Read Request Size to
				 * 2048 bytes
				 */
				v &= ~PEX_DC_MAX_RRS_MSK;
				v |= PEX_DC_MAX_RD_RQ_SIZE(4);

				myk_pci_write16(dp, PEX_DEV_CTRL, v);
			}
			if (lp->chip_id == CHIP_ID_YUKON_FE_P ||
			    lp->chip_id == CHIP_ID_YUKON_UL_2 ||
			    lp->chip_id == CHIP_ID_YUKON_OPT) {
				/* set TWSI timeout value to maximum */
				v = myk_pci_read16(dp, 0xa4 /* OTP_LDR_CTRL */);

				v |= 7 << 9 /* OTP_LDR_CTRL_EEPROM_TIMEOUT_VAL_MSK */;

				myk_pci_write16(dp, 0xa4 /* OTP_LDR_CTRL */, v);
#ifdef REPLAY_TIMER
			} else if (lp->chip_id == CHIP_ID_YUKON_EC) {
				/* PEX Ack Reply Timeout to 40 us */
				myk_pci_write16(dp,
				    0x22a /* PEX_ACK_RPLY_TOX1 */, 0x2710);
#endif
			}
		}
#endif
		/* make config registers read only */
		OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_OFF);

		lp->initialized = 1;
	}

	return (GEM_SUCCESS);
}

/*
 * From DoInitRamQueue() - Initialize the RAM Buffer Address of a single Queue
 */
static void
myk_init_ramqueue(struct gem_dev *dp,
	uint16_t q, uint8_t startpg, uint8_t endpg)
{
	uint32_t	start;
	uint32_t	end;

	start = startpg * 4096/8;
	end = (endpg * 4096/8) - 1;

	/* release local reset */
	OUTB(dp, RB_ADDR(q, RB_CTRL), RB_RST_CLR);

	/* configure addresses */
	OUTL(dp, RB_ADDR(q, RB_START), start);
	OUTL(dp, RB_ADDR(q, RB_END), end);
	OUTL(dp, RB_ADDR(q, RB_WP), start);
	OUTL(dp, RB_ADDR(q, RB_RP), start);

	if (q == Q_R1 || q == Q_R2) {
		uint32_t	space = (endpg - startpg) * 4096/8;
		uint32_t	tp = space - space/4;

		/*
		 * On receive queue's set the thresholds
		 * give receiver priority when > 3/4 full
		 * send pause when down to 2K
		 */
		OUTL(dp, RB_ADDR(q, RB_RX_UTHP), tp);
		OUTL(dp, RB_ADDR(q, RB_RX_LTHP), space/2);

		tp = space - 2048/8;
		OUTL(dp, RB_ADDR(q, RB_RX_UTPP), tp);
		OUTL(dp, RB_ADDR(q, RB_RX_LTPP), space/4);
	} else {
		/*
		 * As YUKON is used ((GMAC Tx FIFO is only 1 kB)
		 * we NEED Store & Forward of the RAM buffer.
		 */
		OUTB(dp, RB_ADDR(q, RB_CTRL), RB_ENA_STFWD);
	}

	/* set queue operational */
	OUTB(dp, RB_ADDR(q, RB_CTRL), RB_ENA_OP_MD);
	FLSHB(dp, RB_ADDR(q, RB_CTRL));
}

/*
 * From SkGeY2InitPrefetchUnit() - Initialize a Prefetch Unit
 *
 * Description:
 *      Calling this function requires an already configured list element
 *      table. The prefetch unit to be configured is specified in the parameter
 *      'Queue'. The function is able to initialze the prefetch units of
 *      the following queues: Q_R1, Q_R2, Q_XS1, Q_XS2, Q_XA1, Q_XA2.
 *      The funcution should be called before SkGeInitPort().
 *
 */
static void
myk_prefetch_init(
	struct gem_dev *dp, uint32_t qaddr, uint64_t addr, uint16_t last)
{
	/* disable the prefetch unit */
	OUTB(dp, Y2_QADDR(qaddr, PREF_UNIT_CTRL_REG), PREF_UNIT_RST_SET);
	OUTB(dp, Y2_QADDR(qaddr, PREF_UNIT_CTRL_REG), PREF_UNIT_RST_CLR);

	/* Set the list base address */
	OUTL(dp, Y2_QADDR(qaddr, PREF_UNIT_ADDR_HI_REG), addr >> 32);
	OUTL(dp, Y2_QADDR(qaddr, PREF_UNIT_ADDR_LOW_REG), (uint32_t)addr);

	/* Set the list last index */
	OUTW(dp, Y2_QADDR(qaddr, PREF_UNIT_LAST_IDX_REG), last);

	/* turn on prefetch unit */
	OUTB(dp, Y2_QADDR(qaddr, PREF_UNIT_CTRL_REG), PREF_UNIT_OP_ON);

	FLSHL(dp, Y2_QADDR(qaddr, PREF_UNIT_CTRL_REG));
}

static void
myk_rx_set_checksum(struct gem_dev *dp)
{
	struct msk_rx_desc	*rdp;
	struct myk_dev		*lp = dp->private;
	struct myk_port		*pp = &lp->descset[PORT(dp)];

	ASSERT(pp->rx_desc_tail == 0);

	rdp = &RXDESC(RX_DESC_BASE(dp))[SLOT(pp->rx_desc_tail, RX_DESC_SIZE)];
	pp->rx_desc_tail++;

	rdp->msk_addr = LE_32(
	    (dp->gc.gc_hck_rx_start << 16) | dp->gc.gc_hck_rx_start);
	rdp->msk_control = LE_32(OP_TCPSTART | HW_OWNER);
}

/*
 * From SkGeY2InitStatBmu() -    Initialize the Status BMU
 *
 * Description:
 *      Calling this function requires an already configured list element
 *      table. Ensure the status BMU is only initialized once during
 *  DriverInit - InitLevel2 required.
 */
static void
myk_status_desc_init(struct myk_dev *lp)
{
	struct gem_dev	*dp = lp->descset[0].dp;
	uint64_t	base;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	bzero(STATUS_DESC_BASE(dp), STATUS_LE_BYTES);
	lp->status_desc_head = 0;

	/* disable the prefetch unit */
	OUTB(dp, STAT_CTRL, SC_STAT_RST_SET);
	OUTB(dp, STAT_CTRL, SC_STAT_RST_CLR);

	DPRINTF(0, (CE_CONT, "!%s: %s: status_le_base:0x%llx",
	    dp->name, __func__, STATUS_DESC_BASE_DMA(dp)));

	DPRINTF(0, (CE_CONT, "!%s: %s: rx_desc_base:0x%llx",
	    dp->name, __func__, RX_DESC_BASE_DMA(dp)));

	/* Set the list base address */
	base = STATUS_DESC_BASE_DMA(dp);
	OUTL(dp, STAT_LIST_ADDR_LO, base);
	OUTL(dp, STAT_LIST_ADDR_HI, base >> 32);

	/* Set the list last index */
	OUTW(dp, STAT_LAST_IDX, STATUS_DESC_SIZE - 1);

	if (lp->chip_id == CHIP_ID_YUKON_EC &&
	    lp->chip_rev == CHIP_REV_YU_EC_A1) {
		/* HWF_WA_DEV_43_418 */

		/* WA for dev. #4.3 */
		OUTW(dp, STAT_TX_IDX_TH, ST_TXTH_IDX_MASK);

		/* set Status-FIFO watermark */
		OUTB(dp, STAT_FIFO_WM, 0x21);	/* WA for dev. #4.18 */

		/* set Status-FIFO ISR watermark */
		OUTB(dp, STAT_FIFO_ISR_WM, 7);	/* WA for dev. #4.18 */

		/* WA for dev. #4.3 and #4.18 */
		/* set Status-FIFO Tx timer init value */
		OUTL(dp, STAT_TX_TIMER_INI, HW_MS_TO_TICKS(dp, 10));
	} else {
		OUTW(dp, STAT_TX_IDX_TH, 0x000a);

		/* set Status-FIFO watermark */
		OUTB(dp, STAT_FIFO_WM, 0x10);

		/* set Status-FIFO ISR watermark */
		OUTB(dp, STAT_FIFO_ISR_WM, 0x10); /* HWF_WA_DEV_4109 */

		/* set ISR Timer Init Value to 400 */
		OUTL(dp, STAT_ISR_TIMER_INI, 0x0190);

		/* set Status-FIFO Tx timer init value */
		OUTL(dp, STAT_TX_TIMER_INI, myk_us2clk(dp, 10000));
	}
	DPRINTF(0, (CE_CONT, "!%s: STAT_TX_TIMER_INIT:0x%x",
	    __func__, INL(dp, STAT_TX_TIMER_INI)));

	/* enable the prefetch unit */
	/* operational bit not functional for Yukon-EC, but fixed in Yukon-2 */
	OUTB(dp, STAT_CTRL, SC_STAT_OP_ON);

	/* start Status-FIFO timer */
	OUTB(dp, STAT_TX_TIMER_CTRL, TIM_START);
	OUTB(dp, STAT_LEV_TIMER_CTRL, TIM_START);
	OUTB(dp, STAT_ISR_TIMER_CTRL, TIM_START);
}

static int
myk_init_port(struct gem_dev *dp)
{
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[port];
	uint32_t	ramsize;
	uint32_t	rxspace;
	int		i;
	uint32_t	reg;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));
	ASSERT(mutex_owned(&dp->intrlock));

	/*
	 * from SkY2PortStart() -- Initialize prefetc units
	 */
	/* Setup Rx prefetch unit */ /* ok */
	pp->rx_phys32hi = ~0U;	/* invalidate */
	pp->rx_desc_head = 0;
	pp->rx_desc_tail = 0;
	pp->rx_slot_tail = 0;
	myk_prefetch_init(dp, RXQADDR(port),
	    RX_DESC_BASE_DMA(dp), RX_DESC_SIZE - 1);


	/* Setup Tx prefetch unit */ /* ok */
	pp->tx_phys32hi = ~0U;	/* invalidate */
	pp->tx_mss = 0;		/* invalidate */
	pp->tx_hck_offsets = 0;	/* invalidate */
	pp->tx_vtag = 0;	/* invalidate */
	pp->tx_head = 0;
	pp->tx_tail = 0;
	myk_prefetch_init(dp,
	    TXQADDR(port), dp->tx_ring_dma, dp->gc.gc_tx_ring_size - 1);
#ifdef notyet
	/* setup watermarks and timers for low latency optimization */
	if (pp->low_latency) {
		OUTB(dp, STAT_FIFO_WM, 1);
		OUTB(dp, STAT_FIFO_ISR_WM, 1);
		OUTL(dp, STAT_LEV_TIMER_INI, 50);
		OUTL(dp, STAT_ISR_TIMER_INI, 10);
	}
#endif
	/*
	 * initialize mac
	 */
	if (!lp->asf_enabled) {
		/* 10Mbps half duplex without flow control */
		OUTW(dp, GMAC_REG(port, GM_GP_CTRL),
		    GM_GPCR_AU_DUP_DIS | GM_GPCR_AU_SPD_DIS |
		    GM_GPCR_AU_FCT_DIS | GM_GPCR_FC_TX_DIS |
		    GM_GPCR_FC_RX_DIS);
	}

	/*
	 * SkGeInitPort()
	 */

	/*
	 * SkGmInitMac()
	 */

	/* dummy read the Interrupt Source Register. */
	FLSHB(dp, MR_ADDR(port, GMAC_IRQ_SRC));

	/*
	 * SkGmResetCounter()
	 */
	reg = INW(dp, GMAC_REG(port, GM_PHY_ADDR));

	/* set MIB Clear Counter Mode */ /* ok */
	OUTW(dp, GMAC_REG(port, GM_PHY_ADDR), reg | GM_PAR_MIB_CLR);

	/* read all MIB Counters with Clear Mode set. */
	for (i = GM_MIB_CNT_BASE; i <= GM_TXE_FIFO_UR; i += 8) {
		(void) INW(dp, GMAC_REG(port, i));
#ifdef notyet
		/* ensure the reset is performed correctly */
#endif
	}

	/* clear MIB Clear Counter Mode */ /* ok */
	OUTW(dp, GMAC_REG(port, GM_PHY_ADDR), reg & ~GM_PAR_MIB_CLR);
	/* end of SkGmResetCounter() */

	/* setup Transmit Control Register. */	/* ok */
	OUTW(dp, GMAC_REG(port, GM_TX_CTRL), TX_COL_THR(TX_COL_DEF));

	/* setup Receive Control Register: disable FCS, promiscous mode */
	OUTW(dp, GMAC_REG(port, GM_RX_CTRL), GM_RXCR_CRC_DIS);	/* ok */

	/* setup Transmit Flow Control Register */
	OUTW(dp, GMAC_REG(port, GM_TX_FLOW_CTRL), 0xffff);	/* ok */

	/* setup Transmit Parameter Register */
	OUTW(dp, GMAC_REG(port, GM_TX_PARAM),
	    TX_JAM_LEN_VAL(TX_JAM_LEN_DEF) |
	    TX_JAM_IPG_VAL(TX_JAM_IPG_DEF) |
	    TX_IPG_JAM_DATA(TX_IPG_JAM_DEF) |
	    TX_BACK_OFF_LIM(TX_BOF_LIM_DEF));	/* ok */

	/* configure the Serial Mode Register for VLAN and longer MTU */
	reg = DATA_BLIND_VAL(DATA_BLIND_DEF) |
	    GM_SMOD_VLAN_ENA | IPG_DATA_VAL(IPG_DATA_DEF);	/* ok */

	if (dp->mtu > ETHERMTU || (dp->misc_flag & GEM_VLAN_SOFT)) {
		/* enable jumbo mode (Max. Frame Length = 9018) */
		reg |= GM_SMOD_JUMBO_ENA;	/* ok */
	}

#ifdef CONFIG_NEW_FLOW_CONTROL	/* notyet */
	/* if the chip has new flow control capability, enable it */
	if (lp->new_flow_control) {
		ret |= GM_NEW_FLOW_CTRL;
	}
#endif
	OUTW(dp, GMAC_REG(port, GM_SERIAL_MODE), reg);	/* ok */


	/* Set station address: later in set_rx_filter() */

	/* disable interrupts for counter overflows */
	OUTW(dp, GMAC_REG(port, GM_TX_IRQ_MSK), 0);	/* ok */
	OUTW(dp, GMAC_REG(port, GM_RX_IRQ_MSK), 0);	/* ok */
	OUTW(dp, GMAC_REG(port, GM_TR_IRQ_MSK), 0);	/* ok */

	/* end of SkGmInitMac() */

	/*
	 * from SkGeInitMacFifo() --  Initialize all MAC FIFOs
	 */
	/* Configure Rx MAC FIFO. */
	reg = GMF_OPER_ON | GMF_RX_F_FL_ON;	/* is GMF_RX_CTRL_DEF */
#ifdef CONFIG_YUKON_I
	if (lp->chip_id == CHIP_ID_YUKON_LITE) {
		reg &= ~GMF_RX_F_FL_ON;
	}
#endif
	OUTB(dp, MR_ADDR(port, RX_GMF_CTRL_T), GMF_RST_CLR);	/* ok */
	OUTW(dp, MR_ADDR(port, RX_GMF_CTRL_T), reg);	/* ok */

	/* set Rx GMAC FIFO Flush Mask (after clearing reset) */
	reg = GMR_FS_ANY_ERR;
	if (lp->chip_id == CHIP_ID_YUKON_XL &&
	    lp->chip_rev >= CHIP_REV_YU_XL_A0 &&
	    lp->chip_rev <= CHIP_REV_YU_XL_A3) {
		/* HWF_WA_DEV_4115 */
		reg = 0;
	}
	OUTW(dp, MR_ADDR(port, RX_GMF_FL_MSK), reg);	/* ok */

	/* default: 0x0a -> 56 bytes on Yukon-1 and 64 bytes on Yukon-2 */
	reg = 0xa;	/* is RX_GMF_FL_THR_DEF */
	if (IS_YUKON2(lp->chip_id) && lp->asf_enabled) {
		reg -= 2;
	} else if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
	    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
		/* HWF_WA_DEV_521 */
		reg = 0x178;
	} else {
		/*
		 * Because Pause Packet Truncation in GMAC is not working,
		 * we have to increase the Flush Threshold to 64 bytes
		 * in order to flush pause packets in Rx FIFO on Yukon-1
		 */
		reg++;
	}

	/* set Rx GMAC FIFO Flush Threshold (after clearing reset) */
	OUTW(dp, MR_ADDR(port, RX_GMF_FL_THR), reg);	/* ok */

	/* configure Tx GMAC FIFO */
	OUTB(dp, MR_ADDR(port, TX_GMF_CTRL_T), GMF_RST_CLR);	/* ok */
	OUTW(dp, MR_ADDR(port, TX_GMF_CTRL_T), GMF_OPER_ON);	/* ok */


	if (lp->chip_id == CHIP_ID_YUKON_EC_U ||
	    lp->chip_id == CHIP_ID_YUKON_EX ||
	    lp->chip_id >= CHIP_ID_YUKON_FE_P) {

		/* set Rx Pause Threshold */
		if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
		    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
			reg = 0xc4;
		} else if (lp->chip_id == CHIP_ID_YUKON_EC_U ||
		    lp->chip_id == CHIP_ID_YUKON_FE_P) {
			reg = 0x80;	/* ECU_ULPP */
		} else {
			reg = 0x05c0;	/* EXT_ULPP */
		}
		OUTW(dp, MR_ADDR(port, RX_GMF_UP_THR), reg);

		OUTW(dp, MR_ADDR(port, RX_GMF_LP_THR), 0x60); /* 768/8 */

		if ((lp->chip_id == CHIP_ID_YUKON_EX &&
		    lp->chip_rev != CHIP_REV_YU_EX_A0) ||
		    lp->chip_id >= CHIP_ID_YUKON_FE_P) {
			/* Yukon-Extreme BO and further Extreme devices */
			/* enable Store & Forward mode for TX */
			OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T), TX_STFW_ENA);

		} else if (dp->mtu > ETHERMTU ||
		    (dp->misc_flag & GEM_VLAN_SOFT)) {
			/* set Tx GMAC FIFO Almost Empty Threshold */
			OUTL(dp, MR_ADDR(port, TX_GMF_AE_THR),
			    (0x0400 << 16) | 0x0070);
			/* disable Store & Forward mode for TX */
			OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T), TX_STFW_DIS);

		} else {
			/* enable Store & Forward mode for TX */
			OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T), TX_STFW_ENA);
		}
	}

	if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
	    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
		/* HWF_WA_DEV_520 */
		/* disable dynamic watermark */
		reg = INW(dp, MR_ADDR(port, TX_GMF_EA));
		OUTW(dp, MR_ADDR(port, TX_GMF_EA), reg & ~TX_DYN_WM_ENA);
	}
	/* end of SkGeInitMacFifo() */ /* ok */
#ifdef NEVER
	/* from SkGePortVlan() */ /* was gone */
	if ((dp->misc_flag & GEM_VLAN_HARD)) {
		OUTL(dp, MR_ADDR(port, RX_GMF_CTRL_T), RX_VLAN_STRIP_ON);
		OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T), TX_VLAN_TAG_ON);
	} else {
		OUTL(dp, MR_ADDR(port, RX_GMF_CTRL_T), RX_VLAN_STRIP_OFF);
		OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T), TX_VLAN_TAG_OFF);
	}
#endif
	/*
	 * From SkGeInitRamBufs()
	 * Determine available ram buffer space (in 4K blocks).
	 */
	ramsize = INB(dp, B2_E_0);
	if (ramsize == 0) {
		ramsize = 128 / 4;
	}
	if (lp->chip_id == CHIP_ID_YUKON_FE ||
	    lp->chip_id == CHIP_ID_YUKON_FE_P ||
	    (lp->chip_id == CHIP_ID_YUKON_EC && lp->chip_cap == 2)) {
		ramsize = 4 / 4;
	}

	/* Give transmitter one third (rounded up) */
	rxspace = ramsize - (ramsize + 2) / 3;

	if (ramsize > 0) {
		myk_init_ramqueue(dp, RXQADDR(port), 0, rxspace);
		myk_init_ramqueue(dp, TXQADDR(port), rxspace, ramsize);
		/* make sure tx SyncQ is disabled */
		OUTB(dp, RB_ADDR(port == 0 ? Q_XS1 : Q_XS2, RB_CTRL),
		    RB_RST_SET);
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: ram buffer %dK (rx:%dK, tx:%dK)",
	    dp->name, __func__,
	    ramsize*4, rxspace*4, ramsize*4 - rxspace*4));

	/* end of SkGeInitRamBufs() */
#if 0
	/*
	 * Disable Force Sync bit and Alloc bit in Tx RAM interface
	 * arbiter as we don't use Sync Tx queue.
	 */
	OUTB(dp, MR_ADDR(port, TXA_CTRL),
	    TXA_DIS_FSYNC | TXA_DIS_ALLOC | TXA_STOP_RC);
#endif
	/*
	 * from SkGeInitBmu()
	 */
	/* Rx Queue: Release all local resets and set the watermark */
	OUTL(dp, Q_ADDR(RXQADDR(port), Q_CSR), BMU_CLR_RESET);
	OUTL(dp, Q_ADDR(RXQADDR(port), Q_CSR), BMU_OPER_INIT);
	OUTL(dp, Q_ADDR(RXQADDR(port), Q_CSR), BMU_FIFO_OP_ON);

	OUTW(dp, Q_ADDR(RXQADDR(port), Q_WM), lp->is_pcie ? 0x80 : 0x600);

	if (lp->chip_id == CHIP_ID_YUKON_EC_U &&
	    lp->chip_rev >= CHIP_REV_YU_EC_U_A0) {
		/* MAC Rx RAM Read is controlled by hardware */
		OUTL(dp, Q_ADDR(RXQADDR(port), Q_F), F_M_RX_RAM_DIS);
	}

	/* XXX -- no Tx synchronous queue */

	/* Setup Tx asynchronous queue */
	OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), BMU_CLR_RESET);
	OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), BMU_OPER_INIT);
	OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), BMU_FIFO_OP_ON);

	OUTW(dp, Q_ADDR(TXQADDR(port), Q_WM),  0x600);

	if (lp->chip_id == CHIP_ID_YUKON_EC_U &&
	    lp->chip_rev == CHIP_REV_YU_EC_U_A0) {
		/* fix for Yukon-EC Ultra: set BMU FIFO level */
		OUTW(dp, Q_ADDR(TXQADDR(port), Q_AL), 0x01a0);
	}
	/* end of SkGeInitBmu() */

	if (lp->chip_id == CHIP_ID_YUKON_EX ||
	    lp->chip_id == CHIP_ID_YUKON_SUPR) {	/* ok */
		/*
		 * Disable flushing of non ASF packets;
		 * must be done after initializing the BMUs;
		 * drivers without ASF support should do this too, otherwise
		 * it may happen that they cannot run on an ASF devices;
		 * remember that the MAC FIFO isn't reset during
		 * initialization.
		 */
		OUTL(dp, MR_ADDR(port, RX_GMF_CTRL_T),
		    BIT_22 /* RXMF_TCTL_MACSEC_FLSH_DIS */);
	}

	if (lp->chip_id >= CHIP_ID_YUKON_SUPR) {
		/* Enable RX Home Address & Routing Header checksum fix */
		OUTW(dp, RX_GMF_FL_CTRL,
		    RX_IPV6_SA_MOB_ENA | RX_IPV6_DA_MOB_ENA);

		/* Enable TX Home Address & Routing Header checksum fix */
		OUTL(dp, 0x6b8 /* TBMU_TEST */,
		    BIT_25 /* TBMU_TEST_HOME_ADD_FIX_EN */  |
		    BIT_27 /* TBMU_TEST_ROUTING_ADD_FIX_EN */);
	}

	/*
	 * end of SkGeInitPort()
	 */
	/* disable Rx GMAC FIFO Flush Mode */
	OUTB(dp, MR_ADDR(port, RX_GMF_CTRL_T), GMF_RX_F_FL_OFF);

	/* SkGeRxCsum() */
	/* XXX - FE_P chipset need to enable rx checksum always. */
	OUTL(dp, Q_ADDR(RXQADDR(port), Q_CSR), BMU_ENA_RX_CHKSUM);

	/* put first command into rx descriptor ring */
	myk_rx_set_checksum(dp);

	return (GEM_SUCCESS);
}

/*
 * from SkMacRxTxEnable
 */
static int
myk_start_port(struct gem_dev *dp)
{
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));

	/* Enable Transmit FIFO Underrun */
	OUTB(dp, MR_ADDR(port, GMAC_IRQ_MSK), GMAC_DEF_MSK);

	OUTW(dp, GMAC_REG(port, GM_GP_CTRL),
	    INW(dp, GMAC_REG(port, GM_GP_CTRL)) |
	    GM_GPCR_RX_ENA | GM_GPCR_TX_ENA);

	/* Enable interrupts for the port */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* Enable interrupts from phy/mac for port */
		lp->descset[port].hwe_imsk =
		    ((port == 0) ? Y2_HWE_L1_MASK : Y2_HWE_L2_MASK) |
		    Y2_IS_TIST_OV | Y2_IS_MST_ERR | Y2_IS_IRQ_STAT |
		    Y2_IS_PCI_EXP;

		lp->descset[port].imsk |=
		    PORTIRQ_MSK(port) | Y2_IS_HW_ERR | Y2_IS_STAT_BMU;

		OUTL(dp, B0_HWE_IMSK,
		    lp->descset[0].hwe_imsk | lp->descset[1].hwe_imsk);
		FLSHL(dp, B0_HWE_IMSK);
		OUTL(dp, B0_IMSK,
		    lp->descset[0].imsk | lp->descset[1].imsk);
		FLSHL(dp, B0_IMSK);
	}

	return (GEM_SUCCESS);
}

static int
myk_stop_port(struct gem_dev *dp)
{
	int		i;
	uint32_t	reg;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;
	uint_t		rxq;
	uint32_t	XaCsr;
	uint32_t	CsrStart;
	uint32_t	CsrStop;
	uint32_t	CsrIdle;
	uint32_t	CsrTest;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rxq = RXQADDR(port);
	/*
	 * set the proper values of Q_CSR register layout depending
	 * on the chip
	 */
	if (IS_YUKON2(lp->chip_id)) {
		CsrStart = BMU_START;
		CsrStop = BMU_STOP;
		CsrIdle = BMU_IDLE;
		CsrTest = BMU_IDLE;
#ifdef CONFIG_YUKON_I
	} else {
		CsrStart = CSR_START;
		CsrStop = CSR_STOP;
		CsrIdle = CSR_SV_IDLE;
		CsrTest = CSR_SV_IDLE | CSR_STOP;
#endif
	}

	DPRINTF(2, (CE_CONT, "%s: %s: (0) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));
	/*
	 * Stop Tx
	 */
	if (!lp->asf_enabled) {
		/* SkMacRxTxDisable() */
		reg = INW(dp, GMAC_REG(port, GM_GP_CTRL));
		reg &= ~(GM_GPCR_TX_ENA | GM_GPCR_RX_ENA);
		OUTW(dp, GMAC_REG(port, GM_GP_CTRL), reg);
	}

	/* stop transmit queue */
	OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), CsrStop);

	DPRINTF(2, (CE_CONT, "%s: %s: (1) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));

	/*
	 * If the BMU is in the reset state CSR_STOP will terminate
	 * immediately.
	 */
	for (i = 0; i < 100; i++) {
		XaCsr = INL(dp, Q_ADDR(TXQADDR(port), Q_CSR));
		if ((XaCsr & CsrTest) == CsrIdle) {
			break;
		}

		if ((XaCsr & CsrStop) != 0) {
			OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), CsrStart);
			OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), CsrStop);
		}

		drv_usecwait(100);
	}

	if (!lp->asf_enabled) {
		/* SkGmHardRst() */
		OUTB(dp, MR_ADDR(port, GPHY_CTRL), GPC_RST_SET);

		/* Workaround for shared GMAC reset */
		if (lp->ports > 1 && port == 0 &&
		    lp->chip_id == CHIP_ID_YUKON_XL &&
		    lp->chip_rev == CHIP_REV_YU_XL_A0 &&
		    lp->descset[1].dp->mac_active) {
			/* EMPTY */
		} else {
			OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_RST_SET);
		}
	}

	/* Disable Force Sync bit and Enable Alloc bit */
	OUTB(dp, MR_ADDR(port, TXA_CTRL),
	    TXA_DIS_FSYNC | TXA_DIS_ALLOC | TXA_STOP_RC);

	/* Stop Interval Timer and Limit Counter of Tx Arbiter */
	OUTL(dp, MR_ADDR(port, TXA_ITI_INI), 0);
	OUTL(dp, MR_ADDR(port, TXA_LIM_INI), 0);

	/* Perform a local reset of the port's Tx path */
	if (IS_YUKON2(lp->chip_id)) {
		/* Reset the PCI FIFO of the async Tx queue */
		OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR),
		    BMU_RST_SET | BMU_FIFO_RST);

		/* Reset the Tx prefetch units */
		OUTB(dp, Y2_QADDR(TXQADDR(port),
		    PREF_UNIT_CTRL_REG), PREF_UNIT_RST_SET);
#ifdef CONFIG_YUKON_I
	} else {
		/* Reset the PCI FIFO of the async Tx queue */
		OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), CSR_SET_RESET);
#endif
	}

	/* Reset the RAM Buffer async TX queue */
	OUTL(dp, RB_ADDR(TXQADDR(port), RB_CTRL), RB_RST_SET);

	/* do the reset only if ASF is not enabled */
	if (!lp->asf_enabled) {
		/* Reset Tx MAC FIFO */
		OUTB(dp, MR_ADDR(port, TX_GMF_CTRL_T), GMF_RST_SET);
	}

	/* set Pause Off */
	OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_PAUSE_OFF);

	DPRINTF(2, (CE_CONT, "%s: %s: (3) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));

	/*
	 * Stop Rx
	 */
	if (IS_YUKON2(lp->chip_id)) {
		/*
		 * The RX Stop command will not work for Yukon-2 if the BMU
		 * does not reach the end of packet and since we can't make
		 * sure that we have incoming data, we must reset the BMU
		 * while it is not doing a DMA transfer. Since it is possible
		 * that the RX path is still active, the RX RAM buffer will
		 * be stopped first, so any possible incoming data will not
		 * trigger a DMA. After the RAM buffer is stopped, the BMU
		 * is polled until any DMA in progress is ended and only then
		 * it will be reset.
		 */

		/* disable the RAM Buffer receive queue */
		OUTB(dp, RB_ADDR(rxq, RB_CTRL), RB_DIS_OP_MD);

		i = 0;
		while (INB(dp, RB_ADDR(rxq, Q_RSL))
		    != INB(dp, RB_ADDR(rxq, Q_RL))) {
			if (i++ > 200) {
				cmn_err(CE_WARN,
				    "!%s: %s: timeout: receiver stop failed",
				    dp->name, __func__);
				break;
			}
			drv_usecwait(10);
		}

		/*
		 * If the Rx side is blocked, the above loop cannot terminate.
		 * But, if there was any traffic it should be terminated, now.
		 * However, stop the Rx BMU and the Prefetch Unit !
		 */
		OUTL(dp, Q_ADDR(rxq, Q_CSR), BMU_RST_SET | BMU_FIFO_RST);
		/* reset the Rx prefetch unit */
		OUTB(dp, Y2_QADDR(RXQADDR(port),
		    PREF_UNIT_CTRL_REG), PREF_UNIT_RST_SET);
#ifdef CONFIG_YUKON_I
	} else {
		/*
		 * The RX Stop Command will not terminate if no buffers
		 * are queued in the RxD ring. But it will always reach
		 * the Idle state. Therefore we can use this feature to
		 * stop the transfer of received packets.
		 */
		/* stop the port's receive queue */
		OUTL(dp, Q_ADDR(rxq, Q_CSR), CSR_STOP);

		i = 100;
		do {
			/* read the status */
			reg = INL(dp, Q_ADDR(rxq, Q_CSR));

			/* timeout if i==0 (bug fix for #10748) */
			if (--i == 0) {
				cmn_err(CE_CONT, "!%s: %s: timeout: stop rx",
				    dp->name, __func__);
				break;
			}
			/*
			 * Because of the ASIC problem report entry from
			 * 21.08.1998 it is required to wait until CSR_STOP
			 * is reset and CSR_SV_IDLE is set. (valid for
			 * GENESIS only)
			 */
		} while ((reg & (CSR_SV_IDLE | CSR_STOP)) != CSR_SV_IDLE);
		/* The path data transfer activity is fully stopped now */

		/* Perform a local reset of the port's Rx path */
		/* Reset the PCI FIFO of the Rx queue */
		OUTL(dp, Q_ADDR(rxq, Q_CSR), CSR_SET_RESET);
#endif
	}

	DPRINTF(2, (CE_CONT, "%s: %s: (10) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));

	/* Reset the RAM Buffer receive queue */
	OUTB(dp, RB_ADDR(rxq, RB_CTRL), RB_RST_SET);
	DPRINTF(2, (CE_CONT, "%s: %s: (11) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));

	if (!lp->asf_enabled) {
		/* Reset Rx MAC FIFO */
		OUTB(dp, MR_ADDR(port, RX_GMF_CTRL_T), GMF_RST_SET);
	}
	/* end of SkGeStopPort() */

	DPRINTF(2, (CE_CONT, "%s: %s: (20) hwidx:%d",
	    dp->name, __func__, INW(dp, STAT_PUT_IDX)));

	/* Disable port IRQ */
#ifdef notdef
	lp->descset[port].hwe_imsk = 0;
	lp->descset[port].imsk = 0;
#endif
	OUTL(dp, B0_HWE_IMSK,
	    lp->descset[0].hwe_imsk | lp->descset[1].hwe_imsk);
	FLSHL(dp, B0_HWE_IMSK);

	OUTL(dp, B0_IMSK, lp->descset[0].imsk | lp->descset[1].imsk);
	FLSHL(dp, B0_IMSK);

	if (lp->ports <= 1) {
		/* we can do full reset */
		lp->initialized = -1;
	}
	(void) myk_reset_port(dp);

	return (GEM_SUCCESS);
}

static int
myk_attach_port(struct gem_dev *dp)
{
	int		i;
	uint16_t	tmp16;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	/* get the MAC address */
	for (i = 0; i < ETHERADDRL; i += 2) {
		tmp16 = INW(dp, B2_MAC_1 + port * 8 + i);
		dp->dev_addr.ether_addr_octet[i + 0] = (uint8_t)tmp16;
		dp->dev_addr.ether_addr_octet[i + 1] = (uint8_t)(tmp16 >> 8);
	}

#ifndef GEM_CONFIG_JUMBO_FRAME
	dp->rx_buf_len =
	    ROUNDUP2(dp->mtu + sizeof (struct ether_header) + 4, 8);
#endif

#ifdef CONFIG_CKSUM_OFFLOAD
#ifdef CONFIG_LSO
	dp->misc_flag |= GEM_LSO;
#endif /* CONFIG_LSO */
	if (lp->tx_full_cksum) {
		/* experimental */
		dp->misc_flag |= GEM_CKSUM_FULL_IPv4 | GEM_CKSUM_FULL_IPv6;
	} else {
		dp->misc_flag |= GEM_CKSUM_PARTIAL;
	}
#endif /* CONFIG_CKSUM_OFFLOAD */

#ifdef CONFIG_VLAN_HW
	if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
	    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
		dp->misc_flag |= GEM_VLAN_SOFT;
	} else {
		dp->misc_flag |= GEM_VLAN_HARD;
	}
#else
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	if (lp->ports > 1) {
		dp->misc_flag |= GEM_SOFTINTR;
	}

	/* led control */
	if (lp->chip_id == CHIP_ID_YUKON_EC_U ||
	    lp->chip_id == CHIP_ID_YUKON_EX ||
	    lp->chip_id >= CHIP_ID_YUKON_FE_P) {
		/* LED Configuration is stored in GPIO */
		lp->descset[port].blink_ctrl =
		    CFG_LED_MODE(INB(dp, B2_GP_IO));
	} else {
		lp->descset[port].blink_ctrl =
		    CFG_LED_MODE(INB(dp, B2_Y2_HW_RES));
	}

	return (GEM_SUCCESS);
}

#ifdef GEM_CONFIG_JUMBO_FRAME
static void
myk_fixup_params(struct gem_dev *dp)
{
	int		i;
	uint16_t	tmp16;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	dp->rx_buf_len =
	    ROUNDUP2(dp->mtu + sizeof (struct ether_header) + 4, 8);
}
#endif

static uint32_t
myk_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_be(addr, ETHERADDRL) & 0x3f);
}

static int
myk_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint16_t	reg;
	uint16_t	mode;
	uint64_t	hash;
	uint8_t		mac[ETHERADDRL];
	uint_t		port = PORT(dp);

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	hash = 0ULL;
	bcopy(dp->cur_addr.ether_addr_octet, mac, ETHERADDRL);

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		mode = GM_RXCR_UCF_ENA | GM_RXCR_MCF_ENA;
		bzero(mac, ETHERADDRL);
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscuous */
		mode = 0;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* all multicast */
		mode = GM_RXCR_UCF_ENA;
	} else {
		mode = GM_RXCR_UCF_ENA | GM_RXCR_MCF_ENA;
		for (i = 0; i < dp->mc_count; i++) {
			hash |= 1ULL << dp->mc_list[i].hash;
		}
	}
	/* we temporally use promiscious mode */
	reg = INW(dp, GMAC_REG(port, GM_RX_CTRL))
	    & ~(GM_RXCR_UCF_ENA | GM_RXCR_MCF_ENA);
	OUTW(dp, GMAC_REG(port, GM_RX_CTRL), reg);

	if (mode & GM_RXCR_MCF_ENA) {
		/* update multicast filter */
		OUTW(dp, GMAC_REG(port, GM_MC_ADDR_H1), hash);
		OUTW(dp, GMAC_REG(port, GM_MC_ADDR_H2), hash >> 16);
		OUTW(dp, GMAC_REG(port, GM_MC_ADDR_H3), hash >> 32);
		OUTW(dp, GMAC_REG(port, GM_MC_ADDR_H4), hash >> 48);
	}

	/* update ethernet mac addresss */
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(dp, B2_MAC_1 + port * 8 + i, mac[i]);
		OUTB(dp, B2_MAC_2 + port * 8 + i, mac[i]);
	}

	/* virtual address for data */
	myk_set_macaddr(dp, port, GM_SRC_ADDR_2L,
	    dp->cur_addr.ether_addr_octet);

	/* physical address: used for pause frames */
	myk_set_macaddr(dp, port, GM_SRC_ADDR_1L,
	    dp->cur_addr.ether_addr_octet);

	/* update control register */
	OUTW(dp, GMAC_REG(port, GM_RX_CTRL), reg | mode);

	return (GEM_SUCCESS);
}

static int
myk_set_media(struct gem_dev *dp)
{
	uint16_t	val;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	/* get current setting in GM_GP_CTRL register */
	val = INW(dp, GMAC_REG(port, GM_GP_CTRL));

	DPRINTF(0, (CE_CONT, "!%s: %s: called, gpcr:%b",
	    dp->name, __func__, val, GPCR_BITS));

#ifdef CONFIG_AUTO_MODE
	if (!dp->anadv_autoneg) {
#endif
		/* disable auto mode selection first */
		val |= GM_GPCR_AU_DUP_DIS | GM_GPCR_AU_SPD_DIS;
		OUTW(dp, GMAC_REG(port, GM_GP_CTRL), val);
		FLSHW(dp, GMAC_REG(port, GM_GP_CTRL));

		/* select speed, flow control and duplex mode manually */
		val &= ~(GM_GPCR_GIGS_ENA | GM_GPCR_SPEED_100);
		switch (dp->speed) {
		case GEM_SPD_1000:
			val |= GM_GPCR_GIGS_ENA | GM_GPCR_SPEED_100;
			break;

		case GEM_SPD_100:
			val |= GM_GPCR_SPEED_100;
			break;

		case GEM_SPD_10:
			break;
		}

		/* select duplex mode */
		if (dp->full_duplex) {
			val |= GM_GPCR_DUP_FULL;
		} else {
			val &= ~GM_GPCR_DUP_FULL;
		}
#ifdef CONFIG_AUTO_MODE
	} else {
		/* select speed and duplex mode automatically */
		val &= ~(GM_GPCR_AU_DUP_DIS | GM_GPCR_AU_SPD_DIS);
	}
#endif

	/*
	 * select flow control manually
	 */
	val |= GM_GPCR_AU_FCT_DIS;
	val &= ~(GM_GPCR_FC_TX_DIS | GM_GPCR_FC_RX_DIS);
	switch (dp->flow_control) {
	case FLOW_CONTROL_NONE:
		/* disable Rx and Tx flow-control */
		val |= GM_GPCR_FC_TX_DIS | GM_GPCR_FC_RX_DIS;
		break;

	case FLOW_CONTROL_SYMMETRIC:
		break;

	case FLOW_CONTROL_TX_PAUSE:
		/* disable Rx flow-control only */
		val |= GM_GPCR_FC_RX_DIS;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		/* disable Tx flow-control only */
		val |= GM_GPCR_FC_TX_DIS;
		break;
	}

	if (dp->flow_control != FLOW_CONTROL_NONE) {
		OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_PAUSE_ON);
	} else {
		OUTB(dp, MR_ADDR(port, GMAC_CTRL), GMC_PAUSE_OFF);
	}

	OUTW(dp, GMAC_REG(port, GM_GP_CTRL), val);
	FLSHW(dp, GMAC_REG(port, GM_GP_CTRL));

	DPRINTF(0, (CE_CONT, "!%s: %s: new gpcr:%b",
	    dp->name, __func__, val, GPCR_BITS));

	/* WA for dev. #4.209 */
	if (lp->chip_id == CHIP_ID_YUKON_EC_U &&
	    lp->chip_rev == CHIP_REV_YU_EC_U_A1) {
		/* enable/disable Store & Forward mode for TX */
		OUTL(dp, MR_ADDR(port, TX_GMF_CTRL_T),
		    (dp->speed != GEM_SPD_1000) ? TX_STFW_ENA : TX_STFW_DIS);
	}

	return (GEM_SUCCESS);
}

static int
myk_get_stats(struct gem_dev *dp)
{
	/* not implemented yet */
	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
myk_tx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int			i;
#ifdef CONFIG_VLAN_HW
	uint32_t		vtag;
#endif
	int			used;
	uint64_t		addr;
	uint32_t		phys32hi;
	uint32_t		mark;
	struct myk_dev		*lp = dp->private;
	struct myk_port		*pp = &lp->descset[PORT(dp)];
	ddi_dma_cookie_t	*dcp;
	struct msk_tx_desc	*tdp;
	uint32_t		offsets;
	uint32_t		hck_start;
	uint32_t		hck_stuff;
	uint32_t		mss;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum: %d, slot %d, frags: %d flag: %llx",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif

#define	GET_TX_LE(dp, s, n)	\
	&TXDESC((dp)->tx_ring)[SLOT((s) + (n)++, (dp)->gc.gc_tx_ring_size)]

	used = 0;
	dcp = dmacookie;
	mark = HW_OWNER;

#ifdef CONFIG_VLAN_HW
	vtag = (flag & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		mark |= INS_VLAN;
	}
#endif
	/* Check for TCP Large Segmentation Offload */
	mss = (flag & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;
	hck_start = (flag & GEM_TXFLAG_HCKSTART) >> GEM_TXFLAG_HCKSTART_SHIFT;
	hck_stuff = (flag & GEM_TXFLAG_HCKSTUFF) >> GEM_TXFLAG_HCKSTUFF_SHIFT;

	if (mss) {
		/* tcp large send offloading */
		DPRINTF(2, (CE_CONT,
		    "!%s: %s: hckstart:%d, hckstuff:%d, mss:%d, %s",
		    dp->name, __func__,
		    hck_start, hck_stuff, mss,
		    (flag & GEM_TXFLAG_TCP) ? "tcp" : ""));

		mark |= OP_LARGESEND;

		if (!lp->tx_full_cksum) {
			mss += hck_start +
			    ((flag & GEM_TXFLAG_TCPHLEN) >>
			    GEM_TXFLAG_TCPHLEN_SHIFT) * 4;
		}

		if (mss != pp->tx_mss) {
			/* allocate a new tx desc entry */
			tdp = GET_TX_LE(dp, slot, used);

			tdp->msk_addr = LE_32(mss);
			if (lp->tx_full_cksum) {
				tdp->msk_control = LE_32(OP_MSS | HW_OWNER);
			} else {
				tdp->msk_control = LE_32(OP_LRGLEN | HW_OWNER);
			}

			pp->tx_mss = mss;
		}
	} else if (offsets = ((hck_start << 16) | hck_stuff)) {
		/* tcp/udp partial checksum offloading for old yukon2 nics */
		DPRINTF(4, (CE_CONT,
		    "!%s: %s: partial offload (%s), offsets:%08x",
		    dp->name, __func__,
		    (flag & GEM_TXFLAG_TCP) ? "tcp": "udp", offsets));

		mark |= OP_PACKET | CALSUM | WR_SUM | INIT_SUM | LOCK_SUM;
		if (flag & GEM_TXFLAG_UDP) {
			mark |= UDPTCP;
		}

		if (offsets != pp->tx_hck_offsets) {
			/* we use a new tx desc for hwcksum */
			tdp = GET_TX_LE(dp, slot, used);

			/*
			 * number of packets should be 1,
			 * use initial checksum in the packet area
			 */
			tdp->msk_addr = LE_32(offsets);
			tdp->msk_control = LE_32(
			    OP_TCPLISW | HW_OWNER | (1 << 16) | 0);

			pp->tx_hck_offsets = offsets;
		}
	} else if (flag & (GEM_TXFLAG_TCP | GEM_TXFLAG_UDP)) {
		/* tcp/udp full checksum offloading for latest yukon2 nics */
		/* advanced checksum for EXTREME or FE+ or later */
		DPRINTF(4, (CE_CONT, "!%s: %s: full offload (%s)",
		    dp->name, __func__,
		    (flag & GEM_TXFLAG_TCP) ? "tcp": "udp"));

		mark |= OP_PACKET | CALSUM;
	} else {
		/* no offloading */
		mark |= OP_PACKET;
	}

	for (i = 0; i < frags; i++) {
		addr = dcp->dmac_laddress;
		phys32hi = (uint32_t)(addr >> 32);
#ifdef CONFIG_VLAN_HW
		if (phys32hi != pp->tx_phys32hi ||
		    (vtag && vtag != pp->tx_vtag)) {
			/* change segment addresss and/or vtag */
			tdp = GET_TX_LE(dp, slot, used);
			tdp->msk_addr = LE_32(phys32hi);
			if (vtag) {
				uint32_t	tmp0;

				tmp0 = OP_ADDR64 | HW_OWNER | OP_VLAN
				    | BSWAP_16(vtag);
				tdp->msk_control = LE_32(tmp0);
				pp->tx_vtag = vtag;
				vtag = 0;
			} else {
				tdp->msk_control = LE_32(OP_ADDR64 | HW_OWNER);
			}

			if ((addr + dcp->dmac_size) >> 32 == phys32hi) {
				pp->tx_phys32hi = phys32hi;
			} else {
				pp->tx_phys32hi = ~0U;	/* invalid */
			}
		}
#else
		if (phys32hi != pp->tx_phys32hi) {
			/* change segment addresss */
			tdp = GET_TX_LE(dp, slot, used);
			tdp->msk_addr = LE_32(phys32hi);
			tdp->msk_control = LE_32(OP_ADDR64 | HW_OWNER);

			if ((addr + dcp->dmac_size) >> 32 == phys32hi) {
				pp->tx_phys32hi = phys32hi;
			} else {
				pp->tx_phys32hi = ~0U;	/* invalid */
			}
		}
#endif

		tdp = GET_TX_LE(dp, slot, used);
		addr = (uint32_t)addr;
		mark |= (uint32_t)dcp->dmac_size;
		tdp->msk_addr = LE_32(addr);
		tdp->msk_control = LE_32(mark);

		mark = OP_BUFFER | HW_OWNER;
		dcp++;
	}

	/* close the fragment list */
	tdp->msk_control |= LE_32(EOP);

	return (used);

#undef	GET_TX_LE
}

static void
myk_tx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[port];

	DPRINTF(1, (CE_CONT, "!%s: %s: start_slot %d, nslot %d",
	    dp->name, __func__, start_slot, nslot));

	/* update tail position */
	pp->tx_tail = SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size);

	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	OUTW(dp, Y2_QADDR(TXQADDR(port), PREF_UNIT_PUT_IDX_REG), pp->tx_tail);
}

static void
myk_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
#if DEBUG_LEVEL > 1
	int			i;
#endif
	uint64_t		addr;
	uint32_t		phys32hi;
	struct myk_dev		*lp = dp->private;
	struct myk_port		*pp = &lp->descset[PORT(dp)];
	struct msk_rx_desc	*rdp;
	uint32_t		tmp;

	/*
	 * we always consume two descriptors
	 */
#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT, "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	addr = dmacookie->dmac_laddress;
	phys32hi = (uint32_t)(addr >> 32);
	if (pp->rx_phys32hi != phys32hi) {
		/* allocate a new rx desc entry */
		rdp = &RXDESC(RX_DESC_BASE(dp))[
		    SLOT(pp->rx_desc_tail, RX_DESC_SIZE)];
		pp->rx_desc_tail++;

		rdp->msk_addr = LE_32(phys32hi);
		rdp->msk_control = LE_32(OP_ADDR64 | HW_OWNER);

		pp->rx_phys32hi = phys32hi;
	}

	/* allocate a new rx desc entry */
	rdp = &RXDESC(RX_DESC_BASE(dp))[SLOT(pp->rx_desc_tail, RX_DESC_SIZE)];
	pp->rx_desc_tail++;

	rdp->msk_addr = LE_32((uint32_t)addr);
	tmp = OP_PACKET | HW_OWNER | (uint32_t)dmacookie->dmac_size;
	rdp->msk_control = LE_32(tmp);
}

static void
myk_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[port];

	DPRINTF(2, (CE_CONT, "!%s: %s: start:%d, nslot:%d, rx_desc(%d, %d)",
	    dp->name, __func__, start_slot, nslot,
	    pp->rx_desc_head, pp->rx_desc_tail));

	myk_rx_desc_dma_sync(dp,
	    SLOT(pp->rx_desc_head, RX_DESC_SIZE),
	    pp->rx_desc_tail - pp->rx_desc_head,
	    DDI_DMA_SYNC_FORDEV);

	pp->rx_desc_head = pp->rx_desc_tail;

	OUTW(dp, Y2_QADDR(RXQADDR(port), PREF_UNIT_PUT_IDX_REG),
	    SLOT(pp->rx_desc_tail, RX_DESC_SIZE));
}

#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
myk_tx_desc_head(struct gem_dev *dp)
{
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[PORT(dp)];

	return (pp->tx_head);
}
#else
static uint_t
myk_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[PORT(dp)];

	slot = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);

	if (INSIDE(slot, pp->tx_head, pp->tx_tail)) {
		return (0);
	}
#ifdef DEBUG_TX_TIMEOUT
	if (dp->tx_desc_head > 100000) {
		return (0);
	}
#endif

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d, [%d, %d]",
	    dp->name, __func__, slot, pp->tx_head, pp->tx_tail));

	return (GEM_TX_DONE);
}
#endif

static uint64_t
myk_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint64_t		flag = GEM_RX_DONE;
	struct myk_dev		*lp = dp->private;
	struct myk_port		*pp = &lp->descset[PORT(dp)];
	uint32_t		count;
	struct rx_soft_desc	*rdp;

	ASSERT(ndesc == 1);

	if (slot == pp->rx_slot_tail) {
		/* not received yet */
		return (0);
	}

	rdp = &pp->rx_slot[slot];
	count = (rdp->rx_stat & GMR_FS_LEN_MSK) >> 16;

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: slot:%d, len:0x%x, stat:0x%x, cksum:0x%x, "
	    "tag:0x%x, count:0x%x",
	    dp->name, __func__, slot, rdp->rx_len, rdp->rx_stat,
	    rdp->rx_cksum, rdp->rx_tag, count));

	if (rdp->rx_stat & GMR_FS_VLAN) {
		count -= 4;
	}

	if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
	    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
		/*
		 * This chip has hardware problems that generates bogus
		 * status. So do only marginal checking and expect higher
		 * level protocols to handle crap frames.
		 */
		if ((rdp->rx_len & GEM_RX_LEN) != count) {
			goto ok;
		}
	}

	if ((rdp->rx_stat & (GMR_FS_ANY_ERR | GMR_FS_RX_OK))
	    != GMR_FS_RX_OK) {
		/* ignore good flow control packets */
		if ((rdp->rx_stat & GMR_FS_GOOD_FC) == 0) {
			DPRINTF(0, (CE_NOTE,
			    "!%s: %s: rx error, status 0x%x length %d",
			    dp->name, __func__, rdp->rx_stat,
			    rdp->rx_len & GEM_RX_LEN));

			/* error packet */
			dp->stats.errrcv++;

			if (rdp->rx_stat &
			    (GMR_FS_LONG_ERR | GMR_FS_UN_SIZE)) {
				dp->stats.frame_too_long++;
				/* dp->stats.runt++; */
			} else if (rdp->rx_stat & GMR_FS_FRAGMENT) {
				dp->stats.frame++;
			} else if (rdp->rx_stat & GMR_FS_CRC_ERR) {
				dp->stats.crc++;
			} else if (rdp->rx_stat & GMR_FS_RX_FF_OV) {
				dp->stats.overflow++;
			}
		}
		return (flag | GEM_RX_ERR);
	}
ok:
	if (!lp->rx_full_cksum) {
		flag |= (((uint64_t)rdp->rx_cksum) << GEM_RX_CKSUM_SHIFT);
	}
	flag |= ((uint64_t)rdp->rx_tag) << GEM_RX_VTAG_SHIFT;
	rdp->rx_stat = 0;
	rdp->rx_cksum = 0;
	rdp->rx_tag = 0;

	return (flag | rdp->rx_len);
}

static void
myk_tx_desc_init(struct gem_dev *dp, int slot)
{
	/* EMPTY */
}

static void
myk_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[PORT(dp)];

	bzero(&pp->rx_slot[slot], sizeof (struct rx_soft_desc));
}

/*
 * Device depend interrupt handler
 */
static void
myk_hw_error(struct gem_dev *dp, uint_t port, uint32_t status)
{
	DPRINTF(0, (CE_NOTE, "!%s: hw error interrupt status 0x%x",
	    dp->name, status));

	if (status & Y2_IS_PAR_RD1) {
		cmn_err(CE_WARN,
		    "!%s: ram data read parity error", dp->name);
		/* Clear IRQ */
		OUTW(dp, RAM_BUFFER(port, B3_RI_CTRL), RI_CLR_RD_PERR);
	}

	if (status & Y2_IS_PAR_WR1) {
		cmn_err(CE_WARN,
		    "!%s: ram data write parity error", dp->name);
		OUTW(dp, RAM_BUFFER(port, B3_RI_CTRL), RI_CLR_WR_PERR);
	}

	if (status & Y2_IS_PAR_MAC1)  {
		cmn_err(CE_WARN,
		    "!%s: MAC parity error", dp->name);
		OUTB(dp, MR_ADDR(port, TX_GMF_CTRL_T), GMF_CLI_TX_PE);
	}

	if (status & Y2_IS_PAR_RX1) {
		cmn_err(CE_WARN,
		    "!%s: RX parity error", dp->name);
		OUTL(dp, Q_ADDR(RXQADDR(port), Q_CSR), BMU_CLR_IRQ_PAR);
	}

	if (status & Y2_IS_TCP_TXA1) {
		cmn_err(CE_WARN,
		    "!%s: TCP segmentation error", dp->name);
		OUTL(dp, Q_ADDR(TXQADDR(port), Q_CSR), BMU_CLR_IRQ_TCP);
	}
}

static boolean_t
myk_hw_intr(struct gem_dev *dp)
{
	boolean_t	fatal = B_FALSE;
	uint32_t	status;
	int		ce_level;
	uint_t		bus_err;

	status = INL(dp, B0_HWE_ISRC);

	if (status & Y2_IS_TIST_OV) {
		OUTB(dp, GMAC_TI_ST_CTRL, GMT_ST_CLR_IRQ);
	}

	if (status & (Y2_IS_MST_ERR | Y2_IS_IRQ_STAT)) {
		bus_err = myk_pci_read16(dp, PCI_CONF_STAT);
		cmn_err(CE_WARN, "!%s: pci hw error (0x%x)",
		    dp->name, bus_err);

		OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_ON);

		myk_pci_write16(dp, PCI_CONF_STAT, bus_err |
		    PCI_STAT_PERROR | PCI_STAT_S_SYSERR | PCI_STAT_R_MAST_AB |
		    PCI_STAT_R_TARG_AB | PCI_STAT_S_PERROR);

		OUTB(dp, B2_TST_CTRL1, TST_CFG_WRITE_OFF);
	}

	if (status & Y2_IS_PCI_EXP) {
		/* PCI-Express uncorrectable Error occurred */
		bus_err = myk_pci_read32(dp, PEX_UNC_ERR_STAT);

		/* clear the interrupt */
		OUTL(dp, B2_TST_CTRL1, TST_CFG_WRITE_ON);
		myk_pci_write32(dp, PEX_UNC_ERR_STAT, 0xffffffff);
		OUTL(dp, B2_TST_CTRL1, TST_CFG_WRITE_OFF);

		ce_level = CE_NOTE;
		if (bus_err & PEX_FATAL_ERRORS) {
			fatal = B_TRUE;
			OUTL(dp, B0_HWE_IMSK,
			    INL(dp, B0_HWE_IMSK) & ~Y2_IS_PCI_EXP);
			ce_level = CE_WARN;
		}
		cmn_err(ce_level, "!%s: pci express error (0x%b)",
		    dp->name, bus_err, PEX_UNC_STAT_BITS);
	}

	if (status & Y2_HWE_L1_MASK) {
		/* need to reset the port */
		myk_hw_error(dp, 0, status);
	}

	status >>= 8;
	if (status & Y2_HWE_L1_MASK) {
		/* need to reset the port */
		myk_hw_error(dp, 1, status);
	}

	return (fatal);
}

/* Interrupt from PHY */
static void
myk_phy_intr(struct gem_dev *dp)
{
	uint16_t	istatus;
	uint16_t	phystat;

	ASSERT(mutex_owned(&dp->intrlock));

	istatus = myk_mii_read(dp, PHY_MARV_INT_STAT);
	phystat = myk_mii_read(dp, PHY_MARV_PHY_STAT);
#ifdef lint
	phystat = phystat;
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s status 0x%b 0x%b",
	    dp->name, __func__,
	    istatus, PHY_MARV_INT_BITS, phystat, PHY_MARV_PHY_STAT_BITS));

	if (istatus & PHY_M_IS_AN_COMPL) {
		cmn_err(CE_NOTE, "!%s: %s: autonegotiation completed",
		    dp->name, __func__);
	} else {
		if (istatus & PHY_M_IS_LSP_CHANGE) {
			cmn_err(CE_NOTE, "!%s: %s: link speed changed",
			    dp->name, __func__);
		}

		if (istatus & PHY_M_IS_DUP_CHANGE) {
			cmn_err(CE_NOTE, "!%s: %s: duplex changed",
			    dp->name, __func__);
		}

		if (istatus & PHY_M_IS_LST_CHANGE) {
			cmn_err(CE_NOTE, "!%s: %s: link status changed",
			    dp->name, __func__);
		}
	}
}

static void
myk_mac_intr(struct gem_dev *dp, unsigned port)
{
	uint8_t		status;

	status = INB(dp, MR_ADDR(port, GMAC_IRQ_SRC));

	DPRINTF(0, (CE_CONT,  "!%s: %s: mac interrupt status 0x%x",
	    dp->name, __func__, status));

	if (status & GM_IS_RX_FF_OR) {
		/* ++sky2->net_stats.rx_fifo_errors; */
		OUTB(dp, MR_ADDR(port, RX_GMF_CTRL_T), GMF_CLI_RX_FO);
	}

	if (status & GM_IS_TX_FF_UR) {
		/* ++sky2->net_stats.tx_fifo_errors; */
		OUTB(dp, MR_ADDR(port, TX_GMF_CTRL_T), GMF_CLI_TX_FU);
	}
}

/* This should never happen it is a fatal situation */
static void
myk_descriptor_error(struct gem_dev *dp,
    uint_t port, const char *rxtx, uint32_t mask)
{
	cmn_err(CE_WARN,
	    "!%s: %s: %s: descriptor error (hardware problem)",
	    dp->name, __func__, rxtx);

	OUTL(dp, B0_IMSK, INL(dp, B0_IMSK) & ~mask);
}

/*
 * Real hardware interrupt service routine
 */
#define	V_ISR_HWERR	0x0008
#define	V_ISR_LSC	0x0004
#define	V_ISR_TX	0x0002
#define	V_ISR_RX	0x0001
static uint_t
myk_interrupt(struct gem_dev *dp0)
{
	int		i;
	int		sync_rest;
	uint_t		ret = DDI_INTR_UNCLAIMED;
	uint32_t	status;
	uint_t		isr[2];
	int		slot;
	int		tail;
	struct myk_dev	*lp = dp0->private;
	struct myk_port	*pp;

	/* Reading this mask interrupts as side effect */
	status = INL(dp0, B0_Y2_SP_ISRC2);

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: status:%x",
	    dp0->name, ddi_get_lbolt(), __func__, status));

	if (status == 0) {
		/* not for us */
		goto done;
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: time:%d %s: isrc2:%b, isrc:%x, isrc3:%x",
	    dp0->name, ddi_get_lbolt(), __func__, status, IS_BITS,
	    INL(dp0, B0_ISRC), INL(dp0, B0_Y2_SP_ISRC3)));

	if ((status & (lp->descset[0].imsk | lp->descset[1].imsk)) == 0) {
		/* not for us */
		goto done;
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: time:%d %s: isrc2:%b, isrc:%x, isrc3:%x",
	    dp0->name, ddi_get_lbolt(), __func__, status, IS_BITS,
	    INL(dp0, B0_ISRC), INL(dp0, B0_Y2_SP_ISRC3)));

	status &= lp->descset[0].imsk | lp->descset[1].imsk;


	/* clear virtual interrupt source resgister */
	isr[0] = isr[1] = 0;

	if (status & Y2_IS_HW_ERR) {
		(void) myk_hw_intr(dp0);
		isr[0] |= V_ISR_HWERR;
		isr[1] |= V_ISR_HWERR;
	}
	if (status & Y2_IS_IRQ_PHY1) {
		pp = &lp->descset[0];
		if (pp->dp->misc_flag & GEM_SOFTINTR) {
			mutex_enter(&pp->dp->intrlock);
		}
		myk_phy_intr(pp->dp);
		if (pp->dp->misc_flag & GEM_SOFTINTR) {
			mutex_exit(&pp->dp->intrlock);
		}
		isr[0] |= V_ISR_LSC;
	}
	if (status & Y2_IS_IRQ_PHY2) {
		pp = &lp->descset[1];
		mutex_enter(&pp->dp->intrlock);
		myk_phy_intr(pp->dp);
		mutex_exit(&pp->dp->intrlock);
		isr[1] |= V_ISR_LSC;
	}
	if (status & Y2_IS_IRQ_MAC1) {
		pp = &lp->descset[0];
		myk_mac_intr(pp->dp, 0);
		isr[0] |= V_ISR_HWERR;
	}
	if (status & Y2_IS_IRQ_MAC2) {
		pp = &lp->descset[1];
		myk_mac_intr(pp->dp, 1);
		isr[1] |= V_ISR_HWERR;
	}
	if (status & Y2_IS_CHK_RX1) {
		pp = &lp->descset[0];
		myk_descriptor_error(pp->dp, 0, "receive", Y2_IS_CHK_RX1);
		isr[0] |= V_ISR_HWERR;
		cmn_err(CE_CONT, "!%s: rx_desc (%d, %d)",
		    pp->dp->name, pp->rx_desc_head, pp->rx_desc_tail);
	}
	if (status & Y2_IS_CHK_RX2) {
		pp = &lp->descset[1];
		myk_descriptor_error(pp->dp, 1, "receive", Y2_IS_CHK_RX2);
		isr[1] |= V_ISR_HWERR;
	}
	if (status & Y2_IS_CHK_TXA1) {
		pp = &lp->descset[0];
		myk_descriptor_error(pp->dp, 0, "transmit", Y2_IS_CHK_TXA1);
		isr[0] |= V_ISR_HWERR;
	}
	if (status & Y2_IS_CHK_TXA2) {
		pp = &lp->descset[1];
		myk_descriptor_error(pp->dp, 1, "transmit", Y2_IS_CHK_TXA2);
		isr[1] |= V_ISR_HWERR;
	}

#ifdef notdef
	if (dp0->poll_interval != lp->last_poll_interval) {
		myk_set_coalesce(dp0,
		    dp0->poll_pkt_delay, dp0->poll_interval/1000);
		lp->last_poll_interval = dp0->poll_interval;
	}
#endif

	/*
	 * Interpret reported status descriptors
	 */
	OUTL(dp0, STAT_CTRL, SC_STAT_CLR_IRQ);

	/* it also flush the previous pended write */
	tail = INW(dp0, STAT_PUT_IDX);

	sync_rest = 0;
	slot = lp->status_desc_head;
	while (slot != tail) {
		uint_t	control;
		uint_t	port;
		uint_t	length;
		uint_t	pos;
		uint_t	op;
		uint_t	css;
		struct msk_stat_desc	*sdp;

		if ((sync_rest--) == 0) {
			sync_rest = 10;
			myk_status_desc_dma_sync(dp0, slot, sync_rest,
			    DDI_DMA_SYNC_FORKERNEL);
		}

		/* get pointer to the current status descriptor */
		sdp = &((struct msk_stat_desc *)(void *)
		    STATUS_DESC_BASE(dp0))[slot];

#define	SDES_OP		0x7f000000U
#define		SDES_OP_SHIFT	24
#define	SDES_LENGTH	0x0000ffffU

		control = sdp->msk_control;
		control = LE_32(control);
		port = (control >> CSS_SHIFT) & CSS_LINK_BIT;
		op = (control & SDES_OP) >> SDES_OP_SHIFT;
		length = control & SDES_LENGTH;
		status = sdp->msk_status;
		status = LE_32(status);

		if ((control & HW_OWNER) == 0) {
			cmn_err(CE_WARN,
			    "!%s: %s status descriptor[%d] is invalid "
			    "(tail:%d)",
			    dp0->name, __func__, slot, tail);
#if DEBUG_LEVEL > 5
			for (i = slot; i != tail;
			    i = SLOT(i + 1, STATUS_DESC_SIZE)) {
				sdp = &((struct msk_stat_desc *)(void *)
				    STATUS_DESC_BASE(dp0))[i];
				cmn_err(CE_CONT, "!%d: %x",
				    i,  LE_32(sdp->msk_control),
				    LE_32(sdp->msk_status));
			}
#endif
			break;
		}

		/* give the descriptor to HW */
		sdp->msk_control = 0;
#ifdef notdef
		myk_status_desc_dma_sync(dp0, slot, 1, DDI_DMA_SYNC_FORDEV);
#endif
		/* interpret status descriptors */

		if (port >= lp->ports) {
			cmn_err(CE_WARN,
			    "!%s: %s: status entry hw error:"
			    "port %d (should be be 0 or 1), slot:%d",
			    dp0->name, __func__,
			    port, slot);
			goto next;
		}

		DPRINTF(3, (CE_CONT,
		    "!%s: %s: idx:%d op:0x%x len:0x%x, stat:0x%x",
		    dp0->name, __func__, slot, op, length, status));

		switch (op) {
		case OP_RXSTAT >> SDES_OP_SHIFT:
			pp = &lp->descset[port];

			DPRINTF(1, (CE_CONT,
			    "!%s: %s: idx:%d OP_RXSTAT(%x) "
			    "cntl:0x%x, stat:0x%x slot_tail:%d",
			    dp0->name, __func__, slot, op, control, status,
			    pp->rx_slot_tail));
#ifdef CONFIG_CKSUM_OFFLOAD
			if (lp->rx_full_cksum) {
				css = control >> CSS_SHIFT;
				if (css & CSS_TCPUDPCSOK) {
					if (css & CSS_ISTCP) {
						length |= GEM_RX_CKSUM_TCP;
					} else if (css & CSS_ISUDP) {
						length |= GEM_RX_CKSUM_UDP;
					}
				}
				if (css & CSS_IPV4CSUMOK) {
					length |= GEM_RX_CKSUM_IPv4;
				}
			}
#endif
			i = pp->rx_slot_tail;
			pp->rx_slot[i].rx_len = length;
			pp->rx_slot[i].rx_stat = status;
			pp->rx_slot_tail = SLOT(i + 1, RX_RING_SIZE);
			isr[port] |= V_ISR_RX;
			break;

		case OP_RXVLAN >> SDES_OP_SHIFT:
		case OP_RXCHKSVLAN >> SDES_OP_SHIFT:
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: idx:%d %s(%x)  tag:0x%x, cksum:0x%x",
			    dp0->name, __func__, slot,
			    (op == OP_RXVLAN) ? "OP_RXVLAN" : "OP_RXCHKSVLAN",
			    op, length, status & 0xffff));

			pp = &lp->descset[port];
			i = pp->rx_slot_tail;
			pp->rx_slot[i].rx_tag = (uint16_t)BSWAP_16(length);

#ifdef CONFIG_CKSUM_OFFLOAD	/* testing */
			if (op == OP_RXCHKSVLAN) {
				pp->rx_slot[i].rx_cksum =
				    (uint16_t)(status & 0xffffU);
			}
#endif
			break;

		case OP_RXCHKS >> SDES_OP_SHIFT:
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: idx:%d OP_RXCHKS(%x)  "
			    "control:0x%x stat:0x%x",
			    dp0->name, __func__,
			    slot, op, control, status & 0xffff));

			pp = &lp->descset[port];

#ifdef CONFIG_CKSUM_OFFLOAD	/* testing */
			pp->rx_slot[pp->rx_slot_tail].rx_cksum =
			    (uint16_t)(status & 0xffffU);
#endif
			break;

		case OP_TXINDEXLE >> SDES_OP_SHIFT:
			/* TX_INDEX_LE reports status for both ports */
			pp = &lp->descset[0];
			pos = status & 0xfff;
			if (pp->tx_head != pos) {
				pp->tx_head = pos;
				isr[0] |= V_ISR_TX;
			}

			if (lp->ports == 2) {
				/* for additional second port */
				pp = &lp->descset[1];
				pos = ((length << 8) | (status >> 24)) & 0xfff;
				if (pp->tx_head != pos) {
					pp->tx_head = pos;
					isr[1] |= V_ISR_TX;
				}
			}
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: idx:%d OP_TXINDEXLE(%x)  p0:%d p1:%d",
			    dp0->name, __func__, slot, op,
			    status & 0xfff,
			    ((length << 8) | (status >> 24)) & 0xfff));

			break;

		default:
			cmn_err(CE_WARN,
			    "!%s: %s: idx:%d unknown status opcode 0x%x, "
			    "slot:%d",
			    dp0->name, __func__, slot, op, slot);
			break;
		}
next:
		/* advance status head */
		slot = SLOT(slot + 1, STATUS_DESC_SIZE);
	}

	if (lp->status_desc_head != slot &&
	    lp->chip_id == CHIP_ID_YUKON_EC &&
	    lp->chip_rev == CHIP_REV_YU_EC_A1) {
		/* HWF_WA_DEV_43_418 */
		OUTB(dp0, STAT_TX_TIMER_CTRL, TIM_STOP);
		OUTB(dp0, STAT_TX_TIMER_CTRL, TIM_START);
	}

	/* save new head */
	lp->status_desc_head = slot;

	/* kick software interrupt handler for each port */
	for (i = 0; i < lp->ports; i++) {
		lp->descset[i].isr = isr[i];
		if (isr[i] && lp->descset[i].dp->soft_id) {
			ddi_trigger_softintr(lp->descset[i].dp->soft_id);
		}
	}
	ret = DDI_INTR_CLAIMED;

done:
	/* enable interrupts again */
	OUTL(dp0, B0_Y2_SP_ICR, 2);

	return (ret);
}

/*
 * soft interrupt service routine
 */
static uint_t
myk_soft_intr(struct gem_dev *dp)
{
	uint32_t	flags = 0;
	uint_t		isr;
	boolean_t	need_to_reset = B_FALSE;
	struct myk_dev	*lp = dp->private;
	struct myk_port	*pp = &lp->descset[PORT(dp)];

	DPRINTF(2, (CE_CONT, "!%s: time:%d %s: isr:%x",
	    dp->name, ddi_get_lbolt(), __func__, pp->isr));

	ASSERT(mutex_owned(&dp->intrlock));

	if ((dp->misc_flag & GEM_SOFTINTR) == 0) {
		if (myk_interrupt(dp) != DDI_INTR_CLAIMED) {
			return (DDI_INTR_UNCLAIMED);
		}
	}

	if ((isr = pp->isr) == 0) {
		return (DDI_INTR_UNCLAIMED);
	}

	pp->isr = 0;

	if (!dp->mac_active) {
		return (DDI_INTR_UNCLAIMED);
	}

	if (isr & V_ISR_RX) {
		(void) gem_receive(dp);
	}

	if (isr & V_ISR_TX) {
		if (gem_tx_done(dp)) {
			flags = INTR_RESTART_TX;
		}
	}

	if (isr & V_ISR_LSC) {
		if (gem_mii_link_check(dp)) {
			flags = INTR_RESTART_TX;
		}
	}

	if (isr & V_ISR_HWERR) {
		need_to_reset = B_TRUE;
	}

	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flags = INTR_RESTART_TX;
	}

	return (DDI_INTR_CLAIMED | flags);
}

/*
 * HW depend MII routine
 */
static void
myk_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
myk_mii_read(struct gem_dev *dp, uint_t reg)
{
	int		i;
	uint16_t	val;
	int		port = PORT(dp);

	ASSERT(port == 0);

	OUTW(dp, GMAC_REG(port, GM_SMI_CTRL),
	    GM_SMI_CT_PHY_AD(dp->mii_phy_addr) |
	    GM_SMI_CT_REG_AD(reg) | GM_SMI_CT_OP_RD);
#if DEBUG_LEVEL > 5
	val = INW(dp, GMAC_REG(port, GM_SMI_CTRL));
	if ((val & GM_SMI_CT_OP_RD) == 0) {
		cmn_err(CE_WARN,
		    "!%s: %s: PHY read impossible on Port %d (Ctrl=0x%04x)",
		    dp->name, __func__, port, val);
		return (0);
	}
#endif
	i = 0;
	while ((INW(dp, GMAC_REG(port, GM_SMI_CTRL)) & GM_SMI_CT_RD_VAL) == 0) {
		if (i++ > 10000) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return (0);
		}
		drv_usecwait(10);
	}

	val = INW(dp, GMAC_REG(port, GM_SMI_DATA));
	DPRINTF(5, (CE_CONT, "!%s: %s: reg:0x%x val:0x%x",
	    dp->name, __func__, reg, val));

	return (val);
}

static void
myk_mii_write_raw(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;
	uint_t		port = PORT(dp);

	DPRINTF(5, (CE_CONT, "!%s: %s: called, reg:0x%x, val:0x%x",
	    dp->name, __func__, reg, val));

	OUTW(dp, GMAC_REG(port, GM_SMI_DATA), val);
	OUTW(dp, GMAC_REG(port, GM_SMI_CTRL),
	    GM_SMI_CT_PHY_AD(dp->mii_phy_addr) | GM_SMI_CT_REG_AD(reg));

	i = 0;
	while (INW(dp, GMAC_REG(port, GM_SMI_CTRL)) & GM_SMI_CT_BUSY) {
		if (i++ > 1000) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(10);
	}
}
static void
myk_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	struct myk_dev	*lp = dp->private;

	if (reg == MII_CONTROL) {
		if (val & MII_CONTROL_RESET) {
			/* myk_mii_config() will reset phy */
			return;
		}
		if (val & MII_CONTROL_ANE) {
			/*
			 * Need to restart autonegotiation.
			 * XXX - current version of gem issue ANE
			 * with RSAN always.
			 */
			val |= MII_CONTROL_RSAN;
#ifdef NEVER
			/*
			 * FE_P fails in autonegotiation for giga PHYs when
			 * autonegotiation starts with RESET bit.
			 * But other chipset seem to prefer with  RESET bit.
			 */
			if (lp->chip_id != CHIP_ID_YUKON_FE_P) {
				val |= MII_CONTROL_RESET;
			}
#endif
		} else {
			/* forced mode requires reset */
			val |= MII_CONTROL_RESET;
		}
	}
	myk_mii_write_raw(dp, reg, val);
}

/*
 * myk_mii_config: phy initialization after phy reset
 */
static int
myk_mii_config(struct gem_dev *dp)	/* ok for optima */
{
	uint16_t	pg;
	uint16_t	ctrl;
	uint16_t	physpec;
	uint16_t	ledctrl;
	uint_t		blinkctrl;
	uint16_t	ledover;
	uint_t		port = PORT(dp);
	struct myk_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (dp->anadv_autoneg &&
	    !HW_HAS_NEWER_PHY(lp->chip_id) &&
	    (dp->mii_status & MII_STATUS_XSTATUS) != 0) {	/* ok */
		uint16_t ectrl = myk_mii_read(dp, PHY_MARV_EXT_CTRL);

		ectrl &= ~(PHY_M_EC_M_DSC_MSK | PHY_M_EC_S_DSC_MSK |
		    PHY_M_EC_MAC_S_MSK);
		ectrl |= PHY_M_EC_MAC_S(MAC_TX_CLK_25_MHZ);

		if (lp->chip_id == CHIP_ID_YUKON_EC) {
			ectrl |= PHY_M_EC_DSC_2(2) | PHY_M_EC_DOWN_S_ENA;
		} else {
			ectrl |= PHY_M_EC_M_DSC(0) | PHY_M_EC_S_DSC(1);
		}
		myk_mii_write_raw(dp, PHY_MARV_EXT_CTRL, ectrl);
	}

	physpec = myk_mii_read(dp, PHY_MARV_PHY_CTRL);	/* ok */
	if (myk_is_copper(dp)) {		/* ok */
		if (lp->chip_id == CHIP_ID_YUKON_FE ||
		    lp->chip_id == CHIP_ID_YUKON_FE_P) {
			/* enable automatic crossover */
			physpec |= PHY_M_PC_MDI_XMODE(PHY_M_PC_ENA_AUTO) >> 1;

			if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
			    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
				uint16_t tmp;

				/* Enable Class A driver for FE+ A0 */
				tmp = myk_mii_read(dp, PHY_MARV_FE_SPEC_2);
				tmp |= PHY_M_FESC_SEL_CL_A;
				myk_mii_write_raw(dp, PHY_MARV_FE_SPEC_2, tmp);
			}

		} else {
#ifdef NEVER
			if (HW_FEATURE(dp, HWF_PHY_CLASS_A_100BT)) {
				/* Enable Class A driver for 100Base-T */
				tmp = myk_mii_read(dp, PHY_MARV_EXT_CTRL_2);
				tmp |= BIT_12S;
				myk_mii_write_raw(dp, PHY_MARV_EXT_CTRL_2, tmp);
			}

			if (HW_FEATURE(dp, HWF_TRAFO_LESS_ENABLE)) {
				/* connect internal MDI termination to AVDD */
				myk_mii_write_raw(dp, PHY_MARV_PAGE_ADDR, 3);

				myk_mii_write_raw(dp,
				    PHY_MARV_PAGE_DATA, 0x0002);

				/* apply fixes in PHY AFE */
				myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 0x00ff);

				/* enable half amplitude mode */
				myk_mii_write_raw(dp, 24, 0xa104);
				myk_mii_write_raw(dp, 23, 0x2002);

				myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 0);
			}
#endif
			/* disable Energy Detect Mode */
			physpec &= ~PHY_M_PC_EN_DET_MSK;

			/* enable automatic crossover */
			physpec |= PHY_M_PC_MDI_XMODE(PHY_M_PC_ENA_AUTO);

			/* downshift on PHY 88E1112 and 88E1149 is changed */
			if (dp->anadv_autoneg &&
			    HW_HAS_NEWER_PHY(lp->chip_id)) {
				/*
				 * set downshift counter to 3x and enable
				 * downshift
				 */
				physpec &= ~PHY_M_PC_DSC_MSK;
				physpec |= PHY_M_PC_DSC(2) |
				    PHY_M_PC_DOWN_S_ENA;
			}
		}
	} else { /* ok */
		/* workaround for deviation #4.88 (CRC errors) */
		/* disable Automatic Crossover */
		physpec &= ~PHY_M_PC_MDIX_MSK;
	} /* ok */
	myk_mii_write_raw(dp, PHY_MARV_PHY_CTRL, physpec);

	/* special setup for PHY 88E1112 Fiber */
	if (lp->chip_id == CHIP_ID_YUKON_XL && !myk_is_copper(dp)) {
		/* Fiber: select 1000BASE-X only mode MAC Specific Ctrl Reg. */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 2);
		ctrl = myk_mii_read(dp, PHY_MARV_PHY_CTRL);
		ctrl &= ~PHY_M_MAC_MD_MSK;
		ctrl |= PHY_M_MAC_MODE_SEL(PHY_M_MAC_MD_1000BX);
		myk_mii_write_raw(dp, PHY_MARV_PHY_CTRL, ctrl);

		/* select page 1 to access Fiber registers */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 1);
		if (lp->pmd_type == 'P') {
			/* for SFP-module set SIGDET polarity to low */
			ctrl = myk_mii_read(dp, PHY_MARV_PHY_CTRL);
			ctrl |= PHY_M_FIB_SIGD_POL;
			myk_mii_write_raw(dp, PHY_MARV_PHY_CTRL, ctrl);
		}
	}

	/* issue software reset */
	ctrl = myk_mii_read(dp, MII_CONTROL);

	/* this doesn't make sense because ctrl is 0 after myk_reset_port() */
	if (!dp->anadv_autoneg) {
		ctrl &= ~MII_CONTROL_ANE;
	}
	myk_mii_write_raw(dp, MII_CONTROL, ctrl | MII_CONTROL_RESET);

	/* use generic routine to setup mode and duplex */
	if (gem_mii_config_default(dp) != GEM_SUCCESS) {
		return (GEM_FAILURE);
	}

	/*
	 * additional setup after PHY reset and writing
	 * configuration to mii mode control register
	 */

	/* Setup Phy LED's */
	ledctrl = PHY_M_LED_PULS_DUR(PULS_170MS);
	ledover = 0;
	blinkctrl = lp->descset[port].blink_ctrl;

	if (lp->chip_id == CHIP_ID_YUKON_FE) {
		/* on 88E3082 these bits are at 11..9 (shifted left) */
		ledctrl |= PHY_M_LED_BLINK_RT(BLINK_84MS) << 1;

		ctrl = PHY_M_FELP_LED2_CTRL(LED_PAR_CTRL_LINK) |
		    /* change ACT LED control to LINK/ACT or blink mode */
		    PHY_M_FELP_LED1_CTRL(
		    (blinkctrl == CFG_LED_COMB_ACT_LNK) ?
		    LED_PAR_CTRL_LNK_AC : LED_PAR_CTRL_ACT_BL) |
		    PHY_M_FELP_LED0_CTRL(
		    /* check for LINK_LED mux */
		    (blinkctrl == CFG_LED_LINK_MUX_P60) ?
		    LED_PAR_CTRL_LINK : LED_PAR_CTRL_SPEED);

		myk_mii_write_raw(dp, PHY_MARV_FE_LED_PAR, ctrl);

	} else if (lp->chip_id == CHIP_ID_YUKON_FE_P) {
		/* Enable Link Partner Next Page */
		physpec |= PHY_M_PC_ENA_LIP_NP;

		/* disable Energy Detect and enable scrambler */
		physpec &= ~(PHY_M_PC_ENA_ENE_DT | PHY_M_PC_DIS_SCRAMB);

		myk_mii_write_raw(dp, PHY_MARV_PHY_CTRL, physpec);

		/* set LED2 -> ACT, LED1 -> LINK, LED0 -> SPEED */
		ctrl = PHY_M_FELP_LED2_CTRL(
		    (blinkctrl == CFG_LED_COMB_ACT_LNK) ?
		    LED_PAR_CTRL_LNK_AC : LED_PAR_CTRL_ACT_BL) |
		    PHY_M_FELP_LED1_CTRL(LED_PAR_CTRL_LINK) |
		    PHY_M_FELP_LED0_CTRL(
		    /* check for LINK_LED mux */
		    (blinkctrl == CFG_LED_LINK_MUX_P60) ?
		    LED_PAR_CTRL_LINK : LED_PAR_CTRL_SPEED);

		myk_mii_write_raw(dp, PHY_MARV_FE_LED_PAR, ctrl);

	} else if (HW_HAS_NEWER_PHY(lp->chip_id)) {
		uint_t	ledconf;
		uint_t	mode;

		/* save page register */
		pg = myk_mii_read(dp, PHY_MARV_EXT_ADR);

		/* select page 3 to access LED control register */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 3);

		/* set LED Function Control register */
		ledconf = PHY_M_LEDC_LOS_CTRL(LC_LNK_ON_ACT_BL) |
		    PHY_M_LEDC_STA1_CTRL(LC_LINK_ON) |	/* 100 Mbps */
		    PHY_M_LEDC_STA0_CTRL(LC_LINK_ON);	/* 1000 Mbps */

		mode = LC_LINK_ON;	/* 10 Mbps: on */

		if (lp->chip_id == CHIP_ID_YUKON_XL) {
			/* set Polarity Control register */
			myk_mii_write_raw(dp, PHY_MARV_PHY_STAT,
			    PHY_M_POLC_LS1_P_MIX(4)
			    | PHY_M_POLC_IS0_P_MIX(4)
			    | PHY_M_POLC_LOS_CTRL(2)
			    | PHY_M_POLC_INIT_CTRL(2)
			    | PHY_M_POLC_STA1_CTRL(2)
			    | PHY_M_POLC_STA0_CTRL(2));
		} else if (lp->chip_id == CHIP_ID_YUKON_EC_U ||
		    lp->chip_id == CHIP_ID_YUKON_EX ||
		    lp->chip_id >= CHIP_ID_YUKON_SUPR) {

			/* check for LINK_LED mux */
			if (blinkctrl == CFG_LED_LINK_MUX_P60) {
				/* LED scheme 2 */
				/* set GPHY LED Config */
				OUTW(dp, GPHY_CTRL,
				    INW(dp, GPHY_CTRL) | GPC_LED_CONF_VAL(4));
			} else {
				/* check for LED config bit 8 in PCI Our4 */

				if ((INW(dp, Y2_CFG_SPC + PCI_OUR_REG_4)
				    & P_PIN63_LINK_LED_ENA) == 0) {
					/* LED scheme 1 */
					/* 10 Mbps: forced Off */
					mode = LC_FORCE_OFF;
					if ((blinkctrl !=
					    CFG_LED_ACT_OFF_NOTR)) {
						/*
						 * set LED[5:4] Function
						 * Control and Polarity
						 */
						myk_mii_write_raw(dp,
						    PHY_MARV_INT_STAT,
						    /* LED_ACT to Link/Act. */
						    (PHY_M_LEDC_STA1_CTRL(
						    LC_LNK_ON_ACT_BL) |
						    /* LED_DUP to Duplex */
						    PHY_M_LEDC_STA0_CTRL(
						    LC_DUPLEX_ON)));
					}
				}
			}
			/* set Blink Rate in LED Timer Control Register */
			myk_mii_write_raw(dp, PHY_MARV_INT_MASK,
			    ledctrl | PHY_M_LED_BLINK_RT(BLINK_84MS));
		}

		ledconf |= PHY_M_LEDC_INIT_CTRL(mode);

		/* set LED Function Control register */
		myk_mii_write_raw(dp, PHY_MARV_PHY_CTRL, ledconf);

		/* restore page register */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, pg);

	} else {
		/* set Tx LED (LED_TX) to blink mode on Rx OR Tx activity */
		ledctrl |= PHY_M_LED_BLINK_RT(BLINK_84MS) | PHY_M_LEDC_TX_CTRL;

		/* on PHY 88E1111 there is a change for LED control */
		if (lp->chip_id == CHIP_ID_YUKON_EC &&
		    blinkctrl == CFG_LED_DUAL_ACT_LNK) {
			/* Yukon-EC needs setting of 2 bits: 0,6=11) */
			ledctrl |= PHY_M_LEDC_TX_C_LSB;
		}
		/* turn off the Rx LED (LED_RX) */
		ledover |= PHY_M_LED_MO_RX(MO_LED_OFF);
	}
#ifdef notyet
	if ((blinkctrl & 0x2) != 0) {
		/* disable blink mode (LED_DUPLEX) on collisions */
		ledctrl |= PHY_M_LEDC_DP_CTRL;
	}
#endif
	if (lp->chip_id == CHIP_ID_YUKON_EC_U ||
	    lp->chip_id == CHIP_ID_YUKON_UL_2) {
		/* apply fixes in PHY AFE */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 255);

		/* increase differential signal amplitude in 10BASE-T */
		myk_mii_write_raw(dp, 0x18, 0xaa99);
		myk_mii_write_raw(dp, 0x17, 0x2011);

		if (lp->chip_id == CHIP_ID_YUKON_EC_U) {
			/* fix for IEEE A/B Symmetry failure in 1000BASE-T */
			myk_mii_write_raw(dp, 0x18, 0xa204);
			myk_mii_write_raw(dp, 0x17, 0x2002);
		}

		/* set page register to 0 */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 0);
	} else if (lp->chip_id == CHIP_ID_YUKON_FE_P &&
	    lp->chip_rev == CHIP_REV_YU_FE2_A0) {
		/* apply workaround for integrated registors calibration */
		myk_mii_write_raw(dp, PHY_MARV_PAGE_ADDR, 17);
		myk_mii_write_raw(dp, PHY_MARV_PAGE_DATA, 0x3f60);
#ifdef CONFIG_OPTIMA
	} else if (lp->chip_id == CHIP_ID_YUKON_OPT && lp->chip_rev == 0) {
		/* apply fixes in PHY AFE */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 0x00ff);

		/* apply RDAC termination workaround */
		myk_mii_write_raw(dp, 24, 0x2800);
		myk_mii_write_raw(dp, 23, 0x2001);

		/* set page register back to 0 */
		myk_mii_write_raw(dp, PHY_MARV_EXT_ADR, 0);
#endif
	} else if (lp->chip_id != CHIP_ID_YUKON_EX &&
	    lp->chip_id < CHIP_ID_YUKON_SUPR) {
		/* no effect on Yukon-XL */
		myk_mii_write_raw(dp, PHY_MARV_LED_CTRL, ledctrl);

		if (!dp->anadv_autoneg || dp->speed == GEM_SPD_100) {
			/* turn on 100 Mbps LED (LED_LINK100) */
			ledover |= PHY_M_LED_MO_100(MO_LED_ON);
		}
		if (ledover) {
			myk_mii_write_raw(dp, PHY_MARV_LED_OVER, ledover);
		}
	}

	/* clear PHY IRQ status */
	myk_mii_read(dp, PHY_MARV_INT_STAT);

	/* enable PHY interrupt on auto-negotiation complete and link change */
	ctrl = PHY_M_IS_AN_ERROR | PHY_M_IS_LST_CHANGE
	    | PHY_M_IS_FIFO_ERROR | PHY_M_IS_END_CHANGE;
	if (dp->anadv_autoneg) {
		ctrl |= PHY_M_IS_AN_COMPL;
	}
	myk_mii_write_raw(dp, PHY_MARV_INT_MASK, ctrl);

	return (GEM_SUCCESS);
}

/*
 * myk_mii_init: phy initialization after power up
 */
static int
myk_mii_init(struct gem_dev *dp)
{
	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
mykattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	int			t8;
	ddi_acc_handle_t	conf_ha;
	int			vid;
	int			did;
	uint8_t			revid;
	uint8_t			lat;
	uint8_t			chip_id;
	uint8_t			chip_rev;
	uint8_t			chip_cap;
	uint8_t			pm_cap;
	uint16_t		pmcsr = 0;
	uint_t			pcie_cap;
	uint8_t			pmd_type;
	int			ports;
	int			unit;
	const char		*drv_name;
	struct gem_dev		*dp[2];
	struct myk_dev		*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(0, (CE_CONT, "!%s%d: %s: called %s",
	    drv_name, unit, __func__, drv_name));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_ha) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s: pci_config_setup failed",
		    drv_name);
		goto err;
	}

	vid = pci_config_get16(conf_ha, PCI_CONF_VENID);
	did = pci_config_get16(conf_ha, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_ha, PCI_CONF_REVID);
	lat = pci_config_get8(conf_ha, PCI_CONF_LATENCY_TIMER);

	/* enable clock unconditionally */
	pci_config_put32(conf_ha, PCI_OUR_REG_3, 0);

	/* ensure the pm status is D0 mode */
	pm_cap = gem_search_pci_cap(dip, conf_ha, PCI_CAP_ID_PM);
	if (pm_cap) {
		pmcsr = pci_config_get16(conf_ha, pm_cap + PCI_PMCSR);
	}
	(void) gem_pci_set_power_state(dip, conf_ha, PCI_PMCSR_D0);

	pci_config_put16(conf_ha, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_ha, PCI_CONF_COMM));

	cmn_err(CE_CONT,
	    "!%s%d: (vid: 0x%04x, did: 0x%04x, revid: 0x%02x,"
	    " latency timer: 0x%02x)",
	    drv_name, unit, vid, did, revid, lat);

	if (pci_config_get32(conf_ha, PCI_OUR_REG_2) == 0xffffffffU) {
		cmn_err(CE_WARN, "!%s: still in powerdown mode", drv_name);
		goto err;
	}

	switch (cmd) {
	case DDI_RESUME:
		pci_config_teardown(&conf_ha);

		/* prepare for full reset including status unit */
		dp[0] = ddi_get_driver_private(dip);
		lp = dp[0]->private;
		lp->initialized = -1;

		return (gem_resume(dip));

	case DDI_ATTACH:
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_MEM64, PCI_ADDR_MASK,
		    &myk_dev_attr, (caddr_t *)&base,
		    &regs_ha) != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * release global reset, otherwise chip id is read
		 * as 0xff for EC_U.
		 */
		ddi_put8(regs_ha, (uint8_t *)(base + B0_CTST), CS_RST_CLR);

		chip_id = ddi_get8(regs_ha, (uint8_t *)(base + B2_CHIP_ID));
		chip_rev = ddi_get8(regs_ha, (uint8_t *)(base + B2_MAC_CFG));
		chip_cap = chip_rev & 0xf;
		chip_rev = (chip_rev & CFG_CHIP_R_MSK) >> 4;

		switch (chip_id) {
		case CHIP_ID_YUKON_XL:
		case CHIP_ID_YUKON_EC_U:
		case CHIP_ID_YUKON_EX:
		case CHIP_ID_YUKON_EC:
		case CHIP_ID_YUKON_FE:
		case CHIP_ID_YUKON_FE_P:
		case CHIP_ID_YUKON_SUPR:
		case CHIP_ID_YUKON_UL_2:
#ifdef CONFIG_OPTIMA
		case CHIP_ID_YUKON_OPT:
#endif
			/* supported chipset */
			break;

		default:
			cmn_err(CE_WARN,
			    "!%s%d: unsupported chip type 0x%x",
			    drv_name, unit, chip_id);
			ddi_regs_map_free(&regs_ha);
			goto err;
		}

		/* This rev is really old, and requires untested workarounds */
		if (chip_id == CHIP_ID_YUKON_EC &&
		    chip_rev == CHIP_REV_YU_EC_A1) {
			cmn_err(CE_WARN,
			    "!%s%d: unsupported revision"
			    " Yukon-%s (0x%x) rev %d cap x%f",
			    drv_name, unit,
			    yukon2_name[chip_id - CHIP_ID_YUKON_XL],
			    chip_id, chip_rev, chip_cap);
			ddi_regs_map_free(&regs_ha);
			goto err;
		}

		/* check the number of ports for the nic */
		pmd_type = ddi_get8(regs_ha, (void *)(base + B2_PMD_TYP));
		ports = 1;
		t8 = ddi_get8(regs_ha, (void *)(base + B2_Y2_HW_RES));
		if ((t8 & CFG_DUAL_MAC_MSK) == CFG_DUAL_MAC_MSK) {
			t8 = ddi_get8(regs_ha,
			    (void *)(base + B2_Y2_CLK_GATE));
			if ((t8 & Y2_STATUS_LNK2_INAC) == 0) {
				/* the nic has an additional port */
				ports++;
			}
		}

		if (chip_id >= CHIP_ID_YUKON_XL &&
		    chip_id <= CHIP_ID_YUKON_SUPR) {
			if (chip_id == CHIP_ID_YUKON_EX ||
			    chip_id == CHIP_ID_YUKON_SUPR) {
#ifdef notyet
				uint_t	tmp;

				/* stop the watchdog */
				ddi_put32(regs_ha,
				    (void *)(base + CPU_WDOG), 0);
				/* Do not touch bit 5..3 */
				tmp = ddi_get16(regs_ha,
				    (void *)(base + B28_Y2_ASF_STAT_CMD));
				tmp &= ~(HCU_CCSR_AHB_RST
				    | HCU_CCSR_CPU_RST_MODE
				    | HCU_CCSR_UC_STATE_MSK);
				/*
				 * CPU clock divider shouldn't be used because
				 * - ASF firmware may malfunction
				 * - Yukon-Supreme: Parallel FLASH doesn't
				 *   support divided clocks
				 */
				if ((tmp & HCU_CCSR_CPU_CLK_DIVIDE_MSK) != 0) {
					DPRINTF(0, (CE_CONT,
					    "CPU Clock Divider bits are set"
					    " (0x%x), cleared now", tmp));
					tmp &= ~HCU_CCSR_CPU_CLK_DIVIDE_MSK;
				}

				ddi_put16(regs_ha,
				    (void *)(base + B28_Y2_ASF_STAT_CMD), tmp);
				/* stop the watchdog */
				ddi_put32(regs_ha,
				    (void *)(base + CPU_WDOG), 0);
#endif
			} else {
				ddi_put8(regs_ha,
				    (void *)(base + B28_Y2_ASF_STAT_CMD),
				    Y2_ASF_RESET);
			}
			/* disable ASF */
			ddi_put16(regs_ha,
			    (void *)(base + B0_CTST), Y2_ASF_DISABLE);
		}

		pcie_cap = gem_search_pci_cap(dip, conf_ha, PCI_CAP_ID_PCI_E);

		cmn_err(CE_NOTE,
		    "!%s%d: Yukon2-%s (0x%x) "
		    "rev %d, cap x%x, pcie:0x%x pm_cap:0x%x 0x%x, nport:%d",
		    drv_name, unit,
		    yukon2_name[chip_id - CHIP_ID_YUKON_XL], chip_id,
		    chip_rev, chip_cap, pcie_cap, pm_cap, pmcsr, ports);

		if (pcie_cap) {
			uint32_t	devcap;
			uint32_t	devcsr;

			devcap = pci_config_get32(conf_ha, pcie_cap + 4);
			devcsr = pci_config_get32(conf_ha, pcie_cap + 8);

			DPRINTF(0, (CE_CONT,
			    "!%s: pcie cap:%x, dev_cap:%x, dev_csr:%x",
			    __func__,
			    pci_config_get32(conf_ha, pcie_cap),
			    devcap, devcsr));
			/*
			 * increase max read request 2:512, 3:1024, 4:2048
			 */
#if 0
#if 1
			if ((devcsr & (7 << 12)) == (2 << 12)) {
				devcsr = (devcsr & ~(7 << 12)) | (4 << 12);
				pci_config_put32(conf_ha,
				    pcie_cap + 8, devcsr);
			}
#else
			devcsr = (devcsr & ~(7 << 12)) | (3 << 12);
			pci_config_put32(conf_ha, pcie_cap + 8, devcsr);
#endif
#endif
		}
		pci_config_teardown(&conf_ha);
		conf_ha = NULL;

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		(void) sprintf(gcp->gc_name, "%s%%d", drv_name);

		/* configuration of tx and rx rings and buffers */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = GEM_MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags*2 + 1;
		gcp->gc_tx_desc_unit_shift = 3;		/* 8 byte */
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = myk_tx_copy_thresh;
		gcp->gc_tx_desc_write_oo = B_FALSE;

		gcp->gc_rx_buf_align = sizeof (uint64_t) - 1;
		gcp->gc_rx_max_frags = 1;
		/* XXX - we allocate rx descriptors in io_area by ourselves */
		gcp->gc_rx_desc_unit_shift = -1;
		/* XXX - RX_RING_SIZE isn't real descriptor size */
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_copy_thresh = myk_rx_copy_thresh;

		gcp->gc_io_area_size = RX_DESC_BYTES + STATUS_LE_BYTES;
		gcp->gc_hck_rx_start = sizeof (struct ether_header);

		/* map attributes */
		gcp->gc_dev_attr = myk_dev_attr;
		gcp->gc_buf_attr = myk_buf_attr;
		gcp->gc_desc_attr = myk_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = myk_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = myk_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen =
		    gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = myk_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen =
		    gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;
		gcp->gc_mii_hw_link_detection = B_TRUE;
		gcp->gc_mii_stop_mac_on_linkdown = B_TRUE;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		/*
		 * Reset phy after link down, otherwise flow control
		 * capabilities may not be advertised corretly.
		 */
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_TRUE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &myk_attach_port;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &myk_fixup_params;
#endif
		gcp->gc_reset_chip = &myk_reset_port;
		gcp->gc_init_chip = &myk_init_port;
		gcp->gc_start_chip = &myk_start_port;
		gcp->gc_stop_chip = &myk_stop_port;
		gcp->gc_multicast_hash = &myk_mcast_hash;
		gcp->gc_set_rx_filter = &myk_set_rx_filter;
		gcp->gc_set_media = &myk_set_media;
		gcp->gc_get_stats = &myk_get_stats;
		gcp->gc_interrupt = &myk_soft_intr;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &myk_tx_desc_write;
		gcp->gc_rx_desc_write = &myk_rx_desc_write;
		gcp->gc_tx_start = &myk_tx_start;
		gcp->gc_rx_start = &myk_rx_start;

		gcp->gc_tx_desc_init = &myk_tx_desc_init;
		gcp->gc_rx_desc_init = &myk_rx_desc_init;
#ifdef GEM_CONFIG_TX_HEAD_PTR
		gcp->gc_tx_desc_stat = NULL;
		gcp->gc_tx_desc_head = &myk_tx_desc_head;
#else
		gcp->gc_tx_desc_stat = &myk_tx_desc_stat;
#endif
		gcp->gc_rx_desc_stat = &myk_rx_desc_stat;
		gcp->gc_tx_desc_clean = &myk_tx_desc_init;
		gcp->gc_rx_desc_clean = &myk_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = &myk_mii_init;
		gcp->gc_mii_config = &myk_mii_config;
		gcp->gc_mii_sync = &myk_mii_sync;
		gcp->gc_mii_read = &myk_mii_read;
		gcp->gc_mii_write = &myk_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* number of max ports in specification */
		gcp->gc_nports = 2;

		/* MSI/MSIX interrupts */
		gcp->gc_nintrs_req = 1;

		gcp->gc_max_lso = 64 * 1024 - 1;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_max_mtu = MSK_JUMBO_MTU;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN;
#endif
		/* SkGeInit0() */
		lp = kmem_zalloc(sizeof (struct myk_dev), KM_SLEEP);
		lp->chip_id = chip_id;
		lp->chip_rev = chip_rev;
		lp->chip_cap = chip_cap;
		lp->is_pcie = pcie_cap != 0;
		lp->pm_cap = pm_cap;
		lp->pmd_type = pmd_type;
		lp->ports = (uint8_t)ports;

		if ((lp->chip_id == CHIP_ID_YUKON_EX &&
		    lp->chip_rev != CHIP_REV_YU_EX_B0) ||
		    lp->chip_id == CHIP_ID_YUKON_SUPR ||
#ifdef CONFIG_OPTIMA
		    lp->chip_id == CHIP_ID_YUKON_OPT ||
#endif
		    lp->chip_id == CHIP_ID_YUKON_FE_P) {
			lp->tx_full_cksum = B_TRUE;
		}

		if (lp->chip_id == CHIP_ID_YUKON_EX ||
		    lp->chip_id == CHIP_ID_YUKON_SUPR ||
		    lp->chip_id == CHIP_ID_YUKON_FE_P) {
			lp->rx_full_cksum = B_TRUE;
			gcp->gc_hck_rx_start = 0;
		}

		/* initialize head of port list */
		ddi_set_driver_private(dip, NULL);
		dp[0] = dp[1] = NULL;

		for (i = 0; i < ports; i++) {
			dp[i] = gem_do_attach(dip, i,
			    gcp, base, &regs_ha, lp, sizeof (*lp));

			if (dp[i] == NULL) {
				goto err_free_gc;
			}
		}

		kmem_free(gcp, sizeof (*gcp));

		if (dp[0]->misc_flag & GEM_SOFTINTR) {
			/* add real hw interrupt handler */
			if (ddi_add_intr(dip, 0, &lp->iblock_cookie, NULL,
			    (uint_t (*)(caddr_t))myk_interrupt,
			    (caddr_t)dp[0]) != DDI_SUCCESS) {
				cmn_err(CE_WARN, "!%s%d: ddi_add_intr failed",
				    drv_name, unit);
				goto err_free_mem;
			}
		}

		myk_status_desc_init(lp);
#if 0
		/* low latency configuration */
		OUTB(dp[0], STAT_FIFO_WM, 1);
		OUTB(dp[0], STAT_FIFO_ISR_WM, 1);
		OUTL(dp[0], STAT_LEV_TIMER_INI, 50);
		OUTL(dp[0], STAT_ISR_TIMER_INI, 10);
		OUTL(dp[0], STAT_TX_TIMER_INI, HW_MS_TO_TICKS(dp[0], 10));
#endif
#if 0
		OUTW(dp[0], STAT_TX_IDX_TH, 10);
		OUTB(dp[0], STAT_FIFO_WM, 0x10);
		OUTB(dp[0], STAT_FIFO_ISR_WM, 0x10);
		OUTL(dp[0], STAT_LEV_TIMER_INI, myk_us2clk(dp[0], 50));
		OUTL(dp[0], STAT_ISR_TIMER_INI, 10);
		OUTL(dp[0], STAT_TX_TIMER_INI, myk_us2clk(dp[0], 50));
#endif

		DPRINTF(-1, (CE_CONT,
		    "!%s: status desc conf: "
		    "fifo_wm:%d, fifo_isr_wm:%d, "
		    "lev_timer:%dus, isr_timer:%dus, "
		    "tx_timer:%dus",
		    dp[0]->name,
		    INB(dp[0], STAT_FIFO_WM),
		    INB(dp[0], STAT_FIFO_ISR_WM),
		    INL(dp[0], STAT_LEV_TIMER_INI)/myk_us2clk(dp[0], 1),
		    INL(dp[0], STAT_ISR_TIMER_INI)/myk_us2clk(dp[0], 1),
		    INL(dp[0], STAT_TX_TIMER_INI)/myk_us2clk(dp[0], 1)));

		return (DDI_SUCCESS);

err_free_gc:
		kmem_free(gcp, sizeof (*gcp));
err_free_mem:
		kmem_free(lp, sizeof (struct myk_dev));
err:
		if (conf_ha != NULL) {
			pci_config_teardown(&conf_ha);
		}

		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
mykdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct gem_dev  *dp;
	struct myk_dev	*lp;

	dp = ddi_get_driver_private(dip);
	lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* turn off LED's */
	OUTW(dp, B0_CTST, Y2_LED_STAT_OFF);

	/* stop the hardware completely */
#ifdef notdef
	/* XXX - this disables FP_P chipset forever */
	myk_set_phy_power_state(dp, PCI_PMCSR_D3HOT);
#endif
	OUTW(dp, B0_CTST, Y2_LED_STAT_OFF);
	OUTB(dp, B0_CTST, CS_RST_SET);
	FLSHB(dp, B0_CTST);

	switch (cmd) {
	case DDI_SUSPEND:
		return (gem_suspend(dip));

	case DDI_DETACH:
		if (dp->misc_flag & GEM_SOFTINTR) {
			ddi_remove_intr(dip, 0, lp->iblock_cookie);
		}
		(void) gem_do_detach(dip);
		return (DDI_SUCCESS);
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef GEM_CONFIG_GLDv3
GEM_STREAM_OPS(myk_ops, mykattach, mykdetach);
#else
static	struct module_info mykminfo = {
	0,			/* mi_idnum */
	"myk",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit mykrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&mykminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit mykwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&mykminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};
static struct streamtab	myk_info = {
	&mykrinit,	/* st_rdinit */
	&mykwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_myk_ops = {
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
	&myk_info,	/* cb_str */
	D_MP,		/* cb_flag */
#ifdef notdef
	CB_REV,		/* cb_rev */
	nodev,		/* cb_aread */
	nodev,		/* cb_awrite */
#endif
};

static	struct dev_ops myk_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	mykattach,	/* devo_attach */
	mykdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_myk_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce	/* devo_quiesce */
#endif
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&myk_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!myk: _init: called"));
	gem_mod_init(&myk_ops, "myk");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&myk_ops);
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

	DPRINTF(2, (CE_CONT, "!myk: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&myk_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
