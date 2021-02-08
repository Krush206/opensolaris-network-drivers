/*
 * ae_gem.c : The AMD PCNET/PCI/FAST  Ethernet Driver for Solaris
 *
 * Copyright (c) 2006-2011 Masayuki Murayama.  All rights reserved.
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

#pragma	ident	"@(#)ae_gem.c	1.13 11/09/19"

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

#include "gem.h"
#include "gem_mii.h"

#include "pcnet.h"

char	ident[] = "pcnet driver v" VERSION;

/*
 * Useful macros
 */
#define	ROUNDUP2(x, y)	(((x)+(y)-1) & ~((y)-1))
#define	ONESEC		(drv_usectohz(1*1000000))

#define	IBP_VADDR(dp)	((void *)((dp)->io_area))

#if defined(MAP_MEM) || defined(__sparc)
#define	FLSHL(dp, r)	(void)INL(dp, r)
#define	FLSHW(dp, r)	(void)INW(dp, r)
#else
#define	FLSHL(dp, r)
#define	FLSHW(dp, r)
#endif

#define	RXDESC(t)	((struct rx_desc *)((void *)(t)))
#define	TXDESC(t)	((struct tx_desc *)((void *)(t)))

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int ae_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (ae_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Our configration for PCNET
 */
#define	MAXTXFRAGS	(min(8, GEM_MAXTXFRAGS))
#define	MAXRXFRAGS	1

#define	TX_RING_SIZE_P2	6		/* 64 */
#define	RX_RING_SIZE_P2	8		/* 256 */

#define	TX_RING_SIZE	(1 << TX_RING_SIZE_P2)
#define	RX_RING_SIZE	(1 << RX_RING_SIZE_P2)

#ifndef	TX_BUF_SIZE
#define	TX_BUF_SIZE	TX_RING_SIZE
#endif

#ifndef	RX_BUF_SIZE
#define	RX_BUF_SIZE	RX_RING_SIZE
#endif

#define	OUR_INTR_MASK	\
	(CSR0_BABL | CSR0_MISS | CSR0_MERR | CSR0_RINT | CSR0_TINT)

static int	ae_tx_copy_thresh = 256;
static int	ae_rx_copy_thresh = 256;

/*
 * Local device definitions
 */
struct chip_info {
	uint16_t	partid;
	char		*name;
	uint_t		flags;
#define	CAP_MII		0x0001
#define	CAP_FDX		0x0002
#define	CAP_CSR5	0x0004
#define	CAP_NOUFLO	0x0008
#define	CAP_CSR7	0x0010

} chiptable[] = {
{
	0x2420, "AMD PCnet/PCI AM79C970",
	0,
},
{
	0x2430, "AMD PCnet/PCI AM79C970",
	0,
},
{
	0x2621, "AMD PCnet/PCI II AM79C970A",
	CAP_FDX | CAP_CSR5,
},
{
	0x2623, "AMD PCnet/FAST AM79C971",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
{
	0x2624, "AMD PCnet/FAST+ AM79C972",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
{
	0x2625, "AMD PCnet/FAST III AM79C973",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
{
	0x2626, "AMD PCnet/Home AM79C978",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
{
	0x2627, "AMD PCnet/FAST III AM79C975",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
{
	0x2628, "AMD PCnet/PRO AM79C976",
	CAP_FDX | CAP_MII | CAP_CSR5 | CAP_CSR7 | CAP_NOUFLO,
},
};

#define	CHIPTABLESIZE	(sizeof (chiptable)/sizeof (struct chip_info))

struct ae_dev {
	struct chip_info	*chip;
	kmutex_t		reglock;

	/* register shadows */
	uint16_t		rap;
	uint16_t		csr0;
	uint16_t		csr3;
#ifdef CONFIG_POLLING
	uint16_t		csr7;
#endif
	/* mii phy emulator state */
	uint16_t		bmcr;
	uint16_t		bmsr;
	uint16_t		adv;
#ifdef CONFIG_POLLING
	clock_t			last_poll_interval;
	boolean_t		do_polling;
#endif
	boolean_t		check_link_state;
};

/*
 * private functions
 */

/* mii operations */
static void  ae_mii_sync(struct gem_dev *);
static uint16_t ae_mii_read(struct gem_dev *, uint_t);
static void ae_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int ae_attach_chip(struct gem_dev *);
static int ae_reset_chip(struct gem_dev *);
static int ae_init_chip(struct gem_dev *);
static int ae_start_chip(struct gem_dev *);
static int ae_stop_chip(struct gem_dev *);
static int ae_set_media(struct gem_dev *);
static int ae_set_rx_filter(struct gem_dev *);
static int ae_get_stats(struct gem_dev *);

/* descriptor operations */
static int ae_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void ae_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t ae_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t ae_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void ae_tx_desc_init(struct gem_dev *dp, int slot);
static void ae_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t ae_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr ae_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr ae_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t ae_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t ae_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffull,		/* dma_attr_count_max */
	16,			/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
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

/* =============================================================== */
/*
 * Local IO support
 */
/* =============================================================== */
static uint16_t
ae_csr_read(struct gem_dev *dp, uint16_t reg)
{
	struct ae_dev	*lp = dp->private;

	ASSERT(mutex_owned(&lp->reglock));
	if (lp->rap != reg) {
		OUTL(dp, RAP, reg);
		FLSHL(dp, RAP);
		lp->rap = reg;
	}
	return ((uint16_t)INL(dp, RDP));
}

static void
ae_csr_write(struct gem_dev *dp, uint16_t reg, uint16_t val)
{
	struct ae_dev	*lp = dp->private;

	ASSERT(mutex_owned(&lp->reglock));
	if (lp->rap != reg) {
		OUTL(dp, RAP, reg);
		FLSHL(dp, RAP);
		lp->rap = reg;
	}
	OUTL(dp, RDP, val);
	FLSHL(dp, RDP);
}

static uint16_t
ae_bcr_read(struct gem_dev *dp, uint16_t reg)
{
	struct ae_dev	*lp = dp->private;

	ASSERT(mutex_owned(&lp->reglock));
	if (lp->rap != reg) {
		OUTL(dp, RAP, reg);
		FLSHL(dp, RAP);
		lp->rap = reg;
	}
	return ((uint16_t)INL(dp, BDP));
}

static void
ae_bcr_write(struct gem_dev *dp, uint16_t reg, uint16_t val)
{
	struct ae_dev	*lp = dp->private;

	ASSERT(mutex_owned(&lp->reglock));
	if (lp->rap != reg) {
		OUTL(dp, RAP, reg);
		FLSHL(dp, RAP);
		lp->rap = reg;
	}
	OUTL(dp, BDP, val);
	FLSHL(dp, BDP);
}

/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
ae_reset_chip(struct gem_dev *dp)
{
	struct ae_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&lp->reglock);
	OUTW(dp, RAP_W, CSR88);
	FLSHW(dp, RAP_W);
	if (INW(dp, RAP_W) == CSR88) {
		/* reset in word mode */
		DPRINTF(0, (CE_CONT, "!%s: %s: reset in word mode",
		    dp->name, __func__));
		(void) INW(dp, RST_W);
	} else {
		/* reset in double word mode */
		DPRINTF(0, (CE_CONT, "!%s: %s: reset in double word mode",
		    dp->name, __func__));
		(void) INL(dp, RST);
	}
	lp->rap = CSR0;
	drv_usecwait(10);

	/* ensure that we are in double word mode */
	OUTL(dp, RDP, 0);
	FLSHL(dp, RDP);

	/* move to 32bit software style */
	ae_csr_write(dp, CSR58, CSR58_SWSTYLE_PCNETPCI);

	mutex_exit(&lp->reglock);

	DPRINTF(1, (CE_CONT, "!%s: %s: done", dp->name, __func__));

	return (GEM_SUCCESS);
}

/*
 * Setup am79c790
 */
static int
ae_init_chip(struct gem_dev *dp)
{
	uint16_t		val;
	struct init_block	*ibp;
	struct ae_dev		*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* setup initialization block */
	ibp = IBP_VADDR(dp);
	bzero(ibp, sizeof (struct init_block));
	ibp->ib_rdra = LE_32(dp->rx_ring_dma);
	ibp->ib_tdra = LE_32(dp->tx_ring_dma);
	ibp->ib_rlen = LE_8(RX_RING_SIZE_P2 << IB_LEN_SHIFT);
	ibp->ib_tlen = LE_8(TX_RING_SIZE_P2 << IB_LEN_SHIFT);

	mutex_enter(&lp->reglock);

	/* CSR1, CSR2: initialize block address */
	ae_csr_write(dp, CSR1, (uint16_t)dp->io_area_dma);
	ae_csr_write(dp, CSR2, (uint16_t)(dp->io_area_dma >> 16));

	/* CSR3: interrupt mask and bus swap */
#if DEBUG_LEVEL
	cmn_err(CE_CONT, "!%s: %s: csr3:0x%b",
	    dp->name, __func__, ae_csr_read(dp, CSR3), CSR3_BITS);
#endif
	lp->csr3 = CSR0_ISR & ~OUR_INTR_MASK;
	if ((lp->chip->flags & CAP_CSR5) == 0) {
		/*
		 * We need to restart nic
		 * on IDON interrupt after re-initialization
		 * due to change rx filter setting.
		 */
		lp->csr3 &= ~CSR0_IDON;
	}
	if (lp->chip->flags & CAP_NOUFLO) {
		lp->csr3 |= CSR3_DXSUFLO;
	}
	ae_csr_write(dp, CSR3, lp->csr3);

	/* CSR4: test and feature control */
#if DEBUG_LEVEL
	cmn_err(CE_CONT, "!%s: %s: csr4:0x%b",
	    dp->name, __func__, ae_csr_read(dp, CSR4), CSR4_BITS);
#endif
	ae_csr_write(dp, CSR4, CSR4_TXDPOLL | CSR4_APAD_XMT | CSR4_TXSTRTM);

	/* CSR5: extended interrupt control 1 */
	if (lp->chip->flags & CAP_CSR5) {
		DPRINTF(0, (CE_CONT, "!%s: %s: csr5:0x%b",
		    dp->name, __func__, ae_csr_read(dp, CSR5), CSR5_BITS));
		ae_csr_write(dp, CSR5, CSR5_TOKINTD | CSR5_LTINTEN);
	}

#ifdef CONFIG_POLLING
	/* CSR7: extended interrupt control 2 */
	if (lp->chip->flags & CAP_CSR7) {
		DPRINTF(0, (CE_CONT, "!%s: %s: csr7:0x%b",
		    dp->name, __func__, ae_csr_read(dp, CSR7), CSR7_BITS));
		lp->csr7 = 0;
	}
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: csr80:0x%x",
	    dp->name, __func__, ae_csr_read(dp, CSR80)));
	val = 0;
	if (dp->rxthr < 16) {
		val |= CSR80_RCVFW_16;
	} else if (dp->rxthr < 64) {
		val |= CSR80_RCVFW_64;
	} else {
		val |= CSR80_RCVFW_112;
	}

	if (dp->txthr <= 20) {
		val |= CSR80_XMTSP_20;
	} else if (dp->txthr <= 64) {
		val |= CSR80_XMTSP_64;
	} else if (dp->txthr <= 128) {
		val |= CSR80_XMTSP_128;
	} else {
		val |= CSR80_XMTSP_248;
	}
	if (dp->txmaxdma <= 16) {
		val |= CSR80_XMTFW_16;
	} else if (dp->txmaxdma <= 64) {
		val |= CSR80_XMTFW_64;
	} else {
		val |= CSR80_XMTFW_108;
	}
	val |= min(max(dp->txmaxdma, dp->rxmaxdma), CSR80_DMATC);
	ae_csr_write(dp, CSR80, val);
	DPRINTF(0, (CE_CONT, "!%s: %s: new csr80:0x%x",
	    dp->name, __func__, ae_csr_read(dp, CSR80)));

	DPRINTF(0, (CE_CONT, "!%s: %s: bsbc:0x%b",
	    dp->name, __func__, ae_bcr_read(dp, BSBC), BSBC_BITS));

	val = ae_bcr_read(dp, BSBC) | BSBC_MEMCMD;
	if ((lp->chip->flags & CAP_NOUFLO) && dp->txthr >= dp->mtu) {
		val |= BSBC_NOUFLO;
	}
	ae_bcr_write(dp, BSBC, val);
	DPRINTF(0, (CE_CONT, "!%s: %s: new bsbc:0x%b",
	    dp->name, __func__, ae_bcr_read(dp, BSBC), BSBC_BITS));

	mutex_exit(&lp->reglock);

#ifdef CONFIG_POLLING
	lp->last_poll_interval = 0;
#endif
	DPRINTF(1, (CE_CONT, "!%s: %s: done", dp->name, __func__));

	return (GEM_SUCCESS);
}

static uint_t
ae_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	return (gem_ether_crc_le(addr, ETHERADDRL) >> (32 - 6));
}

static int
ae_set_rx_filter(struct gem_dev *dp)
{
	int			i;
	uint16_t		mode;
	uint16_t		val;
	uint64_t		mhash;
	struct init_block	*ibp = IBP_VADDR(dp);
	struct ae_dev		*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called, rxmode:%b, mccnt:%d",
	    dp->name, __func__, dp->rxmode, RXMODE_BITS, dp->mc_count));

	/* clear mode and hash table */
	mode = (lp->chip->flags & CAP_MII) ?
	    CSR15_PORTSEL_MII : CSR15_PORTSEL_10BASET;
	mhash = 0;

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* do nothing otherwise it will case infinit recursive calls. */
		return (GEM_SUCCESS);
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscious mode implies all multicast and all physical */
		mode |= CSR15_PROM;
		mhash = ~0ull;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* accept all multicast packets */
		mhash = ~0ull;
	} else {
		/*
		 * make hash table to select interrested
		 * multicast address only.
		 */
		for (i = 0; i < dp->mc_count; i++) {
			mhash |= 1ull << dp->mc_list[i].hash;
		}
	}

	/* update mode */
	ibp->ib_mode = LE_16(mode);

	/* update hash table */
	ibp->ib_ladrf[0] = LE_32((uint32_t)mhash);
	ibp->ib_ladrf[1] = LE_32((uint32_t)(mhash >> 32));

	/* set mac address */
	bcopy(dp->cur_addr.ether_addr_octet, (void *)ibp->ib_padr, ETHERADDRL);

	if (!dp->mac_active) {
		DPRINTF(4, (CE_CONT, "!%s: returned", __func__));
		return (GEM_SUCCESS);
	}

	if (lp->chip->flags & CAP_CSR5) {
		/* we can notify the update to the nic gracefully */

		/* suspend the nic */
		mutex_enter(&lp->reglock);
		val = ae_csr_read(dp, CSR5);
		ae_csr_write(dp, CSR5, val | CSR5_SPND);

		for (i = 0; (ae_csr_read(dp, CSR5) & CSR5_SPND) == 0; i++) {
			if (i > 1000) {
				mutex_exit(&lp->reglock);
				cmn_err(CE_NOTE,
				    "!%s: %s: failed to suspend nic",
				    dp->name, __func__);
				goto reset_nic;
			}
			drv_usecwait(10);
		}

		/* write new mode and logical filter */
		ae_csr_write(dp, CSR15, LE_16(mode));
		for (i = 0; i < 4; i++) {
			ae_csr_write(dp, CSR8 + i, LE_16((uint16_t)mhash));
			mhash >>= 16;
		}

		/* resume the nic */
		ae_csr_write(dp, CSR5, val);
		mutex_exit(&lp->reglock);

		DPRINTF(4, (CE_CONT, "!%s: returned", __func__));
		return (GEM_SUCCESS);
	}

reset_nic:
	/* load initialization block into nic */
	mutex_enter(&lp->reglock);
	ae_csr_write(dp, CSR0, CSR0_STOP | lp->csr0);
	ae_csr_write(dp, CSR0, CSR0_INIT | lp->csr0);
	mutex_exit(&lp->reglock);

	DPRINTF(4, (CE_CONT, "!%s: returned", __func__));
	return (GEM_SUCCESS);
}

static int
ae_set_media(struct gem_dev *dp)
{
	struct ae_dev		*lp = dp->private;

	lp->do_polling =
	    (dp->speed == GEM_SPD_100) &&
	    (lp->chip->flags & CAP_CSR7) != 0;

	return (GEM_SUCCESS);
}

static int
ae_start_chip(struct gem_dev *dp)
{
	int			i;
	struct init_block	*ibp = IBP_VADDR(dp);
	struct ae_dev		*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* prepare initilization block for DMA. */
	gem_ioarea_dma_sync(dp,
	    (off_t)(((caddr_t)ibp) - dp->io_area),
	    sizeof (struct init_block), DDI_DMA_SYNC_FORDEV);

	mutex_enter(&lp->reglock);

	/* load initialization block into nic */
	ae_csr_write(dp, CSR0, CSR0_INIT);

	/* wait for 10 mS to complete initialization. */
	for (i = 0; (ae_csr_read(dp, CSR0) & CSR0_IDON) == 0; i++) {
		if (i > 1000) {
			cmn_err(CE_WARN,
			    "!%s: %s: failed to load initialization block "
			    "into nic",
			    dp->name, __func__);
			mutex_exit(&lp->reglock);
			return (GEM_FAILURE);
		}
		drv_usecwait(10);
	}

	/* clear bogus interrupt and status bits */
	ae_csr_write(dp, CSR0, ae_csr_read(dp, CSR0) & CSR0_STAT);

	/* kick rx and tx */
	lp->csr0 = 0;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		lp->csr0 |= CSR0_IENA;
	}
	ae_csr_write(dp, CSR0, CSR0_STRT | lp->csr0);

	mutex_exit(&lp->reglock);

	return (GEM_SUCCESS);
}

static int
ae_stop_chip(struct gem_dev *dp)
{
	struct ae_dev	*lp = dp->private;
	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable interrupts and stop nic */
	mutex_enter(&lp->reglock);
	lp->csr0 &= ~CSR0_IENA;
	ae_csr_write(dp, CSR0, CSR0_STOP | lp->csr0);
	mutex_exit(&lp->reglock);

	return (GEM_SUCCESS);
}

static int
ae_get_stats(struct gem_dev *dp)
{
	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));
	/* do nothing */
	return (GEM_SUCCESS);
}

/*
 * discriptor  manupiration
 */
static int
ae_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
#ifdef DEBUG_LEVEL
	int			i;
#endif
	struct tx_desc		*tdp;
	ddi_dma_cookie_t	*dcp;
	uint32_t		mark;
	int			curslot;
	int			n;
	struct ae_dev	*lp = dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum %d, slot %d, frags %d flag 0x%llx",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr 0x%x, len 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor(s) in reversed order
	 */
	mark = TMD1_OWN | TMD1_ENP;
	if ((flag & GEM_TXFLAG_INTR) && (lp->chip->flags & CAP_CSR5)) {
		mark |= TMD1_LTINT;
	}

	dcp = &dmacookie[frags - 1];
	for (n = frags - 1; n > 0; n--, dcp--) {
		curslot = SLOT(slot + n, TX_RING_SIZE);

		tdp = &TXDESC(dp->tx_ring)[curslot];
		mark |= (-dcp->dmac_size) & TMD1_BCNT;
		tdp->tmd0 = LE_32(dcp->dmac_address);
		tdp->tmd2 = 0;
		tdp->tmd1 = LE_32(mark);
		mark = TMD1_OWN;
	}
	/*
	 * specify descriptor control flags for first fragment.
	 */
	if (flag & GEM_TXFLAG_HEAD) {
		mark &= ~TMD1_OWN;
	}
	mark |= (TMD1_STP | ((-dcp->dmac_size) & TMD1_BCNT));

	tdp = &TXDESC(dp->tx_ring)[slot];
	tdp->tmd0 = LE_32(dcp->dmac_address);
	tdp->tmd2 = 0;
	tdp->tmd1 = LE_32(mark);

	return (frags);
}

static void
ae_tx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	struct ae_dev	*lp = dp->private;

	if (nslots > 1) {
		gem_tx_desc_dma_sync(dp,
		    SLOT(start_slot + 1, TX_RING_SIZE),
		    nslots - 1, DDI_DMA_SYNC_FORDEV);
	}

	TXDESC(dp->tx_ring)[start_slot].tmd1 |= LE_32(TMD1_OWN);

	gem_tx_desc_dma_sync(dp, start_slot, 1, DDI_DMA_SYNC_FORDEV);

	if (dp->mac_active) {
		/* kick Tx engine */
		mutex_enter(&lp->reglock);
		ae_csr_write(dp, CSR0, CSR0_TDMD | lp->csr0);
		mutex_exit(&lp->reglock);
	}
}

static void
ae_rx_desc_write(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	struct rx_desc		*rdp;

	rdp = &RXDESC(dp->rx_ring)[slot];

	rdp->rmd0 = LE_32(dmacookie->dmac_address);
	rdp->rmd2 = 0;
	rdp->rmd1 = LE_32(RMD1_OWN | ((-dmacookie->dmac_size) & RMD1_BCNT));
}

static uint_t
ae_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint_t		err = GEM_TX_DONE;
	uint32_t	tmd1;
	uint32_t	tmd2;
	struct tx_desc	*tdp;
	struct ae_dev	*lp = dp->private;

	/* XXX - we must check last descripor of the packet. */
	tdp = &TXDESC(dp->tx_ring)[SLOT(slot + ndesc - 1, TX_RING_SIZE)];
	tmd1 = tdp->tmd1;
	tmd1 = LE_32(tmd1);

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d ndesc:%d",
	    dp->name, __func__, slot, ndesc));

	if (tmd1 & TMD1_OWN) {
		/* not transmitted yet */
		return (0);
	}
	tmd2 = tdp->tmd2;
	tmd2 = LE_32(tmd2);

	DPRINTF(2, (CE_CONT, "!%s: %s: tmd1:%b, tmd2:%b",
	    dp->name, __func__, tmd1, TMD1_BITS, tmd2, TMD2_BITS));

	if (tmd2 & TMD2_ANYERROR) {
		dp->stats.errxmt++;
		if ((tmd2 & (TMD2_BUFF | TMD2_RTRY | TMD2_LCOL))
		    == TMD2_BUFF) {
			DPRINTF(1, (CE_WARN, "!%s: %s: TMD2_BUFF set, tmd2:%b",
			    dp->name, __func__, tmd2, TMD2_BITS));
			/* EMPTY */
		} else if (tmd2 & TMD2_UFLO) {
			dp->stats.underflow++;
			if ((lp->chip->flags & CAP_NOUFLO) == 0) {
				/* need to restart TX */
				err = GEM_TX_ERR;
			}
			DPRINTF(1, (CE_NOTE,
			    "!%s: %s: UFLO set, tmd2:%b",
			    dp->name, __func__, tmd2, TMD2_BITS));
		} else if (tmd2 & TMD2_LCOL) {
			dp->stats.xmtlatecoll++;
		} else if ((tmd2 & (TMD2_LCAR | TMD2_RTRY)) == TMD2_LCAR) {
			dp->stats.nocarrier++;
		} else if (tmd2 & TMD2_RTRY) {
			dp->stats.excoll++;
			dp->stats.collisions += 16;
		} else {
			dp->stats.xmit_internal_err++;
		}
	} else {
		if (tmd1 & TMD1_ONE) {
			dp->stats.first_coll++;
			dp->stats.collisions++;
		} else if (tmd1 & TMD1_MORE) {
			dp->stats.multi_coll++;
			dp->stats.collisions += (tmd2 & TMD2_TRC);
		} else if (tmd1 & TMD1_DEF) {
			dp->stats.defer++;
		}
	}

	return (err);
}

static uint64_t
ae_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	rmd2;
	uint32_t	rmd1;
	uint_t		len;
	struct rx_desc	*rdp;

	rdp = &RXDESC(dp->rx_ring)[slot];

	rmd1 = rdp->rmd1;
	rmd1 = LE_32(rmd1);

	DPRINTF(2, (CE_CONT, "!%s: %s: rmd1:%b, rmd2:%x",
	    dp->name, __func__, rmd1, RMD1_BITS, LE_32(rdp->rmd2)));

	if (rmd1 & RMD1_OWN)  {
		return (0);
	}
	rmd2 = rdp->rmd2;
	rmd2 = LE_32(rmd2);

	if (rmd1 & RMD1_ERR) {
		dp->stats.errrcv++;

		if (rmd1 & RMD1_FRAM) {
			dp->stats.frame++;
		} else if (rmd1 & RMD1_OFLO) {
			dp->stats.overflow++;
		} else if (rmd1 & RMD1_CRC) {
			dp->stats.crc++;
		} else {
			/* RMD1_BUFF, RMD1_BPE */
			dp->stats.rcv_internal_err++;
		}
		DPRINTF(2, (CE_CONT, "!%s: %s: rmd1:%b, rmd2:%x",
		    dp->name, __func__, rmd1, RMD1_BITS, LE_32(rdp->rmd2)));
		return (GEM_RX_ERR | GEM_RX_DONE);
	}

	len = (rmd2 & RMD2_MCNT);

	if (len > ETHERMAX + ETHERFCSL) {
		dp->stats.errrcv++;
		dp->stats.frame_too_long++;
		DPRINTF(2, (CE_CONT, "!%s: %s: rmd1:%b, rmd2:%x",
		    dp->name, __func__, rmd1, RMD1_BITS, LE_32(rdp->rmd2)));
		return (GEM_RX_ERR | GEM_RX_DONE);
	} else if (len < ETHERFCSL) {
		dp->stats.errrcv++;
		dp->stats.runt++;
		DPRINTF(2, (CE_CONT, "!%s: %s: rmd1:%b, rmd2:%x",
		    dp->name, __func__, rmd1, RMD1_BITS, LE_32(rdp->rmd2)));
		return (GEM_RX_ERR | GEM_RX_DONE);
	}


	return (GEM_RX_DONE | (len - ETHERFCSL));
}

static void
ae_tx_desc_init(struct gem_dev *dp, int slot)
{
	bzero(&TXDESC(dp->tx_ring)[slot], sizeof (struct tx_desc));
}

static void
ae_rx_desc_init(struct gem_dev *dp, int slot)
{
	bzero(&RXDESC(dp->rx_ring)[slot], sizeof (struct rx_desc));
}

/*
 * Device depend interrupt handler
 */
static uint_t
ae_interrupt(struct gem_dev *dp)
{
	boolean_t	need_to_reset = B_FALSE;
	uint16_t	csr0;
	uint16_t	csr7;
	uint_t		restart_tx = 0;
	struct ae_dev	*lp = dp->private;

	mutex_enter(&lp->reglock);

	csr0 = ae_csr_read(dp, CSR0);

	if (lp->chip->flags & CAP_CSR7) {
		csr7 = ae_csr_read(dp, CSR7);
	} else {
		csr7 = 0;
	}

	DPRINTF(2, (CE_CONT, "!%s: time:%d %s: csr0:0x%b, csr7:0x%b",
	    dp->name, ddi_get_lbolt(), __func__,
	    csr0, CSR0_BITS, csr7, CSR7_BITS));

	if (((csr0 & ~lp->csr3) & CSR0_ISR) == 0 &&
	    (csr7 & (CSR7_STINT | CSR7_STINTE))
	    != (CSR7_STINT | CSR7_STINTE)) {
		/* not for us */
		mutex_exit(&lp->reglock);
		return (DDI_INTR_UNCLAIMED);
	}

	/* clear all interrputs */
	ae_csr_write(dp, CSR0, csr0);

	if (csr7 & CSR7_STINT) {
		/* also clear timer interrupt */
		ae_csr_write(dp, CSR7, csr7);
	}
	mutex_exit(&lp->reglock);

	if (!dp->mac_active) {
		/* inhibit interrupts */
		mutex_enter(&lp->reglock);
		ae_csr_write(dp, CSR0, 0);
		mutex_exit(&lp->reglock);

		/* ack to all interrupts */
		return (DDI_INTR_CLAIMED);
	}

#ifdef CONFIG_POLLING
	if (lp->do_polling && dp->poll_interval != lp->last_poll_interval) {
		mutex_enter(&lp->reglock);
		/*
		 * It's time to check tx and rx statistics
		 */
		if (dp->poll_interval) {
			uint32_t	val;
			/* polling mode */

			/* mask rx and tx interrupts */
			lp->csr3 |= CSR3_RINTM | CSR3_TINTM;

			/* compute and set software timer interval */
			val = dp->poll_interval/12800;
			val = max(val, 1);
			val = min(val, 0xffff);
			ae_bcr_write(dp, STVAL, (uint16_t)val);

			/* enable software interval timer */
			lp->csr7 |= CSR7_STINTE;

			/* pretend as we were interrupted from polling timer */
			csr7 |= CSR7_STINT;
		} else {
			/* normal mode */
			/* enable rx and tx interrupts */
			lp->csr3 &= ~(CSR3_RINTM | CSR3_TINTM);

			/* disable software interval timer interrupt */
			lp->csr7 &= ~CSR7_STINTE;
		}
		ae_csr_write(dp, CSR3, lp->csr3);
		ae_csr_write(dp, CSR7, lp->csr7);
		mutex_exit(&lp->reglock);

		lp->last_poll_interval = dp->poll_interval;
	}

	if (csr7 & CSR7_STINT) {
		/* force to process ROK and TOK */
		csr0 |= (CSR0_RINT | CSR0_TINT);
	}
#endif /* CONFIG_POLLING */

	if (csr0 & CSR0_RINT) {
		(void) gem_receive(dp);
	}

	if (csr0 & CSR0_TINT) {
		if (gem_tx_done(dp)) {
			restart_tx = INTR_RESTART_TX;
		}
	}

	if (csr0 & CSR0_IDON) {
		need_to_reset = B_TRUE;
		DPRINTF(-2, (CE_CONT, "!%s: time:%d %s: csr0:0x%b, csr7:0x%b",
		    dp->name, ddi_get_lbolt(), __func__,
		    csr0, CSR0_BITS, csr7, CSR7_BITS));
	}

#define	CSR0_HWERR	(CSR0_BABL | CSR0_MERR | CSR0_MISS)
	if ((csr0 & (CSR0_HWERR | CSR0_TXON | CSR0_RXON))
	    != (CSR0_TXON | CSR0_RXON)) {
		DPRINTF(-2, (CE_CONT, "!%s: hardware error: csr0:%b",
		    dp->name, csr0, CSR0_BITS));
		need_to_reset = B_TRUE;
	}
#undef CSR0_HWERR

	if (need_to_reset) {
		(void) gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);

		restart_tx = INTR_RESTART_TX;
	}

	return (DDI_INTR_CLAIMED | restart_tx);
}

/*
 * MII Interfaces
 */
static void
ae_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
ae_mii_read(struct gem_dev *dp, uint_t index)
{
	uint16_t	val;
	struct ae_dev	*lp = dp->private;

	mutex_enter(&lp->reglock);
	ae_bcr_write(dp, MIIADDR,
	    (dp->mii_phy_addr << MIIADDR_ADDR_SHIFT) | index);
	val = ae_bcr_read(dp, MIIMDR);
	mutex_exit(&lp->reglock);

	return (val);
}

static void
ae_mii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	uint16_t	reg;
	struct ae_dev	*lp = dp->private;

	DPRINTF(10, (CE_CONT, "!%s: %s: reg:%d, val:0x%04x",
	    dp->name, __func__, index, val));

	mutex_enter(&lp->reglock);
	switch (index) {
	default:
		ae_bcr_write(dp, MIIADDR,
		    (dp->mii_phy_addr << MIIADDR_ADDR_SHIFT) | index);
		ae_bcr_write(dp, MIIMDR, val);

		DPRINTF(10, (CE_CONT, "!%s: %s: read back: val:0x%04x",
		    dp->name, __func__, ae_bcr_read(dp, MIIMDR)));
		break;
#ifdef CONFIG_ANAS
	case MII_CONTROL:
		/*
		 * As we cannot directly access MII_CONTROL
		 * register, we use MIICAS register, instead of
		 * MII_CONTROL register.
		 */

		/* disable AN setup first */
		reg = ae_bcr_read(dp, MIICAS) & ~MIICAS_XPHYRST;
		ae_bcr_write(dp, MIICAS, reg | MIICAS_DANAS);

		reg &= ~(MIICAS_DANAS | MIICAS_XPHYFD |
		    MIICAS_XPHYSP | MIICAS_XPHYANE);

		if (val & MII_CONTROL_RESET) {
			reg |= MIICAS_XPHYRST;
		}
		if (val & (MII_CONTROL_ANE)) {
			reg |= MIICAS_XPHYANE;
		}
		if (val & MII_CONTROL_FDUPLEX) {
			reg |= MIICAS_XPHYFD;
		}
		if ((val & MII_CONTROL_SPEED) == MII_CONTROL_100MB) {
			reg |= MIICAS_XPHYSP;
		}
		ae_bcr_write(dp, MIICAS, reg);
		break;
#endif /* CONFIG_ANAS */
	}
	mutex_exit(&lp->reglock);
}

static uint16_t
ae_nomii_read(struct gem_dev *dp, uint_t index)
{
	uint16_t	ret;
	uint16_t	val;
	struct ae_dev	*lp = dp->private;

	ret = 0;
	switch (index) {
	case MII_CONTROL:
		ret = lp->bmcr;
		break;

	case MII_STATUS:
		ret = lp->bmsr;

		if (lp->check_link_state) {
			mutex_enter(&lp->reglock);
			val = ae_bcr_read(dp, LED0);
			if (val != LED0_DEFAULT) {
				ret |= MII_STATUS_LINKUP;
			}
			mutex_exit(&lp->reglock);
			DPRINTF(1, (CE_CONT, "!%s: %s: LED0:%b",
			    dp->name, __func__, val, LED_BITS));
		} else {
			/*
			 * XXX - VMware doesn't seem to emulate  LED_LEDOUT
			 * bit in LED0 register, which indicates the  current
			 * link status.
			 */
			ret |= MII_STATUS_LINKUP;
		}

		break;

	case MII_AN_ADVERT:
		ret = lp->adv;
		break;
	}
	return (ret);
}

static void
ae_nomii_write(struct gem_dev *dp, uint_t index, uint16_t val)
{
	struct ae_dev	*lp = dp->private;

	switch (index) {
	case MII_CONTROL:
		lp->bmcr = val & ~MII_CONTROL_RESET;
		break;

	case MII_AN_ADVERT:
		lp->adv = val;
		break;

	default:
		cmn_err(CE_WARN,
		    "!%s: %s: writing to register %d in phy isn't permitted.",
		    dp->name, __func__, index);
		break;
	}
}

static int
ae_mii_probe(struct gem_dev *dp)
{
	uint16_t	val;
	struct ae_dev	*lp = dp->private;

	if ((lp->chip->flags & CAP_MII) == 0) {
		/*
		 * prepare for MII PHY emulation
		 */

		/* We don't have a MII phy. No need to scan it */
		dp->mii_phy_addr = -1;

		lp->bmsr = MII_STATUS_10;
		if (lp->chip->flags & CAP_FDX) {
			lp->bmsr |= MII_STATUS_10_FD;
		}
		dp->gc.gc_mii_read = &ae_nomii_read;
		dp->gc.gc_mii_write = &ae_nomii_write;
	}

	return (gem_mii_probe_default(dp));
}

static int
ae_mii_init(struct gem_dev *dp)
{
	uint16_t	val;
	struct ae_dev	*lp = dp->private;

	mutex_enter(&lp->reglock);

	if (lp->chip->flags & CAP_MII) {
		/* force to select MII port */
		ae_csr_write(dp, CSR15,
		    (ae_csr_read(dp, CSR15) & ~CSR15_PORTSEL)
		    | CSR15_PORTSEL_MII);
		ae_bcr_write(dp, MC, ae_bcr_read(dp, MC) & ~MC_ASEL);
#ifdef CONFIG_ANAS
		/* For MII media, enable MII auto polling */
		ae_bcr_write(dp, MIICAS,
		    (ae_bcr_read(dp, MIICAS) & ~MIICAS_XPHYRST)
		    | MIICAS_APEP);
#else
		/* disable AN auto setup, enable MII auto polling */
		ae_bcr_write(dp, MIICAS, 
		    (ae_bcr_read(dp, MIICAS) & ~MIICAS_XPHYRST)
		    | MIICAS_APEP | MIICAS_DANAS);
#endif
	} else {
		/* enable automatic media selection */
		ae_bcr_write(dp, MC, ae_bcr_read(dp, MC) | MC_ASEL);

		/* For non-MII media, we must setup duplex mode manually */
		ASSERT(!dp->anadv_autoneg);
		val = ae_bcr_read(dp, FDC) & ~(FDC_AUIFD | FDC_FDEN);
		if (dp->full_duplex) {
			val |= FDC_AUIFD | FDC_FDEN;
		}
		ae_bcr_write(dp, FDC, val);
	}

	mutex_exit(&lp->reglock);

	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver kernel interface) routine
 */
/* ======================================================== */
static int
ae_attach_chip(struct gem_dev *dp)
{
	int			i;
	uint32_t		val;
	uint32_t		rev;
	uint8_t			*mac;
	struct chip_info	*p;
	static uint8_t		zeros[] = {0, 0, 0, 0, 0, 0 };
	struct ae_dev		*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	mutex_enter(&lp->reglock);

	/*
	 * read mac address from EEPROM
	 */
	mac = &dp->dev_addr.ether_addr_octet[0];

	val = INL(dp, APROM);
	mac[0] = (uint8_t)(val >> (8*0));
	mac[1] = (uint8_t)(val >> (8*1));
	mac[2] = (uint8_t)(val >> (8*2));
	mac[3] = (uint8_t)(val >> (8*3));

	val = INL(dp, APROM + 4);
	mac[4] = (uint8_t)(val >> (8*0));
	mac[5] = (uint8_t)(val >> (8*1));

	if ((mac[0] & 1) || bcmp(mac, zeros, ETHERADDRL) == 0) {
		/* factory address in eeprom is corrupted */
		gem_generate_macaddr(dp, mac);
	}

	rev = (ae_csr_read(dp, CSR89) << 16) | ae_csr_read(dp, CSR88);
	rev = (rev >> CSR88_PARTID_SHIFT) & CSR88_PARTID_MASK;

	for (i = 0, p = chiptable; i < CHIPTABLESIZE; i++, p++) {
		if (p->partid == rev) {
			/* found */
			cmn_err(CE_CONT, "!%s: partid: 0x%04x %s",
			    dp->name, p->partid, p->name);
			goto chip_found;
		}
	}

	/* Not found */
	cmn_err(CE_WARN, "!%s: %s: wrong partid: 0x%x",
	    dp->name, __func__, rev);
	return (GEM_FAILURE);

chip_found:
	lp->chip = p;

#if DEBUG_LEVEL > 5
	/* enable LED program */
	ae_bcr_write(dp, MC, ae_bcr_read(dp, MC) | MC_LEDPE);

	/* dump LED configuration */
	for (i = 0; i < 4; i++) {
		val = ae_bcr_read(dp, LED0 + i);
		cmn_err(CE_CONT, "!%s: LED%d:%b", dp->name, i, val, LED_BITS);
	}
	ae_bcr_write(dp, MC, ae_bcr_read(dp, MC) & ~MC_LEDPE);
#endif
	mutex_exit(&lp->reglock);
#ifdef CONFIG_ANAS
	/* pcnet doesn't support changing advertise register with ANAS */
	dp->mii_advert_ro = B_TRUE;
#endif
	lp->check_link_state =
	    gem_prop_get_int(dp, "check-link-state", 0) != 0;
	DPRINTF(1, (CE_CONT, "%s: %s: check-link-state:%d",
	    dp->name, __func__, lp->check_link_state));

#ifdef NEVER /* GEM_CONFIG_GLDv3 */
	/* VLAN is not supported */
	dp->misc_flag |= GEM_VLAN_SOFT;
#endif
	dp->misc_flag |= GEM_POLL_RXONLY;

	DPRINTF(1, (CE_CONT, "!%s: %s: done", dp->name, __func__));

	return (GEM_SUCCESS);	/* currently return code is not used. */
}

static int
aeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	ddi_acc_handle_t	conf_handle;
	int			vid;
	int			did;
	int			revid;
	int			unit;
	const char		*drv_name;
	struct gem_dev		*dp;
	void			*base;
	ddi_acc_handle_t	regs_handle;
	struct gem_conf		*gcp;
	struct ae_dev		*lp;
	uint32_t		ilr;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);
	DPRINTF(3, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	/*
	 * Common routine after power-on
	 */

	/* fix config registers */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		goto err;
	}

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    pci_config_get16(conf_handle, PCI_CONF_COMM) |
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME);

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
#ifdef lint
	vid = vid;
	did = did;
	ilr = ilr;
#endif
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/*
		 * Map in the device registers.
		 */
		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &ae_dev_attr,
		    (void *)&base, &regs_handle) != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * Check hardware revision
		 */
chip_found:
		cmn_err(CE_CONT, "!%s%d: rev:0x%x", drv_name, unit, revid);

		/*
		 * construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		(void) sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_desc_unit_shift = 4;
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = ae_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint32_t) - 1;
		gcp->gc_rx_max_frags = MAXRXFRAGS;
		gcp->gc_rx_desc_unit_shift = 4;
		gcp->gc_rx_ring_size = RX_RING_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE;
		gcp->gc_rx_copy_thresh = ae_rx_copy_thresh;

		gcp->gc_io_area_size = sizeof (struct init_block);

		/* map attributes */
		gcp->gc_dev_attr = ae_dev_attr;
		gcp->gc_buf_attr = ae_buf_attr;
		gcp->gc_desc_attr = ae_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = ae_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = ae_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = ae_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_NONE;

		/* mii mode */
		gcp->gc_mii_mode = GEM_MODE_100BASETX;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/10;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		gcp->gc_mii_an_delay = ONESEC/10;	/* 100mS */
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_an_oneshot = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &ae_attach_chip;
		gcp->gc_reset_chip = &ae_reset_chip;
		gcp->gc_init_chip = &ae_init_chip;
		gcp->gc_start_chip = &ae_start_chip;
		gcp->gc_stop_chip = &ae_stop_chip;
		gcp->gc_multicast_hash = &ae_mcast_hash;
		gcp->gc_set_rx_filter = &ae_set_rx_filter;
		gcp->gc_set_media = &ae_set_media;
		gcp->gc_get_stats = &ae_get_stats;
		gcp->gc_interrupt = &ae_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &ae_tx_desc_write;
		gcp->gc_rx_desc_write = &ae_rx_desc_write;
		gcp->gc_tx_start = &ae_tx_start;
		gcp->gc_rx_start = NULL;
		gcp->gc_tx_desc_stat = &ae_tx_desc_stat;
		gcp->gc_rx_desc_stat = &ae_rx_desc_stat;
		gcp->gc_tx_desc_init = &ae_tx_desc_init;
		gcp->gc_rx_desc_init = &ae_rx_desc_init;
		gcp->gc_tx_desc_clean = &ae_tx_desc_init;
		gcp->gc_rx_desc_clean = &ae_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &ae_mii_probe;
		gcp->gc_mii_init = &ae_mii_init;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &ae_mii_sync;
		gcp->gc_mii_read = &ae_mii_read;
		gcp->gc_mii_write = &ae_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* jumbo packet is not supported */
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);
		gcp->gc_max_mtu = ETHERMTU;

		lp = kmem_zalloc(sizeof (struct ae_dev), KM_SLEEP);

		mutex_init(&lp->reglock, NULL, MUTEX_DRIVER, NULL);

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_handle, lp, sizeof (struct ae_dev));

		kmem_free(gcp, sizeof (*gcp));

		if (dp == NULL) {
			mutex_destroy(&lp->reglock);
			goto err_free_mem;
		}

		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof (struct ae_dev));
err:
		return (DDI_FAILURE);
	}

	return (DDI_FAILURE);
}

static int
aedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	volatile int	x;
	struct gem_dev	*dp;
	struct ae_dev	*lp;

	dp = GEM_GET_DEV(dip);
	lp = dp->private;

	switch (cmd) {
	case DDI_DETACH:
		/* stop the chip completely */
		mutex_enter(&lp->reglock);
		x = INL(dp, RST);
#ifdef lint
		x = x;
#endif
		mutex_exit(&lp->reglock);

		mutex_destroy(&lp->reglock);
		return (gem_do_detach(dip));

	case DDI_SUSPEND:
		/* stop the chip completely */
		mutex_enter(&lp->reglock);
		x = INL(dp, RST);
		mutex_exit(&lp->reglock);

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
GEM_STREAM_OPS(ae_ops, aeattach, aedetach);
#else
static	struct module_info aeminfo = {
	0,			/* mi_idnum */
	"ae",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit aerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&aeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit aewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&aeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	ae_info = {
	&aerinit,	/* st_rdinit */
	&aewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_ae_ops = {
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
	&ae_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops ae_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	aeattach,	/* devo_attach */
	aedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_ae_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power	/* devo_power */
};
#endif /* GEM_CONFIG_GLDv3 */

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&ae_ops,	/* driver ops */
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
	int	status;

	DPRINTF(2, (CE_CONT, "!ae: _init: called"));

	gem_mod_init(&ae_ops, "ae");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&ae_ops);
	}
	return (status);
}

/*
 * _fini :
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!ae: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&ae_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
