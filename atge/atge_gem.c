/*
 * atge_gem.c: Attansic L1/L2, Atheros L1/L1C Ethernet MAC driver
 *
 * Copyright (c) 2008-2012 Masayuki Murayama.  All rights reserved.
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
 * Masafumi Ohta who tested the driver repeatedly on his eee PC
 */
#pragma	ident	"@(#)atge_gem.c	1.15 12/11/02"

/*
 * Change log
 *
 * 2008/10/05 2.6.1 released
 * 2009/07/08 L2 didn't exit power save mode in atge_reset_phy()
 *            MII_DBG_ADDR -> MII_DBG_DATA
 * 2009/07/09 2.6.5 released
 */

/*
 * TODO:
 * rx cksum - not supported because of no UDP cksum
 * L1C
 * force to load EEPROM
 * wol
 * msi_patch
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
#include <sys/atomic.h>

#include "gem_mii.h"
#include "gem.h"
#include "atl1reg.h"

char	ident[] = "atheros L1 driver v" VERSION;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int atge_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (atge_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC	drv_usectohz(1*1000000)
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#define	FLSHB(dp, reg)	(void) INB(dp, reg)
#define	FLSHW(dp, reg)	(void) INW(dp, reg)
#define	FLSHL(dp, reg)	(void) INL(dp, reg)

#define	UPDATE_MBOX_1(dp)	{	\
	struct atge_dev	*lp = dp->private;	\
	OUTL(dp, MBOX_1,	\
	    (lp)->tx_buffer_tail << MBOX_TBD_TAIL_SHIFT	\
	    | (lp)->rx_status_head << MBOX_RSD_HEAD_SHIFT	\
	    | (lp)->rx_buffer_tail << MBOX_RBD_TAIL_SHIFT);	\
}

#define	INSIDE(slot, head, tail)	\
	(((head) <= (tail)) ?	\
	    ((head) <= (slot) && (slot) < (tail)) :	\
	    ((slot) < (tail) || (head) <= (slot)))

#ifndef VTAG_SIZE
#define	VTAG_SIZE	4
#endif

#define	NITEMS(a)	(sizeof (a) / sizeof (a[0]))

#ifdef GEM3
#define	IS_MAC_ONLINE(dp)	((dp)->mac_state == MAC_STATE_ONLINE)
#define	mac_active	mac_state
#else
#define	IS_MAC_ONLINE(dp)	((dp)->mac_active)
#endif
/*
 * Our configuration
 */
#ifdef GEM_CONFIG_TX_DIRECT
#define	MAXTXFRAGS	min(GEM_MAXTXFRAGS, 18)
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
#define	TX_BUF_SIZE	256
#endif

#ifndef TX_RING_SIZE
#define	TX_RING_SIZE	(min(TX_BUF_SIZE * min(MAXTXFRAGS, 4), 512))
#endif

#ifndef RX_BUF_SIZE
#define	RX_BUF_SIZE	256
#endif

static int	atge_tx_copy_thresh = 256;
static int	atge_rx_copy_thresh = 256;

/*
 * the nic state
 */
struct l2_rxf {
	uint32_t	rf_head;	/* offset from rxf_offset[][] */
	uint32_t	rf_tail;	/* offset from rxf_offset[][] */
	int		rf_curr_ring;
	uint16_t	rf_seqnum;
};

struct atge_dev {
	boolean_t	initialized;
	boolean_t	need_to_reset;

	uint8_t		mac_addr[ETHERADDRL];
	uint8_t		revid;	/* chip revision id */

	uint32_t	mac_ctl;
	uint32_t	imr;
	uint32_t	isr_pended;

	volatile uint_t	tx_buffer_head;	/* L1 L1E L1C */
	uint_t		tx_buffer_tail;	/* L1 L1E L2 L1C */
	uint_t		rx_buffer_head;	/* L2 */
	uint_t		rx_buffer_tail;	/* L1 */
	uint_t		rx_status_head;	/* L1 */
	uint_t		rx_status_tail;	/* L1 */

	kmutex_t	mbox_lock;	/* L1 */

	offset_t	tbd_offset;
	offset_t	rbd_offset;	/* L1 L2 */
	offset_t	ism_offset;	/* L1 */
#define	tbd_tail_offset	ism_offset	/* L1 */
	offset_t	sm_offset;


	offset_t	rxf_offset[4][2];	/* L1E L2E */
	offset_t	rxf_mb_offset[4][2];	/* L1E L2E */
	size_t		rxf_buf_len;		/* L1E L2E */
	size_t		rxf_buf_hiwat;		/* L1E L2E */

	size_t		tbd_size;
#ifdef RESET_TEST
	int		reset_test;
#endif
	uint32_t	gphyc;
	boolean_t	phy_initialized;

	struct l2_rxf	rxf[4];
	int		chip;

	uint_t		tbd_pos[TX_BUF_SIZE];	/* L2 */
#ifdef CONFIG_ADAPTIVE_COALESCE
	int		last_poll_interval;
#endif
	/* interrupt status bits configuration */
	uint32_t	isr_disable;
	uint32_t	isr_enable;
	uint32_t	isr_tx_ok;
	uint32_t	isr_tx_err;
	uint32_t	isr_rx_ok;
	uint32_t	isr_rx_err;
	uint32_t	isr_dma_err;
	uint32_t	isr_gphy;
	uint32_t	isr_sm;
	uint32_t	isr_pcie_err;
	uint32_t	isr_fatal_err;
	uint32_t	isr_clear_all;
	char		*isr_bits;

	/* statistics */
	clock_t		last_stats_update;

	/* pci-e configuration info */
	uint_t		maxpayload;
	uint_t		maxrequest;

	boolean_t	intr_rdclr;

	uint_t		ctrl_flags;
#define	ATL1C_ASPM_L0S_SUPPORT	0x0001
#define	ATL1C_ASPM_L1_SUPPORT	0x0002

	boolean_t	msi_lnkpatch;
};

/*
 * Supported chips
 */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
	int		chip_type;
#define	CHIP_L1		0
#define	CHIP_L2		1
#define	CHIP_L1E	2
#define	CHIP_L2E_A	3
#define	CHIP_L2E_B	4
#define	CHIP_L1C	5
#define	CHIP_L2C	6
#define	CHIP_L2C_B	7
#define	CHIP_L2C_B2	8
#define	CHIP_L1D	9
#define	CHIP_L1D_2_0	10
};

static struct chip_info atge_chiptbl[] = {
	{ 0x1969, 0x1048, "Attansic L1", CHIP_L1},
	{ 0x1969, 0x2048, "Attansic L2", CHIP_L2},
	{ 0x1969, 0x1026, "Atheros AR8121/8113/8114", CHIP_L1E},
	{ 0x1969, 0x1063, "Atheros AR8131", CHIP_L1C},
	{ 0x1969, 0x1062, "Atheros AR8132", CHIP_L2C},
	{ 0x1969, 0x2060, "Atheros AR8152 v1.1", CHIP_L2C_B},
	{ 0x1969, 0x2062, "Atheros AR8152 v2.0", CHIP_L2C_B2},
	{ 0x1969, 0x1073, "Atheros AR8151 v1.0", CHIP_L1D},
	{ 0x1969, 0x1083, "Atheros AR8151 v2.0", CHIP_L1D_2_0},
};

static struct patch_info {
	uint16_t	devid;
	uint8_t		revid;
	uint16_t	subvenid;
	uint16_t	subsysid;
} patch_list[] = {
	{0x2060U, 0xC1U, 0x1019U, 0x8152U},
	{0x2060U, 0xC1U, 0x1019U, 0x2060U},
	{0x2060U, 0xC1U, 0x1019U, 0xE000U},
	{0x2062U, 0xC0U, 0x1019U, 0x8152U},
	{0x2062U, 0xC0U, 0x1019U, 0x2062U},
	{0x2062U, 0xC0U, 0x1458U, 0xE000U},
	{0x2062U, 0xC1U, 0x1019U, 0x8152U},
	{0x2062U, 0xC1U, 0x1019U, 0x2062U},
	{0x2062U, 0xC1U, 0x1458U, 0xE000U},
	{0x2062U, 0xC1U, 0x1565U, 0x2802U},
	{0x2062U, 0xC1U, 0x1565U, 0x2801U},
	{0x1073U, 0xC0U, 0x1019U, 0x8151U},
	{0x1073U, 0xC0U, 0x1019U, 0x1073U},
	{0x1073U, 0xC0U, 0x1458U, 0xE000U},
	{0x1083U, 0xC0U, 0x1458U, 0xE000U},
	{0x1083U, 0xC0U, 0x1019U, 0x8151U},
	{0x1083U, 0xC0U, 0x1019U, 0x1083U},
	{0x1083U, 0xC0U, 0x1462U, 0x7680U},
	{0x1083U, 0xC0U, 0x1565U, 0x2803U},
};


/*
 * Macros to identify chip generation.
 */

/* ======================================================== */

/* mii operations */
static void  atge_mii_sync(struct gem_dev *);
static uint16_t atge_mii_read_raw(struct gem_dev *, uint_t);
static uint16_t atge_mii_read_ext(struct gem_dev *, uint_t, uint_t);
static uint16_t atge_mii_read(struct gem_dev *, uint_t);
static void atge_mii_write_raw(struct gem_dev *, uint_t, uint16_t);
static void atge_mii_write_ext(struct gem_dev *, uint_t, uint_t, uint16_t);
static void atge_mii_write(struct gem_dev *, uint_t, uint16_t);
static int atge_mii_init(struct gem_dev *);
static void atge_mii_tune_phy(struct gem_dev *dp);

/* nic operations */
static int atge_attach_chip(struct gem_dev *);
static int atge_reset_chip(struct gem_dev *);
static int atge_init_chip_1(struct gem_dev *);
static int atge_start_chip(struct gem_dev *);
static int atge_stop_chip(struct gem_dev *);
static int atge_set_media(struct gem_dev *);
static int atge_set_rx_filter(struct gem_dev *);
static int atge_get_stats(struct gem_dev *);
static int atge_init_chip_1e(struct gem_dev *);
static int atge_init_chip_1c(struct gem_dev *);

/* descriptor operations */
static int atge_tx_desc_write_1_1e(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags);
static void atge_tx_start_1_1e(struct gem_dev *dp, int startslot, int nslot);
static void atge_rx_desc_write_1(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags);
static void atge_rx_start_1(struct gem_dev *dp, int startslot, int nslot);
#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t atge_tx_desc_head_1(struct gem_dev *dp);
static uint_t atge_tx_desc_head_1e(struct gem_dev *dp);
#else
static uint_t atge_tx_desc_stat_1_1e(struct gem_dev *dp, int slot, int ndesc);
#endif /* GEM_CONFIG_TX_HEAD_PTR */
static uint64_t atge_rx_desc_stat_1(struct gem_dev *dp, int slot, int ndesc);

static void atge_tx_desc_init_1_1e(struct gem_dev *dp, int slot);
static void atge_rx_desc_init_1(struct gem_dev *dp, int slot);

static void atge_rx_desc_write_1e(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags);
static uint64_t atge_rx_desc_stat_1e(struct gem_dev *dp, int slot, int ndesc);
static void atge_rx_desc_init_1e(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t atge_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr atge_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr atge_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t atge_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x00ffffffffffull,	/* dma_attr_addr_hi */
	0x000000003fffull,	/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0x00003ffc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000000027ffull,	/* dma_attr_maxxfer */
	0x0000ffffffffull,	/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t atge_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0x0000ffffffffull,	/* dma_attr_addr_hi */
	0x0000ffffffffull,	/* dma_attr_count_max */
	128,			/* dma_attr_align */
	0x0000fffc,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0x0000ffffffffull,	/* dma_attr_maxxfer */
	0x0000ffffffffull,	/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * HW manipulation routines
 */
/* ======================================================== */
#ifdef DEBUG_IOTRACE

#undef	OUTB
#undef	OUTW
#undef	OUTL
#undef	INB
#undef	INW
#undef	INL


#define	OUTB(dp, p, v)	\
	atge_put8((dp)->regs_handle, \
		(void *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	OUTW(dp, p, v)	\
	atge_put16((dp)->regs_handle, \
		(void *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	OUTL(dp, p, v)	\
	atge_put32((dp)->regs_handle, \
		(void *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	INB(dp, p)	\
	atge_get8((dp)->regs_handle, \
		(void *)(((caddr_t)(dp)->base_addr) + (p)))
#define	INW(dp, p)	\
	atge_get16((dp)->regs_handle, \
		(void *)(((caddr_t)(dp)->base_addr) + (p)))
#define	INL(dp, p)	\
	atge_get32((dp)->regs_handle, \
		(void *)(((caddr_t)(dp)->base_addr) + (p)))
#if 0
extern uint8_t atge_get8(ddi_acc_handle_t handle, uint8_t *dev_addr);
extern uint16_t atge_get16(ddi_acc_handle_t handle, uint16_t *dev_addr);
extern uint32_t atge_get32(ddi_acc_handle_t handle, uint32_t *dev_addr);
extern uint64_t atge_get64(ddi_acc_handle_t handle, uint64_t *dev_addr);
extern void atge_put8(ddi_acc_handle_t handle, uint8_t *dev_addr,
    uint8_t value);
extern void atge_put16(ddi_acc_handle_t handle, uint16_t *dev_addr,
    uint16_t value);
extern void atge_put32(ddi_acc_handle_t handle, uint32_t *dev_addr,
    uint32_t value);
extern void atge_put64(ddi_acc_handle_t handle, uint64_t *dev_addr,
    uint64_t value);
#endif

uint8_t
atge_get8(ddi_acc_handle_t handle, uint8_t *dev_addr)
{
	uint8_t	ret;

	ret = ddi_get8(handle, dev_addr);
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) -> %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    ret));
	return (ret);
}

uint16_t
atge_get16(ddi_acc_handle_t handle, uint16_t *dev_addr)
{
	uint16_t	ret;

	ret = ddi_get16(handle, dev_addr);
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) -> %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    ret));
	return (ret);
}

uint32_t
atge_get32(ddi_acc_handle_t handle, uint32_t *dev_addr)
{
	uint32_t	ret;

	ret = ddi_get32(handle, dev_addr);
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) -> %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    ret));
	return (ret);
}

uint64_t
atge_get64(ddi_acc_handle_t handle, uint64_t *dev_addr)
{
	uint64_t	ret;

	ret = ddi_get64(handle, dev_addr);

	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) -> %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    ret));
	return (ret);
}

void
atge_put8(ddi_acc_handle_t handle, uint8_t *dev_addr, uint8_t value)
{
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) <- %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    value));
	ddi_put8(handle, dev_addr, value);
}

void
atge_put16(ddi_acc_handle_t handle, uint16_t *dev_addr, uint16_t value)
{
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) <- %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    value));
	ddi_put16(handle, dev_addr, value);
}

void
atge_put32(ddi_acc_handle_t handle, uint32_t *dev_addr, uint32_t value)
{
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) <- %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    value));
	ddi_put32(handle, dev_addr, value);
}

void
atge_put64(ddi_acc_handle_t handle, uint64_t *dev_addr, uint64_t value)
{
	DPRINTF(-1, (CE_CONT, "!%s: %x %x(%x) <- %x",
	    __func__, handle, dev_addr,
	    ((caddr_t)dev_addr) - ((ddi_acc_hdl_t *)handle)->ah_addr,
	    value));
	ddi_put64(handle, dev_addr, value);
}
#endif /* DEBUG_IOTRACE */

__INLINE__ static void
atge_ioarea_dma_sync(struct gem_dev *dp, off_t start, size_t len, int how)
{
	(void) ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)(dp->io_area_dma - dp->rx_ring_dma + start),
	    len, how);
}

__INLINE__ static void
atge_rbd_desc_dma_sync_1(struct gem_dev *dp, int head, int nslot)
{
	int	n;
	int	m;
	struct atge_dev	*lp = dp->private;

	/* sync active descriptors */
	if (nslot == 0) {
		/* no rx descriptor ring */
		return;
	}

	n = dp->gc.gc_rx_ring_size - head;
	if ((m = nslot - n) > 0) {
		(void) atge_ioarea_dma_sync(dp,
		    (off_t)lp->rbd_offset,
		    (size_t)(m * sizeof (struct rx_buffer_desc)),
		    DDI_DMA_SYNC_FORDEV);
		nslot = n;
	}

	(void) atge_ioarea_dma_sync(dp,
	    (off_t)(head * sizeof (struct rx_buffer_desc) + lp->rbd_offset),
	    (size_t)(nslot * sizeof (struct rx_buffer_desc)),
	    DDI_DMA_SYNC_FORDEV);
}

__INLINE__ static void
atge_rbd_desc_dma_sync_2(struct gem_dev *dp, int head, int nslot, uint_t type)
{
	int	n;
	int	m;
	struct atge_dev	*lp = dp->private;

	/* sync active descriptors */
	if (nslot == 0) {
		/* no rx descriptor ring */
		return;
	}

	n = dp->gc.gc_rx_ring_size - head;
	if ((m = nslot - n) > 0) {
		(void) atge_ioarea_dma_sync(dp, (off_t)lp->rbd_offset,
		    (size_t)(m * 1536), type);
		nslot = n;
	}

	(void) atge_ioarea_dma_sync(dp, (off_t)(head * 1536 + lp->rbd_offset),
	    (size_t)(nslot * 1536), type);
}

__INLINE__ static void
atge_rfd_desc_dma_sync_1c(struct gem_dev *dp, int head, int nslot)
{
	int	n;
	int	m;
	struct atge_dev	*lp = dp->private;

	/* sync active descriptors */
	if (nslot == 0) {
		/* no rx descriptor ring */
		return;
	}

	n = dp->gc.gc_rx_ring_size - head;
	if ((m = nslot - n) > 0) {
		(void) atge_ioarea_dma_sync(dp,
		    (off_t)lp->rbd_offset,
		    (size_t)(m * sizeof (struct rx_free_desc)),
		    DDI_DMA_SYNC_FORDEV);
		nslot = n;
	}

	(void) atge_ioarea_dma_sync(dp,
	    (off_t)(head * sizeof (struct rx_free_desc) + lp->rbd_offset),
	    (size_t)(nslot * sizeof (struct rx_free_desc)),
	    DDI_DMA_SYNC_FORDEV);
}

__INLINE__ static void
atge_tx_desc_dma_sync(struct gem_dev *dp, int head, int nslot)
{
	int	n;
	int	m;
	struct atge_dev	*lp = dp->private;

	/* sync active descriptors */
	if (nslot == 0) {
		/* no tx descriptor ring */
		return;
	}

	n = dp->gc.gc_tx_ring_size - head;
	if ((m = nslot - n) > 0) {
		(void) atge_ioarea_dma_sync(dp,
		    (off_t)lp->tbd_offset,
		    (size_t)(m * sizeof (struct tx_buffer_desc)),
		    DDI_DMA_SYNC_FORDEV);
		nslot = n;
	}

	(void) atge_ioarea_dma_sync(dp,
	    (off_t)(head * sizeof (struct tx_buffer_desc) + lp->tbd_offset),
	    (size_t)(nslot * sizeof (struct tx_buffer_desc)),
	    DDI_DMA_SYNC_FORDEV);
}

static void
atge_set_aspm_1c(struct gem_dev *dp, uint_t flags)
{
	uint32_t	val;
	uint32_t	link_l1_timer;
	struct atge_dev	*lp = dp->private;

	/* disable l0s_l1 */
	val = INL(dp, PM_CTRL);
#if 0
	val &= ~(PM_CTRL_L1_ENTRY_TIMER
	    | PM_CTRL_CLK_SWH_L1
	    | PM_CTRL_ASPM_L0S_EN
	    | PM_CTRL_ASPM_L1_EN
	    | PM_CTRL_MAC_ASPM_CHK
	    | PM_CTRL_SERDES_PD_EX_L1);

	val |= (PM_CTRL_SERDES_BUFS_RX_L1_EN
	    | PM_CTRL_SERDES_PLL_L1_EN
	    | PM_CTRL_SERDES_L1_EN);
#else
	val &= ~(PM_CTRL_ASPM_L1_EN |
	    PM_CTRL_ASPM_L0S_EN |
	    PM_CTRL_MAC_ASPM_CHK);

	/* L1 timer */
	if (lp->chip == CHIP_L2C_B2 || lp->chip == CHIP_L1D_2_0) {
		val &= ~PMCTRL_TXL1_AFTER_L0S;
		if (dp->mii_state == MII_STATE_LINKUP &&
		    (dp->speed == GEM_SPD_1000 || dp->speed == GEM_SPD_100)) {
			link_l1_timer = L1D_PMCTRL_L1_ENTRY_TM_16US;
		} else {
			link_l1_timer = 1;
		}
		val = (val & ~L1D_PMCTRL_L1_ENTRY_TM)
		    | link_l1_timer << L1D_PMCTRL_L1_ENTRY_TM_SHIFT;
	} else {
		if (dp->mii_state == MII_STATE_LINKUP &&
		    dp->speed == GEM_SPD_10) {
			link_l1_timer = 1;
		} else if (lp->chip == CHIP_L2C_B) {
			link_l1_timer = L2CB1_PM_CTRL_L1_ENTRY_TM;
		} else {
			link_l1_timer = L1C_PM_CTRL_L1_ENTRY_TM;
		}

		val = (val & ~PM_CTRL_L1_ENTRY_TIMER)
		    | link_l1_timer << PM_CTRL_L1_ENTRY_TIMER_SHIFT;
	}

	/* L0S/L1 enable */
	if ((flags & ATL1C_ASPM_L0S_SUPPORT) &&
	    dp->mii_state == MII_STATE_LINKUP) {
		val |= PM_CTRL_ASPM_L0S_EN | PM_CTRL_MAC_ASPM_CHK;
	}
	if (flags & ATL1C_ASPM_L1_SUPPORT) {
		val |= PM_CTRL_ASPM_L1_EN | PM_CTRL_MAC_ASPM_CHK;
	}

	/* L2CB & L1D & L2CB2 & L1D2 */
	switch (lp->chip) {
	case CHIP_L2C_B:
	case CHIP_L1D:
	case CHIP_L2C_B2:
	case CHIP_L1D_2_0:
		val &= ~PM_CTRL_PM_REQ_TIMER;
		val |= PM_CTRL_PM_REQ_TO_DEF << PM_CTRL_PM_REQ_TIMER_SHIFT;

		val |= PM_CTRL_RCVR_WT_TIMER |
		    PM_CTRL_SERDES_PD_EX_L1 |
		    PM_CTRL_CLK_SWH_L1;
		val &= ~(PM_CTRL_SERDES_L1_EN |
		    PM_CTRL_SERDES_PLL_L1_EN |
		    PM_CTRL_SERDES_BUFS_RX_L1_EN |
		    PM_CTRL_SA_DLY_EN |
		    PM_CTRL_HOTRST);

		/* disable l0s if link down or L2CB */
		if (dp->mii_state != MII_STATE_LINKUP ||
		    lp->chip == CHIP_L2C_B) {
			val &= ~PM_CTRL_ASPM_L0S_EN;
		}
		break;

	default: /* L1C */
		val &= ~PM_CTRL_L1_ENTRY_TIMER;

		if (dp->mii_state == MII_STATE_LINKUP) {
			val |= PM_CTRL_SERDES_L1_EN
			    | PM_CTRL_SERDES_PLL_L1_EN
			    | PM_CTRL_SERDES_BUFS_RX_L1_EN;

			val &= ~(PM_CTRL_SERDES_PD_EX_L1
			    | PM_CTRL_CLK_SWH_L1
			    | PM_CTRL_ASPM_L0S_EN
			    | PM_CTRL_ASPM_L1_EN);
		} else {
			/* link down */
			val |= PM_CTRL_CLK_SWH_L1;

			val &= ~(PM_CTRL_SERDES_L1_EN
			    | PM_CTRL_SERDES_PLL_L1_EN
			    | PM_CTRL_SERDES_BUFS_RX_L1_EN
			    | PM_CTRL_ASPM_L0S_EN);
		}
	}
#endif
	OUTL(dp, PM_CTRL, val);
}

static boolean_t
atge_wait_idle(struct gem_dev *dp, uint_t which)
{
	int	i;
	uint32_t	val;

	for (i = 0; (val = INL(dp, IDLE)) & which; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "!%s: %s: timeout, IDLE: %b",
			    dp->name, __func__, val, IDLE_BITS);

			return (B_FALSE);
		}
		delay(drv_usectohz(1000));
	}
	return (B_TRUE);
}

static int
atge_reset_chip(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	uint32_t	mstc;
	int		wait;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called, time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* initialize soft copy of mac address registers */
	bzero(lp->mac_addr, ETHERADDRL);

	if (!lp->initialized) {
		switch (lp->chip) {
		case CHIP_L1:
			/* initialize PCI-E */
			OUTL(dp, 0x12fc, 0x6500);

			/* change pcie flow control mode */
			OUTL(dp, 0x1008, 0x8000 | INL(dp, 0x1008));
			break;

		case CHIP_L2:
			/* initialize PCI-E */
			OUTL(dp, 0x12fc, 0x6500);

			/* PCIE_DLL_TX_CTRL */
			OUTL(dp, 0x1104, 0x0568);
			break;

		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:

			(void)INL(dp, WOLCTL);
			OUTL(dp, WOLCTL, 0);

			OUTL(dp, 0x010c, INL(dp, 0x010c) & ~0x12000);
			/* LTSSM_ID_CTRL */
			OUTL(dp, 0x12fc, INL(dp, 0x12fc) & ~0x1000);

			/* pclk sel could switch to 25M */
			mstc = INL(dp, MSTC) & ~MSTC_CLK_SEL_DIS_1c;
			OUTL(dp, MSTC, mstc);

			if (lp->chip == CHIP_L1C || lp->chip == CHIP_L2C) {
				OUTL(dp, PCIE_PHYMISC,
				    INL(dp, PCIE_PHYMISC) | 0x0004);
			} else {
				if ((mstc & MSTC_WAKEN_25M_1c) == 0) {
					OUTL(dp, MSTC,
					    mstc | MSTC_WAKEN_25M_1c);
				}
			}

			if (lp->chip == CHIP_L2C_B && lp->revid == 0xc0) {
				val = INL(dp, PCIE_PHYMISC2);
				val &= ~(PCIE_PHYMISC2_SERDES_TH
				    | PCIE_PHYMISC2_SERDES_CDR);
				val |= 3U << PCIE_PHYMISC2_SERDES_TH_SHIFT
				    | 3U << PCIE_PHYMISC2_SERDES_CDR_SHIFT;
				OUTL(dp, PCIE_PHYMISC2, val);

				/* extend L1 sync timer */
				OUTL(dp, LINKCTRL,
				    INL(dp, LINKCTRL) | LINKCTRL_EXT_SYNC);
			}

			/* disable l0s_l1 */
			atge_set_aspm_1c(dp, 0);
			break;

		default:
			/* L1E/L2E_A/L2E_B */
			/* change pcie flow control mode */
			OUTL(dp, 0x1008, 0x8000 | INL(dp, 0x1008));
			break;
		}
		lp->initialized = B_TRUE;
	}

	/* before clearing interrupts, save ISR to avoid bogus interrupts */
	OUTL(dp, IMR, 0);
	OUTL(dp, ISR, lp->isr_disable);	/* for L1C */
	FLSHL(dp, IMR);
	lp->isr_pended |= INL(dp, ISR);

	wait = 1000;
	switch (lp->chip) {
	case CHIP_L1:
	case CHIP_L2:
		OUTL(dp, MSTC, MSTC_SRST);
		break;

	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		mstc = INL(dp, MSTC) | MSTC_OOB_DIS_OFF_1c; /* ok */
		OUTL(dp, MSTC, mstc | MSTC_SRST); /* ok */
		wait = 10*1000;
		break;

	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
		OUTL(dp, MSTC, MSTC_LEDMODE_1e | MSTC_SRST);
		break;
	}
	FLSHL(dp, MSTC);

	delay(drv_usectohz(wait));
#if 0
	for (i = 0; (val = INL(dp, IDLE)) & IDLE_ALL; i++) {
		if (i > 10) {
			cmn_err(CE_WARN, "!%s: %s: timeout, IDLE: %b",
			    dp->name, __func__, val, IDLE_BITS);

			return (GEM_FAILURE);
		}
		delay(drv_usectohz(1000));
	}
#else
	if (!atge_wait_idle(dp, IDLE_ALL)) {
		return (GEM_FAILURE);
	}
#endif

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		OUTL(dp, MSTC, mstc); /* ok */
		OUTL(dp, MACCTL, INL(dp, MACCTL) | MACCTL_SPEED_MODE_SW);
		break;
	}

	/* clear wake on lan state - is it required for every reset ? */
	(void) INL(dp, WOLCTL);
	OUTL(dp, WOLCTL, 0);

	/* disable interrupt again */
	OUTL(dp, IMR, 0);
	FLSHL(dp, IMR);

	/* clock switch setting for L2C_B, L2CB2 and L1D_2 */
	switch (lp->chip) {
	case CHIP_L2C_B:
		val = INL(dp, SERDES_LOCK);
		val &= ~SERDES_MAC_CLK_SLOWDOWN;
		val &= ~SERDES_PYH_CLK_SLOWDOWN;
		OUTL(dp, SERDES_LOCK, val);		/* ok */
		break;

	case CHIP_L2C_B2:
	case CHIP_L1D_2_0:
		val = INL(dp, SERDES_LOCK);
		val |= SERDES_MAC_CLK_SLOWDOWN;
		val |= SERDES_PYH_CLK_SLOWDOWN;
		OUTL(dp, SERDES_LOCK, val);		/* ok */
		break;
	}

	return (GEM_SUCCESS);
}

static int
atge_init_chip_1(struct gem_dev *dp)
{
	int		i;
	int		ret;
	uint32_t	val;
	uint32_t	hiwat;
	uint32_t	lowat;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* clear bogus interrupt status */
	OUTL(dp, ISR, lp->isr_disable | lp->isr_clear_all);

	/* mac control: setup later */
	lp->mac_ctl = MACCTL_CRCEN | MACCTL_AUTOPAD
	    | 7U << MACCTL_PRMLEN_SHIFT | MACCTL_FD | MACCTL_SPEED_10_100
	    | MACCTL_TXFC | MACCTL_RXFC;

	/* setup various base address */
	OUTL(dp, DESCBASE_1, 0);
	OUTL(dp, RSDADDR_1, dp->rx_ring_dma);
	OUTL(dp, TBDADDR_1, dp->io_area_dma + lp->tbd_offset);
	OUTL(dp, ISMADDR_1, dp->io_area_dma + lp->ism_offset);
	OUTL(dp, SMADDR_1, dp->io_area_dma + lp->sm_offset);
	OUTL(dp, RBDADDR_1, dp->io_area_dma + lp->rbd_offset);

	/* the number of descriptors */
	OUTL(dp, RBD_RSD_CNT_1,
	    dp->gc.gc_rx_ring_size << 16 | dp->gc.gc_rx_ring_size);
	OUTL(dp, TBD_CNT_1, dp->gc.gc_tx_ring_size);

	/* load the descriptor configuration */
	OUTL(dp, DESCEN_1, 1);

	/* mailbox */
	lp->tx_buffer_tail = 0;
	lp->rx_status_head = 0;
	lp->rx_status_tail = 0;
	lp->rx_buffer_tail = 0;
	UPDATE_MBOX_1(dp);
	DPRINTF(0, (CE_CONT, "!%s: %s: MBOX:0x%x",
	    dp->name, __func__, INL(dp, MBOX_1)));

	/* inter packet gap and inter frame gap */
	OUTL(dp, GAPC, GAPC_STD);
	DPRINTF(0, (CE_CONT, "!%s: %s: GAPC:0x%x",
	    dp->name, __func__, INL(dp, GAPC)));

	/* half duplex control register */
	OUTL(dp, HDCTL, HDCTL_STD);
	DPRINTF(0, (CE_CONT, "!%s: %s: HDCTL:0x%x",
	    dp->name, __func__, INL(dp, HDCTL)));

	/* Interrupt Delay */
#ifdef CONFIG_ADAPTIVE_COALESCE
	val = 1;	/* 2uS */
#else
	val = 200/2;	/* original: 200uS */
#endif
	OUTW(dp, INTRDELAY_INIT, val);
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY_INIT:0x%x",
	    dp->name, __func__, INW(dp, INTRDELAY_INIT)));

	OUTL(dp, MSTC, MSTC_ITIMER);

	/* Interrupt Clear Timer */
	OUTW(dp, INTRCLR_TIMER, 100*1000/2); /* original: 100mS in 2uS */

	DPRINTF(0, (CE_CONT, "!%s: %s: INTRCLR_TIMER:0x%x",
	    dp->name, __func__, INW(dp, INTRCLR_TIMER)));

	/* MTU */
	OUTL(dp, MTU,
	    sizeof (struct ether_header) + dp->mtu + ETHERFCSL + VTAG_SIZE);
	DPRINTF(0, (CE_CONT, "!%s: %s: MTU: 0x%x",
	    dp->name, __func__, INL(dp, MTU)));

	/* Rx jumbo packet configuration */
	OUTL(dp, RXJCFG_1,
	    16U << RXJCFG_RSDTIMER_SHIFT
	    | 1U << RXJCFG_LKAH_SHIFT
	    | (dp->rx_buf_len / 8) << RXJCFG_THRESH_SHIFT);

	DPRINTF(0, (CE_CONT, "!%s: %s: RXJCFG: 0x%x",
	    dp->name, __func__, INL(dp, RXJCFG_1)));

	/* descriptor flow control */
	val = (INL(dp, MSTC) & (MSTC_DID | MSTC_REV)) >> MSTC_REV_SHIFT;
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: device id:%x", dp->name, __func__, val));
	if (val == 0x8001 || val == 0x9001 ||
	    val == 0x9002 || val == 0x9003) {
		/* setup RBD flow control using RXFFC register */
		lowat = (dp->gc.gc_rx_ring_size * 7) / 8;
		hiwat = max(dp->gc.gc_rx_ring_size / 16, 2);
		OUTL(dp, RXFFC_1, hiwat << RXFFC_HIWAT_SHIFT | lowat);

		/* setup RSD flow control */
		OUTL(dp, RSDFC_1, lowat << RSDFC_LOWAT_SHIFT | hiwat);
	} else {
		/* setup RXF flow control */
		val = INL(dp, RAM_RXF_SIZE_1);
		DPRINTF(0, (CE_CONT, "!%s: %s: ram rxf size :%d",
		    dp->name, __func__, val));
		lowat = max(val / 16, 192);	/* was min 192 */
		hiwat = max((val * 7) / 8, lowat + 16);
		OUTL(dp, RXFFC_1, hiwat << RXFFC_HIWAT_SHIFT | lowat);

		/* setup RSD flow control */
		val = INL(dp, RAM_RSD_SIZE_1);
		DPRINTF(0, (CE_CONT, "!%s: %s: ram rsd size :%d",
		    dp->name, __func__, val));
		lowat = max(val / 8, 2);	/* was min 2 */
		hiwat = max((val * 7) / 8, lowat + 3);
		OUTL(dp, RSDFC_1, lowat << RSDFC_LOWAT_SHIFT | hiwat);
	}

	/* TXQCTL: */
	OUTL(dp, TXQCTL_1,
	    0x100U << TXQCTL_TXF_BURST_NUM_SHIFT
	    | 16U << TXQCTL_TBD_FETCH_TH_SHIFT
	    | TXQCTL_EN
	    | TXQCTL_ENH_MODE
	    | 4U << TXQCTL_TBD_BURST_NUM_SHIFT);
	DPRINTF(0, (CE_CONT, "!%s: %s: TXQCTL:0x%x",
	    dp->name, __func__, INL(dp, TXQCTL_1)));

	/* tx jumbo packet cfg */
	OUTL(dp, TXJCFG_1,
	    1U << TXJCFG_TBDIPG_SHIFT
	    | (dp->rx_buf_len / 8) << TXJCFG_THRESH_SHIFT);
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: TXJCFG:0x%x",
	    dp->name, __func__, INL(dp, TXJCFG_1)));

	/* RXQCTL: */
	OUTL(dp, RXQCTL_1,
	    RXQCTL_EN
	    | RXQCTL_CUT_THRU_EN
	    | 1U << RXQCTL_RBD_PREF_MIN_IPG_SHIFT
	    | 8U << RXQCTL_RSD_BURST_THRESH_SHIFT
	    | 8U << RXQCTL_RBD_BURST_NUM_SHIFT);
	DPRINTF(0, (CE_CONT, "!%s: %s: RXQCTL: 0x%x",
	    dp->name, __func__, INL(dp, RXQCTL_1)));

	/* dma control */
	OUTL(dp, DMAC_1,
	    DMAC_DMAW_EN_1 | DMAC_DMAR_EN_1
	    | MAXDMA_256 << DMAC_DMAW_MAXDMA_SHIFT
	    | MAXDMA_256 << DMAC_DMAR_MAXDMA_SHIFT
	    | DMAC_DMAR_ENH_ORDER);	/* was OUT_ORDER */
	DPRINTF(0, (CE_CONT, "!%s: %s: DMAC:0x%x",
	    dp->name, __func__, INL(dp, DMAC_1)));

	/* configure ISM / SM */
#ifdef CONFIG_ADAPTIVE_COALESCE
	OUTL(dp, ISMTHRESH_1,
	    dp->gc.gc_tx_ring_size << ISMTHRESH_TBD_SHIFT
	    | (max(dp->poll_pkt_delay * 2, 1)));
#else
	OUTL(dp, ISMTHRESH_1,
	    max(dp->gc.gc_tx_ring_size, 4) << ISMTHRESH_TBD_SHIFT | 4);
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: ISMTHRESH:0x%x",
	    dp->name, __func__, INL(dp, ISMTHRESH_1)));

	/* ISMDELAY: original ism_tx_timer 2uS, ism_rx_timer 2uS */
#ifdef CONFIG_ADAPTIVE_COALESCE
	OUTL(dp, ISMDELAY_1,
	    1U << ISMDELAY_TBD_SHIFT | (max(dp->poll_pkt_delay, 1) * 6));
#else
	OUTL(dp, ISMDELAY_1, 1U << ISMDELAY_TBD_SHIFT | 1);
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: ISMDELAY:0x%x",
	    dp->name, __func__, INL(dp, ISMDELAY_1)));

	OUTL(dp, SMDELAY_1, 200*1000/2);	/* 200mS */

	/* enable ISM / SM */
	OUTL(dp, ISMCTL_1, ISMCTL_ISM_EN | ISMCTL_SM_EN);

	if (INL(dp, ISR) & ISR_PCIE_ERR_1) {
		/* pci-e phy linkdown */
		ret = GEM_FAILURE;
	} else {
		ret = GEM_SUCCESS;
	}

	/* clear all interrupt status */
	OUTL(dp, ISR, lp->isr_clear_all);

	/* clear statistics area */
	bzero(dp->io_area + lp->sm_offset, sizeof (uint32_t) * STAT_MAX);

	return (ret);
}

static int
atge_init_chip_2(struct gem_dev *dp)
{
	int		i;
	int		ret;
	uint32_t	val;
	uint32_t	hiwat;
	uint32_t	lowat;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* clear bogus interrupt status */
	OUTL(dp, ISR, lp->isr_disable | lp->isr_clear_all);

	/* mac control: setup later */
	lp->mac_ctl = MACCTL_CRCEN | MACCTL_AUTOPAD
	    | 7U << MACCTL_PRMLEN_SHIFT | MACCTL_FD | MACCTL_SPEED_10_100
	    | MACCTL_TXFC | MACCTL_RXFC;

	/* setup various base address */
	OUTL(dp, DESCBASE_2, 0);
	OUTL(dp, TBDADDR_2, dp->io_area_dma + lp->tbd_offset);
	OUTL(dp, TSDADDR_2, dp->tx_ring_dma);
	OUTL(dp, RBDADDR_2, dp->io_area_dma + lp->rbd_offset);

	/* number of descriptors */
	OUTW(dp, TBDLEN_2, lp->tbd_size/4);
	OUTW(dp, TSDCNT_2, dp->gc.gc_tx_ring_size);
	OUTW(dp, RBDCNT_2, dp->gc.gc_rx_ring_size);

	/* interpacket gap and interframe gap */
	OUTL(dp, GAPC, GAPC_STD);
	DPRINTF(0, (CE_CONT, "!%s: %s: GAPC:0x%x",
	    dp->name, __func__, INL(dp, GAPC)));

	/* half duplex control register */
	OUTL(dp, HDCTL, HDCTL_STD);
	DPRINTF(0, (CE_CONT, "!%s: %s: HDCTL:0x%x",
	    dp->name, __func__, INL(dp, HDCTL)));

	/* Interrupt Delay */
#ifdef CONFIG_ADAPTIVE_COALESCE
	val = 1;	/* 2uS */
	lp->last_poll_interval = 0;
#else
	val = 200/2;	/* original: 200uS */
#endif
	OUTW(dp, INTRDELAY_INIT, val);
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY_INIT:0x%x",
	    dp->name, __func__, INW(dp, INTRDELAY_INIT)));

	OUTL(dp, MSTC, MSTC_ITIMER);

	/* Interrupt Clear Timer */
	OUTW(dp, INTRCLR_TIMER, 100*1000/2);	/* original: 100mS in 2uS */
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRCLR_TIMER:0x%x",
	    dp->name, __func__, INW(dp, INTRCLR_TIMER)));

	/* MTU */
	OUTL(dp, MTU,
	    sizeof (struct ether_header) + dp->mtu + ETHERFCSL + VTAG_SIZE);
	DPRINTF(0, (CE_CONT, "!%s: %s: MTU: 0x%x",
	    dp->name, __func__, INL(dp, MTU)));

	OUTL(dp, TXTHRESH_2, 0x177);

	/* flow control */
	OUTW(dp, RXHIWAT_2, dp->gc.gc_rx_ring_size * 7 / 8);
	OUTW(dp, RXLOWAT_2, min(dp->gc.gc_rx_ring_size / 12, 16/8));

	/* mailbox */
	lp->tx_buffer_tail = 0;
	lp->rx_buffer_head = 0;
	OUTW(dp, TBDTAIL_2, 0);
	OUTW(dp, RBDTAIL_2, 0);

	/* dma control */
	OUTB(dp, DMAR_2, 1);
	OUTB(dp, DMAW_2, 1);

	if (INL(dp, ISR) & ISR_PCIE_ERR_2) {
		/* pci-e phy linkdown */
		ret = GEM_FAILURE;
	} else {
		ret = GEM_SUCCESS;
	}

	/* clear all interrupt status */
	OUTL(dp, ISR, lp->isr_clear_all);

	return (ret);
}

static int
atge_init_chip_1e(struct gem_dev *dp)
{
	int		i;
	int		q;
	int		ret;
	uint32_t	val;
	uint32_t	imt;
	uint32_t	hiwat;
	uint32_t	lowat;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* clear bogus interrupt status */
	OUTL(dp, ISR, lp->isr_disable | lp->isr_clear_all);

	/* mac control: do later */
	lp->mac_ctl = MACCTL_CRCEN | MACCTL_AUTOPAD
	    | 7U << MACCTL_PRMLEN_SHIFT | MACCTL_FD | MACCTL_SPEED_10_100
	    | MACCTL_TXFC | MACCTL_RXFC;

	/* setup various base address */
	OUTL(dp, DESCBASE_1e, 0);
	OUTL(dp, TBDADDR_1e, dp->io_area_dma + lp->tbd_offset);
	OUTL(dp, TBD_CNT_1e, dp->gc.gc_tx_ring_size);
	OUTL(dp, TBD_TAIL_PTR_1e, dp->io_area_dma + lp->tbd_tail_offset);

	OUTL(dp, RXF0_RING_BASE_1e + 0, dp->io_area_dma + lp->rxf_offset[0][0]);
	OUTL(dp, RXF0_RING_BASE_1e + 4, dp->io_area_dma + lp->rxf_offset[0][1]);
	OUTL(dp, RXF_RING_MB_1e + 0, dp->io_area_dma + lp->rxf_mb_offset[0][0]);
	OUTL(dp, RXF_RING_MB_1e + 4, dp->io_area_dma + lp->rxf_mb_offset[0][1]);
	OUTB(dp, RXF_RING_VALID_1e + 0, 1);
	OUTB(dp, RXF_RING_VALID_1e + 1, 1);
	OUTB(dp, RXF_RING_VALID_1e + 2, 0);
	OUTB(dp, RXF_RING_VALID_1e + 3, 0);
	OUTB(dp, RXF_RING_VALID_1e + 4, 0);
	OUTB(dp, RXF_RING_VALID_1e + 5, 0);
	OUTB(dp, RXF_RING_VALID_1e + 6, 0);
	OUTB(dp, RXF_RING_VALID_1e + 7, 0);

	OUTL(dp, RXF_RING_HIWAT_1e, lp->rxf_buf_hiwat);

	/* load the descriptor configuration */
	OUTL(dp, DESCEN_1e, 1);

	/* Interrupt Delay */
#ifdef CONFIG_ADAPTIVE_COALESCE
	imt = 1;	/* 2uS */
#else
	imt = 200/2;	/* original: 200uS */
#endif
	OUTW(dp, INTRDELAY_INIT, imt);	/* worked only for tx_pkt */
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY_INIT:0x%x",
	    dp->name, __func__, INW(dp, INTRDELAY_INIT)));
	OUTW(dp, INTRDELAY2_INIT, imt);	/* workded only for rx_pkt */
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY2_INIT:0x%x",
	    dp->name, __func__, INW(dp, INTRDELAY2_INIT)));

	OUTL(dp, MSTC,
	    MSTC_LEDMODE_1e | MSTC_ITIMER_TX_1e | MSTC_ITIMER_RX_1e);
	DPRINTF(0, (CE_CONT, "!%s: %s: MSTC:0x%x",
	    dp->name, __func__, INL(dp, MSTC)));

	/* inter packet gap and inter frame gap */
#ifdef NEVER
	OUTL(dp, GAPC, GAPC_STD);	/* use default */
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: GAPC:0x%x",
	    dp->name, __func__, INL(dp, GAPC)));

	/* half duplex control register */
	OUTL(dp, HDCTL, HDCTL_STD);
	DPRINTF(0, (CE_CONT, "!%s: %s: HDCTL:0x%x",
	    dp->name, __func__, INL(dp, HDCTL)));
#ifdef CONFIG_ADAPTIVE_COALESCE
	OUTW(dp, RX_COALSC_PKT_1e, 1);	/* didn't work */
	OUTW(dp, RX_COALSC_TO_1e, 1);	/* didn't work */
	OUTW(dp, TX_COALSC_PKT_1e, dp->gc.gc_tx_ring_size/8); /* in descs */
	OUTW(dp, TX_COALSC_TO_1e, 0xffff);	/* no timeout */
#else
	/* original parameters */
	OUTW(dp, RX_COALSC_PKT_1e, max(1, dp->poll_pkt_delay));
	OUTW(dp, RX_COALSC_TO_1e, 4);
	OUTW(dp, TX_COALSC_PKT_1e, dp->gc.gc_tx_ring_size/16);
	OUTW(dp, TX_COALSC_TO_1e, imt*4/3);
#endif
	/* Interrupt Clear Timer */
	OUTW(dp, INTRCLR_TIMER, 100*1000/2); /* original: 100mS in 2uS */

	DPRINTF(0, (CE_CONT, "!%s: %s: INTRCLR_TIMER:0x%x",
	    dp->name, __func__, INW(dp, INTRCLR_TIMER)));

	/* MTU */
	OUTL(dp, MTU,
	    sizeof (struct ether_header) + dp->mtu + ETHERFCSL + VTAG_SIZE);
	DPRINTF(0, (CE_CONT, "!%s: %s: MTU: 0x%x",
	    dp->name, __func__, INL(dp, MTU)));

	if (lp->chip != CHIP_L2E_B) {
		val = dp->mtu + sizeof (struct ether_header) + ETHERFCSL + 4;
		if (dp->mtu <= 1500) {
			/* store and forward */
			/* EMPTY */
		} else if (dp->mtu < 6*1024) {
			val = val * 2 / 3;
		} else {
			val = val / 2;
		}
		OUTW(dp, TXJCFG_1e, ROUNDUP(val, 8) / 8);
	}

	/* TXQCTL: */
	if (lp->chip != CHIP_L2E_B) {
		OUTW(dp, TXQCTL_1e + 2, 128U << lp->maxrequest);
	}

	OUTW(dp, TXQCTL_1e,
	    TXQCTL_EN
	    | TXQCTL_ENH_MODE
	    | 5U << TXQCTL_TBD_BURST_NUM_SHIFT);
	DPRINTF(0, (CE_CONT, "!%s: %s: TXQCTL:0x%x",
	    dp->name, __func__, INL(dp, TXQCTL_1e)));

	/*
	 * configure RXQ
	 */
	if (lp->chip != CHIP_L2E_B) {
		/* Rx jumbo packet configuration */
		OUTW(dp, RXJCFG_1e,
		    1U << RXJCFG_LKAH_SHIFT | (dp->rx_buf_len / 8));

		DPRINTF(0, (CE_CONT, "!%s: %s: RXJCFG: 0x%x",
		    dp->name, __func__, INL(dp, RXJCFG_1e)));

		/* flow control */
		val = INL(dp, RAM_RXF_SIZE_1e);
		hiwat = val * 4 / 5;
		lowat = val / 5;
		OUTL(dp, RXFFC_1e, hiwat << RXFFC_HIWAT_SHIFT | lowat);
	}
	/* RSS: */
	OUTL(dp, IDT_TABLE0_1e, 0);
	OUTL(dp, BASE_CPU_NUM_1e, 0);

	/* RXQCTL: */
	OUTL(dp, RXQCTL_1e,
	    RXQCTL_EN
	    | RXQCTL_CUT_THRU_EN
	    | RXQCTL_IPV6_CKSUM_EN
	    | RXQCTL_PBA_ALIGN_32);
	DPRINTF(0, (CE_CONT, "!%s: %s: RXQCTL: 0x%x",
	    dp->name, __func__, INL(dp, RXQCTL_1e)));

	/* DMA control */
	OUTL(dp, DMAC_1e,
	    DMAC_RXCMB_1e /* | DMAC_TXCMB_1e */
	    | 4U << DMAC_DMAW_DLY_CNT_SHIFT
	    | 15U << DMAC_DMAR_DLY_CNT_SHIFT
	    | DMAC_DMAR_REQ_PRI
	    | lp->maxpayload << DMAC_DMAW_MAXDMA_SHIFT
	    | lp->maxrequest << DMAC_DMAR_MAXDMA_SHIFT
	    | DMAC_DMAR_OUT_ORDER);
	DPRINTF(0, (CE_CONT, "!%s: %s: DMAC:0x%x",
	    dp->name, __func__, INL(dp, DMAC_1e)));

	/* statistics messages */
	OUTL(dp, SM_TIMER_1e, 200000);	/* 400mS */

	if (INL(dp, ISR) & ISR_PCIE_ERR_1e) {
		/* pci-e phy linkdown */
		ret = GEM_FAILURE;
	} else {
		ret = GEM_SUCCESS;
	}

	/* clear all interrupt status */
	OUTL(dp, ISR, lp->isr_clear_all);

	for (q = 0; q < 4; q++) {
		lp->rxf[q].rf_head = 0;
		lp->rxf[q].rf_tail = 0;
		lp->rxf[q].rf_curr_ring = 0;
		lp->rxf[q].rf_seqnum = 0;
		for (i = 0; i < 2; i++) {
			*((uint32_t *)(dp->io_area +
			    lp->rxf_mb_offset[q][i])) = 0;
		}
	}

	/* clear statistics area */
	bzero(dp->io_area + lp->sm_offset, sizeof (uint32_t) * STAT_MAX);
	atge_ioarea_dma_sync(dp, lp->sm_offset,
	    (size_t)(sizeof (uint32_t) * STAT_MAX), DDI_DMA_SYNC_FORDEV);

	return (ret);
}

static int
atge_init_chip_1c(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	uint32_t	rx_delay;
	uint32_t	tx_delay;
	uint32_t	hiwat;
	uint32_t	lowat;
	uint64_t	addr;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* clear bogus interrupt status */
	OUTL(dp, ISR, lp->isr_disable | lp->isr_clear_all);	/* ok */

	/* clear wol status */
	OUTL(dp, WOLCTL, 0);	/* ok */

	/* clock gating */
	OUTL(dp, CLK_GATING_1c, 0);	/* ok */

	/* interrupt clear timer */
	OUTL(dp, INTRCLR_TIMER_lc, 0); /* original: 100mS in 2uS */ /* ok */
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRCLR_TIMER:0x%x",
	    dp->name, __func__, INL(dp, INTRCLR_TIMER_lc)));

	/*
	 * setup various base address
	 */
	/* the base address of tx descriptor array */
	ASSERT(((dp->io_area_dma + lp->tbd_offset) >> 32) == 0);
	OUTL(dp, TX_BASE_HI_1c, 0);	/* ok */
	OUTL(dp, NTPD_HEAD_LO_1c, dp->io_area_dma + lp->tbd_offset);	/* ok */
	OUTL(dp, HTPD_HEAD_LO_1c, 0);	/* ok */

	OUTL(dp, TPD_RING_SIZE_1c, dp->gc.gc_tx_ring_size);	/* ok */

	/* the base address of rx descriptor array */
	ASSERT((dp->io_area_dma + lp->rbd_offset) >> 32 == 0);
	OUTL(dp, RX_BASE_HI_1c, 0);	/* ok */
	OUTL(dp, RFD_HEAD_LO_1c + 0*4, dp->io_area_dma + lp->rbd_offset); /* ok */
#if 0
	OUTL(dp, RFD_HEAD_LO_1c + 1*4, 0);
	OUTL(dp, RFD_HEAD_LO_1c + 2*4, 0);
	OUTL(dp, RFD_HEAD_LO_1c + 3*4, 0);
#endif
	OUTL(dp, RFD_RING_SIZE_1c, dp->gc.gc_rx_ring_size);	/* ok */

	OUTL(dp, RX_BUF_SIZE_1c, dp->rx_buf_len);	/* ok */
	DPRINTF(0, (CE_CONT, "!%s: %s: RX_BUF_SIZE: 0x%x",
	    dp->name, __func__, INL(dp, RX_BUF_SIZE_1c)));

	OUTL(dp, RRD_HEAD_LO_1c + 0*4, dp->rx_ring_dma);	/* ok */
#if 0
	OUTL(dp, RRD_HEAD_LO_1c + 1*4, 0);
	OUTL(dp, RRD_HEAD_LO_1c + 2*4, 0);
	OUTL(dp, RRD_HEAD_LO_1c + 3*4, 0);
#endif
	OUTL(dp, RRD_RING_SIZE_1c, dp->gc.gc_rx_ring_size);
#if 0
	/* the base address of interrupt massage */
	ASSERT((dp->io_area_dma + lp->ism_offset) >> 32 == 0);
	OUTL(dp, CMB_BASE_LO_1c, dp->io_area_dma + lp->ism_offset);

	/* the base address of statistics area */
	addr = dp->io_area_dma + lp->sm_offset;
	OUTL(dp, SMB_BASE_HI_1c, addr >> 32);
	OUTL(dp, SMB_BASE_LO_1c, addr);
#endif
	/* sram setting for L2C_B */
	if (lp->chip == CHIP_L2C_B) {
		OUTL(dp, SRAM_RXF_LEN_1c, 0x000002a0U);		/* ok */
		OUTL(dp, SRAM_TXF_LEN_1c, 0x00000100U);		/* ok */
		OUTL(dp, SRAM_RXF_ADDR_1c, 0x029f0000U);	/* ok */
		OUTL(dp, SRAM_RFD0_INFO_1c, 0x02bf02a0U);	/* ok */
		OUTL(dp, SRAM_TXF_ADDR_1c, 0x03bf02c0U);	/* ok */
		OUTL(dp, SRAM_TRD_ADDR_1c, 0x03df03c0U);	/* ok */

		/* TX watermark, to enter l1 state. */
		OUTL(dp, TXF_WATER_MARK_1c, 0);		/* ok */

		/* RXD threshold */
		OUTL(dp, RXD_DMAC_1c, 0);		/* ok */
	}

	/* load the descriptor configuration */
	OUTL(dp, DESCEN_1c, 1);		/* ok */

#if 0
	/* clock switch setting for L2C_B, L2CB2 and L1D_2 */
	switch (lp->chip) {
	case CHIP_L2C_B:
		val = INL(dp, SERDES_LOCK);
		val &= ~SERDES_MAC_CLK_SLOWDOWN;
		val &= ~SERDES_PYH_CLK_SLOWDOWN;
		OUTL(dp, SERDES_LOCK, val);		/* ok */
		break;

	case CHIP_L2C_B2:
	case CHIP_L1D_2_0:
		val = INL(dp, SERDES_LOCK);
		val |= SERDES_MAC_CLK_SLOWDOWN;
		val |= SERDES_PYH_CLK_SLOWDOWN;
		OUTL(dp, SERDES_LOCK, val);		/* ok */
		break;
	}
#endif
	/* Interrupt Delay */
	val = 0;
#ifdef CONFIG_INTR_RDCLR
	/* XXX - don't use CONFIG_INTR_RDCLR, it hangs the system */
	val |= MSTC_INT_RDCLR_1c;
	lp->intr_rdclr = B_TRUE;
#else
	lp->intr_rdclr = B_FALSE;
#endif
#ifdef CONFIG_ADAPTIVE_COALESCE
	rx_delay = 1;		/* original: 400uS */
	tx_delay = 1;		/* original: 2mS */
#else
	rx_delay = 400/2;	/* original: 400uS */
	tx_delay = 2000/2;	/* original: 2mS */
#endif
	val |= MSTC_TX_ITIMER_EN_1c | MSTC_RX_ITIMER_EN_1c;

	OUTL(dp, INTRDELAY_INIT, rx_delay << 16 | tx_delay);
	DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY_INIT:0x%x",
	    dp->name, __func__, INL(dp, INTRDELAY_INIT)));

	/* master control */
	OUTL(dp, MSTC, val);
	DPRINTF(0, (CE_CONT, "!%s: %s: MSTC:0x%x",
	    dp->name, __func__, INL(dp, MSTC)));


	/* MTU */
	OUTL(dp, MTU,
	    sizeof (struct ether_header) + dp->mtu + ETHERFCSL + VTAG_SIZE);
	DPRINTF(0, (CE_CONT, "!%s: %s: MTU: 0x%x",
	    dp->name, __func__, INL(dp, MTU)));

	/*
	 * configure tx
	 */
	OUTL(dp, TSO_THRESH_1c, (7*1024) >> 3);

	/* TXQCTL: */
	switch (lp->chip) {
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
		val = (128U << lp->maxrequest) / 2;
		break;

	default:
		val = 128U << lp->maxrequest;
		break;
	}
	OUTL(dp, TXQCTL_1c,
	    val << TXQCTL_TXF_BURST_NUM_SHIFT
#if 0
	    | TXQCTL_EN
#endif
	    | TXQCTL_ENH_MODE
	    | 5U << TXQCTL_NUM_TPD_BURST_SHIFT);
	DPRINTF(0, (CE_CONT, "!%s: %s: TXQCTL:0x%x",
	    dp->name, __func__, INL(dp, TXQCTL_1c)));


	/*
	 * configure rx
	 */
	/* RXQCTL: */
	OUTL(dp, RXQCTL_1c,
	    8U << RXQCTL_RFD_BURST_NUM_SHIFT
#if 0
	    | RXQCTL_EN
#endif
	    | RXQCTL_IPV6_CKSUM_EN);
	DPRINTF(0, (CE_CONT, "!%s: %s: RXQCTL: 0x%x",
	    dp->name, __func__, INL(dp, RXQCTL_1c)));

#ifdef NEVER
	/* RSS */
	OUTL(dp, IDT_TABLE0_1c, 0xe4e4e4e4);
	OUTL(dp, BASE_CPU_NUM_1c, 0);
#endif
	/* RXJCFG */
	DPRINTF(0, (CE_CONT, "!%s: %s: RXJCFG: 0x%x",
	    dp->name, __func__, INL(dp, RXJCFG_1e)));

	/* RXFFC */
	DPRINTF(0, (CE_CONT, "!%s: %s: RXFFC: 0x%x",
	    dp->name, __func__, INL(dp, RXFFC_1e)));

	/* mac control: do later */
	lp->mac_ctl = INL(dp, MACCTL)
	    | MACCTL_CRCEN | MACCTL_AUTOPAD
	    | 7U << MACCTL_PRMLEN_SHIFT | MACCTL_FD | MACCTL_SPEED_10_100
	    | MACCTL_TXFC | MACCTL_RXFC | MACCTL_SINGLE_PAUSE
	    | MACCTL_HASH_ALG_CRC32;

	/* mailbox */
	lp->tx_buffer_tail = 0;
#ifdef NEVER
	lp->rx_status_head = 0;
	lp->rx_status_tail = 0;
	lp->rx_buffer_tail = 0;
#endif
#ifdef NEVER
	/* HDS */
	OUTL(dp, HDS_CTL_1c, 0);
	DPRINTF(0, (CE_CONT, "!%s: %s: HDS_CTL: 0x%x",
	    dp->name, __func__, INL(dp, HDS_CTL_1c)));
#endif
	/* GAPC */
	DPRINTF(0, (CE_CONT, "!%s: %s: GAPC:0x%x",
	    dp->name, __func__, INL(dp, GAPC)));

	/* DMA control */
	OUTL(dp, DMAC_1c,
	    DMAC_SMB_DIS_1c
	    | 4U << DMAC_DMAW_DLY_CNT_SHIFT
	    | 15U << DMAC_DMAR_DLY_CNT_SHIFT
	    | DMAC_DMAR_REQ_PRI
#if 0
	    | lp->maxpayload << DMAC_DMAW_MAXDMA_SHIFT
#else
	    | MAXDMA_128 << DMAC_DMAW_MAXDMA_SHIFT
#endif
	    | lp->maxrequest << DMAC_DMAR_MAXDMA_SHIFT
	    | DMAC_DMAR_OUT_ORDER);				/* ok */
	DPRINTF(0, (CE_CONT, "!%s: %s: DMAC:0x%x",
	    dp->name, __func__, INL(dp, DMAC_1c)));

	/* clear all interrupt status */
	OUTL(dp, ISR, lp->isr_clear_all);

	/* clear statistics area */
	bzero(dp->io_area + lp->sm_offset, sizeof (uint32_t) * STAT_MAX);

	return (GEM_SUCCESS);
}

static uint_t
atge_mcast_hash(struct gem_dev *dp, uint8_t *addr)
{
	/* extract MSB 6bits of crc in big endian */
	return (gem_ether_crc_be(addr, ETHERADDRL) >> (32 - 6));
}

static int
atge_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint32_t	mode;
	uint64_t	mhash;
	uint8_t		*mac;
	static uint8_t	invalid_mac[ETHERADDRL] = {0, 0, 0, 0, 0, 0};
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT,
#ifdef GEM3
	    "!%s: %s: called at time:%d, state:%d, rxmode:%b, MACCTL:%08b",
#else
	    "!%s: %s: called at time:%d, active:%d, rxmode:%b, MACCTL:%08b",
#endif
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active,
	    dp->rxmode, RXMODE_BITS, INL(dp, MACCTL), MACCTL_BITS));

	mode = MACCTL_AB;
	mhash = 0ULL;
	mac = &dp->cur_addr.ether_addr_octet[0];

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* disable all filters */
		mode = 0;
		mac = invalid_mac;
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* XXX - no need to set MACCTL_AAM bit */
		mode |= MACCTL_PROMISC;
		mhash = ~0ULL;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		mode |= MACCTL_AAM;
	} else if (dp->mc_count > 0) {
		/* make a hash table */
		for (i = 0; i < dp->mc_count; i++) {
			mhash |= 1ULL << dp->mc_list[i].hash;
		}
	}

	if (bcmp(lp->mac_addr, mac, ETHERADDRL) != 0) {
		OUTL(dp, MACADDR,
		    mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
		OUTL(dp, MACADDR + 4,
		    mac[0] << 8 | mac[1]);
		bcopy(mac, lp->mac_addr, ETHERADDRL);
	}

	if ((mode & MACCTL_AAM) == 0) {
		/* need to set multicast hash table */
		OUTL(dp, MHASH, (uint32_t)mhash);
		OUTL(dp, MHASH + 4, (uint32_t)(mhash >> 32));
	}

	/* update rx filter mode */
	lp->mac_ctl = mode
	    | (lp->mac_ctl & ~(MACCTL_PROMISC | MACCTL_AAM | MACCTL_AB));
	OUTL(dp, MACCTL, lp->mac_ctl);

	return (GEM_SUCCESS);
}

static int
atge_set_media(struct gem_dev *dp)
{
	uint32_t	new;
	int	pkt_coalsc;
	int	pkt_time;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d, "
#ifdef GEM3
	    "state:%d, MACCTL:%b",
#else
	    "active:%d, MACCTL:%b",
#endif
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->mac_active, INL(dp, MACCTL), MACCTL_BITS));

	new = lp->mac_ctl &
	    ~(MACCTL_FD | MACCTL_SPEED | MACCTL_TXFC | MACCTL_RXFC);

	/* duplex mode */
	if (dp->full_duplex) {
		new |= MACCTL_FD;
	}

	/* speed */
	switch (dp->speed) {
	case GEM_SPD_1000:
		new |= MACCTL_SPEED_1000;
		pkt_time = 12;
		break;

	case GEM_SPD_100:
		new |= MACCTL_SPEED_10_100;
		pkt_time = 120;
		break;

	case GEM_SPD_10:
		new |= MACCTL_SPEED_10_100;
		/* no need to coalesce interrupts */
		pkt_time = 0;
		break;
	}

	/* setup flow control */
	switch (dp->flow_control) {
	case FLOW_CONTROL_SYMMETRIC:
		new |= MACCTL_TXFC | MACCTL_RXFC;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		new |= MACCTL_TXFC;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		new |= MACCTL_RXFC;
		break;
	}

	if (lp->mac_ctl != new) {
		lp->mac_ctl = new;
		OUTL(dp, MACCTL, new);
	}

	/* setup coalsce parameters depend on media speed */
	switch (lp->chip) {
	case CHIP_L1E:
#ifndef CONFIG_ADAPTIVE_COALESCE
		/* interrupt delay for rx_pkt */
		OUTW(dp, INTRDELAY2_INIT,
		    min(0xffff, pkt_time * dp->poll_pkt_delay / 2));
		DPRINTF(0, (CE_CONT, "!%s: %s: INTRDELAY2_INIT:0x%x",
		    dp->name, __func__, INW(dp, INTRDELAY2_INIT)));
#endif
		break;

	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		/* interrupt delay for rx_pkt and tx packets */
		new = min(0xffff, pkt_time * dp->poll_pkt_delay) / 2;
#ifndef CONFIG_ADAPTIVE_COALESCE
		OUTL(dp, INTRDELAY_INIT,
		    new << 16 | min(0xffff, new << 1));
#else
		OUTL(dp, INTRDELAY_INIT,
		    1U << 16 |
		    (min(0xffff, (pkt_time * dp->gc.gc_tx_buf_size / 8) / 2)));
#endif
		break;
	}

	return (GEM_SUCCESS);
}

static int
atge_start_chip(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		OUTL(dp, TXQCTL_1c, INL(dp, TXQCTL_1c) | TXQCTL_EN);
		OUTL(dp, RXQCTL_1c, INL(dp, RXQCTL_1c) | RXQCTL_EN);
		break;
	}

	/*
	 * Start mac
	 */
	/* we don't use MACCTL_RXVTAG */
	lp->mac_ctl |= MACCTL_TX | MACCTL_RX;
	OUTL(dp, MACCTL, lp->mac_ctl);

	/* enable interrupt */
	lp->imr = lp->isr_rx_err | lp->isr_rx_ok |
	    lp->isr_tx_err | lp->isr_tx_ok |
	    lp->isr_dma_err | lp->isr_gphy | lp->isr_sm | lp->isr_pcie_err;
	DPRINTF(0, (CE_CONT, "!%s: %s: imr:%b",
	    dp->name, __func__, lp->imr, lp->isr_bits));
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		OUTL(dp, IMR, lp->imr);
	}

	return (GEM_SUCCESS);
}

static int
atge_stop_chip(struct gem_dev *dp)
{
	uint8_t		*mac;
	struct atge_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		OUTL(dp, TXQCTL_1c, INL(dp, TXQCTL_1c) & ~TXQCTL_EN); /* ok */
		OUTL(dp, RXQCTL_1c, INL(dp, RXQCTL_1c) & ~RXQCTL_EN); /* ok */
		DPRINTF(0, (CE_CONT, "!%s: %s: tx_list:%d %d, reg:%d",
		    dp->name, __func__,
		    lp->tx_buffer_head, lp->tx_buffer_tail,
		    INL(dp, MB_PRIO_CONS_IDX) >> 16));

		atge_wait_idle(dp, IDLE_RXQ | IDLE_TXQ);

		lp->mac_ctl &= ~(MACCTL_TX | MACCTL_RX);
		OUTL(dp, MACCTL, lp->mac_ctl);

		atge_wait_idle(dp, IDLE_TXMAC | IDLE_RXMAC);
		break;
	}
	/* Disable interrupts by clearing the interrupt mask */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* XXX - don't clear imr to avoid bogus interrupts */
		OUTL(dp, IMR, 0);
	}

	/* restore factory mac address */
	mac = dp->dev_addr.ether_addr_octet;
	OUTL(dp, MACADDR,
	    mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
	OUTL(dp, MACADDR + 4,
	    mac[0] << 8 | mac[1]);
	bcopy(mac, lp->mac_addr, ETHERADDRL);

	(void) atge_reset_chip(dp);

	return (GEM_SUCCESS);
}

static boolean_t
atge_read_eeprom_1(struct gem_dev *dp, uint_t offset, uint32_t *valp)
{
	int	i;

	if (offset & 3) {
		/* offset should be aligned by 4 */
		return (B_FALSE);
	}

	OUTL(dp, VPDDATA, 0);
	OUTL(dp, VPDCAP, offset << VPDCAP_ADDR_SHIFT);

	delay(drv_usectohz(2000));

	for (i = 0; (INL(dp, VPDCAP) & VPDCAP_FLAG) == 0; i++) {
		if (i > 10) {
			/* timeout */
			DPRINTF(0, (CE_CONT, "!%s: %s: timeout",
			    dp->name, __func__));
			return (B_FALSE);
		}
		delay(drv_usectohz(2000));
	}

	*valp = INL(dp, VPDDATA);

	DPRINTF(4, (CE_CONT, "!%s: %s: offset:%x: %x",
	    dp->name, __func__, offset, *valp));

	return (B_TRUE);
}

static boolean_t
atge_read_eeprom_1c(struct gem_dev *dp, uint_t offset, uint32_t *valp)
{
	int	i;
	uint32_t	otp;
	uint32_t	val;
	int		ret;

	if (offset & 3) {
		/* offset should be aligned by 4 */
		return (B_FALSE);
	}

	otp = INL(dp, OTPCTL);
	if ((otp & OTPCTL_CLK_EN) == 0) {
		OUTL(dp, OTPCTL, otp | OTPCTL_CLK_EN);
	}

	OUTL(dp, EEPROM_DATA, 0);
	OUTL(dp, EEPROMCTL, offset << EEC_ADDR_SHIFT);

	for (i = 0; (INL(dp, EEPROMCTL) & EEC_RW) == 0; i++) {
		if (i > 10) {
			/* timeout */
			DPRINTF(0, (CE_CONT, "!%s: %s: timeout",
			    dp->name, __func__));
			ret = B_FALSE;
			goto x;
		}
		delay(drv_usectohz(100));
	}

	ret = B_TRUE;
	val = INL(dp, EEPROMCTL);
	*valp = (val & EEC_DATA_HI) << 16 | INL(dp, EEPROM_DATA) >> 16;

	DPRINTF(0, (CE_CONT, "!%s: %s: offset:%x: %x",
	    dp->name, __func__, offset, *valp));
x:
	if ((otp & OTPCTL_CLK_EN) == 0) {
		OUTL(dp, OTPCTL, otp);
	}

	return (ret);
}

static boolean_t
atge_read_eeprom(struct gem_dev *dp, uint_t offset, uint32_t *valp)
{
	struct atge_dev	*lp = dp->private;

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		return (atge_read_eeprom_1c(dp, offset, valp));
	}
	return (atge_read_eeprom_1(dp, offset, valp));
}

#ifdef DEBUG_LEVEL
static void
atge_eeprom_dump(struct gem_dev *dp)
{
	int		i;
#define	EEPROM_DUMP_SIZE	0x10
	uint32_t	prom[EEPROM_DUMP_SIZE];

	for (i = 0; i < EEPROM_DUMP_SIZE; i++) {
		(void) atge_read_eeprom(dp, i*4, &prom[i]);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < EEPROM_DUMP_SIZE; i += 4) {
		cmn_err(CE_CONT, "!0x%02x: 0x%08x 0x%08x 0x%08x 0x%08x",
		    i, prom[i], prom[i + 1], prom[i + 2], prom[i + 3]);
	}
#undef	EEPROM_DUMP_SIZE
}
#endif

static boolean_t
atge_read_mac_addr(struct gem_dev *dp, uint8_t *mac)
{
	uint32_t	i;
	uint_t		offset;
	uint32_t	val;
	uint16_t	reg;
	boolean_t	(*read_prom)(struct gem_dev *, uint_t, uint32_t *);
	static uint8_t	zero6[ETHERADDRL] = {0, 0, 0, 0, 0, 0};

	/* check if eeprom exists */
	val = INL(dp, FLASHCTL);
	if (val & FLASHCTL_VPD_EN) {
		val &= ~FLASHCTL_VPD_EN;
		OUTL(dp, FLASHCTL, val);
	}

	if ((INW(dp, PCIE_CAP_LIST) & 0xff00) == 0x6c00) {
		/* get mac address from EEPROM content */
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: reading factory mac address from eeprom",
		    dp->name, __func__));
		read_prom = &atge_read_eeprom;
		offset = 0x100;
	} else {
		cmn_err(CE_NOTE,
		    "!%s: %s: reading flash rom: not supported",
		    dp->name, __func__);
		return (B_FALSE);
	}

	for (i = 0; i < 2048; i++) {
		if (!(*read_prom)(dp, offset, &val)) {
			break;
		}
		offset += 4;

		if ((val & 0xff) != 0x5a) {
			break;
		}
		reg = val >> 16;

		if (!(*read_prom)(dp, offset, &val)) {
			break;
		}
		offset += 4;

		if (reg ==  MACADDR) {
			mac[2] = (uint8_t)(val >> 24);
			mac[3] = (uint8_t)(val >> 16);
			mac[4] = (uint8_t)(val >> 8);
			mac[5] = (uint8_t)val;
		} else if (reg == MACADDR + 4) {
			mac[0] = (uint8_t)(val >> 8);
			mac[1] = (uint8_t)val;
		} else {
			break;
		}
	}

	if ((mac[0] & 1) || bcmp(mac, zero6, ETHERADDRL) == 0) {
		/* wrong mac address */
		return (B_FALSE);
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: mac address found (%x:%x:%x:%x:%x:%x)",
	    dp->name, __func__,
	    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));

	return (B_TRUE);
}

static int
atge_attach_chip(struct gem_dev *dp)
{
	uint8_t		*mac;
	uint32_t	uval;
	struct atge_dev	*lp = dp->private;
	static uint8_t	zero6[6] = {0, 0, 0, 0, 0, 0};

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%ld",
	    dp->name, __func__, ddi_get_lbolt()));

	mac = dp->dev_addr.ether_addr_octet;
	bzero(mac, ETHERADDRL);
#ifdef CONFIG_READ_MACADDR_FROM_EEPROM
	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		/* XXX - todo: load eeprom contants */
		break;

	default:
		(void) atge_read_mac_addr(dp, mac);
	}
#endif
	if ((mac[0] & 1) || bcmp(mac, zero6, ETHERADDRL) == 0) {
		/* try to get mac address from BIOS */
		uval = INL(dp, MACADDR + 4);
		mac[0] = (uint8_t)(uval >> 8);
		mac[1] = (uint8_t)uval;
		uval = INL(dp, MACADDR);
		mac[2] = (uint8_t)(uval >> 24);
		mac[3] = (uint8_t)(uval >> 16);
		mac[4] = (uint8_t)(uval >> 8);
		mac[5] = (uint8_t)uval;
	}
	if ((mac[0] & 1) || bcmp(mac, zero6, ETHERADDRL) == 0) {
		/* factory mac address seems to be corrupted */
		gem_generate_macaddr(dp, mac);
	}

	gem_get_mac_addr_conf(dp);

	if ((mac[0] & 1) || bcmp(mac, zero6, ETHERADDRL) == 0) {
		/* wrong mac address, generate new one. */
		cmn_err(CE_NOTE,
		    "!%s: mac address %x:%x:%x:%x:%x:%x is corrupted.",
		    dp->name,
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

		gem_generate_macaddr(dp, mac);
	}
#ifdef DEBUG_LEVEL
	if (DEBUG_LEVEL > 10) {
		atge_eeprom_dump(dp);
	}
#endif /* DEBUG_LEVEL */

	uval = INL(dp, PCIE_DEVCTRL);
	lp->maxpayload = (uval >> 5) & 7;
	lp->maxrequest = (uval >> 12) & 7;
	lp->maxpayload = min(lp->maxpayload, MAXDMA_1024);
	lp->maxrequest = min(lp->maxrequest, MAXDMA_1024);
#ifndef GEM_CONFIG_JUMBO_FRAME
	/* rx buffer size should be aligned by 8 */
	dp->rx_buf_len = ROUNDUP(dp->rx_buf_len, 8);
#endif
	mutex_init(&lp->mbox_lock,
	    NULL, MUTEX_DRIVER, (void *)dp->iblock_cookie);

	switch (lp->chip) {
	case CHIP_L1:
	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
#ifdef CONFIG_CKSUM_OFFLOAD
#if 0
		/* GEM_CKSUM_HEADER_IPv4 didn't work for partial checksum */
		dp->misc_flag |= GEM_CKSUM_PARTIAL;
#endif
#if 0 /* CONFIG_LSO */
		dp->misc_flag |= GEM_LSO_IPv4 | GEM_LSO_IPv4_PHCKSUM
		    | GEM_LSO_CLR_LEN_IPv4 | GEM_LSO_MULTIPLE_MSS_ONLY;
#endif /* CONFIG_LSO */
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef not_tested /* CONFIG_VLAN_HW */
		dp->misc_flag |= GEM_VLAN_HARD;
#else
		dp->misc_flag |= GEM_VLAN_SOFT;
#endif
		break;

	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
#ifdef CONFIG_CKSUM_OFFLOAD
#ifdef CONFIG_LSO
		dp->misc_flag |= GEM_LSO_IPv4 | GEM_LSO_IPv4_PHCKSUM
		    | GEM_LSO_CLR_LEN_IPv4 | GEM_LSO_MULTIPLE_MSS_ONLY;
#endif
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef CONFIG_VLAN_HW
		dp->misc_flag |= GEM_VLAN_HARD;
#else
		dp->misc_flag |= GEM_VLAN_SOFT;
#endif
		break;

	case CHIP_L2:
	default:
		break;
	}

	/*
	 * configure rx
	 */
	switch (lp->chip) {
	case CHIP_L1:
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		dp->misc_flag |= GEM_POLL_RXONLY;
		break;

	case CHIP_L2:
		dp->misc_flag |= GEM_NORXBUF;
		break;

	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
		dp->misc_flag |= GEM_NORXBUF;
		lp->rxf_buf_hiwat =  lp->rxf_buf_len  -
		    ROUNDUP(sizeof (struct ether_header) + dp->mtu + ETHERFCSL
		    + VTAG_SIZE + sizeof (struct rx_status_desc), 32);
		dp->misc_flag |= GEM_POLL_RXONLY;
		break;
	}

	return (GEM_SUCCESS);
}

#ifdef GEM_CONFIG_JUMBO_FRAME
static void
atge_fixup_params(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	/* rx buffer size should be aligned by 8 */
	dp->rx_buf_len = ROUNDUP(dp->rx_buf_len, 8);
}

#endif
static int
atge_update_stats(struct gem_dev *dp)
{
	int		i;
	int		diff;
	uint32_t	uval;
	uint32_t	*statp;
	struct atge_dev	*lp = dp->private;

	DPRINTF(4, (CE_CONT, "!%s: %s:", dp->name, __func__));

	statp = (uint32_t *)(dp->io_area + lp->sm_offset);

	switch (lp->chip) {
	case CHIP_L2:
		/* no statistics info */
		return (GEM_SUCCESS);

	case CHIP_L1:
		atge_ioarea_dma_sync(dp, lp->sm_offset,
		    (size_t)(sizeof (uint32_t) * STAT_MAX),
		    DDI_DMA_SYNC_FORKERNEL);
		break;

	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		/* read statistics counter registers by ourselves */
		for (i = 0; i < STAT_MAX; i++) {
			statp[i] = INL(dp,
			    MAC_STATS_BASE + i * sizeof (uint32_t));
		}
		break;
	}

	for (i = 0; i < STAT_MAX; i++) {
		diff = statp[i];
		switch (i) {
		case STAT_RxCRC:
			dp->stats.errrcv += diff;
			dp->stats.crc += diff;
			break;

		case STAT_RxLONG:
		case STAT_RxLEN:
			dp->stats.errrcv += diff;
			dp->stats.frame_too_long += diff;
			break;

		case STAT_RxRUNT: /* too short and crc is good */
		case STAT_RxFRAG: /* too short and crc is bad */
			dp->stats.errrcv += diff;
			dp->stats.runt += diff;
			break;

		case STAT_RxFIFO:
			dp->stats.errrcv += diff;
			dp->stats.overflow += diff;
			break;

		case STAT_RxNORRD:
			dp->stats.errrcv += diff;
			dp->stats.norcvbuf += diff;
			break;

		case STAT_RxALIGN:
			dp->stats.errrcv += diff;
			dp->stats.frame += diff;
			break;

		case STAT_TxEXCDEFER:
		case STAT_TxABORT:
			dp->stats.errxmt += diff;
			dp->stats.excoll += diff;
			break;

		case STAT_TxDEFER:
			dp->stats.defer += diff;
			break;

		case STAT_Tx1COL:
			dp->stats.errxmt += diff;
			dp->stats.first_coll += diff;
			break;

		case STAT_TxMCOL:
			dp->stats.errxmt += diff;
			dp->stats.multi_coll += diff;
			break;

		case STAT_TxLATE_COL:
			dp->stats.errxmt += diff;
			dp->stats.xmtlatecoll += diff;
			break;

		case STAT_TxUNDERRUN:
		case STAT_TxTRUNC:
			dp->stats.errxmt += diff;
			dp->stats.underflow += diff;
			break;

		default:
		case STAT_TxCTRL:
		case STAT_TxBYTE:
		case STAT_TxRD_EOP:
		case STAT_TxLEN:
			break;
		}
	}

#if 0
	DPRINTF(4, (CE_CONT, "!%s: %s: sm_magic:%x",
	    dp->name, __func__, statp[STAT_SM_VALID]));
	statp[STAT_SM_VALID] = 0;
#endif
	return (GEM_SUCCESS);
}

static int
atge_get_stats(struct gem_dev *dp)
{
	clock_t	now;
	struct atge_dev	*lp = dp->private;

	now = ddi_get_lbolt();

	if (now - lp->last_stats_update > drv_usectohz(10*1000)) {
		atge_update_stats(dp);
		lp->last_stats_update = now;
	}
	return (GEM_SUCCESS);

}

/*
 * tx/rx descriptor manipulations
 */
#ifdef DEBUG_LEVEL
static int atge_tx_frags[MAXTXFRAGS];
#endif
static int
atge_tx_desc_write_1_1e(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int	i;
	int	used;
	uint_t	tx_ring_size = dp->gc.gc_tx_ring_size;
	uint64_t	headerlen;
	uint64_t	addr;
	uint64_t	mark;
	uint64_t	tmp;
#ifdef CONFIG_CKSUM_OFFLOAD
	uint64_t	mss;
	uint64_t	hck_start;
	uint64_t	hck_stuff;
	uint64_t	ip_header_len;
	uint64_t	lsolen = 0;
#endif
#ifdef CONFIG_VLAN_HW
	uint64_t	vtag;
#endif
	struct atge_dev	*lp = dp->private;
	struct tx_buffer_desc	*tdp;
	ddi_dma_cookie_t	*dcp;


#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags:0x%llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 2
	if (flags & GEM_TXFLAG_INTR) {
		cmn_err(CE_CONT,
		    "!%s: %s time: %d, "
		    "seqnum: %d, slot %d, frags: %d flags:0x%llx",
		    dp->name, __func__, ddi_get_lbolt(),
		    dp->tx_desc_tail, slot, frags, flags);
	}
#endif
#if DEBUG_LEVEL > 0
	flags |= GEM_TXFLAG_INTR;
#endif
#ifdef DEBUG_LEVEL
	atge_tx_frags[min(frags, MAXTXFRAGS) - 1]++;
#endif
	/*
	 * write tx descriptor(s)
	 */
	mark = 0;
#ifdef CONFIG_ADAPTIVE_COALESCE
	/*
	 * XXX - don't use TBF_DMAINT or TBF_TXINT for L1E chipset.
	 * it caused tx-hang.
	 */
	if (lp->chip == CHIP_L1 && (flags & GEM_TXFLAG_INTR)) {
		mark |= TBF_DMAINT;
	}
#endif
	headerlen = 0;

#ifdef CONFIG_CKSUM_OFFLOAD
	/* Check for TCP Large Segmentation Offload */
	mss = (flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;
	hck_start = (flags & GEM_TXFLAG_HCKSTART) >> GEM_TXFLAG_HCKSTART_SHIFT;
	hck_stuff = (flags & GEM_TXFLAG_HCKSTUFF) >> GEM_TXFLAG_HCKSTUFF_SHIFT;

	ip_header_len = hck_start - sizeof (struct ether_header);

	if (mss) {
		uint64_t	tcp_header_len;

		lsolen = dmacookie[0].dmac_type;
		tcp_header_len =
		    ((flags & GEM_TXFLAG_TCPHLEN)
		    >> GEM_TXFLAG_TCPHLEN_SHIFT) * 4;
		headerlen = tcp_header_len + hck_start;

		mark |= mss << TBF_MSS_SHIFT
		    | TBF_HDRFLG
		    | (tcp_header_len >> 2) << TBF_TCPHL_SHIFT
		    | (ip_header_len >> 2) << TBF_IPHL_SHIFT
		    | TBF_TCPC
		    | TBF_IPC
		    | TBF_SEGMENT;
		if (flags & GEM_TXFLAG_IPv6) {
			mark |= TBF_COALESE;
		}

		/* TBF_CKSUM flag is not required */
		if (flags & GEM_TXFLAG_SWVTAG) {
			/* XXX - need to check if it is really required */
			mark |= TBF_ETHTYPE;
		}
	} else if (hck_start) {
		/* XXX - don't use custom checksum for UDP */
		mark |= hck_stuff << TBF_CKSUMOFF_SHIFT
		    | hck_start << TBF_PAYLDOFF_SHIFT
		    | TBF_CKSUM;
	}
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef CONFIG_VLAN_HW
	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		vtag = vtag << 4 | vtag >> 13 | ((vtag >> 9) & 0x8);
		vtag = TBF_INSVTAG | vtag << TBF_VTAG_SHIFT;
		mark |= vtag;
	}
#endif
	dcp = &dmacookie[0];
	used = 0;
	for (i = 0; i < frags - 1; i++, dcp++) {
		ASSERT(dcp->dmac_laddress != 0);
		if (headerlen > 0 && headerlen < dcp->dmac_size) {
			tdp = &((struct tx_buffer_desc *)(dp->io_area +
			    lp->tbd_offset))[SLOT(slot + used, tx_ring_size)];
			used++;

			tmp = mark | headerlen;
			addr = dcp->dmac_laddress;
			tdp->tb_flags = LE_64(tmp);
			tdp->tb_addr = LE_64(addr);

			mark &= ~(TBF_BUFLEN | TBF_HDRFLG);
			dcp->dmac_size -= headerlen;
			dcp->dmac_laddress += headerlen;
			lsolen -= headerlen;
			headerlen = 0;
#if DEBUG_LEVEL > 10
		} else if (headerlen > dcp->dmac_size) {
			/* no need to break the fragment in two pieces */
			cmn_err(CE_CONT, "!%s: header is fragmented",
			    __func__);
#endif
		}
		tdp = &((struct tx_buffer_desc *)(dp->io_area +
		    lp->tbd_offset))[SLOT(slot + used, tx_ring_size)];
		used++;

		tmp = mark | dcp->dmac_size;
		addr = dcp->dmac_laddress;
		tdp->tb_flags = LE_64(tmp);
		tdp->tb_addr = LE_64(addr);

		mark &= ~(TBF_BUFLEN | TBF_HDRFLG);

		headerlen -= min(headerlen, dcp->dmac_size);
		lsolen -= dcp->dmac_size;
		if (mss && lsolen == 0) {
			tdp->tb_flags |= LE_64(TBF_EOP);

			mss = 0;
			mark = 0;
			if (lp->chip == CHIP_L1 && (flags & GEM_TXFLAG_INTR)) {
				mark |= TBF_DMAINT;
			}
			ASSERT(headerlen == 0);
			ASSERT(mark == 0);
#ifdef CONFIG_VLAN_HW
			if (vtag) {
				mark |= vtag;
			}
#endif
			mark |= hck_stuff << TBF_CKSUMOFF_SHIFT
			    | hck_start << TBF_PAYLDOFF_SHIFT
			    | TBF_CKSUM;
		}
	}

	tdp->tb_flags |= LE_64(TBF_EOP);

#if DEBUG_LEVEL > 4
	if (mss) {
		cmn_err(CE_CONT,
		    "!%s: frags:%d, used:%d, addr:%p flags:%p",
		    __func__, frags, used,
		    LE_64(tdp->tb_addr), LE_64(tdp->tb_flags));
	}
#endif
	return (used);
}

static void
atge_tx_start_1_1e(struct gem_dev *dp, int start_slot, int nslot)
{
	int		tail;
	struct atge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	atge_tx_desc_dma_sync(dp, start_slot, nslot);

	/* kick Tx dma engine */
	tail = SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size);
	if (lp->chip == CHIP_L1) {
		mutex_enter(&lp->mbox_lock);
		lp->tx_buffer_tail = tail;
		UPDATE_MBOX_1(dp);
		mutex_exit(&lp->mbox_lock);
	} else /* L1E */ {
		lp->tx_buffer_tail = tail;
		OUTL(dp, MBOX_1e, tail);
	}
}

#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
atge_tx_desc_head_1(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	return (lp->tx_buffer_head);
}

static uint_t
atge_tx_desc_head_1e(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	lp->tx_buffer_head = INW(dp, TBDHEAD_1e);
	return (lp->tx_buffer_head);
}
#else
static uint_t
atge_tx_desc_stat_1(struct gem_dev *dp, int slot, int ndesc)
{
	int	last;
	struct atge_dev	*lp = dp->private;

	last = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(last, lp->tx_buffer_head, lp->tx_buffer_tail)) {
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: slot:%d+%d tx_buffer:[%d, %d](%d)",
		    dp->name, __func__,
		    slot, ndesc,
		    lp->tx_buffer_head, lp->tx_buffer_tail,
		    lp->tx_buffer_tail - lp->tx_buffer_head));
		return (0);
	}
	return (GEM_TX_DONE);
}

static uint_t
atge_tx_desc_stat_1e(struct gem_dev *dp, int slot, int ndesc)
{
	int	last;
	struct atge_dev	*lp = dp->private;

	last = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(last, lp->tx_buffer_head, lp->tx_buffer_tail)) {
		/* update tx_buffer_head and check again */
		lp->tx_buffer_head = INW(dp, TBDHEAD_1e);
		if (INSIDE(last, lp->tx_buffer_head, lp->tx_buffer_tail)) {
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: slot:%d+%d tx_buffer:[%d, %d](%d)",
			    dp->name, __func__,
			    slot, ndesc,
			    lp->tx_buffer_head, lp->tx_buffer_tail,
			    lp->tx_buffer_tail - lp->tx_buffer_head));
			return (0);
		}
	}
	return (GEM_TX_DONE);
}
#endif /* GEM_CONFIG_TX_HEAD_PTR */

static void
atge_tx_desc_dump_1_1e(struct gem_dev *dp)
{
	int	i;
	struct tx_buffer_desc	*tdp;
	struct atge_dev	*lp = dp->private;

	cmn_err(CE_CONT, "!%s: %s: tx_ring_size:%d, head:%d, tail:%d",
	    dp->name, __func__, dp->gc.gc_tx_ring_size,
	    lp->tx_buffer_head, lp->tx_buffer_tail);

	for (i = 0; i < dp->gc.gc_tx_ring_size; i++) {
		tdp = &((struct tx_buffer_desc *)(dp->io_area +
		    lp->tbd_offset))[i];
		cmn_err(CE_CONT, "!%d: %llx %llx",
		    LE_64(tdp->tb_addr),
		    LE_64(tdp->tb_flags));
	}
}

static void
atge_tx_desc_init_1_1e(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

static int
atge_tx_desc_write_2(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int	i;
	size_t	len;
	size_t	rest;
	uint32_t	mark;
	struct atge_dev	*lp = dp->private;
	caddr_t		tbp;
	caddr_t		txbuf;
#ifdef CONFIG_VLAN_HW
	uint_t	vtag;
#endif

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags:0x%llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor
	 */
#ifdef GEM3
	txbuf = dp->tx_curr_txbuf->txb_buf;
#else
#define	GET_TXBUF(dp, sn)	\
	(dp)->tx_buf[SLOT((dp)->tx_slots_base + (sn), (dp)->gc.gc_tx_buf_size)]

	txbuf = GET_TXBUF(dp, dp->tx_desc_tail).txb_buf;
#endif
	len = dmacookie->dmac_size;
	mark = len;
#ifdef CONFIG_VLAN_HW
	if (vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT) {
		vtag = ((vtag << 4) & 0xfff0)
		    | ((vtag >> 13) & 0x0007)
		    | ((vtag >> 9) & 0x8);
		mark |= TH_INSVTAG | vtag << TH_VTAG_SHIFT;
	}
#endif
	tbp = dp->io_area + lp->tbd_offset + lp->tx_buffer_tail;
	*(uint32_t *)tbp = LE_32(mark);

	lp->tx_buffer_tail += 4;
	if (lp->tx_buffer_tail >= lp->tbd_size) {
		lp->tx_buffer_tail -= lp->tbd_size;
	}

	rest = len;
	len = min(len, lp->tbd_size - lp->tx_buffer_tail);
	tbp = dp->io_area + lp->tbd_offset + lp->tx_buffer_tail;
	bcopy(txbuf, tbp, len);
	rest -= len;
	if (rest) {
		bcopy(txbuf + len, dp->io_area + lp->tbd_offset, rest);
		lp->tx_buffer_tail = ROUNDUP(rest, 4);
	} else {
		lp->tx_buffer_tail += ROUNDUP(len, 4);
		if (lp->tx_buffer_tail >= lp->tbd_size) {
			lp->tx_buffer_tail -= lp->tbd_size;
		}
	}
	/* save the last position */
	ASSERT((lp->tx_buffer_tail & 3) == 0);
	lp->tbd_pos[slot] = lp->tx_buffer_tail;

	/* clear tx status descriptor */
	((uint32_t *)dp->tx_ring)[slot] = 0;

	DPRINTF(1, (CE_CONT, "!%s: %s: slot:%d len:%d+%d pos:0x%x",
	    dp->name, __func__, slot, len, rest, lp->tbd_pos[slot]));
	return (1);
}

static void
atge_tx_start_2(struct gem_dev *dp, int start_slot, int nslot)
{
	int	last;
	struct atge_dev	*lp = dp->private;

	last = SLOT(start_slot + nslot - 1, dp->gc.gc_tx_ring_size);
	OUTW(dp, TBDTAIL_2, lp->tbd_pos[last] / 4);

	DPRINTF(1, (CE_CONT, "!%s: %s: start_slot:%d nslot:%d, pos:0x%x",
	    dp->name, __func__, start_slot, nslot, lp->tbd_pos[last]));
}

static uint_t
atge_tx_desc_stat_2(struct gem_dev *dp, int slot, int ndesc)
{
	uint32_t	tsr;

	ASSERT(ndesc == 1);
	tsr = ((uint32_t *)dp->tx_ring)[slot];
	tsr = LE_32(tsr);

	DPRINTF(1, (CE_CONT, "!%s: %s: slot:%d tsr:0x%b",
	    dp->name, __func__, slot, tsr, TS_BITS));

	return ((tsr & TS_UPDATE) ? GEM_TX_DONE : 0);
}

static void
atge_tx_desc_init_2(struct gem_dev *dp, int slot)
{
	((uint32_t *)(dp->tx_ring))[slot] = 0;
}

#ifdef DEBUG_LEVEL
static int atge_tx_frags[MAXTXFRAGS];
#endif
static int
atge_tx_desc_write_1c(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	int	i;
	int	used = 0;
	uint_t	tx_ring_size = dp->gc.gc_tx_ring_size;
	uint64_t	headerlen;
	uint64_t	addr;
	uint64_t	mark;
	uint64_t	tmp;
#ifdef CONFIG_CKSUM_OFFLOAD
	uint64_t	mss;
	uint64_t	hck_start;
	uint64_t	hck_stuff;
	uint64_t	ip_header_len;
	int		lsolen = 0;
#endif
#ifdef CONFIG_VLAN_HW
	uint64_t	vtag;
#endif
	struct atge_dev	*lp = dp->private;
	struct tx_buffer_desc	*tdp;
	ddi_dma_cookie_t	*dcp;

#if DEBUG_LEVEL > 4
	cmn_err(CE_CONT,
	    "!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags:0x%llx",
	    dp->name, __func__, ddi_get_lbolt(),
	    dp->tx_desc_tail, slot, frags, flags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x, lsolen: %d",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size,
		    dmacookie[i].dmac_type);
	}
#endif
#if DEBUG_LEVEL > 2
	if (flags & GEM_TXFLAG_INTR) {
		cmn_err(CE_CONT,
		    "!%s: %s time: %d, "
		    "seqnum: %d, slot %d, frags: %d flags:0x%llx",
		    dp->name, __func__, ddi_get_lbolt(),
		    dp->tx_desc_tail, slot, frags, flags);
	}
#endif
#ifdef DEBUG_LEVEL
	atge_tx_frags[min(frags, MAXTXFRAGS) - 1]++;
#endif
	/*
	 * write tx descriptor(s)
	 */
	mark = 0;
	headerlen = 0;

#ifdef CONFIG_CKSUM_OFFLOAD
	/* Check for TCP Large Segmentation Offload */
	mss = (flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;
	hck_start = (flags & GEM_TXFLAG_HCKSTART) >> GEM_TXFLAG_HCKSTART_SHIFT;
	hck_stuff = (flags & GEM_TXFLAG_HCKSTUFF) >> GEM_TXFLAG_HCKSTUFF_SHIFT;

	ip_header_len = hck_start - sizeof (struct ether_header);

	if (mss) {
		uint64_t	tcp_header_len;

		tcp_header_len =
		    ((flags & GEM_TXFLAG_TCPHLEN)
		    >> GEM_TXFLAG_TCPHLEN_SHIFT) * 4;
		headerlen = hck_start + tcp_header_len;

		lsolen = dmacookie[0].dmac_type;

		if (flags & GEM_TXFLAG_IPv4) {
			if (lsolen == headerlen) {
#ifdef DEBUG_LEVEL
				cmn_err(CE_CONT, "!%s: no tcp payload",
				    __func__);
#endif
				mss = 0;
				goto hck_sum;
			}
			mark |= mss << TPD_MSS_SHIFT
			    | hck_start << TPD_CKOFF_SHIFT
			    | TPD_LSO | TPD_LSO_IPv4;
		}
		if (flags & GEM_TXFLAG_IPv6) {
			if (lsolen == headerlen) {
				mss = 0;
				goto hck_sum;
			}
			mark |= mss << TPD_MSS_SHIFT
			    | hck_start << TPD_CKOFF_SHIFT
			    | TPD_LSO | TPD_LSO_IPv6;

			/*
			 * IPv6 needs extra tpd
			 */
			tdp = &((struct tx_buffer_desc *)(dp->io_area +
			    lp->tbd_offset))[SLOT(slot + used, tx_ring_size)];
			used++;

			tdp->tpd_flags = TPD_LSO | TPD_LSO_IPv6;
			tdp->tpd_addr = lsolen;
		}

		/* TBF_CKSUM flag is not required */
		if (flags & GEM_TXFLAG_SWVTAG) {
			/* XXX - need to check if it is really required */
			mark |= TPD_ETHTYPE;
		}
		DPRINTF(4, (CE_CONT,
		    "headerlen:%d, tcphlen:%d, lsolen:%d, mss_frag:%d",
		    headerlen, tcp_header_len, lsolen,
		    (lsolen - hck_start - tcp_header_len) % mss));
	} else if (hck_start) {
hck_sum:
		/* XXX - don't use custom checksum for UDP */
		mark |= (hck_stuff >> 1) << TPD_CCSUM_OFF_SHIFT
		    | (hck_start >> 1) << TPD_CKOFF_SHIFT
		    | TPD_CCSUM;
	}
#endif /* CONFIG_CKSUM_OFFLOAD */
#ifdef CONFIG_VLAN_HW
	vtag = (flags >> GEM_TXFLAG_VTAG_SHIFT) & 0xffff;
	if (vtag) {
		vtag = TPD_INSVTAG | BSWAP_16(vtag) << TPD_VTAG_SHIFT;
		mark |= vtag;
	}
#endif
	dcp = &dmacookie[0];
	for (i = 0; i < frags; i++, dcp++) {
		ASSERT(dcp->dmac_laddress != 0);
#ifdef DEBUG_LEVEL
		if (dcp->dmac_size == 0) {
			cmn_err(CE_CONT, "!%s: dmac_sizep == 0", __func__);
		}
#endif
		if (headerlen > 0 && headerlen < dcp->dmac_size) {
			tdp = &((struct tx_buffer_desc *)(dp->io_area +
			    lp->tbd_offset))[SLOT(slot + used, tx_ring_size)];
			used++;

			tmp = mark | headerlen;
			addr = dcp->dmac_laddress;
			tdp->tpd_addr = LE_64(addr);
			tdp->tpd_flags = LE_64(tmp);

			dcp->dmac_size -= headerlen;
			dcp->dmac_laddress += headerlen;
			lsolen -= headerlen;

			headerlen = 0;
			mark = 0;
#if DEBUG_LEVEL > 10
		} else if (headerlen > dcp->dmac_size) {
			/* no need to break the fragment in two pieces */
			cmn_err(CE_CONT, "!%s: header is fragmented",
			    __func__);
#endif
		}

		tdp = &((struct tx_buffer_desc *)(dp->io_area +
		    lp->tbd_offset))[SLOT(slot + used, tx_ring_size)];
		used++;

		tmp = mark | dcp->dmac_size;
		addr = dcp->dmac_laddress;
		tdp->tpd_addr = LE_64(addr);
		tdp->tpd_flags = LE_64(tmp);
		mark = 0;

		headerlen -= min(headerlen, dcp->dmac_size);
		lsolen -= dcp->dmac_size;
		if (mss && lsolen == 0) {
			tdp->tpd_flags |= LE_64(TPD_EOP);

			mss = 0;
			ASSERT(headerlen == 0);
			ASSERT(mark == 0);
#ifdef CONFIG_VLAN_HW
			if (vtag) {
				mark |= vtag;
			}
#endif
			mark |= (hck_stuff >> 1) << TPD_CCSUM_OFF_SHIFT
			    | (hck_start >> 1) << TPD_CKOFF_SHIFT
			    | TPD_CCSUM;
		}
	}
	tdp->tpd_flags |= LE_64(TPD_EOP);

#if DEBUG_LEVEL > 4
	if (mss) {
		cmn_err(CE_CONT,
		    "!%s: frags:%d, used:%d, addr:%p flags:%p",
		    __func__, frags, used,
		    LE_64(tdp->tpd_addr), LE_64(tdp->tpd_flags));
	}
#endif
	return (used);
}

static void
atge_tx_start_1c(struct gem_dev *dp, int start_slot, int nslot)
{
	int		tail;
	struct atge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	atge_tx_desc_dma_sync(dp, start_slot, nslot);

	/* kick Tx dma engine */
	tail = SLOT(start_slot + nslot, dp->gc.gc_tx_ring_size);
	lp->tx_buffer_tail = tail;
	OUTL(dp, MB_PRIO_PROD_IDX, tail << 16);
}

#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
atge_tx_desc_head_1c(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	lp->tx_buffer_head = INL(dp, MB_PRIO_CONS_IDX) >> 16;
	return (lp->tx_buffer_head);
}
#else
static uint_t
atge_tx_desc_stat_1c(struct gem_dev *dp, int slot, int ndesc)
{
	int	last;
	struct atge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: slot:%d, ndesc:%d",
	    dp->name, __func__, slot, ndesc));

	last = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(last, lp->tx_buffer_head, lp->tx_buffer_tail)) {
		/* update tx_buffer_head and check again */
		lp->tx_buffer_head = INL(dp, MB_PRIO_CONS_IDX) >> 16;
		if (INSIDE(last, lp->tx_buffer_head, lp->tx_buffer_tail)) {
			DPRINTF(2, (CE_CONT,
			    "!%s: %s: slot:%d+%d tx_buffer:[%d, %d](%d)",
			    dp->name, __func__,
			    slot, ndesc,
			    lp->tx_buffer_head, lp->tx_buffer_tail,
			    lp->tx_buffer_tail - lp->tx_buffer_head));
			return (0);
		}
	}
	return (GEM_TX_DONE);
}
#endif

static void
atge_tx_desc_dump_1c(struct gem_dev *dp)
{
	int	i;
	struct tx_buffer_desc	*tdp;
	struct atge_dev	*lp = dp->private;

	cmn_err(CE_CONT, "!%s: %s: tx_ring_size:%d, head:%d, tail:%d",
	    dp->name, __func__, dp->gc.gc_tx_ring_size,
	    lp->tx_buffer_head, lp->tx_buffer_tail);

	for (i = 0; i < dp->gc.gc_tx_ring_size; i++) {
		tdp = &((struct tx_buffer_desc *)(dp->io_area +
		    lp->tbd_offset))[i];
		cmn_err(CE_CONT, "!%d: %llx %llx",
		    i,
		    LE_64(tdp->tpd_flags),
		    LE_64(tdp->tpd_addr));
	}
}

static void
atge_tx_desc_init_1c(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

static void
atge_rx_desc_write_1(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	uint64_t	dmaaddr;
	uint32_t	hi_addr;
	uint32_t	lo_addr;
	uint64_t	len;
	struct atge_dev	*lp = dp->private;
	struct rx_buffer_desc	*rbdp;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT,
	    "!%s: %s: seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
	/*
	 * write a RX descriptor
	 */
	rbdp = &((struct rx_buffer_desc *)(dp->io_area + lp->rbd_offset))[slot];

	dmaaddr = dmacookie->dmac_laddress;
	lo_addr = (uint32_t)dmaaddr;
	hi_addr = (uint32_t)(dmaaddr >> 32);
	len = dmacookie->dmac_size;

	rbdp->rb_addr = LE_32(lo_addr);
	rbdp->rb_haddr = LE_32(hi_addr);
	rbdp->rb_flags = LE_32(len);
}

static void
atge_rx_start_1(struct gem_dev *dp, int start_slot, int nslot)
{
	int	tail;
	struct atge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	atge_rbd_desc_dma_sync_1(dp, start_slot, nslot);

	tail = SLOT(start_slot + nslot, dp->gc.gc_rx_ring_size);
	mutex_enter(&lp->mbox_lock);
	lp->rx_buffer_tail = tail;
	UPDATE_MBOX_1(dp);
	mutex_exit(&lp->mbox_lock);
}

static uint64_t
atge_rx_desc_stat_1(struct gem_dev *dp, int slot, int ndesc)
{
	int		retry;
	int		head;
	uint_t		index;
	uint64_t	len;
	uint64_t	rsd0;
	uint64_t	rsd1;
	uint64_t	ret;
	struct rx_status_desc	*rsdp;
	struct atge_dev	*lp = dp->private;

	retry = 0;
	ret = 0;
again:
	rsdp = &((struct rx_status_desc *)(dp->rx_ring))[lp->rx_status_head];
	rsd0 = rsdp->rsd0;
	rsd1 = rsdp->rsd1;

	rsd0 = LE_64(rsd0);
	rsd1 = LE_64(rsd1);

	if ((rsd0 & RSD0_OWN) == 0) {
		/* not yet received */
		return (0);
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: slot:%x, %llx, %llx, flags:%b, rsd/rbd index:0x%x",
	    dp->name, __func__, slot, rsd0, rsd1,
	    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS,
	    INL(dp, CURR_RBD_RSD_1)));

	/* invalidate the rx status descriptor */
	rsdp->rsd0 = 0;
	ret = GEM_RX_DONE;

	if ((rsd0 & RSD0_NUM) != 1) {
		if (retry++ < 1) {
			gem_rx_desc_dma_sync(dp,
			    slot, 1, DDI_DMA_SYNC_FORKERNEL);
			goto again;
		}
		cmn_err(CE_WARN, "!%s: %s: (rsd0(%llx) & RSD0_NUM) != 1",
		    dp->name, __func__, rsd0);
		lp->need_to_reset = B_TRUE;
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	index = (rsd0 & RSD0_INDEX) >> RSD0_INDEX_SHIFT;
	if (slot != index) {
		cmn_err(CE_WARN, "!%s: %s: slot(%d) != index(%d)",
		    dp->name, __func__, slot, index);
		lp->need_to_reset = B_TRUE;
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/* now current slot == index in current return descriptor */

	/*
	 * Increase rx_status_head to point the next descriptor,
	 * we don't update MBOX here. it have to be updated when new
	 * rx buffers are prepared later. we also don't use mutex lock,
	 * because an assignment of a 4 byte word is naturally atomic,
	 * the modification is only performed in this function and
	 * intrlock is held.
	 */
	lp->rx_status_head = SLOT(index + 1, dp->gc.gc_rx_ring_size);

#ifdef NEVER
	if (rsd1 & (RSD1_E_TRUNC | RSD1_E_OVF |
	    RSD1_E_RUNT | RSD1_E_CODE | RSD1_E_CRC)) {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: slot:%x, %llx, %llx, flags:%b, "
		    "rsd/rbd index:0x%x",
		    dp->name, __func__, slot, rsd0, rsd1,
		    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS,
		    INL(dp, CURR_RBD_RSD_1)));

		ret |= GEM_RX_ERR;
		dp->stats.errrcv++;
		if (rsd1 & RSD1_E_TRUNC) {
			dp->stats.frame_too_long++;
		} else if (rsd1 & RSD1_E_OVF) {
			dp->stats.overflow++;
		} else if (rsd1 & RSD1_E_RUNT) {
			dp->stats.runt++;
		} else if (rsd1 & RSD1_E_CODE) {
			dp->stats.frame++;
		} else if (rsd1 & RSD1_E_CRC) {
			dp->stats.crc++;
		} else {
			dp->stats.rcv_internal_err++;
		}
	}
#endif /* USE_GETSTAT_RX */

	len = (rsd0 & RSD0_LEN) >> RSD0_LEN_SHIFT;
	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}
#ifdef CONFIG_CKSUM_OFFLOAD
	if ((rsd1 & RSD1_IPv4) &&
	    (rsd1 & (RSD1_E_IPCHKSUM | RSD1_E_L4CHKSUM)) == 0) {
		/* IPv4 packets, full checksum */
		ret |= GEM_RX_CKSUM_IPv4;
		if (rsd1 & RSD1_TCP) {
			ret |= GEM_RX_CKSUM_TCP;
		} else if (rsd1 & RSD1_UDP) {
			ret |= GEM_RX_CKSUM_UDP;
		}
	}
#endif /* CONFIG_CKSUM_OFFLOAD */
	return (ret | len);
}

static void
atge_rx_desc_init_1(struct gem_dev *dp, int slot)
{
	struct rx_status_desc	*rsdp;

	rsdp = &((struct rx_status_desc *)(dp->rx_ring))[slot];
	rsdp->rsd0 = 0;
}

static void
atge_rx_desc_write_2(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	uint32_t	*rbdp;
	struct atge_dev	*lp = dp->private;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT,
	    "!%s: %s: seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
	/*
	 * write a RX descriptor
	 */
	rbdp = (uint32_t *)(dp->io_area + lp->rbd_offset + slot * 1536);

	rbdp[0] = 0;
}

static void
atge_rx_start_2(struct gem_dev *dp, int start_slot, int nslot)
{
	struct atge_dev	*lp = dp->private;

	DPRINTF(1, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	atge_rbd_desc_dma_sync_2(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	OUTW(dp, RBDTAIL_2,
	    SLOT(start_slot + nslot, dp->gc.gc_rx_ring_size));
}


static uint64_t
atge_rx_desc_stat_2(struct gem_dev *dp, int slot, int ndesc)
{
	uint64_t	len;
	off_t		io_offset;
	uint32_t	rs0;
	uint32_t	rs1;
	uint32_t	*rsp;
	struct atge_dev	*lp = dp->private;

	/* XXX - don't use ndesc, it is not valid */
	atge_rbd_desc_dma_sync_2(dp, slot, 1, DDI_DMA_SYNC_FORKERNEL);

	io_offset = lp->rbd_offset + lp->rx_buffer_head * 1536;

	rsp = (uint32_t *)(dp->io_area + io_offset);
	rs0 = rsp[0];
	rs0 = LE_32(rs0);
	rs1 = rsp[1];
	rs1 = LE_32(rs1);
	if ((rs0 & RS_UPDATE) == 0) {
		return (0);
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: slot:%x flags:%b %x",
	    dp->name, __func__, slot, rs0, RS_BITS, rs1));

	len = rs0 & RS_PKTSIZE;
	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}
#ifdef CONFIG_VLAN_HW
	if (rs0 & RS_VLAN) {
		len |= (((uint64_t)rs1 & 0xfff0ULL) >> 4) << GEM_RX_VID_SHIFT;
		len |= ((uint64_t)rs1 & 0x0007ULL) << GEM_RX_PRI_SHIFT;
		len |= (((uint64_t)rs1 & 0x0008ULL) >> 3) << GEM_RX_CFI_SHIFT;
	}
#endif
	return (GEM_RX_DONE | len);
}

static mblk_t *
atge_get_packet_2(struct gem_dev *dp, struct rxbuf *rbp, size_t pkt_len)
{
	mblk_t		*mp;
	off_t		io_offset;
	uint32_t	rs0;
	uint32_t	rs1;
	uint32_t	*rsp;
	struct atge_dev	*lp = dp->private;

	/* check rx status desc */
	mp = NULL;
	io_offset = lp->rbd_offset + lp->rx_buffer_head * 1536;

	rsp = (uint32_t *)(dp->io_area + io_offset);
	rs0 = rsp[0];
	rs0 = LE_32(rs0);
	rs1 = rsp[1];
	rs1 = LE_32(rs1);

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: %b %x plen:%d",
	    dp->name, __func__, rs0, RS_BITS, rs1, (int)pkt_len));

	if ((rs0 & RS_OK) == 0) {
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: errored pkt, %b %x plen:%d",
		    dp->name, __func__,
		    rs0, RS_BITS, rs1, (int)pkt_len));

		if (rs0 & (RS_TRUNC | RS_FRAG |
		    RS_RUNT | RS_CODE | RS_CRC)) {
			dp->stats.errrcv++;
			if (rs0 & RS_TRUNC) {
				dp->stats.frame_too_long++;
			}
			if (rs0 & (RS_RUNT | RS_FRAG)) {
				dp->stats.runt++;
			}
			if (rs0 & RS_CODE) {
				dp->stats.frame++;
			}
			if (rs0 & (RS_CRC | RS_FRAG)) {
				dp->stats.crc++;
			}
		} else {
			dp->stats.errrcv++;
			dp->stats.rcv_internal_err++;
		}
		goto x;
	}

	/* aquire packet buffer */
	if ((mp = allocb(pkt_len, BPRI_MED)) == NULL) {
		/*
		 * No receive buffer, OS resource exaust
		 */
		dp->stats.errrcv++;
		dp->stats.norcvbuf++;
		goto x;
	}

	mp->b_wptr = mp->b_rptr + pkt_len;
	bcopy(dp->io_area + io_offset + sizeof (uint32_t) * 2,
	    mp->b_rptr, pkt_len);
x:
	lp->rx_buffer_head =
	    SLOT(lp->rx_buffer_head + 1, dp->gc.gc_rx_ring_size);

	return (mp);
}

static void
atge_rx_desc_init_2(struct gem_dev *dp, int slot)
{
	uint32_t	*rsp;
	struct atge_dev	*lp = dp->private;

	rsp = (uint32_t *)(dp->io_area + lp->rbd_offset + slot * 1536);
	rsp[0] = 0;
}

static void
atge_rx_desc_write_1e(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	/* EMPTY */
}

static void
atge_rxf_dump_1e(struct gem_dev *dp, uint8_t *start, size_t len)
{
	int	i;

	for (i = 0; i < len; i += 8) {
		cmn_err(CE_CONT,
		    "%d: %02x %02x %02x %02x %02x %02x %02x %02x",
		    i,
		    start[i+0], start[i+1], start[i+2], start[i+3],
		    start[i+4], start[i+5], start[i+6], start[i+7]);
	}
}

static mblk_t *
atge_get_packet_1e(struct gem_dev *dp, struct rxbuf *rbp, size_t pkt_len)
{
	int		index;
	int		head;
	int		tail;
	uint64_t	len;
	mblk_t		*mp;
	off_t		io_offset;
	uint64_t	rsd0;
	uint64_t	rsd1;
	struct l2_rxf	*rxfp;
	struct rx_status_desc	*rsdp;
	struct atge_dev	*lp = dp->private;

	mp = NULL;
	rxfp = &lp->rxf[0];
	index = rxfp->rf_curr_ring;

	/*
	 * get current rx status desc. No need to flush io cache lines for
	 * that here. We have flushed in rx_desc_stat_1e.
	 */
	io_offset = lp->rxf_offset[0][index] + rxfp->rf_head;
	rsdp = (struct rx_status_desc *)(dp->io_area + io_offset);
	rsd0 = rsdp->rsd0;
	rsd1 = rsdp->rsd1;

	rsd0 = LE_64(rsd0);
	rsd1 = LE_64(rsd1);

	len = (rsd0 & RSD0_LEN) >> RSD0_LEN_SHIFT;

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: %llx, %llx, flags:%b, pkt_len:%d",
	    dp->name, __func__, rsd0, rsd1,
	    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS, (int)pkt_len));
#ifdef notdef
	if ((uint16_t)(rsd0 & RSD0_SEQNUM) != rxfp->rf_seqnum) {
		cmn_err(CE_WARN,
		    "!%s: %s: wrong seq num, (actual) %d != (expected) %d, "
		    "rsg0:%llx, rsd1:%llx, flags:%b, pkt_len:%d, "
		    "rxf%d + head:%d",
		    dp->name, __func__,
		    (int)(rsd0 & RSD0_SEQNUM), rxfp->rf_seqnum,
		    rsd0, rsd1,
		    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS, (int)pkt_len,
		    index, rxfp->rf_head);
		atge_rxf_dump_1e(dp,
		    (uint8_t *)dp->io_area + (io_offset - 64), 128);
		lp->need_to_reset = B_TRUE;
		goto x;
	}
	rxfp->rf_seqnum++;
#endif
	if (rsd1 &
	    (RSD1_E_DES_ADDR | RSD1_E_LEN | RSD1_E_TRUNC | RSD1_E_OVF |
	    RSD1_E_RUNT | RSD1_E_DRIBBLE | RSD1_E_CODE | RSD1_E_CRC)) {
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: rx errored packet: %llx, %llx, flags:%b",
		    dp->name, __func__, rsd0, rsd1,
		    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS));
#ifdef NEVER
		if (rsd1 & (RSD1_E_TRUNC | RSD1_E_OVF |
		    RSD1_E_RUNT | RSD1_E_CODE | RSD1_E_CRC)) {
			dp->stats.errrcv++;
			if (rsd1 & RSD1_E_TRUNC) {
				dp->stats.frame_too_long++;
			}
			if (rsd1 & RSD1_E_OVF) {
				dp->stats.overflow++;
			}
			if (rsd1 & RSD1_E_RUNT) {
				dp->stats.runt++;
			}
			if (rsd1 & RSD1_E_CODE) {
				dp->stats.frame++;
			}
			if (rsd1 & RSD1_E_CRC) {
				dp->stats.crc++;
			}
		} else {
			dp->stats.errrcv++;
			dp->stats.rcv_internal_err++;
		}
#endif
		goto x;
	}

	/* aquire packet buffer */
	if ((mp = allocb(pkt_len, BPRI_MED)) == NULL) {
		/*
		 * No receive buffer, OS resource exaust
		 */
		dp->stats.errrcv++;
		dp->stats.norcvbuf++;
		goto x;
	}

	mp->b_wptr = mp->b_rptr + pkt_len;
	bcopy(dp->io_area + io_offset + sizeof (struct rx_status_desc),
	    mp->b_rptr, pkt_len);
x:
	/* update next rx buffer offset in the rx ring */
	rxfp->rf_head += ROUNDUP(sizeof (struct rx_status_desc) + len, 32);
	if (rxfp->rf_head >= lp->rxf_buf_hiwat) {
		DPRINTF(4, (CE_CONT, "!%s: %s: switching rx0 ring %d -> %d",
		    dp->name, __func__,
		    rxfp->rf_curr_ring, rxfp->rf_curr_ring ^ 1));

		/* reset current ring */
		io_offset = lp->rxf_mb_offset[0][index];
		*((uint32_t *)(dp->io_area + io_offset)) = 0;
		atge_ioarea_dma_sync(dp, io_offset, sizeof (uint32_t),
		    DDI_DMA_SYNC_FORDEV);

		/* notify the nic that the rx ring become available again */
		OUTB(dp, RXF_RING_VALID_1e + index, 1);

		/* select the next ring */
		rxfp->rf_curr_ring ^= 1;
		rxfp->rf_head = 0;
		rxfp->rf_tail = 0;
	}

	return (mp);
}

static uint64_t
atge_rx_desc_stat_1e(struct gem_dev *dp, int slot, int ndesc)
{
	int		i;
	uint64_t	len;
	uint64_t	ret;
	int		tail;
	struct l2_rxf	*rxfp;
	uint_t		index;
	off_t		io_offset;
	uint64_t	rsd0;
	uint64_t	rsd1;
	struct rx_status_desc	*rsdp;
	struct atge_dev	*lp = dp->private;

	ret = GEM_RX_DONE;

	rxfp = &lp->rxf[0];
	index = rxfp->rf_curr_ring;

	if (rxfp->rf_head < rxfp->rf_tail) {
		/* we have remained packets in current rx ring */
		io_offset = lp->rxf_offset[0][index] + rxfp->rf_head;
		goto read_rx_desc;
	}

	/* get new tail position */
	io_offset = lp->rxf_mb_offset[0][index];
	atge_ioarea_dma_sync(dp, io_offset, sizeof (uint32_t),
	    DDI_DMA_SYNC_FORKERNEL);
	tail = *((uint32_t *)(dp->io_area + io_offset));
	tail = LE_32(tail);

	/* save the new tail */
	rxfp->rf_tail = tail;

	if (rxfp->rf_head == tail) {
		/* not yet received */
		return (0);
	} else if (rxfp->rf_head > tail) {
		cmn_err(CE_WARN,
		    "!%s: wrong rx packet offset, "
		    "rx:%d head:%x tail:%x hiwat:%x",
		    dp->name, index, rxfp->rf_head, rxfp->rf_tail,
		    lp->rxf_buf_hiwat);

		/* XXX - terminate gem_receive() immediately */
		lp->need_to_reset = B_TRUE;
		return (0);
	}

	/* we have new packets */
	for (i = 0; i < 2; i++) {
		io_offset = lp->rxf_offset[0][index] + rxfp->rf_head;
		atge_ioarea_dma_sync(dp,
		    io_offset, rxfp->rf_tail - rxfp->rf_head,
		    DDI_DMA_SYNC_FORKERNEL);
read_rx_desc:
		rsdp = (struct rx_status_desc *)(dp->io_area + io_offset);

		rsd0 = rsdp->rsd0;
		rsd1 = rsdp->rsd1;

		rsd0 = LE_64(rsd0);
		rsd1 = LE_64(rsd1);

		DPRINTF(1, (CE_CONT,
		    "!%s: %s: slot:%x, %llx, %llx, flags:%b",
		    dp->name, __func__, slot, rsd0, rsd1,
		    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS));

		if ((uint16_t)(rsd0 & RSD0_SEQNUM) == rxfp->rf_seqnum) {
			rxfp->rf_seqnum++;
			goto correct_seqnum;
		}

		DPRINTF(-1, (CE_CONT, "!%s: %s: waiting for a while",
		    dp->name, __func__));
		drv_usecwait(10);
	}

	cmn_err(CE_WARN,
	    "!%s: %s: wrong seq num, (actual) %d != (expected) %d, "
	    "rsg0:%llx, rsd1:%llx, flags:%b, pkt_len:%d, "
	    "rxf%d + head:%d",
	    dp->name, __func__,
	    (int)(rsd0 & RSD0_SEQNUM), rxfp->rf_seqnum,
	    rsd0, rsd1,
	    (uint32_t)(rsd1 & RSD1_FLAGS), RSD1_BITS,
	    (int)((rsd0 & RSD0_LEN) >> RSD0_LEN_SHIFT),
	    index, rxfp->rf_head);
	atge_rxf_dump_1e(dp, (uint8_t *)dp->io_area + (io_offset - 64), 128);

	/* XXX - terminate gem_receive() immediately */
	lp->need_to_reset = B_TRUE;
	return (0);

correct_seqnum:

#ifdef CONFIG_CKSUM_OFFLOAD
	if ((rsd1 & (RSD1_IPv4 | RSD1_IPv6)) &&
	    (rsd1 & (RSD1_E_IPCHKSUM | RSD1_E_L4CHKSUM)) == 0) {
		if (rsd1 & RSD1_TCP) {
			ret |= GEM_RX_CKSUM_TCP;
		} else if (rsd1 & RSD1_UDP) {
			ret |= GEM_RX_CKSUM_UDP;
		}
	}
#endif /* CONFIG_CKSUM_OFFLOAD */

	len = (rsd0 & RSD0_LEN) >> RSD0_LEN_SHIFT;
	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}

	return (ret | len);
}

static void
atge_rx_desc_init_1e(struct gem_dev *dp, int slot)
{
	/* do nothing */
}

static void
atge_rx_desc_write_1c(struct gem_dev *dp, int slot,
	ddi_dma_cookie_t *dmacookie, int frags)
{
	uint64_t	dmaaddr;
	struct atge_dev	*lp = dp->private;
	struct rx_free_desc	*rfdp;

#if DEBUG_LEVEL > 2
{
	int	i;

	cmn_err(CE_CONT,
	    "!%s: %s: seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
}
#endif
	/*
	 * write a RX descriptor
	 */
	rfdp = &((struct rx_free_desc *)(dp->io_area + lp->rbd_offset))[slot];

	dmaaddr = dmacookie->dmac_laddress;
	rfdp->rf_addr = LE_64(dmaaddr);
}
#if 0
static void
atge_rx_start_1c(struct gem_dev *dp, int start_slot, int nslot)
{
	int	tail;
	struct atge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	atge_rfd_desc_dma_sync_1c(dp, start_slot, nslot);

	tail = SLOT(start_slot + nslot, dp->gc.gc_rx_ring_size);
	lp->rx_buffer_tail = tail;
	OUTL(dp, MB_RFD_PROD_IDX + 0*4, tail);
}
#else
static void
atge_rx_start_1c(struct gem_dev *dp, int start_slot, int nslot)
{
	int	i;
	int	tail;
	struct rx_status_desc	*rsdp;
	struct atge_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	/* invalidate rx status descriptors */
	for (i = 0; i < nslot; i++) {
		tail = SLOT(start_slot + i, dp->gc.gc_rx_ring_size);
		rsdp = &((struct rx_status_desc *)(dp->rx_ring))[tail];
		rsdp->rsd0 = 0;	/* sanity */
		rsdp->rsd1 = 0;
	}
	gem_rx_desc_dma_sync(dp,
	    SLOT(start_slot, dp->gc.gc_rx_ring_size), nslot,
	    DDI_DMA_SYNC_FORDEV);

	/* sync rx buffer descriptors */
	atge_rfd_desc_dma_sync_1c(dp, start_slot, nslot);

	/* notify to the nic */
	OUTL(dp, MB_RFD_PROD_IDX + 0*4,
	    SLOT(start_slot + nslot, dp->gc.gc_rx_ring_size));
}
#endif

static uint64_t
atge_rx_desc_stat_1c(struct gem_dev *dp, int slot, int ndesc)
{
#ifdef NEVER
	int		retry;
#endif
	int		head;
	uint_t		index;
	uint64_t	len;
	uint64_t	rsd0;
	uint64_t	rsd1;
	uint64_t	ret;
	struct rx_status_desc	*rsdp;
	struct atge_dev	*lp = dp->private;

	ASSERT(ndesc == 1);
	ret = 0;
#ifdef NEVER
	retry = 0;
again:
#endif
	rsdp = &((struct rx_status_desc *)(dp->rx_ring))[slot];

	rsd1 = rsdp->rsd1;
	rsd1 = LE_64(rsd1);

	if ((rsd1 & RRS1_UPDATED) == 0) {
		/* not yet received */
		return (0);
	}

	rsd0 = rsdp->rsd0;
	rsd0 = LE_64(rsd0);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%x, %llx, %llx, flags:%b",
	    dp->name, __func__, slot, rsd0, rsd1,
	    (uint32_t)(rsd1 >> RRS1_PKT_SIZE_SHIFT), RRS1_BITS));

	/*
	 * check fatal errors
	 */
	if (((rsd0 & RRS0_CNT) >> RRS0_CNT_SHIFT) != 1) {
#ifdef NEVER
		if (retry++ < 1) {
			gem_rx_desc_dma_sync(dp,
			    slot, 1, DDI_DMA_SYNC_FORKERNEL);
			goto again;
		}
#endif
		cmn_err(CE_WARN, "!%s: %s: (rsd0(%llx) & RRS0_CNT) != 1",
		    dp->name, __func__, rsd0);
		lp->need_to_reset = B_TRUE;
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	index = (rsd0 & RRS0_INDEX) >> RRS0_INDEX_SHIFT;
	if (slot != index) {
		cmn_err(CE_WARN, "!%s: %s: slot(%d) != index(%d)",
		    dp->name, __func__, slot, index);
		lp->need_to_reset = B_TRUE;
		return (GEM_RX_DONE | GEM_RX_ERR);
	}

	/* now current slot == index in current return descriptor */
	ret = GEM_RX_DONE;
#ifdef NEVER
	/* invalidate the rx status descriptor */
	rsdp->rsd0 = 0;
	rsdp->rsd1 = 0;

	/*
	 * Increase rx_status_head to point the next descriptor,
	 * we don't update MBOX here. It have to be updated later when
	 * new rx buffers are prepared. we also don't use mutex lock,
	 * because an assignment of a 4 byte word is naturally atomic,
	 * the modification is only performed in this function and
	 * intrlock is held.
	 */
	lp->rx_status_head = SLOT(index + 1, dp->gc.gc_rx_ring_size);
#endif

	if (rsd1 & (RRS1_E_SUM | RRS1_LEN_ERR)) {
		ret |= GEM_RX_ERR;
#ifdef NEVER
		if (rsd1 & (RRS1_E_TRUNC | RRS1_E_RUNT |
		    RRS1_E_FAE | RRS1_E_CRC)) {
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: slot:%x, %llx, %llx, flags:%b",
			    dp->name, __func__, slot, rsd0, rsd1,
			    (uint32_t)(rsd1 >> 32), RRS1_BITS));

			dp->stats.errrcv++;
			if (rsd1 & RRS1_E_TRUNC) {
				dp->stats.frame_too_long++;
			} else if (rsd1 & RRS1_E_RUNT) {
				dp->stats.runt++;
			} else if (rsd1 & RRS1_E_FAE) {
				dp->stats.frame++;
			} else if (rsd1 & RRS1_E_CRC) {
				dp->stats.crc++;
			} else {
				dp->stats.rcv_internal_err++;
			}
		}
#endif /* USE_GETSTAT_RX */
		return (ret);
	}


	len = (rsd1 & RRS1_PKT_SIZE) >> RRS1_PKT_SIZE_SHIFT;
	if (len >= ETHERFCSL) {
		len -= ETHERFCSL;
	}

	return (ret | len);
}

static void
atge_rx_desc_init_1c(struct gem_dev *dp, int slot)
{
	struct rx_status_desc	*rsdp;

	rsdp = &((struct rx_status_desc *)(dp->rx_ring))[slot];
	rsdp->rsd0 = 0;	/* sanity */
	rsdp->rsd1 = 0;
}

/*
 * Device depend interrupt handler
 */
static uint_t
atge_interrupt(struct gem_dev *dp)
{
	int		i;
	uint32_t	isr;
	uint32_t	bogus_isr;
	uint_t		uval;
	uint_t		flag = 0;
	struct atge_dev	*lp = dp->private;
	struct intr_status_msg *ismp;

	switch (lp->chip) {
	case CHIP_L1:
#ifdef CONFIG_ISM_L1C
	case CHIP_L1C:
#endif
		atge_ioarea_dma_sync(dp, lp->ism_offset,
		    sizeof (struct intr_status_msg), DDI_DMA_SYNC_FORKERNEL);

		ismp = (struct intr_status_msg *)(dp->io_area + lp->ism_offset);
		isr = ismp->ism_isr;
		isr = LE_32(isr);
		break;

	default:
		isr = INL(dp, ISR);
		break;
	}
	bogus_isr = lp->isr_pended;
	lp->isr_pended = 0;

	if (((isr | bogus_isr) & lp->imr) == 0) {
#ifdef GEM_CONFIG_INTR_MSI
		if ((dp->intr_types &
		    (DDI_INTR_TYPE_MSIX | DDI_INTR_TYPE_MSI)) &&
		    dp->gc.gc_nintrs_req > 0)  {
			/*
			 * IMR doesn't seem to mask interrupts
			 * in INTR_RDCLR mode
			 */
			if (!lp->intr_rdclr) {
				cmn_err(CE_WARN,
				    "!%s: %s: bogus interrupt "
				    "isr:%b bogus:%b isr_pended:%b",
				    dp->name, __func__,
				    isr, lp->isr_bits, bogus_isr, lp->isr_bits,
				    lp->isr_pended, lp->isr_bits);
			}
			return (DDI_INTR_CLAIMED);
		}
#endif
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: Interrupt, time:%d isr: %b ",
	    dp->name, ddi_get_lbolt(), isr, lp->isr_bits));

	switch (lp->chip) {
	case CHIP_L1:
		/* update tx buffer head */
		uval = ismp->ism_tbd;
		lp->tx_buffer_head = LE_16(uval);

		/* update rx buffer tail */
		uval = ismp->ism_rsd;
		lp->rx_status_tail = LE_16(uval);
		break;
	default:
		/* nothing to do */
		break;
	}

	if (!IS_MAC_ONLINE(dp)) {
		/* the device is not active, no more interrupts */
		lp->imr = 0;
		OUTL(dp, IMR, lp->imr);
		FLSHL(dp, IMR);

		/* clear and disable interrupts */
		if (lp->chip == CHIP_L1) {
			ismp->ism_isr = 0;
		}
		OUTL(dp, ISR, isr | lp->isr_disable);
		FLSHL(dp, ISR);

		return (DDI_INTR_CLAIMED);
	}
#ifdef TEST_RESET_ON_ERRROR
	lp->reset_test++;
	if ((lp->reset_test % 10000) == 9999) {
		lp->need_to_reset = B_TRUE;
	}
#endif
	/* clear and disable interrupts */
	if (isr & lp->isr_gphy) {
		/* ack to interrupts from PHY */
		sema_p(&dp->hal_op_lock);
		(void) atge_mii_read_raw(dp, 19);
		sema_v(&dp->hal_op_lock);
	}

	/* XXX - RDCLR mode cannot handle PHY interrupts */
	OUTL(dp, ISR, isr | lp->isr_disable);

#if defined(CONFIG_ADAPTIVE_COALESCE)
	if ((dp->speed == GEM_SPD_1000 || dp->speed == GEM_SPD_100) &&
	    dp->poll_interval != lp->last_poll_interval) {
		uint_t	val;

		val = dp->poll_interval/2000;
		val = max(val, 1);
		val = min(val, 0xffff);

		switch (lp->chip) {
		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			OUTW(dp, INTRDELAY2_INIT, val);
			DPRINTF(5, (CE_CONT, "!%s: %s: INTRDELAY2_INIT:0x%x",
			    dp->name, __func__, INW(dp, INTRDELAY2_INIT)));
			break;
#ifdef notyet
		case CHIP_L2:
			OUTW(dp, INTRDELAY_INIT, val);
			DPRINTF(5, (CE_CONT, "!%s: %s: INTRDELAY_INIT:0x%x",
			    dp->name, __func__, INW(dp, INTRDELAY_INIT)));
			break;
#endif
		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			/* don't change delay for tx */
			OUTW(dp, INTRDELAY_INIT + 2, val);
			break;
		}

		lp->last_poll_interval = dp->poll_interval;
	}
#endif
	switch (lp->chip) {
	case CHIP_L1:
#ifdef CONFIG_ISM_L1C
	case CHIP_L1C:
#endif
		/* XXX - Is clearning isr not required for msi mode ? */
		/*
		 * XXX - the nic might not update isr because interrupts
		 * are disabled now.
		 */
		/*
		 * there will be race conditions between clearing the
		 * isr in the interrupt status message and updating it from
		 * the nic.
		 * But it will not cause serious probems even if the nic
		 * updates isr right before we clear it.
		 * The interrupt reasons we don't serve now, will remain
		 * until the next update from the nic.
		 */
		ismp->ism_isr &= ~isr;
		atge_ioarea_dma_sync(dp, lp->ism_offset,
		    sizeof (struct intr_status_msg), DDI_DMA_SYNC_FORDEV);
	}

	if (!lp->intr_rdclr) {
		FLSHL(dp, ISR);
	}

	/* mask interrupts we are not interrested in */
	isr &= lp->imr;

	if (isr & lp->isr_sm) {
		/* update statistics */
		DPRINTF(4, (CE_CONT,
		    "!%s: statistics full, isr:0x%b", dp->name,
		    isr, lp->isr_bits));
		atge_update_stats(dp);
	}

	if (isr & lp->isr_pcie_err) {
		/* pci-e phy link down */
		cmn_err(CE_WARN,
		    "!%s: pcie error, isr:0x%b", dp->name,
		    isr, lp->isr_bits);
		OUTL(dp, IMR, 0);
		lp->need_to_reset = B_TRUE;
	}

	if (isr & lp->isr_dma_err) {
		/* DMA transfer error happened */
		cmn_err(CE_WARN,
		    "!%s: dma error happened, isr:0x%b",
		    dp->name, isr, lp->isr_bits);
		OUTL(dp, IMR, 0);
		lp->need_to_reset = B_TRUE;
	}

	if (isr & lp->isr_gphy) {
		/* link status has changed */
		DPRINTF(1, (CE_CONT, "!%s: %s: link status has changed",
		    dp->name, __func__));
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			flag |= INTR_RESTART_TX;
		}
#endif
	}

#ifdef GEM3
	if (((isr & lp->isr_tx_ok) &&
	    dp->tx_free_bufs < dp->gc.gc_tx_buf_size / 1) ||
	    (isr & lp->isr_tx_err)) {
		/* reclaim tx descriptors */
		if (isr & lp->isr_tx_err) {
			cmn_err(CE_WARN, "!%s: tx error, isr:%b",
			    dp->name, isr, lp->isr_bits);
		}
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

#endif
	if (isr & (lp->isr_rx_ok | lp->isr_rx_err)) {
		/*
		 * one or more packets were received, or receive error
		 * happened
		 */
#ifdef notdef
		if (isr & lp->isr_rx_err) {
			/* receive error happened */
#if DEBUG_LEVEL > 4
			cmn_err(CE_WARN, "!%s: receive error, isr:0x%b",
			    dp->name, isr, lp->isr_bits);
#endif
			dp->stats.overflow++;
		}
#endif
		(void) gem_receive(dp);
	}

#ifndef GEM3
	if (isr & (lp->isr_tx_ok | lp->isr_tx_err)) {
		/* reclaim tx descriptors */
		if (isr & lp->isr_tx_err) {
			cmn_err(CE_WARN, "!%s: tx error, isr:%b",
			    dp->name, isr, lp->isr_bits);
		}
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

#endif
	if ((isr & lp->isr_fatal_err) || lp->need_to_reset) {
#if DEBUG_LEVEL > 4
		switch (lp->chip) {
		case CHIP_L1:
		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			atge_tx_desc_dump_1_1e(dp);
		}
#endif
		gem_restart_nic(dp, GEM_RESTART_KEEP_BUF);
		flag |= INTR_RESTART_TX;
		lp->need_to_reset = B_FALSE;
	}

	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* enable interrupts again */
		OUTL(dp, ISR, lp->isr_enable);
	}

	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routines
 */
static void
atge_reset_phy(struct gem_dev *dp)	/* L1C */
{
	uint32_t	uval;
	struct atge_dev	*lp = dp->private;

	if (lp->phy_initialized) {
		/* no need to initialize phy twice after power up */
		return;
	}
	lp->phy_initialized = B_TRUE;

	switch (lp->chip) {
	case CHIP_L2:
		OUTW(dp, GPHYC, lp->gphyc | GPHYC_ENABLE);
		delay(drv_usectohz(1000));
		break;

	case CHIP_L1:
	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
		/* reset PHY first */
		OUTW(dp, GPHYC, lp->gphyc);
		delay(drv_usectohz(2000));
		OUTW(dp, GPHYC, lp->gphyc | GPHYC_ENABLE);
		delay(drv_usectohz(2000));
		break;

	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		/* reset PHY first */
		DPRINTF(0, (CE_CONT, "!%s: %s: GPHYC:%x",
		    dp->name, __func__, INL(dp, GPHYC)));
#if 0
		lp->gphyc = INL(dp, GPHYC) & 
		    ~(GPHYC_EXT_RESET_1c | GPHYC_PHY_IDDQ |
		    GPHYC_GATE_25M_EN | GPHYC_PWDOWN_HW | GPHYC_CLS_1c);
		lp->gphyc &= ~GPHYC_GIGA_DIS;
		lp->gphyc |= GPHYC_DEFAULT_1c;
#endif
		OUTL(dp, GPHYC, lp->gphyc);
		FLSHL(dp, 0);
		delay(drv_usectohz(10));
		DPRINTF(0, (CE_CONT, "!%s: %s: GPHYC:%x:%x",
		    dp->name, __func__, lp->gphyc, INL(dp, GPHYC)));

		OUTL(dp, GPHYC, lp->gphyc | GPHYC_EXT_RESET_1c);
		FLSHL(dp, 0);
		delay(drv_usectohz(800));
		break;
	}

	switch (lp->chip) {
	case CHIP_L1E:
	case CHIP_L2E_A:
	case CHIP_L2E_B:
		/* patch for L1E and L2E phys */
		/* eable hibernation mode */
		atge_mii_write_raw(dp, MII_DBG_ADDR, 0xb);
		atge_mii_write_raw(dp, MII_DBG_DATA, 0xbc00);

		/* set Class A/B for all modes */
		atge_mii_write_raw(dp, MII_DBG_ADDR, 0);
		atge_mii_write_raw(dp, MII_DBG_DATA, 0x02ef);

		/* 10B */
		atge_mii_write_raw(dp, MII_DBG_ADDR, 0x12);
		atge_mii_write_raw(dp, MII_DBG_DATA, 0x4c04);

		/* 1000T power */
		atge_mii_write_raw(dp, MII_DBG_ADDR, 0x4);
		atge_mii_write_raw(dp, MII_DBG_DATA, 0x8bbb);

		atge_mii_write_raw(dp, MII_DBG_ADDR, 0x5);
		atge_mii_write_raw(dp, MII_DBG_DATA, 0x2c46);

		delay(drv_usectohz(1000));
		break;

	case CHIP_L2:
		atge_mii_write_raw(dp, MII_DBG_ADDR, 0);
		uval = atge_mii_read_raw(dp, MII_DBG_DATA);
		if (uval & 0x1000) {
			atge_mii_write_raw(dp, MII_DBG_DATA, uval & ~0x1000);
		}
		break;

	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		/* switch clock */
		if (lp->chip == CHIP_L2C_B) {	/* ok */
			/* MIIDBG_CFGLPSPD */
			atge_mii_write_raw(dp, MII_DBG_ADDR, 10);
			uval = atge_mii_read_raw(dp, MII_DBG_DATA);
			atge_mii_write_raw(dp, MII_DBG_DATA,
			    uval & ~0x2000);	/* ~CFGLPSPD_RSTCNT_CLK125SW */
		}
		/* clear bit3 of dbgport 59 to lower voltage */
		if (lp->chip == CHIP_L2C_B || lp->chip == CHIP_L2C_B2) {
			/* VOLT_CTRL */
			atge_mii_write_raw(dp, MII_DBG_ADDR, 59);
			uval = atge_mii_read_raw(dp, MII_DBG_DATA);
			atge_mii_write_raw(dp, MII_DBG_DATA,
			    uval & ~0x0008);	/* VOLT_CTRL_SWLOWEST */
			delay(drv_usectohz(20000));
		}

		/* power saving config */
		/* LEGCYPS */
		if (lp->chip == CHIP_L1D || lp->chip == CHIP_L1D_2_0) {
			uval = 0x129d;
		} else {
			uval = 0x36dd;
		}
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_41);
		atge_mii_write_raw(dp, MII_DBG_DATA, uval);

		/* hib */
		/* SYSMODCTRL */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_4);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    11U << ANA_IECHO_ADJ_3_SHIFT
		    | 11U << ANA_IECHO_ADJ_2_SHIFT
		    | 8U << ANA_IECHO_ADJ_1_SHIFT
		    | 8U << ANA_IECHO_ADJ_0_SHIFT);

		/* disable AZ(EEE) by default */
		if (lp->chip == CHIP_L1D ||
		    lp->chip == CHIP_L1D_2_0 ||
		    lp->chip == CHIP_L2C_B2) {
			OUTL(dp, LPICTRL,
			    INL(dp, LPICTRL) & ~LPICTRL_EN);

			atge_mii_write_ext(dp,
			    MIIEXT_ANEG, MIIEXT_LOCAL_EEEADV, 0);
			atge_mii_write_ext(dp, MIIEXT_PCS, MIIEXT_CLDCTRL3,
			    L2CB_CLDCTRL3);
		}

		/*
		 * additional mii setting for L*C/L*D chipset
		 */
		/* ANACTRL */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_0);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    ANA_RESTART_CAL
		    | 7U << ANA_MANUL_SWICH_ON_SHIFT
		    | ANA_MAN_ENABLE
		    | ANA_SEL_HSP
		    | ANA_EN_HB
		    | ANA_OEN_125M);

		/* SRDSYSMOD */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_5);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    2U << ANA_SERDES_CDR_BW_SHIFT
		    | ANA_MS_PAD_DBG
		    | ANA_SERDES_EN_DEEM
		    | ANA_SERDES_SEL_HSP
		    | ANA_SERDES_EN_PLL
		    | ANA_SERDES_EN_LCKDT);

		/* TST10BTCFG */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_18);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    ANA_LOOP_SEL_10BT
		    | ANA_EN_MASK_TB
		    | ANA_EN_10BT_IDLE
		    | 1U << ANA_INTERVAL_SEL_TIMER_SHIFT);

		/* UNH-IOL test issue, set bit7 */ 
		/* TST100BTCFG */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_54);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    44U << ANA_LONG_CABLE_TH_100_SHIFT
		    | 33U << ANA_SHORT_CABLE_TH_100_SHIFT
		    | ANA_BP_BAD_LINK_ACCUM
		    | ANA_BP_SMALL_BW);

#ifdef notdef
		/* SYSMODCTRL */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_4);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    11U << ANA_IECHO_ADJ_3_SHIFT
		    | 11U << ANA_IECHO_ADJ_2_SHIFT
		    | 8U << ANA_IECHO_ADJ_1_SHIFT
		    | 8U << ANA_IECHO_ADJ_0_SHIFT);

		/* LEGCYPS */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_41);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    atge_mii_read_raw(dp, MII_DBG_DATA)
		    & ~ANA_TOP_PS_EN);

		/* HIBNEG */
		atge_mii_write_raw(dp, MII_DBG_ADDR, MII_ANA_CTRL_11);
		atge_mii_write_raw(dp, MII_DBG_DATA,
		    atge_mii_read_raw(dp, MII_DBG_DATA)
		    & ~ANA_PS_HIB_EN);
#endif
		break;

	default:
		/* do nothing */
		break;
	}

	/* common: enable link change interrupts */
	atge_mii_write_raw(dp, MII_IER, MII_IER_LINKUP | MII_IER_LINKDOWN);
}

static void
atge_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}

static uint32_t
atge_mii_idle(struct gem_dev *dp)
{
	int		i;
	uint32_t	val;
	struct atge_dev	*lp = dp->private;

	for (i = 0; (val = INL(dp, MIIC)) & (MIIC_GO | MIIC_BUSY); i++) {
		if (i > 120) {
			cmn_err(CE_WARN, "%s: %s: timeout",
			    dp->name, __func__);
			break;
		}
		drv_usecwait(20);
	}

	return (val);
}

static uint16_t
atge_mii_read_core(struct gem_dev *dp, uint_t phyid, uint_t reg)
{
	uint32_t	val;
	uint_t		clk;
	struct atge_dev	*lp = dp->private;

	DPRINTF(3, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	clk = MIIC_CLK_4;

	switch (lp->chip) {
	case CHIP_L2C_B2:
	case CHIP_L1D_2_0:
		if (dp->mii_state != MII_STATE_LINKUP) {
			clk = MIIC_CLK_128;
		}
		break;
	}

	if (phyid == 0) {
		/* internal PHY */
		OUTL(dp, MIIC, clk | MIIC_GO | MIIC_SUPR | MIIC_READ
		    | reg << MIIC_ADDR_SHIFT);
	} else {
		/* extention registers */
		OUTL(dp, MIICEXT,
		    phyid << MIICEXT_DEVAD_SHIFT
		    | reg << MIICEXT_REG_SHIFT);
		OUTL(dp, MIIC, clk | MIIC_GO | MIIC_SUPR | MIIC_READ
		    | MIIC_EXT);
	}

	val = atge_mii_idle(dp);
	if (val & (MIIC_GO | MIIC_BUSY)) {
		/* timeout */
		val = 0;
	}

	DPRINTF(3, (CE_CONT, "!%s: %s: reg:0x%x, val:0x%x",
	    dp->name, __func__, reg, val & MIIC_DATA));

	return (val & MIIC_DATA);
}

static uint16_t
atge_mii_read_raw(struct gem_dev *dp, uint_t reg)
{
	return (atge_mii_read_core(dp, 0, reg));
}

static uint16_t
atge_mii_read_ext(struct gem_dev *dp, uint_t phyid, uint_t reg)
{
	return (atge_mii_read_core(dp, phyid, reg));
}

static uint16_t
atge_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	val;
	struct atge_dev	*lp = dp->private;

	val = atge_mii_read_raw(dp, reg);

	switch (reg) {
	case MII_STATUS:
		switch (lp->chip) {
		case CHIP_L2:
		case CHIP_L2E_B:
			/* mask XSTATUS bit in MII_STATUS register */
			val &= ~MII_STATUS_XSTATUS;
			break;
		}
		break;

	case MII_XSTATUS:
		switch (lp->chip) {
		case CHIP_L2E_A:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
			/* actually no giga bit capability */
			val = 0;
			break;
		}
		break;
	}

	return (val);
}

static void
atge_mii_write_core(struct gem_dev *dp, uint_t phyid, uint_t reg, uint16_t val)
{
	uint_t	clk;
	struct atge_dev	*lp = dp->private;

	DPRINTF(3, (CE_CONT, "!%s: %s: phyid:0x%x, reg:0x%x, val:0x%x",
	    dp->name, __func__, phyid, reg, val));

	clk = MIIC_CLK_4;

	switch (lp->chip) {
	case CHIP_L2C_B2:
	case CHIP_L1D_2_0:
		if (dp->mii_state != MII_STATE_LINKUP) {
			clk = MIIC_CLK_128;
		}
		break;
	}

	if (phyid == 0) {
		/* internal phy */
		OUTL(dp, MIIC, clk | MIIC_GO | MIIC_SUPR
		    | reg << MIIC_ADDR_SHIFT | val);
	} else {
		/* extention registers */
		OUTL(dp, MIICEXT,
		    phyid << MIICEXT_DEVAD_SHIFT
		    | reg << MIICEXT_REG_SHIFT);
		OUTL(dp, MIIC, clk | MIIC_GO | MIIC_SUPR
		    | MIIC_EXT | val);
	}

	(void)atge_mii_idle(dp);
}

static void
atge_mii_write_raw(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	atge_mii_write_core(dp, 0, reg, val);
}

static void
atge_mii_write_ext(struct gem_dev *dp, uint_t phyid, uint_t reg, uint16_t val)
{
	atge_mii_write_core(dp, phyid, reg, val);
}

static void
atge_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	struct atge_dev	*lp = dp->private;

	DPRINTF(3, (CE_CONT, "!%s: %s: reg:0x%x, val:0x%x",
	    dp->name, __func__, reg, val));

	if (reg == MII_CONTROL && (val & MII_CONTROL_RESET)) {
		/* reset PHY externally once to program phy */
		atge_reset_phy(dp);

		/*
		 * don't issue a reset here, otherwise it occationally
		 * advertises incorrect capability
		 */
	} else if (reg == MII_CONTROL && (val & MII_CONTROL_RSAN)) {
		/*
		 * We must issue AUTO-NEG with reset, otherwise
		 * it doesn't advertise the correct capability.
		 */
		atge_mii_write_raw(dp, reg, val | MII_CONTROL_RESET);

	} else if (reg == MII_CONTROL) {
		/*
		 * Avoid to write MII_CONTROL register without RESET bit
		 */

	} else {
		atge_mii_write_raw(dp, reg, val);
	}
}

static int
atge_mii_init(struct gem_dev *dp)
{
	uint_t	uval;
	struct atge_dev	*lp = dp->private;

	sema_p(&dp->hal_op_lock);
	atge_reset_phy(dp);

	if (lp->chip == CHIP_L1) {
		/* exit power saving mode */
		atge_mii_write_raw(dp, 29, 0x0029);
		atge_mii_write_raw(dp, 30, 0x0000);
	}

	atge_mii_tune_phy(dp);

	sema_v(&dp->hal_op_lock);

	return (GEM_SUCCESS);
}

static int
atge_mii_probe(struct gem_dev *dp)
{
	sema_p(&dp->hal_op_lock);
	atge_reset_phy(dp);
	dp->mii_phy_addr = -1;
	sema_v(&dp->hal_op_lock);

	return (gem_mii_probe_default(dp));
}

static int
atge_mii_config(struct gem_dev *dp)
{
	int	ret;

	ret = gem_mii_config_default(dp);

	if (!dp->anadv_autoneg) {
		uint16_t	val;
		/*
		 * write specified mode to phy.
		 */
		val = gem_mii_read(dp, MII_CONTROL) &
		    ~(MII_CONTROL_SPEED | MII_CONTROL_FDUPLEX |
		    MII_CONTROL_ANE | MII_CONTROL_RSAN);

		if (dp->full_duplex) {
			val |= MII_CONTROL_FDUPLEX;
		}

		switch (dp->speed) {
		case GEM_SPD_1000:
			val |= MII_CONTROL_1000MB;
			break;

		case GEM_SPD_100:
			val |= MII_CONTROL_100MB;
			break;

		default:
			cmn_err(CE_WARN, "%s: unknown speed:%d",
			    dp->name, dp->speed);
			/* FALLTHROUGH */
		case GEM_SPD_10:
			/* for GEM_SPD_10, do nothing */
			break;
		}

		sema_p(&dp->hal_op_lock);
		atge_mii_write_raw(dp, MII_CONTROL, val | MII_CONTROL_RESET);
		sema_v(&dp->hal_op_lock);
	}

	return (ret);
}

static void
atge_mii_tune_phy(struct gem_dev *dp)
{
	boolean_t	adj_thresh;
	uint_t		phy_val;
	struct atge_dev	*lp = dp->private;

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:

		switch (lp->chip) {
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			adj_thresh = B_TRUE;
			break;

		default:
			adj_thresh = B_FALSE;
			break;
		}

		if (dp->mii_state == MII_STATE_LINKUP) {
			/* link up */
			atge_set_aspm_1c(dp, lp->ctrl_flags);
			/* az with brcm, half-amp */
			if (lp->chip == CHIP_L1D_2_0) {
				phy_val =  atge_mii_read_ext(dp,
				   MIIEXT_PCS, MIIEXT_CLDCTRL6);

				phy_val &= CLDCTRL6_CAB_LEN;
				phy_val >>= CLDCTRL6_CAB_LEN_SHIFT;

				phy_val = phy_val > CLDCTRL6_CAB_LEN_SHORT ?
					AZ_ANADECT_LONG : AZ_ANADECT_DEF;
				atge_mii_write_raw(dp,
				    MII_DBG_ADDR, MIIDBG_AZ_ANADECT);
				atge_mii_write_raw(dp,
				    MII_DBG_DATA, phy_val);
			}

			/* threshold adjust */
			if (adj_thresh && dp->speed == GEM_SPD_100 &&
			   lp->msi_lnkpatch) {
				atge_mii_write_raw(dp,
				    MII_DBG_ADDR, MIIDBG_MSE16DB);
				atge_mii_write_raw(dp,
				    MII_DBG_DATA, L1D_MSE16DB_UP);
				atge_mii_write_raw(dp,
				    MII_DBG_ADDR, MIIDBG_SYSMODCTRL);
				atge_mii_write_raw(dp,
				    MII_DBG_DATA, L1D_SYSMODCTRL_IECHOADJ_DEF);
			}
		} else {
			/* link down */
			atge_set_aspm_1c(dp, 0);
			if (adj_thresh && lp->msi_lnkpatch) {
				atge_mii_write_raw(dp,
				    MII_DBG_ADDR, MIIDBG_SYSMODCTRL);
				atge_mii_write_raw(dp,
				    MII_DBG_DATA, SYSMODCTRL_IECHOADJ_DEF);
				atge_mii_write_raw(dp,
				    MII_DBG_ADDR, MIIDBG_MSE16DB);
				atge_mii_write_raw(dp,
				    MII_DBG_DATA, L1D_MSE16DB_DOWN);
			}
		}
	}
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static void
atge_power_up(dev_info_t *dip, ddi_acc_handle_t	conf_handle)
{
	/* ensure we can access registers through IO space. */
	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    (pci_config_get16(conf_handle, PCI_CONF_COMM)
	    | PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE)
	    & ~PCI_COMM_INTX_DISABLE);

	/* ensure PME is clear */
	pci_config_put32(conf_handle, PM_CTRLSTAT, 0);
	delay(drv_usectohz(1000));

	/* ensure pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);
}

#define	WOL_MAG		0x1
#define	WOL_LNKC	0x2

static void
atge_power_saving_1c(struct gem_dev *dp, uint32_t wolflag)
{
	uint32_t	mstc;
	uint32_t	macctl;
	uint32_t	gphyc;
	uint32_t	wolctl;
	uint32_t	speed;
	uint16_t	phy_data;
	struct atge_dev	*lp = dp->private;

	wolctl = 0;

	mstc = INL(dp, MSTC);
	mstc &= ~MSTC_CLK_SEL_DIS_1c;

	macctl = INL(dp, MACCTL);
	if (dp->mii_state == MII_STATE_LINKUP &&
	    dp->speed == GEM_SPD_1000) {
		speed = MACCTL_SPEED_1000;
	} else {
		speed = MACCTL_SPEED_10_100;
	}
	macctl = (macctl & ~MACCTL_SPEED) | speed;
	macctl &= ~(MACCTL_FD | MACCTL_RX | MACCTL_TX);
	if (dp->full_duplex) {
		macctl |= MACCTL_FD;
	}

	gphyc = INL(dp, GPHYC);
	gphyc &= ~(GPHYC_EXT_RESET_1c | GPHYC_CLS_1c);

	if (wolflag == 0) {
		/* without WoL */
		mstc |= MSTC_CLK_SEL_DIS_1c;
		gphyc |= GPHYC_PHY_IDDQ | GPHYC_PWDOWN_HW;
		OUTL(dp, MSTC, mstc);
		OUTL(dp, MACCTL, macctl);
		OUTL(dp, GPHYC, gphyc);
		OUTL(dp, WOLCTL, 0);
		lp->initialized = B_FALSE;
		return;
	}

	gphyc |= GPHYC_EXT_RESET_1c;
	if (wolflag & WOL_MAG) {
		macctl |= MACCTL_RX | MACCTL_AB;
		wolctl |= WOLCTL_MGC_EN | WOLCTL_MGC_PME_EN;
		if (lp->chip == CHIP_L2C_B && lp->revid == 0xc1) {
			wolctl |= WOLCTL_PAT_EN | WOLCTL_PAT_PME_EN;
		}
	}

	if (wolflag & WOL_LNKC) {
		wolctl |= WOLCTL_LCHG_EN | WOLCTL_LCHG_PME_EN;
		atge_mii_write_raw(dp, MII_IER, MII_IER_LINKUP);
	}

	/* clear PHY interrupt */
	phy_data = atge_mii_read_raw(dp, MII_ISR);

	DPRINTF(0, (CE_CONT,
	    "!%s: suspend MAC:0x%x, MSTC:0x%x, GPHYC:0x%x ,WOLCTL:0x%x",
	    dp->name, macctl, mstc, gphyc, wolctl));

	OUTL(dp, MSTC, mstc);
	OUTL(dp, MACCTL, macctl);
	OUTL(dp, GPHYC, gphyc);
	OUTL(dp, WOLCTL, wolctl);
}

static void
atge_power_down(struct gem_dev *dp)
{
	struct atge_dev	*lp = dp->private;

	switch (lp->chip) {
	case CHIP_L1C:
	case CHIP_L2C:
	case CHIP_L2C_B:
	case CHIP_L2C_B2:
	case CHIP_L1D:
	case CHIP_L1D_2_0:
		atge_set_aspm_1c(dp, 0);
		atge_power_saving_1c(dp, 0);
		break;

	default:
		break;
	}
}

static int
atgeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
#if 0
	int			n;
#endif
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	uint16_t		vid;
	uint16_t		did;
	uint16_t		subvid;
	uint16_t		subsid;
	uint8_t			revid;
	int			unit;
	struct chip_info	*p;
	const char		*drv_name;
	struct gem_dev		*dp;
	struct atge_dev		*lp;
	uint8_t			*base;
	ddi_acc_handle_t	reg_ha;
	struct gem_conf		*gcp;
	uint32_t		ilr;
	offset_t		io_off;
	int			mtu;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(1, (CE_CONT, "!%s%d: %s: called at time:%d",
	    drv_name, unit, __func__, ddi_get_lbolt()));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: pci_config_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	subvid = pci_config_get16(conf_handle, PCI_CONF_SUBVENID);
	subsid = pci_config_get16(conf_handle, PCI_CONF_SUBSYSID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	/* check chip revision and report it */
	for (i = 0, p = atge_chiptbl; i < NITEMS(atge_chiptbl); i++, p++) {
		if (p->venid == vid && p->devid == did) {
			/* found */
			cmn_err(CE_CONT,
			    "!%s%d: %s "
			    "(vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
			    drv_name, unit, p->name, vid, did, revid);
			break;
		}
	}
	if (i >= NITEMS(atge_chiptbl)) {
		/* Not found */
		cmn_err(CE_NOTE,
		    "!%s%d: %s: unknown PCI venid/devid (0x%x, 0x%x)",
		    drv_name, unit, __func__, vid, did);
		/* assume the latest chipset */
		p = &atge_chiptbl[NITEMS(atge_chiptbl) - 1];
	}

	switch (cmd) {
	case DDI_RESUME:
		atge_power_up(dip, conf_handle);

		dp = GEM_GET_DEV(dip);
		lp = dp->private;
		lp->initialized = B_FALSE;
		lp->phy_initialized = B_FALSE;
		return (gem_resume(dip));

	case DDI_ATTACH:
		atge_power_up(dip, conf_handle);

		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_MEM64, PCI_ADDR_MASK,
		    &atge_dev_attr, (void *)&base, &reg_ha)
		    != DDI_SUCCESS) {
			goto err;
		}

		/*
		 * allocate local device structure
		 */
		lp = kmem_zalloc(sizeof (struct atge_dev), KM_SLEEP);
		lp->revid = revid;
		lp->initialized = B_FALSE;
		lp->phy_initialized = B_FALSE;
		lp->need_to_reset = B_FALSE;
		lp->isr_pended = 0;

		/* determin chip type */
#ifdef DEBUG_L1E
		lp->chip = CHIP_L1E;
#else
		lp->chip = p->chip_type;
#endif
		if (p->chip_type == CHIP_L1E) {
			/* fix chip type */
			if (revid >= 0xf0) {
				lp->chip = CHIP_L2E_B;
			} else {
				if (ddi_get32(reg_ha,
				    (void *)(base + PHYSTAT)) & PHYSTAT_100M) {
					lp->chip = CHIP_L1E;
				} else {
					lp->chip = CHIP_L2E_A;
				}
			}
		}

		/* configure initial value for gphyc register */
		switch (lp->chip) {
		case CHIP_L1:
		case CHIP_L2:
			lp->gphyc = 0;
			break;

		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			lp->gphyc = GPHYC_DEFAULT_1e;
			break;

		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			lp->gphyc = GPHYC_DEFAULT_1c;
			break;
		}

		/* configure interrupts status bits */
		mtu = ddi_prop_get_int(DDI_DEV_T_ANY, dip,
		    DDI_PROP_DONTPASS, "mtu", ETHERMTU);
		DPRINTF(4, (CE_CONT, "!%s%d: %s: mtu:%d",
		    drv_name, unit, __func__, mtu));
		switch (lp->chip) {
		case CHIP_L1:
			lp->isr_disable = ISR_DIS_INT_1;
			lp->isr_enable = ISR_DIS_SM_1 | ISR_DIS_DMA_1;
			lp->isr_rx_err =
			    ISR_RXF_OVF_1 | ISR_RBD_UNRUN_1 | ISR_RSD_OVF_1 |
			    ISR_HOST_RBD_UNRUN_1 | ISR_HOST_RSD_OVF_1;
			lp->isr_rx_ok = ISR_ISM_RX_1;
			lp->isr_tx_err = 0;
#ifdef CONFIG_ADAPTIVE_COALESCE
			lp->isr_tx_ok = ISR_TX_DMA_1;
#else
			lp->isr_tx_ok = ISR_ISM_TX_1;
#endif
			lp->isr_dma_err =
			    ISR_DMAR_TO_RST_1 | ISR_DMAW_TO_RST_1;
			lp->isr_gphy = ISR_GPHY_1;
			lp->isr_sm = ISR_SM_1;
			lp->isr_pcie_err = ISR_PCIE_ERR_1;
			lp->isr_bits = ISR_BITS_1;
			lp->isr_fatal_err = 0;
			lp->isr_clear_all = 0x3fffffffU;
			break;

		case CHIP_L2:
			lp->isr_disable = ISR_DIS_INT_2;
			lp->isr_enable = 0;
			lp->isr_rx_err =
			    ISR_RXD_OV_2 | ISR_RXS_OV_2 | ISR_RXF_OVF_2;
			lp->isr_rx_ok = ISR_RX_UPDATE_2;
			lp->isr_tx_err = 0;
			lp->isr_tx_ok = ISR_TX_UPDATE_2;
			lp->isr_dma_err =
			    ISR_DMAR_TO_RST_2 | ISR_DMAW_TO_RST_2;
			lp->isr_gphy = ISR_PHY_2;
			lp->isr_sm = 0;
			lp->isr_pcie_err = ISR_PCIE_ERR_2;
			lp->isr_bits = ISR_BITS_2;
			lp->isr_fatal_err = 0;
			lp->isr_clear_all = 0x3fffffffU;
			break;

		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			lp->isr_disable = ISR_DIS_INT_1e;
			lp->isr_enable = 0;
			lp->isr_rx_err = ISR_RXF_OVF_1e | ISR_RXF0_OV_1e;
			lp->isr_rx_ok = ISR_RX_PKT_1e;
			lp->isr_tx_err = ISR_TXF_UNRUN_1e;
			lp->isr_tx_ok = ISR_TX_PKT_1e;
			lp->isr_dma_err =
			    ISR_DMAR_TO_RST_1e | ISR_DMAW_TO_RST_1e;
			lp->isr_gphy = ISR_GPHY_1e;
			lp->isr_sm = ISR_SM_1e;
			lp->isr_pcie_err = ISR_PCIE_ERR_1e;
			lp->isr_bits = ISR_BITS_1e;
			lp->isr_fatal_err = 0;
			lp->isr_clear_all = 0x7fffffffU;
			break;

		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			lp->isr_disable = ISR_DIS_INT_1c;
			lp->isr_enable = 0;
			lp->isr_rx_err =
			    ISR_RFD0_UR_1c | ISR_RFD1_UR_1c |
			    ISR_RFD2_UR_1c | ISR_RFD3_UR_1c |
			    ISR_HW_RXF_OV_1c;
			lp->isr_rx_ok =
			    ISR_RX_PKT0_1c | ISR_RX_PKT1_1c |
			    ISR_RX_PKT2_1c | ISR_RX_PKT3_1c;
			lp->isr_tx_err = ISR_TXF_UR_1c | ISR_TXQ_TO_RST_1c;
			lp->isr_tx_ok = ISR_TX_PKT_1c;
			lp->isr_dma_err =
			    ISR_DMAR_TO_RST_1c | ISR_DMAW_TO_RST_1c;
			lp->isr_gphy = ISR_GPHY_1c;
			lp->isr_sm = ISR_SMB_1c;
			lp->isr_pcie_err = ISR_PHY_LINKDOWN_1c;
			lp->isr_bits = ISR_BITS_1c;
			lp->isr_fatal_err = 0;
			lp->isr_clear_all = 0x7fffffffU;
			break;
		}

		/*
		 * allocate ioarea
		 */
		io_off = 0;

		switch (lp->chip) {
		case CHIP_L1:
			/* statistics message area */
			lp->sm_offset = io_off = ROUNDUP(io_off, 8);
			io_off += sizeof (uint32_t) * STAT_MAX;
			DPRINTF(0, (CE_CONT, "!%s%d: sm_offset:0x%x",
			    drv_name, unit, lp->sm_offset));

			/* tx buffer descriptors */
			lp->tbd_offset = io_off = ROUNDUP(io_off, 8);
			io_off +=
			    sizeof (struct tx_buffer_desc) * TX_RING_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: tbd_offset:0x%x",
			    drv_name, unit,  lp->tbd_offset));

			/* rx buffer descriptors */
			lp->rbd_offset = io_off = ROUNDUP(io_off, 8);
			io_off +=
			    sizeof (struct rx_buffer_desc) * RX_BUF_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: rbd_offset:0x%x",
			    drv_name, unit,  lp->rbd_offset));

			/* interrupt status message area */
			lp->ism_offset = io_off = ROUNDUP(io_off, 8);
			io_off += sizeof (struct intr_status_msg);
			DPRINTF(0, (CE_CONT, "!%s%d: ism_offset:0x%x",
			    drv_name, unit,  lp->ism_offset));
			break;

		case CHIP_L2:
			/* tx buffer descriptors */
			lp->tbd_offset = io_off = ROUNDUP(io_off, 8);
			lp->tbd_size = 1536 * TX_BUF_SIZE;
			io_off += lp->tbd_size;
			DPRINTF(0, (CE_CONT, "!%s%d: tbd_offset:0x%x",
			    drv_name, unit,  lp->tbd_offset));

			/*
			 * tx descriptors in gem are used for tx status
			 * descriptors
			 */

			/* rx buffer descriptors */
			lp->rbd_offset =
			    io_off = ROUNDUP(io_off + 8, 128) - 8;
			io_off += 1536 * RX_BUF_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: rbd_offset:0x%x",
			    drv_name, unit,  lp->rbd_offset));
			break;

		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			/*
			 * Statistics message are *not* DMAed actually.
			 * But we allocate it in DMA area for compatibility
			 * with L1.
			 */
			lp->sm_offset = io_off = ROUNDUP(io_off, 8);
			io_off += sizeof (uint32_t) * STAT_MAX;
			DPRINTF(0, (CE_CONT, "!%s%d: sm_offset:0x%x",
			    drv_name, unit, lp->sm_offset));

			/* tx buffer descriptors */
			lp->tbd_offset = io_off = ROUNDUP(io_off, 64);
			io_off +=
			    sizeof (struct tx_buffer_desc) * TX_RING_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: tbd_offset:0x%x",
			    drv_name, unit,  lp->tbd_offset));

			/* rx buffer */
			lp->rxf_buf_len = ROUNDUP(
			    sizeof (struct ether_header) + mtu + VTAG_SIZE +
			    ETHERFCSL + sizeof (struct rx_status_desc), 32) *
			    (RX_BUF_SIZE / 2);

			lp->rxf_offset[0][0] = io_off = ROUNDUP(io_off, 32);
			io_off += lp->rxf_buf_len;
			DPRINTF(0, (CE_CONT, "!%s%d: rxf_offset[0][0]:0x%x",
			    drv_name, unit,  lp->rxf_offset[0][0]));

			lp->rxf_offset[0][1] = io_off = ROUNDUP(io_off, 32);
			io_off += lp->rxf_buf_len;
			DPRINTF(0, (CE_CONT, "!%s%d: rxf_offset[0][1]:0x%x",
			    drv_name, unit,  lp->rxf_offset[0][1]));

			/* tx_tail */
			lp->tbd_tail_offset = io_off = ROUNDUP(io_off, 4);
			io_off += 4;
			DPRINTF(0, (CE_CONT, "!%s%d: tbd_tail_offset:0x%x",
			    drv_name, unit,  lp->tbd_tail_offset));

			/* rx mail box */
			lp->rxf_mb_offset[0][0] = io_off = ROUNDUP(io_off, 4);
			io_off += 4;
			DPRINTF(0, (CE_CONT,
			    "!%s%d: rxf_mb_offset[0][0]:0x%x",
			    drv_name, unit,  lp->rxf_mb_offset[0][0]));

			lp->rxf_mb_offset[0][1] = io_off = ROUNDUP(io_off, 4);
			io_off += 4;
			DPRINTF(0, (CE_CONT,
			    "!%s%d: rxf_mb_offset[0][1]:0x%x",
			    drv_name, unit,  lp->rxf_mb_offset[0][1]));
			break;

		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			/* dma alignment seems to be 16byte, not 8 byte */
			/* statistics message area */
			lp->sm_offset = io_off = ROUNDUP(io_off, 16);
			io_off += sizeof (uint32_t) * STAT_MAX;
			DPRINTF(0, (CE_CONT, "!%s%d: sm_offset:0x%x",
			    drv_name, unit, lp->sm_offset));

			/* tx buffer descriptors */
			lp->tbd_offset = io_off = ROUNDUP(io_off, 16);
			io_off +=
			    sizeof (struct tx_buffer_desc) * TX_RING_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: tbd_offset:0x%x",
			    drv_name, unit,  lp->tbd_offset));

			/* rx free buffer descriptors */
			lp->rbd_offset = io_off = ROUNDUP(io_off, 16);
			io_off +=
			    sizeof (struct rx_free_desc) * RX_BUF_SIZE;
			DPRINTF(0, (CE_CONT, "!%s%d: rbd_offset:0x%x",
			    drv_name, unit,  lp->rbd_offset));

			/* interrupt status message area */
			lp->ism_offset = io_off = ROUNDUP(io_off, 16);
			io_off += sizeof (struct intr_status_msg);
			DPRINTF(0, (CE_CONT, "!%s%d: ism_offset:0x%x",
			    drv_name, unit,  lp->ism_offset));
		}
		/* round up io area to the next 8 byte boundary */
		io_off = ROUNDUP(io_off, 8);

		/*
		 * construct gem configuration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_auto_pad = B_TRUE;

		/* we use rx desc area for rx return descriptors */
		gcp->gc_rx_buf_align = sizeof (uint64_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = RX_BUF_SIZE - 1;

		gcp->gc_io_area_size = io_off;
		gcp->gc_hck_rx_start = sizeof (struct ether_header);

		/* map attributes */
		gcp->gc_dev_attr = atge_dev_attr;
		gcp->gc_buf_attr = atge_buf_attr;
		gcp->gc_desc_attr = atge_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = atge_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = atge_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		gcp->gc_dma_attr_rxbuf = atge_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* timeout parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
#if 0
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;
#else
		gcp->gc_flow_control = FLOW_CONTROL_NONE;
#endif

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout = 45 * (ONESEC / 10);
		gcp->gc_mii_an_wait = ONESEC / 4;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;
		gcp->gc_mii_hw_link_detection = B_TRUE;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
#ifdef CONFIG_DONT_RESET_PHY	/* not defined */
		gcp->gc_mii_dont_reset = B_TRUE;
#else
		gcp->gc_mii_dont_reset = B_FALSE;
#endif

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &atge_attach_chip;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &atge_fixup_params;
#endif
		gcp->gc_reset_chip = &atge_reset_chip;
		gcp->gc_start_chip = &atge_start_chip;
		gcp->gc_stop_chip = &atge_stop_chip;
		gcp->gc_multicast_hash = &atge_mcast_hash;
		gcp->gc_set_rx_filter = &atge_set_rx_filter;
		gcp->gc_set_media = &atge_set_media;
		gcp->gc_get_stats = &atge_get_stats;
		gcp->gc_interrupt = &atge_interrupt;

		/* mii operations */
		gcp->gc_mii_probe = &atge_mii_probe;
		gcp->gc_mii_init = &atge_mii_init;
		gcp->gc_mii_config = &atge_mii_config;
		gcp->gc_mii_sync = &atge_mii_sync;
		gcp->gc_mii_read = &atge_mii_read;
		gcp->gc_mii_write = &atge_mii_write;
		gcp->gc_mii_tune_phy = &atge_mii_tune_phy;
		gcp->gc_mii_stop_mac_on_linkdown = B_FALSE;

		gcp->gc_hck_rx_start = sizeof (struct ether_header) + 20;

		/* enable MSI/MSIX interrupts */
		gcp->gc_nintrs_req = 1;

		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		/* descriptor operation */
		switch (lp->chip) {
		case CHIP_L1:
			gcp->gc_tx_max_frags = MAXTXFRAGS;
#ifdef CONFIG_LSO
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
#else
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
#endif
			gcp->gc_tx_buf_size = TX_BUF_SIZE;
			gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
			gcp->gc_tx_ring_size = TX_RING_SIZE;
			gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;

			/* tx: direct buffer */
			gcp->gc_tx_desc_unit_shift = -1;
			gcp->gc_tx_copy_thresh = atge_tx_copy_thresh;

			/* rx: direct buffer */
			gcp->gc_rx_desc_unit_shift = 4; /* desc */
			gcp->gc_rx_copy_thresh = atge_rx_copy_thresh;

			gcp->gc_init_chip = &atge_init_chip_1;

			gcp->gc_tx_desc_write = &atge_tx_desc_write_1_1e;
			gcp->gc_tx_start = &atge_tx_start_1_1e;
#ifdef GEM_CONFIG_TX_HEAD_PTR
			gcp->gc_tx_desc_stat = NULL;
			gcp->gc_tx_desc_head = &atge_tx_desc_head_1;
#else
			gcp->gc_tx_desc_stat = &atge_tx_desc_stat_1;
#endif /* GEM_CONFIG_TX_HEAD_PTR */
			gcp->gc_tx_desc_init = &atge_tx_desc_init_1_1e;
			gcp->gc_tx_desc_clean = &atge_tx_desc_init_1_1e;
			gcp->gc_rx_desc_write = &atge_rx_desc_write_1;
			gcp->gc_rx_start = &atge_rx_start_1;
			gcp->gc_get_packet = &gem_get_packet_default;
			gcp->gc_rx_desc_stat = &atge_rx_desc_stat_1;
			gcp->gc_rx_desc_init = &atge_rx_desc_init_1;
			gcp->gc_rx_desc_clean = &atge_rx_desc_init_1;

			gcp->gc_max_lso = 16 * 1024 - 1;
			gcp->gc_max_mtu =
			    10*1024 - sizeof (struct ether_header);
			break;

		case CHIP_L2:
			gcp->gc_tx_max_frags = 1;
			gcp->gc_tx_max_descs_per_pkt = 1;

			gcp->gc_tx_buf_size = TX_BUF_SIZE;
			gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
			gcp->gc_tx_ring_size = TX_BUF_SIZE;
			gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;

			/* tx: variable length ring with status descriptors */
			gcp->gc_tx_desc_unit_shift = 2;	/* 4byte */
			gcp->gc_tx_copy_thresh = INT32_MAX;

			/* rx: fixed length record ring */
			gcp->gc_rx_desc_unit_shift = -1;
			gcp->gc_rx_copy_thresh = INT32_MAX;

			gcp->gc_init_chip = &atge_init_chip_2;

			gcp->gc_tx_desc_write = &atge_tx_desc_write_2;
			gcp->gc_tx_start = &atge_tx_start_2;
			gcp->gc_tx_desc_stat = &atge_tx_desc_stat_2;
			gcp->gc_tx_desc_init = &atge_tx_desc_init_2;
			gcp->gc_tx_desc_clean = &atge_tx_desc_init_2;

			gcp->gc_rx_desc_write = &atge_rx_desc_write_2;
			gcp->gc_rx_start = &atge_rx_start_2;
			gcp->gc_get_packet = &atge_get_packet_2;
			gcp->gc_rx_desc_stat = &atge_rx_desc_stat_2;
			gcp->gc_rx_desc_init = &atge_rx_desc_init_2;
			gcp->gc_rx_desc_clean = &atge_rx_desc_init_2;

			gcp->gc_max_mtu = ETHERMTU;
			break;

		case CHIP_L1E:
		case CHIP_L2E_A:
		case CHIP_L2E_B:
			gcp->gc_tx_max_frags = MAXTXFRAGS;
#ifdef CONFIG_LSO
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
#else
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
#endif
			gcp->gc_tx_buf_size = TX_BUF_SIZE;
			gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
			gcp->gc_tx_ring_size = TX_RING_SIZE;
			gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;

			/* tx: direct buffer */
			gcp->gc_tx_desc_unit_shift = -1; /* separate desc */
			gcp->gc_tx_copy_thresh = atge_tx_copy_thresh;

			/* rx: variable length record ring */
			gcp->gc_rx_desc_unit_shift = -1; /* ring */
			gcp->gc_rx_copy_thresh = INT32_MAX; /* ring */

			gcp->gc_init_chip = &atge_init_chip_1e;

			gcp->gc_tx_desc_write = &atge_tx_desc_write_1_1e;
			gcp->gc_tx_start = &atge_tx_start_1_1e;
#ifdef GEM_CONFIG_TX_HEAD_PTR
			gcp->gc_tx_desc_stat = NULL;
			gcp->gc_tx_desc_head = &atge_tx_desc_head_1e;
#else
			gcp->gc_tx_desc_stat = &atge_tx_desc_stat_1e;
#endif
			gcp->gc_tx_desc_init = &atge_tx_desc_init_1_1e;
			gcp->gc_tx_desc_clean = &atge_tx_desc_init_1_1e;
			gcp->gc_rx_desc_write = &atge_rx_desc_write_1e;
			gcp->gc_rx_start = NULL;
			gcp->gc_get_packet = &atge_get_packet_1e;
			gcp->gc_rx_desc_stat = &atge_rx_desc_stat_1e;
			gcp->gc_rx_desc_init = &atge_rx_desc_init_1e;
			gcp->gc_rx_desc_clean = &atge_rx_desc_init_1e;

			gcp->gc_max_lso = 16 * 1024 - 1;
			gcp->gc_max_mtu =
			    0x2000 - sizeof (struct ether_header);
			break;

		case CHIP_L1C:
		case CHIP_L2C:
		case CHIP_L2C_B:
		case CHIP_L2C_B2:
		case CHIP_L1D:
		case CHIP_L1D_2_0:
			for (i = 0; i < NITEMS(patch_list); i++) {
				struct patch_info	*p;

				p = &patch_list[i];
				if (p->devid == did &&
				    p->revid == revid &&
				    p->subvenid == subvid &&
				    p->subsysid == subsid) {
					/* found */
					lp->msi_lnkpatch = B_TRUE;
					break;
				}
			}
			gcp->gc_tx_max_frags = MAXTXFRAGS;
#ifdef CONFIG_LSO
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
#else
			gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
#endif
			gcp->gc_tx_buf_size = TX_BUF_SIZE;
			gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
			gcp->gc_tx_ring_size = TX_RING_SIZE;
			gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;

			/* tx: direct buffer */
			gcp->gc_tx_desc_unit_shift = -1;
			gcp->gc_tx_copy_thresh = atge_tx_copy_thresh;

			/* rx: direct buffer */
			gcp->gc_rx_desc_unit_shift = 4; /* desc */
			gcp->gc_rx_copy_thresh = atge_rx_copy_thresh;

			gcp->gc_init_chip = &atge_init_chip_1c;

			gcp->gc_tx_desc_write = &atge_tx_desc_write_1c;
			gcp->gc_tx_start = &atge_tx_start_1c;
#ifdef GEM_CONFIG_TX_HEAD_PTR
			gcp->gc_tx_desc_stat = NULL;
			gcp->gc_tx_desc_head = &atge_tx_desc_head_1c;
#else
			gcp->gc_tx_desc_stat = &atge_tx_desc_stat_1c;
#endif
			gcp->gc_tx_desc_init = &atge_tx_desc_init_1c;
			gcp->gc_tx_desc_clean = &atge_tx_desc_init_1c;
			gcp->gc_rx_desc_write = &atge_rx_desc_write_1c;
			gcp->gc_rx_start = &atge_rx_start_1c;
			gcp->gc_get_packet = &gem_get_packet_default;
			gcp->gc_rx_desc_stat = &atge_rx_desc_stat_1c;
			gcp->gc_rx_desc_init = &atge_rx_desc_init_1c;
			gcp->gc_rx_desc_clean = &atge_rx_desc_init_1c;

			gcp->gc_max_lso = 64 * 1024 - 1;
			gcp->gc_max_mtu =
			    6*1024 - sizeof (struct ether_header);

			gcp->gc_mii_stop_mac_on_linkdown = B_TRUE;
			break;
		}
		DPRINTF(-1, (CE_CONT,
		    "!%s: tx buf size:%d limit:%d, ring size:%d limit:%d",
		    __func__, gcp->gc_tx_buf_size, gcp->gc_tx_buf_limit,
		    gcp->gc_tx_ring_size, gcp->gc_tx_ring_limit));

		dp = gem_do_attach(dip, 0,
		    gcp, base, &reg_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));
		if (dp == NULL) {
			goto  err_free_mem;
		}

		pci_config_teardown(&conf_handle);
		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof (struct atge_dev));
	}
err:
	pci_config_teardown(&conf_handle);
	return (DDI_FAILURE);
}

static int
atgedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	uint8_t	*mac;
	struct gem_dev  *dp = GEM_GET_DEV(dip);
	struct atge_dev  *lp = dp->private;

	switch (cmd) {
	case DDI_DETACH:
		/* restore factory mac address */
		atge_power_down(dp);
		mac = dp->dev_addr.ether_addr_octet;
		OUTL(dp, MACADDR,
		    mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
		OUTL(dp, MACADDR + 4,
		    mac[0] << 8 | mac[1]);

		DPRINTF(0, (CE_NOTE,
		    "!%s: factory mac address %x:%x:%x:%x:%x:%x is restored.",
		    dp->name,
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
		mutex_destroy(&lp->mbox_lock);
		return (gem_do_detach(dip));

	case DDI_SUSPEND:
		atge_power_down(dp);
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
GEM_STREAM_OPS(atge_ops, atgeattach, atgedetach);
#else
static	struct module_info atgeminfo = {
	0,			/* mi_idnum */
	"atge",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64 * 1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit atgerinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&atgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit atgewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&atgeminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	atge_info = {
	&atgerinit,	/* st_rdinit */
	&atgewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_atge_ops = {
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
	&atge_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops atge_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	atgeattach,	/* devo_attach */
	atgedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_atge_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce,	/* devo_quiesce */
#endif
};
#endif
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&atge_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!atge: _init: called"));
	status = gem_mod_init(&atge_ops, "atge");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&atge_ops);
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

	DPRINTF(2, (CE_CONT, "!atge: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&atge_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
