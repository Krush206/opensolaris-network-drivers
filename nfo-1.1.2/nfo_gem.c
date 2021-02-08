/*
 * nfo_gem.c: nforce ethernet mac driver
 *
 * Copyright (c) 2005-2006 Masayuki Murayama.  All rights reserved.
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
#pragma	ident	"@(#)nfo_gem.c	1.6 06/09/11"

/*
 Change log
 04/11/2006 sometimes autonegotiation results wrong mode after loading
	       (now gc_mii_dont_reset fixed is TRUE)
            tcp_checksum_mask removed
            force_mode fixed
	    nfoattach didn't return DDI_FAILURE on error
	    release 0.8.3
 */

/*
 TODO:
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

/* XXX - macro *u* in nvenet header file conflicts with solaris u definition */
#undef u

#include "basetype.h"
#include "os.h"
#include "drvinfo.h"
#include "adapter.h"

char	ident[] = "nVIDIA nForce nic driver v" VERSION;
char	_depends_on[] = {"misc/gld"};

/* Debugging support */
#ifdef DEBUG_LEVEL
static int nfo_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (nfo_debug>(n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#ifndef FALSE
#  define	FALSE	(0)
#endif
#ifndef TRUE
#  define	TRUE	(!FALSE)
#endif
#define	ONESEC	drv_usectohz(1*1000000)
#define	ARRAY_SIZE(a)	(sizeof(a)/sizeof(a[0]))

#ifdef MAP_MEM
#define	FLSHB(dp,reg)		INB(dp, reg)
#define	FLSHW(dp,reg)		INW(dp, reg)
#define	FLSHL(dp,reg)		INL(dp, reg)
#else /* MAP_MEM */
#define	FLSHB(dp,reg)
#define	FLSHW(dp,reg)
#define	FLSHL(dp,reg)
#endif /* MAP_MEM */

/* workaround for implicit use of memset/memcpy in gcc structual assignment */
#define	STRUCT_COPY(a, b)	bcopy(&(b), &(a), sizeof(a))

/*
 * Our configuration
 */
#define TX_BUF_SIZE	64
#define TX_RING_SIZE	(TX_BUF_SIZE * 4)
#define RX_RING_SIZE	64

static int	nfo_tx_copy_thresh = 256;
static int	nfo_rx_copy_thresh = 256;

/*
 * nForce GbE private data structures
 */

/* tx buffer management */
struct nfo_tx_desc {
	TX_INFO_ADAP		TxInfoAdap;	/* for use of nvenetlib */
	struct nfo_tx_desc	*Next;		/* for use of nvenetlib */
	PVOID			pvID;		/* link to txbuf in gem */
#ifdef notdef
	PVOID			buf;
	long			status;
	long			entries;
	ddi_dma_cookie_t	dc[64];
#else
	uint32_t		status;		/* transmit status */
#endif
};

/* memory management for nvenetlib */
struct nfo_mem_head {
	ddi_dma_handle_t	nmh_dma_handle;
	ddi_acc_handle_t	nmh_acc_handle;
	caddr_t			nmh_addr;
	size_t			nmh_len;
};

/* supported chipsets */
struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	char		*name;
	int		hwmode;
#define			HWMODE1	1
#define			HWMODE2	2
};


/* allocated rx buffer link */
struct dlink {
	struct rxbuf	*dl_next;
	struct rxbuf	*dl_prev;
};

/* private driver state */
struct nfo_dev {
	uint8_t			mac_addr[ETHERADDRL];
	uint8_t			factory_mac_addr[ETHERADDRL];

	int			forced_speed;
	int			force_duplex;
	int			force_mode;

	ADAPTER_API		*pADApi;
	OS_API			sol_api;

	kmutex_t		phylock;

	struct chip_info	*chip;

	int			intr_tx_done;
	int			tx_list_len;
	struct nfo_tx_desc	tx_desc[TX_RING_SIZE];

	/*
	 * XXX - As it leaks rx buffers after resetting it by a sequence of
	 * calling a pair of pfnDeinit and pfnInit, we need to trace
	 * allocated rx buffers by ourselves.
	 */
	struct rxbuf		rb_act;
#define		RBACT(rbp)	((struct dlink *) &(rbp)->rxb_dmacookie[1])
	boolean_t		drain;

#ifdef DEBUG_LEVEL
	seqnum_t		rx_desc_head;
	seqnum_t		rx_desc_tail;
#endif
	/* statstics support */
	clock_t			last_stats_time;
};

/*
 * Macros to identify chip generation.
 */
static struct chip_info nfo_chiptbl[] = {
	0x10de,	0x01c3,	"nForce2 MCP",		HWMODE1,
	0x10de,	0x0066,	"nForce2 MCP-T",		HWMODE1,
	0x10de,	0x00d6,	"nForce3 MCP3",		HWMODE1,
	0x10de,	0x0086,	"nForce3 mac type 4",	HWMODE2,
	0x10de,	0x008c,	"nForce3 mac type 5",	HWMODE2,
	0x10de,	0x00e6,	"nForce3 mac type 6",	HWMODE2,
	0x10de,	0x00df,	"nForce3 mac type 7",	HWMODE2,
	0x10de,	0x0056,	"nForce4 mac type 8 (CK804)",	HWMODE2,
	0x10de,	0x0057,	"nForce4 CK8-04 Ultra",	HWMODE2,
	0x10de,	0x0037,	"nForce mac type 10 (MCP04)",	HWMODE2,
	0x10de,	0x0038,	"nForce mac type 11 (MCP04)",	HWMODE2,
	0x10de,	0x0268,	"nForce mac type 12 (MCP51)",	HWMODE1,
	0x10de,	0x0269,	"nForce4 410/430 (MCP51)",	HWMODE1,
	0x10de,	0x0372,	"nForce mac type 14 (MCP55)",	HWMODE2,
	0x10de,	0x0373,	"nForce mac type 15 (MCP55)",	HWMODE2,
};
#define CHIPTABLESIZE   (sizeof(nfo_chiptbl)/sizeof(struct chip_info))

static char *nfo_adapter_error_names[] = {
	"ADAPTERERR_NONE", 			/* 0 */
	"ADAPTERERR_COULD_NOT_ALLOC_CONTEXT",	/* 1 */
	"ADAPTERERR_COULD_NOT_CREATE_CONTEXT",	/* 2 */
	"ADAPTERERR_COULD_NOT_OPEN_PHY",	/* 3 */
	"ADAPTERERR_TRANSMIT_QUEUE_FULL",	/* 4 */
	"ADAPTERERR_COULD_NOT_INIT_PHY",	/* 5 */
	"ADAPTERERR_PHYS_SIZE_SMALL",		/* 6 */
	"ADAPTERERR_ERROR",			/* 7 */
	"undocumented",				/* 8 */
};

/* ======================================================== */

/* local functions */ 
static int nfo_create_adapter_object(struct gem_dev *dp);
static int nfo_destroy_adapter_object(struct gem_dev *dp);

/* mii operations */
static void  nfo_mii_sync(struct gem_dev *);
static uint16_t  nfo_mii_read(struct gem_dev *, uint_t);
static void nfo_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int nfo_reset_chip(struct gem_dev *);
static void nfo_init_chip(struct gem_dev *);
static void nfo_start_chip(struct gem_dev *);
static int nfo_stop_chip(struct gem_dev *);
static void nfo_set_media(struct gem_dev *);
static void nfo_set_rx_filter(struct gem_dev *);
static void nfo_get_stats(struct gem_dev *);
static int nfo_init_mac_addr(struct gem_dev *);

/* descriptor operations */
static int nfo_tx_desc_write(struct gem_dev *dp, uint_t slot,
		    ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq);
static uint_t nfo_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void nfo_tx_desc_init(struct gem_dev *dp, int slot);
static void nfo_tx_desc_clean(struct gem_dev *dp, int slot);

/* interrupt handler */
static u_int nfo_interrupt(struct gem_dev *dp);

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

static ddi_dma_attr_t nfo_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0x0000ffffull,		/* dma_attr_count_max */
	0,/* patched later */	/* dma_attr_align */
	0x0000ffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0,/* patched later */	/* dma_attr_sgllen */
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


/*******************************************************************/
/*
 *	OS API for nvenetlib.o
 */
/*******************************************************************/

static NV_API_CALL NV_SINT32
nfo_sol_lockalloc(PNV_VOID ctx, NV_SINT32 type, PNV_VOID *pLock)
{
	struct gem_dev 		*dp = (struct gem_dev *)ctx;
	struct nfo_dev 		*lp = (struct nfo_dev *)dp->private;

	*pLock = (PNV_VOID)&lp->phylock;
	mutex_init(&lp->phylock, NULL, MUTEX_DRIVER, (void *)dp->iblock_cookie);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_lockacquire(PNV_VOID ctx, NV_SINT32 type, PNV_VOID lock)
{
	mutex_enter((kmutex_t *)lock);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_lockrelease(PNV_VOID ctx, NV_SINT32 type, PNV_VOID lock)
{
	mutex_exit((kmutex_t *)lock);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_alloc(PNV_VOID ctx, MEMORY_BLOCK *mem)
{
	struct gem_dev 		*dp = (struct gem_dev *)ctx;
	struct nfo_dev 		*lp = (struct nfo_dev *)dp->private;
	size_t			req_size;
	struct nfo_mem_head	mh;
	ddi_dma_cookie_t	dmac;
	int			count;
	int			err;

	DPRINTF(1, (CE_CONT, "%s: nfo_sol_alloc: in (size:%d)",
		dp->name, mem->uiLength));

	if ((err = ddi_dma_alloc_handle(dp->dip,
			&dp->gc.gc_dma_attr_desc,
			DDI_DMA_SLEEP,NULL,
			&mh.nmh_dma_handle)) != DDI_SUCCESS) {
		cmn_err(CE_WARN,
		"!%s: nfo_sol_alloc: ddi_dma_alloc_handle failed: %d",
			dp->name, err);
		return (FALSE);
	}

	req_size = (size_t)mem->uiLength + sizeof(struct nfo_mem_head);

	if ((err = ddi_dma_mem_alloc(mh.nmh_dma_handle,
			req_size,
			&dp->gc.gc_desc_attr,
			DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
			&mh.nmh_addr, &mh.nmh_len,
			&mh.nmh_acc_handle)) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
			"!%s: nfo_sol_alloc: ddi_dma_mem_alloc failed: "
			"ret %d, request size: %d",
				dp->name, err, req_size);
			ddi_dma_free_handle(&mh.nmh_dma_handle);
			return (FALSE);
		}

	if ((err = ddi_dma_addr_bind_handle(mh.nmh_dma_handle,
				NULL, mh.nmh_addr, mh.nmh_len,
				DDI_DMA_RDWR | DDI_DMA_CONSISTENT,
				DDI_DMA_SLEEP, NULL,
				&dmac, &count)) != DDI_SUCCESS) {
			ASSERT(err != DDI_DMA_INUSE);
			cmn_err(CE_WARN,
				"!%s: gem_alloc_memory: "
				"ddi_dma_addr_bind_handle failed: %d",
				dp->name, err);
			ddi_dma_mem_free(&mh.nmh_acc_handle);
			ddi_dma_free_handle(&mh.nmh_dma_handle);
			return (FALSE);
	}
	ASSERT(count == 1);

	mem->pLogical = (void *) (mh.nmh_addr + sizeof(mh));
	mem->pPhysical = (void *) (long) (dmac.dmac_laddress + sizeof(mh));

	/* copy memory header at the head of allocated memoery */
	bcopy(&mh, mh.nmh_addr, sizeof(mh));

	/* clear the payload area */
	bzero(mem->pLogical, mem->uiLength);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_free(PNV_VOID ctx, MEMORY_BLOCK *mem)
{
	struct gem_dev		*dp = (struct gem_dev *)ctx;
	struct nfo_dev		*lp = (struct nfo_dev *)dp->private;
	struct nfo_mem_head	mh;

	DPRINTF(4, (CE_CONT, "%s: nfo_sol_free: in", dp->name));

	bcopy(((caddr_t)mem->pLogical) - sizeof(mh), &mh, sizeof(mh));

	ddi_dma_unbind_handle(mh.nmh_dma_handle);
	ddi_dma_mem_free(&mh.nmh_acc_handle);
	ddi_dma_free_handle(&mh.nmh_dma_handle);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_allocex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex)
{
	struct gem_dev	*dp = (struct gem_dev *)ctx;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;
	MEMORY_BLOCK	mem_block;
    
	mem_block_ex->pLogical = NULL;
	mem_block_ex->uiLengthOrig = mem_block_ex->uiLength;

	if ((mem_block_ex->AllocFlags & ALLOC_MEMORY_ALIGNED) != 0 &&
	    (mem_block_ex->AlignmentSize > 1)) {
		mem_block_ex->uiLengthOrig += mem_block_ex->AlignmentSize;
	}

	mem_block.uiLength = mem_block_ex->uiLengthOrig;

	if (nfo_sol_alloc(ctx, &mem_block) == 0) {
		return (FALSE);
	}

	mem_block_ex->pLogicalOrig = mem_block.pLogical;
	mem_block_ex->pPhysicalOrigLow =
			(NV_UINT32)((unsigned long)mem_block.pPhysical);
	mem_block_ex->pPhysicalOrigHigh = 0;

	mem_block_ex->pPhysical = mem_block.pPhysical;
	mem_block_ex->pLogical = mem_block.pLogical;

	if (mem_block_ex->uiLength != mem_block_ex->uiLengthOrig) {
		unsigned int offset;

		offset = mem_block_ex->pPhysicalOrigLow
				& (mem_block_ex->AlignmentSize - 1);

		if (offset) {
			mem_block_ex->pPhysical = (PNV_VOID)
				((unsigned long)mem_block_ex->pPhysical + 
					mem_block_ex->AlignmentSize - offset);
			mem_block_ex->pLogical = (PNV_VOID)
				((unsigned long)mem_block_ex->pLogical +
					mem_block_ex->AlignmentSize - offset);
		}
	}

	return (TRUE);
}

static NV_API_CALL NV_SINT32 
nfo_sol_freeex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex) 
{
	struct gem_dev	*dp = (struct gem_dev *)ctx;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;
	MEMORY_BLOCK	mem_block;

	mem_block.pLogical  = mem_block_ex->pLogicalOrig;
	mem_block.pPhysical = (PNV_VOID) (long) mem_block_ex->pPhysicalOrigLow;
	mem_block.uiLength  = mem_block_ex->uiLengthOrig;

	return (nfo_sol_free(ctx, &mem_block));
}

static NV_API_CALL NV_SINT32
nfo_sol_clear(PNV_VOID ctx, PNV_VOID mem, NV_SINT32 length)
{
	bzero(mem, length);
	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_delay(PNV_VOID ctx, NV_UINT32 usec)
{
	drv_usecwait(usec);
	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_allocrxbuf(PNV_VOID ctx, MEMORY_BLOCK *mem, PNV_VOID *id) 
{
	struct gem_dev	*dp = (struct gem_dev *)ctx;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;
	struct rxbuf	*rbp;

	if ((rbp = gem_get_rxbuf(dp, FALSE)) == NULL) {
		DPRINTF(0, (CE_CONT, "%s: %s: error", dp->name, __func__));
		return (FALSE);
	}

	ASSERT(rbp->rxb_nfrags == 1);

#ifdef DEBUG_LEVEL
	if (mem->uiLength > rbp->rxb_buf_len) {
		cmn_err(CE_WARN,
		"%s: %s: allocated rx buffer size of smaller than requested, "
			"requested:%d, actual:%d",
			dp->name, __func__, mem->uiLength, rbp->rxb_buf_len);
	}
#endif
	mem->pLogical  = (PNV_VOID)rbp->rxb_buf;
	mem->uiLength  = rbp->rxb_buf_len;
	mem->pPhysical = (PNV_VOID)(long)rbp->rxb_dmacookie[0].dmac_laddress;

	DPRINTF(4, (CE_CONT,
		"%s: %s: logical:0x%p, len:0x%p, physical:0x%p, rx_desc:%d",
		dp->name, __func__,
		mem->pLogical, mem->uiLength, mem->pPhysical,
		lp->rx_desc_tail));

	DPRINTF(4, (CE_CONT, "%s: %s: rx_desc(%d, %d)",
		dp->name, __func__, lp->rx_desc_head, lp->rx_desc_tail));

	*id = (PNV_VOID) rbp;
#ifdef DEBUG_LEVEL
	rbp->rxb_slot = lp->rx_desc_tail++;
#endif
	/* append rbp into active rx buffer list */
	RBACT(rbp)->dl_next = RBACT(RBACT(&lp->rb_act)->dl_prev)->dl_next;
	RBACT(rbp)->dl_prev = RBACT(&lp->rb_act)->dl_prev;

	RBACT(RBACT(&lp->rb_act)->dl_prev)->dl_next = rbp;
	RBACT(&lp->rb_act)->dl_prev = rbp;

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_freerxbuf(PNV_VOID ctx, MEMORY_BLOCK *mem, PNV_VOID id)
{
	int		slot;
	struct gem_dev *dp = (struct gem_dev *)ctx;
	struct nfo_dev *lp = (struct nfo_dev *)dp->private;

	DPRINTF(3, (CE_CONT, "%s: %s: in rxb:0x%p", dp->name, __func__, id));

	DPRINTF(4, (CE_CONT,
		"%s: %s: rx_desc[%d, %d], freeing %d",
		dp->name, __func__,
		lp->rx_desc_head, lp->rx_desc_tail,
		((struct rxbuf *)id)->rxb_slot));

	if (RBACT((struct rxbuf *) id)->dl_next == NULL ||
	    RBACT((struct rxbuf *) id)->dl_prev == NULL) {
		/* rbp has been free */
		cmn_err(CE_WARN, "%s: %s: freeing free rx buffer",
			dp->name, __func__);
		return (FALSE);
	}

	/*
	 * remove rbp from active rx buffer list
	 */
	RBACT(RBACT((struct rxbuf *) id)->dl_prev)->dl_next =
					RBACT((struct rxbuf *) id)->dl_next;
	RBACT(RBACT((struct rxbuf *) id)->dl_next)->dl_prev =
					RBACT((struct rxbuf *) id)->dl_prev;

	RBACT((struct rxbuf *) id)->dl_prev = NULL;
	RBACT((struct rxbuf *) id)->dl_next = NULL;

	/*
	 * No need to free mapping resources if it have
	 */
	gem_free_rxbuf((struct rxbuf *) id);
#ifdef DEBUG_LEVEL
	lp->rx_desc_head++;
#endif
	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_packettx(PNV_VOID ctx, PNV_VOID id, NV_UINT32 success)
{
	struct gem_dev	*dp = (struct gem_dev *)ctx;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;
	int		slot;

	slot = ((struct nfo_tx_desc *)id) - lp->tx_desc;

	DPRINTF(2, (CE_CONT, "%s: %s: slot:%d", dp->name, __func__, slot));

	/* XXX - this routine seems to be called from interrupt handler only */
#ifdef notdef
	if (!lp->drain) {
		mutex_enter(&dp->xmitlock);
	}
#else
	ASSERT(mutex_owned(&dp->xmitlock));
#endif
	if ((lp->tx_desc[slot].status & GEM_TXFLAG_INTR) != 0) {
		lp->intr_tx_done = TRUE;
	}

	lp->tx_desc[slot].status |= GEM_TX_DONE;
#ifdef notdef
	if (!lp->drain) {
		mutex_exit(&dp->xmitlock);
	}
#endif
	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_packetrx(PNV_VOID ctx, PNV_VOID data, NV_UINT32 status, 
              PNV_UINT8 newbuf, NV_UINT8 priority)
{
	ADAPTER_READ_DATA	*readdata = (ADAPTER_READ_DATA *)data;
	struct rxbuf		*rbp = (struct rxbuf *) readdata->pvID;
	int			slot;
	int			len;
	mblk_t			*mp = NULL;
	struct gem_dev		*dp = (struct gem_dev *)ctx;
	struct nfo_dev		*lp = (struct nfo_dev *)dp->private;

	DPRINTF(4, (CE_CONT,
		"%s: %s: rx_desc[%d, %d], freeing %d",
		dp->name, __func__,
		lp->rx_desc_head, lp->rx_desc_tail, rbp->rxb_slot));

	ASSERT(rbp->rxb_mp != NULL);
	ASSERT(rbp->rxb_nfrags == 1);

        len = readdata->ulTotalLength;

	DPRINTF(4, (CE_CONT,
		"%s: %s: in rbp:0x%p len:%d bounce:%d",
		dp->name, __func__,
		rbp, len, (rbp->rxb_flags & RXB_FLAGS_BOUNCE) != 0));

	if (status == 0 || lp->drain) {
		/* errored packet */
		DPRINTF(4, (CE_CONT, "%s: %s: error", dp->name, __func__));

		nfo_sol_freerxbuf(ctx, NULL, (PNV_VOID) rbp);
		return (TRUE);
	}

	if (len <= dp->gc.gc_rx_copy_thresh) {
		/*
		 * Copy the packet
		 */

		/* allocate a new small mblk */
		if ((mp = allocb(len, BPRI_MED)) == NULL) {
			/* no memory */
			dp->stats.norcvbuf++;
			goto x;
		}
		mp->b_wptr = mp->b_rptr + len;

		ddi_dma_sync(rbp->rxb_dh, 0, len, DDI_DMA_SYNC_FORKERNEL);

		bcopy(rbp->rxb_buf, mp->b_rptr, len);
	}
	else {
		/*
		 * Direct reception
		 */
		ddi_dma_unbind_handle(rbp->rxb_dh);
		rbp->rxb_nfrags = 0;

		/* remove the associated mblk from the rbp */
		mp = rbp->rxb_mp;
		rbp->rxb_mp = NULL;
		mp->b_wptr = mp->b_rptr + len;

		if ((rbp->rxb_flags & RXB_FLAGS_BOUNCE) != 0) {
			/*
			 * copy the received packet from
			 * the bounce buffer
			 */
			bcopy(rbp->rxb_buf, mp->b_rptr, len);
		}
	}
x:
	nfo_sol_freerxbuf(ctx, NULL, (PNV_VOID) rbp);

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);
	if (mp != NULL) {
		gld_recv(dp->macinfo, mp);
	}
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	return (TRUE);
}

static NV_API_CALL NV_SINT32
nfo_sol_linkchanged(PNV_VOID ctx, NV_SINT32 enabled)
{
	struct gem_dev *dp = (struct gem_dev *)ctx;
	struct nfo_dev *lp = (struct nfo_dev *)dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called, time:%d",
		dp->name, __func__, ddi_get_lbolt()));
#ifdef notdef
	nfo_set_media(dp);
#endif
	return (TRUE);
}

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */
static int
nfo_reset_chip(struct gem_dev *dp)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called, time:%d",
		dp->name, __func__, ddi_get_lbolt()));

	if (lp->pADApi != NULL) {
		lp->pADApi->pfnDisableInterrupts(lp->pADApi->pADCX);
		lp->pADApi->pfnStop(lp->pADApi->pADCX,
				AFFECT_TRANSMITTER | AFFECT_RECEIVER);
		drv_usecwait(100);
		if (lp->drain) {
			do {
				lp->pADApi->pfnHandleInterrupt(
						lp->pADApi->pADCX);
				drv_usecwait(100);
			} while (lp->pADApi->pfnQueryInterrupt(
						lp->pADApi->pADCX));
			lp->drain = FALSE;
		}
		lp->pADApi->pfnDeinit(lp->pADApi->pADCX,
				AFFECT_TRANSMITTER | AFFECT_RECEIVER);

		/*
		 * XXX - As pfnDeinit seems to reset PHY, 
		 * need to synchronize its state with mii state in gem.
		 */
		(void) GEM_MII_CONFIG(dp);
		dp->mii_timer = 0;
		dp->mii_state = MII_STATE_RESETTING;
#ifdef NEVER
		/*
		 * XXX - nvenetlib bug:
		 * As nvenetlib frees rx buffers twice, don't use calling
		 * pfnClose and Open to reset it.
		 */
		nfo_destory_adapter_object(dp);
		nfo_create_adapter_object(dp);
#endif
	}

	return (GEM_SUCCESS);
}

static void
nfo_init_chip(struct gem_dev *dp)
{
	int		i;
	int		status;
	uint8_t		mac[ETHERADDRL];
	NV_UINT32	linkup;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
		dp->name, __func__, ddi_get_lbolt()));

	linkup = TRUE;

	/*
	 * Initialize hardware
	 * XXX - this routine will allocate rx buffers.
	 */
	status = lp->pADApi->pfnInit(
		    lp->pADApi->pADCX, 
		    lp->forced_speed,	/* force speed */ 
		    lp->force_duplex,	/* force full duplex */
		    lp->force_mode,	/* force mode */
		    FALSE,		/* async mode */
		    &linkup
		    );

	if (status != ADAPTERERR_NONE) {
		cmn_err(CE_WARN, "%s: %s: initialize failed",
			dp->name, __func__);
	}

	lp->tx_list_len = 0;
	
	lp->last_stats_time = ddi_get_lbolt();


	DPRINTF(2, (CE_CONT, "%s: %s: out", dp->name, __func__));
}

static void
nfo_start_chip(struct gem_dev *dp)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
		dp->name, __func__, ddi_get_lbolt()));

	/* fix speed and duplex */
	nfo_set_media(dp);

	/*
	 * Turn on the hardware
	 */
	(void) lp->pADApi->pfnEnableInterrupts(lp->pADApi->pADCX);
	(void) lp->pADApi->pfnStart(lp->pADApi->pADCX);
}

static int
nfo_stop_chip(struct gem_dev *dp)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
		dp->name, __func__, ddi_get_lbolt()));

	lp->drain = TRUE;
	nfo_reset_chip(dp);

	return (GEM_SUCCESS);
}

static void
nfo_set_rx_filter(struct gem_dev *dp)	
{
	int		i, j;
	uint8_t		*mac;
	uint8_t		oraddr[ETHERADDRL];
	uint8_t		andaddr[ETHERADDRL];
	PACKET_FILTER	hwfilter;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d, "
		"active:%d, rxmode:%b",
		dp->name, __func__, ddi_get_lbolt(),
		dp->nic_active, dp->rxmode, "\020\002AllMulti\001Promisc"));

	/*
	 * Initialize filter
	 */
	bzero(&hwfilter, sizeof(hwfilter));

	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		hwfilter.ulFilterFlags = ACCEPT_ALL_PACKETS;
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 || dp->mc_count > 32) {
		hwfilter.ulFilterFlags = ACCEPT_MULTICAST_PACKETS
				       | ACCEPT_UNICAST_PACKETS
				       | ACCEPT_BROADCAST_PACKETS;

		hwfilter.acMulticastMask[0]    = 0xff;
		hwfilter.acMulticastAddress[0] = 0x01;
		hwfilter.acMulticastMask[1]    = 0xff;
		hwfilter.acMulticastAddress[1] = 0x00;

        }
        else if (dp->mc_count > 0) {

		/*
		 * Process multicast address list
		 */
		hwfilter.ulFilterFlags = ACCEPT_MULTICAST_PACKETS
				       | ACCEPT_UNICAST_PACKETS
				       | ACCEPT_BROADCAST_PACKETS;

		for (j = 0; j < ETHERADDRL; j++) {
			andaddr[j] = 0xff;
			oraddr[j]  = 0x00;
		}

		for (i = 0; i < dp->mc_count; i++) {
			mac = dp->mc_list[i].addr.ether_addr_octet;
			for (j = 0; j < ETHERADDRL; j++) {
				andaddr[j] &= mac[j];
				oraddr[j]  |= mac[j];
			}
		}

		/* 
		 * Write 1s to mask where all addresses have 0 bit or 1 bit. 
		 * Compute mask and address value.
		 */
		for (j = 0; j < ETHERADDRL; j++) {
			hwfilter.acMulticastAddress[j] = andaddr[j] & oraddr[j];
			hwfilter.acMulticastMask[j]    = andaddr[j] |~oraddr[j];
		}
        }
	else {
		hwfilter.ulFilterFlags = ACCEPT_UNICAST_PACKETS
				       |  ACCEPT_BROADCAST_PACKETS;
	}

	lp->pADApi->pfnSetPacketFilter(lp->pADApi->pADCX, &hwfilter);

	if (bcmp(dp->cur_addr.ether_addr_octet,
			lp->mac_addr, ETHERADDRL) != 0) {
		/* Set new node address */
		bcopy(dp->cur_addr.ether_addr_octet, lp->mac_addr, ETHERADDRL);
		lp->pADApi->pfnSetNodeAddress(lp->pADApi->pADCX, lp->mac_addr);
	}

	DPRINTF(2, (CE_CONT, "%s: %s - out", dp->name, __func__));
}

static void
nfo_set_media(struct gem_dev *dp)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d, nic_active:%d",
		dp->name, __func__, ddi_get_lbolt(), dp->nic_active));

	if (!dp->nic_active) {
		return;
	}

	/*
	 * Set speed and duplex settings on the hardware
	 */
	lp->pADApi->pfnSetSpeedDuplex(lp->pADApi->pADCX);
}

static void
nfo_get_stats(struct gem_dev *dp)
{
	int		i;
	uint64_t	c, t;
	ADAPTER_STATS	stats;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	if (!dp->nic_active) {
		return;
	}

	lp->pADApi->pfnGetStatistics(lp->pADApi->pADCX, &stats);

	lp->last_stats_time = ddi_get_lbolt();

        dp->stats.errrcv =
			stats.ulMissedFrames +
			stats.ulFailedReceptions +
			stats.ulCRCErrors +
			stats.ulFramingErrors +
			stats.ulOverFlowErrors ;

	dp->stats.errxmt =
			stats.ulFailedTransmissions +
			stats.ulRetryErrors +
			stats.ulUnderflowErrors +
			stats.ulLossOfCarrierErrors +
			stats.ulLateCollisionErrors;

	dp->stats.crc      = stats.ulCRCErrors;
	dp->stats.overflow = stats.ulOverFlowErrors;
	dp->stats.frame    = stats.ulFramingErrors;
	dp->stats.missed   = stats.ulMissedFrames;
	dp->stats.runt     = stats.ulRxRuntPacketErrors;
	dp->stats.frame_too_long = stats.ulRxFrameTooLongCount;

        for (t = 0, i = 2; i < MAX_TRANSMIT_COLISION_STATS; i++) {
		t += stats.aulSuccessfulTransmitsAfterCollisions[i];
	}
        for (c = 0, i = 1; i < MAX_TRANSMIT_COLISION_STATS; i++) {
		c += stats.aulSuccessfulTransmitsAfterCollisions[i]*i;
	}
        dp->stats.collisions = c + stats.ulRetryErrors*16;
        dp->stats.first_coll = stats.aulSuccessfulTransmitsAfterCollisions[1];
        dp->stats.multi_coll = t;
        dp->stats.excoll    = stats.ulRetryErrors;
        dp->stats.nocarrier = stats.ulLossOfCarrierErrors;
        dp->stats.defer     = stats.ulDeferredTransmissions;
        dp->stats.underflow = stats.ulUnderflowErrors;
#ifdef notdef
	dp->stats.xmtlatecoll = stats.ulLateCollisionErrors;
#endif
}

/*
 * discriptor manupiration
 */
static int
nfo_tx_desc_write(struct gem_dev *dp, uint_t slot,
		ddi_dma_cookie_t *dmacookie, int frags, uint32_t intreq)
{
	int			i;
	int			total;
	ddi_dma_cookie_t	*dcp = dmacookie;
	ADAPTER_WRITE_DATA	txdata;
	ADAPTER_WRITE_OFFLOAD	write_offload = { 0 };
	int			status;
	struct nfo_dev		*lp = (struct nfo_dev *)dp->private;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"!%s: %s time: %d, seqnum: %d, slot %d, frags: %d flags: %d",
		dp->name, __func__, ddi_get_lbolt(),
		dp->tx_desc_tail, slot, frags, intreq);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%llx, len: 0x%x",
			i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * Put the fragment information in tx ring
	 */
	lp->tx_desc[slot].status  = GEM_TXFLAG_INTR & intreq;
	txdata.pvID		  = (PNV_VOID) &lp->tx_desc[slot];
	txdata.ulNumberOfElements = frags;

	total = 0;
        for (i = 0; i < frags; i++) { 
		txdata.sElement[i].pPhysical =
					(PNV_VOID) (long) dcp[i].dmac_laddress;
		txdata.sElement[i].ulLength  = dcp[i].dmac_size;
		total += dcp[i].dmac_size;
        }

	txdata.ulTotalLength = total;
	txdata.psOffload = &write_offload;
#ifdef TEST_TX_ERR
	if ((dp->tx_desc_tail % 10000) == 999) {
		status = ADAPTERERR_NONE;
		lp->tx_desc[slot].status |= GEM_TX_DONE | GEM_TX_ERR;
	} else
#endif
	status = lp->pADApi->pfnWrite(lp->pADApi->pADCX, &txdata);

	if (status != ADAPTERERR_NONE) {
		/*
		 * XXX: when nvenetlib returns TX_QUEUE_FULL,
		 * it need to reset nvenetlib to recover.
		 */
		lp->tx_desc[slot].status |= GEM_TX_DONE | GEM_TX_ERR;

		i = min(status, ARRAY_SIZE(nfo_adapter_error_names) - 1);
		cmn_err(CE_WARN,
			"%s: %s - transmit error %s (%d)",
			dp->name, __func__, nfo_adapter_error_names[i], status);
		lp->drain = TRUE;
	}
	lp->tx_list_len++;

	return (frags);
}

static uint_t
nfo_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;
	uint_t		ret;

	ret = lp->tx_desc[slot].status;
	if ((ret & (GEM_TX_DONE | GEM_TX_ERR)) == 0) {
		return (0);
	}

	ASSERT(lp->tx_list_len > 0);

	lp->tx_list_len--;
	if ((lp->tx_list_len == 0 && dp->tx_blocked) ||
	    (ret & GEM_TXFLAG_INTR) !=0) {
		lp->intr_tx_done = TRUE;
	}
	return (ret & (GEM_TX_DONE | GEM_TX_ERR));
}

static void
nfo_tx_desc_init(struct gem_dev *dp, int slot)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	lp->tx_desc[slot].status = 0;
}

/*
 * Device depend interrupt handler
 */
static u_int
nfo_interrupt(struct gem_dev *dp)
{
	uint_t		flag = 0;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "%s: %s: called", dp->name, __func__));

	mutex_enter(&dp->xmitlock);
	if (!lp->pADApi->pfnQueryInterrupt(lp->pADApi->pADCX)) {
		/* Not for us */
		mutex_exit(&dp->xmitlock);
		return (DDI_INTR_UNCLAIMED);
	}

        lp->pADApi->pfnHandleInterrupt(lp->pADApi->pADCX);
        lp->pADApi->pfnEnableInterrupts(lp->pADApi->pADCX);
	mutex_exit(&dp->xmitlock);

	if (lp->intr_tx_done) {
		lp->intr_tx_done = FALSE;
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}

	return (DDI_INTR_CLAIMED | flag);
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
	uint32_t	val;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	if (dp->nic_active &&
	   (ddi_get_lbolt() - lp->last_stats_time) >= ONESEC) {
		nfo_get_stats(dp);
	}

	if (!lp->pADApi->pfnReadPhy(
			lp->pADApi->pADCX, dp->mii_phy_addr, reg, &val)) {
		val = 0xffff;
	}
	return ((uint16_t) val);
}

static void
nfo_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	(void) lp->pADApi->pfnWritePhy(
			lp->pADApi->pADCX, dp->mii_phy_addr, reg, val);
}


static int
nfo_create_adapter_object(struct gem_dev *dp)
{
	struct nfo_dev		*lp = (struct nfo_dev *)dp->private;
	OS_API			*osapi;
	CMNDATA_OS_ADAPTER	adapterdata;
	int			status;
	uint32_t		phyaddr;	/* unused */
	ADAPTER_OPEN_PARAMS	OpenParams;

	DPRINTF(2, (CE_CONT, "%s: %s (hwmode:%d)",
		dp->name, __func__, lp->chip->hwmode));

	/*
	 * Plug into the API
	 */
	osapi					= &lp->sol_api;
	osapi->pOSCX				= dp;
	osapi->pfnAllocMemory			= nfo_sol_alloc;
	osapi->pfnFreeMemory			= nfo_sol_free;
	osapi->pfnClearMemory			= nfo_sol_clear;
	osapi->pfnAllocMemoryEx			= nfo_sol_allocex;
	osapi->pfnFreeMemoryEx			= nfo_sol_freeex;
	osapi->pfnStallExecution		= nfo_sol_delay;
	osapi->pfnAllocReceiveBuffer		= nfo_sol_allocrxbuf;
	osapi->pfnFreeReceiveBuffer		= nfo_sol_freerxbuf;
	osapi->pfnPacketWasReceived		= nfo_sol_packetrx;
	osapi->pfnPacketWasSent			= nfo_sol_packettx;
	osapi->pfnLinkStateHasChanged		= nfo_sol_linkchanged;
	osapi->pfnLockAlloc			= nfo_sol_lockalloc;
	osapi->pfnLockAcquire			= nfo_sol_lockacquire;
	osapi->pfnLockRelease			= nfo_sol_lockrelease;

	/*
	 * Set open parameters 
	 */
	bzero(&OpenParams, sizeof(OpenParams));

	OpenParams.MaxDpcLoop			= 2;
	OpenParams.MaxRxPkt			= RX_RING_SIZE;
	OpenParams.MaxTxPkt			= TX_RING_SIZE;
	OpenParams.SentPacketStatusSuccess	= TRUE;
	OpenParams.SentPacketStatusFailure	= FALSE;
	OpenParams.MaxRxPktToAccumulate		= 6;
	OpenParams.ulPollInterval		= 0;	/* for throughput */
	OpenParams.SetForcedModeEveryNthRxPacket = 0;
	OpenParams.SetForcedModeEveryNthTxPacket = 0;
	OpenParams.RxForcedInterrupt		= 0;
	OpenParams.TxForcedInterrupt		= 0;
	OpenParams.pOSApi			= osapi;
	OpenParams.pvHardwareBaseAddress	= dp->base_addr;
	OpenParams.bASFEnabled			= 0;
	OpenParams.ulDescriptorVersion		= lp->chip->hwmode;
	OpenParams.ulMaxPacketSize		=
			dp->mtu + sizeof(struct ether_header) + ETHERFCSL;
	OpenParams.DeviceId			= lp->chip->devid;

	status = ADAPTER_Open(&OpenParams, (void **)&lp->pADApi, &phyaddr);

	if(status != ADAPTERERR_NONE) {
		cmn_err(CE_WARN,
			"%s: %s: ADAPTER_Open failed", dp->name, __func__);
		return (GEM_FAILURE);
	}

	/*
	 * Set the adapter data
	 */
	bzero(&adapterdata, sizeof(adapterdata));

	adapterdata.ulMediaIF = (lp->chip->hwmode == HWMODE1)
				? MEDIA_IF_MII : MEDIA_IF_RGMII;

	if (lp->chip->hwmode == HWMODE2) {
		adapterdata.ulChecksumOffloadBM = 0x0FFFF;
	}
	adapterdata.ulModeRegTxReadCompleteEnable = 1;

	lp->pADApi->pfnSetCommonData(lp->pADApi->pADCX, &adapterdata);

	/*
	 * Offload capabilities to hardware
	 */
	if (lp->chip->hwmode == HWMODE2) {
		NV_UINT32 offloadcap = 0;

		/*
		 * Set capabilities on hardware
		 */
		NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_IPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_TCPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_UDPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_IPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_TCPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_UDPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V6_TX_BP_TCPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V6_TX_BP_UDPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V6_RX_BP_TCPCHECKSUM);
		NV_BIT_SET(offloadcap, DEVCAPS_V6_RX_BP_UDPCHECKSUM);

		lp->pADApi->pfnSetChecksumOffload(
			lp->pADApi->pADCX, offloadcap);

	}

	DPRINTF(2, (CE_CONT, "%s: %s: return", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int
nfo_destroy_adapter_object(struct gem_dev *dp)
{
	struct nfo_dev		*lp = (struct nfo_dev *)dp->private;

	lp->pADApi->pfnClose(lp->pADApi->pADCX, FALSE);
	mutex_destroy(&lp->phylock);

	return (GEM_SUCCESS);
}

static int
nfo_attach_chip(struct gem_dev *dp)
{
	int		i;
	boolean_t	forced_autonego;
	struct nfo_dev	*lp = (struct nfo_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called at time:%d",
		dp->name, __func__, ddi_get_lbolt()));

        /* fix rx buffer length, it needs additional 48 bytes. */
        dp->rx_buf_len += 48;

	/*
	 * Set speed and duplex settings
	 */
	dp->anadv_1000hdx = 0;	/* 1000HD isn't supported by the mac */
	if (dp->mii_fixedmode &&
	   (dp->speed == GEM_SPD_1000) && !dp->full_duplex) {
		cmn_err(CE_WARN,
	"%s: fixed 1000Mbps half duplex isn't supported, using full duplex.",
			dp->name);
		dp->anadv_1000fdx = TRUE;
		dp->anadv_1000hdx = FALSE;
		dp->anadv_100fdx  = FALSE;
		dp->anadv_100hdx  = FALSE;
		dp->anadv_10fdx   = FALSE;
		dp->anadv_10hdx   = FALSE;
		dp->mii_fixedmode = FALSE;
	}

	forced_autonego = (dp->anadv_1000fdx + dp->anadv_1000hdx +
			   dp->anadv_100fdx  + dp->anadv_100hdx  +
			   dp->anadv_10fdx   + dp->anadv_10hdx) == 1;

	if (dp->mii_fixedmode && dp->speed == GEM_SPD_100) {
		lp->forced_speed  = 100;
		lp->force_duplex = dp->full_duplex ? 2 : 1;
		lp->force_mode    = 1;
	}
	else if (dp->mii_fixedmode && dp->speed == GEM_SPD_10) {
		lp->forced_speed  = 10;
		lp->force_duplex = dp->full_duplex ? 2 : 1;
		lp->force_mode    = 1;
	}
	else if (!forced_autonego) {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE */
		lp->force_duplex  = 0;
		lp->forced_speed  = 0;
		lp->force_mode    = 0;
		dp->anadv_1000fdx = TRUE;
		dp->anadv_1000hdx = FALSE;
		dp->anadv_100fdx  = TRUE;
		dp->anadv_100hdx  = TRUE;
		dp->anadv_10fdx   = TRUE;
		dp->anadv_10hdx   = TRUE;
	}
	else if (dp->anadv_1000fdx) {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_1000FD */
		lp->force_duplex  = 2;
		lp->forced_speed  = 1000;
		lp->force_mode    = 0;
	}
	else if (dp->anadv_100fdx) {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100FD */
		lp->force_duplex  = 2;
		lp->forced_speed  = 100;
		lp->force_mode    = 0;
	}
	else if (dp->anadv_100hdx) {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100_HD */
		lp->force_duplex  = 1;
		lp->forced_speed  = 100;
		lp->force_mode    = 0;
	}
	else if (dp->anadv_10fdx) {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_FD */
		lp->force_duplex  = 2;
		lp->forced_speed  = 10;
		lp->force_mode    = 0;
	}
	else /* if (dp->anadv_10hdx) */ {
		/* NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_HD */
		lp->force_duplex  = 1;
		lp->forced_speed  = 10;
		lp->force_mode    = 0;
	}

	RBACT(&lp->rb_act)->dl_next = &lp->rb_act;
	RBACT(&lp->rb_act)->dl_prev = &lp->rb_act;

	nfo_create_adapter_object(dp);

	/* Save factory mac address */
	lp->pADApi->pfnGetNodeAddress(lp->pADApi->pADCX, lp->factory_mac_addr);

	/*
	 * BUG - As the factory mac address is set in incorrect format,
	 * aka in reversed order after power-on, we must re-write it.
	 * But it aren't copies anywhre else, the original address gone
	 * when we fix it.
	 */
	for (i = 0; i < ETHERADDRL; i++) {
		dp->dev_addr.ether_addr_octet[i] =
			lp->factory_mac_addr[ETHERADDRL - 1 - i];
	}

#if DEBUG_LEVEL > 4
{
	uint8_t		*mac;

	mac = dp->dev_addr.ether_addr_octet;
	cmn_err(CE_CONT, "%s: mac:%02x:%02x:%02x:%02x:%02x:%02x",
		dp->name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
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
	struct nfo_dev		*lp;
	caddr_t			base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	uint32_t		val;
	uint32_t		ilr;

	unit =  ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(2, (CE_CONT, "!%s%d: nfoattach: called at time:%d",
		drv_name, unit, ddi_get_lbolt()));

	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: ddi_regs_map_setup failed",
			drv_name, unit);
		return (DDI_FAILURE);
	}

	/* ensure we can access the registers through IO/MEM32 space. */
	ret = pci_config_get16(conf_handle, PCI_CONF_COMM);
	ret |= PCI_COMM_IO | PCI_COMM_ME | PCI_COMM_MAE;
	ilr = pci_config_get32(conf_handle, PCI_CONF_ILINE);
	pci_config_put16(conf_handle, PCI_CONF_COMM, ret);

	DPRINTF(0, (CE_CONT, "!%s%d: ilr 0x%08x", drv_name, unit, ilr));

	/* ensure the pmr status is  D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	vid  = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did  = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	revid= pci_config_get8(conf_handle, PCI_CONF_REVID);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		/* Check if chip is supported.  */
		for (i = 0, p = nfo_chiptbl; i < CHIPTABLESIZE; i++, p++) {
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
			"!%s: nfo_attach: wrong PCI venid/devid (0x%x, 0x%x)",
				drv_name, vid, did);

			/* assuming latest chipset */
			p = &nfo_chiptbl[CHIPTABLESIZE-1];
		}

		/* Map the device registers in the kernel virtual space.  */
		if (gem_pci_regs_map_setup(dip, PCI_ADDR_MEM32, &nfo_dev_attr,
			(caddr_t *)&base, &regs_ha) != DDI_SUCCESS) {
			goto err;
		}

		/* construct gem configration */
		gcp = (struct gem_conf *) kmem_zalloc(sizeof(*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof(uint8_t) - 1;
		gcp->gc_tx_max_frags = GEM_MAXTXFRAGS;
		gcp->gc_tx_desc_size = 0;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_buf_size  = TX_BUF_SIZE;
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags;
		gcp->gc_tx_auto_pad  = FALSE;
		gcp->gc_tx_copy_thresh = nfo_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof(uint32_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_size = 0;			/* not used */
		gcp->gc_rx_ring_size = 0;			/* not used */
		gcp->gc_rx_buf_size  = 0;			/* not used */
		gcp->gc_rx_max_descs_per_pkt = gcp->gc_rx_max_frags;
		gcp->gc_rx_copy_thresh = nfo_rx_copy_thresh;
		gcp->gc_rx_buf_max     = INT32_MAX;	/* not under control */
		gcp->gc_rx_header_len  = 0;

		/* map attributes */
		STRUCT_COPY(gcp->gc_dev_attr, nfo_dev_attr);
		STRUCT_COPY(gcp->gc_buf_attr, nfo_buf_attr);
		STRUCT_COPY(gcp->gc_desc_attr, nfo_dev_attr);

		/* dma attributes */
		STRUCT_COPY(gcp->gc_dma_attr_desc, nfo_dma_attr_desc);
		STRUCT_COPY(gcp->gc_dma_attr_txbuf, nfo_dma_attr_buf);
		gcp->gc_dma_attr_txbuf.dma_attr_align  = gcp->gc_tx_buf_align+1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;
		STRUCT_COPY(gcp->gc_dma_attr_rxbuf, nfo_dma_attr_buf);
		gcp->gc_dma_attr_rxbuf.dma_attr_align  = gcp->gc_rx_buf_align+1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;


		/* timeout parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

                /* mii mode */
#ifdef notdef
		/* XXX - obsolete */
		gcp->gc_mii_mode =
			p->hwmode == HWMODE2
				? GEM_MODE_1000BASET : GEM_MODE_100BASETX;
#endif
		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout    = MII_RESET_TIMEOUT;
		gcp->gc_mii_an_timeout       = MII_AN_TIMEOUT;
		gcp->gc_mii_an_wait          = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT; 

		/* workaround for PHY */
		gcp->gc_mii_an_delay        = ONESEC/10;
#if 0
		gcp->gc_mii_linkdown_action = MII_ACTION_NONE;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_NONE;
#else
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
#endif
		gcp->gc_mii_dont_reset      = TRUE;	/* XXX */

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &nfo_attach_chip;
		gcp->gc_reset_chip = &nfo_reset_chip;
		gcp->gc_init_chip  = &nfo_init_chip;
		gcp->gc_start_chip = &nfo_start_chip;
		gcp->gc_stop_chip  = &nfo_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &nfo_set_rx_filter;
		gcp->gc_set_media = &nfo_set_media;
		gcp->gc_get_stats = &nfo_get_stats;
		gcp->gc_interrupt = &nfo_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &nfo_tx_desc_write;
		gcp->gc_rx_desc_write = NULL;

		gcp->gc_tx_desc_stat = &nfo_tx_desc_stat;
		gcp->gc_rx_desc_stat = NULL;
		gcp->gc_tx_desc_init = &nfo_tx_desc_init;
		gcp->gc_rx_desc_init = NULL;
		gcp->gc_tx_desc_clean = &nfo_tx_desc_init;
		gcp->gc_rx_desc_clean = NULL;
		gcp->gc_get_packet    = NULL;

		/* mii operations */
		gcp->gc_mii_init = &gem_mii_init_default;
		gcp->gc_mii_config = &gem_mii_config_default;
		gcp->gc_mii_sync = &nfo_mii_sync;
		gcp->gc_mii_read = &nfo_mii_read;
		gcp->gc_mii_write = &nfo_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		lp = (struct nfo_dev *)
			kmem_zalloc(sizeof(struct nfo_dev), KM_SLEEP);
		lp->chip  = p;

		dp = gem_do_attach(dip, gcp, base, &regs_ha, lp, sizeof(*lp));

		kmem_free(gcp, sizeof(*gcp));
		if (dp == NULL) {
			cmn_err(CE_WARN, "!%s%d: gem_do_attach failed",
				drv_name, unit);
			goto err_free_mem;
		}

		return (DDI_SUCCESS);

err_free_mem:
		kmem_free(lp, sizeof(struct nfo_dev));
err:
		pci_config_teardown(&conf_handle);
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
nfodetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	gld_mac_info_t		*macinfo;
	struct gem_dev		*dp;
	struct nfo_dev		*lp;
	timeout_id_t		old_id;
	struct rxbuf		*rbp;
	struct rxbuf		*orbp;

        macinfo = (gld_mac_info_t *) ddi_get_driver_private(dip);
	dp = (struct gem_dev *) macinfo->gldm_private;
	lp = (struct nfo_dev *) dp->private;

	switch (cmd) {
	case DDI_DETACH:
		/*
		 * Ensure the mii timer routine stopped before destroying
		 * the adaptor object.
		 */
		if (dp->link_watcher_id != 0) {
			do {
				untimeout(old_id = dp->link_watcher_id);
			} while (old_id != dp->link_watcher_id);
			dp->link_watcher_id = 0;
		}

		/*
		 * XXX - We need to restore factory mac address.
		 * Once we changed the node addresss, there is no way
		 * to get the factory address.
		 */
		lp->pADApi->pfnSetNodeAddress(
			lp->pADApi->pADCX, lp->factory_mac_addr);

		/* free allocated resources */
		mutex_enter(&dp->intrlock);
		mutex_enter(&dp->xmitlock);
		nfo_destroy_adapter_object(dp);

		rbp = RBACT(&lp->rb_act)->dl_next;
		if (rbp != &lp->rb_act) {
			cmn_err(CE_WARN, "%s: %s active rxbuf leaked",
				dp->name, __func__);
			do {
				cmn_err(CE_CONT,
					"id %d (%p)", rbp->rxb_slot, rbp);
				orbp = rbp;
				rbp = RBACT(rbp)->dl_next;
				gem_free_rxbuf(orbp);
			} while (rbp != &lp->rb_act);
		}
		mutex_exit(&dp->xmitlock);
		mutex_exit(&dp->intrlock);

		return (gem_do_detach(dip));

	case DDI_SUSPEND:
		/*
		 * XXX - as we don't read mac address from the hardware
		 * register at DDI_RESUME, here we don't need to restore
		 * the factory mac address.
		 */
		return (gem_suspend(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
static	struct module_info nfominfo = {
	0,			/* mi_idnum */
	"nfo",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	TX_RING_SIZE*ETHERMAX,	/* mi_hiwat */
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
	D_NEW | D_MP	/* cb_flag */
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
	ddi_power	/* devo_power */
};

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
	status = mod_install(&modlinkage);

	return (status);
}

/*
 * _fini : 
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!nfo: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
