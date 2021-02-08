/*
 *  ife_gem.c: Intel 8255x 10/100 ethernet controler driver for Solaris
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
#pragma ident	"@(#)ife_gem.c	1.5 11/09/19"

/*
 Change log
09/22/2007 2.6.0 release

 */

/*
 TODO:
	pci device ID table
	extended TxCB support
	503 support for i82557
	auto-nego
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
#include "i8255xreg.h"

char	ident[] = "i8255x nic driver v" VERSION;

/*
 * Useful macros and typedefs
 */
#define	ROUNDUP(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#define	INT8(x)		(0xff & (x))
#define	INT16(x)	(0xffff & (x))

#define	FLUSH(dp)	(void)INW(dp, SCBSTAT)

#define	SETINTMASK(dp, mask)	OUTB((dp), SCBCMD+1, (mask)>>8)
/* Debugging support */
#ifdef DEBUG_LEVEL
static int ife_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (ife_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Descriptor implementation
 */
/* Command block size */
#define	MAXTXFRAGS	(min(8, GEM_MAXTXFRAGS))
#define	MAX_CB_SIZE	\
	ROUNDUP(sizeof (struct cb)+ \
	max(sizeof (struct tcb)+sizeof (struct tbd)*MAXTXFRAGS, 6*8+2), 8)
#define	MAX_CONFIG_SIZE		ROUNDUP(22, sizeof (uint32_t))
#define	MAX_STATISTICS_SIZE	128

/*
 * Our configuration
 */
#define	TX_BUF_SIZE	64
#define	TX_RING_SIZE	TX_BUF_SIZE
#define	RX_BUF_SIZE	256
#define	RX_RING_SIZE	RX_BUF_SIZE

#define	ONESEC			(drv_usectohz(1*1000000))

static int	ife_tx_copy_thresh = 256;
static int	ife_rx_copy_thresh = 256;

/*
 * Supported chips
 */
#define	REV_D101A4	4	/* i82558A */
#define	REV_D101B0	5	/* i82558B */
#define	REV_D101MA	8	/* i82559 */
#define	REV_D101S	9	/* i82669ER */
#define	REV_D102	12
#define	REV_D102C	13
#define	REV_D102E	15

#define	IS_i82557(rev)			((rev) < REV_D101A4)
#define	IS_i82558A_OR_LATER(rev)	((rev) >= REV_D101A4)
#define	IS_i82558B(rev)			((rev) == REV_D101B0)
#define	IS_i82559_OR_LATER(rev)		((rev) >= 6)
#define	IS_i82559ER(rev)		((rev) == REV_D101S)

struct chip_info {
	uint16_t	venid;
	uint16_t	devid;
	uint8_t		revmin;
	uint8_t		revmax;
	uint16_t	flags;
#define	ICH	0x0001
	char		*name;

} ife_chiptbl[] = {
	0x8086,	0x1229,	0x01, 0x03, 0, "i82557",
	0x8086,	0x1229,	0x04, 0x04, 0, "i82558 A Step",
	0x8086,	0x1229,	0x05, 0x05, 0, "i82558 B Step",
	0x8086,	0x1229,	0x06, 0x08, 0, "i82559",
	0x8086,	0x1229,	0x09, 0x09, 0, "i82559ER",
	0x8086,	0x1229,	0x0a, 0x0b, 0, "i82559",
	0x8086,	0x1229,	0x0c, 0x0e, 0, "i82550",
	0x8086,	0x1229,	0x0f, 0x10, 0, "i82551",
	0x8086,	0x1209,	0x00, 0xff, 0, "i82559ER",
	0x8086,	0x1029,	0x00, 0xff, 0, "i82559",
	0x8086,	0x1030,	0x00, 0xff, 0, "In business fast ethernet",
	0x8086,	0x1031,	0x00, 0xff, ICH, "PRO/100VE",	/* ICH3 */
	0x8086,	0x1032,	0x00, 0xff, ICH, "PRO/100VE",	/* ICH3 */
	0x8086,	0x1033,	0x00, 0xff, ICH, "PRO/100VM",	/* ICH3 */
	0x8086,	0x1034,	0x00, 0xff, ICH, "PRO/100VM",	/* ICH3 */
	0x8086,	0x1035,	0x00, 0xff, ICH, "i82562EH",	/* ICH3 */
	0x8086,	0x1036,	0x00, 0xff, ICH, "i82562EH",	/* ICH3 */
	0x8086,	0x1037,	0x00, 0xff, ICH, "i82562EH",	/* ICH3 */
	0x8086,	0x1038,	0x00, 0xff, ICH, "PRO/100VM",	/* ICH3 */
	0x8086,	0x1039,	0x00, 0xff, ICH, "PRO/100VE w/ 82562ET/EZ PHY",
	0x8086,	0x103a,	0x00, 0xff, ICH, "PRO/100VE w/ 82562ET/EZ(CNR)",
	0x8086,	0x103b,	0x00, 0xff, ICH, "PRO/100VM w/ 82562EM/EZ PHY",
	0x8086,	0x103c,	0x00, 0xff, ICH, "PRO/100VM w/ 82562EM/EZ(CNR)",
	0x8086,	0x103d,	0x00, 0xff, ICH, "PRO/100VE(mob)",
	0x8086,	0x103e,	0x00, 0xff, ICH, "PRO/100VM(mob)",
	0x8086,	0x1050,	0x00, 0xff, ICH, "PRO/100VM",	/* ICH5 */
	0x8086,	0x1051,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1052,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1053,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1054,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1055,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1056,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1057,	0x00, 0xff, ICH, "82559",	/* ICH5 */
	0x8086,	0x1059,	0x00, 0xff, 0, "PRO/100M",
	0x8086,	0x1064,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1065,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1066,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1067,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1068,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1069,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x106a,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x106b,	0x00, 0xff, ICH, "82559",	/* ICH6 */
	0x8086,	0x1091,	0x00, 0xff, ICH, "82559",	/* ICH7 */
	0x8086,	0x1092,	0x00, 0xff, ICH, "82559",	/* ICH7 */
	0x8086,	0x1093,	0x00, 0xff, ICH, "82559",	/* ICH7 */
	0x8086,	0x1094,	0x00, 0xff, ICH, "82559",	/* ICH7 */
	0x8086,	0x1095,	0x00, 0xff, ICH, "82559",	/* ICH7 */
	0x8086,	0x1227,	0x00, 0xff, 0, "i82559",
	0x8086,	0x1228,	0x00, 0xff, 0, "i82559",
	0x8086,	0x2449,	0x00, 0xff, ICH, "i82559",	/* ICH2 */
	0x8086,	0x2459,	0x00, 0xff, ICH, "i82559",	/* ICH2 */
	0x8086,	0x245d,	0x00, 0xff, ICH, "i82559",	/* ICH2 */
	0x8086,	0x27dc,	0x00, 0xff, ICH, "i82559",	/* ICH7 */
	0x8086,	0x5200,	0x00, 0xff, 0, "i82559",
	0x8086,	0x5201,	0x00, 0xff, 0, "i82559",
};
#define	CHIPTABLESIZE	(sizeof (ife_chiptbl)/sizeof (struct chip_info))

struct ife_dev {
	uint8_t			rev_id;
	uint16_t		intmask;	/* intmask field in scbcmd */
	struct chip_info	*hw_info;

	uint8_t			config[MAX_CONFIG_SIZE];

	int			eeprom_addr_bits;
	uint16_t		prom[128];

	uint8_t			mac_addr[ETHERADDRL];
	clock_t			last_stats_time;

	/* rx buffer management */
	struct rxbuf		*rbp[RX_RING_SIZE];

	/* cuc status */
	boolean_t		cuc_active;

	/* local lock */
	kmutex_t		reglock;
};

/* ======================================================== */

/* mii operations */
static void  ife_mii_sync(struct gem_dev *);
static uint16_t  ife_mii_read(struct gem_dev *, uint_t);
static void ife_mii_write(struct gem_dev *, uint_t, uint16_t);

/* nic operations */
static int ife_reset_chip(struct gem_dev *);
static int ife_init_chip(struct gem_dev *);
static int ife_start_chip(struct gem_dev *);
static int ife_stop_chip(struct gem_dev *);
static int ife_set_media(struct gem_dev *);
static int ife_set_rx_filter(struct gem_dev *);
static int ife_get_stats(struct gem_dev *);
static int ife_init_mac_addr(struct gem_dev *);

/* descriptor operations */
static int ife_cmd_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag);
static void ife_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static uint_t ife_cmd_desc_stat(struct gem_dev *dp, int slot, int ndesc);
static uint64_t ife_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void ife_cmd_desc_init(struct gem_dev *dp, int slot);
static void ife_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t ife_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr ife_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr ife_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t ife_dma_attr_txbuf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	MAXTXFRAGS,		/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t ife_dma_attr_rxbuf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	1,			/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t ife_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
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
 * misc subroutines
 */
/* ======================================================== */
/*
 * bcopy does not work correctly for little-endian attributed
 * memory on sparc.
 */
#ifdef notdef
static void
ife_bcopy_le(void *sp, void *dp, size_t len)
{
	int	i;

	for (i = 0; i < len; i++) {
		((uint8_t *)dp)[i] = ((uint8_t *)sp)[i];
	}
}
#else
/* the descripor area has native endian attibute now */
#define	ife_bcopy_le	bcopy
#endif

/* ======================================================== */
/*
 * HW manupilation subroutines
 */
/* ======================================================== */
static boolean_t
ife_issue_scbcmd(struct gem_dev *dp, uint16_t cmd, uint32_t arg)
{
	int	i;
	int	ret;
	struct ife_dev	*lp = dp->private;
	static uint16_t	prev_cmd = 0;

	ret = B_FALSE;

	mutex_enter(&lp->reglock);

	for (i = 0; INB(dp, SCBCMD) & (SC_CUC | SC_RUC); i++) {
		if (i > 100) {
			cmn_err(cmd == SC_CUC_RESUME ? CE_CONT : CE_WARN,
			    "!%s: %s: timeout, prev_cmd:%x, cmd:%x",
			    dp->name, __func__, prev_cmd, cmd);
			goto x;
		}
		if (i > 20) {
			drv_usecwait(10);
		}
	}

	ret = B_TRUE;

	switch (cmd) {
	case SC_CUC_START:
	case SC_CUC_LOADSDMP:
	case SC_CUC_LOADBASE:
	case SC_RUC_START:
	case SC_RUC_LOADHDS:
	case SC_RUC_LOADBASE:
		OUTL(dp, GENPTR, arg);
	}

	OUTB(dp, SCBCMD, cmd);
	prev_cmd = cmd;
x:
	mutex_exit(&lp->reglock);

	return (ret);
}

#define	LOCAL_OP	0x8
static int
ife_send_op(struct gem_dev *dp, uint32_t opcode)
{
	int	i;
	mblk_t	*mp;
	int	ret = GEM_SUCCESS;

	ASSERT(!mutex_owned(&dp->xmitlock));

	/* allocate a dummy mblk to fake gem_send_common */
	if ((mp = allocb(ETHERMIN, BPRI_MED)) == NULL) {
		cmn_err(CE_WARN, "!%s: %s: allocb failed", dp->name, __func__);
		return (GEM_FAILURE);
	}
#ifdef GEM_CONFIG_GLDv3
	/* avoid errors in vtag scanning */
	mp->b_wptr = mp->b_rptr + ETHERMIN;
	bzero(mp->b_rptr, ETHERMIN);
#endif
	for (i = 0; gem_send_common(dp, mp, opcode | LOCAL_OP); i++) {
		if (i > 100) {
			cmn_err(CE_WARN, "!%s: %s: failed, opcode:%d",
			    dp->name, __func__, opcode);
			freemsg(mp);
			ret = GEM_FAILURE;
			break;
		}
		gem_reclaim_txbuf(dp);
	}

	return (ret);
}

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */
static int
ife_reset_chip(struct gem_dev *dp)
{
	struct ife_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s", dp->name, __func__));

	/* clear pended interrupts */
	OUTW(dp, SCBSTAT, INW(dp, SCBSTAT));

	/* write a reset command into PORT register */
	OUTL(dp, PORT, PORT_SOFTRESET);

	/* wait 10 system clocks and 5 transmit clocks (10uS) */
	drv_usecwait(10);

	/* update software copy of device state */
	lp->cuc_active = B_FALSE;

	/* mask interrupt */
	lp->intmask = SC_M;
	SETINTMASK(dp, lp->intmask);

	return (GEM_SUCCESS);
}

static int
ife_init_chip(struct gem_dev *dp)
{
	int		i;
	struct ife_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* disable early Rx Int */
	OUTB(dp, EARLYRXINT, 0);

	/* disable flow control */
	OUTB(dp, FCTRL, 0);

	if (lp->rev_id >= 6 /* i82559 or later */) {
		/* clear general control register */
		OUTB(dp, GENCTRL, 0);
	}

	/* invalidate mac address */
	for (i = 0; i < ETHERADDRL; i++) {
		lp->mac_addr[i] = 0xff;
	}

	bzero(dp->io_area, MAX_STATISTICS_SIZE);

	/* load base addresses */
	(void) ife_issue_scbcmd(dp, SC_CUC_LOADBASE, 0);
	(void) ife_issue_scbcmd(dp, SC_RUC_LOADBASE, 0);
	(void) ife_issue_scbcmd(dp, SC_CUC_LOADSDMP, dp->io_area_dma);

	/* send first configure command to enable Tx/Rx */
	ife_send_op(dp, OP_CONFIGURE);

	/* kick CU */
	(void) ife_issue_scbcmd(dp, SC_CUC_START, dp->tx_ring_dma);
	lp->cuc_active = B_TRUE;

	return (GEM_SUCCESS);
}

static int
ife_start_chip(struct gem_dev *dp)
{
	struct ife_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(mutex_owned(&dp->intrlock));

	/* enable interrupts */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		lp->intmask &= ~SC_M;
		SETINTMASK(dp, lp->intmask);
	}

	/* as RU has halted, kick it now */
	(void) ife_issue_scbcmd(dp, SC_RUC_START,
	    dp->rx_buf_head->rxb_dmacookie[0].dmac_address);

	return (GEM_SUCCESS);
}

static int
ife_stop_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	stat;
	struct ife_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* stop rx */
	(void) ife_issue_scbcmd(dp, SC_RUC_ABORT, 0);

	/* wait for the nic becomes idle */
	i = (dp->speed == GEM_SPD_100) ? 20 : 200;
	while (1) {
		stat = INW(dp, SCBSTAT) & (SS_CUS | SS_RUS);
		if (stat == (SS_CUS_IDLE | SS_RUS_IDLE) ||
		    stat == (SS_CUS_SUSPENDED | SS_RUS_IDLE)) {
			break;
		}
		if (i-- == 0) {
			cmn_err(CE_NOTE, "%s: %s: timeout: stat:0x%x",
			    dp->name, __func__, stat);
			break;
		}
		drv_usecwait(10);
	}

	/* stop the hardware completely */
	(void) ife_reset_chip(dp);

	return (GEM_SUCCESS);
}

/*
 * EEPROM I/O routines
 */
#define	EEPROM_READ_CMD		6

/* 2 reads required for 66MHz operation */
#define	IFE_EEPROM_DELAY(dp)	{ INB(dp, EECTRL); INB(dp, EECTRL); }

static void
ife_check_eeprom_size(struct gem_dev *dp)
{
	int		i;
	int		addr_bits;
	uint8_t		chip_select;
	uint8_t		di;
	uint16_t	ret;
	struct ife_dev	*lp = dp->private;

	/* enable eeprom interface register */
	chip_select = EC_EECS;
	OUTB(dp, EECTRL, chip_select);

	/* output eeprom command */
	for (i = 4; i >= 0; i--) {
		di = ((EEPROM_READ_CMD >> i) & 1) << EC_EEDI_SHIFT;
		OUTB(dp, EECTRL, chip_select | di);
		OUTB(dp, EECTRL, chip_select | di | EC_EESK);
		IFE_EEPROM_DELAY(dp);

		OUTB(dp, EECTRL, chip_select | di);
		IFE_EEPROM_DELAY(dp);
	}

	/* How many address bits required  until eeprom responds with 0 */
	i = 0;
	do {
		OUTB(dp, EECTRL, chip_select);
		OUTB(dp, EECTRL, chip_select | EC_EESK);
		IFE_EEPROM_DELAY(dp);

		OUTB(dp, EECTRL, chip_select);
		IFE_EEPROM_DELAY(dp);
		i++;
	} while ((INB(dp, EECTRL) & EC_EEDO) && i < 8);

	/* save the result */
	lp->eeprom_addr_bits = i;

	/* read 16bits of data to terminate the seaquence */
	for (i = 16; i > 0; i--) {
		OUTB(dp, EECTRL, chip_select | EC_EESK);
		IFE_EEPROM_DELAY(dp);

		OUTB(dp, EECTRL, chip_select);
		IFE_EEPROM_DELAY(dp);
	}

	/* De-activate the EEPROM */
	OUTB(dp, EECTRL, 0);
}

static uint16_t
ife_read_eeprom(struct gem_dev *dp, uint_t addr)
{
	int		i;
	uint_t		cmd;
	uint8_t		chip_select;
	uint8_t		di;
	uint16_t	ret;
	struct ife_dev	*lp = dp->private;

	DPRINTF(3, (CE_CONT, "!%s: %s: called: addr_bits:%d",
	    dp->name, __func__, lp->eeprom_addr_bits));

	if (addr >= (1 << lp->eeprom_addr_bits)) {
		cmn_err(CE_WARN,
		    "!%s: %s: address(0x%x) must be lower than 0x%x",
		    dp->name, __func__, addr, 1 << lp->eeprom_addr_bits);
		return (0);
	}

	/* make command bits */
	cmd = (EEPROM_READ_CMD << lp->eeprom_addr_bits) | addr;

	/* enable eeprom interface register */
	chip_select = EC_EECS;
	OUTB(dp, EECTRL, chip_select);
	IFE_EEPROM_DELAY(dp);

	/* output eeprom command */
	for (i = 4 + lp->eeprom_addr_bits; i >= 0; i--) {
		di = ((cmd >> i) & 1) << EC_EEDI_SHIFT;
		OUTB(dp, EECTRL, chip_select | di);
		OUTB(dp, EECTRL, chip_select | di | EC_EESK);
		IFE_EEPROM_DELAY(dp);

		OUTB(dp, EECTRL, chip_select | di);
		IFE_EEPROM_DELAY(dp);
	}

	/* get returned value */
	ret = 0;
	for (i = 16; i > 0; i--) {
		/* get 1 bit */
		OUTB(dp, EECTRL, chip_select | EC_EESK);
		IFE_EEPROM_DELAY(dp);

		ret = (ret << 1)
		    | ((INB(dp, EECTRL) >> EC_EEDO_SHIFT) & 1);

		OUTB(dp, EECTRL, chip_select);
		IFE_EEPROM_DELAY(dp);
	}

	/* Terminate the EEPROM access. */
	OUTB(dp, EECTRL, 0);
	IFE_EEPROM_DELAY(dp);

	DPRINTF(3, (CE_CONT, "!%s: %s: returned 0x%x",
	    dp->name, __func__, ret));

	return (ret);
}

#ifdef DEBUG_LEVEL
static void
ife_eeprom_dump(struct gem_dev *dp)
{
	int		i;
	struct ife_dev	*lp = dp->private;

	for (i = 0; i < 128; i++) {
		lp->prom[i] = ife_read_eeprom(dp, i);
	}

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);
	for (i = 0; i < 128; i += sizeof (lp->prom[0])) {
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x ",
		    i, lp->prom[i], lp->prom[i+1],
		    lp->prom[i+2], lp->prom[i+3]);
	}
}
#endif /* DEBUG_LEVEL */

static uint8_t ife_config_default[24] = {
	/*  0 */ 22,		/* size of configure block itself */
	/*  1 */ 0x08,		/* Tx/Rx FIFO threshold */
	/*  2 */ 0,		/* atapive IFS */
	/*  3 */ 0,		/* terminale on CL/read AL/MWI enable */
	/*  4 */ 0,		/* Rx DMA max byte count */
	/*  5 */ 0,		/* Tx DMA max byte count */
	/*  6 */ 0x02 | C6_SaveBadFrames | C6_DiscardOverruns_ |
		C6_ExtStatCount_ | C6_ExtTCB_ | C6_CIInterrupt,
	/*  7 */ 0x00,
	/*  8 */ C8_MII,
	/*  9 */ 0x00,
	/* 10 */ 0x06 | C10_PreambleLen_7 | C10_NSAI,
	/* 11 */ 0x00,
	/* 12 */ C12_IFS_STD,
	/* 13 */ 0x00,
	/* 14 */ 0xf2,
	/* 15 */ 0x48,
	/* 16 */ 0x00,		/* FC delay LSB */
	/* 17 */ 0x40,		/* FC delay MSB */
	/* 18 */ 0xf0 | C18_Padding,
	/* 19 */ C19_AutoFDX,	/* Enable full duplex pin */
	/* 20 */ 0x3f,
	/* 21 */ 0x05,
	/* 22 */ 0x00,		/* pad for double word boundary */
	/* 23 */ 0x00,		/* pad for double word boundary */
};


static void
ife_flow_control(struct gem_dev *dp)
{
	struct ife_dev	*lp = dp->private;
	int		rev = lp->rev_id;

	/* setup current flow-control mode */

	if (IS_i82557(rev)) {
		/* i82557 doesn't support flow-control */
		return;
	}

	switch (dp->flow_control) {
	case FLOW_CONTROL_TX_PAUSE:
		/* send pause only */
		lp->config[16] = 0x00;
		lp->config[17] = 0x40;
		lp->config[19] &= ~(C19_RxFCRestart | C19_RxFCRestop);
		lp->config[19] |= C19_RejectFC;
		lp->config[19] &= ~C19_TxFC_;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		/* receive pause only */
		lp->config[16] = 0x011f & 0xff;
		lp->config[17] = 0x011f >> 8;
		lp->config[19] |=
		    (C19_RejectFC | C19_RxFCRestart | C19_RxFCRestop);
		lp->config[19] |= C19_TxFC_;
		break;

	case FLOW_CONTROL_SYMMETRIC:
		/* tx and rx pause */
		lp->config[16] = 0x011f & 0xff;
		lp->config[17] = 0x011f >> 8;
		lp->config[19] |=
		    (C19_RejectFC | C19_RxFCRestart | C19_RxFCRestop);
		lp->config[19] &= ~C19_TxFC_;
		break;

	case FLOW_CONTROL_NONE:
		/* Disable tx and rx flow-control */
		lp->config[16] = 0x00;
		lp->config[17] = 0x40;
		lp->config[19] &=
		    ~(C19_RejectFC | C19_RxFCRestart | C19_RxFCRestop);
		lp->config[19] |= C19_TxFC_;
		break;
	}
}

static int
ife_attach_chip(struct gem_dev *dp)
{
	int		i;
	uint16_t	val;
	struct ife_dev	*lp = dp->private;
	int		rev = lp->rev_id;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ife_check_eeprom_size(dp);
#if DEBUG_LEVEL > 3
	ife_eeprom_dump(dp);
#endif
	for (i = 0; i < ETHERADDRL; i += 2) {
		val = ife_read_eeprom(dp, i/2);
		dp->dev_addr.ether_addr_octet[i] = (uint8_t)val;
		dp->dev_addr.ether_addr_octet[i+1] = (uint8_t)(val >> 8);
	}

	/* clear statistics */
	bzero(&dp->stats, sizeof (dp->stats));

	/* make config parameters */
	bcopy(ife_config_default, lp->config, sizeof (ife_config_default));

	/* config[2]: we don't use adaptive ifs */

	if (IS_i82558A_OR_LATER(rev)) {
		lp->config[3] |= C3_MWIEn;
	}

	/* Rx DMA max byte cnt */
	lp->config[4] = min(0x7f, dp->rxmaxdma/4);

	/* Tx DMA max byte cnt */
	lp->config[5] = min(0x7f, dp->txmaxdma/4) | C5_DMBC;
#ifdef EXTEND_TCB
	if (IS_i82558A_OR_LATER(rev)) {
		lp->config[6] &= ~C6_ExtTCB_;
		if (IS_i82559_OR_LATER(rev)) {
			/* 82559 or later */
			lp->config[6] |= C6_ExtStatCount_;
			lp->config[6] |= C6_TCOStat;
		} else {
			lp->config[6] &= ~C6_ExtStatCount_;
			lp->config[6] &= ~C6_TCOStat;
		}
	}
#endif
	/* config 503/MII */
	if (IS_i82557(rev) && dp->mii_phy_addr == -1) {
		/* It's 503 */
		lp->config[8]  &= ~C8_MII;
		lp->config[15] |= C15_CRSCDT;
	}

	if (IS_i82558A_OR_LATER(rev)) {
		lp->config[12] |= C12_LinearPrioMode;
	}

#ifdef NEVER
	if (rev == REV_D101B0 /* 82558B */) {
		/* for IP checksum offload */
		lp->config[13] = 0;
		lp->config[14] = 0;
	}
#endif
	ife_flow_control(dp);

	if (IS_i82557(rev)) {
		/* i82557 does not support long packets */
		/*
		 * XXX - we cannot implement software vlan because
		 * i82557 doesn't receive any long packets.
		 */
		dp->mtu = min(ETHERMTU, dp->mtu);
	} else {
#ifdef GEM_CONFIG_GLDv3
		dp->mtu = min(IFE_MAX_MTU - 4, dp->mtu);
		lp->config[18] |= C18_LongRxOK;
		dp->misc_flag |= GEM_VLAN_SOFT;
#else
		dp->mtu = min(ETHERMTU, dp->mtu);
#endif
#ifdef notdef
		/* priority flow-control is not implemented */
		lp->config[18] &= ~C18_PrioFCThr;
		lp->config[18] |= 0 << C18_PrioFCThr_SHIFT;
		lp->config[20] &= ~C20_PrioFCLocation;
#endif
	}

	if (IS_i82558A_OR_LATER(rev) && !IS_i82559ER(rev)) {
		/* disable WOL */
		lp->config[19] |= C19_MagicPkt_;
	}

	/* i8255x requires control packets to be tranmitted */
	dp->misc_flag |= GEM_CTRL_PKT;

#if DEBUG_LEVEL > 1
	for (i = 0; i < lp->config[0]; i++) {
		cmn_err(CE_CONT, "!%d:0x%02x", i, lp->config[i]);
	}
#endif
	return (GEM_SUCCESS);
}

static int
ife_set_rx_filter(struct gem_dev *dp)
{
	uint8_t		c15_old;
	uint8_t		c21_old;
	uint8_t		*new_mac;
	struct ife_dev	*lp = dp->private;

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(!mutex_owned(&dp->xmitlock));

	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		new_mac = (void *)"\002\000\000\000\000\000";
	} else {
		new_mac = dp->cur_addr.ether_addr_octet;
	}

	/* check mac address */
	if (bcmp(new_mac, lp->mac_addr, ETHERADDRL) != 0) {
		bcopy(new_mac, lp->mac_addr, ETHERADDRL);
		if (ife_send_op(dp, OP_ADDRSETUP) != GEM_SUCCESS) {
			return (GEM_FAILURE);
		}
	}

	/* check multicast address */
	c15_old = lp->config[15];
	c21_old = lp->config[21];

	lp->config[15] &= ~C15_Promiscuous;
	lp->config[21] &= ~C21_MulticastAll;
	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		/* do nothing */
	} else if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscous */
		lp->config[15] |= C15_Promiscuous;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) ||
		dp->mc_count > (MAX_CB_SIZE-sizeof (struct cb)-2)/ETHERADDRL) {
		/* all multi */
		lp->config[21] |= C21_MulticastAll;
	} else {
		/* Normal mode */
		if (ife_send_op(dp, OP_MULTICAST) != GEM_SUCCESS) {
			return (GEM_FAILURE);
		}
	}

	if ((c15_old ^ lp->config[15]) | (c21_old ^ lp->config[21])) {
		if (ife_send_op(dp, OP_CONFIGURE) != GEM_SUCCESS) {
			return (GEM_FAILURE);
		}
	}

	return (GEM_SUCCESS);
}


static int
ife_set_media(struct gem_dev *dp)
{
	struct ife_dev	*lp = dp->private;

	ASSERT(mutex_owned(&dp->intrlock));
	ASSERT(!mutex_owned(&dp->xmitlock));

	/* update configuration */
	if ((lp->config[19] & C19_AutoFDX) == 0) {
		if (dp->full_duplex) {
			lp->config[19] |= C19_ForceFDX;
		} else {
			lp->config[19] &= ~C19_ForceFDX;
		}
	}

	ife_flow_control(dp);

	/* Notify current duplex mode to MAC */
	return (ife_send_op(dp, OP_CONFIGURE));
}

static int
ife_get_stats(struct gem_dev *dp)
{
	int		i;
	int		err;
	uint32_t	x;
	struct ife_dev	*lp = dp->private;
	uint32_t	*stat = (uint32_t *)dp->io_area;
	int		magic_off;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (!dp->mac_active) {
		/* dump base is not loaded yet */
		return (GEM_SUCCESS);
	}

	lp->last_stats_time = ddi_get_lbolt();

	/* using rx desc area for statistics information */
	if (IS_i82559_OR_LATER(lp->rev_id)) {
		magic_off = 80;
	} else if (IS_i82558A_OR_LATER(lp->rev_id)) {
		magic_off = 72;
	} else {
		magic_off = 64;
	}

	ddi_dma_sync(dp->desc_dma_handle,
	    (off_t)0, (size_t)(magic_off + sizeof (uint32_t)),
	    DDI_DMA_SYNC_FORKERNEL);

	if (stat[magic_off/4] == 0xA007) {

		dp->stats.excoll += x = stat[SO_TxMaxCollisionsErrors/4];
		dp->stats.errxmt += x;

		dp->stats.xmtlatecoll += x = stat[SO_TxLateCollisions/4];
		dp->stats.errxmt += x;

		dp->stats.underflow += x = stat[SO_TxUnderrunErrors/4];
		dp->stats.errxmt += x;

		dp->stats.nocarrier += x = stat[SO_TxLostCarrierSense/4];
		dp->stats.errxmt += x;

		dp->stats.defer += stat[SO_TxDeferred/4];
		dp->stats.first_coll += stat[SO_TxSingleCollision/4];
		dp->stats.multi_coll += stat[SO_TxMultipleCollision/4];
		dp->stats.collisions += stat[SO_TxTotalCollision/4];

		dp->stats.crc += x = stat[SO_RxCRCErrrs/4];
		dp->stats.errrcv += x;

		dp->stats.frame += x = stat[SO_RxAlignmentErrors/4];
		dp->stats.errrcv += x;

		dp->stats.missed += x = stat[SO_RxResourceErrors/4];
		dp->stats.errrcv += x;

		dp->stats.overflow += x = stat[SO_RxOverrunErrors/4];
		dp->stats.errrcv += x;

		dp->stats.runt += x = stat[SO_RxShortFrameErrors/4];
		dp->stats.errrcv += x;

		/* clear stamp */
		stat[magic_off/4] = 0;
	}

	if (stat[magic_off/4] == 0) {
		if (ife_issue_scbcmd(dp, SC_CUC_SDMPRST, 0)) {
			stat[magic_off/4] = 0xffff;
		}
	}

	return (GEM_SUCCESS);
}

static char *ife_cu_op_name[] = {
	"NOP",
	"ADDRSETUP",
	"CONFIGURE",
	"MULTICAST",
	"TX",
	"LOADMCODE",
	"DUMP",
	"DIAG",
};

#ifdef TXTIMEOUT_TEST
static int	ife_send_cnt;
#endif
static int
ife_cmd_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flag)
{
	int		i;
	uint32_t	tmp0;
	uint32_t	tmp1;
	int		local_flag;
	ddi_dma_cookie_t	*dcp;
	struct cb	*cbp;
	struct tcb	*tcbp;
	struct tbd	*tbdp;
	uint32_t	cbp_dma;
	uint8_t		*bp;
	uint_t		opcode;
	struct ife_dev	*lp = dp->private;

	local_flag = (flag & GEM_TXFLAG_PRIVATE) >> GEM_TXFLAG_PRIVATE_SHIFT;

	cbp = (void *)(dp->tx_ring + MAX_CB_SIZE * slot);
	cbp_dma = dp->tx_ring_dma + MAX_CB_SIZE * slot;
	bp = (uint8_t *)&cbp[1];

	opcode = OP_TX;
	if (local_flag) {
		opcode = local_flag & OPMASK;
	}

#if DEBUG_LEVEL > 3
	cmn_err(CE_CONT,
	    "!%s: %s: seqnum:%d, slot:%d, frags:%d flag:0x%llx, op:%s",
	    dp->name, __func__, dp->tx_desc_tail, slot, frags, flag,
	    ife_cu_op_name[opcode]);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!%d: addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
#if DEBUG_LEVEL > 3
	flag |= GEM_TXFLAG_INTR;
#endif

	switch (opcode) {
	case OP_NOP:		/* 0 */
		break;

	case OP_ADDRSETUP:	/* 1 */
		/* setup mac address */
		ife_bcopy_le(dp->dev_addr.ether_addr_octet, bp, ETHERADDRL);
		break;

	case OP_CONFIGURE:	/* 2 */
		/* set configure information */
		ife_bcopy_le(lp->config, bp, sizeof (lp->config));
		break;

	case OP_MULTICAST:	/* 3 */
		/* send multicast address list */
	ASSERT(dp->mc_count <= (MAX_CB_SIZE-sizeof (struct cb)-2)/ETHERADDRL);

		*(uint16_t *)bp = LE_16(dp->mc_count * ETHERADDRL);
		bp += sizeof (uint16_t);

		for (i = 0; i < dp->mc_count; i++) {
			ife_bcopy_le(dp->mc_list[i].addr.ether_addr_octet,
			    bp, ETHERADDRL);
			bp += ETHERADDRL;
#if DEBUG_LEVEL > 3
			cmn_err(CE_CONT,
			    "!%s: adding mcast(%d) "
			    "%02x:%02x:%02x:%02x:%02x:%02x",
			    dp->name, i,
			    dp->mc_list[i].addr.ether_addr_octet[0],
			    dp->mc_list[i].addr.ether_addr_octet[1],
			    dp->mc_list[i].addr.ether_addr_octet[2],
			    dp->mc_list[i].addr.ether_addr_octet[3],
			    dp->mc_list[i].addr.ether_addr_octet[4],
			    dp->mc_list[i].addr.ether_addr_octet[5]);
#endif
		}
		break;

	case OP_TX:	/* 4 */
		if (dp->speed == GEM_SPD_10 && !dp->full_duplex) {
			/* workaround for tx hang on 10Mbps half duplex */
			(void) ife_issue_scbcmd(dp, SC_CUC_NOP, 0);
			/* wait for 1uS at least */
			drv_usecwait(10);
		}

		tcbp = (struct tcb *)bp;
		tbdp = (struct tbd *)&tcbp[1];

		/* copy fragment list from dmacookie */
		for (i = 0; i < frags; i++, tbdp++) {
			tmp0 = dmacookie[i].dmac_size;
			tbdp->tb_size = LE_32(tmp0);
			tmp1 = dmacookie[i].dmac_address;
			tbdp->tb_addr = LE_32(tmp1);
		}

		/* make tcb */
		tmp0 = (frags << TCB_TBDNUM_SHIFT)
		    | (((dp->txthr + TXTHR_UNIT - 1) / TXTHR_UNIT)
		    << TCB_TXTHR_SHIFT);
		tcbp->tcb_ctrl = LE_32(tmp0);
#ifdef EXTEND_TCB
		if (IS_i82558A_OR_LATER(lp->rev_id)) {
			if (frags > 2) {
				tmp0 = cbp_dma +
				    sizeof (struct cb) + sizeof (struct tcb) +
				    2*sizeof (struct tbd);
				tcbp->tcb_tbdptr = LE_32(tmp0);
			} else {
				tcbp->tcb_tbdptr = LE_32(TBD_ADDR_NULL);
			}
		} else
#endif
		{
			tmp0 = cbp_dma +
			    sizeof (struct cb) + sizeof (struct tcb);
			tcbp->tcb_tbdptr = LE_32(tmp0);
		}
		break;

	default:
		cmn_err(CE_PANIC, "%s: %s: opcode:%x not supported",
		    dp->name, __func__, opcode);
		break;
	}

	/* make command block */
	tmp0 = opcode << CS_OP_SHIFT;
	if (flag & GEM_TXFLAG_INTR) {
		tmp0 |= CS_I;
	}
	if (opcode == OP_TX) {
		tmp0 |= CS_SF_;
	}
	cbp->cmdstat = LE_32(tmp0);

#if DEBUG_LEVEL > 2
	ASSERT((cbp_dma & (2*sizeof (uint32_t) - 1)) == 0);
#endif
#ifdef TXTIMEOUT_TEST
	ife_send_cnt++;
	if (ife_send_cnt > 100) {
	}
#endif
	return (1);
}

static void
ife_cmd_start(struct gem_dev *dp, int start_slot, int nslot)
{
	uint8_t		*bp;
	struct ife_dev	*lp = dp->private;
	const uint_t	tx_ring_size = dp->gc.gc_tx_ring_size;

	DPRINTF(1, (CE_CONT, "!%s: %s start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	/* Add suspend bit into last descriptor */
	bp = ((uint8_t *)dp->tx_ring) +
	    MAX_CB_SIZE * SLOT(start_slot + nslot - 1, tx_ring_size);
	bp[3] |= CS_S >> (8*3);

	/* flush new descriptors */
	gem_tx_desc_dma_sync(dp, start_slot, nslot, DDI_DMA_SYNC_FORDEV);

	/* remove suspend bit in the previous descriptor */
	bp = ((uint8_t *)dp->tx_ring) +
	    MAX_CB_SIZE * SLOT(start_slot - 1, tx_ring_size);
	bp[3] &= ~(CS_S >> (8*3));

	/* flush it */
	gem_tx_desc_dma_sync(dp,
	    SLOT(start_slot - 1, tx_ring_size), 1, DDI_DMA_SYNC_FORDEV);

	/* Here we resume potentially supended CU. */
	if (lp->cuc_active) {
		(void) ife_issue_scbcmd(dp, SC_CUC_RESUME, 0);
	}
}

static void
ife_rx_desc_write(struct gem_dev *dp, int slot,
	    ddi_dma_cookie_t *dmacookie, int frags)
{
	int		i;
	uint32_t	tmp;
	struct rxbuf	*rbp;
	struct rfd	*rfd;
	struct ife_dev	*lp = dp->private;

#if DEBUG_LEVEL > 1
	cmn_err(CE_CONT,
	    "!%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, "!  frag: %d addr: 0x%x, len: 0x%x",
		    i, dmacookie[i].dmac_address, dmacookie[i].dmac_size);
	}
#endif
	ASSERT(frags == 1);

	rbp = dp->rx_buf_tail;
	lp->rbp[slot] = rbp;

	rfd = (struct rfd *)rbp->rxb_buf;
	rfd->cmdstat = LE_32(CS_EL);	/* end of list */
	rfd->link = 0;
	tmp = (((uint32_t)dmacookie->dmac_size) - sizeof (struct rfd))
	    << RFD_SIZE_SHIFT;
	rfd->size = LE_32(tmp);

	/* append this frame to the hardware rx list */
	rbp = lp->rbp[SLOT(slot - 1, dp->gc.gc_rx_ring_size)];
	if (rbp) {
		/* Link this to the previous rfd */
		/* XXX - we left the suspend bit set */
		rfd = (void *)rbp->rxb_buf;
		tmp = (uint32_t)dmacookie->dmac_address;
		rfd->link = LE_32(tmp);
	}
}

static void
ife_rx_start(struct gem_dev *dp, int start_slot, int nslot)
{
	int		slot;
	struct rxbuf	*rbp;
	struct rfd	*rfd;
	uint8_t		mark;
	struct ife_dev	*lp = dp->private;
	const uint_t	rx_ring_size = dp->gc.gc_rx_ring_size;

	DPRINTF(1, (CE_CONT, "!%s: %s start:%d, nslot:%d",
	    dp->name, __func__, start_slot, nslot));

	/*
	 * As i8255x doesn't have a rx descriptor ring, no need
	 * to flush that.
	 */
	slot = start_slot + nslot;
	mark = 0;
	while (slot-- != start_slot) {
		rbp = lp->rbp[SLOT(slot, rx_ring_size)];

		/* remove end_of_list bit except for the last descriptor */
		rfd = (struct rfd *)rbp->rxb_buf;
		((uint8_t *)&rfd->cmdstat)[3] &= ~mark;

		ddi_dma_sync(rbp->rxb_dh,
		    0, sizeof (struct rfd), DDI_DMA_SYNC_FORDEV);
		mark = CS_EL >> 24;
	}

	rbp = lp->rbp[SLOT(slot, rx_ring_size)];
	if (rbp) {
		/* append new rx buffers to the hardware rx list */

		/* remove suspend bit except */
		rfd = (void *)rbp->rxb_buf;
		((uint8_t *)&rfd->cmdstat)[3] &= ~(CS_EL >> 24);

		ddi_dma_sync(rbp->rxb_dh,
		    0, sizeof (struct rfd), DDI_DMA_SYNC_FORDEV);
	}
}

static uint_t
ife_cmd_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	struct cb	*cbp;
	uint32_t	cstat;
	struct ife_dev	*lp = dp->private;

	cbp = (void *)(dp->tx_ring + MAX_CB_SIZE * slot);

	cstat = cbp->cmdstat;
	cstat = LE_32(cstat);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, cmdstat:0x%b op:%s scbstat:0x%04x",
	    dp->name, __func__, slot, cstat, CS_BITS,
	    ife_cu_op_name[(cstat & CS_OP) >> CS_OP_SHIFT],
	    INW(dp, SCBSTAT)));

	if ((cstat & CS_C) == 0) {
		/* not completed */
		return (0);
	}

	/* tx error statitcs */
	if (cstat & CS_U) {
		/* tx underrun happened */
		dp->txthr = min(TXTHR_SF*TXTHR_UNIT, dp->txthr + 64);
	}

	return (GEM_TX_DONE);
}

#ifdef DEBUG_LEVEL
static void
ife_dump_packet(struct gem_dev *dp, uint8_t *bp, int n)
{
	int	i;

	for (i = 0; i < n; i += 8, bp += 8) {
		cmn_err(CE_CONT, "%02x %02x %02x %02x %02x %02x %02x %02x",
		    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7]);
	}
}
#endif

static uint64_t
ife_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint_t		flag;
	uint_t		len;
	uint16_t	cstat;
	struct rxbuf	*rbp;
	struct rfd	*rfd;
	struct ife_dev	*lp = dp->private;

	rbp = lp->rbp[slot];
	ASSERT(rbp != NULL);
#ifdef notdef
	ddi_dma_sync(rbp->rxb_dh, 0,
	    sizeof (struct rfd), DDI_DMA_SYNC_FORKERNEL);
#endif
	rfd = (struct rfd *)rbp->rxb_buf;
	len = rfd->size;
	len = LE_32(len) & RFD_COUNT;
	cstat = rfd->cmdstat;
	cstat = LE_32(cstat);

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: slot:%d, rx_cstat:0x%b framesize:0x%x",
	    dp->name, __func__, slot, cstat, CS_BITS, len));

	if ((cstat & CS_C) == 0) {
		/* not received yet */
		return (0);
	}

	lp->rbp[slot] = NULL;

	flag = GEM_RX_DONE;
	if ((cstat & CS_OK) == 0) {
		flag |= GEM_RX_ERR;
	}
#if DEBUG_LEVEL > 3
	ife_dump_packet(dp, dp->rx_buf_head->rxb_buf, len);
#endif
	return (flag | len);
}

static void
ife_cmd_desc_init(struct gem_dev *dp, int slot)
{
	struct cb	*cbp;
	uint32_t	cbp_dma;
	uint32_t	link;
	struct ife_dev	*lp = dp->private;
	const uint_t	tx_ring_size = dp->gc.gc_tx_ring_size;

	/*
	 * Make cmd block ring structure
	 */
	cbp = (void *)(dp->tx_ring + MAX_CB_SIZE * slot);

	/* link to the next cb in the command block ring */
	link = dp->tx_ring_dma + MAX_CB_SIZE * SLOT(slot + 1, tx_ring_size);
	cbp->link = LE_32(link);
}

static void
ife_rx_desc_init(struct gem_dev *dp, int slot)
{
	struct ife_dev	*lp = dp->private;

	lp->rbp[slot] = NULL;
}

/*
 * Device depend interrupt handler
 */
#define	INTRBITS	\
	(SS_CX | SS_FR | SS_CNA | SS_RNR | SS_MDI | SS_SWI | SS_FCP)
static uint_t
ife_interrupt(struct gem_dev *dp)
{
	uint16_t	stat;
	uint_t		flag = 0;
	struct ife_dev	*lp = dp->private;
	uint16_t	intmask_saved = lp->intmask;

	stat = INW(dp, SCBSTAT);
	if (!((lp->intmask & SC_M) == 0 && (stat & INTRBITS))) {
		/* Not for us */
		return (DDI_INTR_UNCLAIMED);
	}
#ifndef CONFIG_OPT_IO
	/* Before we clear PCI interrupt signal line, mask all interrupts */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		lp->intmask |= SC_M;
		SETINTMASK(dp, lp->intmask);
	}
#endif
	DPRINTF(2, (CE_CONT, "!%s: Interrupt, scbstat: %b",
	    dp->name, stat, SCBSTAT_BITS));

	/* clear interrupt */
	OUTW(dp, SCBSTAT, stat);
#ifdef MAP_MEM
	FLUSH(dp);
#endif

	if (!dp->mac_active) {
#ifdef CONFIG_OPT_IO
		/* the device is not active, no more interrupts */
		lp->intmask |= SC_M;
		SETINTMASK(dp, lp->intmask);
#endif
		return (DDI_INTR_CLAIMED);
	}

	if (stat & (SS_FR | SS_RNR)) {
		/* packets were received */
		(void) gem_receive(dp);

		if (stat & SS_RNR) {
			/* as RU has halted, kick it now */
			DPRINTF(0, (CE_NOTE, "!%s: RU not ready", dp->name));
			(void) ife_issue_scbcmd(dp, SC_RUC_START,
			    dp->rx_buf_head->rxb_dmacookie[0].dmac_address);
		}
	}

	if (stat & SS_CX) {
		/* cmd has processed */
		if (gem_tx_done(dp)) {
			flag |= INTR_RESTART_TX;
		}
	}
#ifndef CONFIG_OPT_IO
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		/* restore interrupt mask */
		lp->intmask = intmask_saved;
		SETINTMASK(dp, lp->intmask);
	}
#endif
	return (DDI_INTR_CLAIMED | flag);
}

/*
 * HW depend MII routine
 */
static int
ife_mii_config(struct gem_dev *dp)
{
	uint16_t	val;
	struct ife_dev	*lp = dp->private;

	if ((dp->mii_phy_id & 0xfffffff0) == 0x20005c00) {
		/* configure DP83840/DP83840A */
		val = gem_mii_read(dp, 23);
		gem_mii_write(dp, 23, val | 0x0422 | 0x0100);
	}

	return (gem_mii_config_default(dp));
}

static void
ife_mii_sync(struct gem_dev *dp)
{
	/* do nothing */
}

static uint16_t
ife_mii_read(struct gem_dev *dp, uint_t reg)
{
	uint32_t	val;
	int		i;
	struct ife_dev	*lp = dp->private;

	if (dp->mac_active &&
	    (ddi_get_lbolt() - lp->last_stats_time) > ONESEC) {
#if 1 /* for debug */
		ife_get_stats(dp);
#endif
		if (IS_i82557(lp->rev_id) &&
		    ((uint32_t *)dp->io_area)[SO_RxGoodFrames/4] == 0) {
			ife_send_op(dp, OP_MULTICAST);
		}
	}

	OUTL(dp, MDICTRL,
	    MC_OP_READ | (dp->mii_phy_addr << MC_PHYADD_SHIFT) |
	    (reg << MC_REGADD_SHIFT));
	FLUSH(dp);

	/* wait until done */
	drv_usecwait(40);
	for (i = 0; (((val = INL(dp, MDICTRL)) & MC_R) == 0); i++) {
		if (i > 100) {
			cmn_err(CE_CONT, "!%s: %s: timeout: 0x%b",
			    dp->name, __func__, val, MII_STATUS_BITS);
			return (0);
		}
		drv_usecwait(20);
	}
	DPRINTF(4, (CE_CONT, "!%s: %s: done: 0x%04x",
	    dp->name, __func__, (uint16_t)val));
	return (val & MC_DATA);
}

static void
ife_mii_write(struct gem_dev *dp, uint_t reg, uint16_t val)
{
	int		i;

	OUTL(dp, MDICTRL,
	    MC_OP_WRITE | (dp->mii_phy_addr << MC_PHYADD_SHIFT) |
	    (reg << MC_REGADD_SHIFT) | val);
	FLUSH(dp);

	i = 0;
	do {
		drv_usecwait(20);
		if (i++ > 100) {
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			return;
		}
	} while ((INL(dp, MDICTRL) & MC_R) == 0);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */
static int
ifeattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			unit;
	const char		*drv_name;
	int			i;
	ddi_acc_handle_t	conf_handle;
	int			vid;
	int			did;
	int			svid;
	int			ssid;
	uint8_t			revid;
	struct chip_info	*p;
	struct gem_dev		*dp;
	struct ife_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called", drv_name, unit, __func__));

	/*
	 * Check if chip is supported.
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!%s%d: %s: pci_config_setup failed",
		    drv_name, unit, __func__);
		goto err;
	}

	vid = pci_config_get16(conf_handle, PCI_CONF_VENID);
	did = pci_config_get16(conf_handle, PCI_CONF_DEVID);
	svid = pci_config_get16(conf_handle, PCI_CONF_SUBVENID);
	ssid = pci_config_get16(conf_handle, PCI_CONF_SUBSYSID);
	revid = pci_config_get8(conf_handle, PCI_CONF_REVID);

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure the pmr status is D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);

	pci_config_teardown(&conf_handle);

	switch (cmd) {
	case DDI_RESUME:
		return (gem_resume(dip));

	case DDI_ATTACH:
		for (i = 0, p = ife_chiptbl; i < CHIPTABLESIZE; i++, p++) {
			if (((p->venid == vid && p->devid == did) ||
			    (p->venid == svid && p->devid == ssid)) &&
			    (p->revmin <= revid && revid <= p->revmax)) {
				/* found */
				goto chip_found;
			}
		}

		/* Not found */
		cmn_err(CE_WARN,
		    "!%s%d: %s: wrong PCI venid/devid (0x%x, 0x%x)",
		    drv_name, unit, __func__, vid, did);
		goto err;
chip_found:
		cmn_err(CE_CONT,
		    "!%s%d: %s (vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
		    drv_name, unit, p->name, vid, did, revid);
		/* fix revision id */
		if (revid == 0xff) {
			revid = 1;
		}
		if (p->flags & ICH) {
			revid = REV_D101MA;
		}

		if (gem_pci_regs_map_setup(dip,
#ifdef MAP_MEM
		    PCI_ADDR_MEM32, PCI_ADDR_MASK,
#else
		    PCI_ADDR_IO, PCI_ADDR_MASK,
#endif
		    &ife_dev_attr, (void *)&base, &regs_ha)
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
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		gcp->gc_tx_max_descs_per_pkt = 1;
		gcp->gc_tx_desc_unit_shift = 7;	/* 128 byte */
		gcp->gc_tx_buf_size = TX_BUF_SIZE;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size;
		gcp->gc_tx_ring_size = TX_RING_SIZE;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_FALSE;
		gcp->gc_tx_copy_thresh = ife_tx_copy_thresh;
		gcp->gc_tx_desc_write_oo = B_FALSE;

		gcp->gc_rx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = -1; /* no rx desc */
		gcp->gc_rx_ring_size = RX_BUF_SIZE;
		gcp->gc_rx_buf_max = gcp->gc_rx_ring_size - 1;
		gcp->gc_rx_copy_thresh = ife_rx_copy_thresh;
		gcp->gc_rx_header_len = sizeof (struct rfd);

		gcp->gc_io_area_size = MAX_STATISTICS_SIZE;

		/* map attributes */
		gcp->gc_dev_attr = ife_dev_attr;
		gcp->gc_buf_attr = ife_buf_attr;
		gcp->gc_desc_attr = ife_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = ife_dma_attr_desc;
		gcp->gc_dma_attr_txbuf = ife_dma_attr_txbuf;
		gcp->gc_dma_attr_rxbuf = ife_dma_attr_rxbuf;

		/* time out parameters */
		gcp->gc_tx_timeout = GEM_TX_TIMEOUT;
		gcp->gc_tx_timeout_interval = GEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		gcp->gc_flow_control = IS_i82558A_OR_LATER(revid)
		    ? FLOW_CONTROL_RX_PAUSE : FLOW_CONTROL_NONE;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval =GEM_LINK_WATCH_INTERVAL;
		gcp->gc_mii_an_watch_interval = GEM_LINK_WATCH_INTERVAL/5;
		gcp->gc_mii_reset_timeout = MII_RESET_TIMEOUT;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* MII work arounds */
		gcp->gc_mii_addr_min = 0;
		gcp->gc_mii_an_delay = 0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RSA;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_an_oneshot = B_FALSE;

		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &ife_attach_chip;
		gcp->gc_reset_chip = &ife_reset_chip;
		gcp->gc_init_chip = &ife_init_chip;
		gcp->gc_start_chip = &ife_start_chip;
		gcp->gc_stop_chip = &ife_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &ife_set_rx_filter;
		gcp->gc_set_media = &ife_set_media;
		gcp->gc_get_stats = &ife_get_stats;
		gcp->gc_interrupt = &ife_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &ife_cmd_desc_write;
		gcp->gc_rx_desc_write = &ife_rx_desc_write;
		gcp->gc_tx_start = &ife_cmd_start;
		gcp->gc_rx_start = &ife_rx_start;

		gcp->gc_tx_desc_init = &ife_cmd_desc_init;
		gcp->gc_rx_desc_init = &ife_rx_desc_init;
		gcp->gc_tx_desc_stat = &ife_cmd_desc_stat;
		gcp->gc_rx_desc_stat = &ife_rx_desc_stat;
		gcp->gc_tx_desc_clean = &ife_cmd_desc_init;
		gcp->gc_rx_desc_clean = &ife_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &gem_mii_probe_default;
		gcp->gc_mii_init = NULL;
		gcp->gc_mii_config = &ife_mii_config;
		gcp->gc_mii_sync = &ife_mii_sync;
		gcp->gc_mii_read = &ife_mii_read;
		gcp->gc_mii_write = &ife_mii_write;
		gcp->gc_mii_tune_phy = NULL;

		/* MSI/MSIX interrupts */
		gcp->gc_nintrs_req = 1;

		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_max_mtu = IFE_MAX_MTU - 4;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		lp = kmem_zalloc(sizeof (struct ife_dev), KM_SLEEP);

		lp->rev_id = revid;
		lp->hw_info = p;
		mutex_init(&lp->reglock, NULL, MUTEX_DRIVER, NULL);

		dp = gem_do_attach(dip, 0,
		    gcp, base, &regs_ha, lp, sizeof (*lp));

		kmem_free(gcp, sizeof (*gcp));

		if (dp) {
			return (DDI_SUCCESS);
		}

		mutex_destroy(&lp->reglock);
err_free_mem:
		kmem_free(lp, sizeof (struct ife_dev));
err:
		return (DDI_FAILURE);
	}
	return (DDI_FAILURE);
}

static int
ifedetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;
	struct gem_dev	*dp;
	struct ife_dev	*lp;

	dp = GEM_GET_DEV(dip);
	lp = dp->private;

	switch (cmd) {
	case DDI_SUSPEND:
		return (gem_suspend(dip));

	case DDI_DETACH:

		/* free driver depend resource */
		mutex_destroy(&lp->reglock);

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
GEM_STREAM_OPS(ife_ops, ifeattach, ifedetach);
#else
static	struct module_info ifeminfo = {
	0,			/* mi_idnum */
	"ife",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64 * 1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit iferinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ifeminfo,		/* qi_minfo */
	NULL,			/* qi_mstat */
};

static	struct qinit ifewinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&ifeminfo,		/* qi_minfo */
	NULL,			/* qi_mstat */
};

static struct streamtab	ife_info = {
	&iferinit,	/* st_rdinit */
	&ifewinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL,		/* st_muxwrinit */
};

static	struct cb_ops cb_ife_ops = {
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
	&ife_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops ife_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	ifeattach,	/* devo_attach */
	ifedetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_ife_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power,	/* devo_power */
};
#endif

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&ife_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!ife: _init: called"));
	gem_mod_init(&ife_ops, "ife");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&ife_ops);
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

	DPRINTF(2, (CE_CONT, "!ife: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&ife_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
