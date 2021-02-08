/*
 * run_uwgem.c: Ralink RT2870 usb 802.11n WIFI Driver for Solaris
 */

/* --- copyright from if_run.c --- */

/*	$OpenBSD: if_run.c,v 1.35 2009/12/10 21:04:16 oga Exp $	*/

/*-
 * Copyright (c) 2008,2009 Damien Bergamini <damien.bergamini@free.fr>
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

/* --- copyright from rt2870.c in opensolaris --- */
/*-
 * Ralink Technology RT2700U/RT2800U/RT3000U chipset driver.
 * http://www.ralinktech.com/
 */

/*
 * Copyright 2009 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

/*
 * Copyright (c) 2007, 2008
 *	Damien Bergamini <damien.bergamini@free.fr>
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
 * Ralink Technology RT2870 chipset driver
 * http://www.ralinktech.com/
 */

#pragma ident "@(#)run_uwgem.c	1.6 16/05/22"

/*
 *  Changelog:
 */

/*
 * TODO
 *
 */
/* ======================================================= */

/*
 * Solaris system header files and macros
 */

/* minimum kernel headers for drivers */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/sysmacros.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/byteorder.h>

/* ethernet stuff */
#include <sys/ethernet.h>

/* interface card depend stuff */
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strsun.h>
#include <sys/strlog.h>
#include <sys/usb/usba.h>

#include <sys/mac.h>
#ifndef MAC_VERSION
#include <sys/mac_provider.h>
#endif
#include <sys/mac_ether.h>
#if 0
#include <sys/mac_wifi.h>
#endif
#include <sys/net80211.h>
#include <sys/net80211_proto.h>

#include "uwgem.h"

/* hardware stuff */
#include "rt2860_reg.h"
#include "rt2870_reg.h"

#include "rt2870_var.h"

char	ident[] = "RT2870 driver v" VERSION;

/*
 * Useful macros
 */
#define	CHECK_AND_JUMP(err, label)	if (err != USB_SUCCESS) goto label
#define	LE16P(p)	((((uint8_t *)(p))[1] << 8) | ((uint8_t *)(p))[0])

#define	N(a)	(sizeof (a) / sizeof ((a)[0]))

#define	DELAY(n)	drv_usecwait(n)

#define	CHIP_RT3070(r)	(((r) >> 16) == 0x3070)
#define	CHIP_RT3071(r)	(((r) >> 16) == 0x3071)
#define	CHIP_RT3090A(r)	(((r) >> 16) == 0x3090)
#define	CHIP_RT3390(r)	(((r) >> 16) == 0x3390)
#define	CHIP_RT3572(r)	(((r) >> 16) == 0x3572)
#define	CHIP_RT3090(r)	(CHIP_RT3071(r) || CHIP_RT3090A(r))
#define	CHIP_RT30XX(r)	((((r) >> 20) == 0x307) || CHIP_RT3090A(r))

/*
 * Debugging
 */

/* Debug Flags */
#define IEEE80211_MSG_BRUSSELS  0x80000000      /* BRUSSELS */
#define IEEE80211_MSG_DEBUG     0x40000000      /* IFF_DEBUG equivalent */
#define IEEE80211_MSG_DUMPPKTS  0x20000000      /* IFF_LINK2 equivalant */
#define IEEE80211_MSG_CRYPTO    0x10000000      /* crypto work */
#define IEEE80211_MSG_INPUT     0x08000000      /* input handling */
#define IEEE80211_MSG_XRATE     0x04000000      /* rate set handling */
#define IEEE80211_MSG_ELEMID    0x02000000      /* element id parsing */
#define IEEE80211_MSG_NODE      0x01000000      /* node handling */
#define IEEE80211_MSG_ASSOC     0x00800000      /* association handling */
#define IEEE80211_MSG_AUTH      0x00400000      /* authentication handling */
#define IEEE80211_MSG_SCAN      0x00200000      /* scanning */
#define IEEE80211_MSG_OUTPUT    0x00100000      /* output handling */
#define IEEE80211_MSG_STATE     0x00080000      /* state machine */
#define IEEE80211_MSG_POWER     0x00040000      /* power save handling */
#define IEEE80211_MSG_DOT1X     0x00020000      /* 802.1x authenticator */
#define IEEE80211_MSG_DOT1XSM   0x00010000      /* 802.1x state machine */
#define IEEE80211_MSG_RADIUS    0x00008000      /* 802.1x radius client */
#define IEEE80211_MSG_RADDUMP   0x00004000      /* dump 802.1x radius packets */
#define IEEE80211_MSG_RADKEYS   0x00002000      /* dump 802.1x keys */
#define IEEE80211_MSG_WPA       0x00001000      /* WPA/RSN protocol */
#define IEEE80211_MSG_ACL       0x00000800      /* ACL handling */
#define IEEE80211_MSG_WME       0x00000400      /* WME protocol */
#define IEEE80211_MSG_SUPERG    0x00000200      /* Atheros SuperG protocol */
#define IEEE80211_MSG_DOTH      0x00000100      /* 802.11h support */
#define IEEE80211_MSG_INACT     0x00000080      /* inactivity handling */
#define IEEE80211_MSG_ROAM      0x00000040      /* sta-mode roaming */
#define IEEE80211_MSG_CONFIG    0x00000020      /* wificonfig/dladm */
#define IEEE80211_MSG_ACTION    0x00000010      /* action frame handling */
#define IEEE80211_MSG_HT        0x00000008      /* 11n mode debug */
#define IEEE80211_MSG_ANY       0xffffffff      /* anything */

#ifdef DEBUG_LEVEL
static int rt2870_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (rt2870_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

#define	RT2870_DBG_80211	(1 << 0)
#define	RT2870_DBG_DMA		(1 << 1)
#define	RT2870_DBG_EEPROM	(1 << 2)
#define	RT2870_DBG_FW		(1 << 3)
#define	RT2870_DBG_HW		(1 << 4)
#define	RT2870_DBG_INTR		(1 << 5)
#define	RT2870_DBG_RX		(1 << 6)
#define	RT2870_DBG_SCAN		(1 << 7)
#define	RT2870_DBG_TX		(1 << 8)
#define	RT2870_DBG_RADIO	(1 << 9)
#define	RT2870_DBG_RESUME	(1 << 10)
#define	RT2870_DBG_MSG		(1 << 11)

static uint32_t rt2870_dbg_flags =
	RT2870_DBG_MSG |
	RT2870_DBG_EEPROM |
	RT2870_DBG_FW |
	RT2870_DBG_SCAN |
#if 0
	RT2870_DBG_RADIO |
	RT2870_DBG_RX |
#endif
	RT2870_DBG_TX |
	RT2870_DBG_80211 |
	0;

#ifdef DEBUG
#define	RUN_DEBUG \
	rt2870_debug_printf
#else
#define	RUN_DEBUG
#endif

/*
 * Our configration
 */
/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))


/*
 * RX/TX buffer size
 */

/*
 * Local device definitions
 */
/* see rt2870_var.h */

/*
 * Read only variables
 */
static uint8_t rt2870_new2[] = {
#include "microcode.hex"
};

static const struct ieee80211_rateset rt2870_rateset_11b =
	{ 4, { 2, 4, 11, 22 } };
#if 1
static const struct ieee80211_rateset rt2870_rateset_11g =
	{ 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };

static const struct ieee80211_rateset rt2870_rateset_11a =
	{ 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };
#else
/* N.B. - don't include 11b rates in 11g rateset */
static const struct ieee80211_rateset rt2870_rateset_11g =
	{ 8, { 12, 18, 24, 36, 48, 72, 96, 108 } };

static const struct ieee80211_rateset rt2870_rateset_11a =
	{ 8, { 12, 18, 24, 36, 48, 72, 96, 108 } };
#endif

static const struct {
	uint32_t	reg;
	uint32_t	val;
} rt2870_def_mac[] = {
	RT2870_DEF_MAC
};

static const struct {
	uint8_t	reg;
	uint8_t	val;
} rt2870_def_bbp[] = {
	RT2860_DEF_BBP
};

static const struct rfprog {
	uint8_t		chan;
	uint32_t	r1, r2, r3, r4;
} rt2860_rf2850[] = {
#ifdef NEW_RF2850
	{1, 0x98402ecc, 0x984c0786, 0x9816b455, 0x9800510b},
	{2, 0x98402ecc, 0x984c0786, 0x98168a55, 0x9800519f},
	{3, 0x98402ecc, 0x984c078a, 0x98168a55, 0x9800518b},
	{4, 0x98402ecc, 0x984c078a, 0x98168a55, 0x9800519f},
	{5, 0x98402ecc, 0x984c078e, 0x98168a55, 0x9800518b},
	{6, 0x98402ecc, 0x984c078e, 0x98168a55, 0x9800519f},
	{7, 0x98402ecc, 0x984c0792, 0x98168a55, 0x9800518b},
	{8, 0x98402ecc, 0x984c0792, 0x98168a55, 0x9800519f},
	{9, 0x98402ecc, 0x984c0796, 0x98168a55, 0x9800518b},
	{10, 0x98402ecc, 0x984c0796, 0x98168a55, 0x9800519f},
	{11, 0x98402ecc, 0x984c079a, 0x98168a55, 0x9800518b},
	{12, 0x98402ecc, 0x984c079a, 0x98168a55, 0x9800519f},
	{13, 0x98402ecc, 0x984c079e, 0x98168a55, 0x9800518b},
	{14, 0x98402ecc, 0x984c07a2, 0x98168a55, 0x98005193},

	/* 802.11 UNI / HyperLan 2 */
	{36, 0x98402ecc, 0x984c099a, 0x98158a55, 0x980ed1a3},
	{38, 0x98402ecc, 0x984c099e, 0x98158a55, 0x980ed193},
	{40, 0x98402ec8, 0x984c0682, 0x98158a55, 0x980ed183},
	{44, 0x98402ec8, 0x984c0682, 0x98158a55, 0x980ed1a3},
	{46, 0x98402ec8, 0x984c0686, 0x98158a55, 0x980ed18b},
	{48, 0x98402ec8, 0x984c0686, 0x98158a55, 0x980ed19b},
	{52, 0x98402ec8, 0x984c068a, 0x98158a55, 0x980ed193},
	{54, 0x98402ec8, 0x984c068a, 0x98158a55, 0x980ed1a3},
	{56, 0x98402ec8, 0x984c068e, 0x98158a55, 0x980ed18b},
	{60, 0x98402ec8, 0x984c0692, 0x98158a55, 0x980ed183},

	{62, 0x98402ec8, 0x984c0692, 0x98158a55, 0x980ed193},
	{64, 0x98402ec8, 0x984c0692, 0x98158a55, 0x980ed1a3},

	/* 802.11 HyperLan 2 */
	{100, 0x98402ec8, 0x984c06b2, 0x98178a55, 0x980ed783},

	{102, 0x98402ec8, 0x985c06b2, 0x98578a55, 0x980ed793},
	{104, 0x98402ec8, 0x985c06b2, 0x98578a55, 0x980ed1a3},
	{108, 0x98402ecc, 0x985c0a32, 0x98578a55, 0x980ed193},

	{110, 0x98402ecc, 0x984c0a36, 0x98178a55, 0x980ed183},
	{112, 0x98402ecc, 0x984c0a36, 0x98178a55, 0x980ed19b},
	{116, 0x98402ecc, 0x984c0a3a, 0x98178a55, 0x980ed1a3},
	{118, 0x98402ecc, 0x984c0a3e, 0x98178a55, 0x980ed193},
	{120, 0x98402ec4, 0x984c0382, 0x98178a55, 0x980ed183},
	{124, 0x98402ec4, 0x984c0382, 0x98178a55, 0x980ed193},
	{126, 0x98402ec4, 0x984c0382, 0x98178a55, 0x980ed15b},
	{128, 0x98402ec4, 0x984c0382, 0x98178a55, 0x980ed1a3},
	{132, 0x98402ec4, 0x984c0386, 0x98178a55, 0x980ed18b},
	{134, 0x98402ec4, 0x984c0386, 0x98178a55, 0x980ed193},
	{136, 0x98402ec4, 0x984c0386, 0x98178a55, 0x980ed19b},
	{140, 0x98402ec4, 0x984c038a, 0x98178a55, 0x980ed183},

	/* 802.11 UNII */
	{149, 0x98402ec4, 0x984c038a, 0x98178a55, 0x980ed1a7},
	{151, 0x98402ec4, 0x984c038e, 0x98178a55, 0x980ed187},
	{153, 0x98402ec4, 0x984c038e, 0x98178a55, 0x980ed18f},
	{157, 0x98402ec4, 0x984c038e, 0x98178a55, 0x980ed19f},
	{159, 0x98402ec4, 0x984c038e, 0x98178a55, 0x980ed1a7},
	{161, 0x98402ec4, 0x984c0392, 0x98178a55, 0x980ed187},
	{165, 0x98402ec4, 0x984c0392, 0x98178a55, 0x980ed197},
	{167, 0x98402ec4, 0x984c03d2, 0x98179855, 0x9815531f},
	{169, 0x98402ec4, 0x984c03d2, 0x98179855, 0x98155327},
	{171, 0x98402ec4, 0x984c03d6, 0x98179855, 0x98155307},
	{173, 0x98402ec4, 0x984c03d6, 0x98179855, 0x9815530f},

	/* Japan */
	{184, 0x95002ccc, 0x9500491e, 0x9509be55, 0x950c0a0b},
	{188, 0x95002ccc, 0x95004922, 0x9509be55, 0x950c0a13},
	{192, 0x95002ccc, 0x95004926, 0x9509be55, 0x950c0a1b},
	{196, 0x95002ccc, 0x9500492a, 0x9509be55, 0x950c0a23},
	{208, 0x95002ccc, 0x9500493a, 0x9509be55, 0x950c0a13},
	{212, 0x95002ccc, 0x9500493e, 0x9509be55, 0x950c0a1b},
	{216, 0x95002ccc, 0x95004982, 0x9509be55, 0x950c0a23},
#else
	RT2860_RF2850
#endif
};

static const struct {
	uint8_t	n, r, k;
} rt2870_rf3020_freqs[] = {
	RT3070_RF3020
};

static const struct {
	uint8_t	reg;
	uint8_t	val;
} rt3070_def_rf[] = {
	RT3070_DEF_RF
};


/*
 * private functions
 */
static uint16_t rt2870_eeprom_read_2(struct uwgem_dev *dp, uint16_t addr);
static uint16_t rt2870_efuse_read_2(struct uwgem_dev *dp, uint16_t addr);
static int rt2870_rt3070_rf_read(struct uwgem_dev *, uint8_t, uint8_t *);
static int rt2870_rt3070_rf_write(struct uwgem_dev *, uint8_t, uint8_t);

/* newstate */

/* nic operations */
static int rt2870_reset_chip(struct uwgem_dev *, uint_t);
static int rt2870_init_chip(struct uwgem_dev *);
static int rt2870_start_chip(struct uwgem_dev *);
static int rt2870_stop_chip(struct uwgem_dev *);
static int rt2870_set_rx_filter(struct uwgem_dev *);
static int rt2870_get_stats(struct uwgem_dev *);

/* packet operations */
static mblk_t *rt2870_tx_make_packet(struct uwgem_dev *, mblk_t *,
    uint_t, struct ieee80211_node *ni);
static mblk_t *rt2870_rx_make_packet(struct uwgem_dev *, mblk_t *);

/* private ieee80211 functions */
extern int ieee80211_fix_rate(ieee80211_node_t *, struct ieee80211_rateset *, int);
extern void ieee80211_setbasicrates(struct ieee80211_rateset *,
    enum ieee80211_phymode);

/* =============================================================== */
/*
 * I/O functions
 */
/* =============================================================== */
#define	RT2870_WRITE2(dp, p, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	RT2870_WRITE_2,	\
	/* wValue */	(v),	\
	/* wIndex */	(p),	\
	/* wLength */	0,	\
	/* bufp */	NULL,	\
	/* size */	0)) != USB_SUCCESS) goto label

#define	RT2870_WRITE(dp, p, v, errp, label)	\
	if ((*(errp) = rt2870_write(dp, p, v)) != USB_SUCCESS) goto label

#define	RT2870_READ_REGION(dp, reg, v, len, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	RT2870_READ_REGION_1,	\
	/* wValue */	0,	\
	/* wIndex */	(reg),	\
	/* wLength */	(len),	\
	/* bufp */	(v),	\
	/* size */	(len))) != USB_SUCCESS) goto label

#define	RT2870_READ(dp, reg, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ RT2870_READ_REGION_1,	\
	/* wValue */	0,	\
	/* wIndex */	(reg),	\
	/* wLength */	4,	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

#define	RT2870_EEPROM_READ2(dp, addr, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ RT2870_EEPROM_READ,	\
	/* wValue */	0,	\
	/* wIndex */	(addr)*2,	\
	/* wLength */	2,	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

#define	RT2870_RST(dp, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	RT2870_RESET,	\
	/* wValue */	(v),	\
	/* wIndex */	0,	\
	/* wLength */	0,	\
	/* bufp */	NULL,	\
	/* size */	0)) != USB_SUCCESS) goto label

/* =============================================================== */
/*
 * Debugging support
 */
/* =============================================================== */

#ifdef DEBUG
static void
rt2870_debug_printf(uint32_t dbg_flags, const int8_t *fmt, ...)
{
	va_list args;

	if (dbg_flags & rt2870_dbg_flags) {
		va_start(args, fmt);
		vcmn_err(CE_CONT, fmt, args);
		va_end(args);
	}
}
#endif /* DEBUG */

/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
rt2870_write(struct uwgem_dev *dp, uint16_t reg, uint32_t val)
{
	int	err;

	RT2870_WRITE2(dp, reg, (uint16_t)val, &err, fail);
	RT2870_WRITE2(dp, reg + 2, (uint16_t)(val >> 16), &err, fail);
fail:
	return (err);
}

static int
rt2870_write_region(struct uwgem_dev *dp,
    uint16_t reg, const uint8_t *buf, int len)
{
	int err;
	int off;
	/*
	 * NB: the WRITE_REGION_1 command is not stable on RT2860.
	 * We thus issue multiple WRITE_2 commands instead.
	 */
	ASSERT((len & 1) == 0);
	for (off = 0; off < len; off += 2) {
		RT2870_WRITE2(dp, reg + off, buf[off + 1] << 8 | buf[off],
		    &err, usberr);
	}

usberr:
	if (err != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: failed, reg:0x%x len:%x",
		    dp->name, __func__, reg, len);
	}
	return (err);
}

static int
rt2870_set_region_4(struct uwgem_dev *dp,
    uint32_t addr, uint32_t data, int size)
{
	int err;

	for (; size > 0; size--, addr += 4) {
		RT2870_WRITE(dp, addr, data, &err, usberr);
	}
usberr:
	if (err != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: failed, addr:0x%x data:%x",
		    dp->name,  __func__, addr, data);
	}
	return (err);
}

/*
 * Send a command to the 8051 microcontroller unit.
 */
static int
rt2870_mcu_cmd(struct uwgem_dev *dp, uint8_t cmd, uint8_t token, uint16_t arg)
{
	struct rt2870_softc *sc = dp->private;
	int	ntries;
	uint32_t tmp;
	int err;

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_H2M_MAILBOX, &tmp, &err, usberr);
		if ((tmp & RT2860_H2M_BUSY) == 0) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: cmd:%x arg:%x, mcu is not ready",
			    dp->name, __func__, cmd, arg);
			return (USB_FAILURE);
		}
		drv_usecwait(2);
	}

	RT2870_WRITE(dp, RT2860_H2M_MAILBOX,
	    RT2860_H2M_BUSY | token << 16 | arg, &err, usberr);
	RT2870_WRITE(dp, RT2860_HOST_CMD, cmd, &err, usberr);
#ifdef WAIT_WRITE
	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_H2M_MAILBOX, &tmp, &err, usberr);
		if (!(tmp & RT2860_H2M_BUSY)) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: cmd:%x arg:%x: write timeout",
			    dp->name, __func__, cmd, arg);
			return (USB_FAILURE);
		}
	}
#endif
	return (USB_SUCCESS);

usberr:
	cmn_err(CE_CONT, "!%s: %s: cmd:%x arg:%x, usb i/o error",
	    dp->name, __func__, cmd, arg);
	return (err);
}

static int
rt2870_load_microcode(struct uwgem_dev *dp, uint32_t mac_rev)
{
	struct rt2870_softc *sc = dp->private;
	int		ntries;
	size_t		size;
	uint8_t		*ucode, *fptr;
	uint32_t	off, i;
	int		err;
	uint32_t	tmp;

	/*
	 * choose microcode
	 * Note. sc->mac_rev isn't initialized yet
	 */
	if (
#if 1
	    (mac_rev >> 16) == 0x2860 ||
	    (mac_rev >> 16) == 0x2872 ||
	    (mac_rev >> 16) == 0x3070
#else
	    (mac_rev >> 16) != 0x3071
#endif
	    ) {
		ucode = (void *)rt2870_new2;
		size = 4096;
	} else {
		/* RT3071 or later */
		ucode = (void *)&rt2870_new2[4096];
		size = 4096;
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %s(): The size of ucode is: %d bytes, mac_rev is: 0x%x",
	    dp->name, __func__, size, mac_rev));

	RT2870_READ(dp, RT2860_ASIC_VER_ID, &tmp, &err, usberr);

	/* write microcode image */
	DPRINTF(0, (CE_CONT, "!%s: %s: start loading ucode",
	    dp->name, __func__));
	err = rt2870_write_region(dp, RT2870_FW_BASE, ucode, size);
	DPRINTF(0, (CE_CONT, "!%s: %s: end loading ucode",
	    dp->name, __func__));
	if (err != USB_SUCCESS) {
		goto usberr;
	}

	DPRINTF(0, (CE_CONT, "!%s: %s: writing MAILBOX_CID",
	    dp->name, __func__));
	RT2870_WRITE(dp, RT2860_H2M_MAILBOX_CID, ~0U, &err, usberr);

	DPRINTF(0, (CE_CONT, "!%s: %s: writing MAILBOX_STATUS",
	    dp->name, __func__));
	RT2870_WRITE(dp, RT2860_H2M_MAILBOX_STATUS, ~0U, &err, usberr);

	/* kick microcontroller unit */
	DPRINTF(0, (CE_CONT, "!%s: %s: start microcontroller",
	    dp->name, __func__));
	RT2870_RST(dp, 8, &err, usberr);

	delay(drv_usectohz(10*1000));
	DPRINTF(0, (CE_CONT, "!%s: %s: writing MAILBOX",
	    dp->name, __func__));
	RT2870_WRITE(dp, RT2860_H2M_MAILBOX, 0, &err, usberr);

	DPRINTF(0, (CE_CONT, "!%s: %s: writing CMD_BOOT",
	    dp->name, __func__));
	/* XXX - the code from ralink don't use RT2860_TOKEN_NO_INTR */
	if ((err = rt2870_mcu_cmd(dp, RT2860_MCU_CMD_BOOT,
	    /* RT2860_TOKEN_NO_INTR */ 0, 0)) != USB_SUCCESS) {
		goto usberr;
	}
#ifdef NEVER
	/* XXX - the code from ralink don't include the code below */
	if (CHIP_RT3070(mac_rev) ||
	    CHIP_RT3071(mac_rev) ||
	    CHIP_RT3572(mac_rev)) {
		drv_usecwait(200);
		rt2870_mcu_cmd(dp, 0x36, RT2860_TOKEN_NO_INTR, 0);
		drv_usecwait(10);
	}
#endif

	/* wait until microcontroller is ready */
	for (ntries = 0; ; ntries++) {
		drv_usecwait(1000);
		RT2870_READ(dp, RT2860_SYS_CTRL, &tmp, &err, usberr);
		if (tmp & RT2860_MCU_READY) {
			break;
		}
		if (ntries > 1000) {
			RUN_DEBUG(RT2870_DBG_FW, "!%s: %s: "
			    "timeout waiting for MCU to initialize",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	DPRINTF(0, (CE_CONT, "!%s: %s: ucode loaded successfully",
	    dp->name, __func__));
usberr:
	if (err != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: failed to load ucode",
		    dp->name, __func__);
	}
	return (err);
}

static int
rt2870_set_macaddr(struct uwgem_dev *dp, const uint8_t *addr)
{
	struct rt2870_softc *sc = dp->private;
	int	err;

	RT2870_WRITE(dp, RT2860_MAC_ADDR_DW0,
	    addr[0] | addr[1] << 8 | addr[2] << 16 | addr[3] << 24,
	    &err, usberr);
	RT2870_WRITE(dp, RT2860_MAC_ADDR_DW1,
	    addr[4] | addr[5] << 8 | 0xff << 16,
	    &err, usberr);
usberr:
#ifdef DEBUG
	if (err != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: failed", __func__);
	}
#endif
	return (err);
}

/*
 * Reading and writing from/to the BBP is different from RT2560 and RT2661.
 * We access the BBP through the 8051 microcontroller unit which means that
 * the microcode must be loaded first.
 */
static int
rt2870_bbp_read(struct uwgem_dev *dp, uint8_t reg, uint8_t *val)
{
#if 0
	uint32_t val;
	int ntries;
	int err = USB_SUCCESS;

	for (ntries = 0; ntries < 10; ntries++) {
		RT2870_READ(dp,
		    RT2860_H2M_BBPAGENT, &val, &err, usberr);
		if (!(val & RT2860_BBP_CSR_KICK)) {
			goto ready;
		}
	}
	RUN_DEBUG(RT2870_DBG_FW,
	    "!%s: %s: could not read from BBP through MCU",
	    dp->name, __func__);
	return (0);

ready:
	struct rt2870_softc *sc = dp->private;

	RT2870_WRITE(dp, RT2860_H2M_BBPAGENT, RT2860_BBP_RW_PARALLEL |
	    RT2860_BBP_CSR_KICK | RT2860_BBP_CSR_READ | (reg << 8),
	    &err, usberr);

	(void) rt2870_mcu_cmd(dp, RT2860_MCU_CMD_BBP, RT2860_TOKEN_NO_INTR, 0);
	DELAY(1000);

	for (ntries = 0; ntries < 100; ntries++) {
		RT2870_READ(dp, RT2860_H2M_BBPAGENT, &val, &err, usberr);
		if (!(val & RT2860_BBP_CSR_KICK)) {
			return (val & 0xff);
		}
		DELAY(1);
	}
	RUN_DEBUG(RT2870_DBG_FW,
	    "!%s: %s: could not read from BBP through MCU",
	    dp->name, __func__);
	return (USB_FAILURE);
usberr:
	*val =  (tmp & 0xff);
	return (err);
#else
	uint32_t tmp;
	int ntries, err;

	for (ntries = 0; ntries < 10; ntries++) {
		RT2870_READ(dp, RT2860_BBP_CSR_CFG, &tmp, &err, usberr);
		if (!(tmp & RT2860_BBP_CSR_KICK))
			goto ready;
	}
	RUN_DEBUG(RT2870_DBG_MSG,
	    "!%s: %s: could not read from BBP", dp->name, __func__);
	return (0);

ready:
	tmp = RT2860_BBP_CSR_READ | RT2860_BBP_CSR_KICK | reg << 8;
	RT2870_WRITE(dp, RT2860_BBP_CSR_CFG, tmp, &err, usberr);

	for (ntries = 0; ntries < 10; ntries++) {
		RT2870_READ(dp, RT2860_BBP_CSR_CFG, &tmp, &err, usberr);
		if (!(tmp & RT2860_BBP_CSR_KICK))
			goto done;
	}

usberr:
	cmn_err(CE_WARN, "!%s: %s: could not read from BBP",
	    dp->name,  __func__);
	return (USB_FAILURE);

done:
	*val =  (tmp & 0xff);
	return (USB_SUCCESS);
#endif
}

static int
rt2870_bbp_write(struct uwgem_dev *dp, uint8_t reg, uint8_t val) /* ok */
{
#if 0
	int ntries;
	uint32_t tmp;
	int err = USB_SUCCESS;

	for (ntries = 0; ntries < 10; ntries++) {
		RT2870_READ(dp, RT2860_H2M_BBPAGENT, &tmp, &err, usberr);
		if (!(tmp & RT2860_BBP_CSR_KICK))
			goto ready;
		DELAY(1);
	}
	RUN_DEBUG(RT2870_DBG_FW,
	    "!%s: %s: RT2860_BBP_CSR_KICK is set", dp->name, __func__);
	return (USB_FAILURE);

ready:
	RT2870_WRITE(dp, RT2860_H2M_BBPAGENT, RT2860_BBP_RW_PARALLEL |
	    RT2860_BBP_CSR_KICK | (reg << 8) | val, &err, usberr);

	(void) rt2870_mcu_cmd(dp, RT2860_MCU_CMD_BBP, RT2860_TOKEN_NO_INTR, 0);
	DELAY(1000);
usberr:
	return (err);
#else
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int ntries, err;

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_BBP_CSR_CFG, &tmp, &err, usberr);
		if ((tmp & RT2860_BBP_CSR_KICK) == 0) {
			break;
		}
		if (ntries > 10) {
			cmn_err(CE_WARN,
			    "!%s: timeout: RT2860_BBP_CSR_CFG is not ready");
			return (USB_FAILURE);
		}
	}

	tmp = RT2860_BBP_CSR_KICK | reg << 8 | val;
	RT2870_WRITE(dp, RT2860_BBP_CSR_CFG, tmp, &err, usberr);

#ifdef WAIT_WRITE
	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_BBP_CSR_CFG, &tmp, &err, usberr);
		if ((tmp & RT2860_BBP_CSR_KICK) == 0) {
			break;
		}
		if (ntries > 10) {
			cmn_err(CE_WARN,
			    "!%s: timeout: RT2860_BBP_CSR_CFG is busy");
			return (USB_FAILURE);
		}
	}
#endif
	return (USB_SUCCESS);
usberr:
	cmn_err(CE_WARN, "!%s: %s: usb i/o error", dp->name, __func__);
	return (err);

#endif
}

static int
rt2870_bbp_init(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	int i, ntries;
	uint8_t bbp0;
	uint8_t tmp8;

	/* wait for BBP to wake up */
	for (ntries = 0; ; ntries++) {
		if (rt2870_bbp_read(dp, 0, &bbp0) != USB_SUCCESS) {
			cmn_err(CE_WARN, "!%s: %s: usb i/o error",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
		if (bbp0 != 0 && bbp0 != 0xff) {
			break;
		}
		if (ntries > 20) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout waiting for BBP to wake up",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	/* initialize BBP registers to default values */
	for (i = 0; i < N(rt2870_def_bbp); i++) {
		rt2870_bbp_write(dp, rt2870_def_bbp[i].reg,
		    rt2870_def_bbp[i].val);
	}

	if (CHIP_RT30XX(sc->mac_rev) ||
	    CHIP_RT3572(sc->mac_rev) ||
	    CHIP_RT3390(sc->mac_rev)) {
		rt2870_bbp_write(dp, 79, 0x13);
		rt2870_bbp_write(dp, 80, 0x05);
		rt2870_bbp_write(dp, 81, 0x33);
	} else if ((sc->mac_rev & 0xffff) != 0x0101) {
		/* fix BBP84 for RT2860E or later */
		rt2870_bbp_write(dp, 84, 0x19);
	}

	if (CHIP_RT3090(sc->mac_rev) || CHIP_RT3390(sc->mac_rev)) {
		/* enable DC filter */
		if ((sc->mac_rev & 0xffff) >= 0x0211) {
			rt2870_bbp_write(dp, 103, 0xc0);
		}
		/* improve power consumption */
		rt2870_bbp_read(dp, 138, &tmp8);
		if (sc->ntxchains == 1) {
			/* turn off tx DAC_1 */
			tmp8 |= 0x20;
		}

		if (sc->nrxchains == 1) {
			/* turn off tx ADC_1 */
			tmp8 &= ~0x02;
		}
		rt2870_bbp_write(dp, 138, tmp8);

		/* improve power consumption in RT3071 Ver.E */
		if ((sc->mac_rev & 0xffff) >= 0x0211) {
			rt2870_bbp_read(dp, 31, &tmp8);
			tmp8 &= ~0x03;
			rt2870_bbp_write(dp, 31, tmp8);
		}
	} else if (CHIP_RT3070(sc->mac_rev)) {
		if ((sc->mac_rev & 0xffff) >= 0x0201) {
			/* enable DC filter */
			rt2870_bbp_write(dp, 103, 0xc0);

			/* improve power consumption in RT3070 Ver.F */
			rt2870_bbp_read(dp, 31, &tmp8);
			tmp8 &= ~0x3;
			rt2870_bbp_write(dp, 31, tmp8);
		}
		/* TX_LO1_en, RF R17 register Bit 3 to 0 */
		rt2870_rt3070_rf_read(dp, 17, &tmp8);
		tmp8 &= ~0x08;
		/* to fix rx long range issue */
		if (sc->ext_2ghz_lna == 0) {
			tmp8 |= 0x20;
		}
		/* set RF_R17_bit[2:0] equal to EEPROM setting at 0x48h */
		if ((sc->rssi_2ghz[2] & 0x07) >= 1) {
			tmp8 &= ~0x07;	/* clean bit [2:0] */
			tmp8 |= sc->rssi_2ghz[2] & 0x07;
		}
		rt2870_rt3070_rf_write(dp, 17, tmp8);
	}

	if (sc->mac_rev == 0x28600100) {	/* R */
		rt2870_bbp_write(dp, 69, 0x16);
		rt2870_bbp_write(dp, 73, 0x12);
	}

	return (USB_SUCCESS);
}

/*
 * RF routines
 */
static int
rt2870_rt2870_rf_write(struct uwgem_dev *dp, uint8_t reg, uint32_t val) /* ok */
{
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int ntries;
	int err;

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_RF_CSR_CFG0, &tmp, &err, usberr);
		if ((tmp & RT2860_RF_REG_CTRL) == 0) {
			break;
		}
		if (ntries > 10) {
			RUN_DEBUG(RT2870_DBG_FW,
			    "!%s: %s: could not write to RF",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	/* RF registers are 24-bit on the RT2860 */
	tmp = RT2860_RF_REG_CTRL | 24 << RT2860_RF_REG_WIDTH_SHIFT |
	    (val & 0x3fffff) << 2 | (reg & 3);
	RT2870_WRITE(dp, RT2860_RF_CSR_CFG0, tmp, &err, usberr);
#ifdef WAIT_WRITE
	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT2860_RF_CSR_CFG0, &tmp, &err, usberr);
		if ((tmp & RT2860_RF_REG_CTRL) == 0) {
			break;
		}
		if (ntries > 10) {
			cmn_err(CE_WARN,
			    "!%s: %s: write operation was unfinished ",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}
#endif
	return (err);

usberr:
	cmn_err(CE_WARN, "!%s: %s: usb i/o error", dp->name, __func__);
	return (err);
}

static void
rt2870_select_chan_group(struct uwgem_dev *dp, int group,
    enum ieee80211_state state, uint_t htmode)	/* ok */
{
	struct rt2870_softc	*sc = dp->private;
	uint32_t	tmp;
	uint8_t	tmp8;
	int	err;
	struct ieee80211com	*ic = &dp->ic;

	rt2870_bbp_write(dp, 62, 0x37 - sc->lna[group]);
	rt2870_bbp_write(dp, 63, 0x37 - sc->lna[group]);
	rt2870_bbp_write(dp, 64, 0x37 - sc->lna[group]);
	rt2870_bbp_write(dp, 86, 0x00);

	if (group == 0) {
		if (sc->ext_2ghz_lna) {
			rt2870_bbp_write(dp, 82, 0x62);
			rt2870_bbp_write(dp, 75, 0x46);
		} else {
			rt2870_bbp_write(dp, 82, 0x84);
			rt2870_bbp_write(dp, 75, 0x50);
		}
	} else {
		if (sc->ext_5ghz_lna) {
			rt2870_bbp_write(dp, 82, 0xf2);
			rt2870_bbp_write(dp, 75, 0x46);
		} else {
			rt2870_bbp_write(dp, 82, 0xf2);
			rt2870_bbp_write(dp, 75, 0x50);
		}
	}

	RT2870_READ(dp, RT2860_TX_BAND_CFG, &tmp, &err, usberr);
	tmp &= ~(RT2860_5G_BAND_SEL_N | RT2860_5G_BAND_SEL_P |
	    RT2860_TX_BAND_SEL);
	tmp |= (group == 0) ? RT2860_5G_BAND_SEL_N : RT2860_5G_BAND_SEL_P;

	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		tmp |= RT2860_TX_BAND_SEL;
	}

	RT2870_WRITE(dp, RT2860_TX_BAND_CFG, tmp, &err, usberr);
	DPRINTF(1, (CE_CONT, "!%s: %s tx_band_cfg:%x",
	    dp->name, __func__, tmp));

	/* enable appropriate Power Amplifiers and Low Noise Amplifiers */
#if 0
	tmp = RT2860_RFTR_EN | RT2860_TRSW_EN;
	if (group == 0) {	/* 2GHz */
		tmp |= RT2860_PA_PE_G0_EN | RT2860_LNA_PE_G0_EN;
		if (sc->ntxchains > 1) {
			tmp |= RT2860_PA_PE_G1_EN;
		}
		if (sc->nrxchains > 1) {
			tmp |= RT2860_LNA_PE_G1_EN;
		}
	} else {		/* 5GHz */
		tmp |= RT2860_PA_PE_A0_EN | RT2860_LNA_PE_A0_EN;
		if (sc->ntxchains > 1) {
			tmp |= RT2860_PA_PE_A1_EN;
		}
		if (sc->nrxchains > 1) {
			tmp |= RT2860_LNA_PE_A1_EN;
		}
	}
#else
	if (group == 0) {	/* 2GHz */
		tmp = 0x50f0a;
		if (sc->ntxchains == 1) {
			tmp &= ~0xc;
		}
		if (sc->nrxchains == 1) {
			tmp &= ~0xc00;
		}
	} else {
		tmp = 0x50f05;
		if (sc->ntxchains == 1) {
			tmp &= ~0xc;
		}
		if (sc->nrxchains == 1) {
			tmp &= ~0xc00;
		}
	}
#endif
	RT2870_WRITE(dp, RT2860_TX_PIN_CFG, tmp, &err, usberr);
	DPRINTF(1, (CE_CONT, "!%s: %s tx_pin_cfg:%x",
	    dp->name, __func__, tmp));

	rt2870_bbp_read(dp, 4, &tmp8);
	tmp8 &= ~0x18;
	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_ABOVE ||
	    htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		tmp8 |= 0x10;
	}
	rt2870_bbp_write(dp, 4, tmp8);

	rt2870_bbp_read(dp, 3, &tmp8);
	tmp8 &= ~0x20;
	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		tmp8 |= 0x20;
	}
	rt2870_bbp_write(dp, 3, tmp8);

	/* set initial AGC value */
	if (group == 0) {
		/* b/g */
		if (CHIP_RT3070(sc->mac_rev) ||
		    CHIP_RT3090(sc->mac_rev) ||
		    CHIP_RT3572(sc->mac_rev) ||
		    CHIP_RT3390(sc->mac_rev)) {
			tmp8 = 0x1c + 2*sc->lna[0];
		} else {
			tmp8 = 0x2e + sc->lna[0];
		}
	} else {
		/* a */
		/* 11n support */
		if (htmode == IEEE80211_HTINFO_2NDCHAN_ABOVE ||
		    htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
			tmp8 = 0x3a + (sc->lna[group] * 5) / 3;
		} else {
			tmp8 = 0x32 + (sc->lna[group] * 5) / 3;
		}
	}
	rt2870_bbp_write(dp, 66, tmp8);
usberr:
	;
}

static void
rt2870_rt2870_set_chan(struct uwgem_dev *dp, uint_t chan,
    enum ieee80211_state state, uint_t htmode)
{
	struct rt2870_softc	*sc = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	const struct rfprog	*rfprog = rt2860_rf2850;
	uint32_t		r1, r2, r3, r4;
	int8_t			txpow1, txpow2;
	int			i;

	/* find the settings for this channel (we know it exists) */
	i = 0;
	while (rfprog[i].chan != chan) {
		i++;
	}

	r2 = rfprog[i].r2;
#ifdef NEW_RF2850
	r2 >>= 2;
#endif
	if (sc->ntxchains == 1) {
		r2 |= 1 << 12;		/* 1T: disable Tx chain 2 */
	}
	if (sc->nrxchains == 1) {
		r2 |= 1 << 15 | 1 << 4;	/* 1R: disable Rx chains 2 & 3 */
	} else if (sc->nrxchains == 2) {
		r2 |= 1 << 4;		/* 2R: disable Rx chain 3 */
	}

	/* use Tx power values from EEPROM */
	txpow1 = sc->txpow1[i];
	txpow2 = sc->txpow2[i];
#if 0
	if (IEEE80211_IS_CHAN_5GHZ(c)) {
		txpow1 = txpow1 << 1 | 1;
		txpow2 = txpow2 << 1 | 1;
	}
#else
	if (chan > 14) {
		if (txpow1 >= 0) {
			txpow1 = txpow1 << 1;
		} else {
			txpow1 = (7 + txpow1) << 1 | 1;
		}
		if (txpow2 >= 0) {
			txpow2 = txpow2 << 1;
		} else {
			txpow2 = (7 + txpow2) << 1 | 1;
		}
	}
#endif
	r1 = rfprog[i].r1;
	r3 = rfprog[i].r3;
	r4 = rfprog[i].r4;
#ifdef NEW_RF2850
	r1 >>= 2;
	r3 >>= 2;
	r4 >>= 2;
#endif
	r3 = (r3 & ~(0x1f << 7)) | txpow1 << 7;
	r4 = (r4 & (~(0x3f << 13)) & (~(0x1f << 4)))
	    | sc->freq << 13 | txpow2 << 4;

	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_BELOW ||
	    htmode == IEEE80211_HTINFO_2NDCHAN_ABOVE) {
		r4 |= 0x80000;
	}

	rt2870_rt2870_rf_write(dp, RT2860_RF1, r1);
	rt2870_rt2870_rf_write(dp, RT2860_RF2, r2);
	rt2870_rt2870_rf_write(dp, RT2860_RF3, r3 & ~1);
	rt2870_rt2870_rf_write(dp, RT2860_RF4, r4);

	DELAY(200);

	rt2870_rt2870_rf_write(dp, RT2860_RF1, r1);
	rt2870_rt2870_rf_write(dp, RT2860_RF2, r2);
	rt2870_rt2870_rf_write(dp, RT2860_RF3, r3 | 1);
	rt2870_rt2870_rf_write(dp, RT2860_RF4, r4);

	DELAY(200);

	rt2870_rt2870_rf_write(dp, RT2860_RF1, r1);
	rt2870_rt2870_rf_write(dp, RT2860_RF2, r2);
	rt2870_rt2870_rf_write(dp, RT2860_RF3, r3 & ~1);
	rt2870_rt2870_rf_write(dp, RT2860_RF4, r4);
}

static int
rt2870_rt3070_rf_read(struct uwgem_dev *dp, uint8_t reg, uint8_t *val)
{
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int err, ntries;

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT3070_RF_CSR_CFG, &tmp, &err, usberr);
		if ((tmp & RT3070_RF_KICK) == 0) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: could not read RT3070_RF_CSR_CFG",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	tmp = RT3070_RF_KICK | reg << 8;
	RT2870_WRITE(dp, RT3070_RF_CSR_CFG, tmp, &err, usberr);

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT3070_RF_CSR_CFG, &tmp, &err, usberr);
		if (!(tmp & RT3070_RF_KICK)) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_WARN,
			    "!%s: %s: could not read RT3070_RF_CSR_CFG",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	*val = tmp & 0xff;
	return (USB_SUCCESS);

usberr:
	RUN_DEBUG(RT2870_DBG_MSG,
	    "!%s: %s: could not read to RF", dp->name, __func__);
	return (err);
}

static int
rt2870_rt3070_rf_write(struct uwgem_dev *dp, uint8_t reg, uint8_t val)
{
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int err, ntries;

	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT3070_RF_CSR_CFG, &tmp, &err, usberr);
		if ((tmp & RT3070_RF_KICK) == 0) {
			break;
		}
		if (ntries > 10) {
			cmn_err(CE_WARN,
			    "!%s: %s: could not read RT3070_RF_CSR_CFG",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	tmp = RT3070_RF_WRITE | RT3070_RF_KICK | reg << 8 | val;
	RT2870_WRITE(dp, RT3070_RF_CSR_CFG, tmp, &err, usberr);
#ifdef WAIT_WRITE
	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT3070_RF_CSR_CFG, &tmp, &err, usberr);
		if ((tmp & RT3070_RF_KICK) == 0) {
			break;
		}
		if (ntries > 10) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout: write unfinished",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}
#endif
	return (USB_SUCCESS);

usberr:
	cmn_err(CE_WARN, "!%s: %s: usb i/o error", dp->name, __func__);
	return (err);
}

static void
rt2870_rt3070_set_chan(struct uwgem_dev *dp, uint_t chan,
    enum ieee80211_state state, uint_t htmode)
{
	struct ieee80211com	*ic = &dp->ic;
	uint8_t	rf;
	int8_t	txpow1, txpow2;
	struct rt2870_softc	*sc = dp->private;

	ASSERT(chan >= 1 && chan <= 14);	/* RT3070 is 2GHz only */

	/* use Tx power values from EEPROM */
	txpow1 = sc->txpow1[chan - 1];
	txpow2 = sc->txpow2[chan - 1];

	/* setup channel parameters */
	rt2870_rt3070_rf_write(dp, 2, rt2870_rf3020_freqs[chan - 1].n);
	rt2870_rt3070_rf_write(dp, 3, rt2870_rf3020_freqs[chan - 1].k);
	rt2870_rt3070_rf_read(dp, 6, &rf);
	rf = (rf & ~0x03) | rt2870_rf3020_freqs[chan - 1].r;
	rt2870_rt3070_rf_write(dp, 6, rf);
#if 0
{
	uint8_t n, r, k;
	rt2870_rt3070_rf_read(dp, 2, &n);
	rt2870_rt3070_rf_read(dp, 3, &k);
	rt2870_rt3070_rf_read(dp, 6, &r);
	cmn_err(CE_CONT, "!%s: %s: chan:%d, %d %d %d",
	    dp->name, __func__, chan, n, r, k);
}
#endif
	/* set Tx0 power */
	rt2870_rt3070_rf_read(dp, 12, &rf);
	rf = (rf & ~0x1f) | txpow1;
	rt2870_rt3070_rf_write(dp, 12, rf);

	/* set Tx1 power */
	rt2870_rt3070_rf_read(dp, 13, &rf);
	rf = (rf & ~0x1f) | txpow2;
	rt2870_rt3070_rf_write(dp, 13, rf);

	/* stream tx/rx setting */
	rt2870_rt3070_rf_read(dp, 1, &rf);	/* R */
	rf &= ~0xfc;
	if (sc->ntxchains == 1) {
		rf |= 1 << 7 | 1 << 5;	/* 1T: disable Tx chains 2 & 3 */
	} else if (sc->ntxchains == 2) {
		rf |= 1 << 7;		/* 2T: disable Tx chain 3 */
	}
	if (sc->nrxchains == 1) {
		rf |= 1 << 6 | 1 << 4;	/* 1R: disable Rx chains 2 & 3 */
	} else if (sc->nrxchains == 2) {
		rf |= 1 << 6;		/* 2R: disable Rx chain 3 */
	}
	rt2870_rt3070_rf_write(dp, 1, rf);	/* R */

	/* set RF offset */
	rt2870_rt3070_rf_read(dp, 23, &rf);	/* R */
	rf = (rf & ~0x7f) | sc->freq;
	rt2870_rt3070_rf_write(dp, 23, rf);	/* R */

	/* program RF filter */
	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_ABOVE ||
	    htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		rf = sc->rf24_40mhz;
	} else {
		rf = sc->rf24_20mhz;
	}
	rt2870_rt3070_rf_write(dp, 24, rf);
	rt2870_rt3070_rf_write(dp, 31, rf);

	/* enable RF tuning */
	rt2870_rt3070_rf_read(dp, 7, &rf);	/* R */
	rt2870_rt3070_rf_write(dp, 7, rf | 0x01);	/* R */
}

static int
rt2870_set_rx_antenna(struct uwgem_dev *dp, int aux)	/* ok */
{
	struct rt2870_softc	*sc = dp->private;
	uint32_t	tmp;
	int	err;

	err = rt2870_mcu_cmd(dp,
	    0x73, RT2860_TOKEN_NO_INTR, aux ? 0x0000 : 0x0100);
	if (err != USB_SUCCESS) {
		goto usberr;
	}

	RT2870_READ(dp, RT2860_GPIO_CTRL, &tmp, &err, usberr);
	tmp &= ~0x0808;
	RT2870_WRITE(dp,
	    RT2860_GPIO_CTRL, aux ? (tmp | 0x08) : tmp, &err, usberr);
usberr:
	return (err);
}

static int
rt2870_set_chan(struct uwgem_dev *dp, struct ieee80211_channel *c,
    enum ieee80211_state state, uint_t htmode)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	uint_t chan, group;
	int	err = USB_FAILURE;

	chan = ieee80211_chan2ieee(ic, c);
	if (chan == 0 || chan == IEEE80211_CHAN_ANY) {
		cmn_err(CE_WARN, "!%s: %s(): chan (%d) is invalid",
		    dp->name, __func__, chan);
		return (err);
	}

	/* 11n support */
	if (htmode == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		/*
		 * shift down the center of the channel to support 40MHz
		 * band width with the lower 2nd channel.
		 */
		if (chan < 3) {
			cmn_err(CE_NOTE,
			    "!%s: %s: invalid 2nd chan: chan:%d, htmode:%d",
			    dp->name, __func__, chan, htmode);
			chan = 3;
		}
		chan = chan - 2;

	} else if (htmode == IEEE80211_HTINFO_2NDCHAN_ABOVE) {
		/*
		 * shift up the center of the channel to support 40MHz
		 * band width with the upper 2nd channel.
		 */
		chan = chan + 2;
	} else {
		htmode = 0;
	}

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: chan:%d", dp->name, __func__, chan));

	if ((CHIP_RT3070(sc->mac_rev) ||
	    CHIP_RT3090(sc->mac_rev) ||
	    CHIP_RT3390(sc->mac_rev)) &&
	    (sc->rf_rev == RT3070_RF_3020 ||
	    sc->rf_rev == RT3070_RF_2020 ||
	    sc->rf_rev == RT3070_RF_3021 ||
	    sc->rf_rev == RT3070_RF_3022)) {
		rt2870_rt3070_set_chan(dp, chan, state, htmode);
	} else {
		rt2870_rt2870_set_chan(dp, chan, state, htmode);
	}

	/* 802.11a uses a 16 microseconds short interframe space */
	sc->sifs = IEEE80211_IS_CHAN_5GHZ(c) ? 16 : 10;

	/* determine channel group */
	if (chan <= 14) {
		group = 0;
	} else if (chan <= 64) {
		group = 1;
	} else if (chan <= 128) {
		group = 2;
	} else {
		group = 3;
	}
	/* XXX - necessary only when group has changed! */
	rt2870_select_chan_group(dp, group, state, htmode);

	DELAY(1000);

	return (USB_SUCCESS);
}

static int
rt2870_rt3070_filter_calib(struct uwgem_dev *dp,
    uint8_t init, uint8_t target, uint8_t *val)
{
	struct rt2870_softc *sc = dp->private;
	uint8_t	rf22, rf24;
	uint8_t	bbp55_pb, bbp55_sb, delta;
	int	ntries;
	boolean_t	need_backtrack;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* program filter */
	rf24 = init;	/* initial filter value */
	rt2870_rt3070_rf_write(dp, 24, rf24);

	/* enable baseband loopback mode */
	rt2870_rt3070_rf_read(dp, 22, &rf22);	/* R */
	rt2870_rt3070_rf_write(dp, 22, rf22 | 0x01);	/* R */

	/* set power and frequency of passband test tone */
	rt2870_bbp_write(dp, 24, 0x00);	/* R */
	for (ntries = 0; ntries < 100; ntries++) {
		/* transmit test tone */
		rt2870_bbp_write(dp, 25, 0x90);
		DELAY(1000);

		/* read received power */
		if (rt2870_bbp_read(dp, 55, &bbp55_pb) != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: bbp_read pb: failed", dp->name, __func__);
			return (USB_FAILURE);
		}

		if (bbp55_pb != 0) {
			cmn_err(CE_CONT,
			    "!%s: %s: passband test tone: bbp55:0x%02x",
			    dp->name, __func__, bbp55_pb);
			break;
		}
	}

	/* set power and frequency of stopband test tone */
	need_backtrack = B_FALSE;
	rt2870_bbp_write(dp, 24, 0x06);
	for (ntries = 0; ntries < 100; ntries++) {

		/* transmit test tone */
		rt2870_bbp_write(dp, 25, 0x90);
		DELAY(1000);

		/* read received power */
		if (rt2870_bbp_read(dp, 55, &bbp55_sb) != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: bbp_read sb failed", dp->name, __func__);
			return (USB_FAILURE);
		}

		delta = bbp55_pb - bbp55_sb;
		if (delta > target) {
			if (need_backtrack) {
				rf24--;	/* backtrack */
				rt2870_rt3070_rf_write(dp, 24, rf24);
			}
			*val = rf24;
			cmn_err(CE_CONT,
			    "!%s: %s: stopband test tone:"
			    " bbp55:0x%02x, rf24:%02x->%02x",
			    dp->name, __func__, bbp55_sb, init, rf24);
			goto done;
		}
		if (delta == target) {
			need_backtrack = B_TRUE;
		}

		/* reprogram filter */
		rf24++;
		rt2870_rt3070_rf_write(dp, 24, rf24);
	}
	cmn_err(CE_WARN,
	    "!%s: %s: failed: stopband test tone",
	    dp->name, __func__);

done:
	/* restore initial state */
	rt2870_bbp_write(dp, 24, 0x00);	/* R */

	/* disable baseband loopback mode */
	rt2870_rt3070_rf_read(dp, 22, &rf22);	/* R */
	rt2870_rt3070_rf_write(dp, 22, rf22 & ~0x01);	/* R */

	return (USB_SUCCESS);
}

static int
rt2870_rt3070_rf_init(struct uwgem_dev *dp)
{
	struct rt2870_softc	*sc = dp->private;
	uint32_t	tmp;
	uint8_t	rf, bbp4;
	uint8_t	target;
	int	i;
	int	err;

	rt2870_rt3070_rf_read(dp, 30, &rf);
	/* toggle RF R30 bit 7 */
	rt2870_rt3070_rf_write(dp, 30, rf | 0x80);
	DELAY(1000);
	rt2870_rt3070_rf_write(dp, 30, rf & ~0x80);

	/* load default value into rf registers */
	for (i = 0; i < N(rt3070_def_rf); i++) {
		rt2870_rt3070_rf_write(dp, rt3070_def_rf[i].reg,
		    rt3070_def_rf[i].val);
	}
	if (CHIP_RT3070(sc->mac_rev)) {	/* R */
		if (sc->mac_rev & 0xffff < 0x201) {
			/*
			 * Workaround for DAC issue.
			 * change voltage from 1.2V to 1.35V for RT3070
			 */
			RT2870_READ(dp, RT3070_LDO_CFG0, &tmp, &err, usberr);
			tmp = (tmp & ~0x0f000000) | 0x0d000000;
			RT2870_WRITE(dp, RT3070_LDO_CFG0, tmp, &err, usberr);
		}
	} else if (CHIP_RT3071(sc->mac_rev)) {	/* R */
		/* set RF6 bit6 first */
		rt2870_rt3070_rf_read(dp, 6, &rf);
		rt2870_rt3070_rf_write(dp, 6, rf | 0x40);

		rt2870_rt3070_rf_write(dp, 31, 0x14);

		RT2870_READ(dp, RT3070_LDO_CFG0, &tmp, &err, usberr);
		tmp &= ~0x1f000000;
		if (sc->dac_test_bit && (sc->mac_rev & 0xffff) < 0x0211) {
			tmp |= 0x0d000000;
		} else {
			tmp |= 0x01000000;
		}
		RT2870_WRITE(dp, RT3070_LDO_CFG0, tmp, &err, usberr);

		/* workaround for LNA_PE_G1 fail issue */
		RT2870_READ(dp, RT3070_GPIO_SWITCH, &tmp, &err, usberr);
		RT2870_WRITE(dp, RT3070_GPIO_SWITCH, tmp & ~0x20, &err, usberr);
	}

	/*
	 * Calibrate RF filter
	 */
	/* select 20MHz bandwidth */
	rt2870_bbp_read(dp, 4, &bbp4);
	rt2870_bbp_write(dp, 4, bbp4 & ~0x18);	/* R */
	rt2870_rt3070_rf_read(dp, 31, &rf);
	rt2870_rt3070_rf_write(dp, 31, rf & ~0x20);	/* R */

	/* calibrate filter for 20MHz bandwidth */
	sc->rf24_20mhz = 0x1f;	/* default value */

	if (CHIP_RT3090(sc->mac_rev) ||
	    CHIP_RT3572(sc->mac_rev) ||
	    CHIP_RT3390(sc->mac_rev)) {
		target = 0x13;
	} else {
		target = 0x16;
	}
	rt2870_rt3070_filter_calib(dp, 0x07, target, &sc->rf24_20mhz);

	/* select 40MHz bandwidth */
	rt2870_bbp_read(dp, 4, &bbp4);
	rt2870_bbp_write(dp, 4, (bbp4 & ~0x18) | 0x10); /* R */
	rt2870_rt3070_rf_read(dp, 31, &rf);
	rt2870_rt3070_rf_write(dp, 31, rf | 0x20);	/* R */

	/* calibrate filter for 40MHz bandwidth */
	sc->rf24_40mhz = 0x2f;	/* default value */
	if (CHIP_RT3090(sc->mac_rev) ||
	    CHIP_RT3572(sc->mac_rev) ||
	    CHIP_RT3390(sc->mac_rev)) {
		target = 0x15;
	} else {
		target = 0x19;
	}
	rt2870_rt3070_filter_calib(dp, 0x27, target, &sc->rf24_40mhz);

	/* go back to 20MHz bandwidth */
	rt2870_bbp_read(dp, 4, &bbp4);	/* R */
	rt2870_bbp_write(dp, 4, bbp4 & ~0x18);	/* R */
	rt2870_rt3070_rf_read(dp, 31, &rf);
	rt2870_rt3070_rf_write(dp, 31, rf & ~0x20);	/* R */

	/* Workaround for Tx to Rx IQ glitch issue */
	if ((CHIP_RT3070(sc->mac_rev) && (sc->mac_rev & 0xffff) < 0x0201) ||
	    (CHIP_RT3071(sc->mac_rev) && (sc->mac_rev & 0xffff) < 0x0211)) {
		rt2870_rt3070_rf_write(dp, 27, 0x03);	/* R */
	}

	/* setup led for open drain */
	RT2870_READ(dp, RT3070_OPT_14, &tmp, &err, usberr);	/* R */
	RT2870_WRITE(dp, RT3070_OPT_14, tmp | 1, &err, usberr);	/* R */

	rt2870_rt3070_rf_read(dp, 17, &rf);	/* R */
	rf &= ~RT3070_TX_LO1;
	if (sc->ext_2ghz_lna == 0 &&
	    ((CHIP_RT3071(sc->mac_rev) && (sc->mac_rev & 0xffff) >= 0x0211) ||
	    CHIP_RT3070(sc->mac_rev))) {
		/* fix for long range Rx issue */
		rf |= 0x20;
	}
	if ((sc->rssi_2ghz[2] & 0x07) >= 1) {
		/* set RF R17 bit 2:0 equal to EEPROM setting at 0x48h */
		rf = (rf & ~0x07) | (sc->rssi_2ghz[2] & 0x07);
	}
	rt2870_rt3070_rf_write(dp, 17, rf);	/* R */

	if (CHIP_RT3071(sc->mac_rev)) {
		/* normal RF power up sequence */
		rt2870_rt3070_rf_read(dp, 1, &rf);	/* R */
		rf &= ~(RT3070_RX0_PD | RT3070_TX0_PD);
		rf |= RT3070_RF_BLOCK | RT3070_RX1_PD | RT3070_TX1_PD;
		rt2870_rt3070_rf_write(dp, 1, rf);

		rt2870_rt3070_rf_read(dp, 15, &rf);	/* R */
		rt2870_rt3070_rf_write(dp, 15, rf & ~RT3070_TX_LO2);

		rt2870_rt3070_rf_read(dp, 20, &rf);	/* R */
		rt2870_rt3070_rf_write(dp, 20, rf & ~RT3070_RX_LO1);

		rt2870_rt3070_rf_read(dp, 21, &rf);	/* R */
		rt2870_rt3070_rf_write(dp, 21, rf & ~RT3070_RX_LO2);

		rt2870_rt3070_rf_read(dp, 27, &rf);	/* R */
		rf &= ~0x77;
		if ((sc->mac_rev & 0xffff) < 0x0211) {
			rf |= 0x03;
		}
		rt2870_rt3070_rf_write(dp, 27, rf);
	} else if (CHIP_RT3070(sc->mac_rev)) {
		/* workaround for throughput issue */
		rt2870_rt3070_rf_read(dp, 27, &rf);	/* R */
		rf &= ~0x77;
		if ((sc->mac_rev & 0xffff) < 0x0201) {
			rf |= 0x03;
		}
		rt2870_rt3070_rf_write(dp, 27, rf);	/* R */
	}
	return (USB_SUCCESS);
usberr:
	return (USB_FAILURE);
}

static void
rt2870_updateprot(struct uwgem_dev *dp)
{
	struct ieee80211com *ic = &dp->ic;
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int err;

	RT2870_READ(dp, RT2860_TX_RTS_CFG, &tmp, &err, usberr);	/* R */
	tmp = (tmp & ~(0xffff << 8)) | (0x1000 << 8);
	RT2870_WRITE(dp, RT2860_TX_RTS_CFG, tmp, &err, usberr);

	tmp = RT2860_RTSTH_EN | RT2860_PROT_NAV_SHORT | RT2860_TXOP_ALLOW_ALL;
	/* setup protection frame rate (MCS code) */
	tmp |= (ic->ic_curmode == IEEE80211_MODE_11A) ? 0 : 3;

	/* CCK frames don't require protection */
	RT2870_WRITE(dp, RT2860_CCK_PROT_CFG, tmp, &err, usberr);

	if (ic->ic_flags & IEEE80211_F_USEPROT) {
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS) {
			tmp |= RT2860_PROT_CTRL_RTS_CTS;
		} else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY) {
			tmp |= RT2860_PROT_CTRL_CTS;
		}
	}
	RT2870_WRITE(dp, RT2860_OFDM_PROT_CFG, tmp, &err, usberr);
usberr:
	;
}

static void
rt2870_updateprot_ht(struct uwgem_dev *dp)
{
	struct ieee80211com *ic = &dp->ic;
	struct ieee80211_node *in = ic->ic_bss;
	struct rt2870_softc *sc = dp->private;
	uint32_t	tmp20, tmp40;
	uint32_t	prot_mm20, prot_mm40;
	uint32_t	prot_gf20, prot_gf40;
	int		err;

	sc->cur_htcap = in->in_htcap;
	sc->cur_htopmode = in->in_htopmode;

	switch(in->in_htopmode & IEEE80211_HTINFO_OPMODE) {
	case IEEE80211_HTINFO_OPMODE_PURE: /* 0 */
		/*
		 * NO PROTECT
		 * 1.All STAs in the BSS are 20/40 MHz HT
		 * 2. in ai 20/40MHz BSS
		 * 3. all STAs are 20MHz in a 20MHz BSS
		 * Pure HT. no protection.
		 */

		/*
		 * MM20_PROT_CFG
		 *	Reserved (31:27)
		 * 	PROT_TXOP(25:20) -- 010111
		 *	PROT_NAV(19:18)  -- 01 (Short NAV protection)
		 *  PROT_CTRL(17:16) -- 00 (None)
		 * 	PROT_RATE(15:0)  -- 0x4004 (OFDM 24M)
		 */
		prot_mm20 = 0x01744004;	

		/*
		 * MM40_PROT_CFG
		 *	Reserved (31:27)
		 * 	PROT_TXOP(25:20) -- 111111
		 *	PROT_NAV(19:18)  -- 01 (Short NAV protection)
		 *  PROT_CTRL(17:16) -- 00 (None)
		 * 	PROT_RATE(15:0)  -- 0x4084 (duplicate OFDM 24M)
		 */
		prot_mm40 = 0x03f44084;

		/*
		 * GF20_PROT_CFG
		 *	Reserved (31:27)
		 * 	PROT_TXOP(25:20) -- 010111
		 *	PROT_NAV(19:18)  -- 01 (Short NAV protection)
		 *  PROT_CTRL(17:16) -- 00 (None)
		 * 	PROT_RATE(15:0)  -- 0x4004 (OFDM 24M)
		 */
		prot_gf20 = 0x01744004;

		/*
		 * GF40_PROT_CFG
		 *	Reserved (31:27)
		 * 	PROT_TXOP(25:20) -- 111111
		 *	PROT_NAV(19:18)  -- 01 (Short NAV protection)
		 *  PROT_CTRL(17:16) -- 00 (None)
		 * 	PROT_RATE(15:0)  -- 0x4084 (duplicate OFDM 24M)
		 */
		prot_gf40 = 0x03f44084;

		if(in->in_htopmode & IEEE80211_HTINFO_NONGF_PRESENT) {
			/* PROT_NAV(19:18)  -- 01 (Short NAV protectiion)*/
			/* PROT_CTRL(17:16) -- 01 (RTS/CTS)*/
			prot_gf20 = 0x01754004;
			prot_gf40 = 0x03f54084;
		}
		break;
			
	case IEEE80211_HTINFO_OPMODE_PROTOPT: /* 1 */
		/*
		 * This is "HT non-member protection mode."
		 */
		/* If there may be non-HT STAs my BSS*/
		tmp20 = 0x01744004;	/* PROT_CTRL(17:16) : 0 (None) */
		tmp40 = 0x03f44084;	/* duplicaet legacy 24M. BW set 1. */
		if (in->in_erp & IEEE80211_ERP_USE_PROTECTION) {
			/*
			 * ERP use Protection bit is set,
			 * use protection rate at Clause 18.
			 */
			tmp20 = 0x01740003;

			/* Don't duplicate RTS/CTS in CCK mode. 0x03f40083; */
			tmp40 = 0x03f40003;
		}

		/* Assign Protection method for 20/40 MHz packets */
		tmp20 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;
		tmp40 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;

		prot_mm20 = tmp20;
		prot_mm40 = tmp40;
		prot_gf20 = tmp20;
		prot_gf40 = tmp40;
		break;
			
	case IEEE80211_HTINFO_OPMODE_HT20PR: /* 2 */
		/*
		 * If only HT STAs are in BSS. at least one is 20MHz.
		 * Only protect 40MHz packets
		 */
		tmp20 = 0x01744004;	/* PROT_CTRL(17:16) : 0 (None)*/
		tmp40 = 0x03f44084;	/* duplicaet legacy 24M. BW set 1.*/

		if (in->in_erp & IEEE80211_ERP_USE_PROTECTION) {
			/*
			 * ERP use Protection bit is set,
			 * use protection rate at Clause 18.
			 */
			tmp20 = 0x01740003;

			/* Don't duplicate RTS/CTS in CCK mode. 0x03f40083; */
			tmp40 = 0x03f40003;
		} 

		/* Assign Protection method for 40MHz packets */
		tmp40 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;
		prot_mm20 = tmp20;
		prot_mm40 = tmp40;

		if(in->in_htopmode & IEEE80211_HTINFO_NONGF_PRESENT) {
			tmp20 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;
		}

		prot_gf20 = tmp20;
		prot_gf40 = tmp40;
		break;
			
	case IEEE80211_HTINFO_OPMODE_MIXED: /* 3 */
		/*
		 * HT mixed mode. PROTECT ALL!
		 * Assign Rate
		 */
		tmp20 = 0x01744004;	/* duplicaet legacy 24M. BW set 1.*/
		tmp40 = 0x03f44084;

		/*
		 * both 20MHz and 40MHz are protected.
		 * Whether use RTS or CTS-to-self depends on the
		 */
		if (in->in_erp & IEEE80211_ERP_USE_PROTECTION) {
			/*
			 * ERP use Protection bit is set,
			 * use protection rate at Clause 18.
			 */
			tmp20 = 0x01740003;

			/* Don't duplicate RTS/CTS in CCK mode. 0x03f40083*/
			tmp40 = 0x03f40003;
		}

		/*Assign Protection method for 20&40 MHz packets*/

		tmp20 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;
		tmp40 |= RT2860_PROT_CTRL_RTS_CTS | RT2860_PROT_NAV_SHORT;

		prot_mm20 = tmp20;
		prot_mm40 = tmp40;
		prot_gf20 = tmp20;
		prot_gf40 = tmp40;
		break;	
#if 0			
	case 8:
		/* Special on for Atheros problem n chip.*/
		tmp20 = 0x01754004;	/*duplicaet legacy 24M. BW set 1.*/
		tmp40 = 0x03f54084;

		if (in->in_erp & IEEE80211_ERP_USE_PROTECTION) {
			/*
			 * ERP use Protection bit is set,
			 * use protection rate at Clause 18.
			 */
			tmp20 = 0x01750003;

			/* Don't duplicate RTS/CTS in CCK mode. 0x03f40083*/
			tmp40 = 0x03f50003;
		}
			
		prot_mm20 = tmp20; 	/* 0x01754004; */
		prot_mm40 = tmp40;	/* 0x03f54084; */
		prot_gf20 = tmp20; 	/* 0x01754004; */
		prot_gf40 = tmp40;	/* 0x03f54084; */
		break;		
#endif
	}

	RT2870_WRITE(dp, RT2860_MM20_PROT_CFG, prot_mm20, &err, usberr);
	RT2870_WRITE(dp, RT2860_MM40_PROT_CFG, prot_mm40, &err, usberr);
	RT2870_WRITE(dp, RT2860_GF20_PROT_CFG, prot_gf20, &err, usberr);
	RT2870_WRITE(dp, RT2860_GF40_PROT_CFG, prot_gf40, &err, usberr);

usberr:
	;
}

static int
rt2870_set_leds(struct uwgem_dev *dp, uint16_t which)	/* ok */
{
	struct rt2870_softc *sc = dp->private;

	return (rt2870_mcu_cmd(dp,
	    RT2860_MCU_CMD_LEDS, RT2860_TOKEN_NO_INTR,
	    which | (sc->leds & 0x7f)));
}

static int	/* o */
rt2870_txrx_enable(struct uwgem_dev *dp)
{
	int		err = USB_SUCCESS;
	int		ntries;
	uint32_t	tmp;
	struct rt2870_softc	*sc = dp->private;

	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL, RT2860_MAC_TX_EN, &err, usberr);
	for (ntries = 0; ntries < 200; ntries++) {
		RT2870_READ(dp, RT2860_WPDMA_GLO_CFG, &tmp, &err, usberr);
		if ((tmp & (RT2860_TX_DMA_BUSY | RT2860_RX_DMA_BUSY)) == 0) {
			break;
		}
		if (ntries >= 200) {
			cmn_err(CE_WARN, "!%s: hw not ready", __func__);
			return (USB_FAILURE);
		}
		DELAY(1000);
	}
	DELAY(50);

	tmp |= RT2860_RX_DMA_EN | RT2860_TX_DMA_EN | RT2860_TX_WB_DDONE;
	RT2870_WRITE(dp, RT2860_WPDMA_GLO_CFG, tmp, &err, usberr);

	/* enable Rx bulk aggregation (set timeout and limit) */
	tmp = RT2860_USB_TX_EN | RT2860_USB_RX_EN | RT2860_USB_RX_AGG_EN |
	    RT2860_USB_RX_AGG_TO(128) | RT2860_USB_RX_AGG_LMT(21); /* LMT(2) */
	RT2870_WRITE(dp, RT2860_USB_DMA_CFG, tmp, &err, usberr);

	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL,
	    RT2860_MAC_RX_EN | RT2860_MAC_TX_EN, &err, usberr);

	return (USB_SUCCESS);
usberr:
	cmn_err(CE_WARN, "!%s: %s: usb i/o error", dp->name, __func__);
	return (err);
}

static int
rt2870_reset_chip(struct uwgem_dev *dp, uint_t reset_type)
{
	int	i;
	int	err = USB_SUCCESS;
	uint32_t	mac_rev;
	struct rt2870_softc *sc = dp->private;

	for (i = 0; i < 5; i++) {
		RT2870_READ(dp, RT2860_ASIC_VER_ID, &mac_rev, &err, usberr);
		if (mac_rev != 0 && mac_rev != 0xffffffffU) {
			goto mac_ready;
		}
usberr:
		DELAY(1000);
	}
	return (USB_FAILURE);

mac_ready:
#ifdef DEBUG_RT30XX
	mac_rev = DEBUG_RT30XX;
#endif
	if (reset_type == 1) {
		/* cold start */
		sc->initialized = B_FALSE;
		err = rt2870_load_microcode(dp, mac_rev);
		if (err != USB_SUCCESS) {
			RUN_DEBUG(RT2870_DBG_MSG,
			    "!%s: %s: could not load 8051 microcode",
			    dp->name, __func__);
#ifdef notdef
			/* this caused panic */
			rt2870_stop_chip(dp);
#endif
			return (USB_FAILURE);
		}
		(void) rt2870_set_leds(dp, 0);
	}
	return (err);
}

static int
rt2870_init_chip(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com	*ic;
	int			i, err, qid, ridx, ntries;
	uint8_t			bbp1, bbp3;
	uint32_t		tmp;

	DPRINTF(1, (CE_CONT, "!%s: %s: called ", dp->name, __func__));

	ic = &dp->ic;

	for (ntries = 0; ; ntries++) {
		DELAY(1000);
		RT2870_READ(dp, RT2860_WPDMA_GLO_CFG, &tmp, &err, fail);
		if ((tmp & (RT2860_TX_DMA_BUSY | RT2860_RX_DMA_BUSY)) == 0) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_NOTE,
			    "!%s: %s:timeout waiting for DMA engine",
			    dp->name, __func__);
			break;
		}
	}
	tmp &= RT2860_HDR_SEG_LEN;	/* was 0xff0 */
	tmp |= RT2860_TX_WB_DDONE;
	RT2870_WRITE(dp, RT2860_WPDMA_GLO_CFG, tmp, &err, fail);

	/* workaround to avoid hang-on issue */
	for (ntries = 0; ; ntries++) {
		DELAY(10);
		RT2870_READ(dp, RT2860_ASIC_VER_ID, &tmp, &err, fail);
		if (tmp != 0 && tmp != 0xffffffffU) {
			break;
		}
		if (ntries > 100) {
			cmn_err(CE_NOTE,
			    "!%s: %s:timeout waiting for mac idle",
			    dp->name, __func__);
			break;
		}
	}

	/* turn off PME_OEN to solve high-current issue */
	RT2870_READ(dp, RT2860_SYS_CTRL, &tmp, &err, fail);	/* R */
	RT2870_WRITE(dp, RT2860_SYS_CTRL, tmp & ~RT2860_PME_OEN, &err, fail);

	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL,
	    RT2860_BBP_HRST | RT2860_MAC_SRST, &err, fail);	/* R */
	RT2870_WRITE(dp, RT2860_USB_DMA_CFG, 0, &err, fail);	/* R */

	RT2870_RST(dp, 1, &err, reset_err);			/* R */

	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL, 0, &err, fail);	/* R */

	for (i = 0; i < N(rt2870_def_mac); i++) {
		uint32_t	reg;
		uint32_t	val;

		reg = rt2870_def_mac[i].reg;
		val = rt2870_def_mac[i].val;

		if (reg == RT2860_TX_SW_CFG0 &&
		    (CHIP_RT3070(sc->mac_rev) ||
		    CHIP_RT3071(sc->mac_rev) ||
		    CHIP_RT3572(sc->mac_rev) ||
		    CHIP_RT3090(sc->mac_rev) ||
		    CHIP_RT3390(sc->mac_rev))) {
			val = 0x00000400;
		}
		RT2870_WRITE(dp, reg, val, &err, fail);
	}

	/* STA MAC registers */
	RT2870_WRITE(dp, RT2860_WMM_AIFSN_CFG, 0x00002273, &err, fail);
	RT2870_WRITE(dp, RT2860_WMM_CWMIN_CFG, 0x00002344, &err, fail);
	RT2870_WRITE(dp, RT2860_WMM_CWMAX_CFG, 0x000034aa, &err, fail);

	if (CHIP_RT3090(sc->mac_rev) ||
	    CHIP_RT3572(sc->mac_rev) ||
	    CHIP_RT3390(sc->mac_rev)) {
		/* initialize serial mac registers */
		RT2870_WRITE(dp, RT2860_TX_SW_CFG1, 0, &err, fail);
		if ((sc->mac_rev & 0xffff) < 0x0211) {
			if (sc->dac_test_bit) {
				RT2870_WRITE(dp, RT2860_TX_SW_CFG2, 0x2c,
				    &err, fail);
			} else {
				RT2870_WRITE(dp, RT2860_TX_SW_CFG2, 0x0f,
				    &err, fail);
			}
		} else {
			RT2870_WRITE(dp, RT2860_TX_SW_CFG2, 0, &err, fail);
		}
	} else if (CHIP_RT3070(sc->mac_rev)) {
		if ((sc->mac_rev & 0xffff) < 0x0201) {
			RT2870_WRITE(dp, RT2860_TX_SW_CFG1, 0, &err, fail);
			RT2870_WRITE(dp, RT2860_TX_SW_CFG2, 0x2c, &err, fail);
		} else {
			RT2870_WRITE(dp, RT2860_TX_SW_CFG2, 0, &err, fail);
		}
	}

	/* wait while MAC is busy */
	for (ntries = 0; ; ntries++) {
		DELAY(1000);
		RT2870_READ(dp, RT2860_MAC_STATUS_REG, &tmp, &err, fail);
		if ((tmp &
		    (RT2860_RX_STATUS_BUSY | RT2860_TX_STATUS_BUSY)) == 0) {
			break;
		}
		if (ntries > 100) {
			RUN_DEBUG(RT2870_DBG_FW, "!%s: %s:"
			    "timeout waiting for MAC", dp->name, __func__);
			rt2870_stop_chip(dp);
			goto fail;
		}
	}

	/* clear Host to MCU mailbox */
	RT2870_WRITE(dp, RT2860_H2M_BBPAGENT, 0, &err, fail);	/* R */
	RT2870_WRITE(dp, RT2860_H2M_MAILBOX, 0, &err, fail);	/* R */
	if (CHIP_RT3090(sc->mac_rev)) {	/* R */
		(void) rt2870_mcu_cmd(dp, RT2860_MCU_CMD_BOOT, 0x00, 0x0000);
	}
	DELAY(1000);	/* R */

	if ((err = rt2870_bbp_init(dp)) != USB_SUCCESS) {
		rt2870_stop_chip(dp);
		goto fail;
	}

	if (sc->mac_rev >= 0x28720200 && sc->mac_rev < 0x30700200) { /* R */
		/* enlarge MAX_LEN_CFG */
		RT2870_READ(dp, RT2860_MAX_LEN_CFG, &tmp, &err, fail);
		tmp &= 0xfff;
		tmp |= RT2860_MAX_PSDU_LEN32K << RT2860_MAX_PSDU_LEN_SHIFT;
		RT2870_WRITE(dp, RT2860_MAX_LEN_CFG, tmp, &err, fail);
	}

	/* clear WCID table */
	for (i = 0; i < RT2870_WCID_MAX; i++) {	/* R */
		static const uint8_t invalid_mac[]
		    = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, };

		err = rt2870_write_region(dp,
		    RT2860_WCID_ENTRY(i), invalid_mac, 8);
		if (err != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: failed to clear WCID table",
			    dp->name, __func__);
			goto fail;
		}
	}
#if 0
	/* clear statistics counters */
	RT2870_READ(dp, RT2860_RX_STA_CNT0, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_RX_STA_CNT1, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_RX_STA_CNT2, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT0, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT1, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT2, &tmp, &err, fail);	/* R */
#endif
	if (!sc->initialized) {
		/* clear shared key mode */
		rt2870_set_region_4(dp, RT2860_SKEY_MODE_0_7, 0, 4);

		/* reset WCID attribute table */
		rt2870_set_region_4(dp,
		    RT2860_WCID_ATTR(0), RT2860_RX_PKEY_EN, 256);
	}

	/* disable TSF sync */
	RT2870_READ(dp, RT2860_BCN_TIME_CFG, &tmp, &err, fail); /* R */
	tmp &= ~(RT2860_BCN_TX_EN
	    | RT2860_TBTT_TIMER_EN
	    | (3 << RT2860_TSF_SYNC_MODE_SHIFT)
	    | RT2860_TSF_TIMER_EN);
	RT2870_WRITE(dp, RT2860_BCN_TIME_CFG, tmp, &err, fail); /* R */

	/* XXX - Why do we need to clear statistics counters again ? */
	RT2870_READ(dp, RT2860_RX_STA_CNT0, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_RX_STA_CNT1, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_RX_STA_CNT2, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT0, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT1, &tmp, &err, fail);	/* R */
	RT2870_READ(dp, RT2860_TX_STA_CNT2, &tmp, &err, fail);	/* R */

	RT2870_READ(dp, RT2860_US_CYC_CNT, &tmp, &err, fail);	/* R */
	tmp = (tmp & ~0xff) | 0x1e;
	RT2870_WRITE(dp, RT2860_US_CYC_CNT, tmp, &err, fail);	/* R */

	if ((sc->mac_rev & 0xffff) != 0x0101) {
		RT2870_WRITE(dp, RT2860_TXOP_CTRL_CFG, 0x0000583f,
		    &err, fail);	/* R */
	}

	/* --- end of basic initialization --- */

	/* fix WMM parameter */
	RT2870_WRITE(dp, RT2860_WMM_TXOP0_CFG, 0, &err, fail);	/* R */

	if (ic->ic_phytype == IEEE80211_T_DS) {
		/* 11b */
		RT2870_WRITE(dp, RT2860_WMM_TXOP1_CFG,
		    96 << 16 | 192, &err, fail);	/* R */
	} else {
		RT2870_WRITE(dp, RT2860_WMM_TXOP1_CFG,
		    48 << 16 | 96, &err, fail);		/* R */
	}

	/*
	 * Apply the product configuration in EEPROM.
	 */

	/* init Tx power for all Tx rates (from EEPROM) */
	for (ridx = 0; ridx < 5; ridx++) {
		if (sc->txpow20mhz[ridx] == 0xffffffffU) {
			continue;
		}
		RT2870_WRITE(dp, RT2860_TX_PWR_CFG(ridx),
		    sc->txpow20mhz[ridx], &err, fail);
	}

	/* write vendor-specific BBP values (from EEPROM) */
	for (i = 0; i < 8; i++) {
		if (sc->bbp[i].reg == 0 || sc->bbp[i].reg == 0xff) {
			continue;
		}
		rt2870_bbp_write(dp, sc->bbp[i].reg, sc->bbp[i].val);
	}

	/* select Main antenna for 1T1R devices */
	if (sc->rf_rev == RT3070_RF_3020) {
		rt2870_set_rx_antenna(dp, 0);	/* R */
	}

	/* send LEDs operating mode to microcontroller */
	(void) rt2870_mcu_cmd(dp,
	    RT2860_MCU_CMD_LED1, RT2860_TOKEN_NO_INTR, sc->led[0]);
	(void) rt2870_mcu_cmd(dp,
	    RT2860_MCU_CMD_LED2, RT2860_TOKEN_NO_INTR, sc->led[1]);
	(void) rt2870_mcu_cmd(dp,
	    RT2860_MCU_CMD_LED3, RT2860_TOKEN_NO_INTR, sc->led[2]);
	(void) rt2870_mcu_cmd(dp,
	    RT2860_MCU_CMD_LED_RSSI, RT2860_TOKEN_NO_INTR,
	    (sc->leds >> 7) & 1);

	/* disable non-existing Rx chains */
	rt2870_bbp_read(dp, 3, &bbp3);
	bbp3 &= ~(1 << 3 | 1 << 4);
	if (sc->nrxchains == 2) {
		bbp3 |= 1 << 3;
	} else if (sc->nrxchains == 3) {
		bbp3 |= 1 << 4;
	}
	rt2870_bbp_write(dp, 3, bbp3);

	/* disable non-existing Tx chains */
	rt2870_bbp_read(dp, 1, &bbp1);
	if (sc->ntxchains == 1) {
		bbp1 &= ~(1 << 3 | 1 << 4);
	}
	rt2870_bbp_write(dp, 1, bbp1);

	if ((sc->mac_rev >> 16) >= 0x3070) {
		rt2870_rt3070_rf_init(dp);
	}
#ifdef NEVER	/* moved into start_chip */
	/* select default channel */
	rt2870_set_chan(dp, ic->ic_curchan, ic->ic_state, 0);

	/* turn radio LED on */
	(void) rt2870_set_leds(dp, RT2860_LED_RADIO);

	if ((err = rt2870_txrx_enable(dp) != USB_SUCCESS)) {
		goto fail;
	}
#endif
	sc->last_updatestats = ddi_get_lbolt();
	sc->initialized = B_TRUE;

	RUN_DEBUG(RT2870_DBG_MSG,
	    "!%s: %s: successfully finished ", dp->name, __func__);
	return (USB_SUCCESS);

reset_err:
	cmn_err(CE_WARN, "!%s: %s: could not reset the chipset",
	    dp->name,  __func__);
fail:
	return (USB_FAILURE);
}

/*
 * Setup rt2870
 */
static int
rt2870_start_chip(struct uwgem_dev *dp)
{
	int	err = USB_SUCCESS;
	struct ieee80211com	*ic = &dp->ic;

	/* select default channel */
	rt2870_set_chan(dp, ic->ic_curchan, ic->ic_state, 0);

	/* turn radio LED on */
	(void) rt2870_set_leds(dp, RT2860_LED_RADIO);

	if ((err = rt2870_txrx_enable(dp) != USB_SUCCESS)) {
		goto usberr;
	}

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));

	return (err);
}

static int
rt2870_stop_chip(struct uwgem_dev *dp)
{
	int		qid;
	uint32_t	tmp;
	int		ntries;
	int		err = USB_SUCCESS;
	struct rt2870_softc	*sc = dp->private;

	(void) rt2870_set_leds(dp, 0);	/* turn all LEDs off */

	/* disable Tx/Rx */
	RT2870_READ(dp, RT2860_MAC_SYS_CTRL, &tmp, &err, usberr);
	tmp &= ~(RT2860_MAC_RX_EN | RT2860_MAC_TX_EN);
	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL, tmp, &err, usberr);

	/* wait for pending Tx to complete */
	for (ntries = 0; ntries < 100; ntries++) {
		RT2870_READ(dp, RT2860_TXRXQ_PCNT, &tmp, &err, usberr);
		if ((tmp & RT2860_TX2Q_PCNT_MASK) == 0) {
			break;
		}
	}
	DELAY(1000);
	RT2870_WRITE(dp, RT2860_USB_DMA_CFG, 0, &err, usberr);

	/* reset adapter */
	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL,
	    RT2860_BBP_HRST | RT2860_MAC_SRST, &err, usberr);
	RT2870_WRITE(dp, RT2860_MAC_SYS_CTRL, 0, &err, usberr);
	DELAY(1000);

usberr:

	return (err);

}

static void rt2870_iter_func(void *arg, struct ieee80211_node *ni);
static void rt2870_updatestats(struct uwgem_dev *dp);

static void
rt2860_drain_stats_fifo(struct uwgem_dev *dp)
{
	struct rt2870_softc	*sc = dp->private;
	struct rt2870_amrr_node	*amn;
	uint32_t	stat;
	uint8_t	wcid, mcs, pid;
	int	err;

	/* drain Tx status FIFO (maxsize = 16) */
	for (; ; ) {
		RT2870_READ(dp, RT2860_TX_STAT_FIFO, &stat, &err, usberr);
		if ((stat & RT2860_TXQ_VLD) == 0) {
			break;
		}
		DPRINTF(10, (CE_CONT, "!%s: %s: tx stat 0x%08x",
		    dp->name, __func__, stat));

		wcid = (stat >> 8) & 0xff;

		/* if no ACK was requested, no feedback is available */
		if ((stat & RT2860_TXQ_ACKREQ) == 0 || wcid == 0xff) {
			continue;
		}
		/* update per-STA AMRR stats */
		amn = &sc->amn[wcid];
		amn->amn_txcnt++;
		if (stat & RT2860_TXQ_OK) {
			/*
			 * Check if there were retries, ie if the Tx success
			 * rate is different from the requested rate.  Note
			 * that it works only because we do not allow rate
			 * fallback from OFDM to CCK.
			 */
			mcs = (stat >> RT2860_TXQ_MCS_SHIFT) & 0x7f;
			pid = (stat >> RT2860_TXQ_PID_SHIFT) & 0xf;
			if (mcs != pid - 1) {
				amn->amn_retrycnt++;
			}
		} else {
			amn->amn_retrycnt++;
		}
	}
usberr:
	;
}

static int
rt2870_get_stats(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	uint32_t	val;
	int		err;

	RT2870_READ(dp, RT2860_TX_STA_CNT0, &val, &err, usberr);
	dp->stats.tx_err = val & 0xffff;

	RT2870_READ(dp, RT2860_TX_STA_CNT1, &val, &err, usberr);
	dp->stats.tx_ok += val & 0xffffU;
	dp->stats.tx_retry += (val >> 16) & 0xffffU;

#if 0
	RUN_DEBUG(RT2870_DBG_MSG, "!%s: %s: "
	    "tx: err %d, ok:%d, retry:%d",
	    dp->name, __func__,
	    dp->stats.tx_err, dp->stats.tx_ok, dp->stats.tx_retry);
#endif

	/* get statistics for each wifi-client */
	rt2860_drain_stats_fifo(dp);
	rt2870_updatestats(dp);

	if (ic->ic_state == IEEE80211_S_RUN &&
	    (ic->ic_bss->in_flags & IEEE80211_NODE_HT) &&
	    !(sc->cur_htcap == ic->ic_bss->in_htcap &&
	    sc->cur_htopmode == ic->ic_bss->in_htopmode)) {
		rt2870_updateprot_ht(dp);
	}
usberr:
	return (err);
}

static int
rt2870_set_rx_filter(struct uwgem_dev *dp)
{
	int	err;
	uint32_t	tmp = 0;

	err = rt2870_set_macaddr(dp, dp->ic.ic_macaddr);
	if (err != USB_SUCCESS) {
		goto usberr;
	}

	/* set Rx filter */
	tmp = RT2860_DROP_CRC_ERR | RT2860_DROP_PHY_ERR;
	if (dp->ic.ic_opmode != IEEE80211_M_MONITOR) {
		tmp |= RT2860_DROP_UC_NOME | RT2860_DROP_DUPL |
		    RT2860_DROP_CTS | RT2860_DROP_BA | RT2860_DROP_ACK |
		    RT2860_DROP_VER_ERR | RT2860_DROP_CTRL_RSV |
		    RT2860_DROP_CFACK | RT2860_DROP_CFEND;
		if (dp->ic.ic_opmode == IEEE80211_M_STA) {
			tmp |= RT2860_DROP_RTS | RT2860_DROP_PSPOLL;
		}
	}

	if (dp->rxmode & RXMODE_PROMISC) {
		tmp &= ~RT2860_DROP_UC_NOME;
	}

	RT2870_WRITE(dp, RT2860_RX_FILTR_CFG, tmp, &err, usberr);

usberr:
	return (err);
}

/*
 * send/receive packet check
 */

/* quickly determine if a given rate is CCK or OFDM */
#define	RT2860_RATE_IS_OFDM(rate) ((rate) >= 12 && (rate) != 22)

#define	RT2860_ACK_SIZE		14	/* 10 + 4(FCS) */
#define	RT2860_SIFS_TIME	10

static uint8_t
rt2870_rate2mcs(uint8_t rate)
{
	switch (rate) {
	/* CCK rates */
	case 2:
		return (0);
	case 4:
		return (1);
	case 11:
		return (2);
	case 22:
		return (3);
	/* OFDM rates */
	case 12:
		return (0);
	case 18:
		return (1);
	case 24:
		return (2);
	case 36:
		return (3);
	case 48:
		return (4);
	case 72:
		return (5);
	case 96:
		return (6);
	case 108:
		return (7);
	}

	return (0);	/* shouldn't get there */
}

/*
 * Return the expected ack rate for a frame transmitted at rate `rate'.
 */
static int
rt2870_ack_rate(struct ieee80211com *ic, int rate)
{
	switch (rate) {
	/* CCK rates */
	case 2:
		return (2);
	case 4:
	case 11:
	case 22:
		return ((ic->ic_curmode == IEEE80211_MODE_11B) ? 4 : rate);

	/* OFDM rates */
	case 12:
	case 18:
		return (12);
	case 24:
	case 36:
		return (24);
	case 48:
	case 72:
	case 96:
	case 108:
		return (48);
	}

	/* default to 1Mbps */
	return (2);
}


/*
 * Compute the duration (in us) needed to transmit `len' bytes at rate `rate'.
 * The function automatically determines the operating mode depending on the
 * given rate. `flags' indicates whether short preamble is in use or not.
 */
static uint16_t
rt2870_txtime(int len, int rate, uint32_t flags)
{
	uint16_t	txtime;

	if (RT2860_RATE_IS_OFDM(rate)) {
		/* IEEE Std 802.11g-2003, pp. 44 */
		txtime = (8 + 4 * len + 3 + rate - 1) / rate;
		txtime = 16 + 4 + 4 * txtime + 6;
	} else {
		/* IEEE Std 802.11b-1999, pp. 28 */
		txtime = (16 * len + rate - 1) / rate;
		if (rate != 2 && (flags & IEEE80211_F_SHPREAMBLE)) {
			txtime +=  72 + 24;
		} else {
			txtime += 144 + 48;
		}
	}
	return (txtime);
}

static mblk_t *
rt2870_tx_make_packet(struct uwgem_dev *dp, mblk_t *mp, uint_t type,
    struct ieee80211_node *ni)
{
	struct ieee80211com	*ic = &dp->ic;
	struct rt2870_softc	*sc = dp->private;
	struct rt2870_tx_queue	*qp;
	struct rt2870_txd	*txd;
	struct rt2870_txwi	*txwi;
	struct ieee80211_frame	*wh;
	int			qid, off, rate, err;
	int			tmp, pktlen;
	uint_t			mcs;
	uint_t			pid;
	uint16_t		dur = 0;
	mblk_t			*m = NULL;
	mblk_t			*m0;
	uint8_t			*bp;
	boolean_t		ampdu = B_FALSE;
	extern const int ieee80211_htrates[];
	int			pad = 0;
	int			hasqos;

	pktlen = msgdsize(mp);

	off = sizeof (struct rt2870_txd) + sizeof (*txwi);
#ifdef CONFIG_DATAPAD
	if (IEEE80211_QOS_HAS_SEQ((struct ieee80211_qosframe *)mp->b_rptr)) {
		ASSERT(pktlen >= sizeof (struct ieee80211_qosframe));
#if 0
		RUN_DEBUG(RT2870_DBG_TX,
		    "!%s: %s(): qos_has_seq", dp->name, __func__);
#endif
		pad = 2;
	}
#endif

	/*
	 * 4 byte zeros are required at the tail of the packet.
	 * As encription seems to require some additional space,
	 * we add 64 byte. (actually WPA2 requires 8 byte)
	 */
	m = allocb(off + P2ROUNDUP(pktlen + pad, 4) + 64, BPRI_MED);
	if (m == NULL) {
		RUN_DEBUG(RT2870_DBG_TX,
		    "!%s: %s(): can't alloc mblk", dp->name, __func__);
		dp->stats.noxmtbuf++;
		goto fail;
	}
	ASSERT(m->b_cont == NULL);

	/* clear packet header area and make it single buffer */
	bzero(m->b_rptr, off + pad);

	m->b_rptr += off + pad;
	bp = m->b_rptr;

	/* copy the entire payload */
	for (m0 = mp; m0 != NULL; m0 = m0->b_cont) {
		tmp = MBLKL(m0);
		(void) bcopy(m0->b_rptr, bp, tmp);
		bp += tmp;
	}
	m->b_wptr = bp;

	wh = (struct ieee80211_frame *)m->b_rptr;

	if (type == IEEE80211_FC0_TYPE_DATA) {
		/* prepare 11n/qos and encription */
		(void) ieee80211_encap(ic, m, ni);
#if DEBUG_LEVEL > 10
		if (pktlen != msgdsize(m)) {
			cmn_err(CE_CONT,
			    "!%s: %s: pkt length was changed by ieee80211_encap:"
			    " %d->%d",
			    dp->name, __func__, pktlen, msgdsize(m));
			pktlen = msgdsize(m);
		}
#endif
	} else if (type == IEEE80211_FC0_TYPE_MGT) {
		uint8_t	*cp;

		cp = (uint8_t *)&wh[1];
		if (cp[0] == IEEE80211_ACTION_CAT_BA &&
		    cp[1] == IEEE80211_ACTION_BA_ADDBA_REQUEST) {
			uint_t	t;
			uint16_t	baparamset;
			uint16_t	baseq;

			baparamset = cp[4] << 8 | cp[3];
			baseq = cp[8] << 8 | cp[7];
			DPRINTF(1, (CE_CONT, "!%s: %s: addba req: %04x %04x",
			    dp->name, __func__, baparamset, baseq));

			t = (baparamset & IEEE80211_BAPS_TID)
			    >> IEEE80211_BAPS_TID_S;

			/* fix bufsiz */
			baparamset &= ~IEEE80211_BAPS_BUFSIZ;
			baparamset |= 32U << IEEE80211_BAPS_BUFSIZ_S;
			cp[3] = (uint8_t)baparamset;
			cp[4] = (uint8_t)(baparamset >> 8);

			/* XXX - fix start seq. Is it right to add one? */
			baseq = ((ni->in_txseqs[t] + 1)
			    << IEEE80211_BASEQ_START_S)
			    & IEEE80211_BASEQ_START;
			cp[7] = (uint8_t)baseq;
			cp[8] = (uint8_t)(baseq >> 8);

		}
	}
#ifdef DEBUG_LEVEL
	if (rt2870_debug > 10) {
		cmn_err(CE_CONT, "!%s: %s: FC0:%02x->%02x",
		    dp->name, __func__, mp->b_rptr[0], m->b_rptr[0]);
	}
#endif
	/* encryption */
	if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		/*
		 * XXX - WEP flag also includes WPA/WPA2.
		 */
		struct ieee80211_key *k;

		/* Here, do software encription if required */
		k = ieee80211_crypto_encap(ic, m);
#if DEBUG_LEVEL > 10
		if (pktlen != msgdsize(m)) {
			cmn_err(CE_CONT,
			    "!%s: %s: pkt length was changed by encription:"
			    " %d->%d",
			    dp->name, __func__, pktlen, msgdsize(m));
		}
#endif
		if (k == NULL) {
			RUN_DEBUG(RT2870_DBG_TX,
			    "!%s: %s(): ieee80211_crypto_encap failed",
			    dp->name, __func__);
			goto fail;
		}
		ASSERT(wh == (void *)m->b_rptr);
	}

	/* ensure that tx packet is single fragment again */
	if (m->b_cont) {
		mblk_t	*t;

		cmn_err(CE_CONT, "!%s: b_cont != NULL", __func__);
		pktlen = msgdsize(m);
		m->b_rptr -= off + pad;
		m->b_wptr = m->b_rptr + off + P2ROUNDUP(pktlen + pad, 4) + 4;
		t = msgpullup(m, -1);
		freemsg(m);
		if (t == NULL) {
			goto fail;
		}
		m = t;
		m->b_rptr += off + pad;
		m->b_wptr = m->b_rptr + P2ROUNDUP(pktlen + pad, 4) + 4;
	}

	/*
	 * ensure the actual (i.e w/o tailing padds) packet size
	 * into air after encription
	 */
	pktlen = m->b_wptr - m->b_rptr;

	/*
	 * process tx descriptor and wifi header
	 */
	/* restore head of mblk */
	m->b_rptr -= off + pad;

	/*
	 * determin qid for index of bulkout pipe
	 */
	hasqos = IEEE80211_QOS_HAS_SEQ(wh);
	if (type != IEEE80211_FC0_TYPE_DATA) {
		/* index of bulk-out pipe for mgmt */
		qid = sc->mgtqid;	/* 0 or 5 */

	} else if (hasqos) {
		uint_t	tid;
		struct ieee80211_qosframe *qwh = (void *)wh;

		tid = qwh->i_qos[0] & IEEE80211_QOS_TID;

		/* [0..3] index of bulk-out pipe */
		qid = TID_TO_WME_AC(tid);

		if ((qwh->i_qos[0] & IEEE80211_QOS_ACKPOLICY_BA) ==
		    IEEE80211_QOS_ACKPOLICY_BA) {
			/* enable ampdu for the packet */
			ampdu = B_TRUE;

			/* clear software mark of ampdu */
			qwh->i_qos[0] &= ~IEEE80211_QOS_ACKPOLICY_BA;
		}
	} else {
		/* index of bulk-out pipe for normal priority */
		qid = EDCA_AC_BE;	/* it is 0 */
	}
	ASSERT(qid >= 0 && qid <= 5);

	/* save the qid into mblk header */
	m->b_datap->db_cksum16 = qid;

	/*
	 * setup TX descriptor
	 */
	txd = (void *) m->b_rptr;
	txd->flags = RT2860_TX_QSEL_EDCA;
	tmp = sizeof (*txwi) + P2ROUNDUP(pktlen + pad, 4);
	txd->len = LE_16(tmp);

	/*
	 * setup TX Wireless Information
	 */
	txwi = (void *)(txd + 1);
#if 1
	txwi->wcid = (type == IEEE80211_FC0_TYPE_DATA) ?
	    RT2870_AID2WCID(ni->in_associd) : 0xff;
#else
	txwi->wcid = 0;
#endif
	tmp = pktlen;
	if (hasqos) {
#ifndef CONFIG_DATAPAD
		/*
		 * Don't include 2-byte pads after QOS into packet length,
		 * that is added by IEEE80211_F_DATAPAD in ic_flags.
		 */
		tmp -= 2;
#else
		ASSERT(pad == 2);
		ASSERT(m->b_rptr + off == (void *)(txwi + 1));
		/*
		 * move qos ieee80211 header to correct position
		 */
		memmove((void *)(txwi + 1), wh, sizeof (struct ieee80211_qosframe));
		wh = (void *)(txwi + 1);

		/* clear 2-byte pad */
		((uint8_t *)wh)[sizeof (struct ieee80211_qosframe) + 0] = 0;
		((uint8_t *)wh)[sizeof (struct ieee80211_qosframe) + 1] = 0;
#endif
	}
	txwi->len = LE_16(tmp);
	txwi->xflags = 0;

	/*
	 * pickup a rate
	 */
	DPRINTF(10, (CE_CONT, "!%s: txrate:%d",
	    __func__, dp->txrate));
	if (IEEE80211_IS_MULTICAST(wh->i_addr1) ||
	    IEEE80211_ADDR_EQ(wh->i_addr1, wh->i_addr3) ||
	    type != IEEE80211_FC0_TYPE_DATA) {

		/* get rate for non-data or broadcast packets */
		rate = ni->in_rates.ir_rates[0] & IEEE80211_RATE_VAL;
		ASSERT(rate == 2);

		/* get MCS code from rate */
		mcs = RT2860_PHY_CCK | rt2870_rate2mcs(rate);

		if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
			dur = 0;
		} else {
			dur = rt2870_txtime(RT2860_ACK_SIZE,
			    rt2870_ack_rate(ic, rate), ic->ic_flags) + sc->sifs;
		}

		txwi->txop = RT2860_TX_TXOP_BACKOFF;

	} else if ( ic->ic_curmode == IEEE80211_MODE_11NG) {
		/*
		 * 11n 2.4GHz
		 * get mcs code in 11n specification
		 */
		/*
		 * XXX - fixed rate for HT is not supported yet by
		 * net80211 layer
		 */
		mcs = ((ni->in_htcap & IEEE80211_HTCAP_GREENFIELD)
		    ? RT2860_PHY_HT_GF : RT2860_PHY_HT)
		    | ni->in_htrates.rs_rates[dp->txrate];

		if (ni->in_chw == 40) {
			mcs |= RT2860_PHY_BW40;
		}

		if (ic->ic_htcaps & ni->in_htcap &
		    (ni->in_chw != 40 ?
		    IEEE80211_HTCAP_SHORTGI20 : IEEE80211_HTCAP_SHORTGI40)) {
			mcs |= RT2860_PHY_SGI;
		}

		txwi->xflags |= 7 << RT2860_TX_BAWINSIZE_SHIFT;

		/* XXX - notyet implemented */
		dur = 20 + 6;

		txwi->txop = RT2860_TX_TXOP_HT;
	} else {
		/* 11a/11b/11g */
		/* get rate for data packets */
		if (ic->ic_fixed_rate != IEEE80211_FIXED_RATE_NONE) {
			rate = ic->ic_fixed_rate & IEEE80211_RATE_VAL;
		} else {
			rate = ni->in_rates.ir_rates[dp->txrate]
			    & IEEE80211_RATE_VAL;
		}

		if (!RT2860_RATE_IS_OFDM(rate)) {
			mcs = RT2860_PHY_CCK;
			if (rate != 2 &&
			    (ic->ic_flags & IEEE80211_F_SHPREAMBLE)) {
				mcs |= RT2860_PHY_SHPRE;
			}
		} else {
			mcs = RT2860_PHY_OFDM;
			if (ic->ic_flags & IEEE80211_F_SHPREAMBLE) {
				mcs |= RT2860_PHY_SHPRE;
			}
		}

		/* get MCS code from rate */
		mcs |= rt2870_rate2mcs(rate);

		dur = rt2870_txtime(RT2860_ACK_SIZE,
		    rt2870_ack_rate(ic, rate), ic->ic_flags) + sc->sifs;

		txwi->txop = RT2860_TX_TXOP_HT;
	}
	txwi->phy = LE_16(mcs);

	/*
	 * We store the MCS code into the driver-private PacketID field.
	 * The PacketID is latched into TX_STAT_FIFO when Tx completes so
	 * that we know at which initial rate the frame was transmitted.
	 * We add 1 to the MCS code because setting the PacketID field to
	 * 0 means that we don't want feedback in TX_STAT_FIFO.
	 */
	pid = ((mcs + 1) & 0xf) << RT2860_TX_PID_SHIFT;
	txwi->len |= LE_16(pid);
#if 0
	if (type == IEEE80211_FC0_TYPE_DATA) {
		txwi->txop = RT2860_TX_TXOP_HT;
	} else {
		txwi->txop = RT2860_TX_TXOP_BACKOFF;
	}
#endif
	if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		txwi->xflags |= RT2860_TX_ACK;
#if 0
		dur = rt2870_txtime(RT2860_ACK_SIZE, rt2870_ack_rate(ic, rate),
		    ic->ic_flags) + sc->sifs;
#endif
		wh->i_dur[0] = (uint8_t)dur;
		wh->i_dur[1] = (uint8_t)(dur >> 8);
	}

	if (/* (ni->in_flags & IEEE80211_NODE_HT) */
	    ic->ic_curmode == IEEE80211_MODE_11NG && ni->in_htstbc) {
		txwi->flags |= RT2860_TX_TS;
	}

	if (ampdu) {
		txwi->flags |=
		    (4 << RT2860_TX_MPDU_DSITY_SHIFT) | RT2860_TX_AMPDU;
	}

	DPRINTF(10, (CE_CONT, "!%s: %s: "
	    "sending frame qid=%d len:%d wcid=%d rate=%d mcs=%d dur=%d",
	    dp->name, __func__,
	    qid, LE16P(&txd->len) , txwi->wcid, rate, mcs, dur));
#ifdef DEBUG_LEVEL
	if (rt2870_debug > 10) {
#if 0
		int	i;
		for (i = 0; i < off; i += 4) {
			cmn_err(CE_CONT, "%02x %02x %02x %02x",
			    m->b_rptr[i + 0], m->b_rptr[i + 1],
			    m->b_rptr[i + 2], m->b_rptr[i + 3]);
		}
#endif
		uwgem_dump_pkt(__func__, m->b_rptr + off, pktlen, rate, 0);
	}
#endif
	/*
	 * XXX - additional 4 byte zero padds are required at the tail
	 * of usb packet, otherwise tx packets are ignored without errors.
	 */
	m->b_wptr = m->b_rptr + off + P2ROUNDUP(pktlen + pad, 4) + 4;
	((uint32_t *)m->b_wptr)[-1] = 0;

	return (m);

fail:
	if (m) {
		freemsg(m);
	}
	return (NULL);
}

/*
 * Return the Rx chain with the highest RSSI for a given frame.
 */
static uint8_t
rt2860_maxrssi_chain(struct uwgem_dev *dp, const struct rt2860_rxwi *rxwi)
{
	struct rt2870_softc *sc = dp->private;
	uint8_t rxchain = 0;
	uint8_t	rssi[3];

	rssi[0] = (rxwi->rssi[0] + (dp->ic.ic_maxrssi + 1) / 2)
	    % (dp->ic.ic_maxrssi + 1);
	rssi[1] = (rxwi->rssi[1] + (dp->ic.ic_maxrssi + 1) / 2)
	    % (dp->ic.ic_maxrssi + 1);
	rssi[2] = (rxwi->rssi[2] + (dp->ic.ic_maxrssi + 1) / 2)
	    % (dp->ic.ic_maxrssi + 1);

	DPRINTF(10, (CE_CONT, "!%s: %s: rssi %d %d %d",
	    dp->name, __func__,
	    rxwi->rssi[0], rxwi->rssi[1], rxwi->rssi[2]));
	if (sc->nrxchains > 1)
		if (rssi[1] > rssi[rxchain])
			rxchain = 1;
	if (sc->nrxchains > 2)
		if (rssi[2] > rssi[rxchain])
			rxchain = 2;

	return (rxchain);
}

static mblk_t *
rt2870_rx_make_packet(struct uwgem_dev *dp, mblk_t *mp_head)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	mblk_t		*mp;
	mblk_t		*new;
	mblk_t		**tail;
	uint32_t	len;
	uint32_t	pktlen;
	uint_t		total;
	uint_t		rest;
	uint8_t		*rxbuf;
	uint8_t		ant;
	uint_t		rssi;
	uint32_t	flags;
	struct rt2870_rxd	*rxd;
	struct rt2860_rxwi	*rxwi;
	int		l2pad;
	uint8_t		fc1;

	total = (uintptr_t)(mp_head->b_wptr - mp_head->b_rptr);
	rest = total;
	rxbuf = (uint8_t *)mp_head->b_rptr;

	tail = &new;

	/* reuse mp_head */
	mp = mp_head;
	mp->b_next = NULL;

	while (rest > sizeof (uint32_t) +
	    sizeof (struct rt2860_rxwi) + sizeof (struct rt2870_rxd)) {

		len = rxbuf[1] << 8 | rxbuf[0];

		/* validate the length */
		if (len & 3 ||
		    rest <
		    sizeof (uint32_t) + len + sizeof (struct rt2870_rxd)) {
			/* corrupted packet */
			cmn_err(CE_CONT,
			    "!%s: %s: packet length error: rest %u, len %u",
			    dp->name, __func__, rest, len);
			DPRINTF(-1, (CE_CONT,
			    "!%s: %s: rest:%d (total:%d) len:%d(0x%x)"
			    " pktlen:%d+%d, flags:0x%b",
			    dp->name, __func__,
			    rest, total,
			    len, rxbuf[3]<<24|rxbuf[2]<<16|rxbuf[1]<<8|rxbuf[0],
			    pktlen, sizeof (*rxwi),
			    flags, RT2870_RX_FLAG_BITS));
			break;
		}

		rxd = (struct rt2870_rxd *)(rxbuf + sizeof (uint32_t) + len);
		rxwi = (struct rt2860_rxwi *)(rxbuf + sizeof (uint32_t));
		fc1  = ((uint8_t *)(rxwi + 1))[1];

		pktlen = rxwi->len;
		pktlen = LE_16(pktlen) & 0xfff;

		flags = rxd->flags;
		flags = LE_32(flags);
#if 1
if (fc1 & IEEE80211_FC1_ORDER) {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: rest:%d(total:%d) len:%d(0x%x)"
		    " pktlen:%d+%d, flags:0x%b",
		    dp->name, __func__,
		    rest, total,
		    len, rxbuf[3]<<24|rxbuf[2]<<16|rxbuf[1]<<8|rxbuf[0],
		    pktlen, sizeof (*rxwi),
		    flags, RT2870_RX_FLAG_BITS));
}
#endif
		/* HW may insert 2 padding bytes after 802.11 header */
		if (rxd->flags & LE_32(RT2860_RX_L2PAD)) {
#if 0
			RUN_DEBUG(RT2870_DBG_RX,
			    "!%s: %s: 2 padding bytes after 80211 header!",
			    dp->name, __func__);
#endif
			/* required for firmware v22 */
			pktlen += 2;
		}

		if (len < sizeof (struct rt2860_rxwi) + pktlen) {
#if 0
			cmn_err(CE_CONT,
			    "!%s: %s: bad RXWI length %u > %u, rest:%d",
			    dp->name, __func__,
			    pktlen, len - sizeof (struct rt2860_rxwi), rest);
			break;
#else
			uwgem_dump_pkt(__func__,
			    (void *)(rxwi + 1), pktlen, 0, 0);
			pktlen = len - sizeof (struct rt2860_rxwi);
#endif
		}


		if (flags & (RT2860_RX_CRCERR | RT2860_RX_ICVERR)) {
#if 0
			RUN_DEBUG(RT2870_DBG_RX,
			    "!%s: %s: rx crc error & rx icv error!",
			    dp->name, __func__);
#endif
			dp->stats.crc++;
			dp->stats.errrcv++;
			goto next;
		}

		if (flags & (RT2860_RX_MICERR)) {
			RUN_DEBUG(RT2870_DBG_RX,
			    "!%s: %s: rx mic error!", dp->name, __func__);
			dp->stats.errrcv++;
			goto next;
		}

		ant = rt2860_maxrssi_chain(dp, rxwi);
		rssi = (rxwi->rssi[ant] + (dp->ic.ic_maxrssi + 1) / 2)
		    % (dp->ic.ic_maxrssi + 1);

		if (mp == NULL) {
			/* prepare a new mblk */
			if ((mp = dupb(mp_head)) == NULL) {
				RUN_DEBUG(RT2870_DBG_RX,
				    "!%s: %s: alloc mblk error",
				    dp->name, __func__);
				dp->stats.norcvbuf++;
				break;
			}
		}

		/* add the wifi frame into the receive list */
		mp->b_rptr = (void *)(rxwi + 1);
		mp->b_wptr = mp->b_rptr + pktlen;
#ifdef CONFIG_DATAPAD
		if (rxd->flags & LE_32(RT2860_RX_L2PAD)) {
			memmove(mp->b_rptr + 2, mp->b_rptr,
			    sizeof (struct ieee80211_qosframe));
			mp->b_rptr += 2;
		}
#endif
		if (fc1 & IEEE80211_FC1_ORDER) {
			memmove(mp->b_rptr + 4, mp->b_rptr,
			    sizeof (struct ieee80211_qosframe));
			mp->b_rptr += 4;
		}
		mp->b_cont = (void *)(uintptr_t)rssi;
		mp->b_prev = (void *)(uintptr_t)0;	/* S/N */
#ifdef DEBUG_LEVEL
		if (rt2870_debug > 10) {
			uwgem_dump_pkt(__func__,
			    mp->b_rptr, mp->b_wptr - mp->b_rptr,
			    0 /* rate */, (uintptr_t)mp->b_cont);
		}
#endif
		*tail = mp;
		tail = &mp->b_next;
		mp = NULL;
next:
		rxbuf = (uint8_t *)(rxd + 1);
		rest -= sizeof (uint32_t) + len + sizeof (struct rt2870_rxd);
	}

	/* terminate the recieve list */
	*tail = NULL;

	/* free unused mblk */
	if (mp && mp != mp_head) {
		freemsg(mp);
	}
	ASSERT(new == NULL || new == mp_head);
	return (new);
}

/* ======================================================== */
/*
 * State change and the related managements
 */
/* ======================================================== */
static int
rt2870_updateslot(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	uint32_t tmp;
	int err;
	extern char *ieee80211_phymode_name[];

	DPRINTF(4, (CE_CONT, "!%s: %s: curmode:%s (%d)",
	    dp->name, __func__,
	    ieee80211_phymode_name[ic->ic_curmode], ic->ic_curmode));

	RT2870_READ(dp, RT2860_BKOFF_SLOT_CFG, &tmp, &err, usberr);
	tmp &= ~0xff;
	tmp |= (ic->ic_flags & IEEE80211_F_SHSLOT) ? 9 : 20;
	RT2870_WRITE(dp, RT2860_BKOFF_SLOT_CFG, tmp, &err, usberr);

usberr:
	return (err);
}

static void
rt2870_amrr_node_init(const struct rt2870_amrr *amrr,
    struct rt2870_amrr_node *amn)
{
	amn->amn_success = 0;
	amn->amn_recovery = 0;
	amn->amn_txcnt = amn->amn_retrycnt = 0;
	amn->amn_success_threshold = amrr->amrr_min_success_threshold;
}

static void
rt2870_amrr_choose(struct uwgem_dev *dp, struct rt2870_amrr *amrr,
    struct ieee80211_node *ni, struct rt2870_amrr_node *amn)
{
	struct rt2870_softc *sc = dp->private;
	int	txrate;
	int	nrates;
	int	min_rate;
#define	RV(rate)	((rate) & IEEE80211_RATE_VAL)
#define	is_success(amn)	\
	((amn)->amn_retrycnt < (amn)->amn_txcnt / 5)
#define	is_failure(amn)	\
	((amn)->amn_retrycnt > (amn)->amn_txcnt / 3)
#define	is_enough(amn)		\
	((amn)->amn_txcnt > 10)
#define	is_min_rate(ni)		\
	(txrate == min_rate)
#define	is_max_rate(ni)		\
	(txrate == nrates - 1)
#define	increase_rate(ni)	\
	(txrate++)
#define	decrease_rate(ni)	\
	(txrate--)
#define	reset_cnt(amn)		\
	(amn)->amn_txcnt = (amn)->amn_retrycnt = 0;
	int need_change = 0;
#if 1
	txrate = dp->txrate;
	if (ni->in_flags & IEEE80211_NODE_HT) {
		nrates = min(ni->in_htrates.rs_nrates, sc->ntxchains * 8);
		min_rate = (sc->ntxchains - 1) * 8;
	} else {
		nrates = ni->in_rates.ir_nrates;
		min_rate = 0;
	}
#endif
	RUN_DEBUG(RT2870_DBG_RADIO, "!run: %s: retry::%d txcnt:%d",
	    __func__,
	    amn->amn_retrycnt, amn->amn_txcnt);

	if (is_success(amn) && is_enough(amn)) {
		amn->amn_success++;
		if (amn->amn_success >= amn->amn_success_threshold &&
		    !is_max_rate(ni)) {
			amn->amn_recovery = 1;
			amn->amn_success = 0;
			increase_rate(ni);
			DPRINTF(10, (CE_CONT, "!%s: %s: "
			    "increase rate = %d, #tx = %d, #retries = %d",
			    dp->name, __func__,
			    (ni->in_flags & IEEE80211_NODE_HT)
			    ? ieee80211_htrates[RV(ni->in_htrates.rs_rates[dp->txrate])]
			    : RV(ni->in_rates.ir_rates[dp->txrate]),
			    amn->amn_txcnt, amn->amn_retrycnt));
			need_change = 1;
		} else {
			amn->amn_recovery = 0;
		}
	} else if (is_failure(amn)) {
		amn->amn_success = 0;
		if (!is_min_rate(ni)) {
			if (amn->amn_recovery) {
				amn->amn_success_threshold *= 2;
				if (amn->amn_success_threshold >
				    amrr->amrr_max_success_threshold) {
					amn->amn_success_threshold =
					    amrr->amrr_max_success_threshold;
				}
			} else {
				amn->amn_success_threshold =
				    amrr->amrr_min_success_threshold;
			}
			decrease_rate(ni);
			DPRINTF(10, (CE_CONT, "!%s: %s: "
			    "decrease rate = %d, #tx = %d, #retries = %d",
			    dp->name, __func__,
			    (ni->in_flags & IEEE80211_NODE_HT)
			    ? ieee80211_htrates[RV(ni->in_htrates.rs_rates[dp->txrate])]
			    : RV(ni->in_rates.ir_rates[dp->txrate]),
			    amn->amn_txcnt, amn->amn_retrycnt));
			need_change = 1;
		}
		amn->amn_recovery = 0;
	}

	if (is_enough(amn) || need_change) {
		reset_cnt(amn);
	}
#if 1
	/* update tx rate */
	dp->txrate = txrate;
	if ((ni->in_flags & IEEE80211_NODE_HT) == 0) {
		/* for backward compatibity */
		ni->in_txrate = txrate;
	}
#endif
#undef RV
}

static void
rt2870_iter_func(void *arg, struct ieee80211_node *ni)
{
	struct uwgem_dev *dp = (struct uwgem_dev *)arg;
	struct rt2870_softc *sc = dp->private;
	uint8_t wcid;

	wcid = RT2870_AID2WCID(ni->in_associd);
	rt2870_amrr_choose(dp, &sc->amrr, ni, &sc->amn[wcid]);
	DPRINTF(10, (CE_CONT, "!%s: %s: wcid:0x%x",
	    dp->name, __func__, wcid));
}

static void
rt2870_updatestats(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	clock_t	now;

	now = ddi_get_lbolt();
	if (now - sc->last_updatestats > drv_usectohz(500*1000)) {
		if (ic->ic_opmode == IEEE80211_M_STA) {
			rt2870_iter_func(dp, ic->ic_bss);
		} else {
			ieee80211_iterate_nodes(&ic->ic_sta,
			    rt2870_iter_func, dp);
		}
		sc->last_updatestats = now;
	}
}

static int
rt2870_enable_mrr(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	int err;

#define	CCK(mcs)	(mcs)
#define	OFDM(mcs)	((uint32_t)1 << 3 | (mcs))
	RT2870_WRITE(dp, RT2860_LG_FBK_CFG0,
	    OFDM(6) << 28 |	/* 54->48 */
	    OFDM(5) << 24 |	/* 48->36 */
	    OFDM(4) << 20 |	/* 36->24 */
	    OFDM(3) << 16 |	/* 24->18 */
	    OFDM(2) << 12 |	/* 18->12 */
	    OFDM(1) <<  8 |	/* 12-> 9 */
	    OFDM(0) <<  4 |	/*  9-> 6 */
	    OFDM(0),		/*  6-> 6 */
	    &err, usberr);

	RT2870_WRITE(dp, RT2860_LG_FBK_CFG1,
	    CCK(2) << 12 |	/* 11->5.5 */
	    CCK(1) <<  8 |	/* 5.5-> 2 */
	    CCK(0) <<  4 |	/*   2-> 1 */
	    CCK(0),		/*   1-> 1 */
	    &err, usberr);
#undef	OFDM
#undef	CCK
usberr:
	return (err);
}

static int
rt2870_set_txpreamble(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	uint32_t tmp;
	int err;

	RT2870_READ(dp, RT2860_AUTO_RSP_CFG, &tmp, &err, usberr);
	if (dp->ic.ic_flags & IEEE80211_F_SHPREAMBLE) {
		tmp |= RT2860_CCK_SHORT_EN;
	} else {
		tmp &= ~RT2860_CCK_SHORT_EN;
	}
	RT2870_WRITE(dp, RT2860_AUTO_RSP_CFG, tmp, &err, usberr);
	DPRINTF(1, (CE_CONT, "!%s: %s: auto_rsp_cfg:0x%x",
	    dp->name, __func__, tmp));
usberr:
	return (err);
}

static int
rt2870_set_bssid(struct uwgem_dev *dp, const uint8_t *bssid)	/* ok */
{
	int err;

	RT2870_WRITE(dp, RT2860_MAC_BSSID_DW0,
	    bssid[0] | bssid[1] << 8 | bssid[2] << 16 | bssid[3] << 24,
	    &err, usberr);
	RT2870_WRITE(dp, RT2860_MAC_BSSID_DW1,
	    bssid[4] | bssid[5] << 8,
	    &err, usberr);
	DPRINTF(4, (CE_CONT, "!%s: %s: %x:%x:%x:%x:%x:%x",
	    dp->name, __func__,
	    bssid[0], bssid[1], bssid[2], bssid[3],
	    bssid[4], bssid[5]));
usberr:
	return (err);
}

static int
rt2870_set_basicrates(struct uwgem_dev *dp)
{
	struct rt2870_softc *sc = dp->private;
	struct ieee80211com *ic = &dp->ic;
	int err;

	DPRINTF(1, (CE_CONT, "!%s: %s: ic_curmode:0x%x",
	    dp->name, __func__, ic->ic_curmode));

	/* set basic rates mask */
	if (ic->ic_curmode == IEEE80211_MODE_11B) {
		RT2870_WRITE(dp, RT2860_LEGACY_BASIC_RATE, 0x003,
		    &err, usberr);
	} else if (ic->ic_curmode == IEEE80211_MODE_11A) {
		RT2870_WRITE(dp, RT2860_LEGACY_BASIC_RATE, 0x150,
		    &err, usberr);
	} else {	/* 11g */
		RT2870_WRITE(dp, RT2860_LEGACY_BASIC_RATE, 0x15f,
		    &err, usberr);
	}
usberr:
	return (err);
}

static void
rt2870_newassoc(struct ieee80211com *ic, struct ieee80211_node *in, int isnew)
{
	struct uwgem_dev *dp = (struct uwgem_dev *)ic;
	struct rt2870_softc *sc = dp->private;
	uint32_t off;
	uint8_t *fptr, wcid = 0;
	int i;

	if (isnew && in->in_associd != 0) {
		/* only interested in true associations */
		wcid = RT2870_AID2WCID(in->in_associd);

		/* init WCID table entry */
		rt2870_write_region(dp,
		    RT2860_WCID_ENTRY(wcid),
		    in->in_macaddr, IEEE80211_ADDR_LEN);
	}
	/* solaris wifi framework doesn't have ieee80211_amrr_node_init() */
	rt2870_amrr_node_init(&sc->amrr, &sc->amn[wcid]);

	/* set rate to some reasonable initial value */
	if (in->in_flags & IEEE80211_NODE_HT) {
		in->in_txrate = in->in_rates.ir_nrates - 1;
		dp->txrate =
		    min(in->in_htrates.rs_nrates, 8 * sc->ntxchains) - 1;
	} else {
		i = in->in_rates.ir_nrates - 1;
		while (i > 0 &&
		    (in->in_rates.ir_rates[i] & IEEE80211_RATE_VAL) > 72) {
			i--;
		}
		dp->txrate = i;
		in->in_txrate = i;
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: new assoc isnew=%d wcid=%d, initial rate=%d,"
	    " in_flags:0x%x, in_htcap:0x%x, in_htparam:0x%x,"
	    " ctrlchan:%d, 2ndchan:%d, htopmode:0x%x,"
	    " reqcw:%d, chw:%d flags:%x htstbc:%d",
	    dp->name, __func__, isnew, wcid,
	    (in->in_flags & IEEE80211_NODE_HT)
	    ? in->in_htrates.rs_rates[dp->txrate] & IEEE80211_RATE_VAL
	    : in->in_rates.ir_rates[dp->txrate] & IEEE80211_RATE_VAL,
	    in->in_flags, in->in_htcap, in->in_htparam,
	    in->in_htctlchan, in->in_ht2ndchan, in->in_htopmode,
	    in->in_reqcw, in->in_chw, in->in_chan ? in->in_chan->ich_flags : 0,
	    in->in_htstbc));
	DPRINTF(1, (CE_CONT,
	    "!%s: %s: addr=%x:%x:%x:%x:%x:%x",
	    dp->name, __func__,
	    in->in_macaddr[0], in->in_macaddr[1], in->in_macaddr[2],
	    in->in_macaddr[3], in->in_macaddr[4], in->in_macaddr[5]));
#if DEBUG_LEVEL >= 0
	cmn_err(CE_CONT, "!b/g rateset: num:%d", in->in_rates.ir_nrates);
	for (i = 0; i < in->in_rates.ir_nrates; i++) {
		int	rate = in->in_rates.ir_rates[i] & IEEE80211_RATE_VAL;
		cmn_err(CE_CONT, "!%d (%dMbps)", rate, rate / 2);
	}
	ieee80211_ht_announce(ic);
#endif
#if DEBUG_LEVEL >= 0
	if (in->in_flags & IEEE80211_NODE_HT) {
		cmn_err(CE_CONT,
		    "!htrateset: num:%d", in->in_htrates.rs_nrates);
		for (i = 0; i < in->in_htrates.rs_nrates; i++) {
			int	mcs = in->in_htrates.rs_rates[i];
			cmn_err(CE_CONT, "!%d (%dMbps)",
			    mcs, ieee80211_htrates[mcs] * in->in_chw / 40);
		}
		ieee80211_ht_announce(ic);
	}
#endif
}

static int
rt2870_enable_tsf_sync(struct uwgem_dev *dp)
{
	struct ieee80211com *ic = &dp->ic;
	uint32_t tmp;
	int err;

	RT2870_READ(dp, RT2860_BCN_TIME_CFG, &tmp, &err, usberr);

	tmp &= ~0x1fffff;
	tmp |= ic->ic_bss->in_intval * 16;
	tmp |= RT2860_TSF_TIMER_EN | RT2860_TBTT_TIMER_EN;

	if (ic->ic_opmode == IEEE80211_M_STA) {
		/*
		 * Local TSF is always updated with remote TSF on beacon
		 * reception.
		 */
		tmp |= 1 << RT2860_TSF_SYNC_MODE_SHIFT;
	}

	RT2870_WRITE(dp, RT2860_BCN_TIME_CFG, tmp, &err, usberr);
	DPRINTF(1, (CE_CONT, "!%s: %s: bcn_time_cfg:0x%x",
	    dp->name, __func__, tmp));

usberr:
	return (err);
}

static int
rt2870_newstate(struct uwgem_dev *dp, enum ieee80211_state nstate, int arg)
{
	struct ieee80211com *ic = &dp->ic;
	struct rt2870_softc *sc = dp->private;
	enum ieee80211_state ostate;
	struct ieee80211_node *ni;
	uint32_t tmp, sta[3];
	uint8_t wcid;
	int err = USB_SUCCESS;
	int	do_scan = 0;

	ostate = ic->ic_state;
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: %s(%x) -> %s(%x), chan:%d", dp->name, __func__,
	    ieee80211_state_name[ostate], ostate,
	    ieee80211_state_name[nstate], nstate,
	    ieee80211_chan2ieee(ic, ic->ic_curchan)));

	if (ostate == IEEE80211_S_RUN) {
		/* turn link LED on */
		(void) rt2870_set_leds(dp, RT2860_LED_RADIO);

		/* Set media status to 'No Link'. */
		/* Stop Rx of data frames. */
		/* Rest TSF. */
		/* disable TSF sync */
		RT2870_READ(dp, RT2860_BCN_TIME_CFG, &tmp,
		    &err, usberr);
		tmp &= ~(RT2860_BCN_TX_EN
		    | RT2860_TBTT_TIMER_EN
		    | (3 << RT2860_TSF_SYNC_MODE_SHIFT)
		    | RT2860_TSF_TIMER_EN);
		RT2870_WRITE(dp, RT2860_BCN_TIME_CFG, tmp,
		    &err, usberr);

		/* back to 20Mhz mode */
		rt2870_set_chan(dp, ic->ic_curchan, nstate, 0);

		/* flush all cam entries */

		/* clear ht state */
		sc->cur_htcap = -1;
		sc->cur_htopmode = -1;
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
		/* clear ht state */
		sc->cur_htcap = -1;
		sc->cur_htopmode = -1;

		/* abort TSF synchronization */
		RT2870_READ(dp, RT2860_BCN_TIME_CFG, &tmp,
		    &err, usberr);
		tmp &= ~(RT2860_BCN_TX_EN
		    | RT2860_TBTT_TIMER_EN
		    | (3 << RT2860_TSF_SYNC_MODE_SHIFT)
		    | RT2860_TSF_TIMER_EN);
		RT2870_WRITE(dp, RT2860_BCN_TIME_CFG, tmp,
		    &err, usberr);

		(void) rt2870_updateslot(dp);
		(void) rt2870_enable_mrr(dp);
		(void) rt2870_set_txpreamble(dp);
		(void) rt2870_set_basicrates(dp);
		rt2870_updateprot(dp);
		break;

	case IEEE80211_S_SCAN:
		rt2870_set_chan(dp, ic->ic_curchan, nstate, 0);
		rt2870_updateprot(dp);
		break;

	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
		rt2870_set_chan(dp, ic->ic_curchan, nstate, 0);
		rt2870_updateprot(dp);
		break;

	case IEEE80211_S_RUN:
		rt2870_updateprot(dp);
		rt2870_updateprot_ht(dp);
		ni = ic->ic_bss;
		ieee80211_fix_rate(ni, &ni->in_rates, IEEE80211_F_DOSORT);
		ieee80211_setbasicrates(&ni->in_rates, ic->ic_curmode);

		rt2870_set_chan(dp, ic->ic_curchan, nstate,
		    (ni->in_chw == 40 &&
		    (ni->in_chan->ich_flags & IEEE80211_CHAN_HT40))
		    ? (ni->in_ht2ndchan & IEEE80211_HTINFO_2NDCHAN)
		    : IEEE80211_HTINFO_2NDCHAN_NONE);

		if (ic->ic_opmode != IEEE80211_M_MONITOR) {
			(void) rt2870_updateslot(dp);
			(void) rt2870_enable_mrr(dp);
			(void) rt2870_set_txpreamble(dp);
			(void) rt2870_set_basicrates(dp);
			(void) rt2870_set_bssid(dp, ni->in_bssid);
		}
		if (ic->ic_opmode == IEEE80211_M_STA) {
			/* fake a join to init the tx rate */
			rt2870_newassoc(ic, ic->ic_bss, 1);
		}

		if (ic->ic_opmode != IEEE80211_M_MONITOR) {
			(void) rt2870_enable_tsf_sync(dp);

			/* clear statistic registers used by AMRR */
			RT2870_READ_REGION(dp, RT2860_TX_STA_CNT0,
			    (uint8_t *)sta, sizeof (sta), &err, usberr);
		}

		/* turn link LED on */
		(void) rt2870_set_leds(dp, RT2860_LED_RADIO |
		    (IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan) ?
		    RT2860_LED_LINK_2GHZ : RT2860_LED_LINK_5GHZ));
		break;
	}
usberr:
	return (err);
}

/* ======================================================== */
/*
 * setup configuration
 */
/* ======================================================== */
#ifdef DEBUG_LEVEL
static void
rt2870_eeprom_dump(struct uwgem_dev *dp, int size)
{
	struct rt2870_softc	*sc = dp->private;
	int	err;
	int	i;
	uint16_t val;
	uint32_t tmp;
	uint16_t	w0, w1, w2, w3;

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);

	err = USB_SUCCESS;

	for (i = 0; i < size; i += 4) {
		w0 = (*sc->sc_eeprom_read)(dp, i + 0);
		w1 = (*sc->sc_eeprom_read)(dp, i + 1);
		w2 = (*sc->sc_eeprom_read)(dp, i + 2);
		w3 = (*sc->sc_eeprom_read)(dp, i + 3);
		cmn_err(CE_CONT, "!0x%02x: 0x%04x 0x%04x 0x%04x 0x%04x",
		    i, w0, w1, w2, w3);
	}
usberr:
	;
}
#endif

/*
 * Add `delta' (signed) to each 4-bit sub-word of a 32-bit word.
 * Used to adjust per-rate Tx power registers.
 */
static uint32_t
b4inc(uint32_t b32, int8_t delta)
{
	int8_t i, b4;

	for (i = 0; i < 8; i++) {
		b4 = b32 & 0xf;
		b4 += delta;
		if (b4 < 0) {
			b4 = 0;
		} else if (b4 > 0xf) {
			b4 = 0xf;
		}
		b32 = b32 >> 4 | b4 << 28;
	}
	return (b32);
}

/* Read 16-bit from eFUSE ROM (RT3070 only.) */
static uint16_t
rt2870_efuse_read_2(struct uwgem_dev *dp, uint16_t addr)
{
	struct rt2870_softc	*sc = dp->private;
	uint32_t	tmp;
	uint16_t	reg;
	int		err, ntries;

	RT2870_READ(dp, RT3070_EFUSE_CTRL, &tmp, &err, usberr);

	/* convert addr in word to addr in byte */
	addr *= 2;
	/*
	 * Read one 16-byte block into registers EFUSE_DATA[0-3]:
	 * DATA0: F E D C
	 * DATA1: B A 9 8
	 * DATA2: 7 6 5 4
	 * DATA3: 3 2 1 0
	 */
	tmp &= ~(RT3070_EFSROM_MODE_MASK | RT3070_EFSROM_AIN_MASK);
	tmp |= (addr & ~0xf) << RT3070_EFSROM_AIN_SHIFT | RT3070_EFSROM_KICK;
	RT2870_WRITE(dp, RT3070_EFUSE_CTRL, tmp, &err, usberr);
	for (ntries = 0; ; ntries++) {
		RT2870_READ(dp, RT3070_EFUSE_CTRL, &tmp, &err, usberr);
		if (!(tmp & RT3070_EFSROM_KICK)) {
			break;
		}
		if (ntries >= 500) {
			cmn_err(CE_WARN, "!%s: timeout", __func__);
			return (0xffff);
		}
		DELAY(2);
	}

	if ((tmp & RT3070_EFUSE_AOUT_MASK) == RT3070_EFUSE_AOUT_MASK) {
		DPRINTF(10, (CE_WARN, "!%s: illegal address:0x%04x, ctrl:%x",
		    __func__, addr/2, tmp));
		return (0xffff);	/* address not found */
	}
	DPRINTF(10, (CE_CONT, "!%s: %s: ctrl:%x", dp->name, __func__, tmp));

	/* determine to which 32-bit register our 16-bit word belongs */
	reg = RT3070_EFUSE_DATA3 - (addr & 0xc);
	RT2870_READ(dp, reg, &tmp, &err, usberr);

	return ((addr & 2) ? tmp >> 16 : tmp & 0xffff);
usberr:
	cmn_err(CE_WARN, "!%s: usb i/o error", __func__);
	return (0xffff);
}

static uint16_t
rt2870_eeprom_read_2(struct uwgem_dev *dp, uint16_t addr)
{
	int	err;
	uint16_t	val;

	RT2870_EEPROM_READ2(dp, addr, &val, &err, usberr);

	return (val);
usberr:
	DPRINTF(1, (CE_CONT, "!%s: io error", __func__));
	return (0xffffU);
}

static int
rt2870_read_eeprom(struct uwgem_dev *dp)
{
	struct rt2870_softc	*sc = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	uint16_t	(*rt2870_eeprom_read)(struct uwgem_dev *, uint16_t);
	int	err;
	int8_t	delta_2ghz, delta_5ghz;
	uint32_t	tmp;
	uint16_t	val;
	int	ridx, ant, i;

	rt2870_eeprom_read = sc->sc_eeprom_read;

	/* read EEPROM version */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_VERSION);
	RUN_DEBUG(RT2870_DBG_EEPROM, "!%s: EEPROM rev=%d, FAE=%d",
	    dp->name, val & 0xff, val >> 8);

	/* read MAC address */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_MAC01);
	dp->dev_addr[0] = val & 0xff;
	dp->dev_addr[1] = val >> 8;
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_MAC23);
	dp->dev_addr[2] = val & 0xff;
	dp->dev_addr[3] = val >> 8;
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_MAC45);
	dp->dev_addr[4] = val & 0xff;
	dp->dev_addr[5] = val >> 8;
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: MAC address is: %x:%x:%x:%x:%x:%x\n",
	    dp->name,
	    dp->dev_addr[0], dp->dev_addr[1],
	    dp->dev_addr[2], dp->dev_addr[3],
	    dp->dev_addr[4], dp->dev_addr[5]);

	/* read country code */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_COUNTRY);
	RUN_DEBUG(RT2870_DBG_EEPROM, "!%s: EEPROM region code=0x%04x",
	    dp->name, val);

	/* read default BBP settings */
	for (i = 0; i < 8; i++) {
		val = rt2870_eeprom_read(dp, RT2860_EEPROM_BBP_BASE + i);
		sc->bbp[i].val = val & 0xff;
		sc->bbp[i].reg = val >> 8;
		RUN_DEBUG(RT2870_DBG_EEPROM, "!%s: BBP%d=0x%02x",
		    dp->name, sc->bbp[i].reg, sc->bbp[i].val);
	}

	/* read RF frequency offset from EEPROM */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_FREQ_LEDS);
	sc->freq = ((val & 0xff) != 0xff) ? val & 0xff : 0;
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: EEPROM freq offset %d (should be 0-127)",
	    dp->name, sc->freq & 0xff);

	if ((sc->leds = (val >> 8)) != 0xff) {
		/* read LEDs operating mode */
		sc->led[0] = rt2870_eeprom_read(dp, RT2860_EEPROM_LED1);
		sc->led[1] = rt2870_eeprom_read(dp, RT2860_EEPROM_LED2);
		sc->led[2] = rt2870_eeprom_read(dp, RT2860_EEPROM_LED3);
	} else {
		/* broken EEPROM, use default settings */
		sc->leds = 0x01;
		sc->led[0] = 0x5555;
		sc->led[1] = 0x2221;
		sc->led[2] = 0x5627;	/* differs from RT2860 */
	}
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: EEPROM LED mode=0x%02x, LEDs=0x%04x/0x%04x/0x%04x",
	    dp->name, sc->leds, sc->led[0], sc->led[1], sc->led[2]);

	/* read RF information */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_ANTENNA);
	if (val == 0xffff) {
		RUN_DEBUG(RT2870_DBG_EEPROM,
		    "!%s: invalid EEPROM antenna info, using default",
		    dp->name);
		if (CHIP_RT3090(sc->mac_rev) || CHIP_RT3390(sc->mac_rev)) {
			/* default to RF3020 1T1R */
			sc->rf_rev = RT3070_RF_3020;
			sc->ntxchains = 1;
			sc->nrxchains = 1;
		} else {
			/* default to RF2820 1T2R */
			sc->rf_rev = RT2860_RF_2820;
			sc->ntxchains = 1;
			sc->nrxchains = 2;
		}
	} else {
		sc->rf_rev = (val >> 8) & 0xf;
		sc->ntxchains = (val >> 4) & 0xf;
#ifdef CONFIG_MULTISTREAM
		sc->nrxchains = val & 0xf;
#else
		sc->nrxchains = 1;
#endif
	}
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: EEPROM RF rev=0x%02x chains=%dT%dR",
	    dp->name, sc->rf_rev, sc->ntxchains, sc->nrxchains);

	/* check if RF supports automatic Tx access gain control */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_CONFIG);
	RUN_DEBUG(RT2870_DBG_EEPROM, "!%s: EEPROM CFG 0x%04x", dp->name, val);
	if ((val & 0xff) != 0xff) {
		sc->ext_5ghz_lna = (val >> 3) & 1;	/* for a */
		sc->ext_2ghz_lna = (val >> 2) & 1;	/* for g */
		sc->calib_2ghz = sc->calib_5ghz = (val >> 1) & 1;
	}
	if ((val & 0xff00) != 0xff00) {
		sc->dac_test_bit = (val >> 15) & 1;
	}

	/* read power settings for 2GHz channels */
	for (i = 0; i < 14; i += 2) {
		val = rt2870_eeprom_read(dp,
		    RT2860_EEPROM_PWR2GHZ_BASE1 + i / 2);
		sc->txpow1[i + 0] = (int8_t)(val & 0xff);
		sc->txpow1[i + 1] = (int8_t)(val >> 8);

		val = rt2870_eeprom_read(dp,
		    RT2860_EEPROM_PWR2GHZ_BASE2 + i / 2);
		sc->txpow2[i + 0] = (int8_t)(val & 0xff);
		sc->txpow2[i + 1] = (int8_t)(val >> 8);
	}
	/* fix broken Tx power entries */
	for (i = 0; i < 14; i++) {
		if (sc->txpow1[i] < 0 || sc->txpow1[i] > 31) {
			sc->txpow1[i] = 5;
		}
		if (sc->txpow2[i] < 0 || sc->txpow2[i] > 31) {
			sc->txpow2[i] = 5;
		}
		RUN_DEBUG(RT2870_DBG_EEPROM,
		    "!%s: chan %d: power1=%d, power2=%d",
		    dp->name,
		    rt2860_rf2850[i].chan, sc->txpow1[i], sc->txpow2[i]);
	}
	/* read power settings for 5GHz channels */
	/*
	 * XXX - The max index of txpow in latest if_run.c is 40,
	 * instead of 36.
	 */
	for (i = 0; i < 40; i += 2) {
		val = rt2870_eeprom_read(dp,
		    RT2860_EEPROM_PWR5GHZ_BASE1 + i / 2);
		sc->txpow1[i + 14] = (int8_t)(val & 0xff);
		sc->txpow1[i + 15] = (int8_t)(val >> 8);

		val = rt2870_eeprom_read(dp,
		    RT2860_EEPROM_PWR5GHZ_BASE2 + i / 2);
		sc->txpow2[i + 14] = (int8_t)(val & 0xff);
		sc->txpow2[i + 15] = (int8_t)(val >> 8);
	}
	/* fix broken Tx power entries */
	for (i = 0; i < 40; i++) {
		if (sc->txpow1[14 + i] < -7 || sc->txpow1[14 + i] > 15) {
			sc->txpow1[14 + i] = 5;
		}
		if (sc->txpow2[14 + i] < -7 || sc->txpow2[14 + i] > 15) {
			sc->txpow2[14 + i] = 5;
		}
		RUN_DEBUG(RT2870_DBG_EEPROM,
		    "!%s: chan %d: power1=%d, power2=%d",
		    dp->name,
		    rt2860_rf2850[14 + i].chan, sc->txpow1[14 + i],
		    sc->txpow2[14 + i]);
	}

	/* read Tx power compensation for each Tx rate */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_DELTAPWR);
	delta_2ghz = delta_5ghz = 0;
	if ((val & 0xff) != 0xff && (val & 0x80)) {
		delta_2ghz = val & 0xf;
		if (!(val & 0x40)) {	/* negative number */
			delta_2ghz = -delta_2ghz;
		}
	}
	val >>= 8;
	if ((val & 0xff) != 0xff && (val & 0x80)) {
		delta_5ghz = val & 0xf;
		if (!(val & 0x40)) {	/* negative number */
			delta_5ghz = -delta_5ghz;
		}
	}
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: power compensation=%d (2GHz), %d (5GHz)",
	    dp->name, delta_2ghz, delta_5ghz);

	for (ridx = 0; ridx < 5; ridx++) {
		uint32_t	reg;

		val = rt2870_eeprom_read(dp, RT2860_EEPROM_RPWR + ridx*2);
		reg = ((uint32_t)val) << 16;
		val = rt2870_eeprom_read(dp, RT2860_EEPROM_RPWR + ridx*2 + 1);
		reg |= val;

		sc->txpow20mhz[ridx] = reg;
		sc->txpow40mhz_2ghz[ridx] = b4inc(reg, delta_2ghz);
		sc->txpow40mhz_5ghz[ridx] = b4inc(reg, delta_5ghz);

		RUN_DEBUG(RT2870_DBG_EEPROM, "!%s: "
		    "ridx %d: power 20MHz=0x%08x, 40MHz/2GHz=0x%08x, "
		    "40MHz/5GHz=0x%08x",
		    dp->name, ridx, sc->txpow20mhz[ridx],
		    sc->txpow40mhz_2ghz[ridx], sc->txpow40mhz_5ghz[ridx]);
	}

	/* read RSSI offsets and LNA gains from EEPROM */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_RSSI1_2GHZ);
	sc->rssi_2ghz[0] = val & 0xff;	/* Ant A */
	sc->rssi_2ghz[1] = val >> 8;	/* Ant B */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_RSSI2_2GHZ);
	sc->rssi_2ghz[2] = val & 0xff;	/* Ant C */
	sc->lna[2] = val >> 8;		/* channel group 2 */

	val = rt2870_eeprom_read(dp, RT2860_EEPROM_RSSI1_5GHZ);
	sc->rssi_5ghz[0] = val & 0xff;	/* Ant A */
	sc->rssi_5ghz[1] = val >> 8;	/* Ant B */
	val = rt2870_eeprom_read(dp, RT2860_EEPROM_RSSI2_5GHZ);
	sc->rssi_5ghz[2] = val & 0xff;	/* Ant C */
	sc->lna[3] = val >> 8;		/* channel group 3 */

	val = rt2870_eeprom_read(dp, RT2860_EEPROM_LNA);
	sc->lna[0] = val & 0xff;	/* channel group 0 */
	sc->lna[1] = val >> 8;		/* channel group 1 */

	/* fix broken 5GHz LNA entries */
	if (sc->lna[2] == 0 || sc->lna[2] == 0xff) {
		RUN_DEBUG(RT2870_DBG_EEPROM,
		    "!%s: invalid LNA for channel group %d", dp->name, 2);
		sc->lna[2] = sc->lna[1];
	}
	if (sc->lna[3] == 0 || sc->lna[3] == 0xff) {
		RUN_DEBUG(RT2870_DBG_EEPROM,
		    "!%s: invalid LNA for channel group %d", dp->name, 3);
		sc->lna[3] = sc->lna[1];
	}
	RUN_DEBUG(RT2870_DBG_EEPROM,
	    "!%s: LNA %x %x %x %x",
	    dp->name, sc->lna[0], sc->lna[1], sc->lna[2], sc->lna[3]);

	/* fix broken RSSI offset entries */
	for (ant = 0; ant < 3; ant++) {
		if (sc->rssi_2ghz[ant] < -10 || sc->rssi_2ghz[ant] > 10) {
			RUN_DEBUG(RT2870_DBG_EEPROM,
			    "!%s: invalid RSSI%d offset: %d (2GHz)",
			    ant + 1, dp->name, sc->rssi_2ghz[ant]);
			sc->rssi_2ghz[ant] = 0;
		}
		if (sc->rssi_5ghz[ant] < -10 || sc->rssi_5ghz[ant] > 10) {
			RUN_DEBUG(RT2870_DBG_EEPROM,
			    "!%s: invalid RSSI%d offset: %d (2GHz)",
			    dp->name, ant + 1, sc->rssi_5ghz[ant]);
			sc->rssi_5ghz[ant] = 0;
		}
	}

usberr:
	return (err);
}

static const char *
rt2870_get_rf(uint8_t rev)
{
	switch (rev) {
	case RT2860_RF_2820:
		return ("RT2820");

	case RT2860_RF_2850:
		return ("RT2850");

	case RT2860_RF_2720:
		return ("RT2720");

	case RT2860_RF_2750:
		return ("RT2750");

	case RT3070_RF_3020:
		return ("RT3020");

	case RT3070_RF_2020:
		return ("RT2020");

	case RT3070_RF_3021:
		return ("RT3021");

	case RT3070_RF_3022:
		return ("RT3022");

	case RT3070_RF_3052:
		return ("RT3052");

	default:
		return ("unknown");
	}
}

static int
rt2870_attach_chip(struct uwgem_dev *dp)
{
	int	i;
	int	err;
	int	ret;
	uint32_t	tmp;
	struct rt2870_softc	*sc = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	extern int ieee80211_debug;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

#if DEBUG_LEVEL > 0
	ieee80211_debug = IEEE80211_MSG_HT;
	ieee80211_debug = IEEE80211_MSG_CONFIG;
	ieee80211_debug = -1;
	ieee80211_debug &= ~(IEEE80211_MSG_ELEMID | IEEE80211_MSG_ASSOC | IEEE80211_MSG_HT);
	ieee80211_debug &= ~(IEEE80211_MSG_INPUT);
#if 0
	ieee80211_debug |= IEEE80211_MSG_HT;
#endif
	ieee80211_debug &= ~(IEEE80211_MSG_CRYPTO);
	ieee80211_debug &= ~(IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS);
#endif
	ret = USB_SUCCESS;

	sc->amrr.amrr_min_success_threshold =  1;
	sc->amrr.amrr_max_success_threshold = 10;

	sc->mac_rev = 0;
	RT2870_READ(dp, RT2860_ASIC_VER_ID, &sc->mac_rev, &err, usberr);
#ifdef DEBUG_RT30XX
	sc->mac_rev = DEBUG_RT30XX;
#endif

	/* select bulkout pipe for management */
	if ((sc->mac_rev >> 16) == 0x2860 || dp->num_bulkout_pipes < 6) {
		sc->mgtqid = 0;
	} else {
		sc->mgtqid = 5;
	}

	/* check whether the ROM is eFUSE ROM or EEPROM */
	sc->sc_eeprom_read = rt2870_eeprom_read_2;
	if ((sc->mac_rev & 0xfff00000) >= 0x30700000) {
		RT2870_READ(dp, RT3070_EFUSE_CTRL, &tmp, &err, usberr);
		DPRINTF(0,(CE_CONT, "!%s: EFUSE_CTRL=0x%08x", dp->name, tmp));
		if (tmp & RT3070_SEL_EFUSE) {
			sc->sc_eeprom_read = rt2870_efuse_read_2;
			DPRINTF(0, (CE_CONT, "!%s: eFUSE rom", dp->name));
		}
	}

	/* retrieve MAC address and various other things from EEPROM */
	(void) rt2870_read_eeprom(dp);

	cmn_err(CE_CONT,
	    "!%s: MAC/BBP RT%X (rev 0x%04X), RF %s (%dT%dR)",
	    dp->name,
	    sc->mac_rev >> 16, sc->mac_rev & 0xffff,
	    rt2870_get_rf(sc->rf_rev), sc->ntxchains, sc->nrxchains);

	/* setup capabilities */
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* set device capabilities */
	ic->ic_caps =
	    IEEE80211_C_TXPMGT |	/* tx power management */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_SHSLOT;		/* short slot time supported */
#ifdef CONFIG_11N
	ic->ic_caps |= IEEE80211_C_PMGT;
#endif
	/* WPA/WPA2 support */
	/* XXX - w/o this flag, dladm reported "cannot connect: operation not supported" */
	ic->ic_caps |= IEEE80211_C_WPA; /* Support WPA/WPA2 */
#ifdef CONFIG_IBSS
	ic->ic_caps |= IEEE80211_C_IBSS;	/* ad-hoc mode */
#endif

	/* 802.11n/HT */
#ifdef CONFIG_11N
	ic->ic_htcaps = IEEE80211_HTC_HT;
#ifdef CONFIG_AMSDU
	ic->ic_htcaps |= IEEE80211_HTC_AMSDU;
	ic->ic_htcaps |= IEEE80211_HTCAP_MAXAMSDU_7935;
#endif
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI20;
#ifdef CONFIG_BW40
	ic->ic_htcaps |= IEEE80211_HTCAP_CHWIDTH40;
#endif
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI40;
#ifdef CONFIG_AMPDU
	ic->ic_htcaps |= IEEE80211_HTC_AMPDU;
#endif
#endif /* CONFIG_11N */

	if (sc->rf_rev == RT2860_RF_2750 || sc->rf_rev == RT2860_RF_2850) {
		/* set supported .11a rates */
		ic->ic_sup_rates[IEEE80211_MODE_11A] = rt2870_rateset_11a;

		/* set supported .11a channels */
		for (i = 14; i < N(rt2860_rf2850); i++) {
			uint8_t chan = rt2860_rf2850[i].chan;
			ic->ic_sup_channels[chan].ich_freq =
			    ieee80211_ieee2mhz(chan, IEEE80211_CHAN_5GHZ);
			ic->ic_sup_channels[chan].ich_flags =
			    IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM;
#ifdef CONFIG_11N
			ic->ic_sup_channels[chan].ich_flags |=
			    IEEE80211_CHAN_HT20;
#endif
		}
	}

	/* set supported .11b and .11g rates */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = rt2870_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = rt2870_rateset_11g;

	/* set supported .11b and .11g channels (1 through 14) */
	DPRINTF(0, (CE_CONT, "!%s: ic_sup_channels:%p",
	    dp->name, &ic->ic_sup_channels[0]));
	for (i = 1; i <= 14; i++) {
		ic->ic_sup_channels[i].ich_freq =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_sup_channels[i].ich_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
#ifdef CONFIG_PASSIVE
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_PASSIVE;
#endif
#ifdef CONFIG_11N
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT20;
#endif
	}
#ifdef CONFIG_11N
	/* for channels in 5GHz */
	if (sc->rf_rev == RT2860_RF_2750 || sc->rf_rev == RT2860_RF_2850) {
		int	minchan;
		int	maxchan;

		minchan = rt2860_rf2850[0].chan;
		maxchan = rt2860_rf2850[N(rt2860_rf2850) - 1].chan;

		for (i = minchan; i <= maxchan; i++) {
			if ((ic->ic_sup_channels[i].ich_flags &
			    IEEE80211_CHAN_HT20) == 0) {
				/* HT is not supported for the channel */
				continue;
			}
#ifdef CONFIG_BW40
			if (i <= maxchan - 5) {
				ic->ic_sup_channels[i].ich_flags
				    |= IEEE80211_CHAN_HT40U;
			}
			if (i >= minchan + 5) {
				ic->ic_sup_channels[i].ich_flags
				    |= IEEE80211_CHAN_HT40D;
			}
#endif
		}
	}

#ifdef CONFIG_BW40
	/* for channels in 2GHz */
	for (i = 1; i <= 14 - 5; i++) {
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40U;
	}
	for (i = 1 + 5; i <= 14; i++) {
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40D;
	}
#endif
#endif
	ic->ic_maxrssi = 127;
	ic->ic_set_shortslot = NULL;	/* not implemented */

	ic->ic_def_txkey = 0;

#ifndef CONFIG_DATAPAD
	ic->ic_flags = IEEE80211_F_DATAPAD;
#endif
	rt2870_eeprom_dump(dp, 64);
	dp->rx_buf_len = 12*2048;

	return (ret);

usberr:
	cmn_err(CE_WARN, "!%s: %s: usb error detected (%d)",
	    dp->name, __func__, err);
	return (USB_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
static int
rt2870attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_iblock_cookie_t	c;
	int			ret;
	int			revid;
	int			unit;
	int			vid;
	int			pid;
	struct chip_info	*p;
	int			len;
	const char		*drv_name;
	struct uwgem_dev	*dp;
	void			*base;
	struct uwgem_conf	*ugcp;
	struct rt2870_softc	*sc;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called, cmd:%d",
	    drv_name, unit, __func__, cmd));

	if (cmd == DDI_ATTACH) {
		/*
		 * Check if the chip is supported.
		 */
		vid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-vendor-id", -1);
		pid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-product-id", -1);
		revid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-revision-id", -1);

		/*
		 * construct uwgem configration
		 */
		ugcp = kmem_zalloc(sizeof (*ugcp), KM_SLEEP);

		/* name */
		/*
		 * XXX - softmac requires that ppa is the instance number
		 * of the device, otherwise it hangs in seaching the device.
		 */
		sprintf(ugcp->uwgc_name, "%s%d", drv_name, unit);
		ugcp->uwgc_ppa = unit;

		ugcp->uwgc_ifnum = 0;
		ugcp->uwgc_alt = 0;

		ugcp->uwgc_tx_list_max = 64;

		ugcp->uwgc_rx_header_len = 0;
		ugcp->uwgc_rx_list_max = 8;

		/* time out parameters */
		ugcp->uwgc_tx_timeout = UWGEM_TX_TIMEOUT;
		ugcp->uwgc_tx_timeout_interval = UWGEM_TX_TIMEOUT_INTERVAL;

		/* I/O methods */

		/* mac operation */
		ugcp->uwgc_attach_chip = &rt2870_attach_chip;
		ugcp->uwgc_reset_chip = &rt2870_reset_chip;
		ugcp->uwgc_init_chip = &rt2870_init_chip;
		ugcp->uwgc_start_chip = &rt2870_start_chip;
		ugcp->uwgc_stop_chip = &rt2870_stop_chip;

		ugcp->uwgc_set_rx_filter = &rt2870_set_rx_filter;
		ugcp->uwgc_get_stats = &rt2870_get_stats;
		ugcp->uwgc_interrupt = NULL;

		/* packet operation */
		ugcp->uwgc_tx_make_packet = &rt2870_tx_make_packet;
		ugcp->uwgc_rx_make_packet = &rt2870_rx_make_packet;

		/* newstate */
		ugcp->uwgc_newstate = &rt2870_newstate;

		sc = kmem_zalloc(sizeof (struct rt2870_softc), KM_SLEEP);

		dp = uwgem_do_attach(dip,
		    ugcp, sc, sizeof (struct rt2870_softc));

		kmem_free(ugcp, sizeof (*ugcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

err_free_mem:
		kmem_free(sc, sizeof (struct rt2870_softc));
err_close_pipe:
err:
		return (DDI_FAILURE);
	}

	if (cmd == DDI_RESUME) {
		dp = UWGEM_GET_DEV(dip);
		sc = dp->private;
		sc->sc_flags &= ~RT2870_FWLOADED;
		sc->initialized = B_FALSE;
		return (uwgem_resume(dip));
	}

	return (DDI_FAILURE);
}

static int
rt2870detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	if (cmd == DDI_DETACH) {
		ret = uwgem_do_detach(dip);
		if (ret != DDI_SUCCESS) {
			return (DDI_FAILURE);
		}
		return (DDI_SUCCESS);
	}
	if (cmd == DDI_SUSPEND) {
		return (uwgem_suspend(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
UWGEM_STREAM_OPS(rt2870_ops, rt2870attach, rt2870detach);

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&rt2870_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!rt2870: _init: called"));

	status = uwgem_mod_init(&rt2870_ops, "run");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		uwgem_mod_fini(&rt2870_ops);
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

	DPRINTF(2, (CE_CONT, "!rt2870: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		uwgem_mod_fini(&rt2870_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
