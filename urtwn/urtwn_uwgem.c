/* from OpenBSD if_urtwn.c and if_urtwnreg.h */

/*	$OpenBSD: if_urtwn.c,v 1.12 2010/12/31 20:50:14 damien Exp $	*/

/*-
 * Copyright (c) 2010 Damien Bergamini <damien.bergamini@free.fr>
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
 * Driver for Realtek RTL8188CUS/RTL8188RU/RTL8192CU.
 */
#pragma ident "@(#)urtwn_uwgem.c	1.2 16/05/22"

#if 0
TODO:
 802.11n a-mpdu
#endif
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
#include <sys/mac_wifi.h>
#include <sys/net80211.h>
#include <sys/net80211_proto.h>

#include "uwgem.h"

#undef SLOT
#undef __packed
#define	__packed	__attribute((packed))
#define	BIT(n)	(1U << (n))
#define	N(a)	(sizeof (a) / sizeof ((a)[0]))
#define	NITEMS(x)	N(x)

#include "if_urtwnreg.h"
#include "compat.h"

char	ident[] = "RTL8188CU/8192CU v" VERSION;

/*
 * Useful macros
 */
#ifndef ABS
#define	ABS(x)	((x) < 0 ? -(x) : (x))
#endif

#define	htole64(x)	LE_64(x)
#define	htole32(x)	LE_32(x)
#define	htole16(x)	LE_16(x)
#define	letoh64(x)	LE_64(x)
#define	letoh32(x)	LE_32(x)
#define	letoh16(x)	LE_16(x)

#define	IS_92C_SERIAL(x)	(((x) & URTWN_CHIP_92C) != 0)

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
static int urtwn_debug = DEBUG_LEVEL;
#define	IOTRACE	10	/* was -1 */
#define	DPRINTF(n, args)	if (urtwn_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

#ifdef DEBUG
#define	RUN_DEBUG \
	urtwn_debug_printf
#else
#define	RUN_DEBUG
#endif

/*
 * Our configration
 */

#define	IQK_ADDA_REG_NUM	16
#define	IQK_MAC_REG_NUM	4
#define	IQK_BB_REG_NUM	9
#define	rFPGA0_XA_HSSIParameter1	0x820	/* RF 3 wire register */

/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))

static char *rtw_initmac = NULL;

/*
 * RX/TX buffer size
 */

/*
 * Local definitions
 */
struct urtwn_dev {
	boolean_t initialized;

	int	ac2idx[WME_NUM_AC];

	u_int	sc_flags;
#define URTWN_FLAG_CCK_HIPWR	0x01

	u_int	chip;
#define URTWN_CHIP_92C		0x01
#define URTWN_CHIP_92C_1T2R	0x02
#define URTWN_CHIP_UMC		0x04
#define URTWN_CHIP_UMC_A_CUT	0x08

	uint8_t	board_type;
	uint8_t	regulatory;
	uint8_t	pa_setting;
        uint8_t	ExternalPA;
	int	avg_pwdb;
	int	thcal_state;
	int	thcal_lctemp;
	int	ntxchains;
	int	nrxchains;
	int	ledlink;

	int	fwcur;
	struct r92c_rom	rom;

	uint32_t	rf_chnlbw[R92C_MAX_CHAINS];

	int	chipversion;
	int	rfintfs;
	int	lbkmode;
	int	hci;
	int	network_mode;
	int	channel;
	int	wireless_mode;
	int	vrtl_carrier_sense;
	int	vcs_type;
	int	frag_thresh;
	int	preamble;
	int	scan_mode;	/* active, passive */
	int	adhoc_tx_pwr;
	int	soft_ap;
	int	power_mgnt;

	int	radio_enable;
	int	long_retry_lmt;
	int	short_retry_lmt;
	int	busy_thresh;
	int	ack_policy;
	int	mp_mode;
	int	software_encrypt;
	int	software_dencrypt;
	int	wmm_enable;

	int	uapsd_enable;
	int	uapsd_max_sp;
	int	uapsd_acbk_en;
	int	uapsd_acbe_en;
	int	uapsd_acvi_en;
	int	uapsd_acvo_en;

#ifdef CONFIG_11N
	int	ht_enable;
	int	cbw40_enable;
	int	ampdu_enable;
#endif

	int	rf_config;
	int	low_power;
	int	wifi_spec;
	int	channel_plan;

	int	bAcceptAddaReq;
	int	antdiv_cfg;
#ifdef CONFIG_AUTOSUSPEND
	int	usbss_enable;
#else
	int	usbss_enable;
#endif
	int	hwpdn_mode;	/* 2: by eFuse config */
	int	hwpwrp_detected;	/* 0: disable, 1:enable */

	int	ishighspeed;
	int	pktsize;
	int	fwready;

	uint32_t	ADDA_backup[IQK_ADDA_REG_NUM];
	uint32_t	IQK_MAC_backup[IQK_MAC_REG_NUM];
	uint32_t	IQK_BB_backup[IQK_BB_REG_NUM];
	boolean_t	bRfPiEnable;
	uint32_t	RegE94;
	uint32_t	RegE9C;
	uint32_t	RegEB4;
	uint32_t	RegEBC;
	boolean_t	iqk_initialized;

        uint32_t	RfRegChnlVal[2];
};

/*
 * Read only variables
 */
/* Firmware */
static const uint8_t urtwn_rtl8192cfwU_fw[] = {
#include "urtwn_rtl8192cfwU.hex"
};

static const uint8_t urtwn_rtl8192cfwT_fw[] = {
#include "urtwn_rtl8192cfwT.hex"
};

static const struct ieee80211_rateset urtwn_rateset_11b =
	{ 4, { 2, 4, 11, 22 } };

static const struct ieee80211_rateset urtwn_rateset_11g =
	{ 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };

struct ether_addr urtwn_broadcastaddr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* private */
static void urtwn_temp_calib(struct uwgem_dev *dp);

/* public 802.11 state operation */
static int urtwn_newstate(struct uwgem_dev *, enum ieee80211_state, int );

/* public nic operations */
static int urtwn_reset_chip(struct uwgem_dev *, uint_t);
static int urtwn_init_chip(struct uwgem_dev *);
static int urtwn_start_chip(struct uwgem_dev *);
static int urtwn_stop_chip(struct uwgem_dev *);
static int urtwn_set_rx_filter(struct uwgem_dev *);
static int urtwn_get_stats(struct uwgem_dev *);

/* public packet operations */
static mblk_t *urtwn_tx_make_packet(struct uwgem_dev *, mblk_t *,
    uint_t, struct ieee80211_node *);
static mblk_t *urtwn_rx_make_packet(struct uwgem_dev *, mblk_t *);

/* private ieee80211 functions */
int ieee80211_fix_rate(ieee80211_node_t *, struct ieee80211_rateset *, int);
void ieee80211_setbasicrates(struct ieee80211_rateset *,
    enum ieee80211_phymode);

/* =============================================================== */
/*
 * I/O functions
 */
/* =============================================================== */
/* Aliases. */
#define	urtwn_bb_write	urtwn_write_4
#define urtwn_bb_read	urtwn_read_4
#define	urtwn_bb_bset(dp, reg, val, mask, shift) \
	urten_bb_write((dp), (reg),	\
	    (urten_bb_read((dp), (reg)) & ~(((mask) << (shift)))) \
	     | (((val) & (mask)) << (shift)))

#define	OUT(dp, req, val, ix, len, buf, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* value */    (buf),	\
	/* size */     (len))) != USB_SUCCESS) goto label

#define	IN(dp, req, val, ix, len, buf, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* valuep */   (buf),	\
	/* size */     (len))) != USB_SUCCESS) goto label

/* =============================================================== */
static int
urtwn_write_region_1(struct uwgem_dev *dp, uint16_t addr, uint8_t *buf,
    int len)
{
	int	err;

	OUT(dp, R92C_REQ_REGS, addr, 0, len, buf, &err, usberr);
	return (err);
usberr:
	cmn_err(CE_CONT, "!%s: %s: error: addr:0x%x len:0x%x",
	    dp->name, __func__, addr, len);
	return (err);
}

static void
urtwn_write_1(struct uwgem_dev *dp, uint16_t addr, uint8_t val)
{
	DPRINTF(IOTRACE, (CE_CONT, "!%s: W8 %x %x", dp->name, addr, val));
	urtwn_write_region_1(dp, addr, &val, 1);
}

static void
urtwn_write_2(struct uwgem_dev *dp, uint16_t addr, uint16_t val)
{
	uint8_t	buf[2];

	buf[0] = (uint8_t)val;
	buf[1] = (uint8_t)(val >> 8);
	DPRINTF(IOTRACE, (CE_CONT, "!%s: W16 %x %x", dp->name, addr, val));
	urtwn_write_region_1(dp, addr, buf, 2);
}

static void
urtwn_write_4(struct uwgem_dev *dp, uint16_t addr, uint32_t val)
{
	uint8_t	buf[4];

	buf[0] = (uint8_t)val;
	buf[1] = (uint8_t)(val >> 8);
	buf[2] = (uint8_t)(val >> 16);
	buf[3] = (uint8_t)(val >> 24);
	DPRINTF(IOTRACE, (CE_CONT, "%s: W32 %x %x", dp->name, addr, val));
	urtwn_write_region_1(dp, addr, buf, 4);
}

static int
urtwn_write_N(struct uwgem_dev *dp, uint16_t addr, uint8_t *buf,
    int len)
{
	DPRINTF(IOTRACE, (CE_CONT, "%s: WN %x %x", dp->name, addr, len));
	return (urtwn_write_region_1(dp, addr, buf, len));
}

static int
urtwn_read_region_1(struct uwgem_dev *dp, uint16_t addr, uint8_t *buf,
    int len)
{
	int	err;

	IN(dp, R92C_REQ_REGS, addr, 0, len, buf, &err, usberr);
	return (err);
usberr:
	cmn_err(CE_CONT, "!%s: %s: error: addr:0x%x len:0x%x",
	    dp->name, __func__, addr, len);
	return (err);
}

static uint8_t
urtwn_read_1(struct uwgem_dev *dp, uint16_t addr)
{
	uint8_t val;

	if (urtwn_read_region_1(dp, addr, &val, 1) != USB_SUCCESS) {
		return (0xff);
	}
	DPRINTF(IOTRACE, (CE_CONT, "%s: R8 %x %x", dp->name, addr, val));
	return (val);
}

static uint16_t
urtwn_read_2(struct uwgem_dev *dp, uint16_t addr)
{
	uint8_t val[2];

	if (urtwn_read_region_1(dp, addr, val, 2) != USB_SUCCESS) {
		return (0xffff);
	}
	DPRINTF(IOTRACE, (CE_CONT, "%s: R16 %x %x", dp->name,
	    addr, val[1] << 8 | val[0]));
	return (val[1] << 8 | val[0]);
}

static uint32_t
urtwn_read_4(struct uwgem_dev *dp, uint16_t addr)
{
	uint8_t val[4];

	if (urtwn_read_region_1(dp, addr, val, 4) != USB_SUCCESS) {
		return (0xffffffff);
	}
	DPRINTF(IOTRACE, (CE_CONT, "%s: R32 %x %x", dp->name,
	    addr, (val[3] << 24 | val[2] << 16 | val[1] << 8 | val[0])));
	return (val[3] << 24 | val[2] << 16 | val[1] << 8 | val[0]);
}

static int
urtwn_fw_cmd(struct uwgem_dev *dp, uint8_t id, const void *buf, int len)
{
	struct urtwn_dev	*lp = dp->private;
	struct r92c_fw_cmd	cmd;
	int	ntries;
	uint8_t	*cp;

	/* Wait for current FW box to be empty. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (!(urtwn_read_1(dp, R92C_HMETFR) & (1 << lp->fwcur))) {
			break;
		}
		drv_usecwait(1);
	}
	if (ntries == 100) {
		cmn_err(CE_WARN, "!%s: could not send firmware command %d",
		    dp->name, id);
		return (USB_FAILURE);
	}
	memset(&cmd, 0, sizeof (cmd));
	ASSERT(len <= sizeof (cmd.msg));
	memcpy(cmd.msg, buf, len);

	/* Write the first word last since that will trigger the FW. */
	cp = (uint8_t *)&cmd;
	if (len >= 4) {
		cmd.id = id | R92C_CMD_FLAG_EXT;
		urtwn_write_2(dp, R92C_HMEBOX_EXT(lp->fwcur),
		    cp[2] << 8 | cp[1]);
		urtwn_write_4(dp, R92C_HMEBOX(lp->fwcur),
		   cp[5] << 24 | cp[4] << 16 | cp[3] << 8 | cp[0]);
	} else {
		cmd.id = id;
		urtwn_write_4(dp, R92C_HMEBOX(lp->fwcur),
		   cp[3] << 24 | cp[2] << 16 | cp[1] << 8 | cp[0]);
	}

	lp->fwcur = (lp->fwcur + 1) % R92C_H2C_NBOX;
	return (USB_SUCCESS);
}

static void
urtwn_rf_write(struct uwgem_dev *dp, int chain, uint8_t addr, uint32_t val)
{
	urtwn_bb_write(dp, R92C_LSSI_PARAM(chain),
	    SM(R92C_LSSI_PARAM_ADDR, addr) |
	    SM(R92C_LSSI_PARAM_DATA, val));
}

static uint32_t
urtwn_rf_read(struct uwgem_dev *dp, int chain, uint8_t addr)
{
	uint32_t reg[R92C_MAX_CHAINS], val;

	reg[0] = urtwn_bb_read(dp, R92C_HSSI_PARAM2(0));
	if (chain != 0) {
		reg[chain] = urtwn_bb_read(dp, R92C_HSSI_PARAM2(chain));
	}

	urtwn_bb_write(dp, R92C_HSSI_PARAM2(0),
	    reg[0] & ~R92C_HSSI_PARAM2_READ_EDGE);
	drv_usecwait(1000);

	urtwn_bb_write(dp, R92C_HSSI_PARAM2(chain),
	    RW(reg[chain], R92C_HSSI_PARAM2_READ_ADDR, addr) |
	    R92C_HSSI_PARAM2_READ_EDGE);
	drv_usecwait(1000);

	urtwn_bb_write(dp, R92C_HSSI_PARAM2(0),
	    reg[0] | R92C_HSSI_PARAM2_READ_EDGE);
	drv_usecwait(1000);

	if (urtwn_bb_read(dp, R92C_HSSI_PARAM1(chain)) & R92C_HSSI_PARAM1_PI) {
		val = urtwn_bb_read(dp, R92C_HSPI_READBACK(chain));
	} else {
		val = urtwn_bb_read(dp, R92C_LSSI_READBACK(chain));
	}
	return (MS(val, R92C_LSSI_READBACK_DATA));
}

static void
urtwn_cam_write(struct uwgem_dev *dp, uint32_t addr, uint32_t data)
{
	urtwn_write_4(dp, R92C_CAMWRITE, data);
	urtwn_write_4(dp, R92C_CAMCMD,
	    R92C_CAMCMD_POLLING | R92C_CAMCMD_WRITE |
	    SM(R92C_CAMCMD_ADDR, addr));
}

static int
urtwn_llt_write(struct uwgem_dev *dp, uint32_t addr, uint32_t data)
{
	int	ntries;

	urtwn_write_4(dp, R92C_LLT_INIT,
	    SM(R92C_LLT_INIT_OP, R92C_LLT_INIT_OP_WRITE) |
	    SM(R92C_LLT_INIT_ADDR, addr) |
	    SM(R92C_LLT_INIT_DATA, data));

	/* Wait for write operation to complete. */
	for (ntries = 0; ; ntries++) {
		if (MS(urtwn_read_4(dp, R92C_LLT_INIT),
		    R92C_LLT_INIT_OP) == R92C_LLT_INIT_OP_NO_ACTIVE) {
			/* done */
			break;
		}
		if (ntries >= 20) {
			return (USB_FAILURE);
		}
	}
	return (USB_SUCCESS);
}

static uint8_t
urtwn_efuse_read_1(struct uwgem_dev *dp, uint16_t addr)
{
	uint32_t reg;
	int ntries;
	uint8_t	ret = 0xff;
#ifdef DEBUG_LEVEL
	int	saved_debug_level;

	saved_debug_level = urtwn_debug;
	urtwn_debug = -1000;
#endif
	reg = urtwn_read_4(dp, R92C_EFUSE_CTRL);
	reg = RW(reg, R92C_EFUSE_CTRL_ADDR, addr);
	reg &= ~R92C_EFUSE_CTRL_VALID;
	urtwn_write_4(dp, R92C_EFUSE_CTRL, reg);

	/* Wait for read operation to complete. */
	for (ntries = 0; ntries < 100; ntries++) {
		reg = urtwn_read_4(dp, R92C_EFUSE_CTRL);
		if (reg & R92C_EFUSE_CTRL_VALID) {
			/* done */
			ret = MS(reg, R92C_EFUSE_CTRL_DATA);
			goto x;
		}
		drv_usecwait(5);
	}
	cmn_err(CE_CONT, "!%s: could not read efuse byte at address 0x%x",
	    dp->name, addr);
x:
#ifdef DEBUG_LEVEL
	urtwn_debug = saved_debug_level;
#endif
	return (ret);
}

static void
urtwn_efuse_read(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	uint8_t	*rom = (uint8_t *)&lp->rom;
	uint16_t	addr = 0;
	uint32_t	reg;
	uint8_t	off, msk;
	int	i;

	reg = urtwn_read_2(dp, R92C_SYS_ISO_CTRL);
	if (!(reg & R92C_SYS_ISO_CTRL_PWC_EV12V)) {
		urtwn_write_2(dp, R92C_SYS_ISO_CTRL,
		    reg | R92C_SYS_ISO_CTRL_PWC_EV12V);
	}
	reg = urtwn_read_2(dp, R92C_SYS_FUNC_EN);
	if (!(reg & R92C_SYS_FUNC_EN_ELDR)) {
		urtwn_write_2(dp, R92C_SYS_FUNC_EN,
		    reg | R92C_SYS_FUNC_EN_ELDR);
	}
	reg = urtwn_read_2(dp, R92C_SYS_CLKR);
	if ((reg & (R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M)) !=
	    (R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M)) {
		urtwn_write_2(dp, R92C_SYS_CLKR,
		    reg | R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M);
	}
	memset(&lp->rom, 0xff, sizeof (lp->rom));
	while (addr < 512) {
		reg = urtwn_efuse_read_1(dp, addr);
		if (reg == 0xff)
			break;
		addr++;
		off = reg >> 4;
		msk = reg & 0xf;
		for (i = 0; i < 4; i++) {
			if (msk & (1U << i)) {
				continue;
			}
			rom[off * 8 + i * 2 + 0] =
			    urtwn_efuse_read_1(dp, addr);
			addr++;
			rom[off * 8 + i * 2 + 1] =
			    urtwn_efuse_read_1(dp, addr);
			addr++;
		}
	}
#ifdef DEBUG_LEVEL
	if (urtwn_debug >= 2) {
		/* Dump ROM content. */
		cmn_err(CE_CONT, "!%s: %s: dump rom", dp->name, __func__);
		for (i = 0; i < sizeof (lp->rom); i += 8) {
			cmn_err(CE_CONT,
			    "!%02x: %02x %02x %02x %02x %02x %02x %02x %02x",
			    i,
			    rom[i + 0], rom[i + 1], rom[i + 2], rom[i + 3],
			    rom[i + 4], rom[i + 5], rom[i + 6], rom[i + 7]);
		}
	}
#endif
}

static int
urtwn_read_chipid(struct uwgem_dev *dp)
{
	struct urtwn_dev	*lp = dp->private;
	uint32_t	reg;

	lp->chip = 0;
	reg = urtwn_read_4(dp, R92C_SYS_CFG);
	DPRINTF(10, (CE_CONT, "!%s: %s: SYS_CFG:%x", dp->name, __func__, reg));

	if (reg & R92C_SYS_CFG_TRP_VAUX_EN) {
		/* test chip, not supported */
		return (USB_FAILURE);
	}
	if (reg & R92C_SYS_CFG_TYPE_92C) {
		lp->chip |= URTWN_CHIP_92C;
		/* Check if it is a castrated 8192C. */
		if (MS(urtwn_read_4(dp, R92C_HPON_FSM),
		    R92C_HPON_FSM_CHIP_BONDING_ID) ==
		    R92C_HPON_FSM_CHIP_BONDING_ID_92C_1T2R) {
			lp->chip |= URTWN_CHIP_92C_1T2R;
		}
	}
	if (reg & R92C_SYS_CFG_VENDOR_UMC) {
		lp->chip |= URTWN_CHIP_UMC;
		if (MS(reg, R92C_SYS_CFG_CHIP_VER_RTL) == 0) {
			lp->chip |= URTWN_CHIP_UMC_A_CUT;
		}
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: SYS_CFG:%x, chip:%x",
	    dp->name, __func__, reg, lp->chip));
	return (USB_SUCCESS);
}

static void
urtwn_dump_rom(struct uwgem_dev *dp, struct r92c_rom *rp)
{
	cmn_err(CE_CONT, "!id 0x%04x, dbg_sel 0x%x, vid 0x%x, pid 0x%x",
	    rp->id, rp->dbg_sel, rp->vid, rp->pid);

	cmn_err(CE_CONT, "!usb_opt 0x%x, ep_setting 0x%x, usb_phy 0x%x",
	    rp->usb_opt, rp->ep_setting, rp->usb_phy);

	cmn_err(CE_CONT, "!macaddr %02x:%02x:%02x:%02x:%02x:%02x",
	    rp->macaddr[0], rp->macaddr[1],
	    rp->macaddr[2], rp->macaddr[3],
	    rp->macaddr[4], rp->macaddr[5]);

	cmn_err(CE_CONT, "!string %s, subcustomer_id 0x%x",
	    rp->string, rp->subcustomer_id);

	cmn_err(CE_CONT, "!cck_tx_pwr c0: %d %d %d, c1: %d %d %d",
	    rp->cck_tx_pwr[0][0], rp->cck_tx_pwr[0][1], rp->cck_tx_pwr[0][2],
	    rp->cck_tx_pwr[1][0], rp->cck_tx_pwr[1][1], rp->cck_tx_pwr[1][2]);

	cmn_err(CE_CONT, "!ht40_1s_tx_pwr c0 %d %d %d, c1 %d %d %d",
	    rp->ht40_1s_tx_pwr[0][0], rp->ht40_1s_tx_pwr[0][1],
	    rp->ht40_1s_tx_pwr[0][2],
	    rp->ht40_1s_tx_pwr[1][0], rp->ht40_1s_tx_pwr[1][1],
	    rp->ht40_1s_tx_pwr[1][2]);

	cmn_err(CE_CONT, "!ht40_2s_tx_pwr_diff c0: %d %d %d, c1: %d %d %d",
	    rp->ht40_2s_tx_pwr_diff[0] & 0xf, rp->ht40_2s_tx_pwr_diff[1] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[2] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[0] >> 4, rp->ht40_2s_tx_pwr_diff[1] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[2] >> 4);


	cmn_err(CE_CONT, "!ht20_tx_pwr_diff c0: %d %d %d, c1: %d %d %d",
	    rp->ht20_tx_pwr_diff[0] & 0xf, rp->ht20_tx_pwr_diff[1] & 0xf,
	    rp->ht20_tx_pwr_diff[2] & 0xf,
	    rp->ht20_tx_pwr_diff[0] >> 4, rp->ht20_tx_pwr_diff[1] >> 4,
	    rp->ht20_tx_pwr_diff[2] >> 4);

	cmn_err(CE_CONT, "!ofdm_tx_pwr_diff c0: %d %d %d, c1: %d %d %d",
	    rp->ofdm_tx_pwr_diff[0] & 0xf, rp->ofdm_tx_pwr_diff[1] & 0xf,
	    rp->ofdm_tx_pwr_diff[2] & 0xf,
	    rp->ofdm_tx_pwr_diff[0] >> 4, rp->ofdm_tx_pwr_diff[1] >> 4,
	    rp->ofdm_tx_pwr_diff[2] >> 4);

	cmn_err(CE_CONT, "!ht40_max_pwr_offset c0: %d %d %d, c1: %d %d %d",
	    rp->ht40_max_pwr[0] & 0xf, rp->ht40_max_pwr[1] & 0xf,
	    rp->ht40_max_pwr[2] & 0xf,
	    rp->ht40_max_pwr[0] >> 4, rp->ht40_max_pwr[1] >> 4,
	    rp->ht40_max_pwr[2] >> 4);

	cmn_err(CE_CONT, "!ht20_max_pwr_offset c0: %d %d %d, c1: %d %d %d",
	    rp->ht20_max_pwr[0] & 0xf, rp->ht20_max_pwr[1] & 0xf,
	    rp->ht20_max_pwr[2] & 0xf,
	    rp->ht20_max_pwr[0] >> 4, rp->ht20_max_pwr[1] >> 4,
	    rp->ht20_max_pwr[2] >> 4);

	cmn_err(CE_CONT, "!xtal_calib %d, tssi %d %d, thermal %d",
	    rp->xtal_calib, rp->tssi[0], rp->tssi[1], rp->thermal_meter);

	cmn_err(CE_CONT,
	    "!rf_opt1 0x%x, rf_opt2 0x%x, rf_opt3 0x%x, rf_opt4 0x%x",
	    rp->rf_opt1, rp->rf_opt2, rp->rf_opt3, rp->rf_opt4);

	cmn_err(CE_CONT, "!channnel_plan %d, version %d customer_id 0x%x",
	    rp->channel_plan, rp->version, rp->curstomer_id);
}

static void
urtwn_read_rom(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	struct r92c_rom *rom = &lp->rom;

	/* Read full ROM image. */
	urtwn_efuse_read(dp);

	urtwn_dump_rom(dp, rom);

	/* XXX - Weird but this is what the vendor driver does. */
	lp->pa_setting = urtwn_efuse_read_1(dp, 0x1fa);
	DPRINTF(0, (CE_CONT, "!%s: %s: PA setting=0x%x",
	    dp->name, __func__, lp->pa_setting));

	lp->board_type = MS(rom->rf_opt1, R92C_ROM_RF1_BOARD_TYPE);
	lp->regulatory = MS(rom->rf_opt1, R92C_ROM_RF1_REGULATORY);

	DPRINTF(0, (CE_CONT, "!%s: %s: regulatory type=%d, board type:%d",
	    dp->name, __func__, lp->regulatory, lp->board_type));

	IEEE80211_ADDR_COPY(dp->dev_addr, rom->macaddr);
}

/*
 * Initialize rate adaptation in firmware.
 */
static int
urtwn_ra_init(struct uwgem_dev *dp)
{
	static const uint8_t map[] =
	    { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 };
	struct ieee80211com *ic = &dp->ic;
	struct ieee80211_node *ni = ic->ic_bss;
	struct ieee80211_rateset *rs = &ni->in_rates;
	struct r92c_fw_cmd_macid_cfg cmd;
	uint32_t	rates, basicrates;
	uint8_t		raid;
	int		maxrate, maxbasicrate, error, i, j;
	boolean_t	sgi = B_FALSE;
	uint_t	uval;

	/* Get normal and basic rates mask. */
	rates = basicrates = 0;
	maxrate = maxbasicrate = 0;
	for (i = 0; i < rs->ir_nrates; i++) {
		/* Convert 802.11 rate to HW rate index. */
		for (j = 0; j < NITEMS(map); j++) {
			if ((rs->ir_rates[i] & IEEE80211_RATE_VAL) == map[j]) {
				goto rate_found;
			}
		}
		/* Unknown rate, skip. */
		continue;
rate_found:
		rates |= 1U << j;
		maxrate = max(maxrate, j);

		if (rs->ir_rates[i] & IEEE80211_RATE_BASIC) {
			basicrates |= 1U << j;
			maxbasicrate = max(maxbasicrate, j);
		}
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: curmode:%d, assoc_id:0x%x",
	    dp->name, __func__, ic->ic_curmode, ni->in_associd));
#ifdef CONFIG_11N
	if (ic->ic_curmode == IEEE80211_MODE_11NG) {
		raid = R92C_RAID_11GN;
		for (i = 0; i < ni->in_htrates.rs_nrates; i++) {
			j = ni->in_htrates.rs_rates[i] + 12;
			rates |= 1U << j;
			maxrate = max(maxrate, j);
		}
#ifdef CONFIG_SHORTGI
		if (ni->in_chw == 40 && (ic->ic_htcaps & ni->in_htcap
		    & IEEE80211_HTCAP_SHORTGI40)) {
			sgi = B_TRUE;
		} else if (ni->in_chw != 40 && (ic->ic_htcaps & ni->in_htcap
		    & IEEE80211_HTCAP_SHORTGI20)) {
			sgi = B_TRUE;
		}
#endif
	} else 
#endif
	if (ic->ic_curmode == IEEE80211_MODE_11B) {
		raid = R92C_RAID_11B;
	} else {
		raid = R92C_RAID_11BG;
	}
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: "
	    "raid=0x%x rates=0x%08x, basicrates=0x%08x, maxrate:%d, sgi:%d",
	    dp->name, __func__,
	    raid, rates, basicrates, maxrate, sgi));

	if (basicrates == 0) {
		basicrates |= 1;	/* add 1Mbps */
	}

	/* Set rates mask for group addressed frames. */
	cmd.macid = URTWN_MACID_BC | URTWN_MACID_VALID;
	if (sgi) {
		cmd.macid |= BIT(5);
	}
	uval = (raid << 28 | basicrates);
	cmd.mask[0] = uval;
	cmd.mask[1] = uval >> 8;
	cmd.mask[2] = uval >> 16;
	cmd.mask[3] = uval >> 24;
	error = urtwn_fw_cmd(dp, R92C_CMD_MACID_CONFIG, &cmd, sizeof (cmd));
	if (error != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: could not add broadcast station",
		    dp->name, __func__);
		return (error);
	}
	/* Set initial MRR rate. */
	DPRINTF(0, (CE_CONT, "!%s: %s: maxbasicrate=%d\n",
	    dp->name, __func__, maxbasicrate));
	urtwn_write_1(dp, R92C_INIDATA_RATE_SEL(URTWN_MACID_BC),
	    maxbasicrate | (sgi ? BIT(6) : 0));

	/* Set rates mask for unicast frames. */
	cmd.macid = URTWN_MACID_BSS | URTWN_MACID_VALID;
	if (sgi) {
		cmd.macid |= BIT(5);
	}
	uval = raid << 28 | rates;
	cmd.mask[0] = uval;
	cmd.mask[1] = uval >> 8;
	cmd.mask[2] = uval >> 16;
	cmd.mask[3] = uval >> 24;
	error = urtwn_fw_cmd(dp, R92C_CMD_MACID_CONFIG, &cmd, sizeof (cmd));
	if (error != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: could not add BSS station",
		    dp->name, __func__);
		return (error);
	}

	/* Set initial MRR rate. */
	DPRINTF(0, (CE_CONT, "!%s: %s: maxrate=%d",
	    dp->name, __func__, maxrate));
	urtwn_write_1(dp, R92C_INIDATA_RATE_SEL(URTWN_MACID_BSS),
	    maxrate | (sgi ? BIT(6) : 0));

	/* Indicate highest supported rate. */
	if (ni->in_flags & IEEE80211_NODE_HT) {
		ni->in_txrate = rs->ir_nrates - 1;
		dp->txrate = ni->in_htrates.rs_nrates - 1;
	} else {
		ni->in_txrate = rs->ir_nrates - 1;
		dp->txrate = rs->ir_nrates - 1;
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: initial rate=%d,"
	    " in_flags:0x%x, in_htcap:0x%x, in_htparam:0x%x,"
	    " ctrlchan:%d, 2ndchan:%d, htopmode:0x%x,"
	    " reqcw:%d, chw:%d flags:%x htstbc:%d",
	    dp->name, __func__,
	    (ni->in_flags & IEEE80211_NODE_HT)
	    ? ni->in_htrates.rs_rates[dp->txrate] 
	    : ni->in_rates.ir_rates[dp->txrate] & IEEE80211_RATE_VAL,
	    ni->in_flags, ni->in_htcap, ni->in_htparam,
	    ni->in_htctlchan, ni->in_ht2ndchan, ni->in_htopmode,
	    ni->in_reqcw, ni->in_chw, ni->in_chan ? ni->in_chan->ich_flags : 0,
	    ni->in_htstbc));
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: addr=%x:%x:%x:%x:%x:%x",
	    dp->name, __func__,
	    ni->in_macaddr[0], ni->in_macaddr[1], ni->in_macaddr[2],
	    ni->in_macaddr[3], ni->in_macaddr[4], ni->in_macaddr[5]));
#if DEBUG_LEVEL >= 0
	cmn_err(CE_CONT, "!b/g rateset: num:%d", ni->in_rates.ir_nrates);
	for (i = 0; i < ni->in_rates.ir_nrates; i++) {
		int	rate = ni->in_rates.ir_rates[i] & IEEE80211_RATE_VAL;
		cmn_err(CE_CONT, "!%d (%dMbps)", rate, rate / 2);
	}
	ieee80211_ht_announce(ic);

	if (ni->in_flags & IEEE80211_NODE_HT) {
		cmn_err(CE_CONT, "!htrateset: num:%d",
		    ni->in_htrates.rs_nrates);
		for (i = 0; i < ni->in_htrates.rs_nrates; i++) {
			int	mcs = ni->in_htrates.rs_rates[i];
			cmn_err(CE_CONT, "!%d (%dMbps)",
			    mcs, ieee80211_htrates[mcs] * ni->in_chw / 40);
		}
		ieee80211_ht_announce(ic);
	}
#endif
	return (USB_SUCCESS);
}

static uint_t
urtwn_get_netype(struct uwgem_dev *dp)
{
	uint_t	ret;
	struct ieee80211com	*ic = &dp->ic;

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		ret = R92C_CR_NETTYPE_INFRA;
		break;

	case IEEE80211_M_IBSS:
		ret = R92C_CR_NETTYPE_ADHOC;
		break;
	
	default:
		ret = R92C_CR_NETTYPE_NOLINK;
		break;
	}

	return (ret);
}

static void
urtwn_set_netype0_msr(struct uwgem_dev *dp, uint8_t type)
{
	uint8_t	val8;

	val8 = urtwn_read_1(dp, R92C_CR + 2) & 0x0c;
	urtwn_write_1(dp, R92C_CR + 2, val8 | type);
}

static void
urtwn_tsf_sync_enable(struct uwgem_dev *dp)
{
	struct ieee80211_node *ni = dp->ic.ic_bss;
	uint64_t	tsf;

	/* Enable TSF synchronization. */
	urtwn_write_1(dp, R92C_BCN_CTRL,
	    urtwn_read_1(dp, R92C_BCN_CTRL) & ~R92C_BCN_CTRL_DIS_TSF_UDT0);

	/* Correct TSF */
#ifdef notyet
	if (ic->ic_opmode == IEEE80211_M_IBSS ||
	    ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/* disable beacon */
	}
#endif
	urtwn_write_1(dp, R92C_BCN_CTRL,
	    urtwn_read_1(dp, R92C_BCN_CTRL) & ~R92C_BCN_CTRL_EN_BCN);

	/* Set initial TSF. */
#if 0
	memcpy(&tsf, ni->in_tstamp, 8);
#else
	tsf = ni->in_tstamp.tsf;
#endif
	tsf = letoh64(tsf);
#define	IEEE80211_DUR_TU	1024
	tsf = tsf - (tsf % (ni->in_intval * IEEE80211_DUR_TU));
	tsf -= IEEE80211_DUR_TU;
	urtwn_write_4(dp, R92C_TSFTR + 0, tsf);
	urtwn_write_4(dp, R92C_TSFTR + 4, tsf >> 32);

	urtwn_write_1(dp, R92C_BCN_CTRL,
	    urtwn_read_1(dp, R92C_BCN_CTRL) | R92C_BCN_CTRL_EN_BCN);
#ifdef notyet
	if (ic->ic_opmode == IEEE80211_M_IBSS ||
	    ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/* resume beacon */
	}
#endif
}

static void
urtwn_set_led(struct uwgem_dev *dp, int led, int on)
{
	struct urtwn_dev *lp = dp->private;
	uint8_t reg;

	if (led == URTWN_LED_LINK) {
		reg = urtwn_read_1(dp, R92C_LEDCFG0) & 0x70;
		if (!on) {
			reg |= R92C_LEDCFG0_DIS;
		}
		urtwn_write_1(dp, R92C_LEDCFG0, reg);
		lp->ledlink = on;	/* Save LED state. */
	}
}

/* ARGSUSED */
static void
urtwn_calib_to(struct uwgem_dev *dp, void *arg)
{
	struct ieee80211com	*ic = &dp->ic;
	struct urtwn_dev *lp = dp->private;
	struct r92c_fw_cmd_rssi cmd;

	if (ic->ic_state != IEEE80211_S_RUN) {
		return;
	}

	if (lp->avg_pwdb != -1) {
		/* Indicate Rx signal strength to FW for rate adaptation. */
		memset(&cmd, 0, sizeof (cmd));
		cmd.macid = 0;	/* BSS. */
		cmd.pwdb = lp->avg_pwdb;
		DPRINTF(3, (CE_CONT, "!%s: %s: sending RSSI command avg=%d",
		    dp->name, __func__, lp->avg_pwdb));
		urtwn_fw_cmd(dp, R92C_CMD_RSSI_SETTING, &cmd, sizeof (cmd));
	}

	/* Do temperature compensation. */
	urtwn_temp_calib(dp);
}

#if 0
static void
urtwn_updateedca(struct ieee80211com *ic)
{
	/* Do it in a process context. */
	urtwn_do_async(ic->ic_softc, urtwn_updateedca_cb, NULL, 0);
}

/* ARGSUSED */
static void
urtwn_updateedca_cb(struct uwgem_dev *dp, void *arg)
{
	static const uint16_t aci2reg[WME_NUM_AC] = {
		R92C_EDCA_BE_PARAM,
		R92C_EDCA_BK_PARAM,
		R92C_EDCA_VI_PARAM,
		R92C_EDCA_VO_PARAM
	};
	struct ieee80211com *ic = &dp->ic;
	struct ieee80211_edca_ac_params *ac;
	int s, aci, aifs, slottime;

	s = splnet();
	slottime = (ic->ic_flags & IEEE80211_F_SHSLOT) ? 9 : 20;
	for (aci = 0; aci < WME_NUM_AC; aci++) {
		ac = &ic->ic_edca_ac[aci];
		/* AIFS[AC] = AIFSN[AC] * aSlotTime + aSIFSTime. */
		aifs = ac->ac_aifsn * slottime + 10;
		urtwn_write_4(dp, aci2reg[aci],
		    SM(R92C_EDCA_PARAM_TXOP, ac->ac_txoplimit) |
		    SM(R92C_EDCA_PARAM_ECWMIN, ac->ac_ecwmin) |
		    SM(R92C_EDCA_PARAM_ECWMAX, ac->ac_ecwmax) |
		    SM(R92C_EDCA_PARAM_AIFS, aifs));
	}
	splx(s);
}
#endif

#if 0
int
urtwn_set_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct urtwn_softc *sc = ic->ic_softc;
	struct urtwn_cmd_key cmd;

	/* Defer setting of WEP keys until interface is brought up. */
	if ((ic->ic_if.if_flags & (IFF_UP | IFF_RUNNING)) !=
	    (IFF_UP | IFF_RUNNING))
		return (0);

	/* Do it in a process context. */
	cmd.key = *k;
	cmd.associd = (ni != NULL) ? ni->in_associd : 0;
	urtwn_do_async(dp, urtwn_set_key_cb, &cmd, sizeof (cmd));
	return (0);
}

static void
urtwn_set_key_cb(struct uwgem_dev *dp, void *arg)
{
	static const uint8_t etherzeroaddr[6] = { 0 };
	struct ieee80211com *ic = &sc->sc_ic;
	struct urtwn_cmd_key *cmd = arg;
	struct ieee80211_key *k = &cmd->key;
	const uint8_t *macaddr;
	uint8_t keybuf[16], algo;
	int i, entry;

	/* Map net80211 cipher to HW crypto algorithm. */
	switch (k->k_cipher) {
	case IEEE80211_CIPHER_WEP40:
		algo = R92C_CAM_ALGO_WEP40;
		break;
	case IEEE80211_CIPHER_WEP104:
		algo = R92C_CAM_ALGO_WEP104;
		break;
	case IEEE80211_CIPHER_TKIP:
		algo = R92C_CAM_ALGO_TKIP;
		break;
	case IEEE80211_CIPHER_CCMP:
		algo = R92C_CAM_ALGO_AES;
		break;
	default:
		return;
	}
	if (k->k_flags & IEEE80211_KEY_GROUP) {
		macaddr = etherzeroaddr;
		entry = k->k_id;
	} else {
		macaddr = ic->ic_bss->in_macaddr;
		entry = 4;
	}
	/* Write key. */
	memset(keybuf, 0, sizeof (keybuf));
	memcpy(keybuf, k->k_key, MIN(k->k_len, sizeof (keybuf)));
	for (i = 0; i < 4; i++) {
		urtwn_cam_write(dp, R92C_CAM_KEY(entry, i),
		    LE_READ_4(&keybuf[i * 4]));
	}
	/* Write CTL0 last since that will validate the CAM entry. */
	urtwn_cam_write(dp, R92C_CAM_CTL1(entry),
	    LE_READ_4(&macaddr[2]));
	urtwn_cam_write(dp, R92C_CAM_CTL0(entry),
	    SM(R92C_CAM_ALGO, algo) |
	    SM(R92C_CAM_KEYID, k->k_id) |
	    SM(R92C_CAM_MACLO, LE_READ_2(&macaddr[0])) |
	    R92C_CAM_VALID);
}

static void
urtwn_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct urtwn_softc *sc = ic->ic_softc;
	struct urtwn_cmd_key cmd;

	if (!(ic->ic_if.if_flags & IFF_RUNNING) ||
	    ic->ic_state != IEEE80211_S_RUN)
		return;	/* Nothing to do. */

	/* Do it in a process context. */
	cmd.key = *k;
	cmd.associd = (ni != NULL) ? ni->in_associd : 0;
	urtwn_do_async(dp, urtwn_delete_key_cb, &cmd, sizeof (cmd));
}

static void
urtwn_delete_key_cb(struct uwgem_dev *dp, void *arg)
{
	struct urtwn_cmd_key *cmd = arg;
	struct ieee80211_key *k = &cmd->key;
	int i, entry;

	if (k->k_flags & IEEE80211_KEY_GROUP)
		entry = k->k_id;
	else
		entry = 4;
	urtwn_cam_write(dp, R92C_CAM_CTL0(entry), 0);
	urtwn_cam_write(dp, R92C_CAM_CTL1(entry), 0);
	/* Clear key. */
	for (i = 0; i < 4; i++) {
		urtwn_cam_write(dp, R92C_CAM_KEY(entry, i), 0);
	}
}
#endif

static int
urtwn_power_on(struct uwgem_dev *dp)	/* done */
{
	uint32_t	reg;
	int		ntries;

	DPRINTF(0, (CE_CONT, "!%s: %s: enter", dp->name, __func__));

	/* Wait for autoload done bit set */
	ntries = 0;
	while ((urtwn_read_1(dp, R92C_APS_FSMCO) &
	    R92C_APS_FSMCO_PFM_ALDN) == 0) {
		if (ntries++ >= 1000) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout waiting for chip autoload",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
		drv_usecwait(5);
	}

	/* Unlock ISO/CLK/Power control register (0x1C[7:0] = 0x00) */
	urtwn_write_1(dp, R92C_RSV_CTRL, 0);

	/* Move SPS into PWM mode. */
	urtwn_write_1(dp, R92C_SPS0_CTRL, 0x2b);
	drv_usecwait(100);

	reg = urtwn_read_1(dp, R92C_LDOV12D_CTRL);
	if ((reg & R92C_LDOV12D_CTRL_LDV12_EN) == 0) {
		urtwn_write_1(dp, R92C_LDOV12D_CTRL,
		    reg | R92C_LDOV12D_CTRL_LDV12_EN);
		drv_usecwait(100);

		urtwn_write_1(dp, R92C_SYS_ISO_CTRL,
		    urtwn_read_1(dp, R92C_SYS_ISO_CTRL) &
		    ~R92C_SYS_ISO_CTRL_MD2PP);
	}

	/* Auto enable WLAN. */
	urtwn_write_2(dp, R92C_APS_FSMCO,
	    urtwn_read_2(dp, R92C_APS_FSMCO) | R92C_APS_FSMCO_APFM_ONMAC);

	ntries = 0;
	while ((urtwn_read_2(dp, R92C_APS_FSMCO) &
	    R92C_APS_FSMCO_APFM_ONMAC) != 0) {
		if (ntries++ >= 1000) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout waiting for MAC auto ON",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	/* Enable radio, GPIO and LED functions. */
	ASSERT((R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN) == 0x0812);
	urtwn_write_2(dp, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN);

	/* Release RF digital isolation. */
	urtwn_write_2(dp, R92C_SYS_ISO_CTRL,
	    urtwn_read_2(dp, R92C_SYS_ISO_CTRL) & ~R92C_SYS_ISO_CTRL_DIOR);

	/* Initialize MAC. */
	urtwn_write_1(dp, R92C_APSD_CTRL,
	    urtwn_read_1(dp, R92C_APSD_CTRL) & ~R92C_APSD_CTRL_OFF);

	ntries = 0;
	while ((urtwn_read_1(dp, R92C_APSD_CTRL) &
	    R92C_APSD_CTRL_OFF_STATUS) != 0) {
		if (ntries++ >= 200) {
			cmn_err(CE_WARN,
			    "!%s: %s: timeout waiting for MAC initialization",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
	}

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	reg = urtwn_read_2(dp, R92C_CR);
	reg |= R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN |
	    R92C_CR_TXDMA_EN | R92C_CR_RXDMA_EN | R92C_CR_PROTOCOL_EN |
	    R92C_CR_SCHEDULE_EN | R92C_CR_MACTXEN | R92C_CR_MACRXEN |
	    R92C_CR_ENSEC;
	urtwn_write_2(dp, R92C_CR, reg);

	urtwn_write_1(dp, 0xfe10, 0x19);

	DPRINTF(0, (CE_CONT, "!%s: %s: return", dp->name, __func__));
	return (USB_SUCCESS);
}

static int
urtwn_llt_init(struct uwgem_dev *dp)
{
	int	i, error;

	/* Reserve pages [0, R92C_TX_PAGE_COUNT]. */
	for (i = 0; i < R92C_TX_PAGE_COUNT; i++) {
		if ((error = urtwn_llt_write(dp, i, i + 1)) != USB_SUCCESS) {
			return (error);
		}
	}
	/* NB: 0xff indicates end-of-list. */
	if ((error = urtwn_llt_write(dp, i, 0xff)) != USB_SUCCESS) {
		return (error);
	}
	/*
	 * Use pages [R92C_TX_PAGE_COUNT + 1; R92C_TXPKTBUF_COUNT - 1]
	 * as ring buffer.
	 */
	for (++i; i < R92C_TXPKTBUF_COUNT - 1; i++) {
		if ((error = urtwn_llt_write(dp, i, i + 1)) != USB_SUCCESS) {
			return (error);
		}
	}
	/* Make the last page point to the beginning of the ring buffer. */
	error = urtwn_llt_write(dp, i, R92C_TX_PAGE_COUNT + 1);
	return (error);
}

static void
urtwn_fw_reset(struct uwgem_dev *dp)
{
	uint16_t reg;
	int ntries;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Tell 8051 to reset itself. */
	urtwn_write_1(dp, R92C_HMETFR + 3, 0x20);

	/* Wait until 8051 resets by itself. */
	for (ntries = 0; ntries < 100; ntries++) {
		reg = urtwn_read_2(dp, R92C_SYS_FUNC_EN);
		if (!(reg & R92C_SYS_FUNC_EN_CPUEN)) {
			return;
		}
		drv_usecwait(50);
	}
	/* Force 8051 reset. */
	urtwn_write_2(dp, R92C_SYS_FUNC_EN, reg & ~R92C_SYS_FUNC_EN_CPUEN);
	cmn_err(CE_CONT, "!%s: %s: timeout", dp->name, __func__);
}

static int
urtwn_fw_loadpage(struct uwgem_dev *dp, int page, uint8_t *buf, int len)
{
	uint32_t reg;
	int off, mlen, error = USB_SUCCESS;

	reg = urtwn_read_1(dp, R92C_MCUFWDL + 2) << 16;
	reg = RW(reg, R92C_MCUFWDL_PAGE, page);
	urtwn_write_1(dp, R92C_MCUFWDL + 2, reg >> 16);

	off = R92C_FW_START_ADDR;
	while (len > 0) {
		if (len >= 196) {
			mlen = 196;
		} else if (len >= 4) {
			mlen = 4;
		} else {
			mlen = 1;
		}
		error = urtwn_write_N(dp, off, buf, mlen);
		if (error != USB_SUCCESS) {
			break;
		}
		off += mlen;
		buf += mlen;
		len -= mlen;
	}
	return (error);
}

static int
urtwn_load_firmware(struct uwgem_dev *dp)
{
	struct urtwn_dev 	*lp = dp->private;
	const struct r92c_fw_hdr *hdr;
	const char	*name;
	const uint8_t	*fw, *ptr;
	size_t		len;
	uint32_t	reg;
	int		mlen, ntries, page;
	int		error = USB_SUCCESS;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Read firmware image from the filesystem. */
	if ((lp->chip & (URTWN_CHIP_UMC_A_CUT | URTWN_CHIP_92C))
	    == URTWN_CHIP_UMC_A_CUT) {
		fw = urtwn_rtl8192cfwU_fw;
		len = sizeof (urtwn_rtl8192cfwU_fw);
	} else {
		fw = urtwn_rtl8192cfwT_fw;
		len = sizeof (urtwn_rtl8192cfwT_fw);
	}

	if (len < sizeof (*hdr)) {
		cmn_err(CE_WARN, "!%s: firmware too short", dp->name);
		error = USB_FAILURE;
		goto fail;
	}
	ptr = fw;
	hdr = (const struct r92c_fw_hdr *)ptr;
	/* Check if there is a valid FW header and skip it. */
	if ((letoh16(hdr->signature) >> 4) == 0x88c ||
	    (letoh16(hdr->signature) >> 4) == 0x92c) {
		DPRINTF(0, (CE_CONT, "!%s: %s: FW V%d.%d %02d-%02d %02d:%02d",
		    dp->name, __func__,
		    letoh16(hdr->version), letoh16(hdr->subversion),
		    hdr->month, hdr->date, hdr->hour, hdr->minute));
		ptr += sizeof (*hdr);
		len -= sizeof (*hdr);
	}

	if (urtwn_read_1(dp, R92C_MCUFWDL) & 0x80) {
		urtwn_fw_reset(dp);
		urtwn_write_1(dp, R92C_MCUFWDL, 0);
	}

	/* download enable */
	urtwn_write_1(dp, R92C_SYS_FUNC_EN + 1,
	    urtwn_read_1(dp, R92C_SYS_FUNC_EN + 1) |
	    R92C_SYS_FUNC_EN_CPUEN >> 8);

	urtwn_write_1(dp, R92C_MCUFWDL,
	    urtwn_read_1(dp, R92C_MCUFWDL) | R92C_MCUFWDL_EN);

	urtwn_write_1(dp, R92C_MCUFWDL + 2,
	    urtwn_read_1(dp, R92C_MCUFWDL + 2) & ~0x08);

	/* download firmware */
	for (page = 0; len > 0; page++) {
		mlen = MIN(len, R92C_FW_PAGE_SIZE);
		error = urtwn_fw_loadpage(dp, page, (void *)ptr, mlen);
		if (error != USB_SUCCESS) {
			cmn_err(CE_WARN, "!%s: could not load firmware page %d",
			    dp->name);
			goto fail;
		}
		ptr += mlen;
		len -= mlen;
	}

	/* download disable */
	urtwn_write_1(dp, R92C_MCUFWDL,
	    urtwn_read_1(dp, R92C_MCUFWDL) & ~R92C_MCUFWDL_EN);
	urtwn_write_1(dp, R92C_MCUFWDL + 1, 0);

	/* Wait for checksum report. */
	ntries = 0;
	while ((urtwn_read_4(dp, R92C_MCUFWDL) &
	    R92C_MCUFWDL_CHKSUM_RPT) == 0) {
		if (ntries++ >= 1000) {
			cmn_err(CE_WARN,
			    "!%s: timeout waiting for checksum report",
			    dp->name);
			error = USB_FAILURE;
			goto fail;
		}
	}

	/* Wait for firmware readiness. */
	reg = urtwn_read_4(dp, R92C_MCUFWDL);
	reg = (reg & ~R92C_MCUFWDL_WINTINI_RDY) | R92C_MCUFWDL_RDY;
	urtwn_write_4(dp, R92C_MCUFWDL, reg);

	ntries = 0;
	while ((urtwn_read_4(dp, R92C_MCUFWDL) &
	    R92C_MCUFWDL_WINTINI_RDY) == 0) {
		if (ntries++ >= 1000) {
			cmn_err(CE_WARN,
			    "!%s: timeout waiting for firmware readiness",
			    dp->name);
			error = USB_FAILURE;
			goto fail;
		}
		drv_usecwait(5000);
	}
fail:
	DPRINTF(0, (CE_CONT, "!%s: %s return(%d)", dp->name, __func__, error));
	return (error);
}

static int
urtwn_dma_init(struct uwgem_dev *dp)
{
	int hashq, hasnq, haslq, nqueues, nqpages, nrempages;
	uint32_t reg;
	int error;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Initialize LLT table. */
	error = urtwn_llt_init(dp);
	if (error != USB_SUCCESS) {
		return (error);
	}

	/*
	 * InitQueueReservePage		need to check
	 */
	/* Get Tx queues to USB endpoints mapping. */
	hashq = hasnq = haslq = 0;
	reg = urtwn_read_2(dp, R92C_USB_EP + 1);
	DPRINTF(0, (CE_CONT, "!%s: %s: USB endpoints mapping 0x%x",
	    dp->name, __func__, reg));
	if (MS(reg, R92C_USB_EP_HQ) != 0) {
		hashq = 1;
	}
	if (MS(reg, R92C_USB_EP_NQ) != 0) {
		hasnq = 1;
	}
	if (MS(reg, R92C_USB_EP_LQ) != 0) {
		haslq = 1;
	}
	nqueues = hashq + hasnq + haslq;
	if (nqueues == 0) {
		DPRINTF(0, (CE_WARN, "!%s: %s no queues", dp->name, __func__));
		return (USB_FAILURE);
	}
	/* Get the number of pages for each queue. */
	nqpages = (R92C_TX_PAGE_COUNT - R92C_PUBQ_NPAGES) / nqueues;
	/* The remaining pages are assigned to the high priority queue. */
	nrempages = (R92C_TX_PAGE_COUNT - R92C_PUBQ_NPAGES) % nqueues;

	/* Set number of pages for normal priority queue. */
	urtwn_write_1(dp, R92C_RQPN_NPQ, hasnq ? nqpages : 0);
	urtwn_write_4(dp, R92C_RQPN,
	    /* Set number of pages for public queue. */
	    SM(R92C_RQPN_PUBQ, R92C_PUBQ_NPAGES) |
	    /* Set number of pages for high priority queue. */
	    SM(R92C_RQPN_HPQ, hashq ? nqpages + nrempages : 0) |
	    /* Set number of pages for low priority queue. */
	    SM(R92C_RQPN_LPQ, haslq ? nqpages : 0) |
	    /* Load values. */
	    R92C_RQPN_LD);

	/*
	 * InitTxBufferBoundary()
	 */
	urtwn_write_1(dp, R92C_TXPKTBUF_BCNQ_BDNY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(dp, R92C_TXPKTBUF_MGQ_BDNY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(dp, R92C_TXPKTBUF_WMAC_LBK_BF_HD, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(dp, R92C_TRXFF_BNDY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(dp, R92C_TDECTRL + 1, R92C_TX_PAGE_BOUNDARY);

	/*
	 * InitQueuePriority()
	 */
	/* Set queue to USB pipe mapping. */
	reg = urtwn_read_2(dp, R92C_TRXDMA_CTRL);
	reg &= ~R92C_TRXDMA_CTRL_QMAP_M;
	if (nqueues == 1) {
		if (hashq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ;
		} else if (hasnq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_NQ;
		} else {
			reg |= R92C_TRXDMA_CTRL_QMAP_LQ;
		}
	} else if (nqueues == 2) {
		/* All 2-endpoints configs have a high priority queue. */
		if (!hashq) {
			return (USB_FAILURE);
		}
		if (hasnq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ_NQ;
		} else {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ_LQ;
		}
	} else {
		reg |= R92C_TRXDMA_CTRL_QMAP_3EP;
	}
	urtwn_write_2(dp, R92C_TRXDMA_CTRL, reg);

	/*
	 * InitPageBoundary()
	 */
	/* Set Tx/Rx transfer page boundary. */
	urtwn_write_2(dp, R92C_TRXFF_BNDY + 2, 0x27ff);

	/*
	 * InitTransferPageSize()
	 */
	/* Set Tx/Rx transfer page size. */
	urtwn_write_1(dp, R92C_PBP,
	    SM(R92C_PBP_PSRX, R92C_PBP_128) |
	    SM(R92C_PBP_PSTX, R92C_PBP_128));

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));

	return (USB_SUCCESS);
}

static void
urtwn_mac_init(struct uwgem_dev *dp)
{
	int i;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Write MAC initialization values. */
	for (i = 0; i < NITEMS(rtl8192cu_mac); i++) {
		urtwn_write_1(dp, rtl8192cu_mac[i].reg, rtl8192cu_mac[i].val);
	}
	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_bb_init(struct uwgem_dev *dp)
{
	struct urtwn_dev	*lp = dp->private;
	const struct urtwn_bb_prog	*prog;
	uint32_t	reg;
	int	i;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Enable BB and RF. */
	urtwn_write_2(dp, R92C_SYS_FUNC_EN,
	    urtwn_read_2(dp, R92C_SYS_FUNC_EN) |
	    R92C_SYS_FUNC_EN_BBRSTB | R92C_SYS_FUNC_EN_BB_GLB_RST |
	    R92C_SYS_FUNC_EN_DIO_RF);

	urtwn_write_1(dp, R92C_AFE_PLL_CTRL, 0x83);
	urtwn_write_1(dp, R92C_AFE_PLL_CTRL + 1, 0xdb);

	urtwn_write_1(dp, R92C_RF_CTRL,
	    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);

	urtwn_write_1(dp, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBA | R92C_SYS_FUNC_EN_USBD |
	    R92C_SYS_FUNC_EN_BB_GLB_RST | R92C_SYS_FUNC_EN_BBRSTB);

	urtwn_write_1(dp, R92C_LDOHCI12_CTRL, 0x0f);
	urtwn_write_1(dp, 0x15, 0xe9);
	urtwn_write_1(dp, R92C_AFE_XTAL_CTRL + 1, 0x80);

	/* Config BB and AGC */

	/* Select BB programming based on board type. */
	if (!(lp->chip & URTWN_CHIP_92C)) {
		if (lp->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = &rtl8188ce_bb_prog;
		} else if (lp->board_type == R92C_BOARD_TYPE_HIGHPA) {
			prog = &rtl8188ru_bb_prog;
		} else {
			prog = &rtl8188cu_bb_prog;
		}
	} else {
		if (lp->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = &rtl8192ce_bb_prog;
		} else {
			prog = &rtl8192cu_bb_prog;
		}
	}
	/* Write BB initialization values. */
	for (i = 0; i < prog->count; i++) {
		/* additional delay depend on registers */
		switch (prog->regs[i]) {
		case 0xfe:
			delay(drv_usectohz(50*1000));
			break;

		case 0xfd:
			delay(drv_usectohz(5*1000));
			break;

		case 0xfc:
			delay(drv_usectohz(1*1000));
			break;

		case 0xfb:
			drv_usecwait(50);
			break;

		case 0xfa:
			drv_usecwait(5);
			break;

		case 0xf9:
			drv_usecwait(1);
			break;
		}

		urtwn_bb_write(dp, prog->regs[i], prog->vals[i]);
		drv_usecwait(1);
	}

	/* from phy_ConfigBBExternalPA() */
        if (lp->ExternalPA) {
		urtwn_bb_write(dp, 0xee8,
		    urtwn_bb_read(dp, 0xee8) | BIT(28));

		urtwn_bb_write(dp, 0x860,
		    urtwn_bb_read(dp, 0x860) |
		    BIT(26) | BIT(21) | BIT(10) | BIT(5));

		urtwn_bb_write(dp, 0x870,
		   urtwn_bb_read(dp, 0x870) & ~BIT(10));

		urtwn_bb_write(dp, 0xc80, 0x20000080);
		urtwn_bb_write(dp, 0xc88, 0x40000100);
	}

	if (lp->chip & URTWN_CHIP_92C_1T2R) {
		/* 8192C 1T only configuration. from phy_BB8192C_Config_1T */
		reg = urtwn_bb_read(dp, R92C_FPGA0_TXINFO);
		reg = (reg & ~0x00000003) | 0x2;
		urtwn_bb_write(dp, R92C_FPGA0_TXINFO, reg);

		reg = urtwn_bb_read(dp, R92C_FPGA1_TXINFO);
		reg = (reg & ~0x00300033) | 0x00200022;
		urtwn_bb_write(dp, R92C_FPGA1_TXINFO, reg);

		reg = urtwn_bb_read(dp, R92C_CCK0_AFESETTING);
		reg = (reg & ~0xff000000) | 0x45 << 24;
		urtwn_bb_write(dp, R92C_CCK0_AFESETTING, reg);

		reg = urtwn_bb_read(dp, R92C_OFDM0_TRXPATHENA);
		reg = (reg & ~0x000000ff) | 0x23;
		urtwn_bb_write(dp, R92C_OFDM0_TRXPATHENA, reg);

		reg = urtwn_bb_read(dp, R92C_OFDM0_AGCPARAM1);
		reg = (reg & ~0x00000030) | 1 << 4;
		urtwn_bb_write(dp, R92C_OFDM0_AGCPARAM1, reg);

		reg = urtwn_bb_read(dp, 0xe74);
		reg = (reg & ~0x0c000000) | 2 << 26;
		urtwn_bb_write(dp, 0xe74, reg);

		reg = urtwn_bb_read(dp, 0xe78);
		reg = (reg & ~0x0c000000) | 2 << 26;
		urtwn_bb_write(dp, 0xe78, reg);

		reg = urtwn_bb_read(dp, 0xe7c);
		reg = (reg & ~0x0c000000) | 2 << 26;
		urtwn_bb_write(dp, 0xe7c, reg);

		reg = urtwn_bb_read(dp, 0xe80);
		reg = (reg & ~0x0c000000) | 2 << 26;
		urtwn_bb_write(dp, 0xe80, reg);

		reg = urtwn_bb_read(dp, 0xe88);
		reg = (reg & ~0x0c000000) | 2 << 26;
		urtwn_bb_write(dp, 0xe88, reg);
	}

	/* Write AGC values. */
	for (i = 0; i < prog->agccount; i++) {
		urtwn_bb_write(dp, R92C_OFDM0_AGCRSSITABLE,
		    prog->agcvals[i]);
		drv_usecwait(1);
	}

	if (urtwn_bb_read(dp, R92C_HSSI_PARAM2(0)) &
	    R92C_HSSI_PARAM2_CCK_HIPWR) {
		lp->sc_flags |= URTWN_FLAG_CCK_HIPWR;
	}

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_rf_init(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	const struct urtwn_rf_prog *prog;
	uint32_t reg, type;
	int i, j, idx;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Select RF programming based on board type. */
	if (!(lp->chip & URTWN_CHIP_92C)) {
		if (lp->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = rtl8188ce_rf_prog;
		} else if (lp->board_type == R92C_BOARD_TYPE_HIGHPA) {
			prog = rtl8188ru_rf_prog;
		} else {
			prog = rtl8188cu_rf_prog;
		}
	} else {
		prog = rtl8192ce_rf_prog;
	}

	for (i = 0; i < lp->nrxchains; i++) {
		uint32_t	saved;
		uint32_t	mask;
		/* Save RF_ENV control type. */
		idx = i / 2;
		mask = 0xffffU << ((i % 2) * 16);
		saved = urtwn_bb_read(dp, R92C_FPGA0_RFIFACESW(idx)) & mask;

		/* Set RF_ENV enable. */
		reg = urtwn_bb_read(dp, R92C_FPGA0_RFIFACEOE(i));
		reg |= 0x100000;
		urtwn_bb_write(dp, R92C_FPGA0_RFIFACEOE(i), reg);
		drv_usecwait(1);

		/* Set RF_ENV output high. */
		reg = urtwn_bb_read(dp, R92C_FPGA0_RFIFACEOE(i));
		reg |= 0x10;
		urtwn_bb_write(dp, R92C_FPGA0_RFIFACEOE(i), reg);
		drv_usecwait(1);

		/* Set address and data lengths of RF registers. */
		reg = urtwn_bb_read(dp, R92C_HSSI_PARAM2(i));
		reg &= ~R92C_HSSI_PARAM2_ADDR_LENGTH;
		urtwn_bb_write(dp, R92C_HSSI_PARAM2(i), reg);
		drv_usecwait(1);

		reg = urtwn_bb_read(dp, R92C_HSSI_PARAM2(i));
		reg &= ~R92C_HSSI_PARAM2_DATA_LENGTH;
		urtwn_bb_write(dp, R92C_HSSI_PARAM2(i), reg);
		drv_usecwait(1);

		/* Write RF initialization values for this chain. */
		for (j = 0; j < prog[i].count; j++) {
			if (prog[i].regs[j] >= 0xf9 &&
			    prog[i].regs[j] <= 0xfe) {
				/*
				 * These are fake RF registers offsets that
				 * indicate a delay is required.
				 */
				delay(drv_usectohz(50*1000));
				continue;
			}
			urtwn_rf_write(dp, i, prog[i].regs[j],
			    prog[i].vals[j]);
			drv_usecwait(1);
		}

		/* Restore RF_ENV control type. */
		reg = urtwn_bb_read(dp, R92C_FPGA0_RFIFACESW(idx)) & ~mask;
		urtwn_bb_write(dp, R92C_FPGA0_RFIFACESW(idx), reg | saved);
	}

	if ((lp->chip & (URTWN_CHIP_UMC_A_CUT | URTWN_CHIP_92C)) ==
	    URTWN_CHIP_UMC_A_CUT) {
		urtwn_rf_write(dp, 0, R92C_RF_RX_G1, 0x30255);
		urtwn_rf_write(dp, 0, R92C_RF_RX_G2, 0x50a00);
	}

	/* Cache RF register CHNLBW. */
	for (i = 0; i < 2; i++) {
		lp->rf_chnlbw[i] = urtwn_rf_read(dp, i, R92C_RF_CHNLBW);
	}

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_cam_init(struct uwgem_dev *dp)
{
	uint8_t		ucIndex;
	uint32_t	ulCommand;
	uint32_t	ulContent;
	int		i;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	for (ucIndex = 0; ucIndex < R92C_CAM_ENTRY_COUNT; ucIndex++) {
		/* keyid must be set in config field */
		ulContent = (ucIndex & 3)
		    | R92C_CAM_ALGO_AES << R92C_CAM_ALGO_S
		    | R92C_CAM_VALID;

		/* polling bit, and No Write enable, and address */
		ulCommand = R92C_CAMCMD_POLLING
		    | R92C_CAMCMD_WRITE
		    | R92C_CAM_CTL0(ucIndex);
		/* write content 0 is equal to mark invalid */
		urtwn_write_4(dp, R92C_CAMWRITE, ulContent);
		urtwn_write_4(dp, R92C_CAMCMD, ulCommand);
	}

	for (ucIndex = 0; ucIndex < R92C_CAM_ENTRY_COUNT; ucIndex++) {

		for (i = 0; i < /* CAM_CONTENT_COUNT */ 8; i++) {
			/* filled id in CAM config 2 byte */
			if (i == 0) {
				ulContent = (ucIndex & 3)
				    | R92C_CAM_ALGO_AES << R92C_CAM_ALGO_S
				    | R92C_CAM_VALID;
			} else {
				ulContent = 0;
			}

			/* polling bit, and No Write enable, and address */
			ulCommand = R92C_CAMCMD_POLLING
			    | R92C_CAMCMD_WRITE
			    | R92C_CAM_CTL0(ucIndex) | (uint_t)i;
			/* write content 0 is equall to mark invalid */
			urtwn_write_4(dp, R92C_CAMWRITE, ulContent);
			urtwn_write_4(dp, R92C_CAMCMD, ulCommand);
		}
	}

	/* Invalidate all CAM entries. */
	urtwn_write_4(dp, R92C_CAMCMD, R92C_CAMCMD_POLLING | R92C_CAMCMD_CLR);

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_pa_bias_init(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	uint8_t	reg;
	int	i;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	for (i = 0; i < lp->nrxchains; i++) {
		if (lp->pa_setting & (1U << i)) {
			continue;
		}
		urtwn_rf_write(dp, i, R92C_RF_IPA, 0x0f406);
		urtwn_rf_write(dp, i, R92C_RF_IPA, 0x4f406);
		urtwn_rf_write(dp, i, R92C_RF_IPA, 0x8f406);
		urtwn_rf_write(dp, i, R92C_RF_IPA, 0xcf406);
	}
	if (!(lp->pa_setting & 0x10)) {
		reg = urtwn_read_1(dp, 0x16);
		reg = (reg & ~0xf0) | 0x90;
		urtwn_write_1(dp, 0x16, reg);
	}

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_rxfilter_init(struct uwgem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Initialize Rx filter. */
	/* TODO: use better filter for monitor mode. */
	urtwn_write_4(dp, R92C_RCR,
	    R92C_RCR_AAP | R92C_RCR_APM | R92C_RCR_AM | R92C_RCR_AB |
	    R92C_RCR_APP_ICV | R92C_RCR_AMF | R92C_RCR_HTC_LOC_CTRL |
	    R92C_RCR_APP_MIC | R92C_RCR_APP_PHYSTS);

	/* Accept all multicast frames. */
	urtwn_write_4(dp, R92C_MAR + 0, 0xffffffff);
	urtwn_write_4(dp, R92C_MAR + 4, 0xffffffff);

	/* Accept all management frames. */
	urtwn_write_2(dp, R92C_RXFLTMAP0, 0xffff);

	/* Reject all control frames. */
	urtwn_write_2(dp, R92C_RXFLTMAP1, 0x0000);

	/* Accept all data frames. */
	urtwn_write_2(dp, R92C_RXFLTMAP2, 0xffff);

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_edca_init(struct uwgem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* set spec SIFS (used in NAV) */
	urtwn_write_2(dp, R92C_SPEC_SIFS, 0x100a);
	urtwn_write_2(dp, R92C_MAC_SPEC_SIFS, 0x100a);

	/* set SIFS for CCK */
	urtwn_write_2(dp, R92C_SIFS_CCK, 0x100a);

	/* set SIFS for OFDM */
	urtwn_write_2(dp, R92C_SIFS_OFDM, 0x100a);

	/* TXOP */
	urtwn_write_4(dp, R92C_EDCA_BE_PARAM, 0x005ea42b);
	urtwn_write_4(dp, R92C_EDCA_BK_PARAM, 0x0000a44f);
	urtwn_write_4(dp, R92C_EDCA_VI_PARAM, 0x005ea324);
	urtwn_write_4(dp, R92C_EDCA_VO_PARAM, 0x002fa226);

	DPRINTF(0, (CE_CONT, "!%s: %s return", dp->name, __func__));
}

static void
urtwn_write_txpower(struct uwgem_dev *dp, int chain,
    uint16_t power[URTWN_RIDX_COUNT])
{
	uint32_t reg;

	/* Write per-CCK rate Tx power. */
	if (chain == 0) {
		reg = urtwn_bb_read(dp, R92C_TXAGC_A_CCK1_MCS32);
		reg = RW(reg, R92C_TXAGC_A_CCK1,  power[0]);
		urtwn_bb_write(dp, R92C_TXAGC_A_CCK1_MCS32, reg);

		reg = urtwn_bb_read(dp, R92C_TXAGC_B_CCK11_A_CCK2_11);
		reg = RW(reg, R92C_TXAGC_A_CCK2,  power[1]);
		reg = RW(reg, R92C_TXAGC_A_CCK55, power[2]);
		reg = RW(reg, R92C_TXAGC_A_CCK11, power[3]);
		urtwn_bb_write(dp, R92C_TXAGC_B_CCK11_A_CCK2_11, reg);
	} else {
		reg = urtwn_bb_read(dp, R92C_TXAGC_B_CCK1_55_MCS32);
		reg = RW(reg, R92C_TXAGC_B_CCK1,  power[0]);
		reg = RW(reg, R92C_TXAGC_B_CCK2,  power[1]);
		reg = RW(reg, R92C_TXAGC_B_CCK55, power[2]);
		urtwn_bb_write(dp, R92C_TXAGC_B_CCK1_55_MCS32, reg);

		reg = urtwn_bb_read(dp, R92C_TXAGC_B_CCK11_A_CCK2_11);
		reg = RW(reg, R92C_TXAGC_B_CCK11, power[3]);
		urtwn_bb_write(dp, R92C_TXAGC_B_CCK11_A_CCK2_11, reg);
	}
	/* Write per-OFDM rate Tx power. */
	urtwn_bb_write(dp, R92C_TXAGC_RATE18_06(chain),
	    SM(R92C_TXAGC_RATE06, power[ 4]) |
	    SM(R92C_TXAGC_RATE09, power[ 5]) |
	    SM(R92C_TXAGC_RATE12, power[ 6]) |
	    SM(R92C_TXAGC_RATE18, power[ 7]));
	urtwn_bb_write(dp, R92C_TXAGC_RATE54_24(chain),
	    SM(R92C_TXAGC_RATE24, power[ 8]) |
	    SM(R92C_TXAGC_RATE36, power[ 9]) |
	    SM(R92C_TXAGC_RATE48, power[10]) |
	    SM(R92C_TXAGC_RATE54, power[11]));
	/* Write per-MCS Tx power. */
	urtwn_bb_write(dp, R92C_TXAGC_MCS03_MCS00(chain),
	    SM(R92C_TXAGC_MCS00,  power[12]) |
	    SM(R92C_TXAGC_MCS01,  power[13]) |
	    SM(R92C_TXAGC_MCS02,  power[14]) |
	    SM(R92C_TXAGC_MCS03,  power[15]));
	urtwn_bb_write(dp, R92C_TXAGC_MCS07_MCS04(chain),
	    SM(R92C_TXAGC_MCS04,  power[16]) |
	    SM(R92C_TXAGC_MCS05,  power[17]) |
	    SM(R92C_TXAGC_MCS06,  power[18]) |
	    SM(R92C_TXAGC_MCS07,  power[19]));
	urtwn_bb_write(dp, R92C_TXAGC_MCS11_MCS08(chain),
	    SM(R92C_TXAGC_MCS08,  power[20]) |
	    SM(R92C_TXAGC_MCS09,  power[21]) |
	    SM(R92C_TXAGC_MCS10,  power[22]) |
	    SM(R92C_TXAGC_MCS11,  power[23]));
	urtwn_bb_write(dp, R92C_TXAGC_MCS15_MCS12(chain),
	    SM(R92C_TXAGC_MCS12,  power[24]) |
	    SM(R92C_TXAGC_MCS13,  power[25]) |
	    SM(R92C_TXAGC_MCS14,  power[26]) |
	    SM(R92C_TXAGC_MCS15,  power[27]));
}

#ifdef CHAN_ORG
static void
urtwn_get_txpower(struct uwgem_dev *dp, int chain,
    struct ieee80211_channel *c, uint_t ht40m,
    uint16_t power[URTWN_RIDX_COUNT])
#else
static void
urtwn_get_txpower(struct uwgem_dev *dp, int chain,
    int chan, uint_t ht40m,
    uint16_t power[URTWN_RIDX_COUNT])
#endif
{
	struct urtwn_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	struct r92c_rom *rom = &lp->rom;
	uint16_t cckpow, ofdmpow, htpow, diff, max;
	const struct urtwn_txpwr *base;
	int ridx, group;
#ifdef CHAN_ORG
	int chan;

	/* Determine channel group. */
	chan = ieee80211_chan2ieee(ic, c);	/* XXX center freq! */
#endif
	if (chan <= 3) {
		group = 0;
	} else if (chan <= 9) {
		group = 1;
	} else {
		group = 2;
	}

	/* Get original Tx power based on board type and RF chain. */
	if (!(lp->chip & URTWN_CHIP_92C)) {
		if (lp->board_type == R92C_BOARD_TYPE_HIGHPA) {
			base = &rtl8188ru_txagc[chain];
		} else {
			base = &rtl8192cu_txagc[chain];
		}
	} else {
		base = &rtl8192cu_txagc[chain];
	}

	memset(power, 0, URTWN_RIDX_COUNT * sizeof (power[0]));
	if (lp->regulatory == 0) {
		for (ridx = 0; ridx <= 3; ridx++) {
			power[ridx] = base->pwr[0][ridx];
		}
	}
	for (ridx = 4; ridx < URTWN_RIDX_COUNT; ridx++) {
		if (lp->regulatory == 3) {
			power[ridx] = base->pwr[0][ridx];
			/* Apply vendor limits. */
			if (ht40m) {
				max = rom->ht40_max_pwr[group];
			} else {
				max = rom->ht20_max_pwr[group];
			}
			max = (max >> (chain * 4)) & 0xf;
			if (power[ridx] > max) {
				power[ridx] = max;
			}
		} else if (lp->regulatory == 1) {
			if (ht40m == 0) {
				power[ridx] = base->pwr[group][ridx];
			}
		} else if (lp->regulatory != 2) {
			power[ridx] = base->pwr[0][ridx];
		}
	}

	/* Compute per-CCK rate Tx power. */
	cckpow = rom->cck_tx_pwr[chain][group];	/* cck_tx_pwr is 0x5a */
	for (ridx = 0; ridx < 4; ridx++) {
		power[ridx] += cckpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}

	htpow = rom->ht40_1s_tx_pwr[chain][group];
	if (lp->ntxchains > 1) {
		/* Apply reduction for 2 spatial streams. */
		diff = rom->ht40_2s_tx_pwr_diff[group];
		diff = (diff >> (chain * 4)) & 0xf;
		htpow = (htpow > diff) ? htpow - diff : 0;
	}

	/* Compute per-OFDM rate Tx power. */
	diff = rom->ofdm_tx_pwr_diff[group];
	diff = (diff >> (chain * 4)) & 0xf;
	ofdmpow = htpow + diff;	/* HT->OFDM correction. */
	for (ridx = 4; ridx < 12; ridx++) {
		power[ridx] += ofdmpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}

	/* Compute per-MCS Tx power. */
	if (ht40m == 0) {
		diff = rom->ht20_tx_pwr_diff[group];
		diff = (diff >> (chain * 4)) & 0xf;
		htpow += diff;	/* HT40->HT20 correction. */
	}
	for (ridx = 12; ridx < URTWN_RIDX_COUNT; ridx++) {
		power[ridx] += htpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}
#ifdef DEBUG_LEVEL
	if (urtwn_debug >= 10) {
		/* Dump per-rate Tx power values. */
		printf("Tx power for chain %d:\n", chain);
		for (ridx = 0; ridx < URTWN_RIDX_COUNT; ridx++) {
			printf("Rate %d = %u\n", ridx, power[ridx]);
		}
	}
#endif
}

#ifdef CHAN_ORG
static void
urtwn_set_txpower(struct uwgem_dev *dp, struct ieee80211_channel *c,
    uint_t ht40m)
#else
static void
urtwn_set_txpower(struct uwgem_dev *dp, int chan, uint_t ht40m)
#endif
{
	struct urtwn_dev *lp = dp->private;
	uint16_t power[URTWN_RIDX_COUNT];
	int i;

	for (i = 0; i < lp->ntxchains; i++) {
		/* Compute per-rate Tx power values. */
#ifdef CHAN_ORG
		urtwn_get_txpower(dp, i, c, ht40m, power);
#else
		urtwn_get_txpower(dp, i, chan, ht40m, power);
#endif
		/* Write per-rate Tx power values to hardware. */
		urtwn_write_txpower(dp, i, power);
	}
}

static void
urtwn_set_chan(struct uwgem_dev *dp, struct ieee80211_channel *c,
    uint_t ht40m)
{
	struct urtwn_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	uint32_t	reg;
	uint_t	chan;
	int	i;

	chan = ieee80211_chan2ieee(ic, c);	/* XXX center freq! */

	DPRINTF(10, (CE_CONT, "!%s: %s enter, chan:%d",
	    dp->name, __func__, chan));

	if (ht40m == IEEE80211_HTINFO_2NDCHAN_ABOVE) {
		chan += 2;
	} else if (ht40m == IEEE80211_HTINFO_2NDCHAN_BELOW) {
		chan -= 2;
	}

#ifdef CHAN_ORG
	/* Set Tx power for this new channel. */
	urtwn_set_txpower(dp, c, ht40m);
#else
	/* Set Tx power for this new channel. */
	urtwn_set_txpower(dp, chan, ht40m);
#endif
	for (i = 0; i < lp->nrxchains; i++) {
		urtwn_rf_write(dp, i, R92C_RF_CHNLBW,
		    RW(lp->rf_chnlbw[i], R92C_RF_CHNLBW_CHNL, chan));
	}

	if (ht40m) {
		/* Is secondary channel below or above primary? */
		int prichlo = (ht40m == IEEE80211_HTINFO_2NDCHAN_ABOVE);

		urtwn_write_1(dp, R92C_BWOPMODE,
		    urtwn_read_1(dp, R92C_BWOPMODE) & ~R92C_BWOPMODE_20MHZ);

		reg = urtwn_read_1(dp, R92C_RRSR + 2);
		reg = (reg & ~0x6f) | (prichlo ? 1 : 2) << 5;
		urtwn_write_1(dp, R92C_RRSR + 2, reg);

		urtwn_bb_write(dp, R92C_FPGA0_RFMOD,
		    urtwn_bb_read(dp, R92C_FPGA0_RFMOD) | R92C_RFMOD_40MHZ);
		urtwn_bb_write(dp, R92C_FPGA1_RFMOD,
		    urtwn_bb_read(dp, R92C_FPGA1_RFMOD) | R92C_RFMOD_40MHZ);

		/* Set CCK side band. */
		reg = urtwn_bb_read(dp, R92C_CCK0_SYSTEM);
		reg = (reg & ~0x00000010) | (prichlo ? 0 : 1) << 4;
		urtwn_bb_write(dp, R92C_CCK0_SYSTEM, reg);

		reg = urtwn_bb_read(dp, R92C_OFDM1_LSTF);
		reg = (reg & ~0x00000c00) | (prichlo ? 1 : 2) << 10;
		urtwn_bb_write(dp, R92C_OFDM1_LSTF, reg);

		urtwn_bb_write(dp, R92C_FPGA0_ANAPARAM2,
		    urtwn_bb_read(dp, R92C_FPGA0_ANAPARAM2) &
		    ~R92C_FPGA0_ANAPARAM2_CBW20);

		reg = urtwn_bb_read(dp, 0x818);
		reg = (reg & ~(3 << 26)) | (prichlo ? 2 : 1) << 26;
		urtwn_bb_write(dp, 0x818, reg);

		/* Select 40MHz bandwidth. */
		urtwn_rf_write(dp, 0, R92C_RF_CHNLBW,
		    (lp->rf_chnlbw[0] & ~0xfff) | chan);
	} else {
		urtwn_write_1(dp, R92C_BWOPMODE,
		    urtwn_read_1(dp, R92C_BWOPMODE) | R92C_BWOPMODE_20MHZ);

		urtwn_bb_write(dp, R92C_FPGA0_RFMOD,
		    urtwn_bb_read(dp, R92C_FPGA0_RFMOD) & ~R92C_RFMOD_40MHZ);
		urtwn_bb_write(dp, R92C_FPGA1_RFMOD,
		    urtwn_bb_read(dp, R92C_FPGA1_RFMOD) & ~R92C_RFMOD_40MHZ);

		urtwn_bb_write(dp, R92C_FPGA0_ANAPARAM2,
		    urtwn_bb_read(dp, R92C_FPGA0_ANAPARAM2) |
		    R92C_FPGA0_ANAPARAM2_CBW20); //

		/* Select 20MHz bandwidth. */
		urtwn_rf_write(dp, 0, R92C_RF_CHNLBW,
		    (lp->rf_chnlbw[0] & ~0xfff) | R92C_RF_CHNLBW_BW20 | chan);
	}
}

/*
 *-------------------------------------------------------------------------
 *	IQK
 *-------------------------------------------------------------------------
 */

#define MAX_TOLERANCE		5
#define IQK_DELAY_TIME		1 	/* ms */

#define IQK_MAC_REG_NUM		4
#define IQK_ADDA_REG_NUM	16
#define IQK_BB_REG_NUM		9
#define HP_THERMAL_NUM		8

static uint8_t			/* bit0 = 1 => Tx OK, bit1 = 1 => Rx OK */
_PHY_PathA_IQK(struct uwgem_dev *dp, boolean_t configPathB)
{
	uint32_t regEAC, regE94, regE9C, regEA4;
	uint8_t result = 0;

	DPRINTF(0, (CE_CONT, "!%s: %s: Path A IQK!", dp->name, __func__));

	/* path-A IQK setting */
	urtwn_bb_write(dp, 0xe30, 0x10008c1f);
	urtwn_bb_write(dp, 0xe34, 0x10008c1f);
	urtwn_bb_write(dp, 0xe38, 0x82140102);

	urtwn_bb_write(dp, 0xe3c, configPathB ? 0x28160202 : 0x28160502);

	/* path-B IQK setting */
	if (configPathB) {
		urtwn_bb_write(dp, 0xe50, 0x10008c22);
		urtwn_bb_write(dp, 0xe54, 0x10008c22);
		urtwn_bb_write(dp, 0xe58, 0x82140102);
		urtwn_bb_write(dp, 0xe5c, 0x28160202);
	}

	/* LO calibration setting */
	urtwn_bb_write(dp, 0xe4c, 0x001028d1);

	/* One shot, path A LOK & IQK */
	urtwn_bb_write(dp, 0xe48, 0xf9000000);
	urtwn_bb_write(dp, 0xe48, 0xf8000000);

	delay(drv_usectohz(IQK_DELAY_TIME * 1000));

	/* Check failed */
	regEAC = urtwn_bb_read(dp, 0xeac);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xeac = 0x%x",
	    dp->name, __func__, regEAC));

	regE94 = urtwn_bb_read(dp, 0xe94);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xe94 = 0x%x",
	    dp->name, __func__, regE94));

	regE9C= urtwn_bb_read(dp, 0xe9c);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xe9c = 0x%x",
	    dp->name, __func__, regE9C));

	regEA4= urtwn_bb_read(dp, 0xea4);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xea4 = 0x%x",
	    dp->name, __func__, regEA4));

        if (!(regEAC & BIT(28)) &&
	    (((regE94 & 0x03FF0000) >> 16) != 0x142) &&
	    (((regE9C & 0x03FF0000) >> 16) != 0x42) ) {
		result |= 1;
	} else {
		/* if Tx not OK, ignore Rx */
		return (result);
	}

	if (!(regEAC & BIT(27)) &&
		/* if Tx is OK, check whether Rx is OK */
		(((regEA4 & 0x03FF0000) >> 16) != 0x132) &&
		(((regEAC & 0x03FF0000) >> 16) != 0x36)) {
		result |= 0x02;
	} else {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: Path A Rx IQK fail!!", dp->name, __func__));
	}

	return (result);
}

static uint8_t				//bit0 = 1 => Tx OK, bit1 = 1 => Rx OK
_PHY_PathB_IQK(struct uwgem_dev *dp)
{
	uint32_t	regEAC, regEB4, regEBC, regEC4, regECC;
	uint8_t	result = 0;

	/*
	 * One shot, path B LOK & IQK
	 */
	urtwn_bb_write(dp, 0xe60, 0x00000002);
	urtwn_bb_write(dp, 0xe60, 0x00000000);

	delay(drv_usectohz(IQK_DELAY_TIME*1000));

	/* Check failed */
	regEAC = urtwn_bb_read(dp, 0xeac);
	DPRINTF(2,(CE_CONT, "!%s: %s: 0xeac = 0x%x",
	    dp->name, __func__, regEAC));

	regEB4 = urtwn_bb_read(dp, 0xeb4);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xeb4 = 0x%x",
	    dp->name, __func__, regEB4));

	regEBC= urtwn_bb_read(dp, 0xebc);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xebc = 0x%x",
	    dp->name, __func__, regEBC));

	regEC4= urtwn_bb_read(dp, 0xec4);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xec4 = 0x%x",
	    dp->name, __func__, regEC4));

	regECC= urtwn_bb_read(dp, 0xecc);
	DPRINTF(0, (CE_CONT, "!%s: %s: 0xecc = 0x%x",
	    dp->name, __func__, regECC));

	if (!(regEAC & BIT(31)) &&
	    (((regEB4 & 0x03FF0000) >> 16) != 0x142) &&
	    (((regEBC & 0x03FF0000) >> 16) != 0x42)) {
		result |= 0x01;
	} else {
		return (result);
	}

	if (!(regEAC & BIT(30)) &&
		(((regEC4 & 0x03FF0000) >> 16) != 0x132) &&
		(((regECC & 0x03FF0000) >> 16) != 0x36)) {
		result |= 0x02;
	} else {
		DPRINTF(0, (CE_CONT, "!%s: %s: Path B Rx IQK fail!!",
		    dp->name, __func__));
	}

	return (result);
}

#define	rOFDM0_XARxIQImbalance	0xc14  //RxIQ imblance matrix
#define	rOFDM0_XBRxIQImbalance	0xc1c
#define	rOFDM0_ECCAThreshold	0xc4c // energy CCA
#define	rOFDM0_AGCRSSITable	0xc78
#define	rOFDM0_HTSTFAGC		0xc7c
#define	rOFDM0_XATxIQImbalance	0xc80
#define	rOFDM0_XBTxIQImbalance	0xc88
#define	rOFDM0_XCTxIQImbalance	0xc90
#define	rOFDM0_XCTxAFE		0xc94
#define	rOFDM0_XDTxAFE		0xc9c
#define	rOFDM0_RxIQExtAnta	0xca0

static void
_PHY_PathAFillIQKMatrix(
	struct uwgem_dev *dp,
	boolean_t   	bIQKOK,
	int		result[][8],
	uint8_t		final_candidate,
	boolean_t	bTxOnly
	)
{
	uint32_t	Oldval_0, X, TX0_A, reg;
	int	Y, TX0_C;
	uint32_t	val;

	DPRINTF(0, (CE_CONT, "!%s: %s: Path A IQ Calibration %s !",
	    dp->name, __func__,
	    bIQKOK ? "Success" : "Failed"));

        if (final_candidate == 0xFF) {
		return;
	} else if (bIQKOK) {

		Oldval_0 =
		    (urtwn_bb_read(dp, rOFDM0_XATxIQImbalance) >> 22) & 0x3FF;

		X = result[final_candidate][0];
		if ((X & 0x00000200) != 0) {
			X = X | 0xFFFFFC00;
		}
		TX0_A = (X * Oldval_0) >> 8;
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: X = 0x%lx, TX0_A = 0x%lx, Oldval_0 0x%lx",
		     dp->name, __func__, X, TX0_A, Oldval_0));

		val = urtwn_bb_read(dp, rOFDM0_XATxIQImbalance) & ~0x3FF;
		urtwn_bb_write(dp, rOFDM0_XATxIQImbalance, val | TX0_A);

		val = urtwn_bb_read(dp, rOFDM0_ECCAThreshold) & ~ BIT(31);
		urtwn_bb_write(dp, rOFDM0_ECCAThreshold,
		    val | (((X * Oldval_0 >> 7) & 1) << 31));

		Y = result[final_candidate][1];
		if ((Y & 0x00000200) != 0) {
			Y = Y | 0xFFFFFC00;
		}

		TX0_C = (Y * Oldval_0) >> 8;
		DPRINTF(0, (CE_CONT, "!%s: %s: Y = 0x%lx, TX = 0x%lx",
		    dp->name, __func__, Y, TX0_C));
		val = urtwn_bb_read(dp, rOFDM0_XCTxAFE) & ~0xF0000000;
		urtwn_bb_write(dp, rOFDM0_XCTxAFE,
		    val | ((TX0_C & 0x3C0) >> 6) << 28);

		val = urtwn_bb_read(dp, rOFDM0_XATxIQImbalance) & ~0x003F0000;
		urtwn_bb_write(dp, rOFDM0_XATxIQImbalance,
		    val | (TX0_C&0x3F) << 16);

		val = urtwn_bb_read(dp, rOFDM0_ECCAThreshold) & ~BIT(29);
		urtwn_bb_write(dp, rOFDM0_ECCAThreshold,
		    val | ((Y* Oldval_0>>7) & 0x1) << 29);

	        if (bTxOnly) {
			DPRINTF(0, (CE_CONT, "!:%s: %s: only Tx OK",
			    dp->name, __func__));
			return;
		}

		reg = result[final_candidate][2];
		val = urtwn_bb_read(dp, rOFDM0_XARxIQImbalance) & ~0x3FF;
		urtwn_bb_write(dp, rOFDM0_XARxIQImbalance, val | reg);

		val = urtwn_bb_read(dp, rOFDM0_XARxIQImbalance) & ~0x3FF;
		urtwn_bb_write(dp, rOFDM0_XARxIQImbalance, val | reg);

		reg = result[final_candidate][3] & 0x3F;
		val = urtwn_bb_read(dp, rOFDM0_XARxIQImbalance) & ~0xFC00;
		urtwn_bb_write(dp, rOFDM0_XARxIQImbalance, val | reg << 10);

		reg = (result[final_candidate][3] >> 6) & 0xF;
		val = urtwn_bb_read(dp, 0xca0) & ~0xF0000000;
		urtwn_bb_write(dp, 0xca0, val | reg << 28);
	}
}

void
_PHY_PathBFillIQKMatrix(
	struct uwgem_dev	*dp,
	boolean_t   	bIQKOK,
	int		result[][8],
	uint8_t		final_candidate,
	boolean_t	bTxOnly			//do Tx only
	)
{
	uint32_t	Oldval_1, X, TX1_A, reg;
	int	Y, TX1_C;
	uint32_t	val;

	DPRINTF(0, (CE_CONT, "!%s: %s: Path B IQ Calibration %s !",
	    dp->name, __func__, (bIQKOK)?"Success":"Failed"));

        if (final_candidate == 0xFF) {
		return;
	}
	else if (bIQKOK) {
		Oldval_1 =
		    (urtwn_bb_read(dp, rOFDM0_XBTxIQImbalance) >> 22) & 0x3FF;

		X = result[final_candidate][4];
		if ((X & 0x00000200) != 0) {
			X = X | 0xFFFFFC00;
		}

		TX1_A = (X * Oldval_1) >> 8;
		DPRINTF(0, (CE_CONT, "!%s: %s: X = 0x%lx, TX1_A = 0x%lx",
		    dp->name, __func__, X, TX1_A));

		val = urtwn_bb_read(dp, rOFDM0_XBTxIQImbalance) & ~0x3FF;
		urtwn_bb_write(dp, rOFDM0_XBTxIQImbalance, val | TX1_A);

		val = urtwn_bb_read(dp, rOFDM0_ECCAThreshold) & ~BIT(27);
		urtwn_bb_write(dp, rOFDM0_ECCAThreshold,
		    val | ((X* Oldval_1>>7) & 0x1) << 27);

		Y = result[final_candidate][5];
		if ((Y & 0x00000200) != 0) {
			Y = Y | 0xFFFFFC00;
		}
		TX1_C = (Y * Oldval_1) >> 8;
		DPRINTF(0, (CE_CONT, "!%s: %s: Y = 0x%lx, TX1_C = 0x%lx",
		    dp->name, __func__, Y, TX1_C));
		val = urtwn_bb_read(dp, rOFDM0_XDTxAFE) & 0xF0000000;
		urtwn_bb_write(dp, rOFDM0_XDTxAFE,
		    val | ((TX1_C&0x3C0)>>6) << 28);

		urtwn_bb_read(dp, rOFDM0_XBTxIQImbalance) & ~0x003F0000;
		urtwn_bb_write(dp, rOFDM0_XBTxIQImbalance,
		    val | (TX1_C & 0x3F) << 16);

		val = urtwn_bb_read(dp, rOFDM0_ECCAThreshold) & ~BIT(25);
		urtwn_bb_write(dp, rOFDM0_ECCAThreshold,
		    val | ((Y * Oldval_1 >> 7) & 1) << 25);

		if (bTxOnly) {
			return;
		}

		reg = result[final_candidate][6];
		val = urtwn_bb_read(dp, rOFDM0_XBRxIQImbalance) & ~0x3FF;
		urtwn_bb_write(dp, rOFDM0_XBRxIQImbalance, val | reg);

		reg = result[final_candidate][7] & 0x3F;
		val = urtwn_bb_read(dp, rOFDM0_XBRxIQImbalance) & ~0xFC00;
		urtwn_bb_write(dp, rOFDM0_XBRxIQImbalance,
		    val | reg << 10);

		reg = (result[final_candidate][7] >> 6) & 0xF;
		val = urtwn_bb_read(dp, rOFDM0_AGCRSSITable) & ~0x0000F000;
		urtwn_bb_write(dp, rOFDM0_AGCRSSITable, val | reg << 12);
	}
}

static void
_PHY_SaveADDARegisters(struct uwgem_dev *dp,
    uint32_t *ADDAReg, uint32_t *ADDABackup, int RegisterNum)
{
	int	i;

	//RTPRINT(FINIT, INIT_IQK, ("Save ADDA parameters.\n"));
	for (i = 0; i < RegisterNum; i++) {
		ADDABackup[i] = urtwn_bb_read(dp, ADDAReg[i]);
	}
}

static void
_PHY_SaveMACRegisters(
    struct uwgem_dev *dp, uint32_t *MACReg, uint32_t *MACBackup)
{
	int	i;

	//RTPRINT(FINIT, INIT_IQK, ("Save MAC parameters.\n"));
	for (i = 0 ; i < IQK_MAC_REG_NUM - 1; i++) {
		MACBackup[i] = urtwn_read_1(dp, MACReg[i]);
	}
	MACBackup[i] = urtwn_read_4(dp, MACReg[i]);
}

static void
_PHY_MACSettingCalibration(struct uwgem_dev *dp,
    uint32_t *MACReg, uint32_t *MACBackup)
{
	int	i = 0;

	//RTPRINT(FINIT, INIT_IQK, ("MAC settings for Calibration.\n"));

	urtwn_write_1(dp, MACReg[i], 0x3f);

	for (i = 1 ; i < IQK_MAC_REG_NUM - 1; i++) {
		urtwn_write_1(dp, MACReg[i], MACBackup[i] & ~BIT(3));
	}
	urtwn_write_1(dp, MACReg[i], MACBackup[i] & ~BIT(5));
}

static void
_PHY_ReloadMACRegisters(struct uwgem_dev *dp,
     uint32_t *MACReg, uint32_t *MACBackup)
{
	int	i;

	//RTPRINT(FINIT, INIT_IQK, ("Reload MAC parameters !\n"));
	for (i = 0 ; i < IQK_MAC_REG_NUM - 1; i++) {
		urtwn_write_1(dp, MACReg[i], MACBackup[i]);
	}
	urtwn_write_4(dp, MACReg[i], MACBackup[i]);
}

static void
_PHY_ReloadADDARegisters(struct uwgem_dev *dp,
    uint32_t *ADDAReg, uint32_t *ADDABackup, int RegiesterNum)
{
	int	i;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	for (i = 0 ; i < RegiesterNum; i++) {
		urtwn_bb_write(dp, ADDAReg[i], ADDABackup[i]);
	}
}

static void
_PHY_PathADDAOn(struct uwgem_dev *dp,
    uint32_t *ADDAReg, boolean_t isPathAOn, boolean_t is2T)
{
	uint32_t	pathOn;
	int	i;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	pathOn = isPathAOn ? 0x04db25a4 : 0x0b1b25a4;
	if (B_FALSE == is2T) {
		pathOn = 0x0bdb25a0;
		urtwn_bb_write(dp, ADDAReg[0], 0x0b1b25a0);
	} else {
		urtwn_bb_write(dp, ADDAReg[0], pathOn);
	}

	for (i = 1; i < IQK_ADDA_REG_NUM ; i++) {
		urtwn_bb_write(dp, ADDAReg[i], pathOn);
	}
}

static void
_PHY_PathAStandBy(struct uwgem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	urtwn_bb_write(dp, 0xe28, 0x0);
	urtwn_bb_write(dp, 0x840, 0x00010000);
	urtwn_bb_write(dp, 0xe28, 0x80800000);
}

static void
_PHY_PIModeSwitch(struct uwgem_dev *dp, boolean_t PIMode)
{
	uint32_t	mode;

	DPRINTF(0, (CE_CONT, "!%s: %s: BB Switch to %s mode!",
	    dp->name, __func__, (PIMode ? "PI" : "SI")));

	mode = PIMode ? 0x01000100 : 0x01000000;
	urtwn_bb_write(dp, 0x820, mode);
	urtwn_bb_write(dp, 0x828, mode);
}

#define	MAX_TOLERANCE	5

static boolean_t
_PHY_SimularityCompare(struct uwgem_dev *dp,
    int result[][8], uint8_t c1, uint8_t c2)
{
	struct urtwn_dev *lp = dp->private;
	int		i, j;
	uint32_t	SimularityBitMap;
	int		bound;
	uint8_t		final_candidate[2] = {0xff, 0xff};	//for path A and path B
	boolean_t	bResult = B_TRUE;
	boolean_t	is2T = lp->ntxchains == 2;

	if (is2T) {
		bound = 8;
	} else {
		bound = 4;
	}

	SimularityBitMap = 0;

	for (i = 0; i < bound; i++) {
		if (ABS(result[c1][i] - result[c2][i]) > MAX_TOLERANCE) {
			if ((i == 2 || i == 6) && SimularityBitMap == 0) {
				if (result[c1][i] + result[c1][i+1] == 0) {
					final_candidate[i / 4] = c2;
					continue;
				}
				if (result[c2][i] + result[c2][i+1] == 0) {
					final_candidate[i / 4] = c1;
					continue;
				}
			}
			SimularityBitMap |= 1 << i;
		}
	}

	if (SimularityBitMap == 0) {
		for (i = 0; i < bound / 4; i++) {
			if (final_candidate[i] != 0xff) {
				for (j = i*4; j < (i+1)*4-2; j++) {
					result[3][j] =
					    result[final_candidate[i]][j];
				}
				bResult = B_FALSE;
			}
		}
		return (bResult);
	}

	if ((SimularityBitMap & 0x0f) == 0) {
		/* path A OK */
		for (i = 0; i < 4; i++) {
			result[3][i] = result[c1][i];
		}
	} else if ((SimularityBitMap & 0xf0) == 0 && is2T) {
		/* path B OK */
		for (i = 4; i < 8; i++) {
			result[3][i] = result[c1][i];
		}
	}
	return (B_FALSE);
}

static void
_PHY_IQCalibrate(struct uwgem_dev *dp,
    int result[][8], uint8_t t, boolean_t is2T)
{
	struct urtwn_dev *lp = dp->private;
	uint32_t	i;
	uint8_t	PathAOK, PathBOK;
	static uint_t	ADDA_REG[IQK_ADDA_REG_NUM] = {
		0x85c, 0xe6c, 0xe70, 0xe74,
		0xe78, 0xe7c, 0xe80, 0xe84,
		0xe88, 0xe8c, 0xed0, 0xed4,
		0xed8, 0xedc, 0xee0, 0xeec,
	};

	static uint_t	IQK_MAC_REG[IQK_MAC_REG_NUM] = {
		0x522, 0x550, 0x551, 0x040
	};

	static uint_t	IQK_BB_REG[IQK_BB_REG_NUM] = {
		0xc04, 0xc08, 0x874, 0xb68, 0xb6c,
		0x870, 0x860, 0x864, 0x800,
	};

#if MP_DRIVER	/* is *not* defined */
	const uint32_t	retryCount = 9;
#else
	const uint32_t	retryCount = 2;
#endif
	/*
	 * Note: IQ calibration must be performed after loading
	 * PHY_REG.txt, and radio_a, radio_b.txt
	 */
	uint32_t	bbvalue;
	boolean_t	isNormal = B_TRUE;

	if (t == 0) {
	 	bbvalue = urtwn_bb_read(dp, 0x800);
		DPRINTF(0, (CE_CONT, "!%s: %s: 0x%08lx",
		    dp->name, __func__, bbvalue));

	 	/* Save ADDA parameters, turn Path A ADDA on */
	 	_PHY_SaveADDARegisters(dp,
		    ADDA_REG, lp->ADDA_backup, NITEMS(ADDA_REG));

		_PHY_SaveMACRegisters(dp,
		    IQK_MAC_REG, lp->IQK_MAC_backup);

		_PHY_SaveADDARegisters(dp,
		    IQK_BB_REG, lp->IQK_BB_backup, NITEMS(IQK_BB_REG));
	}
 	_PHY_PathADDAOn(dp, ADDA_REG, B_TRUE, is2T);

	if (t == 0) {
		lp->bRfPiEnable =
		    (urtwn_bb_read(dp, rFPGA0_XA_HSSIParameter1) & BIT(8)) != 0;
	}

	if (!lp->bRfPiEnable) {
		/* Switch BB to PI mode to do IQ Calibration. */
		_PHY_PIModeSwitch(dp, B_TRUE);
	}

	urtwn_bb_write(dp, 0x800,
	    urtwn_bb_read(dp, 0x800) & ~BIT(24));

	urtwn_bb_write(dp, 0xc04, 0x03a05600);
	urtwn_bb_write(dp, 0xc08, 0x000800e4);
	urtwn_bb_write(dp, 0x874, 0x22204000);

	urtwn_bb_write(dp, 0x870,
	    urtwn_bb_read(dp, 0x870) | BIT(10));
	urtwn_bb_write(dp, 0x870,
	    urtwn_bb_read(dp, 0x870) | BIT(26));
	urtwn_bb_write(dp, 0x860,
	    urtwn_bb_read(dp, 0x860) & ~BIT(10));
	urtwn_bb_write(dp, 0x864,
	    urtwn_bb_read(dp, 0x864) & ~BIT(10));

	if (is2T) {
		urtwn_bb_write(dp, 0x840, 0x00010000);
		urtwn_bb_write(dp, 0x844, 0x00010000);
	}

	/* MAC settings */
	_PHY_MACSettingCalibration(dp, IQK_MAC_REG, lp->IQK_MAC_backup);

	/* Page B init */
	urtwn_bb_write(dp, 0xb68, 0x00080000);

	if (is2T) {
		urtwn_bb_write(dp, 0xb6c, 0x00080000);
	}

	/* IQ calibration setting */
	//RTPRINT(FINIT, INIT_IQK, ("IQK setting!\n"));
	urtwn_bb_write(dp, 0xe28, 0x80800000);
	urtwn_bb_write(dp, 0xe40, 0x01007c00);
	urtwn_bb_write(dp, 0xe44, 0x01004800);

	for (i = 0; i < retryCount; i++) {
		PathAOK = _PHY_PathA_IQK(dp, is2T);
		if (PathAOK == 0x03) {
			DPRINTF(0, (CE_CONT, "!%s: %s: Path A IQK Success!!",
			    dp->name, __func__));
			result[t][0] = (urtwn_bb_read(dp, 0xe94) & 0x3FF0000)
			    >> 16;
			result[t][1] = (urtwn_bb_read(dp, 0xe9c) & 0x3FF0000)
			    >> 16;
			result[t][2] = (urtwn_bb_read(dp, 0xea4) & 0x3FF0000)
			    >> 16;
			result[t][3] = (urtwn_bb_read(dp, 0xeac) & 0x3FF0000)
			    >> 16;
			break;

		} else if (i == (retryCount - 1) && PathAOK == 0x01) {
			/* Tx IQK OK */
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: Path A IQK Only  Tx Success!!",
			    dp->name, __func__));
			result[t][0] =
			    (urtwn_bb_read(dp, 0xe94) & 0x3FF0000) >> 16;
			result[t][1] =
			    (urtwn_bb_read(dp, 0xe9c) & 0x3FF0000) >> 16;
		}
	}

	if (PathAOK == 0) {
		cmn_err(CE_NOTE, "!%s: %s: Path A IQK failed!!",
		    dp->name, __func__);
	}

	if (is2T) {
		_PHY_PathAStandBy(dp);

		/*  Turn Path B ADDA on */
		_PHY_PathADDAOn(dp, ADDA_REG, B_FALSE, is2T);

		for (i = 0 ; i < retryCount ; i++) {
			PathBOK = _PHY_PathB_IQK(dp);
			if (PathBOK == 0x03) {
				DPRINTF(0, (CE_CONT,
				    "%s: %s: Path B IQK Success!!",
				     dp->name, __func__));
				result[t][4] = (urtwn_bb_read(dp, 0xeb4)
				    & 0x3FF0000) >> 16;
				result[t][5] = (urtwn_bb_read(dp, 0xebc)
				    & 0x3FF0000) >> 16;
				result[t][6] = (urtwn_bb_read(dp, 0xec4)
				    & 0x3FF0000) >> 16;
				result[t][7] = (urtwn_bb_read(dp, 0xecc)
				    & 0x3FF0000) >> 16;
				break;
			}

			if (i == (retryCount - 1) && PathBOK == 0x01) {
				/* Tx IQK OK */
				DPRINTF(0, (CE_CONT,
				    "!%s: %s: Path B Only Tx IQK Success!!",
				    dp->name, __func__));
				result[t][4] = (urtwn_bb_read(dp,
				    0xeb4) & 0x3FF0000) >> 16;
				result[t][5] = (urtwn_bb_read(dp,
				    0xebc) & 0x3FF0000) >> 16;
			}
		}

		if (PathBOK == 0) {
			    cmn_err(CE_NOTE, "!%s: %s: Path B IQK failed!!",
			    dp->name, __func__);
		}
	}

	/* Back to BB mode, load original value */
	urtwn_bb_write(dp, 0xe28, 0);

	if (t != 0) {
		if (!lp->bRfPiEnable) {
			/*
			 * Switch back BB to SI mode after finish
			 * IQ Calibration.
			 */
			_PHY_PIModeSwitch(dp, B_FALSE);
	        }

		/* Reload ADDA power saving parameters */
	 	_PHY_ReloadADDARegisters(dp,
		    ADDA_REG, lp->ADDA_backup, NITEMS(ADDA_REG));

		/* Reload MAC parameters */
		_PHY_ReloadMACRegisters(dp,
		    IQK_MAC_REG, lp->IQK_MAC_backup);

	 	/* Reload BB parameters */
	 	_PHY_ReloadADDARegisters(dp,
		    IQK_BB_REG, lp->IQK_BB_backup, NITEMS(IQK_BB_REG));

		/* Restore RX initial gain */
		urtwn_bb_write(dp, 0x840, 0x00032ed3);
		if (is2T) {
			urtwn_bb_write(dp, 0x844, 0x00032ed3);
	        }

		/* load 0xe30 IQC default value */
		urtwn_bb_write(dp, 0xe30, 0x01008c00);
		urtwn_bb_write(dp, 0xe34, 0x01008c00);
	}

	DPRINTF(0, (CE_CONT, "!%s: %s: return", dp->name, __func__));
}

static int
urtwn_iq_calib_chain(struct uwgem_dev *dp, int chain, uint16_t tx[2],
    uint16_t rx[2])
{
	struct urtwn_dev *lp = dp->private;
	uint32_t status;
	int offset = chain * 0x20;

	if (chain == 0) {	/* IQ calibration for chain 0. */
		/* IQ calibration settings for chain 0. */
		urtwn_bb_write(dp, 0xe30, 0x10008c1f);
		urtwn_bb_write(dp, 0xe34, 0x10008c1f);
		urtwn_bb_write(dp, 0xe38, 0x82140102);

		if (lp->ntxchains > 1) {
			urtwn_bb_write(dp, 0xe3c, 0x28160202);	/* 2T */
			/* IQ calibration settings for chain 1. */
			urtwn_bb_write(dp, 0xe50, 0x10008c22);
			urtwn_bb_write(dp, 0xe54, 0x10008c22);
			urtwn_bb_write(dp, 0xe58, 0x82140102);
			urtwn_bb_write(dp, 0xe5c, 0x28160202);
		} else
			urtwn_bb_write(dp, 0xe3c, 0x28160502);	/* 1T */

		/* LO calibration settings. */
		urtwn_bb_write(dp, 0xe4c, 0x001028d1);
		/* We're doing LO and IQ calibration in one shot. */
		urtwn_bb_write(dp, 0xe48, 0xf9000000);
		urtwn_bb_write(dp, 0xe48, 0xf8000000);

	} else {		/* IQ calibration for chain 1. */
		/* We're doing LO and IQ calibration in one shot. */
		urtwn_bb_write(dp, 0xe60, 0x00000002);
		urtwn_bb_write(dp, 0xe60, 0x00000000);
	}

	/* Give LO and IQ calibrations the time to complete. */
	drv_usecwait(1000);

	/* Read IQ calibration status. */
	status = urtwn_bb_read(dp, 0xeac);

	if (status & (1 << (28 + chain * 3))) {
		return (0);	/* Tx failed. */
	}

	/* Read Tx IQ calibration results. */
	tx[0] = (urtwn_bb_read(dp, 0xe94 + offset) >> 16) & 0x3ff;
	tx[1] = (urtwn_bb_read(dp, 0xe9c + offset) >> 16) & 0x3ff;
	if (tx[0] == 0x142 || tx[1] == 0x042) {
		return (0);	/* Tx failed. */
	}

	if (status & (1 << (27 + chain * 3))) {
		return (1);	/* Rx failed. */
	}

	/* Read Rx IQ calibration results. */
	rx[2] = (urtwn_bb_read(dp, 0xea4 + offset) >> 16) & 0x3ff;
	rx[3] = (urtwn_bb_read(dp, 0xeac + offset) >> 16) & 0x3ff;
	if (rx[2] == 0x132 || rx[3] == 0x036) {
		return (1);	/* Rx failed. */
	}

	return (3);	/* Both Tx and Rx succeeded. */
}

static void
urtwn_iq_calib(struct uwgem_dev *dp, boolean_t bReCovery)
{
	struct urtwn_dev *lp = dp->private;
	static uint32_t	IQK_BB_REG[] = {
		rOFDM0_XARxIQImbalance,	rOFDM0_XBRxIQImbalance,
		rOFDM0_ECCAThreshold,	rOFDM0_AGCRSSITable,
		rOFDM0_XATxIQImbalance,	rOFDM0_XBTxIQImbalance,
		rOFDM0_XCTxIQImbalance,	rOFDM0_XCTxAFE,
		rOFDM0_XDTxAFE,	rOFDM0_RxIQExtAnta,
	};
	int		result[4][8];	/* last is final result */
	uint8_t		i, final_candidate;
	boolean_t	bPathAOK, bPathBOK;
	uint32_t	RegE94, RegE9C, RegEA4, RegEAC;
	uint32_t	RegEB4, RegEBC, RegEC4, RegECC;
	uint_t		RegTmp = 0;
	boolean_t	is12simular, is13simular, is23simular;
	boolean_t	bStartContTx = B_FALSE, bSingleTone = B_FALSE;
#if (MP_DRIVER == 1)
	bStartContTx = pAdapter->MptCtx.bStartContTx;
#endif

	/* ignore IQK when continuous Tx */
	if (bStartContTx) {
		return;
	}

#if DISABLE_BB_RF
	return;
#endif

	ASSERT(NITEMS(IQK_BB_REG) == 10);
	if (bReCovery) {
		_PHY_ReloadADDARegisters(dp,
		    IQK_BB_REG, lp->IQK_BB_backup, NITEMS(IQK_BB_REG));
		return;
	}

	for (i = 0; i < 8; i++) {
		result[0][i] = 0;
		result[1][i] = 0;
		result[2][i] = 0;
		result[3][i] = 0;
	}

	final_candidate = 0xff;
	bPathAOK = B_FALSE;
	bPathBOK = B_FALSE;
	is12simular = B_FALSE;
	is23simular = B_FALSE;
	is13simular = B_FALSE;

	for (i = 0; i < 3; i++) {
	 	if (lp->ntxchains == 2) {
			 _PHY_IQCalibrate(dp, result, i, B_TRUE);
	 		//_PHY_DumpRFReg(dp);
	 	} else {
	 		// For 88C 1T1R
	 		_PHY_IQCalibrate(dp, result, i, B_FALSE);
 		}

		if (i == 1) {
			is12simular = _PHY_SimularityCompare(dp,
			    result, 0, 1);
			if (is12simular) {
				final_candidate = 0;
				break;
			}
		}

		if (i == 2) {
			is13simular = _PHY_SimularityCompare(dp,
			    result, 0, 2);
			if (is13simular) {
				final_candidate = 0;
				break;
			}

			is23simular = _PHY_SimularityCompare(dp,
			    result, 1, 2);
			if (is23simular) {
				final_candidate = 1;
			} else {
				for (i = 0; i < 8; i++) {
					RegTmp += result[3][i];
				}

				if (RegTmp != 0) {
					final_candidate = 3;
				} else {
					final_candidate = 0xFF;
				}
			}
		}
	}

        for (i = 0; i < 4; i++) {
		RegE94 = result[i][0];
		RegE9C = result[i][1];
		RegEA4 = result[i][2];
		RegEAC = result[i][3];
		RegEB4 = result[i][4];
		RegEBC = result[i][5];
		RegEC4 = result[i][6];
		RegECC = result[i][7];
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: IQK: "
		    "RegE94=%x RegE9C=%x RegEA4=%x RegEAC=%x "
		    "RegEB4=%x RegEBC=%x RegEC4=%x RegECC=%x",
		    dp->name, __func__,
		    RegE94, RegE9C, RegEA4, RegEAC,
		    RegEB4, RegEBC, RegEC4, RegECC));
	}

	if (final_candidate != 0xff) {
		lp->RegE94 = RegE94 = result[final_candidate][0];
		lp->RegE9C = RegE9C = result[final_candidate][1];
		RegEA4 = result[final_candidate][2];
		RegEAC = result[final_candidate][3];
		lp->RegEB4 = RegEB4 = result[final_candidate][4];
		lp->RegEBC = RegEBC = result[final_candidate][5];
		RegEC4 = result[final_candidate][6];
		RegECC = result[final_candidate][7];

		DPRINTF(0, (CE_CONT, "!%s: IQK: final_candidate is %x",
		    dp->name, final_candidate));

		DPRINTF(0, (CE_CONT,
		    "!%s: %s: IQK: "
		    "RegE94=%x RegE9C=%x RegEA4=%x RegEAC=%x "
		    "RegEB4=%x RegEBC=%x RegEC4=%x RegECC=%x ",
		    dp->name, __func__,
		    RegE94, RegE9C, RegEA4, RegEAC,
		    RegEB4, RegEBC, RegEC4, RegECC));

		bPathAOK = bPathBOK = B_TRUE;
	} else {
		lp->RegE94 = lp->RegEB4 = 0x100;	/* X default value */
		lp->RegE9C = lp->RegEBC = 0x0;	/* Y default value */
	}

	if (RegE94 != 0) {
		_PHY_PathAFillIQKMatrix(dp,
		    bPathAOK, result, final_candidate, (RegEA4 == 0));
	}

	if (lp->ntxchains == 2) {
		if (RegEB4 != 0) {
			_PHY_PathBFillIQKMatrix(dp,
			    bPathBOK, result, final_candidate, (RegEC4 == 0));
		}
	}

	_PHY_SaveADDARegisters(dp,
	    IQK_BB_REG, lp->IQK_BB_backup, 10);
}

static void
urtwn_lc_calib(struct uwgem_dev *dp)		/* OK */
{
	struct urtwn_dev	*lp = dp->private;
	uint32_t	rf_ac[2];
	uint8_t	txmode;
	int	i;

	/* Check continuous TX and Packet TX */
	txmode = urtwn_read_1(dp, R92C_OFDM1_LSTF + 3);
	if ((txmode & 0x70) != 0) {
		DPRINTF(0, (CE_CONT, "!%s: %s: Deal with contisuous TX case",
		    dp->name, __func__));

		/* Disable all continuous Tx. */
		urtwn_write_1(dp, R92C_OFDM1_LSTF + 3, txmode & ~0x70);

		/* Set RF mode to standby mode. */
		for (i = 0; i < lp->nrxchains; i++) {
			rf_ac[i] = urtwn_rf_read(dp, i, R92C_RF_AC);
		}
#ifdef BUGFIX_OPENBSD
		for (i = 0; i < lp->nrxchains; i++) {
			urtwn_rf_write(dp, i, R92C_RF_AC,
			    RW(rf_ac[i], R92C_RF_AC_MODE,
			    R92C_RF_AC_MODE_STANDBY));
		}
#else
		/* the original driver in *BSD writes nothing. bug? */
#endif
	} else {
		DPRINTF(0, (CE_CONT, "!%s: %s: Deal with Packet TX case",
		    dp->name, __func__));

		/* Block all Tx queues. */
		urtwn_write_1(dp, R92C_TXPAUSE, 0xffU);
	}
	/* Start calibration. */
#ifdef NEVER /* COMPAT_L */
	(void) urtwn_rf_read(dp, 0, R92C_RF_CHNLBW);
#endif
	urtwn_rf_write(dp, 0, R92C_RF_CHNLBW,
	    urtwn_rf_read(dp, 0, R92C_RF_CHNLBW) | R92C_RF_CHNLBW_LCSTART);

	/* Give calibration the time to complete. */
	delay(drv_usectohz(100*1000));

	/* Restore configuration. */
	if ((txmode & 0x70) != 0) {
		/* Restore Tx mode. */
		urtwn_write_1(dp, R92C_OFDM1_LSTF + 3, txmode);

		/* Restore RF mode. */
		for (i = 0; i < lp->nrxchains; i++) {
			urtwn_rf_write(dp, i, R92C_RF_AC, rf_ac[i]);
		}
	} else {
		/* Unblock all Tx queues. */
		urtwn_write_1(dp, R92C_TXPAUSE, 0x00);
	}
}

static void
urtwn_temp_calib(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	int temp;

	if (lp->thcal_state == 0) {
		/* Start measuring temperature. */
		urtwn_rf_write(dp, 0, R92C_RF_T_METER, 0x60);
		lp->thcal_state = 1;
		return;
	}
	lp->thcal_state = 0;

	/* Read measured temperature. */
	temp = urtwn_rf_read(dp, 0, R92C_RF_T_METER) & 0x1f;
	if (temp == 0) {
		/* Read failed, skip. */
		return;
	}
	DPRINTF(3, (CE_CONT, "!%s: %s: temperature=%d",
	     dp->name, __func__, temp));

	/*
	 * Redo LC calibration if temperature changed significantly since
	 * last calibration.
	 */
	if (lp->thcal_lctemp == 0) {
		/* First LC calibration is performed in urtwn_init(). */
		lp->thcal_lctemp = temp;
	} else if (ABS(temp - lp->thcal_lctemp) > 1) {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: LC calib triggered by temp: %d -> %d",
		    dp->name, __func__,
		    lp->thcal_lctemp, temp));
		urtwn_lc_calib(dp);
		/* Record temperature of last LC calibration. */
		lp->thcal_lctemp = temp;
	}
}

/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
urtwn_reset_chip(struct uwgem_dev *dp, uint_t flags)
{
	int	error = USB_SUCCESS;
	struct urtwn_dev	*lp = dp->private;

	return (error);
}

static int
urtwn_init_chip(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	struct urtwn_rx_data *data;
	uint32_t reg;
	int	i;
	int	error = USB_SUCCESS;

	/* from rtw_drv_init() in os_dep/linux/usb_intf.c as drv_probe */

	/* Init firmware commands ring. */
	lp->fwcur = 0;

	/* from rtl8192cu_hal_init() */

	/* Power on adapter. */
	error = urtwn_power_on(dp);
	if (error != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: failed to reset the device",
		    dp->name, __func__);
		goto fail;
	}

	/* Initialize DMA. */
	error = urtwn_dma_init(dp);
	if (error != USB_SUCCESS) {
		goto fail;
	}

	/*
	 * InitDriverInfoSize()
	 */
	/* Set info size in Rx descriptors (in 64-bit words). */
	urtwn_write_1(dp, R92C_RX_DRVINFO_SZ, 4);

	/*
	 * InitInterrupt()
	 */
	/* Init interrupts. */
	urtwn_write_4(dp, R92C_HISR, 0xffffffffU);
	urtwn_write_4(dp, R92C_HIMR, 0xffffffffU);

	/*
	 * InitID()
	 */
	/* Set MAC address. */
	for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
		urtwn_write_1(dp, R92C_MACID + i, ic->ic_macaddr[i]);
	}

	/*
	 * InitNetworkType()
	 */
	/* Set initial network type. */
	reg = urtwn_read_4(dp, R92C_CR);

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
	default:
		reg = RW(reg, R92C_CR_NETTYPE, R92C_CR_NETTYPE_INFRA);
		break;

	case IEEE80211_M_IBSS:
		reg = RW(reg, R92C_CR_NETTYPE, R92C_CR_NETTYPE_ADHOC);
		break;
	}

	urtwn_write_4(dp, R92C_CR, reg);

	/*
	 * InitWMACSetting()
	 */
	urtwn_rxfilter_init(dp);

	/*
	 * InitAdaptiveCtrl()
	 */
	reg = urtwn_read_4(dp, R92C_RRSR);
	reg = RW(reg, R92C_RRSR_RATE_BITMAP, R92C_RRSR_RATE_CCK_ONLY_1M);
	urtwn_write_4(dp, R92C_RRSR, reg);

        /* SIFS (used in NAV) */
        urtwn_write_2(dp, R92C_SPEC_SIFS,
            SM(R92C_SPEC_SIFS_CCK, 0x10) | SM(R92C_SPEC_SIFS_OFDM, 0x10));

	/* Set short/long retry limits. */
	urtwn_write_2(dp, R92C_RL,
	    SM(R92C_RL_SRL, 0x30) | SM(R92C_RL_LRL, 0x30));

	/*
	 * InitEDCA
	 */
	/* Initialize EDCA parameters. */
	urtwn_edca_init(dp);

	/*
	 * _InitRateFallBack()
	 */
	/* Setup rate fallback. */
	urtwn_write_4(dp, R92C_DARFRC + 0, 0x00000000);
	urtwn_write_4(dp, R92C_DARFRC + 4, 0x10080404);
	urtwn_write_4(dp, R92C_RARFRC + 0, 0x04030201);
	urtwn_write_4(dp, R92C_RARFRC + 4, 0x08070605);

	/*
	 * InitRetryFunction()
	 */
	urtwn_write_1(dp, R92C_FWHW_TXQ_CTRL,
	    urtwn_read_1(dp, R92C_FWHW_TXQ_CTRL) |
	    R92C_FWHW_TXQ_CTRL_AMPDU_RTY_NEW);
	/* Set ACK timeout. */
	urtwn_write_1(dp, R92C_ACKTO, 0x40);

	/*
	 * InitUsbAggregationSetting()
	 */
	/* Setup USB aggregation. */
	/* tx */
	reg = urtwn_read_4(dp, R92C_TDECTRL);
	reg = RW(reg, R92C_TDECTRL_BLK_DESC_NUM, 6);
	urtwn_write_4(dp, R92C_TDECTRL, reg);

	/* rx */
	urtwn_write_1(dp, R92C_TRXDMA_CTRL,
	    urtwn_read_1(dp, R92C_TRXDMA_CTRL) |
	    R92C_TRXDMA_CTRL_RXDMA_AGG_EN);
#ifdef BUGFIX_OPENBSD /* COMPAT_L */
	urtwn_write_1(dp, R92C_USB_SPECIAL_OPTION,
	    urtwn_read_1(dp, R92C_USB_SPECIAL_OPTION) &
	    ~R92C_USB_SPECIAL_OPTION_AGG_EN);
#else
	/* XXX - usb rx aggrigation doesn't work */
	urtwn_write_1(dp, R92C_USB_SPECIAL_OPTION,
	    urtwn_read_1(dp, R92C_USB_SPECIAL_OPTION) |
	    R92C_USB_SPECIAL_OPTION_AGG_EN);
#endif

	urtwn_write_1(dp, R92C_RXDMA_AGG_PG_TH, 48);
	urtwn_write_1(dp, R92C_USB_DMA_AGG_TO, 4);
#ifndef BUGFIX_OPENBSD /* COMPAT_L */
	/* not required */
	urtwn_write_1(dp, R92C_USB_AGG_TH, 8);
	urtwn_write_1(dp, R92C_USB_AGG_TO, 6);
#endif
	/*
	 * InitBeaconParameters()
	 */
	/* Initialize beacon parameters. */
	urtwn_write_2(dp, R92C_TBTT_PROHIBIT, 0x6404);
	urtwn_write_1(dp, R92C_DRVERLYINT, 0x05);
	urtwn_write_1(dp, R92C_BCNDMATIM, 0x02);
	urtwn_write_2(dp, R92C_BCNTCFG, 0x660f);

	/*
	 * InitAMPDUAddregation
	 */
	/* Setup AMPDU aggregation. */
	urtwn_write_4(dp, R92C_AGGLEN_LMT, 0x99997631U); /* MCS7~0 */
	urtwn_write_1(dp, R92C_AGGR_BREAK_TIME, 0x16);
	urtwn_write_2(dp, 0x4ca, 0x0708);

	/*
	 * InitBeaconMaxError()
	 */
	urtwn_write_1(dp, R92C_BCN_MAX_ERR, 0xffU);

	/*
	 * BeaconFunctionEnable()
	 */
	urtwn_write_1(dp, R92C_BCN_CTRL, R92C_BCN_CTRL_DIS_TSF_UDT0);

	/*
	 * FirmwareDownload92C()
	 */
	/* Load 8051 microcode. */
	error = urtwn_load_firmware(dp);
	if (error != USB_SUCCESS) {
		goto fail;
	}
	lp->fwready = B_TRUE;

#ifdef notyet
	/*
	 * InitRFType() not implemented yet
	 */
        pHalData->rf_chip       = RF_6052;

        if (pregpriv->rf_config != RF_819X_MAX_TYPE) {
                pHalData->rf_type = pregpriv->rf_config;
                DPRINTF(0, ("Set RF Chip ID to RF_6052 and RF type to %d.",
		    pHalData->rf_type);
                return;
        }

        if (lp->ntxchain2 == 2)) {
                pHalData->rf_type = RF_1T2R;
                DPRINTF(0, ("Set RF Chip ID to RF_6052 and RF type to 1T2R."));

        } else if (IS_92C_SERIAL(pHalData->VersionID)) {
                pHalData->rf_type = RF_2T2R;
                DPRINTF(0, ("Set RF Chip ID to RF_6052 and RF type to 2T2R."));
                //return;
        } else {
                pHalData->rf_type = RF_1T1R;
                DPRINTF(0, ("Set RF Chip ID to RF_6052 and RF type to 1T1R."));
        }
#endif
	/*
	 * PHY_MACConfig8192C()
	 */
	/* Initialize MAC/BB/RF blocks. */
	urtwn_mac_init(dp);

	urtwn_write_4(dp, R92C_RCR,
	    urtwn_read_4(dp, R92C_RCR) & ~R92C_RCR_ADF);

	/* PHY_BBConfig8192C */
	urtwn_bb_init(dp);

	/* PHY_RFConfig8192C */
	urtwn_rf_init(dp);

	/* Turn CCK and OFDM blocks on. */
	urtwn_bb_write(dp, R92C_FPGA0_RFMOD,
	    urtwn_bb_read(dp, R92C_FPGA0_RFMOD) | R92C_RFMOD_CCK_EN);

	urtwn_bb_write(dp, R92C_FPGA0_RFMOD,
	    urtwn_bb_read(dp, R92C_FPGA0_RFMOD) | R92C_RFMOD_OFDM_EN);

	/* Clear per-station keys table. */
	urtwn_cam_init(dp);	/* OK */

	/* Enable hardware sequence numbering. */
	urtwn_write_1(dp, R92C_HWSEQ_CTRL, 0xff);	/* OK */

	/* Perform LO and IQ calibrations. */
#ifdef CONFIG_IQ_CALIB
	urtwn_iq_calib(dp, lp->iqk_initialized);	/* NG */
	lp->iqk_initialized = B_TRUE;
#endif

	/* Perform LC calibration. */
	urtwn_lc_calib(dp);			/* NG */

	/* Fix USB interference issue. */
	urtwn_write_1(dp, 0xfe40, 0xe0);	/* OK */
	urtwn_write_1(dp, 0xfe41, 0x8d);	/* OK */
	urtwn_write_1(dp, 0xfe42, 0x80);	/* OK */
	urtwn_write_4(dp, 0x20c, 0xfd0320);	/* OK */

	urtwn_pa_bias_init(dp);

	if ((lp->chip & URTWN_CHIP_92C_1T2R) == 0) {
		/* 1T1R */
		urtwn_write_4(dp, R92C_LEDCFG0,
		    urtwn_read_4(dp, R92C_LEDCFG0) | BIT(23));

		urtwn_write_4(dp, R92C_FPGA0_RFPARAM(0),
		    urtwn_read_4(dp, R92C_FPGA0_RFPARAM(0)) | BIT(13));
	}

	/*
	 * from rtl8192c_InitHalDm() in hal/92c/rtl8192c_dm.c
	 */
	/* Initialize GPIO setting. */
	urtwn_write_1(dp, R92C_GPIO_MUXCFG,
	    urtwn_read_1(dp, R92C_GPIO_MUXCFG) & ~R92C_GPIO_MUXCFG_ENBT);	/* OK */

	/* Fix for lower temperature. */
	urtwn_write_1(dp, 0x15, 0xe9);	/* OK */
#if 1
	/* Set default channel. */
	ic->ic_bss->in_chan = ic->ic_ibss_chan;
	urtwn_set_chan(dp, ic->ic_ibss_chan, IEEE80211_HTINFO_2NDCHAN_NONE);
#endif
#ifdef notyet
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			urtwn_set_key(ic, NULL, &ic->ic_nw_keys[i]);
		urtwn_wait_async(sc);
	}
#endif
fail:
	return (error);
}

static int
urtwn_start_chip(struct uwgem_dev *dp)
{
	return (USB_SUCCESS);
}

static int
urtwn_stop_chip(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	int	forced = 1;
	uint8_t	value8;
	uint16_t	value16;
	uint32_t	value32;
        uint8_t	eRFPath = 0;
	uint8_t	retry_cnts = 0;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));
#ifdef BUGFIX_OPENBSD
	/*
	 * RF Off Sequence
	 *	from _DisableRFAFEAndResetBB(Adapter)
	 */
	/* TXPAUSE 0x522[7:0] = 0xFF : Pause MAC TX queue */
        urtwn_write_1(dp, R92C_TXPAUSE, 0xFF);

	/* RF path 0 offset 0x00 = 0x00 : disable RF */
	urtwn_rf_write(dp, 0, 0, 0);

	/* APSD_CTRL 0x600[7:0] = 0x40 */
        urtwn_write_1(dp, R92C_APSD_CTRL, R92C_APSD_CTRL_OFF);

	/* SYS_FUNC_EN 0x02[7:0] = 0x16 : reset BB state machine */
        urtwn_write_1(dp, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBD |
	    R92C_SYS_FUNC_EN_USBA |
	    R92C_SYS_FUNC_EN_BB_GLB_RST);

	/* SYS_FUNC_EN 0x02[7:0] = 0x14 : reset BB state machine */
        value8 &= (~R92C_SYS_FUNC_EN_BB_GLB_RST);
        urtwn_write_1(dp, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBD | R92C_SYS_FUNC_EN_USBA);

	/*
	 * Reset digital sequence
	 *	from _ResetDigitalProcedure1(Adapter, B_TRUE)
	 */
	if (urtwn_read_1(dp, R92C_MCUFWDL) & BIT(1)) {

		//IF fw in RAM code, do reset

		/* reset MCU ready status */
		urtwn_write_1(dp, R92C_MCUFWDL, 0);

		if (lp->fwready) {
			/* 8051 reset by self */
			urtwn_write_1(dp, R92C_HMETFR + 3, 0x20);

			for (retry_cnts = 0;
			    (urtwn_read_2(dp, R92C_SYS_FUNC_EN) &
			    R92C_SYS_FUNC_EN_CPUEN);
			    retry_cnts++) {
				if (retry_cnts >= 100) {
					/* timeout */
					cmn_err(CE_WARN,
					    "!%s: %s: 8051 reset failed!",
					    dp->name, __func__);
					urtwn_write_1(dp,
					    R92C_SYS_FUNC_EN + 1, 0x50);
					delay(drv_usectohz(10*1000));
					break;
				}
				delay(drv_usectohz(50*1000));
			}
			lp->fwready = B_FALSE;
		}
	}

	/* Reset MAC and Enable 8051 */
	urtwn_write_1(dp, R92C_SYS_FUNC_EN + 1, 0x54);

	/* reset MCU ready status */
	urtwn_write_1(dp, R92C_MCUFWDL, 0);

	if (forced) {
		/* SYS_CLKR 0x08[15:0] = 0x30A3 : disable MAC clock */
		urtwn_write_2(dp, R92C_SYS_CLKR, 0x70A3);

		/* AFE_PLL_CTRL 0x28[7:0] = 0x80 : disable AFE PLL */
		urtwn_write_1(dp, R92C_AFE_PLL_CTRL, 0x80);

		/* AFE_XTAL_CTRL 0x24[15:0] = 0x880F : gated AFE DIG_CLOCK */
		urtwn_write_2(dp, R92C_AFE_XTAL_CTRL, 0x880F);

		/* SYS_ISO_CTRL 0x00[7:0] = 0xF9 : isolated digital to PON */
		urtwn_write_1(dp, R92C_SYS_ISO_CTRL, 0xF9);
	}

	/*
	 * Pull GPIO PIN to balance level and LED control
	 *	from _DisableGPIO(Adapter)
	 */

	/* 1. Disable GPIO[7:0] */

	/*  GPIO_PIN_CTRL 0x44[31:0]=0x000 */
	urtwn_write_2(dp, R92C_GPIO_PIN_CTRL + 2, 0x0000);

	/*  Value = GPIO_PIN_CTRL[7:0] */
	value32 = urtwn_read_4(dp, R92C_GPIO_PIN_CTRL) & ~0x0000ff00;
	value32 |= ((value32 << 8) & 0x0000ff00) | 0x00ff0000;
	/*  GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value << 8) */
	urtwn_write_4(dp, R92C_GPIO_PIN_CTRL, value32);

        /* Disable GPIO[10:8] */
	/*  GPIO_MUXCFG 0x42 [15:0] = 0x0780 */
        urtwn_write_1(dp, R92C_GPIO_MUXCFG + 3, 0x00);

	value16 = urtwn_read_2(dp, R92C_GPIO_MUXCFG + 2) & ~0x00f0;

        value16 |= (((value16 & 0x000f) << 4) | 0x0780);
        urtwn_write_2(dp, R92C_GPIO_PIN_CTRL+2, value16);

	/* LEDCFG 0x4C[15:0] = 0x8080 : Disable LED0 & 1 */
        urtwn_write_2(dp, R92C_LEDCFG0, 0x8080);


	/*
	 * Reset digital sequence
	 *	from _ResetDigitalProcedure2(Adapter);
	 */
	/* SYS_CLKR 0x08[15:0] = 0x70a3 : disable ELDR clock */
	urtwn_write_2(dp, R92C_SYS_CLKR, 0x70A3);

	/* SYS_ISO_CTRL 0x01[7:0] = 0x82 : isolated ELDR to PON */
	urtwn_write_1(dp, R92C_SYS_ISO_CTRL + 1, 0x82);

	/*
	 * Disable analog sequence
	 *	_DisableAnalog(Adapter, B_TRUE);
	 */
        if (forced) {
		/* LDOA15_CTRL 0x20[7:0] = 0x04 : disable A15 power */
                urtwn_write_1(dp, R92C_LDOA15_CTRL, 0x04);

		/* LDOV12D_CTRL 0x21[7:0] = 0x54:disable digital core power */
                urtwn_write_1(dp, R92C_LDOV12D_CTRL,
                    urtwn_read_1(dp, R92C_LDOV12D_CTRL) &
		    ~R92C_LDOV12D_CTRL_LDV12_EN);
        }

	/* SPS0_CTRL 0x11[7:0] = 0x23 : enter PFM mode */
	urtwn_write_1(dp, R92C_SPS0_CTRL, 0x23);

	/* APS_FSMCO 0x04[15:0] = 0x4802 : set USB suspend */
	urtwn_write_2(dp, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_APDM_HOST | R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_PFM_ALDN);

	urtwn_write_1(dp, R92C_RSV_CTRL, 0x0E);
#endif /* BUGFIX_OPENBSD */
	return (USB_SUCCESS);
}

static int
urtwn_set_rx_filter(struct uwgem_dev *dp)
{
	int	error = USB_SUCCESS;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	return (error);
}

static int
urtwn_get_stats(struct uwgem_dev *dp)
{
#if 1
	urtwn_calib_to(dp, NULL);
#endif
	return (USB_SUCCESS);
}

/*
 * packet transmittion
 */
static mblk_t *
urtwn_tx_make_packet(struct uwgem_dev *dp, mblk_t *mp,
    uint_t type, struct ieee80211_node *ni)
{
	struct urtwn_dev 	*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_frame	*wh;
	struct ieee80211_key	*k = NULL;
	struct r92c_tx_desc	*txd;
	uint16_t	qos;
	uint16_t	sum;
	uint8_t		raid;
	uint8_t		tid = 0;
	uint8_t		ac;
	int		i;
	int		hasqos;
	int		pktlen;
	int		mblen;
	int		pad;
	mblk_t		*m;
	mblk_t		*m0;
	uint8_t		*bp;
	uint_t		ht40m;
	uint_t		mac_id;
	boolean_t	ampdu = B_FALSE;


	pad = 8;
	pktlen = msgdsize(mp);
	m = allocb(sizeof (*txd) + pktlen + 64, BPRI_MED);
	if (m == NULL) {
		cmn_err(0, (CE_WARN,
		    "!%s: %s: can't alloc mblk", dp->name, __func__));
		dp->stats.noxmtbuf++;
		return (NULL);
	}
	ASSERT(m->b_cont == NULL);

	m->b_rptr += sizeof (*txd) + pad;
	bp = m->b_rptr;

	/* copy the entire payload */
	for (m0 = mp; m0 != NULL; m0 = m0->b_cont) {
		mblen = MBLKL(m0);
		(void) bcopy(m0->b_rptr, bp, mblen);
		bp += mblen;
	}
	m->b_wptr = bp;

	wh = (struct ieee80211_frame *)m->b_rptr;

	if (type == IEEE80211_FC0_TYPE_DATA) {
		(void) ieee80211_encap(ic, m, ni);
	}
	else if (type == IEEE80211_FC0_TYPE_MGT) {
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

			/* fix start seq */
			baseq = ((ni->in_txseqs[t] + 1)
			    << IEEE80211_BASEQ_START_S)
			    & IEEE80211_BASEQ_START;
			cp[7] = (uint8_t)baseq;
			cp[8] = (uint8_t)(baseq >> 8);
		}
	}

	/* encryption */
	if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		struct ieee80211_key *k;
		k = ieee80211_crypto_encap(ic, m);
		if (k == NULL) {
			freemsg(m);
			m = NULL;
			return (NULL);
		}
		ASSERT(wh == (void *)m->b_rptr);
	}

	/* ensure the tx packet is single fragment */
	if (m->b_cont) {
		cmn_err(CE_CONT, "!%s: b_cont != NULL", __func__);
		bp = m->b_wptr;
		for (m0 = m->b_cont; m0 != NULL; m0 = m0->b_cont) {
			mblen = MBLKL(m0);
			(void) bcopy(m0->b_rptr, bp, mblen);
			bp += mblen;
		}
		m->b_wptr = bp;
		freemsg(m->b_cont);
		m->b_cont = NULL;
	}

	hasqos = IEEE80211_QOS_HAS_SEQ(wh);
	if (type != IEEE80211_FC0_TYPE_DATA) {
		/* Use AC_VO for management frames. */
		ac = WME_AC_VO;
	} else if (hasqos) {
		/* data frames in 11n mode */
		struct ieee80211_qosframe	*qwh = (void *)wh;

		tid = qwh->i_qos[0] & IEEE80211_QOS_TID;
		ac = TID_TO_WME_AC(tid);

		if ((qwh->i_qos[0] & IEEE80211_QOS_ACKPOLICY_BA) ==
		    IEEE80211_QOS_ACKPOLICY_BA) {

			/* enable ampdu for the packet */
			ampdu = B_TRUE;

			/* clear software mark of ampdu */ /* required */
			qwh->i_qos[0] &= ~IEEE80211_QOS_ACKPOLICY_BA;
		}
	} else {
		/* non-qos data frames */
		ac = WME_AC_BE;
	}

	/* save the USB pipe to use for this AC. */
	m->b_datap->db_cksum16 = lp->ac2idx[ac];

	/* fill tx descriptor. */
	pktlen = MBLKL(m);
	if (((sizeof (*txd) + pktlen) & (lp->pktsize - 1)) != 0) {
		/* trick, pad is *not* required */
		pad = 0;
	}

	/*
	 * find *correct* base as usb packet
	 */
	m->b_rptr -= (sizeof (*txd) + pad);

	/*
	 * make tx desc
	 */
	txd = (struct r92c_tx_desc *)m->b_rptr;
	memset(txd, 0, sizeof (*txd) + pad);

	txd->txdw0 |= htole32(
	    SM(R92C_TXDW0_PKTLEN, pktlen) |
	    SM(R92C_TXDW0_OFFSET, (sizeof (*txd))) |
	    R92C_TXDW0_OWN | R92C_TXDW0_FSG | R92C_TXDW0_LSG);

	mac_id = URTWN_MACID_BSS;
	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		txd->txdw0 |= htole32(R92C_TXDW0_BMCAST);
		mac_id = URTWN_MACID_BC;
	}

	/* fix pad field */
	if (pad) {
		txd->txdw1 |= htole32(SM(R92C_TXDW1_PKTOFF, (pad / 8)));
	}
#ifdef notyet
	if (k != NULL) {
		switch (k->k_cipher) {
		case IEEE80211_CIPHER_WEP40:
		case IEEE80211_CIPHER_WEP104:
		case IEEE80211_CIPHER_TKIP:
			cipher = R92C_TXDW1_CIPHER_RC4;
			break;

		case IEEE80211_CIPHER_CCMP:
			cipher = R92C_TXDW1_CIPHER_AES;
			break;

		default:
			cipher = R92C_TXDW1_CIPHER_NONE;
			break;
		}
		txd->txdw1 |= htole32(SM(R92C_TXDW1_CIPHER, cipher));
	}
#endif
	/*
	 * pickup a rate
	 */
	DPRINTF(10, (CE_CONT, "!%s: txrate:%d",
	    __func__, dp->txrate));
	if (IEEE80211_IS_MULTICAST(wh->i_addr1) ||
	    IEEE80211_ADDR_EQ(wh->i_addr1, wh->i_addr3) ||
	    type != IEEE80211_FC0_TYPE_DATA) {
		/* broadcast or multicast packets */
		txd->txdw1 |= htole32(
		    SM(R92C_TXDW1_MACID, URTWN_MACID_BC) |
		    SM(R92C_TXDW1_RAID, R92C_RAID_11B));

		if (type == IEEE80211_FC0_TYPE_MGT) {
			txd->txdw1 |=
			    htole32(SM(R92C_TXDW1_QSEL, R92C_TXDW1_QSEL_MGNT));
		}

		/* Force CCK1. */
		txd->txdw4 |= htole32(R92C_TXDW4_DRVRATE);

		/* Use 1Mbps */
		txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 0));

		/* XXX - no need to setup protection */
	}
	else if (type == IEEE80211_FC0_TYPE_DATA) {
		if (ic->ic_curmode == IEEE80211_MODE_11NG) {
			/* 11n 2.4GHz */
			raid = R92C_RAID_11BGN;
		} else if (ic->ic_curmode == IEEE80211_MODE_11B) {
			/* 11b */
			raid = R92C_RAID_11B;
		} else {
			/* 11g */
			raid = R92C_RAID_11BG;
		}

		txd->txdw1 |= htole32(
		    SM(R92C_TXDW1_MACID, mac_id) |
		    SM(R92C_TXDW1_QSEL, tid) |
		    SM(R92C_TXDW1_RAID, raid)
		    );

		if (ampdu) {
			txd->txdw1 |= htole32(R92C_TXDW1_AGGEN);
#ifdef notyet
			txd->txdw5 |= htole32(0x14U << 24);
#endif
		} else {
			txd->txdw1 |= htole32(R92C_TXDW1_AGGBK);
		}

		/* sqnum in txdw3 was moved */

		if (hasqos) {
			txd->txdw4 |= htole32(R92C_TXDW4_QOS);	/* BIT6 */
		}
#ifdef notyet
		/* ampdu density */
		txd->txdw4 |= htole32(7 << 20);
#endif
		/* full_txdesc_vcs */

		/*
		 * select band width for 802.11n
		 */
		ht40m = (ic->ic_curmode == IEEE80211_MODE_11NG &&
		    ni->in_chw == 40 &&
                    (ni->in_chan->ich_flags & IEEE80211_CHAN_HT40))
                    ? (ni->in_ht2ndchan & IEEE80211_HTINFO_2NDCHAN)
		    : IEEE80211_HTINFO_2NDCHAN_NONE;

		if (ht40m != IEEE80211_HTINFO_2NDCHAN_NONE) {
			txd->txdw4 |= htole32(R92C_TXDW4_40MHZ);
			if (ht40m == IEEE80211_HTINFO_2NDCHAN_ABOVE) {
				txd->txdw4 |= htole32(1U << R92C_TXDW4_SCO_S);
			} else if (ht40m == IEEE80211_HTINFO_2NDCHAN_BELOW) {
				txd->txdw4 |= htole32(2U << R92C_TXDW4_SCO_S);
			}
		}

		/*
		 * select protection
		 */
		if (ic->ic_curmode == IEEE80211_MODE_11B) {
			/* none */
		} else if (ic->ic_curmode == IEEE80211_MODE_11NG) {
#ifdef notdef
			if (ic->ic_htprotmode == IEEE80211_PROT_RTSCTS
			    /* || ampdu */) {
				txd->txdw4 |= htole32(
				    R92C_TXDW4_RTSEN |
				    R92C_TXDW4_HWRTSEN);
			}
#endif
			switch(ni->in_htopmode & IEEE80211_HTINFO_OPMODE) {
			case IEEE80211_HTINFO_OPMODE_PURE: /* 0 */
				if (ni->in_htopmode & IEEE80211_HTINFO_NONGF_PRESENT) {
					txd->txdw4 |= htole32(
					    R92C_TXDW4_RTSEN |
					    R92C_TXDW4_HWRTSEN);
				}
				break;

			case IEEE80211_HTINFO_OPMODE_PROTOPT: /* 1 */
				txd->txdw4 |= htole32(
				    R92C_TXDW4_RTSEN |
				    R92C_TXDW4_HWRTSEN);
				break;

			case IEEE80211_HTINFO_OPMODE_HT20PR: /* 2 */
				if (ht40m != IEEE80211_HTINFO_2NDCHAN_NONE ||
				    ((ni->in_htcap & IEEE80211_HTCAP_GREENFIELD)
				    && (ni->in_htopmode & IEEE80211_HTINFO_NONGF_PRESENT))) {
					txd->txdw4 |= htole32(
					    R92C_TXDW4_RTSEN |
					    R92C_TXDW4_HWRTSEN);
				}
				break;

			case IEEE80211_HTINFO_OPMODE_MIXED: /* 3 */
				txd->txdw4 |= htole32(
				    R92C_TXDW4_RTSEN |
				    R92C_TXDW4_HWRTSEN);
				break;
			}
		} else if (ic->ic_flags & IEEE80211_F_USEPROT) {
			/* for 11g */
			if (ic->ic_protmode == IEEE80211_PROT_RTSCTS) {
				txd->txdw4 |= htole32(
				    R92C_TXDW4_RTSEN |
				    R92C_TXDW4_HWRTSEN);
			} else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY) {
				txd->txdw4 |= htole32(
				    R92C_TXDW4_CTS2SELF |
				    R92C_TXDW4_HWRTSEN);
			}
		}

		/* Send RTS at OFDM24. */
		txd->txdw4 |= htole32(SM(R92C_TXDW4_RTSRATE, 8));

		txd->txdw5 |= htole32(0x0001ff00);
		txd->txdw5 |= htole32(0x0000000b);

		if (0) {
			/* for driver dbg */

			/* driver uses rate */
			txd->txdw4 |= htole32(R92C_TXDW4_DRVRATE);

			if (ni->in_flags & IEEE80211_NODE_HT) {
				//SGI
				txd->txdw5 |= htole32(R92C_TXDW5_SGI);
			}
			/* init rate - mcs7 */
			txd->txdw5 |= htole32(0x00000013 - 7);
		}
#ifdef noyet
        } else if (pxmitframe->frame_tag == TXAGG_FRAMETAG) {
                DPRINTF(0, ("pxmitframe->frame_tag == TXAGG_FRAMETAG"));
#endif
	}

	/* Set sequence number */
	sum = (wh->i_seq[1] << 8 | wh->i_seq[0]) >> IEEE80211_SEQ_SEQ_SHIFT;
	txd->txdseq |= htole16(sum);

	if (!hasqos) {
		/* Use HW sequence numbering for non-QoS frames. */
		txd->txdw4  |= htole32(R92C_TXDW4_HWSEQ);
		txd->txdseq |= htole16(0x8000U);
	}

	/* Compute Tx descriptor checksum. */
	sum = 0;
	ASSERT(sizeof (*txd) / 2 == 16);
	for (i = 0; i < sizeof (*txd) / 2; i++) {
		sum ^= ((uint16_t *)txd)[i];
	}
	txd->txdsum = sum;	/* NB: already little endian. @ txdw7 */
#if 0
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: %08x %08x %08x %08x %08x %08x %08x %08x, tid:%x qos:%x",
	    dp->name, __func__,
	    ((uint32_t *)txd)[0], ((uint32_t *)txd)[1],
	    ((uint32_t *)txd)[2], ((uint32_t *)txd)[3],
	    ((uint32_t *)txd)[4], ((uint32_t *)txd)[5],
	    ((uint32_t *)txd)[6], ((uint32_t *)txd)[7], tid, hasqos));
#endif
#ifdef DEBUG_LEVEL
	if (urtwn_debug >= 10) {
		uwgem_dump_pkt(__func__, (void *)wh,
		    (uintptr_t)m->b_wptr - (uintptr_t)wh, 0, 0);
	}
#endif
	return (m);
}

/*
 * packet reception
 */
static void
urtwn_update_avgrssi(struct uwgem_dev *dp, int rate, int8_t rssi)
{
	struct urtwn_dev *lp = dp->private;
	int pwdb;

	/* Convert antenna signal to percentage. */
	if (rssi <= -100 || rssi >= 20)
		pwdb = 0;
	else if (rssi >= 0)
		pwdb = 100;
	else
		pwdb = 100 + rssi;
	if (rate <= 3) {
		/* CCK gain is smaller than OFDM/MCS gain. */
		pwdb += 6;
		if (pwdb > 100) {
			pwdb = 100;
		}
		if (pwdb <= 14) {
			pwdb -= 4;
		} else if (pwdb <= 26) {
			pwdb -= 8;
		} else if (pwdb <= 34) {
			pwdb -= 6;
		} else if (pwdb <= 42) {
			pwdb -= 2;
		}
	}
	if (lp->avg_pwdb == -1) {	/* Init. */
		lp->avg_pwdb = pwdb;
	} else if (lp->avg_pwdb < pwdb) {
		lp->avg_pwdb = ((lp->avg_pwdb * 19 + pwdb) / 20) + 1;
	} else {
		lp->avg_pwdb = ((lp->avg_pwdb * 19 + pwdb) / 20);
	}
	DPRINTF(4, (CE_CONT, "!%s: %s: PWDB=%d EMA=%d",
	    dp->name, __func__, pwdb, lp->avg_pwdb));
}

static int8_t
urtwn_get_rssi(struct uwgem_dev *dp, int rate, void *physt)
{
	struct urtwn_dev *lp = dp->private;
	static const int8_t cckoff[] = { 16, -12, -26, -46 };
	struct r92c_rx_phystat *phy;
	struct r92c_rx_cck *cck;
	uint8_t rpt;
	int8_t rssi;

	if (rate <= 3) {
		cck = (struct r92c_rx_cck *)physt;
		if (lp->sc_flags & URTWN_FLAG_CCK_HIPWR) {
			rpt = (cck->agc_rpt >> 5) & 0x3;
			rssi = (cck->agc_rpt & 0x1f) << 1;
		} else {
			rpt = (cck->agc_rpt >> 6) & 0x3;
			rssi = cck->agc_rpt & 0x3e;
		}
		rssi = cckoff[rpt] - rssi;
	} else {	/* OFDM/HT. */
		phy = (struct r92c_rx_phystat *)physt;
		rssi = ((letoh32(phy->phydw1) >> 1) & 0x7f) - 110;
	}
	return (rssi);
}

static mblk_t *
urtwn_rx_frame(struct uwgem_dev *dp, uint8_t *buf, int pktlen,
    mblk_t *mp_head, mblk_t *mp)
{
	struct r92c_rx_stat	*stat;
	uint32_t		rxdw0;
	uint32_t		rxdw3;
	uint8_t			rate;
	int8_t			rssi = 0;
	int			infosz;
	struct urtwn_dev *lp = dp->private;

	stat = (struct r92c_rx_stat *)buf;
	rxdw0 = letoh32(stat->rxdw0);
	rxdw3 = letoh32(stat->rxdw3);

	if (rxdw0 & (R92C_RXDW0_CRCERR | R92C_RXDW0_ICVERR)) {
		/*
		 * This should not happen since we setup our Rx filter
		 * to not receive these frames.
		 */
		dp->stats.crc++;
		dp->stats.errrcv++;
		return (NULL);
	}
	if (pktlen < sizeof (struct ieee80211_frame)) {
		dp->stats.runt++;
		dp->stats.errrcv++;
		return (NULL);
	}

	rate = MS(rxdw3, R92C_RXDW3_RATE);
	if (rxdw3 & R92C_RXDW3_HT) {
		/* convert mcs to MHz*2 */
		rate = ieee80211_htrates[rate] * 2;
	}
	infosz = MS(rxdw0, R92C_RXDW0_INFOSZ) * 8;

	/* Get RSSI from PHY status descriptor if present. */
	if (infosz != 0 && (rxdw0 & R92C_RXDW0_PHYST)) {
		rssi = urtwn_get_rssi(dp, rate, &stat[1]);
#if 1
		/* Update our average RSSI. */
		urtwn_update_avgrssi(dp, rate, rssi);
		rssi = lp->avg_pwdb;
#endif
	}

	DPRINTF(10, (CE_CONT,
	    "!%s: %s: Rx frame len=%d rate=%d infosz=%d rssi=%d rate=%d,0x%x",
	    dp->name, __func__,
	    pktlen, rate, infosz, rssi, rate, stat[1]));

	if (mp == NULL) {
		mp = dupb(mp_head);
		if (mp == NULL) {
			dp->stats.norcvbuf++;
			dp->stats.errrcv++;
			return (NULL);
		}
	}
	/* Finalize mblk. */
	mp->b_rptr = ((uint8_t *)&stat[1]) + infosz;
	mp->b_wptr = mp->b_rptr + pktlen;

	mp->b_cont = (void *)(intptr_t)rssi;
	mp->b_prev = 0;	/* rate */

	return (mp);
}

static mblk_t *
urtwn_rx_make_packet(struct uwgem_dev *dp, mblk_t *mp_head)
{
	struct r92c_rx_stat	*stat;
	uint32_t	rxdw0;
	mblk_t		*reuse_mp;
	mblk_t		*mp;
	mblk_t		**tailp;
	uint8_t		*buf;
	int		len, totlen, pktlen, infosz, npkts;

	buf = mp_head->b_rptr;
	len = MBLKL(mp_head);
	if (len < sizeof (*stat)) {
		DPRINTF(0, (CE_CONT, "!%s: %s: xfer too short %d",
		    dp->name, __func__, len));
		return (NULL);
	}

	/* Get the number of encapsulated frames. */
	stat = (struct r92c_rx_stat *)buf;
	npkts = MS(letoh32(stat->rxdw2), R92C_RXDW2_PKTCNT);

	DPRINTF(3, (CE_CONT, "!%s: %s: Rx %d frames in one chunk",
	    dp->name, __func__, npkts));

	/* Process all of them. */
	reuse_mp = mp_head;
	ASSERT(mp_head->b_prev == NULL);
	ASSERT(mp_head->b_next == NULL);
	ASSERT(mp_head->b_cont == NULL);
	tailp = &mp_head->b_next;

	while (npkts-- > 0) {
		if (len < sizeof (*stat)) {
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: len(=%d) is shorter than header(=%d)",
			    dp->name, __func__, len, sizeof (*stat)));
			mp_head = NULL;
			break;
		}
		stat = (struct r92c_rx_stat *)buf;
		rxdw0 = letoh32(stat->rxdw0);

		pktlen = MS(rxdw0, R92C_RXDW0_PKTLEN);
		if (pktlen == 0) {
			DPRINTF(0, (CE_CONT, "!%s: %s: pktlen=%d",
			    dp->name, __func__, pktlen));
			mp_head = NULL;
			break;
		}

		infosz = MS(rxdw0, R92C_RXDW0_INFOSZ) * 8;

		/* Make sure everything fits in xfer. */
		totlen = sizeof (*stat) + infosz + pktlen;
		if (totlen > len) {
			DPRINTF(0, (CE_CONT,
			    "!%s: %s: len(=%d) is shorter than totlen(=%d)",
			    dp->name, __func__, len, totlen));
			mp_head = NULL;
			break;
		}

		/* Process 802.11 frames */
		mp = urtwn_rx_frame(dp, buf, pktlen, mp_head, reuse_mp);
		if (mp == NULL) {
			DPRINTF(0, (CE_CONT,
			   "!%s: %s: unexpected errors in urtwn_rx_frame()",
			    dp->name, __func__));
			mp_head = NULL;
			break;
		}
#ifdef DEBUG_LEVEL
		if (urtwn_debug >= 10) {
			uwgem_dump_pkt(__func__, mp->b_rptr, MBLKL(mp),
			    (intptr_t)mp->b_prev, (intptr_t)mp->b_cont);
		}
#endif
		if (reuse_mp == NULL) {
			/* *not* first frame */
			*tailp = mp;
			tailp = &mp->b_next;
		} else {
			reuse_mp = NULL;
		}

		/* Next chunk is 128-byte aligned. */
		totlen = P2ROUNDUP(totlen, 128);
		buf += totlen;
		len -= totlen;
	}

	/* terminate received mblk list */
	*tailp = NULL;

	return (mp_head);
}

static void
_update_ampdu_factor(struct uwgem_dev *dp, uint_t max_AMPDU_len)
{
	uint8_t	RegToSet_Normal[4] = {0x41, 0xa8, 0x72, 0xb9};
	uint8_t	RegToSet_BT[4] = {0x31, 0x74, 0x42, 0x97};
	uint8_t	FactorToSet;
	uint8_t	*pRegToSet;
	uint8_t	index = 0;
#ifdef CONFIG_BT_COEXISTx
	struct btcoexist_priv	 *pbtpriv = &(padapter->halpriv.bt_coexist);

	if ((pbtpriv->BT_Coexist) &&
	    (pbtpriv->BT_CoexistType == BT_CSR_BC4)) {
		pRegToSet = RegToSet_BT; // 0x97427431;
	} else
#endif
	{
		pRegToSet = RegToSet_Normal; // 0xb972a841;
	}

	FactorToSet = max_AMPDU_len;
	if (FactorToSet <= 3) {
		FactorToSet = (1 << (FactorToSet + 2));
		if (FactorToSet > 0xf) {
			FactorToSet = 0xf;
		}

		for (index = 0; index < 4; index++) {
			if ((pRegToSet[index] & 0xf0) > (FactorToSet << 4)) {
				pRegToSet[index] =
				    (pRegToSet[index] & 0x0f) |
				    (FactorToSet << 4);
			}
			if ((pRegToSet[index] & 0x0f) > FactorToSet) {
				pRegToSet[index] =
				    (pRegToSet[index] & 0xf0) | (FactorToSet);
			}

			urtwn_write_1(dp,
			    (R92C_AGGLEN_LMT + index), pRegToSet[index]);
		}
		//RT_TRACE(_Comp, _Level, Fmt)(COMP_MLME, DBG_LOUD,
		//    ("Set HW_VAR_AMPDU_FACTOR: %#x\n", FactorToSet));
	}
}

static int
urtwn_newstate(struct uwgem_dev *dp, enum ieee80211_state nstate, int arg)
{
	struct ieee80211com *ic = &dp->ic;
	struct urtwn_dev *lp = dp->private;
	struct ieee80211_node *ni;
	enum ieee80211_state ostate;
	uint32_t reg;
	uint_t	ht40m;
	uint_t	sifs_time;
	int err = USB_SUCCESS;
	int	max_AMPDU_len;
	int	min_MPDU_spacing;

	ostate = ic->ic_state;
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: %s(%x) -> %s(%x), chan:%d", dp->name, __func__,
	    ieee80211_state_name[ostate], ostate,
	    ieee80211_state_name[nstate], nstate,
	    ieee80211_chan2ieee(ic, ic->ic_curchan)));

	if (ostate == IEEE80211_S_RUN) {
		/* Turn link LED off. */
		urtwn_set_led(dp, URTWN_LED_LINK, 0);

		/* Set media status to 'No Link'. */
		urtwn_set_netype0_msr(dp, R92C_CR_NETTYPE_NOLINK);

		/* Stop Rx of data frames. */
		urtwn_write_2(dp, R92C_RXFLTMAP2, 0);

		/* Rest TSF. */
		urtwn_write_1(dp, R92C_DUAL_TSF_RST, 0x03);

		/* Disable TSF synchronization. */
		urtwn_write_1(dp, R92C_BCN_CTRL,
		    urtwn_read_1(dp, R92C_BCN_CTRL) |
		    R92C_BCN_CTRL_DIS_TSF_UDT0);

		/* back to 20Mhz mode */
		urtwn_set_chan(dp, ic->ic_curchan, IEEE80211_HTINFO_2NDCHAN_NONE);

		if (ic->ic_opmode == IEEE80211_M_IBSS ||
		    ic->ic_opmode == IEEE80211_M_HOSTAP) {
			/* Stop BCN */
			urtwn_write_1(dp, R92C_BCN_CTRL,
			    urtwn_read_1(dp, R92C_BCN_CTRL) &
			    ~(R92C_BCN_CTRL_EN_BCN | R92C_BCN_CTRL_TXBCN_RPT));
		}

		/* flush all cam entries */
		urtwn_cam_init(dp);
	}
#ifdef BUGFIX_OPENBSD
	if (ostate == IEEE80211_S_SCAN && nstate != IEEE80211_S_SCAN) {
		/*
		 * End of scanning
		 */
		/* flush 4-AC Queue after site_survey */
		urtwn_write_1(dp, R92C_TXPAUSE, 0x0);

		urtwn_write_4(dp, R92C_RCR,
		    urtwn_read_4(dp, R92C_RCR) |
		    (R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));
#ifdef notyet
		/* config MSR */
		Set_NETYPE0_MSR(padapter, (pmlmeinfo->state & 0x3));

		/* turn on dynamic functions */
		Restore_DM_Func_Flag(padapter);
#endif
	}
#endif
	switch (nstate) {
	case IEEE80211_S_INIT:
		/* Turn link LED off. */
		urtwn_set_led(dp, URTWN_LED_LINK, 0);
		break;

	case IEEE80211_S_SCAN:
		if (ostate != IEEE80211_S_SCAN) {
			/*
			 * begin of scanning
			 */

			/* from hal/rtl8192c/usb/rtl8192c_cmd.c */

			/* Set gain for scanning. */
			reg = urtwn_bb_read(dp, R92C_OFDM0_AGCCORE1(0));
			reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
			urtwn_bb_write(dp, R92C_OFDM0_AGCCORE1(0), reg);

			reg = urtwn_bb_read(dp, R92C_OFDM0_AGCCORE1(1));
			reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
			urtwn_bb_write(dp, R92C_OFDM0_AGCCORE1(1), reg);

			urtwn_set_netype0_msr(dp, R92C_CR_NETTYPE_NOLINK);

			/* Allow Rx from any BSSID. */
			urtwn_write_4(dp, R92C_RCR,
			    urtwn_read_4(dp, R92C_RCR) &
			    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));
#ifdef BUGFIX_OPENBSD
			/* Stop Rx of data frames. */
			urtwn_write_2(dp, R92C_RXFLTMAP2, 0);

			/* disable update TSF */
			urtwn_write_1(dp, R92C_BCN_CTRL,
			    urtwn_read_1(dp, R92C_BCN_CTRL) | BIT(4));
#endif
		}

		/* Make link LED blink during scan. */
		urtwn_set_led(dp, URTWN_LED_LINK, !lp->ledlink);

		/* from site_survey() in core/rtw_mlme_ext.c */

		/* Pause AC Tx queues. */
		urtwn_write_1(dp, R92C_TXPAUSE,
		    urtwn_read_1(dp, R92C_TXPAUSE) | 0x0f);

		urtwn_set_chan(dp, ic->ic_curchan, IEEE80211_HTINFO_2NDCHAN_NONE);
		break;

	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
		/* setup EDCA parameters. */
		urtwn_write_4(dp, R92C_EDCA_VO_PARAM, 0x002f3217);
		urtwn_write_4(dp, R92C_EDCA_VI_PARAM, 0x005e4317);
		urtwn_write_4(dp, R92C_EDCA_BE_PARAM, 0x00105320);
		urtwn_write_4(dp, R92C_EDCA_BK_PARAM, 0x0000a444);

		/* Set initial gain under link. */
		reg = urtwn_bb_read(dp, R92C_OFDM0_AGCCORE1(0));
		reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x32);
		urtwn_bb_write(dp, R92C_OFDM0_AGCCORE1(0), reg);

		reg = urtwn_bb_read(dp, R92C_OFDM0_AGCCORE1(1));
		reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x32);
		urtwn_bb_write(dp, R92C_OFDM0_AGCCORE1(1), reg);

		urtwn_set_netype0_msr(dp, R92C_CR_NETTYPE_NOLINK);

		/* Allow Rx from any BSSID. */
		urtwn_write_4(dp, R92C_RCR,
		    urtwn_read_4(dp, R92C_RCR) &
		    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

		urtwn_set_chan(dp, ic->ic_curchan, IEEE80211_HTINFO_2NDCHAN_NONE);
		break;

	case IEEE80211_S_RUN:
		ni = ic->ic_bss;
		ieee80211_fix_rate(ni, &ni->in_rates, IEEE80211_F_DOSORT);
		ieee80211_setbasicrates(&ni->in_rates, ic->ic_curmode);

                ht40m = (ni->in_chw == 40 &&
                    (ni->in_chan->ich_flags & IEEE80211_CHAN_HT40))
                    ? (ni->in_ht2ndchan & IEEE80211_HTINFO_2NDCHAN)
		    : IEEE80211_HTINFO_2NDCHAN_NONE;

		urtwn_set_chan(dp, ic->ic_curchan, ht40m);

#ifdef CONFIG_11N
		if (ni->in_flags & IEEE80211_NODE_HT) {

			if (lp->ntxchains > 1 &&
			    ni->in_htrates.rs_nrates >= 16) {
				ni->in_htrates.rs_nrates = 16;
			} else {
				ni->in_htrates.rs_nrates = 8;
			}
			dp->txrate = ni->in_htrates.rs_nrates - 1;
			/* setup AMPDU */
			/*
			 * AMPDU_para [1:0]:Max AMPDU Len
			 *   0:8k 1:16k 2:32k 3:64k
			 * AMPDU_para [4:2]:Min MPDU Start Spacing
			 */
			max_AMPDU_len = ni->in_htparam & 0x3;
			min_MPDU_spacing = (ni->in_htparam >> 2) & 0x7;

			urtwn_write_1(dp, R92C_AMPDU_MIN_SPACE,
			    (urtwn_read_1(dp, R92C_AMPDU_MIN_SPACE) & ~0x7) |
			    min_MPDU_spacing);

			_update_ampdu_factor(dp, max_AMPDU_len);

#ifdef notyet
			/* Config current HT Protection mode. */
			pmlmeinfo->HT_protection = pmlmeinfo->HT_info.infos[1] & 0x3;
#endif
		}
#endif
		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			urtwn_set_chan(dp, ic->ic_ibss_chan, IEEE80211_HTINFO_2NDCHAN_NONE);

			/* Enable Rx of data frames. */
			urtwn_write_2(dp, R92C_RXFLTMAP2, 0xffff);

			/* Turn link LED on. */
			urtwn_set_led(dp, URTWN_LED_LINK, 1);
			break;
		}

		/* Intialize rate adaptation. */
		urtwn_ra_init(dp);

		/* Set media status to 'Associated'. */
		urtwn_set_netype0_msr(dp, urtwn_get_netype(dp));

		/* Set BSSID. */
		urtwn_write_4(dp, R92C_BSSID + 0, LE_READ_4(&ni->in_bssid[0]));
		urtwn_write_4(dp, R92C_BSSID + 4, LE_READ_2(&ni->in_bssid[4]));

		if (ic->ic_curmode == IEEE80211_MODE_11NG) {
			/* 802.11n 2.4G */
			urtwn_write_1(dp, R92C_INIRTS_RATE_SEL, 0xa);
		} else if (ic->ic_curmode == IEEE80211_MODE_11B) {
			urtwn_write_1(dp, R92C_INIRTS_RATE_SEL, 0);
		} else {
			/* 802.11b/g */
			urtwn_write_1(dp, R92C_INIRTS_RATE_SEL, 3);
		}
#if 0
		/* Flush all AC queues. */
		urtwn_write_1(dp, R92C_TXPAUSE, 0);
#endif
		/* Enable Rx of data frames. */
		urtwn_write_2(dp, R92C_RXFLTMAP2, 0xffff);

		/* Set beacon interval. */
		urtwn_write_2(dp, R92C_BCN_INTERVAL, ni->in_intval);

		if (ic->ic_opmode == IEEE80211_M_STA) {
			/* Allow Rx from our BSSID only. */
#if 1
			urtwn_write_4(dp, R92C_RCR,
			    urtwn_read_4(dp, R92C_RCR) |
			    R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN);
#else
			urtwn_write_4(dp, R92C_RCR,
			    (urtwn_read_4(dp, R92C_RCR) &
			    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN))
			    | R92C_RCR_ADF);
#endif

			/* Enable TSF synchronization. */
			urtwn_tsf_sync_enable(dp);
		}

		sifs_time = (ni->in_flags & IEEE80211_NODE_HT) ? 14 : 10;
		urtwn_write_1(dp, R92C_SIFS_CCK + 1, sifs_time);
		urtwn_write_1(dp, R92C_SIFS_OFDM + 1, sifs_time);
		urtwn_write_1(dp, R92C_SPEC_SIFS + 1, sifs_time);
		urtwn_write_1(dp, R92C_MAC_SPEC_SIFS + 1, sifs_time);
		urtwn_write_1(dp, R92C_R2T_SIFS + 1, sifs_time);
		urtwn_write_1(dp, R92C_T2T_SIFS + 1, sifs_time);

		/* Turn link LED on. */
		urtwn_set_led(dp, URTWN_LED_LINK, 1);

		/* Reset average RSSI. */
		lp->avg_pwdb = -1;

		/* Reset temperature calibration state machine. */
		lp->thcal_state = 0;
		lp->thcal_lctemp = 0;
		break;
	}
	return (err);
}

static int
urtwn_attach_chip(struct uwgem_dev *dp)
{
	struct urtwn_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	int	i;
	int	error;
	int	ret;
	extern int ieee80211_debug;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

#if DEBUG_LEVEL > 0
	ieee80211_debug = IEEE80211_MSG_HT;
	ieee80211_debug = IEEE80211_MSG_CONFIG;
	ieee80211_debug = -1;
	ieee80211_debug &= ~(IEEE80211_MSG_ELEMID | IEEE80211_MSG_ASSOC | IEEE80211_MSG_HT);
	ieee80211_debug &= ~(IEEE80211_MSG_INPUT);
#if 0
#endif
	ieee80211_debug &= ~(IEEE80211_MSG_CRYPTO);
	ieee80211_debug &= ~(IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS);
#endif
	ret = USB_SUCCESS;

	/* from rtw_init_netdev() in os_dep/linux/os_intfs.c */
	/* from loadparam() in os_dep/linux/os_intfs.c */

	lp->chipversion = VERSION_TEST_CHIP_88C;
	lp->rfintfs = HWPI;
	lp->lbkmode = 0;
	lp->hci = RTL8192C_USB;
	lp->network_mode = Ndis802_11IBSS;
	lp->channel = 1;
	lp->wireless_mode = WIRELESS_11BG;
	lp->vrtl_carrier_sense = AUTO_VCS;
	lp->vcs_type = RTS_CTS;
	lp->frag_thresh = 2346;
	lp->preamble = PREAMBLE_LONG;
	lp->scan_mode = 1;	/* active, passive */
	lp->adhoc_tx_pwr = 1;
	lp->soft_ap = 0;
	lp->power_mgnt = PS_MODE_ACTIVE;

	lp->radio_enable = 1;
	lp->long_retry_lmt = 7;
	lp->short_retry_lmt = 7;
	lp->busy_thresh = 40;
	lp->ack_policy = 0 /* NORMAL_ACK */;
	lp->mp_mode = 0;
	lp->software_encrypt = 0;
	lp->software_dencrypt = 0;
	lp->wmm_enable = 1;

	lp->uapsd_enable = 0;
	lp->uapsd_max_sp = NO_LIMIT;
	lp->uapsd_acbk_en = 0;
	lp->uapsd_acbe_en = 0;
	lp->uapsd_acvi_en = 0;
	lp->uapsd_acvo_en = 0;

#ifdef CONFIG_11N
	lp->ht_enable = 1;
#ifdef CONFIG_40MHZ
	lp->cbw40_enable = 1;
#endif
#ifdef CONFIG_AMPDU
	lp->ampdu_enable = 1;
#endif
#endif

	lp->rf_config = RF_819X_MAX_TYPE;
	lp->low_power = 0;
	lp->wifi_spec = 0;
	lp->channel_plan = 14 /* RT_CHANNEL_DOMAIN_MAX */ ;

	lp->bAcceptAddaReq = B_TRUE;
	lp->antdiv_cfg = 2;
#ifdef CONFIG_AUTOSUSPEND
	lp->usbss_enable = 1;
#else
	lp->usbss_enable = 0;
#endif
	lp->hwpdn_mode = 2;	/* 2: by eFuse config */
	lp->hwpwrp_detected = 1;	/* 0: disable, 1:enable */

	lp->ishighspeed = dp->highspeed;
	if (dp->highspeed) {
		lp->pktsize = 512;
	} else {
		lp->pktsize = 64;
	}

	/* from rtl8192cu_interface_configure() in hal/rtl8192c/usb/usb_halinit.c */
#ifdef CONFIG_TX_AGGR
	lp->UsbTxAggMode = 1;
	lp->UsbTxAggDescNum = 6;
#endif

#ifdef CONFIG_RX_AGGR
	lp->UsbRxAggMpde = USB_RX_AGG_DMA;
	lp->UsbRxAggBlockCount = 8;	/* unit 512b */
	lp->UsbRxAggBlockTimeout = 6;
	lp->UsbRxAggPageCount = 48;	/* unit 128b */
	lp->UsbRxAggPageTimeout = 4;	/* 34mS/(2^4) */
#endif
	switch (dp->num_bulkout_pipes) {
	case 3:
		lp->ac2idx[WME_AC_BK] = 2;
		lp->ac2idx[WME_AC_BE] = 2;
		lp->ac2idx[WME_AC_VI] = 1;
		lp->ac2idx[WME_AC_VO] = 0;	/* highest prio. */
		break;

	case 2:
		lp->ac2idx[WME_AC_BK] = 1;
		lp->ac2idx[WME_AC_BE] = 1;
		lp->ac2idx[WME_AC_VI] = 0;
		lp->ac2idx[WME_AC_VO] = 0;	/* highest prio. */
		break;

	case 1:
		lp->ac2idx[WME_AC_BK] = 0;
		lp->ac2idx[WME_AC_BE] = 0;
		lp->ac2idx[WME_AC_VI] = 0;
		lp->ac2idx[WME_AC_VO] = 0;	/* highest prio. */
		break;

	default:
		cmn_err(CE_NOTE, "!%s: incorrect number of bulkout pipes (%d)",
		    dp->name, dp->num_bulkout_pipes);
		return (USB_FAILURE);
	}

	lp->fwready = B_FALSE;

	error = urtwn_read_chipid(dp);
	if (error != USB_SUCCESS) {
		cmn_err(CE_NOTE, "!%s: unsupported test chip", dp->name);
		return (error);
	}

	/* Determine number of Tx/Rx chains. */
	if (lp->chip & URTWN_CHIP_92C) {
		lp->ntxchains = (lp->chip & URTWN_CHIP_92C_1T2R) ? 1 : 2;
		lp->nrxchains = 2;
	} else {
		lp->ntxchains = 1;
		lp->nrxchains = 1;
	}
	urtwn_read_rom(dp);

	cmn_err(CE_CONT,
	    "!%s: version:%x, MAC/BB RTL%s, RF 6052 %dT%dR, address %s",
	    dp->name,
	    lp->chip,
	    (lp->chip & URTWN_CHIP_92C) ? "8192CU" :
	    (lp->board_type == R92C_BOARD_TYPE_HIGHPA) ? "8188RU" :
	    (lp->board_type == R92C_BOARD_TYPE_MINICARD) ? "8188CE-VAU" :
	    "8188CUS", lp->ntxchains, lp->nrxchains,
	    ether_sprintf((void *)dp->dev_addr));

	/* setup capabilities */
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* set device capabilities */
	ic->ic_caps =
	    IEEE80211_C_TXPMGT |	/* tx power management */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_SHSLOT;		/* short slot time supported */
#ifdef CONFIG_IBSS
	ic->ic_caps |= IEEE80211_C_IBSS;	/* ad-hoc mode*/
#endif
#ifdef CONFIG_11N
	ic->ic_caps |= IEEE80211_C_PMGT;
#endif
#ifdef ORG
	    IEEE80211_C_WEP |		/* WEP. */
	    IEEE80211_C_RSN;		/* WPA/RSN. */
#endif

	/* WPA/WPA2 support */
	ic->ic_caps |= IEEE80211_C_WPA; /* Support WPA/WPA2 */

#ifdef ORG
	/* Set HT capabilities. */
	ic->ic_htcaps =
	    IEEE80211_HTCAP_DSSSCCK40;
#endif

	/*
	 * Support 802.11n/HT
	 */
#ifdef CONFIG_11N
	ic->ic_htcaps = IEEE80211_HTC_HT;
	ic->ic_htcaps |= IEEE80211_HTC_AMSDU;
	ic->ic_htcaps |= IEEE80211_HTCAP_MAXAMSDU_7935;
#ifdef CONFIG_SHORTGI
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI20;
#endif
#ifdef CONFIG_40MHZ
	ic->ic_htcaps |= IEEE80211_HTCAP_CHWIDTH40;
#ifdef CONFIG_SHORTGI
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI40;
#endif
#endif
#ifdef CONFIG_AMPDU
	ic->ic_htcaps |= IEEE80211_HTC_AMPDU;
#endif
#endif
	/* set supported .11b and .11g rates */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = urtwn_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = urtwn_rateset_11g;

	/* set supported .11b and .11g channels (1 through 14) */
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
#ifdef CONFIG_40MHZ
	/* for channels in 5GHz: not implemented yet */

	/* for channels in 2GHz */
	for (i = 1; i <= 14 - 5; i++) {
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40U;
	}
	for (i = 1 + 5; i <= 14; i++) {
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40D;
	}
#endif
#endif
	ic->ic_maxrssi = 100;
	ic->ic_set_shortslot = NULL;	/* not implemented */

	ic->ic_def_txkey = 0;

	ic->ic_flags = 0;
	dp->rx_buf_len = 16*1024;

	dp->bulkout_timeout =
	    dp->ugc.uwgc_tx_timeout / drv_usectohz(1000*1000);
#if 0
	/* Map 802.11 access categories to USB pipes. */
	lp->ac2idx[WME_AC_BK] =
	lp->ac2idx[WME_AC_BE] =
	    (dp->num_bulkout_pipes == 3) ? 2 :
	    ((dp->num_bulkout_pipes == 2) ? 1 : 0);
	lp->ac2idx[WME_AC_VI] = (dp->num_bulkout_pipes == 3) ? 1 : 0;
	lp->ac2idx[WME_AC_VO] = 0;	/* Always use highest prio. */
#endif
	lp->iqk_initialized = B_FALSE;

	/* from _MappingOutEP in hal/rtl8192c/usb/usb_halinit.c */
	return (ret);

usberr:
	cmn_err(CE_WARN, "!%s: %s: usb error detected (%d)",
	    dp->name, __func__, error);
	return (USB_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
static int
urtwnattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct urtwn_dev	*lp;

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
		ugcp->uwgc_attach_chip = &urtwn_attach_chip;
		ugcp->uwgc_reset_chip = &urtwn_reset_chip;
		ugcp->uwgc_init_chip = &urtwn_init_chip;
		ugcp->uwgc_start_chip = &urtwn_start_chip;
		ugcp->uwgc_stop_chip = &urtwn_stop_chip;

		ugcp->uwgc_set_rx_filter = &urtwn_set_rx_filter;
		ugcp->uwgc_get_stats = &urtwn_get_stats;
		ugcp->uwgc_interrupt = NULL;

		/* packet operation */
		ugcp->uwgc_tx_make_packet = &urtwn_tx_make_packet;
		ugcp->uwgc_rx_make_packet = &urtwn_rx_make_packet;

		/* newstate */
		ugcp->uwgc_newstate = &urtwn_newstate;

		lp = kmem_zalloc(sizeof (struct urtwn_dev), KM_SLEEP);

		dp = uwgem_do_attach(dip, ugcp, lp, sizeof (struct urtwn_dev));

		kmem_free(ugcp, sizeof (*ugcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

err_free_mem:
		kmem_free(lp, sizeof (struct urtwn_dev));
err_close_pipe:
err:
		return (DDI_FAILURE);
	}

	if (cmd == DDI_RESUME) {
		return (uwgem_resume(dip));
	}

	return (DDI_FAILURE);
}

static int
urtwndetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
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
UWGEM_STREAM_OPS(urtwn_ops, urtwnattach, urtwndetach);

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&urtwn_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!urtwn: _init: called"));

	status = uwgem_mod_init(&urtwn_ops, "urtwn");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		uwgem_mod_fini(&urtwn_ops);
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

	DPRINTF(2, (CE_CONT, "!urtwn: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		uwgem_mod_fini(&urtwn_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
