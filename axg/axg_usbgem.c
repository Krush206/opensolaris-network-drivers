/*
 * axg_usbgem.c : ASIX AX88178a/179 USB to Gigabit Ethernet Driver for Solaris
 *
 * Copyright (c) 2004-2015 Masayuki Murayama.  All rights reserved.
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


#pragma ident "%W %E"

/*
 *  Changelog:
 */

/*
 * TODO
 */
/* ======================================================= */

/*
 * Solaris system header files and macros
 */

/* minimum kernel headers for drivers */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/byteorder.h>
#ifdef CONFIG_CKSUM_OFFLOAD
#include <sys/pattr.h>
#endif
#include <sys/sysmacros.h>

/* ethernet stuff */
#include <sys/ethernet.h>

/* interface card depend stuff */
#include <sys/stropts.h>
#ifdef CONFIG_CKSUM_OFFLOAD
#include <sys/strsubr.h>	/* required for hcksum_* functions */
#endif
#include <sys/stream.h>
#include <sys/strlog.h>
#include <sys/usb/usba.h>
#include "usbgem.h"

/* hardware stuff */
#include "usbgem_mii.h"
#include "ax88178a_reg.h"

char	ident[] = "ax88178a/88179 driver v" VERSION;

/*
 * Useful macros
 */

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int axg_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (axg_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Our configration for ax88178a/88179
 */
/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))

/*
 * RX/TX buffer size
 */

/*
 * Local device definitions
 */
struct chip_info {
	uint16_t	vid;	/* usb vendor id */
	uint16_t	pid;	/* usb product id */
	char		*name;
};

struct chip_info chiptbl_8817x[] = {
{
	0x0b95, 0x1790,
	"ASIX AX88179 10/100/1000",
},
{
	0x0b95, 0x178a,
	"ASIX AX88178A 10/100/1000",
},
{
	0x0df6, 0x0072,
	"Sitecom USB 3.0 to Gigabit Adapter",
},
{
	0x17ef, 0x304b,
	"ThinkPad OneLinkDock USB GigaLAN",
},
{
	0x0930, 0x0a13,
	"Toshiba USB3.0 to Gigabit LAN Adapter",
},
{
	0x04e8, 0xa100,
	"Samsung USB Ethernet Adapter",
},
};

#define	CHIPTABLESIZE	(sizeof (chiptbl_8817x) / sizeof (struct chip_info))

struct axg_dev {
	/*
	 * Misc HW information
	 */
	struct chip_info	*chip;
	uint16_t		rxctl;
	uint16_t		mode;
	uint8_t			usb_link_status;
	boolean_t		phy_has_reset;
	uint8_t		last_link_state;
	uint8_t		last_int_data[8];
};

static struct {
	uint8_t	ctrl;
	uint8_t	timer_l;
	uint8_t	timer_h;
	uint8_t	size;
	uint8_t	ifg;
} AX88179_BULKIN_SIZE[] = {
	{7, 0x4f, 0,    0x12, 0xff},
	{7, 0x20, 3,    0x16, 0xff},
	{7, 0xae, 7,    0x18, 0xff},
	{7, 0xcc, 0x4c, 0x18, 8},
};

/*
 * private functions
 */

/* mii operations */
static uint16_t axg_mii_read(struct usbgem_dev *, uint_t, int *errp);
static void axg_mii_write(struct usbgem_dev *, uint_t, uint16_t, int *errp);

/* nic operations */
static int axg_reset_chip(struct usbgem_dev *);
static int axg_init_chip(struct usbgem_dev *);
static int axg_start_chip(struct usbgem_dev *);
static int axg_stop_chip(struct usbgem_dev *);
static int axg_set_media(struct usbgem_dev *);
static int axg_set_rx_filter(struct usbgem_dev *);
static int axg_get_stats(struct usbgem_dev *);
static void  axg_interrupt(struct usbgem_dev *, mblk_t *);

/* packet operations */
static mblk_t *axg_tx_make_packet(struct usbgem_dev *, mblk_t *);
static mblk_t *axg_rx_make_packet(struct usbgem_dev *, mblk_t *);

/* =============================================================== */
/*
 * I/O functions
 */
/* =============================================================== */
#define	OUT(dp, req, val, ix, len, buf, errp, label)	\
	if ((*(errp) = usbgem_ctrl_out((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* value */    (buf),	\
	/* size */     (len))) != USB_SUCCESS) goto label

#define	IN(dp, req, val, ix, len, buf, errp, label)	\
	if ((*(errp) = usbgem_ctrl_in((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* valuep */   (buf),	\
	/* size */     (len))) != USB_SUCCESS) goto label

#define	OUT_VAL(dp, req, val, ix, len, value, errp, label)	\
	if ((*(errp) = usbgem_ctrl_out_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* value */    (value))) != USB_SUCCESS) goto label

#define	IN_VAL(dp, req, val, ix, len, valp, errp, label)	\
	if ((*(errp) = usbgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		    | USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */ (req),	\
	/* wValue */   (val),	\
	/* wIndex */   (ix),	\
	/* wLength */  (len),	\
	/* valuep */   (void *)(valp))) != USB_SUCCESS) goto label


/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int axg_bEEE = 0;

static void
axg_EEE_setting(struct usbgem_dev *dp)
{
	int	err = USB_SUCCESS;
	
	if (axg_bEEE) {
		// Enable EEE
		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MACR, 2, 0x07, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MAADR, 2, 0x3c, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MACR, 2, 0x4007, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MAADR, 2, 0x06, &err, usberr);
	} else {
		// Disable EEE
		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MACR, 2, 0x07, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MAADR, 2, 0x3c, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MACR, 2, 0x4007, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    GMII_PHY_MAADR, 2, 0, &err, usberr);
	}
usberr:
	;
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
}

static int axg_bGETH = 0;

static void
axg_Gether_setting(struct usbgem_dev *dp)
{
	int	err = USB_SUCCESS;

	if (axg_bGETH) {
		/* Enable Green Ethernet */
		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    31, 2, 0x03, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    25, 2, 0x3247, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    31, 2, 0x05, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    1, 2, 0x0680, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    31, 2, 0, &err, usberr);
	} else {
		/* Disable Green Ethernet */
		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    31, 2, 0x03, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    25, 2, 0x3246, &err, usberr);

		OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID,
		    31, 2, 0, &err, usberr);
	}
usberr:
	;
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
}

static int
axg_check_eeprom(struct usbgem_dev *dp)
{
	int	i;
	int	j;
	int	err = USB_FAILURE;
	uint8_t	buf[2] = {0};
	uint8_t	eeprom[20] = {0};
	uint16_t	csum = 0;

	/* Read EEPROM content */
	for (i = 0 ; i < 6; i++) {

		OUT_VAL(dp, ACCESS_MAC, SROMADDR,
		    1, 1, i, &err, usberr);

		OUT_VAL(dp, ACCESS_MAC, SROMCMD,
		    1, 1, SROMCMD_EEP_RD, &err, usberr);

		j = 0;
		do {
			if (j++ > 100) {
				goto usberr;
			}
			IN(dp, ACCESS_MAC, SROMCMD,
			    1, 1, buf, &err, usberr);
		} while ((buf[0] & SROMCMD_EEP_BUSY) != 0);

		IN(dp, ACCESS_MAC, SROMDATA_LOW,
		    2, 2, &eeprom[i * 2], &err, usberr);

		if ((i == 0) && (eeprom[0] == 0xff)) {
			goto usberr;
		}
	}

	csum = eeprom[6] + eeprom[7] + eeprom[8] + eeprom[9];
	csum = (csum >> 8) + (csum & 0xff);

	if ((csum + eeprom[10]) == 0xff) {
		err = USB_SUCCESS;
	}

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_check_efuse(struct usbgem_dev *dp, void *ledmode)
{
	int	i;
	int	err = USB_FAILURE;
	uint8_t		efuse[64] = {0x00};
	uint16_t	csum = 0;

	IN(dp, ACCESS_EFUSE, 0, 64, 64, efuse, &err, usberr);

	if (efuse[0] == 0xff) {
		goto usberr;
	}

	for (i = 0; i < 64; i++) {
		csum = csum + efuse[i];
	}

	while (csum > 255) {
		csum = (csum & 0x00ff) + ((csum >> 8) & 0x00ff);
	}

	if (csum == 0xff) {
		memcpy((uint8_t	*)ledmode, &efuse[51], 2);
		err = USB_SUCCESS;
	}

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_convert_old_led(struct usbgem_dev *dp, uint8_t efuse, void *ledvalue)
{
	uint8_t	ledmode = 0;
	int	err = USB_SUCCESS;
	uint16_t	tmp = 0;
	uint16_t	led = 0;

	/* loaded the old eFuse LED Mode */
	if (efuse) {
		IN_VAL(dp, ACCESS_EFUSE, 0x18, 1, 2, &tmp, &err, usberr);
		ledmode = tmp;
	} else {
		/* loaded the old EEprom LED Mode */
		IN_VAL(dp, ACCESS_EEPROM, 0x3c, 1, 2, &tmp, &err, usberr);

		ledmode = tmp >> 8;
	}

	DPRINTF(0, (CE_CONT, "!%s:%s Old LED Mode = %02X\n",
	    dp->name, __func__, ledmode));

	switch (ledmode) {
	case 0xff:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;

	case 0xfe:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID;
		break;

	case 0xfd:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 |
		      LED2_LINK_10 | LED_VALID;
		break;

	case 0xfc:
		led = LED0_ACTIVE | LED1_ACTIVE | LED1_LINK_1000 | LED2_ACTIVE |
		      LED2_LINK_100 | LED2_LINK_10 | LED_VALID;
		break;

	default:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	}

	memcpy((uint8_t	*)ledvalue, &led, 2);

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_led_setting(struct usbgem_dev *dp)
{
	int	err = USB_FAILURE;
	uint8_t	ledfd = 0;
	uint8_t	value = 0;
	uint16_t	tmp = 0;
	uint16_t	ledact = 0;
	uint16_t	ledlink = 0;
	uint16_t	ledvalue = 0;
	int		j;
	uint8_t		buf[2];

	/* Check AX88179 version. UA1 or UA2 */
	IN(dp, ACCESS_MAC, GENERALSTATUS, 1, 1, &value, &err, usberr);

	/* UA1 */
	if ((value & GENERALSTATUS_SECLD) == 0) {
		OUT_VAL(dp, ACCESS_MAC, GPIO, 1, 1,
		    GPIO_GPIO3EN | GPIO_GPIO2EN | GPIO_GPIO1EN,
		    &err, usberr);
	}

	/* check EEprom */
	if (axg_check_eeprom(dp) == USB_SUCCESS) {
		OUT_VAL(dp, ACCESS_MAC, SROMADDR, 1, 1,
		    0x42, &err, usberr);

		OUT_VAL(dp, ACCESS_MAC, SROMCMD, 1, 1,
		    SROMCMD_EEP_RD, &err, usberr);

		j = 0;
		do {
			if (j++ >= 10) {
				DPRINTF(0, (CE_CONT, "!%s: %s: timeout",
				    dp->name, __func__));
				goto usberr;
			}
			IN_VAL(dp, ACCESS_MAC, SROMCMD, 1, 1,
			    &value, &err, usberr);
		} while ((value & SROMCMD_EEP_BUSY) != 0);

		IN(dp, ACCESS_MAC, SROMDATA_HIGH, 1, 1,
		    &value, &err, usberr);

		ledvalue = value << 8;

		IN(dp, ACCESS_MAC, SROMDATA_LOW, 1, 1,
		    &value, &err, usberr);
		ledvalue |= value;

		/* load internal ROM for default setting */
		if (ledvalue == 0xffff ||
		    (ledvalue & LED_VALID) == 0) {
			axg_convert_old_led(dp, 0, &ledvalue);
		}

	} else if (axg_check_efuse(dp, &ledvalue) == USB_SUCCESS) {
		/* check efuse */
		if (ledvalue == 0xffff ||
		    (ledvalue & LED_VALID) == 0) {
			axg_convert_old_led(dp, 0, &ledvalue);
		}
	} else {
		axg_convert_old_led(dp, 0, &ledvalue);
	}

	OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_PAGE_SELECT,
	    2, GMII_PHY_PAGE_SELECT_EXT, &err, usberr);

	OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_PHYPAGE,
	    2, 0x2c, &err, usberr);

	IN_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_LED_ACTIVE,
	    2, &ledact, &err, usberr);

	IN_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_LED_LINK,
	    2, &ledlink, &err, usberr);

	ledact &= ~(GMII_LED0_ACTIVE | GMII_LED1_ACTIVE | GMII_LED2_ACTIVE);
	ledlink &= ~(GMII_LED0_LINK_10 | GMII_LED0_LINK_100 | GMII_LED0_LINK_1000
	    | GMII_LED1_LINK_10 | GMII_LED1_LINK_100 | GMII_LED1_LINK_1000
	    | GMII_LED2_LINK_10 | GMII_LED2_LINK_100 | GMII_LED2_LINK_1000);

	if (ledvalue & LED0_ACTIVE) {
		ledact |= GMII_LED0_ACTIVE;
	}
	if (ledvalue & LED1_ACTIVE) {
		ledact |= GMII_LED1_ACTIVE;
	}
	if (ledvalue & LED2_ACTIVE) {
		ledact |= GMII_LED2_ACTIVE;
	}

	if (ledvalue & LED0_LINK_10) {
		ledlink |= GMII_LED0_LINK_10;
	}
	if (ledvalue & LED1_LINK_10) {
		ledlink |= GMII_LED1_LINK_10;
	}
	if (ledvalue & LED2_LINK_10) {
		ledlink |= GMII_LED2_LINK_10;
	}

	if (ledvalue & LED0_LINK_100) {
		ledlink |= GMII_LED0_LINK_100;
	}
	if (ledvalue & LED1_LINK_100) {
		ledlink |= GMII_LED1_LINK_100;
	}
	if (ledvalue & LED2_LINK_100) {
		ledlink |= GMII_LED2_LINK_100;
	}

	if (ledvalue & LED0_LINK_1000) {
		ledlink |= GMII_LED0_LINK_1000;
	}
	if (ledvalue & LED1_LINK_1000) {
		ledlink |= GMII_LED1_LINK_1000;
	}
	if (ledvalue & LED2_LINK_1000) {
		ledlink |= GMII_LED2_LINK_1000;
	}

	OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_LED_ACTIVE,
	    2, ledact, &err, usberr);

	OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_LED_LINK,
	    2, ledlink, &err, usberr);

	OUT_VAL(dp, ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_PAGE_SELECT,
	    2, GMII_PHY_PAGE_SELECT_PAGE(0), &err, usberr);

	/* LED full duplex setting */
	ledfd = 0;
	if (ledvalue & LED0_FD) {
		ledfd |= 0x01;
	}
	else if ((ledvalue & LED0_USB3_MASK) == 0) {
		ledfd |= 0x02;
	}


	if (ledvalue & LED1_FD) {
		ledfd |= 0x04;
	}
	else if ((ledvalue & LED1_USB3_MASK) == 0) {
		ledfd |= 0x08;
	}

	if (ledvalue & LED2_FD) {
		/* LED2_FD */
		ledfd |= 0x10;
	}
	else if ((ledvalue & LED2_USB3_MASK) == 0) {
		/* LED2_USB3 */
		ledfd |= 0x20;
	}

	OUT_VAL(dp, ACCESS_MAC, 0x73, 1, 1, ledfd, &err, usberr);

	err = USB_SUCCESS;
usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_AutoDetach(struct usbgem_dev *dp)
{
	int		err = USB_SUCCESS;
	uint16_t	tmp16 = 0;
	uint8_t		tmp8 = 0;

	IN_VAL(dp, ACCESS_EEPROM, 0x43, 1, 2, &tmp16, &err, usberr);

	if ((tmp16 == 0xffff) || (!(tmp16 & 0x0100))) {
		DPRINTF(2, (CE_CONT, "!%s: %s: auto_detach disabled(0x%04x)",
		    dp->name, __func__, tmp16));
		err = USB_SUCCESS;
		goto usberr;
	}

	/* Enable Auto Detach bit */
	IN_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1, &tmp8, &err, usberr);
	tmp8 |= CLKSELECT_ULR;
	OUT_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1, tmp8, &err, usberr);

	IN_VAL(dp, ACCESS_MAC, PHYPWR, 2, 2, &tmp16, &err, usberr);
	tmp16 |= PHYPWR_AUTODETACH;
	OUT_VAL(dp, ACCESS_MAC, PHYPWR, 2, 2, tmp16, &err, usberr);

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));

	return (err);
}

static int
axg_reset_phy(struct usbgem_dev *dp)
{
	int	err;
	struct axg_dev	*lp = dp->private;

	if (lp->phy_has_reset) {
		return (USB_SUCCESS);
	}

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* Power up ethernet PHY */
	OUT_VAL(dp, ACCESS_MAC, PHYPWR, 2, 2,
	    0, &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, PHYPWR, 2, 2,
	    PHYPWR_IPRL,
	    &err, usberr);
	delay(drv_usectohz(200*1000));

	/* reset phy once after power-up */
	lp->phy_has_reset = B_TRUE;

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_reset_chip(struct usbgem_dev *dp)
{
	int		err = USB_SUCCESS;
	uint8_t		buf[2];
	uint_t		val;
	struct axg_dev	*lp = dp->private;

	lp->phy_has_reset = B_FALSE;

	OUT_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1,
	    0,
	    &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1,
	    CLKSELECT_ACS | CLKSELECT_BCS,
	    &err, usberr);
	delay(drv_usectohz(100*1000));

	lp->rxctl = 0;
	OUT_VAL(dp, ACCESS_MAC, RXCTL, 2, 2, lp->rxctl, &err, usberr);

	lp->mode = 0;	/* i.e. 10M half */
	OUT_VAL(dp, ACCESS_MAC, MEDIUM, 2, 2, lp->mode, &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, BULKINQCTRL, 1, 1, 0, &err, usberr);

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

/*
 * Setup ax88172
 */
static int
axg_init_chip(struct usbgem_dev *dp)
{
	int		i;
	uint32_t	val;
	int		err = USB_SUCCESS;
	uint16_t	reg;
	uint8_t		buf[2];
	uint16_t	tmp16;
	struct axg_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#if 0
	/* for stop->start cycle */
	(void) axg_reset_phy(dp);
#endif
	/* power up mac */
	OUT_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1,
	    CLKSELECT_ACS | CLKSELECT_BCS,
	    &err, usberr);
	delay(drv_usectohz(100*1000));

	/* RX bulk configuration, default for USB3.0 to Giga */
	OUT(dp, ACCESS_MAC, BULKINQCTRL, 5, 5,
	    &AX88179_BULKIN_SIZE[0],
	    &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, PAUSE_LOW, 1, 1,
	    0x34,
	    &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, PAUSE_HIGH, 1, 1,
	    0x52,
	    &err, usberr);

	/* Enable checksum offload */
	OUT_VAL(dp, ACCESS_MAC, RXCOE, 1, 1,
#ifdef CONFIG_CKSUM_OFFLOAD
	    RXCOE_IP | RXCOE_TCP | RXCOE_UDP | RXCOE_ICMP |
	    RXCOE_IGMP | RXCOE_TCPV6 | RXCOE_UDPV6 | RXCOE_ICMPV6,
#else
	    0,
#endif
	    &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, TXCOE, 1, 1,
#ifdef CONFIG_CKSUM_OFFLOAD
	    TXCOE_IP | TXCOE_TCP | TXCOE_UDP |
	    TXCOE_TCPV6 | TXCOE_UDPV6,
#else
	    0,
#endif
	    &err, usberr);

	/* XXX - Don't touch RX_CTL register */

	/* XXX - Don't touch MEDIUM register */

	axg_led_setting(dp);
	axg_EEE_setting(dp);
	axg_Gether_setting(dp);
	axg_AutoDetach(dp);

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}

static int
axg_start_chip(struct usbgem_dev *dp)
{
	int	err = USB_SUCCESS;
	struct axg_dev	*lp = dp->private;

	/* nothing to do */
usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));

	return (err);
}

static int
axg_stop_chip(struct usbgem_dev *dp)
{
	uint16_t	val16;
	int		err = USB_SUCCESS;
	struct axg_dev	*lp = dp->private;

	/* stop mac */
	lp->mode &= ~MEDIUM_RECEIVE_EN;
	OUT_VAL(dp, ACCESS_MAC, MEDIUM, 2, 2,
	    lp->mode, &err, usberr);

	/* stop rx */
	lp->rxctl = 0;
	OUT_VAL(dp, ACCESS_MAC, RXCTL, 2, 2,
	    lp->rxctl, &err, usberr);

	/* power down mac */
	OUT_VAL(dp, ACCESS_MAC, CLKSELECT, 1, 1,
	    0, &err, usberr);

	/* power down ethernet PHY */
	OUT_VAL(dp, ACCESS_MAC, PHYPWR, 2, 2,
	    PHYPWR_BZ | PHYPWR_IPRL, &err, usberr);

	delay(drv_usectohz(200));

	lp->phy_has_reset = B_FALSE;

usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end (%s)",
	    dp->name, __func__,
	    err == USB_SUCCESS ? "success" : "error"));

	return (err);
}

static int
axg_get_stats(struct usbgem_dev *dp)
{
	/* EMPTY */
	return (USB_SUCCESS);
}

static uint_t
axg_mcast_hash(struct usbgem_dev *dp, const uint8_t *addr)
{
	return (usbgem_ether_crc_be(addr) >> (32 - 6));
}

static int
axg_set_rx_filter(struct usbgem_dev *dp)
{
	int		i;
	uint8_t		mhash[8];
	uint8_t		physaddr[ETHERADDRL];
	uint_t		h;
#ifdef DEBUG_LEVEL
	uint16_t	tmp16;
#endif
	int		err = USB_SUCCESS;
	struct axg_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called, rxmode:0x%b",
	    dp->name, __func__, dp->rxmode, RXMODE_BITS));

	bzero(mhash, sizeof (mhash));
	bzero(physaddr, sizeof (physaddr));

	lp->rxctl = 0;
	if (dp->rxmode & RXMODE_ENABLE) {
		lp->rxctl |= RXCTL_START;
		bcopy(dp->cur_addr.ether_addr_octet, physaddr, ETHERADDRL);
	}

	lp->rxctl |= RXCTL_AB;

	if (dp->rxmode & RXMODE_PROMISC) {
		/* promiscious mode implies all multicast and all physical */
		lp->rxctl |= RXCTL_PRO;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		/* accept all multicast packets */
		lp->rxctl |= RXCTL_AMALL;
	} else if (dp->mc_count > 0) {
		/*
		 * make hash table to select interresting
		 * multicast address only.
		 */
		lp->rxctl |= RXCTL_AM;
		for (i = 0; i < dp->mc_count; i++) {
			h = dp->mc_list[i].hash;
			mhash[h / 8] |= 1 << (h % 8);
		}
	}

	OUT(dp, ACCESS_MAC, NODEID, ETHERADDRL, ETHERADDRL,
	    physaddr, &err, usberr);

	OUT(dp, ACCESS_MAC, MCASTFILTER_ARRY,
	    MCASTFILTER_SIZE, MCASTFILTER_SIZE, &mhash[0], &err, usberr);

	OUT_VAL(dp, ACCESS_MAC, RXCTL, 2, 2,
	    (lp->rxctl & RXCTL_START) ? lp->rxctl : 0 ,
	    &err, usberr);

#ifdef DEBUG_LEVEL
	IN_VAL(dp, ACCESS_MAC, RXCTL, 2, 2, &tmp16, &err, usberr);
#endif
usberr:
	DPRINTF(2, (CE_CONT, "!%s: %s: end rxctl:0x%04b (%s)",
	    dp->name, __func__, tmp16, RXCTL_BITS,
	    err == USB_SUCCESS ? "success" : "error"));

	return (err);
}

static int
axg_set_media(struct usbgem_dev *dp)
{
	int		err = USB_FAILURE;
	struct axg_dev	*lp = dp->private;
	int		i;
	uint32_t	tmp32;

	lp->rxctl |= RXCTL_START;

	i = 10;
again:
	OUT_VAL(dp, ACCESS_MAC, RXCTL, 2, 2,
	    lp->rxctl,
	    &err, usberr);

	/*
	 * check the usb device control TX FIFO full
	 * or empty
	 */
	IN_VAL(dp, 0x81, 0x8c, 0, 4, &tmp32, &err, usberr);

	if ((tmp32 & 0x40000000) != 0) {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: reg:%x i:%d\n",
		    dp->name, __func__, tmp32, i));
		if (i-- < 0) {
			/* time over */
			cmn_err(CE_WARN, "!%s: %s: timeout",
			    dp->name, __func__);
			goto usberr;
		}

		OUT_VAL(dp, ACCESS_MAC, RXCTL, 2, 2,
		    0, &err, usberr);
		goto again;
	}

	lp->mode = MEDIUM_RECEIVE_EN;

	if (dp->full_duplex) {
		lp->mode |= MEDIUM_FULL_DUPLEX;

		switch (dp->flow_control) {
		case FLOW_CONTROL_TX_PAUSE:
			lp->mode |= MEDIUM_TXFLOW_CTRLEN;
			break;

		case FLOW_CONTROL_SYMMETRIC:
			lp->mode |= MEDIUM_TXFLOW_CTRLEN | MEDIUM_RXFLOW_CTRLEN;
			break;

		case FLOW_CONTROL_RX_PAUSE:
			lp->mode |= MEDIUM_RXFLOW_CTRLEN;
			break;
		}
	}
#ifdef CONFIG_VLAN
	lp->mode |= MEDIUM_JUMBO_EN;
#else
	if (dp->mtu > 1500) {
		lp->mode |= MEDIUM_JUMBO_EN;
	}
#endif
	i = 3;	/* default index of bulkin configuration */
	switch (dp->speed) {
	case USBGEM_SPD_1000:
		lp->mode |= MEDIUM_GIGAMODE | MEDIUM_EN_125MHZ;

		/* configure bulk-in mode */
		if (lp->usb_link_status & LINKSTATUS_USB_SS) {
			i = 0;
		} else if (lp->usb_link_status & LINKSTATUS_USB_HS) {
			i = 1;
		}
		break;

	case USBGEM_SPD_100:
		lp->mode |= MEDIUM_PS;

		/* configure bulk-in mode */
		if (lp->usb_link_status & (LINKSTATUS_USB_SS | LINKSTATUS_USB_HS)) {
			i = 2;
		}
		break;

	case USBGEM_SPD_10:
		/* nothing to do */
		break;
	}

	/* RX bulk configuration */
	OUT(dp, ACCESS_MAC, BULKINQCTRL, 5, 5,
	    &AX88179_BULKIN_SIZE[i], &err, usberr);

	DPRINTF(10, (CE_CONT, "!%s:%s Write medium type: 0x%04b",
	    dp->name, __func__, lp->mode, MEDIUM_BITS));

	/* Configure current medium type */
	OUT_VAL(dp, ACCESS_MAC, MEDIUM, 2, 2, lp->mode,
	    &err, usberr);

	err = USB_SUCCESS;

usberr:
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: media_status:0x%04b end (%s)",
	    dp->name, __func__, lp->mode, MEDIUM_BITS,
	    err == USB_SUCCESS ? "success" : "error"));
	return (err);
}


/*
 * send/receive packet check
 */
static mblk_t *
axg_tx_make_packet(struct usbgem_dev *dp, mblk_t *mp)
{
	struct axg_dev	*lp = dp->private;
	int		n;
	size_t		len;
	mblk_t		*new;
	mblk_t		*tp;
	uint8_t		*bp;
	size_t		header_size;
	uint32_t	mss = 0;

	len = msgdsize(mp);

#ifdef CONFIG_CKSUM_OFFLOAD
	if (DB_CKSUMFLAGS(mp) & HW_LSO) {
		mss = DB_LSOMSS(mp);
	}
#if defined(DEBUG_LEVEL) && DEBUG_LEVEL > 0
	if (mss > 0) {
		cmn_err(CE_CONT, "!%s: %s: called, pktlen:%d, mss:%d",
		    dp->name, __func__, len, mss);
		usbgem_dump_packet(dp, __func__, mp, B_TRUE);
	}
#endif
#endif
	header_size = 8;

	/* re-allocate the mblk */
	if ((new = allocb(header_size + len, 0)) == NULL) {
		goto x;
	}

	/* constuct new packet */
	bp = new->b_rptr;

	/* add a header */
	LE_OUT32(bp, len);	/* word 0: packet length w/o header */
	LE_OUT32(bp + 4, mss);	/* word 1: mss info for LSO */
	bp += header_size;

	/* copy contents of the buffer */
	for (tp = mp; tp; tp = tp->b_cont) {
		n = tp->b_wptr - tp->b_rptr;
		bcopy(tp->b_rptr, bp, n);
		bp += n;
	}

	/* close the payload of the packet */
	new->b_wptr = bp;
x:
	return (new);
}

static void
axg_dump_packet(struct usbgem_dev *dp, uint8_t *bp, int n)
{
	int	i;
	int	mp;

	for (i = 0; i < n; i += 8, bp += 8) {
		cmn_err(CE_CONT, "%02x %02x %02x %02x %02x %02x %02x %02x",
		    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7]);
	}
}

#define	PKT_HEADER_SIZE	16

static mblk_t *
axg_rx_make_packet(struct usbgem_dev *dp, mblk_t *mp)
{
	mblk_t		*tp;
	mblk_t		*mp_ret;
	mblk_t		**mpp;
	int		desc_cnt;
	size_t		desc_off;
	uint8_t		*descp;
	uint8_t		*bp;
	uint32_t	desc;
	size_t		len;

	DPRINTF(10, (CE_CONT, "!%s: %s: called, pktlen:%d",
	    dp->name, __func__, msgdsize(mp)));

#ifdef DEBUG_LEVEL
	if (axg_debug > 10) {
		axg_dump_packet(dp, mp->b_rptr, msgdsize(mp));
	}
#endif
	if (msgdsize(mp) < 8) {
		cmn_err(CE_CONT, "!%s: %s: called, pktlen:%d",
		    dp->name, __func__, msgdsize(mp));
		axg_dump_packet(dp, mp->b_rptr, msgdsize(mp));
		return (NULL);
	}

	/* base of usb packet info */
	mp->b_wptr -= 4;

	/* the number of descriptors for ethernet packets */
	desc_cnt = LE_IN16(mp->b_wptr);

	/* base of descriptor array */
	descp = mp->b_rptr + LE_IN16(mp->b_wptr + 2);

	tp = mp;
	mpp = &mp_ret;
	bp = mp->b_rptr;

	while (desc_cnt--) {
		desc = LE_IN32(descp);
		DPRINTF(9, (CE_CONT,
		    "!%s: %s: rxdesc:%b L3:%d L4:%d\n",
		    dp->name, __func__,
		    desc, RXDESC_BITS,
		    (RXDESC_L3_TYPE_MASK & desc) >> RXDESC_L3_TYPE_SHIFT,
		    (RXDESC_L4_TYPE_MASK & desc) >> RXDESC_L4_TYPE_SHIFT));

		len = (desc & RXDESC_LEN) >> RXDESC_LEN_SHIFT;

		if (len < ETHERMIN) {
			cmn_err(CE_NOTE, "!%s: %s: packet is too short len:%d",
			    dp->name, __func__, len);
			dp->stats.errrcv++;
			dp->stats.runt++;
			break;
		}
		if (len > sizeof (struct ether_header)
		    + sizeof (struct ether_vlan_header) + dp->mtu +  ETHERFCSL) {
			dp->stats.errrcv++;
			dp->stats.frame_too_long++;
			break;
		}

		/* Check CRC or runt packet */
		if ((desc & (RXDESC_CRC_ERR | RXDESC_DROP_ERR)) == 0) {

			if (tp == NULL) {
				tp = dupb(mp);
				if (tp == NULL) {
					/* no memory */
					dp->stats.errrcv++;
					dp->stats.norcvbuf++;
					break;
				}
			}

			/* analyse the header of the received usb frame */
			tp->b_rptr = bp;
			tp->b_wptr = bp + len;

			*mpp = tp;
			mpp = &tp->b_next;

			tp = NULL;
		} else {
			dp->stats.errrcv++;
			if (desc & RXDESC_CRC_ERR) {
				dp->stats.crc++;
			}
			if (desc & RXDESC_DROP_ERR) {
				dp->stats.runt++;
			}
		}

		bp += P2ROUNDUP(len,  8);
		descp += 4;
	}

	*mpp = NULL;

	return (mp);
}

/*
 * MII Interfaces
 */
static uint16_t
axg_mii_read(struct usbgem_dev *dp, uint_t index, int *errp)
{
	uint16_t	val;
	int		err;

	DPRINTF(4, (CE_CONT, "!%s: %s: called, ix:%d",
	    dp->name, __func__, index));
#ifdef notdef
	IN_VAL(dp, ACCESS_MAC, LINKSTATUS, 1, 1, &val,
	    &err, usberr);
#endif


	IN_VAL(dp, ACCESS_PHY, dp->mii_phy_addr, index, 2, &val,
	    errp, usberr);

	return (val);

usberr:
	cmn_err(CE_CONT,
	    "!%s: %s: usberr(%d) detected", dp->name, __func__, *errp);

	return (0);
}

static void
axg_mii_write(struct usbgem_dev *dp, uint_t index, uint16_t val, int *errp)
{
	DPRINTF(4, (CE_CONT, "!%s: %s called, reg:%x val:%x",
	    dp->name, __func__, index, val));

	OUT_VAL(dp, ACCESS_PHY, dp->mii_phy_addr, index, 2, val,
	    errp, usberr);

usberr:
	;
}

static void
axg_interrupt(struct usbgem_dev *dp, mblk_t *mp)
{
	uint8_t	new;
	uint8_t		*bp = mp->b_rptr;
	struct axg_dev	*lp = dp->private;

	if (msgdsize(mp) < 8) {
		return;
	}

	if (bcmp(lp->last_int_data, bp, 8) != 0) {
		DPRINTF(2, (CE_CONT,
		    "!%s: %s: size:%d, %02x %02x %02x %02x %02x %02x %02x %02x",
		    dp->name, __func__, mp->b_wptr - mp->b_rptr,
		    bp[0], bp[1], bp[2], bp[3], bp[4], bp[5], bp[6], bp[7]));
		bcopy(bp, lp->last_int_data, 8);
	}

	new = LE_IN8(mp->b_rptr + 2);

	if ((lp->last_link_state ^ new) & INT_PPLS_LINK) {
		usbgem_mii_update_link(dp);
	}

	lp->last_link_state = new;
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
#ifdef DEBUG_LEVEL
static void
axg_eeprom_dump(struct usbgem_dev *dp, int size)
{
	int		i;
	int		err;
	uint16_t	tmp16[8];

	cmn_err(CE_CONT, "!%s: eeprom dump:", dp->name);

	for (i = 0; i < size; i += 8) {
		IN(dp, ACCESS_EEPROM, i, 1*8, 2*8, &tmp16[0], &err, usberr);
		cmn_err(CE_CONT, "!%x : %04x %04x %04x %04x %04x %04x %04x %04x",
		    i,
		    tmp16[0], tmp16[1], tmp16[2], tmp16[3],
		    tmp16[4], tmp16[5], tmp16[6], tmp16[7]);
	}
usberr:
	;
}
#endif

static int
axg_attach_chip(struct usbgem_dev *dp)
{
	int	err;
	struct axg_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	err = USB_SUCCESS;

	/*
	 * Read the factory mac address in EEPROM
	 */

	IN(dp, ACCESS_EEPROM, 0, ETHERADDRL/2, ETHERADDRL,
	    dp->dev_addr.ether_addr_octet, &err, usberr);

	dp->rx_buf_len = (24 + 2) * 1024;
#ifdef CONFIG_CKSUM_OFFLOAD
	dp->misc_flag |= USBGEM_CKSUM_FULL_IPv4 | USBGEM_CKSUM_HEADER_IPv4
#if 0
	    | USBGEM_LSO_IPv4	/* LSO does't work */
#endif
	    ;
#endif
#ifdef CONFIG_VLAN
	dp->misc_flag |= USBGEM_VLAN;
#endif
#if DEBUG_LEVEL > 0
	axg_eeprom_dump(dp, 0x80);
#endif
	return (err);

usberr:
	cmn_err(CE_WARN, "%s: %s: usb error detected (%d)",
	    dp->name, __func__, err);
	return (USB_FAILURE);
}

static int
axg_mii_probe(struct usbgem_dev *dp)
{
	struct axg_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	(void) axg_reset_phy(dp);

	/* no need to scan built-in phy */
	dp->mii_phy_addr = AX88179_PHY_ID;

	return (usbgem_mii_probe_default(dp));
}

static int
axg_mii_init(struct usbgem_dev *dp)
{
	struct axg_dev	*lp = dp->private;

	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* required for hot plug */
	lp->phy_has_reset = B_FALSE;
	(void) axg_reset_phy(dp);

	return (USB_SUCCESS);
}

static int
axgattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
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
	struct usbgem_dev	*dp;
	void			*base;
	struct usbgem_conf	*ugcp;
	struct axg_dev		*lp;

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

		for (i = 0, p = chiptbl_8817x; i < CHIPTABLESIZE; i++, p++) {
			if (p->vid == vid && p->pid == pid) {
				/* found */
				cmn_err(CE_CONT, "!%s%d: %s "
				    "(vid: 0x%04x, did: 0x%04x, revid: 0x%02x)",
				    drv_name, unit, p->name, vid, pid, revid);
				goto chip_found;
			}
		}

		/* Not found */
		cmn_err(CE_WARN, "!%s: %s: wrong usb venid/prodid (0x%x, 0x%x)",
		    drv_name, __func__, vid, pid);

		/* assume 88772 */
		p = &chiptbl_8817x[CHIPTABLESIZE - 1];
chip_found:
		/*
		 * construct usbgem configration
		 */
		ugcp = kmem_zalloc(sizeof (*ugcp), KM_SLEEP);

		/* name */
		/*
		 * softmac requires that ppa is the instance number
		 * of the device, otherwise it hangs in seaching the device.
		 */
		sprintf(ugcp->usbgc_name, "%s%d", drv_name, unit);
		ugcp->usbgc_ppa = unit;

		ugcp->usbgc_ifnum = 0;
		ugcp->usbgc_alt = 0;

		ugcp->usbgc_tx_list_max = 256;

		ugcp->usbgc_rx_header_len = 0;
		ugcp->usbgc_rx_list_max = 64;

		/* time out parameters */
		ugcp->usbgc_tx_timeout = USBGEM_TX_TIMEOUT;
		ugcp->usbgc_tx_timeout_interval = USBGEM_TX_TIMEOUT_INTERVAL;

		/* flow control */
		/*
		 * XXX - flow control caused link down frequently under
		 * heavy traffic
		 */
		ugcp->usbgc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* MII timeout parameters */
		ugcp->usbgc_mii_link_watch_interval = ONESEC;
		ugcp->usbgc_mii_an_watch_interval = ONESEC/5;
		ugcp->usbgc_mii_reset_timeout = MII_RESET_TIMEOUT; /* 1 sec */
		ugcp->usbgc_mii_an_timeout = MII_AN_TIMEOUT;	/* 5 sec */
		ugcp->usbgc_mii_an_wait = 0;
		ugcp->usbgc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		ugcp->usbgc_mii_an_delay = ONESEC/10;
		ugcp->usbgc_mii_linkdown_action = MII_ACTION_RSA;
		ugcp->usbgc_mii_linkdown_timeout_action = MII_ACTION_RESET;
		ugcp->usbgc_mii_dont_reset = B_FALSE;
		ugcp->usbgc_mii_hw_link_detection = B_TRUE;
		ugcp->usbgc_mii_stop_mac_on_linkdown = B_TRUE;

		/* I/O methods */

		/* mac operation */
		ugcp->usbgc_attach_chip = &axg_attach_chip;
		ugcp->usbgc_reset_chip = &axg_reset_chip;
		ugcp->usbgc_init_chip = &axg_init_chip;
		ugcp->usbgc_start_chip = &axg_start_chip;
		ugcp->usbgc_stop_chip = &axg_stop_chip;
		ugcp->usbgc_multicast_hash = &axg_mcast_hash;

		ugcp->usbgc_set_rx_filter = &axg_set_rx_filter;
		ugcp->usbgc_set_media = &axg_set_media;
		ugcp->usbgc_get_stats = &axg_get_stats;
		ugcp->usbgc_interrupt = &axg_interrupt;

		/* packet operation */
		ugcp->usbgc_tx_make_packet = &axg_tx_make_packet;
		ugcp->usbgc_rx_make_packet = &axg_rx_make_packet;

		/* mii operations */
		ugcp->usbgc_mii_probe = &axg_mii_probe;
		ugcp->usbgc_mii_init = &axg_mii_init;
		ugcp->usbgc_mii_config = &usbgem_mii_config_default;
		ugcp->usbgc_mii_read = &axg_mii_read;
		ugcp->usbgc_mii_write = &axg_mii_write;

		/* mtu */
		ugcp->usbgc_min_mtu = ETHERMTU;
		ugcp->usbgc_max_mtu = ETHERMTU;
		ugcp->usbgc_default_mtu = ETHERMTU;

		lp = kmem_zalloc(sizeof (struct axg_dev), KM_SLEEP);
		lp->chip = p;
		lp->last_link_state = 0;
		lp->phy_has_reset = B_FALSE;

		dp = usbgem_do_attach(dip, ugcp, lp, sizeof (struct axg_dev));

		kmem_free(ugcp, sizeof (*ugcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

err_free_mem:
		kmem_free(lp, sizeof (struct axg_dev));
err_close_pipe:
err:
		return (DDI_FAILURE);
	}

	if (cmd == DDI_RESUME) {
		dp = USBGEM_GET_DEV(dip);
		lp = dp->private;
#if 0
		/* force to reset phy */
		lp->phy_has_reset = B_FALSE;
#endif
		return (usbgem_resume(dip));
	}

	return (DDI_FAILURE);
}

static int
axgdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	DPRINTF(2, (CE_CONT, "!%s%d: %s: called, cmd:%d",
	    ddi_driver_name(dip), ddi_get_instance(dip),
	    __func__, cmd));

	if (cmd == DDI_DETACH) {
		ret = usbgem_do_detach(dip);
		if (ret != DDI_SUCCESS) {
			return (DDI_FAILURE);
		}
		return (DDI_SUCCESS);
	}
	if (cmd == DDI_SUSPEND) {
		return (usbgem_suspend(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef USBGEM_CONFIG_GLDv3
USBGEM_STREAM_OPS(axg_ops, axgattach, axgdetach);
#else
static	struct module_info axgminfo = {
	0,			/* mi_idnum */
	"axg",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	ETHERMTU*128,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit axgrinit = {
	(int (*)()) NULL,	/* qi_putp */
	usbgem_rsrv,		/* qi_srvp */
	usbgem_open,		/* qi_qopen */
	usbgem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&axgminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit axgwinit = {
	usbgem_wput,		/* qi_putp */
	usbgem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&axgminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	axg_info = {
	&axgrinit,	/* st_rdinit */
	&axgwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_axg_ops = {
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
	&axg_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops axg_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	usbgem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	axgattach,	/* devo_attach */
	axgdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_axg_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	usbgem_power,	/* devo_power */
#if DEVO_REV >= 4
	usbgem_quiesce,	/* devo_quiesce */
#endif
};
#endif

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&axg_ops,	/* driver ops */
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

	DPRINTF(2, (CE_CONT, "!axg: _init: called"));

	status = usbgem_mod_init(&axg_ops, "axg");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		usbgem_mod_fini(&axg_ops);
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

	DPRINTF(2, (CE_CONT, "!axg: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		usbgem_mod_fini(&axg_ops);
	}
	status;
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
