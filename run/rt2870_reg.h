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

#ifndef _RT2870_REG_H
#define	_RT2870_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#define RT2860_USB_DMA_CFG		0x02a0	/* RT2870 only */

#define	RT2860_H2M_MAILBOX_CID		0x7014
#define RT2860_H2M_MAILBOX_STATUS	0x701c
#define RT2860_H2M_BBPAGENT		0x7028
#define RT2860_BCN_BASE(vap)		(0x7800 + (vap) * 512)

/* possible flags for register TXRXQ_PCNT */
#define RT2860_RX0Q_PCNT_MASK	0xff000000
#define RT2860_TX2Q_PCNT_MASK	0x00ff0000
#define RT2860_TX1Q_PCNT_MASK	0x0000ff00
#define RT2860_TX0Q_PCNT_MASK	0x000000ff

/* possible flags for register USB_DMA_CFG */
#define RT2860_USB_TX_BUSY		(1U << 31)
#define RT2860_USB_RX_BUSY		(1U << 30)
#define RT2860_USB_EPOUT_VLD_SHIFT	24
#define RT2860_USB_TX_EN		(1U << 23)
#define RT2860_USB_RX_EN		(1U << 22)
#define RT2860_USB_RX_AGG_EN		(1U << 21)
#define RT2860_USB_TXOP_HALT		(1U << 20)
#define RT2860_USB_TX_CLEAR		(1U << 19)
#define RT2860_USB_PHY_WD_EN		(1U << 16)
#define RT2860_USB_PHY_MAN_RST		(1U << 15)
#define RT2860_USB_RX_AGG_LMT(x)	((x) << 8)	/* in unit of 1KB */
#define RT2860_USB_RX_AGG_TO(x)		((x) & 0xff)	/* in unit of 33ns */


#define	RT2870_FW_BASE		0x3000

#define RT2860_RF_2820	1	/* 2T3R */
#define RT2860_RF_2850	2	/* dual-band 2T3R */
#define RT2860_RF_2720	3	/* 1T2R */
#define RT2860_RF_2750	4	/* dual-band 1T2R */
#define RT3070_RF_3020	5	/* 1T1R */
#define RT3070_RF_2020	6	/* b/g */
#define RT3070_RF_3021	7	/* 1T2R */
#define RT3070_RF_3022	8	/* 2T2R */
#define RT3070_RF_3052	9	/* dual-band 2T2R */

/* USB commands for RT2870 only */
#define	RT2870_RESET		1
#define	RT2870_WRITE_2		2
#define	RT2870_WRITE_REGION_1	6
#define	RT2870_READ_REGION_1	7
#define	RT2870_EEPROM_READ	9

#define RT2870_DEF_MAC					\
	{ RT2860_BCN_OFFSET0,		0xf8f0e8e0 },	\
	{ RT2860_LEGACY_BASIC_RATE,	0x0000013f },	\
	{ RT2860_HT_BASIC_RATE,		0x00008003 },	\
	{ RT2860_MAC_SYS_CTRL,		0x00000000 },	\
	{ RT2860_RX_FILTR_CFG,		0x00017f97 },	\
	{ RT2860_BKOFF_SLOT_CFG,	0x00000209 },	\
	{ RT2860_TX_SW_CFG0,		0x00000000 },	\
	{ RT2860_TX_SW_CFG1,		0x00080606 },	\
	{ RT2860_TX_LINK_CFG,		0x00001020 },	\
	{ RT2860_TX_TIMEOUT_CFG,	0x000a2090 },	\
	{ RT2860_MAX_LEN_CFG,		3840 | 0x1000 }, \
	{ RT2860_LED_CFG,		0x7f031e46 },	\
	{ RT2860_MAX_PCNT,		0x1f3fbf9f },	\
	{ RT2860_TX_RTY_CFG,		0x47d01f0f },	\
	{ RT2860_AUTO_RSP_CFG,		0x00000013 },	\
	/* { RT2860_WMM_AIFSN_CFG,		0x00002273 }, */	\
	/* { RT2860_WMM_CWMIN_CFG,		0x00002344 }, */	\
	/* { RT2860_WMM_CWMAX_CFG,		0x000034aa }, */	\
	{ RT2860_CCK_PROT_CFG,		0x05740003 },	\
	{ RT2860_OFDM_PROT_CFG,		0x05740003 },	\
	{ RT2860_PBF_CFG,		0x00f40006 },	\
	{ RT2860_MM40_PROT_CFG,		0x03f44084 },	\
	{ RT2860_WPDMA_GLO_CFG,		0x00000030 },	\
	{ RT2860_GF20_PROT_CFG,		0x01744004 },	\
	{ RT2860_GF40_PROT_CFG,		0x03f44084 },	\
	{ RT2860_MM20_PROT_CFG,		0x01744004 },	\
	{ RT2860_TXOP_CTRL_CFG,		0x0000583f },	\
	{ RT2860_TX_RTS_CFG,		0x00092b20 },	\
	{ RT2860_EXP_ACK_TIME,		0x002400ca },	\
	{ RT2860_TXOP_HLDR_ET,		0x00000002 },	\
	{ RT2860_XIFS_TIME_CFG,		0x33a41010 },	\
	{ RT2860_PWR_PIN_CFG,		0x00000003 }

/* RT2870 RX descriptor */
struct rt2870_rxd {
	/* single 32-bit field */
	volatile uint32_t	flags;
};
#ifdef notdef
#define	RT2870_RX_FLAG_BITS	"\020"\
	"\021DEC"	\
	"\020AMPDU"	\
	"\017L2PAD"	\
	"\016RSSI"	\
	"\015HTC" 	\
	"\014AMSDU"	\
	"\013MICERR"	\
	"\012ICVERR"	\
	"\011CRCERR"	\
	"\010MYBSS"	\
	"\007BC"	\
	"\006MC"	\
	"\005UC2ME"	\
	"\004FRAG"	\
	"\003NULL"	\
	"\002DATA"	\
	"\001BA"
#endif

/* RT2870 TX descriptor */
struct rt2870_txd {
	volatile uint16_t	len;
	volatile uint8_t	pad;
	volatile uint8_t	flags;
#define	RT2870_TX_QSEL_SHIFT	1
#define	RT2870_TX_QSEL_MGMT	(0 << 1)
#define	RT2870_TX_QSEL_HCCA	(1 << 1)
#define	RT2870_TX_QSEL_EDCA	(2 << 1)
#define	RT2870_TX_WIV		(1 << 0)
#define	RT2870_TX_NEXT		0x40
#define	RT2870_TX_BURST		0x80
};


/* TX Wireless Information */
struct rt2870_txwi {
	uint8_t		flags;
	uint8_t		txop;
	uint16_t	phy;
	uint8_t		xflags;
	uint8_t		wcid;	/* Wireless Client ID */
	uint16_t	len;
	uint32_t	iv;
	uint32_t	eiv;
};


#define RT3070_OPT_14			0x0114

/* RT3070 registers */
#define RT3070_RF_CSR_CFG		0x0500
#define RT3070_EFUSE_CTRL		0x0580
#define RT3070_EFUSE_DATA0		0x0590
#define RT3070_EFUSE_DATA1		0x0594
#define RT3070_EFUSE_DATA2		0x0598
#define RT3070_EFUSE_DATA3		0x059c
#define RT3070_LDO_CFG0			0x05d4
#define RT3070_GPIO_SWITCH		0x05dc

#define RT3070_RF_KICK		(1U << 17)
#define RT3070_RF_WRITE		(1U << 16)

/* possible flags for register EFUSE_CTRL */
#define RT3070_SEL_EFUSE	(1U << 31)
#define RT3070_EFSROM_KICK	(1U << 30)
#define RT3070_EFSROM_AIN_MASK	0x03ff0000
#define RT3070_EFSROM_AIN_SHIFT	16
#define RT3070_EFSROM_MODE_MASK	0x000000c0
#define RT3070_EFUSE_AOUT_MASK	0x0000003f

#define RT3070_RF_BLOCK	(1U << 0)
#define RT3070_RX0_PD	(1U << 2)
#define RT3070_TX0_PD	(1U << 3)
#define RT3070_RX1_PD	(1U << 4)
#define RT3070_TX1_PD	(1U << 5)

/* possible flags for RT3020 RF register 15 */
#define RT3070_TX_LO2	(1U << 3)

/* possible flags for RT3020 RF register 17 */
#define RT3070_TX_LO1	(1U << 3)

/* possible flags for RT3020 RF register 20 */
#define RT3070_RX_LO1	(1U << 3)

/* possible flags for RT3020 RF register 21 */
#define RT3070_RX_LO2	(1U << 3)

#ifndef RT2860_RF1
#define RT2860_RF1	0
#define RT2860_RF2	2
#define RT2860_RF3	1
#define RT2860_RF4	3
#endif

#define RT3070_RF3020	\
	{ 241, 2, 2 },	\
	{ 241, 2, 7 },	\
	{ 242, 2, 2 },	\
	{ 242, 2, 7 },	\
	{ 243, 2, 2 },	\
	{ 243, 2, 7 },	\
	{ 244, 2, 2 },	\
	{ 244, 2, 7 },	\
	{ 245, 2, 2 },	\
	{ 245, 2, 7 },	\
	{ 246, 2, 2 },	\
	{ 246, 2, 7 },	\
	{ 247, 2, 2 },	\
	{ 248, 2, 4 }

#define RT3070_DEF_RF	\
	{  4, 0x40 },	\
	{  5, 0x03 },	\
	{  6, 0x02 },	\
	{  7, 0x60 },	\
	{  9, 0x0f },	\
	{ 10, 0x41 },	\
	{ 11, 0x21 },	\
	{ 12, 0x7b },	\
	{ 14, 0x90 },	\
	{ 15, 0x58 },	\
	{ 16, 0xb3 },	\
	{ 17, 0x92 },	\
	{ 18, 0x2c },	\
	{ 19, 0x02 },	\
	{ 20, 0xba },	\
	{ 21, 0xdb },	\
	{ 24, 0x16 },	\
	{ 25, 0x01 },	\
	{ 29, 0x1f }
#ifdef __cplusplus
}
#endif

#define	RT2870_RX_FLAG_BITS	\
	"\020"	\
	"\021DEC"	\
	"\020AMPDU"	\
	"\017L2PAD"	\
	"\016RSSI"	\
	"\015HTC" 	\
	"\014AMSDU"	\
	"\013MICERR"	\
	"\012ICVERR"	\
	"\011CRCERR"	\
	"\010MYBSS"	\
	"\007BC"	\
	"\006MC"	\
	"\005UC2ME"	\
	"\004FRAG"	\
	"\003NULL"	\
	"\002DATA"	\
	"\001BA"


#endif /* _RT2870_REG_H */
