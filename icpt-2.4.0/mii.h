/*
 * mii.h : MII registers
 */

#pragma	ident	"%W% %E%"

#define	MII_CONTROL 		0
#define	MII_STATUS 		1
#define	MII_PHYIDH		2
#define	MII_PHYIDL		3
#define	MII_AN_ADVERT		4
#define	MII_AN_LPABLE		5
#define	MII_AN_EXPANSION	6
#define	MII_AN_NXTPGXMIT	7
#define	MII_AN_LPANXT		8
#define	MII_MS_CONTROL		9
#define	MII_MS_STATUS		10
#define	MII_XSTATUS		15

/* for 1000BaseT support */
#define	MII_1000TC		MII_MS_CONTROL
#define	MII_1000TS		MII_MS_STATUS

#define	MII_CONTROL_RESET	0x8000
#define	MII_CONTROL_LOOPBACK	0x4000
#define	MII_CONTROL_ANE		0x1000
#define	MII_CONTROL_PWRDN	0x0800
#define	MII_CONTROL_ISOLATE	0x0400
#define	MII_CONTROL_RSAN	0x0200
#define	MII_CONTROL_FDUPLEX	0x0100
#define	MII_CONTROL_COLTST	0x0080
#define	MII_CONTROL_SPEED	0x2040

#define	MII_CONTROL_10MB	0x0000
#define	MII_CONTROL_100MB	0x2000
#define	MII_CONTROL_1000MB	0x0040

#define	MII_CONTROL_BITS	\
	"\020"	\
	"\020RESET"	\
	"\017LOOPBACK"	\
	"\016100MB"	\
	"\015ANE"	\
	"\014PWRDN"	\
	"\013ISOLATE"	\
	"\012RSAN"	\
	"\011FDUPLEX"	\
	"\010COLTST"	\
	"\0071000M"

#define	MII_STATUS_100_BASE_T4		0x8000
#define	MII_STATUS_100_BASEX_FD		0x4000
#define	MII_STATUS_100_BASEX		0x2000
#define	MII_STATUS_10_FD		0x1000
#define	MII_STATUS_10			0x0800
#define	MII_STATUS_100_BASE_T2_FD	0x0400
#define	MII_STATUS_100_BASE_T2		0x0200
#define	MII_STATUS_XSTATUS		0x0100
#define	MII_STATUS_MFPRMBLSUPR		0x0040
#define	MII_STATUS_ANDONE		0x0020
#define	MII_STATUS_REMFAULT		0x0010
#define	MII_STATUS_CANAUTONEG		0x0008
#define	MII_STATUS_LINKUP		0x0004
#define	MII_STATUS_JABBERING		0x0002
#define	MII_STATUS_EXTENDED		0x0001

#define	MII_STATUS_BITS	\
	"\020"	\
	"\020100_BASE_T4"	\
	"\017100_BASEX_FD"	\
	"\016100_BASEX"	\
	"\01510_BASE_FD"	\
	"\01410_BASE"	\
	"\013100_BASE_T2_FD"	\
	"\012100_BASE_T2"	\
	"\011XSTATUS"	\
	"\007MFPRMBLSUPR"	\
	"\006ANDONE"	\
	"\005REMFAULT"	\
	"\004CANAUTONEG"	\
	"\003LINKUP"	\
	"\002JABBERING"	\
	"\001EXTENDED"

#define	MII_AN_ADVERT_NP	0x8000
#define	MII_AN_ADVERT_REMFAULT	0x2000
#define	MII_AN_ADVERT_SELECTOR  0x001f

#define	MII_ABILITY_ASM_DIR		0x0800	/* for annex 28B */
#define	MII_ABILITY_PAUSE		0x0400	/* for IEEE 802.3x */
#define	MII_ABILITY_100BASE_T4		0x0200
#define	MII_ABILITY_100BASE_TX_FD	0x0100
#define	MII_ABILITY_100BASE_TX		0x0080
#define	MII_ABILITY_10BASE_T_FD		0x0040
#define	MII_ABILITY_10BASE_T		0x0020

#define	MII_AN_LPABLE_NP	0x8000

#define	MII_STATUS_ABILITY	\
	(MII_STATUS_100_BASE_T4	|	\
	 MII_STATUS_100_BASEX_FD |	\
	 MII_STATUS_100_BASEX |	\
	 MII_STATUS_10 |	\
	 MII_STATUS_10_FD)

#define	MII_ABILITY	\
	(MII_ABILITY_ASM_DIR |	\
	 MII_ABILITY_PAUSE |	\
	 MII_ABILITY_100BASE_T4	|	\
	 MII_ABILITY_100BASE_TX_FD |	\
	 MII_ABILITY_100BASE_TX |	\
	 MII_ABILITY_10BASE_T |	\
	 MII_ABILITY_10BASE_T_FD)

#define	MII_ABILITY_BITS	\
	"\020"	\
	"\014ASM_DIR"	\
	"\013PAUSE"	\
	"\012100BASE_T4"	\
	"\011100BASE_TX_FD"	\
	"\010100BASE_TX"	\
	"\00710BASE_T_FD"	\
	"\00610BASE_T"

#define	MII_AN_EXP_PARFAULT	0x0010
#define	MII_AN_EXP_LPCANNXTP	0x0008
#define	MII_AN_EXP_CANNXTPP	0x0004
#define	MII_AN_EXP_PAGERCVD 	0x0002
#define	MII_AN_EXP_LPCANAN 	0x0001

#define	MII_AN_EXP_BITS	\
	"\020"	\
	"\005PARFAULT"	\
	"\004LPCANNXTP"	\
	"\003CANNXTPP"	\
	"\002PAGERCVD"	\
	"\001LPCANAN"

#define	MII_1000TC_TESTMODE	0xe000
#define	MII_1000TC_CFG_EN	0x1000
#define	MII_1000TC_CFG_VAL	0x0800
#define	MII_1000TC_PORTTYPE	0x0400
#define	MII_1000TC_ADV_FULL	0x0200
#define	MII_1000TC_ADV_HALF	0x0100

#define	MII_1000TC_BITS	\
	"\020"	\
	"\015CFG_EN"	\
	"\014CFG_VAL"	\
	"\013PORTTYPE"	\
	"\012FULL"	\
	"\011HALF"

#define	MII_1000TS_CFG_FAULT	0x8000
#define	MII_1000TS_CFG_MASTER	0x4000
#define	MII_1000TS_LOCALRXOK	0x2000
#define	MII_1000TS_REMOTERXOK	0x1000
#define	MII_1000TS_LP_FULL	0x0800
#define	MII_1000TS_LP_HALF	0x0400

#define	MII_1000TS_BITS	\
	"\020"	\
	"\020CFG_FAULT"	\
	"\017CFG_MASTER"	\
	"\014CFG_LOCALRXOK"	\
	"\013CFG_REMOTERXOK"	\
	"\012LP_FULL"	\
	"\011LP_HALF"

#define	MII_XSTATUS_1000BASEX_FD	0x8000
#define	MII_XSTATUS_1000BASEX		0x4000
#define	MII_XSTATUS_1000BASET_FD	0x2000
#define	MII_XSTATUS_1000BASET		0x1000

#define	MII_XSTATUS_BITS	\
	"\020"	\
	"\0201000BASEX_FD"	\
	"\0171000BASEX"		\
	"\0161000BASET_FD"	\
	"\0151000BASET"

#define	MII_READ_CMD(p, r)	\
	((6<<(18+5+5)) | ((p)<<(18+5)) | ((r)<<18))

#define	MII_WRITE_CMD(p, r, v)	\
	((5<<(18+5+5)) | ((p)<<(18+5)) | ((r)<<18) | (2 << 16) | (v))
