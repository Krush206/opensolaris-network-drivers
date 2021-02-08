/*
 * rhinereg.h : Macro definitions for VIA rhine ethernet mac chips.
 * Based on VIA VT86C100A and VT6102, VT6105LOM, VT6105M data sheets.
 * Coded by Masayuki Murayama (KHF04453@nifty.ne.jp)
 * This file is public domain.
 */
#ifndef _RHINEREG_H_
#define	_RHINEREG_H_

#pragma	ident	"@(#)rhinereg.h 1.16     07/12/02"

/*
 * Known PCI vendor-id/device-id for rhine chips
 */
#define	VID_VIA		0x1106U
#define	DID_VT86C100A	0x6100U
#define	DID_VT3043	0x3043U
#define	DID_VT6102	0x3065U
#define	DID_VT6105	0x3106U
#define	DID_VT6105M	0x3053U

/*
 * VIA Rhine series Register definitions
 */
/* Offsets to the device registers. */
#define	PAR		0x00	/* B: Physical addr */
#define	RCR		0x06	/* B: Rx Config */
#define	TCR		0x07	/* B: Tx Config */
#define	CR		0x08	/* W: Command Reg */
#define	TXQW		0x0a	/* B: Transmit Queue Wake */
#define	ISR		0x0c	/* W: Interrupt Status */
#define	IMR		0x0e	/* W: Interrupt Mask */
#define	MAR		0x10	/* B: Multicast Addr */
#define	CRDA		0x18	/* L: Current Rx Desc Addr */
#define	CTDA		0x1c	/* L: Current Tx Desc Addr */
#define	MPHY		0x6c	/* B: PHY addr */
#define	MIISR		0x6d	/* B */
#define	BCR		0x6e	/* W */
#define	MIICR		0x70	/* B */
#define	MIIADR		0x71	/* B */
#define	MIIDATA		0x72	/* W */
#define	EECSR		0x74	/* B */
#define	CFGA		0x78	/* B */
#define	CFGB		0x79	/* B */
#define	CFGC		0x7a	/* B */
#define	CFGD		0x7b	/* B */
#define	MPAC		0x7c	/* W */
#define	CRCC		0x7e	/* W */
#define	MISC		0x80	/* W */
#define	MISR		0x84	/* B */
#define	MIMR		0x86	/* B */
#define	CAMMASK		0x88	/* L: for rhine III */
#define	CAMCTRL		0x92	/* B: for rhine III */
#define	CAMADDR		0x93	/* B: for rhine III */
#define	FCR0		0x98	/* B: flow control (rhine III) */
#define	FCR1		0x99	/* B: flow control (rhine III) */
#define	PAUSE_TIMER	0x9a	/* W: sending pause value (rhine III) */
#define	STICKHW		0x83	/* B */
#define	SOFT_TMR0	0x9c
#define	SOFT_TMR1	0x9e
#define	WOLCR_SET	0xa0	/* B:  Wake On Lan command register */
#define	PWCFG_SET	0xa1	/* B */
#define	TSTREG_SET	0xa2	/* B */
#define	WOLCG_SET	0xa3	/* B */
#define	WOLCR_CLR	0xa4	/* B */
#define	PWCFG_CLR	0xa5	/* B */
#define	TSTREG_CLR	0xa6	/* B */
#define	WOLCG_CLR	0xa7	/* B */
#define	PWRCSR_SET	0xa8	/* B */
#define	PWRCSR1_SET	0xa9	/* B */
#define	PWRCSR_CLR	0xac	/* B */
#define	PWRCSR1_CLR	0xad	/* B */
#define	PATRN_CRC	0xb0	/* L */
#define	BYTEMSK0	0xc0	/* L */
#define	BYTEMSK1	0xd0	/* L */
#define	BYTEMSK2	0xe0	/* L */
#define	BYTEMSK3	0xf0	/* L */

#define	MULTICAST_CAM_SIZE	32
#define	VLANTAG_CAM_SIZE	32

/* offset 0x00-0x05  Ethernet Address 0x00-0x05 */
/* No bit field */

/* RCR (offset 0x06, RW) Receive Control register */
#define	RCR_RRFT	0xe0U	/* Receive FIFO threshold */
#define		RCR_RRFT_SHIFT	5U
#define		RRFT_MASK	7U
#define		RRFT_64		0U
#define		RRFT_32		1U
#define		RRFT_128	2U
#define		RRFT_256	3U
#define		RRFT_512	4U
#define		RRFT_768	5U
#define		RRFT_1024	6U
#define		RRFT_SF		7U
#define	RCR_PRO		0x10U	/* Promiscous, all phiscal packet accepted */
#define	RCR_AB		0x08U	/* Broadcast packets accepted */
#define	RCR_AM		0x04U	/* Multicast packets accepted */
#define	RCR_AR		0x02U	/* Small (runt) packets accepted */
#define	RCR_SEP		0x01U	/* Error packets accepted */

#define	RCR_BITS	"\020\5Promisc\4ABroad\3AMulti\2ARunt\1SEP"


/* TCR (offset 0x07, RW) Transmit Control Register */
#define	TCR_RTFT	0xe0U	/* Transmit FIFO threshold, multiple of 128 */
#define		TCR_RTFT_SHIFT	5U
#define		RTFT_MASK	7U
#define		RTFT_128	0U
#define		RTFT_256	1U
#define		RTFT_512	2U
#define		RTFT_1024	3U
#define		RTFT_SF		4U
#define	TCR_RxTAG	0x10U	/* hardware rx tag filter enable (6105M) */
#define	TCR_OFSET	0x08U	/* 0: VIA private spec, 1(default): std */
#define	TCR_LB		0x06U
#define		TCR_LB_MASK		0x3U
#define		TCR_LB_SHIFT		1U
#define		TCR_LB_NORMAL		(0U << TCR_LB_SHIFT)
#define		TCR_LB_LOOPBACK		(1U << TCR_LB_SHIFT)
#define		TCR_LB_MIILOOPBACK	(2U << TCR_LB_SHIFT)
#define		TCR_LB_233LOOPBACK	(3U << TCR_LB_SHIFT)
#define	TCR_TxTAG	0x01U	/* hardware tx tagging enable (6105M) */


/* CR(offset 0x08-0x09, RW) Command register */

#define	CR_SRST		0x8000U	/* Software reset	*/
#define	CR_RDMD1	0x4000U	/* Poll the RD once	*/
#define	CR_TDMD1	0x2000U	/* Poll the TD once	*/
#define	CR_DPOLL	0x0800U	/* Disable TD/RD auto polling */
#define	CR_FDX		0x0400U	/* Full duplex		*/
#define	CR_ETEN		0x0200U	/* was EarlyTX		*/
#define	CR_EREN		0x0100U	/* Enable early receive mode */
#define	CR_RDMD		0x0040U	/* Poll the RD once	*/
#define	CR_TDMD		0x0020U	/* Poll the TD once	*/
#define	CR_TXON		0x0010U	/* Turn on transmit DMA state */
#define	CR_RXON		0x0008U	/* Turn on receive DMA state */
#define	CR_STOP		0x0004U	/* Stop NIC		*/
#define	CR_STRT		0x0002U	/* Start NIC		*/
#define	CR_INIT		0x0001U	/* Init process begin	*/

#define	CR_BITS	\
	"\020"	\
	"\20SRst\14DPoll\13Fdx\12ETEn\11EREn"	\
	"\7RxDmd\6TxDmd\5TxOn\4RxOn\3Stop\2Strt\1Init"

/* offset 0x0c-0x0d ISR: Interrupt status register */

#define	ISR_KEYI	0x8000U	/* General purpose (II) / Rx wake up */
#define	ISR_SRCI	0x4000U	/* link changed */
#define	ISR_ABTI	0x2000U
#define	ISR_NORBF	0x1000U
#define	ISR_PKRACE	0x0800U
#define	ISR_OVFI	0x0400U
#define	ISR_ETI		0x0200U	/* TX FIFO underrun (II) / PHY status chage */
#define	ISR_ERI		0x0100U	/* early receive interrupt */

#define	ISR_CNT		0x0080U	/* statstics count max */
#define	ISR_BE		0x0040U	/* bus error */
#define	ISR_RU		0x0020U	/* receive underrun */
#define	ISR_TU		0x0010U	/* transmit underrun */
#define	ISR_TXE		0x0008U	/* transmit error */
#define	ISR_RXE		0x0004U	/* receive error */
#define	ISR_PTX		0x0002U	/* packet transmitted */
#define	ISR_PRX		0x0001U	/* packet received */

#define	ISR_BITS	\
	"\020"	\
	"\020KEYI\017SRCI\016ABTI\015NORBF"	\
	"\014PKRACE\013OVFlI\012ETI_MIIChange\011ERI"	\
	"\010Cnt\007BE\006RU\005TU"	\
	"\004TxE\003RxE\002PTx\001PRx"

/* IMR (offset 0x0e-0x0f) Interrupt mask register */
/* same as above */

/* MAR (offset 0x10-0x17) Multicast mask register */

/* Curr_RX (offset 0x18-0x1b, doulbe word RW) current RX descriptor address */
/* Curr_TX (offset 0x1c-0x1f, doulbe word RW) current TX descriptor address */

/* RDESC0-3 (offset 0x20-0x2f) Rx Descriptor */

/* The Rx buffer descriptor. */
struct rx_desc {
	volatile uint32_t	rd_status;
	volatile uint32_t	rd_length;
	volatile uint32_t	rd_addr;
	volatile uint32_t	rd_next;
};


/* Receive status field in RDES0 */
#define	RSR_RXOK	0x8000U	/* Received a good packet */
#define	RSR_MAR		0x2000U	/* Multicast address received */
#define	RSR_BAR		0x1000U	/* Broadcast address received */
#define	RSR_PHY		0x0800U	/* Physical address received */
#define	RSR_CHN		0x0400U
#define	RSR_STP		0x0200U	/* Start of packet */
#define	RSR_EDP		0x0100U	/* End of packet */
#define	RSR_BUFF	0x0080U	/* Receive buffer error */
#define	RSR_SERR	0x0040U	/* System bus error */
#define	RSR_RUNT	0x0020U	/* Runt Packet received */
#define	RSR_LONG	0x0010U	/* Long packet received */
#define	RSR_FOV		0x0008U	/* FIFO overflow */
#define	RSR_FAE		0x0004U	/* Frame Alignment error */
#define	RSR_CRC		0x0002U	/* CRC Error error */
#define	RSR_RERR	0x0001U	/* Receive error */

/* useful macros for RSR bits */
#define	RSR_ERRORS	\
	(RSR_BUFF|RSR_SERR|RSR_RUNT|RSR_LONG| \
	RSR_FOV|RSR_FAE|RSR_CRC|RSR_RERR)

#define	RSR_BITS	\
	"\020"	\
	"\20RXOK\16MAR\15BAR\14PHY\13CHN\12STP\11EDP"	\
	"\10BUFF\7SERR\6RUNT\5LONG\4FOV\3FAE\2CRC\1RERR"

#define	RDES0_OWN	0x80000000U
#define	RDES0_LEN_SHIFT	16U
#define	RDES0_LEN_MASK	0x7fffU

#define	RDES1_RBS	0x000007ffU	/* Receive buffer size */
#define	RDES1_C		0x00008000U	/* Chain buffer */

/* TDES0-3 (offset 0x40-0x4f) Tx Descriptor */
struct tx_desc {
	volatile uint32_t	td_csr;
	volatile uint32_t	td_length;
	volatile uint32_t	td_addr;
	volatile uint32_t	td_next;
};

#define	TDES0_OWN	0x80000000U
#define	TDES0_TSR	0x0000ffffU

/* TX status bits in TDSE0.  */
#define	TSR_TXERR	0x8000U	/* underflow/abort/late collision/CRS */
#define	TSR_JAB		0x4000U	/* Jabber (undefined for II) */
#define	TSR_SERR	0x2000U	/* master abort/target abort/parity error */
#define	TSR_TBUFF	0x1000U	/* invalid TD format or TD underflow */
#define	TSR_UDF		0x0800U	/* TX Underflow */
#define	TSR_CRS		0x0400U	/* carrier sense lost */
#define	TSR_OWC		0x0200U	/* late collision */
#define	TSR_ABT		0x0100U	/* exceed max collision */
#define	TSR_CDH		0x0080U	/* CD Heart beat */
#define	TSR_COLS	0x0010U	/* Experience collision on trasnmit event */
#define	TSR_NCR		0x000fU	/* Collision retry count */

#define	TSR_BITS	\
	"\020\20TXERR\16SERR\15TBUFF\14UDF\13CRS\12OWC\11ABT\10CDH"

#define	TDES1_IC	0x00800000U	/* assert interrupt immidiately */
#define	TDES1_EDP	0x00400000U	/* End of transmit packet */
#define	TDES1_STP	0x00200000U	/* Start of transmit packet */

#define	TDES1_CRC	0x00010000U	/* Disable CRC generation */
#define	TDES1_C		0x00008000U	/* Chained */
#define	TDES1_TLNG	0x000007ffU	/* Transmit buffer size */

#define	TDES1_BITS	"\020\030IC\027EDP\026STP\021CRC_DIS\020C"

#define	TDES3_TDCTL	0x00000001U	/* surpress interrupt */
#define	TDES3_NEXT	0xfffffff0U	/* surpress interrupt */

/* MPHY (offset 0x6c) Phy address register */

#define	PHYADR_MPO	0xc0U		/* MII polling interval (def 0) */
#define	PHYADR_MFDC	0x20U		/* Accelerate MDC sleep 4times */
#define		PHYADR_MPO_MASK		0x3U
#define		PHYADR_MPO_SHIFT	6U
#define		PHYADR_MPO_1024	(0U << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_512	(1U << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_128	(2U << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_64	(3U << PHYADR_MPO_SHIFT)
#define	PHYADR_PHYAD	0x1fU		/* phy addrss mask (def 0x01) */


/* MIISR (offset 0x6d) */

#define	MIISR_PHYRST	0x80U	/* Phy reset, by software driven */
#define	MIISR_GPIO1POL	0x80U	/* Phy reset, by software driven */
#define	MIISR_MFDC	0x10U	/* use default ext'l PHY device address as 1 */
#define	MIISR_PHYOPT	0x10U	/* use default ext'l PHY device address as 1 */
#define	MIISR_MIIERR	0x08U	/* Phy device received error */
#define	MIISR_MRERR	0x04U	/* Phy device received error */
#define	MIISR_LNKFL	0x02U	/* Link Fail */
#define	MIISR_SPD10	0x01U	/* Phy Speed 1:100M, 0:10M */


/* BCR (offset 0x6e-0x6f) Bus Config register */
#define	BCR_VIDFLTR	0x8000U		/* VLAN ID filter (IIIM) */
#define	BCR_HTQBLK	0x4000U		/* Higher tx queue blocks lower(IIIM) */
#define	BCR_MED_FDX	0x8000U		/* fixed FDX mode (II) */
#define	BCR_MED_SPD100	0x4000U		/* fixed 100M mode (II) */
#define	BCR_CTFT	0x3800U		/* TX FIFO threshold */
#define		BCR_CTFT_USE_TDES	0U	/* use tx descriptor */
#define		BCR_CTFT_MASK		0x7U
#define		BCR_CTFT_SHIFT		11U
#define	BCR_POT		0x0700U			/* Polling timer interval */
#define		BCR_POT_MASK		0x7U
#define		BCR_POT_SHIFT		8U
#define	BCR_MED2	0x0080U			/* Medium select control */
#define	BCR_EXTLED	0x0040U			/* Extra LED control */
#define	BCR_CRFT	0x0038U			/* RX FIFO threshold */
#define		BCR_CRFT_USE_RDES	0U	/* use rx descriptor */
#define		BCR_CRFT_MASK		0x7U
#define		BCR_CRFT_SHIFT		3U
#define		BCR_CRFT_EXTLED		0x0040U	/* Extra LED support control */
#define		BCR_CRFT_MED2		0x0080U	/* Medium select control */
#define	BCR_DMA_MASK	0x0007U			/* DMA length */
#define		BCR_DMA_32		0U
#define		BCR_DMA_64		1U
#define		BCR_DMA_128		2U
#define		BCR_DMA_256		3U
#define		BCR_DMA_512		4U
#define		BCR_DMA_1024		5U
#define		BCR_DMA_NOLIMIT		6U

/* MIICR (0x70) MII interface control registger */
#define	MIICR_MAUTO	0x80U	/* MII management auto port polling disable */
#define	MIICR_RCMD	0x40U	/* Read enable, reset when read complete */
#define	MIICR_WCMD	0x20U	/* Write enable, reset when write complete */
#define	MIICR_MDPM	0x10U	/* Direct PHY programming mode enable */
#define	MIICR_MDOUT	0x08U	/* DP output enable indicator */
#define	MIICR_MDO	0x04U	/* DP management port data out */
#define	MIICR_MDC	0x01U	/* DP management port clock */
#define	MIICR_MDI	0x02U	/* DP input while PHY status */

/* MIIADR (0x71) MII CSR Offset address registger */
#define	MIIADR_MIDLE	0x80U	/* not in polling cycle (II or later) */
#define	MIIADR_MSRCEN	0x40U	/* close the poll function of MDONE */
#define	MIIADR_MDONE	0x20U	/* MDIO auto polling data ready */
#define	MIIADR_MAD	0x1fU	/* MII management port address */

/* MIIDATA(0x72-73) MII data register  */

/* EECSR (0x74) Direct program EEPROM interface */
#define	EECSR_EEPR	0x80U	/* EEPROM programmed status */
#define	EECSR_EMBP	0x40U	/* EEPROM embedded programming enable */
#define	EECSR_AUTOLD	0x20U	/* Dynamic reload EEPROM content */
#define	EECSR_DPM	0x10U	/* EEPROM mode */
#define	EECSR_ECS	0x08U	/* chip select status */
#define	EECSR_ECK	0x04U	/* clock status */
#define	EECSR_EDI	0x02U	/* data in status */
#define		EECSR_EDI_SHIFT	1U	/* data in status */
#define	EECSR_EDO	0x01	/* data out status */
#define		EECSR_EDO_SHIFT	0U	/* data out status */

/* TEST (offset 0x75) */
/* DEGUG0 (offset 0x76) */
/* DEGUG1 (offset 0x77) */

/* CFGA (offset 0x78 ) configration A register */
#define	CFGA_EELOAD	0x80U	/* Enable EEPROM and direct programming */
#define	CFGA_MIIOPT	0x40U	/* 0: w/o extension clk, 1: w/ extension clk */
#define	CFGA_MMIOEN	0x20U	/* enable memory mapped I/O (VT86C100A) */

/* CFGB (offset 0x79 ) configration B register */
#define	CFGB_QPKTDIS	0x80U	/* Disable transmit frame queuing */
#define	CFGB_PERRDIS	0x40U	/* Disable data parity generation (II) */
#define	CFGB_TXPACE	0x40U	/* TX descriptor paceing algorithm (100) */
#define	CFGB_MRLDIS	0x20U	/* Memory read line 0:enable 1:disable */
#define	CFGB_TXARBIT	0x10U	/* Arbitration priority */
#define	CFGB_RXARBIT	0x08U	/* Arbitration priority */
#define	CFGB_MRWAIT	0x04U	/* Master read insert one wait state 2-2-2-2 */
#define	CFGB_MWAIT	0x02U	/* Master write insert one wait state 2-2-2-2 */
#define	CFGB_LATMEN	0x01U	/* latency timer */

/* CFGC (offset 0x7a) */
#define	CFGC_MED3	0x80U	/* Medium select control (II) */
#define	CFGC_BROPT	0x40U	/* Tie the unused bootrom address MA to high */
#define	CFGC_DLYEN	0x20U	/* turn on delay in reading bootrprom */
#define	CFGC_BTSEL	0x08U	/* BOOTPROM timing select */
#define	CFGC_BPS	0x07U	/* BOOTPROM size select */
#define		CFGC_BPS_NONE	0x00U	/*    NO BOOTPROM */
#define		CFGC_BPS_8K	0x01U	/*    8K size */
#define		CFGC_BPS_16K	0x02U	/*    16K size */
#define		CFGC_BPS_32K	0x03U	/*    32K size */
#define		CFGC_BPS_64K	0x04U	/*    64K size */

/* CFGD (offset 0x7b) */
#define	CFGD_MMIOEN	0x80U	/* Memory mapped I/O access enable (II) */
#define	CFGD_GPIOEN	0x80U	/* GPIO2 Input status change monitor(100) */
#define	CFGD_DIAG	0x40U	/* Diagnostic mode 0:disable, 1:enable */
#define	CFGD_MRLEN	0x20U	/* PCI memory read line capable 0:no, 1:yes */
#define	CFGD_MAGICKEY	0x10U	/* Magic key enable(100) */
#define	CFGD_PMCDIG	0x10U	/* PMCC(0x82) setting mode (II) */
#define	CFGD_CRANDOM	0x08U	/* Backoff algorithm : random */
#define	CFGD_CAP	0x04U	/* Capture effect solution : DEC solusion */
#define	CFGD_MBA	0x02U	/* Capture effect solution : AMD solusion */
#define	CFGD_BAKOPT	0x01U	/* Backoff algorithm 0: disable, 1: enable */

/* MISC (offset 0x80-0x81) */
#define	MISC_FORSRST	0x4000U	/* Force software reset */
#define	MISC_VAXJMP	0x2000U	/* There is a AUX power outside, for soft ref */
#define	MISC_Tm1EN	0x0100U	/* Enable software timer 1 to count */
#define	MISC_Tm0US	0x0020U	/* timer 0 micro second mode */
#define	MISC_FDXTFEN	0x0010U	/* Full duplex flow tx control enable */
#define	MISC_FDXRFEN	0x0008U	/* Full duplex flow rx control enable */
#define	MISC_HDXFEN	0x0004U	/* Half duplex flow control enable */
#define	MISC_Tm0Susp	0x0002U	/* 1 when Soft timer0 timeout */
#define	MISC_Tm0EN	0x0001U	/* Enable software timer 0 */

/* STICKHW (offset 0x83) */
#define	STICKHW_LGWOL	0x80	/* Legacy WOL enable, jumper strapping MD5 */
#define	STICKHW_WOLSR	0x08U	/* Legacy WOL status */
#define	STICKHW_WOLEN	0x04U	/* Legacy WOL enable */
#define	STICKHW_DS1	0x02U	/* Sticky DS1_shadow, R/W by software */
#define	STICKHW_DS0	0x01U	/* Sticky DS0_shadow, suspend well write port */

/* MISR (offset 0x84) */
#define	MISR_PMEI	0x80U	/* Power event report in test mode */
#define	MISR_UDPICLR	0x40U	/* User defined, host driven interrupt */
#define	MISR_UDPISET	0x20U	/* User defined, host driven interrupt */
#define	MISR_SSRCI	0x10U	/* suspend well status changed */
#define	MISR_TDWBI	0x08U	/* TD WB queue race interrupt */
#define	MISR_PHYI	0x04U	/* PHY status changed */
#define	MISR_TIM1I	0x02U	/* Software timer 1 interrupt */
#define	MISR_TIM0I	0x01U	/* Software timer 0 interrupt */

#define	MISR_BITS	\
	"\020\010PMEI\007UDPI_CLR\007UDPICLR\006UDPISET\005SSRCI"	\
	"\004TDWBI\003PHYI\002TIM1I\001TIM0I"

/* MIMR (offset 0x86) */
#define	MIMR_PMEIM	0x80U	/* Power event report in test mode mask */
#define	MIMR_UDPI	0x40U	/* User defined, host driven interrupt mask */
#define	MIMR_UDPIM	0x40U	/* User defined, host driven interrupt mask */
#define	MIMR_SSRCIM	0x10U	/* */
#define	MIMR_TDWBIM	0x08U	/* TD WB queue race interrupt mask */
#define	MIMR_TIM1IM	0x02U	/* Software timer 1 interrupt mask */
#define	MIMR_TIM0IM	0x01U	/* Software timer 0 interrupt mask */
/* BPMA (offset 0x8c-0c8d) */
/* BPMD (offset 0x8f) */
/* BPCMD (offset 0x90) */
#define	BPCMD_EBPWR	0x02U	/* BOOTROM embedded write command */
#define	BPCMD_EBPRD	0x01U	/* BOOTROM embedded read command */

/* BPIN_DATA (offset 0x91) */

/* CAMCTRL (offset 0x92) 6105M only */
#define	CAMCTRL_RD	0x08U	/* cam read command */
#define	CAMCTRL_WR	0x04U	/* cam write command */
#define	CAMCTRL_VLAN	0x02U	/* cam select (1:VLAN, 0:multicast) */
#define	CAMCTRL_EN	0x01U	/* enable cam access */

/* EE_CHKSUM (offset 0x93) */
/* SUSPEND_MII_AD (offset 0x94-95) */
/* SU_PHYID (offset 0x96) */
/* FCR0 (offset 0x98) rhine3 only */
/* FCR1 (offset 0x99) rhine3 only */
#define	FCR1_TXLOWAT	0xc0
#define		FCR1_TXLOWAT_MASK	0x3U
#define		FCR1_TXLOWAT_SHIFT	6U
#define		FCR1_TXLOWAT_4	(0U << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_8	(1U << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_16	(2U << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_24	(3U << FCR1_TXLOWAT_SHIFT)
#define	FCR1_TXHIWAT	0x30
#define		FCR1_TXHIWAT_MASK	0x3U
#define		FCR1_TXHIWAT_SHIFT	4U
#define		FCR1_TXHIWAT_24	(0U << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_32	(1U << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_48	(2U << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_64	(3U << FCR1_TXHIWAT_SHIFT)
#define	FCR1_XONOFF_EN	0x08U	/* enable sending xoff pause frame */
#define	FCR1_FDFCTX_EN	0x04U	/* enable sending pause frame */
#define	FCR1_FDFCRX_EN	0x02U	/* enable receiving pause frame */
#define	FCR1_HDFC_EN	0x01U	/* enable half duplex flow control */

/* PAUSE_TIMER (offset 0x9a-0x9b) pause timer value to send */
/* SOFT_TMR0 (offset 0x9c-0x9d) */
/* SOFT_TMR1 (offset 0x9e-0x9f) */

/* WOLCR.SET WOLCR.CLR (offset 0xa0, 0xa4 RWC) Wake On Lan command register */
#define	WOLCR_LinkOFF	0x80U	/* enable link of detected */
#define	WOLCR_LinkON	0x40U	/* enable link on detected */
#define	WOLCR_MAGICEN	0x20U	/* enable Magic packet filter */
#define	WOLCR_UNICAST	0x10U	/* enable UNICAST filter */
#define	WOLCR_PTNMH	0x0fU	/* enable pattern match filtering */

/* PWCFG.SET PWCFG.CLR (offset 0xa1, 0xa5 RWC) */
#define	PWCFG_SMIITIME	0x80U	/* Internal MII interface timing */
#define	PWCFG_WOLTYPE	0x20U	/* Drive WOL output by 1:pulse or 0:button */
#define	PWCFG_LEGCYWOL	0x10U	/* Enale legacy wake on lan */
#define	PWCFG_WOLSR	0x02U	/* Legacy WOL_SR shadow */
#define	PWCFG_WOLEN	0x01U	/* Legacy WOL_EN shadow */

/* TESTREG.SET TESTREG.CLR (offset 0xa2, 0xa6 RWC) */
#define	TESTREG_SNORM	0x01U	/* All power state capable while PHYTEST=0 */

/* WOLCG.SET WOLCG.CLR (offset 0xa3, 0xa7 RWC) */
#define	WOLCG_PMEOVR	0x80U	/* Force PMEEN always over PME_EN (legacy) */
#define	WOLCG_SFDX	0x40U	/* Full duplex in suspend well */
#define	WOLCG_SAM	0x20U	/* Accept multicast in suspend well */
#define	WOLCG_SAB	0x10U	/* Accept broadcast in suspend well */
#define	WOLCG_SMIIACC	0x08U	/* MDC acceleration */
#define	WOLCG_SMIIOPT	0x04U	/* MIIOPT to extend clock in suspend well */

/* PWRCSR.SET PSWCSR.CLR (offset 0xa8, 0xac RWC) undocumented */

/* PATTERN CRC0-3 (offset 0xb0-0xbf, RW) */
/* BYTEMASK0-3 CRC0-3 (offset 0xc0-0xff, RW) */

#endif	/* _RHINEREG_H_ */
