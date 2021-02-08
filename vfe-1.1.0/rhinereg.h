/*
 * rhinereg.h : Macro definitions for VIA rhine ethernet mac chips.
 * Based on VIA VT86C100A and VT6102, VT6105LOM, VT6105M data sheets.
 * Coded by Masayuki Murayama (KHF04453@nifty.ne.jp)
 * This file is public domain.
 */

#pragma	ident	"@(#)rhinereg.h	1.11 05/06/06"

/*
 * Known PCI vendor-id/device-id for rhine chips
 */
#define	VID_VIA		0x1106
#define	DID_VT86C100A	0x6100
#define	DID_VT3043	0x3043
#define	DID_VT6102	0x3065
#define	DID_VT6105	0x3106
#define	DID_VT6105M	0x3053

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
#define	CRDA		0x18	/* L: Current Rx Desc Addr*/
#define	CTDA		0x1c	/* L: Current Tx Desc Addr*/
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
#define	RCR_RRFT	0xe0	/* Receive FIFO threshold */
#define		RCR_RRFT_SHIFT	5
#define		RRFT_MASK	7
#define		RRFT_64		0
#define		RRFT_32		1
#define		RRFT_128	2
#define		RRFT_256	3
#define		RRFT_512	4
#define		RRFT_768	5
#define		RRFT_1024	6
#define		RRFT_SF		7
#define	RCR_PRO		0x10	/* Promiscous, all phiscal packet accepted */
#define	RCR_AB		0x08	/* Broadcast packets accepted */
#define	RCR_AM		0x04	/* Multicast packets accepted */
#define	RCR_AR		0x02	/* Small (runt) packets accepted */
#define	RCR_SEP		0x01	/* Error packets accepted */

#define	RCR_BITS	"\020\5Promisc\4ABroad\3AMulti\2ARunt\1SEP"


/* TCR (offset 0x07, RW) Transmit Control Register */
#define	TCR_RTFT	0xe0	/* Transmit FIFO threshold, multiple of 128 */
#define		TCR_RTFT_SHIFT	5
#define		RTFT_MASK	7
#define		RTFT_128	0
#define		RTFT_256	1
#define		RTFT_512	2
#define		RTFT_1024	3
#define		RTFT_SF		4
#define	TCR_OFSET	0x08	/* 0: VIA private spec, 1(default): std */
#define	TCR_LB		0x06
#define		TCR_LB_MASK		0x3
#define		TCR_LB_SHIFT		1
#define		TCR_LB_NORMAL		(0 << TCR_LB_SHIFT)
#define		TCR_LB_LOOPBACK		(1 << TCR_LB_SHIFT)
#define		TCR_LB_MIILOOPBACK	(2 << TCR_LB_SHIFT)
#define		TCR_LB_233LOOPBACK	(3 << TCR_LB_SHIFT)


/* CR(offset 0x08-0x09, RW) Command register */

#define	CR_SRST		0x8000	/* Software reset	*/
#define	CR_RDMD1	0x4000	/* Poll the RD once	*/
#define	CR_TDMD1	0x2000	/* Poll the TD once	*/
#define	CR_DPOLL	0x0800	/* Disable TD/RD auto polling */
#define	CR_FDX		0x0400	/* Full duplex		*/
#define	CR_ETEN		0x0200	/* was EarlyTX		*/
#define	CR_EREN		0x0100	/* Enable early receive mode */
#define	CR_RDMD		0x0040	/* Poll the RD once	*/
#define	CR_TDMD		0x0020	/* Poll the TD once	*/
#define	CR_TXON		0x0010	/* Turn on transmit DMA state */
#define	CR_RXON		0x0008	/* Turn on receive DMA state */
#define	CR_STOP		0x0004	/* Stop NIC		*/
#define	CR_STRT		0x0002	/* Start NIC		*/
#define	CR_INIT		0x0001	/* Init process begin	*/

#define	CR_BITS	\
	"\020"	\
	"\20SRst\14DPoll\13Fdx\12ETEn\11EREn"	\
	"\7RxDmd\6TxDmd\5TxOn\4RxOn\3Stop\2Strt\1Init"

/* offset 0x0c-0x0d ISR: Interrupt status register */

#define	ISR_KEYI	0x8000	/* General purpose 6102 / Rx wake up */
#define	ISR_SRCI	0x4000	/* link changed */
#define	ISR_ABTI	0x2000
#define	ISR_NORBF	0x1000
#define	ISR_PKRACE	0x0800
#define	ISR_OVFI	0x0400
#define	ISR_ETI		0x0200	/* TX FIFO underrun 6102 / PHY status chage */
#define	ISR_ERI		0x0100	/* early receive interrupt */

#define	ISR_CNT		0x0080	/* statstics count max */
#define	ISR_BE		0x0040	/* bus error */
#define	ISR_RU		0x0020	/* receive underrun */
#define	ISR_TU		0x0010	/* transmit underrun */
#define	ISR_TXE		0x0008	/* transmit error */
#define	ISR_RXE		0x0004	/* receive error */
#define	ISR_PTX		0x0002	/* packet transmitted */
#define	ISR_PRX		0x0001	/* packet received */

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
	uint32_t	rd_status;
	uint32_t	rd_length;
	uint32_t	rd_addr;
	uint32_t	rd_next;
};


/* Receive status field in RDES0 */
#define	RSR_RXOK	0x8000	/* Received a good packet */
#define	RSR_MAR		0x2000	/* Multicast address received */
#define	RSR_BAR		0x1000	/* Broadcast address received */
#define	RSR_PHY		0x0800	/* Physical address received */
#define	RSR_CHN		0x0400
#define	RSR_STP		0x0200	/* Start of packet */
#define	RSR_EDP		0x0100	/* End of packet */
#define	RSR_BUFF	0x0080	/* Receive buffer error */
#define	RSR_SERR	0x0040	/* System bus error */
#define	RSR_RUNT	0x0020	/* Runt Packet received */
#define	RSR_LONG	0x0010	/* Long packet received */
#define	RSR_FOV		0x0008	/* FIFO overflow */
#define	RSR_FAE		0x0004	/* Frame Alignment error */
#define	RSR_CRC		0x0002	/* CRC Error error */
#define	RSR_RERR	0x0001	/* Receive error */

/* useful macros for RSR bits */
#define	RSR_ERRORS	(RSR_BUFF|RSR_SERR|RSR_RUNT|RSR_LONG| \
			 RSR_FOV|RSR_FAE|RSR_CRC|RSR_RERR)

#define	RSR_BITS	\
	"\020"	\
	"\20RXOK\16MAR\15BAR\14PHY\13CHN\12STP\11EDP"	\
	"\10BUFF\7SERR\6RUNT\5LONG\4FOV\3FAE\2CRC\1RERR"

#define	RDES0_OWN	0x80000000
#define	RDES0_LEN_SHIFT	16
#define	RDES0_LEN_MASK	0x7fff

#define	RDES1_RBS	0x000007ff	/* Receive buffer size */
#define	RDES1_C		0x00008000	/* Chain buffer */

/* TDES0-3 (offset 0x40-0x4f) Tx Descriptor */
struct tx_desc {
	uint32_t	td_csr;
	uint32_t	td_length;
	uint32_t	td_addr;
	uint32_t	td_next;
};

#define	TDES0_OWN	0x80000000
#define	TDES0_TSR	0x0000ffff

/* TX status bits in TDSE0.  */
#define	TSR_TXERR	0x8000	/* underflow/abort/late collision/CRS */
#define	TSR_JAB		0x4000	/* Jabber (undefined for VT6102)*/
#define	TSR_SERR	0x2000	/* master abort/target abort/parity error */
#define	TSR_TBUFF	0x1000	/* invalid TD format or TD underflow */
#define	TSR_UDF		0x0800	/* TX Underflow */
#define	TSR_CRS		0x0400	/* carrier sense lost */
#define	TSR_OWC		0x0200	/* late collision */
#define	TSR_ABT		0x0100	/* exceed max collision */
#define	TSR_CDH		0x0080	/* CD Heart beat */
#define	TSR_COLS	0x0010	/* Experience collision on trasnmit event */
#define	TSR_NCR		0x000f	/* Collision retry count */

#define	TSR_BITS	\
	"\020\20TXERR\16SERR\15TBUFF\14UDF\13CRS\12OWC\11ABT\10CDH"

#define	TDES1_IC	0x00800000	/* assert interrupt immidiately */
#define	TDES1_EDP	0x00400000	/* End of transmit packet */
#define	TDES1_STP	0x00200000	/* Start of transmit packet */

#define	TDES1_CRC	0x00010000	/* Disable CRC generation */
#define	TDES1_C		0x00008000	/* Chained */
#define	TDES1_TLNG	0x000007ff	/* Transmit buffer size */

#define	TDES1_BITS	"\020\030IC\027EDP\026STP\021CRC_DIS\020C"

#define	TDES3_TDCTL	0x00000001	/* surpress interrupt */
#define	TDES3_NEXT	0xfffffff0	/* surpress interrupt */

/* PHY_ADR (offset 0x6c) Phy address register */

#define	PHYADR_MPO	0xc0		/* MII polling timer interval (def 0)*/
#define	PHYADR_MFDC	0x20		/* Accelerate MDC sleep 4times */
#define		PHYADR_MPO_MASK		0x3
#define		PHYADR_MPO_SHIFT	6
#define		PHYADR_MPO_1024	(0 << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_512	(1 << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_128	(2 << PHYADR_MPO_SHIFT)
#define		PHYADR_MPO_64	(3 << PHYADR_MPO_SHIFT)
#define	PHYADR_PHYAD	0x1f		/* phy addrss mask (def 0x01) */


/* MIISR (offset 0x6d) */

#define	MIISR_PHYRST	0x80	/* Phy reset, by software driven */
#define	MIISR_GPIO1POL	0x80	/* Phy reset, by software driven */
#define	MIISR_MFDC	0x10	/* use default ext'l PHY device address as 1 */
#define	MIISR_PHYOPT	0x10	/* use default ext'l PHY device address as 1 */
#define	MIISR_MIIERR	0x08	/* Phy device received error */
#define	MIISR_MRERR	0x04	/* Phy device received error */
#define	MIISR_LNKFL	0x02	/* Link Fail */
#define	MIISR_SPD10	0x01	/* Phy Speed 1:100M, 0:10M */


/* BCR (offset 0x6e-0x6f) Bus Config register */
#define	BCR_MED_FDX	0x8000			/* fixed FDX mode (RHINE II) */
#define	BCR_MED_SPD100	0x4000			/* fixed 100M mode (RHINE II) */
#define	BCR_CTFT	0x3800			/* RX FIFO threshold */
#define		BCR_CTFT_USE_TDES	0	/* use tx descriptor */
#define		BCR_CTFT_MASK		0x7
#define		BCR_CTFT_SHIFT		11
#define	BCR_POT		0x0700			/* Polling timer interval */
#define		BCR_POT_MASK		0x7
#define		BCR_POT_SHIFT		8
#define	BCR_MED2	0x0080			/* Medium select control */
#define	BCR_EXTLED	0x0040			/* Extra LED control */
#define	BCR_CRFT	0x0038			/* RX FIFO threshold */
#define		BCR_CRFT_USE_RDES	0	/* use rx descriptor */
#define		BCR_CRFT_MASK		0x7
#define		BCR_CRFT_SHIFT		3
#define		BCR_CRFT_EXTLED		0x0040	/* Extra LED support control */
#define		BCR_CRFT_MED2		0x0080	/* Medium select control */
#define	BCR_DMA_MASK	0x0007			/* DMA length */
#define		BCR_DMA_32		0
#define		BCR_DMA_64		1
#define		BCR_DMA_128		2
#define		BCR_DMA_256		3
#define		BCR_DMA_512		4
#define		BCR_DMA_1024		5
#define		BCR_DMA_NOLIMIT		6

/* MIICR (0x70) MII interface control registger */
#define	MIICR_MAUTO	0x80	/* MII management auto port polling disable */
#define	MIICR_RCMD	0x40	/* Read enable, reset when read complete */
#define	MIICR_WCMD	0x20	/* Write enable, reset when write complete */
#define	MIICR_MDPM	0x10	/* Direct PHY programming mode enable */
#define	MIICR_MDOUT	0x08	/* Direct programming out put enable indicator*/
#define	MIICR_MDO	0x04	/* Direct programming status as management port data out */
#define	MIICR_MDC	0x01	/* Direct programming status as management port clock */
#define	MIICR_MDI	0x02	/* Direct programming input while PHY status */

/* MIIADR (0x71) MII CSR Offset address registger */
#define	MIIADR_MIDLE	0x80	/* not in polling cycle (VT6102 or later) */
#define	MIIADR_MSRCEN	0x40	/* close the poll function of MDONE */
#define	MIIADR_MDONE	0x20	/* MDIO auto polling data ready */
#define	MIIADR_MAD	0x1f	/* MII management port address */

/* MIIDATA(0x72-73) MII data register  */

/* EECSR (0x74) Direct program EEPROM interface */
#define	EECSR_EEPR	0x80	/* EEPROM programmed status */
#define	EECSR_EMBP	0x40	/* EEPROM embedded programming enable */
#define	EECSR_AUTOLD	0x20	/* Dynamic reload EEPROM content */
#define	EECSR_DPM	0x10	/* EEPROM mode */
#define	EECSR_ECS	0x08	/* chip select status */
#define	EECSR_ECK	0x04	/* clock status */
#define	EECSR_EDI	0x02	/* data in status */
#define		EECSR_EDI_SHIFT	1	/* data in status */
#define	EECSR_EDO	0x01	/* data out status */
#define		EECSR_EDO_SHIFT	0	/* data out status */

/* TEST (offset 0x75)*/
/* DEGUG0 (offset 0x76)*/
/* DEGUG1 (offset 0x77)*/

/* CFGA (offset 0x78 ) configration A register */
#define	CFGA_EELOAD	0x80	/* Enable EEPROM embedded and direct programming */
#define	CFGA_MIIOPT	0x40	/* 0: w/o extension clk, 1: w/ extension clk */
#define	CFGA_MMIOEN	0x20	/* enable memory mapped I/O (VT86C100A) */

/* CFGB (offset 0x79 ) configration B register */
#define	CFGB_QPKTDIS	0x80	/* Disable transmit frame queuing */
#define	CFGB_PERRDIS	0x40	/* Disable data parity generation (6102) */
#define	CFGB_TXPACE	0x40	/* TX descriptor paceing algorithm (100) */
#define	CFGB_MRLDIS	0x20	/* Memory read line support 0:enable 1:disable*/
#define	CFGB_TXARBIT	0x10	/* Arbitration priority */
#define	CFGB_RXARBIT	0x08	/* Arbitration priority */
#define	CFGB_MRWAIT	0x04	/* Master read insert one wait state 2-2-2-2 */
#define	CFGB_MWAIT	0x02	/* Master write insert one wait state 2-2-2-2 */
#define	CFGB_LATMEN	0x01	/* latency timer */

/* CFGC (offset 0x7a) */
#define	CFGC_MED3	0x80	/* Medium select control (VT6102) */
#define	CFGC_BROPT	0x40	/* Tie the unused bootrom address MA to high */
#define	CFGC_DLYEN	0x20	/* turn on delay transaction while memory read bootrprom */
#define	CFGC_BTSEL	0x08	/* BOOTPROM timing select */
#define	CFGC_BPS	0x07	/* BOOTPROM size select */
#define		CFGC_BPS_NONE	0x00	/*    NO BOOTPROM */
#define		CFGC_BPS_8K	0x01	/*    8K size */
#define		CFGC_BPS_16K	0x02	/*    16K size */
#define		CFGC_BPS_32K	0x03	/*    32K size */
#define		CFGC_BPS_64K	0x04	/*    64K size */

/* CFGD (offset 0x7b) */
#define	CFGD_MMIOEN	0x80	/* Memory mapped I/O access enable (6102) */
#define	CFGD_GPIOEN	0x80	/* GPIO2 Input status change monitor(100) */
#define	CFGD_DIAG	0x40	/* Diagnostic mode 0:disable, 1:enable */
#define	CFGD_MRLEN	0x20	/* PCI memory read line capable 0:no, 1:yes */
#define	CFGD_MAGICKEY	0x10	/* Magic key enable(100) */
#define	CFGD_PMCDIG	0x10	/* PMCC(0x82) setting mode (6102) */
#define	CFGD_CRANDOM	0x08	/* Backoff algorithm random */
#define	CFGD_CAP	0x04	/* Capture effect solusion : DEC solusion */
#define	CFGD_MBA	0x02	/* Capture effect solusion : AMD solusion */
#define	CFGD_BAKOPT	0x01	/* Backoff algorithm 0: disable, 1: enable */

/* MISC (offset 0x80-0x81) */
#define	MISC_FORSRST	0x4000	/* Force software reset */
#define	MISC_VAXJMP	0x2000	/* There is a AUX power outside, for sofr ref.*/
#define	MISC_Tm1EN	0x0100	/* Enable software timer 1 to count */
#define	MISC_Tm0US	0x0020	/* timer 0 micro second mode */
#define	MISC_FDXTFEN	0x0010	/* Full duplex flow tx control enable */
#define	MISC_FDXRFEN	0x0008	/* Full duplex flow rx control enable */
#define	MISC_HDXFEN	0x0004	/* Half duplex flow control enable */
#define	MISC_Tm0Susp	0x0002	/* 1 when Soft timer0 timeout */
#define	MISC_Tm0EN	0x0001	/* Enable software timer 0 */

/* STICKHW (offset 0x83) */
#define	STICKHW_LGWOL	0x80	/* Legacy WOL enable, status for software reference from jumper strapping MD5 */
#define	STICKHW_WOLSR	0x08	/* Legacy WOL status */
#define	STICKHW_WOLEN	0x04	/* Legacy WOL enable */
#define	STICKHW_DS1	0x02	/* Sticky DS0_shadow, R/W by software */
#define	STICKHW_DS0	0x01	/* Sticky DS0_shadow, suspend well DS write port */

/* MISR (offset 0x84) */
#define	MISR_PMEI	0x80	/* Power event report in test mode */
#define	MISR_UDPICLR	0x40	/* User defined, host driven interrupt */
#define	MISR_UDPISET	0x20	/* User defined, host driven interrupt */
#define	MISR_SSRCI	0x10	/* suspend well status changed */
#define	MISR_TDWBI	0x08	/* TD WB queue race, will cause when TX shotdown*/
#define	MISR_PHYI	0x04	/* PHY status changed */
#define	MISR_TIM1I	0x02	/* Software timer 1 interrupt */
#define	MISR_TIM0I	0x01	/* Software timer 0 interrupt */

#define	MISR_BITS	\
	"\020\010PMEI\007UDPI_CLR\007UDPICLR\006UDPISET\005SSRCI"	\
	"\004TDWBI\003PHYI\002TIM1I\001TIM0I"

/* MIMR (offset 0x86) */
#define	MIMR_PMEIM	0x80	/* Power event report in test mode mask */
#define	MIMR_UDPI	0x40	/* User defined, host driven interrupt mask*/
#define	MIMR_UDPIM	0x40	/* User defined, host driven interrupt mask*/
#define	MIMR_SSRCIM	0x10	/* */
#define	MIMR_TDWBIM	0x08	/* TD WB queue race, will cause when TX shotdown mask*/
#define	MIMR_TIM1IM	0x02	/* Software timer 1 interrupt mask */
#define	MIMR_TIM0IM	0x01	/* Software timer 0 interrupt mask */
/* BPMA (offset 0x8c-0c8d) */
/* BPMD (offset 0x8f) */
/* BPCMD (offset 0x90) */
#define BPCMD_EBPWR	0x02	/* BOOTROM embedded write command */
#define BPCMD_EBPRD	0x01	/* BOOTROM embedded read command */

/* BPIN_DATA (offset 0x91) */

/* CAMCTRL (offset 0x92) 6105M only */
#define	CAMCTRL_RD	0x08	/* cam read command */
#define	CAMCTRL_WR	0x04	/* cam write command */
#define	CAMCTRL_VLAN	0x02	/* cam select (1: VLAN 0: multicast)*/
#define	CAMCTRL_EN	0x01	/* enable cam access */

/* EE_CHKSUM (offset 0x93) */
/* SUSPEND_MII_AD (offset 0x94-95) */
/* SU_PHYID (offset 0x96) */
/* FCR0 (offset 0x98) rhine3 only */
/* FCR1 (offset 0x99) rhine3 only */
#define FCR1_TXLOWAT	0xc0
#define		FCR1_TXLOWAT_MASK	0x3
#define		FCR1_TXLOWAT_SHIFT	6
#define		FCR1_TXLOWAT_4	(0 << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_8	(1 << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_16	(2 << FCR1_TXLOWAT_SHIFT)
#define		FCR1_TXLOWAT_24	(3 << FCR1_TXLOWAT_SHIFT)
#define FCR1_TXHIWAT	0x30
#define		FCR1_TXHIWAT_MASK	0x3
#define		FCR1_TXHIWAT_SHIFT	4
#define		FCR1_TXHIWAT_24	(0 << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_32	(1 << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_48	(2 << FCR1_TXHIWAT_SHIFT)
#define		FCR1_TXHIWAT_64	(3 << FCR1_TXHIWAT_SHIFT)
#define FCR1_XONOFF_EN	0x08	/* enable sending xoff pause frame */
#define FCR1_FDFCTX_EN	0x04	/* enable sending pause frame */
#define FCR1_FDFCRX_EN	0x02	/* enable receiving pause frame */
#define FCR1_HDFC_EN	0x01	/* enable half duplex flow control */

/* PAUSE_TIMER (offset 0x9a-0x9b) pause timer value to send */
/* SOFT_TMR0 (offset 0x9c-0x9d) */
/* SOFT_TMR1 (offset 0x9e-0x9f) */

/* WOLCR.SET WOLCR.CLR (offset 0xa0, 0xa4 RWC) Wake On Lan command register */
#define	WOLCR_LinkOFF	0x80	/* enable link of detected */
#define	WOLCR_LinkON	0x40	/* enable link on detected */
#define	WOLCR_MAGICEN	0x20	/* enable Magic packet filter */
#define	WOLCR_UNICAST	0x10	/* enable UNICAST filter */
#define	WOLCR_PTNMH	0x0f	/* enable pattern match filtering */

/* PWCFG.SET PWCFG.CLR (offset 0xa1, 0xa5 RWC) */
#define	PWCFG_SMIITIME	0x80	/* Internal MII interface timing */
#define	PWCFG_WOLTYPE	0x20	/* Drive WOL output by 1:pulse or 0:button */
#define	PWCFG_LEGCYWOL	0x10	/* Enale legacy wake on lan */
#define	PWCFG_WOLSR	0x02	/* Legacy WOL_SR shadow */
#define	PWCFG_WOLEN	0x01	/* Legacy WOL_EN shadow */

/* TESTREG.SET TESTREG.CLR (offset 0xa2, 0xa6 RWC) */
#define	TESTREG_SNORM	0x01	/* All power state capable while PHYTEST=0 */

/* WOLCG.SET WOLCG.CLR (offset 0xa3, 0xa7 RWC) */
#define	WOLCG_PMEOVR	0x80	/* Force PMEEN always over PME_EN (legacy) */
#define	WOLCG_SFDX	0x40	/* Full duplex in suspend well */
#define	WOLCG_SAM	0x20	/* Accept multicast in suspend well */
#define	WOLCG_SAB	0x10	/* Accept broadcast in suspend well */
#define	WOLCG_SMIIACC	0x08	/* MDC acceleration */
#define	WOLCG_SMIIOPT	0x04	/* MIIOPT to extend clock in suspend well */

/* PWRCSR.SET PSWCSR.CLR (offset 0xa8, 0xac RWC) undocumented */

/* PATTERN CRC0-3 (offset 0xb0-0xbf, RW) */
/* BYTEMASK0-3 CRC0-3 (offset 0xc0-0xff, RW) */
