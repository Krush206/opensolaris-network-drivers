/*
 * %W% %E%
 * Macro definitions for Realtek RTL8169/RTL8168 giga bit ethernet controler
 * based on Realtek RTL8169/RTL8168 data sheet
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */
#ifndef	_RTL8169REG_H_
#define	_RTL8169REG_H_

/*
 * Register offset
 */
#define	IDR		0x00	/* Base of ID registers */
#define	MAR		0x08	/* Base of multicast registers */
#define	DTCCR		0x10	/* Dump Tally Counter */
#define	DTCCRL		0x10	/* low 32-bit of DTCCR */
#define	DTCCRH		0x14	/* high 32-bit of DTCCR */
#define	CustomLED	0x18	/* 8168 */
#define	TNPDS		0x20	/* Transmit Normal Priority Descriptors */
#define	TNPDSL		0x20	/* low 32-bit of TNPDS */
#define	TNPDSH		0x24	/* high 32-bit of TNPDS */
#define	THPDS		0x28	/* Transmit High Priority Descriptors */
#define	THPDSL		0x28	/* low 32-bit of THPDS */
#define	THPDSH		0x2c	/* high 32-bit of THPDS */
#define	FLASH		0x30	/* FLASH memory read/write register (8169) */
#define	ERBCR		0x34	/* Eealy Receive Byte Count register (8169) */
#define	ERSR		0x36	/* Early Rx Status register (8169) */
#define	CR		0x37	/* Command register */
#define	TPPoll		0x38	/* Transmit Priority Polling register*/
#define	IMR		0x3c	/* Interrupt Mask register */
#define	ISR		0x3e	/* Interrupt Status register */
#define	TCR		0x40	/* Transmit Configuration register */
#define	RCR		0x44	/* Receive Configuration register */
#define	TCTR		0x48	/* Timer CounT register */
#define	MPC		0x4c	/* Missed packet counter (8169) */
#define	CR9346		0x50	/* 93C46 command register */
#define	CFG0		0x51	/* Configration register 0 */
#define	CFG1		0x52	/* Configration register 1 */
#define	CFG2		0x53	/* Configration register 2 */
#define	CFG3		0x54	/* Configration register 3 */
#define	CFG4		0x55	/* Configration register 4 */
#define	CFG5		0x56	/* Configration register 5 (8169) */
#define	TDFNR		0x57	/* (8168) */
#define	TimerInt	0x58	/* Timer Interrupt register */
#define	MULINT		0x5c	/* Multiple Interrupt select (8169) */
#define	PHYAR		0x60	/* PHY Access Register */
#define	TBICSR		0x64	/* TBI Control and Status regiser (8169) */
#define	TBIANAR		0x68	/* TBI Auto Negotiation Advertisement register*/
#define	TBILPAR		0x6a	/* TBI Auto Negotiation Link Partner register */
#define	PHYS		0x6c	/* PHY status */
#define	GPIO		0x6e	/* 8168 */
#define	PMCH		0x6f	/* 8168 */
#define	ERIDR		0x70	/* 8168 */
#define	ERIAR		0x74	/* 8168 */
#define	EPHY_RXER_NUM	0x7c	/* 8168 */
#define	EPHYAR		0x80	/* PCI-E PHY Access Register */
#define	Wakeup0		0x84	/* Power Management Wakeup register 0 */
#define	Wakeup1		0x8c	/* Power Management Wakeup register 1 */
#define	Wakeup2LD	0x94	/* Power Management Wakeup register 2 */
#define	Wakeup2HD	0x9C	/* Power Management Wakeup register 2 */
#define	Wakeup3LD	0xa4	/* Power Management Wakeup register 3 */
#define	Wakeup3HD	0xac	/* Power Management Wakeup register 3 */
#define	Wakeup4LD	0xb4	/* Power Management Wakeup register 4 */
#define	Wakeup4HD	0xbC	/* Power Management Wakeup register 4 */
#define	OCPDR		0xb0	/* (8168) */
#define	MACOCP		OCPDR	/* (8168) */
#define	OCPAR		0xb4	/* (8168) */
#define	PHYOCP		0xb8	/* (8168) */
#define	CRC0		0xc4	/* 16bit CRC of wakeup */
#define	CRC1		0xc6	/* 16bit CRC of wakeup */
#define	CRC2		0xc8	/* 16bit CRC of wakeup */
#define	CRC3		0xca	/* 16bit CRC of wakeup */
#define	CRC4		0xcc	/* 16bit CRC of wakeup */
#define	MCUCmd		0xd3	/* */
#define	RMS		0xda	/* Rx packet Maximum Size */
#define	EFUSEAR		0xdc	/* E-FUSE access register */
#define	CpCR		0xe0	/* C+ Command register */
#define	INTRMOD		0xe2	/* interrupt moderation */
#define	RDSAR		0xe4	/* Receive Descriptor Start Address */
#define	RDSARL		0xe4	/* low 32-bit of RDSAR */
#define	RDSARH		0xe8	/* high 32-bit of RDSAR */
#define	ETThR		0xec	/* Early Transnit threshold register (8169) */
#define	MTPS		ETThR	/* Max Tx packet Size registger (8168) */
#define	FER		0xf0	/* Function Event register (8169) */
#define	FEMR		0xf4	/* Function Event Mask register (8169) */
#define	FFEMR		0xf8	/* Function forcs evet register (8169) */

/* 8168 and 810x */
#define	CSIDR		0x64
#define	CSIAR		0x68
#define	DBGREG		0xd1
#define	PHYIO		0xf8

/* Tally Counter Command register */
#define	DTCCR_ADDR	0xffffffc0	/* low 32 bit of base address */
#define	DTCCR_CMD	0x00000008

/* Tally counter offsets */
#define	TC_TxOK_8	0	/* 8byte */
#define	TC_RxOK_8	8	/* 8byte */
#define	TC_TxEr_8	16	/* 8byte */
#define	TC_RxEr_4	24	/* 4byte */
#define	TC_MissPkt_2	28	/* 2byte */
#define	TC_FAE_2	30	/* 2byte */
#define	TC_Tx1Col_4	32	/* 4byte */
#define	TC_TxMCol_4	36	/* 4byte */
#define	TC_RxOkPhy_8	40	/* 8byte */
#define	TC_RxOkBrd_8	48	/* 8byte */
#define	TC_RxOkMu_4	56	/* 4byte */
#define	TC_TxAbt_2	60	/* 2byte */
#define	TC_TxUndrn_2	62	/* 2byte */

struct rtl8169_tally_counters {
	uint64_t	tc_txok;	/* 8byte */

	uint64_t	tc_rxok;	/* 8byte */

	uint64_t	tc_txer;	/* 8byte */

	uint32_t	tc_rxer;	/* 4byte */
	uint16_t	tc_misspkt;	/* 2byte */
	uint16_t	tc_fae;		/* 2byte */

	uint32_t	tc_tx1col;	/* 4byte */
	uint32_t	tc_txmcol;	/* 4byte */

	uint64_t	tc_rxokphy;	/* 8byte */

	uint64_t	tc_rxokbrd;	/* 8byte */

	uint32_t	tc_rxokmu;	/* 4byte */
	uint16_t	tc_txabt;	/* 2byte */
	uint16_t	tc_txundrn;	/* 2byte */
};

#define	TC_SIZE		64	/* 64 bit aligned */

/* ERSR Early Rx Status, offset 0x36 (8169only) */
#define	ERSR_ERGood	0x08U		/* Early Rx Good packet */
#define	ERSR_ERBad	0x04U		/* Early Rx Bad packet */
#define	ERSR_EROVW	0x02U		/* Early Rx OverWrite */
#define	ERSR_EROK	0x01U		/* Early Rx OK */

/* CR : Command register,  offset 0x37  */
#define	CR_STOP		0x80U		/* Stop request (8168C/CP) */
#define	CR_RST		0x10U		/* Reset */
#define	CR_RE		0x08U		/* Receiver enable */
#define	CR_TE		0x04U		/* Transmitter enable */

#define	CR_BITS	"\020\005RST\004RE\003TE"

/* TPPoll: Transmit Priority Polling,  offset 0x38 */
#define	TPPoll_HPQ	0x80U		/* High priority queue polling */
#define	TPPoll_NPQ	0x40U		/* Normal priority queue polling */
#define	TPPoll_FSWInt	0x01U		/* Forced software interrupt */

/* IMR  & ISR */
#define	INTR_SERR	0x8000U		/* System Error Interrupt (8169) */
#define	INTR_TimeOut	0x4000U		/* Time out interrupt */
#define	INTR_SWInt	0x0100U		/* Software Interrupt */
#define	INTR_TDU	0x0080U		/* Tx Descriptor Unavailable */
#define	INTR_FOVW	0x0040U		/* Rx FIFO overflow */
#define	INTR_PUN	0x0020U		/* Packet Underrun/Link change (8169)*/
#define	INTR_LinkChg	INTR_PUN	/* Link change (8168)*/
#define	INTR_RDU	0x0010U		/* Rx Descriptor Unavailable */
#define	INTR_TER	0x0008U		/* Transmit Error */
#define	INTR_TOK	0x0004U		/* Transmit OK */
#define	INTR_RER	0x0002U		/* Receive Error */
#define	INTR_ROK	0x0001U		/* Receive OK */

#define	INTR_BITS	\
	"\020"	\
	"\020SERR"	\
	"\017TimeOut"	\
	"\011SWInt"	\
	"\010TDU"	\
	"\007FOVW"	\
	"\006PUN"	\
	"\005RDU"	\
	"\004TER"	\
	"\003TOK"	\
	"\002RER"	\
	"\001ROK"

/* TCR: Transmit Configuration register */
#define	TCR_IFG		0x03080000U	/* Interframe Gap */
#define		TCR_IFG_802_3		0x03000000U	/* 802.3 standard */
#define		TCR_IFG_HALF_8169	0x02000000U
#define	TCR_LBK		0x00060000U	/* Loopback test */
#define		TCR_LBK_SHIFT		17
#define		TCR_LBK_NORMAL		(0U << TCR_LBK_SHIFT)
#define		TCR_LBK_DLBK		(1U << TCR_LBK_SHIFT)
#define	TCR_CRC		0x00010000U	/* Inhibit Appending CRC */
#define	TCR_MXDMA	0x00000700U
#define		TCR_MXDMA_SHIFT		8
#define		TCR_MXDMA_BASE		16
#define		TCR_MXDMA_16		(0U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_32		(1U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_64		(2U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_128		(3U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_256		(4U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_512		(5U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_1024		(6U << TCR_MXDMA_SHIFT)
#define		TCR_MXDMA_UNLIMITED	(7U << TCR_MXDMA_SHIFT)

/* RCR: Receive Configuration register */

/* rx_mode_bits */
#define	RCR_MulERINT	0x00800000U	/* 8169only */

#define	RCR_RER8	0x00010000U
#define	RCR_128_INT	0x00008000U
#define	RCR_FET_MULTI	0x00004000U
#define	RCR_HALF_REFET	0x00002000U
#define	RCR_RXFTH	0x0000e000U	/* Receive FIFO threshold */
#define		RCR_RXFTH_SHIFT		13	
#define		RCR_RXFTH_64		(2U << RCR_RXFTH_SHIFT)
#define		RCR_RXFTH_128		(3U << RCR_RXFTH_SHIFT)
#define		RCR_RXFTH_256		(4U << RCR_RXFTH_SHIFT)
#define		RCR_RXFTH_512		(5U << RCR_RXFTH_SHIFT)
#define		RCR_RXFTH_1024		(6U << RCR_RXFTH_SHIFT)
#define		RCR_RXFTH_SF		(7U << RCR_RXFTH_SHIFT)

#define	RCR_EARLY_OFF	0x00000800U	/* 8168 */
#define	RCR_MXDMA	0x00000700U	/* */
#define		RCR_MXDMA_SHIFT		8
#define		RCR_MXDMA_64		(2U << RCR_MXDMA_SHIFT)
#define		RCR_MXDMA_128		(3U << RCR_MXDMA_SHIFT)
#define		RCR_MXDMA_256		(4U << RCR_MXDMA_SHIFT)
#define		RCR_MXDMA_512		(5U << RCR_MXDMA_SHIFT)
#define		RCR_MXDMA_1024		(6U << RCR_MXDMA_SHIFT)
#define		RCR_MXDMA_UNLIMITED	(7U << RCR_MXDMA_SHIFT)
#define	RCR_9356SEL	0x00000040U	/* EEPROM is 9356 */
#define	RCR_AER		0x00000020U	/* Accept Error packet */
#define	RCR_AR		0x00000010U	/* Accept runt */
#define	RCR_AB		0x00000008U	/* Accept broadcast */
#define	RCR_AM		0x00000004U	/* Accept Multicast */
#define	RCR_APM		0x00000002U	/* Accept physical match */
#define	RCR_AAP		0x00000001U	/* Accept all physical */

#define	RCR_ACCEPT_MODE		\
	(RCR_AER | RCR_AR | RCR_AB | RCR_AM | RCR_APM | RCR_AAP)

#define RCR_MASK \
	(RCR_MulERINT | RCR_RER8 | \
	RCR_128_INT | RCR_FET_MULTI | RCR_HALF_REFET | \
	RCR_RXFTH | RCR_MXDMA | \
	RCR_9356SEL | RCR_ACCEPT_MODE)

#define	RCR_BITS	\
	"\020"	\
	"\030MulERInt"	\
	"\021RER8"	\
	"\020128INT"	\
	"\017FET_MULTI"	\
	"\016HALF_REFET"	\
	"\0079356SEL"	\
	"\006AER"	\
	"\005AR"	\
	"\004AB"	\
	"\003AM"	\
	"\002APM"	\
	"\001AAP"

/* CR9346 : 0x50 */
#define	CR9346_EEM	0xc0U
#define		CR9346_EEM_SHIFT	6
#define		CR9346_EEM_NORMAL	(0U << CR9346_EEM_SHIFT)
#define		CR9346_EEM_AUTOLD	(1U << CR9346_EEM_SHIFT)
#define		CR9346_EEM_PROGRAM	(2U << CR9346_EEM_SHIFT)
#define		CR9346_EEM_WE		(3U << CR9346_EEM_SHIFT)
#define	CR9346_EECS	0x08U
#define	CR9346_EESK	0x04U
#define	CR9346_EEDI	0x02U
#define		CR9346_EEDI_SHIFT	1
#define	CR9346_EEDO	0x01U
#define		CR9346_EEDO_SHIFT	0

/* Config 0  offset 0x51 */

/* Config 1  offset 0x52 */
#define	CFG1_LEDS	0xc0U
#define	CFG1_LEDS1	0x80U
#define	CFG1_LEDS0	0x40U
#define	CFG1_DVRLOAD	0x20U	/* 8169 */
#define	CFG1_LWACT	0x10U	/* 8169 */
#define	CFG1_SPEEDDOWN	0x10U	/* 8168 */
#define	CFG1_MEMMAP	0x08U
#define	CFG1_IOMAP	0x04U
#define	CFG1_VPD	0x02U
#define	CFG1_PMEn	0x01U

/* Config 2  offset 0x53 */
#define	CFG2_AuxStatus		0x10U
#define	CFG2_PCIBusWidth	0x08U	/* 1:64bit 0:32bit (8169) */
#define	CFG2_PCICLKF		0x07U	/* 8169 */
#define	CFG2_PCICLKF_33		0x00U
#define	CFG2_PCICLKF_66		0x01U	/* unreliable */

#define	CFG2_BITS	\
	"\020"		\
	"\005AuxStatus"	\
	"\004PCI64bit"

/* Config 3  offset 0x54 */
#define	CFG3_GNTSel	0x80U	/* Gnt Select 0:No delay 1:delay one clock R */
#define	CFG3_MAGIC	0x20U	/* Magic packet */
#define	CFG3_LINKUP	0x10U
#define	CFG3_CARDB_EN	0x08U	/* CardBus Enable (RO) (8169) */
#define	CFG3_CLKRUN_EN	0x04U	/* CLK RUN Enable (RO) (8169) */
#define	CFG3_JUMBO	0x04U	/* enable jumbo frame (8168) */
#define	CFG3_FUNCREGEN	0x02U	/* (8169) */
#define	CFG3_FBTBEN	0x01U	/* (8169) */
#define	CFG3_BEACON_EN	0x01U	/* (8168) */

/* Configration register 4  offset 0x55 */
#define	CFG4_LWPMT	0x10U	/* (8169) */
#define	CFG4_LWPTN	0x04U	/* (8169) */
#define	CFG4_JUMBO	0x02U	/* enable jumbo frame (8168) */

/* Configration register 5  offset 0x56 */
#define	CFG5_BWF	0x40U	/* Broadcast Wakeup Frame */
#define	CFG5_MWF	0x20U	/* Multicast Wakeup Frame */
#define	CFG5_UWF	0x10U	/* Unicast Wakeup Frame */
#define	CFG5_LANWAKE	0x02U	/* LANWake Signal 1:Enable 0:Disable (8169) */
#define	CFG5_PME_STS	0x01U	/* PME status bit (8169) */

/* Basic mode status register */
/* Auto-negotiation Advertisement register */
/* Auto-negotiation Link Partner Ability register */
/* Auto-negotiation Expansion register */
/* Config5 */

/* PHYAR: PHY  Accress Register: offset 0x60 */
#define	PHYAR_FLAG	0x80000000U
#define	PHYAR_REGADDR	0x001f0000U
#define		PHYAR_REGADDR_SHIFT	16
#define	PHYAR_DATA	0x0000ffffU

/* TBICSR: Ten Bit Interface Control and Status Register: offset 0x64 (8169) */
#define	TBICSR_Reset	0x80000000U	/* TBI reset */
#define	TBICSR_LoopBack	0x40000000U
#define	TBICSR_NWEn	0x20000000U	/* TBI auto negotiation enable */
#define	TBICSR_ReNW	0x10000000U	/* TBI Restart auto negotiation */
#define	TBICSR_LinkOk	0x02000000U	/* TBI Link OK (RO)*/
#define	TBICSR_NWcomp	0x01000000U	/* TBI Nway complete (RO)*/

/* TBIANAR: TBI Auto-Negotiation Ability register, offset 0x68 (8169) */
#define	TBIANAR_RF	0x3000U
#define		TBIANAR_RF_SHIFT	12
#define		TBIANAR_RF_NoError	(0U << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_Offline	(1U << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_LinkFailure	(2U << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_ANError	(3U << TBIANAR_RF_SHIFT)
#define	TBIANAR_PAUSE	0x0100U
#define	TBIANAR_ASM_DIR	0x0080U
#define	TBIANAR_PS	(TBIANAR_ASM_DIR | TBIANAR_PAUSE)
#define		TBIANAR_PS_SHIFT	7
#define		TBIANAR_PS_NoPause	(0U << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_AsymPause	(1U << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_SymPause	(2U << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_Both		(3U << TBIANAR_PS_SHIFT)
#define	TBIANAR_FullDup	0x0020	/* Full Duplex ability */

/* TBILPAR: TBI Auto-Negotiation Link Partnar Ability reg, offset 0x68 (8169)*/
#define	TBILPAR_NextPage 0x8000U	/* the Link partnar has a next page */
#define	TBILPAR_Ack	0x4000U	/* */
#define	TBILPAR_RF	0x3000U	/* Remote fault */
#define		TBILPAR_RF_SHIFT	12
#define		TBILPAR_RF_NoError	(0U << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_Offline	(1U << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_LinkFailure	(2U << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_ANError	(3U << TBIANAR_RF_SHIFT)
#define	TBILPAR_HalfDup	0x0400U	/* link partnar supports half duplex */
#define	TBILPAR_FullDup	0x0200U	/* link partnar supports full duplex */

/* PHYS: PHY(GMII or TBI) stutas register, offset 0x6c */
#define	PHYS_EnTBI	0x80U	/* 1: TBI mode, 0:GMII mode (8169) */
#define	PHYS_TxFlow	0x40U	/* Tx Flow control 1:enabeled */
#define	PHYS_RxFlow	0x20U	/* Rx Flow control 1:enabeled */
#define	PHYS_1000MF	0x10U	/* GMII only */
#define	PHYS_100M	0x08U	/* GMII or MII only */
#define	PHYS_10M	0x04U	/* GMII or MII only */
#define	PHYS_LinkSts	0x02U	/* 1:ok, 0:no link */
#define	PHYS_FullDup	0x01U	/* 1: full duplex, 0: half duplex */

#define	PHYS_BITS	\
	"\020"	\
	"\010EnTBI"	\
	"\007TxFlow"	\
	"\006RxFlow"	\
	"\0051000MF"	\
	"\004100M"	\
	"\00310M"	\
	"\002LinkSts"	\
	"\001FullDup"

/* EPHYAR: EPHY Accress Register: offset 0x60 */
#define	EPHYAR_FLAG	0x80000000U
#define	EPHYAR_REGADDR	0x001f0000U
#define		EPHYAR_REGADDR_SHIFT	16
#define	EPHYAR_DATA	0x0000ffffU

/* CSIAR: CSI Accress Register: offset 0x68 */
#define	CSIAR_FLAG	0x80000000U
#define	CSIAR_ByteEn	0x0000f000U
#define		CSIAR_ByteEn_SHIFT	12
#define	CSIAR_ADDR	0x00000fffU

/* ERIAR: ERI access register */
#define	ERIAR_FLAG	0x80000000U
#define	ERIAR_WRITE	0x80000000U
#define	ERIAR_READ	0x00000000U
#define		ERIAR_ADDR_ALIGN	4
		/* ERI access register address must be 4 byte alignment */
#define	ERIAR_ExGMAC	0
#define	ERIAR_MSIX	1
#define	ERIAR_ASF	2
#define	ERIAR_OOB	2
#define	ERIAR_TYPE_SHIFT	16
#define	ERIAR_ByteEn	0x0f
#define		ERIAR_ByteEn_SHIFT	12


/* OCPDR: offset 0xb0 (8168) */
#define	OCPDR_WRITE	0x80000000U
#define	OCPDR_READ	0x00000000U
#define	OCPDR_REG	0x00ff0000U
#define		OCPDR_GPHY_REG_SHIFT	16
/* #define		OCPDR_REG_MASK	0xFF */
#define	OCPDR_DATA	0x0000ffffU

/* OCPAR: offset 0xb4 (8168) */
#define	OCPAR_FLAG		0x80000000U
#define	OCPAR_GPHY_WRITE	0x8000f060U
#define	OCPAR_GPHY_READ		0x0000f060U

/* PHYOCP/MACOCP */
#define	OCPR_WRITE	0x80000000
#define	OCPR_READ	0x00000000
#define		OCPR_ADDR_REG_SHIFT	16
#define	OCPR_FLAG	0x80000000
#define	OCP_STD_PHY_BASE_PAGE	0x0A40

#define	OOB_CMD_RESET		0x00
#define	OOB_CMD_DRIVER_START	0x05
#define	OOB_CMD_DRIVER_STOP	0x06
#define	OOB_CMD_SET_IPMAC	0x41

/* DBGREG: debug register, offset 0xd1 */
#define	DBGREG_FIX_NAK_1	0x10U
#define	DBGREG_FIX_NAK_2	0x08U
#define	DBGREG_PIN_E2		0x01U

/* MCUCmd_reg */
#define	MCUCmd_NOW_IS_OOB	0x80U	/* (1U << 7) */
#define	MCUCmd_TXFIFO_EMPTY	0x20U	/* (1U << 5) */
#define	MCUCmd_RXFIFO_EMPTY	0x10U	/* (1U << 4) */

/* RMS: Receive Packet Maximum Size, offset 0xda */
#define	RMS_MASK	0x3fffU

/* C+CR; C+Command, offset 0xe0 */
#define	CpCR_ENDIAN	0x0200U	/* 1: Big Endian 0:Little endian (8169) */
#define	CpCR_BIST	0x8000U
#define	CpCR_Macdbgo_oe	0x4000U
#define	CpCR_EnAnaPLL	0x4000U	/* 8169 */
#define	CpCR_NORMAL	0x2000U
#define	CpCR_HALFDUP	0x1000U	/* force half duplex */
#define	CpCR_RXFLOW	0x0800U	/* force to enable rx flow control */
#define	CpCR_TXFLOW	0x0400U	/* force to enable tx flow control */
#define	CpCR_CXPL	0x0200U
#define	CpCR_ASF	0x0100U
#define	CpCR_StatDis	0x0080U
#define	CpCR_RxVLAN	0x0040U	/* Rx VLAN de-tagging enable */
#define	CpCR_RxChkSum	0x0020U	/* Rx Checksum offload enable */
#define	CpCR_DAC	0x0010U	/* PCI dual cycle access enable (8169) */
#define	CpCR_MulRW	0x0008U	/* PCI multiple read/write enable (8169) */
#define	CpCR_Macdbgo_sel	0x001cU
#define	CpCR_INTT_3	0x0003U
#define	CpCR_INTT_2	0x0002U
#define	CpCR_INTT_1	0x0001U
#define	CpCR_INTT_0	0x0000U

/* EFUSEAR: E-FUSE accress register, 0ffset 0xdc (8168) */
#define	EFUSE_WRITE	0x80000000U
#define	EFUSE_WRITE_OK	0x00000000U
#define	EFUSE_READ	0x00000000U
#define	EFUSE_READ_OK	0x80000000U
#define	EFUSE_REG	0x0003ff00U
#define		EFUSE_REG_SHIFT	8
#define		EFUSE_REG_MASK	(EFUSE_REG >> EFUSE_REG_SHIFT)
#define	EFUSE_DATA		0x000000ffU

/* ETThR: Early Transmit Threshold register, 0ffset 0xec (8169) */
#define	ETThR_MASK	0x3fU
#define	ETThR_UNIT	32U

/* MTPS: Max Transmit Packet Size register, 0ffset 0xec (8168) */
#define	MTPS_MASK	0x3fU
#define	MTPS_UNIT	128U

#define	FER_INTR	0x00008000U
#define	FER_GWAKE	0x00000010U

#define	FEMR_INTR	0x00008000U
#define	FEMR_WKUP	0x00004000U
#define	FEMR_GWAKE	0x00000010U

/* PHYIO at 0xf8 */
#define	PHYIO_FLAG	0x80000000U
#define	PHYIO_WRITE	0x80000000U
#define	PHYIO_READ	0x00000000U
#define	PHYIO_REG	0x001f0000U
#define		PHYIO_REG_MASK	0x001f0000U
#define		PHYIO_REG_SHIFT	16
#define	PHYIO_DATA	0xffffU

/*
 * Offset to EPROM contents
 */
#define	EPROM_EthernetID	0x0e

struct tx_desc {
	volatile uint32_t	txd0;
#define	TXD0_OWN	0x80000000U	/* 1: the descriptor owned by the nic */
#define	TXD0_EOR	0x40000000U	/* End of Descriptor Ring */
#define	TXD0_FS		0x20000000U	/* First Segment Descriptor */
#define	TXD0_LS		0x10000000U	/* Last Segment Descriptor */
#define	TXD0_LGSEN	0x08000000U	/* Large Send */
#define	TXD0_LGSMSS	0x07ff0000U
#define		TXD0_LGSMSS_SHIFT	16
#define	TXD0_IPCS	0x00040000U	/* IP checksum offload enable */
#define	TXD0_UDPCS	0x00020000U	/* UDP checksum offload enable */
#define	TXD0_TCPCS	0x00010000U	/* TCP checksum offload enable */
#define	TXD0_FRAMELEN	0x0000ffffU

	volatile uint32_t	txd1;
#define	TXD1_UDPCS	0x80000000U	/* UDP checksum offload enable */
#define	TXD1_TCPCS	0x40000000U	/* TCP checksum offload enable */
#define	TXD1_IPCS	0x20000000U	/* IP checksum offload enable */
#define	TXD1_TAGC	0x00020000U	/* VLAN tag control 1:enable */
#define	TXD1_VLAN	0x0000ffffU	/* VLAN tagid + pri + cfi */
#define	TXD1_VID(x)	((((x) >> 8) & 0x0fU) | (((x) << 8) & 0xff00U))
#define	TXD1_PRI	0x000000e0U	/* VLAN priority */
#define		TXD1_PRI_SHIFT	5
#define	TXD1_CFI	0x00000010U	/* VLAN CFI bit */
#define		TXD1_CFI_SHIFT	4

#define	TXD1_VTAG(x)	((((x) >> 8) & 0xffU) | (((x) << 8) & 0xff00U))

	volatile uint32_t	txd2;	/* Low 32-bit address of Tx buffer */
	volatile uint32_t	txd3;	/* High 32-bit address of Tx buffer */
};

#define	TXD0_BITS	\
	"\020"		\
	"\040OWN"	\
	"\037EOR"	\
	"\036FS"	\
	"\035LS"	\
	"\034LGSEN"	\
	"\023IPCS"	\
	"\022UDPCS"	\
	"\021TCPCS"

struct rx_desc {
	volatile uint32_t	rxd0;
#define	RXD0_OWN	0x80000000U	/* 1: descriptor owned by nic */
#define	RXD0_EOR	0x40000000U	/* End of Rx descriptor */
#define	RXD0_FS		0x20000000U	/* First Segment descriptor */
#define	RXD0_LS		0x10000000U	/* Last Segment descriptor */
#define	RXD0_MAR	0x08000000U	/* Multicast address Packet received */
#define	RXD0_PAM	0x04000000U	/* Physical address Matched */
#define	RXD0_BAR	0x02000000U	/* Broadcast address Packet received */
#define	RXD0_BOVF	0x01000000U	/* Buffer overflow (8169) */
#define	RXD0_FOVF	0x00800000U	/* FIFO overflow (8169) */
#define	RXD0_RWT	0x00400000U	/* Receive Watchdog Timer Expired */
#define	RXD0_RES	0x00200000U	/* Receive Error Summary */
#define	RXD0_RUNT	0x00100000U	/* Runt Packet */
#define	RXD0_CRC	0x00080000U	/* CRC Error */
#define	RXD0_PID	0x00060000U	/* Protocol ID */
#define		RXD0_PID_SHIFT	17
#define		RXD0_PID_NONIP	(0U << RXD0_PID_SHIFT)
#define		RXD0_PID_TCP	(1U << RXD0_PID_SHIFT)
#define		RXD0_PID_UDP	(2U << RXD0_PID_SHIFT)
#define		RXD0_PID_IP	(3U << RXD0_PID_SHIFT)
#define		RXD0_PID_IP_NEW	(0U << RXD0_PID_SHIFT)
#define	RXD0_IPF	0x00010000U	/* IP checksum failure */
#define	RXD0_UDPF	0x00008000U	/* UDP checksum failure */
#define	RXD0_TCPF	0x00004000U	/* TCP checksum failure */
#define	RXD0_FRAMELEN	0x00003fffU

	volatile uint32_t	rxd1;
#define	RXD1_IPv6	0x80000000U
#define	RXD1_IPv4	0x40000000U
#define	RXD1_TAVA	0x00010000U	/* Tan Available */
#define	RXD1_VLANTAG	0x0000ffffU	/* VLAN Tag */

	volatile uint32_t	rxd2;	/* Low 32-bit Address of Rx buffer */
	volatile uint32_t	rxd3;	/* High 32-bit Addres of Rx buffer */
};

#define	RXD0_BITS	\
	"\020"		\
	"\040OWN"	\
	"\037EOR"	\
	"\036FS"	\
	"\035LS"	\
	"\034MAR"	\
	"\033PAM"	\
	"\032BAR"	\
	"\031BOVF"	\
	"\030FOVF"	\
	"\027RWT"	\
	"\026RES"	\
	"\025RUNT"	\
	"\024CRC"	\
	"\021IPF"	\
	"\020UDPF"	\
	"\017TCPF"

#endif	/* _RTL8169REG_H_ */
