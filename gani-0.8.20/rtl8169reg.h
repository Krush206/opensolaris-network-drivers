/*
 * %W% %E%
 * Macro definitions for Realtek 8169 giga ethernet controler
 * based on Realtek RTL8169 data sheet
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */

/*
 * Register offset
 */
#define	IDR		0x00	/* Base of ID registers */
#define	MAR		0x08	/* Base of multicast registers */
#define	DTCCRL		0x10	/* Dump Tally Counter (low) */
#define	DTCCRH		0x14	/* Dump Tally Counter (high) */
#define	TNPDS		0x20	/* Transmit Normal Priority Descriptors */
#define	TNPDSL		0x20	/* low 32-bit of TNPDS */
#define	TNPDSH		0x24	/* high 32-bit of TNPDS */
#define	THPDS		0x28	/* Transmit High Priority Descriptors */
#define	THPDSL		0x28	/* low 32-bit of THPDS */
#define	THPDSH		0x2c	/* low 32-bit of THPDS */
#define	FLASH		0x30	/* FLASH memory read/write register */
#define	ERBCR		0x34	/* Eealy Receive Byte Count register */
#define	ERSR		0x36	/* Early Rx Status register */
#define	CR		0x37	/* Command register */
#define	TPPoll		0x38	/* Transmit Priority Polling register*/
#define	IMR		0x3c	/* Interrupt Mask register */
#define	ISR		0x3e	/* Interrupt Status register */
#define	TCR		0x40	/* Transmit Configuration register */
#define	RCR		0x44	/* Receive Configuration register */
#define	TCTR		0x48	/* Timer CounT register */
#define	MPC		0x4c	/* Missed packet counter */
#define	CR9346		0x50	/* 93C46 command register */
#define	CFG0		0x51	/* Configration register 0 */
#define	CFG1		0x52	/* Configration register 1 */
#define	CFG2		0x53	/* Configration register 2 */
#define	CFG3		0x54	/* Configration register 3 */
#define	CFG4		0x55	/* Configration register 4 */
#define	TimerInt	0x58	/* Timer Interrupt register */
#define	MULINT		0x5c	/* Multiple Interrupt select */
#define	PHYAR		0x60	/* PHY Access Register */
#define	TBICSR		0x64	/* TBI Control and Status regiser 0 */
#define	TBIANAR		0x68	/* TBI Auto Negotiation Advertisement register*/
#define	TBILPAR		0x6a	/* TBI Auto Negotiation Link Partner register */
#define	PHYS		0x6c	/* PHY status */
#define	Wakeup0		0x84	/* Power Management Wakeup register 0 */
#define	Wakeup1		0x8c	/* Power Management Wakeup register 1 */
#define	Wakeup2LD	0x94	/* Power Management Wakeup register 1 */
#define	Wakeup2HD	0x9C	/* Power Management Wakeup register 1 */
#define	Wakeup3LD	0xa4	/* Power Management Wakeup register 1 */
#define	Wakeup3HD	0xaC	/* Power Management Wakeup register 1 */
#define	Wakeup4LD	0xb4	/* Power Management Wakeup register 1 */
#define	Wakeup4HD	0xbC	/* Power Management Wakeup register 1 */
#define	CRC0		0xc4	/* 16bit CRC of wakeup */
#define	CRC1		0xc6	/* 16bit CRC of wakeup */
#define	CRC2		0xc8	/* 16bit CRC of wakeup */
#define	CRC3		0xca	/* 16bit CRC of wakeup */
#define	CRC4		0xcc	/* 16bit CRC of wakeup */
#define	RMS		0xda	/* Rx packet Maximum Size */
#define	CpCR		0xe0	/* C+ Command register */
#define	RDSAR		0xe4	/* Receive Descriptor Start Address */
#define	RDSARL		0xe4	/* low 32-bit of RDSAR */
#define	RDSARH		0xe8	/* high 32-bit of RDSAR */
#define	ETThR		0xec	/* Early Transnit threshold register */
#define	FER		0xf0	/* Function Event register */
#define	FEMR		0xf4	/* Function Event Mask register */
#define	FFEMR		0xf8	/* Function forcs evet register */

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

/* ERSR Early Rx Status, offset 0x36 */
#define	ERSR_ERGood	0x08		/* Early Rx Good packet */
#define	ERSR_ERBad	0x04		/* Early Rx Bad packet */
#define	ERSR_EROVW	0x02		/* Early Rx OverWrite */
#define	ERSR_EROK	0x01		/* Early Rx OK */

/* CR : Command register,  offset 0x37  */
#define	CR_RST		0x10		/* Reset */
#define	CR_RE		0x08		/* Receiver enable */
#define	CR_TE		0x04		/* Transmitter enable */

#define	CR_BITS	"\020\005RST\004RE\003TE"

/* TPPoll: Transmit Priority Polling,  offset 0x38 */
#define	TPPoll_HPQ	0x80		/* High priority queue polling */
#define	TPPoll_NPQ	0x40		/* Normal priority queue polling */
#define	TPPoll_FSWInt	0x01		/* Forced software interrupt */

/* IMR  & ISR */
#define	INTR_SERR	0x8000		/* System Error Interrupt */
#define	INTR_TimeOut	0x4000		/* Time out interrupt */
#define	INTR_SWInt	0x0100		/* Software Interrupt */
#define	INTR_TDU	0x0080		/* Tx Descriptor Unavailable */
#define	INTR_FOVW	0x0040		/* Rx FIFO overflow */
#define	INTR_PUN	0x0020		/* Packet Underrun/Link change */
#define	INTR_RDU	0x0010		/* Rx Descriptor Unavailable */
#define	INTR_TER	0x0008		/* Transmit Error */
#define	INTR_TOK	0x0004		/* Transmit OK */
#define	INTR_RER	0x0002		/* Receive Error */
#define	INTR_ROK	0x0001		/* Receive OK */

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
#define	TCR_MACVER	0x7c800000	/* MAC version */		
#define		TCR_MACVER_B	0x00000000
#define		TCR_MACVER_D	0x00800000
#define		TCR_MACVER_E	0x04000000
#define		TCR_MACVER_F	0x10000000
#define	TCR_IFG		0x03080000	/* Interframe Gap */
#define	TCR_IFG_802_3	0x03000000	/* 802.3 standard */
#define	TCR_LBK		0x00060000	/* Loopback test */
#define	TCR_LBK_SHIFT		17
#define	TCR_LBK_NORMAL		(0 << TCR_LBK_SHIFT)
#define	TCR_LBK_DLBK		(1 << TCR_LBK_SHIFT)
#define	TCR_CRC		0x00010000	/* Inhibit Appending CRC */
#define	TCR_MXDMA	0x00000700
#define	TCR_MXDMA_SHIFT	8
#define	TCR_MXDMA_BASE		16
#define	TCR_MXDMA_16		(0 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_32		(1 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_64		(2 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_128		(3 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_256		(4 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_512		(5 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_1024		(6 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_UNLIMITED	(7 << TCR_MXDMA_SHIFT)

/* RCR: Receive Configuration register */

/* rx_mode_bits */
#define	RCR_MulERINT	0x00100000

#define	RCR_RER8	0x00010000
#define	RCR_RXFTH	0x0000e000	/* Receive FIFO threshold */
#define	RCR_RXFTH_SHIFT	13	
#define	RCR_RXFTH_64		(2 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_128		(3 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_256		(4 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_512		(5 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_1024		(6 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_NONE		(7 << RCR_RXFTH_SHIFT)

#define	RCR_MXDMA	0x00000700	/* */
#define	RCR_MXDMA_SHIFT	8
#define	RCR_MXDMA_64		(2 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_128		(3 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_256		(4 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_512		(5 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_1024		(6 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_UNLIMITED	(7 << RCR_MXDMA_SHIFT)
#define	RCR_9356SEL	0x00000040	/* EEPROM is 9356 */
#define	RCR_AER		0x00000020	/* Accept Error packet */
#define	RCR_AR		0x00000010	/* Accept runt */
#define	RCR_AB		0x00000008	/* Accept broadcast */
#define	RCR_AM		0x00000004	/* Accept Multicast */
#define	RCR_APM		0x00000002	/* Accept physical match */
#define	RCR_AAP		0x00000001	/* Accept all physical */

#define	RCR_ACCEPT_MODE		\
	(RCR_AER | RCR_AR | RCR_AB | RCR_AM | RCR_APM | RCR_AAP)

#define	RCR_MASK	\
	(RCR_MulERINT | RCR_RER8 | RCR_RXFTH | RCR_MXDMA | \
	 RCR_9356SEL | RCR_ACCEPT_MODE)


#define	RCR_BITS	\
  "\020\031MulInt\021RER8\0079356SEL\006AER\005AR\004AB\003AM\002APM\001AAP"

/* CR9346 : 0x50 */
#define	CR9346_EEM	0xc0
#define	CR9346_EEM_SHIFT	6	
#define		CR9346_EEM_NORMAL	(0 << CR9346_EEM_SHIFT)
#define		CR9346_EEM_AUTOLD	(1 << CR9346_EEM_SHIFT)
#define		CR9346_EEM_PROGRAM	(2 << CR9346_EEM_SHIFT)
#define		CR9346_EEM_WE		(3 << CR9346_EEM_SHIFT)
#define	CR9346_EECS	0x08
#define	CR9346_EESK	0x04
#define	CR9346_EEDI	0x02
#define	CR9346_EEDI_SHIFT	1
#define	CR9346_EEDO	0x01
#define	CR9346_EEDO_SHIFT	0

/* Config 0  offset 0x51 */

/* Config 1  offset 0x52 */
#define	CFG1_LEDS	0xc0
#define	CFG1_DVRLOAD	0x20
#define	CFG1_LWACT	0x10
#define	CFG1_MEMMAP	0x08
#define	CFG1_IOMAP	0x04
#define	CFG1_VPD	0x02
#define	CFG1_PMEn	0x01

/* Config 2  offset 0x53 */
#define	CFG2_AuxStatus		0x10
#define	CFG2_PCIBusWidth	0x08	/* 1:64bit 0:32bit */
#define	CFG2_PCICLKF		0x07
#define	CFG2_PCICLKF_33		0x00
#define	CFG2_PCICLKF_66		0x01

#define	CFG2_BITS	\
	"\020"		\
	"\005AuxStatus"	\
	"\004PCI64bit"

/* Config 3  offset 0x54 */
#define	CFG3_GNTSel	0x80	/* Gnt Select 0:No delay 1:delay one clock R */
#define	CFG3_MAGIC	0x20	/* Magic packet */
#define	CFG3_LINKUP	0x10
#define	CFG3_CARDB_EN	0x08	/* CardBus Enable (RO) */
#define	CFG3_CLKRUN_EN	0x04	/* CLK RUN Enable (RO) */
#define	CFG3_FUNCREGEN	0x02
#define	CFG3_FBTBEN	0x01

/* Configration register 4  offset 0x55 */
#define	CFG4_LWPMT	0x10
#define	CFG4_LWPTN	0x04	/* */

/* Configration register 5  offset 0x56 */
#define	CFG5_BWF	0x40	/* Broadcast Wakeup Frame */
#define	CFG5_MWF	0x20	/* Multicast Wakeup Frame */
#define	CFG5_UWF	0x10	/* Unicast Wakeup Frame */
#define	CFG5_LANWAKE	0x02	/* LANWake Signal 1:Enable 0:Disable */
#define	CFG5_PME_STS	0x01	/* PME status bit */

/* Basic mode status register */
/* Auto-negotiation Advertisement register */
/* Auto-negotiation Link Partner Ability register */
/* Auto-negotiation Expansion register */
/* Config5 */

/* PHYAR: PHY  Accress Register: offset 0x60 */
#define	PHYAR_FLAG	0x80000000
#define	PHYAR_REGADDR	0x001f0000
#define	PHYAR_REGADDR_SHIFT	16
#define	PHYAR_DATA	0x0000ffff

/* TBICSR: Ten Bit Interface Control and Status Register: offset 0x64 */
#define	TBICSR_Reset	0x80000000	/* TBI reset */
#define	TBICSR_LoopBack	0x40000000
#define	TBICSR_NWEn	0x20000000	/* TBI auto negotiation enable */
#define	TBICSR_ReNW	0x10000000	/* TBI Restart auto negotiation */
#define	TBICSR_LinkOk	0x02000000	/* TBI Link OK (RO)*/
#define	TBICSR_NWcomp	0x01000000	/* TBI Nway complete (RO)*/

/* TBIANAR: TBI Auto-Negotiation Ability register, offset 0x68 */
#define	TBIANAR_RF	0x3000
#define		TBIANAR_RF_SHIFT	12
#define		TBIANAR_RF_NoError	(0 << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_Offline	(1 << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_LinkFailure	(2 << TBIANAR_RF_SHIFT)
#define		TBIANAR_RF_ANError	(3 << TBIANAR_RF_SHIFT)
#define	TBIANAR_PAUSE	0x0100
#define	TBIANAR_ASM_DIR	0x0080
#define	TBIANAR_PS	(TBIANAR_ASM_DIR | TBIANAR_PAUSE)
#define		TBIANAR_PS_SHIFT	7
#define		TBIANAR_PS_NoPause	(0 << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_AsymPause	(1 << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_SymPause	(2 << TBIANAR_PS_SHIFT)
#define		TBIANAR_PS_Both		(3 << TBIANAR_PS_SHIFT)
#define	TBIANAR_FullDup	0x0020	/* Full Duplex ability */

/* TBILPAR: TBI Auto-Negotiation Link Partnar Ability register, offset 0x68 */
#define	TBILPAR_NextPage 0x8000	/* the Link partnar has a next page */
#define	TBILPAR_Ack	0x4000	/* */
#define	TBILPAR_RF	0x3000	/* Remote fault */
#define		TBILPAR_RF_SHIFT	12
#define		TBILPAR_RF_NoError	(0 << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_Offline	(1 << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_LinkFailure	(2 << TBIANAR_RF_SHIFT)
#define		TBILPAR_RF_ANError	(3 << TBIANAR_RF_SHIFT)
#define	TBILPAR_HalfDup	0x0400	/* the link partnar supports half duplex */
#define	TBILPAR_FullDup	0x0200	/* the link partnar supports half duplex */

/* PHYS: PHY(GMII or TBI) stutas register, offset 0x6c */
#define	PHYS_EnTBI	0x80	/* 1: TBI mode, 0:GMII mode */
#define	PHYS_TxFlow	0x40	/* Tx Flow control 1:enabeled */
#define	PHYS_RxFlow	0x20	/* Rx Flow control 1:enabeled */
#define	PHYS_1000MF	0x10	/* GMII only */
#define	PHYS_100M	0x08	/* GMII or MII only */
#define	PHYS_10M	0x04	/* GMII or MII only */
#define	PHYS_LinkSts	0x02	/* 1:ok, 0:no link */
#define	PHYS_FullDup	0x01	/* 1: Full duplex */

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
	

/* RMS: Receive Packet Maximum Size, offset 0xda */
#define	RMS_MASK	0x3fff

/* C+CR; C+Command, offset 0xe0 */
#define	CpCR_ENDIAN	0x0200	/* 1: Big Endian 0:Little endian*/
#define	CpCR_RxVLAN	0x0040	/* Rx VLAN de-tagging enable */
#define	CpCR_RxChkSum	0x0020	/* Rx Checksum offload enable */
#define	CpCR_DAC	0x0010	/* PCI dual cycle access enable */
#define	CpCR_MulRW	0x0008	/* PCI multiple read/write enable */

/* ETThR: Early Transmit Threshold register, 0ffset 0xec */
#define	ETThR_MASK	0x3f
#define	ETThR_UNIT	32

#define	FER_INTR	0x00008000
#define	FER_GWAKE	0x00000010

#define	FEMR_INTR	0x00008000
#define	FEMR_WKUP	0x00004000
#define	FEMR_GWAKE	0x00000010

/*
 * Offset to EPROM contents
 */
#define	EPROM_EthernetID	0x0e

struct tx_desc {
	uint32_t	txd0;
#define	TXD0_OWN	0x80000000	/* 1: the descriptor owned by the nic */
#define	TXD0_EOR	0x40000000	/* End of Descriptor Ring */
#define	TXD0_FS		0x20000000	/* First Segment Descriptor */
#define	TXD0_LS		0x10000000	/* Last Segment Descriptor */
#define	TXD0_LGSEN	0x08000000	/* Large Send */
#define	TXD0_LGSMSS	0x07ff0000
#define	TXD0_IPCS	0x00040000	/* IP checksum offload enable */
#define	TXD0_UDPCS	0x00020000	/* UDP checksum offload enable */
#define	TXD0_TCPCS	0x00010000	/* TCP checksum offload enable */
#define	TXD0_FRAMELEN	0x0000ffff

	uint32_t	txd1;
#define	TXD1_TAGC	0x00020000	/* VLAN tag control 1:enable */
#define	TXD1_VLANTAG	0x0000ff0f	/* VLAN tag */
#define	TXD1_VID(x)	(((x) >> 8) | ((x) << 8))	/* tag format */
#define	TXD1_PRI	0x000000e0	/* VLAN priority */
#define		TXD1_PRI_SHIFT	5
#define	TXD1_CFI	0x00000010	/* VLAN CFI bit */

	uint32_t	txd2;	/* Low 32-bit address of Tx buffer */
	uint32_t	txd3;	/* High 32-bit address of Tx buffer */
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
	uint32_t	rxd0;
#define	RXD0_OWN	0x80000000	/* 1: descriptor owned by nic */
#define	RXD0_EOR	0x40000000	/* End of Rx descriptor */
#define	RXD0_FS		0x20000000	/* First Segment descriptor */
#define	RXD0_LS		0x10000000	/* Last Segment descriptor */
#define	RXD0_MAR	0x08000000	/* Multicast address Packet received */
#define	RXD0_PAM	0x04000000	/* Physical address Matched */
#define	RXD0_BAR	0x02000000	/* Broadcast address Packet received */
#define	RXD0_BOVF	0x01000000	/* Buffer overflow */
#define	RXD0_FOVF	0x00800000	/* FIFO overflow */
#define	RXD0_RWT	0x00400000	/* Receive Watchdog Timer Expired */
#define	RXD0_RES	0x00200000	/* Receive Error Summary */
#define	RXD0_RUNT	0x00100000	/* Runt Packet */
#define	RXD0_CRC	0x00080000	/* CRC Error */
#define	RXD0_PID	0x00060000	/* Protocol ID */
#define		RXD0_PID_SHIFT	17
#define		RXD0_PID_NONIP	(0 << RXD0_PID_SHIFT)
#define		RXD0_PID_TCP	(1 << RXD0_PID_SHIFT)
#define		RXD0_PID_UDP	(2 << RXD0_PID_SHIFT)
#define		RXD0_PID_IP	(3 << RXD0_PID_SHIFT)
#define	RXD0_IPF	0x00010000	/* IP checksum failure */
#define	RXD0_UDPF	0x00008000	/* UDP checksum failure */
#define	RXD0_TCPF	0x00004000	/* TCP checksum failure */
#define	RXD0_FRAMELEN	0x00003fff

	uint32_t	rxd1;
#define	RXD1_TAVA	0x00010000	/* Tan Available */
#define	RXD1_VLANTAG	0x0000ffff	/* VLAN Tag */

	uint32_t	rxd2;	/* Low 32-bit Address of Rx buffer */
	uint32_t	rxd3;	/* High 32-bit Addres of Rx buffer */
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
