/*
 * @(#)rtl8139reg.h	1.7 03/10/23
 * Macro definitions for Realtek 8129/8139 fast ethernet controler
 * based on Realtek RTL8139C data sheet
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */

/*
 * Register offset
 */
#define	IDR	0x0000	/* Base of ID registers */
#define	MAR	0x0008	/* Base of multicast registers */
#define	TSD	0x0010	/* Base of Transmit Status of Descriptor */
#define	TSAD	0x0020	/* Base of Transmit Start Address of Descriptor */
#define	RBSTART	0x0030	/* Receive Buffer Start Address */
#define	ERBCR	0x0034	/* Eealy Receive Byte Count register */
#define	ERSR	0x0x36	/* Early Rx Status register */
#define	CR	0x0037	/* Command register */
#define	CAPR	0x0038	/* Current Addresss of Packet Read */
#define	CBR	0x003a	/* Current Buffer Addresss */
#define	IMR	0x003c	/* Interrupt Mask register */
#define	ISR	0x003e	/* Interrupt Status register */
#define	TCR	0x0040	/* Transmit Configuration register */
#define	RCR	0x0044	/* Receive Configuration register */
#define	TCTR	0x0048	/* Timer CounT register */
#define	MPC	0x004c	/* Missed packet counter */
#define	CR9346	0x0050	/* 93C46 command register */
#define	CONFIG0	0x0051	/* Configration register 0 */
#define	CONFIG1	0x0052	/* Configration register 1 */
#define	TimInt	0x0054	/* Timer Interrupt register */	
#define	MSR	0x0058	/* Media status register */
#define	CONFIG3	0x0059	/* Configuration register 3 */
#define	CONFIG4	0x005a	/* Configuration register 4 (8139 only) */
#define MII8129	0x005a	/* MII interface (8129) */
#define	HLTCLK	0x005b	/* power control */
#define	MULINT	0x005c	/* Multiple Interrupt select */
#define	RERID	0x005e	/* PCI Revision ID = 0x10 */
#define	TSADESC	0x0060	/* Transmit status of all descriptors */
#define	BMCR	0x0062	/* Basic Mode Control register */
#define	BMSR	0x0064	/* Basic Mode Status register */
#define	ANAR	0x0066	/* Auto Negotiation Advertisement register */
#define	ANLPAR	0x0068	/* Auto Negotiation Link Partner register */
#define	ANER	0x006a	/* Auto Negotiation Expansion register */
#define	DIS	0x006c	/* Disconnect counter */
#define	FCSC	0x006e	/* False Carrier Sense counter */
#define	NWAYTR	0x0070	/* N-Way test register */
#define	REC	0x0072	/* RX_ER register */
#define	CSCR	0x0074	/* CS Configuration register */
#define	PHY1_PARM 0x0078 /* PHY parameter 1 */
#define	TW_PARM	0x007c	/* Twister parameter */
#define	PHY2_PARM 0x0080 /* PHY parameter 2 */
#define MII8130	0x0082	/* MII interface (8130) */
#define	CONFIG5	0x00d8	/* Configuration register 5 */

/*
 * Bit field definitions
 */
/* Receive status register in Rx packet field */
#define	RSR_MAR		0x8000	/* Multicast address received */
#define	RSR_PAM		0x4000	/* Physical address Matched */
#define	RSR_BAR		0x2000	/* Broadcast address Received */
#define	RSR_ISE		0x0020	/* Symbol error */
#define	RSR_RUNT	0x0010	/* Runt packet received */
#define	RSR_LONG	0x0008	/* Long packet */
#define	RSR_CRC		0x0004	/* CRC error */
#define	RSR_FAE		0x0002	/* Frame alignment error */
#define	RSR_ROK		0x0001	/* Receive OK */

#define	RSR_ERRS	(RSR_ISE | RSR_RUNT | RSR_LONG | RSR_CRC | RSR_FAE)
#define	RSR_BITS	\
	"\020"		\
	"\020MAR"	\
	"\017PAM"	\
	"\016BAR"	\
	"\006ISE"	\
	"\005RUNT"	\
	"\004LONG"	\
	"\003CRC"	\
	"\002FAE"	\
	"\001ROK"

/* Transmit Status register (TSD0-3) */
#define	NTXDESC	4

#define	TSR_CRS		0x80000000	/* Carrier Sense Lost */
#define	TSR_TABT	0x40000000	/* Transmit Abort */
#define	TSR_OWC		0x20000000	/* Out of Window Collision */
#define	TSR_CDH		0x10000000	/* CD Heart beat */
#define	TSR_NCC		0x0f000000	/* Number of Collision count */
#define	TSR_NCC_SHIFT		24
#define	TSR_ERTXTH_MASK	0x003f0000	/* Early Tx threshold */
#define	TSR_ERTXTH_SHIFT	16
#define	TSR_ERTXTH_UNIT		32
#define	TSR_ERTXTH_SF	TSR_ERTXTH_MASK
#define	TSR_ERTXTH(x)	\
		((((x)/TSR_ERTXTH_UNIT)<<TSR_ERTXTH_SHIFT) & TSR_ERTXTH_MASK)
#define	TSR_TOK		0x00008000	/* Transmit OK */
#define	TSR_TUN		0x00004000	/* Transmit FIFO Underrun */
#define	TSR_OWN		0x00002000	/* Tx DMA operation done */
#define	TSR_SIZE	0x00001fff	/* Descriptor size */

#define	TX_STATUS_BITS	\
	"\020"		\
	"\040CRS"	\
	"\037TABT"	\
	"\036OWC"	\
	"\035CDH"	\
	"\020TOK"	\
	"\017TUN"	\
	"\016OWN"

/* ERSR */
#define	ERSR_ERGood	0x08		/* Early Rx Good packet */
#define	ERSR_ERBad	0x04		/* Early Rx Bad packet */
#define	ERSR_EROVW	0x02		/* Early Rx OverWrite */
#define	ERSR_EROK	0x01		/* Early Rx OK */

/* CR : Command register (uint8_t) */
#define	CR_RST		0x10		/* Reset */
#define	CR_RE		0x08		/* Receiver enable */
#define	CR_TE		0x04		/* Transmitter enable */
#define	CR_BUFE		0x01		/* Buffer Empty */

#define	CR_BITS	"\020\005RST\004RE\003TE\001BUFE"

/* IMR  & ISR */
#define	INTR_SERR	0x8000		/* System Error Interrupt */
#define	INTR_TIMEOUT	0x4000		/* Time out interrupt */
#define	INTR_LenChg	0x2000		/* Cable Length Change */
#define	INTR_FOVF	0x0040		/* Rx FIFO overflow */
#define	INTR_PUN	0x0020		/* Packet Underrun/Link change */
#define	INTR_RXOVW	0x0010		/* Rx buffer overflow */
#define	INTR_TER	0x0008		/* Transmit Error interrupt*/
#define	INTR_TOK	0x0004		/* Transmit OK interrupt*/
#define	INTR_RER	0x0002		/* Receive Error interrupt*/
#define	INTR_ROK	0x0001		/* Receive OK interrupt*/

#define	INTR_BITS	\
	"\020"	\
	"\020SERR"	\
	"\017TIMEOUT"	\
	"\016LenChg"	\
	"\007FOVF"	\
	"\006PUN"	\
	"\005RXOVF"	\
	"\004TER"	\
	"\003TOK"	\
	"\002RER"	\
	"\001ROK"

/* TCR: Transmit Configuration register */
#define	TCR_HWREV		0x7cc00000 /* Hardware revision ID */		
#define		HWREV_8139	0x40000000	
#define		HWREV_8139K	0x60000000	
#define		HWREV_8139A	0x70000000
#define		HWREV_8139A_G	0x70800000
#define		HWREV_8139B	0x78000000
#define		HWREV_8130	0x7c000000
#define		HWREV_8139C	0x74000000
#define		HWREV_8100	0x78800000
#define		HWREV_8100B	0x74400000
#define		HWREV_8139CP	0x74800000
#define		HWREV_8101L	0x74c00000
#define	TCR_IFG		0x03000000	/* Interframe Gap */
#define	TCR_IFG_SHIFT		24
#define	TCR_IFG_802_3		(3 << TCR_IFG_SHIFT)	/* 802.3 standard */
#define	TCR_LBK		0x00060000	/* Loopback test */
#define	TCR_LBK_SHIFT		17
#define	TCR_LBK_NORMAL		(0 << TCR_LBK_SHIFT)
#define	TCR_LBK_LOOPBACK	(3 << TCR_LBK_SHIFT)
#define	TCR_CRC		0x00010000	/* Inhibit Appending CRC */
#define	TCR_MXDMA	0x00000700
#define	TCR_MXDMA_SHIFT		8
#define	TCR_MXDMA_BASE		16
#define	TCR_MXDMA_16		(0 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_32		(1 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_64		(2 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_128		(3 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_256		(4 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_512		(5 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_1024		(6 << TCR_MXDMA_SHIFT)
#define	TCR_MXDMA_2048		(7 << TCR_MXDMA_SHIFT)

#define	TCR_TXRR	0x000000f0	/* Tx retry count */
#define	TCR_TXRR_SHIFT		4
#define	TCR_CLRABT	0x00000001	/* Retrans the packet aborded */
	
/* Receive Configuration register */


/* Bits in RxConfig. */

/* rx_mode_bits */
#define	RCR_ERTH	0x0f000000
#define	RCR_ERTH_SHIFT	24
#define	RCR_ERTH_NONE	(0 << RCR_ERTH_SHIFT)

#define	RCR_MulERINT	0x00020000
#define	RCR_RER8	0x00010000
#define	RCR_RXFTH	0x0000e000	/* Receive FIFO threshold */
#define	RCR_RXFTH_SHIFT	13	
#define	RCR_RXFTH_16		(0 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_32		(1 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_64		(2 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_128		(3 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_256		(4 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_512		(5 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_1024		(6 << RCR_RXFTH_SHIFT)
#define	RCR_RXFTH_NONE		(7 << RCR_RXFTH_SHIFT)

#define	RCR_RBLEN_SHIFT	11
#define	RBLEN_8K	0
#define	RBLEN_16K	1
#define	RBLEN_32K	2
#define	RBLEN_64K	3
#define	RBLEN(i)	(8192 << (i))
#define	RBLEN_PAD	16

#define	RCR_MXDMA	0x00000700	/* */
#define	RCR_MXDMA_SHIFT	8
#define	RCR_MXDMA_16		(0 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_32		(1 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_64		(2 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_128		(3 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_256		(4 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_512		(5 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_1024		(6 << RCR_MXDMA_SHIFT)
#define	RCR_MXDMA_UNLIMIT	(7 << RCR_MXDMA_SHIFT)
#define	RCR_WRAP	0x00000080	/* */
#define	RCR_9356SEL	0x00000040	/* EEPROM is 9356 */
#define	RCR_AER		0x00000020	/* Accept Error packet */
#define	RCR_AR		0x00000010	/* Accept runt */
#define	RCR_AB		0x00000008	/* Accept broadcast */
#define	RCR_AM		0x00000004	/* Accept Multicast */
#define	RCR_APM		0x00000002	/* Accept physical match */
#define	RCR_AAP		0x00000001	/* Accept all physical */

#define	RCR_ACCEPT_MODE		\
	(RCR_AER | RCR_AR | RCR_AB | RCR_AM | RCR_APM | RCR_AAP)

#define	RCR_BITS	\
	"\020\006AER\005AR\004AB\003AM\002APM\001AAP"

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

/* Config 0 */
/* Config 1 */
#define	CONFIG1_LEDS	0xc0
#define	CONFIG1_DVRLOAD	0x20
#define	CONFIG1_LWACT	0x10
#define	CONFIG1_MEMMAP	0x08
#define	CONFIG1_IOMAP	0x04
#define	CONFIG1_VPD	0x02
#define	CONFIG1_PMEn	0x01

#define CONFIG1_FULL	0x40	/* 8129 only */
#define CONFIG1_SLEEP	0x02    /* 8129 and 8139 only */
#define CONFIG1_PWRDWN	0x01    /* 8129 and 8139 only */

/* MSR : Media Status register */
#define	MSR_TXFCE	0x80	/* Tx Flow control enable */
#define	MSR_RXFCE	0x40	/* Rx Flow control enable */
#define	MSR_AUX_STATUS	0x10	/* Aux. power present status */
#define	MSR_SPEED_10	0x08	/* 10Mbps mode */
#define	MSR_LINKB	0x04	/* Inverse of link status */
#define	MSR_TXPF	0x02	/* 8139 sends pause packet */
#define	MSR_RXPF	0x01

#define	MSR_BITS	\
	"\020"		\
	"\010TXFCE"	\
	"\007RXFCE"	\
	"\005AUX_STATUS"	\
	"\004SPEED_10"	\
	"\003LINKB"	\
	"\002TXPF"	\
	"\001RXPF"

/* Config 3 */
#define	CONFIG3_GNTSel	0x80	/* Gnt Select 0:No delay 1:delay one clock R */
#define	CONFIG3_PARM_E	0x40	/* Parameter enable */
#define	CONFIG3_Magic	0x20	/* Magic packet */
#define	CONFIG3_LinkUp	0x10
#define	CONFIG3_CardB_En 0x08
#define	CONFIG3_CLKRUN_En 0x04
#define	CONFIG3_FuncRegEn_En 0x02
#define	CONFIG3_FBtBEn 0x01

/* Configration register 4 (0x5a, uint8_t) 8139 only */
#define	CONFIG4_RxFIFOAutoClr	0x80
#define	CONFIG4_LWPTN		0x04	/* not on 8139, 8139A */

/* MII interface (0x5a, uint8_t) for 8129, (0x82, uint8_t) for 8130 */
#define MII_MDM	0x80
#define MII_MDO	0x04
#define MII_MDO_SHIFT	2
#define MII_MDI	0x02
#define MII_MDI_SHIFT	1
#define MII_MDC	0x01

/* BMCR (almost same with MII_CONTROL register) */
#define	BMCR_RESET	0x8000	/* PHY reset */
#define	BMCR_Spd_Set	0x2000	/* 100Mbps */
#define	BMCR_ANE	0x1000	/* auto negotiation enable */
#define	BMCR_RSA	0x0200	/* restart auto negotiation */
#define	BMCR_duplex	0x0100	/* 100Mbps */

/* Basic mode status register */
/* Auto-negotiation Advertisement register */
/* Auto-negotiation Link Partner Ability register */
/* Auto-negotiation Expansion register */
/* Config5 */

/*
 * Offset to EPROM contents
 */
#define	EPROM_EthernetID	0x0e
