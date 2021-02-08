/*
 * sis190/191 register offsets
 *
 * Coded by Masayuki Murayama (KHF04453@nifty.ne.jp)
 * This file is public domain.
 */
#pragma	ident	"%Z%%M% %I%     %E%"

#ifndef	_SIS190REG_H_
#define	_SIS190REG_H_

#define	TxCTL		0x00	/* transmit control register */
#define	TxDESC		0x04	/* transmit descriptor base address */
#define	Reserved0	0x08
#define	TxSTAT		0x0c	/* transmit status register */
#define	RxCTL		0x10	/* receive control register */
#define	RxDESC		0x14	/* receive descriptor base address */
#define	Reserved1	0x18
#define	RxSTAT 		0x1c	/* receive status register */
#define	ISR		0x20	/* interrupt status register */
#define	IMR		0x24	/* interrupt mask register */
#define	ICR		0x28	/* interrupt control register */
#define	TIMER		0x2c	/* interval timer */
#define	PMCTL		0x30	/* power management control register */
#define	Reserved2	0x34
#define	ROMCTL		0x38	/* eeprom interface control register */
#define	ROMCMD		0x3c	/* eeprom interface command register */
#define	StationCTL 	0x40	/* station control register */
#define	GMIICTL 	0x44	/* GMII control register */
#define	GMacIOCR 	0x48	/* Gigabit mac command register */
#define	GMacIOCTL 	0x4c	/* Gigabit mac control register */
#define	TxMacCTL 	0x50	/* transmit mac control register */
#define	TxTimerLimit 	0x54	/* transmit timer control register */
#define	RGMIIDelay 	0x58	/* RGMII delay */
#define	Reserved3	0x5c
#define	RxMacCTL	0x60	/* rx mac control register */
#define	RxMacAddr	0x62	/* my station address */
#define	MCASTHASH	0x68	/* multicast hash base */
#define	RxWOL		0x70	/* WOL control register */
#define	RxWOLData	0x74	/* WOL pattern register */
#define	RxMPSCTL	0x78
#define	Reserved4	0x7c

struct sige_txdesc {
	volatile uint32_t	td_pktsize;
	volatile uint32_t	td_cmdsts;
	volatile uint32_t	td_bufptr;
	volatile uint32_t	td_flags;
};

/* pktsize field */
#define	TX_INS_VTAG	0x80000000U
#define	TX_MSS_SHIFT	16

/*
 * tx descriptor command bits
 */
#define	TDC_TXOWN	0x80000000U
#define	TDC_TXINT	0x40000000U
#define	TDC_THOL3	0x30000000U
#define	TDC_THOL2	0x20000000U
#define	TDC_THOL1	0x10000000U
#define	TDC_THOL0	0x00000000U
#define	TDC_LSEN	0x08000000U
#define	TDC_IPCS	0x04000000U
#define	TDC_TCPCS	0x02000000U
#define	TDC_UDPCS	0x01000000U
#define	TDC_BSTEN	0x00800000U
#define	TDC_EXTEN	0x00400000U
#define	TDC_DEFEN	0x00200000U
#define	TDC_BKFEN	0x00100000U
#define	TDC_CRSEN	0x00080000U
#define	TDC_COLEN	0x00040000U
#define	TDC_CRCEN	0x00020000U
#define	TDC_PADEN	0x00010000U
#define	TDC_VTAG	0x0000ffffU


#define	TDS_OWC		0x00080000U
#define	TDS_ABT		0x00040000U
#define	TDS_FIFO	0x00020000U
#define	TDS_CRS		0x00010000U
#define	TDS_COLLS	0x0000ffffU

#define	TDS_BITS	\
	"\020"	\
	"\040TXOWN"	\
	"\037TXINT"	\
	"\024OHS"	\
	"\023ABT"	\
	"\022FIFO"	\
	"\021CRS"

#define	TX_RING_EOD	0x80000000U

struct sige_rxdesc {
	volatile uint32_t	rd_stssize;
	volatile uint32_t	rd_pktinfo;
	volatile uint32_t	rd_bufptr;
	volatile uint32_t	rd_flags;
};

/*
 * rx descriptor status bits
 */
#define	RDS_TAGON	0x80000000U
#define	RDS_DESCS	0x3f000000U
#define		RDS_DESCS_SHIFT	24
#define	RDS_ABORT	0x00800000U
#define	RDS_SHORT	0x00400000U
#define	RDS_LIMIT	0x00200000U
#define	RDS_MIIER	0x00100000U
#define	RDS_OVRUN	0x00080000U
#define	RDS_NIBON	0x00040000U
#define	RDS_COLON	0x00020000U
#define	RDS_CRCOK	0x00010000U
#define	RDS_SIZE	0x0000ffffU

#define	RDS_BITS	\
	"\020"	\
	"\040TAGON"	\
	"\030ABORT"	\
	"\027SHORT"	\
	"\026LIMIT"	\
	"\025MIIER"	\
	"\024OVRUN"	\
	"\023NIBON"	\
	"\022COLON"	\
	"\021CRCOK"

/*
 * rx descriptor infomation bits
 */
#define	RDI_RXOWN	0x80000000U
#define	RDI_RXINT	0x40000000U
#define	RDI_IPON	0x20000000U
#define	RDI_TCPON	0x10000000U
#define	RDI_UDPON	0x08000000U
#define	RDI_IPOK	0x04000000U
#define	RDI_TCPOK	0x02000000U
#define	RDI_UDPOK	0x01000000U
#define	RDI_WAKUP	0x00400000U
#define	RDI_MAGIC	0x00200000U
#define	RDI_PAUSE	0x00100000U
#define	RDI_CAST	0x000c0000U
#define	RDI_CAST_SHIFT	18
#define	RDI_BCAST	(3U << RDI_CAST_SHIFT)
#define	RDI_MCAST	(2U << RDI_CAST_SHIFT)
#define	RDI_UCAST	(1U << RDI_CAST_SHIFT)
#define	RDI_CRCOFF	0x00020000U
#define	RDI_PREADD	0x00010000U
#define	RDI_VTAG	0x0000ffffU

#define	RDI_BITS	\
	"\020"	\
	"\040RXOWN"	\
	"\037RXINT"	\
	"\036IPON"	\
	"\035TCPON"	\
	"\034UDPON"	\
	"\027WAKUP"	\
	"\026MAGIC"	\
	"\025PAUSE"	\
	"\022CRCOFF"	\
	"\021PREADD"

#define	RX_RING_EOD	0x80000000U

/*
 * interrupt bits for IMR register and ISR registers
 */
#define	INT_SOFT	0x40000000U
#define	INT_TIMER	0x20000000U
#define	INT_PAUSEF	0x00080000U
#define	INT_MAGICP	0x00040000U
#define	INT_WAKEF	0x00020000U
#define	INT_LINK	0x00010000U
#define	INT_RXIDLE	0x00000080U
#define	INT_RXDONE	0x00000040U
#define	INT_TXIDLE	0x00000008U
#define	INT_TXDONE	0x00000004U
#define	INT_RXHALT	0x00000002U
#define	INT_TXHALT	0x00000001U
#define	INT_BITS	\
	"\020"	\
	"\037soft"	\
	"\036timer"	\
	"\024pausef"	\
	"\023magicp"	\
	"\022wakef"	\
	"\021link"	\
	"\010rxidle"	\
	"\007rxdone"	\
	"\004txidle"	\
	"\003txdone"	\
	"\002rxhalt"	\
	"\001txhalt"

/*
 * TxCTL register
 */
#define	TxCTL_POLL	0x00000010U
#define	TxCTL_EN	0x00000001U

/*
 * RxCTL register
 */
#define	RxCTL_POLL	0x00000010U
#define	RxCTL_EN	0x00000001U

/*
 * eeprom interface register
 */
#define	EI_DATA		0xffff0000U
#define		EI_DATA_SHIFT	16
#define	EI_OFFSET	0x0000fc00U
#define		EI_OFFSET_SHIFT	10
#define	EI_OP		0x00000300U
#define		EI_OP_SHIFT	8
#define		EI_OP_RD	(2U << EI_OP_SHIFT)
#define		EI_OP_WR	(1U << EI_OP_SHIFT)
#define	EI_REQ		0x00000080U
#define	EI_DO		0x00000008U	/* not used */
#define	EI_DI		0x00000004U	/* not used */
#define	EI_CLK		0x00000002U	/* not used */
#define	EI_CS		0x00000001U

/*
 * Station CTL register, offset 0x40
 */
#define	SC_LOOPBACK	0x80000000U
#define	SC_RGMII	0x00008000U	/* 1:RGMII, 0:GMII/MII */
#define	SC_FDX		0x00001000U	/* full duplex mode */
#define	SC_SPEED	0x00000c00U	/* speed */
#define		SC_SPEED_SHIFT		10
#define		SC_SPEED_1000		(3U << SC_SPEED_SHIFT)
#define		SC_SPEED_100		(2U << SC_SPEED_SHIFT)
#define		SC_SPEED_10		(1U << SC_SPEED_SHIFT)

/*
 * Gigabit Media Independent Interface CTL register
 */
#define	GMI_DATA	0xffff0000U
#define		GMI_DATA_SHIFT	16
#define	GMI_REG		0x0000f800U
#define		GMI_REG_SHIFT	11
#define	GMI_PHY		0x000007c0U
#define		GMI_PHY_SHIFT	6
#define	GMI_OP		0x00000020U
#define	GMI_OP_SHIFT		5
#define	GMI_OP_WR	(1 << GMI_OP_SHIFT)
#define	GMI_OP_RD	(0 << GMI_OP_SHIFT)
#define	GMI_REQ		0x00000010U
#define	GMI_MDIO	0x00000008U 	/* not used */
#define	GMI_MDDIR	0x00000004U	/* not used */
#define	GMI_MDC		0x00000002U	/* not used */
#define	GMI_MDEN	0x00000001U	/* not used */

/*
 * EEPROM Addresses
 */
#define	EEPROMSignature	0x00
#define	EEPROMCLK	0x01
#define	EEPROMInfo	0x02
#define	EEPROMMACAddr	0x03

/*
 * RxMacCTL
 */
#define	RM_AB		0x0800U		/* Accept broadcast packet */
#define	RM_AM		0x0400U		/* Accept multicast packet */
#define	RM_AP		0x0200U		/* Accept physical */
#define	RM_AAP		0x0100U		/* Accept all physical */
#define RM_STRIP_VLAN	0x0020U
#define RM_STRIP_FCS	0x0010U
#define RM_PAD_ENB	0x0004U
#define RM_PAD_LEN	10

#endif /* _SIS190REG_H_ */
