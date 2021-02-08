/*
 * DP83820 register definition
 * Based on National Semiconductor DP83820 datasheet.
 * Coded by Masayuki Murayama
 * This file is public domain.
 */

#pragma	ident	"%W% %E%"

/*
 * Tx/Rx descriptor
 */
struct nsg_desc32 {
	volatile uint32_t	d_link;		/* link to the next */
	volatile uint32_t	d_bufptr;	/* ptr to the first fragment */
	volatile uint32_t	d_cmdsts;	/* command/status field */
	volatile uint32_t	d_extsts;	/* extented status */
};

struct nsg_desc64 {
	volatile uint32_t	d_link_lo;	/* link to the next */
	volatile uint32_t	d_link_hi;	/* link to the next */
	volatile uint32_t	d_bufptr_lo;	/* ptr to the first fragment */
	volatile uint32_t	d_bufptr_hi;	/* ptr to the first fragment */

	volatile uint32_t	d_cmdsts;	/* command/status field */
	volatile uint32_t	d_extsts;	/* extented status */
};

/* CMDSTS common Bit Definition */
#define	CMDSTS_OWN	0x80000000U	/* 1: data consumer owns */
#define	CMDSTS_MORE	0x40000000U	/* Not the last descriptor */
#define	CMDSTS_INTR	0x20000000U
#define	CMDSTS_SUPCRC	0x10000000U
#define	CMDSTS_INCCRC	CMDSTS_SUPCRC
#define	CMDSTS_OK	0x08000000U	/* Packet is OK */
#define	CMDSTS_SIZE	0x0000ffffU	/* Descriptor byte count */

/* Transmit Status Bit Definition */
#define	CMDSTS_TXA	0x04000000U	/* Transmit abort */
#define	CMDSTS_TFU	0x02000000U	/* Transmit FIFO Underrun */
#define	CMDSTS_CRS	0x01000000U	/* Carrier sense lost */
#define	CMDSTS_TD	0x00800000U	/* Transmit deferred */
#define	CMDSTS_ED	0x00400000U	/* Exessive deferrral */
#define	CMDSTS_OWC	0x00200000U	/* Out of window collision */
#define	CMDSTS_EC	0x00100000U	/* Excessive collision */
#define	CMDSTS_CCNT	0x000f0000U	/* Collision count */
#define	CMDSTS_CCNT_SHIFT	(16)
#define		CCNT_MASK	0xfU	/* Collision count mask */

#define	TXSTAT_BITS	\
	"\020"		\
	"\040Own"	\
	"\037More"	\
	"\036Intr"	\
	"\035SupCrc"	\
	"\034Ok"	\
	"\033Abort"	\
	"\032UnderRun"	\
	"\031NoCarrier"	\
	"\030Deferd"	\
	"\027ExcDefer"	\
	"\026OWColl"	\
	"\025ExcColl"

/* Receive Status Bit Definitions */
#define	CMDSTS_RXA	0x04000000U	/* Receive abort */
#define	CMDSTS_RXO	0x02000000U	/* Receive overrun */
#define	CMDSTS_DEST	0x01800000U	/* Destination class */
#define	CMDSTS_DEST_SHIFT	23	/* Destination class */
#define		DEST_REJECT	0U
#define		DEST_NODE	1U
#define		DEST_MULTI	2U
#define		DEST_BROAD	3U
#define	CMDSTS_LONG	0x00400000U	/* Too long packet received */
#define	CMDSTS_RUNT	0x00200000U	/* Runt packet received */
#define	CMDSTS_ISE	0x00100000U	/* Invalid symbol error */
#define	CMDSTS_CRCE	0x00080000U	/* CRC error */
#define	CMDSTS_FAE	0x00040000U	/* Frame alignment */
#define	CMDSTS_LBP	0x00020000U	/* Loopback packet */
#define	CMDSTS_IRL	0x00010000U	/* In-Range Length Error */

#define	RXSTAT_BITS	\
	"\020"		\
	"\040Own"	\
	"\037More"	\
	"\036Intr"	\
	"\035IncCrc"	\
	"\034Ok"	\
	"\032OverRun"	\
	"\031MCast"	\
	"\030UniMatch"	\
	"\027TooLong"	\
	"\026Runt"	\
	"\025RxISErr"	\
	"\024CrcErr"	\
	"\023FaErr"	\
	"\022LoopBk"	\
	"\021RxCol"

#define	EXTSTS_UDPERR	0x00400000U	/* rx only */
#define	EXTSTS_UDPPKT	0x00200000U
#define	EXTSTS_TCPERR	0x00100000U	/* rx only */
#define	EXTSTS_TCPPKT	0x00080000U
#define	EXTSTS_IPERR	0x00040000U	/* rx only */
#define	EXTSTS_IPPKT	0x00020000U
#define	EXTSTS_VPKT	0x00010000U
#define	EXTSTS_VTCO	0x0000ffffU

#define	EXTSTS_BITS	\
	"\020"		\
	"\027UDPErr"	\
	"\026UDPPkt"	\
	"\025TCPErr"	\
	"\024TCPPkt"	\
	"\023IPErr"	\
	"\022IPPkt"	\
	"\021VPkt"

/*
 * Offsets of MAC Operational Registers
 */
#define	CR		0x00		/* Command register */
#define	CFG		0x04		/* Configuration register */
#define	MEAR		0x08		/* EEPROM access register */
#define	PTSCR		0x0c		/* PCI test control register */
#define	ISR		0x10		/* Interrupt status register */
#define	IMR		0x14		/* Interrupt mask register */
#define	IER		0x18		/* Interrupt enable register */
#define	IHR		0x1c		/* Interrupt hold register */
#define	TXDP		0x20		/* Transmit descriptor pointer reg */
#define	TXDP_HI		0x24		/* Transmit descriptor pointer reg */
#define	TXCFG		0x28		/* Transmit configuration register */
#define	GPIOR		0x2c		/* General purpose I/O control reg */
#define	RXDP		0x30		/* Receive descriptor pointer reg */
#define	RXDP_HI		0x34		/* Receive descriptor pointer reg */
#define	RXCFG		0x38		/* Receive configration register */
#define	PQCR		0x3c		/* Priority queueing control reg */
#define	WCSR		0x40		/* Wake-on-lan control register */
#define	PCR		0x44		/* Pause control/status register */
#define	RFCR		0x48		/* Receive filter control register */
#define	RFDR		0x4c		/* Receive filter data register */
#define	BRAR		0x50		/* Boot rom address */
#define	BRDR		0x54		/* Boot rom data */
#define	SRR		0x58		/* silicon revision register */
#define	MIBC		0x5c		/* MIB control register */
#define	MII_DATA_BASE	0x60		/* MIB data registers */
#define	TXDP1		0xa0		/* Tx Desc pointer priority 1 reg */
#define	TXDP2		0xa4		/* Tx Desc pointer priority 2 reg */
#define	TXDP3		0xa8		/* Tx Desc pointer priority 3 reg */
#define	RXDP1		0xb0		/* Rx Desc pointer priority 1 reg */
#define	RXDP2		0xb4		/* Rx Desc pointer priority 2 reg */
#define	RXDP3		0xb8		/* Rx Desc pointer priority 3 reg */
#define	VRCR		0xbc		/* VLAN/IP rx control reg */
#define	VTCR		0xc0		/* VLAN/IP tx control reg */
#define	VTDR		0xc4		/* VLAN data reg */
#define	CCSR		0xcc		/* Clockrun control/status reg */
#define	TBICR		0xe0		/* TBI control register */
#define	TBISR		0xe4		/* TBI status register */
#define	TBIAR		0xe8		/* TBI AN advertisement register */
#define	TBILPAR		0xec		/* TBI AN link partner ability reg */
#define	TBIER		0xf0		/* TBI AN expansion reg */
#define	TBIESR		0xf4		/* TBI AN extended status reg */


/* Command register, offset 0x00*/
#define	CR_RELOAD	0x0001e000U	/* rx priority queue select */
#define	CR_RXPRI_BIT(n)	(1U << ((n)+13))
#define	CR_TXPRI	0x00001e00U	/* tx priority queue select */
#define	CR_TXPRI_BIT(n)	(1U << ((n)+9))
#define	CR_RST		0x00000100U	/* Reset */
#define	CR_SWI		0x00000080U	/* Software interrupt */
#define	CR_RXR		0x00000020U	/* Receiver reset */
#define	CR_TXR		0x00000010U	/* Transmit reset */
#define	CR_RXD		0x00000008U	/* Receiver disable */
#define	CR_RXE		0x00000004U	/* Receiver enable */
#define	CR_TXD		0x00000002U	/* Transmit disable */
#define	CR_TXE		0x00000001U	/* Transmit enable */

#define	CR_BITS	\
	"\020"		\
	"\011Reset"	\
	"\010SWI"	\
	"\006RxReset"	\
	"\005TxReset"	\
	"\004RxDisable"	\
	"\003RxEnable"	\
	"\002TxDisable"	\
	"\001TxEnable"

/* Configration register, offset 0x04 */
#define	CFG_LNKSTS	0x80000000U	/* Link up */
#define	CFG_SPDSTS	0x60000000U	/* */
#define	CFG_DUPSTS	0x10000000U	/* */
#define	CFG_TBI_EN	0x01000000U	/* Enable Ten-Bit I/F */
#define	CFG_MODE_1000	0x00400000U	/* Enable 1000Mbps mode */
#define	CFG_PINT_CTL	0x001c0000U	/* PHY interrupt control */
#define	CFG_TMRTEST	0x00020000U	/* Speeds up 100uS timer to 4uS */
#define	CFG_MRM_DIS	0x00010000U	/* disable memry read multiple */
#define	CFG_MWI_DIS	0x00008000U	/* disable memry write invalidate */
#define	CFG_T64ADDR	0x00004000U	/* Enable target 64bit address (ro) */
#define	CFG_PCI64_DET	0x00002000U	/* PCI 64bit bus detected (ro) */
#define	CFG_DATA64_EN	0x00001000U	/* Enable 64bit data transter (rw) */
#define	CFG_M64ADDR	0x00000800U	/* Enable master 64bit address (rw) */
#define	CFG_PHY_RST	0x00000400U	/* external PHY reset */
#define	CFG_PHY_DIS	0x00000200U	/* external PHY disable */
#define	CFG_EXTSTS_EN	0x00000100U	/* Enable extended status */
#define	CFG_REQALG	0x00000080U	/* PCI Bus request algorithm */
#define	CFG_SB		0x00000040U	/* Single backoff */
#define	CFG_POW		0x00000020U	/* Program out of window timer */
#define	CFG_EXD		0x00000010U	/* Excessive deferral timer disable */
#define	CFG_PESEL	0x00000008U	/* Parity error detection action */
#define	CFG_BROM_DIS	0x00000004U	/* BootRom disable */
#define	CFG_EXT_125	0x00000002U	/* select external 125MHz reference */
#define	CFG_BEM		0x00000001U	/* Big endian mode */

#define	CFG_BITS	\
	"\020"	\
	"\040CFG_LNKSTS"	\
	"\031TBI_EN"	\
	"\027MODE_1000"	\
	"\022TMRTEST"	\
	"\021MRM_DIS"	\
	"\020MWI_DIS"	\
	"\017T64ADDR"	\
	"\016PCI64_DET"	\
	"\015DATA64_EN"	\
	"\014M64ADDR"	\
	"\013PHY_RST"	\
	"\012PHY_DIS"	\
	"\011EXTSTS_EN"	\
	"\010REQALG"	\
	"\007SB"	\
	"\006POW"	\
	"\005EXD"	\
	"\004PESEL"	\
	"\003BROM_DIS"	\
	"\002EXT_125"	\
	"\001BEM"

/* MEAR: EEPROM access register, offset 0x08 */
#define	MEAR_MDC	0x00000040U
#define	MEAR_MDDIR	0x00000020U
#define	MEAR_MDIO	0x00000010U
#define	MEAR_MDIO_SHIFT	4
#define	MEAR_EESEL	0x00000008U
#define	MEAR_EECLK	0x00000004U
#define	MEAR_EEDO	0x00000002U
#define	MEAR_EEDO_SHIFT	1
#define	MEAR_EEDI	0x00000001U
#define	MEAR_EEDI_SHIFT	0

/* PCI Test Control register */
#define	DISCARD_TEST	0x40000000U	/* Discard timer test mode */

/* Interrupt status register */
#define	ISR_TXDESC3	0x40000000U
#define	ISR_TXDESC2	0x20000000U
#define	ISR_TXDESC1	0x10000000U
#define	ISR_TXDESC0	0x08000000U
#define	ISR_RXDESC3	0x04000000U
#define	ISR_RXDESC2	0x02000000U
#define	ISR_RXDESC1	0x01000000U
#define	ISR_RXDESC0	0x00800000U
#define	ISR_TXRCMP	0x00400000U
#define	ISR_RXRCMP	0x00200000U
#define	ISR_DPERR	0x00100000U	/* Detected parity error */
#define	ISR_SSERR	0x00080000U	/* Signaled system error */
#define	ISR_RMABT	0x00040000U	/* received master abort */
#define	ISR_RTABT	0x00020000U	/* Rx target abort */
#define	ISR_RXSOVR	0x00010000U	/* Rx status FIFO overrun */
#define	ISR_HIBINT	0x00008000U
#define	ISR_PHY		0x00004000U
#define	ISR_PME		0x00002000U
#define	ISR_SWI		0x00001000U
#define	ISR_MIB		0x00000800U
#define	ISR_TXURN	0x00000400U
#define	ISR_TXIDLE	0x00000200U
#define	ISR_TXERR	0x00000100U
#define	ISR_TXDESC	0x00000080U
#define	ISR_TXOK	0x00000040U
#define	ISR_RXORN	0x00000020U
#define	ISR_RXIDLE	0x00000010U
#define	ISR_RXEARLY	0x00000008U
#define	ISR_RXERR	0x00000004U
#define	ISR_RXDESC	0x00000002U
#define	ISR_RXOK	0x00000001U

#define	INTR_BITS	\
	"\020"		\
	"\037TxDesc3"	\
	"\036TxDesc2"	\
	"\035TxDesc1"	\
	"\034TxDesc0"	\
	"\033RxDesc3"	\
	"\032RxDesc2"	\
	"\031RxDesc1"	\
	"\030RxDesc0"	\
	"\027TXRCMP"	\
	"\026RXRCMP"	\
	"\025DPErr"	\
	"\024SSErr"	\
	"\023RMAbt"	\
	"\022RTAbt"	\
	"\021RxSOVR"	\
	"\020HIBErr"	\
	"\017PHY"	\
	"\016PME"	\
	"\015SWI"	\
	"\014MIB"	\
	"\013TxUrn"	\
	"\012TxIdle"	\
	"\011TxErr"	\
	"\010TxDesc"	\
	"\007TxOk"	\
	"\006RxORN"	\
	"\005RxIdle"	\
	"\004RxEarly"	\
	"\003RxErr"	\
	"\002RxDesc"	\
	"\001RxOk"


/* Interrupt enable reigster, offset 0x18 */
#define	IER_IE		0x00000001U	/* Interrupt enable */

/* Interrupt hold reigster, offset 0x1c */
#define	IH_IHCTL	0x00000100U	/* enable hold off timer */
#define	IH_IH		0x000000ffU	/* hold off timer in 100uS */

/* Transmit configuration register, 0x28 */
#define	TXCFG_CSI	0x80000000U	/* carrier sense ignore */
#define	TXCFG_HBI	0x40000000U	/* heart beat ignore */
#define	TXCFG_MLB	0x20000000U	/* MAC loop back */
#define	TXCFG_ATP	0x10000000U	/* Automatic transmit padding */
#define	TXCFG_ECRETRY	0x00800000U	/* excessive collision retry */
#define	TXCFG_MXDMA	0x00700000U	/* max dma burst size */
#define		TXCFG_MXDMA_SHIFT	20
#define		TXCFG_MXDMA_1024	(0U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_8		(1U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_16		(2U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_32		(3U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_64		(4U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_128		(5U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_256		(6U << TXCFG_MXDMA_SHIFT)
#define		TXCFG_MXDMA_512		(7U << TXCFG_MXDMA_SHIFT)
#define	TXCFG_BRST_DIS	0x00080000U	/* disable burst for tx1000hdx */
#define	TXCFG_FLTH	0x0000ff00U	/* Tx fill threshold */
#define		TXCFG_FLTH_SHIFT	8
#define		TXCFG_FLTH_UNIT		32
#define	TXCFG_DRTH	0x000000ffU	/* Tx drain threshold */
#define		TXCFG_DRTH_UNIT		32

#define	TXFIFOSIZE	(8*1024)

#define	TXCFG_BITS	"\020\040CSI\037HBI\036MLB\035ATP"


/* General purpose I/O control register, 0x2c */
#define	GP5_IN		0x00004000U
#define	GP4_IN		0x00002000U
#define	GP3_IN		0x00001000U
#define	GP2_IN		0x00000800U
#define	GP1_IN		0x00000400U
#define	GP5_OE		0x00000200U
#define	GP4_OE		0x00000100U
#define	GP3_OE		0x00000080U
#define	GP2_OE		0x00000040U
#define	GP1_OE		0x00000020U
#define	GP5_OUT		0x00000010U
#define	GP4_OUT		0x00000008U
#define	GP3_OUT		0x00000004U
#define	GP2_OUT		0x00000002U
#define	GP1_OUT		0x00000001U

/* Reveive configuration register, offset 0x38 */
#define	RXCFG_AEP		0x80000000U	/* accept error packets */
#define	RXCFG_ARP		0x40000000U	/* accept runt packets */
#define	RXCFG_STRIPCRC		0x20000000U	/* strip crc:1 */
#define	RXCFG_RX_FD		0x10000000U	/* receive tx packets */
#define	RXCFG_ALP		0x08000000U	/* accept long packets */
#define	RXCFG_AIRL		0x04000000U	/* accept in-range len err */
#define	RXCFG_MXDMA		0x00700000U	/* max dma burst size */
#define		RXCFG_MXDMA_SHIFT	(20)
#define		RXCFG_MXDMA_1024	(0U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_8	(1U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_16	(2U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_32	(3U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_64	(4U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_128	(5U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_256	(6U << RXCFG_MXDMA_SHIFT)
#define		RXCFG_MXDMA_512	(7U << RXCFG_MXDMA_SHIFT)
#define	RXCFG_DRTH		0x0000003eU	/* Rx drain threshold */
#define		RXCFG_DRTH_SHIFT	(1)
#define		RXCFG_DRTH_UNIT		(8)

#define	RXFIFOSIZE	(32 * 1024)

#define	RXCFG_BITS	"\020\040AEP\037ARP\035ATX\034AJAB"


/* PQCR Priority Queueing Control register, offset 0x3c */
#define	PQCR_RXPQ	0x000000e0U	/* number of queues */
#define		PQCR_RXPQ_SHIFT	2
#define		PQCR_RXPQ_1	(0 << PQCR__RXPQ_SHIFT)
#define		PQCR_RXPQ_2	(1 << PQCR__RXPQ_SHIFT)
#define		PQCR_RXPQ_3	(2 << PQCR__RXPQ_SHIFT)
#define		PQCR_RXPQ_4	(3 << PQCR__RXPQ_SHIFT)
#define	PQCR_TXFAIR	0x00000002U	/* Enable Tx fairness */
#define	PQCR_TXPQEN	0x00000001U	/* Enable Tx priority queueing */

/* WCSR Wake command/status register, offset 0x40 */
#define	WCSR_MPR	0x80000000U
#define	WCSR_PATM3	0x40000000U
#define	WCSR_PATM2	0x20000000U
#define	WCSR_PATM1	0x10000000U
#define	WCSR_PATM0	0x08000000U
#define	WCSR_ARPR	0x04000000U
#define	WCSR_BCASTR	0x02000000U
#define	WCSR_MCASTR	0x01000000U
#define	WCSR_UCASTR	0x00800000U
#define	WCSR_PHYINT	0x00400000U
#define	WCSR_SOHACK	0x00200000U
#define	WCSR_MPSOE	0x00000400U
#define	WCSR_WKMAG	0x00000200U
#define	WCSR_WKPAT3	0x00000100U
#define	WCSR_WKPAT2	0x00000080U
#define	WCSR_WKPAT1	0x00000040U
#define	WCSR_WKPAT0	0x00000020U
#define	WCSR_WKARP	0x00000010U	/* wake on arp */
#define	WCSR_WKBCP	0x00000008U	/* wake on broadcast */
#define	WCSR_WKMCP	0x00000004U	/* wake on multicast */
#define	WCSR_WKUCP	0x00000002U	/* wake on unicast */
#define	WCSR_WKPHY	0x00000001U	/* wake on PHY interrupt*/

/* PCR: Pause control/status register, offset 0x44 */
#define	PCR_PSEN		0x80000000U	/* Pause Enable */
#define	PCR_PS_MCAST		0x40000000U	/* Pause on multicast */
#define	PCR_PS_DA		0x20000000U	/* Pause on DA */
#define	PCR_PS_ACT		0x10000000U	/* Pause active */
#define	PCR_PS_RCVD		0x08000000U	/* Pause frame receved */
#define	PCR_PS_STHI		0x03000000U	/* */
#define		PCR_PS_STHI_SHIFT		24
#define		PCR_PS_STHI_DIS		(0U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STHI_2		(1U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STHI_4		(2U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STHI_8		(3U << PCR_PS_STHI_SHIFT)
#define	PCR_PS_STLO		0x00c00000U	/* */
#define		PCR_PS_STLO_SHIFT		22
#define		PCR_PS_STLO_DIS		(0U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STLO_2		(1U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STLO_4		(2U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_STLO_8		(3U << PCR_PS_STHI_SHIFT)
#define	PCR_PS_FFHI		0x00300000U	/* */
#define		PCR_PS_FFHI_SHIFT		20
#define		PCR_PS_FFHI_DIS		(0U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFHI_2		(1U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFHI_4		(2U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFHI_8		(3U << PCR_PS_STHI_SHIFT)
#define	PCR_PS_FFLO		0x000c0000U	/* */
#define		PCR_PS_FFLO_SHIFT		18
#define		PCR_PS_FFLO_DIS		(0U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFLO_2		(1U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFLO_4		(2U << PCR_PS_STHI_SHIFT)
#define		PCR_PS_FFLO_8		(3U << PCR_PS_STHI_SHIFT)
#define	PCR_PS_TX		0x00020000U	/* Transmit pause frame */
#define	PCR_PAUSE_CNT		0x0000ffffU	/* Pause counter value */

#define	PCR_BITS	\
	"\020" \
	"\040PCR_PSEN" \
	"\037PCR_PS_MCAST" \
	"\036PCR_PS_DA" \
	"\035PCR_PS_ACT" \
	"\034PCR_PS_RCVD" \
	"\022PCR_PS_TX"

/* RFCR:0x48 Receive filter control register */
#define	RFCR_RFEN		0x80000000U	/* receive filter enable */
#define	RFCR_AAB		0x40000000U	/* accept all broadcast */
#define	RFCR_AAM		0x20000000U	/* accept all multicast */
#define	RFCR_AAU		0x10000000U	/* accept all unicast */
#define	RFCR_APM		0x08000000U	/* accept perfect match */
#define	RFCR_APAT		0x07800000U	/* accept on pattern match */
#define	RFCR_APAT_SHIFT		23		/* pattern match base */
#define	RFCR_AARP		0x00400000U	/* accept arp packets */
#define	RFCR_MHEN		0x00200000U	/* multicast hash enable */
#define	RFCR_UHEN		0x00100000U	/* unicast hash enable */
#define	RFCR_ULM		0x00080000U	/* U/L bit mask */
#define	RFCR_RFADDR		0x000003ffU
#define	RFCR_RFADDR_SHIFT	0

/* Receive filter offset */
#define	RFADDR_MAC		0x000
#define	RFADDR_PCOUNT01		0x006
#define	RFADDR_PCOUNT23		0x008
#define	RFADDR_MULTICAST	0x100
#define	RFADDR_PMATCH0		0x200
#define	RFADDR_PMATCH1		0x280
#define	RFADDR_PMATCH2		0x300
#define	RFADDR_PMATCH3		0x380

#define	MCAST_HASH_BITS_P2	11		/* 2^11 bits == 2048b bits */
#define	MCAST_HASH_BITS		(1 << MCAST_HASH_BITS_P2)

/* RFCR:0x4c Receive filter control register */
#define	RFDR_BMASK		0x00030000U
#define	RFDR_BMASK1		0x00020000U
#define	RFDR_BMASK0		0x00010000U

/* Receive filter data register */

/* SRR: 0x58 Silicon revision register */
#define	SRR_REV			0x0000ffffU
#define	SRR_REV_REV_B		0x0103U

/* MIBC: MIB control register, offset 0x5c */
#define	MIBC_MIBS	0x00000008U	/* MIB counter strobe */
#define	MIBC_ACLR	0x00000004U	/* clear all counters */
#define	MIBC_FRZ	0x00000002U	/* freeze */
#define	MIBC_WRN	0x00000001U	/* warning test indicator */

/* offsets of MIB data registers */
#define	MIB_RXErroredPkts	0x60
#define	MIB_RXFCSErrors		0x64
#define	MIB_RXMsdPktErrors	0x68
#define	MIB_RXFAErrors		0x6c
#define	MIB_RXSymbolErrors	0x70
#define	MIB_RXFrameTooLong	0x74
#define	MIB_RXIRLErrors		0x78
#define	MIB_RXBadOpcodes	0x7c
#define	MIB_RXPauseFrames	0x80
#define	MIB_TXPauseFrames	0x84
#define	MIB_TXSQEErrors		0x88

/* VRCR: VLAN/IP receive control register, offset 0xbc */
#define	VRCR_RUDPE	0x00000080U	/* reject UDP checksum errored pkts */
#define	VRCR_RTCPE	0x00000040U	/* reject TCP checksum errored pkts */
#define	VRCR_RIPE	0x00000020U	/* reject IP checksum errored pkts */
#define	VRCR_IPEN	0x00000010U	/* checksum enable */
#define	VRCR_DUTF	0x00000008U	/* dicard packets w/o a VLAN tag */
#define	VRCR_DVTF	0x00000004U	/* dicard packets w/ a VLAG tag */
#define	VRCR_VTREN	0x00000002U	/* Enable VLAN tag stripping */
#define	VRCR_VTDEN	0x00000001U	/* Enable VLAN tag detection */

/* VRCR: VLAN/IP transmit control register, offset 0xc0 */
#define	VTCR_PPCHK	0x00000008U	/* elable tcp/udp/ip on per pkt */
#define	VTCR_GCHK	0x00000004U	/* elable tcp/udp/ip checksum */
#define	VTCR_VPPTI	0x00000002U	/* Insert VLAN tag on per pkt basis */
#define	VTCR_VGTI	0x00000001U	/* Insert VLAN tag to all packets */

/* VDR: VLAN/IP transmit tag data register, offset 0xc0 */
#define	VDCR_VTCI	0xffff0000U
#define	VDCR_VTCI_SHIFT	16
#define	VDCR_VTYPE	0x0000ffffU

/* CCSR: Clock run Control status register, offset 0xcc */
#define	CCSR_PMESTS	0x00008000U
#define	CCSR_PMEEN	0x00000100U
#define	CCSR_CLKRUN_EN	0x00000001U

/* TBICR: TBI control register, offset 0xe0 */
#define	TBICR_MR_LOOPBACK	0x00004000U
#define	TBICR_AN_ENABLE		0x00001000U
#define	TBICR_RESTART_AN	0x00000200U

/* TBISR: TBI status register, offset 0xe4 */
#define	TBISR_LINK_STATUS	0x00000020U
#define	TBISR_AN_COMPLETE	0x00000004U

/* EEPROM address */
#define	EEPROM_PMATCH0	0x000c
#define	EEPROM_PMATCH1	0x000b
#define	EEPROM_PMATCH2	0x000a
