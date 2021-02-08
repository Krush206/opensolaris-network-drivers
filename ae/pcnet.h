/*
 * @(#)pcnet.h	1.5 11/02/20
 * AMD AM79C79x PCNET/PCI series register definition 
 * Based on AMD 79C790, 790A, 791 datasheets.
 * Coded by Masayuki Murayama.
 * This file is public domain.
 */
#ifndef _PCNET_H_
#define	_PCNET_H_
/*
 * I/O register offsets
 */

/* 16bit mode */
#define	APROM_W	0x00		/* address prom base */
#define	RDP_W	0x10		/* data port */
#define	RAP_W	0x12		/* address port */
#define	RST_W	0x14		/* reset register */
#define	BDP_W	0x16		/* bus config data port */

/* 32bit mode */
#define	APROM	0x00		/* address prom base */
#define	RDP	0x10		/* data port */
#define	RAP	0x14		/* address port */
#define	RST	0x18		/* reset register */
#define	BDP	0x1c		/* bus config data port */

/*
 * CSR registers, which are accessed by RAP and RDP
 */
#define	CSR0	0	/* Am79C87x Controller Status and Control Register */
#define	CSR1	1	/* Initialization Block Address 0 (maps to CSR16) */
#define	CSR2	2	/* Initializatioon Block Address 1 (maps to CSR17) */
#define	CSR3	3	/* Interrupt Masks and Deferrall Control */
#define	CSR4	4	/* Interrupt Masks, Configration and status */
#define	CSR5	5	/* Extended Interrupt, Configration and status bits */
#define	CSR7	7	/* Extended Interrupt, Configration and status bits */
#define	CSR8	8	/* Logical address filter base */
#define	CSR12	12	/* Physical address base */
#define	CSR15	15	/* mode */
#define	CSR47	47	/* Transmit Polling Interval */
#define	CSR49	49	/* Receive Polling Interval */
#define	CSR58	58	/* Software style (mapped to BCR20) */
#define	CSR76	76	/* Receive descriptor ring length */
#define	CSR78	78	/* Transmit descriptor ring length */
#define	CSR80	80	/* FIFO threshold and DMA burst control */
#define	CSR88	88	/* Chip ID (32bit) */
#define	CSR89	89	/* Chip ID (only msb 16bit) */
#define	CSR112	112	/* Missed Frame Count */
#define	CSR114	114	/* Receive Collision Count */
#define	CSR122	122	/* Receive Frame Alignment Control */
#define	CSR124	124	/* BMU Test register */
#define	CSR125	125	/* MAC Enhanced configuration control */

/* CSR0: controller status and command register */
#define	CSR0_ERR	0x8000U	/* BABL | CERR | MISS | MERR */
#define	CSR0_BABL	0x4000U	/* tx timeout error RWc */
#define	CSR0_CERR	0x2000U	/* Collision error RWc */
#define	CSR0_MISS	0x1000U	/* Missed frame caused by no rx desc RWc */
#define	CSR0_MERR	0x0800U	/* Memory error RWc */
#define	CSR0_RINT	0x0400U	/* rx int RWc */
#define	CSR0_TINT	0x0200U	/* tx int RWc */
#define	CSR0_IDON	0x0100U	/* initialization done RWc */
#define	CSR0_INTR	0x0080U	/* interrupt occured R- */
#define	CSR0_IENA	0x0040U	/* Interrupt Enable RW */
#define	CSR0_RXON	0x0020U	/* rx enabled R- */
#define	CSR0_TXON	0x0010U	/* tx enabled R- */
#define	CSR0_TDMD	0x0008U	/* send tx now -W */
#define	CSR0_STOP	0x0004U	/* stop controller -W */
#define	CSR0_STRT	0x0002U	/* enable controller -W */
#define	CSR0_INIT	0x0001U	/* initialize contoller -W */

#define CSR0_STAT	0x7f00U	/* status bits in CSR0 */

#define CSR0_ISR	\
	(CSR0_BABL | CSR0_MISS | CSR0_MERR | \
	 CSR0_RINT | CSR0_TINT | CSR0_IDON)

#define	CSR0_BITS	\
	"\020"	\
	"\020ERR"	\
	"\017BABL"	\
	"\016CERR"	\
	"\015MISS"	\
	"\014MERR"	\
	"\013RINT"	\
	"\012TINT"	\
	"\011IDON"	\
	"\010INTR"	\
	"\007IENA"	\
	"\006RXON"	\
	"\005TXON"	\
	"\004TDMD"	\
	"\003STOP"	\
	"\002STRT"	\
	"\001INIT"

/* CSR1: Initilization Block Address 0 */
/* CSR2: Initilization Block Address 1 */
/* CSR3: Interrupt Masks and Deferral Control */
#define	CSR3_BABLM	0x4000U	/* Babble Mask masked if set */
#define	CSR3_MISSM	0x1000U	/* Missed Frame Mask masked if set */
#define	CSR3_MERRM	0x0800U	/* Memory Error Mask masked if set */
#define	CSR3_RINTM	0x0400U	/* Rx intr Mask masked if set */
#define	CSR3_TINTM	0x0200U	/* Tx intr Mask masked if set */
#define	CSR3_IDONM	0x0100U	/* IDON Mask masked if set */
#define	CSR3_DXSUFLO	0x0040U	/* disable tx stop on underflow error (971-)*/
#define	CSR3_LAPPEN	0x0020U	/* Look Ahead packet processing enable */
#define	CSR3_DXMT2PD	0x0010U	/* disable tx two part deferral */
#define	CSR3_EMBA	0x0008U	/* Enable modified back-off algorithm */
#define	CSR3_BSWP	0x0004U	/* 1:Big Endian, 0:Little Endian */

#define	CSR3_BITS	\
	"\020"	\
	"\017BABLM"	\
	"\015MISSM"	\
	"\014MERRM"	\
	"\013RINTM"	\
	"\012TINTM"	\
	"\011IDONM"	\
	"\007DXSUFLO"	\
	"\006LAPPEN"	\
	"\005DXMT2PD"	\
	"\004EMBA"	\
	"\003BSWP"

/* CSR4: Test and Feaures control */
#define	CSR4_ENTST	0x8000U	/* Enable test (790) */
#define	CSR4_DMAPLUS	0x4000U	/* dma plus */
#define	CSR4_TIMER	0x2000U	/* timer enable (790) */
#define	CSR4_TXDPOLL	0x1000U	/* disable tx poll */
#define	CSR4_APAD_XMT	0x0800U	/* auto pad tx */
#define	CSR4_ASTRP_RCV	0x0400U	/* auto strip rx */
#define	CSR4_MFCO	0x0200U	/* missed frame counter overflow */
#define	CSR4_MFCOM	0x0100U	/* missed frame counter overflow masak*/
#define	CSR4_UINTCMD	0x0080U	/* user interrupt command */
#define	CSR4_UINT	0x0040U	/* user interrupt (ro) */
#define	CSR4_RCVCCO	0x0020U	/* rx collistion counter overflow (ro) */
#define	CSR4_RCVCCOM	0x0010U	/* rx collistion counter overflow mask */
#define	CSR4_TXSTRT	0x0008U	/* transmit start status (ro) */
#define	CSR4_TXSTRTM	0x0004U	/* transmit start status (ro) */
#define	CSR4_JAB	0x0002U	/* jabber error (ro) (790) */
#define	CSR4_JABM	0x0001U	/* jabber error mask (790) */

#define	CSR4_BITS	\
	"\020"	\
	"\020EnTST"	\
	"\017DMAPLUS"	\
	"\016TIMER"	\
	"\015TXDPOLL"	\
	"\014APAD_XMT"	\
	"\013ASTRP_RCV"	\
	"\012MFCO"	\
	"\011MFCOM"	\
	"\010UINTCMD"	\
	"\007UINT"	\
	"\006RCVCCO"	\
	"\005RCVCCOM"	\
	"\004TXSTRT"	\
	"\003TXSTRTM"	\
	"\002JAB"	\
	"\001JABM"

/* CSR5: extended control and interrupt 1 only for 790A or later */
#define	CSR5_TOKINTD	0x8000U	/* disable TOK interrupt */
#define	CSR5_LTINTEN	0x4000U	/* enable LTINT */
#define	CSR5_SINT	0x0800U	/* system interrupt */
#define	CSR5_SINTE	0x0400U	/* system interrupt enable */
#define	CSR5_SLPINT	0x0200U	/* sleep interrupt */
#define	CSR5_SLPINTE	0x0100U	/* sleep interrupt */
#define	CSR5_EXDINT	0x0080U	/* excesive deferral */
#define	CSR5_EXDINTE	0x0040U	/* excesive deferral interrupt */
#define	CSR5_MPPLBA	0x0020U	/* magic packet */
#define	CSR5_MPINT	0x0010U	/* magic packet */
#define	CSR5_MPINTE	0x0008U	/* magic packet */
#define	CSR5_MPEN	0x0004U	/* magic packet enabe */
#define	CSR5_MPMODE	0x0002U	/* magic packet mode */
#define	CSR5_SPND	0x0001U	/* suspend */

#define	CSR5_BITS	\
	"\020"	\
	"\020TOKINTD"	\
	"\017LTINTEN"	\
	"\014SINT"	\
	"\013SINTE"	\
	"\012SLPINT"	\
	"\011SLPINTE"	\
	"\010EXDINT"	\
	"\007EXDINTE"	\
	"\006MPPLBA"	\
	"\005MPINT"	\
	"\004MPINTE"	\
	"\003MPEN"	\
	"\002MPMODE"	\
	"\001SPND"

/* CSR7: extended control and interrupt 2 only for 790A or later */
#define	CSR7_FASTSPNDE	0x8000U	/* fast suspend enable */
#define	CSR7_RXFRTG	0x4000U	/* rx frame tag */
#define	CSR7_RDMD	0x2000U	/* rx demand */
#define	CSR7_RXDPOLL	0x1000U	/* rx disable polling */
#define	CSR7_STINT	0x0800U	/* software timer interrupt RO */
#define	CSR7_STINTE	0x0400U	/* software timer interrupt enable RW */
#define	CSR7_MREINT	0x0200U	/* MII read error intr RO */
#define	CSR7_MREINTE	0x0100U	/* MII read error intr enable RW */
#define	CSR7_MAPINT	0x0080U	/* MII autopoll interrupt  RO */
#define	CSR7_MAPINTE	0x0040U	/* MII autopoll interrupt enable RW */
#define	CSR7_MCCINT	0x0020U	/* MII cmd complete interrupt RO */
#define	CSR7_MCCINTE	0x0010U	/* MII cmd complete interrupt enable RW */
#define	CSR7_MCCIINT	0x0008U	/* MII cmd int'l complete interrupt RO */
#define	CSR7_MCCIINTE	0x0004U	/* MII cmd int'l complete interrupt enable RW */
#define	CSR7_MIIPDTINT	0x0002U	/* MII PHY detect transition interrupt RO */
#define	CSR7_MIIPDTINTE	0x0001U	/* MII PHY detect transition interrupt enable RW */

#define	CSR7_BITS	\
	"\020"	\
	"\020FASTSPNDE"	\
	"\017RXFRTG"	\
	"\016RDMD"	\
	"\015RXDPOLL"	\
	"\014STINT"	\
	"\013STINTE"	\
	"\012MREINT"	\
	"\011MREINTE"	\
	"\010MAPINT"	\
	"\007MAPINTE"	\
	"\006MCCINT"	\
	"\005MCCINTE"	\
	"\004MCCIINT"	\
	"\003MCCIINTE"	\
	"\002MIIPDTINT"	\
	"\001MIIPDTINTE"

/* CSR8-11: logical address filter */
/* CSR12-14: physical address */

/* CSR15 mode */
#define	CSR15_PROM	0x8000U	/* promiscuous mode */
#define	CSR15_DRCVBC	0x4000U	/* disable receive broadcast */
#define	CSR15_DRCVPA	0x2000U	/* disable receive physical */
#define	CSR15_DLNKTST	0x1000U	/* disable link status */
#define	CSR15_DAPC	0x0800U	/* disable automatic polarity correction */
#define	CSR15_MEMDEC	0x0400U	/* loop back mode */
#define	CSR15_LRT	0x0200U
#define	CSR15_PORTSEL	0x0180U
#define		CSR15_PORTSEL_MASK	0x3U
#define		CSR15_PORTSEL_SHIFT	7
#define		CSR15_PORTSEL_AUI	(0U << CSR15_PORTSEL_SHIFT)
#define		CSR15_PORTSEL_10BASET	(1U << CSR15_PORTSEL_SHIFT)
#define		CSR15_PORTSEL_GPSI	(2U << CSR15_PORTSEL_SHIFT)
#define		CSR15_PORTSEL_MII	(3U << CSR15_PORTSEL_SHIFT)
#define	CSR15_INTL	0x0040U	/* internal loopback */
#define	CSR15_DRTY	0x0020U	/* disable retry */
#define	CSR15_FCOLL	0x0010U	/* force collision */
#define	CSR15_DXMTFCS	0x0008U	/* disable tx CRC */
#define	CSR15_LOOP	0x0004U	/* loopback enable */
#define	CSR15_DTX	0x0002U	/* disable tx results */
#define	CSR15_DRX	0x0001U	/* disable rx results */

/* CSR46 receive poll time counter */
/* CSR47 receive polling interval */
/* CSR48 receive poll time counter */
/* CSR49 receive polling interval */

/* CSR58 (an alias of BCR20) */
#define	CSR58_APPERREN	0x0400U	/* advanced parity error handling enable */
#define	CSR58_CSRPCNET	0x0200U	/* csr3 and csr4 maps PCNET (ro) */
#define	CSR58_SSIZE32	0x0100U	/* 32bit software style (ro) */
#define	CSR58_SWSTYLE	0x00ffU	/* software stype (rw) */
#define		CSR58_SWSTYLE_LANCE	0U
#define		CSR58_SWSTYLE_ILLAC	1U	/* gone with 971 or later */
#define		CSR58_SWSTYLE_PCNETPCI	2U
#define		CSR58_SWSTYLE_PCNETPCI3	3U	/* ? */

/* CSR80 FIFO threshold and DMA burst control */
#define	CSR80_RCVFW	0x3000U
#define	CSR80_RCVFW_SHIFT	12	/* rx fifo threashold */
#define		CSR80_RCVFW_16	(0U << CSR80_RCVFW_SHIFT)
#define		CSR80_RCVFW_64	(1U << CSR80_RCVFW_SHIFT)
#define		CSR80_RCVFW_112	(2U << CSR80_RCVFW_SHIFT)
#define	CSR80_XMTSP	0x0c00U		/* tx fifo threashold for start*/
#define	CSR80_XMTSP_SHIFT	10
#define		CSR80_XMTSP_20	(0U << CSR80_XMTSP_SHIFT)
#define		CSR80_XMTSP_64	(1U << CSR80_XMTSP_SHIFT)
#define		CSR80_XMTSP_128	(2U << CSR80_XMTSP_SHIFT)
#define		CSR80_XMTSP_248	(3U << CSR80_XMTSP_SHIFT)
#define	CSR80_XMTFW	0x0300U		/* tx fifo threashold for start*/
#define	CSR80_XMTFW_SHIFT	8	/* tx fifo threashold for bus */
#define		CSR80_XMTFW_16	(0U << CSR80_XMTFW_SHIFT)
#define		CSR80_XMTFW_64	(1U << CSR80_XMTFW_SHIFT)
#define		CSR80_XMTFW_108	(2U << CSR80_XMTFW_SHIFT)
#define	CSR80_DMATC	0x00ffU		/* DMA burst length */

/* CSR88(lower) and CSR89(upper) */
#define	CSR88_VAR		0xf0000000U
#define		CSR88_VAR_MASK	0xfU
#define		CSR88_VAR_SHIFT	28
#define	CSR88_PARTID	0x0ffff000U
#define		CSR88_PARTID_MASK	0xffffU
#define		CSR88_PARTID_SHIFT	12
#define	CSR88_MANFID	0x00000ffeU
#define		CSR88_MANFID_MASK	0x7ffU
#define		CSR88_MANFID_SHIFT	1

/*
 * BCR registers
 */
#define	MC		2	/* miscellaneous configration */
#define	LED0		4	/* LED0 status */
#define	LED1		5	/* LED1 status */
#define	LED2		6	/* LED2 status */
#define	LED3		7	/* LED3 status */
#define	FDC		9	/* Full-duplex control */
#define	BSBC		18	/* Burst and Bus control */
#define	SWS		20	/* Software Style */
#define	SRAMSIZ		25	/* SRAM Size */
#define	SRAMB		26	/* SRAM Boundary */
#define	SRAMIC		27	/* SRAM Interface control */

#define	STVAL		31	/* software timer */
#define	MIICAS		32	/* MII control and statuus */
#define	MIIADDR		33	/* MII address */
#define	MIIMDR		34	/* MDIO data */
#define	PCIVID		35	/* PCI VID */

/* MC location 2 */
#define	MC_TMAULOOP	0x4000U
#define	MC_LEDPE	0x1000U	/* LED program enable */
#define	MC_APROMWE	0x0100U	/* Address PROM write enable */
#define	MC_INTLEVEL	0x0080U	/* INT_A: 0:level, 1:edge trigger */
#define	MC_EADISEL	0x0008U	/* EADI select */
#define	MC_AWAKE	0x0004U	/* */
#define	MC_ASEL		0x0002U	/* automatic media select 1:on, 0:off */
#define	MC_XMAUSEL	0x0001U	/* reserved */

/* LED[0-3] location 4-7 */
#define	LED_LEDOUT	0x8000U	/* current value of the LED pin */
#define	LED_LEDPOL	0x4000U	/* polarity 0:low driven, 1:high */
#define	LED_LEDDIS	0x2000U	/* disable */
#define	LED_100E	0x1000U	/* 100M */
#define	LED_MIISE	0x0800U	/* MOII selected  */
#define	LED_DXCVRCTL	0x0400U	/* AUI selected  */
#define	LED_MPSE	0x0200U	/* magic packet enabled  */
#define	LED_FDLSE	0x0100U	/* full duplex */
#define	LED_PSE		0x0080U	/* pulse stratcher enable */
#define	LED_LNKSE	0x0040U	/* link status enable */
#define	LED_RCVME	0x0020U	/* receive match status enable */
#define	LED_XMTE	0x0010U	/* transmit status enable */
#define	LED_RXPOLE	0x0008U	/* receive polarity enable */
#define	LED_RCVE	0x0004U	/* receive status enable */
#define	LED_JABE	0x0002U	/* jabber status enable */
#define	LED_COLE	0x0001U	/* collision status enable */

#define	LED0_DEFAULT	(LED_PSE | LED_LNKSE)

#define	LED_BITS	\
	"\020"	\
	"\020LEDOUT"	\
	"\017LEDPOL"	\
	"\016LEDDIS"	\
	"\015100E"	\
	"\014MIISE"	\
	"\013DXCVRCTL"	\
	"\012MPSE"	\
	"\011FDLSE"	\
	"\010PSE"	\
	"\007LNKSE"	\
	"\006RCVME"	\
	"\005XMTE"	\
	"\004RXPOLE"	\
	"\003RCVE"	\
	"\002JABE"	\
	"\001COLE"

/* FDC location 9 */
#define	FDC_FDRPAD	0x0004U	/* full-duplex runt packet accept disable */
#define	FDC_AUIFD	0x0002U	/* AUI full-duplex */
#define	FDC_FDEN	0x0001U	/* full-duplex enable */

/* BSBC location 18 */
#define	BSBC_ROMTMG	0xf000U	/* expansion ROM timing */
#define		BSBC_ROMTMG_SHIFT	12
#define	BSBC_NOUFLO	0x0800U	/* tx store & forward */
#define	BSBC_MEMCMD	0x0200U	/* 0:MRL, 1:MRM */
#define	BSBC_EXTREQ	0x0100U	/* extend request  */
#define	BSBC_DWIO	0x0080U	/* double word I/O R- */
#define	BSBC_BREADE	0x0040U	/* burst read enable */
#define	BSBC_BWRITE	0x0020U	/* burst write enable */
#define	BSBC_TSTSHDW	0x0018U	/* reserved */
#define	BSBC_LINBC	0x0007U	/* reserved */

#define	BSBC_BITS	\
	"\020"	\
	"\013NOUFLO"	\
	"\012MEMCMD"	\
	"\011EXTREQ"	\
	"\010DWIO"	\
	"\007BREADE"	\
	"\006BWRITE"

/* MIICAS location 31*/
#define	MIICAS_ANTST	0x8000U	/* reserved */
#define	MIICAS_MIIPD	0x4000U	/* MII phy detect */
#define	MIICAS_FMDC	0x3000U	/* fast management data clock */
#define		MIICAS_FMDC_SHIFT	12
#define		MIICAS_FMDC_2_5	(0U << MII_CAS_FMDC_SHIFT)	/* 2.5MHz */
#define		MIICAS_FMDC_5	(1U << MII_CAS_FMDC_SHIFT)	/* 5MHz */
#define		MIICAS_FMDC_10	(3U << MII_CAS_FMDC_SHIFT)	/* 10MHz */
#define	MIICAS_APEP	0x0800U	/* auto poll external phy */
#define	MIICAS_APDW	0x0700U	/* auto poll Dwell time */
#define	MIICAS_APDW_SHIFT	8
#define		MIICAS_APDW_CONT	(0U << MIICAS_APDW_SHIFT)
#define		MIICAS_APDW_128C	(1U << MIICAS_APDW_SHIFT)
#define		MIICAS_APDW_256C	(2U << MIICAS_APDW_SHIFT)
#define		MIICAS_APDW_512C	(3U << MIICAS_APDW_SHIFT)
#define		MIICAS_APDW_1024C	(4U << MIICAS_APDW_SHIFT)
#define		MIICAS_APDW_2048C	(5U << MIICAS_APDW_SHIFT)
#define	MIICAS_DANAS	0x0080U	/* disable AN auto setup */
#define	MIICAS_XPHYRST	0x0040U	/* external phy reset on RESET */
#define	MIICAS_XPHYANE	0x0020U	/* external phy force AN enable */
#define	MIICAS_XPHYFD	0x0010U	/* ex'phy FD when AN disabled */
#define	MIICAS_XPHYSP	0x0008U	/* ex'phy 100M when AN disabled */
#define	MIICAS_MIIuL	0x0004U	/* ex'phy uL6692 */
#define	MIICAS_MIIILP	0x0002U	/* MII internal loopback mode */
#define	MIICAS_FCON	0x0001U	/* ex'phy fast configuration mode */

/* MIIADDR */
#define	MIIADDR_ADDR	0x03e0U
#define		MIIADDR_ADDR_MASK	0x1fU
#define		MIIADDR_ADDR_SHIFT	5
#define	MIIADDR_REG	0x001fU
#define		MIIADDR_REG_MASK	0x1fU
#define		MIIADDR_REG_SHIFT	0


/*
 * Initialization Block
 */
struct init_block {
	volatile uint16_t	ib_mode;	/* csr15 */
	volatile uint8_t	ib_rlen;
	volatile uint8_t	ib_tlen;
#define	IB_LEN_SHIFT	4

	volatile uint8_t	ib_padr[6];	/* csr12-14 */
	volatile uint8_t	:8;
	volatile uint8_t	:8;

	volatile uint32_t	ib_ladrf[2];	/* csr8-11 */
	volatile uint32_t	ib_rdra;	/* csr24-25 */
	volatile uint32_t	ib_tdra;	/* csr31-32 */
};

/*
 * Rx and Tx Descriptors
 */
struct rx_desc {
	volatile uint32_t	rmd0;	/* buffer address */
	volatile uint32_t	rmd1;
	volatile uint32_t	rmd2;
	volatile uint32_t	rmd3;	/* unused */
};

#define	RMD1_OWN	0x80000000U	/* owned by controller */
#define	RMD1_ERR	0x40000000U	/* FRAM, OFLO, CRC, BUFF, BPE */
#define	RMD1_FRAM	0x20000000U	/* framing error */
#define	RMD1_OFLO	0x10000000U	/* overflow error */
#define	RMD1_CRC	0x08000000U	/* crc error */
#define	RMD1_BUFF	0x04000000U	/* buffer error */
#define	RMD1_STP	0x02000000U	/* start of packet */
#define	RMD1_ENP	0x01000000U	/* end of packet */
#define	RMD1_BPE	0x00800000U	/* bus parity error */
#define	RMD1_PAM	0x00400000U	/* physical address match */
#define	RMD1_LAFM	0x00200000U	/* logical address match */
#define	RMD1_BAM	0x00100000U	/* broadcast address match */
#define	RMD1_BCNT	0x0000ffffU	/* buffer byte count */

#define	RMD1_BITS	\
	"\020"	\
	"\040OWN"	\
	"\037ERR"	\
	"\036FRAM"	\
	"\035OFLO"	\
	"\034CRC"	\
	"\033BUFF"	\
	"\032STP"	\
	"\031ENP"	\
	"\030BPE"	\
	"\027PAM"	\
	"\026LAFM"	\
	"\025BAM"

#define	RMD2_RFRTAG	0x7fff0000U
#define		RMD2_RFRTAG_MASK	0x7fffU
#define	RMD2_RFRTAG_SHIFT	16
#define	RMD2_MCNT	0x00000fffU

struct tx_desc {
	volatile uint32_t	tmd0;	/* buffer address */
	volatile uint32_t	tmd1;
	volatile uint32_t	tmd2;
	volatile uint32_t	tmd3;	/* unused */
};

#define	TMD1_OWN	0x80000000U	/* owned by controller */
#define	TMD1_ERR	0x40000000U	/* UFLO|LCOL|LCAR|RTRY|BPE */
#define	TMD1_ADD_FCS	0x20000000U	/* add fcs (at STP only) */
#define	TMD1_NO_FCS	TMD1_ADD_FCS	/* no fcs (at ENP only) */
#define	TMD1_MORE	0x10000000U	/* more than one retry was needed */
#define	TMD1_LTINT	TMD1_MORE	/* suppress interrupt on tx done */
#define	TMD1_ONE	0x08000000U	/* exactly one retry was needed */
#define	TMD1_DEF	0x04000000U	/* deferred */
#define	TMD1_STP	0x02000000U	/* start of packet */
#define	TMD1_ENP	0x01000000U	/* end of packet */
#define	TMD1_BPE	0x00800000U	/* bus parity error */
#define	TMD1_BCNT	0x0000ffffU

#define	TMD1_BITS	\
	"\020"	\
	"\040OWN"	\
	"\037ERR"	\
	"\036ADD_FCS"	\
	"\035MORE"	\
	"\034ONE"	\
	"\033DEF"	\
	"\032STP"	\
	"\031ENP"	\
	"\030BPE"

#define	TMD2_BUFF	0x80000000U	/* buffer error (no ENP) */
#define	TMD2_UFLO	0x40000000U	/* under flow */
#define	TMD2_EXDEF	0x20000000U
#define	TMD2_LCOL	0x10000000U	/* late collision */
#define	TMD2_LCAR	0x08000000U	/* lost carry */
#define	TMD2_RTRY	0x04000000U	/* retry */
#define	TMD2_TRC	0x0000000fU	/* tx retry count */

#define	TMD2_BITS	\
	"\020"	\
	"\040BUFF"	\
	"\037UFLO"	\
	"\036EXDEF"	\
	"\035LCOL"	\
	"\034LCAR"	\
	"\033RTRY"

#define	TMD2_ANYERROR	(TMD2_BUFF|TMD2_UFLO|TMD2_LCOL|TMD2_LCAR|TMD2_RTRY)

#define	DESCRING_ALIGN	(16)

#endif /* _PCNET_H_ */
