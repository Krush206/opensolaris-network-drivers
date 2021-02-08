/*
 * @(#)mtd800reg.h	1.2 09/02/23
 * register definition of myson mtd803
 * Coded by M.Murayama. This file is public domain.
 */

#ifndef __MTD800REG_H__
#define	__MTD800REG_H__

/*
 * Register offsets
 */
#define	PAR		0x00	/*  physical address register */
#define	MAR		0x08	/*  multicast address register */

#define	FAR		0x10	/* flow control address register */
#define	RCR		0x18	/* receive configuration */
#define	TCR		0x1a	/* transmit configration */
#define	BCR		0x1c	/* bus command register */

#define	TxPDR		0x20	/* -/W */
#define	RxPDR		0x24	/* -/W */
#define	RxCWP		0x28
#define	TxLBA		0x2c	/* transmit list base address */

#define	RxLBA		0x30	/* receive list base address */
#define	ISR		0x34	/* interrupt status register */
#define	IMR		0x38	/* interrupt mask register */
#define	FCLT		0x3c	/* flow control low threshold */
#define	FCHT		0x3e	/* flow control high threshold */

#define	MII		0x40
#define	SROM_CR		0x42
#define	BROM_CR		0x43
#define	TC_MPA		0x44
#define	TC_CRC		0x46
#define	TC_TSR		0x48
#define	PHYBA		0x4c	/* 2byte * 12: internal PHY register */

/*  RCR: receive configuration regster, offset 0x18 */
#define	RCR_RxS		0x8000U	/* receive status */
#define	RCR_EIEN	0x4000U	/* early interrupt enable */
#define	RCR_RFCEN	0x2000U	/* receive flow control enable */
#define	RCR_NDFA	0x1000U
#define	RCR_RBLEN	0x0800U	/* receive burst length */
#define	RCR_RPB		0x0700U
#define		RCR_RPB_SHIFT		8
#define		RCR_RPB_1	(0U << RCR_RPB_SHIFT)
#define		RCR_RPB_4	(1U << RCR_RPB_SHIFT)
#define		RCR_RPB_8	(2U << RCR_RPB_SHIFT)
#define		RCR_RPB_16	(3U << RCR_RPB_SHIFT)
#define		RCR_RPB_32	(4U << RCR_RPB_SHIFT)
#define		RCR_RPB_64	(5U << RCR_RPB_SHIFT)
#define		RCR_RPB_128	(6U << RCR_RPB_SHIFT)
#define		RCR_RPB_512	(7U << RCR_RPB_SHIFT)

#define	RCR_PROM	0x0080U	/* promiscuous mode */
#define	RCR_AB		0x0040U	/* accept broadcast packet */
#define	RCR_AM		0x0020U	/* accept multicast */
#define	RCR_ARP		0x0008U	/* accept runt packet */
#define	RCR_ALP		0x0004U	/* accept long packet */
#define	RCR_SEP		0x0002U	/* accept all error packet */
#define	RCR_RE		0x0001U	/* receive enable */

#define	RxFilterMode	\
	(RCR_PROM | RCR_AB | RCR_AM | RCR_ARP | RCR_ALP | RCR_SEP)

#define	RCR_BITS	\
	"\020"	\
	"\020RxS"	\
	"\017EIEN"	\
	"\016RFCEN"	\
	"\015NDFA"	\
	"\014RBLEN"	\
	"\010PROM"	\
	"\007AB"	\
	"\006AM"	\
	"\004ARP"	\
	"\003ALP"	\
	"\002SEP"	\
	"\001RE"

/*  TCR: transmit configuration regster, offset 0x1a */
#define	TCR_TxS		0x8000	/* transmit status */
#define	TCR_BACKOPT	0x1000	/* optional back off */
#define	TCR_FBACK	0x0800	/* fast back off */
#define	TCR_Enhanced	0x0200
#define	TCR_TFCEN	0x0100	/* tx flow control paket enable back off */
#define	TCR_TFT		0x00e0	/* tx fifo threshold */
#define		TCR_TFT_SHIFT	5
#define		TCR_TFT_64	(0 << TCR_TFT_SHIFT)
#define		TCR_TFT_32	(1 << TCR_TFT_SHIFT)
#define		TCR_TFT_128	(2 << TCR_TFT_SHIFT)
#define		TCR_TFT_256	(3 << TCR_TFT_SHIFT)
#define		TCR_TFT_512	(4 << TCR_TFT_SHIFT)
#define		TCR_TFT_768	(5 << TCR_TFT_SHIFT)
#define		TCR_TFT_1024	(6 << TCR_TFT_SHIFT)
#define		TCR_TFT_SF	(7 << TCR_TFT_SHIFT)
#define	TCR_FD		0x0010	/* duplex mode 1:full, 0:half */
#define	TCR_PS		0x0008	/* port speed 1:10M 0:100M */
#define	TCR_TE		0x0004	/* transmit enable */
#define	TCR_LB		0x0003
#define		TCR_LB_SHIFT	0
#define		TCR_LB_NORMAL	(0 << TCR_LB_SHIFT)
#define		TCR_LB_MII	(1 << TCR_LB_SHIFT)

/* bus command register, offset 0x1c  */
#define	BCR_Prog	0x00000200U
#define	BCR_RLE		0x00000100U
#define	BCR_RME		0x00000080U
#define	BCR_WIE		0x00000040U
#define	BCR_PBL		0x00000038U	/* programable burst length */
#define		BCR_PBL_SHIFT	3
#define		BCR_PBL_1	(0U << BCR_PBL_SHIFT)	/* 1 longword */
#define		BCR_PBL_4	(1U << BCR_PBL_SHIFT)	/* 4 longwords */
#define		BCR_PBL_8	(2U << BCR_PBL_SHIFT)	/* 8 longwords */
#define		BCR_PBL_16	(3U << BCR_PBL_SHIFT)	/* 16 longwords */
#define		BCR_PBL_32	(4U << BCR_PBL_SHIFT)	/* 32 longwords */
#define		BCR_PBL_64	(5U << BCR_PBL_SHIFT)	/* 64 longwords */
#define		BCR_PBL_128	(6U << BCR_PBL_SHIFT)	/* 128 longwords */
#define		BCR_PBL_512	(7U << BCR_PBL_SHIFT)	/* 512 longwords */
#define	BCR_SWR		0x00000001U	/* software reset */

/* interrupt status(0x34)/enable(0x38) registers. */
#define	INT_PDF		0x00040000U	/* parallel detection fault */
#define	INT_RFCON	0x00020000U	/* rx flowcontol on */
#define	INT_RFCOFF	0x00010000U	/* rx flowcontol off */
#define	INT_LSC		0x00008000U	/* link status change */
#define	INT_ANC		0x00004000U	/* auto negotiation completed */
#define	INT_FBE		0x00002000U	/* fatal bus error */
#define	INT_ET		0x00001800U	/* error type */
#define	INT_ET_PE		(0U << INR_ET_SHIFT)	/* parity error */
#define	INT_ET_MA		(1U << INR_ET_SHIFT)	/* master abort */
#define	INT_ET_TA		(2U << INR_ET_SHIFT)	/* target abort error */
#define	INT_TUNF	0x00000400U	/* tx underrun */
#define	INT_ROVF	0x00000200U	/* rx overflow */
#define	INT_ETI		0x00000100U	/* early transmit interrupt */
#define	INT_ERI		0x00000080U	/* early receive interrupt */
#define	INT_CNTOVF	0x00000040U	/* statistic counter overflow */
#define	INT_RBU		0x00000020U	/* receive buffer unavailable */
#define	INT_TBU		0x00000010U	/* tranmit buffer unavailable */
#define	INT_TI		0x00000008U	/* transmit done */
#define	INT_RI		0x00000004U	/* receive done */
#define	INT_RxErr	0x00000002U	/* receive error */

#define	INT_BITS	\
	"\020"	\
	"\023PDF"	\
	"\022RFON"	\
	"\021RFOFF"	\
	"\020LSC"	\
	"\017ANC"	\
	"\016FBE"	\
	"\013TUNF"	\
	"\012ROVF"	\
	"\011ETI"	\
	"\010ERI"	\
	"\017CNTOVF"	\
	"\006RBU"	\
	"\005TBU"	\
	"\004TI"	\
	"\003RI"	\
	"\002RxErr"

/* MII management, offset 0x40 */
#define	MII_MOUT	0x08U		/* MDIO output enable */
#define	MII_MDO		0x04U		/* data port output status */
#define		MII_MDO_SHIFT		2
#define	MII_MDI		0x02U		/* data port input status */
#define		MII_MDI_SHIFT		1
#define	MII_MDC		0x01U		/* clock port status */

/* SROM_CR, offset 0x42 */
#define	SROM_DPM	0x80U		/* direct programming */
#define	SROM_SPROMPS	0x40U		/* srom programming status (R/-) */
#define	SROM_AUTOLD	0x10U		/* load srom contents (R/Wsc) */
#define	SROM_ECS	0x08U
#define	SROM_ECK	0x04U
#define	SROM_EDI	0x02U		/* data to srom */
#define		SROM_EDI_SHIFT	1
#define	SROM_EDO	0x01U		/* data from srom */
#define		SROM_EDO_SHIFT	0

/* BROM_CR, offset 0x43 */
#define	BROM_BRWE	0x20U		/* bootrom write enable */
#define	BROM_BRSZ	0x1cU		/* boot rom size */
#define		BROM_BRSZ_SHIFT	2
#define		BROM_BRSZ_0	(0U << BROM_BRSZ_SHIFT)
#define		BROM_BRSZ_8K	(1U << BROM_BRSZ_SHIFT)
#define		BROM_BRSZ_16K	(2U << BROM_BRSZ_SHIFT)
#define		BROM_BRSZ_32K	(3U << BROM_BRSZ_SHIFT)
#define		BROM_BRSZ_64K	(4U << BROM_BRSZ_SHIFT)
#define		BROM_BRSZ_128K	(5U << BROM_BRSZ_SHIFT)
#define	BROM_BRPSD	0x03U		/* boot rom speed select */
#define		BROM_BRPSD_SHIFT	0
#define		BROM_BRPSD_120nS	(0U << BROM_BRPSD_SHIFT)
#define		BROM_BRPSD_180nS	(1U << BROM_BRPSD_SHIFT)
#define		BROM_BRPSD_240nS	(2U << BROM_BRPSD_SHIFT)
#define		BROM_BRPSD_300nS	(3U << BROM_BRPSD_SHIFT)

/* TC_TSR, offset 0x48 */
#define	TC_TSR_ABORT		0xff000000U
#define	TC_TSR_ABORT_SHIFT		24
#define	TC_TSR_LCOL		0x00ff0000U
#define	TC_TSR_LCOL_SHIFT	16
#define	TC_TSR_NSR		0x0000ffffU
#define	TC_TSR_NSR_SHIFT	0


/* Tx buffer descriptors. */
struct tx_desc {
	volatile uint32_t	tdes0;	/* status */
	volatile uint32_t	tdes1;	/* ctrl and length */
	volatile uint32_t	tdes2;	/* buffer address */
	volatile uint32_t	tdes3;	/* next descriptor */
};

#define	TDES0_OWN	0x80000000U
#define	TDES0_ABORT	0x00002000U	/* abort (excolls or late colls) */
#define	TDES0_CSL	0x00001000U	/* carrier sense lost */
#define	TDES0_LC	0x00000800U	/* late collision */
#define	TDES0_EC	0x00000400U	/* excessive collision */
#define	TDES0_DFR	0x00000200U	/* deferred */
#define	TDES0_HF	0x00000100U	/* heartbeat failure */
#define	TDES0_NCR	0x000000ffU	/* collision retry count */

#define	TDS_BITS	\
	"\020"	\
	"\040OWN"	\
	"\016ABORT"	\
	"\015CSL"	\
	"\014LC"	\
	"\013EC"	\
	"\012DFR"	\
	"\011HF"

#define	TDES1_IC	0x80000000U	/* interrupt control */
#define	TDES1_EIC	0x40000000U	/* early interrupt control */
#define	TDES1_LD	0x20000000U	/* the last segment in a frame */
#define	TDES1_FD	0x10000000U	/* the first segment in a frame */
#define	TDES1_CRC	0x08000000U	/* crc append */
#define	TDES1_PAD	0x04000000U	/* pad control */
#define	TDES1_RTLC	0x02000000U	/* retry late collision */
#define	TDES1_PKTS	0x003ff800U	/* packet size */
#define		TDES1_PKTS_SHIFT	11
#define	TDES1_TBS	0x000007ffU	/* transmit buffer size */

#define	TDES1_BITS	\
	"\020"	\
	"\040IC"	\
	"\037EIC"	\
	"\036LD"	\
	"\035FD"	\
	"\034CRC"	\
	"\033PAD"	\
	"\032RTLC"

/* Rx buffer descriptors. */
struct rx_desc {
	volatile uint32_t	rdes0;	/* status */
	volatile uint32_t	rdes1;	/* length */
	volatile uint32_t	rdes2;	/* buffer address */
	volatile uint32_t	rdes3;	/* next descriptor */
};

#define	RDES0_OWN	0x80000000U
#define	RDES0_FLNG	0x07ff0000U	/* received length */
#define		RDES0_FLNG_SHIFT	16
#define	RDES0_MAR	0x00004000U	/* multicast address received */
#define	RDES0_BAR	0x00002000U	/* broadcast address received */
#define	RDES0_PHY	0x00001000U	/* physical address received */
#define	RDES0_FSD	0x00000800U	/* first descriptor */
#define	RDES0_LSD	0x00000400U	/* last descriptor */
#define	RDES0_ES	0x00000080U	/* error summary */
#define	RDES0_RUNT	0x00000040U	/* shorter than 64 */
#define	RDES0_LONG	0x00000020U	/* longer than 1518 */
#define	RDES0_FAE	0x00000010U	/* frame align error */
#define	RDES0_CRC	0x00000008U	/* crc error */
#define	RDES0_RXER	0x00000004U	/* receive coding error */

#define	RDS_BITS	\
	"\020"	\
	"\040OWN"	\
	"\017MAR"	\
	"\016BAR"	\
	"\015PHY"	\
	"\014FSD"	\
	"\013LSD"	\
	"\010ES"	\
	"\007RUNT"	\
	"\006LONG"	\
	"\005FAE"	\
	"\004CRC"	\
	"\003RXER"

#define	DESC_ALIGN	16

#endif /* __MTD800REG_H__ */
