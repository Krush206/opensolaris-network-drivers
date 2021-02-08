/*
 * @(#)tnete100areg.h	1.4 05/05/09
 * Macro definition for Texas Instruments ThanderLAN TNETE100A nic.
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */

/*
 * Known PCI vendor-id/device-id 
 */
#define	PCI_VID_TI		0x104c
#define	PCI_DID_TNETE100A	0x0500

/*
 * TNETE100A register definitions
 */

/* Offsets to Host registers. */
#define	HOST_CMD		0x00	/* int */
#define	CH_PARM			0x04	/* int */
#define	DIO_ADR			0x08	/* short */
#define	HOST_INT		0x0a	/* short */
#define	DIO_DATA		0x0c	/* int */

/* Offsets to Internal registers. */
#define	NETCMD			0x00
#define	NETSIO			0x01
#define	NETSTS			0x02
#define	NETMASK			0x03
#define	NETCFG			0x04
#define	MANTEST			0x06
#define	DEFAULT_VENID		0x08
#define	DEFAULT_DEVID		0x0a
#define	DEFAULT_REV		0x0c
#define	DEFAULT_SUBCLASS	0x0d
#define	DEFAULT_MINGNT		0x0e
#define	DEFAULT_MAXLAT		0x0f
#define	Areg_0			0x10	/* base of Areg_0 */
#define	Areg_1			0x16	/* base of Areg_1 */
#define	Areg_2			0x1c	/* base of Areg_2 */
#define	Areg_3			0x22	/* base of Areg_3 */
#define	HASH1			0x28
#define	HASH2			0x2c
#define	Tx_UN_GOOD		0x30
#define	Rx_OV_GOOD		0x34
#define	DeferredTx		0x38
#define	CRCError		0x3a
#define	CodeError		0x3b
#define	Multicollisions		0x3c
#define	Singlecollisions	0x3e
#define	ExcessiveCollisions	0x40
#define	LateCollisions		0x41
#define	CarrierLoss		0x42
#define	Acommit			0x43
#define	LEDreg			0x44
#define	BSIZEreg		0x45
#define	MaxRx			0x46
#define	INTDIS			0x48

/*
 * HOST_CMD register
 */
#define	HC_Go		0x80000000
#define	HC_Stop		0x40000000
#define	HC_Ack		0x20000000
#define	HC_ChSel	0x1fe00000
#define		HC_ChSel_SHIFT	21
#define	HC_EOC		0x00100000
#define	HC_Rx		0x00080000
#define	HC_Nes		0x00040000
#define	HC_AdRst	0x00008000
#define	HC_LdTmr	0x00004000
#define	HC_LdThr	0x00002000
#define	HC_ReqInt	0x00001000
#define	HC_IntsOff	0x00000800
#define	HC_IntsOn	0x00000400
#define	HC_AckCount	0x000000ff

#define	HC_TxGO(c)		(HC_Go | ((c) << HC_ChSel_SHIFT))
#define	HC_RxGO(c)		(HC_Go | HC_Rx | ((c) << HC_ChSel_SHIFT))
#define	HC_TxSTOP(c)		(HC_Stop | ((c) << HC_ChSel_SHIFT))
#define	HC_RxSTOP(c)		(HC_Stop | HC_Rx | ((c) << HC_ChSel_SHIFT))

/*
 * CH_PARM register on adapter check interrupt
 */
#define	AdC_Channel	0x1fe00000
#define	AdC_Channel_SHIFT	21
#define	AdC_List	0x00100000
#define	AdC_Rx		0x00080000
#define	AdC_Rd		0x00040000
#define	AdC_FailureCode	0x000000ff

#define	FC_DataPar	1	/* Data parity error */
#define	FC_AdrsPar	2	/* Address parity error */
#define	FC_Mabort	3	/* Master abort */
#define	FC_Rabort	4	/* Target abort */
#define	FC_ListErr	5	/* List error */
#define	FC_AckErr	6	/* Acknowledge error */
#define	FC_IovErr	7	/* Int overflow error */

/*
 * Host Interrupt register
 */
#define	HI_IntVec	0x1fe0
#define		HI_IntVec_SHIFT	5
#define	HI_IntType	0x001c
#define		HI_IntType_SHIFT	2

#define	IntType_NoIntr		0
#define	IntType_TxEOF		1
#define	IntType_StatisticsOvf	2
#define	IntType_RxEOF		3
#define	IntType_Dummy		4
#define	IntType_TxEOC		5
#define	IntType_AdNet		6
#define	IntType_RxEOC		7

/*
 * DIO Address register
 */
#define	DIO_ADR_INC	0x8000	/* Address increment */
#define	DIO_RAM_ADR	0x4000	/* RAM Address Base */
#define	DIO_ADR_SEL	0x3fff	/* Address */

/*
 * Network Command Register - NetCmd@0x00(DIO)
 */
#define	NCMD_NRESET	0x80	/* Not Reset */
#define	NCMD_NWRAP	0x40	/* Not Wrap i.e. Not loop back mode */
#define	NCMD_CSF	0x20	/* Copy Short Frame, i.e. Runt */
#define	NCMD_CAF	0x10	/* Copy All Frame, i.e. promiscious */
#define	NCMD_NOBRX	0x08	/* No broadcast */
#define	NCMD_DUPLEX	0x04	/* Full duplex */
#define	NCMD_TRFRAM	0x02	/* Token Ring mode */
#define	NCMD_TXPACE	0x01	/* Tx Pacing */

/*
 * Network Serial I/O Register - NetSio@0x01(DIO)
 */
#define	NSIO_MINTEN	0x80	/* MII Interrupt enable */
#define	NSIO_ECLOK	0x40	/* EEPROM SIO clock */
#define	NSIO_ETXEN	0x20	/* EEPROM SIO transmit enable */
#define	NSIO_EDATA	0x10	/* EEPROM SIO data */
#define	NSIO_EDATA_SHIFT	4
#define	NSIO_NMRST	0x08	/* MII Not reset */
#define	NSIO_MCLK	0x04	/* MII SIO clock */
#define	NSIO_MTXEN	0x02	/* MII SIO transmit enable */
#define	NSIO_MDATA	0x01	/* MII SIO data */
#define	NSIO_MDATA_SHIFT	0

/*
 * Network Status register - NetSts@0x02(DIO)
 */
#define	NSTS_MIRQ	0x80	/* MII interrupt request */
#define	NSTS_HBEAT	0x40	/* Heart beat error */
#define	NSTS_TXSTOP	0x20	/* Transmitter stopped */
#define	NSTS_RXSTOP	0x10	/* Receiver stopped */

/*
 * Network Status Mask register - NetMask@0x03(DIO)
 */
#define	NMASK_MASK7	0x80	/* MII interrupt request */
#define	NMASK_MASK6	0x40	/* Heart beat error */
#define	NMASK_MASK5	0x20	/* Transmitter stopped */
#define	NMASK_MASK4	0x10	/* Receiver stopped */

/*
 * Network Configuration register - NetMask@0x04(DIO)
 */
#define	NCFG_RclkTest	0x8000	/* Test MRCLK */
#define	NCFG_TclkTest	0x4000	/* Test MTCLK */
#define	NCFG_BITrate	0x2000	/* bit level 10Mbps MII */
#define	NCFG_RxCRC	0x1000	/* transfer RxCRC into rx buffer */
#define	NCFG_PEF	0x0800	/* Pass error frames */
#define	NCFG_ONEfrag	0x0400	/* single fragment mode for Rx */
#define	NCFG_ONEchn	0x0200	/* single channel mode for Tx */
#define	NCFG_MTEST	0x0100	/* Manufacturing test */
#define	NCFG_PHY_En	0x0080	/* 10M bps on-chip PHY enable */
#define	NCFG_MACselect	0x007f	/* MAC protocol select */
#define	MACselect_CSMACD	0	/* 802.3 - 10/100M bps */
#define	MACselect_100VG_ch	1	/* 100VG-AnyLAN */
#define	MACselect_100VG_cstat	2	/* 100VG-AnyLAN */
#define	MACselect_802_3u	3	/* 100VG-AnyLAN */

/*
 * Adaptor Commit register - Acommit@0x43(DIO)
 */
#define	AC_TX_64	0x00
#define	AC_TX_128	0x10
#define	AC_TX_256	0x20
#define	AC_TX_512	0x30
#define	AC_TX_1024	0x40
#define	AC_TX_SF	0x50
#define	AC_PHY_MTXD3	0x08	/* full duplex disable */
#define	AC_PHY_MTXD2	0x04	/* loopback enble */
#define	AC_PHY_MTXD1	0x02	/* 10BaseT(0) AUI-ThinNet(1) select */
#define	AC_PHY_MTXER	0x01	/* reserved */

/*
 * Burst size register - BSIZEreg@0x45(DIO)
 */
#define	BS_TX_16	0x00
#define	BS_TX_32	0x10
#define	BS_TX_64	0x20	/* default */
#define	BS_TX_128	0x30
#define	BS_TX_256	0x40
#define	BS_TX_512	0x50
#define	BS_RX_16	0x00
#define	BS_RX_32	0x01
#define	BS_RX_64	0x02	/* default */
#define	BS_RX_128	0x03
#define	BS_RX_256	0x04
#define	BS_RX_512	0x05

/*
 * Interrupt Disable register - INTDIS@0x48(DIO)
 */
#define	INTDIS_TxEOC	0x04
#define	INTDIS_RxEOF	0x02
#define	INTDIS_RxEOC	0x01

/*
 * Definition of Tx and Rx lists
 */
struct frag {
	uint32_t	DataCount;
	uint32_t	DataAddress;
};
#define	DATACOUNT_LAST	0x80000000	/* in datacount field in struct frag */

#define TNE_MAXRXFRAGS	10
struct rx_list {
	uint32_t	forward_ptr;
	uint32_t	rx_cstat;
	struct frag	frags[TNE_MAXRXFRAGS];
};

/* Bit definition on Rx CSTAT and FRAMESIZE */
#define	RC_FRAMESIZE	0xffff0000	/* received frame size */
#define		RC_FRAMESIZE_SHIFT	16
#define		RC_FRAMESIZE_MASK	0xffff
#define	RC_FrmCmp	0x00004000	/* Frame complete */
#define	RC_RxEOC	0x00000800	/*  */
#define	RC_RxError	0x00000400	/* Error frame */
#define	RC_DPpr		0x00000010	/* Demand priority frame priority */
#define	RC_MBO		0x00003000	/* Must be one for backword compatibility */

#define	RC_BITS	\
	"\020"	\
	"\017FrmCmp"	\
	"\014RxEOC"	\
	"\013RxError"	\
	"\005DPpr"

#define TNE_MAXTXFRAGS 10
struct tx_list {
	uint32_t	forward_ptr;
	uint32_t	tx_cstat;
	struct frag	frags[TNE_MAXRXFRAGS];
};

/* Bit definition on Tx CSTAT */
#define	TC_FRAMESIZE	0xffff0000	/* tx frame size */
#define		TC_FRAMESIZE_SHIFT	16
#define		TC_FRAMESIZE_MASK	0xffff
#define	TC_FrmCmp	0x00004000	/* Frame complete */
#define	TC_TxEOC	0x00000800	/*  */
#define	TC_PassCRC	0x00000200	/* use last data as CRC */
#define	TC_NetPri	0x00000007	/* Demand priority frame priority */
#define	TC_MBO		0x00003000	/* Must be one for backword compatibility */

#define	TC_BITS	\
	"\020"	\
	"\017FrmCmp"	\
	"\014TxEOC"	\
	"\011PassCRC"

