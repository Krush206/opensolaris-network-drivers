/*
 * @(#)3c90xreg.h	1.3 11/09/19
 * 3COM 3C90XX register definition.
 *
 * This header file is based on 3COM's 3C90xB amd 3C90xC data sheets
 * This file is public domain.
 */

/*
 * Offset of registers in a window
 */

/* common and extended registers */
#define	WxIntStatus		0xe
#define	WxCommand		0xe
#define	WxTxPktId		0x18	/* 90xB */
#define	WxTimer			0x1a
#define	WxTxStatus		0x1b
#define	WxIntStatus_1e		0x1e
#define	WxIntStatusAuto		0x1e
#define	WxCommand_1e		0x1e
#define	WxDmaCtrl		0x20
#define	WxDnListPtr		0x24
#define	WxDnBurstThresh		0x2a
#define	WxDnPriorityThresh	0x2c
#define	WxDnPoll		0x2d
#define	WxTxFreeThresh		0x2f	/* 90x */
#define	WxUpPktStatus		0x30
#define	WxFreeTimer		0x34
#define	WxCountdown		0x36
#define	WxUpListPtr		0x38
#define	WxUpPriorityThresh	0x3c
#define	WxUpPoll		0x3d
#define	WxUpBurstThresh		0x3e
#define	WxRealTimeCnt		0x40
#define	WxDebugData		0x70	/* 90XB/C */
#define	WxDebugControl		0x74	/* 90XB/C */
#define	WxDnMaxBurst		0x78	/* 90XB/C */
#define	WxPowerMgmtCtrl		0x7a

/* Window 0 */
#define	W0EepromCommand		0xa
#define	W0EepromData		0xc

/* Window 1 */
#define	W1RxError		0x4	/* obsolete, 90X only */
#define	W1RxStatus		0x8	/* obsolete, 90X only */
#define	W1Timer			0xa	/* obsolete, 90X only */
#define	W1TxStatus		0xb	/* obsolete, 90X only */
#define	W1TxFree		0xc	/* obsolete, 90X only */

/* Window 2 */
#define	W2StationAddress	0x0
#define	W2StationMask		0x6
#define	W2ResetOptions		0xc	/* w 90xB/C */

/* Window 3 */
#define	W3InternalConfig	0x0	/* dw */
#define	W3MaxPktSize		0x4	/* w 90XB/C */
#define	W3MacControl		0x6	/* w */
#define	W3ResetOptions_old	0x8	/* w, obsolete, 90x */
#define	W3MediaOptions		0x8	/* w */
#define	W3RxFree		0xa	/* w */
#define	W3TxFree		0xc	/* w ro */

/* Window 4 */
#define	W4FifoDiagnostic	0x4
#define	W4NetworkDiagnostic	0x6
#define	W4PhysicalMgmt		0x8
#define	W4MediaStatus		0xa
#define	W4BadSSD		0xc
#define	W4UpperBytesOk		0xd

/* Window 5 */
#define	W5TxStartThresh		0x0	/* ro */
#define	W5TxavailableThresh	0x2	/* not supported */
#define	W5RxEarlyThresh		0x6
#define	W5RxFilter		0x8
#define	W5TxReclaimThresh	0x9	/* ro, 90xB */
#define	W5InterruptEnable	0xa
#define	W5IndicationEnable	0xc

/* Window 6 */
#define	W6CarrierLost		0x0	/* B */
#define	W6SqeErrors		0x1	/* B */
#define	W6MultipleCollisions	0x2	/* B */
#define	W6SingleCollisions	0x3	/* B */
#define	W6LateCollisions	0x4	/* B */
#define	W6RxOverruns		0x5	/* B */
#define	W6FramesXmittedOk	0x6	/* B */
#define	W6FramesRcvdOk		0x7	/* B */
#define	W6FramesDeferred	0x8	/* B */
#define	W6UpperFramesOk		0x9	/* B */
#define	W6BytesRcvdOk		0xa	/* W */
#define	W6BytesXmittedOk	0xc	/* W */

/* Window 7 */
#define	W7VlanEtherType		0x4	/* W */
#define	W7Timer			0xa	/* B, obsolete, 90X only */
#define	W7TxStatus		0xb	/* B, obsolete, 90X only */

/*
 * Register definitions
 */

/*
 * Common and extended registers
 */

/* Command register (offset 0xe) */
#define	CmdShift		11
#define	CmdGlobalReset		(0x00 << CmdShift)
#	define	tpAuiReset		0x0001
#	define	endecReset		0x0002
#	define	networkReset		0x0004
#	define	fifoReset		0x0008
#	define	aismReset		0x0010
#	define	hostReset		0x0020
#	define	vcoReset		0x0040
#	define	upDownReset		0x0080
#define	CmdSelectRegisterWindow	(0x01 << CmdShift)
#define	CmdEnableDcConverter	(0x02 << CmdShift)
#define	CmdRxDisable		(0x03 << CmdShift)
#define	CmdRxEnable		(0x04 << CmdShift)
#define	CmdRxReset		(0x05 << CmdShift)
#	define	tpAuiRxReset		0x0001
#	define	endecRxReset		0x0002
#	define	networkRxReset		0x0004
#	define	fifoRxReset		0x0008
#	define	upRxReset		0x0100
#define	CmdUpStall		((0x06 << CmdShift) | 0)
#define	CmdUpUnstall		((0x06 << CmdShift) | 1)
#define	CmdDnStall		((0x06 << CmdShift) | 2)
#define	CmdDnUnstall		((0x06 << CmdShift) | 3)
#define	CmdTxDone		(0x07 << CmdShift)
#define	CmdRxDiscard		(0x08 << CmdShift)
#define	CmdTxEnable		(0x09 << CmdShift)
#define	CmdTxDisable		(0x0a << CmdShift)
#define	CmdTxReset		(0x0b << CmdShift)
#	define	tpAuiTxReset		0x0001
#	define	endecTxReset		0x0002
#	define	networkTxReset		0x0004
#	define	fifoTxReset		0x0008
#	define	dnTxReset		0x0100
#define	CmdRequestInterrupt	(0x0c << CmdShift)
#define	CmdAcknowledgeInterrupt	(0x0d << CmdShift)
#	define	interruptLatchAck	0x0001
#	define	rxEarlyAck		0x0020
#	define	intRequestedAck		0x0040
#	define	dnCompleteAck		0x0200
#	define	upCompleteAck		0x0400
#	define	AllAck	\
		(interruptLatchAck | rxEarlyAck | intRequestedAck | \
		 dnCompleteAck | upCompleteAck )
#define	CmdSetInterruptEnable	(0x0e << CmdShift)
#define	CmdSetIndicationEnable	(0x0f << CmdShift)
#	define	AllIndications	\
		(IS_interruptLatch | IS_hostError | IS_txComplete |	\
		 IS_rxComplete | IS_rxEarly | IS_intRequested |	\
		 IS_updateStats | IS_linkEvent | IS_dnComplete |	\
		 IS_upComplete)
#define	CmdSetRxFilter		(0x10 << CmdShift)
#	define	rxFilterIndividual	0x0001
#	define	rxFilterAllMulticast	0x0002
#	define	rxFilterBroadcast	0x0004
#	define	rxFilterPromiscuous	0x0008
#	define	rxFilterMulticastHash	0x0010	/* 90xB */

#define	CmdSetRxEarlyThresh	(0x11 << CmdShift)
#define		RxEarlyThreshUnit	4
#define		RxEarlyThreshMask	0x07ff
#define	CmdTxAgain		(0x12 << CmdShift)
#define	CmdSetTxStartThresh	(0x13 << CmdShift)
#define		TxStartThreshUnit	4
#define		TxStartThreshMask	0x07ff

#define	CmdStatisticsEnable	(0x15 << CmdShift)
#define	CmdStatisticsDisable	(0x16 << CmdShift)
#define	CmdDisableDcConverter	(0x17 << CmdShift)
#define	CmdSetTxReclaimThresh	(0x18 << CmdShift)
#define		TxReclaimThreshUnit	16
#define		TxReclaimThreshMask	0x00ff
#define	CmdSetHashFilterBits	(0x19 << CmdShift)	/* 90xB */
#define	CmdTxFifoBisect		(0x1b << CmdShift)

/* IntStatus register (offset 0xe) */
#define	IS_interruptLatch	0x0001
#define	IS_hostError		0x0002
#define	IS_txComplete		0x0004	/* packets were written into fifo */
#define	IS_rxComplete		0x0010	/* packets received in fifo */
#define	IS_rxEarly		0x0020
#define	IS_intRequested		0x0040
#define	IS_updateStats		0x0080	/* 90xB */
#define	IS_linkEvent		0x0100
#define	IS_dnComplete		0x0200	/* a packet download completed */
#define	IS_upComplete		0x0400	/* a packet upload completed */
#define	IS_cmdInProgress	0x1000
#define	IS_windowNumber		0xe000
#define		IS_windowNumberMask	7
#define		IS_windowNumberShift	13

#define	IntStatusBits	\
	"\020"	\
	"\015cmdInProgress"	\
	"\013upComplete"	\
	"\012dnComplete"	\
	"\011linkEvent"	\
	"\010updateStats"	\
	"\007intRequested"	\
	"\006rxEarly"	\
	"\005rxComplete"	\
	"\003txComplete"	\
	"\002hostError"	\
	"\001interruptLatch"

/* DmaCtrl, offset 20 */
#define	DC_dnCmplReq		0x00000002	/* ro */
#define	DC_dnStalled		0x00000004	/* ro */
#define	DC_upComplete		0x00000008	/* ro */
#define	DC_dnComplete		0x00000010	/* ro */
#define	DC_upRxEarlyEnable	0x00000020	/* rw */
#define	DC_armCountdown		0x00000040	/* ro */
#define	DC_dnInProg		0x00000080	/* ro */
#define	DC_counterSpeed		0x00000100	/* rw 0:3.2uS, 1:320nS */
#define	DC_countdownMode	0x00000200	/* rw 0:from now, 1:from intr*/
#define	DC_upAltSeqDisable	0x00010000	/* rw 90xB */
#define	DC_defeatMWI		0x00100000	/* rw 90xB */
#define	DC_defeatMRL		0x00200000	/* rw 90xB */
#define	DC_upOverDiscDisable	0x00400000	/* rw 90xB */
#define	DC_targetAbort		0x40000000	/* ro 90xB */
#define	DC_masterAbort		0x80000000	/* ro 90xB */

/* DnBurstThreth, offset 2a */
#define	DnBurstThreshMask	0x1f	/* 90xB/C */
#define	DnBurstThreshUnit	32	/* 90xB/C */

/* DnPriorityThresh, offset 2c */
#define	DnPriorityThreshMask	0x3f	/* 90xB/C */
#define	DnPriorityThreshUnit	32	/* 90xB/C */

/* DnPoll, offset 2d */
#define	DnPollMask		0x7f	/* 90xB/C */

/* TxFreeThresh, offset 2f */
#define	TxFreeThreshMask	0xff
#define	TxFreeThreshUnit	256	/* 90x only */

/* UpPriorityThresh, offset 3c */
#define	UpPriorityThreshMask	0x1f	/* 90xB/C */
#define	UpPriorityThreshUnit	32	/* 90xB/C */

/* UpBurstThreth, offset 3e */
#define	UpBurstThreshMask	0x1f	/* 90xB/C */
#define	UpBurstThreshUnit	32	/* 90xB/C */

/* DnMaxBurst, offset 78 0x90XB/C */
#define	DnMaxBurstMin	0x0020
#define	DnMaxBurstMax	0x07e0

/*
 * Window 0
 */
/* EepromCommand register (offset 0xa) */
#define	EC_eepromAddress6(n)	(((n) & 0x3f) | (((n) << 2) & 0x300))
#define		EE_WriteDisable		0
#define		EE_WriteAll		1
#define		EE_EraseAll		2
#define		EE_WriteEnable		3
#define		EE_WriteRegister	(1 << 2)
#define		EE_ReadRegister		(2 << 2)
#define		EE_EraseRegister	(3 << 2)
#define	EC_eepromAddress8(n)	((n) & 0xff) /* for 3c575 cardbus products */
#define	EC_eepromBusy		0x8000

/* EEPROM data structure 	offset */
#define	EO_NodeAddress		0x00
#define	EO_DeviceID		0x03
#define	EO_Date			0x04
#define	EO_Division		0x05
#define	EO_ProductCode		0x05
#define	EO_ManufactureID	0x07
#define	EO_PciParm		0x08
#define	EO_RomInfo		0x09
#define	EO_OEMNodeAddress	0x0a
#define	EO_SoftwareIntormation	0x0d
#define	EO_CompatibilityWord	0x0e
#define	EO_SoftwareIntormation2	0x0f
#define	EO_CapabilitiesWord	0x10
#define	EO_InternalConfigWord0	0x12
#define	EO_InternalConfigWord1	0x13
#define	EO_AnalogGiagnostic	0x14
#define	EO_SoftwareIntormation3	0x15
#define	EO_LANWorksData		0x16
#define	EO_ChecksumA		0x17
#define	EO_SubsystemVendorId	0x17
#define	EO_SubsystemId		0x18
#define	EO_MediaOptionB		0x19
#define	EO_ChecksumB		0x20
/* for 3C575 */
#define	EepromOffset_CB		0x30

/*
 * Window 1
 */
/* RxError register (offset 4, byte) */
#define	RE_rxOverrun		0x01
#define	RE_runtFrame		0x02
#define	RE_alignmentError	0x04
#define	RE_crCError		0x08
#define	RE_oversizedFrame	0x10
#define	RE_dribbleBits		0x80

/* RxStatus register (offset 8) */
#define	RS_rxBytes		0x1fff
#define	RS_rxError		0x4000
#define	RS_rxIncomplete		0x8000

#define	RxStatusBits	"\020\020Incomplete\017Error"

/*
 * Window 2
 */
/* reset option register (offset c)  90XB/C */
#define	RO_featureSet		0x0007	
#define	RO_d3ResetDisable	0x0008			/* only 90xB */
#define	RO_smBusDisable		RO_d3ResetDisable	/* only 90xC */
#define	RO_disableAdvFD		0x0010			/* only 90xB */
#define	RO_smBusMode		RO_disableAdvFD		/* only 90xC */
#define	RO_invertLEDPwr		RO_disableAdvFD		/* only cardbus */
#define	RO_disableAdv100	0x0020
#define	RO_disableAutoNego	0x0040			/* only 90xB */
#define	RO_ee16KInstalled	RO_disableAutoNego	/* only 90xC */
#define	RO_debugMode		0x0080	/* rw */
#define	RO_fastAutoNeg		0x0100	/* ro */
#define	RO_fastEE		0x0200	/* ro */
#define	RO_forcedConfig		0x0400	/* rw */
#define	RO_testPdtPdr		0x0800	/* ro only 90xB */
#define	RO_fastReset		RO_testPdtPdr	/* ro only 90xC */
#define	RO_test100Tx		0x1000
#define	RO_test100Rx		0x2000
#define	RO_invertMIIPwr		0x4000	

#define	ResetOptionsBits	"\020"	\
	"\016test100Rx"		\
	"\015test100Tx"		\
	"\014testPdtPdr"	\
	"\013forcedConfig"	\
	"\012fastEE"		\
	"\011fastAutoNeg"	\
	"\010debugMode"		\
	"\007disableAutoNego"	\
	"\006disableAdv100"	\
	"\005disableAdvFD"	\
	"\004d3ResetDisable"

#define	ResetOptionsBits_C	"\020"	\
	"\016test100Rx"		\
	"\015test100Tx"		\
	"\014fastReset"		\
	"\013forcedConfig"	\
	"\012fastEE"		\
	"\011fastAutoNeg"	\
	"\010debugMode"		\
	"\007ee16KInstalled"	\
	"\006disableAdv100"	\
	"\005smBusMode"		\
	"\004smBusDisable"

#define	ResetOptionsBits_575	"\020"	\
	"\017invertLEDPwr"		\
	"\016test100Rx"		\
	"\015test100Tx"		\
	"\014fastReset"		\
	"\013forcedConfig"	\
	"\012fastEE"		\
	"\011fastAutoNeg"	\
	"\010debugMode"		\
	"\007ee16KInstalled"	\
	"\006disableAdv100"	\
	"\005inverMIIPwr"		\
	"\004smBusDisable"

/* TxStatus register */
#define	TS_txReclaimError	0x02	/* 90xB */
#define	TS_txStatusOverflow	0x04	/* TX status overflow */
#define	TS_maxCollisions	0x08	/* Maximum Collitions */
#define	TS_txUnderrun		0x10	/* Underrun error, TX reset required */
#define	TS_txJabber		0x20	/* Jabber error */
#define	TS_txInterruptRequested	0x40	/* txIndicated */
#define	TS_txComplete		0x80	/* TX is complete */

#define	TxStatusBits	\
	"\020"	\
	"\010txComplete"	\
	"\007txInterruptRequested"	\
	"\006txJabber"	\
	"\005txUnderrun"	\
	"\004maxCollisions"	\
	"\003txStatusOverflow"	\
	"\002txReclaimError"

/*
 * Window 3
 */
/* Internal Configuration (offset 0, dword) */
#define	IC_ramSize		0x00000007U	/* 90x */
#define		IC_ramSizeShift		0
#define		IC_ramSize8KB		(0U << ramSizeShift)
#define		IC_ramSize32KB		(2U << ramSizeShift)
#define		IC_ramSize64KB		(3U << ramSizeShift)
#define		IC_ramSize128KB		(4U << ramSizeShift)
#define	IC_ramWidth		0x00000008
#define	IC_romSize		0x000000c0
#define		IC_romSizeShift		6
#define		IC_romSize8KB_A		(0U << romSizeShift)
#define		IC_romSize16KB_A	(1U << romSizeShift)
#define		IC_romSize32KB_A	(2U << romSizeShift)
#define		IC_romSize64KB_A	(3U << romSizeShift)
#define		IC_romSize64KB_B	(0U << romSizeShift)
#define		IC_romSize128KB_B	(1U << romSizeShift)
#define	IC_disableBadSsdDetect	0x00000100U
#define	IC_ramLocation		0x00000200U
#define	IC_enableTxLarge	0x00004000U
#define	IC_ramPartition		0x00030000U
#define		IC_ramPartitionShift	16
#define		IC_ramPartition53		(0U << ramPartitionShift)
#define		IC_ramPartition31		(1U << ramPartitionShift)
#define		IC_ramPartition11		(2U << ramPartitionShift)
#define		IC_ramPartition35		(3U << ramPartitionShift)

#define	IC_xcvrSelect		0x00f00000U
#define		IC_xcvrSelectShift	20
#define		xcvrSelect10BASET	0U
#define		xcvrSelectAUI		1U
#define		xcvrSelect10BASE2	3U
#define		xcvrSelect100BASETX	4U
#define		xcvrSelect100BASEFX	5U
#define		xcvrSelectMII		6U
#define		xcvrSelectAN		8U	/* 10baseT */
#define		xcvrSelectMIIExt	9U
#define	IC_autoSelect		0x01000000U
#define	IC_disableBiosROM	0x02000000U

#define	InternalConfigBits	\
	"\020"	\
	"\032disableBiosROM"	\
	"\031autoSelect	"	\
	"\017enableTxLarge"	\
	"\012ramLocation"	\
	"\011disableBadSsdDetect"	\
	"\010ramWidth"

/* MacControl register (offset 6) */
#define	MC_deferExtendEnable	0x0001
#define	MC_deferTimerSelect	0x001e
#define		MC_deferTimerSelectShift	1
#define		MC_deferTimer(T)	\
		  ((((T) < 32) ? (T)/16 : ((T)/32 + 1)) << MC_deferTimerShift)
#define	MC_fullDuplexEnable	0x0020
#define	MC_allowLargePackets	0x0040
#define	MC_extendAfterCollision	0x0080
#define	MC_flowControlEnable	0x0100
#define	MC_vltEnable		0x0200	/* vlan tagging for 90xB */
#define	MC_vlanOversizedEn	0x0400	/* 1:w/o oversized flag on rx 90xC */


/* ResetOptions register (offset 8) for 90x */
/* MediaOptions register (offset 8) for 3C90xB/C */
/* XXX - the register name was changed, we describe with the new name */
#define	MO_baseT4Available	0x0001	/* ro */
#define	MO_baseTxAvailable	0x0002	/* ro */
#define	MO_baseFxAvailable	0x0004	/* ro */
#define	MO_10bTAvailable	0x0008	/* ro */
#define	MO_coaxAvailable	0x0010	/* ro */
#define	MO_auiAvailable		0x0020	/* ro */
#define	MO_miiConnector		0x0040	/* ro, 90x */
#define	MO_miiDevice		MO_miiConnector	/* ro, 90xB/C */
#define	MO_vcoConfig		0x0100	/* 0:external, 1:intenal, 90x */
#define	MO_10BaseFL		MO_vcoConfig
#define	MO_forcedConfig		0x1000	/* rw 90x */
#define	MO_testMode		0xe000	/* rw 90x */
#define		MO_testModeShift	13

#define	MediaOptionsBits	"\020"	\
	"\001baseT4Available"	\
	"\002baseTxAvailable"	\
	"\003baseFxAvailable"	\
	"\004\0610bTAvailable"	\
	"\005coaxAvailable"	\
	"\006auiAvailable"	\
	"\007miiConnector"	\
	"\011vcoConfig"		\
	"\015forcedConfig"

#define	MediaOptionsBits_BC	"\020"	\
	"\001baseT4Available"	\
	"\002baseTxAvailable"	\
	"\003baseFxAvailable"	\
	"\004\0610bTAvailable"	\
	"\005coaxAvailable"	\
	"\006auiAvailable"	\
	"\007miiDevice"		\
	"\011\0610BaseFL"

/* RxFree register (offset a, word) */

/* TxFree register, offset 0xc */
#define	TxFreeMask		0x0fff	/* ro */
/*
 * Window 4
 */
/* FifoDiagnostic register (offset 4, word) */
#define	FD_rxTestMode		0x0001
#define	FD_txTestMode		0x0002
#define	FD_inMemBIST		0x0004
#define	FD_inMemBFC		0x0008
#define	FD_inMemBF		0x0010
#define	FD_txOverrun		0x0400
#define	FD_rxOverrun		0x0800
#define	FD_rxUnderrun		0x2000
#define	FD_receiving		0x8000	/* ro */

/* NetworkDiagnostic register (offset 6, word) */
#define	ND_testLowVoltageDetector	0x0001
#define	ND_asicRevesion		0x003e
#define		ND_asicRevesionShift	1
#define	ND_upperBytesEnable	0x0040
#define	ND_statisticsEnabled	0x0080
#define	ND_txFatalError		0x0100
#define	ND_transmitting		0x0200	/* ro */
#define	ND_rxEnabled		0x0400
#define	ND_txEnabled		0x0800
#define	ND_fifoLoopback		0x2000
#define	ND_endecLoopback	0x4000
#define	ND_externalLoopback	0x8000

/* PhysicalMgmt register (offset 8, word) */
#define	PM_mgmtClk		0x0001
#define	PM_mgmtData		0x0002
#define		PM_mgmtDataShift	1
#define	PM_mgmtDir		0x0004	/* 0:in 1:out */
#define	PM_cat5LinkTestDefeat	0x8000

/* MediaStatus register (offset a) */
#define	MS_crcStripDisable	0x0004
#define	MS_enableSqeStats	0x0008
#define	MS_collisionDetect	0x0010
#define	MS_jabberGuardEnable	0x0040
#define	MS_linkBeatEnale	0x0080
#define	MS_jabberDetect		0x0200
#define	MS_polarityReversed	0x0400
#define	MS_linkDetect		0x0800
#define	MS_dcConverterEnabled	0x4000
#define	MS_auiDisabled		0x8000
#define	MS_txInProg		0x1000

#define	MediaStatusBits	"\020"	\
	"\003crcStripDisable"	\
	"\004enableSqeStats"	\
	"\005collisionDetect"	\
	"\007jabberGuardEnable"	\
	"\010linkBeatEnale"	\
	"\012jabberDetect"	\
	"\013polarityReversed"	\
	"\014linkDetect"	\
	"\017dcConverterEnabled"	\
	"\020auiDisabled"

#define	MS_txInProg		0x1000
/* UpperByteOk register (offset d, byte) */
#define	UB_upperByteRcvdOk	0x0f
#define		UB_upperByteRcvdOkShift	0
#define	UB_upperByteXmittedOk	0xf0
#define		UB_upperByteXmittedOkShift	4

/*
 * Window 5
 */
/* RxFilter register (offset 8, byte) */
#define	RF_receiveIndividual	0x01
#define	RF_receiveMulticast	0x02
#define	RF_receiveBroadcast	0x04
#define	RF_receiveAllFrames	0x08
#define	RF_receiveMulticastHash	0x10	/* 90xB */

/* InterruptEnable register (offset a, word, ro) */
/* IndicationEnable register (offset c, word, ro) */

/*
 * Window 6
 */
/* UpperFramesOk register (offset 9) */
#define	UF_upperFramesRcvdOk	0x03
#define		UF_upperFramesRcvdOkShift	0
#define	UF_upperFramesXmittedOk	0x30
#define		UF_upperFramesXmittedOkShift	4

/*
 * Download (tx) and Upload (rx) descriptor structure
 */
/* Download Fragment */
struct DnFrag {
	volatile uint32_t	DnFragAddr;
	volatile uint32_t	DnFragLen;
};
#define	DF_dnFragLen	0x00001fff
#define	DF_dnFragLast	0x80000000

/* Download Header */
struct DPD_type0 {
	volatile uint32_t	DnNextPtr;
	volatile uint32_t	FrameStartHeader;
	struct DnFrag		frags[63];
};

struct DPD_type1 {
	volatile uint32_t	DnNextPtr;
	volatile uint32_t	ScheduleTime;
	volatile uint32_t	FrameStartHeader;
	struct DnFrag		frags[63];
};

/* FrameStartHeader for 90x */
#define	FS_txLength		0x00001fff
#define	FS_crcAppendDisable	0x00002000
#define	FS_txIndicate		0x00008000
#define	FS_dnIndicate		0x80000000

/* FrameStartHeader for 90xB */
#define	FS_mdupBndy		0x00000003
#define		FS_mdupBndyDWord	0x00000000
#define		FS_mdupBndyNone		0x00000001
#define		FS_mdupBndyWord		0x00000002
#define	FS_pktId		0x000000fc
#define		FS_pktIdShift		2
#define	FS_crcAppendDisable_B	0x00002000
#define	FS_txIndicate_B		0x00008000
#define	FS_dnComplete		0x00010000
#define	FS_addIpChecksum	0x02000000
#define	FS_addTcpChecksum	0x04000000
#define	FS_addUdpChecksum	0x08000000
#define	FS_rndupDefeat		0x10000000
#define	FS_dndEmpty		0x20000000
#define	FS_dnIndicate		0x80000000

/* ScheduleTime for 90xB only */
#define	ST_scheduleTime		0x00ffffff
#define	ST_loadTimeCnt		0x10000000
#define	ST_scheduleTimeValid	0x20000000

/* Upload Fragment */
struct UpFrag {
	volatile uint32_t	UpFragAddr;
	volatile uint32_t	UpFragLen;
};
#define	UF_upFragLen	0x00001fff
#define	UF_upFragLast	0x80000000

/* Upload Header */
struct UPD {
	volatile uint32_t	UpNextPtr;
	volatile uint32_t	UpPktStatus;
	struct UpFrag		frags[63];
};

/* bits in UpPktStatus */
#define	UPS_upPktLen		0x00001fff
#define	UPS_upError		0x00004000
#define	UPS_upComplete		0x00008000
#define	UPS_upOverrun		0x00010000
#define	UPS_runtFrame		0x00020000
#define	UPS_alignmentError	0x00040000
#define	UPS_crcError		0x00080000
#define	UPS_oversizedFrame	0x00100000
#define	UPS_dribbleBits		0x00800000
#define	UPS_upOverflow		0x01000000
#define	UPS_ipChecksumError	0x02000000
#define	UPS_tcpChecksumError	0x04000000
#define	UPS_udpChecksumError	0x08000000
#define	UPS_impliedBufferEnable	0x10000000
#define	UPS_ipChecksumChecked	0x20000000
#define	UPS_tcpChecksumChecked	0x40000000
#define	UPS_udpChecksumChecked	0x80000000

#define	UpPktStatusBits	\
	"\020"	\
	"\040udpChecksumChecked"	\
	"\037tcpChecksumChecked"	\
	"\036ipChecksumChecked"	\
	"\035impliedBufferEnable"	\
	"\034udpChecksumError"	\
	"\033tcpChecksumError"	\
	"\032ipChecksumError"	\
	"\031upOverflow"	\
	"\030dribbleBits"	\
	"\025oversizedFrame"	\
	"\024crcError"	\
	"\023alignmentError"	\
	"\022runtFrame"	\
	"\021upOverrun"	\
	"\020upComplete"	\
	"\017upError"


