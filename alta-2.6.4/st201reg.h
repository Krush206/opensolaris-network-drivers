/*
 * @(#)st201reg.h	1.5 08/02/03
 * Macro definition of Sundance Technology ST201 Fast Ethernet MAC.
 * Based on Sundance Technology ST201 Fast Ethernet MAC Data sheet
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */
#pragma	ident	"@(#)st201reg.h 1.5     08/02/03"

#ifndef _ST201REG_H_
#define	_ST201REG_H_
/*
 * Known PCI vendor-id/device-id of Sundance ST201 chips
 */
#define	PCI_VID_DLINK		0x1186
#define	PCI_DID_DFE550		0x1002	/* DL10050A */
#define	PCI_VID_SUNDANCE	0x13f0
#define	PCI_DID_ST201		0x0201 /* IC Plus Corp IP100 */
#define	PCI_DID_IP100A		0x0200 /* IC Plus Corp IP100A */

/*
 * Sundance ST201 register definitions
 */
/* Offsets to the device registers. */
#define	DMACtrl			0x00
#define	TxDMAListPtr		0x04	/* long */
#define	TxDMABurstThresh	0x08
#define	TxDMAUrgentThresh	0x09
#define	TxDMAPollPeriod		0x0a
#define	RxDMAStatus		0x0c
#define	RxDMAListPtr		0x10	/* long */
#define	RxDMABurstThresh	0x14
#define	RxDMAUrgentThresh	0x15
#define	RxDMAPollPeriod		0x16
#define	Countdown		0x18	/* documentation bug, it was 0x48 */
#define	DebugCtrl		0x1a
#define	DebugCtrl1		0x1c	/* rev 0x14 or later */
#define	AsicCtrl		0x30	/* long */
#define	EepromData		0x34	/* word */
#define	EepromCtrl		0x36
#define	FIFOCtrl		0x3a
#define	TxStartThresh		0x3c
#define	RxEarlyThresh		0x3e
#define	ExpRomAddr		0x40
#define	ExpRomData		0x44
#define	WakeEvent		0x45
#define	TxStatus		0x46	/* word */
#define	IntStatusAck		0x4a	/* word */
#define	IntEnable		0x4c	/* word */
#define	IntStatus		0x4e	/* word */
#define	MACCtrl0		0x50	/* word */
#define	MACCtrl1		0x52	/* word */
#define	StationAddress		0x54	/* word[3] */
#define	MaxFrameSize		0x5a
#define	ReceiveMode		0x5c	/* byte */
#define	TxReleaseThresh		0x5d	/* byte */
#define	PhyCtrl			0x5e	/* byte */
#define	HashTable		0x60	/* word[4] */

/* statistics registers */
#define	OctetsRecevedOK		0x68
#define	OctetsTransmittedOK	0x6c
#define	FramesTransmittedOK	0x70
#define	FramesReceivedOK	0x72
#define	CarrierSenseErrors	0x74
#define	LateCollisionss		0x75
#define	MultipleColFrames	0x76
#define	SingleColFrames		0x77
#define	FramesWDeferedXmt	0x78
#define	FramesLostRxErrors	0x79
#define	FramesWEXDeferral	0x7a
#define	FramesAbortXSClls	0x7b
#define	BcstFramesXmtdOK	0x7c
#define	BcstFramesRcvdOK	0x7d
#define	McstFramesXmtdOK	0x7e
#define	McstFramesRcvdOK	0x7f

/* Rx Fragment descriptor */
/* offset 0 : RxDMANextPtr */
/* offset 4 : RxFrameStatus */
#define	RFS_RxDMAFrameLen	0x00001fffU
#define	RFS_RxFrameError	0x00004000U
#define	RFS_RxDMAComplete	0x00008000U
#define	RFS_RxFIFOOverrun	0x00010000U
#define	RFS_RxRuntFrame		0x00020000U
#define	RFS_RxAlignmentError	0x00040000U
#define	RFS_RxFCSError		0x00080000U
#define	RFS_RxOversizedFrame	0x00100000U
#define	RFS_DribbleBits		0x00800000U
#define	RFS_RxDMAOverflow	0x01000000U
#define	RFS_ImpliedBufferEnable	0x10000000U

#define	RFS_ERRORS	\
	(RFS_RxFIFOOverrun | RFS_RxRuntFrame | RFS_RxAlignmentError |	\
	RFS_RxFCSError | RFS_RxOversizedFrame | RFS_RxDMAOverflow)

#define	RFS_BITS	\
	"\020"	\
	"\017RxFrameError"	\
	"\020RxDMAComplete"	\
	"\021RxFIFOOverrun"	\
	"\022RxRuntFrame"	\
	"\023RxAlignmentError"	\
	"\024RxFCSError"	\
	"\025RxOversizedFrame"	\
	"\030DribbleBits"	\
	"\031RxDMAOverflow"	\
	"\035ImpliedBufferEnable"

/* offset n*8+0 : RxDMAFragAddr */
/* offset n*8+4 : RxDMAFragLen */
#define	RFD_RxDMAFragLen	0x00001fffU
#define	RFD_RxDMALastFrag	0x80000000U

/* TFD */
/* offset 0 : TxDMANextPtr */
/* offset 4 : TxDMAFragControl */
#define	TFC_WordAlign		0x00000003U
#define	TFC_WordAlignDword	0x00000000U
#define	TFC_WordAlignWord	0x00000002U
#define	TFC_WordAlignDisable	0x00000001U
#define	TFC_FrameId		0x000003fcU
#define		TFC_FrameIdShift		2
#define	TFC_FcsAppendDisable	0x00002000U
#define	TFC_TxIndicate		0x00008000U
#define	TFC_TxDMAComplete	0x00010000U
#define	TFC_TxDMAIndicate	0x80000000U

#define	TFC_BITS	\
	"\020"	\
	"\040TFC_TxDMAIndicate"	\
	"\021TFC_TxDMAComplete"	\
	"\020TFC_TxIndicate"	\
	"\016TFC_FcsAppendDisable"

/* offset n*8+0 : TxDMAFragAddr */
/* offset n*8+4 : TxDMAFragLen */
#define	TFD_TxDMAFragLen	0x00001fffU
#define	TFD_TxDMAFragLast	0x80000000U

/* offset 0x00 : DMACtrl long */
#define	DC_RxDMAHalted		0x00000001U	/* R/- */
#define	DC_TxDMACmplReq		0x00000002U	/* R/- */
#define	DC_TxDMAHalted		0x00000004U	/* R/- */
#define	DC_RxDMAComplete	0x00000008U	/* R/- */
#define	DC_TxDMAComplete	0x00000010U	/* R/- */
#define	DC_RxDMAHalt		0x00000100U	/* -/Wsc */
#define	DC_RxDMAResume		0x00000200U	/* -/Wsc */
#define	DC_TxDMAHalt		0x00000400U	/* -/Wsc */
#define	DC_TxDMAResume		0x00000800U	/* -/Wsc */
#define	DC_TxDMAInProg		0x00004000U	/* R/- */
#define	DC_DMAHaltBusy		0x00008000U	/* R/- */
#define	DC_RxEarlyEnable	0x00020000U	/* R/W */
#define	DC_CountdownSpeed	0x00040000U	/* R/W */
#define	DC_CountdownMode	0x00080000U	/* R/W */
#define	DC_MWIDisable		0x00100000U	/* R/W */
#define	DC_RxDMAOverrunFrame	0x00400000U	/* R/W */
#define	DC_CountdownIntEnable	0x00800000U	/* R/- */
#define	DC_TargetAbort		0x40000000U	/* R/- */
#define	DC_MasterAbort		0x80000000U	/* R/- */

#define	DC_BITS	\
	"\020"	\
	"\001RxDMAHalted"	\
	"\002TxDMACmplReq"	\
	"\003TxDMAHalted"	\
	"\004RxDMAComplete"	\
	"\005TxDMAComplete"	\
	"\017TxDMAInProg"	\
	"\020DMAHaltBusy"	\
	"\022RxEarlyEnable"	\
	"\023CountdownSpeed"	\
	"\024CountdownMode"	\
	"\025MWIDisable"	\
	"\027RxDMAOverrunFrame"	\
	"\030CountdownIntEnable"	\
	"\037TargetAbort"	\
	"\040MasterAbort"

/* offset 0x04 : TxDMAListPtr long */
/* offset 0x08 : TxDMABurstThresh byte */
#define	TxDMABurstThreshUnit	32U
#define	TxDMABurstThreshMax	0x1fU

/* offset 0x09 : TxDMAUrgentThresh byte */
#define	TxDMAUrgentThreshUnit	32U
#define	TxDMAUrgentThreshMax	0x3fU

/* offset 0x14 : RxDMAUrgentThresh byte */
#define	RxDMAUrgentThreshUnit	32U
#define	RxDMAUrgentThreshMax	0x1fU

/* offset 0x0a : TxDMAPollPeriod byte */
#define	TxDMAPollPeriodUnit	320U		/* in nano second */
#define	TxDMAPollPeriodMax	0x7fU

/* offset 0x0c : RxDMAStatus long */
/* offset 0x10 : RxDMAListPtr long */
/* offset 0x14 : RxDMABurstThresh byte */
#define	RxDMABurstThreshUnit	32U
#define	RxDMABurstThreshMax	0x1fU

/* offset 0x14 : RxDMAUrgentThresh byte */
#define	RxDMAUrgentThreshUnit	32U
#define	RxDMAUrgentThreshMax	0x1fU

/* offset 0x16 : RxDMAPollPeriod byte */
#define	RxDMAPollPeriodUnit	320U		/* in nano second */
#define	RxDMAPollPeriodMax	0x7fU

/* offset 0x1a : DebugCntl word */
#define	DC_GPIO0Ctrl		0x0001U
#define	DC_GPIO1Ctrl		0x0002U
#define	DC_GPIO0		0x0004U
#define	DC_GPIO1		0x0008U

/* offset 0x30 : AsicCtrl long */
#define	AC_ExpRomSize		0x00000002U	/* R/W 0=32kB 1=64kB */
#define	AC_TxLargeEnable	0x00000004U	/* R/W */
#define	AC_RxLargeEnable	0x00000008U	/* R/W */
#define	AC_ExpRomDisable	0x00000010U	/* R/W */
#define	AC_PhySpeed10		0x00000020U	/* R/- */
#define	AC_PhySpeed100		0x00000040U	/* R/- */
#define	AC_PhyMedia		0x00000080U	/* R/- */
#define	AC_ForcedConfig		0x00000700U	/* R/W */
#define	 AC_ForcedConfigShift		8
#define	 AC_ForcedConfigNone	(0U << AC_ForcedConfigShift)
#define	 AC_ForcedConfigMode1	(1U << AC_ForcedConfigShift)
#define	AC_D3ResetDisable	0x00000800U	/* R/W */
#define	AC_SpeedupMode		0x00002000U	/* R/W */
#define	AC_LEDMode		0x00004000U	/* R/W */
#define	AC_RstOutPolarity	0x00008000U	/* R/W */
#define	AC_GlobalReset		0x00010000U	/* -/Wsc */
#define	AC_RxReset		0x00020000U	/* -/Wsc */
#define	AC_TxReset		0x00040000U	/* -/Wsc */
#define	AC_DMA			0x00080000U	/* -/Wsc */
#define	AC_FIFO			0x00100000U	/* -/Wsc */
#define	AC_Network		0x00200000U	/* -/Wsc */
#define	AC_Host			0x00400000U	/* -/Wsc */
#define	AC_AutoInit		0x00800000U	/* -/Wsc */
#define	AC_RstOut		0x01000000U	/* -/Wsc */
#define	AC_InterruptRequest	0x02000000U	/* -/Wsc */
#define	AC_ResetBusy		0x04000000U	/* R/- */

#define	AC_BITS	\
	"\020"	\
	"\002ExpRomSize"	\
	"\003TxLargeEnable"	\
	"\004RxLargeEnable"	\
	"\005ExpRomDisable"	\
	"\006PhySpeed10"	\
	"\007PhySpeed100"	\
	"\010PhyMedia"	\
	"\011ForcedConfig1"	\
	"\014D3ResetDisable"	\
	"\016SpeedupMode"	\
	"\017LEDMode"	\
	"\020RstOutPolarity"	\
	"\033ResetBusy"

/* offset 0x34 : EepromData word */
/* offset 0x36 : EepromCtrl word */
#define	EEC_Address		0x00ffU
#define	EEC_Opcode		0x0300U
#define	 EEC_OpcodeShift	8
#define	 EEC_OpcodeWr		(1U << EEC_OpcodeShift)
#define	 EEC_OpcodeRd		(2U << EEC_OpcodeShift)
#define	 EEC_OpcodeEr		(3U << EEC_OpcodeShift)
#define	 EEC_OpcodeWrDis	(0U << 6)
#define	 EEC_OpcodeWrAll	(1U << 6)
#define	 EEC_OpcodeErAll	(2U << 6)
#define	 EEC_OpcodeWrEn		(3U << 6)
#define	EEC_Busy		0x8000U

/* offset 0x3a : FIFOCtrl word */
#define	FC_RAMTestMode		0x0001U	/* R/W */
#define	FC_RxOverrunFrame	0x0200U	/* R/W */
#define	FC_RxFIFOFull		0x0800U	/* R/- */
#define	FC_Transmitting		0x4000U	/* R/- */
#define	FC_Receiving		0x8000U	/* R/- */

/* offset 0x3c : TxStartThresh word */
#define	TxStartThreshUnit	4
#define	TxStartThreshMax	0x1fffU

/* offset 0x3e : RxEarlyThresh word */
#define	RxEarlyThreshUnit	4
#define	RxEarlyThreshDisable	0x1ffcU

/* offset 0x40 : ExpRomAddr long */
/* offset 0x44 : ExpRomData word */
/* offset 0x45 : WakeEvent byte */
#define	WE_WakePktEnable	0x01U	/* R/W */
#define	WE_MagicPktEnable	0x02U	/* R/W */
#define	WE_LinkEventEnable	0x04U	/* R/W */
#define	WE_WakePolarity		0x08U	/* R/W */
#define	WE_WakePktEvent		0x10U	/* -/W */
#define	WE_MagicPktEvent	0x20U	/* -/W */
#define	WE_LinkEvent		0x40U	/* -/W */
#define	WE_WakeOnLanEnable	0x80U	/* R/W */

/* offset 0x46 : TxStatus word */
#define	TS_TxReleaseError	0x0002U
#define	TS_TxStatusOverflow	0x0004U
#define	TS_MaxCollisions	0x0008U
#define	TS_TxUnderrun		0x0010U
#define	TS_TxIndicateReqd	0x0040U
#define	TS_TxComplete		0x0080U
#define	TS_TxFrameId		0xff00U
#define	TS_TxFrameIdShift	8

#define	TS_BITS	\
	"\020"	\
	"\002TxReleaseError"	\
	"\003TxStatusOverflow"	\
	"\004MaxCollisions"	\
	"\005TxUnderrun"	\
	"\007TxIndicateReqd"	\
	"\010TxComplete"

/* offset 0x48 : Countdown word */
/* offset 0x4a : IntStatusAck word */
/* offset 0x4c : IntEnable word */
/* offset 0x4e : IntStatus word */
#define	INT_InterruptStatus	0x0001U
#define	INT_HostError		0x0002U
#define	INT_TxComplete		0x0004U
#define	INT_MACControlFrame	0x0008U
#define	INT_RxComplete		0x0010U
#define	INT_RxEarly		0x0020U
#define	INT_IntRequest		0x0040U
#define	INT_UpdateStats		0x0080U
#define	INT_LinkEvent		0x0100U
#define	INT_TxDMAComplete	0x0200U
#define	INT_RxDMAComplete	0x0400U

#define	INT_BITS	\
	"\020"	\
	"\002HostError"	\
	"\003TxComplete"	\
	"\004MACControlFrame"	\
	"\005RxComplete"	\
	"\006RxEarly"	\
	"\007IntRequest"	\
	"\010UpdateStats"	\
	"\011LinkEvent"	\
	"\012TxDMAComplete"	\
	"\013RxDMAComplete"


/* offset 0x50 : MACCtrl0 word */
#define	MC0_IFSSelect		0x0003U	/* R/W */
#define		MC0_IFSSelect802_3	0x0000U
#define		MC0_IFSSelect128	0x0001U
#define		MC0_IFSSelect224	0x0002U
#define		MC0_IFSSelect544	0x0003U
#define	MC0_FullDuplexEnable	0x0020U	/* R/W */
#define	MC0_RcvLargeFrames	0x0040U	/* R/W */
#define	MC0_FlowControlEnable	0x0100U	/* R/W */
#define	MC0_RcvFCS		0x0200U	/* R/W */
#define	MC0_FIFOLoopback	0x0400U	/* R/W */
#define	MC0_MACLoopback		0x0800U	/* R/W */

#define	MC0_BITS	\
	"\020"	\
	"\006FullDuplexEnable"	\
	"\007RcvLargeFrames"	\
	"\011FlowControlEnable"	\
	"\012RcvFCS"	\
	"\013FIFOLoopback"	\
	"\014MACLoopback"

/* offset 0x52 : MACCtrl1 word */
#define	MC1_CollisionDetect	0x0001U	/* R/- */
#define	MC1_CarrierSense	0x0002U	/* R/- */
#define	MC1_TxInProg		0x0004U	/* R/- */
#define	MC1_TxError		0x0008U	/* R/- */
#define	MC1_StatisticsEnable	0x0020U	/* -/Wsc */
#define	MC1_StatisticsDisable	0x0040U	/* -/Wsc */
#define	MC1_StatisticsEnabled	0x0080U	/* R/- */
#define	MC1_TxEnable		0x0100U	/* -/Wsc */
#define	MC1_TxDisable		0x0200U	/* -/Wsc */
#define	MC1_TxEnabled		0x0400U	/* R/- */
#define	MC1_RxEnable		0x0800U	/* -/Wsc */
#define	MC1_RxDisable		0x1000U	/* -/Wsc */
#define	MC1_RxEnabled		0x2000U	/* R/- */
#define	MC1_Paused		0x4000U	/* R/- */

#define	MC1_BITS	\
	"\020"	\
	"\001CollisionDetect"	\
	"\002CarrierSense"	\
	"\003TxInProg"	\
	"\004TxError"	\
	"\010StatisticsEnabled"	\
	"\013TxEnabled"	\
	"\016RxEnabled"	\
	"\017Paused"

/* offset 0x54-0x58 : StationAddress[3] word */
/* offset 0x5a : MaxFrameSize word */
/* offset 0x5c : ReceiveMode byte */
#define	RM_ReceiveUnicast	0x01U	/* R/W */
#define	RM_ReceiveMulticast	0x02U	/* R/W */
#define	RM_ReceiveBroadcast	0x04U	/* R/W */
#define	RM_ReceiveAllFrames	0x08U	/* R/W */
#define	RM_ReceiveMulticastHash	0x10U	/* R/W */
#define	RM_ReceiveIPMulticast	0x20U	/* R/W */

#define	RM_BITS	\
	"\020"	\
	"\001ReceiveUnicast"	\
	"\002ReceiveMulticast"	\
	"\003ReceiveBroadcast"	\
	"\004ReceiveAllFrames"	\
	"\005ReceiveMulticastHash"	\
	"\006ReceiveIPMulticast"

/* offset 0x5d : TxReleaseThresh byte */
#define	TxReleaseThreshUnit	16
#define	TxReleaseThreshMax	0xff
#define	TxReleaseThreshDisable	TxReleaseThreshMax

/* offset 0x5e : PhyCtrl byte */
#define	PC_MgmtClk		0x01U	/* -/W */
#define	PC_MgmtData		0x02U	/* R/W */
#define		PC_MgmtDataShift	1
#define	PC_MgmtDir		0x04U	/* -/W */
#define	PC_DisableClk25		0x08U	/* ?/W */
#define	PC_PhyDuplexPolarity	0x10U	/* ?/W */
#define	PC_PhyDuplexStatus	0x20U	/* R/- */
#define	PC_PhySpeedStatus	0x40U	/* R/- */
#define	PC_PhyLinkStatus	0x80U	/* R/- */

#define	PC_BITS	\
	"\020"	\
	"\001MgmtClk"	\
	"\002MgmtData"	\
	"\003MgmtDir"	\
	"\004DisableClk25"	\
	"\005PhyDuplexPolarity"	\
	"\006PhyDuplexFull"	\
	"\007PhySpeed100"	\
	"\010PhyLinkUp"

/* offset 0x60-0x66 : Hashtable[4] word */


#endif	/* _ST201REG_H_ */
