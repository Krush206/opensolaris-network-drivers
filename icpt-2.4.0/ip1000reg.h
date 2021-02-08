/*
 * @(#)ip1000reg.h	1.2 07/02/19
 * Macro definition of IC Plus IP1000A Gigabit Ethernet MAC.
 * Based on TC9020 GbE Ethernet MAC Data sheet
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */

/*
 * Known PCI vendor-id/device-id
 */
#define	PCI_VID_DLINK		0x1186
#define	PCI_DID_DL2000		0x2000
#define	PCI_VID_SUNDANCE	0x13f0
#define	PCI_DID_TC9020		0x2020
#define	PCI_DID_IP1000A		0x1023

/*
 * TC2020 register definitions
 */
/* Offsets to the device registers. */
#define	DMACtrl			0x00	/* DW */
#define	TFDListPtr		0x10	/* QW */
#define	TxDMABurstThresh	0x18	/* B */
#define	TxDMAUrgentThresh	0x19	/* B */
#define	TxDMAPollPeriod		0x1a	/* B */
#define	RFDListPtr		0x1c	/* QW */

#define	RxDMABurstThresh	0x24	/* B */
#define	RxDMAUrgentThresh	0x25	/* B */
#define	RxDMAPollPeriod		0x26	/* B */
#define	RxDMAIntCtrl		0x28	/* DW */
#define	DebugCtrl		0x2c	/* W */

#define	AsicCtrl		0x30	/* DW */
#define	FIFOCtrl		0x38	/* W */
#define	RxEarlyThresh		0x3a	/* W */
#define	FlowOffThresh		0x3c	/* W */
#define	FlowOnThresh		0x3e	/* W */

#define	TxStartThresh		0x44	/* W */
#define	EepromData		0x48	/* W */
#define	EepromCtrl		0x4a	/* W */
#define	ExpRomAddr		0x4c	/* DW */

#define	ExpRomData		0x50	/* B */
#define	WakeEvent		0x51	/* B */
#define	Countdown		0x54	/* DW */
#define	IntStatusAck		0x5a	/* W */
#define	IntEnable		0x5c	/* W */
#define	IntStatus		0x5e	/* W */

#define	TxStatus		0x60	/* DW */
#define	MACCtrl			0x6c	/* DW */

#define	VLANTag			0x70	/* DW */
#define	PhyCtrl			0x76	/* B */
#define	StationAddress		0x78	/* B*6 */

#define	VLANId			0x80	/* DW */
#define	MaxFrameSize		0x86	/* W */
#define	ReceiveMode		0x88	/* W */
#define	VLANHashTable		0x8a	/* W */
#define	HashTable		0x8c	/* DW*2 */

#define	RMONStatisticsMask	0x98	/* DW */
#define	StatisticsMask		0x9c	/* DW */


/* Ethernet MIB Statistics registers */
#define	OctetRcvdOk		0xa8	/* DW */
#define	McstOctetRcvdOk		0xac	/* DW */
#define	BcstOctetRcvdOk		0xb0	/* DW */
#define	FramesRcvdOk		0xb4	/* DW */
#define	McstFramesRcvdOk	0xb8	/* DW */
#define	RxJumboFrames		0xbc	/* W */
#define	BcstFramesRcvdOk	0xbe	/* W */
#define	TCPCheckSumErrors	0xc0	/* W */
#define	IPCheckSumErrors	0xc2	/* W */
#define	UDPCheckSumErrors	0xc4	/* W */

#define	MacControlFramesRcvd	0xc6	/* W */
#define	FrameTooLongErrors	0xc8	/* W */
#define	InRangeLengthErrors	0xca	/* W */
#define	FramesCheckSeqErrors	0xcc	/* W */
#define	FramesLostRxErrors	0xce	/* W */

#define	OctetXmtOk		0xd0	/* DW */
#define	McstOctetXmtOk		0xd4	/* DW */
#define	BcstOctetXmtOk		0xd8	/* DW */
#define	FramesXmtdOk		0xdc	/* DW */
#define	McstFramesXmtdOk	0xe0	/* DW */
#define	FramesWDeferredXmt	0xe4	/* DW */
#define	LateCollisions		0xe8	/* DW */
#define	MultiColFrames		0xec	/* DW */
#define	SingleColFrames		0xf0	/* DW */
#define	BcstFramesXmtdOk	0xf6	/* W */
#define	CarrierSenseErrors	0xf8	/* W */
#define	MacControlFramesXmtd	0xfa	/* W */
#define	FramesAbortXSColls	0xfc	/* W */
#define	FramesWEXDeferral	0xfe	/* W */

/* RMON Statistics */
#define	ESCollisions			0x100
#define	ESOctetsTransmit		0x104
#define	ESPktsTransmit			0x108
#define	ESPkts64OctetsTransmit		0x10c
#define	ESPkts65to127OctetsTransmit	0x110
#define	ESPkts128to255OctetsTransmit	0x114
#define	ESPkts256to511OctetsTransmit	0x118
#define	ESPkts512to1023ctetsTransmit	0x11c
#define	ESPkts1024to1518ctetsTransmit	0x120
#define	ESCRCAlignErrors		0x124
#define	ESUndersizePkts			0x128
#define	ESFragments			0x12c
#define	ESJabbers			0x130
#define	ESOctets			0x134
#define	ESPkts				0x138
#define	ESPkts64Octets			0x13c
#define	ESPkts65to127Octets		0x140
#define	ESPkts128to255Octets		0x144
#define	ESPkts256to511Octets		0x148
#define	ESPkts512to1023ctets		0x14c
#define	ESPkts1024to1518ctets		0x150

/* Rx Fragment descriptor */
/* offset 0 : RxDMANextPtr */
/* offset 4 : RxFrameStatus */
#define	RFS_TCI			0xffff000000000000ULL
#define	RFS_TCIShift		48

#define	RFS_RxFrameLen		0x0000ffffULL
#define	RFS_RxFIFOOverrun	0x00010000ULL
#define	RFS_RxRuntFrame		0x00020000ULL
#define	RFS_RxAlignmentError	0x00040000ULL
#define	RFS_RxFCSError		0x00080000ULL
#define	RFS_RxOversizedFrame	0x00100000ULL
#define	RFS_RxLengthError	0x00200000ULL
#define	RFS_VLANDetected	0x00400000ULL
#define	RFS_TCPDetected		0x00800000ULL
#define	RFS_TCPError		0x01000000ULL
#define	RFS_UDPDetected		0x02000000ULL
#define	RFS_UDPError		0x04000000ULL
#define	RFS_IPDetected		0x08000000ULL
#define	RFS_IPError		0x10000000ULL
#define	RFS_FrameStart		0x20000000ULL
#define	RFS_FrameEnd		0x40000000ULL
#define	RFS_RFDDone		0x80000000ULL

#define	RFS_BITS	\
	"\020"	\
	"\021RxFIFOOverrun"	\
	"\022RxRuntFrame"	\
	"\023RxAlignmentError"	\
	"\024RxFCSError"	\
	"\025RxOversizedFrame"	\
	"\026RxLengthError"	\
	"\027VLAN_detected"	\
	"\030TCP_detected"	\
	"\031TCP_error"		\
	"\032UDP_detected"	\
	"\033UDP_error"		\
	"\034IP_detected"	\
	"\035IP_error"		\
	"\036FrameStart"	\
	"\037FrameEnd"		\
	"\040RFDDone"

/* offset 0x10 : FragInfo */
#define	RFI_FragLen		0xffff000000000000ULL
#define	RFI_FragLenShift	48
#define	RFI_FragAddr		0x000000ffffffffffULL

/* TFD */
/* offset 0 : 39..0 TxDMANextPtr */
/* offset 0 : 63..40 reserved */

/* offset 8 : TxDMAFragControl */

#define	TFC_UserPri		0x0000e00000000000ULL
#define		TFC_UserPriShift	45
#define	TFC_CFI			0x0000100000000000ULL
#define	TFC_VID			0x00000fff00000000ULL
#define		TFC_VIDShift		32
#define	TFC_VTAG		(TFC_UserPri | TFC_CFI | TFC_VID)
#define		TFC_VTAGShift		TFC_VIDShift

#define	TFC_TFDDone		0x0000000080000000ULL
#define	TFC_VLANTagInsert	0x0000000010000000ULL
#define	TFC_FragCount		0x000000000f000000ULL
#define		TFC_FragCountShift	24
#define	TFC_TxDMAIndicate	0x0000000000800000ULL
#define	TFC_TxIndicate		0x0000000000400000ULL
#define	TFC_FcsAppendDisable	0x0000000000200000ULL
#define	TFC_IPCksumEn		0x0000000000100000ULL
#define		TFC_IPCksumEnShift	20
#define	TFC_UDPCksumEn		0x0000000000080000ULL
#define		TFC_UDPCksumEnShift	19
#define	TFC_TCPCksumEn		0x0000000000040000ULL
#define		TFC_TCPCksumEnShift	18

#define	TFC_WordAlign		0x0000000000030000ULL
#define	TFC_WordAlignShift		16
#define	TFC_WordAlignDword	(0 << TFC_WordAlignShift)
#define	TFC_WordAlignWord	(2 << TFC_WordAlignShift)
#define	TFC_WordAlignDisable	(1 << TFC_WordAlignShift)

#define	TFC_FrameId		0x000000000000ffffULL

#define	TFC_BITS	\
	"\020"	\
	"\040TFDDone"	\
	"\035VLANTagInsert"	\
	"\030TxDMAIndicate"	\
	"\027TxIndicate"	\
	"\026FcsAppendDisable"	\
	"\025IP_En"	\
	"\024UDP_En"	\
	"\023TCP_En"

/* offset n*8+0 : FragInfo */
#define	TFI_FragLen		0xffff000000000000ULL
#define	TFI_FragLenShift	48
#define	TFI_FragAddr		0x000000ffffffffffULL

/* offset 0x00 : DMACtrl DW */
#define	DC_RxDMAComplete	0x00000008	/* R/- */
#define	DC_RxDMAPollNow		0x00000010	/* -/W */
#define	DC_TxDMAComplete	0x00000800	/* R/- */
#define	DC_TxDMAPollNow		0x00001000	/* -/W */
#define	DC_TxDMAInProg		0x00008000	/* R/- */
#define	DC_MWIDisable		0x00040000	/* R/W */
#define	DC_TxWriteBackDisable	0x00080000	/* R/W */
#define	DC_TxBurstLimit		0x00700000	/* R/W */
#define		DC_TxBurstLimitShift	20
#define		DC_TxBurstLimitMax	(5 << DC_TxBurstLimitShift)
#define	DC_TargetAbort		0x40000000	/* R/- */
#define	DC_MasterAbort		0x80000000	/* R/- */

#define	DC_BITS	\
	"\020"	\
	"\004RxDMAComplete"	\
	"\005RxDMAPollNow"	\
	"\014TxDMAComplete"	\
	"\015TxDMAPollNow"	\
	"\020TxDMAInProg"	\
	"\023MWIDisable"	\
	"\024TxWriteBackDis"	\
	"\037TargetAbort"	\
	"\040MasterAbort"

/* offset 0x10 : TFDListPtr DW*2 */

/* offset 0x18 : TxDMABurstThresh B */
#define	TxDMABurstThreshUnit	32
#define	TxDMABurstThreshMin	8		/* 256 byte */
#define	TxDMABurstThreshMax	0xff		/* 8160 byte */

/* offset 0x19 : TxDMAUrgentThresh byte */
#define	TxDMAUrgentThreshUnit	32
#define	TxDMAUrgentThreshMin	4		/* 128 byte */
#define	TxDMAUrgentThreshMax	0xff		/* 8160 byte */

/* offset 0x1a : TxDMAPollPeriod byte */
#define	TxDMAPollPeriodUnit	320		/* in nano second */
#define	TxDMAPollPeriodMin	1
#define	TxDMAPollPeriodMax	0xff

/* offset 0x1c : RFDListPtr DW*2 */

/* offset 0x24 : RxDMABurstThresh B */
#define	RxDMABurstThreshUnit	32
#define	RxDMABurstThreshMin	8		/* 256 byte */
#define	RxDMABurstThreshMax	0xff		/* 8160 byte */

/* offset 0x25 : RxDMAUrgentThresh byte */
#define	RxDMAUrgentThreshUnit	32
#define	RxDMAUrgentThreshMin	4		/* 128 byte */
#define	RxDMAUrgentThreshMax	0xff		/* 8160 byte */

/* offset 0x26 : RxDMAPollPeriod byte */
#define	RxDMAPollPeriodUnit	320		/* in nano second */
#define	RxDMAPollPeriodMin	1
#define	RxDMAPollPeriodMax	0xff		/* 81.92 uS */

/* offset 0x28 : RxDMAIntCtrl DW */
#define	RIC_RxFrameCount	0x000000ff
#define	RIC_PriorityThresh	0x00001c00
#define		RIC_PriorityThreshShift	10
#define		RIC_PriorityThreshMax	RIC_PriorityThresh
#define	RIC_RxDMAWaitTime	0xffff0000
#define		RIC_RxDMAWaitTimeShift	16
#define	RxDMAWaitTime_1uS	(1000/64)

/* offset 0x2c : DebugCntl W */
#define	DBG_GPIO0Ctrl		0x0001
#define	DBG_GPIO1Ctrl		0x0002
#define	DBG_GPIO0		0x0004
#define	DBG_GPIO1		0x0008
#define	DBG_dbDisableDnHalt	0x0010
#define	DBG_dbDisableUpHalt	0x0020
#define	DBG_dbWrSameFSH		0x0040
#define	DBG_dbNearEmpty		0x0080
#define	DBG_dbSyncContrDone	0x0100
#define	DBG_dbFrCurDoneAck	0x0200
#define	DBG_dbFrcSpd1000	0x0400

/* offset 0x30 : AsicCtrl DW */
#define	AC_ExpRomSize		0x00000002	/* R/W 0=32kB 1=64kB */
#define	AC_PhySpeed10		0x00000010	/* R/- */
#define	AC_PhySpeed100		0x00000020	/* R/- */
#define	AC_PhySpeed1000		0x00000040	/* R/- */
#define	AC_PhyMedia		0x00000080	/* R/- 0:twisted pair, 1:fibre*/
#define	AC_ForcedConfig		0x00000700	/* R/W */
#define		AC_ForcedConfigShift	8
#define		AC_ForcedConfigNone	(0 << AC_ForcedConfigShift)
#define		AC_ForcedConfigMode1	(1 << AC_ForcedConfigShift)
#define	AC_D3ResetDisable	0x00000800	/* R/W */
#define	AC_SpeedupMode		0x00002000	/* R/W */
#define	AC_LEDMode		0x00004000	/* R/W */
#define	AC_RstOutPolarity	0x00008000	/* R/W */
#define	AC_GlobalReset		0x00010000	/* -/Wsc */
#define	AC_RxReset		0x00020000	/* -/Wsc */
#define	AC_TxReset		0x00040000	/* -/Wsc */
#define	AC_DMA			0x00080000	/* -/Wsc */
#define	AC_FIFO			0x00100000	/* -/Wsc */
#define	AC_Network		0x00200000	/* -/Wsc */
#define	AC_Host			0x00400000	/* -/Wsc */
#define	AC_AutoInit		0x00800000	/* -/Wsc */
#define	AC_RstOut		0x01000000	/* -/Wsc */
#define	AC_InterruptRequest	0x02000000	/* -/Wsc */
#define	AC_ResetBusy		0x04000000	/* R/W */
#define	AC_LEDSpeed		0x08000000	/* R/W */
#define	AC_LEDMode1		0x20000000	/* R/W */

#define	AC_BITS	\
	"\020"	\
	"\002ExpRomSize"	\
	"\005PhySpeed10"	\
	"\006PhySpeed100"	\
	"\007PhySpeed1000"	\
	"\010PhyMedia"	\
	"\011ForcedConfig1"	\
	"\014D3ResetDisable"	\
	"\016SpeedupMode"	\
	"\017LEDMode"	\
	"\020RstOutPolarity"	\
	"\033ResetBusy"

/* offset 0x38 : FIFOCtrl W */
#define	FC_RAMTestMode		0x0001	/* R/W */
#define	FC_Transmitting		0x4000	/* R/- */
#define	FC_Receiving		0x8000	/* R/- */

/* offset 0x3a : RxEarlyThresh W */
#define	RxEarlyThreshMax		0x7ff

/* offset 0x3c : FlowOffThresh W */
/* offset 0x3e : FlowOnThresh W */
#define	FlowThreshUnit	16

/* offset 0x44 : TxStartThresh W */
#define	TxStartThreshUnit	2	/* datasheet says 4, but it seems 2 */
#define	TxStartThreshMax	0x0fff

/* offset 0x48 : EepromData W */
/* offset 0x4a : EepromCtrl W */
#define	EEC_Address		0x00ff
#define	EEC_Opcode		0x0300
#define		EEC_OpcodeShift	8
#define		EEC_OpcodeWr		(1 << EEC_OpcodeShift)
#define		EEC_OpcodeRd		(2 << EEC_OpcodeShift)
#define		EEC_OpcodeEr		(3 << EEC_OpcodeShift)
#define		EEC_OpcodeWrDis		(0 << 6)
#define		EEC_OpcodeWrAll		(1 << 6)
#define		EEC_OpcodeErAll		(2 << 6)
#define		EEC_OpcodeWrEn		(3 << 6)
#define	EEC_Busy		0x8000

/* offset 0x4c : ExpRomAddr DW */
/* offset 0x50 : ExpRomData B */
/* offset 0x51 : WakeEvent B */
#define	WE_WakePktEnable	0x01	/* R/W */
#define	WE_MagicPktEnable	0x02	/* R/W */
#define	WE_LinkEventEnable	0x04	/* R/W */
#define	WE_WakePolarity		0x08	/* R/W */
#define	WE_WakePktEvent		0x10	/* R/- */
#define	WE_MagicPktEvent	0x20	/* R/- */
#define	WE_LinkEvent		0x40	/* R/- */
#define	WE_WakeOnLanEnable	0x80	/* R/W */

/* offset 0x54 : Countdown DW */
#define	CD_Count		0x0000ffff
#define	CD_CountdownSpeed	0x01000000	/* 0:3200nS 1:320nS when 1Gbps*/
#define	CD_CountdownMode	0x02000000	/* 0:now, 1: after TxDMAcomp */
#define	CD_CountdownIntEnabled	0x04000000

/* offset 0x5a : IntStatusAck W */
/* offset 0x5c : IntEnable W */
/* offset 0x5e : IntStatus W */
#define	INT_InterruptStatus	0x0001
#define	INT_HostError		0x0002
#define	INT_TxComplete		0x0004
#define	INT_MACControlFrame	0x0008
#define	INT_RxComplete		0x0010
#define	INT_RxEARLY		0x0020
#define	INT_IntRequested	0x0040
#define	INT_UpdateStats		0x0080
#define	INT_LinkEvent		0x0100
#define	INT_TxDMAComplete	0x0200
#define	INT_RxDMAComplete	0x0400
#define	INT_RFDListEnd		0x0800
#define	INT_RxDMAPriority	0x1000

#define	INT_BITS	\
	"\020"	\
	"\001InterruptStatus"	\
	"\002HostError"	\
	"\003TxComplete"	\
	"\004MACControlFrame"	\
	"\005RxComplete"	\
	"\006RxEarly"		\
	"\007IntRequested"	\
	"\010UpdateStats"	\
	"\011LinkEvent"	\
	"\012TxDMAComplete"	\
	"\013RxDMAComplete"	\
	"\014RFDListEnd"	\
	"\015RxDMAPriority"

/* offset 0x60 : TxStatus DW */
#define	TS_TxError		0x00000001
#define	TS_LateCollision	0x00000004
#define	TS_MaxCollisions	0x00000008
#define	TS_TxUnderrun		0x00000010
#define	TS_TxIndicateReqd	0x00000040
#define	TS_TxComplete		0x00000080
#define	TS_TxFrameId		0xffff0000
#define	TS_TxFrameIdShift	16

#define	TS_BITS	\
	"\020"	\
	"\001TxError"	\
	"\003LateCollision"	\
	"\004MaxCollisions"	\
	"\005TxUnderrun"	\
	"\007TxIndicateReqd"	\
	"\010TxComplete"

/* offset 0x6c : MACCtrl DW */
#define	MC_IFSSelect		0x00000003	/* R/W */
#define		MC_IFSSelectShift		0
#define		MC_IFSSelect96		(0 << MC_IFSSelectShift)
#define		MC_IFSSelect1024	(1 << MC_IFSSelectShift)
#define		MC_IFSSelect1792	(2 << MC_IFSSelectShift)
#define		MC_IFSSelect4352	(3 << MC_IFSSelectShift)
#define		MC_IFSSelect802_3	MC_IFSSelect96	/* standard */
#define	MC_DuplexSelect		0x00000020	/* R/W 0:half, 1:full*/
#define	MC_RcvLargeFrames	0x00000040	/* R/W */	/* reserved */
#define	MC_TxFlowControlEnable	0x00000080	/* R/W */
#define	MC_RxFlowControlEnable	0x00000100	/* R/W */
#define	MC_RcvFCS		0x00000200	/* R/W */
#define	MC_FIFOLoopback		0x00000400	/* R/W */
#define	MC_MACLoopback		0x00000800	/* R/W */
#define	MC_AutoVLANtagging	0x00001000	/* R/W */
#define	MC_AutoVLANuntagging	0x00002000	/* R/W */
#define	MC_CollisionDetect	0x00010000	/* R/- */
#define	MC_CarrierSense		0x00020000	/* R/- */
#define	MC_StatisticsEnable	0x00200000	/* -/Wsc */
#define	MC_StatisticsDisable	0x00400000	/* -/Wsc */
#define	MC_StatisticsEnabled	0x00800000	/* R/- */
#define	MC_TxEnable		0x01000000	/* -/Wsc */
#define	MC_TxDisable		0x02000000	/* -/Wsc */
#define	MC_TxEnabled		0x04000000	/* R/- */
#define	MC_RxEnable		0x08000000	/* -/Wsc */
#define	MC_RxDisable		0x10000000	/* -/Wsc */
#define	MC_RxEnabled		0x20000000	/* R/- */
#define	MC_Paused		0x40000000	/* R/- */

#define	MC_BITS	\
	"\020"			\
	"\006DuplexSelect"	\
	"\007RcvLargeFrames"	\
	"\010TxFlowControlEnable"	\
	"\011RxFlowControlEnable"	\
	"\012RcvFCS"		\
	"\013FIFOLoopback"	\
	"\014MACLoopback"	\
	"\015AutoVLANtagging"	\
	"\016AutoVLANuntagging"	\
	"\021CollisionDetect"	\
	"\022CarrierSense"	\
	"\030StatisticsEnabled"	\
	"\033TxEnabled"		\
	"\036RxEnabled"		\
	"\037Paused"

/* offset 0x76 : PhyCtrl B */
#define	PC_MgmtClk		0x01	/* -/W */
#define	PC_MgmtData		0x02	/* R/W */
#define		PC_MgmtDataShift	1
#define	PC_MgmtDir		0x04	/* -/W */
#define	PC_PhyDuplexPolarity	0x08	/* R/W */
#define	PC_PhyDuplexStatus	0x10	/* R/- */
#define	PC_PhyLnkPolarity	0x20	/* R/W */
#define	PC_LinkSpeed		0xc0	/* R/- */
#define		PC_LinkSpeedShift	6
#define	PC_LinkSpeed_down	(0 << PC_LinkSpeedShift)
#define	PC_LinkSpeed_10		(1 << PC_LinkSpeedShift)
#define	PC_LinkSpeed_100	(2 << PC_LinkSpeedShift)
#define	PC_LinkSpeed_1000	(3 << PC_LinkSpeedShift)

/* offset 0x78-0x7d : StationAddress W*3 */

/* offset 0x86 : MaxFrameSize W */

/* offset 0x88 : ReceiveMode W */
#define	RM_ReceiveUnicast	0x0001	/* R/W */
#define	RM_ReceiveMulticast	0x0002	/* R/W */
#define	RM_ReceiveBroadcast	0x0004	/* R/W */
#define	RM_ReceiveAllFrames	0x0008	/* R/W */
#define	RM_ReceiveMulticastHash	0x0010	/* R/W */
#define	RM_ReceiveIPMulticast	0x0020	/* R/W */
#define	RM_ReceiveVLANMatch	0x0100	/* R/W */
#define	RM_ReceiveVLANHash	0x0200	/* R/W */

#define	RM_BITS	\
	"\020"			\
	"\001RxUnicast"		\
	"\002RxMulticast"	\
	"\003RxBroadcast"	\
	"\004RxAllFrames"	\
	"\005RxMulticastHash"	\
	"\006RxIPMulticast"	\
	"\011RxVLANMatch"	\
	"\012RxVLANHash"

/* offset 0x8c-0x93 : Hashtable DW*2 */
