/*
 * i8255xreg.h: Intel i8255x 10/100 Mbps Ethernet controller registers
 * @(#)i8255xreg.h	1.2 11/09/19
 * Based on Intel Open Souce Software Developer Manual
 * Coded by Masayuki Murayama KHF04453@nifty.ne.jp
 * This file is public domain.
 */

/*
 * Register offsets
 */
#define	SCBSTAT		0x00	/* W SCB status word */
#define	SCBCMD		0x02	/* W SCB command word */
#define	GENPTR		0x04	/* D SCB general pointer */
#define	PORT		0x08	/* D Port interface */
#define	EECTRL		0x0e	/* B EEPROM control register */
#define	MDICTRL		0x10	/* D MDI  control register */
#define	RXBC		0x14	/* D Rx DMA byte count register */
#define	EARLYRXINT	0x18	/* B earlt rx interrupt register */
#define	FCTHR		0x19	/* B Flow control threashold register */
#define	FCTRL		0x1a	/* B Flow control on/off register */
#define	PMDR		0x1b	/* B PMDR register */
#define	GENCTRL		0x1c	/* B general control register */
#define	GENSTAT		0x1d	/* B general status register */
#define	FEVENT		0x30	/* D Function event register */
#define	FEVMASK		0x34	/* D Function event mask register */
#define	FPRESTATE	0x38	/* D Function present state register */
#define	FORCEEV		0x3c	/* D Force event register */


/* SCB Status Word */
#define	SS_CX		0x8000	/* cmd done w/ intr */
#define	SS_TNO		0x8000	/* Tx not OK (82557) */
#define	SS_FR		0x4000	/* Rx done */
#define	SS_CNA		0x2000	/* CU not active / */
#define	SS_RNR		0x1000	/* RU not ready */
#define	SS_MDI		0x0800	/* MDI operation done */
#define	SS_SWI		0x0400	/* software interrupt accepted */
#define	SS_FCP		0x0100	/* flow control pause (82558-) */
#define	SS_CUS		0x00c0	/* CU status */
#define	SS_CUS_SHIFT	6
#define		CUS_IDLE	0U
#define		CUS_SUSPENDED	1U
#define		CUS_LPQ		2U
#define		CUS_HPQ		3U

#define		SS_CUS_IDLE		(CUS_IDLE << SS_CUS_SHIFT)
#define		SS_CUS_SUSPENDED	(CUS_SUSPENDED << SS_CUS_SHIFT)
#define		SS_CUS_LPQ		(CUS_LPQ << SS_CUS_SHIFT)
#define		SS_CUS_HPQ		(CUS_HPQ << SS_CUS_SHIFT)
#define	SS_RUS		0x003c		/* RU status */
#define	SS_RUS_SHIFT	2
#define		RUS_IDLE	0U
#define		RUS_SUSPENDED	1U
#define		RUS_NORSRC	2U
#define		RUS_READY	4U

#define		SS_RUS_IDLE		(RUS_IDLE << SS_RUS_SHIFT)
#define		SS_RUS_SUSPENDED	(RUS_SUSPENDED << SS_RUS_SHIFT)
#define		SS_RUS_NORSRC		(RUS_NORSRC << SS_RUS_SHIFT)
#define		SS_RUS_READY		(RUS_READY << SS_RUS_SHIFT)

#define	SCBSTAT_BITS	\
	"\020"	\
	"\020CX"	\
	"\017FR"	\
	"\016CNA"	\
	"\015RNR"	\
	"\014MDI"	\
	"\013SWI"	\
	"\011FCP"

/* SCB Command Word */
#define	SC_INTMASK	0xfc00U	/* individual interrupt mask (82558-) */
#define	SC_IM_CX	0x8000U	/* 1: disable intr, 0:enable intr */
#define	SC_IM_FR	0x4000U
#define	SC_IM_CNA	0x2000U
#define	SC_IM_RNR	0x1000U
#define	SC_IM_ER	0x0800U
#define	SC_IM_FCP	0x0400U
#define	SC_SI		0x0200U	/* generate software interrupt */
#define	SC_M		0x0100U	/* interrupt mask 1:mask 0: nomask */
#define	SC_CUC		0x00f0U	/* CU command */
#define		SC_CUC_SHIFT	4
#define		SC_CUC_NOP	(0x0U << SC_CUC_SHIFT)
#define		SC_CUC_START	(0x1U << SC_CUC_SHIFT)
#define		SC_CUC_RESUME	(0x2U << SC_CUC_SHIFT)
#define		SC_CUC_LOADSDMP	(0x4U << SC_CUC_SHIFT)
#define		SC_CUC_SDMP	(0x5U << SC_CUC_SHIFT)
#define		SC_CUC_LOADBASE	(0x6U << SC_CUC_SHIFT)
#define		SC_CUC_SDMPRST	(0x7U << SC_CUC_SHIFT)
#define		SC_CUC_SRESUME	(0xaU << SC_CUC_SHIFT)
#define	SC_RUC		0x0007U	/* RU command */
#define		SC_RUC_SHIFT	0
#define		SC_RUC_NOP	(0U << SC_RUC_SHIFT)
#define		SC_RUC_START	(1U << SC_RUC_SHIFT)
#define		SC_RUC_RESUME	(2U << SC_RUC_SHIFT)
#define		SC_RUC_REDIRECT	(3U << SC_RUC_SHIFT)
#define		SC_RUC_ABORT	(4U << SC_RUC_SHIFT)
#define		SC_RUC_LOADHDS	(5U << SC_RUC_SHIFT)
#define		SC_RUC_LOADBASE	(6U << SC_RUC_SHIFT)

/*
 * Statistical counter offsets
 */
#define	SO_TxGoodFrames			0
#define	SO_TxMaxCollisionsErrors	4
#define	SO_TxLateCollisions		8
#define	SO_TxUnderrunErrors		12
#define	SO_TxLostCarrierSense		16
#define	SO_TxDeferred			20
#define	SO_TxSingleCollision		24
#define	SO_TxMultipleCollision		28
#define	SO_TxTotalCollision		32
#define	SO_RxGoodFrames			36
#define	SO_RxCRCErrrs			40
#define	SO_RxAlignmentErrors		44
#define	SO_RxResourceErrors		48
#define	SO_RxOverrunErrors		52
#define	SO_RxCollisionDetectErrors	56
#define	SO_RxShortFrameErrors		60
#define	SO_FlowCtrlTxPause		64	/* 82558- */
#define	SO_FlowCtrlRxPause		68	/* 82558- */
#define	SO_FlowCtrlRxUnsuported		72	/* 82558- */
#define	SO_TxTCOFrames			76	/* word 82559- */
#define	SO_RxTCOFrames			78	/* word 82559- */

/* Port register */
#define	PORT_SOFTRESET	0x0		/* entire reset */
#define	PORT_SELFTEST	0x1
#define	PORT_SELECTIVE	0x2
#define	PORT_DUMP	0x3
#define	PORT_DUMPWAKEUP	0x5		/* 82559 */
#define	PORT_ADDR	0xfffffff0

/* EEPROM control register */
#define	EC_EEDO_SHIFT	3
#define	EC_EEDO	(1U << EC_EEDO_SHIFT)	/* serial data-out from eeprom */
#define	EC_EEDI_SHIFT	2
#define	EC_EEDI	(1U << EC_EEDI_SHIFT)	/* serial data-in to eeprom */
#define	EC_EECS		0x02	/* chip select 0:disable, 1:enable */
#define	EC_EESK		0x01	/* serial clock */

/* MDI control register */
#define	MC_IE	0x20000000	/* interrupt enable */
#define	MC_R	0x10000000	/* ready */
#define	MC_OP_SHIFT	26
#define	MC_OP_WRITE	(1U << MC_OP_SHIFT)
#define	MC_OP_READ	(2U << MC_OP_SHIFT)
#define	MC_PHYADD_SHIFT	21
#define	MC_REGADD_SHIFT	16
#define	MC_DATA	0x0000ffffU

/* Flow control threadshold register */
#define	FT_512	0
#define	FT_1024	1
#define	FT_1280	2
#define	FT_1536	3
#define	FT_1792	4
#define	FT_2048	5
#define	FT_2304	6
#define	FT_2560	7

/* Flow control register */
#define	FC_PAUSED_LOW	0x10	/* RO */
#define	FC_PAUSED	0x08	/* RO */
#define	FC_FULL		0x04	/* RO */
#define	FC_XOFF		0x02
#define	FC_XON		0x01

/* Shared memory operations */

/* OPCODES */
#define	OP_NOP		0
#define	OP_ADDRSETUP	1	/* individual address setup */
#define	OP_CONFIGURE	2	/* load the device with operating parameters */
#define	OP_MULTICAST	3	/* setup one or more multicast address */
#define	OP_TX		4	/* Transmit a single frame */
#define	OP_LOADMCODE	5	/* download microcode to the device */
#define	OP_DUMP		6	/* */
#define	OP_DIAG		7	/* kick selftest and report the result */
#define	OPMASK		7

#define	CS_EL		0x80000000U
#define	CS_S		0x40000000U	/* suspend */
#define	CS_I		0x20000000U	/* generate interrupt */
#define	CS_CID		0x1f000000U	/* CNA interrupt delay (tcb only) */
#define		CS_CID_SHIFT	24
#define	CS_H		0x00100000U
#define	CS_SF_		0x00080000U	/* 0:simplify mode, 1:flexible mode */
#define	CS_OP		0x00070000U
#define		CS_OP_SHIFT	16
#define	CS_C		0x00008000U	/* completed */
#define	CS_OK		0x00002000U	/* no error  */
#define	CS_U		0x00001000U	/* underrum happened */
#define	CS_CRC		0x00000800U	/* CRC error happened */
#define	CS_ALE		0x00000400U	/* alignment error happened */
#define	CS_LONG		0x00000200U	/* packet is too long */
#define	CS_OVERRUN	0x00000100U	/* DMA overrun */
#define	CS_SHORT	0x00000080U	/* runt */
#define	CS_TYPE		0x00000020U	/* type:1, length:0 */
#define	CS_RXERR	0x00000010U	/* error */

#define	CS_BITS		\
	"\020"	\
	"\040EL"	\
	"\037S"		\
	"\036I"		\
	"\025H"		\
	"\024SF_"	\
	"\020C"		\
	"\016OK"	\
	"\015U"		\
	"\014CRC"	\
	"\013ALE"	\
	"\012LONG"	\
	"\011OVERRUN"	\
	"\010SHORT"	\
	"\006TYPE"	\
	"\005RXERR"

/* command block */
struct cb {
	volatile uint32_t	cmdstat;
	volatile uint32_t	link;
};

/* Transmit command block */
struct tcb {
	volatile uint32_t	tcb_tbdptr;	/* tbd array address */
	volatile uint32_t	tcb_ctrl;
};

#define	TCB_TBDNUM	0xff000000U
#define	TCB_TBDNUM_SHIFT	24
#define	TCB_TXTHR	0x00ff0000U
#define	TCB_TXTHR_SHIFT	16
#define		TXTHR_UNIT	8U
#define		TXTHR_SF	0xe0U
#define	TCB_EOF		0x00008000U	/* for backword compatibility */
#define	TCB_CNT		0x000003ffU	/* tcb byte count */

/* Transmit buffer descriptor */
struct tbd {
	volatile uint32_t	tb_addr;
#define	TBD_ADDR_NULL	0xffffffff
	volatile uint32_t	tb_size;
#define	TBD_EL	0x00010000
};

#define	IFE_MAX_MTU	(2600 - 14)

struct rfd {
	volatile uint32_t	cmdstat;
	volatile uint32_t	link;
	volatile uint32_t	:32;	/* not used */
	volatile uint32_t	size;
};

#define	RFD_SIZE	0x3fff0000U	/* prepared buffer size */
#define	RFD_SIZE_SHIFT	16

#define	RFD_EOF		0x00008000U	/* end of frame */
#define	RFD_F		0x00004000U
#define	RFD_COUNT	0x00003fffU	/* actual received byte count */

/*
 * Configuration format
 */

#define	C0_CONST		0x00
#define	C1_CONST		0x00
#define	C2_CONST		0x00

#define	C3_CONST		0x00
#define	C3_TermWriteOnCL	0x08
#define	C3_ReadAlEn		0x04
#define	C3_TypeEn		0x02
#define	C3_MWIEn		0x01

#define	C4_CONST		0x00

#define	C5_CONST		0x00
#define	C5_DMBC			0x80

#define	C6_CONST		0x02
#define	C6_SaveBadFrames	0x80
#define	C6_DiscardOverruns_	0x40
#define	C6_ExtStatCount_	0x20
#define	C6_ExtTCB_		0x10
#define	C6_CIInterrupt		0x08
#define	C6_TNOInter		0x04	/* 82557 */
#define	C6_TCOStat		0x04	/* 82559 */
#define	C6_LateSCB		0x04	/* 82557 */

#define	C7_DynamicTCB		0x80
#define	C7_2FrameInFIFO		0x40
#define	C7_UnderrunRetry	0x06
#define	C7_UnderrunRetry_SHIFT	1
#define	C7_DiscardShortRx	0x01

#define	C8_CSMADisable		0x80
#define	C8_MII			0x01

#define	C9_LinkWakeupEn		0x20
#define	C9_VLAN_TCO		0x10

#define	C10_CONST		0x06
#define	C10_LoopBack_Normal	0x00
#define	C10_LoopBack_Int	0x40
#define	C10_LoopBack_Ext	0xc0
#define	C10_PreambleLen_1	0x00
#define	C10_PreambleLen_3	0x10
#define	C10_PreambleLen_7	0x20	/* default */
#define	C10_PreambleLen_15	0x30
#define	C10_NSAI		0x08	/* No Source Address Insersion */

#define	C11_CONST		0x00
#define	C11_LinearPrio		0x07	/* must be 0 */

#define	C12_IFS_SHIFT		4
#define	C12_IFS_STD		(6 << C12_IFS_SHIFT)
#define	C12_LinearPrioMode	0x01

#define	C13_CONST		0x00
#define	C14_CONST		0xf2

#define	C15_CONST		0x48
#define	C15_CRSCDT		0x80
#define	C15_CRC16		0x20
#define	C15_IgnoreUL		0x10
#define	C15_WaitAfterWin	0x04
#define	C15_BroadcastDes	0x02
#define	C15_Promiscuous		0x01

#define	C18_CONST		0x80
#define	C18_PrioFCThr		0x70
#define	C18_PrioFCThr_SHIFT	4
#define	C18_LongRxOK		0x08
#define	C18_RxCRCTrans		0x04
#define	C18_Padding		0x02
#define	C18_Stripping		0x01

#define	C19_AutoFDX		0x80
#define	C19_ForceFDX		0x40
#define	C19_RejectFC		0x20
#define	C19_RxFCRestart		0x10
#define	C19_RxFCRestop		0x08
#define	C19_TxFC_		0x04
#define	C19_MagicPkt_		0x02

#define	C20_CONST		0x1f
#define	C20_MultipleIA		0x40
#define	C20_PrioFCLocation	0x20

#define	C21_MulticastAll	0x08
#define	C21_CONST		0x05
