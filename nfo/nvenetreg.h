/*
 * nvenetreg.h:  %Z%%M% %I%     %E%
 * Macro definitions for NVIDIA nForce integrated ethernet controller
 * Based on experience on re-engineering of forcedeth.c driver in linux.
 */
#ifndef _NVENETREG_H_
#define	_NVENETREG_H_
/*
 * Register offsets
 */
#define	ISR			0x000	/* interrupt status register */
#define	IMR			0x004	/* interrupt mask register */
#define	TIMERCTL		0x008
#define	TIMER			0x00c
#define	MSIMAP0			0x020
#define	MSIMAP1			0x024
#define	MSIIMR			0x030
#define	MACRESET		0x034
#define	TxCFG			0x080	/* tx configration */
#define	TxCTRL			0x084	/* tx control */
#define	TxSTAT			0x088	/* tx status */
#define	RxPF			0x08c	/* rx packet filter */
#define	MAXBUFSIZE		0x090
#define	RxCTRL			0x094	/* rx control */
#define	RxSTAT			0x098	/* rx status */
#define	BACKOFF			0x09c
#define	TxDEFER			0x0a0
#define	RxDEFER			0x0a4
#define	MACADDR			0x0a8	/* uint32 * 2 */
#define	MCASTADDR		0x0b0	/* uint32 * 2 */
#define	MCASTMASK		0x0b8	/* uint32 * 2 */
#define	PHYIF			0x0c0
#define	TxDESCBASE		0x100
#define	RxDESCBASE		0x104
#define	RINGCFG			0x108
#define	TxPOLL			0x10c
#define	RxPOLL			0x110
#define	RxDMASTAT		0x130	/* was setup5 */
#define	TXWM			0x13c	/* tx watermark */
#define	DMACTRL			0x144	/* DMA for tx and rx */
#define	TxDESCBASE_HI		0x148
#define	RxDESCBASE_HI		0x14c
#define	TXPAUSE			0x170
#define	MIIISR			0x180
#define	MIIIMR			0x184
#define	MIIAUTO			0x188
#define	MIICFG			0x18c
#define	MIICTL			0x190
#define	MIIDATA			0x194
#define	WAKECTRL		0x200
#define	PTNCRC			0x204
#define	PTNMASK			0x208
#define	PWRCTRL			0x268
#define	PWRSTAT			0x26c
#define	StatTxCNT		0x280
#define	StatTxZeroReXmt		0x284
#define	StatTxOneReXmt		0x288
#define	StatTxManyReXmt		0x28c
#define	StatTxLateCol		0x290
#define	StatTxUnderflow		0x294
#define	StatTxLostCarrier	0x298
#define	StatTxExcessDef		0x29c
#define	StatTxRetryErr		0x2a0
#define	StatRxFrameErr		0x2a4
#define	StatRxExtraByte		0x2a8
#define	StatRxLateCol		0x2ac
#define	StatRxRunt		0x2b0
#define	StatRxFrameTooLong	0x2b4
#define	StatRxOverflow		0x2b8
#define	StatRxFCSErr		0x2bc
#define	StatRxFrameAlignErr	0x2c0
#define	StatRxLenErr		0x2c4
#define	StatRxUnicast		0x2c8
#define	StatRxMulticast		0x2cc
#define	StatRxBroadcast		0x2d0
#define	StatTxDef		0x2d4
#define	StatTxFrame		0x2d8
#define	StatRxCnt		0x2dc
#define	StatTxPause		0x2e0
#define	StatRxPause		0x2e4
#define	StatRxDropFrame		0x2e8
#define	VLANCTRL		0x300
#define	MSIXMAP0		0x3e0
#define	MSIXMAP1		0x3e4
#define	MSIXISR			0x3f0
#define	PWRSTAT2		0x600

/* Interrupt Status register, offset 0 */
#define	INTR_RXERR_FATAL	0x8000U	/* rx fatal error */
#define	INTR_TXDESC		0x0100U	/* tx forced intr */
#define	INTR_RXDESC		0x0080U	/* rx forced intr */
#define	INTR_LSC		0x0040U	/* link status changed */
#define	INTR_TIMER		0x0020U	/* interval timer */
#define	INTR_TXOK		0x0010U	/* tx ok */
#define	INTR_TXERR		0x0008U	/* tx failed */
#define	INTR_RDU		0x0004U	/* rx descriptor unavail */
#define	INTR_RXOK		0x0002U	/* rx ok */
#define	INTR_RXERR		0x0001U	/* rx failed */

#define	INTR_ALL		0x81ffU

#define	INTR_BITS	\
	"\020"	\
	"\020RxErrFatal"	\
	"\011TXDESC"	\
	"\010RXDESC"	\
	"\007LSC"	\
	"\006TIMER"	\
	"\005TXOK"	\
	"\004TXERR"	\
	"\003RDU"	\
	"\002RXOK"	\
	"\001RXERR"

/* TxCFG register, offset 0x80 */
#define	TxCFG_JAM_SEQ		0x01000000U
#define	TxCFG_UDF_MASK		0x00200000U
#define	TxCFG_OWC_MASK		0x00100000U
#define	TxCFG_CRS_MASK		0x00080000U
#define	TxCFG_DEF_MASK		0x00040000U
#define	TxCFG_ABORT_MASK	0x00020000U
#define	TxCFG_RETRYE_MASK	0x00010000U
#define	TxCFG_BURST_EN		0x00001000U
#define	TxCFG_MAXRETRY		0x00000f00U
#define	TxCFG_TWO_DEF_EN	0x00000020U
#define	TxCFG_FCS_EN		0x00000010U
#define	TxCFG_PAD_EN		0x00000008U
#define	TxCFG_RETRY_EN		0x00000004U
#define	TxCFG_HD		0x00000002U	/* half duplex (legacy) */
#define	TxCFG_TxPAUSE		0x00000001U	/* tx pause: 1:en 0:dis */

#define	TxCFG_DEFAULT		0x003b0f3cU	/* initial value */

/* Tx Control register, offset 0x084 */
#define	TxCTRL_MGMT_ST		0x40000000U
#define	TxCTRL_TxPATH		0x01000000U
#define	TxCTRL_SYNC		0x000f0000U
#define		TxCTRL_SYNC_SHIFT	16U
#define		TxCTRL_SYNC_NOTREADY	(0U << TxCTRL_SYNC_SHIFT)
#define		TxCTRL_SYNC_PHYINIT	(4U << TxCTRL_SYNC_SHIFT)
#define	TxCTRL_HSEMA		0x0000f000U
#define		TxCTRL_HSEMA_SHIFT	12U
#define		TxCTRL_HSEMA_LOADED	(4U << TxCTRL_HSEMA_SHIFT)
#define		TxCTRL_HSEMA_ACQ	(15U << TxCTRL_HSEMA_SHIFT)
#define	TxCTRL_MSEMA		0x00000f00U
#define		TxCTRL_MSEMA_SHIFT	8U
#define		TxCTRL_MSEMA_FREE	(0U << TxCTRL_HSEMA_SHIFT)
#define	TxCTRL_START		0x00000001U

#define	TxCTRL_BITS	\
	"\020"	\
	"\037MGMT_ST"	\
	"\031TxPATH"	\
	"\001START"

/* Tx Status register, offset 0x088 */
#define	TxSTAT_RUNNING		0x00000001U

/* Rx Packet Filter register, offset 0x08c */
#define	RxPF_MBO		0x007f0000U	/* constant */
#define	RxPF_PROMISC		0x00000080U	/* accept all physical */
#define	RxPF_UNICAST		0x00000020U	/* accept my phisical */
#define	RxPF_LOOPBACK		0x00000010U	/* loopback mode 1:en, 0:dis */
#define	RxPF_RxPAUSE		0x00000008U	/* 1:enable, 0:disable */

/* Receiver Control register, offset 0x094 */
#define	RxCTRL_RxPATH		0x01000000U
#define	RxCTRL_START		0x00000001U

/* Receiver Status register, offset 0x098 */
#define	RxSTAT_RUNNING		0x00000001U

/* Backoff control register, offset 0x09c */
#define	BACKOFF_LEGACY		0x80000000U
#define	BACKOFF_SPEED		0x0003ff00U
#define		BACKOFF_SPEED_SHIFT	8U
#define		BACKOFF_SPEED_10M	(0x07fU << BACKOFF_SPEED_SHIFT)
#define		BACKOFF_SPEED_100M	(0x07fU << BACKOFF_SPEED_SHIFT)
#define		BACKOFF_SPEED_1G	(0x3ffU << BACKOFF_SPEED_SHIFT)
#define		BACKOFF_SPEED_MII	(0x07fU << BACKOFF_SPEED_SHIFT)
#define	BACKOFF_SEED		0x000000ffU

/* Tx defferral timing register, offset 0x0a0 */
#define	TD_IFDEF		0x00ff0000U
#define		TD_IFG_SHIFT	16
#define		TD_IFG_MII		(21 << TD_IFG_SHIFT)
#define		TD_IFG_RGMII_1G_FD	(20 << TD_IFG_SHIFT)
#define		TD_IFG_RGMII_OTHER	(22 << TD_IFG_SHIFT)
#define	TD_IFG2			0x0000ff00U
#define		TD_IFG2_SHIFT	8
#define		TD_IFG2_MII		(5 << TD_IFG2_SHIFT)
#define		TD_IFG2_RGMII_10_100	(7 << TD_IFG2_SHIFT)
#define		TD_IFG2_RGMII_1000	(5 << TD_IFG2_SHIFT)
#define		TD_IFG2_DEFAULT		(0 << TD_IFG2_SHIFT)
#define	TD_IFG1			0x000000ffU
#define		TD_IFG1_SHIFT	0
#define		TD_IFG1_DEFAULT		(15 << TD_IFG1_SHIFT)

/* PHY interface register, offset 0x0c0 */
#define	PHYIF_RGMII		0x10000000U	/* 1:RGMII, 0:MII */
#define	PHYIF_SPEED		0x00000003U	/* speed select */
#define		PHYIF_SPEED_SHIFT	0U
#define		PHYIF_SPEED_10M		(0U << PHYIF_SPEED_SHIFT)
#define		PHYIF_SPEED_100M	(1U << PHYIF_SPEED_SHIFT)
#define		PHYIF_SPEED_1G		(2U << PHYIF_SPEED_SHIFT)
#define	PHYIF_HALF		0x00000100U	/* duplex 1:half 0:full */

/* Ring Sizes register, offset 0x108 */
#define	RINGCFG_RX		0x03ff0000U
#define		RINGCFG_RX_SHIFT	16
#define	RINGCFG_TX		0x000003ffU
#define		RINGCFG_TX_SHIFT	0
#define	RINGSIZE_MAX	0x400

/* Transmit poll register, offset 0x10c */
#define	TxPOLL_MACADDR_REV	0x00008000U

/* Receive poll register, offset 0x110 */
#define	RxPOLL_EN		0x00010000U
#define	RxPOLL_ITV		0x00000fffU	/* interval */
#define		RxPOLL_ITV_SHIFT	0U
#define		RxPOLL_ITV_10M		(1000U << RxPOLL_ITV_SHIFT)
#define		RxPOLL_ITV_100M		(100U << RxPOLL_ITV_SHIFT)
#define		RxPOLL_ITV_1G		(50U << RxPOLL_ITV_SHIFT)

/* Rx DMA status register, offset 130 */
#define	RxDMASTAT_RUNNING	0x80000000U

/* DMA control register, offset 0x144 */
#define	DMACTRL_TxRCMP_EN	0x00002000U
#define	DMACTRL_RXCHECK		0x00000400U
#define	DMACTRL_DFMT		0x00000300U
#define		DMACTRL_DFMT_SHIFT	8U
#define		DMACTRL_DFMT_1		(0U << DMACTRL_DFMT_SHIFT)
#define		DMACTRL_DFMT_2		(1U << DMACTRL_DFMT_SHIFT)
#define		DMACTRL_DFMT_3		(2U << DMACTRL_DFMT_SHIFT)
#define	DMACTRL_VLANINS		0x00000080U
#define	DMACTRL_VLANSTRIP	0x00000040U
#define	DMACTRL_RESET		0x00000010U
#define	DMACTRL_IDLE		0x00000008U
#define	DMACTRL_STOP		0x00000004U	/* -w */
#define	DMACTRL_RxPOLL		0x00000002U	/* -w */
#define	DMACTRL_TxPOLL		0x00000001U	/* -w */

#define	DMACTRL_BITS	\
	"\020"	\
	"\016DescEx"	\
	"\013RXCHECK"	\
	"\010VLANINS"	\
	"\007VLANSTRIP"	\
	"\005RESET"	\
	"\004IDLE"	\
	"\003STOP"	\
	"\002RxPOLL"	\
	"\001TxPOLL"

/* tx pause control register, offset 170 */
#define	TXPAUSE_DIS	0x0fff0080U
#define	TXPAUSE_EN1	0x01800010U
#define	TXPAUSE_EN2	0x056003f0U
#define	TXPAUSE_EN3	0x09f00880U

/* MII interrupt status register, offset 180 */
/* MII interrupt mask register, offset 184 */
#define	MIIISR_LSC		0x00000008U	/* link status changed */
#define	MIIISR_DONE		0x00000004U	/* MII read/write done */
#define	MIIISR_ERROR		0x00000001U	/* MII read/write error */

#define	MIIISR_ALL	(MIIISR_LSC | MIIISR_DONE | MIIISR_ERROR)

#define	MIIISR_BITS	\
	"\020"	\
	"\004LSC"	\
	"\003Done"	\
	"\001Error"

/* MII auto sense control register, offset 188 */
#define	MIIAUTO_PHYADDR		0x1f000000U
#define		MIIAUTO_PHYADDR_SHIFT	24U
#define	MIIAUTO_EN		0x00100000U
#define	MIIAUTO_PHYVALID	0x00040000U
#define	MIIAUTO_LINKUP		0x00000004U
#define	MIIAUTO_BUSY		0x00000002U

/* MII Control register, offset 0x190 */
#define	MIICTL_BUSY		0x00008000U
#define	MIICTL_WRITE		0x00000400U	/* 1:write, 0:read */
#define	MIICTL_ADDR		0x000003e0U	/* phy address */
#define		MIICTL_ADDR_SHIFT	5U
#define	MIICTL_REG		0x0000001fU	/* register location */
#define		MIICTL_REG_SHIFT	0U

/* Wake up Flags register, offset 0x200 */
#define	WAKECTRL_Dx_MAGPAT(x)		(1U << ((x)*4))
#define	WAKECTRL_Dx_WAKEUPPAT(x)	(2U << ((x)*4))
#define	WAKECTRL_Dx_LSC(x)		(4U << ((x)*4))

#define	WAKECTRL_ENABLE	\
	(WAKECTRL_Dx_MAGPAT(0) | WAKECTRL_Dx_MAGPAT(1) |	\
	WAKECTRL_Dx_MAGPAT(2) | WAKECTRL_Dx_MAGPAT(3))

/* Power control register, offset 268 */
#define	PWRCTRL_D3SUPP		0x40000000U
#define	PWRCTRL_D2SUPP		0x04000000U
#define	PWRCTRL_D1SUPP		0x02000000U

/* Power State register, offset 0x26c */
#define	PWRSTAT_POWEREDUP	0x00008000U
#define	PWRSTAT_VALID		0x00000100U
#define	PWRSTAT_STATE		0x00000003U
#define		PWRSTAT_STATE_D0	0U
#define		PWRSTAT_STATE_D1	1U
#define		PWRSTAT_STATE_D2	2U
#define		PWRSTAT_STATE_D3	3U

/* VLAN control register, offset 0x300 */
#define	VLANCTRL_EN		0x2000U

/* Power State2 register, offset 0x600 */
#define	PWRSTAT2_GATE_CLOCKS	0x0f00U
#define	PWRSTAT2_PHY_RESET	0x0004U
#define	PWRSTAT2_POWERUP_A3	0x0001U
#define	PWRSTAT2_POWERUP_MASK	0x0f15U

/*
 * Rx and Tx descriptor formats
 */
struct rxdesc32 {
	volatile uint32_t	rxd0;	/* buffer address */
	volatile uint32_t	rxd1;	/* control, status, length */
};

struct rxdesc64 {
	volatile uint32_t	rxd0;	/* buffer address (high) */
	volatile uint32_t	rxd1;	/* buffer address (low) */
	volatile uint32_t	rxd2;	/* vtag */
	volatile uint32_t	rxd3;	/* control, status, length */
};

/* common bits in rx descriptors */
#define	RX_FLAG_MASK_1		0xffff0000U
#define	RX_LEN_MASK_1		0x0000ffffU
#define	RX_FLAG_MASK_2		0xffffc000U
#define	RX_LEN_MASK_2		0x00003fffU

struct txdesc32 {
	volatile uint32_t	txd0;	/* buffer address */
	volatile uint32_t	txd1;	/* control, status, length */
};

struct txdesc64 {
	volatile uint32_t	txd0;	/* buffer address (high) */
	volatile uint32_t	txd1;	/* buffer address (low) */
	volatile uint32_t	txd2;	/* vtag */
	volatile uint32_t	txd3;	/* control, status, length */
};

/* common bits in tx descriptors */
#define	TxD_OWN			0x80000000U

/*
 * Tx descriptor control and status flags for format 1.
 */
/* control bits */
#define	TxD_OWN_1		TxD_OWN		/* 0x80000000 */
#define	TxD_ERR_1		0x40000000U
#define	TxD_UDF_1		0x20000000U
#define	TxD_OWC_1		0x10000000U
#define	TxD_CRS_1		0x08000000U
#define	TxD_DEF_1		0x04000000U
#define	TxD_INT_1		0x01000000U
#define	TxD_INT_1_NF4		0x00800000U
#define	TxD_COL_1		0x00f00000U
#define		TxD_COL_SHIFT_1		20
#define	TxD_ABORT_1		0x00080000U
#define	TxD_LAST_1		0x00010000U
#define	TxD_LEN_1		0x0000ffffU

/*
 * Tx descriptor control and status flags for format 2 and 3.
 */
/* control bits */
#define	TxD_OWN_2		TxD_OWN		/* 0x80000000 */
#define	TxD_INT_2		0x40000000U
#define	TxD_LAST_2		0x20000000U
#define	TxD_TSO_2		0x10000000U
#define	TxD_CKSUM_IPv4_2	0x08000000U
#define	TxD_CKSUM_TCPUDP_2	0x04000000U
#define	TxD_MSS_2		0x03ffc000U
#define		TxD_MSS_SHIFT_2		14U
#define	TxD_LEN_2		0x00003fffU

/* status bits in txd1 or txd3 */
#define	TxD_ERR_2		0x40000000U
#define	TxD_UDF_2		0x10000000U
#define	TxD_OWC_2		0x08000000U
#define	TxD_CRS_2		0x04000000U
#define	TxD_TOK_2		0x02000000U
#define	TxD_COL_2		0x00f00000U
#define		TxD_COL_SHIFT_2		20
#define	TxD_DEF_2		0x00080000U
#define	TxD_ABORT_2		0x00040000U

/* vlan bits in txd2 of format 3 */
#define	TxD_VLAN_EN		0x00040000U
#define	TxD_VTAG		0x0000ffffU

/*
 * Rx descriptor control and status flags for format 1.
 */
/* common bits */
#define	RxD_OWN			0x80000000U
#define	RxD_ERR			0x40000000U
#define	RxD_LEN			0x00003fffU

/* control and status bits */
#define	RxD_OWN_1		RxD_OWN
#define	RxD_ERR_1		RxD_ERR
#define	RxD_FRAME_1		0x20000000U
#define	RxD_OVF_1		0x10000000U
#define	RxD_CRC_1		0x08000000U
#define	RxD_ERR4_1		0x04000000U
#define	RxD_ERR3_1		0x02000000U
#define	RxD_ERR2_1		0x01000000U
#define	RxD_ERR1_1		0x00800000U
#define	RxD_SUBSTRACT1_1	0x00040000U
#define	RxD_MISSED_1		0x00020000U
#define	RxD_VALID_1		0x00010000U
#define	RxD_LEN_1		RxD_LEN

#define	RxD_1_BITS	\
	"\020"	\
	"\040OWN"	\
	"\037ERR"	\
	"\036FRAME"	\
	"\035OVF"	\
	"\034CRC"	\
	"\033ERR4"	\
	"\032ERR3"	\
	"\031ERR2"	\
	"\030ERR1"	\
	"\023SUBSTRACT1"	\
	"\022MISSED"	\
	"\021VALID"

/*
 * Rx descriptor control and status flags for format 2 and 3.
 */
/* control and status bits in rxd1 or rxd3 */
#define	RxD_OWN_2		RxD_OWN
#define	RxD_ERR_2		RxD_ERR
#define	RxD_VALID_2		0x20000000U
#define	RxD_CKSUMP_2		0x1c000000U
#define		RxD_CKSUMP_SHIFT_2	26U
#define		CKSUMP_2		7U
#define		CKSUMP_UDP_2		6U
#define		CKSUMP_TCP_2		5U
#define		CKSUMP_IP_2		4U
#define		CKSUMP_IP_ERR_2		3U
#define		CKSUMP_UDP_ERR_2	2U
#define		CKSUMP_TCP_ERR_2	1U
#define		CKSUMP_NONE_2		0U
#define	RxD_SUBSTRACT1_2	0x02000000U
#define	RxD_FRAME_2		0x01000000U
#define	RxD_OVF_2		0x00800000U
#define	RxD_CRC_2		0x00400000U
#define	RxD_ERR4_2		0x00200000U
#define	RxD_ERR3_2		0x00100000U
#define	RxD_ERR2_2		0x00080000U
#define	RxD_ERR1_2		0x00040000U
#define	RxD_LEN_2		RxD_LEN

#define	RxD_2_BITS	\
	"\020"	\
	"\040OWN"	\
	"\037ERR"	\
	"\036VALID"	\
	"\035CKSUMOK"	\
	"\034CKSUMP_UDP"	\
	"\033CKSUMP_TCP"	\
	"\032SUBSTRACT1"	\
	"\031FRAME"	\
	"\030OVF"	\
	"\027CRC"	\
	"\026ERR4"	\
	"\025ERR3"	\
	"\024ERR2"	\
	"\023ERR1"

/* vlan bits in rxd2 of format 3 */
#define	RxD_VLAN		0x00010000U
#define	RxD_VTAG		0x0000ffffU

#endif /* _NVENETREG_H_ */
