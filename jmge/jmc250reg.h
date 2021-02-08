/*
 *  jme250reg.h: JMicrom JMC250/260 PCI-E ethernet controller driver.
 *
 *  Based on JMC250/260 datasheet.
 *  This file is public domain.  Coded by M.Murayama.
 */
#pragma ident	"@(#)jmc250reg.h	1.4 12/04/04"

#ifndef	_JMC250REG_H_
#define	_JMC250REG_H_

/* register offset for configration registers */
#define	PCIREG_PRIV_EFUSE1	0xb8
#define	PCIREG_PRIV_EFUSE2	0xbc

/* resgister offset */
#define	TXCS	0x00	/* 4: transmit control and status */
#define	TXDBAL	0x04	/* 8: tx queue descriptors base addr */
#define	TXDBAH	0x08
#define	TXQDC	0x0c	/* 4: tx queue length */
#define	TXNDA	0x10	/* 4: tx queue next descriptor addr */
#define	TXMCS	0x14	/* 4: tx mac control and status */
#define	TXPFC	0x18	/* 4: tx pause frame control */
#define	TXTRHD	0x1c	/* 4: tx timer/retry control at half duplex */
#define	RXCS	0x20	/* 4: rx control and status */
#define	RXDBAL	0x24	/* 8: rx descriptors base addr */
#define	RXDBAH	0x28
#define	RXQDC	0x2c	/* 4: rx queue length */
#define	RXNDA	0x30	/* 4: rx queue next descriptor addr */
#define	RXMCS	0x34	/* 4: rx mac control and status */
#define	RXUMA	0x38	/* 8: rx unicast mac address */
#define	RXMCHT	0x40	/* 4: rx multicast hash table */
#define	WFODP	0x48	/* 4: wakeup frame output data port */
#define	WFOI	0x4c	/* 4: wakeup frame output interface */
#define	SMI	0x50	/* 4: station management interface */
#define	GHC	0x54	/* 4: global host control */
#define	PMCS	0x60	/* 4: power management control and status */
#define	STAT	0x60	/* 4: base of statistic registers */
#define	ERR7C	0x7C	/* 4: TXBAD, RXBAD */

/* giga PHY and EEPROM control registers */
#define	PHYPWR	0x424	/* 4: phy power down*/
#define	GIGACSR	0x428	/* 4: */
#define	LINKSTS	0x430	/* 2: link status */
#define	SMBCSR	0x440	/* 4: SMB control and status */
#define	SMBINTF	0x444	/* 4: SMB interface */

/* MISC registers */
#define	TMCSR	0x800	/* 4: timer control and status */
#define	GPIO	0x804	/* 4: general purpose io */
#define	GPREG0	0x808	/* 4: general purpose REG0 */
#define	GPREG1	0x80c	/* 4: general purpose REG1 */
#define	MSINUM	0x810	/* 16: msix entry number */
#define	IEVE	0x820	/* 4: interrupt event status */
#define	IREQ	0x824	/* 4: interrupt request status */
#define	IENS	0x828	/* 4: interrupt enable - setting port */
#define	IENC	0x82c	/* 4: interrupt enable - clearing port */
#define	PCCRX0	0x830	/* 4: packet completion coalescing cotrol of rxq0 */
#define	PCCRX1	0x834	/* 4: packet completion coalescing cotrol of rxq1 */
#define	PCCRX2	0x838	/* 4: packet completion coalescing cotrol of rxq2 */
#define	PCCRX3	0x83c	/* 4: packet completion coalescing cotrol of rxq3 */
#define	PCCTX	0x840	/* 4: packet completion coalescing cotrol of txqs */
#define	CHIPMODE 0x844	/* 4: revision and mode */
#define	SHBAHI	0x848	/* 4: shadow status base address [63:32] */
#define	SHBALO	0x84c	/* 4: shadow status base address [31:0] */
#define	TIMER1	0x870	/* 4: */
#define	TIMER2	0x874	/* 4: */
#define	APMC	0x87c	/* 4: aggressive power mode control */
#define	PCCSRX0	0x880	/* 4: packet completion coaliscing status base */
#define	PCCSRX1	0x884
#define	PCCSRX2	0x888
#define	PCCSRX3	0x88c
#define	PCCSTX	0x890	/* 4: packet completion coaliscing status of txq */
#define	TXEMP	0x894	/* 4: tx queues empty indicator */

/*
 * register definitions
 */

/* PCIREG_PRIV_EFUSE1:, offset 0xb8 */
#define	PCICMD_EFUSE1_CMDREQ		0x00800000U	/* */
#define	PCICMD_EFUSE1_CMDSELMSK		0x00600000U	/* */
#define		PCICMD_EFUSE1_CMDSELSFT	21
#define		PCICMD_EFUSE1_CMDSELAUTOLD	(1 << PCICMD_EFUSE1_CMDSELSFT)
#define		PCICMD_EFUSE1_CMDSELRDCMD	(2 << PCICMD_EFUSE1_CMDSELSFT)
#define		PCICMD_EFUSE1_CMDSELBLOWCMD	(3 << PCICMD_EFUSE1_CMDSELSFT)

#define	PCICMD_EFUSE1_AUTOLDERRFLG	0x00010000U	/* */
#define	PCICMD_EFUSE1_INITLDDONE	0x00000001U	/* */

/* PCIREG_PRIV_EFUSE2:, offset 0xbc */
#define	PCICMD_EFUSE2_RESET		0x00008000U	/* */

/* TXCS: transmit control and status, offset 0x00 */
#define	TXCS_QW		0x0f000000U	/* queue weight */
#define		TXCS_QW_SHIFT		24
#define	TXCS_MSSINlIT	0x00800000U	/* pseudo interrupts for lso */
#define	TXCS_QSEL	0x00070000U	/* queue selection */
#define		TXCS_QSEL_SHIFT		16
#define	TXCS_Q7S	0x00008000U	/* queue 7 start */
#define	TXCS_Q6S	0x00004000U	/* queue 6 start */
#define	TXCS_Q5S	0x00002000U	/* queue 5 start */
#define	TXCS_Q4S	0x00001000U	/* queue 4 start */
#define	TXCS_Q3S	0x00000800U	/* queue 3 start */
#define	TXCS_Q2S	0x00000400U	/* queue 2 start */
#define	TXCS_Q1S	0x00000200U	/* queue 1 start */
#define	TXCS_Q0S	0x00000100U	/* queue 0 start */
#define		TXCS_QnS(n)	(TXCS_Q0S << (n))
#define	TXCS_FT		0x000000c0U	/* fifo threashold */
#define		TXCS_FT_SHIFT	6
#define		TXCS_FT_4QW	(0U << TXCS_FT_SHIFT)
#define		TXCS_FT_8QW	(1U << TXCS_FT_SHIFT)
#define		TXCS_FT_12QW	(2U << TXCS_FT_SHIFT)
#define		TXCS_FT_16QW	(3U << TXCS_FT_SHIFT)
#define	TXCS_DS		0x00000030U	/* dma size */
#define		TXCS_DS_SHIFT	4
#define		TXCS_DS_64B	(0U << TXCS_DS_SHIFT)	/* 64 byte */
#define		TXCS_DS_128B	(1U << TXCS_DS_SHIFT)	/* 128 byte */
#define		TXCS_DS_256B	(2U << TXCS_DS_SHIFT)	/* 256 byte */
#define		TXCS_DS_512B	(3U << TXCS_DS_SHIFT)	/* 512 byte */
#define	TXCS_BR		0x00000004U	/* burst read */
#define	TXCS_SP		0x00000002U	/* suspend */
#define	TXCS_EN		0x00000001U	/* enable */

/* TXQDC: tx queue descriptor count, offset 0x0c */
#define	TXQDC_DCNT	0x000007f0U	/* descriptor count */

/* TXNDA: tx queue next descriptor address, offset 0x10 */
#define	TXNDA_NWDA	0xfffffff0U	/* next descriptor address */
#define	TXNDA_NDE	0x00000008U	/* next descriptor empty */
#define	TXNDA_NDV	0x00000004U	/* next descriptor valid */
#define	TXNDA_NDW	0x00000002U	/* next descriptor waiting */
#define	TXNDA_NDF	0x00000001U	/* next descriptor fetching */

/* TXMCS: tx mac control and status , offset 0x14 */
#define	TXMCS_IFG2	0xc0000000U	/* internal frame gap */
#define		TXMCS_IFG2_SHIFT	30
#define		TXMCS_IFG2_6_4	(0U << TXMCS_IFG2_SHIFT)
#define		TXMCS_IFG2_8_5	(1U << TXMCS_IFG2_SHIFT)
#define		TXMCS_IFG2_10_6	(2U << TXMCS_IFG2_SHIFT)
#define		TXMCS_IFG2_12_7	(3U << TXMCS_IFG2_SHIFT)
#define		TXMCS_IFG2_STD	TXMCS_IFG2_8_5
#define	TXMCS_IFG1	0x30000000U	/* internal frame gap */
#define		TXMCS_IFG1_SHIFT	28
#define		TXMCS_IFG1_8_4	(0U << TXMCS_IFG1_SHIFT)
#define		TXMCS_IFG1_12_6	(1U << TXMCS_IFG1_SHIFT)
#define		TXMCS_IFG1_16_8	(2U << TXMCS_IFG1_SHIFT)
#define		TXMCS_IFG1_20_10 (3U << TXMCS_IFG1_SHIFT)
#define		TXMCS_IFG1_STD	TXMCS_IFG1_16_8
#define	TXMCS_PSEPKT	0x00ff0000U	/* pause packet sent count*/
#define		TXMCS_PSEPKT_SHIFT	16
#define	TXMCS_THOLD	0x00000300U	/* transmit threshold */
#define		TXMCS_THOLD_SHIFT	8
#define		TXMCS_THOLD_SF	(3U << TXMCS_THOLD_SHIFT)
#define		TXMCS_THOLD_1_2	(2U << TXMCS_THOLD_SHIFT)
#define		TXMCS_THOLD_1_4	(1U << TXMCS_THOLD_SHIFT)
#define		TXMCS_THOLD_1_8	(0U << TXMCS_THOLD_SHIFT)
#define	TXMCS_BSTEN	0x00000080U	/* frame bursting enable */
#define	TXMCS_EXTEN	0x00000040U	/* carrier extention enable */
#define	TXMCS_DEFEN	0x00000020U	/* defer enable */
#define	TXMCS_BKFEN	0x00000010U	/* back off enable */
#define	TXMCS_CRSEN	0x00000008U	/* carrier sense enable */
#define	TXMCS_COLEN	0x00000004U	/* collision enable */
#define	TXMCS_CRCEN	0x00000002U	/* CRC enable */
#define	TXMCS_PADEN	0x00000001U	/* padding enable */

#define	TXMCS_BITS	"\020"	\
	"\010BSTEn"	\
	"\007EXTEn"	\
	"\006DEFEn"	\
	"\005BKFEn"	\
	"\004CRSEn"	\
	"\003COLEn"	\
	"\002CRCEn"	\
	"\001PADEn"

/* TXPFC: tx pause frame control, offset 0x18 */
#define	TXPFC_VT	0xffff0000U	/* vlan tag for pause */
#define	TXPFC_VLANEN	0x00008000U	/* vlan enable for pause */
#define	TXPFC_FCRXSL2	0x00004000U	/* rxs fifo low water mark */
#define	TXPFC_FCRXSH2	0x00002000U	/* rxs fifo high water mark */
#define	TXPFC_FCRXSL0	0x00001800U	/* rxs fifo low water mark */
#define	TXPFC_FCRXSH0	0x00000600U	/* rxs fifo high water mark */
#define	TXPFC_FCRXS	0x00000100U	/* enable fc by rxs fifo's  wm */
#define	TXPFC_FCRXDL	0x000000e0U	/* ctrl of rxd fifo's low wm */
#define	TXPFC_FCRXDH	0x0000001cU	/* ctrl of rxd fifo's high wm */
#define	TXPFC_FCRXD	0x00000002U	/* enable fc by rxd fifo's wm */
#define	TXPFC_PFEN	0x00000001U	/* pause frame enable */
#define		TXPFC_FCRXSL(n)		(((n) & 4) << 14 | ((n) & 3) << 11)
#define		TXPFC_FCRXSH(n)		(((n) & 4) << 13 | ((n) & 3) << 9)

/* TXTRHD: timer/retry, offset 0x1c */
#define	TXTRHD_TXPEN	0x80000000U	/* retry tranmit period enable */
#define	TXTRHD_TXP	0x7fffff00U	/* retry tranmit period */
#define		TXTRHD_TXP_SHIFT	8
#define	TXTRHD_TXREN	0x00000080U	/* transmit retry enable */
#define	TXTRHD_TXRL	0x0000007fU	/* transmit retry limit */

/* RXCS: rx control and status, offset 0x00 */
#define	RXCS_REO	0x40000000U	/* rx empty option */
#define	RXCS_FF		0x30000000U	/* fifo full hiwat for tx pause pkt */
#define		RXCS_FF_SHIFT	28
#define		RXCS_FF_16T 	(0U << RXCS_FF_SHIFT)
#define		RXCS_FF_32T 	(1U << RXCS_FF_SHIFT)
#define		RXCS_FF_64T 	(2U << RXCS_FF_SHIFT)
#define		RXCS_FF_128T 	(3U << RXCS_FF_SHIFT)
#define	RXCS_FT		0x0c000000U	/* fifo threshold for next rx pkt */
#define		RXCS_FT_SHIFT	26
#define		RXCS_FT_16QW 	(0U << RXCS_FT_SHIFT)
#define		RXCS_FT_32QW 	(1U << RXCS_FT_SHIFT)
#define		RXCS_FT_64QW 	(2U << RXCS_FT_SHIFT)
#define		RXCS_FT_128QW 	(3U << RXCS_FT_SHIFT)
#define	RXCS_DRS	0x03000000U	/* dma request size in byte */
#define		RXCS_DRS_SHIFT	24
#define		RXCS_DRS_16B 	(0U << RXCS_DRS_SHIFT)
#define		RXCS_DRS_32B 	(1U << RXCS_DRS_SHIFT)
#define		RXCS_DRS_64B 	(2U << RXCS_DRS_SHIFT)
#define		RXCS_DRS_128B 	(3U << RXCS_DRS_SHIFT)
#define	RXCS_QSEL	0x00030000U	/* queue selection */
#define		RXCS_QSEL_SHIFT		16
#define	RXCS_RG		0x0000f000U	/* load rx descriptor retry time-gap */
#define		RXCS_RG_SHIFT	12
#define		RXCS_RG_256nS	(0U << RXCS_RG_SHIFT)
#define		RXCS_RG_512nS	(1U << RXCS_RG_SHIFT)
#define		RXCS_RG_1024nS	(2U << RXCS_RG_SHIFT)
#define		RXCS_RG_2048nS	(3U << RXCS_RG_SHIFT)
#define		RXCS_RG_4096nS	(4U << RXCS_RG_SHIFT)
#define		RXCS_RG_8192nS	(5U << RXCS_RG_SHIFT)
#define		RXCS_RG_16384nS	(6U << RXCS_RG_SHIFT)
#define		RXCS_RG_32768nS	(7U << RXCS_RG_SHIFT)
#define	RXCS_RC		0x00000f00U	/* load rx descriptor retry counter */
#define		RXCS_RC_SHIFT	8
#define		RXCS_RCn(n)	(((n) / 4) << RXCS_RG_SHIFT)
#define	RXCS_PFREN	0x00000080U	/* pause frame rx enable */
#define	RXCS_WPREN	0x00000040U	/* wakeup packet received enable */
#define	RXCS_MPREN	0x00000020U	/* magic packet received enable */
#define	RXCS_SPREN	0x00000010U	/* short packet received enable */
#define	RXCS_ABORT	0x00000008U	/* abort */
#define	RXCS_QST	0x00000004U	/* queue start */
#define	RXCS_SP		0x00000002U	/* suspend */
#define	RXCS_EN		0x00000001U	/* enable */

/* RXQDC: rx queue descriptor count, offset 0x2c */
#define	RXQDC_RXSWT	0x08000000U	/* status write back timing */
#define	RXQDC_R_SPE_SEL	0x04000000U
#define	RXQDC_DCNT	0x000007f0U	/* descriptor count */
#define		RXQDC_DCNT_MAX	1024

/* RXNDA: rx queue next descriptor address , offset 0x30 */
#define	RXNDA_NWDA	0xfffffff0U	/* next descriptor address */
#define	RXNDA_NDE	0x00000008U	/* next descriptor empty */
#define	RXNDA_NDV	0x00000004U	/* next descriptor valid */
#define	RXNDA_NDW	0x00000002U	/* next descriptor waiting */
#define	RXNDA_NDF	0x00000001U	/* next descriptor fetching */

/* RXMCS: rx mac control and status, offset 0x34 */
#define	RXMCS_CSX	0x00f00000U	/* RSS packet filter */
#define		RXMCS_CSX_TCPv6	(8 << 20)
#define		RXMCS_CSX_TCPv4	(4 << 20)
#define		RXMCS_CSX_UDPv6	(2 << 20)
#define		RXMCS_CSX_UDPv4	(1 << 20)
#define	RXMCS_RSSPF	0x00030000U	/* RSS packet filter */
#define	RXMCS_RSSPF_SHIFT	16
#define		RXMCS_RSSPF_U	(0U << RXMCS_RSSPF_SHIFT)
#define		RXMCS_RSSPF_UB	(1U << RXMCS_RSSPF_SHIFT)
#define		RXMCS_RSSPF_UBM	(2U << RXMCS_RSSPF_SHIFT)
#define		RXMCS_RSSPF_ALL	(3U << RXMCS_RSSPF_SHIFT)
#define	RXMCS_AF	0x00000800U	/* all frame reception */
#define	RXMCS_BF	0x00000400U	/* broadcast frame reception */
#define	RXMCS_FFHTEN	0x00000200U	/* multicast hash table enable */
#define	RXMCS_UF	0x00000100U	/* unicast frame reception */
#define	RXMCS_AMF	0x00000080U	/* all multicast frame reception */
#define	RXMCS_MF	0x00000040U	/* multicast frame reception */
#define	RXMCS_RCDEN	0x00000020U	/* receive collision detected packets */
#define	RXMCS_FCEN	0x00000008U	/* flow control enable */
#define	RXMCS_TAGRM	0x00000004U	/* VLAN tag removal enable */
#define	RXMCS_POEN	0x00000002U	/* insert 10byte pad before rx pkts */
#define	RXMCS_CSEN	0x00000001U	/* checksum offload enable */

/* WFOI: wakeup frame output interface, offset 0x4c */
#define	WFOI_MBSEL	0x00000070U	/* mask bit select */
#define		WFOI_MBSEL_SHIFT	4
#define		WFOI_MBSEL_31_0		(0U << WFOI_MBSEL_SHIFT)
#define		WFOI_MBSEL_63_32	(1U << WFOI_MBSEL_SHIFT)
#define		WFOI_MBSEL_95_64	(2U << WFOI_MBSEL_SHIFT)
#define		WFOI_MBSEL_127_96	(3U << WFOI_MBSEL_SHIFT)
#define	WFOI_MCSEL	0x00000008U	/* mask/crc select */
#define	WFOI_WFSEL	0x00000007U	/* wakeup frame select */

/* SMI: station management interface, offset 0x50 */
#define	SMI_DATA	0xffff0000U	/* SMI data port */
#define		SMI_DATA_SHIFT	16
#define	SMI_REGADDR	0x0000f800U	/* SMI register address */
#define		SMI_REGADDR_SHIFT	11
#define	SMI_PHYADDR	0x000007c0U	/* SMI phy address */
#define		SMI_PHYADDR_SHIFT	6
#define	SMI_WR		0x00000020U	/* SMI write */
#define	SMI_REQ		0x00000010U	/* SMI request */
#define	SMI_MDIO	0x00000008U	/* SMI MDIO */
#define	SMI_MODE	0x00000004U	/* SMI MODE */
#define	SMI_MDC		0x00000002U	/* SMI MDC */
#define	SMI_MDEN	0x00000001U	/* SMI MDEN */

/* GHC: global host control, offset 0x54 */
#define	GHC_LOOPBACK	0x80000000U	/* loopback enable */
#define	GHC_SWRST	0x40000000U	/* software reset */
#define GHC_PSRXIDL	0x04000000U	/* power save rx dma */
#define GHC_DISPSFIFORD	0x02000000U	/* disable power save fifo rd port */
#define GHC_DISPSFIFOWT	0x01000000U	/* disable power save fifo write port */
#define GHC_TCPCKSRC	0x00c00000U
#define		GHC_TCPCKSRC_SHIFT	22
#define		GHC_TCPCKSRC_OFF	(0U << GHC_TCPCKSRC_SHIFT)
#define		GHC_TCPCKSRC_GPHY	(1U << GHC_TCPCKSRC_SHIFT)
#define		GHC_TCPCKSRC_PCIE	(2U << GHC_TCPCKSRC_SHIFT)
#define		GHC_TCPCKSRC_INVALID	(3U << GHC_TCPCKSRC_SHIFT)
#define GHC_TXCKSRC	0x00300000U
#define		GHC_TXCKSRC_SHIFT	20
#define		GHC_TXCKSRC_OFF		(0U << GHC_TXCKSRC_SHIFT)
#define		GHC_TXCKSRC_GPHY	(1U << GHC_TXCKSRC_SHIFT)
#define		GHC_TXCKSRC_PCIE	(2U << GHC_TXCKSRC_SHIFT)
#define		GHC_TXCKSRC_INVALID	(3U << GHC_TXCKSRC_SHIFT)
#define	GHC_DISAMP	0x00000080U	/* disable auto mapping phy status */
#define	GHC_DPX		0x00000040U	/* duplex mode 1:full, 0:half */
#define	GHC_SPD		0x00000030U	/* speed */
#define		GHC_SPD_SHIFT	4
#define		GHC_SPD_10M	(1U << GHC_SPD_SHIFT)
#define		GHC_SPD_100M	(2U << GHC_SPD_SHIFT)
#define		GHC_SPD_1000M	(3U << GHC_SPD_SHIFT)
#define	GHC_LNKOFF	0x00000004U	/* link off indicator (ro) */
#define	GHC_LNKON	0x00000002U	/* link on indicator (ro) */
#define	GHC_LSPEN	0x00000001U	/* link status polling enable */

#define	GHC_BITS	"\020"	\
	"\040LOOPBACK"	\
	"\037SWRST"	\
	"\007DPX"	\
	"\006SPD"	\
	"\003LNKOFF"	\
	"\002LNKON"	\
	"\001LSPEN"

/* PMCS: power manaement control and status, offset 0x60 */

/* PHYPWR: phy power down register */
#define	PHYPWR_CLK_SEL		0x08000000U
#define	PHYPWR_DOWN2		0x04000000U
#define	PHYPWR_DOWN1_SW		0x02000000U
#define	PHYPWR_DOWN1_SEL	0x01000000U

/* GIGACSR: Giga PHY control and status register */
#define	GIGACSR_GIGAMS		0x40000000U
#define	GIGACSR_GIGAMC		0x30000000U
#define	GIGACSR_TXC		0x0f000000U	/* tx clk phase control */
#define	GIGACSR_RXC		0x00f00000U	/* rx clk phase control */
#define	GIGACSR_TXCINV		0x00080000U	/* tx clk inverse */
#define	GIGACSR_RXCINV		0x00040000U	/* rx clk inverse */
#define	GIGACSR_RST		0x00010000U	/* */
#define	GIGACSR_BISTOK		0x00000200U	/* */
#define	GIGACSR_BISTDONE	0x00000100U	/* */
#define	GIGACSR_BISTLED		0x00000010U	/* */
#define	GIGACSR_BISTCTRL	0x00000003U	/* */

/* LNKSTS: PHY link status register */
#define	LNKSTS_SPD	0x0000c000U
#define	LNKSTS_DPX	0x00002000U
#define	LNKSTS_PRS	0x00001000U
#define	LNKSTS_SDRS	0x00000800U
#define	LNKSTS_LINK	0x00000400U
#define	LNKSTS_ANCMP	0x00000200U
#define	LNKSTS_MCS	0x00000040U
#define	LNKSTS_APAUSE	0x00000002U
#define	LNKSTS_PAUSE	0x00000001U

/* SMBCSR: SMB control and status register */
#define	SMBCSR_SLVADDR	0x7f000000U
#define		SMBCSR_SLVADDR_SHIFT	24
#define	SMBCSR_DNACK	0x00040000U	/* (rw1c) */
#define	SMBCSR_CNACK	0x00020000U	/* (rw1c) */
#define	SMBCSR_RELOAD	0x00010000U
#define	SMBCSR_CURADDR	0x0000ff00U	/* (ro) */
#define		SMBCSR_CURADDR_SHIFT	8
#define	SMBCSR_SCLSTS	0x00000080U	/* (ro) */
#define	SMBCSR_SDASTS	0x00000040U	/* (ro) */
#define	SMBCSR_EEPROMD	0x00000020U	/* (ro) */
#define	SMBCSR_INIDONE	0x00000010U	/* (ro) */
#define	SMBCSR_BUSY	0x0000000fU	/* (ro) */

/* SMBINTF: SMB interface register */
#define	SMBINTF_HWDATR	0xff000000U	/* ro */
#define	SMBINTF_HWDATW	0x00ff0000U	/* rw */
#define	SMBINTF_HWADDR	0x0000ff00U	/* rw */
#define	SMBINTF_HWRWN	0x00000020U	/* rw */
#define	SMBINTF_HWCMD	0x00000010U	/* rw */
#define	SMBINTF_FASTM	0x00000008U	/* rw */
#define	SMBINTF_GPIOSCL	0x00000004U	/* rw */
#define	SMBINTF_GPIOSDA	0x00000002U	/* rw */
#define	SMBINTF_GPIOEN	0x00000001U	/* rw */

/*
 * MISC registers
 */
/* TMCSR: timer control register, offset 0x00 */
#define	TMCSR_SWIT	0x80000000U	/* sw intr trigger */
#define	TMCSR_ISTS	0x10000000U	/* sw intr status */
#define	TMCSR_EN	0x01000000U	/* timer enable */
#define	TMCSR_CNT	0x00ffffffU	/* timer count */

/* GPIO: GPIO control/status, offset 0x04 */
#define	GPIO_GP3_I	0x80000000U
#define	GPIO_GP4_I	0x40000000U
#define	GPIO_GP4_O	0x20000000U
#define	GPIO_GP4_OE	0x10000000U
#define	GPIO_GP3_O	0x08000000U
#define	GPIO_GP3_OE	0x04000000U
#define	GPIO_GP34_GPIO	0x02000000U
#define	GPIO_GP34_C	0x3e000000U
#define	GPIO_GP2_I	0x00100000U
#define	GPIO_GP2_O	0x00040000U
#define	GPIO_GP2_OE	0x00020000U
#define	GPIO_GP1_I	0x00001000U
#define	GPIO_GP1_O	0x00000400U
#define	GPIO_GP1_OE	0x00000200U
#define	GPIO_GP1_EN	0x00000100U
#define	GPIO_GP0_I	0x00000010U
#define	GPIO_GP0_O	0x00000004U
#define	GPIO_GP0_OE	0x00000002U
#define	GPIO_GP0_EN	0x00000001U

/* GPREG0: */
#define	GPREG0_DISSH	0xff000000U
#define		GPREG0_DISSH_DW7	0x80000000U
#define		GPREG0_DISSH_DW6	0x40000000U
#define		GPREG0_DISSH_DW5	0x20000000U
#define		GPREG0_DISSH_DW4	0x10000000U
#define		GPREG0_DISSH_DW3	0x08000000U
#define		GPREG0_DISSH_DW2	0x04000000U
#define		GPREG0_DISSH_DW1	0x02000000U
#define		GPREG0_DISSH_DW0	0x01000000U
#define	GPREG0_PCIRLMT	0x00300000U
#define		GPREG0_PCIRLMT_SHIFT	20
#define		GPREG0_PCIRLMT_8	(0U << GPREG0_PCIRLMT_SHIFT)
#define		GPREG0_PCIRLMT_6	(1U << GPREG0_PCIRLMT_SHIFT)
#define		GPREG0_PCIRLMT_5	(2U << GPREG0_PCIRLMT_SHIFT)
#define		GPREG0_PCIRLMT_4	(3U << GPREG0_PCIRLMT_SHIFT)
#define	GPREG0_ENSC	0x00040000U
#define	GPREG0_DISPCCMC	0x00020000U
#define	GPREG0_FCSLO	0x00010000U
#define	GPREG0_DDLRSTd	0x00008000U
#define	GPREG0_STICKYRST 0x00004000U
#define	GPREG0_DDLRST	0x00002000U
#define	GPREG0_LINTS	0x00001000U	/* 0: direct, 1:polling */
#define	GPREG0_MSIGSEL	0x00000800U
#define	GPREG0_DSMBPU	0x00000400U
#define	GPREG0_PCCTU	0x00000300U
#define		GPREG0_PCCTU_SHIFT	8
#define		GPREG0_PCCTU_16nS	(0U << GPREG0_PCCTU_SHIFT)
#define		GPREG0_PCCTU_256nS	(1U << GPREG0_PCCTU_SHIFT)
#define		GPREG0_PCCTU_1uS	(2U << GPREG0_PCCTU_SHIFT)
#define		GPREG0_PCCTU_1mS	(3U << GPREG0_PCCTU_SHIFT)
#define	GPREG0_ENIEVES	0x00000080U
#define	GPREG0_ENPMEM	0x00000020U
#define	GPREG0_PHYADDR	0x0000001fU

/* GPREG1: */
#define	GPREG1_dis_clk_rx	0x04000000U	/* */
#define	GPREG1_PCREQN		0x00020000U
#define	GPREG1_HALFMODEPATCH	0x00000040U
#define	GPREG1_RSSPATCH		0x00000020U
#define	GPREG1_INTDTI	0x00000018U	/* interrupt delay unit */
#define		GPREG1_INTDTI_SHIFT	3
#define		GPREG1_INTDTI_16nS	(0U << GPREG1_INTDTI_SHIFT)
#define		GPREG1_INTDTI_256nS	(1U << GPREG1_INTDTI_SHIFT)
#define		GPREG1_INTDTI_1uS	(2U << GPREG1_INTDTI_SHIFT)
#define		GPREG1_INTDTI_16uS	(3U << GPREG1_INTDTI_SHIFT)
#define	GPREG1_INTDEN	0x00000007U	/* interrupt delay enable */

/* IEVE: interrupt event status offset 0x20 */
#define	IEVE_SW		0x80000000U	/* software interrupt */
#define	IEVE_TM		0x40000000U	/* timer-up interrupt */
#define	IEVE_LINK	0x20000000U	/* link status change */
#define	IEVE_RXPF	0x10000000U	/* pause frame received */
#define	IEVE_RXMF	0x08000000U	/* magic packet received */
#define	IEVE_RXWF	0x04000000U	/* wakeup Frame received */
#define	IEVE_PCCRX0TO	0x02000000U	/* rx timeout for Q0 */
#define	IEVE_PCCRX1TO	0x01000000U	/* rx timeout for Q1 */
#define	IEVE_PCCRX2TO	0x00800000U	/* rx timeout for Q2 */
#define	IEVE_PCCRX3TO	0x00400000U	/* rx timeout for Q3 */
#define		IEVE_PCCRXTO(n)	(IEVE_PCCRX0TO >> (n))
#define	IEVE_PCCTXTO	0x00200000U	/* tx timeout */
#define	IEVE_PCCRX0	0x00100000U	/* rx count coalescing for Q0 */
#define	IEVE_PCCRX1	0x00080000U	/* rx count coalescing for Q1 */
#define	IEVE_PCCRX2	0x00040000U	/* rx count coalescing for Q2 */
#define	IEVE_PCCRX3	0x00020000U	/* rx count coalescing for Q3 */
#define		IEVE_PCCRX(n)	(IEVE_PCCRX0 >> (n))
#define	IEVE_PCCTX	0x00010000U	/* tx count */
#define	IEVE_RX3EMP	0x00008000U	/* rx3 empty */
#define	IEVE_RX2EMP	0x00004000U	/* rx2 empty */
#define	IEVE_RX1EMP	0x00002000U	/* rx1 empty */
#define	IEVE_RX0EMP	0x00001000U	/* rx0 empty */
#define		IEVE_RXEMP(n)	(IEVE_RX0EMP << (n))
#define	IEVE_RX3	0x00000800U
#define	IEVE_RX2	0x00000400U
#define	IEVE_RX1	0x00000200U
#define	IEVE_RX0	0x00000100U
#define		IEVE_RX(n)	(IEVE_RX0 << (n))
#define	IEVE_TX7	0x00000080U
#define	IEVE_TX6	0x00000040U
#define	IEVE_TX5	0x00000020U
#define	IEVE_TX4	0x00000010U
#define	IEVE_TX3	0x00000008U
#define	IEVE_TX2	0x00000004U
#define	IEVE_TX1	0x00000002U
#define	IEVE_TX0	0x00000001U
#define		IEVE_TX(n)	(IEVE_TX0 << (n))	/* Tx done */

#define	IEVE_BITS	"\020"	\
	"\040SW"	\
	"\037TM"	\
	"\036LINK"	\
	"\035RxPF"	\
	"\034RxMF"	\
	"\033RxWF"	\
	"\032PccRx0TO"	\
	"\031PccRx1TO"	\
	"\030PccRx2TO"	\
	"\027PccRx3TO"	\
	"\026PccTxTO"	\
	"\025PccRx0"	\
	"\024PccRx1"	\
	"\023PccRx2"	\
	"\022PccRx3"	\
	"\021PccTx"	\
	"\020Rx3Emp"	\
	"\017Rx2Emp"	\
	"\016Rx1Emp"	\
	"\015Rx0Emp"	\
	"\014Rx3"	\
	"\013Rx2"	\
	"\012Rx1"	\
	"\011Rx0"	\
	"\010Tx7"	\
	"\007Tx6"	\
	"\006Tx5"	\
	"\005Tx4"	\
	"\004Tx3"	\
	"\003Tx2"	\
	"\002Tx1"	\
	"\001Tx0"

/* SHBALO: shadow status base address low */
#define	SHBALO_LO		0xffffffe0U
#define	SHBALO_POSTFORCE	0x00000002U
#define	SHBALO_POSTEN		0x00000001U

/* PCCRX: packet completion coalescing controling of Rx */
#define	PCCRX(n)	(PCCRX0 + (n)*4)
#define	PCCRX_TO	0xffff0000U	/* time out value */
#define		PCCRX_TO_SHIFT	16
#define	PCCRX_CNT	0x0000ff00U	/* packet count */
#define		PCCRX_CNT_SHIFT	8
#define		PCCRX_CNT_MAX	(PCCRX_CNT >> PCCRX_CNT_SHIFT)

/* PCCTX: packet completion coalescing controling of Tx */
#define	PCCTX_TO	0xffff0000U	/* time out value */
#define		PCCTX_TO_SHIFT	16
#define		PCCTX_TO_MAX	(PCCTX_TO >> PCCTX_TO_SHIFT)
#define	PCCTX_CNT	0x0000ff00U	/* packet count */
#define		PCCTX_CNT_SHIFT	8
#define		PCCTX_CNT_MAX	(PCCTX_CNT >> PCCTX_CNT_SHIFT)
#define	PCCTX_EN7	0x00000080	/* coalescing enable for tx7 */
#define	PCCTX_EN6	0x00000040	/* coalescing enable for tx6 */
#define	PCCTX_EN5	0x00000020	/* coalescing enable for tx5 */
#define	PCCTX_EN4	0x00000010	/* coalescing enable for tx4 */
#define	PCCTX_EN3	0x00000008	/* coalescing enable for tx3 */
#define	PCCTX_EN2	0x00000004	/* coalescing enable for tx2 */
#define	PCCTX_EN1	0x00000002	/* coalescing enable for tx1 */
#define	PCCTX_EN0	0x00000001	/* coalescing enable for tx0 */
#define		PCCTX_EN(n)	(PCCTX_EN0 << (n))

/* CHIPMODE: chip mode and fpga version */
#define	CHIPMODE_FPGA	0xffff0000U	/* fpga version */
#define		CHIPMODE_FPGA_SHIFT	16
#define	CHIPMODE_REV	0x0000ff00U	/* chip revision */
#define		CHIPMODE_REV_SHIFT	8
#define	CHIPMODE_MODE	0x0000000fU	/* chip mode */
#define	CHIPMODE_MODE_48PIN	0xcU
#define	CHIPMODE_MODE_64PIN	0x4U
#define	CHIPMODE_MODE_128MAC	0x3U
#define	CHIPMODE_MODE_128DBG	0x2U
#define	CHIPMODE_MODE_128PHY	0x0U

/* TIMER1: timer register */
#define	TIMER1_EN	0x01000000U
#define	TIMER1_CNT	0x00ffffffU
#define	TIMER1_UNIT	1024		/* in nano second */

/* TIMER2: timer register */
#define	TIMER2_EN	0x01000000U
#define	TIMER2_CNT	0x00ffffffU
#define	TIMER2_UNIT	1024		/* in nano second */

/* APMC: aggressive power mode control */
#define	APMC_PSD_SRS	0x80000000U	/* pcie shutdown status */
#define	APMC_PSD_EN	0x40000000U	/* pcie shutdown enable */
#define	APMC_PHP_EN	0x20000000U	/* pseudo hot-plug enable */
#define	APMC_EPIEN	0x04000000U	/* external plug-in enable */
#define	APMC_EPIENC	0x03000000U	/* EPIC control */
#define	APMC_DISSRAM	0x00000004U
#define	APMC_DISCLKPM	0x00000002U
#define	APMC_DISCLKTX	0x00000001U

/* PCCSRX: packet completion coalescing controling status of Rx */
#define	PCCSRX(n)	(PCCSRX + (n)*4)
#define	PCCSRX_TO	0xffff0000U	/* time out value */
#define		PCCSRX_TO_SHIFT	16
#define	PCCSRX_CNT	0x0000ff00U	/* packet count */
#define		PCCSRX_CNT_SHIFT	8

/* PCCSTX: packet completion coalescing status of tx queues */
#define	PCCSTX_TO	0xffff0000U
#define		PCCSTX_TO_SHIFT	16
#define	PCCSTX_CNT	0x0000ff00U
#define		PCCSTX_CNT_SHIFT	8

/* TX Queues Empty Indicator */
#define	TXEMP_TX7EMP	0x00000080
#define	TXEMP_TX6EMP	0x00000040
#define	TXEMP_TX5EMP	0x00000020
#define	TXEMP_TX4EMP	0x00000010
#define	TXEMP_TX3EMP	0x00000008
#define	TXEMP_TX2EMP	0x00000004
#define	TXEMP_TX1EMP	0x00000002
#define	TXEMP_TX0EMP	0x00000001
#define		TXEMP_TXnEMP(n)	(1 << (n))

/*
 * Tx and Rx descriptors
 */
struct tx_desc {
	volatile uint32_t	td0;	/* control flags */
	volatile uint32_t	td1;	/* buffer length */
	volatile uint32_t	td2;	/* pktsize or 64bit high addr */
	volatile uint32_t	td3;	/* 32bit low addr */
};

#define	TD0_OWN		0x80000000U
#define	TD0_INT		0x40000000U
#define	TD0_64BIT	0x20000000U
#define	TD0_TCPCS	0x10000000U
#define	TD0_UDPCS	0x08000000U
#define	TD0_IPCS	0x04000000U
#define	TD0_LSEN	0x02000000U
#define	TD0_TAGON	0x01000000U
#define	TD0_VTAG	0x0000ffffU
#define	TD0_UP		0x0000e000U
#define	TD0_CFI		0x00001000U
#define	TD0_VLANID	0x00000fffU

#define	TD0_BITS	\
	"\020"	\
	"\040OWN"	\
	"\037INT"	\
	"\03664BIT/TMOUT"	\
	"\035TCPCS/TRYOUT"	\
	"\034UDPCS/COLON"	\
	"\033IPCS"	\
	"\032LSEN"	\
	"\031TAGON"

#define	TD1_MSS		0xfffc0000U
#define		TD1_MSS_SHIFT	18
#define	TD1_BUFLEN	0x0003ffffU

#define	TD2_PKTSZ	0x0003ffffU

/* returned flags */
#define	TD0_TMOUT	0x20000000U
#define	TD0_TRYOUT	0x10000000U
#define	TD0_COLON	0x08000000U
#define	TD0_HDRSZ	0x000000ffU

#define	TD1_SEGCNT	0xffff0000U
#define		TD1_SEGCNT_SHIFT	16
#define	TD1_TSCNT	0x0000ffffU
#define		TD1_TSCNT_SHIFT		0


struct rx_desc {
	volatile uint32_t	rd0;
	volatile uint32_t	rd1;
	volatile uint32_t	rd2;
	volatile uint32_t	rd3;
};

#define	RD0_OWN		0x80000000U
#define	RD0_INT		0x40000000U
#define	RD0_64BIT	0x20000000U

/* returned status */
#define	RD0_MF		0x20000000U	/* fragmented IP */
#define	RD0_TCPON	0x10000000U
#define	RD0_UDPON	0x08000000U
#define	RD0_IPCS	0x04000000U
#define	RD0_TCPCS	0x02000000U
#define	RD0_UDPCS	0x01000000U
#define	RD0_TAGON	0x00800000U
#define	RD0_IPV4	0x00400000U
#define	RD0_IPV6	0x00200000U
#define	RD0_PAUSE	0x00100000U
#define	RD0_MAGIC	0x00080000U
#define	RD0_WAKEUP	0x00040000U
#define	RD0_DEST	0x00030000U
#define		RD0_DEST_SHIFT	16
#define		RD0_DEST_BR	(3U << RD0_DEST_SHIFT)
#define		RD0_DEST_MULT	(2U << RD0_DEST_SHIFT)
#define		RD0_DEST_UNI	(1U << RD0_DEST_SHIFT)
#define	RD0_UP		0x0000e000U
#define	RD0_CFI		0x00001000U
#define	RD0_VLANID	0x00000fffU

#define	RD0_BITS	"\020"	\
	"\040OWN"	\
	"\037INT"	\
	"\036MF/64BIT"	\
	"\035TCPOn"	\
	"\034UDPOn"	\
	"\033IPCS"	\
	"\032TCPCS"	\
	"\031UDPCS"	\
	"\030TagOn"	\
	"\027IPv4"	\
	"\026IPv6"	\
	"\025Pause"	\
	"\024Magic"	\
	"\023WakeUp"

#define	RD1_WBCPL	0x80000000U
#define	RD1_DCNT	0x7f000000U
#define		RD1_DCNT_SHIFT	24
#define	RD1_LIMIT	0x00800000U	/* too long */
#define	RD1_MIIERR	0x00400000U
#define	RD1_NIBON	0x00200000U
#define	RD1_COLON	0x00100000U
#define	RD1_ABORT	0x00080000U
#define	RD1_SHORT	0x00040000U
#define	RD1_OVERUN	0x00020000U
#define	RD1_CRCERR	0x00010000U
#define	RD1_RECVFS	0x0000ffffU	/* received frame size */

#define	RD1_BITS	"\020"	\
	"\040WBCPL"	\
	"\030Limit"	\
	"\027MIIErr"	\
	"\026NIBOn"	\
	"\025COLOn"	\
	"\024Abort"	\
	"\023Short"	\
	"\022Overun"	\
	"\021CRCErr"

#define	RD3_HT		0x00001f00U
#define		RD3_HT_SHIFT	8
#define		RD3_HT_NONE	(0U << RD3_HT_SHIFT)
#define		RD3_HT_IPv4	(1U << RD3_HT_SHIFT)
#define		RD3_HT_IPv4_TCP	(2U << RD3_HT_SHIFT)
#define		RD3_HT_IPv6	(4U << RD3_HT_SHIFT)
#define		RD3_HT_IPv6_TCP	(6U << RD3_HT_SHIFT)
#define	RD3_HF		0x0000000fU
#define		RD3_HF_NONE	0U
#define		RD3_HF_RSS	1U

/* Vendor PHY registers */
#define PHY_SPEC_ADDR_REG	0x1e
#define PHY_SPEC_REG_READ	0x00004000
#define PHY_SPEC_REG_WRITE	0x00008000

#define PHY_CALIBRATION_DELAY	20

#define PHY_SPEC_DATA_REG	0x1f

#define PHY_EXT_COMM_0_REG	0x30
#define PHY_EXT_COMM_1_REG	0x31
#define PHY_EXT_COMM_2_REG	0x32
#define PHY_EXT_COMM_2_CALI_ENABLE	0x01
#define PHY_EXT_COMM_2_CALI_MODE_0	0x02
#define PHY_EXT_COMM_2_CALI_LATCH	0x10

/* Vendor PCI registers */
#define	PCI_PRIV_PE1	0xe4

#define	PE1_REVID	0xff000000U
#define	PE1_GPREG1	0x00ff0000U
#define	PE1_GPREG0	0x0000ff00U
#define	PE1_GPREG0_PBG	0x0000c000U
#define		PE1_GPREG0_PBG_SHIFT	14
#define		PE1_GPREG0_ENBG		(0U << PE1_GPREG0_PBG_SHIFT)
#define		PE1_GPREG0_PDD3COLD	(1U << PE1_GPREG0_PBG_SHIFT)
#define		PE1_GPREG0_PDPCIESD	(2U << PE1_GPREG0_PBG_SHIFT)
#define		PE1_GPREG0_PDPCIEIDDQ	(3U << PE1_GPREG0_PBG_SHIFT)
#define	PE1_ASPMOPTH	0x000000c0U
#define	PE1_ASPMOPTL	0x00000030U
#define	PE1_RDYDMA	0x00000008U
#define	PE1_MULTIFUN	0x00000004U
#define	PE1_ASPMSUPRT	0x00000003U


#define PCI_PRIV_SHARE_NICCTRL	0xf5
#define FLAG_PHYEA_ENABLE	0x2


#endif	/* _JMC250REG_H_ */
