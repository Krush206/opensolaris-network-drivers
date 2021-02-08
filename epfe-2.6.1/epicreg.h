/*
 * @(#)epicreg.h	1.4 09/02/11
 * SMC LAN83C171 EPIC/XF Fast Ethermet Controler register definisions
 * This file is public domain. Coded by M. Murayama
 */

/* register offset */
#define	COMMAND	0x00	/* command register */
#define	INTSTAT	0x04	/* interrupt status register */
#define	INTMASK	0x08	/* interrupt MASK register */
#define	GENCTL	0x0c	/* general control register */
#define	NVCTL	0x10	/* non-volatile control register */
#define	EECTL	0x14	/* eeprom control register */
#define	PBLCNT	0x18	/* PCI burst length count */
#define	TEST1	0x1c	/* test register 1 */
#define	CRCCNT	0x20	/* CRC error count register */
#define	ALICNT	0x24	/* Frame alignment error counter */
#define	MPCNT	0x28	/* Missed packet counter */
#define	RXFIFO	0x2c	/* receive fifo control */
#define	MMCTL	0x30	/* MII control register */
#define	MMDATA	0x34	/* MII data register */
#define	MIICFG	0x38	/* MII configuration register */
#define	IPG	0x3c	/* inter packet gap register */
#define	LAN0	0x40	/* mac address register */
#define	ID_CHK	0x4c	/* board id regsiter */
#define	MC0	0x50	/* multicast hash table register */
#define	RXCON	0x60	/* receive control register */
#define	RXSTAT	0x64	/* receive status register */
#define	RXCNT	0x68	/* receive byte count register */
#define	RXTEST	0x6c	/* receive test register */
#define	TXCON	0x70	/* transmit control register */
#define	TXSTAT	0x74	/* transmit status register */
#define	TDPAR	0x78	/* transmit packet address register */
#define	TXTEST	0x7c	/* transmit test register */
#define	PRFDAR	0x80	/* PCI receive first descriptor address */
#define	PRCDAR	0x84	/* PCI receive current descriptor address */
#define	PRSTAT	0xa4	/* PCI receive stat */
#define	PRCPTHR	0xb0	/* PCI receive copy threshold */
#define	ROMDATA	0xb4	/* */
#define	PTFDAR	0xc0	/* PCI transmit first descriptor address */
#define	PTCDAR	0xc4	/* PCI transmit current descriptor address */
#define	ETXTHR	0xdc	/* early transmit threshold */


/* COMMAND register */
#define	CMD_TXUGO	0x0080
#define	CMD_STOP_RDMA	0x0040	/* Stop receive side  dma */
#define	CMD_STOP_TDMA	0x0020	/* Stop transmit side dma */
#define	CMD_NEXTFRAME	0x0010	/* receive next packet */
#define	CMD_RXQUEUED	0x0008
#define	CMD_TXQUEUED	0x0004
#define	CMD_START_RX	0x0002	/* start receiver */
#define	CMD_STOP_RX	0x0001	/* stop receiver */

#define	CMD_BITS	"\020"	\
	"\010TxUGo"	\
	"\007StopRxDMA"	\
	"\006StopTxDMA"	\
	"\005NextFrame"	\
	"\004RxQueued"	\
	"\003TxQueued"	\
	"\002StartRx"	\
	"\001StopRx"

/* Interrupt Status register */
#define	INT_PTA_171	0x08000000	/* PCI target abort (171-5)*/
#define	INT_PMA_171	0x04000000	/* PCI master abort (171-5)*/
#define	INT_APE_171	0x02000000	/* PCI address parity error (171-5)*/
#define	INT_DPE_171	0x01000000	/* PCI data parity error (171-5)*/
#define	INT_RSV		0x00800000	/* Receive status valid */
#define	INT_RCTS	0x00400000	/* Receive copy threshold status */
#define	INT_RBE		0x00200000	/* Receive buffer empty */
#define	INT_TCIP	0x00100000	/* transmit copy in progress */
#define	INT_RCIP	0x00080000	/* receive copy in progress */
#define	INT_TXIDLE	0x00040000	/* transmit idle */
#define	INT_RXIDLE	0x00020000	/* receive idle */
#define	INT_INT_ACTV	0x00010000	/* interrupt active */
#define	INT_GP2_INT	0x00008000	/* GPIO(2) is low */
#define	INT_PME		0x00004000	/* power management event (171-5) */
#define	INT_PTA_170	0x00004000	/* PCI Target Abort (170) */
#define	INT_PMA_170	0x00002000	/* PCI Master Abort (170) */
#define	INT_APE_170	0x00001000	/* PCI address parity error */
#define	INT_FATAL_INT	0x00001000	/* fatal error (27:24) occured (171-5)*/
#define	INT_RCT_171	0x00000800	/* rx copy threshold crossed (171-5)*/
#define	INT_DPE_170	0x00000800	/* PCI data parity error (170)*/
#define	INT_PREI	0x00000400	/* */
#define	INT_CNT		0x00000200	/* counter overflow */
#define	INT_TXU		0x00000100	/* transmit underrun */
#define	INT_TQE		0x00000080	/* transmit queue empty */
#define	INT_TCC		0x00000040	/* transmit chain complete */
#define	INT_TXC		0x00000020	/* transmit complete */
#define	INT_RXE		0x00000010	/* receive error */
#define	INT_OVW		0x00000008	/* receive buffer overflow */
#define	INT_RQE		0x00000004	/* receive queue empty */
#define	INT_HCC		0x00000002	/* header copy complete */
#define	INT_RCC		0x00000001	/* receive copy complete */

#define	INT_BITS_170	"\020"	\
	"\030RSV"	\
	"\027RCTS"	\
	"\026RBE"	\
	"\025TCIP"	\
	"\024RCIP"	\
	"\023TXIDLE"	\
	"\022RXIDLE"	\
	"\021INT_ACTV"	\
	"\020PT"	\
	"\017PTA"	\
	"\016PMA"	\
	"\015APE"	\
	"\014DPE"	\
	"\013PREI"	\
	"\012CNT"	\
	"\011TQU"	\
	"\010TQE"	\
	"\007TCC"	\
	"\006TXC"	\
	"\005RXE"	\
	"\004OVW"	\
	"\003RQE"	\
	"\002HCC"	\
	"\001RCC"


#define	INT_BITS_171	"\020"	\
	"\034PTA"	\
	"\033PMA"	\
	"\032APE"	\
	"\031DPE"	\
	"\030RSV"	\
	"\027RCTS"	\
	"\026RBE"	\
	"\025TCIP"	\
	"\024RCIP"	\
	"\023TXIDLE"	\
	"\022RXIDLE"	\
	"\021INT_ACTV"	\
	"\020GP2_INT"	\
	"\017PME"	\
	"\015FATAL_INT"	\
	"\014RCT"	\
	"\013PREI"	\
	"\012CNT"	\
	"\011TQU"	\
	"\010TQE"	\
	"\007TCC"	\
	"\006TXC"	\
	"\005RXE"	\
	"\004OVW"	\
	"\003RQE"	\
	"\002HCC"	\
	"\001RCC"


/* General Control register */
#define	GC_D3_PWR	0x00400000	/* */
#define	GC_PWR_STATE	0x00300000	/* */
#define	GC_PMA_INTA_EN	0x00080000	/* */
#define	GC_MAGIC	0x00040000	/* magic packet enable */
#define	GC_RESET_DIS	0x00020000	/* reset disable */
#define	GC_SCLK_EN	0x00010000	/* system clock enable when pwr down */
#define	GC_RESET_PHY	0x00004000	/* reset PHY */
#define	GC_SOFT		0x00003000	/* */
#define		GC_SOFT_SHIFT	12
#define	GC_MEMRD_LINE	0x00000800	/* memrd line */
#define	GC_MEMRD_MULT	0x00000400	/* memrd multiple */
#define	GC_RXFIFO	0x00000300	/* receive fifo threshold */
#define		GC_RXFIFO_SHIFT	8
#define		GC_RXFIFO_128	(3 << GC_RXFIFO_SHIFT)
#define		GC_RXFIFO_96	(2 << GC_RXFIFO_SHIFT)
#define		GC_RXFIFO_64	(1 << GC_RXFIFO_SHIFT)
#define		GC_RXFIFO_32	(0 << GC_RXFIFO_SHIFT)
#define	GC_TXDMAPRI	0x00000080	/* transmit dma priority */
#define	GC_RXDMAPRI	0x00000040	/* receive dma priority */
#define	GC_BE		0x00000020	/* big endian */
#define	GC_ONECOPY	0x00000010	/* oncopy mode (automatic NEXTFRAME) */
#define	GC_PWRDOWN	0x00000008	/* */
#define	GC_SOFTINT	0x00000004	/* cause software interrupt */
#define	GC_INTEN	0x00000002	/* enable interrupt */
#define	GC_SOFTRESET	0x00000001	/* software reset*/

#define	GENCTL_BITS	\
	"\020"	\
	"\027D3_PWR"	\
	"\024PMA_INTA_EN"	\
	"\023MAGIC"	\
	"\022RESET_DIS"	\
	"\021SCLK_EN"	\
	"\017RESET_PHY"	\
	"\014MEMRD_LINE"	\
	"\013MEMRD_MULT"	\
	"\010TXDMAPRI"	\
	"\007RXDMAPRI"	\
	"\006BE"	\
	"\005ONECOPY"	\
	"\004PWRDOWN"	\
	"\003SOFTINT"	\
	"\002INTEN"	\
	"\001SOFTRESET"


#define	NVCTL_FETPWRPHY		0x00004000	/* 175 */
#define	NVCTL_MULTI_FUNC	0x00002000	/* 175 */
#define	NVCTL_STSCHG_EN		0x00001000	/* 175 */
#define	NVCTL_PHYPWRDOWN_N	0x00000800	/* 175 */
#define	NVCTL_EN_FBTB		0x00000400	/* 175 */
#define	NVCTL_MODEM_EN		0x00000200	/* 175 */
#define	NVCTL_ROMWR_EN		0x00000100	/* 175 */
#define	NVCTL_ROMSPEED		0x00000080	/* 175 */
#define	NVCTL_STATUSREG_EN	0x00000040	/* 175 */

#define	NVCTL_IPG_DLY		0x00000780	/* 171 */
#define		NVCTL_IPG_DLY_SHIFT	7	/* 171 */
#define	NVCTL_CBMODE		0x00000040	/* 171 */

#define	NVCTL_GPIO2		0x00000020
#define	NVCTL_GPIO1		0x00000010
#define	NVCTL_GPOE2		0x00000008
#define	NVCTL_GPOE1		0x00000004
#define	NVCTL_CLKRUN		0x00000002
#define	NVCTL_MEMMAP		0x00000001

#define	NVCTL_BITS	\
	"\020"	\
	"\007CBMODE"	\
	"\006GPIO2"	\
	"\005GPIO1"	\
	"\004GPOE2"	\
	"\003GPOE1"	\
	"\002CLKRUN"	\
	"\001MEMMAP"

#define	MPCNT_MASK	0x000000ff

/* EEPROM Control register */
#define	EECTL_SIZE	0x40		/* eeprom size (ro) */
#define	EECTL_EERDY	0x20		/* input data is valid (ro) */
#define	EECTL_EEDO	0x10		/* data output from eeprom */
#define		EECTL_EEDO_SHIFT	4
#define	EECTL_EEDI	0x08		/* data input to eeprom */
#define		EECTL_EEDI_SHIFT	3
#define	EECTL_EESK	0x04		/* serial clock to eeprom */
#define	EECTL_EECS	0x02		/* chip select to eeprom */
#define	EECTL_EN	0x01		/* aquire MA pins for eeprom access */

/* MII control register */
#define	MMCTL_PHYADDR	0x3e00		/* phy address field */
#define		MMCTL_PHYADDR_SHIFT	9
#define	MMCTL_PHYREG	0x01f0		/* phy register field */
#define		MMCTL_PHYREG_SHIFT	4
#define	MMCTL_RESPONDER 0x0008		/* */
#define	MMCTL_WRITE	0x0002		/* start write operation */
#define	MMCTL_READ	0x0001		/* start read operation */

/* MII configuration register */
#define	MIICFG_NOTXCLK	0x8000
#define	MIICFG_TXCLKFF	0x4000
#define	MIICFG_ALTDIR	0x0080
#define	MIICFG_ALTDATA	0x0040
#define	MIICFG_ALTCLK	0x0020
#define	MIICFG_EN_SMI	0x0010
#define	MIICFG_PHY_PRE	0x0008
#define	MIICFG_694LINK	0x0004
#define	MIICFG_EN694	0x0002
#define	MIICFG_SM_EN	0x0001	/* 1: 10M, 0:IEEE MII */

#define	MIICFG_BITS	\
	"\020"	\
	"\200NOTXCLK"	\
	"\170TXCLKFF"	\
	"\010ALTDIR"	\
	"\007ALTDATA"	\
	"\006ALTCLK"	\
	"\005EN_SMI"	\
	"\004PHY_PRE"	\
	"\003694LINK"	\
	"\002EN694"	\
	"\001SM_EN"

/* Receive Control register */
#define	RXCON_EXBUF	0x0300	/* external buffer size select */
#define		RXCON_EXBUF_SHIFT	8
#define		RXCON_EXBUF128	(3 << RXCON_EXBUF_SHIFT)
#define		RXCON_EXBUF32	(2 << RXCON_EXBUF_SHIFT)
#define		RXCON_EXBUF16	(1 << RXCON_EXBUF_SHIFT)
#define	RXCON_EARLY	0x0080	/* early receive enable */
#define	RXCON_MONITOR	0x0040	/* monitor mode */
#define	RXCON_PROMISC	0x0020	/* promiscious mode */
#define	RXCON_INV	0x0010	/* receive inverse of indivisual */
#define	RXCON_MULT	0x0008	/* receive multicast packet */
#define	RXCON_BR	0x0004	/* receive broadcast packet */
#define	RXCON_RUNT	0x0002	/* receive runt packet */
#define	RXCON_ERR	0x0001	/* save errored packet */

#define	RXCON_BITS	"\020"	\
	"\010EARLY"	\
	"\007MONITOR"	\
	"\006PROMISC"	\
	"\005INV"	\
	"\004MULT"	\
	"\003BR"	\
	"\002RUNT"	\
	"\001ERR"

/* Receive Status register */
#define	RXSTAT_DIS	0x40		/* receiver disable */
#define	RXSTAT_BR	0x20		/* broadcast recognized */
#define	RXSTAT_MULT	0x10		/* multicast recognized */
#define	RXSTAT_MISSED	0x08		/* missed packet */
#define	RXSTAT_CRC	0x04		/* crc error */
#define	RXSTAT_FAE	0x02		/* frame alignment error */
#define	RXSTAT_PRI	0x01		/* packet receive intact */

#define	RXSTAT_BITS	"\020"	\
	"\007DIS"	\
	"\006BR"	\
	"\005MULT"	\
	"\004MISSED"	\
	"\003CRC"	\
	"\002FAE"	\
	"\001PRI"

/* Transmit Control register */
#define	TXCON_SLOT_MASK	0xf8		/* slot time */
#define		TXCON_SLOT_SHIFT	3
#define		TXCON_SLOT(n)	((((n)/32)-1) << TXCON_SLOT_SHIFT)
#define	TXCON_LPBK	0x06		/* loop back mode */
#define		TXCON_LPBK_SHIFT	1
#define		TXCON_LPBK_INT	(1 << TXCON_LPBK_SHIFT)	/* internal loop back */
#define		TXCON_LPBK_EXT	(2 << TXCON_LPBK_SHIFT)	/* external loop back */
#define		TXCON_LPBK_FDX	(3 << TXCON_LPBK_SHIFT)	/* full duplex */
#define	TXCON_EARLY	0x01		/* early transmit enable */

#define	TXCON_BITS	"\020"	\
	"\001EARLY"

/* Transmit Status register */
#define	TXSTAT_CC	0x1f00		/* collision count */
#define		TXSTAT_CC_SHIFT	8
#define	TXSTAT_DFR	0x0080		/* deferral */
#define	TXSTAT_OWC	0x0040		/* out of window collision */
#define	TXSTAT_CDH	0x0020		/* collision detect heartbeat */
#define	TXSTAT_UN	0x0010		/* underrun */
#define	TXSTAT_CSL	0x0008		/* carrier sense lost */
#define	TXSTAT_COL	0x0004		/* transmitted with collisions */
#define	TXSTAT_ND	0x0002		/* non-deferred transmission */
#define	TXSTAT_PT	0x0001		/* Packet transmitted */

#define	TXSTAT_BITS	"\020"	\
	"\010DFR"	\
	"\007OWC"	\
	"\006CDH"	\
	"\005UN"	\
	"\004CSL"	\
	"\003CO"	\
	"\002ND"	\
	"\001PT"

#define	ETXTHR_MASK	0x07ff

/*
 * Transmit and receive descriptor format
 */
struct epic_desc {
	volatile uint32_t	desc_stat;
	volatile uint32_t	desc_buf;
	volatile uint32_t	desc_ctl;
	volatile uint32_t	desc_next;
};

struct epic_frag {
	volatile uint32_t	frag_ptr;
	volatile uint32_t	frag_len;
};

#define	TD_STAT_LEN	0xffff0000
#define		TD_STAT_LEN_SHIFT	16
#define	TD_STAT_OWN	0x00008000	/* 1:nic owns, 0:host owns */
/* the rest bits are same as TXSTAT */

#define	TD_STAT_BITS	"\020"	\
	"\020OWN"	\
	"\010DFR"	\
	"\007OWC"	\
	"\006CDH"	\
	"\005UN"	\
	"\004CSL"	\
	"\003CO"	\
	"\002ND"	\
	"\001PT"

#define	TD_CTL_LASTDSCR	0x00100000	/* last descriptor of current tx frame*/
#define	TD_CTL_NOCRC	0x00080000	/* disable automatic CRC generation */
#define	TD_CTL_IAF	0x00040000	/* interrupt after transmitted */
#define	TD_CTL_LFFORM	0x00020000	/* 1:len+ptr 0:ptr+len */
#define	TD_CTL_FRAGLIST	0x00010000	/* descriptor points frag list */
#define	TD_CTL_BUFLEN	0x0000ffff	/* data buffer length */

#define	TD_CTL_BITS	"\020"	\
	"\025LAST"	\
	"\024NOCRC"	\
	"\023IAF"	\
	"\022LFFORM"	\
	"\021FRAGLIST"

#define	RD_STAT_LEN	0xffff0000	/* received frame length */
#define		RD_STAT_LEN_SHIFT	16
#define	RD_STAT_OWN	0x00008000	/* 1:nic owns, 0:host owns */
#define	RD_STAT_HC	0x00004000	/* header copied */
#define	RD_STAT_FLE	0x00002000	/* fragment list error */
#define	RD_STAT_VALID	0x00001000	/* network status valid */

/* the rest bits are same as RXSTAT */

#define	RD_STAT_BITS	"\020"	\
	"\020OWN"	\
	"\017HC"	\
	"\016FLE"	\
	"\015VALID"	\
	"\007DIS"	\
	"\006BR"	\
	"\005MULT"	\
	"\004MISSED"	\
	"\003CRC"	\
	"\002FAE"	\
	"\001PRI"

#define	RD_CTL_HEADER	0x00040000	/* for header copy */
#define	RD_CTL_LFFORM	0x00020000	/* 1: len+ptr 0:ptr+len */
#define	RD_CTL_FRAGLIST	0x00010000	/* the descriptor points frag list */
#define	RD_CTL_BUFLEN	0x0000ffff	/* data buffer length */

#define	RD_CTL_BITS	"\020"	\
	"\023HEADER"	\
	"\022LFFORM"	\
	"\021FRAGLIST"
