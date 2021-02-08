/*
 * Etherlink III register definition.
 *
 * This header file is based on 3COM's "EtherLink III LAN PC Cards ERS"
 */

/*
 * Window and register definitions
 */

/* Window 0  Read function*/
#define	W0_MANUFACTUREID	0x0
#define	W0_PRODID		0x2
#define	W0_CONFCNTL		0x4
#define	W0_ADDRCONF		0x6
#define	W0_RSRCCONF		0x8
#define	W0_EEPROMCMD		0xa
#define	W0_EEPROMDATA		0xc
#define	WX_STATUS		0xe
#define	WX_WINDOW		0xf

/* Window 0  write function */
#define	WX_COMMAND		0xe

/* Window 1 */
#define	W1_RX_PIO_DATA		0x0	/* RO */
#define	W1_TX_PIO_DATA		0x0	/* WO */
#define	W1_RX_STATUS		0x8
#define	W1_TIMER		0xa
#define	W1_TX_STATUS		0xb
#define	W1_FREE_XMIT_BYTES	0xc

/* Window 2 */
#define	W2_ADDRESS		0x0
#define	W2_SRAM_DIAGNOSTIC	0x6
#define	W2_MULTIFUNC_CONFIG	0x8
#define	W2_ALT_EEPROM_COMMAND	0xa

/* Window 3 */
#define	W3_TX_RECLAIM_THRESHOLD	0x6
#define	W3_RESOURCE_CONFIG	0x8
#define	W3_FREE_RECEIVE_BYTES	0xa
#define	W3_FREE_XMIT_BYTES	0xc

/* Window 4  Read/write function (manual is funny) */
#define	W4_FIFO_DIAG		0x6
#define	W4_ETHERNET_CNTL_STATUS	0x8
#define	W4_MEDIA_TYPE_STATUS	0xa

/* Window 5 */
#define	W5_TX_STARTS_THRESHOLD	0x0
#define	W5_TX_AVAIL_THRESHOLD	0x2
#define	W5_RX_EARLY_THRESHOLD	0x6
#define	W5_RX_FILTER		0x8
#define	W5_INTERRUPT_MASK	0xa
#define	W5_READ_ZERO_MASK	0xc

/* Window 6 */
#define	W6_CARRIER_SENSE_LOST	0x0	/* B */
#define	W6_SQE			0x1	/* B */
#define	W6_MULTI_COLLISION	0x2	/* B */
#define	W6_ONE_COLLISION	0x3	/* B */
#define	W6_LATE_COLLISION	0x4	/* B */
#define	W6_RX_OVERRUN		0x5	/* B */
#define	W6_TX_OK		0x6	/* B */
#define	W6_RX_OK		0x7	/* B */
#define	W6_TX_DEFER		0x8	/* B */
#define	W6_RX_TOTAL_BYTES	0xa	/* W */
#define	W6_TX_TOTAL_BYTES	0xc	/* W */

/* Registger definitions */

#define	CMD_SHIFT	11
#define	OP_GLOBAL_RESET			(0x00 << CMD_SHIFT)
#define	OP_SELECT_REGISTER_WINDOW	(0x01 << CMD_SHIFT)
#define	OP_START_COAXIAL_TRANSCEIVER	(0x02 << CMD_SHIFT)
#define	OP_RX_DISABLE			(0x03 << CMD_SHIFT)
#define	OP_RX_ENABLE			(0x04 << CMD_SHIFT)
#define	OP_RX_RESET			(0x05 << CMD_SHIFT)
#define	OP_RX_DISCARD_TOP_PKT		(0x08 << CMD_SHIFT)
#define	OP_TX_ENABLE			(0x09 << CMD_SHIFT)
#define	OP_TX_DISABLE			(0x0a << CMD_SHIFT)
#define	OP_TX_RESET			(0x0b << CMD_SHIFT)
#define	OP_REQUEST_INTERRUPT		(0x0c << CMD_SHIFT)
#define	OP_ACK_INTERRUPT		(0x0d << CMD_SHIFT)
#define	OP_SET_INTERRUPT_MASK		(0x0e << CMD_SHIFT)
#define	OP_SET_READ_ZERO_MASK		(0x0f << CMD_SHIFT)
#define	OP_SET_RX_FILTER		(0x10 << CMD_SHIFT)
# define	RX_ALL		0x8
# define	RX_BROADCAST	0x4
# define	RX_MULTICAST	0x2
# define	RX_INDIVIDUAL	0x1

#define	OP_SET_RX_EARLY_THRESHOLD	(0x11 << CMD_SHIFT)
#define	OP_SET_TX_AVAIL_THRESHOLD	(0x12 << CMD_SHIFT)
#define	OP_SET_TX_START_THRESHOLD	(0x13 << CMD_SHIFT)
# define	INVALID_SIZE	0x07fc
#define	OP_STATISTICS_ENABLE		(0x15 << CMD_SHIFT)
#define	OP_STATISTICS_DISABLE		(0x16 << CMD_SHIFT)
#define	OP_STOP_COAXIAL_TRANSCEIVER	(0x17 << CMD_SHIFT)

/*
 * Common register
 */
/* Command register (offset 0xe) */

/* Status register (offset 0xe) */
#define	INTR_IL			0x01
#define	INTR_AF			0x02
#define	INTR_TC			0x04
#define	INTR_TA			0x08
#define	INTR_RC			0x10
#define	INTR_RE			0x20
#define	INTR_IR			0x40
#define	INTR_US			0x80

#define	STATUS_IP		0x1000
#define	STATUS_WIN_NO_SHIFT	13
#define	STATUS_WIN_NO_MASK	0x7

#define	STATUS_BITS	\
	"\020"	\
	"\015CmdBusy"	\
	"\010UpdateStatistics"	\
	"\007InterruptRequested"	\
	"\006RxEarly"	\
	"\005RxComplete"	\
	"\004TxAvailable"	\
	"\003TxComplete"	\
	"\002AdapterFailure"	\
	"\001InterruptLatch"

/*
 * Window 0
 */
/* Configration control (offset 0x4) */
#define	CONFCNTL_RST	0x0004	/* reset pc pard */
#define	CONFCNTL_ENA	0x0001	/* enable pc card */

/* Address configuration (offset 0x6) */
#define	ADDRCONF_XCVR_SHIFT	14
#define	ADDRCONF_XCVR_MASK	0x3
#define	ADDRCONF_XCVR_10BASET	0x0	/* i.e. TP */
#define	ADDRCONF_XCVR_AUI	0x2	/* external transceiver */
#define	ADDRCONF_XCVR_10BASE2	0x3	/* i.e. BNC */

/* Resource configuration reg (offset 0x8) */
#define 	RSRCCONF_3C589		0x3f00
#define 	RSRCCONF_CONST		0x0f00

/* EEPROM command register (offset 0xa) */
#define	EEPROM_EBY	0x8000
#define	EEPROM_TST	0x4000
#define	EEPROM_READ	0x0080
#define	EEPROM_WRITE	0x0040
#define	EEPROM_ERASE	0x00c0
#define	EEPROM_EN	0x0030
#define	EEPROM_DIS	0x0000
#define	EEPROM_ERASEALL	0x0020
#define	EEPROM_WRITEALL	0x0010
#define	EEPROM_ADDR	0x000f

/* EEPROM data structue 	offset */
#define	PROM_ADDR_MAC		0x00
#define	PROM_ADDR_PROD_ID	0x03
#define	PROM_ADDR_DATE		0x04
#define	PROM_ADDR_MANU_ID	0x07
#define	PROM_ADDR_ADDRCONF	0x08
#define	PROM_ADDR_RSRCCONF	0x09
#define	PROM_ADDR_OEM_MAC	0x0a
#define	PROM_ADDR_SOFT_INFO	0x0d
#define		SOFT_INFO_FD	0x8000
#define		SOFT_INFO_LBD	0x4000
#define	PROM_ADDR_COMPAT_WORD	0x0e
#define	PROM_ADDR_CKSUM		0x0f

/*
 * Window 1
 */
/* FIFO register */
#define	FIFO_RX_STATUS_IC		0x8000
#define	FIFO_RX_STATUS_ER		0x4000
#define	FIFO_RX_STATUS_CODE_MASK	0x7
#define	FIFO_RX_STATUS_CODE_SHIFT	11
#define	RX_ERR_CODE_OVERRUN		0
#define	RX_ERR_CODE_RUNT_PKT_ERR	3
#define	RX_ERR_CODE_FRAMING_ERR		4
#define	RX_ERR_CODE_CRC_ERR		5
#define	FIFO_RX_STATUS_BYTES		0x7ff

/* TX packet header */
#define	TX_FIFO_INT	0x8000	/* Interrupt on successful TX completion */
#define	TX_FIFO_DCG	0x2000	/* Disable CRC Generation */
#define	TX_FIFO_LEN	0x07ff	/* packet length in byte */
 
/* RX Status register */
#define	RX_STATUS_IC	0x8000	/* packet incomplete or rx fifoempty */
#define	RX_STATUS_ER	0x4000	/* rx packet incomplete or rx fifoempty */
#define	RX_STATUS_CODE_MASK	0x3800	/* error code */
#define	RX_STATUS_CODE_SHIFT	11
#define	RX_STATUS_CODE_OVERRUN	0x0
#define	RX_STATUS_CODE_OVERSIZE	0x1
#define	RX_STATUS_CODE_DRIBBLE	0x2	/* if ER == 0 */
#define	RX_STATUS_CODE_RUNT	0x3
#define	RX_STATUS_CODE_FRAME	0x4
#define	RX_STATUS_CODE_CRC	0x5
#define	RX_STATUS_BYTES		0x07ff

#define	RX_STATUS_BITS	"\020\020IC\017ER"
	
/* TX Status register */
#define	TX_STATUS_CM	0x80	/* TX is complete */
#define	TX_STATUS_IS	0x40	/* Interrupt on Successful TX completion */
#define	TX_STATUS_JB	0x20	/* Jabber error */
#define	TX_STATUS_UN	0x10	/* Underrun error, TX reset required */
#define	TX_STATUS_MC	0x08	/* Maximum Collitions */
#define	TX_STATUS_OF	0x04	/* TX status overflow */

#define	TX_STATUS_BITS	"\020\010CM\007IS\006JB\005UN\004MC\003CM"

/*
 * Window 4
 */
/* Media type and status register */
#define	MEDIA_TPE		0x8000	/* RO */
#define	MEDIA_CE		0x4000	/* RO */
#define	MEDIA_LK		0x0080	/* RW */
#define	MEDIA_JE		0x0040	/* RW */
#define	MEDIA_LEDD		0x0001	/* LED disable (E version) */

/* Net diag port */
#define	NET_DIAG_EL		0x8000	/* External loop back */
#define	NET_DIAG_ENL		0x4000	/* Endec loop back */
#define	NET_DIAG_ECL		0x2000	/* Ethernet contoller loop back */
#define	NET_DIAG_FL		0x1000	/* FIFO loop back */
#define	NET_DIAG_TXE		0x0800	/* TX enabled */
#define	NET_DIAG_RXE		0x0400	/* RX enabled */
#define	NET_DIAG_TXT		0x0200	/* Transmitting */
#define	NET_DIAG_TXR		0x0100	/* TX reset required */
#define	NET_DIAG_SE		0x0080	/* Statistics enabled */
#define	NET_DIAG_ASIC		0x003e	/* ASIC revision */
#define	NET_DIAG_TLD		0x0001	/* Test low voltage detector */

#define	NET_DIAG_BITS	"\020\020EL\017ENL\016ECL\015FL\014TXE\013RXE" \
			"\012TXT\011TXR\010SE\001TLD"

/* FIFO diagnostic register */
#define	FIFO_DIAG_RXR		0x8000	/* RX receiving */
#define	FIFO_DIAG_RXU		0x2000	/* RX Underrun */
#define	FIFO_DIAG_RSO		0x1000	/* RX Status Overrun */
#define	FIFO_DIAG_RXO		0x0800	/* RX Overrun */
#define	FIFO_DIAG_TXO		0x0400	/* TX Overrun */

#define	FIFO_DIAG_BITS	"\020\020RXR\016RXU\015RSO\014RXO\013TXO"


/*
 * Identify codes
 */
#define MANFID_3COM             0x0101
#define PRODID_3COM_3C562       0x0562
