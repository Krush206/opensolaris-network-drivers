/*
 * dp8390reg.h  register definision of DP8390 core
 * @(#)dp8390reg.h	1.2 04/04/11
 * based on NS DP83905 data sheet
 * This file is public domain.
 */

/*
 * Page 0 register offsets
 */
#define	PX_CR		0x00	/* Command */
#define	P0_PSTART_W	0x01	/* Page Start Register (W) */
#define	P0_PSTOP_W	0x02	/* Page Stop Register (W) */
#define	P0_BNRY		0x03	/* Boundary Pointer (R/W) */
#define	P0_TPSR_W	0x04	/* Transmit Page Start Address (W) */
#define	P0_TBCR0_W	0x05	/* Transmit Byte Count Register LSB (W) */
#define	P0_TBCR1_W	0x06	/* Transmit Byte Count Register MSB (W) */
#define	P0_ISR		0x07	/* Interrupt Status Pointer (R/W) */
#define	P0_RSAR0_W	0x08	/* Remote Start Address Register LSB  (W) */
#define	P0_RSAR1_W	0x09	/* Remote Start Address Register MSB (W) */
#define	P0_RBCR0_W	0x0a	/* Remote Byte Count Register LSB (W) */
#define	P0_RBCR1_W	0x0b	/* Remote Byte Count Register MSB (W) */
#define	P0_RCR_W	0x0c	/* Receive Configuration Register (W) */
#define	P0_TCR_W	0x0d	/* Transmit Configuration Register (W) */
#define	P0_DCR_W	0x0e	/* Data Configuration Register (W) */
#define	P0_IMR_W	0x0f	/* Interrupt Mask Register (W) */

#define	P0_CLDA0_R	0x01	/* Current Local DMA Address0 (R) */
#define	P0_CLDA1_R	0x02	/* Current Local DMA Address1 (R) */
#define	P0_TSR_R	0x04	/* Transmit Status Register (R) */
#define	P0_NCR_R	0x05	/* Number of Collisions (R) */
#define	P0_FIFO_R	0x06	/* Number of Collisions (R) */
#define	P0_CRDA0_R	0x08	/* Current Local DMA Address0 (R) */
#define	P0_CRDA1_R	0x09	/* Current Local DMA Address1 (R) */
#define	P0_RSR_R	0x0c	/* Receive Status Register (R) */
#define	P0_CNTR0_R	0x0d	/* Tally Counter0 (Frame alignment err) (R) */
#define	P0_CNTR1_R	0x0e	/* Tally Counter1 (CRC errors) (R) */
#define	P0_CNTR2_R	0x0f	/* Tally Counter1 (Missed pkt errors) (R) */

/*
 * Page 1 register offsets
 */
#define	P1_PAR		0x01	/* Physical address register (R/W) */
#define	P1_CURR		0x07	/* Current Page register (R/W) */
#define	P1_MAR		0x08	/* Multicast address register (R/W) */

/*
 * Page 2 register offsets
 */
#define	P2_CLDA0_W	0x01	/* Current Local DMA address (W) */
#define	P2_CLDA1_W	0x02	/* Current Local DMA Address1 (W) */

#define	P2_PSTART_R	0x01	/* Page Start Register (R) */
#define	P2_PSTOP_R	0x02	/* Page Stop Register (R) */
#define	P2_TPSR_R	0x04	/* Transmit Page start (R) */
#define	P2_RCR_R	0x0c	/* Receive Configuration Register (R) */
#define	P2_TCR_R	0x0d	/* Transmit Configuration Register (R) */
#define	P2_DCR_R	0x0e	/* Data Configuration Register (R) */
#define	P2_IMR_R	0x0f	/* Interrupt Mask Register (R) */

/*
 * Register bit fields
 */
/* CR: Command register */
#define	CR_PS		0xc0	/* Page select */
#define	CR_PS_SHIFT	6	/* Page select shift bits */
#define		CR_PS_0	(0 << CR_PS_SHIFT)
#define		CR_PS_1	(1 << CR_PS_SHIFT)
#define		CR_PS_2	(2 << CR_PS_SHIFT)
#define		CR_PS_3	(3 << CR_PS_SHIFT)
#define	CR_RD		0x38	/* Remote DMA command */
#define	CR_RD_SHIFT	3
#define	CR_RD_RRD	(1 << CR_RD_SHIFT)	/* Remote Read */
#define	CR_RD_RWR	(2 << CR_RD_SHIFT)	/* Remote write */
#define	CR_RD_ABORT	(4 << CR_RD_SHIFT)	/* Abort/Complete dma */
#define	CR_TXP		0x04	/* Transmit packet */
#define	CR_STA		0x02	/* Start NIC core */
#define	CR_STP		0x01	/* Stop NIC core */

/* ISR: Interrupt Status register/IMR: Interrupt Mask register */
#define	ISR_RST		0x80	/* reset status */
#define	ISR_RDC		0x40	/* Remote DMA complete */
#define	ISR_CNT		0x20	/* Counter overflow */
#define	ISR_OVW		0x10	/* Overwrite warning (not common)*/
#define	ISR_TXE		0x08	/* Transmit error (ex-collision/FIFOunderrun)*/
#define	ISR_RXE		0x04	/* Receive error (CRC/FAE/FIFOoverrun/Missed)*/
#define	ISR_PTX		0x02	/* Packet transmitted */
#define	ISR_PRX		0x01	/* Packet received */

#define	ISR_BITS	\
	"\020\010Rst\007RDC\006Cnt\005OVW\004TXE\003RXE\002PTX\001PRX"

/* DCR: Data configuration register */
#define	DCR_FT		0x60	/* FIFO threshold select */
#define	DCR_FT_SHIFT	5
#define	DCR_FT_2	(0 << DCR_FT_SHIFT)
#define	DCR_FT_4	(1 << DCR_FT_SHIFT)
#define	DCR_FT_8	(2 << DCR_FT_SHIFT)
#define	DCR_FT_12	(3 << DCR_FT_SHIFT)
#define	DCR_ARM		0x10	/* Auto-Initialize remote */
#define	DCR_LS_NORMAL	0x08	/* Loopback select (0:loopback, 1:normal) */
#define	DCR_LAS		0x04	/* Long Address select */
#define	DCR_BOS		0x02	/* Byte oder select */
#define	DCR_WTS		0x01	/* Word transfer select */

/* TCR: Transmit configuration register */
#define	TCR_OFST	0x10	/* Collision offset enable 0:normal */
#define	TCR_ATD		0x08	/* Auto transmit disable */
#define	TCR_LB		0x06	/* Encoded loopback control */
#define	TCR_LB_SHIFT	1
#define	TCR_LB_NORMAL	(0 << TCR_LB_SHIFT)
#define	TCR_LB_NIC	(1 << TCR_LB_SHIFT)
#define	TCR_LB_ENDEC	(2 << TCR_LB_SHIFT)
#define	TCR_LB_EXT	(3 << TCR_LB_SHIFT)
#define	TCR_CRC		0x01	/* Inhibit CRC */

/* TSR: Transmit Status register */
#define	TSR_OWC		0x80	/* Out of Windodw collision */
#define	TSR_CDH		0x40	/* CD heartbeat */
#define	TSR_FU		0x20	/* FIFO underrun */
#define	TSR_CRS		0x10	/* Carrier Sense Lost */
#define	TSR_ABT		0x08	/* Transmit Aborted */
#define	TSR_COL		0x04	/* Transmit collided */
#define	TSR_ND		0x02	/* no defered */
#define	TSR_PTX		0x01	/* Packet transmitted */

#define	TSR_BITS	\
	"\020"	\
	"\010OWC"	\
	"\007CDH"	\
	"\006FU"	\
	"\005CRS"	\
	"\004ABT"	\
	"\003COL"	\
	"\001PTX"

/* RCR: Receive Configuration register */
#define	RCR_MON		0x20	/* Monitor mode (legacy) */
#define	RCR_PRO		0x10	/* Promiscuous physical */
#define	RCR_AM		0x08	/* Accept Multicast */
#define	RCR_AB		0x04	/* Accept Braodcast */
#define	RCR_AR		0x02	/* Accept Runt packets */
#define	RCR_SEP		0x01	/* Save Errored packets */

/* RSR: Receive Status register */
#define	RSR_DFR		0x80	/* Deferring */
#define	RSR_DIS		0x40	/* Receiver disabled */
#define	RSR_PHY		0x20	/* Physical/Multicast address (0:phys 1:mult)*/
#define	RSR_MPA		0x10	/* Missed packet */
#define	RSR_FO		0x08	/* FIFO Overrun */
#define	RSR_FAE		0x04	/* Frame alignment error */
#define	RSR_CRC		0x02	/* CRC error */
#define	RSR_PRX		0x01	/* Packet received intact */

#define	RSR_BITS	\
	"\020"	\
	"\010DFR"	\
	"\007DIS"	\
	"\006PHY"	\
	"\005MPA"	\
	"\004FO"	\
	"\003FAE"	\
	"\002CRC"	\
	"\001PRX"

#define	DP8390_PAGE_SIZE	(256)
#define	DP8390_PAGE_SHIFT	(8)
#define	DP8390_PTOB(x)		((x) << DP8390_PAGE_SHIFT)
#define	DP8390_BTOP(x)		((x) >> DP8390_PAGE_SHIFT)

/* receive packet header */
struct rhead {
	uint8_t		rh_rsr;		/* receive status */
	uint8_t		rh_next;	/* next page */
	uint16_t	rh_size;	/* packet size in byte */
};
