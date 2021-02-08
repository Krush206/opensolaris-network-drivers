/*
 * DP8390 hardware management structure dends of Solaris and OS interface
 * @(#)dp8390var.h	1.8 05/05/09
 *  Copright (C) 2001-2004  Masayuki Murayama (KHF04453@nifty.ne.jp)
 *
 *  CAUTION: NO WARRANTY
 *  This software may be used and distributed according to the terms
 *  of the BSD License.
 */

#ifndef _dp8390_var_h
#define _dp8390_var_h

struct mcast_addr {
	struct ether_addr	addr;
	uint32_t		hash;
};

struct dp8390_stats {
	uint32_t	intr;

	uint32_t	crc;		/* # receive crc errors */
	uint32_t	errrcv;
	uint32_t	overflow;	/* # receiver overflows */
	uint32_t	frame;		/* # receive framming errors */
	uint32_t	missed;		/* # receive missed */
	uint32_t	runt;		/* # ilegal input packet length */
	uint32_t	frame_too_long;	/* # ilegal input packet length */
	uint32_t	norcvbuf;	/* # esballoc/allocb failed */

	uint32_t	collisions;	/* # collisions */
	uint32_t	first_coll;	/* # collisions */
	uint32_t	multi_coll;	/* # collisions */
	uint32_t	excoll;
	uint32_t	nocarrier;	/* # loss of carrier errors */
	uint32_t	defer;		/* # defers */
	uint32_t	errxmt;		/* # total output errors */
	uint32_t	underflow;	/* # transmit underflows */
	uint32_t	xmtlatecoll;	/* # transmit late collisions */
};

/*
 * Transmit buffer configration
 */
#define TX_BUF_SIZE \
	((ETHERMAX + ETHERFCSL + DP8390_PAGE_SIZE - 1)/(DP8390_PAGE_SIZE))
#define	NTXBUF_DEFAULT	1
#define	NTXBUF_MAX	16

struct txbuf {
	struct txbuf	*next;
	uint8_t		tx_start;	/* First page of this tx buf */
	uint8_t		pad;
	uint16_t	tx_len;		/* Packet length to transmit */
};

struct dp8390_dev {

	dev_info_t		*dip;		/* associated dev_info */
	char			name[32];
	long			base_addr;	/* assigned I/O port base */
	ddi_acc_handle_t	handle;		/* handle to map regs */
	ddi_iblock_cookie_t	cookie;		/* cookie from ddi_add_intr */

	/* Eternet addess management */
	struct	ether_addr	curraddr;	/* current address */
	struct	ether_addr	factaddr;	/* factory address */

	/* local memory configuration for TX and RX buffer */
	u_int			ram_start;	/* start address in page */
	u_int			ram_size;	/* size in page */
	int			ntxbuf;		/* # of tx buffer*/
	uint8_t			rx_start;	/* top page of rx buffer */
	uint8_t			rx_stop;	/* stop page of rx buffer */

	/* Rx buffer management */
	kmutex_t		intrlock;	/* protect intr-side fields */
	boolean_t		intr_busy;	/* interrupt is active */
	uint8_t			rx_head;	/* head of FIFO buffer */

	/* Tx buffer management */
	kmutex_t		xmitlock;	/* protect xmit-side fields */
	kcondvar_t		drain_cv;	/* wait for Tx draining */
	struct txbuf 		*tx_head;	/* head of Tx ready queue */
	int			tx_ready_len;	/* length of Tx ready queue */
	struct txbuf 		*tx_free;	/* free list of txbuf */
	struct txbuf 		tx[NTXBUF_MAX];
	clock_t			tx_start_time;	/* last tx time */
	int			tx_busy;	/* num of active sender */
	boolean_t		tx_blocked;
	volatile timeout_id_t	tx_timeout_id;

	/* Remote DMA management */
	ksema_t			rdma_sema;	/* semaphone for rmda */

	/* Media status management */
	boolean_t		speed100; 	/* speed 0:10M 1:100M */

	/* Rx filter and multicast support */
	int			mc_count_req;
	int			mc_count;
	struct mcast_addr	*mc_list;
	u_int			rxmode;	/* receive filter mode  */
#define		RXMODE_PROMISC		0x01
#define		RXMODE_ALLMULTI_REQ	0x02
#define		RXMODE_MULTI_OVF	0x04
#define		RXMODE_ALLMULTI		(RXMODE_ALLMULTI_REQ | RXMODE_MULTI_OVF)
	u_int			rcr;	/* soft copy of hsw rcr */


	/* Driver state */
	boolean_t		started;	/* misc. flags */

	/* Misc HW information */
	u_int			xfer_width;	/* data xfer width capability */
#define			XFER_32BIT	0x4
#define			XFER_16BIT	0x2
#define			XFER_8BIT	0x1

	u_int			nic_state;	/* 8390 core state */
#define	NIC_INACTIVE		0		/* not working */
#define	NIC_ACTIVE		1		/* TX/RX are active */
#define	NIC_RX_OVERRUN		2		/* in processing rx overrun */

	kmutex_t		pagelock;	/* protect registger page  */
	uint8_t			cr_rd;		/* rd field of command reg */
	uint8_t			cr_ps;		/* ps field of command reg */
	uint8_t			imr;		/* logical copy of IMR reg */
	uint8_t			imr_shadow;	/* HW copy of IMR reg */
	uint8_t			dcr_wts; 	/* wts bit in DCR */
	uint8_t			pad[3];
	void			*private;	/* Private field */

	/* interface to OS and other module */
	int	(*get_header)(struct dp8390_dev *, struct rhead *, int);
	int	(*put_pkt)(struct dp8390_dev *, mblk_t *, int);
	mblk_t	*(*get_pkt)(struct dp8390_dev *, int, int);
	void	(*sendup)(struct dp8390_dev *, mblk_t *);
	void	(*rx_overrun_continue)(struct dp8390_dev *);

	struct dp8390_stats	stat;

	/* temporary transmit buffer */
	uint32_t		txbuf[(ETHERMAX/sizeof(uint32_t))+1];
};

/*
 * RDMA lock operations
 */
#define RDMA_LOCK(dp)   sema_p(&(dp)->rdma_sema)
#define RDMA_UNLOCK(dp) sema_v(&(dp)->rdma_sema)

/*
 * return value of interrupt
 */
#define	INTR_FATAL_ERR	0x0200
#define	INTR_RESTART_TX	0x0100
#define	INTR_ISR_MASK	0x00ff

/*
 * Exported procedures
 */
extern int dp8390_attach(struct dp8390_dev *);
extern int dp8390_detach(struct dp8390_dev *);
extern void dp8390_init_chip(struct dp8390_dev *);
extern void dp8390_set_mac_addr(struct dp8390_dev *);
extern void dp8390_start_chip(struct dp8390_dev *);
extern void dp8390_stop_chip(struct dp8390_dev *);
extern int dp8390_interrupt(struct dp8390_dev *);
extern int dp8390_tx_start(struct dp8390_dev *, mblk_t *mp);
extern void dp8390_get_stats(struct dp8390_dev *);
extern uint32_t dp8390_ether_crc(uint8_t *);
extern void dp8390_set_rx_filter(struct dp8390_dev *);

extern void dp8390_rdma_setup(struct dp8390_dev *, int, u_int, u_int);
extern int  dp8390_rdma_finish(struct dp8390_dev *);
extern void dp8390_rdma_abort(struct dp8390_dev *);

extern int dp8390_rdma_get_header(struct dp8390_dev *, struct rhead *, int);
extern mblk_t * dp8390_rdma_get_pkt(struct dp8390_dev *, int, int);
extern int dp8390_rdma_put_pkt(struct dp8390_dev *, mblk_t *, int);

extern void dp8390_rx_overrun_continue(struct dp8390_dev *dp);

/* don't use ddi_rep_put16 function */
#ifdef i386
#define	INB(dp, p)	\
		inb(((dp)->base_addr) + (p))

#define	INW(dp, p)	\
		inw(((dp)->base_addr) + (p))

#define	INL(dp, p)	\
		inl(((dp)->base_addr) + (p))

#define	OUTB(dp, p, x)	\
		outb(((dp)->base_addr) + (p), (uint8_t)(x))

#define	OUTW(dp, p, x)	\
		outw(((dp)->base_addr) + (p), (uint16_t)(x))

#define	OUTL(dp, p, x)	\
		outl(((dp)->base_addr) + (p), (uint32_t)(x))

#define	INSB(dp, p, a, c)	\
		repinsb(((dp)->base_addr) + (p), (void *)(a), c)

#define	INSW(dp, p, a, c)	\
		repinsw(((dp)->base_addr) + (p), (void *)(a), c)

#define	INSL(dp, p, a, c)	\
		repinsd(((dp)->base_addr) + (p), (void *)(a), c)

#define	OUTSB(dp, p, a, c)	\
		repoutsb(((dp)->base_addr) + (p), (void *)(a), c)

#define	OUTSW(dp, p, a, c)	\
		repoutsw(((dp)->base_addr) + (p), (void *)(a), c)

#define	OUTSL(dp, p, a, c)	\
		repoutsd(((dp)->base_addr) + (p), (void *)(a), c)

#else

#define	INB(dp, p)	\
		ddi_get8((dp)->handle, (uint8_t *)((dp)->base_addr + (p)))
#define	INW(dp, p)	\
		ddi_get16((dp)->handle, (uint16_t *)((dp)->base_addr + (p)))
#define	INL(dp, p)	\
		ddi_get32((dp)->handle, (uint32_t *)((dp)->base_addr + (p)))
#define	OUTB(dp, p, x)	\
		ddi_put8((dp)->handle, (uint8_t *)((dp)->base_addr + (p)), (uint8_t)(x))
#define	OUTW(dp, p, x)	\
		ddi_put16((dp)->handle,(uint16_t *)((dp)->base_addr + (p)), (uint16_t)(x))
#define	OUTL(dp, p, x)	\
		ddi_put32((dp)->handle,(uint32_t *)((dp)->base_addr + (p)), (uint32_t)(x))

#define	INSB(dp, p, a, c)	\
		ddi_rep_get8((dp)->handle, (uint8_t *)(a), (uint8_t *)((dp)->base_addr + (p)), c, DDI_DEV_NO_AUTOINCR)

#define	INSW(dp, p, a, c)	\
		ddi_rep_get16((dp)->handle, (uint16_t *)(a), (uint16_t *)(((dp)->base_addr) + (p)), c, DDI_DEV_NO_AUTOINCR)

#define	INSL(dp, p, a, c)	\
		ddi_rep_get32((dp)->handle, (uint32_t *)(a), (uint32_t *)(((dp)->base_addr) + (p)), c, DDI_DEV_NO_AUTOINCR)

#define	OUTSB(dp, p, a, c)	\
		ddi_rep_put8((dp)->handle, (uint8_t *)(a), (uint8_t *)(((dp)->base_addr) + (p)), c, DDI_DEV_NO_AUTOINCR)

#define	OUTSW(dp, p, a, c)	\
		ddi_rep_put16((dp)->handle, (uint16_t *)(a), (uint16_t *)(((dp)->base_addr)) + (p), c, DDI_DEV_NO_AUTOINCR)

#define	OUTSL(dp, p, a, c)	\
		ddi_rep_put32((dp)->handle, (uint32_t *)(a), (uint32_t *)(((dp)->base_addr) + (p)), c, DDI_DEV_NO_AUTOINCR)

#endif

#endif /* _dp8390var_h */
