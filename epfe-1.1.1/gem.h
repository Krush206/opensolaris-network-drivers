/*
 * gem.h: General Ethernet MAC driver framework
 * (C) Copyright 2002-2006 Masayuki Murayama KHF04453@nifty.ne.jp
 */

#ifndef _GEM_H_
#define _GEM_H_

#pragma	ident	"%W% %E%"

#include <sys/gld.h>

/*
 * Useful macros and typedefs
 */
#define	GEM_NAME_LEN	32

#define GEM_TX_TIMEOUT		(drv_usectohz(5*1000000))
#define GEM_TX_TIMEOUT_INTERVAL	(drv_usectohz(1*1000000))
#define GEM_LINK_WATCH_INTERVAL	(drv_usectohz(1*1000000))	/* 1 sec */

/* general return code */
#define	GEM_SUCCESS	0
#define	GEM_FAILURE	(-1)

/* return code of gem_tx_done */
#define	INTR_RESTART_TX	0x80000000

/*
 * I/O instructions
 */
#define	OUTB(dp, p, v)	\
	ddi_put8((dp)->regs_handle, \
		(uint8_t *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	OUTW(dp, p, v)	\
	ddi_put16((dp)->regs_handle, \
		(uint16_t *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	OUTL(dp, p, v)	\
	ddi_put32((dp)->regs_handle, \
		(uint32_t *)((caddr_t)((dp)->base_addr) + (p)), v)
#define	INB(dp, p)	\
	ddi_get8((dp)->regs_handle, \
		(uint8_t *)(((caddr_t)(dp)->base_addr) + (p)))
#define	INW(dp, p)	\
	ddi_get16((dp)->regs_handle, \
		(uint16_t *)(((caddr_t)(dp)->base_addr) + (p)))
#define	INL(dp, p)	\
	ddi_get32((dp)->regs_handle, \
		(uint32_t *)(((caddr_t)(dp)->base_addr) + (p)))

struct gem_stats {
	uint32_t	intr;

	uint32_t	crc;
	uint32_t	errrcv;
	uint32_t	overflow;
	uint32_t	frame;
	uint32_t	missed;
	uint32_t	runt;
	uint32_t	frame_too_long;
	uint32_t	norcvbuf;

	uint32_t	collisions;
	uint32_t	first_coll;
	uint32_t	multi_coll;
	uint32_t	excoll;
	uint32_t	nocarrier;
	uint32_t	defer;
	uint32_t	errxmt;
	uint32_t	underflow;
	uint32_t	xmtlatecoll;
};

#define	GEM_MAXTXSEGS		4
#define	GEM_MAXRXSEGS		1

#ifdef IOMMU
#define	GEM_MAXTXFRAGS		GEM_MAXTXSEGS
#define	GEM_MAXRXFRAGS		GEM_MAXRXSEGS
#else
#define	GEM_MAXTXFRAGS		(GEM_MAXTXSEGS*2)
#define	GEM_MAXRXFRAGS		(GEM_MAXRXSEGS*2)
#endif

/* TX buffer management */
struct txbuf {
	struct txbuf		*txb_next;

	/* direct buffer mapping management */
	ddi_dma_handle_t	txb_dh[GEM_MAXTXSEGS];
	uint_t			txb_dh_used;
	mblk_t			*txb_mp;
	ddi_dma_cookie_t	txb_dmacookie[GEM_MAXTXFRAGS];
	uint_t			txb_nfrags;

	/* bounce buffer management */
	ddi_dma_handle_t	txb_bdh;
	ddi_acc_handle_t	txb_bah;
	caddr_t			txb_buf;	/* vaddr of bounce buffer */
	uint64_t		txb_buf_dma;	/* paddr of bounce buffer */

	/* timeout management */
	clock_t			txb_stime;

	/* Hardware descriptor info */
	uint_t			txb_slot;
	uint_t			txb_ndesc;
	uint32_t		txb_flag;
};


/* RX buffer management */
struct rxbuf {
	/* Hardware independant section */
	struct rxbuf		*rxb_next;
	struct gem_dev		*rxb_devp;
	uint_t			rxb_flags;
#define	RXB_FLAGS_BOUNCE	0x1

	/* direct buffer mapping management */
	mblk_t 			*rxb_mp;
	ddi_dma_handle_t	rxb_dh;
	caddr_t			rxb_buf;	
	size_t			rxb_buf_len;	
	ddi_dma_cookie_t	rxb_dmacookie[GEM_MAXRXFRAGS];
	uint_t			rxb_nfrags;

	/* bounce buffer management */
	ddi_acc_handle_t	rxb_bah;
	ddi_dma_handle_t	rxb_bdh;
	caddr_t			rxb_bbuf;	
	size_t			rxb_bbuf_len;	

	/* Hardware descriptor info */
	uint_t			rxb_slot;
	uint_t			rxb_ndesc;
};

struct mcast_addr {
	struct ether_addr	addr;
	uint32_t		hash;
};

#define	GEM_MAXMC		64
#define	GEM_MCALLOC		(sizeof(struct mcast_addr) * GEM_MAXMC)

typedef	int32_t			seqnum_t;
#define	SUB(x, y)		((seqnum_t)((x) - (y)))
#define	SLOT(seqnum, size)	(((unsigned int)(seqnum)) & ((size)-1))

/*
 * mac soft state
 */
struct gem_dev {
	dev_info_t		*dip;
	gld_mac_info_t		*macinfo;
	char			name[GEM_NAME_LEN];
	void			*base_addr;
	ddi_acc_handle_t	regs_handle;
	ddi_iblock_cookie_t	iblock_cookie;

	/* MAC address information */
	struct ether_addr	cur_addr;
	struct ether_addr	dev_addr;

	/* Descriptor rings, buffers */
	ddi_dma_handle_t	desc_dma_handle;
	ddi_acc_handle_t	desc_acc_handle;
	caddr_t			rx_ring;
	caddr_t			tx_ring;
	caddr_t			tx_pad;
	caddr_t			rx_buf;

	uint64_t		rx_ring_dma;	/* dma address in PCI bus */
	uint64_t		tx_ring_dma;	/* dma address in PCI bus */
	uint64_t		tx_pad_dma;
	uint64_t		rx_buf_dma;

	/* RX descriptor ring management */
	kmutex_t		intrlock;
	boolean_t		intr_busy;
	seqnum_t		rx_desc_head;
	seqnum_t		rx_desc_tail;

	/* Rx buffer management */
	struct rxbuf		*rx_buf_head;
	struct rxbuf		*rx_buf_tail;
	struct rxbuf		*rx_buf_freelist;
	int			rx_buf_allocated;
	int			rx_buf_freecnt;
	int			rx_buf_len;

	/* TX descriptor ring management */
	kmutex_t		xmitlock;
	kcondvar_t		drain_cv;
	seqnum_t		tx_desc_head;
	seqnum_t		tx_desc_tail;
	seqnum_t		tx_desc_intr;
	int			tx_desc_rsvd;

	/* TX buffer resource management */
	struct txbuf		*tx_buf;
	struct txbuf		*tx_buf_head;
	struct txbuf		*tx_buf_tail;
	struct txbuf		*tx_softq_head;
	struct txbuf		*tx_buf_freelist;
	int			tx_buf_freecnt;
	clock_t			tx_start_time;

	/* TX state management */
	int			tx_busy;
	boolean_t		tx_blocked;

	/* NIC state */
	boolean_t		nic_active;	/* tx and rx are running */
	boolean_t		nic_online;	/* logically opened */

	/* robustness: timer and watchdog */
	timeout_id_t		timeout_id;


	/* MII management */
	boolean_t		anadv_1000fdx;
	boolean_t		anadv_1000hdx;
	boolean_t		anadv_100t4;
	boolean_t		anadv_100fdx;
	boolean_t		anadv_100hdx;
	boolean_t		anadv_10fdx;
	boolean_t		anadv_10hdx;
	boolean_t		full_duplex;
	int			speed;
#define		GEM_SPD_10	0
#define		GEM_SPD_100	1
#define		GEM_SPD_1000	2
	int			flow_control;
#define		FLOW_CONTROL_NONE	0
#define		FLOW_CONTROL_SYMMETRIC	1
#define		FLOW_CONTROL_TX_PAUSE	2
#define		FLOW_CONTROL_RX_PAUSE	3

	boolean_t		no_preamble;
	uint32_t		mii_phy_id;
	int			mii_phy_addr;
	int			mii_state;
#define		MII_STATE_UNKNOWN		0
#define		MII_STATE_RESETTING		1
#define		MII_STATE_AUTONEGOTIATING	2
#define		MII_STATE_AN_DONE		3
#define		MII_STATE_MEDIA_SETUP		4
#define		MII_STATE_LINKUP		5
#define		MII_STATE_LINKDOWN		6

	clock_t			mii_last_check;	/* in tick */
	clock_t			mii_timer;	/* in tick */
#define		MII_RESET_TIMEOUT	drv_usectohz(1000*1000)
#define		MII_AN_TIMEOUT		drv_usectohz(5000*1000)
#define		MII_LINKDOWN_TIMEOUT	drv_usectohz(10000*1000)
	clock_t			mii_interval;	/* in tick */

	boolean_t		mii_fixedmode;
	timeout_id_t		link_watcher_id;
	boolean_t		mii_supress_msg;

	/* multcast list management */
	int16_t			mc_count;
	int16_t			mc_count_req;
	struct mcast_addr	*mc_list;
	uint32_t		rxmode;
#define		RXMODE_PROMISC		0x01
#define		RXMODE_ALLMULTI_REQ	0x02
#define		RXMODE_MULTI_OVF	0x04
#define		RXMODE_ALLMULTI		(RXMODE_ALLMULTI_REQ | RXMODE_MULTI_OVF)

	/* statistcs */
	struct gem_stats		stats;

	/* pointer to local structure */
	void			*private;
	int			priv_size;

	/* polling mode */
	clock_t	last_intr_time;	/**/
	int	rx_intr_cnt;	/* rx intr counter */
	int	rx_pkt_cnt;	/* rx pkt counter */
	int	tx_pkt_cnt;	/* tx pkt counter */
	int	poll_intr_cnt;	/* serviced interrupt counter*/

	int	poll_pkt_delay;	/* in number of packets */
	int	poll_pkt_hiwat;	/* max pkt count */

	int	poll_interval;	/* optimized interval in uS */

	/* configuration */
	struct gem_conf {
		/* name */
		char	gc_name[GEM_NAME_LEN];

		/* specification on tx and rx dma engine */
		long	gc_tx_buf_align;
		int	gc_tx_max_frags;
		int	gc_tx_desc_size;
		int	gc_tx_ring_size;
		int	gc_tx_buf_size;
		int	gc_tx_max_descs_per_pkt;
		int	gc_tx_copy_thresh;
		boolean_t gc_tx_auto_pad;

		long	gc_rx_buf_align;
		int	gc_rx_max_frags;
		int	gc_rx_desc_size;
		int	gc_rx_ring_size;
		int	gc_rx_buf_size;
		int	gc_rx_max_descs_per_pkt;
		int	gc_rx_copy_thresh;
		int	gc_rx_buf_max;
		int	gc_rx_header_len;

		/* memory mapping attributes */
		struct ddi_device_acc_attr	gc_dev_attr;
		struct ddi_device_acc_attr	gc_buf_attr;
		struct ddi_device_acc_attr	gc_desc_attr;

		/* dma attributes */
		ddi_dma_attr_t		gc_dma_attr_desc;
		ddi_dma_attr_t		gc_dma_attr_txbuf;
		ddi_dma_attr_t		gc_dma_attr_rxbuf;

		/* tx time out parameters */
		clock_t	gc_tx_timeout;
		clock_t	gc_tx_timeout_interval;

		/* auto negotiation capability */
		int		gc_flow_control;

		/* MII mode */
		int	gc_mii_mode;
#define		GEM_MODE_100BASETX	0
#define		GEM_MODE_1000BASET	1
#define		GEM_MODE_1000BASETX	2

		/* MII link state watch parameters */
		clock_t	gc_mii_linkdown_timeout;
		clock_t	gc_mii_link_watch_interval;
		clock_t	gc_mii_reset_timeout;

		clock_t	gc_mii_an_watch_interval;
		clock_t	gc_mii_an_timeout;
		clock_t	gc_mii_an_wait;	
		clock_t	gc_mii_an_delay;

		/* MII configuration */
		int	gc_mii_addr_min;	
		int	gc_mii_linkdown_action;	
		int	gc_mii_linkdown_timeout_action;	
#define		MII_ACTION_NONE		0
#define		MII_ACTION_RESET	1
#define		MII_ACTION_RSA		2
		boolean_t	gc_mii_dont_reset;	
		boolean_t	gc_mii_an_oneshot;

		/* I/O methods */

		/* mac operation */
		int	(*gc_attach_chip)(struct gem_dev *dp);
		int	(*gc_reset_chip)(struct gem_dev *dp);
		void	(*gc_init_chip)(struct gem_dev *dp);
		void	(*gc_start_chip)(struct gem_dev *dp);
		int	(*gc_stop_chip)(struct gem_dev *dp);
		uint32_t (*gc_multicast_hash)(struct gem_dev *dp, uint8_t *);
		void	(*gc_set_rx_filter)(struct gem_dev *dp);
		void	(*gc_set_media)(struct gem_dev *dp);
		void	(*gc_get_stats)(struct gem_dev *dp);
		u_int	(*gc_interrupt)(struct gem_dev *dp);

		/* descriptor operation */
		int	(*gc_tx_desc_write)(struct gem_dev *dp, uint_t slot,
			    ddi_dma_cookie_t *dmacookie, int frags, uint32_t flag);
#define			GEM_TXFLAG_INTR		0x00000001
#define			GEM_TXFLAG_IPv4		0x00000002
#define			GEM_TXFLAG_IPv6		0x00000004
#define			GEM_TXFLAG_TCP		0x00000008
#define			GEM_TXFLAG_UDP		0x00000010
#define			GEM_TXFLAG_PRIVATE	0x0000ff00
#define				GEM_TXFLAG_PRIVATE_SHIFT	8
#define				GEM_TXFLAG_PRIVATE_MASK	0xff
#define			GEM_TXFLAG_VID		0x0fff0000
#define				GEM_TXFLAG_VID_SHIFT		16
#define				GEM_TXFLAG_VID_MASK		0xfff
#define			GEM_TXFLAG_CFI		0x10000000
#define			GEM_TXFLAG_PRI		0xe0000000
#define				GEM_TXFLAG_PRI_SHIFT		29
#define				GEM_TXFLAG_PRI_MASK		0x7

		int	(*gc_rx_desc_write)(struct gem_dev *dp, uint_t slot,
			    ddi_dma_cookie_t *dmacookie, int frags);

		uint_t	(*gc_tx_desc_stat)
				(struct gem_dev *dp, int slot, int ndesc);
#define			GEM_TX_DONE	0x00010000
#define			GEM_TX_ERR	0x00020000

		uint64_t (*gc_rx_desc_stat)
				(struct gem_dev *dp, int slot, int ndesc);
#define			GEM_RX_CKSUM_UDP	0x0000000800000000ull
#define			GEM_RX_CKSUM_TCP	0x0000000400000000ull
#define			GEM_RX_CKSUM_IPv6	0x0000000200000000ull
#define			GEM_RX_CKSUM_IPv4	0x0000000100000000ull
#define			GEM_RX_PRI	0xe0000000
#define			GEM_RX_PRI_SHIFT	29
#define			GEM_RX_CFI	0x10000000
#define			GEM_RX_VTAG	0x0fff0000
#define			GEM_RX_VTAG_SHIFT	16
#define			GEM_RX_ERR	0x00008000
#define			GEM_RX_DONE	0x00004000
#define			GEM_RX_LEN	0x00003fff	/* 16KB - 1 */

		void	(*gc_tx_desc_init)(struct gem_dev *dp, int slot);
		void	(*gc_rx_desc_init)(struct gem_dev *dp, int slot);
		void	(*gc_tx_desc_clean)(struct gem_dev *dp, int slot);
		void	(*gc_rx_desc_clean)(struct gem_dev *dp, int slot);

		/* mii operations */
		int	(*gc_mii_init)(struct gem_dev *dp);
		int	(*gc_mii_config)(struct gem_dev *dp);
		void	(*gc_mii_sync)(struct gem_dev *dp);
		uint16_t (*gc_mii_read)(struct gem_dev *dp, uint_t reg);
		void	(*gc_mii_write)(struct gem_dev *dp, uint_t reg, uint16_t val);
		void	(*gc_mii_tune_phy)(struct gem_dev *dp);

		/* packet in/out operation for copy-style  */
		void	(*gc_put_packet)(struct gem_dev *dp, mblk_t *, void *, size_t);
		mblk_t	*(*gc_get_packet)(struct gem_dev *dp, struct rxbuf *, size_t);
	} gc;

	uint32_t	misc_flag;
#define		GEM_VLAN_HARD	0x00000040
#define		GEM_VLAN_SOFT	0x00000020
#define		GEM_VLAN	(GEM_VLAN_HARD | GEM_VLAN_SOFT)
#define		GEM_CKSUM_UDP	0x00000010
#define		GEM_CKSUM_TCP	0x00000008
#define		GEM_CKSUM_IPv6	0x00000004
#define		GEM_CKSUM_IPv4	0x00000002
#define		GEM_NOINTR	0x00000001

	timeout_id_t	intr_watcher_id;

	/* buffer size */
	uint_t	mtu;

	/* performance tuning parameters */
	uint_t	txthr;		/* tx fifo threshoold */
	uint_t	txmaxdma;	/* tx max dma burst size */ 
	uint_t	rxthr;		/* rx fifo threshoold */
	uint_t	rxmaxdma;	/* tx max dma burst size */ 

#ifdef GEM_DEBUG_LEVEL
	int	tx_cnt;
#endif
};

#define	GEM_ATTACH_CHIP(dp)	(dp->gc.gc_attach_chip)(dp)
#define	GEM_RESET_CHIP(dp)	(dp->gc.gc_reset_chip)(dp)
#define	GEM_INIT_CHIP(dp)	(dp->gc.gc_init_chip)(dp)
#define	GEM_START_CHIP(dp)	(dp->gc.gc_start_chip)(dp)
#define	GEM_STOP_CHIP(dp)	(dp->gc.gc_stop_chip)(dp)
#define	GEM_SET_MEDIA(dp)	(dp->gc.gc_set_media)(dp)
#define	GEM_MULTICAST_HASH(dp, addr) (dp->gc.gc_multicast_hash)(dp, addr)
#define	GEM_SET_RX_FILTER(dp)	(dp->gc.gc_set_rx_filter)(dp)
#define	GEM_GET_STATS_CHIP(dp)	(dp->gc.gc_get_stats)(dp)
#define	GEM_INTERRUPT(dp)	(dp->gc.gc_interrupt)(dp)

#define	GEM_TX_DESC_WRITE(dp, slot, dmacookie, frags, flag) \
	(dp->gc.gc_tx_desc_write)(dp, slot, dmacookie, frags, flag)

#define	GEM_RX_DESC_WRITE(dp, slot, dmacookie, frags) \
	(dp->gc.gc_rx_desc_write)(dp, slot, dmacookie, frags)

#define	GEM_TX_DESC_STAT(dp, slot, ndesc)	\
	(dp->gc.gc_tx_desc_stat)(dp, slot, ndesc)
#define	GEM_RX_DESC_STAT(dp, slot, ndesc)	\
	(dp->gc.gc_rx_desc_stat)(dp, slot, ndesc)

#define	GEM_TX_DESC_INIT(dp, slot)	(dp->gc.gc_tx_desc_init)(dp, slot)
#define	GEM_RX_DESC_INIT(dp, slot)	(dp->gc.gc_rx_desc_init)(dp, slot)
#define	GEM_TX_DESC_CLEAN(dp, slot)	(dp->gc.gc_tx_desc_clean)(dp, slot)
#define	GEM_RX_DESC_CLEAN(dp, slot)	(dp->gc.gc_rx_desc_clean)(dp, slot)

#define	GEM_MII_INIT(dp)	(dp->gc.gc_mii_init)(dp)
#define	GEM_MII_CONFIG(dp)	(dp->gc.gc_mii_config)(dp)
#define	GEM_MII_SYNC(dp)	(dp->gc.gc_mii_sync)(dp)
#define	GEM_MII_READ(dp, reg)	(dp->gc.gc_mii_read)(dp, reg)
#define	GEM_MII_WRITE(dp, reg, val)	(dp->gc.gc_mii_write)(dp, reg, val)
#define	GEM_MII_TUNE_PHY(dp)	(dp->gc.gc_mii_tune_phy)(dp)

#define	GEM_PUT_PACKET(dp, m, off, len)	(dp->gc.gc_put_packet)(dp, m, off, len)
#define	GEM_GET_PACKET(dp, rbp, len)	(dp->gc.gc_get_packet)(dp, rbp, len)

/*
 * Exported functions
 */
void gem_generate_macaddr(struct gem_dev *, uint8_t *);
boolean_t gem_get_mac_addr_conf(struct gem_dev *);
int gem_mii_init_default(struct gem_dev *);
int gem_mii_config_default(struct gem_dev *);
clock_t gem_mii_link_check(struct gem_dev *);
int gem_reclaim_txbuf(struct gem_dev *dp);
#ifdef GEM_COMPAT
void gem_restart_tx(struct gem_dev *);
#else
void gem_restart_nic(struct gem_dev *dp, boolean_t keep_tx_buf);
#endif
boolean_t gem_tx_done(struct gem_dev *);
int gem_receive(struct gem_dev *);
struct gem_dev *gem_do_attach(dev_info_t *,
		struct gem_conf *, void *, ddi_acc_handle_t *, void *, int);

int gem_send_common(struct gem_dev *, mblk_t *, uint32_t, uint32_t);
#define	GEM_SEND_COPY	0x80000000
#define	GEM_SEND_CTRL	0x000000ff	/* private flags for control packets */
#define	GEM_SEND_IPv4	0x00000100
#define	GEM_SEND_IPv6	0x00000200
#define	GEM_SEND_TCP	0x00000400
#define	GEM_SEND_UDP	0x00000800

mblk_t *gem_get_packet_default(struct gem_dev *, struct rxbuf *, size_t);

uint32_t gem_ether_crc_le(uint8_t *addr);
uint32_t gem_ether_crc_be(uint8_t *addr);
int gem_do_detach(dev_info_t *);

struct rxbuf *gem_get_rxbuf(struct gem_dev *, int);
void gem_free_rxbuf(struct rxbuf *);

int gem_resume(dev_info_t *);
int gem_suspend(dev_info_t *);
int gem_pci_set_power_state(dev_info_t *, ddi_acc_handle_t, uint_t);
int gem_pci_regs_map_setup(dev_info_t *, uint32_t,
	struct ddi_device_acc_attr *, caddr_t *, ddi_acc_handle_t *);

#define	gem_getinfo	gld_getinfo
#define	gem_open	gld_open
#define	gem_close	gld_close
#define	gem_wput	gld_wput
#define	gem_wsrv	gld_wsrv
#define	gem_rsrv	gld_rsrv

/*
 * binary compatibility for pci_config related functions
 */
#if defined(_ILP32) && !defined(pci_config_get8)
#define	pci_config_get8(h, o)		pci_config_getb(h, o)
#define	pci_config_get16(h, o)		pci_config_getw(h, o)
#define	pci_config_get32(h, o)		pci_config_getl(h, o)
#define	pci_config_get64(h, o)		pci_config_getll(h, o)

#define	pci_config_put8(h, o, v)	pci_config_putb(h, o, v)
#define	pci_config_put16(h, o, v)	pci_config_putw(h, o, v)
#define	pci_config_put32(h, o, v)	pci_config_putl(h, o, v)
#define	pci_config_put64(h, o, v)	pci_config_putll(h, o, v)
#endif
#endif /* _GEM_H_ */
