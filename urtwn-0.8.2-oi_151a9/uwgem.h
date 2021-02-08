/*
 * uwgem.h: General USB to 802.11 MAC driver framework
 * %W% %E%
 * (C) Copyright 2003-2009 Masayuki Murayama KHF04453@nifty.ne.jp
 */

#ifndef __UWGEM_H__
#define __UWGEM_H__

#pragma	ident	"%W% %E%"

/*
 * Useful macros and typedefs
 */
#define	UWGEM_NAME_LEN	32

#define UWGEM_TX_TIMEOUT		(drv_usectohz(3*1000000))
#define UWGEM_TX_TIMEOUT_INTERVAL	(drv_usectohz(1*1000000))
#define UWGEM_LINK_WATCH_INTERVAL	(drv_usectohz(1*1000000))

/* general return code */
#define	UWGEM_SUCCESS	0
#define	UWGEM_FAILURE	1

/* return code of uwgem_tx_done */
#define	INTR_RESTART_TX	0x80000000U

struct uwgem_stats {
	uint32_t	intr;

	uint32_t	crc;
	uint32_t	phy_err;
	uint32_t	tx_err;
	uint32_t	tx_ok;
	uint32_t	tx_retry;

	uint32_t	errrcv;
	uint32_t	overflow;
	uint32_t	frame;
	uint32_t	missed;
	uint32_t	runt;
	uint32_t	frame_too_long;
	uint32_t	norcvbuf;
	uint32_t	sqe;

	uint32_t	collisions;
	uint32_t	first_coll;
	uint32_t	multi_coll;
	uint32_t	excoll;
	uint32_t	xmit_internal_err;
	uint32_t	nocarrier;
	uint32_t	defer;
	uint32_t	errxmt;
	uint32_t	underflow;
	uint32_t	xmtlatecoll;
	uint32_t	noxmtbuf;
	uint32_t	jabber;


	uint64_t	rbytes;
	uint64_t	obytes;
	uint64_t	rpackets;
	uint64_t	opackets;
	uint32_t	rbcast;
	uint32_t	obcast;
	uint32_t	rmcast;
	uint32_t	omcast;
	uint32_t	rcv_internal_err;
};

struct mcast_addr {
	uint8_t		addr[IEEE80211_ADDR_LEN];
	uint32_t	hash;
};

#define	UWGEM_MAXMC	64
#define	UWGEM_MCALLOC	(sizeof(struct mcast_addr) * UWGEM_MAXMC)

#define	SLOT(dp, n)	((n) % (dp)->ugc.uwgc_tx_list_max)

/*
 * mac soft state
 */
struct uwgem_dev {
	/* ieee80211 info must be first */
	struct ieee80211com	ic;

	/* ieee80211 misc managements */
	int			(*ic_newstate)(struct ieee80211com *,
				    enum ieee80211_state, int); /* ok */
	void			(*ic_recv_mgmt)(ieee80211com_t *ic,
				     mblk_t *mp, struct ieee80211_node *in,
				    int subtype, int rssi, uint32_t rstamp);

	dev_info_t		*dip;
	char			name[UWGEM_NAME_LEN];

	/* pointer to usb private data */
	usb_client_dev_data_t	*reg_data;

	/* usb handles */
#define	UWGEM_MAX_BULKIN_PIPES	16
#define	UWGEM_MAX_BULKOUT_PIPES	16
	int			num_bulkin_pipes;
	int			num_bulkout_pipes;
	usb_pipe_handle_t	default_pipe;
	usb_pipe_handle_t	bulkin_pipe[UWGEM_MAX_BULKIN_PIPES];
	usb_pipe_handle_t	bulkout_pipe[UWGEM_MAX_BULKOUT_PIPES];
	usb_pipe_handle_t	intr_pipe;

	/* usb endpoints */
	usb_ep_descr_t		*ep_default;
	usb_ep_descr_t		*ep_bulkin;
	usb_ep_descr_t		*ep_bulkout;
	usb_ep_descr_t		*ep_intr;
	boolean_t		highspeed;

	/* usb policies */
	usb_pipe_policy_t	policy_default;
	usb_pipe_policy_t	policy_bulkin;
	usb_pipe_policy_t	policy_bulkout;
	usb_pipe_policy_t	policy_interrupt;

	/* MAC address information */
	uint8_t			dev_addr[IEEE80211_ADDR_LEN];

	/* RX state and resource management */
	kmutex_t		rxlock;
	int			rx_busy_cnt;
	boolean_t		rx_active;
	kcondvar_t		rx_drain_cv;

	/* RX buffer management */
	int			rx_buf_len;

	/* TX state and resource management */
	kmutex_t		txlock;
	int			tx_busy_cnt;
	usb_bulk_req_t		*tx_free_list;
	kcondvar_t		tx_drain_cv;
	clock_t			tx_start_time;
	int			bulkout_timeout;	/* in second */
	int			tx_max_packets;
	int			tx_seq_num;
	int			tx_intr_pended;

	/* NIC state from OS view */
	int			nic_state;
#define	NIC_STATE_UNKNOWN	0
#define	NIC_STATE_STOPPED	1
#define	NIC_STATE_INITIALIZED	2
#define	NIC_STATE_ONLINE	3

	/* MAC state from hardware view */
	uint_t			mac_state;
#define	MAC_STATE_DISCONNECTED	0	/* it includes suspended state too */
#define	MAC_STATE_STOPPED	1	/* powered up / buf not initialized */
#define	MAC_STATE_INITIALIZED	2	/* initialized */
#define	MAC_STATE_ONLINE	3	/* working correctly  */
#define	MAC_STATE_ERROR		4	/* need to restart nic */

	clock_t			fatal_error;

	/* robustness: timer and watchdog */
	uint_t			watchdog_stop;
	kt_did_t		watchdog_did;
	kcondvar_t		watchdog_cv;
	kmutex_t		watchdog_lock;
	clock_t			watchdog_timeout;
	clock_t			watchdog_interval;

	/* asynchronous mgt queue */
	mblk_t			*mgtq_head;
	mblk_t			*mgtq_tail;

	/* wifi link watcher */
	clock_t			lw_last_check;	/* in tick */
	clock_t			lw_timer;	/* in tick */
#define		LW_SCAN_TIMEOUT		drv_usectohz(10000*1000)
#define		LW_LINKDOWN_TIMEOUT	drv_usectohz(10000*1000)

	clock_t			lw_interval;	/* in tick */
	clock_t			lw_linkup_delay;	/* in tick */

	uint_t			lw_watcher_state;
	kt_did_t		lw_watcher_did;
	kcondvar_t		lw_watcher_wait_cv;
	kmutex_t		lw_lock;
	int			lw_scan_count;
	clock_t			lw_scan_start;

	krwlock_t		dev_state_lock;	/* mac_state and nic_state */
	ksema_t			hal_op_lock;	/* serialize hw operations */
#ifdef CONFIG_DRV_OP_LOCK
	ksema_t			drv_op_lock;	/* hotplug op lock */
#endif

	ksema_t			rxfilter_lock;
	ksema_t			ioctl_lock;
	/* multcast list */
	int			mc_count;
	int			mc_count_req;
	struct mcast_addr	*mc_list;
	/* rx filter mode */
	int			rxmode;
#define		RXMODE_PROMISC		0x01
#define		RXMODE_ALLMULTI_REQ	0x02
#define		RXMODE_MULTI_OVF	0x04
#define		RXMODE_ENABLE		0x08
#define		RXMODE_ALLMULTI		(RXMODE_ALLMULTI_REQ | RXMODE_MULTI_OVF)
#define		RXMODE_BITS	\
			"\020"	\
			"\004ENABLE"	\
			"\003MULTI_OVF"	\
			"\002ALLMULTI_REQ"	\
			"\001PROMISC"

	/* statistcs */
	struct uwgem_stats		stats;

	/* pointer to local structure */
	void			*private;
	int			priv_size;

	/* configuration */
	struct uwgem_conf {
		/* name */
		char		uwgc_name[UWGEM_NAME_LEN];
		int		uwgc_ppa;

		/* specification on usb */
		int	uwgc_ifnum;	/* interface number */
		int	uwgc_alt;	/* alternate */

		/* specification on tx engine */
		int		uwgc_tx_list_max;

		/* specification on rx engine */
		int		uwgc_rx_header_len;
		int		uwgc_rx_list_max;

		/* time out parameters */
		clock_t		uwgc_tx_timeout;
		clock_t		uwgc_tx_timeout_interval;

#ifdef notdef
		/* link manager timeout parameters */
		clock_t	uwgc_linkdown_timeout;
		clock_t	uwgc_link_watch_interval;
		clock_t	uwgc_link_scan_interval;

#endif
		/* I/O methods */

		/* mac operation */
		int	(*uwgc_attach_chip)(struct uwgem_dev *dp);
		int	(*uwgc_reset_chip)(struct uwgem_dev *dp, uint_t how);
		int	(*uwgc_init_chip)(struct uwgem_dev *dp);
		int	(*uwgc_start_chip)(struct uwgem_dev *dp);
		int	(*uwgc_stop_chip)(struct uwgem_dev *dp);
		uint32_t (*uwgc_multicast_hash)(struct uwgem_dev *dp,
		    const uint8_t *);
		int	(*uwgc_set_rx_filter)(struct uwgem_dev *dp);
		int	(*uwgc_get_stats)(struct uwgem_dev *dp);
		void	(*uwgc_interrupt)(struct uwgem_dev *dp, mblk_t *mp);

		/* packet manupilation */
		mblk_t	*(*uwgc_tx_make_packet)(struct uwgem_dev *dp,
		    mblk_t *mp, uint_t, struct ieee80211_node *);
		mblk_t	*(*uwgc_rx_make_packet)(struct uwgem_dev *dp,
		    mblk_t *mp);

		/* link state change */
		int (*uwgc_newstate)(struct uwgem_dev *,
		    enum ieee80211_state, int);
	} ugc;

	int	misc_flag;
#define UWGEM_VLAN	0x0001

	/* buffer size */
	uint_t	mtu;

#ifdef UWGEM_DEBUG_LEVEL
	int	tx_cnt;
#endif
	int	txrate;
};

/*
 * Exported functions
 */
int uwgem_mgt_send(ieee80211com_t *ic, mblk_t *mp, uint8_t type);

int uwgem_bulk_out(struct uwgem_dev *dp, mblk_t *mp, int qid, int async);

int uwgem_ctrl_out(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    void *bp, int size);

int uwgem_ctrl_in(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    void *bp, int size);

int uwgem_ctrl_out_val(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    uint32_t v);

int uwgem_ctrl_in_val(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    void *valp);

void uwgem_generate_macaddr(struct uwgem_dev *, uint8_t *);
boolean_t uwgem_get_mac_addr_conf(struct uwgem_dev *);
void uwgem_lw_start_scan(struct uwgem_dev *);
void uwgem_restart_tx(struct uwgem_dev *);
struct uwgem_dev *uwgem_do_attach(dev_info_t *,
    struct uwgem_conf *, void *, int);
int uwgem_do_detach(dev_info_t *);

int uwgem_resume(dev_info_t *);
int uwgem_suspend(dev_info_t *);
int uwgem_quiesce(dev_info_t *);

#if DEVO_REV < 4
#define UWGEM_STREAM_OPS(dev_ops, attach, detach) \
        DDI_DEFINE_STREAM_OPS(dev_ops, nulldev, nulldev, attach, detach, \
        nodev, NULL, D_MP, NULL)
#else
#define UWGEM_STREAM_OPS(dev_ops, attach, detach) \
	DDI_DEFINE_STREAM_OPS(dev_ops, nulldev, nulldev, attach, detach, \
	nodev, NULL, D_MP, NULL, uwgem_quiesce)
#endif
int uwgem_mod_init(struct dev_ops *, char *);
void uwgem_mod_fini(struct dev_ops *);

void
uwgem_dump_pkt(const char *, const uint8_t *, int32_t, int32_t, int32_t rssi);

void
uwgem_dump_ie(const uint8_t *);

#define UWGEM_GET_DEV(dip) \
	((struct uwgem_dev *)(ddi_get_driver_private(dip)))

#endif /* __UWGEM_H__ */
