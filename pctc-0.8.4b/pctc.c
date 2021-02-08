/*
 *  A Solaris Ethernet driver for the 3com 3c589 card.
 *  Revised from Solaris ddk.
 *  @(#)pctc.c	1.5 02/08/10 
 *  Copright (C) 2001-2002  Masayuki Murayama (KHF04453@nifty.ne.jp)
 */

/*
 Change log:
 2002/03/25  0.8.0 sccs-id 1.1 mrym
	alpha-version
 2002/04/25  0.8.0 sccs-id 1.1 mrym
	alpha-version release  (no significant change)
 2002/06/01  0.8.1 sccs-id 1.1 mrym
	fixed for SPARCompiler on sparc platform
 2002/06/05  0.8.2 sccs-id 1.1 mrym
	fixed for sparc (misalignment on REPOUTSD() in pctc_send)
 2002/08/07  0.8.3 sccs-id 1.2 mrym
	change ppa to slot number (was instance number)
 */

/* Copyright from Solaris DDK (Devece driver Development Kit)  */

/*
 * Copyright (c) 1991-1994,1997, by Sun Microsystems, Inc.
 * All Rights Reserved
 */

/*
 * Solaris headers and macros
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strlog.h>
#include <sys/kmem.h>
#include <sys/stat.h>
#include <sys/kstat.h>
#include <sys/dlpi.h>
#include <sys/strsun.h>
#include <sys/ethernet.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#include <sys/gld.h>
#include <sys/pccard.h>

#include <sys/ddi_impldefs.h>
#include <sys/pcmcia.h>
#include <sys/cs_stubs.h>

#include "pctcreg.h"

char	ident[] = "3c589 PCMCIA card driver v" VERSION;
char	_depends_on[] = {"misc/gld misc/cs"};

#define	ROUNDUP(x, y)	P2ROUNDUP(x, y)

/* boolean type */
typedef	uint32_t	bool_t;
#ifdef FALSE
# undef FALSE
#endif
#define	FALSE	(0)
#define	TRUE	(!FALSE)

#ifdef DEBUG_LEVEL
static int pctc_debug = DEBUG_LEVEL;
#define DPRINTF(n, args) if (pctc_debug>(n)) cmn_err args
#else
#define DPRINTF(n, args)
#endif

/*
 * I/O Macros
 */
#define	INB(lp, p)		csx_Get8((lp)->reg_handle, (uint32_t)(p))
#define	INW(lp, p)		csx_Get16((lp)->reg_handle, (uint32_t)(p))
#define	INL(lp, p)		csx_Get32((lp)->reg_handle, (uint32_t)(p))

#define	OUTB(lp, p, v)		csx_Put8((lp)->reg_handle, (uint32_t)(p), v)
#define	OUTW(lp, p, v)		csx_Put16((lp)->reg_handle, (uint32_t)(p), v)
#define	OUTL(lp, p, v)		csx_Put32((lp)->reg_handle, (uint32_t)(p), v)

#define	REPINSD(lp, p, addr, len)	\
	csx_RepGet32(lp->reg_handle, (uint32_t *)(addr), p, len, \
			CS_DEV_NO_AUTOINCR)

#define	REPINSB(lp, p, addr, len)	\
	csx_RepGet8(lp->reg_handle, (uint8_t *)(addr), p, len, \
			CS_DEV_NO_AUTOINCR)

#define	REPOUTSD(lp, p, addr, len)	\
	csx_RepPut32(lp->reg_handle, (uint32_t *)(addr), p, len, \
			CS_DEV_NO_AUTOINCR)

#define	REPOUTSB(lp, p, addr, len)	\
	csx_RepPut8(lp->reg_handle, (uint8_t *)(addr), p, len, \
			CS_DEV_NO_AUTOINCR)

#define	EXEC_CMD(lp, cmd)	OUTW(lp, WX_COMMAND, cmd)
#define	EXEC_CMD_WAIT(lp, cmd)	pctc_exec_cmd_wait(lp, cmd)

#define	SET_WIN(lp, n)	EXEC_CMD(lp, OP_SELECT_REGISTER_WINDOW | (n))
				

/* Time in jiffies before concluding Tx hung */
#define TX_TIMEOUT_TICKS	drv_usectohz(10000000)	/* 10 sec */
#define TX_TIMEOUT_INTERVAL	drv_usectohz(1000000)	/* 1 sec */

#define	OUR_INTR_BITS	(INTR_AF | INTR_TC | INTR_RC | INTR_US)

struct pctc_stats {
	/* tx statistics */
	int	errxmt;
	int	errrcv;
	int	collisions;
	int	excoll;
	int	defer;
	int	underflow;
	int	xmtlatecoll;
	int	nocarrier;
	int	first_coll;
	int	multi_coll;

	/* rx statistics */
	int	missed;
	int	rx_errs[8];
#define		rx_err_frame		rx_errs[RX_STATUS_CODE_FRAME]
#define		rx_err_crc		rx_errs[RX_STATUS_CODE_CRC]
#define		rx_err_overflow		rx_errs[RX_STATUS_CODE_OVERRUN]
#define		rx_err_short		rx_errs[RX_STATUS_CODE_RUNT]
#define		rx_err_frame_too_long	rx_errs[RX_STATUS_CODE_OVERSIZE]
	int	norcvbuf;

	int	intr;
};

struct pctc_dev {
	/* device structure link */
	dev_info_t		*dip;
	char			name[32];
	client_handle_t		handle;
	gld_mac_info_t		*macinfo;

	int			base_addr;	/* assigned port address */
	acc_handle_t		reg_handle;
	u_int			sn;		/* socket number */

	/* ethernet addrss */
	struct ether_addr	cur_addr;	/* current mac address */
	struct ether_addr	dev_addr;	/* factory mac address */

	/* device status */
	bool_t			open;
	bool_t			started;
	int			probe_port;

	/* low-level interrupt */
	kmutex_t		tc_intrlock;
	kmutex_t		tc_xmitlock;
	krwlock_t		tc_winlock;
	ddi_iblock_cookie_t	irq_iblk_cookie;
	kcondvar_t		drain_cv;
	bool_t			intr_busy;

	/* hi-level interrupt */
	kmutex_t		hi_intrlock;
	bool_t			intr_hilevel;
	bool_t			softintr_req;
	ddi_softintr_t		soft_id;

	/* event lock */
	kmutex_t		event_mutex;
	kmutex_t		event_hi_mutex;
	kcondvar_t		readywait_cv;
	ddi_iblock_cookie_t	iblk_cookie;
	ddi_iblock_cookie_t	softint_cookie;

	/* tx management */
	clock_t			tx_start_time;
	clock_t			tx_acked_time;
	volatile timeout_id_t	timeout_id;
	int			tx_blocked;
#define		FIFO_NOTBLOCKED	0
#define		FIFO_BUSY	1
#define		FIFO_NOSPACE	2
#define		FIFO_STOPPED	3

	bool_t			tx_busy;
	int			tx_fifo_size;
	int			tx_threshold;

	/* multicast support */
	int			mc_count;
	int			rxmode;
#define	RXMODE_PROMISC		0x1
#define	RXMODE_ALLMULTI		0x2

	int			card_event;
#define	EVENT_CARD_INSERTED	0x01
#define	CARD_READY		0x02
#define	WAITING_CARD_INSERTED	0x80

	int			media;

	struct pctc_stats	stats;
};

/* private functions */
static  int pctc_event(event_t event, int prio, event_callback_args_t *args);
static  void pctc_release(struct pctc_dev *lp);

static int pctc_do_attach_cs(struct pctc_dev *lp, gld_mac_info_t *);
static void pctc_do_detach_cs(struct pctc_dev *lp);
static void pctc_read_stats(struct pctc_dev *dev);
static void pctc_start_chip(struct pctc_dev *dev);
static void pctc_init_chip(struct pctc_dev *dev);
static int  pctc_receive(struct pctc_dev *dev);
static int  pctc_transmit_done(struct pctc_dev *dev);
static int  pctc_read_eeprom(struct pctc_dev *dev, int offset);
static void pctc_tx_timeout(void *data);
static void pctc_set_rx_filter(struct pctc_dev *dev);
static void pctc_set_media(struct pctc_dev *dev);
static void pctc_exec_cmd_wait(struct pctc_dev *dev, int cmd);

/* DDI/DKI functions */
static	int pctc_probe(dev_info_t *);
static	int pctc_attach(dev_info_t *, ddi_attach_cmd_t);
static	int pctc_detach(dev_info_t *, ddi_detach_cmd_t);

/* GLD interfaces */
static	int pctc_reset(gld_mac_info_t *);
static	int pctc_start(gld_mac_info_t *);
static	int pctc_stop(gld_mac_info_t *);
static	int pctc_set_mac_address(gld_mac_info_t *, unsigned char *);
static	int pctc_set_multicast(gld_mac_info_t *, struct ether_addr *, int);
static	int pctc_set_promiscuous(gld_mac_info_t *, int);
static	int pctc_get_stats(gld_mac_info_t *, struct gld_stats *);
static	int pctc_send(gld_mac_info_t *, mblk_t *);
static	u_int pctc_intr(gld_mac_info_t *);
static	u_int pctc_highintr(gld_mac_info_t *);

/* Data access requirements. */
static struct ddi_device_acc_attr dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

static	struct module_info pctc_minfo = {
	0,		/* mi_idnum */
	"pctc",		/* mi_idname */
	0,		/* mi_minpsz */
	ETHERMTU,	/* mi_maxpsz */
	3*1024,		/* mi_hiwat */
	1,		/* mi_lowat */
};

static	struct qinit pctc_rinit = {
	(int (*)()) NULL,	/* qi_putp */
	gld_rsrv,		/* qi_srvp */
	gld_open,		/* qi_qopen */
	gld_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&pctc_minfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit pctc_winit = {
	gld_wput,		/* qi_putp */
	gld_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&pctc_minfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	pctc_info = {
	&pctc_rinit,		/* st_rdinit */
	&pctc_winit,		/* st_wrinit */
	NULL,			/* st_muxrinit */
	NULL			/* st_muxwrinit */
};

static	struct cb_ops cb_pctc_ops = {
	nulldev,	/* cb_open */
	nulldev,	/* cb_close */
	nodev,		/* cb_strategy */
	nodev,		/* cb_print */
	nodev,		/* cb_dump */
	nodev,		/* cb_read */
	nodev,		/* cb_write */
	nodev,		/* cb_ioctl */
	nodev,		/* cb_devmap */
	nodev,		/* cb_mmap */
	nodev,		/* cb_segmap */
	nochpoll,	/* cb_chpoll */
	ddi_prop_op,	/* cb_prop_op */
	&pctc_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops pctc_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gld_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	pctc_probe,	/* devo_probe */
	pctc_attach,	/* devo_attach */
	pctc_detach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_pctc_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&pctc_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/*
 * Ethernet broadcast address definition.
 */
static struct ether_addr etherbroadcastaddr = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

int
_init(void)
{
	DPRINTF(2, (CE_CONT, "pctc: _init: called"));
	return mod_install(&modlinkage);
}

int
_fini(void)
{
	return mod_remove(&modlinkage);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

/*ARGSUSED*/
static int
pctc_probe(dev_info_t *dip)
{
	DPRINTF(2, (CE_CONT, "?pctc_probe: called"));
	return (DDI_PROBE_SUCCESS);
}

static int
pctc_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct pctc_dev		*lp;
	ddi_iblock_cookie_t	c;
	gld_mac_info_t		*macinfo;
	char			propname[32];
	int			unit;

	unit = ddi_get_instance(dip);

	DPRINTF(2, (CE_CONT, "?pctc%d: pctc_attach: called", unit));

	if (cmd == DDI_ATTACH) {
		DPRINTF(2, (CE_CONT, "pctc%d: pctc_attach: called cmd:ATTACH",
				unit));

		macinfo = gld_mac_alloc(dip);
		if (macinfo == NULL) {
			cmn_err(CE_WARN,
				"pctc%d: pctc_attach: gld_mac_alloc failed",
				unit);
			return DDI_FAILURE;
		}

		/*
		 * Allocate soft data structure
		 */
		lp = (struct pctc_dev *)kmem_zalloc(sizeof(*lp), KM_SLEEP);
		if (lp == NULL) {
			gld_mac_free(macinfo);
			cmn_err(CE_WARN,
				"pctc%d: pctc_attach: kmem_zalloc failed",
				unit);
			return DDI_FAILURE;
		}

		lp->dip = dip;
		lp->macinfo = macinfo;

		sprintf(lp->name, "%s%d", ddi_driver_name(dip), unit);

		/* GLD ver2 don't lock on xmit */
		macinfo->gldm_devinfo = dip;
		macinfo->gldm_private = (caddr_t)lp;

		macinfo->gldm_reset        = pctc_reset;
		macinfo->gldm_start        = pctc_start;
		macinfo->gldm_stop         = pctc_stop;
		macinfo->gldm_set_mac_addr = pctc_set_mac_address;
		macinfo->gldm_send         = pctc_send;
		macinfo->gldm_set_promiscuous = pctc_set_promiscuous;
		macinfo->gldm_get_stats    = pctc_get_stats;
		macinfo->gldm_ioctl        = NULL; 
		macinfo->gldm_set_multicast= pctc_set_multicast;
		macinfo->gldm_intr         = pctc_intr;
		macinfo->gldm_mctl         = NULL;

		macinfo->gldm_ident   = ident;
		macinfo->gldm_type    = DL_ETHER;
		macinfo->gldm_minpkt  = 0;
		macinfo->gldm_maxpkt  = ETHERMTU;
		macinfo->gldm_addrlen = ETHERADDRL;
		macinfo->gldm_saplen  = -2;

		if (pctc_do_attach_cs(lp, macinfo)) {
			cmn_err(CE_WARN,
				"%s: pctc_attach: pctc_do_attach_cs failed",
				lp->name);
			goto err_free_private;
		}

		macinfo->gldm_ppa     = lp->sn;
		macinfo->gldm_cookie  = lp->irq_iblk_cookie;

		/*
		 * Get the vendor ethernet address.
		 */
		macinfo->gldm_vendor_addr =
					lp->dev_addr.ether_addr_octet;
		macinfo->gldm_broadcast_addr =
					etherbroadcastaddr.ether_addr_octet;

		if (gld_register(dip, "pctc", macinfo) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
				"%s: pctc_attach: gld_register failed",
				lp->name);
			goto err_free_private;
		}

		if (lp->intr_hilevel) {
			if (ddi_add_softintr(dip,  DDI_SOFTINT_MED,
					&lp->soft_id,
					NULL, NULL,
					(uint_t (*)(caddr_t))&gld_intr,
					(caddr_t)macinfo) != DDI_SUCCESS) {
				cmn_err(CE_WARN,
				"%s: pctc_attach: ddi_add_softintr failed",
					lp->name);
				goto err_unregister;
			}
		}

		DPRINTF(2, (CE_CONT, "%s: pctc_attach: return: successed",
			lp->name));

		return (DDI_SUCCESS);

err:
		/* release allocated resource */
err_unregister:
		gld_unregister(macinfo);
err_free_private:
		kmem_free((caddr_t)lp, sizeof (struct pctc_dev));

		cmn_err(CE_WARN, "pctc%d: pctc_attach: return: failure", unit);

		return DDI_FAILURE;
	}

	cmn_err(CE_WARN, "pctc%d: pctc_attach: cmd:%d return: failure",
		unit, cmd);

	return DDI_FAILURE;
}

static int
pctc_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct pctc_dev *lp;
	gld_mac_info_t	*macinfo;
	timeout_id_t	id;

	if (cmd == DDI_DETACH) {
		macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
		lp = (struct pctc_dev *)macinfo->gldm_private;

		if (lp->intr_hilevel) {

			mutex_enter(&lp->hi_intrlock);
			id = lp->soft_id;
			lp->soft_id = 0;
			mutex_exit(&lp->hi_intrlock);

			ddi_remove_softintr(id);
		}

		gld_unregister(macinfo);

		/*
		 * Unregister with cardservice
		 */
		pctc_do_detach_cs(lp);

		/*
		 * Release memory and mapping resources
		 */
		kmem_free((caddr_t)lp, sizeof(struct pctc_dev));

		gld_mac_free(macinfo);

		return DDI_SUCCESS;
	} 

	return DDI_FAILURE;
}

/* ================================================================ */

/*
 * Solaris Card Service interface
 */

typedef struct cs_csfunc2text_strings_t {
        uint32_t        item;
        char            *text;
} cs_csfunc2text_strings_t;

static void
pctc_errprint(struct pctc_dev *lp, int func, int ret)
{
	error2text_t		cft;
	cs_csfunc2text_strings_t *cfs;
	extern cs_csfunc2text_strings_t cs_csfunc2text_funcstrings[];
	char			func_name[40];

	sprintf(func_name, "func #%d", func); 

	for (cfs = cs_csfunc2text_funcstrings;
				cfs->item != CSFuncListEnd; cfs++) {
		if (cfs->item == func) {
			strncpy(func_name, cfs->text, 39);
			break;
		}
	}

	cft.item = ret;
	csx_Error2Text(&cft);
	cmn_err(CE_WARN, "%s: func:%s: failed %s (0x%x)",
			lp->name, func_name, cft.text, ret);
}

static int
pctc_do_attach_cs(struct pctc_dev *lp, gld_mac_info_t *macinfo)
{
	client_reg_t		client_reg;
	map_log_socket_t        map_log_socket;
	sockmask_t              sockmask;
	int			i, ret;
	ddi_iblock_cookie_t	c;
	char			propname[32];

	DPRINTF(3, (CE_CONT, "?%s: pctc_do_attach_cs: called", lp->name));

	/* Register with Card Services */
	client_reg.Attributes= INFO_IO_CLIENT | INFO_CARD_SHARE;
        client_reg.EventMask = (CS_EVENT_CARD_READY |
				CS_EVENT_CARD_INSERTION |
				CS_EVENT_CARD_REMOVAL |
				CS_EVENT_CARD_REMOVAL_LOWP);
	client_reg.event_handler = (csfunction_t *)&pctc_event;
	client_reg.Version       = CS_VERSION;
	client_reg.event_callback_args.client_data = lp;
	client_reg.dip           = lp->dip;

	if ((ret = csx_RegisterClient(&lp->handle, &client_reg))
			!= CS_SUCCESS) {
		pctc_errprint(lp, RegisterClient, ret);
		return DDI_FAILURE;
	}
	lp->iblk_cookie = *(client_reg.iblk_cookie);

	/* Get logical socket number and store in pcepp_state_t */
	if ((ret = csx_MapLogSocket(lp->handle, &map_log_socket))
			!= CS_SUCCESS) {
		error2text_t cft;

		cft.item = ret;
		csx_Error2Text(&cft);
		cmn_err(CE_WARN, "pctc_do_attach_cs: %s:"
				"MapLogSocket failed %s (0x%x)\n",
				lp->name, cft.text, ret);
		return DDI_FAILURE;
	}
	lp->sn = map_log_socket.PhySocket;

	sprintf(lp->name, "%s%d@%d",
		ddi_driver_name(lp->dip), ddi_get_instance(lp->dip), lp->sn);

	/* select media */
	sprintf(propname, "pctc%d-media-type", lp->sn);
	lp->media = ddi_getprop(DDI_DEV_T_ANY, lp->dip, DDI_PROP_DONTPASS,
				propname, ADDRCONF_XCVR_10BASET);

	switch (lp->media) {
	case ADDRCONF_XCVR_10BASET:
	case ADDRCONF_XCVR_10BASE2:
	case ADDRCONF_XCVR_AUI:
		/* OK */
		break;

	default:
		cmn_err(CE_WARN,
			"%s: unknown media type (%d) specified. "
			"using 10baseT",
			lp->name, lp->media);
		lp->media = ADDRCONF_XCVR_10BASET;
		break;
	}

	/*
	 * Setup event related mutex and cv
	 */
	mutex_init(&lp->event_hi_mutex, NULL,
		    MUTEX_DRIVER, *(client_reg.iblk_cookie));

	ddi_get_soft_iblock_cookie(lp->dip, DDI_SOFTINT_MED, &c);
	lp->softint_cookie = c;
	mutex_init(&lp->event_mutex, NULL, MUTEX_DRIVER, c);
	cv_init(&lp->readywait_cv, NULL, CV_DRIVER, (void *)NULL);
	cv_init(&lp->drain_cv, NULL, CV_DRIVER, (void *)NULL);
	mutex_enter(&lp->event_mutex);

	/*
	 * After the RequestSocketMask call,
	 *      we can start receiving events
	 */
	sockmask.EventMask = (CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL);

	ret = csx_RequestSocketMask(lp->handle, &sockmask);
	if (ret != CS_SUCCESS) {
		error2text_t cft;

		cft.item = ret;
		csx_Error2Text(&cft);
		cmn_err(CE_WARN, "%s.%d: attach, "
			    "RequestSocketMask failed %s (0x%x)\n",
			    lp->name, lp->sn, cft.text, ret);
		mutex_exit(&lp->event_mutex);
		goto err;
	}

	/*
	 * If the card is inserted and this attach is triggered
	 *      by an open, that open will wait until
	 *      pcepp_card_insertion() is completed
	 */

	lp->card_event |= WAITING_CARD_INSERTED;
	while ((lp->card_event & EVENT_CARD_INSERTED) == 0) {
		cv_wait(&lp->readywait_cv, &lp->event_mutex);
	}

	/*
	 * PC Card now must be present and fully functional
	 */
	if ((lp->card_event & CARD_READY) == 0) {
		cmn_err(CE_WARN, "%s.%d: attach, card not ready\n",
			lp->name, lp->sn);
		mutex_exit(&lp->event_mutex);
		goto err;
	}
	mutex_exit(&lp->event_mutex);
	return DDI_SUCCESS;

err:
	mutex_destroy(&lp->event_mutex);
	mutex_destroy(&lp->event_hi_mutex);
	cv_destroy(&lp->readywait_cv);

	return DDI_FAILURE;
} /* pctc_do_attach_cs */

static void pctc_do_detach_cs(struct pctc_dev *lp)
{
	release_socket_mask_t   rsm;
	int		ret;

	if (lp->card_event & CARD_READY) {
		pctc_release(lp);
	}

	if (lp->handle) {
		ret = csx_ReleaseSocketMask(lp->handle, &rsm);
		if (ret != CS_SUCCESS) {
			pctc_errprint(lp, ReleaseSocketMask, ret);
		}

		ret = csx_DeregisterClient(lp->handle);
		if (ret != CS_SUCCESS) {
			pctc_errprint(lp, DeregisterClient, ret);
		}

		/* release event related mutex and cv */
		mutex_destroy(&lp->event_mutex);
		mutex_destroy(&lp->event_hi_mutex);
		cv_destroy(&lp->readywait_cv);

		lp->handle = 0;
	}
} /* pctc_detach */

static boolean_t
pctc_card_insertion(struct pctc_dev *lp)
{
	client_handle_t		handle = lp->handle;
	tuple_t			tuple;
	cisparse_t		parse;
	u_short			*buf;
	ddi_iblock_cookie_t	c;
	int			i, j, multi = 0;
	int			softinfo;
	int			err;
	config_req_t		config_req;
	io_req_t		io_req;
	irq_req_t		irq_req;
	static char		*media[] = { "10BASE-T", "AUI",
					"Unknown media", "10BASE2"};

	DPRINTF(2, (CE_CONT, "?%s: pctc_card_insertion: called", lp->name));

	bzero(&config_req, sizeof(config_req));
	config_req.Attributes = CONF_ENABLE_IRQ_STEERING; /* ?? */
	config_req.IntType    = SOCKET_INTERFACE_MEMORY_AND_IO;
	config_req.Present    = CONFIG_OPTION_REG_PRESENT;
	config_req.Vcc        = 50;
	config_req.ConfigIndex= 1;

	tuple.Attributes = 0;
	tuple.DesiredTuple = CISTPL_CONFIG;
	if ((csx_GetFirstTuple(handle, &tuple)) != CS_SUCCESS) {
		pctc_errprint(lp, GetFirstTuple, err);
		goto failed;
	}

	buf = (u_short *)tuple.TupleData;
	tuple.TupleDataMax = sizeof(tuple.TupleData);
	tuple.TupleOffset = 0;

	if ((err = csx_GetTupleData(handle, &tuple)) != CS_SUCCESS) {
		pctc_errprint(lp, GetTupleData, err);
		goto failed;
	}

	if ((err = csx_Parse_CISTPL_CONFIG(handle, &tuple, &parse.config))
			!= CS_SUCCESS) {
		pctc_errprint(lp, ParseTuple, err);
		goto failed;
	}

	config_req.ConfigBase = parse.config.base;
	config_req.Present = parse.config.present;

	/*
	 * Allocate I/O range
	 */
	bzero(&io_req, sizeof(io_req));
	io_req.NumPorts1   = 16;
	io_req.Attributes1 = IO_DATA_PATH_WIDTH_16;
	io_req.IOAddrLines = 4;
	if ((err = csx_RequestIO(handle, &io_req)) != CS_SUCCESS) {
		pctc_errprint(lp, RequestIO, err);
		goto failed;
	}
	/* save allocated I/O address */
	lp->base_addr  = io_req.BasePort1.base;
	lp->reg_handle = io_req.BasePort1.handle;

	/*
	 * Register Interrupt handler
	 */
	bzero(&irq_req, sizeof(irq_req));
	lp->intr_hilevel = ddi_intr_hilevel(lp->dip, 0);

	DPRINTF(2, (CE_CONT, "%s: interrupt level is %s",
		lp->name, lp->intr_hilevel ? "high" : "low"));

	if (lp->intr_hilevel) {
		irq_req.irq_handler = (csfunction_t *)&pctc_highintr;
	} else {
		irq_req.irq_handler = (csfunction_t *)&gld_intr;
	}
	irq_req.Attributes  = IRQ_TYPE_EXCLUSIVE;
	irq_req.irq_handler_arg = (void *)lp->macinfo;

	if ((err = csx_RequestIRQ(handle, &irq_req)) != CS_SUCCESS) {
		pctc_errprint(lp, RequestIRQ, err);
		goto failed;
	}
	c = *(irq_req.iblk_cookie);
	if (lp->intr_hilevel) {
		/* make hi-level and low-level locks */
		mutex_init(&lp->hi_intrlock, NULL, MUTEX_DRIVER, c);

		ddi_get_soft_iblock_cookie(lp->dip, DDI_SOFTINT_MED, &c);
		mutex_init(&lp->tc_intrlock, NULL, MUTEX_DRIVER, c);
		mutex_init(&lp->tc_xmitlock, NULL, MUTEX_DRIVER, c);
		cv_init(&lp->drain_cv, NULL, CV_DRIVER, (void *)NULL);
		rw_init(&lp->tc_winlock, NULL, RW_DRIVER, c);
	}
	else {
		/* make low-level locks */
		mutex_init(&lp->tc_intrlock, NULL, MUTEX_DRIVER, c);
		mutex_init(&lp->tc_xmitlock, NULL, MUTEX_DRIVER, c);
		cv_init(&lp->drain_cv, NULL, CV_DRIVER, (void *)NULL);
		rw_init(&lp->tc_winlock, NULL, RW_DRIVER, c);
	}
	lp->irq_iblk_cookie = c;

	if ((err=csx_RequestConfiguration(handle, &config_req)) != CS_SUCCESS) {
		pctc_errprint(lp, RequestConfiguration, err);
		goto failed;
	}


	rw_enter(&lp->tc_winlock, RW_WRITER);

	for (i = 0; i < ETHERADDRL/2; i++) {
		int	val;

		val = pctc_read_eeprom(lp, PROM_ADDR_MAC + i);
		lp->dev_addr.ether_addr_octet[i*2]   = val >> 8;
		lp->dev_addr.ether_addr_octet[i*2+1] = val;
	}
	lp->cur_addr = lp->dev_addr;

	softinfo = pctc_read_eeprom(lp, PROM_ADDR_SOFT_INFO);

	SET_WIN(lp, 1);
	lp->tx_fifo_size = INW(lp, W1_FREE_XMIT_BYTES);
	rw_exit(&lp->tc_winlock);

	cmn_err(CE_CONT, "?%s: 3Com 3c%s at I/O addr 0x%x, %s, "
		"hw_addr %02x:%02x:%02x:%02x:%02x:%02x\n",
		lp->name, (multi ? "562" : "589"),
		lp->base_addr, 
		media[lp->media],
		lp->dev_addr.ether_addr_octet[0],
		lp->dev_addr.ether_addr_octet[1],
		lp->dev_addr.ether_addr_octet[2],
		lp->dev_addr.ether_addr_octet[3],
		lp->dev_addr.ether_addr_octet[4],
		lp->dev_addr.ether_addr_octet[5]);

	DPRINTF(0, (CE_CONT, "?%s: softinfo:0x%04x, tx_fifo:0x%04x",
		lp->name, softinfo, lp->tx_fifo_size));

	return TRUE;

failed:
	pctc_release(lp);
	return FALSE;
} /* pctc_card_insertion */

static void
pctc_release(struct pctc_dev *lp)
{
	int			ret;
	release_config_t	rc;
	io_req_t		io_req;
	irq_req_t		irq_req;

	DPRINTF(2, (CE_CONT, "?%s: pctc_release: called", lp->name));

	if ((ret = csx_ReleaseConfiguration(lp->handle, &rc)) != CS_SUCCESS) {
		pctc_errprint(lp, ReleaseConfiguration, ret);
	}

	if ((ret = csx_ReleaseIO(lp->handle, &io_req)) != CS_SUCCESS) {
		pctc_errprint(lp, ReleaseIO, ret);
	}

	if ((ret = csx_ReleaseIRQ(lp->handle, &irq_req)) != CS_SUCCESS) {
		pctc_errprint(lp, ReleaseIRQ, ret);
	}

	if (lp->intr_hilevel) {
		mutex_destroy(&lp->hi_intrlock);
	}
	mutex_destroy(&lp->tc_intrlock);
	mutex_destroy(&lp->tc_xmitlock);
	cv_destroy(&lp->drain_cv);
	rw_destroy(&lp->tc_winlock);

	lp->card_event &= ~CARD_READY;
} /* pctc_release */


static int
pctc_event(event_t event, int priority,
		       event_callback_args_t *args)
{
	struct pctc_dev	*lp = args->client_data;
	int			retcode;

	DPRINTF(3, (CE_CONT, "?%s: pctc_event: called, event: 0x%x",
			lp->name, event));

	retcode = CS_SUCCESS;

	if (priority & CS_EVENT_PRI_HIGH) {
		mutex_enter(&lp->event_hi_mutex);
	}
	else {
		mutex_enter(&lp->event_mutex);
	}

	switch (event) {
	case CS_EVENT_CARD_REMOVAL:
		DPRINTF(1, (CE_CONT, "?%s: pctc_event: CS_EVENT_CARD_REMOVAL",
				lp->name));
		lp->card_event &= ~EVENT_CARD_INSERTED;
		if (lp->card_event & CARD_READY) {
			lp->started = FALSE;
			lp->tx_busy = 1;
		}
		break;

	case CS_EVENT_CARD_INSERTION:
		DPRINTF(1, (CE_CONT, "?%s: pctc_event: CS_EVENT_CARD_INSERTION",
				lp->name));
		ASSERT(priority & CS_EVENT_PRI_LOW);
		lp->card_event |= EVENT_CARD_INSERTED;
		if (pctc_card_insertion(lp)) {
			lp->card_event |= CARD_READY;
		}
		if (lp->card_event & WAITING_CARD_INSERTED) {
			lp->card_event &= ~WAITING_CARD_INSERTED;
			cv_signal(&lp->readywait_cv);
		}
		break;
	}

	if (priority & CS_EVENT_PRI_HIGH) {
		mutex_exit(&lp->event_hi_mutex);
		/* How can I call gld_sched at low-interrput level */
	}
	else {
		mutex_exit(&lp->event_mutex);
	}

	return retcode;
} /* pctc_event */

/*====================================================================*/
/*
 * Solaris GLD interface implementation
 */
/*====================================================================*/

/* prooved */
static int
pctc_reset(gld_mac_info_t *macinfo)
{
	struct pctc_dev *lp  = (struct pctc_dev *)macinfo->gldm_private;

	DPRINTF(2, (CE_CONT, "?%s: pctc_reset: called", lp->name));

	/* reset mac address to factory address */
	lp->cur_addr = lp->dev_addr;

	/* reset multicast list */
	lp->mc_count = 0;

	mutex_enter(&lp->tc_intrlock);
	mutex_enter(&lp->tc_xmitlock);
	rw_enter(&lp->tc_winlock, RW_WRITER);
	pctc_init_chip(lp);
	rw_exit(&lp->tc_winlock);
	mutex_exit(&lp->tc_xmitlock);
	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}

/* prooved */
static int
pctc_start(gld_mac_info_t *macinfo)
{
	struct pctc_dev *lp = (struct pctc_dev *)macinfo->gldm_private;

	DPRINTF(2, (CE_CONT, "?%s: pctc_start: called", lp->name));

	mutex_enter(&lp->tc_intrlock);
	mutex_enter(&lp->tc_xmitlock);
	lp->started = TRUE;
	lp->open    = TRUE;
	pctc_start_chip(lp);
	mutex_exit(&lp->tc_xmitlock);
	mutex_exit(&lp->tc_intrlock);

	lp->timeout_id =
		timeout(pctc_tx_timeout, (void *)lp, TX_TIMEOUT_INTERVAL);

	DPRINTF(2, (CE_CONT, "?%s: pctc_start: status %b",
			lp->name, INW(lp, WX_STATUS), STATUS_BITS));

	return GLD_SUCCESS;
}

static int
pctc_set_mac_address(gld_mac_info_t *macinfo, unsigned char *addr)
{
	struct pctc_dev *lp;
	int		i;

	lp = (struct pctc_dev *)macinfo->gldm_private;

	DPRINTF(2, (CE_CONT, "?%s: pctc_set_mac_address: called started:%d "
		"addr: %02x:%02x:%02x:%02x:%02x:%02x",
		lp->name, lp->started,
		addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]));

	bcopy(addr, &lp->cur_addr.ether_addr_octet[0], ETHERADDRL);

	mutex_enter(&lp->tc_intrlock);	/* for command */
	mutex_enter(&lp->tc_xmitlock);	/* for lp->started */

	if (lp->started) {
		EXEC_CMD(lp, OP_RX_DISABLE);
	}

	rw_enter(&lp->tc_winlock, RW_WRITER);
	SET_WIN(lp, 2);
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(lp, W2_ADDRESS + i, lp->cur_addr.ether_addr_octet[i]);
	}
	SET_WIN(lp, 1);
	rw_exit(&lp->tc_winlock);

	if (lp->started) {
		EXEC_CMD(lp, OP_RX_ENABLE);
	}

	mutex_exit(&lp->tc_xmitlock);
	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}

static int
pctc_set_multicast(gld_mac_info_t *macinfo, struct ether_addr *ep, int flag)
{
	struct pctc_dev *lp;

	lp = (struct pctc_dev *)macinfo->gldm_private;

	DPRINTF(2, (CE_CONT, "?%s: pctc_set_multicast: called started:%d "
		"flag:%d, addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
		lp->name, lp->started, flag,
		ep->ether_addr_octet[0], ep->ether_addr_octet[1],
		ep->ether_addr_octet[2], ep->ether_addr_octet[3],
		ep->ether_addr_octet[4], ep->ether_addr_octet[5]));

	if (flag == GLD_MULTI_ENABLE) {
		lp->mc_count++;
	}
	else {
		ASSERT(flag == GLD_MULTI_DISABLE);
		lp->mc_count--;
	}

	mutex_enter(&lp->tc_intrlock);
	pctc_set_rx_filter(lp);
	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}

static int
pctc_set_promiscuous(gld_mac_info_t *macinfo, int flag)
{
	struct pctc_dev *lp;

	lp = (struct pctc_dev *)macinfo->gldm_private;

	DPRINTF(2, (CE_CONT,
		"?%s: pctc_set_promiscuous: called. started:%d flag:%d",
		lp->name, lp->started, flag));

	switch(flag) {
	case GLD_MAC_PROMISC_NONE:
		lp->rxmode &= ~(RXMODE_PROMISC | RXMODE_ALLMULTI);
		break;

	case GLD_MAC_PROMISC_PHYS:
		lp->rxmode |= RXMODE_PROMISC;
		break;

	case GLD_MAC_PROMISC_MULTI:
		lp->rxmode |= RXMODE_ALLMULTI;
		break;
	}

	mutex_enter(&lp->tc_intrlock);
	pctc_set_rx_filter(lp);
	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}

static	int
pctc_get_stats(gld_mac_info_t *macinfo, struct gld_stats *gs)
{
	struct pctc_dev *lp;
	struct pctc_stats *ts;

	lp = (struct pctc_dev *)macinfo->gldm_private;

	mutex_enter(&lp->tc_intrlock); /* for lp->stats */

	pctc_read_stats(lp);
	ts = &lp->stats;

	gs->glds_errxmt    = ts->errxmt;
	gs->glds_errrcv    = ts->errrcv;

	gs->glds_collisions= ts->collisions;
	gs->glds_excoll    = ts->excoll;
	gs->glds_defer     = ts->defer;
	gs->glds_underflow = ts->underflow;
	gs->glds_xmtlatecoll = ts->xmtlatecoll;

	gs->glds_frame     = ts->rx_err_frame;
	gs->glds_crc       = ts->rx_err_crc;
	gs->glds_overflow  = ts->rx_err_overflow;
	gs->glds_short     = ts->rx_err_short;

	gs->glds_missed    = ts->missed;
	gs->glds_nocarrier = ts->nocarrier;

	gs->glds_norcvbuf  = ts->norcvbuf;
	gs->glds_intr      = ts->intr;

	/* gs->glds_media_specific */
	gs->glds_dot3_first_coll     = ts->first_coll;
	gs->glds_dot3_multi_coll     = ts->multi_coll;
	gs->glds_dot3_sqe_error      = 0;
	gs->glds_dot3_mac_xmt_error  = 0;
	gs->glds_dot3_mac_rcv_error  = 0;
	gs->glds_dot3_frame_too_long = ts->rx_err_frame_too_long;

	gs->glds_speed  = 10000000;
	gs->glds_duplex = GLD_DUPLEX_HALF;
	gs->glds_media  =
		(lp->media == ADDRCONF_XCVR_10BASET) ? GLDM_10BT :
		(lp->media == ADDRCONF_XCVR_10BASE2 ) ? GLDM_BNC :
		(lp->media == ADDRCONF_XCVR_AUI) ? GLDM_AUI : GLDM_UNKNOWN;

	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}

static int
pctc_stop(gld_mac_info_t *macinfo)
{
	struct pctc_dev *lp;
	timeout_id_t	old_id;

	lp = (struct pctc_dev *)macinfo->gldm_private;

	/* stop timer */
	if (lp->timeout_id) {
		do {
			untimeout(old_id = lp->timeout_id);
		} while (lp->timeout_id != old_id);
		lp->timeout_id = 0;
	}

	/* stop RX and TX thread */
	mutex_enter(&lp->tc_intrlock);
	mutex_enter(&lp->tc_xmitlock);
	lp->started = FALSE;
	mutex_exit(&lp->tc_xmitlock);

	/* stop interrupts gracefully */
	while (lp->intr_busy) {
		cv_wait(&lp->drain_cv, &lp->tc_intrlock);
	}

	/* wait for tx drain */
	mutex_enter(&lp->tc_xmitlock);
	while (lp->tx_busy) {
		cv_wait(&lp->drain_cv, &lp->tc_xmitlock);
	}

	/* inhibit interrupt */
	EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | 0);

	/* no more statistics */
	EXEC_CMD(lp, OP_STATISTICS_DISABLE);

	/* stop receiver and transceiver */	
	EXEC_CMD(lp, OP_RX_DISABLE);
	EXEC_CMD(lp, OP_TX_DISABLE);

	rw_enter(&lp->tc_winlock, RW_WRITER);
	SET_WIN(lp, 4);

	/* stop jabber and link beat, LED disable */
	OUTW(lp, W4_MEDIA_TYPE_STATUS, MEDIA_LEDD);

	if (lp->media != ADDRCONF_XCVR_10BASET) {
		EXEC_CMD(lp, OP_STOP_COAXIAL_TRANSCEIVER);
		drv_usecwait(800);
	}
	
	SET_WIN(lp, 0);
	OUTW(lp, W0_RSRCCONF, RSRCCONF_CONST);
	OUTW(lp, W0_CONFCNTL, 0);

	SET_WIN(lp, 1);
	rw_exit(&lp->tc_winlock);
        
	lp->open  = FALSE;

	mutex_exit(&lp->tc_xmitlock);
	mutex_exit(&lp->tc_intrlock);

	return GLD_SUCCESS;
}


#ifdef DEBUG
static int	pctc_send_busy_cnt = 0;
#ifdef UNDERRUN_TEST
static int	pctc_underrun_test = 0;
#endif
#ifdef TIMEOUT_TEST
static int	pctc_timeout_test = 0;
#endif
#define	NFRAGS	10
static int pctc_pcont4;
static int pctc_pcont2;
static int pctc_pcont1;
static int pctc_frags[NFRAGS];
#endif /* DEBUG */
pctc_send(gld_mac_info_t *macinfo, mblk_t *mp)
{
	struct pctc_dev	*lp;
	size_t			pkt_len;
	size_t			len;
	size_t			len2;
	size_t			rest;
	mblk_t			*mp0;
	static uint32_t		pad = 0;
	int			tx_free;
	int			required;
	bool_t			restart_tx = FALSE;
#ifdef DEBUG
	static int		send_cnt = 0;
	int			frags;
#endif

	lp = (struct pctc_dev *)macinfo->gldm_private;

	/* Do we have enough room in fifo? */
	len2 = pkt_len = msgdsize(mp);

	DPRINTF(2, (CE_CONT, "?%s: pctc_send: "
		"mp:0x%p pkt_len:%d, started:%d tx_busy:%d tx_blocked:%d",
		lp->name, (void *)mp, pkt_len, lp->started,
		lp->tx_busy, lp->tx_blocked));

	mutex_enter(&lp->tc_xmitlock);

	if (!lp->started) {
		/*
		 * Drainning for device close or the card is not present
		 * Do not wait according to GLD driver specification.
		 */
		/* Do not free msg here, GLD does it */
		lp->tx_blocked = FIFO_STOPPED;
		mutex_exit(&lp->tc_xmitlock);
		return GLD_FAILURE;
	}

	if (lp->tx_busy) {
		/*
		 * Someone uses Tx FIFO port. Do not wait.
		 */
#ifdef DEBUG
		pctc_send_busy_cnt++;
#endif
		lp->tx_blocked = FIFO_BUSY;
		mutex_exit(&lp->tc_xmitlock);

		return GLD_NORESOURCES;
	}

#define	TX_FIFO_PAD	16	/* right ? */

	rw_enter(&lp->tc_winlock, RW_READER);
	tx_free = INW(lp, W1_FREE_XMIT_BYTES);
	rw_exit(&lp->tc_winlock);

	required = ROUNDUP((int)pkt_len, 4);
	tx_free -= required + TX_FIFO_PAD;
	if (tx_free < 0) {
		/*
		 * No enough space in Tx FIFO. Do not wait.
		 */
		lp->tx_blocked = FIFO_NOSPACE;
		mutex_exit(&lp->tc_xmitlock);

		/* Do not free msg here, GLD may reuse it */
		return GLD_NORESOURCES;
	}

	lp->tx_busy = TRUE;
	lp->tx_blocked = FIFO_NOTBLOCKED;
	lp->tx_start_time = ddi_get_lbolt();
#ifdef DEBUG
	send_cnt++;
#endif
	mutex_exit(&lp->tc_xmitlock);
#ifdef DEBUG	
#ifdef UNDERRUN_TEST
	if (pctc_underrun_test > 0) {
		if ((send_cnt % pctc_underrun_test) == 0) {
			/* cause tx underrun error */
			EXEC_CMD(lp, OP_SET_TX_START_THRESHOLD | 4);
		}
	}
#endif
#ifdef TIMEOUT_TEST
	if (pctc_timeout_test > 0) {
		if ((send_cnt % pctc_timeout_test) == 0) {
			/* cause tx timeout error */
			EXEC_CMD(lp, OP_TX_DISABLE);
		}
	}
#endif
#endif /* DEBUG */
	DPRINTF(3, (CE_CONT, "?%s: pctc_send: length:%d status:%b",
		lp->name, pkt_len, INW(lp, WX_STATUS), STATUS_BITS));
	rw_enter(&lp->tc_winlock, RW_READER);

	OUTL(lp, W1_TX_PIO_DATA, TX_FIFO_INT | pkt_len);

	pkt_len = required;
#ifdef DEBUG
	frags = 0;
#endif
	for (mp0 = mp; mp0; mp0 = mp0->b_cont) {
		uint8_t	*bp = mp0->b_rptr;
#ifdef DEBUG
		frags++;
#endif
		rest = mp0->b_wptr - mp0->b_rptr;
		pkt_len -= rest;

#if defined(DEBUG) || defined(sparc)
		if ((len = (long)bp & 3) > 0) {
			len = 4 - len;
			if (len <= rest)  {
				REPOUTSB(lp, W1_TX_PIO_DATA, bp, len);
				bp += len;
				rest -= len;
			}
		}
#endif
		if ((len = rest & ~3) > 0) {
			ASSERT(((long)bp & 3) == 0);
			REPOUTSD(lp, W1_TX_PIO_DATA, bp, len >> 2);
			bp += len;
			rest -= len;
		}

		if (rest) {
			REPOUTSB(lp, W1_TX_PIO_DATA, bp, rest);
		}
	}
#ifdef DEBUG
	pctc_frags[min(NFRAGS - 1, frags - 1)]++;
	if (mp->b_cont == NULL &&
	    ((long)mp->b_rptr & PAGEOFFSET) + len2 <= PAGESIZE) {
		if (((long)mp->b_rptr & 3) == 0) {
			pctc_pcont4++;
		}
		else if (((long)mp->b_rptr & 1) == 0) {
			pctc_pcont2++;
		}
		else {
			pctc_pcont1++;
		}
	}
#endif
	/* put pad till next double word boundary */
	ASSERT(pkt_len >= 0 && pkt_len < 4);
	if (pkt_len > 0) {
		REPOUTSB(lp, W1_TX_PIO_DATA, &pad, pkt_len);
	}

	rw_exit(&lp->tc_winlock);

	mutex_enter(&lp->tc_xmitlock);

	lp->tx_busy = FALSE;
	if (!lp->started) {
		/* someone waits for drainning */
		cv_broadcast(&lp->drain_cv);
	}
	else /* (lp->started is TRUE) */ {
		/* kick someone waiting for Tx fofo unused */
		restart_tx = (lp->tx_blocked == FIFO_BUSY);
	}

	mutex_exit(&lp->tc_xmitlock);

	/* free message block */
	freemsg(mp);

	if (restart_tx) {
		gld_sched(lp->macinfo);
	}

	return GLD_SUCCESS;
}

/* prooved */
static void
pctc_tx_timeout(void *data)
{
	struct pctc_dev *lp  = (struct pctc_dev *)data;
	clock_t		now;
	bool_t		restart_tx = FALSE;
	int		n;

	/* block tx and rx */
	mutex_enter(&lp->tc_intrlock);
	mutex_enter(&lp->tc_xmitlock);

	now = ddi_get_lbolt();
	if ((now - lp->tx_start_time > TX_TIMEOUT_TICKS) &&
	    (lp->tx_blocked == FIFO_BUSY || lp->tx_blocked == FIFO_NOSPACE ||
	     lp->tx_acked_time - lp->tx_start_time < 0)) {
   
		cmn_err(
#if 0
			CE_PANIC,
#else
			CE_WARN,
#endif
			"%s: Transmit timed out detected: %s",
			lp->name,
			lp->tx_blocked == FIFO_BUSY ? "tx-fifo busy" :
			lp->tx_blocked == FIFO_NOSPACE ? "tx-fifo no space" :
			lp->tx_acked_time - lp->tx_start_time < 0 ? "no ack" :
			"unknown reason");

		/* block interrupts */
		lp->started = FALSE;
		mutex_exit(&lp->tc_xmitlock);

		/* wait for interrupt service stop */
		n = 0;
		while (lp->intr_busy) {
			if (cv_timedwait(&lp->drain_cv,
			    &lp->tc_intrlock,
			    drv_usectohz(1000000) + ddi_get_lbolt()) == -1) {
				cmn_err(CE_WARN,
				    "%s: pctc_tx_timeout: intr drain timeout",
					lp->name);
			}
			if (n++ > 10) {
				mutex_exit(&lp->tc_intrlock);
				cmn_err(CE_PANIC,
					"%s: pctc_tx_timeout: tx intr timeout",
					lp->name);
				/* Not Reached */
			}
		}

		/* wait for Tx drain */
		mutex_enter(&lp->tc_xmitlock);
		n = 0;
		while (lp->tx_busy) {
			if (cv_timedwait(&lp->drain_cv,
			    &lp->tc_xmitlock,
			    drv_usectohz(1000000) + ddi_get_lbolt()) == -1) {
				cmn_err(CE_WARN,
					"%s: pctc_tx_timeout: tx drain timeout",
					lp->name);
			}
			if (n++ > 10) {
				mutex_exit(&lp->tc_xmitlock);
				mutex_exit(&lp->tc_intrlock);
				cmn_err(CE_PANIC,
					"%s: pctc_tx_timeout: tx drain timeout",
					lp->name);
				/* Not Reached */
			}
		}

		lp->stats.errxmt++;
		lp->tx_start_time = now;

		EXEC_CMD_WAIT(lp, OP_TX_RESET);
		EXEC_CMD(lp, OP_TX_ENABLE);

		EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | OUR_INTR_BITS);

		lp->started = TRUE;

		/* need to kick gld tx process */
		restart_tx = TRUE;
	}
	mutex_exit(&lp->tc_xmitlock);
	mutex_exit(&lp->tc_intrlock);

	if (restart_tx) {
		gld_sched(lp->macinfo);
	}

	lp->timeout_id = timeout(pctc_tx_timeout, data, TX_TIMEOUT_INTERVAL);

}

/*
 * Interrupt handlers
 */
static u_int
pctc_highintr(gld_mac_info_t *macinfo)
{
	struct pctc_dev	*lp;
	int			status;
	timeout_id_t		call_id;

	lp  = (struct pctc_dev *)macinfo->gldm_private;

	mutex_enter(&lp->hi_intrlock);

	/* Read interrupt status */
	status = INW(lp, WX_STATUS);

	if ((status & INTR_IL) == 0) {
		/*
		 * The device is not interrupting.
		 */
		mutex_exit(&lp->hi_intrlock);
		return DDI_INTR_UNCLAIMED;
	}

	if ((call_id = lp->soft_id) == 0) {
		/*
		 * The driver is not ready yet.
		 * Clear all interrupt bits.
		 */
		EXEC_CMD(lp, OP_ACK_INTERRUPT | 0xff);

		mutex_exit(&lp->hi_intrlock);
		return DDI_INTR_CLAIMED;
	}

	/*
	 * Clear all interrupt mask and ack interrupt,
	 * but do not clear any bits in intr status register.
	 */
	EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | 0);
	EXEC_CMD(lp, OP_ACK_INTERRUPT | INTR_IL);

	lp->softintr_req = 1;
	mutex_exit(&lp->hi_intrlock);

	/* Schedule lower interrupt handler */
	ddi_trigger_softintr(call_id);

	return DDI_INTR_CLAIMED;
}

#if DEBUG_LEVEL > 0
static int pctc_multi_intr_cnt = 0;
#endif

static u_int
pctc_intr(gld_mac_info_t *macinfo)
{
	struct pctc_dev *lp;
	int		serviced = DDI_INTR_UNCLAIMED;
	int		status;
	bool_t		restart_xmit;
	int		loop = 0;

	lp = (struct pctc_dev *) macinfo->gldm_private;
	status = 0;
	restart_xmit = FALSE;

	/*
	 * Check if we are interrupted ?
	 */
	mutex_enter(&lp->tc_intrlock);

	if (lp->intr_hilevel) {

		mutex_enter(&lp->hi_intrlock);
		if (lp->softintr_req == 0) {
			/*
			 * High level interrupt do not call us 
			 */
			mutex_exit(&lp->hi_intrlock);
			mutex_exit(&lp->tc_intrlock);
			return DDI_INTR_UNCLAIMED;
		}
		lp->softintr_req = 0;
		mutex_exit(&lp->hi_intrlock);

		/* pretend as if we are interrupted */
		status = INTR_IL;
	}

	/* Get current interrupt state */
	if (((status |= INW(lp, WX_STATUS)) & INTR_IL) == 0) {
		/*
		 * The chip does not rais any interrupt bits
		 */
		mutex_exit(&lp->tc_intrlock);

		return DDI_INTR_UNCLAIMED;
	}

	if (lp->intr_busy) {
		/* interrupt service is running */
#if DEBUG_LEVEL > 0
		pctc_multi_intr_cnt++;
#endif
		mutex_exit(&lp->tc_intrlock);
		cmn_err(CE_PANIC,
			"%s: pctc_intr: interrupt service running, status:%b",
			lp->name, status, STATUS_BITS);
		return DDI_INTR_UNCLAIMED;
	}

	/*
	 * Start interrupt service
	 */
	serviced = DDI_INTR_CLAIMED;
	lp->stats.intr++;
	lp->intr_busy = TRUE;

	DPRINTF(2, (CE_CONT, "?%s: pctc_intr: status:%b",
		lp->name, status, STATUS_BITS));

	if (!lp->started) {
		goto x;
	}

	if (lp->intr_hilevel == 0) {
		/* disable interrupts */
		EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | 0);
		EXEC_CMD(lp, OP_ACK_INTERRUPT | INTR_IL);
	}
again:
	if ((status & INTR_RE) != 0) {
		cmn_err(CE_WARN, "%s: unexpected RX_EARLY, status: %b",
			lp->name, status, STATUS_BITS);
		EXEC_CMD(lp, OP_ACK_INTERRUPT | INTR_RE);
	}

	if ((status & INTR_RC) != 0) {
		/* Receive Complete */
		pctc_receive(lp);
	}

	if ((status & INTR_TA) != 0) {
		cmn_err(CE_WARN, "%s: unexpected TX_AVAILABLE. status:%b",
			lp->name, status, STATUS_BITS);

		EXEC_CMD(lp, OP_ACK_INTERRUPT | INTR_TA);
	}

	if ((status & INTR_TC) != 0) {
		int	tx_status;

		DPRINTF(2, (CE_CONT, "?%s: INTR_TX_COMPLETE. status: %b",
				lp->name, status, STATUS_BITS));

		restart_xmit |= pctc_transmit_done(lp);
	}
	
	if ((status & INTR_US) != 0) {
		/* Update Statistics */
		pctc_read_stats(lp);
	}


	if ((status & INTR_AF) != 0) {
		int	fifo_diag;

		/* Process adapter failure interrupt */

		rw_enter(&lp->tc_winlock, RW_WRITER);
		SET_WIN(lp, 4);
		fifo_diag = INW(lp, W4_FIFO_DIAG);
		SET_WIN(lp, 1);
		rw_exit(&lp->tc_winlock);

		cmn_err(CE_WARN, "%s: ADAPTER_FAILURE, fifo_diag:%b",
				lp->name, fifo_diag, FIFO_DIAG_BITS);

		if ((fifo_diag & FIFO_DIAG_RXU) != 0) {
			EXEC_CMD(lp, OP_RX_RESET);
			pctc_set_rx_filter(lp);
			EXEC_CMD(lp, OP_RX_ENABLE);
			lp->stats.errrcv++;
		}

		if ((fifo_diag & FIFO_DIAG_TXO) != 0) {

			/* stop transmiting down-stream thread */
			mutex_enter(&lp->tc_xmitlock);
			lp->started = FALSE;
			while (lp->tx_busy) {
				cv_wait(&lp->drain_cv, &lp->tc_xmitlock);
			}

			/* reset transmitter */
			EXEC_CMD_WAIT(lp, OP_TX_RESET);

			/* enable transmitter again */
			EXEC_CMD(lp, OP_TX_ENABLE);

			restart_xmit = TRUE;
			lp->started = TRUE;
			mutex_exit(&lp->tc_xmitlock);
			lp->stats.errxmt++;
		}
		/* clear the interrupt */
		EXEC_CMD(lp, OP_ACK_INTERRUPT | INTR_AF);
	}

	if (restart_xmit) {
		/* restart write-side stream */
		mutex_exit(&lp->tc_intrlock);
		gld_sched(macinfo);
		mutex_enter(&lp->tc_intrlock);
	}

	/*
	 * Postamble for interrupt thread
	 */
x:
	if (lp->started) {
		/* enable interrupt again */
#ifdef notdef
		if ((status = INW(lp, WX_STATUS)) & OUR_INTR_BITS) {
			loop++;
			if (loop > 1) {
				cmn_err(CE_WARN,
			 "%s: pctc_intr: still interrupped (%d), stat:%b",
					lp->name, loop,
					status, STATUS_BITS);
			}
			if (loop > 10) {
				cmn_err(CE_PANIC,
				"%s: pctc_intr: maximun service loop exceeded",
					lp->name);
			}
			goto again;
		}
#endif
		EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | OUR_INTR_BITS);
	}
	else {
		EXEC_CMD(lp, OP_ACK_INTERRUPT | 0xfe);

		cv_broadcast(&lp->drain_cv);
	}
	lp->intr_busy = FALSE;

	mutex_exit(&lp->tc_intrlock);

	return serviced;
}

static void
pctc_read_stats(struct pctc_dev *lp)
{
	int		first_coll;
	int		multi_coll;
	volatile int	x;

	DPRINTF(2, (CE_CONT, "?%s: pctc_read_stats: called", lp->name));

	ASSERT(mutex_owned(&lp->tc_intrlock));

	rw_enter(&lp->tc_winlock, RW_WRITER);
	SET_WIN(lp, 6);
	EXEC_CMD(lp, OP_STATISTICS_DISABLE);

	lp->stats.nocarrier 	+= INB(lp, W6_CARRIER_SENSE_LOST); 
	/* heartbeat err */	x= INB(lp, W6_SQE);

	multi_coll		 = INB(lp, W6_MULTI_COLLISION);
	lp->stats.multi_coll	+= multi_coll;
	first_coll		 = INB(lp, W6_ONE_COLLISION);
	lp->stats.first_coll	+= first_coll;

	lp->stats.xmtlatecoll	+= INB(lp, W6_LATE_COLLISION);
	lp->stats.missed	+= INB(lp, W6_RX_OVERRUN);
	/* Tx packets   */	x= INB(lp, W6_TX_OK);
	/* Rx packets   */	x= INB(lp, W6_RX_OK);
	lp->stats.defer  	+= INB(lp, W6_TX_DEFER); 

	/* Total RX bytes */	x= INW(lp, W6_RX_TOTAL_BYTES);
	/* Total TX bytes */	x= INW(lp, W6_TX_TOTAL_BYTES);

	/*
	 * Guess total collisions
	 */
	lp->stats.collisions += first_coll + multi_coll*2;

	EXEC_CMD(lp, OP_STATISTICS_ENABLE);
	SET_WIN(lp,  1);
	rw_exit(&lp->tc_winlock);
}

/* Checked */
static int
pctc_receive(struct pctc_dev *lp)
{
	u_short		rxstat;
	size_t		length;
	mblk_t		*mp;
	size_t		xfer_len;

	ASSERT(mutex_owned(&lp->tc_intrlock)); /* for stats */

	rw_enter(&lp->tc_winlock, RW_READER);
	while (((rxstat = INW(lp, W1_RX_STATUS)) & RX_STATUS_IC) == 0) {

		DPRINTF(3, (CE_CONT, "?%s: pctc_receive: status %b, rxstat %b",
			lp->name, INW(lp, WX_STATUS), STATUS_BITS,
			rxstat, RX_STATUS_BITS));

		if ((rxstat & RX_STATUS_ER) != 0) {
			int errcode;

			/* RX error happened */
			lp->stats.errrcv++;

			errcode = (rxstat & RX_STATUS_CODE_MASK)
						>> RX_STATUS_CODE_SHIFT;
			lp->stats.rx_errs[errcode]++;

			EXEC_CMD_WAIT(lp, OP_RX_DISCARD_TOP_PKT);

			continue;
		}


		length  = rxstat & RX_STATUS_BYTES;
		ASSERT(length >= ETHERMIN && length <= ETHERMAX);
		xfer_len = ROUNDUP(length, 4);

		DPRINTF(3, (CE_CONT,
			"?%s: pctc_receive: packet size %d, status %b",
			lp->name, length, rxstat, RX_STATUS_BITS));

		if ((mp = allocb(xfer_len, BPRI_LO)) != NULL) {
			mp->b_wptr = mp->b_rptr + length;
			REPINSD(lp, W1_RX_PIO_DATA, mp->b_rptr, xfer_len >> 2);
		} else {
			/* receive buffer could not be allocated */
			lp->stats.norcvbuf++;
		}

		EXEC_CMD_WAIT(lp, OP_RX_DISCARD_TOP_PKT);

		if (mp != NULL) {
			rw_exit(&lp->tc_winlock);
			mutex_exit(&lp->tc_intrlock);

			gld_recv(lp->macinfo, mp);

			mutex_enter(&lp->tc_intrlock);
			rw_enter(&lp->tc_winlock, RW_READER);
		}
	}
	rw_exit(&lp->tc_winlock);

	return 0;
}

static int
pctc_transmit_done(struct pctc_dev  *lp)
{
	int	error = 0;
	int	restart_xmit;

	ASSERT(mutex_owned(&lp->tc_intrlock));

	rw_enter(&lp->tc_winlock, RW_READER);

	/* Clear the Tx status stack. */
	while ((INW(lp, WX_STATUS) & INTR_TC) != 0) {
		u_char tx_status;

		tx_status = INB(lp, W1_TX_STATUS);
		OUTB(lp, W1_TX_STATUS, 0);

		if ((tx_status & TX_STATUS_OF) != 0) {
			cmn_err(CE_WARN,
			    "%s: tx status overflow: tx_status: %b",
			    lp->name, tx_status, TX_STATUS_BITS);
		}

		/* update stats */
		if ((tx_status & TX_STATUS_UN) != 0) {
			/* fifo underflow */
			lp->stats.underflow++;
			lp->stats.errxmt++;
		}

		if ((tx_status & TX_STATUS_MC) != 0) {
			/* exceed maximum collisions */
			lp->stats.excoll++;
			lp->stats.collisions += 16;
			lp->stats.errxmt++;
		}

		if ((tx_status & TX_STATUS_JB) != 0) {
			lp->stats.errxmt++;
		}

		/* collect status bits */
		error |= (tx_status &
				 (TX_STATUS_JB | TX_STATUS_UN | TX_STATUS_MC));
	}
	rw_exit(&lp->tc_winlock);

	mutex_enter(&lp->tc_xmitlock);

	if (error != 0) {
		if ((error & (TX_STATUS_UN | TX_STATUS_JB)) != 0) {
			/* wait for tx fifo free */
			lp->started = FALSE;
			while (lp->tx_busy) {
				cv_wait(&lp->drain_cv,
					&lp->tc_xmitlock);
			}
			EXEC_CMD_WAIT(lp, OP_TX_RESET);
		}

		if ((error & TX_STATUS_MC) != 0) {
			EXEC_CMD(lp, OP_STATISTICS_ENABLE);
		}

		EXEC_CMD(lp, OP_TX_ENABLE); 
		lp->started = TRUE;
	}

	/*
	 * kick someone who waits for enough space in TX fifo
	 */
	restart_xmit = ((lp->tx_blocked == FIFO_STOPPED) || 
			(lp->tx_blocked == FIFO_NOSPACE));

	lp->tx_acked_time = ddi_get_lbolt();
	mutex_exit(&lp->tc_xmitlock);

	return restart_xmit;
}

/* ================================================================== */
/*
 * HW manupilation routines
 */
/* ================================================================== */
static int
pctc_read_eeprom(struct pctc_dev *lp, int offset)
{
	int	i = 0;

	/* window should be write-locked */
	ASSERT(rw_read_locked(&lp->tc_winlock) == 0);

	SET_WIN(lp, 0);
	OUTW(lp, W0_EEPROMCMD, EEPROM_READ | (offset & EEPROM_ADDR));

	drv_usecwait(162);

	while ((INW(lp, W0_EEPROMCMD) & EEPROM_EBY) != 0) {
		if (i++ > 20) {
			cmn_err(CE_WARN, "%s: pctc_read_eeprom: timeout",
				lp->name);
			return -1;
		}
		drv_usecwait(10);
	}
	return INW(lp, W0_EEPROMDATA);
}

static void
pctc_set_media(struct pctc_dev *lp)
{
	int	m = lp->media;

#ifdef notdef
	ASSERT(mutex_owned(&lp->tc_intrlock)); /* for command */
#endif
	/* window should be write-locked */
	ASSERT(rw_read_locked(&lp->tc_winlock) == 0);
	SET_WIN(lp, 0);

	OUTW(lp, W0_ADDRCONF, m << ADDRCONF_XCVR_SHIFT);

	SET_WIN(lp, 4);
	if (m == ADDRCONF_XCVR_10BASET) {
		/* enable link beat and jabber, enable LED */
		OUTW(lp, W4_MEDIA_TYPE_STATUS, MEDIA_JE | MEDIA_LK);
		EXEC_CMD(lp, OP_STOP_COAXIAL_TRANSCEIVER);
	} else {
		/* disable link beat and jabber, enable LED */
		OUTW(lp, W4_MEDIA_TYPE_STATUS, 0);
		EXEC_CMD(lp, OP_START_COAXIAL_TRANSCEIVER);
	}
	drv_usecwait(800);

	/* back to window 1 */
	SET_WIN(lp, 1);
}

static void
pctc_init_chip(struct pctc_dev *lp) 
{
	int	i;

	DPRINTF(2, (CE_CONT, "?%s: pctc_init_chip: called", lp->name));

	ASSERT(mutex_owned(&lp->tc_intrlock));
	ASSERT(mutex_owned(&lp->tc_xmitlock));
	ASSERT(rw_read_locked(&lp->tc_winlock) == 0);

	/* disable IRQ at PIC - do nothing */

	/* Enable adaptor */
	SET_WIN(lp, 0);
	OUTW(lp, W0_CONFCNTL, CONFCNTL_ENA);

	/* Reset transmitter */
	EXEC_CMD_WAIT(lp, OP_TX_RESET);

	/* Reset receiver */
	EXEC_CMD(lp, OP_RX_RESET);

	/* Set InterruptMask */
	EXEC_CMD(lp, OP_SET_INTERRUPT_MASK | OUR_INTR_BITS);

	/* Set ReadZeroMask (0xff) */
	EXEC_CMD(lp, OP_SET_READ_ZERO_MASK | 0xff);

	/* Set RxFilter */
	lp->rxmode = 0;	/* normal operation mode */
	pctc_set_rx_filter(lp);

	/* End of initialization */
	SET_WIN(lp, 1);
}

static void
pctc_start_chip(struct pctc_dev *lp) 
{
	ASSERT(mutex_owned(&lp->tc_intrlock));
	ASSERT(mutex_owned(&lp->tc_xmitlock));

	/* window shuould be write locked */
	ASSERT(rw_read_locked(&lp->tc_winlock) == 0);

	/* Get RxFree - do nothing */

	/* Get TxFree - do nothing */

	/* Enable statistics */
	pctc_read_stats(lp);
	bzero(&lp->stats, sizeof(lp->stats));
	EXEC_CMD(lp, OP_STATISTICS_ENABLE);

	/* Set media */
	pctc_set_media(lp);

	/* Enable transmitter */
	EXEC_CMD(lp, OP_SET_TX_START_THRESHOLD | INVALID_SIZE);
	EXEC_CMD(lp, OP_SET_TX_AVAIL_THRESHOLD | INVALID_SIZE);
	EXEC_CMD(lp, OP_TX_ENABLE);

	/* Enable receiver */
	EXEC_CMD(lp, OP_SET_RX_EARLY_THRESHOLD | INVALID_SIZE);
	EXEC_CMD(lp, OP_RX_ENABLE);

	/* Install adapter interrupt vector */
	SET_WIN(lp, 0);
	OUTW(lp, W0_RSRCCONF, 0x3f00);

	/* Enable IRQ at PIC - do nothing */

	/* back to window 1 */
	SET_WIN(lp, 1);
}


static void
pctc_set_rx_filter(struct pctc_dev *lp)
{
	int	operand;

	ASSERT(mutex_owned(&lp->tc_intrlock));

	/* Normal RX operation mode */
	operand = RX_INDIVIDUAL | RX_BROADCAST;

	if ((lp->rxmode & RXMODE_PROMISC) != 0) {
		/* Promiscous mode */
		operand |=  RX_MULTICAST | RX_ALL;
	} else
	if (lp->mc_count > 0 || (lp->rxmode & RXMODE_ALLMULTI) != 0) {
		/* Multicast mode */
		operand |=  RX_MULTICAST;
	}

	EXEC_CMD(lp, OP_SET_RX_FILTER | operand);
}

static void
pctc_exec_cmd_wait(struct pctc_dev *lp, int cmd)
{
	int	i = 0;
	int	err = FALSE;

	ASSERT(mutex_owned(&lp->tc_intrlock));

	if (lp->intr_hilevel) {
		mutex_enter(&lp->hi_intrlock);
	}

	EXEC_CMD(lp, cmd);

	while ((INW(lp, WX_STATUS) & STATUS_IP) != 0) {
		if (i++ > 10) {
			err = TRUE;
			break;
		}
		drv_usecwait(10);
	}

	if (lp->intr_hilevel) {
		mutex_exit(&lp->hi_intrlock);
	}

	if (err) {
		cmn_err(CE_WARN, "%s: cmd timeout error: cmd:0x%04",
		       lp->name, cmd);
	}
}
