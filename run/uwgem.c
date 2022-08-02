/*
 * uwgem.c: General USB to IEEE802.11 mac driver framework
 *
 * Copyright (c) 2010 Masayuki Murayama.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the author nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#pragma	ident	"%Z%%M% %I%     %E%"
/*
 * Thanks to:
 *   Kazuyoshi Sato
 *   Mr. Nomoto
 */

/*
 * Change log
 */

/*
 * TODO:
 *
 */

/*
 * System Header files.
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/vtrace.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/stream.h>		/* required by MBLK* */
#include <sys/strsun.h>		/* required by mionack() */
#include <sys/strsubr.h>	/* required by freemsgchain() */
#include <sys/byteorder.h>

/* protocol related headers */
#include <inet/common.h>
#include <inet/led.h>
#include <inet/mi.h>
#include <inet/nd.h>

#include <sys/mac_provider.h>
#ifdef TEST_ETHER
#include <sys/mac_ether.h>
#endif
#include <sys/mac_wifi.h>
#include <sys/net80211.h>
#include <sys/net80211_proto.h>

#include <sys/crypto/common.h>
#include <sys/crypto/api.h>
#include <inet/wifi_ioctl.h>

/* I/O bus depend headers */
#include <sys/usb/usba.h>
#if 0
#include <sys/usb/usba/usba_types.h>
#endif

/* supplement definitions */
extern const char *usb_str_cr(usb_cr_t);

#include <sys/note.h>

#include "uwgem.h"

#ifdef MODULE
char	ident[] = "usb general ethernet mac driver v" VERSION;
#else
extern char	ident[];
#endif

/* Debugging support */
#ifdef UWGEM_DEBUG_LEVEL
static int uwgem_debug = UWGEM_DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (uwgem_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	DEFAULT_PIPE(dp)	((dp)->reg_data->dev_default_ph)

/*
 * configuration parameters
 */
#define	USBDRV_MAJOR_VER	2
#define	USBDRV_MINOR_VER	0

#define	MAXPKTBUF(dp)		IEEE80211_MAX_LEN

#define	STOP_GRACEFUL	B_TRUE

/*
 * Private functions
 */
static int uwgem_open_pipes(struct uwgem_dev *dp);
static int uwgem_close_pipes(struct uwgem_dev *dp);
static void uwgem_intr_cb(usb_pipe_handle_t, usb_intr_req_t *);
static void uwgem_bulkin_cb(usb_pipe_handle_t, usb_bulk_req_t *);
static void uwgem_bulkout_cb(usb_pipe_handle_t, usb_bulk_req_t *);

static int uwgem_linkwacher_start(struct uwgem_dev *);
static void uwgem_linkwatcher_stop(struct uwgem_dev *);

/* local buffer management */
static int uwgem_init_rx_buf(struct uwgem_dev *);

/* internal mac interfaces */
static void uwgem_tx_timeout(struct uwgem_dev *);
static void uwgem_link_watcher(struct uwgem_dev *);
static int uwgem_mac_init(struct uwgem_dev *);
static int uwgem_mac_start(struct uwgem_dev *);
static int uwgem_mac_stop(struct uwgem_dev *, int);
static void uwgem_mac_ioctl(struct uwgem_dev *, queue_t *, mblk_t *);

static int uwgem_ctrl_retry = 5;
static int uwgem_bulk_retry = 5;

/* usb event support */
static int uwgem_disconnect_cb(dev_info_t *dip);
static int uwgem_reconnect_cb(dev_info_t *dip);
int uwgem_suspend(dev_info_t *dip);
int uwgem_resume(dev_info_t *dip);

static const uint8_t uwgem_bcastaddr[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

uint8_t wifi_bcastaddr[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

#ifdef MODULE
extern struct mod_ops mod_miscops;

static struct modlmisc modlmisc = {
	&mod_miscops,
	"uwgem v" VERSION,
};

static struct modlinkage modlinkage = {
	MODREV_1, &modlmisc, NULL
};

/*
 * _init : done
 */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, "!uwgem: _init: called"));
	status = mod_install(&modlinkage);

	return (status);
}

/*
 * _fini : done
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!uwgem: _fini: called"));
	status = mod_remove(&modlinkage);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
#endif /* MODULE */

/* ============================================================== */
/*
 * Ether CRC calculation utilities
 */
/* ============================================================== */
int
uwgem_prop_get_int(struct uwgem_dev *dp, char *prop_template, int def_val)
{
	char	propname[32];

	(void) sprintf(propname, prop_template, dp->name);

	return (ddi_prop_get_int(DDI_DEV_T_ANY, dp->dip,
	    DDI_PROP_DONTPASS, propname, def_val));
}

static int
uwgem_population(uint32_t x)
{
	int	i;
	int	cnt;

	cnt = 0;
	for (i = 0; i < 32; i++) {
		if (x & (1U << i)) {
			cnt++;
		}
	}
	return (cnt);
}

static clock_t
uwgem_timestamp_nz()
{
	clock_t	now;
	now = ddi_get_lbolt();
	return (now ? now : (clock_t)1);
}

/* =============================================================== */
/*
 * Debugging support
 */
/* =============================================================== */
static const char *uwgem_ieee80211_mgt_subtype_name[] = {
	"assoc_req",	"assoc_resp",	"reassoc_req",	"reassoc_resp",
	"probe_req",	"probe_resp",	"reserved#6",	"reserved#7",
	"beacon",	"atim",		"disassoc",	"auth",
	"deauth",	"reserved#13",	"reserved#14",	"reserved#15"
};

void
uwgem_dump_pkt(const char *func,
    const uint8_t *buf, int32_t len, int32_t rate, int32_t rssi)
{
#define MSGBUF_SIZE	0x1000
	struct ieee80211_frame *wh;
	int8_t *buf1;
	int8_t buf2[25];
	int i;

	buf1 = kmem_zalloc(MSGBUF_SIZE, KM_NOSLEEP);
	if (buf1 == NULL) {
		return;
	}

	bzero(buf2, sizeof (buf2));
	wh = (struct ieee80211_frame *)buf;

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
#if 1
	case IEEE80211_FC0_TYPE_MGT:
#if 0
		if (((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
		    >> IEEE80211_FC0_SUBTYPE_SHIFT) == 8) {
			/* beacon */
			goto x;
		}
		if (((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
		    >> IEEE80211_FC0_SUBTYPE_SHIFT) != 0xd) {
			/* beacon */
			goto x;
		}
#endif
		break;
#endif
#if 1
	case IEEE80211_FC0_TYPE_CTL:
		break;
#endif
#if 1
	case IEEE80211_FC0_TYPE_DATA:
		break;
#endif
	default:
		goto x;
	}

	switch (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) {
	case IEEE80211_FC1_DIR_NODS:
		(void) snprintf(buf2, sizeof (buf2), "NODS %s",
		    ieee80211_macaddr_sprintf(wh->i_addr2));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "->%s",
		    ieee80211_macaddr_sprintf(wh->i_addr1));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "(%s)",
		    ieee80211_macaddr_sprintf(wh->i_addr3));
		(void) strncat(buf1, buf2, sizeof (buf2));
		break;
	case IEEE80211_FC1_DIR_TODS:
		(void) snprintf(buf2, sizeof (buf2), "TODS %s",
		    ieee80211_macaddr_sprintf(wh->i_addr2));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "->%s",
		    ieee80211_macaddr_sprintf(wh->i_addr3));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "(%s)",
		    ieee80211_macaddr_sprintf(wh->i_addr1));
		(void) strncat(buf1, buf2, sizeof (buf2));
		break;

	case IEEE80211_FC1_DIR_FROMDS:
		(void) snprintf(buf2, sizeof (buf2), "FRDS %s",
		    ieee80211_macaddr_sprintf(wh->i_addr3));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "->%s",
		    ieee80211_macaddr_sprintf(wh->i_addr1));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "(%s)",
		    ieee80211_macaddr_sprintf(wh->i_addr2));
		(void) strncat(buf1, buf2, sizeof (buf2));
		break;

	case IEEE80211_FC1_DIR_DSTODS:
		(void) snprintf(buf2, sizeof (buf2), "DSDS %s",
		    ieee80211_macaddr_sprintf((uint8_t *)&wh[1]));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "->%s  ",
		    ieee80211_macaddr_sprintf(wh->i_addr3));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "%s",
		    ieee80211_macaddr_sprintf(wh->i_addr2));
		(void) strncat(buf1, buf2, sizeof (buf2));
		(void) snprintf(buf2, sizeof (buf2), "->%s",
		    ieee80211_macaddr_sprintf(wh->i_addr1));
		(void) strncat(buf1, buf2, sizeof (buf2));
		break;
	}
	cmn_err(CE_CONT, "!%s: len:%d %s", func, len, buf1);
	bzero(buf1, sizeof (buf1));

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_DATA:
		(void) sprintf(buf2, "data");
		break;

	case IEEE80211_FC0_TYPE_MGT:
		(void) snprintf(buf2, sizeof (buf2), "%s",
		    uwgem_ieee80211_mgt_subtype_name[
		    (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
		    >> IEEE80211_FC0_SUBTYPE_SHIFT]);
		break;

	default:
		(void) snprintf(buf2, sizeof (buf2), "type#%d",
		    wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK);
		break;
	}
	(void) strncat(buf1, buf2, sizeof (buf2));
	if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		(void) sprintf(buf2, " WEP");
		(void) strcat(buf1, buf2);
	}
	if (rate >= 0) {
		(void) snprintf(buf2,  sizeof (buf2), " %d.%dM", rate / 2, (rate & 1) ? 5 : 0);
		(void) strncat(buf1, buf2, sizeof (buf2));
	}
	if (rssi >= 0) {
		(void) snprintf(buf2,  sizeof (buf2), " +%d", rssi);
		(void) strncat(buf1, buf2, sizeof (buf2));
	}
	cmn_err(CE_CONT, "!%s: %s", func, buf1);
	bzero(buf1, sizeof (buf1));

	if (len > 0) {
		for (i = 0; i < (len > 80 ? 80 : len); i++) {
			if ((i & 0x03) == 0)
				(void) strcat(buf1, " ");
			(void) snprintf(buf2, 3, "%02x", buf[i]);
			(void) strncat(buf1, buf2, 3);
		}
		cmn_err(CE_CONT, "!%s: %s", func,  buf1);
	}
x:
	kmem_free(buf1, MSGBUF_SIZE);
}

void
uwgem_dump_ie(const uint8_t *iep)
{
	int	len;
	int	i;
	int8_t *buf1;
	int8_t buf2[25];

	if (iep == NULL) {
		cmn_err(CE_CONT, "%s: NULL", __func__);
		return;
	}

	buf1 = kmem_zalloc(MSGBUF_SIZE, KM_NOSLEEP);

	len = iep[1] + 2;

	if (len > 0) {
		for (i = 0; i < (len > 80 ? 80 : len); i++) {
			if ((i & 0x03) == 0)
				(void) strcat(buf1, " ");
			(void) snprintf(buf2, 3, "%02x", iep[i]);
			(void) strncat(buf1, buf2, 3);
		}
		cmn_err(CE_CONT, "!%s", buf1);
	}

	kmem_free(buf1, MSGBUF_SIZE);
}

#undef MSGBUF_SIZE
#ifdef UWGEM_GCC_RUNTIME
/*
 * gcc3 runtime routines
 */
#pragma weak memcmp
int
memcmp(const void *s1, const void *s2, size_t n)
{
	int	i;
	int	ret;

	ret = 0;
	for (i = 0; i < n; i++) {
		ret = (int)((uint8_t *)s1)[i] - (int)((uint8_t *)s2)[i];
		if (ret) {
			return (ret);
		}
	}
	return (0);
}

#pragma weak memset
void *
memset(void *s, int c, size_t n)
{
	if ((c & 0xff) == 0) {
		bzero(s, n);
	} else {
		while (n--) {
			((uint8_t *)s)[n] = c;
		}
	}
	return (s);
}

#pragma weak _memcpy = memcpy
#pragma weak memcpy
void *
memcpy(void *s1, const void *s2, size_t n)
{
	bcopy(s2, s1, n);
	return (s1);
}
#endif /* UWGEM_GCC_RUNTIME */
/* ============================================================== */
/*
 * hardware operations
 */
/* ============================================================== */
static int
uwgem_hal_reset_chip(struct uwgem_dev *dp, uint_t how)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_reset_chip)(dp, how);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_init_chip(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_init_chip)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_attach_chip(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_attach_chip)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_set_rx_filter(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_set_rx_filter)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_newstate(struct uwgem_dev *dp, enum ieee80211_state nstate, int arg)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_newstate)(dp, nstate, arg);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_start_chip(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_start_chip)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_stop_chip(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_stop_chip)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}

static int
uwgem_hal_get_stats(struct uwgem_dev *dp)
{
	int	err;

	sema_p(&dp->hal_op_lock);
	err = (*dp->ugc.uwgc_get_stats)(dp);
	sema_v(&dp->hal_op_lock);
	return (err);
}


/* ============================================================== */
/*
 * USB pipe management
 */
/* ============================================================== */
static boolean_t
uwgem_rx_start_unit(struct uwgem_dev *dp, usb_bulk_req_t *req)
{
	mblk_t	*mp;
	int	err;
	usb_flags_t	flags;

	ASSERT(req);

	mp = allocb(dp->rx_buf_len, BPRI_MED);
	if (mp == NULL) {
		cmn_err(CE_WARN, "!%s: %s: failed to allocate mblk",
		    dp->name, __func__);
		goto err;
	}

	req->bulk_len = dp->rx_buf_len;
	req->bulk_data = mp;
	req->bulk_client_private = (usb_opaque_t)dp;
	req->bulk_timeout = 0;
	req->bulk_attributes = USB_ATTRS_SHORT_XFER_OK;
	req->bulk_cb = uwgem_bulkin_cb;
	req->bulk_exc_cb = uwgem_bulkin_cb;
	req->bulk_completion_reason = 0;
	req->bulk_cb_flags = 0;

	flags = 0;
	err = usb_pipe_bulk_xfer(dp->bulkin_pipe[0], req, flags);

	if (err != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: failed to bulk_xfer for rx, err:%d",
		    dp->name, err);
		goto err;
	}
	return (B_TRUE);
err:
	return (B_FALSE);
}

/* ============================================================== */
/*
 * Rx/Tx buffer management
 */
/* ============================================================== */
static int
uwgem_init_rx_buf(struct uwgem_dev *dp)
{
	int	i;
	usb_bulk_req_t	*req;

	ASSERT(dp->mac_state == MAC_STATE_ONLINE);

	for (i = 0; i < dp->ugc.uwgc_rx_list_max; i++) {
		req = usb_alloc_bulk_req(dp->dip, 0, USB_FLAGS_SLEEP);
		if (req == NULL) {
			cmn_err(CE_WARN,
			    "!%s: %s: failed to allocate bulkreq for rx",
			    dp->name, __func__);
			return (USB_FAILURE);
		}
		if (!uwgem_rx_start_unit(dp, req)) {
			/* free req */
			usb_free_bulk_req(req);
			return (USB_FAILURE);
		}
		mutex_enter(&dp->rxlock);
		dp->rx_busy_cnt++;
		mutex_exit(&dp->rxlock);
	}
	return (USB_SUCCESS);
}

/* ============================================================== */
/*
 * memory resource management
 */
/* ============================================================== */
static int
uwgem_free_memory(struct uwgem_dev *dp)
{
	usb_bulk_req_t	*req;

	/* free requst structure */
	while (req = dp->tx_free_list) {
		dp->tx_free_list =
		    (usb_bulk_req_t *)req->bulk_client_private;
		req->bulk_data = NULL;
		usb_free_bulk_req(req);
	}
	return (USB_SUCCESS);
}

static int
uwgem_alloc_memory(struct uwgem_dev *dp)
{
	int	i;
	usb_bulk_req_t	*req;

	/* allocate tx requests */
	dp->tx_free_list = NULL;
	for (i = 0; i < dp->ugc.uwgc_tx_list_max; i++) {
		req = usb_alloc_bulk_req(dp->dip, 0, USB_FLAGS_SLEEP);
		if (req == NULL) {
			cmn_err(CE_WARN,
			    "!%s:%s failed to allocate tx requests",
			    dp->name, __func__);

			/* free partially allocated tx requests */
			(void) uwgem_free_memory(dp);
			return (USB_FAILURE);
		}

		/* add the new one allocated into tx free list */
		req->bulk_client_private = (usb_opaque_t)dp->tx_free_list;
		dp->tx_free_list = req;
	}

	return (USB_SUCCESS);
}

/* ========================================================== */
/*
 * Start transmission.
 * Return zero on success,
 */
/* ========================================================== */

#ifdef TXTIMEOUT_TEST
static int uwgem_send_cnt = 0;
#endif

/*
 * uwgem_send is used only to send data packet into ethernet line.
 */
#define	FLAG_INTR	0x100
#define	FLAG_TYPE	0x0ff
static mblk_t *
uwgem_send_common(struct uwgem_dev *dp, mblk_t *mp, uint32_t flags)
{
	int		err;
	mblk_t		*new;
	usb_bulk_req_t	*req;
	int		mcast;
	int		bcast;
	int		len;
	int		qid;
	boolean_t	intr;
	struct ieee80211_frame	*wh;
	struct ieee80211_node	*ni = NULL;
	uint8_t		type;
	usb_flags_t	usb_flags = 0;
#ifdef UWGEM_DEBUG_LEVEL
	usb_pipe_state_t	p_state;
#endif
	DPRINTF(2, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	wh = (struct ieee80211_frame *)mp->b_rptr;
	ni = ieee80211_find_txnode(&dp->ic, wh->i_addr1);
	if (ni == NULL) {
#ifdef notyet
		mutex_enter(&sc->sc_txlock);
		sc->sc_tx_err++;
		mutex_exit(&sc->sc_txlock);
#endif
		ASSERT(mp->b_cont == NULL);
		freemsg(mp);
		return (NULL);
	}

	intr = (flags & FLAG_INTR) != 0;
	type = flags & FLAG_TYPE;
	len = msgdsize(mp);

	bcast = 0;
	mcast = 0;
	if (wh->i_addr1[0] & 1) {
		if (bcmp(mp->b_rptr, &uwgem_bcastaddr,
		    IEEE80211_ADDR_LEN) == 0) {
			bcast = 1;
		} else {
			mcast = 1;
		}
	}

	new = (*dp->ugc.uwgc_tx_make_packet)(dp, mp, type, ni);
	if (new == NULL) {
		/*
		 * no memory resource. we don't stop downstream,
		 * we just discard the packet.
		 */
		DPRINTF(0, (CE_CONT, "!%s: %s: no memory",
		    dp->name, __func__));
		ASSERT(mp->b_cont == NULL);
		freemsg(mp);
		mp = NULL;

		mutex_enter(&dp->txlock);
		dp->stats.noxmtbuf++;
		dp->stats.errxmt++;
		mutex_exit(&dp->txlock);
		goto fail;
	}
	qid = new->b_datap->db_cksum16;

	ASSERT(new->b_cont == NULL);

	mutex_enter(&dp->txlock);
	if (dp->tx_free_list == NULL) {
		/*
		 * no tx free slot
		 */
		ASSERT(mp != NULL);
		ASSERT(dp->tx_busy_cnt == dp->ugc.uwgc_tx_list_max);
		mutex_exit(&dp->txlock);

		DPRINTF(4, (CE_CONT, "!%s: %s: no free slot",
		    dp->name, __func__));
		if (new && new != mp) {
			/* free reallocated message */
			freemsg(new);
		}
		goto fail;
	}
	req = dp->tx_free_list;
	dp->tx_free_list = (usb_bulk_req_t *)req->bulk_client_private;
	dp->tx_busy_cnt++;

	if (dp->tx_free_list == NULL) {
		intr = B_TRUE;
	}
	if (intr) {
		dp->tx_intr_pended++;
	}
	DB_TCI(new) = intr;
#ifdef UWGEM_DEBUG_LEVEL
	new->b_datap->db_cksum32 = dp->tx_seq_num;
	dp->tx_seq_num++;
#endif
	dp->stats.obytes += len;
	dp->stats.opackets++;
	if (bcast | mcast) {
		dp->stats.obcast += bcast;
		dp->stats.omcast += mcast;
	}
	mutex_exit(&dp->txlock);

	DPRINTF(2, (CE_CONT, "!%s: %s: sending", dp->name, __func__));

	req->bulk_len = (long)new->b_wptr - (long)new->b_rptr;
	req->bulk_data = new;
	req->bulk_client_private = (usb_opaque_t)dp;
	req->bulk_timeout = dp->bulkout_timeout;	/* in second */
	req->bulk_attributes = 0;
	req->bulk_cb = uwgem_bulkout_cb;
	req->bulk_exc_cb = uwgem_bulkout_cb;
	req->bulk_completion_reason = 0;
	req->bulk_cb_flags = 0;

	if (intr) {
		usb_flags = USB_FLAGS_SLEEP;
	}

	if ((err = usb_pipe_bulk_xfer(dp->bulkout_pipe[qid], req, usb_flags))
	    != USB_SUCCESS) {

		/* failed to transfer the packet, discard it. */
		freemsg(new);
#ifdef SANITY
		req->bulk_data = NULL;
#endif
		/* recycle the request block */
		mutex_enter(&dp->txlock);
		dp->tx_busy_cnt--;
		req->bulk_client_private = (usb_opaque_t)dp->tx_free_list;
		dp->tx_free_list = req;
		mutex_exit(&dp->txlock);
		cv_broadcast(&dp->tx_drain_cv);

		cmn_err(CE_WARN,
		    "!%s: %s: usb_pipe_bulk_xfer: failed: err:%d",
		    dp->name, __func__, err);

		/* we use another flag to indicate error state. */
		if (dp->fatal_error == (clock_t)0) {
			dp->fatal_error = uwgem_timestamp_nz();
			cv_signal(&dp->watchdog_cv);
		}
	} else {
		/* record the start time */
		dp->tx_start_time = ddi_get_lbolt();
	}

	if (new != mp) {
		freemsg(mp);
	}
	mp = NULL;

	if (err == USB_SUCCESS && (usb_flags & USB_FLAGS_SLEEP) != 0) {
		uwgem_bulkout_cb(dp->bulkout_pipe[qid], req);
	}
fail:
	if (ni) {
		ieee80211_free_node(ni);
	}
	return (mp);
}

int
uwgem_mgt_send(ieee80211com_t *ic, mblk_t *mp, uint8_t type)
{
	struct uwgem_dev *dp = (void *)ic;

#ifdef NEW_LOCK
	/*
	 * XXX - we use RW_READ, instead of RW_WRITER. Because this
	 * routine will be called to send addba_req while data packets
	 * are sent.
	 */
	rw_enter(&dp->dev_state_lock, RW_READER);
#endif
#if UWGEM_DEBUG_LEVEL >= 10
	uwgem_dump_pkt(__func__, mp->b_rptr, mp->b_wptr - mp->b_rptr, 0, 0);
#endif
	mp->b_next = NULL;
	if (dp->mac_state == MAC_STATE_ONLINE) {
		mp = uwgem_send_common(dp, mp, type);
	}
#ifdef NEW_LOCK
	rw_exit(&dp->dev_state_lock);
#endif
	if (mp) {
		/* XXX - can we return error ? */
		freemsg(mp);
	}
	return (DDI_SUCCESS);
}

static void
uwgem_recv_mgmt(ieee80211com_t *ic, mblk_t *mp, struct ieee80211_node *in,
    int subtype, int rssi, uint32_t rstamp)
{
	struct uwgem_dev *dp = (void *)ic;

	/*
	 * we cannot use dupmsg() because pointers in mblt_t are used
	 * to store arguments of recv_mgmt().
	 */
	mp = copyb(mp);
	if (mp == NULL) {
		return;
	}

	mp->b_datap->db_credp = (void *)in;
	mp->b_datap->db_cksum32 = subtype;
	mp->b_datap->db_cksumstart = rssi;
	mp->b_datap->db_cksumend = rstamp;

	mutex_enter(&dp->watchdog_lock);
	if (dp->mgtq_head) {
		dp->mgtq_tail->b_next = mp;
	} else {
		dp->mgtq_head = mp;
	}
	dp->mgtq_tail = mp;
	mp->b_next = NULL;
	mutex_exit(&dp->watchdog_lock);
	cv_signal(&dp->watchdog_cv);
}

static int
uwgem_restart_nic(struct uwgem_dev *dp, uint_t reset_type)
{
	int	ret;
	int	flags = 0;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	ASSERT(dp->mac_state != MAC_STATE_DISCONNECTED);

	/*
	 * ensure to stop the nic
	 */
	if (dp->mac_state > MAC_STATE_STOPPED) {
		(void) uwgem_mac_stop(dp, MAC_STATE_STOPPED);
	}

	/* now the nic get quiescent, reset the chip */
	if (uwgem_hal_reset_chip(dp, reset_type) != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: failed to reset chip",
		    dp->name, __func__);
		goto err;
	}
	dp->mac_state = MAC_STATE_STOPPED;

	/*
	 * restore the nic state step by step
	 */
	if (dp->nic_state >= NIC_STATE_INITIALIZED) {
		if (uwgem_mac_init(dp) != USB_SUCCESS) {
			cmn_err(CE_WARN, "!%s: %s: failed to initialize chip",
			    dp->name, __func__);
			goto err;
		}

		/* setup mac address and enable rx filter */
		sema_p(&dp->rxfilter_lock);
		dp->rxmode |= RXMODE_ENABLE;
		ret = uwgem_hal_set_rx_filter(dp);
		sema_v(&dp->rxfilter_lock);
		if (ret != USB_SUCCESS) {
			goto err;
		}
	}

	if (dp->nic_state >= NIC_STATE_ONLINE) {
		(void) uwgem_mac_start(dp);

		/* just restore ic_state */
		(void) uwgem_hal_newstate(dp, dp->ic.ic_state, -1);

		/* restart downstream */
		mac_tx_update(dp->ic.ic_mach);
	}

	/* update the link state asynchronously */
	cv_signal(&dp->lw_watcher_wait_cv);

	return (USB_SUCCESS);
err:
#ifdef UWGEM_CONFIG_FMA
	ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
	return (USB_FAILURE);
}

static void
uwgem_watchdog(struct uwgem_dev *dp)
{
	int	ret;
	uint_t	how;
	clock_t	intvl;
	mblk_t	*mp;

	intvl = dp->watchdog_interval;
	for (; ; ) {
		mutex_enter(&dp->watchdog_lock);
		ret = cv_timedwait(&dp->watchdog_cv, &dp->watchdog_lock,
		    intvl + ddi_get_lbolt());
		how = RW_READER;
		mutex_exit(&dp->watchdog_lock);

		/* restore inteval to default for the next turn */
		intvl = dp->watchdog_interval;

		if (dp->watchdog_stop) {
			break;
		}

again:
		rw_enter(&dp->dev_state_lock, how);

		if (dp->mac_state == MAC_STATE_DISCONNECTED) {
#ifdef NEW_LOCK
			rw_exit(&dp->dev_state_lock);
#endif
			goto skip;
		}

		if (dp->nic_state == NIC_STATE_ONLINE &&
		    dp->fatal_error) {
			if (how == RW_READER) {
				/*
				 * need to upgrade dev_state_lock from
				 * shared mode to exclusive mode before
				 * we restart nic.
				 */
				rw_exit(&dp->dev_state_lock);
				how = RW_WRITER;
				goto again;
			}
			cmn_err(CE_WARN, "!%s: %s: restarting the nic:"
			    " fatal_error:%p nic_state:%d mac_state:%d",
			    dp->name, __func__,
			    dp->fatal_error, dp->nic_state, dp->mac_state);

			(void) uwgem_restart_nic(dp, 0);
		}
#ifdef NEW_LOCK
		rw_exit(&dp->dev_state_lock);
#endif
		mutex_enter(&dp->watchdog_lock);
		while (mp = dp->mgtq_head) {
			struct ieee80211_node *in;
			int	subtype;
			int	rssi;
			uint32_t	rstamp;

			/* take one from management request queue */
			dp->mgtq_head = mp->b_next;
			mp->b_next = NULL;
			if (dp->mgtq_head == NULL) {
				dp->mgtq_tail = NULL;
			}
			mutex_exit(&dp->watchdog_lock);

			in = (void *)mp->b_datap->db_credp;
			subtype = mp->b_datap->db_cksum32;
			rssi = mp->b_datap->db_cksumstart;
			rstamp = mp->b_datap->db_cksumend;

			mp->b_datap->db_credp = 0;
			mp->b_datap->db_cksum32 = 0;
			mp->b_datap->db_cksumstart = 0;
			mp->b_datap->db_cksumend = 0;

			(*dp->ic_recv_mgmt)(&dp->ic, mp,
			    in, subtype, rssi, rstamp);
			freemsg(mp);
			mutex_enter(&dp->watchdog_lock);
		}
		mutex_exit(&dp->watchdog_lock);
skip:
#ifdef NEW_LOCK
		;
#else
		rw_exit(&dp->dev_state_lock);
#endif
	}
	thread_exit();
}


static int
uwgem_watchdog_start(struct uwgem_dev *dp)
{
	int	err;
	kthread_t	*wdth;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* make a first call of uwgem_lw_link_check() */
	dp->watchdog_stop = 0;
	dp->watchdog_interval = drv_usectohz(1000*1000);

	wdth = thread_create(NULL, 0, uwgem_watchdog, dp, 0, &p0,
	    TS_RUN, minclsyspri);
	if (wdth == NULL) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to create a watchdog thread",
		    dp->name, __func__);
		return (USB_FAILURE);
	}
	dp->watchdog_did = wdth->t_did;

	return (USB_SUCCESS);
}

static void
uwgem_watchdog_stop(struct uwgem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));
	if (dp->watchdog_did) {
		/* Ensure timer routine stopped */
		dp->watchdog_stop = 1;
		cv_signal(&dp->watchdog_cv);
		thread_join(dp->watchdog_did);
		dp->watchdog_did = 0;
	}
}

/* ================================================================== */
/*
 * solaris usba callback handlers
 */
/* ================================================================== */
static void
uwgem_bulkin_cb(usb_pipe_handle_t pipe, usb_bulk_req_t *req)
{
	mblk_t	*newmp;
	mblk_t	*mp;
	mblk_t	*tp;
	int	len = 0;
	int	pkts = 0;
	int	bcast = 0;
	int	mcast = 0;
	boolean_t	prepare_next;
	struct uwgem_dev	*dp;
	struct ieee80211com	*ic;
	struct ieee80211_node	*ni;

	dp = (struct uwgem_dev *)req->bulk_client_private;
	ic = &dp->ic;
	mp = req->bulk_data;
	req->bulk_data = NULL;

	DPRINTF(2, (CE_CONT, "!%s: %s: mp:%p, len:%d, cr:%s(%d)",
	    dp->name, __func__, mp, mp->b_wptr - mp->b_rptr,
	    usb_str_cr(req->bulk_completion_reason),
	    req->bulk_completion_reason));

	/*
	 * We cannot acquire dev_state_lock here, because the routine
	 * should not block uwgem_mac_stop() to avoid dead lock.
	 * So we use a simple membar operation to get the state correctly.
	 */
	membar_consumer();

	if (req->bulk_completion_reason == USB_CR_OK &&
	    dp->nic_state == NIC_STATE_ONLINE) {
		mp->b_next = NULL;
		newmp = (*dp->ugc.uwgc_rx_make_packet)(dp, mp);

		if (newmp != mp) {
			/* the message has been reallocated, free old one */
			while (tp = mp) {
				mp = mp->b_next;
				tp->b_cont = NULL;
				tp->b_prev = NULL;
				tp->b_next = NULL;
				freemsg(tp);
			}
		}

		/* received message may include one or more ethernet packets */
		mp = newmp;
		while (mp) {
			len += mp->b_wptr - mp->b_rptr;
			pkts++;

			if (((struct ieee80211_frame *)
			    mp->b_rptr)->i_addr1[0] & 1) {
				if (bcmp(mp->b_rptr, &uwgem_bcastaddr,
				    IEEE80211_ADDR_LEN) == 0) {
					bcast++;
				} else {
					mcast++;
				}
			}

			tp = mp->b_next;
			mp->b_next = NULL;

			/* grab a reference to the source node */
			ni = ieee80211_find_rxnode(ic,
			    (struct ieee80211_frame *)mp->b_rptr);
			if (ni) {
				uint32_t	rate;
				uint32_t	rssi;

				rssi = (uintptr_t)mp->b_cont;
				rate = (uintptr_t)mp->b_prev;
				mp->b_cont = NULL;
				mp->b_prev = NULL;
#if UWGEM_DEBUG_LEVEL > 100
				uwgem_dump_pkt(__func__, mp->b_rptr,
				    mp->b_wptr - mp->b_rptr, rate, rssi);
#endif
				/* send up if it is a valid packet */
				ieee80211_input(ic, mp, ni, rssi, rate);

				/* node is no longer needed */
				ieee80211_free_node(ni);
			} else {
				cmn_err(CE_CONT, "!%s: %s: ni not found",
				    dp->name, __func__);
				freemsg(mp);
			}

			mp = tp;
		}
	} else {
		freemsg(mp);
		len = 0;
	}

	mutex_enter(&dp->rxlock);
	dp->stats.rbytes += len;
	dp->stats.rpackets += pkts;
	if (bcast | mcast) {
		dp->stats.rbcast += bcast;
		dp->stats.rmcast += mcast;
	}
	mutex_exit(&dp->rxlock);

	if (dp->mac_state != MAC_STATE_ONLINE || !dp->rx_active) {
		/* no need to prepare the next packets */
		dp->rx_active = B_FALSE;
		goto stop_rx;
	}

	/* prepare to receive the next packets */
	if (!uwgem_rx_start_unit(dp, req)) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to fill next rx packet",
		    dp->name, __func__);

		/* stop preparing rx packets */
		dp->rx_active = B_FALSE;

		/*
		 * we use another flag to indicate error state.
		 * if we acquire dev_state_lock for RW_WRITER here,
		 * uwgem_mac_stop() may hang.
		 */
		if (dp->fatal_error == (clock_t)0) {
			dp->fatal_error = uwgem_timestamp_nz();
			cv_signal(&dp->watchdog_cv);
		}
		goto stop_rx;
	}
	return;

stop_rx:
	usb_free_bulk_req(req);

	mutex_enter(&dp->rxlock);
	dp->rx_busy_cnt--;
	if (dp->rx_busy_cnt == 0) {
		/* wake up someone who waits for me */
		cv_broadcast(&dp->rx_drain_cv);
	}
	mutex_exit(&dp->rxlock);
}

static void
uwgem_bulkout_cb(usb_pipe_handle_t pipe, usb_bulk_req_t *req)
{
	boolean_t	intr;
	boolean_t	tx_sched;
	struct uwgem_dev	*dp;

	dp = (struct uwgem_dev *)req->bulk_client_private;
	tx_sched = B_FALSE;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: cr:%s(%d) cb_flags:0x%x busycnt:%d",
	    dp->name, __func__,
	    usb_str_cr(req->bulk_completion_reason),
	    req->bulk_completion_reason,
	    req->bulk_cb_flags,
	    dp->tx_busy_cnt));

	/* we have finished to transfer the packet into tx fifo */
	intr = DB_TCI(req->bulk_data);
	freemsg(req->bulk_data);
#ifdef SANITY
	req->bulk_data = NULL;
#endif
	if (req->bulk_completion_reason != USB_CR_OK) {
		cmn_err(CE_WARN,
		    "!%s: %s: error: cr:%s(%d) cb_flags:0x%x busycnt:%d",
		    dp->name, __func__,
		    usb_str_cr(req->bulk_completion_reason),
		    req->bulk_completion_reason,
		    req->bulk_cb_flags,
		    dp->tx_busy_cnt);
		if (dp->fatal_error == (clock_t)0) {
			dp->fatal_error = uwgem_timestamp_nz();
			cv_signal(&dp->watchdog_cv);
		}
	}

	mutex_enter(&dp->txlock);

	if (intr) {
		ASSERT(dp->tx_intr_pended > 0);
		/* find the last interrupt we have scheduled */
		if (--(dp->tx_intr_pended) == 0) {
			tx_sched = B_TRUE;
		}
	}

	ASSERT(dp->tx_busy_cnt > 0);
	req->bulk_client_private = (usb_opaque_t)dp->tx_free_list;
	dp->tx_free_list = req;
	dp->tx_busy_cnt--;

#ifdef CONFIG_TX_LIMITER
	if (tx_sched) {
		dp->tx_max_packets =
		    min(dp->tx_max_packets + 1, dp->ugc.uwgc_tx_list_max);
	}
#endif
	if (dp->mac_state != MAC_STATE_ONLINE && dp->tx_busy_cnt == 0) {
		cv_broadcast(&dp->tx_drain_cv);
	}

	mutex_exit(&dp->txlock);

	if (tx_sched) {
		mac_tx_update(dp->ic.ic_mach);
	}
}

static void
uwgem_intr_cb(usb_pipe_handle_t ph, usb_intr_req_t *req)
{
	struct uwgem_dev	*dp;

	dp = (struct uwgem_dev *)req->intr_client_private;
	dp->stats.intr++;

	if (req->intr_completion_reason == USB_CR_OK) {
		(*dp->ugc.uwgc_interrupt)(dp, req->intr_data);
	}

	/* free the request and data */
	usb_free_intr_req(req);
}

/* ======================================================================== */
/*
 * link management support routines
 */
/* ======================================================================== */
static int
uwgem_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	int	ret;
	struct uwgem_dev *dp;
	enum ieee80211_state ostate;
	boolean_t	restore_lock;
	struct ieee80211com *bss_ic;

	dp = (struct uwgem_dev *)ic;
	ostate = dp->ic.ic_state;
#ifdef NEW_LOCK
	/* EMPTY */
#else
	if (restore_lock =
	    (nstate != ostate && rw_read_locked(&dp->dev_state_lock))) {
		rw_exit(&dp->dev_state_lock);
		rw_enter(&dp->dev_state_lock, RW_WRITER);
	}
#endif
#if UWGEM_DEBUG_LEVEL > 10
	if (ic->ic_bss != NULL) {
		cmn_err(CE_CONT, "!%s: ic:%x ic_bss:%x in_ic:%x",
		    dp->name, ic, ic->ic_bss, ic->ic_bss->in_ic);
	} else {
		cmn_err(CE_CONT, "!%s: ic:%x ic_bss:%x",
		    dp->name, ic, ic->ic_bss);
	}
#endif
#ifdef NEW_LOCK
	rw_enter(&dp->dev_state_lock,
	    nstate != ostate ? RW_WRITER : RW_WRITER);
#endif
	/*
	 * XXX - as ieee80211_new_state() will be called again recursively
	 * for transitting to SCAN, we must call uwgem_hal_newstate()
	 * before calling dp->ic_newstate().
	 * Otherwise register operations will be reversed.
	 */
	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		(void) uwgem_hal_newstate(dp, nstate, arg);
	}
#ifdef NEW_LOCK
	rw_exit(&dp->dev_state_lock);
#endif

	ret = (*dp->ic_newstate)(ic, nstate, arg);

#ifdef NEW_LOCKx	/* this calles recursive lock */
	rw_exit(&dp->dev_state_lock);
#endif

	if (nstate == IEEE80211_S_SCAN && ostate != IEEE80211_S_SCAN) {
		/* start scanning access points */
		dp->lw_scan_count = 0;
		dp->lw_scan_start = ddi_get_lbolt();
		cv_signal(&dp->lw_watcher_wait_cv);
	}

#ifdef NEW_LOCK
	/* EMPTY */
#else
	if (restore_lock) {
		rw_downgrade(&dp->dev_state_lock);
	}
#endif
	return (ret);
}

extern void ieee80211_send_nulldata(ieee80211_node_t *);

static void
uwgem_lw_link_watcher(struct uwgem_dev *dp)
{
	struct ieee80211com *ic = &dp->ic;
	int	null_count = 0;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	for (; ; ) {
		mutex_enter(&dp->lw_lock);
		if (dp->lw_interval) {
			(void) cv_timedwait(&dp->lw_watcher_wait_cv,
			    &dp->lw_lock,
			    dp->lw_interval + ddi_get_lbolt());
		} else {
			cv_wait(&dp->lw_watcher_wait_cv,
			    &dp->lw_lock);
		}

		if (dp->lw_watcher_state == -1) {
			mutex_exit(&dp->lw_lock);
			break;
		}
		mutex_exit(&dp->lw_lock);

		dp->lw_interval = drv_usectohz(1000*1000);

		rw_enter(&dp->dev_state_lock, RW_READER);

		if (dp->mac_state != MAC_STATE_ONLINE) {
			/*
			 * different from ethernet devices, wifi nic
			 * must be *working* to make connections.
			 */
#ifdef NEW_LOCK
			rw_exit(&dp->dev_state_lock);
#endif
			goto next;
		}
#ifdef NEW_LOCK
		rw_exit(&dp->dev_state_lock);
#endif

		switch (ic->ic_state) {
		case IEEE80211_S_SCAN:
			DPRINTF(3, (CE_CONT,
			    "!%s: %s: nextscan", dp->name, __func__));
			if (dp->lw_scan_count++ > 0) {
				if (ddi_get_lbolt() - dp->lw_scan_start
				    < LW_SCAN_TIMEOUT) {
					ieee80211_next_scan(&dp->ic);
				}
			}
			dp->lw_interval = drv_usectohz(200*1000);
			break;

		case IEEE80211_S_RUN:
			(void) uwgem_hal_get_stats(dp);
			if (null_count++ > 30*10) {
				ieee80211_send_nulldata(dp->ic.ic_bss);
				null_count = 0;
			}

			dp->lw_interval = drv_usectohz(100*1000);
			break;
		}
next:
#ifdef NEW_LOCK
		;
#else
		rw_exit(&dp->dev_state_lock);
#endif
	}
	thread_exit();
}

static int
uwgem_lw_start(struct uwgem_dev *dp)
{
	int	err;
	kthread_t	*lwth;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* make a first call of uwgem_lw_link_check() */
	dp->lw_watcher_state = 0;
	dp->lw_interval = drv_usectohz(1000*1000); /* 1sec */
	dp->lw_last_check = ddi_get_lbolt();

	lwth = thread_create(NULL, 0, uwgem_lw_link_watcher, dp, 0, &p0,
	    TS_RUN, minclsyspri);
	if (lwth == NULL) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to create a link watcher thread",
		    dp->name, __func__);
		return (USB_FAILURE);
	}
	dp->lw_watcher_did = lwth->t_did;

	return (USB_SUCCESS);
}

static void
uwgem_lw_stop(struct uwgem_dev *dp)
{
	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (dp->lw_watcher_did) {
		/* Ensure timer routine stopped */
		mutex_enter(&dp->lw_lock);
		dp->lw_watcher_state = -1;
		cv_signal(&dp->lw_watcher_wait_cv);
		mutex_exit(&dp->lw_lock);
		thread_join(dp->lw_watcher_did);
		dp->lw_watcher_did = 0;
	}
}

/* ============================================================== */
/*
 * internal mac register operation interface
 */
/* ============================================================== */
/*
 * uwgem_mac_init: cold start
 */
static int
uwgem_mac_init(struct uwgem_dev *dp)
{
	int	err;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		/* pretend we succeeded */
		return (USB_SUCCESS);
	}

	ASSERT(dp->mac_state == MAC_STATE_STOPPED);

	/* reset fatal error timestamp */
	dp->fatal_error = (clock_t)0;

	/* reset tx side state */
	mutex_enter(&dp->txlock);
	dp->tx_busy_cnt = 0;
	dp->tx_max_packets = dp->ugc.uwgc_tx_list_max;
	mutex_exit(&dp->txlock);

	/* reset rx side state */
	mutex_enter(&dp->rxlock);
	dp->rx_busy_cnt = 0;
	mutex_exit(&dp->rxlock);
	dp->rx_active = B_FALSE;

	ASSERT(dp->mgtq_head == NULL);
	ASSERT(dp->mgtq_tail == NULL);
#if 0
	err = uwgem_init_rx_buf(dp);
	if (err != USB_SUCCESS) {
		goto x;
	}
#endif
	err = uwgem_hal_init_chip(dp);
	if (err != USB_SUCCESS) {
		goto x;
	}
	dp->mac_state = MAC_STATE_INITIALIZED;
x:
	return (err);
}

/*
 * uwgem_mac_start: warm start
 */
static int
uwgem_mac_start(struct uwgem_dev *dp)
{
	int	err;
	int	i;
	usb_flags_t	flags = 0;
	usb_intr_req_t	*req;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		/* do nothing but don't return failure */
		return (USB_SUCCESS);
	}

	if (dp->mac_state != MAC_STATE_INITIALIZED) {
		/* don't return failure */
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: mac_state(%d) is not MAC_STATE_INITIALIZED",
		    dp->name, __func__, dp->mac_state));
		goto x;
	}

	dp->mac_state = MAC_STATE_ONLINE;

	if (uwgem_hal_start_chip(dp) != USB_SUCCESS) {
		cmn_err(CE_NOTE,
		    "!%s: %s: usb error was detected during start_chip",
		    dp->name, __func__);
		goto x;
	}
#if 1
	/* prepare rx packets */
	if (uwgem_init_rx_buf(dp) != USB_SUCCESS) {
		goto err_stop_intr;
	}
#endif
	dp->rx_active = B_TRUE;

#ifdef UWGEM_DEBUG_LEVEL
	if (dp->intr_pipe) {
		usb_pipe_state_t	p_state;

		usb_pipe_get_state(dp->intr_pipe, &p_state, 0);
		ASSERT(p_state == USB_PIPE_STATE_IDLE);
	}
#endif /* UWGEM_DEBUG_LEVEL */

	if (dp->ugc.uwgc_interrupt && dp->intr_pipe) {
		usb_ep_data_t	*ep_tree_node;
		usb_ep_descr_t	*ep_intr;

		/* make a request for interrupt */
		ep_tree_node = usb_lookup_ep_data(dp->dip, dp->reg_data,
		    dp->ugc.uwgc_ifnum, dp->ugc.uwgc_alt, 0,
		    USB_EP_ATTR_INTR, USB_EP_DIR_IN);
		if (!ep_tree_node) {
			cmn_err(CE_WARN, "!%s: %s: failed to get ep_tree_data",
			    dp->name, __func__);
			goto x;
		}
		ep_intr = &ep_tree_node->ep_descr;

		req = usb_alloc_intr_req(dp->dip, 0, USB_FLAGS_SLEEP);
		if (req == NULL) {
			cmn_err(CE_WARN, "!%s: %s: failed to allocate intreq",
			    dp->name, __func__);
			goto x;
		}
		req->intr_data = NULL;
		req->intr_client_private = (usb_opaque_t)dp;
		req->intr_timeout = 0;
		req->intr_attributes =
		    USB_ATTRS_SHORT_XFER_OK | USB_ATTRS_AUTOCLEARING;
		req->intr_len = ep_intr->wMaxPacketSize;
		req->intr_cb = uwgem_intr_cb;
		req->intr_exc_cb = uwgem_intr_cb;
		req->intr_completion_reason = 0;
		req->intr_cb_flags = 0;

		err = usb_pipe_intr_xfer(dp->intr_pipe, req, flags);
		if (err != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: err:%d failed to start polling of intr pipe",
			    dp->name, err);
			goto x;
		}
	}

	return (USB_SUCCESS);

err_stop_intr:
	/* stop the interrupt pipe */
	DPRINTF(0, (CE_CONT, "!%s: %s: FAULURE", dp->name, __func__));
	if (dp->ugc.uwgc_interrupt && dp->intr_pipe) {
		usb_pipe_stop_intr_polling(dp->intr_pipe, USB_FLAGS_SLEEP);
	}
x:
	ASSERT(dp->mac_state == MAC_STATE_ONLINE);
	/* we use another flag to indicate we are in error state. */
	cmn_err(CE_WARN, "!%s: %s: faulure", dp->name, __func__);
	if (dp->fatal_error == (clock_t)0) {
		dp->fatal_error = uwgem_timestamp_nz();
		cv_signal(&dp->watchdog_cv);
	}
	return (USB_FAILURE);
}

static int
uwgem_mac_stop(struct uwgem_dev *dp, int new_state)
{
	int	i;
	mblk_t	*mp;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/*
	 * we must have writer lock for dev_state_lock
	 */
	ASSERT(new_state == MAC_STATE_STOPPED ||
	    new_state == MAC_STATE_DISCONNECTED);

	switch (dp->mac_state) {
	case MAC_STATE_ONLINE:
		/* stop polling interrupt pipe */
		if (dp->ugc.uwgc_interrupt && dp->intr_pipe) {
			usb_pipe_stop_intr_polling(dp->intr_pipe,
			    USB_FLAGS_SLEEP);
		}
		/* FALLTHRU */

	case MAC_STATE_INITIALIZED:
		if (new_state != MAC_STATE_DISCONNECTED) {
			/* stop the nic hardware completely */
			if (uwgem_hal_stop_chip(dp) != USB_SUCCESS) {
				(void) uwgem_hal_reset_chip(dp, 0);
			}
		}

		/* stop preparing new rx packets and sending new packets */
		dp->mac_state = new_state;

		/* other processors must get mac_state correctly after here */
		membar_producer();
		ASSERT(dp->mac_state != MAC_STATE_ONLINE);

		/* cancel all requests we have sent */
		/*
		 * usb_pipe_reset() during the usb device was
		 * disconnected caused to panic the system.
		 * USB_FLAGS_SLEEP is required, otherwise callback
		 * function won't be called.
		 */
		for (i = 0; i < dp->num_bulkin_pipes; i++) {
			usb_pipe_reset(dp->dip, dp->bulkin_pipe[i],
			    USB_FLAGS_SLEEP, NULL, 0);
		}
		for (i = 0; i < dp->num_bulkout_pipes; i++) {
			usb_pipe_reset(dp->dip, dp->bulkout_pipe[i],
			    USB_FLAGS_SLEEP, NULL, 0);
		}
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: rx_busy_cnt:%d tx_busy_cnt:%d",
		    dp->name, __func__, dp->rx_busy_cnt, dp->tx_busy_cnt));

		/*
		 * ensure callback routines have been executed for
		 * cancelled packets
		 */
		mutex_enter(&dp->rxlock);
		while (dp->rx_busy_cnt > 0) {
			cv_wait(&dp->rx_drain_cv, &dp->rxlock);
		}
		mutex_exit(&dp->rxlock);

		DPRINTF(2, (CE_CONT,
		    "!%s: %s: rx_busy_cnt is %d",
		    dp->name, __func__, dp->rx_busy_cnt));

		mutex_enter(&dp->txlock);
		while (dp->tx_busy_cnt > 0) {
			cv_wait(&dp->tx_drain_cv, &dp->txlock);
		}
		mutex_exit(&dp->txlock);
		DPRINTF(2, (CE_CONT, "!%s: %s: tx_busy_cnt is %d now",
		    dp->name, __func__, dp->tx_busy_cnt));
		/* FALLTHRU */

	case MAC_STATE_STOPPED:
		dp->mac_state = new_state;
		/* FALLTHRU */

	case MAC_STATE_DISCONNECTED:
		/* discard all management requests pended */
		mutex_enter(&dp->watchdog_lock);
		mp = dp->mgtq_head;
		dp->mgtq_head = NULL;
		dp->mgtq_tail = NULL;
		mutex_exit(&dp->watchdog_lock);
		if (mp) {
#if 0
			freemsg/*chain*/(mp);
#else
			mblk_t	*tp;

			while (mp) {
				tp = mp->b_next;
				mp->b_next = NULL;
				freemsg(mp);
				mp = tp;
			}
#endif
		}
		break;
	}

	return (USB_SUCCESS);
}

static int
uwgem_add_multicast(struct uwgem_dev *dp, const uint8_t *ep)
{
	int	cnt;
	int	err = USB_SUCCESS;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	sema_p(&dp->rxfilter_lock);
	if (dp->mc_count_req++ < UWGEM_MAXMC) {
		/* append the new address at the end of the mclist */
		cnt = dp->mc_count;
		bcopy(ep, dp->mc_list[cnt].addr, IEEE80211_ADDR_LEN);
#ifdef notyet
		if (dp->ugc.uwgc_multicast_hash) {
			dp->mc_list[cnt].hash =
			    (*dp->ugc.uwgc_multicast_hash)(dp, ep);
		}
#endif
		dp->mc_count = cnt + 1;
	}

	if (dp->mc_count_req != dp->mc_count) {
		/* multicast address list overflow */
		dp->rxmode |= RXMODE_MULTI_OVF;
	} else {
		dp->rxmode &= ~RXMODE_MULTI_OVF;
	}

	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		/* tell new multicast list to the hardware */
		err = uwgem_hal_set_rx_filter(dp);
	}
	sema_v(&dp->rxfilter_lock);

	return (err);
}

static int
uwgem_remove_multicast(struct uwgem_dev *dp, const uint8_t *ep)
{
	size_t		len;
	int		i;
	int		cnt;
	int		err = USB_SUCCESS;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	sema_p(&dp->rxfilter_lock);
	dp->mc_count_req--;
	cnt = dp->mc_count;
	for (i = 0; i < cnt; i++) {
		if (bcmp(ep, &dp->mc_list[i].addr, IEEE80211_ADDR_LEN)) {
			continue;
		}
		/* shrink the mclist by copying forward */
		len = (cnt - (i + 1)) * sizeof (*dp->mc_list);
		if (len > 0) {
			bcopy(&dp->mc_list[i+1], &dp->mc_list[i], len);
		}
		dp->mc_count--;
		break;
	}

	if (dp->mc_count_req != dp->mc_count) {
		/* multicast address list overflow */
		dp->rxmode |= RXMODE_MULTI_OVF;
	} else {
		dp->rxmode &= ~RXMODE_MULTI_OVF;
	}

	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		err = uwgem_hal_set_rx_filter(dp);
	}
	sema_v(&dp->rxfilter_lock);

	return (err);
}

/* ============================================================== */
/*
 * ioctl interface
 */
/* ============================================================== */

/* ============================================================== */
/*
 * GLDv3 interface
 */
/* ============================================================== */
static int	uwgem_m_getstat(void *, uint_t, uint64_t *);
static int	uwgem_m_start(void *);
static void	uwgem_m_stop(void *);
static int	uwgem_m_setpromisc(void *, boolean_t);
static int	uwgem_m_multicst(void *, boolean_t, const uint8_t *);
static int	uwgem_m_unicst(void *, const uint8_t *);
static mblk_t	*uwgem_m_tx(void *, mblk_t *);
static void	uwgem_m_ioctl(void *, queue_t *, mblk_t *);
static int	uwgem_m_setprop(void *arg, const char *pr_name,
		    mac_prop_id_t wldp_pr_num,
		    uint_t wldp_length, const void *wldp_buf);
#ifdef MAC_VERSION_V1
static int	uwgem_m_getprop(void *arg, const char *pr_name,
		    mac_prop_id_t wldp_pr_num,
		    uint_t wldp_length, void *wldp_buf);
#else
static int	uwgem_m_getprop(void *arg, const char *pr_name,
		    mac_prop_id_t wldp_pr_num, uint_t pr_flags,
		    uint_t wldp_length, void *wldp_buf, uint_t *);
#endif
#ifdef MAC_VERSION_V1
static void	uwgem_m_propinfo(void *arg, const char *pr_name,
    mac_prop_id_t wldp_pr_num, mac_prop_info_handle_t mph);
#endif


static mac_callbacks_t uwgem_m_callbacks = {
#ifdef MAC_VERSION_V1
	MC_IOCTL | MC_SETPROP | MC_GETPROP | MC_PROPINFO,
#else
	MC_IOCTL | MC_SETPROP | MC_GETPROP,
#endif
	uwgem_m_getstat,
	uwgem_m_start,
	uwgem_m_stop,
	uwgem_m_setpromisc,
	uwgem_m_multicst,
	uwgem_m_unicst,
	uwgem_m_tx,
#ifdef MAC_VERSION_V1
	NULL,
#endif
	uwgem_m_ioctl,
	NULL, /* m_getcapab */
	NULL,
	NULL,
	uwgem_m_setprop,
	uwgem_m_getprop,
#ifdef MAC_VERSION_V1
	uwgem_m_propinfo,
#endif
};

static int
uwgem_m_start(void *arg)
{
	int	ret;
	int	err;
	struct uwgem_dev *dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	err = EIO;

	rw_enter(&dp->dev_state_lock, RW_WRITER);
	dp->nic_state = NIC_STATE_ONLINE;

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		err = 0;
		goto x;
	}
	if (uwgem_mac_init(dp) != USB_SUCCESS) {
		goto x;
	}

	/* initialize rx filter state */
	sema_p(&dp->rxfilter_lock);
	dp->rxmode |= RXMODE_ENABLE;
	ret = uwgem_hal_set_rx_filter(dp);
	sema_v(&dp->rxfilter_lock);

	if (ret != USB_SUCCESS) {
		goto x;
	}

	if (uwgem_mac_start(dp) != USB_SUCCESS) {
		goto x;
	}
#ifndef NEW_LOCK
	(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
#endif
	err = 0;
x:
	rw_exit(&dp->dev_state_lock);
#ifdef NEW_LOCK
	if (err == 0) {
		(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
	}
#endif
	return (err);
}

static void
uwgem_m_stop(void *arg)
{
	struct uwgem_dev	*dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

#ifdef NEW_LOCK
	(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
#endif
	/*
	 * stop rx gracefully
	 */
	rw_enter(&dp->dev_state_lock, RW_WRITER);

	sema_p(&dp->rxfilter_lock);
	dp->rxmode &= ~RXMODE_ENABLE;
	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		(void) uwgem_hal_set_rx_filter(dp);
	}
	sema_v(&dp->rxfilter_lock);

	/*
	 * make the nic state inactive
	 */
	dp->nic_state = NIC_STATE_STOPPED;
#ifdef NEW_LOCK
	/* EMPTY */
#else
	(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
#endif
	/* stop mac completely */
	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		(void) uwgem_mac_stop(dp, MAC_STATE_STOPPED);
	}
	rw_exit(&dp->dev_state_lock);
}

static int
uwgem_m_multicst(void *arg, boolean_t add, const uint8_t *ep)
{
	int	err;
	int	ret;
	struct uwgem_dev	*dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rw_enter(&dp->dev_state_lock, RW_READER);
	if (add) {
		ret = uwgem_add_multicast(dp, ep);
	} else {
		ret = uwgem_remove_multicast(dp, ep);
	}
	rw_exit(&dp->dev_state_lock);

	err = 0;
	if (ret != USB_SUCCESS) {
#ifdef UWGEM_CONFIG_FMA
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
#endif
		err = EIO;
	}

	return (err);
}

static int
uwgem_m_setpromisc(void *arg, boolean_t on)
{
	int	err;
	struct uwgem_dev	*dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rw_enter(&dp->dev_state_lock, RW_READER);

	sema_p(&dp->rxfilter_lock);
	if (on) {
		dp->rxmode |= RXMODE_PROMISC;
	} else {
		dp->rxmode &= ~RXMODE_PROMISC;
	}

	err = 0;
	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		if (uwgem_hal_set_rx_filter(dp) != USB_SUCCESS) {
			err = EIO;
		}
	}
	sema_v(&dp->rxfilter_lock);

	rw_exit(&dp->dev_state_lock);

#ifdef UWGEM_CONFIG_FMA
	if (err != 0) {
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
	}
#endif
	return (err);
}

static int
uwgem_m_getstat(void *arg, uint_t stat, uint64_t *valp)
{
	int	ret;
	int	mcs;
	int	rate;
	uint64_t	val;
	struct uwgem_dev	*dp = arg;
	struct ieee80211com	*ic = &dp->ic;
	ieee80211_node_t	*ni = ic->ic_bss;
	struct uwgem_stats	*gstp = &dp->stats;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rw_enter(&dp->dev_state_lock, RW_READER);
	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		rw_exit(&dp->dev_state_lock);
		return (0);
	}
	ret = uwgem_hal_get_stats(dp);

#ifdef UWGEM_CONFIG_FMA
	if (ret != USB_SUCCESS) {
		rw_exit(&dp->dev_state_lock);

		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
		return (EIO);
	}
#endif
#ifdef NEW_LOCK
	rw_exit(&dp->dev_state_lock);
#endif
	ret = 0;
	switch (stat) {
	case MAC_STAT_IFSPEED:
		if (ic->ic_state != IEEE80211_S_RUN) {
			val = 0;
			break;
		}
		if (ni->in_flags & IEEE80211_NODE_HT) {
			mcs = ni->in_htrates.rs_rates[dp->txrate];
			val = ieee80211_htrates[mcs] * 1000000LL / 2;
			if (ni->in_chw == 40) {
				val = val * 2;
			}
		} else {
			/* for 11a/b/g */
			if (ic->ic_fixed_rate == IEEE80211_FIXED_RATE_NONE) {
				val = ni->in_rates.ir_rates[dp->txrate] &
				    IEEE80211_RATE_VAL;
			} else {
				val = ic->ic_fixed_rate;
			}
			val = val * 1000000LL / 2;
		}
		break;

	case MAC_STAT_NORCVBUF:
		val = gstp->norcvbuf;
		break;

	case MAC_STAT_IERRORS:
		val = gstp->errrcv;
		break;

	case MAC_STAT_NOXMTBUF:
		val = gstp->noxmtbuf;
		break;

	case MAC_STAT_OERRORS:
		val = gstp->errxmt;
		break;

	case MAC_STAT_RBYTES:
		val = gstp->rbytes;
		break;

	case MAC_STAT_IPACKETS:
		val = gstp->rpackets;
		break;

	case MAC_STAT_OBYTES:
		val = gstp->obytes;
		break;

	case MAC_STAT_OPACKETS:
		val = gstp->opackets;
		break;

	case WIFI_STAT_TX_FAILED:
		val = gstp->tx_err;
		break;

	case WIFI_STAT_TX_RETRANS:
		val = gstp->tx_retry;
		break;

	case WIFI_STAT_FCS_ERRORS:
		val = gstp->crc;
		break;

	case WIFI_STAT_WEP_ERRORS:
	case WIFI_STAT_TX_FRAGS:
	case WIFI_STAT_MCAST_TX:
	case WIFI_STAT_RTS_SUCCESS:
	case WIFI_STAT_RTS_FAILURE:
	case WIFI_STAT_ACK_FAILURE:
	case WIFI_STAT_RX_FRAGS:
	case WIFI_STAT_MCAST_RX:
	case WIFI_STAT_RX_DUPS:
		ret = ieee80211_stat(ic, stat, &val);
		break;

	default:
#if UWGEM_DEBUG_LEVEL > 2
		cmn_err(CE_WARN,
		    "!%s: unrecognized parameter value = %d",
		    __func__, stat);
#endif
		*valp = 0;
		ret =  ENOTSUP;
	}

	*valp = val;
#ifndef NEW_LOCK
	rw_exit(&dp->dev_state_lock);
#endif

	return (ret);
}

static int
uwgem_m_unicst(void *arg, const uint8_t *mac)
{
	int	err;
	struct uwgem_dev	*dp = arg;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	rw_enter(&dp->dev_state_lock, RW_READER);

	sema_p(&dp->rxfilter_lock);
	bcopy(mac, dp->ic.ic_macaddr, IEEE80211_ADDR_LEN);
	dp->rxmode |= RXMODE_ENABLE;

	err = 0;
	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		if (uwgem_hal_set_rx_filter(dp) != USB_SUCCESS) {
			err = EIO;
		}
	}
	sema_v(&dp->rxfilter_lock);
	rw_exit(&dp->dev_state_lock);

#ifdef UWGEM_CONFIG_FMA
	if (err != 0) {
		ddi_fm_service_impact(dp->dip, DDI_SERVICE_DEGRADED);
	}
#endif
	return (err);
}

/*
 * uwgem_m_tx is used only for sending data packets into ethernet wire.
 */
static mblk_t *
uwgem_m_tx(void *arg, mblk_t *mp_head)
{
	int	limit;
	mblk_t	*mp;
	mblk_t	*nmp;
	uint32_t	flags;
	struct uwgem_dev	*dp = arg;

	DPRINTF(4, (CE_CONT, "!%s: %s: called", dp->name, __func__));
#if 1
	mp = mp_head;
#else
	mp = copyb(mp_head);
	freemsg(mp_head);
	mp_head = mp;
	uwgem_dump_pkt(__func__, mp->b_rptr, mp->b_wptr - mp->b_rptr, 0, 0);
#endif
	flags = 0;

	rw_enter(&dp->dev_state_lock, RW_READER);

	if (dp->ic.ic_state != IEEE80211_S_RUN ||
	    dp->mac_state != MAC_STATE_ONLINE) {
		/* some nics hate to send packets during the link is down */
#if 0
		freemsg/*chain*/(mp);
#else
		mblk_t	*tp;

		while (mp) {
			tp = mp->b_next;
			mp->b_next = NULL;
			freemsg(mp);
			mp = tp;
		}
#endif
		mp = NULL;
		goto x;
	}

	ASSERT(dp->nic_state == NIC_STATE_ONLINE);

	limit = dp->tx_max_packets;
	for (; limit-- && mp; mp = nmp) {
		nmp = mp->b_next;
		mp->b_next = NULL;
		if (uwgem_send_common(dp, mp,
		    IEEE80211_FC0_TYPE_DATA
		    | ((limit == 0 && nmp) ? FLAG_INTR : 0))) {
			mp->b_next = nmp;
			break;
		}
	}
#ifdef CONFIG_TX_LIMITER
	if (mp == mp_head) {
		/* no packets were sent, descrease allocation limit */
		mutex_enter(&dp->txlock);
		dp->tx_max_packets = max(dp->tx_max_packets - 1, 1);
		mutex_exit(&dp->txlock);
	}
#endif
x:
	rw_exit(&dp->dev_state_lock);

	return (mp);
}

static void
uwgem_m_ioctl(void *arg, queue_t *wq, mblk_t *mp)
{
	struct uwgem_dev	*dp = arg;
	struct ieee80211com	*ic = &dp->ic;
	int			err;
#ifdef UWGEM_DEBUG_LEVEL
	struct iocblk		*iocp;
	mblk_t			*mp1;
	wldp_t			*wp;
#endif

#ifdef UWGEM_DEBUG_LEVEL
	iocp = (struct iocblk *)mp->b_rptr;
	mp1 = mp->b_cont;
	wp = NULL;
	if (mp1) {
		wp = (wldp_t *)mp1->b_rptr;
	}
	DPRINTF(1, (CE_CONT, "!%s: %s: called, cmd:0x%x wldp_id:0x%x",
	    dp->name, __func__, iocp->ioc_cmd, wp ? wp->wldp_id : 0));
#endif
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_enter(&dp->dev_state_lock, RW_READER);
#endif
	err = ieee80211_ioctl(ic, wq, mp);

	if (err == ENETRESET) {
		if (ic->ic_des_esslen) {
#ifdef NEW_LOCK
	/* EMPTY */
#else
			rw_exit(&dp->dev_state_lock);
			rw_enter(&dp->dev_state_lock, RW_WRITER);
#endif
			if (dp->ic.ic_state != IEEE80211_S_INIT) {
				(void) ieee80211_new_state(ic,
				    IEEE80211_S_INIT, -1);
			}
			DPRINTF(1, (CE_CONT, "!%s: %s: start scanning...",
			    dp->name, __func__));
			(void) ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
		}
	}
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_exit(&dp->dev_state_lock);
#endif
}

/*
 * Call back function for get/set proporty
 */
static int
#ifdef MAC_VERSION_V1
uwgem_m_getprop(void *arg, const char *pr_name, mac_prop_id_t wldp_pr_num,
    uint_t wldp_length, void *wldp_buf)
#else
uwgem_m_getprop(void *arg, const char *pr_name, mac_prop_id_t wldp_pr_num,
    uint_t pr_flags, uint_t wldp_length, void *wldp_buf, uint_t *perm)
#endif
{
	struct uwgem_dev	*dp = (struct uwgem_dev *)arg;
	int			err = 0;
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_enter(&dp->dev_state_lock, RW_READER);
#endif
#ifdef MAC_VERSION_V1
	err = ieee80211_getprop(&dp->ic, pr_name, wldp_pr_num,
	    wldp_length, wldp_buf);
#else
	err = ieee80211_getprop(&dp->ic, pr_name, wldp_pr_num,
	    pr_flags, wldp_length, wldp_buf, perm);
#endif
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_exit(&dp->dev_state_lock);
#endif
	return (err);
}

#ifdef MAC_VERSION_V1
static void
uwgem_m_propinfo(void *arg, const char *pr_name, mac_prop_id_t wldp_pr_num,
    mac_prop_info_handle_t mph)
{
	struct uwgem_dev	*dp = (struct uwgem_dev *)arg;

	ieee80211_propinfo(&dp->ic, pr_name, wldp_pr_num, mph);
}
#endif
static int
uwgem_m_setprop(void *arg, const char *pr_name, mac_prop_id_t wldp_pr_num,
    uint_t wldp_length, const void *wldp_buf)
{
	struct uwgem_dev	*dp = (struct uwgem_dev *)arg;
	ieee80211com_t		*ic = &dp->ic;
	int			err;

	DPRINTF(1, (CE_CONT, "!%s: %s: called,"
	    " pr_name:%s id_num:%d length:%d",
	    dp->name, __func__,
	    pr_name ? pr_name : "(none)", wldp_pr_num, wldp_length));
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_enter(&dp->dev_state_lock, RW_READER);
#endif
	err = ieee80211_setprop(ic, pr_name, wldp_pr_num, wldp_length,
	    wldp_buf);
	if (err == ENETRESET) {
		if (ic->ic_des_esslen) {
			if (dp->ic.ic_state != IEEE80211_S_INIT) {
				(void) ieee80211_new_state(ic,
				    IEEE80211_S_INIT, -1);
			}
			DPRINTF(1, (CE_CONT, "!%s: %s: start scanning...",
			    dp->name, __func__));
			(void) ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
		}
		err = 0;
	}
#ifdef NEW_LOCK
	/* EMPTY */
#else
	rw_exit(&dp->dev_state_lock);
#endif
	return (err);
}

static void
uwgem_gld3_init(struct uwgem_dev *dp, mac_register_t *macp, wifi_data_t *wdp)
{
#ifdef TEST_ETHER
	macp->m_type_ident = MAC_PLUGIN_IDENT_ETHER;
#else
	macp->m_type_ident = MAC_PLUGIN_IDENT_WIFI;
#endif
	macp->m_driver = dp;
	macp->m_dip = dp->dip;
	macp->m_src_addr = dp->ic.ic_macaddr;
	macp->m_callbacks = &uwgem_m_callbacks;
	macp->m_min_sdu = 0;
	macp->m_max_sdu = dp->mtu;
#ifdef TEST_ETHER
#else
	macp->m_pdata = wdp;
	macp->m_pdata_size = sizeof (*wdp);
#endif
}


/* ======================================================================== */
/*
 * .conf interface
 */
/* ======================================================================== */
void
uwgem_generate_macaddr(struct uwgem_dev *dp, uint8_t *mac)
{
	extern char	hw_serial[];
	char		*hw_serial_p;
	int		i;
	uint64_t	val;
	uint64_t	key;

	cmn_err(CE_NOTE,
	    "!%s: using temp ether address,"
	    " do not use this for long time",
	    dp->name);

	/* prefer a fixed address for DHCP */
	hw_serial_p = &hw_serial[0];
	val = stoi(&hw_serial_p);

	key = 0;
	for (i = 0; i < UWGEM_NAME_LEN; i++) {
		if (dp->name[i] == 0) {
			break;
		}
		key ^= dp->name[i];
	}
	key ^= ddi_get_instance(dp->dip);
	val ^= key << 32;

	/* generate a local address */
	mac[0] = 0x02;
	mac[1] = (uint8_t)(val >> 32);
	mac[2] = (uint8_t)(val >> 24);
	mac[3] = (uint8_t)(val >> 16);
	mac[4] = (uint8_t)(val >> 8);
	mac[5] = (uint8_t)val;
}

boolean_t
uwgem_get_mac_addr_conf(struct uwgem_dev *dp)
{
	char		propname[32];
	char		*valstr;
	uint8_t		mac[IEEE80211_ADDR_LEN];
	char		*cp;
	int		c;
	int		i;
	int		j;
	uint8_t		v;
	uint8_t		d;
	uint8_t		ored;

	DPRINTF(3, (CE_CONT, "!%s: %s: called", dp->name, __func__));
	/*
	 * Get ethernet address from .conf file
	 */
	(void) sprintf(propname, "mac-addr");
	if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
	    DDI_PROP_DONTPASS, propname, &valstr)) != DDI_PROP_SUCCESS) {
		return (B_FALSE);
	}

	if (strlen(valstr) != IEEE80211_ADDR_LEN*3-1) {
		goto syntax_err;
	}

	cp = valstr;
	j = 0;
	ored = 0;
	for (;;) {
		v = 0;
		for (i = 0; i < 2; i++) {
			c = *cp++;

			if (c >= 'a' && c <= 'f') {
				d = c - 'a' + 10;
			} else if (c >= 'A' && c <= 'F') {
				d = c - 'A' + 10;
			} else if (c >= '0' && c <= '9') {
				d = c - '0';
			} else {
				goto syntax_err;
			}
			v = (v << 4) | d;
		}

		mac[j++] = v;
		ored |= v;
		if (j == IEEE80211_ADDR_LEN) {
			/* done */
			break;
		}

		c = *cp++;
		if (c != ':') {
			goto syntax_err;
		}
	}

	if (ored == 0) {
		uwgem_generate_macaddr(dp, mac);
	}
	for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
		dp->dev_addr[i] = mac[i];
	}
	ddi_prop_free(valstr);
	return (B_TRUE);

syntax_err:
	cmn_err(CE_CONT,
	    "!%s: read mac addr: trying .conf: syntax err %s",
	    dp->name, valstr);
	ddi_prop_free(valstr);

	return (B_FALSE);
}

static void
uwgem_read_conf(struct uwgem_dev *dp)
{
	int	val;

	DPRINTF(1, (CE_CONT, "!%s: %s: called", dp->name, __func__));
}

/* ======================================================================== */
/*
 * attach/detatch/usb support
 */
/* ======================================================================== */
static void
uwgem_bulkout_end(usb_pipe_handle_t pipe, usb_bulk_req_t *req)
{
	struct uwgem_dev	*dp;

	dp = (struct uwgem_dev *)req->bulk_client_private;

	DPRINTF(2, (CE_CONT,
	    "!%s: %s: cr:%s(%d) cb_flags:0x%x busycnt:%d",
	    dp->name, __func__,
	    usb_str_cr(req->bulk_completion_reason),
	    req->bulk_completion_reason,
	    req->bulk_cb_flags,
	    dp->tx_busy_cnt));

	/* we have finished to transfer the packet into tx fifo */

	if (req->bulk_completion_reason != USB_CR_OK) {
		cmn_err(CE_WARN,
		    "!%s: %s: error: cr:%s(%d) cb_flags:0x%x",
		    dp->name, __func__,
		    usb_str_cr(req->bulk_completion_reason),
		    req->bulk_completion_reason,
		    req->bulk_cb_flags);

		if (dp->fatal_error == (clock_t)0) {
			dp->fatal_error = uwgem_timestamp_nz();
			cv_signal(&dp->watchdog_cv);
		}
	}

	usb_free_bulk_req(req);
}

int
uwgem_bulk_out(struct uwgem_dev *dp, mblk_t *mp, int qid, int async)
{
	int		i;
	int		err;
	usb_bulk_req_t	*req;
	size_t		len;

	len = mp->b_wptr - mp->b_rptr;

	DPRINTF(2, (CE_CONT, "!%s: %s: mp:%p, len:%d, qid:%d",
	    dp->name, __func__, mp, (int)len, qid));

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		return (USB_PIPE_ERROR);
	}

	req = usb_alloc_bulk_req(dp->dip, 0, USB_FLAGS_SLEEP);

	for (i = uwgem_bulk_retry; i > 0; i--) {
		req->bulk_data = mp;
		req->bulk_len = len;
		req->bulk_client_private = (usb_opaque_t)dp;
#if 0
		req->bulk_timeout = dp->bulkout_timeout;        /* in second */
		req->bulk_attributes = USB_ATTRS_AUTOCLEARING;
		req->bulk_attributes = USB_ATTRS_PIPE_RESET;
#else
		req->bulk_timeout = 10;
		req->bulk_attributes = 0;
#endif
		req->bulk_cb = async ? uwgem_bulkout_end : NULL;
		req->bulk_exc_cb = async ? uwgem_bulkout_end : NULL;
		req->bulk_completion_reason = 0;
		req->bulk_cb_flags = 0;
		DPRINTF(100, (CE_CONT,
		    "!%s: %s: req->bulk_data:%p, req->bulk_len:%d, qid:%d",
		    dp->name, __func__,
		    req->bulk_data, (int)req->bulk_len, qid));

		if ((err = usb_pipe_bulk_xfer(dp->bulkout_pipe[qid],
		    req, async ? 0 : USB_FLAGS_SLEEP)) == USB_SUCCESS) {
			/* done */
			break;
		}

		cmn_err(CE_WARN,
		    "!%s: %s: usb_pipe_bulk_xfer: failed: err:%d",
		    dp->name, __func__, err);

		/* we use another flag to indicate error state. */
		if (dp->fatal_error == (clock_t)0) {
			dp->fatal_error = uwgem_timestamp_nz();
			cv_signal(&dp->watchdog_cv);
		}
	}

	if (!async) {
		usb_free_bulk_req(req);
	}

	return (err);
}

int
uwgem_ctrl_out(struct uwgem_dev *dp,
	uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
	void *bp, int size)
{
	mblk_t			*data;
	usb_ctrl_setup_t	setup;
	usb_cr_t		completion_reason;
	usb_cb_flags_t		cb_flags;
	usb_flags_t		flags;
	int			i;
	int			ret;

	DPRINTF(4, (CE_CONT, "!%s: %s "
	    "reqt:0x%02x req:0x%02x val:0x%04x ix:0x%04x len:0x%02x "
	    "bp:0x%p nic_state:%d",
	    dp->name, __func__, reqt, req, val, ix, len, bp, dp->nic_state));

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		return (USB_PIPE_ERROR);
	}

	data = NULL;
	if (size > 0) {
		if ((data = allocb(size, 0)) == NULL) {
			return (USB_FAILURE);
		}

		bcopy(bp, data->b_rptr, size);
		data->b_wptr = data->b_rptr + size;
	}

	setup.bmRequestType = reqt;
	setup.bRequest = req;
	setup.wValue = val;
	setup.wIndex = ix;
	setup.wLength = len;
	setup.attrs = 0;	/* attributes */

	for (i = uwgem_ctrl_retry; i > 0; i--) {
		completion_reason = 0;
		cb_flags = 0;

		ret = usb_pipe_ctrl_xfer_wait(DEFAULT_PIPE(dp),
		    &setup, &data, &completion_reason, &cb_flags, 0);

		if (ret == USB_SUCCESS) {
			break;
		}
		if (i == 1) {
			cmn_err(CE_WARN,
			    "!%s: %s failed: "
			    "reqt:0x%x req:0x%x val:0x%x ix:0x%x len:0x%x "
			    "ret:%d cr:%s(%d), cb_flags:0x%x %s",
			    dp->name, __func__, reqt, req, val, ix, len,
			    ret, usb_str_cr(completion_reason),
			    completion_reason,
			    cb_flags,
			    (i > 1) ? "retrying..." : "fatal");
		}
	}

	if (data != NULL) {
		freemsg(data);
	}

	return (ret);
}

int
uwgem_ctrl_in(struct uwgem_dev *dp,
	uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
	void *bp, int size)
{
	mblk_t			*data;
	usb_ctrl_setup_t	setup;
	usb_cr_t		completion_reason;
	usb_cb_flags_t		cb_flags;
	int			i;
	int			ret;
	int			reclen;

	DPRINTF(4, (CE_CONT,
	    "!%s: %s:"
	    " reqt:0x%02x req:0x%02x val:0x%04x ix:0x%04x len:0x%02x"
	    " bp:x%p mac_state:%d",
	    dp->name, __func__, reqt, req, val, ix, len, bp, dp->mac_state));

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		return (USB_PIPE_ERROR);
	}

	data = NULL;

	setup.bmRequestType = reqt;
	setup.bRequest = req;
	setup.wValue = val;
	setup.wIndex = ix;
	setup.wLength = len;
	setup.attrs = USB_ATTRS_AUTOCLEARING;	/* XXX */

	for (i = uwgem_ctrl_retry; i > 0; i--) {
		completion_reason = 0;
		cb_flags = 0;
		ret = usb_pipe_ctrl_xfer_wait(DEFAULT_PIPE(dp), &setup, &data,
		    &completion_reason, &cb_flags, 0);

		if (ret == USB_SUCCESS) {
			reclen = msgdsize(data);
			bcopy(data->b_rptr, bp, min(reclen, size));
			break;
		}
		if (i == 1) {
			cmn_err(CE_WARN,
			    "!%s: %s failed: "
			    "reqt:0x%x req:0x%x val:0x%x ix:0x%x len:0x%x "
			    "ret:%d cr:%s(%d) cb_flags:0x%x %s",
			    dp->name, __func__,
			    reqt, req, val, ix, len,
			    ret, usb_str_cr(completion_reason),
			    completion_reason,
			    cb_flags,
			    (i > 1) ? "retrying..." : "fatal");
		}
	}

	if (data) {
		freemsg(data);
	}

	return (ret);
}

int
uwgem_ctrl_out_val(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    uint32_t v)
{
	uint8_t	buf[4];

	/* convert to little endian from native byte order */
	switch (len) {
	case 4:
		buf[3] = v >> 24;
		buf[2] = v >> 16;
		/* fall thru */
	case 2:
		buf[1] = v >> 8;
		/* fall thru */
	case 1:
		buf[0] = v;
	}

	return (uwgem_ctrl_out(dp, reqt, req, val, ix, len, buf, len));
}

int
uwgem_ctrl_in_val(struct uwgem_dev *dp,
    uint8_t reqt, uint8_t req, uint16_t val, uint16_t ix, uint16_t len,
    void *valp)
{
	uint8_t		buf[4];
	uint_t		v;
	int		err;

#ifdef SANITY
	bzero(buf, sizeof (buf));
#endif
	err = uwgem_ctrl_in(dp, reqt, req, val, ix, len, buf, len);
	if (err == USB_SUCCESS) {
		v = 0;
		switch (len) {
		case 4:
			v |= buf[3] << 24;
			v |= buf[2] << 16;
			/* FALLTHROUGH */
		case 2:
			v |= buf[1] << 8;
			/* FALLTHROUGH */
		case 1:
			v |= buf[0];
		}

		switch (len) {
		case 4:
			*(uint32_t *)valp = v;
			break;
		case 2:
			*(uint16_t *)valp = v;
			break;
		case 1:
			*(uint8_t *)valp = v;
			break;
		}
	}
	return (err);
}

/*
 * Attach / detach / disconnect / reconnect management
 */
static int
uwgem_open_pipes(struct uwgem_dev *dp)
{
	int			i;
	int			ret;
	int			ifnum;
	int			alt;
	usb_ep_data_t		*ep_tree_node;
	usb_pipe_policy_t	policy;

#define	MAX_ASYNC_REQS	(UWGEM_MAX_BULKIN_PIPES + UWGEM_MAX_BULKOUT_PIPES + 1)

	ifnum = dp->ugc.uwgc_ifnum;
	alt = dp->ugc.uwgc_alt;

	DPRINTF(0, (CE_CONT, "!%s: %s: called,ifnum:%d alt:%d",
	    dp->name, __func__, ifnum, alt));

	/* open bulkin pipes */
	for (i = 0; i < UWGEM_MAX_BULKIN_PIPES; i++) {
		ep_tree_node = usb_lookup_ep_data(dp->dip, dp->reg_data,
		    dp->ugc.uwgc_ifnum, dp->ugc.uwgc_alt,
		    i, USB_EP_ATTR_BULK, USB_EP_DIR_IN);
		if (ep_tree_node == NULL) {
			DPRINTF(2, (CE_CONT,
			    "!%s: %s: ep_tree_node(bulkin) is NULL",
			    dp->name, __func__));
			break;
		}

		/* open the bulkin pipe */
		bzero(&policy, sizeof (usb_pipe_policy_t));
		policy.pp_max_async_reqs = MAX_ASYNC_REQS;
		if ((ret = usb_pipe_open(dp->dip,
		    &ep_tree_node->ep_descr, &policy, USB_FLAGS_SLEEP,
		    &dp->bulkin_pipe[i])) != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: ret:%d failed to open bulk-in pipe #%d ",
			    dp->name, __func__, ret, i);
			dp->bulkin_pipe[i] = NULL;
			goto err;
		}
		if (ep_tree_node->ep_descr.wMaxPacketSize == 512) {
			dp->highspeed = B_TRUE;
		}
		DPRINTF(1, (CE_CONT,
		    "!%s: %s: bulkin_pipe#%d opened successfully",
		    dp->name, __func__, i));
	}
	dp->num_bulkin_pipes = i;

	/* open bulkout pipes */
	for (i = 0; i < UWGEM_MAX_BULKOUT_PIPES; i++) {
		ep_tree_node = usb_lookup_ep_data(dp->dip, dp->reg_data,
		    dp->ugc.uwgc_ifnum, dp->ugc.uwgc_alt,
		    i, USB_EP_ATTR_BULK, USB_EP_DIR_OUT);
		if (ep_tree_node == NULL) {
			DPRINTF(2, (CE_CONT,
			    "!%s: %s: eep_tree_node(bulkout) is NULL",
			    dp->name, __func__));
			break;
		}

		/* open the bulkout pipe */
		bzero(&policy, sizeof (usb_pipe_policy_t));
		policy.pp_max_async_reqs = MAX_ASYNC_REQS;

		if ((ret = usb_pipe_open(dp->dip,
		    &ep_tree_node->ep_descr, &policy, USB_FLAGS_SLEEP,
		    &dp->bulkout_pipe[i])) != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: err:%x: failed to open bulk-out pipe",
			    dp->name, __func__, ret);
			dp->bulkout_pipe[i] = NULL;
			goto err;
		}
		if (ep_tree_node->ep_descr.wMaxPacketSize == 512) {
			dp->highspeed = B_TRUE;
		}
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: bulkout_pipe#%d opened successfully",
		    dp->name, __func__, i));
	}
	dp->num_bulkout_pipes = i;

	/* open interrupt pipe */
	ep_tree_node = usb_lookup_ep_data(dp->dip, dp->reg_data,
	    dp->ugc.uwgc_ifnum, dp->ugc.uwgc_alt,
	    0, USB_EP_ATTR_INTR, USB_EP_DIR_IN);

	if (ep_tree_node) {
		/* open interrupt pipe */
		bzero(&dp->policy_interrupt, sizeof (usb_pipe_policy_t));
		dp->policy_interrupt.pp_max_async_reqs = MAX_ASYNC_REQS;
		if ((ret = usb_pipe_open(dp->dip,
		    &ep_tree_node->ep_descr,
		    &dp->policy_interrupt, USB_FLAGS_SLEEP,
		    &dp->intr_pipe)) != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: ret:%x failed to open interrupt pipe",
			    dp->name, __func__, ret);
			dp->intr_pipe = NULL;
			goto err;
		}
		DPRINTF(1, (CE_CONT, "!%s: %s: intr_pipe opened successfully",
		    dp->name, __func__));
	} else {
		/* don't care */
		DPRINTF(1, (CE_CONT, "!%s: %s: no intr pipe",
		    dp->name, __func__));
	}

	/* XXX -- no need to open default pipe */


	return (USB_SUCCESS);

err:
	for (i = 0; i < UWGEM_MAX_BULKIN_PIPES; i++) {
		if (dp->bulkin_pipe[i]) {
			usb_pipe_close(dp->dip,
			    dp->bulkin_pipe[i], USB_FLAGS_SLEEP, NULL, 0);
			dp->bulkin_pipe[i] = NULL;
		}
	}
	dp->num_bulkin_pipes = 0;

	for (i = 0; i < UWGEM_MAX_BULKOUT_PIPES; i++) {
		if (dp->bulkout_pipe[i]) {
			usb_pipe_close(dp->dip,
			    dp->bulkout_pipe[i], USB_FLAGS_SLEEP, NULL, 0);
			dp->bulkout_pipe[i] = NULL;
		}
	}
	dp->num_bulkout_pipes = 0;

	if (dp->intr_pipe) {
		usb_pipe_close(dp->dip,
		    dp->intr_pipe, USB_FLAGS_SLEEP, NULL, 0);
		dp->intr_pipe = NULL;
	}

	return (USB_FAILURE);
}

static int
uwgem_close_pipes(struct uwgem_dev *dp)
{
	int	i;

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	if (dp->intr_pipe) {
		usb_pipe_close(dp->dip,
		    dp->intr_pipe, USB_FLAGS_SLEEP, NULL, 0);
		dp->intr_pipe = NULL;
	}
	DPRINTF(1, (CE_CONT, "!%s: %s: 1", dp->name, __func__));

	for (i = 0; i < dp->num_bulkin_pipes; i++) {
		ASSERT(dp->bulkin_pipe[i]);
		usb_pipe_close(dp->dip, dp->bulkin_pipe[i],
		    USB_FLAGS_SLEEP, NULL, 0);
		dp->bulkin_pipe[i] = NULL;
	}
	dp->num_bulkin_pipes = 0;
	DPRINTF(1, (CE_CONT, "!%s: %s: 2", dp->name, __func__));

	for (i = 0; i < dp->num_bulkout_pipes; i++) {
		ASSERT(dp->bulkout_pipe[i]);
		usb_pipe_close(dp->dip, dp->bulkout_pipe[i],
		    USB_FLAGS_SLEEP, NULL, 0);
		dp->bulkout_pipe[i] = NULL;
	}
	dp->num_bulkout_pipes = 0;
	DPRINTF(1, (CE_CONT, "!%s: %s: 3", dp->name, __func__));

	return (USB_SUCCESS);
}

static int
uwgem_freeze_device(struct uwgem_dev *dp, int new_state)
{
	DPRINTF(0, (CE_NOTE, "!%s: %s: called", dp->name, __func__));

	/* stop nic activity */
	(void) uwgem_mac_stop(dp, new_state);

	/*
	 * Here we free all memory resource allocated, because it will
	 * cause to panic the system that we free usb_bulk_req objects
	 * during the usb device is disconnected.
	 */
	(void) uwgem_free_memory(dp);

	return (USB_SUCCESS);
}

static int
uwgem_disconnect_cb(dev_info_t *dip)
{
	int	ret;
	uint_t	mac_state;
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);

	cmn_err(CE_NOTE, "!%s: the usb device was disconnected (dp=%p)",
	    dp->name, dp);

	/* start serialize */
	rw_enter(&dp->dev_state_lock, RW_WRITER);

	mac_state = dp->mac_state;
	ret = uwgem_freeze_device(dp, MAC_STATE_DISCONNECTED);

	/* end serialize */
	rw_exit(&dp->dev_state_lock);
#if 0
	if (mac_state == MAC_STATE_ONLINE) {
		mac_link_update(dp->ic.ic_mach, LINK_STATE_DOWN);
	}
#endif
	DPRINTF(0, (CE_NOTE, "!%s: %s return:", dp->name, __func__));

	return (ret);
}

static int
uwgem_recover_device(struct uwgem_dev *dp)
{
	int	err;

	DPRINTF(0, (CE_NOTE, "!%s: %s: called", dp->name, __func__));

	err = USB_SUCCESS;

	/* re-initialize the usb connection */
	uwgem_close_pipes(dp);
	if ((err = uwgem_open_pipes(dp)) != USB_SUCCESS) {
		goto x;
	}

	/* update mac state */
	dp->mac_state = MAC_STATE_STOPPED;

	/* allocate memory resources again */
	if ((err = uwgem_alloc_memory(dp)) != USB_SUCCESS) {
		goto x;
	}

	/* recover nic state */
	(void) uwgem_restart_nic(dp, 1);
x:
	return (err);
}

static int
uwgem_reconnect_cb(dev_info_t *dip)
{
	int	err = USB_SUCCESS;
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);
	DPRINTF(0, (CE_CONT, "!%s: called, dp=%p", ddi_get_name(dip), dp));
#if 0
	/* check device changes after disconnect */
	if (usb_check_same_device(dp->dip, NULL, USB_LOG_L2, -1,
	    USB_CHK_BASIC | USB_CHK_CFG, NULL) != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: different device connected", dp->name);
		goto x;
	}
#endif
	cmn_err(CE_NOTE, "!%s: the usb device was reconnected", dp->name);

	/* start serialize */
	rw_enter(&dp->dev_state_lock, RW_WRITER);

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		err = uwgem_recover_device(dp);
	}
#ifdef NEW_LOCK
	/* end of serialize */
	rw_exit(&dp->dev_state_lock);
#endif
	/* XXX - should we restart scaning always on reconnect ? */
	if (dp->ic.ic_state != IEEE80211_S_INIT) {
		(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
		(void) ieee80211_new_state(&dp->ic, IEEE80211_S_SCAN, -1);
	}
#ifndef NEW_LOCK
	/* end of serialize */
	rw_exit(&dp->dev_state_lock);
#endif
	/* kick potentially stopped house keeping threads */
	cv_signal(&dp->watchdog_cv);
	cv_signal(&dp->lw_watcher_wait_cv);

	DPRINTF(0, (CE_CONT, "!%s: return, dp=%p", ddi_get_name(dip), dp));
x:
	return (err == USB_SUCCESS ? DDI_SUCCESS : DDI_FAILURE);
}

int
uwgem_suspend(dev_info_t *dip)
{
	int	err = USB_SUCCESS;
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);

	DPRINTF(0, (CE_CONT, "!%s: %s: callded", dp->name, __func__));

	/* start serialize */
	rw_enter(&dp->dev_state_lock, RW_WRITER);

	if (dp->mac_state != MAC_STATE_DISCONNECTED) {
		err = uwgem_freeze_device(dp, MAC_STATE_STOPPED);

		/* move to DISCONNECTED state to inhibit further operations */
		dp->mac_state = MAC_STATE_DISCONNECTED;
	}

	/* end serialize */
	rw_exit(&dp->dev_state_lock);

	return (err == USB_SUCCESS ? DDI_SUCCESS : DDI_FAILURE);
}

int
uwgem_resume(dev_info_t *dip)
{
	int	err = USB_SUCCESS;
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);

	DPRINTF(0, (CE_CONT, "!%s: %s: callded", dp->name, __func__));
#if 0
	/* check device changes after disconnect */
	if (usb_check_same_device(dp->dip, NULL, USB_LOG_L2, -1,
	    USB_CHK_BASIC | USB_CHK_CFG, NULL) != USB_SUCCESS) {
		cmn_err(CE_CONT,
		    "!%s: no or different device installed", dp->name);
		goto x;
	}
#endif
	/* start serialize */
	rw_enter(&dp->dev_state_lock, RW_WRITER);

	if (dp->mac_state == MAC_STATE_DISCONNECTED) {
		err = uwgem_recover_device(dp);
	}
#ifdef NEW_LOCK
	/* end of serialize */
	rw_exit(&dp->dev_state_lock);
#endif
	if (dp->ic.ic_state != IEEE80211_S_INIT) {
		(void) ieee80211_new_state(&dp->ic, IEEE80211_S_INIT, -1);
		(void) ieee80211_new_state(&dp->ic, IEEE80211_S_SCAN, -1);
	}
#ifndef NEW_LOCK
	/* end of serialize */
	rw_exit(&dp->dev_state_lock);
#endif
	/* kick potentially stopped house keeping threads */
	cv_signal(&dp->watchdog_cv);
	cv_signal(&dp->lw_watcher_wait_cv);
x:
	return (err == USB_SUCCESS ? DDI_SUCCESS : DDI_FAILURE);
}

#define	UWGEM_LOCAL_DATA_SIZE()	\
	(sizeof (struct uwgem_dev) + UWGEM_MCALLOC)

struct uwgem_dev *
uwgem_do_attach(dev_info_t *dip,
    struct uwgem_conf *gc, void *lp, int lmsize)
{
	struct uwgem_dev	*dp;
	struct ieee80211com	*ic;
	int			i;
	mac_register_t		*macp;
	int			ret;
	int			unit;
	int			err;
	wifi_data_t		wd = { 0 };

	unit = ddi_get_instance(dip);

	DPRINTF(0, (CE_CONT, "!%s: %s: called", gc->uwgc_name, __func__));

	/*
	 * Allocate soft data structure
	 */
	dp = kmem_zalloc(UWGEM_LOCAL_DATA_SIZE(), KM_SLEEP);
	if (dp == NULL) {
		return (NULL);
	}
	if ((macp = mac_alloc(MAC_VERSION)) == NULL) {
		cmn_err(CE_WARN, "!%s: %s: mac_alloc failed",
		    gc->uwgc_name, __func__);
		goto err_free_private;
	}

	/* link to private area */
	dp->private = lp;
	dp->priv_size = lmsize;
	dp->mc_list = (struct mcast_addr *)&dp[1];

	dp->dip = dip;
	bcopy(gc->uwgc_name, dp->name, UWGEM_NAME_LEN);

	/* setup for ieee80211 */
	ic = &dp->ic;

	/*
	 * register with usb service
	 */
	if (usb_client_attach(dip, USBDRV_VERSION, 0) != USB_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: usb_client_attach failed",
		    dp->name, __func__);
		goto err_free_mac;
	}

	if (usb_get_dev_data(dip, &dp->reg_data,
	    USB_PARSE_LVL_ALL, 0) != USB_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: usb_get_dev_data failed",
		    dp->name, __func__);
		dp->reg_data = NULL;
		goto err_unregister_client;
	}
#ifdef UWGEM_DEBUG_LEVEL
	usb_print_descr_tree(dp->dip, dp->reg_data);
#endif

	if (uwgem_open_pipes(dp) != USB_SUCCESS) {
		/* failed to open pipes */
		cmn_err(CE_WARN, "!%s: %s: failed to open pipes",
		    dp->name, __func__);
		goto err_unregister_client;
	}

	/*
	 * Initialize mutexs and condition variables
	 */
	mutex_init(&dp->rxlock, NULL, MUTEX_DRIVER, NULL);
	mutex_init(&dp->txlock, NULL, MUTEX_DRIVER, NULL);
	cv_init(&dp->rx_drain_cv, NULL, CV_DRIVER, NULL);
	cv_init(&dp->tx_drain_cv, NULL, CV_DRIVER, NULL);
	rw_init(&dp->dev_state_lock, NULL, RW_DRIVER, NULL);
	mutex_init(&dp->lw_lock, NULL, MUTEX_DRIVER, NULL);
	cv_init(&dp->lw_watcher_wait_cv, NULL, CV_DRIVER, NULL);
	sema_init(&dp->hal_op_lock, 1, NULL, SEMA_DRIVER, NULL);
	sema_init(&dp->rxfilter_lock, 1, NULL, SEMA_DRIVER, NULL);
	sema_init(&dp->ioctl_lock, 1, NULL, SEMA_DRIVER, NULL);

	mutex_init(&dp->watchdog_lock, NULL, MUTEX_DRIVER, NULL);
	cv_init(&dp->watchdog_cv, NULL, CV_DRIVER, NULL);

	/*
	 * Initialize configuration
	 */
	dp->ugc = *gc;

	dp->mtu = IEEE80211_MTU;
	dp->rxmode = 0;

	dp->nic_state = NIC_STATE_STOPPED;
	dp->mac_state = MAC_STATE_STOPPED;

	/*
	 * Get media mode infomation from .conf file
	 */
	uwgem_read_conf(dp);

	/* rx_buf_len depend on MTU */
	dp->rx_buf_len = MAXPKTBUF(dp) + dp->ugc.uwgc_rx_header_len;

	/* setup initial state */
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* setup call backs */
	ic->ic_xmit = uwgem_mgt_send;
	ic->ic_watchdog = NULL;	/* uwgem_watchdog */	/* XXX */
	ic->ic_set_shortslot = NULL; /* uwgem_set_shortslot */	/* XXX */

	ic->ic_def_txkey = 0;

	/* reset the chip */
	if (uwgem_hal_reset_chip(dp, 1) != USB_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to reset the usb device",
		    dp->name, __func__);
		goto err_destroy_locks;
	}

	/* HW dependant paremeter initialization */
	if (uwgem_hal_attach_chip(dp) != USB_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to attach the usb device",
		    dp->name, __func__);
		goto err_destroy_locks;
	}

	/* copy mac address */
	IEEE80211_ADDR_COPY(dp->ic.ic_macaddr, dp->dev_addr);

	ieee80211_attach(ic);

	/* register WPA door */
	ieee80211_register_door(ic, ddi_driver_name(dip),
	    ddi_get_instance(dip));

	/* override state transition machine */
	dp->ic_newstate = ic->ic_newstate;
	ic->ic_newstate = uwgem_newstate;

	/* override management action on receiving management packets() */
	dp->ic_recv_mgmt = ic->ic_recv_mgmt;
	ic->ic_recv_mgmt = uwgem_recv_mgmt;

	ieee80211_media_init(ic);

	/* allocate resources */
	if (uwgem_alloc_memory(dp) != USB_SUCCESS) {
		goto err_detach_80211;
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %02x:%02x:%02x:%02x:%02x:%02x",
	    dp->name,
	    dp->ic.ic_macaddr[0], dp->ic.ic_macaddr[1], dp->ic.ic_macaddr[2],
	    dp->ic.ic_macaddr[3], dp->ic.ic_macaddr[4], dp->ic.ic_macaddr[5]));


	/* pre-calculated tx timeout in second for performance */
	dp->bulkout_timeout =
	    dp->ugc.uwgc_tx_timeout / drv_usectohz(1000*1000);

	/*
	 * Provide initial settings for the WiFi plugin; whenever this
	 * information changes, we need to call mac_plugindata_update()
	 */
	wd.wd_opmode = ic->ic_opmode;
	wd.wd_secalloc = WIFI_SEC_NONE;
	IEEE80211_ADDR_COPY(wd.wd_bssid, ic->ic_bss->in_bssid);

	uwgem_gld3_init(dp, macp, &wd);

	if (ret = mac_register(macp, &dp->ic.ic_mach)) {
		cmn_err(CE_WARN, "!%s: mac_register failed, error:%d",
		    dp->name, ret);
		goto err_free_memory;
	}

	/*
	 * Create minor node of type DDI_NT_NET_WIFI
	 */
	err = ddi_create_minor_node(dip, dp->name, S_IFCHR,
	    unit + 1, DDI_NT_NET_WIFI, 0);
	if (err != DDI_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to create minor node",
		    dp->name, __func__);
		goto err_unregister_gld;
	}

	if (usb_register_hotplug_cbs(dip,
	    uwgem_disconnect_cb, uwgem_reconnect_cb)
	    != USB_SUCCESS) {
		cmn_err(CE_WARN,
		    "!%s: %s: failed to register hotplug cbs",
		    dp->name, __func__);
		goto err_remove_minor_node;
	}

	/* start wifi link watcher */
	if (uwgem_lw_start(dp) != USB_SUCCESS) {
		goto err_unregister_hotplug;
	}

	/* start wifi watchdog */
	if (uwgem_watchdog_start(dp) != USB_SUCCESS) {
		goto err_stop_link_watcher;
	}

	ddi_set_driver_private(dip, (caddr_t)dp);
	mac_free(macp);

	DPRINTF(2, (CE_CONT, "!%s: %s: return: success", dp->name, __func__));

	return (dp);

err_stop_watchdog:
	/* stop tx timeout watcher */
	uwgem_watchdog_stop(dp);

err_stop_link_watcher:
	/* stop the link manager */
	uwgem_lw_stop(dp);

err_unregister_hotplug:
	usb_unregister_hotplug_cbs(dip);

err_remove_minor_node:
	ddi_remove_minor_node(dip, NULL);

err_unregister_gld:
	mac_unregister(dp->ic.ic_mach);

err_free_memory:
	uwgem_free_memory(dp);

err_detach_80211:
	ieee80211_detach(&dp->ic);

err_destroy_locks:
	/* release locks and condition variables */
	mutex_destroy(&dp->rxlock);
	mutex_destroy(&dp->txlock);
	cv_destroy(&dp->rx_drain_cv);
	cv_destroy(&dp->tx_drain_cv);
	rw_destroy(&dp->dev_state_lock);
	mutex_destroy(&dp->lw_lock);
	cv_destroy(&dp->lw_watcher_wait_cv);
	sema_destroy(&dp->hal_op_lock);
	sema_destroy(&dp->rxfilter_lock);
	sema_destroy(&dp->ioctl_lock);

	mutex_destroy(&dp->watchdog_lock);
	cv_destroy(&dp->watchdog_cv);

err_close_pipes:
	(void) uwgem_close_pipes(dp);

err_unregister_client:
	usb_client_detach(dp->dip, dp->reg_data);
	dp->reg_data = NULL;

	/* release basic memory resources */
err_free_mac:
	mac_free(macp);

err_free_private:
	kmem_free((caddr_t)dp->private, dp->priv_size);
	kmem_free((caddr_t)dp, UWGEM_LOCAL_DATA_SIZE());

	DPRINTF(-1, (CE_CONT, "!%s: %s: return: failure",
	    ddi_driver_name(dip), __func__));

	return (NULL);
}

int
uwgem_do_detach(dev_info_t *dip)
{
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);

	DPRINTF(0, (CE_CONT, "!%s: %s: called", dp->name, __func__));

	/* stop watchdog */
	uwgem_watchdog_stop(dp);

	/* stop the link manager */
	uwgem_lw_stop(dp);

	/* unregister with hotplug service */
	usb_unregister_hotplug_cbs(dip);

	ddi_remove_minor_node(dip, NULL);

	/* unregister with gld v3 */
	if (mac_unregister(dp->ic.ic_mach) != 0) {
		cmn_err(CE_WARN, "!%s: %s: failed",
		    dp->name, __func__);
		return (DDI_FAILURE);
	}

	/* unregister with usb service */
	(void) uwgem_free_memory(dp);

	/* detach ieee802.11 */
	ieee80211_detach(&dp->ic);

	/* release locks and condition variables */
	mutex_destroy(&dp->txlock);
	mutex_destroy(&dp->rxlock);
	cv_destroy(&dp->tx_drain_cv);
	cv_destroy(&dp->rx_drain_cv);
	rw_destroy(&dp->dev_state_lock);
	mutex_destroy(&dp->lw_lock);
	cv_destroy(&dp->lw_watcher_wait_cv);
	sema_destroy(&dp->hal_op_lock);
	sema_destroy(&dp->rxfilter_lock);
	sema_destroy(&dp->ioctl_lock);

	mutex_destroy(&dp->watchdog_lock);
	cv_destroy(&dp->watchdog_cv);

	(void) uwgem_close_pipes(dp);

	usb_client_detach(dp->dip, dp->reg_data);
	dp->reg_data = NULL;

	/* release basic memory resources */
	kmem_free((caddr_t)dp->private, dp->priv_size);
	kmem_free((caddr_t)dp, UWGEM_LOCAL_DATA_SIZE());

	DPRINTF(2, (CE_CONT, "!%s: %s: return: success",
	    ddi_driver_name(dip), __func__));

	return (DDI_SUCCESS);
}

int
uwgem_mod_init(struct dev_ops *dop, char *name)
{
	major_t	major;

	major = ddi_name_to_major(name);
	if (major == DDI_MAJOR_T_NONE) {
		return (DDI_FAILURE);
	}
	mac_init_ops(dop, name);
	return (DDI_SUCCESS);
}

void
uwgem_mod_fini(struct dev_ops *dop)
{
	mac_fini_ops(dop);
}

int
uwgem_quiesce(dev_info_t *dip)
{
	struct uwgem_dev	*dp;

	dp = UWGEM_GET_DEV(dip);

	ASSERT(dp != NULL);

	rw_enter(&dp->dev_state_lock, RW_WRITER);
	if (dp->mac_state != MAC_STATE_DISCONNECTED &&
	    dp->mac_state != MAC_STATE_STOPPED) {
		if (uwgem_hal_stop_chip(dp) != USB_SUCCESS) {
			(void) uwgem_hal_reset_chip(dp, 0);
		}
		dp->mac_state = MAC_STATE_STOPPED;
	}
	rw_exit(&dp->dev_state_lock);

	/* devo_quiesce() must return DDI_SUCCESS always */
	return (DDI_SUCCESS);
}
