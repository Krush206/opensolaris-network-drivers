/* copyright here */
#pragma ident "%W% %E%"

/*
 *  Changelog:
 */

/*
 * TODO
 *
 */
/* ======================================================= */

/*
 * Solaris system header files and macros
 */

/* minimum kernel headers for drivers */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/sysmacros.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/byteorder.h>

/* ethernet stuff */
#include <sys/ethernet.h>

/* interface card depend stuff */
#include <sys/stropts.h>
#include <sys/stream.h>
#include <sys/strsun.h>
#include <sys/strlog.h>
#include <sys/usb/usba.h>

#include <sys/mac.h>
#ifndef MAC_VERSION
#include <sys/mac_provider.h>
#endif
#include <sys/mac_ether.h>
#include <sys/mac_wifi.h>
#include <sys/net80211.h>

#include "uwgem.h"

#ifdef DEBUG_LEVEL
#include "net80211/net80211_impl.h"
#endif

/* hardware stuff */
#undef SLOT
#if 1
#define	__packed	__attribute((packed))
#else
#define	__packed
#endif
#define	BIT(n)	(1U << (n))
#include "if_rsureg.h"

char	ident[] = "RTL8188SU/8192SU driver v" VERSION;

/*
 * Useful macros
 */
#define	CHECK_AND_JUMP(err, label)	if (err != USB_SUCCESS) goto label
#define	LE16P(p)	((((uint8_t *)(p))[1] << 8) | ((uint8_t *)(p))[0])
#define	LE32P(p)	((LE16P(((uint8_t*)(p)) + 2) << 16) | LE16P(p))

#define	N(a)	(sizeof (a) / sizeof ((a)[0]))

#define	DELAY(n)	drv_usecwait(n)

#define	MAX_XFER_SIZE	(64*1024)

#define	htole32(x)	LE_32(x)
#define	htole16(x)	LE_16(x)
#define	letoh32(x)	LE_32(x)
#define	letoh16(x)	LE_16(x)

/*
 * Debugging
 */
#ifdef DEBUG_LEVEL
static int rsu_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (rsu_debug > (n)) cmn_err args
#else
#define	DPRINTF(n, args)
#endif

#ifdef DEBUG
#define	RUN_DEBUG \
	rsu_debug_printf
#else
#define	RUN_DEBUG
#endif

/*
 * Our configration
 */
/* timeouts */
#define	ONESEC		(drv_usectohz(1*1000000))


/*
 * RX/TX buffer size
 */

/*
 * Local definitions
 */
/* see rsu_var.h */
struct rsu_dev {
	boolean_t initialized;
	uint8_t	cut;		/* chip revision */
	uint8_t	board_type;
	uint8_t	customer_id;
	uint8_t	qid2idx[16];
	int	scan_pass;
	uint8_t	cmd_seq;
	int	usb_pkt_size;
	kcondvar_t	cv_scan_done;
	kmutex_t	scan_done_lock;
#define	EEPROM_MAX_SIZE	(16*8)
	uint8_t	rom[EEPROM_MAX_SIZE];
};

/*
 * Read only variables
 */
static uint8_t rtl8712fw[129304] = {
#include MICROCODE
};

static const struct ieee80211_rateset rsu_rateset_11b =
	{ 4, { 2, 4, 11, 22 } };

static const struct ieee80211_rateset rsu_rateset_11g =
	{ 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };

struct ether_addr rsu_broadcastaddr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* newstate */

/* nic operations */
static int rsu_reset_chip(struct uwgem_dev *, uint_t);
static int rsu_init_chip(struct uwgem_dev *);
static int rsu_start_chip(struct uwgem_dev *);
static int rsu_stop_chip(struct uwgem_dev *);
static int rsu_set_rx_filter(struct uwgem_dev *);
static int rsu_get_stats(struct uwgem_dev *);

/* packet operations */
static mblk_t *rsu_tx_make_packet(struct uwgem_dev *, mblk_t *,
    uint_t, struct ieee80211_node *ni);
static mblk_t *rsu_rx_make_packet(struct uwgem_dev *, mblk_t *);

/* =============================================================== */
/*
 * I/O functions
 */
/* =============================================================== */
#define	RSU_WRITE_VAL(dp, reg, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	sizeof (v),	\
	/* value */	(v))) != USB_SUCCESS) goto label

#define	RSU_WRITE8(dp, reg, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	1,	\
	/* value */	(v))) != USB_SUCCESS) goto label

#define	RSU_WRITE16(dp, reg, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	2,	\
	/* value */	(v))) != USB_SUCCESS) goto label

#define	RSU_WRITE32(dp, reg, v, errp, label)	\
	if ((*(errp) = uwgem_ctrl_out_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_HOST_TO_DEV	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	4,	\
	/* value */	(v))) != USB_SUCCESS) goto label

#define	RSU_READ_VAL(dp, reg, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	sizeof (*(vp)),	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

#define	RSU_READ8(dp, reg, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	1,	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

#define	RSU_READ16(dp, reg, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	2,	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

#define	RSU_READ32(dp, reg, vp, errp, label)	\
	if ((*(errp) = uwgem_ctrl_in_val((dp), 	\
	/* bmRequestType */ USB_DEV_REQ_DEV_TO_HOST	\
		| USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV,	\
	/* bRequest */	5,	\
	/* wValue */	(uint16_t)(reg),	\
	/* wIndex */	0,	\
	/* wLength */	4,	\
	/* valuep */	(vp))) != USB_SUCCESS) goto label

void
rsu_write_1(struct uwgem_dev *dp, int reg, uint8_t v)
{
	int	err;

	DPRINTF(10, (CE_CONT, "!%s: W8 %x: %02x", dp->name, reg, v));
	RSU_WRITE_VAL(dp, reg, v, &err, usberr);
usberr:
	return;
}

void
rsu_write_2(struct uwgem_dev *dp, int reg, uint16_t v)
{
	int	err;

	DPRINTF(10, (CE_CONT, "!%s: W16 %x: %04x", dp->name, reg, v));
	RSU_WRITE_VAL(dp, reg, v, &err, usberr);
usberr:
	return;
}

void
rsu_write_4(struct uwgem_dev *dp, int reg, uint32_t v)
{
	int	err;

	DPRINTF(10, (CE_CONT, "!%s: W32 %x: %08x", dp->name, reg, v));
	RSU_WRITE_VAL(dp, reg, v, &err, usberr);
usberr:
	return;
}

uint8_t
rsu_read_1(struct uwgem_dev *dp, int reg)
{
	int	err;
	uint8_t	v = 0;

	RSU_READ_VAL(dp, reg, &v, &err, usberr);
	DPRINTF(10, (CE_CONT, "!%s: R8 %x: %02x", dp->name, reg, v));
usberr:
	return (v);
}

uint16_t
rsu_read_2(struct uwgem_dev *dp, int reg)
{
	int	err;
	uint16_t	v = 0;

	RSU_READ_VAL(dp, reg, &v, &err, usberr);
	DPRINTF(10, (CE_CONT, "!%s: R16 %x: %04x", dp->name, reg, v));
usberr:
	return (v);
}

uint32_t
rsu_read_4(struct uwgem_dev *dp, int reg)
{
	int	err;
	uint32_t	v = 0;

	RSU_READ_VAL(dp, reg, &v, &err, usberr);
	DPRINTF(10, (CE_CONT, "!%s: R32 %x: %08x", dp->name, reg, v));
usberr:
	return (v);
}
#if 0
int
rsu_write_mem(struct uwgem_dev *dp, mblk_t *mp, uint_t which)
{
	int	err;
	int	pipe;
#ifdef notyet
	if (which == RTL8712_DMA_VOQ) {
		pipe = 0;
	} else {
		cmn_err(CE_CONT, "!%s: %s: illegal pipe %x",
		    dp->name, __func__, which);
		return (USB_FAILURE);
	}
#endif
	err = uwgem_bulk_out(dp, mp, pipe, 0);

	return (err);
}
#endif

#if 0
#ifdef CONFIG_11N
/* List of devices that have HT support disabled. */
static const struct usb_devno rsu_devs_noht[] = {
	{ USB_VENDOR_ASUS,		USB_PRODUCT_ASUS_RTL8192SU_1 },
	{ USB_VENDOR_AZUREWAVE,		USB_PRODUCT_AZUREWAVE_RTL8192SU_4 }
};
#endif
#endif

void	rsu_calib_to(void *);
void	rsu_calib_cb(struct uwgem_dev *, void *);
static int rsu_newstate(struct uwgem_dev *, enum ieee80211_state, int);
int	rsu_set_key(struct uwgem_dev *, struct ieee80211_node *,
	    struct ieee80211_key *);
void	rsu_delete_key(struct uwgem_dev *, int);
static int rsu_site_survey(struct uwgem_dev *);
static int rsu_join_bss(struct uwgem_dev *, struct ieee80211_node *);
static int rsu_disconnect(struct uwgem_dev *);
static mblk_t *rsu_event_survey(struct uwgem_dev *, uint8_t *, int);
static mblk_t *rsu_event_join_bss(struct uwgem_dev *, uint8_t *, int);
static mblk_t *rsu_rx_event(struct uwgem_dev *, uint8_t, uint8_t *, int);
static mblk_t *rsu_rx_multi_event(struct uwgem_dev *, mblk_t *);
static int8_t	rsu_get_rssi(struct uwgem_dev *, int, void *);
static mblk_t *
rsu_rx_frame(struct uwgem_dev *, uint8_t *, size_t, mblk_t *, mblk_t *);
static mblk_t *rsu_rx_multi_frame(struct uwgem_dev *, mblk_t *);
static void rsu_power_off(struct uwgem_dev *);
static int rsu_fw_loadsection(struct uwgem_dev *, uint8_t *, int, int);
static int rsu_load_firmware(struct uwgem_dev *);

static int
rsu_fw_iocmd(struct uwgem_dev *dp, uint32_t iocmd)
{
	int ntries;
	int reg;

	rsu_write_4(dp, R92S_IOCMD_CTRL, iocmd);
#if 0
	delay(drv_usectohz(100*1000));
#endif
	for (ntries = 0; ntries < 50; ntries++) {
		if ((reg = rsu_read_4(dp, R92S_IOCMD_CTRL)) == 0) {
			return (USB_SUCCESS);
		}
		delay(drv_usectohz(20*1000));
		DPRINTF(0, (CE_CONT, "!%s: %s: reg:%x",
		    dp->name, __func__, reg));
	}
	return (ETIMEDOUT);
}

static uint8_t
rsu_efuse_read_1(struct uwgem_dev *dp, uint16_t addr)
{
	uint32_t	reg;
	int		ntries;

	/*
	 * The original code just do single byte read/write
	 */
	rsu_write_1(dp, R92S_EFUSE_CTRL + 1, (uint8_t)addr);
	rsu_write_1(dp, R92S_EFUSE_CTRL + 2,
	    (rsu_read_1(dp, R92S_EFUSE_CTRL + 2) & ~0x03) |
	    ((addr >> 8) & 0x03));
	rsu_write_1(dp, R92S_EFUSE_CTRL + 3, 0x72);	/* read command */

	/* Wait for read operation to complete. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (rsu_read_1(dp, R92S_EFUSE_CTRL + 3) &
		    (R92S_EFUSE_CTRL_VALID >> 24)) {
			return (rsu_read_1(dp, R92S_EFUSE_CTRL));
		}
		DELAY(5);
	}
	cmn_err(CE_CONT, "!%s: could not read efuse byte at address 0x%x",
	    dp->name, addr);
	return (0xff);
}

static int
rsu_read_rom(struct uwgem_dev *dp)
{
	struct rsu_dev	*lp = dp->private;
	uint8_t		*rom = lp->rom;
	uint16_t	addr = 0;
	uint32_t	reg;
	uint8_t		off, msk;
	int		i;
#ifndef CONFIG_NO_ROM
	/* Make sure that ROM type is eFuse and that autoload succeeded. */
	reg = rsu_read_1(dp, R92S_EE_9346CR);
	if ((reg & (R92S_9356SEL | R92S_EEPROM_EN)) != R92S_EEPROM_EN) {
		return (USB_FAILURE);
	}

	/* Turn on 2.5V to prevent eFuse leakage. */
	reg = rsu_read_1(dp, R92S_EFUSE_TEST + 3);
	rsu_write_1(dp, R92S_EFUSE_TEST + 3, reg | 0x80);
	delay(drv_usectohz(20*1000));
	rsu_write_1(dp, R92S_EFUSE_TEST + 3, reg & ~0x80);
#endif
	/* Read full ROM image. */
	memset(&lp->rom, 0xff, sizeof (lp->rom));
#ifndef CONFIG_NO_ROM
	while (addr < 512) {
		reg = rsu_efuse_read_1(dp, addr);
		if (reg == 0xff) {
			break;
		}
		addr++;
		off = reg >> 4;
		msk = reg & 0xf;
		for (i = 0; i < 4; i++) {
			if (msk & (1 << i)) {
				continue;
			}
			rom[off * 8 + i * 2 + 0] =
			    rsu_efuse_read_1(dp, addr);
			addr++;
			rom[off * 8 + i * 2 + 1] =
			    rsu_efuse_read_1(dp, addr);
			addr++;
		}
	}
#else
{
	static uint8_t rom_copy[EEPROM_MAX_SIZE] = {
		0x29, 0x81, 0x00, 0x00, 0xa9, 0x16, 0x00, 0x00,
		0xaa, 0x07, 0x47, 0x00, 0x90, 0x85, 0x62, 0x9c,
		0x06, 0x00, 0x00, 0x0a, 0x79, 0xed, 0x94, 0x4e,
		0x17, 0x03, 0x4d, 0x61, 0x6e, 0x75, 0x66, 0x61,

		0x63, 0x74, 0x75, 0x72, 0x65, 0x72, 0x20, 0x52,
		0x65, 0x61, 0x6c, 0x74, 0x65, 0x6b, 0x20, 0x18,
		0x03, 0x52, 0x54, 0x4c, 0x38, 0x31, 0x38, 0x38,
		0x53, 0x20, 0x57, 0x4c, 0x41, 0x4e, 0x20, 0x41,

		0x64, 0x61, 0x70, 0x74, 0x65, 0x72, 0x20, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x02, 0x02, 0x0b, 0x00, 0x00, 0x2c, 0x2b, 0x2a,
		0x10, 0x00, 0x10, 0x33, 0x32, 0x31, 0x10, 0x10,

		0x10, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x00,
		0x12, 0x00, 0x00, 0x09, 0x0d, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	memcpy(lp->rom, rom_copy, sizeof (lp->rom));
}
#endif
	return (USB_SUCCESS);
}

static int
rsu_fw_cmd(struct uwgem_dev *dp, uint8_t code, void *buf, int len)
{
	struct rsu_dev *lp = dp->private;
	struct r92s_tx_desc *txd;
	struct r92s_fw_cmd_hdr *cmd;
	int	cmdsz, xferlen;
	mblk_t	*mp;
	int	pktsize;

	/* Round-up command length to a multiple of 8 bytes. */
	cmdsz = P2ROUNDUP(len, 8);
	pktsize = dp->highspeed ? 512 : 64;
	xferlen = sizeof (*txd) + sizeof (*cmd) + cmdsz;
	if ((xferlen & (pktsize - 1)) == 0) {
		cmdsz += 8;
		xferlen += 8;
	}

	ASSERT(xferlen <= RSU_TXBUFSZ);

	mp = allocb(xferlen, 0);
	mp->b_wptr = mp->b_rptr + xferlen;
	memset(mp->b_rptr, 0, xferlen);

	/* Setup Tx descriptor. */
	txd = (struct r92s_tx_desc *)mp->b_rptr;
	txd->txdw0 = htole32(
	    SM(R92S_TXDW0_OFFSET, sizeof (*txd)) |
	    SM(R92S_TXDW0_PKTLEN, sizeof (*cmd) + cmdsz) |
	    R92S_TXDW0_OWN | R92S_TXDW0_FSG | R92S_TXDW0_LSG);

	/* XXX - we must specify a queue >= 8 for fw commands */
	txd->txdw1 = htole32(SM(R92S_TXDW1_QSEL, R92S_TXDW1_QSEL_H2C));

	/* Setup command header. */
	cmd = (struct r92s_fw_cmd_hdr *)(mp->b_rptr + sizeof (*txd));
	cmd->len = htole16(cmdsz);
	cmd->code = code;
	cmd->seq = lp->cmd_seq;
	lp->cmd_seq = (lp->cmd_seq + 1) & 0x7f;

	/* Copy command payload. */
	memcpy(&cmd[1], buf, len);

	DPRINTF(10, (CE_CONT, "!%s: %s: Tx cmd code=%d len=%d\n"
	    "td0:%08x\ntd1:%08x\ntd2:%08x\ntd3:%08x\n"
	    "cd0:%08x\ncd1:%08x\npl0:%08x\npl1:%08x\n",
	    dp->name, __func__, code, cmdsz,
	    ((uint32_t *)(txd))[0],
	    ((uint32_t *)(txd))[1],
	    ((uint32_t *)(txd))[2],
	    ((uint32_t *)(txd))[3],

	    ((uint32_t *)(cmd))[0],
	    ((uint32_t *)(cmd))[1],

	    ((uint32_t *)(mp->b_rptr + sizeof (*txd) + sizeof (*cmd)))[0],
	    ((uint32_t *)(mp->b_rptr + sizeof (*txd) + sizeof (*cmd)))[1]));

	/* XXX - it seems we can use any bulk-out pipes to issue fw commands. */

	return (uwgem_bulk_out(dp, mp, lp->qid2idx[RSU_QID_H2C], 0));
}

#if 0
static void
rsu_calib_to(void *arg)
{
	struct uwgem_dev *sc = arg;

	/* Do it in a process context. */
	rsu_do_async(dp, rsu_calib_cb, NULL, 0);
}

/* ARGSUSED */
static void
rsu_calib_cb(struct uwgem_dev *sc, void *arg)
{
	uint32_t reg;

#ifdef notyet
	/* Read WPS PBC status. */
	rsu_write_1(dp, R92S_MAC_PINMUX_CTRL,
	    R92S_GPIOMUX_EN | SM(R92S_GPIOSEL_GPIO, R92S_GPIOSEL_GPIO_JTAG));
	rsu_write_1(dp, R92S_GPIO_IO_SEL,
	    rsu_read_1(dp, R92S_GPIO_IO_SEL) & ~R92S_GPIO_WPS);
	reg = rsu_read_1(dp, R92S_GPIO_CTRL);
	if (reg != 0xff && (reg & R92S_GPIO_WPS))
		DPRINTF(0, (CE_CONT, "WPS PBC is pushed\n"));
#endif
	/* Read current signal level. */
	if (rsu_fw_iocmd(dp, 0xf4000001) == USB_SUCCESS) {
		reg = rsu_read_4(dp, R92S_IOCMD_DATA);
		DPRINTF(1, (CE_CONT, "RSSI=%d%%\n", reg >> 4));
	}

	timeout_add_sec(&sc->calib_to, 2);
}
#endif

static int
rsu_newstate(struct uwgem_dev *dp, enum ieee80211_state nstate, int arg)
{
	struct ieee80211com	*ic = &dp->ic;
	struct rsu_dev		*lp = dp->private;
	enum ieee80211_state	ostate;
	struct ieee80211_node	*ni;
	int			error, s;
	int			i;

	ostate = ic->ic_state;
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: %s(%x) -> %s(%x), chan:%d, arg:0x%x", dp->name, __func__,
	    ieee80211_state_name[ostate], ostate,
	    ieee80211_state_name[nstate], nstate,
	    ieee80211_chan2ieee(ic, ic->ic_curchan), arg));

	if (ostate == IEEE80211_S_RUN) {
#if 0
		/* Stop calibration. */
		timeout_del(&sc->calib_to);
#endif
		/* Disassociate from our current BSS. */
		(void)rsu_disconnect(dp);
	}

	if (ostate == IEEE80211_S_SCAN) {
		if (nstate != IEEE80211_S_SCAN) {
			mutex_enter(&lp->scan_done_lock);
			while (lp->scan_pass != 0) {
				cv_timedwait(&lp->cv_scan_done,
				    &lp->scan_done_lock,
				    ddi_get_lbolt() + drv_usectohz(100000));
			}
			mutex_exit(&lp->scan_done_lock);
		}
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
#if 0
		/* XXX - no need to reset hardware */
		if (ostate != IEEE80211_S_INIT) {
			rsu_stop_chip(dp);
			rsu_init_chip(dp);
		}
#endif
#if 0
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++) {
			DPRINTF(0, (CE_CONT, "%s:%s: key ix%d", 
		    dp->name, __func__, i));
#ifdef CONFIG_CRYPTO_HW
			if (ic->ic_nw_keys[i].wk_keylen != 0) {
				/* valid key */
				rsu_set_key(dp, NULL, &ic->ic_nw_keys[i]);
			} else
#endif
			{
				rsu_delete_key(dp, i);
			}
		}
#endif
		break;

	case IEEE80211_S_SCAN:
		if (ostate != IEEE80211_S_SCAN) {
			/* start scanning */
			mutex_enter(&lp->scan_done_lock);
			while (lp->scan_pass != 0) {
				cv_timedwait(&lp->cv_scan_done,
				    &lp->scan_done_lock,
				    ddi_get_lbolt() + drv_usectohz(100000));
			}
			lp->scan_pass++;
			mutex_exit(&lp->scan_done_lock);

			error = rsu_site_survey(dp);
			if (error != USB_SUCCESS) {
				cmn_err(CE_CONT,
				    "!%s: could not send site survey command",
				    dp->name);

				/* stop scanner thread */
				mutex_enter(&lp->scan_done_lock);
				lp->scan_pass = 0;
				cv_signal(&lp->cv_scan_done);
				mutex_exit(&lp->scan_done_lock);
			}
		}
		break;

	case IEEE80211_S_AUTH:
#if 1
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++) {
			DPRINTF(0, (CE_CONT, "%s:%s: key ix%d", 
		    dp->name, __func__, i));
			if (ic->ic_nw_keys[i].wk_keylen != 0) {
				/* valid key */
				rsu_set_key(dp, NULL, &ic->ic_nw_keys[i]);
			} else {
				rsu_delete_key(dp, i);
			}
		}
#endif
#if 0
		uwgem_dump_ie(ic->ic_bss->in_wpa_ie);
#endif
		error = rsu_join_bss(dp, ic->ic_bss);
		if (error != USB_SUCCESS) {
			cmn_err(CE_CONT, "!%s: could not send join command",
			    dp->name);
#if 0
			ieee80211_begin_scan(&ic->ic_if);
#endif
			break;
		}
		break;

	case IEEE80211_S_ASSOC:
		break;

	case IEEE80211_S_RUN:
		/* Indicate highest supported rate. */
		ic->ic_bss->in_txrate = ic->ic_bss->in_rates.ir_nrates - 1;
#if 0
		/* Start periodic calibration. */
		timeout_add_sec(&sc->calib_to, 2);
#endif
		break;
	}
x:
	return (error);
}

int
rsu_set_key(struct uwgem_dev *dp, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct ieee80211com	*ic = (struct ieee80211com *)dp;
	struct r92s_fw_cmd_set_key	key;

#if 0
	/* Defer setting of WEP keys until interface is brought up. */
	if ((ic->ic_if.if_flags & (IFF_UP | IFF_RUNNING)) !=
	    (IFF_UP | IFF_RUNNING))
		return (0);
#endif

	memset(&key, 0, sizeof (key));

	/* Map net80211 cipher to HW crypto algorithm. */

	switch (k->wk_cipher->ic_cipher) {
	case IEEE80211_CIPHER_WEP:
		if (k->wk_keylen > 40/8 ) {
			key.algo = R92S_KEY_ALGO_WEP104;
		} else {
			key.algo = R92S_KEY_ALGO_WEP40;
		}
		break;

	case IEEE80211_CIPHER_TKIP:	/* WPA */
		key.algo = R92S_KEY_ALGO_TKIP;
		break;

	case IEEE80211_CIPHER_AES_CCM:	/* WPA2 */
		key.algo = R92S_KEY_ALGO_AES;
		break;

	default:
		return (0);
	}

	key.id = k->wk_keyix;	/* was wk_id */
	key.grpkey = (k->wk_flags & IEEE80211_KEY_GROUP) != 0;
	memcpy(key.key, k->wk_key, min(k->wk_keylen, sizeof (key.key)));
	DPRINTF(0, (CE_CONT, "!%s: %s id:%d len:%d key:%02x %02x %02x %02x %02x",
	    dp->name, __func__,
	    key.id, k->wk_keylen,
	    k->wk_key[0], k->wk_key[1], k->wk_key[2],
	    k->wk_key[3], k->wk_key[4]));

	(void)rsu_fw_cmd(dp, R92S_CMD_SET_KEY, &key, sizeof (key));

	return (0);
}

/* ARGSUSED */
void
rsu_delete_key(struct uwgem_dev *dp, int keyix)
{
	struct ieee80211com	*ic = (struct ieee80211com *)dp;
	struct r92s_fw_cmd_set_key key;

	memset(&key, 0, sizeof (key));
	key.id = keyix;	

	(void)rsu_fw_cmd(dp, R92S_CMD_SET_KEY, &key, sizeof (key));
}

int
rsu_site_survey(struct uwgem_dev *dp)
{
	struct rsu_dev	*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct r92s_fw_cmd_sitesurvey	cmd;

	memset(&cmd, 0, sizeof (cmd));
	if (ic->ic_flags & IEEE80211_F_ASCAN) {
		/* do active scan */
		cmd.active = htole32(1);
	}

	cmd.limit = htole32(48);

	if (cmd.active && ic->ic_des_esslen) {
		/* Do a directed scan for second pass. */
		DPRINTF(0, (CE_CONT, "!%s: %s: Do a directed scan for %s",
		    dp->name, __func__, ic->ic_des_essid));
		cmd.ssidlen = htole32(ic->ic_des_esslen);
		memcpy(cmd.ssid, ic->ic_des_essid, ic->ic_des_esslen);
	}
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: sending site survey command, pass=%d",
	    dp->name, __func__, lp->scan_pass));
	return (rsu_fw_cmd(dp, R92S_CMD_SITE_SURVEY, &cmd, sizeof (cmd)));
}

extern uint8_t *ieee80211_add_wme_info(uint8_t *frm, struct ieee80211_wme_state *wme) ;

#ifdef notdef

/*
 * Add a QoS Capability element to a frame (see 7.3.2.35).
 */
static uint8_t *
ieee80211_add_qos_capability(uint8_t *frm, struct ieee80211com *ic)
{
#define	IEEE80211_ELEMID_QOS_CAP	46
	*frm++ = IEEE80211_ELEMID_QOS_CAP;
	*frm++ = 1;
	*frm++ = 0;	/* QoS Info */
	return frm;
}

/*
 * Add an RSN element to a frame (see 7.3.2.25).
 */

#define IEEE80211_OUI   ((const uint8_t[]){ 0x00, 0x0f, 0xac })
#define MICROSOFT_OUI   ((const uint8_t[]){ 0x00, 0x50, 0xf2 })

static uint8_t *
ieee80211_add_rsn_body(uint8_t *frm, struct ieee80211com *ic,
    const struct ieee80211_node *ni, int wpa)
{
	const uint8_t *oui = wpa ? MICROSOFT_OUI : IEEE80211_OUI;
	uint8_t *pcount;
	uint16_t count;

	/* write Version field */
	LE_WRITE_2(frm, 1); frm += 2;

	/* write Group Cipher Suite field (see Table 20da) */
	memcpy(frm, oui, 3); frm += 3;
	switch (ni->ni_rsngroupcipher) {
	case IEEE80211_CIPHER_WEP40:
		*frm++ = 1;
		break;
	case IEEE80211_CIPHER_TKIP:
		*frm++ = 2;
		break;
	case IEEE80211_CIPHER_CCMP:
		*frm++ = 4;
		break;
	case IEEE80211_CIPHER_WEP104:
		*frm++ = 5;
		break;
	default:
		/* can't get there */
		panic("invalid group cipher!");
	}

	pcount = frm; frm += 2;
	count = 0;
	/* write Pairwise Cipher Suite List */
	if (ni->ni_rsnciphers & IEEE80211_CIPHER_USEGROUP) {
		memcpy(frm, oui, 3); frm += 3;
		*frm++ = 0;
		count++;
	}
	if (ni->ni_rsnciphers & IEEE80211_CIPHER_TKIP) {
		memcpy(frm, oui, 3); frm += 3;
		*frm++ = 2;
		count++;
	}
	if (ni->ni_rsnciphers & IEEE80211_CIPHER_CCMP) {
		memcpy(frm, oui, 3); frm += 3;
		*frm++ = 4;
		count++;
	}
	/* write Pairwise Cipher Suite Count field */
	LE_WRITE_2(pcount, count);

	pcount = frm; frm += 2;
	count = 0;
	/* write AKM Suite List (see Table 20dc) */
	if (ni->ni_rsnakms & IEEE80211_AKM_IEEE8021X) {
		memcpy(frm, oui, 3); frm += 3;
		*frm++ = 1;
		count++;
	}
	if (ni->ni_rsnakms & IEEE80211_AKM_PSK) {
		memcpy(frm, oui, 3); frm += 3;
		*frm++ = 2;
		count++;
	}
	/* write AKM Suite List Count field */
	LE_WRITE_2(pcount, count);

	if (!wpa) {
		/* write RSN Capabilities field */
		LE_WRITE_2(frm, ni->ni_rsncaps); frm += 2;

		/* no PMKID List for now */
	}
	return frm;
}

static uint8_t *
ieee80211_add_rsn(uint8_t *frm, struct ieee80211com *ic,
    const struct ieee80211_node *ni)
{
	uint8_t *plen;

	*frm++ = IEEE80211_ELEMID_RSN;
	plen = frm++;	/* length filled in later */
	frm = ieee80211_add_rsn_body(frm, ic, ni, 0);

	/* write length field */
	*plen = frm - plen - 1;
	return frm;
}

/*
 * Add a vendor-specific WPA element to a frame.
 * This is required for compatibility with Wi-Fi Alliance WPA.
 */
static uint8_t *
ieee80211_add_wpa(uint8_t *frm, struct ieee80211com *ic,
    const struct ieee80211_node *ni)
{
	uint8_t *plen;

	*frm++ = IEEE80211_ELEMID_VENDOR;
	plen = frm++;	/* length filled in later */
	memcpy(frm, MICROSOFT_OUI, 3); frm += 3;
	*frm++ = 1;	/* WPA */
	frm = ieee80211_add_rsn_body(frm, ic, ni, 1);

	/* write length field */
	*plen = frm - plen - 1;
	return frm;
}
#endif /* notyet */

static uint16_t
rsu_get_capinfo(ieee80211com_t *ic)
{
	uint16_t capinfo;

	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan)) {
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	}
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;

	return (capinfo);
}

static int
rsu_join_bss(struct uwgem_dev *dp, struct ieee80211_node *ni)
{
	struct rsu_dev	*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct ndis_wlan_bssid_ex	*bss;
	struct ndis_802_11_fixed_ies	*fixed;
	struct r92s_fw_cmd_auth	auth;
	uint8_t		buf[sizeof (*bss) + 128], *frm;
	uint8_t		opmode;
	int		error;
	uint16_t	cap;

	/* Let the FW decide the opmode based on the capinfo field. */
	opmode = NDIS802_11AUTOUNKNOWN;
	DPRINTF(0, (CE_CONT, "!%s: %s: setting operating mode to %d",
	    dp->name, __func__, opmode));

	error = rsu_fw_cmd(dp, R92S_CMD_SET_OPMODE, &opmode, sizeof (opmode));
	if (error != USB_SUCCESS) {
		return (error);
	}

	memset(&auth, 0, sizeof (auth));
	if (ic->ic_flags & IEEE80211_F_WPA) {
		auth.mode = R92S_AUTHMODE_WPA;
		auth.dot1x = (ic->ic_bss->in_authmode == IEEE80211_AUTH_8021X);
	} else {
		auth.mode = R92S_AUTHMODE_OPEN;
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: setting auth mode to %d",
	    dp->name, __func__, auth.mode));

	error = rsu_fw_cmd(dp, R92S_CMD_SET_AUTH, &auth, sizeof (auth));
	if (error != USB_SUCCESS) {
		return (error);
	}

	memset(buf, 0, sizeof (buf));
	bss = (struct ndis_wlan_bssid_ex *)buf;
	IEEE80211_ADDR_COPY(bss->macaddr, ni->in_bssid);
	bss->ssid.ssidlen = htole32(ni->in_esslen);
	memcpy(bss->ssid.ssid, ni->in_essid, ni->in_esslen);
	if (ic->ic_flags & (IEEE80211_F_PRIVACY | IEEE80211_F_WPA)) {
		bss->privacy = htole32(1);
	}
	bss->rssi = htole32(ni->in_rssi);
	if (ic->ic_curmode == IEEE80211_MODE_11B) {
		bss->networktype = htole32(NDIS802_11DS);
	} else {
		bss->networktype = htole32(NDIS802_11OFDM24);
	}
	bss->config.len = htole32(sizeof (bss->config));
	bss->config.bintval = htole32(ni->in_intval);
	bss->config.dsconfig = htole32(ieee80211_chan2ieee(ic, ni->in_chan));
	bss->inframode = htole32(NDIS802_11INFRASTRUCTURE);
	memcpy(bss->supprates, ni->in_rates.ir_rates,
	    ni->in_rates.ir_nrates);

	/* Write the fixed fields of the beacon frame. */
	fixed = (struct ndis_802_11_fixed_ies *)&bss[1];
	memcpy(&fixed->tstamp, &ni->in_tstamp, 8);
	fixed->bintval = htole16(ic->ic_lintval);
	/* fixed->bintval = htole16(ni->in_intval); */

	cap = rsu_get_capinfo(ic);
	fixed->capabilities = htole16(cap);
	/* fixed->capabilities = htole16(ni->in_capinfo); */

	/* Write IEs to be included in the association request. */
	frm = (uint8_t *)&fixed[1];

	/* XXX - RSN IE/WPA IE will come from opt ie */

#ifdef notyet
	if (ni->in_flags & IEEE80211_NODE_QOS) {
		frm = ieee80211_add_qos_capability(frm, ic);
	}
#endif
	if ((ic->ic_flags & IEEE80211_F_WME) && ni->in_wme_ie != NULL) {
		frm = ieee80211_add_wme_info(frm, &ic->ic_wme);
	}
#ifdef CONFIG_11N
	if ((ic->ic_flags_ext & IEEE80211_FEXT_HT) &&
	    ni->in_htcap_ie != NULL) {
		if (ni->in_htcap_ie[0] == IEEE80211_ELEMID_HTCAP) {
			DPRINTF(0, (CE_CONT, "!%s: %s join w/ htcap",
			    dp->name, __func__));
			frm = ieee80211_add_htcap(frm, ic->ic_bss);
		}
		if (ni->in_htcap_ie[0] == IEEE80211_ELEMID_VENDOR) {
			frm = ieee80211_add_htcap_vendor(frm, ic->ic_bss);
		}
	}
#endif
	if (ic->ic_opt_ie != NULL) {
		bcopy(ic->ic_opt_ie, frm, ic->ic_opt_ie_len);
		frm += ic->ic_opt_ie_len;
#if 0
		uwgem_dump_ie(ic->ic_opt_ie);
#endif
	}

	bss->ieslen = htole32(frm - (uint8_t *)fixed);
	bss->len = htole32(((frm - buf) + 3) & ~3);

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: "
	    "sending join bss command to %s chan %d (opt_ie:%p len:%d)",
	    dp->name, __func__,
	    ether_sprintf((void *)bss->macaddr), letoh32(bss->config.dsconfig),
	    ic->ic_opt_ie, ic->ic_opt_ie_len));

	return (rsu_fw_cmd(dp, R92S_CMD_JOIN_BSS, buf, frm - buf));
}

#ifdef CONFIG_NEW_JOIN
static int
rsu_join_bss_ie(struct uwgem_dev *dp, uint8_t *assoc_ie)
{
	struct rsu_dev	*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_node	*ni = ic->ic_bss;
	struct ndis_wlan_bssid_ex	*bss;
	struct ndis_802_11_fixed_ies	*fixed;
	struct r92s_fw_cmd_auth	auth;
	uint8_t		buf[sizeof (*bss) + 128], *frm;
	uint8_t		opmode;
	int		error;

	/* Let the FW decide the opmode based on the capinfo field. */
	opmode = NDIS802_11AUTOUNKNOWN;
	DPRINTF(0, (CE_CONT, "!%s: %s: setting operating mode to %d",
	    dp->name, __func__, opmode));

	error = rsu_fw_cmd(dp, R92S_CMD_SET_OPMODE, &opmode, sizeof (opmode));
	if (error != USB_SUCCESS) {
		return (error);
	}

	memset(&auth, 0, sizeof (auth));
	if (ic->ic_flags & IEEE80211_F_WPA) {
		auth.mode = R92S_AUTHMODE_WPA;
		auth.dot1x = (ic->ic_bss->in_authmode == IEEE80211_AUTH_8021X);
	} else {
		auth.mode = R92S_AUTHMODE_OPEN;
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: setting auth mode to %d",
	    dp->name, __func__, auth.mode));

	error = rsu_fw_cmd(dp, R92S_CMD_SET_AUTH, &auth, sizeof (auth));
	if (error != USB_SUCCESS) {
		return (error);
	}

	memset(buf, 0, sizeof (buf));
	bss = (struct ndis_wlan_bssid_ex *)buf;
	IEEE80211_ADDR_COPY(bss->macaddr, ni->in_bssid);
	bss->ssid.ssidlen = htole32(ni->in_esslen);
	memcpy(bss->ssid.ssid, ni->in_essid, ni->in_esslen);
	if (ic->ic_flags & (IEEE80211_F_PRIVACY | IEEE80211_F_WPA)) {
		bss->privacy = htole32(1);
	}
	bss->rssi = htole32(ni->in_rssi);
	if (ic->ic_curmode == IEEE80211_MODE_11B) {
		bss->networktype = htole32(NDIS802_11DS);
	} else {
		bss->networktype = htole32(NDIS802_11OFDM24);
	}
	bss->config.len = htole32(sizeof (bss->config));
	bss->config.bintval = htole32(ni->in_intval);
	bss->config.dsconfig = htole32(ieee80211_chan2ieee(ic, ni->in_chan));
	bss->inframode = htole32(NDIS802_11INFRASTRUCTURE);
	memcpy(bss->supprates, ni->in_rates.ir_rates,
	    ni->in_rates.ir_nrates);

	/* Write the fixed fields of the beacon frame. */
	fixed = (struct ndis_802_11_fixed_ies *)&bss[1];
	src = &assoc_ie[sizeof (struct ieee80211header)];

	memcpy(&fixed->tstamp, &ni->in_tstamp, 8);
	fixed->bintval = LE_16(src[3] << 16 | src[2]);
	fixed->capabilities = LE_16(src[1] << 16 | src[0]);

	/* Write IEs to be included in the association request. */
	frm = (uint8_t *)&fixed[1];

#ifdef notyet
	if (ni->in_flags & IEEE80211_NODE_QOS) {
		frm = ieee80211_add_qos_capability(frm, ic);
	}
#endif
	if ((ic->ic_flags & IEEE80211_F_WME) && ni->in_wme_ie != NULL) {
		frm = ieee80211_add_wme_info(frm, &ic->ic_wme);
	}
#ifdef CONFIG_11N
	if ((ic->ic_flags_ext & IEEE80211_FEXT_HT) &&
	    ni->in_htcap_ie != NULL &&
	    ni->in_htcap_ie[0] == IEEE80211_ELEMID_HTCAP) {
		DPRINTF(0, (CE_CONT, "!%s: %s join w/ htcap",
		    dp->name, __func__));
		frm = ieee80211_add_htcap(frm, ic->ic_bss);
	}
	if ((ic->ic_flags_ext & IEEE80211_FEXT_HT) &&
	    ni->in_htcap_ie != NULL &&
	    ni->in_htcap_ie[0] == IEEE80211_ELEMID_VENDOR) {
		frm = ieee80211_add_htcap_vendor(frm, ic->ic_bss);
	}
	if (ic->ic_opt_ie != NULL) {
		bcopy(ic->ic_opt_ie, frm, ic->ic_opt_ie_len);
		frm += ic->ic_opt_ie_len;
#if 0
		uwgem_dump_ie(ic->ic_opt_ie);
#endif
	}
#endif
	bss->ieslen = htole32(frm - (uint8_t *)fixed);
	bss->len = htole32(((frm - buf) + 3) & ~3);
	DPRINTF(0, (CE_CONT,
	    "!%s: %s: sending join bss command to %s chan %d (opt_ie:%p len:%d)",
	    dp->name, __func__,
	    ether_sprintf((void *)bss->macaddr), letoh32(bss->config.dsconfig),
	    ic->ic_opt_ie, ic->ic_opt_ie_len));
	return (rsu_fw_cmd(dp, R92S_CMD_JOIN_BSS, buf, frm - buf));
}
#endif /* NEW_JOIN */

static int
rsu_disconnect(struct uwgem_dev *dp)
{
	uint32_t zero = 0;	/* :-) */

	/* Disassociate from our current BSS. */
	DPRINTF(0, (CE_CONT, "sending disconnect command\n"));
	return (rsu_fw_cmd(dp, R92S_CMD_DISCONNECT, &zero, sizeof (zero)));
}

static mblk_t *
rsu_ndis_bssid_to_80211_mgt(struct uwgem_dev *dp, uint8_t *buf, int len, int subtype)
{
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_frame	*wh;
	struct ndis_wlan_bssid_ex *bss;
	mblk_t		*mp = NULL;
	int		pktlen;
	uint32_t        rate;
	uint32_t        rssi;

	mp = NULL;

	if (len < sizeof (*bss)) {
		goto x;
	}

	bss = (struct ndis_wlan_bssid_ex *)buf;
	if (len < sizeof (*bss) + letoh32(bss->ieslen)) {
		goto x;
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: found BSS %s: len=%d chan=%d inframode=%d "
	    "networktype=%d privacy=%d iesslen:%d",
	    dp->name, __func__,
	    ether_sprintf((void *)bss->macaddr), letoh32(bss->len),
	    letoh32(bss->config.dsconfig), letoh32(bss->inframode),
	    letoh32(bss->networktype), letoh32(bss->privacy),
	    letoh32(bss->ieslen)));

	/* Build a fake beacon frame to let net80211 do all the parsing. */
	pktlen = sizeof (*wh) + letoh32(bss->ieslen);

	mp = allocb(pktlen, 0);
	if (mp == NULL) {
		goto x;
	}

	wh = (struct ieee80211_frame *)mp->b_rptr;
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT | subtype;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(uint16_t *)wh->i_dur = 0;
	if (subtype == IEEE80211_FC0_SUBTYPE_BEACON) {
		IEEE80211_ADDR_COPY(wh->i_addr1, (void *)&rsu_broadcastaddr);
	} else {
		IEEE80211_ADDR_COPY(wh->i_addr1, ic->ic_macaddr);
	}
	IEEE80211_ADDR_COPY(wh->i_addr2, bss->macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, bss->macaddr);
	*(uint16_t *)wh->i_seq = 0;
	memcpy(&wh[1], (uint8_t *)&bss[1], letoh32(bss->ieslen));

	mp->b_wptr = mp->b_rptr + pktlen;

	rssi = letoh32(bss->rssi);
	rate = 0;

	mp->b_cont = (void *)(uintptr_t)rssi;
	mp->b_prev = (void *)(uintptr_t)rate;

	uwgem_dump_pkt(__func__, mp->b_rptr, mp->b_wptr - mp->b_rptr, rate, rssi);
x:
	return (mp);
}

static mblk_t *
rsu_event_survey(struct uwgem_dev *dp, uint8_t *buf, int len)
{
	mblk_t	*mp = NULL;
#if 0
	struct rsu_dev		*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_node	*ni;
	struct ieee80211_frame	*wh;
	struct ndis_wlan_bssid_ex *bss;
	int	pktlen;
	uint32_t        rate;
	uint32_t        rssi;

	if (len < sizeof (*bss)) {
		return (mp);
	}

	bss = (struct ndis_wlan_bssid_ex *)buf;
	if (len < sizeof (*bss) + letoh32(bss->ieslen)) {
		return (mp);
	}

	DPRINTF(1, (CE_CONT,
	    "!%s: %s: found BSS %s: len=%d chan=%d inframode=%d "
	    "networktype=%d privacy=%d iesslen:%d",
	    dp->name, __func__,
	    ether_sprintf((void *)bss->macaddr), letoh32(bss->len),
	    letoh32(bss->config.dsconfig), letoh32(bss->inframode),
	    letoh32(bss->networktype), letoh32(bss->privacy),
	    letoh32(bss->ieslen)));

	/* Build a fake beacon frame to let net80211 do all the parsing. */
	pktlen = sizeof (*wh) + letoh32(bss->ieslen);

	mp = allocb(pktlen, 0);
	if (mp == NULL) {
		return (mp);
	}

	wh = (struct ieee80211_frame *) mp->b_rptr;
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(uint16_t *)wh->i_dur = 0;
	IEEE80211_ADDR_COPY(wh->i_addr1, (void *)&rsu_broadcastaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, bss->macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, bss->macaddr);
	*(uint16_t *)wh->i_seq = 0;
	memcpy(&wh[1], (uint8_t *)&bss[1], letoh32(bss->ieslen));

	mp->b_wptr = mp->b_rptr + pktlen;
#if 0
	ni = ieee80211_find_rxnode(ic, wh);
	rxi.rxi_flags = 0;
	rxi.rxi_rssi = letoh32(bss->rssi);
	rxi.rxi_tstamp = 0;
	ieee80211_input(ifp, m, ni, &rxi);
	/* Node is no longer needed. */
	ieee80211_release_node(ic, ni);
#elif 0
	/* grab a reference to the source node */
	ni = ieee80211_find_rxnode(ic, wh);
	if (ni) {

		rssi = letoh32(bss->rssi);
		rate = 0;
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
#else
	rssi = letoh32(bss->rssi);
	rate = 0;
#endif
	mp->b_cont = (void *)(uintptr_t)rssi;
	mp->b_prev = (void *)(uintptr_t)rate;

	uwgem_dump_pkt(__func__, mp->b_rptr, mp->b_wptr - mp->b_rptr, rate, rssi);
#else
	mp = rsu_ndis_bssid_to_80211_mgt(dp, buf, len, IEEE80211_FC0_SUBTYPE_BEACON);
#endif
	return (mp);
}

static mblk_t *
rsu_event_join_bss(struct uwgem_dev *dp, uint8_t *buf, int len)
{
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_node	*ni = ic->ic_bss;
	struct r92s_event_join_bss	*rsp;
	int	res;
	mblk_t	*mp = NULL;

	if (len < sizeof (*rsp)) {
		return (NULL);
	}
	rsp = (struct r92s_event_join_bss *)buf;
	res = (int)letoh32(rsp->join_res);

	DPRINTF(0, (CE_CONT, "!%s: %s: Rx join BSS event len=%d res=%d",
	    dp->name, __func__, len, res));
	if (res <= 0) {
#if 0
		ic->ic_stats.is_rx_auth_fail++;
#endif
		cmn_err(CE_CONT, "%s: %s: failed", dp->name, __func__);
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
		return (NULL);
	}
	DPRINTF(0, (CE_CONT, "!%s: %s: associated with %s associd=%d",
	    dp->name, __func__,
	    ether_sprintf((void *)rsp->bss.macaddr), letoh32(rsp->associd)));
#if 0
	/* XXX -- what does it means 0xc000? */
	ni->in_associd = letoh32(rsp->associd) | 0xc000;
#else
	ni->in_associd = letoh32(rsp->associd);
#endif
#if 0
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		ni->ni_flags |= IEEE80211_NODE_TXRXPROT;
	}
#endif

	ieee80211_new_state(ic, IEEE80211_S_ASSOC,
	    IEEE80211_FC0_SUBTYPE_AUTH);
#if 1
	ieee80211_new_state(ic, IEEE80211_S_RUN,
	    IEEE80211_FC0_SUBTYPE_ASSOC_RESP);
#else
/*
	mp = rsu_ndis_bssid_to_80211_mgt(dp, (uint8_t *)&rsp->bss,
	    len - (((uint8_t *)&rsp->bss) - ((uint8_t *)rsp)), IEEE80211_FC0_SUBTYPE_ASSOC_RESP);
*/
{
	struct ieee80211_frame *wh;
	int	pktlen;
	uint8_t	*frm;

        /* Build a fake beacon frame to let net80211 do all the parsing. */
        pktlen = sizeof (*wh) + 2*3 + letoh32(rsp->bss.ieslen);

        mp = allocb(pktlen, 0);
        if (mp == NULL) {
                return (mp);
        }

        wh = (struct ieee80211_frame *) mp->b_rptr;
        wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
            IEEE80211_FC0_SUBTYPE_ASSOC_RESP;
        wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
        *(uint16_t *)wh->i_dur = 0;
        IEEE80211_ADDR_COPY(wh->i_addr1, ic->ic_macaddr);
        IEEE80211_ADDR_COPY(wh->i_addr2, rsp->bss.macaddr);
        IEEE80211_ADDR_COPY(wh->i_addr3, rsp->bss.macaddr);
        *(uint16_t *)wh->i_seq = 0;
	frm = (uint8_t *)&wh[1];
	frm[0] = 0;	/* capability info */
	frm[1] = 0;	/* capability info */
	frm[2] = res > 0 ? 0 : 1;	/* status */
	frm[3] = 0;	/* status */
	frm[4] = letoh32(rsp->associd);
	frm[5] = letoh32(rsp->associd) >> 8 ;

        memcpy(frm + 6, (uint8_t *)&rsp[1], letoh32(rsp->bss.ieslen));

        mp->b_wptr = mp->b_rptr + pktlen;

	mp->b_wptr = frm + 6;	/* debug */
}
#endif

	return (mp);
}

static mblk_t *
rsu_rx_event(struct uwgem_dev *dp, uint8_t code, uint8_t *buf, int len)
{
	struct rsu_dev *lp = dp->private;
	struct ieee80211com *ic = &dp->ic;
	mblk_t	*mp_ret = NULL;

	DPRINTF(10, (CE_CONT, "!%s: %s: Rx event code=%d len=%d",
	    dp->name, __func__, code, len));

	switch (code) {
	case R92S_EVT_SURVEY:
		if (ic->ic_state == IEEE80211_S_SCAN) {
			/* XXX - this routine just make beacons */
			mp_ret = rsu_event_survey(dp, buf, len);

		}
		break;

	case R92S_EVT_SURVEY_DONE:
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: site survey pass %d done, found %d BSS",
		    dp->name, __func__,
		    lp->scan_pass, letoh32(*(uint32_t *)buf)));

		/* scan done */
		mutex_enter(&lp->scan_done_lock);
		lp->scan_pass = 0;
		mutex_exit(&lp->scan_done_lock);
		cv_signal(&lp->cv_scan_done);
#ifdef notdef
		if (ic->ic_state == IEEE80211_S_SCAN) {
			/* no need to terminate scan sequece by ourselves */
			ieee80211_end_scan(ic);
		}
#endif

		break;

	case R92S_EVT_JOIN_BSS:
		if (ic->ic_state == IEEE80211_S_AUTH) {
			mp_ret = rsu_event_join_bss(dp, buf, len);
		}
		break;

	case R92S_EVT_DEL_STA:
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: disassociated from %s",
		    dp->name, __func__, ether_sprintf((void *)buf)));
		if (ic->ic_state == IEEE80211_S_RUN &&
		    IEEE80211_ADDR_EQ(ic->ic_bss->in_bssid, buf))
			ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
		break;

	case R92S_EVT_WPS_PBC:
		DPRINTF(0, (CE_CONT, "WPS PBC pushed."));
		break;
#ifdef CONFIG_AMPDU
	case R92S_EVT_ADDBA_REQ_REPORT:
{
/*
		struct ADDBA_Req_Report_parm {
			uint8_t		MacAddress[ETHERADDRL];
			uint16_t	StartSeqNum;
			uint8_t		tid;
		};
*/
		struct ieee80211_frame	wh;
		struct ieee80211_node	*in;
		int	tid;
		int	seq;

		seq = buf[7] << 8 | buf[6];
		tid =  buf[8];

		DPRINTF(0, (CE_CONT,
		    "!%s: %s: addba_req_report from:%s start_seq_num:%d, tid:%d",
		    dp->name, __func__,
		    ether_sprintf((void *)buf), seq, tid));

		bzero(&wh, sizeof (wh));
		wh.i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT;
		IEEE80211_ADDR_COPY(&wh.i_addr1, ic->ic_macaddr);
		IEEE80211_ADDR_COPY(&wh.i_addr2, buf);
		in = ieee80211_find_rxnode(ic, &wh);
		if (in) {
			ic->ic_addba_response(in,
			    /* tap */ &in->in_tx_ampdu[TID_TO_WME_AC(tid)],
			    /* code */ IEEE80211_STATUS_SUCCESS,
			    /* baparamset */ 0,
			    /* batimeout */ 0);
			 /* node is no longer needed */
			ieee80211_free_node(in);
		}
}
		break;
#endif
	case R92S_EVT_FWDBG:
#ifdef DEBUG_LEVEL
		buf[len-1] = '\0';
		cmn_err(CE_CONT, "!%s: %s: FWLOG: %s",
		    dp->name, __func__, (char *)buf);
#endif
		break;

	default:
		cmn_err(CE_CONT, "!%s: %s: unkown rx event code=%d len=%d",
		    dp->name, __func__, code, len);
		break;
	}

	return (mp_ret);
}

static mblk_t *
rsu_rx_multi_event(struct uwgem_dev *dp, mblk_t	*mp_head)
{
	uint8_t		*buf = mp_head->b_rptr;
	int		len = mp_head->b_wptr - mp_head->b_rptr;
	struct r92s_fw_cmd_hdr *cmd;
	int		cmdsz;
	mblk_t		*mp_ret;
	mblk_t		**mp_tail = &mp_ret;
	mblk_t		*mp;

	DPRINTF(10, (CE_CONT, "Rx events len=%d\n", len));

	/* Skip Rx status. */
	buf += sizeof (struct r92s_rx_stat);
	len -= sizeof (struct r92s_rx_stat);

	/* Process all events. */
	for (;;) {
		/* Check that command header fits. */
		if (len < sizeof (*cmd)) {
			break;
		}
		cmd = (struct r92s_fw_cmd_hdr *)buf;
		/* Check that command payload fits. */
		cmdsz = letoh16(cmd->len);
		if (len < sizeof (*cmd) + cmdsz) {
			break;
		}

		/* Process firmware event. */
		mp = rsu_rx_event(dp, cmd->code, (uint8_t *)&cmd[1], cmdsz);
		if (mp != NULL) {
			*mp_tail = mp;
			mp_tail = &mp->b_next;
		}

		if (!(cmd->seq & R92S_FW_CMD_MORE)) {
			/* done */
			break;
		}

		buf += sizeof (*cmd) + cmdsz;
		len -= sizeof (*cmd) + cmdsz;
	}

	*mp_tail = NULL;
	return (mp_ret);
}

static int8_t
rsu_get_rssi(struct uwgem_dev *dp, int rate, void *physt)
{
	static const int8_t cckoff[] = { 14, -2, -20, -40 };
	struct r92s_rx_cck *cck;
	struct r92s_rx_phystat *phy;
	uint8_t	rpt;
	int8_t	rssi;

	if (rate <= 3) {
		cck = (struct r92s_rx_cck *)physt;
		rpt = (cck->agc_rpt >> 6) & 0x3;
		rssi = cck->agc_rpt & 0x3e;
		rssi = cckoff[rpt] - rssi;
	} else {
		/* OFDM/HT. */
		phy = (struct r92s_rx_phystat *)physt;
		rssi = ((letoh32(phy->phydw1) >> 1) & 0x7f) - 106;
	}
	return (rssi);
}

static mblk_t *
rsu_rx_frame(struct uwgem_dev *dp, uint8_t *buf, size_t pktlen,
    mblk_t *mp_head, mblk_t *mp)
{
	struct ieee80211com *ic = &dp->ic;
#if 0
	struct ieee80211_rxinfo	rxi;
#endif
	struct ieee80211_frame	*wh;
	struct ieee80211_node	*ni;
	struct r92s_rx_stat	*stat;
	uint32_t	rxdw0, rxdw3;
	uint8_t	rate;
	int8_t	rssi = 0;
	int	s, infosz;

	stat = (struct r92s_rx_stat *)buf;
	rxdw0 = letoh32(stat->rxdw0);
	rxdw3 = letoh32(stat->rxdw3);

	if (rxdw0 & R92S_RXDW0_CRCERR) {
		dp->stats.errrcv++;
		return (NULL);
	}
	if (pktlen < sizeof (*wh) /* || pktlen > MCLBYTES */) {
		dp->stats.errrcv++;
		return (NULL);
	}

	rate = MS(rxdw3, R92S_RXDW3_RATE);
	infosz = MS(rxdw0, R92S_RXDW0_INFOSZ) * 8;

	/* Get RSSI from PHY status descriptor if present. */
	if (infosz != 0) {
		rssi = rsu_get_rssi(dp, rate, &stat[1]);
	}

	DPRINTF(3, (CE_CONT,
	    "!%s: %s: Rx frame len=%d rate=%d infosz=%d rssi=%d",
	    dp->name, __func__,
	    pktlen, rate, infosz, rssi));

	if (mp == NULL) {
		mp = dupb(mp_head);
	}
	if (mp == NULL) {
		dp->stats.errrcv++;
		goto x;
	}
#ifdef notyet
	/* Hardware does Rx TCP checksum offload. */
	if (rxdw3 & R92S_RXDW3_TCPCHKVALID) {
		if (rxdw3 & R92S_RXDW3_TCPCHKRPT) {
			mp->m_pkthdr.csum_flags |= M_TCP_CSUM_IN_OK;
		} else {
			mp->m_pkthdr.csum_flags |= M_TCP_CSUM_IN_BAD;
		}
	}
#endif
	wh = (struct ieee80211_frame *)((uint8_t *)&stat[1] + infosz);
	mp->b_rptr = (void *)wh;
	mp->b_wptr = mp->b_rptr + pktlen;
#if 0
	rxi.rxi_rssi = rssi;
	rxi.rxi_tstamp = 0;	/* Unused. */
#endif
	mp->b_cont = (void *)(uintptr_t)rssi;
	mp->b_prev = (void *)(uintptr_t)rate;
	if (rate != 0) {
		/* update current rate */
		ic->ic_bss->in_txrate = rate;
	}

x:
	return (mp);
}

static mblk_t *
rsu_rx_multi_frame(struct uwgem_dev *dp, mblk_t *mp_head)
{
	struct r92s_rx_stat	*stat;
	uint32_t	rxdw0;
	int		totlen, pktlen, infosz, npkts;
	int		len;
	uint8_t		*buf;
	mblk_t		*reuse_mp;
	mblk_t		*mp;
	mblk_t		*mp_ret;
	mblk_t		**mp_tail = &mp_ret;

	/* Get the number of encapsulated frames. */
	buf = mp_head->b_rptr;
	len = mp_head->b_wptr - mp_head->b_rptr;

	stat = (struct r92s_rx_stat *)mp_head->b_rptr;
	npkts = MS(letoh32(stat->rxdw2), R92S_RXDW2_PKTCNT);
	DPRINTF(3, (CE_CONT, "!%s: %s: Rx %d frames in one chunk",
	    dp->name, __func__, npkts));

	/* Process all of them. */
	reuse_mp = mp_head;
	while (npkts-- > 0) {
		if (len < sizeof (*stat)) {
			break;
		}
		stat = (struct r92s_rx_stat *)buf;
		rxdw0 = letoh32(stat->rxdw0);

		pktlen = MS(rxdw0, R92S_RXDW0_PKTLEN);
		if (pktlen == 0) {
			break;
		}
		infosz = MS(rxdw0, R92S_RXDW0_INFOSZ) * 8;

		/* Make sure everything fits in xfer. */
		totlen = sizeof (*stat) + infosz + pktlen;
		if (totlen > len) {
			break;
		}
		/* Process 802.11 frame. */
		mp = rsu_rx_frame(dp, buf, pktlen, mp_head, reuse_mp);
		if (mp != NULL) {
			*mp_tail = mp;
			mp_tail = &mp->b_next;

			/* original mp was consumed */
			reuse_mp = NULL;
		}

		/* Next chunk is 128-byte aligned. */
		totlen = (totlen + 127) & ~127;
		buf += totlen;
		len -= totlen;
	}

	*mp_tail = NULL;

	return (mp_ret);
}

static mblk_t *
rsu_rx_make_packet(struct uwgem_dev *dp, mblk_t *mp_head)
{
	struct r92s_rx_stat	*stat;
	int	len;

	DPRINTF(4, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	len = mp_head->b_wptr - mp_head->b_rptr;
	if (len < sizeof (*stat)) {
		cmn_err(CE_CONT, "!%s: %s: xfer too short %d",
		    dp->name, __func__, len);
		mp_head = NULL;
		goto x;
	}

	/* Determine if it is a firmware C2H event or an 802.11 frame. */
	stat = (struct r92s_rx_stat *)mp_head->b_rptr;
	if ((letoh32(stat->rxdw1) & 0x1ff) == 0x1ff) {
		mp_head = rsu_rx_multi_event(dp, mp_head);
	} else {
		mp_head = rsu_rx_multi_frame(dp, mp_head);
	}
x:
	return (mp_head);
}

/*
 * Power on sequence for A-cut adapters.
 */
static void
rsu_power_on_acut(struct uwgem_dev *dp)
{
	uint32_t reg;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	rsu_write_1(dp, R92S_SPS0_CTRL + 1, 0x53);
	rsu_write_1(dp, R92S_SPS0_CTRL + 0, 0x57);

	/* Enable AFE macro block's bandgap and Mbias. */
	rsu_write_1(dp, R92S_AFE_MISC,
	    rsu_read_1(dp, R92S_AFE_MISC) |
	    R92S_AFE_MISC_BGEN | R92S_AFE_MISC_MBEN);

	/* Enable LDOA15 block. */
	rsu_write_1(dp, R92S_LDOA15_CTRL,
	    rsu_read_1(dp, R92S_LDOA15_CTRL) | R92S_LDA15_EN);

	rsu_write_1(dp, R92S_SPS1_CTRL,
	    rsu_read_1(dp, R92S_SPS1_CTRL) | R92S_SPS1_LDEN);
	DELAY(2*1000);
	/* Enable switch regulator block. */
	rsu_write_1(dp, R92S_SPS1_CTRL,
	    rsu_read_1(dp, R92S_SPS1_CTRL) | R92S_SPS1_SWEN);

	rsu_write_4(dp, R92S_SPS1_CTRL, 0x00a7b267);

	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 1,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL + 1) | 0x08);

	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x20);

	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 1,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL + 1) & ~0x90);

	/* Enable AFE clock. */
	rsu_write_1(dp, R92S_AFE_XTAL_CTRL + 1,
	    rsu_read_1(dp, R92S_AFE_XTAL_CTRL + 1) & ~0x04);
	/* Enable AFE PLL macro block. */
	rsu_write_1(dp, R92S_AFE_PLL_CTRL,
	    rsu_read_1(dp, R92S_AFE_PLL_CTRL) | 0x11);
	/* Attach AFE PLL to MACTOP/BB. */
	rsu_write_1(dp, R92S_SYS_ISO_CTRL,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL) & ~0x11);

	/* Switch to 40MHz clock instead of 80MHz. */
	rsu_write_2(dp, R92S_SYS_CLKR,
	    rsu_read_2(dp, R92S_SYS_CLKR) & ~R92S_SYS_CLKSEL);

	/* Enable MAC clock. */
	rsu_write_2(dp, R92S_SYS_CLKR,
	    rsu_read_2(dp, R92S_SYS_CLKR) |
	    R92S_MAC_CLK_EN | R92S_SYS_CLK_EN);

	rsu_write_1(dp, R92S_PMC_FSM, 0x02);

	/* Enable digital core and IOREG R/W. */
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x08);

	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x80);

	/* Switch the control path to firmware. */
	reg = rsu_read_2(dp, R92S_SYS_CLKR);
	reg = (reg & ~R92S_SWHW_SEL) | R92S_FWHW_SEL;
	rsu_write_2(dp, R92S_SYS_CLKR, reg);

	rsu_write_2(dp, R92S_CR, 0x37fc);

	/* Fix USB RX FIFO issue. */
	rsu_write_1(dp, 0xfe5c,
	    rsu_read_1(dp, 0xfe5c) | 0x80);
	rsu_write_1(dp, 0x00ab,
	    rsu_read_1(dp, 0x00ab) | 0xc0);

	rsu_write_1(dp, R92S_SYS_CLKR,
	    rsu_read_1(dp, R92S_SYS_CLKR) & ~R92S_SYS_CPU_CLKSEL);
}

/*
 * Power on sequence for B-cut and C-cut adapters.
 */
static void
rsu_power_on_bcut(struct uwgem_dev *dp)
{
	uint32_t	reg;
	int		ntries;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Prevent eFuse leakage. */
	rsu_write_1(dp, 0x37, 0xb0);	/* ok */
	delay(drv_usectohz(10*1000));
	rsu_write_1(dp, 0x37, 0x30);

	/* Switch the control path to hardware. */
	reg = rsu_read_1(dp, R92S_SYS_CLKR + 1);
#if (R92S_FWHW_SEL >> 8) != 0x80
#error
#endif
#if (R92S_SWHW_SEL | R92S_FWHW_SEL) >> 8 != 0xc0
#error
#endif
	DPRINTF(0, (CE_CONT, "!%s: %s: SYS_CLKR+1=0x%x",
	    dp->name, __func__, reg));
	if (reg & (R92S_FWHW_SEL >> 8)) {
		rsu_write_1(dp, R92S_SYS_CLKR + 1,
		    reg & ~((R92S_SWHW_SEL | R92S_FWHW_SEL) >> 8));	/* ok */
	}
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) & ~0x8c);	/* ok */
	delay(drv_usectohz(1000));
	DPRINTF(100, (CE_CONT, "!%s: %s: SYS_CLKR = %x",
	    dp->name, __func__, rsu_read_2(dp, R92S_SYS_CLKR)));

	rsu_write_1(dp, R92S_SPS0_CTRL + 1, 0x53);	/* ok */
	rsu_write_1(dp, R92S_SPS0_CTRL + 0, 0x57);	/* ok */

	reg = rsu_read_1(dp, R92S_AFE_MISC);
	rsu_write_1(dp, R92S_AFE_MISC, reg | R92S_AFE_MISC_BGEN); /* ok */

	rsu_write_1(dp, R92S_AFE_MISC, reg | R92S_AFE_MISC_BGEN |
	    R92S_AFE_MISC_MBEN | R92S_AFE_MISC_I32_EN); /* ok */

	/* Enable PLL. */
	rsu_write_1(dp, R92S_LDOA15_CTRL,
	    rsu_read_1(dp, R92S_LDOA15_CTRL) | R92S_LDA15_EN);	/* ok */

	rsu_write_1(dp, R92S_LDOV12D_CTRL,
	    rsu_read_1(dp, R92S_LDOV12D_CTRL) | R92S_LDV12_EN);	/* ok */

	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 1,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL + 1) | 0x08);	/* ok */

	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x20);	/* ok */

	/* Support 64KB IMEM. */
	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 1,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL + 1) & ~0x97);	/* ok */

	/* Enable AFE clock. */
	rsu_write_1(dp, R92S_AFE_XTAL_CTRL + 1,
	    rsu_read_1(dp, R92S_AFE_XTAL_CTRL + 1) & ~0x04);

	/* Enable AFE PLL macro block. */
	reg = rsu_read_1(dp, R92S_AFE_PLL_CTRL);	/* ok */
	rsu_write_1(dp, R92S_AFE_PLL_CTRL, reg | 0x11);	/* ok */
	DELAY(500);
	rsu_write_1(dp, R92S_AFE_PLL_CTRL, reg | 0x51);	/* ok */
	DELAY(500);
	rsu_write_1(dp, R92S_AFE_PLL_CTRL, reg | 0x11);	/* ok */
	DELAY(500);

	/* Attach AFE PLL to MACTOP/BB. */
	rsu_write_1(dp, R92S_SYS_ISO_CTRL,
	    rsu_read_1(dp, R92S_SYS_ISO_CTRL) & ~0x11);	/* ok */

	/* Switch to 40MHz clock. */
	rsu_write_1(dp, R92S_SYS_CLKR, 0x00);	/* ok */
	DPRINTF(100, (CE_CONT, "!%s: %s: switch to 40MHz clk: SYS_CLKR = %x",
	    dp->name, __func__, rsu_read_2(dp, R92S_SYS_CLKR)));

	/* Disable CPU clock and 80MHz SSC. */
	rsu_write_1(dp, R92S_SYS_CLKR,
	    rsu_read_1(dp, R92S_SYS_CLKR) | 0xa0);	/* ok */
	DPRINTF(100, (CE_CONT, "!%s: %s: disable CPU clk: SYS_CLKR = %x",
	    dp->name, __func__, rsu_read_2(dp, R92S_SYS_CLKR)));
#if (R92S_MAC_CLK_EN | R92S_SYS_CLK_EN) >> 8 != 0x18
#error
#endif
	/* Enable MAC clock. */
	rsu_write_1(dp, R92S_SYS_CLKR + 1,
	    rsu_read_1(dp, R92S_SYS_CLKR + 1) |
	    ((R92S_MAC_CLK_EN | R92S_SYS_CLK_EN) >> 8));	/* ok */
	DPRINTF(100, (CE_CONT, "!%s: %s: enable MAC clk: SYS_CLKR = %x",
	    dp->name, __func__, rsu_read_2(dp, R92S_SYS_CLKR)));

	rsu_write_1(dp, R92S_PMC_FSM, 0x02);	/* ok */

	/* Enable digital core and IOREG R/W. */
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x08);	/* ok */

	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1,
	    rsu_read_1(dp, R92S_SYS_FUNC_EN + 1) | 0x80);	/* ok */

	/* Switch the control path to firmware. */
#if (R92S_SWHW_SEL >> 8) != 0x40
#error
#endif
#if (R92S_FWHW_SEL >> 8) != 0x80
#error
#endif
	reg = rsu_read_1(dp, R92S_SYS_CLKR + 1);
	reg = (reg & ~(R92S_SWHW_SEL >> 8)) | (R92S_FWHW_SEL >> 8);
	rsu_write_1(dp, R92S_SYS_CLKR + 1, reg);	/* ok */
	DPRINTF(100, (CE_CONT, "!%s: %s: switch to fw: SYS_CLKR = %x",
	    dp->name, __func__, rsu_read_2(dp, R92S_SYS_CLKR)));

	rsu_write_1(dp, R92S_CR + 0, 0xfc);	/* ok */
	rsu_write_1(dp, R92S_CR + 1, 0x37);	/* ok */

	/* Fix USB RX FIFO issue. */
	rsu_write_1(dp, 0x1025FE5C,
	    rsu_read_1(dp, 0x1025FE5C) | 0x80);	/* ok */

	rsu_write_1(dp, R92S_SYS_CLKR,
	    rsu_read_1(dp, R92S_SYS_CLKR) & ~R92S_SYS_CPU_CLKSEL);	/* ok */
	rsu_write_1(dp, 0x1025FE1C, 0x80);	/* ok */

	/* Make sure TxDMA is ready to download firmware. */
	for (ntries = 20; ntries > 0; ntries--) {
		reg = rsu_read_1(dp, R92S_TCR);
		if ((reg & (R92S_TCR_IMEM_CHK_RPT | R92S_TCR_EMEM_CHK_RPT)) ==
		    (R92S_TCR_IMEM_CHK_RPT | R92S_TCR_EMEM_CHK_RPT)) {
			break;
		}
		DELAY(5);
	}
	if (ntries == 0) {
		/* Reset TxDMA. */
		reg = rsu_read_1(dp, R92S_CR);
		rsu_write_1(dp, R92S_CR, reg & ~R92S_CR_TXDMA_EN);
		DELAY(2);
		rsu_write_1(dp, R92S_CR, reg | R92S_CR_TXDMA_EN);
		cmn_err(CE_WARN, "!%s: %s: failed to initialize chip",
		    dp->name, __func__);
	}
}

static void
rsu_power_off(struct uwgem_dev *dp)
{
	struct rsu_dev *lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	/* Turn RF off. */
	rsu_write_1(dp, R92S_RF_CTRL, 0x00);
	delay(drv_usectohz(20*1000));

	/* Turn MAC off. */
	/* Switch control path. */
	rsu_write_1(dp, R92S_SYS_CLKR + 1, 0x38);

	/* Reset MACTOP. */
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1, 0x70);
	rsu_write_1(dp, R92S_PMC_FSM, 0x06);
	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 0, 0xf9);
	rsu_write_1(dp, R92S_SYS_ISO_CTRL + 1, 0xe8);

	/* Disable AFE PLL. */
	rsu_write_1(dp, R92S_AFE_PLL_CTRL, 0x00);

	/* Disable A15V. */
	rsu_write_1(dp, R92S_LDOA15_CTRL, 0x54);

	/* Disable eFuse 1.2V. */
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1, 0x50);
	rsu_write_1(dp, R92S_LDOV12D_CTRL, 0x24);

	/* Enable AFE macro block's bandgap and Mbias. */
	rsu_write_1(dp, R92S_AFE_MISC, 0x30);

	/* Disable 1.6V LDO. */
	rsu_write_1(dp, R92S_SPS0_CTRL + 0, 0x56);
	rsu_write_1(dp, R92S_SPS0_CTRL + 1, 0x43);

	lp->initialized = 0;
}

#define	RSU_MAXBUFSZ_MAX	(8192 + sizeof (struct r92s_tx_desc))
#define	RSU_MAXBUFSZ_LINUX	(48*1024 + sizeof (struct r92s_tx_desc))
#if 0
#define	RSU_MAXBUFSZ	0xffffU
#else
#define	RSU_MAXBUFSZ	(3*1024)
#endif

static int
rsu_fw_loadsection(struct uwgem_dev *dp, uint8_t *buf, int len, int maxbufsz)
{
	struct rsu_dev *lp = dp->private;
	struct r92s_tx_desc *txd;
	int	pipe;
	int	mlen, error = USB_SUCCESS;
	mblk_t	*mp;
	int	i;
	int	pktsize = dp->highspeed ? 512 : 64;
#if 0
	pipe = lp->qid2idx[RSU_QID_VO];	/* don't care */
#else
	pipe = 0;	/* use the first bulk-in pipe */
#endif
	while (len > 0) {
		mlen = min(len, maxbufsz);
		if (((sizeof (*txd) + mlen) & (pktsize - 1)) == 0) {
			mlen -= 8;
		}

		/* allocate message buffer */
		mp = allocb(sizeof (*txd) + mlen, 0);
		if (mp == NULL) {
			cmn_err(CE_CONT, "!%s: %s: failed to allocate mblk",
			    dp->name, __func__);
			error = USB_FAILURE;
			break;
		}
		mp->b_wptr = mp->b_rptr + sizeof (*txd) + mlen;

		/* make descriptor */
		txd = (struct r92s_tx_desc *)mp->b_rptr;
		memset(txd, 0, sizeof (*txd));
		txd->txdw0 = htole32(SM(R92S_TXDW0_PKTLEN, mlen));
#if R92S_TXDW0_LINIP != 0x10000000
# error
#endif
		if (mlen == len) {
			/* Last chunk. */
			txd->txdw0 |= htole32(R92S_TXDW0_LINIP);
		}

		/* copy payload */
		memcpy(&txd[1], buf, mlen);

#if DEBUG_LEVEL > 10
{
		int	sum = 0;

		for (i = 0; i < mp->b_wptr - mp->b_rptr; i++) {
			sum += mp->b_rptr[i];
		}
		DPRINTF(0, (CE_CONT,
		    "!%s: Wm %x %x sum:%x\n",
		    dp->name, pipe, mp->b_wptr - mp->b_rptr, sum & 0xff));
		for (i = 0; i < 52; i += 8) {
			uint8_t	*bp = &mp->b_rptr[i];
			DPRINTF(0, (CE_CONT,
			    "\t%02x %02x %02x %02x %02x %02x %02x %02x",
			    bp[0], bp[1], bp[2], bp[3],
			    bp[4], bp[5], bp[6], bp[7]));
		}
}
#endif
		error = uwgem_bulk_out(dp, mp, pipe, 0 /* async: 1 */);
		if (error != USB_SUCCESS) {
			break;
		}
		buf += mlen;
		len -= mlen;

		DPRINTF(10, (CE_CONT, "!%s: %s: TCR:%x",
		    dp->name, __func__, rsu_read_4(dp, R92S_TCR)));
	}
	return (error);
}

static int
rsu_load_firmware(struct uwgem_dev *dp)
{
	struct ieee80211com	*ic = &dp->ic;
	struct r92s_fw_hdr	*hdr;
	struct r92s_fw_priv	*dmem;
	uint8_t	*imem, *emem;
	int	imemsz, ememsz;
	u_char	*fw;
	size_t	size;
	uint32_t	reg;
	uint32_t	reg1;
	int	ntries, error;
	int	n;

	/* Enable CPU. */
	DPRINTF(0, (CE_CONT, "!%s: %s: SYS_FUNC_EN 0x%x",
	    dp->name, __func__, rsu_read_1(dp, 0x10250003)));

	/* Read firmware image from the filesystem. */
#ifdef never
	if ((error = loadfirmware("rsu-rtl8712fw", &fw, &size)) != 0) {
		cmn_err(CE_CONT,
		    "!%s: %s: failed loadfirmware of file %s (error %d)",
		    dp->name, __func__, "rsu-rtl8712fw", error);
		return (error);
	}
#else
	fw = &rtl8712fw[0];
	size = sizeof (rtl8712fw);
#endif
	if (size < sizeof (*hdr)) {
		cmn_err(CE_CONT, "!%s: %s: firmware too short",
		    dp->name, __func__);
		error = EINVAL;
		goto fail;
	}

	hdr = (struct r92s_fw_hdr *)fw;
	if (hdr->signature != htole16(0x8712) &&
	    hdr->signature != htole16(0x8192)) {
		cmn_err(CE_CONT, "!%s: %s: invalid firmware signature 0x%x",
		    dp->name, __func__, letoh16(hdr->signature));
		error = EINVAL;
		goto fail;
	}

	DPRINTF(0, (CE_CONT, "FW V%d(%x) %02x-%02x %02x:%02x\n",
	    letoh16(hdr->version), letoh16(hdr->version),
	    hdr->month, hdr->day, hdr->hour, hdr->minute));

	/* Make sure that driver and firmware are in sync. */
	if (hdr->privsz != htole32(sizeof (*dmem))) {
		cmn_err(CE_CONT, "!%s: %s: unsupported firmware image",
		    dp->name, __func__);
		error = EINVAL;
		goto fail;
	}

	/* Get FW sections sizes. */
	imemsz = letoh32(hdr->imemsz);
	ememsz = letoh32(hdr->sramsz);
	DPRINTF(0, (CE_CONT, "!%s: imemsz:%d(%x), ememsz:%d(%x), dmemsz:%d",
	    dp->name, imemsz, imemsz, ememsz, ememsz, sizeof (*dmem)));

	/* Check that all FW sections fit in image. */
	if (size < sizeof (*hdr) + imemsz + ememsz) {
		cmn_err(CE_CONT, "!%s: %s: firmware too short",
		    dp->name, __func__);
		error = EINVAL;
		goto fail;
	}
	imem = (uint8_t *)&hdr[1];
	emem = imem + imemsz;

	/* Load IMEM section. */
	error = rsu_fw_loadsection(dp, imem, imemsz, RSU_MAXBUFSZ);
	if (error != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: could not load firmware section %s",
		    dp->name, __func__, "IMEM");
		goto fail;
	}
	/* Wait for load to complete. */
	for (ntries = 100; ntries > 0; ntries--) {	/* was 10 */
		reg = rsu_read_2(dp, R92S_TCR);
		if (reg & R92S_TCR_IMEM_CODE_DONE) {
			break;
		}
#if 0
		DELAY(1000);	/* was 10 */
#else
		delay(drv_usectohz(10*1000));
#endif
	}
	if (ntries == 0 || (reg & R92S_TCR_IMEM_CHK_RPT) == 0) {
		cmn_err(CE_CONT, "!%s: %s: timeout waiting for %s transfer",
		    dp->name, __func__, "IMEM");
		error = ETIMEDOUT;
		goto fail;
	}

	/* Load EMEM section. */
	error = rsu_fw_loadsection(dp, emem, ememsz, RSU_MAXBUFSZ);
	if (error != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: could not load firmware section %s",
		    dp->name, __func__, "EMEM");
		goto fail;
	}
	/* Wait for load to complete. */
	for (ntries = 100; ntries > 0; ntries--) {
		reg = rsu_read_2(dp, R92S_TCR);
		if (reg & R92S_TCR_EMEM_CODE_DONE) {
			break;
		}
#if 0
		DELAY(1000);
#else
		delay(drv_usectohz(10*1000));
#endif
	}
	if (ntries == 0 || (reg & R92S_TCR_EMEM_CHK_RPT) == 0) {
		cmn_err(CE_CONT,
		    "!%s: %s: timeout waiting for %s transfer, TCR:%x",
		    dp->name, __func__, "EMEM", reg);
		error = ETIMEDOUT;
		goto fail;
	}

	/* Enable CPU. */
	reg = rsu_read_1(dp, R92S_SYS_CLKR);
	DPRINTF(0, (CE_CONT, "!%s: %s: WT SYS_CLKR to 0x%x(ori=0x%x)",
	    dp->name, __func__, reg | R92S_SYS_CPU_CLKSEL, reg)); 

	rsu_write_1(dp, R92S_SYS_CLKR, reg | R92S_SYS_CPU_CLKSEL);
	reg1 = rsu_read_1(dp, R92S_SYS_CLKR);

	if (reg1 != (reg | R92S_SYS_CPU_CLKSEL)) {
		cmn_err(CE_CONT, "!%s: %s: could not enable system clock",
		    dp->name, __func__);
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: Error=> WT SYS_FUNC_EN fail; SYS_CLKR = %x;"
		    "  target_val = %x",
		    dp->name, __func__, reg1, reg | R92S_SYS_CPU_CLKSEL));
		error = EIO;
		goto fail;
	}

	reg = rsu_read_1(dp, R92S_SYS_FUNC_EN + 1);
	DPRINTF(0, (CE_CONT, "!%s: %s: WT SYS_FUNC_EN+1 to 0x%x[ori=0x%x]\n",
	    dp->name, __func__, reg | BIT(2), reg));
	rsu_write_1(dp, R92S_SYS_FUNC_EN + 1, reg | BIT(2));

	reg1 = rsu_read_1(dp, R92S_SYS_FUNC_EN + 1);
	if (reg1 != (reg | BIT(2))) {
		cmn_err(CE_CONT, "!%s: %s: could not enable microcontroller",
		    dp->name, __func__);
		DPRINTF(0, (CE_CONT, "!%s: %s: Error=> WT SYS_FUNC_EN fail;"
		    " SYS_FUNC_EN=%x; target_val=%x",
		    dp->name, __func__, reg1, reg | BIT(2)));
		error = EIO;
		goto fail;
	}

	DPRINTF(0, (CE_CONT, "!%s: %s: WT TCR |_BASECHG", dp->name, __func__));
#if 0
	reg = rsu_read_1(dp, R92S_TCR);
	rsu_write_1(dp, R92S_TCR, reg | 0x40);
#endif
	reg = rsu_read_4(dp, R92S_TCR);
	DPRINTF(0, (CE_CONT, "!%s: %s: RD TCR = %x", dp->name, __func__, reg));


	/* Wait for CPU to initialize. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (rsu_read_2(dp, R92S_TCR) & R92S_TCR_IMEM_RDY) {
			break;
		}
		delay(drv_usectohz(1*1000));
	}
	if (ntries == 100) {
		cmn_err(CE_CONT, "!%s: %s: timeout waiting for microcontroller",
		    dp->name, __func__);
		rsu_write_2(dp, 0x10250348, 0xc000);
		rsu_write_2(dp, 0x10250348, 0xc001);
		rsu_write_2(dp, 0x10250348, 0x2000);
		rsu_write_2(dp, 0x10250348, 0x2001);
		rsu_write_2(dp, 0x10250348, 0x2002);
		rsu_write_2(dp, 0x10250348, 0x2003);
		error = ETIMEDOUT;
		goto fail;
	}

	/* Update DMEM section before loading. */
	dmem = &hdr->priv;

	memset(dmem, 0, sizeof (*dmem));
	dmem->hci_sel = R92S_HCI_SEL_USB | R92S_HCI_SEL_8172;	/* ok */

	dmem->nendpoints = dp->num_bulkin_pipes + dp->num_bulkout_pipes;
	dmem->rf_config = 0x12;	/* 1T2R */	/* ok */

	dmem->mp_mode = 0;
	dmem->vcs_type = R92S_VCS_TYPE_AUTO;
	dmem->vcs_mode = R92S_VCS_MODE_RTS_CTS;
	dmem->bw40_en = (ic->ic_htcaps & IEEE80211_HTCAP_CHWIDTH40) != 0;
	dmem->turbo_mode = 1;

	/* Load DMEM section. */
	error = rsu_fw_loadsection(dp, (uint8_t *)dmem, sizeof (*dmem),
	    RSU_MAXBUFSZ);
	if (error != USB_SUCCESS) {
		cmn_err(CE_CONT, "!%s: %s: could not load firmware section %s",
		    dp->name, __func__, "DMEM");
		goto fail;
	}
	/* Wait for load to complete. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (rsu_read_2(dp, R92S_TCR) & R92S_TCR_DMEM_CODE_DONE) {
			break;
		}
		delay(drv_usectohz(20*1000));
	}
	if (ntries == 100) {
		cmn_err(CE_CONT, "!%s: %s: timeout waiting for %s transfer",
		    dp->name, __func__, "DMEM");
		error = ETIMEDOUT;
		goto fail;
	}

	reg = rsu_read_1(dp, 0x1025000A);
	if (reg & BIT(4)) {
		//When boot from EEPROM , FW need more time to read EEPROM
		n = 60;
	} else {
		//boot from EFUSE
		n = 30;
	}

	/* Wait for firmware readiness. */
	for (ntries = 0; ntries < n; ntries++) {
		if ((rsu_read_2(dp, R92S_TCR) & R92S_TCR_FWRDY)) {
			break;
		}
		delay(drv_usectohz(1000*1000));
	}
	if (ntries == n) {
		cmn_err(CE_CONT,
		    "!%s: %s: timeout waiting for firmware readiness, tcr:%x",
		    dp->name, __func__, rsu_read_2(dp, R92S_TCR));
#if 0
		error = ETIMEDOUT;
#endif
		goto fail;
	}
	error = 0;
fail:
	DPRINTF(0, (CE_CONT, "!%s: %s: return: error:%d", dp->name, __func__, error));
	return (error);
}

/* =============================================================== */
/*
 * Hardware manupilation
 */
/* =============================================================== */
static int
rsu_reset_chip(struct uwgem_dev *dp, uint_t flags)
{
	int	error = USB_SUCCESS;
	struct rsu_dev	*lp = dp->private;
#if 0
	rsu_write_1(dp, 0x102502E0 + 0x14, 0);
#endif
	rsu_write_1(dp, 0xFE58, 0);
#if 0
	/* Power on adapter. */
	if (lp->cut == 1) {
		rsu_power_on_acut(dp);
	} else {
		rsu_power_on_bcut(dp);
	}
	/* Load firmware. */
	(void)rsu_load_firmware(dp);
#endif
	return (error);
}

static int
rsu_init_chip(struct uwgem_dev *dp)
{
	struct ieee80211com	*ic = &dp->ic;
	struct r92s_set_pwr_mode	cmd;
	int	i, error;
	struct rsu_dev *lp = dp->private;

	if (lp->initialized) {
		return (USB_SUCCESS);
	}
	if (!lp->initialized) {
		/* initialize sequence counter */
		lp->cmd_seq = 1;	/* was 0 */
#if 0
		rsu_power_off(dp);
#endif
		/* Power on adapter. */
		if (lp->cut == 1) {
			rsu_power_on_acut(dp);
		} else {
			rsu_power_on_bcut(dp);
		}
#if 0
		for (i = 0; i < 0x200; i++) {
			rsu_read_1(dp, i);
		}
#endif
		/* Load firmware. */
#if 0
		DPRINTF(0, (CE_CONT, "!%s: %s: before dl_fw: SYS_FUNC_EN:%x",
		    dp->name, __func__, rsu_read_1(dp, R92S_SYS_FUNC_EN + 1)));
#endif
		error = rsu_load_firmware(dp);
		if (error != 0) {
			goto fail;
		}
		DPRINTF(0, (CE_CONT, "!%s: %s: after dl_fw: SYS_FUNC_EN:%x",
		    dp->name, __func__, rsu_read_1(dp, R92S_SYS_FUNC_EN + 1)));
#if 1
		lp->initialized = B_TRUE;
#endif
	}

	/* Enable Rx TCP checksum offload. */
#ifdef CONFIG_RTL8712_TCP_CSUM_OFFLOAD_RX
	DPRINTF(0, (CE_CONT, "!%s: %s: before setting RCR: %x",
	    dp->name, __func__, rsu_read_4(dp, R92S_RCR)));
	rsu_write_4(dp, R92S_RCR,
	    rsu_read_4(dp, R92S_RCR) | 0x04000000);
	DPRINTF(0, (CE_CONT, "!%s: %s: after setting RCR: %x",
	    dp->name, __func__, rsu_read_4(dp, R92S_RCR)));
#endif
#ifdef CONFIG_RTL8712_TCP_CSUM_OFFLOAD_TX_x
	DPRINTF(0, (CE_CONT, "!%s: %s: before setting TCR: %x",
	    dp->name, __func__, rsu_read_4(dp, R92S_TCR)));
	rsu_write_4(dp, R92S_TCR,
	    rsu_read_4(dp, R92S_TCR) | 0x02000000);
	DPRINTF(0, (CE_CONT, "!%s: %s: after setting TCR: %x",
	    dp->name, __func__, rsu_read_4(dp, R92S_TCR)));
#endif
	/* Append PHY status. */
	rsu_write_4(dp, R92S_RCR,
	    rsu_read_4(dp, R92S_RCR) | 0x02000000);

	rsu_write_4(dp, 0x0040,
	    rsu_read_4(dp, 0x0040) & ~0xff000000);

        /* for usb rx aggregation */
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500B5=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500B5)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500D9=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500D9)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500BD=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500BD)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x1025FE5B=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x1025FE5B)));

	/* Use 128 bytes pages. */
	rsu_write_1(dp, 0x00b5,
	    rsu_read_1(dp, 0x00b5) | 0x01);	/* ok */
	/* Enable USB Rx aggregation. */
	rsu_write_1(dp, 0x00bd,
	    rsu_read_1(dp, 0x00bd) | 0x80);	/* ok */
	/* Set USB Rx aggregation threshold. */
	rsu_write_1(dp, 0x00d9, 0x01);	/* ok */
	/* Set USB Rx aggregation timeout (1.7ms/4). */
	rsu_write_1(dp, 0xfe5b, 0x04);

        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500B5=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500B5)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500D9=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500D9)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x102500BD=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x102500BD)));
        DPRINTF(0, (CE_CONT, "!%s: %s: 0x1025FE5B=0x%x",
            dp->name, __func__, rsu_read_1(dp, 0x1025FE5B)));

	/* Fix USB Rx FIFO issue. */
	rsu_write_1(dp, 0xfe5c,
	    rsu_read_1(dp, 0xfe5c) | 0x80);	/* ok */

	/* end of hal_init */

#if 0
	/* Set MAC address. */
	for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
		rsu_write_1(dp, R92S_MACID + i, ic->ic_macaddr[i]);
		ic->ic_macaddr[i] = rsu_read_1(dp, R92S_MACID + i);
	}
#endif

	/* NB: it really takes that long for firmware to boot. */
	delay(drv_usectohz(1500*1000));
#if 1
	DPRINTF(0, (CE_CONT, "!%s: %s: setting MAC address to %s",
	    dp->name, __func__, ether_sprintf((void *)ic->ic_macaddr)));
	error = rsu_fw_cmd(dp, R92S_CMD_SET_MAC_ADDRESS, ic->ic_macaddr,
	    IEEE80211_ADDR_LEN);
	if (error != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: could not set MAC address", dp->name);
		goto fail;
	}
#endif
#if 1
	rsu_write_1(dp, R92S_USB_HRPWM,
	    R92S_USB_HRPWM_PS_ST_ACTIVE | R92S_USB_HRPWM_PS_ALL_ON);

	memset(&cmd, 0, sizeof (cmd));
	cmd.mode = R92S_PS_MODE_ACTIVE;
	DPRINTF(0, (CE_CONT, "!%s: %s: setting ps mode to %d",
	    dp->name, __func__, cmd.mode));
	error = rsu_fw_cmd(dp, R92S_CMD_SET_PWR_MODE, &cmd, sizeof (cmd));
	if (error != USB_SUCCESS) {
		cmn_err(CE_WARN, "!%s: could not set PS mode", dp->name);
		goto fail;
	}
#endif
#if 1
	if (ic->ic_htcaps & IEEE80211_HTCAP_CHWIDTH40) {
		/* Enable 40MHz mode. */
		DPRINTF(0, (CE_CONT, "!%s: %s: setting 40MHz mode",
		    dp->name, __func__));
		error = rsu_fw_iocmd(dp,
		    SM(R92S_IOCMD_CLASS, 0xf4U) |
		    SM(R92S_IOCMD_VALUE, 0x0007U) |
		    SM(R92S_IOCMD_INDEX, 0x00U));
		if (error != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: could not enable 40MHz mode",
			    dp->name, __func__);
			goto fail;
		}
	} else  {
		/* Disable 40MHz mode. */
		error = rsu_fw_iocmd(dp,
		    SM(R92S_IOCMD_CLASS, 0xf4) |
		    SM(R92S_IOCMD_VALUE, 0x0005) |
		    SM(R92S_IOCMD_INDEX, 0x00));
		if (error != USB_SUCCESS) {
			cmn_err(CE_WARN,
			    "!%s: %s: could not disable 40MHz mode",
			    dp->name, __func__);
			goto fail;
		}
	}
#endif
	/* Set default channel. */
	ic->ic_bss->in_chan = ic->ic_ibss_chan;

	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		rsu_delete_key(dp, i);
	}

#if 0
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++) {
			rsu_set_key(dp, NULL, &ic->ic_nw_keys[i]);
		}
	}
#endif
	lp->scan_pass = 0;

	return (USB_SUCCESS);
fail:
	return (USB_FAILURE);
}

static int
rsu_start_chip(struct uwgem_dev *dp)
{
	return (USB_SUCCESS);
}

static int
rsu_stop_chip(struct uwgem_dev *dp)
{
	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));
#if 1
	/* Power off adapter. */
	rsu_power_off(dp);
#endif
	return (USB_SUCCESS);
}

static int
rsu_set_rx_filter(struct uwgem_dev *dp)
{
	int	error;
	int	i;
	uint8_t	*mac;

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

	mac = (void *)dp->ic.ic_macaddr;
#if 0
	/* XXX - don't use MACID register */
	for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
		rsu_write_1(dp, R92S_MACID + i, mac[i]);
	}
#endif
	error = rsu_fw_cmd(dp, R92S_CMD_SET_MAC_ADDRESS, mac,
	    IEEE80211_ADDR_LEN);
#if 0
	DELAY(1000);
#else
	delay(drv_usectohz(10*1000));
#endif
	return (error);
}

static int
rsu_get_stats(struct uwgem_dev *dp)
{
	return (USB_SUCCESS);
}

static mblk_t *
rsu_tx_make_packet(struct uwgem_dev *dp, mblk_t *mp,
    uint_t type, struct ieee80211_node *ni)
{
	struct rsu_dev *lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	struct ieee80211_frame	*wh;
	struct ieee80211_key	*k = NULL;
	struct r92s_tx_desc	*txd;
	uint16_t	qos;
	uint8_t		qid, tid = 0;
	int		hasqos, xferlen, error;
	int		pktlen;
	mblk_t		*m, *m0;
	size_t		mblen;
	uint8_t		*bp;
	int		ampdu = B_FALSE;

#if DEBUG_LEVEL > 3
	m = msgpullup(mp, -1);
	uwgem_dump_pkt(__func__, m->b_rptr, m->b_wptr - m->b_rptr, 0, 0);
	freemsg(m);
#endif
	pktlen = msgdsize(mp);

	/*
	 * encryption requires some additional bytes and 32 bytes
	 * for padds.
	 */
	sizeof (*txd);
	m = allocb(sizeof (*txd) + pktlen + 64, BPRI_MED);
	if (m == NULL) {
		cmn_err(0, (CE_WARN,
		    "!%s: %s: can't alloc mblk", dp->name, __func__));
		dp->stats.noxmtbuf++;
		goto fail;
	}
	ASSERT(m->b_cont == NULL);
	m->b_rptr += sizeof (*txd);

	bp = m->b_rptr;
	for (m0 = mp; m0 != NULL; m0 = m0->b_cont) {
		mblen = MBLKL(m0);
		(void) bcopy(m0->b_rptr, bp, mblen);
		bp += mblen;
	}
	m->b_wptr = bp;
	wh = (struct ieee80211_frame *)m->b_rptr;

	if (type == IEEE80211_FC0_TYPE_DATA) {
		/* 
		 * As type of nulldata packet is MGT
		 * it never come here
		 */
		(void) ieee80211_encap(ic, m, ni);

	} else if (type == IEEE80211_FC0_TYPE_MGT) {
		uint8_t	*cp;

		cp = (uint8_t *)&wh[1];
		if (cp[0] == IEEE80211_ACTION_CAT_BA &&
		    cp[1] == IEEE80211_ACTION_BA_ADDBA_REQUEST) {
			uint_t	t;
			uint16_t	baparamset;
			uint16_t	baseq;

			baparamset = cp[4] << 8 | cp[3];
			baseq = cp[8] << 8 | cp[7];
			DPRINTF(1, (CE_CONT, "!%s: %s: addba req: %04x %04x",
			    dp->name, __func__, baparamset, baseq));

			t = (baparamset & IEEE80211_BAPS_TID)
			    >> IEEE80211_BAPS_TID_S;

			/* fix bufsiz */
			baparamset &= ~IEEE80211_BAPS_BUFSIZ;
			baparamset |= 32U << IEEE80211_BAPS_BUFSIZ_S;
			cp[3] = (uint8_t)baparamset;
			cp[4] = (uint8_t)(baparamset >> 8);

			/* XXX - fix start seq. Is it right to add one? */
			baseq = ((ni->in_txseqs[t] + 1)
			    << IEEE80211_BASEQ_START_S)
			    & IEEE80211_BASEQ_START;
			cp[7] = (uint8_t)baseq;
			cp[8] = (uint8_t)(baseq >> 8);

		}
	}

	if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		struct ieee80211_key *k;
		k = ieee80211_crypto_encap(ic, m);
		if (k == NULL) {
			freemsg(m);
			m = NULL;
			cmn_err(CE_CONT, "!%s: %s: crypto failed",
			    dp->name, __func__);
			goto fail;
		}
		ASSERT(wh == (void *)m->b_rptr);
	}

	/* ensure that tx packet is single fragment again */
	if (m->b_cont) {
		cmn_err(CE_CONT, "!%s: b_cont != NULL", __func__);
		bp = m->b_wptr;
		for (m0 = m->b_cont; m0 != NULL; m0 = m0->b_cont) {
			mblen = MBLKL(m0);
			(void) bcopy(m0->b_rptr, bp, mblen);
			bp += mblen;
		}
		m->b_wptr = bp;
		freemsg(m->b_cont);
		m->b_cont = NULL;
	}

	/* update packet length after encap and encryption */
	pktlen = m->b_wptr - m->b_rptr;

	if (hasqos = (IEEE80211_QOS_HAS_SEQ(wh))) {
		/* all 11n data packet exept null data */

		struct ieee80211_qosframe *qwh = (void *)wh;

		tid = qwh->i_qos[0] & IEEE80211_QOS_TID;

		if ((qwh->i_qos[0] & IEEE80211_QOS_ACKPOLICY_BA) ==
		    IEEE80211_QOS_ACKPOLICY_BA) {
			/* enable ampdu for the packet */
			ampdu = B_TRUE;

			/* clear software mark of ampdu */
			qwh->i_qos[0] &= ~IEEE80211_QOS_ACKPOLICY_BA;
		}
	} else {
		/* all of non 11n packets and 11n null data */
		tid = 0;
	}
	qid = rsu_ac2qid[TID_TO_WME_AC(tid)];

	/* Get the USB pipe to use for this queue id. */
	m->b_datap->db_cksum16 = lp->qid2idx[qid];

	/* Fill Tx descriptor. */
	m->b_rptr -= sizeof (*txd);
	txd = (struct r92s_tx_desc *)m->b_rptr;
	memset(txd, 0, sizeof (*txd));

	if (IEEE80211_IS_MULTICAST(wh->i_addr1) ||
	    IEEE80211_ADDR_EQ(wh->i_addr1, wh->i_addr3) ||
	    type != IEEE80211_FC0_TYPE_DATA) {

		txd->txdw0 |= htole32(
		    SM(R92S_TXDW0_PKTLEN, pktlen) |
		    SM(R92S_TXDW0_OFFSET, sizeof (*txd)) |
		    R92S_TXDW0_OWN | R92S_TXDW0_FSG | R92S_TXDW0_LSG);

		txd->txdw1 |= htole32(
		    SM(R92S_TXDW1_MACID, R92S_MACID_BSS) |	/* is 5 */
		    SM(R92S_TXDW1_QSEL, R92S_TXDW1_QSEL_BE));	/* is 3 */

		if (!hasqos) {
			txd->txdw1 |= htole32(R92S_TXDW1_NONQOS);
		}

		txd->txdw2 |= htole32(R92S_TXDW2_BMCAST);

		if (type == IEEE80211_FC0_TYPE_DATA) {
			txd->txdw4 = htole32(0x80000000);	//driver uses data rate
		} else {
			txd->txdw4 = htole32(0x80002040);	//driver uses data rate
		}
		if (type == IEEE80211_FC0_TYPE_DATA ||
		    type == IEEE80211_FC0_TYPE_MGT) {
			txd->txdw5 = htole32(0x001f8000);	// 1M
		} else {
			txd->txdw5 = htole32(0x001f9600);	/* what is this ? */
		}

	} else if (type == IEEE80211_FC0_TYPE_DATA) {
		txd->txdw0 |= htole32(
		    SM(R92S_TXDW0_PKTLEN, pktlen) |
		    SM(R92S_TXDW0_OFFSET, sizeof (*txd)) |
		    R92S_TXDW0_OWN | R92S_TXDW0_FSG | R92S_TXDW0_LSG);

		txd->txdw1 |= htole32(
		    SM(R92S_TXDW1_MACID, R92S_MACID_BSS) |	/* is 5 */
		    SM(R92S_TXDW1_QSEL, R92S_TXDW1_QSEL_BE));	/* is 3 */

		if (!hasqos) {
			txd->txdw1 |= htole32(R92S_TXDW1_NONQOS);
		}
#ifdef CONFIG_CRYPTO_HW
		/* HW encryption/decryption */

		if (ic->ic_def_txkey != IEEE80211_KEYIX_NONE) {
			struct ieee80211_key *k;
			int32_t	cipher;

			k = &ic->ic_nw_keys[ic->ic_def_txkey];
			if (k != NULL) {
				switch (k->wk_cipher->ic_cipher) {
				case IEEE80211_CIPHER_WEP:
					cipher = R92S_TXDW1_CIPHER_WEP;
					break;
				case IEEE80211_CIPHER_TKIP:
					cipher = R92S_TXDW1_CIPHER_TKIP;
					break;
				case IEEE80211_CIPHER_AES_CCM:
					cipher = R92S_TXDW1_CIPHER_AES;
					break;
				default:
					cipher = R92S_TXDW1_CIPHER_NONE;
				}
			txd->txdw1 |= htole32(
			    SM(R92S_TXDW1_CIPHER, cipher) |
			    SM(R92S_TXDW1_KEYIDX, k->wk_keyix));
			}
		}
#endif /* CONFIG_CRYPTO_HW */
		if (!ampdu) {
			txd->txdw2 |= htole32(R92S_TXDW2_BK);
		}

		/*
		 * Firmware will use and increment the sequence number for the
		 * specified TID.
		 */
		txd->txdw3 |= htole32(SM(R92S_TXDW3_SEQ, tid));
	}

#if DEBUG_LEVEL > 3
	cmn_err(CE_CONT, "!%s: %s: txdesc %08x %08x %08x %08x %08x %08x, pipe:%d",
	    dp->name, "rsu_tx_make_packet end",
	    txd->txdw0, txd->txdw1, txd->txdw2, txd->txdw3,
	    txd->txdw4, txd->txdw5,
	    m->b_datap->db_cksum16);
	uwgem_dump_pkt("rsu_tx_make_packet end",
	    m->b_rptr + sizeof (*txd), m->b_wptr - m->b_rptr - sizeof (*txd), 0, 0);
#endif
	if (((m->b_wptr - m->b_rptr) & (lp->usb_pkt_size - 1)) == 0) {
		/* add a zero-size frame as padds */
		txd = (void *)m->b_wptr;
		m->b_wptr += sizeof (*txd);

		memset(txd, 0, sizeof (*txd));

		txd->txdw0 |= htole32(
		    SM(R92S_TXDW0_PKTLEN, 0) |
		    SM(R92S_TXDW0_OFFSET, sizeof (*txd)) |
		    R92S_TXDW0_OWN | R92S_TXDW0_FSG | R92S_TXDW0_LSG);
	}
fail:
	return (m);
}

static int
rsu_mgt_send(ieee80211com_t *ic, mblk_t *mp, uint8_t type)
{
	struct uwgem_dev *dp = (void *)ic;
	int	ret = DDI_SUCCESS;
	struct ieee80211_frame *wh;
	uint8_t	*cp;

	wh = (struct ieee80211_frame *)mp->b_rptr;

	if ((wh->i_fc[0] & IEEE80211_FC0_VERSION_MASK) != IEEE80211_FC0_VERSION_0) {
		/* corrupted */
		goto x;
	}

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_DATA:
		if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
		    IEEE80211_FC0_SUBTYPE_NODATA) {

			/* null data for power management */
			DPRINTF(10, (CE_CONT,
			    "!%s: %s: send null data",
			    dp->name, __func__));
			ret = uwgem_mgt_send(ic, mp, type);

			/* we consumed mp, avoid to free it */
			mp = NULL;
		}
		break;

	case IEEE80211_FC0_TYPE_MGT:
		cp = (uint8_t *)&wh[1];
#ifdef CONFIG_AMPDU
		if (cp[0] == IEEE80211_ACTION_CAT_BA &&
		    cp[1] == IEEE80211_ACTION_BA_ADDBA_REQUEST) {
			uint32_t	tid;
			uint16_t	baparamset;
			uint16_t	baseq;

			baparamset = cp[4] << 8 | cp[3];
			baseq = cp[8] << 8 | cp[7];
			DPRINTF(1, (CE_CONT,
			    "!%s: %s: addba req: baparamset:%04x baseq:%04x",
			    dp->name, __func__, baparamset, baseq));

			tid = (baparamset & IEEE80211_BAPS_TID)
			    >> IEEE80211_BAPS_TID_S;
			tid = htole32(tid);
			(void)rsu_fw_cmd(dp, R92S_CMD_ADDBA_REQ, &tid, sizeof (tid));

			/* XXX - free mp here, don't pass mp to send routine */
			ASSERT(mp != NULL);
			break;
		}
#endif
#ifdef CONFIG_NEW_JOIN
		{
		}
#endif
		break;
	}
x:
	if (mp) {
		/* drop */
		freemsg(mp);
	}
	return (DDI_SUCCESS);
}

static int
rsu_attach_chip(struct uwgem_dev *dp)
{
	int	i;
	int	err;
	int	ret;
	uint32_t	tmp;
	uint8_t	tmp8;
	struct rsu_dev	*lp = dp->private;
	struct ieee80211com	*ic = &dp->ic;
	extern int ieee80211_debug;
#ifdef CONFIG_11N
	int	maxchan;
#endif

	DPRINTF(0, (CE_CONT, "!%s: %s enter", dp->name, __func__));

#if DEBUG_LEVEL > 0
	ieee80211_debug = IEEE80211_MSG_HT;
	ieee80211_debug = IEEE80211_MSG_CONFIG;
	ieee80211_debug = -1;
	ieee80211_debug &= ~(IEEE80211_MSG_ELEMID | IEEE80211_MSG_ASSOC | IEEE80211_MSG_HT);
	ieee80211_debug &= ~(IEEE80211_MSG_INPUT);
#if 0
	ieee80211_debug |= IEEE80211_MSG_HT;
#endif
	ieee80211_debug &= ~(IEEE80211_MSG_CRYPTO);
	ieee80211_debug &= ~(IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS);
#endif
	ret = USB_SUCCESS;

	/* retrieve MAC address and various other things from EEPROM */
	tmp8 = rsu_read_1(dp, R92S_EE_9346CR);
	DPRINTF(0, (CE_CONT, "!%s: %s EE_9346CR:%x", dp->name, __func__, tmp8));
	cmn_err(CE_CONT, "!%s: boot from %s",
	    dp->name, (tmp8 & R92S_9356SEL) ? "eeprom" : "efuse");

	if (tmp8 & R92S_EEPROM_EN) {
		DPRINTF(0, (CE_CONT,
		    "!%s: %s: Autoload OK!!", dp->name, __func__));
		/*
		 * <Roger_Note>
		 * The following operation are prevent Efuse leakage
		 * by turn on 2.5V.
		 * 2008.11.25.
		 */
		tmp8 = rsu_read_1(dp, R92S_EFUSE_TEST + 3);
		rsu_write_1(dp, R92S_EFUSE_TEST + 3, tmp8 | BIT(7));
		delay(drv_usectohz(1*1000));
		rsu_write_1(dp, R92S_EFUSE_TEST + 3, tmp8 & ~BIT(7));

		/* Chip version  */
#if 0
		rsu_write_1(dp, R92S_PMC_FSM, 0x02);
#endif
		tmp = rsu_read_4(dp, R92S_PMC_FSM);
		tmp8 = (tmp >> 15) & 0x1f;
		DPRINTF(0, (CE_CONT, "!%s: PMC_FSM: 0x%x", dp->name, tmp));
		if (tmp8 == 3) {
			lp->cut = 3;
		} else {
			lp->cut = (tmp8 >> 1) + 1;
		}

		bzero(lp->rom, sizeof (lp->rom));
#if 0
		for (i = 0; i < 128/8; i++) {
			efuse_pg_packet_read(dp, i, &lp->rom[i*8]);
		}
#else
		rsu_read_rom(dp);
#endif
		lp->customer_id = lp->rom[0x52];
		lp->board_type = lp->rom[0x54];

		for (i = 0; i < 128; i += 8) {
			cmn_err(CE_CONT,
			    "!%s: %02x %02x %02x %02x %02x %02x %02x %02x",
			    dp->name,
			    lp->rom[i + 0], lp->rom[i + 1],
			    lp->rom[i + 2], lp->rom[i + 3],
			    lp->rom[i + 4], lp->rom[i + 5],
			    lp->rom[i + 6], lp->rom[i + 7]);
		}

		cmn_err(CE_CONT,
		    "!%s: chip version:0x%x, customer id:0x%x, board type:0x%x",
		    dp->name, lp->cut, lp->customer_id, lp->board_type);

		for (i = 0; i < ETHERADDRL; i++) {
#if 0
			/* XXX - don't use MACID register */
			rsu_write_1(dp, R92S_MACID + i, lp->rom[0x12 + i]);
			dp->dev_addr[i] = rsu_read_1(dp, R92S_MACID + i);
#else
			dp->dev_addr[i] = lp->rom[0x12 + i];
#endif
#ifdef COMPAT_L
			rsu_write_1(dp, R92S_MACID + i, dp->dev_addr[i]);
#endif
		}
	}
	

	/* setup capabilities */
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* set device capabilities */
	ic->ic_caps =
	    IEEE80211_C_TXPMGT |	/* tx power management */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_SHSLOT |	/* short slot time supported */
	    IEEE80211_C_WEP |		/* WEP. */
	    IEEE80211_C_WPA | 		/* WPA/RSN. right ? */
	    0;
#ifdef CONFIG_11N
	ic->ic_caps |= IEEE80211_C_PMGT;
#endif
#ifdef ORG
	    IEEE80211_C_SCANALL |	/* Hardware scan. */
	    IEEE80211_C_SHPREAMBLE |	/* Short preamble supported. */
	    IEEE80211_C_SHSLOT |	/* Short slot time supported. */
	    IEEE80211_C_SCANALL |	/* Hardware scan. */
	    IEEE80211_C_WEP |		/* WEP. */
	    IEEE80211_C_RSN;		/* WPA/RSN. */
#endif

	/* WPA/WPA2 support */
	ic->ic_caps |= IEEE80211_C_WPA; /* Support WPA/WPA2 */

	/*
	 * Support 802.11n/HT
	 */
#ifdef CONFIG_11N
	ic->ic_htcaps = IEEE80211_HTC_HT;
	ic->ic_htcaps |= IEEE80211_HTC_AMSDU;
	ic->ic_htcaps |= IEEE80211_HTCAP_MAXAMSDU_7935;
	ic->ic_htcaps |= IEEE80211_HTCAP_CHWIDTH40;
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI20;
	ic->ic_htcaps |= IEEE80211_HTCAP_SHORTGI40;
	ic->ic_htcaps |= IEEE80211_HTCAP_TXSTBC;
	ic->ic_htcaps |= IEEE80211_HTCAP_DSSSCCK40;
	ic->ic_htcaps |= IEEE80211_HTC_AMPDU;
#endif
	/* set supported .11b and .11g rates */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = rsu_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = rsu_rateset_11g;

	/* set supported .11b and .11g channels (1 through 14) */
	for (i = 1; i <= 14; i++) {
		ic->ic_sup_channels[i].ich_freq =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_sup_channels[i].ich_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_PASSIVE;
#ifdef CONFIG_11N
		ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT20;
		if (i <= 14 - 5) {
			ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40U;
		}
		for (i >= 1 + 5) {
			ic->ic_sup_channels[i].ich_flags |= IEEE80211_CHAN_HT40D;
		}
#endif /* CONFIG_11N */
	}

	ic->ic_maxrssi = 127;
	ic->ic_set_shortslot = NULL;	/* not implemented */

	ic->ic_def_txkey = 0;

	dp->rx_buf_len = 12*2048;

	dp->bulkout_timeout =
	    dp->ugc.uwgc_tx_timeout / drv_usectohz(1000*1000);

	switch (dp->num_bulkin_pipes + dp->num_bulkout_pipes) {
	case 4:
		/* index of bulkout pipes */
		lp->qid2idx[RSU_QID_VO] = 0;
		lp->qid2idx[RSU_QID_VI] = 0;
		lp->qid2idx[RSU_QID_BE] = 1;
		lp->qid2idx[RSU_QID_BK] = 1;
		lp->qid2idx[RSU_QID_H2C] = 2;
		break;

	default:
		cmn_err(CE_CONT, "!%s: not supported: num of end points:%d",
		    dp->name, dp->num_bulkin_pipes + dp->num_bulkout_pipes);
		break;
	}

	lp->usb_pkt_size = dp->highspeed ? 512 : 64;

	/* ingore management messages */
	ic->ic_xmit = &rsu_mgt_send;

	return (ret);

usberr:
	cmn_err(CE_WARN, "!%s: %s: usb error detected (%d)",
	    dp->name, __func__, err);
	return (USB_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (device driver DKI) routine
 */
/* ======================================================== */
static int
rsuattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			i;
	ddi_iblock_cookie_t	c;
	int			ret;
	int			revid;
	int			unit;
	int			vid;
	int			pid;
	struct chip_info	*p;
	int			len;
	const char		*drv_name;
	struct uwgem_dev	*dp;
	void			*base;
	struct uwgem_conf	*ugcp;
	struct rsu_dev		*lp;
	size_t			maxbulklen;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, "!%s%d: %s: called, cmd:%d",
	    drv_name, unit, __func__, cmd));

	if (cmd == DDI_ATTACH) {
		/*
		 * Check if the chip is supported.
		 */
		vid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-vendor-id", -1);
		pid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-product-id", -1);
		revid = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    "usb-revision-id", -1);

		/*
		 * construct uwgem configration
		 */
		ugcp = kmem_zalloc(sizeof (*ugcp), KM_SLEEP);

		/* name */
		/*
		 * XXX - softmac requires that ppa is the instance number
		 * of the device, otherwise it hangs in seaching the device.
		 */
		sprintf(ugcp->uwgc_name, "%s%d", drv_name, unit);
		ugcp->uwgc_ppa = unit;

		ugcp->uwgc_ifnum = 0;
		ugcp->uwgc_alt = 0;

		ugcp->uwgc_tx_list_max = 1;

		ugcp->uwgc_rx_header_len = 0;
		ugcp->uwgc_rx_list_max = 8;

		/* time out parameters */
		ugcp->uwgc_tx_timeout = UWGEM_TX_TIMEOUT;
		ugcp->uwgc_tx_timeout_interval = UWGEM_TX_TIMEOUT_INTERVAL;

		/* I/O methods */

		/* mac operation */
		ugcp->uwgc_attach_chip = &rsu_attach_chip;
		ugcp->uwgc_reset_chip = &rsu_reset_chip;
		ugcp->uwgc_init_chip = &rsu_init_chip;
		ugcp->uwgc_start_chip = &rsu_start_chip;
		ugcp->uwgc_stop_chip = &rsu_stop_chip;

		ugcp->uwgc_set_rx_filter = &rsu_set_rx_filter;
		ugcp->uwgc_get_stats = &rsu_get_stats;
		ugcp->uwgc_interrupt = NULL;

		/* packet operation */
		ugcp->uwgc_tx_make_packet = &rsu_tx_make_packet;
		ugcp->uwgc_rx_make_packet = &rsu_rx_make_packet;

		/* newstate */
		ugcp->uwgc_newstate = &rsu_newstate;

		lp = kmem_zalloc(sizeof (struct rsu_dev), KM_SLEEP);

		usb_pipe_get_max_bulk_transfer_size(dip, &maxbulklen);
		DPRINTF(0, (CE_CONT, "!%s%d: %s: maxbulklen:%d\n",
		    drv_name, unit, __func__, maxbulklen));

		dp = uwgem_do_attach(dip, ugcp, lp, sizeof (struct rsu_dev));

		kmem_free(ugcp, sizeof (*ugcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

err_free_mem:
		kmem_free(lp, sizeof (struct rsu_dev));
err_close_pipe:
err:
		return (DDI_FAILURE);
	}

	if (cmd == DDI_RESUME) {
#if 0
		dp = UWGEM_GET_DEV(dip);
		lp = dp->private;
		lp->sc_flags &= ~RT2870_FWLOADED;
		lp->initialized = B_FALSE;
#endif
		return (uwgem_resume(dip));
	}

	return (DDI_FAILURE);
}

static int
rsudetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int	ret;

	if (cmd == DDI_DETACH) {
		ret = uwgem_do_detach(dip);
		if (ret != DDI_SUCCESS) {
			return (DDI_FAILURE);
		}
		return (DDI_SUCCESS);
	}
	if (cmd == DDI_SUSPEND) {
		return (uwgem_suspend(dip));
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
UWGEM_STREAM_OPS(rsu_ops, rsuattach, rsudetach);

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&rsu_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/* ======================================================== */
/*
 * _init : done
 */
/* ======================================================== */
int
_init(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!rsu: _init: called"));

	uwgem_mod_init(&rsu_ops, "rsu");
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		uwgem_mod_fini(&rsu_ops);
	}
	return (status);
}

/*
 * _fini : done
 */
int
_fini(void)
{
	int	status;

	DPRINTF(2, (CE_CONT, "!rsu: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		uwgem_mod_fini(&rsu_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
