/*
 * Copyright 2009 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

/*
 * Copyright (c) 2007, 2008
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef	_RT2870_VAR_H
#define	_RT2870_VAR_H

#include <sys/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * EDCA Access Categories.
 */
enum ieee80211_edca_ac {
	EDCA_AC_BK  = 1,	/* Background */
	EDCA_AC_BE  = 0,	/* Best Effort */
	EDCA_AC_VI  = 2,	/* Video */
	EDCA_AC_VO  = 3		/* Voice */
};
#define	EDCA_NUM_AC	4

#define	RT2870_SUCCESS		0

#define	RT2860_RSSI_OFFSET	92
/* HW supports up to 255 STAs */
#define	RT2870_WCID_MAX		254
#define	RT2870_AID2WCID(aid)	((aid) & 0xff)

#define	RT2870_TX_LIST_COUNT	8
#define	RT2870_RX_LIST_COUNT	8

struct rt2870_amrr {
	uint_t	amrr_min_success_threshold;
	uint_t	amrr_max_success_threshold;
};

struct rt2870_amrr_node {
	int	amn_success;
	int	amn_recovery;
	int	amn_success_threshold;
	int	amn_txcnt;
	int	amn_retrycnt;
};

struct rt2870_softc {
	struct rt2870_amrr	amrr;
	enum ieee80211_state	sc_ostate;

#define	RT2870_ENABLED		(1 << 0)
#define	RT2870_FWLOADED		(1 << 1)
#define	RT2870_UPD_BEACON	(1 << 2)
#define	RT2870_ADVANCED_PS	(1 << 3)
#define	RT2870_F_RUNNING	(1 << 4)
#define	RT2870_F_SUSPEND	(1 << 5)
#define	RT2870_F_QUIESCE	(1 << 6)

	uint32_t		sc_ic_flags;

	int			mgtqid;
	int			sifs;

	/* firmware related info */
	uint32_t		mac_rev;
	uint8_t			freq;

	/* antenna configuration */
	uint8_t			rf_rev;
	uint8_t			ntxchains;
	uint8_t			nrxchains;

	int			fixed_ridx;

	uint8_t			rf24_20mhz;
	uint8_t			rf24_40mhz;
	uint8_t			ext_2ghz_lna;
	uint8_t			ext_5ghz_lna;
	uint8_t			calib_2ghz;
	uint8_t			calib_5ghz;
	uint8_t			dac_test_bit;
	int8_t			txpow1[54];
	int8_t			txpow2[54];
	int8_t			rssi_2ghz[3];
	int8_t			rssi_5ghz[3];
	uint8_t			lna[4];
#if 0
	uint8_t			tssi_2ghz[9];
	uint8_t			tssi_5ghz[9];
	uint8_t			step_2ghz;
	uint8_t			step_5ghz;
#endif
	uint32_t		sc_flags;
	/* RT2870 RCR */
	uint32_t		sc_rcr;

	uint32_t		rf_regs[4];
#if 0
	uint8_t			txpow[14];
#endif

	/* eeprom info */
	struct {
		uint8_t	reg;
		uint8_t	val;
	}			bbp[8];
	uint8_t			leds;
	uint16_t		led[3];
	uint32_t		txpow20mhz[5];
	uint32_t		txpow40mhz_2ghz[5];
	uint32_t		txpow40mhz_5ghz[5];

	struct rt2870_amrr_node	amn[RT2870_WCID_MAX + 1];

	int			led_mode;	/* ok */
	int			hw_radio;	/* ok */
	int			rx_ant;	/* ok */
	int			tx_ant;	/* ok */
	int			nb_ant;	/* ok */

	int			dwelltime;	/* ok */

	/* kstats */
	uint32_t		sc_tx_nobuf;	/* ok */
	uint32_t		sc_rx_nobuf;	/* ok */
	uint32_t		sc_tx_err;	/* ok */
	uint32_t		sc_rx_err;	/* ok */
	uint32_t		sc_tx_retries;	/* ok */

	uint16_t		(*sc_eeprom_read)(struct uwgem_dev *, uint16_t);

	/* power on flag */
	boolean_t		initialized;

	/* ammr */
	clock_t			last_updatestats;

	/* protection */
	int			cur_htcap;
	int			cur_htopmode;
};

#ifdef __cplusplus
}
#endif

#endif /* _RT2860_VAR_H */
