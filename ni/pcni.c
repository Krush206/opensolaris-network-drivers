/*
 * @(#)pcni.c	1.13 06/01/22
 * NE2000 compatible PCMCIA Ethernet Driver for Solaris
 *
 * Copyright (c) 2002-2006 Masayuki Murayama.  All rights reserved.
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
/*
 Changelog:

 08/06/2002  pcni_read_macaddr_rdma() fix to set mac address
	     pcni_config() don't ignore config 0.

	0.8.2 released
 09/08/2002
	pcni_probe deleted

 07/28/2003 0.8.4
	modified for 8bit mode card and tested with gcc3
	IN and OUT macros fixed. multicast hash
*/

/* Copyright from Solaris DDK (Devece driver Development Kit) */
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
#include <sys/vtrace.h>
#include <sys/dlpi.h>
#include <sys/strsun.h>
#include <sys/ethernet.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/gld.h>
#include <sys/miiregs.h>
#include "dp8390reg.h"
#include "dp8390var.h"

#include <sys/pccard.h>
#include <sys/ddi_impldefs.h>
#include <sys/pcmcia.h>
#include <sys/cs_stubs.h>

char	ident[] = "pcni PCMCIA card driver v" VERSION;

char	_depends_on[] = {"misc/gld drv/dp8390"};

#define	FALSE	(0)
#define	TRUE	(!FALSE)

/*
 * Useful macros
 */
#define	MII_ABILITY_PAUSE	0x0400	/* added by IEEE 802.3x */

#define	MII_STATUS_ABILITY	\
		(MII_STATUS_100_BASE_T4	|	\
		 MII_STATUS_100_BASEX_FD |	\
		 MII_STATUS_100_BASEX |	\
		 MII_STATUS_10 |	\
		 MII_STATUS_10_FD)

#define	MII_ABILITY	\
		(MII_ABILITY_100BASE_T4	|	\
		 MII_ABILITY_100BASE_TX_FD |	\
		 MII_ABILITY_100BASE_TX |	\
		 MII_ABILITY_10BASE_T |	\
		 MII_ABILITY_10BASE_T_FD)

#define	MII_ABILITY_BITS	\
	"\020"	\
	"\016RemoteFault"	\
	"\013PAUSE"	\
	"\012100BASE_T4"	\
	"\011100BASE_TX_FD"	\
	"\010100BASE_TX"	\
	"\00710BASE_T_FD"	\
	"\00610BASE_T"

#define	MII_STATUS_BITS	\
	"\020"	\
	"\020100_BASE_T4"	\
	"\017100_BASEX_FD"	\
	"\016100_BASEX"	\
	"\01510_BASE_FD"	\
	"\01410_BASE"	\
	"\007MFPRMBLSUPR"	\
	"\006ANDONE"	\
	"\005REMFAULT"	\
	"\004CANAUTONEG"	\
	"\003LINKUP"	\
	"\002JABBERING"	\
	"\001EXTENDED"

#define	KB	(1024)

/*
 * DL10019/10022 support
 */
#define	DL_MAC_ADDR_PORT	0x14	/* 0x14 .. 0x19 */
#define DL_REV_PORT		0x1a
#define DL_SUM_PORT		0x1b
#define DL_GPIO_PORT		0x1c
#define DL_DIAG_PORT		0x1d

#define GPIO_MCLK		0x80
#define GPIO_MOUT		0x40
#define GPIO_MOUT_SHIFT		6
#define GPIO_MWR_22		0x20
#define GPIO_MWR_19		0x10
#define GPIO_MIN		0x10
#define GPIO_MIN_SHIFT		4

#define	GPIO_MII		(GPIO_MCLK|GPIO_MOUT|GPIO_MWR_22|GPIO_MIN)

/*
 * MDIO command format
 */
#define	MII_OP_SHIFT	(5+5+2+16)
#define	MII_PMD_SHIFT	(5+2+16)
#define	MII_REG_SHIFT	(2+16)
#define	MII_LT_SHIFT	(16)

#define	MII_OP_WR	0x5
#define	MII_OP_RD	0x6
#define	MII_LT_WR	0x2

/*
 * Our hardware configurations
 */
#define	PCNIMAXMC		64
#define	PCNIMCALLOC		(PCNIMAXMC * sizeof(struct ether_addr)) 

#define	WATCHDOG_INTERVAL	drv_usectohz(1000000)	/* 1 sec */
#define	LINK_WATCH_INTERVAL	drv_usectohz(1000000)	/* 1 sec */

/* I/O port */
#define PORT_DATA	0x10
#define PORT_RESET	0x1f
#define PORT_MISC	0x18	/* For IBM CCAE and Socket EA cards */

/* Local memory configutation */
#define MEM_START	0x40
#define MEM_SIZE_8K 	0x20
#define MEM_SIZE_16K	0x40
#define	SA_PROM_SIZE	16
#define	SA_PROM_ADDR	0

static int pcni_auto_detect_xfer_size = 1;
static int pcni_auto_detect_mem_size = 0;
static int pcni_use_shmem = 1;
static int pcni_dump_cis = 0;
static int pcni_enable_100 = 0;


struct pcni_dev {
	/* device structure link */
	client_handle_t		handle;
	gld_mac_info_t		*macinfo;

	/* assigned port address */
	int			sn;		/* socket number */

	/* card event */
	kmutex_t		event_hi_mutex;
	kmutex_t		event_mutex;
	kcondvar_t		readywait_cv;

	/* additional information for hi-level intr */
	kmutex_t		hi_intrlock;
	ddi_softintr_t		soft_id;
	int			intr_hilevel;
	int			softintr_req;

	/* MII management for 100M chip */
	boolean_t		full_duplex;
#ifdef notdef
	boolean_t		speed100;
#endif
	boolean_t		flow_control;
	boolean_t		flow_control_req;
	int			mii_state;
#define		MII_STATE_RESETTING		0
#define		MII_STATE_AUTONEGOTIATING	1
#define		MII_STATE_LINKUP		2
#define		MII_STATE_LINKDOWN		3

	int			mii_timer;	/* in secound */
#define		MII_RESET_TIMEOUT	1
#define		MII_AUTONEGO_TIMEOUT	4
#define		MII_LINKDOWN_TIMEOUT	10

	boolean_t		mii_fixedmode;
	timeout_id_t		link_watcher_id;
	boolean_t		mii_supress_msg;
	int			phy;


	int			state;	/* from link */
#define	CARD_CONFIGURED		0x02
#define	CARD_INSERTED		0x20
#define	WAITING_CARD_INSERTED	0x80


	/* request structure for PCMCIA */
	io_req_t		io;
	irq_req_t		irq;
	config_req_t		conf;
	window_handle_t		win;

	/* shared memory */
	caddr_t			base;

	caddr_t			mem_start;
	caddr_t			mem_end;

	/* pcni hw info */
	int			chip;
#define	CHIP_TYPICAL	0
#define	CHIP_DL10019	1
#define	CHIP_DL10022	2

	boolean_t		shmem;
};

/*
 * Function prototypes.
 */
/* private functions */
static void pcni_cs_error(struct dp8390_dev *dp, char *func, int ret);

static int pcni_mii_init(struct dp8390_dev *dp);
static void pcni_mii_stop(struct dp8390_dev *dp);

static void pcni_config(struct dp8390_dev *dp);
static void pcni_release(struct dp8390_dev *dp);
static int pcni_event(event_t event, int priority, event_callback_args_t *args);
static int pcni_open(struct dp8390_dev *dp);
static int pcni_close(struct dp8390_dev *dp);
static void pcni_reset_chip(struct dp8390_dev *dp);

static int pcni_do_attach_cs(struct dp8390_dev *dp, gld_mac_info_t *macinfo);
static void pcni_do_detach_cs(struct dp8390_dev *dp);

static int pcni_shmem_get_header(struct dp8390_dev *, struct rhead *, int);
static mblk_t *pcni_shmem_get_pkt(struct dp8390_dev *, int, int);
static int pcni_shmem_put_pkt(struct dp8390_dev *, mblk_t *, int);
static void pcni_sendup(struct dp8390_dev *, mblk_t *);

static int pcni_setup_config_register(struct dp8390_dev *dp);
static int pcnisetup(struct dp8390_dev  *dp);
static int pcni_setup_shmem_window(struct dp8390_dev  *, int, int, int);
static int pcni_setup_dma_config(struct dp8390_dev *, int, int);
static void pcni_watchdog(void *);



/* DDI/DKI functions */
static	int pcni_attach(dev_info_t *, ddi_attach_cmd_t);
static	int pcni_detach(dev_info_t *, ddi_detach_cmd_t);

/* GLD interfaces */
static	int pcni_reset(gld_mac_info_t *);
static	int pcni_start(gld_mac_info_t *);
static	int pcni_stop(gld_mac_info_t *);
static	int pcni_set_mac_address(gld_mac_info_t *, unsigned char *);
static	int pcni_set_multicast(gld_mac_info_t *, unsigned char *, int);
static	int pcni_set_promiscuous(gld_mac_info_t *, int);
static	int pcni_get_stats(gld_mac_info_t *, struct gld_stats *);
static	int pcni_send(gld_mac_info_t *, mblk_t *);
static	uint_t pcni_intr(gld_mac_info_t *);
static	uint_t pcni_highintr(gld_mac_info_t *);

static	struct module_info pcniminfo = {
	0,		/* mi_idnum */
	"pcni",		/* mi_idname */
	0,		/* mi_minpsz */
	ETHERMTU,	/* mi_maxpsz */
	32*1024,	/* mi_hiwat */
	1,		/* mi_lowat */
};

static	struct qinit pcnirinit = {
	(int (*)()) NULL,	/* qi_putp */
	gld_rsrv,		/* qi_srvp */
	gld_open,		/* qi_qopen */
	gld_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&pcniminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit pcniwinit = {
	gld_wput,		/* qi_putp */
	gld_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&pcniminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	pcni_info = {
	&pcnirinit,		/* st_rdinit */
	&pcniwinit,		/* st_wrinit */
	NULL,			/* st_muxrinit */
	NULL			/* st_muxwrinit */
};

static	struct cb_ops cb_pcni_ops = {
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
	&pcni_info,	/* cb_stream */
	D_NEW|D_MP	/* cb_flag */
};

static	struct dev_ops pcni_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gld_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	pcni_attach,	/* devo_attach */
	pcni_detach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_pcni_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&pcni_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/*
 * Patchable debug flag.
 * Set this to nonzero to enable *all* error messages.
 */

#ifdef DEBUG_LEVEL
static int pcni_debug = DEBUG_LEVEL;
#define DPRINTF(n, args) if (pcni_debug>(n)) cmn_err args
#else
#define DPRINTF(n, args)
#endif

/*
 * Ethernet broadcast address definition.
 */
static	struct ether_addr	etherbroadcastaddr = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/*
 * _init : done
 */
int
_init(void)
{
	DPRINTF(2, (CE_CONT, "!pcni: _init: called"));
	return mod_install(&modlinkage);
}

/*
 * _fini : done
 */
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

/*
 * Probe for device.: not needed
 */

static	int	pcni_probe_word_mode = 1;
static	int	pcni_probe_word_io   = 1;

static void
pcni_program_rdma(struct dp8390_dev *dp, int rcmd, uint16_t addr, uint16_t bytes)
{
	/* Select page 0 and stop H/W */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

	OUTB(dp, P0_DCR_W, DCR_FT_8 | DCR_LS_NORMAL | dp->dcr_wts);

	OUTB(dp, P0_RBCR0_W, 0x00);		/* reset remote dma */
	OUTB(dp, P0_RBCR1_W, 0x00);
	OUTB(dp, P0_IMR_W,   0x00);		/* inhibit interrupts */
	OUTB(dp, P0_ISR,     0xFF);		/* clear interrupts */
	OUTB(dp, P0_RCR_W,   RCR_MON);		/* disable rx */
	OUTB(dp, P0_TCR_W,   TCR_LB_NIC);	/* disable tx */
	OUTB(dp, P0_RBCR0_W, bytes),
	OUTB(dp, P0_RBCR1_W, bytes >> 8);
	OUTB(dp, P0_RSAR0_W, addr);		/* prom starts from addr 0 */
	OUTB(dp, P0_RSAR1_W, addr >> 8);
	OUTB(dp, PX_CR, rcmd | CR_STA);		/* start */
}

static void
pcni_probe_SAprom(struct dp8390_dev *dp, uint8_t *SA_prom)
{
	int	i;

	pcni_program_rdma(dp, CR_RD_RRD,
			SA_PROM_ADDR, SA_PROM_SIZE*sizeof(uint16_t));

	if (dp->dcr_wts != 0) {
		for(i = 0; i < SA_PROM_SIZE; i++) {
			SA_prom[i] = INW(dp, PORT_DATA);
		}
	}
	else {
		for(i = 0; i < SA_PROM_SIZE; i++) {
			SA_prom[i] = INB(dp, PORT_DATA);
			(void)INB(dp, PORT_DATA);
		}
	}
#ifdef DEBUG_LEVEL
	if (pcni_debug > 0) {
		cmn_err(CE_CONT, "!%s: SA_prom dump: in %s-mode",
			dp->name, dp->dcr_wts ? "WORD": "BYTE");

		for (i = 0; i < SA_PROM_SIZE; i += 8) {
			cmn_err(CE_CONT,
		"!SA_prom[%02x]: %02x %02x %02x %02x %02x %02x %02x %02x",
			i,
			SA_prom[i+0], SA_prom[i+1], SA_prom[i+2], SA_prom[i+3],
			SA_prom[i+4], SA_prom[i+5], SA_prom[i+6], SA_prom[i+7]);
		}
	}
#endif
	/* Issue STP command to stop NIC */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);
}

static void
pcni_xfer_test(struct dp8390_dev *dp)
{
	uint32_t	ptn[2];
	uint32_t	xferd[2];
	int		i;
	int		flags;
	int		addr;
	int		len;

	addr = DP8390_PTOB(dp->ram_start);	/* use first RAM page */
	len  = sizeof(ptn);
	flags = 0;

	/* test pattarn */
	ptn[0] = 0x443322aa;
	ptn[1] = 0x88776655;

	/*
	 * Test 1: 8bit transfer write/read test
	 */
	bzero(xferd, len);
	pcni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint8_t); i++) {
		OUTB(dp, PORT_DATA, ((uint8_t *)ptn)[i]);
	}
	pcni_program_rdma(dp, CR_RD_RRD, addr, len);
	for (i = 0; i < len/sizeof(uint8_t); i++) {
		((uint8_t *)xferd)[i] = INB(dp, PORT_DATA);
	}
	if ((INB(dp, P0_ISR) & ISR_RDC) != 0) {
		if (bcmp((uint8_t *)ptn, (uint8_t *)xferd, len) == 0) {
			flags |= XFER_8BIT;
		}
	}
	/* Stop HW */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

	/*
	 * Test 2:
	 * 16bit transfer write/read test
	 */
	bzero(xferd, len);
	pcni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint16_t); i++) {
		OUTW(dp, PORT_DATA, ((uint16_t *)ptn)[i]);
	}
	pcni_program_rdma(dp,CR_RD_RRD, addr, len);
	for (i = 0; i < len/sizeof(uint16_t); i++) {
		((uint16_t *)xferd)[i] = INW(dp, PORT_DATA);
	}
	if ((INB(dp, P0_ISR) & ISR_RDC) != 0) {
		if (bcmp(ptn, xferd, len) == 0) {
			flags |= XFER_16BIT;
		}
	}
	/* Stop HW */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

	/*
	 * Test 3:
	 * 32bit transfer from 32bit boundary
	 */
	bzero(xferd, len);
	pcni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint32_t); i++) {
		OUTL(dp, PORT_DATA, ((uint32_t *)ptn)[i]);
	}
	pcni_program_rdma(dp, CR_RD_RRD, addr, len);
	for (i = 0; i < len/sizeof(uint32_t); i++) {
		((uint32_t *)xferd)[i] = INL(dp, PORT_DATA);
	}
	if ((INB(dp, P0_ISR) & ISR_RDC) != 0) {
		if (bcmp(ptn, xferd, len) == 0) {
			flags |= XFER_32BIT;
		}
	}
	/* Stop HW */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

	/* report */
	cmn_err(CE_CONT, "!%s: data transfer test: 8bit:%s 16bit:%s 32bit:%s",
		dp->name,
		flags & XFER_8BIT ? "OK" : "NG",
		flags & XFER_16BIT ? "OK" : "NG",
		flags & XFER_32BIT ? "OK" : "NG");

	dp->xfer_width = flags;
}

static int
pcni_probe_mem(struct dp8390_dev *dp, int *startp, int *stopp)
{
	uint32_t	ptn;
	uint32_t	xferd;
	int		i;
	int		len;
	int		rx_start;
	int		rx_stop;

	len  = sizeof(ptn);
#define	TEST_PTN	0x12345678
	/* rx_start: find first writable address */
	for (rx_start = 0; rx_start < 0x100; rx_start += 0x10) {
		/* write test pattern */
		pcni_program_rdma(dp, CR_RD_RWR, DP8390_PTOB(rx_start), len);
		ptn = rx_start ^ TEST_PTN;
		OUTSW(dp, PORT_DATA, (void *)&ptn, len/sizeof(uint16_t));
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT);

		/* read test pattern */
		pcni_program_rdma(dp, CR_RD_RRD, DP8390_PTOB(rx_start), len);
		INSW(dp, PORT_DATA, (void *)&xferd, len/sizeof(uint16_t));
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT);

		DPRINTF(5, (CE_CONT,
			"!pcni_probe_mem: rx_start:%x, ptn:%x xferd:%x",
			rx_start, ptn, xferd));

		if (ptn == xferd) {
			/* writable page was found */
			goto mem_found;
		}
	}
	return 0;

mem_found:
	ptn = TEST_PTN;
	pcni_program_rdma(dp, CR_RD_RWR, DP8390_PTOB(rx_start), len);
	OUTSW(dp, PORT_DATA, (void *)&ptn, len/sizeof(uint16_t));
	OUTB(dp, PX_CR, CR_RD_ABORT);

	for (rx_stop  = rx_start + 0x10; rx_stop < 0x100; rx_stop += 0x10) {

		ptn = TEST_PTN ^ rx_stop;
		pcni_program_rdma(dp, CR_RD_RWR, DP8390_PTOB(rx_stop), len);
		OUTSW(dp, PORT_DATA, (void *)&ptn, len/sizeof(uint16_t));
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT);

		pcni_program_rdma(dp, CR_RD_RRD, DP8390_PTOB(rx_stop), len);
		INSW(dp, PORT_DATA, (void *)&xferd, len/sizeof(uint16_t));
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT);

		if (ptn != xferd) {
			/* not writable */
			DPRINTF(3, (CE_CONT,
				"!pcni_probe_mem: rx_stop:%x not writable",
				rx_stop));
			break;
		}

		pcni_program_rdma(dp, CR_RD_RRD, DP8390_PTOB(rx_start), len);
		INSW(dp, PORT_DATA, (void *)&xferd, len/sizeof(uint16_t));
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT);

		if (TEST_PTN != xferd) {
			/* first address was destroyed */
			DPRINTF(3, (CE_CONT,
				"!pcni_probe_mem: xferd:0x%x", xferd));
			DPRINTF(3, (CE_CONT,
			"!pcni_probe_mem: rx_stop:%x aliased with rx_start",
				rx_stop));
			break;
		}
	}

	/* Stop HW */
	OUTB(dp, PX_CR, CR_RD_ABORT | CR_STP);

	/* report */
	cmn_err(CE_CONT,
		"!%s: pcni_probe_mem: mem:%dKbyte(rx_start:%x rx_stop:%x)",
		dp->name,
		(DP8390_PTOB(rx_stop - rx_start))>>10, rx_start, rx_stop);

	*startp = rx_start;
	*stopp = rx_stop;

	return 1;
}

static int
pcni_dump_am(struct dp8390_dev *dp)
{
	win_req_t	req;
	map_mem_page_t	mem;
	u_char		*virt;
	int		i;
	int		j;
	struct pcni_dev	*lp;
	char		*hex = "0123456789abcdef";
	static char	buf[128];
	char		*bp;
	int		c;
	int		ret;

	DPRINTF(1, (CE_CONT, "!%s: pcni_dump_am()", dp->name));

	lp = (struct pcni_dev *) dp->private;
	bzero(&req, sizeof(req));

	/* Allocate a 4K memory window */
	req.Attributes = WIN_DATA_WIDTH_8|WIN_MEMORY_TYPE_AM|WIN_ENABLE;
	req.Base.base = 0;
#ifdef notdef
	req.Size = 0x1000;
#else
	req.Size = 0;
#endif

	DPRINTF(0, (CE_CONT, "!pcni_dump_am: csx_RequestWindow: handle:%x",
		lp->handle));

	req.win_params.AccessSpeed = 1; /* means 250nS */
	ret = csx_RequestWindow(lp->handle, &lp->win, &req);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestWindow", i);
		return i;
	}
	csx_GetMappedAddr(req.Base.handle, (void **)&virt);

	mem.Page = 0;
	mem.CardOffset = 0;
	ret = csx_MapMemPage(lp->win, &mem);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "MapMemPage", i);
		return i;
	}
#define	CHAR_PER_LINE	(16*2)
	for (i = 0; i < req.Size; i += CHAR_PER_LINE) {
		bzero(buf, sizeof(buf));
		bp = buf;
#ifdef notdef
		sprintf(bp, "%04d:", i);
		bp += strlen(bp);
#endif
		/* hexdecimal dump */
		for (j = 0; j < CHAR_PER_LINE; j+=2) {
			bp[0] = hex[(virt[i+j] >> 4) & 0xf];
			bp[1] = hex[(virt[i+j]     ) & 0xf];
			bp[2] = ' ';
			bp += strlen(bp);
		}

		/* charactor dump */
		for (j = 0; j < CHAR_PER_LINE; j+=2) {
			c = virt[i+j];
			if (c <= ' ') {
				c = '.';
			}
			else if (c >= 127) {
				c = '.';
			}
			*bp++ = c;
		}
		cmn_err(CE_CONT, "!%s", buf);
	}

	ret = csx_ReleaseWindow(lp->win);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "ReleaseWindow", i);
		return i;
	}

	return 0;
}

static int
pcni_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct dp8390_dev	*dp;
	struct pcni_dev	*lp;
	int			i;
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	gld_mac_info_t		*macinfo;
	int			unit;
	char			propname[32];
	char			*valstr;
	int			val;

	unit = ddi_get_instance(dip);

	DPRINTF(2, (CE_CONT,
		"!pcni%d: pcni_attach: dip:x%x called", unit, dip));

	if (cmd == DDI_ATTACH) {
		DPRINTF(2, (CE_CONT,
		"!pcni%d: pcni_attach: called cmd:ATTACH", unit));

		macinfo = gld_mac_alloc(dip);
		if (macinfo == NULL) {
			return DDI_FAILURE;
		}

		/*
		 * Allocate soft data structure
		 */
		dp = (struct dp8390_dev *)
			kmem_zalloc(sizeof(struct dp8390_dev) +
				sizeof(struct pcni_dev) + PCNIMCALLOC,
				KM_SLEEP);

		if (dp == NULL) {
			gld_mac_free(macinfo);
			return DDI_FAILURE;
		}
		dp->dip = dip;

		lp = (struct pcni_dev *)&dp[1];
		dp->private = (caddr_t)lp;

		lp->macinfo = macinfo;

		dp->mc_count_req = 0;
		dp->mc_count = 0;
		dp->mc_list  = (struct mcast_addr *)&lp[1];
		sprintf(dp->name, "%s%d", ddi_driver_name(dip), unit);

		macinfo->gldm_devinfo	   = dip;
		macinfo->gldm_private	   = (caddr_t)dp;
		macinfo->gldm_reset        = pcni_reset;
		macinfo->gldm_start        = pcni_start;
		macinfo->gldm_stop         = pcni_stop;
		macinfo->gldm_set_mac_addr = pcni_set_mac_address;
		macinfo->gldm_send         = pcni_send;
		macinfo->gldm_set_promiscuous = pcni_set_promiscuous;
		macinfo->gldm_get_stats    = pcni_get_stats;
		macinfo->gldm_ioctl        = NULL; 
		macinfo->gldm_set_multicast= pcni_set_multicast;
		macinfo->gldm_intr         = pcni_intr;
		macinfo->gldm_mctl         = NULL;

		macinfo->gldm_ident   = ident;
		macinfo->gldm_type    = DL_ETHER;
		macinfo->gldm_minpkt  = 0;
		macinfo->gldm_maxpkt  = ETHERMTU;
		macinfo->gldm_addrlen = ETHERADDRL;
		macinfo->gldm_saplen  = -2;

		/*
		 * Wait for the card inserted and set up interrupt handler
		 */
		lp->mii_fixedmode = TRUE;
		lp->full_duplex   = FALSE;
		dp->speed100      = FALSE;
		lp->flow_control  = lp->flow_control_req  = FALSE;

		if (pcni_do_attach_cs(dp, macinfo) != DDI_SUCCESS) {
			cmn_err(CE_NOTE,
				"%s: pcni_attach: peci_doattach_cs failed",
				dp->name);
			goto err_do_attach_cs;
		}

		macinfo->gldm_ppa     = lp->sn;
		macinfo->gldm_cookie  = dp->cookie;

		/*
		 * Get media mode infomation from .conf file
		 */
		sprintf(propname, "%s%d-duplex",
				ddi_driver_name(dp->dip), lp->sn);
		if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
                                DDI_PROP_DONTPASS, propname, &valstr
                                )) == DDI_PROP_SUCCESS) {
			lp->mii_fixedmode = TRUE;
			if (strcmp(valstr, "full") == 0) {
				lp->full_duplex = TRUE;
				lp->flow_control_req = TRUE;
			}
			else if (strcmp(valstr, "half") == 0) {
				lp->full_duplex = FALSE;
			}
			else {
				cmn_err(CE_WARN,
					"%s: property %s: illegal value (%s)",
					dp->name, propname, valstr);
			}
			ddi_prop_free(valstr);
		}

		sprintf(propname, "%s%d-speed",
				ddi_driver_name(dp->dip), lp->sn);
		val = ddi_getprop(DDI_DEV_T_ANY, dp->dip,
					DDI_PROP_DONTPASS, propname, -1);
		if (val != -1)  {
			lp->mii_fixedmode = TRUE;
			if (val == 100) {
				dp->speed100 = TRUE;
			}
			else if (val == 10) {
				dp->speed100 = FALSE;
			}
			else {
				cmn_err(CE_WARN,
					"%s: property %s: illegal value (%d)",
					dp->name, propname, val);
			}
		}

		sprintf(propname, "%s%d-flow-control",
			ddi_driver_name(dp->dip), lp->sn);
		val = ddi_getprop(DDI_DEV_T_ANY, dp->dip, DDI_PROP_DONTPASS, propname, -1);
		if (val != -1) {
			if (val == 1) {
				lp->flow_control_req = TRUE;
			}
			else if (val == 0) {
				lp->flow_control_req = FALSE;
			}
			else {
				cmn_err(CE_WARN,
					"%s: property %s: illegal value (%d)",
					dp->name, propname, val);
			}
		}

		if (lp->mii_fixedmode && !lp->full_duplex) {
			lp->flow_control_req = FALSE;
			lp->flow_control     = FALSE;
		}

		/*
		 * Stop the chip
		 */
		(void)pcni_reset_chip(dp);

		dp8390_attach(dp);

		if (pcni_mii_init(dp) != 0) {
			cmn_err(CE_NOTE,
				"%s: pcni_attach: peci_mii_init failed",
				dp->name);
			goto err_gld_register;
		}

		/*
		 * Get the vendor ethernet address.
		 */
		dp->curraddr = dp->factaddr;
		macinfo->gldm_vendor_addr =
					dp->factaddr.ether_addr_octet;
		macinfo->gldm_broadcast_addr =
					etherbroadcastaddr.ether_addr_octet;

		/*
		 * Add interrupt to system.
		 */
		if (gld_register(dip, "pcni", macinfo) != DDI_SUCCESS) {
			goto err_gld_register;
		}

		if (lp->intr_hilevel) {
			if (ddi_add_softintr(dp->dip,
				DDI_SOFTINT_MED, &lp->soft_id,
				NULL, NULL,
				(uint_t (*)(caddr_t))&gld_intr,
				(caddr_t)macinfo) != DDI_SUCCESS) {

				goto err_add_intr;
			}
		}

		DPRINTF(2, (CE_CONT, "!pcni_attach: return: success"));
		return DDI_SUCCESS;

		/* release allocated resource */
err_add_intr:
		gld_unregister(macinfo);

err_gld_register:
		pcni_do_detach_cs(dp);

err_do_attach_cs:
		kmem_free((caddr_t)dp,
			sizeof(struct dp8390_dev) +
			sizeof(struct pcni_dev) + PCNIMCALLOC);

		gld_mac_free(macinfo);

		return DDI_FAILURE;
	}
	return (DDI_FAILURE);
}

static int
pcni_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	gld_mac_info_t		*macinfo;
	struct dp8390_dev	*dp;
	struct pcni_dev	*lp;
	int			i;
	timeout_id_t		id;

	DPRINTF(3, (CE_CONT, "!pcni_detach: dip:x%x called", dip));
	if (cmd == DDI_DETACH) {
		macinfo  = (gld_mac_info_t *)ddi_get_driver_private(dip);
		dp = (struct dp8390_dev *)macinfo->gldm_private;
		lp       = (struct pcni_dev *)dp->private;

		pcni_mii_stop(dp);

		/*
		 * unregister interrupt handler
		 */
		if (lp->intr_hilevel) {
			ddi_remove_softintr(lp->soft_id);

			mutex_enter(&lp->hi_intrlock);
			lp->soft_id = 0;
			mutex_exit(&lp->hi_intrlock);
		}
		gld_unregister(macinfo);

		/*
		 * Unregister with cardservice
		 */
		pcni_do_detach_cs(dp);

		/*
		 * Release memory
		 */
		kmem_free((caddr_t)dp,
			sizeof(struct dp8390_dev) +
			sizeof(struct pcni_dev) + PCNIMCALLOC);

		gld_mac_free(macinfo);

		DPRINTF(3, (CE_CONT, "!pcnidetach: return: success"));

		return DDI_SUCCESS;
	} 

	return DDI_FAILURE;
}

static int
pcni_reset(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	/* reset multicast list */
	dp->started  = FALSE;

	mutex_enter(&dp->pagelock);
	dp8390_init_chip(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
pcni_set_mac_address(gld_mac_info_t *macinfo, unsigned char *addr)
{
	struct dp8390_dev *dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	bcopy((void *)addr,
		dp->curraddr.ether_addr_octet, ETHERADDRL);

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	mutex_enter(&dp->pagelock);
	dp8390_set_mac_addr(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
pcni_start(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	mutex_enter(&dp->pagelock);
	dp8390_start_chip(dp);
	mutex_exit(&dp->pagelock);

	dp->started  = TRUE;

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	/* kick tx watchdog and autonego */
	dp->tx_timeout_id =
		timeout(pcni_watchdog, (void *)macinfo, WATCHDOG_INTERVAL);

	return GLD_SUCCESS;
}

static int
pcni_stop(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;
	timeout_id_t            old_id;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	if (dp->tx_timeout_id != 0) {
		do {
			(void) untimeout(old_id = dp->tx_timeout_id);
		} while (old_id != dp->tx_timeout_id);
		dp->tx_timeout_id = 0;
	}

	mutex_enter(&dp->intrlock);

	dp->started = FALSE;

	/* stop Rx and Tx */
	dp8390_stop_chip(dp);

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
pcni_set_multicast(gld_mac_info_t *macinfo, unsigned char *ep, int flag)
{
	struct dp8390_dev	*dp;
	size_t			len;
	int			i;
	int			cnt;

	dp = (struct dp8390_dev *)macinfo->gldm_private;


	cnt = dp->mc_count;

	if (flag == GLD_MULTI_ENABLE) {
		/* append new addess into mclist */
		if (cnt < PCNIMAXMC) {
			bcopy(ep, dp->mc_list[cnt].addr.ether_addr_octet,
					ETHERADDRL);
			dp->mc_list[cnt].hash = dp8390_ether_crc(ep);
			dp->mc_count++;
		}
		dp->mc_count_req++;
	}
	else {
		for (i=0; i<cnt; i++) {
			if (bcmp(ep, dp->mc_list[i].addr.ether_addr_octet,
					ETHERADDRL) != 0) {
				continue;
			}
			/* squeeze mclist by copying forward */
			len = (cnt - (i + 1)) * sizeof(*dp->mc_list);
			if (len > 0) {
				bcopy(&dp->mc_list[i+1], &dp->mc_list[i], len);
			}
			dp->mc_count--;
			break;
		}
		dp->mc_count_req--;
	}

	mutex_enter(&dp->intrlock);
	dp->rxmode &= ~RXMODE_MULTI_OVF;
	mutex_enter(&dp->pagelock);
	if (dp->mc_count_req != dp->mc_count) {
		dp->rxmode |= RXMODE_MULTI_OVF;
	}
	dp8390_set_rx_filter(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
pcni_set_promiscuous(gld_mac_info_t *macinfo, int flag)
{
	struct dp8390_dev *dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;


	if (flag == GLD_MAC_PROMISC_NONE) {
		dp->rxmode &= ~(RXMODE_PROMISC |RXMODE_ALLMULTI_REQ);
	}
	else if (flag == GLD_MAC_PROMISC_MULTI) {
		dp->rxmode |= RXMODE_ALLMULTI_REQ;
	}
	else {
		dp->rxmode |= RXMODE_PROMISC;
	}

	mutex_enter(&dp->intrlock);

	mutex_enter(&dp->pagelock);
	dp8390_set_rx_filter(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static	int
pcni_get_stats(gld_mac_info_t *macinfo, struct gld_stats *gs)
{
	struct dp8390_dev	*dp;
	struct dp8390_stats	*ds;
	struct pcni_dev		*lp;


	dp = (struct dp8390_dev *)macinfo->gldm_private;
	lp = (struct pcni_dev *)dp->private;

	mutex_enter(&dp->intrlock); /* for dp->stat */

	dp8390_get_stats(dp);

	ds = &dp->stat;

	gs->glds_errxmt    = ds->errxmt;
	gs->glds_errrcv    = ds->errrcv;
	gs->glds_collisions= ds->collisions;

	gs->glds_excoll    = ds->excoll;
	gs->glds_defer     = ds->defer;
	gs->glds_frame     = ds->frame;
	gs->glds_crc       = ds->crc;

	gs->glds_overflow  = ds->overflow;
	gs->glds_underflow = ds->underflow;
	gs->glds_short     = ds->runt;
	gs->glds_missed    = ds->missed;
	gs->glds_xmtlatecoll = ds->xmtlatecoll;
	gs->glds_nocarrier = ds->nocarrier;
	gs->glds_norcvbuf  = ds->norcvbuf;
	gs->glds_intr      = ds->intr;

	/* all before here must be kept in place for v0 compatibility */
	gs->glds_speed     = dp->speed100 ? 100000000: 10000000;
	gs->glds_media     = GLDM_TP;
	gs->glds_duplex    = lp->full_duplex
				? GLD_DUPLEX_FULL : GLD_DUPLEX_HALF;

        gs->glds_dot3_first_coll     = ds->first_coll;
        gs->glds_dot3_multi_coll     = ds->multi_coll;
        gs->glds_dot3_sqe_error      = 0;
        gs->glds_dot3_mac_xmt_error  = 0;
        gs->glds_dot3_mac_rcv_error  = 0;
        gs->glds_dot3_frame_too_long = ds->frame_too_long;

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

#if DEBUG
#ifdef TIMEOUT_TEST
static int	pcni_timeout_test = 0;
#endif
#endif /* DEBUG */

static int
pcni_send(gld_mac_info_t *macinfo, mblk_t *mp)
{
	struct dp8390_dev *dp;
	int		err;
	int		len;
#ifdef DEBUG
	static int		sent_cnt = 0;
#endif

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	DPRINTF(3, (CE_CONT, "!pcni_send: called:"));

	len = msgdsize(mp);
	if (len > ETHERMAX) {
		cmn_err(CE_WARN,  "pcni_send: msg too big:  %d", len);
		dp->stat.errxmt++;

		/* Do not free mp here. GLD will free mp */

		return GLD_NOTSUPPORTED;
	}
#ifdef DEBUG
	sent_cnt++;
#endif
	if ((err = dp8390_tx_start(dp, mp)) != 0) {
		/*
		 * Transmition failed.
		 */
		if (err > 1) {
			/* nic is inactive */
			dp->stat.errxmt++;
			return GLD_FAILURE;
		}
		return GLD_NORESOURCES;
	}
#if defined(DEBUG) && defined(TIMEOUT_TEST)
	if (pcni_timeout_test > 0) {
		if ((sent_cnt % pcni_timeout_test) == 0) {

			/* cause tx timeout */

			mutex_enter(&dp->pagelock);
			dp->imr = dp->imr_shadow = 0;
			OUTB(dp, P0_IMR_W, 0);
			mutex_exit(&dp->pagelock);
		}
	}
#endif

	freemsg(mp);

	return GLD_SUCCESS;
}

static u_int
pcni_intr(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;
	struct pcni_dev		*lp;
	int			serviced;
	int			isr;

	dp = (struct dp8390_dev *)macinfo->gldm_private;
	lp = (struct pcni_dev *)dp->private;
	serviced = DDI_INTR_UNCLAIMED;

	mutex_enter(&dp->intrlock);

	if (lp->intr_hilevel) {
		mutex_enter(&lp->hi_intrlock);
		if (lp->softintr_req == 0) {
			mutex_exit(&lp->hi_intrlock);
			mutex_exit(&dp->intrlock);

			return serviced;
		}
		lp->softintr_req = 0;
		mutex_exit(&lp->hi_intrlock);

		serviced = DDI_INTR_CLAIMED;
	}

	if ((isr = dp8390_interrupt(dp)) != 0) {
		serviced = DDI_INTR_CLAIMED;

		if (isr & INTR_FATAL_ERR) {
			dp8390_stop_chip(dp);

			mutex_enter(&dp->xmitlock);

			mutex_enter(&dp->pagelock);
			pcni_reset_chip(dp);
			dp8390_init_chip(dp);
			dp8390_set_mac_addr(dp);
			dp8390_start_chip(dp);
			mutex_exit(&dp->pagelock);

			mutex_exit(&dp->xmitlock);
		}
	}

	mutex_exit(&dp->intrlock);

	if ((isr & (INTR_RESTART_TX | INTR_RESTART_TX)) != 0) {
		gld_sched(macinfo);
	}

	DPRINTF(4, (CE_CONT, "!%s: pcni_intr: device running: isr:%b",
		dp->name, isr & INTR_ISR_MASK, ISR_BITS));

	return serviced;
}

static u_int
pcni_highintr(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;
	struct pcni_dev		*lp;
	int			ret;
	int			isr;
	timeout_id_t		call_id = 0;

	dp  = (struct dp8390_dev *)macinfo->gldm_private;
	lp  = (struct pcni_dev *)dp->private;
	ret = DDI_INTR_UNCLAIMED;

	mutex_enter(&lp->hi_intrlock);
	mutex_enter(&dp->pagelock);
	ASSERT(dp->imr == dp->imr_shadow);
	isr = INB(dp, P0_ISR) & dp->imr;
	mutex_exit(&dp->pagelock);

	if (lp->soft_id != 0) {
		if (isr != 0) {
			/*  mask all interrupts */
			dp->imr_shadow = 0;
			mutex_enter(&dp->pagelock);
			OUTB(dp, P0_IMR_W, 0);
			mutex_exit(&dp->pagelock);

			/* call low-level interrupt handler */
			lp->softintr_req = isr;
			call_id = lp->soft_id;
			ret = DDI_INTR_CLAIMED;
		}
	}
	else {
		if (isr != 0) {
			/* ack all interrupts */
			mutex_enter(&dp->pagelock);
			OUTB(dp, P0_ISR, isr);
			mutex_exit(&dp->pagelock);

			ret = DDI_INTR_CLAIMED;
		}
	}
	mutex_exit(&lp->hi_intrlock);

	if (call_id != 0) {
		ddi_trigger_softintr(call_id);
	}

	return ret;
}

/*====================================================================*/
static void
pcni_cs_error(struct dp8390_dev *dp, char *func, int ret)
{
	error2text_t	cft;

	cft.item = ret;
	csx_Error2Text(&cft);

	cmn_err(CE_WARN, "!%s: pcni_cs_error: func:%s: failed %s (0x%x)\n",
			dp->name, func, cft.text, ret);
}

/*====================================================================*/
/*
 * Copy routines between memory and pccard.
 */
/*===================================================================== */
/*
 * source is PCMCIA shared memory area
 */
static void
pcni_memcpy(uint8_t *dest, uint8_t *src, int c)
{
	uint16_t	*d = (uint16_t *) dest;
	uint16_t	*s = (uint16_t *) src;
	int		odd;

	odd = c & 1;
	c >>= 1;

	while (c-- > 0) {
		*d++ = *s++;
	}

	/* get last byte by fetching a word and masking */
	if (odd) {
		*d = (*d & 0xff00) | (*s & 0x00ff);
	}
}

static int
pcni_shmem_get_header(struct dp8390_dev *dp, struct rhead *hdr, int page)
{
	struct pcni_dev	*lp;
	caddr_t		xfer_start;

	lp = (struct pcni_dev *)dp->private;

	xfer_start = lp->mem_start + DP8390_PTOB(page - dp->ram_start);

	DPRINTF(4, (CE_CONT,
		"!shmem_get_header: page:x%x, xfer_start:x%x",
		page, xfer_start));

	pcni_memcpy((uint8_t *)hdr, (uint8_t *)xfer_start, sizeof(*hdr));
}

static mblk_t *
pcni_shmem_get_pkt(struct dp8390_dev *dp, int nbytes, int addr)
{
	struct pcni_dev	*lp;
	int		error = 0;
	mblk_t		*mp;
	caddr_t		xfer_start;
	char		*buf;

	lp = (struct pcni_dev *)dp->private;

	DPRINTF(4, (CE_CONT, "!%s: shmem_get_pkt: cnt:%d, addr:x%x",
		dp->name, nbytes, addr));
	/*
	 * Allocate mblk
	 */
	if ((mp = allocb(roundup(nbytes, sizeof(uint32_t)), BPRI_LO)) == NULL) {
		dp->stat.errrcv++;
		dp->stat.norcvbuf++;
		error = 1;
		DPRINTF(1, (CE_CONT, "%s: allocb fail", dp->name));
		return NULL;
	}

	/* fix data block size */
	mp->b_wptr = mp->b_rptr + nbytes;
	xfer_start = lp->mem_start + addr - DP8390_PTOB(dp->ram_start);
	buf = (char *)mp->b_rptr;

	if (xfer_start + nbytes > lp->mem_end) {
		/* We must wrap the input move. */
		int semi_count = lp->mem_end - xfer_start;

		pcni_memcpy((uint8_t *)buf, (uint8_t *)xfer_start, semi_count);

		buf += semi_count;
		addr = DP8390_PTOB(dp->rx_start);
		xfer_start  = lp->mem_start + addr -
			DP8390_PTOB(dp->ram_start);
		nbytes -= semi_count;
	}

	pcni_memcpy((uint8_t *)buf, (uint8_t *)xfer_start, nbytes);

	return mp;
}

/*====================================================================*/

static int
pcni_shmem_put_pkt(struct dp8390_dev *dp, mblk_t *mp, int start_page)
{
	struct pcni_dev	*lp;
	caddr_t		shmem;
	size_t		count;
	mblk_t		*mp0;
	mblk_t		*mp1;
	int		best_align;
	int		tx_copy = 0;


	lp    = (struct pcni_dev *)dp->private;
	count = msgdsize(mp);
	shmem = lp->mem_start +
		(DP8390_PTOB(start_page - dp->ram_start));

	DPRINTF(3, (CE_CONT, "!shmem_put_pkt: cnt:%d start_page:x%x shmem:x%x",
		count, start_page, shmem));
	/*
	 * Check buffer alignment and length.
	 */
	best_align = sizeof(uint16_t)-1; /* too strong ? */

	if (best_align){
		size_t	required_align;

		required_align = 0;
		for (mp0 = mp; mp0->b_cont; mp0 = mp0->b_cont) {
			required_align |= mp0->b_wptr - mp0->b_rptr;
#ifdef sparc
			required_align |= (size_t)mp0->b_rptr;
#endif
		}
#ifdef sparc
		/* for the last message fragment */
		required_align |= mp0->b_wptr - mp0->b_rptr;
#endif

		if ((required_align & best_align) != 0) {
			tx_copy = 1;
		}
	}

	if (tx_copy) {
		mp1 = msgpullup(mp, -1);
		if (mp1 == NULL) {
			/* error: no resource */
			return 1;
		}
	}

	for (mp0 = mp; mp0; mp0 = mp0->b_cont) {
		count = mp0->b_wptr - mp0->b_rptr;
		pcni_memcpy((uint8_t *)shmem, (uint8_t *)mp0->b_rptr, count);
		shmem += count;
	}

	if (tx_copy) {
		freemsg(mp1);
	}

	return 0;
}

/*======================================================================*/
static int
pcni_do_attach_cs(struct dp8390_dev *dp, gld_mac_info_t *macinfo)
{
	client_reg_t		client_reg;
	struct pcni_dev		*lp;
	map_log_socket_t        map_log_socket;
	sockmask_t              sockmask;
	int			ret;
	ddi_iblock_cookie_t	c;

	lp = (struct pcni_dev *)dp->private;
#ifdef notdef
	lp->conf.Attributes = CONF_ENABLE_IRQ_STEERING;
	lp->conf.IntType    = SOCKET_INTERFACE_MEMORY_AND_IO;
#endif
	/*
	 * Register with Card Services
	 */
	client_reg.dip        = dp->dip;
	client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
	client_reg.EventMask  =
		CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
		CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
		CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
	client_reg.event_handler = (csfunction_t *)&pcni_event;
	client_reg.Version = CS_VERSION;
	client_reg.event_callback_args.client_data = dp;

	ret = csx_RegisterClient(&lp->handle, &client_reg);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RegisterClient", ret);
		return DDI_FAILURE;
	}

	DPRINTF(2, (CE_CONT,
		"!%s: pcni_do_attach_cs: RegisterClient: lp:0x%x handle:0x%x",
		dp->name, lp, lp->handle));

	/* Get logical socket number */
	ret = csx_MapLogSocket(lp->handle, &map_log_socket);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "MapLogSocket", ret);
		return DDI_FAILURE;
	}

	lp->sn = map_log_socket.PhySocket;
	sprintf(dp->name, "%s%d@%d",
		ddi_driver_name(dp->dip), ddi_get_instance(dp->dip), lp->sn);

	/*
	 * initialize event related mutex and condv
	 */
	c = *(client_reg.iblk_cookie);
	mutex_init(&lp->event_hi_mutex, NULL, MUTEX_DRIVER, c);
	ddi_get_soft_iblock_cookie(dp->dip, DDI_SOFTINT_MED, &c);
	mutex_init(&lp->event_mutex, NULL, MUTEX_DRIVER, c);
	cv_init(&lp->readywait_cv, NULL, CV_DRIVER, (void *)NULL);


	mutex_enter(&lp->event_mutex);

	/*
	 * After the RequestSocketMask call,
	 *      we can start receiving events
	 */
	sockmask.EventMask = CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL;

	ret = csx_RequestSocketMask(lp->handle, &sockmask);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestSocketMask", ret);
		mutex_exit(&lp->event_mutex);
		goto err;
	}

	/*
	 * If the card is inserted and this attach is triggered
	 *      by an open, that open will wait until
	 *      pcepp_card_insertion() is completed
	 */
	lp->state |= WAITING_CARD_INSERTED;
	while ((lp->state & CARD_INSERTED) == 0) {
		cv_wait(&lp->readywait_cv, &lp->event_mutex);
	}
	lp->state &= ~WAITING_CARD_INSERTED;

	/*
	 * PC Card now must be present and fully functional
	 */
	if ((lp->state & CARD_CONFIGURED) == 0) {
		cmn_err(CE_CONT,
			"!%s.%d: pcni_do_attach_cs: card not ready",
			dp->name, lp->sn);
		mutex_exit(&lp->event_mutex);

		pcni_do_detach_cs(dp);
		goto err;
	}

	mutex_exit(&lp->event_mutex);
	return DDI_SUCCESS;

err:
	return DDI_FAILURE;
}

static void
pcni_do_detach_cs(struct dp8390_dev *dp)
{
	struct pcni_dev		*lp;
	release_socket_mask_t	rsm;
	int			ret;
	int			unit;

	lp = (struct pcni_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: pcni_detach_cs called", dp->name));

	if (lp->state & CARD_CONFIGURED) {
		pcni_release(dp);
	}

	if (lp->handle) {
		ret = csx_ReleaseSocketMask(lp->handle, &rsm);
		if (ret != CS_SUCCESS) {
			pcni_cs_error(dp, "ReleaseSocketMask", ret);
		}

		ret = csx_DeregisterClient(lp->handle);
		if (ret != CS_SUCCESS) {
			pcni_cs_error(dp, "DeregisterClient", ret);
		}
		DPRINTF(2, (CE_CONT,
	"!%s: pcni_do_detatach_cs: DeregisterClient: lp:0x%x handle:0x%x",
			dp->name, lp, lp->handle));

		/* release event related mutex and cv */
		mutex_destroy(&lp->event_mutex);
		mutex_destroy(&lp->event_hi_mutex);
		cv_destroy(&lp->readywait_cv);

		lp->handle = 0;
	}
}

/*======================================================================*/
/*
 * Normal remote DMA support
 */
/*======================================================================*/
static int
pcni_read_macaddr_rdma(struct dp8390_dev *dp)
{
	uint8_t	SA_prom[SA_PROM_SIZE];

	/* try word transfer first */
	dp->dcr_wts = 0;
	pcni_probe_SAprom(dp, SA_prom);
	if (SA_prom[14] == 'W' && SA_prom[15] == 'W') {
		/* prom was read correctly */
		cmn_err(CE_CONT, "!%s: trying rdma: succeeded: type word",
			dp->name);
		bcopy(SA_prom, dp->factaddr.ether_addr_octet, ETHERADDRL);
		return TRUE;
	}

	cmn_err(CE_CONT, "!%s: trying rdma: failed", dp->name);

	return FALSE;
}

/*======================================================================*/
/*
 *  DL10019/DL10022 support
 */
/*======================================================================*/
static int
pcni_read_macaddr_dl(struct dp8390_dev *dp)
{
	int		i;
	uint8_t		sum;
	uint8_t		reg;
	uint8_t		mac[ETHERADDRL+2];
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;

	sum = 0;
	for (i = 0; i < ETHERADDRL + 2; i++) {
		sum += mac[i] = INB(dp, DL_MAC_ADDR_PORT + i);
	}

	if (sum != 0xff) {
		cmn_err(CE_CONT, "!%s: trying dl10019/10022: failed: "
			" %02x:%02x:%02x:%02x:%02x:%02x (cksum %02x.%02x)",
			dp->name,
			mac[0], mac[1], mac[2], mac[3],
			mac[4], mac[5], mac[6], mac[7]);
		return FALSE;
	}

	cmn_err(CE_CONT, "!%s: trying dl10019/10022: succeeded", dp->name);

	bcopy((void *)mac, dp->factaddr.ether_addr_octet, ETHERADDRL);

	/* test 10019 or 10022 */
	reg = INB(dp, PORT_RESET);

	lp->chip = CHIP_DL10019;
	if ((reg == 0x91) || (reg == 0x99)) {
		lp->chip = CHIP_DL10022;
	}

	return TRUE;
}

static void
pcni_generate_macaddr(struct dp8390_dev *dp, uint8_t *mac)
{
	int		i;
	uint_t		val;
	extern char	hw_serial[];

	cmn_err(CE_CONT,
		"!%s: using temporary ether address, do not use this long time",
		dp->name);
	val = 0;
	for (i = 0; i < 11; i++) {
		if (hw_serial[i] == 0) {
			break;
		}
		val = val * 10 + hw_serial[i] - '0';
	}
	val ^= (uint_t)ddi_get_lbolt();
	mac[0] = 2;
	mac[1] = 0;
	mac[2] = val >> 24;
	mac[3] = val >> 16;
	mac[4] = val >> 8;
	mac[5] = val;
}

static int
pcni_read_macaddr_conf(struct dp8390_dev *dp)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;
	char		propname[32];
	char		*valstr;
	uint8_t		mac[ETHERADDRL];
	char		*cp;
	int		c;
	int		i;
	int		j;
	int		v;
	int		d;
	uint8_t		non_zeros = 0;

	/*
	 * Get ethernet address from .conf file
	 */
	sprintf(propname, "%s%d-mac-addr", ddi_driver_name(dp->dip), lp->sn);
	valstr = NULL;
	if ((ddi_prop_lookup_string(DDI_DEV_T_ANY, dp->dip,
			DDI_PROP_DONTPASS, propname, &valstr
			)) != DDI_PROP_SUCCESS) {
		cmn_err(CE_CONT,
			"!%s: trying to read %s property from .conf: undefined",
			dp->name, propname);
		goto auto_generate;
	}

	if (strlen(valstr) != ETHERADDRL*3-1) {
		goto syntax_err;
	}

	cp = valstr;
	j  = 0;
	non_zeros = 0;
	while (1) {
		v = 0;
		for (i = 0; i < 2; i++) {
			c = *cp++;

			if (c >= 'a' && c <= 'f') {
				d = c - 'a' + 10;
			}
			else if (c >= 'A' && c <= 'F') {
				d = c - 'A' + 10;
			}
			else if (c >= '0' && c <= '9') {
				d = c - '0';
			}
			else {
				goto syntax_err;
			}
			v = (v << 4) | d;
		}

		mac[j++] = v;
		non_zeros |= v;
		if (j == ETHERADDRL) {
			/* done */
			break;
		}

		c = *cp++;
		if (c != ':') {
			goto syntax_err;
		}
	}
	ddi_prop_free(valstr);

	if (non_zeros == 0) {
		goto auto_generate;
	}
	bcopy((void *)mac, dp->factaddr.ether_addr_octet, ETHERADDRL);
	return TRUE;

syntax_err:
	ddi_prop_free(valstr);
	cmn_err(CE_CONT,
		"!%s: read mac addr: trying .conf: syntax err %s",
		dp->name, valstr);
	return FALSE;

auto_generate:
	pcni_generate_macaddr(dp, dp->factaddr.ether_addr_octet);
	return TRUE;
}

static void
pcni_config(struct dp8390_dev *dp)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;
	client_handle_t handle = lp->handle;
	static tuple_t		tuple;
	static cisparse_t	parse;
	int			i, ret;
	int			rx_start, rx_stop;
	int			manfid, prodid;
	get_configuration_info_t conf;
	static char		chip_name[80];
	ddi_iblock_cookie_t	c;
	cistpl_cftable_entry_t		*cfg;
	cistpl_cftable_entry_io_t	*io;

	DPRINTF(3, (CE_CONT, "!%s: pcni_config called", dp->name));

	tuple.Attributes = 0;
	tuple.TupleDataMax = sizeof(tuple.TupleData);
	tuple.TupleOffset  = 0;
	tuple.DesiredTuple = CISTPL_CONFIG;
	if ((ret = csx_GetFirstTuple(handle, &tuple)) != CS_SUCCESS) {
		pcni_cs_error(dp, "GetFirstTuple", ret);
		goto failed;
	}

	if ((ret = csx_GetTupleData(handle, &tuple)) != CS_SUCCESS) {
		pcni_cs_error(dp, "GetTupleData", ret);
		goto failed;
	}

	if ((ret = csx_ParseTuple(handle,
			&tuple, &parse, CISTPL_CONFIG)) != CS_SUCCESS) {
		pcni_cs_error(dp, "ParseTuple", ret);
		goto failed;
	}

	lp->conf.Attributes = CONF_ENABLE_IRQ_STEERING;
	lp->conf.IntType    = SOCKET_INTERFACE_MEMORY_AND_IO;
	lp->conf.ConfigBase = parse.config.base;
	lp->conf.Present = parse.config.present;

	/* Configure card */
	lp->state |= CARD_CONFIGURED;

	/* Look up current Vcc */
	bzero(&conf, sizeof(conf));
	conf.Socket = lp->sn;
	if (ret = csx_GetConfigurationInfo(&handle, &conf) != CS_SUCCESS) {
		pcni_cs_error(dp, "GetConfigurationInfo", ret);
		goto failed;
	}
	lp->conf.Vcc = conf.Vcc;
	DPRINTF(0, (CE_CONT, "!pcni_config: Vcc:%d", conf.Vcc));
	if (conf.Vcc != 50 && conf.Vcc != 33) {
		lp->conf.Vcc = conf.Vcc = 50;
	}

	/*
	 * Check Manifacture ID
	 */
	tuple.DesiredTuple = CISTPL_MANFID;
	tuple.Attributes = 0;
	if ((csx_GetFirstTuple(handle, &tuple) == CS_SUCCESS) &&
	    (csx_GetTupleData(handle, &tuple) == CS_SUCCESS)) {
		uint8_t	*buf;

		buf = &tuple.TupleData[0];
		manfid = (buf[1]<<8) | buf[0];
		prodid = (buf[3]<<8) | buf[2];
		cmn_err(CE_CONT, "!%s: (manfid, prodid) = (%x, %x)",
			dp->name, manfid, prodid);
	}

	/*
	 * Allocate IO space
	 */
	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	tuple.Attributes = 0;
	if ((ret = csx_GetFirstTuple(handle, &tuple)) != CS_SUCCESS) {
		pcni_cs_error(dp, "GetFirstTuple", ret);
		goto failed;
	}

	/* analyse configuration table */
	do {
		cfg = &parse.cistpl_cftable_entry;
		io  = &parse.cistpl_cftable_entry.io;

		if (csx_GetTupleData(handle, &tuple) != CS_SUCCESS) {
			continue;
		}
		if (csx_ParseTuple(handle, &tuple, &parse,
					CISTPL_CFTABLE_ENTRY) != CS_SUCCESS) {
			continue;
		}

		DPRINTF(0, (CE_CONT,
"!%s: pcni_config: index:%d flags:0x%x lines:%d ranges:%d (0x%x, 0x%x), (0x%x, 0x%x)",
			dp->name, cfg->index, cfg->flags, io->addr_lines, io->ranges,
			io->range[0].addr, io->range[0].length,
			io->range[1].addr, io->range[1].length));

		if ((cfg->flags & CISTPL_CFTABLE_TPCE_FS_IO) == 0) {
			/* No valid IO range */
			continue;
		}

		lp->conf.ConfigIndex = cfg->index;

		if (io->ranges < 2) {
			i = 0;
			lp->io.NumPorts2 = 0;
		}
		else if (io->ranges == 2) {
			i = (io->range[1].length > io->range[0].length) ? 1 : 0;
			lp->io.BasePort2.base = io->range[1 & ~i].addr;
			lp->io.NumPorts2      = io->range[1 & ~i].length + 1;
		}
		else {
			cmn_err(CE_NOTE, "%s: invalid io ranges:%d",
				dp->name, io->ranges);
			continue;
		}

		lp->io.BasePort1.base = io->range[i].addr;
		lp->io.IOAddrLines    = io->addr_lines;

		if (io->range[i].addr == 0 && io->range[i].length == 0) {
			/* port address is not specified */
			lp->io.NumPorts1 = 1 << io->addr_lines;
		} else {
			lp->io.NumPorts1 = io->range[i].length + 1;
		}

		cmn_err(CE_CONT,
			"!%s: pcni_config: requesting "
			"BasePort1:0x%x NumPorts1:%d "
			"NumPorts2:%d IOAddrLines:%d",
			dp->name,
			lp->io.BasePort1.base, lp->io.NumPorts1,
			lp->io.NumPorts2, lp->io.IOAddrLines);

		if (lp->io.NumPorts1 + lp->io.NumPorts2 < 32) {
			/* Try next entry */
			continue;
		}

		if (lp->io.NumPorts1 == 32) {
			lp->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
			if (lp->io.NumPorts2 > 0) {
				/* for master/slave multifunction cards */
				lp->io.Attributes2 = IO_DATA_PATH_WIDTH_8;
				lp->irq.Attributes = 
					IRQ_TYPE_DYNAMIC_SHARING|IRQ_FIRST_SHARED;
			}
		}
		else {
			/* This should be two 16-port windows */
			lp->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
			lp->io.Attributes2 = IO_DATA_PATH_WIDTH_16;
		}

		DPRINTF(3, (CE_CONT, "!%s: pcni_request_port: port: 0x%x",
			dp->name, lp->io.BasePort1.base));

		ret = csx_RequestIO(lp->handle, &lp->io);
		if (ret == CS_SUCCESS) {
			/* Done */
			break;
		}

	} while (csx_GetNextTuple(handle, &tuple) == CS_SUCCESS);

	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestIO", ret);
		goto failed;
	}

	ASSERT(sizeof(dp->base_addr) == sizeof(void *));
	dp->handle = lp->io.BasePort1.handle;
	(void) csx_GetMappedAddr(dp->handle, (void **)&dp->base_addr);

	cmn_err(CE_CONT, "!%s: pcni_config: allocated: BasePort1:0x%x "
		"NumPorts1:%d NumPorts2:%d IOAddrLines:%d",
		dp->name,
		dp->base_addr, lp->io.NumPorts1,
		lp->io.NumPorts2, lp->io.IOAddrLines);


	cmn_err(CE_CONT,
		"!%s: pcni_config: mem.windows:%d, length:0x%x",
		dp->name, cfg->mem.windows, cfg->mem.window[0].length);

	/*
	 * Register interrupt handler
	 */
	lp->intr_hilevel = ddi_intr_hilevel(dp->dip, 0);
	if (lp->intr_hilevel) {
		lp->irq.irq_handler = (csfunction_t *)&pcni_highintr;
	} else {
		lp->irq.irq_handler = (csfunction_t *)&gld_intr;
	}
	lp->irq.Attributes = IRQ_TYPE_EXCLUSIVE;
	lp->irq.irq_handler_arg = (void *)lp->macinfo;

	if ((ret = csx_RequestIRQ(handle, &lp->irq)) != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestIRQ", ret);
		goto failed;
	}

	c = *(lp->irq.iblk_cookie);

	if (lp->intr_hilevel) {
		/* hi-level locks */
		mutex_init(&lp->hi_intrlock, NULL, MUTEX_DRIVER, c);
		mutex_init(&dp->pagelock, NULL, MUTEX_DRIVER, c);

		/* low-level locks */
		ddi_get_soft_iblock_cookie(dp->dip, DDI_SOFTINT_MED, &c);

		mutex_init(&dp->intrlock, NULL, MUTEX_DRIVER, c);
		mutex_init(&dp->xmitlock, NULL, MUTEX_DRIVER, c);
	}
	else {
		/* low-level locks */
		mutex_init(&dp->pagelock, NULL, MUTEX_DRIVER, c);
		mutex_init(&dp->intrlock, NULL, MUTEX_DRIVER, c);
		mutex_init(&dp->xmitlock, NULL, MUTEX_DRIVER, c);
	}
	cv_init(&dp->drain_cv, NULL, CV_DRIVER, NULL);
	sema_init(&dp->rdma_sema, 1, NULL, SEMA_DRIVER, NULL);
	dp->cookie = c;

	/*
	 * RequestConfigration
	 */
	ret = csx_RequestConfiguration(handle, &lp->conf);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestConfiguration", ret);
		goto failed;
	}

	/*
	 * Get ethernet address
	 */
	if (pcni_dump_cis) {
		pcni_dump_am(dp);
	}
	if (pcni_read_macaddr_rdma(dp)) {
		goto found;
	}
	if (pcni_read_macaddr_dl(dp)) {
		/* change default mode to use auto-negotiation */
		lp->mii_fixedmode = FALSE;
		lp->flow_control  = lp->flow_control_req = TRUE;
		goto found;
	}
	if (pcni_read_macaddr_conf(dp)) {
		goto found;
	}

	cmn_err(CE_WARN, "!%s: no valid ether address found", dp->name);
	goto failed;
	
found:
	/*
	 * Configure local memory
	 */
	dp->dcr_wts = DCR_WTS;
	rx_start = MEM_START;
	rx_stop  = MEM_START + MEM_SIZE_16K;
	dp->xfer_width = XFER_16BIT;

	if (pcni_use_shmem != 0 &&
	    cfg->mem.windows == 1 &&
	    cfg->mem.window[0].length >= DP8390_PTOB(MEM_SIZE_16K)) {
		/*
		 * Try to allocate window for memory access
		 */
		if (pcni_setup_shmem_window(dp, rx_start, rx_stop, 0) == 0) {
			goto configure_done;
		}
	}

	if (pcni_auto_detect_mem_size) {
		if (pcni_probe_mem(dp, &rx_start, &rx_stop) == 0) {
			cmn_err(CE_WARN,
			"!%s: failed to detect local memoy size",
				dp->name);
			goto failed;
		}
	}

	pcni_setup_dma_config(dp, rx_start, rx_stop);

	if (pcni_auto_detect_xfer_size) {
		pcni_xfer_test(dp);
		if (dp->xfer_width == 0) {
			cmn_err(CE_WARN,
				"!%s: failed to testing rdma xfer size",
				dp->name);
			goto failed;
		}
	}

configure_done:
	dp->sendup  = &pcni_sendup;

	/*
	 * Print hardware information
	 */
	strcpy(chip_name, "Compatible");
	if (lp->chip == CHIP_DL10019) {
		sprintf(chip_name, "(DL10019 rev %02x)", INB(dp, DL_REV_PORT));
	}
	else if (lp->chip == CHIP_DL10022) {
		sprintf(chip_name, "(DL10022 rev %02x)", INB(dp, DL_REV_PORT));
	}

	cmn_err(CE_CONT, 
	"!%s: NE2000 %s: io 0x%03x hw_addr %02x:%02x:%02x:%02x:%02x:%02x",
		dp->name, chip_name, dp->base_addr,
		dp->factaddr.ether_addr_octet[0],
		dp->factaddr.ether_addr_octet[1],
		dp->factaddr.ether_addr_octet[2],
		dp->factaddr.ether_addr_octet[3],
		dp->factaddr.ether_addr_octet[4],
		dp->factaddr.ether_addr_octet[5]);

	return;

failed:
	pcni_release(dp);
	return;
}

static void
pcni_release(struct dp8390_dev *dp)
{
	struct pcni_dev	*lp;
	int			ret;
	release_config_t	rc;

	lp = (struct pcni_dev *)dp->private;

	DPRINTF(1, (CE_CONT, "!%s: pcni_release called", dp->name));

	if (lp->shmem) {
		if ((ret = csx_ReleaseWindow(lp->win)) != CS_SUCCESS) {
			pcni_cs_error(dp, "ReleaseWindow", ret);
		}
	}
	rc.Socket = lp->sn;
	ret = csx_ReleaseConfiguration(lp->handle, &rc);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "ReleaseConfiguration", ret);
	}

	ret = csx_ReleaseIO(lp->handle, &lp->io);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "ReleaseIO", ret);
	}

	ret = csx_ReleaseIRQ(lp->handle, &lp->irq);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "ReleaseIRQ", ret);
	}

	if (lp->intr_hilevel) {
		mutex_destroy(&lp->hi_intrlock);
	}

	mutex_destroy(&dp->pagelock);
	mutex_destroy(&dp->intrlock);
	mutex_destroy(&dp->xmitlock);
	cv_destroy(&dp->drain_cv);
	sema_destroy(&dp->rdma_sema);

	lp->state &= ~CARD_CONFIGURED;
}

static int
pcni_event(event_t event, int priority, event_callback_args_t *args)
{
	struct dp8390_dev	*dp;
	struct pcni_dev		*lp;
	int			ret;

	dp = (struct dp8390_dev *)args->client_data;
	lp = (struct pcni_dev *)dp->private;

	DPRINTF(2, (CE_CONT, "!%s: pcni_event called", dp->name));

	ret = CS_SUCCESS;

	if (priority & CS_EVENT_PRI_HIGH) {
		mutex_enter(&lp->event_hi_mutex);
	}
	else {
		mutex_enter(&lp->event_mutex);
	}

	switch (event) {
	case CS_EVENT_CARD_REMOVAL:
		lp->state &= ~CARD_INSERTED;
		break;

	case CS_EVENT_CARD_INSERTION:
		ASSERT(priority & CS_EVENT_PRI_LOW);
		lp->state |= CARD_INSERTED;
		pcni_config(dp);
		if (lp->state & WAITING_CARD_INSERTED) {
			cv_signal(&lp->readywait_cv);
		}
		break;

	case CS_EVENT_RESET_PHYSICAL:
		if (lp->state & CARD_CONFIGURED) {
			release_config_t	rc;

			rc.Socket = lp->sn;
			csx_ReleaseConfiguration(lp->handle, &rc);
		}
		break;

	case CS_EVENT_CARD_RESET:
		if (lp->state & CARD_CONFIGURED) {
			csx_RequestConfiguration(lp->handle, &lp->conf);
			if (dp->started) {
				mutex_enter(&dp->intrlock);

				dp8390_stop_chip(dp);

				mutex_enter(&dp->xmitlock);

				mutex_enter(&dp->pagelock);
				pcni_reset_chip(dp);
				dp8390_init_chip(dp);
				dp8390_start_chip(dp);
				mutex_exit(&dp->pagelock);

				mutex_exit(&dp->xmitlock);
				mutex_exit(&dp->intrlock);
			}
		}
		break;
	}

	if (priority & CS_EVENT_PRI_HIGH) {
		mutex_exit(&lp->event_hi_mutex);
	} else {
		mutex_exit(&lp->event_mutex);
	}
	return ret;
}

static void
pcni_reset_chip(struct dp8390_dev *dp)
{
	int	i;

	DPRINTF(2, (CE_CONT, "!%s: pcni_reset_chip: called", dp->name));

	dp->nic_state = NIC_INACTIVE;

	OUTB(dp, PORT_RESET, INB(dp, PORT_RESET));

	for (i = 1000; i > 0; i--) {
		if ((INB(dp, P0_ISR) & ISR_RST) != 0) {
			/*
			 * Done
			 */
			OUTB(dp, P0_ISR, ISR_RST);
			return;
		}
		drv_usecwait(10);
	}

	OUTB(dp, P0_ISR, ISR_RST);
    
	cmn_err(CE_WARN, "!%s: pcni_reset_chip() did not complete.", dp->name);
}


static void
pcni_watchdog(void *arg)
{
	struct dp8390_dev	*dp;
	struct pcni_dev		*lp;
	clock_t			now;
	int			kick_tx = 0;

	dp = (struct dp8390_dev *)(((gld_mac_info_t *)arg)->gldm_private);
	lp = (struct pcni_dev *)dp->private;

	DPRINTF(3, (CE_CONT, "%s: pcni_watchdog: called", dp->name));

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	/*
	 * Check TX timeout
	 */
#define	TX_TIMEOUT_TICKS	drv_usectohz(5000000)	/* 5 sec */
	now = ddi_get_lbolt();
	if (dp->nic_state == NIC_ACTIVE &&
	    dp->tx_head != NULL &&
	    now - dp->tx_start_time > TX_TIMEOUT_TICKS) {
		int	isr;
		int	txsr;

		mutex_enter(&dp->pagelock);
		isr  = INB(dp, P0_ISR);
		txsr = INB(dp, P0_TSR_R);
		mutex_exit(&dp->pagelock);

		cmn_err(CE_WARN, "!%s: tx timeout. resetting device, "
				 "%s tsr:0x%02x, isr:0x%02x imr:0x%02x",
			dp->name,
			(txsr & TSR_ABT) ? "excess collisions." :
			(isr) ? "lost interrupt?" : "cable problem?",
			txsr, isr, dp->imr_shadow);
		dp->stat.errxmt++;

		mutex_exit(&dp->xmitlock);
		dp8390_stop_chip(dp);
		mutex_enter(&dp->xmitlock);

		mutex_enter(&dp->pagelock);
		pcni_reset_chip(dp);
		dp8390_init_chip(dp);
		mutex_exit(&dp->pagelock);
		kick_tx = 1;
	}

reschedule:
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	if (kick_tx) {
		gld_sched(lp->macinfo);
	}

	dp->tx_timeout_id = timeout(pcni_watchdog, arg, WATCHDOG_INTERVAL);
}

/*====================================================================*/

static int
pcni_setup_dma_config(struct dp8390_dev *dp, int rx_start, int rx_stop)
{
	dp->ram_start	= rx_start;
	dp->ram_size	= rx_stop - rx_start;
	dp->ntxbuf	= DP8390_PTOB(dp->ram_size) / (8*KB);

	/* set block I/O functions */
	dp->get_header = &dp8390_rdma_get_header;
	dp->get_pkt    = &dp8390_rdma_get_pkt;
	dp->put_pkt    = &dp8390_rdma_put_pkt;

	return 0;
}

static int
pcni_setup_shmem_window(struct dp8390_dev *dp, int rx_start,
			      int rx_stop, int cm_offset)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;
	win_req_t	req;
	int		offset;
	int		ret;
	map_mem_page_t	mem;
	int		window_size;
	int		ramsize;
	int		i;


	/*
	 * Round window size up to power of two
	 */
	window_size = min(DP8390_PTOB(rx_stop - rx_start), 32 * KB);
	while ((window_size & (window_size - 1)) != 0) {
		window_size += window_size & ~(window_size - 1);
	}

	/*
	 * Allocate a memory window
	 */
	req.Attributes = WIN_DATA_WIDTH_16 | WIN_MEMORY_TYPE_CM |WIN_ENABLE
			 | WIN_ACC_LITTLE_ENDIAN;
	req.Base.base  = 0;
	req.Size       = window_size;
	req.win_params.AccessSpeed  = 1 | WIN_USE_WAIT;

	ret = csx_RequestWindow(lp->handle, &lp->win, &req);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "RequestWindow", ret);
		goto failed;
	}

	mem.CardOffset = DP8390_PTOB(rx_start) + cm_offset;
	offset         = mem.CardOffset % window_size;
	mem.CardOffset-= offset;
	mem.Page       = 0;
	ret = csx_MapMemPage(lp->win, &mem);
	if (ret != CS_SUCCESS) {
		pcni_cs_error(dp, "MapMemPage", ret);
		goto failed;
	}

	csx_GetMappedAddr(req.Base.handle, (void **)&lp->base);
	cmn_err(CE_CONT, "!%s: RequestWindow: handle:0x%x base:0x%x, size:0x%x",
		dp->name, req.Base.handle, lp->base, req.Size);

	/* write/read test */
	for (i = 0; i < DP8390_PTOB(6); i += 2) {
		*(uint16_t *)(lp->base + offset + i) = i;
	}

	for (i = 0; i < DP8390_PTOB(6); i += 2) {
		if (*(uint16_t *)(lp->base + offset + i) != i) {
			pcni_reset_chip(dp);
			csx_ReleaseWindow(lp->win);
			lp->base = NULL;
			lp->win = NULL;
			goto failed;
		}
	}

	dp->ram_start  = rx_start;
	dp->ram_size   = DP8390_BTOP(req.Size - offset);
	dp->ntxbuf     = DP8390_PTOB(dp->ram_size) / (8*KB);
	lp->mem_start  = lp->base + offset;
	lp->mem_end    = lp->base + req.Size;

	/* set up I/O functions */
	dp->get_header = &pcni_shmem_get_header;
	dp->get_pkt = &pcni_shmem_get_pkt;
	dp->put_pkt = &pcni_shmem_put_pkt;

	lp->shmem = TRUE;

	return 0;

failed:
	return 1;
}

static void
pcni_sendup(struct dp8390_dev *dp, mblk_t *mp)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;

	gld_recv(lp->macinfo, mp);
}

static void
dl_set_media(struct dp8390_dev *dp)
{
	struct pcni_dev *lp;

	lp = (struct pcni_dev *)dp->private;

	if (lp->chip == CHIP_DL10022) {
		/*
		 * Disable collision detection on full duplex links
		 */
		if (lp->full_duplex) {
			OUTB(dp, DL_DIAG_PORT, 4);
		} else {
			OUTB(dp, DL_DIAG_PORT, 0);
		}
	}
	return;
}

static void
dl_mii_sync(struct dp8390_dev *dp)
{
	struct pcni_dev *lp;
	uint8_t		val;
	uint8_t		mwr;
	int		i;

	lp = (struct pcni_dev *)dp->private;

	mwr = GPIO_MWR_19;
	if (lp->chip == CHIP_DL10022) {
		mwr = GPIO_MWR_22;
	}

	val = (INB(dp, DL_GPIO_PORT) & ~GPIO_MII) | mwr | GPIO_MOUT;
	for (i = 0; i < 32; i++) {
		OUTB(dp, DL_GPIO_PORT, val);
		OUTB(dp, DL_GPIO_PORT, val | GPIO_MCLK);
	}
}

static uint16_t
dl_mii_read(struct dp8390_dev *dp, int reg)
{
	struct pcni_dev *lp;
	uint32_t	cmd;
	int		i;
	uint16_t	ret;
	uint8_t		val;
	uint8_t		d;
	uint8_t		mwr;

	lp = (struct pcni_dev *)dp->private;

	mwr = GPIO_MWR_19;
	if (lp->chip == CHIP_DL10022) {
		mwr = GPIO_MWR_22;
	}

	cmd = (MII_OP_RD << MII_OP_SHIFT)
	    | (lp->phy << MII_PMD_SHIFT)
	    | (reg << MII_REG_SHIFT);

	val = INB(dp, DL_GPIO_PORT) & ~GPIO_MII;
	for (i = 31; i >= 18; i--) {
		d = ((cmd >> i) << GPIO_MOUT_SHIFT) & GPIO_MOUT;

		OUTB(dp, DL_GPIO_PORT, val | d | mwr);
		OUTB(dp, DL_GPIO_PORT, val | d | mwr | GPIO_MCLK);
	}

	ret = 0;
	for (i = 17; i >= 0; i--) {
		OUTB(dp, DL_GPIO_PORT, val);
		ret <<= 1;
		ret |= (INB(dp, DL_GPIO_PORT) >> GPIO_MIN_SHIFT) & 1;
		OUTB(dp, DL_GPIO_PORT, val | GPIO_MCLK);
	}

	OUTB(dp, DL_GPIO_PORT, val);
	OUTB(dp, DL_GPIO_PORT, val | GPIO_MCLK);

	return ret;
}

static void
dl_mii_write(struct dp8390_dev *dp, int reg, uint16_t value)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;
	uint32_t	cmd;
	int		i;
	uint8_t		d;
	uint8_t		val;
	uint8_t		mwr;

	mwr = GPIO_MWR_19;
	if (lp->chip == CHIP_DL10022) {
		mwr = GPIO_MWR_22;
	}

	cmd = (MII_OP_WR << MII_OP_SHIFT)
	    | (lp->phy << MII_PMD_SHIFT)
	    | (reg << MII_REG_SHIFT)
	    | (MII_LT_WR << MII_LT_SHIFT)
	    | value;

	val = INB(dp, DL_GPIO_PORT) & ~GPIO_MII;
	for (i = 31; i >= 0; i--) {
		d = ((cmd >> i) << GPIO_MOUT_SHIFT) & GPIO_MOUT;

		OUTB(dp, DL_GPIO_PORT, val | d | mwr);
		OUTB(dp, DL_GPIO_PORT, val | d | mwr | GPIO_MCLK);
	}

	OUTB(dp, DL_GPIO_PORT, val);
	OUTB(dp, DL_GPIO_PORT, val | GPIO_MCLK);
	OUTB(dp, DL_GPIO_PORT, val);
	OUTB(dp, DL_GPIO_PORT, val | GPIO_MCLK);
	OUTB(dp, DL_GPIO_PORT, val);
}

/* ======================================================================== */
/*
 * General MII support routines
 */
/* ======================================================================== */

#define	MII_SYNC(dp)		dl_mii_sync(dp)
#define	MII_READ(dp, reg)	dl_mii_read(dp, reg)
#define	MII_WRITE(dp, reg, val)	dl_mii_write(dp, reg, val)

static int
pcni_mii_config(struct dp8390_dev *dp)
{
	uint16_t	mii_stat;
	uint16_t	val;
	int		i, j;
	uint32_t	phyid;
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;

	DPRINTF(3, (CE_CONT, "!%s: pcni_mii_config: called", dp->name));

	if (lp->mii_fixedmode) {
		val = 0;
		if (lp->full_duplex) {
			val |= MII_CONTROL_FDUPLEX;
		}
		if (dp->speed100) {
			val |= MII_CONTROL_100MB;
		}
		MII_WRITE(dp, MII_CONTROL, val);
		return 0;
	}

	/*
	 * Set advertisement register
	 */
	mii_stat = MII_READ(dp, MII_STATUS);
	DPRINTF(3, (CE_CONT, "!%s: pcni_mii_config: MII_STATUS reg:%b",
		dp->name, mii_stat, MII_STATUS_BITS));

	if ((mii_stat & MII_STATUS_ABILITY) == 0) {
		/* it's funny */
		cmn_err(CE_WARN, "!%s: wrong ability bits: mii_status:%b",
			dp->name, mii_stat, MII_STATUS_BITS);
		return -1;
	}

	/* Do not change rest of ability bits in advert reg */
	val = MII_READ(dp, MII_AN_ADVERT) & ~MII_ABILITY;

	/*
	 * XXX - don't support 100M mode in default, which cause
	 * to run over rx ring.
	 */
	if (pcni_enable_100) {
		if ((mii_stat & MII_STATUS_100_BASE_T4) != 0) {
			val |= MII_ABILITY_100BASE_T4;
		}
		if ((mii_stat & MII_STATUS_100_BASEX_FD) != 0) {
			val |= MII_ABILITY_100BASE_TX_FD;
		}
		if ((mii_stat & MII_STATUS_100_BASEX) != 0) {
			val |= MII_ABILITY_100BASE_TX;
		}
	}

	if ((mii_stat & MII_STATUS_10_FD) != 0) {
		val |= MII_ABILITY_10BASE_T;
	}
	if ((mii_stat & MII_STATUS_10) != 0) {
		val |= MII_ABILITY_10BASE_T_FD;
	}
#ifdef NO_PAUSE
	val &= ~ MII_ABILITY_PAUSE;
#endif
	DPRINTF(3, (CE_CONT,
		"!%s: pcni_mii_config: setting MII_AN_ADVERT reg:%b",
		dp->name, val, MII_ABILITY_BITS));

	MII_WRITE(dp, MII_AN_ADVERT, val);

	return 0;
}

static void
pcni_mii_link_watcher(struct dp8390_dev *dp)
{
	uint16_t	mii_stat;
	uint16_t	advert;
	uint16_t	lpable;
	uint16_t	val;
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;

	/* read PHY status */
	mii_stat = MII_READ(dp, MII_STATUS);
	DPRINTF(4, (CE_CONT,
	"!%s: pcni_mii_link_watcher: called: mii_state:%d MII_STATUS reg:%b",
		dp->name, lp->mii_state,
		mii_stat, MII_STATUS_BITS));

	if (lp->mii_state == MII_STATE_RESETTING) {
		if (--lp->mii_timer > 0) {
			/* wait for time-up */
			goto next;
		}
		/* Timer expired, ensure reset bit is not set */
		MII_SYNC(dp);
		if ((MII_READ(dp, MII_CONTROL) & MII_CONTROL_RESET) != 0) {
			/* reset have not done, try reset again */
			cmn_err(CE_WARN, "!%s: resetting mii not complete.",
				dp->name);
			goto reset_phy;
		}
		/* Configure PHY registers */
		if (pcni_mii_config(dp) != 0) {
			goto reset_phy;
		}

		if (lp->mii_fixedmode) {
			lp->mii_state = MII_STATE_LINKDOWN;
			lp->mii_timer = MII_LINKDOWN_TIMEOUT;
			goto next;
		}
		else {
			/* Issue auto-negotiation command */
			goto autonego;
		}
	}

	if (lp->mii_state == MII_STATE_AUTONEGOTIATING) {
		/*
		 * Autonegotiation in progress
		 */
		lp->mii_timer--;

		if ((mii_stat & MII_STATUS_REMFAULT) != 0) {
			/*
			 * link partner does not have auto nego capability
			 * What do we do ?
			 */
			cmn_err(CE_CONT, "!%s: auto nego remote fault",
				dp->name);
			goto reset_phy;
		}

		if ((mii_stat & MII_STATUS_ANDONE) == 0) {
			if (lp->mii_timer == 0) {
				/*
				 * Auto-negotiation timed out,
				 * Reset PHY and try again.
				 */
				if (!lp->mii_supress_msg) {
					cmn_err(CE_WARN,
					"!%s: autonegotiation timed out.",
					dp->name);
					lp->mii_supress_msg = TRUE;
				}
				goto reset_phy;
			}
			/*
			 * Auto-negotiation in progress. Wait.
			 */
			goto next;
		}
		/*
		 * Auto-negotiation have done.
		 * Assume that we are link down state and fall through.
		 */
		lp->mii_state = MII_STATE_LINKDOWN;
		lp->mii_timer = MII_LINKDOWN_TIMEOUT;
		cmn_err(CE_CONT, "!%s: auto-negotiation done", dp->name);

		/*
		 * 06/02/2002
		 * Some PHY needs delay before reading LPAR reg
		 */
		goto next;
	}

	lp->mii_supress_msg = FALSE;

	if (lp->mii_state != MII_STATE_LINKDOWN &&
	    ((mii_stat & MII_STATUS_LINKUP) == 0)) {
		/*
		 * link going down
		 */
		cmn_err(CE_NOTE, "!%s: link down detected: mii_stat:%b",
			dp->name, mii_stat, MII_STATUS_BITS);

		lp->mii_state = MII_STATE_LINKDOWN;
		lp->mii_timer = MII_LINKDOWN_TIMEOUT;
	}
	else if (lp->mii_state != MII_STATE_LINKUP &&
		((mii_stat & MII_STATUS_LINKUP) != 0)) {
		/*
		 * Link going up
		 */
		lp->mii_state = MII_STATE_LINKUP;

		DPRINTF(1, (CE_CONT, "!%s: link up detected: mii_stat:%b",
			dp->name, mii_stat, MII_STATUS_BITS));

		if (!lp->mii_fixedmode) {
			/*
			 * Auto negosiation
			 */
			/* determine full/half and 100mbps/10mbps */
			advert = MII_READ(dp, MII_AN_ADVERT);
			lpable = MII_READ(dp, MII_AN_LPABLE);

			DPRINTF(1, (CE_CONT,
			"!%s: pcni_mii_link_watcher: advert:%b, lpable:%b",
				dp->name,
				advert, MII_ABILITY_BITS,
				lpable, MII_ABILITY_BITS));
			/*
			 * configure link mode according to AN priority.
			 */	
			val = advert & lpable;
			if ((val & MII_ABILITY_100BASE_TX_FD) != 0) {
				/* 100BaseTx & fullduplex */
				dp->speed100    = TRUE;
				lp->full_duplex = TRUE;
				lp->flow_control =
				      (val & MII_ABILITY_PAUSE) ? TRUE : FALSE;
			}
			else if ((val & MII_ABILITY_100BASE_TX) != 0) {
				/* 100BaseTx & half duplex */
				dp->speed100    = TRUE;
				lp->full_duplex = FALSE;
				lp->flow_control= FALSE;
			}
			else if ((val & MII_ABILITY_10BASE_T_FD) != 0) {
				/* 10BaseT & full duplex */
				dp->speed100    = FALSE;
				lp->full_duplex = TRUE;
				lp->flow_control =
				      (val & MII_ABILITY_PAUSE) ? TRUE : FALSE;
			}
			else if ((val & MII_ABILITY_10BASE_T) != 0) {
				/* 10BaseT & half duplex */
				dp->speed100    = FALSE;
				lp->full_duplex = FALSE;
				lp->flow_control= FALSE;
			}
			else {
				/* No solution */
				cmn_err(CE_WARN,
	"!%s: link up but auto-nego failed, no common mode with link partner, "
					"advert:%b lpable:%b",
					dp->name,
					advert, MII_ABILITY_BITS,
					lpable, MII_ABILITY_BITS);
				goto reset_phy;
			}

			mutex_enter(&dp->intrlock);
			dl_set_media(dp);
			mutex_exit(&dp->intrlock);
		}

		/*
		 * MII_CONTROL_100MB and  MII_CONTROL_FDUPLEX are ignored
		 * when MII_CONTROL_ANEN is set.
		 */
		cmn_err(CE_CONT,
			"!%s: Link up: %d Mbps %s duplex %s flow control",
			dp->name,
			dp->speed100 ? 100 : 10,
			lp->full_duplex ? "full" : "half",
			lp->flow_control ? "with" : "without");
	}
	else {
		/* link status not changed */
		if (!lp->mii_fixedmode &&
		     (lp->mii_state == MII_STATE_LINKDOWN)) {
			lp->mii_timer--;
			if (lp->mii_timer <= 0) {
				goto reset_phy;
			}
		}
	}
	goto next;

reset_phy:
	if (!lp->mii_supress_msg) {
		cmn_err(CE_CONT, "!%s: resetting PHY", dp->name);
	}
	lp->mii_state = MII_STATE_RESETTING;
	lp->mii_timer = MII_RESET_TIMEOUT;

	MII_WRITE(dp, MII_CONTROL, MII_CONTROL_RESET);
	goto next;

autonego:
	if (!lp->mii_supress_msg) {
		cmn_err(CE_CONT, "!%s: start auto-negotiation", dp->name);
	}
	lp->mii_state = MII_STATE_AUTONEGOTIATING;
	lp->mii_timer = MII_AUTONEGO_TIMEOUT;

	val = MII_READ(dp, MII_CONTROL);
	MII_WRITE(dp, MII_CONTROL,
		 val | MII_CONTROL_ANE | MII_CONTROL_RSAN);
	/* fall through */

next:
	lp->link_watcher_id =
		timeout((void (*)(void *))& pcni_mii_link_watcher,
				(void *)dp, LINK_WATCH_INTERVAL);

	return;
}

static int
pcni_mii_init(struct dp8390_dev *dp)
{
	struct pcni_dev *lp = (struct pcni_dev *)dp->private;
	int		phy;
	uint16_t	adv;
	uint16_t	status;
	uint32_t	phyid;

	DPRINTF(3, (CE_CONT, "!%s: pcni_mii_init: called", dp->name));

	if (lp->chip != CHIP_DL10019 && lp->chip != CHIP_DL10022) {
		return 0;
	}

	MII_SYNC(dp);

	/*
	 * Scan PHY
	 */
	for (phy = 0; phy < 32; phy++) {
		lp->phy = phy;
		status = MII_READ(dp, MII_STATUS);

		if (status != 0xffff && status != 0x0000) {
			goto PHY_found;
		}
	}

	cmn_err(CE_WARN, "!%s: failed to find PHY", dp->name);
	return -1;

PHY_found:
	phyid  = MII_READ(dp, MII_PHYIDH) << 16;
	phyid |= MII_READ(dp, MII_PHYIDL);
	adv = MII_READ(dp, MII_AN_ADVERT);
	cmn_err(CE_CONT, "!%s: PHY (0x%08x) found at %d,\n"
		"status:%b,\nadvert:%b,\nlpar:%b",
		dp->name, phyid, lp->phy,
		status, MII_STATUS_BITS,
		adv, MII_ABILITY_BITS,
		MII_READ(dp, MII_AN_LPABLE),
		MII_ABILITY_BITS);

	/* reset PHY */
	lp->mii_state = MII_STATE_RESETTING;
	lp->mii_timer = MII_RESET_TIMEOUT;
	MII_WRITE(dp, MII_CONTROL, MII_CONTROL_RESET);

	/* schedule first call of rh_link_watcher */
	lp->link_watcher_id =
		timeout((void (*)(void *))pcni_mii_link_watcher,
				(void *)dp, LINK_WATCH_INTERVAL);
	return 0;
}

static void
pcni_mii_stop(struct dp8390_dev *dp)
{
	timeout_id_t	old_id;
	struct pcni_dev *lp;

	lp = (struct pcni_dev *)dp->private;

	if (lp->chip != CHIP_DL10019 && lp->chip != CHIP_DL10022) {
		return;
	}

	/* Ensure timer routine stopped */
	if (lp->link_watcher_id != 0) {
		do {
			untimeout(old_id = lp->link_watcher_id);
		} while (old_id != lp->link_watcher_id);
		lp->link_watcher_id = 0;
	}
}
