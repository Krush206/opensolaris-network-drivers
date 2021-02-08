/*
 * @(#)ni.c	1.11 06/01/21
 * Solaris Multithreaded STREAMS DLPI NE2000 compatible PCI Ethernet Driver
 * Revised from Solaris ddk.
 *
 * Copyright (c) 2002-2004 Masayuki Murayama.  All rights reserved.
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
 08/02/2002 0.8.0 release

	0.8.2 release

 09/08/2002
	niprobe deleted

 07/28/2003 0.8.4
	modified for 8bit mode card and tested with gcc3
	IN and OUT macros fixed. multicast hash implemented.

 06/19/2004 0.8.5
	PnP ISA devices supported

 */
/*
 TODO:
  HOLTEK 80229  only 32 bit I/O and local memory is 8KB
 */


/* Copyright from Solaris DDK (Devece driver Development Kit)
/*
 * Copyright (c) 1991-1994,1997, by Sun Microsystems, Inc.
 * All Rights Reserved
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
#ifdef SUPPORT_ISA
#include <sys/ddi_impldefs.h>
#endif
#include <sys/sunddi.h>
#include <sys/pci.h>

#include <sys/gld.h>

#include "dp8390reg.h"
#include "dp8390var.h"


char	ident[] = "NE2000 driver v" VERSION;
char   _depends_on[] = {"misc/gld drv/dp8390"};

#define	FALSE	(0)
#define	TRUE	(!FALSE)

#ifdef DEBUG_LEVEL
static int	ni_debug = DEBUG_LEVEL;
#define	DPRINTF(n, args)	if (ni_debug > (n)) cmn_err args
#else
static int	ni_debug = 0;
#define	DPRINTF(n, args)
#endif

/*
 * Our hardware configurations
 */
#define	NIMAXMC		(64)
#define	NIMCALLOC	(NIMAXMC * sizeof(struct mcast_addr)) 

/* I/O port */
#define PORT_DATA	0x10
#define PORT_RESET	0x1f

/* Local memory configutation */
#define MEM_START	0x40
#define MEM_SIZE_8K 	0x20
#define MEM_SIZE_16K	0x40
#define	SA_PROM_SIZE	16
#define	SA_PROM_ADDR	0

/* Timeouts */
#define	WATCHDOG_INTERVAL	drv_usectohz(1000000)	/* 1 sec */
#define	TX_TIMEOUT_TICKS	drv_usectohz(5000000)	/* 5 sec */
#ifdef AUTO_XFER_SIZE
static int ni_auto_detect_xfer_size = 1;
#endif
/*
 * Function prototypes.
 */
/* private functions */
static	void ni_probe_SAprom(struct dp8390_dev *, uint8_t *);
static	int ni_reset_chip(struct dp8390_dev *);
static	void ni_watchdog(void *);

/* DDI/DKI functions */
static	int niattach(dev_info_t *, ddi_attach_cmd_t);
static	int nidetach(dev_info_t *, ddi_detach_cmd_t);
static	int niinfo(dev_info_t *, ddi_info_cmd_t, void *, void **);

/* GLD interface */
static	int ni_reset(gld_mac_info_t *);
static	int ni_start(gld_mac_info_t *);
static	int ni_stop(gld_mac_info_t *);
static	int ni_set_mac_addr(gld_mac_info_t *, unsigned char *);
static	int ni_set_multicast(gld_mac_info_t *, unsigned char *, int);
static	int ni_set_promiscuous(gld_mac_info_t *, int);
static	int ni_get_stats(gld_mac_info_t *, struct gld_stats *);
static	int ni_send(gld_mac_info_t *, mblk_t *);
static	u_int ni_intr(gld_mac_info_t *);

/* local interface for dp8390 core */
static	void ni_sendup_msg(struct dp8390_dev *, mblk_t *);
static	void ni_rx_overrun_continue(struct dp8390_dev *);

/*
 * STREAMS structures
 */
/* Data access requirements. */
static struct ddi_device_acc_attr dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

static	struct module_info niminfo = {
	0,			/* mi_idnum */
	"ni",			/* mi_idname */
	0,			/* mi_minpsz */
	ETHERMTU,		/* mi_maxpsz */
	32*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit nirinit = {
	(int (*)()) NULL, /* qi_putp */
	gld_rsrv,	/* qi_srvp */
	gld_open,	/* qi_qopen */
	gld_close,	/* qi_qclose */
	(int (*)()) NULL, /* qi_qadmin */
	&niminfo,	/* qi_minfo */
	NULL		/* qi_mstat */
};

static	struct qinit niwinit = {
	gld_wput,	/* qi_putp */
	gld_wsrv,	/* qi_srvp */
	(int (*)()) NULL, /* qi_qopen */
	(int (*)()) NULL, /* qi_qclose */
	(int (*)()) NULL, /* qi_qadmin */
	&niminfo,	/* qi_minfo */
	NULL		/* qi_mstat */
};

static struct streamtab	ni_info = {
	&nirinit,	/* st_rdinit */
	&niwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_ni_ops = {
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
	&ni_info,	/* cb_stream */
	D_MP		/* cb_flag */
};

static	struct dev_ops ni_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gld_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	niattach,	/* devo_attach */
	nidetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_ni_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	ddi_power	/* devo_power */
};

static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&ni_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};


/*
 * Ethernet broadcast address definition.
 */
static	struct ether_addr	etherbroadcastaddr = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/*
 * _init
 */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, "ni: _init: called"));

	status = mod_install(&modlinkage);
	return (status);
}

/*
 * _fini
 */
int
_fini(void)
{
	int	status;

	if ((status = mod_remove(&modlinkage)) == 0) {
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

/*
 * Probe for device.: not needed
 */

static void
ni_program_rdma(struct dp8390_dev *dp, int rcmd, uint16_t addr, uint16_t bytes)
{
	/* Select page 0 and stop H/W */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

	OUTB(dp, P0_DCR_W, DCR_FT_8 | DCR_LS_NORMAL | dp->dcr_wts);

	OUTB(dp, P0_RBCR0_W, 0x00);		/* reset remote dma */
	OUTB(dp, P0_RBCR1_W, 0x00);
	OUTB(dp, P0_IMR_W,   0x00);		/* inhibit interrupts */
	OUTB(dp, P0_ISR,     0xff);		/* clear interrupts */
	OUTB(dp, P0_RCR_W,   RCR_MON);		/* disable rx */
	OUTB(dp, P0_TCR_W,   TCR_LB_NIC);	/* disable tx */
	OUTB(dp, P0_RBCR0_W, bytes),
	OUTB(dp, P0_RBCR1_W, bytes >> 8);
	OUTB(dp, P0_RSAR0_W, addr);		/* prom starts from addr 0 */
	OUTB(dp, P0_RSAR1_W, addr >> 8);
	OUTB(dp, PX_CR, rcmd | CR_STA);		/* start */
}

static void
ni_probe_SAprom(struct dp8390_dev *dp, uint8_t *SA_prom)
{
	int	i;

	ni_program_rdma(dp, CR_RD_RRD,
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

	if (ni_debug > 0) {
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

	/* Issue STP command to stop NIC */
	OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);
}
#ifdef AUTO_XFER_SIZE
static void
ni_xfer_test(struct dp8390_dev *dp)
{
	uint32_t	ptn[4];
	uint32_t	xferd[4];
	int		i;
	int		flags;
	int		addr;
	int		len;

	addr = DP8390_PTOB(dp->ram_start);	/* use first RAM page */
	len  = sizeof(ptn);
	flags = 0;

	/* test pattarn */
	ptn[0] = 0x44332211;
	ptn[1] = 0x88776655;
	ptn[2] = 0x44332211;
	ptn[3] = 0x88776655;

	/*
	 * Test 1: 8bit transfer write/read test
	 */
	len = sizeof(ptn) - sizeof(uint8_t);
	bzero(xferd, len);
	ni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint8_t); i++) {
		OUTB(dp, PORT_DATA, ((uint8_t *)ptn)[i]);
	}
	ni_program_rdma(dp, CR_RD_RRD, addr, len);
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
	len = sizeof(ptn) - sizeof(uint16_t);
	bzero(xferd, len);
	ni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint16_t); i++) {
		OUTW(dp, PORT_DATA, ((uint16_t *)ptn)[i]);
	}
	ni_program_rdma(dp,CR_RD_RRD, addr, len);
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
	len = sizeof(ptn) - sizeof(uint32_t);
	bzero(xferd, len);
	ni_program_rdma(dp, CR_RD_RWR, addr, len);
	for (i = 0; i < len/sizeof(uint32_t); i++) {
		OUTL(dp, PORT_DATA, ((uint32_t *)ptn)[i]);
	}
	ni_program_rdma(dp, CR_RD_RRD, addr, len);
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
	return;
}
#endif /* AUTO_XFER_SIZE */
/*
 * Interface exists: make available by filling in network interface
 * record.  System will initialize the interface when it is ready
 * to accept packets.
 */
static int
niattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct dp8390_dev	*dp;
	int			i;
	int			n;
	ddi_iblock_cookie_t	c;
	ddi_acc_handle_t	conf_handle;
	uint16_t		comm;
	uint8_t			SA_prom[SA_PROM_SIZE];
	char			*card;
	gld_mac_info_t		*macinfo;
	int			unit;
	uint_t			len;
	void			*regs;
	boolean_t		is8029as;
	boolean_t		is_pci = FALSE;

	unit = ddi_get_instance(dip);

	DPRINTF(2, (CE_CONT, "niattach: called"));

	if (cmd == DDI_ATTACH) {
		DPRINTF(2, (CE_CONT, "niattach: called cmd:ATTACH"));
		macinfo = gld_mac_alloc(dip);
		if (macinfo == NULL) {
			return DDI_FAILURE;
		}
		/*
		 * Allocate soft data structure
		 */
		dp = (struct dp8390_dev *)
			kmem_zalloc(sizeof(*dp) + NIMCALLOC, KM_SLEEP);
		if (dp == NULL) {
			gld_mac_free(macinfo);
			return DDI_FAILURE;
		}

		dp->mc_count_req = 0;
		dp->mc_count = 0;
		dp->mc_list = (struct mcast_addr *) &dp[1];

		dp->dip = dip;
		sprintf(dp->name, "%s%d", ddi_driver_name(dip), unit);

		macinfo->gldm_devinfo = dip;
		macinfo->gldm_private = (caddr_t)dp;

		macinfo->gldm_reset	= ni_reset;
		macinfo->gldm_start	= ni_start;
		macinfo->gldm_stop	= ni_stop;
		macinfo->gldm_set_mac_addr = ni_set_mac_addr;
		macinfo->gldm_send	= ni_send;
		macinfo->gldm_set_promiscuous = ni_set_promiscuous;
		macinfo->gldm_get_stats	= ni_get_stats;
		macinfo->gldm_ioctl	= NULL; 
		macinfo->gldm_set_multicast= ni_set_multicast;
		macinfo->gldm_intr	= ni_intr;
		macinfo->gldm_mctl	= NULL;

		macinfo->gldm_ident	= ident;
		macinfo->gldm_type	= DL_ETHER;
		macinfo->gldm_minpkt	= 0;
		macinfo->gldm_maxpkt	= ETHERMTU;
		macinfo->gldm_addrlen	= ETHERADDRL;
		macinfo->gldm_saplen	= -2;
		macinfo->gldm_ppa	= unit;

		/*
		 * Get iblock cookie
		 */
		if (ddi_get_iblock_cookie(dip, 0, &c) != DDI_SUCCESS) {
			cmn_err(CE_CONT,
				"niattach: ddi_get_iblock_cookie: failed");
			goto err_get_cookie;
		}

		/*
		 * Initialize mutex's for this device.
		 */
		dp->cookie = c;
		macinfo->gldm_cookie = c;
		mutex_init(&dp->intrlock, NULL, MUTEX_DRIVER, (void *)c);
		mutex_init(&dp->xmitlock, NULL, MUTEX_DRIVER, (void *)c);
		mutex_init(&dp->pagelock, NULL, MUTEX_DRIVER, (void *)c);
		cv_init(&dp->drain_cv, NULL, CV_DRIVER, NULL);
		sema_init(&dp->rdma_sema, 1, NULL, SEMA_DRIVER, NULL);

		/*
		 * Map in the device registers.
		 */

		/* Search IO-range */
		len  = 0;
		regs = NULL;
		if (ddi_prop_lookup_int_array(
			DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
			"reg", (int **)&regs, &len) != DDI_PROP_SUCCESS) {
			cmn_err(CE_WARN,
				"%s: failed to get reg property", dp->name);
			goto err_get_cookie;
		}
		ASSERT(regs != NULL && len > 0);
#ifdef SUPPORT_ISA
		if (strncmp(ddi_driver_name(ddi_get_parent(dip)), "isa", 3)
		    == 0) {
			struct regspec		*isa_regs;

			isa_regs = (struct regspec *)regs;
			n = (len * sizeof(int)) / sizeof(struct regspec);
			for (i = 0; i < n; i++) {
				cmn_err(CE_CONT,
				"!%s: isa_regs[%d]: %08x.%08x.%08x",
					dp->name, i,
					isa_regs[i].regspec_bustype,
					isa_regs[i].regspec_addr,
					isa_regs[i].regspec_size);
			}

			for (i = 1; i < n; i++) {
				if (isa_regs[i].regspec_bustype & 0x80000000) {
					/* special to PnP device */
					continue;
				}
				if (isa_regs[i].regspec_bustype == 1) {
					ddi_prop_free(isa_regs);
					goto io_range_found;
				}
			}
		} else
#endif /* SUPPORT_ISA */
		{
			struct pci_phys_spec	*pci_regs;

			pci_regs = (struct pci_phys_spec *)regs;
			is_pci = TRUE;

			n = (len * sizeof(int)) / sizeof(struct pci_phys_spec);
			for (i = 0; i < n; i++) {
				cmn_err(CE_CONT,
				"!%s: pci_regs[%d]: %08x.%08x.%08x.%08x.%08x",
					dp->name, i,
					pci_regs[i].pci_phys_hi,
					pci_regs[i].pci_phys_mid,
					pci_regs[i].pci_phys_low,
					pci_regs[i].pci_size_hi,
					pci_regs[i].pci_size_low);
			}

			for (i = 0; i < n; i++) {
				if ((pci_regs[i].pci_phys_hi & PCI_REG_ADDR_M)
				     == PCI_ADDR_IO) {
					ddi_prop_free(regs);
					goto io_range_found;
				}
			}
		}

		cmn_err(CE_WARN, "%s: failed to find IO space", dp->name);
		ddi_prop_free(regs);
		goto err_get_cookie;

io_range_found:
		if (ddi_regs_map_setup(dip, i, (caddr_t *)&dp->base_addr,
					0, 0, &dev_attr, &dp->handle)) {
			cmn_err(CE_NOTE, "%s: ddi_regs_map_setup failed",
				dp->name);
			goto err_mapin;
		}

#ifdef PCI_SUPPORT
		if (is_pci) {
			/*
			 * Make sure the I/O access enable. bits are set
			 * in the config command register
			 */
			if (pci_config_setup(dip, &conf_handle)!=DDI_SUCCESS) {
				cmn_err(CE_NOTE,
					"%s: ddi_regs_map_setup failed",
					dp->name);
				goto err_mapin;
			}
			comm = pci_config_get16(conf_handle, PCI_CONF_COMM);
			comm |= PCI_COMM_IO;
			pci_config_put16(conf_handle, PCI_CONF_COMM, comm);
			pci_config_teardown(&conf_handle);
		}
#endif /* PCI_SUPPORT */

		/*
		 * Stop the chip.
		 */
		if (ni_reset_chip(dp) != 0) {
			cmn_err(CE_NOTE, "%s: ni_reset_chip failed", dp->name);
			goto err_mapin;
		}

		/*
		 * Probe SA_prom and get the vendor ethernet address.
		 */
#ifdef BYTE_MODE_TEST
		/* for prom bytye-probe test */
		dp->dcr_wts = 0;
		ni_probe_SAprom(dp, SA_prom);
		goto read_prom_ok;
#endif
		/* try word transfer mode first */
		dp->dcr_wts = DCR_WTS;
		ni_probe_SAprom(dp, SA_prom);
		if (SA_prom[14] == 'W' && SA_prom[15] == 'W') {
			/* prom was read successfully */
			goto read_prom_ok;
		}

		/* try byte transfer mode next */
		dp->dcr_wts = 0;
		ni_probe_SAprom(dp, SA_prom);

		if (SA_prom[14] == 'B' && SA_prom[15] == 'B') {
			/* prom was read successfully */
			goto read_prom_ok;
		}

		cmn_err(CE_WARN,
			"%s: niattach: SA_eeprom is funny, assuming byte-mode",
			dp->name);

read_prom_ok:
		for (i = 0; i < ETHERADDRL; i++) {
			dp->factaddr.ether_addr_octet[i] =
			dp->curraddr.ether_addr_octet[i] = SA_prom[i];
		}
		macinfo->gldm_vendor_addr =
				dp->factaddr.ether_addr_octet;
		macinfo->gldm_broadcast_addr =
				etherbroadcastaddr.ether_addr_octet;

		/*
		 * Check local memory size and bus width
		 */
		OUTB(dp, PX_CR, CR_PS_3 | CR_RD_ABORT | CR_STP);
		is8029as = (INB(dp, 0xe) == 0x29 && INB(dp, 0xf) == 0x80);
		OUTB(dp, PX_CR, CR_PS_0 | CR_RD_ABORT | CR_STP);

		if (dp->dcr_wts != 0) {
			card          = "NE2000 16bit";
			dp->ram_start = MEM_START;
			dp->ram_size  = MEM_SIZE_16K;
			dp->xfer_width= XFER_16BIT;
			if (is8029as) {
				dp->xfer_width |= XFER_32BIT;
				card = "NE2000 16bit (rtl8029as)";
			}
			dp->ntxbuf  = 4;
		}
		else {
			card          = "NE2000 8bit";
			dp->ram_start = MEM_START;
			dp->ram_size  = MEM_SIZE_8K;
			dp->xfer_width= XFER_8BIT;
			dp->ntxbuf  = 2;
		}

		DPRINTF(0, (CE_CONT, "!%s: %s", dp->name, card));

		/*
		 * Test data xfer capability through PCI bus
		 * If the system cause panic, Do not test this.
		 */
#ifdef AUTO_XFER_SIZE
		if (ni_auto_detect_xfer_size) {
			ni_xfer_test(dp);
		}
#endif /* AUTO_XFER_SIZE */

		/*
		 * Initialize dp8390 core and register reset and
		 * remote DMA routines
		 */
		if (dp8390_attach(dp) != 0) {
			cmn_err(CE_NOTE, "%s: niattach: dp8390_attach failed",
					dp->name);
			goto err_mapin;
		}

		dp->sendup  = &ni_sendup_msg;
		dp->rx_overrun_continue  = &ni_rx_overrun_continue;

		/*
		 * Add interrupt to system.
		 */
		if (gld_register(dip, "ni", macinfo) != DDI_SUCCESS) {
			goto err_add_intr;
		}

		if (ddi_add_intr(dip, 0, NULL, NULL, gld_intr,
					(caddr_t)macinfo) != DDI_SUCCESS) {
			ddi_regs_map_free(&dp->handle);
			cmn_err(CE_NOTE, "%s: ddi_add_intr failed", dp->name);
			goto err_add_intr;
		}

		DPRINTF(2, (CE_CONT, "%s: niattach: return: successed",
			dp->name));
		return (DDI_SUCCESS);

err:
		/* release allocated resource */
err_add_intr:
		(void) ddi_remove_intr(dip, 0, c);
		ddi_regs_map_free(&dp->handle);
err_mapin:
		mutex_destroy(&dp->intrlock);
		mutex_destroy(&dp->xmitlock);
		mutex_destroy(&dp->pagelock);
		cv_destroy(&dp->drain_cv);
		sema_destroy(&dp->rdma_sema);
err_get_cookie:
		kmem_free((caddr_t)dp, sizeof(*dp) + NIMCALLOC);

		cmn_err(CE_WARN, "ni%d: niattach: failed", unit);

		return DDI_FAILURE;
	}
	return DDI_FAILURE;
}

static int
nidetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	int			i;
	struct dp8390_dev	*dp;
	gld_mac_info_t		*macinfo;

	if (cmd == DDI_DETACH) {
		macinfo = (gld_mac_info_t *)ddi_get_driver_private(dip);
		dp = (struct dp8390_dev *)macinfo->gldm_private;

		dp8390_detach(dp);

		/*
		 * unregister interrupt handler
		 */
		ddi_remove_intr(dip, 0, dp->cookie);
		if (gld_unregister(macinfo) != DDI_SUCCESS) {
			return (DDI_FAILURE);
		}

		mutex_destroy(&dp->xmitlock);
		mutex_destroy(&dp->intrlock);
		mutex_destroy(&dp->pagelock);
		cv_destroy(&dp->drain_cv);
		sema_destroy(&dp->rdma_sema);

		/*
		 * Release memory and mapping resources
		 */
		ddi_regs_map_free(&dp->handle);
		kmem_free((caddr_t)dp, sizeof(*dp) + NIMCALLOC);
		gld_mac_free(macinfo);
		DPRINTF(2, (CE_CONT, "%s: nidetach: return: success",
			dp->name));

		return (DDI_SUCCESS);
	} 
	return (DDI_FAILURE);
}

/*=================================================================*/
/*
 * GLD interface
 */
/*=================================================================*/

static int
ni_reset(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	dp->started = FALSE;

	mutex_enter(&dp->pagelock);
	dp8390_init_chip(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
ni_set_mac_addr(gld_mac_info_t *macinfo, unsigned char *addr)
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
ni_start(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	mutex_enter(&dp->pagelock);
	dp8390_start_chip(dp);
	mutex_exit(&dp->pagelock);

	dp->started = TRUE;

	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

        /* kick tx watchdog */
	dp->tx_timeout_id =
		timeout(ni_watchdog, (void *)macinfo, WATCHDOG_INTERVAL);

	return GLD_SUCCESS;
}

static int
ni_stop(gld_mac_info_t *macinfo)
{
	struct dp8390_dev	*dp;
	timeout_id_t		old_id;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	if (dp->tx_timeout_id != 0) {
		do {
			(void) untimeout(old_id = dp->tx_timeout_id);
		} while (old_id != dp->tx_timeout_id);
		dp->tx_timeout_id = 0;
	}

	/* call device dependant close routine */
	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	dp->started = FALSE;
	mutex_exit(&dp->xmitlock);

	/* stop Rx and Tx */
	dp8390_stop_chip(dp);

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
ni_set_multicast(gld_mac_info_t *macinfo, unsigned char *ep, int flag)
{
	struct dp8390_dev	*dp;
	size_t			len;
	int			i;
	int			cnt;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	cnt = dp->mc_count;

	if (flag == GLD_MULTI_ENABLE) {
		/* append new addess into mc_list */
		if (cnt < NIMAXMC) {
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
			/* squeeze mc_list by copying forward */
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
	if (dp->mc_count != dp->mc_count_req) {
		dp->rxmode |= RXMODE_MULTI_OVF;
	}
	mutex_enter(&dp->pagelock);
	dp8390_set_rx_filter(dp);
	mutex_exit(&dp->pagelock);

	mutex_exit(&dp->intrlock);

	return GLD_SUCCESS;
}

static int
ni_set_promiscuous(gld_mac_info_t *macinfo, int flag)
{
	struct dp8390_dev *dp;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	if (flag == GLD_MAC_PROMISC_NONE) {
		dp->rxmode &= ~(RXMODE_PROMISC | RXMODE_ALLMULTI_REQ);
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
ni_get_stats(gld_mac_info_t *macinfo, struct gld_stats *gs)
{
	struct dp8390_dev	*dp;
	struct dp8390_stats	*ds;

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);

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
	gs->glds_speed     = (dp->speed100 ? 100 : 10) * 1000000;
	gs->glds_media     = GLDM_TP;
	gs->glds_duplex    = GLD_DUPLEX_HALF;

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
static int	ni_timeout_test = 0;
#endif
#endif /* DEBUG */

static int
ni_send(gld_mac_info_t *macinfo, mblk_t *mp)
{
	struct dp8390_dev *dp;
	int		err;
	size_t		len;
#ifdef DEBUG
	static int	sent_cnt = 0;
#endif

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	DPRINTF(3, (CE_CONT, "ni_send: called:"));

	len = msgdsize(mp);
	if (len > ETHERMAX) {
		cmn_err(CE_WARN, "%s: ni_send: msg too big: len:%d",
			dp->name, len);
		dp->stat.errxmt++;

		/* Do not free mp here, GLD will free it */

		return GLD_NOTSUPPORTED;
	}

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
	if (ni_timeout_test > 0) {
		if ((sent_cnt % ni_timeout_test) == 0) {

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

/* ================================================================ */
/*
 * Interrupt service routine
 */
/* ================================================================ */
static u_int
ni_intr(gld_mac_info_t *macinfo)
{
	struct dp8390_dev *dp;
	int		isr = 0;
	int		restart;

	DPRINTF(2, (CE_CONT, "ni_intr: called"));

	dp = (struct dp8390_dev *)macinfo->gldm_private;

	mutex_enter(&dp->intrlock);
	isr = dp8390_interrupt(dp);
	mutex_exit(&dp->intrlock);

	if ((isr & INTR_RESTART_TX) != 0) {
		gld_sched(macinfo);
	}

	DPRINTF(2, (CE_CONT, "ni_intr: device running: %s",
			isr ? "DDI_INTR_CLAIMED" : "DDI_INTR_UNCLAIMED"));

	return (isr ? DDI_INTR_CLAIMED : DDI_INTR_UNCLAIMED);
}

static void
ni_rx_overrun_continue(struct dp8390_dev *dp)
{
	mutex_enter(&dp->intrlock);
	dp8390_rx_overrun_continue(dp);
	mutex_exit(&dp->intrlock);
	gld_sched((gld_mac_info_t *)ddi_get_driver_private(dp->dip));
}

/* ================================================================ */
/*
 * Packet in/out
 */
/* ================================================================ */
static void
ni_sendup_msg(struct dp8390_dev *dp, mblk_t *mp)
{
	gld_recv((gld_mac_info_t *)ddi_get_driver_private(dp->dip), mp);
}

/* ================================================================ */
/*
 * Card dependand HW manupilation
 */
/* ================================================================ */
static int
ni_reset_chip(struct dp8390_dev *dp)
{
	int	i;

	OUTB(dp, PORT_RESET, INB(dp, PORT_RESET));

	i = 0;
	while ((INB(dp, P0_ISR) & ISR_RST) == 0) {
		if (i++ > 100) {
			cmn_err(CE_NOTE, "%s: timeout: reset", dp->name);
			return -1;
		}
		drv_usecwait(10);
	}

	/* clear interrupt */
	OUTB(dp, P0_ISR, ISR_RST);
	return 0;
}

/*====================================================================*/

static void
ni_watchdog(void *arg)
{
	gld_mac_info_t		*macinfo;
	struct dp8390_dev	*dp;
	clock_t			now;
	uint8_t			isr;
	uint8_t			tsr;
	boolean_t		kick_tx = FALSE;

	macinfo = (gld_mac_info_t *)arg;
	dp = (struct dp8390_dev *)macinfo->gldm_private;

	DPRINTF(3, (CE_CONT, "%s: ni_watchdog: called", dp->name));

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);

	/*
	 * Check TX timeout
	 */
	now = ddi_get_lbolt();
	if (dp->nic_state == NIC_ACTIVE &&
	    dp->tx_head != NULL &&
	    now - dp->tx_start_time > TX_TIMEOUT_TICKS) {

		mutex_enter(&dp->pagelock);
		isr = INB(dp, P0_ISR);
		tsr = INB(dp, P0_TSR_R);
		mutex_exit(&dp->pagelock);

		cmn_err(CE_WARN,
		"%s: tx timeout: resetting device, tsr:%b, isr:%b imr:%b",
			dp->name, tsr, TSR_BITS, isr, ISR_BITS,
			dp->imr_shadow, ISR_BITS);

		dp->stat.errxmt++;

		mutex_exit(&dp->xmitlock);
		dp8390_stop_chip(dp);
		mutex_enter(&dp->xmitlock);

		mutex_enter(&dp->pagelock);
		if (ni_reset_chip(dp) != 0) {
			cmn_err(CE_NOTE, "%s: ni_reset_chip failed", dp->name);
		}

		dp8390_init_chip(dp);
		dp8390_set_mac_addr(dp);
		dp8390_start_chip(dp);

		mutex_exit(&dp->pagelock);

		kick_tx = TRUE;
	}

reschedule:
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);

	if (kick_tx) {
		gld_sched(macinfo);
	}

	dp->tx_timeout_id = timeout(ni_watchdog, arg, WATCHDOG_INTERVAL);

	return;
}
