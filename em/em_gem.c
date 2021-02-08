/*
 *  em_gem.c : Intel 8254x/8257x/8258x gigabit ethernet MAC driver for Solaris
 *
 * Copyright (c) 2006-2013 Masayuki Murayama.  All rights reserved.
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

/* Original copyright of FreeBSD em driver */

/**************************************************************************

Copyright (c) 2001-2009, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#pragma	ident	"@(#)em_gem.c	1.5 13/11/22"

/*
 * TODO:
 * fix e1000_osdep.c
 * prepare two different version of em_init_chip for new and legacy chipset
 */

/*
 * System Header files.
 */
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/debug.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/errno.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/byteorder.h>
#include <sys/ethernet.h>
#include <sys/pci.h>

#include "gem_mii.h"
#include "gem.h"
#include "e1000_api.h"

#include "e1000_osdep.h"

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

char	ident[] = "i8254x driver v" VERSION ;

/* Debugging support */
#ifdef DEBUG_LEVEL
static int em_debug = DEBUG_LEVEL;
#if DEBUG_LEVEL > 4
#define	CONS	"^"
#else
#define	CONS	"!"
#endif
#define	DPRINTF(n, args)	if (em_debug > (n)) cmn_err args
#else
#define	CONS	"!"
#define	DPRINTF(n, args)
#endif

/*
 * Useful macros and typedefs
 */
#define	ONESEC			(drv_usectohz(1*1000000))

#define	ROUNDUP(x, a)		(((x) + (a) - 1) & ~((a) - 1))
#define	roundup2(x, a)		ROUNDUP(x, a)

#ifdef MAP_MEM
#define	FLSHB(dp, reg)		INB(dp, reg)
#define	FLSHW(dp, reg)		INW(dp, reg)
#define	FLSHL(dp, reg)		INL(dp, reg)
#else
#define	FLSHB(dp, reg)
#define	FLSHW(dp, reg)
#define	FLSHL(dp, reg)
#endif /* MAP_MEM */

#define	offsetof(t, m)	((long)&(((t *)0)->m))

#define	CONFIG_LEM

/* Tunables */

/*
 * E1000_TXD: Maximum number of Transmit Descriptors
 * Valid Range: 80-256 for 82542 and 82543-based adapters
 *              80-4096 for others
 * Default Value: 256
 *   This value is the number of transmit descriptors allocated by the driver.
 *   Increasing this value allows the driver to queue more transmits. Each
 *   descriptor is 16 bytes.
 *   Since TDLEN should be multiple of 128bytes, the number of transmit
 *   desscriptors should meet the following condition.
 *      (num_tx_desc * sizeof (struct e1000_tx_desc)) % 128 == 0
 */
#define	EM_MIN_TXD		80
#define	EM_MAX_TXD_82543	256
#define	EM_MAX_TXD		4096
#define	EM_DEFAULT_TXD		EM_MAX_TXD_82543

/*
 * E1000_RXD - Maximum number of receive Descriptors
 * Valid Range: 80-256 for 82542 and 82543-based adapters
 *              80-4096 for others
 * Default Value: 256
 *   This value is the number of receive descriptors allocated by the driver.
 *   Increasing this value allows the driver to buffer more incoming packets.
 *   Each descriptor is 16 bytes.  A receive buffer is also allocated for each
 *   descriptor. The maximum MTU size is 16110.
 *   Since TDLEN should be multiple of 128bytes, the number of transmit
 *   desscriptors should meet the following condition.
 *      (num_tx_desc * sizeof (struct e1000_tx_desc)) % 128 == 0
 */
#define	EM_MIN_RXD		80
#define	EM_MAX_RXD_82543	256
#define	EM_MAX_RXD		4096
#define	EM_DEFAULT_RXD		EM_MAX_RXD_82543

/*
 * EM_TIDV - Transmit Interrupt Delay Value
 * Valid Range: 0-65535 (0=off)
 * Default Value: 64
 *   This value delays the generation of transmit interrupts in units of
 *   1.024 microseconds. Transmit interrupt reduction can improve CPU
 *   efficiency if properly tuned for specific network traffic. If the
 *   system is reporting dropped transmits, this value may be set too high
 *   causing the driver to run out of available transmit descriptors.
 */
#define	EM_TIDV				64

/*
 * EM_TADV - Transmit Absolute Interrupt Delay Value
 * (Not valid for 82542/82543/82544)
 * Valid Range: 0-65535 (0=off)
 * Default Value: 64
 *   This value, in units of 1.024 microseconds, limits the delay in which a
 *   transmit interrupt is generated. Useful only if E1000_TIDV is non-zero,
 *   this value ensures that an interrupt is generated after the initial
 *   packet is sent on the wire within the set amount of time.  Proper tuning,
 *   along with E1000_TIDV, may improve traffic throughput in specific
 *   network conditions.
 */
#define	EM_TADV				64

/*
 * EM_RDTR - Receive Interrupt Delay Timer (Packet Timer)
 * Valid Range: 0-65535 (0=off)
 * Default Value: 0
 *   This value delays the generation of receive interrupts in units of 1.024
 *   microseconds.  Receive interrupt reduction can improve CPU efficiency if
 *   properly tuned for specific network traffic. Increasing this value adds
 *   extra latency to frame reception and can end up decreasing the throughput
 *   of TCP traffic. If the system is reporting dropped receives, this value
 *   may be set too high, causing the driver to run out of available receive
 *   descriptors.
 *
 *   CAUTION: When setting E1000_RDTR to a value other than 0, adapters
 *            may hang (stop transmitting) under certain network conditions.
 *            If this occurs a WATCHDOG message is logged in the system
 *            event log. In addition, the controller is automatically reset,
 *            restoring the network connection. To eliminate the potential
 *            for the hang ensure that E1000_RDTR is set to 0.
 */
#define	EM_RDTR				0

/*
 * Receive Interrupt Absolute Delay Timer (Not valid for 82542/82543/82544)
 * Valid Range: 0-65535 (0=off)
 * Default Value: 64
 *   This value, in units of 1.024 microseconds, limits the delay in which a
 *   receive interrupt is generated. Useful only if E1000_RDTR is non-zero,
 *   this value ensures that an interrupt is generated after the initial
 *   packet is received within the set amount of time.  Proper tuning,
 *   along with E1000_RDTR, may improve traffic throughput in specific network
 *   conditions.
 */
#define	EM_RADV				64

/*
 * Inform the stack about transmit checksum offload capabilities.
 */
#define	E1000_CHECKSUM_FEATURES		(CSUM_TCP | CSUM_UDP)

#ifdef E1000_TSO
/*
 * Inform the stack about transmit segmentation offload capabilities.
 */
#define	E1000_TCPSEG_FEATURES		CSUM_TSO
#endif

/*
 * This parameter controls the duration of transmit watchdog timer.
 */
#define	E1000_TX_TIMEOUT		5	/* set to 5 seconds */

/*
 * This parameter controls when the driver calls the routine to reclaim
 * transmit descriptors.
 */
#define	E1000_TX_CLEANUP_THRESHOLD	(adapter->num_tx_desc / 8)
#define	E1000_TX_OP_THRESHOLD		(adapter->num_tx_desc / 32)

/*
 * This parameter controls whether or not autonegotation is enabled.
 *              0 - Disable autonegotiation
 *              1 - Enable  autonegotiation
 */
#define	DO_AUTO_NEG			1

/*
 * This parameter control whether or not the driver will wait for
 * autonegotiation to complete.
 *              1 - Wait for autonegotiation to complete
 *              0 - Don't wait for autonegotiation to complete
 */
#define	WAIT_FOR_AUTO_NEG_DEFAULT	0

/* Tunables -- End */

#define	AUTONEG_ADV_DEFAULT	\
	(ADVERTISE_10_HALF | ADVERTISE_10_FULL | \
	ADVERTISE_100_HALF | ADVERTISE_100_FULL | \
	ADVERTISE_1000_FULL)

#define	AUTO_ALL_MODES		0

/* PHY master/slave setting */
#define	E1000_MASTER_SLAVE		e1000_ms_hw_default

#define	E1000_VENDOR_ID			0x8086
#define	E1000_FLASH			0x0014 /* Flash memory on ICH8 */

#define	E1000_JUMBO_PBA			0x00000028
#define	E1000_DEFAULT_PBA		0x00000030
#define	E1000_SMARTSPEED_DOWNSHIFT	3
#define	E1000_SMARTSPEED_MAX		15
#define	E1000_MAX_INTR			10

#define	MAX_NUM_MULTICAST_ADDRESSES	128
#define	PCI_ANY_ID			(~0U)
#define	ETHER_ALIGN			2
#define	E1000_TX_BUFFER_SIZE		((uint32_t)1514)
#define	E1000_FC_PAUSE_TIME		0x0680
#define	EM_EEPROM_APME			0x400
#define	EM_82544_APME			0x0004

/*
 * TDBA/RDBA should be aligned on 16 byte boundary. But TDLEN/RDLEN should be
 * multiple of 128 bytes. So we align TDBA/RDBA on 128 byte boundary. This will
 * also optimize cache line size effect. H/W supports up to cache line size 128.
 */
#define	E1000_DBA_ALIGN			128

#define	SPEED_MODE_BIT (1<<21)		/* On PCI-E MACs only */

/* PCI Config defines */
#define	E1000_BAR_TYPE(v)		((v) & E1000_BAR_TYPE_MASK)
#define	E1000_BAR_TYPE_MASK		0x00000001
#define	E1000_BAR_TYPE_MMEM		0x00000000
#define	E1000_BAR_TYPE_IO		0x00000001
#define	E1000_BAR_MEM_TYPE(v)		((v) & E1000_BAR_MEM_TYPE_MASK)
#define	E1000_BAR_MEM_TYPE_MASK		0x00000006
#define	E1000_BAR_MEM_TYPE_32BIT	0x00000000
#define	E1000_BAR_MEM_TYPE_64BIT	0x00000004

/* additional macros from if_e1000.h */
#define	E1000_TX_HEAD_ADDR_SHIFT	7
#define	E1000_PBA_BYTES_SHIFT	0xA

#define	PCIR_EXPRESS_DEVICE_CTL		0x8
#define	PCIR_EXPRESS_LINK_CAP		0xc
#define	PCIR_EXPRESS_LINK_CTL		0x10
#define	PCIM_EXP_CTL_MAX_READ_REQUEST	0x7000
#define	PCIM_EXP_CTL_MAX_PAYLOAD	0x00e0
#define	PCIM_LINK_CAP_ASPM		0x00000c00

/* ========================================================== */
/*
 *  Tunable default values.
 */
/* ========================================================== */

#define	E1000_TICKS_TO_USECS(ticks)	((1024 * (ticks) + 500) / 1000)
#define	E1000_USECS_TO_TICKS(usecs)	((1000 * (usecs) + 512) / 1024)
#define	M_TSO_LEN	66		/* mbuf with just hdr and TSO pkthdr */

#define EM_MAX_SCATTER		32
#define EM_VFTA_SIZE		128
#define EM_TSO_SIZE		(65535 + sizeof(struct ether_vlan_header))
#define EM_TSO_SEG_SIZE		4096	/* Max dma segment size */
#define EM_MSIX_MASK		0x01F00000 /* For 82574 use */
#define EM_MSIX_LINK		0x01000000 /* For 82574 use */
#define ETH_ZLEN		60
#define ETH_ADDR_LEN		6
#define CSUM_OFFLOAD		7	/* Offload bits in mbuf flag */

/*
 * 82574 has a nonstandard address for EIAC
 * and since its only used in MSIX, and in
 * the em driver only 82574 uses MSIX we can
 * solve it just using this define.
 */
#define	EM_EIAC	0x000DC

static int e1000_tx_int_delay_dflt = E1000_TICKS_TO_USECS(EM_TIDV);
static int e1000_rx_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RDTR);
static int e1000_tx_abs_int_delay_dflt = E1000_TICKS_TO_USECS(EM_TADV);
static int e1000_rx_abs_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RADV);
static int em_rxd = EM_DEFAULT_RXD;
static int em_txd = EM_DEFAULT_TXD;
static int e1000_smart_pwr_down = FALSE;

/* How many packets rxeof tries to clean at a time */
static int e1000_rx_process_limit = 100;

/*
 * Our configuration
 */
#ifdef GEM_CONFIG_TX_DIRECT
#define	MAXTXFRAGS	(min(GEM_MAXTXFRAGS, 8))	/* LSO is not supported */
#else
#define	MAXTXFRAGS	1
#endif
#define	MAXRXFRAGS	1

#ifdef CONFIG_USE_TXDWB
#define	OUR_INTR_BITS	\
	(E1000_IMS_RXT0 | E1000_IMS_TXDW | \
	| E1000_IMS_RXSEQ | E1000_IMS_RXO | E1000_IMS_LSC)
#else
#define	OUR_INTR_BITS	\
	(E1000_IMS_RXT0 | E1000_ICR_TXQE | \
	E1000_IMS_RXDMT0 | E1000_IMS_RXSEQ | E1000_IMS_RXO | E1000_IMS_LSC)
#endif

#define	EM_FIFO_HDR	0x10

#define	EM_CORE_LOCK(adapter)
#define	EM_CORE_UNLOCK(adapter)

static int	em_tx_copy_thresh = 256;
static int	em_rx_copy_thresh = 256;

#define	E1000_TICKS_TO_USECS(ticks)	((1024 * (ticks) + 500) / 1000)
#define	E1000_USECS_TO_TICKS(usecs)	((1000 * (usecs) + 512) / 1024)
#if 1
static int em_tx_int_delay_dflt = E1000_TICKS_TO_USECS(EM_TIDV);
static int em_rx_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RDTR);
static int em_tx_abs_int_delay_dflt = E1000_TICKS_TO_USECS(EM_TADV);
static int em_rx_abs_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RADV);
#else
static int em_tx_int_delay_dflt = 0;
static int em_rx_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RDTR);
static int em_tx_abs_int_delay_dflt = 0;
static int em_rx_abs_int_delay_dflt = E1000_TICKS_TO_USECS(EM_RADV);
#endif

static int em_smart_pwr_down = B_FALSE;
static int global_quad_port_a = 0;

/*
 * Chip dependant MAC state
 */
struct em_dev {
	/* private data for e1000_hw module */
	struct e1000_hw		hw;

	/* memory maps for registers */
	ddi_acc_handle_t	reg_ha;
	void			*io_base;
	ddi_acc_handle_t	io_ha;
	ddi_acc_handle_t	conf_ha;
	ddi_acc_handle_t	flash_ha;

	uint32_t		tx_fifo_head;
	uint32_t		tx_head_addr;
	uint32_t		tx_fifo_size;
	uint32_t		tx_fifo_thresh;
	boolean_t		tx_fifo_full;
	int			tx_fifo_reset_cnt;
	timeout_id_t		txfifo_watcher_id;

	boolean_t		nic_active;
	int			tx_head;
	int			tx_tail;
	int			tx_tail_real;
	boolean_t		pcix_82544;

	int			tx_int_delay;
	int			rx_int_delay;
	int			tx_abs_int_delay;
	int			rx_abs_int_delay;

	/* register shadows */
	uint32_t		our_intr_mask;
	uint32_t		rctl;

	uint32_t		txd_cmd;
#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	struct e1000_context_desc	tx_ctx;
#endif
	uint32_t		pba;

	size_t			dev_spec_size;
	int			smartspeed;

	int			min_frame_size;
	int			max_frame_size;

	/* workarea for multicast */
	uint8_t			mta[GEM_MAXMC * ETHERADDRL];

	uint8_t			pcie_offset;
	uint8_t			pme_offset;
#ifdef CONFIG_NEW_EM
	boolean_t		msix;
#endif
#ifdef CONFIG_SEPARETE_PHY
	int32_t			(*setup_link)();
#endif
	boolean_t		has_manage;
	uint32_t		wol;
	boolean_t		has_amt;

	struct e1000_hw_stats	stats;

	uint16_t		fc;
};

/* ======================================================== */
/* local operations */
static void em_txfifo_watcher(void *arg);
static int em_82547_tx_fifo_reset(struct gem_dev *dp);
static void em_txfifo_restart(struct gem_dev *dp);
static int em_enable_phy_wakeup(struct em_dev *adapter);
static void em_mii_config_param(struct gem_dev *dp);

/* mii operations */
static void  em_mii_sync(struct gem_dev *);
static uint16_t  em_mii_read(struct gem_dev *, uint_t);
static void em_mii_write(struct gem_dev *, uint_t, uint16_t);
/* nic operations */
static int em_reset_chip(struct gem_dev *);
static int em_init_chip(struct gem_dev *);
static int em_start_chip(struct gem_dev *);
static int em_stop_chip(struct gem_dev *);
static int em_set_media(struct gem_dev *);
static int em_set_rx_filter(struct gem_dev *);
static int em_get_stats(struct gem_dev *);
static int em_attach_chip(struct gem_dev *);

/* descriptor operations */
static int em_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t intreq);
static void em_tx_start(struct gem_dev *dp, int slot, int frags);
static void em_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags);
static void em_rx_start(struct gem_dev *dp, int slot, int frags);
#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t em_tx_desc_head(struct gem_dev *);
#else
static uint_t em_tx_desc_stat(struct gem_dev *, int, int);
#endif
static uint64_t em_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc);

static void em_tx_desc_init(struct gem_dev *dp, int slot);
static void em_rx_desc_init(struct gem_dev *dp, int slot);

/* interrupt handler */
static uint_t em_interrupt(struct gem_dev *dp);

/* ======================================================== */

/* mapping attributes */
/* Data access requirements. */
static struct ddi_device_acc_attr em_dev_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* On sparc, the buffers should be native endianness */
static struct ddi_device_acc_attr em_buf_attr = {
	DDI_DEVICE_ATTR_V0,
	DDI_NEVERSWAP_ACC,	/* native endianness */
	DDI_STRICTORDER_ACC
};

static ddi_dma_attr_t em_dma_attr_buf = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	0, /* patched later */	/* dma_attr_align */
	0,			/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	0, /* patched later */	/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

static ddi_dma_attr_t em_dma_attr_desc = {
	DMA_ATTR_V0,		/* dma_attr_version */
	0,			/* dma_attr_addr_lo */
	0xffffffffull,		/* dma_attr_addr_hi */
	0xffffffffull,		/* dma_attr_count_max */
	E1000_DBA_ALIGN,	/* dma_attr_align */
	0xffffffff,		/* dma_attr_burstsizes */
	1,			/* dma_attr_minxfer */
	0xffffffffull,		/* dma_attr_maxxfer */
	0xffffffffull,		/* dma_attr_seg */
	1,			/* dma_attr_sgllen */
	1,			/* dma_attr_granular */
	0			/* dma_attr_flags */
};

/* ======================================================== */
/*
 * HW manupilation routines
 */
/* ======================================================== */
uint8_t
e1000_reg_read8(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	return (ddi_get8(lp->reg_ha,
	    (uint8_t *)(((long)hw->hw_addr) + port)));
}

uint16_t
e1000_reg_read16(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	return (ddi_get16(lp->reg_ha,
	    (uint16_t *)(((long)hw->hw_addr) + port)));
}

uint32_t
e1000_reg_read32(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	return (ddi_get32(lp->reg_ha,
	    (uint32_t *)(((long)hw->hw_addr) + port)));
}

void
e1000_reg_write8(struct e1000_hw *hw, unsigned long port, uint8_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put8(lp->reg_ha,
	    (uint8_t *)(((long)hw->hw_addr) + port), value);
}

void
e1000_reg_write16(struct e1000_hw *hw, unsigned long port, uint16_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put16(lp->reg_ha,
	    (uint16_t *)(((long)hw->hw_addr) + port), value);
}

void
e1000_reg_write32(struct e1000_hw *hw, unsigned long port, uint32_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put32(lp->reg_ha,
	    (uint32_t *)(((long)hw->hw_addr) + port), value);
}

uint32_t
e1000_io_read32(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	ddi_put32(lp->io_ha,
	    (uint32_t *)((long)lp->io_base), port);
	return (ddi_get32(lp->io_ha,
	    (uint32_t *)(((long)lp->io_base) + 4)));
}

void
e1000_io_write32(struct e1000_hw *hw, unsigned long port, uint32_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put32(lp->io_ha,
	    (uint32_t *)((long)lp->io_base), port);
	ddi_put32(lp->io_ha,
	    (uint32_t *)(((long)lp->io_base) + 4), value);
}

uint16_t
e1000_flash_read16(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	return (ddi_get16(lp->flash_ha,
	    (uint16_t *)(((intptr_t)hw->flash_address) + port)));
}

uint32_t
e1000_flash_read32(struct e1000_hw *hw, unsigned long port)
{
	struct em_dev	*lp = hw->back;

	return (ddi_get32(lp->flash_ha,
	    (uint32_t *)(((intptr_t)hw->flash_address) + port)));
}

void
e1000_flash_write16(struct e1000_hw *hw, unsigned long port, uint16_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put16(lp->flash_ha,
	    (uint16_t *)(((intptr_t)hw->flash_address) + port), value);
}

void
e1000_flash_write32(struct e1000_hw *hw, unsigned long port, uint32_t value)
{
	struct em_dev	*lp = hw->back;

	ddi_put32(lp->flash_ha,
	    (uint32_t *)(((intptr_t)hw->flash_address) + port), value);
}

static void
em_enable_vlans(struct gem_dev *dp)
{
	struct e1000_hw	*hw = &((struct em_dev *)dp->private)->hw;
	uint32_t	ctrl;

	E1000_WRITE_REG(hw, E1000_VET, 0x8100);

	ctrl = E1000_READ_REG(hw, E1000_CTRL);
	ctrl |= E1000_CTRL_VME;
	E1000_WRITE_REG(hw, E1000_CTRL, ctrl);
}

static void
em_disable_vlans(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	uint32_t	ctrl;

	ctrl = E1000_READ_REG(&lp->hw, E1000_CTRL);
	ctrl &= ~E1000_CTRL_VME;
	E1000_WRITE_REG(&lp->hw, E1000_CTRL, ctrl);
}
static void
em_enable_intr(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;

	if (lp->msix) {
		E1000_WRITE_REG(&lp->hw, EM_EIAC, EM_MSIX_MASK);
		lp->our_intr_mask |= EM_MSIX_MASK;
	}
	E1000_WRITE_REG(&lp->hw, E1000_IMS, lp->our_intr_mask);
}

static void
em_disable_intr(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;

	if (lp->msix) {
		E1000_WRITE_REG(&lp->hw, EM_EIAC, 0);
	}
	E1000_WRITE_REG(&lp->hw, E1000_IMC, 0xffffffffU);
}

void
e1000_write_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *valp)
{
	struct em_dev	*lp = hw->back;

	pci_config_put16(lp->conf_ha, reg, *valp);
}

void
e1000_read_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *valp)
{
	struct em_dev	*lp = hw->back;

	*valp = pci_config_get16(lp->conf_ha, reg);
}

void
e1000_pci_set_mwi(struct e1000_hw *hw)
{
	struct em_dev	*lp = hw->back;

	pci_config_put16(lp->conf_ha, PCI_CONF_COMM,
	    lp->hw.bus.pci_cmd_word | PCI_COMM_MEMWR_INVAL);
}

void
e1000_pci_clear_mwi(struct e1000_hw *hw)
{
	struct em_dev	*lp = hw->back;

	pci_config_put16(lp->conf_ha, PCI_CONF_COMM,
	    lp->hw.bus.pci_cmd_word & ~PCI_COMM_MEMWR_INVAL);
}
/*
 * Read the PCI Express capabilities
 */
int32_t
e1000_read_pcie_cap_reg(struct e1000_hw *hw, uint32_t reg, uint16_t *value)
{
	int32_t		error = E1000_SUCCESS;
	struct em_dev	*lp = hw->back;

	if (lp->pcie_offset != 0) {
		e1000_read_pci_cfg(&lp->hw, lp->pcie_offset + reg, value);
	} else {
		ASSERT(E1000_NOT_IMPLEMENTED != 0);
		error = E1000_NOT_IMPLEMENTED;
	}

	return (error);
}
#if 0
int32_t
e1000_alloc_zeroed_dev_spec_struct(struct e1000_hw *hw, uint32_t size)
{
	int32_t		error = 0;
	struct em_dev	*lp = hw->back;

	hw->dev_spec = kmem_alloc(size, KM_NOSLEEP);
	if (hw->dev_spec == NULL) {
		error = ENOMEM;
	}
	bzero(hw->dev_spec, size);

	lp->dev_spec_size = size;

	return (error);
}

void
e1000_free_dev_spec_struct(struct e1000_hw *hw)
{
	struct em_dev	*lp = hw->back;

	ASSERT(lp->dev_spec_size != 0);
	if (hw->dev_spec != NULL) {
		kmem_free(hw->dev_spec, lp->dev_spec_size);
		hw->dev_spec = NULL;
		lp->dev_spec_size = 0;
	}
}
#endif

#define	MAX_INTS_PER_SEC	8000
#define	DEFAULT_ITR		1000000000/(MAX_INTS_PER_SEC * 256)

static void
em_initialize_receive_unit(struct gem_dev *dp)
{
	uint64_t	bus_addr;
	uint32_t	reg_rctl;
	uint32_t	reg_rxcsum;
	struct em_dev	*adapter = dp->private;
	struct e1000_hw	*hw = &adapter->hw;

	DPRINTF(0, (CE_CONT, CONS "%s %s: begin", dp->name, __func__));

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	reg_rctl = E1000_READ_REG(hw, E1000_RCTL);
#ifdef CONFIG_NEW_EM
	/* Do not disable if ever enabled on this hardware */
	if ((hw->mac.type != e1000_82574) && (hw->mac.type != e1000_82583)) {
		E1000_WRITE_REG(hw, E1000_RCTL, reg_rctl & ~E1000_RCTL_EN);
	}
#else
	E1000_WRITE_REG(hw, E1000_RCTL, reg_rctl & ~E1000_RCTL_EN);
#endif
	if (adapter->hw.mac.type >= e1000_82540) {
		E1000_WRITE_REG(&adapter->hw, E1000_RADV,
		    adapter->rx_abs_int_delay);
		/*
		 * Set the interrupt throttling rate. Value is calculated
		 * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
		 */
		E1000_WRITE_REG(&adapter->hw, E1000_ITR, DEFAULT_ITR);
	}

#ifdef CONFIG_NEW_EM
	/*
	** When using MSIX interrupts we need to throttle
	** using the EITR register (82574 only)
	*/
	if (hw->mac.type == e1000_82574) {
		int	i;
		for (i = 0; i < 4; i++) {
			E1000_WRITE_REG(hw, E1000_EITR_82574(i),
			    DEFAULT_ITR);
		}
		/* Disable accelerated acknowledge */
		E1000_WRITE_REG(hw, E1000_RFCTL, E1000_RFCTL_ACK_DIS);
	}
#ifdef notyet
	if (ifp->if_capenable & IFCAP_RXCSUM) {
		rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
		rxcsum |= (E1000_RXCSUM_IPOFL | E1000_RXCSUM_TUOFL);
		E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);
	}
#endif
	/*
	 * TEMPORARY WORKAROUND: on some systems with 82573
	 * long latencies are observed, like Lenovo X60. This
	 * change eliminates the problem, but since having positive
	 * values in RDTR is a known source of problems on other
	 * platforms another solution is being sought.
	 */
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);
#endif

	/* Setup the Base and Length of the Rx Descriptor Ring */
	bus_addr = dp->rx_ring_dma;
	E1000_WRITE_REG(&adapter->hw, E1000_RDLEN(0),
	     dp->gc.gc_rx_ring_size * sizeof(struct e1000_rx_desc));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAH(0),
	    (u32)(bus_addr >> 32));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAL(0),
	    (u32)bus_addr);

#ifdef CONFIG_NEW_EM
#ifdef notdef
	E1000_WRITE_REG(hw, E1000_RDT(0), dp->gc.gc_rx_ring_size - 1);
#endif

	/* Set early receive threshold on appropriate hw */
	if (((hw->mac.type == e1000_ich9lan) ||
	    (hw->mac.type == e1000_pch2lan) ||
	    (hw->mac.type == e1000_ich10lan)) &&
	    (dp->mtu > ETHERMTU)) {
		u32 rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		E1000_WRITE_REG(hw, E1000_RXDCTL(0), rxdctl | 3);
		E1000_WRITE_REG(hw, E1000_ERT, 0x100 | (1 << 13));
	}

	if (adapter->hw.mac.type == e1000_pch2lan) {
		if (dp->mtu > ETHERMTU) {
			e1000_lv_jumbo_workaround_ich8lan(hw, TRUE);
		} else {
			e1000_lv_jumbo_workaround_ich8lan(hw, FALSE);
		}
	}
#endif
	/* Setup the Receive Control Register */
	reg_rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	reg_rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
	    E1000_RCTL_RDMTS_HALF |
	    (adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

#ifdef CONFIG_NO_RXCRC
	/* Strip the CRC */
	reg_rctl |= E1000_RCTL_SECRC;
#endif
	/* Make sure VLAN Filters are off */
	reg_rctl &= ~E1000_RCTL_VFE;

	if (e1000_tbi_sbp_enabled_82543(&adapter->hw)) {
		reg_rctl |= E1000_RCTL_SBP;
	} else {
		reg_rctl &= ~E1000_RCTL_SBP;
	}

	switch (dp->rx_buf_len) {
	case 2048:
		reg_rctl |= E1000_RCTL_SZ_2048;
		break;

	case 4096:
		reg_rctl |= E1000_RCTL_SZ_4096 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;

	case 8192:
	case 9234:
		reg_rctl |= E1000_RCTL_SZ_8192 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;

	case 16384:
		reg_rctl |= E1000_RCTL_SZ_16384 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;

	default:
		cmn_err(CE_WARN, "!%s: incorrect rx buffer size:%d",
		    dp->name, dp->rx_buf_len);
		reg_rctl |= E1000_RCTL_SZ_2048;
		break;
	}

	if (dp->mtu > ETHERMTU) {
		reg_rctl |= E1000_RCTL_LPE;
	} else {
		reg_rctl &= ~E1000_RCTL_LPE;
	}

	/* Enable 82543 Receive Checksum Offload for TCP and UDP */
	if ((hw->mac.type >= e1000_82543) &&
	    (dp->misc_flag & GEM_VLAN_HARD)) {
		reg_rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
		reg_rxcsum |= (E1000_RXCSUM_IPOFL | E1000_RXCSUM_TUOFL);
		E1000_WRITE_REG(hw, E1000_RXCSUM, reg_rxcsum);
	}

	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, reg_rctl);

	DPRINTF(1, (CE_CONT, "!%s: %s: rx_buf_len:%d, rctl:0x%x",
	    dp->name, __func__, dp->rx_buf_len, reg_rctl));

	/*
	 * Setup the HW Rx Head and
	 * Tail Descriptor Pointers
	 */
	E1000_WRITE_REG(hw, E1000_RDH(0), 0);
	E1000_WRITE_REG(hw, E1000_RDT(0), 0);	/* empty */

	adapter->rctl = reg_rctl;
}

static void
em_initialize_transmit_unit(struct gem_dev *dp)
{
	uint32_t	tctl, tarc, tipg = 0;
	struct em_dev	*adapter = dp->private;
	struct e1000_hw	*hw = &adapter->hw;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: begin", dp->name, __func__));
	/* Setup the Base and Length of the Tx Descriptor Ring */
	E1000_WRITE_REG(hw, E1000_TDLEN(0),
	    dp->gc.gc_tx_ring_size * sizeof (struct e1000_tx_desc));
	E1000_WRITE_REG(hw, E1000_TDBAH(0),
	    (uint32_t)(dp->tx_ring_dma >> 32));
	E1000_WRITE_REG(hw, E1000_TDBAL(0), (uint32_t)dp->tx_ring_dma);

	/* Setup the HW Tx Head and Tail descriptor pointers */
	E1000_WRITE_REG(hw, E1000_TDT(0), 0);
	E1000_WRITE_REG(hw, E1000_TDH(0), 0);

	DPRINTF(2, (CE_CONT, CONS "%s %s: Base = 0x%x, Length = 0x%x",
	    dp->name, __func__,
	    E1000_READ_REG(hw, E1000_TDBAL(0)),
	    E1000_READ_REG(hw, E1000_TDLEN(0))));

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (hw->mac.type) {
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		tipg |= DEFAULT_82542_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82542_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
		break;

	case e1000_80003es2lan:
		tipg = DEFAULT_82543_TIPG_IPGR1;
		tipg |= DEFAULT_80003ES2LAN_TIPG_IPGR2 <<
		    E1000_TIPG_IPGR2_SHIFT;
		break;

	default:
		if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
		    (adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(&adapter->hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(&adapter->hw, E1000_TIDV,
	    adapter->tx_int_delay > 0 ? adapter->tx_int_delay : 1);

	if (hw->mac.type >= e1000_82540) {
		E1000_WRITE_REG(&adapter->hw, E1000_TADV,
		    adapter->tx_abs_int_delay);
	}
	if (adapter->hw.mac.type == e1000_82571 ||
	    adapter->hw.mac.type == e1000_82572) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= SPEED_MODE_BIT;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
	} else if (hw->mac.type == e1000_80003es2lan) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(1));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(1), tarc);
	}

	/* Setup Transmit Descriptor Base Settings */
	adapter->txd_cmd = E1000_TXD_CMD_IFCS;
	if (adapter->tx_int_delay > 0)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl = E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
	    (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);

	if (adapter->hw.mac.type >= e1000_82571)
		tctl |= E1000_TCTL_MULR;

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

}

/*
 * Bit of a misnomer, what this really means is
 * to enable OS management of the system... aka
 * to disable special hardware management features 
 */
static void
em_init_manageability(struct gem_dev *dp)
{
	struct em_dev *adapter = dp->private;

	/* A shared code workaround */
#define E1000_82542_MANC2H E1000_MANC2H
	if (adapter->has_manage) {
		int manc2h = E1000_READ_REG(&adapter->hw, E1000_MANC2H);
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);

                /* enable receiving management packets to the host */
		manc |= E1000_MANC_EN_MNG2HOST;
#define E1000_MNG2HOST_PORT_623 (1 << 5)
#define E1000_MNG2HOST_PORT_664 (1 << 6)
		manc2h |= E1000_MNG2HOST_PORT_623;
		manc2h |= E1000_MNG2HOST_PORT_664;
		E1000_WRITE_REG(&adapter->hw, E1000_MANC2H, manc2h);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * Give control back to hardware management
 * controller if there is one.
 */
static void
em_release_manageability(struct gem_dev *dp)
{
	struct em_dev *adapter = dp->private;

	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* re-enable hardware interception of ARP */
		manc |= E1000_MANC_ARP_EN;
		manc &= ~E1000_MANC_EN_MNG2HOST;

		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * em_get_hw_control sets the {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means
 * that the driver is loaded. For AMT version type f/w
 * this means that the network i/f is open.
 */
static void
em_get_hw_control(struct gem_dev *dp)
{
	struct em_dev *adapter = dp->private;
	u32 ctrl_ext, swsm;

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
		    swsm | E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*
 * em_release_hw_control resets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is no longer loaded. For AMT versions of the
 * f/w this means that the network i/f is closed.
 */
static void
em_release_hw_control(struct gem_dev *dp)
{
	struct em_dev *adapter = dp->private;
	u32 ctrl_ext, swsm;

	if (!adapter->has_manage) {
		return;
	}

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
		    swsm & ~E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
}

static int
em_is_valid_ether_addr(u8 *addr)
{
	char zero_addr[6] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || (!bcmp(addr, zero_addr, ETHERADDRL))) {
		return (FALSE);
	}

	return (TRUE);
}

/*
 * Parse the interface capabilities with regard
 * to both system management and wake-on-lan for
 * later use.
 */
static void
em_get_wakeup(struct gem_dev *dp)
{
	struct em_dev	*adapter = dp->private;
	u16		eeprom_data = 0, device_id, apme_mask;

	adapter->has_manage = e1000_enable_mng_pass_thru(&adapter->hw);
	apme_mask = EM_EEPROM_APME;

	switch (adapter->hw.mac.type) {
	case e1000_82573:
	case e1000_82583:
		adapter->has_amt = TRUE;
		/* FALLTHRU */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		if (adapter->hw.bus.func == 1) {
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		} else {
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		}
		break;

	case e1000_ich8lan:
	case e1000_ich9lan:
	case e1000_ich10lan:
	case e1000_pchlan:
	case e1000_pch2lan:
		apme_mask = E1000_WUC_APME;
		adapter->has_amt = TRUE;
		eeprom_data = E1000_READ_REG(&adapter->hw, E1000_WUC);
		break;

	default:
		e1000_read_nvm(&adapter->hw,
		    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & apme_mask) {
		adapter->wol = (E1000_WUFC_MAG | E1000_WUFC_MC);
	}
	/*
         * We have the eeprom settings, now apply the special cases
         * where the eeprom may be wrong or the board won't support
         * wake on lan on a particular port
	 */
	device_id = adapter->hw.device_id;
        switch (device_id) {
	case E1000_DEV_ID_82571EB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting */
		if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		    E1000_STATUS_FUNC_1) {
			adapter->wol = 0;
		}
		break;

	case E1000_DEV_ID_82571EB_QUAD_COPPER:
	case E1000_DEV_ID_82571EB_QUAD_FIBER:
	case E1000_DEV_ID_82571EB_QUAD_COPPER_LP:
                /* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0) {
			adapter->wol = 0;
		}
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4) {
			global_quad_port_a = 0;
		}
                break;
	}
	return;
}

/*
 * Enable PCI Wake On Lan capability
 */
static void
em_enable_wakeup(struct gem_dev *dp)
{
	struct em_dev	*adapter = dp->private;
	u32		pmc, ctrl, ctrl_ext, rctl;
	u16     	status;
	ddi_acc_handle_t	conf_handle;

	if ((pmc = adapter->pme_offset) == 0) {
		return;
	}

	/* Advertise the wakeup capability */
	ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	ctrl |= (E1000_CTRL_SWDPIN2 | E1000_CTRL_SWDPIN3);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);

	if ((adapter->hw.mac.type == e1000_ich8lan) ||
	    (adapter->hw.mac.type == e1000_pchlan) ||
	    (adapter->hw.mac.type == e1000_ich9lan) ||
	    (adapter->hw.mac.type == e1000_ich10lan)) {
		e1000_suspend_workarounds_ich8lan(&adapter->hw);
	}

	/* Keep the laser running on Fiber adapters */
	if (adapter->hw.phy.media_type == e1000_media_type_fiber ||
	    adapter->hw.phy.media_type == e1000_media_type_internal_serdes) {
		ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		ctrl_ext |= E1000_CTRL_EXT_SDP3_DATA;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT, ctrl_ext);
	}
#ifdef notyet
	/*
	 * Determine type of Wakeup: note that wol
	 * is set with all bits on by default.
	 */
	if ((ifp->if_capenable & IFCAP_WOL_MAGIC) == 0) {
		adapter->wol &= ~E1000_WUFC_MAG;
	}

	if ((ifp->if_capenable & IFCAP_WOL_MCAST) == 0) {
		adapter->wol &= ~E1000_WUFC_MC;
	} else {
		rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	}
#endif
	if ((adapter->hw.mac.type == e1000_pchlan) ||
	    (adapter->hw.mac.type == e1000_pch2lan)) {
		if (em_enable_phy_wakeup(adapter)) {
			return;
		}
	} else {
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_WUFC, adapter->wol);
	}

	if (adapter->hw.phy.type == e1000_phy_igp_3) {
		e1000_igp3_phy_powerdown_workaround_ich8lan(&adapter->hw);
	}

        /* Request PME */
        status = pci_config_get16(adapter->conf_ha, pmc + PCI_PMCSR);
	status &= ~(PCI_PMCSR_PME_STAT | PCI_PMCSR_PME_EN);
#if 0
	if (ifp->if_capenable & IFCAP_WOL) {
		status |= PCI_PMCSR_PME_STAT | PCI_PMCSR_PME_EN;
	}
#endif
        pci_config_put16(adapter->conf_ha, pmc + PCI_PMCSR, status);

	return;
}

/*
 * WOL in the newer chipset interfaces (pchlan)
 * require thing to be copied into the phy
 */
static int
em_enable_phy_wakeup(struct em_dev *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mreg, ret = 0;
	u16 preg;
	int i;

	/* copy MAC RARs to PHY RARs */
	e1000_copy_rx_addrs_to_phy_ich8lan(hw);

	/* copy MAC MTA to PHY MTA */
	for (i = 0; i < adapter->hw.mac.mta_reg_count; i++) {
		mreg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
		e1000_write_phy_reg(hw, BM_MTA(i), (u16)(mreg & 0xFFFF));
		e1000_write_phy_reg(hw, BM_MTA(i) + 1,
		    (u16)((mreg >> 16) & 0xFFFF));
	}

	/* configure PHY Rx Control register */
	e1000_read_phy_reg(&adapter->hw, BM_RCTL, &preg);
	mreg = E1000_READ_REG(hw, E1000_RCTL);
	if (mreg & E1000_RCTL_UPE)
		preg |= BM_RCTL_UPE;
	if (mreg & E1000_RCTL_MPE)
		preg |= BM_RCTL_MPE;
	preg &= ~(BM_RCTL_MO_MASK);
	if (mreg & E1000_RCTL_MO_3)
		preg |= (((mreg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT)
				<< BM_RCTL_MO_SHIFT);
	if (mreg & E1000_RCTL_BAM)
		preg |= BM_RCTL_BAM;
	if (mreg & E1000_RCTL_PMCF)
		preg |= BM_RCTL_PMCF;
	mreg = E1000_READ_REG(hw, E1000_CTRL);
	if (mreg & E1000_CTRL_RFCE)
		preg |= BM_RCTL_RFCE;
	e1000_write_phy_reg(&adapter->hw, BM_RCTL, preg);

	/* enable PHY wakeup in MAC register */
	E1000_WRITE_REG(hw, E1000_WUC,
	    E1000_WUC_PHY_WAKE | E1000_WUC_PME_EN);
	E1000_WRITE_REG(hw, E1000_WUFC, adapter->wol);

	/* configure and enable PHY wakeup in PHY registers */
	e1000_write_phy_reg(&adapter->hw, BM_WUFC, adapter->wol);
	e1000_write_phy_reg(&adapter->hw, BM_WUC, E1000_WUC_PME_EN);

	/* activate PHY wakeup */
	ret = hw->phy.ops.acquire(hw);
	if (ret) {
		printf("Could not acquire PHY\n");
		return ret;
	}
	e1000_write_phy_reg_mdic(hw, IGP01E1000_PHY_PAGE_SELECT,
	                         (BM_WUC_ENABLE_PAGE << IGP_PAGE_SHIFT));
	ret = e1000_read_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, &preg);
	if (ret) {
		printf("Could not read PHY page 769\n");
		goto out;
	}
	preg |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
	ret = e1000_write_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, preg);
	if (ret)
		printf("Could not set PHY Host Wakeup bit\n");
out:
	hw->phy.ops.release(hw);

	return ret;
}

static void
em_led_func(void *arg, int onoff)
{
	struct em_dev	*adapter = arg;
 
	EM_CORE_LOCK(adapter);
	if (onoff) {
		e1000_setup_led(&adapter->hw);
		e1000_led_on(&adapter->hw);
	} else {
		e1000_led_off(&adapter->hw);
		e1000_cleanup_led(&adapter->hw);
	}
	EM_CORE_UNLOCK(adapter);
}

/*
** Disable the L0S and L1 LINK states
*/
static void
em_disable_aspm(struct gem_dev *dp)
{
	struct em_dev	*adapter = dp->private;
	int		reg;
	uint16_t	link_cap,link_ctrl;

	switch (adapter->hw.mac.type) {
	case e1000_82573:
	case e1000_82574:
	case e1000_82583:
		break;
	default:
		return;
	}

	if (adapter->pcie_offset == 0) {
		return;
	}

	reg = adapter->pcie_offset + PCIR_EXPRESS_LINK_CAP;
	link_cap = pci_config_get16(adapter->conf_ha, reg);
	if ((link_cap & PCIM_LINK_CAP_ASPM) == 0) {
		return;
	}
	reg = adapter->pcie_offset + PCIR_EXPRESS_LINK_CTL;
	link_ctrl = pci_config_get16(adapter->conf_ha, reg);
	link_ctrl &= 0xFFFC; /* turn off bit 1 and 2 */
	pci_config_put16(adapter->conf_ha, reg, link_ctrl);
	return;
}
/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
em_update_stats_counters(struct gem_dev *dp)
{
	struct em_dev	*adapter = dp->private;

	if(adapter->hw.phy.media_type == e1000_media_type_copper ||
	   (E1000_READ_REG(&adapter->hw, E1000_STATUS) & E1000_STATUS_LU)) {
		adapter->stats.symerrs += E1000_READ_REG(&adapter->hw, E1000_SYMERRS);
		adapter->stats.sec += E1000_READ_REG(&adapter->hw, E1000_SEC);
	}
	adapter->stats.crcerrs += E1000_READ_REG(&adapter->hw, E1000_CRCERRS);
	adapter->stats.mpc += E1000_READ_REG(&adapter->hw, E1000_MPC);
	adapter->stats.scc += E1000_READ_REG(&adapter->hw, E1000_SCC);
	adapter->stats.ecol += E1000_READ_REG(&adapter->hw, E1000_ECOL);

	adapter->stats.mcc += E1000_READ_REG(&adapter->hw, E1000_MCC);
	adapter->stats.latecol += E1000_READ_REG(&adapter->hw, E1000_LATECOL);
	adapter->stats.colc += E1000_READ_REG(&adapter->hw, E1000_COLC);
	adapter->stats.dc += E1000_READ_REG(&adapter->hw, E1000_DC);
	adapter->stats.rlec += E1000_READ_REG(&adapter->hw, E1000_RLEC);
	adapter->stats.xonrxc += E1000_READ_REG(&adapter->hw, E1000_XONRXC);
	adapter->stats.xontxc += E1000_READ_REG(&adapter->hw, E1000_XONTXC);
	/*
	** For watchdog management we need to know if we have been
	** paused during the last interval, so capture that here.
	*/
	adapter->stats.xoffrxc += E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	adapter->stats.xofftxc += E1000_READ_REG(&adapter->hw, E1000_XOFFTXC);
	adapter->stats.fcruc += E1000_READ_REG(&adapter->hw, E1000_FCRUC);
	adapter->stats.prc64 += E1000_READ_REG(&adapter->hw, E1000_PRC64);
	adapter->stats.prc127 += E1000_READ_REG(&adapter->hw, E1000_PRC127);
	adapter->stats.prc255 += E1000_READ_REG(&adapter->hw, E1000_PRC255);
	adapter->stats.prc511 += E1000_READ_REG(&adapter->hw, E1000_PRC511);
	adapter->stats.prc1023 += E1000_READ_REG(&adapter->hw, E1000_PRC1023);
	adapter->stats.prc1522 += E1000_READ_REG(&adapter->hw, E1000_PRC1522);
	adapter->stats.gprc += E1000_READ_REG(&adapter->hw, E1000_GPRC);
	adapter->stats.bprc += E1000_READ_REG(&adapter->hw, E1000_BPRC);
	adapter->stats.mprc += E1000_READ_REG(&adapter->hw, E1000_MPRC);
	adapter->stats.gptc += E1000_READ_REG(&adapter->hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	adapter->stats.gorc += E1000_READ_REG(&adapter->hw, E1000_GORCH);
	adapter->stats.gotc += E1000_READ_REG(&adapter->hw, E1000_GOTCH);

	adapter->stats.rnbc += E1000_READ_REG(&adapter->hw, E1000_RNBC);
	adapter->stats.ruc += E1000_READ_REG(&adapter->hw, E1000_RUC);
	adapter->stats.rfc += E1000_READ_REG(&adapter->hw, E1000_RFC);
	adapter->stats.roc += E1000_READ_REG(&adapter->hw, E1000_ROC);
	adapter->stats.rjc += E1000_READ_REG(&adapter->hw, E1000_RJC);

	adapter->stats.tor += E1000_READ_REG(&adapter->hw, E1000_TORH);
	adapter->stats.tot += E1000_READ_REG(&adapter->hw, E1000_TOTH);

	adapter->stats.tpr += E1000_READ_REG(&adapter->hw, E1000_TPR);
	adapter->stats.tpt += E1000_READ_REG(&adapter->hw, E1000_TPT);
	adapter->stats.ptc64 += E1000_READ_REG(&adapter->hw, E1000_PTC64);
	adapter->stats.ptc127 += E1000_READ_REG(&adapter->hw, E1000_PTC127);
	adapter->stats.ptc255 += E1000_READ_REG(&adapter->hw, E1000_PTC255);
	adapter->stats.ptc511 += E1000_READ_REG(&adapter->hw, E1000_PTC511);
	adapter->stats.ptc1023 += E1000_READ_REG(&adapter->hw, E1000_PTC1023);
	adapter->stats.ptc1522 += E1000_READ_REG(&adapter->hw, E1000_PTC1522);
	adapter->stats.mptc += E1000_READ_REG(&adapter->hw, E1000_MPTC);
	adapter->stats.bptc += E1000_READ_REG(&adapter->hw, E1000_BPTC);

	if (adapter->hw.mac.type >= e1000_82543) {
		adapter->stats.algnerrc += 
		E1000_READ_REG(&adapter->hw, E1000_ALGNERRC);
		adapter->stats.rxerrc += 
		E1000_READ_REG(&adapter->hw, E1000_RXERRC);
		adapter->stats.tncrs += 
		E1000_READ_REG(&adapter->hw, E1000_TNCRS);
		adapter->stats.cexterr += 
		E1000_READ_REG(&adapter->hw, E1000_CEXTERR);
		adapter->stats.tsctc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTC);
		adapter->stats.tsctfc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTFC);
	}
#if 0
	ifp = adapter->ifp;

	ifp->if_collisions = adapter->stats.colc;

	/* Rx Errors */
	ifp->if_ierrors = adapter->dropped_pkts + adapter->stats.rxerrc +
	    adapter->stats.crcerrs + adapter->stats.algnerrc +
	    adapter->stats.ruc + adapter->stats.roc +
	    adapter->stats.mpc + adapter->stats.cexterr;

	/* Tx Errors */
	ifp->if_oerrors = adapter->stats.ecol +
	    adapter->stats.latecol + adapter->watchdog_events;
#endif
}
/* ==================================================================== */
/*
 *  gem methods
 */
/* ==================================================================== */
static int
em_reset_chip(struct gem_dev *dp)
{
	uint32_t	err;
	struct em_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, CONS "%s: %s called", dp->name, __func__));
	ASSERT(lp->hw.back != NULL);
	ASSERT(lp->hw.hw_addr != NULL);

	em_disable_intr(dp);

	lp->nic_active = B_FALSE;
#ifdef notdef
	/* allow resetting phy */
	lp->hw.phy.reset_disable = B_FALSE;
#endif
	if (e1000_reset_hw(&lp->hw) != E1000_SUCCESS) {
		return (GEM_FAILURE);
	}

	/* When hardware is reset, fifo_head is also reset */
	lp->tx_fifo_head = 0;

	return (GEM_SUCCESS);
}

static int
em_init_chip(struct gem_dev *dp)
{
	uint32_t	val;
	uint32_t	pba;
	uint32_t	rx_buffer_size;
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	DPRINTF(0, (CE_CONT, CONS "%s: %s called", dp->name, __func__));

	lp->nic_active = B_FALSE;

	em_mii_config_param(dp);

	/*
	 * from em_init_locked ()
	 */

	/*
	 * from em_reset()
	 */
#ifdef CONFIG_NEW_EM
	/* Set up smart power down as default off on newer adapters. */
	if (!e1000_smart_pwr_down && (hw->mac.type == e1000_82571 ||
	    hw->mac.type == e1000_82572)) {
		uint16_t phy_tmp = 0;

		/* Speed up time to link by disabling smart power down. */
		e1000_read_phy_reg(hw,
		    IGP02E1000_PHY_POWER_MGMT, &phy_tmp);
		phy_tmp &= ~IGP02E1000_PM_SPD;
		e1000_write_phy_reg(hw,
		    IGP02E1000_PHY_POWER_MGMT, phy_tmp);
	}
#endif /* CONFIG_NEW_EM */

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 *
	 * Devices before the 82547 had a Packet Buffer of 64K.
	 *   Default allocation: PBA=48K for Rx, leaving 16K for Tx.
	 * After the 82547 the buffer was reduced to 40K.
	 *   Default allocation: PBA=30K for Rx, leaving 10K for Tx.
	 *   Note: default does not leave enough room for Jumbo Frame >10k.
	 */
	switch (hw->mac.type) {

#ifdef CONFIG_LEM
	case e1000_82547:
	case e1000_82547_rev_2: /* 82547: Total Packet Buffer is 40K */
		if (lp->max_frame_size > 8192) {
			pba = E1000_PBA_22K; /* 22K for Rx, 18K for Tx */
		} else {
			pba = E1000_PBA_30K; /* 30K for Rx, 10K for Tx */
		}
		lp->tx_fifo_head = 0;
		lp->tx_head_addr = pba << E1000_TX_HEAD_ADDR_SHIFT;
		lp->tx_fifo_size =
		    (E1000_PBA_40K - pba) << E1000_PBA_BYTES_SHIFT;
		break;
#endif /* CONFIG_LEM */

#ifdef CONFIG_NEW_EM
	/* Total Packet Buffer on these is 48K */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		pba = E1000_PBA_32K; /* 32K for Rx, 16K for Tx */
		break;
	case e1000_82573: /* 82573: Total Packet Buffer is 32K */
		pba = E1000_PBA_12K; /* 12K for Rx, 20K for Tx */
		break;
	case e1000_82574:
	case e1000_82583:
		pba = E1000_PBA_20K; /* 20K for Rx, 20K for Tx */
		break;
	case e1000_ich8lan:
		pba = E1000_PBA_8K;
		break;
	case e1000_ich9lan:
	case e1000_ich10lan:
		/* Boost Receive side for jumbo frames */
		if (lp->max_frame_size > 4096)
			pba = E1000_PBA_14K;
		else
			pba = E1000_PBA_10K;
		break;
	case e1000_pchlan:
	case e1000_pch2lan:
	case e1000_pch_lpt:
		pba = E1000_PBA_26K;
		break;
#endif
	default:
		/* Devices before 82547 had a Packet Buffer of 64K.   */
		if (lp->max_frame_size > 8192) {
			pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
		} else {
			pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
		}
	}
	DPRINTF(0, (CE_CONT, CONS "%s: %s: pba=%dK", dp->name, __func__, pba));
	E1000_WRITE_REG(&lp->hw, E1000_PBA, pba);

	/*
	 * These parameters control the automatic generation (Tx) and
	 * response (Rx) to Ethernet PAUSE frames.
	 * - High water mark should allow for at least two frames to be
	 *   received after sending an XOFF.
	 * - Low water mark works best when it is very near the high water mark.
	 *   This allows the receiver to restart by sending XON when it has
	 *   drained a bit. Here we use an arbitary value of 1500 which will
	 *   restart after one full frame is pulled from the buffer. There
	 *   could be several smaller frames in the buffer and if so they will
	 *   not trigger the XON until their total number reduces the buffer
	 *   by 1500.
	 * - The pause time is fairly large at 1000 x 512ns = 512 usec.
	 */
	rx_buffer_size = ((E1000_READ_REG(hw, E1000_PBA) & 0xffff) << 10);
	hw->fc.high_water = rx_buffer_size -
	    roundup2(lp->max_frame_size, 1024);
	hw->fc.low_water = hw->fc.high_water - 1500;

	if (lp->fc) /* locally set flow control value? */
		hw->fc.requested_mode = lp->fc;
	else
		hw->fc.requested_mode = e1000_fc_full;

	if (hw->mac.type == e1000_80003es2lan) {
		hw->fc.pause_time = 0xFFFF;
	} else {
		hw->fc.pause_time = E1000_FC_PAUSE_TIME;
	}
	hw->fc.send_xon = TRUE;

#ifdef E1000_DEV_ID_82574LA
	/* XXX - not implemented */
	/* Set Flow control, use the tunable location if sane */
	switch (dp->flow_control) {
	default:
	case FLOW_CONTROL_NONE:
		hw->fc.requested_mode = e1000_fc_none;
		break;

	case FLOW_CONTROL_SYMMETRIC:
		hw->fc.requested_mode = e1000_fc_full;
		break;

	case FLOW_CONTROL_TX_PAUSE:
		hw->fc.requested_mode = e1000_fc_tx_pause;
		break;

	case FLOW_CONTROL_RX_PAUSE:
		hw->fc.requested_mode = e1000_fc_rx_pause;
		break;
	}
#else
	hw->fc.type = e1000_fc_full;
#endif

#ifdef CONFIG_NEW_EM
	/* Device specific overrides/settings */
	switch (hw->mac.type) {
	case e1000_pchlan:
		/* Workaround: no TX flow ctrl for PCH */
		hw->fc.requested_mode = e1000_fc_rx_pause;
		hw->fc.pause_time = 0xFFFF; /* override */
		if (dp->mtu > ETHERMTU) {
			hw->fc.high_water = 0x3500;
			hw->fc.low_water = 0x1500;
		} else {
			hw->fc.high_water = 0x5000;
			hw->fc.low_water = 0x3000;
		}
		hw->fc.refresh_time = 0x1000;
		break;
	case e1000_pch2lan:
	case e1000_pch_lpt:
		hw->fc.high_water = 0x5C20;
		hw->fc.low_water = 0x5048;
		hw->fc.pause_time = 0x0650;
		hw->fc.refresh_time = 0x0400;
		/* Jumbos need adjusted PBA */
		if (dp->mtu > ETHERMTU)
			E1000_WRITE_REG(hw, E1000_PBA, 12);
		else
			E1000_WRITE_REG(hw, E1000_PBA, 26);
		break;
	case e1000_ich9lan:
	case e1000_ich10lan:
		if (dp->mtu > ETHERMTU) {
			hw->fc.high_water = 0x2800;
			hw->fc.low_water = hw->fc.high_water - 8;
			break;
		}
		/* else fall thru */
	default:
		if (hw->mac.type == e1000_80003es2lan)
			hw->fc.pause_time = 0xFFFF;
		break;
	}
#endif /* CONFIG_NEW_EM */

	/* Issue a global reset */
	e1000_reset_hw(hw);
	E1000_WRITE_REG(hw, E1000_WUC, 0);
#ifdef CONFIG_NEW_EM
	em_disable_aspm(dp);
#endif
#ifdef notdef
#ifdef CONFIG_HW_LINK_DETECTION
	/* XXX - what should we do ? */
#else
	dp->mii_state = MII_STATE_UNKNOWN;
#endif
#endif
	lp->hw.mac.get_link_status = 1;

	/* XXX - init_hw also restart autonegotiation */
	if (e1000_init_hw(hw) != E1000_SUCCESS) {
		cmn_err(CE_WARN, "!%s: %s: e1000_init_hw: failed",
		    dp->name, __func__);
	}
#ifdef CONFIG_NEW_EM
	E1000_WRITE_REG(hw, E1000_VET, ETHERTYPE_VLAN);
#ifdef notdef
	/* XXX - don't call get_phy_info, it waits until the link become up */
	e1000_get_phy_info(hw);
#endif
	/* XXX - don't call check_for_link, link will be down now */
	e1000_check_for_link(hw);
#endif
	/* XXX - em_reset: ends */
#if 0
	em_update_link_status(dp);
#endif
	/* Configure for OS presence */
	em_init_manageability(dp);

	DPRINTF(0, (CE_CONT, CONS "%s: %s: rx_ring base:%x, tx ring base:%x",
	    dp->name, __func__,
	    dp->rx_ring, dp->tx_ring));

	em_initialize_transmit_unit(dp);

	/* XXX - we setup multicast table later */

	em_initialize_receive_unit(dp);

#ifdef notyet	/* from legacy em */
	/* Use real VLAN Filter support? */
	if (ifp->if_capenable & IFCAP_VLAN_HWTAGGING) {
		if (ifp->if_capenable & IFCAP_VLAN_HWFILTER) {
			/* Use real VLAN Filter support */
			lem_setup_vlan_hw_support(adapter);
		} else {
			u32 ctrl;
			ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
			ctrl |= E1000_CTRL_VME;
			E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
		}
	}
#else
	em_enable_vlans(dp);
#endif

#ifdef never
	/* Don't lose promiscuous settings */
	lem_set_promisc(adapter);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

#endif
	e1000_clear_hw_cntrs_base_generic(hw);

#ifdef CONFIG_MSI
	/* MSI/X configuration for 82574 */
	if (hw->mac.type == e1000_82574) {
		int tmp;
		tmp = E1000_READ_REG(hw, E1000_CTRL_EXT);
		tmp |= E1000_CTRL_EXT_PBA_CLR;
		E1000_WRITE_REG(hw, E1000_CTRL_EXT, tmp);
		/*
		 * Set the IVAR - interrupt vector routing.
		 * Each nibble represents a vector, high bit
		 * is enable, other 3 bits are the MSIX table
		 * entry, we map RXQ0 to 0, TXQ0 to 1, and
		 * Link (other) to 2, hence the magic number.
		 */
		E1000_WRITE_REG(hw, E1000_IVAR, 0x800A0908);
	}
#endif

	lp->our_intr_mask = 0;

#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	bzero(&lp->tx_ctx, sizeof (lp->tx_ctx));
#endif
	lp->tx_head = 0;
	lp->tx_tail = 0;
	lp->tx_tail_real = 0;

	lp->tx_fifo_head = 0;
	lp->tx_fifo_full = B_FALSE;

	return (GEM_SUCCESS);
}

static int
em_set_rx_filter(struct gem_dev *dp)
{
	int		i;
	uint32_t	reg_rctrl;
	struct em_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	reg_rctrl = lp->rctl;

	/* Set Receive filter control register */
	if ((dp->rxmode & RXMODE_ENABLE) == 0) {
		E1000_WRITE_REG(&lp->hw, E1000_RCTL, 0);
		return (GEM_SUCCESS);
	}

	if (dp->rxmode & RXMODE_PROMISC) {
		reg_rctrl |= E1000_RCTL_UPE | E1000_RCTL_MPE;
	} else if ((dp->rxmode & RXMODE_ALLMULTI) || dp->mc_count > 32) {
		reg_rctrl |= E1000_RCTL_MPE;
		reg_rctrl &= ~E1000_RCTL_UPE;
	} else {
		reg_rctrl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
	}

	/* Load Multicast hash table */
	if ((reg_rctrl & E1000_RCTL_MPE) == 0) {
		uint8_t	*dest;

		if (lp->hw.mac.type == e1000_82542 &&
		    lp->hw.revision_id == E1000_REVISION_2) {
			if (lp->hw.bus.pci_cmd_word &
			    CMD_MEM_WRT_INVALIDATE) {
				e1000_pci_clear_mwi(&lp->hw);
			}
			reg_rctrl |= E1000_RCTL_RST;
			E1000_WRITE_REG(&lp->hw, E1000_RCTL, reg_rctrl);
			drv_usecwait(5000);
		}

		dest = lp->mta;
		for (i = 0; i < dp->mc_count; i++) {
			bcopy(dp->mc_list[i].addr.ether_addr_octet,
			    dest, ETHERADDRL);
			dest += ETHERADDRL;
		}
#ifdef E1000_DEV_ID_82574LA
		e1000_update_mc_addr_list(&lp->hw, lp->mta, dp->mc_count);
#else
		e1000_update_mc_addr_list(&lp->hw, lp->mta,
		    dp->mc_count, 1, lp->hw.mac.rar_entry_count);
#endif

		if (lp->hw.mac.type == e1000_82542 &&
		    lp->hw.revision_id == E1000_REVISION_2) {
			reg_rctrl &= ~E1000_RCTL_RST;
			E1000_WRITE_REG(&lp->hw, E1000_RCTL, reg_rctrl);
			drv_usecwait(5000);

			if (lp->hw.bus.pci_cmd_word & CMD_MEM_WRT_INVALIDATE) {
				e1000_pci_set_mwi(&lp->hw);
			}
		}
	}

	/* Load station address if it has been changed */
	if (bcmp(dp->cur_addr.ether_addr_octet,
	    lp->hw.mac.addr, ETHERADDRL) != 0) {
		bcopy(dp->cur_addr.ether_addr_octet,
		    lp->hw.mac.addr, ETHERADDRL);
		e1000_rar_set(&lp->hw, lp->hw.mac.addr, 0);
	}

	/* Load new rx filter mode */
	E1000_WRITE_REG(&lp->hw, E1000_RCTL, reg_rctrl);

	return (GEM_SUCCESS);
}

static int
em_start_chip(struct gem_dev *dp)
{
	struct em_dev	*adapter = dp->private;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	adapter->our_intr_mask = IMS_ENABLE_MASK;
	adapter->nic_active = B_TRUE;
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		em_enable_intr(dp);
	}

	/* AMT based hardware can now take control from firmware */
	if (adapter->has_manage && adapter->has_amt) {
		em_get_hw_control(dp);
	}


	return (GEM_SUCCESS);
}

static int
em_stop_chip(struct gem_dev *dp)	/* done */
{
	struct em_dev	*adapter = dp->private;
	struct e1000_hw	*hw = &adapter->hw;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	/* disable interrupt */
	if ((dp->misc_flag & GEM_NOINTR) == 0) {
		em_disable_intr(dp);
	}
	adapter->our_intr_mask = 0;

	adapter->nic_active = B_FALSE;

	e1000_reset_hw(hw);
	if (hw->mac.type >= e1000_82544) {
		E1000_WRITE_REG(hw, E1000_WUC, 0);
	}
#ifdef CONFIG_NEW_EM
	e1000_led_off(hw);
	e1000_cleanup_led(hw);
#endif

	DPRINTF(0, (CE_CONT, CONS "%s: %s: exited", dp->name, __func__));

	return (GEM_SUCCESS);
}

static int			/* done */
em_set_media(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;
	extern int gem_speed_value[];

	DPRINTF(0, (CE_CONT, CONS "%s: %s: %s duplex, %d Mbps",
	    dp->name, __func__,
	    dp->full_duplex ? "full" : "half",
	    gem_speed_value[dp->speed]));

	e1000_get_phy_info(hw);

	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		/* ESB2 fix */
		e1000_cfg_on_link_up(hw);
	}

	/* Check if we must disable SPEED_MODE bit on PCI-E */
	if ((dp->speed != GEM_SPD_1000) &&
	    ((hw->mac.type == e1000_82571) ||
	    (hw->mac.type == e1000_82572))) {
		int tarc0;

		tarc0 = E1000_READ_REG(&lp->hw, E1000_TARC(0));
		tarc0 &= ~SPEED_MODE_BIT;
		E1000_WRITE_REG(&lp->hw, E1000_TARC(0), tarc0);
	}
	lp->smartspeed = 0;

	return (GEM_SUCCESS);
}

static int
em_get_stats(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	if (lp->hw.phy.media_type == e1000_media_type_copper ||
	    (E1000_READ_REG(&lp->hw, E1000_STATUS) & E1000_STATUS_LU)) {
		lp->stats.symerrs += E1000_READ_REG(&lp->hw, E1000_SYMERRS);
		lp->stats.sec += E1000_READ_REG(&lp->hw, E1000_SEC);
	}
	lp->stats.crcerrs += E1000_READ_REG(&lp->hw, E1000_CRCERRS);
	lp->stats.mpc += E1000_READ_REG(&lp->hw, E1000_MPC);
	lp->stats.scc += E1000_READ_REG(&lp->hw, E1000_SCC);
	lp->stats.ecol += E1000_READ_REG(&lp->hw, E1000_ECOL);

	lp->stats.mcc += E1000_READ_REG(&lp->hw, E1000_MCC);
	lp->stats.latecol += E1000_READ_REG(&lp->hw, E1000_LATECOL);
	lp->stats.colc += E1000_READ_REG(&lp->hw, E1000_COLC);
	lp->stats.dc += E1000_READ_REG(&lp->hw, E1000_DC);
	lp->stats.rlec += E1000_READ_REG(&lp->hw, E1000_RLEC);
	lp->stats.xonrxc += E1000_READ_REG(&lp->hw, E1000_XONRXC);
	lp->stats.xontxc += E1000_READ_REG(&lp->hw, E1000_XONTXC);
	lp->stats.xoffrxc += E1000_READ_REG(&lp->hw, E1000_XOFFRXC);
	lp->stats.xofftxc += E1000_READ_REG(&lp->hw, E1000_XOFFTXC);
	lp->stats.fcruc += E1000_READ_REG(&lp->hw, E1000_FCRUC);
	lp->stats.prc64 += E1000_READ_REG(&lp->hw, E1000_PRC64);
	lp->stats.prc127 += E1000_READ_REG(&lp->hw, E1000_PRC127);
	lp->stats.prc255 += E1000_READ_REG(&lp->hw, E1000_PRC255);
	lp->stats.prc511 += E1000_READ_REG(&lp->hw, E1000_PRC511);
	lp->stats.prc1023 += E1000_READ_REG(&lp->hw, E1000_PRC1023);
	lp->stats.prc1522 += E1000_READ_REG(&lp->hw, E1000_PRC1522);
	lp->stats.gprc += E1000_READ_REG(&lp->hw, E1000_GPRC);
	lp->stats.bprc += E1000_READ_REG(&lp->hw, E1000_BPRC);
	lp->stats.mprc += E1000_READ_REG(&lp->hw, E1000_MPRC);
	lp->stats.gptc += E1000_READ_REG(&lp->hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	lp->stats.gorc += E1000_READ_REG(&lp->hw, E1000_GORCL);
	lp->stats.hgorc += E1000_READ_REG(&lp->hw, E1000_GORCH);
	lp->stats.gotc += E1000_READ_REG(&lp->hw, E1000_GOTCL);
	lp->stats.hgotc += E1000_READ_REG(&lp->hw, E1000_GOTCH);

	lp->stats.rnbc += E1000_READ_REG(&lp->hw, E1000_RNBC);
	lp->stats.ruc += E1000_READ_REG(&lp->hw, E1000_RUC);
	lp->stats.rfc += E1000_READ_REG(&lp->hw, E1000_RFC);
	lp->stats.roc += E1000_READ_REG(&lp->hw, E1000_ROC);
	lp->stats.rjc += E1000_READ_REG(&lp->hw, E1000_RJC);

	lp->stats.tor += E1000_READ_REG(&lp->hw, E1000_TORH);
	lp->stats.tot += E1000_READ_REG(&lp->hw, E1000_TOTH);

	lp->stats.tpr += E1000_READ_REG(&lp->hw, E1000_TPR);
	lp->stats.tpt += E1000_READ_REG(&lp->hw, E1000_TPT);
	lp->stats.ptc64 += E1000_READ_REG(&lp->hw, E1000_PTC64);
	lp->stats.ptc127 += E1000_READ_REG(&lp->hw, E1000_PTC127);
	lp->stats.ptc255 += E1000_READ_REG(&lp->hw, E1000_PTC255);
	lp->stats.ptc511 += E1000_READ_REG(&lp->hw, E1000_PTC511);
	lp->stats.ptc1023 += E1000_READ_REG(&lp->hw, E1000_PTC1023);
	lp->stats.ptc1522 += E1000_READ_REG(&lp->hw, E1000_PTC1522);
	lp->stats.mptc += E1000_READ_REG(&lp->hw, E1000_MPTC);
	lp->stats.bptc += E1000_READ_REG(&lp->hw, E1000_BPTC);

	/* Interrupt Counts */

	lp->stats.iac += E1000_READ_REG(&lp->hw, E1000_IAC);
	lp->stats.icrxptc += E1000_READ_REG(&lp->hw, E1000_ICRXPTC);
	lp->stats.icrxatc += E1000_READ_REG(&lp->hw, E1000_ICRXATC);
	lp->stats.ictxptc += E1000_READ_REG(&lp->hw, E1000_ICTXPTC);
	lp->stats.ictxatc += E1000_READ_REG(&lp->hw, E1000_ICTXATC);
	lp->stats.ictxqec += E1000_READ_REG(&lp->hw, E1000_ICTXQEC);
	lp->stats.ictxqmtc += E1000_READ_REG(&lp->hw, E1000_ICTXQMTC);
	lp->stats.icrxdmtc += E1000_READ_REG(&lp->hw, E1000_ICRXDMTC);
	lp->stats.icrxoc += E1000_READ_REG(&lp->hw, E1000_ICRXOC);

	if (lp->hw.mac.type >= e1000_82543) {
		lp->stats.algnerrc += E1000_READ_REG(&lp->hw, E1000_ALGNERRC);
		lp->stats.rxerrc += E1000_READ_REG(&lp->hw, E1000_RXERRC);
		lp->stats.tncrs += E1000_READ_REG(&lp->hw, E1000_TNCRS);
		lp->stats.cexterr += E1000_READ_REG(&lp->hw, E1000_CEXTERR);
		lp->stats.tsctc += E1000_READ_REG(&lp->hw, E1000_TSCTC);
		lp->stats.tsctfc += E1000_READ_REG(&lp->hw, E1000_TSCTFC);
	}
#ifdef notdef
	ifp = lp->ifp;

	ifp->if_collisions = lp->stats.colc;

	/* Rx Errors */

	ifp->if_ierrors =
	    lp->dropped_pkts +
	    lp->stats.rxerrc +
	    lp->stats.crcerrs +
	    lp->stats.algnerrc +
	    lp->stats.ruc + lp->stats.roc +
	    lp->stats.mpc + lp->stats.cexterr;

	/* Tx Errors */
	ifp->if_oerrors = lp->stats.ecol + lp->stats.latecol;
#endif

	return (GEM_SUCCESS);
}

/*
 * tx and rx discriptor  manupiration
 */
#ifdef notdef
#define	TXD_OFFSET(m)	((int)((long)&(((struct e1000_tx_desc *)0)->m)))
#define	RXD_OFFSET(m)	((int)((long)&(((struct e1000_rx_desc *)0)->m)))

#define	TXD64(tdp, member)	((uint64_t *)(tdp))[TXD_OFFSET(member)/8]
#define	TXD32(tdp, member)	((uint32_t *)(tdp))[TXD_OFFSET(member)/4]
#define	TXD16(tdp, member)	((uint16_t *)(tdp))[TXD_OFFSET(member)/2]
#define	TXD8(tdp, member)	((uint8_t *)(tdp))[TXD_OFFSET(member)]

#define	RXD64(rdp, member)	((uint64_t *)(rdp))[RXD_OFFSET(member)/8]
#define	RXD32(rdp, member)	((uint32_t *)(rdp))[RXD_OFFSET(member)/4]
#define	RXD16(rdp, member)	((uint16_t *)(rdp))[RXD_OFFSET(member)/2]
#define	RXD8(rdp, member)	((uint8_t *)(rdp))[RXD_OFFSET(member)]
#else
#define	TXD64(tdp, member)	(tdp)->member
#define	TXD32(tdp, member)	(tdp)->member
#define	TXD16(tdp, member)	(tdp)->member
#define	TXD8(tdp, member)	(tdp)->member

#define	RXD64(rdp, member)	(rdp)->member
#define	RXD32(rdp, member)	(rdp)->member
#define	RXD16(rdp, member)	(rdp)->member
#define	RXD8(rdp, member)	(rdp)->member
#endif

#if 1
static int
em_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	uint_t			used;
	uint32_t		txd_upper;
	uint32_t		txd_lower;
#ifdef CONFIG_VLAN
	uint16_t		vtag;
#endif
	struct e1000_tx_desc	*tdp;
	ddi_dma_cookie_t	*dcp;
	struct em_dev		*lp = dp->private;
	int			i;
	uint64_t		addr;
	uint32_t		len;
#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	uint32_t		hck_start;
	uint32_t		hck_stuff;
	uint32_t		mss = 0;
#endif

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    CONS "%s: time:%d %s seqnum: %d, slot %d, frags: %d flags: %lld",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_active_tail, slot, frags, flags);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
	txd_upper = 0;
	txd_lower = 0;
	used = 0;
#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	if (flags & (GEM_TXFLAG_MSS | GEM_TXFLAG_TCP | GEM_TXFLAG_UDP)) {
		struct e1000_context_desc	*tcp;
		struct e1000_context_desc	new_ctx;
		uint32_t	hcksum_cmd;
#define	EHDLEN	sizeof (struct ether_header)

		txd_upper = E1000_TXD_POPTS_TXSM << 8;
		if (flags & GEM_TXFLAG_IPv4) {
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
		}
		txd_lower = (E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D);

		mss = (flags & GEM_TXFLAG_MSS) >> GEM_TXFLAG_MSS_SHIFT;

		hck_start = (flags & GEM_TXFLAG_HCKSTART) >>
		    GEM_TXFLAG_HCKSTART_SHIFT;
		hck_stuff = (flags & GEM_TXFLAG_HCKSTUFF) >>
		    GEM_TXFLAG_HCKSTUFF_SHIFT;

		bzero(&new_ctx, sizeof (new_ctx));
		tcp = &new_ctx;

		/* IPv4 header */
		if (flags & GEM_TXFLAG_IPv4) {
			tcp->lower_setup.ip_fields.ipcss = EHDLEN;
			tcp->lower_setup.ip_fields.ipcso = 
			    EHDLEN + offsetof(struct ip, ip_sum);
			tcp->lower_setup.ip_fields.ipcse = LE_16(hck_start - 1);
			hcksum_cmd = E1000_TXD_CMD_IP;
		} else {
			/* others, i.e. IPv6 */
			tcp->lower_setup.ip_config = 0;
			hcksum_cmd = 0;
		}

		/* TCP or UDP */
		tcp->upper_setup.tcp_fields.tucss = hck_start;
		tcp->upper_setup.tcp_fields.tucse = 0;
		tcp->upper_setup.tcp_fields.tucso = hck_stuff;

	 	if (flags & GEM_TXFLAG_TCP) {
			/*
			 * XXX - we must request IP checksum too, otherwize
			 * ip check sum field will be cleared.
			 */
			hcksum_cmd |= E1000_TXD_CMD_TCP;
		}

		tcp->tcp_seg_setup.data = 0;
		if (mss) {
			txd_lower |= E1000_TXD_CMD_TSE;

                        tcp->tcp_seg_setup.fields.hdr_len =
			    ((flags & GEM_TXFLAG_TCPHLEN)
			    >> GEM_TXFLAG_TCPHLEN_SHIFT) << 2;
                        tcp->tcp_seg_setup.fields.mss = LE_16(mss);
		}

		tcp->cmd_and_length =
		    LE_32(hcksum_cmd | E1000_TXD_CMD_DEXT | lp->txd_cmd);

		if (bcmp(tcp, &lp->tx_ctx, sizeof (*tcp)) != 0 || mss) {
			/* need to update context */
			tcp = &((struct e1000_context_desc *)dp->tx_ring)[slot];
			*tcp = new_ctx;
			lp->tx_ctx = new_ctx;
			
			/* we consumed an additional slot */
			used++;
		}
#undef EHDLEN
	}
#endif /* CONFIG_CKSUM_OFFLOAD  && defined(CONFIG_VLAN) */

	txd_lower |= lp->txd_cmd;

	for (dcp = dmacookie, i = 0; i < frags; dcp++, i++) {
		tdp = &((struct e1000_tx_desc *)dp->tx_ring)[
		    SLOT(slot + used, dp->gc.gc_tx_ring_size)];
		used++;

		addr = dcp->dmac_laddress;
		len = dcp->dmac_size;
		if (lp->pcix_82544 && len > 4) {
			switch (((addr & 0x7) + (len & 0xf)) & 0xf) {
			case 1: case 2: case 3: case 4:
			case 9: case 0xa: case 0xb: case 0xc:
				len -= 4;
				tdp->buffer_addr = LE_64(addr);
				tdp->lower.data = LE_32(txd_lower | len);
				tdp->upper.data = LE_32(txd_upper);

				addr += len;
				len = 4;

				tdp = &((struct e1000_tx_desc *)dp->tx_ring)[
				    SLOT(slot + used, dp->gc.gc_tx_ring_size)];
				used++;
			}
		}

		tdp->buffer_addr = LE_64(addr);
		tdp->lower.data = LE_32(txd_lower | len);
		tdp->upper.data = LE_32(txd_upper);
	}

	/* fix last descriptor */
#ifdef CONFIG_VLAN
	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		DPRINTF(2, (CE_CONT, CONS "%s: %s: vtag:%x",
		    dp->name, __func__, vtag));
		tdp->lower.data |= LE_32(E1000_TXD_CMD_VLE);
		tdp->upper.fields.special = LE_16(vtag);
	}
#if defined(CONFIG_CKSUM_OFFLOAD)
	if (mss) {
		tdp->lower.data |= LE_32(E1000_TXD_CMD_TSE);
	}
#endif
#endif
	tdp->lower.data |= LE_32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);

	return (used);
}
#else
/* from t8 */
static int
em_tx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags, uint64_t flags)
{
	uint_t			used;
	uint32_t		new_ctx;
	uint32_t		txd_upper;
	uint32_t		txd_lower;
#ifdef CONFIG_VLAN
	uint16_t		vtag;
#endif
	struct e1000_tx_desc	*tdp;
	ddi_dma_cookie_t	*dcp;
	struct em_dev		*lp = dp->private;
	int			i;
	uint64_t		addr;
	uint64_t		len;
	uint32_t		hck_start;
	uint32_t		hck_stuff;

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
	    CONS "%s: time:%d %s seqnum: %d, slot %d, frags: %d flags: %lld",
	    dp->name, ddi_get_lbolt(), __func__,
	    dp->tx_active_tail, slot, frags, flags);

	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "%d: addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	/*
	 * write tx descriptor in reversed order.
	 */
	txd_upper = 0;
	txd_lower = 0;
	used = 0;
#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	new_ctx = 0;
	if (flags & (GEM_TXFLAG_TCP | GEM_TXFLAG_UDP)) {
		txd_upper = E1000_TXD_POPTS_TXSM << 8;
		if (flags & GEM_TXFLAG_IPv4) {
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
		}
		txd_lower = (E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D);
		new_ctx = flags &
		    (GEM_TXFLAG_TCP | GEM_TXFLAG_UDP |
		    GEM_TXFLAG_IPv4 | GEM_TXFLAG_IPv6);
	}

	if (new_ctx != lp->tx_cksum_ctx && new_ctx != 0) {
		struct e1000_context_desc *tcp;
		uint32_t	hcksum_cmd;
#define	EHDLEN	sizeof (struct ether_header)

		hck_start =
		    (flags & GEM_TXFLAG_HCKSTART) >> GEM_TXFLAG_HCKSTART_SHIFT;
		hck_stuff =
		    (flags & GEM_TXFLAG_HCKSTUFF) >> GEM_TXFLAG_HCKSTUFF_SHIFT;

		tcp = &((struct e1000_context_desc *)dp->tx_ring)[slot];

		/* IPv4 header */
		if (new_ctx & GEM_TXFLAG_IPv4) {
			tcp->lower_setup.ip_fields.ipcss = EHDLEN;
			tcp->lower_setup.ip_fields.ipcso = 
			    EHDLEN + offsetof(struct ip, ip_sum);
			tcp->lower_setup.ip_fields.ipcse = LE_16(hck_start - 1);
			hcksum_cmd = E1000_TXD_CMD_IP;
		} else {
			/* others, i.e. IPv6 */
			tcp->lower_setup.ip_config = 0;
			hcksum_cmd = 0;
		}

		/* TCP or UDP */
		tcp->upper_setup.tcp_fields.tucss = hck_start;
		tcp->upper_setup.tcp_fields.tucse = 0;
		tcp->upper_setup.tcp_fields.tucso = hck_stuff;

	 	if (new_ctx & GEM_TXFLAG_TCP) {
			/*
			 * XXX - we must request IP checksum too, otherwize
			 * ip check sum field will be cleared.
			 */
			hcksum_cmd |= E1000_TXD_CMD_TCP;
		}

		tcp->tcp_seg_setup.data = 0;
		tcp->cmd_and_length =
		    LE_32(hcksum_cmd | E1000_TXD_CMD_DEXT | lp->txd_cmd);

		/* we consumed an additional slot */
		used++;
#undef EHDLEN
	}
	lp->tx_cksum_ctx = new_ctx;
#endif /* CONFIG_CKSUM_OFFLOAD  && defined(CONFIG_VLAN) */

	txd_lower |= lp->txd_cmd;

	for (dcp = dmacookie, i = 0; i < frags; dcp++, i++) {
		tdp = &((struct e1000_tx_desc *)dp->tx_ring)[
		    SLOT(slot + used, dp->gc.gc_tx_ring_size)];
		used++;

		addr = dcp->dmac_laddress;
		len = dcp->dmac_size;
		if (lp->pcix_82544 && len > 4) {
			switch (((addr & 0x7) + (len & 0xf)) & 0xf) {
			case 1: case 2: case 3: case 4:
			case 9: case 0xa: case 0xb: case 0xc:
				len -= 4;
				tdp->buffer_addr = LE_64(addr);
				tdp->lower.data = LE_32(txd_lower | len);
				tdp->upper.data = LE_32(txd_upper);

				addr += len;
				len = 4;

				tdp = &((struct e1000_tx_desc *)dp->tx_ring)[
				    SLOT(slot + used, dp->gc.gc_tx_ring_size)];
				used++;
			}
		}

		tdp->buffer_addr = LE_64(addr);
		tdp->lower.data = LE_32(txd_lower | len);
		tdp->upper.data = LE_32(txd_upper);
	}

	/* fix last descriptor */
#ifdef CONFIG_VLAN
	vtag = (flags & GEM_TXFLAG_VTAG) >> GEM_TXFLAG_VTAG_SHIFT;
	if (vtag) {
		DPRINTF(2, (CE_CONT, CONS "%s: %s: vtag:%x",
		    dp->name, __func__, vtag));
		tdp->lower.data |= LE_32(E1000_TXD_CMD_VLE);
		tdp->upper.fields.special = LE_16(vtag);
	}
#endif
	tdp->lower.data |= LE_32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);

	return (used);
}
#endif

static void
em_tx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	struct em_dev	*lp = dp->private;

	/* first of all, record the real descriptor tail position */
	lp->tx_tail_real = SLOT(start_slot + nslots, dp->gc.gc_tx_ring_size);

	if (
#ifndef TEST_82547
	    lp->hw.mac.type == e1000_82547 &&
#endif
	    !dp->full_duplex) {
		uint32_t	txd_lower;
		int		accepted;
		int		len;
		int		slot;
		int		i;
		struct e1000_tx_desc *tdp;

		if (lp->tx_fifo_full) {
			/* do nothing */
			return;
		}

		len = 0;
		accepted = 0;
		for (i = 0; i < nslots; i++) {
			slot = SLOT(start_slot + i, dp->gc.gc_tx_ring_size);
			tdp = &((struct e1000_tx_desc *)dp->tx_ring)[slot];
			txd_lower = tdp->lower.data;
			txd_lower = LE_32(txd_lower);

			if ((txd_lower &
			    (E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D))
			    == E1000_TXD_CMD_DEXT) {
				/* this is not real fragment */
				continue;
			}

			/* accumulate fragment size */
			len += txd_lower & 0xffff;

			if ((txd_lower & E1000_TXD_CMD_EOP) == 0) {
				continue;
			}

			accepted = i + 1;

			/* we are at the last fragment in a packet */
			lp->tx_fifo_head +=
			    ROUNDUP(len + EM_FIFO_HDR, EM_FIFO_HDR);
			len = 0;

			if (lp->tx_fifo_head > lp->tx_fifo_thresh) {
				/* this is the last packet we can accept */
				lp->tx_fifo_full = B_TRUE;
				DPRINTF(3, (CE_CONT,
				    CONS "%s: %s: tx fifo is full",
				    dp->name, __func__));
				break;
			}
		}
		DPRINTF(3, (CE_CONT, CONS "%s: %s: nslots:%d, accepted:%d",
		    dp->name, __func__, nslots, accepted));
		/* fix valid slots */
		ASSERT(accepted > 0);
		nslots = accepted;
	}

	gem_tx_desc_dma_sync(dp, start_slot, nslots, DDI_DMA_SYNC_FORDEV);

	if (dp->mac_active) {
		lp->tx_tail = SLOT(start_slot + nslots, dp->gc.gc_tx_ring_size);
		E1000_WRITE_REG(&lp->hw, E1000_TDT(0), lp->tx_tail);
	}
}

static void
em_rx_desc_write(struct gem_dev *dp, int slot,
    ddi_dma_cookie_t *dmacookie, int frags)
{
	struct e1000_rx_desc	*rdp;
	struct em_dev		*lp = dp->private;
#if DEBUG_LEVEL > 2
	int			i;

	ASSERT(frags == 1);

	cmn_err(CE_CONT,
	    CONS "%s: %s seqnum: %d, slot %d, frags: %d",
	    dp->name, __func__, dp->rx_active_tail, slot, frags);
	for (i = 0; i < frags; i++) {
		cmn_err(CE_CONT, CONS "  frag: %d addr: 0x%llx, len: 0x%x",
		    i, dmacookie[i].dmac_laddress, dmacookie[i].dmac_size);
	}
#endif
	rdp = &((struct e1000_rx_desc *)dp->rx_ring)[slot];

	rdp->buffer_addr = LE_64(dmacookie->dmac_laddress);
	rdp->status = 0;
}

static void
em_rx_start(struct gem_dev *dp, int start_slot, int nslots)
{
	struct em_dev	*lp = dp->private;

	gem_rx_desc_dma_sync(dp, start_slot, nslots, DDI_DMA_SYNC_FORDEV);
	if (dp->mac_active) {
		E1000_WRITE_REG(&lp->hw,
		    E1000_RDT(0),
		    SLOT(start_slot + nslots, dp->gc.gc_rx_ring_size));
	}
}

static int
em_82547_tx_fifo_reset(struct gem_dev *dp)
{
	uint32_t	tdt;
	uint32_t	tdh;
	uint32_t	tdft;
	uint32_t	tdfh;
	uint32_t	tdfts;
	uint32_t	tdfhs;
	uint32_t	tdfpc;
	uint32_t	tctl;
	struct em_dev	*lp = dp->private;

	if (((tdt = E1000_READ_REG(&lp->hw, E1000_TDT(0))) ==
	    (tdh = E1000_READ_REG(&lp->hw, E1000_TDH(0)))) &&
	    ((tdft = E1000_READ_REG(&lp->hw, E1000_TDFT)) ==
	    (tdfh = E1000_READ_REG(&lp->hw, E1000_TDFH))) &&
	    ((tdfts = E1000_READ_REG(&lp->hw, E1000_TDFTS)) ==
	    (tdfhs = E1000_READ_REG(&lp->hw, E1000_TDFHS))) &&
	    ((tdfpc = E1000_READ_REG(&lp->hw, E1000_TDFPC)) == 0)) {

		/* Disable TX unit */
		tctl = E1000_READ_REG(&lp->hw, E1000_TCTL);
		E1000_WRITE_REG(&lp->hw, E1000_TCTL, tctl & ~E1000_TCTL_EN);

		/* Reset FIFO pointers */
		E1000_WRITE_REG(&lp->hw, E1000_TDFT,  lp->tx_head_addr);
		E1000_WRITE_REG(&lp->hw, E1000_TDFH,  lp->tx_head_addr);
		E1000_WRITE_REG(&lp->hw, E1000_TDFTS, lp->tx_head_addr);
		E1000_WRITE_REG(&lp->hw, E1000_TDFHS, lp->tx_head_addr);

		/* Re-enable TX unit */
		E1000_WRITE_REG(&lp->hw, E1000_TCTL, tctl);
		E1000_WRITE_FLUSH(&lp->hw);

		lp->tx_fifo_head = 0;
		lp->tx_fifo_reset_cnt++;
		lp->tx_fifo_full = B_FALSE;
		DPRINTF(3, (CE_CONT,
		    CONS "%s: %s: tx fifo was reset successfully",
		    dp->name, __func__));
		return (GEM_SUCCESS);
	} else {
		DPRINTF(3, (CE_CONT, CONS "%s: %s: failed to reset tx fifo: "
		    "tdt:%x, tdh:%x, "
		    "tdft:%x, tdfh:%x, "
		    "tdfts:%x, tdfhs:%x, "
		    "tdfpc:%x",
		    dp->name, __func__,
		    tdt, tdh,
		    tdft, tdfh,
		    tdfts, tdfhs,
		    tdfpc));
		return (GEM_FAILURE);
	}
}

static void
em_txfifo_restart(struct gem_dev *dp)
{
	int		rest;
	struct em_dev	*lp = dp->private;

	if (em_82547_tx_fifo_reset(dp) == GEM_SUCCESS) {
		ASSERT(!lp->tx_fifo_full);

		/* we also reload descriptors */
		if ((rest = lp->tx_tail_real - lp->tx_tail) < 0) {
			rest += dp->gc.gc_tx_ring_size;
		}
		if (rest > 0) {
			em_tx_start(dp, lp->tx_head, rest);
		}
		lp->txfifo_watcher_id = 0;
	} else {
		/*
		 * We cannot load the tx descriptors for the time
		 * being, because tx fifo isn't empty now.
		 * Wait while tx fifo is draining.
		 */
		lp->txfifo_watcher_id =
		    timeout(em_txfifo_watcher, (void *)dp, 1);
	}
}

static void
em_txfifo_watcher(void *arg)
{
	struct gem_dev	*dp = arg;
	struct em_dev	*lp = dp->private;

	mutex_enter(&dp->intrlock);
	mutex_enter(&dp->xmitlock);
	if (!lp->tx_fifo_full) {
		cmn_err(CE_WARN, "!%s: %s: tx fifo isn't full",
		    dp->name, __func__);
		goto x;
	}

	em_txfifo_restart(dp);

x:
	mutex_exit(&dp->xmitlock);
	mutex_exit(&dp->intrlock);
}

#ifdef GEM_CONFIG_TX_HEAD_PTR
static uint_t
em_tx_desc_head(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;

	if (lp->nic_active) {
		lp->tx_head = E1000_READ_REG(&lp->hw, E1000_TDH(0));
	}
	return (lp->tx_head);
}
#else
#define	INSIDE(slot, head, tail)	\
	(((head) <= (tail)) ?	\
		((head) <= (slot) && (slot) < (tail)) :	\
		((slot) < (tail) || (head) <= (slot)))
static uint_t
em_tx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint8_t			status;
	int			i;
	int			last_slot;
	int			cols;
	struct e1000_tx_desc	*tdp;
	struct em_dev		*lp = dp->private;

	tdp = &((struct e1000_tx_desc *)dp->tx_ring)[
	    SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size)];

	status = tdp->upper.fields.status;

	DPRINTF(1, (CE_CONT,
	    CONS "%s: time:%d %s: slot:%d, status:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, status));
#ifdef CONFIG_USE_TXDWB
	if ((status & E1000_TXD_STAT_DD) == 0) {
		return (0);
	}
	lp->tx_head = SLOT(slot + ndesc, dp->gc.gc_tx_ring_size);
#else
	if (lp->nic_active) {
		lp->tx_head = E1000_READ_REG(&lp->hw, E1000_TDH(0));
	}
	last_slot = SLOT(slot + ndesc - 1, dp->gc.gc_tx_ring_size);
	if (INSIDE(last_slot, lp->tx_head, lp->tx_tail)) {
		return (0);
	}
#endif
	if (lp->tx_fifo_full && lp->tx_head == lp->tx_tail) {
		em_txfifo_restart(dp);
	}

	return (GEM_TX_DONE);
}
#endif /* GEM_CONFIG_TX_HEAD_PTR */

static uint64_t
em_rx_desc_stat(struct gem_dev *dp, int slot, int ndesc)
{
	uint64_t		len;
	uint64_t		flags;
	uint8_t			status;
	struct e1000_rx_desc	*rdp;
	struct em_dev		*lp = dp->private;

	rdp = &((struct e1000_rx_desc *)dp->rx_ring)[slot];

	status = LE_8(rdp->status);

	DPRINTF(2, (CE_CONT,
	    CONS "%s: time:%d %s: slot:%d, status:0x%x",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, status));

	if ((status & E1000_RXD_STAT_DD) == 0) {
		return (0);
	}
#ifdef CONFIG_NO_RXCRC
	len = LE_16(rdp->length);
#else
	len = LE_16(rdp->length) - ETHERFCSL;
#endif
	DPRINTF(1, (CE_CONT,
	    CONS "%s: time:%d %s: slot:%d, status:0x%x, errors:0x%x, len:%d",
	    dp->name, ddi_get_lbolt(), __func__,
	    slot, status, rdp->errors, len));

	flags = GEM_RX_DONE;

#ifdef CONFIG_VLAN
	if (status & E1000_RXD_STAT_VP) {
		flags |= ((uint64_t)LE_16(rdp->special)) << GEM_RX_VTAG_SHIFT;
	}
#endif
#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	/* XXX - we have full hw checksum function for rx */
	if ((lp->hw.mac.type >= e1000_82543) &&
	    (status & E1000_RXD_STAT_IXSM) == 0) {

		if ((status & E1000_RXD_STAT_IPCS) &&
		    (rdp->errors & E1000_RXD_ERR_IPE) == 0) {
			flags |= GEM_RX_CKSUM_IPv4;
		}

		if ((status & E1000_RXD_STAT_TCPCS) &&
		    (rdp->errors & E1000_RXD_ERR_TCPE) == 0) {
			/* this includes both of tcp and udp */
			flags |= (GEM_RX_CKSUM_TCP | GEM_RX_CKSUM_UDP);
		}
	}
#endif
	return (flags | (len & GEM_RX_LEN));
}

static void
em_tx_desc_init(struct gem_dev *dp, int slot)
{
	bzero(&((struct e1000_tx_desc *)dp->tx_ring)[slot],
	    sizeof (struct e1000_tx_desc));
}

static void
em_rx_desc_init(struct gem_dev *dp, int slot)
{
	bzero(&((struct e1000_rx_desc *)dp->rx_ring)[slot],
	    sizeof (struct e1000_rx_desc));
}

/*
 * Device depend interrupt handler
 */
static uint_t
em_interrupt(struct gem_dev *dp)
{
	uint32_t	reg_icr;
	uint_t		flags = 0;
	struct em_dev	*lp = dp->private;

	reg_icr = E1000_READ_REG(&lp->hw, E1000_ICR);

	DPRINTF(4, (CE_CONT, CONS "%s: time:%d %s: icr:%x",
	    dp->name, ddi_get_lbolt(), __func__, reg_icr));

	if (reg_icr == 0 || (lp->hw.mac.type >= e1000_82571 &&
	    (reg_icr & E1000_ICR_INT_ASSERTED) == 0) ||
	/*
	 * XXX: some laptops trigger several spurious interrupts
	 * on em(4) when in the resume cycle. The ICR register
	 * reports all-ones value in this case. Processing such
	 * interrupts would lead to a freeze. I don't know why.
	 */
	    reg_icr == 0xffffffffU) {
		return (DDI_INTR_UNCLAIMED);
	} else if ((reg_icr & lp->our_intr_mask) == 0) {
		/* not for us */
		return (DDI_INTR_UNCLAIMED);
	}

	DPRINTF(1, (CE_CONT, CONS "%s: time:%d %s: icr:%x",
	    dp->name, ddi_get_lbolt(), __func__, reg_icr));

	if (!dp->mac_active) {
		return (DDI_INTR_CLAIMED);
	}

	reg_icr &= lp->our_intr_mask;

	if (reg_icr & (E1000_ICR_RXT0 | E1000_ICR_RXDMT0)) {
		(void) gem_receive(dp);
	}

	if (reg_icr & (E1000_ICR_TXDW | E1000_ICR_TXQE)) {
		if (gem_tx_done(dp)) {
			flags |= INTR_RESTART_TX;
		}
	}

	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		DPRINTF(0, (CE_NOTE, CONS "%s: %s: link status changed, icr:%x",
		    dp->name, __func__, reg_icr));
		lp->hw.mac.get_link_status = 1;
#ifdef GEM3
		gem_mii_link_check(dp);
#else
		if (gem_mii_link_check(dp)) {
			flags |= INTR_RESTART_TX;
		}
#endif
	}

	if (reg_icr & E1000_ICR_RXO) {
		cmn_err(CE_NOTE, "!%s: RX overrun", dp->name);
	}

	return (DDI_INTR_CLAIMED | flags);
}

/* ======================================================== */
/*
 * HW depend MII routine
 */
/* ======================================================== */
static void
em_mii_sync(struct gem_dev *dp)
{
	/* nothing to do */
}


static boolean_t
em_update_link_status(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;
	u32 link_check = B_FALSE;

	/* Get the cached link value or read phy for real */
#ifdef notdef
	hw->mac.get_link_status = 1;
#endif
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			/* Do the work to read phy */
			e1000_check_for_link(hw);
			link_check = !hw->mac.get_link_status;
#ifdef notdef
			/*
			 * XXX - The lines below were moved to set_media
			 * to adapt gem framework.
			 */
			if (link_check) {
				/* ESB2 fix */
				e1000_cfg_on_link_up(hw);
			}
#endif
		} else {
			link_check = B_TRUE;
		}
		break;

	case e1000_media_type_fiber:
		e1000_check_for_link(hw);
		link_check =
		    (E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU) != 0;
		break;

	case e1000_media_type_internal_serdes:
		e1000_check_for_link(hw);
		link_check = (hw->mac.serdes_has_link) != 0;
		break;

	default:
	case e1000_media_type_unknown:
		break;
	}

#ifdef notdef
	/*
	 * XXX - The lines below were moved to set_media to adapt
	 * gem framework.
	 */

	/* Now check for a transition */
	if (link_check && (dp->mii_state != MII_STATE_LINKUP)) {

		e1000_get_speed_and_duplex(hw, &adapter->link_speed,
		    &adapter->link_duplex);

		/* Check if we must disable SPEED_MODE bit on PCI-E */
		if ((dp->speed != GEM_SPD_1000) &&
		    ((hw->mac.type == e1000_82571) ||
		    (hw->mac.type == e1000_82572))) {
			int tarc0;
			tarc0 = E1000_READ_REG(hw, E1000_TARC(0));
			tarc0 &= ~SPEED_MODE_BIT;
			E1000_WRITE_REG(hw, E1000_TARC(0), tarc0);
		}
	}
#endif
	return (link_check);
}

static uint16_t
em_mii_read(struct gem_dev *dp, uint32_t reg)
{
	uint16_t	val;
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	if (reg == MII_STATUS && hw->mac.get_link_status == 0) {
		/* use cached link status */
		val = dp->mii_status;
		if (em_update_link_status(dp)) {
			val |= MII_STATUS_LINKUP | MII_STATUS_ANDONE;
		} else {
			val &= ~(MII_STATUS_LINKUP | MII_STATUS_ANDONE);
		}

	} else if (e1000_read_phy_reg(hw, reg, &val) != E1000_SUCCESS) {
		val = 0;
	}

	return (val);
}

static void
em_mii_write(struct gem_dev *dp, uint32_t reg, uint16_t val)
{
#ifdef notdef
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	if (reg == MII_CONTROL && (val & MII_CONTROL_RESET)) {
		/* allow resetting phy and restarting autonegotiation */
		hw->phy.reset_disable = B_FALSE;
	}
#endif
	/* don't write to phy */
}

static int
em_mii_probe(struct gem_dev *dp)
{
	uint16_t	status;
	struct em_dev	*lp = dp->private;

	/*
	 * We need not scan phy here, because e1000_init_phy_params(),
	 * which has been called by e1000_setup_init_funcs(), is
	 * responsible for identifying the phy.
	 */

	/* fake gem */
	dp->mii_phy_addr = 1;

	/* force to read real phy status */
	lp->hw.mac.get_link_status = 1;

	status = gem_mii_read(dp, MII_STATUS);

	dp->mii_status = status;
	dp->mii_status_ro = ~status;
	dp->mii_phy_id  = (gem_mii_read(dp, MII_PHYIDH) << 16) |
	    gem_mii_read(dp, MII_PHYIDL);

	if (dp->mii_phy_addr < 0) {
		cmn_err(CE_CONT, "!%s: using internal/non-MII PHY(0x%08x)",
		    dp->name, dp->mii_phy_id);
	} else {
		cmn_err(CE_CONT, "!%s: MII PHY (0x%08x) found at %d",
		    dp->name, dp->mii_phy_id, dp->mii_phy_addr);
	}

	cmn_err(CE_CONT,
	    "!%s: PHY control:%b, status:%b, advert:%b, lpar:%b, exp:%b",
	    dp->name,
	    gem_mii_read(dp, MII_CONTROL), MII_CONTROL_BITS,
	    status, MII_STATUS_BITS,
	    gem_mii_read(dp, MII_AN_ADVERT), MII_ABILITY_BITS,
	    gem_mii_read(dp, MII_AN_LPABLE), MII_ABILITY_BITS,
	    gem_mii_read(dp, MII_AN_EXPANSION), MII_AN_EXP_BITS);

	dp->mii_xstatus = 0;
	if (status & MII_STATUS_XSTATUS) {
		dp->mii_xstatus = gem_mii_read(dp, MII_XSTATUS);

		cmn_err(CE_CONT, "!%s: xstatus:%b",
		    dp->name, dp->mii_xstatus, MII_XSTATUS_BITS);

		dp->mii_xstatus &= ~MII_XSTATUS_1000BASET;
	}
	dp->mii_xstatus_ro = ~dp->mii_xstatus;

	return (GEM_SUCCESS);
}

static int
em_mii_init(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	/* setup defaults for link parameters */

	hw->mac.autoneg = B_TRUE;
	hw->phy.autoneg_wait_to_complete = B_FALSE;
	hw->phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = B_FALSE;
		hw->phy.ms_type = e1000_ms_hw_default;
	}

	return (GEM_SUCCESS);
}

static void
em_mii_config_param(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;
	uint16_t	mii_stat;
	uint16_t	val;

	static uint16_t fc_cap_encode[4] = {
		/* none */		e1000_fc_none,
		/* symmetric */		e1000_fc_full,
		/* tx */		e1000_fc_tx_pause,
		/* rx-symmetric */	e1000_fc_rx_pause,
	};

	DPRINTF(0, (CE_CONT, "!%s: %s: called, nic_state:%d",
	    dp->name, __func__, dp->nic_state));

	/*
	 * Configure bits in advertisement register
	 */
	mii_stat = dp->mii_status;

	DPRINTF(1, (CE_CONT, "!%s: %s: MII_STATUS reg:%b",
	    dp->name, __func__, mii_stat, MII_STATUS_BITS));

	if ((mii_stat & MII_STATUS_ABILITY_TECH) == 0) {
		/* it's funny */
		cmn_err(CE_WARN, "!%s: wrong ability bits: mii_status:%b",
		    dp->name, mii_stat, MII_STATUS_BITS);
		return;
	}

	hw->mac.autoneg = dp->anadv_autoneg;

	val = 0;
	if (dp->anadv_100fdx) {
		val |= ADVERTISE_100_FULL;
	}
	if (dp->anadv_100hdx) {
		val |= ADVERTISE_100_HALF;
	}
	if (dp->anadv_10fdx) {
		val |= ADVERTISE_10_FULL;
	}
	if (dp->anadv_10hdx) {
		val |= ADVERTISE_10_HALF;
	}

	if (mii_stat & MII_STATUS_XSTATUS) {
		/*
		 * 1000Base-T GMII support
		 */
		if (!dp->anadv_autoneg) {
			/* enable manual configuration */
			switch (dp->anadv_1000t_ms) {
			default:
			case 0:	/* auto */
			case 1:	/* slave */
				hw->phy.ms_type = e1000_ms_force_slave;
				break;

			case 2:	/* master */
				hw->phy.ms_type = e1000_ms_force_master;
				break;
			}
		} else {
			if (dp->anadv_1000fdx) {
				val |= ADVERTISE_1000_FULL;
			}
			switch (dp->anadv_1000t_ms) {
			case 0: /* auto: do nothing */
			default:
				hw->phy.ms_type = e1000_ms_auto;
				break;

			case 1: /* slave */
				hw->phy.ms_type = e1000_ms_force_slave;
				break;

			case 2: /* master */
				hw->phy.ms_type = e1000_ms_force_master;
				break;
			}
		}
	}
	hw->phy.autoneg_advertised = val;

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: setting autoneg:%d, advertised:%x, ms_type:%d",
	    dp->name, __func__,
	    hw->mac.autoneg, hw->phy.autoneg_advertised, hw->phy.ms_type));

	/* set flow control capability */
	hw->fc.requested_mode =
	    fc_cap_encode[dp->anadv_asmpause * 2 + dp->anadv_pause];

	/* Workaround: no TX flow ctrl for PCH */
	if (hw->mac.type == e1000_pchlan) {
		switch (hw->fc.requested_mode) {
		case e1000_fc_full:
			hw->fc.requested_mode = e1000_fc_rx_pause;
			break;

		case e1000_fc_tx_pause:
			hw->fc.requested_mode = e1000_fc_none;
			break;
		}
	}

	DPRINTF(0, (CE_CONT,
	    "!%s: %s: setting fc_requested_mode:%d, pause:%d, asmpause:%d",
	    dp->name, __func__,  hw->fc.requested_mode,
	    dp->anadv_pause, dp->anadv_asmpause));

	lp->hw.fc.current_mode = lp->hw.fc.requested_mode;
	e1000_force_mac_fc(&lp->hw);
}

static int
em_mii_config(struct gem_dev *dp)
{
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	if (dp->nic_state < NIC_STATE_INITIALIZED) {
		return (GEM_FAILURE);
	}

	em_mii_config_param(dp);

	/* set parameters to PHY */
#ifdef CONFIG_SEPARETE_PHY
	(* lp->setup_link)(hw);
#else
	e1000_setup_link(hw);
#endif

#ifdef notdef
	/* avoid to restart auto-negotiation on autonegotiation timeout */
	hw->phy.reset_disable = B_TRUE;
#endif
	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 * OS depend (device driver) routine
 */
/* ======================================================== */

static void
em_fixup_params(struct gem_dev *dp)
{
	struct nfo_dev	*lp = dp->private;

	DPRINTF(0, (CE_CONT, "!%s: %s: called at time:%d",
	    dp->name, __func__, ddi_get_lbolt()));

	/* fix rx buffer size according with the HW spec.  */
	if (dp->rx_buf_len <= 2048) {
		dp->rx_buf_len = 2048;
	} else if (dp->rx_buf_len <= 4096) {
		dp->rx_buf_len = 4096;
	} else if (dp->gc.gc_max_mtu != 9234 - sizeof (struct ether_header)
	    && dp->rx_buf_len <= 8192) {
		dp->rx_buf_len = 8192;
	} else if (dp->gc.gc_max_mtu == 9234 - sizeof (struct ether_header)
	    && dp->rx_buf_len <= 9234) {
		dp->rx_buf_len = 9234;
	} else if (dp->rx_buf_len <= 16384) {
		dp->rx_buf_len = 16384;
	}
}

static int
em_attach_chip(struct gem_dev *dp)
{
	uint32_t	val;
	uint16_t	rx_buffer_size;
	struct em_dev	*lp = dp->private;
	struct e1000_hw	*hw = &lp->hw;

	DPRINTF(0, (CE_CONT, CONS "%s: %s: called", dp->name, __func__));

	em_get_wakeup(dp);

	if (lp->has_manage && !lp->has_amt) {
		em_get_hw_control(dp);
	}
#ifdef notdef	/* moved to mii_init() */
	/* setup defaults for link parameters */

	hw->mac.autoneg = B_TRUE;
	hw->phy.autoneg_wait_to_complete = B_FALSE;
	hw->phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = B_FALSE;
		hw->phy.ms_type = e1000_ms_hw_default;
	}
#endif
	/* XXX - continuing e1000_attach */

	/* TODO: new em has only 2048, 4096 and 9K */
	/* fix rx buffer size according with the HW spec.  */
	if (dp->rx_buf_len > 16384) {
		cmn_err(CE_WARN,
		    CONS "%s: %s: rx buffer size is too big (%d), using 16384",
		    dp->name, __func__, dp->rx_buf_len);
		dp->rx_buf_len = 16384;
	}
	em_fixup_params(dp);

	e1000_init_script_state_82541(&lp->hw, B_TRUE);
	e1000_set_tbi_compatibility_82543(&lp->hw, B_TRUE);

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = B_FALSE;
		hw->phy.ms_type = E1000_MASTER_SLAVE;
	}

	/*
	 * Set the max frame size assuming standard ethernet
	 * sized frames
	 */
#ifdef CONFIG_NO_RXCRC
	lp->max_frame_size = dp->mtu + sizeof (struct ether_header);
	lp->min_frame_size = ETHERMIN;
#else
	lp->max_frame_size = dp->mtu + sizeof (struct ether_header) + ETHERFCSL;
	lp->min_frame_size = ETHERMIN + ETHERFCSL;
#endif
	lp->tx_fifo_thresh = dp->mtu - sizeof (struct ether_header) - 4 - 32;
#ifdef CONFIG_NEW_EM
	lp->hw.mac.max_frame_size = lp->max_frame_size;
#endif

	/*
	 * This controls when hardware reports transmit completion
	 * status.
	 */
	hw->mac.report_tx_early = 1;

	/*
	 * EEPROM sections
	 */
	/* Make sure we have a good EEPROM before we read from it */
	if (e1000_validate_nvm_checksum(hw) < 0) {
		/*
		 * Some PCI-E parts fail the first check due to
		 * the link being in sleep state, call it again,
		 * if it fails a second time its a real issue.
		 */
		if (e1000_validate_nvm_checksum(hw) < 0) {
			cmn_err(CE_WARN,
			    CONS "%s: %s: The EEPROM Checksum Is Not Valid",
			    dp->name, __func__);
			return (GEM_FAILURE);
		}
	}

	/* Copy the permanent MAC address out of the EEPROM */
	if (e1000_read_mac_addr(hw) < 0) {
		cmn_err(CE_WARN,
		    CONS "%s: %s: EEPROM read error while reading mac addr",
		    dp->name, __func__);
#ifndef DEBUG
		return (GEM_FAILURE);
#endif
	}

	if (!em_is_valid_ether_addr(hw->mac.perm_addr)) {
		cmn_err(CE_WARN, CONS "%s: %s:Invalid mac address",
		    dp->name, __func__);
#ifndef DEBUG
		return (GEM_FAILURE);
#endif
	}

	bcopy(hw->mac.perm_addr, dp->dev_addr.ether_addr_octet, ETHERADDRL);
#ifdef notdef
	bzero(hw->mac.addr, ETHERADDRL);
#else
	bcopy(hw->mac.perm_addr, hw->mac.addr, ETHERADDRL);
#endif
	/* Indicate SOL/IDER usage */
	if (e1000_check_reset_block(&lp->hw)) {
		cmn_err(CE_NOTE,
		    CONS
		    "%s: %s: PHY reset is blocked due to SOL/IDER session.",
		    dp->name, __func__);
	}

#if defined(CONFIG_CKSUM_OFFLOAD) && defined(CONFIG_VLAN)
	if (hw->mac.type >= e1000_82543) {
		dp->misc_flag |= GEM_CKSUM_PARTIAL;
		dp->misc_flag |= GEM_CKSUM_HEADER_IPv4;
	}
#endif
#ifdef CONFIG_VLAN
	dp->misc_flag |= GEM_VLAN_HARD;
#endif
	cmn_err(CE_CONT, CONS "%s: firmware version: %s",
	    dp->name, FIRMWARE_VERSION);

	return (GEM_SUCCESS);
}

/* ======================================================== */
/*
 *  Determine hardware revision.
 */
/* ======================================================== */
static int
em_identify_hardware(dev_info_t *dip, struct em_dev *lp)	/* ok */
{
	const int	unit = ddi_get_instance(dip);
	const char	*drv_name = ddi_driver_name(dip);

	/* Make sure our PCI config space has the necessary stuff set */
	lp->hw.bus.pci_cmd_word = pci_config_get16(lp->conf_ha, PCI_CONF_COMM);

	if (!((lp->hw.bus.pci_cmd_word & PCI_COMM_ME) &&
	    (lp->hw.bus.pci_cmd_word & PCI_COMM_MAE))) {
		cmn_err(CE_CONT, CONS "%s%d: %s: "
		    "Memory Access and/or Bus Master bits were not set!",
		    drv_name, unit, __func__);
		lp->hw.bus.pci_cmd_word |= (PCI_COMM_ME | PCI_COMM_MAE);
		pci_config_put16(lp->conf_ha,
		    PCI_CONF_COMM, lp->hw.bus.pci_cmd_word);
	}

	/* Save off the information about this board */
	lp->hw.vendor_id = pci_config_get16(lp->conf_ha, PCI_CONF_VENID);
	lp->hw.device_id = pci_config_get16(lp->conf_ha, PCI_CONF_DEVID);
	lp->hw.revision_id = pci_config_get8(lp->conf_ha, PCI_CONF_REVID);
	lp->hw.subsystem_vendor_id =
	    pci_config_get16(lp->conf_ha, PCI_CONF_SUBVENID);
	lp->hw.subsystem_device_id =
	    pci_config_get16(lp->conf_ha, PCI_CONF_SUBSYSID);

	/* Do Shared Code Init and Setup */
	if (e1000_set_mac_type(&lp->hw)) {
		cmn_err(CE_WARN, CONS "%s%d: %s: Setup init failure",
		    drv_name, unit, __func__);
		return (DDI_FAILURE);
	}

	DPRINTF(0, (CE_CONT, "!%s%d: %s: type:%d",
	    drv_name, unit, __func__, lp->hw.mac.type));

	return (DDI_SUCCESS);
}

static void
e1000_free_pci_resources(struct em_dev *lp)
{
	lp->hw.hw_addr = 0;

	if (lp->io_ha) {
		ddi_regs_map_free(&lp->io_ha);
		lp->io_ha = 0;
	}

	if (lp->flash_ha) {
		ddi_regs_map_free(&lp->flash_ha);
		lp->flash_ha = 0;
	}
}

static int
em_allocate_pci_resources(dev_info_t *dip, struct em_dev *lp)
{
	const int	unit = ddi_get_instance(dip);
	const char	*drv_name = ddi_driver_name(dip);
	int		err = 0;

	lp->reg_ha = 0;
	lp->io_ha = 0;
	lp->flash_ha = 0;
	lp->hw.hw_addr = 0;

	/* the first base address is for mapping control registers */
	if (gem_pci_regs_map_setup(dip,
	    PCI_CONF_BASE0, PCI_REG_REG_M,
	    &em_dev_attr,
	    (void *)&lp->hw.hw_addr, &lp->reg_ha) != DDI_SUCCESS) {
		cmn_err(CE_WARN, CONS "%s%d: %s: ddi_regs_map_setup failed",
		    drv_name, unit, __func__);
		lp->hw.hw_addr = 0;
		err = ENXIO;
		goto x;
	}

	if (lp->hw.mac.type > e1000_82543) {
		/* Figure our where our IO BAR is ? */
		if (gem_pci_regs_map_setup(dip,
		    PCI_ADDR_IO, PCI_ADDR_MASK,
		    &em_dev_attr,
		    (void *)&lp->io_base, &lp->io_ha) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
			    CONS "%s%d: %s: Unable to locate IO BAR",
			    drv_name, unit, __func__);
			lp->io_ha = 0;
			err = ENXIO;
			goto x;
		}
	}

	lp->hw.back = lp;

	return (0);

x:
	e1000_free_pci_resources(lp);

	return (err);
}


static int
emattach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int			unit;
	const char		*drv_name;
	int			i;
	ddi_acc_handle_t	conf_handle;
	uint16_t		pci_cmd;
#ifdef DEBUG_LEVEL
	uint32_t		iline;
	uint8_t			latim;
#endif
	int			pcie;
	struct gem_dev		*dp;
	struct em_dev		*lp;
	void			*base;
	ddi_acc_handle_t	regs_ha;
	struct gem_conf		*gcp;
	int			num_tx_desc;
	int			num_rx_desc;
	struct e1000_hw		*hw;

	unit = ddi_get_instance(dip);
	drv_name = ddi_driver_name(dip);

	DPRINTF(3, (CE_CONT, CONS "%s%d: %s: called (%s)",
	    drv_name, unit, __func__, ident));

	/*
	 * Common codes after power-up
	 */
	if (pci_config_setup(dip, &conf_handle) != DDI_SUCCESS) {
		cmn_err(CE_WARN, CONS "%s%d: ddi_regs_map_setup failed",
		    drv_name, unit);
		return (DDI_FAILURE);
	}

#ifdef DEBUG_LEVEL
	iline =	pci_config_get32(conf_handle, PCI_CONF_ILINE);
	latim = pci_config_get8(conf_handle, PCI_CONF_LATENCY_TIMER);
#endif

	pci_config_put16(conf_handle, PCI_CONF_COMM,
	    PCI_COMM_IO | PCI_COMM_MAE | PCI_COMM_ME |
	    pci_config_get16(conf_handle, PCI_CONF_COMM));

	/* ensure D0 mode */
	(void) gem_pci_set_power_state(dip, conf_handle, PCI_PMCSR_D0);


	switch (cmd) {
	case DDI_RESUME:
#ifdef CONFIG_NEW_EM
		dp = GEM_GET_DEV(dip);
		lp = (struct em_dev *)dp->private;
		if (lp->hw.mac.type == e1000_pch2lan)
			e1000_resume_workarounds_pchlan(&lp->hw);
#endif
		if (gem_resume(dip) == DDI_SUCCESS) {
			pci_config_teardown(&conf_handle);
			return (DDI_SUCCESS);
		}
		goto err;

	case DDI_ATTACH:

		DPRINTF(0, (CE_CONT,
		    CONS "%s%d: ilr 0x%08x, latency_timer:0x%02x",
		    drv_name, unit, iline, latim));

		/*
		 * Setup device specific information
		 */
		lp = kmem_zalloc(sizeof (struct em_dev), KM_SLEEP);
		hw = &lp->hw;

		lp->conf_ha = conf_handle;
		lp->pcie_offset =
		    gem_search_pci_cap(dip, conf_handle, PCI_CAP_ID_PCI_E);
		lp->pme_offset =
		    gem_search_pci_cap(dip, conf_handle, PCI_CAP_ID_PM);

		if (em_identify_hardware(dip, lp) != DDI_SUCCESS) {
			cmn_err(CE_WARN,
			    CONS "%s%d: unknown mac type: "
			    "vid:%04x, did:%04x, svid:%04x,"
			    " ssid:%04x, rev:%02x",
			    drv_name, unit,
			    hw->vendor_id,
			    hw->device_id,
			    hw->subsystem_vendor_id,
			    hw->subsystem_device_id,
			    hw->revision_id);
			goto err_freelp;
		}

		/*
		 * Map in the device registers.
		 */
		if (em_allocate_pci_resources(dip, lp) != DDI_SUCCESS) {
			goto err_freelp;
		}

		/*
		 * For ICH8 and family we need to
		 * map the flash memory, and this
		 * must happen after the MAC is
		 * identified
		 */
		if ((hw->mac.type == e1000_ich8lan) ||
		    (hw->mac.type == e1000_ich9lan) ||
		    (hw->mac.type == e1000_ich10lan) ||
		    (hw->mac.type == e1000_pchlan) ||
		    (hw->mac.type == e1000_pch2lan) ||
		    (hw->mac.type == e1000_pch_lpt)) {
			if (gem_pci_regs_map_setup(dip,
			    E1000_FLASH, PCI_REG_REG_M,
			    &em_dev_attr,
			    (caddr_t *)&hw->flash_address,
			    &lp->flash_ha) != DDI_SUCCESS) {
				cmn_err(CE_WARN, CONS
				    "%s%d: %s: failed to map flash regisgers",
				    drv_name, unit, __func__);
				lp->flash_ha = 0;
				goto err_free_maps;
			}
		}

		/* Do Shared Code initialization */
		if (e1000_setup_init_funcs(hw, B_TRUE) != E1000_SUCCESS) {
			cmn_err(CE_WARN,
			    CONS "%s%d: %s: Setup of Shared code failed",
			    drv_name, unit, __func__);
			goto err_free_maps;
		}

#ifdef CONFIG_SEPARETE_PHY
		/*
		 * Workaround to separate mac and phy operation
		 */
		lp->setup_link = hw->mac.ops.setup_link;
		hw->mac.ops.setup_link = e1000_null_ops_generic;
#endif

		e1000_get_bus_info(hw);

		/* Set up some sysctls for the tunable interrupt delays */
		lp->rx_int_delay = em_rx_int_delay_dflt;
		lp->tx_int_delay = em_tx_int_delay_dflt;
		if (hw->mac.type >= e1000_82540) {
			lp->rx_abs_int_delay = em_rx_abs_int_delay_dflt;
			lp->tx_abs_int_delay = em_tx_abs_int_delay_dflt;
		}

		/*
		 * Validate number of transmit and receive descriptors. It
		 * must not exceed hardware maximum, and must be multiple
		 * of E1000_DBA_ALIGN.
		 */
		if (((em_txd * sizeof (struct e1000_tx_desc)) %
		    E1000_DBA_ALIGN) != 0||
		    (hw->mac.type >= e1000_82544 && em_txd > EM_MAX_TXD) ||
		    (hw->mac.type < e1000_82544 &&
		    em_txd > EM_MAX_TXD_82543) ||
		    (em_txd < EM_MIN_TXD)) {
			cmn_err(CE_CONT,
			    CONS "Using %d TX descriptors instead of %d!",
			    EM_DEFAULT_TXD, em_txd);
			num_tx_desc = EM_DEFAULT_TXD;
		} else {
			num_tx_desc = em_txd;
		}

		if (((em_rxd * sizeof (struct e1000_rx_desc)) %
		    E1000_DBA_ALIGN) != 0 ||
		    (hw->mac.type >= e1000_82544
		    && em_rxd > EM_MAX_RXD) ||
		    (hw->mac.type < e1000_82544
		    && em_rxd > EM_MAX_RXD_82543) ||
		    (em_rxd < EM_MIN_RXD)) {
			cmn_err(CE_CONT,
			    CONS "Using %d RX descriptors instead of %d!",
			    EM_DEFAULT_RXD, em_rxd);
			num_rx_desc = EM_DEFAULT_RXD;
		} else {
			num_rx_desc = em_rxd;
		}
		DPRINTF(0, (CE_CONT,
		    CONS "%s%d: num_tx_desc:%d, num_rx_desc:%d",
		    drv_name, unit, num_tx_desc, num_rx_desc));

		/* Identify 82544 on PCIX */
		if (hw->bus.type == e1000_bus_type_pcix &&
		    hw->mac.type == e1000_82544) {
			lp->pcix_82544 = B_TRUE;
		} else {
			lp->pcix_82544 = B_FALSE;
		}
#ifdef TEST_82544
		lp->pcix_82544 = B_TRUE;
#endif

		/*
		 * Construct gem configration
		 */
		gcp = kmem_zalloc(sizeof (*gcp), KM_SLEEP);

		/* name */
		sprintf(gcp->gc_name, "%s%d", drv_name, unit);

		/* consistency on tx and rx */
		gcp->gc_tx_buf_align = sizeof (uint8_t) - 1;
		gcp->gc_tx_max_frags = MAXTXFRAGS;
		if (lp->pcix_82544) {
			gcp->gc_tx_max_frags *= 2;
		}
		gcp->gc_tx_max_descs_per_pkt = gcp->gc_tx_max_frags + 1;
		gcp->gc_tx_desc_unit_shift = 4;	/* 16 byte */
		gcp->gc_tx_buf_size = num_tx_desc;
		gcp->gc_tx_buf_limit = gcp->gc_tx_buf_size - 1;
		gcp->gc_tx_ring_size = num_tx_desc;
		gcp->gc_tx_ring_limit = gcp->gc_tx_ring_size - 1;
		gcp->gc_tx_auto_pad = B_TRUE;
		gcp->gc_tx_copy_thresh = em_tx_copy_thresh;

		gcp->gc_rx_buf_align = sizeof (uint64_t) - 1;
		gcp->gc_rx_max_frags = 1;
		gcp->gc_rx_desc_unit_shift = 4;
		gcp->gc_rx_ring_size = num_rx_desc;
		gcp->gc_rx_buf_max = num_rx_desc - 1;
		gcp->gc_rx_copy_thresh = em_rx_copy_thresh;

		gcp->gc_io_area_size = 0;

		/* map attributes */
		gcp->gc_dev_attr = em_dev_attr;
		gcp->gc_buf_attr = em_buf_attr;
		gcp->gc_desc_attr = em_buf_attr;

		/* dma attributes */
		gcp->gc_dma_attr_desc = em_dma_attr_desc;

		gcp->gc_dma_attr_txbuf = em_dma_attr_buf;
		gcp->gc_dma_attr_txbuf.dma_attr_align =
		    gcp->gc_tx_buf_align + 1;
		gcp->gc_dma_attr_txbuf.dma_attr_sgllen = gcp->gc_tx_max_frags;

		gcp->gc_dma_attr_rxbuf = em_dma_attr_buf;
		gcp->gc_dma_attr_rxbuf.dma_attr_align =
		    gcp->gc_rx_buf_align + 1;
		gcp->gc_dma_attr_rxbuf.dma_attr_sgllen = gcp->gc_rx_max_frags;

		/* time out parameters */
		gcp->gc_tx_timeout = 3*ONESEC;
		gcp->gc_tx_timeout_interval = ONESEC;

		/* MII timeout parameters */
		gcp->gc_mii_link_watch_interval = ONESEC;
		gcp->gc_mii_an_watch_interval = ONESEC/5;
		gcp->gc_mii_reset_timeout = ONESEC;	/* 1 sec */
		gcp->gc_mii_an_timeout = MII_AN_TIMEOUT*2;	/* 10 sec */
		gcp->gc_mii_an_wait = 0;
		gcp->gc_mii_linkdown_timeout = MII_LINKDOWN_TIMEOUT;

		/* setting for general PHY */
		gcp->gc_mii_addr_min =  1;
		gcp->gc_mii_an_delay =  0;
		gcp->gc_mii_linkdown_action = MII_ACTION_RESET;
		gcp->gc_mii_linkdown_timeout_action = MII_ACTION_NONE;

		/* enable phy reset within the first reset */
		gcp->gc_mii_dont_reset = B_FALSE;
		gcp->gc_mii_stop_mac_on_linkdown = B_TRUE;
#ifdef CONFIG_HW_LINK_DETECTION
		gcp->gc_mii_hw_link_detection = B_TRUE;
#else
		gcp->gc_mii_hw_link_detection = B_FALSE;
#endif
#ifdef notdef
		/* allow resetting phy */
		lp->hw.phy.reset_disable = B_FALSE;
#endif
		/* I/O methods */

		/* mac operation */
		gcp->gc_attach_chip = &em_attach_chip;
#ifdef GEM_CONFIG_JUMBO_FRAME
		gcp->gc_fixup_params = &em_fixup_params;
#endif
		gcp->gc_reset_chip = &em_reset_chip;
		gcp->gc_init_chip = &em_init_chip;
		gcp->gc_start_chip = &em_start_chip;
		gcp->gc_stop_chip = &em_stop_chip;
		gcp->gc_multicast_hash = NULL;
		gcp->gc_set_rx_filter = &em_set_rx_filter;
		gcp->gc_set_media = &em_set_media;
		gcp->gc_get_stats = &em_get_stats;
		gcp->gc_interrupt = &em_interrupt;

		/* descriptor operation */
		gcp->gc_tx_desc_write = &em_tx_desc_write;
		gcp->gc_rx_desc_write = &em_rx_desc_write;
		gcp->gc_tx_start = &em_tx_start;
		gcp->gc_rx_start = &em_rx_start;
#ifdef GEM_CONFIG_TX_HEAD_PTR
		gcp->gc_tx_desc_head = &em_tx_desc_head;
		gcp->gc_tx_desc_stat = NULL;
#else
		gcp->gc_tx_desc_stat = &em_tx_desc_stat;
#endif
		gcp->gc_rx_desc_stat = &em_rx_desc_stat;
		gcp->gc_tx_desc_init = &em_tx_desc_init;
		gcp->gc_rx_desc_init = &em_rx_desc_init;
		gcp->gc_tx_desc_clean = &em_tx_desc_init;
		gcp->gc_rx_desc_clean = &em_rx_desc_init;

		/* mii operations */
		gcp->gc_mii_probe = &em_mii_probe;
		gcp->gc_mii_init = &em_mii_init;
		gcp->gc_mii_config = &em_mii_config;
		gcp->gc_mii_sync = &em_mii_sync;
		gcp->gc_mii_read = &em_mii_read;
		gcp->gc_mii_write = &em_mii_write;
		gcp->gc_flow_control = FLOW_CONTROL_RX_PAUSE;

		/* offload and jumbo frame */
		gcp->gc_max_lso = 64 * 1024 - 1;
		gcp->gc_default_mtu = ETHERMTU;
		gcp->gc_min_mtu = ETHERMIN - sizeof (struct ether_header);

		switch (hw->mac.type) {
		case e1000_82571:
		case e1000_82572:
		case e1000_ich9lan:
		case e1000_ich10lan:
		case e1000_pch2lan:
		case e1000_pch_lpt:
		case e1000_82574:
		case e1000_82583:
		case e1000_80003es2lan: /* 9K Jumbo Frame size */
			gcp->gc_max_mtu = 9234 - sizeof (struct ether_header);
			break;

		case e1000_pchlan:
			gcp->gc_max_mtu = 4096 - sizeof (struct ether_header);
			break;

			/* Adapters that do not support jumbo frames */
		case e1000_ich8lan:
			gcp->gc_max_mtu = ETHERMTU;
			break;
		default:
			gcp->gc_max_mtu = 0x3f00 - sizeof (struct ether_header);
		}

		dp = gem_do_attach(dip, 0,
		    gcp, hw->hw_addr, &lp->reg_ha, lp, sizeof (*lp));
		kmem_free(gcp, sizeof (*gcp));

		if (dp != NULL) {
			return (DDI_SUCCESS);
		}

err_free_maps:
		e1000_free_pci_resources(lp);

err_freelp:
		kmem_free(lp, sizeof (struct em_dev));
err:
		pci_config_teardown(&conf_handle);
		return (DDI_FAILURE);
	}
	pci_config_teardown(&conf_handle);
	return (DDI_FAILURE);
}

static int
emdetach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct gem_dev		*dp;
	struct em_dev		*lp;
	timeout_id_t		old_id;
	int			ret;

	dp = GEM_GET_DEV(dip);
	lp = (struct em_dev *)dp->private;

	switch (cmd) {
	case DDI_DETACH:
		em_release_manageability(dp);
		em_release_hw_control(dp);

		if (dp->link_watcher_id) {
			do {
				untimeout(old_id = dp->link_watcher_id);
			} while (old_id != dp->link_watcher_id);
			dp->link_watcher_id = 0;
		}

		if (lp->flash_ha) {
			ddi_regs_map_free(&lp->flash_ha);
		}

		if (lp->io_ha) {
			ddi_regs_map_free(&lp->io_ha);
		}

		pci_config_teardown(&lp->conf_ha);

		return (gem_do_detach(dip));

	case DDI_SUSPEND:
		ret = gem_suspend(dip);
		if (ret == DDI_SUCCESS) {
			em_release_manageability(dp);
			em_release_hw_control(dp);
#ifdef CONFIG_NEW_EM
			em_enable_wakeup(dp);
#endif
		}
	}
	return (DDI_FAILURE);
}

/* ======================================================== */
/*
 * OS depend (loadable streams driver) routine
 */
/* ======================================================== */
#ifdef GEM_CONFIG_GLDv3
GEM_STREAM_OPS(em_ops, emattach, emdetach);
#else
static	struct module_info emminfo = {
	0,			/* mi_idnum */
	"em",			/* mi_idname */
	0,			/* mi_minpsz */
	INFPSZ,			/* mi_maxpsz */
	64*1024,		/* mi_hiwat */
	1,			/* mi_lowat */
};

static	struct qinit emrinit = {
	(int (*)()) NULL,	/* qi_putp */
	gem_rsrv,		/* qi_srvp */
	gem_open,		/* qi_qopen */
	gem_close,		/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&emminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static	struct qinit emwinit = {
	gem_wput,		/* qi_putp */
	gem_wsrv,		/* qi_srvp */
	(int (*)()) NULL,	/* qi_qopen */
	(int (*)()) NULL,	/* qi_qclose */
	(int (*)()) NULL,	/* qi_qadmin */
	&emminfo,		/* qi_minfo */
	NULL			/* qi_mstat */
};

static struct streamtab	em_info = {
	&emrinit,	/* st_rdinit */
	&emwinit,	/* st_wrinit */
	NULL,		/* st_muxrinit */
	NULL		/* st_muxwrinit */
};

static	struct cb_ops cb_em_ops = {
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
	&em_info,	/* cb_stream */
	D_MP,		/* cb_flag */
};

static	struct dev_ops em_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	gem_getinfo,	/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	emattach,	/* devo_attach */
	emdetach,	/* devo_detach */
	nodev,		/* devo_reset */
	&cb_em_ops,	/* devo_cb_ops */
	NULL,		/* devo_bus_ops */
	gem_power,	/* devo_power */
#if DEVO_REV >= 4
	gem_quiesce,	/* devo_quiesce */
#endif
};
#endif /* GEM_CONFIG_GLDv3 */
static struct modldrv modldrv = {
	&mod_driverops,	/* Type of module.  This one is a driver */
	ident,
	&em_ops,	/* driver ops */
};

static struct modlinkage modlinkage = {
	MODREV_1, &modldrv, NULL
};

/* ======================================================== */
/*
 * Loadable module support
 */
/* ======================================================== */
int
_init(void)
{
	int 	status;

	DPRINTF(2, (CE_CONT, CONS "em: _init: called"));

	status = gem_mod_init(&em_ops, "em");
	if (status != DDI_SUCCESS) {
		return (status);
	}
	status = mod_install(&modlinkage);
	if (status != DDI_SUCCESS) {
		gem_mod_fini(&em_ops);
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

	DPRINTF(2, (CE_CONT, CONS "em: _fini: called"));
	status = mod_remove(&modlinkage);
	if (status == DDI_SUCCESS) {
		gem_mod_fini(&em_ops);
	}
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
