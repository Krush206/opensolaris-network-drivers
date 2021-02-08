/*
 * dp8390.c: A general DP8390 ethernet driver core for Solaris.
 * @(#)dp8390.c	1.11 06/01/21
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

*/

/*
 * Solaris headers
 */
#include <sys/debug.h>
#include <sys/types.h>
#include <sys/kmem.h>

#include <sys/conf.h>
#include <sys/cmn_err.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/ethernet.h>
#include <sys/strsun.h>
#include <sys/errno.h>

#include <sys/modctl.h>

#include "dp8390reg.h"
#include "dp8390var.h"

#ifdef DEBUG_LEVEL
static int	dp8390_debug = DEBUG_LEVEL;
#define DPRINTF(n, args) if (dp8390_debug>(n)) cmn_err args
#else
#define DPRINTF(n, args) 
#endif

/*
 * System Macros
 */
#define	FALSE	(0)
#define	TRUE	(!(FALSE))

#define	ROUNDUP(x, y)	(((x)+(y)-1) & -(y))

#define	SET_CMD(dp, cmd)	\
	OUTB(dp, PX_CR, dp->cr_ps | dp->cr_rd | cmd)

#define	SET_RDMA_CMD(dp, rcmd)	\
	OUTB(dp, PX_CR, dp->cr_ps | (dp->cr_rd = rcmd))

#define	SET_PAGE(dp, pno)	\
	OUTB(dp,PX_CR, (dp->cr_ps = (pno << CR_PS_SHIFT)) | dp->cr_rd)

/*
 * Our confiugration
 */
#define	OUR_INTR_BITS	\
	(ISR_CNT | ISR_OVW | ISR_TXE | ISR_RXE | ISR_PTX | ISR_PRX)


static void dp8390_transmit_done(struct dp8390_dev *dp, int isr);
static int dp8390_receive(struct dp8390_dev *dp);
static void dp8390_rx_overrun(struct dp8390_dev *dp);

static int dp8390_tx_unit_start(struct dp8390_dev *, unsigned int, int);

#ifdef MODULE
extern struct mod_ops mod_miscops;

static struct modlmisc modlmisc = {
	&mod_miscops,
	"NE2000 core module v" VERSION,
};

static struct modlinkage modlinkage = {
	MODREV_1, &modlmisc, NULL
};

/*
 * _init
 */
int
_init(void)
{
	return mod_install(&modlinkage);
}

/*
 * _fini :
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

#endif /* MODULE */
#ifdef notdef
/* ============================================================== */
/*
 * gcc3 runtime routines
 */
/* ============================================================== */
int
gem_memcmp(void *s1, void *s2, int n)
{
	int	i;
	int	ret;

	ret = 0;
	for (i = 0; i < n; i++) {
		ret = (int)((uint8_t *)s1)[i] - (int)((uint8_t *)s2)[i];
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

void *
gem_memset(void *s, int c, size_t n)
{
	if ((c & 0xff) == 0) {
		bzero(s, n);
	}
	else {
		while (n--) {
			((uint8_t *)s)[n] = c;
		}
	}
	return s;
}

void *
gem_memcpy(void *s1, void *s2, size_t n)
{
	bcopy(s2, s1, n);
	return s1;
}
#endif
int
dp8390_attach(struct dp8390_dev *dp)
{
	int		i;
	int		page;
	struct txbuf	*tp;

	/*
	 * Initialize NIC state
	 */
	dp->nic_state = NIC_INACTIVE;

	/*
	 * configure local memory
	 */
	/* determin number of tx buffers */
	dp->ntxbuf = min(max(dp->ntxbuf, NTXBUF_DEFAULT), NTXBUF_MAX);

	page = dp->ram_start;
	tp = &dp->tx[0];

	/* assign tx buffer page */
	for (i = dp->ntxbuf; i--; tp++, page += TX_BUF_SIZE) {
		tp->tx_start = page;
#ifdef SANITY
		tp->tx_len   = 0;
#endif
	}

	/* construct receive ring */
	dp->rx_start = page;
	dp->rx_stop  = dp->ram_start + dp->ram_size;
	dp->rx_head  = dp->rx_start;

	/* initialize register state */
	dp->cr_rd = CR_RD_ABORT;	/* remote DMA abort */
	SET_PAGE(dp, 0);

	/* set default rdma methods */
	if (dp->get_pkt == NULL) {
		dp->get_pkt    = &dp8390_rdma_get_pkt;
	}
	if (dp->put_pkt == NULL) {
		dp->put_pkt    = &dp8390_rdma_put_pkt;
	}
	if (dp->get_header == NULL) {
		dp->get_header = &dp8390_rdma_get_header;
	}

	return 0;
}

int
dp8390_detach(struct dp8390_dev *dp)
{
	/* do nothing */
	return 0;
}

/*
 * Transmit routine
 */
static int
dp8390_tx_unit_start(struct dp8390_dev *dp, uint_t len, int addr)
{
	mutex_enter(&dp->pagelock);

	if ((INB(dp, PX_CR) & CR_TXP) != 0) {
		/*
		 * wrong hardware state: transmitter is busy
		 */
		mutex_exit(&dp->pagelock);

		cmn_err(CE_WARN,
		"%s:dp8390_tx_unit_start: transmitter is busy", dp->name);

		return -1;
	}

	/* set packet length */
	OUTB(dp, P0_TBCR0_W, len);
	OUTB(dp, P0_TBCR1_W, len >> 8);

	/* set packet start page */
	OUTB(dp, P0_TPSR_W, addr);

	/*
	 * kick the transmitter
	 * XXX -  bug fixed, see itu floppy by http://www.tools.de/solaris/
	 */
	SET_CMD(dp, CR_STA | CR_TXP);

	mutex_exit(&dp->pagelock);

	/* update last transmit time to detect tx timeout */
	dp->tx_start_time = ddi_get_lbolt();

	return 0;
}

int
dp8390_tx_start(struct dp8390_dev *dp, mblk_t *mp)
{
	struct txbuf	*txbp;
	struct txbuf	*tp;
	int		len;
	char		*name = dp->name;
	int		err = 0;

	DPRINTF(3, (CE_CONT, "!%s: dp8390_tx_start: called", name));

	mutex_enter(&dp->xmitlock);
	if (dp->nic_state != NIC_ACTIVE) {
		/*
		 * Transmitter has stopped or it is in rx overrun state.
		 */
		dp->tx_blocked = TRUE;

		if (dp->nic_state = NIC_INACTIVE) {
			cmn_err(CE_NOTE,
				"%s: dp8390_tx_start: nic is inactive", name);
		}

		mutex_exit(&dp->xmitlock);

		return 1; /* no resource */
	}

	/* get one from txcmd free list */
	if ((txbp = dp->tx_free) == NULL) {
		dp->tx_blocked = TRUE;
		mutex_exit(&dp->xmitlock);
		return 1; /* no resource */
	}
	dp->tx_free = txbp->next;

	dp->tx_blocked = FALSE;
	dp->tx_busy++;
	mutex_exit(&dp->xmitlock);

        len = msgdsize(mp);
	txbp->tx_len = max(len, ETHERMIN);

	err = (* (dp->put_pkt))(dp, mp, txbp->tx_start);

	/* ready to transmit */ 
	mutex_enter(&dp->xmitlock);

	dp->tx_busy--;

	if (dp->nic_state != NIC_ACTIVE && dp->tx_busy == 0) {
		/*
		 * The nic state has changed to inactive or overrun while
		 * we are writing the packets into the nic local memory.
		 * In this case, some one should be waiting for we have done.
		 * Wake up them.
		 */
		cv_broadcast(&dp->drain_cv);
		mutex_exit(&dp->xmitlock);

		return 1;
	}

	if (err != 0) {
		/*
		 * Failed to writing the packet into local memory
		 */
		txbp->next = dp->tx_free;
		dp->tx_free = txbp;
		mutex_exit(&dp->xmitlock);

		return 1;
	}

	/* append this packet to Tx ready queue */
	txbp->next = NULL;
	if ((tp = dp->tx_head) != NULL) {
#ifdef DEBUG
		int	n = 1;
#endif
		/*
		 * we cannot transmit it because of
		 * transmitter is working now.
		 */
		while (tp->next != NULL) {
			tp = tp->next;
			ASSERT(++n < dp->ntxbuf);
		}
		tp->next = txbp;
		dp->tx_ready_len++;
		ASSERT(dp->tx_ready_len <= dp->ntxbuf);

		mutex_exit(&dp->xmitlock);

		/* txbp was queued successfully */
		return 0;
	}

	dp->tx_head = txbp;
	ASSERT(dp->tx_ready_len == 0);
	dp->tx_ready_len = 1;

	dp8390_tx_unit_start(dp, txbp->tx_len, txbp->tx_start);

	mutex_exit(&dp->xmitlock);

	return 0;
}

/*
 * Interrupt services
 */
void
dp8390_rx_overrun_continue(struct dp8390_dev *dp)
{
	int	ret;
#if 0
	mutex_enter(&dp->intrlock);
#endif
	ASSERT(dp->intr_busy == TRUE);
	ASSERT(dp->nic_state == NIC_RX_OVERRUN);

	/* do rest of phase 2 of rx overrun */
	dp8390_rx_overrun(dp);

	/* recover IMR before returning from interrupt */
	ASSERT(dp->imr_shadow == 0);
	mutex_enter(&dp->pagelock);
	dp->imr_shadow = dp->imr;
	OUTB(dp, P0_IMR_W, dp->imr);
	mutex_exit(&dp->pagelock);

	ASSERT(dp->nic_state == NIC_ACTIVE);
	dp->intr_busy = FALSE;
#if 0
	mutex_exit(&dp->intrlock);
#endif
}

static void
dp8390_transmit_done(struct dp8390_dev *dp, int isr)
{
	int		tsr;
	int		rsr;
	int		ncr;
	int		cmd;
	int		tx_was_aborted;
	int		i;
	struct txbuf	*txbp;

	ASSERT(mutex_owned(&dp->xmitlock));
	ASSERT(dp->nic_state == NIC_ACTIVE);

	if ((txbp = dp->tx_head) == NULL) {
		/* no transmitted buffer */
		return;
	}

	/* read registers */
	mutex_enter(&dp->pagelock); /* XXX */
	cmd = INB(dp, PX_CR);
	tsr = INB(dp, P0_TSR_R);
	ncr = INB(dp, P0_NCR_R);
	rsr = INB(dp, P0_RSR_R);
	mutex_exit(&dp->pagelock); /* XXX */

	if ((tsr & TSR_PTX) != 0) {
		/*
		 * Transmited successfully
		 */
		if ((tsr & TSR_COL) != 0) {

			/* collision occured */
			if (ncr == 0) {
				/* the HW does not have ncr */
				ncr = 1;
			}

			dp->stat.collisions += ncr;
			if (ncr == 1) {
				dp->stat.first_coll++;
			}
			else {
				dp->stat.multi_coll++;
			}
		}
		else if ((tsr & TSR_ND) == 0) {
			/* deferred */
			dp->stat.defer++;
		}
	}
	else if ((tsr & ~(TSR_PTX | TSR_COL | TSR_ND)) != 0) {
		/*
		 * failed to transmit
		 */
		dp->stat.errxmt++;
		if ((tsr & TSR_ABT) != 0) {
			dp->stat.excoll++;
			dp->stat.collisions += 16;
		}
		if ((tsr & TSR_CRS) != 0) {
			dp->stat.nocarrier++;
		}
		if ((tsr & TSR_FU) != 0) {
			dp->stat.underflow++;
		}
		if ((tsr & TSR_OWC) != 0) {
			dp->stat.xmtlatecoll++;
		}
	}
	else  {
		/*
		 * HW error ?
		 */
		cmn_err(CE_NOTE, "%s: unexpected TX-done interrupt. "
			"nic_state:0x%x, tx_start_time:%d"
			" cmd:%02x isr:%b tsr:%b",
			dp->name, dp->nic_state,
			dp->tx_start_time, cmd, isr, ISR_BITS, tsr, TSR_BITS);
		return;
	}

	/*
	 * Remove finished txcmd from ready queue
	 */
	dp->tx_head = txbp->next;
	dp->tx_ready_len--;
	ASSERT(dp->tx_ready_len >= 0);

	/* link it to tx free list */
	txbp->next = dp->tx_free;
	dp->tx_free = txbp;

	/*
	 * If the next tx buffer is ready to transmit, kick it now.
	 */
	if ((txbp = dp->tx_head) != NULL) {
		/* found, send it now */
		dp8390_tx_unit_start(dp, txbp->tx_len, txbp->tx_start);
	}
}

#ifdef OVERRUN_TEST
static int	dp8390_rx_overrun_test = 0;
#endif
static int
dp8390_receive(struct dp8390_dev *dp)
{
	uint8_t		rx_tail;
	int		rest;
	int		pos;
	int		len;
	struct rhead	hdr;
	mblk_t		*mp;
	char		*name = dp->name;

	DPRINTF(3, (CE_CONT, "!%s: dp8390_receive: called", name));

	ASSERT(mutex_owned(&dp->intrlock));

	/* end of unread packets */
	mutex_enter(&dp->pagelock);
	SET_PAGE(dp, 1);
	rx_tail = INB(dp, P1_CURR);
	SET_PAGE(dp, 0);
	mutex_exit(&dp->pagelock);

	if ((rest = rx_tail - dp->rx_head) < 0) {
		rest += dp->rx_stop - dp->rx_start;
	}

	while (rest > 0) {
		mp = NULL;

		/* reply to interrupt */
		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_ISR, ISR_PRX | ISR_RXE);
		mutex_exit(&dp->pagelock);

		(dp->get_header)(dp, &hdr, dp->rx_head);

		DPRINTF(3, (CE_CONT,
		"!%s: dp8390_receive: hdr: rsr:%b next:0x%02x size:0x%04x",
			name, hdr.rh_rsr, RSR_BITS,
			hdr.rh_next, hdr.rh_size));

		len = hdr.rh_size - sizeof(struct rhead);

		if (len < ETHERMIN) {
			/* too short */
			cmn_err(CE_NOTE,
		"%s: dp8390_receive: runt packet: len:%d rsr:%b next:0x%02x",
				name, len,
				hdr.rh_rsr, RSR_BITS, hdr.rh_next);
			dp->stat.errrcv++;
			dp->stat.runt++;
			goto next;
		}

 		if (len > ETHERMAX + ETHERFCSL) {
			/* too long */
			cmn_err(CE_NOTE,
		"%s: dp8390_receive: too long: len:%d rsr:%b next:0x%02x",
				name, len,
				hdr.rh_rsr, RSR_BITS, hdr.rh_next);

			dp->stat.errrcv++;
			dp->stat.frame_too_long++;
			goto next;
		}

		if ((hdr.rh_rsr & (RSR_FO | RSR_FAE | RSR_CRC | RSR_PRX))
								!= RSR_PRX) {
			cmn_err(CE_NOTE,
		"%s: dp8390_receive: rx error: len:%d rsr:%b nxpg:0x%02x",
				name, len,
				hdr.rh_rsr, RSR_BITS, hdr.rh_next);

			dp->stat.errrcv++;
			if ((hdr.rh_rsr & RSR_FO) != 0) {
				dp->stat.overflow++;
			}
			goto next;
		}

		/* the packet was received without error */
		pos =  DP8390_PTOB(dp->rx_head) + sizeof(struct rhead);
		mp = (dp->get_pkt)(dp, len, pos);
		if (mp == NULL) {
			/*
			 * Failed to allocate msg block
			 */
			dp->stat.norcvbuf++;
			goto next;
		}

next:
		/* check if the rx ring buffer is not corrupted */
		if (hdr.rh_next >= dp->rx_stop || hdr.rh_next < dp->rx_start) {
			cmn_err(CE_NOTE,
				"%s: next frame inconsistency, 0x%02x\n",
				name, dp->rx_head);
			return -1;
		}

		if ((len = hdr.rh_next - dp->rx_head) < 0) {
			len += dp->rx_stop - dp->rx_start;
		}
		rest -= len;
		if (rest < 0 || len == 0) {
			cmn_err(CE_WARN, "%s: fatal error: rx ring corrupted",
				name);
			return -1;
		}

		/* Update software current page */
		dp->rx_head = hdr.rh_next;

		/* Update boundary page */
		pos = dp->rx_head - 1;
		if (pos < dp->rx_start) {
			pos = dp->rx_stop - 1;
		}

		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_BNRY, pos);
		if (rest == 0) {
			SET_PAGE(dp, 1);
			rx_tail = INB(dp, P1_CURR);
			SET_PAGE(dp, 0);
			if ((rest = rx_tail - dp->rx_head) < 0) {
				rest += dp->rx_stop - dp->rx_start;
			}
		}
		mutex_exit(&dp->pagelock);

		if (mp != NULL) {
			mutex_exit(&dp->intrlock);
			dp->sendup(dp, mp);
			mutex_enter(&dp->intrlock);
		}
	}

#ifdef OVERRUN_TEST
	if (++dp8390_rx_overrun_test % 10000 == 0) {
		dp->imr &= ~(ISR_PRX | ISR_RXE);
	}
#endif
	return 0;
}

static void
dp8390_rx_overrun(struct dp8390_dev *dp)
{
	int		tx_unfinished = FALSE;
	struct txbuf	*txbp;
	char		*name = dp->name;

	ASSERT(mutex_owned(&dp->intrlock));

#ifdef OVERRUN_TEST
	dp->imr |= (ISR_PRX | ISR_RXE);
#endif
	if (dp->nic_state == NIC_RX_OVERRUN) {
		goto phase2;
	}

	cmn_err(CE_NOTE, "!%s: dp8390_rx_overrun", name);

	/* step1: stop downstream thread */
	ASSERT(dp->nic_state == NIC_ACTIVE);

	mutex_enter(&dp->xmitlock); /* to change ei_nic_state */
	dp->nic_state = NIC_RX_OVERRUN;
	while (dp->tx_busy > 0) {
		cv_wait(&dp->drain_cv, &dp->xmitlock);
	}
	mutex_exit(&dp->xmitlock);

	ASSERT(dp->cr_rd == CR_RD_ABORT);
    
	/* Step2: stop transmitter by issuing stop command */
	mutex_enter(&dp->pagelock);
	SET_CMD(dp, CR_STP);
	mutex_exit(&dp->pagelock);
    
	dp->stat.overflow++;

	/* Step3: wait for TX completes */
	if (dp->speed100) {
		/* do not suspend rx overrun routine */
		drv_usecwait(160);
	}
	else /* SPEED_10M */ {
		if ((void *)dp->rx_overrun_continue == NULL) {
			/* do not suspend rx overrun routine */
			drv_usecwait(1600);
		} else {
			/* suspend rx overrun routine */
			timeout(
			    (void (*)(void *))dp->rx_overrun_continue,
			    (void *)dp, drv_usectohz(1600));

			return;
		}
	}

phase2:
	/* Step4: Reset RDMA engine by clearing remote byte counter */
	mutex_enter(&dp->pagelock);
	OUTB(dp, P0_RBCR0_W, 0);
	OUTB(dp, P0_RBCR1_W, 0);

	/* Step5: Restart NIC and process received packets */
	txbp = dp->tx_head;
	if (txbp != NULL && 
	   (INB(dp, P0_ISR) & (ISR_PTX | ISR_TXE)) == 0) {
		/*
		 * TX was not finished. So we must send
		 * this packet again.
		 */
		tx_unfinished = TRUE;
	}

	OUTB(dp, P0_TCR_W, TCR_LB_NIC);
	SET_CMD(dp, CR_STA);

#ifdef DEBUG
	if (txbp != NULL && !tx_unfinished) {
		ASSERT((INB(dp, P0_ISR) & (ISR_PTX | ISR_TXE)) != 0);
	}
#endif
	mutex_exit(&dp->pagelock);

	ASSERT(dp->nic_state == NIC_RX_OVERRUN);
	(void)dp8390_receive(dp);

	/* Step6: Reply to Rx Overrun interrupt */
	mutex_enter(&dp->xmitlock);
	mutex_enter(&dp->pagelock);
	OUTB(dp, P0_ISR, ISR_OVW);

	/* Step6: Enable transmitter again */
	OUTB(dp, P0_TCR_W, TCR_LB_NORMAL); 
	if (tx_unfinished) {
		/* XXX: keep start bit in command register */
		SET_CMD(dp, CR_STA | CR_TXP);
	}
	mutex_exit(&dp->pagelock);

	/* Step7: Restore previous nic state */
	dp->nic_state = NIC_ACTIVE;
	mutex_exit(&dp->xmitlock);
}

static int	dp_multi_intr_cnt = 0;

int
dp8390_interrupt(struct dp8390_dev *dp)
{
	uint32_t	isr;
	uint32_t	imr;
	uint32_t	isr_hw;
	int		restart_tx = 0;
	char		*name = dp->name;

	ASSERT(mutex_owned(&dp->intrlock));

	/*
	 * If we are in rx overrun state now, don't touch any registers
	 */
	if (dp->nic_state == NIC_RX_OVERRUN) {
		return 0;
	}

	/*
	 * Check if we are interrupted.
	 */
	mutex_enter(&dp->pagelock);
	isr_hw = INB(dp, P0_ISR);
	mutex_exit(&dp->pagelock);

	isr = isr_hw & dp->imr;

	DPRINTF(4, (CE_CONT, "!%s: dp8390_interrupt: isr_hw:%b isr&imr:%b",
			name, isr_hw, ISR_BITS, isr, ISR_BITS));

	if (isr == 0) {
		/* not for us */
		return 0;
	}

	DPRINTF(2, (CE_CONT, "!%s: dp8390_interrupt: isr_hw:%b isr&imr:%b",
			name, isr_hw, ISR_BITS, isr, ISR_BITS));
#ifdef DEBUG_LEVEL
	if (dp->intr_busy) {
		/* never happen */
		cmn_err(CE_WARN, "%s: dp8390_interrupt: multiple invocation.",
			dp->name);
		dp_multi_intr_cnt++;

		return 0;
	}
#endif
	ASSERT(!dp->intr_busy);
	dp->intr_busy = TRUE;
	dp->stat.intr++;

	/*
	 * Start interrupt processing
	 */

	/* inactivate interrupt line by inhibiting all interrupts */
	mutex_enter(&dp->pagelock);
	if (dp->imr_shadow != 0) {
		OUTB(dp, P0_IMR_W, dp->imr_shadow = 0);
	}
	mutex_exit(&dp->pagelock);

	if (dp->nic_state == NIC_INACTIVE) {

		/*
		 * Now closing. clear obsolute interrupts.
		 */
		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_ISR, isr_hw);
		mutex_exit(&dp->pagelock);

		cmn_err(CE_WARN, "%s: interrupted, but card is inactive"
			" isr&imr: %b, imr: %b",
			name, isr, ISR_BITS, dp->imr, ISR_BITS);
		goto x;
	}

	if ((isr & ISR_OVW) != 0) {
		/*
		 * RX buffer ring overrun
		 */
		dp8390_rx_overrun(dp);

		if (dp->nic_state == NIC_RX_OVERRUN) {
			/*
			 * XXX - do nothing 
			 * don't enable interrupts until we will have processed
			 * rx overrun.
			 *
			 * We don't reset intr_busy.
			 */
			return isr;
		}
		goto x;
	}

	if ((isr & (ISR_PRX | ISR_RXE)) != 0) {
		/*
		 * Receive notify w/ or w/o error
		 */
		DPRINTF(2, (CE_CONT,
			"!%s: dp8390_interrupt: calling dp8390_receive", name));
		if (dp8390_receive(dp) < 0) {
			isr |= INTR_FATAL_ERR;
		}
	}

	if ((isr & (ISR_PTX | ISR_TXE)) != 0) {
		/*
		 * Transmit done
		 */
		mutex_enter(&dp->xmitlock);

		/* reply to interrupt */
		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_ISR, ISR_PTX | ISR_TXE);
		mutex_exit(&dp->pagelock);

		dp8390_transmit_done(dp, isr);

		if (dp->tx_blocked) {
			/* check low water mark */
			restart_tx = (dp->tx_ready_len <= 1);
		}
		mutex_exit(&dp->xmitlock);
	}

	if ((isr & ISR_CNT) != 0) {
		dp8390_get_stats(dp);

		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_ISR, ISR_CNT);
		mutex_exit(&dp->pagelock);
	}

	if ((isr & ISR_RDC) != 0) {
		cmn_err(CE_WARN,
		"%s: dp8390_interrupt: RDC intr occured. isr: %b",
			name, isr, ISR_BITS);

		mutex_enter(&dp->pagelock);
		OUTB(dp, P0_ISR, ISR_RDC);
		mutex_exit(&dp->pagelock);
	}

x:
	/* restore interrupt mask */
	if (dp->nic_state == NIC_INACTIVE) {
		/* do not enable interrup */
		ASSERT(dp->imr_shadow == 0);

		/* someone may waiting for me */
		cv_broadcast(&dp->drain_cv);
	}
	else if (dp->nic_state == NIC_ACTIVE) {
		if (restart_tx) {
			isr |= INTR_RESTART_TX;
		}
		/*
		 * recover IMR before returning from interrupt
		 */
		mutex_enter(&dp->pagelock);
		dp->imr_shadow = dp->imr;
		OUTB(dp, P0_IMR_W, dp->imr);
		mutex_exit(&dp->pagelock);
	}

	dp->intr_busy = FALSE;

	return isr;
}

/*
 * Misc HW manupilations
 */
void
dp8390_init_chip(struct dp8390_dev *dp)
{
	int		i;
	struct txbuf	*tp;

	ASSERT(mutex_owned(&dp->pagelock));
	ASSERT(dp->nic_state == NIC_INACTIVE);

	SET_CMD(dp, CR_STP);

	OUTB(dp, P0_DCR_W, dp->dcr_wts | DCR_FT_8 | DCR_LS_NORMAL);

	/* reset remote dma engine */
	OUTB(dp, P0_RBCR0_W, 0);
	OUTB(dp, P0_RBCR1_W, 0);

	/* disable tx and rx */
	OUTB(dp, P0_RCR_W, RCR_MON);
	OUTB(dp, P0_TCR_W, TCR_LB_NIC);

#ifdef SANITY
	/* set tx buffer: do nothing */
	OUTB(dp, P0_TPSR_W, dp->ram_start);
#endif

	/* construct tx buffer free list */
	tp = &dp->tx[0];
	dp->tx_free = tp;
	for (i = dp->ntxbuf; i--; tp++) {
		tp->next = &tp[1];
	}
	dp->tx[dp->ntxbuf-1].next = NULL;
	dp->tx_ready_len = 0;
	dp->tx_head = NULL;

	/* set receive ring up */
	OUTB(dp, P0_PSTART_W, dp->rx_start);
	OUTB(dp, P0_PSTOP_W,  dp->rx_stop);

	dp->rx_head = dp->rx_start;
	OUTB(dp, P0_BNRY,     dp->rx_stop - 1);
	SET_PAGE(dp, 1);
	OUTB(dp, P1_CURR, dp->rx_head);
	SET_PAGE(dp, 0);

	/* inhibit all interrupts */
	dp->imr        = 0;
	dp->imr_shadow = 0;
#ifdef SANITY
	OUTB(dp, P0_ISR, 0xff);
#endif
	OUTB(dp, P0_IMR_W, dp->imr);

	dp->rcr = 0;
}

/*
 * Set MAC address
 */
void
dp8390_set_mac_addr(struct dp8390_dev *dp)
{
	int	i;
	uint8_t	*mac;

	ASSERT(mutex_owned(&dp->pagelock));

	/* Stop receiver */
	if (dp->nic_state != NIC_INACTIVE) {
		/* tmporary rx off */
		OUTB(dp, P0_RCR_W, RCR_MON);
	}

	SET_PAGE(dp, 1);

	mac = dp->curraddr.ether_addr_octet;
	for (i = 0; i < ETHERADDRL; i++) {
		OUTB(dp, P1_PAR + i, mac[i]);
#ifdef DEBUG
		if (INB(dp, P1_PAR + i) != mac[i]) {
			cmn_err(CE_WARN,
			"%s: set_mac_addr: failed to write PAR reg at %d",
				dp->name, i);
		}
#endif
	}
	SET_PAGE(dp, 0);

	/* restore rx mode */
	if (dp->nic_state != NIC_INACTIVE) {
		OUTB(dp, P0_RCR_W, dp->rcr);
	}
}

void
dp8390_start_chip(struct dp8390_dev *dp)
{
	ASSERT(mutex_owned(&dp->pagelock));

	dp->nic_state    = NIC_ACTIVE;
	dp->tx_busy      = 0;
#if 0
	dp->rxover_state = 0;
#endif
	dp->intr_busy    = FALSE;

	/* clear pended interrupts */
	OUTB(dp, P0_ISR, 0xff);

	/* enable interrupts */
	dp->imr = dp->imr_shadow = OUR_INTR_BITS;
	OUTB(dp, P0_IMR_W, dp->imr);

	/* start chip */
	SET_CMD(dp, CR_STA);

	/* enable tx and rx */
	OUTB(dp, P0_TCR_W, TCR_LB_NORMAL);
#ifdef notdef
	dp8390_set_rx_filter(dp);
#endif
	return;
}

void
dp8390_stop_chip(struct dp8390_dev *dp)
{
	int	ret;

	ASSERT(&dp->intrlock);

	mutex_enter(&dp->xmitlock);
	dp->nic_state = NIC_INACTIVE;
	/* TODO: should not clear isr, but how about imr ? */
	mutex_exit(&dp->xmitlock);

	/* wait for RX stopped */
	while (dp->intr_busy) {
		ret = cv_timedwait(&dp->drain_cv,
				&dp->intrlock,
				drv_usectohz(10*1000*1000) + ddi_get_lbolt());
		if (ret < 0) {
			cmn_err(CE_PANIC,
				"%s: dp8390_stop_chip: rx stop timeout.",
				dp->name);
			/* Not Reached */
		}
	}

	/* wait for TX stopped */
	mutex_enter(&dp->xmitlock);
	while (dp->tx_busy > 0) {
		ret = cv_timedwait(&dp->drain_cv,
				&dp->xmitlock,
				drv_usectohz(10*1000*1000) + ddi_get_lbolt());
		if (ret < 0) {
			cmn_err(CE_PANIC,
				"%s: dp8390_stop_chip: tx stop timeout.",
				dp->name);
			/* Not Reached */
		}
	}
	mutex_exit(&dp->xmitlock);

	mutex_enter(&dp->pagelock);

	/* stop chip */
	ASSERT(dp->cr_rd == CR_RD_ABORT);
	SET_CMD(dp, CR_STP);

	/* mask and clear all interrupts */
	dp->imr        = 0;
	dp->imr_shadow = 0;
	OUTB(dp, P0_IMR_W, dp->imr);

	/* stop Rx and Tx */
	OUTB(dp, P0_RCR_W, RCR_MON);
	OUTB(dp, P0_TCR_W, TCR_LB_NIC);

	mutex_exit(&dp->pagelock);
}

void
dp8390_get_stats(struct dp8390_dev *dp)
{
	uint8_t	frame;
	uint8_t	crc;
	uint8_t	missed;

	ASSERT(&dp->intrlock); /* for dp->stat */

	mutex_enter(&dp->pagelock);
	frame  = INB(dp, P0_CNTR0_R);
	crc    = INB(dp, P0_CNTR1_R);
	missed = INB(dp, P0_CNTR2_R);
	mutex_exit(&dp->pagelock);

	dp->stat.frame  += frame;
	dp->stat.crc    += crc;
	dp->stat.missed += missed;
	dp->stat.errrcv += frame + crc + missed;
    
	return;
}

#define	CRC32_POLY_BE	0x04c11db7

uint32_t
dp8390_ether_crc(uint8_t *addr)
{
	int		idx;
	int		bit;
	u_int		data;
	uint32_t	crc;

	crc = 0xffffffff;
	for (idx = 0; idx < ETHERADDRL; idx++) {
                for (data = *addr++, bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc << 1)
			    ^ ((((crc >> 31) ^ data) & 1) ? CRC32_POLY_BE : 0);
		}
	}
	return crc;
}

void
dp8390_set_rx_filter(struct dp8390_dev *dp)
{
	int	i, h;
	uint8_t	rcr;
	uint8_t	mhash[8];

	ASSERT(mutex_owned(&dp->pagelock));

	for (i = 0; i < 8; i++) {
		mhash[i] = 0;
	}

	rcr = RCR_AB | RCR_AR | RCR_SEP; /* accept broadcast */

	SET_PAGE(dp, 0);
	OUTB(dp, P0_RCR_W, rcr);

  	if ((dp->rxmode & RXMODE_PROMISC) != 0) {
		rcr |= RCR_PRO | RCR_AM; /* promicuous and multicast */
		for (i = 0; i < 8; i++) {
			mhash[i] = 0xff;
		}
	}
	else if ((dp->rxmode & RXMODE_ALLMULTI) != 0 || dp->mc_count > 32) {
		rcr |= RCR_AM; /* multicast */
		for (i = 0; i < 8; i++) {
			mhash[i] = 0xff;
		}
	}
	else if (dp->mc_count > 0) {
		rcr |= RCR_AM; /* multicast */
		for (i = 0; i < dp->mc_count; i++) {
			h = dp->mc_list[i].hash >> (32 - 6);
			mhash[h / 8] |= 1 << (h % 8);
		}
	}
	dp->rcr = rcr;

	SET_PAGE(dp, 1);
	for (i = 0; i < 8; i++) {
		OUTB(dp, P1_MAR + i, mhash[i]);
	}

	SET_PAGE(dp, 0);
	OUTB(dp, P0_RCR_W, rcr);

	return;
}


/*====================================================================*/
/*
 * RDMA support
 */
/*====================================================================*/
void
dp8390_rdma_setup(struct dp8390_dev *dp, int cmd, uint_t nbytes, uint_t addr)
{
	DPRINTF(2, (CE_CONT, "!%s:%s: nbytes:0x%02x addr:0x%02x",
			dp->name, __func__, nbytes, addr));

	ASSERT(mutex_owned(&dp->pagelock));
	ASSERT(cmd == CR_RD_RRD || cmd == CR_RD_RWR);

	/* set dma count */
	OUTB(dp, P0_RBCR0_W, nbytes);
	OUTB(dp, P0_RBCR1_W, nbytes >> 8);

	/* set start address of local memory */
	OUTB(dp, P0_RSAR0_W, addr);
	OUTB(dp, P0_RSAR1_W, addr >> 8);

	/* start rdma */
	SET_RDMA_CMD(dp, cmd);

	return;
}

int
dp8390_rdma_finish(struct dp8390_dev *dp)
{
	int	i;

	ASSERT(mutex_owned(&dp->pagelock));

	i = 100;
	while ((INB(dp, P0_ISR) & ISR_RDC) == 0) {
		if (i == 0) {
			/* time out */
			cmn_err(CE_WARN,
			"%s: %s: timeout: CRDA:%02x %02x, CLDA:%02x %02x",
				dp->name, __func__,
				INB(dp, P0_CRDA0_R), INB(dp, P0_CRDA0_R),
				INB(dp, P0_CLDA0_R), INB(dp, P0_CLDA0_R));
			return -1;
		}
		drv_usecwait(10);
		i--;
	}

	/* reply to  interrupt */
        OUTB(dp, P0_ISR, ISR_RDC);

	/* stop rdma */
	SET_RDMA_CMD(dp, CR_RD_ABORT);

	return 0;
}

void
dp8390_rdma_abort(struct dp8390_dev *dp)
{
	ASSERT(mutex_owned(&dp->pagelock));

	/* force to stop rdma */
	SET_RDMA_CMD(dp, CR_RD_ABORT);

	/* reset RDMA engine by writing zero to rbcnt registers */
	OUTB(dp, P0_RBCR0_W, 0);
	OUTB(dp, P0_RBCR1_W, 0);

	/* reply to interrupt */
        OUTB(dp, P0_ISR, ISR_RDC);

	return;
}


#define	PORT_DATA	0x10

#define	XFER_IN(dp, unit, port, bp, rest)	\
	switch (unit) {	\
	case 4:	\
		INSL(dp, port, bp, (rest)>>2); \
		break;	\
	case 2:	\
		INSW(dp, port, bp, (rest)>>1); \
		break;	\
	case 1:	\
		INSB(dp, port, bp, (rest)); \
		break;	\
	default:	\
		cmn_err(CE_PANIC, "%s: illegal bad xfer unit:%d", \
			dp->name, unit);	\
		break;	\
	}

#ifdef __amd64
#undef OUTSB
#undef OUTSW
/* #undef OUTSL */

#define	OUTSB(dp, p, b, c)	\
{	\
	int	i;	\
	for (i = 0; i < (c); i++) { \
		OUTB(dp, p, ((uint8_t *)(b))[i]); \
	} \
}

#define	OUTSW(dp, p, b, c)	\
{	\
	int	i;	\
	for (i = 0; i < (c); i++) { \
		OUTW(dp, p, ((uint16_t *)(b))[i]); \
	} \
}
/*
#define	OUTSL(dp, p, b, c)	\
{	\
	int	i;	\
	for (i = 0; i < (c); i++) { \
		OUTL(dp, p, ((uint32_t *)(b))[i]); \
	} \
}
*/
#endif

#define	XFER_OUT(dp, unit, port, bp, rest)	\
	switch (unit) {	\
	case 4:	\
		OUTSL(dp, port, bp, (rest)>>2); \
		break;	\
	case 2:	\
		OUTSW(dp, port, bp, (rest)>>1);	\
		break;	\
	case 1:	\
		OUTSB(dp, port, bp, (rest)); \
		break;	\
	default:	\
		cmn_err(CE_PANIC, "%s: illegal bad xfer unit:%d", \
			dp->name, unit);	\
		break;	\
	}

static int8_t	cvt_to_best_width[/* xfer width ability bits*/] = {
	0,	/* 0: illegal */
	1,	/* 1: 8BIT */
	2,	/* 2: 16BIT */
	2,	/* 3: 16BIT|8BIT */
	4,	/* 4: 32BIT */
	4,	/* 5: 32BIT|8BIT */
	4,	/* 6: 32BIT|16BIT */
	4,	/* 7: 32BIT|16BIT|8BIT */
};

int
dp8390_rdma_get_header(struct dp8390_dev *dp, struct rhead *hdr, int pos)
{
	int	i;
	int	error = 0;
	uint8_t	buf[sizeof(struct rhead)];

	/* acquire RDMA counter */
	RDMA_LOCK(dp);

	mutex_enter(&dp->pagelock);
	dp8390_rdma_setup(dp, CR_RD_RRD,
				sizeof(struct rhead), DP8390_PTOB(pos));
	mutex_exit(&dp->pagelock);

	XFER_IN(dp, cvt_to_best_width[dp->xfer_width],
		PORT_DATA, buf, sizeof(struct rhead));

	mutex_enter(&dp->pagelock);
	if (dp8390_rdma_finish(dp) != 0) {
		cmn_err(CE_WARN,
			"%s: dp8390_rdma_get_header: failed to stop rdma",
			dp->name);
		dp8390_rdma_abort(dp);
		error = 1;
	}
	mutex_exit(&dp->pagelock);

	hdr->rh_rsr  = buf[0];
	hdr->rh_next = buf[1];
	hdr->rh_size = (buf[3] << 8) | buf[2];

	/* release RDMA counter */
	RDMA_UNLOCK(dp);

	return error;
}

mblk_t *
dp8390_rdma_get_pkt(struct dp8390_dev *dp, int nbytes, int ring_offset)
{
	int		error = 0;
	int		retries = 0;
	mblk_t		*mp;
	int		best_width;
	struct ether_header	*ehp;

#ifdef TRACEx
	DPRINTF(3, (CE_CONT, "!%s: dp8390_rdma_get_pkt: called: nbytes:%d",
			dp->name, nbytes));
#endif 
	/*
	 * Allocate mblk
	 */
	if ((mp = allocb(ROUNDUP(nbytes, sizeof(uint32_t)), BPRI_LO)) == NULL) {
		dp->stat.errrcv++;
		dp->stat.norcvbuf++;
		DPRINTF(1, (CE_CONT, "!%s: dp8390_rdma_get_pkt: allocb fail",
				dp->name));
		return NULL;
	}
	/* fix data block size */
	mp->b_wptr = mp->b_rptr + nbytes;

	/*
	 * Round up the byte count to adapt best aligned transfer unit.
	 */
	best_width = cvt_to_best_width[dp->xfer_width];
	nbytes = ROUNDUP(nbytes, best_width);

	/* acquire RDMA counter */
	RDMA_LOCK(dp);

	mutex_enter(&dp->pagelock);
	dp8390_rdma_setup(dp, CR_RD_RRD, nbytes, ring_offset);
	mutex_exit(&dp->pagelock);

	XFER_IN(dp, best_width, PORT_DATA, mp->b_rptr, nbytes);

	/* wait for DMA complete */
	mutex_enter(&dp->pagelock);
	if (dp8390_rdma_finish(dp) != 0) {
		/* time out */
		cmn_err(CE_WARN, "%s: dp8390_rdma_get_pkt: RDMA do not stop",
			dp->name);
		dp8390_rdma_abort(dp);
		error = 1;
	}
	mutex_exit(&dp->pagelock);

	/* release RDMA */
	RDMA_UNLOCK(dp);

	if (error) {
		freemsg(mp);
		mp = NULL;
	}
	return mp;
}

int
dp8390_rdma_put_pkt(struct dp8390_dev *dp, mblk_t *mp, int start_page)
{
	mblk_t		*mp0;
	int		error;
	size_t		pkt_size;
	size_t		nbytes;
	long		best_align_mask;
	long		best_width;
	int		tx_copy = FALSE;
#if defined (DEBUG) || defined(DEBUG_LEVEL)
	int		n;
	size_t		sent;
#endif

	pkt_size = msgdsize(mp);

#if DEBUG_LEVEL > 2
	cmn_err(CE_CONT,
		"%s: dp8390_rdma_put_pkt: size:%d, start:0x%x",
		dp->name, pkt_size, start_page);

	for (n = 0, mp0 = mp; mp0; mp0 = mp0->b_cont, n++) {
		cmn_err(CE_CONT, "  [%d] mp:0x%p, len %d\n",
			n, mp0->b_rptr, mp0->b_wptr - mp0->b_rptr);
	}
#endif
	best_width = cvt_to_best_width[dp->xfer_width];
	ASSERT(best_width > 0 && (best_width & (best_width - 1)) == 0);
	best_align_mask = best_width - 1;

	/*
	 * Check buffer alignment and length.
	 */
	nbytes = pkt_size;
	if (best_align_mask) {
		long	required_align;
		int	extent;

		/* check if the message block is aligned */
		required_align = 0;
		for (mp0 = mp; mp0->b_cont; mp0 = mp0->b_cont) {
			required_align |= mp0->b_wptr - mp0->b_rptr;
#ifdef sparc
			required_align |= (long)mp0->b_rptr;
#endif
		}

		/* for the last message block */
#ifdef sparc
		required_align |= (long)mp0->b_rptr;
#endif
		tx_copy = (required_align & best_align_mask) != 0;

		/* extend packet size to the next best_aligned boudnary */
		nbytes = ROUNDUP(pkt_size, best_width);
		extent = nbytes - pkt_size;
		if (ROUNDUP((long)mp0->b_wptr, PAGESIZE) - (long)mp0->b_wptr
		    >= extent) {
			mp0->b_wptr += extent;
		}
	}

	/* Acquire RDMA counter and start RDMA */
	RDMA_LOCK(dp);

	mutex_enter(&dp->pagelock);
	dp8390_rdma_setup(dp, CR_RD_RWR, nbytes, DP8390_PTOB(start_page));
	mutex_exit(&dp->pagelock);

	/* Transfer data from memory to NIC */
#ifdef DEBUG
	sent = 0;
#endif
	if (tx_copy) {
		caddr_t	bp = (caddr_t)dp->txbuf;
		size_t	rest;

		/*
		 * Copy into the temporary tx buffer
		 * because the mblk is not aligned.
		 */
		for (mp0 = mp; mp0; mp0 = mp0->b_cont) {
			rest = mp0->b_wptr - mp0->b_rptr;
			bcopy(mp0->b_rptr, bp, rest);
			bp += rest;
		}

		/* padding: clear to the next best aligned boundary */
		if ((rest = nbytes - pkt_size) > 0) {
			bzero(bp, rest);
		}

		/* transfer the temporary buffer into the NIC memory */
		DPRINTF(2, (CE_CONT,
	"%s: %s tx_copy best_width:%d port_data:0x%x buf:0x%p, len:0x%x",
		dp->name, __func__, best_width, PORT_DATA, dp->txbuf, nbytes));
		XFER_OUT(dp, best_width, PORT_DATA, dp->txbuf, nbytes);
#ifdef DEBUG
		sent = nbytes;
#endif
	}
	else {
		for (mp0 = mp; mp0; mp0 = mp0->b_cont) {
			size_t	rest;
			size_t	len;

			DPRINTF(2, (CE_CONT,
				"%s: %s direct best_width:%d port_data:0x%x",
				dp->name, __func__, best_width, PORT_DATA));

			/* transfer each fragment of the mblk into the NIC */
			rest  = mp0->b_wptr - mp0->b_rptr;

			if ((len = rest & ~best_align_mask) > 0) {
				XFER_OUT(dp, best_width, PORT_DATA,
					mp0->b_rptr, len);
				rest -= len;
#ifdef DEBUG
				sent += len;
#endif
			}

			if (rest > 0) {
				/*
				 * The fragment length is not best aligned.
				 * This happens only for the last fragment.
				 */
				uint32_t	tmpbuf;

				ASSERT(mp0->b_cont == NULL);

				/*
				 * Copy the rest of the frabmet of the mblk
				 * into temporary buffer to align tranfer size. 
				 */
				bcopy(mp0->b_rptr + len, &tmpbuf, rest);

				/*
				 * Extend transfer size to the next best
				 * aligned boundary.
				 */
				XFER_OUT(dp, best_width, PORT_DATA,
					&tmpbuf, best_width);
#ifdef DEBUG
				sent += best_width;
#endif
			}
		}
	}
#ifdef DEBUG
	if (nbytes != sent) {
		error = 1;
		dp8390_rdma_abort(dp);
		cmn_err(CE_WARN,
			"%s:dma_get_pkt: nbytes:%d sent:%d tx_copy:%d\n",
			dp->name, nbytes, sent, tx_copy);
		goto x;
	}
#endif
	mutex_enter(&dp->pagelock);
	if ((error = dp8390_rdma_finish(dp)) != 0) {
		dp8390_rdma_abort(dp);
		cmn_err(CE_WARN,
			"%s: dp8390_rdma_put_pkt: RDMA did not stop",
			dp->name);
	}
	mutex_exit(&dp->pagelock);
x:
	/* Release RDMA counter */
	RDMA_UNLOCK(dp);

	DPRINTF(3, (CE_CONT, "!%s: dma_put_pkt: actual sent %d bytes",
			dp->name, sent));

	return error;
}
