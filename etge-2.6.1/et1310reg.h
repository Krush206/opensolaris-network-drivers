/*******************************************************************************
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright © 2005 Agere Systems Inc. 
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *
 * ET1310_address_map.h - Contains the register mapping for the ET1310
 *
 *------------------------------------------------------------------------------
 *
 * SOFTWARE LICENSE
 *
 * This software is provided subject to the following terms and conditions,
 * which you should read carefully before using the software.  Using this
 * software indicates your acceptance of these terms and conditions.  If you do
 * not agree with these terms and conditions, do not use the software.
 *
 * Copyright © 2005 Agere Systems Inc.
 * All rights reserved.
 *
 * Redistribution and use in source or binary forms, with or without
 * modifications, are permitted provided that the following conditions are met:
 *
 * . Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following Disclaimer as comments in the code as
 *    well as in the documentation and/or other materials provided with the
 *    distribution.
 * 
 * . Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following Disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * . Neither the name of Agere Systems Inc. nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Disclaimer
 *
 * THIS SOFTWARE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, INFRINGEMENT AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  ANY
 * USE, MODIFICATION OR DISTRIBUTION OF THIS SOFTWARE IS SOLELY AT THE USERS OWN
 * RISK. IN NO EVENT SHALL AGERE SYSTEMS INC. OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, INCLUDING, BUT NOT LIMITED TO, CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *   
 ******************************************************************************/




/******************************************************************************
 *  VERSION CONTROL INFORMATION
 ******************************************************************************

      $RCSFile: $
         $Date: 2005/10/28 18:43:43 $
     $Revision: 1.8 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef _ET1310REG_H_
#define _ET1310REG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* address map of GLOBAL module */
#define	TXQ_START_ADDR			0x0000
#define	TXQ_END_ADDR			0x0004
#define	RXQ_START_ADDR			0x0008
#define	RXQ_END_ADDR			0x000c
#define	PM_CSR				0x0010
#define	INT_STATUS			0x0018
#define	INT_MASK			0x001c
#define	INT_ALIAS_CLR_EN		0x0020
#define	INT_STATUS_ALIAS		0x0024
#define	SW_RESET			0x0028
#define	SLV_TIMER			0x002c
#define	MSI_CONFIG			0x0030
#define	LOOP_BACK			0x0034
#define	WATCHDOG_TIMER			0x0038

#define	TXDMA_CSR			0x1000
#define	TXDMA_PR_BASE_HI		0x1004
#define	TXDMA_PR_BASE_LO		0x1008
#define	TXDMA_PR_NUM_DES		0x100C
#define	TXDMA_TXQ_WR_ADDR		0x1010
#define	TXDMA_TXQ_WR_ADDR_EXT		0x1014
#define	TXDMA_TXQ_RD_ADDR		0x1018
#define	TXDMA_DMA_WB_BASE_HI		0x101c
#define	TXDMA_DMA_WB_BASE_LO		0x1020
#define	TXDMA_SERVICE_REQUEST		0x1024
#define	TXDMA_SERVICE_COMPLETE		0x1028
#define	TXDMA_CACHE_RD_INDEX		0x102c
#define	TXDMA_CACHE_WR_INDEX		0x1030
#define	TXDMA_ERROR			0x1034
#define	TXDMA_DESC_ABORT_COUNT		0x1038
#define	TXDMA_PAYLOAD_ABORT_CNT		0x103c
#define	TXDMA_WRITE_BACK_ABORT_CNT	0x1040
#define	TXDMA_DESC_TIMEOUT_CNT		0x1044
#define	TXDMA_PAYLOAD_TIMEOUT_CNT	0x1048
#define	TXDMA_WRITE_BACK_TIMEOUT_CNT	0x104c
#define	TXDMA_DESC_ERROR_COUNT		0x1050
#define	TXDMA_PAYLOAD_ERROR_CNT		0x1054
#define	TXDMA_WRITE_BACK_ERROR_CNT	0x1058
#define	TXDMA_DROPPED_TLP_COUNT		0x105c
#define	TXDMA_NEW_SERVICE_COMPLETE	0x1060
#define	TXDMA_ETHERNET_PACKET_COUNT	0x1064

#define	RXDMA_CSR			0x2000
#define	RXDMA_DMA_WB_BASE_LO		0x2004
#define	RXDMA_DMA_WB_BASE_HI		0x2008
#define	RXDMA_NUM_PKT_DONE		0x200C
#define	RXDMA_MAX_PKT_TIME		0x2010
#define	RXDMA_RXQ_RD_ADDR		0x2014
#define	RXDMA_RXQ_RD_ADDR_EXT		0x2018
#define	RXDMA_RXQ_WR_ADDR		0x201C
#define	RXDMA_PSR_BASE_LO		0x2020
#define	RXDMA_PSR_BASE_HI		0x2024
#define	RXDMA_PSR_NUM_DES		0x2028
#define	RXDMA_PSR_AVAIL_OFFSET		0x202C
#define	RXDMA_PSR_FULL_OFFSET		0x2030
#define	RXDMA_PSR_ACCESS_INDEX		0x2034
#define	RXDMA_PSR_MIN_DES		0x2038
#define	RXDMA_FBR0_BASE_LO		0x203C
#define	RXDMA_FBR0_BASE_HI		0x2040
#define	RXDMA_FBR0_NUM_DES		0x2044
#define	RXDMA_FBR0_AVAIL_OFFSET		0x2048
#define	RXDMA_FBR0_FULL_OFFSET		0x204C
#define	RXDMA_FBC0_RD_INDEX		0x2050
#define	RXDMA_FBR0_MIN_DES		0x2054
#define	RXDMA_FBR1_BASE_LO		0x2058
#define	RXDMA_FBR1_BASE_HI		0x205C
#define	RXDMA_FBR1_NUM_DES		0x2060
#define	RXDMA_FBR1_AVAIL_OFFSET		0x2064
#define	RXDMA_FBR1_FULL_OFFSET		0x2068
#define	RXDMA_FBC1_RD_INDEX		0x206C
#define	RXDMA_FBR1_MIN_DES		0x2070

#define	TXQ_START_ADDR_MASK		0x000003ffU

#define	TXQ_END_ADDR_MASK		0x000003ffU

#define	RXQ_START_ADDR_MASK		0x000003ffU

#define	RXQ_END_ADDR_MASK		0x000003ffU

/* power management csr, offset 0x0010 */
#define	PM_CSR_PM_JAGCORE_RX_RDY	0x00000200U
#define	PM_CSR_PM_JAGCORE_TX_RDY	0x00000100U
#define	PM_CSR_PM_PHY_LPED_EN		0x00000080U
#define	PM_CSR_PM_PHY_SW_COMA		0x00000040U
#define	PM_CSR_PM_RXCLK_GATE		0x00000020U
#define	PM_CSR_PM_TXCLK_GATE		0x00000010U
#define	PM_CSR_PM_SYSCLK_GATE		0x00000008U
#define	PM_CSR_PM_JAGCORE_RX_EN		0x00000004U
#define	PM_CSR_PM_JAGCORE_TX_EN		0x00000002U
#define	PM_CSR_PM_GIGEPHY_EN		0x00000001U

/* interrupt status, offset 0x0018 */
#define	INT_SLV_TIMEOUT			0x00100000U
#define	INT_MAC_STAT_INTERRUPT		0x00080000U
#define	INT_RXMAC_INTERRUPT		0x00040000U
#define	INT_TXMAC_INTERRUPT		0x00020000U
#define	INT_PHY_INTERRUPT		0x00010000U
#define	INT_WAKE_ON_LAN			0x00008000U
#define	INT_WATCHDOG_INTERRUPT		0x00004000U
#define	INT_RXDMA_ERR			0x00000200U
#define	INT_RXDMA_PKT_STAT_RING_LOW	0x00000100U
#define	INT_RXDMA_FB_RING1_LOW		0x00000080U
#define	INT_RXDMA_FB_RING0_LOW		0x00000040U
#define	INT_RXDMA_XFR_DONE		0x00000020U
#define	INT_TXDMA_ERR			0x00000010U
#define	INT_TXDMA_ISR			0x00000008U

#define	INT_BITS	\
	"\020"	\
	"\025SLV_TIMEOUT"	\
	"\024MAC_STAT"	\
	"\023RXMAC"	\
	"\022TXMAC"	\
	"\021PHY"	\
	"\020WAKE_ON_LAN"	\
	"\017WATCHDOG"	\
	"\012RXDMA_ERR"	\
	"\011RXDMA_PKT_STAT_RING_LOW"	\
	"\010RXDMA_FB_RING1_LOW"	\
	"\007RXDMA_FB_RING0_LOW"	\
	"\006RXDMA_XFR_DONE"	\
	"\005TXDMA_ERR"	\
	"\004TXDMA_ISR"

/* software reset registger, offset 0x28 */
#define	SW_RESET_SELFCLR_DIS		0x80000000U
#define	SW_RESET_MMC			0x00000040U
#define	SW_RESET_MAC_STAT		0x00000020U
#define	SW_RESET_MAC			0x00000010U
#define	SW_RESET_RXMAC			0x00000008U
#define	SW_RESET_TXMAC			0x00000004U
#define	SW_RESET_RXDMA			0x00000002U
#define	SW_RESET_TXDMA			0x00000001U

#define	SW_RESET_BITS	\
	"\020"	\
	"\040SELFCLR_DIS"	\
	"\007MMC"	\
	"\006MAC_STAT"	\
	"\005MAC"	\
	"\004RXMAC"	\
	"\003TXMAC"	\
	"\002RXDMA"	\
	"\001TXDMA"

/* slv timer register, offset 0x2c */
#define	SLV_TIMER_MASK			0x00ffffffU

/* msi config register, offset 0x30 */
#define	MSI_CONFIG_MSI_TC		0x00070000U
#define		MSI_CONFIG_MSI_TC_SHIFT		16
#define	MSI_CONFIG_MSI_VECTOR		0x0000003fU
#define		MSI_CONFIG_MSI_VECTOR_SHIFT	0

/* loopback register, offset 0x34 */
#define	LOOP_BACK_DMA_LOOPBACK		0x00000002U
#define	LOOP_BACK_MAC_LOOPBACK		0x00000001U


/* txdma control status reg, offset 0x1000 */
#define	TXDMA_CSR_TRAFFIC_CLASS		0x00001e00U
#define		TXDMA_CSR_TRAFFIC_CLASS_SHIFT	9
#define	TXDMA_CSR_SNGL_EPKT_MODE	0x00000100U
#define	TXDMA_CSR_CACHE_THRSHLD		0x000000f0U
#define		TXDMA_CSR_CACHE_THRSHLD_SHIFT	4
#define	TXDMA_CSR_DROP_TLP_DIS		0x00000002U
#define	TXDMA_CSR_HALT			0x00000001U

/* txdma packet ring base address hi reg at 0x1004 */
/* txdma packet ring base address low reg at 0x1008 */
/* txdma packet ring number of descriptor reg at 0x100c */
#define	TXDMA_PR_NUM_DES_MASK		0x003fffffU

/* txdma tx queue write address reg at 0x1010 */
#define	TXDMA_TXQ_WR_WRAP		0x00000400U
#define	TXDMA_TXQ_WR_MASK		0x000003ffU

/* txdma tx queue write address external reg at 0x1014 */
#define	TXDMA_TXQ_WR_EXT_WRAP		0x00000400U
#define	TXDMA_TXQ_WR_EXT_ADDR		0x000003ffU

/* txdma tx queue read address reg at 0x1018 */
#define	TXDMA_TXQ_RD_WRAP		0x00000400U
#define	TXDMA_TXQ_RD_MASK		0x000003ffU

/* txdma status writeback address hi reg at 0x101c */
/* txdma status writeback address lo reg at 0x1020 */

/* txdma service request reg at 0x1024 */
#define	TXDMA_SERVICE_REQUEST_WRAP	0x00000400U
#define	TXDMA_SERVICE_REQUEST_MASK	0x000003ffU


/* txdma service complete reg at 0x1028 */
#define	TXDMA_SERVICE_COMPLETE_WRAP	0x00000400U
#define	TXDMA_SERVICE_COMPLETE_MASK	0x000003ffU


/* txdma tx descriptor cache read index reg at 0x102c */
#define	TXDMA_RDI_WRAP			0x00000010U
#define	TXDMA_RDI_MASK			0x0000000fU


/* txdma tx descriptor cache write index reg at 0x1030 */
#define	TXDMA_WRI_WRAP			0x00000010U
#define	TXDMA_WRI_MASK			0x0000000fU


/* txdma error reg at 0x1034 */
#define	TXDMA_ERR_WRBK_REWIND		0x00000200U
#define	TXDMA_ERR_WRBK_RESEND		0x00000100U
#define	TXDMA_ERR_DESCR_REWIND		0x00000020U
#define	TXDMA_ERR_DESCR_RESEND		0x00000010U
#define	TXDMA_ERR_PYLD_REWIND		0x00000002U
#define	TXDMA_ERR_PYLD_RESEND		0x00000001U


#define	UINT32	uint32_t

/******************************************************************************
   structure for control status reg in rxdma address map
   Located at address 0x2000
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_CSR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:14;                          //bits 18-31
        UINT32 halt_status:1;                       //bit 17
        UINT32 pkt_done_flush:1;                    //bit 16
        UINT32 pkt_drop_disable:1;                  //bit 15
        UINT32 unused1:1;                           //bit 14
        UINT32 fbr1_enable:1;                       //bit 13
        UINT32 fbr1_size:2;                         //bits 11-12
        UINT32 fbr0_enable:1;                       //bit 10
        UINT32 fbr0_size:2;                         //bits 8-9
        UINT32 dma_big_endian:1;                    //bit 7
        UINT32 pkt_big_endian:1;                    //bit 6
        UINT32 psr_big_endian:1;                    //bit 5
        UINT32 fbr_big_endian:1;                    //bit 4
        UINT32 tc:3;                                //bits 1-3
        UINT32 halt:1;                              //bit 0
    #else
        UINT32 halt:1;                              //bit 0
        UINT32 tc:3;                                //bits 1-3
        UINT32 fbr_big_endian:1;                    //bit 4
        UINT32 psr_big_endian:1;                    //bit 5
        UINT32 pkt_big_endian:1;                    //bit 6
        UINT32 dma_big_endian:1;                    //bit 7
        UINT32 fbr0_size:2;                         //bits 8-9
        UINT32 fbr0_enable:1;                       //bit 10
        UINT32 fbr1_size:2;                         //bits 11-12
        UINT32 fbr1_enable:1;                       //bit 13
        UINT32 unused1:1;                           //bit 14
        UINT32 pkt_drop_disable:1;                  //bit 15
        UINT32 pkt_done_flush:1;                    //bit 16
        UINT32 halt_status:1;                       //bit 17
        UINT32 unused2:14;                          //bits 18-31
    #endif
    } bits;
} 
RXDMA_CSR_t, *PRXDMA_CSR_t;
#endif

#define	RXDMA_CSR_HALT_STATUS		0x00020000U
#define	RXDMA_CSR_PKT_DONE_FLUSH	0x00010000U
#define	RXDMA_CSR_PKT_DROP_DISABLE	0x00008000U
#define	RXDMA_CSR_FBR1_ENABLE		0x00002000U
#define	RXDMA_CSR_FBR1_SIZE		0x00001800U
#define		RXDMA_CSR_FBR1_SIZE_SHIFT	11
#define		RXDMA_CSR_FBR1_SIZE_2048	(0 << RXDMA_CSR_FBR1_SIZE_SHIFT)
#define		RXDMA_CSR_FBR1_SIZE_4096	(1 << RXDMA_CSR_FBR1_SIZE_SHIFT)
#define		RXDMA_CSR_FBR1_SIZE_8192	(2 << RXDMA_CSR_FBR1_SIZE_SHIFT)
#define		RXDMA_CSR_FBR1_SIZE_16384	(3 << RXDMA_CSR_FBR1_SIZE_SHIFT)
#define		FBR1_SIZE_BYTE(n)	(2048 << (n))
#define	RXDMA_CSR_FBR0_ENABLE		0x00000400U
#define	RXDMA_CSR_FBR0_SIZE		0x00000300U
#define		RXDMA_CSR_FBR0_SIZE_SHIFT	8
#define		RXDMA_CSR_FBR0_SIZE_128		(0 << RXDMA_CSR_FBR0_SIZE_SHIFT)
#define		RXDMA_CSR_FBR0_SIZE_256		(1 << RXDMA_CSR_FBR0_SIZE_SHIFT)
#define		RXDMA_CSR_FBR0_SIZE_512		(2 << RXDMA_CSR_FBR0_SIZE_SHIFT)
#define		RXDMA_CSR_FBR0_SIZE_1024	(3 << RXDMA_CSR_FBR0_SIZE_SHIFT)
#define		FBR0_SIZE_BYTE(n)	(128 << (n))
#define	RXDMA_CSR_DMA_BIG_ENDIAN	0x00000080U
#define	RXDMA_CSR_PKT_BIG_ENDIAN	0x00000040U
#define	RXDMA_CSR_PSR_BIG_ENDIAN	0x00000020U
#define	RXDMA_CSR_FBR_BIG_ENDIAN	0x00000010U
#define	RXDMA_CSR_TC			0x0000000eU
#define		RXDMA_CSR_TC_SHIFT		1
#define	RXDMA_CSR_HALT			0x00000001U


/******************************************************************************
   structure for dma writeback lo reg in rxdma address map
   located at address 0x2004
 *****************************************************************************/
#ifdef notdef
typedef struct _RXDMA_DMA_WB_BASE_LO_t
{
    UINT32 addr_lo;                                 //bits 0-31
} 
RXDMA_DMA_WB_BASE_LO_t, *PRXDMA_DMA_WB_BASE_LO_t;
#endif

/******************************************************************************
   structure for dma writeback hi reg in rxdma address map
   located at address 0x2008
 *****************************************************************************/
#ifdef notdef
typedef struct _RXDMA_DMA_WB_BASE_HI_t
{
    UINT32 addr_hi;                                 //bits 0-31
} 
RXDMA_DMA_WB_BASE_HI_t, *PRXDMA_DMA_WB_BASE_HI_t;
#endif

/******************************************************************************
   structure for number of packets done reg in rxdma address map
   located at address 0x200C
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_NUM_PKT_DONE_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:24;                           //bits 8-31
        UINT32 num_done:8;                          //bits 0-7
    #else
        UINT32 num_done:8;                          //bits 0-7
        UINT32 unused:24;                           //bits 8-31
    #endif
    } bits;
} 
RXDMA_NUM_PKT_DONE_t, *PRXDMA_NUM_PKT_DONE_t;
#endif

#define RXDMA_NUM_PKT_DONE_MASK	0x000000ffU

/******************************************************************************
   structure for max packet time reg in rxdma address map
   located at address 0x2010
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_MAX_PKT_TIME_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:14;                           //bits 18-31
        UINT32 time_done:18;                        //bits 0-17
    #else
        UINT32 time_done:18;                        //bits 0-17
        UINT32 unused:14;                           //bits 18-31
    #endif
    } bits;
} 
RXDMA_MAX_PKT_TIME_t, *PRXDMA_MAX_PKT_TIME_t;
#endif
#define	RXDMA_MAX_PKT_TIME_MASK		0x0003ffffU

/******************************************************************************
   structure for rx queue read address reg in rxdma address map
   located at address 0x2014
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_RXQ_RD_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_rd_wrap:1;                       //bit 10
        UINT32 rxq_rd:10;                           //bits 0-9
    #else
        UINT32 rxq_rd:10;                           //bits 0-9
        UINT32 rxq_rd_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_RD_ADDR_t, *PRXDMA_RXQ_RD_ADDR_t;
#endif

#define	RXDMA_RXQ_RD_ADDR_WRAP		0x00000400U
#define	RXDMA_RXQ_RD_ADDR_MASK		0x000003ffU

/******************************************************************************
   structure for rx queue read address external reg in rxdma address map
   located at address 0x2018
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_RXQ_RD_ADDR_EXT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_rd_ext_wrap:1;                   //bit 10
        UINT32 rxq_rd_ext:10;                       //bits 0-9
    #else
        UINT32 rxq_rd_ext:10;                       //bits 0-9
        UINT32 rxq_rd_ext_wrap:1;                   //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_RD_ADDR_EXT_t, *PRXDMA_RXQ_RD_ADDR_EXT_t;
#endif

#define	RXDMA_RXQ_RD_ADDR_EXT_WRAP	0x00000400U
#define	RXDMA_RXQ_RD_ADDR_EXT_MASK	0x000003ffU

/******************************************************************************
   structure for rx queue write address reg in rxdma address map
   located at address 0x201C
 *****************************************************************************/
#ifdef notdef
typedef union _RXDMA_RXQ_WR_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_wr_wrap:1;                       //bit 10
        UINT32 rxq_wr:10;                           //bits 0-9
    #else
        UINT32 rxq_wr:10;                           //bits 0-9
        UINT32 rxq_wr_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_WR_ADDR_t, *PRXDMA_RXQ_WR_ADDR_t;
#endif

/* packet status ring base address lo reg at 0x2020 */
/* packet status ring base address hi reg at 0x2024 */
/* packet status ring number of descriptors reg at 0x2028 */
/* packet status ring available offset reg at 0x202C */
#define	RXDMA_PSR_AVAIL_WRAP		0x00001000
#define	RXDMA_PSR_AVAIL_MASK		0x00000fff

/* packet status ring full offset reg at 0x2030 */
#define	RXDMA_PSR_FULL_OFFSET_WRAP	0x00001000U
#define	RXDMA_PSR_FULL_OFFSET_MASK	0x00000fffU

/* packet status ring access index reg at 0x2034 */
/* packet status ring minimum descriptors reg at 0x2038 */
/* free buffer ring base lo address reg at 0x203c */
/* free buffer ring base hi address reg at 0x2040 */
/* free buffer ring number of descriptors reg at 0x2044 */

/* free buffer ring 0 available offset reg at 0x2048 */
#define	RXDMA_FBR_AVAIL_OFFSET_WRAP	0x00000400
#define	RXDMA_FBR_AVAIL_OFFSET_MASK	0x000003ff

/* free buffer ring 0 full offset reg at 0x204c */
#define	RXDMA_FBR_FULL_OFFSET_WRAP	0x00000400
#define	RXDMA_FBR_FULL_OFFSET_MASK	0x000003ff

/* free buffer cache 0 full offset reg at 0x2050 */
/* free buffer ring 0 minimum descriptor reg at 0x2054 */
/* free buffer ring 1 base address lo reg at 0x2058 - 0x205c */


/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF TXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for control reg in txmac address map
   located at address 0x3000
 *****************************************************************************/
#ifdef notdef
typedef union _TXMAC_CTL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:24;                           //bits 8-31
        UINT32 cklseg_diable:1;                     //bit 7
        UINT32 ckbcnt_disable:1;                    //bit 6
        UINT32 cksegnum:1;                          //bit 5
        UINT32 async_disable:1;                     //bit 4
        UINT32 fc_disable:1;                        //bit 3
        UINT32 mcif_disable:1;                      //bit 2
        UINT32 mif_disable:1;                       //bit 1
        UINT32 txmac_en:1;                          //bit 0
    #else
        UINT32 txmac_en:1;                          //bit 0
        UINT32 mif_disable:1;                       //bit 1 mac interface
        UINT32 mcif_disable:1;                      //bit 2 memory controller interface
        UINT32 fc_disable:1;                        //bit 3
        UINT32 async_disable:1;                     //bit 4
        UINT32 cksegnum:1;                          //bit 5
        UINT32 ckbcnt_disable:1;                    //bit 6
        UINT32 cklseg_diable:1;                     //bit 7
        UINT32 unused:24;                           //bits 8-31
    #endif
    } bits;
} 
TXMAC_CTL_t, *PTXMAC_CTL_t;
#endif

#define	TXMAC_CTL_CKLSEG_DIABLE		0x00000080U
#define	TXMAC_CTL_CKBCNT_DISABLE	0x00000040U
#define	TXMAC_CTL_CKSEGNUM		0x00000020U
#define	TXMAC_CTL_ASYNC_DISABLE		0x00000010U
#define	TXMAC_CTL_FC_DISABLE		0x00000008U
#define	TXMAC_CTL_MCIF_DISABLE		0x00000004U
#define	TXMAC_CTL_MIF_DISABLE		0x00000002U
#define	TXMAC_CTL_TXMAC_EN		0x00000001U

/******************************************************************************
   structure for shadow pointer reg in txmac address map
   located at address 0x3004
 *****************************************************************************/
typedef union _TXMAC_SHADOW_PTR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:5;                         //bits 27-31
        UINT32 txq_rd_ptr:11;                       //bits 16-26
        UINT32 reserved:5;                          //bits 11-15
        UINT32 txq_wr_ptr:11;                       //bits 0-10
    #else
        UINT32 txq_wr_ptr:11;                       //bits 0-10
        UINT32 reserved:5;                          //bits 11-15
        UINT32 txq_rd_ptr:11;                       //bits 16-26
        UINT32 reserved2:5;                         //bits 27-31
    #endif
    } bits;
} 
TXMAC_SHADOW_PTR_t, *PTXMAC_SHADOW_PTR_t;


/******************************************************************************
   structure for error count reg in txmac address map
   located at address 0x3008
 *****************************************************************************/
typedef union _TXMAC_ERR_CNT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 reserved:4;                          //bits 8-11
        UINT32 txq_underrun:4;                      //bits 4-7
        UINT32 fifo_underrun:4;                     //bits 0-3
    #else
        UINT32 fifo_underrun:4;                     //bits 0-3
        UINT32 txq_underrun:4;                      //bits 4-7
        UINT32 reserved:4;                          //bits 8-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} TXMAC_ERR_CNT_t, *PTXMAC_ERR_CNT_t;


/******************************************************************************
   structure for max fill reg in txmac address map
   located at address 0x300C
 *****************************************************************************/
typedef union _TXMAC_MAX_FILL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 max_fill:12;                         //bits 0-11
    #else
        UINT32 max_fill:12;                         //bits 0-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} 
TXMAC_MAX_FILL_t, *PTXMAC_MAX_FILL_t;


/******************************************************************************
   structure for cf parameter reg in txmac address map
   located at address 0x3010
 *****************************************************************************/
#ifdef notdef
typedef union _TXMAC_CF_PARAM_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 cfep:16;                             //bits 16-31
        UINT32 cfpt:16;                             //bits 0-15
    #else
        UINT32 cfpt:16;                             //bits 0-15
        UINT32 cfep:16;                             //bits 16-31
    #endif
    } bits;
} 
TXMAC_CF_PARAM_t, *PTXMAC_CF_PARAM_t;
#endif

#define	TXMAC_CF_PARAM_CFEP		0xffff0000U
#define		TXMAC_CF_PARAM_CFEP_SHIFT	16
#define	TXMAC_CF_PARAM_CFPT		0x0000ffffU
#define		TXMAC_CF_PARAM_CFPT_SHIFT	0

/******************************************************************************
   structure for tx test reg in txmac address map
   located at address 0x3014
 *****************************************************************************/
typedef union _TXMAC_TXTEST_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:15;                          //bits 17-31
        UINT32 reserved1:1;                         //bit 16
        UINT32 txtest_en:1;                         //bit 15
        UINT32 unused1:4;                           //bits 11-14
        UINT32 txqtest_ptr:11;                      //bits 0-11
    #else
        UINT32 txqtest_ptr:11;                      //bits 0-10
        UINT32 unused1:4;                           //bits 11-14
        UINT32 txtest_en:1;                         //bit 15
        UINT32 reserved1:1;                         //bit 16
        UINT32 unused2:15;                          //bits 17-31
    #endif
    } bits;
} 
TXMAC_TXTEST_t, *PTXMAC_TXTEST_t;


/******************************************************************************
   structure for error reg in txmac address map
   located at address 0x3018
 *****************************************************************************/
typedef union _TXMAC_ERR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:23;                          //bits 9-31
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused1:2;                           //bits 6-7
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 lseg_err:1;                          //bit 2
        UINT32 segnum_err:1;                        //bit 1
        UINT32 seg0_err:1;                          //bit 0
    #else
        UINT32 seg0_err:1;                          //bit 0
        UINT32 segnum_err:1;                        //bit 1
        UINT32 lseg_err:1;                          //bit 2
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 unused1:2;                           //bits 6-7
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused2:23;                          //bits 9-31
    #endif
    } bits;
} 
TXMAC_ERR_t, *PTXMAC_ERR_t;


/******************************************************************************
   structure for error interrupt reg in txmac address map
   located at address 0x301C
 *****************************************************************************/
typedef union _TXMAC_ERR_INT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:23;                          //bits 9-31
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused1:2;                           //bits 6-7
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 lseg_err:1;                          //bit 2
        UINT32 segnum_err:1;                        //bit 1
        UINT32 seg0_err:1;                          //bit 0
    #else
        UINT32 seg0_err:1;                          //bit 0
        UINT32 segnum_err:1;                        //bit 1
        UINT32 lseg_err:1;                          //bit 2
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 unused1:2;                           //bits 6-7
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused2:23;                          //bits 9-31
    #endif
    } bits;
} 
TXMAC_ERR_INT_t, *PTXMAC_ERR_INT_t;


/******************************************************************************
   structure for error interrupt reg in txmac address map
   located at address 0x3020
 *****************************************************************************/
typedef union _TXMAC_CP_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:30;                           //bits 2-31
        UINT32 bp_req:1;                            //bit 1
        UINT32 bp_xonxoff:1;                        //bit 0
    #else
        UINT32 bp_xonxoff:1;                        //bit 0
        UINT32 bp_req:1;                            //bit 1
        UINT32 unused:30;                           //bits 2-31
    #endif
    } bits;
} 
TXMAC_BP_CTRL_t, *PTXMAC_BP_CTRL_t;


/******************************************************************************
   Tx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
#ifdef notdef
typedef struct _TXMAC_t
{                                                   //Location:
    TXMAC_CTL_t             ctl;                    //  0x3000
    TXMAC_SHADOW_PTR_t      shadow_ptr;             //  0x3004
    TXMAC_ERR_CNT_t         err_cnt;                //  0x3008
    TXMAC_MAX_FILL_t        max_fill;               //  0x300C
    TXMAC_CF_PARAM_t        cf_param;               //  0x3010
    TXMAC_TXTEST_t          tx_test;                //  0x3014
    TXMAC_ERR_t             err;                    //  0x3018
    TXMAC_ERR_INT_t         err_int;                //  0x301C
    TXMAC_BP_CTRL_t         bp_ctrl;                //  0x3020
}
TXMAC_t, *PTXMAC_t;
#endif

#define	TXMAC_CTL		0x3000
#define	TXMAC_SHADOW_PTR	0x3004
#define	TXMAC_ERR_CNT		0x3008
#define	TXMAC_MAX_FILL		0x300c
#define	TXMAC_CF_PARAM		0x3010
#define	TXMAC_TXTEST		0x3014
#define	TXMAC_ERR		0x3018
#define	TXMAC_ERR_INT		0x301c
#define	TXMAC_BP_CTRL		0x3020

/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF RXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for rxmac control reg in rxmac address map
   located at address 0x4000
 *****************************************************************************/
typedef union _RXMAC_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:25;                         //bits 7-31
        UINT32 rxmac_int_disable:1;                 //bit 6
        UINT32 async_disable:1;                     //bit 5
        UINT32 mif_disable:1;                       //bit 4
        UINT32 wol_disable:1;                       //bit 3
        UINT32 pkt_filter_disable:1;                //bit 2
        UINT32 mcif_disable:1;                      //bit 1
        UINT32 rxmac_en:1;                          //bit 0
    #else
        UINT32 rxmac_en:1;                          //bit 0
        UINT32 mcif_disable:1;                      //bit 1
        UINT32 pkt_filter_disable:1;                //bit 2
        UINT32 wol_disable:1;                       //bit 3
        UINT32 mif_disable:1;                       //bit 4
        UINT32 async_disable:1;                     //bit 5
        UINT32 rxmac_int_disable:1;                 //bit 6
        UINT32 reserved:25;                         //bits 7-31
    #endif
    } bits;
} 
RXMAC_CTRL_t, *PRXMAC_CTRL_t;

#define	RXMAC_CTRL_RXMAC_INT_DISABLE	0x00000040U
#define	RXMAC_CTRL_ASYNC_DISABLE	0X00000020U
#define	RXMAC_CTRL_MIF_DISABLE		0x00000010U
#define	RXMAC_CTRL_WOL_DISABLE		0x00000008U
#define	RXMAC_CTRL_PKT_FILTER_DISABLE	0x00000004U
#define	RXMAC_CTRL_MCIF_DISABLE		0x00000002U
#define	RXMAC_CTRL_RXMAC_EN		0x00000001U

/******************************************************************************
   structure for Wake On Lan Control and CRC 0 reg in rxmac address map
   located at address 0x4004
 *****************************************************************************/
typedef union _RXMAC_WOL_CTL_CRC0_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc0:16;                             //bits 16-31
        UINT32 reserve:4;                           //bits 12-15
        UINT32 ignore_pp:1;                         //bit 11
        UINT32 ignore_mp:1;                         //bit 10
        UINT32 clr_intr:1;                          //bit 9
        UINT32 ignore_link_chg:1;                   //bit 8
        UINT32 ignore_uni:1;                        //bit 7
        UINT32 ignore_multi:1;                      //bit 6
        UINT32 ignore_broad:1;                      //bit 5
        UINT32 valid_crc4:1;                        //bit 4
        UINT32 valid_crc3:1;                        //bit 3
        UINT32 valid_crc2:1;                        //bit 2
        UINT32 valid_crc1:1;                        //bit 1
        UINT32 valid_crc0:1;                        //bit 0
    #else
        UINT32 valid_crc0:1;                        //bit 0
        UINT32 valid_crc1:1;                        //bit 1
        UINT32 valid_crc2:1;                        //bit 2
        UINT32 valid_crc3:1;                        //bit 3
        UINT32 valid_crc4:1;                        //bit 4
        UINT32 ignore_broad:1;                      //bit 5
        UINT32 ignore_multi:1;                      //bit 6
        UINT32 ignore_uni:1;                        //bit 7
        UINT32 ignore_link_chg:1;                   //bit 8
        UINT32 clr_intr:1;                          //bit 9
        UINT32 ignore_mp:1;                         //bit 10
        UINT32 ignore_pp:1;                         //bit 11
        UINT32 reserve:4;                           //bits 12-15
        UINT32 crc0:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CTL_CRC0_t, *PRXMAC_WOL_CTL_CRC0_t;


/******************************************************************************
   structure for CRC 1 and CRC 2 reg in rxmac address map
   located at address 0x4008
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC12_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc2:16;                             //bits 16-31
        UINT32 crc1:16;                             //bits 0-15
    #else
        UINT32 crc1:16;                             //bits 0-15
        UINT32 crc2:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CRC12_t, *PRXMAC_WOL_CRC12_t;


/******************************************************************************
   structure for CRC 3 and CRC 4 reg in rxmac address map
   located at address 0x400C
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC34_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc4:16;                             //bits 16-31
        UINT32 crc3:16;                             //bits 0-15
    #else
        UINT32 crc3:16;                             //bits 0-15
        UINT32 crc4:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CRC34_t, *PRXMAC_WOL_CRC34_t;


/******************************************************************************
   structure for Wake On Lan Source Address Lo reg in rxmac address map
   located at address 0x4010
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_LO_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 sa3:8;                               //bits 24-31
        UINT32 sa4:8;                               //bits 16-23
        UINT32 sa5:8;                               //bits 8-15
        UINT32 sa6:8;                               //bits 0-7
    #else
        UINT32 sa6:8;                               //bits 0-7
        UINT32 sa5:8;                               //bits 8-15
        UINT32 sa4:8;                               //bits 16-23
        UINT32 sa3:8;                               //bits 24-31
    #endif
    } bits;
} 
RXMAC_WOL_SA_LO_t, *PRXMAC_WOL_SA_LO_t;


/******************************************************************************
   structure for Wake On Lan Source Address Hi reg in rxmac address map
   located at address 0x4014
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_HI_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 sa1:8;                               //bits 8-15
        UINT32 sa2:8;                               //bits 0-7
    #else
        UINT32 sa2:8;                               //bits 0-7
        UINT32 sa1:8;                               //bits 8-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_SA_HI_t, *PRXMAC_WOL_SA_HI_t;


/******************************************************************************
   structure for Wake On Lan mask reg in rxmac address map
   located at address 0x4018 - 0x4064
 *****************************************************************************/
typedef struct _RXMAC_WOL_MASK_t
{
    UINT32 mask;                                    //bits 0-31
} 
RXMAC_WOL_MASK_t, *PRXMAC_WOL_MASK_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 1 reg in rxmac address map
   located at address 0x4068
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr1_3:8;                           //bits 24-31
        UINT32 addr1_4:8;                           //bits 16-23
        UINT32 addr1_5:8;                           //bits 8-15
        UINT32 addr1_6:8;                           //bits 0-7
    #else
        UINT32 addr1_6:8;                           //bits 0-7
        UINT32 addr1_5:8;                           //bits 8-15
        UINT32 addr1_4:8;                           //bits 16-23
        UINT32 addr1_3:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR1_t, *PRXMAC_UNI_PF_ADDR1_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 2 reg in rxmac address map
   located at address 0x406C
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr2_3:8;                           //bits 24-31
        UINT32 addr2_4:8;                           //bits 16-23
        UINT32 addr2_5:8;                           //bits 8-15
        UINT32 addr2_6:8;                           //bits 0-7
    #else
        UINT32 addr2_6:8;                           //bits 0-7
        UINT32 addr2_5:8;                           //bits 8-15
        UINT32 addr2_4:8;                           //bits 16-23
        UINT32 addr2_3:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR2_t, *PRXMAC_UNI_PF_ADDR2_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 1 & 2 reg in rxmac address map
   located at address 0x4070
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR3_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr2_1:8;                           //bits 24-31
        UINT32 addr2_2:8;                           //bits 16-23
        UINT32 addr1_1:8;                           //bits 8-15
        UINT32 addr1_2:8;                           //bits 0-7
    #else
        UINT32 addr1_2:8;                           //bits 0-7
        UINT32 addr1_1:8;                           //bits 8-15
        UINT32 addr2_2:8;                           //bits 16-23
        UINT32 addr2_1:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR3_t, *PRXMAC_UNI_PF_ADDR3_t;


/******************************************************************************
   structure for Multicast Hash reg in rxmac address map
   located at address 0x4074 - 0x4080
 *****************************************************************************/
typedef struct _RXMAC_MULTI_HASH_t
{
    UINT32 hash;                                    //bits 0-31
} 
RXMAC_MULTI_HASH_t, *PRXMAC_MULTI_HASH_t;


/******************************************************************************
   structure for Packet Filter Control reg in rxmac address map
   located at address 0x4084
 *****************************************************************************/
#ifdef notdef
typedef union _RXMAC_PF_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:9;                           //bits 23-31
        UINT32 min_pkt_size:7;                      //bits 16-22
        UINT32 unused1:12;                          //bits 4-15
        UINT32 filter_frag_en:1;                    //bit 3
        UINT32 filter_uni_en:1;                     //bit 2
        UINT32 filter_multi_en:1;                   //bit 1
        UINT32 filter_broad_en:1;                   //bit 0
    #else
        UINT32 filter_broad_en:1;                   //bit 0
        UINT32 filter_multi_en:1;                   //bit 1
        UINT32 filter_uni_en:1;                     //bit 2
        UINT32 filter_frag_en:1;                    //bit 3
        UINT32 unused1:12;                          //bits 4-15
        UINT32 min_pkt_size:7;                      //bits 16-22
        UINT32 unused2:9;                           //bits 23-31
    #endif
    } bits;
} 
RXMAC_PF_CTRL_t, *PRXMAC_PF_CTRL_t;
#endif

#define	RXMAC_PF_CTRL_MIN_PKT_SIZE	0x007f0000
#define		RXMAC_PF_CTRL_MIN_PKT_SIZE_SHIFT	16
#define	RXMAC_PF_CTRL_FILTER_FRAG_EN	0x00000008
#define	RXMAC_PF_CTRL_FILTER_UNI_EN	0x00000004
#define	RXMAC_PF_CTRL_FILTER_MULTI_EN	0x00000002
#define	RXMAC_PF_CTRL_FILTER_BROAD_EN	0x00000001

/******************************************************************************
   structure for Memory Controller Interface Control Max Segment reg in rxmac 
   address map.  Located at address 0x4088
 *****************************************************************************/
#ifdef notdef
typedef union _RXMAC_MCIF_CTRL_MAX_SEG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 max_size:8;                          //bits 2-9
        UINT32 fc_en:1;                             //bit 1
        UINT32 seg_en:1;                            //bit 0
    #else
        UINT32 seg_en:1;                            //bit 0
        UINT32 fc_en:1;                             //bit 1
        UINT32 max_size:8;                          //bits 2-9
        UINT32 reserved:22;                         //bits 10-31
    #endif
    } bits;
} 
RXMAC_MCIF_CTRL_MAX_SEG_t, *PRXMAC_MCIF_CTRL_MAX_SEG_t;
#endif

#define	RXMAC_MCIF_CTRL_MAX_SEG_MAX_SIZE	0x000003fc
#define		RXMAC_MCIF_CTRL_MAX_SEG_MAX_SIZE_SHIFT	2
#define	RXMAC_MCIF_CTRL_MAX_SEG_FC_EN		0x00000002
#define	RXMAC_MCIF_CTRL_MAX_SEG_SEG_EN		0x00000001

/******************************************************************************
   structure for Memory Controller Interface Water Mark reg in rxmac address 
   map.  Located at address 0x408C
 *****************************************************************************/
typedef union _RXMAC_MCIF_WATER_MARK_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:6;                         //bits 26-31
        UINT32 mark_hi:10;                          //bits 16-25
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 mark_lo:10;                          //bits 0-9
    #else
        UINT32 mark_lo:10;                          //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 mark_hi:10;                          //bits 16-25
        UINT32 reserved2:6;                         //bits 26-31
    #endif
    } bits;
} 
RXMAC_MCIF_WATER_MARK_t, *PRXMAC_MCIF_WATER_MARK_t;


/******************************************************************************
   structure for Rx Queue Dialog reg in rxmac address map.  
   located at address 0x4090
 *****************************************************************************/
typedef union _RXMAC_RXQ_DIAG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:6;                         //bits 26-31
        UINT32 rd_ptr:10;                           //bits 16-25
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 wr_ptr:10;                           //bits 0-9
    #else
        UINT32 wr_ptr:10;                           //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 rd_ptr:10;                           //bits 16-25
        UINT32 reserved2:6;                         //bits 26-31
    #endif
    } bits;
} 
RXMAC_RXQ_DIAG_t, *PRXMAC_RXQ_DIAG_t;


/******************************************************************************
   structure for space availiable reg in rxmac address map.  
   located at address 0x4094
 *****************************************************************************/
typedef union _RXMAC_SPACE_AVAIL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:15;                        //bits 17-31
        UINT32 space_avail_en:1;                    //bit 16
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 space_avail:10;                      //bits 0-9
    #else
        UINT32 space_avail:10;                      //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 space_avail_en:1;                    //bit 16
        UINT32 reserved2:15;                        //bits 17-31
    #endif
    } bits;
} 
RXMAC_SPACE_AVAIL_t, *PRXMAC_SPACE_AVAIL_t;


/******************************************************************************
   structure for management interface reg in rxmac address map.  
   located at address 0x4098
 *****************************************************************************/
typedef union _RXMAC_MIF_CTL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserve:14;                          //bits 18-31
        UINT32 drop_pkt_en:1;                       //bit 17
        UINT32 drop_pkt_mask:17;                    //bits 0-16
    #else
        UINT32 drop_pkt_mask:17;                    //bits 0-16
        UINT32 drop_pkt_en:1;                       //bit 17
        UINT32 reserve:14;                          //bits 18-31
    #endif
    } bits;
} 
RXMAC_MIF_CTL_t, *PRXMAC_MIF_CTL_t;


/******************************************************************************
   structure for Error reg in rxmac address map.  
   located at address 0x409C
 *****************************************************************************/
typedef union _RXMAC_ERROR_REG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserve:28;                          //bits 4-31
        UINT32 mif:1;                               //bit 3
        UINT32 async:1;                             //bit 2
        UINT32 pkt_filter:1;                        //bit 1
        UINT32 mcif:1;                              //bit 0
    #else
        UINT32 mcif:1;                              //bit 0
        UINT32 pkt_filter:1;                        //bit 1
        UINT32 async:1;                             //bit 2
        UINT32 mif:1;                               //bit 3
        UINT32 reserve:28;                          //bits 4-31 
    #endif
    } bits;
} 
RXMAC_ERROR_REG_t, *PRXMAC_ERROR_REG_t;


/******************************************************************************
   Rx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
#ifdef notdef
typedef struct _RXMAC_t
{                                                   //Location:
    RXMAC_CTRL_t                ctrl;               //  0x4000
    RXMAC_WOL_CTL_CRC0_t        crc0;               //  0x4004
    RXMAC_WOL_CRC12_t           crc12;              //  0x4008
    RXMAC_WOL_CRC34_t           crc34;              //  0x400C
    RXMAC_WOL_SA_LO_t           sa_lo;              //  0x4010
    RXMAC_WOL_SA_HI_t           sa_hi;              //  0x4014
    RXMAC_WOL_MASK_t            mask0_word0;        //  0x4018
    RXMAC_WOL_MASK_t            mask0_word1;        //  0x401C
    RXMAC_WOL_MASK_t            mask0_word2;        //  0x4020
    RXMAC_WOL_MASK_t            mask0_word3;        //  0x4024
    RXMAC_WOL_MASK_t            mask1_word0;        //  0x4028
    RXMAC_WOL_MASK_t            mask1_word1;        //  0x402C
    RXMAC_WOL_MASK_t            mask1_word2;        //  0x4030
    RXMAC_WOL_MASK_t            mask1_word3;        //  0x4034
    RXMAC_WOL_MASK_t            mask2_word0;        //  0x4038
    RXMAC_WOL_MASK_t            mask2_word1;        //  0x403C
    RXMAC_WOL_MASK_t            mask2_word2;        //  0x4040
    RXMAC_WOL_MASK_t            mask2_word3;        //  0x4044
    RXMAC_WOL_MASK_t            mask3_word0;        //  0x4048
    RXMAC_WOL_MASK_t            mask3_word1;        //  0x404C
    RXMAC_WOL_MASK_t            mask3_word2;        //  0x4050
    RXMAC_WOL_MASK_t            mask3_word3;        //  0x4054
    RXMAC_WOL_MASK_t            mask4_word0;        //  0x4058
    RXMAC_WOL_MASK_t            mask4_word1;        //  0x405C
    RXMAC_WOL_MASK_t            mask4_word2;        //  0x4060
    RXMAC_WOL_MASK_t            mask4_word3;        //  0x4064
    RXMAC_UNI_PF_ADDR1_t        uni_pf_addr1;       //  0x4068
    RXMAC_UNI_PF_ADDR2_t        uni_pf_addr2;       //  0x406C
    RXMAC_UNI_PF_ADDR3_t        uni_pf_addr3;       //  0x4070
    RXMAC_MULTI_HASH_t          multi_hash1;        //  0x4074
    RXMAC_MULTI_HASH_t          multi_hash2;        //  0x4078
    RXMAC_MULTI_HASH_t          multi_hash3;        //  0x407C
    RXMAC_MULTI_HASH_t          multi_hash4;        //  0x4080
    RXMAC_PF_CTRL_t             pf_ctrl;            //  0x4084
    RXMAC_MCIF_CTRL_MAX_SEG_t   mcif_ctrl_max_seg;  //  0x4088
    RXMAC_MCIF_WATER_MARK_t     mcif_water_mark;    //  0x408C
    RXMAC_RXQ_DIAG_t            rxq_diag;           //  0x4090
    RXMAC_SPACE_AVAIL_t         space_avail;        //  0x4094

    RXMAC_MIF_CTL_t             mif_ctrl;           //  0x4098
    RXMAC_ERROR_REG_t           err_reg;            //  0x409C
}
RXMAC_t, *PRXMAC_t;
#endif

#define	RXMAC_CTRL		0x4000
#define	RXMAC_WOL_CTL_CRC0	0x4004
#define	RXMAC_WOL_CRC12		0x4008
#define	RXMAC_WOL_CRC34		0x400c
#define	RXMAC_WOL_SA_LO		0x4010
#define	RXMAC_WOL_SA_HI		0x4014
#define	RXMAC_WOL_MASK		0x4018
#define	RXMAC_UNI_PF_ADDR1	0x4068
#define	RXMAC_UNI_PF_ADDR2	0x406c
#define	RXMAC_UNI_PF_ADDR3	0x4070
#define	RXMAC_MULTI_HASH	0x4074
#define	RXMAC_PF_CTRL		0x4084
#define	RXMAC_MCIF_CTRL_MAX_SEG	0x4088
#define	RXMAC_MCIF_WATER_MARK	0x408c
#define	RXMAC_RXQ_DIAG		0x4090
#define	RXMAC_SPACE_AVAIL	0x4094
#define	RXMAC_MIF_CTL		0x4098
#define	RXMAC_ERROR_REG		0x409c

/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for configuration #1 reg in mac address map.  
   located at address 0x5000
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_CFG1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 soft_reset:1;                        //bit 31
        UINT32 sim_reset:1;                         //bit 30
        UINT32 reserved3:10;                        //bits 20-29
        UINT32 reset_rx_mc:1;                       //bit 19
        UINT32 reset_tx_mc:1;                       //bit 18
        UINT32 reset_rx_fun:1;                      //bit 17
        UINT32 reset_tx_fun:1;                      //bit 16
        UINT32 reserved2:7;                         //bits 9-15
        UINT32 loop_back:1;                         //bit 8
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 rx_flow:1;                           //bit 5
        UINT32 tx_flow:1;                           //bit 4
        UINT32 syncd_rx_en:1;                       //bit 3
        UINT32 rx_enable:1;                         //bit 2
        UINT32 syncd_tx_en:1;                       //bit 1
        UINT32 tx_enable:1;                         //bit 0
    #else
        UINT32 tx_enable:1;                         //bit 0
        UINT32 syncd_tx_en:1;                       //bit 1
        UINT32 rx_enable:1;                         //bit 2
        UINT32 syncd_rx_en:1;                       //bit 3
        UINT32 tx_flow:1;                           //bit 4
        UINT32 rx_flow:1;                           //bit 5
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 loop_back:1;                         //bit 8
        UINT32 reserved2:7;                         //bits 9-15
        UINT32 reset_tx_fun:1;                      //bit 16
        UINT32 reset_rx_fun:1;                      //bit 17
        UINT32 reset_tx_mc:1;                       //bit 18
        UINT32 reset_rx_mc:1;                       //bit 19
        UINT32 reserved3:10;                        //bits 20-29
        UINT32 sim_reset:1;                         //bit 30
        UINT32 soft_reset:1;                        //bit 31
    #endif
    } bits;
} 
MAC_CFG1_t, *PMAC_CFG1_t;
#endif

#define	MAC_CFG1_SOFT_RESET	0x80000000U
#define	MAC_CFG1_SIM_RESET	0x40000000U
#define	MAC_CFG1_RESET_RX_MC	0x00080000U
#define	MAC_CFG1_RESET_TX_MC	0x00040000U
#define	MAC_CFG1_RESET_RX_FUN	0x00020000U
#define	MAC_CFG1_RESET_TX_FUN	0x00010000U
#define	MAC_CFG1_LOOP_BACK	0x00000100U
#define	MAC_CFG1_RX_FLOW	0x00000020U
#define	MAC_CFG1_TX_FLOW	0x00000010U
#define	MAC_CFG1_SYNCD_RX_EN	0x00000008U
#define	MAC_CFG1_RX_ENABLE	0x00000004U
#define	MAC_CFG1_SYNCD_TX_EN	0x00000002U
#define	MAC_CFG1_TX_ENABLE	0x00000001U

#define	CFG1_BITS	\
	"\020"	\
	"\040SOFT_RST"	\
	"\037SIM_RST"	\
	"\024RST_RX_MC"	\
	"\023RST_TX_MC"	\
	"\022RST_RX_FUN"	\
	"\021RST_TX_FUN"	\
	"\011LOOP_BACK"	\
	"\006RX_FLOW"	\
	"\005TX_FLOW"	\
	"\004SYNCD_RX_EN"	\
	"\003RX_EN"	\
	"\002SYNCD_TX_EN"	\
	"\001TX_EN"

/******************************************************************************
   structure for configuration #2 reg in mac address map.  
   located at address 0x5004
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_CFG2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved3:16;                        //bits 16-31
        UINT32 preamble_len:4;                      //bits 12-15
        UINT32 reserved2:2;                         //bits 10-11
        UINT32 if_mode:2;                           //bits 8-9
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 huge_frame:1;                        //bit 5
        UINT32 len_check:1;                         //bit 4
        UINT32 undefined:1;                         //bit 3
        UINT32 pad_crc:1;                           //bit 2
        UINT32 crc_enable:1;                        //bit 1
        UINT32 full_duplex:1;                       //bit 0
    #else
        UINT32 full_duplex:1;                       //bit 0
        UINT32 crc_enable:1;                        //bit 1
        UINT32 pad_crc:1;                           //bit 2
        UINT32 undefined:1;                         //bit 3
        UINT32 len_check:1;                         //bit 4
        UINT32 huge_frame:1;                        //bit 5
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 if_mode:2;                           //bits 8-9
        UINT32 reserved2:2;                         //bits 10-11
        UINT32 preamble_len:4;                      //bits 12-15
        UINT32 reserved3:16;                        //bits 16-31
    #endif
    } bits;
} 
MAC_CFG2_t, *PMAC_CFG2_t;
#endif

#define	MAC_CFG2_PREAMBLE_LEN		0x0000f000U
#define		MAC_CFG2_PREAMBLE_LEN_SHIFT	12
#define	MAC_CFG2_IF_MODE		0x00000300U
#define		MAC_CFG2_IF_MODE_SHIFT		8
#define		MAC_CFG2_IF_MODE_1000		(2U  << MAC_CFG2_IF_MODE_SHIFT)
#define		MAC_CFG2_IF_MODE_100_10		(1U  << MAC_CFG2_IF_MODE_SHIFT)
#define	MAC_CFG2_HUGE_FRAME		0x00000020U
#define	MAC_CFG2_LEN_CHECK		0x00000010U
#define	MAC_CFG2_PAD_CRC		0x00000004U
#define	MAC_CFG2_CRC_ENABLE		0x00000002U
#define	MAC_CFG2_FULL_DUPLEX		0x00000001U

#define	CFG2_BITS	\
	"\020"	\
	"\006HUGE_FRAME"	\
	"\005LEN_CHECK"	\
	"\003PAD_CRC"	\
	"\002CRC_EN"	\
	"\001FULL_DUPLEX"

/******************************************************************************
   structure for Interpacket gap reg in mac address map.  
   located at address 0x5008
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_IPG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:1;                          //bit 31
        UINT32 non_B2B_ipg_1:7;                     //bits 24-30
        UINT32 undefined2:1;                        //bit 23
        UINT32 non_B2B_ipg_2:7;                     //bits 16-22
        UINT32 min_ifg_enforce:8;                   //bits 8-15
        UINT32 undefined1:1;                        //bit 7
        UINT32 B2B_ipg:7;                           //bits 0-6
    #else
        UINT32 B2B_ipg:7;                           //bits 0-6
        UINT32 undefined1:1;                        //bit 7
        UINT32 min_ifg_enforce:8;                   //bits 8-15
        UINT32 non_B2B_ipg_2:7;                     //bits 16-22
        UINT32 undefined2:1;                        //bit 23
        UINT32 non_B2B_ipg_1:7;                     //bits 24-30
        UINT32 reserved:1;                          //bit 31
    #endif
    } bits;
} 
MAC_IPG_t, *PMAC_IPG_t;
#endif

#define	MAC_IPG_NON_B2B_IPG_1		0x7f000000U
#define		MAC_IPG_NON_B2B_IPG_1_SHIFT	24
#define	MAC_IPG_NON_B2B_IPG_2		0x007f0000U
#define		MAC_IPG_NON_B2B_IPG_2_SHIFT	16
#define	MAC_IPG_MIN_IFG_ENFORCE		0x0000ff00U
#define		MAC_IPG_MIN_IFG_ENFORCE_SHIFT	8
#define	MAC_IPG_B2B_IPG			0x0000003fU
#define		MAC_IPG_B2B_IPG_SHIFT		0


/******************************************************************************
   structure for half duplex reg in mac address map.  
   located at address 0x500C
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_HFDP_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:8;                         //bits 24-31
        UINT32 alt_beb_trunc:4;                     //bits 23-20
        UINT32 alt_beb_enable:1;                    //bit 19
        UINT32 bp_no_backoff:1;                     //bit 18
        UINT32 no_backoff:1;                        //bit 17
        UINT32 excess_defer:1;                      //bit 16
        UINT32 rexmit_max:4;                        //bits 12-15
        UINT32 reserved1:2;                         //bits 10-11
        UINT32 coll_window:10;                      //bits 0-9
    #else
        UINT32 coll_window:10;                      //bits 0-9
        UINT32 reserved1:2;                         //bits 10-11
        UINT32 rexmit_max:4;                        //bits 12-15
        UINT32 excess_defer:1;                      //bit 16
        UINT32 no_backoff:1;                        //bit 17
        UINT32 bp_no_backoff:1;                     //bit 18
        UINT32 alt_beb_enable:1;                    //bit 19
        UINT32 alt_beb_trunc:4;                     //bits 23-20
        UINT32 reserved2:8;                         //bits 24-31
    #endif
    } bits;
} 
MAC_HFDP_t, *PMAC_HFDP_t;
#endif

#define	MAC_HFDP_ALT_BEB_TRUNC		0x00f00000
#define		MAC_HFDP_ALT_BEB_TRUNC_SHIFT	20
#define	MAC_HFDP_ALT_BEB_ENABLE		0x00080000
#define	MAC_HFDP_BP_NO_BACKOFF		0x00040000
#define	MAC_HFDP_NO_BACKOFF		0x00020000
#define	MAC_HFDP_EXCESS_DEFER		0x00010000
#define	MAC_HFDP_REXMIT_MAX		0x0000f000
#define		MAC_HFDP_REXMIT_MAX_SHIFT	12
#define	MAC_HFDP_COLL_WINDOW		0x000003ff
#define		MAC_HFDP_COLL_WINDOW_SHIFT	0

/******************************************************************************
   structure for Maximum Frame Length reg in mac address map.  
   located at address 0x5010
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_MAX_FM_LEN_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 max_len:16;                          //bits 0-15
    #else
        UINT32 max_len:16;                          //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MAC_MAX_FM_LEN_t, *PMAC_MAX_FM_LEN_t;
#endif

#define	MAC_MAX_FM_LEN_MASK		0x0000ffffU

/******************************************************************************
   structure for Reserve 1 reg in mac address map.  
   located at address 0x5014 - 0x5018
 *****************************************************************************/
typedef struct _MAC_RSV_t
{
    UINT32 value;                                   //bits 0-31
} 
MAC_RSV_t, *PMAC_RSV_t;


/******************************************************************************
   structure for Test reg in mac address map.  
   located at address 0x501C
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_TEST_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:29;                           //bits 3-31
        UINT32 mac_test:3;                          //bits 0-2
    #else
        UINT32 mac_test:3;                          //bits 0-2
        UINT32 unused:29;                           //bits 3-31
    #endif
    } bits;
} 
MAC_TEST_t, *PMAC_TEST_t;
#endif

#define	MAC_TEST_MASK	0x00000007U

/******************************************************************************
   structure for MII Management Configuration reg in mac address map.  
   located at address 0x5020
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_CFG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reset_mii_mgmt:1;                    //bit 31
        UINT32 reserved:25;                         //bits 6-30
        UINT32 scan_auto_incremt:1;                 //bit 5
        UINT32 preamble_suppress:1;                 //bit 4
        UINT32 undefined:1;                         //bit 3
        UINT32 mgmt_clk_reset:3;                    //bits 0-2
    #else
        UINT32 mgmt_clk_reset:3;                    //bits 0-2
        UINT32 undefined:1;                         //bit 3
        UINT32 preamble_suppress:1;                 //bit 4
        UINT32 scan_auto_incremt:1;                 //bit 5
        UINT32 reserved:25;                         //bits 6-30
        UINT32 reset_mii_mgmt:1;                    //bit 31
    #endif
    } bits;
} 
MII_MGMT_CFG_t, *PMII_MGMT_CFG_t;
#endif

#define	MII_MGMT_CFG_RESET_MII_MGMT	0x80000000U
#define	MII_MGMT_CFG_SCAN_AUTO_INCREMT	0x00000020U
#define	MII_MGMT_CFG_PREAMBLE_SUPPRESS	0x00000010U
#define	MII_MGMT_CFG_MGMT_CLK_RESET	0x00000007U
#define		MII_MGMT_CFG_MGMT_CLK_RESET_SHIFT	0


/******************************************************************************
   structure for MII Management Command reg in mac address map.  
   located at address 0x5024
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_CMD_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:30;                         //bits 2-31
        UINT32 scan_cycle:1;                        //bit 1
        UINT32 read_cycle:1;                        //bit 0
    #else
        UINT32 read_cycle:1;                        //bit 0
        UINT32 scan_cycle:1;                        //bit 1
        UINT32 reserved:30;                         //bits 2-31    
    #endif
    } bits;
} 
MII_MGMT_CMD_t, *PMII_MGMT_CMD_t;
#endif

#define	MII_MGMT_CMD_SCAN_CYCLE		0x00000002U
#define	MII_MGMT_CMD_READ_CYCLE		0x00000001U

/******************************************************************************
   structure for MII Management Address reg in mac address map.  
   located at address 0x5028
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:19;                        //bit 13-31
        UINT32 phy_addr:5;                          //bits 8-12
        UINT32 reserved1:3;                         //bits 5-7
        UINT32 reg_addr:5;                          //bits 0-4
    #else
        UINT32 reg_addr:5;                          //bits 0-4
        UINT32 reserved1:3;                         //bits 5-7
        UINT32 phy_addr:5;                          //bits 8-12
        UINT32 reserved2:19;                        //bit 13-31
    #endif
    } bits;
} 
MII_MGMT_ADDR_t, *PMII_MGMT_ADDR_t;
#endif

#define	MII_MGMT_ADDR_PHY_ADDR		0x0001f000U
#define		MII_MGMT_ADDR_PHY_ADDR_SHIFT	12
#define	MII_MGMT_ADDR_REG_ADDR		0x0000001fU
#define		MII_MGMT_ADDR_REG_ADDR_SHIFT	0

/******************************************************************************
   structure for MII Management Control reg in mac address map.  
   located at address 0x502C
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 phy_ctrl:16;                         //bits 0-15
    #else
        UINT32 phy_ctrl:16;                         //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MII_MGMT_CTRL_t, *PMII_MGMT_CTRL_t;
#endif

#define	MII_MGMT_CTRL_MASK		0x0000ffffU

/******************************************************************************
   structure for MII Management Status reg in mac address map.  
   located at address 0x5030
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_STAT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 phy_stat:16;                         //bits 0-15
    #else
        UINT32 phy_stat:16;                         //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MII_MGMT_STAT_t, *PMII_MGMT_STAT_t;
#endif

#define	MII_MGMT_STAT_MASK		0x0000ffffU

/******************************************************************************
   structure for MII Management Indicators reg in mac address map.  
   located at address 0x5034
 *****************************************************************************/
#ifdef notdef
typedef union _MII_MGMT_INDICATOR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:29;                         //bits 3-31
        UINT32 not_valid:1;                         //bit 2
        UINT32 scanning:1;                          //bit 1
        UINT32 busy:1;                              //bit 0
    #else
        UINT32 busy:1;                              //bit 0
        UINT32 scanning:1;                          //bit 1
        UINT32 not_valid:1;                         //bit 2
        UINT32 reserved:29;                         //bits 3-31
    #endif
    } bits;
} 
MII_MGMT_INDICATOR_t, *PMII_MGMT_INDICATOR_t;
#endif

#define	MII_MGMT_INDICATOR_NOT_VALID	0x00000004U
#define	MII_MGMT_INDICATOR_SCANNING	0x00000002U
#define	MII_MGMT_INDICATOR_BUSY		0x00000001U

/******************************************************************************
   structure for Interface Control reg in mac address map.  
   located at address 0x5038
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_IF_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reset_if_module:1;                   //bit 31
        UINT32 reserved4:3;                         //bit 28-30
        UINT32 tbi_mode:1;                          //bit 27
        UINT32 ghd_mode:1;                          //bit 26
        UINT32 lhd_mode:1;                          //bit 25
        UINT32 phy_mode:1;                          //bit 24
        UINT32 reset_per_mii:1;                     //bit 23
        UINT32 reserved3:6;                         //bits 17-22
        UINT32 speed:1;                             //bit 16
        UINT32 reset_pe100x:1;                      //bit 15
        UINT32 reserved2:4;                         //bits 11-14
        UINT32 force_quiet:1;                       //bit 10
        UINT32 no_cipher:1;                         //bit 9
        UINT32 disable_link_fail:1;                 //bit 8
        UINT32 reset_gpsi:1;                        //bit 7
        UINT32 reserved1:6;                         //bits 1-6
        UINT32 enab_jab_protect:1;                  //bit 0
    #else
        UINT32 enab_jab_protect:1;                  //bit 0
        UINT32 reserved1:6;                         //bits 1-6
        UINT32 reset_gpsi:1;                        //bit 7
        UINT32 disable_link_fail:1;                 //bit 8
        UINT32 no_cipher:1;                         //bit 9
        UINT32 force_quiet:1;                       //bit 10
        UINT32 reserved2:4;                         //bits 11-14
        UINT32 reset_pe100x:1;                      //bit 15
        UINT32 speed:1;                             //bit 16
        UINT32 reserved3:6;                         //bits 17-22
        UINT32 reset_per_mii:1;                     //bit 23
        UINT32 phy_mode:1;                          //bit 24
        UINT32 lhd_mode:1;                          //bit 25
        UINT32 ghd_mode:1;                          //bit 26
        UINT32 tbi_mode:1;                          //bit 27
        UINT32 reserved4:3;                         //bit 28-30
        UINT32 reset_if_module:1;                   //bit 31
    #endif
    } bits;
} 
MAC_IF_CTRL_t, *PMAC_IF_CTRL_t;
#endif

#define	MAC_IF_CTRL_RESET_IF_MODULE	0x80000000U
#define	MAC_IF_CTRL_TBI_MODE		0x08000000U
#define	MAC_IF_CTRL_GHD_MODE		0x04000000U
#define	MAC_IF_CTRL_LHD_MODE		0x02000000U
#define	MAC_IF_CTRL_PHY_MODE		0x01000000U
#define	MAC_IF_CTRL_RESET_PER_MII	0x00800000U
#define	MAC_IF_CTRL_SPEED		0x00010000U
#define	MAC_IF_CTRL_RESET_PE100X	0x00008000U
#define	MAC_IF_CTRL_FORCE_QUIET		0x00000400U
#define	MAC_IF_CTRL_NO_CIPHER		0x00000200U
#define	MAC_IF_CTRL_DISABLE_LINK_FAIL	0x00000100U
#define	MAC_IF_CTRL_RESET_GPSI		0x00000080U
#define	MAC_IF_CTRL_ENAB_JAB_PROTECT	0x00000001U

#define	IF_CTRL_BITS	\
	"\020"	\
	"\040RST_IF_MODULE"	\
	"\034TBI"	\
	"\033GHD"	\
	"\032LHD"	\
	"\031PHY"	\
	"\030RST_PER_MII"	\
	"\021SPEED"	\
	"\020RST_PE100X"	\
	"\013FORCE_QUIET"	\
	"\012NO_CIPHER"	\
	"\011DIS_LINK_FAIL"	\
	"\010RESET_GPSI	"	\
	"\001EN_JAB_PROTECT"

/******************************************************************************
   structure for Interface Status reg in mac address map.  
   located at address 0x503C
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_IF_STAT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 excess_defer:1;                      //bit 9
        UINT32 clash:1;                             //bit 8
        UINT32 phy_jabber:1;                        //bit 7
        UINT32 phy_link_ok:1;                       //bit 6
        UINT32 phy_full_duplex:1;                   //bit 5
        UINT32 phy_speed:1;                         //bit 4
        UINT32 pe100x_link_fail:1;                  //bit 3
        UINT32 pe10t_loss_carrie:1;                 //bit 2
        UINT32 pe10t_sqe_error:1;                   //bit 1
        UINT32 pe10t_jabber:1;                      //bit 0
    #else
        UINT32 pe10t_jabber:1;                      //bit 0
        UINT32 pe10t_sqe_error:1;                   //bit 1
        UINT32 pe10t_loss_carrie:1;                 //bit 2
        UINT32 pe100x_link_fail:1;                  //bit 3
        UINT32 phy_speed:1;                         //bit 4
        UINT32 phy_full_duplex:1;                   //bit 5
        UINT32 phy_link_ok:1;                       //bit 6
        UINT32 phy_jabber:1;                        //bit 7
        UINT32 clash:1;                             //bit 8
        UINT32 excess_defer:1;                      //bit 9
        UINT32 reserved:22;                         //bits 10-31
    #endif
    } bits;
} 
MAC_IF_STAT_t, *PMAC_IF_STAT_t;
#endif

#define	MAC_IF_STAT_EXCESS_DEFER	0x00000200U
#define	MAC_IF_STAT_CLASH		0x00000100U
#define	MAC_IF_STAT_PHY_JABBER		0x00000080U
#define	MAC_IF_STAT_PHY_LINK_OK		0x00000040U
#define	MAC_IF_STAT_PHY_FULL_DUPLEX	0x00000020U
#define	MAC_IF_STAT_PHY_SPEED		0x00000010U
#define	MAC_IF_STAT_PE100X_LINK_FAIL	0x00000008U
#define	MAC_IF_STAT_PE10T_LOSS_CARRIE	0x00000004U
#define	MAC_IF_STAT_PE10T_SQE_ERROR	0x00000002U
#define	MAC_IF_STAT_PE10T_JABBER	0x00000001U

/******************************************************************************
   structure for Mac Station Address, Part 1 reg in mac address map.  
   located at address 0x5040
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_STATION_ADDR1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 Octet6:8;                            //bits 24-31
        UINT32 Octet5:8;                            //bits 16-23
        UINT32 Octet4:8;                            //bits 8-15
        UINT32 Octet3:8;                            //bits 0-7
    #else
        UINT32 Octet3:8;                            //bits 0-7
        UINT32 Octet4:8;                            //bits 8-15
        UINT32 Octet5:8;                            //bits 16-23
        UINT32 Octet6:8;                            //bits 24-31
    #endif
    } bits;
} 
MAC_STATION_ADDR1_t, *PMAC_STATION_ADDR1_t;
#endif

/******************************************************************************
   structure for Mac Station Address, Part 2 reg in mac address map.  
   located at address 0x5044
 *****************************************************************************/
#ifdef notdef
typedef union _MAC_STATION_ADDR2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 Octet2:8;                            //bits 24-31
        UINT32 Octet1:8;                            //bits 16-23
        UINT32 reserved:16;                         //bits 0-15
    #else
        UINT32 reserved:16;                         //bit 0-15
        UINT32 Octet1:8;                            //bits 16-23
        UINT32 Octet2:8;                            //bits 24-31
    #endif
    } bits;
} 
MAC_STATION_ADDR2_t, *PMAC_STATION_ADDR2_t;
#endif

/******************************************************************************
   MAC Module of JAGCore Address Mapping
 *****************************************************************************/
#ifdef notdef
typedef struct _MAC_t
{                                                   //Location:
    MAC_CFG1_t                  cfg1;               //  0x5000
    MAC_CFG2_t                  cfg2;               //  0x5004
    MAC_IPG_t                   ipg;                //  0x5008
    MAC_HFDP_t                  hfdp;               //  0x500C
    MAC_MAX_FM_LEN_t            max_fm_len;         //  0x5010
    MAC_RSV_t                   rsv1;               //  0x5014
    MAC_RSV_t                   rsv2;               //  0x5018
    MAC_TEST_t                  mac_test;           //  0x501C
    MII_MGMT_CFG_t              mii_mgmt_cfg;       //  0x5020
    MII_MGMT_CMD_t              mii_mgmt_cmd;       //  0x5024
    MII_MGMT_ADDR_t             mii_mgmt_addr;      //  0x5028
    MII_MGMT_CTRL_t             mii_mgmt_ctrl;      //  0x502C
    MII_MGMT_STAT_t             mii_mgmt_stat;      //  0x5030
    MII_MGMT_INDICATOR_t        mii_mgmt_indicator; //  0x5034
    MAC_IF_CTRL_t               if_ctrl;            //  0x5038
    MAC_IF_STAT_t               if_stat;            //  0x503C
    MAC_STATION_ADDR1_t         station_addr_1;     //  0x5040
    MAC_STATION_ADDR2_t         station_addr_2;     //  0x5044
}
MAC_t, *PMAC_t;
#endif

#define	MAC_CFG1		0x5000
#define	MAC_CFG2		0x5004
#define	MAC_IPG			0x5008
#define	MAC_HFDP		0x500c
#define	MAC_MAX_FM_LEN		0x5010
#define	MAC_TEST		0x501c
#define	MII_MGMT_CFG		0x5020
#define	MII_MGMT_CMD		0x5024
#define	MII_MGMT_ADDR		0x5028
#define	MII_MGMT_CTRL		0x502c
#define	MII_MGMT_STAT		0x5030
#define	MII_MGMT_INDICATOR	0x5034
#define	MAC_IF_CTRL		0x5038
#define	MAC_IF_STAT		0x503c
#define	MAC_STATION_ADDR1	0x5040
#define	MAC_STATION_ADDR2	0x5044

/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MAC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF MAC STAT REGISTER ADDRESS MAP               ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for Carry Register One and it's Mask Register reg located in mac
   stat address map address 0x6130 and 0x6138.
 *****************************************************************************/
typedef union _MAC_STAT_REG_1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 tr64:1;                              //bit 31
        UINT32 tr127:1;                             //bit 30
        UINT32 tr255:1;                             //bit 29
        UINT32 tr511:1;                             //bit 28
        UINT32 tr1k:1;                              //bit 27
        UINT32 trmax:1;                             //bit 26
        UINT32 trmgv:1;                             //bit 25
        UINT32 unused:8;                            //bits 17-24
        UINT32 rbyt:1;                             //bit 16
        UINT32 rpkt:1;                             //bit 15
        UINT32 rfcs:1;                             //bit 14
        UINT32 rmca:1;                             //bit 13
        UINT32 rbca:1;                             //bit 12
        UINT32 rxcf:1;                             //bit 11
        UINT32 rxpf:1;                             //bit 10
        UINT32 rxuo:1;                             //bit 9
        UINT32 raln:1;                             //bit 8
        UINT32 rflr:1;                             //bit 7
        UINT32 rcde:1;                             //bit 6
        UINT32 rcse:1;                             //bit 5
        UINT32 rund:1;                             //bit 4
        UINT32 rovr:1;                             //bit 3
        UINT32 rfrg:1;                             //bit 2
        UINT32 rjbr:1;                             //bit 1
        UINT32 rdrp:1;                             //bit 0
    #else
        UINT32 rdrp:1;                             //bit 0
        UINT32 rjbr:1;                             //bit 1
        UINT32 rfrg:1;                             //bit 2
        UINT32 rovr:1;                             //bit 3
        UINT32 rund:1;                             //bit 4
        UINT32 rcse:1;                             //bit 5
        UINT32 rcde:1;                             //bit 6 
        UINT32 rflr:1;                             //bit 7
        UINT32 raln:1;                             //bit 8
        UINT32 rxuo:1;                             //bit 9
        UINT32 rxpf:1;                             //bit 10
        UINT32 rxcf:1;                             //bit 11
        UINT32 rbca:1;                             //bit 12
        UINT32 rmca:1;                             //bit 13
        UINT32 rfcs:1;                             //bit 14
        UINT32 rpkt:1;                             //bit 15
        UINT32 rbyt:1;                             //bit 16
        UINT32 unused:8;                            //bits 17-24
        UINT32 trmgv:1;                             //bit 25
        UINT32 trmax:1;                             //bit 26
        UINT32 tr1k:1;                              //bit 27
        UINT32 tr511:1;                             //bit 28
        UINT32 tr255:1;                             //bit 29
        UINT32 tr127:1;                             //bit 30
        UINT32 tr64:1;                              //bit 31
    #endif
    } bits;
} 
MAC_STAT_REG_1_t, *PMAC_STAT_REG_1_t;


/******************************************************************************
   structure for Carry Register Two Mask Register reg in mac stat address map.  
   located at address 0x613C
 *****************************************************************************/
typedef union _MAC_STAT_REG_2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:12;                           //bit 20-31
        UINT32 tjbr:1;                             //bit 19
        UINT32 tfcs:1;                             //bit 18
        UINT32 txcf:1;                             //bit 17        
        UINT32 tovr:1;                             //bit 16
        UINT32 tund:1;                             //bit 15
        UINT32 tfrg:1;                             //bit 14
        UINT32 tbyt:1;                             //bit 13
        UINT32 tpkt:1;                             //bit 12
        UINT32 tmca:1;                             //bit 11
        UINT32 tbca:1;                             //bit 10
        UINT32 txpf:1;                             //bit 9
        UINT32 tdfr:1;                             //bit 8
        UINT32 tedf:1;                             //bit 7
        UINT32 tscl:1;                             //bit 6
        UINT32 tmcl:1;                             //bit 5
        UINT32 tlcl:1;                             //bit 4
        UINT32 txcl:1;                             //bit 3
        UINT32 tncl:1;                             //bit 2
        UINT32 tpfh:1;                             //bit 1
        UINT32 tdrp:1;                             //bit 0
    #else
        UINT32 tdrp:1;                             //bit 0
        UINT32 tpfh:1;                             //bit 1
        UINT32 tncl:1;                             //bit 2
        UINT32 txcl:1;                             //bit 3
        UINT32 tlcl:1;                             //bit 4
        UINT32 tmcl:1;                             //bit 5
        UINT32 tscl:1;                             //bit 6
        UINT32 tedf:1;                             //bit 7
        UINT32 tdfr:1;                             //bit 8
        UINT32 txpf:1;                             //bit 9
        UINT32 tbca:1;                             //bit 10
        UINT32 tmca:1;                             //bit 11
        UINT32 tpkt:1;                             //bit 12
        UINT32 tbyt:1;                             //bit 13
        UINT32 tfrg:1;                             //bit 14
        UINT32 tund:1;                             //bit 15
        UINT32 tovr:1;                             //bit 16
        UINT32 txcf:1;                             //bit 17    
        UINT32 tfcs:1;                             //bit 18
        UINT32 tjbr:1;                             //bit 19
        UINT32 unused:12;                         //bit 20-31
    #endif
    } bits;
} 
MAC_STAT_REG_2_t, *PMAC_STAT_REG_2_t;




/******************************************************************************
   MAC STATS Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MAC_STAT_t
{                                                   //Location:
    UINT32 pad[32];                                 //  0x6000 - 607C

    //Tx/Rx 0-64 Byte Frame Counter
    UINT32 TR64;                                    //  0x6080
    
    //Tx/Rx 65-127 Byte Frame Counter
    UINT32 TR127;                                   //  0x6084
    
    //Tx/Rx 128-255 Byte Frame Counter
    UINT32 TR255;                                   //  0x6088
    
    //Tx/Rx 256-511 Byte Frame Counter
    UINT32 TR511;                                   //  0x608C
    
    //Tx/Rx 512-1023 Byte Frame Counter
    UINT32 TR1K;                                    //  0x6090
    
    //Tx/Rx 1024-1518 Byte Frame Counter
    UINT32 TRMax;                                   //  0x6094
    
    //Tx/Rx 1519-1522 Byte Good VLAN Frame Count
    UINT32 TRMgv;                                   //  0x6098
    
    //Rx Byte Counter
    UINT32 RByt;                                    //  0x609C
    
    //Rx Packet Counter
    UINT32 RPkt;                                    //  0x60A0
    
    //Rx FCS Error Counter
    UINT32 RFcs;                                    //  0x60A4
    
    //Rx Multicast Packet Counter
    UINT32 RMca;                                    //  0x60A8
    
    //Rx Broadcast Packet Counter
    UINT32 RBca;                                    //  0x60AC
    
    //Rx Control Frame Packet Counter
    UINT32 RxCf;                                    //  0x60B0
    
    //Rx Pause Frame Packet Counter
    UINT32 RxPf;                                    //  0x60B4
    
    //Rx Unknown OP Code Counter
    UINT32 RxUo;                                    //  0x60B8
    
    //Rx Alignment Error Counter
    UINT32 RAln;                                    //  0x60BC
    
    //Rx Frame Length Error Counter
    UINT32 RFlr;                                    //  0x60C0
    
    //Rx Code Error Counter
    UINT32 RCde;                                    //  0x60C4
    
    //Rx Carrier Sense Error Counter
    UINT32 RCse;                                    //  0x60C8
    
    //Rx Undersize Packet Counter
    UINT32 RUnd;                                    //  0x60CC
    
    //Rx Oversize Packet Counter
    UINT32 ROvr;                                    //  0x60D0
    
    //Rx Fragment Counter
    UINT32 RFrg;                                    //  0x60D4
    
    //Rx Jabber Counter
    UINT32 RJbr;                                    //  0x60D8
    
    //Rx Drop
    UINT32 RDrp;                                    //  0x60DC
    
    //Tx Byte Counter
    UINT32 TByt;                                    //  0x60E0
    
    //Tx Packet Counter
    UINT32 TPkt;                                    //  0x60E4
    
    //Tx Multicast Packet Counter
    UINT32 TMca;                                    //  0x60E8
    
    //Tx Broadcast Packet Counter
    UINT32 TBca;                                    //  0x60EC
    
    //Tx Pause Control Frame Counter
    UINT32 TxPf;                                    //  0x60F0
    
    //Tx Deferral Packet Counter
    UINT32 TDfr;                                    //  0x60F4
    
    //Tx Excessive Deferral Packet Counter
    UINT32 TEdf;                                    //  0x60F8
    
    //Tx Single Collision Packet Counter
    UINT32 TScl;                                    //  0x60FC
    
    //Tx Multiple Collision Packet Counter
    UINT32 TMcl;                                    //  0x6100
    
    //Tx Late Collision Packet Counter
    UINT32 TLcl;                                    //  0x6104
    
    //Tx Excessive Collision Packet Counter
    UINT32 TXcl;                                    //  0x6108
    
    //Tx Total Collision Packet Counter
    UINT32 TNcl;                                    //  0x610C
    
    //Tx Pause Frame Honored Counter
    UINT32 TPfh;                                    //  0x6110
    
    //Tx Drop Frame Counter
    UINT32 TDrp;                                    //  0x6114
    
    //Tx Jabber Frame Counter
    UINT32 TJbr;                                    //  0x6118
    
    //Tx FCS Error Counter
    UINT32 TFcs;                                    //  0x611C
    
    //Tx Control Frame Counter
    UINT32 TxCf;                                    //  0x6120
    
    //Tx Oversize Frame Counter
    UINT32 TOvr;                                    //  0x6124
    
    //Tx Undersize Frame Counter
    UINT32 TUnd;                                    //  0x6128
    
    //Tx Fragments Frame Counter
    UINT32 TFrg;                                    //  0x612C
    
    //Carry Register One Register
    MAC_STAT_REG_1_t Carry1;                        //  0x6130
    
    //Carry Register Two Register
    MAC_STAT_REG_2_t Carry2;                           //  0x6134
    
    //Carry Register One Mask Register
    MAC_STAT_REG_1_t Carry1M;                       //  0x6138
    
    //Carry Register Two Mask Register
    MAC_STAT_REG_2_t Carry2M;                       //  0x613C
}
MAC_STAT_t, *PMAC_STAT_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                END OF MAC STAT REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MMC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for Main Memory Controller Control reg in mmc address map.  
   located at address 0x7000
 *****************************************************************************/
#ifdef notdef
typedef union _MMC_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:25;                         //bits 7-31
        UINT32 force_ce:1;                          //bit 6
        UINT32 rxdma_disable:1;                     //bit 5
        UINT32 txdma_disable:1;                     //bit 4
        UINT32 txmac_disable:1;                     //bit 3
        UINT32 rxmac_disable:1;                     //bit 2
        UINT32 arb_disable:1;                       //bit 1
        UINT32 mmc_enable:1;                        //bit 0
    #else
        UINT32 mmc_enable:1;                        //bit 0
        UINT32 arb_disable:1;                       //bit 1
        UINT32 rxmac_disable:1;                     //bit 2
        UINT32 txmac_disable:1;                     //bit 3
        UINT32 txdma_disable:1;                     //bit 4
        UINT32 rxdma_disable:1;                     //bit 5
        UINT32 force_ce:1;                          //bit 6
        UINT32 reserved:25;                         //bits 7-31
    #endif
    } bits;
} 
MMC_CTRL_t, *PMMC_CTRL_t;
#endif

#define	MMC_CTRL_FORCE_CE	0x00000040
#define	MMC_CTRL_RXDMA_DISABLE	0x00000020
#define	MMC_CTRL_TXDMA_DISABLE	0x00000010
#define	MMC_CTRL_TXMAC_DISABLE	0x00000008
#define	MMC_CTRL_RXMAC_DISABLE	0x00000004
#define	MMC_CTRL_ARB_DISABLE	0x00000002
#define	MMC_CTRL_MMC_ENABLE	0x00000001

/******************************************************************************
   structure for Main Memory Controller Host Memory Access Address reg in mmc 
   address map.  Located at address 0x7004
 *****************************************************************************/
#ifdef notdef
typedef union _MMC_SRAM_ACCESS_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 byte_enable:16;                      //bits 16-31
        UINT32 reserved2:2;                         //bits 14-15
        UINT32 req_addr:10;                         //bits 4-13
        UINT32 reserved1:1;                         //bit 3
        UINT32 is_ctrl_word:1;                      //bit 2
        UINT32 wr_access:1;                         //bit 1
        UINT32 req_access:1;                        //bit 0
    #else
        UINT32 req_access:1;                        //bit 0
        UINT32 wr_access:1;                         //bit 1
        UINT32 is_ctrl_word:1;                      //bit 2
        UINT32 reserved1:1;                         //bit 3
        UINT32 req_addr:10;                         //bits 4-13
        UINT32 reserved2:2;                         //bits 14-15
        UINT32 byte_enable:16;                      //bits 16-31
    #endif
    } bits;
} 
MMC_SRAM_ACCESS_t, *PMMC_SRAM_ACCESS_t;
#endif

#define	MMC_SRAM_ACCESS_BYTE_ENABLE	0xffff0000U
#define	MMC_SRAM_ACCESS_REQ_ADDR	0x00003ff0U
#define	MMC_SRAM_ACCESS_IS_CTRL_WORD	0x00000004U
#define	MMC_SRAM_ACCESS_WR_ACCESS	0x00000002U
#define	MMC_SRAM_ACCESS_REQ_ACCESS	0x00000001U


/******************************************************************************
   structure for Main Memory Controller Host Memory Access Data reg in mmc 
   address map.  Located at address 0x7008 - 0x7014
 *****************************************************************************/
#ifdef notdef
typedef struct _MMC_SRAM_WORD_t
{
    UINT32 data;                                    //bits 0-31
} MMC_SRAM_WORD_t, *PMMC_SRAM_WORD_t;
#endif

/******************************************************************************
   Memory Control Module of JAGCore Address Mapping
 *****************************************************************************/
#ifdef notdef
typedef struct _MMC_t
{                                                   //Location:
    MMC_CTRL_t          mmc_ctrl;                   //  0x7000
    MMC_SRAM_ACCESS_t   sram_access;                //  0x7004
    MMC_SRAM_WORD_t     sram_word1;                 //  0x7008
    MMC_SRAM_WORD_t     sram_word2;                 //  0x700C
    MMC_SRAM_WORD_t     sram_word3;                 //  0x7010
    MMC_SRAM_WORD_t     sram_word4;                 //  0x7014
}
MMC_t, *PMMC_t;
#endif

#define	MMC_CTRL	0x7000
#define	MMC_SRAM_ACCESS	0x7004
#define	MMC_SRAM_WORD	0x7008

/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MMC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF EXP ROM REGISTER ADDRESS MAP                ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   Expansion ROM Module of JAGCore Address Mapping
 *****************************************************************************/

/* Take this out until it is not empty */
#if 0
typedef struct _EXP_ROM_t
{

}
EXP_ROM_t, *PEXP_ROM_t;
#endif
/*===========================================================================*/
/*===========================================================================*/
/*===                 END OF EXP ROM REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/



/******************************************************************************
   JAGCore Address Mapping
 *****************************************************************************/
#define	INTERNAL_MEM_SIZE	0x400U
#define	INTERNAL_MEM_RX_OFFSET	0x1ffU	/* tx:50% rx:50% */

#ifdef notdef
typedef struct _ADDRESS_MAP_t
{
    GLOBAL_t    global;
    UCHAR       unused_global[4096 - sizeof (GLOBAL_t)];    //unused section of global address map
    TXDMA_t     txdma;
    UCHAR       unused_txdma[4096 - sizeof (TXDMA_t)];     //unused section of txdma address map
    RXDMA_t     rxdma;
    UCHAR       unused_rxdma[4096 - sizeof (RXDMA_t)];     //unused section of rxdma address map
    TXMAC_t     txmac;
    UCHAR       unused_txmac[4096 - sizeof (TXMAC_t)];     //unused section of txmac address map
    RXMAC_t     rxmac;
    UCHAR       unused_rxmac[4096 - sizeof (RXMAC_t)];     //unused section of rxmac address map
    MAC_t       mac;
    UCHAR       unused_mac[4096 - sizeof (MAC_t)];       //unused section of mac address map
    MAC_STAT_t  macStat;
    UCHAR       unused_mac_stat[4096 - sizeof (MAC_STAT_t)];  //unused section of mac stat address map
    MMC_t       mmc;
    UCHAR       unused_mmc[4096 - sizeof (MMC_t)];       //unused section of mmc address map
    UCHAR       unused_[1015808];       //unused section of address map

/* Take this out until it is not empty */
#if 0
    EXP_ROM_t   exp_rom;
#endif

    UCHAR       unused_exp_rom[4096];   //MGS-size TBD
    UCHAR       unused__[524288];       //unused section of address map
} 
ADDRESS_MAP_t, *PADDRESS_MAP_t;
#endif
/*===========================================================================*/


/******************************************************************************
   Typedefs for Tx Descriptor Ring
 *****************************************************************************/
/******************************************************************************
   TXDESC_WORD2_t structure holds part of the control bits in the Tx Descriptor
   ring for the ET-1310 
 *****************************************************************************/
#ifdef notdef
typedef union _txdesc_word2_t 
{
    UINT32 value;
    struct 
    {
        #ifdef _BIT_FIELDS_HTOL
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
        #else
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
        #endif /* _BIT_FIELDS_HTOL */
    } bits;
}TXDESC_WORD2_t, *PTXDESC_WORD2_t;
#endif


/******************************************************************************
   TXDESC_WORD3_t structure holds part of the control bits in the Tx Descriptor
   ring for the ET-1310 
 *****************************************************************************/
#ifdef notdef
typedef union  _txdesc_word3_t 
{
    UINT32 value;
    struct 
    {
        #ifdef _BIT_FIELDS_HTOL
            UINT32 unused:17;                       //bits 15-31
            UINT32 udpa:1;                          //bit 14(UDP checksum assist)
            UINT32 tcpa:1;                          //bit 13(TCP checksum assist)
            UINT32 ipa:1;                           //bit 12(IP checksum assist)
            UINT32 vlan:1;                          //bit 11(append VLAN tag)
            UINT32 hp:1;                            //bit 10(Packet is a Huge packet)
            UINT32 pp:1;                            //bit  9(pad packet)
            UINT32 mac:1;                           //bit  8(MAC override)
            UINT32 crc:1;                           //bit  7(append CRC)
            UINT32 e:1;                             //bit  6(Tx frame has error)
            UINT32 pf:1;                            //bit  5(send pause frame)
            UINT32 bp:1;                            //bit  4(Issue half-duplex backpressure (XON/XOFF)
            UINT32 cw:1;                            //bit  3(Control word - no packet data)
            UINT32 ir:1;                            //bit  2(interrupt the processor when this pkt sent)
            UINT32 f:1;                             //bit  1(first packet in the sequence)
            UINT32 l:1;                             //bit  0(last packet in the sequence)
        #else
            UINT32 l:1;                             //bit  0(last packet in the sequence)
            UINT32 f:1;                             //bit  1(first packet in the sequence)
            UINT32 ir:1;                            //bit  2(interrupt the processor when this pkt sent)
            UINT32 cw:1;                            //bit  3(Control word - no packet data)
            UINT32 bp:1;                            //bit  4(Issue half-duplex backpressure (XON/XOFF)
            UINT32 pf:1;                            //bit  5(send pause frame)
            UINT32 e:1;                             //bit  6(Tx frame has error)
            UINT32 crc:1;                           //bit  7(append CRC)
            UINT32 mac:1;                           //bit  8(MAC override)
            UINT32 pp:1;                            //bit  9(pad packet)
            UINT32 hp:1;                            //bit 10(Packet is a Huge packet)
            UINT32 vlan:1;                          //bit 11(append VLAN tag)
            UINT32 ipa:1;                           //bit 12(IP checksum assist)
            UINT32 tcpa:1;                          //bit 13(TCP checksum assist)
            UINT32 udpa:1;                          //bit 14(UDP checksum assist)
            UINT32 unused:17;                       //bits 15-31
        #endif /* _BIT_FIELDS_HTOL */
    } bits;
}TXDESC_WORD3_t, *PTXDESC_WORD3_t;
#endif



/******************************************************************************
   TX_DESC_ENTRY_t is sructure representing each descriptor on the ring
 *****************************************************************************/
#ifdef notdef
typedef struct _tx_desc_entry_t 
{
    UINT32         DataBufferPtrHigh;
    UINT32         DataBufferPtrLow;
    TXDESC_WORD2_t word2;                           // control words how to xmit the
    TXDESC_WORD3_t word3;                           // data (detailed above)
}
TX_DESC_ENTRY_t, *PTX_DESC_ENTRY_t;
#endif

struct tx_desc {
	volatile uint32_t	txd0;	/* data buffer ptr high */
	volatile uint32_t	txd1;	/* data buffer ptr low */
	volatile uint32_t	txd2;
#define	TXD2_VLAN_PRIO	0xe0000000U
#define		TXD2_VLAN_PRIO_SHIFT	29
#define	TXD2_VLAN_CFI	0x10000000U
#define		TXD2_VLAN_CFI_SHIFT	28
#define	TXD2_VLAN_TAG	0x0fff0000U
#define		TXD2_VLAN_TAG_SHIFT	16
#define	TXD2_LENGTH	0x0000ffffU
	volatile uint32_t	txd3;
#define	TXD3_UDPA	0x00004000U	/* UDP checksum assist */
#define	TXD3_TCPA	0x00002000U	/* TCP checksum assist */
#define	TXD3_IPA	0x00001000U	/* IP checksum assist */
#define	TXD3_VLAN	0x00000800U	/* append VLAN tag */
#define	TXD3_HP		0x00000400U	/* Packet is a Huge packet */
#define	TXD3_PP		0x00000200U	/* pad packet */
#define	TXD3_MAC	0x00000100U	/* MAC override */
#define	TXD3_CRC	0x00000080U	/* append CRC */
#define	TXD3_E		0x00000040U	/* Tx frame has error */
#define	TXD3_PF		0x00000020U	/* send pause frame */
#define	TXD3_BP		0x00000010U	/* Issue half-duplex backpressure */
#define	TXD3_CW		0x00000008U	/* Control word  (no packet data) */
#define	TXD3_IR		0x00000004U	/* interrupt */
#define	TXD3_F		0x00000002U	/* first fragment */
#define	TXD3_L		0x00000001U	/* last fragment */
};

/*===========================================================================*/




/******************************************************************************
   Typedefs for Tx DMA engine status writeback
 *****************************************************************************/
/******************************************************************************
   TX_STATUS_BLOCK_t is sructure representing the status of the Tx DMA engine
   it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/
#ifdef notdef
typedef union _tx_status_block_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 serv_cpl_wrap:1;                     //bit 10
        UINT32 serv_cpl:10;                         //bits 0-9
    #else
        UINT32 serv_cpl:10;                         //bits 0-9
        UINT32 serv_cpl_wrap:1;                     //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TX_STATUS_BLOCK_t, *PTX_STATUS_BLOCK_t;
#endif

#define	TX_STATUS_BLOCK_SERVICE_CPL_WRAP	0x00000400U
#define	TX_STATUS_BLOCK_SERVICE_CPT	0x000003ffU

/* #define USE_FBR0 TRUE */

#ifdef USE_FBR0
//#define FBR0_BUFFER_SIZE 256
#endif

//#define FBR1_BUFFER_SIZE 2048

/* #define FBR_CHUNKS 32 */

/* #define MAX_DESC_PER_RING_RX         1024  */

/******************************************************************************
   number of RFDs - default and min
 *****************************************************************************/
#ifdef USE_FBR0
    #define RFD_LOW_WATER_MARK              40
    #define NIC_MIN_NUM_RFD                 64
    #define NIC_DEFAULT_NUM_RFD             1024
#else
    #define RFD_LOW_WATER_MARK              20
    #define NIC_MIN_NUM_RFD                 64
    #define NIC_DEFAULT_NUM_RFD             256
#endif

#define NUM_PACKETS_HANDLED 256

#define ALCATEL_BAD_STATUS       0xe47f0000
#define ALCATEL_MULTICAST_PKT    0x01000000
#define ALCATEL_BROADCAST_PKT    0x02000000


/******************************************************************************
   typedefs for Free Buffer Descriptors
 *****************************************************************************/
#ifdef notdef
typedef union _FBR_WORD2_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 bi:10;                               //bits 0-9(Buffer Index)
    #else
        UINT32 bi:10;                               //bits 0-9(Buffer Index)
        UINT32 reserved:22;                         //bit 10-31
    #endif
    } bits;
}
FBR_WORD2_t, *PFBR_WORD2_t;

typedef struct _FBR_DESC_t
{
    UINT32         addr_lo;
    UINT32         addr_hi;
    FBR_WORD2_t    word2;
}
FBR_DESC_t, *PFBR_DESC_t;
#endif

struct fbr_desc {
	volatile uint32_t	fbd0;	/* low address */
	volatile uint32_t	fbd1;	/* high address */
	volatile uint32_t	fbd2;
#define	FBD2_BI	0x000003ffU
};

/*===========================================================================*/




/******************************************************************************
   Typedefs for Packet Status Ring Descriptors
 *****************************************************************************/
#ifdef notdef
typedef union _PKT_STAT_DESC_WORD0_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        // top 16 bits are from the Alcatel Status Word as enumerated in
        // PE-MCXMAC Data Sheet IPD DS54 0210-1 (also IPD-DS80 0205-2)
#if 0
        UINT32 asw_trunc:1;                         //bit 31(Rx frame truncated)
#endif
        UINT32 asw_long_evt:1;                      //bit 31(Rx long event)
        UINT32 asw_VLAN_tag:1;                      //bit 30(VLAN tag detected)
        UINT32 asw_unsupported_op:1;                //bit 29(unsupported OP code)
        UINT32 asw_pause_frame:1;                   //bit 28(is a pause frame)
        UINT32 asw_control_frame:1;                 //bit 27(is a control frame)
        UINT32 asw_dribble_nibble:1;                //bit 26(spurious bits after EOP)
        UINT32 asw_broadcast:1;                     //bit 25(has a broadcast address)
        UINT32 asw_multicast:1;                     //bit 24(has a multicast address)
        UINT32 asw_OK:1;                            //bit 23(valid CRC + no code error)
        UINT32 asw_too_long:1;                      //bit 22(frame length > 1518 bytes)
        UINT32 asw_len_chk_err:1;                   //bit 21(frame length field incorrect)
        UINT32 asw_CRC_err:1;                       //bit 20(CRC error)
        UINT32 asw_code_err:1;                      //bit 19(one or more nibbles signalled as errors)
        UINT32 asw_false_carrier_event:1;           //bit 18(bad carrier since last good packet)
        UINT32 asw_RX_DV_event:1;                   //bit 17(short receive event detected)
        UINT32 asw_prev_pkt_dropped:1;              //bit 16(e.g. IFG too small on previous)
        UINT32 unused:5;                            //bits 11-15
        UINT32 vp:1;                                //bit 10(VLAN Packet)
        UINT32 jp:1;                                //bit 9(Jumbo Packet)
        UINT32 ft:1;                                //bit 8(Frame Truncated)
        UINT32 drop:1;                              //bit 7(Drop packet)
        UINT32 rxmac_error:1;                       //bit 6(RXMAC Error Indicator)
        UINT32 wol:1;                               //bit 5(WOL Event)
        UINT32 tcpp:1;                              //bit 4(TCP checksum pass)
        UINT32 tcpa:1;                              //bit 3(TCP checksum assist)
        UINT32 ipp:1;                               //bit 2(IP checksum pass)
        UINT32 ipa:1;                               //bit 1(IP checksum assist)
        UINT32 hp:1;                                //bit 0(hash pass)
    #else
        UINT32 hp:1;                                //bit 0(hash pass)
        UINT32 ipa:1;                               //bit 1(IP checksum assist)
        UINT32 ipp:1;                               //bit 2(IP checksum pass)
        UINT32 tcpa:1;                              //bit 3(TCP checksum assist)
        UINT32 tcpp:1;                              //bit 4(TCP checksum pass)
        UINT32 wol:1;                               //bit 5(WOL Event)
        UINT32 rxmac_error:1;                       //bit 6(RXMAC Error Indicator)
        UINT32 drop:1;                              //bit 7(Drop packet)
        UINT32 ft:1;                                //bit 8(Frame Truncated)
        UINT32 jp:1;                                //bit 9(Jumbo Packet)
        UINT32 vp:1;                                //bit 10(VLAN Packet)
        UINT32 unused:5;                            //bits 11-15
        UINT32 asw_prev_pkt_dropped:1;              //bit 16(e.g. IFG too small on previous)
        UINT32 asw_RX_DV_event:1;                   //bit 17(short receive event detected)
        UINT32 asw_false_carrier_event:1;           //bit 18(bad carrier since last good packet)
        UINT32 asw_code_err:1;                      //bit 19(one or more nibbles signalled as errors)
        UINT32 asw_CRC_err:1;                       //bit 20(CRC error)
        UINT32 asw_len_chk_err:1;                   //bit 21(frame length field incorrect)
        UINT32 asw_too_long:1;                      //bit 22(frame length > 1518 bytes)
        UINT32 asw_OK:1;                            //bit 23(valid CRC + no code error)
        UINT32 asw_multicast:1;                     //bit 24(has a multicast address)
        UINT32 asw_broadcast:1;                     //bit 25(has a broadcast address)
        UINT32 asw_dribble_nibble:1;                //bit 26(spurious bits after EOP)
        UINT32 asw_control_frame:1;                 //bit 27(is a control frame)
        UINT32 asw_pause_frame:1;                   //bit 28(is a pause frame)
        UINT32 asw_unsupported_op:1;                //bit 29(unsupported OP code)
        UINT32 asw_VLAN_tag:1;                      //bit 30(VLAN tag detected)
        UINT32 asw_long_evt:1;                      //bit 31(Rx long event)
#if 0
        UINT32 asw_trunc:1;                         //bit 31(Rx frame truncated)
#endif
    #endif
    } bits;
}
PKT_STAT_DESC_WORD0_t, *PPKT_STAT_WORD0_t;

typedef union _PKT_STAT_DESC_WORD1_t
{
    UINT32 value;
    struct
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:4;                            //bits 28-31
        UINT32 ri:2;                                //bits 26-27(Ring Index)
        UINT32 bi:10;                               //bits 16-25(Buffer Index)
        UINT32 length:16;                           //bit 0-15(length in bytes)
    #else
        UINT32 length:16;                           //bit 0-15(length in bytes)
        UINT32 bi:10;                               //bits 16-25(Buffer Index)
        UINT32 ri:2;                                //bits 26-27(Ring Index)
        UINT32 unused:4;                            //bits 28-31
    #endif
    } bits;
}
PKT_STAT_DESC_WORD1_t, *PPKT_STAT_WORD1_t;

typedef struct _PKT_STAT_DESC_t
{
    PKT_STAT_DESC_WORD0_t   word0;
    PKT_STAT_DESC_WORD1_t   word1;
}
PKT_STAT_DESC_t, *PPKT_STAT_DESC_t;
#endif

struct psr_desc {
	volatile uint32_t	psrd0;
#define	PSRD0_ASW_LONG_EVT	0x80000000U	/* Rx long event */
#define	PSRD0_ASW_VLAN_TAG	0x40000000U	/* VLAN tag detected */
#define	PSRD0_ASW_UNSUPPORTED_OP	0x20000000U	/* unsupported OP code */
#define	PSRD0_ASW_PAUSE_FRAME	0x10000000U	/* is a pause frame */
#define	PSRD0_ASW_CONTROL_FRAME	0x08000000U	/* is a control frame */
#define	PSRD0_ASW_DRIBBLE_NIBBLE	0x04000000U	/* spurious bits after EOP */
#define	PSRD0_ASW_BROADCAST	0x02000000U	/* has a broadcast address */
#define	PSRD0_ASW_MULTICAST	0x01000000U	/* has a multicast address */
#define	PSRD0_ASW_OK	0x00800000U	/* valid CRC + no code error */
#define	PSRD0_ASW_TOO_LONG	0x00400000U	/* frame length > 1518 bytes */
#define	PSRD0_ASW_LEN_CHK_ERR	0x00200000U	/* frame length field incorrect */
#define	PSRD0_ASW_CRC_ERR	0x00100000U	/* CRC error */
#define	PSRD0_ASW_CODE_ERR	0x00080000U	/* one or more nibbles signalled as errors */
#define	PSRD0_ASW_FALSE_CARRIER_EVENT	0x00040000U	/* bad carrier since last good packet */
#define	PSRD0_ASW_RX_DV_EVENT	0x00020000U	/* short receive event detected */
#define	PSRD0_ASW_PREV_PKT_DROPPED	0x00010000U	/* IFG too small on previous */
#define	PSRD0_VP	0x00000400U	/* VLAN Packet */
#define	PSRD0_JP	0x00000200U	/* Jumbo Packet */
#define	PSRD0_FT	0x00000100U	/* Frame Truncated */
#define	PSRD0_DROP	0x00000080U	/* Drop packet */
#define	PSRD0_RXMAC_ERROR	0x00000040U	/* RXMAC Error Indicator */
#define	PSRD0_WOL	0x00000020U	/* WOL Event */
#define	PSRD0_TCPP	0x00000010U	/* TCP checksum pass */
#define	PSRD0_TCPA	0x00000008U	/* TCP checksum assist */
#define	PSRD0_IPP	0x00000004U	/* IP checksum pass */
#define	PSRD0_IPA	0x00000002U	/* IP checksum assist */
#define	PSRD0_HP	0x00000001U	/* hash pass */

	volatile uint32_t	psrd1;
#define	PSRD1_RI	0x0c000000U	/* Ring Index */
#define		PSRD1_RI_SHIFT	26	/* Ring Index */
#define	PSRD1_BI	0x03ff0000U	/* Buffer Index */
#define		PSRD1_BI_SHIFT	16
#define	PSRD1_LENGTH	0x0000ffffU	/* length in bytes */
};
/*===========================================================================*/




/******************************************************************************
   Typedefs for the RX DMA status word
 *****************************************************************************/
/******************************************************************************
   RXSTAT_WORD0_t structure holds part of the status bits of the Rx DMA engine
   that get copied out to memory by the ET-1310.  Word 0 is a 32 bit word which
   contains Free Buffer ring 0 and 1 available offset.
 *****************************************************************************/
#ifdef notdef
typedef union _rxstat_word0_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 FBR1unused:5;                        //bits 27-31
        UINT32 FBR1wrap:1;                          //bit 26
        UINT32 FBR1offset:10;                       //bits 16-25
        UINT32 FBR0unused:5;                        //bits 11-15
        UINT32 FBR0wrap:1;                          //bit 10
        UINT32 FBR0offset:10;                       //bits 0-9
    #else
        UINT32 FBR0offset:10;                       //bits 0-9
        UINT32 FBR0wrap:1;                          //bit 10
        UINT32 FBR0unused:5;                        //bits 11-15
        UINT32 FBR1offset:10;                       //bits 16-25
        UINT32 FBR1wrap:1;                          //bit 26
        UINT32 FBR1unused:5;                        //bits 27-31
    #endif
    } bits;
}RXSTAT_WORD0_t, *PRXSTAT_WORD0_t;
#endif

#define	RXSTAT_FBR1_WRAP	0x04000000U
#define	RXSTAT_FBR1_OFFSET	0x03ff0000U
#define		RXSTAT_FBR1_OFFSET_MASK	0x3ffU
#define		RXSTAT_FBR1_OFFSET_SHIFT	16
#define	RXSTAT_FBR0_WRAP	0x00000400U
#define	RXSTAT_FBR0_OFFSET	0x000003ffU
#define		RXSTAT_FBR0_OFFSET_MASK	0x3ffU
#define		RXSTAT_FBR0_OFFSET_SHIFT	0



/******************************************************************************
   RXSTAT_WORD1_t structure holds part of the status bits of the Rx DMA engine
   that get copied out to memory by the ET-1310.  Word 3 is a 32 bit word which
   contains the Packet Status Ring available offset.
 *****************************************************************************/
#ifdef notdef
typedef union _rxstat_word1_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 PSRunused:3;                            //bits 29-31
        UINT32 PSRwrap:1;                              //bit 28
        UINT32 PSRoffset:12;                           //bits 16-27
        UINT32 reserved:16;                            //bits 0-15
    #else
        UINT32 reserved:16;                            //bits 0-15
        UINT32 PSRoffset:12;                           //bits 16-27
        UINT32 PSRwrap:1;                              //bit 28
        UINT32 PSRunused:3;                            //bits 29-31
    #endif
    } bits;
}RXSTAT_WORD1_t, *PRXSTAT_WORD1_t;
#endif

#define	RXSTAT_PSR_WRAP		0x10000000U
#define	RXSTAT_PSR_OFFSET	0x0fff0000U
#define		RXSTAT_PSR_OFFSET_SHIFT	16

/******************************************************************************
   RX_STATUS_BLOCK_t is sructure representing the status of the Rx DMA engine
   it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/
#ifdef notdef
typedef struct _rx_status_block_t 
{
    RXSTAT_WORD0_t    Word0;
    RXSTAT_WORD1_t    Word1;
} 
RX_STATUS_BLOCK_t, *PRX_STATUS_BLOCK_t;
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ET1310REG_H__ */
