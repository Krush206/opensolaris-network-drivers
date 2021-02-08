/*
 * %W% %E%
 * Macro definitions for ASIX AX88178a USB to fast ethernet controler.
 * This file is public domain. Coded by M.Murayama (KHF04453@nifty.com)
 */

#ifndef	_AX88178A_REG_
#define	_AX88178A_REG_

/*
 * Vendor command difinitions
 */

/* Access space IDs */
#define	ACCESS_MAC		0x01U
#define	ACCESS_PHY		0x02U
#define	ACCESS_WAKEUP		0x03U
#define	ACCESS_EEPROM		0x04U
#define	ACCESS_EFUSE		0x05U
#define	ACCESS_EEPROM_EFUSE	0x06U
#define	AXCESS_EFUSE_WR_EN	0x09U
#define	AXCESS_EFUSE_WR_DIS	0x0aU
#define	ACCESS_MFAB		0x10U

/* register offset in ACCESS_MAC */
#define	LINKSTATUS		0x02U
#define	GENERALSTATUS		0x03U
#define	SROMADDR		0x07U
#define	SROMDATA_LOW		0x08U
#define	SROMDATA_HIGH		0x09U
#define	SROMCMD			0x0aU
#define RXCTL			0x0bU
#define NODEID			0x10U
#define MCASTFILTER_ARRY	0x16U
#define MEDIUM			0x22U
#define MONITOR			0x24U
#define GPIO			0x25U
#define PHYPWR			0x26U
#define BULKINQCTRL		0x2eU
#define	BULKIN_QTIMR_LOW	0x2fU
#define BULKIN_QTIMR_HIGH	0x30U
#define BULKIN_QSIZE		0x31U
#define BULKIN_QIFG		0x32U
#define CLKSELECT		0x33U
#define RXCOE			0x34U
#define TXCOE			0x35U
#define PAUSE_HIGH		0x54U
#define PAUSE_LOW		0x55U


/* LINKSTATUS: physical link status register */
#define	LINKSTATUS_USB_SS	0x04U	/* USB 3.x */
#define	LINKSTATUS_USB_HS	0x02U	/* USB 2.x */
#define	LINKSTATUS_USB_FS	0x01U	/* USB 1.x */

/* GENERALSTATUS: general status register */
/* Check AX88179 version. UA1:Bit2 = 0,  UA2:Bit2 = 1 */
#define	GENERALSTATUS_SECLD	0x04U


/* SROMCMD: srom command register */
#define	SROMCMD_EEP_BUSY	0x10U	/* EEprom access module busy */
#define	SROMCMD_EEP_WR		0x08U	/* EEprom write command */
#define	SROMCMD_EEP_RD		0x04U	/* EEprom read command */


/* RXCTL: receive control register */
#define RXCTL_TXPADCRC		0x0400U	/* checksum value in rx header 3 */
#define RXCTL_IPE		0x0200U	/* Enable IP header in receive buffer aligned on 32-bit aligment */
#define RXCTL_DROPCRCERR	0x0100U	/* Drop CRC error packet */
#define RXCTL_START		0x0080U	/* Ethernet MAC start */
#define RXCTL_AP		0x0020U	/* Accept physcial address from Multicast array */
#define RXCTL_AM		0x0010U	/* Accept multicast address from Multicast array */
#define RXCTL_AB		0x0008U	/* Accetp Brocadcast frames */
#define RXCTL_HA8B		0x0004U	/* HW auto-added 8-bytes data */
					/* when meet USB bulk in transfer boundary (1024/512/64) */
#define RXCTL_AMALL		0x0002U	/* Accetp all multicast frames */
#define RXCTL_PRO		0x0001U	/* Promiscuous Mode */

#define	RXCTL_BITS \
	"\020" \
	"\013TXPADCRC" \
	"\012IPE" \
	"\011DROPCRCERR" \
	"\010START" \
	"\006AP" \
	"\005AM" \
	"\004AB" \
	"\003HA8B" \
	"\002AMALL" \
	"\001PRO"

/* MCASTFILTER: multicast filter array */
#define MCASTFILTER_SIZE	8

/* MEDIUM: media status mode register */
#define MEDIUM_JUMBO_EN		0x8040U
#define MEDIUM_RECEIVE_EN	0x0100U
#define MEDIUM_PS		0x0200U	/* 100MHz mode */
#define MEDIUM_TXFLOW_CTRLEN	0x0020U
#define MEDIUM_RXFLOW_CTRLEN	0x0010U
#define MEDIUM_EN_125MHZ	0x0008U	/* for 1GHz mode */
#define MEDIUM_FULL_DUPLEX	0x0002U
#define MEDIUM_GIGAMODE		0x0001U	/* 1GHz mode */

#define	MEDIUM_BITS \
	"\020" \
	"\012PS" \
	"\011RECEIVE_EN" \
	"\006TXFLOW_CTRLEN" \
	"\005RXFLOW_CTRLEN" \
	"\004EN_125MHZ" \
	"\002FULL_DUPLEX" \
	"\001GIGAMODE"

/* MONITOR: monitor mode register */
#define MONITOR_PMETYPE		0x40U
#define MONITOR_PMEPOL		0x20U
#define MONITOR_RW_FLAG		0x10U
#define MONITOR_RWWF		0x08U
#define MONITOR_RWMP		0x04U
#define MONITOR_RWLC		0x02U

/* GPIO: gpio configuration register */
#define GPIO_GPIO3EN		0x80U
#define GPIO_GPIO2EN		0x40U
#define GPIO_GPIO1EN		0x20U

/* PHYPWR: phy power reset control register */
#define	PHYPWR_AUTODETACH	0x1000U
#define	PHYPWR_IPRL		0x0020U
#define	PHYPWR_BZ		0x0010U

/* BULKINQCTL: rx bulkin queue control register */
#define BULKINQCTL_SIZE		0x04U	/* use buffer size */
#define BULKINQCTL_IFG		0x02U	/* use inter frame gap */
#define BULKINQCTL_TIME		0x01U	/* use timer */

/* CLKSELECT: clock select register */
#define CLKSELECT_ACSREQ	0x10U
#define CLKSELECT_ULR		0x08U
#define CLKSELECT_ACS		0x02U
#define CLKSELECT_BCS		0x01U

/* RXCOE: rx offloading register */
#define RXCOE_ICMPV6		0x80U
#define RXCOE_UDPV6		0x40U
#define RXCOE_TCPV6		0x20U
#define RXCOE_IGMP		0x10U
#define RXCOE_ICMP		0x08U
#define RXCOE_UDP		0x04U
#define RXCOE_TCP		0x02U
#define RXCOE_IP		0x01U

/* TXCOE: tx offloading register */
#define TXCOE_ICMV6		0x80U
#define TXCOE_UDPV6		0x40U
#define TXCOE_TCPV6		0x20U
#define TXCOE_IGMP		0x10U
#define TXCOE_ICMP		0x08U
#define TXCOE_UDP		0x04U
#define TXCOE_TCP		0x02U
#define TXCOE_IP		0x01U


/* misc configuration for Ethernet PHY*/
#define AX88179_PHY_ID			0x03

/* misc configuration for EEPROM */
#define AX_EEPROM_LEN			0x40
#define AX_EEP_EFUSE_CORRECT		0x00
#define AX88179_EEPROM_MAGIC		0x17900b95U


/*****************************************************************************/
/* GMII register definitions */

#define GMII_PHY_MACR		0x0dU
#define GMII_PHY_MAADR		0x0eU
#define GMII_PHY_PHYSR		0x11U	/* PHY specific status register */
#define	GMII_PHY_PHYSR_SMASK		0xc000
#define	GMII_PHY_PHYSR_GIGA		0x8000
#define	GMII_PHY_PHYSR_100		0x4000
#define	GMII_PHY_PHYSR_FULL		0x2000
#define	GMII_PHY_PHYSR_LINK		0x400

#define GMII_PHYPAGE		0x1eU

#define GMII_PHY_PAGE_SELECT	0x1fU

#define	GMII_PHY_PAGE_SELECT_EXT	0x0007
#define	GMII_PHY_PAGE_SELECT_PAGE(x)	((x) & GMII_PHY_PAGE_SELECT_EXT)

/* Cicada MII Registers */

#define GMII_LED_ACTIVE			0x1a
#define	GMII_LED0_ACTIVE		(1U << 4)
#define	GMII_LED1_ACTIVE		(1U << 5)
#define	GMII_LED2_ACTIVE		(1U << 6)

#define GMII_LED_LINK			0x1c
#define GMII_LED0_LINK_10		(1U << 0)
#define GMII_LED0_LINK_100		(1U << 1)
#define GMII_LED0_LINK_1000		(1U << 2)
#define GMII_LED1_LINK_10		(1U << 4)
#define GMII_LED1_LINK_100		(1U << 5)
#define GMII_LED1_LINK_1000		(1U << 6)
#define GMII_LED2_LINK_10		(1U << 8)
#define GMII_LED2_LINK_100		(1U << 9)
#define GMII_LED2_LINK_1000		(1U << 10)

#define	LED_VALID	(1 << 15) /* UA2 LED Setting */

#define	LED0_ACTIVE	(1 << 0)
#define	LED0_LINK_10	(1 << 1)
#define	LED0_LINK_100	(1 << 2)
#define	LED0_LINK_1000	(1 << 3)
#define	LED0_FD		(1 << 4)
#define LED0_USB3_MASK	\
    (LED0_ACTIVE | LED0_LINK_10 | LED0_LINK_100 | LED0_LINK_1000 | LED0_FD)

#define	LED1_ACTIVE	(1 << 5)
#define	LED1_LINK_10	(1 << 6)
#define	LED1_LINK_100	(1 << 7)
#define	LED1_LINK_1000	(1 << 8)
#define	LED1_FD		(1 << 9)
#define LED1_USB3_MASK	\
    (LED1_ACTIVE | LED1_LINK_10 | LED1_LINK_100 | LED1_LINK_1000 | LED1_FD)

#define	LED2_ACTIVE	(1 << 10)
#define	LED2_LINK_10	(1 << 11)
#define	LED2_LINK_100	(1 << 12)
#define	LED2_LINK_1000	(1 << 13)
#define	LED2_FD		(1 << 14)
#define LED2_USB3_MASK	\
    (LED2_ACTIVE | LED2_LINK_10 | LED2_LINK_100 | LED2_LINK_1000 | LED2_FD)



/******************************************************************************/

/* interrupt messages */
#define INT_PPLS_LINK		(1 << 0)
#define INT_SPLS_LINK		(1 << 1)
#define INT_CABOFF_UNPLUG	(1 << 7)

/* rx packet descriptor */

/* byte 0 */
#define RXDESC_L4CSUM_ERR	0x00000001U
#define RXDESC_L3CSUM_ERR	0x00000002U

#define RXDESC_L4_TYPE_MASK	0x0000001cU
#define	RXDESC_L4_TYPE_SHIFT	2
#define		L4_TYPE_UDP	1
#define		L4_TYPE_ICMP	2
#define		L4_TYPE_IGMP	3
#define		L4_TYPE_TCP	4
#define		L4_TYPE_ICMPv6	5

#define RXDESC_L3_TYPE_MASK	0x00000060U
#define RXDESC_L3_TYPE_SHIFT	5
#define		L3_TYPE_IP	1
#define		L3_TYPE_IPv6	2

#define RXDESC_CE		0x00000080U

/* byte 1 */
#define RXDESC_VLAN_IND		0x00000700U
#define	RXDESC_VLAN_IND_SHIFT	8
#define RXDESC_RXOK		0x00000800U
#define RXDESC_PRI		0x00007000U
#define	RXDESC_PRI_SHIFT		12
#define RXDESC_BMC		0x00008000U

/* byte 2,3 */
#define RXDESC_LEN		0x1fff0000U
#define 	RXDESC_LEN_SHIFT	16
#define RXDESC_CRC_ERR		0x20000000U
#define RXDESC_MII_ERR		0x40000000U
#define RXDESC_DROP_ERR		0x80000000U

#define RXDESC_BITS	\
	"\020"	\
	"\040DROPER"	\
	"\037MIIER"	\
	"\036CRCER"	\
	"\020BMC"	\
	"\014RXOK"	\
	"\010CE"	\
	"\002L4CSUMER"	\
	"\001L3CSUMER"

#endif

