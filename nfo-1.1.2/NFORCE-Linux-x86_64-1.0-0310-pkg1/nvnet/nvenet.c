/****************************************************************** \
|*                                                                 *|
|*                                                                 *|
|*  (c) NVIDIA Corporation. All rights reserved                    *| 
|*                                                                 *|
|*  THE INFORMATION CONTAINED HEREIN IS PROPRIETARY AND            *|
|*  CONFIDENTIAL                                                   *|
|*  TO NVIDIA, CORPORATION. USE, REPORDUCTION OR DISCLOSURE TO ANY *|
|*  THIRD PARTY IS SUBJECT TO WRITTEN PRE-APPROVAL BY NVIDIA CORP. *|
|*                                                                 *|
|*  THE INFORMATION CONTAINED HEREIN IS PROVIDED "AS IS" WITHOUT   *|
|*  EXPRESS OR IMPLIED WARRANTY OF ANY KIND, INCLUDING ALL IMPLIED *|
|*  WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS    *|
|*  FOR A PARTICULAR PURPOSE.                                      *|
|*                                                                 *|
********************************************************************/

#include "basetype.h"
#include "os.h"
#include "drvinfo.h"
#include "adapter.h"
#include "nvenet.h"
#include "nvenet_version.h"

/*
 * Driver information
 */ 

#define DRIVER_NAME           "nvnet"
#define DRIVER_RELEASE_DATE   "July 1, 2004"

MODULE_AUTHOR("NVIDIA (linux-nforce-bugs@nvidia.com)");
MODULE_DESCRIPTION("NVIDIA Corporation NVNET Ethernet Driver Version "
                   DRIVER_VERSION " " DRIVER_RELEASE_DATE);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,10)
MODULE_LICENSE("NVIDIA");
#endif

static struct proc_dir_entry *nvenet_proc_root = NULL;

/*********************************************************************
                    Module parameters
*********************************************************************/

/*
 * Force speed duplex
 */

enum nvenet_force_speed_duplex_settings 
{ 
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE,
    NVENET_FORCE_SPEED_DUPLEX_FORCE_10_HALF_DUPLEX,
    NVENET_FORCE_SPEED_DUPLEX_FORCE_10_FULL_DUPLEX,
    NVENET_FORCE_SPEED_DUPLEX_FORCE_100_HALF_DUPLEX,
    NVENET_FORCE_SPEED_DUPLEX_FORCE_100_FULL_DUPLEX,
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_HD,
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_FD,
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100_HD,
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100FD,
    NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_1000FD

};

static int force_speed_duplex  = -1;
static char *nvenet_force_speed_duplex_str[] = {
    "autonegote",
    "10 Mbps half duplex",
    "10 Mbps full duplex",
    "100 Mpbs half duplex",
    "100 Mbs full duplex",
    "autonegotiate for 10 half duplex",
    "autonegotiate for 10 full duplex",
    "autonegotiate for 100 half duplex",
    "autonegotiate for 100 full duplex",
    "autonegotiate for 1000 full duplex"
};

/*
 * Autonegotiation mode
 */
enum nvenet_auto_negotiate_settings 
{ 
    NVENET_NO_AUTO_NEGOTIATE, 
    NVENET_AUTO_NEGOTIATE
};

static char *nvenet_auto_negotiate_str[] = 
{ 
    "disabled", 
    "enabled" 
};

static int auto_negotiate = -1;

/*
 * Poll interval
 */
static int poll_interval_in_us = -1;

/*
 * HWMode settings
 *      1 - Legacy
 *      2 - Non Legacy
 */
enum nvenet_hwmode_settings {
    NVENET_HW_MODE_1   = 1, 
    NVENET_HW_MODE_2
};

static int hwmode = -1;


/*
 * Segmentation offload
 */

enum nvenet_seg_offload_settings 
{ 
    NVENET_SEG_OFFLOAD_DISABLED, 
    NVENET_SEG_OFFLOAD_ENABLED 
};

static char *nvenet_seg_offload_str[] = 
{ 
    "disabled", 
    "enabled" 
};

static int seg_offload = -1;

/*
 * MTU settings
 */
static int mtu = -1;

/*
 * Media settings
 */
enum nvenet_media_settings 
{ 
    NVENET_MEDIA_AUTO,
    NVENET_MEDIA_RGMII, 
    NVENET_MEDIA_MII
};

static char *nvenet_media_str[] = 
{ 
    "auto",
    "rgmii", 
    "mii" 
};

static int media = -1;

/*
 * Tx checksum offload
 */
enum nvenet_tx_checksum_offload_settings 
{
    NVENET_TX_CHECKSUM_DISABLED, 
    NVENET_TX_CHECKSUM_ENABLED 
};

static char *nvenet_tx_checksum_offload_str[] =  
{ 
    "disabled", 
    "enabled" 
};

static int tx_checksum_offload = -1;

/*
 * Rx checksum offload
 */
enum nvenet_rx_checksum_offload_settings 
{ 
    NVENET_RX_CHECKSUM_DISABLED, 
    NVENET_RX_CHECKSUM_ENABLED 
};

static char *nvenet_rx_checksum_offload_str[] = 
{ 
    "disabled" , 
    "enabled" 
};

static int rx_checksum_offload = -1;

/*
 * Optimization mode
 */
enum optimization_settings 
{
    NVENET_OPTIMIZATION_THROUGHPUT, 
    NVENET_OPTIMIZATION_CPU
};

static char *nvenet_optimization_str[] = 
{ 
    "Throughput", 
    "CPU" 
};
static int optimization = -1;


/*
 * Maximum rx packets
 */
static int max_tx_packets = -1;
static int max_rx_packets = -1;


/*
 * Module parameters
 */

MODULE_PARM(auto_negotiate, "i");
MODULE_PARM_DESC(auto_negotiate, "Control autonegotiation by setting 0 for disable and 1 for enable.");

MODULE_PARM(force_speed_duplex, "i");
MODULE_PARM_DESC(force_speed_duplex, "Control speed and duplex by setting appropriate value . See README for details of this feature.");

MODULE_PARM(max_tx_packets, "i");
MODULE_PARM_DESC(max_tx_packets, "Control max number of outstanding tx packets by setting a value between 32 and 1024.");

MODULE_PARM(max_rx_packets, "i");
MODULE_PARM_DESC(max_rx_packets, "Control max number of rx packets by setting a value between 32 and 1024.");

MODULE_PARM(optimization, "i");
MODULE_PARM_DESC(optimization, "Control optimization by setting value 0 for optimizing throughput and 1 for optimizing CPU.");

MODULE_PARM(poll_interval_in_us, "i");
MODULE_PARM_DESC(poll_interval_in_us, "Control poll interval when optimization is CPU by setting a value between 0 and 2000.");

MODULE_PARM(hwmode, "i");
MODULE_PARM_DESC(hwmode, "Control hwmode by setting 1 for hardware mode 1 and 2 for hardware mode 2. See README for details.");

MODULE_PARM(seg_offload, "i");
MODULE_PARM_DESC(seg_offload, "Control segmentation offload by setting 0 for disable and 1 for enable. This parameter is allowed in hardware mode 2 only.");

MODULE_PARM(mtu, "i");
MODULE_PARM_DESC(mtu, "Control mtu by setting value between 576 and 9202. This parameter is allowed in hardware mode 2 only.");

MODULE_PARM(media, "i");
MODULE_PARM_DESC(media, "Control media interface by setting value 0 for Auto, 1 for RGMII, and 2 for MII. This parameter is allowed in hardware mode 2 only.");

MODULE_PARM(tx_checksum_offload, "i");
MODULE_PARM_DESC(tx_checksum_offload, "Control tx checksum offload by setting value 0 for disable and 1 for enable. This parameter is allowed in hardware mode 2 only.");

MODULE_PARM(rx_checksum_offload, "i");
MODULE_PARM_DESC(rx_checksum_offload, "Control rx checksum offload by setting value 0 for disable and 1 for enable. This parameter is allowed in hardware mode 2 only.");


struct nvenet_parameter nvenet_mode_1_parameters[] = 
{
    {"force_speed_duplex", TRUE, &force_speed_duplex, NVENET_OFFSET(force_speed_duplex), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE, NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100FD, NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE, nvenet_force_speed_duplex_str },

    {"auto_negotiate", TRUE, &auto_negotiate, NVENET_OFFSET(auto_negotiate), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_NO_AUTO_NEGOTIATE, NVENET_AUTO_NEGOTIATE, NVENET_AUTO_NEGOTIATE, nvenet_auto_negotiate_str },

    {"optimization", TRUE, &optimization, NVENET_OFFSET(optimization), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_OPTIMIZATION_THROUGHPUT, NVENET_OPTIMIZATION_CPU, NVENET_OPTIMIZATION_THROUGHPUT, nvenet_optimization_str},

    {"max_tx_packets", TRUE, &max_tx_packets, NVENET_OFFSET(max_tx_packets), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), NVENET_MIN_TX_PACKETS, NVENET_MAX_TX_PACKETS, NVENET_DEFAULT_TX_PACKETS, NULL },

    {"max_rx_packets", TRUE, &max_rx_packets, NVENET_OFFSET(max_rx_packets), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), NVENET_MIN_RX_PACKETS, NVENET_MAX_RX_PACKETS, NVENET_DEFAULT_RX_PACKETS, NULL },

    {"poll_interval_in_us", TRUE, &poll_interval_in_us, NVENET_OFFSET(poll_interval_in_us), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), 0, 2000, 0, NULL },

    {"seg_offload", FALSE, &seg_offload, NVENET_OFFSET(seg_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_SEG_OFFLOAD_DISABLED, NVENET_SEG_OFFLOAD_ENABLED, NVENET_SEG_OFFLOAD_DISABLED, nvenet_seg_offload_str},

    {"mtu", FALSE, &mtu, NVENET_OFFSET(mtu), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), MIN_PACKET_MTU_SIZE, MAX_PACKET_MTU_SIZE, 1500, NULL},

    {"media", FALSE, &media, NVENET_OFFSET(media), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_MEDIA_MII, NVENET_MEDIA_RGMII, NVENET_MEDIA_MII, nvenet_media_str},

    {"tx_checksum_offload", FALSE, &tx_checksum_offload, NVENET_OFFSET(tx_checksum_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_TX_CHECKSUM_DISABLED, NVENET_TX_CHECKSUM_ENABLED, NVENET_TX_CHECKSUM_DISABLED, nvenet_tx_checksum_offload_str},

    {"rx_checksum_offload", FALSE, &rx_checksum_offload, NVENET_OFFSET(rx_checksum_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_RX_CHECKSUM_DISABLED, NVENET_RX_CHECKSUM_ENABLED, NVENET_RX_CHECKSUM_DISABLED, nvenet_rx_checksum_offload_str},

};

struct nvenet_parameter nvenet_mode_2_parameters[] = 
{
    {"force_speed_duplex", TRUE, &force_speed_duplex, NVENET_OFFSET(force_speed_duplex), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE, NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100FD, NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE, nvenet_force_speed_duplex_str },

    {"auto_negotiate", TRUE, &auto_negotiate, NVENET_OFFSET(auto_negotiate), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_NO_AUTO_NEGOTIATE, NVENET_AUTO_NEGOTIATE, NVENET_AUTO_NEGOTIATE, nvenet_auto_negotiate_str },

    {"optimization", TRUE, &optimization, NVENET_OFFSET(optimization), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_OPTIMIZATION_THROUGHPUT, NVENET_OPTIMIZATION_CPU, NVENET_OPTIMIZATION_THROUGHPUT, nvenet_optimization_str},

    {"max_tx_packets", TRUE, &max_tx_packets, NVENET_OFFSET(max_tx_packets), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), NVENET_MIN_TX_PACKETS, NVENET_MAX_TX_PACKETS, NVENET_DEFAULT_TX_PACKETS, NULL },

    {"max_rx_packets", TRUE, &max_rx_packets, NVENET_OFFSET(max_rx_packets), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), NVENET_MIN_RX_PACKETS, NVENET_MAX_RX_PACKETS, NVENET_DEFAULT_RX_PACKETS, NULL },

    {"poll_interval_in_us", TRUE, &poll_interval_in_us, NVENET_OFFSET(poll_interval_in_us), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), 0, 2000, 0, NULL },

    {"seg_offload", TRUE, &seg_offload, NVENET_OFFSET(seg_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_SEG_OFFLOAD_DISABLED, NVENET_SEG_OFFLOAD_ENABLED, NVENET_SEG_OFFLOAD_ENABLED, nvenet_seg_offload_str},

    {"mtu", TRUE, &mtu, NVENET_OFFSET(mtu), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), MIN_PACKET_MTU_SIZE, MAX_PACKET_MTU_SIZE, 1500, NULL},

    {"media", TRUE, &media, NVENET_OFFSET(media), NVENET_PARAM_TYPE_VALUE, sizeof(NV_UINT32), NVENET_MEDIA_MII, NVENET_MEDIA_RGMII, NVENET_MEDIA_RGMII, nvenet_media_str},

    {"tx_checksum_offload", TRUE, &tx_checksum_offload, NVENET_OFFSET(tx_checksum_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_TX_CHECKSUM_DISABLED, NVENET_TX_CHECKSUM_ENABLED, NVENET_TX_CHECKSUM_ENABLED, nvenet_tx_checksum_offload_str},

    {"rx_checksum_offload", TRUE, &rx_checksum_offload, NVENET_OFFSET(rx_checksum_offload), NVENET_PARAM_TYPE_INDEX, sizeof(NV_UINT32), NVENET_RX_CHECKSUM_DISABLED, NVENET_RX_CHECKSUM_ENABLED, NVENET_RX_CHECKSUM_ENABLED, nvenet_rx_checksum_offload_str},

};

/* 
 * Table with PCI device identification
 */
static struct pci_device_id
nvenet_pci_table[]  __devinitdata = {
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_1, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_2, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_3, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_4, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_5, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_6, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_7, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_8, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_9, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_10, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_11, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_12, PCI_ANY_ID, PCI_ANY_ID},
    {PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NVENET_13, PCI_ANY_ID, PCI_ANY_ID},
    {0, 0, 0, 0}
};

MODULE_DEVICE_TABLE(pci, nvenet_pci_table);

static struct pci_driver nvenet_driver = 
{
    name:     "nvnet",
    id_table: nvenet_pci_table,
    probe:    nvenet_probe,
    remove:   nvenet_remove,
#ifdef CONFIG_PM
    suspend:  nvenet_suspend,
    resume:   nvenet_resume,
#endif

};

/*
 * Reboot notification
 */
struct notifier_block nvenet_reboot_notifier = 
{
    notifier_call   : nvenet_reboot_handler,
    next            : NULL,
    priority        : 0
};

static int
nvenet_reboot_handler(struct notifier_block *nb, unsigned long event, void *p)
{
#ifdef CONFIG_PM
    struct pci_dev *pdev = NULL;
    
    PRINTK(NV_DBGLEVEL_REBOOT, "nvenet: nvenet_reboot_notification - In\n");
    
    switch (event)
    {
    case SYS_POWER_OFF:
    case SYS_HALT:
    case SYS_DOWN:
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 0)
        while ((pdev = pci_get_device(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID, pdev)) != NULL) {
#else
	while ((pdev = pci_find_device(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID, pdev)) != NULL) {
#endif
	    if (pci_dev_driver(pdev) == &nvenet_driver) {
	        nvenet_suspend(pdev, 3);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 0)
	        pci_dev_put(pdev);
#endif
	        break;
	    }
	}
    }
    
    PRINTK(NV_DBGLEVEL_REBOOT, "nvenet: nvenet_reboot_notification - Out\n");
#endif
    return NOTIFY_DONE;

}

/*
 * PROC file system structures
 */
static struct file_operations nvenet_proc_full_info_fops =
{
    open        : nvenet_proc_full_info_open,    
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};

static struct file_operations nvenet_proc_hardware_info_fops =
{
    open        : nvenet_proc_hardware_info_open,    
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};

static struct file_operations nvenet_proc_configuration_fops =
{
    open        : nvenet_proc_configuration_open,    
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};

static struct file_operations nvenet_proc_tx_stats_fops  =
{
    open        : nvenet_proc_tx_stats_open,    
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};

static struct file_operations nvenet_proc_rx_stats_fops =
{
    open        : nvenet_proc_rx_stats_open,    
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};

static struct file_operations nvenet_proc_command_line_fops = 
{
    open        : nvenet_proc_command_line_open,
    read        : nvenet_proc_read,
    release     : nvenet_proc_release
};


static int __devinit
nvenet_probe( struct pci_dev *pdev, const struct pci_device_id *entry)
{
    struct nvenet_private *priv = NULL; /* pointer to dev's private data */
    struct net_device *dev      = NULL; /* pointer to network driver struct */
    char  *memptr               = NULL; /* pointer to devices hardware memory */
    int status = 0;
    int resources = 0;
    NV_UINT16 command;
    int i;
    

    PRINTK(NV_DBGLEVEL_PROBE, "nvnet: nvenet_probe - In\n");

    do
    {
        /*************************************************************
                    Set PCI capabilities
        *************************************************************/
        if (pci_set_dma_mask(pdev, 0xffffffff)) 
        {
            printk(KERN_EMERG "nvnet: nvenet_probe- pci_set_dma_mask failed\n");
            PRINTK(NV_DBGLEVEL_PROBE, "nvenet_probe - Out \n");
            status = -EIO;
            break;
        }

        if (pci_enable_device (pdev)) 
        {
            printk(KERN_EMERG "nvnet: nvenet_probe - pci_enable_device failed\n");
            PRINTK(NV_DBGLEVEL_PROBE, "nvnet: nvenet_probe - Out \n");
            status = -EIO;
            break;
        }
        resources |= NVENET_PCI_ENABLE_DONE;

        pci_set_master(pdev);

        /* 
         * Check the status of the net_device and turn it on if necessary
         */
        pci_read_config_word(pdev, PCI_COMMAND, &command);
    
        if ((!(command & PCI_COMMAND_MASTER)) || 
            (!(command & PCI_COMMAND_MEMORY)))
        {
            printk(KERN_INFO "nvnet: Setting bus master enable and memory map access\n");
            /*
             * Enable bus master and memory mapped access
             */
            pci_write_config_word(
                pdev, 
                PCI_COMMAND,
                command | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY
                );
        }

        /*************************************************************
                    Allocate resources
        *************************************************************/

        dev = alloc_etherdev(sizeof(struct nvenet_private));
        if (!dev) 
        {
            printk(KERN_EMERG "nvnet: alloc_etherdev failed\n");
            status = -ENOMEM;
            break;
        }
        resources |= NVENET_ALLOC_ETHERDEV_DONE;

        priv                = (struct nvenet_private *)dev->priv;
        priv->pdev          = pdev; 
        dev->base_addr      = pci_resource_start(pdev,0);
        dev->irq            = pdev->irq;
        pci_set_drvdata(pdev, dev);
        SET_MODULE_OWNER(dev);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 5, 0)
	SET_NETDEV_DEV(dev, &pdev->dev);
#endif

        read_module_parameters(dev);

        spin_lock_init(&priv->lock); 

        if(!request_mem_region(
                    pci_resource_start(pdev, 0),
                    pci_resource_len(pdev, 0), 
                    "nvenet"))  
        {
            printk(KERN_EMERG"nvnet: request_mem_region failed\n");
            status = -EIO;
            break;
        }
        resources |= NVENET_REQUEST_MEM_REGION_DONE;

        memptr = ioremap(
                        pci_resource_start(pdev, 0), 
                        pci_resource_len(pdev, 0));

        if(memptr == NULL)
        {
            printk(KERN_EMERG "nvnet: ioremap failed \n");
            status = -ENOMEM;
            break;
        }    
        resources      |= NVENET_IOREMAP_DONE;
        priv->regs      = (char *)memptr;


        /*
         * Allocate TX structures
         */
        priv->tx_ring_full      = 0;
        priv->tx_ring_size      = priv->max_tx_packets * sizeof(nvenet_tx_info_os_t);
        priv->tx_ring_index     = 0;
        priv->tx_ring_busy_count     = 0;
        /*
         * Allocate space for the tx ring
         */
        priv->tx_ring = (nvenet_tx_info_os_t *) pci_alloc_consistent(
                                                        priv->pdev, 
                                                        priv->tx_ring_size, 
                                                        &priv->tx_ring_dma);

        if (!priv->tx_ring) 
        {
            printk(KERN_EMERG "nvenet_proble: pci_alloc_consistent failed\n");
            status = -ENOMEM;
            break;
        }
        resources |= NVENET_TX_RING_ALLOCATED;


        dev->mtu = priv->mtu;

        if (create_adapter_object(dev) != NVENET_STATUS_SUCCESS)
        {
            printk(KERN_EMERG "ERROR: nvenet_probe, unable to create adapter object\n");
            status = -ENOMEM;
            break;
        }
        resources |= NVENET_ADAPTER_OBJECT_CREATED;

        /* 
         * Mac address is loaded at boot time into h/w reg, but it's loaded backwards
         * we get the address, save it, and reverse it. The saved value is loaded
         * back into address at close time.
         */
        PRINTK(NV_DBGLEVEL_INIT, "nvnet: nvenet_probe - get mac address\n");

        priv->pADApi->pfnGetNodeAddress( priv->pADApi->pADCX, priv->permanent_address);

        for (i=0; i < 6; i++)
        {
            dev->dev_addr[i] = priv->reversed_address[i] = priv->permanent_address[6 - 1 - i];
        }

        /*
         * Tell hardware what the ethernet address is
         */
        PRINTK(NV_DBGLEVEL_INIT, "nvnet: nvenet_probe - setting mac address\n");

        priv->pADApi->pfnSetNodeAddress(priv->pADApi->pADCX, priv->reversed_address);    

        /*************************************************************
                Set ethernet object
        **************************************************************/

        dev->open               = nvenet_open;
        dev->stop               = nvenet_close;
        dev->hard_start_xmit    = nvenet_xmit;
        dev->do_ioctl           = nvenet_ioctl;
        dev->get_stats          = nvenet_stats;
        dev->set_multicast_list = nvenet_multicast;
        dev->change_mtu         = nvenet_change_mtu;
        dev->set_mac_address    = nvenet_mac_address;
#if 0
        dev->tx_timeout         = nvenet_watchdog;
        dev->watchdog_timeo     = 1 * HZ; 
#endif
        if (NVENET_HW_MODE_2 == priv->hwmode)
        {
            if (priv->tx_checksum_offload)
            {
                dev->features = (NETIF_F_SG | NETIF_F_HW_CSUM);
            } 
#ifdef NETIF_F_TSO
            if (priv->seg_offload)
            {
                dev->features |= NETIF_F_TSO;
            }
#endif
        }

        if (register_netdev(dev))
        {
            printk(KERN_EMERG "nvnet: register_netdev failed\n");
            status = -ENODEV;
            break;
        }

    } while(0);

    /* 
     * Check status
     */
    if (status == 0)
    {
        create_nvenet_proc_entries(dev);

    }
    else
    {
        if (resources & NVENET_ADAPTER_OBJECT_CREATED)
        {

            delete_adapter_object(priv->pADApi);
        }

        if (resources & NVENET_TX_RING_ALLOCATED)
        {

            pci_free_consistent(
                    priv->pdev, 
                    priv->tx_ring_size,
                    priv->tx_ring, 
                    priv->tx_ring_dma);
        }

        if (resources & NVENET_IOREMAP_DONE)
        {
            iounmap(memptr);
        }

        if (resources & NVENET_REQUEST_MEM_REGION_DONE)
        {
            release_mem_region( 
                    pci_resource_start(pdev, 0), 
                    pci_resource_len(pdev, 0));
        }

        if (resources & NVENET_PCI_ENABLE_DONE)
        {
            pci_disable_device(pdev);
        }

        if (resources & NVENET_ALLOC_ETHERDEV_DONE)
        {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
            kfree(dev);
#else
            free_netdev(dev);
#endif
        }
    }

    PRINTK(NV_DBGLEVEL_PROBE, "nvnet: nvenet_probe - Out with status 0x%x\n", status);
    return status;
}


static int
nvenet_open(struct net_device *dev)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    int status;

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_open - In\n");

    priv->linkup                    = TRUE;
	priv->async_mode				= FALSE;

    /*
     * Initialize hardware
     */
    status = priv->pADApi->pfnInit(
                    priv->pADApi->pADCX, 
                    priv->speed,  /* force speed */ 
                    priv->force_duplex, /* force full duplex */
                    priv->force_mode,   /* force mode */
                    priv->async_mode,	/* async mode */
                    &priv->linkup
                    );

    if(status != ADAPTERERR_NONE) 
    {
        PRINTK_ERROR("nvnet: nvenet_open_adapter - initialize failed\n");
        PRINTK(NV_DBGLEVEL_INIT, "nvnet: nvenet_open -  Out\n");
        return -EAGAIN;
    }

    /* Set the node address */
    priv->pADApi->pfnSetNodeAddress(priv->pADApi->pADCX, priv->reversed_address);

    /*
     * Set speed and duplex settings on the hardware
     */
    priv->pADApi->pfnSetSpeedDuplex(priv->pADApi->pADCX);
    
    /*
     * Initialize tx ring indexes
     */
    priv->tx_ring_full          = 0;
    priv->tx_ring_index         = 0;
    priv->tx_ring_busy_count    = 0;

    if(request_irq(dev->irq, &nvenet_interrupt, SA_SHIRQ, dev->name, dev)) 
    {

        printk(KERN_EMERG "nvnet: nvenet_open - request_irq failed\n");
        PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_open -  Out\n");
        return -EAGAIN;
    }

    /*
     * Turn on hardware
     */
    priv->pADApi->pfnStart(priv->pADApi->pADCX);

    netif_start_queue(dev);
    if (priv->linkup == BMSR_LSTATUS)
        netif_carrier_on(dev);
    else
        netif_carrier_off(dev);

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_open - Out\n");
    return 0;
}

static struct net_device_stats *
nvenet_stats(struct net_device *dev)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    ADAPTER_API *pADApi;
    ADAPTER_STATS stats;

    PRINTK(NV_DBGLEVEL_STATS, "nvnet: nvenet_stats - In\n");
    pADApi = priv->pADApi;
    if (pADApi) 
    {    
        pADApi->pfnGetStatistics(pADApi->pADCX, &stats);
        priv->stats.rx_packets = stats.ulSuccessfulReceptions;
        priv->stats.tx_packets = stats.ulSuccessfulTransmissions;
        priv->stats.rx_errors  = stats.ulMissedFrames +
                                 stats.ulFailedReceptions +
                                 stats.ulCRCErrors +
                                 stats.ulFramingErrors +
                                 stats.ulOverFlowErrors ;

        priv->stats.tx_errors  = stats.ulFailedTransmissions +
                                 stats.ulRetryErrors +
                                 stats.ulUnderflowErrors +
                                 stats.ulLossOfCarrierErrors +
                                 stats.ulLateCollisionErrors;

        priv->stats.collisions = stats.ulLateCollisionErrors; 

        priv->stats.rx_over_errors = stats.ulOverFlowErrors;
        priv->stats.rx_crc_errors = stats.ulCRCErrors; 
        priv->stats.rx_frame_errors = stats.ulFramingErrors;
        priv->stats.rx_missed_errors = stats.ulMissedFrames;

        priv->stats.tx_carrier_errors = stats.ulLossOfCarrierErrors;
        

    }

    PRINTK(NV_DBGLEVEL_STATS, "nvnet: nvenet_stats - Out\n");
    return &priv->stats;
}

/*
 * Get multicast addresses that OS is interested in
 */
static void
nvenet_multicast(struct net_device *dev)
{
    struct nvenet_private *priv;
    struct dev_mc_list *mcptr;
    ADAPTER_API *pADApi;
    u8 oraddr[6];
    u8 andaddr[6];
    int i;

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_multicast - In\n");

    priv    = (struct nvenet_private *)dev->priv;
    pADApi  = priv->pADApi;

    spin_lock_irq(&priv->lock);
    /*
     * Initialize filter
     */
    priv->hwfilter.ulFilterFlags = 0;
    for(i = 0; i < 6; i++)
    {
        priv->hwfilter.acMulticastAddress[i]  = 0;
        priv->hwfilter.acMulticastMask[i]     = 0;
    }


    if(dev->flags & IFF_PROMISC)
    {
        priv->hwfilter.ulFilterFlags |= ACCEPT_ALL_PACKETS;
    } /* if(dev->flags & IFF_PROMISC) */
    else if (dev->mc_count || (dev->flags & IFF_ALLMULTI))
    {
        priv->hwfilter.ulFilterFlags |= ACCEPT_MULTICAST_PACKETS;
        priv->hwfilter.ulFilterFlags |= ACCEPT_UNICAST_PACKETS;
        priv->hwfilter.ulFilterFlags |= ACCEPT_BROADCAST_PACKETS;

        if (dev->flags & IFF_ALLMULTI)
        {
            priv->hwfilter.acMulticastMask[0]    = 0xff;
            priv->hwfilter.acMulticastAddress[0] = 0x01;
            priv->hwfilter.acMulticastMask[1]    = 0xff;
            priv->hwfilter.acMulticastAddress[1] = 0x00;
        } /* if (dev->flags & IFF_ALLMULTI) */
        else
        {
            /*
             * Process multicast address list
             */

            mcptr = dev->mc_list;
            for(i = 0; i < 6; i++)
                oraddr[i] = andaddr[i] = mcptr->dmi_addr[i];

            mcptr = mcptr->next;
            for(;mcptr; mcptr = mcptr->next)
            {
                PRINTK(NV_DBGLEVEL_OPEN, "nvnet: multicast - addr len for addr: %u\n",\
                   mcptr->dmi_addrlen);
                for(i = 0; i < 6; i++)
                {
                    u8 mcaddr   = mcptr->dmi_addr[i];
                    andaddr[i]  &= mcaddr;
                    oraddr[i]   |= mcaddr;
                }
            }

            /* 
             * Write 1s to mask where all addresses have 0 bit or 1 bit. 
             * Compute mask and address value.
             */
            for(i = 0; i < 6; i++)
            {
                priv->hwfilter.acMulticastAddress[i] = andaddr[i] & oraddr[i];
                priv->hwfilter.acMulticastMask[i]    = andaddr[i] | (~oraddr[i]);
                PRINTK(NV_DBGLEVEL_OPEN, "nvnet: multicast mask[%u] = 0x%x addr[%u] = 0x%x\n",
                    i, priv->hwfilter.acMulticastMask[i],
                    i, priv->hwfilter.acMulticastAddress[i]);
            } /* for (i=0; i < 6; i++) */
        } /* else */

    } /* else if (dev->mc_count || (dev->flags & IFF_ALLMULTI)) */
    else
    {

        priv->hwfilter.ulFilterFlags |= ACCEPT_UNICAST_PACKETS;
        priv->hwfilter.ulFilterFlags |= ACCEPT_BROADCAST_PACKETS;
    } /* else */

    pADApi->pfnSetPacketFilter(pADApi->pADCX, &priv->hwfilter);
    spin_unlock_irq(&priv->lock);
    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_multicast - Out\n");

}

static irqreturn_t
nvenet_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
{
    struct net_device *dev;
    struct nvenet_private *priv;

    dev     = (struct net_device *)dev_instance;
    priv    = (struct nvenet_private *)dev->priv;

    PRINTK(NV_DBGLEVEL_INTR, "nvnet: nvenet_interrupt - In (%u, %p, %p)\n", \
                 irq, dev_instance, regs);

    if(!dev)
      return IRQ_NONE;

    spin_lock(&priv->lock);

    if(priv->pADApi->pfnQueryInterrupt(priv->pADApi->pADCX)) 
    {   
        priv->pADApi->pfnHandleInterrupt(priv->pADApi->pADCX);
        priv->pADApi->pfnEnableInterrupts(priv->pADApi->pADCX);

    }

    spin_unlock(&priv->lock);

    PRINTK(NV_DBGLEVEL_INTR, "nvenet - nvenet_interrupt -  Out\n");

    return IRQ_HANDLED;
}


static int
nvenet_close(struct net_device *dev)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_close - In \n");

    netif_carrier_off(dev);
    netif_stop_queue(dev);

    if(priv->pADApi)
    {
        priv->pADApi->pfnDeinit(priv->pADApi->pADCX, AFFECT_TRANSMITTER | AFFECT_RECEIVER);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 28)
    synchronize_irq();
#else
    synchronize_irq(dev->irq);
#endif

    free_irq(dev->irq, dev); 

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_close - OUT\n");
    return 0;
}

static int
nvenet_ioctl(struct net_device *dev, struct ifreq *request, int cmd)
{
    u16 *data = (u16 *)&request->ifr_data;
    struct nvenet_private *priv;
    int retval  = 0;
    ADAPTER_API *pADApi;

    PRINTK(NV_DBGLEVEL_IOCTL, "nvnet: nvenet_ioctl - In cmd = %x\n", cmd);

    priv = (struct nvenet_private *)dev->priv;
    pADApi = priv->pADApi;


    switch(cmd) 
    {
        case SIOCETHTOOL: 
        { 
               u32 ethcmd;
        
            if (copy_from_user(&ethcmd, (char *)request->ifr_data, sizeof(ethcmd))) {

                PRINTK_ERROR("nvnet: nvenet_ioctl - copy_from_user failed\n");
                return -EFAULT;
            }

            switch(ethcmd) {
                case ETHTOOL_GDRVINFO: 
                {
                    struct ethtool_drvinfo info = {ETHTOOL_GDRVINFO};
                    
                    strcpy(info.driver, DRIVER_NAME);
                    strcpy(info.version, DRIVER_VERSION);

                    if (priv->pdev)
                        strcpy(info.bus_info, pci_name(priv->pdev));

                    if (copy_to_user((char *)request->ifr_data, &info, sizeof(info))) 
                    {
                        PRINTK_ERROR("nvnet: nvenet_ioctl - copy_to_user failed\n");
                        return -EFAULT;
                    }      
                    break;
                }
                case ETHTOOL_GSET:
                {
	                struct ethtool_cmd settings;
	                NV_UINT32 speed = pADApi->pfnGetLinkSpeed(pADApi->pADCX);
	                NV_UINT32 duplex = pADApi->pfnGetLinkMode(pADApi->pADCX);
	                
	                memset((void *) &settings, 0, sizeof (settings));

	                settings.supported = ETHTOOL_SUPPORTED_INTERFACE;
                    settings.speed = speed / 1000000;
	                settings.duplex = duplex - 1; /*DUPLEX_HALF or DUPLEX_FULL*/
                    settings.port = PORT_TP;
                    settings.phy_address = priv->phyaddr;
                    settings.transceiver = XCVR_INTERNAL;
                    settings.autoneg = (priv->force_mode == 0 ? AUTONEG_ENABLE : AUTONEG_DISABLE);

                    if (copy_to_user((char *)request->ifr_data, &settings, sizeof(settings))) 
                    {
                        PRINTK_ERROR("nvnet: nvenet_ioctl - copy_to_user failed\n");
                        return -EFAULT;
                    }      
	                break;
                }
                case ETHTOOL_GLINK:
                {
	                struct ethtool_value linkvalue = {ETHTOOL_GLINK};
	                NV_UINT32 linkup;

	                priv->pADApi->pfnGetLinkState(priv->pADApi->pADCX, &linkup);
	                linkvalue.data = linkup;

                    if (copy_to_user((char *)request->ifr_data, &linkvalue, sizeof(linkvalue))) 
                    {
                        PRINTK_ERROR("nvnet: nvenet_ioctl - copy_to_user failed\n");
                        return -EFAULT;
                    }      
	                break;
                }
                default:
                    retval = -EOPNOTSUPP;
                    break;

                 }
            }
            break;

        case SIOCGMIIPHY:             /* Get address of MII PHY in use. */
        case SIOCDEVPRIVATE:          /* for binary compat, remove in 2.5 */
            data[0] = priv->phyaddr; 
            break;
        
        case SIOCGMIIREG:             /* Read MII PHY register. */
        case SIOCDEVPRIVATE+1:        /* for binary compat, remove in 2.5 */

            if(NVENET_STATUS_SUCCESS != pADApi->pfnReadPhy(
                                            pADApi->pADCX, 
                                            priv->phyaddr, 
                                            data[1] & 0x1f, 
                                            (NV_UINT32 *)&data[3])) 
            {
                PRINTK_ERROR("nvnet: nvenet_ioctl - ReadPhy failed\n");
                retval = -EBUSY;
            }
            break;
        case SIOCSMIIREG:             /* Write MII PHY register. */
        case SIOCDEVPRIVATE+2:        /* for binary compat, remove in 2.5 */

            if(NVENET_STATUS_SUCCESS != pADApi->pfnWritePhy(
                                            pADApi->pADCX, 
                                            priv->phyaddr, 
                                            data[1] & 0x1f, 
                                            data[3])) 
            {
                 PRINTK_ERROR("nvnet: nvenet_ioctl - WritePhy failed\n");
                 retval = -EBUSY; 
            }
            break;


        default:
            retval = -EOPNOTSUPP;
            break;
    }

    PRINTK(NV_DBGLEVEL_IOCTL, "nvenet_ioctl : out with ret =%x\n", retval);

    return retval;
}

static int
nvenet_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct nvenet_private *priv;
    ADAPTER_WRITE_DATA txdata;
    ADAPTER_API *pADApi;
    nvenet_tx_info_os_t *tx_ring;
    ADAPTER_WRITE_OFFLOAD write_offload = { 0 };
    int retval = 0;
    int i;
    int status, fragments;


    PRINTK(NV_DBGLEVEL_TX, "nvnet: nvenet_xmit - In\n");

    priv    = (struct nvenet_private *)dev->priv;
    pADApi  = priv->pADApi;

    if(!skb) 
    {
        printk(KERN_EMERG "nvenet_xmit - invalid skb\n");
        dev_kfree_skb_any(skb);
        PRINTK(NV_DBGLEVEL_TX, "nvnet: nvenet_xmit - Out\n");
        return 0;
    }

    fragments = skb_shinfo(skb)->nr_frags;

    if ((skb->len <=0) || (skb->len > priv->max_frame_size && !fragments))
    {
        printk(KERN_EMERG "nvenet_xmit - skb->len  > priv->max_frame_size\n");
        dev_kfree_skb_any(skb);
        PRINTK(NV_DBGLEVEL_TX, "nvnet: nvenet_xmit - Out \n");
        return 0;
    }

    spin_lock_irq(&priv->lock);

    /*
     * Check if index is free
     */
    if (priv->tx_ring_busy_count == priv->max_tx_packets) 
    {
        /*
         * all tx ring entries are consumed 
         */
        priv->tx_ring_full = 1;
        PRINTK_ERROR("nvnet: nvenet_xmit - skb - tx ring full\n");
        netif_stop_queue(dev);
        spin_unlock_irq(&priv->lock);
        return 1;

    }

    /*
     * Put the fragment information in tx ring
     */

    tx_ring = priv->tx_ring + priv->tx_ring_index;
    memset(&txdata, 0, sizeof(txdata));

    txdata.pvID                 =  (PNV_VOID)tx_ring;
    tx_ring->pvID               = (PNV_VOID)skb;
    tx_ring->pCoalesceBuffer    = NULL;

    txdata.ulNumberOfElements   = 1;
    txdata.ulTotalLength        = cpu_to_le32(skb->len);

    if (!fragments)
    {
        tx_ring->dma_addr[0]            = pci_map_single(
                                                priv->pdev, 
                                                skb->data, 
                                                skb->len, 
                                                PCI_DMA_TODEVICE);
        txdata.sElement[0].pPhysical    = (PNV_VOID)(
                                          (unsigned long)cpu_to_le32(
                                                tx_ring->dma_addr[0]));
        txdata.sElement[0].ulLength     = skb->len;

    }
    else
    {
        PRINTK(NV_DBGLEVEL_TX, "nvnet: Number of Fragments is set\n");

        tx_ring->dma_addr[0] = pci_map_single(
                                        priv->pdev, 
                                        skb->data,
                                        skb->len - skb->data_len,
                                        PCI_DMA_TODEVICE);

        tx_ring->len[0]      = cpu_to_le32(skb->len - skb->data_len);

        txdata.sElement[0].pPhysical = (PNV_VOID)(
                                       (unsigned long)cpu_to_le32(
                                       tx_ring->dma_addr[0]));
        txdata.sElement[0].ulLength  = tx_ring->len[0];

        for (i=0; i < fragments; i++)
        { 
            skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

            tx_ring->dma_addr[i+1]  = pci_map_single(
                                            priv->pdev,
                                            (PNV_VOID)
                                            ((u8*)page_address(frag->page) +
                                            frag->page_offset),
                                            frag->size,
                                            PCI_DMA_TODEVICE);

            tx_ring->len[i+1]       = cpu_to_le32(frag->size);

            txdata.sElement[i+1].pPhysical  = (PNV_VOID)(unsigned long)
                                              cpu_to_le32(
                                                tx_ring->dma_addr[i+1]);
            txdata.sElement[i+1].ulLength   = tx_ring->len[i+1];

            txdata.ulNumberOfElements++;
        }

    }


    txdata.psOffload = &write_offload;

    if (priv->tx_checksum_offload && (CHECKSUM_HW == skb->ip_summed))
    {
        priv->tx_checksum_done_count++;
        write_offload.usBitmask     |= priv->tx_checksum_mask;
    }
    
#if NETIF_F_TSO
    if (priv->seg_offload && fragments)
    { 
        NV_UINT16 mss;

        mss                     = skb_shinfo(skb)->tso_size;
        write_offload.ulMss     |= mss;
        NV_BIT_SET(write_offload.usBitmask, ADAPTER_WRITE_OFFLOAD_BP_SEGOFFLOAD);
    }
#endif

    status = pADApi->pfnWrite(pADApi->pADCX, &txdata);

    switch(status) 
    {
        case ADAPTERERR_NONE:
            priv->stats.tx_bytes += skb->len;
            priv->tx_ring_busy_count++;
            priv->tx_ring_index = (priv->tx_ring_index + 1) % priv->max_tx_packets;
            dev->trans_start    = jiffies; 
            break;

        case ADAPTERERR_TRANSMIT_QUEUE_FULL:
            dev_kfree_skb_any(skb);
            netif_stop_queue(dev);
            retval  = 1;
            PRINTK_ERROR("nvnet: nvenet_xmit -  queue full\n");
            break;

        default:
            dev_kfree_skb_any(skb);
            PRINTK_ERROR("nvnet: nvenet_xmit - transmit error\n");
            break;
    }

    spin_unlock_irq(&priv->lock);
    PRINTK(NV_DBGLEVEL_TX, "nvnet: nvenet_xmit - Out\n");
    return retval;

}

/*
 * Shutdown the hardware
 */
static void __devexit
nvenet_remove(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_remove - IN\n");

    unregister_netdev(dev);

    remove_nvenet_proc_entries(dev);

    if (priv->pADApi != NULL)
    {
        priv->pADApi->pfnSetNodeAddress(
                priv->pADApi->pADCX, 
                priv->permanent_address); 

        delete_adapter_object(priv->pADApi);
        priv->pADApi = NULL;

    }

    pci_free_consistent(
            priv->pdev, 
            priv->tx_ring_size, 
            priv->tx_ring, 
            priv->tx_ring_dma); 

    iounmap(priv->regs);
    release_mem_region(pci_resource_start(pdev, 0), pci_resource_len(pdev,0));

    pci_disable_device(pdev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    kfree(dev);
#else
    free_netdev(dev);
#endif

    pci_set_drvdata(pdev, NULL);

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_remove -  Out\n");
}

static int __init
nvenet_init_module(void)
{
    int status = -1;
    PRINTK(NV_DBGLEVEL_INIT, "nvnet: nvenet_init_module - In\n");

    // Create root of nvnet proc hierarchy

    nvenet_proc_root = create_proc_entry("nvnet", S_IFDIR, proc_net);

    if (nvenet_proc_root != NULL)
    {
        status = pci_module_init(&nvenet_driver);
	if (status >=0)
	{
	    register_reboot_notifier(&nvenet_reboot_notifier);
	}
    }
    else
    {
        PRINTK_ERROR("nvnet: create_proc_entry failed to create proc root\n");
    }

    PRINTK(NV_DBGLEVEL_INIT, "nvnet: nvenet_init_module - Out\n");

    return status;
}

static void __exit
nvenet_cleanup_module(void)
{
    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_cleanup_module - In\n");

    unregister_reboot_notifier(&nvenet_reboot_notifier);

    pci_unregister_driver(&nvenet_driver);

    remove_proc_entry("nvnet", proc_net);

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_cleanup_module - Out\n");
}

module_init(nvenet_init_module);
module_exit(nvenet_cleanup_module);

#ifdef CONFIG_PM

int
nvenet_suspend(struct pci_dev *pcidev, u32 state)
{
    struct net_device *dev = (struct net_device *)pci_get_drvdata(pcidev);
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    ADAPTER_POWERSTATE power_state = {0};

    PRINTK(NV_DBGLEVEL_POWER, "nvnet: nvenet_suspend - In\n");

    if (dev && dev->priv)
    {

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        pci_save_state(priv->pdev, priv->pci_state);
#else
        pci_save_state(priv->pdev);
#endif
        priv->pADApi->pfnGetPowerState(priv->pADApi->pADCX, &power_state);
        if (netif_running(dev))
        {
            netif_device_detach(dev);
            priv->pADApi->pfnDeinit(priv->pADApi->pADCX, AFFECT_TRANSMITTER);
            priv->pADApi->pfnClearTxDesc(priv->pADApi->pADCX);
            priv->tx_ring_full      = 0;
            priv->tx_ring_index     = 0;
            priv->tx_ring_busy_count     = 0;
        }
	else
	{
	    priv->pADApi->pfnStart(priv->pADApi->pADCX); //start receiver
	}

        power_state.ulPowerFlags &= ~POWER_STATE_D0;
        if (state == 1)
        {
            power_state.ulPowerFlags |= POWER_STATE_D1;
        }
        else if (state == 2)
        {
            power_state.ulPowerFlags |= POWER_STATE_D2;
        }
        else if (state == 3)
        {
            power_state.ulPowerFlags |= POWER_STATE_D3;
        }
    
        power_state.ulMagicPacketWakeUpFlags    = POWER_STATE_ALL;
        power_state.ulLinkChangeWakeUpFlags     = POWER_STATE_ALL;
        power_state.ulPatternWakeUpFlags        = 0;
        priv->pADApi->pfnSetPowerState(priv->pADApi->pADCX, &power_state);
    }

    pci_enable_wake(priv->pdev, 3, 1);  //D3hot
    pci_enable_wake(priv->pdev, 4, 1);  //D3cold
    pci_set_power_state(priv->pdev, 3);

    PRINTK(NV_DBGLEVEL_POWER, "nvnet: nvenet_suspend - Out\n");
    return 0;
}

int
nvenet_resume(struct pci_dev *pcidev)
{
    struct net_device *dev;
    struct nvenet_private *priv;
    ADAPTER_POWERSTATE power_state = {0};
    int status;

    PRINTK(NV_DBGLEVEL_POWER, "nvnet: nvenet_resume - In\n");

    dev  = (struct net_device *)pci_get_drvdata(pcidev);
    priv = (struct nvenet_private *)dev->priv;

    if (dev && dev->priv)
    {
        pci_set_power_state(priv->pdev, 0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        pci_restore_state(priv->pdev, priv->pci_state);
#else
        pci_restore_state(priv->pdev);
#endif
        pci_enable_wake(priv->pdev, 3, 0);
        pci_enable_wake(priv->pdev, 4, 0);

        if (netif_running(dev))
        {
            power_state.ulPowerFlags |= POWER_STATE_D0;
            priv->pADApi->pfnSetPowerState(priv->pADApi->pADCX, &power_state);
            
            priv->linkup                    = TRUE;
			priv->async_mode				= FALSE;

            /*
             * Initialize hardware
             */
            status = priv->pADApi->pfnInit(
                        priv->pADApi->pADCX, 
                        priv->speed,  /* force speed */ 
                        priv->force_duplex, /* force full duplex */
                        priv->force_mode,   /* force mode */
                        priv->async_mode,	/* async mode */
                        &priv->linkup
                        );
            if(status != ADAPTERERR_NONE) 
            {
                printk(KERN_EMERG "nvnet: nvenet_resumet - pfnInit failed\n");
                return -EAGAIN;
            }

            /* Set the node address */
            priv->pADApi->pfnSetNodeAddress(priv->pADApi->pADCX, priv->reversed_address);
    
            /*
             * Set filter again
             */
            nvenet_multicast(dev);

            /*
             * Initialize tx ring indexes
             */
            priv->tx_ring_full      = 0;
            priv->tx_ring_index     = 0;
            priv->tx_ring_busy_count     = 0;

            /*
             * Turn on hardware
             */
            priv->pADApi->pfnStart(priv->pADApi->pADCX);
        }

        netif_device_attach(dev);
    }  

    PRINTK(NV_DBGLEVEL_POWER, "nvnet: nvenet_resume - Out\n");

    return 0;
}

#endif //CONFIG_PM

#if 0
static void nvenet_watchdog(struct net_device *dev)
{
    PRINTK(NV_DBGLEVEL_WATCHDOG, "nvnet: nvenet_watchdog - In\n");

    nvenet_close(dev);
    nvenet_open(dev);

    PRINTK(NV_DBGLEVEL_WATCHDOG, "nvnet: nvenet_watchdog - Out\n");
}
#endif

static int 
nvenet_change_mtu(struct net_device *dev, int mtu)
{
    struct nvenet_private *priv;
    PADAPTER_API pADApi;
    int newframesize;
    int oldmtu;
    int status = 0;
    
    priv = (struct nvenet_private *)dev->priv;

    spin_lock_irq(&priv->lock);
    newframesize = mtu + ETH_HLEN + FCS_LEN;
    
    if ((mtu < MIN_PACKET_MTU_SIZE) || (mtu > MAX_PACKET_MTU_SIZE))
    {
        printk(KERN_INFO "nvnet: nvenet_change_mtu failed, mtu size %u is not valid\n", mtu);
        spin_unlock_irq(&priv->lock);
        return -EINVAL;
    }  
    
    /*
     * Update mtu
     */
    oldmtu = priv->mtu;

    if (newframesize != priv->max_frame_size)
    {
        /*
         * Use the new mtu
         */
        priv->mtu = mtu;

        if (netif_running(dev))
        {
            nvenet_close(dev);
        }

        pADApi = priv->pADApi;
        delete_adapter_object(pADApi);

        if (create_adapter_object(dev) == NVENET_STATUS_SUCCESS)
        {
            dev->mtu = mtu;            
        }
        else
        {
            priv->mtu = oldmtu;
            status = -ENOMEM;
            create_adapter_object(dev);
        }

        nvenet_open(dev);
    }

    spin_unlock_irq(&priv->lock);

    PRINTK(NV_DBGLEVEL_OPEN, "nvnet: nvenet_change_mtu - out with status 0x%x\n", status);
    return status;
}

static int
nvenet_mac_address(struct net_device *dev, void *addr)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct sockaddr * macaddr = (struct sockaddr *)addr;
  
    spin_lock_irq(&priv->lock);

    /* turn off hw */
    priv->pADApi->pfnStop(priv->pADApi->pADCX, AFFECT_RECEIVER);

    /* Set the node address */
    memcpy(dev->dev_addr, macaddr->sa_data, ETH_ALEN);
    memcpy(priv->reversed_address, macaddr->sa_data, ETH_ALEN);
    priv->pADApi->pfnSetNodeAddress(priv->pADApi->pADCX, priv->reversed_address);    

    /* turn on hw again */
    priv->pADApi->pfnStart(priv->pADApi->pADCX);

    spin_unlock_irq(&priv->lock);

    return 0;
}

static void
delete_adapter_object(PADAPTER_API pADApi)
{
    pADApi->pfnClose(pADApi->pADCX, FALSE);

}

static int
create_adapter_object(struct net_device *dev)
{
    struct nvenet_private *priv;
    OS_API *osapi;
    int status;
    ADAPTER_OPEN_PARAMS OpenParams = { 0 };

    PRINTK(NV_DBGLEVEL_INIT, "nvnet: create_adapter_object - In\n");

    priv = (struct nvenet_private *)dev->priv;

    /*
     * Plug into the API
     */
    osapi                           = &priv->linuxapi;
    osapi->pOSCX                    = dev;
    osapi->pfnAllocMemory           = linuxalloc;
    osapi->pfnFreeMemory            = linuxfree;
    osapi->pfnClearMemory           = linuxclear;
    osapi->pfnAllocMemoryEx         = linuxallocex;
    osapi->pfnFreeMemoryEx          = linuxfreeex;
    osapi->pfnStallExecution        = linuxdelay;
    osapi->pfnAllocReceiveBuffer    = linuxallocrxbuf;
    osapi->pfnFreeReceiveBuffer     = linuxfreerxbuf;
    osapi->pfnPacketWasReceived     = linuxpacketrx;
    osapi->pfnPacketWasSent         = linuxpackettx;
    osapi->pfnLinkStateHasChanged   = linuxlinkchanged;
    osapi->pfnLockAlloc             = linuxlockalloc;
    osapi->pfnLockAcquire           = linuxlockacquire;
    osapi->pfnLockRelease           = linuxlockrelease;
    priv->linkup                    = TRUE;
    priv->max_frame_size            = priv->mtu + ETH_HLEN + FCS_LEN;
    priv->hpna                      = FALSE;

    PRINTK(NV_DBGLEVEL_INFO, "hwmode = 0x%x, max_rx_pkts = 0x%x , max_tx_pkts = 0x%x poll = 0x%x\n",
            priv->hwmode, priv->max_rx_packets, priv->max_tx_packets, priv->poll_interval_in_us);

    //
    // Set open parameters 
    //
    OpenParams.MaxDpcLoop               = 2;
    OpenParams.MaxRxPkt                 = priv->max_rx_packets;
    OpenParams.MaxTxPkt                 = priv->max_tx_packets;
    OpenParams.SentPacketStatusSuccess  = 1;
    OpenParams.SentPacketStatusFailure  = 0;
    OpenParams.MaxRxPktToAccumulate     = 6;

    if (priv->optimization == NVENET_OPTIMIZATION_THROUGHPUT)
    {
        OpenParams.ulPollInterval = 0;
    }
    else
    {
        OpenParams.ulPollInterval = priv->poll_interval_in_us;
    }

    OpenParams.SetForcedModeEveryNthRxPacket = 0;
    OpenParams.SetForcedModeEveryNthTxPacket = 0;
    OpenParams.RxForcedInterrupt = 0;
    OpenParams.TxForcedInterrupt = 0;
    OpenParams.pOSApi = osapi;
    OpenParams.pvHardwareBaseAddress = priv->regs;
    OpenParams.bASFEnabled = 0;
    OpenParams.ulDescriptorVersion = priv->hwmode;
    OpenParams.ulMaxPacketSize = priv->max_frame_size;
    OpenParams.DeviceId = priv->pdev->device;

    status = ADAPTER_Open (&OpenParams, (void **)&(priv->pADApi), 
                           &priv->phyaddr);

    if(status != ADAPTERERR_NONE) 
    {
        PRINTK_ERROR("nvnet: create_adapter_object, ADAPTER_Open failed\n");
        PRINTK(NV_DBGLEVEL_INIT, "nvnet: create_adapter_object -  Out with error\n");
        return NVENET_STATUS_FAILURE;
    }

    //
    // Set the adapter data
    //
    memset(
        &priv->adapterdata,
        0,
        sizeof(priv->adapterdata)
        );

    priv->adapterdata.ulMediaIF            = priv->media;

    if (NVENET_HW_MODE_2 == priv->hwmode)
    {
        priv->adapterdata.ulChecksumOffloadBM  = 0x0FFFF;
    }
    priv->adapterdata.ulModeRegTxReadCompleteEnable    = 1;

    priv->pADApi->pfnSetCommonData( priv->pADApi->pADCX, &priv->adapterdata);

    priv->tx_checksum_mask   = 0;

    //
    // Offload capabilities to hardware
    //
    if (NVENET_HW_MODE_2 == priv->hwmode)
    {
        NV_UINT32 offloadcap = 0;

        //
        // Save offload bitmask so we can pass them in xmit
        //
        NV_BIT_SETVALUE(
            priv->tx_checksum_mask,
            1,
            ADAPTER_WRITE_OFFLOAD_BP_IPV4CHECKSUM,
            ADAPTER_WRITE_OFFLOAD_BP_IPV4CHECKSUM
            );

        NV_BIT_SETVALUE(
            priv->tx_checksum_mask,
            1,
            ADAPTER_WRITE_OFFLOAD_BP_IPV6CHECKSUM,
            ADAPTER_WRITE_OFFLOAD_BP_IPV6CHECKSUM
            );

        NV_BIT_SETVALUE(
            priv->tx_checksum_mask,
            1,
            ADAPTER_WRITE_OFFLOAD_BP_IPCHECKSUM,
            ADAPTER_WRITE_OFFLOAD_BP_IPCHECKSUM
            );

        NV_BIT_SETVALUE(
            priv->tx_checksum_mask,
            1,
            ADAPTER_WRITE_OFFLOAD_BP_TCPCHECKSUM,
            ADAPTER_WRITE_OFFLOAD_BP_TCPCHECKSUM
            );
        NV_BIT_SETVALUE(
            priv->tx_checksum_mask,
            1,
            ADAPTER_WRITE_OFFLOAD_BP_UDPCHECKSUM,
            ADAPTER_WRITE_OFFLOAD_BP_UDPCHECKSUM
            );

        //
        // Set capabilities on hardware
        //
        NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_IPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_TCPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V4_TX_BP_UDPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_IPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_TCPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V4_RX_BP_UDPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V6_TX_BP_TCPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V6_TX_BP_UDPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V6_RX_BP_TCPCHECKSUM);
        NV_BIT_SET(offloadcap, DEVCAPS_V6_RX_BP_UDPCHECKSUM);

        priv->pADApi->pfnSetChecksumOffload(
            priv->pADApi->pADCX,
            offloadcap
            );

    }

    PRINTK(NV_DBGLEVEL_INIT, "nvnet: create_adapter_object - Out with success\n");
    return NVENET_STATUS_SUCCESS;

}


/*********************************************************************
                    Local routines
*********************************************************************/

static NV_API_CALL NV_SINT32
linuxlockalloc(PNV_VOID ctx, NV_SINT32 type, PNV_VOID *pLock)
{
    struct net_device *dev = (struct net_device *)ctx;
    struct nvenet_private *priv  = dev->priv;

    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockalloc - IN\n");

    spin_lock_init(&priv->phylock);
    *pLock = (void **)&priv->phylock;

    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockalloc - Out\n");

    return 1;

}

static NV_API_CALL NV_SINT32
linuxlockacquire(PNV_VOID ctx, NV_SINT32 type, PNV_VOID lock)
{
    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockacquire: IN\n");

    spin_lock((spinlock_t*)lock);

    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockacquire: OUT\n");
    return 1;

}

static NV_API_CALL NV_SINT32
linuxlockrelease(PNV_VOID ctx, NV_SINT32 type, PNV_VOID lock)
{
    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockrelease: In\n");

    spin_unlock((spinlock_t*)lock);

    PRINTK(NV_DBGLEVEL_LOCK, "nvnet: linuxlockrelease: Out\n");
    return 1;
}

static NV_API_CALL NV_SINT32
linuxalloc(PNV_VOID ctx, MEMORY_BLOCK *mem)
{
    struct net_device *dev = (struct net_device *)ctx;
    struct nvenet_private *priv  = dev->priv;
    dma_addr_t dma_addr;

    PRINTK(NV_DBGLEVEL_INIT, "nvnet: linuxalloc: In\n");
    mem->pLogical = (void *)pci_alloc_consistent(priv->pdev,  
                             (size_t)mem->uiLength, &dma_addr);
    if (!mem->pLogical) 
    {
        PRINTK_ERROR("nvnet: linuxalloc: memory allocation failed\n");
        PRINTK(NV_DBGLEVEL_INIT, "nvnet: linuxalloc: Out\n");
        return 0;
    }

    memset(mem->pLogical, 0, mem->uiLength);
    mem->pPhysical = (void *)dma_addr;

    PRINTK(NV_DBGLEVEL_INIT,"nvnet: linuxalloc: Out\n");
    return 1;

}

static NV_API_CALL NV_SINT32
linuxfree(PNV_VOID ctx, MEMORY_BLOCK *mem)
{
    struct net_device *dev = (struct net_device *)ctx;
    struct nvenet_private *priv = dev->priv;
    dma_addr_t dma_addr = (dma_addr_t)mem->pPhysical;
 
    PRINTK(NV_DBGLEVEL_INIT, "nvnet: linuxfree: In\n");

    pci_free_consistent(priv->pdev, (size_t)mem->uiLength, 
                        mem->pLogical, dma_addr);

    PRINTK(NV_DBGLEVEL_INIT, "nvnet: linuxfree: Out\n");
    return 1;

}

static NV_API_CALL NV_SINT32
linuxallocex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex)
{
    MEMORY_BLOCK mem_block;
    
    mem_block_ex->pLogical = NULL;
    mem_block_ex->uiLengthOrig = mem_block_ex->uiLength;

    if ((mem_block_ex->AllocFlags & ALLOC_MEMORY_ALIGNED) &&
        (mem_block_ex->AlignmentSize > 1))
    {
        mem_block_ex->uiLengthOrig += mem_block_ex->AlignmentSize;
    }

    mem_block.uiLength = mem_block_ex->uiLengthOrig;

    if (linuxalloc(ctx, &mem_block) == 0)
    {
        return 0;
    }

    mem_block_ex->pLogicalOrig = mem_block.pLogical;
    mem_block_ex->pPhysicalOrigLow = (NV_UINT32)((unsigned long)mem_block.pPhysical);
    mem_block_ex->pPhysicalOrigHigh = 0;

    mem_block_ex->pPhysical = mem_block.pPhysical;
    mem_block_ex->pLogical = mem_block.pLogical;

    if (mem_block_ex->uiLength != mem_block_ex->uiLengthOrig)
    {
        unsigned int offset;
        offset = mem_block_ex->pPhysicalOrigLow & (mem_block_ex->AlignmentSize - 1);

        if (offset)
        {
            mem_block_ex->pPhysical = (PNV_VOID)
                                      ((unsigned long)mem_block_ex->pPhysical + 
                                      mem_block_ex->AlignmentSize - offset);
            mem_block_ex->pLogical = (PNV_VOID)
                                     ((unsigned long)mem_block_ex->pLogical +
                                      mem_block_ex->AlignmentSize - offset);
        }   // if (offset)
    }       // if (mem_block_ex->uiLength != mem_block_ex->uiLengthOrig)
                
    return 1;
}

static NV_API_CALL NV_SINT32 
linuxfreeex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex) 
{
    MEMORY_BLOCK mem_block;

    mem_block.pLogical = mem_block_ex->pLogicalOrig;
    mem_block.pPhysical = (PNV_VOID)((unsigned long)mem_block_ex->pPhysicalOrigLow);
    mem_block.uiLength = mem_block_ex->uiLengthOrig;

    return linuxfree(ctx, &mem_block);
}

static NV_API_CALL NV_SINT32
linuxclear(PNV_VOID ctx, PNV_VOID mem, NV_SINT32 length)
{
    memset(mem, 0, length);
    return 1;
}

static NV_API_CALL NV_SINT32
linuxdelay(PNV_VOID ctx, NV_UINT32 usec)
{
    udelay(usec);
    return 1;
}

static NV_API_CALL NV_SINT32
linuxallocrxbuf(PNV_VOID ctx, MEMORY_BLOCK *mem, PNV_VOID *id) 
{
    struct net_device *dev  = (struct net_device *)ctx;
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct sk_buff *skb;
    struct nvenet_rx_info *info;
    NV_UINT32 length;

    length = mem->uiLength + 2 + sizeof(struct nvenet_rx_info);

    skb = dev_alloc_skb(length);
    if(!skb) 
    {
        PRINTK_ERROR("nvnet: linuxallocrxbuf - skb allocation failed\n");
        return 0;
    }

    info = (struct nvenet_rx_info *)((u8 *)skb->tail + 
                                    2 + 
                                    mem->uiLength
                                    );

    skb_reserve(skb, 2);        
 
    mem->pLogical = (PNV_VOID)skb->data;

    info->skb       = skb;
    info->uiLength  = priv->max_frame_size;
    *id             = (PNV_VOID) info;        
    info->dma       = pci_map_single(
                        priv->pdev, 
                        skb->data, 
                        info->uiLength,  
                        PCI_DMA_FROMDEVICE
                        );
    mem->pPhysical  = (PNV_VOID)((unsigned long)cpu_to_le32(info->dma));
    return 1;

}

static NV_API_CALL NV_SINT32
linuxfreerxbuf(PNV_VOID ctx, MEMORY_BLOCK *mem, PNV_VOID id)
{
    struct net_device *dev = (struct net_device *)ctx;
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct sk_buff *skb;
    struct nvenet_rx_info *info;

    PRINTK(NV_DBGLEVEL_RX, "nvnet: linuxfreerxbuf: In\n");

    info = (struct nvenet_rx_info *)id;
    skb  = info->skb;

    NVENET_ASSERT(skb != NULL);

    if (skb) 
    {
        pci_unmap_single(
                priv->pdev, 
                info->dma, 
               (unsigned long)info->uiLength, 
                PCI_DMA_FROMDEVICE
                );
        dev_kfree_skb(skb);
    }

    PRINTK(NV_DBGLEVEL_RX,  "nvnet: linuxfreerxbuf: Out\n");
    return 1;
}

static NV_API_CALL NV_SINT32
linuxpackettx(PNV_VOID ctx, PNV_VOID id, NV_UINT32 success)
{
    struct net_device *dev = (struct net_device *)ctx;
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    nvenet_tx_info_os_t *tx_ring = (nvenet_tx_info_os_t *)id;
    struct sk_buff *skb;
    NV_UINT32 i;

    PRINTK(NV_DBGLEVEL_TX, "nvnet: linuxpackettx - In\n");

    skb = (struct sk_buff *)tx_ring->pvID;
    NVENET_ASSERT(skb != NULL);

    if (!skb_shinfo(skb)->nr_frags)
    {
        pci_unmap_single(
                priv->pdev, 
                tx_ring->dma_addr[0],
                skb->len, 
                PCI_DMA_TODEVICE);
    }
    else
    {
        for (i=0; i <= skb_shinfo(skb)->nr_frags; i++)
        {
            pci_unmap_single(
                    priv->pdev, 
                    tx_ring->dma_addr[i], 
                    tx_ring->len[i], 
                    PCI_DMA_TODEVICE);
        }
    }

    tx_ring->pvID    = NULL;
    dev_kfree_skb_irq(skb);

    priv->tx_ring_busy_count--;

    if ((priv->tx_ring_full) && (priv->tx_ring_busy_count < priv->max_tx_packets)) 
    {

        NVENET_ASSERT (priv->tx_ring_busy_count < priv->max_tx_packets);

        priv->tx_ring_full = 0;
        PRINTK_ERROR("nvnet: linuxpackettx - waking up queue \n");
        netif_wake_queue(dev);

    }

    PRINTK(NV_DBGLEVEL_TX, "nvnet: linuxpackettx - Out\n");
    return 1;
}

static NV_API_CALL NV_SINT32
linuxpacketrx(PNV_VOID ctx, PNV_VOID data, NV_UINT32 status, 
              PNV_UINT8 newbuf, NV_UINT8 priority)
{
    struct sk_buff *skb;
    struct net_device *dev;
    struct nvenet_private *priv;
    ADAPTER_READ_DATA *readdata;
    struct nvenet_rx_info *info;
    u16 packetLen, len;
   
    PRINTK(NV_DBGLEVEL_RX, "nvnet: linuxpacketrx- In status = 0x%x\n", status);

    dev         = (struct net_device *)ctx;
    priv        = (struct nvenet_private *)dev->priv;
    readdata   = (ADAPTER_READ_DATA *)data;
    
    info         = (struct nvenet_rx_info *)readdata->pvID;
    skb          = info->skb;

    NVENET_ASSERT(skb != NULL);
    skb->dev     = dev;

    if(status)
    {
        pci_unmap_single(
                priv->pdev, 
                info->dma, 
                info->uiLength, 
                PCI_DMA_FROMDEVICE);

	/* check length field */
	len = readdata->ulTotalLength;
	packetLen = ntohs(((u16*)skb->data)[6]);
	if (packetLen <= 1500)
	{
            if (len > 60)
            {
                packetLen += ETH_HLEN;
                if (packetLen != len)
                {
                    if (packetLen < len)
                    {
                        /* frame has extra padding */
                        len = packetLen;
                    }
                    else
                    {
                        /* length in header is > bytes received */
                        dev_kfree_skb(skb);
                        goto end_rx;
                    }
                }
            }
            else
            {
                if (packetLen > 46)
                {
		    dev_kfree_skb(skb);
		    goto end_rx;
                }
            }
        }

        skb_put(skb, len);
        skb->protocol = eth_type_trans(skb, dev);

        if (priv->rx_checksum_offload)
        {
            rx_checksum_offload_handler( dev, skb, priv, readdata);
        }
        /*
         * Send the received packet up
         */
		netif_rx(skb);

	    /* set rx time */
	    dev->last_rx = jiffies;

        /* 
         * Update receive statistics
         */
        priv->stats.rx_bytes += readdata->ulTotalLength;
    }
    else 
    {
        pci_unmap_single(
                priv->pdev, 
                info->dma, 
                info->uiLength, 
                PCI_DMA_FROMDEVICE);

        dev_kfree_skb(skb);
    }

end_rx:
    PRINTK(NV_DBGLEVEL_RX, "nvnet: linuxpacketrx- Out\n");
    return 1;
}

static NV_API_CALL NV_SINT32
linuxlinkchanged(PNV_VOID ctx, NV_SINT32 enabled)
{
    struct net_device *dev;
    struct nvenet_private *priv;

    dev = (struct net_device *)ctx;
    priv = (struct nvenet_private *)dev->priv;

    PRINTK(NV_DBGLEVEL_LINK, "nvnet: linuxlinkchanged - In\n");

    /*
     * Set speed and duplex settings on the hardware
     */
    priv->pADApi->pfnSetSpeedDuplex(priv->pADApi->pADCX);

    if(enabled) 
    {
        netif_carrier_on(dev);
        dev->flags |= IFF_RUNNING; 
    }
    else 
    {
        netif_carrier_off(dev);
        dev->flags &= ~IFF_RUNNING; 
    }

    PRINTK(NV_DBGLEVEL_LINK, "nvnet: linuxlinkchanged - Out\n");
    return 1;
}

static void
rx_checksum_offload_handler(struct net_device *dev, struct sk_buff *skb, 
                            struct nvenet_private *priv, 
                            ADAPTER_READ_DATA *readdata)
{
    NV_UINT8 checksum_status = readdata->sOffload.ucChecksumStatus;

    if ((checksum_status != RDFLAG_CHK_NOCHECKSUM) && 
        (checksum_status != RDFLAG_CHK_RESERVED))
    {
        do 
        {
            if (checksum_status == RDFLAG_CHK_IPFAIL)
            {
                priv->rx_checksum_failure_count++;
                skb->ip_summed  = CHECKSUM_NONE;
                break;
            }

            //
            // IP checksum passed
            //
            if (checksum_status != RDFLAG_CHK_IPPASSNOTCPUDP)
            {
                //
                // It is TCP/UDP packet
                //
                if ((checksum_status == RDFLAG_CHK_IPPASSTCPPASS) ||
                    (checksum_status == RDFLAG_CHK_IPPASSUDPPASS))
                {
                    priv->rx_checksum_success_count++;
                    skb->ip_summed      = CHECKSUM_UNNECESSARY;
                    break;
                }
                else 
                {
                    priv->rx_checksum_failure_count++;
                    skb->ip_summed      = CHECKSUM_NONE;
                    break;
                }
            }
            else
            {
                priv->rx_checksum_success_count++;
                skb->ip_summed      = CHECKSUM_UNNECESSARY;
                break;
            }
        } while (0);
    }
    else
    {
        skb->ip_summed      = CHECKSUM_NONE;
        priv->rx_checksum_not_ip_count++;
    }

}

static int 
read_module_parameters(struct net_device *dev)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct nvenet_parameter *param;
    int value;
    int count, i;
    NV_UINT8 *buffer;

    /*
     * Hardware mode
     */
    if (hwmode != -1) 
    {
        char *str = NULL;

        if ((priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_4) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_5) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_6) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_7) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_8) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_9) &&
            (priv->pdev->device != PCI_DEVICE_ID_NVIDIA_NVENET_10))
        {
            if (hwmode != NVENET_HW_MODE_1)
            {
                printk(KERN_INFO "nvnet: hwmode value %d not allowed for device id %x\n.\n", hwmode, priv->pdev->device);
            }

            /*
             * Use hardware mode 1
             */
            priv->hwmode = NVENET_HW_MODE_1;
        }
        else
        {  
            if ((hwmode != NVENET_HW_MODE_1) && (hwmode != NVENET_HW_MODE_2))
            {
                printk(KERN_INFO "nvnet:  wrong value %d for hwmode. Using value %d\n", hwmode, NVENET_HW_MODE_2);
                priv->hwmode = NVENET_HW_MODE_2;
            }
            else
            {
                priv->hwmode = hwmode;
            }

        }
        switch (priv->hwmode)
        {
        case NVENET_HW_MODE_1:
            str = "Mode 1";
            break;
        case NVENET_HW_MODE_2:
            str = "Mode 2";
            break;
        }
        printk(KERN_INFO "nvnet: selecting hwmode setting as %s\n", str);
    }
    else
    {
        if ((PCI_DEVICE_ID_NVIDIA_NVENET_4 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_5 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_6 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_7 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_8 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_9 == priv->pdev->device) ||
            (PCI_DEVICE_ID_NVIDIA_NVENET_10 == priv->pdev->device))
        {
            priv->hwmode = NVENET_HW_MODE_2;
        }
        else
        { 
            priv->hwmode = NVENET_HW_MODE_1;
        }
    }

    if (NVENET_HW_MODE_1 == priv->hwmode)
    {
        param   = nvenet_mode_1_parameters;
        count   = sizeof(nvenet_mode_1_parameters)/ sizeof(struct nvenet_parameter);
    }
    else 
    {
        param   = nvenet_mode_2_parameters;
        count   = sizeof(nvenet_mode_2_parameters)/ sizeof(struct nvenet_parameter);

    }

    for (i=0; i < count; i++, param++)
    {
    
        buffer  = (PNV_UINT8)priv + param->offset;
        
        if (FALSE == param->is_available)
        {
            value = param->default_value;

            switch (param->size)
            {
            case 1:        
                *(PNV_UINT8)buffer = (NV_UINT8)value;
                break;
            case 2:
                *(NV_UINT16 *)buffer = (NV_UINT16)value;
                break;
            case 4:
                *(PNV_UINT32)buffer = (NV_UINT32)value;
                break;
            }

            if (*param->user_value != -1)
            { 
                printk(KERN_EMERG "nvnet: parameter %s not available in mode %u. Ignoring parameter\n",
                   param->name, priv->hwmode);
            }

            continue;
        }
    
        if (-1 == *param->user_value)
        {
            value = param->default_value;
        }
        else
        {
            if ((*param->user_value < param->min_value) || (*param->user_value > param->max_value))
            {
                value = param->default_value;
            }
            else
            {
                value = *param->user_value;
            }
            if (NVENET_PARAM_TYPE_INDEX == param->type)
            {
                printk(KERN_INFO "nvnet: setting parameter %s to %s\n", param->name, param->str[value]);
            }
            else
            {
                printk(KERN_INFO "nvnet: setting parameter %s to value %u\n", param->name, value);
            }
        }
    
        switch (param->size)
        {
        case 1:        
             *(PNV_UINT8)buffer = (NV_UINT8)value;
             break;
        case 2:
            *(NV_UINT16 *)buffer = (NV_UINT16)value;
            break;
        case 4:
            *(PNV_UINT32)buffer = (NV_UINT32)value;
            break;
        }
    }

    /*
     * Set speed and duplex settings
     */

    priv->force_mode = (priv->auto_negotiate == NVENET_NO_AUTO_NEGOTIATE ? 1 : 0);

    switch (priv->force_speed_duplex)
    {
    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE:
        priv->force_duplex  = 0;
        priv->speed         = 0;
        priv->force_mode    = 0;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_FORCE_10_HALF_DUPLEX:
        priv->force_duplex  = 1;
        priv->speed         = 10;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_FORCE_10_FULL_DUPLEX:
        priv->force_duplex   = 2;
        priv->speed          = 10;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_FORCE_100_HALF_DUPLEX:
        priv->force_duplex  = 1;
        priv->speed         = 100;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_FORCE_100_FULL_DUPLEX:
        priv->force_duplex  = 2;
        priv->speed         = 100;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_HD:
        priv->force_duplex  = 1;
        priv->speed         = 10;
        priv->force_mode    = 0;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_10_FD:
        priv->force_duplex  = 2;
        priv->speed         = 10;
        priv->force_mode    = 0;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100_HD:
        priv->force_duplex  = 1;
        priv->speed         = 100;
        priv->force_mode    = 0;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_100FD:
        priv->force_duplex  = 2;
        priv->speed         = 100;
        priv->force_mode    = 0;
        break;

    case NVENET_FORCE_SPEED_DUPLEX_AUTO_NEGOTIATE_FOR_1000FD:
        priv->force_duplex  = 2;
        priv->speed         = 1000;
        priv->force_mode    = 0;
        break;

    default:
        priv->force_duplex  = 0;
        priv->speed         = 0;
        priv->force_mode    = 0;
        break;
    }
    return 0;
}

/*
 * PROC File system support routines
 */


static NVENET_STATUS
create_nvenet_proc_entries(struct net_device *dev)
{

    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct proc_dir_entry *entry;
    char buffer[80];

    if (NULL == nvenet_proc_root)
    {
        PRINTK_ERROR("nvnet: proc root doesn't exist\n");
        return NVENET_STATUS_FAILURE;
    }
    /*
     * Full information
     */

    sprintf(buffer, "%s.info", dev->name);

    entry = create_proc_entry(
                buffer, 
                S_IFREG,
                nvenet_proc_root
                );

    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }
    
    entry->proc_fops = &nvenet_proc_full_info_fops;
    entry->data      = dev;

    /*
     * Device name directory
     */
    priv->proc_entry = create_proc_entry(    
                            dev->name,
                            S_IFDIR,
                            nvenet_proc_root
                            );

    if (NULL == priv->proc_entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }
    /*
     * command line parameters
     */

    entry = create_proc_entry(
                "command_line",
                 S_IFREG,
                 priv->proc_entry
                 );
    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }
    entry->proc_fops    = &nvenet_proc_command_line_fops;
    entry->data         = dev;



    /*
     * Hardware information
     */
    entry = create_proc_entry(
                "hardware_info",
                 S_IFREG,
                 priv->proc_entry
                 );
    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }
    entry->proc_fops    = &nvenet_proc_hardware_info_fops;
    entry->data         = dev;

    /*
     * Configuration
     */
    entry = create_proc_entry(
                "configuration",
                 S_IFREG,
                 priv->proc_entry
                 );

    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }
    entry->proc_fops    = &nvenet_proc_configuration_fops;
    entry->data         = dev;


    /*
     * Tx stats
     */
    entry = create_proc_entry(
                "tx_stats",
                 S_IFREG,
                 priv->proc_entry
                 );

    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }

    entry->proc_fops    = &nvenet_proc_tx_stats_fops;
    entry->data         = dev;
    /*
     * Rx stats
     */
    entry = create_proc_entry(
                "rx_stats",
                 S_IFREG,
                 priv->proc_entry
                 );

    if (NULL == entry)
    {
        PRINTK_ERROR("nvnet: create_nvenet_proc_entries - create_proc_entry failed\n");
        return NVENET_STATUS_FAILURE;
    }

    entry->proc_fops    = &nvenet_proc_rx_stats_fops;
    entry->data         = dev;

    return NVENET_STATUS_SUCCESS;

}

static void
remove_nvenet_proc_entries(struct net_device *dev)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    char buffer[80];

    remove_proc_entry("tx_stats", priv->proc_entry);
    remove_proc_entry("rx_stats", priv->proc_entry);
    remove_proc_entry("configuration", priv->proc_entry);
    remove_proc_entry("hardware_info", priv->proc_entry);
    remove_proc_entry("command_line", priv->proc_entry);

    sprintf(buffer, "%s.info", dev->name);
    remove_proc_entry(buffer, nvenet_proc_root);
    remove_proc_entry(dev->name, nvenet_proc_root);
}

static int
nvenet_proc_full_info_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_full_info_open - In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_proc_full_info_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));

    length  = 0;

    length += proc_fill_hardware_info(  
                    dev,
                    proc_data->buffer + length,
                    sizeof(proc_data->buffer) - length
                    );

    length += proc_fill_configuration(
                    dev,
                    proc_data->buffer + length,
                    sizeof(proc_data->buffer) -length
                    );

    length += proc_fill_tx_stats(
                    dev,    
                    proc_data->buffer + length, 
                    sizeof(proc_data->buffer) - length
                    );

    length += proc_fill_rx_stats(
                    dev,
                    proc_data->buffer + length,
                    sizeof(proc_data->buffer) - length
                    );

    PRINTK(NV_DBGLEVEL_INFO, "Size of PROC = 0x%x\n", length);

    if (length < 0)
    {
        kfree(proc_data);
        return -ENOMEM;
    }
    
    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_full_info_open - Out\n");
    return 0;

}

static int
nvenet_proc_hardware_info_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_hardware_info_open- In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_proc_hardware_info_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));
    length = proc_fill_hardware_info(
                dev,
                proc_data->buffer,
                sizeof(proc_data->buffer)
                );
      
    if (length < 0)
    {
        PRINTK_ERROR("nvnet: proc_fill_hardware_info returned negative length\n");
        return -ENOMEM; 
    }
    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_hardware_info_open -  Out\n");
    return 0;
}

static int
nvenet_proc_command_line_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_cmdline_open: In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_proc_command_line_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));

    length = proc_fill_command_line_info(
                dev,
                proc_data->buffer,
                sizeof(proc_data->buffer)
                );

    if (length < 0)
    {
        kfree(proc_data);
        return -ENOMEM;
    }

    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_cmdline_open: Out\n");
    return 0;
}


static int
nvenet_proc_configuration_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_configuration_open - In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_proc_configuration_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));

    length =  proc_fill_configuration(
                dev,
                proc_data->buffer,
                sizeof(proc_data->buffer)
                );
    
    if (length < 0)
    {
        PRINTK_ERROR("nvnet: nvenet_proc_configuration_open - proc_fill_configuration returned -ve length\n");
        return -ENODEV;
    }

    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_configuration_open - Out\n");
    return 0;

}

static int
nvenet_proc_rx_stats_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_rx_stats_open - In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_proc_rx_stats_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));

    length = proc_fill_rx_stats(
                    dev,
                    proc_data->buffer,  
                    sizeof(proc_data->buffer)
                    );

    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC,"nvnet: nvenet_proc_rx_stats_open - Out\n");
    return 0;

}

static int
nvenet_proc_tx_stats_open(struct inode *inode, struct file *file)
{
    struct nvenet_proc_data *proc_data;
    struct proc_dir_entry *entry = PDE(inode);
    struct net_device *dev = entry->data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC,"nvnet: nvenet_proc_tx_stats_open - In\n");

    proc_data = kmalloc(sizeof(struct nvenet_proc_data), GFP_KERNEL);
    
    if (NULL == proc_data)
    {
        PRINTK_ERROR("nvnet: nvnenet_tx_stats_open - kmalloc failed\n");
        return -ENOMEM;
    }

    file->private_data = (void *)proc_data;  
    memset(proc_data,0, sizeof(struct nvenet_proc_data));

    length = proc_fill_tx_stats(
                    dev,    
                    proc_data->buffer, 
                    sizeof(proc_data->buffer)
                    );

    if (length < 0)
    {
        kfree(proc_data);
        return -ENOMEM;
    }

    proc_data->length = length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_tx_stats_open - Out\n");
    return 0;

}

static int
nvenet_proc_release(struct inode *inode, struct file *file)
{
    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_close - IN\n");

    if (file->private_data)
    {
        kfree(file->private_data);
    }

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_close - Out\n");
    return 0;
} 

static ssize_t
nvenet_proc_read(struct file *file, char *buffer, size_t len, 
                 loff_t *offset)
{
    struct nvenet_proc_data *proc_data;
    int length;

    PRINTK(NV_DBGLEVEL_PROC, "nvnet: nvenet_proc_read - In \n");

    proc_data = file->private_data;
    if (NULL == proc_data)
    {   
        PRINTK_ERROR("nvnet: nvenet_proc_read - proc_data is NULL\n");
        return 0;
    }

    length = NVENET_MIN(len, proc_data->length - *offset);

    if (copy_to_user(buffer, proc_data->buffer + *offset, length))
    {
        PRINTK_ERROR("nvnet: nvenet_proc_read - copy_to_user failed\n");
        return -EFAULT;
    }
    
    *offset += length;

    return length;
}


static int
proc_fill_tx_stats(struct net_device *dev, char *buffer, int bufferlen)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    ADAPTER_STATS stats;
    int length, i;

    priv->pADApi->pfnGetStatistics(priv->pADApi->pADCX, &stats);

    length = 0;
    length += sprintf(buffer + length,
                      "\n******************** TRANSMISSION STATISTICS *********************\n");

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "SuccessfulTx", stats.ulSuccessfulTransmissions);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "FailedTx", stats.ulFailedTransmissions);
    
    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "RetryErrors", stats.ulRetryErrors);
    
    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "UnderflowErrors", stats.ulUnderflowErrors);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "LossOfCarrierErrors", stats.ulLossOfCarrierErrors);
    
    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "LateCollisionErrors", stats.ulLateCollisionErrors);
   
    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "DeferredTx", stats.ulDeferredTransmissions);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "ExcessDeferredTx", stats.ulExcessDeferredTransmissions);

    if (priv->tx_checksum_offload)
    {
        length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                         "TxChecksumOffloaded", priv->tx_checksum_done_count);
    }

    length += sprintf(buffer + length, "%-*.*s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "SuccessfulTxAfterCollsion-");

    for (i=0; i < MAX_TRANSMIT_COLISION_STATS; i++)
    {
        length += sprintf(buffer + length, "%-*.*s%2d%-*.*s: %llu\n", 5, 5, " ", i,
                          NVENET_MAX_FIELD_LENGTH - 7, NVENET_MAX_STRING_LENGTH - 7,
                          " Collision", stats.aulSuccessfulTransmitsAfterCollisions[i]);
    }

    return length;
}

static int
proc_fill_rx_stats(struct net_device *dev, char *buffer, int bufferlen)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    ADAPTER_STATS stats;
    int length;

    priv->pADApi->pfnGetStatistics(priv->pADApi->pADCX, &stats);

    length = 0;
    length += sprintf(buffer + length,
                      "\n******************** RECEIVE STATISTICS *********************\n");

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "SuccessfulRx", stats.ulSuccessfulReceptions);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "FailedRx", stats.ulFailedReceptions);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "CRCErrors", stats.ulCRCErrors);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "LengthErrors", stats.ulLengthErrors);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "FramingErrors", stats.ulFramingErrors);

    length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "OverFlowErrors", stats.ulOverFlowErrors);

    if (NVENET_HW_MODE_2 == priv->hwmode)
    {
        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxExtraByteCount", stats.ulRxExtraByteCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxFrameTooLongCount", stats.ulRxFrameTooLongCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxFrameAlignmentErrorCount", stats.ulRxFrameAlignmentErrorCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxLateCollisionCount", stats.ulRxLateCollisionErrors);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxRuntPacketCount", stats.ulRxRuntPacketErrors);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxUnicastFrameCount", stats.ulRxUnicastFrameCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxMulticastFrameCount", stats.ulRxMulticastFrameCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxBroadcastFrameCount", stats.ulRxBroadcastFrameCount);

        length += sprintf(buffer + length, "%-*.*s: %llu\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxPromiscuousModeFrameCount", stats.ulRxPromiscuousModeFrameCount);

        if (priv->rx_checksum_offload)
        {
            length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                              "RxChecksumSuccess", priv->rx_checksum_success_count);

            length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                              "RxChecksumFailure", priv->rx_checksum_failure_count);

            length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                              "RxChecksumNotIPPacket", priv->rx_checksum_not_ip_count);
        }
    }

    return length;
}


static int
proc_fill_command_line_info(struct net_device *dev, char *buffer, 
                            int bufferlen)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct nvenet_parameter *param;
    int count, i, length;
    int found = 0;  
    int index = 0;

    if (NVENET_HW_MODE_1 == priv->hwmode)
    {
        param   = nvenet_mode_1_parameters;
        count   = sizeof(nvenet_mode_1_parameters)/
                  sizeof(struct nvenet_parameter);
    }
    else 
    {
        param   = nvenet_mode_2_parameters;
        count   = sizeof(nvenet_mode_2_parameters)/
                  sizeof(struct nvenet_parameter);

    }

    length = 0;
    length += sprintf(buffer + length,
                      "\n****************** COMMAND LINE PARAMETERS ******************\n");

                
    for (i=0; i < count; i++, param++)
    {
        
        if (-1 == *param->user_value)
        {
            continue;
        }
        else
        {
            found = 1;

            if (FALSE == param->is_available)
            {

                length += sprintf(
                             buffer + length,
                            "%u) %-*.*s: %s is not available in current hardware mode %u\n",
                            index++,
                            20,
                            20,
                            param->name,
                            param->name,
                            priv->hwmode
                            );
                continue;

            }

            if ((*param->user_value < param->min_value) ||
                (*param->user_value > param->max_value))
            {
                if (param->type == NVENET_PARAM_TYPE_INDEX)
                {
                    length += sprintf(
                                buffer + length,
                                "%u) %-*.*s:Value %u is not correct. Setting %s to default mode %s\n",
                                index++,
                                20,
                                20,
                                param->name,
                                *param->user_value,
                                param->name,
                                param->str[param->default_value]
                                );
                }
                else
                {

                    length += sprintf(
                                buffer + length,
                                "%u) %-*.*s:Value %u is not correct. Setting %s to default value %u\n",
                                index++,
                                20,
                                20,
                                param->name,
                                *param->user_value,
                                param->name,
                                param->default_value
                                );
                }
            }
            else
            {
                if (param->type == NVENET_PARAM_TYPE_INDEX)
                {
                    length += sprintf(
                                buffer + length,
                                "%u) %-*.*s: Using command line value %u. Setting %s to mode %s\n",
                                index++,
                                20,
                                20,
                                param->name,
                                *param->user_value,
                                param->name,
                                param->str[*param->user_value]
                                );
                }
                else
                {
                    length += sprintf(
                                buffer + length,
                                "%u) %-*.*s: Using command line value %u. Setting %s to value %u\n",
                                index++,
                                20,
                                20,
                                param->name,
                                *param->user_value,
                                param->name,
                                *param->user_value
                                );
                }
            }
        }
    
    }

    if (!found)
    {

        length += sprintf(
                    buffer + length,
                    "No command line parameters specified\n"    
                    );
    }

    return length;
}

static int
proc_fill_hardware_info(struct net_device *dev, char *buffer, 
                        int bufferlen)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    char revision_id;
    int length;
    u32 current_state = 0;
    char *state[] = {"D0", "D1", "D2", "D3"};
    

    length = 0;
    length += sprintf(buffer + length,
                      "\n******************** HARDWARE INFORMATION  *********************\n");
    
    length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "BusInterface", "PCI");

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "BusNumber", priv->pdev->bus->number);

    length += sprintf(buffer + length, "%-*.*s: 0x%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                     "VendorID", priv->pdev->vendor);

    length += sprintf(buffer + length, "%-*.*s: 0x%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                     "DeviceID", priv->pdev->device);

    pci_read_config_byte(priv->pdev, PCI_REVISION_ID, &revision_id);

    length += sprintf(buffer + length, "%-*.*s: 0x%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                     "RevisionID", revision_id & 0xFF);

    length += sprintf(buffer + length, "%-*.*s: 0x%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                     "SubsystemVendorID", priv->pdev->subsystem_vendor);

    length += sprintf(buffer + length, "%-*.*s: 0x%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                     "SubsystemDeviceID", priv->pdev->subsystem_device);

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "IRQ", priv->pdev->irq);

    length += sprintf(buffer + length, "%-*.*s: 0x%lx\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "BaseMemory", pci_resource_start(priv->pdev,0));

    length += sprintf(buffer + length, "%-*.*s: %ld\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "BaseMemorySize", pci_resource_len(priv->pdev, 0));

    current_state = priv->pdev->current_state;
    if (current_state > 3)
        current_state = 3;

    length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "CurrentPowerState", state[current_state]);

    return length;
}

static int
proc_fill_configuration(struct net_device *dev, char *buffer, 
                        int bufferlen)
{
    struct nvenet_private *priv = (struct nvenet_private *)dev->priv;
    struct dev_mc_list *mcptr;
    int length; 
    NV_UINT32 speed;
    NV_UINT32 i;
    NV_UINT32 linkup;

    length = 0;

    length += sprintf(buffer + length,
                      "\n***************** CONFIGURATION INFORMATION *******************\n");

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "HardwareMode", priv->hwmode);

    length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "Optimization", nvenet_optimization_str[priv->optimization]);

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "MTU", priv->mtu);

    length += sprintf(buffer + length, "%-*.*s: %x:%x:%x:%x:%x:%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "PermanentMACAddress",
                      priv->permanent_address[5], priv->permanent_address[4],
                      priv->permanent_address[3], priv->permanent_address[2],
                      priv->permanent_address[1], priv->permanent_address[0]);

    length += sprintf(buffer + length, "%-*.*s: %x:%x:%x:%x:%x:%x\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "CurrentMACAddress",
                      dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
                      dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

    priv->pADApi->pfnGetLinkState(priv->pADApi->pADCX, &linkup);

    length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "Link", linkup == TRUE ? "up" : "down");

    if (TRUE == linkup)
    {
        speed = priv->pADApi->pfnGetLinkSpeed(priv->pADApi->pADCX);

        length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "LinkSpeed",speed);
    }

    length += sprintf(buffer + length, "%-*.*s: ", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "Filter");

    if (priv->hwfilter.ulFilterFlags & ACCEPT_ALL_PACKETS)
    {
        length += sprintf( buffer + length, "promiscuous");
    }
    else
    {
        if (priv->hwfilter.ulFilterFlags & ACCEPT_MULTICAST_PACKETS)
        {
            length += sprintf( buffer + length, "unicast");
        }

        if (priv->hwfilter.ulFilterFlags & ACCEPT_BROADCAST_PACKETS)
        {
            length += sprintf( buffer + length, " broadcast");
        }

        if (priv->hwfilter.ulFilterFlags & ACCEPT_MULTICAST_PACKETS)
        {
            length += sprintf( buffer + length, " multicast");
        }
    }

    length += sprintf( buffer + length, "\n");
    
    if (dev->mc_list)
    {
        length += sprintf(buffer + length, "%-*.*s- \n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "MulticastAddresses -");
        mcptr = dev->mc_list;

        for (i =0, mcptr = dev->mc_list; mcptr; mcptr = mcptr->next, i++)
        {
            length += sprintf(buffer + length, "%-*.*sAddr %u%-*.*s: %x:%x:%x:%x:%x:%x \n",
                        5,5, "", i, NVENET_MAX_FIELD_LENGTH -11, NVENET_MAX_STRING_LENGTH -11, " ",
                        mcptr->dmi_addr[0], mcptr->dmi_addr[1], mcptr->dmi_addr[2],
                        mcptr->dmi_addr[3], mcptr->dmi_addr[4], mcptr->dmi_addr[5]);
        }
    }
    

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "TxQueueSize", priv->max_tx_packets);

    length += sprintf(buffer + length, "%-*.*s: %u\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "RxQueueSize", priv->max_rx_packets);

    length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                      "ScatterGather", "enabled");

    if (NVENET_HW_MODE_2 == priv->hwmode)
    {

        length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "TxChecksumOffload", nvenet_tx_checksum_offload_str[priv->tx_checksum_offload]);

        length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "RxChecksumOffload", nvenet_rx_checksum_offload_str[priv->rx_checksum_offload]);

#ifdef NETIF_F_TSO

        length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "SegmentationOffload", nvenet_seg_offload_str[priv->seg_offload]);
#endif
        
        length += sprintf(buffer + length, "%-*.*s: %s\n", NVENET_MAX_FIELD_LENGTH, NVENET_MAX_STRING_LENGTH,
                          "MediaType", nvenet_media_str[priv->media]);
    }

    return length;
}
