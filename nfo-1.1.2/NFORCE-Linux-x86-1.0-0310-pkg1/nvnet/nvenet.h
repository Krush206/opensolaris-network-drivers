/************************************************************************ \
|*                                                                       *|
|*                                                                       *|
|*  Copyright 2001 NVIDIA Corporation. All rights reserved               *| 
|*                                                                       *|
|*  THE INFORMATION CONTAINED HEREIN IS PROPRIETARY AND CONFIDENTIAL     *|
|*  TO NVIDIA, CORPORATION. USE, REPORDUCTION OR DISCLOSURE TO ANY       *|
|*    THIRD PARTY IS SUBJECT TO WRITTEN PRE-APPROVAL BY NVIDIA CORP.     *|
|*                                                                       *|
|*    THE INFORMATION CONTAINED HEREIN IS PROVIDED "AS IS" WITHOUT       *|
|*    EXPRESS OR IMPLIED WARRANTY OF ANY KIND, INCLUDING ALL IMPLIED     *|
|*    WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A  *|
|*    PARTICULAR PURPOSE.                                                *|
|*                                                                       *|
**************************************************************************/

#ifndef __NVENET_H__
#define	__NVENET_H__ 

#include <linux/module.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/mii.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <asm/io.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/irq.h>            
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>

/*********************************************************************
                    Macros
**********************************************************************/
static int
nvenet_reboot_handler(struct notifier_block *nb, unsigned long event, void *p);

/*
 * Status definitions
 */
typedef NV_SINT32   NVENET_STATUS;
#define NVENET_STATUS_FAILURE          0
#define NVENET_STATUS_SUCCESS          1

/*
 * Resource allocation defintions
 */
#define NVENET_PCI_ENABLE_DONE              0x00000001
#define NVENET_REQUEST_MEM_REGION_DONE      0x00000002
#define NVENET_IOREMAP_DONE                 0x00000004
#define NVENET_ALLOC_ETHERDEV_DONE          0x00000008
#define NVENET_TX_RING_ALLOCATED            0x00000010
#define NVENET_ADAPTER_OBJECT_CREATED       0x00000020

/*
 * Maximum Tx ring entries
 */
#define NVENET_MIN_TX_PACKETS           32
#define NVENET_MAX_TX_PACKETS           1024
#define NVENET_DEFAULT_TX_PACKETS       64

#define NVENET_MIN_RX_PACKETS           32
#define NVENET_MAX_RX_PACKETS           1024
#define NVENET_DEFAULT_RX_PACKETS       64

#define NV_MIN_TX_RING_ENTRIES         32
#define NV_DEFAULT_TX_RING_ENTRIES     64
#define NV_MAX_TX_RING_ENTRIES         1024

/*
 * nVidia NVENET PCI device identifier
 */
#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_1
#define PCI_DEVICE_ID_NVIDIA_NVENET_1    0x01C3
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_2
#define PCI_DEVICE_ID_NVIDIA_NVENET_2    0x0066
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_3
#define PCI_DEVICE_ID_NVIDIA_NVENET_3   0x00D6
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_4	
#define PCI_DEVICE_ID_NVIDIA_NVENET_4	0x0086
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_5	
#define PCI_DEVICE_ID_NVIDIA_NVENET_5	0x008C
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_6	
#define PCI_DEVICE_ID_NVIDIA_NVENET_6	0x00E6
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_7	
#define PCI_DEVICE_ID_NVIDIA_NVENET_7	0x00DF
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_8
#define PCI_DEVICE_ID_NVIDIA_NVENET_8   0x0056
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_9
#define PCI_DEVICE_ID_NVIDIA_NVENET_9   0x0057
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_10
#define PCI_DEVICE_ID_NVIDIA_NVENET_10  0x0037
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_11
#define PCI_DEVICE_ID_NVIDIA_NVENET_11  0x0038
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_12
#define PCI_DEVICE_ID_NVIDIA_NVENET_12  0x0268
#endif

#ifndef PCI_DEVICE_ID_NVIDIA_NVENET_13
#define PCI_DEVICE_ID_NVIDIA_NVENET_13  0x0269
#endif

// pci_name()
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,22)
#define pci_name(pdev) pdev->slot_name
#endif

//#define NVENET_DEBUG                   1

/*
 * Debug controls
 */

#define NV_DBGLEVEL_INIT        0x00000001
#define NV_DBGLEVEL_OPEN        0x00000002
#define NV_DBGLEVEL_TX          0x00000004
#define NV_DBGLEVEL_INTR        0x00000008
#define NV_DBGLEVEL_RX          0x00000010
#define NV_DBGLEVEL_IOCTL       0x00000020
#define NV_DBGLEVEL_LOCK        0x00000040
#define NV_DBGLEVEL_STATS       0x00000080
#define NV_DBGLEVEL_LINK        0x00000100
#define NV_DBGLEVEL_PROBE       0x00000200
#define NV_DBGLEVEL_CLOSE       0x00000400
#define NV_DBGLEVEL_CSUM        0x00000800
#define NV_DBGLEVEL_PROC        0x00001000
#define NV_DBGLEVEL_POWER       0x00002000
#define NV_DBGLEVEL_WATCHDOG    0x00004000
#define NV_DBGLEVEL_REBOOT      0x00008000
#define NV_DBGLEVEL_INFO        0x00010000
#define NV_DBGLEVEL_ALL         0xFFFFFFFF


#ifdef NVENET_DEBUG
static unsigned long nvenet_debug = 0 ;


#define PRINTK(level, fmt, args...) if (nvenet_debug & level) \
    printk(KERN_EMERG "nvnet: " fmt, ## args)
#define NVENET_ASSERT(A)  if (!(A)) \
    printk(KERN_EMERG "Assertion failed line %x, file %s\n", __LINE__, __FILE__)

#define PRINTK_ERROR(fmt, args...) \
    printk(KERN_EMERG "nvnet: " fmt, ## args)
#else
#define PRINTK(level, fmt, args...)
#define PRINTK_ERROR(fmt, args...) 
#define NVENET_ASSERT(A) 
#endif

#define NVENET_OFFSET(field)   (NV_UINT32)(&((struct nvenet_private *)0)->field)
#define NVENET_MIN(a,b) ((a) < (b)? (a): (b))

//
// Macros used by proc file system
//
#define NVENET_MAX_FIELD_LENGTH         30
#define NVENET_MAX_STRING_LENGTH        30

//
// Parameter type
//
#define NVENET_PARAM_TYPE_INDEX      0
#define NVENET_PARAM_TYPE_VALUE      1

//
// FCS Len
//
#ifndef FCS_LEN
#define FCS_LEN   4
#endif

//
// Ethtool
//
#define ETHTOOL_SUPPORTED_INTERFACE     0xCF

//
// Interrupt handling
//
#ifndef IRQ_HANDLED
typedef void irqreturn_t;
#define IRQ_HANDLED
#define IRQ_NONE
#endif

//
// Proc filesystem
//
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,4)
#define PDE(inode) ((inode)->u.generic_ip)
#endif


/*********************************************************************
                    Structures
**********************************************************************/


struct nvenet_parameter
{

    char *name;
    NV_BOOLEAN is_available;
    int *user_value;
    int offset;
    int type;
    int size;
    int min_value;
    int max_value;
    int default_value;
    char **str;

} ;

struct nvenet_proc_data 
{
    NV_UINT32 length;
    char buffer[PAGE_SIZE];
};

struct nvenet_rx_info {

    struct sk_buff *skb;
    dma_addr_t dma;
    u32 uiLength;
    u32 reserved;
};

typedef struct _nvnet_tx_info_os 
{
    //Don't add anything above this structure
    TX_INFO_ADAP TxInfoAdap;
    struct _tx_info_os *Next;
    PVOID pvID;
    PVOID pCoalesceBuffer;
    unsigned long status;
    unsigned long entries;
    dma_addr_t dma_addr[MAX_SKB_FRAGS + 1];
    unsigned long len[MAX_SKB_FRAGS + 1];

} nvenet_tx_info_os_t;

struct nvenet_private 
{
    ADAPTER_API *pADApi;             
    OS_API linuxapi;          
    NV_UINT32 max_frame_size;  
    nvenet_tx_info_os_t *tx_ring;
    NV_UINT32 tx_ring_full;
    NV_UINT32 tx_ring_index;
    NV_UINT32 tx_ring_busy_count;
    NV_UINT32 tx_ring_size;
    NV_UINT32 hwmode;
    NV_UINT32 hpna;          
    char *regs;
    NV_UINT32 phyaddr;       
    int linkup;         
    int linkspeed;    
    spinlock_t lock;
    spinlock_t phylock;         
    unsigned long lockflags;     
    dma_addr_t tx_ring_dma;
    NV_UINT8 permanent_address[6+2];
    NV_UINT8 reversed_address[6+2];
    struct pci_dev *pdev;           
    CMNDATA_OS_ADAPTER adapterdata;
    PACKET_FILTER hwfilter;   
    struct proc_dir_entry *proc_entry;
#ifdef CONFIG_PM
    uint32_t pci_state[16];
#endif
    NV_UINT32 speed;
    NV_UINT32 force_mode;
    NV_UINT32 force_duplex;
	NV_UINT32 async_mode;

    NV_UINT32 max_tx_packets;
    NV_UINT32 max_rx_packets;

    /*
     * Statistics
     */
    struct net_device_stats stats;
    NV_UINT32 rx_checksum_success_count;
    NV_UINT32 rx_checksum_failure_count;
    NV_UINT32 rx_checksum_not_ip_count;
    NV_UINT32 tx_checksum_done_count;

    /* 
     * Module parameters
     */
    NV_UINT32 poll_interval_in_us;
    NV_UINT32 force_speed_duplex;
    NV_UINT32 auto_negotiate;
    NV_UINT32 optimization;
    NV_UINT32 mtu;
    NV_UINT32 media;
    NV_UINT32 seg_offload;
    NV_UINT32 tx_checksum_mask;
    NV_UINT32 tx_checksum_offload;
    NV_UINT32 rx_checksum_offload;
};

/*********************************************************************
                        Function Prototypes
**********************************************************************/

static int 
nvenet_open(struct net_device *dev);

static int 
nvenet_close(struct net_device *dev);

static int 
nvenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);

static struct net_device_stats *
nvenet_stats(struct net_device *dev);

static int
nvenet_xmit(struct sk_buff *skb, struct net_device *dev);

static int 
nvenet_change_mtu(struct net_device *dev, int mtu);

static irqreturn_t
nvenet_interrupt(int irq, void *dev_instance, struct pt_regs *regs);

static void 
nvenet_multicast(struct net_device *dev);

static int
nvenet_mac_address(struct net_device *dev, void *addr);

static int __devinit 
nvenet_probe(struct pci_dev *pdev, const struct pci_device_id *entry);

int 
nvenet_resume(struct pci_dev *pcidev);

int 
nvenet_suspend(struct pci_dev *pcidev, u32 state);

static void 
__devexit nvenet_remove(struct pci_dev *pdev);

#if 0
static void 
nvenet_watchdog(struct net_device *dev);
#endif

static int
create_adapter_object(struct net_device *dev);

static void
delete_adapter_object(PADAPTER_API pADApi);

static NV_API_CALL NV_SINT32
linuxlockalloc(void *ctx, NV_SINT32 type, void **pLock);

static NV_API_CALL NV_SINT32
linuxlockacquire(void *ctx, NV_SINT32 type, void *lock);

static NV_API_CALL NV_SINT32
linuxlockrelease(void *ctx, NV_SINT32 type, void *lock);

static NV_API_CALL NV_SINT32
linuxalloc(void *ctx, MEMORY_BLOCK *mem);

static NV_API_CALL NV_SINT32
linuxfree(void *ctx, MEMORY_BLOCK *mem);

static NV_API_CALL NV_SINT32
linuxallocex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex);

static NV_API_CALL NV_SINT32 
linuxfreeex(PNV_VOID ctx, MEMORY_BLOCKEX *mem_block_ex);

static NV_API_CALL NV_SINT32
linuxclear(PNV_VOID ctx, PNV_VOID mem, NV_SINT32 length);

static NV_API_CALL NV_SINT32
linuxdelay(void *ctx, NV_UINT32 usec);

static NV_API_CALL NV_SINT32
linuxallocrxbuf(void *ctx, MEMORY_BLOCK *mem, void **id) ;

static NV_API_CALL NV_SINT32
linuxfreerxbuf(void *ctx, MEMORY_BLOCK *mem, void *id);

static NV_API_CALL NV_SINT32
linuxpackettx(void *ctx, void *id, NV_UINT32 success);

static NV_API_CALL NV_SINT32
linuxpacketrx(PNV_VOID ctx, PNV_VOID data, NV_UINT32 success, 
              PNV_UINT8 newbuf, NV_UINT8 priority);

static NV_API_CALL NV_SINT32
linuxlinkchanged(void *ctx, NV_SINT32 enabled);

static void
rx_checksum_offload_handler(struct net_device *dev, struct sk_buff *skb,
                            struct nvenet_private *priv, 
                            ADAPTER_READ_DATA *readdata);


static int 
read_module_parameters(struct net_device *dev);


static int
nvenet_proc_full_info_open(struct inode *inode, struct file *file);

static int
nvenet_proc_command_line_open(struct inode *inode, struct file *file);

static int
nvenet_proc_hardware_info_open(struct inode *inode, struct file *file);

static int
nvenet_proc_configuration_open(struct inode *inode, struct file *file);

static int
nvenet_proc_rx_stats_open(struct inode *inode, struct file *file);

static int
nvenet_proc_tx_stats_open(struct inode *inode, struct file *file);


static int
nvenet_proc_release(struct inode *inode, struct file *file);

static ssize_t
nvenet_proc_read(struct file *file, char *buffer, size_t len, 
                 loff_t *offset);

static NVENET_STATUS
create_nvenet_proc_entries(struct net_device *dev);

static void
remove_nvenet_proc_entries(struct net_device *dev);

static int
proc_fill_tx_stats(struct net_device *dev, char *buffer, int length);

static int
proc_fill_rx_stats(struct net_device *dev, char *buffer, int bufferlen);

static int
proc_fill_command_line_info(struct net_device *dev, char *buffer, 
                            int bufferlen);

static int
proc_fill_hardware_info(struct net_device *dev, char *buffer, 
                        int bufferlen);

static int
proc_fill_configuration(struct net_device *dev, char *buffer, 
                        int bufferlen);

#endif

