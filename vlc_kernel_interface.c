#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */
#include <linux/init.h>     /* Needed for the macros */

#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/kthread.h>

//#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/inet.h>
#include <linux/netdevice.h>   /* net_device, net_device_stats etc */
#include <linux/etherdevice.h> /* Ethernet device standards. */
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/if_arp.h>	/* For ARPHRD_ETHER */
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/percpu.h>
#include <net/net_namespace.h>
#include <linux/u64_stats_sync.h>

#include <rtdm/rtdm_driver.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>

#include "dbg.h"
#include "vlc_kernel_interface.h"
#include "vlc_timer_handler.h"
#include "vlc_packet_handler.h"

MODULE_AUTHOR("Albin Severinson");
MODULE_LICENSE("GPL");

//The VLC device
struct net_device *vlc_dev;

struct net_device *get_vlc_dev()
{
  return vlc_dev;
}

/* Open the VLC device. Runs when ifconfig "ups" the device.
 */
int vlc_open(struct net_device *dev)
{
  printk(KERN_INFO "VLC: Opening device.\n");

  memcpy(dev->dev_addr, VLC_MAC_ADDR, ETH_ALEN);
  netif_start_queue(dev);
  return 0;
}

/* Release the VLC device. Runs when ifconfig "downs" the device.
 */
int vlc_release(struct net_device *dev)
{
  printk(KERN_INFO "VLC: Releasing device.\n");

  netif_stop_queue(dev);
  return 0;
}

/* Called by the kernel whenever it wants to send a packet.
 */
int vlc_tx(struct sk_buff *skb, struct net_device *dev)
{
  int rc = 0;
  debug("VLC TX: Outgoing packet recieved [%d]!\n", skb->len);

  //Place the skb in the care of the packet management subsystem
  rc = place_skb(skb);
  if(rc != 0) goto error;

  //Stop receiving packets
  netif_stop_queue(vlc_dev);

  return 0;

 error:
  dev_kfree_skb(skb);
  return -1;
}

/* Should be used to send received packets to upper layers. Not
 * currently used.
 */
void vlc_rx(struct net_device *dev, struct vlc_packet _pkt)
{
  return;
}

/* Struct for keeping track of the device stats.
 */
struct net_device_stats *vlc_stats(struct net_device *dev)
{
  struct vlc_priv *priv = netdev_priv(dev);
  return &priv->stats;
}

/* Struct for keeping track of which functions the kernel should call
 * for what job.
 */
static const struct net_device_ops vlc_netdev_ops = {
  .ndo_open = vlc_open,
  .ndo_stop = vlc_release,
  .ndo_start_xmit = vlc_tx,
  .ndo_get_stats = vlc_stats,
};

/* 
 * Initialize the VLC driver module. Runs at module insertion.
 */
static int __init vlc_init_module(void)
{
  int ret = -ENOMEM;
  struct vlc_priv *priv = NULL;

  printk(KERN_INFO "VLC: Initializing module...\n");

  //Allocate a net_device. This function allocates it with the
  //etherdevice standard values.
  vlc_dev = alloc_etherdev(sizeof(struct vlc_priv));
  if(!vlc_dev) goto out_free_netdev;

  //Override some of the values.
  vlc_dev->netdev_ops = &vlc_netdev_ops;
  //vlc_dev->flags |= IFF_NOARP;
  vlc_dev->features |= NETIF_F_HW_CSUM;

  //Init the vlc_priv struct
  priv = netdev_priv(vlc_dev);
  memset(priv, 0, sizeof(struct vlc_priv));
  priv->dev = vlc_dev;

  //Setup the packet buffer rings
  ret = setup_packet_handler(20);
  if(ret == -1) goto out_free_netdev;

  //Setup the GPIO
  if ( gpio_request(GPIO_OUTPUT, "GPIO_OUTPUT")
       || gpio_request(GPIO_INPUT, "GPIO_INPUT") ) {
    printk("VLC: Request GPIO failed!\n");
    goto out_free_netdev;
  }

  //TODO: Check return value.
  gpio_direction_output(GPIO_OUTPUT, GPIOF_INIT_HIGH);
  gpio_direction_input(GPIO_INPUT);

  //Init the timer handler subsystem
  ret = setup_timer_handler();
  if(ret != 0) goto out_free_netdev;

  //Register the net device.
  ret = register_netdev(vlc_dev);
  if(ret) goto out_free_netdev;
  
  printk(KERN_INFO "VLC: Module initialized!\n");

  return 0;

 out_free_netdev:
  printk(KERN_INFO "VLC: Module init failed.\n");

  free_netdev(vlc_dev);

  teardown_pool();

  return ret;
}

/*
 * Cleanup the VLC driver module. Runs at module removal.
 */
static void __exit vlc_cleanup_module(void)
{
  printk(KERN_INFO "VLC: Removing module...\n");

  /*
  //Stop thread
  if(task_rx_tx){
    //Destroy the packet handler sem so that it'll return
    //packet_handler_sem_destroy();

    //Stop the packet handler
    kthread_stop(task_rx_tx);
    task_rx_tx = NULL;
  }
  */

  //Unregister and free net device
  if(vlc_dev){
    unregister_netdev(vlc_dev);
    free_netdev(vlc_dev);
  }

  //Stop and cleanup the timers
  cleanup_timer_handler();

  //Clean up the packet buffers
  teardown_pool();

  gpio_free(GPIO_OUTPUT);
  gpio_free(GPIO_INPUT);

  printk(KERN_INFO "VLC: Module removed\n");

  return;
}

/* Define the routines to be run when the module is inserted and
 * removed respetively.
 */
module_init(vlc_init_module);
module_exit(vlc_cleanup_module);
