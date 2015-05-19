#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <rtdm/rtdm_driver.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/kthread.h>

#include "dbg.h"
#include "vlc_kernel_interface.h"
#include "vlc_timer_handler.h"
#include "vlc_packet_handler.h"


/* Pointers to the packets currently being transmitted in the packet
 * ring.
 */
struct vlc_packet *current_tx_packet = NULL;
struct vlc_packet *current_rx_packet = NULL;

int tx_buffer_ring_size = 0;
int rx_buffer_ring_size = 0;

//Semaphore for keeping track of when the package handler should run
rtdm_sem_t packet_handler_sem;
rtdm_task_t packet_handler_task;
//static struct task_struct *packet_handler_task = NULL;

void packet_handler_sem_up(void)
{
  //rtdm_sem_up(&packet_handler_sem);
}

void packet_handler_sem_destroy(void)
{
  rtdm_sem_destroy(&packet_handler_sem);
}


struct vlc_packet *get_tx_packet()
{
  return current_tx_packet;
}

struct vlc_packet *get_rx_packet()
{
  return current_rx_packet;
}

/* Receive an skb from the kernel interface functions and place it
 * into a global variable. update_pools will pick up the skb and reset
 * the variable.
 */
struct sk_buff *placed_skb = NULL;
int place_skb(struct sk_buff *skb)
{
  //Return -1 if there's already an skb placed
  if(placed_skb) return -1;

  //Orphan the skb, so that we own it
  skb_orphan(skb);

  //Place the skb
  placed_skb = skb;

  //Spin while waiting for the handler to pick up the SKB, at which
  //point the skb will be set to NULL
  //while(placed_skb);

  return 0;
}

/* Return the currently placed skb. The kernel interface will use this
 * to check for when the placed skb has been picked up by the packet
 * management subsystem.
 */
struct sk_buff *get_skb()
{
  return placed_skb;
}

/* Loop through both pools, cleanup sent packets, and send received
 * packets to upper layers.
 */
void update_pools(void *arg)
{
  //Used for storing the current number of packets in the buffer rings
  int tx_packets = 0;
  u16 sequence = 0;

  struct vlc_packet *tx_packet = current_tx_packet;
  struct vlc_packet *rx_packet = current_rx_packet;
  struct vlc_priv *priv = netdev_priv(get_vlc_dev());
  struct net_device *vlc_dev = get_vlc_dev();

  int i = 0;

  debug("VLC: Packet handler running!\n");

  /* Loop through the packets. If one is NULL, there's been an error
   * and we should stop. We should also stop if the kernel has said
   * so.
   */
  while(tx_packet && rx_packet){
    //msleep(1);
    //debug("VLC: Update pools running!\n");
    rtdm_task_sleep(1000000000 / (FREQ /( 50 * 8 * 2)));


    /* Run one loop of the buffer ring.
     */
    for(i = 0;i < tx_buffer_ring_size;i++){

      /* If an RX packet is complete, we've received it from the
       * interface and it should be packaged and sent to the kernel. We
       * will also clean up the buffers so that the packet can be used
       * to receive another packet later.
       */
      if(rx_packet->state == PACKET_COMPLETE){

        //Create an empty sk_buff
        rx_packet->skb = dev_alloc_skb(rx_packet->rx_len);

        //Check that it got allocated
        if(!rx_packet->skb){
          if(printk_ratelimit()) debug("VLC: Low on mem - packet droped.\n");
          priv->stats.rx_dropped++;
          rx_packet->state = PACKET_CORRUPTED;
        }

        else{
          //Assign the packet length
          rx_packet->skb->len = rx_packet->rx_len;

          //Copy the data from the buffer into the sk_buff
          memcpy(skb_put(rx_packet->skb, rx_packet->rx_len), 
                 rx_packet->rx_buff, rx_packet->rx_len);

          //Zero out the buffer and packet length
          memset(rx_packet->rx_buff, 0, ETH_DATA_LEN);
          rx_packet->rx_len = 0;

          //Zero out the indexes
          rx_packet->bit_index = 0;
          rx_packet->byte_index = 0;
          rx_packet->current_preamble = 0;
      
          //Update statistics
          priv->stats.rx_packets++;
          priv->stats.rx_bytes += rx_packet->skb->len;

          //Write metadata to skb
          rx_packet->skb->dev = vlc_dev;
          rx_packet->skb->protocol = eth_type_trans(rx_packet->skb, vlc_dev);
          rx_packet->skb->ip_summed = CHECKSUM_NONE;

          debug("VLC RX: Packet passed to kernel [%d]!\n", rx_packet->skb->len);

          //Pass packet to upper lebels
          if(netif_rx(rx_packet->skb) != NET_RX_SUCCESS) 
            debug("VLC RX: Packet droped when passed to upper layers.\n");

          //The skb is the responsibility of the kernel now
          rx_packet->skb = NULL;

          //Set the packet status to ready again
          rx_packet->state = PACKET_READY;
        }
      }

      if(rx_packet->state == PACKET_CORRUPTED){
        //Zero out the buffer and packet length
        memset(rx_packet->rx_buff, 0, ETH_DATA_LEN);
        rx_packet->rx_len = 0;

        //Zero out the indexes
        rx_packet->bit_index = 0;
        rx_packet->byte_index = 0;
        rx_packet->current_preamble = 0;

        //Set the packet status to ready again
        rx_packet->state = PACKET_READY;        
      }

      /* If the packet is sent, and set to complete by the timer
       * handler, free the skb and update the stats. If we've previously
       * stoped the netif queue we start it again here.
       */
      if(tx_packet->state == PACKET_COMPLETE){
        debug("VLC TX: Packet removed from buffer [%d]!\n", tx_packet->skb->len);

        //Update stats
        priv->stats.tx_packets++;
        priv->stats.tx_bytes += tx_packet->skb->len;

        //Free the packet
        dev_kfree_skb_irq(tx_packet->skb);
        tx_packet->skb = NULL;
     
        //Zero out the indexes
        tx_packet->bit_index = 0;
        tx_packet->byte_index = 0;
        tx_packet->current_preamble = 0;

        //Set the packet status to empty. Packet is set to ready when
        //the kernel has provided an skb to transmit.
        tx_packet->state = PACKET_EMPTY;

        if(tx_packets == tx_buffer_ring_size) netif_wake_queue(vlc_dev);

        tx_packets--;
      }

      /* If the tx packet is empty, and there's an skb waiting, place
       * the skb into the packet and reset the placed skb.
       */
      if(tx_packet->state == PACKET_EMPTY && placed_skb){

        //Assign the SKB to the packet
        tx_packet->skb = placed_skb;

        //Set the packet state to ready
        tx_packet->state = PACKET_READY;

        tx_packets++;

        //Save the timestamp
        vlc_dev->trans_start = jiffies;

        debug("VLC TX: Picked up SKB! %d packets in buffer\n", tx_packets);

        placed_skb = NULL;

        //If the buffer isn't full, enable receiving packets again
        if(tx_packets < tx_buffer_ring_size){
          netif_wake_queue(vlc_dev);
        }
      }

      //Step to next packet in rings
      rx_packet = rx_packet->next;
      tx_packet = tx_packet->next;
    }
    //debug("VLC: Update pools finished!\n");
  }
  return;
}

/* Teardown the TX packet pool.
 */
int teardown_tx_pool(void)
{
  int i = 0;
  struct vlc_packet *next_packet = NULL;

  for(i = 0;i < tx_buffer_ring_size;i++){
    next_packet = current_tx_packet->next;
    kfree(current_tx_packet);
    current_tx_packet = next_packet;
  }

  return 0;
}

/* Teardown the RX pool.
 */
int teardown_rx_pool(void)
{
  int i = 0;
  struct vlc_packet *next_packet = NULL;

  for(i = 0;i < rx_buffer_ring_size;i++){
    next_packet = current_rx_packet->next;
    kfree(current_rx_packet);
    current_rx_packet = next_packet;
  }

  return 0;
}

/* Teardown both TX and RX pool.
 */
int teardown_pool()
{
  rtdm_sem_destroy(&packet_handler_sem);
  rtdm_task_destroy(&packet_handler_task);
  //kthread_stop(packet_handler_task);
  teardown_tx_pool();
  teardown_rx_pool();
  return 0;
}

int setup_tx_pool(int num)
{
  int i = 0;
  struct vlc_packet *initial_packet = NULL;
  debug("VLC: Setting up TX pool.\n");

  //Allocate the first packet
  current_tx_packet = kmalloc(sizeof(struct vlc_packet), GFP_KERNEL);
  if(!current_tx_packet) goto error;

  //Set the packet status to empty
  current_tx_packet->state = PACKET_EMPTY;

  //Zero out the indexes
  current_tx_packet->bit_index = 0;
  current_tx_packet->byte_index = 0;
  current_tx_packet->current_preamble = 0;
    
  //Set the device
  current_tx_packet->dev = get_vlc_dev();

  //Store the initial packet
  initial_packet = current_tx_packet;

  for(i = 0;i < num - 1;i++){
    
    current_tx_packet->next = kmalloc(sizeof(struct vlc_packet), GFP_KERNEL);
    current_tx_packet = current_tx_packet->next;
    if(!current_tx_packet) goto error;

    //Set the packet status to empty
    current_tx_packet->state = PACKET_EMPTY;

    //Zero out the indexes
    current_tx_packet->bit_index = 0;
    current_tx_packet->byte_index = 0;
    current_tx_packet->current_preamble = 0;
    
    //Set the device
    current_tx_packet->dev = get_vlc_dev();
  }

  //Close the loop
  current_tx_packet->next = initial_packet;

  //Store the ring size
  tx_buffer_ring_size = num;

  return 0;

 error:

  //Cleanup what we've done so far.
  debug("VLC: Error when setting up TX pool.\n");
  current_tx_packet = initial_packet;
  while(current_tx_packet){
    initial_packet = current_tx_packet->next;
    kfree(current_tx_packet);
    current_rx_packet = initial_packet;
  }

  return -1;
}

int setup_rx_pool(int num)
{
  int i = 0;
  struct vlc_packet *initial_packet = NULL;
  debug("VLC: Setting up RX pool.\n");

  //Allocate the first packet
  current_rx_packet = kmalloc(sizeof(struct vlc_packet), GFP_KERNEL);
  if(!current_rx_packet) goto error;

  //Set the packet status to ready
  current_rx_packet->state = PACKET_READY;

  //Zero out the indexes
  current_rx_packet->bit_index = 0;
  current_rx_packet->byte_index = 0;
  current_rx_packet->current_preamble = 0;

  //Set the device
  current_rx_packet->dev = get_vlc_dev();

  //Zero out the RX buffer and len
  memset(current_rx_packet->rx_buff, 0, ETH_DATA_LEN);
  current_rx_packet->rx_len = 0;

  //Store the first packet
  initial_packet = current_rx_packet;

  for(i = 0;i < num - 1;i++){
    
    current_rx_packet->next = kmalloc(sizeof(struct vlc_packet), GFP_KERNEL);
    current_rx_packet = current_rx_packet->next;
    if(!current_rx_packet) goto error;

    //Set the packet status to ready
    current_rx_packet->state = PACKET_READY;

    //Zero out the indexes
    current_rx_packet->bit_index = 0;
    current_rx_packet->byte_index = 0;
    current_rx_packet->current_preamble = 0;

    //Set the device
    current_rx_packet->dev = get_vlc_dev();

    //Zero out the RX buffer and len
    memset(current_rx_packet->rx_buff, 0, ETH_DATA_LEN);
    current_rx_packet->rx_len = 0;
  }

  //Close the loop
  current_rx_packet->next = initial_packet;

  //Store the ring size
  rx_buffer_ring_size = num;

  return 0;

 error:
  
  //Cleanup what we've done so far.
  debug("VLC: Error when setting up RX pool.\n");
  current_rx_packet = initial_packet;
  while(current_rx_packet){
    initial_packet = current_rx_packet->next;
    kfree(current_rx_packet);
    current_rx_packet = initial_packet;
  }
  return -1;
}

/* Setup the packet handling subsystem.
 */
int setup_packet_handler(int buffer_size)
{
  int rc = 0;

  //Init the semaphore
  rtdm_sem_init(&packet_handler_sem, 0);

  rc = setup_tx_pool(buffer_size);
  if(rc != 0) goto error;

  rc = setup_rx_pool(buffer_size);
  if(rc != 0) goto error;

  rtdm_task_init(&packet_handler_task, "VLC packet handler task", update_pools, 
                 NULL, RTDM_TASK_LOWEST_PRIORITY, 0);

  //packet_handler_task = kthread_run(update_pools, NULL, "VLC packet management");
  //if(IS_ERR(packet_handler_task)) goto error;

  return 0;

 error:
  debug("VLC: Error when seting up packet pool.\n");
  //if(packet_handler_task) kthread_stop(packet_handler_task);

  return -1;
}
