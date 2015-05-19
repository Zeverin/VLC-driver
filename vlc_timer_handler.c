#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <rtdm/rtdm_driver.h>
#include <rtdk.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/gpio.h>
#include <asm/div64.h>

#include "dbg.h"
#include "vlc_kernel_interface.h"
#include "vlc_timer_handler.h"
#include "vlc_packet_handler.h"

rtdm_task_t rx_handler_task;
rtdm_task_t tx_handler_task;

struct vlc_priv *priv = NULL;
struct vlc_packet *rx_packet = NULL;
struct vlc_packet *tx_packet = NULL;

u16 rx_sequence_num = 0;
u16 tx_sequence_num = 0;



/* Mask out a specific bit from an int.
 */
inline unsigned int mask_bit(unsigned int n, int bitnum)
{
  return (n & (1 << bitnum)) >> bitnum;
}

u32 rx_A = 1;
u32 rx_B = 0;
#define MOD_ADLER 65521
inline void rx_update_adler(u8 byte)
{
  rx_A = (rx_A + byte) % MOD_ADLER;
  rx_B = (rx_B + rx_A) % MOD_ADLER;
}

inline void rx_reset_adler(void)
{
  rx_A = 1;
  rx_B = 0;
}

u32 tx_A = 1;
u32 tx_B = 0;
inline void tx_update_adler(u8 byte)
{
  tx_A = (tx_A + byte) % MOD_ADLER;
  tx_B = (tx_B + tx_A) % MOD_ADLER;
}

inline void tx_reset_adler(void)
{
  tx_A = 1;
  tx_B = 0;
}

rtdm_sem_t rx_sem;
rtdm_sem_t tx_sem;

nanosecs_abs_t tx_sleep_slot;
nanosecs_abs_t rx_sleep_slot;
nanosecs_abs_t sleep_increment = 1000000000 / FREQ;
inline void send_bit(u8 bit)
{
  //rtdm_sem_down(&tx_sem);
  //debug("VLC TX: SEND BIT!\n");

  //Update the sleep slot to the next slot
  //while(tx_sleep_slot < rtdm_clock_read_monotonic()) tx_sleep_slot += sleep_increment;
  tx_sleep_slot += sleep_increment;

  //Wait until the next window
  rtdm_task_sleep_abs(tx_sleep_slot, RTDM_TIMERMODE_ABSOLUTE);

  gpio_set_value(GPIO_OUTPUT, bit);

  //rtdm_sem_up(&rx_sem);
}

/* Get a bit from the interface and apply the early-late
 * synchronization method. The input is sampled three times, split
 * evenly over the sleep slot. If the average of one of the edge
 * samples deviates, the slot is adjusted.
 */
#define EARLY_LATE_AVG_PERIOD 16
u32 early_late_avgs[3] = {0};
nanosecs_abs_t early_late_slot;
inline u8 get_bit_early_late(void)
{
  int i = 0;
  u8 bit = 0;
  u8 bit_average = 0;
  u32 early_late_sum_avg = 0;
  u32 early_late_avg_lower = 0;
  u32 early_late_avg_upper = 0;

  //Let the averages decay
  for(i = 0;i < 3;i++){
    early_late_avgs[i] = early_late_avgs[i] - early_late_avgs[i] / EARLY_LATE_AVG_PERIOD;
  }

  //Update the sleep increment
  rx_sleep_slot += sleep_increment;

  for(i = 0;i < 3;i++){    
    //Take 3 values equally spaced over the sleep slot
    rtdm_task_sleep_abs(rx_sleep_slot - (early_late_slot) + (early_late_slot * i),
                        RTDM_TIMERMODE_ABSOLUTE);

    //Get a bit
    bit = gpio_get_value(GPIO_INPUT);

    //Calculate the average of the current value
    bit_average += bit * 4;

    //Update the averages. Bump the resolution by multiplying with a
    //large number.
    early_late_avgs[i] += bit * 10000;

    //Add the numbers together
    early_late_sum_avg += early_late_avgs[i];
  }

  //Divide by 3 to get the average
  early_late_sum_avg /= 3;

  //Adjust the sleep slots if one of the edge values deviate from the
  //avgerage
  early_late_avg_lower = early_late_sum_avg * 3/4;
  early_late_avg_upper = early_late_sum_avg * 5/4;

  if(early_late_avgs[0] < early_late_avg_lower ||
     early_late_avgs[0] > early_late_avg_upper){
    rx_sleep_slot += early_late_slot;
    tx_sleep_slot += early_late_slot;

    debug("VLC SYNC: Adjusted sleep slot forwards.\n");
  }

  else if(early_late_avgs[2] < early_late_avg_lower || 
          early_late_avgs[2] > early_late_avg_upper){
    rx_sleep_slot -= early_late_slot;
    tx_sleep_slot -= early_late_slot;

    debug("VLC SYNC: Adjusted sleep slot backwards.\n");
  }

  //Return the average
  if(bit_average <= 6) return 0;
  else return 1;
}

inline u8 get_bit(void)
{
  //Update the sleep slot
  rx_sleep_slot += sleep_increment;

  //Wait until the next window
  rtdm_task_sleep_abs(rx_sleep_slot, RTDM_TIMERMODE_ABSOLUTE);

  return gpio_get_value(GPIO_INPUT);
}

inline void send_manchester_symbol(u8 tx_bit)
{
  //Send the first bit
  send_bit(tx_bit);

  //The second bit in a manchester symbol is the inverse of the first bit
  send_bit(tx_bit ^ 1);
}

inline u8 get_manchester_symbol(void)
{
  u8 rx_bit = get_bit();

  //If the manchester symbol is incorrect, assume the second bit was flipped
  if(get_bit() == rx_bit) return rx_bit;
  else return rx_bit;
}

inline void tx_send_ack(void)
{
  int i = 0;
  for(i = 0;i < 32;i++){
    send_bit(mask_bit((int) VLC_ACK, i));
  }
  if(rx_packet->state == RX_SENDING_ACK){

    rx_packet->state = PACKET_COMPLETE;

    /*
    //Check if this packet is a DUP
    if(rx_packet->sequence == rx_sequence_num){
      rx_packet->state = PACKET_COMPLETE;
      rx_sequence_num++;
    }
    
    //If it is, set it to corrupted
    else rx_packet->state == PACKET_CORRUPTED;
    */
  }
}

inline void tx_send_preamble(void)
{
  int i = 0;
  for(i = 0;i < 32;i++){
    send_bit(mask_bit((int) VLC_PREAMBLE, i));
  }

  tx_packet->state = TX_SENDING_SEQUENCE;
}

inline void tx_send_sequence(void)
{
  int i = 0;

  //Assign a sequence number
  tx_packet->sequence = tx_sequence_num++;

  for(i = 0;i < 16;i++){
    send_manchester_symbol(mask_bit((int) tx_packet->sequence, i));
  }

  tx_packet->state = TX_SENDING_PACKET_LEN;
}

inline void tx_send_len(void)
{
  int i = 0;
  for(i = 0;i < 32;i++){
    send_manchester_symbol(mask_bit((int) tx_packet->skb->len, i));
  }

  tx_packet->state = TX_SENDING_PACKET_DATA;
}

inline void tx_send_data(void)
{
  int byte = 0;
  int bit = 0;

  //Reset the tx checksum prior to calculating a new one
  tx_reset_adler();

  for(byte = 0;byte < tx_packet->skb->len;byte++){
    for(bit = 0;bit < 8;bit++){
      send_manchester_symbol(mask_bit(tx_packet->skb->data[byte], bit));
    }

    //Update the rolling checksum
    tx_update_adler(tx_packet->skb->data[byte]);
  }

  //Store the computed checksum in the packet
  tx_packet->checksum = tx_B;

  tx_packet->state = TX_SENDING_CHECKSUM;
}

inline void tx_send_checksum(void)
{
  int bit = 0;
  for(bit = 0;bit < 32;bit++){
    send_manchester_symbol(mask_bit(tx_packet->checksum, bit));
  }

  tx_packet->state = TX_WAITING_ACK;
}

inline void tx_wait_ack(void)
{
  int bit = 0;
  for(bit = 0;bit < 20000 && tx_packet->state == TX_WAITING_ACK;bit++){
    if(rx_packet->state == RX_SENDING_ACK) tx_send_ack();
    else send_manchester_symbol(1);
  }

  /* Retransmit the packet if we didn't get an ACK
   * TODO: Possible race condition between this comparison and the one
   * in the rx handler.
   */
  if(tx_packet->state == TX_WAITING_ACK){
    tx_packet->state = PACKET_READY;
    priv->stats.tx_errors++;
  }
}

void tx_handler(void *arg)
{
  while(tx_packet){ 

    //If the current packet isn't ready, step to the next one
    if(tx_packet->state == PACKET_COMPLETE ||
       tx_packet->state == PACKET_EMPTY ||
       tx_packet->state == PACKET_CORRUPTED){

      tx_packet = tx_packet->next;      
      if(rx_packet->state == RX_SENDING_ACK) tx_send_ack();
      else send_bit(1);
    }

    else if(tx_packet->state == TX_WAITING_ACK){
      if(rx_packet->state == RX_SENDING_ACK) tx_send_ack();
      else send_bit(1);
    }

    else if(tx_packet->state == PACKET_READY){
      if(rx_packet->state == RX_SENDING_ACK) tx_send_ack();

      tx_send_preamble();
      debug("VLC TX: Preamble sent!\n");
      if(tx_packet->state == TX_SENDING_SEQUENCE) tx_send_sequence();
      debug("VLC TX: Sequence sent!\n");
      if(tx_packet->state == TX_SENDING_PACKET_LEN) tx_send_len();
      debug("VLC TX: Length sent!\n");
      if(tx_packet->state == TX_SENDING_PACKET_DATA) tx_send_data();
      debug("VLC TX: Data sent!\n");
      if(tx_packet->state == TX_SENDING_CHECKSUM) tx_send_checksum();
      debug("VLC TX: Checksum sent!\n");

      if(tx_packet->state == TX_WAITING_ACK) tx_wait_ack();
    }
  }
  return;
}

inline void rx_get_preamble(void)
{
  rx_packet->current_preamble = 0;

  while(rx_packet->current_preamble != (int) VLC_PREAMBLE){
    rx_packet->current_preamble = (rx_packet->current_preamble >> 1) | 
      (get_bit() << 31); 

    if(rx_packet->current_preamble == (int) VLC_ACK){
      if(tx_packet->state == TX_WAITING_ACK) tx_packet->state = PACKET_COMPLETE;
    }
  }

  rx_packet->state = RX_GETTING_SEQUENCE;
}

inline void rx_get_sequence(void)
{
  int bit = 0;
  rx_packet->sequence = 0;

  for(bit = 0;bit < 16;bit++){
    rx_packet->sequence |= get_manchester_symbol() << bit;
  }

  rx_packet->state = RX_GETTING_PACKET_LEN;
}

inline void rx_get_len(void)
{  
  int bit = 0;
  rx_packet->rx_len = 0;

  for(bit = 0;bit < 32;bit++){
    rx_packet->rx_len |= get_manchester_symbol() << bit;
  }

  if(rx_packet->rx_len <= ETH_DATA_LEN) rx_packet->state = RX_GETTING_PACKET_DATA;
  else{
    debug("VLC RX: Incorrect packet length: %d!\n", rx_packet->rx_len);
    rx_packet->state = PACKET_READY;
  }
}

inline void rx_get_data(void)
{
  int byte = 0;
  int bit = 0;

  //Reset the checksum prior to computing a new one
  rx_reset_adler();

  for(byte = 0;byte < rx_packet->rx_len;byte++){
    for(bit = 0;bit < 8;bit++){
      rx_packet->rx_buff[byte] |= get_manchester_symbol() << bit;
    }
    rx_update_adler(rx_packet->rx_buff[byte]);
  }

  rx_packet->state = RX_GETTING_PACKET_CHECKSUM;
}

inline void rx_get_checksum(void)
{
  int bit = 0;
  rx_packet->checksum = 0;

  for(bit = 0;bit < 32;bit++){
    rx_packet->checksum |= get_manchester_symbol() << bit;
  }

  if(rx_packet->checksum == rx_B){
    rx_packet->state = RX_SENDING_ACK;
  }
  else{
    debug("VLC RX: Packet corrupted!\n");
    rx_packet->state = PACKET_CORRUPTED;
  }
}

inline void rx_send_ack(void)
{
  rx_packet->current_preamble = 0;
  while(rx_packet->state == RX_SENDING_ACK){
    rx_packet->current_preamble = (rx_packet->current_preamble >> 1) | (get_bit() << 31);

    if(rx_packet->current_preamble == (int) VLC_ACK){
      if(tx_packet->state == TX_WAITING_ACK) tx_packet->state = PACKET_COMPLETE;
    }
  }
}

void rx_handler(void *arg)
{
  while(rx_packet){

    //If the current packet isn't ready, step to the next one
    if(rx_packet->state != PACKET_READY){
      rx_packet = rx_packet->next;
      get_bit();
    }

    else{
      if(rx_packet->state == PACKET_READY) rx_get_preamble();
      debug("VLC RX: Got preamble!\n");
      if(rx_packet->state == RX_GETTING_SEQUENCE) rx_get_sequence();
      debug("VLC RX: Got sequence!\n");
      if(rx_packet->state == RX_GETTING_PACKET_LEN) rx_get_len();
      debug("VLC RX: Got length!\n");
      if(rx_packet->state == RX_GETTING_PACKET_DATA) rx_get_data();
      debug("VLC RX: Got data!\n");
      if(rx_packet->state == RX_GETTING_PACKET_CHECKSUM) rx_get_checksum();
      if(rx_packet->state == RX_SENDING_ACK) rx_send_ack();
      else priv->stats.rx_errors++;
    }
  }
  return;
}

int setup_timer_handler(void)
{
  int ret = 0;

  priv = netdev_priv(get_vlc_dev());

  //Get the first packets in the packet ring
  rx_packet = get_rx_packet();
  tx_packet = get_tx_packet();

  //Init semas
  rtdm_sem_init(&tx_sem, 1);
  rtdm_sem_init(&rx_sem, 1);

  //Store the current time
  rx_sleep_slot = rtdm_clock_read_monotonic();
  tx_sleep_slot = rx_sleep_slot + sleep_increment / 2;

  early_late_slot = do_div(sleep_increment, 5);

  //Return an error if either is NULL
  if(!rx_packet || !tx_packet) goto error;

  //Start the rx and tx handler tasks
  rtdm_task_init(&rx_handler_task, "VLC rx handler", rx_handler, 
                 NULL, RTDM_TASK_HIGHEST_PRIORITY, 0);
  if(ret) goto error;

  rtdm_task_init(&tx_handler_task, "VLC tx handler", tx_handler, 
                 NULL, RTDM_TASK_HIGHEST_PRIORITY, 0);
  if(ret) goto error;

  return 0;

 error:
  rtdm_task_destroy(&rx_handler_task);
  rtdm_task_destroy(&tx_handler_task);
  return -1;
}

void cleanup_timer_handler(void)
{
  rtdm_task_destroy(&tx_handler_task);
  rtdm_task_destroy(&rx_handler_task);

  rtdm_sem_destroy(&rx_sem);
  rtdm_sem_destroy(&tx_sem);
}



