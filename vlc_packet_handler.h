#ifndef _vlc_packet_handler_h
#define _vlc_packet_handler_h

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/types.h>

#include "vlc_kernel_interface.h"

//General packet states
#define PACKET_EMPTY 0
#define PACKET_READY 1
#define PACKET_CORRUPTED 2
#define PACKET_COMPLETE 3

//States for RX packets
#define RX_WAITING_FOR_PREAMBLE 4
#define RX_GETTING_SEQUENCE 5
#define RX_GETTING_PACKET_LEN 6
#define RX_GETTING_PACKET_DATA 7
#define RX_GETTING_PACKET_CHECKSUM 8
#define RX_SENDING_ACK 9

//States for TX packets
#define TX_SENDING_PREAMBLE 9
#define TX_SENDING_SEQUENCE 10
#define TX_SENDING_PACKET_LEN 11
#define TX_SENDING_PACKET_DATA 12
#define TX_SENDING_CHECKSUM 13
#define TX_WAITING_ACK 14

//A VLC packet
struct vlc_packet{
  u8 state;                    /* The current status of the packet. */
  u16 sequence;                /* The sequence number of the packet. */
  u16 bit_index;                 
  u16 byte_index;
  u32 current_preamble;        /* The currently collected preamble. */
  struct vlc_packet *next;     /* The next packet in the packet pool. */
  struct net_device *dev;      /* The device owning the packet. */
  struct sk_buff *skb;         /* The buffer used to store TX data. */
  u16 rx_len;                  /* The buffer for storing the RX packet
                                  length. */
  u8 rx_buff[ETH_DATA_LEN];    /* The buffer used to store RX data. */
  u32 checksum;                /* The Adler-32 checksum of the packet. */
};

void packet_handler_sem_up(void);
void packet_handler_sem_destroy(void);
struct vlc_packet *get_tx_packet(void);
struct vlc_packet *get_rx_packet(void);
int place_skb(struct sk_buff *);
struct sk_buff *get_skb(void);
void update_pools(void *);
int teardown_pool(void);
int setup_packet_handler(int );

#endif
