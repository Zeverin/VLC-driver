#ifndef _vlc_timer_handler_h
#define _vlc_timer_handler_h

#include <rtdm/rtdm_driver.h>
#include "vlc_packet_handler.h"

//The preamble that preceeds every packet
//#define VLC_PREAMBLE 2863311530 /* 10101010101010101010101010101010 in base 10 */

//#define VLC_PREAMBLE 861230387 /* 001100110101010101010101010100110011 */

//The sequence sent prior to every packet
#define VLC_PREAMBLE 858993466

//The sequence sent to acknowledge that a packet has been received
#define VLC_ACK 2858850645  /* 10101010011001101001100101010101 */

//Preamble has equal number of ones and zeros but isn't a square wave.
//#define VLC_PREAMBLE 2858850645  /* 10101010011001101001100101010101


//GPIO addresses
#define GPIO_INPUT 50
#define GPIO_OUTPUT 66//60

int setup_timer_handler(void);
void cleanup_timer_handler(void);
//void send_manchester_bit(rtdm_timer_t *);
//void timer_handler(void *);


#endif
