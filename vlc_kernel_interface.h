#ifndef _vlc_kernel_interface_h
#define _vlc_kernel_interface_h

#include <linux/netdevice.h>

#define VLC_MAC_ADDR "\0SNUL0"

//The frequency of the physical layer.
#define FREQ 25000

//Some private data for the driver. Not currently used (I think).
struct vlc_priv {
  struct net_device_stats stats; /* Device stats */
  struct vlc_packet *ppool;
  struct vlc_packet *rx_queue;  /* List of incoming packets */
  int rx_int_enabled;
  struct net_device *dev;
};

struct net_device *get_vlc_dev(void);

#endif
