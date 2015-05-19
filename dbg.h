/* Debug macro inspired by the amazing Zed Shaw.
 */

//#define NDEBUG

#ifndef __dbg_h
#define __dbg_h

#ifdef NDEBUG
#define debug(M, ...)
#else
#define debug(M, ...) printk(KERN_NOTICE M, ##__VA_ARGS__)
#endif

#endif
