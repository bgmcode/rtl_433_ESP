/** @file
    This is a generated file from tools/update_rtl_433_devices.sh
*/

#ifndef INCLUDE_RTL_433_DEVICES_H_
#define INCLUDE_RTL_433_DEVICES_H_

#include "r_device.h"

#ifndef MY_DEVICES
#  define DEVICES                    \
    DECL(reolink_doorbell)           \
    /* Add new decoders here. */
#  define NUMOF_OOK_DEVICES 1
#  define NUMOF_FSK_DEVICES 0
/* Add new decoders here. */
#else
/**
 * Subset of devices that I have access to and have tested with
 */
#  define DEVICES         \
    DECL(lacrosse_tx141x) \
/* Add new personal decoders here. */
#  define NUMOF_OOK_DEVICES 1
#  define NUMOF_FSK_DEVICES 0
#endif

#define DECL(name) extern r_device name;
DEVICES
#undef DECL

#ifdef RTL_FLEX
extern r_device* flex_create_device(char* spec); // maybe put this in some header file?
#  define NUMOFDEVICES 214
#endif

#endif /* INCLUDE_RTL_433_DEVICES_H_ */
