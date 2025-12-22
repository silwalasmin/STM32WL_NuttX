/****************************************************************************
 * mcu_mag.c
 * uORB topic definition for magnetometer data
 ****************************************************************************/

#include <nuttx/config.h>
#include <uORB/uORB.h>
#include <uORB/mcu_mag.h>

ORB_DEFINE(mcu0_mag, struct mcu_mag_s, NULL);
