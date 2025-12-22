#ifndef __UORB_MCU_MAG_H
#define __UORB_MCU_MAG_H

#include <stdint.h>

struct mcu_mag_s
{
  uint64_t timestamp;
  float x;
  float y;
  float z;
};

ORB_DECLARE(mcu0_mag);

#endif