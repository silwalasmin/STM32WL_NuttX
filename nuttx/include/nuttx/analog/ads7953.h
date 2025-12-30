/****************************************************************************
 * include/nuttx/analog/ads7953.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7953_H
#define __INCLUDE_NUTTX_ANALOG_ADS7953_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADS7953 Device Information
 * - 12-bit resolution
 * - 16 single-ended channels
 * - 1 MSPS sampling rate
 * - SPI interface (Mode 0, CPOL=0, CPHA=0)
 */

#define ADS7953_RESOLUTION      12
#define ADS7953_MAX_CHANNELS    16

/* IOCTL Commands */

#define ANIOC_ADS7953_SET_REF   _ANIOC(0x0080)  /* Set reference voltage */
#define ANIOC_ADS7953_SET_RANGE _ANIOC(0x0081)  /* Set input range */
#define ANIOC_ADS7953_READ_CH   _ANIOC(0x0082)  /* Read specific channel */

/* Reference voltage range selection */

#define ADS7953_RANGE_VREF      0  /* Input range: 0 to Vref (2.5V) */
#define ADS7953_RANGE_2VREF     1  /* Input range: 0 to 2*Vref (5V) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADS7953 configuration structure */

struct ads7953_config_s
{
  uint32_t spi_freq;        /* SPI clock frequency (max 20 MHz) */
  uint8_t  spi_mode;        /* SPI mode (should be 0) */
  uint16_t vref_mv;         /* Reference voltage in millivolts */
  uint8_t  range;           /* Input range selection */
  uint16_t channel_mask;    /* Bitmask of enabled channels (0-15) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ads7953_initialize
 *
 * Description:
 *   Initialize the ADS7953 ADC device.
 *
 * Input Parameters:
 *   spi    - An SPI driver instance
 *   config - Configuration structure (NULL for defaults)
 *
 * Returned Value:
 *   A pointer to the ADC lower-half driver instance on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct adc_dev_s *ads7953_initialize(FAR struct spi_dev_s *spi,
                                         FAR const struct ads7953_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_ADS7953_H */
