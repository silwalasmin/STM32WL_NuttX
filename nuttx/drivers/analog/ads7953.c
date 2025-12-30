/****************************************************************************
 * drivers/analog/ads7953.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads7953.h>

#if defined(CONFIG_ADC_ADS7953)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADS7953 SPI Frame Format (16-bit)
 *
 * The ADS79xx family uses a pipelined SPI protocol:
 * - Frame N: Send command for channel X, receive data from channel (N-2)
 * - Result has 2-frame latency from command
 *
 * 16-bit Command Frame (MSB first):
 * Bit 15-12: Mode select
 * Bit 11:    Program enable (write settings)
 * Bit 10-7:  Channel address (0-15)
 * Bit 6:     Vref range (0=2.5V, 1=5V)
 * Bit 5:     Power down
 * Bit 4:     GPIO data output
 * Bit 3-0:   GPIO data
 *
 * 16-bit Output Frame (MSB first):
 * Bit 15-12: Channel address of this sample
 * Bit 11-0:  12-bit ADC data
 */

/* Mode Selection (bits 15-12) */

#define ADS7953_MODE_MANUAL     (0x1 << 12)  /* Manual channel select */
#define ADS7953_MODE_AUTO1      (0x2 << 12)  /* Auto-1 mode */
#define ADS7953_MODE_AUTO2      (0x3 << 12)  /* Auto-2 mode */
#define ADS7953_MODE_CONT       (0x0 << 12)  /* Continue previous mode */

/* Command bits */

#define ADS7953_PROG_EN         (1 << 11)    /* Program enable */
#define ADS7953_CHAN_SHIFT      7            /* Channel address shift */
#define ADS7953_CHAN_MASK       (0x0F << 7)  /* Channel mask */
#define ADS7953_CHAN(n)         (((n) & 0x0F) << 7)

/* Settings bits (active when PROG_EN is set) */

#define ADS7953_RANGE_5V        (1 << 6)     /* 0-5V range (2x Vref) */
#define ADS7953_RANGE_2V5       (0 << 6)     /* 0-2.5V range (1x Vref) */
#define ADS7953_POWERDOWN       (1 << 5)     /* Power down mode */
#define ADS7953_GPIO_OUT        (1 << 4)     /* GPIO data read in next frame */

/* Output frame parsing */

#define ADS7953_OUT_CHAN_SHIFT  12
#define ADS7953_OUT_CHAN_MASK   (0x0F << 12)
#define ADS7953_OUT_DATA_MASK   0x0FFF

/* Extract channel from output */

#define ADS7953_GET_CHAN(x)     (((x) >> ADS7953_OUT_CHAN_SHIFT) & 0x0F)
#define ADS7953_GET_DATA(x)     ((x) & ADS7953_OUT_DATA_MASK)

/* Default configuration */

#define ADS7953_SPI_FREQ_DEFAULT    10000000  /* 10 MHz default */
#define ADS7953_VREF_MV_DEFAULT     2500      /* 2.5V reference */

/* SPI Configuration */

#define ADS7953_SPI_MODE        SPIDEV_MODE0  /* CPOL=0, CPHA=0 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7953_dev_s
{
  FAR struct adc_dev_s      *adc;         /* ADC device reference */
  FAR struct spi_dev_s      *spi;         /* SPI device reference */
  FAR const struct adc_callback_s *cb;    /* Upper-half callback */

  uint32_t                  spi_freq;     /* SPI frequency */
  uint16_t                  vref_mv;      /* Reference voltage in mV */
  uint16_t                  channel_mask; /* Enabled channels bitmask */
  uint8_t                   current_ch;   /* Current channel being read */
  uint8_t                   range;        /* Voltage range setting */
  bool                      enabled;      /* Conversion enabled flag */

  /* Cached command word */

  uint16_t                  cmd_settings; /* Settings bits (range, gpio, etc) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI helpers */

static void     ads7953_lock(FAR struct spi_dev_s *spi);
static void     ads7953_unlock(FAR struct spi_dev_s *spi);
static uint16_t ads7953_xfer(FAR struct ads7953_dev_s *priv, uint16_t cmd);
static int      ads7953_read_channel(FAR struct ads7953_dev_s *priv,
                                     uint8_t channel, uint16_t *data);

/* ADC lower-half operations */

static int      ads7953_bind(FAR struct adc_dev_s *dev,
                             FAR const struct adc_callback_s *callback);
static void     ads7953_reset(FAR struct adc_dev_s *dev);
static int      ads7953_setup(FAR struct adc_dev_s *dev);
static void     ads7953_shutdown(FAR struct adc_dev_s *dev);
static void     ads7953_rxint(FAR struct adc_dev_s *dev, bool enable);
static int      ads7953_ioctl(FAR struct adc_dev_s *dev, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC lower-half operations */

static const struct adc_ops_s g_ads7953_ops =
{
  .ao_bind     = ads7953_bind,
  .ao_reset    = ads7953_reset,
  .ao_setup    = ads7953_setup,
  .ao_shutdown = ads7953_shutdown,
  .ao_rxint    = ads7953_rxint,
  .ao_ioctl    = ads7953_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7953_lock
 *
 * Description:
 *   Lock and configure the SPI bus for ADS7953 communication.
 *
 ****************************************************************************/

static void ads7953_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ADS7953_SPI_MODE);
  SPI_SETBITS(spi, 16);
}

/****************************************************************************
 * Name: ads7953_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void ads7953_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: ads7953_xfer
 *
 * Description:
 *   Perform a single 16-bit SPI transfer.
 *
 * Input Parameters:
 *   priv - Device private data
 *   cmd  - 16-bit command word to send
 *
 * Returned Value:
 *   16-bit response from device
 *
 ****************************************************************************/

static uint16_t ads7953_xfer(FAR struct ads7953_dev_s *priv, uint16_t cmd)
{
  uint16_t response;

  SPI_SETFREQUENCY(priv->spi, priv->spi_freq);

  /* Select the device */

  SPI_SELECT(priv->spi, SPIDEV_ADC(0), true);

  /* Exchange 16-bit word */

  response = (uint16_t)SPI_SEND(priv->spi, cmd);

  /* Deselect the device */

  SPI_SELECT(priv->spi, SPIDEV_ADC(0), false);

  return response;
}

/****************************************************************************
 * Name: ads7953_read_channel
 *
 * Description:
 *   Read a single channel from the ADS7953.
 *
 *   Due to the pipelined nature of the ADS7953, we need to send 3 frames:
 *   Frame 1: Send command to select channel -> get garbage
 *   Frame 2: Send continue command -> get garbage (conversion in progress)
 *   Frame 3: Send continue command -> get actual data for our channel
 *
 * Input Parameters:
 *   priv    - Device private data
 *   channel - Channel number (0-15)
 *   data    - Pointer to store the 12-bit result
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int ads7953_read_channel(FAR struct ads7953_dev_s *priv,
                                uint8_t channel, uint16_t *data)
{
  uint16_t cmd;
  uint16_t response;
  uint8_t  resp_channel;

  if (channel >= ADS7953_MAX_CHANNELS)
    {
      return -EINVAL;
    }

  ads7953_lock(priv->spi);

  /* Build command: Manual mode + Program enable + Channel + Settings */

  cmd = ADS7953_MODE_MANUAL | ADS7953_PROG_EN |
        ADS7953_CHAN(channel) | priv->cmd_settings;

  /* Frame 1: Send channel select command */

  ads7953_xfer(priv, cmd);

  /* Frame 2: Continue mode (keep settings, start conversion) */

  cmd = ADS7953_MODE_CONT | ADS7953_CHAN(channel) | priv->cmd_settings;
  ads7953_xfer(priv, cmd);

  /* Frame 3: Read result from our channel */

  response = ads7953_xfer(priv, cmd);

  ads7953_unlock(priv->spi);

  /* Parse response */

  resp_channel = ADS7953_GET_CHAN(response);
  *data = ADS7953_GET_DATA(response);

  /* Verify we got data from the expected channel */

  if (resp_channel != channel)
    {
      ainfo("Channel mismatch: expected %d, got %d\n", channel, resp_channel);

      /* Not necessarily an error - could be pipeline delay.
       * Return the data anyway and let caller decide.
       */
    }

  ainfo("CH%d: raw=0x%04x data=%u\n", channel, response, *data);

  return OK;
}

/****************************************************************************
 * Name: ads7953_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half driver.
 *
 ****************************************************************************/

static int ads7953_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: ads7953_reset
 *
 * Description:
 *   Reset the ADS7953 device.
 *
 ****************************************************************************/

static void ads7953_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  uint16_t cmd;

  DEBUGASSERT(priv != NULL);

  ainfo("Resetting ADS7953\n");

  ads7953_lock(priv->spi);

  /* Send manual mode command with program enable to reset state */

  cmd = ADS7953_MODE_MANUAL | ADS7953_PROG_EN | priv->cmd_settings;
  ads7953_xfer(priv, cmd);

  /* Send a few dummy frames to flush the pipeline */

  ads7953_xfer(priv, ADS7953_MODE_CONT);
  ads7953_xfer(priv, ADS7953_MODE_CONT);

  ads7953_unlock(priv->spi);

  priv->current_ch = 0;
  priv->enabled = false;
}

/****************************************************************************
 * Name: ads7953_setup
 *
 * Description:
 *   Configure and initialize the ADS7953 device.
 *
 ****************************************************************************/

static int ads7953_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  ainfo("Setting up ADS7953: freq=%lu, vref=%u mV, range=%s\n",
        (unsigned long)priv->spi_freq, priv->vref_mv,
        priv->range ? "5V" : "2.5V");

  /* Build settings command based on configuration */

  priv->cmd_settings = 0;

  if (priv->range == ADS7953_RANGE_2VREF)
    {
      priv->cmd_settings |= ADS7953_RANGE_5V;
    }

  /* Perform a reset to initialize the device */

  ads7953_reset(dev);

  return OK;
}

/****************************************************************************
 * Name: ads7953_shutdown
 *
 * Description:
 *   Disable the ADS7953 and put it in power-down mode.
 *
 ****************************************************************************/

static void ads7953_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  uint16_t cmd;

  DEBUGASSERT(priv != NULL);

  ainfo("Shutting down ADS7953\n");

  priv->enabled = false;

  ads7953_lock(priv->spi);

  /* Send power-down command */

  cmd = ADS7953_MODE_MANUAL | ADS7953_PROG_EN | ADS7953_POWERDOWN;
  ads7953_xfer(priv, cmd);

  ads7953_unlock(priv->spi);
}

/****************************************************************************
 * Name: ads7953_rxint
 *
 * Description:
 *   Enable or disable RX interrupts (triggers ADC conversion).
 *
 *   Since ADS7953 doesn't have a hardware interrupt, this function
 *   triggers a software-based conversion when enabled.
 *
 ****************************************************************************/

static void ads7953_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  uint16_t data;
  int32_t  value;
  uint8_t  ch;
  int      ret;

  DEBUGASSERT(priv != NULL);

  ainfo("rxint: enable=%d\n", enable);

  priv->enabled = enable;

  if (enable && priv->cb != NULL)
    {
      /* Scan through all enabled channels and report data */

      for (ch = 0; ch < ADS7953_MAX_CHANNELS; ch++)
        {
          /* Check if this channel is enabled */

          if ((priv->channel_mask & (1 << ch)) == 0)
            {
              continue;
            }

          ret = ads7953_read_channel(priv, ch, &data);
          if (ret < 0)
            {
              aerr("Failed to read channel %d: %d\n", ch, ret);
              continue;
            }

          /* Convert to 32-bit value as expected by ADC framework */

          value = (int32_t)data;

          /* Report the sample to upper-half driver */

          priv->cb->au_receive(dev, ch, value);
        }
    }
}

/****************************************************************************
 * Name: ads7953_ioctl
 *
 * Description:
 *   Handle device-specific IOCTL commands.
 *
 ****************************************************************************/

static int ads7953_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
      case ANIOC_ADS7953_SET_REF:
        {
          /* Set reference voltage in millivolts */

          priv->vref_mv = (uint16_t)arg;
          ainfo("Set Vref to %u mV\n", priv->vref_mv);
        }
        break;

      case ANIOC_ADS7953_SET_RANGE:
        {
          /* Set input range (0 = 2.5V, 1 = 5V) */

          priv->range = (uint8_t)arg;

          if (priv->range == ADS7953_RANGE_2VREF)
            {
              priv->cmd_settings |= ADS7953_RANGE_5V;
            }
          else
            {
              priv->cmd_settings &= ~ADS7953_RANGE_5V;
            }

          ainfo("Set range to %s\n", priv->range ? "5V" : "2.5V");
        }
        break;

      case ANIOC_ADS7953_READ_CH:
        {
          /* Read a specific channel (arg = channel number) */

          uint16_t data;
          uint8_t channel = (uint8_t)(arg & 0xFF);

          ret = ads7953_read_channel(priv, channel, &data);
          if (ret >= 0)
            {
              ret = (int)data;
            }
        }
        break;

      case ANIOC_TRIGGER:
        {
          /* Trigger a conversion on all enabled channels */

          ads7953_rxint(dev, true);
        }
        break;

      default:
        {
          aerr("Unrecognized IOCTL command: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7953_initialize
 *
 * Description:
 *   Initialize the ADS7953 ADC driver.
 *
 * Input Parameters:
 *   spi    - An SPI driver instance
 *   config - Configuration structure (NULL for defaults)
 *
 * Returned Value:
 *   A pointer to the ADC device structure on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct adc_dev_s *ads7953_initialize(FAR struct spi_dev_s *spi,
                                         FAR const struct ads7953_config_s *config)
{
  FAR struct ads7953_dev_s *priv;
  FAR struct adc_dev_s     *adc;

  DEBUGASSERT(spi != NULL);

  ainfo("Initializing ADS7953\n");

  /* Allocate the driver state structure */

  priv = (FAR struct ads7953_dev_s *)kmm_zalloc(sizeof(struct ads7953_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate driver structure\n");
      return NULL;
    }

  /* Allocate the ADC device structure */

  adc = (FAR struct adc_dev_s *)kmm_zalloc(sizeof(struct adc_dev_s));
  if (adc == NULL)
    {
      aerr("ERROR: Failed to allocate ADC device structure\n");
      kmm_free(priv);
      return NULL;
    }

  /* Initialize the private structure */

  priv->spi = spi;
  priv->adc = adc;

  /* Apply configuration or use defaults */

  if (config != NULL)
    {
      priv->spi_freq     = config->spi_freq;
      priv->vref_mv      = config->vref_mv;
      priv->range        = config->range;
      priv->channel_mask = config->channel_mask;
    }
  else
    {
      /* Use default configuration */

      priv->spi_freq     = ADS7953_SPI_FREQ_DEFAULT;
      priv->vref_mv      = ADS7953_VREF_MV_DEFAULT;
      priv->range        = ADS7953_RANGE_VREF;
      priv->channel_mask = 0xFFFF;  /* All 16 channels enabled */
    }

  priv->current_ch = 0;
  priv->enabled = false;
  priv->cmd_settings = 0;

  /* Initialize the ADC device structure */

  adc->ad_ops  = &g_ads7953_ops;
  adc->ad_priv = priv;

  return adc;
}

#endif /* CONFIG_ADC_ADS7953 */
