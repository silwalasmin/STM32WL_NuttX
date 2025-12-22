/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_spi.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"

#include "stm32wl5.h"
#include "nucleo-wl55jc.h"

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_STM32WL5_SPI1
struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32WL5_SPI2S2
struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function stm32wl5_spidev_initialize(void)
{
#ifdef CONFIG_STM32WL5_SPI1
  g_spi1 = stm32wl5_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");
      return;
    }

#ifdef CONFIG_MTD_M25P
  stm32wl5_configgpio(GPIO_MT25Q_CS);
#endif

#ifdef CONFIG_LCD_SSD1680
  stm32wl5_configgpio(GPIO_SSD1680_CS);
  stm32wl5_configgpio(GPIO_SSD1680_CMD);
  stm32wl5_configgpio(GPIO_SSD1680_RST);
  stm32wl5_configgpio(GPIO_SSD1680_BUSY);
#endif

#endif

#ifdef CONFIG_STM32WL5_SPI2S2
  g_spi2 = stm32wl5_spibus_initialize(2);
#endif
}

#ifdef CONFIG_STM32WL5_SPI1
void stm32wl5_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");

#if defined(CONFIG_MTD_M25P)
  if (devid == SPIDEV_FLASH(0))
    {
      stm32wl5_gpiowrite(GPIO_MT25Q_CS, !selected);
    }
#endif

#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32wl5_gpiowrite(GPIO_SSD1680_CS, !selected);
    }
#endif

#if defined(CONFIG_CAN_MCP2515)
  if (devid == SPIDEV_CANBUS(0))
    {
      stm32wl5_gpiowrite(GPIO_MCP2515_CS, !selected);
    }
#endif

#ifdef HAVE_MMCSD
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32wl5_gpiowrite(GPIO_SPI_CS_SD_CARD, !selected);
    }
#endif
}

uint8_t stm32wl5_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
#if defined(CONFIG_MTD_M25P)
  if (devid == SPIDEV_FLASH(0))
    {
      return SPI_STATUS_PRESENT;
    }
#endif

#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      return SPI_STATUS_PRESENT;
    }
#endif

  return 0;
}

int stm32wl5_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32wl5_gpiowrite(GPIO_SSD1680_CMD, !cmd);
    }
#endif

  return OK;
}

#endif

#ifdef CONFIG_STM32WL5_SPI2S2
void stm32wl5_spi2s2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");
}

uint8_t stm32wl5_spi2s2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int stm32wl5_spi2s2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}

#endif

#endif /* CONFIG_STM32WL5_SPI1 || CONFIG_STM32WL5_SPI2S2 */