/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_appinit.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>

#ifdef CONFIG_MTD_M25P
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#endif

#include <stm32wl5.h>
#include <stm32wl5_uart.h>
#include <stm32wl5_pwr.h>

#include <arch/board/board.h>

#include "nucleo-wl55jc.h"

#ifndef CONFIG_NSH_PROC_MOUNTPOINT
#define CONFIG_NSH_PROC_MOUNTPOINT "/proc"
#endif

#ifdef CONFIG_MTD_M25P
extern struct spi_dev_s *g_spi1;
#endif

int board_app_initialize(uintptr_t arg)
{
  int ret = OK;
#ifdef CONFIG_MTD_M25P
  struct mtd_dev_s *mtd;
#endif

  (void)arg;

#ifdef HAVE_PROC
  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif

#if defined(CONFIG_USERLED_LOWER)
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS_LOWER)
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_FLASH_MOUNT)
  ret = stm32wl5_flash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_flash_init() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_IPCC)
  ret = ipcc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ipcc_init() failed\n");
    }
#endif

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
  stm32wl5_pwr_boot_c2();
#endif

#ifdef CONFIG_MTD_M25P
  /* Initialize SPI */

  syslog(LOG_INFO, "Initializing SPI for MT25Q flash\n");

  stm32wl5_spidev_initialize();

  if (!g_spi1)
    {
      syslog(LOG_ERR, "ERROR: SPI1 not initialized\n");
      return -ENODEV;
    }

  /* Initialize MT25Q flash */

  mtd = m25p_initialize(g_spi1);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: m25p_initialize() failed\n");
      return -ENODEV;
    }

  /* Register MTD device */

  ret = register_mtddriver("/dev/mtd0", mtd, 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "MT25Q registered at /dev/mtd0\n");

#ifdef CONFIG_FS_LITTLEFS
  /* Try to mount LittleFS */

  ret = nx_mount("/dev/mtd0", "/mnt/flash", "littlefs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Mount failed, formatting...\n");
      
      /* Format the flash */
      ret = nx_mount("/dev/mtd0", "/mnt/flash", "littlefs", 0, "autoformat");
      if (ret < 0)
        {
          syslog(LOG_ERR, "Format failed: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "LittleFS formatted and mounted\n");
        }
    }
  else
    {
      syslog(LOG_INFO, "LittleFS mounted at /mnt/flash\n");
    }
#endif

#endif /* CONFIG_MTD_M25P */

  return OK;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32wl5_get_uniqueid(uniqueid);
  return OK;
}
#endif