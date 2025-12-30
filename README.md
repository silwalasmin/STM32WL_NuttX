# Data Logger - NuttX uORB Application

## Overview

Multi-threaded data logging system for STM32WL55 that reads magnetometer data from UART, publishes to uORB topic, and writes to SPI flash using LittleFS.

---

## Data Flow

```
EPDM Board ──UART──► uart_producer ──uORB──► flash_consumer ──SPI──► MT25Q Flash
                     (priority 150)         (priority 100)
```

---

## Folder Structure

```
apps/
├── examples/
│   ├── data_logger/
│   │   ├── Kconfig
│   │   ├── Make.defs
│   │   ├── Makefile
│   │   ├── data_logger_main.c
│   │   ├── mcu_mag.c
│   │   
│   │
│   ├── uart_read/
│   │   ├── Kconfig
│   │   ├── Make.defs
│   │   ├── Makefile
│   │   ├── uart_read_main.c
│   │   ├── mcu_mag.c
│   │   
│   │
│   └── flash_write/
│       ├── Kconfig
│       ├── Make.defs
│       ├── Makefile
│       └── flash_write_main.c
│__ system/
|   |
|   |__uorb/
|   |__uORB/mcu_mag.h
|
nuttx/
├── boards/arm/stm32wl5/nucleo-wl55jc/
│   ├── include/
│   │   └── board.h              ← SPI pin definitions
│   └── src/
│       ├── nucleo-wl55jc.h      ← CS pin definition
│       ├── stm32_appinit.c      ← MTD + LittleFS init
│       ├── stm32_spi.c          ← SPI driver + CS control
│       └── Makefile
```

---

## uORB Topic

### Location

`apps/examples/data_logger/` (or `apps/examples/uart_read/`)

### mcu_mag.h

```c
#ifndef __MCU_MAG_H
#define __MCU_MAG_H

#include <stdint.h>
#include <uORB/uORB.h>

struct mcu_mag_s
{
  uint64_t timestamp;
  float x;
  float y;
  float z;
};

ORB_DECLARE(mcu0_mag);

#endif
```

### mcu_mag.c

```c
#include <uORB/uORB.h>
#include "mcu_mag.h"

ORB_DEFINE(mcu0_mag, struct mcu_mag_s, NULL);
```

---

## Hardware Configuration

### SPI1 Pins (board.h)

| Signal | Pin | Macro |
|--------|-----|-------|
| MISO | PA6 | `GPIO_SPI1_MISO_2` |
| MOSI | PA7 | `GPIO_SPI1_MOSI_2` |
| SCK | PA5 | `GPIO_SPI1_SCK_2` |

### Flash CS Pin (nucleo-wl55jc.h)

| Signal | Pin | Macro |
|--------|-----|-------|
| CS | PA4 | `GPIO_MT25Q_CS` |

```c
#define GPIO_MT25Q_CS (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
```

### UART1 Pins (board.h)

| Signal | Pin |
|--------|-----|
| TX | PB6 |
| RX | PB7 |

---

## NuttX Configuration

### Required Configs (.config)

```makefile
# Core
CONFIG_BOARDCTL=y
CONFIG_NSH_ARCHINIT=y
CONFIG_BUILTIN=y
CONFIG_NSH_BUILTIN_APPS=y

# SPI and Flash
CONFIG_STM32WL5_SPI1=y
CONFIG_MTD=y
CONFIG_MTD_M25P=y

# Filesystem
CONFIG_FS_LITTLEFS=y

# UART
CONFIG_STM32WL5_USART1=y

# uORB
CONFIG_UORB=y

# Pthreads
CONFIG_PTHREAD_MUTEX_TYPES=y

# Apps
CONFIG_EXAMPLES_DATA_LOGGER=y
CONFIG_EXAMPLES_UART_READ=y
CONFIG_EXAMPLES_FLASH_WRITE=y
```

### Enable in menuconfig

```
System Type --->
  STM32WL5 Peripheral Support --->
    [*] SPI1
    [*] USART1

Device Drivers --->
  Memory Technology Device (MTD) Support --->
    [*] SPI-based M25P/MT25Q FLASH

File Systems --->
  [*] LittleFS file system

Application Configuration --->
  [*] Support Builtin Applications
  NSH Library --->
    [*] Enable built-in applications
    [*] NSH archinit
  System Libraries and NSH Add-Ons --->
    [*] uORB
  Examples --->
    [*] Data Logger with uORB
    [*] UART Read (optional)
    [*] Flash Write (optional)

RTOS Features --->
  [*] pthreads
```


---

## Board Initialization

### stm32_appinit.c (relevant section)

```c
#include <nuttx/config.h>
#include <sys/mount.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_MTD_M25P
extern struct spi_dev_s *g_spi1;
#endif

int board_app_initialize(uintptr_t arg)
{
  int ret = OK;

#ifdef CONFIG_MTD_M25P
  struct mtd_dev_s *mtd;

  /* Initialize SPI */
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
  /* Mount LittleFS with autoformat */
  ret = nx_mount("/dev/mtd0", "/mnt/flash", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR, "LittleFS mount failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "LittleFS mounted at /mnt/flash\n");
    }
#endif

#endif /* CONFIG_MTD_M25P */

  return OK;
}
```

### stm32_spi.c (relevant section)

```c
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include "stm32wl5.h"
#include "nucleo-wl55jc.h"

#ifdef CONFIG_STM32WL5_SPI1
struct spi_dev_s *g_spi1;
#endif

void stm32wl5_spidev_initialize(void)
{
#ifdef CONFIG_STM32WL5_SPI1
  g_spi1 = stm32wl5_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: Failed to initialize SPI1\n");
      return;
    }

#ifdef CONFIG_MTD_M25P
  stm32wl5_configgpio(GPIO_MT25Q_CS);
#endif

#endif
}

#ifdef CONFIG_STM32WL5_SPI1
void stm32wl5_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
#if defined(CONFIG_MTD_M25P)
  if (devid == SPIDEV_FLASH(0))
    {
      stm32wl5_gpiowrite(GPIO_MT25Q_CS, !selected);
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
  return 0;
}
#endif
```

---

## Build and Flash

```bash
# Configure
cd nuttx
./tools/configure.sh nucleo-wl55jc:nsh

# Enable configs
make menuconfig

# Build
make -j$(nproc)

# Flash
st-flash write nuttx.bin 0x8000000
```

---

## Usage

### Run combined data_logger

```
nsh> data_logger
Reading from /dev/ttyS1, publishing to mcu0_mag
Writing to /mnt/flash/maglog.txt (max 10 lines)
Published: X=30751.11 Y=-22271.66 Z=22324.00
[1] X=30751.11 Y=-22271.66 Z=22324.00 T=123456789
...
Done. Wrote 10 lines to /mnt/flash/maglog.txt
```

### Run separate apps

```
nsh> uart_read &
nsh> flash_write &
```

### Verify flash content

```
nsh> cat /mnt/flash/maglog.txt
```

### Check mounted filesystems

```
nsh> mount
nsh> ls /mnt/flash
```

---

## Data Format

### Input (from EPDM board via UART)

```
Mag: X=30751.11 Y=-22271.66 Z=22324.00 |B|=44045.64 mG | Msgs:6524
```

### Output (to flash file)

```
X=30751.11 Y=-22271.66 Z=22324.00 T=123456789
```

---

## Storage Stack

```
┌─────────────────────────────┐
│   Application (open/write)  │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│   POSIX VFS                 │
│   /mnt/flash/maglog.txt     │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│   LittleFS                  │
│   (wear leveling)           │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│   MTD Driver                │
│   /dev/mtd0                 │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│   SPI Driver (M25P)         │
│   PA4=CS PA5=SCK PA6=MISO   │
│   PA7=MOSI                  │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│   MT25Q SPI Flash           │
└─────────────────────────────┘
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `command not found` | Enable `CONFIG_NSH_BUILTIN_APPS=y` and `CONFIG_BUILTIN=y` |
| `undefined reference to g_orb_mcu0_mag` | Add `mcu_mag.c` to Makefile CSRCS |
| `Failed to open /dev/ttyS1` | Enable `CONFIG_STM32WL5_USART1=y` |
| `LittleFS mount failed` | First mount uses `autoformat` option |
| `No /dev/mtd0` | Check `stm32_appinit.c` MTD initialization |
| `Parse failed` | Check UART data format matches `strstr()` parsing |
| `poll() stuck` | Use timeout: `poll(&pfd, 1, 5000)` |
| Build error: missing priority | Use hardcoded values in Makefile |

---

## Files Description

| File | Purpose |
|------|---------|
| `data_logger_main.c` | Combined app with two threads |
| `uart_read_main.c` | Standalone UART reader + uORB publisher |
| `flash_write_main.c` | Standalone uORB subscriber + flash writer |
| `mcu_mag.h` | uORB topic structure declaration |
| `mcu_mag.c` | uORB topic definition |
| `stm32_appinit.c` | Board init: SPI, MTD, LittleFS mount |
| `stm32_spi.c` | SPI bus init, CS pin control |
| `nucleo-wl55jc.h` | Board-specific pin definitions |
| `board.h` | SPI and UART pin mappings |

---

## License

Apache 2.0 (NuttX)