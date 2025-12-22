/****************************************************************************
 * uart_flash_main.c
 * Reads data from UART and writes 10 lines to flash
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define UART_DEV    "/dev/ttyS1"
#define FLASH_FILE  "/mnt/flash/log.txt"
#define BUFFER_SIZE 128
#define MAX_LINES   10

int uart_flash_main(int argc, char *argv[])
{
  int uart_fd;
  int flash_fd;
  char buffer[BUFFER_SIZE];
  int idx = 0;
  int line_count = 0;
  char c;

  uart_fd = open(UART_DEV, O_RDONLY);
  if (uart_fd < 0)
    {
      printf("Failed to open %s\n", UART_DEV);
      return -1;
    }

  flash_fd = open(FLASH_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (flash_fd < 0)
    {
      printf("Failed to open %s\n", FLASH_FILE);
      close(uart_fd);
      return -1;
    }

  printf("Reading from %s, writing to %s\n", UART_DEV, FLASH_FILE);

  while (line_count < MAX_LINES)
    {
      if (read(uart_fd, &c, 1) == 1)
        {
          buffer[idx++] = c;

          if (c == '\n' || idx >= BUFFER_SIZE - 1)
            {
              buffer[idx] = '\0';
              
              /* Write to flash */
              write(flash_fd, buffer, idx);
              fsync(flash_fd);
              
              /* Print to console */
              printf("[%d] %s", line_count + 1, buffer);
              
              line_count++;
              idx = 0;
            }
        }
    }

  printf("Done. Wrote %d lines to %s\n", line_count, FLASH_FILE);

  close(flash_fd);
  close(uart_fd);
  return 0;
}
