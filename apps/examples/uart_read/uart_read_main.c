/****************************************************************************
 * uart_read_main.c
 * Reads UART data and publishes to uORB
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/mcu_mag.h>

#define UART_DEV    "/dev/ttyS1"
#define BUFFER_SIZE 128

int uart_read_main(int argc, char *argv[])
{
  int uart_fd;
  int pub;
  char buffer[BUFFER_SIZE];
  int idx = 0;
  char c;
  struct mcu_mag_s data;

  uart_fd = open(UART_DEV, O_RDONLY);
  if (uart_fd < 0)
    {
      printf("Failed to open %s\n", UART_DEV);
      return -1;
    }

  /* Initialize and advertise uORB topic */

  memset(&data, 0, sizeof(data));
  pub = orb_advertise(ORB_ID(mcu0_mag), &data);
  if (pub < 0)
    {
      printf("Failed to advertise mcu0_mag\n");
      close(uart_fd);
      return -1;
    }

  printf("Reading from %s, publishing to mcu0_mag\n", UART_DEV);

  while (1)
    {
      if (read(uart_fd, &c, 1) == 1)
        {
          buffer[idx++] = c;

          if (c == '\n' || idx >= BUFFER_SIZE - 1)
            {
              buffer[idx] = '\0';

              /* Find X=, Y=, Z= in buffer */

              char *xptr = strstr(buffer, "X=");
              char *yptr = strstr(buffer, "Y=");
              char *zptr = strstr(buffer, "Z=");

              if (xptr && yptr && zptr)
                {
                  data.x = strtof(xptr + 2, NULL);
                  data.y = strtof(yptr + 2, NULL);
                  data.z = strtof(zptr + 2, NULL);
                  data.timestamp = orb_absolute_time();

                  orb_publish(ORB_ID(mcu0_mag), pub, &data);
                  printf("Published: X=%.2f Y=%.2f Z=%.2f\n",
                         data.x, data.y, data.z);
                }

              idx = 0;
            }
        }
    }

  close(uart_fd);
  return 0;
}