/****************************************************************************
 * flash_write_main.c
 * Subscribes to uORB topic and writes to flash
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/mcu_mag.h>

#define FLASH_FILE  "/mnt/flash/maglog.txt"
#define MAX_LINES   10

int flash_write_main(int argc, char *argv[])
{
  int sub;
  int flash_fd;
  int line_count = 0;
  char buffer[128];
  struct mcu_mag_s data;
  struct pollfd pfd;

  /* Subscribe to uORB topic */

  sub = orb_subscribe(ORB_ID(mcu0_mag));
  if (sub < 0)
    {
      printf("Failed to subscribe to mcu0_mag\n");
      return -1;
    }

  /* Open flash file */

  flash_fd = open(FLASH_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (flash_fd < 0)
    {
      printf("Failed to open %s\n", FLASH_FILE);
      orb_unsubscribe(sub);
      return -1;
    }

  printf("Writing to %s (max %d lines)\n", FLASH_FILE, MAX_LINES);

  pfd.fd = sub;
  pfd.events = POLLIN;

  while (line_count < MAX_LINES)
    {
      if (poll(&pfd, 1, -1) > 0)
        {
          orb_copy(ORB_ID(mcu0_mag), sub, &data);

          /* Format data as string */

          int len = snprintf(buffer, sizeof(buffer),
                             "X=%.2f Y=%.2f Z=%.2f T=%llu\n",
                             data.x, data.y, data.z, data.timestamp);

          /* Write to flash */

          write(flash_fd, buffer, len);
          fsync(flash_fd);

          /* Print to console */

          printf("[%d] %s", line_count + 1, buffer);

          line_count++;
        }
    }

  printf("Done. Wrote %d lines to %s\n", line_count, FLASH_FILE);

  close(flash_fd);
  orb_unsubscribe(sub);
  return 0;
}