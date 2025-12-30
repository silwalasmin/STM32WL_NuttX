#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>
#include <sched.h>

#include <uORB/uORB.h>
#include <uORB/mcu_mag.h>

#define UART_DEV "/dev/ttyS1"
#define FLASH_FILE "/mnt/flash/maglog.txt"
#define MAX_LINES 10 // lines to save in flash
#define BUFFER_SIZE 128

static void *uart_producer_thread(void *arg)
{
  int uart_fd;
  int pub;
  int idx = 0;
  char buffer[BUFFER_SIZE];
  char c;
  struct mcu_mag_s data;

  uart_fd = open(UART_DEV, O_RDONLY);
  if (uart_fd < 0)
    {
      printf("Failed to open %s\n", UART_DEV);
      return NULL;
    }

  memset(&data, 0, sizeof(data));
  pub = orb_advertise(ORB_ID(mcu0_mag), &data);
  if (pub < 0)
    {
      printf("Failed to advertise mcu0_mag\n");
      close(uart_fd);
      return NULL;
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
  return NULL;
}

static void *flash_consumer_thread(void *arg)
{
  int sub;
  int flash_fd;
  int line_count = 0;
  char buffer[BUFFER_SIZE];
  struct mcu_mag_s data;
  struct pollfd pfd;

  sub = orb_subscribe(ORB_ID(mcu0_mag));
  if (sub < 0)
    {
      printf("Failed to subscribe to mcu0_mag\n");
      return NULL;
    }

  flash_fd = open(FLASH_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (flash_fd < 0)
    {
      printf("Failed to open %s\n", FLASH_FILE);
      orb_unsubscribe(sub);
      return NULL;
    }

  printf("Writing to %s (max %d lines)\n", FLASH_FILE, MAX_LINES);

  pfd.fd = sub;
  pfd.events = POLLIN;

  while (line_count < MAX_LINES)
    {
      if (poll(&pfd, 1, 5000) > 0)
        {
          orb_copy(ORB_ID(mcu0_mag), sub, &data);

          int len = snprintf(buffer, sizeof(buffer),
                             "X=%.2f Y=%.2f Z=%.2f T=%llu\n",
                             data.x, data.y, data.z,
                             (unsigned long long)data.timestamp);

          write(flash_fd, buffer, len);
          fsync(flash_fd);

          printf("[%d] %s", line_count + 1, buffer);

          line_count++;
        }
    }

  printf("Done. Wrote %d lines to %s\n", line_count, FLASH_FILE);

  close(flash_fd);
  orb_unsubscribe(sub);
  return NULL;
}

int data_logger_main(int argc, char *argv[])
{
  pthread_t tid1, tid2;
  pthread_attr_t attr;
  struct sched_param param;

  pthread_attr_init(&attr);

  /* reading from uart (higher priority) */

  param.sched_priority = 150;
  pthread_attr_setschedparam(&attr, &param);
  pthread_create(&tid1, &attr, uart_producer_thread, NULL);

  /* saving to file (lower priority) */

  param.sched_priority = 100;
  pthread_attr_setschedparam(&attr, &param);
  pthread_create(&tid2, &attr, flash_consumer_thread, NULL);

  /* Wait for threads to finish */

  pthread_join(tid1, NULL);
  pthread_join(tid2, NULL);

  return 0;
}