/****************************************************************************
 * examples/nodelink/nodelink_main.c
 *
 * openvela inter-node periodic status reporter (TCP client).
 * Connects to server and sends JSON node status every INTERVAL ms.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_IP    CONFIG_EXAMPLES_NODELINK_SERVER_IP
#define SERVER_PORT  CONFIG_EXAMPLES_NODELINK_SERVER_PORT
#define INTERVAL_MS  CONFIG_EXAMPLES_NODELINK_INTERVAL_MS
#define NODE_ID      "esp32s3-openvela"

#ifdef CONFIG_EXAMPLES_NODELINK_UDP
#  define SOCK_TYPE  SOCK_DGRAM
#else
#  define SOCK_TYPE  SOCK_STREAM
#endif

static struct sockaddr_in g_server_addr;

static int open_socket(void)
{
  int fd;

  fd = socket(AF_INET, SOCK_TYPE, 0);
  if (fd < 0)
    {
      perror("socket");
      return -1;
    }

  memset(&g_server_addr, 0, sizeof(g_server_addr));
  g_server_addr.sin_family      = AF_INET;
  g_server_addr.sin_port        = htons(SERVER_PORT);
  g_server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

#ifndef CONFIG_EXAMPLES_NODELINK_UDP
  if (connect(fd, (struct sockaddr *)&g_server_addr,
              sizeof(g_server_addr)) < 0)
    {
      perror("connect");
      close(fd);
      return -1;
    }
#endif

  return fd;
}

int main(int argc, FAR char *argv[])
{
  char buf[256];
  int  fd = -1;
  int  seq = 0;
  int  ret;

  printf("[nodelink] Starting: %s -> %s:%d every %d ms (%s)\n",
         NODE_ID, SERVER_IP, SERVER_PORT, INTERVAL_MS,
#ifdef CONFIG_EXAMPLES_NODELINK_UDP
         "UDP"
#else
         "TCP"
#endif
         );

  while (1)
    {
      if (fd < 0)
        {
          printf("[nodelink] Opening socket to %s:%d ...\n",
                 SERVER_IP, SERVER_PORT);
          fd = open_socket();
          if (fd < 0)
            {
              printf("[nodelink] Socket failed, retry in 3s\n");
              usleep(3000 * 1000);
              continue;
            }

          printf("[nodelink] Ready!\n");
        }

      snprintf(buf, sizeof(buf),
               "{\"node\":\"%s\",\"seq\":%d,\"uptime_ms\":%lu}\n",
               NODE_ID, seq++,
               (unsigned long)(clock() * 1000 / CLOCKS_PER_SEC));

#ifdef CONFIG_EXAMPLES_NODELINK_UDP
      ret = sendto(fd, buf, strlen(buf), 0,
                   (struct sockaddr *)&g_server_addr,
                   sizeof(g_server_addr));
#else
      ret = write(fd, buf, strlen(buf));
#endif
      if (ret < 0)
        {
          perror("write");
          close(fd);
          fd = -1;
          continue;
        }

      printf("[nodelink] sent: %s", buf);
      usleep(INTERVAL_MS * 1000);
    }

  close(fd);
  return 0;
}
