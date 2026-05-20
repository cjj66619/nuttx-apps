/****************************************************************************
 * examples/wificonn/wificonn_main.c
 *
 * Usage: wificonn <ssid> <password>
 *   Connects wlan0 to a WPA2-PSK AP and obtains an IP via DHCP.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <wireless/wapi.h>
#include <netutils/netlib.h>

#define IFNAME    "wlan0"
#define TIMEOUT_S 5

int main(int argc, FAR char *argv[])
{
  int           sock;
  int           ret;
  int           wait;
  struct in_addr addr;

  if (argc < 3)
    {
      fprintf(stderr, "Usage: wificonn <ssid> <password>\n");
      fprintf(stderr, "  e.g. wificonn 312 125110011000\n");
      return EXIT_FAILURE;
    }

  const char *ssid = argv[1];
  const char *psk  = argv[2];

  printf("[wificonn] SSID=%-16s  connecting...\n", ssid);

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
    {
      perror("socket");
      return EXIT_FAILURE;
    }

  /* Set WPA version = WPA2 */

  ret = wpa_driver_wext_set_auth_param(sock, IFNAME,
                                       IW_AUTH_WPA_VERSION,
                                       IW_AUTH_WPA_VERSION_WPA2);
  if (ret < 0)
    {
      fprintf(stderr, "[wificonn] set WPA version failed: %d\n", ret);
      close(sock);
      return EXIT_FAILURE;
    }

  /* Set cipher = CCMP */

  ret = wpa_driver_wext_set_auth_param(sock, IFNAME,
                                       IW_AUTH_CIPHER_PAIRWISE,
                                       IW_AUTH_CIPHER_CCMP);
  if (ret < 0)
    {
      fprintf(stderr, "[wificonn] set cipher failed: %d\n", ret);
      close(sock);
      return EXIT_FAILURE;
    }

  /* Set PSK */

  ret = wpa_driver_wext_set_key_ext(sock, IFNAME,
                                    WPA_ALG_CCMP,
                                    psk, strlen(psk));
  if (ret < 0)
    {
      fprintf(stderr, "[wificonn] set PSK failed: %d\n", ret);
      close(sock);
      return EXIT_FAILURE;
    }

  /* Set SSID (triggers connection) */

  ret = wapi_set_essid(sock, IFNAME, ssid, WAPI_ESSID_ON);
  if (ret < 0)
    {
      fprintf(stderr, "[wificonn] set ESSID failed: %d\n", ret);
      close(sock);
      return EXIT_FAILURE;
    }

  close(sock);

  /* DHCP */

  printf("[wificonn] Requesting IP via DHCP");
  fflush(stdout);

  ret = netlib_obtain_ipv4addr(IFNAME);
  if (ret < 0)
    {
      printf("\n[wificonn] DHCP failed: %d\n", ret);
      return EXIT_FAILURE;
    }

  /* Print result */

  printf("[wificonn] Waiting for IP");
  fflush(stdout);

  for (wait = 0; wait < TIMEOUT_S; wait++)
    {
      sleep(1);
      if (netlib_get_ipv4addr(IFNAME, &addr) == 0 &&
          addr.s_addr != 0)
        {
          printf("\n[wificonn] OK! IP=%s\n", inet_ntoa(addr));
          return EXIT_SUCCESS;
        }

      printf(".");
      fflush(stdout);
    }

  printf("\n[wificonn] Timeout\n");
  return EXIT_FAILURE;
}
