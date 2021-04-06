#ifndef _UDP_NET_H_
#define _UDP_NET_H_

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "tcpip_adapter.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <lwip/dns.h>

#define NTP_PACKET_SIZE 48

typedef enum {
    WAIT_SET=1,
    WIFI_SCAN,
    WIFI_LINK,
    TIME_GET,
    TIME_OK
}NETWORKSTATUS;

extern NETWORKSTATUS netWorkStatus;

extern char myssid[36];
extern char mypass[36];
extern char ssidReady;

extern uint16_t ap_count;
extern wifi_ap_record_t ap_info[20];
extern char ntpSuccess;

extern int my_hour;
extern int my_minute;
extern int my_second;

void network_task(void *pvParameters);



#endif