
#ifndef MAIN_HEAD_H_
#define MAIN_HEAD_H_
#include <string.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/i2c.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* General Group */
EventGroupHandle_t common_event_groups;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries*/
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define UDP_SEND_ACTIVE    BIT2
#define UDP_DATE_ACTIVE    BIT2

/* wifi drive conf */
#define ESP_WIFI_SSID      "test"
#define ESP_WIFI_PASS      "test_2123123"
#define MAX_STA_CONN       5
void wifi_init_softap();

/*       Wifi UDP Conf          */
#define PORT 80
#define MACADDR	{0x16,	0x34,	0x56,	0x78,	0x90,	0xab}
void init_server(uint8_t prior,uint16_t mem_us);


#endif /* MAIN_HEAD_H_ */
