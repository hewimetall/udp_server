
#ifndef MAIN_HEAD_H_
#define MAIN_HEAD_H_


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
/* wifi drive conf */
#define ESP_WIFI_SSID      "test"
#define ESP_WIFI_PASS      "test_2123123"
#define MAX_STA_CONN       5
void wifi_init_softap();


#endif /* MAIN_HEAD_H_ */
