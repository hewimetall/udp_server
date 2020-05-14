#include "head.h"

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
//	mpu_init();
//    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);

    /* Create common var*/
    common_event_groups = xEventGroupCreate();

    /* ESP_WIFI_MODE_AP */
    wifi_init_softap();
    /* ESP UDP SERV START */
    udp_server_start((uint8_t) 5,(uint16_t) 1024*2);
    mpu_start((uint8_t) 5,(uint16_t) 1024*2);

}
