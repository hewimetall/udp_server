#include "head.h"

void app_main()
{
	char t[]="hellow word";
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Create common var*/
    common_event_groups = xEventGroupCreate();

    /* ESP_WIFI_MODE_AP */
    wifi_init_softap();
    /* ESP UDP SERV START */
    udp_server_start((uint8_t) 5,(uint16_t) 1024*2);
//    server_send_q_date(&t,sizeof( t)-1);
}
