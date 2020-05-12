#include "head.h"

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Create common var*/
    common_event_groups = xEventGroupCreate();

    /* ESP_WIFI_MODE_AP */
    wifi_init_softap();
    /* ESP UDP SERV START */
    init_server((uint8_t) 5,(uint16_t) 1024*2);

}
