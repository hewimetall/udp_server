#include "head.h"

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* ESP_WIFI_MODE_AP */
    wifi_init_softap();

}
