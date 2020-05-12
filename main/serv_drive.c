#include "head.h"
#define START_MESSENG "start"
static const char *TAG = "drive udp";

static int socket_init(char *addr_str, struct sockaddr_in *destAddr) {
	destAddr->sin_addr.s_addr = htonl(INADDR_ANY);
	destAddr->sin_family = AF_INET;
	destAddr->sin_port = htons(PORT);
	inet_ntoa_r(destAddr->sin_addr, addr_str, sizeof(addr_str) - 1);
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		return -1;
	}
	ESP_LOGI(TAG, "Socket created");
	return sock;
}

static esp_err_t socket_send_date(int *sock, struct sockaddr_in *sourceAddr,
		size_t len) {

	char messenge[] = "hellow";

	int lens = sendto(*sock, messenge, sizeof(messenge) - 1, 0, sourceAddr,
			len);

	if (lens < 0) {
		ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
		return ESP_FAIL;
	}
	return ESP_OK;
}

static void udp_server_task(void *pvParameters) {
	char rx_buffer[128];
	char addr_str[128];
	struct sockaddr_in destAddr;
	int sock;

	xEventGroupWaitBits(common_event_groups, /* The event group being tested. */
	WIFI_CONNECTED_BIT, /* The bits within the event group to wait for. */
	pdFALSE, /* BIT_0 & BIT_4 not should be cleared before returning. */
	pdFALSE, /* Don't wait for both bits, either bit will do. */
	portMAX_DELAY);/* Wait inf time. */
	while (1) {
		sock = socket_init(&addr_str, &destAddr);
		if (sock < 0)
			break;

		int err = bind(sock, (struct sockaddr*) &destAddr, sizeof(destAddr));
		if (err < 0) {
			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		}
		ESP_LOGI(TAG, "Socket binded");
		while (1) {
			ESP_LOGI(TAG, "Waiting for data");
			struct sockaddr_in sourceAddr;
			socklen_t socklen = sizeof(sourceAddr);
			int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
					(struct sockaddr*) &sourceAddr, &socklen);

			if (len < 0) {
				ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
				break;
			}
			// Data received
			else if (rx_buffer[0] == 't') {
				for (;;) {
					int err = socket_send_date(&sock, &sourceAddr,
							sizeof(sourceAddr));
					if (err == ESP_FAIL) {
						break;
					}
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}
			}
		}
		if (sock != -1) {
			ESP_LOGE(TAG, "Shutting down socket and restarting...");
			shutdown(sock, 0);
			close(sock);
		}
	}
	vTaskDelete(NULL);
}

esp_err_t udp_server_start(uint8_t prior, uint16_t mem) {
	xTaskCreate(udp_server_task, "udp_server", mem, NULL, prior, NULL);
	return ESP_OK;
}
