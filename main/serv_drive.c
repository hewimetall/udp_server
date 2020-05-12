#include "head.h"
static QueueHandle_t x_queue_date;

static const char *TAG = "drive udp";
static struct str_date {
	char date_str[60];
	size_t len;
} date_r;

void server_send_q_date(char *p, size_t len) {

	bzero(date_r.date_str, len + 1);
	strcpy(date_r.date_str, p);
	date_r.date_str[len + 1] = '\0';
	date_r.len = len;

	ESP_LOGI(TAG, "DATE len %i: %s", date_r.len, date_r.date_str);

	xQueueSend(x_queue_date, (void*) &date_r, portMAX_DELAY);

}

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
		size_t len_sourceAddr) {
	struct str_date date_rx ;

	if (xQueueReceive(x_queue_date, &(date_rx), portMAX_DELAY) == pdPASS) {
		int lens = sendto(*sock, date_rx.date_str, date_rx.len, 0, sourceAddr,
				len_sourceAddr);

		if (lens < 0) {
			ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
			return ESP_FAIL;
		}
		xEventGroupSetBits(common_event_groups, /* The event group being updated. */
		UDP_DATE_ACTIVE); /* DATE set bit in event group*/

		return ESP_OK;
	}
	return ESP_FAIL;
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
				xEventGroupSetBits(common_event_groups, /* The event group being updated. */
				UDP_SEND_ACTIVE); /* Wifi connect set bit in event group*/

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
	x_queue_date = xQueueCreate(3, sizeof(date_r));

	xTaskCreate(udp_server_task, "udp_server", mem, NULL, prior, NULL);
	return ESP_OK;
}
