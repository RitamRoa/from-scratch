#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "csi_handler.h"
#include "dsp_processor.h"

#define WIFI_SSID        "raowaifi"
#define WIFI_PASSWORD    "netbeka123"
#define TAG              "MAIN"
#define PING_INTERVAL_MS 10

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ─── DSP TASK ────────────────────────────────────────────────────
// Runs on Core 1 — reads ring buffer, processes signal
void dsp_task(void *pvParameters)
{
    dsp_processor_init();
    csi_frame_t frame;
    float       cleaned[TOP_K_SUBCARRIERS];
    int         top_k[TOP_K_SUBCARRIERS];
    uint32_t    frame_count = 0;

    while (1) {
        if (csi_handler_read(&frame)) {
            dsp_processor_push(&frame);
            frame_count++;

            // Only run heavy DSP every 50 frames
            if (frame_count % 10 == 0) {
                dsp_processor_get_signal(cleaned, top_k);
                printf("CLEAN[%lu] subs:%d,%d,%d vals:%.1f %.1f %.1f\n",
                    (unsigned long)frame_count,
                    top_k[0], top_k[1], top_k[2],
                    cleaned[0], cleaned[1], cleaned[2]);
            }
        }
        // Always yield — non negotiable
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── SELF-PING TASK ──────────────────────────────────────────────
void ping_task(void *pvParameters)
{
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(3333);
    dest.sin_addr.s_addr = inet_addr("10.71.92.46"); // gateway

    uint8_t buf[32] = {0xAB};
    ESP_LOGI(TAG, "Ping task running");

    while (1) {
        sendto(sock, buf, sizeof(buf), 0,
               (struct sockaddr *)&dest, sizeof(dest));
        vTaskDelay(pdMS_TO_TICKS(PING_INTERVAL_MS));
    }
}

// ─── WIFI ────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to %s...", WIFI_SSID);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

// ─── MAIN ────────────────────────────────────────────────────────
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    csi_handler_init();

    xTaskCreatePinnedToCore(ping_task, "ping_task", 4096,
                            NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(dsp_task, "dsp_task", 8192,
                            NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "All tasks running");
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
if (frame_count % 10 == 0) {
    dsp_processor_get_signal(cleaned, top_k);
    presence_state_t presence = dsp_get_presence(cleaned);

    const char *presence_str[] = {"EMPTY", "SINGLE", "MULTI"};
    printf("PRESENCE:%s vals:%.1f %.1f %.1f\n",
        presence_str[presence],
        cleaned[0], cleaned[1], cleaned[2]);
}