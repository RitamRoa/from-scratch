#include <string.h>
#include <math.h>
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

#define WIFI_SSID      "raowaifi"
#define WIFI_PASSWORD  "netbeka123"
#define TAG            "CSI-MONITOR"
#define PING_PORT      3333
#define PING_INTERVAL_MS 10   // 10ms = ~100Hz

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ─── CSI CALLBACK ───────────────────────────────────────────────
// RUNS ON CORE 0 — fast only, no heavy work
static uint32_t csi_frame_count = 0;

void csi_callback(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    int8_t *buf = info->buf;
    int num_subcarriers = info->len / 2;
    csi_frame_count++;

    // Print every 10th frame to avoid flooding serial
    if (csi_frame_count % 10 != 0) return;

    printf("CSI[%d][%lu subs]: ", (int)csi_frame_count, (unsigned long)num_subcarriers);
    for (int i = 0; i < 10 && i < num_subcarriers; i++) {
        int8_t I = buf[i * 2];
        int8_t Q = buf[i * 2 + 1];
        int amplitude = abs(I) + abs(Q);
        printf("%3d ", amplitude);
    }
    printf("\n");
}

// ─── CSI INIT ───────────────────────────────────────────────────
void csi_init(void)
{
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI enabled");
}

// ─── SELF-PING TASK ─────────────────────────────────────────────
// Runs on Core 1 — sends UDP packets to itself every 10ms
// This forces consistent CSI frame generation ~100Hz
void ping_task(void *pvParameters)
{
    // Wait for WiFi to be ready
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(500)); // small settle delay

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create ping socket");
        vTaskDelete(NULL);
        return;
    }

    // Send to broadcast on hotspot subnet
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(PING_PORT);
    dest.sin_addr.s_addr = inet_addr("10.71.92.255"); // hotspot broadcast

    uint8_t ping_buf[32] = {0xAB}; // dummy payload

    ESP_LOGI(TAG, "Self-ping task started at ~100Hz");

    while (1) {
        sendto(sock, ping_buf, sizeof(ping_buf), 0,
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
    csi_init();

    // Pin ping task to Core 1, DSP will also run on Core 1 later
    xTaskCreatePinnedToCore(ping_task, "ping_task", 4096,
                            NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "CSI pipeline running...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}