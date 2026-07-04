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
#include "esp_http_server.h"
#include "csi_handler.h"
#include "dsp_processor.h"


#define WIFI_SSID        "raowaifi"
#define WIFI_PASSWORD    "netbeka123"
#define TAG              "MAIN"
#define PING_INTERVAL_MS 10

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static httpd_handle_t server = NULL;

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    char *json;
};

static void ws_async_send(void *arg)
{
    struct async_resp_arg *resp_arg = arg;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)resp_arg->json;
    ws_pkt.len = strlen(resp_arg->json);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(resp_arg->hd, resp_arg->fd, &ws_pkt);
    free(resp_arg->json);
    free(resp_arg);
}

static void send_ws_data(const char *json_data) {
    if (!server) return;
    
    size_t fds = 16;
    int client_fds[16];
    if (httpd_get_client_list(server, &fds, client_fds) == ESP_OK) {
        for (int i = 0; i < fds; i++) {
            struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
            if (resp_arg) {
                resp_arg->hd = server;
                resp_arg->fd = client_fds[i];
                resp_arg->json = strdup(json_data);
                
                if (httpd_queue_work(server, ws_async_send, resp_arg) != ESP_OK) {
                    free(resp_arg->json);
                    free(resp_arg);
                }
            }
        }
    }
}

// ─── DSP TASK ────────────────────────────────────────────────────
// Runs on Core 1 — reads ring buffer, processes signal
void dsp_task(void *pvParameters)
{
    dsp_processor_init();
    csi_frame_t  frame;
    uint32_t     frame_count = 0;

    while (1) {
        if (csi_handler_read(&frame)) {
            dsp_processor_push(&frame);
            frame_count++;

            if (frame_count % 10 == 0) {
                csi_output_t out = dsp_processor_get_output();
                const char *p[] = {"EMPTY", "SINGLE", "MULTI"};
                printf("PRESENCE:%s PEOPLE:%d MOTION:%.2f BR:%d bpm\n",
                    p[out.presence],
                    out.person_count,
                    out.motion_score,
                    out.br_bpm);
                    
                char json[2048];
                int offset = snprintf(json, sizeof(json), "{\"presence\":\"%s\",\"person_count\":%d,\"motion_score\":%.2f,\"br_bpm\":%d,\"rssi\":%d,\"subcarriers\":[",
                    p[out.presence], out.person_count, out.motion_score, out.br_bpm, frame.rssi);
                
                for (int i = 0; i < 64; i++) {
                    offset += snprintf(json + offset, sizeof(json) - offset, "%.2f%s", frame.amp[i], (i == 63) ? "]}" : ",");
                }
                
                send_ws_data(json);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── EMBEDDED HTML ──────────────────────────────────────────────
extern const char monitor_html_start[] asm("_binary_monitor_html_start");
extern const char monitor_html_end[]   asm("_binary_monitor_html_end");

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    // Send in 4 KB chunks — avoids stack overflow on large files
    const char *ptr = monitor_html_start;
    size_t remaining = (size_t)(monitor_html_end - monitor_html_start);
    const size_t CHUNK = 4096;

    while (remaining > 0) {
        size_t to_send = (remaining > CHUNK) ? CHUNK : remaining;
        if (httpd_resp_send_chunk(req, ptr, (ssize_t)to_send) != ESP_OK) {
            httpd_resp_send_chunk(req, NULL, 0); // abort
            return ESP_FAIL;
        }
        ptr       += to_send;
        remaining -= to_send;
    }
    // Signal end of chunked response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = root_handler,
    .user_ctx = NULL
};

// ─── HTTPD WEBSOCKET SERVER ──────────────────────────────────────
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    return ESP_OK;
}

static const httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t s = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Larger stack for the httpd worker — needed when sending big HTML files
    config.stack_size         = 8192;
    config.send_wait_timeout  = 10;  // seconds
    config.recv_wait_timeout  = 10;  // seconds
    config.max_uri_handlers   = 8;

    if (httpd_start(&s, &config) == ESP_OK) {
        httpd_register_uri_handler(s, &root_uri);
        httpd_register_uri_handler(s, &ws);
        ESP_LOGI(TAG, "HTTP server started — open http://<ESP32-IP>/ in your browser");
        return s;
    }
    return NULL;
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
    
    // Dynamically get the gateway IP to ensure we are pinging the actual AP
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info;
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        dest.sin_addr.s_addr = ip_info.gw.addr;
    } else {
        dest.sin_addr.s_addr = inet_addr("10.71.92.46"); // Fallback
    }

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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // DISABLE POWER SAVE FOR REAL-TIME CSI

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

    server = start_webserver();

    xTaskCreatePinnedToCore(ping_task, "ping_task", 4096,
                            NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(dsp_task, "dsp_task", 8192,
                            NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "All tasks running");
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
