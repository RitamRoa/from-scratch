#include "csi_handler.h"
#include "dsp_processor.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include <string.h>

#define TAG "MAIN"

static httpd_handle_t server = NULL;

// ─── MULTI-CLIENT WEBSOCKET BROADCAST ───────────────────────────
#define MAX_WS_CLIENTS 8
static int ws_clients[MAX_WS_CLIENTS];
static int ws_client_count = 0;
static portMUX_TYPE ws_mutex = portMUX_INITIALIZER_UNLOCKED;

// ─── SMART CSI PACKET FILTERING ─────────────────────────────────
static const uint8_t LAPTOP_MAC[6] = {0xe0, 0xd5, 0x5d, 0x1a, 0x53, 0x6f};

bool csi_mac_filter_check(const uint8_t *incoming_mac) {
    // 1. If it's the laptop, drop it instantly.
    if (memcmp(incoming_mac, LAPTOP_MAC, 6) == 0) {
        return false; 
    }
    // 2. Let everything else through to feed the DSP math engine.
    return true; 
}

// Remove dead file descriptors from the registry
static void cleanup_dead_clients(void) {
  int dead[MAX_WS_CLIENTS];
  int dead_count = 0;
  int alive[MAX_WS_CLIENTS];
  int alive_count = 0;

  portENTER_CRITICAL(&ws_mutex);
  for (int i = 0; i < ws_client_count; i++) {
    if (httpd_ws_get_fd_info(server, ws_clients[i]) ==
        HTTPD_WS_CLIENT_WEBSOCKET) {
      alive[alive_count++] = ws_clients[i];
    } else {
      dead[dead_count++] = ws_clients[i];
    }
  }
  memcpy(ws_clients, alive, sizeof(int) * alive_count);
  ws_client_count = alive_count;
  portEXIT_CRITICAL(&ws_mutex);

  // Safe to log here outside critical section
  for (int i = 0; i < dead_count; i++) {
    ESP_LOGI(TAG, "WS client dropped: fd=%d", dead[i]);
  }
}

// Broadcast a JSON string to every registered WebSocket client.
// Called from dsp_task (Core 1) every 10 frames.
static void send_ws_data(const char *json_data) {
  if (!server || ws_client_count == 0)
    return;

  // Snapshot the client list under lock so we hold the lock minimally
  int count;
  int clients_copy[MAX_WS_CLIENTS];
  portENTER_CRITICAL(&ws_mutex);
  count = ws_client_count;
  memcpy(clients_copy, ws_clients, sizeof(int) * count);
  portEXIT_CRITICAL(&ws_mutex);

  httpd_ws_frame_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.payload = (uint8_t *)json_data;
  pkt.len = strlen(json_data);
  pkt.type = HTTPD_WS_TYPE_TEXT;

  bool any_dead = false;
  for (int i = 0; i < count; i++) {
    esp_err_t ret = httpd_ws_send_frame_async(server, clients_copy[i], &pkt);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Send failed fd=%d (ret=%d), will prune", clients_copy[i],
               ret);
      any_dead = true;
    }
  }
  if (any_dead)
    cleanup_dead_clients();
}

// ─── DSP TASK ────────────────────────────────────────────────────
// Runs on Core 1 — reads ring buffer, processes signal
void dsp_task(void *pvParameters) {
  dsp_processor_init();
  csi_frame_t frame;
  uint32_t frame_count = 0;

  while (1) {
    if (csi_handler_read(&frame)) {
      dsp_processor_push(&frame);
      frame_count++;

      if (frame_count % 10 == 0) {
        csi_output_t out = dsp_processor_get_output();

        // Global network guard: bypass DSP mathematical inference if no
        // operator dashboard connected
        if (ws_client_count == 0) {
          out.presence = PRESENCE_EMPTY;
          out.person_count = 0;
          out.motion_score = 0.00f;
          out.br_bpm = 0;
          out.br_bpm_2 = 0;
        }

        const char *p[] = {"EMPTY", "SINGLE", "MULTI"};
        printf("PRESENCE:%s PEOPLE:%d MOTION:%.2f BR:%d bpm\n", p[out.presence],
               out.person_count, out.motion_score, out.br_bpm);

        // Average BR for multi-occupancy
        int br_avg = 0;
        if (out.br_bpm > 0 && out.br_bpm_2 > 0) {
          br_avg = (out.br_bpm + out.br_bpm_2) / 2;
        } else if (out.br_bpm > 0) {
          br_avg = out.br_bpm;
        }

        char json[2048];
        int offset = snprintf(
            json, sizeof(json),
            "{\"presence\":\"%s\",\"person_count\":%d,\"motion_score\":%.2f,"
            "\"br_bpm\":%d,\"br_bpm_2\":%d,\"br_avg\":%d,"
            "\"rssi\":%d,\"clients\":%d,\"subcarriers\":[",
            p[out.presence], out.person_count, out.motion_score, out.br_bpm,
            out.br_bpm_2, br_avg, frame.rssi, ws_client_count);

        for (int i = 0; i < 64; i++) {
          offset += snprintf(json + offset, sizeof(json) - offset, "%.2f%s",
                             frame.amp[i], (i == 63) ? "]}" : ",");
        }

        send_ws_data(json);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ─── EMBEDDED HTML & ROUTING ───────────────────────────────────
extern const char heatmap3d_html_start[] asm("_binary_heatmap3d_html_start");
extern const char heatmap3d_html_end[] asm("_binary_heatmap3d_html_end");

static esp_err_t root_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, "<html><body style=\"background:#000;\"></body></html>",
                  -1);
  return ESP_OK;
}

static esp_err_t monitor_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  // Send in chunks to handle large file
  const size_t chunk = 4096;
  size_t remaining = (size_t)(heatmap3d_html_end - heatmap3d_html_start);
  const char *ptr = heatmap3d_html_start;
  while (remaining > 0) {
    size_t send_len = remaining < chunk ? remaining : chunk;
    if (httpd_resp_send_chunk(req, ptr, send_len) != ESP_OK)
      break;
    ptr += send_len;
    remaining -= send_len;
  }
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL};

static const httpd_uri_t monitor_uri = {.uri = "/monitor",
                                        .method = HTTP_GET,
                                        .handler = monitor_handler,
                                        .user_ctx = NULL};

// ─── HTTPD WEBSOCKET SERVER ──────────────────────────────────────
static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    // WebSocket handshake — register the new client fd
    int fd = httpd_req_to_sockfd(req);
    bool accepted = false;
    int total = 0;

    portENTER_CRITICAL(&ws_mutex);
    if (ws_client_count < MAX_WS_CLIENTS) {
      ws_clients[ws_client_count++] = fd;
      accepted = true;
      total = ws_client_count;
    }
    portEXIT_CRITICAL(&ws_mutex);

    if (accepted) {
      ESP_LOGI(TAG, "WS client connected: fd=%d  total=%d", fd, total);
    } else {
      ESP_LOGW(TAG, "WS client list full (max %d), rejecting fd=%d",
               MAX_WS_CLIENTS, fd);
    }
    return ESP_OK;
  }

  // Receive and discard any incoming frames from clients
  uint8_t buf[128] = {0};
  httpd_ws_frame_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.payload = buf;
  pkt.len = sizeof(buf);
  httpd_ws_recv_frame(req, &pkt, sizeof(buf));
  return ESP_OK;
}

static const httpd_uri_t ws = {.uri = "/ws",
                               .method = HTTP_GET,
                               .handler = ws_handler,
                               .user_ctx = NULL,
                               .is_websocket = true};

static httpd_handle_t start_webserver(void) {
  httpd_handle_t s = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.stack_size = 8192;
  config.send_wait_timeout = 10;
  config.recv_wait_timeout = 10;
  config.max_uri_handlers = 8;
  config.max_open_sockets = 12;

  memset(ws_clients, -1, sizeof(ws_clients));
  ws_client_count = 0;

  if (httpd_start(&s, &config) == ESP_OK) {
    httpd_register_uri_handler(s, &root_uri);
    httpd_register_uri_handler(s, &monitor_uri);
    httpd_register_uri_handler(s, &ws);
    ESP_LOGI(TAG, "HTTP server ready — open http://192.168.4.1/monitor");
    return s;
  }
  return NULL;
}

// ─── WIFI ────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "Station connected, AID=%d, MAC=" MACSTR, event->aid,
             MAC2STR(event->mac));
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "Station disconnected, AID=%d, MAC=" MACSTR, event->aid,
             MAC2STR(event->mac));
  }
}

void wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));

  wifi_config_t ap_config = {
      .ap =
          {
              .ssid = "SPARS-DEMO-NODE",
              .ssid_len = 0,
              .password = "",
              .channel = 6,
              .authmode = WIFI_AUTH_OPEN,
              .max_connection = 4,
              .beacon_interval = 100,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  ESP_LOGI(TAG, "SoftAP started: SSID=SPARS-DEMO-NODE  gateway=192.168.4.1");
}

// ─── MAIN ────────────────────────────────────────────────────────
void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  wifi_init();
  csi_handler_init();

  server = start_webserver();

  xTaskCreatePinnedToCore(dsp_task, "dsp_task", 8192, NULL, 4, NULL, 1);

  ESP_LOGI(TAG, "SPARS ready — connect to SSID SPARS-DEMO-NODE then open "
                "http://192.168.4.1/monitor");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
