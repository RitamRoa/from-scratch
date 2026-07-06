#include "csi_handler.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define TAG "CSI-HANDLER"

// ─── RING BUFFER ─────────────────────────────────────────────────
// Written by Core 0 (WiFi), read by Core 1 (DSP)
// NEVER let AI rewrite this — too critical
static csi_frame_t ring_buf[CSI_BUF_LEN];
static volatile int  rb_write = 0;
static volatile int  rb_read  = 0;

static inline int rb_next(int idx) {
    return (idx + 1) % CSI_BUF_LEN;
}

static inline int rb_empty(void) {
    return rb_read == rb_write;
}

// Returns 1 if successfully wrote, 0 if full (drop frame)
static int rb_push(const csi_frame_t *frame) {
    int next = rb_next(rb_write);
    if (next == rb_read) return 0; // full, drop
    ring_buf[rb_write] = *frame;
    rb_write = next;
    return 1;
}

// Returns 1 if successfully read, 0 if empty
static int rb_pop(csi_frame_t *out) {
    if (rb_empty()) return 0;
    *out = ring_buf[rb_read];
    rb_read = rb_next(rb_read);
    return 1;
}

// ─── CSI CALLBACK ────────────────────────────────────────────────
// CORE 0 — copy data immediately, never store pointer
static void csi_callback(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    int8_t *buf = info->buf;
    int num_subs = info->len / 2;
    if (num_subs > NUM_SUBCARRIERS) num_subs = NUM_SUBCARRIERS;

    // Build frame — copy amplitude values immediately
    csi_frame_t frame;
    for (int i = 0; i < NUM_SUBCARRIERS; i++) {
        if (i < num_subs) {
            float I = (float)buf[i * 2];
            float Q = (float)buf[i * 2 + 1];
            frame.amp[i] = sqrtf(I*I + Q*Q); // true amplitude
        } else {
            frame.amp[i] = 0.0f;
        }
    }
    
    // Smoothed RSSI — 10-sample EMA removes frame-to-frame jitter
    // α=0.1 → TC ~10 frames, prevents distance ring from jumping
    static float rssi_smooth = -60.0f;
    rssi_smooth = rssi_smooth * 0.9f + (float)info->rx_ctrl.rssi * 0.1f;
    frame.rssi = (int8_t)rssi_smooth;

    rb_push(&frame);
}

// ─── PUBLIC API ──────────────────────────────────────────────────
void csi_handler_init(void)
{
    wifi_csi_config_t cfg = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI handler ready");
}

int csi_handler_read(csi_frame_t *out)
{
    return rb_pop(out);
}