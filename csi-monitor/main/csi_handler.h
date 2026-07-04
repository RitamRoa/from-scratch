#pragma once
#include "esp_wifi.h"

#define CSI_BUF_LEN        128   // ring buffer size (frames)
#define NUM_SUBCARRIERS    64    // subcarriers per frame
#define TOP_K_SUBCARRIERS  10    // top subcarriers by variance

// One CSI frame — amplitude only, copied out of callback immediately
typedef struct {
    float amp[NUM_SUBCARRIERS];
    int8_t rssi;
} csi_frame_t;

void csi_handler_init(void);
// Returns 1 if a frame was available, 0 if buffer empty
int  csi_handler_read(csi_frame_t *out);