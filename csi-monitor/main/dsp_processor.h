#pragma once
#include "csi_handler.h"

// ─── WELFORD STATE ───────────────────────────────────────────────
typedef struct {
    float    mean;
    float    M2;
    float    variance;
    uint32_t count;
} welford_t;

// ─── PRESENCE ────────────────────────────────────────────────────
typedef enum {
    PRESENCE_EMPTY  = 0,
    PRESENCE_SINGLE = 1,
    PRESENCE_MULTI  = 2
} presence_state_t;

// ─── OUTPUT PACKET ───────────────────────────────────────────────
// Everything the WebSocket will send
typedef struct {
    presence_state_t presence;
    int              person_count;
    float            presence_score;
    float            motion_score;
    float            br_hz;          // primary breathing rate in Hz (0 = unknown)
    int              br_bpm;         // primary breathing rate in BPM
    float            br_hz_2;        // second person BR in Hz (0 = not detected)
    int              br_bpm_2;       // second person BR in BPM
    float            rssi;
    float            subcarriers[64]; // raw amplitudes for heatmap
} csi_output_t;

#define HAMPEL_THRESH    3.0f
#define PRESENCE_CONFIRM 2
#define WARMUP_FRAMES    150   // frames before detection starts

void             dsp_processor_init(void);
void             dsp_processor_push(const csi_frame_t *frame);
csi_output_t     dsp_processor_get_output(void);