#pragma once
#include "csi_handler.h"

#define HAMPEL_WINDOW   10    // samples each side
#define HAMPEL_THRESH   3.0f  // standard deviations

void dsp_processor_init(void);
void dsp_processor_push(const csi_frame_t *frame);
void dsp_processor_get_signal(float *out, int *top_k_indices);
typedef enum {
    PRESENCE_EMPTY   = 0,
    PRESENCE_SINGLE  = 1,
    PRESENCE_MULTI   = 2
} presence_state_t;

#define PRESENCE_THRESHOLD  10.0f
#define PRESENCE_CONFIRM    5  // frames before confirming state change

presence_state_t dsp_get_presence(const float *cleaned_vals);