#pragma once
#include "csi_handler.h"

#define HAMPEL_WINDOW   10    // samples each side
#define HAMPEL_THRESH   3.0f  // standard deviations

void dsp_processor_init(void);
void dsp_processor_push(const csi_frame_t *frame);
void dsp_processor_get_signal(float *out, int *top_k_indices);