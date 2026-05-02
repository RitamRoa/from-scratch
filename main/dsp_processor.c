#include "dsp_processor.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TAG         "DSP"
#define HISTORY_LEN 64  // frames of history per subcarrier

static float history[NUM_SUBCARRIERS][HISTORY_LEN];
static int   hist_idx = 0;
static int   hist_count = 0;

// ─── HAMPEL FILTER ───────────────────────────────────────────────
// Removes outliers from a signal window
// Returns cleaned value for center of window
static float hampel_filter(float *window, int len)
{
    int center = len / 2;

    // Calculate median
    float sorted[HISTORY_LEN];
    memcpy(sorted, window, len * sizeof(float));

    // Simple insertion sort (small window, fine for embedded)
    for (int i = 1; i < len; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j+1] = sorted[j];
            j--;
        }
        sorted[j+1] = key;
    }
    float median = sorted[len / 2];

    // Calculate MAD (Median Absolute Difference)
    float diffs[HISTORY_LEN];
    for (int i = 0; i < len; i++) {
        diffs[i] = fabsf(window[i] - median);
    }
    // Sort diffs for median
    for (int i = 1; i < len; i++) {
        float key = diffs[i];
        int j = i - 1;
        while (j >= 0 && diffs[j] > key) {
            diffs[j+1] = diffs[j];
            j--;
        }
        diffs[j+1] = key;
    }
    float mad = 1.4826f * diffs[len / 2]; // consistent with std dev

    // If center value is outlier, replace with median
    if (mad > 0 && fabsf(window[center] - median) > HAMPEL_THRESH * mad) {
        return median; // outlier replaced
    }
    return window[center]; // not an outlier
}

// ─── VARIANCE HELPER ─────────────────────────────────────────────
static float subcarrier_variance(int sub_idx)
{
    if (hist_count < 2) return 0.0f;
    int len = hist_count < HISTORY_LEN ? hist_count : HISTORY_LEN;

    float mean = 0.0f;
    for (int i = 0; i < len; i++) {
        mean += history[sub_idx][i];
    }
    mean /= len;

    float var = 0.0f;
    for (int i = 0; i < len; i++) {
        float d = history[sub_idx][i] - mean;
        var += d * d;
    }
    return var / len;
}

// ─── PUBLIC API ──────────────────────────────────────────────────
void dsp_processor_init(void)
{
    memset(history, 0, sizeof(history));
    hist_idx   = 0;
    hist_count = 0;
    ESP_LOGI(TAG, "DSP processor ready");
}

void dsp_processor_push(const csi_frame_t *frame)
{
    // Store raw amplitudes into history ring
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        history[s][hist_idx] = frame->amp[s];
    }
    hist_idx = (hist_idx + 1) % HISTORY_LEN;
    if (hist_count < HISTORY_LEN) hist_count++;
}

static int locked_indices[TOP_K_SUBCARRIERS] = {0};
static int indices_locked = 0;

void dsp_processor_get_signal(float *out, int *top_k_indices)
{
    if (hist_count < HISTORY_LEN) {
        for (int i = 0; i < TOP_K_SUBCARRIERS; i++) {
            out[i] = 0.0f;
            top_k_indices[i] = i + 1;
        }
        return;
    }

    // Lock subcarrier selection after first stable reading
    if (!indices_locked) {
        float variances[NUM_SUBCARRIERS];
        for (int s = 0; s < NUM_SUBCARRIERS; s++) {
            variances[s] = (s == 0) ? 0.0f : subcarrier_variance(s);
        }
        float used[NUM_SUBCARRIERS];
        memcpy(used, variances, sizeof(used));
        for (int k = 0; k < TOP_K_SUBCARRIERS; k++) {
            int best = 0;
            for (int s = 1; s < NUM_SUBCARRIERS; s++) {
                if (used[s] > used[best]) best = s;
            }
            locked_indices[k] = best;
            used[best] = -1.0f;
        }
        indices_locked = 1;
        ESP_LOGI("DSP", "Subcarriers locked: %d %d %d %d %d",
            locked_indices[0], locked_indices[1], locked_indices[2],
            locked_indices[3], locked_indices[4]);
    }

    // Use locked subcarriers from here on
    float window[HISTORY_LEN];
    for (int k = 0; k < TOP_K_SUBCARRIERS; k++) {
        int s = locked_indices[k];
        if (top_k_indices) top_k_indices[k] = s;
        for (int i = 0; i < HISTORY_LEN; i++) {
            int idx = (hist_idx - HISTORY_LEN + i + HISTORY_LEN) % HISTORY_LEN;
            window[i] = history[s][idx];
        }
        out[k] = hampel_filter(window, HISTORY_LEN);
    }
}
presence_state_t dsp_get_presence(const float *cleaned_vals)
{
    static presence_state_t current = PRESENCE_EMPTY;
    static int confirm_count = 0;

    // Use only first 3 locked subcarriers for presence
    // They are highest variance = most reliable
    float max_val = 0.0f;
    float avg_val = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (cleaned_vals[i] > max_val) max_val = cleaned_vals[i];
        avg_val += cleaned_vals[i];
    }
    avg_val /= 3.0f;

    presence_state_t detected;
    if (avg_val < 8.0f) {
        detected = PRESENCE_EMPTY;
    } else if (avg_val < 15.5f) {
        detected = PRESENCE_SINGLE;
    } else {
        detected = PRESENCE_MULTI;
    }

    if (detected == current) {
        confirm_count = 0;
    } else {
        confirm_count++;
        if (confirm_count >= PRESENCE_CONFIRM) {
            current = detected;
            confirm_count = 0;
        }
    }

    return current;
}