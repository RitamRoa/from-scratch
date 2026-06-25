#include "dsp_processor.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TAG "DSP"

// ─── WELFORD STATE PER SUBCARRIER ────────────────────────────────
static welford_t wf[NUM_SUBCARRIERS];
static uint32_t  total_frames = 0;

// ─── SIGNAL BUFFER FOR BR FFT ────────────────────────────────────
#define BR_BUF_LEN 512
static float     br_buf[BR_BUF_LEN];  // variance signal over time
static int        br_buf_idx = 0;
static int        br_buf_full = 0;

// ─── LOCKED SUBCARRIERS ──────────────────────────────────────────
#define TOP_K 8
static int  top_k[TOP_K];
static int  top_k_locked = 0;

// ─── WELFORD UPDATE ──────────────────────────────────────────────
static void welford_update(welford_t *w, float x)
{
    w->count++;
    float delta  = x - w->mean;
    w->mean     += delta / (float)w->count;
    float delta2 = x - w->mean;
    w->M2       += delta * delta2;
    if (w->count > 1)
        w->variance = w->M2 / (float)(w->count - 1);
}

// ─── HAMPEL ON SMALL WINDOW ──────────────────────────────────────
// Applied to BR buffer to remove motion spikes before FFT
static float hampel_point(float *buf, int len, int center)
{
    float tmp[32];
    int   half = 15;
    int   start = center - half;
    int   end   = center + half;
    if (start < 0) start = 0;
    if (end >= len) end = len - 1;
    int   wlen = end - start + 1;
    if (wlen <= 0) return buf[center];

    for (int i = 0; i < wlen; i++) tmp[i] = buf[start + i];

    // insertion sort
    for (int i = 1; i < wlen; i++) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    float median = tmp[wlen / 2];

    float diffs[32];
    for (int i = 0; i < wlen; i++)
        diffs[i] = fabsf(buf[start + i] - median);
    for (int i = 1; i < wlen; i++) {
        float key = diffs[i];
        int j = i - 1;
        while (j >= 0 && diffs[j] > key) { diffs[j+1] = diffs[j]; j--; }
        diffs[j+1] = key;
    }
    float mad = 1.4826f * diffs[wlen / 2];

    if (mad > 0 && fabsf(buf[center] - median) > HAMPEL_THRESH * mad)
        return median;
    return buf[center];
}

// ─── SIMPLE FFT MAGNITUDE ────────────────────────────────────────
// Returns dominant frequency in Hz given sample rate
// Uses Goertzel-style peak search — no full FFT needed for BR
#define SAMPLE_RATE_HZ  10.0f  // BR buffer updated at ~10Hz

static float find_peak_hz(float *buf, int len, float freq_min, float freq_max)
{
    float best_power = 0.0f;
    float best_freq  = 0.0f;
    float freq_res   = SAMPLE_RATE_HZ / (float)len;

    // Search only in physiological range
    int bin_min = (int)(freq_min / freq_res);
    int bin_max = (int)(freq_max / freq_res);
    if (bin_min < 1) bin_min = 1;
    if (bin_max >= len / 2) bin_max = len / 2 - 1;

    for (int k = bin_min; k <= bin_max; k++) {
        // DFT for single bin k
        float real = 0.0f, imag = 0.0f;
        float omega = 2.0f * 3.14159f * k / (float)len;
        for (int n = 0; n < len; n++) {
            real += buf[n] * cosf(omega * n);
            imag -= buf[n] * sinf(omega * n);
        }
        float power = real*real + imag*imag;
        if (power > best_power) {
            best_power = power;
            best_freq  = k * freq_res;
        }
    }
    return best_freq;
}

// ─── SELECT TOP-K SUBCARRIERS ────────────────────────────────────
static void select_top_k(void)
{
    float vars[NUM_SUBCARRIERS];
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        // Skip edge subcarriers — noise dominated
        if (s < 6 || s > 57) {
            vars[s] = 0.0f;
        } else {
            vars[s] = wf[s].variance;
        }
    }

    float used[NUM_SUBCARRIERS];
    memcpy(used, vars, sizeof(vars));

    for (int k = 0; k < TOP_K; k++) {
        int best = 6; // start from non-edge
        for (int s = 7; s < 58; s++) {
            if (used[s] > used[best]) best = s;
        }
        top_k[k] = best;
        used[best] = -1.0f;
    }

    top_k_locked = 1;
    ESP_LOGI(TAG, "Top-K locked: %d %d %d %d %d %d %d %d",
        top_k[0], top_k[1], top_k[2], top_k[3],
        top_k[4], top_k[5], top_k[6], top_k[7]);
}

// ─── PRESENCE DETECTION ──────────────────────────────────────────
// Based on variance score not absolute amplitude
static presence_state_t detect_presence(float variance_score,
                                         float *motion_score_out)
{
    static presence_state_t current   = PRESENCE_EMPTY;
    static int               confirm   = 0;
    static float             baseline  = -1.0f;
    static int               baseline_count = 0;

    // Build baseline from first 50 frames after warmup
    if (baseline < 0.0f) {
        baseline_count++;
        if (baseline_count == 1) baseline = variance_score;
        else baseline = baseline * 0.9f + variance_score * 0.1f;

        if (baseline_count < 50) {
            *motion_score_out = 0.0f;
            return PRESENCE_EMPTY;
        }
        ESP_LOGI(TAG, "Baseline variance: %.3f", baseline);
    }

    // Motion score = how many times above baseline
    float ratio = (baseline > 0.001f) ? (variance_score / baseline) : 1.0f;
    *motion_score_out = ratio;

    presence_state_t detected;
    if (ratio < 1.5f) {
        detected = PRESENCE_EMPTY;
    } else if (ratio < 4.0f) {
        detected = PRESENCE_SINGLE;
    } else {
        detected = PRESENCE_MULTI;
    }

    // Confirm before switching — prevents flicker
    if (detected == current) {
        confirm = 0;
    } else {
        confirm++;
        if (confirm >= PRESENCE_CONFIRM) {
            current = detected;
            confirm = 0;
            const char *s[] = {"EMPTY", "SINGLE", "MULTI"};
            ESP_LOGI(TAG, "Presence -> %s (ratio=%.2f)", s[current], ratio);
        }
    }

    return current;
}

// ─── PUBLIC API ──────────────────────────────────────────────────
void dsp_processor_init(void)
{
    memset(wf, 0, sizeof(wf));
    memset(br_buf, 0, sizeof(br_buf));
    total_frames  = 0;
    br_buf_idx    = 0;
    br_buf_full   = 0;
    top_k_locked  = 0;
    ESP_LOGI(TAG, "DSP ready — Welford variance mode");
}

void dsp_processor_push(const csi_frame_t *frame)
{
    total_frames++;

    // Update Welford stats for every subcarrier
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        welford_update(&wf[s], frame->amp[s]);
    }

    // Lock top-K after warmup
    if (!top_k_locked && total_frames == WARMUP_FRAMES) {
        select_top_k();
    }
}

csi_output_t dsp_processor_get_output(void)
{
    csi_output_t out;
    memset(&out, 0, sizeof(out));

    // Raw subcarrier amplitudes for dashboard heatmap
    // Use current mean as the display value
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        out.subcarriers[s] = wf[s].mean;
    }

    if (!top_k_locked || total_frames < WARMUP_FRAMES) {
        out.presence = PRESENCE_EMPTY;
        return out;
    }

    // Compute combined variance score across top-K subcarriers
    // This is the core signal — high when someone moves, low when empty
    float variance_score = 0.0f;
    for (int k = 0; k < TOP_K; k++) {
        variance_score += wf[top_k[k]].variance;
    }
    variance_score /= TOP_K;

    // Presence detection via variance ratio
    float motion_score = 0.0f;
    out.presence      = detect_presence(variance_score, &motion_score);
    out.motion_score  = motion_score;
    out.presence_score = variance_score;

    // Person count estimate from motion score
    if (out.presence == PRESENCE_EMPTY) {
        out.person_count = 0;
    } else if (out.presence == PRESENCE_SINGLE) {
        out.person_count = 1;
    } else {
        out.person_count = 2; // 2+ — exact count needs multi-node
    }

    // Feed variance signal into BR buffer at ~10Hz
    // (dsp_task calls get_output at ~10Hz already)
    br_buf[br_buf_idx] = variance_score;
    br_buf_idx = (br_buf_idx + 1) % BR_BUF_LEN;
    if (br_buf_idx == 0) br_buf_full = 1;

    // BR extraction — only when single occupancy and buffer full
    if (out.presence == PRESENCE_SINGLE && br_buf_full) {
        // Apply Hampel to remove motion artifact spikes
        float clean_buf[BR_BUF_LEN];
        for (int i = 0; i < BR_BUF_LEN; i++) {
            clean_buf[i] = hampel_point(br_buf, BR_BUF_LEN, i);
        }

        // Remove DC (subtract mean)
        float mean = 0.0f;
        for (int i = 0; i < BR_BUF_LEN; i++) mean += clean_buf[i];
        mean /= BR_BUF_LEN;
        for (int i = 0; i < BR_BUF_LEN; i++) clean_buf[i] -= mean;

        // Find peak in breathing range 0.1-0.5 Hz
        float br_hz = find_peak_hz(clean_buf, BR_BUF_LEN, 0.1f, 0.5f);
        out.br_hz  = br_hz;
        out.br_bpm = (int)(br_hz * 60.0f);
    }

    return out;
}