#include "dsp_processor.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TAG "DSP"

// ─── WELFORD STATE PER SUBCARRIER ────────────────────────────────
// wf[]      = slow baseline  (updates at 1/10 rate — long-term env)
// fast_wf[] = fast activity  (updates every frame — current motion)
static welford_t wf[NUM_SUBCARRIERS];
static welford_t fast_wf[NUM_SUBCARRIERS];
static uint32_t  total_frames = 0;

// ─── SIGNAL BUFFER FOR BR FFT ────────────────────────────────────
#define BR_BUF_LEN 256
static float br_buf[BR_BUF_LEN];   // fast_variance signal over time
static int   br_buf_idx  = 0;
static int   br_buf_fill = 0;       // how many samples accumulated (capped at BR_BUF_LEN)


// ─── EMA / WELFORD UPDATE ────────────────────────────────────────
static void welford_update(welford_t *w, float x, float alpha_mean, float alpha_var)
{
    if (w->count == 0) {
        w->mean     = x;
        w->variance = 0.0f;
        w->count    = 1;
    } else {
        float diff   = x - w->mean;
        w->mean     += alpha_mean * diff;
        w->variance  = (1.0f - alpha_var) * w->variance + alpha_var * diff * diff;
        w->count++;
    }
}


// ─── HAMPEL ON SMALL WINDOW ──────────────────────────────────────
// Applied to BR buffer to remove motion spikes before FFT
static float hampel_point(float *buf, int len, int center)
{
    float tmp[32];
    int   half  = 15;
    int   start = center - half;
    int   end   = center + half;
    if (start < 0)    start = 0;
    if (end >= len)   end   = len - 1;
    int   wlen  = end - start + 1;
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


// ─── DFT PEAK SEARCH ─────────────────────────────────────────────
// Returns dominant frequency in Hz within [freq_min, freq_max].
// Also writes the peak power to *peak_power_out.
// Uses per-bin DFT — adequate resolution for BR at this buffer size.
#define SAMPLE_RATE_HZ 10.0f   // BR buffer updated at ~10 Hz

static float find_peak_hz(float *buf, int len,
                           float freq_min, float freq_max,
                           float *peak_power_out)
{
    float best_power = 0.0f;
    float best_freq  = 0.0f;
    float freq_res   = SAMPLE_RATE_HZ / (float)len;

    int bin_min = (int)(freq_min / freq_res);
    int bin_max = (int)(freq_max / freq_res);
    if (bin_min < 1)          bin_min = 1;
    if (bin_max >= len / 2)   bin_max = len / 2 - 1;

    for (int k = bin_min; k <= bin_max; k++) {
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

    if (peak_power_out) *peak_power_out = best_power;
    return best_freq;
}

// ─── NOISE FLOOR ESTIMATE ────────────────────────────────────────
// Average power across all bins in range — used for 3× gate.
static float noise_floor_power(float *buf, int len,
                                float freq_min, float freq_max)
{
    float freq_res = SAMPLE_RATE_HZ / (float)len;
    int   bin_min  = (int)(freq_min / freq_res);
    int   bin_max  = (int)(freq_max / freq_res);
    if (bin_min < 1)         bin_min = 1;
    if (bin_max >= len / 2)  bin_max = len / 2 - 1;

    float total = 0.0f;
    int   count = 0;
    for (int k = bin_min; k <= bin_max; k++) {
        float real = 0.0f, imag = 0.0f;
        float omega = 2.0f * 3.14159f * k / (float)len;
        for (int n = 0; n < len; n++) {
            real += buf[n] * cosf(omega * n);
            imag -= buf[n] * sinf(omega * n);
        }
        total += real*real + imag*imag;
        count++;
    }
    return (count > 0) ? (total / count) : 0.0f;
}

// ─── SECOND PEAK SEARCH ──────────────────────────────────────────
// Searches for a second BR peak at least 0.05 Hz away from primary,
// with at least 40% of primary peak power.
static float find_second_peak_hz(float *buf, int len,
                                  float freq_min, float freq_max,
                                  float primary_hz, float primary_power,
                                  float *second_power_out)
{
    float freq_res      = SAMPLE_RATE_HZ / (float)len;
    float min_sep_hz    = 0.05f;
    float min_power     = primary_power * 0.4f;
    float best_power    = 0.0f;
    float best_freq     = 0.0f;

    int bin_min = (int)(freq_min / freq_res);
    int bin_max = (int)(freq_max / freq_res);
    if (bin_min < 1)         bin_min = 1;
    if (bin_max >= len / 2)  bin_max = len / 2 - 1;

    for (int k = bin_min; k <= bin_max; k++) {
        float freq = k * freq_res;
        if (fabsf(freq - primary_hz) < min_sep_hz) continue;  // too close to primary

        float real = 0.0f, imag = 0.0f;
        float omega = 2.0f * 3.14159f * k / (float)len;
        for (int n = 0; n < len; n++) {
            real += buf[n] * cosf(omega * n);
            imag -= buf[n] * sinf(omega * n);
        }
        float power = real*real + imag*imag;
        if (power > best_power && power >= min_power) {
            best_power = power;
            best_freq  = freq;
        }
    }

    if (second_power_out) *second_power_out = best_power;
    return best_freq;
}


// ─── PRESENCE DETECTION ──────────────────────────────────────────
// Receives fast_var/slow_var ratio — motion-responsive, no freeze needed.
static presence_state_t detect_presence(float ratio,
                                         float *motion_score_out)
{
    static presence_state_t current = PRESENCE_EMPTY;
    static int               confirm = 0;

    *motion_score_out = ratio;

    presence_state_t detected;
    if (ratio < 1.18f) {
        detected = PRESENCE_EMPTY;
    } else if (ratio < 6.0f) {
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
    memset(wf,      0, sizeof(wf));
    memset(fast_wf, 0, sizeof(fast_wf));
    memset(br_buf,  0, sizeof(br_buf));
    total_frames = 0;
    br_buf_idx   = 0;
    br_buf_fill  = 0;
    ESP_LOGI(TAG, "DSP ready — Dual-rate Welford mode");
}

void dsp_processor_push(const csi_frame_t *frame)
{
    total_frames++;

    // fast_wf: every frame — tracks current activity
    // alpha_mean=0.01 → TC ~100 frames (~1s at 100Hz)
    // alpha_var =0.02 → variance settles in ~50 frames
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        welford_update(&fast_wf[s], frame->amp[s], 0.01f, 0.02f);
    }

    // slow_wf: every 10th frame — long-term environment baseline
    // alpha_mean=0.005 @ 1/10 rate → effective TC ~2000 frames (~20s at 100Hz)
    // alpha_var =0.01  @ 1/10 rate → variance TC ~1000 frames (~10s)
    if (total_frames % 10 == 0) {
        for (int s = 0; s < NUM_SUBCARRIERS; s++) {
            welford_update(&wf[s], frame->amp[s], 0.005f, 0.01f);
        }
    }
}

csi_output_t dsp_processor_get_output(void)
{
    csi_output_t out;
    memset(&out, 0, sizeof(out));

    // Heatmap: use fast mean for more responsive display
    for (int s = 0; s < NUM_SUBCARRIERS; s++) {
        out.subcarriers[s] = fast_wf[s].mean;
    }

    if (total_frames < WARMUP_FRAMES) {
        out.presence = PRESENCE_EMPTY;
        return out;
    }

    // ── Compute fast_variance and slow_variance (subcarriers 6–57) ──
    float fast_var = 0.0f;
    float slow_var = 0.0f;
    int   count    = 0;
    for (int s = 6; s <= 57; s++) {
        fast_var += fast_wf[s].variance;
        slow_var += wf[s].variance;
        count++;
    }
    fast_var /= (float)count;
    slow_var /= (float)count;

    // Ratio: how much more active fast is vs slow baseline
    float ratio = (slow_var > 0.0001f) ? (fast_var / slow_var) : 1.0f;

    // ── Presence detection ───────────────────────────────────────
    float motion_score  = 0.0f;
    out.presence        = detect_presence(ratio, &motion_score);
    out.motion_score    = motion_score;
    out.presence_score  = fast_var;

    // Person count
    if (out.presence == PRESENCE_EMPTY) {
        out.person_count = 0;
    } else if (out.presence == PRESENCE_SINGLE) {
        out.person_count = 1;
    } else {
        out.person_count = 2;
    }

    // ── Feed fast_variance into BR buffer (~10Hz call rate) ──────
    br_buf[br_buf_idx] = fast_var;
    br_buf_idx = (br_buf_idx + 1) % BR_BUF_LEN;
    if (br_buf_fill < BR_BUF_LEN) br_buf_fill++;

    // ── Motion-clear guard ───────────────────────────────────────
    // Only attempt BR when motion has been calm for 30+ consecutive calls
    static int motion_clear_count = 0;
    if (motion_score > 1.6f) {
        motion_clear_count = 0;
    } else {
        if (motion_clear_count < 100) motion_clear_count++;
    }

    // ── BR extraction conditions ─────────────────────────────────
    int br_ready = (out.presence == PRESENCE_SINGLE) &&
                   (motion_score < 1.6f)             &&
                   (br_buf_fill >= BR_BUF_LEN)       &&
                   (motion_clear_count > 30);

    if (br_ready) {
        // Hampel filter — remove motion-spike outliers
        float clean_buf[BR_BUF_LEN];
        for (int i = 0; i < BR_BUF_LEN; i++) {
            clean_buf[i] = hampel_point(br_buf, BR_BUF_LEN, i);
        }

        // DC removal
        float dc = 0.0f;
        for (int i = 0; i < BR_BUF_LEN; i++) dc += clean_buf[i];
        dc /= BR_BUF_LEN;
        for (int i = 0; i < BR_BUF_LEN; i++) clean_buf[i] -= dc;

        // Noise floor in breathing band
        float nf = noise_floor_power(clean_buf, BR_BUF_LEN, 0.1f, 0.5f);

        // Primary BR peak — must be 3× noise floor to count
        float primary_power = 0.0f;
        float primary_hz    = find_peak_hz(clean_buf, BR_BUF_LEN,
                                            0.1f, 0.5f, &primary_power);

        if (primary_power > 3.0f * nf && primary_hz > 0.0f) {
            out.br_hz  = primary_hz;
            out.br_bpm = (int)(primary_hz * 60.0f);

            // ── Dual-peak: second person BR ──────────────────────
            float second_power = 0.0f;
            float second_hz    = find_second_peak_hz(clean_buf, BR_BUF_LEN,
                                                      0.1f, 0.5f,
                                                      primary_hz, primary_power,
                                                      &second_power);

            if (second_hz > 0.0f && second_power > 0.0f) {
                // Upgrade single → multi based on two distinct BR frequencies
                out.br_hz_2  = second_hz;
                out.br_bpm_2 = (int)(second_hz * 60.0f);
                if (out.presence == PRESENCE_SINGLE) {
                    out.presence     = PRESENCE_MULTI;
                    out.person_count = 2;
                    ESP_LOGI(TAG, "Dual BR detected: %.2fHz + %.2fHz → MULTI",
                             primary_hz, second_hz);
                }
            }
        }
        // If peak not strong enough: br_bpm stays 0 (not confident)
    }

    return out;
}