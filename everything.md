# SPARS — WiFi CSI Presence & Vital Signs Monitor
### Comprehensive Technical Reference

---

## Abstract

SPARS (Spatial Presence And Respiratory Sensing) is a passive, contactless human sensing system built on a single ESP32-S3 microcontroller. It exploits **Channel State Information (CSI)** — the per-subcarrier amplitude and phase data embedded in every 802.11n WiFi packet — to detect the presence of humans and extract physiological signals (breathing rate) without any camera, PIR, or wearable sensor.

The fundamental physical principle: a human body, being ~60% water, absorbs and scatters 2.4 GHz RF energy. Micro-scale thoracic movement from breathing (1–20 mm displacement) modulates the multipath propagation environment, producing measurable periodic variation in received signal amplitude across subcarriers. SPARS separates this subtle signal from gross motion and environmental drift using a dual-rate Welford online variance estimator, a Hampel outlier filter, and a per-bin Goertzel-style DFT — all running in real time on the ESP32-S3's second CPU core with no RTOS heap allocation in the hot path.

Person count is not inferred from motion amplitude (which is unreliable with a single receiver). Instead, MULTI-occupancy is confirmed exclusively through FFT dual-peak detection: two people at rest produce two distinct spectral peaks in the breathing band (0.1–0.5 Hz) separated by ≥ 0.05 Hz, each with ≥ 40% relative power. This architectural decision eliminates a major source of false positives.

The firmware exposes a WebSocket endpoint and an embedded HTTP server. A standalone HTML dashboard (`heatmap3d.html`) renders the live data as an isometric 3D polar heatmap on the browser's `<canvas>` element with no external JS dependencies except the embedded dashboard file itself.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ESP32-S3 (Dual Core)                               │
│                                                                             │
│  CORE 0 (WiFi/Network)              CORE 1 (DSP / Application)              │
│  ────────────────────               ──────────────────────────              │
│                                                                             │
│  ┌──────────────────┐               ┌───────────────────────────────────┐   │
│  │  WiFi Driver     │  CSI packet   │  dsp_task (FreeRTOS, 8KB stack)   │   │
│  │  802.11n SoftAP  │──────────────▶│                                   │   │
│  │  SSID:           │               │  ┌─────────────────────────────┐  │   │
│  │  SPARS-DEMO-NODE │               │  │  csi_handler_read()         │  │   │
│  └────────┬─────────┘               │  │  pops from ring buffer      │  │   │
│           │                         │  └──────────────┬──────────────┘  │   │
│  csi_callback() fires               │                 │ csi_frame_t      │   │
│  on every received pkt              │  ┌──────────────▼──────────────┐  │   │
│           │                         │  │  dsp_processor_push()       │  │   │
│  ┌────────▼──────────────────┐      │  │  Update fast_wf + slow_wf   │  │   │
│  │  Laptop MAC Filter:       │      │  │  per subcarrier (Fast/Slow) │  │   │
│  │  Drops LAPTOP_MAC         │      │  └──────────────┬──────────────┘  │   │
│  │  Passes everything else   │      │                 │ every 10 frames  │   │
│  ├───────────────────────────┤      │  ┌──────────────▼──────────────┐  │   │
│  │  Amplitude extraction     │      │  │  dsp_processor_get_output() │  │   │
│  │  amp[i] = √(I²+Q²)        │      │  │  • Ratio = fast_var/slow_var│  │   │
│  │  RSSI EMA (α=0.1)         │      │  │  • detect_presence()        │  │   │
│  └────────┬──────────────────┘      │  │  • BR FFT pipeline          │  │   │
│           │                         │  │  • JSON serialise           │  │   │
│  ┌────────▼──────────────────┐      │  └──────────────┬──────────────┘  │   │
│  │  Ring Buffer              │      │                 │ csi_output_t     │   │
│  │  128 × csi_frame_t        │      │                 │                  │   │
│  │  lock-free, Core0 writes  │      │                 │                  │   │
│  │  Core1 reads              │      │                 │                  │   │
│  └───────────────────────────┘      └─────────────────│───────────────────┘   │
│                                                       │                      │
│  ┌────────────────────────────────────────────────────▼────────────────┐     │
│  │  HTTP Server (esp_http_server)                                       │     │
│  │  GET /         → Blank fallback page                                │     │
│  │  GET /monitor  → serves heatmap3d.html (chunked, 4 KB slices)       │     │
│  │  GET /ws       → WebSocket upgrade with TCP_NODELAY, async JSON     │     │
│  └─────────────────────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                            WebSocket ws://
                                    │
            ┌───────────────────────▼──────────────────────────┐
            │  Browser  —  heatmap3d.html                       │
            │                                                   │
            │  updateHUD(data)                                  │
            │  buildHeatTarget(data) ─── canvas 2D render loop │
            │  Isometric 3D polar heatmap (8 rings × 16 segs)  │
            │  Vitals card, motion histogram, BR display        │
            └───────────────────────────────────────────────────┘
```

---

## File Structure

```
from-scratch/
├── heatmap3d.html            # Standalone dashboard (development copy, 1468 lines)
├── implementation_plan.md    # Design notes
└── csi-monitor/              # ESP-IDF project root
    ├── CMakeLists.txt        # Project cmake — requires IDF_PATH env var
    ├── sdkconfig             # ESP-IDF menuconfig output (WiFi, FreeRTOS, etc.)
    ├── monitor.html          # Embedded HTML — compiled into firmware binary
    ├── dashboard.html        # Admin dashboard (separate UI)
    ├── three.min.js          # Three.js (referenced by alternate UI)
    └── main/
        ├── CMakeLists.txt    # Component cmake — registers SRCS + EMBED_TXTFILES
        ├── csi_handler.h     # Public API: csi_frame_t, constants, init/read
        ├── csi_handler.c     # CSI callback, ring buffer, amplitude extraction
        ├── dsp_processor.h   # Public API: welford_t, presence_state_t, csi_output_t
        ├── dsp_processor.c   # All signal processing: Welford, Hampel, DFT, presence
        └── main.c            # Entry point: WiFi init, HTTP server, task creation
```

**Key build note:** `monitor.html` is compiled directly into the firmware as a binary symbol via `EMBED_TXTFILES`. The C code references it through `_binary_monitor_html_start` / `_binary_monitor_html_end` linker symbols — no SD card or SPIFFS required.

---

## Data Flow — End to End

```
WiFi packet received (Core 0, ~100 Hz)
    │
    ▼
csi_callback()
    │  Extract I/Q pairs from buf[len/2 × 2]
    │  Compute amp[i] = √(I²+Q²) for 64 subcarriers
    │  Smooth RSSI: rssi_smooth = 0.9×prev + 0.1×new  (α=0.1, TC≈10 frames)
    │  rb_push(&frame)  →  ring buffer slot (drop if full)
    │
    ▼ (crosses core boundary via shared memory ring buffer)
    │
dsp_task() polls every 10 ms (Core 1)
    │  csi_handler_read(&frame) → rb_pop()
    │  dsp_processor_push(&frame)
    │      fast_wf[s]: update every frame     α_mean=0.02, α_var=0.04
    │      slow_wf[s]: update every 10th frame α_mean=0.002, α_var=0.005
    │
    │  Every 10 frames (~10 Hz effective output rate):
    │  dsp_processor_get_output()
    │      ├── Compute fast_var, slow_var (subcarriers 6–57)
    │      ├── ratio = fast_var / slow_var  [capped at 20.0]
    │      ├── detect_presence(ratio)  →  EMPTY | SINGLE
    │      ├── person_count = 0 | 1  (MULTI only via FFT below)
    │      ├── BR pipeline (if br_ready):
    │      │       Hampel filter → DC remove → noise floor → find_peak_hz()
    │      │       find_second_peak_hz() → if confirmed: MULTI, person_count=2
    │      └── JSON serialise → send_ws_data()
    │
    ▼
WebSocket frame → Browser
    │
    ▼
updateHUD(data) + buildHeatTarget(data) → canvas render @ ~60 fps
```

---

## Module Reference

### `csi_handler.h` / `csi_handler.c`

**Purpose:** Intercept WiFi CSI callbacks on Core 0, extract amplitude, and buffer frames into a lock-free ring buffer for Core 1 consumption.

#### Constants

| Name | Value | Meaning |
|------|-------|---------|
| `CSI_BUF_LEN` | 128 | Ring buffer depth in frames |
| `NUM_SUBCARRIERS` | 64 | Subcarriers captured per frame |
| `TOP_K_SUBCARRIERS` | 10 | Reserved (unused in current build) |

#### Struct: `csi_frame_t`
```c
typedef struct {
    float  amp[NUM_SUBCARRIERS]; // √(I²+Q²) per subcarrier
    int8_t rssi;                 // EMA-smoothed RSSI, dBm
} csi_frame_t;
```

#### Functions

| Function | Signature | Description |
|----------|-----------|-------------|
| `csi_handler_init` | `void (void)` | Configures CSI (LLTF only, no HT-LTF), registers `csi_callback`, enables CSI collection |
| `csi_handler_read` | `int (csi_frame_t *out)` | Non-blocking pop from ring buffer. Returns 1 on success, 0 if empty |

#### Internal Functions (static)

| Function | Description |
|----------|-------------|
| `csi_callback` | WiFi driver callback. Copies I/Q → amplitude immediately (never stores pointer). Computes RSSI EMA. Calls `rb_push`. |
| `rb_push` | Write to ring buffer. Drops frame silently if full (Core 0 safety — no blocking). |
| `rb_pop` | Read from ring buffer. Returns 0 if empty. |
| `rb_next` | Modular index increment. |
| `rb_empty` | Checks `rb_read == rb_write`. |

#### WiFi CSI Configuration
```c
wifi_csi_config_t cfg = {
    .lltf_en           = true,   // Long Training Field — most stable subcarriers
    .htltf_en          = false,  // HT-LTF disabled — fewer subcarriers, cleaner signal
    .stbc_htltf2_en    = false,
    .ltf_merge_en      = true,   // Merge multiple LTF fields for SNR improvement
    .channel_filter_en = false,  // Raw CSI — no hardware smoothing
    .manu_scale        = false,
};
```

**Why LLTF only:** LLTF gives 52 usable subcarriers (6–57 in the 64-element array) at better SNR than HT-LTF. The outer subcarriers (0–5, 58–63) are DC/guard band and excluded from variance computation.

---

### `dsp_processor.h` / `dsp_processor.c`

**Purpose:** All signal processing. Dual-rate Welford variance tracking, presence detection with hysteresis, Hampel outlier filtering, per-bin DFT for breathing rate extraction, dual-peak second person detection.

#### Constants

| Name | Value | Meaning |
|------|-------|---------|
| `HAMPEL_THRESH` | 3.0 | Outlier gate: points > 3×MAD from median are replaced |
| `PRESENCE_CONFIRM` | 2 | Consecutive mismatching detections required to switch state |
| `WARMUP_FRAMES` | 80 | Frames discarded at startup before detection begins |
| `BR_BUF_LEN` | 256 | Samples in breathing rate ring buffer |
| `SAMPLE_RATE_HZ` | 10.0 | Effective BR buffer sample rate (get_output called every 10 frames) |

#### Struct: `welford_t`
```c
typedef struct {
    float    mean;
    float    M2;         // unused field (historical — EMA replaces Welford M2)
    float    variance;   // exponentially-weighted variance
    uint32_t count;      // total update count
} welford_t;
```

#### Enum: `presence_state_t`
```c
typedef enum {
    PRESENCE_EMPTY  = 0,
    PRESENCE_SINGLE = 1,
    PRESENCE_MULTI  = 2   // only set via dual BR peak FFT — never by motion score
} presence_state_t;
```

#### Struct: `csi_output_t` — the WebSocket packet
```c
typedef struct {
    presence_state_t presence;     // EMPTY / SINGLE / MULTI
    int              person_count; // 0, 1, or 2
    float            presence_score; // raw fast_var (diagnostic)
    float            motion_score;   // fast_var/slow_var ratio (capped at 20)
    float            br_hz;          // primary BR in Hz (0 = not yet computed)
    int              br_bpm;         // primary BR in BPM (persists last known)
    float            br_hz_2;        // second person BR in Hz
    int              br_bpm_2;       // second person BR in BPM
    float            rssi;           // from frame (EMA-smoothed)
    float            subcarriers[64]; // fast_wf mean per subcarrier (for heatmap)
} csi_output_t;
```

#### Public API

| Function | Signature | Description |
|----------|-----------|-------------|
| `dsp_processor_init` | `void (void)` | Zeros all Welford state, BR buffer, frame counters |
| `dsp_processor_push` | `void (const csi_frame_t *)` | Updates fast_wf every frame, slow_wf every 10th frame |
| `dsp_processor_get_output` | `csi_output_t (void)` | Runs full pipeline: ratio, presence, BR FFT, JSON-ready struct |

#### Internal Functions (static)

| Function | Signature | Role |
|----------|-----------|------|
| `welford_update` | `(welford_t*, float x, float α_mean, float α_var)` | EMA mean + EMA variance update in one pass |
| `hampel_point` | `(float* buf, int len, int center) → float` | Sliding window Hampel identifier on 31-point window |
| `find_peak_hz` | `(float* buf, int len, float f_min, float f_max, float* power) → float` | Per-bin DFT peak search, returns Hz of dominant frequency |
| `noise_floor_power` | `(float* buf, int len, float f_min, float f_max) → float` | Mean power across all bins in range — 3× gate reference |
| `find_second_peak_hz` | `(float* buf, int len, float f_min, float f_max, float primary_hz, float primary_power, float* power) → float` | Second peak search: must be ≥ 0.05 Hz from primary, ≥ 40% relative power |
| `detect_presence` | `(float ratio, float* motion_score_out) → presence_state_t` | Stateful EMPTY/SINGLE classifier with hysteresis and confirm gate |

---

## Signal Processing Deep Dive

### 1. Dual-Rate Welford Variance Tracker

Two independent Welford arrays run per subcarrier:

```
fast_wf[64]  — updated every frame
    α_mean = 0.02  →  mean TC ≈ 50 frames  ≈ 0.5s
    α_var  = 0.04  →  variance TC ≈ 25 frames

slow_wf[64]  — updated every 10th frame
    α_mean = 0.002  →  effective mean TC ≈ 5000 frames ≈ 50s
    α_var  = 0.005  →  effective variance TC ≈ 2000 frames ≈ 20s
```

The **update equation** (EMA variant, not classical Welford):
```
diff      = x − mean
mean     += α_mean × diff
variance  = (1 − α_var) × variance + α_var × diff²
```

`slow_wf` is the long-term environmental fingerprint. `fast_wf` tracks the current "activity level." The ratio of their variances is the primary detection signal.

**Why slower alphas for slow_wf:** With `α=0.002`, the baseline takes ~50 seconds to adapt after a person leaves. This means re-entry within that window produces a larger ratio spike because the baseline still "remembers" the presence pattern. Without this, the empty-room ratio (fast/slow) collapsed to 0.37–0.96, making re-entry invisible.

### 2. Presence Detection — `detect_presence()`

```
ratio = fast_var_avg(subs 6–57) / slow_var_avg(subs 6–57)
ratio = clamp(ratio, 0, 20.0)   ← hard cap prevents startup explosion

if ratio < 1.05 → EMPTY
else            → SINGLE
(MULTI is NEVER set here)
```

**State machine with two guards:**

1. **Hold frames** (exit hysteresis): When transitioning SINGLE → EMPTY, the state is held for 15 consecutive frames (~1.5s at 10 Hz) before actually dropping. Prevents presence dropping during momentary stillness.

2. **Confirm counter**: A state change requires `PRESENCE_CONFIRM = 2` consecutive non-matching detections before committing. Prevents single-frame flicker from triggering state logs.

**Key design decision — MULTI removed from threshold logic:**
Motion ratio alone cannot distinguish 1 vs 2 people with a single receiver. A person moving alone can produce ratios > 5 just from vigorous movement. MULTI is exclusively promoted from FFT dual-peak confirmation.

### 3. Breathing Rate Pipeline

**Buffer:** `br_buf[256]` stores `fast_var` sampled at ~10 Hz. At `SAMPLE_RATE_HZ = 10 Hz` and `BR_BUF_LEN = 256`, the buffer represents 25.6 seconds of variance history with frequency resolution of `10/256 ≈ 0.039 Hz`.

**Gate conditions (all must be true):**
```c
br_ready = (presence == PRESENCE_SINGLE)
         && (motion_score < 3.0f)      // not actively moving
         && (br_buf_fill >= 256)        // buffer fully warmed up
         && (motion_clear_count > 10)   // 10+ consecutive calm frames
```

`motion_clear_count` resets to 0 whenever `motion_score > 2.5`. Gross motion invalidates the buffer's spectral content.

**Pipeline when `br_ready = true`:**

```
br_buf[256]  (ring buffer, fast_var over time)
    │
    ▼
Hampel filter  (per-point, 31-sample window)
    Replaces spikes > 3×MAD from local median
    Removes motion transients without distorting baseline
    │
    ▼
DC removal  (subtract mean of entire buffer)
    Removes the static offset so FFT sees only AC variation
    │
    ▼
Noise floor estimate  (noise_floor_power, 0.1–0.5 Hz)
    Mean power across all bins in the breathing band
    │
    ▼
find_peak_hz  (per-bin DFT, 0.1–0.5 Hz = 6–30 BPM)
    primary_hz, primary_power
    Gate: primary_power > 3.0 × noise_floor
    │
    ├── br_hz = primary_hz
    ├── br_bpm = (int)(primary_hz × 60)
    │
    ▼
find_second_peak_hz  (same range, skip ±0.05 Hz around primary)
    Requires: |freq − primary_hz| ≥ 0.05 Hz
    Requires: second_power ≥ 0.4 × primary_power
    │
    If confirmed:
    ├── br_hz_2, br_bpm_2 set
    ├── presence = PRESENCE_MULTI
    └── person_count = 2
```

**DFT implementation:** Per-bin Goertzel-style computation (not an FFT). For each frequency bin k:
```
ω = 2π × k / N
real += buf[n] × cos(ω × n)
imag -= buf[n] × sin(ω × n)
power = real² + imag²
```
This is O(N²) but adequate at N=256 and ~10 Hz call rate on the ESP32-S3's 240 MHz Xtensa LX7.

**Last-known BR persistence:**
```c
static int last_known_bpm = 0;
if (out.br_bpm > 0) last_known_bpm = out.br_bpm;
if (out.br_bpm == 0 && last_known_bpm > 0)
    out.br_bpm = last_known_bpm;
```
Once BR is computed, it persists in `out.br_bpm` across future frames where conditions aren't met (motion, presence change). Prevents the display from blanking.

### 4. RSSI Smoothing

```c
rssi_smooth = rssi_smooth * 0.9f + (float)info->rx_ctrl.rssi * 0.1f;
frame.rssi = (int8_t)rssi_smooth;
```

α=0.1 gives TC≈10 frames. Raw RSSI has 3–5 dBm frame-to-frame jitter due to multipath and body orientation. The EMA removes this jitter so the heatmap distance ring (which is RSSI-driven) doesn't flicker between adjacent rings.

---

### `main.c`

**Purpose:** Entry point. WiFi initialization, HTTP/WebSocket server, FreeRTOS task creation.

#### Key Functions

| Function | Core/Context | Role |
|----------|-------------|------|
| `app_main` | Core 0 | Entry. NVS init → WiFi AP → CSI init → HTTP server → spawn dsp_task |
| `wifi_init` | Core 0 | Configures Open SoftAP mode with SSID "SPARS-DEMO-NODE" on Channel 6 |
| `wifi_event_handler` | Core 0 | Logs station association/disassociation events |
| `start_webserver` | Core 0 | Starts esp_http_server with 8KB stack, registers `/`, `/monitor`, `/ws` |
| `root_handler` | HTTP thread | Serves a blank, dark HTML fallback page to regular clients |
| `monitor_handler` | HTTP thread | Serves `heatmap3d.html` in 4KB chunks to avoid stack overflow |
| `ws_handler` | HTTP thread | Accepts WebSocket upgrade, applies TCP_NODELAY, registers clients |
| `send_ws_data` | Core 1 | Thread-safely broadcasts telemetry JSON to active WebSocket descriptors |
| `dsp_task` | Core 1 | Main DSP loop: read frame → push → every 10 frames: get_output → JSON → WS |

#### Task Configuration
```
dsp_task   pinned Core 1,  priority 4,  stack 8192 bytes
```

Higher priority for ping ensures WiFi traffic (and therefore CSI data) is maintained even when DSP is busy.

#### JSON Payload Format
```json
{
  "presence":     "SINGLE",
  "person_count": 1,
  "motion_score": 1.47,
  "br_bpm":       15,
  "br_bpm_2":     0,
  "br_avg":       15,
  "rssi":         -52,
  "subcarriers":  [12.3, 11.8, 13.1, ...]  // 64 values
}
```

`br_avg` is computed in `dsp_task`: average of `br_bpm` + `br_bpm_2` if both > 0, else just `br_bpm`.

#### Smart CSI Packet Filtering & Socket Optimization

1. **Inverted MAC Filtering Engine**:
   - The ESP32 collects CSI packets from any active device. To avoid stationary laptop telemetry requests polluting the Welford DSP math variance and resetting presence output to EMPTY, the callback filters packets using a hardcoded exclusion check:
     `static const uint8_t LAPTOP_MAC[6] = {0xe0, 0xd5, 0x5d, 0x1a, 0x53, 0x6f};`
   - In `csi_mac_filter_check()`, incoming packets matching the laptop's MAC are instantly dropped (`return false`), while all other packets (like those from the judge's phone) are fed straight to the DSP engine.

2. **TCP_NODELAY Socket Optimization**:
   - Inside `ws_handler`, Nagle's buffering algorithm is deactivated on the WebSocket socket descriptor by setting `TCP_NODELAY` via `setsockopt()`. This ensures telemetry updates are pushed out instantly with zero buffering lag:
     `setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));`

---

## Frontend — `heatmap3d.html`

A self-contained single-file HTML dashboard (1468 lines). No build step, no npm, no external dependencies beyond the WebSocket connection. Runs directly in a browser.

### Rendering Architecture

**Canvas 2D** (not WebGL). The `<canvas>` element covers the full viewport. A `requestAnimationFrame` loop runs at ~60fps, independent of the WebSocket update rate (~10 Hz).

**Coordinate system:**
- Isometric projection: `x_screen = OX + x - y×0.5`, `y_screen = OY + (x+y)×0.4`
- 8 concentric rings × 16 angular segments = 128 cells
- Each segment maps to 4 CSI subcarriers (64 / 16 = 4)
- Ring radius is RSSI-driven: RSSI > -35 → ring 1.5, RSSI > -45 → ring 2.5, etc.

**Draw order:** Cells are depth-sorted by their screen Y before drawing (painter's algorithm). This gives correct isometric overlap without Z-buffer.

**Heat interpolation:** `hcurrent[r][s]` lerps toward `htarget[r][s]` at 5% per frame:
```js
hcurrent[r][s] += (htarget[r][s] - hcurrent[r][s]) * 0.05;
```
This smooths out choppy data updates.

### Key JavaScript Functions

| Function | Role |
|----------|------|
| `render()` | rAF loop. Lerps heat, sorts cells by screenY, draws all 128 cells, beacons, compass, ring labels |
| `drawCell(ri, si, heat)` | Renders one isometric cell: top face + two walls (front-facing determined by viewAngle) |
| `buildHeatTarget(data)` | Converts 64 subcarrier amplitudes → 16 segment values → Gaussian ring spread → `htarget` |
| `updateHUD(data)` | Updates all DOM elements: presence, person count, lastKnownBR, motion histogram, RSSI, signal strength |
| `isoPoint(ring, angDeg)` | Projects polar (ring, angle) → isometric screen (x, y), accounting for `viewAngle` rotation |
| `heatRGB(h)` | Maps heat value [0,1] → 5-stop color gradient (beige → green → yellow → orange → red) |
| `find_peak_hz` / `noise_floor_power` / `find_second_peak_hz` | DFT helpers (same algorithm as firmware, mirrored) |
| `detect_presence(ratio)` | Presence state machine (same logic as firmware, mirrored) |
| `genDemo()` | Generates synthetic presence/subcarrier data for offline demo mode |
| `connectWS()` | Opens WebSocket to `ws://<IP>/ws`. Falls back to demo after 5s if no connection |
| `setConnState(st)` | Updates connection indicator (LIVE / DEMO / OFFLINE) |
| `showIpOverlay()` | Shows IP entry modal when no IP is resolvable from URL/hostname |

### Breathing Rate Display

```
lastKnownBR (module-level variable) — never resets

On each WebSocket message:
  if br_avg > 0: lastKnownBR = br_avg
  if br_avg == 0 && br_bpm > 0: lastKnownBR = br_bpm

Display:
  lastKnownBR > 0:
    Value: large number (#2e2d29 if 12–20 BPM, #946f32 amber outside normal)
    Sub-label: "BPM AVG"
  lastKnownBR == 0:
    Value: "--"
    Sub-label: "CALIBRATING"
```

### Subcarrier → Heatmap Mapping

```
subcarriers[0..63]
    │
    ▼ Group into 16 segments (4 subcarriers each)
segVals[16] = mean of subcarriers[s×4 .. s×4+3]

    │
    ▼ Normalize to [0,1]

    │
    ▼ Angular spread (adjacent segment bleed at 0.6×)
segSpread[16]

    │
    ▼ Gaussian ring weighting (σ=2.5 rings, peak from RSSI)
htarget[ring][seg] = segSpread[seg] × gaussian(ring, peakRing, 2.5) × presMulti

presMulti: EMPTY=0.15, SINGLE=1.0, MULTI=1.2
```

---

## Tunable Parameters — Cheat Sheet

| Parameter | File | Current Value | Effect of Increasing |
|-----------|------|---------------|----------------------|
| `WELFORD_ALPHA_FAST` | dsp_processor.c | 0.50f | Extremely fast variance tracking, highly sensitive to immediate movement |
| `WELFORD_ALPHA_SLOW` | dsp_processor.c | 0.01f | Baseline baseline tracking speed |
| `EMPTY threshold` | dsp_processor.c | 1.05 | Harder to declare empty (more sensitivity) |
| `ratio hard cap` | dsp_processor.c | 20.0 | Higher cap allows finer MULTI distinction |
| `hold_frames` | dsp_processor.c | 15 | Longer hold before SINGLE→EMPTY |
| `PRESENCE_CONFIRM` | dsp_processor.h | 2 | More confirms = less flicker, slower response |
| `motion_clear threshold` | dsp_processor.c | 2.5 | Higher = BR attempts during more motion |
| `motion_clear_count gate` | dsp_processor.c | 10 | Fewer calm frames needed before BR attempt |
| `BR motion gate` | dsp_processor.c | 3.0 | Higher = BR attempts during more motion |
| `HAMPEL_THRESH` | dsp_processor.h | 3.0 | Higher = fewer points replaced (more outliers pass) |
| `BR_BUF_LEN` | dsp_processor.c | 256 | Larger = better freq resolution, longer warmup |
| `RSSI EMA α` | csi_handler.c | 0.1 | Higher = less smoothing, jumpier distance ring |
| `WARMUP_FRAMES` | dsp_processor.h | 80 | More warmup = more stable initial baseline |

---

## State Transition Diagram

```
              ratio ≥ 1.05 (×2 confirms)
     ┌───────────────────────────────────┐
     │                                   ▼
  EMPTY                              SINGLE ──────────────────────┐
     ▲                                   │                         │
     │  ratio < 1.05 (×2 confirms)      │  dual BR peaks          │
     │  after 15 hold frames expire     │  confirmed by FFT        │
     └───────────────────────────────────┘                         ▼
                                                                MULTI
                                                                   │
                                                  next get_output  │
                                                  (presence reset  │
                                                   to SINGLE base) │
                                                                   │
                                                    ▼ (re-evaluate each output call)
```

**MULTI is transient per output call** — it is set by `br_ready` dual-peak logic each time `dsp_processor_get_output()` runs. It is not persisted in the `detect_presence()` state machine. If the FFT buffer clears or motion resumes, the next output will revert to SINGLE.

---

## Known Limitations & Design Tradeoffs

| Limitation | Root Cause | Current Mitigation |
|------------|-----------|-------------------|
| Single receiver — no spatial localisation | 1 ESP32, no antenna array | RSSI-based range estimate (noisy) |
| BR requires ~25s warmup | Buffer must fill (256 × 100ms) + 10 calm frames | `last_known_bpm` persistence keeps display stable |
| MULTI only during stillness | FFT needs calm conditions | Architectural honesty — stated as reduced false positive |
| RSSI distance is ±1 ring | Multipath, body orientation | 10-sample EMA in csi_handler.c |
| Same-BPM two-person detection fails | Peaks < 0.05 Hz apart merge | No fix possible with 1 receiver |
| Subcarrier 0–5, 58–63 excluded | DC/guard band — unreliable | Hard-coded range [6,57] in variance compute |
| No mutex on ring buffer | Core 0 writes, Core 1 reads | Single-producer single-consumer pattern + volatile indices make it safe without mutex on Xtensa LX7 (write index updated only after slot written) |

---

## Build & Flash

```bash
# Set up ESP-IDF environment (once)
. $IDF_PATH/export.sh

# From csi-monitor/
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# Serial output:
# PRESENCE:SINGLE PEOPLE:1 MOTION:1.47 BR:15 bpm
```

**To update the embedded dashboard:** Edit `monitor.html` inside `csi-monitor/`, then rebuild and reflash. The HTML is baked into the firmware at compile time via `EMBED_TXTFILES`.

**Browser access:** After flash, note the IP from serial monitor, open `http://<IP>/` in any browser. The WebSocket connects automatically to `ws://<IP>/ws`.

---

## Glossary

| Term | Definition |
|------|-----------|
| CSI | Channel State Information — per-subcarrier complex amplitude/phase of a received WiFi packet |
| LLTF | Legacy Long Training Field — preamble field used for CSI extraction at 52 subcarriers |
| EMA | Exponential Moving Average — `y = α×x + (1−α)×y_prev` |
| Welford | Online algorithm for computing running mean + variance in one pass |
| Hampel | Robust outlier detection: points > k×MAD from sliding-window median are replaced |
| MAD | Median Absolute Deviation — robust scatter estimate, multiplied by 1.4826 to estimate σ |
| DFT | Discrete Fourier Transform — here computed per-bin, not via FFT |
| SNR | Signal-to-Noise Ratio |
| TC | Time Constant — frames required for EMA to reach 63% of a step change |
| BPM | Beats (breaths) Per Minute |
| STA | WiFi Station mode (client, not AP) |
| RSSI | Received Signal Strength Indicator, in dBm |
| rAF | requestAnimationFrame — browser's 60fps render loop hook |
