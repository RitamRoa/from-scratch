# Add WebSocket Server to ESP32 Firmware

This plan outlines the steps to add a WebSocket server to the ESP32-S3 firmware, allowing it to broadcast real-time CSI metrics (presence, motion, breathing rate, and subcarrier data) directly to `heatmap3d.html`.

## User Review Required

> [!IMPORTANT]
> Adding an HTTP/WebSocket server requires enabling the HTTPD WebSocket support in the ESP-IDF configuration. The build will take a moment as it recompiles the newly added HTTPD component.

## Open Questions

- None at the moment. The plan covers all necessary changes.

## Proposed Changes

### Configuration
#### [MODIFY] [sdkconfig](file:///c:/Users/Ritham/Desktop/from-scratch/csi-monitor/sdkconfig)
- Append `CONFIG_HTTPD_WS_SUPPORT=y` to enable WebSocket support for the ESP-IDF HTTP server component.

### Firmware (C Code)

#### [MODIFY] [csi_handler.h](file:///c:/Users/Ritham/Desktop/from-scratch/csi-monitor/main/csi_handler.h)
- Add an `int8_t rssi` field to `csi_frame_t` so that RSSI is captured and available for the WebSocket JSON payload.

#### [MODIFY] [csi_handler.c](file:///c:/Users/Ritham/Desktop/from-scratch/csi-monitor/main/csi_handler.c)
- In the `csi_callback`, extract the `rssi` from `wifi_csi_info_t->rx_ctrl.rssi` and save it to the frame before pushing it to the ring buffer.

#### [MODIFY] [main.c](file:///c:/Users/Ritham/Desktop/from-scratch/csi-monitor/main/main.c)
- Include `<esp_http_server.h>`.
- Add an HTTP server initialization function `start_webserver()` that starts an HTTPD instance and registers a `/ws` endpoint (`is_websocket = true`).
- Maintain a global `httpd_handle_t server = NULL;` variable.
- In `dsp_task`, after processing a frame (e.g., every 10 frames), format the output into a JSON string using `snprintf`.
- Send the JSON string to all connected WebSocket clients using `httpd_get_client_list` and `httpd_ws_send_frame_async` (or store the connected socket descriptor dynamically).

## Verification Plan

### Automated Tests
- Build the project using `idf.py build` to ensure the HTTPD component compiles and links correctly without errors.

### Manual Verification
- Flash the updated firmware using `idf.py -p COM7 flash monitor`.
- Open `heatmap3d.html` in the browser and verify that it successfully connects to the ESP32 via WebSocket and displays live metrics.
