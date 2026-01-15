# Room Presence Detection

* [中文版](./README_cn.md)

This example demonstrates how to use 4 XIAO ESP32 devices to detect human presence and movement in a room using Wi-Fi CSI (Channel State Information) technology.

## Features

- **Multi-link CSI Detection**: 3 independent CSI links for improved accuracy and coverage
- **Presence Detection**: Detects stationary humans (breathing, slight movements)
- **Movement Detection**: Detects active movement (walking, gestures)
- **Web Interface**: Real-time monitoring via built-in WiFi hotspot
- **Per-Link Sensitivity**: Each sensor link can be individually tuned
- **Auto Calibration**: 30-second calibration learns baseline environment
- **Persistent Settings**: Calibration and sensitivity saved across power cycles
- **LED Indicators**: Visual status feedback on each device

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                          Room                                │
│                                                              │
│   [TX Sender]  ────── CSI ──────→  [RX1 Master]              │
│       │                               ↑   ↑                  │
│       │                          ESP-NOW  │                  │
│       └────── CSI ──→ [RX2 Slave 1] ──┘   │                  │
│       │                                    │                  │
│       └────── CSI ──→ [RX3 Slave 2] ──────┘                  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
                         │
                         ↓ WiFi AP (192.168.4.1)
                   [Phone/PC Browser]
```

## Hardware Required

- 4× XIAO ESP32 devices (ESP32-C3, ESP32-C5, ESP32-C6, or ESP32-S3)
- USB cables for flashing
- Power supply for each device (USB power bank or adapter)
- (Optional) External antennas for better range

### Recommended Device Assignment

| Role | Recommended Device | Reason |
|------|-------------------|--------|
| TX (Sender) | ESP32-C3 | Simple, low power, stable |
| RX1 (Master) | ESP32-S3 | More RAM for web server |
| RX2 (Slave 1) | ESP32-C3/C5/C6 | Any CSI-capable device |
| RX3 (Slave 2) | ESP32-C3/C5/C6 | Any CSI-capable device |

## Project Structure

```
room_presence_detection/
├── send_TX/                    # Transmitter firmware
│   └── main/app_main.c         # ESP-NOW packet broadcaster
├── recv_master_RX1/            # Master receiver firmware  
│   └── main/
│       ├── app_main.c          # Detection + Web server
│       └── web/                # Web interface files
│           ├── index.html
│           ├── style.css
│           └── app.js
├── recv_slave/                 # Slave receiver firmware
│   └── main/app_main.c         # Detection + ESP-NOW reporting
├── README.md                   # This file
└── README_cn.md                # Chinese documentation
```

## Quick Start

### Step 1: Configure Slave Node IDs

Before flashing slaves, **you must set a unique node_id** for each slave device.

Edit `recv_slave/main/app_main.c`, find line ~106:

```c
#ifndef CONFIG_SLAVE_NODE_ID
#define CONFIG_SLAVE_NODE_ID 1    // Change to 1 for RX2, 2 for RX3
#endif
```

**Important**: 
- RX2 (first slave) should have `CONFIG_SLAVE_NODE_ID = 1`
- RX3 (second slave) should have `CONFIG_SLAVE_NODE_ID = 2`

Alternatively, set via `idf.py menuconfig` or compile with:
```bash
idf.py build -D CONFIG_SLAVE_NODE_ID=2
```

### Step 2: Flash All Devices

```bash
# 1. TX - Sender
cd send_TX
idf.py set-target esp32c3    # or esp32c5, esp32c6, esp32s3
idf.py build flash -p /dev/ttyUSB0

# 2. RX1 - Master Receiver
cd ../recv_master_RX1
idf.py set-target esp32s3    # recommended for web server
idf.py build flash -p /dev/ttyUSB1

# 3. RX2 - Slave Receiver 1 (node_id = 1)
cd ../recv_slave
# Edit app_main.c: set CONFIG_SLAVE_NODE_ID to 1
idf.py set-target esp32c5    # or esp32c3, esp32c6
idf.py build flash -p /dev/ttyUSB2

# 4. RX3 - Slave Receiver 2 (node_id = 2)
# Edit app_main.c: set CONFIG_SLAVE_NODE_ID to 2
idf.py build flash -p /dev/ttyUSB3
```

### Step 3: Physical Placement

For optimal coverage in a small room:

```
        Wall
    ┌───────────────────────┐
    │                       │
    │   [TX]          [RX1] │
    │                       │
Wall│        [RX2]          │ Wall
    │                       │
    │                       │
    │   [RX3]               │
    │                       │
    └───────────────────────┘
        Door/Wall
```

- **TX**: One corner of the room
- **RX1 (Master)**: Opposite corner (diagonal from TX)
- **RX2, RX3**: Other two corners or wall midpoints

This creates overlapping coverage zones for reliable detection.

### Step 4: Power Up and Connect

1. Power on all 4 devices
2. On your phone/PC, connect to WiFi: **`RoomSensor`** (password: **`12345678`**)
3. Open browser and navigate to: **`http://192.168.4.1`**

### Step 5: Calibrate the System

**Critical for accurate detection:**

1. **Empty the room completely** - no people, no moving objects
2. Click **"Start Calibration (30s)"** on the web interface
3. Wait 30 seconds (countdown shown)
4. Calibration auto-stops; thresholds are saved automatically

After calibration, all links should show "Clear" when the room is empty.

## Web Interface Guide

### Main Status Display

| Status | Icon Color | Meaning |
|--------|-----------|---------|
| Empty | Gray | No human detected |
| Presence | Blue | Someone present (stationary) |
| Motion | Green | Active movement detected |

### Sensor Link Cards

Each card shows:
- **Status**: Clear / Presence / Motion
- **Presence Value**: Wander metric (higher = more presence activity)
- **Motion Value**: Jitter metric (higher = more movement)
- **Sensitivity Sliders**: Adjust detection threshold per link

### Sensitivity Adjustment

Each link has two sensitivity sliders:

| Slider | What it controls |
|--------|-----------------|
| **Presence Sensitivity** | Lower = harder to detect presence |
| **Motion Sensitivity** | Lower = harder to detect motion |

**How to tune:**
1. After calibration, if a link incorrectly shows "Presence" when empty:
   - **Lower** that link's Presence Sensitivity
2. If a link fails to detect a person who is there:
   - **Raise** that link's Presence Sensitivity
3. Click **"Apply"** after adjusting

Settings are saved automatically and persist across reboots.

### Detection Logic

The master uses voting to determine final status:
- **Room Occupied**: ≥2 links detect presence or motion
- **Person Moving**: ≥2 links detect motion
- **Room Empty**: <2 links detecting anything

This multi-link voting reduces false positives from single-link noise.

## Configuration Options

### WiFi Settings (in `recv_master_RX1/main/app_main.c`)

```c
#define CONFIG_WIFI_CHANNEL     11          // Must match all devices
#define CONFIG_AP_SSID          "RoomSensor" // WiFi hotspot name
#define CONFIG_AP_PASSWORD      "12345678"   // WiFi password
```

### Sender Settings (in `send_TX/main/app_main.c`)

```c
#define CONFIG_WIFI_CHANNEL     11    // Must match receivers
#define CONFIG_SEND_FREQUENCY   100   // Packets per second (Hz)
```

### Detection Parameters

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| `wander_threshold` | Calibrated | ~0.0001 | Presence detection baseline |
| `jitter_threshold` | Calibrated | ~0.0003 | Motion detection baseline |
| `wander_sensitivity` | Web UI | 0.15 | Presence sensitivity multiplier |
| `jitter_sensitivity` | Web UI | 0.20 | Motion sensitivity multiplier |

## LED Status Indicators

Each RX device shows status via WS2812 LED:

| LED State | Meaning |
|-----------|---------|
| **Off** | Room empty |
| **White** | Someone present (not moving) |
| **Green** | Movement detected |
| **Yellow Blinking** | Calibrating |

## Troubleshooting

### Web Interface Shows "Initializing..."
- Ensure TX device is powered on and transmitting
- Check all devices are on the same WiFi channel (default: 11)
- Look for `CSI SEND` logs from TX device serial monitor

### Slave Links Not Active
- Verify slave node_id is unique (1 or 2)
- Check slaves are receiving CSI data (serial log shows `wander=` values)
- Ensure TX MAC address matches in all receiver code

### Always Shows "Presence" Even When Empty
1. **Recalibrate** with room completely empty
2. If still detecting, **lower the Presence Sensitivity** on affected links
3. Check for moving objects (fans, curtains, pets)

### Never Detects People
1. **Raise the Presence/Motion Sensitivity** on all links
2. Reduce distance between TX and RX devices
3. Ensure devices have line-of-sight paths through the room

### Detection is Inconsistent
- Some links may have poor signal paths; adjust their sensitivity individually
- Try repositioning devices for better coverage
- Use external antennas if available

### Device Stuck in Download Mode
If ESP32 won't boot after flashing:
1. Unplug USB, wait 3 seconds, replug
2. Press RESET button (not BOOT)
3. Run `idf.py erase-flash` then re-flash

## Advanced: Customizing Detection

### Adjusting the Algorithm

In `wifi_radar_cb()` function, detection uses:
```c
// Detection formula: signal * sensitivity > threshold
if (wander_average * wander_sensitivity > wander_threshold) {
    // Presence detected
}
```

- Higher `wander_sensitivity` → more sensitive (more detections)
- Lower `wander_sensitivity` → less sensitive (fewer false positives)

### Changing Voting Threshold

In `fuse_detection_results()`:
```c
// Currently requires ≥2 links to confirm detection
if (room_votes >= 2 || (room_votes >= 1 && active_links < 2)) {
    g_state.room_status = true;
}
```

Change `2` to `1` for single-link detection (more sensitive, more false positives).

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [ESP-Radar Component](https://components.espressif.com/components/espressif/esp-radar)
- [Wi-Fi CSI Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-channel-state-information)

## License

MIT
