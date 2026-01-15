# Room Presence Detection

* [中文版](./README_cn.md)

This example demonstrates how to use 4 XIAO ESP32 devices to detect human presence and movement in a room using Wi-Fi CSI (Channel State Information) technology.

## Features

- **Multi-link CSI Detection**: Uses 3 CSI links for improved accuracy and coverage
- **Presence Detection**: Detects if someone is in the room (even when standing still)
- **Movement Detection**: Detects if someone is moving
- **Web Interface**: Real-time status display via WiFi hotspot
- **Calibration**: One-click calibration for environment adaptation
- **LED Indicators**: Visual status on each device

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        Room                             │
│                                                         │
│   [TX Sender]  ────CSI────→  [RX1 Master]               │
│       │                         ↑  ↑                    │
│       │                    ESP-NOW │                    │
│       └───CSI──→ [RX2 Slave 1]─────┘ │                  │
│       │                              │                  │
│       └───CSI──→ [RX3 Slave 2]───────┘                  │
│                                                         │
└─────────────────────────────────────────────────────────┘
                         │
                         ↓ WiFi AP
                   [Phone/PC Browser]
                   http://192.168.4.1
```

## Hardware Required

- 4x XIAO ESP32 devices (ESP32C3, ESP32C6, or ESP32S3)
- USB cables for flashing
- (Optional) External antennas for better range

## Recommended Placement

For best coverage in a small room:
- **TX (Sender)**: Place in one corner of the room
- **RX1 (Master)**: Place at the opposite corner
- **RX2 (Slave 1)**: Place at the midpoint of one wall
- **RX3 (Slave 2)**: Place at the midpoint of another wall

This creates a triangular coverage pattern that maximizes detection accuracy.

## Quick Start

### 1. Flash the Firmware

Flash each device with its corresponding firmware:

```bash
# TX - Sender (any corner)
cd send
idf.py set-target esp32c3  # or esp32c6, esp32s3
idf.py flash -p /dev/ttyUSB0

# RX1 - Master Receiver (opposite corner)
cd ../recv_master
idf.py set-target esp32c3
idf.py flash -p /dev/ttyUSB1

# RX2 - Slave Receiver 1 (wall midpoint)
cd ../recv_slave
idf.py set-target esp32c3
idf.py flash -p /dev/ttyUSB2

# RX3 - Slave Receiver 2 (another wall midpoint)
# First, set node_id to 2 in NVS, or modify the code
idf.py flash -p /dev/ttyUSB3
```

### 2. Power Up All Devices

Connect all 4 devices to power. They will automatically begin communicating.

### 3. Access the Web Interface

1. On your phone or computer, connect to WiFi network `RoomSensor` (password: `12345678`)
2. Open a browser and go to `http://192.168.4.1`
3. You'll see the real-time detection status

### 4. Calibrate the System

For best accuracy:
1. Make sure the room is empty
2. Click "Start Calibration" on the web interface
3. Wait about 10 seconds
4. Click "Stop Calibration"

The system will learn the baseline signal patterns and set optimal thresholds.

## Web Interface Features

- **Main Status Display**: Shows Empty (gray), Presence (blue), or Movement (green)
- **Link Status Cards**: Shows each sensor link's status and metrics
- **History Chart**: Real-time graph of detection metrics
- **Calibration Controls**: Start/stop calibration with threshold display

## LED Status Indicators

Each receiver device shows status via LED:
- **Off**: Room is empty
- **White**: Someone present (but not moving)
- **Green**: Movement detected
- **Yellow Blinking**: Calibrating

## Detection Algorithm

The system uses the `esp-radar` component which calculates two key metrics:

- **Wander**: Indicates presence (breathing, small movements)
- **Jitter**: Indicates motion (walking, arm movements)

The master node fuses results from all 3 links using a voting mechanism:
- If ≥2 links detect presence → room occupied
- If ≥2 links detect movement → movement confirmed

This multi-link approach reduces false positives and improves reliability.

## Project Structure

```
room_presence_detection/
├── send/                 # Transmitter firmware
│   └── main/app_main.c   # ESP-NOW packet broadcaster
├── recv_master/          # Master receiver firmware
│   └── main/
│       ├── app_main.c    # Detection + Web server
│       └── web/          # Web interface files
├── recv_slave/           # Slave receiver firmware
│   └── main/app_main.c   # Detection + ESP-NOW reporting
├── README.md             # This file
└── README_cn.md          # Chinese documentation
```

## Configuration

Key configuration options in the code:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CONFIG_WIFI_CHANNEL` | 11 | WiFi channel for all devices |
| `CONFIG_SEND_FREQUENCY` | 100 Hz | Packet transmission rate |
| `CONFIG_AP_SSID` | "RoomSensor" | WiFi hotspot name |
| `CONFIG_AP_PASSWORD` | "12345678" | WiFi hotspot password |
| `wander_threshold` | 0.0 (calibrated) | Presence detection threshold |
| `jitter_threshold` | 0.0003 | Movement detection threshold |

## Troubleshooting

### No data on web interface
- Check that all devices are powered on
- Ensure sender is transmitting (check serial log)
- Verify all devices are on the same WiFi channel

### High false positive rate
- Run calibration with empty room
- Increase distance between sender and receivers
- Move devices away from metal objects

### Weak detection
- Reduce distance between sender and receivers
- Use external antennas
- Check for WiFi interference from other networks

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [ESP-Radar Component](https://components.espressif.com/components/espressif/esp-radar)
- [Wi-Fi CSI Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-channel-state-information)

## License

Apache-2.0
