# 房间人体检测示例

* [English](./README.md)

本示例演示如何使用 4 个 XIAO ESP32 设备，通过 Wi-Fi CSI（信道状态信息）技术检测房间内的人体存在和移动。

## 功能特点

- **多链路 CSI 检测**：使用 3 条 CSI 链路提高准确性和覆盖范围
- **存在检测**：检测房间内是否有人（即使站着不动）
- **移动检测**：检测是否有人在移动
- **Web 界面**：通过 WiFi 热点实时显示状态
- **校准功能**：一键校准适应环境
- **LED 指示灯**：每个设备的可视状态指示

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                        房间                             │
│                                                         │
│   [TX 发送端]  ────CSI────→  [RX1 主接收端]              │
│       │                         ↑  ↑                    │
│       │                    ESP-NOW │                    │
│       └───CSI──→ [RX2 从接收端1]────┘ │                 │
│       │                              │                  │
│       └───CSI──→ [RX3 从接收端2]─────┘                  │
│                                                         │
└─────────────────────────────────────────────────────────┘
                         │
                         ↓ WiFi AP热点
                   [手机/电脑浏览器]
                   http://192.168.4.1
```

## 硬件要求

- 4 个 XIAO ESP32 设备（ESP32C3、ESP32C6 或 ESP32S3）
- USB 数据线用于烧录
- （可选）外置天线以获得更好的覆盖范围

## 推荐摆放位置

为了在小房间内获得最佳覆盖：
- **TX（发送端）**：放在房间的一个角落
- **RX1（主接收端）**：放在对角位置
- **RX2（从接收端1）**：放在一面墙的中点
- **RX3（从接收端2）**：放在另一面墙的中点

这样形成三角形覆盖模式，最大化检测准确性。

## 快速开始

### 1. 烧录固件

为每个设备烧录对应的固件：

```bash
# TX - 发送端（任意角落）
cd send_TX
idf.py set-target esp32c3  # 或 esp32c6, esp32s3
idf.py flash -p /dev/ttyUSB0

# RX1 - 主接收端（对角位置）
cd ../recv_master_RX1
idf.py set-target esp32s3
idf.py flash -p /dev/ttyUSB1

# RX2 - 从接收端1（墙壁中点）
cd ../recv_slave
idf.py set-target esp32c5  #or esp32c3
idf.py flash -p /dev/ttyUSB2

# RX3 - 从接收端2（另一墙壁中点）
# 需要先在 NVS 中设置 node_id 为 2，或修改代码
idf.py flash -p /dev/ttyUSB3
```

### 2. 给所有设备上电

连接所有 4 个设备到电源，它们会自动开始通信。

### 3. 访问 Web 界面

1. 在手机或电脑上连接 WiFi 网络 `RoomSensor`（密码：`12345678`）
2. 打开浏览器访问 `http://192.168.4.1`
3. 您将看到实时检测状态

### 4. 校准系统

为获得最佳准确性：
1. 确保房间内无人
2. 在 Web 界面点击"Start Calibration"
3. 等待约 10 秒
4. 点击"Stop Calibration"

系统将学习基线信号模式并设置最佳阈值。

## Web 界面功能

- **主状态显示**：显示无人（灰色）、有人静止（蓝色）、有人移动（绿色）
- **链路状态卡片**：显示每条传感器链路的状态和指标
- **历史曲线图**：检测指标的实时图表
- **校准控制**：开始/停止校准，显示阈值

## LED 状态指示

每个接收端设备通过 LED 显示状态：
- **熄灭**：房间无人
- **白色**：有人存在（但未移动）
- **绿色**：检测到移动
- **黄色闪烁**：正在校准

## 检测算法

系统使用 `esp-radar` 组件计算两个关键指标：

- **Wander（漫游）**：指示存在（呼吸、微小动作）
- **Jitter（抖动）**：指示运动（行走、手臂移动）

主节点使用投票机制融合所有 3 条链路的结果：
- 如果 ≥2 条链路检测到存在 → 房间有人
- 如果 ≥2 条链路检测到移动 → 确认有人移动

这种多链路方法减少误报，提高可靠性。

## 项目结构

```
room_presence_detection/
├── send/                 # 发送端固件
│   └── main/app_main.c   # ESP-NOW 数据包广播器
├── recv_master/          # 主接收端固件
│   └── main/
│       ├── app_main.c    # 检测 + Web 服务器
│       └── web/          # Web 界面文件
├── recv_slave/           # 从接收端固件
│   └── main/app_main.c   # 检测 + ESP-NOW 上报
├── README.md             # 英文文档
└── README_cn.md          # 本文件
```

## 配置选项

代码中的关键配置选项：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CONFIG_WIFI_CHANNEL` | 11 | 所有设备的 WiFi 信道 |
| `CONFIG_SEND_FREQUENCY` | 100 Hz | 数据包发送频率 |
| `CONFIG_AP_SSID` | "RoomSensor" | WiFi 热点名称 |
| `CONFIG_AP_PASSWORD` | "12345678" | WiFi 热点密码 |
| `wander_threshold` | 0.0（校准后设置）| 存在检测阈值 |
| `jitter_threshold` | 0.0003 | 移动检测阈值 |

## 故障排除

### Web 界面无数据
- 检查所有设备是否已上电
- 确保发送端正在传输（查看串口日志）
- 验证所有设备在同一 WiFi 信道

### 误报率高
- 在空房间运行校准
- 增加发送端和接收端之间的距离
- 将设备远离金属物体

### 检测不灵敏
- 减少发送端和接收端之间的距离
- 使用外置天线
- 检查是否有其他网络的 WiFi 干扰

## 参考资料

- [ESP-IDF 文档](https://docs.espressif.com/projects/esp-idf/zh_CN/)
- [ESP-Radar 组件](https://components.espressif.com/components/espressif/esp-radar)
- [Wi-Fi CSI 指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-guides/wifi.html#wi-fi-channel-state-information)

## 许可证

Apache-2.0
