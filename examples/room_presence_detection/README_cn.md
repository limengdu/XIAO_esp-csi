# 房间人体检测示例

* [English](./README.md)

本示例演示如何使用 4 个 XIAO ESP32 设备，通过 Wi-Fi CSI（信道状态信息）技术检测房间内的人体存在和移动。

## 功能特点

- **多链路 CSI 检测**：3 条独立 CSI 链路，提高准确性和覆盖范围
- **存在检测**：检测静止的人体（呼吸、轻微动作）
- **移动检测**：检测活动运动（行走、手势）
- **Web 界面**：通过内置 WiFi 热点实时监控
- **独立灵敏度**：每个传感器链路可单独调节
- **自动校准**：30 秒校准学习环境基线
- **设置持久化**：校准和灵敏度数据断电保存
- **LED 指示灯**：每个设备的可视状态反馈

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                          房间                                 │
│                                                              │
│   [TX 发送端]  ────── CSI ──────→  [RX1 主接收端]             │
│       │                               ↑   ↑                  │
│       │                          ESP-NOW  │                  │
│       └────── CSI ──→ [RX2 从接收端1] ──┘   │                 │
│       │                                    │                  │
│       └────── CSI ──→ [RX3 从接收端2] ──────┘                 │
│                                                              │
└──────────────────────────────────────────────────────────────┘
                         │
                         ↓ WiFi 热点 (192.168.4.1)
                   [手机/电脑浏览器]
```

## 硬件要求

- 4 个 XIAO ESP32 设备（ESP32-C3、ESP32-C5、ESP32-C6 或 ESP32-S3）
- USB 数据线用于烧录
- 每个设备的电源（USB 充电宝或电源适配器）
- （可选）外置天线以获得更好的覆盖范围

### 推荐设备分配

| 角色 | 推荐设备 | 原因 |
|------|---------|------|
| TX（发送端） | ESP32-C3 | 简单、低功耗、稳定 |
| RX1（主接收端） | ESP32-S3 | 更大内存，适合 Web 服务器 |
| RX2（从接收端1） | ESP32-C3/C5/C6 | 任何支持 CSI 的设备 |
| RX3（从接收端2） | ESP32-C3/C5/C6 | 任何支持 CSI 的设备 |

## 项目结构

```
room_presence_detection/
├── send_TX/                    # 发送端固件
│   └── main/app_main.c         # ESP-NOW 数据包广播器
├── recv_master_RX1/            # 主接收端固件
│   └── main/
│       ├── app_main.c          # 检测 + Web 服务器
│       └── web/                # Web 界面文件
│           ├── index.html
│           ├── style.css
│           └── app.js
├── recv_slave/                 # 从接收端固件
│   └── main/app_main.c         # 检测 + ESP-NOW 上报
├── README.md                   # 英文文档
└── README_cn.md                # 本文件
```

## 快速开始

### 第一步：配置从设备节点 ID

烧录从设备前，**必须为每个从设备设置唯一的 node_id**。

编辑 `recv_slave/main/app_main.c`，找到约第 106 行：

```c
#ifndef CONFIG_SLAVE_NODE_ID
#define CONFIG_SLAVE_NODE_ID 1    // RX2 改为 1，RX3 改为 2
#endif
```

**重要说明**：
- RX2（第一个从设备）应设置 `CONFIG_SLAVE_NODE_ID = 1`
- RX3（第二个从设备）应设置 `CONFIG_SLAVE_NODE_ID = 2`

也可以通过 `idf.py menuconfig` 设置，或编译时指定：
```bash
idf.py build -D CONFIG_SLAVE_NODE_ID=2
```

### 第二步：烧录所有设备

```bash
# 1. TX - 发送端
cd send_TX
idf.py set-target esp32c3    # 或 esp32c5, esp32c6, esp32s3
idf.py build flash -p /dev/ttyUSB0

# 2. RX1 - 主接收端
cd ../recv_master_RX1
idf.py set-target esp32s3    # 推荐用于 Web 服务器
idf.py build flash -p /dev/ttyUSB1

# 3. RX2 - 从接收端1（node_id = 1）
cd ../recv_slave
# 编辑 app_main.c：将 CONFIG_SLAVE_NODE_ID 设置为 1
idf.py set-target esp32c5    # 或 esp32c3, esp32c6
idf.py build flash -p /dev/ttyUSB2

# 4. RX3 - 从接收端2（node_id = 2）
# 编辑 app_main.c：将 CONFIG_SLAVE_NODE_ID 设置为 2
idf.py build flash -p /dev/ttyUSB3
```

### 第三步：设备摆放

为在小房间内获得最佳覆盖：

```
        墙壁
    ┌───────────────────────┐
    │                       │
    │   [TX]          [RX1] │
    │                       │
墙壁│        [RX2]          │ 墙壁
    │                       │
    │                       │
    │   [RX3]               │
    │                       │
    └───────────────────────┘
        门/墙壁
```

- **TX**：房间的一个角落
- **RX1（主接收端）**：对角位置（与 TX 对角）
- **RX2、RX3**：其他两个角落或墙壁中点

这样可创建重叠的覆盖区域，实现可靠检测。

### 第四步：上电并连接

1. 给所有 4 个设备上电
2. 在手机/电脑上连接 WiFi：**`RoomSensor`**（密码：**`12345678`**）
3. 打开浏览器访问：**`http://192.168.4.1`**

### 第五步：校准系统

**这对准确检测至关重要：**

1. **完全清空房间** —— 无人、无移动物体
2. 在 Web 界面点击 **"Start Calibration (30s)"**
3. 等待 30 秒（显示倒计时）
4. 校准自动停止；阈值自动保存

校准后，房间为空时所有链路应显示 "Clear"。

## Web 界面指南

### 主状态显示

| 状态 | 图标颜色 | 含义 |
|------|---------|------|
| Empty | 灰色 | 未检测到人 |
| Presence | 蓝色 | 有人存在（静止） |
| Motion | 绿色 | 检测到活动运动 |

### 传感器链路卡片

每个卡片显示：
- **Status**：Clear / Presence / Motion
- **Presence 值**：Wander 指标（越高 = 存在活动越多）
- **Motion 值**：Jitter 指标（越高 = 运动越多）
- **灵敏度滑块**：调整每个链路的检测阈值

### 灵敏度调节

每个链路有两个灵敏度滑块：

| 滑块 | 控制内容 |
|------|---------|
| **Presence Sensitivity** | 越低 = 越难检测到存在 |
| **Motion Sensitivity** | 越低 = 越难检测到运动 |

**如何调节：**
1. 校准后，如果某个链路在空房间时错误显示 "Presence"：
   - **降低** 该链路的 Presence Sensitivity
2. 如果某个链路无法检测到存在的人：
   - **提高** 该链路的 Presence Sensitivity
3. 调整后点击 **"Apply"**

设置会自动保存，断电重启后保持有效。

### 检测逻辑

主设备使用投票机制确定最终状态：
- **房间有人**：≥2 个链路检测到存在或运动
- **有人移动**：≥2 个链路检测到运动
- **房间无人**：<2 个链路检测到任何东西

这种多链路投票减少了单链路噪声导致的误报。

## 配置选项

### WiFi 设置（在 `recv_master_RX1/main/app_main.c` 中）

```c
#define CONFIG_WIFI_CHANNEL     11           // 必须与所有设备一致
#define CONFIG_AP_SSID          "RoomSensor" // WiFi 热点名称
#define CONFIG_AP_PASSWORD      "12345678"   // WiFi 密码
```

### 发送端设置（在 `send_TX/main/app_main.c` 中）

```c
#define CONFIG_WIFI_CHANNEL     11    // 必须与接收端一致
#define CONFIG_SEND_FREQUENCY   100   // 每秒发送数据包数 (Hz)
```

### 检测参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| `wander_threshold` | 校准设定 | ~0.0001 | 存在检测基线 |
| `jitter_threshold` | 校准设定 | ~0.0003 | 运动检测基线 |
| `wander_sensitivity` | Web 界面 | 0.15 | 存在灵敏度乘数 |
| `jitter_sensitivity` | Web 界面 | 0.20 | 运动灵敏度乘数 |

## LED 状态指示

每个 RX 设备通过 WS2812 LED 显示状态：

| LED 状态 | 含义 |
|---------|------|
| **熄灭** | 房间无人 |
| **白色** | 有人存在（未移动） |
| **绿色** | 检测到运动 |
| **黄色闪烁** | 正在校准 |

## 故障排除

### Web 界面显示 "Initializing..."
- 确保 TX 设备已上电并正在发送
- 检查所有设备是否在同一 WiFi 信道（默认：11）
- 查看 TX 设备串口日志是否有 `CSI SEND` 输出

### 从设备链路不活跃
- 验证从设备 node_id 是否唯一（1 或 2）
- 检查从设备是否接收到 CSI 数据（串口日志显示 `wander=` 值）
- 确保所有接收端代码中 TX MAC 地址一致

### 空房间时始终显示 "Presence"
1. **重新校准**，确保房间完全清空
2. 如果仍然检测到，**降低** 受影响链路的 Presence Sensitivity
3. 检查是否有移动物体（风扇、窗帘、宠物）

### 无法检测到人
1. **提高** 所有链路的 Presence/Motion Sensitivity
2. 减少 TX 和 RX 设备之间的距离
3. 确保设备在房间内有视线路径

### 检测不稳定
- 某些链路可能信号路径较差；单独调整其灵敏度
- 尝试重新定位设备以获得更好的覆盖
- 如有外置天线，请使用

### 设备卡在下载模式
如果 ESP32 烧录后无法启动：
1. 拔掉 USB，等待 3 秒，重新插入
2. 按 RESET 按钮（不是 BOOT）
3. 运行 `idf.py erase-flash` 然后重新烧录

## 进阶：自定义检测

### 调整算法

在 `wifi_radar_cb()` 函数中，检测使用：
```c
// 检测公式：信号 * 灵敏度 > 阈值
if (wander_average * wander_sensitivity > wander_threshold) {
    // 检测到存在
}
```

- 更高的 `wander_sensitivity` → 更灵敏（更多检测）
- 更低的 `wander_sensitivity` → 更不灵敏（更少误报）

### 修改投票阈值

在 `fuse_detection_results()` 中：
```c
// 当前需要 ≥2 个链路确认检测
if (room_votes >= 2 || (room_votes >= 1 && active_links < 2)) {
    g_state.room_status = true;
}
```

将 `2` 改为 `1` 可实现单链路检测（更灵敏，但更多误报）。

## 参考资料

- [ESP-IDF 文档](https://docs.espressif.com/projects/esp-idf/zh_CN/)
- [ESP-Radar 组件](https://components.espressif.com/components/espressif/esp-radar)
- [Wi-Fi CSI 指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-guides/wifi.html#wi-fi-channel-state-information)

## 许可证

MIT
