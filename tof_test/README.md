# VL53L5CX 8×8 ToF 测距测试固件

独立测试固件，用于论文中的激光（ToF）测距数据采集与可视化。**不修改 `firmware/` 下任何文件**，单独烧录。

## 硬件

- 主控：ESP32-S3（与主固件同一块）
- 传感器：VL53L5CX 8×8 ToF
- 多路复用：TCA9548A，地址 `0x70`，使用 **CH0**
- I²C：`SDA=GPIO4`，`SCL=GPIO5`（与主固件一致）

## 编译 / 烧录

```sh
cd tof_test
arduino-cli compile --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M,PartitionScheme=huge_app" .
arduino-cli upload -p /dev/ttyUSB0 --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M,PartitionScheme=huge_app" .
```

> 烧录会覆盖主固件。恢复主固件：进入 `firmware/` 用同法编译烧录。

## 使用

1. 上电，板子发出 WiFi 热点 **`robot`**（密码 `12345678`）。
2. 平板/电脑连接 `robot`。
3. 浏览器打开 **`http://192.168.4.1/tof`**。

## 端点

| 路径 | 说明 |
|------|------|
| `/tof` | 学术风动态热力图页面 |
| `/tof/matrix` | 8×8 距离/状态 JSON |

JSON 结构：

```json
{
  "online": true,
  "fps": 14.8,
  "frames": 1234,
  "front": 342,
  "d": [64 个距离 mm，0=无效],
  "s": [64 个 target_status]
}
```

## 页面功能

- **图 1**：8×8 动态热力图（viridis 色标，白框为前方扇形 ROI：行 0–3，列 1–6）
- **图 2**：前方 ROI 最小距离时间序列（最近 120 点）
- **表 1**：8×8 距离矩阵实时快照
- **导出**：CSV（当前矩阵）/ PNG（热力图），供论文配图

## 参数

| 项 | 值 |
|----|----|
| 分辨率 | 8×8 (64 区) |
| 采样频率 | 15 Hz |
| I²C 速率 | 400 kHz |
| 距离量程 | 0–4000 mm |
| 有效判定 | `target_status == 5` 且 `distance > 0` |
| ROI | 行 0–3 × 列 1–6（前方 ±15° 上半区）|

## 依赖库

- `Adafruit_VL53L5CX` 1.0.1
- `Adafruit_BusIO` 1.17.4
- ESP32 core 3.3.10