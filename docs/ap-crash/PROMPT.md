# 问题：ESP32-S3 开 AP 热点后，客户端一连接就断线/疑似重启

## 环境

- 板子：ESP32-S3-CAM（Hiwonder，GC2145 传感器）
- 框架：Arduino-ESP32 **3.3.11**
- FQBN：`esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB`
- 摄像头：PIXFORMAT_JPEG / FRAMESIZE_VGA / fb_count=3 / CAMERA_FB_IN_PSRAM
- PSRAM 8MB
- 问题在把 WiFi 从 STA 模式改成 AP 模式后出现

## 现象

1. 上电后 `wallbot-cam` 热点能正常广播，手机能搜到
2. 手机连上后几秒内 WiFi 断开，板子疑似重启
3. 改成开放热点（无密码）后现象不变
4. USB 串口完全无输出（一个字节都没有）；改回 STA 模式串口日志正常
5. 曾在串口抓到一次 panic：

```
Guru Meditation Error: Core 0 panic'ed (LoadProhibited)
Backtrace: 0x42038de2 0x4206d9c9 0x4206e68d 0x420708b5 0x42071c2a 0x4206f959 0x420b41c9 0x420afe12 0x4037e1ed

0x42038de2: wpa_ap_get_wpa_ie (esp_wpa_main.c:154)
0x4206d9c9: ieee80211_alloc_proberesp
0x4206e68d: ieee80211_send_proberesp
0x420708b5: hostap_recv_mgmt
0x42071c2a: hostap_input
0x4206f959: ap_rx_cb
0x420b41c9: ppRxPkt
0x420afe12: ppTask
```

另有两次不同 panic：`LoadStoreAlignment`、`StoreProhibited`。

## 已尝试无效

1. WPA2 改开放网络 `WiFi.softAP(ssid, NULL, ch, 0, max_conn)` —— 仍崩
2. `max_connection` 4 → 1 —— 仍崩
3. `WiFi.setSleep(false)` —— 仍崩

## 请回答

1. `wpa_ap_get_wpa_ie` 的 LoadProhibited 在开放网络下为什么还会走到 WPA IE 构造？是否 `WiFi.softAP()` 传 NULL 密码时 Arduino 封装仍按 WPA2 写入 config？
2. ESP32-S3 + Arduino-ESP32 3.3.11，AP 模式与摄像头（PSRAM/JPEG/DMA）是否有已知资源冲突或已知 issue？
3. 为什么 AP 模式下串口完全无输出？是 UART 被影响，还是 panic-reboot 循环但输出被吞？
4. AP 模式与摄像头能否共存？若不能，给出确定可用的替代架构（WIFI_AP_STA / 先 STA 后切 AP / 直接改用 esp_wifi 原生 API 等）。
5. 无串口输出的情况下如何定位（panic 落 flash/NVS、core dump 等）。

## 代码

完整源码见同目录 `ap_all.c`（camchild.ino / config.h / log_page.cpp / stream_server.cpp）。

重点看 `camchild.ino` 的 `wifi_connect()` AP 分支。`log_page.cpp` 的 `GET /` 控制页会发起 3 个 fetch 并挂载 `<img src="http://192.168.4.1:81/stream">`，但**问题在打开页面之前、仅连接热点时就已经出现**。
