# 问题：ESP32-S3 摄像头子机在视觉纠偏检测期间，App 侧实时画面卡死

## 背景

爬壁机器人，三块 ESP32 板：

- 母机：ESP32-S3，16MB Flash，跑 WiFi WebSocket 控制 + NRF24 无线下发
- 摄像头母机：ESP32-S3 + OV3660，`:81/stream` MJPEG
- 摄像头子机：ESP32-S3 + GC2145，跑 AprilTag 36h11 检测 + 视觉纠偏测量（本问题的板子）

App 是 Capacitor + Android WebView 的 H5，通过 img 标签拉 MJPEG 画面（url 形如 http://<ip>:81/stream）。

## 需求

**视觉纠偏检测运行期间，`:81` 的实时画面必须继续正常推送到 App。** 不接受“检测时画面卡住/变小/断流后重连”。

## 当前实现（见 camchild_all.c）

### 端口与实例划分

stream_server.cpp 的 StreamServer_Start() 起**两个** httpd 实例：

- `:80` 控制实例（s_capture_httpd）：`/`、`/jpg`、`/fps`、`/log`、`/align/get`、`/align/ref`、`/align/status`、`/logs`、`/status`
- `:81` 流实例（s_stream_httpd）：`/stream`、`/view`

### 检测模式（cam_module.cpp）

相机初始化固定 PIXFORMAT_JPEG + FRAMESIZE_HD，fb_count=2，CAMERA_FB_IN_PSRAM，CAMERA_GRAB_LATEST。

历史教训：esp32-camera 的 DMA 解析模式在 esp_camera_init 时按 JPEG 定死，运行时 set_pixformat(PIXFORMAT_GRAYSCALE) 会产生坏帧。所以检测模式**只切 framesize 到 QVGA，保持 JPEG**，检测前用 fmt2rgb888 解码成灰度。

Cam_EnterDetectMode() / Cam_ExitDetectMode() 用引用计数 s_detect_refs（portMUX 保护），`/view` 与 `/align/*` 共用。

### 检测流程（align_server.cpp）

`/align/get` -> measure_consensus()：

1. 取 g_measure_lock（mutex，等 1500ms）
2. Cam_EnterDetectMode() -> 切 QVGA，Cam_FlushFrames(2,10)
3. 循环取 ALIGN_FRAMES(3) 帧（最多尝试 4 次），每帧 esp_camera_fb_get() -> AlignDecodeJpegToGray() -> TagDetector_DetectEx()
4. Cam_ExitDetectMode() -> 切回 HD
5. 一致性校验后返回 phi/psi/axial

`?once=1` 时只取 1 帧。

### 可视化流（align_view.cpp）

`:81/view` 是 while(true) 推流 handler：每帧 esp_camera_fb_get() -> 解码灰度 -> 叠加候选框/角点/4 行诊断文字 -> fmt2jpg 编码 -> chunk 推送。整个流期间持有检测模式（Cam_EnterDetectMode() 在 while 前，Exit 在流断开后）。

### 实测现象

1. App 打开纠偏面板后，align.js 每 1200ms 轮询 `/align/get`。
2. 无 tag 时该请求返回 `{"ok":false,"reason":"no_tag"}`，但在此期间 `:81/stream` 的画面**卡住不动**。
3. 有时画面恢复，有时 App 日志出现 `[发现] 缓存地址未连通, 重新扫描`，随后子机 IP 被清空，画面彻底不再回来。
4. 纠偏面板自身的数值仍在刷新（说明 `:80` 在响应）。

## 已排除/已尝试

- 已在 App 侧：camera.js 增加 detectMode 标志，检测期间首帧超时放宽到 20s 且不重连、retryCount 归零；align.js 开关面板时调 CameraModule.setDetectMode('child', true/false)；_httpJson 超时 8s 改为 3s。**问题仍在。**
- 已给 `:80` 的 HTTPD_DEFAULT_CONFIG() 设 max_uri_handlers = 16、stack_size = 16384。
- Cam_EnterDetectMode 的 Cam_FlushFrames 已从 6x30ms 降到 2x10ms。
- g_measure_lock 等待从 8000ms 降到 1500ms。
- `/view` 的 JPEG 解码已移入 g_measure_lock 内，与 `/align/*` 串行。

## 我的怀疑点

1. **`:81` 单实例单线程**：`/stream` 与 `/view` 都注册在 s_stream_httpd 上，两者都是 while(true) 推流 handler。esp_http_server 每个实例一个任务，第二个请求会阻塞到第一个流结束——这是否就是“纠偏时画面卡住”的直接原因？如果是，应该给 `/view` 单独起第三个 httpd 实例，还是干脆取消 `/view`、把叠加渲染并进 `/stream`？
2. **检测模式切 framesize 是否会让正在推流的 `/stream` 取到坏帧或长时间 esp_camera_fb_get() 阻塞**？fb_count=2、CAMERA_GRAB_LATEST 下，set_framesize 后驱动重新配置 DMA，正在跑的 esp_camera_fb_get() 会返回 NULL 还是阻塞？会不会永久拿不到帧？
3. **两个任务并发 esp_camera_fb_get()**（`:81/stream` 的推流任务 + `:80` 的 `/align/get` 检测任务）是否安全？esp32-camera 的 fb 队列在 fb_count=2 下，一边持续取帧一边被另一任务抢帧，是否会导致某一方长期饥饿？
4. **g_measure_lock 是否足够**？它保护的是 TagDetector 的全局静态工作区和 s_dec_gray 解码缓冲，但**没有**保护 esp_camera_fb_get() 本身。是否需要额外的相机访问互斥？
5. **TagDetector_DetectEx 在 QVGA 无 tag 时的耗时**是多少量级？如果单次几十到几百 ms 且每秒被轮询，是否会长期占住 CPU0/CPU1 导致流任务被饿死？

## 想要的具体答案

1. 在 esp32-camera + esp_http_server 的约束下，**“MJPEG 推流 + 周期性同步检测”共存**的正确架构是什么？（单实例多 handler / 多实例 / 单一任务里做流+定期插入检测 / 双缓冲帧队列）
2. set_framesize 运行时切换对正在推流的 esp_camera_fb_get() 到底有什么影响？给出驱动内部行为依据。
3. 是否应该**放弃运行时切 framesize**，改为固定 QVGA（或固定 VGA）让 `/stream` 与检测共用同一分辨率？代价是什么（带宽/检测精度）？
4. 给出**最小改动的具体代码方案**（函数级、可落到 camchild_all.c 的哪些函数），要求：纠偏轮询期间 `:81` 画面持续、不卡、不触发 App 重连。
5. 如果必须在检测精度与推流流畅间取舍，给出推荐参数（framesize、jpeg_quality、fb_count、grab_mode、检测帧数、轮询间隔）。

## 附件

- camchild_all.c：摄像头子机全部源码拼接（3000 行，含 AprilTag 36h11 码表）
- app_js_all.js：App 侧 camera.js + align.js + app.js（发现/地址流程节选）
