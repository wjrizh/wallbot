# 视觉纠偏代码地图（交接用）

本文只覆盖「视觉纠偏」这条链路。目的：接手的人不用拉全仓库就能定位该读哪些文件、改哪里。

---

## 1. 链路总览

```
[K230 CanMV]  --HTTP :80/align/*-->  [母机 ESP32-S3]  --NRF24-->  [子机 F103]
     |                                    ^
     |  :81/stream (MJPEG)                |  WS 0x22/0x23/0x24/0x25/0x26
     v                                    |
  [App WebView]  --HTTP GET /align/get--> K230（监视轮询，不经母机）
```

数据通道分工（重要，改代码别搞混）：

| 数据 | 通道 |
|---|---|
| 视觉测量值（φ/ψ/ax/JSON） | App ↔ K230 直连 HTTP |
| 子机动作（ADJ 帧、TSTA/TSTB/DONE 握手） | App → 母机 WS → NRF → 子机 |
| 闭环状态回传（0x88/0x89） | 母机 → App WS |
| 子机画面 | K230 `:81/stream` 直接给 App `<img>` |

---

## 2. 文件清单

### 2.1 K230 视觉板（MicroPython）

| 文件 | 职责 |
|---|---|
| `k230/dual_tag_align_web.py` | 全部板端逻辑：双码检测、偏差计算、HTTP 服务、MJPEG 推流、UDP 发现。单文件约 1685 行 |
| `k230/flash_k230.py` | 纯串口烧录脚本，把上面的 py 写到板端 `/sdcard/main.py` |

`dual_tag_align_web.py` 内部关键函数/常量：

| 符号 | 行号 | 说明 |
|---|---|---|
| `DUAL_TAG_SEP_MM` / `R_TAG_MM` / `PIPE_RADIUS_MM` | 31/33/34 | 几何常数，改标定先看这里 |
| `TAG_ID_A` / `TAG_ID_B` | 35/36 | 双码 ID |
| `ALIGN_ANGLE_TOL_DEG` / `ALIGN_AXIAL_TOL_MM` / `ALIGN_FRAMES` | 37-39 | K230 侧收敛判据与共识帧数 |
| `WIFI_MODE` / `WIFI_SSID` / `STA_STATIC_IP` | 44-50 | 默认 STA 连平板热点 `robot` |
| `DISCOVERY_PORT` / `DEVICE_ROLE` | 68/72 | UDP 48899，role=`camchild` |
| `PREVIEW_INTERVAL_MS` / `PREVIEW_DETECT_INTERVAL_MS` | 80/83 | 预览 100ms，检码节流 300ms |
| `STREAM_DENY_AFTER_STOP_MS` | 86 | `/align/stop` 后 8s 拒绝拉流自动开采 |
| `delta_report()` | 137 | **偏差核心**：由双码位姿差算出 φ/ψ/ax/arc |
| `fuse_dual_obs()` | 184 | 两码融合成单一位姿 |
| `align_json_ok()` / `align_json_fail()` | 272/290 | `/align/get`、`/align/once` 的 JSON 格式 |
| `push_consensus()` | 301 | 滑窗平均，`ALIGN_FRAMES` 帧才出结果 |
| `set_ref_from_obs()` / `clear_ref()` | 336/349 | 记/清参考位 |
| `camera_pause()` / `camera_resume()` | 724/735 | 软停/恢复，不拆 MediaManager |
| `vision_start()` / `vision_stop()` | 788/826 | 开/关采集 |
| `img_for_detect()` | 855 | 彩色转灰度 |
| `capture_preview()` | 940 | **预览帧**：节流检码 + 叠图 + JPEG |
| `capture_detect_update()` | 1004 | **测量帧**：全量检码 + 写 `g_last_dev/g_last_json` |
| `sync_consensus()` | 1067 | 累计 N 帧共识，`/align/ref`、`/align/once` 用 |
| `handle_control_client()` | 1330 | HTTP 路由分发 |

HTTP 接口：

| 路径 | 作用 |
|---|---|
| `/status` | 纯文本状态，含 ref、last 偏差、cfg |
| `/align/start` `/align/stop` | 开/关采集 |
| `/align/ref` | 记参考；`?clear=1` 清参考 |
| `/align/once` | 触发一次共识测量（闭环每轮调这个） |
| `/align/get` | 返回 `g_last_json` 缓存（监视轮询读这个） |
| `:81/stream` | MJPEG |
| `:80/jpg` | 单帧 |

### 2.2 母机固件（ESP32-S3，Arduino）

| 文件 | 职责 |
|---|---|
| `firmware/align_vision.cpp/.h` | **闭环状态机核心**，约 639 行 |
| `firmware/test_mode.cpp/.h` | TSTA/TSTB/ADJ/DONE 握手，NRF 收发时序 |
| `firmware/web_server.cpp/.h` | WS 服务 + 0x88/0x89 推送 + UDP 发现应答 |
| `firmware/protocol.cpp` | 帧编解码，0x22-0x26 分发入口 |
| `firmware/config.h` | **所有闭环常量**（第 150-180 行区间） |
| `firmware/calibration_test.ino` | 主 sketch，`AlignVision_Init()` + 建任务 |
| `firmware/child_cmd.cpp/.h` | 子机指令队列（被 protocol/joypad_sender/auto_mode 用） |

`align_vision.cpp` 关键位置：

| 符号 | 行号 | 说明 |
|---|---|---|
| `s_state` 及 `AV_ST_*` | 32 / `.h:24-37` | 状态机 |
| `loadTune()` / `saveTune()` | 76/94 | 符号与速度标定，NVS 持久化 |
| `httpGet()` | 154 | 拉 K230 |
| `finish()` | 190 | 统一收尾；此处按 flags 决定是否打 `/align/stop` |
| `handleMeasureFail()` | 212 | 测量失败分类重试 |
| `sendAdjRound()` | 241 | **下发单轮 ADJ**：容差判定、增益、限幅、最小命令抬升 |
| `avStep()` | 316 | 状态机主循环，50ms 一拍 |
| `AV_ST_MEASURE` 分支 | 373-435 | 每轮 `GET /align/once` + 解析 + 判收敛 |
| `AV_ST_WAIT_DONE` 分支 | 436-465 | 等子机 DONE，轮次++，安全上限检查 |
| `AlignVision_Start()` | 486 | App 0x22 入口，参数钳位 |
| `AlignVision_CamCtl()` | 571 | App 0x26 入口，直通 K230 `/align/*` |
| `AlignVision_GetStatus()` | 602 | 填 0x88 的 11 字节 payload |
| `AlignVision_Task()` | 622 | FreeRTOS 任务体 |

`config.h` 闭环常量（约 150-180 行）：

```c
AV_POLL_MS            500      // 两次 /align/once 最小间隔
AV_MAX_ITER           30       // App 不传时的默认轮次上限
AV_SAFETY_MAX_ITER    50       // 绝对上限，防死循环
AV_TOTAL_TIMEOUT_MS   600000   // 闭环总时限 10min
AV_HTTP_FAIL_MAX      3
AV_ALIGNED_HITS       2        // aligned 连击判收敛
AV_CAM_START_WAIT_MS  12000
AV_TEST_READY_WAIT_MS 8000
AV_DONE_TIMEOUT_MS    70000
AV_CMD_MAX_PHI/PSI/AX 10.0/10.0/30.0   // 单轮命令限幅
AV_RAW_TOL_PHI/PSI/AX 2.0/2.0/10.0     // 原始偏差收敛兜底
AV_CMD_MIN_PHI/PSI/AX 0.8/0.8/3.0      // 占位值，待实机标定
AV_DEFAULT_GAIN_PCT   30
```

### 2.3 子机 F103（执行端）

| 文件 | 职责 |
|---|---|
| `firmware/pabi_F103rct6/Core/Src/main.c` | 1840 行，`Adj_Execute()` 在 **1298 行** |

`main.c` 里纠偏相关常量（**改标定必改这里**）：

```c
ADJ_MM_PER_MOTORREV  0.3555f   // 964 行，里程计 mm/电机轴转
ADJ_TRACK_MM         145.0f    // 965 行，轮距
ADJ_PIPE_R_MM        250.0f    // 966 行，管道半径
```

执行入口：`Adj_Execute(phi_deg, psi_deg, ax_mm)`（1298 行），内部拆成
「斜行（轴向+周向融合）→ 自旋（航向）→ 弧长直行」三段，见 1317 行起。

### 2.4 App（`app/www/`）

| 文件 | 职责 |
|---|---|
| `js/modules/align.js` | **纠偏面板全部逻辑**，839 行 |
| `js/modules/camera.js` | 双路 MJPEG，子机通道走 K230 `:81/stream` |
| `js/protocol.js` | 帧编解码，0x22-0x26 / 0x86-0x89 定义在 27-39 行 |
| `js/transport.js` | WS 连接与消息分发（第 90-108 行） |
| `js/app.js` | 自动发现、地址应用、摄像头失效兜底 |
| `index.html` | 面板 DOM 结构；`align.js?v=N` 在第 227 行 |

`align.js` 关键位置：

| 符号 | 行号 | 说明 |
|---|---|---|
| `ADJ_PIPE_R_MM` / `ADJ_TRACK_MM` | 22/23 | 250/145，用于虚拟迭代扣减 |
| `_iter` / `_mon` 状态 | 57/60 | 虚拟迭代、监视轮询 |
| `_build()` | 106 | 整个面板 DOM，⑤ 区在第 226 行起 |
| `_resolveVisionIp()` | 90 | 取 `cam.getIp('child')`，回退 localStorage |
| `_startVision()` | 503 | 发 0x22，**flags=0x02**（517-519 行，只开不关） |
| `_camRef()` | 534 | 发 0x26 action=3，随后自动开监视 |
| `_startMonitor()` / `_stopMonitor()` / `_pollOnce()` | 573/584/591 | App 直连 K230 `/align/get` 轮询，300ms |
| `_onAlignState()` | 642 | 收 0x88，刷状态行与偏差；终态恢复监视 |
| `_onAlignJson()` | 665 | 收 0x89，刷 JSON 原文 |
| `_applyStageResult()` / `_nextIter()` | 386/406 | 虚拟纠偏的迭代扣减（走 0x30/0x31 那条老路） |

### 2.5 遗留 / 死代码（接手后可忽略或清理）

| 路径 | 状态 |
|---|---|
| `firmware/align_bridge.cpp/.h` | **不参与编译**（`.ino` 没 include）。旧的速度闭环方案，已被 `align_vision` 取代。可删 |
| `firmware/child_cmd.cpp/.h` | **仍在用**（`protocol.cpp:265`、`joypad_sender.cpp:93`、`auto_mode.cpp:104`）。不要删 |
| `摄像头子机/`（ESP32-S3-CAM 版，10 个 .cpp/.h） | **已完全被 K230 取代**，固件无任何引用。可整目录删 |
| `摄像头母机/` | 仍存在，但与本纠偏链路无关（那是另一块 ESP32 摄像头板） |
| `app/www/js/modules/coop.js` / `joystick.js` | 子机协同/摇杆，与视觉纠偏无直接关系 |

---

## 3. 三条工作流的时序

### 3.1 记参考

```
App _camRef() → WS 0x26 action=3 → 母机 AlignVision_CamCtl()
  → HTTP GET http://<k230>/align/ref
  → K230 sync_consensus() 累计 ALIGN_FRAMES 帧 → set_ref_from_obs()
  → 返回 {"ok":true,"ref":true,"u":..,"v":..,"theta_deg":..}
App 1.5s 后自动 _startMonitor()
```

### 3.2 监视（人手推车）

```
App _pollOnce() 每 300ms → HTTP GET http://<k230>/align/get
  → K230 返回 g_last_json
  ↑ 该值由主循环 capture_preview() 每 PREVIEW_DETECT_INTERVAL_MS 检码后刷新
App 解析 phi_deg/psi_deg/axial_mm/aligned → 刷偏差行 + 姿态动画
```

闭环期间必须停监视，否则与母机 `/align/once` 抢 K230（单线程）。

### 3.3 闭环

```
App _startVision() → WS 0x22(ip,gain,maxIter,flags=0x02)
母机 AlignVision_Start() → AV_ST_CAM_START
  → HTTP /align/start
  → TestMode_Start() 发 TSTA
  → 等 TSTB（子机就绪，TestMode 状态=3）
  → AV_ST_MEASURE: HTTP /align/once（每轮重新测，不读缓存）
       ├─ aligned 连击 ≥ AV_ALIGNED_HITS → 收敛结束
       └─ 否则 sendAdjRound()：容差判定 → 增益 → 限幅 → 最小抬升
  → TestMode_SendAdj() → NRF → 子机 Adj_Execute()
  → AV_ST_WAIT_DONE: 等轮次变化
       ├─ 轮次 < 安全上限 → 回 AV_ST_MEASURE 再来一轮
       └─ ≥ AV_SAFETY_MAX_ITER → 超时结束
```

每轮母机把 0x88（状态/轮次/偏差）和 0x89（原始 JSON）推给 App。

---

## 4. 协议速查

App ↔ 母机（WS，帧头 `AA 55 cmd len payload crc16`）：

| cmd | 方向 | payload |
|---|---|---|
| `0x22` ALIGN_VISION_START | App→母机 | `ip[4] gain maxIter flags` |
| `0x23` ALIGN_VISION_STOP | App→母机 | 无 |
| `0x24` ALIGN_TUNE | App→母机 | `signPhi signPsi signAx betaDeg speedPct`（有符号字节） |
| `0x25` ALIGN_STEP | App→母机 | `which + float32 phi/psi/ax` |
| `0x26` ALIGN_CAM_CTL | App→母机 | `action + ip[4]` |
| `0x30/0x31/0x32` | App→母机 | 虚拟纠偏 TEST_START/ADJ/STOP |
| `0x86/0x87` | 母机→App | 轮子遥测 / 测试状态 |
| `0x88` | 母机→App | 11 字节，见下 |
| `0x89` | 母机→App | K230 原始 JSON 文本（≤200B） |

`0x88` payload 布局（`AlignVision_GetStatus()`，**以此为准**）：

```
[0] state   [1] iter   [2] rc
[3..4] phi  int16 小端 ×100（单位 0.01°）
[5..6] psi  int16 小端 ×100
[7..8] ax   int16 小端（mm）
[9] aligned  [10] maxIter
```

注意：`align_vision.h:103` 的注释写 `[7] reason`，**已过时**，实际是 `ax` 高字节。以代码为准。

`0x26` action：0=status 1=start 2=stop 3=ref 4=once 5=clearRef（`align_vision.h:69-76`）。

---

## 5. 跨文件必须一致的不变量

改任何一个，其余几处必须同步，否则迭代扣减/偏差换算会错。

| 物理量 | 出现位置 |
|---|---|
| 管道半径 250mm | `k230/dual_tag_align_web.py:34`、`firmware/pabi_F103rct6/Core/Src/main.c:966`、`app/www/js/modules/align.js:22` |
| 轮距 145mm | `main.c:965`、`align.js:23` |
| 双码间距 40mm | `k230/dual_tag_align_web.py:31`（K230 独有） |
| 码真实尺寸 22mm | `k230/dual_tag_align_web.py:32`（K230 独有） |
| 收敛容差 | K230 `ALIGN_ANGLE_TOL_DEG=4.0` / `ALIGN_AXIAL_TOL_MM=20.0`；母机 `AV_RAW_TOL_*=2/2/10`（取 K230 的一半作兜底）。**收紧精度时两处要一起降** |

---

## 6. 常见改动场景

| 想改什么 | 改哪里 |
|---|---|
| 偏差算法（φ/ψ/ax 怎么算） | `k230/dual_tag_align_web.py:137 delta_report()` |
| 收敛判据 | K230 `:37-38`；母机 `config.h` 的 `AV_RAW_TOL_*` / `AV_ALIGNED_HITS` |
| 每轮下发多少 | `firmware/align_vision.cpp:241 sendAdjRound()` |
| 子机怎么走（三段式轨迹） | `main.c:1298 Adj_Execute()` |
| 符号反了 | App 面板「写入标定」→ 0x24 → NVS；不用重烧 |
| 速度 | `ALIGN_TUNE` 的 speedPct；子机 `ADJ_SPEED_REV` |
| 轮次上限 | App 输入框（1..50）→ 0x22 maxIter；`config.h: AV_MAX_ITER` 是默认值 |
| 面板 UI | `app/www/js/modules/align.js:106 _build()` |
| 监视轮询频率 | `align.js _pollOnce()` 尾部 `setTimeout(..., 300)` |
| 预览检码频率 | `dual_tag_align_web.py:83 PREVIEW_DETECT_INTERVAL_MS` |

---

## 7. 构建与烧录

三块板 + App，改哪个烧哪个。

**K230**（改 `k230/*.py` 时）：

```sh
cd /home/z/wallbot-repo
/home/z/toolchain/venv-mpy/bin/python3 k230/flash_k230.py
```

板子靠 `/dev/serial/by-id/*Kendryte*` 识别，不会误烧母机。

**母机固件**（改 `firmware/*` 时）：

```sh
mkdir -p /tmp/wbfw/wallbot_fw && cp firmware/*.cpp firmware/*.h /tmp/wbfw/wallbot_fw/ \
  && cp firmware/calibration_test.ino /tmp/wbfw/wallbot_fw/wallbot_fw.ino
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --output-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
arduino-cli upload -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_* \
  --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```

母机是 `/dev/ttyACM1`，K230 是 `/dev/ttyACM0`，两者 by-id 前缀不同。

**子机 F103**：`firmware/pabi_F103rct6/`，用 STM32CubeIDE 或 CMake 构建，走 ST-Link 烧。

**APK**（改 `app/www/*` 时）：

```sh
cd app && npx cap sync android
cd android && export JAVA_HOME=/home/z/toolchain/jdk-17.0.20.1+1 && ./gradlew assembleDebug
```

产物 `app/android/app/build/outputs/apk/debug/app-debug.apk`。

改 `app/www/js/*.js` **必须**同步改 `index.html` 里对应的 `?v=N`，否则 WebView 用缓存看不到改动。

---

## 8. 联调环境

- 平板开热点，SSID `robot` / 密码 `12345678`
- K230、母机、摄像头板全部连这个热点
- 三块板都监听 UDP 48899，App 广播 `WALLBOT_DISCOVER` 收 `WALLBOT:<role>:<ip>`，role 为 `host` / `camhost` / `camchild`
- K230 额外每 2s 主动广播 `WALLBOT:camchild:<ip>`

**读母机串口**（看 `[AV]` 日志）：

```sh
cat /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*
```

**读 K230 状态**：

```sh
curl http://<k230-ip>/status
curl http://<k230-ip>/align/get
```

---

## 9. 已知待办

1. `AV_CMD_MIN_PHI/PSI/AX`（`config.h:176-178`）是占位值 0.8/0.8/3.0，需实测子机最小可动命令后修正
2. `ADJ_MM_PER_MOTORREV = 0.3555`（`main.c:964`）未经卷尺复核
3. `ADJ_SPEED_REV` 实际值未核对
4. 符号方向（φ/ψ/ax）需实机逐轴验证
5. `firmware/align_bridge.cpp/.h`、`摄像头子机/` 可清理
6. `align_vision.h:103` 注释过时（把 `[7] ax` 写成 `reason`）
