# 爬壁机器人母机控制 App

平板 App（iPad / Android Pad），控制爬壁机器人母机（ESP32）。
基于母机现有 WebSocket 二进制协议，与固件 `calibration_test/` 完全兼容。

## 功能

1. **CAN 电机**：电机 1 / 2 的 使能、失能、设零，并显示实时角度反馈。
2. **步态**：输入步距（10~200 mm）前进；越障；急停。
3. **子母协同**：启动协同、简化自动；显示母机滚转与 IMU 在线状态。

## 界面

平板横屏为固定双栏，全屏不滚动：

```
┌────────────────────┬──────────────────────────┐
│  ● 爬壁机器人 已连接 │  运行状态      清空日志    │
│                    ├──────────────────────────┤
│  连接              │                          │
│  [192.168.4.1][连接]│                          │
│                    │                          │
│  CAN 电机 RS02     │        运行日志           │
│  电机 1      70.7° │                          │
│  [使能][失能][设零] │                          │
│  电机 2     -28.6° │                          │
│  [使能][失能][设零] │                          │
│                    │                          │
│  步态              │                          │
│  步距 [− 90 mm +]  │                          │
│  [前进]  [越障]    │                          │
│  [    急停    ]    │                          │
│                    │                          │
│  子母协同          │                          │
│  [启动协同][简化自动]│                          │
│  母机滚转  2.5° 在线│                          │
└────────────────────┴──────────────────────────┘
```

- 横屏（>700px）：左操作栏 380px + 右状态区。
- 竖屏平板（701~1024px）：操作栏收窄至 340px。
- 窄屏（≤700px）：上下堆叠，兼容手机。

## 流畅性设计

| 措施 | 说明 |
|---|---|
| rAF 合并刷新 | 状态帧到达只暂存，由 `requestAnimationFrame` 每帧统一刷新一次 |
| 无滤镜 | 不使用 `backdrop-filter`，避免 GPU 合成开销 |
| GPU 按压缩放 | 按压反馈用 `transform`，不触发重排 |
| 固定布局 | 全屏不滚动，无滚动条抖动 |
| 实测开销 | 单次状态刷新 0.004 ms（60fps 预算 16.7 ms） |

状态帧 5Hz 到达时，突发多帧只应用最后一帧，实测无丢帧、无抖动。

## 目录结构

```
www/
  index.html            界面结构（平板双栏）
  css/app.css           样式与响应式断点
  js/protocol.js        二进制协议编解码（帧格式、CRC16）
  js/transport.js       WebSocket 连接、自动重连、事件订阅
  js/native.js          Capacitor 原生适配（返回键、禁缩放）
  js/ui.js              toast / 日志 / DOM 工具
  js/modules/can.js     CAN 电机模块
  js/modules/gait.js    步态模块
  js/modules/coop.js    子母协同模块
  js/app.js             应用入口、模块装配、rAF 调度
test/protocol.test.js   协议单元测试
tools/serve.js          本地开发服务器
android-res/xml/         Android 明文流量配置
```

## 协议

帧格式：`AA 55 CMD LEN [PAYLOAD] CRC16_LE`，CRC16 为 Modbus 多项式
（初值 `0xFFFF`，多项式 `0xA001`）。

本 App 使用的命令：

| CMD | 功能 | PAYLOAD |
|---:|---|---|
| 0x05 | CAN 命令 | id(1) subcmd(1)：0=使能 1=失能 2=设零 |
| 0x11 | 越障 | 无 |
| 0x12 | 急停 | val(1)=0 |
| 0x13 | 迈步 | step_mm(4, float32 LE) |
| 0x19 | 启动协同 | 无 |
| 0x1A | 简化自动 | 无 |

状态帧 `0x81`（27 字节 payload / 33 字节总帧）由母机 5Hz 推送，
字段布局与固件 `web_server.cpp:WebServer_PushLaser` 逐字节对齐：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0–3 | u8×4 | 舵机 m1 / m2 / m3 / pb9（角度整数）|
| 4 | u8 | 泵位：bit0 前、bit1 后 |
| 5–8 | f32 LE | CAN 电机 1 弧度 |
| 9–12 | f32 LE | CAN 电机 2 弧度 |
| 13–16 | f32 LE | IMU 滚转 |
| 17 | u8 | IMU 在线标志 |
| 18–21 | u8×4 | 子机 IP（192.168.4.55）|
| 22–25 | f32 LE | 地面标定滚转 |
| 26 | u8 | 纠偏模式 |

其余命令（舵机、泵、角度、空中标定、自动纠偏、掉头、虚拟手柄）已在
`protocol.js` 中实现，UI 可按需接入。

## 运行

### 浏览器预览

```sh
npm run serve          # http://localhost:8080
```

### 打包为 Android 平板 App

```sh
npm install
npx cap add android
mkdir -p android/app/src/main/res/xml
cp android-res/xml/network_security_config.xml android/app/src/main/res/xml/
# 在 android/app/src/main/AndroidManifest.xml 的 <application> 上添加：
#   android:usesCleartextTraffic="true"
#   android:networkSecurityConfig="@xml/network_security_config"
npx cap sync android
npx cap open android   # 用 Android Studio 构建安装
```

### 打包为 iPad App

```sh
npm install
npx cap add ios
npx cap sync ios
npx cap open ios       # 用 Xcode 构建安装（需 macOS）
```

> iOS 默认禁止非加密 WebSocket。若真机连接失败，需在 `Info.plist` 添加
> `NSAppTransportSecurity` → `NSAllowsArbitraryLoads = true`。

## 连接

1. 平板连接母机 WiFi：SSID `CalibTest`，密码 `12345678`。
2. 打开 App，地址填 `192.168.4.1`（默认），点「连接」。
3. 状态指示灯变绿即表示 WebSocket 已连通，可下发命令。

断线后 App 自动重连（1~8 秒退避）。地址保存在本机，下次启动自动填充。

## 扩展

新增功能时：

- **新命令**：在 `www/js/protocol.js` 的 `Protocol` 上添加构造函数。
- **新模块**：在 `www/js/modules/` 新建文件，实现 `mount(container)` 与
  `onState(state)`，在 `www/js/app.js` 的 `modules` 数组注册，
  并在 `_containerFor()` 返回容器（静态 HTML 模块返回 `null`）。
- **CAN 电机增减**：修改 `www/js/modules/can.js` 的 `MOTORS` 数组。
- **协同动作增减**：修改 `www/js/modules/coop.js` 的 `ACTIONS` 数组。

模块间不耦合，均通过 `transport.on('state', ...)` 接收状态帧。

## 测试

```sh
npm test
```

覆盖 CRC16 标准向量、各命令帧字节序、状态帧解析、坏帧拒绝。

## 安全

- 急停按钮下发 `CMD 0x12 val=0`，触发母机步态急停。
- 软件停止不能替代硬件急停；动力回路必须保留可触达的物理断电装置。
- 首次联调建议悬空机器人或卸除负载。
