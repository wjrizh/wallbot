# 爬壁机器人项目 - 开发文件迁移包

打包时间: 2026-09-16

## 目录结构

- firmware/ : ESP32-S3 母机固件源码 (Arduino)
- app/ : Capacitor Android App 源码
- app/www/ : 前端源码 (HTML/CSS/JS)
- app/android/ : Android 原生工程 (不含构建产物)
- ACK闭环测试表.md

## 环境要求

### 固件编译烧录

- arduino-cli 1.5.1
- ESP32 core 3.3.11
- 库: ESP Async WebServer 3.12.1, Async TCP 3.5.0, RF24 1.6.2
- 烧录端口: /dev/ttyACM0 (当前用户需在 uucp 组, 否则 chmod 666)

母机板实测 **16MB Flash + 8MB OPI PSRAM** (`esptool flash_id` 结果)。
**必须用 16MB 分区方案**, 默认 4MB 方案 APP 只有 1.2MB, 加入视觉纠偏与子机直通后已到 95%, 极易爆。
改用 3MB APP 分区后占用约 39%。

- FQBN: `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB`

编译 (arduino-cli 要求目录名与主 .ino 同名, 先复制到临时目录):
```sh
mkdir -p /tmp/wbfw/wallbot_fw && cp firmware/*.cpp firmware/*.h /tmp/wbfw/wallbot_fw/ \
  && cp firmware/calibration_test.ino /tmp/wbfw/wallbot_fw/wallbot_fw.ino
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --output-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```

烧录 (必须用 arduino-cli upload, 会连同 bootloader/分区表一起写入):
```sh
arduino-cli upload -p /dev/ttyACM0 \
  --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```

> **分区表变更的恢复说明**: 本次把母机从默认 4MB 分区改成了 16MB (`app3M_fat9M_16MB`)。
> 若只单独写 app bin 会导致启动失败, 需要整片烧写:
> `esptool --port /dev/ttyACM0 write_flash 0x0 /tmp/wbfw/out/wallbot_fw.ino.merged.bin`
> 正常升级仍推荐 `arduino-cli upload` (它会同时写 bootloader、分区表、app)。
> 若要退回旧的 4MB 分区方案, 把 FQBN 中 `FlashSize=16M,PartitionScheme=app3M_fat9M_16MB` 整段删掉重新编译烧录即可。

### 摄像头板编译烧录 (摄像头母机 / 摄像头子机)

两块摄像头板均为 ESP32-S3-CAM + **8MB Flash + OPI PSRAM**。
务必用 `FlashSize=8M`, 用 16M 会启动即崩:
`Detected size(8192k) smaller than the size in the binary image header(16384k). Probe failed.`

- FQBN: `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB`
- 端口: 摄像头板走 USB 转串口, 通常是 `/dev/ttyUSB0` (母机板是 `/dev/ttyACM0`)
- arduino-cli 要求目录名与主 .ino 同名, 先复制到临时目录再编译:

```sh
mkdir -p /tmp/wbcam/camhost
cp 摄像头母机/*.ino 摄像头母机/*.cpp 摄像头母机/*.h /tmp/wbcam/camhost/ 2>/dev/null
mv /tmp/wbcam/camhost/摄像头母机.ino /tmp/wbcam/camhost/camhost.ino
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB' /tmp/wbcam/camhost
arduino-cli upload -p /dev/ttyUSB0 --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB' /tmp/wbcam/camhost
```

子机同理 (子机已按模块拆分, 共 10 个 .cpp/.h, 必须全部拷贝):

```sh
mkdir -p /tmp/wbcam/camchild
cp 摄像头子机/*.ino 摄像头子机/*.cpp 摄像头子机/*.h /tmp/wbcam/camchild/
mv /tmp/wbcam/camchild/摄像头子机.ino /tmp/wbcam/camchild/camchild.ino
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB' /tmp/wbcam/camchild
arduino-cli upload -p /dev/ttyUSB0 --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB' /tmp/wbcam/camchild
```

摄像头子机纠偏接口 (`:80`): `/align/ref` 记录参考位, `/align/get` 测量偏差,
`/align/status` 查看状态。母机 `align_bridge.cpp` 会周期拉取 `/align/get` 驱动子机纠偏。

### 设备地址 (自动发现)

三块板均通过 DHCP 获取 IP, 并监听 UDP 48899 应答发现请求:

- 请求: 广播 `WALLBOT_DISCOVER` 到 48899
- 应答: `WALLBOT:<role>:<ip>`, role 为 `host` / `camhost` / `camchild`

App 首次启动广播发现并缓存地址; 再次启动直接复用缓存,
仅在 WebSocket 连不上或摄像头重试耗尽时才重新发现。
面板「设备地址」也可手动填写兜底。

### App 构建

- Node.js + npm
- Android SDK (local.properties 里 sdk.dir 需按新机器路径改)
- JDK 17

步骤 (在 app 目录下):
1. npm install
2. npx cap sync android
3. cd android && ./gradlew assembleDebug

APK 输出: android/app/build/outputs/apk/debug/app-debug.apk

## 注意

- node_modules/ 未打包, 新机器执行 npm install
- build/ 与 .gradle/ 未打包, 构建时自动生成
- android/app/src/main/assets/public/ 未打包, 由 npx cap sync android 从 www/ 生成
- android/local.properties 未打包, 需新建并写入 sdk.dir=新机器Android SDK路径
- 固件与 App 状态帧长度必须配套 (当前 payload 32 字节)

