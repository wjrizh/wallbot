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
- FQBN: esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default
- 烧录端口: /dev/ttyACM0 (需 chmod 666)

编译 (在 firmware 目录下):
arduino-cli compile --fqbn esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default --output-dir /tmp/wallbot_build .

烧录:
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default --input-dir /tmp/wallbot_build .

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

