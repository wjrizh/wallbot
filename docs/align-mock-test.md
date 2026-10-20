# 视觉纠偏假数据闭环测试（无摄像头）

**状态：临时测试功能。验证通过后可整体回滚，不影响正式视觉纠偏链路。**

## 版本标记

| 标签 | 提交 | 说明 |
|---|---|---|
| `test-baseline-ac30b5c` | ac30b5c | 加测试功能**之前**的基线，回滚目标 |
| `test-align-mock-v1` | c8104c4 | 本测试功能 v1，已编译烧录 |

### 回滚方式

```sh
git reset --hard test-baseline-ac30b5c    # 完全回到测试前
git reset --hard test-align-mock-v1       # 回到测试版本
```

若只想回滚 App 或某一部分：`git checkout test-baseline-ac30b5c -- app/www/js/modules/align.js`

## 目的

摄像头子机视觉还没调通，先用**假的图像识别数据**验证「母机 → 子机」纠偏信号传输与执行链路是否正常。

```
App 输入假偏差(φ/ψ/ax)
    → WS 0x31
    → 母机 test_mode 拼 "ADJ,φ,ψ,ax"
    → NRF24L01
    → 子机 Test_WaitLoop 收帧 → Adj_Execute 驱动两轮
    → 回 DONE
    → 母机轮次+1，App 显示
```

期间子机每 500ms 回一帧 `'J'` 遥测，App 画轮子转速/里程。

## 协议

### 母机 ↔ App（WebSocket 二进制，AA 55 CMD LEN payload CRC16）

| CMD | 方向 | payload | 说明 |
|---|---|---|---|
| 0x30 | App→母机 | 空 | 开始测试，母机发 TSTA |
| 0x31 | App→母机 | 3×float32LE (phi,psi,ax) | 下发假偏差 |
| 0x32 | App→母机 | 空 | 结束测试，母机发 TSTE |
| 0x86 | 母机→App | 'J' 帧 11 字节 | 轮子运动遥测 |
| 0x87 | 母机→App | [state][round][last_rc] | 测试状态 |

### 母机 ↔ 子机（NRF24L01，ASCII 前缀 + 变长）

| 帧 | 方向 | 说明 |
|---|---|---|
| `TSTA` | 母→子 | 进入测试，子机回 TSTB |
| `TSTB` | 子→母 | 测试就绪 |
| `ADJ,<phi>,<psi>,<ax>` | 母→子 | 假视觉偏差 |
| `DONE` | 子→母 | 本轮执行完毕 |
| `TSTE` | 母→子 | 退出测试 |
| `'J'+10B` | 子→母 | 遥测：stage/dir/a_mm/b_mm/target/pct/rc |

`'J'` 帧格式：

```
[0] 'J'  [1] stage(1=轴向 2=自旋 3=直行)  [2] dir(±1)
[3..4] a_mm int16LE   [5..6] b_mm int16LE   [7..8] target_mm int16LE
[9] pct 0..100        [10] rc(0=进行中 1=完成 2=中止 3=超时打滑)
```

## 子机改动（firmware/pabi_F103rct6/Core/Src/main.c）

- `Test_WaitLoop()`：上电先进这里，等 `TSTA` 或 `COOP`。收 `COOP` 则预取后交 `AUTO()`，正式协同流程不变。
- `Coop_SendRaw()`/`Coop_SendFrame()`：变长 + 有限重试（原 `while` 死等，对端不在会卡死）
- `Test_Telem()`：遥测发送
- `Adj_Move`/`Adj_Spin`：循环内每 500ms 发遥测，各出口带 rc
- `ADJ_SPEED_REV` 8.0 → **3.0**（地面测试慢速，肉眼可看）
- `Coop_IsProtoFrame` 纳入 `TSTA`/`TSTE`，避免被误判成遥控中止

## 母机改动（firmware/）

- 新增 `test_mode.cpp/.h`：状态机 `IDLE→SEND_TSTA→WAIT_TSTB→READY→SEND_ADJ→WAIT_DONE→READY`
- `joypad_sender.cpp`：测试帧发送优先级最高；主循环调 `TestMode_Tick()`
- `web_server.cpp`：WS 0x30/0x31/0x32 分发，0x86 遥测转发，0x87 状态推送
- `calibration_test.ino`：`TestMode_Init()`，推送任务加 `WebServer_PushTestStatus()`

## App 改动（app/www/）

- `js/modules/align.js` 重写：按钮改「视觉纠偏测试」
  - ① 管道截面姿态图：φ→圆周角，ψ→方块自身偏航，ax→径向收缩；超差红/达标绿
  - ② 差速轮：A/B 里程 + 同向=自旋 / 反向=直行判定
  - ③ 阶段进度条 ④ 握手日志
  - 真实摄像头 `/align/get` 折叠在底部（未就绪时不用）
- `js/protocol.js`：`TEST_START/TEST_ADJ/TEST_STOP` + `parseTestTelem/parseTestState`
- `js/transport.js`：`testTelem`/`testState` 事件

## 使用步骤

1. 子机烧 `firmware/pabi_F103rct6/build/pabi_F103rct6.bin`（ST-Link：`st-flash --reset write ... 0x8000000`）
2. 母机烧录：
   ```sh
   mkdir -p /tmp/wbfw/wallbot_fw && cp firmware/*.cpp firmware/*.h /tmp/wbfw/wallbot_fw/ \
     && cp firmware/calibration_test.ino /tmp/wbfw/wallbot_fw/wallbot_fw.ino
   arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
     --output-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
   arduino-cli upload -p /dev/ttyACM0 \
     --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
     --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
   ```
3. App：`npx cap sync android && cd android && JAVA_HOME=/home/z/toolchain/jdk-17.0.20.1+1 ./gradlew assembleDebug`
4. 子机放地面、**不开风机**、轮子能自由转
5. App 点「视觉纠偏测试」→「开始测试」→ 串口应见 `TSTB, child ready`
6. 输入 φ/ψ/ax 或点预设 →「发送模拟偏差」
7. 「结束测试」退出

## 观察要点

| 现象 | 说明 |
|---|---|
| 符号错（越修越歪） | 改 `ADJ_SPIN_SIGN` / `ADJ_MV_SIGN` / `ADJ_AXIAL_SIGN` |
| 直行段两轮同向 | `PID_MA/PID_MB` 符号配错 |
| 里程明显偏大/偏小 | `ADJ_MM_PER_MOTORREV=15`、`ADJ_TRACK_MM=100`、`ADJ_PIPE_R_MM=60` 都是占位值，需卷尺实测 |
| 两轮闪红 + rc=3 | 打滑（`ADJ_SLIP_DIFF_MM=8`），吸在管壁上或负载过大 |

## 注意

- 提交 `c8104c4` 同时带入了**上一轮遗留未提交**的摄像头子机 4 个文件（`align_server.cpp`/`config.h`/`log_page.cpp`/`tag_detector.cpp`，T4 对齐改动）。回滚到 `test-baseline-ac30b5c` 会一并撤销这些改动。
- `firmware/pabi_F103rct6/Drivers/` 有 68MB HAL 库，已入库。若嫌大可在后续单独拆分。
