# 真实视觉纠偏 — 实机测试说明

集成日期 2026-10-25。链路：

```
K230 (:80 /align/*)  <-- HTTP --  ESP32-S3 母机 align_vision
                                    | TestMode_SendAdj (ADJ,phi,psi,ax)
                                    v NRF24L01 (32B 定长, 半双工)
                                STM32F103 子机 Adj_Execute
                                    | DONE / 'J' 遥测 / 'L' 日志
                                    v
                                ESP32-S3 母机 -> 串口 + WS(0x88/0x89/0x86/0x87)
```

---

## 一、烧录

### 1. 母机 (ESP32-S3)

```sh
rm -rf /tmp/wbfw/wallbot_fw /tmp/wbfw/out && mkdir -p /tmp/wbfw/wallbot_fw
cd /home/z/wallbot-repo && cp firmware/*.cpp firmware/*.h /tmp/wbfw/wallbot_fw/
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' --output-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
arduino-cli upload -p /dev/ttyACM0 --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```

编译前需把 calibration_test.ino 复制为 wallbot_fw.ino（目录名须与主 ino 同名）。

### 2. 子机 (STM32F103)

仓库内 firmware/pabi_F103rct6/Core/Src/main.c 已替换为新版（融合纠偏规划、beta=20、R=250、track=145、SPEED=80）。

```sh
cd /home/z/wallbot-repo/firmware/pabi_F103rct6 && make -j4
st-flash --reset write build/pabi_F103rct6.bin 0x8000000
```

子机 ADJ 帧格式、TSTA/TSTB/DONE/TSTE 握手、'J'/'L' 帧与旧版完全一致，母机侧无需适配。

### 3. K230

运行 dual_tag_align_web.py（或已部署的 main.py），记下 IP。默认 AP 模式热点 k230-align / 密码 12345678，IP 192.168.4.1。

### 4. App

```sh
cd /home/z/wallbot-repo/app && npx cap sync android && cd android && ./gradlew assembleDebug
```

APK 路径：app/android/app/build/outputs/apk/debug/app-debug.apk

---

## 二、串口日志速查（母机 115200）

| 前缀 | 来源 | 含义 |
|---|---|---|
| [AV] | align_vision | 闭环状态机：启动、测量、命令、结束 |
| [TEST] | test_mode | TSTA/ADJ/TSTE 收发、TSTB/DONE、超时 |
| [TLM] | test_mode | 子机轮子遥测（阶段/里程/百分比/rc） |
| [CHILD] | 子机 'L' 帧 | 子机段级日志（spgo/mvgo/spin ok/move ok/finish） |

一次成功闭环的日志样例：

```
[AV] init tune sign=+1/+1/+1 beta=20 spd=100%
[AV] start http://10.32.7.230 gain=30% iter<=8 flags=0x06 sign=+1/+1/+1
[AV] cam start -> ok
[TEST] start, send TSTA
[TEST] TSTB, child ready
[AV] child ready (round=0)
[AV] 子机就绪, 开始视觉测量
[AV] once 412ms phi=3.24 psi=-1.80 ax=12.4 aligned=0
[AV] it=1 cmd phi=0.97 psi=-0.54 ax=12.4 (raw 3.24/-1.80/12.4 gain=0.30)
[TEST] round 1 send ADJ,1.0,-0.5,12.4
[CHILD] fuse b20 L36 sc34 r-31
[CHILD] sp1 th22 (b-ps)
[CHILD] spgo a26 n73 to21
[TLM] st=2 dir=1 A=13 B=13 tgt=26 pct=50 rc=0
[CHILD] spin ok t22 a26 A26 B26
[CHILD] mvgo t36 n101 to21
[TLM] st=3 dir=1 A=18 B=18 tgt=36 pct=50 rc=0
[CHILD] move ok d36 A36 B36
[CHILD] sp2 th-20 (0-b)
[CHILD] spin ok t-20 a24 A24 B24
[CHILD] move ok d-31 A-31 B-31
[CHILD] finish rc=0
[TEST] DONE round 1
[AV] round 1 done (6.2s)
[AV] it=2 meas phi=0.62 psi=-0.31 ax=1.8 aligned=1
[AV] aligned hit 1/2
[AV] it=2 meas phi=0.40 psi=-0.20 ax=1.2 aligned=1
[AV] 视觉判定已对齐, 闭环完成
[AV] finish state=7 rc=10 iter=1
[AV] cam stop sent
```

---

## 三、分步测试流程（务必按顺序）

### 步骤 0：链路自检（不发任何运动）

App 打开视觉纠偏面板，填 K230 IP，点「摄像头状态」。

串口应出现：

```
[AV] camctl /align/status -> code=200 8ms mode=ap ip=192.168.4.1 running=no ref=no reason=idle up=42s
```

失败则检查 IP 与 WiFi（母机必须与 K230 同网段）。

### 步骤 1：开摄像头并记参考位

点「开」（/align/start），再点「记参考」（/align/ref）。

```
[AV] camctl /align/start -> code=200 ok
[AV] camctl /align/ref -> code=200 1650ms ref=true id=0 u=312.5 v=238.0 theta=-1.20 edge=96.0
```

记参考时机械臂必须停在参考位姿，双码 ID0+ID1 都要在画面内。

### 步骤 2：干跑（只测不发）

点「单轮干跑」。串口应看到 [AV] once ... 与 [AV] 干跑: 本应下发 ADJ,...，子机完全不动。

验收：手动把标志块左右挪，phi_deg 随之正负变化；转一点角度，psi_deg 变化；沿轴向推，axial_mm 变化。三个量符号与物理方向对上后才进步骤 3。

### 步骤 3：单步标定符号（关键）

每次只跑一个自由度，观察车体实际运动方向。

| 点击 | 下发 | 期望物理方向 | 若反了 |
|---|---|---|---|
| psi自旋 +2 | ADJ,0,2,0 | 车体顺时针自转（俯视） | 面板 psi 选 -，点「写入标定」 |
| psi自旋 -2 | ADJ,0,-2,0 | 逆时针 | 同上 |
| phi直行 +2 | ADJ,2,0,0 | 沿管壁向 phi 正方向 | 面板 phi 选 - |
| phi直行 -2 | ADJ,-2,0,0 | 反向 | 同上 |
| ax斜行 +5mm | ADJ,0,0,5 | 斜行后净位移向 +ax 侧 | 面板 ax 选 - |
| ax斜行 -5mm | ADJ,0,0,-5 | 反向 | 同上 |

标定结果写 NVS，掉电保留；串口打印 [AV] tune ... (saved)。

单步时 flags 不带 START_CAM/STOP_CAM，摄像头保持原状。

### 步骤 4：小增益闭环

面板：增益 30、迭代 8、勾「开摄像头」与「结束关摄像头」，点「开始闭环」。

验收：[AV] it=N meas 里 |phi|+|psi|+|ax| 单调下降，3~6 轮内出现 aligned=1 两次，然后 finish state=7 rc=10。

### 步骤 5：全增益闭环

增益改 100 重跑，轮次应更少。

---

## 四、故障速查

| 串口现象 | 原因 | 处理 |
|---|---|---|
| camctl ... code=-1 | 网络不通 | 母机与 K230 同网段？AP 模式下平板先连 k230-align |
| 视觉未记参考位 | /align/once 返回 no_ref | K230 页面点「记参考位」 |
| K230 摄像头未运行 | 返回 not_running | K230 页面点「开始」，或勾「开摄像头」 |
| 连续未识别到双码, 放弃 | 画面里没有 ID0+ID1 | 调整相机/标志块位置，光照不足也会 |
| 子机 TSTB 超时 | NRF 单向不通 | 查 test_nrf 最小链路；PA6 是否 INPUT |
| 子机测试状态丢失 | 子机超时退出 60s | 检查 [CHILD] 日志卡在哪段 |
| 等 DONE 超时 | 子机运动超时/打滑 | 看 [TLM] 的 pct，卡在 50% 一般是轮子堵转 |
| 命令全在死区但 aligned=0 | 增益太小 | 提高增益或看 phi/psi/ax 本身在死区内 |
| 误差越来越大 | 符号反了 | 回步骤 3 标定 |
| [TLM] rc=3 | 子机里程超时 | 标定 ADJ_MM_PER_MOTORREV |
| [TLM] rc=2 | 子机被中止 | 收到 STOP 或遥控帧 |

---

## 五、可调参数

### 母机 firmware/config.h

| 宏 | 默认 | 说明 |
|---|---|---|
| AV_POLL_MS | 500 | 两次 /align/once 间隔 |
| AV_HTTP_TIMEOUT_MS | 8000 | K230 共识最长 5s |
| AV_MAX_ITER | 8 | 默认最大轮次 |
| AV_TOTAL_TIMEOUT_MS | 180000 | 闭环总时限 |
| AV_ALIGNED_HITS | 2 | 连续 aligned 判收敛 |
| AV_DONE_TIMEOUT_MS | 70000 | 等子机 DONE |
| AV_CMD_MAX_PHI/PSI/AX | 10/10/30 | 单轮限幅 |
| AV_DEFAULT_GAIN_PCT | 30 | 默认增益 |

### 子机 firmware/pabi_F103rct6/Core/Src/main.c

| 宏 | 当前值 | 说明 |
|---|---|---|
| ADJ_SPEED_REV | 80.0 | 纠偏轮速（电机轴转/s） |
| ADJ_MM_PER_MOTORREV | 0.3555 | 里程标定，卷尺复核 |
| ADJ_TRACK_MM | 145.0 | 轮距，影响自旋弧长 |
| ADJ_PIPE_R_MM | 250.0 | 管半径，影响 phi 到弧长 |
| ADJ_DIAG_BETA_DEG | 20.0 | 斜行角 |
| ADJ_MV_SIGN / SPIN_SIGN / AXIAL_SIGN | +1 | 保持 +1，符号在母机侧标定 |
| ADJ_TOL_PHI/PSI/AX | 0.5/0.5/2.0 | 与母机 AV_TOL_* 一致 |

母机侧符号标定生效后，子机三个 SIGN 宏不要再改，否则双重取反。

---

## 六、协议速查

### App 到母机（WS）

| CMD | payload |
|---|---|
| 0x22 启动 | ip[4] gainPct maxIter flags |
| 0x23 停止 | 无 |
| 0x24 标定 | signPhi i8 signPsi i8 signAx i8 betaDeg u8 speedPct u8 |
| 0x25 单步 | which u8 phi f32 psi f32 ax f32 |
| 0x26 视觉控制 | action u8 ip[4]（0=status 1=start 2=stop 3=ref 4=once 5=clrref） |
| 0x30/0x31/0x32 | 旧的模拟偏差测试（保留，台架用） |

flags：bit0 干跑 / bit1 开摄像头 / bit2 结束关摄像头 / bit3 单步

### 母机到 App（WS）

| CMD | payload |
|---|---|
| 0x88 闭环状态 | state u8 iter u8 rc u8 phi i16/100 psi i16/100 ax i16 aligned u8 maxIter u8 |
| 0x89 视觉 JSON | 原始 JSON 文本，最多 200B |
| 0x86/0x87/0x83 | 遥测 / 测试状态 / 日志（不变） |

state：0 空闲 1 启动摄像头 2 启动测试 3 等子机就绪 4 视觉测量 5 下发偏差 6 等子机执行 7 已完成 8 超时 9 视觉侧失败 10 子机侧失败 11 已停止

rc：0 无 1 未记参考 2 未开始 3 无码 4 需双码 5 HTTP 失败 6 达最大轮次 7 总超时 8 子机无响应 9 用户停止 10 完成

---

## 七、NRF 时序说明（改动边界）

闭环不直接操作 RF24，全部经 TestMode_*：

- 母机 10ms JoypadSender_Task 优先级：AutoMode 帧 > TestMode 帧 > ChildCmd > 虚拟手柄
- TestMode_IsActive() 期间该任务只排空 RX 并调 TestMode_Tick()，不发摇杆帧（避免自己占 TX 打断子机回包）
- 母机发送：stopListening -> flush_tx -> clearStatusFlags -> delay(2) -> write -> flush_rx -> startListening
- 子机发送：TX_Mode -> delay(2) -> safe_TxPacket -> RX_Mode；safe_TxPacket 先清 STATUS=0x70 再写 payload
- 子机接收：IRQ(PA6) 高 -> 无包 -> STATUS.RX_DR -> FIFO_STATUS.RX_EMPTY -> 读 -> flush_rx
- test_mode.cpp 新增 portMUX 保护状态与发送缓冲；TestMode_Start() 增加 AutoMode_IsActive() 互斥

---

## 八、本次改动清单

| 文件 | 改动 |
|---|---|
| firmware/align_vision.h/.cpp | 新增：HTTP + 状态机 + NVS 标定 + 单步 |
| firmware/config.h | 新增 AV_* 参数 |
| firmware/test_mode.cpp | 加锁、AutoMode 互斥、[TLM] 串口打印 |
| firmware/web_server.h/.cpp | 0x22-0x26 分发；新增 0x88/0x89 推送 |
| firmware/protocol.cpp | 移除旧 0x22/0x23（align_bridge） |
| firmware/calibration_test.ino | 启动 AlignVision_Task；周期推 0x88 |
| firmware/pabi_F103rct6/Core/Src/main.c | 替换为新版（融合规划、beta=20、R=250、track=145） |
| app/www/js/protocol.js | 0x22-0x26 构造器；0x88/0x89 解析 |
| app/www/js/transport.js | 分发 alignState / alignJson |
| app/www/js/modules/align.js | 新增「5 真实视觉闭环」区 + 单步标定 |
| firmware/align_bridge.h/.cpp | 保留但不再编译引用（可后续删除） |
