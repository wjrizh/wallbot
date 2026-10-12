# 任务：把母机固件的 RS02 CAN 电机驱动从「CSP 位置模式」恢复为「老版标准帧 MIT 运控模式」

## 背景

爬壁机器人 ESP32-S3 母机固件，RS02 一体化电机走 CAN（标准帧，1Mbps）。
固件曾从「老版标准帧 MIT 运控」被改写成「扩展帧 CSP 位置模式」，现在要**改回老版 MIT**，
因为 CSP 版本真机测试「无法使能、App 无角度反馈」。

**目标：恢复老版 MIT 后，电机能正常使能、运动，并把角度反馈上报到 App。**

## 两套驱动的本质区别

### 老版（要恢复成的）——标准帧 MIT 运控，与 STM32 已验证版本一致
- 标准帧，`StdId = 电机ID`（1 或 2），8 字节数据
- **使能**：`data[0..6]=0xFF, data[7]=0xFC`
- **停止/失能**：`data[7]=0xFD`
- **设零**：`data[7]=0xFE`
- **MIT 运控** 12bit 混合打包：
  - 位置 16bit（范围 ±12.57 rad）
  - 速度 12bit（±44 rad/s）
  - Kp 12bit（0~500）
  - Kd 12bit（0~5）
  - 力矩 12bit（±17 N·m）
  - 字节布局：`d0=p>>8, d1=p; d2=v>>4, d3=(v&0xF)<<4|kp>>8, d4=kp; d5=kd>>4, d6=(kd&0xF)<<4|t>>8, d7=t`
- **反馈解析**：只收标准帧，`data[0]=电机ID, data[1..2]=角度原始值`
  - `angle = raw/65535*25.14 - 12.57`（弧度）
- **接口**：`Init / Enable / Disable / SetZero / MIT_Ctrl / StartTraj / IsMoving / GetPosition / ResetTraj / Task_CANMotorDrive`
- **无** `StartReport / CSP_Setup / SafeHome / AutoHome / CSP_SetPos / WriteParam* / csp_ready / g_fb_received`

### 新版（当前在用的，要替换掉的）——扩展帧 CSP 位置模式
- 扩展帧，多了 `StartReport / CSP_Setup / SafeHome / AutoHome / csp_ready / g_fb_received`
- 使能/设零/CSP 位置下发走扩展帧帧格式

## 移植步骤（照做即可）

1. **覆盖驱动**：用 `can_rs02_MIT.cpp` / `can_rs02_MIT.h` 覆盖固件里的 `can_rs02.cpp` / `can_rs02.h`。
   （这两个文件与 `/home/z/calibration_test/` 下老版逐字节一致。）

2. **删自动归零任务**：`calibration_test.ino` 里删掉 `Task_AutoHome` 函数定义及对应的 `xTaskCreate(Task_AutoHome, ...)` 调用。
   （老版驱动没有 AutoHome 依赖。）

3. **改使能分支**：`web_server.cpp` 的 CAN 命令 0x05 使能分支，
   `CAN_RS02_CSP_Setup(id, 20.0f, 8.0f)` → `CAN_RS02_Enable(id)`。

4. **清残留宏引用**：`config.h` 里 CSP 专用宏（`CSP_HOME_SAFE_CUR / CSP_HOME_DUR / CSP_HOME_SPD / CSP_HOME_SAFE_MIN / CSP_HOME_SAFE_MAX` 等）若编译报未使用/未定义，按报错清理或删除；老版 `config.h` 见 `config_MIT.h`，其 CAN 段与新版一致（TX=6/RX=7/TASK_PERIOD=20/RX_QUEUE=50；LOCK_KP=200/LOCK_KD=4/LOCK_VEL=0/LOCK_TORQ=0）。

5. **`rs02_correct.cpp`**：老版与新版逻辑相同（接力微调），可保留新版（新版多一个 settle 窗口），也可用 `rs02_correct_MIT.cpp`。两者接口一致。

6. **编译**（见下方命令），按报错补 `config.h` 残留引用，直到通过。

7. **烧录**。

## 编译环境

- arduino-cli 1.4.1，ESP32 core 3.3.10
- FQBN：`esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default`
- 端口：`/dev/ttyACM0`
- **注意**：目录名必须与主 ino 名一致。做法：把 `firmware/` 复制到 `/tmp/wb_fw`，删掉所有 `.bak*` 文件，把 `calibration_test.ino` 改名为 `wb_fw.ino`，再编译。

```sh
rm -rf /tmp/wb_fw && mkdir -p /tmp/wb_fw && cp firmware/* /tmp/wb_fw/ && \
cd /tmp/wb_fw && rm -f *.bak2 *.bak3 *.bak_mit && mv calibration_test.ino wb_fw.ino && \
arduino-cli compile -b "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default" .
```

## 关键约束 / 经验

- **本环境工具对 `<` 字符敏感**：写文件/shell heredoc 遇 `<`（`#include <...>`、比较符、HTML）会截断。规避：用 `\x3c` 转义，或后处理解码。
- **工具调用格式易被拒**：多命令、`cat`、大量 `|` 的 execute_command 常触发解析失败。改用单条最简命令或 read_file。
- **接口一致性**：全工程引用 `CAN_RS02_*` 的地方（`auto_align.cpp / rs02_correct.cpp / protocol.cpp / web_server.cpp / gait_engine.cpp`）调用的都是两版共有的接口，MIT 覆盖后不应有孤儿引用；若有 `StartReport/SafeHome/AutoHome/CSP_*` 残留调用，一并清理。
- **MIT 帧在电机未使能时也会被 Task 每 20ms 无条件下发**（`Task_CANMotorDrive`），这是老版原有行为，不用改。

## 状态帧与 App 通信（已核对，无需改）

- 帧格式：`AA 55 CMD LEN [PAYLOAD] CRC16_LE`（Modbus 0xA001，初值 0xFFFF）
- 状态帧 `0x81`（32B）：
  - 0-3 舵机 m1/m2/m3/pb9；4 泵位；5-8 CAN1 弧度(float32)；9-12 CAN2 弧度(float32)；
  - 13-16 IMU roll；17 IMU 在线；18-19 激光 mm(LE)；20 激光在线；21 保留；
  - 22-25 地面标定；26 纠偏模式；27-30 夹持器角；31 夹持器在线
- CAN 角度来源 `CAN_RS02_GetPosition(1/2)`，MIT 版该接口存在，App 用 `getFloat32` 解析，一致。
- CAN 命令：`0x04`→`CAN_RS02_StartTraj`；`0x05`→`Enable/Disable/SetZero`（sub 0/1/2）。

## 交付物

把 `_can_port_handoff/` 整个目录交给执行者，包含：
- `can_rs02_MIT.cpp/.h`：老版 MIT 驱动（覆盖用）
- `config_MIT.h`：老版 config.h（参考）
- `rs02_correct_MIT.cpp`：老版 rs02_correct.cpp（可选）
- `can_rs02_CSP.cpp/.h`、`config_CSP.h`、`web_server_CSP.cpp`、`calibration_test_CSP.ino`、`rs02_correct_CSP.cpp`：当前 CSP 版（对照用）
- 本提示词

## 验证标准

1. 编译通过（约 86% flash）
2. 烧录校验通过
3. 真机：网页使能电机 → 电机动、App 显示角度反馈
4. 真机：CAN 角度下发指令（0x04）电机按目标转动