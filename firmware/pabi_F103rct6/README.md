# pabi_F103rct6 子机控制固件（STM32F103RCT6）

来源：github.com/shuaiqideqo/pabi-F103rct6（下载包 pabi_F103rct6.zip）。
已剔除：*.bak_* 备份、build/、MDK 中间产物（*.htm / *.lnp / *.dep / *.uvguix.*）、根目录临时烧录脚本（_flash_*.ps1 / flash_now.bat）与 build_log.txt。

## 目录

- Core/Inc Core/Src — 应用代码，主要逻辑集中在 Core/Src/main.c（约 1341 行）
- Drivers/ — STM32F1xx HAL + CMSIS（体积主体，68MB）
- MDK-ARM/pabi_F103rct6.uvprojx — Keil 工程
- Makefile + gcc_build/STM32F103RCTx_FLASH.ld — GCC 构建

## 硬件资源占用

| 外设 | 用途 |
|---|---|
| TIM2 CH1/CH2 (PA15/PB3) | 编码器 A 轮 |
| TIM3 CH1/CH2 (PB4/PB5) | 编码器 B 轮 |
| TIM4 CH1-4 (PB6-PB9) | 轮电机 PWM（A: IN1/IN2, B: IN3/IN4） |
| TIM5 CH1/CH2 (PA0/PA1) | 风机 PWM、第二路 50Hz |
| TIM8 CH1/CH2 (PC6/PC7) | 舵机 PWM |
| USART2 (PA2/PA3, 115200) | 树莓派串口链路，当前 PI_LINK_EN=0 停用 |
| SPI2 | NRF24L01（与母机通信） |
| MPU6050 + DMP | 姿态 |

## 子母协同协议（NRF24L01 侧）

4 字节 ASCII 帧，接收侧在 32 字节载荷里做子串匹配（Coop_FindFrame），无长度/校验字段。

母机 -> 子机：

| 帧 | 含义 |
|---|---|
| COOP | AUTO 周期开始，子机启动 IMU 周期上报、开风机、下压 |
| GOON | 放行下一步（AUTO 内多处复用；纠偏阶段表示已对齐） |
| LOCK | DONE 后锁定，子机发 ACKD 并结束本周期 |
| STOP | 中止，清摇杆，立即返回手动 |
| ADJ,<phi>,<psi>,<ax> | 纠偏量帧；ADJ,0,0,0 用于使能（armed） |

子机 -> 母机：

| 帧 | 含义 |
|---|---|
| REDY | ①开风机稳住 1s 后；②GO_UP 结束、准备视觉纠偏 |
| DONE | 本周期作业完成（纠偏成功后） |
| ACKD | 收到 LOCK，风机已关，周期结束 |
| 'I' + pitch(4B) + roll(4B) | 作业期间每 200ms 上报 IMU 姿态 |

关键常量（Core/Src/main.c）：ADJ_WAIT_MS=180000、COOP_IMU_PERIOD_MS=200。

## AUTO 周期时序

等 COOP -> 开风机 -> 稳 1s -> REDY① -> 等 GOON -> Compose_IMU -> GO_DOWN(-90) -> 舵机下压 8s -> 吸力加到 1639 -> GO_UP(roll) -> [若本周期收过 ADJ 帧] REDY② + Adj_WaitLoop() 等视觉对齐 -> DONE -> 等 LOCK -> 关风机 -> ACKD -> 下一周期

任一步收到 STOP 或遥控非零摇杆帧即中止（Coop_TakeAbort），刹轮、保持吸力、回手动，不上报 DONE。

## 树莓派串口链路（USART2，默认关闭）

PI_LINK_EN=0 时完全不挂接收中断，纠偏走 NRF。置 1 可恢复网页/树莓派手动控制，协议为 ASCII 行（换行结尾）：

W 进作业模式 / X 退出 / F+ F- F0 风机 / S+ S- 舵机 / F=nnn S=nnnn 直接设 PWM。每条回 OK <cmd> 或 ERR。

## 与摄像头子机的边界

本工程只负责运动执行与协同握手；视觉偏差量由母机（ESP32-S3，firmware/align_bridge.cpp）经 NRF 以 ADJ 帧下发。子机不解析图像，也不参与 tag 检测。
