# 任务：排查 NRF24L01 子母机双向通信失败

## 你的角色

你是一名嵌入式通信调试专家。下面提供了一套爬壁机器人（母机 ESP32-S3 + 子机 STM32F103RCT6）的完整相关源码，请找出「母机发帧、子机收不到」或「子机回帧、母机收不到」的根因。

## 硬件与链路

- 母机：ESP32-S3，Arduino-ESP32 3.3.11，用 RF24 库（C++），软件 SPI
- 子机：STM32F103RCT6，HAL 库，寄存器直驱 NRF24L01（无 RF24 库）
- 两片 NRF24L01 模块，参数必须一致：信道 40；地址 {0x11,0x22,0x33,0x44,0x55}（5 字节）；payload 固定 32 字节；速率 2Mbps，CRC16，自动应答开启

## 业务背景

摄像头视觉纠偏还没调通，所以做了一个假数据测试功能：App 输入假的视觉偏差 φ/ψ/ax，经母机 -> NRF -> 子机，子机驱动两轮运动来修正，然后回报完成。目的是验证通信与执行链路。

```
App --WS--> 母机 test_mode --NRF--> 子机 Test_WaitLoop --驱动轮子--> 回 DONE --NRF--> 母机
```

## 协议

### 母机 -> 子机（ASCII 前缀，放在 32 字节 payload 内靠前）

| 帧 | 含义 |
|---|---|
| TSTA | 进入测试模式 |
| ADJ,<phi>,<psi>,<ax> | 下发假偏差，如 ADJ,5.0,5.0,10.0 |
| TSTE | 退出测试 |
| COOP / GOON / LOCK / STOP | 原有正式协同帧 |
| 32 字节摇杆帧 | [0]=0x00 [1]=speed [2]=spin [5]=key，每 10ms 发一次 |

### 子机 -> 母机

| 帧 | 含义 |
|---|---|
| TSTB | 测试就绪 |
| DONE | 本轮执行完毕 |
| REDY / ACKD | 正式协同应答 |
| 'I'+8B | IMU 姿态（pitch/roll 各 4 字节 float） |
| 'J'+10B | 轮子运动遥测 |

子机接收侧用滑动窗口做 4 字节子串匹配（Coop_FindFrame），不是精确比较。

## 当前现象

1. App 点「开始测试」-> 面板状态变「等 TSTB」-> 5 秒后变「超时」
2. 母机串口打印 [TEST] start, send TSTA，随后 [TEST] timeout in state 2
3. 子机串口完全没有打印 TSTA received, enter test
4. 子机也没有任何动作

## 已经修复过的问题（仍然失败）

### 母机侧

1. NRF_ReceiveFrame 原来用 radio.getDynamicPayloadSize()，但初始化是 radio.setPayloadSize(32) 固定模式，该函数恒返回 0 -> 每次都 flush_rx() 丢弃。已改为 radio.getPayloadSize()。
2. NRF_SendFrame(data, 4) 只传 4 字节，与 32 字节固定 payload 不匹配，radio.write 失败。已改为内部补齐到 32 字节。
3. TestMode_OnFrameSent() 缺 NRF_StartListening()，发完停在 PTX 模式。已补。
4. JoypadSender_Task 每 10ms 末尾无条件发摇杆帧，发完不回 RX，会打断测试接收。已改为测试激活时跳过摇杆帧、保持 RX。
5. NRF_StartListening/StopListening 改成幂等（带 s_listening 状态位），避免反复 flush_rx 冲掉已收到的帧。

### 子机侧

6. 主循环开头 NRF24L01_RxPacket((uint8_t*)Arr_RX) 每轮都在抢 RX FIFO，把母机发的 TSTA 当摇杆帧读走，Test_WaitLoop 永远收不到。已改为 MODE==10 时接收由测试/AUTO 内部独占，外层 continue 跳过。
7. 测试退出后 MODE=9，再点「开始测试」子机不会再进等待循环。已加 g_tsta_pending 标志 + 外层识别 TSTA 后置 MODE=10。

## 需要你重点排查的方向

请逐行比对两端 NRF 配置，特别注意：

1. 寄存器初始化差异：母机 RF24 库默认配置 vs 子机 NRF24L01_Init/RX_Mode/TX_Mode 手写寄存器。重点看 CONFIG(0x00)、EN_AA、EN_RXADDR、SETUP_RETR、RF_CH、RF_SETUP、RX_PW_P0、SETUP_AW。
2. 地址方向：母机 openReadingPipe(1, tx_addr) 用的是 pipe 1 但地址和 writing pipe 相同；子机 RX_ADDRESS 和 TX_ADDRESS 也相同。检查 pipe 号与地址是否真正对上（RF24 库 pipe 1 的地址要求、子机 EN_RXADDR 是否只开了 pipe 0）。
3. 子机 SPI 速度：spi.c 里 NRF24L01_TxPacket 注释提到 SPI2_SetSpeed 被注释掉了，检查 SPI 分频是否超出 NRF24L01 的 10MHz 上限。
4. 子机 NRF24L01_RxPacket 的 STATUS 处理：读 STATUS 后立刻写回（清标志），再判断 RX_OK——这个顺序是否正确？会不会把刚收到的标志清掉？
5. 母机 radio.available() 的语义：在 startListening() 之后，RF24 库的 available() 需要一定时间才会返回 true；TestMode_Tick() 是 10ms 周期调用一次，是否够？
6. CSN/CE 引脚时序：母机 NRF_CE_PIN=47, NRF_CSN_PIN=18，子机是 SPI2 + PA4(CSN) + PA6(IRQ)。检查 CE 拉高/拉低的时机是否满足 NRF24L01 的 10us 要求。
7. 子机 Coop_SendRaw 的 TX/RX 切换：发 TSTB 时调 NRF24L01_TX_Mode() -> 发 -> NRF24L01_RX_Mode()。RX_Mode 里会 FLUSH_RX，如果母机此时正好发下一帧会被冲掉。但当前问题更早——TSTA 就没收到。
8. 子机是否真的执行到了 Test_WaitLoop：上电后 MODE=10，主循环第一轮就进 Test_WaitLoop。检查 printf 是否被 PI_LINK_EN 或串口初始化影响（USART1 是 printf 口，确认波特率 115200）。
9. 中断冲突：子机 HAL_UART_RxCpltCallback 挂在 USART2，但 PI_LINK_EN=0 不挂中断。检查是否有其他中断影响 SPI。
10. 供电：NRF24L01 需要 3.3V 且电流足够（发射瞬间约 115mA），AMS1117 供电不足会导致寄存器读写正常但发射失败。这是硬件问题，但请从代码上给出自检手段（比如读回 RF_SETUP/RF_CH 验证配置是否真的写进去了）。

## 请输出

1. 根因假设排序（按可能性从高到低），每条给出代码依据（文件名 + 行号 + 关键代码片段）
2. 验证方法：每个假设如何用最小改动验证（串口打印什么、读哪个寄存器）
3. 修复代码：直接给出可替换的函数或代码片段
4. 如果发现是我上面列的 7 个修复之一引入的新问题，请明确指出

## 文件说明

| 文件 | 内容 |
|---|---|
| 01_child_main.c | 子机 main.c 全文（1515 行）：协同协议、Test_WaitLoop、Adj_Move/Adj_Spin/Adj_Execute、主循环 |
| 02_child_nrf_driver.c | 子机 NRF24L01 寄存器直驱（spi.c + spi.h，原文件为 GBK 已转 UTF-8） |
| 03_host_nrf_test.cpp | 母机 nrf_sender / joypad_sender / test_mode（含 .h） |
| 04_host_web_server.cpp | 母机 web_server（WS 命令 0x30/0x31/0x32 分发、0x86/0x87 推送） |
| 05_host_config.h | 母机 config.h（引脚、信道、地址、超时） |

## 补充事实

- 子机 main.c 是 CubeMX 生成 + 手写业务，中文注释部分是作者手写，部分乱码（原文件混合编码）
- 母机编译 FQBN：esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
- 子机编译：arm-none-eabi-gcc，Makefile 在 firmware/pabi_F103rct6/Makefile
- 两端都已烧录最新固件，问题可稳定复现
- 正式协同（COOP/GOON/LOCK）此前是否成功过不确定，本次是第一次做双向通信验证
