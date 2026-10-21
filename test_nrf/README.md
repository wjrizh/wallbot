# NRF24L01 子母机信号最小测试

只验证一件事：**母机发的帧，子机收得到吗？子机回的帧，母机收得到吗？**

不复用任何协同/纠偏代码，两边都是独立最小固件。

## 结构

```
test_nrf/
├── child/
│   ├── main_test.c     子机固件：收到 "PING" 帧回 "PONG:"+原内容，其他帧忽略
│   └── build.sh        借用 pabi_F103rct6 的 Drivers 编译
└── host/
    └── nrf_ping/
        └── nrf_ping.ino  母机固件：内嵌网页，点按钮发 PING
```

## 一、烧录子机

```sh
cd /home/z/wallbot-repo && ./test_nrf/child/build.sh
# 输出: /tmp/nrf_test_build/build/pabi_F103rct6.bin
st-flash --reset write /tmp/nrf_test_build/build/pabi_F103rct6.bin 0x8000000
```

子机串口（USART1, 115200）应打印：

```
=== NRF24L01 CHILD PING TEST ===
NRF24L01_Check = 0 (OK)
CONFIG=0F RF_CH=40 STATUS=0E
RX mode ready, waiting for PING...
```

若 `NRF24L01_Check` 不是 0，说明子机 NRF 模块没接好/没供电，先解决硬件。

## 二、烧录母机

```sh
rm -rf /tmp/nrfp && mkdir -p /tmp/nrfp/nrf_ping
cp /home/z/wallbot-repo/test_nrf/host/nrf_ping/nrf_ping.ino /tmp/nrfp/nrf_ping/
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --output-dir /tmp/nrfp/out /tmp/nrfp/nrf_ping
arduino-cli upload -p /dev/ttyACM0 \
  --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --input-dir /tmp/nrfp/out /tmp/nrfp/nrf_ping
```

母机串口（115200）会打印 WiFi 连上后的访问地址：

```
[NRF] ok
[NRF] chipConnected=1
[WiFi] http://192.168.x.x/
[HTTP] started
```

## 三、网页测试

1. 平板/手机连到 `robot` 热点（**开放热点，无密码**）
2. 浏览器打开 `http://192.168.4.1/`
3. 点「发送一次 PING」

### 判读

| 页面显示 | 含义 | 结论 |
|---|---|---|
| 模块状态=就绪，已发送+1，已收到+1，RTT 有几 ms | 双向通 | NRF 链路没问题，之前的 bug 在软件逻辑 |
| 已发送+1，已收到 0，丢包+1 | 母机发出去了，子机没回 | 子机没收到，或子机回不出去 |
| 模块状态=未找到 | 母机 NRF 没接好 | 检查 CE=47/CSN=18/SPI 引脚 |
| 点按钮没反应 | 网页没连上母机 | 检查 WiFi |

失败时日志会打印 `wb` / `sb` 与分段耗时：

- `wb=0`：TX FIFO 满，发送未发起
- `wb=1 sb=0`：已发但未收到硬件 ACK（子机未收 或 母机未在接收窗口）
- `wb=1 sb=1`：已收 ACK 但 300ms 内无 PONG
- `(write A + rx B) it=N maxAvail=Y maxDelay=Z`：A=发送耗时，B=等回包耗时，N=轮询迭代数

## 本次定位的根因（已修）

1. **子机 PA6（NRF IRQ）被 CubeMX 配成 OUTPUT_PP**，应 INPUT。`NRF24L01_TxPacket` 里
   `while(ReadPin(PA6)!=0)` 立即退出，TX 时序错乱。测试固件在 main 里重配为 INPUT+PULLUP。
2. **子机 `NRF24L01_RxPacket` 空读假阳性**：SPI 空 FIFO 读回 `0xFF`，`0xFF & RX_OK` 恒真，
   子机把空读当收包并狂发 PONG 自激（GDB 实测 `rxCount=9631`）。修复：`safe_RxPacket()`
   先查 IRQ 引脚 + STATUS.RX_DR + FIFO_STATUS.RX_EMPTY，并只应答 `PING` 帧。
3. **母机用 `writeBlocking()+txStandBy()` 阻塞等 ACK 200ms**，期间停在 PTX 不收包，
   子机回包全丢（`OBSERVE_TX.ARC_CNT=10`）。改为原协同时序：`write()` 后立即
   `flush_rx()+startListening()` 开接收窗口。
4. **偶发 2000ms 延迟**：ESP32-S3 USB CDC 无主机读取时 `Serial.printf` 阻塞约 2s。
   修复：`Serial.setTxTimeoutMs(0)`。

### 对照子机串口

- 子机打印 `[RX #1] ...` + `[TX] status=20 (TX_OK)` → 子机收到并回了，母机没收到 → 母机接收问题
- 子机毫无反应 → 帧根本没到子机 → 发送端或空中链路问题

## 关键配置（两端必须一致）

| 项 | 值 |
|---|---|
| 信道 | 40 |
| 地址 | `11 22 33 44 55` |
| payload | 固定 32 字节 |
| 速率 | 2Mbps |
| CRC | 16 位 |
| 自动应答 | 开启 |

母机监听用 **pipe 0**（`radio.openReadingPipe(0, ADDR)`），因为子机 `EN_RXADDR` 只开了 pipe 0。原协同代码用的是 pipe 1，这可能是之前收不到回包的原因之一。

## 测完怎么恢复

这两个测试固件不碰原工程。重新烧录原固件即可：

```sh
# 子机
cd /home/z/wallbot-repo/firmware/pabi_F103rct6 && make -j4
st-flash --reset write build/pabi_F103rct6.bin 0x8000000

# 母机
cd /home/z/wallbot-repo && cp firmware/*.cpp firmware/*.h /tmp/wbfw/wallbot_fw/ \
  && cp firmware/calibration_test.ino /tmp/wbfw/wallbot_fw/wallbot_fw.ino
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --output-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
arduino-cli upload -p /dev/ttyACM0 \
  --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```
