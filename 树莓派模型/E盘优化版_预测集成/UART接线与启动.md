# 机载串口 AE（Pi 常驻）

## 接线（4 根；若 5V/GND 已供电则再加 2 根串口）

| 子机 | 树莓派 Zero 2W |
|------|----------------|
| 5V 供电输出 | Pin 2 或 4（5V） |
| GND | Pin 6（GND） |
| PA9 USART1_TX | Pin 10 / GPIO15 RX |
| PA10 USART1_RX | Pin 8 / GPIO14 TX |

共地；UART 为板间 3.3V 直连，不接电脑、不接 USB-TTL。

Pi 侧建议：`/boot/config.txt` 中 `enable_uart=1`，并关掉串口登录（`raspi-config` → Serial → login shell 关、硬件串口开）。设备一般是 `/dev/serial0`。

## 启动

```bash
cd /path/to/E盘优化版_预测集成
pip3 install pyserial   # 若未装
# 可先保留网页调试; 正式用常驻服务:
python3 ae_uart_service.py
```

或改 `ae_uart.service` 里路径后：

```bash
sudo cp ae_uart.service /etc/systemd/system/
sudo systemctl enable --now ae_uart.service
```

## 协议

- 子机在探头下压后发：`CAP\n`
- Pi 回：`OK CAP\n`，采集+预测后发：`R,cm=23.5,near=20,pd=1,st=ok\n`
- 子机经 NRF 转母机，App 显示预测 cm 与标准点
