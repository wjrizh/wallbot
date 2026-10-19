# 母机分区表备份与恢复

烧录日期: 2026-10-19
硬件: ESP32-S3 (QFN56 rev v0.2), 实测 **16MB Flash + 8MB OPI PSRAM**
(esptool flash_id: Manufacturer 46, Device 4018)

## 变更原因

原固件使用默认 4MB 分区 (`PartitionScheme=default`), APP 分区仅 1.2MB。
加入「子机指令直通 + 视觉纠偏闭环」后 sketch 占用 1249596 / 1310720 字节 = **95%**,
再增加任何功能都会编译失败。

改用 16MB 分区方案 `app3M_fat9M_16MB`, APP 分区 3MB, 占用降至 **39%**。

## 当前 FQBN

```
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
```

## 分区布局 (app3M_fat9M_16MB.csv)

| Name | Type | SubType | Offset | Size |
|---|---|---|---|---|
| nvs | data | nvs | 0x9000 | 0x5000 (20K) |
| otadata | data | ota | 0xe000 | 0x2000 (8K) |
| app0 | app | ota_0 | 0x10000 | 0x300000 (3M) |
| app1 | app | ota_1 | 0x310000 | 0x300000 (3M) |
| ffat | data | fat | 0x610000 | 0x9E0000 (9.875M) |
| coredump | data | coredump | 0xFF0000 | 0x10000 (64K) |

## 恢复方法

### 正常升级 (推荐)

```sh
arduino-cli upload -p /dev/ttyACM0 \
  --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB' \
  --input-dir /tmp/wbfw/out /tmp/wbfw/wallbot_fw
```

`arduino-cli upload` 会依次写 bootloader(0x0) + 分区表(0x8000) + boot_app0(0xe000) + app(0x10000)。

### 整片恢复 (分区表被破坏 / 板子起不来)

```sh
esptool --port /dev/ttyACM0 write_flash 0x0 wallbot_fw.ino.merged.bin
```

`merged.bin` 是 16MB 完整镜像, 一次写完所有分区, 之后直接能跑。

### 单写分区表 (分区表错但 bootloader 完好)

```sh
esptool --port /dev/ttyACM0 write_flash 0x8000 docs/partition-backup/host_app3M_fat9M_16MB.partitions.bin
esptool --port /dev/ttyACM0 write_flash 0xe000 docs/partition-backup/host_boot_app0.bin
```

### 退回旧的 4MB 分区

把 FQBN 里的 `,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB` 整段删掉, 重新编译烧录即可。
注意此时 APP 只剩 1.2MB, 当前固件 (1.25MB) **会编译失败**, 需要先精简功能。

## 备份文件

| 文件 | 说明 |
|---|---|
| `app3M_fat9M_16MB.csv` | 分区表源文件 (来自 ESP32 core 3.3.11) |
| `host_app3M_fat9M_16MB.partitions.bin` | 编译产出的分区表二进制, 烧写到 0x8000 |
| `host_bootloader.bin` | 编译产出的 bootloader, 烧写到 0x0 |

## 串口验证

烧录后串口应输出:

```
=== 激光标定测试工具 + NRF子母协同 ===
[CAN] TWAI 1Mbps 扩展帧 启动成功
[WiFi] scan: N networks
[WiFi] STA: ..., IP: ..., GW: ..., MASK: ...
[Web] HTTP + WS 二进制协议 已启动
[UDP] discovery listen :48899
[MAIN] 全部就绪
```
