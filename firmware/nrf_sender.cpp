#include "nrf_sender.h"
#include "config.h"
#include <Arduino.h>
#include <RF24.h>
#include <string.h>

static RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
static uint8_t tx_addr[5] = {NRF_TX_ADDR_0, NRF_TX_ADDR_1, NRF_TX_ADDR_2, NRF_TX_ADDR_3, NRF_TX_ADDR_4};

/* 状态跟踪: 只在真正切换时 flush, 避免连续调用把已收到的帧冲掉 */
static bool s_listening = false;

static bool sendOnce(const uint8_t* buf);

void NRF_Sender_Init(void) {
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, -1);
    if (!radio.begin()) {
        Serial.println("[NRF] 模块未找到!");
        return;
    }
    radio.setChannel(NRF_RF_CH);
    radio.setDataRate(RF24_2MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.setRetries(0, 3);
    radio.setPayloadSize(32);
    /* 子机 EN_RXADDR 只开 pipe0, 母机必须用 pipe0 收, 否则收不到回包 */
    radio.openReadingPipe(0, tx_addr);
    radio.openWritingPipe(tx_addr);
    radio.stopListening();
    s_listening = false;
    Serial.println("[NRF] PTX 模式就绪");
}

/* 固定 32 字节 payload: 发送前补齐, 避免长度不匹配 */
bool NRF_SendFrame(const uint8_t* data, uint8_t len) {
    uint8_t buf[32];
    uint8_t n = (len > 32) ? 32 : len;
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, n);
    return sendOnce(buf);
}

/* 带重试的发送: 每次都走完整的 stopListening->write->startListening,
   失败间隔 25ms 让子机从 TX 切回 RX 再发. 用于测试帧(TSTA/ADJ/TSTE). */
/* 单次发送: 时序对齐已验证的 test_nrf/host sendRaw()
 * 关键: 发送后只 flush_rx, 绝不 flush_tx — write() 内部 ce(LOW) 极短,
 * payload 可能仍在 TX FIFO 等 CE 再拉高; 此时 flush_tx 会把还没发出的帧冲掉 */
static bool sendOnce(const uint8_t* buf) {
    radio.stopListening();
    s_listening = false;
    radio.flush_tx();
    radio.clearStatusFlags();
    delay(2);
    bool ok = radio.write(buf, 32);
    radio.flush_rx();
    radio.startListening();
    s_listening = true;
    return ok;
}

bool NRF_SendFrameRetry(const uint8_t* data, uint8_t len, uint8_t retries) {
    uint8_t buf[32];
    uint8_t n = (len > 32) ? 32 : len;
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, n);
    for (uint8_t i = 0; i < retries; i++) {
        if (sendOnce(buf)) return true;
        delay(25);
    }
    return false;
}

void NRF_StartListening(void) {
    if (s_listening) return;
    radio.flush_rx();
    radio.startListening();
    s_listening = true;
}

void NRF_StopListening(void) {
    if (!s_listening) return;
    radio.stopListening();
    s_listening = false;
}

/* 固定 32 字节 payload: 不用 getDynamicPayloadSize (未启用时返回 0) */
bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len) {
    if (!radio.available()) return false;
    uint8_t n = radio.getPayloadSize();
    if (n == 0 || n > 32) n = 32;
    if (n > maxlen) n = maxlen;
    radio.read(buf, n);
    *out_len = n;
    return true;
}