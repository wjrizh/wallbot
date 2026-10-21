#include "nrf_sender.h"
#include "config.h"
#include <Arduino.h>
#include <RF24.h>
#include <string.h>

static RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
static uint8_t tx_addr[5] = {NRF_TX_ADDR_0, NRF_TX_ADDR_1, NRF_TX_ADDR_2, NRF_TX_ADDR_3, NRF_TX_ADDR_4};

/* 状态跟踪: 只在真正切换时 flush, 避免连续调用把已收到的帧冲掉 */
static bool s_listening = false;

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
    radio.openReadingPipe(1, tx_addr);
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
    return radio.write(buf, 32);
}

void NRF_StartListening(void) {
    if (s_listening) return;
    radio.flush_tx();
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