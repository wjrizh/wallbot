#include "nrf_sender.h"
#include "config.h"
#include <Arduino.h>
#include <RF24.h>

static RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
static uint8_t tx_addr[5] = {NRF_TX_ADDR_0, NRF_TX_ADDR_1, NRF_TX_ADDR_2, NRF_TX_ADDR_3, NRF_TX_ADDR_4};

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
    Serial.println("[NRF] PTX 模式就绪");
}

bool NRF_SendFrame(const uint8_t* data, uint8_t len) {
    return radio.write(data, len);
}

void NRF_StartListening(void) {
    radio.flush_tx();
    radio.flush_rx();
    radio.startListening();
}

void NRF_StopListening(void) {
    radio.stopListening();
}

bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len) {
    if (!radio.available()) return false;
    *out_len = radio.getDynamicPayloadSize();
    if (*out_len == 0 || *out_len > 32) {
        radio.flush_rx();
        return false;
    }
    if (*out_len > maxlen) *out_len = maxlen;
    radio.read(buf, *out_len);
    return true;
}