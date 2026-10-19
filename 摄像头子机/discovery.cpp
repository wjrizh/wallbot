/* discovery.cpp - UDP 自动发现 (母机/App 广播询问, 本机应答角色+IP) */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <stdio.h>
#include "config.h"
#include "discovery.h"

static WiFiUDP g_udp;
static bool g_started = false;

void Discovery_Reset(void) {
    g_udp.stop();
    g_started = false;
}

void Discovery_Poll(void) {
    if (!g_started) {
        if (WiFi.status() != WL_CONNECTED) return;
        if (!g_udp.begin(DISCOVERY_PORT)) {
            Serial.printf("[UDP] begin :%u failed\n", DISCOVERY_PORT);
            return;
        }
        g_started = true;
        Serial.printf("[UDP] discovery listen :%u\n", DISCOVERY_PORT);
    }

    int size = g_udp.parsePacket();
    if (size <= 0) return;

    char rx[64];
    if (size >= (int)sizeof(rx)) { g_udp.flush(); return; }
    int len = g_udp.read(rx, sizeof(rx) - 1);
    if (len <= 0) return;
    rx[len] = 0;
    if (strncmp(rx, DISCOVERY_REQ, strlen(DISCOVERY_REQ)) != 0) return;

    IPAddress rip = g_udp.remoteIP();
    uint16_t rport = g_udp.remotePort();
    char reply[48];
    int n = snprintf(reply, sizeof(reply), "WALLBOT:%s:%s", DEVICE_ROLE,
                     WiFi.localIP().toString().c_str());
    g_udp.beginPacket(rip, rport);
    g_udp.write((const uint8_t *)reply, n);
    g_udp.endPacket();
    Serial.printf("[UDP] reply %s -> %s\n", reply, rip.toString().c_str());
}
