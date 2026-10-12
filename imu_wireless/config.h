#ifndef CONFIG_H
#define CONFIG_H

#define IMU_RX_PIN    16
#define IMU_TX_PIN    17
#define IMU_BAUD      115200
#define IMU_FILTER_ALPHA  0.25f

#define WIFI_SSID     "CalibTest"
#define WIFI_PASSWORD "12345678"
#define IMU_STATIC_IP IPAddress(192,168,4,55)

#endif