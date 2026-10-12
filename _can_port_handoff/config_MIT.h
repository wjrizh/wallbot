#ifndef CONFIG_H
#define CONFIG_H

/* ============================================================
 * 激光标定测试 + NRF子母协同 — 全局配置文件
 * ============================================================ */

/* 1. 舵机 PWM */
#define SERVO_PWM_FREQ         50
#define SERVO_PWM_RESOLUTION   14
#define SERVO_TASK_PERIOD_MS   20
#define SERVO_ANGLE_MIN        0.0f
#define SERVO_ANGLE_MAX        180.0f
#define SERVO_DEFAULT_DURATION 2.0f
#define SERVO_M1_PIN   9
#define SERVO_M2_PIN   10
#define SERVO_M3_PIN   12
#define SERVO_PB9_PIN  21

/* 2. I2C */
#define I2C_SDA_PIN           4
#define I2C_SCL_PIN           5
#define I2C_FREQ              400000
#define TCA9548A_ADDR         0x70

#define LASER_CH0             0
#define LASER_CH1             1
#define LASER_CH2             2
#define LASER_CH3             3

/* 3. 激光 VL53L0X */
#define LASER_FILTER_SIZE     5
#define LASER_SAMPLE_MS       200
#define LASER_OVER_RANGE      8190

#define KF_DEFAULT_E_MEA      0.02f
#define KF_DEFAULT_E_EST      10.0f
#define KF_DEFAULT_Q          0.001f

#define LASER_CH0_OFFSET     -3
#define LASER_CH1_OFFSET    -34
#define LASER_CH2_OFFSET      0
#define LASER_CH3_OFFSET      0

/* 4. 运动学 */
#define KIN_L1                220.0f
#define KIN_L2                220.0f
#define KIN_LIFT_H            135.0f
#define KIN_BASE_M1           30.0f
#define KIN_BASE_M2           85.0f
#define KIN_BASE_M3           25.0f

/* 5. 步态 */
#define GAIT_MOVE_TIME_MS     3000
#define GAIT_ROT_TIME_MS      10000
#define GAIT_PUMP_TIMEOUT_MS  5000
#define GAIT_STEP_DELAY_MS    10

/* 6. 空中标定 */
#define CALIB_AIR_MOVE_DUR        4.0f
#define CALIB_AIR_WAIT_MS         2000
#define CALIB_AIR_STABLE_TIMEOUT_MS  8000
#define CALIB_AIR_STABLE_SIGMA       1.5f

/* 7. RS02 PID 修正 */
#define RS02_CORRECT_DEADZONE      0.002f
#define RS02_CORRECT_MAX_STEP      0.05f
#define RS02_CORRECT_TIMEOUT_MS    20000

/* 8. 自动纠偏 */
#define AA_K_SYS              80.0f
#define AA_STEP_P             0.8f
#define AA_STEP_BASE          0.003f
#define AA_STEP_MIN           0.0005f
#define AA_STEP_MAX           0.10f
#define AA_IMU_DEADZONE       0.5f
#define AA_CONVERGE_DEADZONE  1.0f
#define AA_STABLE_SIGMA       1.2f
#define AA_STABLE_TIMEOUT_MS  5000
#define AA_MAX_ITER           12
#define AA_LIFT_DUR           4.0f
#define AA_RS02_DUR           2.0f
#define AA_RS02_WAIT_MS       1500
#define AA_HOME_DUR           4.0f

/* 9. 真空泵 */
#define PUMP1_PIN   8
#define PUMP2_PIN   11
#define PUMP_ON     HIGH
#define PUMP_OFF    LOW

/* 10. CAN RS02 */
#define CAN_RS02_TX_PIN        6
#define CAN_RS02_RX_PIN        7
#define CAN_RS02_TASK_PERIOD_MS 20
#define CAN_RS02_RX_QUEUE_LEN  50
#define CAN_RS02_LOCK_KP       200.0f
#define CAN_RS02_LOCK_KD       4.0f
#define CAN_RS02_LOCK_VEL      0.0f
#define CAN_RS02_LOCK_TORQ     0.0f

/* 11. WiFi */
#define WIFI_SSID             "CalibTest"
#define WIFI_PASSWORD         "12345678"
#define CHILD_IP              "192.168.4.2"
#define GRIPPER_IP            "192.168.4.3"

/* 12. NRF24L01 */
#define NRF_CE_PIN            47
#define NRF_CSN_PIN           18
#define NRF_SCK_PIN           15
#define NRF_MOSI_PIN          13
#define NRF_MISO_PIN          14
#define NRF_IRQ_PIN           48
#define NRF_RF_CH             40
#define NRF_TX_ADDR_0         0x11
#define NRF_TX_ADDR_1         0x22
#define NRF_TX_ADDR_2         0x33
#define NRF_TX_ADDR_3         0x44
#define NRF_TX_ADDR_4         0x55

/* 13. 虚拟手柄 */
#define JOYPAD_CH_LH          0
#define JOYPAD_CH_LV          1
#define JOYPAD_CH_RH          2
#define JOYPAD_CH_RV          3
#define JOYPAD_SEND_PERIOD_MS 10

#endif