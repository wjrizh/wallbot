/* ==========================================================================
 * 05_host_config.h  --  host global config
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * FILE: firmware/config.h
 * -------------------------------------------------------------------------- */

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
#define SERVO_GRIP_PIN 38
#define GRIPPER_HOME_DEG 130.0f

/* 3. 运动学 */
#define KIN_L1                220.0f
#define KIN_L2                220.0f
#define KIN_LIFT_H            135.0f
#define KIN_BASE_M1           60.0f
#define KIN_BASE_M2           110.0f
#define KIN_BASE_M3           35.0f

/* 5. 步态 */
#define GAIT_MOVE_TIME_MS     3000
#define GAIT_ROT_TIME_MS      10000
#define GAIT_PUMP_TIMEOUT_MS  5000
#define GAIT_STEP_DELAY_MS    10

/* 6. 舵机运动时长 */
#define CALIB_AIR_MOVE_DUR        2.5f
#define CALIB_FAST_MOVE_DUR       0.5f   /* 远伸/复位小姿势快动作 */
#define CALIB_FAST_WAIT_MS        800    /* 小姿势死等(0.5s动作+0.3s冗余) */

/* 7. RS02 PID 修正 */
#define RS02_CORRECT_DEADZONE      0.002f
#define RS02_CORRECT_MAX_STEP      0.05f
#define RS02_CORRECT_TIMEOUT_MS    20000
#define RS02_SETTLE_MS             250

/* 8. 自动纠偏 */
#define AA_K_SYS              80.0f
#define AA_STEP_P             1.2f
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

/* 9.5 IMU (JY62, 有线串口, 主控板预留 IMU UART) */
#define IMU_RX_PIN             40
#define IMU_TX_PIN             39
#define IMU_BAUD               115200
#define IMU_FILTER_ALPHA       0.25f

/* 10. CAN RS02 */
#define CAN_RS02_TX_PIN        6
#define CAN_RS02_RX_PIN        7
#define CAN_RS02_TASK_PERIOD_MS 20
#define CAN_RS02_RX_QUEUE_LEN  50
#define CAN_RS02_LOCK_KP       500.0f
#define CAN_RS02_LOCK_KD       5.0f
#define CAN_RS02_LOCK_VEL      0.0f
#define CAN_RS02_LOCK_TORQ     0.0f

/* 11. WiFi (STA: 连接平板 robot 热点) */
#define WIFI_SSID             "robot"
#define WIFI_PASSWORD         "12345678"
#define WIFI_STATIC_IP        10, 32, 7, 216
#define WIFI_STATIC_GW        10, 32, 7, 1
#define WIFI_STATIC_MASK      255, 255, 255, 0
#define WIFI_STATIC_DNS       10, 32, 7, 1
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

/* 14. 子机指令直通 + 视觉纠偏闭环 */
#define ALIGN_BRIDGE_PERIOD_MS   500    /* 拉取 /align/get 周期 */
#define ALIGN_BRIDGE_TIMEOUT_MS  60000  /* 闭环总超时 */
#define ALIGN_HTTP_TIMEOUT_MS    1500
#define ALIGN_PSI_K              0.60f  /* spin = -K * psi_deg */
#define ALIGN_AXIAL_K            0.80f  /* speed = -K * axial_mm/10 */
#define ALIGN_PSI_DEAD_DEG       3.0f   /* 死区 (夹爪 5° 留裕量) */
#define ALIGN_AXIAL_DEAD_MM      8.0f
#define ALIGN_SPIN_MAX           60     /* spin 限幅 (子机 int8 指令域) */
#define ALIGN_SPEED_MAX          60

/* #define GAIT_SKIP_IMU */  /* 已启用 IMU 步态纠偏 */

/* ===== 尺蠖步态几何 (新机器标定参数, 换机只改这里) ===== */
#define KIN_ARM_LEN        220.0f   /* L1=L2 臂长 (mm) */
#define KIN_LEG_LEN        135.0f   /* M1/M3 垂直脚杆 (mm) */
#define KIN_RATIO_M1       2.0f     /* M1 舵机:几何角 传动比 */
#define KIN_RATIO_M3       2.0f     /* M3 传动比 */
#define KIN_RATIO_M2       1.0f     /* M2 传动比 */
#define KIN_ZERO_M1        10.0f    /* θ1=0(臂水平) 时 M1 舵机读数 */
#define KIN_ZERO_M3        (-5.0f)  /* θ2=0 时 M3 舵机读数 */
#define KIN_ZERO_M2        70.0f    /* 两臂共线(φ=180) 时 M2 舵机读数 */
#define KIN_STAND_THETA    20.0f    /* 站立臂角 (度) */
#define GAIT_STEP_SPEED    4.0f     /* 步态运动时长 (秒), 越大越慢 */

/* ===== 激光避障 ===== */
#define AVOID_STOP_MM      70
#define AVOID_STEP_MAX_MM  120
#define AVOID_STEP_MIN_MM  60
#define AVOID_STEP_FAR_MM  400
#define AVOID_SAMPLE_MS    3000
#define AVOID_SAMPLE_HZ    30
#define AVOID_WAIT_MS      1000
#define AVOID_TRIM         3

#endif

