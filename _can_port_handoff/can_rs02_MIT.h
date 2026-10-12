#ifndef CAN_RS02_H
#define CAN_RS02_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 数据结构 ==================== */

/**
 * @brief 电机反馈数据
 */
typedef struct {
    float    position_rad;   // 当前角度 (rad)
    float    velocity_rads;  // 当前速度 (rad/s)
    float    torque_nm;      // 当前力矩 (N.m)
    uint8_t  mode_state;     // 模式状态: 0=复位 1=标定 2=运行
    bool     has_fault;      // 是否有故障
    bool     has_warning;    // 是否有预警
    float    temp_c;         // 绕组温度 (℃)
} CANMotorFeedback;

/**
 * @brief 五次多项式轨迹规划器 (CAN电机专用, 角度单位为弧度)
 */
typedef struct {
    float    q0;          // 起始弧度
    float    qf;          // 目标弧度
    float    T;           // 执行总时间 (秒)
    uint32_t start_tick;  // 起始时间 (毫秒)
    bool     active;      // 是否正在运动
} CANMotorTraj;

/* 全局反馈缓存 (索引1=电机1, 2=电机2) */
extern float            can_rs02_feedback_rad[3];
extern CANMotorFeedback can_rs02_fb[3];

/* ==================== 接口函数 ==================== */

/**
 * @brief  CAN RS02 电机驱动初始化
 * @note   初始化 TWAI 外设 1Mbps, 自动执行设零/使能/锁定流程, 创建驱动任务
 *         应在 setup() 阶段调用
 */
void CAN_RS02_Init(void);

/**
 * @brief  使能指定电机 (MIT 指令1)
 */
void CAN_RS02_Enable(uint8_t id);

/**
 * @brief  停止指定电机 (MIT 指令2)
 */
void CAN_RS02_Disable(uint8_t id);

/**
 * @brief  设置指定电机机械零位 (MIT 指令4)
 */
void CAN_RS02_SetZero(uint8_t id);

/**
 * @brief  MIT 运控指令 (MIT 指令3)
 * @param  id        电机 ID
 * @param  pos_rad   目标角度 (rad)
 * @param  vel_rads  目标速度 (rad/s)
 * @param  kp        位置刚度
 * @param  kd        阻尼
 * @param  torq_nm   前馈力矩 (N.m)
 */
void CAN_RS02_MIT_Ctrl(uint8_t id, float pos_rad, float vel_rads,
                        float kp, float kd, float torq_nm);

/**
 * @brief  启动五次多项式轨迹
 * @param  id            电机 ID
 * @param  target_rad    目标弧度
 * @param  duration_sec  执行时长 (秒), 传0立即到位
 */
void CAN_RS02_StartTraj(uint8_t id, float target_rad, float duration_sec);

/**
 * @brief  查询轨迹是否正在执行
 */
bool CAN_RS02_IsMoving(uint8_t id);

/**
 * @brief  读取电机实时弧度反馈
 */
float CAN_RS02_GetPosition(uint8_t id);

/**
 * @brief  重置轨迹规划器到指定弧度
 * @param  id   电机 ID
 * @param  rad  目标弧度
 */
void CAN_RS02_ResetTraj(uint8_t id, float rad);

/**
 * @brief  CAN 电机驱动 FreeRTOS 任务 (50Hz)
 * @note   由 CAN_RS02_Init() 自动创建
 */
void Task_CANMotorDrive(void *pvParameters);

#endif /* CAN_RS02_H */