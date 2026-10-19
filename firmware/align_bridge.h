/* align_bridge.h - 子机视觉纠偏闭环 (摄像头子机 /align/get -> 子机 NRF 指令)
 *
 * 由 App 按键触发 (WS 0x22 携带摄像头子机 IP), 母机周期拉取
 * http://<camip>/align/get, 按 phi/psi/axial 解算子机 speed/spin,
 * 经 ChildCmd 队列由 JoypadSender_Task 发往子机。
 *
 * 控制律 (参数见 config.h):
 *   spin  = clamp(-ALIGN_PSI_K   * psi_deg,  +/-ALIGN_SPIN_MAX)
 *   speed = clamp(-ALIGN_AXIAL_K * axial_mm / 10, +/-ALIGN_SPEED_MAX)
 *   |phi| > 容差时只告警 (子机无周向横移自由度), 不改控制量。
 */
#ifndef ALIGN_BRIDGE_H
#define ALIGN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

void AlignBridge_Init(void);
bool AlignBridge_Start(const uint8_t ip[4]);
void AlignBridge_Stop(void);
bool AlignBridge_IsActive(void);
void AlignBridge_Tick(void);

#endif /* ALIGN_BRIDGE_H */
