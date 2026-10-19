/* child_cmd.h - 子机运动指令直通队列 (WS/纠偏 -> NRF)
 *
 * 复用母机既有 32 字节摇杆帧格式, 不改子机协议:
 *   frame[0] = 0x00
 *   frame[1] = speed (int8, 前进为正)
 *   frame[2] = spin  (int8, 右转为正)
 *   frame[3] = 0
 *   frame[4] = 0
 *   frame[5] = key   (9=手动 10=自动 5=存吸盘值 其它=风机/吸盘档位)
 *   frame[6..31] = 0
 *
 * 语义: 单条待发帧 (latest value), 由 JoypadSender_Task 每周期取走并发送,
 *       调用方负责持续下发 (纠偏任务 500ms 一次, App 按键 setInterval)。
 */
#ifndef CHILD_CMD_H
#define CHILD_CMD_H

#include <stdint.h>
#include <stdbool.h>

void ChildCmd_Init(void);
bool ChildCmd_Set(int8_t speed, int8_t spin, uint8_t key);
bool ChildCmd_SetRaw(const uint8_t frame[32]);
bool ChildCmd_Pop(uint8_t out[32]);
void ChildCmd_Clear(void);
bool ChildCmd_Pending(void);

#endif /* CHILD_CMD_H */
