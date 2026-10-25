#ifndef ALIGN_TEST_H
#define ALIGN_TEST_H
#include <stdint.h>
#include <stdbool.h>

/* Standalone vision-align test (App 0x30/0x32)
 * Mother->Child: TSTA / ADJ,... / GOON / TSTE
 * Child->Mother: REDY (after TSTA, and after each ADJ step)
 * Vision: reuse CoopVision; share 0x88/0x89/0x24 with coop
 * Mutex with AutoMode (0x19)
 */

void AlignTest_Init(void);
bool AlignTest_IsActive(void);
void AlignTest_Start(void);
void AlignTest_Stop(void);
void AlignTest_Tick(void);

void AlignTest_FillVisionStatus(uint8_t out[11]);

bool AlignTest_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
void AlignTest_OnFrameSent(void);

#endif
