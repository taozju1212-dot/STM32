#ifndef __RFID_COMPAT_H
#define __RFID_COMPAT_H

#include "stm32f10x.h"

#ifndef BOOL
typedef unsigned char BOOL;
#endif

#ifndef TRUE
#define TRUE    (1)
#define FALSE   (0)
#endif

#ifndef u8
typedef unsigned char u8;
#endif

#ifndef u16
typedef unsigned short u16;
#endif

#ifndef s8
typedef signed char s8;
#endif

/* 函数接口 */
void RFID_Compat_Init(void);
void RFID_Var_Init(void);
BOOL RFID_RespondAnalysis(u8 data);
void RFID_SendResponse(u8 addr, u8 cmd, u8 sw, u8* data, u8 dataLen);
u8 Get_RFIDCheck(u8* ptr, u16 len);
char Get_Module_Ch(u8 idx);
void RFID_Switch_Channel(u8 internal_ch);

#endif
