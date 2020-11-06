#ifndef __CAN2_H__
#define __CAN2_H__

#include "system.h"

void CAN2_Init(void);
u8 CAN2_Send_Dial(u8* msg,u8 len);//·¢ËÍ
u8 SpeedAnalyCAN2(u16 id,u8 *Recbuf);

#endif 
