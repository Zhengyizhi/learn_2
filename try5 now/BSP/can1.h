#ifndef __CAN1_H
#define __CAN1_H

#include "system.h"

void CAN1_Init(void);
u8 CAN1_Send_Msg_chassis(u8* msg,u8 len);
u8 CAN1_Send_Msg_crable(u8* msg,u8 len);
void AngleAnaly(char alldatanum,float jump,float center,float degree);
u8 SpeedAnaly(u16 id,u8 *Recbuf);
#endif

