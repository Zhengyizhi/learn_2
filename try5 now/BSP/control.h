#ifndef __CONTROL_H
#define __CONTROL_H

#include "system.h"
#include "pid.h"
void Get_SystemMode(void);
void MechCondition(void);
void SentAllData_CAN1(void);
void sent_Motor(void);
void pitchgimbel(void);
void GyroCondition(void);//先机械模式
void smallGyrocondition(void);

enum system_mode
{	
	Error_Mode=0,
	Normal_Mode
};

#endif
