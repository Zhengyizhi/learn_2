#ifndef __pid_H
#define __pid_H
#define PIDcontrol_chassis1 0
#define PIDcontrol_chassis2 2
#define PIDcontrol_chassis3 4
#define PIDcontrol_chassis4 6
#define PIDcontrol_gyro_condition 8
#define PIDcontrol_gyrospeed_downgimble 11
#define PIDcontrol_gyrospeed_upgimble 12

#define PIDcontrol_mechangle_downgimble 13
#define PIDcontrol_mechangle_upgimble 14
#define PIDcontrol_gyroangle_downgimble 15
#define PIDcontrol_gyroangle_upgimble 16
#define PIDcontrol_mechspeed_Dial 17
#define PIDcontrol_mechangle_Dial 18
#include "system.h"
typedef struct _PID{//全部设成 float 不会爆掉
	float SetGoal;//目标值
	float ActualVar;//实际值
	float err;
	float I_err;//Ki 的误差累积
	float err_pre;//上一个误差
	float nextSet;//下一个需要输出的
	float Kp;
	float Ki;
	float Kd;
	float KPout;
	float KIout;
	float KDout;
	
	float I_err_limit;//累加保护
  float nextSet_limit;//输出保护
	float blind_limit;//盲区大小
}_PID;
typedef struct _Total_Pid{
  _PID TurnTest[20];//9，10速度机械环；11，12速度陀螺仪；13，14角度机械环；15，16角度陀螺仪
}_Total_Pid;//调参的结构体里带八个PID

float pid_handle1(_Total_Pid* p,char i,float SetGoal,float ActualVar);
float pid_handle2(_Total_Pid* p,char i,float err);
void PID_Init (_Total_Pid* p);
void PID_Init_17_18 (_Total_Pid* p);

#endif

