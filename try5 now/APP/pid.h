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
typedef struct _PID{//ȫ����� float ���ᱬ��
	float SetGoal;//Ŀ��ֵ
	float ActualVar;//ʵ��ֵ
	float err;
	float I_err;//Ki ������ۻ�
	float err_pre;//��һ�����
	float nextSet;//��һ����Ҫ�����
	float Kp;
	float Ki;
	float Kd;
	float KPout;
	float KIout;
	float KDout;
	
	float I_err_limit;//�ۼӱ���
  float nextSet_limit;//�������
	float blind_limit;//ä����С
}_PID;
typedef struct _Total_Pid{
  _PID TurnTest[20];//9��10�ٶȻ�е����11��12�ٶ������ǣ�13��14�ǶȻ�е����15��16�Ƕ�������
}_Total_Pid;//���εĽṹ������˸�PID

float pid_handle1(_Total_Pid* p,char i,float SetGoal,float ActualVar);
float pid_handle2(_Total_Pid* p,char i,float err);
void PID_Init (_Total_Pid* p);
void PID_Init_17_18 (_Total_Pid* p);

#endif

