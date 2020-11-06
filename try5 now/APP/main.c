#include "system.h"
uint32_t Remote_time = 0;
extern int SystemMonitor;
extern _Total_Pid PID_control;
extern _TotalFlag AllFlag;
extern _Total_Receive Alldata;
float see1,see2,see3,see4;
extern _remoteContrl remote_control;
int main(void)
{
	
//	RC_Init();//遥控器  中断
	//CAN1_Init();//接受中断
//	Led_Init();
	PID_Init(&PID_control);
	PID_Init_17_18 (&PID_control);
	
	TIM3_PWM_Init(2500-1,84-1);//2.5ms摩擦轮 1600
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	TIM1_PWM_Init(20000-1,84-1);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	
	System_Init();
	Remote_time = micros();
//	AllFlag.justoneDial_Flag = open;//只改变一次
	while(1)
	{
		{
		if((micros() >= Remote_time))//如果遥控器一段时间内没有收到信息，强行将遥控器复位为没有遥控的状态
		{
			RC_Ctl.rc.ch0 = 0;
			RC_Ctl.rc.ch1 = 0;
			RC_Ctl.rc.ch2 = 0;
			RC_Ctl.rc.ch3 = 0;
			RC_Ctl.rc.s1 = 2;
			RC_Ctl.rc.s2 = 2;
			Stop();
			SystemMonitor = Error_Mode;			
			delay_us(10);
		}
		else
			SystemMonitor = Normal_Mode;
		}
		see1 = Alldata.Redata[6].longtimeangle;
		see2 = Alldata.Redata[7].longtimeangle;
		see3 = remote_control.control_speed[2];
		see4 = remote_control.control_speed[3];
		Loop();
	}
}


