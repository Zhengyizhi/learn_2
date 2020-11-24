#include "system.h"

uint32_t Remote_time = 0;
extern int SystemMonitor;
float see1,see2,see3,see4,see5,see6;
float seeP,seeI,seeD;
float seeerror,seefeedback,seetarget,seedetaerror,seeerror,seeout,seeintegrate;
int main(void)
{
	System_Init();
	PID_Init (&Total_pid);
	control_circle_init();
	Init_kal();
//	control_circle_init();
	Remote_time = micros();
//  Wait_stand_init();
	while(1)
	{
		Loop();
		see1 = Chassis_Moto_Info_Array[0].NowSpeed;
		see2 =  Chassis_Moto_Info_Array[1].NowSpeed;
		
		see3 = stand_Speed2;
		see4 = speedcircle_speed2;
		
		see5= Chassis_Moto_Info_Array[0].NowCurrent;
		see6 = Chassis_Moto_Info_Array[1].NowCurrent;
		
		seeP = Total_pid.pid_test_Array[3].Pout;
		seeI = Total_pid.pid_test_Array[3].Iout;
		seeD = Total_pid.pid_test_Array[3].Dout;
		
		seeerror = Total_pid.pid_test_Array[3].error;
		seefeedback = Total_pid.pid_test_Array[3].feedback;
		seetarget = Total_pid.pid_test_Array[3].target ;
		seeout = Total_pid.pid_test_Array[3].out;
		seeintegrate = Total_pid.pid_test_Array[3].integrate;
	}
}


