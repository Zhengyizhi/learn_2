#include "system.h"

void Key_control()
{
	Key_Initcondition();
	Key_GyroCondition();
	Key_shoot();
	Key_mechgimbel();
	Key_mechcondition();
	Key_SteerEngine_condition();
	Key_Smallgyro_condition();
}



//mech
//				PID_Init(&PID_control);
//				Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;
//				KeyFlag.justonemech_Flag = close;
//				Alldata.Redata[4].longtimecircle = 0;
//				KeyFlag.justonegyro_Flag = open;
//				KeyFlag.justonesmallgyro_Flag = open;
//				Alldata.Redata[6].longtimecircle = 0;
//				Alldata.Redata[7].longtimecircle = 0;
//				Alldata.Redata[6].longtimeangle = 0;
//				Alldata.Redata[7].longtimeangle = 0;
//				TurnAround = 0;
//				Key_Turnforward = 0;


//gyro
//						PID_Init(&PID_control);
//						Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;//Ҫ��¼ת��Ȧ��
//					//	AllFlag.justonegyro_Flag = close;
//						KeyFlag.justonegyro_Flag = close;
//						centeryaw = yaw;
//						Alldata.Redata[6].longtimecircle = 0;
//						Alldata.Redata[4].longtimecircle = 0;//ң����Ŀ��ת��
//						Alldata.Redata[7].longtimecircle = 0;//ʵ��ת��
//						Alldata.Redata[6].preAngle = centeryaw;
//						Alldata.Redata[7].preAngle = centeryaw;
//						Alldata.Redata[6].longtimeangle = 0;
//						Alldata.Redata[7].longtimeangle = 0;		
//						Alldata.Redata[4].longtimeangle = 0;//ң����Ŀ��ת��
//						remote_control.control_downgimbelyawangle = 0;
//						remote_control.control_angleyaw_down = 0;
//						TurnAround = 0;
//						Key_Turnforward = 0;
//						KeyFlag.justonemech_Flag = open;
//						KeyFlag.justonesmallgyro_Flag = open;
