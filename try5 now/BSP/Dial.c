#include "Dial.h"
#include "CAN2.h"
extern _TotalFlag AllFlag;
extern  _Total_Receive Alldata;

short int Store_Dial_speed;//sentmotor();
short int Store_Dial_angle;

extern _Total_Pid PID_control;

////////
float detadial_angle;//机械模式
float firstdial_angle;//不能改变
float unshoot;
float longtime;


float tyrtyr=30;
void Dial_smallgyro_comlattice()
{
	if(AllFlag.justoneDialsmallgyro_Flag == open)
	{
		PID_Init_17_18 (&PID_control);
		Alldata.Redata[8].preAngle = Alldata.Redata[8].NowAngle;
		firstdial_angle = Alldata.Redata[8].preAngle;
		Alldata.Redata[8].longtimeangle = 0;
		Alldata.Redata[8].longtimecircle = 0;
		detadial_angle = 0;
		unshoot = 0;
		AllFlag.justoneDialsmallgyro_Flag = close;
		AllFlag.justoneDialmech_Flag = open;
		AllFlag.justoneDialgyro_Flag = open;
		
	}
	AngleAnaly(8,5000,firstdial_angle,8191);
	longtime = Alldata.Redata[8].longtimeangle*0.044f;
	if(AllFlag.Dial_level_smallgyro == level2)
	{
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f)<=13)
		{
			detadial_angle -=1080;
			unshoot = 0;
		}
		
		
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f)>=20)
		{
			unshoot++;
			if(unshoot >= 300)
			{
				while(detadial_angle <= Alldata.Redata[8].longtimeangle*0.044f)
					detadial_angle+=2380;
			}
		}
	}
	Store_Dial_angle = (short int)(pid_handle1(&PID_control,PIDcontrol_mechangle_Dial,-detadial_angle,-Alldata.Redata[8].longtimeangle*0.044f));//角度环
	Store_Dial_speed = (short int)(pid_handle1(&PID_control,PIDcontrol_mechspeed_Dial,Store_Dial_angle,Alldata.Redata[8].NowSpeed));
}



void Dial_mech_onelattice()
{
	if(AllFlag.justoneDialmech_Flag == open)//且模式
	{
		PID_Init_17_18 (&PID_control);
		Alldata.Redata[8].preAngle = Alldata.Redata[8].NowAngle;
		firstdial_angle = Alldata.Redata[8].preAngle;
		AllFlag.justoneDialmech_Flag = close;
		AllFlag.justoneDialgyro_Flag = open;
		AllFlag.justoneDialsmallgyro_Flag = open;
		Alldata.Redata[8].longtimeangle = 0;
		Alldata.Redata[8].longtimecircle = 0;
		detadial_angle = 0;
		unshoot = 0;
	}
	AngleAnaly(8,5000,firstdial_angle,8191);
	if(AllFlag.Dial_level_mech == level2 && AllFlag.justoneDialmechnumber == open)
	{
		AllFlag.justoneDialmechnumber = close;
		detadial_angle -=1080;//只做一次
	}
	if(AllFlag.Dial_level_mech == level1)
	{
		AllFlag.justoneDialmechnumber = open;
	}
	
	
	
	if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f) >= 20)
	{
		unshoot ++;
		if(unshoot>=300)//1s
		{
			while(detadial_angle <= Alldata.Redata[8].longtimeangle*0.044f)
				detadial_angle += 2380;
			unshoot = 0;
		}
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f) <= 10)
		{
			unshoot = 0;
		}
	}//卡弹处理
	
	Store_Dial_angle = (short int)(pid_handle1(&PID_control,PIDcontrol_mechangle_Dial,-detadial_angle,-Alldata.Redata[8].longtimeangle*0.044f));//角度环
	Store_Dial_speed = (short int)(pid_handle1(&PID_control,PIDcontrol_mechspeed_Dial,Store_Dial_angle,Alldata.Redata[8].NowSpeed));
}
float shoot;
extern int donumber_Dial_gyro;
void Dial_gyro_continuelattice()
{
	float speedd;
	if(AllFlag.justoneDialgyro_Flag == open)//且模式
	{
		PID_Init_17_18 (&PID_control);
		Alldata.Redata[8].preAngle = Alldata.Redata[8].NowAngle;
		firstdial_angle = Alldata.Redata[8].preAngle;
		AllFlag.justoneDialgyro_Flag = close;
		Alldata.Redata[8].longtimeangle = 0;
		Alldata.Redata[8].longtimecircle = 0;
		AllFlag.justoneDialmech_Flag = open;
		AllFlag.justoneDialsmallgyro_Flag = open;
		unshoot = 0;
		shoot = 0;
	}
	if(AllFlag.Dial_level_gyro == level2)
	{
		if(AllFlag.justoneDialgyronumber == open)
		{
			donumber_Dial_gyro++;
			AllFlag.justoneDialgyronumber = close;
		}
	}
	else if(AllFlag.Dial_level_gyro == level1)
	{
		AllFlag.justoneDialgyronumber = open;
	}
	
	if(donumber_Dial_gyro %2 ==0)
	{
		speedd = 0;
		Store_Dial_speed = (short int)(pid_handle1(&PID_control,PIDcontrol_mechspeed_Dial,speedd,Alldata.Redata[8].NowSpeed));
	}
	else if(donumber_Dial_gyro %2 ==1)
	{
		speedd = 2500;
		if(abs(Alldata.Redata[8].NowSpeed - speedd)>=800)
		{
			unshoot++;
			if(unshoot >= 300)
			{
				speedd = -1000;
			}
		}
		if(abs(Alldata.Redata[8].NowSpeed - speedd) <= 200 && speedd == -1000)
		{
			shoot++;
			if(shoot >= 100)
			{
				unshoot = 0;
				speedd = 2500;
				shoot = 0;
			}
		}
		
		Store_Dial_speed = (short int)(pid_handle1(&PID_control,PIDcontrol_mechspeed_Dial,speedd,Alldata.Redata[8].NowSpeed));
		
	}
}






