#include "remote.h"
//14ms处理一次
float Gimbel_remotepitch_Speedsensitivity = 0.05;
float Gimbel_remotepitch_Anglesensitivity = 0.001;
float Gimbel_remoteyaw_Speedsensitivity = 0.1;
float Gimbel_remoteyaw_Anglesensitivity = 0.02;

//#define Right_remote_cross 0
//#define Right_remote_vertical 1
//#define Left_remote_cross 2
//#define Left_remote_vertical 3

/*遥控器灵敏度*/
float Remote_Target_Array[5];
float centeryaw;

float Add_down_gimble;


Circle_condition Circle_analy;

void remote_calculate()
{
	if(Circle_analy.yawgimble_circle == speedset)
	{
		Remote_Target_Array[Right_remote_cross] =  RC_Ctl.rc.ch0 * Gimbel_remoteyaw_Speedsensitivity;
		//!!!!!!!!!!!!!!!!
		Remote_Target_Array[Right_remote_cross] = myconstrain(Remote_Target_Array[Right_remote_cross] , -75,75);//!!!!!!!!!!!!!!
		if(Gimbel_Moto_Info_Array[0].NowAngle <=1150 ||  Gimbel_Moto_Info_Array[0].NowAngle >= 5450)
		{
			Remote_Target_Array[Right_remote_cross] = 0;
		}
		
	//	Remote_Target_Array[Right_remote_vertical] = RC_Ctl.rc.ch1 * Gimbel_remotepitch_sensitivity;
	}
	else if(Circle_analy.yawgimble_circle == angleset)
	{
		Add_down_gimble += RC_Ctl.rc.ch0 * Gimbel_remoteyaw_Anglesensitivity;
		
		Remote_Target_Array[Right_remote_cross] = Add_down_gimble + 3375;

		Remote_Target_Array[Right_remote_cross] = myconstrain(Remote_Target_Array[Right_remote_cross],1208,5504);
		Add_down_gimble = myconstrain(Add_down_gimble,-2167,2129);
	}
	
	if(Circle_analy.pitchgimble_circle == speedset)
	{
//		Remote_Target_Array[Right_remote_vertical] = -RC_Ctl.rc.ch1 * Gimbel_remotepitch_Speedsensitivity;
//		Remote_Target_Array[Right_remote_vertical] = myconstrain(Remote_Target_Array[Right_remote_vertical],-35,35);
//		if(Gimbel_Moto_Info_Array[1].NowAngle <=3362 ||  Gimbel_Moto_Info_Array[0].NowAngle >= 4650)
//		{
//			Remote_Target_Array[Right_remote_vertical] = 0;
//		}
	}
	else if(Circle_analy.pitchgimble_circle == angleset)
	{
		Remote_Target_Array[Right_remote_vertical] = RC_Ctl.rc.ch1 * Gimbel_remotepitch_Anglesensitivity;
	}
	
	if(Circle_analy.Stand_circle == Set)
	{
		
	}
	
	if(Circle_analy.Speedrun_circle == Set)
	{
	
	
	}
	
}

void control_circle_init()
{
	Circle_analy.yawgimble_circle = angleset;
	Circle_analy.pitchgimble_circle = speedset;
	Circle_analy.Stand_circle = Set;
	Circle_analy.Speedrun_circle = Set;
}


