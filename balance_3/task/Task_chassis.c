#include "chassis.h"
#include "my_include.h"

////char test[6];//最大5,"0正确，2有问题"
/////*test [0]环的名字*/
//Circle_control Circle_startSet (uint8_t circle_Flag,uint8_t status)
//{
//	Circle_control circlestatus  = None;
//	
//	assert_param(IS_Circle_GET_FLAG(circle_Flag));//调试的时候可以试试
//	if(status == Set)
//	{
//		circlestatus = Set;
//	}
//	else if(status == ReSet)
//	{
//		circlestatus = ReSet;
//	}	
//	return circlestatus;
//}

short int Store_NextSpeed1;//暂时储存，可以有正负，以数组的形式输出，所以short int
short int Store_NextAngle1;
short int Store_NextSpeed2;
short int Store_NextAngle2;
//short int Store_NextSpeed3;
//short int Store_NextSpeed4;
_Receive mpu_data_Array[6];

#define  center_Pitch 178.3

void Wait_stand_init()
{
	if(pitch>=-180&&pitch<=-170)
	{
		AllFlag.Stand_Flag = _normal;
	}
//	else
//		AllFlag.Stand_Flag = _error;
//	mpu_data_Array[4].preAngle = -170;
//	mpu_data_Array[num].longtimeangle
}

void mpu_AngleAnaly(char num,float jump,float center,float degree,float Nowangle)//4
{	
	if(center - Nowangle<-200 && Nowangle > 0)//越界
	{
		mpu_data_Array[num].longtimeangle = 360 + center - Nowangle;//+
	}
	else
	{
		mpu_data_Array[num].longtimeangle = center - Nowangle;//-
	}
//
//	mpu_data_Array[num].longtimeangle = -degree * mpu_data_Array[num].longtimecircle + ( center - Nowangle );	
//mpu_data_Array[num].preAngle =  Nowangle;
}

float stand_angle1,stand_angle2,stand_Speed1,stand_Speed2;//
float speedcircle_speed1,speedcircle_speed2;

float stand_speedproportion1,speed_speedproportion1;
void chassis_control()
{
	if(AllFlag.Can1interrupt_Flag == _normal && (AllFlag.DMAinterrupt_Flag == _disconnected || AllFlag.RemoteData_Flag == _error))//只读取信息电机再can.c,遥控器突然失联
	{
		Store_NextSpeed1=Store_NextSpeed2=0;
	}
	if(AllFlag.DMAinterrupt_Flag == _normal&& AllFlag.Can1interrupt_Flag == _normal)
	{
		if(Circle_analy.Stand_circle == Set)
		{
			mpu_AngleAnaly(4,200,-178.3,360,pitch);//右轮
	//		stand_angle1 = (short int)(pid_handle1_test(&Total_pid,5,0,mpu_data_Array[4].longtimeangle));
			stand_Speed1 = (short int)(balanstand(&Total_pid,4,0,mpu_data_Array[4].longtimeangle,gyrox,25));
			
		//	mpu_AngleAnaly(1,200,-178.3,360);//左轮
//			stand_angle2 = (short int)(pid_handle1_test(&Total_pid,7,0,mpu_data_Array[4].longtimeangle));
			stand_Speed2 = -stand_Speed1;
		}
		
		if(Circle_analy.Speedrun_circle == Set)
		{
			speedcircle_speed1 = (short int)((balanSpeed(&Total_pid,3,Chassis_Moto_Info_Array[1].NowSpeed,Chassis_Moto_Info_Array[0].NowSpeed))* Total_pid.pid_test_Array[4].Kp);
			//右轮
			speedcircle_speed2 = -speedcircle_speed1;
			//左轮
		}
		
	//	if(Circle_analy.Speedrun_circle == Set && Circle_analy.Stand_circle == Set)
		{
			Store_NextSpeed1 = stand_Speed1 - speedcircle_speed1;
			Store_NextSpeed2 = stand_Speed2 - speedcircle_speed2;
			Store_NextSpeed1 = myconstrain(Store_NextSpeed1,-9000,9000);
			Store_NextSpeed2 = myconstrain(Store_NextSpeed2,-9000,9000);

		
		}
		
//	send_gimble_moto();
  }
	if(AllFlag.Stand_Flag == _normal)
		send_chassis_moto();
}
u8 Send_chassis_Array[8];
void send_chassis_moto()
{
	Send_chassis_Array[0]=(Store_NextSpeed1>>8)&0xff;
	Send_chassis_Array[1]=Store_NextSpeed1&0xff;
	Send_chassis_Array[2] = (Store_NextSpeed2>>8)&0xff;
	Send_chassis_Array[3] = Store_NextSpeed2&0xff;
	CAN1_Send_Msg_chassis(Send_chassis_Array,8);
}





