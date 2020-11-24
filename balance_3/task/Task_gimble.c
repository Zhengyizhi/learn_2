#include "gimble.h"


short int Store_NextSpeed_gimbel1;//yaw
short int Store_NextAngle_gimbel1;
short int Store_NextSpeed_gimbel2;
short int Store_NextAngle_gimbel2;


extern chassis_pid_t pid_gimble_Array[2];

void gimble_control()
{
	if(AllFlag.Can1interrupt_Flag == _normal && (AllFlag.DMAinterrupt_Flag == _disconnected || AllFlag.RemoteData_Flag == _error))//只读取信息电机再can.c,遥控器突然失联
	{
		Store_NextSpeed_gimbel2=Store_NextSpeed_gimbel1=0;
	}
	if(AllFlag.DMAinterrupt_Flag == _normal&& AllFlag.Can1interrupt_Flag == _normal)
	{
//		Gimbel_AngleAnaly(0,200,centeryaw,360);//实际转过度数
//	Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&(pid_gimble_Array[0].Speed),Remote_Target_Array[Right_remote_cross],Chassis_Moto_Info_Array[0].NowSpeed));
////		Store_NextSpeed_gimbel1 = (short int)(pid_handle2_test(&Total_pid,0,Remote_Target_Array[Right_remote_cross],Gimbel_Moto_Info_Array[0].NowSpeed));
		Store_NextAngle_gimbel1 = (short int)(pid_handle1_test(&Total_pid,1,Remote_Target_Array[Right_remote_cross],Gimbel_Moto_Info_Array[0].NowAngle));
		Store_NextSpeed_gimbel1 = (short int)(pid_handle2_test(&Total_pid,0,Store_NextAngle_gimbel1,Gimbel_Moto_Info_Array[0].NowSpeed));
		
		Store_NextSpeed_gimbel2 = (short int)(pid_handle1_test(&Total_pid,2,0,Gimbel_Moto_Info_Array[1].NowSpeed));
	
	}
	send_gimble_moto();
}

u8 Send_gimble_Array[8];//发送云台电流数据

void send_gimble_moto()
{
	Send_gimble_Array[0]=(Store_NextSpeed_gimbel1>>8)&0xff;
	Send_gimble_Array[1]=Store_NextSpeed_gimbel1&0xff;
	Send_gimble_Array[2] = (Store_NextSpeed_gimbel2>>8)&0xff;
	Send_gimble_Array[3] = Store_NextSpeed_gimbel2&0xff;
	CAN1_Send_Msg_gimbel(Send_gimble_Array,8);
}
