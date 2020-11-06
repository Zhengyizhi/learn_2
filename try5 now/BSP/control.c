#include "control.h"
#include "PID.h"
float ProportionRemote_speed = 12.12;//遥控器转一个数值等于12.12速度
float ProportionCrable_angle2=0.001;


short int Store_NextSpeed1;//暂时储存，可以有正负，以数组的形式输出，所以short int
short int Store_NextSpeed2;
short int Store_NextSpeed3;
short int Store_NextSpeed4;
short int Store_NextSpeed_gimbel1;
short int Store_NextAngle_gimbel1;
short int Store_NextSpeed_gimbel2;
short int Store_NextAngle_gimbel2;
short int pid_around;

extern _remoteContrl remote_control;
//失控保护
void Get_SystemMode()	//使用时换成int型返回数值
{

}
extern float realzspeed;
extern float realxspeed;
extern float yaw;
float centeryaw;
extern  _Total_Pid PID_control;
extern  _Total_Receive Alldata;
extern _TotalFlag AllFlag;
extern _TotalKeyFlag KeyFlag;
void MechCondition()
{
	float speedmax;
	char i_speed;
	if(AllFlag.justonemech_Flag == open)
	{
		PID_Init(&PID_control);
		Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;
		AllFlag.justonemech_Flag = close;
		Alldata.Redata[4].longtimecircle = 0;
		AllFlag.justonegyro_Flag = open;
		AllFlag.justonesmallgyro_Flag = open;
		Alldata.Redata[6].longtimecircle = 0;
		Alldata.Redata[7].longtimecircle = 0;
		Alldata.Redata[6].longtimeangle = 0;
		Alldata.Redata[7].longtimeangle = 0;
	}
	AngleAnaly(4,5000,6350,8191);
//	centeryaw = yaw;
	AllFlag.Speedcircle = Gyro;
	AllFlag.Anglecircle = Mechanical;
	Store_NextAngle_gimbel1 = (short int)(pid_handle2(&PID_control,PIDcontrol_mechangle_downgimble,Alldata.Redata[4].longtimeangle * 0.044f));
	Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,Store_NextAngle_gimbel1,-realzspeed));
		centeryaw = yaw;
	remote_control.control_speed[0] = RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed + RC_Ctl.rc.ch0 * ProportionRemote_speed;
	remote_control.control_speed[1]= -(-RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed - RC_Ctl.rc.ch0 * ProportionRemote_speed);
	remote_control.control_speed[2] = -RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed + RC_Ctl.rc.ch0 * ProportionRemote_speed;
	remote_control.control_speed[3] = -(RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed - RC_Ctl.rc.ch0 * ProportionRemote_speed);
	speedmax = remote_control.control_speed[0];
	for(i_speed=1;i_speed<4;i_speed++)
	{
		if(speedmax <= abs(remote_control.control_speed[i_speed]))
		{
			speedmax = abs(remote_control.control_speed[i_speed]);//取最大值
		}
	}
	if(speedmax >= 8000 )
	{
		for(i_speed=0;i_speed<4;i_speed++)
		{
			remote_control.control_speed[i_speed]=((float)(remote_control.control_speed[i_speed]) * 8000/(speedmax));
		}
	}
	Store_NextSpeed1=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis1,remote_control.control_speed[0],Alldata.Redata[0].NowSpeed));
	Store_NextSpeed2=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis2,remote_control.control_speed[1],Alldata.Redata[1].NowSpeed));
	Store_NextSpeed3=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis3,remote_control.control_speed[2],Alldata.Redata[2].NowSpeed));
	Store_NextSpeed4=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis4,remote_control.control_speed[3],Alldata.Redata[3].NowSpeed));//底盘	
	pitchgimbel();
}

void pitchgimbel()
{
	remote_control.control_upgimbelangle += -(RC_Ctl.rc.ch1 * 0.007);
	remote_control.control_angle_up = 4000 + remote_control.control_upgimbelangle;
	
	if(remote_control.control_angle_up <= 3440)
	{
		remote_control.control_angle_up = 3440;
		remote_control.control_upgimbelangle = -560;
	}
	if(remote_control.control_angle_up >= 4450)
	{
		remote_control.control_angle_up = 4450;
		remote_control.control_upgimbelangle = 450;//没有按照度数计算
	}
	Store_NextAngle_gimbel2 = (short int)(pid_handle1(&PID_control,PIDcontrol_mechangle_upgimble,remote_control.control_angle_up,Alldata.Redata[5].NowAngle)); 
	Store_NextSpeed_gimbel2 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_upgimble,Store_NextAngle_gimbel2,realxspeed));//向下位正方向
}

void smallGyrocondition()//ProportionRemote_speed
{
	float theta;
	float cos_theta;
	float sin_theta;
	float forward;
	float left;
	float speedmax;
	char i_speed;
	
	if(AllFlag.justonesmallgyro_Flag == open)
	{
		PID_Init(&PID_control);
		Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;//要记录转过圈数
		AllFlag.justonesmallgyro_Flag = close;
		centeryaw = yaw;
		Alldata.Redata[6].longtimecircle = 0;
		Alldata.Redata[7].longtimecircle = 0;//实际转过
		Alldata.Redata[6].preAngle = centeryaw;
		Alldata.Redata[7].preAngle = centeryaw;
		Alldata.Redata[6].longtimeangle = 0;
		Alldata.Redata[7].longtimeangle = 0;		
		remote_control.control_downgimbelyawangle = 0;
		AllFlag.justonemech_Flag = open;
		AllFlag.justonegyro_Flag = open;	
		remote_control.control_angleyaw_down = 0;
	}
	AllFlag.Speedcircle = Gyro;
	AllFlag.Anglecircle = Gyro;
	Alldata.Redata[7].NowAngle = yaw;	
	remote_control.control_downgimbelyawangle += -RC_Ctl.rc.ch0 * ProportionCrable_angle2;
	remote_control.control_angleyaw_down =centeryaw + remote_control.control_downgimbelyawangle;
	Alldata.Redata[6].NowAngle = remote_control.control_angleyaw_down;
	AngleAnaly(6,200,centeryaw,360);//遥控器转过

	AngleAnaly(7,200,centeryaw,360);
	
	Store_NextAngle_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyroangle_downgimble,Alldata.Redata[6].longtimeangle,Alldata.Redata[7].longtimeangle));
	Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,-Store_NextAngle_gimbel1,-realzspeed));

	theta = ( Alldata.Redata[4].NowAngle - 6350)*0.044f ;
	sin_theta = lookup_sin(theta);
	cos_theta = lookup_cos(theta);
	forward = RC_Ctl.rc.ch3 * ProportionRemote_speed * cos_theta + RC_Ctl.rc.ch2 * ProportionRemote_speed * sin_theta;//3
	left = - RC_Ctl.rc.ch3 * ProportionRemote_speed * sin_theta + RC_Ctl.rc.ch2 * ProportionRemote_speed * cos_theta;//2
	remote_control.control_speed[0] = left + forward + 3000;
	remote_control.control_speed[1]=  left - forward + 3000;
	remote_control.control_speed[2] = -left + forward + 3000;
	remote_control.control_speed[3] = -left - forward + 3000;
	speedmax = remote_control.control_speed[0];
	for(i_speed=1;i_speed<4;i_speed++)
	{
		if(speedmax <= abs(remote_control.control_speed[i_speed]))
		{
			speedmax = abs(remote_control.control_speed[i_speed]);//取最大值
		}
	}
	
	if(speedmax >= 8000 )
	{
		for(i_speed=0;i_speed<4;i_speed++)
		{
			remote_control.control_speed[i_speed]=((float)(remote_control.control_speed[i_speed]) * 8000/(speedmax));
		}
	}
	Store_NextSpeed1=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis1,remote_control.control_speed[0],Alldata.Redata[0].NowSpeed));
	Store_NextSpeed2=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis2,remote_control.control_speed[1],Alldata.Redata[1].NowSpeed));
	Store_NextSpeed3=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis3,remote_control.control_speed[2],Alldata.Redata[2].NowSpeed));
	Store_NextSpeed4=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis4,remote_control.control_speed[3],Alldata.Redata[3].NowSpeed));//底盘	
	pitchgimbel();

}



void GyroCondition()//先机械模式
{
	float speedmax;
	char i_speed;
	if(AllFlag.justonegyro_Flag == open)
	{
		PID_Init(&PID_control);
		Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;//要记录转过圈数
		AllFlag.justonegyro_Flag = close;
		centeryaw = yaw;
		Alldata.Redata[6].longtimecircle = 0;
		Alldata.Redata[4].longtimecircle = 0;//遥控器目标转过
		Alldata.Redata[7].longtimecircle = 0;//实际转过
		Alldata.Redata[6].preAngle = centeryaw;
		Alldata.Redata[7].preAngle = centeryaw;
		Alldata.Redata[6].longtimeangle = 0;
		Alldata.Redata[7].longtimeangle = 0;		
		Alldata.Redata[4].longtimeangle = 0;//遥控器目标转过
		remote_control.control_downgimbelyawangle = 0;
		AllFlag.justonemech_Flag = open;
		AllFlag.justonesmallgyro_Flag = open;
	}
	
	AllFlag.Speedcircle = Gyro;
	AllFlag.Anglecircle = Gyro;
	Alldata.Redata[7].NowAngle = yaw;	
	remote_control.control_downgimbelyawangle += -RC_Ctl.rc.ch0 * ProportionCrable_angle2;
	remote_control.control_angleyaw_down =centeryaw + remote_control.control_downgimbelyawangle;
	Alldata.Redata[6].NowAngle = remote_control.control_angleyaw_down;
	AngleAnaly(6,200,centeryaw,360);//遥控器转过

	AngleAnaly(7,200,centeryaw,360);
	
	Store_NextAngle_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyroangle_downgimble,Alldata.Redata[6].longtimeangle,Alldata.Redata[7].longtimeangle));
	Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,-Store_NextAngle_gimbel1,-realzspeed));

	AngleAnaly(4,5000,6350,8191);
	pid_around = (short int)(pid_handle2(&PID_control,PIDcontrol_gyro_condition , Alldata.Redata[4].longtimeangle * 0.044f));
	remote_control.control_speed[0] = RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed + pid_around;
	remote_control.control_speed[1] = -(-RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed - pid_around);
	remote_control.control_speed[2] = -RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed + pid_around;
	remote_control.control_speed[3] = -(RC_Ctl.rc.ch2 * ProportionRemote_speed + RC_Ctl.rc.ch3 * ProportionRemote_speed - pid_around);

	speedmax=(short int)remote_control.control_speed[0];
		for(i_speed=1;i_speed<4;i_speed++)
		{
			if(speedmax <= abs(remote_control.control_speed[i_speed]))
			{
				speedmax = abs(remote_control.control_speed[i_speed]);//取最大值
			}
		}
		if(speedmax >= 8000 )
		{
			for(i_speed=0;i_speed<4;i_speed++)
			{
				remote_control.control_speed[i_speed]=((float)(remote_control.control_speed[i_speed]) * 8000/(speedmax));
			}
		}
		
	Store_NextSpeed1=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis1,remote_control.control_speed[0],Alldata.Redata[0].NowSpeed));
	Store_NextSpeed2=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis2,remote_control.control_speed[1],Alldata.Redata[1].NowSpeed));
	Store_NextSpeed3=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis3,remote_control.control_speed[2],Alldata.Redata[2].NowSpeed));
	Store_NextSpeed4=(short int)(pid_handle1(&PID_control,PIDcontrol_chassis4,remote_control.control_speed[3],Alldata.Redata[3].NowSpeed));//底盘	
	pitchgimbel();
}
int startone;
float deta_angle;
void forwardcondition()
{
	if(AllFlag.allstart_Flag==unwork)
	{
		if(Alldata.Redata[4].NowAngle>=2200 && Alldata.Redata[4].NowAngle <= 8191)
			deta_angle = ( 6350 - Alldata.Redata[4].NowAngle)*0.044f;
		else if(Alldata.Redata[4].NowAngle > 0 && Alldata.Redata[4].NowAngle < 2200)
			deta_angle = - (Alldata.Redata[4].NowAngle + 1841)*0.044f;
		Store_NextAngle_gimbel1 = (short int)(pid_handle2(&PID_control,PIDcontrol_mechangle_downgimble,deta_angle));
		Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,Store_NextAngle_gimbel1,-realzspeed));
		pitchgimbel();
		Store_NextSpeed1 = Store_NextSpeed2 = Store_NextSpeed3 = Store_NextSpeed4=0;
		if(Store_NextSpeed_gimbel1>=3000)
			Store_NextSpeed_gimbel1 = 3000;
		if(Store_NextSpeed_gimbel1<=-3000)
			Store_NextSpeed_gimbel1 = -3000;//缓慢归中
		sent_Motor();
		
		if(abs(deta_angle) <=20 && startone <=2000)//让yaw轴稳定
		{
			startone++;
			centeryaw = yaw;//记录初始的yaw	
			if(abs(deta_angle) >=10)
			{
				startone =0;//初始化保护
			}
			if(startone == 2000)
			{
				AllFlag.allstart_Flag=stillwork;
				AllFlag.justonegyro_Flag = open;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				AllFlag.justonemech_Flag = open;
				AllFlag.justonesmallgyro_Flag = open;
			}
		}
	}
}




u8 Send_buf[8];//发送电流数据   char
u8 Send_buf_crable[8];//发送云台电流数据
u8 Send_buf_dial[8];

extern short int Store_Dial_speed;

void sent_Motor()
{
	Send_buf[0]=(Store_NextSpeed1>>8)&0xff;
	Send_buf[1]=Store_NextSpeed1&0xff;
	Send_buf[2]=(Store_NextSpeed2>>8)&0xff;
	Send_buf[3]=Store_NextSpeed2&0xff;
	Send_buf[4]=(Store_NextSpeed3>>8)&0xff;
	Send_buf[5]=Store_NextSpeed3&0xff;
	Send_buf[6]=(Store_NextSpeed4>>8)&0xff;
	Send_buf[7]=Store_NextSpeed4&0xff;
	CAN1_Send_Msg_chassis(Send_buf,8);
	
	Send_buf_crable[0]=(Store_NextSpeed_gimbel1>>8)&0xff;
	Send_buf_crable[1]=Store_NextSpeed_gimbel1&0xff;
	Send_buf_crable[2] = (Store_NextSpeed_gimbel2>>8)&0xff;
	Send_buf_crable[3] = Store_NextSpeed_gimbel2&0xff;
	CAN1_Send_Msg_crable(Send_buf_crable,8);
	
	Send_buf_dial[4]=(Store_Dial_speed >> 8)&0xff;
	Send_buf_dial[5]=Store_Dial_speed & 0xff;
	CAN2_Send_Dial(Send_buf_dial,8);
}

extern int donumber_Fiction;
extern int donumber_SteerEngine;//舵机

void SentAllData_CAN1()
{
	if(AllFlag.Can1interrupt_Flag == _disconnected)//电机失联保护
	{
		PID_Init(&PID_control);
		Store_NextSpeed1 = Store_NextSpeed2 = Store_NextSpeed3 = Store_NextSpeed4 = Store_NextSpeed_gimbel1 = Store_NextSpeed_gimbel2=0;//全部卸力;
		Alldata.Redata[4].longtimecircle = 0;
		Alldata.Redata[6].longtimecircle = 0;
		Alldata.Redata[7].longtimecircle = 0;
		Alldata.Redata[6].longtimeangle = 0;
		Alldata.Redata[7].longtimeangle = 0;
		Alldata.Redata[4].longtimeangle = 0;
	}
	else if(AllFlag.Can1interrupt_Flag == _Normal && (AllFlag.DMAinterrupt_Flag == _disconnected || AllFlag.RemoteData_Flag == _Error))//只读取信息电机再can.c,遥控器突然失联
	{
		Store_NextSpeed1 = Store_NextSpeed2 = Store_NextSpeed3 = Store_NextSpeed4 = Store_NextSpeed_gimbel1 = Store_NextSpeed_gimbel2=Store_Dial_speed=0;//全部卸力;
		AllFlag.allstart_Flag = unwork;
		startone = 0;//让yaw稳定
	//要发送出去
		AllFlag.justonegyro_Flag=open;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		AllFlag.justonemech_Flag=open;
		AllFlag.justonesmallgyro_Flag = open;
		TIM_SetCompare2(TIM1,3700);//关上舵机
		TIM_SetCompare1(TIM3,1000);
		TIM_SetCompare2(TIM3,1000);
		AllFlag.Fiction_Flag = unwork;//关上摩擦轮
		sent_Motor();
		AllFlag.justoneDialmech_Flag = open;
		AllFlag.justoneDialgyro_Flag = open;
		AllFlag.justoneDialsmallgyro_Flag = open;
		KeyFlag.Keystart_Flag = unwork;
	}
	else if(AllFlag.DMAinterrupt_Flag == _Normal && AllFlag.Can1interrupt_Flag == _Normal)
	{
		forwardcondition();
		if(AllFlag.allstart_Flag == stillwork)
		{
			if(KeyFlag.Keystart_Flag == stillwork)
			{
				Key_control();
			}
			else if(KeyFlag.Keystart_Flag == unwork)
			{
				Key_uncontrol();
					if(AllFlag.goruning == mech_condition)
					{
						MechCondition();
						SteerEngine_condition();
						if(AllFlag.Fiction_Flag == stillwork)
							Dial_mech_onelattice();
					}
					if(AllFlag.goruning == gyro_condition)
					{
						GyroCondition();
						Friction_condition();
						if(AllFlag.Fiction_Flag == stillwork)
							Dial_gyro_continuelattice();
					}
					if(AllFlag.goruning == smallgyro_condition)
					{
						smallGyrocondition();
						if(AllFlag.Fiction_Flag == stillwork)
							Dial_smallgyro_comlattice();
					}
			}
			sent_Motor();
		}
	}
}
