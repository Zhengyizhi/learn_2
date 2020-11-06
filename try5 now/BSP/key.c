#include "key.h"
extern _TotalKeyFlag KeyFlag;
extern _TotalFlag AllFlag;
extern _Total_Pid PID_control;
extern  _Total_Receive Alldata;
extern _remoteContrl remote_control;
extern float centeryaw;
extern float yaw;
extern short int Store_NextSpeed1;//暂时储存，可以有正负，以数组的形式输出，所以short int
extern short int Store_NextSpeed2;
extern short int Store_NextSpeed3;
extern short int Store_NextSpeed4;
extern short int Store_NextSpeed_gimbel1;
extern short int Store_NextAngle_gimbel1;
extern short int Store_NextSpeed_gimbel2;
extern short int Store_NextAngle_gimbel2;
extern short int pid_around;
extern short int Store_Dial_speed;//sentmotor();
extern short int Store_Dial_angle;


extern char W_key,S_key,A_key,D_key,V_key,Q_key,E_key,C_key,F_key,G_key,R_key,Shift_key,Ctrl_key;

float KeygimbelProportion=0.015;
extern float realzspeed;
extern float realxspeed;
float TurnAround;
float Key_Turnforward;
 int donumbersmallgyro;
float forward_Direction,right_Direction;
float S_longtiome,none_longtiome;
void Key_uncontrol()
{
	KeyFlag.Key_turnruning = gyro_condition;
	KeyFlag.justonegyro_Flag = open;
	KeyFlag.justonemech_Flag = open;
	KeyFlag.justonesmallgyro_Flag = open;
	
	forward_Direction = 0;
	right_Direction = 0;
	TurnAround = 0;
	Key_Turnforward = 0;
	donumbersmallgyro = 0;
	none_longtiome =0;
	S_longtiome = 0;
}


void Key_Initcondition()
{
	if(KeyFlag.Keystart_Flag == stillwork)
	{
		if(KeyFlag.fictioninit == open)
			{
				TIM_SetCompare1(TIM3,1000);
				TIM_SetCompare2(TIM3,1000);
				KeyFlag.fictioninit = close;
				AllFlag.Fiction_Flag = stillwork;//扔回usart.c

//				KeyFlag.key_gyroinit = open;
				KeyFlag.key_shoot = open;
				TurnAround = 0;
				Key_Turnforward = 0;
				KeyFlag.Key_turnruning = gyro_condition;
				KeyFlag.justonesmallgyro_Flag = open;
				KeyFlag.justonemech_Flag = open;
				KeyFlag.justonegyro_Flag = open;
			}
				TIM_SetCompare1(TIM3,1600);
				TIM_SetCompare2(TIM3,1600);			
	}
}
float W_number,S_number,A_number,D_number;

void Key_movement()
{
	if(W_key == 1 && W_number <= 8000)
		W_number +=150;
	else if(W_key == 0 && W_number > 0)
		W_number -=300;
	else if(W_number <0)
		W_number = 0;
	else if(W_number >8000)
		W_number = 8000;
	
	if(S_key == 1 && S_number <= 8000)
		S_number +=150;
	else if(S_key == 0 && S_number > 0)
		S_number -=300;
	else if(S_number <0)
		S_number = 0;
	else if(S_number >8000)
		S_number = 8000;
	
	forward_Direction = W_number - S_number;
	
	if(D_key == 1 && D_number <= 8000)
		D_number +=150;
	else if(D_key == 0 && D_number > 0)
		D_number -=300;
	else if(D_number <0)
		D_number = 0;
	else if(D_number >8000)
		D_number = 8000;
	
	if(A_key == 1 && A_number <= 8000)
		A_number +=150;
	else if(A_key == 0 && A_number > 0)
		A_number -=300;
	else if(A_number <0)
		A_number = 0;
	else if(A_number >8000)
		A_number = 8000;
	
	right_Direction = D_number - A_number;
}



void Key_GyroCondition()
{
	float speedmax;
	char i_speed;
	if(KeyFlag.Key_turnruning == gyro_condition)
	{
					if(KeyFlag.justonegyro_Flag == open)
					{
						PID_Init(&PID_control);
						Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;//要记录转过圈数
					//	AllFlag.justonegyro_Flag = close;
						KeyFlag.justonegyro_Flag = close;
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
						remote_control.control_angleyaw_down = 0;
						TurnAround = 0;
						Key_Turnforward = 0;
						KeyFlag.justonemech_Flag = open;
						KeyFlag.justonesmallgyro_Flag = open;
					}
					
					Alldata.Redata[7].NowAngle = yaw;	
					
					Key_turnaround();
					remote_control.control_downgimbelyawangle += -RC_Ctl.mouse.x * KeygimbelProportion;
					remote_control.control_angleyaw_down =centeryaw + remote_control.control_downgimbelyawangle;
					Alldata.Redata[6].NowAngle = remote_control.control_angleyaw_down + TurnAround;
					AngleAnaly(6,230,centeryaw,360);//目标

					AngleAnaly(7,230,centeryaw,360);
					
					Store_NextAngle_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyroangle_downgimble,Alldata.Redata[6].longtimeangle,Alldata.Redata[7].longtimeangle));
					Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,-Store_NextAngle_gimbel1,-realzspeed));

					AngleAnaly(4,5000,6350,8191);//电机
					pid_around = (short int)(pid_handle2(&PID_control,PIDcontrol_gyro_condition , Alldata.Redata[4].longtimeangle * 0.044f));
					
					if(abs(pid_around)>=60)
					{
						Key_Turnforward++;
						if(Key_Turnforward >= 2000)//1s
							KeyFlag.Key_turnruning = forward_condition;
					}
					else if(abs(pid_around)<=60)
					{
						Key_Turnforward = 0;
						KeyFlag.Key_turnruning = gyro_condition;
					}
					Key_movement();
					
					remote_control.control_speed[0] = right_Direction + forward_Direction + pid_around;
					remote_control.control_speed[1] = -(-right_Direction + forward_Direction - pid_around);
					remote_control.control_speed[2] = -right_Direction + forward_Direction + pid_around;
					remote_control.control_speed[3] = -(right_Direction + forward_Direction - pid_around);

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
					Key_pitchgimbel();
	}
}


void Key_pitchgimbel()
{
	remote_control.control_upgimbelangle += RC_Ctl.mouse.y * 0.049;
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
extern float firstdial_angle;
extern float unshoot;
extern float detadial_angle;
//float longtime11;
void Key_shoot()
{
	if(KeyFlag.key_shoot == open)
	{
		PID_Init_17_18 (&PID_control);
		Alldata.Redata[8].preAngle = Alldata.Redata[8].NowAngle;
		firstdial_angle = Alldata.Redata[8].preAngle;
		Alldata.Redata[8].longtimeangle = 0;
		Alldata.Redata[8].longtimecircle = 0;
		detadial_angle = 0;
		unshoot = 0;
		KeyFlag.key_shoot = close;
	}
	AngleAnaly(8,5000,firstdial_angle,8191);
//	longtime11 = Alldata.Redata[8].longtimeangle*0.044f;
	if(RC_Ctl.mouse.press_l == 1)
	{
		KeyFlag.Key_Dialwork = stillwork;
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f) <= 13)
		{
			detadial_angle -=1080;//目标
			unshoot = 0;
		}//一格发射

	}
	if(KeyFlag.Key_Dialwork == stillwork)
	{
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f) >= 20)
		{
			unshoot++;
			if(unshoot >= 300)
			{
				while(detadial_angle <= Alldata.Redata[8].longtimeangle*0.044f)
					detadial_angle+=2380;
			}
		}
		if(detadial_angle >= Alldata.Redata[8].longtimeangle*0.044f)
		{
			detadial_angle -=200;
		}
		if(abs(detadial_angle - Alldata.Redata[8].longtimeangle*0.044f)<=10)
		{
			KeyFlag.Key_Dialwork = unwork;
		}
	}
	Store_Dial_angle = (short int)(pid_handle1(&PID_control,PIDcontrol_mechangle_Dial,-detadial_angle,-Alldata.Redata[8].longtimeangle*0.044f));//角度环
	Store_Dial_speed = (short int)(pid_handle1(&PID_control,PIDcontrol_mechspeed_Dial,Store_Dial_angle,Alldata.Redata[8].NowSpeed));
}

void Key_turnaround()
{
	Key_levelFlag();
	if(KeyFlag.Key_Vright_Aleft_180 == level2)
	{
		TurnAround -= 180;
		KeyFlag.Key_Vright_Aleft_180 = none_;
	}
	if(KeyFlag.Key_Vright_Aleft_180 == VA_left_level2)
	{
		TurnAround += 180;
		KeyFlag.Key_Vright_Aleft_180 = none_;
	}
	if(KeyFlag.Key_Qleft_90 == level2)
	{
		TurnAround += 90;
		KeyFlag.Key_Qleft_90 = none_;
	}
	if(KeyFlag.Key_Eright_90 == level2)
	{
		TurnAround -=90;
		KeyFlag.Key_Eright_90 = none_;
	}
}
extern float deta_angle;
extern float startone;
void Key_mechgimbel()//回正
{
	if(G_key == 1 || KeyFlag.Key_turnruning == forward_condition)
	{
		KeyFlag.Key_turnruning = forward_condition;
		KeyFlag.Key_stillwork = stillwork;
	}
	if(G_key == 0 && KeyFlag.Key_stillwork == unwork)
	{
		KeyFlag.Key_turnruning = gyro_condition;
		startone = 0;
	}
	
	if(KeyFlag.Key_stillwork == stillwork)
	{
		if(Alldata.Redata[4].NowAngle>=2200 && Alldata.Redata[4].NowAngle <= 8191)
			deta_angle = ( 6350 - Alldata.Redata[4].NowAngle)*0.044f;
		else if(Alldata.Redata[4].NowAngle > 0 && Alldata.Redata[4].NowAngle < 2200)
			deta_angle = - (Alldata.Redata[4].NowAngle + 1841)*0.044f;
		Store_NextAngle_gimbel1 = (short int)(pid_handle2(&PID_control,PIDcontrol_mechangle_downgimble,deta_angle));
		Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,Store_NextAngle_gimbel1,-realzspeed));
		Key_pitchgimbel();
		Store_NextSpeed1 = Store_NextSpeed2 = Store_NextSpeed3 = Store_NextSpeed4=0;
		if(abs(deta_angle) <=20 && startone <=200)//让yaw轴稳定
		{
			startone++;
			centeryaw = yaw;//记录初始的yaw	
			if(startone >= 200)
			{
				KeyFlag.justonegyro_Flag = open;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				KeyFlag.justonemech_Flag = open;
				KeyFlag.justonesmallgyro_Flag = open;
				KeyFlag.Key_turnruning = gyro_condition;
				KeyFlag.Key_stillwork = unwork;
				startone = 0;
			}	
		}
	}
}
// int donumbersmallgyro;
void Key_mechcondition()
{
	float speedmax;
	char i_speed;
	if(Ctrl_key == 1)
	{
		KeyFlag.Key_turnruning = mech_condition;
	}
	if(Ctrl_key == 0 && KeyFlag.Key_turnruning == mech_condition && donumbersmallgyro %2 == 0)
	{
		KeyFlag.Key_turnruning = gyro_condition;
	}
	if(KeyFlag.Key_turnruning == mech_condition)
	{
			if(KeyFlag.justonemech_Flag == open)
			{
				PID_Init(&PID_control);
				Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;
				KeyFlag.justonemech_Flag = close;
				Alldata.Redata[4].longtimecircle = 0;
				KeyFlag.justonegyro_Flag = open;
				KeyFlag.justonesmallgyro_Flag = open;
				Alldata.Redata[6].longtimecircle = 0;
				Alldata.Redata[7].longtimecircle = 0;
				Alldata.Redata[6].longtimeangle = 0;
				Alldata.Redata[7].longtimeangle = 0;
				TurnAround = 0;
				Key_Turnforward = 0;
			}
			AngleAnaly(4,5000,6350,8191);
		//	centeryaw = yaw;
			AllFlag.Speedcircle = Gyro;
			AllFlag.Anglecircle = Mechanical;
			Store_NextAngle_gimbel1 = (short int)(pid_handle2(&PID_control,PIDcontrol_mechangle_downgimble,Alldata.Redata[4].longtimeangle * 0.044f));
			Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,Store_NextAngle_gimbel1,-realzspeed));
				centeryaw = yaw;
			Key_movement();
			remote_control.control_speed[0] =	 right_Direction + forward_Direction + RC_Ctl.mouse.x * 450;
			remote_control.control_speed[1] = -(-right_Direction + forward_Direction - RC_Ctl.mouse.x * 450 );
			remote_control.control_speed[2] = -right_Direction + forward_Direction + RC_Ctl.mouse.x * 450;
			remote_control.control_speed[3] = -(right_Direction + forward_Direction - RC_Ctl.mouse.x * 450);
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
			Key_pitchgimbel();

	}

}
//float S_longtiome,none_longtiome;
void Key_SteerEngine_condition()
{
	if(R_key == 1)
	{
		TIM_SetCompare2(TIM1,2000);//打开
		KeyFlag.SteerEngine_Flag = stillwork;
	}
	if(KeyFlag.SteerEngine_Flag == stillwork)
	{
		if(Q_key==1 || E_key == 1|| V_key == 1 || KeyFlag.SteerEngine_Flag_ == willunwork)
		{
			KeyFlag.SteerEngine_Flag_ = _none;
			none_longtiome =0;
			S_longtiome = 0;
			TIM_SetCompare2(TIM1,3700);//关上
			AllFlag.SteerEngine_Flag = unwork;
		}
	
	}
	if(S_key == 1)
	{
		S_longtiome++;
		if(S_longtiome >= 500)
		{
			S_longtiome = 0;
			KeyFlag.SteerEngine_Flag_ = willunwork;
		}
	}
	if(RC_Ctl.key.v == 0)
	{
		none_longtiome++;
		if(none_longtiome>=5000)//10s
		{
			none_longtiome =0;
			KeyFlag.SteerEngine_Flag_ = willunwork;
		}
	}
}

void Key_Smallgyro_condition()
{
	float theta;
	float cos_theta;
	float sin_theta;
	float forward;
	float left;
	float speedmax;
	char i_speed;
	if(F_key == 1)
	{
		if(KeyFlag.Key_justonesmallgyro_Flag == open)
		{
			donumbersmallgyro++;
			KeyFlag.Key_justonesmallgyro_Flag = close;
		}
	}
	if(donumbersmallgyro %2 == 1)
	{
		if(Ctrl_key == 0 && G_key == 0)
		{
			KeyFlag.Key_turnruning = smallgyro_condition;
		}
	}
	else if(donumbersmallgyro %2 == 0)
	{
		KeyFlag.Key_turnruning = gyro_condition;
	}
	if(F_key == 0)
	{
		KeyFlag.Key_justonesmallgyro_Flag = open;
	}
	if(KeyFlag.Key_turnruning == smallgyro_condition)
	{
				if(KeyFlag.justonesmallgyro_Flag == open)
				{
					PID_Init(&PID_control);
					Alldata.Redata[4].preAngle = Alldata.Redata[4].NowAngle;//要记录转过圈数
					KeyFlag.justonesmallgyro_Flag = close;
					centeryaw = yaw;
					Alldata.Redata[6].longtimecircle = 0;
					Alldata.Redata[7].longtimecircle = 0;//实际转过
					Alldata.Redata[6].preAngle = centeryaw;
					Alldata.Redata[7].preAngle = centeryaw;
					Alldata.Redata[6].longtimeangle = 0;
					Alldata.Redata[7].longtimeangle = 0;		
					remote_control.control_downgimbelyawangle = 0;
					KeyFlag.justonemech_Flag = open;
					KeyFlag.justonegyro_Flag = open;	
					remote_control.control_angleyaw_down = 0;
					TurnAround = 0;
					Key_Turnforward = 0;
				}
				AllFlag.Speedcircle = Gyro;
				AllFlag.Anglecircle = Gyro;
				Alldata.Redata[7].NowAngle = yaw;	
				Key_turnaround();
				remote_control.control_downgimbelyawangle += -RC_Ctl.mouse.x * KeygimbelProportion;
				remote_control.control_angleyaw_down =centeryaw + remote_control.control_downgimbelyawangle;
				Alldata.Redata[6].NowAngle = remote_control.control_angleyaw_down + TurnAround;
				AngleAnaly(6,200,centeryaw,360);//遥控器转过

				AngleAnaly(7,200,centeryaw,360);
				
				Store_NextAngle_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyroangle_downgimble,Alldata.Redata[6].longtimeangle,Alldata.Redata[7].longtimeangle));
				Store_NextSpeed_gimbel1 = (short int)(pid_handle1(&PID_control,PIDcontrol_gyrospeed_downgimble,-Store_NextAngle_gimbel1,-realzspeed));
				Key_movement();
				
				theta = ( Alldata.Redata[4].NowAngle - 6350)*0.044f ;
				sin_theta = lookup_sin(theta);
				cos_theta = lookup_cos(theta);
				forward = forward_Direction * cos_theta + right_Direction * sin_theta;//3
				left = - forward_Direction * sin_theta + right_Direction * cos_theta;//2
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
				Key_pitchgimbel();
	}
}




void Key_levelFlag()
{
	if(V_key==1 && A_key == 0)
		KeyFlag.Key_Vright_Aleft_180 = level1;
	else if(V_key==1 && A_key ==1)
		KeyFlag.Key_Vright_Aleft_180 = VA_left_level1;
	
	if(Q_key == 1)
		KeyFlag.Key_Qleft_90 = level1;
	if(E_key == 1)
		KeyFlag.Key_Eright_90 = level1;
	
	if(KeyFlag.Key_Vright_Aleft_180 == level1 && V_key==0 && A_key == 0)
		KeyFlag.Key_Vright_Aleft_180 = level2;
	if(KeyFlag.Key_Vright_Aleft_180 == VA_left_level1 && V_key==0 && A_key == 0)
		KeyFlag.Key_Vright_Aleft_180 = VA_left_level2;
	if(KeyFlag.Key_Qleft_90 == level1 && Q_key == 0)
		KeyFlag.Key_Qleft_90 = level2;
	if(KeyFlag.Key_Eright_90 == level1 && E_key == 0)
		KeyFlag.Key_Eright_90 = level2;
}




