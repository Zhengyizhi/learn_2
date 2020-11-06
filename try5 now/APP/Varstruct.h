#ifndef __Varstruct_H
#define __Varstruct_H

#include "system.h"

//////////////标志位
typedef enum{
	__none,
	_Normal,
	_disconnected,
	_Error
}Condition_TypeDef;
typedef enum{
	_none_,
	open=2,
	close=3	
}donumber_TypeDef;
typedef enum{
	_none,
	stillwork=1,
	unwork=2,
	willunwork=3
}Working_TypeDef;
typedef enum{
	Mechanical=6,//机械
	Gyro=7,//陀螺仪
	mech_condition=11,
	forward_condition=13,
	gyro_condition=14,
	smallgyro_condition=15
}Mode_TypeDef;
typedef enum{
	none_,
	level1,
	level2,//
	VA_left_level1,
	VA_left_level2
}level_TypeDef;
typedef struct{
	Condition_TypeDef Can1interrupt_Flag;
	Condition_TypeDef DMAinterrupt_Flag;//遥控器中断
	Condition_TypeDef RemoteData_Flag;
	
	donumber_TypeDef justonegyro_Flag;
	donumber_TypeDef justonemech_Flag;//机械模式
	donumber_TypeDef justonesmallgyro_Flag;
	
	Mode_TypeDef goruning;
	Mode_TypeDef Speedcircle;
	Mode_TypeDef Anglecircle;
	
	Working_TypeDef allstart_Flag;
	
	level_TypeDef Fiction_level;//摩擦轮
	level_TypeDef SteerEngine_level;//舵机
	level_TypeDef Dial_level_mech;//拨盘
	level_TypeDef Dial_level_gyro;
	level_TypeDef Dial_level_smallgyro;//小陀螺模式连射
//	donumber_TypeDef justoneDial_Flag;//整个系统只做一次
	
	donumber_TypeDef justoneDialgyronumber;
	donumber_TypeDef justoneFictionnumber;
	donumber_TypeDef justoneSteerEnginenumber;//%2
	donumber_TypeDef justoneDialmechnumber;
	
	donumber_TypeDef justonestartFictionnumber;//初始化一次摩擦轮
	
	donumber_TypeDef justoneDialmech_Flag;
	donumber_TypeDef justoneDialgyro_Flag;
	donumber_TypeDef justoneDialsmallgyro_Flag;
	
	Working_TypeDef Fiction_Flag;
	Working_TypeDef SteerEngine_Flag;//舵机
	Working_TypeDef Dial_Flag_mech;//拨盘
	Working_TypeDef Dial_Flag_gyro;
	
}_TotalFlag;

typedef struct{
	Working_TypeDef Keystart_Flag;
	level_TypeDef Key_Fiction;
	level_TypeDef Key_SteerEngine;//舵机
	level_TypeDef Key_Vright_Aleft_180;
//	level_TypeDef Key_VAleft_180;
	level_TypeDef Key_Qleft_90;
	level_TypeDef Key_Eright_90;
	
	
	
	donumber_TypeDef fictioninit;
	donumber_TypeDef key_shoot;
	
	Mode_TypeDef Key_turnruning;
	Working_TypeDef Key_stillwork;
	Working_TypeDef Key_Dialwork;
	donumber_TypeDef	justonegyro_Flag;
	donumber_TypeDef	justonemech_Flag;
	donumber_TypeDef	justonesmallgyro_Flag;
	
	donumber_TypeDef  Key_justonesmallgyro_Flag;
	
	Working_TypeDef SteerEngine_Flag;
	Working_TypeDef SteerEngine_Flag_;
	
}_TotalKeyFlag;



typedef struct {
	float NowSpeed;//收到的速度
	float NowAngle;//收到的角度
	float Nowtorque;//实际转矩
	float preAngle;//之前的角度
	float detaNowAngle;//数值大小相减
	float NowCurrent;
	float Nowtemper;
	float longtimecircle;                 //////////////////只有这个不是瞬间值
	float longtimeangle;
}_Receive;//不需要初始化

typedef struct _Total_Receive
{
 _Receive Redata[10];//前四个底盘，后两个云台,电机信息,8拨盘
}_Total_Receive;

typedef struct{
	float control_speed[4];
	float control_downgimbelyawangle;//变化量
	float control_angleyaw_down;
	
	float control_upgimbelangle;
	float control_angle_up;
}_remoteContrl;



#endif


