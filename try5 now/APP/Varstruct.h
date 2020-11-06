#ifndef __Varstruct_H
#define __Varstruct_H

#include "system.h"

//////////////��־λ
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
	Mechanical=6,//��е
	Gyro=7,//������
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
	Condition_TypeDef DMAinterrupt_Flag;//ң�����ж�
	Condition_TypeDef RemoteData_Flag;
	
	donumber_TypeDef justonegyro_Flag;
	donumber_TypeDef justonemech_Flag;//��еģʽ
	donumber_TypeDef justonesmallgyro_Flag;
	
	Mode_TypeDef goruning;
	Mode_TypeDef Speedcircle;
	Mode_TypeDef Anglecircle;
	
	Working_TypeDef allstart_Flag;
	
	level_TypeDef Fiction_level;//Ħ����
	level_TypeDef SteerEngine_level;//���
	level_TypeDef Dial_level_mech;//����
	level_TypeDef Dial_level_gyro;
	level_TypeDef Dial_level_smallgyro;//С����ģʽ����
//	donumber_TypeDef justoneDial_Flag;//����ϵͳֻ��һ��
	
	donumber_TypeDef justoneDialgyronumber;
	donumber_TypeDef justoneFictionnumber;
	donumber_TypeDef justoneSteerEnginenumber;//%2
	donumber_TypeDef justoneDialmechnumber;
	
	donumber_TypeDef justonestartFictionnumber;//��ʼ��һ��Ħ����
	
	donumber_TypeDef justoneDialmech_Flag;
	donumber_TypeDef justoneDialgyro_Flag;
	donumber_TypeDef justoneDialsmallgyro_Flag;
	
	Working_TypeDef Fiction_Flag;
	Working_TypeDef SteerEngine_Flag;//���
	Working_TypeDef Dial_Flag_mech;//����
	Working_TypeDef Dial_Flag_gyro;
	
}_TotalFlag;

typedef struct{
	Working_TypeDef Keystart_Flag;
	level_TypeDef Key_Fiction;
	level_TypeDef Key_SteerEngine;//���
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
	float NowSpeed;//�յ����ٶ�
	float NowAngle;//�յ��ĽǶ�
	float Nowtorque;//ʵ��ת��
	float preAngle;//֮ǰ�ĽǶ�
	float detaNowAngle;//��ֵ��С���
	float NowCurrent;
	float Nowtemper;
	float longtimecircle;                 //////////////////ֻ���������˲��ֵ
	float longtimeangle;
}_Receive;//����Ҫ��ʼ��

typedef struct _Total_Receive
{
 _Receive Redata[10];//ǰ�ĸ����̣���������̨,�����Ϣ,8����
}_Total_Receive;

typedef struct{
	float control_speed[4];
	float control_downgimbelyawangle;//�仯��
	float control_angleyaw_down;
	
	float control_upgimbelangle;
	float control_angle_up;
}_remoteContrl;



#endif


