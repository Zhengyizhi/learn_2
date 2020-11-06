#include "PWM.h"


void TIM3_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);//???
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);//???
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9

	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2///CH1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���//ͨ��һ
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

extern _TotalFlag AllFlag; 

int donumber_Fiction;
int donumber_SteerEngine;//���
int donumber_Dial_gyro;
float change;
void Friction_condition()
{
	if(AllFlag.Fiction_level == level2)
	{
		if(AllFlag.justoneFictionnumber == open)
		{
			donumber_Fiction++;
			AllFlag.justoneFictionnumber = close;
		}
		if(donumber_Fiction % 2 == 1)
		{
			if(AllFlag.justonestartFictionnumber == open)
			{
				TIM_SetCompare1(TIM3,1000);
				TIM_SetCompare2(TIM3,1000);
				AllFlag.justonestartFictionnumber = close;
			}
			TIM_SetCompare1(TIM3,1600);
			TIM_SetCompare2(TIM3,1600);
			AllFlag.Fiction_Flag = stillwork;//�ӻ�usart.c
		}
		else if(donumber_Fiction % 2 == 0)
		{
			TIM_SetCompare1(TIM3,1000);
			TIM_SetCompare2(TIM3,1000);
			AllFlag.Fiction_Flag = unwork;
			AllFlag.justonestartFictionnumber = open;
		}
	}
	else if(AllFlag.Fiction_level == level1)
	{
		AllFlag.justoneFictionnumber = open;
		AllFlag.justonestartFictionnumber = open;
	}

}



void TIM1_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);//???

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE,&GPIO_InitStructure);  
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV2;/////////////////////////////////

	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);	

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2///CH1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM3
}


void SteerEngine_condition()//���
{
	if(AllFlag.SteerEngine_level == level2)
	{
		if(AllFlag.justoneSteerEnginenumber == open)
		{
			donumber_SteerEngine++;
			AllFlag.justoneSteerEnginenumber = close;
		}
		if(donumber_SteerEngine %2 == 1)
		{
			TIM_SetCompare2(TIM1,2000);//��
			AllFlag.SteerEngine_Flag = stillwork;
		}
		else if(donumber_SteerEngine %2 == 0)
		{
			TIM_SetCompare2(TIM1,3700);//����
			AllFlag.SteerEngine_Flag = unwork;
		}
	}
	else if(AllFlag.SteerEngine_level == level1)
	{
		AllFlag.justoneSteerEnginenumber = open;
	}
}


