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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PF9

	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2///CH1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器//通道一
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

extern _TotalFlag AllFlag; 

int donumber_Fiction;
int donumber_SteerEngine;//舵机
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
			AllFlag.Fiction_Flag = stillwork;//扔回usart.c
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);  
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV2;/////////////////////////////////

	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);	

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2///CH1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM3
}


void SteerEngine_condition()//舵机
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
			TIM_SetCompare2(TIM1,2000);//打开
			AllFlag.SteerEngine_Flag = stillwork;
		}
		else if(donumber_SteerEngine %2 == 0)
		{
			TIM_SetCompare2(TIM1,3700);//关上
			AllFlag.SteerEngine_Flag = unwork;
		}
	}
	else if(AllFlag.SteerEngine_level == level1)
	{
		AllFlag.justoneSteerEnginenumber = open;
	}
}


