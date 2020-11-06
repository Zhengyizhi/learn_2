#ifndef __PWM_H
#define __PWM_H
#include "Varstruct.h"
#include "system.h"
void TIM3_PWM_Init(u32 arr,u32 psc);
void Friction_condition(void);
void TIM1_PWM_Init(u32 arr,u32 psc);
void SteerEngine_condition(void);//¶æ»ú
#endif


