#ifndef __Chassis_H
#define __Chassis_H
#include "system.h"
#include "my_include.h"
//#define IS_Circle_GET_FLAG(Circle_FLAG) (((Circle_FLAG) == Standcircle) || \
//                                   ((Circle_FLAG) == Speedcircle) || \
//                                   ((Circle_FLAG) == Turnaroundcircle))
////·ûºÏ¾Í·µ»Ø1

void send_chassis_moto(void);
void chassis_control(void);
void mpu_AngleAnaly(char num,float jump,float center,float degree,float Nowangle);
extern float stand_angle1,stand_angle2,stand_Speed1,stand_Speed2;//
extern float speedcircle_speed1,speedcircle_speed2;
#endif
