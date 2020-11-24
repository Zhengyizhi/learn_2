/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *          此代码可任意传播与使用，但请注明出处。                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"
#include "system.h"
#include "system.h"
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
}kalman;

typedef struct{
	kalman P_out_Kalman_Array[10];
	
}P_out_filter;


void kalmanCreate(P_out_filter * p);
float KalmanFilter(kalman* p,float dat);
void Init_kal(void);
extern P_out_filter P_out_kalman;
#endif
