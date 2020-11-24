#include "pid.h"

//PID_Object pid_gimble_Array[4];//0-speeddown,1-angledown,2-speedup,3-angleup

//PID_Object pid_chassis_Array[5];

//chassis_pid_t pid_chassis_Array[2];
//gimble_pid_t pid_gimble_Array[2];




	float pid_handle1(PID_Object* p,float SetGoal,float ActualVar)
{
	float detaerror;
	p->target = SetGoal;
	p->feedback = ActualVar;
	p->error = p->target - p->feedback;
	if(abs(p->error) <= p->error_limit)
		p->error = 0;
	
	p->integrate += p->error;
	detaerror = p->error - p->pre_error;
	
	if(p->integrate >= p->integrate_max)
		p->integrate = p->integrate_max;
	if(p->integrate <= - p->integrate_max)
		p->integrate = - p->integrate_max;

	p->Pout = p->error * p->Kp;
	p->Iout = p->integrate * p->Ki;
	p->Dout = p->Kd * detaerror;
	
	p->pre_error = p->error;
	p->out = p->Pout + p->Iout + p->Dout;
	
	if(p->out >= p->out_max)
		p->out = p->out_max;
	if(p->out <= - p->out_max)
		p->out = - p->out_max;
	
	return(p->out);

}

float pid_handle2(PID_Object* p,float err)
{
//	float detaerror;没有KD
	p->error = err;
	if(abs(p->error) <= p->error_limit)
		p->error = 0;
		
	p->integrate += p->error;
	if(p->integrate >= p->integrate_max)
		p->integrate = p->integrate_max;
	if(p->integrate <= - p->integrate_max)
		p->integrate = - p->integrate_max;
	
	p->Pout = p->error * p->Kp;
	p->Iout = p->integrate * p->Ki;
	
	p->out = p->Pout + p->Iout;
	if(p->out >= p->out_max)
		p->out = p->out_max;
	if(p->out <= - p->out_max)
		p->out = - p->out_max;
	
	return(p->out);
}
/*调试PID*/
_Total_Pid Total_pid;
///////////////////////0       1       2       3       4       5       6       7       8       9       10       11       12       13       14       15       16      17     18   19

float _KP[10]         ={370,     1,    90,   0.0028,   395,      0,      0,      0,      0,      0 };   //   0,      25,      16,     200,      20,      150,      0.01,      6,     5,0};
float _KI[10]         ={0.11,     0,    0,  0.000014,     0,     0,      0,      0,      0,      0  };  //   0,     0.4,     0.4,       0,       0,       0,      0,      0.05,     0,0};
float _KD[10]         ={4,       0,      0,      0,      2.3,       0,      0,      0,      0,      0   }; //   0,       5,       0,       0,       0,       0,      0,      0,     0,0};
float KI_addlimit[10] ={18000,    0,    0,     50000,    0,       0,     0,      0,      0,    0   }; //   0,     33000,   33000,   1000,    30000,    1000,    0,     30000,  30000,0};
float Set_out_limit[10]={10000,  50,  10000,    45,    9000,    0,      0,     0,      0,     0    };// 0,     20000,   20000,   15000,     2000,    20000,     1,     7000,  7000,0};
float Blind[10]       ={0,       0,       0,     0,        0,     0,       0,     0,      0,      0   }; //   0,      0,        0,       1,      2,       1,       0,      2,     2,0};

void PID_Init (_Total_Pid* p)
{
	char i=0;
	for(i=0;i<10;i++)///////////!!!!!!!!!!!!!!!!
	{
		p->pid_test_Array[i].feedback=0.0;
		p->pid_test_Array[i].error=0.0;
		p->pid_test_Array[i].pre_error=0.0;
		p->pid_test_Array[i].integrate=0.0;
		p->pid_test_Array[i].out=0.0;
		p->pid_test_Array[i].target=0.0;
		p->pid_test_Array[i].Dout=0.0;
		p->pid_test_Array[i].Iout=0.0;
		p->pid_test_Array[i].Pout=0.0;
		p->pid_test_Array[i].detaerror = 0.0;
		p->pid_test_Array[i].Kp = _KP[i];
		p->pid_test_Array[i].Ki = _KI[i];
		p->pid_test_Array[i].Kd = _KD[i];
		p->pid_test_Array[i].integrate_max = KI_addlimit[i];
		p->pid_test_Array[i].out_max=Set_out_limit[i];
		p->pid_test_Array[i].error_limit = Blind[i];
	}
}

	float pid_handle1_test(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	float detaerror;
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);

}

	float pid_handle2_test(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	float detaerror;
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Pout = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],p->pid_test_Array[i].Pout);
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);

}


float balanSpeed(_Total_Pid* p,char i,float encoder_left,float encoder_right)
{
	p->pid_test_Array[i].target = (encoder_left - encoder_right) - 0;
	p->pid_test_Array[i].error = p->pid_test_Array[i].pre_error *0.7f + 0.3f * p->pid_test_Array[i].target;
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);
	
	p->pid_test_Array[i].Ki = p->pid_test_Array[i].Kp /200;
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout;
	

	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}

float balanstand(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuspeed,float mpuerror)
{
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
		if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].detaerror = mpuspeed+mpuerror;
	p->pid_test_Array[i].detaerror = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],p->pid_test_Array[i].detaerror);
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Dout = (p->pid_test_Array[i].detaerror)*p->pid_test_Array[i].Kd;
	
	
	
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + 	p->pid_test_Array[i].Dout + p->pid_test_Array[i].Iout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}



//float pid_handle2_test(_Total_Pid* p,char i,float err)
//{
////	float detaerror;没有KD
//	p->TurnTest[i].err = err;
//	if(abs(p->TurnTest[i].err) <= p->TurnTest[i].blind_limit)
//		p->TurnTest[i].err = 0;
//		
//	p->TurnTest[i].I_err += p->TurnTest[i].err;
//	if(p->TurnTest[i].I_err >= p->TurnTest[i].I_err_limit)
//		p->TurnTest[i].I_err = p->TurnTest[i].I_err_limit;
//	if(p->TurnTest[i].I_err <= - p->TurnTest[i].I_err_limit)
//		p->TurnTest[i].I_err = - p->TurnTest[i].I_err_limit;
//	
//	p->TurnTest[i].KPout = p->TurnTest[i].err * p->TurnTest[i].Kp;
//	p->TurnTest[i].KIout = p->TurnTest[i].I_err * p->TurnTest[i].Ki;
//	
//	p->TurnTest[i].nextSet = p->TurnTest[i].KPout + p->TurnTest[i].KIout;
//	if(p->TurnTest[i].nextSet >= p->TurnTest[i].nextSet_limit)
//		p->TurnTest[i].nextSet = p->TurnTest[i].nextSet_limit;
//	if(p->TurnTest[i].nextSet <= - p->TurnTest[i].nextSet_limit)
//		p->TurnTest[i].nextSet = - p->TurnTest[i].nextSet_limit;
//	
//	return(p->TurnTest[i].nextSet);

//}




/////////////////////////////////////////////////////////////////////////////////////
chassis_pid_t pid_chassis_Array[2]={
	{
		/*左轮*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	},
	{
		/*右轮*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	}
};

gimble_pid_t pid_gimble_Array[2] = {
	{
		/*yaw*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	},
	{
		/*pitch*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	}
};



