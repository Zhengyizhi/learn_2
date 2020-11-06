#include "pid.h"
//#define PIDcontrol_chassis1 0
//#define PIDcontrol_chassis2 2
//#define PIDcontrol_chassis3 4
//#define PIDcontrol_chassis4 6
//#define PIDcontrol_gyro_condition 8
//#define PIDcontrol_gyrospeed_downgimble 11
//#define PIDcontrol_gyrospeed_upgimble 12

//#define PIDcontrol_mechangle_downgimble 13
//#define PIDcontrol_mechangle_upgimble 14
//#define PIDcontrol_gyroangle_downgimble 15
//#define PIDcontrol_gyroangle_upgimble 16

//#define PIDcontrol_mechspeed_Dial 17
//#define PIDcontrol_mechangle_Dial 18
///////////////////////0       1       2       3       4       5       6       7       8       9       10       11       12       13       14       15       16      17     18   19

float _KP[20]         ={6,     0,      6,      0,      6,      0,      6,      0,    130,      0,       0,      40,      13,     150,      10,      95,      0,      4,     6,0};
float _KI[20]         ={0.8,   0,    0.8,      0,    0.8,      0,    0.8,      0,      0,      0,       0,     1.1,     0.3,       0,       0,       0,      0,      0,     0,0};
float _KD[20]         ={8,     0,      8,      0,      8,      0,      8,      0,      0,      0,       0,       0,       0,       0,       0,       0,      0,      0,     0,0};
float KI_addlimit[20] ={5000,  0,      5000,   0,      5000,   0,      5000,   0,     1000,    0,       0,     33000,   33000,   1000,    30000,    1000,    0,     30000,  30000,0};
float Set_out_limit[20]={9000, 0,     9000,   0,     9000,   0,     9000,   0,     15000,   0,       0,     20000,   20000,   15000,     2000,    20000,  0,     3000,  3000,0};
float Blind[20]       ={5,     0,       5,     0,      5,      0,       5,     0,      2,      0,        0,      5,        2,       1,      2,       1,       0,      2,     2,0};
_Total_Pid PID_control;

	
	float pid_handle1(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	float detaerror;
	p->TurnTest[i].SetGoal = SetGoal;
	p->TurnTest[i].ActualVar = ActualVar;
	p->TurnTest[i].err = p->TurnTest[i].SetGoal - p->TurnTest[i].ActualVar;
	if(abs(p->TurnTest[i].err) <= p->TurnTest[i].blind_limit)
		p->TurnTest[i].err = 0;
	
	p->TurnTest[i].I_err += p->TurnTest[i].err;
	detaerror = p->TurnTest[i].err - p->TurnTest[i].err_pre;
	
	if(p->TurnTest[i].I_err >= p->TurnTest[i].I_err_limit)
		p->TurnTest[i].I_err = p->TurnTest[i].I_err_limit;
	if(p->TurnTest[i].I_err <= - p->TurnTest[i].I_err_limit)
		p->TurnTest[i].I_err = - p->TurnTest[i].I_err_limit;

	p->TurnTest[i].KPout = p->TurnTest[i].err * p->TurnTest[i].Kp;
	p->TurnTest[i].KIout = p->TurnTest[i].I_err * p->TurnTest[i].Ki;
	p->TurnTest[i].KDout = p->TurnTest[i].Kd * detaerror;
	
	p->TurnTest[i].err_pre = p->TurnTest[i].err;
	p->TurnTest[i].nextSet = p->TurnTest[i].KPout + p->TurnTest[i].KIout + p->TurnTest[i].KDout;
	
	if(p->TurnTest[i].nextSet >= p->TurnTest[i].nextSet_limit)
		p->TurnTest[i].nextSet = p->TurnTest[i].nextSet_limit;
	if(p->TurnTest[i].nextSet <= - p->TurnTest[i].nextSet_limit)
		p->TurnTest[i].nextSet = - p->TurnTest[i].nextSet_limit;
	
	return(p->TurnTest[i].nextSet);

}

float pid_handle2(_Total_Pid* p,char i,float err)
{
//	float detaerror;Ã»ÓÐKD
	p->TurnTest[i].err = err;
	if(abs(p->TurnTest[i].err) <= p->TurnTest[i].blind_limit)
		p->TurnTest[i].err = 0;
		
	p->TurnTest[i].I_err += p->TurnTest[i].err;
	if(p->TurnTest[i].I_err >= p->TurnTest[i].I_err_limit)
		p->TurnTest[i].I_err = p->TurnTest[i].I_err_limit;
	if(p->TurnTest[i].I_err <= - p->TurnTest[i].I_err_limit)
		p->TurnTest[i].I_err = - p->TurnTest[i].I_err_limit;
	
	p->TurnTest[i].KPout = p->TurnTest[i].err * p->TurnTest[i].Kp;
	p->TurnTest[i].KIout = p->TurnTest[i].I_err * p->TurnTest[i].Ki;
	
	p->TurnTest[i].nextSet = p->TurnTest[i].KPout + p->TurnTest[i].KIout;
	if(p->TurnTest[i].nextSet >= p->TurnTest[i].nextSet_limit)
		p->TurnTest[i].nextSet = p->TurnTest[i].nextSet_limit;
	if(p->TurnTest[i].nextSet <= - p->TurnTest[i].nextSet_limit)
		p->TurnTest[i].nextSet = - p->TurnTest[i].nextSet_limit;
	
	return(p->TurnTest[i].nextSet);

}

void PID_Init (_Total_Pid* p)
{
	char i=0;
	for(i=0;i<17;i++)///////////!!!!!!!!!!!!!!!!
	{
		p->TurnTest[i].ActualVar=0.0;
		p->TurnTest[i].err=0.0;
		p->TurnTest[i].err_pre=0.0;
		p->TurnTest[i].I_err=0.0;
		p->TurnTest[i].nextSet=0.0;
		p->TurnTest[i].SetGoal=0.0;
		p->TurnTest[i].KDout=0.0;
		p->TurnTest[i].KIout=0.0;
		p->TurnTest[i].KPout=0.0;
		p->TurnTest[i].Kp = _KP[i];
		p->TurnTest[i].Ki = _KI[i];
		p->TurnTest[i].Kd = _KD[i];
		p->TurnTest[i].I_err_limit = KI_addlimit[i];
		p->TurnTest[i].nextSet_limit=Set_out_limit[i];
		p->TurnTest[i].blind_limit = Blind[i];
	}
}
void PID_Init_17_18 (_Total_Pid* p)
{
	char i=17;
		for(i=17;i<19;i++)
	{
		p->TurnTest[i].ActualVar=0.0;
		p->TurnTest[i].err=0.0;
		p->TurnTest[i].err_pre=0.0;
		p->TurnTest[i].I_err=0.0;
		p->TurnTest[i].nextSet=0.0;
		p->TurnTest[i].SetGoal=0.0;
		p->TurnTest[i].KDout=0.0;
		p->TurnTest[i].KIout=0.0;
		p->TurnTest[i].KPout=0.0;
		p->TurnTest[i].Kp = _KP[i];
		p->TurnTest[i].Ki = _KI[i];
		p->TurnTest[i].Kd = _KD[i];
		p->TurnTest[i].I_err_limit = KI_addlimit[i];
		p->TurnTest[i].nextSet_limit=Set_out_limit[i];
		p->TurnTest[i].blind_limit = Blind[i];
	}

}

