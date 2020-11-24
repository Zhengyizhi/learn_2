#ifndef __2ms_analy_H
#define __2ms_analy_H
#include "system.h"


typedef enum{
	_none = 1,
	_normal =2,
	_disconnected = 3,
	_error = 4
}interrupt_Flag;//interrupt


typedef struct{
	interrupt_Flag DMAinterrupt_Flag;
	interrupt_Flag Can1interrupt_Flag;
	interrupt_Flag RemoteData_Flag;
	interrupt_Flag Stand_Flag;
}conditionFlag;//

extern conditionFlag AllFlag;
void analy_Flag(void);


#endif

