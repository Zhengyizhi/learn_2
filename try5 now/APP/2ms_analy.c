#include "2ms_analy.h"
 float i=0,j=0;
 float record_CAN1pre=0;
 float record_CAN1now=0;
 float record_DMApre=0;
 float record_DMAnow=0;
extern _TotalFlag AllFlag;
extern float DMAinterruptnumber;
extern float CAN1interruptnumber;
void analy_Flag(void)
{
	record_DMAnow=DMAinterruptnumber;
	record_CAN1now=CAN1interruptnumber;
	if(record_DMApre!=record_DMAnow)
	{
		i=0;
		AllFlag.DMAinterrupt_Flag = _Normal;
	}
	if(record_DMApre==record_DMAnow)
	{
		i++;
		if(i>24)//Ò£¿ØÆ÷Ê§Áª
		{
			AllFlag.DMAinterrupt_Flag = _disconnected;
		}
	}
	record_DMApre=record_DMAnow;
	
	if(record_CAN1pre!=record_CAN1now)
	{
		j=0;
		AllFlag.Can1interrupt_Flag = _Normal;
	}
	if(record_CAN1pre==record_CAN1now)
	{
		j++;
		if(j>10)
		{
			AllFlag.Can1interrupt_Flag = _disconnected;
		}
	}
	record_CAN1pre=record_CAN1now;
	
}
	

