#ifndef __MY_include_H
#define __MY_include_H

#define myconstrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define myabs(x) 					((x)>0? (x):(-(x)))


typedef enum{
	None = 0,
	Set = 1,
	ReSet = 2,
	
	speedset = 3,//云台专用
	angleset =4
}Circle_control;


typedef struct{
	Circle_control Stand_circle;
	Circle_control Speedrun_circle;
	Circle_control Turnaround_circle;//底盘大环
	
	Circle_control pitchgimble_circle;
	Circle_control yawgimble_circle;//云台环
}Circle_condition;

extern  Circle_condition Circle_analy;






#define 	nonecircle  ((uint8_t)0x0000)       
#define 	Standcircle  ((uint8_t)0x0001)               
#define 	Speedruncircle ((uint8_t)0x0002)            
#define 	Turnaroundcircle ((uint8_t)0x0003)
//Circle_name;




#endif
