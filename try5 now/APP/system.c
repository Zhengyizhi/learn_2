#include "system.h"

extern int SystemMode;
static volatile uint32_t usTicks = 0;

uint32_t currentTime = 0;
uint32_t loopTime_1ms=0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0; 

short gyrox,gyroy,gyroz;	//陀螺仪原始数据
float pitch,roll,yaw,yaw_10;		//欧拉角
	
//限幅
float constrain(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int constrain_int(int amt,int low,int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

//计数器初始化
static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000; 
}

//以微秒为单位返回系统时间
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
			ms = sysTickUptime;
			cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//微秒级延时
void delay_us(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

//毫秒级延时
void delay_ms(uint32_t ms)
{
	while (ms--)
			delay_us(1000);
}

//以毫秒为单位返回系统时间
uint32_t millis(void)
{
	return sysTickUptime;
}

//系统初始化
void systemInit(void)
{
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	//滴答定时器配置，1ms
}

int SystemMonitor=Normal_Mode;
void Stop()
{
		
}

void Parameter_Init(void)
{      
	
}

int pass_num;
bool pass_flag=1;
void System_Init(void)
{	
	static uint32_t loopTime_mpu6050 = 0;
	CRC_init();	
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);//滴答定时器配置，1ms
	
	Parameter_Init();
	CAN1_Init();
	CAN2_Init();
	RC_Init();
	Led_Init();
	MPU_Init();
	
	UART5_Init();
	
	while(mpu_dmp_init())//注意自检函数
	{
		currentTime = micros();//获取当前系统时间	
		if((int32_t)(currentTime - loopTime_mpu6050) >= 100000)  
		{	
			loopTime_mpu6050 = currentTime + 100000;			//100ms
			pass_num++;
			if(pass_num>=3)//若超时 则屏蔽自检函数
			{
				pass_flag=0;
				pass_num=10;
			}
		}
	}
}
float realxspeed;
float realzspeed;
//主循环
void Loop(void)
{	
	static uint32_t currentTime = 0;
	static uint32_t loopTime_1ms = 0;
	static uint32_t loopTime_2ms = 0;
	
	currentTime = micros();	//获取当前系统时间
	
	if((int32_t)(currentTime - loopTime_1ms) >= 0)  
	{	
		loopTime_1ms = currentTime + 1000;	//1ms		
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//读取角速度
		mpu_dmp_get_data(&roll,&pitch,&yaw);		//读取欧拉角
		realxspeed = gyrox + 22;
		realzspeed = gyroz + 17;//飘 
	}
	
	if((int32_t)(currentTime - loopTime_2ms) >= 0)  
	{	
		loopTime_2ms = currentTime + 2000;	//2ms
		analy_Flag();//判断电机和遥控器状态		
		SentAllData_CAN1();
	}
}








