#include "can2.h"
#include "Varstruct.h"
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/


void CAN2_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
//		nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic); 

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0 CAN_Filter_FIFO0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
} 

void CAN2_RX0_IRQHandler(void)
{
		CanRxMsg RxMessage1;
		if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {				 
				CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
				CAN_Receive(CAN2, CAN_FIFO0, &RxMessage1);
			SpeedAnalyCAN2(RxMessage1.StdId,RxMessage1.Data);
				
		}
}

u8 CAN2_Send_Dial(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1ff;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
	{
		return 1;
	}
  return 0;		
}
char Receive_alldata;
extern  _TotalFlag AllFlag;
extern  _Total_Receive Alldata;
u8 SpeedAnalyCAN2(u16 id,u8 *Recbuf)
{
	switch (id)
	{
		case 0x207 :Receive_alldata = 8;break;
		default :Receive_alldata = 9;return 0;
	}
		Alldata.Redata[Receive_alldata].NowAngle=(short int)(Recbuf[0]<<8 | Recbuf[1]);
		Alldata.Redata[Receive_alldata].NowSpeed=(short int)(Recbuf[2]<<8 | Recbuf[3]);
		Alldata.Redata[Receive_alldata].Nowtorque=(short int)(Recbuf[4]<<8 | Recbuf[5]);
	//	Alldata.Redata[Receive_alldata].Nowtemper=Recbuf[6];//电机传输
	return 1;
}





