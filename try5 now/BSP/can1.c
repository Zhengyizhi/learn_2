#include "can1.h"

void CAN1_Init()
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef filter;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);

	gpio.GPIO_Mode=GPIO_Mode_AF;
	gpio.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_11;
	GPIO_Init(GPIOA,&gpio);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);

	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;//主优先级
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	can.CAN_TTCM = DISABLE;		//非时间触发通信模式
	can.CAN_ABOM = DISABLE;		//软件自动离线管理
	can.CAN_AWUM = DISABLE;		//睡眠模式通过软件唤醒(清楚CAN->MCR的SLEEP位)
	can.CAN_NART = DISABLE;		//禁止报文自动传送 若七个电机接一个CAN 会影响发送 此时可改为ENABLE
	can.CAN_RFLM = DISABLE;		//报文不锁定，新的覆盖旧的
	can.CAN_TXFP = ENABLE;		//优先级由报文标识符决定
	can.CAN_BS1=CAN_BS1_11tq;         //CAN_BS1_9tq;
	can.CAN_BS2=CAN_BS2_2tq;                 //CAN_BS2_4tq;
	can.CAN_Mode=CAN_Mode_Normal;
	can.CAN_Prescaler=3;
	can.CAN_SJW=CAN_SJW_1tq;
	CAN_Init(CAN1,&can);
	
	filter.CAN_FilterNumber=0;  							 			//过滤器0
	filter.CAN_FilterMode=CAN_FilterMode_IdMask;   	//屏蔽模式
	filter.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位宽
	filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;              //过滤器0关联到FIFO0
	filter.CAN_FilterActivation=ENABLE;   				  //激活过滤器
	filter.CAN_FilterIdHigh=0x0000;                 //32位ID
	filter.CAN_FilterIdLow=0x0000;
	filter.CAN_FilterMaskIdHigh=0x0000;             //32位Mask
	filter.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInit(&filter);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
}


u8 CAN1_Send_Msg_chassis(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
	{
		return 1;
	}
  return 0;		
}
u8 CAN1_Send_Msg_crable(u8* msg,u8 len)
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
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
	{
		return 1;
	}
	
  return 0;		

}
static char ReceiveFlag=20;
static char AngletiaocanFlag;
extern _Total_Receive Alldata;
u8 SpeedAnaly(u16 id,u8 *Recbuf)
{
		switch(id)//电机传输	
		{
			case 0x205:ReceiveFlag=4;AngletiaocanFlag=13;break;
			case 0x206:ReceiveFlag=5;AngletiaocanFlag=14;break;//Alldata.Redata[5]
			case 0x201:ReceiveFlag=0;AngletiaocanFlag=1;break;
			case 0x202:ReceiveFlag=1;AngletiaocanFlag=3;break;
			case 0x203:ReceiveFlag=2;AngletiaocanFlag=5;break;
			case 0x204:ReceiveFlag=3;AngletiaocanFlag=7;break;
			default:ReceiveFlag = 9;return 0;//未知ID
		}
		Alldata.Redata[ReceiveFlag].NowAngle=(short int)(Recbuf[0]<<8 | Recbuf[1]);
		Alldata.Redata[ReceiveFlag].NowSpeed=(short int)(Recbuf[2]<<8 | Recbuf[3]);
		Alldata.Redata[ReceiveFlag].NowCurrent=(short int)(Recbuf[4]<<8 | Recbuf[5]);
		Alldata.Redata[ReceiveFlag].Nowtemper=Recbuf[6];//电机传输
		return 1;
}


float CAN1interruptnumber;
void CAN1_RX0_IRQHandler()
{
	CanRxMsg RxMessage;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
		CAN1interruptnumber++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);		//清楚中断挂起
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);		//接收can数据
		SpeedAnaly(RxMessage.StdId,RxMessage.Data);
	}
	
}

//////////////!!!!!!!!!!preangle重置一次
void AngleAnaly(char alldatanum,float jump,float center,float degree)
{
	Alldata.Redata[alldatanum].detaNowAngle = Alldata.Redata[alldatanum].NowAngle - Alldata.Redata[alldatanum].preAngle;
	if(Alldata.Redata[alldatanum].detaNowAngle < -jump)//正转跃变
		Alldata.Redata[alldatanum].longtimecircle ++;//代表转了一圈
	else if (Alldata.Redata[alldatanum].detaNowAngle > jump)//反转跃变
		Alldata.Redata[alldatanum].longtimecircle --;//少了一圈
	Alldata.Redata[alldatanum].longtimeangle = -degree * Alldata.Redata[alldatanum].longtimecircle + ( center - Alldata.Redata[alldatanum].NowAngle );	
Alldata.Redata[alldatanum].preAngle =  Alldata.Redata[alldatanum].NowAngle;
}

//void CAN1_TX_IRQHandler()
//{
//	if(CAN_GetITStatus!=RESET)
//		{
//			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//		}
//}


