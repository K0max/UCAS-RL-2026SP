/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
  
#include "beep.h"   

/**************************************************************************
Function: Buzzer initialization
Input   : none
Output  : none
函数功能：蜂鸣器初始化
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void BEEP_GPIO_Config(void)
{		

	
//	/*关闭JTAG接口*/
//	JTAG_Set(JTAG_SWD_DISABLE);    
//	/*打开SWD接口 可以利用主板的SWD接口调试*/
//	JTAG_Set(SWD_ENABLE);           
	/* 关闭蜂鸣器*/
	BEEP_OFF;
}



/**************************************************************************
Function: Buzzer_Alarm
Input   : Indicates the count of frequencies
Output  : none
函数功能：蜂鸣器报警
入口参数: 指示频率的计数 
返回  值：无
**************************************************************************/	 	
////在中断函数调用
void Buzzer_Alarm(u16 count)
{
	static int count_time;
	if(0 == count)
	{
		BEEP_OFF;
	}
	else if(++count_time >= count)
	{
		BEEP_TOGGLE;
		count_time = 0;	
	}
}




/*********************************************END OF FILE**********************/
