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
#ifndef __USRAT3_H
#define __USRAT3_H 

#include "sys.h"

extern u8 Usart3_Receive;
extern u8 Usart3_Receive_buf[1];
extern u8 temp_data;
u8 AT_Command_Capture(u8 uart_recv);








#endif

