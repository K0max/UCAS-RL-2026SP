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
#ifndef __BEEP_H
#define	__BEEP_H

#include "sys.h"



/* 定义蜂鸣器连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的蜂鸣器引脚 */
#define BEEP_GPIO_PORT    	GPIOA			              	/* GPIO端口 */
#define BEEP_GPIO_PIN				GPIO_PIN_15			        	/* 连接到蜂鸣器的GPIO */


/* 带参宏，可以像内联函数一样使用 */
#define BEEP(a)	if (a)	\
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN);\
					else		\
					GPIO_ResetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN)

					
/* 定义控制IO的宏 */
#define BEEP_TOGGLE		    digitalToggle(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_ON		   		digitalHi(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_OFF			digitalLo(BEEP_GPIO_PORT,BEEP_GPIO_PIN)


					
void BEEP_GPIO_Config(void);
void Buzzer_Alarm(u16 count);

					
#endif /* __BEEP_H */
