#ifndef __SYS_H
#define __SYS_H	
#include "stm32f1xx.h"

//定义一些常用的数据类型短关键字 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  		

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

void JTAG_Set(u8 mode);

////JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 	//输出为高电平		
#define digitalLo(p,i)		 {p->BRR=i;}	 	//输出低电平
#define digitalToggle(p,i) {p->ODR ^=i;} 		//输出反转状态
#define Lidar_Detect_ON						1				//电磁巡线是否开启雷达检测障碍物
#define Lidar_Detect_OFF					0

extern u8 Ros_Rate ;
extern volatile u8 Ros_count;
extern u8 Pick_up_stop;                       //检查是否被拿起标志位
extern int Middle_angle;                      //机械中值默认为0
extern u8 Lidar_Detect;
extern u8 Mode ;                                                    //模式选择，默认是普通的控制模式
extern u8 PS2_ON_Flag;		//默认所有方式不控制
extern float RC_Velocity,RC_Turn_Velocity;			//遥控控制的速度
extern u8 Way_Angle;                                       				 //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern int Motor_Left,Motor_Right;                                 //电机PWM变量 应是motor的 向moto致敬	
extern u16 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity,Target_Velocity; //蓝牙遥控相关的变量
extern u8 Flag_Stop,Flag_Show;                               			 //停止标志位和 显示标志位 默认停止 显示打开
extern int Voltage;               																 //电池电压采样相关的变量
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;     						 //平衡倾角 平衡陀螺仪 转向陀螺仪
extern int Temperature;
extern u32 Distance;                                          		//雷达测距
extern u16 determine;                                       //确定走直线的距离值
extern int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
extern float Move_X,Move_Z;
extern u8 Flag_follow,Flag_avoid,Flag_straight,delay_50,PID_Send;
extern volatile u8 delay_flag;
extern float Acceleration_Z;                       //Z轴加速度计  
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern float Distance_KP ,Distance_KD  ,Distance_KI ;	//距离调整PID参数
extern u8 CCD_Zhongzhi,CCD_Yuzhi;                 //线性CCD相关
extern u16 Angle_ADC;
extern u8 receive_buff[1682];                //定义接收数组
extern int Avoid_Flag;//遥控时雷达避障开启标志位
extern int Lidar_Online_Flag,Lidar_Online_Cnt;
extern volatile int init_cnt,APP_ON,Steady_Flag,time_cnt;
//////////////////////////////////////////////////////////////////////////////
//以下为汇编函数
void WFI_SET(void);		  //执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址

#include "KF.h"
#include "filter.h"
#include "IOI2C.h"
#include "Lidar.h"
#include "usart3.h"
#include "DataScope_DP.h"
#include "pstwo.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "oled.h"
#include "show.h"
#include "tim.h"
#include "usart.h"
#include "beep.h"
#include "control.h"
#include "key.h"
#include "led.h"
#include "delay.h"
#include <math.h>
#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif
