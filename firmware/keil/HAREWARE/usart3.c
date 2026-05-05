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
#include "usart3.h"
#include "Lidar.h"
/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                 //从串口3读取的数据
u8 temp_data;											//串口5接收中断数据存放的缓冲区



/**********************jbr修改接受执行动作的两个浮点数***********************/
#define FLOAT_PACK_SIZE 10
u8 float_buffer[FLOAT_PACK_SIZE];
u8 float_index = 0;
float val1 = 0.0f, val2 = 0.0f;

extern volatile bool answer_flag;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //接收回调函数
{
	if(UartHandle->Instance == UART5){
		static u8 state = 0;//状态位	
		static u8 crc_sum = 0;//校验和
		static u8 cnt = 0;//用于一帧16个点的计数	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//头固定
				{
					Pack_Data.header_0= temp_data;
					state++;
					//校验
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//头固定
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//字长固定
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//速度高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//速度低八位
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//开始角度高八位
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//开始角度低八位
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				Pack_Data.point[cnt].distance_h = temp_data;//16个点的距离数据，高字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				Pack_Data.point[cnt].distance_l = temp_data;//16个点的距离数据，低字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				Pack_Data.point[cnt].Strong = temp_data;//16个点的强度数据
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			
			case 55:
				Pack_Data.end_angle_h = temp_data;//结束角度的高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 56:
				Pack_Data.end_angle_l = temp_data;//结束角度的低八位
				state++;
				crc_sum += temp_data;
				break;
			case 57:
				Pack_Data.crc = temp_data;//校验
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//数据处理，校验正确不断刷新存储的数据
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//清零
				}
				crc_sum = 0;//校验和清零
				break;
			default: break;
		}
	}
	
	
	
	
	// AA + 4+4 + 校验
	
	if(UartHandle->Instance == USART3)
	{

		static u8 float_state = 0;
		 // 新增浮点接收状态机
    if(float_state == 0) {
        if(Usart3_Receive_buf[0] == 0xAA) {
            float_state = 1;
            float_index = 0;
            memset(float_buffer, 0, FLOAT_PACK_SIZE);
					
						float_buffer[float_index++] = Usart3_Receive_buf[0];
        }
    } 
    else if(float_state == 1) {
        float_buffer[float_index++] = Usart3_Receive_buf[0];
        
        // 完整接收后校验
        if(float_index >= FLOAT_PACK_SIZE) {
            // 计算校验和
            u8 checksum = 0;
            for(int i=0; i<FLOAT_PACK_SIZE-1; i++){
                checksum += float_buffer[i];
            }
            
            // 校验通过则解析
            if(checksum == float_buffer[FLOAT_PACK_SIZE-1]) {
							memcpy(&val1, &float_buffer[1], 4);
							memcpy(&val2, &float_buffer[5], 4);
							answer_flag = true;
                
            }
            
            float_state = 0; // 复位状态机
        }
		}
				
				
		
		
		static	int uart_receive=0;//蓝牙接收相关变量
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
		uart_receive=Usart3_Receive_buf[0]; 
		Usart3_Receive=uart_receive;
		if(!AT_Command_Capture(Usart3_Receive_buf[0])&&time_cnt==1000){ 
			if(init_cnt<3) init_cnt++,APP_ON=0;//开机5s后接收到3帧有效蓝牙数据标定当前差值
			else init_cnt=100,APP_ON=1;
		}
		if(uart_receive==0x59)  Flag_velocity=2;  //低速挡（默认值）
		if(uart_receive==0x58)  Flag_velocity=1;  //高速档
		if(uart_receive=='b') Avoid_Flag=!Avoid_Flag;
		if(uart_receive>10)  //默认使用
		{			
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
		}
		if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
		{			
			Flag_velocity=1;//切换至高速档
			if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
														Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}	

		
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //分析数据
		 {
			  if(Receive[3]==0x50) 				 PID_Send=1;
			  else if(Receive[1]!=0x23) 
				{								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					{
						case 0x30:  Moto_Kp=Data;break;
						case 0x31:  Moto_Ki=Data;break;
						case 0x32:  break;
						case 0x33:  break;
						case 0x34:  break; 
					  case 0x35:    break; 
						case 0x36:  break; //预留
						case 0x37:  break; //预留
						case 0x38:  break; //预留
					}
				}				 
			    Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//数组清零
		 } 	
		 
		HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
	}
	
}




//蓝牙AT指令抓包，防止指令干扰到机器人正常的蓝牙通信
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	蓝牙链接时发送的字符，00:11:22:33:44:55为蓝牙的MAC地址
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	共44个字符
	
	蓝牙断开时发送的字符
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	共34个字符
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //蓝牙接受时指针记录器
	static u8 bt_line = 0; //表示现在在第几行
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//断开连接
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//开始连接
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//特殊标识符，开始警惕(使用时要-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //收到‘+’，表示切换了行数	
		disconnect++,connect++;
		return 1;//抓包，禁止控制
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//开始追踪数据是否符合断开的特征，符合时全部屏蔽，不符合时取消屏蔽
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//抓包，禁止控制
		}			

		//追踪连接特征 (bt_line==1&&connect>=13)区段是蓝牙MAC地址，每一个蓝牙MAC地址都不相同，所以直接屏蔽过去
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//抓包，禁止控制
		}	

		//在抓包期间收到其他命令，停止抓包
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//非禁止数据，可以控制
		}			
	}
	
	return 0;//非禁止数据，可以控制
}







