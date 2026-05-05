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

#include "Lidar.h"
#include <string.h>
#include "sys.h"



PointDataProcessDef PointDataProcess[225] ;//更新225个数据
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[225];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据

 u8 Lidar_data_buff[29][58]={0};
int Lidar_Deal_Flag=0;
 u8 receive_buff[1682];                //DMA搬运数据缓冲区
/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
//完成一帧接收后进行处理
void data_process(void) //数据处理
{
	static u8 data_cnt = 0;
	u8 i,m,n;
	u32 distance_sum[8]={0};//2个点的距离和的数组
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//计算16个点的开始角度
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//计算16个点的结束角度
	float area_angle[8]={0};//一帧数据的8个平均角度
	if((start_angle>350)&&(end_angle<13))//因为一帧数据是13度，避免350到10这段范围相加，最后angle反而变成180范围
		end_angle +=360;
	for(m=0;m<8;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/8*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	for(i=0;i<16;i++)
	{
		switch(i)
		{
			case 0:case 1:
				distance_sum[0] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//0~1点的距离和
			  break;
			case 2:case 3:
				distance_sum[1] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//2~3点的距离和
			  break;
			case 4:case 5:
				distance_sum[2] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//4~5点的距离和
			  break;
			case 6:case 7:
				distance_sum[3] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//6~7点的距离和
			  break;
			case 8:case 9:
				distance_sum[4] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//8~9点的距离和
			  break;
			case 10:case 11:
				distance_sum[5] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//10~11点的距离和
			  break;
			case 12:case 13:
				distance_sum[6] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//12~13点的距离和
			  break;
			case 14:case 15:
				distance_sum[7] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//14~15点的距离和
			  break;
			default:break;
								
		}
		  
	}
	for(n=0;n<8;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	  PointDataProcess[data_cnt+n].distance = distance_sum[n]/2;
	}
	data_cnt +=8;
	if(data_cnt>=225)
	{
		for(i=0;i<225;i++)
		{
			Dataprocess[i].angle=PointDataProcess[i].angle;
			Dataprocess[i].distance=PointDataProcess[i].distance;
		}
		data_cnt = 0;
	}
		
}


///**************************************************************************
//Function: Distance_Adjust_PID
//Input   : Current_Distance;Target_Distance
//Output  : OutPut
//函数功能：雷达距离pid
//入口参数: 当前距离和目标距离
//返回  值：电机目标速度
//**************************************************************************/	 	
////雷达距离调整pid
//float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
//{
//	static float Bias,OutPut,Last_Bias;
//	Bias=Target_Distance-Current_Distance;                          	//计算偏差
//	OutPut=-Distance_KP*Bias/10000-Distance_KD*(Bias-Last_Bias)/10000;//位置式PID控制器  //
//	Last_Bias=Bias;                                       		 			//保存上一次偏差
//	return OutPut;                                          	
//}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：雷达跟随距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	

float Lidar_follow_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	OutPut=-0.5*Bias/1000-0.1*(Bias-Last_Bias)/1000;//位置式PID控制器  //
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/	 	
//雷达转向pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)		                                 				 //求出偏差的积分
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //计算偏差
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=-0.01*Bias-0.0001*Integral_bias-0.0001*(Bias-Last_Bias);	//位置式PID控制器
	Last_Bias=Bias;                                       					 		//保存上一次偏差
	if(Turn_Off(Angle_Balance,Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                           					 	//输出
}


void Lidar_data_Receive(void){
	static int j=0;
	for(int i=0;i<58;i++){
		Lidar_data_buff[j][i]=receive_buff[i];
	}
	if(j==28) Lidar_Deal_Flag=1,Lidar_Online_Flag=1,Lidar_Online_Cnt=0;//数据可处理，雷达在线，雷达失联计数器
	j++;j%=29;
}

/**************************************************************************
Function: Lidar lap data processing function
Input   : none
Output  : none
函数功能：雷达一圈数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_data_Deal(void){//雷达点云一圈数据处理
	int i=0,j=0;
	static u8 crc_sum = 0;//校验和
	static u8 cnt = 0;//用于一帧16个点的计数
	for(j=0;j<29;j++)//一圈
	{
		for(i=0;i<58;i++)//一帧
		{
			u8 temp_data=Lidar_data_buff[j][i];//缓存区里读数据
			switch(i%58)
			{
			case 0://头固定
					Pack_Data.header_0= temp_data;
					//校验
					crc_sum += temp_data;
				break;
			case 1://头固定
					Pack_Data.header_1 = temp_data;
					crc_sum += temp_data;
				break;
			case 2:
					//字长固定
					Pack_Data.ver_len = temp_data;
					crc_sum += temp_data;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//速度高八位
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//速度低八位
				crc_sum += temp_data;
				break;
				case 5:
					Pack_Data.start_angle_h = temp_data;//开始角度高八位
					crc_sum += temp_data;
					break;
				case 6:
					Pack_Data.start_angle_l = temp_data;//开始角度低八位
					crc_sum += temp_data;
					break;
				
				case 7:case 10:case 13:case 16:
				case 19:case 22:case 25:case 28:
				case 31:case 34:case 37:case 40:
				case 43:case 46:case 49:case 52:
					Pack_Data.point[cnt].distance_h = temp_data;//16个点的距离数据，高字节
					crc_sum += temp_data;
					break;
				
				case 8:case 11:case 14:case 17:
				case 20:case 23:case 26:case 29:
				case 32:case 35:case 38:case 41:
				case 44:case 47:case 50:case 53:
					Pack_Data.point[cnt].distance_l = temp_data;//16个点的距离数据，低字节
					crc_sum += temp_data;
					break;
				
				case 9:case 12:case 15:case 18:
				case 21:case 24:case 27:case 30:
				case 33:case 36:case 39:case 42:
				case 45:case 48:case 51:case 54:
					Pack_Data.point[cnt].Strong = temp_data;//16个点的强度数据
					crc_sum += temp_data;
					cnt++;
					break;
				
				case 55:
					Pack_Data.end_angle_h = temp_data;//结束角度的高八位
					crc_sum += temp_data;			
					break;
				case 56:
					Pack_Data.end_angle_l = temp_data;//结束角度的低八位
					crc_sum += temp_data;
					break;
				case 57:
					Pack_Data.crc = temp_data;//校验
					cnt = 0;
					if(crc_sum == Pack_Data.crc)
					{
						data_process();//一帧数据处理，校验正确不断刷新存储的数据
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
	}
}



void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{	// 判断是否是串口1
    if(UART4 == huart4.Instance)                                   
    {	// 判断是否是空闲中断
        if(RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))   
        {	 // 清除空闲中断标志（否则会一直不断进入中断）
            __HAL_UART_CLEAR_IDLEFLAG(&huart4);                    
			// 停止本次DMA传输
			HAL_UART_DMAStop(&huart4);                 
			// 计算接收到的数据长度
			uint8_t data_length  = 255 - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);  
			//从缓冲区读取一帧雷达数据
			Lidar_data_Receive();
			// 清零接收缓冲区
			memset(receive_buff,0,data_length);                                            
			data_length = 0;
			// 重启开始DMA传输 每次255字节数据
			HAL_UART_Receive_DMA(&huart4, (uint8_t*)receive_buff, 255);  
        }
    }
}





