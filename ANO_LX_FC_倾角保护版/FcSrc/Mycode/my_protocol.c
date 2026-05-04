#include "my_protocol.h"
#include "User_Task.h"
struct sdata received_data={0,0,140,0,0,0};
struct PID_inc height_PID;
struct PID_inc xy_PID;
u8 RxBuffer[256];//树莓派数据缓存
u8 LidarBuffer[256];//激光雷达数据缓存
u8 pi_receive_done_sign=0;//树莓派接收完成标志位
u8 lidar_receive_done_sign=0;//激光雷达接收完成标志位
s16 CSPX=0,CSPY=0;
u8 x_high=0;
u8 x_low=0;
u8 y_high=0;
u8 y_low=0;
u8 rol_h=0;
u8 rol_l=0;
u8 pit_h=0;
u8 pit_l=0;
u8 yaw_h=0;
u8 yaw_l=0;
u8 att_state=0;

// 发送缓存变量
static u8 tx_buf[15];  // 完整发送帧缓存
static u8 tx_stage = 0;


s16 integ_side=0x4000;
s16 t265_vel_x = 0, t265_vel_y = 0;
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len)//串口发送函数
{
	u16 i=0;
	while(i<len)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC )==RESET);
		USART_SendData(USARTx,*s);
		s++;
		i++;
	}
}
void pi_receive(u8 data)//树莓派接受协议 串口2
{
	static u8 state_1 = 0;
	if(state_1==0&&data==0xAA)	//帧头0xAA
	{
		state_1=1;
		RxBuffer[0]=data;
	}
	else if(state_1==1)	//帧类型字节
	{
		RxBuffer[1]=data;
		if(data == 0x01)		//T265速度帧: AA 01 vx_h vx_l vy_h vy_l FF
			state_1 = 2;
		else if(data == 0x02)	//操作指令帧: AA 02 task_sta com_x com_y com_z com_yaw next_task sp_side FF
			state_1 = 10;
		else
			state_1 = 0;		//未知帧类型
	}
	//--- T265速度帧 (0x01) ---
	else if(state_1==2)		{RxBuffer[2]=data; state_1=3;}	// vx_h
	else if(state_1==3)		{RxBuffer[3]=data; state_1=4;}	// vx_l
	else if(state_1==4)		{RxBuffer[4]=data; state_1=5;}	// vy_h
	else if(state_1==5)		{RxBuffer[5]=data; state_1=6;}	// vy_l
	else if(state_1==6&&data==0xFF)	//帧尾
	{
		RxBuffer[6]=data;
		t265_vel_x = ((s16)RxBuffer[2] << 8) | RxBuffer[3];
		t265_vel_y = ((s16)RxBuffer[4] << 8) | RxBuffer[5];
		state_1 = 0;
	}
	//--- 操作指令帧 (0x02) ---
	else if(state_1==10)	{RxBuffer[2]=data; state_1=11;}	// task_sta
	else if(state_1==11)	{RxBuffer[3]=data; state_1=12;}	// com_x
	else if(state_1==12)	{RxBuffer[4]=data; state_1=13;}	// com_y
	else if(state_1==13)	{RxBuffer[5]=data; state_1=14;}	// com_z
	else if(state_1==14)	{RxBuffer[6]=data; state_1=15;}	// com_yaw
	else if(state_1==15)	{RxBuffer[7]=data; state_1=16;}	// next_task
	else if(state_1==16)	{RxBuffer[8]=data; state_1=17;}	// sp_side
	else if(state_1==17&&data==0xFF)	//帧尾
	{
		RxBuffer[9]=data;
		received_data.sp_side   = RxBuffer[8];
		received_data.task_sta  = RxBuffer[2];
		received_data.com_x     = RxBuffer[3] - received_data.sp_side;
		received_data.com_y     = RxBuffer[4] - received_data.sp_side;
		received_data.com_z     = RxBuffer[5];
		received_data.com_yaw   = RxBuffer[6] - received_data.sp_side;
		received_data.next_task_sign = RxBuffer[7];
		pi_receive_done_sign    = 1;
		state_1 = 0;
	}
	else
	{
		state_1=0;
	}
}
void PID_init()
{
	height_PID.p=2;
	height_PID.i=1.4;
	height_PID.d=0.8;
	height_PID.actual=0;
	height_PID.target=0;
	height_PID.err_current=0;
	height_PID.err_last=0;
	height_PID.err_previous=0;
	xy_PID.p=1.5;
	xy_PID.i=0.9;
	xy_PID.d=0.6;
	xy_PID.actual=0;
	xy_PID.target=0;
	xy_PID.err_current=0;
	xy_PID.err_last=0;
	xy_PID.err_previous=0;
}
s16 height_set(u32 height,u16 height_set)
{
	s16 output=0;
	height_PID.actual=height;
	height_PID.target=height_set; 
	height_PID.err_current=height_PID.target-height_PID.actual;
	output=height_PID.p*(height_PID.err_current-height_PID.err_last)+height_PID.i*height_PID.err_current+height_PID.d*(height_PID.err_current-2*height_PID.err_last+height_PID.err_previous);
	height_PID.err_previous=height_PID.err_last;
	height_PID.err_last=height_PID.err_current;
	if (output>30 ) output=30;
	else if(output<-30) output=-30;
	return output;
}

// 计算校验和（字节1~12累加）
static u8 calc_checksum(u8 *buf)
{
    u16 sum = 0;
    for(u8 i=1; i<=12; i++) sum += buf[i];
    return (u8)(sum & 0xFF);
}

//void pi_send(void)
//{
//    if(tx_stage == 0)
//    {
//        // ========== 组装完整帧 ==========
//        tx_buf[0] = 0xAA;  // 帧头
//        tx_buf[1] = mission_stage;

//        // 姿态角
//        tx_buf[2] = (fc_att.st_data.rol_x100 >> 0) & 0xFF;
//        tx_buf[3] = (fc_att.st_data.rol_x100 >> 8) & 0xFF;
//        tx_buf[4] = (fc_att.st_data.pit_x100 >> 0) & 0xFF;
//        tx_buf[5] = (fc_att.st_data.pit_x100 >> 8) & 0xFF;
//        tx_buf[6] = (fc_att.st_data.yaw_x100 >> 0) & 0xFF;
//        tx_buf[7] = (fc_att.st_data.yaw_x100 >> 8) & 0xFF;
//        tx_buf[8] = fc_att.st_data.state;

//        // X/Y 数据
//        s16 x = ano_of.intergral_x + 0x4000;
//        s16 y = ano_of.intergral_y + 0x4000;
//        tx_buf[9]  = (x >> 0) & 0xFF;
//        tx_buf[10] = (x >> 8) & 0xFF;
//        tx_buf[11] = (y >> 0) & 0xFF;
//        tx_buf[12] = (y >> 8) & 0xFF;

//        // 校验和 + 帧尾
//        tx_buf[13] = calc_checksum(tx_buf);
//        tx_buf[14] = 0xFF;

//        tx_stage = 1;
//    }
//    else if(tx_stage >= 1 && tx_stage <= 15)
//    {
//        // 分段发送（每次1字节）
//        USART_SendData(USART2, tx_buf[tx_stage - 1]);
//        tx_stage++;
//    }
//    else
//    {
//        tx_stage = 0;  // 发送完成，复位
//    }
//}

void pi_send()
{
	static u8 stage=0;
	if(stage==0)
	{
		USART_SendData(USART2,0xAA);
		stage=1;
		s16 num = ano_of.intergral_x + 0x4000;
		x_high=(num >> 8) & 0xFF;
		x_low=num & 0xFF;
		s16 ynum = ano_of.intergral_y + 0x4000;
		y_high=(ynum >> 8) & 0xFF;
		y_low=ynum & 0xFF;
	}
	else if(stage==1)
	{
		USART_SendData(USART2,mission_stage);
		stage=2;
	}
	else if(stage==2)
	{

		USART_SendData(USART2,x_high);
		//USART_SendData(USART2,0x05);
		stage=3;
	}
	else if(stage==3)
	{
		USART_SendData(USART2,x_low);
		stage=4;
	}
	else if(stage==4)
	{

		USART_SendData(USART2,y_high);
		//USART_SendData(USART2,0x05);
		stage=5;
	}
	else if(stage==5)
	{
		USART_SendData(USART2,y_low);
		stage=6;
	}
	else if(stage==6)
	{
		USART_SendData(USART2,0xFF);
		stage=0;
	}
	else stage=0;
}

void my_spcal(s16 x,s16 y)
{
        CSPX = y*0.3 ;
        CSPY = x*0.3 ;
	if(CSPX>30) CSPX=30;
	if(CSPX<-30) CSPX=-30;
	if(CSPY>30) CSPY=30;
	if(CSPY<-30) CSPY=-30;
}
s16 xypid_set(s32 xy,s16 xy_set,u16 speedmax)
{
	s16 output=0;
	xy_PID.actual=xy;
	xy_PID.target=xy_set; 
	xy_PID.err_current=xy_PID.target-xy_PID.actual;
	output=xy_PID.p*(xy_PID.err_current-xy_PID.err_last)+xy_PID.i*xy_PID.err_current+xy_PID.d*(xy_PID.err_current-2*xy_PID.err_last+xy_PID.err_previous);
	xy_PID.err_previous=xy_PID.err_last;
	xy_PID.err_last=xy_PID.err_current;
	if (output>speedmax ) output=speedmax;
	else if(output<-speedmax) output=-speedmax;
	return output;
}