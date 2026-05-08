/*==========================================================================
 * ����    �������ɿ����ô���������
 * ����ʱ�䣺2020-02-06 
 * ����		 �������ƴ�-Jyoun
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ��Ŀ������18084888982��18061373080
============================================================================
 * �����ƴ��ŶӸ�л��ҵ�֧�֣���ӭ��ҽ�Ⱥ���ཻ�������ۡ�ѧϰ��
 * �������������в��õĵط�����ӭ����ש�������
 * �������������ã�����������Ƽ���֧�����ǡ�
 * ������Դ������뻶ӭ�������á��������չ��������ϣ������ʹ��ʱ��ע��������
 * ����̹������С�˳����ݣ��������������ˮ���������ӣ�Ҳ��δ�й�Ĩ��ͬ�е���Ϊ��  
 * ��Դ���ף�����������ף�ϣ����һ������ء����ﻥ������ͬ������
 * ֻ������֧�֣������������ø��á�  
===========================================================================*/
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "my_protocol.h"
extern u8 pi_ctrl_mode;
extern s16 t265_vel_x, t265_vel_y;

_fc_ext_sensor_st ext_sens;

//����ѹ������ݴ����ͨ���ٶȴ���������
static inline void General_Velocity_Data_Handle()
{
	static u8 of_update_cnt, of_alt_update_cnt;
	static u8 dT_ms = 0;
	//ÿһ����dT_ms+1�������ж��Ƿ�ʱ��������
	if (dT_ms != 255)
	{
		dT_ms++;
	}
	// T265 mode: send every 1ms, independent of OF update
	if (pi_ctrl_mode == 1)
	{
		ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = t265_vel_x;
		ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = t265_vel_y;
//		flex_send_t265_vel();
//		flex_send_guangliu_vel(); //�Զ�������ʽ֡���ͣ����ڹ۲����ݵ�ֵ
	}
	// T265 yaw complementary filter (~10Hz, gentle correction)
	{
		static u8 yaw_corr_cnt = 0;
		yaw_corr_cnt++;
		if (yaw_corr_cnt >= 100)
		{
			yaw_corr_cnt = 0;
			if (pi_ctrl_mode == 1)
			{
				s16 diff = t265_yaw_angle - fc_att.st_data.yaw_x100;
				if (diff > 18000) diff -= 36000;
				else if (diff < -18000) diff += 36000;
				fc_att.st_data.yaw_x100 += diff * 0.03f;
			}
		}
	}
	//���OF�����Ƿ����
	else if (of_update_cnt != ano_of.of_update_cnt)
	{
		of_update_cnt = ano_of.of_update_cnt;
		if (ano_of.of1_sta && ano_of.work_sta) //������Ч
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
		}
		else //��Ч
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
		}
	}
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//������z���ٶȣ���z�ٶȸ�ֵΪ��Ч
		ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000;
		//��������
		dt.fun[0x33].WTS = 1;
		//reset
		dT_ms = 0;
	}
}

static inline void General_Distance_Data_Handle()
{
	static u8 of_alt_update_cnt;
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//
		ext_sens.gen_dis.st_data.direction = 0;
		ext_sens.gen_dis.st_data.angle_100 = 270;
		ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
		//��������
		dt.fun[0x34].WTS = 1;
	}
}

void LX_FC_EXT_Sensor_Task(float dT_s) //1ms
{
	//
	General_Velocity_Data_Handle();
	//
	General_Distance_Data_Handle();
}
