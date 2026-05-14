/*==========================================================================
 * ����    �������ɿ�״̬��������Ҫ��ҡ�˽���������У׼�ȣ�
 * ����ʱ�䣺2020-03-31 
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
#include "LX_FC_State.h"
#include "Drv_RcIn.h"
#include "ANO_DT_LX.h"
#include "LX_FC_Fun.h"

volatile _fc_state_st fc_sta;
_sticks_fun_st sti_fun;

//�����֣����
#define TOE_OUT \
	(rc_in.rc_ch.st_data.ch_[ch_1_rol] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1100)

//�����֣��ڰ�
#define TOE_IN \
	(rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] < 1100)

//�����֣�����+����
#define LD_LD \
	(rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1100 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] < 1100)

//�����֣�����+����
#define RD_RD \
	(rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] > 1900)

// ҡ�˴���У׼ˮƽ����
#define STICKS_CALI_HOR_REQ (LD_LD)
// ҡ�˴���У׼��������
#define STICKS_CALI_MAG_REQ (RD_RD)

// ҡ�˽�������requirement
#define STICKS_UNLOCK_REQ (TOE_OUT || TOE_IN)
// ҡ����������
#define STICKS_LOCK_REQ (STICKS_UNLOCK_REQ)
// ��������ʱ��,����
#define UNLOCK_HOLD_TIME_MS (1000)
// ��������ʱ��,����
#define LOCK_HOLD_TIME_MS (300)
// ��ǰ����/����״̬
#define UNLOCK_STATE (fc_sta.unlock_sta)

//
static u16 time_dly_cnt_ms;
static u8 unlock_lock_flag;

static void LX_Unlock_Lock_Check(float *dT_s)
{
	//�ж�yawҡ���Ƿ���»���
	if ((rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1400 && rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1600))
	{
		sti_fun.stick_mit_pos = 1;
		unlock_lock_flag = 1; //�����Ժ����ִ��һ�ν�����������
	}
	else
	{
		sti_fun.stick_mit_pos = 0;
	}
	//���Ԥ�������Ķ���
	if (rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1200 && (sti_fun.stick_mit_pos == 0))
	{
		sti_fun.pre_locking = 1;
	}
	else
	{
		sti_fun.pre_locking = 0;
	}
	//����
	if (unlock_lock_flag == 1) //ִ������
	{
		if (UNLOCK_STATE == 0)
		{
			if (STICKS_UNLOCK_REQ)
			{
				if (time_dly_cnt_ms < UNLOCK_HOLD_TIME_MS)
				{
					time_dly_cnt_ms += *(dT_s)*1e3f;
				}
				else
				{
					FC_Unlock(); //����
					time_dly_cnt_ms = 0;
					unlock_lock_flag = 0; //����ִ��
				}
			}
			else
			{
				time_dly_cnt_ms = 0;
			}
		}
		else if (UNLOCK_STATE == 1)
		{
			if (STICKS_LOCK_REQ)
			{
				if (time_dly_cnt_ms < LOCK_HOLD_TIME_MS)
				{
					time_dly_cnt_ms += *(dT_s)*1e3f;
				}
				else
				{
					FC_Lock(); //����
					time_dly_cnt_ms = 0;
					unlock_lock_flag = 0; //����ִ��
				}
			}
			else
			{
				time_dly_cnt_ms = 0;
			}
		}
	}
	else
	{
		//null
	}
}

void LX_Cali_Trig_Check()
{
	static u8 cali_f;
	//Ϊ����״̬��ִ��
	if (UNLOCK_STATE == 0)
	{
		//ִ������
		if (STICKS_CALI_HOR_REQ)
		{
			//���ִֻ��һ��
			if (cali_f == 0)
			{
				Horizontal_Calibrate();
				cali_f = 1;
			}
		}
		else if (STICKS_CALI_MAG_REQ)
		{
			if (cali_f == 0)
			{
				Mag_Calibrate();
				cali_f = 1;
			}
		}
		else
		{
			cali_f = 0;
		}
	}
}

void LX_FC_State_Task(float dT_s)
{
	//��ң���źŲ�ִ��
	if (rc_in.fail_safe == 0)
	{
		//
		LX_Unlock_Lock_Check(&dT_s);
		//
		LX_Cali_Trig_Check();
	}
}
