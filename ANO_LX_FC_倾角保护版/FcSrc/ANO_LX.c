#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "ANO_DT_LX.h"
#include "ANO_Math.h"
#include "Drv_PwmOut.h"
#include "LX_FC_State.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "Drv_adc.h"
#include "Drv_led.h"
#include "Drv_UbloxGPS.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h"
#include "User_Task.h"

/*==========================================================================
 * ����    �������ɿ����롢���������
 * ����ʱ�䣺2020-01-22 
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

_rt_tar_un rt_tar;
_pwm_st pwm_to_esc;
_fc_bat_un fc_bat;
_fc_att_un fc_att;
_fc_att_qua_un fc_att_qua;
_fc_vel_un fc_vel;

u8 pi_ctrl_mode = 0;	//������Դ��0=RC+���, 1=��ݮ��+T265

//ң��CH5(AUX1)ͨ��ֵ(1000-1500-2000)����ģʽ1-2-3��ģʽ0��Ҫͨ����������ָ������
//ģʽ0����̬����    ->ң��CH1-CH4ֱ�ӿ�����̬�����š�
//ģʽ1������+����   ->ң��CH1/CH2/CH3������̬������ң��CH3(����ҡ��)���ƴ�ֱ�����ٶȡ�
//ģʽ2������        ->ң��CH1/CH2����ˮƽ�����ٶȣ�����ң��CH3(����ҡ��)���ƴ�ֱ�����ٶ�,ң��CH4����YAW��̬��
//ģʽ3���̿�        ->ң��ҡ�˲��������

//
#define MAX_ANGLE 3500	//�����ʱ�Ƕȣ� ��λ 0.01��
#define MAX_YAW_DPS 200 //�����ʱYAW���ٶȣ���λ��ÿ��
//
#define MAX_VELOCITY 500  //�����ʱˮƽ�ٶȣ���λ����ÿ��
#define MAX_VER_VEL_P 300 //�����ʱ��ֱ���ٶȣ���λ����ÿ��
#define MAX_VER_VEL_N 200 //�����ʱ��ֱ���ٶȣ���λ����ÿ��

//////////////////////////////////////////////////////////////////////
//����Ϊ�ɿػ������ܳ��򣬲������û��Ķ��͵��á�
//////////////////////////////////////////////////////////////////////

//ң�������ݴ���
static inline void RC_Data_Task(float dT_s)
{
	static u8 fail_safe_change_mod, fail_safe_return_home;
	static u8 mod_f[3];
	static u16 mod_f_time_cnt;

	//ң��û��ʧ�ر�ǲ�ִ��
	if ( rc_in.fail_safe == 0)
	{
		//ҡ����������ģʽ����̬+��ѹ���ߣ����߶��㣬�̿أ�
		//ע�⣬�̿�ģʽ�£��ɿ�ֻ��Ӧ����ָ���ṩ���źţ�������Ӧҡ�ˡ�
		if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1200)
		{
			LX_Change_Mode(1);
			mod_f[0] = 1;
		}
		else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1700)
		{
			LX_Change_Mode(2);
			mod_f[0] = 2;
			pi_ctrl_mode = 0;
		}
		else
		{
			LX_Change_Mode(2);
			mod_f[0] = 2;
			pi_ctrl_mode = 1;
			if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1400)
				pi_ctrl_mode = 0;
		}
		//���л�ģʽʱִ��
		if (mod_f[1] != mod_f[0])
		{
			mod_f[1] = mod_f[0];
			//�����ģʽ3������һ�Ρ�
			if (mod_f[0] == 3)
			{
				mod_f[2]++;
			}
		}
		//�˶γ�����ʱ2000ms�ڼ���л��̿�ģʽ�Ĵ���,�ﵽ3����ִ�з���
		if (mod_f[2] != 0)
		{
			if (mod_f_time_cnt < 2000)
			{
				mod_f_time_cnt += 1e3f * dT_s;
			}
			else
			{
				u8 tmp;
				if (mod_f[2] >= 3)
				{
					//ִ�з���
					tmp = OneKey_Return_Home();
				}
				else
				{
					//null
				}
				//reset
				if (tmp)
				{
					mod_f_time_cnt = 0;
					mod_f[2] = 0;
				}
			}
		}

		//ҡ������ת������������
		//ҡ������ת����+-500��������
		float tmp_ch_dz[4];
		tmp_ch_dz[ch_1_rol] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_1_rol] - 1500), 0, 40);
		tmp_ch_dz[ch_2_pit] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_2_pit] - 1500), 0, 40);
		tmp_ch_dz[ch_3_thr] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1500), 0, 80);
		tmp_ch_dz[ch_4_yaw] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_4_yaw] - 1500), 0, 80);
		//׼������ʱ��ROL,PIT,YAW��Ч
		if (sti_fun.pre_locking)
		{
			tmp_ch_dz[ch_1_rol] = 0;
			tmp_ch_dz[ch_2_pit] = 0;
			tmp_ch_dz[ch_4_yaw] = 0;
		}
		//ҡ��ת����̬����������
		//ע��0.00217f��0.00238f��Ϊ�˶�Ӧ�Ĳ���������С�Ĳ��֣�ԭ��Ϊ0.002f����500ת������1��
		//ע����������Ҫ����ANO����ϵ���壬һ�������̬�Ǳ�ʾ���򣨰�����λ������ң��ҡ�˷�����ɿ�ʹ�õ�ANO����ϵ��ͬ��
//		if(mod_f[0]<2)
//		{		
			rt_tar.st_data.rol = tmp_ch_dz[ch_1_rol] * 0.00217f * MAX_ANGLE;
			rt_tar.st_data.pit = -tmp_ch_dz[ch_2_pit] * 0.00217f * MAX_ANGLE;		//��Ϊҡ�˸�������Ͷ���ĸ��������෴������ȡ��
			rt_tar.st_data.thr = (rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1000);		//0.1%
			rt_tar.st_data.yaw_dps = -tmp_ch_dz[ch_4_yaw] * 0.00238f * MAX_YAW_DPS;
			//��ݮ�ɿ���ģʽ���� received_data ���� RC ֵ
					if(pi_ctrl_mode == 1)
		{
			rt_tar.st_data.yaw_dps = received_data.com_yaw;
		}

		//��Ϊҡ�˺�����Ͷ���ĺ������෴������ȡ��		
//		}
		//############(ʵʱ����֡�����������ջ����ƣ������︳ֵ����)##############
		//ʵʱXYZ-YAW�����ٶ�(ʵʱ����֡)
//		rt_tar.st_data.yaw_dps = 0;  //����ת�����ٶȣ���ÿ�룬��ʱ��Ϊ��
//		rt_tar.st_data.vel_x = 0;    //ͷ���ٶȣ�����ÿ��
//		rt_tar.st_data.vel_y = 0;    //�����ٶȣ�����ÿ��
//		rt_tar.st_data.vel_z = 0;	 //�����ٶȣ�����ÿ��
		//########################################################################
		//=====
		dt.fun[0x41].WTS = 1; //��Ҫ����rt_tar���ݡ�
		//ʧ�ر�����Ǹ�λ
		fail_safe_change_mod = 0;
		fail_safe_return_home = 0;
	}
	else //��ң���ź�
	{
		//�����󣬶�ʧ�ź�ʧ����Ҫ�������������ɷ���ʱ���Զ��������䣩
		if (fc_sta.unlock_sta != 0)
		{
			//��Ӧ�ĸı�ģʽ
			if (fail_safe_change_mod == 0)
			{
				//ʧ�ر������л����̿�ģʽ
				fail_safe_change_mod = LX_Change_Mode(3);
			}
			else if (fail_safe_return_home == 0)
			{
				//�л����̿�ģʽ�󣬷��ͷ���ָ��
				fail_safe_return_home = OneKey_Return_Home();
			}
		}

		//ʧ�ر���ʱĿ��ֵ
		rt_tar.st_data.rol = 0;
		rt_tar.st_data.pit = 0;
		rt_tar.st_data.thr = 350; //����ģʽ0������ģʽ0ʱʧ�أ����Ź�����ܣ���һ���Ե�����λ������
		//������ʵʱXYZ-YAW�����ٶ�����		
		rt_tar.st_data.yaw_dps = 0;
		rt_tar.st_data.vel_x =
			rt_tar.st_data.vel_y =
				rt_tar.st_data.vel_z = 0;
	}
}

//��������
static inline void ESC_Output(u8 unlocked)
{
	static u8 esc_calibrated;
	static s16 pwm[8];
	//
	pwm[0] = pwm_to_esc.pwm_m1 * 0.1f;
	pwm[1] = pwm_to_esc.pwm_m2 * 0.1f;
	pwm[2] = pwm_to_esc.pwm_m3 * 0.1f;
	pwm[3] = pwm_to_esc.pwm_m4 * 0.1f;
	pwm[4] = pwm_to_esc.pwm_m5 * 0.1f;
	pwm[5] = pwm_to_esc.pwm_m6 * 0.1f;
	pwm[6] = pwm_to_esc.pwm_m7 * 0.1f;
	pwm[7] = pwm_to_esc.pwm_m8 * 0.1f;
	//

	if (esc_calibrated == 0)
	{
//ע�⣬����ESCУ׼���ܣ������ܷ�������Ԥ�ϵ��𻵻��������˺�������Ը���
//һ����ҪУ׼ʱ���������������������ⷢ�����⡣
//У׼ESC�ɹ��󣬼ǵùرմ˹��ܣ�����������⡣
#if (ESC_CALI == 1)
		//
		for (u8 i = 0; i < 8; i++)
		{
			pwm[i] = 1000; //У׼ʱ�����������ţ�ע����Σ�ա�
		}
		//
		//��ң���ź� �� ������������λʱ�����0�����ź�
		if (rc_in.fail_safe == 0 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1150)
		{
			//
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
			//���У׼��ɡ�
			esc_calibrated = 1;
		}
#else
		//û�п�У׼���ܣ�ֱ�ӱ��У׼��ɡ�
		esc_calibrated = 1;
#endif
	}
	else
	{
		//������������������0����
		if (unlocked)
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = LIMIT(pwm[i], 0, 1000);
			}
		}
		else
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
		}
	}
	//���ײ�PWM��������ź�
	DrvMotorPWMSet(pwm);
}

//����ADC�����ص�ѹ
static void Bat_Voltage_Data_Handle()
{
	fc_bat.st_data.voltage_100 = Drv_AdcGetBatVot() * 100; //��λ��10mv
}

//��ʱ1ms����
void ANO_LX_Task()
{
	static u16 tmp_cnt[2];
	//��10ms
	tmp_cnt[0]++;
	tmp_cnt[0] %= 10;
	if (tmp_cnt[0] == 0)
	{
		//ң������
		DrvRcInputTask(0.01f);
		//ң�����ݴ���
		RC_Data_Task(0.01f);
		//�ɿ�״̬����
		LX_FC_State_Task(0.01f); //
		//��������״̬���
		AnoOF_Check_State(0.01f);
		//==
		//��100ms
		tmp_cnt[1]++;
		tmp_cnt[1] %= 10;
		if (tmp_cnt[1] == 0)
		{
			//��ȡ��ص�ѹ��Ϣ
			Bat_Voltage_Data_Handle();
		}
	}
	//�������ڽ��յ�������
	DrvUartDataCheck();
	//GPS���ݴ���
	GPS_Data_Prepare_Task(1);
	//�ⲿ���������ݴ���
	LX_FC_EXT_Sensor_Task(0.001f);
	//ͨ�Ž���
	ANO_LX_Data_Exchange_Task(0.001f);
	//������
	ESC_Output(1); //unlocked
	//�ƹ�����
	LED_1ms_DRV();
}
