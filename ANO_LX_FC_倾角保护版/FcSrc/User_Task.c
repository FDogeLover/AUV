#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "math.h"
#include "angle_protect.h"
#define speed_x 20
#define speed_y 20
#define speed_z 20
#define speed_yaw 20
u16 pid_speed=0;
u8 mission_stage=0;// 用于指示当前任务阶段
u8 mission_done_flag=0;// 当前任务完成后通知上位机状态

void UserTask_OneKeyCmd(void)// 一键功能
{
  static u8 one_key_land_f = 1, one_key_mission_f = 0;
//    static u8 mission_step,eme_stop=1,pi_start_f=0,now_task_mode=0,land_triggered_f = 0;
	static u8 mission_step,eme_stop=1,pi_start_f=0,now_task_mode=0,
					land_triggered_f=0,landing_f=0;
					static u16 landing_cnt=0;
					u8 land_cmd_sent_f=0;           // 降落路径上通用降落指令的标志
  //////////////////////////////////////////////////////////////////////
	pi_ctrl_mode = 1;
//急停：CH_8通道在 1700<CH_8<2200
	if ((rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 2200 )|| (Attitude_Check() == 1)) 
	{
		if (eme_stop == 0) 
		{
			eme_stop = 1;
				//执行急停
			FC_Lock();
			pwm_to_esc.pwm_m1 = 0;
			pwm_to_esc.pwm_m2 = 0;
			pwm_to_esc.pwm_m3 = 0; 
			pwm_to_esc.pwm_m4 = 0;
		}
	} 
	else 
	{
		eme_stop = 0;
	}
	//////////////////////////////////////////////////////////////////////
//一键降落  CH_8通道在 1200<CH_8<1700
  if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200 && rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700)
  {
       if (land_triggered_f == 0 && landing_f == 0)  // 降落完成后禁止再次触发
			 {
						OneKey_Land();
						// 设置下降速度，直接执行降落
						rt_tar.st_data.vel_x = 0;
						rt_tar.st_data.vel_y = 0;
						rt_tar.st_data.vel_z = -50;
						rt_tar.st_data.yaw_dps = 0;
//						dt.fun[0x41].WTS = 1;
						land_triggered_f = 1;
						land_cmd_sent_f = 1;   // 标记：已发送降落指令
						landing_f = 0;
						landing_cnt = 0;
				}
  }
  else  // 离开降落区时清空标志，下次进入时重新触发
  {
      land_triggered_f = 0;
      land_cmd_sent_f = 0;
  }
	 //////////////////////////////////////////////////////////////////////
  // 通用降落触发检测（覆盖 CH_8 降落 + mission default 降落）
  if (land_cmd_sent_f == 1)
  {
	      if (landing_f == 0)  // 尚未完成降落触发
      {
	          if (ano_of.work_sta && ano_of.of_alt_cm < 10)  // 光流有效且高度 < 10cm
          {
              landing_cnt++;
	              if (landing_cnt >= 50)  // 连续约 1 秒确认落地
              {
                  FC_Lock();
                  pwm_to_esc.pwm_m1 = 0;
                  pwm_to_esc.pwm_m2 = 0;
                  pwm_to_esc.pwm_m3 = 0;
                  pwm_to_esc.pwm_m4 = 0;

	                  landing_f = 1;        // 降落完成
              }
          }
          else
          {
	              landing_cnt = 0;          // 未落地则清空计数器
          }
      }
	      // landing_f == 1 -> 降落完成，保持状态不再触发
  }
  else  // land_cmd_sent_f == 0 -> 无降落指令
  {
      landing_f = 0;
      landing_cnt = 0;
  }	//////////////////////////////////////////////////////////////////////
		//任务启动：CH_7通道 1700<CH_7<2200 或 上位机发送任务指令
		if((rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3]<2200) || (received_data.task_sta==1))//上位机远程任务触发判断
		{
				//还没执行
			if(one_key_mission_f ==0)
			{
					//标记已执行
				one_key_mission_f = 1;
					//开始任务
				mission_step = 0;
			}
		}
		else
		{
				//清空标记，以便再次执行
			if(one_key_mission_f==1)
			{
					OneKey_Land();
					land_cmd_sent_f = 1;   // 标记降落
			}		
			one_key_mission_f = 0;		
		}
	///////////////////////////////////////////////////////////////////////
		//任务列表
		if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;  
			static s16 integ_x,integ_y;
			static s16 integ_x_base,integ_y_base;
			static s32 pos_x_base,pos_y_base;
			static u16 icount=0;
			mission_stage=mission_step;
			//
			switch(mission_step)
			{
				case 0:
				{
					//reset
					LX_Change_Mode(2);
					pi_ctrl_mode = 1;
					time_dly_cnt_ms = 0;
					mission_step +=1;
				}
				break;
					case 1://解锁
					{
						PID_init();
						if(FC_Unlock()) mission_step = 5;
					}
				break;

					case 5://视觉控制阶段
				{
					if(received_data.next_task_sign==0)
					{
						// 高度：用视觉模块的高度 + 目标高度
						tar_setdata(received_data.com_x,received_data.com_y,height_set(ano_of.of_alt_cm,received_data.com_z),received_data.com_yaw);
					}
					else if(received_data.next_task_sign==1)
					{
						time_dly_cnt_ms = 0;
						mission_step = 6;
					}
					else
					{
						mission_step = 101;
					}
						
				}
				break;
					case 6://定点降落阶段
				{
					if(received_data.next_task_sign==1)
					{
						// 高度：直接用定位模块的高度
						tar_setdata(received_data.com_x,received_data.com_y,height_set(received_data.com_z,received_data.com_z),received_data.com_yaw);
					}
					else if(received_data.next_task_sign==0)
					{
						time_dly_cnt_ms = 0;
						mission_step =5;
					}
					else
					{
						mission_step = 101;
					}
				}
				break;
				default:
				{
					OneKey_Land();
					land_cmd_sent_f = 1; 
				}
				break;
			}
			
		}
		else
		{
			PID_init();
			mission_step = 0;
			mission_stage=0;
			if(land_cmd_sent_f==0){
					tar_setdata(0,0,0,0);
			}
			else{
				rt_tar.st_data.vel_x=0;rt_tar.st_data.vel_y=0;rt_tar.st_data.yaw_dps=0;dt.fun[0x41].WTS=1;
			}
			
		}
		
}





