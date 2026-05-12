#ifndef __LX_FC_STATE_H
#define __LX_FC_STATE_H

//==����
#include "SysConfig.h"

//==����/����
typedef struct
{
	u8 pre_locking;
	u8 stick_mit_pos;

} _sticks_fun_st;

typedef struct
{
	u8 CID;
	u8 CMD_0;
	u8 CMD_1;
} _cmd_fun_st;
//�ɿ�״̬
typedef struct
{
	//ģʽ
	u8 fc_mode_cmd;
	u8 fc_mode_sta;

	//��������
	u8 unlock_cmd;
	u8 unlock_sta;

	//ָ���
	_cmd_fun_st cmd_fun;

	//state
	u8 imu_ready;
	u8 take_off;
	u8 in_air;
	u8 landing;

} _fc_state_st;

//==��������
extern volatile _fc_state_st fc_sta;
extern _sticks_fun_st sti_fun;
//==��������
//static

//public
void LX_FC_State_Task(float dT_s);

#endif
