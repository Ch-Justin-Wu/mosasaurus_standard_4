#include "tim3_cnt_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "tim.h"
#include "bsp_imu.h"
#include "rc_task.h"
#include "calibrate_task.h"
#include "bsp_uart.h"
#include "sent_task.h"
#include "upper_computer.h"
#include "bsp_buzzer.h"
#include "referee.h"
#include "USB_Commucation.h"

uint8_t buzzer_flag;
extern VISION_t vision_mode;
extern float vision_yaw, vision_pitch; // ����λ��
int time_count = 0, count_2 = 0, time_count_2 = 0;
int IMU_cnt = 0, start_flag = 0;
//ͼ����·
uint32_t referee_uart_rx_last_cnt;
extern volatile uint32_t referee_uart_rx_cnt;
uint8_t referee_priority_flag = 0;

extern uint32_t rc_update_cnt;
extern uint32_t rc_last_update_cnt;
extern uint32_t rc_offline_cnt;
extern uint8_t rc_offline_flag;

extern shoot_status_e shoot_status;

uint8_t rc_offline_check(void);

// 0.1ms
int psc = 49;
void prevent_jam(void);
void shake_three(void);
void long_pre_shoot(void);
// �������
void TIM3_CNT_TASK()
{
	if (htim->Instance == TIM3)
	{
		time_count++;
		upper_computer_reboot();
		if (buzzer_flag == 0) // �ϵ������
		{
			buzzer_on(psc, 450);
		}
		if (buzzer_flag == 1)
			buzzer_off();
		if (psc >= 89)
			psc = 0;

		if (time_count % 13 == 0 && start_flag == 1)
		{
			Gimbal_Task();
			shoot_task();
			USB_TX();
			// canTX_UPPER_COMPUTER(); // ����λ����������
			receive_upper_data();
			//	 DMA_Send();  �ɵ���λ��ͨѶ
		}
		if (time_count % 7 == 0)//3.5ms
		{
			remote_chassis();
			// ��ȡ����ϵͳ
			referee_unpack_fifo_data();
			control_mode_judge();
			
			if (control_mode == KEY_OFF)
			{
				remote_control_data();
			}
			else if (control_mode == KEY_ON)
			{
				key_control_data();
			}

			judge_key();
			//    long_pre_shoot();
		}
		if (time_count >= 1000)
		{
			psc += 10;
			// canTX_UPPER_COMPUTER_2();
			time_count = 0;
		}
	}
	if (htim->Instance == TIM5)
	{
		static uint16_t rc_tim_cnt = 0;
		// ң�������߼��
		rc_tim_cnt++;
		if (rc_tim_cnt >= 35)
		{
			rc_tim_cnt = 0;
			rc_offline_flag = rc_offline_check();
		}

		count_2++;
		if (IMU_cnt > 5)
		{
			start_flag = 1;
			buzzer_flag = 1;
		}
		INS_task();
		if (count_2 >= 1000)
		{
			count_2 = 0;
			if (start_flag == 0)
				IMU_cnt++;
		}
	}
}

// ң�������߼��
// ң���������ݷ�����ʱ�����rc_update_cnt
// ��rc_update_cnt��rc_last_update_cnt���ʱ��˵��ң������������
uint8_t rc_offline_check(void)
{
	//ͼ����·
	if(referee_uart_rx_cnt!=referee_uart_rx_last_cnt)
	{
		referee_priority_flag = 1;
		
	}
	else
	{
		referee_priority_flag = 0;
	}

	if (rc_update_cnt == rc_last_update_cnt)
	{
		rc_offline_cnt++;
	}
	else
	{
		rc_offline_cnt = 0;
	}
	if (rc_offline_cnt >= 10)
	{
		control_mode = KEY_OFF;

		rc_ctrl.rc.ch[0] = 0;
		rc_ctrl.rc.ch[1] = 0;
		rc_ctrl.rc.ch[2] = 0;
		rc_ctrl.rc.ch[3] = 0;
		rc_ctrl.rc.ch[4] = 0;
		rc_ctrl.rc.s[0] = 1;
		rc_ctrl.rc.s[1] = 1;
		rc_ctrl.mouse.x = 0;
		rc_ctrl.mouse.y = 0;
		rc_ctrl.mouse.z = 0;
		rc_ctrl.mouse.press_l = 0;
		rc_ctrl.mouse.press_r = 0;
		rc_ctrl.key.v = 0;

		shoot_status = SHOOT_OFF;

		rc_update_cnt = 0;
		rc_last_update_cnt = 0;
		rc_offline_cnt = 0;
		referee_uart_rx_last_cnt = 0;
		referee_uart_rx_cnt = 0;
		return 1;
	}
	referee_uart_rx_last_cnt = referee_uart_rx_cnt;
	rc_last_update_cnt = rc_update_cnt;
	return 0;
}

int shake_flag = 0, cnt_, One_shoot_trigger_time_count = 0, Five_shoot_trigger_time_count = 0;
uint8_t shoot_jam_flag = 0;

void prevent_jam(void)
{
	if (One_Shoot_flag == 1) // ����ģʽ
	{
		if ((abs(rc_shoot.trigger.target_angle - rc_shoot.trigger.total_angle) > (36859 / 2))) // δ�ﵽĿ��Ƕ�
		{
			One_shoot_trigger_time_count++;
			if (One_shoot_trigger_time_count >= 50)
			{
				rc_shoot.trigger.target_angle = rc_shoot.trigger.total_angle;
				One_shoot_trigger_time_count = 0;
			}
		}
		else
			One_shoot_trigger_time_count = 0;
	}
	else if (Ten_Shoot_flag == 1) // ������
	{
		if ((abs(rc_shoot.trigger.target_angle - rc_shoot.trigger.total_angle) > (36859 / 2))) // δ�ﵽĿ��Ƕ�
		{
			Five_shoot_trigger_time_count++;
			if (Five_shoot_trigger_time_count >= 250)
			{
				rc_shoot.trigger.target_angle = rc_shoot.trigger.total_angle;
				Five_shoot_trigger_time_count = 0;
			}
		}
		else
			Five_shoot_trigger_time_count = 0;
	}
}

void shake_three(void) // ��������
{
	shake_flag = 1;
	if (shake_flag == 1)
	{
		cnt_++;
		if (cnt_ >= 1000)
		{
			rc_shoot.trigger.target_angle = rc_shoot.trigger.total_angle - 5 / 360 * 36859;
			shake_flag = 2;
			cnt_ = 0;
		}
	}
	if (shake_flag == 2)
	{
		cnt_++;
		if (cnt_ >= 1000)
		{
			rc_shoot.trigger.target_angle = rc_shoot.trigger.total_angle + 5 / 360 * 36859;
			shake_flag = 3;
		}
	}
	if (shake_flag == 3)
	{
		cnt_++;
		if (cnt_ >= 1000)
		{
			rc_shoot.trigger.target_angle = rc_shoot.trigger.total_angle + 5 / 360 * 36859;
			shake_flag = 4;
		}
	}
}

uint16_t shoot_count = 0;
void long_pre_shoot(void)
{
	if (MOUSE_pre_right == 1)
		shoot_count++;
	else
		shoot_count = 0;
	if (shoot_count >= 200)
	{
		if (abs(rc_shoot.trigger.target_angle - rc_shoot.trigger.total_angle) < 500) // ��Ϊ��һ���ӵ��Ѿ�����
			rc_shoot.trigger.target_angle -= ((1.0f * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f);
	}
}
