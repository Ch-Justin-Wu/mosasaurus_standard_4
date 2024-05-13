#include "chassis_task.h"
#include "sent_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "bsp_uart.h"
#include "vision_task.h"
#include "USB_Commucation.h"

uint8_t chassis_speed_x, chassis_speed_y;
enum
{
	__servo_open,
	__servo_close
};
uint8_t servo_flag = __servo_close;
uint16_t move_count = 0;
short int f_move_flag = 0;
short int l_move_flag = 0;

int yuuuu = 0;
extern uint16_t set_compare;
extern const uint16_t SERVO_OPEN;
extern const uint16_t SERVO_CLOSE;
void remote_chassis(void)
{
	if (gimbal_set_mode == GIMBAL_INIT) // 初始
	{
		canTX_chassis_first(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, gimbal_y.given_current);
		canTX_chassis_second(1, vision_mode);
	}

	if (gimbal_set_mode == GIMBAL_CALI || gimbal_set_mode == GIMBAL_RELATIVE_ANGLE) // 正常
	{
		canTX_chassis_first(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, gimbal_y.given_current);
		canTX_chassis_second(0, vision_mode);
	}
	if (gimbal_set_mode == GIMBAL_ABSOLUTE_ANGLE) // 随动
	{

		canTX_chassis_first(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, gimbal_y.given_current);
		if (pack.point_num == 5 && (vision_mode == VISION_ON || vision_mode == ASSIST_VISION_ON))
		{
			// 底盘不动
			canTX_chassis_second(3, vision_mode);
		}
		else
		{
			canTX_chassis_second(1, vision_mode);
		}
	}
	if (gimbal_set_mode == GIMBAL_TOP_ANGLE) // 小陀螺
	{
		canTX_chassis_first(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, gimbal_y.given_current);
		if (pack.point_num == 5&&(vision_mode==VISION_ON||vision_mode==ASSIST_VISION_ON))
		{
			// 底盘不动
			canTX_chassis_second(3, vision_mode);
		}
		else
		{
			canTX_chassis_second(2, vision_mode);
		}
	}
	if (gimbal_set_mode == GIMBAL_ZERO_FORCE) // 无力
	{
		canTX_chassis_first(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, 0);
		canTX_chassis_second(3, vision_mode);
		canTX_gimbal_p_2(0);
		canTX_fric(0, 0, 0);

		// set_compare = SERVO_OPEN;
	}
}
