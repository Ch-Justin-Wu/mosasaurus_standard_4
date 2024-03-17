#include "rc_task.h"
#include "remote_control.h"
#include "bsp_math.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "bsp_uart.h"
#include "shoot_task.h"
#include "gimbal_task_behaviour.h"
#include "upper_computer.h"
#include "tim.h"
#include "referee.h"

extern shoot_status_e shoot_status;
extern ext_referee_rc_data_t referee_rc_data_t;

KEY_CONTROL control_mode = KEY_OFF;	 // 控制模式
FIGHT_CONTROL fight_mode = FIGHT_ON; // 战斗模式

extern int16_t SHOOT_LEFT_FRIC_SPEED_MAX;
extern int16_t SHOOT_LEFT_FRIC_SPEED_MIN;
extern int16_t SHOOT_RIGHT_FRIC_SPEED_MAX;
extern int16_t SHOOT_RIGHT_FRIC_SPEED_MIN;
extern UPPER_COMPUTER_VISION_t shoot_vision_mode;

extern enum {
	OPEN,
	CLOSE = 1,
} bullet_state;

extern uint8_t reboot_flag;
extern uint8_t game_status, speed_limit;
int calibrate_start_flag = 0;
uint16_t last_key;
uint8_t spin_flag_1, spin_flag_2, fly_flag = 0, chassis_power_flag = 1;
uint8_t last_TOP_ANGLE_flag = 0;
uint8_t add_flag = 0;
uint8_t time_count_q = 0, time_count_f = 0, time_count_e = 0, time_count_c = 0, time_count_v = 0, time_count_x = 0, time_count_z = 0, time_count_g = 0;
float target_angle = 0;
static int deadline_judge_v(int16_t a);
// 死区判断
static int deadline_judge_v(int16_t a)
{
	if (control_mode == KEY_OFF)
	{
		if (abs(a - RC_MIDD) <= DEADLINE)
			return 1;
		else
			return 0;
	}
	if (control_mode == KEY_ON)
	{
		if (abs(a - KEY_MIDD) <= KEY_DEADLINE)
			return 1;
		else
			return 0;
	}
	return 1;
}
// 控制模式选择
void control_mode_judge(void)
{
	if (rc_ctrl.rc.ch[0] != 0 || rc_ctrl.rc.ch[1] != 0 || rc_ctrl.rc.ch[2] != 0 || rc_ctrl.rc.ch[3] != 0 || rc_ctrl.rc.ch[4] != 0)
		control_mode = KEY_OFF;
	if (KEY_board || MOUSE_x || MOUSE_y || MOUSE_z || RFR_KEY_board || RFR_MOUSE_X || RFR_MOUSE_Y || RFR_MOUSE_Z)
		control_mode = KEY_ON;
}

//
uint8_t shoot_safety_cnt = 0;
// 遥控器控制模式
void remote_control_data(void)
{

	calibrate_start_flag = 0;
	/*****************************************模式选择************************************/
	if (switch_is_up(rc_ctrl.rc.s[1])) // 左上  底盘运动模式选择
	{
		vision_mode = VISION_OFF;											 // 左上关闭自瞄
		if (switch_is_up(rc_ctrl.rc.s[0]) && gimbal_set_mode != GIMBAL_INIT) // 左上右上随动
		{
			gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;
		}
		if (switch_is_mid(rc_ctrl.rc.s[0]) && gimbal_set_mode != GIMBAL_INIT) // 左上右中小陀螺
		{
			gimbal_set_mode = GIMBAL_TOP_ANGLE;
		}
		if (switch_is_down(rc_ctrl.rc.s[0])) // 左上右下无力
		{
			gimbal_set_mode = GIMBAL_ZERO_FORCE;
			//			calibrate_start_flag=1;

			return;
		}
	}
	if (switch_is_mid(rc_ctrl.rc.s[1])) // 左中 视觉选择
	{
		if (switch_is_up(rc_ctrl.rc.s[0])) // 左中右上大风车
		{
			fricspeed = FRIC_MAX;
			vision_mode = VISION_ON;
			shoot_vision_mode = VISION_BIG_WINDWILL;
		}
		if (switch_is_mid(rc_ctrl.rc.s[0])) // 左中右中小风车
		{
			fricspeed = FRIC_MAX;
			vision_mode = VISION_ON;
			shoot_vision_mode = VISION_SMALL_WINDWILL;
		}
		if (switch_is_down(rc_ctrl.rc.s[0])) // 左中右下开启自瞄
		{
			vision_mode = VISION_ON;
			shoot_vision_mode = VISION_NOMAL;
		}
	}
	if (switch_is_down(rc_ctrl.rc.s[1])) // 左下  发子弹
	{

		if (deadline_judge_v(rc_ctrl.rc.ch[4]) == 0)
		{
			shoot_safety_cnt++;
			if (shoot_safety_cnt > 10)
			{
				shoot_safety_cnt = 0;
				if (rc_ctrl.rc.ch[4] > 0)
				{
					shoot_status = SHOOT_ON;
					if (fricspeed == FRIC_MIN)
					{
						rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
						rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
					}
					else
					{
						rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MAX;
						rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
					}
				}
				else
				{
					shoot_status = SHOOT_OFF;
				}
			}
		}

		if (switch_is_down(rc_ctrl.rc.s[0])) // 左下右下单发
		{
			One_Shoot_flag = 1;
		}
		if (switch_is_mid(rc_ctrl.rc.s[0])) // 左下右中不发弹
		{
			One_Shoot_flag = 0;
			Ten_Shoot_flag = 0;
		}
		if (switch_is_up(rc_ctrl.rc.s[0])) //  左下右上连发
		{
			Ten_Shoot_flag = 1;
		}
	}
	else
	{
		shoot_status = SHOOT_OFF;
	}
	if (deadline_judge_v(rc_ctrl.rc.ch[0]) == 0)
	{
		rc_sent.y_speed = limits_change(Y_SPEED_MAXX, Y_SPEED_MINN, rc_ctrl.rc.ch[0], RC_MAXX, RC_MINN);
	}
	else
		rc_sent.y_speed = 0;
	if (deadline_judge_v(rc_ctrl.rc.ch[1]) == 0)
	{
		rc_sent.x_speed = limits_change(X_SPEED_MAXX, X_SPEED_MINN, rc_ctrl.rc.ch[1], RC_MAXX, RC_MINN);
	}
	else
		rc_sent.x_speed = 0;
	if (deadline_judge_v(rc_ctrl.rc.ch[4]) == 0)
	{
		if (rc_ctrl.rc.ch[4] > 0)
			rc_sent.r_speed = (100.0 / 660.0) * (float)(rc_ctrl.rc.ch[4]); // rc_ctrl.rc.ch[4]数据很怪
		else if (rc_ctrl.rc.ch[4] < 0)
			rc_sent.r_speed = (100.0 / 32000.0) * (float)(rc_ctrl.rc.ch[4]);

		if (rc_sent.r_speed > 100 || rc_sent.r_speed < -100)
			rc_sent.r_speed = 0;
	}
	else
		rc_sent.r_speed = 0;

	rc_sent.yaw.target_speed = limits_change(RC_YAW_SPEED_MAXX, RC_YAW_SPEED_MINN, rc_ctrl.rc.ch[2], RC_MAXX, RC_MINN);
	rc_sent.yaw.target_angle = limits_change(RC_YAW_ANGLE_MAXX, RC_YAW_ANGLE_MINN, rc_ctrl.rc.ch[2], RC_MAXX, RC_MINN);

	rc_sent.pitch.target_speed = limits_change(RC_PITCH_SPEED_MAXX, RC_PITCH_SPEED_MINN, rc_ctrl.rc.ch[3], RC_MAXX, RC_MINN);
	rc_sent.pitch.target_angle = limits_change(RC_PITCH_ANGLE_MAXX, RC_PITCH_ANGLE_MINN, rc_ctrl.rc.ch[3], RC_MAXX, RC_MINN);

	//	if(deadline_judge(rc_ctrl.rc.ch[2],1)==0)
	//	{
	//		rc_sent.yaw.target_speed=limits_change(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
	//		rc_sent.yaw.target_angle=limits_change(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
	//	}
	//	else
	//	{
	//		rc_sent.yaw.target_speed=0;
	//		rc_sent.yaw.target_angle=0;
	//	}
	//	if(deadline_judge(rc_ctrl.rc.ch[3],2)==0)
	//	{
	//		rc_sent.pitch.target_speed=limits_change(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
	//		rc_sent.pitch.target_angle=limits_change(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
	//	}
	//	else
	//	{
	//		rc_sent.pitch.target_speed=0;
	//		rc_sent.pitch.target_angle=0;
	//	}
}
/*******************************************************键鼠控制模式 **************************************************************/

void key_control_data(void)
{
	One_Shoot_flag = 0;
	Ten_Shoot_flag = 0;

	// 战斗模式判断
	if ((KEY_board & SHIFT_key) || (RFR_KEY_board & SHIFT_key)) // 按下shift键
		fight_mode = RUN_AWAY;
	else
		fight_mode = FIGHT_ON;
	/*控制更为精细*/
	if (fight_mode == FIGHT_ON)
	{
		/*       控制云台      */
		if (gimbal_set_mode == GIMBAL_TOP_ANGLE) // 小陀螺的yaw
		{
			gimbal_y.gimbal_raw_pid.Kp = 12000;
			gimbal_y.gimbal_raw_pid.max_out = 27000;
			//			rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,RIGHT_YAW_KEY_MAXX,LEFT_YAW_KEY_MAXX,LEFT_YAW_KEY_RANGE,RIGHT_YAW_KEY_RANGE);
		}
		//        rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAX,KEY_MINN);
		else
		{
			gimbal_y.gimbal_raw_pid.Kp = 10000;
			gimbal_y.gimbal_raw_pid.max_out = 17000;
		}
		rc_sent.yaw.target_angle = limits_change(KEY_YAW_ANGLE_MAXX_ON, KEY_YAW_ANGLE_MINN_ON, MOUSE_x, KEY_MAX, KEY_MINN); // 映射鼠标的值到pitch和yaw
		rc_sent.pitch.target_angle = -limits_change(KEY_PITCH_ANGLE_MAXX_ON, KEY_PITCH_ANGLE_MINN_ON, MOUSE_y, KEY_MAX, KEY_MINN);
		if (MOUSE_pre_left == 1)
		{
			if (fricspeed == FRIC_MIN)
			{
				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
				rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
			}
			else
			{
				//			  rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
				//		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MAX;
				rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
			}
			One_Shoot_flag = 1;
		}

		if (MOUSE_pre_right == 1)
			Ten_Shoot_flag = 1;

		/*       控制底盘     */
		if (KEY_board & ws_key)
		{
			if ((KEY_board & W_key) && (!(KEY_board & S_key))) // 只按下w
			{
				rc_sent.x_speed = 250;
			}
			if ((KEY_board & S_key) && (!(KEY_board & W_key)))
			{
				rc_sent.x_speed = -250;
			}
			if ((KEY_board & W_key) && (KEY_board & S_key)) // 同时按下 遵循最后一个按下的按键
			{
				if ((last_key & W_key) && (!(last_key & S_key)))
					rc_sent.x_speed = -250;
				if ((last_key & S_key) && (!(last_key & W_key)))
					rc_sent.x_speed = 250;
			}
		}
		else
			rc_sent.x_speed = 0;

		if (KEY_board & ad_key)
		{
			if ((KEY_board & A_key) && (!(KEY_board & D_key))) // 只按下A
			{
				rc_sent.y_speed = -200;
			}
			if ((KEY_board & D_key) && (!(KEY_board & A_key)))
			{
				rc_sent.y_speed = 200;
			}
			if ((KEY_board & A_key) && (KEY_board & D_key)) // 同时按下 遵循最后一个按下的按键
			{
				if ((last_key & A_key) && (!(last_key & D_key)))
					rc_sent.y_speed = 200;
				if ((last_key & D_key) && (!(last_key & A_key)))
					rc_sent.y_speed = -200;
			}
		}
		else
			rc_sent.y_speed = 0;
	}

	if (fight_mode == RUN_AWAY)
	{
		/*       控制云台      */
		//		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,YAW_KEY_MAXX,KEY_MINN);
		////		rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,YAW_KEY_MAXX,KEY_MINN);  //映射鼠标的值到pitch和yaw
		//		//if(deadline_judge(MOUSE_y,2)==0)
		//		rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,PITCH_KEY_MAXX,KEY_MINN);

		//		if(gimbal_set_mode == GIMBAL_TOP_ANGLE)  //小陀螺的yaw
		//		{
		//			gimbal_y.gimbal_raw_pid.Kp=12000;  //提高小陀螺精度
		//			gimbal_y.gimbal_raw_pid.max_out=20000;
		//			rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,RIGHT_YAW_KEY_MAXX,LEFT_YAW_KEY_MAXX,LEFT_YAW_KEY_RANGE,RIGHT_YAW_KEY_RANGE);
		//		}
		//		else
		//		{
		//			gimbal_y.gimbal_raw_pid.Kp=10000;  //正常模式的参数
		//			gimbal_y.gimbal_raw_pid.max_out=17000;
		//		}
		rc_sent.yaw.target_angle = limits_change(KEY_YAW_ANGLE_MAXX_ON, KEY_YAW_ANGLE_MINN_ON, MOUSE_x, KEY_MAX, KEY_MINN); // 映射鼠标的值到pitch和yaw
		rc_sent.pitch.target_angle = -limits_change(KEY_PITCH_ANGLE_MAXX_ON, KEY_PITCH_ANGLE_MINN_ON, MOUSE_y, KEY_MAX, KEY_MINN);

		if (MOUSE_pre_left == 1)
		{
			if (fricspeed == FRIC_MIN)
			{
				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
				rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
			}
			else
			{
				//				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
				//		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MAX;
				rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
			}
			One_Shoot_flag = 1;
		}
		if (MOUSE_pre_right == 1)
			Ten_Shoot_flag = 1;
		//						rc_shoot.left_fric.target_speed = -SHOOT_FRIC_SPEED_MIN;
		//		  	rc_shoot.right_fric.target_speed = SHOOT_FRIC_SPEED_MIN;
		/*       控制底盘     */
		if (KEY_board & ws_key)
		{
			if ((KEY_board & W_key) && (!(KEY_board & S_key))) // 只按下w
			{
				rc_sent.x_speed = 450;
			}
			if ((KEY_board & S_key) && (!(KEY_board & W_key)))
			{
				rc_sent.x_speed = -450;
			}
			if ((KEY_board & W_key) && (KEY_board & S_key)) // 同时按下 遵循最后一个按下的按键
			{
				if ((last_key & W_key) && (!(last_key & S_key)))
					rc_sent.x_speed = -450;
				if ((last_key & S_key) && (!(last_key & W_key)))
					rc_sent.x_speed = 450;
			}
		}
		else
			rc_sent.x_speed = 0;

		if (KEY_board & ad_key)
		{
			if ((KEY_board & A_key) && (!(KEY_board & D_key))) // 只按下A
			{
				rc_sent.y_speed = -400;
			}
			if ((KEY_board & D_key) && (!(KEY_board & A_key)))
			{
				rc_sent.y_speed = 400;
			}
			if ((KEY_board & A_key) && (KEY_board & D_key)) // 同时按下 遵循最后一个按下的按键
			{
				if ((last_key & A_key) && (!(last_key & D_key)))
					rc_sent.y_speed = 400;
				if ((last_key & D_key) && (!(last_key & A_key)))
					rc_sent.y_speed = -400;
			}
		}
		else
			rc_sent.y_speed = 0;
	}
}

uint8_t flag__ = 0;
void judge_q(void)
{
	if (KEY_board & Q_key)
		time_count_q++;
	else
	{
		if (last_key & Q_key)
		{
			if (time_count_q >= KEY_COUNT) // 7*14ms按下时间
			{
				if (gimbal_set_mode == GIMBAL_ABSOLUTE_ANGLE)
					gimbal_set_mode = GIMBAL_TOP_ANGLE; // 小陀螺
				else
					gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;
			}
		}
		time_count_q = 0; // 计数清零
	}
}

void judge_f(void)
{
	if (KEY_board & F_key)
		time_count_f++;
	else
	{
		if (last_key & F_key)
		{
			if (time_count_f >= KEY_COUNT)
			{
				if (vision_mode == VISION_OFF)
				{
					// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  //开自瞄关闭激光
					vision_yaw = gimbal_y.CAN_actual_angle;
					vision_pitch = gimbal_p.IMU_actual_angle;
					vision_mode = VISION_ON;
					shoot_vision_mode = VISION_NOMAL;
				}
				else
				{
					// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  //关自瞄开启激光
					vision_mode = VISION_OFF;
					shoot_vision_mode = VISION_NOMAL;
				}
			}
		}
		time_count_f = 0;
	}
}

extern uint16_t set_compare;
void judge_e(void)
{
	if (KEY_board & E_key)
		time_count_e++;
	else
	{
		if (last_key & E_key)
		{
			if (time_count_e >= KEY_COUNT)
			{
				if (set_compare == 2050)
				{
					bullet_state = CLOSE;
					set_compare = 600;
				}

				else
				{
					bullet_state = OPEN;
					set_compare = 2050;
				}
			}
		}
		time_count_e = 0;
	}
}

void judge_c(void)
{
	if (KEY_board & C_key)
		time_count_c++;
	else
	{
		if (last_key & C_key)
		{
			if (time_count_c >= KEY_COUNT)
			{
				gimbal_y.target_angle += 180;
			}
		}
		time_count_c = 0;
	}
}

void judge_v(void)
{
	if (KEY_board & V_key)
		time_count_v++;
	else
	{
		if (last_key & V_key)
		{
			if (time_count_v >= KEY_COUNT)
			{
				if (shoot_vision_mode != VISION_SMALL_WINDWILL)
				{
					shoot_vision_mode = VISION_SMALL_WINDWILL; // 开启小风车
					fricspeed = FRIC_MAX;
					vision_mode = VISION_ON;
				}
				else
				{
					shoot_vision_mode = VISION_NOMAL;
					vision_mode = VISION_OFF;
				}
			}
		}
		time_count_v = 0;
	}
}

void judge_g(void)
{
	if (KEY_board & G_key)
		time_count_g++;
	else
	{
		if (last_key & G_key)
		{
			if (time_count_g >= KEY_COUNT)
			{
				if (shoot_vision_mode != VISION_BIG_WINDWILL)
				{
					shoot_vision_mode = VISION_BIG_WINDWILL; // 开启大风车
					fricspeed = FRIC_MAX;
					vision_mode = VISION_ON;
				}
				else // 给上位机发送正常自瞄模式  不使用上位机的值
				{
					shoot_vision_mode = VISION_NOMAL;
					vision_mode = VISION_OFF;
				}
			}
		}
		time_count_g = 0;
	}
}

static void judge_x(void)
{
	if (KEY_board & X_key)
	{
		time_count_x++;
		if (vision_mode == VISION_ON)
		{
			if (time_count_x >= LONG_KEY_COUNT) // 长按1s断开上位机继电器
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				reboot_flag = 1;
			}
		}
	}
	else
	{
		time_count_x = 0;
	}
}

uint8_t time_count_ctrl;
void judge_ctrl(void)
{
	if (KEY_board & CTRL_key)
	{ // 摩擦轮开关
		time_count_ctrl++;
		if (time_count_ctrl >= KEY_COUNT)
		{
			if (shoot_status == SHOOT_ON)
			{
				shoot_status = SHOOT_OFF;
			}
			else
			{
				shoot_status = SHOOT_ON;
			}
		}
	}
	else
	{
		time_count_ctrl = 0;
	}
}

uint8_t supercap_reboot_flag;
void judge_z(void)
{
	if (KEY_board & Z_key)
		time_count_z++;
	else
	{
		if (last_key & Z_key)
		{
			if (time_count_z >= LONG_KEY_COUNT) // 长按1s断开超电开关
			{
				if (supercap_reboot_flag == 0)
					supercap_reboot_flag = 1;
				else
					supercap_reboot_flag = 0;
			}
		}
		time_count_z = 0;
	}
}

void upper_computer_reboot(void)
{
	static uint16_t switch_cnt = 0;
	if (reboot_flag)
	{
		switch_cnt++;
		if (switch_cnt >= 1500)
		{
			switch_cnt = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
			reboot_flag = 0;
		}
	}
}

void judge_key(void)
{
	judge_z();
	judge_c();
	judge_x();
	judge_e();
	judge_f();
	judge_q();
	judge_v();
	judge_g();
	judge_ctrl();
	last_key = KEY_board;
}
