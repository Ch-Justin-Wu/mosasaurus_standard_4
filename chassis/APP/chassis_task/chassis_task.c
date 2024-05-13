#include "chassis_task.h"
#include "supercap.h"
#include "MY_anonymous.h"
#include "referee_UI.h"
#include "buzzer.h"

/**
 * @breif         底盘所有任务函数，实质上是定时器中断，1ms进入一次
 * @param[in]     none
 * @param[out]    none
 * @retval        none
 */
int IMU_cnt = 0, start_flag = 0, calibrate_start_flag;

int MS_Count = 0;
int S_Count = 0;
// uint8_t offline_check_(void);
void my_offline_check(void);
void Time_Service_Task(void);
void CHASSIS_TASK()
{
	static int time_count = 0;

	if (IMU_cnt > 10)
		start_flag = 1;

	time_count++;

	if (time_count % 500 == 0) // 0.5s
	{
		send_game_time();
	}
	// if(time_count%2==0)
	// {
	// 	buzzer_taskScheduler(&buzzer);
	// }
	// INS_task();
	my_offline_check();
	Time_Service_Task();
	if (time_count % 13 == 0 && start_flag == 1)
	{
		// 校准imu
		// if (calibrate_start_flag == 1)
		// 	calibrate_task();
		// IMU_read();
	}
	if(time_count%2==0)
	{
		supercap_can_tx();
		
	}

	if (time_count % 7 == 0)
	{
		// 远程遥控，调试时使用
		//		remote_control();
		//		//底盘运动
		chassis_move();
		canTX_gimbal_y(chassis_center.target_current); // 直接发送云台传过来的电流赋值
	}
	// chassis_control_order.chassis_mode=0;
	if (time_count % 10 == 0)
	{
		Get_Base_Velocities();
		send_gimbal_data_2();
		canTX_chassis_imu();
		// 读取裁判系统
		referee_unpack_fifo_data();
		// 向云台发送数据
		send_gimbal_data();
		//设置
		set_power_mode(&supercap_object, supercap_object.change_flag);
	}

	if (time_count >= 1000) // 清除计数标志    1s
	{
		time_count = 1;
		// 超级电容
		//		supercap();
		if (start_flag == 0)
			IMU_cnt++;
	}
	// 超级电容
	// supercap(S_Count, MS_Count);
}
void Time_Service_Task(void)
{
	MS_Count++;
	if (MS_Count > 999)
	{
		MS_Count = 0;
		S_Count++;
	}
}

void my_offline_check(void)
{ // 上板离线检测
	if (offline_check(&chassis_control_order.offline_check, 100) == OFFLINE)
	{
		chassis_control_order.chassis_mode = CHASSIS_NO_FORCE;
		// buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
	}
	// 电机离线检测
	if (offline_check(&chassis_motor1.offline_check, 100) == OFFLINE || offline_check(&chassis_motor2.offline_check, 100) == OFFLINE || offline_check(&chassis_motor3.offline_check, 100) == OFFLINE || offline_check(&chassis_motor4.offline_check, 100) == OFFLINE || offline_check(&chassis_center.offline_check, 100) == OFFLINE)
	{
		// buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
	}
	// 电容离线检测
	if (offline_check(&supercap_object.offline_check, 200) == OFFLINE)
	{
		//set_power_mode(&supercap_object, BATTRY);
	}
	else
	{
		//set_power_mode(&supercap_object, SUPER_CAP);
	}
}

// uint8_t offline_check_(void)
// {

// 	if (chassis_control_order.can2_online_cnt == chassis_control_order.can2_last_online_cnt)
// 	{
// 		chassis_control_order.can2_offline_cnt++;
// 	}
// 	else
// 	{
// 		chassis_control_order.can2_offline_cnt = 0;
// 	}
// 	if (chassis_control_order.can2_offline_cnt > 100)
// 	{
// 		chassis_control_order.can2_online_cnt = 0;
// 		chassis_control_order.can2_last_online_cnt = 0;
// 		chassis_control_order.chassis_mode = CHASSIS_NO_FORCE;

// 		return 1;
// 	}

// 	chassis_control_order.can2_last_online_cnt = chassis_control_order.can2_online_cnt;
// 	return 0;
// }
