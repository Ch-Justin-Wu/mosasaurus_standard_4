#ifndef __RC_TASK_H
#define __RC_TASK_H
#include "stdint.h"
/*                 遥控器                   */
//死区值
#define DEADLINE 30
//遥控器值范围
#define RC_MIDD 0
#define RC_MAXX 660
#define RC_MINN -660
//云台角度速度范围
#define RC_YAW_SPEED_MAXX 30
#define RC_YAW_SPEED_MINN -30
#define RC_YAW_ANGLE_MAXX 20
#define RC_YAW_ANGLE_MINN -20

#define RC_PITCH_SPEED_MAXX 6
#define RC_PITCH_SPEED_MINN -6
#define RC_PITCH_ANGLE_MAXX 20
#define RC_PITCH_ANGLE_MINN -20

//底盘速度范围  180
#define X_SPEED_MAXX 900
#define X_SPEED_MINN -900

#define Y_SPEED_MAXX 900
#define Y_SPEED_MINN -900

#define Z_SPEED_MAXX 500
#define Z_SPEED_MINN -500


/************************** 键鼠 ******************************/
#define KEY_DEADLINE 2
//战斗模式时的控制范围
#define KEY_PITCH_ANGLE_MAXX_ON 20
#define KEY_PITCH_ANGLE_MINN_ON -20

#define KEY_YAW_ANGLE_MAXX_ON 30
#define KEY_YAW_ANGLE_MINN_ON -30

//逃跑模式时的控制范围
#define KEY_PITCH_ANGLE_MAXX_RUN 35
#define KEY_PITCH_ANGLE_MINN_RUN -35

#define KEY_YAW_ANGLE_MAXX_RUN 50
#define KEY_YAW_ANGLE_MINN_RUN -50

//mouse_x/y值
#define KEY_MIDD 0
#define KEY_MINN        -300
#define KEY_MAX         300
#define LEFT_PITCH_KEY_MAXX   150
#define LEFT_PITCH_KEY_MINN   -150
#define RIGHT_YAW_KEY_MAXX    100
#define RIGHT_YAW_KEY_RANGE   100

#define MOUSE_x               rc_ctrl.mouse.x     
#define MOUSE_y               rc_ctrl.mouse.y
#define MOUSE_z               rc_ctrl.mouse.z
#define MOUSE_pre_left        rc_ctrl.mouse.press_l
#define MOUSE_pre_right       rc_ctrl.mouse.press_r
#define KEY_board             rc_ctrl.key.v

//图传链路
#define RFR_MOUSE_X referee_rc_data_t.mouse_x
#define RFR_MOUSE_Y referee_rc_data_t.mouse_y
#define RFR_MOUSE_Z referee_rc_data_t.mouse_z
#define RFR_MOUSE_pre_left referee_rc_data_t.left_button_down
#define RFR_MOUSE_pre_right referee_rc_data_t.right_button_down
#define RFR_KEY_board referee_rc_data_t.keyboard_value

#define W_key                0x000001
#define S_key                0x000002
#define ws_key               0x000003   
	
#define A_key                0x000004
#define D_key                0x000008
#define ad_key               0x00000C

#define Q_key                0x000040
#define E_key                0x000080
#define qe_key               0x0000C0

#define R_key                0x000100
#define F_key                0x000200
#define rf_key               0x000300
	
#define Z_key                0x000800
#define X_key                0x001000	
	
#define SHIFT_key            0x000010
#define CTRL_key             0x000020
	
#define C_key                0x002000
#define G_key                0x000400
//14bit  
#define V_key                0x004000  
//15bit
#define B_key                0x008000
#define NULL_Key             0x000000

#define   prepare_status  (uint8_t)(1<<3)  //准备模式

#define    KEY_COUNT            6
#define    LONG_KEY_COUNT       140

typedef enum{
    KEY_ON= 0, //键盘开
    KEY_OFF,    //
}KEY_CONTROL;

typedef enum{
    RUN_AWAY= 0, //键盘开
    FIGHT_ON,    //
}FIGHT_CONTROL;

void remote_control_data(void);
void control_mode_judge(void);
void key_control_data(void);
void judge_key(void);
void upper_computer_reboot(void);

extern int calibrate_start_flag;
extern KEY_CONTROL control_mode;
extern FIGHT_CONTROL fight_mode;
extern uint8_t add_flag;
extern float target_angle;

#endif
