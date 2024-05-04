#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "pid.h"
#include "fsm.h"
//#define TEST_SHOOT

#define SHOOT_NUM 1
// #define SHOOT_FRIC_SPEED_MAX  7700   //4000 8000 30
// #define SHOOT_FRIC_SPEED_MIN  4850 //15 还要再调 4700

typedef enum shoot_type_e
{
	NO_SHOOT = 0,
	ONE_SHOOT,
	FIVE_SHOOT,
	TEN_SHOOT,
} shoot_type_e;

/**
 * @brief 发射机构卡弹状态
 *
 */
typedef enum enum_booster_jamming_type
{
	BOOSTER_NOT_JAMMING = 0,
	BOOSTER_JAMMING,
} enum_booster_jamming_type;

typedef struct
{

	float target_speed;
	float actual_speed;

	float target_angle;
	float actual_angle;

	int last_shoot_flag;
	int last_back_flag;
	float set_currunt;
	int16_t acutal_currunt;
	float last_angle;			   // 上一次机械转子角度
	int rounds;					   // 转过的圈数
	int total_angle;			   // 总共转过的角度
	int last_speed;				   // 上一次真实转速
	int record_begin_angle_status; // 是否记录了电机初始角度 0代表没有记录，1代表记录成功
	int begin_angle;			   // 电机初始角度

	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
} trigger_t;

typedef struct
{

	float target_speed;
	float actual_speed;

	float set_currunt;

	PidTypeDef speed_pid;
} fric_t;

typedef struct
{

	fric_t left_fric;
	fric_t right_fric;

	trigger_t trigger;

	Struct_FSM shoot_fsm;
	enum_booster_jamming_type booster_jamming_type;
} shoot_task_t;

typedef enum
{

	FRIC_MAX = 0,
	FRIC_MIN,
} FRIC_SPEED;

typedef enum shoot_status_e
{
	SHOOT_OFF = 0,
	SHOOT_ON,
} shoot_status_e;

void shoot_init(void);
void shoot_task(void);
void fric_speed_control(void);
void Trigger_Motor_Callback(trigger_t *motor, uint16_t angle, int16_t speed);
void antijam_fsm_task(Struct_FSM *fsm, shoot_task_t *booster);
extern shoot_task_t rc_shoot;
extern int One_Shoot_flag;
extern int Ten_Shoot_flag;
extern FRIC_SPEED fricspeed;
extern void trigger_angle_set(void);
extern int trigger_cnt, trigger_cnt_flag;
extern int trigger_back_flag;
#endif
