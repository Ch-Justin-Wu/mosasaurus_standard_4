#include "shoot_task.h"
#include "remote_control.h"
#include "sent_task.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "bsp_math.h"
#include <math.h>

// #define TEST_SHOOT

uint8_t data[2] = {1, 0};
shoot_task_t rc_shoot;
FRIC_SPEED fricspeed = FRIC_MAX;
shoot_status_e shoot_status = SHOOT_OFF;

extern float Ren;
int shoot_times = 0;
int One_Shoot_flag = 0;
int Ten_Shoot_flag = 0;
int16_t SHOOT_LEFT_FRIC_SPEED_MAX = -7600;
int16_t SHOOT_LEFT_FRIC_SPEED_MIN = -7000;
int16_t SHOOT_RIGHT_FRIC_SPEED_MAX = 7600;
int16_t SHOOT_RIGHT_FRIC_SPEED_MIN = 7000;
extern float shoot_speed;
extern uint8_t speed_limit;
float TriggerSpeed[6] = {3.0f, 0.01f, 0.0f, 8000, 500, 0}; // �������ٶȻ� 8000 4.8

float TriggerAngle[6] = {0.25f, 0.0000f, 0.00f, 5000, 0, 0}; // �����ֽǶȻ� 1700

float FricLeftSpeed[6] = {8.0f, 0.0f, 0.0f, 10000.0f, 0.0f, 0}; // ��Ħ�����ٶȻ�

float FricRightSpeed[6] = {8.0f, 0.0f, 0.0f, 10000.0f, 0.0f, 0}; // ��Ħ�����ٶȻ�

static float abs_f(float a);

void trigger_angle_set(void);
// ���·��ӵ�����
static void shoot_angle_clc(void);
// ������
// static void prevent_jam(void);
// Ħ����PID����
static void fric_pid(void);
// ������PID����
static void trigger_pid(void);
// Ħ�������ٿ���
void fric_speed_control(void);

int trigger_back_flag, trigger_back_flag_cnt = 0;
float speed_1;
float speed_2;
// ��������
void shoot_task(void)
{
	if (shoot_status == SHOOT_ON)
	{

		//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //�ص�����
		if (trigger_cnt_flag == 1)
		{
			trigger_cnt++;
			fric_speed_control();
		}

		fric_pid();

		if ((rc_shoot.left_fric.actual_speed < -4000) && (rc_shoot.right_fric.actual_speed > 4000)) // �ȴ�Ħ����ת��֮����ܲ���
			shoot_angle_clc();

		trigger_pid();

		canTX_fric(rc_shoot.left_fric.set_currunt, rc_shoot.right_fric.set_currunt, rc_shoot.trigger.set_currunt);
	}
	else
	{
		canTX_fric(0, 0, 0);
	}
}

float speed_average;
int yuyuyu_flag = 65;
uint16_t count;
void fric_speed_control(void)
{
	if (speed_limit > 29.4)
	{
		fricspeed = FRIC_MAX;
		rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MAX;
		rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
	}
	else
	{
		fricspeed = FRIC_MIN;
		rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
		rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
	}

	if (speed_change_flag)
	{
		if (fricspeed == FRIC_MAX && shoot_speed >= 29.5)
		{
			SHOOT_LEFT_FRIC_SPEED_MAX += 40;
			SHOOT_RIGHT_FRIC_SPEED_MAX -= 40;
		}
		// ƽ������
		speed_average = ((shoot_speed + speed_average * ((float)count)) / ((float)(count + 1.0f)));
		count++;
		if (count > 10)
		{
			if (fricspeed == FRIC_MIN) // 15
			{

				if ((speed_limit - speed_average < 0.3f) && (speed_limit >= 10.0f))
				{
					SHOOT_LEFT_FRIC_SPEED_MIN += 45;
					SHOOT_RIGHT_FRIC_SPEED_MIN -= 45;
				}
				else if (speed_limit - speed_average > 1.0f && (speed_limit >= 10.0f))
				{
					SHOOT_LEFT_FRIC_SPEED_MIN -= 20;
					SHOOT_RIGHT_FRIC_SPEED_MIN += 20;
				}
			}
			else if (fricspeed == FRIC_MAX) // 30
			{
				if (((speed_limit - speed_average < 0.7f) && (speed_limit >= 10.0f)) || (shoot_speed >= 29.5))
				{
					SHOOT_LEFT_FRIC_SPEED_MAX += 40;
					SHOOT_RIGHT_FRIC_SPEED_MAX -= 40;
				}
				else if (((speed_limit - speed_average > 1.3f) && (speed_limit >= 10.0f)) || (shoot_speed <= 26.0))
				{
					SHOOT_LEFT_FRIC_SPEED_MAX -= 25;
					SHOOT_RIGHT_FRIC_SPEED_MAX += 25;
				}
			}
		}
		speed_change_flag = 0;
	}
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
	// ��ȡ������ϵͳ��������  ���и������ƽ׶γ����Կ�����˵
}

// �����ֺ�Ħ���ֵĳ�ʼ��
void shoot_init(void)
{

	PID_Init(&rc_shoot.trigger.speed_pid, TriggerSpeed);
	PID_Init(&rc_shoot.trigger.angle_pid, TriggerAngle);

	PID_Init(&rc_shoot.left_fric.speed_pid, FricLeftSpeed);
	PID_Init(&rc_shoot.right_fric.speed_pid, FricRightSpeed);

	rc_shoot.trigger.actual_angle = 0.0f;
	rc_shoot.trigger.actual_speed = 0.0f;

	rc_shoot.trigger.last_shoot_flag = 0;
	rc_shoot.trigger.last_back_flag = 0;
	rc_shoot.trigger.target_angle = 0.0f;
	rc_shoot.trigger.target_speed = 0.0f;

	rc_shoot.left_fric.actual_speed = 0.0f;
	rc_shoot.left_fric.target_speed = 0;

	rc_shoot.right_fric.actual_speed = 0.0f;
	rc_shoot.right_fric.target_speed = 0;
}

static float abs_f(float a)
{
	if (a < 0)
		a = -1.0f * a;
	return a;
}

int trigger_round;

static void shoot_angle_clc(void)
{
	if (rc_shoot.trigger.rounds <= -90)
	{
		rc_shoot.trigger.target_angle -= (float)(rc_shoot.trigger.rounds) * 8191.0f;
		if (rc_shoot.trigger.target_angle > 0)
			rc_shoot.trigger.target_angle = 0;
		rc_shoot.trigger.total_angle -= (float)(rc_shoot.trigger.rounds) * 8191.0f;
		rc_shoot.trigger.begin_angle = rc_shoot.trigger.total_angle;
		rc_shoot.trigger.rounds = 0;
	}

	if (rc_shoot.trigger.last_shoot_flag == 0)
	{
		if (One_Shoot_flag == 1 || Ten_Shoot_flag == 1)
		{
			rc_shoot.trigger.last_shoot_flag = 1;
			trigger_angle_set();
			shoot_times += 1;
			if (shoot_times > 6)
				shoot_times = 0;
		}
	}

	if (One_Shoot_flag != 1 && Ten_Shoot_flag != 1)
		rc_shoot.trigger.last_shoot_flag = 0;
}
static void fric_pid(void)
{

	PID_Calc(&rc_shoot.left_fric.speed_pid, rc_shoot.left_fric.target_speed, rc_shoot.left_fric.actual_speed);

	rc_shoot.left_fric.set_currunt = rc_shoot.left_fric.speed_pid.out;

	PID_Calc(&rc_shoot.right_fric.speed_pid, rc_shoot.right_fric.target_speed, rc_shoot.right_fric.actual_speed);

	rc_shoot.right_fric.set_currunt = rc_shoot.right_fric.speed_pid.out;
}

float Ren;
static void trigger_pid(void)
{
	//	rc_shoot.trigger.target_angle=Ren;    //���ڲ����ٶȻ�
	PID_Calc(&rc_shoot.trigger.angle_pid, rc_shoot.trigger.target_angle, rc_shoot.trigger.total_angle);

	rc_shoot.trigger.target_speed = rc_shoot.trigger.angle_pid.out;
	//	rc_shoot.trigger.target_speed=Ren;    //���ڲ��ԽǶȻ�
	PID_Calc(&rc_shoot.trigger.speed_pid, rc_shoot.trigger.target_speed, rc_shoot.trigger.actual_speed);

	rc_shoot.trigger.set_currunt = rc_shoot.trigger.speed_pid.out;
}
int trigger_cnt = 0, trigger_cnt_flag = 0, remain_bullet = 0;
void trigger_angle_set(void)
{
	trigger_cnt_flag = 1;
	remain_bullet = ((heat_limit - heat) / 10) - 2; // �������������µ�ʣ�൯��
#if defined(TEST_SHOOT)
	remain_bullet = 5;
#endif
	if (remain_bullet < 0)
		remain_bullet = 0;
	if (One_Shoot_flag == 1) // ����
	{
		if (fabs(rc_shoot.trigger.target_angle - rc_shoot.trigger.total_angle) < ((1.0f * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f))
		{
			if (remain_bullet >= 1)
				rc_shoot.trigger.target_angle -= ((1.0f * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f);
		}
	}
	else // ������
	{
		if (fabs(rc_shoot.trigger.target_angle - rc_shoot.trigger.total_angle) < ((1.0f * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f))
		{
			if (remain_bullet >= 5)
				rc_shoot.trigger.target_angle -= ((5.0f * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f); // �ĳ���������
			else
				rc_shoot.trigger.target_angle -= ((remain_bullet * 36.0f * (360.0f / 8.0f)) / 360.0f * 8191.0f);
		}
	}
}
// ���������ݷ���
void Trigger_Motor_Callback(trigger_t *motor, uint16_t angle, int16_t speed)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5 * (speed + motor->last_speed);
	motor->last_speed = speed;

	if (motor->record_begin_angle_status < 50)
	{
		motor->begin_angle = angle;
		motor->actual_angle = angle;
		motor->last_angle = angle;
		motor->record_begin_angle_status++;
	}
	if (motor->actual_angle - motor->last_angle > 4096)
		motor->rounds--;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle - motor->begin_angle;
}
