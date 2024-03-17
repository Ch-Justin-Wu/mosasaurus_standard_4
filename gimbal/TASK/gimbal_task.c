#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_task_behaviour.h"
#include "sent_task.h"
#include "bsp_imu.h"
#include "can_receive.h"
#include "shoot_task.h"
#include <math.h>
#include "tim.h"

GIMBAL_t gimbal_y,gimbal_p,gimbal_r;
GIMBAL_MODE_t gimbal_set_mode;
//yaw��PID����
//float YawGyroPid[7] 	= {0.2f,			0.0f,			0.0,	2.5f,			0.0f,	 0.0f};  //imu�ǶȻ�  1.8 
float YawGyroPid[7] 	= {0.17f,			0.0f,			0.00f,	10.0f,			0.0f,	 0.15f};  //imu�ǶȻ�  1.8 

float YawEncondePid[7] 	= {0.3f,			0.0f,			0.010f,		50.0f,			0.0f,		0.0f};  //�������ǶȻ�  rhn 1
float YawEncondePidSpeed[7] 	= {500.0f,			50.0f,			0.0f,		10000.0f,			3000.0f,		0.0f};  //�������ٶȻ�  rhn 3
//float YawSpeedPid[6] 			= {10000.0f,			500.0f,			0.0f,		17000.0f,			6000.0f,		0.01f};     //�ٶȻ�  rhn 7000  9000
//float YawSpeedPid[7] 			= {9500.0f,			1800.0f,			0.0f,		17000.0f,			7000.0f,		0.01f, 0.0f };    //�޶�pid
//float YawSpeedPid[6] 			= {9500.0f,			500.0f,			0.0f,		17000.0f,			7000.0f,		0.01f};    //�޶�pid
float YawSpeedPid[7] 			= {10000.0f,			3100.0f,			0.0f,		17000.0f,			6500.0f,		0.01f,  0.7};    //luounyu �»��ַ���
//float YawSpeedPid[7] 			= {5500.0f,			1800.0f,			0.0f,		17000.0f,			8000.0f,		0.01f,  1.3};    //luounyu �»��ַ���


//pitch��PID����
//float PitchGyroPid[6] = {50.0f,			0.0f,			0.0f,		500.0f,			0.0f		,1.5f};

float PitchGyroPid[7] = {0.5f,			0.0f,			0.0f,		50.0f,			0.0f		,0.0f};
float PitchEncondePid[7] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchEncondePidSpeed[7] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchSpeedPid[7] 		= {100.0f,			0.5f,			0.0f,		600.0f,	15.0f,	0.0f};

//PID��ʼ��	
static void YawPitch_PIDinit(void);
//��̨ģʽѡ��
static void GIMBAL_Set_Mode(void);
static void gimbal_rc_mode(void);
//��̨����
static void GIMBAL_Set_Contorl(void);
//PID����
static void GIMBAL_PID(void);
//IMU���ݽ���
static void GIMBAL_CALBACK_GET(void);
//�ٶ�PID
static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_);
int gimbal_imu_cnt=0;
uint16_t set_compare=600;//close

enum
{
	OPEN,
	CLOSE = 1,
} bullet_state = 1;

int8_t us;
void Gimbal_Task(void)
{
	//���
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,set_compare);
	//��ʱ�ȴ�imu�����ȶ�
	//IMU ���ݽ���
	GIMBAL_CALBACK_GET();
	//ģʽѡ��
	GIMBAL_Set_Mode();
	//ģʽ����
	GIMBAL_Set_Contorl();
	//PID����	
//	gimbal_p.target_angle=us;
	GIMBAL_PID();
//	canTX_gimbal_p(gimbal_p.gimbal_raw_pid.out);
	canTX_gimbal_p_2(gimbal_p.gimbal_raw_pid.out);
}
//��̨��ʼ��
void Gimbal_Init(void)
{
	YawPitch_PIDinit(); //PID��ʼ��
	gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
	//yaw�����ݳ�ʼ��
	gimbal_y.IMU_actual_angle=0.0f;
	gimbal_y.IMU_actual_speed=0.0f;
	gimbal_y.CAN_actual_angle=0.0f;
	gimbal_y.CAN_actual_speed=0.0f;
	gimbal_y.target_angle=0.0f;
	gimbal_y.target_speed=0.0f;
	gimbal_y.add_angle=0.0f;
	gimbal_y.given_current=0;
	
	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;//yaw����ģʽ����
	
	//pitch�����ݿ���
	gimbal_p.IMU_actual_angle=0.0f;
	gimbal_p.IMU_actual_speed=0.0f;
	gimbal_p.CAN_actual_angle=0.0f;
	gimbal_p.CAN_actual_speed=0.0f;
	gimbal_p.target_angle=0.0f;
	gimbal_p.target_speed=0.0f;
	gimbal_p.add_angle=0.0f;
	gimbal_p.given_current=0;
	
	gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;//pitch��������


	gimbal_r.IMU_actual_angle=0.0f;
	gimbal_r.IMU_actual_speed=0.0f;
	gimbal_r.CAN_actual_angle=0.0f;
	gimbal_r.CAN_actual_speed=0.0f;
	gimbal_r.target_angle=0.0f;
	gimbal_r.target_speed=0.0f;
	gimbal_r.add_angle=0.0f;
	gimbal_r.given_current=0;

	gimbal_r.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
}

static void YawPitch_PIDinit(void)
{
	//yaw��PID���ݳ�ʼ��
	PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);//�������ٶȿ���
	PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);//�����ǽǶȿ���
	PID_Init(&gimbal_y.gimbal_enconde_pid,YawEncondePid);//�������Ƕȿ���
	PID_Init(&gimbal_y.gimbal_enconde_pid_speed,YawEncondePidSpeed);//�������ٶȿ���

	//pitch��PID���ݳ�ʼ��
	PID_Init(&gimbal_p.gimbal_raw_pid,PitchSpeedPid);//�������ٶȿ���
	PID_Init(&gimbal_p.gimbal_gyro_pid,PitchGyroPid);//�����ǽǶȿ���
	PID_Init(&gimbal_p.gimbal_enconde_pid,PitchEncondePid);//�������Ƕȿ���
	PID_Init(&gimbal_p.gimbal_enconde_pid_speed,PitchEncondePidSpeed);	//�������ٶȿ���
}

static void GIMBAL_Set_Mode(void)
{
	
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_rc_mode();
	//���ģʽѡ��
	if(gimbal_set_mode==GIMBAL_INIT)//��ʼ��ģʽ
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//�����ǽǶȿ���
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_ZERO_FORCE)//����״̬
	{
	    gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;//�������ٶȿ���
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	}
	else if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE)//��̨�涯ģʽ
	{
//		YawGyroPid[5]=0.2;  //С����ģʽ�����Ӵ�
//		PitchGyroPid[5]=0.4;
//		gimbal_y.gimbal_gyro_pid.deadband=0.0f;
//		gimbal_p.gimbal_gyro_pid.deadband=0.0f;
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_TOP_ANGLE)//С����
	{
//		YawGyroPid[5]=0.5;  //С����ģʽ�����Ӵ�
//		PitchGyroPid[5]=0.8;
//		gimbal_y.gimbal_gyro_pid.deadband=0.2f;
//		gimbal_p.gimbal_gyro_pid.deadband=0.7f;
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_RELATIVE_ANGLE)//�������Ƕȿ���
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_CALI)//����ٶȿ���ģʽ
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	}
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
   gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	//gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
}

static void gimbal_rc_mode(void)
{
	if(gimbal_set_mode==GIMBAL_INIT)
	{
		static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
		
		if((fabs(gimbal_y.IMU_actual_angle-INIT_YAW_SET)<GIMBAL_INIT_ANGLE_ERROR)&&(fabs(gimbal_p.IMU_actual_angle-INIT_PITCH_SET)<GIMBAL_INIT_ANGLE_ERROR))
		{
			 //�����ʼ��λ��
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
		}
		//�ϵ��޷���λ���м�λ��
		if(init_time>GIMBAL_INIT_TIME&&init_stop_time < GIMBAL_INIT_STOP_TIME)
		{
			gimbal_set_mode=GIMBAL_ZERO_FORCE;
		}
		if(init_stop_time > GIMBAL_INIT_STOP_TIME)
		{
			gimbal_set_mode= GIMBAL_ABSOLUTE_ANGLE; //��̨�涯
		}
	}
//	else gimbal_set_mode=GIMBAL_ZERO_FORCE; //������
}

static void GIMBAL_Set_Contorl(void)
{
	//ѡ������ֵ��Դ
	Vision_Task();
	
	//ģʽѡ��
   gimbal_control_behaviour();
}
double gimbalp_imuangle;
uint8_t first_update_flag=0;
float imu_can_error_y=0.0f;
static void GIMBAL_CALBACK_GET(void)
{
	//yaw�����ݷ���
    gimbal_y.CAN_actual_angle=yaw_can_rx.angle; //����yaw����ʵ�Ƕ�
	gimbal_y.CAN_total_angle =yaw_can_rx.turns*360.0f+gimbal_y.CAN_actual_angle;		//����yaw��total_angle
	gimbal_y.CAN_actual_speed=yaw_can_rx.speed;
	//yaw��IMU���ݻ�ȡ
    gimbal_y.IMU_actual_angle=INS_angle[0]/(2*3.141590f)*360.0f;
	gimbal_y.IMU_actual_speed=INS_gyro[2];
	
	//pitch���ݷ���
	gimbal_p.CAN_actual_angle=pitch_can_rx.angle/8191.0f*360.0f;
	gimbal_p.CAN_actual_speed=pitch_can_rx.speed;
	//pitchIMU���ݻ�ȡ
    gimbal_p.IMU_actual_angle=-1.00f*INS_angle[1]/(2*3.141590f)*360.0f;
	gimbal_p.IMU_actual_speed=-1.00f*INS_gyro[0];

	//
	gimbal_r.IMU_actual_angle=INS_angle[2]/(2*3.141590f)*360.0f;
	gimbal_r.IMU_actual_speed=INS_gyro[1];
	
	
	if(first_update_flag==0)  //��ȡ�ϵ�ʱyaw�������ǶȺ�imu�ǶȵĹ̶���ֵ  ����ֻ��ȡһ��
	{
		imu_can_error_y=gimbal_y.IMU_actual_angle-(gimbal_y.CAN_actual_angle*360.0f/8192.0f);
	}
	first_update_flag=1;
}

//��̨PId����
static void GIMBAL_PID(void)
{
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_y);
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle>180)
			gimbal_y.target_angle -= 360;
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle<-180)
			gimbal_y.target_angle += 360;	 
		
		gimbal_motor_gyro_pid(&gimbal_y);
		
		if(gimbal_y.IMU_actual_speed > - 0.0f && gimbal_y.IMU_actual_speed < 0.0f)
		{
			gimbal_y.gimbal_raw_pid.out = gimbal_y.gimbal_raw_pid.Pout;
			gimbal_y.gimbal_raw_pid.Iout = 0;
		}
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		if((gimbal_y.target_angle-gimbal_y.CAN_actual_angle)>4096)
			gimbal_y.target_angle -= 8192;
		if((gimbal_y.target_angle-gimbal_y.CAN_actual_angle)<-4096)
			gimbal_y.target_angle += 8192;
		
		gimbal_motor_encode_pid_y(&gimbal_y);		
	}
	
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_gyro_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_encode_pid(&gimbal_p);
	}
}

static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
//	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->CAN_actual_speed);	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}
float spd;
static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_gyro_pid,gimbal_->target_angle,gimbal_->IMU_actual_angle);
	
	gimbal_->target_speed=gimbal_->gimbal_gyro_pid.out;
//	gimbal_->target_speed=spd;
//	if(gimbal_->gimbal_gyro_pid.out>2.0f) gimbal_->gimbal_gyro_pid.out=2.0f; 
//	if(�Ǹ�ֵ!=0)gimbal_->target_speed+=�Ǹ�ֵ;
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

float temp_w;
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_actual_angle);     //pitch�ĵ���������Ϊgimbal_->CAN_actual_angle

//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;
	
//    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
//	
//	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
//	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_total_angle);     //pitch�ĵ���������Ϊgimbal_->CAN_actual_angle
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_actual_angle);
//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;	
	//		gimbal_->target_speed=Ren;	
//    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
//	
//	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
//	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;


}

