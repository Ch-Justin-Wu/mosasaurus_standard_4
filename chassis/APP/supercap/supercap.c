#include "supercap.h"
#include "referee.h"
#include "usart.h"

/***********************************************************
 *@Brief	供外部调用的变量
 ***********************************************************/
int Supercap_Connection_Status = Supercap_Disconnected;
int Supercap_UART_RX_KeepAlive_Flag = 0;
int Capacitor_State = 0;				 // 电容剩余能量状态。0为低，1为高
float supercap_volt = 0;				 // 超级电容电压
int Power_Mode = 0, Last_Power_Mode = 0; // 用于指示底盘加速模式
float supercap_per = 0;					 // 超级电容电量百分比

supercap_handle supercap_object;

/***********************************************************
 *@Brief	仅供supercap.c文件中使用的变量
 ***********************************************************/
int Global_Time = 0;

/***********************************************************
 *@Brief	调整supercap.c函数声明顺序
 ***********************************************************/
void Supercap_Trans_RefereeData(void);
void Supercap_Keep_Alive(void);
void PowerMode_Judgement(void);

/***********************************************************
 * @breif         超级电容，根据不同的功率上限给电容上单片机发送数据
 * @param[in]     none
 * @param[out]    代表不同功率的字符
 * @retval        none
 ***********************************************************/

extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
extern uint8_t supercap_reboot_flag;
void supercap(int S_Cnt, int MS_Cnt)
{
	Global_Time = S_Cnt * 1000 + MS_Cnt; // 计算全局时间
	if (Global_Time % 200 == 0)
		Supercap_Keep_Alive();
	if (Global_Time % 100 == 0)
	{
		Supercap_Trans_RefereeData();
		if (supercap_reboot_flag == 1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 断开超电继电器
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		//		PowerMode_Judgement();
	}
}
/***********************************************************
 *@fuction	:Supercap_Keep_Alive
 *@brief		:确认超电连接状态
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
extern void UART_TX_Supercap_Connection_Check(int Global_Time);
int Keep_Alive_Time_Cnt;
void Supercap_Keep_Alive(void)
{
	Keep_Alive_Time_Cnt++;							// 计时
	UART_TX_Supercap_Connection_Check(Global_Time); // 发送查询请求
	// 在bsp_uart.c的串口中断中，接收到查询反馈后，Keep_Alive_Time_Cnt将置零
	if (Keep_Alive_Time_Cnt > 10) // 1.2s没有收到超电反馈后
		Supercap_Connection_Status = Supercap_Disconnected;
	else
		Supercap_Connection_Status = Supercap_Connected;
}

/***********************************************************
 *@fuction	:Supercap_Trans_RefereeData
 *@brief		:向超电发送裁判系统数据
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
void Supercap_Trans_RefereeData(void)
{
	// 向超电发送功率信息
	uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
	fp32 UART_TX_Power = 0;
	get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max); // 获取裁判系统数据
	UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);								   // 向超电发送功率限制数据与底盘当前功耗
}

/***********************************************************
 *@fuction	:PowerMode_Judgement
 *@brief		:根据超电状态控制底盘功率模式
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
void PowerMode_Judgement(void)
{
	// 底盘功率模式判断，分超电是否连接两种情况
	if (Supercap_Connection_Status == Supercap_Connected)
	{
		if (supercap_volt > 200)
			Power_Mode = High_Voltage_Mode;
		if (supercap_volt <= 150)
			Power_Mode = Low_Voltage_Mode;
		if (Last_Power_Mode == Low_Voltage_Mode && supercap_volt < 180)
			Power_Mode = Low_Voltage_Mode;
	}
	else
	{
		Power_Mode = Medium_Voltage_Mode;
	}
	// 记录上次功率模式
	Last_Power_Mode = Power_Mode;
}

void set_power_mode(supercap_handle *ptr, enum_power_mode mode)
{
	ptr->power_mode = mode;
	if (mode == SUPER_CAP)
	{
		HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_RESET);
	}
	if (mode == BATTRY)
	{
		HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_SET);
		ptr->percent = 0.0f;
		ptr->voltage = 0.0f;
	}
}

void supercap_can_tx(void)
{
	supercap_object.actual_power = power_heat_data_t.chassis_power;
	supercap_object.max_power = (float)robot_state.chassis_power_limit;

	supercap_object.power_buffer = power_heat_data_t.chassis_power_buffer;
	supercap_object.remain_power_buffer = (float)power_heat_data_t.chassis_power_buffer;

	supercap_object.max_power = supercap_object.max_power + supercap_object.remain_power_buffer-15.0f;
	if (supercap_object.remain_power_buffer-10.f < 0)
	{
		supercap_object.max_power = (float)robot_state.chassis_power_limit;
	}
		if (supercap_object.max_power == 0)
		{
			supercap_object.max_power = 45;
		}
	memcpy(supercap_object.tx_buff, &supercap_object.max_power, 4);

	memcpy(supercap_object.tx_buff + 4, &supercap_object.actual_power, 4);

	CAN_Send_Data(&hcan2, SUPER_CAP_TX_ID, supercap_object.tx_buff, 8);
}
