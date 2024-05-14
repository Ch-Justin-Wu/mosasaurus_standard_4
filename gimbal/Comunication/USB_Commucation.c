#include "USB_Commucation.h"

#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "delay.h"
#include "usbd_desc.h"
#include "stdbool.h"
#include "vision_task.h"
#include "gimbal_task.h"

#include "Self_aim.h"
extern uint8_t game_status;
Pack_tx_t pack;
uint8_t start_receive_flag = 0;
Pack_rx_t pack_rx;
uint8_t receive_data[36];
uint8_t Buffer[34], status;
float distance;
/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL)
    return 0xFFFF;
  while (dwLength--)
  {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

bool Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
      (w_expected & 0xff) == pchMessage[dwLength - 2] &&
      ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

// 接收处理函数
void cdc_vcp_data_rx(uint8_t *buf, uint32_t Len)
{
  for (uint16_t j = 0; j < Len; j++)
  {
    if (buf[j] == 0xA5) // 如果这一位等于帧头
    {
      start_receive_flag = 1;
    }
    if (start_receive_flag == 1) // 数据处理
    {
      memcpy(receive_data, buf + j, Len);
      memcpy(&pack_rx, receive_data, Len);
      if (pack_rx.target_y != 0 && pack_rx.target_x != 0)
      { // 弹道解算
        Auto_aim(pack_rx.target_x, pack_rx.target_y, pack_rx.target_z + 0.00f, &gimbal_y.auto_aim_angle, &gimbal_p.auto_aim_angle, &distance);

        // 自瞄标志位
        vision_sent.Control_priority = pack_rx.UP_flag;
        pack_rx.UP_flag = 0;
        
        vision_sent.yaw.target_angle = gimbal_y.auto_aim_angle;
        vision_sent.pitch.target_angle = gimbal_p.auto_aim_angle;
      }

      // 改为下位机解算后注释??
      start_receive_flag = 0;
    }
    return;
  }
}

void USB_TX(void)
{
  pack.hander = 0x5A;
  pack.game_status = game_status;
  pack.point_num;
  pack.is_large_buff ;//
  pack.detect_color = 1;
  pack.target_id = 0x01;
  pack.pitch = gimbal_p.IMU_actual_angle;
  pack.roll = gimbal_r.IMU_actual_angle;
  // pack.pitch = gimbal_r.IMU_actual_angle;
  // pack.roll = -gimbal_p.IMU_actual_angle;
  pack.yaw = gimbal_y.IMU_actual_angle;
  pack.crc16 = 0xffff;
  memcpy(Buffer, &pack, sizeof(pack));
  Append_CRC16_Check_Sum(Buffer, sizeof(pack));
  CDC_Transmit_FS(Buffer, sizeof(pack));
}
