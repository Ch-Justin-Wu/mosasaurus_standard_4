#include "bsp_usb.h"

// #include "string.h"
// #include "stdarg.h"
// #include "stdio.h"
// #include "delay.h"
// #include "usbd_desc.h"
// #include "stdbool.h"
// #include "vision_task.h"
// #include "gimbal_task.h"



//     // int windwill_flag = 0; // 大风车击打标志位 0 自瞄 1 大风车

//     int16_t mode_flag = 1; // 0自瞄以yaw为准，1导航以turnleft为准，2巡航
// // int16_t top_flag = 0;  // 未用到
// int16_t shoot_flag = 0;

// int time1 = 0;
// int time2 = 0;
// int time3 = 0;
// int time4 = 0;

// uint8_t Buffer[34], status;

// DIRECTION WIND_DIRECTION = CLOCKWISE;

// VISION_RESET VISION_RESET_FLAG = OFF;

// Pack_t pack;
// uint8_t receive_data[36];
// float distance;
// uint8_t start_receive_flag = 0;
// Pack_rx_t pack_rx;
// /**
//  * @brief CRC16 Caculation function
//  * @param[in] pchMessage : Data to Verify,
//  * @param[in] dwLength : Stream length = Data + checksum
//  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
//  * @return : CRC16 checksum
//  */
// uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
// {
//     uint8_t ch_data;

//     if (pchMessage == NULL)
//         return 0xFFFF;
//     while (dwLength--)
//     {
//         ch_data = *pchMessage++;
//         wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
//     }

//     return wCRC;
// }

// /**
//  * @brief CRC16 Verify function
//  * @param[in] pchMessage : Data to Verify,
//  * @param[in] dwLength : Stream length = Data + checksum
//  * @return : True or False (CRC Verify Result)
//  */

// bool Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
// {
//     uint16_t w_expected = 0;

//     if ((pchMessage == NULL) || (dwLength <= 2))
//         return false;

//     w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
//     return (
//         (w_expected & 0xff) == pchMessage[dwLength - 2] &&
//         ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
// }
// /**

// @brief Append CRC16 value to the end of the buffer
// @param[in] pchMessage : Data to Verify,
// @param[in] dwLength : Stream length = Data + checksum
// @return none
// */
// void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
// {
//     uint16_t w_crc = 0;

//     if ((pchMessage == NULL) || (dwLength <= 2))
//         return;

//     w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

//     pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
//     pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
// }

// void DMA_Send(void)
// {
//     time4 = HAL_GetTick() - time3;
//     time3 = HAL_GetTick();
//     pack.hander = 0x5A;
//     pack.detect_color = 1;
//     pack.target_id = 0x01;
//     pack.roll = -gimbal_r.IMU_actual_angle;
//     pack.pitch = gimbal_p.IMU_actual_angle;
//     pack.yaw = gimbal_y.IMU_actual_angle;
//     pack.crc16 = 0xffff;
//     memcpy(Buffer, &pack, sizeof(pack));
//     Append_CRC16_Check_Sum(Buffer, sizeof(pack));
//     CDC_Transmit_FS(Buffer, sizeof(pack));
// }

// void cdc_vcp_data_rx(uint8_t *buf, uint32_t Len)
// {
//     uint8_t res;
//     for (uint16_t j = 0; j < Len; j++)
//     {
//         if (buf[j] == 0xA5) // 帧头
//         {
//             start_receive_flag = 1;
//         }
//         if (start_receive_flag == 1) //
//         {
//             memcpy(receive_data, buf, Len);
//             memcpy(&pack_rx, receive_data, Len);
//         }
//         if (start_receive_flag == 1) //????
//         {
//             // Self_aim(pack_rx.target_x, pack_rx.target_y, pack_rx.target_z, &gimbal_y.add_angle, &gimbal_p.add_angle, &distance);
//             //vision_sent.Control_privilege = pack_rx.UP_flag;
//             // 改为下位机解算后注释掉
//             gimbal_y.add_angle = pack_rx.target_yaw;
//             gimbal_p.add_angle = pack_rx.target_pitch;
//             vision_sent.Control_privilege = 1;
//             start_receive_flag = 0;
//             break;
//         }
//     }
// }
