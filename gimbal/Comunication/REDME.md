

# 上位机USB通信

与上位机USB通信可直接使用CubeMX配置完成

## 接收与发送函数

接收函数为下列函数，该函数具有中断特点，当有接收到数据时自动执行该函数，后续在此函数中处理接收数据即可，Buf为接收数据暂存区，Len为接收数据长度，可直接使用。

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
```

此函数为发送函数，将所需发送的数组以及其长度直接传入下列函数即可，参数同上含义

```c
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
```

## 结构体定义

此为发送和接收的结构体类型,使用特定格式使其地址紧凑，在使用memcpy函数时不会出错，前一个为发送的结构体，后一个为接收结构体

### 发送结构体

hander为0x5A，帧头作用

detect_color为1，无作用,参考裁判系统，后续和上位机联系可删除

target_id为0x01，暂无实际作用，原先暂定于发送不同数据标志，用于CAN通信，现因为USB可实现多位发送，暂时不用但保留

后面三个为roll，pitch，yaw角度，为IMU实际角度（IMU反馈角度为弧度制，需转化为角度制）

最后一位为16位crc16位校验，用于筛选无用脏数据

### 接收结构体

hander为0xA5，帧头

后续分别为yaw，pitch，角度值，为目标角度，并未增量，xyz为目标装甲板的三维坐标系用于步兵火控弹道解算，crc16为校验位验收

```c
#pragma pack(1)
typedef struct
{
	uint8_t hander;
	uint8_t detect_color;
	uint8_t target_id;
	float roll;
	float pitch;
	float yaw;
	uint16_t crc16;
}Pack_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint8_t hander;
	float target_yaw;
	float target_pitch;
	float target_x;
	float target_y;
	float target_z;	
    uint8_t UP_flag;
	uint16_t crc16;
}Pack_rx_t;
#pragma pack()
```

## 校验问题

```c
static const uint16_t CRC16_INIT = 0xFFFF;

static const uint16_t W_CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

```

以上为与上位机进行校验时采用的16位数组，用于以下函数种，使用时直接放入头文件即可。

```c
/* USER CODE BEGIN PRIVATE_TYPES */
/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}
```

此函数被后面两个函数给调用，主要作用用于向数组写入校验位和读取校验位

```c
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
```

此函数用于接收时校验接收数据，将接收的数组和长度传入即可，若校验成功则返回1，反之返回0.使用此函数时从中断接收函数里验证后再处理数据即可，下面为接收实例，当数组里存在校验位时返回布尔量为1，则进行if操作

```c
if(Verify_CRC16_Check_Sum(Buf,*Len))
{
    cdc_vcp_data_rx(Buf,*Len);
}
```

```c
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
```

此函数为用于发送时对数组传入校验位，下面时相关使用实例

```c
pack.hander       =0x5A;
pack.detect_color =1;
pack.target_id    =0x01;
pack.roll         =-gimbal_r.IMU_actual_angle;
pack.pitch        =gimbal_p.IMU_actual_angle;
pack.yaw          =gimbal_y.IMU_actual_angle;
pack.crc16        =0xffff;
memcpy(Buffer,&pack,sizeof(pack));
Append_CRC16_Check_Sum(Buffer,sizeof(pack));
```

再发送时将结构体数据直接通过memcpy函数赋值给数组，后面对数组进行校验的函数，后面直接调用发送函数即可。

```c
/**
 * 计算给定向量的偏航角（yaw）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float calc_yaw(float x, float y, float z) {
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    float yaw = atan2f(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = -(yaw * 180 / 3.1415926); // 向左为正，向右为负

    return yaw;
}
```
通过xyz的目标值区去计算yaw轴的角度，然后通过yaw轴的值来控制云台，返回值为绝对值。

```c
/**
 * 计算给定向量的俯仰角（pitch）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */
float calc_pitch(float x, float y, float z) {
    // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
    float pitch = atan2f(z, sqrtf(x * x + y * y));

    // 使用重力加速度模型迭代更新俯仰角
    for (size_t i = 0; i < 20; i++) {
        float v_x = bullet_v * cosf(pitch);
        float v_y = bullet_v * sinf(pitch);

        float t = sqrtf(x * x + y * y) / v_x;
        float h = v_y * t - 0.5 * g * t * t;
        float dz = z - h;

        if (fabsf(dz) < 0.01) {
            break;
        }

        // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
        pitch += asinf(dz / calc_distance(x, y, z));
    }

    // 将弧度制的俯仰角转换为角度制
    pitch = -(pitch * 180 / 3.1415926); // 向上为负，向下为正

    return pitch;
}
```
通过xyz的目标值区去计算pitch轴的角度，然后通过pitch轴的值来控制云台，返回值为绝对值。

```c
/**
 * 计算给定向量的欧几里德距离。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float calc_distance(float x, float y, float z) {
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}
```

```c
通过xyz的目标值区去计算距离，返回值为绝对值，可用于UI中。

/**
 * 计算计算yaw，pitch
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Self_aim(float x,float y,float z,float *yaw,float *pitch,float *distance)
{
    *yaw = -calc_yaw(x, y, z);
    *pitch = calc_pitch(x, y, z);
    *distance = calc_distance(x, y, z);
}
```
最后实现函数，传入xyz值返回yaw和pitch值。