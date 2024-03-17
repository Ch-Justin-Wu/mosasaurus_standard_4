#ifndef __USB_COMMUCATION_H_
#define __USB_COMMUCATION_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include <math.h>
// x朝前 y朝左 z朝上

const float g = 12; // 重力加速度
const float bullet_v = 40.0; // 子弹速度

void Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance);

#ifdef __cplusplus
}
#endif
#endif