#include "Self_aim.h"
#include "arm_math.h"
/**
 * 计算给定向量的偏航角（yaw）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float calc_yaw(float x, float y, float z)
{
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    // float yaw = atan2f(y, x);//
    // arm_math.h库中的atan2f函数
    float yaw = arm_asin_f32(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = -(yaw * 180 / 3.1415926f); // 向左为正，向右为负

    return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float calc_distance(float x, float y, float z)
{
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */
float calc_pitch(float x, float y, float z)
{
    // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
    float pitch = atan2f(z, sqrtf(x * x + y * y));

    // 使用重力加速度模型迭代更新俯仰角
    for (size_t i = 0; i < 20; i++)
    {
        float v_x = bullet_v * cosf(pitch);
        float v_y = bullet_v * sinf(pitch);

        float t = sqrtf(x * x + y * y) / v_x;
        float h = v_y * t - 0.5f * g * t * t;
        float dz = z - h;

        if (fabsf(dz) < 0.01f)
        {
            break;
        }

        // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
        pitch += asinf(dz / calc_distance(x, y, z));
    }

    // 将弧度制的俯仰角转换为角度制
    pitch = -(pitch * 180 / 3.1415926f); // 向上为负，向下为正

    return pitch;
}
/**
 * 计算计算yaw，pitch
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
    *yaw = -calc_yaw(x, y, z);
    *pitch = -calc_pitch(x, y, z); // 4号可能要添加负号
    *distance = calc_distance(x, y, z);
}