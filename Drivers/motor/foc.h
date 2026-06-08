/**
 * @file    foc.h
 * @brief   FOC 控制头文件 (照抄教程 V0)
 */
#ifndef __FOC_H
#define __FOC_H

#include "motor_runtime_param.h"
#include "arm_math.h"
#include <stdint.h>

/* 控制模式枚举 */
typedef enum {
    control_type_null                     = 0,
    control_type_position                = 1,
    control_type_speed                   = 2,
    control_type_torque                  = 3,
    control_type_speed_torque            = 4,
    control_type_position_speed_torque   = 5
} motor_control_type;

/* 控制上下文 */
typedef struct {
    motor_control_type type;
    float position;
    float speed;
    float torque_norm_d;
    float torque_norm_q;
    float max_speed;
    float max_torque_norm;
} motor_control_context_t;

extern motor_control_context_t motor_control_context;
extern arm_pid_instance_f32 pid_position;
extern arm_pid_instance_f32 pid_speed;
extern float foc_accel_limit;  /* 位置环梯形加速度限制 (rad/s²), 默认 5.0 */

/* 函数声明 */
float cycle_diff(float diff, float cycle);
void  foc_forward(float d, float q, float rotor_rad);

void  lib_position_control(float rad);
void  lib_speed_control(float speed);
void  lib_torque_control(float torque_norm_d, float torque_norm_q);
void  lib_speed_torque_control(float speed_rad, float max_torque_norm);
void  lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm);

void  set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p,    float speed_i,    float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d);

#endif /* __FOC_H */
