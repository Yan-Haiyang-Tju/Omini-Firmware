/**
 * @file    motor_runtime_param.h
 * @brief   电机运行时变量声明
 */
#ifndef __MOTOR_RUNTIME_PARAM_H
#define __MOTOR_RUNTIME_PARAM_H

#include "conf.h"

/* 转子角度宏 (照抄教程) */
#define rotor_phy_angle     (encoder_angle - rotor_zero_angle)
#define rotor_logic_angle   (rotor_phy_angle * POLE_PAIRS)

extern float motor_i_u;
extern float motor_i_v;
extern float motor_i_d;
extern float motor_i_q;
extern float motor_speed;
extern float motor_logic_angle;
extern float encoder_angle;
extern float rotor_zero_angle;

#endif /* __MOTOR_RUNTIME_PARAM_H */
