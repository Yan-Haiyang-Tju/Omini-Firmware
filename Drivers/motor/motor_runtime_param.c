/**
 * @file    motor_runtime_param.c
 * @brief   电机运行时变量定义
 */
#include "motor_runtime_param.h"

float motor_i_u       = 0.0f;
float motor_i_v       = 0.0f;
float motor_i_d       = 0.0f;
float motor_i_q       = 0.0f;
float motor_speed     = 0.0f;
float motor_logic_angle = 0.0f;
float encoder_angle   = 0.0f;
float rotor_zero_angle = 0.0f;
