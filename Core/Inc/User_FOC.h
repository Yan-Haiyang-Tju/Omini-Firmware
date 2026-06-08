/**
 * @file    User_FOC.h
 * @brief   FOC 硬件接口 (对齐/使能/编码器/PWM 写)
 */
#ifndef __USER_FOC_H
#define __USER_FOC_H

#include "stm32f1xx_hal.h"

extern int8_t   rotor_dir;
extern uint16_t foc_cached_raw;

void FOC_Init(TIM_HandleTypeDef *htim);
void FOC_Enable(void);
void FOC_Disable(void);
void FOC_AlignRotor(void);
void FOC_UpdateAngle(void);
void set_pwm_duty(float d_u, float d_v, float d_w);

#endif /* __USER_FOC_H */
