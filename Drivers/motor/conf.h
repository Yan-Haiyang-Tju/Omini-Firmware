/**
 * @file    conf.h
 * @brief   电机/硬件参数配置
 */
#ifndef __CONF_H
#define __CONF_H

#include "global_def.h"

/* 电机物理参数 */
#define POLE_PAIRS              7       /* 极对数 (2804 电机 = 14 极) */

/* 电路参数 */
#define R_SHUNT                 0.02f   /* 电流采样电阻 (Ω)          */
#define OP_GAIN                 50.0f   /* INA199A 运放增益           */
#define MAX_CURRENT             2.0f    /* 最大 q 轴电流 (A)          */
#define ADC_REFERENCE_VOLT      3.3f    /* ADC 参考电压               */
#define ADC_BITS                12      /* ADC 精度                   */

/* MCU 配置 */
#define motor_pwm_freq          20000   /* PWM 等效频率 (Hz)          */
#define motor_speed_calc_freq   930     /* 速度计算频率 (Hz)          */

/* 软件参数 */
#define position_cycle          (90.0f * PI) /* 多圈周期 = ±45π (覆盖全行程) */

#endif /* __CONF_H */
