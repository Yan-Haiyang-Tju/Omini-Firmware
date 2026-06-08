/**
 * @file    User_FOC.c
 * @brief   FOC 硬件接口实现
 *          ADC 回调 (FOC 主循环) + TIM3 回调 (速度计算)
 *          + 编码器更新 + PWM 写 + 对齐/使能
 */
#include "User_FOC.h"
#include "main.h"
#include "User_AS5600.h"
#include "User_ADC.h"
#include "foc.h"
#include "motor_runtime_param.h"
#include "filter.h"
#include "arm_math.h"
#include "global_def.h"

/* ———— 外部句柄 (main.c) ———— */
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern TIM_HandleTypeDef  htim1;

/* ———— 私有变量 ———— */
static TIM_HandleTypeDef *foc_htim = NULL;

#define FOC_DUTY_MIN    0.0f
#define FOC_DUTY_MAX    0.9f

/* ———— 全局变量 ———— */
uint16_t foc_cached_raw = 0;
int8_t   rotor_dir      = 1;

/* ======================== PWM 写 (照抄教程) ======================== */

static uint16_t duty_to_compare(float duty)
{
    if (!(duty >= FOC_DUTY_MIN)) {
        duty = FOC_DUTY_MIN;
    } else if (duty > FOC_DUTY_MAX) {
        duty = FOC_DUTY_MAX;
    }

    return (uint16_t)(duty * (float)foc_htim->Instance->ARR);
}

void set_pwm_duty(float d_u, float d_v, float d_w)
{
    if (foc_htim == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_1, duty_to_compare(d_u));
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_2, duty_to_compare(d_v));
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_3, duty_to_compare(d_w));
    if (primask == 0u) {
        __enable_irq();
    }
}

/* ======================== 编码器 ======================== */

void FOC_UpdateAngle(void)
{
    uint16_t raw;
    if (AS5600_ReadRawAngle(&raw) != 0)
        return;

    float new_angle = (float)raw * 2.0f * PI / 4095.0f;

    /* 多圈累积 (照抄教程 SPI 回调) */
    static float last_enc = 0.0f;
    static int   once     = 1;
    if (once) {
        once = 0;
        last_enc = new_angle;
    }
    float diff_angle = cycle_diff(new_angle - last_enc, 2.0f * PI);
    last_enc = new_angle;
    encoder_angle = new_angle;
    foc_cached_raw = raw;  /* 更新缓存 */
    motor_logic_angle = cycle_diff(motor_logic_angle + diff_angle,
                                    position_cycle);
}

/* ======================== 对齐 ======================== */

void FOC_AlignRotor(void)
{
    uint16_t raw;

    FOC_Disable();

    /* 基础矢量1: 直写 CCR 绕开限幅 */
    __disable_irq();
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_1,
                          (uint16_t)(0.5f * (float)foc_htim->Instance->ARR));
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_3, 0);
    __enable_irq();

    FOC_Enable();
    HAL_Delay(400);

    /* 读编码器, 记录零位 */
    if (AS5600_ReadRawAngle(&raw) == 0) {
        encoder_angle      = (float)raw * 2.0f * PI / 4095.0f;
        rotor_zero_angle   = encoder_angle;
        motor_logic_angle  = 0.0f;
    }

    FOC_Disable();
}

/* ======================== 使能 ======================== */

void FOC_Init(TIM_HandleTypeDef *htim)
{
    foc_htim = htim;
    HAL_TIM_PWM_Start(foc_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(foc_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(foc_htim, TIM_CHANNEL_3);
    set_pwm_duty(0.0f, 0.0f, 0.0f);
}

void FOC_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, EN1_Pin | EN2_Pin | EN3_Pin, GPIO_PIN_SET);
}

void FOC_Disable(void)
{
    HAL_GPIO_WritePin(GPIOB, EN1_Pin | EN2_Pin | EN3_Pin, GPIO_PIN_RESET);
}

void FOC_EmergencyStop(void)
{
    if (foc_htim != NULL) {
        __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(foc_htim, TIM_CHANNEL_3, 0);
    }

    FOC_Disable();
}

/* ======================== ADC 回调 (FOC 主循环, 照抄教程) ======================== */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
        return;

    /* 1. 读电流 */
    uint16_t adc_ia = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                           ADC_INJECTED_RANK_1);
    uint16_t adc_ib = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                           ADC_INJECTED_RANK_1);

    motor_i_u = ADC_TO_CURRENT(adc_ia, adc_zero_ia);
    motor_i_v = ADC_TO_CURRENT(adc_ib, adc_zero_ib);

    /* 2. Clarke */
    float i_alpha = 0.0f;
    float i_beta  = 0.0f;
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);

    /* 3. Park */
    float sin_val = arm_sin_f32(rotor_logic_angle);
    float cos_val = arm_cos_f32(rotor_logic_angle);
    float _motor_i_d = 0.0f;
    float _motor_i_q = 0.0f;
    arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q,
                 sin_val, cos_val);

    /* 4. 低通滤波 */
    float filter_alpha = 0.1f;
    motor_i_d = low_pass_filter(_motor_i_d, motor_i_d, filter_alpha);
    motor_i_q = low_pass_filter(_motor_i_q, motor_i_q, filter_alpha);

    /* 5. 控制模式调度 */
    switch (motor_control_context.type) {
    case control_type_position:
        lib_position_control(motor_control_context.position);
        break;
    case control_type_speed:
        lib_speed_control(motor_control_context.speed);
        break;
    case control_type_torque:
        lib_torque_control(motor_control_context.torque_norm_d,
                           motor_control_context.torque_norm_q);
        break;
    case control_type_speed_torque:
        lib_speed_torque_control(motor_control_context.speed,
                                 motor_control_context.max_torque_norm);
        break;
    case control_type_position_speed_torque:
        lib_position_speed_torque_control(
            motor_control_context.position,
            motor_control_context.max_speed,
            motor_control_context.max_torque_norm);
        break;
    default:
        break;
    }
}

/* ======================== TIM3 回调 (速度计算, 照抄教程) ======================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3)
        return;

    FOC_UpdateAngle();  /* 每次 TIM3 中断先读编码器, 保证 speed 准确 */

    static float encoder_angle_last = 0.0f;
    static int   once               = 1;
    if (once) {
        once = 0;
        encoder_angle_last = encoder_angle;
    }

    float diff_angle = cycle_diff(encoder_angle - encoder_angle_last,
                                  2.0f * PI);
    encoder_angle_last = encoder_angle;

    float _motor_speed = diff_angle * (float)motor_speed_calc_freq;
    float filter_alpha = 0.07f;
    motor_speed = low_pass_filter(_motor_speed, motor_speed, filter_alpha);
}
