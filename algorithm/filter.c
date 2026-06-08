/**
 * @file    filter.c
 * @brief   一阶低通滤波器实现
 */
#include "filter.h"

float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_output;
}
