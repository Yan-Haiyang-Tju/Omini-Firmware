/**
 * @file    filter.h
 * @brief   信号滤波器
 */
#ifndef __FILTER_H
#define __FILTER_H

float low_pass_filter(float input, float last_output, float alpha);

#endif /* __FILTER_H */
