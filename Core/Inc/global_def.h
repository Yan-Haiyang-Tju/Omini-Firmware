/**
 * @file    global_def.h
 * @brief   全局宏定义
 */
#ifndef __GLOBAL_DEF_H
#define __GLOBAL_DEF_H

#define PI              3.14159265358979f
#define deg2rad(a)      (PI * (a) / 180.0f)
#define rad2deg(a)      (180.0f * (a) / PI)
#define max(a, b)       ((a) > (b) ? (a) : (b))
#define min(a, b)       ((a) < (b) ? (a) : (b))

#endif /* __GLOBAL_DEF_H */
