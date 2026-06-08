/**
 * @file    User_CAN.h
 * @brief   CAN 通信头文件 (对齐 ESP32 Choloepus 协议)
 */
#ifndef __USER_CAN_H
#define __USER_CAN_H

#include "stm32f1xx_hal.h"

/* ─── 设备 ID (可改) ─── */
#define CAN_DEVICE_ID       20

/* ─── CAN ID (ODrive 风格: (id<<5)+cmd) ─── */
#define CAN_CMD_ID          ((CAN_DEVICE_ID << 5) + 9)    /* 0x0289 控制帧 */
#define CAN_STAT_ID         ((CAN_DEVICE_ID << 5) + 17)   /* 0x02A9 状态帧 */

/* ─── 命令数据布局 (8 字节, 对齐 ESP32) ─── */
#define CAN_DATA_MODE       2       /* Data[2]: 模式       */
#define CAN_DATA_PARAM      1       /* Data[1]: 参数选择   */
#define CAN_DATA_VAL_H      3       /* Data[3]: 值×100 高 */
#define CAN_DATA_VAL_L      4       /* Data[4]: 值×100 低 */

/* ─── 模式枚举 ─── */
#define CAN_MODE_CALIB      0x00    /* 复位/校准           */
#define CAN_MODE_TORQUE     0x01    /* 力矩控制            */
#define CAN_MODE_POS        0x02    /* 位置控制(含速限)    */

/* ─── 参数选择 (Data[1]) ─── */
#define CAN_PARAM_MAIN      0x00    /* 主命令              */
#define CAN_PARAM_SPD_LIMIT 0x01    /* 速度限制 rad/s      */
#define CAN_PARAM_CUR_LIMIT 0x02    /* 电流限制 (×100)     */
#define CAN_PARAM_KP_POS    0x03    /* 位置 Kp (÷10)       */
#define CAN_PARAM_KD_POS    0x04    /* 位置 Kd (÷10)       */
#define CAN_PARAM_KP_SPD    0x05    /* 速度 Kp (÷1000)     */
#define CAN_PARAM_KI_SPD    0x06    /* 速度 Ki (÷10000)    */
#define CAN_PARAM_ACCEL     0x07    /* 加速度限制 (÷10)     */

/* ─── 默认值 ─── */
#define CAN_DEF_SPD_LIMIT   12.0f   /* 默认速度限制 rad/s  */
#define CAN_DEF_CUR_LIMIT   1.0f    /* 默认电流限制 int16   */

/* ─── API ─── */
void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_SendStatus(float pos_rad, float speed, float id, float iq);
void CAN_StartCalibration(void);

#endif /* __USER_CAN_H */
