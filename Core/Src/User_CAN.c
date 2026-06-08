/**
 * @file    User_CAN.c
 * @brief   CAN 通信实现 (对齐 ESP32 Choloepus 协议)
 */
#include "User_CAN.h"
#include "main.h"
#include "foc.h"

/* ─── 外部变量 ─── */
extern float    pos_target_rad;
extern float    torque_target;
extern uint8_t  gripper_recal;
extern uint8_t  foc_mode_pending;   /* 0=none, 1=torque, 2=position */
extern arm_pid_instance_f32 pid_position;
extern arm_pid_instance_f32 pid_speed;
extern float foc_accel_limit;
extern motor_control_context_t motor_control_context;

/* ─── 内部句柄 ─── */
static CAN_HandleTypeDef *can_handle = NULL;

/* ======================== CAN 初始化 ======================== */

void CAN_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return;

    can_handle = hcan;

    /* 滤波器: 接收所有 ID */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(can_handle, &filter);

    HAL_CAN_Start(can_handle);
    HAL_CAN_ActivateNotification(can_handle, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* ======================== CAN 接收回调 ======================== */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef   header;
    uint8_t               data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        return;

    if (header.StdId != CAN_CMD_ID)
        return;

    uint8_t  mode  = data[CAN_DATA_MODE];
    uint8_t  param = data[CAN_DATA_PARAM];
    int16_t  val   = (int16_t)((uint16_t)data[CAN_DATA_VAL_H] << 8
                             | data[CAN_DATA_VAL_L]);
    float    fval  = (float)val / 100.0f;

    /* ——— 参数设置 (Data[1] ≠ 0) ——— */
    if (param == CAN_PARAM_SPD_LIMIT) {
        motor_control_context.max_speed = (float)val / 10.0f;  /* rad/s */
        return;
    }
    if (param == CAN_PARAM_CUR_LIMIT) {
        motor_control_context.max_torque_norm = fval;  /* 0~1 */
        return;
    }
    if (param == CAN_PARAM_KP_POS) {
        pid_position.Kp = (float)val / 10.0f;   return;
    }
    if (param == CAN_PARAM_KD_POS) {
        pid_position.Kd = (float)val / 10.0f;   return;
    }
    if (param == CAN_PARAM_KP_SPD) {
        pid_speed.Kp = (float)val / 1000.0f;    return;
    }
    if (param == CAN_PARAM_KI_SPD) {
        pid_speed.Ki = (float)val / 10000.0f;   return;
    }
    if (param == CAN_PARAM_ACCEL) {
        foc_accel_limit = (float)val / 10.0f;   return;
    }

    /* ——— 主命令 (Data[1] = 0) ——— */
    switch (mode) {

    case CAN_MODE_CALIB:
        gripper_recal = 1;
        break;

    case CAN_MODE_TORQUE:
        torque_target   = fval;
        foc_mode_pending = 1;
        break;

    case CAN_MODE_POS: {
        float target = fval;
        if (fabsf(target - pos_target_rad) < 0.005f) return;
        pos_target_rad   = target;
        foc_mode_pending = 2;
        break;
    }

    default:
        break;
    }
}

/* ======================== CAN 发送状态 ======================== */

void CAN_SendStatus(float pos_rad, float speed, float id, float iq)
{
    if (can_handle == NULL) return;

    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            mailbox;

    int16_t p = (int16_t)(pos_rad * 100.0f);
    int16_t s = (int16_t)(speed * 10.0f);
    int16_t q = (int16_t)(iq * 1000.0f);
    int16_t d = (int16_t)(id * 1000.0f);

    tx_data[0] = (uint8_t)(p >> 8);
    tx_data[1] = (uint8_t)(p & 0xFF);
    tx_data[2] = (uint8_t)(s >> 8);
    tx_data[3] = (uint8_t)(s & 0xFF);
    tx_data[4] = (uint8_t)(q >> 8);
    tx_data[5] = (uint8_t)(q & 0xFF);
    tx_data[6] = (uint8_t)(d >> 8);
    tx_data[7] = (uint8_t)(d & 0xFF);

    tx_header.StdId    = CAN_STAT_ID;
    tx_header.ExtId    = 0;
    tx_header.IDE      = CAN_ID_STD;
    tx_header.RTR      = CAN_RTR_DATA;
    tx_header.DLC      = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(can_handle, &tx_header, tx_data, &mailbox);
}

/* ======================== 校准触发 ======================== */

void CAN_StartCalibration(void)
{
    gripper_recal = 1;
}
