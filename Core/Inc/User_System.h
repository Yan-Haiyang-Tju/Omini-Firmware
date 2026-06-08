/**
 * @file    User_System.h
 * @brief   System state, fault code and LED status interface.
 */
#ifndef __USER_SYSTEM_H
#define __USER_SYSTEM_H

#include "stm32f1xx_hal.h"

typedef enum {
    SYS_STATE_BOOT = 0,
    SYS_STATE_INIT,
    SYS_STATE_SENSOR_CHECK,
    SYS_STATE_ADC_CALIB,
    SYS_STATE_ALIGN,
    SYS_STATE_HOME,
    SYS_STATE_RUN,
    SYS_STATE_FAULT
} system_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_AS5600_INIT = 1,
    FAULT_AS5600_TIMEOUT = 2,
    FAULT_DRIVER = 3,
    FAULT_ADC_CALIB = 4,
    FAULT_ALIGN = 5,
    FAULT_I2C = 6,
    FAULT_STARTUP = 7
} fault_code_t;

void SystemStatus_Init(void);
void SystemStatus_Task(void);

void SystemStatus_SetState(system_state_t state);
system_state_t SystemStatus_GetState(void);

void SystemStatus_SetFault(fault_code_t fault);
void SystemStatus_ClearFault(void);
fault_code_t SystemStatus_GetFault(void);
uint8_t SystemStatus_IsFaultActive(void);

void SystemStatus_LedOk(uint8_t on);
void SystemStatus_LedError(uint8_t on);

#endif /* __USER_SYSTEM_H */
