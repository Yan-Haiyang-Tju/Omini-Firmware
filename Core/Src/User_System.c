/**
 * @file    User_System.c
 * @brief   System state, fault code and LED status implementation.
 */
#include "User_System.h"
#include "main.h"

#define LED_ACTIVE_LEVEL      GPIO_PIN_RESET
#define LED_INACTIVE_LEVEL    GPIO_PIN_SET
#define LED_INIT_PERIOD_MS    500u
#define LED_FAULT_STEP_MS     180u
#define LED_FAULT_PAUSE_MS    900u
#define LED_FAULT_MAX_PULSES  7u

static volatile system_state_t system_state = SYS_STATE_BOOT;
static volatile fault_code_t system_fault = FAULT_NONE;

static void led_write(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? LED_ACTIVE_LEVEL : LED_INACTIVE_LEVEL);
}

void SystemStatus_Init(void)
{
    system_state = SYS_STATE_INIT;
    system_fault = FAULT_NONE;
    SystemStatus_LedOk(0);
    SystemStatus_LedError(0);
}

void SystemStatus_LedOk(uint8_t on)
{
    led_write(LED_OK_GPIO_Port, LED_OK_Pin, on);
}

void SystemStatus_LedError(uint8_t on)
{
    led_write(LED_ERROR_GPIO_Port, LED_ERROR_Pin, on);
}

void SystemStatus_SetState(system_state_t state)
{
    if (system_state == SYS_STATE_FAULT && system_fault != FAULT_NONE) {
        return;
    }

    system_state = state;
}

system_state_t SystemStatus_GetState(void)
{
    return system_state;
}

void SystemStatus_SetFault(fault_code_t fault)
{
    if (fault == FAULT_NONE) {
        return;
    }

    system_fault = fault;
    system_state = SYS_STATE_FAULT;
}

void SystemStatus_ClearFault(void)
{
    system_fault = FAULT_NONE;
    system_state = SYS_STATE_INIT;
}

fault_code_t SystemStatus_GetFault(void)
{
    return system_fault;
}

uint8_t SystemStatus_IsFaultActive(void)
{
    return (system_state == SYS_STATE_FAULT || system_fault != FAULT_NONE);
}

void SystemStatus_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (SystemStatus_IsFaultActive()) {
        uint32_t pulses = (uint32_t)system_fault;
        if (pulses == 0u) {
            pulses = 1u;
        } else if (pulses > LED_FAULT_MAX_PULSES) {
            pulses = LED_FAULT_MAX_PULSES;
        }

        uint32_t active_window = pulses * LED_FAULT_STEP_MS * 2u;
        uint32_t cycle = active_window + LED_FAULT_PAUSE_MS;
        uint32_t phase = now % cycle;
        uint8_t red_on = 0u;

        if (phase < active_window) {
            red_on = ((phase / LED_FAULT_STEP_MS) % 2u) == 0u;
        }

        SystemStatus_LedOk(0);
        SystemStatus_LedError(red_on);
        return;
    }

    SystemStatus_LedError(0);

    switch (system_state) {
    case SYS_STATE_RUN:
        SystemStatus_LedOk(1);
        break;

    case SYS_STATE_BOOT:
    case SYS_STATE_INIT:
    case SYS_STATE_SENSOR_CHECK:
    case SYS_STATE_ADC_CALIB:
    case SYS_STATE_ALIGN:
    case SYS_STATE_HOME:
        SystemStatus_LedOk(((now / LED_INIT_PERIOD_MS) % 2u) == 0u);
        break;

    default:
        SystemStatus_LedOk(0);
        break;
    }
}
