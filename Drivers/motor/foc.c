/**
 * @file    foc.c
 * @brief   FOC 控制实现 (照抄教程 V0, 适配 AS5600 I2C)
 */
#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include <stdbool.h>

/* ———— 本地宏 ———— */
#define deg2rad(a)  (PI * (a) / 180.0f)
#define rad2deg(a)  (180.0f * (a) / PI)
#define max(a, b)   ((a) > (b) ? (a) : (b))
#define min(a, b)   ((a) < (b) ? (a) : (b))
#define rad60       deg2rad(60.0f)
#define SQRT3       1.73205080756887729353f

/* ———— PID 实例 (extern, 供 main.c 串口调参) ———— */
arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;

/* ———— 全局变量 ———— */
float foc_accel_limit = 5.0f;   /* 位置环梯形加速度限制 (rad/s²) */

/* ———— 控制上下文 ———— */
motor_control_context_t motor_control_context;

/* ———— weak 默认 (main.c 覆盖) ———— */
__attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w);
__attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
{
    while (1);
}

/* ======================== SVPWM ======================== */

static void svpwm(float phi, float d, float q,
                  float *d_u, float *d_v, float *d_w)
{
    d = min(d, 1.0f);
    d = max(d, -1.0f);
    q = min(q, 1.0f);
    q = max(q, -1.0f);

    const int v[6][3] = {
        {1,0,0}, {1,1,0}, {0,1,0},
        {0,1,1}, {0,0,1}, {1,0,1}
    };
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0.0f;
    float beta  = 0.0f;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0.0f;
    bool B = fabsf(beta) > SQRT3 * fabsf(alpha);
    bool C = alpha > 0.0f;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha
              - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60)
              - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1.0f - t_m - t_n;

    *d_u = t_m * v[sector - 1][0]
         + t_n * v[sector % 6][0]
         + t_0 / 2.0f;
    *d_v = t_m * v[sector - 1][1]
         + t_n * v[sector % 6][1]
         + t_0 / 2.0f;
    *d_w = t_m * v[sector - 1][2]
         + t_n * v[sector % 6][2]
         + t_0 / 2.0f;
}

/* ======================== 前向 FOC ======================== */

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0.0f;
    float d_v = 0.0f;
    float d_w = 0.0f;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

/* ======================== cycle_diff ======================== */

float cycle_diff(float diff, float cycle)
{
    if (diff > (cycle / 2.0f))
        diff -= cycle;
    else if (diff < (-cycle / 2.0f))
        diff += cycle;
    return diff;
}

/* ======================== 位置环 ======================== */

static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

/* ======================== 速度环 ======================== */

static float speed_loop(float speed_rad)
{
    float diff = speed_rad - motor_speed;
    float out = arm_pid_f32(&pid_speed, diff);
    pid_speed.state[2] = fmaxf(fminf(pid_speed.state[2], 1.0f), -1.0f);
    return out;
}

/* ======================== d轴电流环 ======================== */

static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    float out = arm_pid_f32(&pid_torque_d, diff);
    pid_torque_d.state[2] = fmaxf(fminf(pid_torque_d.state[2], 1.0f), -1.0f);
    return out;
}

/* ======================== q轴电流环 ======================== */

static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    float out = arm_pid_f32(&pid_torque_q, diff);
    pid_torque_q.state[2] = fmaxf(fminf(pid_torque_q.state[2], 1.0f), -1.0f);
    return out;
}

/* ======================== 控制函数 ======================== */

void lib_position_control(float rad)
{
    float d = 0.0f;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_control(float speed)
{
    float d = 0.0f;
    float q = speed_loop(speed);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(fabsf(torque_norm), max_torque_norm)
                * (torque_norm > 0.0f ? 1.0f : -1.0f);
    lib_torque_control(0.0f, torque_norm);
}

void lib_position_speed_torque_control(float position_rad,
    float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabsf(speed_rad), max_speed_rad)
              * (speed_rad > 0.0f ? 1.0f : -1.0f);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

/* ======================== PID 参数设置 ======================== */

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p,    float speed_i,    float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;

    arm_pid_init_f32(&pid_position,  false);
    arm_pid_init_f32(&pid_speed,     false);
    arm_pid_init_f32(&pid_torque_d,  false);
    arm_pid_init_f32(&pid_torque_q,  false);
}
