#ifndef MOTOR_H
#define MOTOR_H

#include "car_config.h"

/* 速度环 PID */
typedef struct {
    float Kp, Ki, Kd;
    float target;
    float measure;
    float error;
    float pre_error;
    float integral;
    float output;
} PID_t;

/* 方向环 PI */
typedef struct {
    float Kp, Ki;
    float target;
    float measure;
    float error, pre_error, integral;
    float output;
} DirPID_t;

extern PID_t pid_L;
extern PID_t pid_R;
extern DirPID_t dir_pid;

float PID_Calc(PID_t *p, float measure);
float DirPID_Calc(DirPID_t *p, float measure);
void Motor_Output(float out_L, float out_R);

/* [GND 2026-11-14] 切换左右轮速度环增益（同时清积分/pre_error）。
   过弯 pivot 与 '2' 原地转都调用它，保证两条路径的控制强度一致。 */
void Motor_SetSpeedGains(float kp, float ki);

/* [BRAKE 2026-11-15] 死区补偿开关。
   补偿的用途是"从静止起步"时别卡在死区里；但刹车段是从运动减到静止，
   低目标时它会把输出硬抬到 150（能动的最小占空比），表现为"一顿一顿"。
   刹车段调用 Motor_SetBoost(0) 关掉，出弯/停车时恢复 1。 */
void Motor_SetBoost(uint8_t en);

#endif /* MOTOR_H */
