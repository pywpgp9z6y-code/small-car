#include "main.h"
#include "motor.h"

/* 左右轮速度环（默认增益集中定义在 car_config.h） */
PID_t pid_L = { .Kp = SPEED_KP_DEFAULT, .Ki = SPEED_KI_DEFAULT, .Kd = 0.0f };
PID_t pid_R = { .Kp = SPEED_KP_DEFAULT, .Ki = SPEED_KI_DEFAULT, .Kd = 0.0f };

/* 方向环 */
DirPID_t dir_pid = { .Kp = 2.5f, .Ki = 0.0f, .target = 0.0f };

/* [BRAKE 2026-11-15] 死区补偿开关，默认开；刹车段由 special.c 关掉 */
static uint8_t motor_boost_en = 1U;

void Motor_SetBoost(uint8_t en)
{
    motor_boost_en = (en != 0U) ? 1U : 0U;
}

/* [GND 2026-11-14] 速度环增益切换：pivot 期间加强，出弯/停车恢复默认。 */
void Motor_SetSpeedGains(float kp, float ki)
{
    pid_L.Kp = kp;  pid_R.Kp = kp;
    pid_L.Ki = ki;  pid_R.Ki = ki;
    pid_L.integral = 0.0f;  pid_R.integral = 0.0f;
    pid_L.pre_error = 0.0f; pid_R.pre_error = 0.0f;
}

float PID_Calc(PID_t *p, float measure)
{
    float kp;

    p->measure   = measure;
    p->pre_error = p->error;
    p->error     = p->target - p->measure;
    p->integral += p->error;
    if (p->integral >  INTEG_LIMIT) p->integral =  INTEG_LIMIT;
    if (p->integral < -INTEG_LIMIT) p->integral = -INTEG_LIMIT;

    /* [PIVOT 2026-11-14] 低速增益调度改为按“幅值”判断，方向无关。
       原来用带符号的 measure/target 比较，target<0（反向 pivot）时永远不放大。 */
    kp = p->Kp;
    {
        float m = (p->measure >= 0.0f) ? p->measure : -p->measure;
        float t = (p->target  >= 0.0f) ? p->target  : -p->target;

        if (m < KP_LOW_SPEED && t > m)
        {
            float ratio = 1.0f - m / KP_LOW_SPEED;
            if (ratio > 1.0f) ratio = 1.0f;
            if (ratio < 0.0f) ratio = 0.0f;
            kp = p->Kp * (1.0f + (KP_BOOST - 1.0f) * ratio);
        }
    }

    p->output = kp * p->error
              + p->Ki * p->integral
              + p->Kd * (p->error - p->pre_error);

    /* 输出限幅 + 抗 windup */
    if (p->output >  PWM_MAX) { p->output =  PWM_MAX; if (p->error > 0) p->integral -= p->error; }
    if (p->output < -PWM_MAX) { p->output = -PWM_MAX; if (p->error < 0) p->integral -= p->error; }
    return p->output;
}

float DirPID_Calc(DirPID_t *p, float measure)
{
    p->measure   = measure;
    p->pre_error = p->error;
    p->error     = p->target - p->measure;

    p->integral += p->error;
    if (p->integral >  INTEG_LIMIT) p->integral =  INTEG_LIMIT;
    if (p->integral < -INTEG_LIMIT) p->integral = -INTEG_LIMIT;

    p->output = p->Kp * p->error + p->Ki * p->integral;
    if (p->output >  TURN_MAX) p->output =  TURN_MAX;
    if (p->output < -TURN_MAX) p->output = -TURN_MAX;
    return p->output;
}

/* [PIVOT 2026-11-14] “还欠速度”判断：目标与实测之差是否与目标同向。
   原来四处都用 target > measure，反向（target<0）时恒为假，死区补偿不生效。 */
#define PID_WANTS_MORE(p)  ((((p)->target - (p)->measure) * (p)->target) > 0.0f)

void Motor_Output(float out_L, float out_R)
{
    if (out_L >  PWM_MAX) out_L =  PWM_MAX;
    if (out_L < -PWM_MAX) out_L = -PWM_MAX;
    if (out_R >  PWM_MAX) out_R =  PWM_MAX;
    if (out_R < -PWM_MAX) out_R = -PWM_MAX;

    /* 死区补偿：只在目标比实测快时抬到最小值。
       [REV 2026-11-14] 反转用单独的死区，正反转不对称。
       [GND 2026-11-14] 目标小于 MOTOR_MIN_TARGET 时不补偿——
       否则刹车段（目标 0.4~1.2）会被抬到 150，车停不住还一直抖。 */
    if (motor_boost_en && pid_L.target >  MOTOR_MIN_TARGET && out_L > 0.0f && out_L < MOTOR_MIN_DUTY_L && PID_WANTS_MORE(&pid_L)) out_L = MOTOR_MIN_DUTY_L;
    if (motor_boost_en && pid_L.target < -MOTOR_MIN_TARGET && out_L < 0.0f && out_L > -MOTOR_MIN_DUTY_L_REV && PID_WANTS_MORE(&pid_L)) out_L = -MOTOR_MIN_DUTY_L_REV;
    if (motor_boost_en && pid_R.target >  MOTOR_MIN_TARGET && out_R > 0.0f && out_R < MOTOR_MIN_DUTY_R && PID_WANTS_MORE(&pid_R)) out_R = MOTOR_MIN_DUTY_R;
    if (motor_boost_en && pid_R.target < -MOTOR_MIN_TARGET && out_R < 0.0f && out_R > -MOTOR_MIN_DUTY_R_REV && PID_WANTS_MORE(&pid_R)) out_R = -MOTOR_MIN_DUTY_R_REV;

    /* [SAFE 2026-11-14] 输出不允许与目标反向。
       PID 过冲时只降到 0（滑行），绝不做“反接制动”——
       否则 DIR 会在零点附近每 10 ms 翻一次，H 桥啸叫、电流成倍，
       反复几次就会把驱动打穿（现场出现过的“突然失控 + 巨大轰鸣”）。 */
    if (pid_L.target >= 0.0f && out_L < 0.0f) out_L = 0.0f;
    if (pid_L.target <= 0.0f && out_L > 0.0f) out_L = 0.0f;
    if (pid_R.target >= 0.0f && out_R < 0.0f) out_R = 0.0f;
    if (pid_R.target <= 0.0f && out_R > 0.0f) out_R = 0.0f;
    if (out_L >= 0) {
        TIM1->CCR1 = (uint32_t)out_L;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else {
        TIM1->CCR1 = (uint32_t)(-out_L);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }

    if (out_R >= 0) {
        TIM1->CCR2 = (uint32_t)out_R;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        TIM1->CCR2 = (uint32_t)(-out_R);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
}
