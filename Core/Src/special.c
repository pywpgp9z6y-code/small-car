#include "special.h"
#include "car_config.h"
#include "motor.h"      /* [PIVOT 2026-11-14] 出弯清速度环积分，需要 pid_L/pid_R */
#include <stdio.h>      /* [CORN 2026-11-15] 锁存快照打印 */

/* [HEX-ZONE 2026-11-15] 1 = 当前处于六边形区间（最近一次 V 之后还没走远），
   期间禁用直角弯检测：六边形里连续的 60° 折角会被误判成直角弯。
   Hex_Update 每拍刷新（它在 Corner_Update 之前调用）。 */
static uint8_t hex_zone_flag = 0U;

/* [CORN 2026-11-15] 锁存瞬间的快照：确认"线在哪一侧"和"转哪个方向"是否一致。
   格式：CORN,<dir>,<mask>,<cnt>,<error×10>
   mask 的 bit i = 通道 i。若把车放在线左侧时 mask 高位亮，说明通道顺序是镜像的。 */
static void Corner_Log(uint8_t why, float dir, uint16_t mask, uint16_t cw_mask,
                       int cnt, float error)
{
    /* [诊断 2026-11-15] 第 4 个字段是"进入 WIDE 那一拍的 mask"：
       区分"进弯段（无缺口）"和"出弯段（有缺口）"的关键证据。 */
    printf("CORN,%d,%d,%d,%d,%d,%d\r\n",
           (int)why, (int)dir, (int)mask, (int)cw_mask, cnt, (int)(error * 10.0f));
}

void Corner_Reset(CornerState *c)
{
    Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);   /* [GND 2026-11-14] */
    Motor_SetBoost(1U);                                        /* [BRAKE 2026-11-15] 停车恢复死区补偿 */
    c->state = 0U;
    c->dir = 1.0f;
    c->yaw = 0.0f;
    c->ticks = 0U;
    c->cooldown = 0U;
    c->cooldown_dist = 0U;  /* [COOLDOWN 2026-11-18] 距离制冷却 */
    c->ramp_ticks = 0U;
    c->yaw_ref = 0.0f;      /* [PIVOT 2026-11-14] */
    c->pivot_ticks = 0U;    /* [PIVOT 2026-11-14] */
    c->dist = 0U;           /* [APPROACH 2026-11-14] */
    c->speed_ref = 0.0f;    /* [APPROACH 2026-11-14] */
    c->phase_ticks = 0U;    /* [APPROACH 2026-11-14] */
    c->stall_ticks = 0U;    /* [SAFE 2026-11-14] */
    c->prev_error = 0.0f;   /* [GND 2026-11-14] */
    c->bk_left = 0U;        /* [KICK 2026-11-14] */
    c->bk_wait = 0U;
    c->bk_tries = 0U;
    c->cw_state = 0U;       /* [CW 2026-11-15] */
    c->cw_dark = 0U;
    c->cw_low = 0U;
    c->cw_side = 0.0f;
    c->cw_ticks = 0U;
    c->cw_mask = 0U;        /* [诊断 2026-11-15] */
    c->armed = 1U;          /* [ARM 2026-11-15] 上电/停车后允许锁存 */
    c->arm_cnt = 0U;
    c->settle_ticks = 0U;   /* [SETTLE 2026-11-15] */
}

uint8_t Corner_IsActive(const CornerState *c)
{
    return (c->state != 0U) ? 1U : 0U;
}

/* [COOLDOWN 2026-11-18] 出弯冷却改为距离制：走过 CORNER_COOLDOWN_CM 才解禁。
   时间制（0.4 s）在 BASE 120 下等于 11 cm，比锯齿段的角间距还长。 */
uint8_t Corner_IsCooling(const CornerState *c)
{
    return (c->dist < c->cooldown_dist) ? 1U : 0U;
}

void Corner_AddDistance(CornerState *c, uint32_t counts)
{
    c->dist += counts;
}

/* ================= 直角弯检测（[CW 2026-11-15]） =================
   指纹：一侧连续全亮（压到弯口）→ 紧接连续全黑（线甩出阵列）。
   缓弯/直道不会出现“宽线之后全黑”，所以这个判据比 |error|>3 更适合直角弯，
   而且“哪一侧宽”直接给出转向方向，不依赖已经失真的质心。 */

/* 单侧宽线判定：左侧 ch0~ch5 亮 >= CW_SIDE_RUN 且明显多于右侧 -> +1（左转）
                 右侧 ch6~ch11 同理 -> -1（右转）
                 其他 -> 0 */
static uint8_t Corner_SideOfMask(uint16_t mask, float *dir)
{
    uint8_t i;
    uint8_t l = 0U, r = 0U;

    for (i = 0U; i < 6U; ++i)
        if ((mask & (uint16_t)(1U << i)) != 0U) ++l;
    for (i = 6U; i < 12U; ++i)
        if ((mask & (uint16_t)(1U << i)) != 0U) ++r;

    if ((l >= CW_SIDE_RUN) && (l > (uint8_t)(r + 1U))) { *dir =  1.0f; return 1U; }
    if ((r >= CW_SIDE_RUN) && (r > (uint8_t)(l + 1U))) { *dir = -1.0f; return 1U; }
    return 0U;
}

/* 进入过弯四段（通用锁存与直角弯检测共用，保证两处初始化一致） */
static void Corner_Enter(CornerState *c, float dir)
{
#if (CORNER_MODE == 1)
    /* [ARC 2026-11-18] 弧线模式：不停车，直接进偏航双环 */
    c->state = CORNER_ST_ARC;
#else
    c->state = CORNER_ST_APPROACH;
#endif
    c->dir   = dir;

    Motor_SetBoost(1U);     /* [BRAKE 2026-11-15] 进弯先恢复死区补偿，刹车段再关 */

    c->yaw = 0.0f;
    c->ticks = 0U;
    c->ramp_ticks = 0U;

    c->yaw_ref = CORNER_TURN_DEG;
    c->pivot_ticks = 0U;

    c->dist = 0U;
    c->speed_ref = BASE_SPEED;
    c->phase_ticks = 0U;
    c->bk_left = 0U;
    c->bk_wait = 0U;
    c->bk_tries = 0U;

    c->cw_state = 0U;
    c->armed    = 0U;       /* [ARM 2026-11-15] 出弯后先封闭锁存，等误差稳定再开 */
    c->arm_cnt  = 0U;
    c->settle_ticks = 0U;   /* [SETTLE 2026-11-15] */
    c->recover_dir = 0.0f;  /* [RECOVER 2026-11-16] */
    c->recover_ticks = 0U;
    c->arc_ticks = 0U;      /* [ARC 2026-11-18] */
    c->arc_line_ticks = 0U;
}

/* 直角弯状态机：巡线时每拍调用（包括 cnt==0 的拍） */
static void Corner_Watch(CornerState *c, uint16_t mask, int cnt, float error,
                         float *dir_out, uint8_t *fire, uint8_t *gave_up)
{
    *fire = 0U;
    *gave_up = 0U;

    if (c->cw_state == 0U)              /* IDLE：找单侧宽线 */
    {
        float d = 0.0f;
        if (Corner_SideOfMask(mask, &d) != 0U)
        {
            c->cw_state = 1U;           /* WIDE：已看到弯口，等全黑 */
            c->cw_side  = d;
            c->cw_mask  = mask;         /* [诊断] 定方向用的那一拍掩码 */
            c->cw_ticks = 0U;
            c->cw_wide  = 0U;           /* [ARC] 宽线总拍数 */
            c->cw_dark  = 0U;
            c->cw_low   = 0U;
        }
        return;
    }

    ++c->cw_ticks;
    ++c->cw_wide;

    /* [FIX 2026-11-16] WIDE 期间**持续更新方向**，用"最后一帧"而不是"第一帧"。
       实测 直角错误1：进入 WIDE 的第一帧是 ch7~10（线刚偏到右边，是车的横向偏置），
       判成右转；而真正压到边缘的那一帧是 ch0~7（l=6 r=2），明确指向左。
       现在每拍只要还能定出方向就覆盖，最终采用失线前最后一次有效的判断。
       [FIX 2026-11-16b] 同时把 cw_ticks 清零：80 拍超时改为"多久没见到弯口特征"，
       否则"线从 ch2~5 漂到贴边再到消失"要 0.8 s 以上，超时会先于全黑触发。 */
    {
        float d2 = 0.0f;
        if (Corner_SideOfMask(mask, &d2) != 0U)
        {
            c->cw_side = d2;
            c->cw_mask = mask;
            c->cw_ticks = 0U;
        }
    }

    /* [FIX 2026-11-16b] "线变窄"必须是"又细又居中"才算恢复正常；
       真弯口的窄线是贴边的（bias 远离 5.5），不能当成"线恢复了"。 */
    {
        uint8_t i, n = 0U;
        float   s = 0.0f;
        uint8_t centered = 0U;

        for (i = 0U; i < 12U; ++i)
            if (((mask >> i) & 1U) != 0U) { s += (float)i; ++n; }

        if (n > 0U)
        {
            float bias = s / (float)n;
            if ((bias > 4.0f) && (bias < 7.0f)) centered = 1U;
        }

        if (cnt == 0)
        {
            if (c->cw_dark < 250U) ++c->cw_dark;
            c->cw_low = 0U;
        }
        else
        {
            c->cw_dark = 0U;
            if ((cnt <= 2) && (centered != 0U))
            {
                if (c->cw_low < 250U) ++c->cw_low;
            }
            else
            {
                c->cw_low = 0U;
            }
        }
    }

#if (CORNER_MODE == 1)
    /* [ARC 2026-11-18] 弧线模式：宽线 + 误差已接近极限 才触发。
       只用"宽线"会把缓弯、十字路口（mask=4095）全当成直角弯。 */
    if ((c->cw_wide >= ARC_WIDE_TICKS) &&
        ((error > ARC_TRIG_ERR) || (error < -ARC_TRIG_ERR)))
    {
        *dir_out = c->cw_side;
        *fire = 1U;
        c->cw_state = 0U;
        return;
    }
#endif

    if (c->cw_dark >= CW_DARK_TICKS)    /* 宽线之后全黑 -> 判定直角弯 */
    {
        *dir_out = c->cw_side;
        *fire = 1U;
        c->cw_state = 0U;
    }
    else if ((c->cw_low >= CW_LOW_TICKS) || (c->cw_ticks >= CW_WIDE_TIMEOUT))
    {
        /* [诊断 2026-11-16] 看到了宽线、但没等到"连续全黑" -> 不是直角弯（或没检测到）
           只在宽线持续够久时上报，避免一闪而过的宽块刷屏。 */
        if (c->cw_ticks >= 5U) *gave_up = 1U;
        c->cw_state = 0U;               /* 线恢复正常 / 宽线太久 -> 不是直角弯 */
    }
}

float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint16_t mask,
                    uint8_t gyro_ok,
                    float gyro_z,
                    float speed_meas,
                    float *speed_cmd)
{
    /* ---- 段 1：冲弯（[APPROACH 2026-11-14] 新增）
       直行到“总位移 - 刹车区”，只保航向、忽略线 PID。
       一旦让方向环参与，车会在 7 cm 内顺着弯口拐起来，pivot 起点就偏了。 */
    if (c->state == CORNER_ST_APPROACH)
    {
        float turn = 0.0f;

        if (gyro_ok)
        {
            c->yaw += gyro_z * 0.01f;      /* 100 Hz，dt = 10 ms */
            turn = -CORNER_APPROACH_KYAW * c->yaw - CORNER_APPROACH_KD * gyro_z;
            if (turn >  CORNER_APPROACH_TURNMAX) turn =  CORNER_APPROACH_TURNMAX;
            if (turn < -CORNER_APPROACH_TURNMAX) turn = -CORNER_APPROACH_TURNMAX;
        }
        ++c->ticks;

        /* [GND 2026-11-14] 从巡航速度平滑降到冲弯速度：
           一刀切会让前馈瞬间跌破死区，车猛地一顿再爬起来（现场看到的“卡顿”）。 */
        if (c->speed_ref > CORNER_APPROACH_SPEED)
        {
            c->speed_ref -= CORNER_APPROACH_RAMP_DEC;
            if (c->speed_ref < CORNER_APPROACH_SPEED)
                c->speed_ref = CORNER_APPROACH_SPEED;
        }
        else
        {
            c->speed_ref = CORNER_APPROACH_SPEED;
        }
        *speed_cmd = c->speed_ref;

        /* [KICK 2026-11-14] 破静摩擦：只改本状态的速度给定，
           不碰 launch kick、不碰 Motor_Output、不改其他状态。
           连续 6 拍没动 -> 目标临时抬到 90 给 6 拍（FF≈288 + P，足够破静摩擦）。 */
        {
            float sp_abs = (speed_meas >= 0.0f) ? speed_meas : -speed_meas;

            if (c->bk_left > 0U)
            {
                --c->bk_left;
                *speed_cmd = CORNER_BREAKAWAY_SPEED;
            }
            else if (sp_abs > 10.0f)
            {
                c->bk_wait = 0U;          /* 正常在跑，清计数 */
                c->bk_tries = 0U;
            }
            else if (sp_abs < CORNER_BREAKAWAY_STALL)
            {
                if (++c->bk_wait >= CORNER_BREAKAWAY_WAIT)
                {
                    c->bk_wait = 0U;
                    if (c->bk_tries < CORNER_BREAKAWAY_TRIES)
                    {
                        ++c->bk_tries;
                        c->bk_left = CORNER_BREAKAWAY_TICKS;
                    }
                }
            }
            else
            {
                c->bk_wait = 0U;          /* 3~10 之间：还在动，不踹 */
            }
        }

        if (c->dist + CORNER_BRAKE_COUNTS >= CORNER_APPROACH_COUNTS)
        {
            c->state       = CORNER_ST_BRAKE;      /* 进刹车区了 */
            c->phase_ticks = 0U;
        }
        else if (c->ticks >= CORNER_APPROACH_MAX_TICKS)
        {
            /* [SAFE 2026-11-14] 冲弯超时：编码器/堵转，或者根本没走到弯口。
               走过一半就刹车，否则直接放弃这个弯——别拿错误位置去 pivot。 */
            if (c->dist >= (CORNER_APPROACH_COUNTS / 2U))
            {
                c->state       = CORNER_ST_BRAKE;
                c->phase_ticks = 0U;
            }
            else
            {
                c->state      = CORNER_ST_LINE;
            c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
                c->ramp_ticks = CORNER_RAMP_TICKS;
            }
        }
        return turn;
    }

    /* ---- 段 2：刹停（[APPROACH 2026-11-14] 新增）
       按“剩余距离”给速度，越靠近停点越慢，停点自动落在 CORNER_APPROACH_COUNTS 上。
       判停用编码器实测速度，不用计时。 */
    if (c->state == CORNER_ST_BRAKE)
    {
        float    turn = 0.0f;
        float    sp_abs;
        float    denom;
        uint32_t remain;

        /* [BRAKE 2026-11-15] 刹车段关掉死区补偿：
           补偿会把低目标的输出硬抬到 150，和"减速到 0"正好相反，
           表现为一轮一轮地"通电-断电"，看起来一顿一顿。 */
        Motor_SetBoost(0U);

        if (gyro_ok)
        {
            c->yaw += gyro_z * 0.01f;
            turn = -CORNER_APPROACH_KYAW * c->yaw - CORNER_APPROACH_KD * gyro_z;
            if (turn >  CORNER_APPROACH_TURNMAX) turn =  CORNER_APPROACH_TURNMAX;
            if (turn < -CORNER_APPROACH_TURNMAX) turn = -CORNER_APPROACH_TURNMAX;
        }
        ++c->phase_ticks;

        remain = (c->dist < CORNER_APPROACH_COUNTS)
               ? (CORNER_APPROACH_COUNTS - c->dist) : 0U;

        denom = (float)CORNER_BRAKE_COUNTS;
        if (denom < 1.0f) denom = 1.0f;

        /* [SAFE 2026-11-14] 关键护栏：
           比例减速只在“刹车区”内成立。冲弯超时进来时 remain 可能有 3000+，
           直接按比例算会给出 900 的速度给定 -> 占空比打满 -> 电机猛冲。
           这里把给定钳死在冲弯速度以内。 */
        if (remain >= CORNER_BRAKE_COUNTS)
            c->speed_ref = CORNER_APPROACH_SPEED;
        else
            c->speed_ref = CORNER_APPROACH_SPEED * (float)remain / denom;

        if (c->speed_ref < 0.0f) c->speed_ref = 0.0f;
        *speed_cmd = c->speed_ref;

        sp_abs = (speed_meas >= 0.0f) ? speed_meas : -speed_meas;

        /* [STOP 2026-11-15] 到位判据改成“距离+速度”双条件：
           比例减速是渐近曲线，remain 永远走不到 0，原来只能靠 600 ms 超时。
           现在剩余 <= 1.1 mm 且速度已低就算停稳。 */
        if ((remain <= CORNER_STOP_ZONE) && (sp_abs < CORNER_BRAKE_STOP))
        {
            Motor_SetSpeedGains(PIVOT_KP, PIVOT_KI);   /* [BAL 2026-11-14] 进 pivot 加大速度环增益 */
            c->state       = CORNER_ST_PIVOT;
            c->phase_ticks = 0U;
            c->pivot_ticks = 0U;
            c->stall_ticks = 0U;   /* [SAFE 2026-11-14] */
            c->ticks       = 0U;
            c->yaw         = 0.0f;         /* pivot 的角度从这里重新算 */
        }
        else if (c->phase_ticks >= CORNER_BRAKE_MAX)
        {
            /* 兜底：编码器不说话也照样转，否则会一直停在原地 */
            Motor_SetSpeedGains(PIVOT_KP, PIVOT_KI);   /* [BAL 2026-11-14] */
            c->state       = CORNER_ST_PIVOT;
            c->phase_ticks = 0U;
            c->pivot_ticks = 0U;
            c->stall_ticks = 0U;   /* [SAFE 2026-11-14] */
            c->ticks       = 0U;
            c->yaw         = 0.0f;
        }
        return turn;
    }

    /* ---- 段 3：原地 pivot（[PIVOT 2026-11-14]）
       speed_cmd 清零，两轮等幅反向 => 绕两轮中点自转。
       退出用陀螺转到目标角。 */
    if (c->state == CORNER_ST_PIVOT)
    {
        float   yaw_rel;
        float   mag;
        uint8_t done = 0U;

        Motor_SetBoost(1U);                /* [BRAKE 2026-11-15] pivot 恢复死区补偿 */

        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;      /* 100 Hz，dt = 10 ms */
        ++c->ticks;

        yaw_rel = c->dir * c->yaw;         /* 沿转弯方向为正 */

        /* 速度清零：两轮只做反向差速，绕两轮中点转 */
        *speed_cmd = 0.0f;
        ++c->pivot_ticks;

        /* 前 15 拍给起转脉冲（静摩擦 390/428），之后稳态 32 */
        mag = (c->pivot_ticks <= PIVOT_KICK_TICKS) ? PIVOT_KICK : PIVOT_TARGET;

        /* [SAFE 2026-11-14] 堵转保护：两轮都不动（被墙/被卡）就别再灌电流，
           否则大占空比长时间堵转会烧驱动。 */
        {
            float sp = (speed_meas >= 0.0f) ? speed_meas : -speed_meas;
            if (sp < PIVOT_STALL_SPEED) ++c->stall_ticks;
            else                        c->stall_ticks = 0U;
        }

        /* 退出：陀螺转够角度，或超时，或堵转 */
        if (gyro_ok && yaw_rel >= (c->yaw_ref - CORNER_ANGLE_TOL))
            done = 1U;
        if (c->ticks >= CORNER_MAX_TICKS)
            done = 1U;
        if (c->stall_ticks >= PIVOT_STALL_TICKS)
            done = 1U;
        /* 陀螺失效时没有角度反馈，用“线已回到中心”兜底，避免一直转到超时 */
        if (!gyro_ok && cnt > 0 && c->ticks >= 20U)
        {
            float ae = (error >= 0.0f) ? error : -error;
            if (ae < 1.5f) done = 1U;
        }

        if (done)
        {
            /* [SETTLE 2026-11-15] 粗转结束不直接回巡线，先进收尾段把剩余几度补掉。
               原来这里直接回 LINE -> 靠出弯后的方向环补，连续弯时补不赢、误差累积。 */
            c->state = CORNER_ST_SETTLE;
            c->settle_ticks = 0U;
            return 0.0f;                   /* 这一拍先断电，下一拍开始收尾 */
        }//重置

        return c->dir * mag;               /* 满舵差速，返回给方向环 */
    }

    /* ---- 段 4：航向收尾（[SETTLE 2026-11-15] 新增）
       粗转断电后车还会滑转几度，滑多少取决于当时的角速度——固定 4° 补偿只在
       某一个转速下刚好。这一段用低速小差速把剩余角度补掉，让每个弯都落在
       yaw_ref ± TOL，连续弯不累积。 */
    if (c->state == CORNER_ST_SETTLE)
    {
        float   yaw_r;
        float   rate;
        float   pred;
        float   mag;
        uint8_t done = 0U;

        Motor_SetBoost(1U);                /* [BRAKE 2026-11-15] 收尾段恢复死区补偿 */

        ++c->settle_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        yaw_r = c->dir * c->yaw;               /* 沿转弯方向的转角，° */
        rate  = c->dir * gyro_z;               /* 沿转弯方向的角速度，°/s */
        pred  = yaw_r + rate * CORNER_COAST_T; /* 预测 50 ms 后的角度（滑行补偿） */

        *speed_cmd = 0.0f;                     /* 收尾段原地转，不前进 */

        if (c->settle_ticks >= CORNER_SETTLE_MAX_TICKS)
            done = 1U;                         /* 超时兜底 */
        else if ((yaw_r >= (c->yaw_ref - CORNER_SETTLE_TOL)) &&
                 (rate <  CORNER_SETTLE_RATE) &&
                 (rate > -CORNER_SETTLE_RATE))
            done = 1U;                         /* 角度到位且已经停住 */

        if (done)
        {
            /* [RECOVER 2026-11-16] 转完 90° 还看不到线 -> 反方向找 30°
               （方向判错 / 冲过头 时的兜底；找到就交回巡线，找不到才放弃） */
            if (cnt == 0)
            {
                c->state         = CORNER_ST_RECOVER;
                c->yaw           = 0.0f;
                c->recover_dir   = -c->dir;         /* 反向 */
                c->recover_ticks = 0U;
                return 0.0f;
            }

            c->state = CORNER_ST_LINE;
            c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
            c->ramp_ticks = CORNER_RAMP_TICKS;

            /* 收尾结束才恢复默认速度环增益（收尾段仍用 pivot 的大增益） */
            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            dir_pid.integral = 0.0f;
            return 0.0f;
        }

        /* [JUDDER 2026-11-15] 收尾段三条规则：
           1) 还在快速旋转 -> 先断电滑行（通电修正会变成反接制动，电流数倍、一顿一顿）；
           2) 慢下来且还差角度 -> 用 ±30 轻推；
           3) 过头 -> 不反向补，交给方向环（反向补会在 10 ms 内翻符号，机械上就是抖）。 */
        if ((rate > CORNER_SETTLE_COAST_RATE) || (rate < -CORNER_SETTLE_COAST_RATE))
            mag = 0.0f;                        /* 还在转：滑行等它停 */
        else if (pred < (c->yaw_ref - CORNER_SETTLE_TOL))
            mag = CORNER_SETTLE_MAG;           /* 停住了但还差，轻推一段 */
        else
            mag = 0.0f;                        /* 到位或已过头：断电 */

        return c->dir * mag;
    }

    /* ---- 段 6：带速度弧线（[ARC 2026-11-18] CORNER_MODE=1）
       不停车，用陀螺双环把车头转过 ARC_YAW_DEG：
         外环：ω_des = K_YAW·(目标角 − 当前角)，限幅 ±200°/s
         内环：turn  = ω_des/1.644 + K_OMEGA·(ω_des − ω)
       全程 speed = BASE，turn ≤ speed（内轮最低 0，不反转）。 */
    if (c->state == CORNER_ST_ARC)
    {
        float yaw_r;
        float rate_r;
        float err_deg;
        float omega_des;
        float turn_r;

        ++c->arc_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        yaw_r  = c->dir * c->yaw;
        rate_r = c->dir * gyro_z;

        *speed_cmd = ARC_CREEP_SPEED;        /* [PIVOT-C] 慢速 creep：旋转中心回到轴心附近 */

        err_deg   = ARC_YAW_DEG - yaw_r;
        omega_des = ARC_K_YAW * err_deg;
        if (omega_des >  ARC_OMEGA_MAX) omega_des =  ARC_OMEGA_MAX;
        if (omega_des < -ARC_OMEGA_MAX) omega_des = -ARC_OMEGA_MAX;

        turn_r = omega_des / OMEGA_PER_TURN + ARC_K_OMEGA * (omega_des - rate_r);

        if (turn_r < 0.0f) turn_r = 0.0f;
        if (turn_r > ARC_TURN_MAX) turn_r = ARC_TURN_MAX;
        /* [PIVOT-C] 这里不再钳 turn ≤ speed——
           内轮要反转（25−120=−95），旋转中心才能回到轴心附近。 */

        /* 出弯：转够且角速度已经收住 */
        if ((err_deg < ARC_YAW_TOL) &&
            (rate_r < 20.0f) && (rate_r > -20.0f))
        {
            c->state = CORNER_ST_LINE;
            c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
            c->ramp_ticks = CORNER_RAMP_TICKS;
            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        /* 线提前回来（细且居中）也可以交回 */
        if ((cnt > 0) && (error > -2.0f) && (error < 2.0f))
        {
            if (c->arc_line_ticks < 250U) ++c->arc_line_ticks;
            if (c->arc_line_ticks >= 5U)
            {
                c->state = CORNER_ST_LINE;
                c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
                c->ramp_ticks = CORNER_RAMP_TICKS;
                Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
                return 0.0f;
            }
        }
        else
        {
            c->arc_line_ticks = 0U;
        }

        if (c->arc_ticks >= ARC_MAX_TICKS)               /* 兜底 */
        {
            c->state = CORNER_ST_LINE;
            c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
            c->ramp_ticks = CORNER_RAMP_TICKS;
            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        return c->dir * turn_r;
    }

    /* ---- 段 5：反向找线（[RECOVER 2026-11-16] 新增）
       转满 90° 之后如果还是看不到线（方向判错、或冲过头），
       反方向原地转最多 CORNER_RECOVER_DEG，转的过程中一看到线就交回巡线；
       转满还没有线才放弃，交给丢线保护去停车。 */
    if (c->state == CORNER_ST_RECOVER)
    {
        float   turn = 0.0f;
        float   ya;

        ++c->recover_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        *speed_cmd = 0.0f;                     /* 原地转，不前进 */

        if (cnt > 0)                           /* 找到了：交回巡线 */
        {
            c->state      = CORNER_ST_LINE;
        c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
            c->ramp_ticks = CORNER_RAMP_TICKS;

            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            dir_pid.integral = 0.0f;
            return 0.0f;
        }

        ya = (c->yaw >= 0.0f) ? c->yaw : -c->yaw;
        if ((ya >= CORNER_RECOVER_DEG) || (c->recover_ticks >= CORNER_RECOVER_TICKS))
        {
            /* 放弃：回 LINE，后面由 LOST_STOP_TICKS 触发停车 */
            c->state         = CORNER_ST_LINE;
            c->cooldown_dist = c->dist;      /* [COOLDOWN] 不设冷却 */
            c->ramp_ticks    = 0U;

            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        turn = c->recover_dir * PIVOT_TARGET;
        return turn;
    }

    /* ---- 正常巡线 ---- */
    /* [COOLDOWN 2026-11-18] 冷却改成距离制，这里不再递减计数 */

    /* [ARM 2026-11-15] 重新武装：误差连续 CORNER_ARM_TICKS 拍低于锁存阈值，
       才允许下一次锁存。没有这一条，一次没转准就会“再锁存再转”，
       现场表现为转 60°→停→再转 60°→转过头丢线。 */
    if ((cnt > 0) && (error > -CORNER_ARM_ERR) && (error < CORNER_ARM_ERR))
    {
        if (c->arm_cnt < 60000U) ++c->arm_cnt;
        if (c->arm_cnt >= CORNER_ARM_TICKS) c->armed = 1U;
    }
    else
    {
        c->arm_cnt = 0U;
    }

    /* [CW 2026-11-15] 直角弯专职检测：每拍都跑，丢线那一拍也必须跑，
       所以放在 cnt==0 提前返回之前。
       [HEX-ZONE] 处于六边形区间时不检测：六边形里连续折角会被误判成直角弯。 */
    if (hex_zone_flag == 0U)
    {
        float   cw_dir = 0.0f;
        uint8_t cw_fire = 0U;
        uint8_t cw_gave_up = 0U;

        Corner_Watch(c, mask, cnt, error, &cw_dir, &cw_fire, &cw_gave_up);

        if ((cw_fire != 0U) && (Corner_IsCooling(c) == 0U) && (c->armed != 0U))
        {
            Corner_Log(1U, cw_dir, mask, c->cw_mask, cnt, error);   /* [CORN] 1 = 直角弯检测触发 */
            Corner_Enter(c, cw_dir);
            *speed_cmd = c->speed_ref;
            return 0.0f;
        }
        else if (cw_gave_up != 0U)
        {
            /* [诊断] 看到了宽线但没等到全黑：说明"线没消失"或"没形成弯口" */
            Corner_Log(3U, c->cw_side, mask, c->cw_mask, cnt, error);
        }
    }

    if (cnt == 0)
    {
        *speed_cmd = BASE_SPEED;
        return pid_turn;//CornerUpdate返回pid_turn,方向环
    }

    /* 大误差/大 turn 进入过弯锁存态 */
    {
        float abs_err  = (error >= 0.0f) ? error : -error;
        float abs_turn = (pid_turn >= 0.0f) ? pid_turn : -pid_turn;

        c->prev_error = error;

        /* [ARM 2026-11-15] 去掉 derr 判据：
           车一旦因为 error 大而打舵，error 自己就会快速变化，derr 必然达标，
           于是“缓弯打一次方向”也会被判成直角弯。现在只认“宽线”（cnt>=4）——
           缓弯的线始终是 2~3 路，直角弯口才会压到 4 路以上。 */
        if ((abs_err > CORNER_ENTER_ERR || abs_turn > CORNER_ENTER_TURN)
            && ((uint32_t)cnt >= CORNER_ENTER_CNT)
            && (c->armed != 0U)
            && (hex_zone_flag == 0U)        /* [HEX-ZONE] 六边形区间内不锁存直角弯 */
            && (Corner_IsCooling(c) == 0U))
        {
            float dir = (pid_turn >= 0.0f) ? 1.0f : -1.0f;
            if (abs_turn < 0.5f)
                dir = (error > 0.0f) ? -1.0f : 1.0f;

            Corner_Log(2U, dir, mask, 0U, cnt, error);      /* [CORN] 2 = 通用锁存触发 */
            Corner_Enter(c, dir);          /* [CW 2026-11-15] 通用锁存与直角弯共用初始化 */
            *speed_cmd = c->speed_ref;
            return 0.0f;                   /* 第一拍直行，不打舵 */
        }
    }

    /* 正常弯道降速 + 出弯速度斜坡 */
    {
        float abs_turn = (pid_turn >= 0.0f) ? pid_turn : -pid_turn;

        *speed_cmd = BASE_SPEED - CURVE_SLOW * abs_turn;//速度基准是速度常量-减速比*拐弯量

        if (*speed_cmd < MIN_CURVE_SPEED)//向下控速
            *speed_cmd = MIN_CURVE_SPEED;

        if (c->ramp_ticks > 0U)
        {
            --c->ramp_ticks;

            float t = (float)(CORNER_RAMP_TICKS - c->ramp_ticks) /
                      (float)CORNER_RAMP_TICKS;//t = 余下冷却时间比全项
            float ramp_speed = CORNER_EXIT_SPEED +
                               (BASE_SPEED - CORNER_EXIT_SPEED) * t;//[PIVOT 2026-11-14] 从 0 加速到 BASE

            if (*speed_cmd > ramp_speed)
                *speed_cmd = ramp_speed;
        }
    }

    return pid_turn;//返回方向环模式
}

void Hex_Reset(HexState *h)
{
    h->v_prev = 0U;
    h->cond_seen = 0U;
    h->arm_lost = 0U;
    h->zone = 0U;
    h->v_stack = 0U;         /* [DBLHEX 2026-11-18] */
    h->hex_yaw = 0.0f;
    h->half_done = 0U;
    h->full_done = 0U;
    h->zone_dist = 0U;
    h->v_active = 0U;
    h->v_yaw = 0.0f;
    hex_zone_flag = 0U;     /* [HEX-ZONE] */

    h->dir = -1.0f;
    h->event_cnt = 0U;
    h->arm_ticks = 0U;
    h->turn_ticks = 0U;
    h->dist = 0U;
    h->v_dist = 0U;
    h->evt_seq = 0U;

    h->dbg_event = 0U;
    h->dbg_dist = 0U;
    h->dbg_right = 0U;
    h->dbg_left = 0U;
    h->dbg_center = 0U;
    h->dbg_mask = 0U;
}

void Hex_AddDistance(HexState *h, uint32_t counts)
{
    h->dist += counts;
}

/* [TURNBIAS 2026-11-18] 是否在六边形区间内（区间内允许更大的转向能力） */
uint8_t Hex_ZoneActive(void)
{
    return hex_zone_flag;
}

HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask,
                     uint8_t gyro_ok,
                     float gyro_z)
{
    HexOutput out = { 0.0f, 0.0f, 0U };

    h->dbg_right = right;
    h->dbg_left = left;
    h->dbg_center = center;
    h->dbg_mask = mask;

    /* [HEX 2026-11-15] 三档开关：0=关闭（只更新上面的快照）；1=只记录；2=接管 */
    if (HEX_MODE == 0U)
        return out;

    /* [FIX 2026-11-16] 过弯状态机运行期间 / 出弯冷却期内，六边形逻辑完全不跑。
       原因：六边形触发是"武装后走够 10 cm"的纯距离计时器，而 dist 用的是
       (|dL|+|dR|)/2 —— pivot 时两轮反向，照样累加但不前进。
       一次 90° pivot 就灌进 6~7 cm 假里程，直角弯出口就会误触发一次原地转。 */
    if ((Corner_IsActive(corner) != 0U) || (Corner_IsCooling(corner) != 0U))
    {
        h->event_cnt = 0U;
        h->v_active  = 0U;
        h->v_prev    = 0U;
        h->arm_lost  = 0U;
        hex_zone_flag = h->zone;    /* [BLIND 2026-11-18] 提前返回也要刷新这个标志，
                                       否则过弯期间它会停在旧值 1，把断线斜穿永久挡掉 */
        return out;
    }

    /* [DBLHEX 2026-11-18] 累计转角（每拍跑，与 V 无关）：
       入栈时为 0；沿六边形方向累计；到 180° 归零并标记半程；
       第二段再累计 180°（总 360°）标记整程完成。 */
    if (h->v_stack != 0U)
    {
        float ya;

        if (gyro_ok)
            h->hex_yaw += h->dir * gyro_z * 0.01f;

        ya = (h->hex_yaw >= 0.0f) ? h->hex_yaw : -h->hex_yaw;

        if (h->half_done == 0U)
        {
            if (ya >= HEX_HALF_DEG)
            {
                h->half_done = 1U;
                h->hex_yaw   = 0.0f;    /* 归零重新累计 */
            }
        }
        else if (h->full_done == 0U)
        {
            if (ya >= HEX_FULL_DEG)
            {
                h->full_done = 1U;      /* 总 360° 已满，之后的 V 才允许出栈 */
            }
        }
    }

    /* ---- V 型判据（[HEX-V 2026-11-15]） --------------------------------
       a = ch0,1 / ch10,11      两端
       b = ch2,3,4 / ch7,8,9    两中段，**三路全亮才算 b 存在**；任意一路暗即 !b
       c = ch5 & ch6            正中
       一次 V = 某侧 a 亮 && c 亮 && 同侧 b 不完整
       两次 V（同侧、间隔 1~4 cm）= 六边形入口                              */
    {
        uint8_t a_l = (uint8_t)((mask & 0x0003U) != 0U);        /* ch0,ch1    */
        uint8_t a_r = (uint8_t)((mask & 0x0C00U) != 0U);        /* ch10,ch11  */
        uint8_t c_s = (uint8_t)(center != 0U);                  /* ch5 & ch6（严） */
        uint8_t c_l = (uint8_t)((mask & 0x00F0U) != 0U);        /* ch4~ch7 任一路（松） */

        uint8_t b_l_n = (uint8_t)(((mask >> 2) & 1U) + ((mask >> 3) & 1U) + ((mask >> 4) & 1U));
        uint8_t b_r_n = (uint8_t)(((mask >> 7) & 1U) + ((mask >> 8) & 1U) + ((mask >> 9) & 1U));

        /* [HEX-V 2026-11-15] 两次判据：
           · 武装（第一次 V）：端点亮 && 正中 ch5&ch6 && 同侧 b 有缺口（严格，防误武装）
           · 触发（第二次）：**正中 ch5&ch6 && 武装侧 b 任意一路为空**，且落在距离窗口内。
             不再要求端点亮——自跑时方向环会让质心偏 ±1 路，端点条件太容易失效。 */
        uint8_t v_l = (uint8_t)(a_l && c_s && (b_l_n <= HEX_V_B_MAX));
        uint8_t v_r = (uint8_t)(a_r && c_s && (b_r_n <= HEX_V_B_MAX));

        int8_t s = (v_l != v_r) ? (int8_t)(v_l ? 1 : -1) : (int8_t)0;

        /* [DBLHEX 2026-11-18] 用"累计转角"决定 V 的含义：
             栈空          + V -> 入栈（入口，10 cm 后转 30°）
             栈满 未满360°  + V -> 忽略（中途的 V 不是出口）
             栈满 已满360°  + V -> 出栈（出口，**不打舵**） */
        if ((h->event_cnt == 0U) && (h->v_active == 0U) &&
            (s != 0) && (h->v_prev == 0U))
        {
            if (h->v_stack == 0U)
            {
                /* ---- 入栈 ---- */
                h->v_stack   = 1U;
                h->hex_yaw   = 0.0f;        /* 陀螺归零，开始累计 */
                h->half_done = 0U;
                h->full_done = 0U;

                h->event_cnt = 1U;
                h->dir       = (float)s;
                h->v_dist    = h->dist;
                h->cond_seen = 0U;
                h->arm_lost  = 0U;
                h->zone      = 1U;
                h->zone_dist = h->dist;
                h->dbg_event = 4U;          /* 4 = 武装（入栈） */
                h->dbg_dist  = h->dist;
                ++h->evt_seq;
            }
            else if (h->full_done != 0U)
            {
                /* ---- 出栈：整程 360° 已满，出口的 V，不转 ---- */
                h->v_stack   = 0U;
                h->zone      = 0U;
                h->zone_dist = h->dist;
                h->dbg_event = 9U;          /* 9 = 出口 V（出栈，不打舵） */
                h->dbg_dist  = h->dist;
                ++h->evt_seq;
            }
            /* 其余情况：中途 V，忽略 */
        }
        /* 第二次：**纯距离触发**（回到最初的设计思路）。
           实测 2026-11-15（六边出错2）：9 cm 之前 c(ch5&ch6) 一直亮、之后必灭，
           所以 "c && b空" 在几何上自相矛盾——等距离够了，c 已经没了。
           而两次 V 的间距非常稳定（手推 10.3 cm、自跑 ~10.2 cm），
           直接用距离做判据最可靠。 */
        else if ((h->event_cnt == 1U) && (h->v_active == 0U))
        {
            uint32_t gap = h->dist - h->v_dist;

            /* [FIX 2026-11-16] 武装期间丢线 = 六边形假设不成立，取消武装。
               否则纯距离计时器会在"闭着眼睛"的情况下照样触发（直角-六边耦合1：
               行98 武装 → 行101 丢线 → 行105 计时到点原地转，全程 mask=0）。 */
            if (mask == 0U)
            {
                if (h->arm_lost < 250U) ++h->arm_lost;
                if (h->arm_lost >= 5U)
                {
                    h->event_cnt = 0U;
                    h->arm_lost  = 0U;
                }
            }
            else
            {
                h->arm_lost = 0U;

                if (gap >= HEX_V_GAP_MIN_COUNTS)    /* 走够距离 -> 原地转 */
                {
                    h->event_cnt  = 2U;
                    h->v_active   = 1U;
                    h->v_yaw      = 0.0f;           /* [原地转] 角度从这里开始累计 */
                    h->turn_ticks = HEX_V_MAX_TICKS;
                    /* [HEX-ZONE 2026-11-18] 触发时同样不续命（区间仍从第一次 V 算起） */
                    if (h->zone == 0U)
                    {
                        h->zone      = 1U;
                        h->zone_dist = h->dist;
                    }
                    h->dbg_event  = 5U;             /* 5 = 触发（开始原地转） */
                    h->dbg_dist   = gap;
                    ++h->evt_seq;
                }
            }
        }

        /* 武装期间换侧 -> 重新武装 */
        if ((h->event_cnt == 1U) && (h->v_active == 0U) &&
            (s != 0) && (h->v_prev == 0U) && (s != (int8_t)h->dir))
        {
            h->dir    = (float)s;
            h->v_dist = h->dist;
        }

        h->v_prev = (s != 0) ? 1U : 0U;

        /* 武装后等太久没等到第二次 -> 解除 */
        if ((h->event_cnt == 1U) &&
            ((h->dist - h->v_dist) > HEX_V_GAP_MAX_COUNTS))
        {
            /* [诊断] 武装超时：12→13 cm 内一次都没满足触发条件 */
            h->dbg_event = 8U;
            h->dbg_dist  = h->dist - h->v_dist;
            ++h->evt_seq;
            h->event_cnt = 0U;
        }

        /* 打舵中：线回到中段（ch4~ch7 有亮）且两端都不亮，就交回方向环。
           [实测 2026-11-15] 必须打够 HEX_V_MIN_TICKS 拍才允许判"摸到线"——
           触发那一拍线本来就在中段，否则 touch 立刻成立，打舵只持续 1 拍 = 白打。 */
        if (h->v_active != 0U)
        {
            uint8_t elapsed = (uint8_t)(HEX_V_MAX_TICKS - h->turn_ticks);
            uint8_t touch   = (uint8_t)(c_l && !a_l && !a_r &&
                                        (elapsed >= HEX_V_MIN_TICKS));
            uint8_t angle_ok = 0U;

            /* [原地转 2026-11-15] 陀螺累计转角，转到 HEX_V_DEG 就停 */
            if (gyro_ok)
            {
                h->v_yaw += gyro_z * 0.01f;
                if (h->v_yaw < 0.0f ? (-h->v_yaw >= (HEX_V_DEG - HEX_V_DEG_TOL))
                                    : ( h->v_yaw >= (HEX_V_DEG - HEX_V_DEG_TOL)))
                {
                    angle_ok = 1U;
                }
            }

            if (angle_ok || touch || (h->turn_ticks == 0U))
            {
                h->dbg_event = 6U;                  /* 6 = 原地转结束 */
                h->dbg_dist  = h->dist - h->v_dist;
                ++h->evt_seq;
                h->v_active  = 0U;
                h->event_cnt = 0U;
            }
            else
            {
                if (HEX_MODE == 2U)
                {
                    out.turn      = h->dir * HEX_V_PIVOT_TURN;  /* 原地转：不前进 */
                    out.speed_cmd = 0.0f;
                    out.have_control = 1U;
                }
                --h->turn_ticks;
            }
        }
    }

    /* [HEX-ZONE] 区间超时：最近一次 V 之后走了 HEX_ZONE_CM 还没再见到 V -> 离开六边形
       [DBLHEX 2026-11-18] 但"在栈内"时不按距离超时——双六边形跑一圈 360° 可能超过
       150 cm，距离超时会让区间在半路失效。出栈时才关区间。 */
    if ((h->zone != 0U) && (h->v_stack == 0U) &&
        ((h->dist - h->zone_dist) > HEX_ZONE_COUNTS))
    {
        h->zone = 0U;
    }

    hex_zone_flag = h->zone;

    return out;
}

void Hex_GetDebug(const HexState *h, HexDebug *d)
{
    d->evt_seq = h->evt_seq;
    d->event = h->dbg_event;
    d->event_dist = h->dbg_dist;
    d->right = h->dbg_right;
    d->left = h->dbg_left;
    d->center = h->dbg_center;
    d->mask = h->dbg_mask;
    d->event_cnt = h->event_cnt;
    d->off_r = h->event_cnt;                 /* 复用：0/1/2 */
    d->off_l = (uint16_t)h->v_active;        /* 复用：1=正在打舵 */
    d->dist = h->dist;
    d->arm = (uint16_t)h->v_prev;            /* 复用：上一拍是否 V */
}
