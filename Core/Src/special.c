#include "special.h"
#include "car_config.h"
#include "motor.h"      /* [PIVOT 2026-11-14] 出弯清速度环积分，需要 pid_L/pid_R */

void Corner_Reset(CornerState *c)
{
    Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);   /* [GND 2026-11-14] */
    Motor_SetBoost(1U);                                        /* [BRAKE 2026-11-15] 停车恢复死区补偿 */
    c->state = 0U;
    c->dir = 1.0f;
    c->yaw = 0.0f;
    c->ticks = 0U;
    c->cooldown = 0U;
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
    c->armed = 1U;          /* [ARM 2026-11-15] 上电/停车后允许锁存 */
    c->arm_cnt = 0U;
    c->settle_ticks = 0U;   /* [SETTLE 2026-11-15] */
}

uint8_t Corner_IsActive(const CornerState *c)
{
    return (c->state != 0U) ? 1U : 0U;
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
    c->state = CORNER_ST_APPROACH;
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
}

/* 直角弯状态机：巡线时每拍调用（包括 cnt==0 的拍） */
static void Corner_Watch(CornerState *c, uint16_t mask, int cnt,
                         float *dir_out, uint8_t *fire)
{
    *fire = 0U;

    if (c->cw_state == 0U)              /* IDLE：找单侧宽线 */
    {
        float d = 0.0f;
        if (Corner_SideOfMask(mask, &d) != 0U)
        {
            c->cw_state = 1U;           /* WIDE：已看到弯口，等全黑 */
            c->cw_side  = d;
            c->cw_ticks = 0U;
            c->cw_dark  = 0U;
            c->cw_low   = 0U;
        }
        return;
    }

    ++c->cw_ticks;

    if (cnt == 0)
    {
        if (c->cw_dark < 250U) ++c->cw_dark;
        c->cw_low = 0U;
    }
    else
    {
        c->cw_dark = 0U;
        if (cnt <= 2)
        {
            if (c->cw_low < 250U) ++c->cw_low;
        }
        else
        {
            c->cw_low = 0U;
        }
    }

    if (c->cw_dark >= CW_DARK_TICKS)    /* 宽线之后全黑 -> 判定直角弯 */
    {
        *dir_out = c->cw_side;
        *fire = 1U;
        c->cw_state = 0U;
    }
    else if ((c->cw_low >= CW_LOW_TICKS) || (c->cw_ticks >= CW_WIDE_TIMEOUT))
    {
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
                c->cooldown   = CORNER_COOLDOWN;
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
            c->state = CORNER_ST_LINE;
            c->cooldown = CORNER_COOLDOWN;
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

    /* ---- 正常巡线 ---- */
    if (c->cooldown > 0U)
        --c->cooldown;

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
       所以放在 cnt==0 提前返回之前。 */
    {
        float   cw_dir = 0.0f;
        uint8_t cw_fire = 0U;

        Corner_Watch(c, mask, cnt, &cw_dir, &cw_fire);

        if ((cw_fire != 0U) && (c->cooldown == 0U) && (c->armed != 0U))
        {
            Corner_Enter(c, cw_dir);
            *speed_cmd = c->speed_ref;
            return 0.0f;
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
            && c->cooldown == 0U)
        {
            float dir = (pid_turn >= 0.0f) ? 1.0f : -1.0f;
            if (abs_turn < 0.5f)
                dir = (error > 0.0f) ? -1.0f : 1.0f;

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
    h->right_prev = 0U;
    h->right_off_cnt = 0U;
    h->left_prev = 0U;
    h->left_off_cnt = 0U;

    h->dir = -1.0f;
    h->event_cnt = 0U;
    h->arm_ticks = 0U;
    h->preturn_ticks = 0U;
    h->turn_ticks = 0U;
    h->dist = 0U;
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

HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask)
{
    HexOutput out = { 0.0f, 0.0f, 0U };

    h->dbg_right = right;
    h->dbg_left = left;
    h->dbg_center = center;
    h->dbg_mask = mask;

    /* [HEX 2026-11-15] 六边形检测总开关（car_config.h 的 HEX_ENABLE）。
       置 0 时只更新上面的调试快照，不产生任何事件、也不覆盖舵量——
       因为直角弯同样会出现 ch11+center，会把过弯打断/误覆盖。
       调试快照仍然更新，所以 HEXSTAT 里的 mask/center/right/left 照样能看。 */
    if (!HEX_ENABLE)
        return out;

    if (h->event_cnt == 1U)
    {
        if (h->arm_ticks < 60000U) ++h->arm_ticks;
        if (h->arm_ticks > HEX_ARM_TIMEOUT) h->event_cnt = 0U;
    }

    /* 右侧 ch11 上升沿 */
    if (right)
    {
        if (!h->right_prev && h->right_off_cnt >= HEX_OFF_TICKS)
        {
            if (h->event_cnt == 0U)
            {
                if (center)
                {
                    h->event_cnt = 1U;
                    h->arm_ticks = 0U;
                    h->dist = 0U;
                    h->preturn_ticks = 0U;
                    h->turn_ticks = 0U;
                    h->dir = -1.0f;

                    h->dbg_event = 1U;
                    h->dbg_dist = h->dist;
                    ++h->evt_seq;
                }
            }
            else if (h->event_cnt == 1U &&
                     h->dir < 0.0f &&
                     h->dist >= HEX_CLEAR_COUNTS)
            {
                h->event_cnt = 2U;
                h->preturn_ticks = 0U;
                h->turn_ticks = HEX_TURN_TICKS;
                /* [SETTLE 2026-11-15] 不再清零 corner->state：
                   原来这里会把正在进行的 APPROACH/BRAKE/PIVOT/SETTLE
                   直接掐断，表现就是"线刚碰到一侧就交回方向环"。
                   hex 的覆盖已经由 control.c 的 !Corner_IsActive && cooldown==0 挡住。 */

                h->dbg_event = 2U;
                h->dbg_dist = h->dist;
                ++h->evt_seq;
            }
        }
        h->right_off_cnt = 0U;
    }
    else
    {
        if (h->right_off_cnt < 60000U) ++h->right_off_cnt;
    }
    h->right_prev = right;

    /* 左侧 ch0 上升沿 */
    if (left)
    {
        if (!h->left_prev && h->left_off_cnt >= HEX_OFF_TICKS)
        {
            if (h->event_cnt == 0U)
            {
                if (center)
                {
                    h->event_cnt = 1U;
                    h->arm_ticks = 0U;
                    h->dist = 0U;
                    h->preturn_ticks = 0U;
                    h->turn_ticks = 0U;
                    h->dir = 1.0f;

                    h->dbg_event = 1U;
                    h->dbg_dist = h->dist;
                    ++h->evt_seq;
                }
            }
            else if (h->event_cnt == 1U &&
                     h->dir > 0.0f &&
                     h->dist >= HEX_CLEAR_COUNTS)
            {
                h->event_cnt = 2U;
                h->preturn_ticks = 0U;
                h->turn_ticks = HEX_TURN_TICKS;
                /* [SETTLE 2026-11-15] 同上：不再清零 corner->state */

                h->dbg_event = 3U;
                h->dbg_dist = h->dist;
                ++h->evt_seq;
            }
        }
        h->left_off_cnt = 0U;
    }
    else
    {
        if (h->left_off_cnt < 60000U) ++h->left_off_cnt;
    }
    h->left_prev = left;

    /* 满舵覆盖，优先级高于通用过弯 */
    if (h->turn_ticks > 0U)
    {
        out.turn = h->dir * TURN_MAX;
        out.speed_cmd = CORNER_SPEED;
        out.have_control = 1U;

        --h->turn_ticks;
        if (h->turn_ticks == 0U) h->event_cnt = 0U;
    }
    else if (h->preturn_ticks > 0U)
    {
        out.turn = h->dir * HEX_PRE_TURN;
        out.speed_cmd = HEX_PRE_SPEED;
        out.have_control = 1U;
        --h->preturn_ticks;
    }

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
    d->off_r = h->right_off_cnt;
    d->off_l = h->left_off_cnt;
    d->dist = h->dist;
    d->arm = h->arm_ticks;
}
