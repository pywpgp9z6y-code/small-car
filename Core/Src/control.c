/**
 * @file    control.c
 * @brief   1 kHz 控制层：传感器 -> 策略 -> 左右目标速度 -> 电机输出
 *
 * 拆分前这些代码全部内联在 main.c 的 Control_Update() 里（约 260 行）。
 * 现在按职责分成：
 *   Control_Direction()  扫描 -> 误差 -> 方向环 -> 过弯/六边形仲裁 -> 左右目标
 *   Control_Speed()      编码器增量 -> 前馈 + 速度环 PID -> Motor_Output
 * 以及五个特殊动作：起步 kick、卡死恢复、原地转 360°、脱线直线 1 m、丢线停车。
 *
 * 【行为等价】本次只做搬运，不改任何状态迁移、限幅与调用顺序：
 *   - 方向环/速度环仍是每 10 个 1 kHz 拍执行一次，且对齐在同一拍；
 *   - 丢线停车 / 卡死停车仍会中断本拍后续逻辑（等价于原来的 return）；
 *   - 未发车时仍每 1 ms 强制写 0 并复位 kick。
 *
 * 设计备忘（0/40 方案、FF/死区标定、kick 微调）与已知隐患清单：
 * 见工程根目录 CONTROL_NOTES.md。
 */
#include "main.h"
#include <stdio.h>          /* [BLIND] printf 诊断 */
#include "control.h"
#include "car_config.h"
#include "car_state.h"
#include "gyro.h"
#include "motor.h"
#include "sensor.h"
#include "special.h"

/* htim3 / htim4 定义在 main.c。main.h 没有 extern 声明，这里显式引用。 */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* ---- 跨拍状态（实例私有，只在本文件可见） ---- */
static CarState car;
static CornerState corner;
static HexState hex;
static uint8_t hex_pivot_active;   /* [原地转] 六边形入口正在原地转 */
static float   turn_prev;          /* [SLEW 2026-11-16] 上一拍的转向命令 */
static float   err_f;              /* [KINEMA 2026-11-18] 低通后的误差（单位：路） */
static float   err_prev;           /* [KINEMA 2026-11-18] 上一拍滤波后的误差（算微分） */

/* [BLIND 2026-11-18] 断线区斜穿状态 */
static uint8_t  blind_phase;       /* 0=关 1=转向到目标角 2=保持航向直行 */
static float    blind_dir;         /* +1 / -1 */
static float    blind_yaw;         /* 已转过的角度，° */
static uint32_t blind_dist;        /* 已走过的 counts */
static uint8_t  blind_seen;        /* 连续看到线的拍数（退出用迟滞） */
static uint16_t blind_tick;        /* 本阶段计时（超时兜底） */

/* ---- 编码器 ---- */
static int16_t last_L = 0, last_R = 0;
static int16_t last_valid_dL = 0, last_valid_dR = 0;

/* ---- 调试快照：中断写、主循环读，只做显示用 ---- */
static volatile int16_t dbg_dL = 0, dbg_dR = 0;
static volatile float   dbg_speed_L = 0.0f, dbg_speed_R = 0.0f;

/* ================= 停车 / 发车 ================= */

static void Launch_Reset(void)
{
    car.launch.done = 0U;
    car.launch.kick_on = 0U;
    car.launch.kick_phase = 0U;
    car.launch.kick_total = 0U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;
}

/* 停车：清运行标志 + 复位目标和积分。
   复位目标是为了避免"停车那一刻目标停在左转状态，下次发车直接窜出去"。
   幂等：丢线、卡死、动作完成都走这里。 */
static void Car_Stop(void)
{
    car.run_flag = 0U;
    pid_L.target = BASE_SPEED;
    pid_R.target = BASE_SPEED;
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    car.speed_cmd = 0.0f;
    Launch_Reset();
    car.lost_cnt = 0U;
    Corner_Reset(&corner);
    Hex_Reset(&hex);
    turn_prev = 0.0f;      /* [SLEW 2026-11-16] 停车后清掉斜率基准 */
    err_f     = 0.0f;      /* [KINEMA 2026-11-18] 清掉误差滤波状态 */
    err_prev  = 0.0f;
    blind_phase = 0U;      /* [BLIND 2026-11-18] */
    blind_yaw   = 0.0f;
    blind_dist  = 0U;

    car.spin.mode = 0U;
    car.spin.yaw = 0.0f;
    car.spin.ticks = 0U;

    car.straight.mode = 0U;
    car.straight.yaw = 0.0f;
    car.straight.speed = 0.0f;
    car.straight.dist = 0U;
    car.straight.ticks = 0U;
}

void Control_Start(void)
{
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    car.speed_cmd = BASE_SPEED;
    Launch_Reset();
    car.run_flag = 1U;
}

uint8_t Control_IsRunning(void)
{
    return (car.run_flag != 0U) ? 1U : 0U;
}

/* ================= 原地转 360° ================= */

void Control_StartSpin(void)
{
    car.run_flag = 1U;
    car.launch.done = 1U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    car.speed_cmd = 0.0f;

    car.spin.yaw = 0.0f;
    car.spin.ticks = 0U;
    car.straight.mode = 0U;
    car.spin.mode = 1U;

    /* [GND 2026-11-14] 用和过弯 pivot 相同的速度环增益，
       否则 '2' 测出来比实际过弯弱 3 倍，没法作为调参依据。 */
    Motor_SetSpeedGains(PIVOT_KP, PIVOT_KI);
}

/* 方向环 100 Hz 调用：陀螺积分 + 角度闭环 */
static void Spin_Update(void)
{
    float yaw_abs;
    float err;
    float cmd;

    if (!Gyro_IsOk())
    {
        Car_Stop();
        return;
    }

    car.spin.yaw += Gyro_GetZ() * 0.01f;   /* 100 Hz，dt = 10 ms */
    ++car.spin.ticks;

    yaw_abs = (car.spin.yaw >= 0.0f) ? car.spin.yaw : -car.spin.yaw;

    if (yaw_abs >= SPIN_STOP_DEG || car.spin.ticks >= SPIN_TIMEOUT)
    {
        Car_Stop();
        return;
    }

    err = SPIN_TARGET_DEG - yaw_abs;

    if (car.spin.ticks <= SPIN_KICK_TICKS)
    {
        /* 前 180 ms 硬踹一下，打破静摩擦 */
        cmd = SPIN_KICK;
    }
    else
    {
        cmd = SPIN_KP * err;
        if (cmd > SPIN_MAX) cmd = SPIN_MAX;
        if (cmd < SPIN_MIN) cmd = SPIN_MIN;
    }

    pid_L.target = -cmd;   /* 左轮反转 */
    pid_R.target =  cmd;   /* 右轮正转 */
}

/* ================= 脱线直线 1 m ================= */

void Control_StartStraight(void)
{
    car.run_flag = 1U;
    car.launch.done = 1U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    car.speed_cmd = 0.0f;

    car.spin.mode = 0U;
    car.straight.yaw = 0.0f;
    car.straight.speed = 0.0f;
    car.straight.dist = 0U;
    car.straight.ticks = 0U;
    car.straight.mode = 1U;

    /* [GND 2026-11-14] 直线动作使用默认增益 */
    Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
}

/* 方向环 100 Hz 调用：陀螺航向保持 + 编码器距离 */
static void Straight_Update(void)
{
    float remain;
    float desired;
    float corr = 0.0f;

    ++car.straight.ticks;

    if (car.straight.dist >= STRAIGHT_TARGET_COUNTS ||
        car.straight.ticks >= STRAIGHT_TIMEOUT)
    {
        Car_Stop();
        return;
    }

    if (Gyro_IsOk())
        car.straight.yaw += Gyro_GetZ() * 0.01f;

    remain = (float)STRAIGHT_TARGET_COUNTS - (float)car.straight.dist;
    if (remain < 0.0f) remain = 0.0f;

    desired = STRAIGHT_SPEED;
    if (remain < (float)STRAIGHT_DECEL_COUNTS)
    {
        desired = STRAIGHT_MIN_SPEED +
                  (STRAIGHT_SPEED - STRAIGHT_MIN_SPEED) *
                  (remain / (float)STRAIGHT_DECEL_COUNTS);
    }

    if (car.straight.speed < desired) car.straight.speed += 0.5f;
    else                             car.straight.speed -= 0.5f;
    if (car.straight.speed < 0.0f) car.straight.speed = 0.0f;

    if (Gyro_IsOk())
    {
        corr = STRAIGHT_KP * (0.0f - car.straight.yaw)
             - STRAIGHT_KD * Gyro_GetZ();
        if (corr >  STRAIGHT_TURN_MAX) corr =  STRAIGHT_TURN_MAX;
        if (corr < -STRAIGHT_TURN_MAX) corr = -STRAIGHT_TURN_MAX;
    }

    pid_L.target = car.straight.speed - corr;
    pid_R.target = car.straight.speed + corr;
    if (pid_L.target < 0.0f) pid_L.target = 0.0f;
    if (pid_R.target < 0.0f) pid_R.target = 0.0f;
}

/* ================= 方向环 ================= */

/* 返回 1 = 本拍已丢线停车，Control_Update 必须立刻退出（等价于原来的 return）。 */
static uint8_t Control_Direction(void)
{
    LineSample line;
    HexOutput  hex_out;
    float      pid_turn = 0.0f;
    float      speed_local;
    float      turn;
    uint8_t    have_control;
    uint8_t    corner_was_active;

    /* 特殊动作只更新目标，其余巡线逻辑全部跳过 */
    if (car.spin.mode)
    {
        Spin_Update();
        return 0U;
    }
    if (car.straight.mode)
    {
        Straight_Update();
        return 0U;
    }

    /* 1) 12 路 -> 偏差 */
    Sensor_Scan(&line);

    /* 2) 方向环：先算出来交给 special 决定是否锁存 */
    if (line.cnt > 0U)//T:没丢线才上方向环？
    {
#if (DIR_MODE == 1)
        /* [KINEMA 2026-11-18] 运动学映射：误差 → 曲率 → 角速度 → 差速
           κ = 2e/d²，ω = v·κ，turn = ω/(2·v/L)  =>  turn = 13.3·v·e/d²
           只有一个参数：前视距离 d。天然满足 turn < v（内轮不反转）。 */
        {
            float v;
            float d;
            float turn_ff;
            float omega_des;
            float err_d;

            err_f += ERR_FILTER_ALPHA * (line.error - err_f);   /* 量化低通 */

            /* [KINEMA 2026-11-18] 微分项（阻尼）：纯 P 律在横向动力学上是
               无阻尼振荡器（ė = v·ψ, ψ̇ = ω, ω = -G·e  =>  ë = -v·G·e），
               必须用 Δe（≈车头角）提供阻尼，否则直道会一直抖。 */
            {
                float de = err_f - err_prev;
                err_prev = err_f;
                err_d    = err_f + KINEMA_TD * de;
            }

            v = car.speed_cmd;                  /* 上一拍的速度目标，够用 */
            if (v < 30.0f) v = 30.0f;           /* 防止低速时映射爆炸 */

            d = (Hex_ZoneActive() != 0U) ? PURSUIT_L_CM_HEX : PURSUIT_L_CM;

            /* [KINEMA 2026-11-18] 符号约定：error = bias - 5.5，error>0 表示线在右侧，
               车应该右转（turn < 0）。所以这里必须带负号——漏掉负号会变成正反馈，
               车会越偏越往外打舵、直接乱转（新方案错误1 就是这个）。 */
            turn_ff   = -CURVE_K * v * err_d / (d * d);  /* 期望差速（含微分阻尼） */
            omega_des = turn_ff * OMEGA_PER_TURN;        /* 期望角速度 °/s（右转为负） */

            /* [ARC 2026-11-18] 纯阻尼：只做速率反馈，不再用 K·(ω_des−ω)。
               原来那形式里 +K·ω_des 与 turn_ff 同号，会把位置增益从 16 放大到 29，
               正是直道高速抖动的来源。 */
            pid_turn = turn_ff - DAMP_K * Gyro_GetZ();
        }
#else
        pid_turn = DirPID_Calc(&dir_pid, line.error);
        pid_turn += DIR_QUAD * (-line.error) * ((line.error >= 0.0f) ? line.error : -line.error);

        /* [TURNBIAS 2026-11-18] 六边形区间内把转向增益放大，让内轮能降到 0 */
        if (Hex_ZoneActive() != 0U)
            pid_turn *= TURN_GAIN_BOOST;

        /* [DAMP 2026-11-16] 陀螺阻尼：车转得越快越"收油" */
        pid_turn -= DIR_DAMP * Gyro_GetZ();
#endif

        if (pid_turn >  TURN_MAX) pid_turn =  TURN_MAX;
        if (pid_turn < -TURN_MAX) pid_turn = -TURN_MAX;

        /* [SLEW 2026-11-16] 转向限斜率：每拍最多变 TURN_SLEW，削掉阶跃 */
        if (pid_turn > turn_prev + TURN_SLEW) pid_turn = turn_prev + TURN_SLEW;
        if (pid_turn < turn_prev - TURN_SLEW) pid_turn = turn_prev - TURN_SLEW;
        turn_prev = pid_turn;
    }

    /* 3) 丢线保护：过弯锁存期间/斜穿期间不因线暂时消失而停 */
    if (line.cnt == 0U && !Corner_IsActive(&corner) && (blind_phase == 0U))//丢线且不处于过弯态
    {
        if (++car.lost_cnt >= LOST_STOP_TICKS)
        {
            Car_Stop();
            return 1U;
        }
    }
    else
    {
        car.lost_cnt = 0U;
    }

    /* 4) 六边形检测先跑，会取消通用过弯锁存 */
    hex_out = Hex_Update(&hex, &corner,
                         line.center, line.right, line.left, line.mask,
                         Gyro_IsOk(), Gyro_GetZ());   /* [原地转] 陀螺闭环 */

    /* [原地转] 六边形入口在做原地转时，前馈必须按 pivot 给（FF_PIVOT），
       否则 FF_DRIVE×45 只有 144，连死区都越不过去。
       注意：这里先清零，只有 hex 真正拿到控制权时才置 1（见下面的门控）。 */
    hex_pivot_active = 0U;

    corner_was_active = Corner_IsActive(&corner);

    speed_local = BASE_SPEED;
    {
        /* [PIVOTFIX 2026-11-15] 传给过弯状态机的是“运动活跃度”，不是带符号平均值。
           pivot 时两轮反向（−45 / +45），带符号平均恒为 0，会让堵转保护
           每拍都判“不动”，0.4 s 就把 pivot 掐断（现场“只转 60° 就停”就是这么来的）。 */
        float sp_l = (dbg_speed_L >= 0.0f) ? dbg_speed_L : -dbg_speed_L;
        float sp_r = (dbg_speed_R >= 0.0f) ? dbg_speed_R : -dbg_speed_R;

        turn = Corner_Update(&corner, pid_turn, line.error, line.cnt,
                             line.mask,                          /* [CW 2026-11-15] 直角弯检测用 */
                             Gyro_IsOk(), Gyro_GetZ(),
                             (sp_l + sp_r) * 0.5f,               /* 幅值平均：pivot 时 ≈ 45，真堵转时 ≈ 0 */
                             &speed_local);
    }
    have_control = (line.cnt > 0U || corner_was_active ||
                    Corner_IsActive(&corner)) ? 1U : 0U;
    car.speed_cmd = speed_local;

    /* 六边形满舵覆盖通用过弯。
       [APPROACH 2026-11-14] 过弯四段期间不允许 hex 抢控制权，
       否则冲弯/刹车会被 15 拍的满舵覆盖打断。
       [CW 2026-11-15] 出弯冷却期内也不许抢：pivot 刚停的那一拍
       Corner_IsActive 已经变假，hex 会立刻把舵拉到 ±25，看起来像"又转了一下"。 */
    if (hex_out.have_control && !Corner_IsActive(&corner) && (Corner_IsCooling(&corner) == 0U))
    {
        turn = hex_out.turn;
        car.speed_cmd = hex_out.speed_cmd;
        have_control = 1U;

        /* [原地转] 速度目标为 0 = 在做原地转，前馈改用 FF_PIVOT */
        hex_pivot_active = (hex_out.speed_cmd == 0.0f) ? 1U : 0U;
    }

    /* [BLIND 2026-11-18] 断线区斜穿：不是直角弯、不是六边形时的丢线兜底。
       转向到 tan=24/65（20.3°）-> 保持航向直行 √(24²+65²)=69.3 cm，
       期间一旦看到线立刻交回方向环。 */
    if ((Corner_IsActive(&corner) == 0U) && (Hex_ZoneActive() == 0U))
    {
        if (blind_phase == 0U)
        {
            if ((line.cnt == 0U) && (car.lost_cnt >= BLIND_START_TICKS))
            {
                blind_phase = 1U;
                blind_dir   = BLIND_DIR_FIXED;   /* [BLIND] 方向固定，不看丢线前的误差 */
                blind_yaw   = 0.0f;
                blind_dist  = 0U;
                blind_seen  = 0U;
                blind_tick  = 0U;
                printf("BLD,1,%d,%d,%d\r\n", (int)car.lost_cnt, (int)blind_dir, (int)(err_f * 10.0f));
            }
        }
        else
        {
            float ya;

            /* [FIX 2026-11-18] 退出要连续 BLIND_SEEN_TICKS 拍看到线。
               原来 1 拍就退出，断线区边缘闪一下 -> 整个状态重置 -> 角度归零
               -> 再启动再归零，永远转不到目标角。 */
            if (line.cnt > 0U)
            {
                if (blind_seen < 250U) ++blind_seen;
            }
            else
            {
                blind_seen = 0U;
            }

            if (blind_seen >= BLIND_SEEN_TICKS)
            {
                blind_phase = 0U;                               /* 真的找到线了，交回 */
            }
            else
            {
                if (Gyro_IsOk())
                    blind_yaw += Gyro_GetZ() * 0.01f;

                blind_dist += (uint32_t)(((dbg_speed_L < 0.0f ? -dbg_speed_L : dbg_speed_L) +
                                          (dbg_speed_R < 0.0f ? -dbg_speed_R : dbg_speed_R)) * 0.5f);
                ++blind_tick;

                if (blind_phase == 1U)
                {
                    turn          = blind_dir * BLIND_TURN;
                    car.speed_cmd = BLIND_TURN_SPEED;

                    ya = (blind_yaw >= 0.0f) ? blind_yaw : -blind_yaw;
                    if ((ya >= BLIND_ANGLE_DEG) || (blind_tick >= BLIND_TURN_MAX_TICKS))
                    {
                        blind_phase = 2U;                       /* 角度到 / 超时 -> 转直行 */
                        blind_tick  = 0U;
                    }
                }
                else
                {
                    float e = (blind_dir * BLIND_ANGLE_DEG) - blind_yaw;

                    turn          = BLIND_K_YAW * e;
                    car.speed_cmd = BLIND_SPEED;

                    if ((blind_dist >= BLIND_DIST_COUNTS) || (blind_tick >= BLIND_RUN_MAX_TICKS))
                        blind_phase = 0U;                       /* 走完还没找到：放弃，交给丢线停车 */
                }
                have_control = 1U;
            }
        }
    }

    /* 5) 差速 -> 左右目标速度（过弯/正常统一在这里写） */
    if (have_control)
    {        /* [NOSPIN 2026-11-18] 不反转护栏：巡线/六边形弧线阶段，turn 不允许超过
           speed_cmd，否则内轮目标为负 -> 电机反接制动（电流数倍、驱动易烧）。
           pivot/原地转走的是另一条路（target 允许为负），所以这里要排除。 */
        if ((!corner_was_active) && (!Corner_IsActive(&corner)) &&
            (car.spin.mode == 0U) && (hex_pivot_active == 0U))
        {
            if (turn >  car.speed_cmd) turn =  car.speed_cmd;
            if (turn < -car.speed_cmd) turn = -car.speed_cmd;
        }

        pid_L.target = car.speed_cmd - turn;
        pid_R.target = car.speed_cmd + turn;

        /* [SAFE 2026-11-14] 目标护栏：任何软件错误都不允许把目标推到几百，
           否则前馈会把占空比打满（现场出现过的“猛冲 + 呼啸”）。 */
        if (pid_L.target >  TARGET_MAX) pid_L.target =  TARGET_MAX;
        if (pid_L.target < -TARGET_MAX) pid_L.target = -TARGET_MAX;
        if (pid_R.target >  TARGET_MAX) pid_R.target =  TARGET_MAX;
        if (pid_R.target < -TARGET_MAX) pid_R.target = -TARGET_MAX;

        /* [BRAKE 2026-11-18] 弧线起步反向脉冲：
           EN=0 是滑行，内轮从 120 掉到 0 要 0.27 s（车前进 7 cm 才开始转）。
           这里给内轮一个短促反接，把它压到 ~0.06 s。
           [PIVOT-C] 方案 C 的内轮本来就在反转（25−120=−95），不再需要这个脉冲。 */
#if (ARC_BRAKE_ENABLE == 1)
        if ((corner.state == CORNER_ST_ARC) && (corner.arc_ticks <= ARC_BRAKE_TICKS))
        {
            if (turn > 0.0f)                 /* 左转：内轮是左轮 */
            {
                if (dbg_speed_L > ARC_BRAKE_SPEED_MIN)
                    pid_L.target = ARC_BRAKE_TARGET;
            }
            else if (turn < 0.0f)            /* 右转：内轮是右轮 */
            {
                if (dbg_speed_R > ARC_BRAKE_SPEED_MIN)
                    pid_R.target = ARC_BRAKE_TARGET;
            }
        }
#endif
				if (!corner_was_active && !Corner_IsActive(&corner)) {   /* [PIVOT 2026-11-14] 锁存期允许一侧目标为负（两轮反向） */
        if (pid_L.target < 0.0f) pid_L.target = 0.0f;
        if (pid_R.target < 0.0f) pid_R.target = 0.0f;
			}
    }
    else
    {
        /* [FIX 2026-11-18] 没有控制权（丢线 + 不在过弯/六边形/斜穿里）时，
           必须把目标清零。原来这一块整个跳过 -> 目标保持上一次的值，
           双六边形1 的最后一段就是：六边形转完、线丢了，目标冻在 0/450，
           车原地打转 0.4 s 才被斜穿接管，直接转出赛道。 */
        pid_L.target = 0.0f;
        pid_R.target = 0.0f;
    }

    return 0U;
}

/* ================= 速度环 ================= */

/* [FF 2026-11-14] 按工况选前馈：
   直道轮子自由滚动（负载小），原地转两轮互顶+横刮（负载大 7 倍）。 */
static uint8_t FF_IsPivotMode(void)
{
    return (uint8_t)((car.spin.mode != 0U) ||
                     (corner.state == CORNER_ST_PIVOT) ||
                     (corner.state == CORNER_ST_SETTLE) ||
                     (corner.state == CORNER_ST_RECOVER) ||   /* [RECOVER 2026-11-16] 找线也是原地转 */
                     (hex_pivot_active != 0U));   /* [原地转] 六边形入口原地转也按 pivot 给前馈 */
}

static float FF_L(float target)
{
    if (FF_IsPivotMode())
        return (target >= 0.0f) ? FF_PIVOT_L : FF_PIVOT_L_REV;
    return (target >= 0.0f) ? FF_DRIVE_L : FF_DRIVE_L_REV;
}

static float FF_R(float target)
{
    if (FF_IsPivotMode())
        return (target >= 0.0f) ? FF_PIVOT_R : FF_PIVOT_R_REV;
    return (target >= 0.0f) ? FF_DRIVE_R : FF_DRIVE_R_REV;
}

/* 返回 1 = 连续卡死已停车，Control_Update 必须立刻退出（等价于原来的 return）。 */
static uint8_t Control_Speed(void)
{
    int16_t now_L;
    int16_t now_R;
    int16_t dL;
    int16_t dR;
    int32_t dL_abs;
    int32_t dR_abs;
    float   speed_L;
    float   speed_R;

    now_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    now_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    dL = (int16_t)(now_L - last_L);
    dR = (int16_t)(now_R - last_R);
    last_L = now_L;
    last_R = now_R;

    /* 坏采样保护：超过 ENC_DELTA_LIMIT 就沿用上一次有效值 */
    if (dL > ENC_DELTA_LIMIT || dL < -ENC_DELTA_LIMIT)
        dL = last_valid_dL;
    else
        last_valid_dL = dL;

    if (dR > ENC_DELTA_LIMIT || dR < -ENC_DELTA_LIMIT)
        dR = last_valid_dR;
    else
        last_valid_dR = dR;

    dbg_dL = dL;
    dbg_dR = dR;

    /* 距离累计：左右轮平均路径，单位 counts */
    dL_abs = (dL < 0) ? -(int32_t)dL : (int32_t)dL;
    dR_abs = (dR < 0) ? -(int32_t)dR : (int32_t)dR;
    Hex_AddDistance(&hex, (uint32_t)((dL_abs + dR_abs) / 2));
    car.straight.dist += (uint32_t)((dL_abs + dR_abs) / 2);
    Corner_AddDistance(&corner, (uint32_t)((dL_abs + dR_abs) / 2));   /* [APPROACH 2026-11-14] */

    /* PID 实际使用的速度：保持当前符号（左轮反转计数，所以取负） */
    speed_L = -(float)dL;          /* TIM3 左 */
    speed_R =  (float)dR;          /* TIM4 右 */

    /* 停车推车时也更新，供主循环打印 */
    dbg_speed_L = speed_L;
    dbg_speed_R = speed_R;

    /* 未发车只采样、不跑 PID，避免停车时积分乱积。
       电机由 Control_Update 末尾的 !run_flag 分支强制写 0。 */
    if (car.run_flag)
    {
        /* speed_cmd 由方向环按 turn 更新（弯道降速），这里不要再覆盖 */

        if (!car.launch.done && !car.spin.mode && !car.straight.mode)
        {
            /* 阶段 1：短脉冲 kick：60 ms on / 60 ms off，
               直到两轮都 > 4，或总时长 600 ms。 */
            if (speed_L > KICK_EXIT_SPEED && speed_R > KICK_EXIT_SPEED)
            {
                car.launch.done = 1U;
                car.launch.settle_timer = SETTLE_TICKS;
            }
            else if (car.launch.kick_total < KICK_MAX_TICKS)
            {
                if (car.launch.kick_phase == 0U)
                {
                    car.launch.kick_on = (car.launch.kick_on == 0U) ? 1U : 0U;
                    car.launch.kick_phase = (car.launch.kick_on != 0U) ? KICK_ON_TICKS : KICK_OFF_TICKS;
                }
                --car.launch.kick_phase;
                ++car.launch.kick_total;

                if (car.launch.kick_on != 0U) Motor_Output(KICK_L, KICK_R);
                else                          Motor_Output(0.0f, 0.0f);
            }
            else
            {
                car.launch.done = 1U;
                car.launch.settle_timer = SETTLE_TICKS;
            }
        }

        /* 中途卡住：速度接近 0 持续 300 ms，重新脉冲 kick。
           spin / straight 模式不要走这个逻辑，否则会两轮正转把车往前踹。 */
        if (car.launch.done && !car.spin.mode && !car.straight.mode&&!Corner_IsActive(&corner))
        {
            if (speed_L < STALL_SPEED && speed_R < STALL_SPEED)
            {
                if (++car.launch.stall_cnt >= STALL_TICKS)
                {
                    car.launch.stall_cnt = 0U;
                    if (car.launch.stall_retry < STALL_RETRIES)
                    {
                        ++car.launch.stall_retry;
                        car.launch.done = 0U;
                        car.launch.kick_on = 0U;
                        car.launch.kick_phase = 0U;
                        car.launch.kick_total = 0U;
                        car.launch.settle_timer = 0U;
                        Motor_Output(0.0f, 0.0f);
                    }
                    else
                    {
                        /* 连续 3 次还起不来，停车保护 */
                        Car_Stop();
                        return 1U;
                    }
                }
            }
            else
            {
                car.launch.stall_cnt = 0U;
                if (speed_L > 10.0f && speed_R > 10.0f)
                    car.launch.stall_retry = 0U;
            }
        }

        if (car.launch.done)
        {
            /* 如果已经明显超速，跳过 FF 保持，直接交给 PID 平滑降输出。
               不要用"超过阈值就 0 占空比"的硬刹车，否则 CCR 会在 0 和前馈值
               之间每 10 ms 跳，看起来就是"非常抖动"。 */
            if (car.launch.settle_timer > 0U &&
                speed_L < BASE_SPEED + 5.0f &&
                speed_R < BASE_SPEED + 5.0f)
            {
                --car.launch.settle_timer;
                Motor_Output(FF_L(pid_L.target) * pid_L.target,
                             FF_R(pid_R.target) * pid_R.target);
            }
            else
            {
                float ff_l = FF_L(pid_L.target) * pid_L.target;
                float ff_r = FF_R(pid_R.target) * pid_R.target;

                /* [GND 2026-11-14] 差速前馈：速度前馈降低后，单靠 FF×turn 不足以
                   压过左右电机的效率差，方向环的小修正会被死区吞掉。
                   这里单独给“两轮差速”一个增益（只加差、不改平均）。 */
                if (!FF_IsPivotMode())
                {
                    float dtrn = (pid_R.target - pid_L.target) * 0.5f;
                    float dff  = DIR_DIFF_GAIN * dtrn;

                    /* [FIX 2026-11-18] 钳位：turn 到 120 时 10×120=1200 会把外轮
                       打到 CCR 1745、内轮断电，并造成积分饱和式超调。 */
                    if (dff >  DIR_DIFF_MAX) dff =  DIR_DIFF_MAX;
                    if (dff < -DIR_DIFF_MAX) dff = -DIR_DIFF_MAX;

                    ff_l -= dff;
                    ff_r += dff;
                }

                Motor_Output(ff_l + PID_Calc(&pid_L, speed_L),
                             ff_r + PID_Calc(&pid_R, speed_R));
            }
        }
    }

    return 0U;
}

/* ================= 1 kHz 调度 ================= */

void Control_Update(void)
{
    static uint16_t div_gyro = 0U;
    static uint16_t div_dir = 0U;
    static uint16_t div_speed = 0U;

    /* 陀螺仪采样：200 Hz（每 5 ms）。
       放中断里是为了周期严格；放主循环会被 printf 阻塞抖到好几毫秒。 */
    if (++div_gyro >= 5U)
    {
        div_gyro = 0U;
        Gyro_Task();
    }

    /* 方向环：100 Hz（10 ms） */
    if (++div_dir >= 10U)
    {
        div_dir = 0U;
        if (Control_Direction())
            return;
    }

    /* 速度环：100 Hz（10 ms） */
    if (++div_speed >= SPEED_DIV)
    {
        div_speed = 0U;
        if (Control_Speed())
            return;
    }

    /* 未发车：电机停转。
       注意：这里不要再刷新 last_L/last_R！
       它们只在 10 ms 采样里更新，所以不发车也能看到真实的推车速度。 */
    if (!car.run_flag)
    {
        Motor_Output(0.0f, 0.0f);
        car.speed_cmd = 0.0f;
        /* [BAL 2026-11-14] 停车时把目标也清零：
           否则过弯途中停车会把上一拍的 ±320 一直挂在串口上，看着像还在转。 */
        pid_L.target = 0.0f;
        pid_R.target = 0.0f;
        Launch_Reset();               /* 下次发车重新 kick */
        return;
    }
}

/* ================= 调试快照 ================= */

void Control_GetDebug(ControlDebug *d)
{
    d->speed_cmd = car.speed_cmd;
    d->speed_L   = dbg_speed_L;
    d->speed_R   = dbg_speed_R;
    d->dL        = dbg_dL;
    d->dR        = dbg_dR;
    d->target_L  = pid_L.target;
    d->target_R  = pid_R.target;
    d->gyro_z    = Gyro_GetZ();
    d->run       = car.run_flag;
}

void Control_GetHexDebug(HexDebug *d)
{
    Hex_GetDebug(&hex, d);
}
