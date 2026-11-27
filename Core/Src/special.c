#include "special.h"
#include "car_config.h"
#include "motor.h"      /* [PIVOT 2026-11-14] 出弯清速度环积分，需要 pid_L/pid_R */
#include <stdio.h>      /* [CORN 2026-11-15] 锁存快照打印 */

/* [HEX-ZONE 2026-11-15] 1 = 当前处于六边形区间（最近一次 V 之后还没走远），
   期间禁用直角弯检测：六边形里连续的 60° 折角会被误判成直角弯。
   Hex_Update 每拍刷新（它在 Corner_Update 之前调用）。 */
static uint8_t hex_zone_flag = 0U;

/* [CW4 2026-11-22] 直角弯"全亮后全丢线"确认状态（Corner_Reset 也要清） */
static uint8_t  cw_pend;                   /* 1 = 已看到全亮，等丢线确认 */
static uint16_t cw_pend_wait;              /* 等确认已经过的拍数（超时作废） */
static uint8_t  corner_gave_up;            /* [LOST3] 直角弯补转 20° 后仍无线 */

/* [ARC-RATE 2026-11-25] 陀螺角速度环的上一拍修正量（限斜率用） */
static float    arc_rate_trim;

/* [FULLDIST 2026-11-24] 诊断：本次运行的第一个“全亮”（>=10 路亮）到当前的距离。
   cw_full_active 只用来找“进入全亮”的上升沿；cw_full_valid 一旦置 1，
   起点就固定不动，后面再出现全亮也不重置。 */
static volatile uint8_t  cw_full_valid;    /* 1 = 本次运行的第一个全亮已记录 */
static uint8_t           cw_full_active;   /* 上一拍是否处于全亮 */
static volatile uint32_t cw_full_dist;     /* 本次第一个全亮时的绝对里程（counts） */

/* [FULLDIST 2026-11-24] 绝对里程：只随 Corner_AddDistance 增加，
   不随 Corner_Enter 的 c->dist 清零而回零。FULLDIST 必须用它，
   否则 Corner_Enter 后 c->dist=0，会算出 2^32 附近的负数。 */
static volatile uint32_t corner_odom_dist;
/* [CORN 2026-11-15] 锁存瞬间的快照：确认"线在哪一侧"和"转哪个方向"是否一致。
   格式：CORN,<why>,<dir>,<mask>,<cw_mask>,<cnt>,<error×10>
   mask 的 bit i = 通道 i。若把车放在线左侧时 mask 高位亮，说明通道顺序是镜像的。

   [ISR-SAFE 2026-11-21] 这里原来直接 printf，而它是在 TIM2 的 1 kHz 中断里调用的：
   一行 ~28 字符 = 2.4 ms，超过 1 ms 节拍 —— 会吞掉控制拍，把触发时序和 CW 记录的
   拍数一起搞乱（交接清单第 2 条那个坑）。现在中断里只把事件塞进环形缓冲，
   由主循环 Corner_PopEvent() 取出来打印。 */
#define CORNLOG_MAX 8U

typedef struct {
    uint8_t  why;
    int8_t   dir;
    uint16_t mask;
    uint16_t cw_mask;
    int16_t  cnt;
    int16_t  err10;
} CornEvent;

static CornEvent corn_ev[CORNLOG_MAX];
static volatile uint8_t corn_head;
static volatile uint8_t corn_tail;

static void Corner_Log(uint8_t why, float dir, uint16_t mask, uint16_t cw_mask,
                       int cnt, float error)
{
    uint8_t nxt = (uint8_t)((corn_head + 1U) % CORNLOG_MAX);

    if (nxt == corn_tail)
        return;                        /* 缓冲满：丢掉新事件，绝不覆盖没打印的 */

    corn_ev[corn_head].why     = why;
    corn_ev[corn_head].dir     = (int8_t)dir;
    corn_ev[corn_head].mask    = mask;
    corn_ev[corn_head].cw_mask = cw_mask;
    corn_ev[corn_head].cnt     = (int16_t)cnt;
    corn_ev[corn_head].err10   = (int16_t)(error * 10.0f);
    corn_head = nxt;
}

/* 主循环调用：取出一条事件，返回 1 = 取到了 */
uint8_t Corner_PopEvent(uint8_t *why, int8_t *dir, uint16_t *mask, uint16_t *cw_mask,
                        int16_t *cnt, int16_t *err10)
{
    if (corn_tail == corn_head)
        return 0U;

    *why     = corn_ev[corn_tail].why;
    *dir     = corn_ev[corn_tail].dir;
    *mask    = corn_ev[corn_tail].mask;
    *cw_mask = corn_ev[corn_tail].cw_mask;
    *cnt     = corn_ev[corn_tail].cnt;
    *err10   = corn_ev[corn_tail].err10;

    corn_tail = (uint8_t)((corn_tail + 1U) % CORNLOG_MAX);
    return 1U;
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
    c->arc_brake_active = 0U;  /* [ARC-BRAKE 2026-11-23] */
    c->arc_brake_ticks  = 0U;

    cw_pend      = 0U;      /* [CW4] 丢线确认状态也清 */
    cw_pend_wait = 0U;
    corner_gave_up = 0U;    /* [LOST3] 直角弯补转放弃标志 */
    arc_rate_trim  = 0.0f;  /* [ARC-RATE] */

    cw_full_valid  = 0U;    /* [FULLDIST] */
    cw_full_active = 0U;
    cw_full_dist   = 0U;
    corner_odom_dist = 0U;
}

uint8_t Corner_IsActive(const CornerState *c)
{
    return (c->state != 0U) ? 1U : 0U;
}

/* [LOST3 2026-11-22] 丢线第一拍 Corner_IsActive 还是 0（CW4 要等全丢 2 拍），
   但 cw_pend 已经在"看到全亮"时置 1 —— control 用这个标志判断"直角弯候选"。 */
uint8_t Corner_IsPending(void)
{
    return (cw_pend != 0U) ? 1U : 0U;
}

/* [LOST3] 直角弯补转 20° 后仍无线：control 看到就直接 LOST_STOP。 */
uint8_t Corner_GaveUpNoLine(void)
{
    return (corner_gave_up != 0U) ? 1U : 0U;
}

/* [FULLDIST 2026-11-24] 主循环取“本次运行第一个全亮到当前”的距离（counts）。
   返回 1 = 本次运行已经出现过全亮；0 = 还没出现过。 */
uint8_t Corner_GetFullDist(const CornerState *c, uint32_t *dist)
{
    (void)c;

    if ((cw_full_valid == 0U) || (corner_odom_dist < cw_full_dist))
        return 0U;

    *dist = corner_odom_dist - cw_full_dist;
    return 1U;
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
    corner_odom_dist += counts;     /* [FULLDIST] 绝对里程，不随 Corner_Enter 清零 */
}

/* [CW2 2026-11-21] WIDE（单侧宽线 -> 等全黑）这一整套判据停用。
   原因：三份实测日志里它只命中过误触发 —— 地面标记的左右边缘（一侧 6 路全亮）、
   45° 弯的贴边、ch10 假信号都能把它武装；而它是锁存结构（80 拍超时），
   直道上 3 路的线永远满足不了"窄且居中"的解除条件，于是"武装后等下一次丢线"，
   直路上随便丢一次线就转 90°。
   代码用 #if 0 保留，方便需要时回滚；正式删除等确认新判据稳定后再说。 */
#if 0
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
#endif /* 0: Corner_SideOfMask 停用 */

/* 进入过弯四段（通用锁存与直角弯检测共用，保证两处初始化一致） */
static void Corner_Enter(CornerState *c, float dir)
{
#if (CORNER_MODE == 1)
    /* [ARC 2026-11-18] 弧线模式：不停车，直接进偏航双环 */
    c->state = CORNER_ST_ARC;
    c->arc_brake_active = 1U;   /* [ARC-BRAKE] 入口里程刹车 */
    c->arc_brake_ticks  = 0U;

    /* 入弯前清速度环积分，保证每次刹车起点一致 */
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    pid_L.pre_error = 0.0f;
    pid_R.pre_error = 0.0f;
#else
    c->state = CORNER_ST_APPROACH;
    c->arc_brake_active = 0U;
    c->arc_brake_ticks  = 0U;
#endif
    c->dir   = dir;
    c->arc_recover_dir = dir;   /* [ARC-RECOVER] 默认原方向，之后按最后有效线改写 */
    c->last_valid_mask = 0U;    /* [ARC-RECOVER] 本次弯重新记录 */

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

    corner_gave_up = 0U;    /* [LOST3] 新弯开始，清掉上一次的"补转放弃" */
    c->recover_dir = 0.0f;  /* [RECOVER 2026-11-16] */
    c->recover_ticks = 0U;
    c->arc_recover_dir = 0.0f;  /* [ARC-RECOVER 2026-11-24] */
    c->last_valid_mask = 0U;
    c->arc_ticks = 0U;      /* [ARC 2026-11-18] */
    c->arc_line_ticks = 0U;
    arc_rate_trim = 0.0f;   /* [ARC-RATE] 新弯从 0 开始修正 */
}

#if 0
/* 直角弯状态机（旧 WIDE 版）：巡线时每拍调用（包括 cnt==0 的拍） */
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
#endif /* 0: 旧 WIDE 判据结束 */

/* [CW2 2026-11-21] 直角弯判据的逐拍记录器：中断里只存，主循环按 'c' 打印。
   每拍 4 个数：cnt / mask / error×10 / 标志位
     标志 bit0 = 左侧全亮, bit1 = 右侧全亮, bit2 = 丢线
   用途：CORNER_FULL_HOLD_TICKS 不靠猜，直接从记录里数
   ——"全亮那一拍"到"mask=0 的第一拍"之间隔了几拍，一次就量出来。 */
#define CWLOG_MAX 128U
static int16_t  cw_log_cnt[CWLOG_MAX];
static uint16_t cw_log_mask[CWLOG_MAX];
static int16_t  cw_log_err[CWLOG_MAX];
static uint8_t  cw_log_flag[CWLOG_MAX];
static uint8_t  cw_log_head;
static uint8_t  cw_log_num;
static volatile uint8_t cw_log_pause;      /* 1 = 主循环正在打印，暂停写入 */
static volatile uint16_t cw_fire_cnt;      /* [CW2] 判据满足次数（还不等于真的进状态机） */
static volatile uint16_t cw_enter_cnt;     /* [CW2] 真正进了直角弯状态机（= 真的打舵） */
static volatile uint16_t cw_blk_arm;       /* [CW2] 判据满足但没武装，被挡 */
static volatile uint16_t cw_blk_cool;      /* [CW2] 判据满足但还在出弯冷却，被挡 */

/* [CWDBG 2026-11-22] 直角弯判据事件环：中断只入队，主循环打印。
   用来一次性看清"丢线沿 -> 候选 -> fire -> 进/被挡"的完整链路。 */
#define CWDEVT_MAX 16U
static CwdDebug cwdevt[CWDEVT_MAX];
static volatile uint8_t cwdevt_head;
static volatile uint8_t cwdevt_tail;

static void Cwd_Push(uint8_t code, const CornerState *c, int cnt,
                     uint16_t mask, float error)
{
    uint8_t nxt = (uint8_t)((cwdevt_head + 1U) % CWDEVT_MAX);
    CwdDebug *d;

    if (nxt == cwdevt_tail)
        return;                     /* 满：丢新事件 */

    d = &cwdevt[cwdevt_head];
    d->code    = code;
    d->cnt     = (uint8_t)cnt;
    d->mask    = mask;
    d->cw_mask = c->cw_mask;
    d->low     = (uint8_t)c->cw_low;
    d->wide    = (uint8_t)c->cw_wide;
    d->pend    = cw_pend;
    d->dark    = (uint8_t)c->cw_dark;
    d->ticks   = c->cw_ticks;
    d->armed   = c->armed;
    d->cool    = (uint8_t)Corner_IsCooling(c);
    d->hz      = hex_zone_flag;
    d->state   = c->state;
    d->err10   = (int16_t)(error * 10.0f);
    cwdevt_head = nxt;
}

uint8_t Corner_PopDebug(CwdDebug *d)
{
    if (cwdevt_tail == cwdevt_head)
        return 0U;

    *d = cwdevt[cwdevt_tail];
    cwdevt_tail = (uint8_t)((cwdevt_tail + 1U) % CWDEVT_MAX);
    return 1U;
}

static void CwLog_Push(int cnt, uint16_t mask, float error, uint8_t flag)
{
    if (cw_log_pause != 0U)
        return;

    cw_log_cnt[cw_log_head]  = (int16_t)cnt;
    cw_log_mask[cw_log_head] = mask;
    cw_log_err[cw_log_head]  = (int16_t)(error * 10.0f);
    cw_log_flag[cw_log_head] = flag;

    cw_log_head = (uint8_t)((cw_log_head + 1U) % CWLOG_MAX);
    if (cw_log_num < CWLOG_MAX) ++cw_log_num;
}

uint8_t Cw_LogCount(void)
{
    return cw_log_num;
}

uint16_t Cw_FireCount(void)
{
    return cw_fire_cnt;
}

uint16_t Cw_EnterCount(void)   { return cw_enter_cnt; }
uint16_t Cw_BlockArm(void)     { return cw_blk_arm;   }
uint16_t Cw_BlockCool(void)    { return cw_blk_cool;  }

void Cw_GetAt(uint8_t i, int16_t *cnt, uint16_t *mask, int16_t *err, uint8_t *flag)
{
    uint8_t k;

    if (i >= cw_log_num)
    {
        *cnt = -1; *mask = 0U; *err = 0; *flag = 0U;
        return;
    }

    k = (uint8_t)((cw_log_head + CWLOG_MAX - cw_log_num + i) % CWLOG_MAX);
    *cnt  = cw_log_cnt[k];
    *mask = cw_log_mask[k];
    *err  = cw_log_err[k];
    *flag = cw_log_flag[k];
}

void Cw_ClearLog(void)
{
    cw_log_pause = 1U;
    cw_log_head  = 0U;
    cw_log_num   = 0U;
    cw_log_pause = 0U;
}

/* [CW2 2026-11-21] 直角弯新判据（全部来自实测，不猜）：
     1) 丢线前 CORNER_FULL_HOLD_TICKS(25) 拍内，某一拍"一侧 6 路全亮、另一侧没全亮"；
        —— 手推直角弯实测：mask 127 持续 16 拍，之后 18 拍才丢线。
     2) 线连续 CORNER_LOST_TICKS(5) 拍完全不在阵列里；
     3) 丢线前最后一条有效线的质心必须在中间附近（3.5 < bias < 7.5）。
        —— 这是 45° 和 90° 最本质的区别：
           90° 车头正对拐角，线最后从"阵列正中"扫出去（实测 mask 32 = ch5）；
           45° 车跟不住，线是"贴着一边"跑掉的（实测 mask 3072 = ch10,11）。
        这条同时把 ch10 假信号挡在门外：假信号在边上，质心不可能居中。
   方向取那个全亮侧：左半排全亮 -> 左转 dir = +1。 */
static void Corner_Watch(CornerState *c, uint16_t mask, int cnt, float error,
                         float *dir_out, uint8_t *fire, uint8_t *gave_up)
{
    uint8_t i;
    uint8_t l = 0U, r = 0U;
    float   bias = 0.0f;
    uint8_t centered = 0U;

    *fire    = 0U;
    *gave_up = 0U;

    for (i = 0U; i < 6U; ++i)
        if ((mask & (uint16_t)(1U << i)) != 0U) ++l;
    for (i = 6U; i < 12U; ++i)
        if ((mask & (uint16_t)(1U << i)) != 0U) ++r;

    if ((l + r) > 0U)
    {
        uint16_t sum = 0U;
        uint8_t  n   = 0U;

        for (i = 0U; i < 12U; ++i)
            if ((mask & (uint16_t)(1U << i)) != 0U) { sum += i; ++n; }

        if (n > 0U)
        {
            bias = (float)sum / (float)n;
            /* [CW2 2026-11-21] 判据不是"必须正中"，而是"线不是从阵列边缘跑掉的"：
               45° 跟不住时线是贴到 ch10/ch11（bias 9.5~11）或 ch0/ch1（bias 0.5）才消失；
               90° 是车头正对拐角，线从中间一带消失（实测 bias 3.5~5）。
               所以取 2.0 < bias < 8.0。跑起来时线可能在还宽着的时候就没了（bias≈3.5），
               原来的 3.5 是硬边界，等于把这种情况判死。 */
            if ((bias > 2.0f) && (bias < 8.0f)) centered = 1U;
        }
    }

    /* 逐拍记录（按 'c' 打印），用于实测窗口/丢线拍数 */
    {
        uint8_t fl = 0U;
        if ((l >= CORNER_SIDE_FULL) && (r < CORNER_SIDE_FULL)) fl |= 0x01U;
        if ((r >= CORNER_SIDE_FULL) && (l < CORNER_SIDE_FULL)) fl |= 0x02U;
        if (cnt == 0) fl |= 0x04U;
        if (centered != 0U) fl |= 0x08U;        /* bit3 = 质心居中 */
        CwLog_Push(cnt, mask, error, fl);
    }

    if (cnt > 0)
    {
        c->cw_dark = 0U;                        /* 复用：连续丢线拍数 */
        c->cw_mask = mask;                      /* 记下最后一条有效线 */

        /* [2026-11-24] 全亮否决改成可关：
           CORNER_FULLBRIGHT_VETO=1 时，>=10 路亮（十字/地面标记）会清掉
           已经挂起的 CW4 候选；=0 时保留候选，让“一侧 6 路全亮 + 后接丢线”
           在六边形出口/45° 接直角处仍然能判成直角弯。 */
#if (CORNER_FULLBRIGHT_VETO == 1)
        if (cnt >= 10)
        {
            c->cw_low   = 0U;
            c->cw_wide  = 0U;
            c->cw_ticks = 0U;
            cw_pend      = 0U;      /* [CW4] 十字路口也取消等确认 */
            cw_pend_wait = 0U;
        }
#endif

        /* 1) 一侧全亮、另一侧没全亮：拐角的"L"形。
              "另一侧也没全亮"用来排除 12 路全亮的地面标记。 */
        {
            uint8_t full_l = (uint8_t)((l >= CORNER_SIDE_FULL) && (r < CORNER_SIDE_FULL));
            uint8_t full_r = (uint8_t)((r >= CORNER_SIDE_FULL) && (l < CORNER_SIDE_FULL));

            if (full_l || full_r)
            {
                c->cw_wide  = 1U;
                c->cw_ticks = 0U;
                c->cw_side  = (full_l != 0U) ? 1.0f : -1.0f;   /* +1 左转 / -1 右转 */

#if (CORNER_CW3_CONFIRM_TICKS > 0)
                if (cw_pend == 0U)      /* 已挂起等丢线时不再累加，别反复重置计时 */
#endif
                {
                    if (c->cw_low < 250U) ++c->cw_low;         /* 复用：连续全亮拍数 */
                }
            }
            else
            {
                c->cw_low = 0U;
                if (c->cw_wide != 0U)
                {
                    if (++c->cw_ticks > CORNER_FULL_HOLD_TICKS)
                    {
                        c->cw_wide  = 0U;               /* 超出窗口，作废 */
                        c->cw_ticks = 0U;
                    }
                }
            }
        }

#if (CORNER_TRIG_MODE == 1)
#if (CORNER_CW3_CONFIRM_TICKS > 0)
        /* [CW4 2026-11-22] 全亮后先挂起，等线完全离开阵列连续
           CORNER_CW3_CONFIRM_TICKS 拍才真正触发。
           六边形关口的"宽实心带"线不会丢 -> 等到 HOLD 窗口后作废，不会误触发；
           真直角弯全亮后线会甩出阵列 -> 只比原来晚 2 拍。 */
        if (cw_pend != 0U)
        {
            if (++cw_pend_wait > CORNER_FULL_HOLD_TICKS)
            {
                cw_pend      = 0U;
                cw_pend_wait = 0U;
                c->cw_low    = 0U;
                Cwd_Push(5U, c, cnt, mask, error);      /* [CWDBG] 候选作废 */
            }
        }
        else if (c->cw_low >= CORNER_L_FULL_TICKS)
        {
            cw_pend      = 1U;
            cw_pend_wait = 0U;
            c->cw_low    = 0U;
            Cwd_Push(4U, c, cnt, mask, error);          /* [CWDBG] 候选挂起 */
        }
#else
        /* [CW3 2026-11-21] 新模式：连续 CORNER_L_FULL_TICKS 拍拍到"一侧全亮"就触发，
           不再等线消失。触发点由几何决定（车头正对拐角那一拍），每个弯一致，
           不像旧模式那样随"什么时候丢线、丢之前采到哪一帧"漂 3~4 拍。 */
        if (c->cw_low >= CORNER_L_FULL_TICKS)
        {
            *dir_out    = c->cw_side;
            *fire       = 1U;
            c->cw_low   = 0U;
            c->cw_wide  = 0U;
            c->cw_ticks = 0U;
            ++cw_fire_cnt;
        }
#endif
#endif
        return;
    }

    /* ---- 丢线 ---- */
    if (c->cw_dark < 250U) ++c->cw_dark;
    /* 注意：cw_ticks 在这里不再累加 —— 窗口衡量的是"最后一次看到全亮
       距离线消失过了多久"，丢线之后的时间不该算进去。 */

#if (CORNER_TRIG_MODE == 1) && (CORNER_CW3_CONFIRM_TICKS > 0)
    /* [CW4] 全亮后已经丢线够拍数 -> 确认是直角弯 */
    if ((cw_pend != 0U) && (c->cw_dark >= CORNER_CW3_CONFIRM_TICKS))
    {
        *dir_out    = c->cw_side;
        *fire       = 1U;
        Cwd_Push(2U, c, cnt, mask, error);              /* [CWDBG] CW4 fire */

        cw_pend      = 0U;
        cw_pend_wait = 0U;
        c->cw_dark   = 0U;
        c->cw_wide   = 0U;
        c->cw_ticks  = 0U;
        ++cw_fire_cnt;
        return;
    }
#endif

    /* 3) 最后一条有效线是否居中（用丢线时冻结的 cw_mask 重新算一次） */
    {
        uint8_t last_centered = 0U;

        if (c->cw_mask != 0U)
        {
            uint16_t sum = 0U;
            uint8_t  n   = 0U;

            for (i = 0U; i < 12U; ++i)
                if ((c->cw_mask & (uint16_t)(1U << i)) != 0U) { sum += i; ++n; }

            if (n > 0U)
            {
                float b = (float)sum / (float)n;
                if ((b > 2.0f) && (b < 8.0f)) last_centered = 1U;
            }
        }

        if ((c->cw_dark >= CORNER_LOST_TICKS) &&
            (c->cw_wide != 0U) &&
            (c->cw_ticks <= CORNER_FULL_HOLD_TICKS) &&
            (last_centered != 0U))
        {
            *dir_out    = c->cw_side;
            *fire       = 1U;
            Cwd_Push(2U, c, cnt, mask, error);          /* [CWDBG] 旧路径 fire */
            c->cw_dark  = 0U;
            c->cw_wide  = 0U;
            c->cw_ticks = 0U;
            ++cw_fire_cnt;                      /* [CW2] 累计触发次数，供 'c' 核对 */
        }
    }
}

/* [ARC-RECOVER 2026-11-24] 用最后一条有效线的 bias 决定段 8 补转方向：
   bias < 4.5 -> 线在左 -> +1；bias > 6.5 -> 线在右 -> -1；
   中间（线基本居中）或没有有效线时退回原来的 c->dir，避免中线误翻转。 */
static float Corner_RecoverDirFromLastMask(const CornerState *c)
{
    uint16_t m = c->last_valid_mask;
    uint16_t sum = 0U;
    uint8_t  i;
    uint8_t  n = 0U;

    for (i = 0U; i < 12U; ++i)
    {
        if ((m & (uint16_t)(1U << i)) != 0U)
        {
            sum += i;
            ++n;
        }
    }

    if (n == 0U)
        return c->dir;

    {
        float bias = (float)sum / (float)n;

        if (bias < 4.5f) return  1.0f;
        if (bias > 6.5f) return -1.0f;
    }

    return c->dir;
}

/* [ARC-EXIT 2026-11-21] ARC 的出口统一到这一处：
     cnt == 0（线没回来）-> CORNER_ST_ARC_RECOVER，按最后有效线的方向继续补转；
     cnt >  0（看到线）  -> CORNER_ST_LINE，正常交回巡线。
   以前三个出口各自写一套，角度到位/超时那两路不看 cnt，没线也直接回 LINE，
   结果被丢线保护判定停车。现在三个出口全部走这里，规则只有一条。
   注意：进 ARC_RECOVER 必须把 yaw 清零，否则 arc 期间累计的 90° 会立刻
   超过 CORNER_RECOVER_DEG，补转一拍就放弃。 */
static void Corner_ArcExit(CornerState *c, int cnt,
                           uint16_t mask, uint16_t cw_mask, float error)
{
    c->arc_brake_active = 0U;   /* [ARC-BRAKE] 出 ARC 立即结束里程刹车 */
    c->arc_brake_ticks  = 0U;

    if (cnt == 0)
    {
        c->arc_recover_dir = Corner_RecoverDirFromLastMask(c);
        c->state         = CORNER_ST_ARC_RECOVER;
        c->yaw           = 0.0f;        /* 补转角从 0 重新量 */
        c->recover_ticks = 0U;
        /* [诊断] 13 = ARC 出口仍无线，开始补转（串口会打 CORN,13,...） */
        Corner_Log(13U, c->arc_recover_dir, mask, cw_mask, cnt, error);
        return;
    }

    c->state         = CORNER_ST_LINE;
    c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
    c->ramp_ticks    = CORNER_RAMP_TICKS;
    Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
}

float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint16_t mask,
                    uint8_t gyro_ok,
                    float gyro_z,
                    float speed_meas,
                    uint8_t flow_fire,
                    float flow_dir,
                    float *speed_cmd)
{
    /* [CWDBG 2026-11-22] 丢线沿快照：把 CW 的全部内部状态打出来 */
    {
        static uint8_t cwdbg_prev_cnt;

        if ((cnt == 0) && (cwdbg_prev_cnt > 0U))
            Cwd_Push(1U, c, cnt, mask, error);
        cwdbg_prev_cnt = (uint8_t)cnt;
    }

    /* [FLOW-OR 2026-11-23] 新策略优先：FLOW 先到就直接进 ARC。
       老 CW4 在下面继续作为兜底；两套由 CORNER_TRIGGER_* 独立开关。 */
#if (CORNER_TRIGGER_FLOW == 1)
    if ((flow_fire != 0U) &&
        (Corner_IsActive(c) == 0U) &&
        (Corner_IsCooling(c) == 0U) &&
        (hex_zone_flag == 0U))
    {
        /* 清掉 CW4 的候选状态，避免同一弯下一拍又被老策略触发一次 */
        cw_pend      = 0U;
        cw_pend_wait = 0U;
        c->cw_low    = 0U;
        c->cw_wide   = 0U;
        c->cw_ticks  = 0U;
        c->cw_dark   = 0U;

        Corner_Log(15U, flow_dir, mask, c->cw_mask, cnt, error);
        Corner_Enter(c, flow_dir);
        *speed_cmd = c->speed_ref;
        return 0.0f;
    }
#else
    (void)flow_fire;
    (void)flow_dir;
#endif

    /* [ARC-RECOVER 2026-11-24] 记录最后一条有效线：ARC 收尾没线时，
       用它的 bias 决定段 8 补转方向，而不是盲目沿用 c->dir。 */
    if (cnt > 0)
        c->last_valid_mask = mask;

    /* [FULLDIST 2026-11-24] 记录“本次运行的第一个全亮”的里程：
       - 只记第一次出现的 >=10 路亮，之后不再重置；
       - 线恢复后再全亮也不改起点，FULLDIST 保持单调累加；
       - 距离 = corner_odom_dist - cw_full_dist，由主循环打印。
         corner_odom_dist 是绝对里程，不会因 Corner_Enter 把 c->dist 清 0
         而变成 2^32 附近的负数。
       这样 FULLDIST 是“从本次第一个全亮点开始”的固定里程，不会因为
       中途又出现一次全亮而清零，硬编码区间不会误伤到后面路段。 */
    if (cnt >= 10)
    {
        if (cw_full_active == 0U)
        {
            cw_full_active = 1U;

            if (cw_full_valid == 0U)
            {
                cw_full_valid = 1U;
                cw_full_dist  = corner_odom_dist;
            }
        }
    }
    else
    {
        cw_full_active = 0U;
    }

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
            /* [RECOVER 2026-11-16] 转完 90° 还看不到线 -> 反方向找
               （方向判错 / 冲过头 时的兜底；找到就交回巡线，找不到才放弃）
               [ARC-EXIT 2026-11-21] 上限随 CORNER_RECOVER_DEG 一起提到 90° */
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

    /* ---- 段 6：固定外轮弧线（[ARC2 2026-11-21] CORNER_MODE=1）
       下面三行旧注释保留作历史，实际实现已改成"固定外轮 + 内轮 0"：
       不停车，用陀螺双环把车头转过 ARC_YAW_DEG：
         外环：ω_des = K_YAW·(目标角 − 当前角)，限幅 ±200°/s
         内环：turn  = ω_des/1.644 + K_OMEGA·(ω_des − ω)
       全程 speed = BASE，turn ≤ speed（内轮最低 0，不反转）。 */
    if (c->state == CORNER_ST_ARC)
    {
        float turn_r;

        ++c->arc_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        /* [ARC2 2026-11-21] 外轮保持、内轮主动反转：
             speed = (外 + 内)/2,  turn = (外 - 内)/2
           -> 外轮 = speed+turn = ARC_OUTER_SPEED（不变速）
              内轮 = speed-turn = ARC_INNER_SPEED（负 = 主动倒转）
           内轮不再靠摩擦"刹到 0"（地面拖拽比反向脉冲大，刹不住），
           而是让电机主动倒转抵消拖拽 -> 不再受电压/地面影响。
           300 / -70 时：Δv = 370 counts/10ms -> ω = 304°/s，90° 约 0.30 s，半径约 5 cm。 */
        *speed_cmd = (ARC_OUTER_SPEED + ARC_INNER_SPEED) * 0.5f;
        turn_r     = (ARC_OUTER_SPEED - ARC_INNER_SPEED) * 0.5f;

#if (ARC_RATE_LOOP_ENABLE == 1)
        /* [ARC-RATE 2026-11-25] 陀螺角速度内环：只改差速，不改中心速度。
           w_des 用计划差速反算；实测 w = dir×gyro_z（带方向）。
           先限幅再限斜率：打滑时最多多给 ±TRIM_MAX，不会把差速推爆。 */
        if (gyro_ok)
        {
            float w_des = turn_r * OMEGA_PER_TURN;
            float trim  = ARC_RATE_KP * (w_des - (float)c->dir * gyro_z);

            if (trim >  ARC_RATE_TRIM_MAX) trim =  ARC_RATE_TRIM_MAX;
            if (trim < -ARC_RATE_TRIM_MAX) trim = -ARC_RATE_TRIM_MAX;
            if (trim > arc_rate_trim + ARC_RATE_TRIM_SLEW)
                trim = arc_rate_trim + ARC_RATE_TRIM_SLEW;
            if (trim < arc_rate_trim - ARC_RATE_TRIM_SLEW)
                trim = arc_rate_trim - ARC_RATE_TRIM_SLEW;

            arc_rate_trim = trim;
            turn_r += trim;
        }
        else
        {
            arc_rate_trim = 0.0f;
        }
#endif
        /* 出口①：出弯角度到了（提前 ARC_LEAD_DEG 断电，让惯性把剩下的几度走完）。
           304°/s 下 90° 只要 0.30 s，断电后还会滑转 5~8°，所以必须提前。
           陀螺掉了就没有角度反馈，用 40 拍（0.4 s ≈ 122°）兜底。
           [ARC-EXIT 2026-11-21] 角度到位分两条：
             - 陀螺在 且 开了收尾 -> 先进 ARC_SETTLE 闭环补角；
             - 陀螺掉了 / 收尾关掉 -> 走统一出口（cnt==0 去补转，cnt>0 才回 LINE）。 */
        if ((gyro_ok && ((c->dir * c->yaw) >= (ARC_YAW_DEG - ARC_LEAD_DEG))) ||
            ((gyro_ok == 0U) && (c->arc_ticks >= 40U)))
        {
#if (ARC_SETTLE_ENABLE == 1)
            /* [ARC2] 断电后先进航向收尾：滑行多少由它闭环补回来。
               陀螺掉了没法闭环，走下面的统一出口。 */
            if (gyro_ok)
            {
                c->state        = CORNER_ST_ARC_SETTLE;
                c->settle_ticks = 0U;
                c->arc_brake_active = 0U;   /* [ARC-BRAKE] 进入收尾，结束入口刹车 */
                c->arc_brake_ticks  = 0U;
                /* [诊断] 11 = 进航向收尾（串口会打 CORN,11,...） */
                Corner_Log(11U, c->dir, mask, c->cw_mask, cnt, error);
                return 0.0f;
            }
#endif
            Corner_ArcExit(c, cnt, mask, c->cw_mask, error);
            return 0.0f;
        }

        /* 出口②：线提前回来（细且居中）也可以交回。
           [FIX 2026-11-21] 但必须已经转过大半，否则会"提前收手"：
           车转到 ~45° 时，阵列（前伸 10 cm）刚好扫到新的一段线、并且是居中的，
           连续 5 拍满足就退出了 —— 表现出来就是"直角弯只打了 45°"。
           实测：六边形出口后的直角弯、45° 接直角的路口都是这个现象。
           所以加一个"至少转过 ARC_YAW_DEG 的 80%"的条件。
           [ARC-EXIT 2026-11-21] 这里 cnt 必然 > 0，统一出口会回 LINE。 */
        if ((cnt > 0) && (error > -2.0f) && (error < 2.0f) &&
            (gyro_ok == 0U || ((c->dir * c->yaw) >= (ARC_YAW_DEG * 0.8f))))
        {
            if (c->arc_line_ticks < 250U) ++c->arc_line_ticks;
            if (c->arc_line_ticks >= 5U)
            {
                Corner_ArcExit(c, cnt, mask, c->cw_mask, error);
                return 0.0f;
            }
        }
        else
        {
            c->arc_line_ticks = 0U;
        }

        /* 出口③：拍数兜底。
           [ARC-EXIT 2026-11-21] 同样走统一出口：1 s 到了但线还没回来，
           不该直接回 LINE 被丢线保护停车，而是先同向补转找线。 */
        if (c->arc_ticks >= ARC_MAX_TICKS)
        {
            Corner_ArcExit(c, cnt, mask, c->cw_mask, error);
            return 0.0f;
        }

        return c->dir * turn_r;
    }

    /* ---- 段 7：弧线断电后的航向收尾（[ARC2 2026-11-21]）
       ARC 断电时车还在以 ~300°/s 旋转，断电后靠惯性滑转。
       滑多少取决于断电瞬间的角速度——实测同一段里 0~15° 都有，
       固定提前量（ARC_LEAD_DEG）治不了这个方差。
       这里用陀螺闭环把车头补到 ARC_YAW_DEG ± ARC_SETTLE_TOL，再交回巡线。
       回滚：把 ARC_SETTLE_ENABLE 置 0，ARC 退出直接回巡线。 */
    if (c->state == CORNER_ST_ARC_SETTLE)
    {
        float err_deg;
        float turn;

        ++c->settle_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        *speed_cmd = ARC_SETTLE_SPEED;

        /* 还要往弯的方向转多少度（正 = 转不够，负 = 转过头了） */
        err_deg = ARC_YAW_DEG - (c->dir * c->yaw);

        /* 到位：角度进容差 且 已经停住 */
        if (gyro_ok &&
            (err_deg <  ARC_SETTLE_TOL) && (err_deg > -ARC_SETTLE_TOL) &&
            (gyro_z  <  ARC_SETTLE_RATE) && (gyro_z  > -ARC_SETTLE_RATE))
        {
            /* [ARC2 2026-11-21] 角度到了但线还没回来 -> 继续朝原方向补转，
               最多 CORNER_RECOVER_DEG(90°)；看到线立刻交回方向环。
               [ARC-EXIT 2026-11-21] 与 ARC 的三个出口共用同一条规则：
               cnt==0 -> ARC_RECOVER，cnt>0 -> LINE。 */
            if (cnt == 0)
            {
                c->arc_recover_dir = Corner_RecoverDirFromLastMask(c);
                c->state         = CORNER_ST_ARC_RECOVER;
                c->yaw           = 0.0f;
                c->recover_ticks = 0U;
                /* [诊断] 12 = 角度到了但没线，开始按最后有效线方向补转 */
                Corner_Log(12U, c->arc_recover_dir, mask, c->cw_mask, cnt, error);
                return 0.0f;
            }

            Corner_ArcExit(c, cnt, mask, c->cw_mask, error);
            return 0.0f;
        }

        if (c->settle_ticks >= ARC_SETTLE_MAX_TICKS)      /* 兜底 */
        {
            /* [ARC-EXIT 2026-11-21] 补角超时也按 cnt 分流：
               还是没线就别回 LINE 等丢线保护停车，先去同向补转找线。 */
            Corner_ArcExit(c, cnt, mask, c->cw_mask, error);
            return 0.0f;
        }

        /* PD：P 补角度，D 压住进收尾那一刻还在的旋转（~300°/s）。
           只用 P 的话，进收尾时 err 还是正的，会继续往弯的方向加舵、转得更多。 */
        turn = ARC_SETTLE_K * err_deg - ARC_SETTLE_KD * (c->dir * gyro_z);
        if (turn >  ARC_SETTLE_TURN_MAX) turn =  ARC_SETTLE_TURN_MAX;
        if (turn < -ARC_SETTLE_TURN_MAX) turn = -ARC_SETTLE_TURN_MAX;

        return c->dir * turn;
    }

    /* ---- 段 8：角度到了但线没回来，朝最后有效线方向补转（[ARC-RECOVER]） ----
       典型场景：45° 之后紧接直角弯，进弯时车还在转，90° 转完线仍在车头外侧。
       这里原地补转，方向由最后一条有效线的 bias 决定（线左 -> 左，线右 -> 右），
       不再盲目沿用 c->dir；最多 CORNER_RECOVER_DEG(90°)，一看到线就交回。 */
    if (c->state == CORNER_ST_ARC_RECOVER)
    {
        float ya;
        float rd;

        ++c->recover_ticks;
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        ya = (c->yaw >= 0.0f) ? c->yaw : -c->yaw;
        rd = (c->arc_recover_dir != 0.0f) ? c->arc_recover_dir : c->dir;

        if (cnt > 0)                          /* 找到线：交给方向环 */
        {
            c->state = CORNER_ST_LINE;
            c->cooldown_dist = c->dist + CORNER_COOLDOWN_COUNTS;
            c->ramp_ticks = CORNER_RAMP_TICKS;
            corner_gave_up = 0U;
            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        if ((ya >= CORNER_RECOVER_DEG) || (c->recover_ticks >= CORNER_RECOVER_TICKS))
        {
            c->state = CORNER_ST_LINE;        /* 补满 20° 还是没有：放弃，交给丢线保护 */
            c->cooldown_dist = c->dist;
            c->ramp_ticks = 0U;
            corner_gave_up = 1U;              /* [LOST3] control 看到就直接 LOST_STOP */
            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        *speed_cmd = 0.0f;                    /* 原地转，不前进 */
        return rd * PIVOT_TARGET;
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
            corner_gave_up = 0U;

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
            corner_gave_up   = 1U;           /* [LOST3] 反向找线 20° 也没线 */

            Motor_SetSpeedGains(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
            return 0.0f;
        }

        turn = c->recover_dir * PIVOT_TARGET;
        return turn;
    }

    /* ---- 正常巡线 ---- */
    /* [COOLDOWN 2026-11-18] 冷却改成距离制，这里不再递减计数 */

    /* [LOST3] 回到线上就清"补转放弃"标志 */
    if (cnt > 0)
        corner_gave_up = 0U;

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
    if ((hex_zone_flag == 0U) && (c->state == CORNER_ST_LINE))
    {
        float   cw_dir = 0.0f;
        uint8_t cw_fire = 0U;
        uint8_t cw_gave_up = 0U;

        Corner_Watch(c, mask, cnt, error, &cw_dir, &cw_fire, &cw_gave_up);

        /* [CW5 2026-11-22] 不再要求 c->armed：
           CW4 已经是"一侧全亮 2 拍 + 全丢线 2 拍"的双重确认，比单靠武装可靠；
           连续直角弯只隔 ~6.7 cm，15 拍武装窗口根本来不及，第二个弯会被 BLKARM 挡掉
           （实测 a连续直角弯1：第一个弯后 10~12 拍就到第二个弯的全亮）。
           armed 只保留给已停用的通用锁存（#if 0），CW 入口不再看它。 */
        if ((CORNER_TRIGGER_CW4 != 0) &&
            (cw_fire != 0U) && (Corner_IsCooling(c) == 0U))
        {
            Cwd_Push(3U, c, cnt, mask, error);                      /* [CWDBG] 进 corner */
            ++cw_enter_cnt;                                        /* [CW2] 真的进去了 */
            Corner_Log(1U, cw_dir, mask, c->cw_mask, cnt, error);   /* [CORN] 1 = 直角弯检测触发 */
            Corner_Enter(c, cw_dir);
            *speed_cmd = c->speed_ref;
            return 0.0f;
        }
        else if ((CORNER_TRIGGER_CW4 != 0) && (cw_fire != 0U))
        {
            /* [CW5] armed 不再参与判断，只统计冷却挡下的次数 */
            Cwd_Push(6U, c, cnt, mask, error);                      /* [CWDBG] 被冷却挡 */
            if (Corner_IsCooling(c) != 0U)  ++cw_blk_cool;
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

        /* [CW2 2026-11-21] 通用锁存也停用：现在"进直角弯状态机"只有一条入口 ——
           线完全离开阵列满 CORNER_LOST_TICKS 拍（上面的 Corner_Watch）。
           原来的"误差大 + 5 路亮"在 45°/60° 上同样成立，等于又开了一个误触发口子。
           保留 prev_error 的赋值，别的分支还在读它。 */
#if 0
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
#endif
        (void)abs_err; (void)abs_turn;     /* 停用后避免 unused 警告 */
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

        /* [HEX-FAST 2026-11-22] 六边形区间限速：60° 顶点主要靠方向环硬拉，
           速度越高越容易把线甩出阵列（出口 V 就是这样丢的）。
           想关掉就把 HEX_ZONE_SPEED_MAX 设成 BASE_SPEED。 */
        if ((hex_zone_flag != 0U) && (*speed_cmd > HEX_ZONE_SPEED_MAX))
            *speed_cmd = HEX_ZONE_SPEED_MAX;
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
    Hex4_Reset();          /* [HEX4 2026-11-21] 新状态机也跟着清 */

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
       c = ch5 || ch6           正中（[V-ONE 2026-11-23] 两路似乎严苛，尝试只看一路）
       一次 V = 某侧 a 亮 && c 亮 && 同侧 b 不完整
       两次 V（同侧、间隔 1~4 cm）= 六边形入口                              */
    {
        uint8_t a_l = (uint8_t)((mask & 0x0003U) != 0U);        /* ch0,ch1    */
        uint8_t a_r = (uint8_t)((mask & 0x0C00U) != 0U);        /* ch10,ch11  */
        uint8_t c_s = (uint8_t)(center != 0U);                  /* ch5 || ch6（[V-ONE] 尝试只看一路） */
        uint8_t c_l = (uint8_t)((mask & 0x00F0U) != 0U);        /* ch4~ch7 任一路（松） */

        uint8_t b_l_n = (uint8_t)(((mask >> 2) & 1U) + ((mask >> 3) & 1U) + ((mask >> 4) & 1U));
        uint8_t b_r_n = (uint8_t)(((mask >> 7) & 1U) + ((mask >> 8) & 1U) + ((mask >> 9) & 1U));

        /* [HEX-V 2026-11-15] 两次判据：
           · 武装（第一次 V）：端点亮 && 正中 ch5||ch6 && 同侧 b 有缺口（严格，防误武装）
           · 触发（第二次）：**正中 ch5||ch6 && 武装侧 b 任意一路为空**，且落在距离窗口内。
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


/* ================= [HEX4 2026-11-21] 六边形：两次 V 判据（新写） =================
   按现场描述重写，和老 Hex_Update 的区别：
     老：第一次 V 就入栈；之后**不看 V**，走够 10 cm 就原地转 30°；出口不打舵。
     新：第一次 V 只"武装"并回正；必须在 10~20 cm 内真的再看到**同侧** V
         且大误差方向一致才进入；出口净角到 320~340° 强制出栈，按入口同方向打 30°。
   状态：0=IDLE 1=ARMED 2=TURN_IN 3=DRIVING 4=TURN_OUT 5=TURN_BACK
   事件码（打印 HEX,<event>）：4=武装 5=进入 6=进弯转完 8=解除 9=出口 10=出弯转完
                              12=DRIVING 中看到 V（第二字段 = 当前净角）
                              13=V1 后的 V（第二字段 = 距 V1 的里程，手推诊断用）
                              14=异侧 V 否决（HEX4_OPP_VETO=1 时）
                              17=净角到 320° 强制出栈
                              18=反向回落 >120° 且回 ±20° 强制出栈
                              19=ARMED 时 FLOW 同侧直角弯否决六边形入口
                              20=TURN_IN 转完或入口窗口内没线，开始反向转回
                              21=反向转回看到线，标记清零后交回方向环
                              22=反向转回没线（转满放弃），标记清零后交回方向环 */
static uint8_t  hx_state;        /* 0=IDLE 1=ARMED 2=TURN_IN 3=DRIVING 4=TURN_OUT 5=TURN_BACK */
static int8_t   hx_dir;          /* 记录的转向方向：+1 左 / -1 右 */
static int8_t   hx_err1;         /* V1 时的大误差方向（可能为 0） */
static int8_t   hx_side;         /* 当前 V 的侧别 */
static uint8_t  hx_vprev;        /* 上一拍有没有 V（算上升沿） */
static uint8_t  hx_vrun;         /* V 去抖：连续同侧拍数（本文件私有，不动结构体） */
static int8_t   hx_vrun_side;
static uint8_t  hx_vl_raw_prev;  /* [V-WIDE 2026-11-22] 上一拍 v_l 原始判据（1 拍展宽用） */
static uint8_t  hx_vr_raw_prev;  /* [V-WIDE 2026-11-22] 上一拍 v_r 原始判据 */
static uint32_t hx_v1_dist;      /* V1 的里程 */
static uint32_t hx_t0;           /* 打舵起始里程 */
static uint32_t hx_exit_lock_dist;/* [2026-11-24] 出栈后禁止入栈到该里程；0 = 无锁 */
static float    hx_yaw;          /* 本次打舵累计角度 */
static float    hx_total;        /* 绕六边形累计转角（绝对值，诊断用） */
static float    hx_net;          /* [HEX4-NET] 有方向净转角，从入口 30° 开始累计 */
static float    hx_net_peak;     /* [HEX4-NET] hx_net 的峰值，用于反向回落判断 */
#if (HEX4_TURNBACK_ENABLE == 1)
static float    hx_turnback_deg; /* [HEX4-TURNBACK] 反向转回目标角 = TURN_IN 实际转过的角 */
#endif
static uint8_t  hx_turnback_done;/* [HEX4-TURNBACK] 1 = 本拍刚结束反向转回（给 control 清 HOLD 用） */
#if (HEX4_TURNBACK_ENABLE == 1)
static uint8_t  hx_drive_lost;   /* [HEX4-TURNBACK] 入口后连续没线的拍数（只在入口窗口内累计） */
#endif
static uint16_t hx_straight;     /* 回正剩余拍数 */
static float    hx_turn_at_v1;   /* V1 那一拍方向环的舵量（回正用） */
static uint16_t hx_seq_last;     /* 上一次已入队的事件序号（用于把事件推进环形缓冲） */
#if (HEX4_ENTRY_ONLY == 1)
static uint32_t hx_lockout_dist; /* [HEX4-ENTRYONLY] 入口完成后禁止再次识别 V 的里程 */
#endif
#if (HEX4_FLOW_VETO == 1)
static uint8_t hx_flow_veto_latch; /* [HEX4-FLOW-VETO] 剩余锁存拍数 */
static int8_t  hx_flow_veto_dir;   /* [HEX4-FLOW-VETO] 锁存的 FLOW 方向 */
#endif

/* ---- [HEX4 2026-11-21] 事件环形缓冲：主循环按序打印，不再"只打最新一条" ----
   原来 main.c 用"快照 + 序号"打印，两次事件之间主循环没跑到就会丢前面的。
   现在中断里入队，主循环取空为止；按 'c' 转储 0.22 s 也不会吞事件。 */
#define HEXEVT_MAX 16U
typedef struct {
    uint8_t  code;
    uint32_t dist;
    uint16_t mask;
    uint8_t  right;
    uint8_t  left;
    uint8_t  center;
    uint16_t seq;
} HexEvent;

static HexEvent hev[HEXEVT_MAX];
static volatile uint8_t hev_head;
static volatile uint8_t hev_tail;

static void HexEvent_Push(uint8_t code, uint32_t dist, uint16_t mask,
                          uint8_t right, uint8_t left, uint8_t center, uint16_t seq)
{
    uint8_t nxt = (uint8_t)((hev_head + 1U) % HEXEVT_MAX);

    if (nxt == hev_tail)
        return;                     /* 满：丢新事件，绝不覆盖未打印的 */

    hev[hev_head].code   = code;
    hev[hev_head].dist   = dist;
    hev[hev_head].mask   = mask;
    hev[hev_head].right  = right;
    hev[hev_head].left   = left;
    hev[hev_head].center = center;
    hev[hev_head].seq    = seq;
    hev_head = nxt;
}

uint8_t Hex_PopEvent(uint8_t *code, uint32_t *dist, uint16_t *mask,
                     uint8_t *right, uint8_t *left, uint8_t *center, uint16_t *seq)
{
    if (hev_tail == hev_head)
        return 0U;

    *code   = hev[hev_tail].code;
    *dist   = hev[hev_tail].dist;
    *mask   = hev[hev_tail].mask;
    *right  = hev[hev_tail].right;
    *left   = hev[hev_tail].left;
    *center = hev[hev_tail].center;
    *seq    = hev[hev_tail].seq;

    hev_tail = (uint8_t)((hev_tail + 1U) % HEXEVT_MAX);
    return 1U;
}

/* ---- [HEX4] V 检测的逐拍记录：用来重新设计 V 判据（按 'h' 打印） ----
   每拍 4 个数：cnt / mask / error×10 / 标志
     标志 bit0 = v_l 成立, bit1 = v_r 成立, bit2 = 去抖后 s!=0, bit3 = 进入 DRIVING */
#define HEXLOG_MAX 128U
static int16_t  hxlog_cnt[HEXLOG_MAX];
static uint16_t hxlog_mask[HEXLOG_MAX];
static int16_t  hxlog_err[HEXLOG_MAX];
static uint8_t  hxlog_flag[HEXLOG_MAX];
static uint8_t  hxlog_head;
static uint8_t  hxlog_num;
static volatile uint8_t hxlog_pause;
static uint8_t  hxlog_after;     /* [HEX4] 事件后还要再记几拍就冻结（示波器式捕获） */

static void HexLog_Push(int cnt, uint16_t mask, float error, uint8_t flag)
{
    if (hxlog_pause != 0U)
        return;

    hxlog_cnt[hxlog_head]  = (int16_t)cnt;
    hxlog_mask[hxlog_head] = mask;
    hxlog_err[hxlog_head]  = (int16_t)(error * 10.0f);
    hxlog_flag[hxlog_head] = flag;

    hxlog_head = (uint8_t)((hxlog_head + 1U) % HEXLOG_MAX);
    if (hxlog_num < HEXLOG_MAX) ++hxlog_num;
}

uint8_t HexLog_Count(void)
{
    return hxlog_num;
}

void HexLog_GetAt(uint8_t i, int16_t *cnt, uint16_t *mask, int16_t *err, uint8_t *flag)
{
    uint8_t k;

    if (i >= hxlog_num)
    {
        *cnt = -1; *mask = 0U; *err = 0; *flag = 0U;
        return;
    }

    k = (uint8_t)((hxlog_head + HEXLOG_MAX - hxlog_num + i) % HEXLOG_MAX);
    *cnt  = hxlog_cnt[k];
    *mask = hxlog_mask[k];
    *err  = hxlog_err[k];
    *flag = hxlog_flag[k];
}

void HexLog_Clear(void)
{
    hxlog_pause = 1U;
    hxlog_head  = 0U;
    hxlog_num   = 0U;
    hxlog_after = 0U;
    hxlog_pause = 0U;
}

void Hex4_Reset(void)
{
    hx_state      = 0U;
    hx_dir        = 0;
    hx_err1       = 0;
    hx_side       = 0;
    hx_vprev      = 0U;
    hx_vrun       = 0U;
    hx_vrun_side  = 0;
    hx_vl_raw_prev = 0U;    /* [V-WIDE] */
    hx_vr_raw_prev = 0U;
    hx_v1_dist    = 0U;
    hx_t0         = 0U;
    hx_exit_lock_dist = 0U;  /* [2026-11-24] */
    hx_yaw        = 0.0f;
    hx_total      = 0.0f;
    hx_net        = 0.0f;
    hx_net_peak   = 0.0f;
#if (HEX4_TURNBACK_ENABLE == 1)
    hx_turnback_deg  = 0.0f;
#endif
    hx_turnback_done = 0U;
#if (HEX4_TURNBACK_ENABLE == 1)
    hx_drive_lost    = 0U;
#endif
    hx_straight   = 0U;
    hx_turn_at_v1 = 0.0f;
    hx_seq_last   = 0U;
#if (HEX4_ENTRY_ONLY == 1)
    hx_lockout_dist = 0U;
#endif
#if (HEX4_FLOW_VETO == 1)
    hx_flow_veto_latch = 0U;
    hx_flow_veto_dir   = 0;
#endif
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
    d->dist = h->dist;
    d->arm = (uint16_t)h->v_prev;

    /* [HEXDBG 2026-11-22] 前三列原来是 0，现在借用来在线看状态机：
       event_cnt = hx_state（0..5）、off_r = hx_total（绝对值）、
       off_l = hx_vrun，net10 = hx_net（有方向净角 ×10）。
       串口 HEXSTAT 行可直接看到净角有没有累计、有没有反向回落。 */
    d->event_cnt = (uint16_t)hx_state;
    d->off_r     = (uint16_t)hx_total;
    d->off_l     = (uint16_t)hx_vrun;
    d->net10     = (int16_t)(hx_net * 10.0f);
}

/* [HEX-EXIT 2026-11-23] 六边形入口/出口时清掉 CW4 候选的挂起状态。
   否则六边形期间 Corner_Watch 被 hex_zone_flag 挡住，旧 cw_pend 会留到出栈，
   出栈后第一个直角弯可能被这个旧候选挡住。 */
static void Hex_ClearCornerWatch(CornerState *c)
{
    cw_pend      = 0U;
    cw_pend_wait = 0U;
    c->cw_state  = 0U;
    c->cw_low    = 0U;
    c->cw_wide   = 0U;
    c->cw_ticks  = 0U;
    c->cw_dark   = 0U;
}

/* [ARMED-COAST 2026-11-27] 是否处于 ARMED（看到第一次 V、走 9 cm 等确认）。
   只暴露状态号，不给 control 任何别的信息：它需要知道的仅仅是
   "现在还是方向环在开车，六边形没有接管"（详见 special.h 的声明注释）。
   放在这里而不是 Hex_ZoneActive 旁边，是因为 hx_state 在本文件 1806 行才声明。 */
uint8_t Hex_IsArmed(void)
{
    return (uint8_t)((hx_state == 1U) ? 1U : 0U);
}

#if (HEX4_TURNBACK_ENABLE == 1)
/* [HEX4-TURNBACK 2026-11-25] 本拍有没有看到线：用过滤后的 mask 判。
   单路坏点已经在 Sensor_Scan 里被清成"无线"，所以 mask != 0 就代表至少 2 路压线，
   不会因为一路悬空/漏读就把反向转回提前结束。 */
static uint8_t Hex_LineSeen(uint16_t mask)
{
    return (uint8_t)((mask != 0U) ? 1U : 0U);
}

/* [HEX4-TURNBACK 2026-11-25] 反向转回结束（看到线 / 转满放弃）：
   六边形标记全部清零，交回方向环；HOLD 逻辑本身不动，只把"刚结束"这件事
   告诉 control（Hex_TakeTurnBackDone），让它在同一拍把旧的 HOLD/补转舵清掉，
   避免反向转回刚结束又被 HOLD 按旧方向打回去。
   lock_until = 结束里程 + 锁存距离：车是掉头往回走的，入口那个 V 马上又会
   出现在阵列里，不锁就会 TURN_IN -> TURN_BACK 死循环。 */
static void Hex_TurnBackExit(HexState *h, CornerState *corner, uint32_t lock_until)
{
    hx_state       = 0U;
    hx_dir         = 0;
    hx_err1        = 0;
    hx_side        = 0;
    hx_vprev       = 0U;
    hx_vrun        = 0U;
    hx_vrun_side   = 0;
    hx_vl_raw_prev = 0U;
    hx_vr_raw_prev = 0U;
    hx_v1_dist     = 0U;
    hx_t0          = 0U;
    hx_yaw         = 0.0f;
    hx_total       = 0.0f;
    hx_net         = 0.0f;
    hx_net_peak    = 0.0f;
    hx_straight    = 0U;
    hx_turn_at_v1  = 0.0f;
    hx_turnback_deg  = 0.0f;
    hx_turnback_done = 1U;
    hx_drive_lost    = 0U;

    /* 老 Hex_Update 的"在栈"标记也一起清，避免 HEXSTAT / 后续逻辑残留 */
    h->zone       = 0U;
    h->v_stack    = 0U;
    h->hex_yaw    = 0.0f;
    h->half_done  = 0U;
    h->full_done  = 0U;
    h->zone_dist  = 0U;
    h->v_active   = 0U;
    h->v_yaw      = 0.0f;
    h->v_prev     = 0U;
    h->event_cnt  = 0U;
    h->arm_ticks  = 0U;
    h->turn_ticks = 0U;

    hex_zone_flag     = 0U;
    hx_exit_lock_dist = lock_until;

#if (HEX4_ENTRY_ONLY == 1)
    hx_lockout_dist = 0U;
#endif
#if (HEX4_FLOW_VETO == 1)
    hx_flow_veto_latch = 0U;
    hx_flow_veto_dir   = 0;
#endif

    Hex_ClearCornerWatch(corner);
}
#endif /* HEX4_TURNBACK_ENABLE */

/* [HEX4-TURNBACK 2026-11-25] 一次性取"反向转回刚结束"：control 在 Hex_Update4
   之后调用，用它在同一拍清掉 HOLD/补转的旧舵。功能关闭时恒为 0。 */
uint8_t Hex_TakeTurnBackDone(void)
{
    uint8_t done = hx_turnback_done;

    hx_turnback_done = 0U;
    return done;
}

HexOutput Hex_Update4(HexState *h,
                      CornerState *corner,
                      uint8_t center,
                      uint8_t right,
                      uint8_t left,
                      uint16_t mask,
                      uint8_t gyro_ok,
                      float gyro_z,
                      float error,
                      uint8_t flow_fire,
                      float flow_dir,
                      float pid_turn)
{
    HexOutput out = { 0.0f, 0.0f, 0U };
    uint8_t  a_l, a_r, c_s;
    uint8_t  b_l_n, b_r_n;
    uint8_t  v_l, v_r;
    uint8_t  rising;
    int8_t   s;
    int8_t   err_dir;

    h->dbg_right  = right;
    h->dbg_left   = left;
    h->dbg_center = center;
    h->dbg_mask   = mask;

#if (HEX4_FLOW_VETO == 0)
    (void)flow_fire;
    (void)flow_dir;
#endif

    if (HEX_MODE == 0U)
        return out;

#if (HEX4_FLOW_VETO == 1)
    /* [HEX4-FLOW-VETO] 默认 HEX4_FLOW_VETO=0，本段不编译，仅作回滚参考。
       若以后恢复：把 FLOW 触发锁存 N 拍，且放在所有提前 return 之前，
       避免 ARC 期间计数器冻结、结束后留下旧方向锁存。 */
    if ((flow_fire != 0U) && (hx_state <= 2U))
    {
        hx_flow_veto_latch = HEX4_FLOW_VETO_LATCH_TICKS;
        hx_flow_veto_dir   = (int8_t)flow_dir;
    }
    else if (hx_flow_veto_latch > 0U)
    {
        --hx_flow_veto_latch;
    }
#endif

    /* 过弯状态机运行时六边形完全不跑（pivot 会灌假里程） */
    if ((Corner_IsActive(corner) != 0U) || (Corner_IsCooling(corner) != 0U))
    {
        hex_zone_flag = 0U;
        hx_vl_raw_prev = 0U;    /* [V-WIDE] 别把过弯前的 V 带到恢复后的第一拍 */
        hx_vr_raw_prev = 0U;
        return out;
    }

#if (HEX4_ENTRY_ONLY == 1)
    /* [HEX4-ENTRYONLY] 入口 30° 完成后先锁一段里程，避免刚打完 30°
       又被同一个六边形的 V 重新入口；锁定期内完全交回方向环。 */
    if (hx_lockout_dist != 0U)
    {
        if (h->dist < hx_lockout_dist)
        {
            hex_zone_flag = 0U;
            hx_vprev       = 0U;
            hx_vrun        = 0U;
            hx_vl_raw_prev = 0U;
            hx_vr_raw_prev = 0U;
            return out;
        }

        hx_lockout_dist = 0U;
    }
#endif

    /* ---- V 判据：a=两端, c=正中(ch5||ch6), b=中段三路，缺口=亮的路数 <= HEX_V_B_MAX ----
       [HEX4 2026-11-21] 端部判据可选"两路都亮"：
       单路假亮（ch10 悬空/漏读）是原版误触发的主要来源，
       要求 ch0&ch1 或 ch10&ch11 同时亮，能挡掉绝大部分单路毛刺。
       [V-ONE 2026-11-23] 两路似乎严苛，尝试只看一路：
       HEX_V_STRICT_END=0 -> a_l=ch0||ch1, a_r=ch10||ch11；
       c_s 同步改成 ch5||ch6（在 sensor.c 的 center 里）。
       回滚：HEX_V_STRICT_END 改 1，sensor.c 的 center 改回 &&。 */
#if (HEX_V_STRICT_END == 1)
    a_l   = (uint8_t)((mask & 0x0003U) == 0x0003U);
    a_r   = (uint8_t)((mask & 0x0C00U) == 0x0C00U);
#else
    a_l   = (uint8_t)((mask & 0x0003U) != 0U);
    a_r   = (uint8_t)((mask & 0x0C00U) != 0U);
#endif
    c_s   = (uint8_t)(center != 0U);
    b_l_n = (uint8_t)(((mask >> 2) & 1U) + ((mask >> 3) & 1U) + ((mask >> 4) & 1U));
    b_r_n = (uint8_t)(((mask >> 7) & 1U) + ((mask >> 8) & 1U) + ((mask >> 9) & 1U));

    v_l = (uint8_t)(a_l && c_s && (b_l_n <= HEX_V_B_MAX));
    v_r = (uint8_t)(a_r && c_s && (b_r_n <= HEX_V_B_MAX));

    /* [V-WIDE 2026-11-22] 先做 1 拍展宽，再做 2 拍去抖：
       六边形顶点在 300 速度下 V 形态实测只持续 1 拍（3184 那一拍），
       原来直接连续 2 拍去抖会漏掉，状态机一直不武装。
       这里把"本拍是 V"扩展成"本拍和下一拍都算 V"，正好满足
       HEX_V_DEBOUNCE=2；2 拍以上的真 V 只是多算 1 拍，不影响。 */
    {
        uint8_t vl_w = (uint8_t)(v_l || hx_vl_raw_prev);
        uint8_t vr_w = (uint8_t)(v_r || hx_vr_raw_prev);

        hx_vl_raw_prev = v_l;
        hx_vr_raw_prev = v_r;
        v_l = vl_w;
        v_r = vr_w;
    }

    s = (int8_t)((v_l != v_r) ? (v_l ? 1 : -1) : 0);

    /* 去抖：连续 HEX_V_DEBOUNCE 拍同侧才算 */
    if (s != 0)
    {
        if (s == hx_vrun_side)
        {
            if (hx_vrun < 250U) ++hx_vrun;
        }
        else
        {
            hx_vrun_side = s;
            hx_vrun      = 1U;
        }
    }
    else
    {
        hx_vrun      = 0U;
        hx_vrun_side = 0;
    }

    if (hx_vrun >= HEX_V_DEBOUNCE) s = hx_vrun_side;
    else                           s = 0;

    /* [HEX4] 逐拍记录 V 的原始结构，用来重新设计判据（按 'h' 打印） */
    {
        uint8_t fl = 0U;
        uint8_t nb = 0U;
        uint8_t k;

        for (k = 0U; k < 12U; ++k)
            if ((mask & (uint16_t)(1U << k)) != 0U) ++nb;

        if (v_l != 0U) fl |= 0x01U;
        if (v_r != 0U) fl |= 0x02U;
        if (s   != 0)  fl |= 0x04U;
        if (hx_state == 3U) fl |= 0x08U;

        HexLog_Push((int)nb, mask, error, fl);
    }

    rising = (uint8_t)((s != 0) && (hx_vprev == 0U));
    hx_vprev = (uint8_t)((s != 0) ? 1U : 0U);

    /* [V2-DIAG 2026-11-22] V1 之后（ARMED / TURN_IN）每一个 V 都打一条，
       供手推过关口确认"异侧 V"到底能不能检到、离 V1 多远：
         HEX,13,<dist_since_v1>,<right>,<left>,<center>,<mask>,<seq>
       dist_since_v1 单位 counts（435.7 = 1 cm）。
       看 mask 或 right/left 就能分出同侧/异侧（V1 的侧别看 HEX,4 的 mask）。 */
    if ((rising != 0U) && ((hx_state == 1U) || (hx_state == 2U)))
    {
        h->dbg_event = 13U;
        h->dbg_dist  = h->dist - hx_v1_dist;
        ++h->evt_seq;
    }

    if (s != 0) hx_side = s;

    /* 大误差方向：error>0 表示线在右 -> 应该右转 -> -1 */
    err_dir = 0;
    if (error >  HEX4_BIG_ERR) err_dir = -1;
    if (error < -HEX4_BIG_ERR) err_dir =  1;

    /* [HEX4-NET 2026-11-23] 从入口 30° 打舵开始累计有方向净转角：
       hx_net = ∫ hx_dir·gyro_z dt。
       同时保留 hx_total 绝对值，仅用于诊断/旧日志。 */
    if (gyro_ok && ((hx_state == 2U) || (hx_state == 3U)))
    {
        float d = gyro_z * 0.01f;

        hx_net += (float)hx_dir * d;
        if (hx_net > hx_net_peak)
            hx_net_peak = hx_net;

        if (hx_state == 3U)
            hx_total += (d >= 0.0f) ? d : -d;
    }

    /* [2026-11-24] 出栈锁到期自动解除；只清锁，不改状态。 */
    if ((hx_exit_lock_dist != 0U) && (h->dist >= hx_exit_lock_dist))
        hx_exit_lock_dist = 0U;

    switch (hx_state)
    {
    case 0U:    /* IDLE：等第一次 V */
        if (rising)
        {
            /* [2026-11-24] 出栈锁存期间不武装：
               否则 1~2 cm 后的 V1 会先把你武装，等 20 cm 锁一到期
               就直接拿这个旧 V1 进栈，等于 20 cm 内还是入了。
               锁到期后必须重新看到 V 上升沿。 */
            if (hx_exit_lock_dist != 0U)
                break;

#if (HEX4_FLOW_VETO == 1)
            /* [HEX4-FLOW-VETO] 用 5 拍锁存，不要求 FLOW 和 V1 严格同一拍。 */
            if ((hx_flow_veto_latch != 0U) &&
                (((hx_flow_veto_dir > 0) && (s > 0)) ||
                 ((hx_flow_veto_dir < 0) && (s < 0))))
            {
                hx_flow_veto_latch = 0U;
                hx_state = 0U;
                Hex_ClearCornerWatch(corner);

                h->dbg_event = 19U;
                h->dbg_dist  = 0U;
                ++h->evt_seq;
                break;
            }
#endif

            Hex_ClearCornerWatch(corner);            /* 入栈时清旧的 CW4 候选 */

            hx_state      = 1U;                 /* ARMED */
            hx_dir        = s;
            hx_err1       = err_dir;
            hx_v1_dist    = h->dist;
            hx_turn_at_v1 = pid_turn;
            hx_straight   = HEX4_STRAIGHT_TICKS;

            h->dbg_event = 4U;                  /* 4 = 武装 */
            h->dbg_dist  = h->dist;
            ++h->evt_seq;
        }
        break;

    case 1U:    /* ARMED：先回正，然后按 HEX4_NEED_V2 决定怎么进 */
#if (HEX4_FLOW_VETO == 1)
        /* [HEX4-FLOW-VETO 2026-11-23] 武装 V 后 FLOW 判出同侧直角弯：
           说明这里更像直角弯而不是六边形入口，取消本次六边形进入。
           锁存 5 拍，避免 V1 与 FLOW 相差一拍时漏掉。 */
        if ((hx_flow_veto_latch != 0U) &&
            (((hx_flow_veto_dir > 0) && (hx_dir > 0)) ||
             ((hx_flow_veto_dir < 0) && (hx_dir < 0))))
        {
            hx_flow_veto_latch = 0U;
            hx_state = 0U;
            Hex_ClearCornerWatch(corner);

            h->dbg_event = 19U;             /* 19 = FLOW 同侧直角弯否决 */
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
            break;
        }
#endif

#if (HEX4_OPP_VETO == 1)
        /* [V2 2026-11-22] 异侧否决：V1 后距离窗口内出现异侧 V -> 取消进入。
           双六边形 / 17 号那种"V1 和 V2 反着来"的位置，靠这一条挡住。
           窗口用独立的 HEX4_OPP_MIN/MAX（手推实测异侧 V 在 4.6 cm，
           比 V2 的 6 cm 下限更近，不能共用）。 */
        if ((rising != 0U) && (s != 0) && (s != hx_dir) &&
            ((h->dist - hx_v1_dist) >= HEX4_OPP_MIN_COUNTS) &&
            ((h->dist - hx_v1_dist) <= HEX4_OPP_MAX_COUNTS))
        {
            hx_state = 0U;

            h->dbg_event = 14U;                 /* 14 = 异侧 V 否决 */
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
            break;
        }
#endif
        if (hx_straight > 0U)
        {
            --hx_straight;

            /* 把方向环被 V 带偏的那部分收回来：给一个反向舵 */
            out.turn         = -HEX4_STRAIGHT_K * hx_turn_at_v1;
            out.speed_cmd    = BASE_SPEED;
            out.have_control = (HEX_MODE == 2U) ? 1U : 0U;

            if (!rising)
                break;                          /* 回正期间不看第二次 V */
        }

#if (HEX4_NEED_V2 == 1)
        /* 模式 1：必须等到同侧、方向一致的第二次 V */
        if (rising)
        {
            uint32_t d = h->dist - hx_v1_dist;

            if ((hx_exit_lock_dist == 0U) &&
                (s == hx_dir) &&
                (d >= HEX4_V2_MIN_COUNTS) && (d <= HEX4_V2_MAX_COUNTS) &&
                ((err_dir == hx_err1) || (hx_err1 == 0)))
            {
                hx_state = 2U;                  /* TURN_IN */
                hx_yaw   = 0.0f;
                hx_net   = 0.0f;                /* [HEX4-NET] 从 30° 打舵开始累计 */
                hx_net_peak = 0.0f;
                hx_t0    = h->dist;

                h->dbg_event = 5U;              /* 5 = 进入六边形 */
                h->dbg_dist  = d;
                ++h->evt_seq;
            }
            else
            {
                /* 侧别/方向/距离不符：以这次 V 重新武装 */
                hx_dir        = s;
                hx_err1       = err_dir;
                hx_v1_dist    = h->dist;
                hx_turn_at_v1 = pid_turn;
                hx_straight   = HEX4_STRAIGHT_TICKS;
            }
        }
        else if ((h->dist - hx_v1_dist) > HEX4_V2_MAX_COUNTS)
        {
            hx_state = 0U;                      /* 窗口过了，解除 */
            Hex_ClearCornerWatch(corner);

            h->dbg_event = 8U;                  /* 8 = 解除武装 */
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
        }
#else
        /* 模式 0（默认）：只要 V1 够了，武装后走够 HEX4_ARM_DIST_CM 就进。
           理由：V1 的判据很硬（端部两路 + 正中 + 中段缺口，实测持续 20 拍），
           而"第二个 V"对车的实际轨迹太敏感——巡线开着的时候入口的第二个 V
           经常不形成（实测 20 cm 内一次都没有）。两次 V 的间距手推实测 8.3 cm，
           所以走 9 cm 和等 V2 在几何上等价，但对轨迹不敏感。 */
        /* [2026-11-24] 出栈后 20 cm 内只武装、不进入 TURN_IN；
           继续测 V，走过锁存距离后自动恢复。 */
        if ((hx_exit_lock_dist == 0U) &&
            ((h->dist - hx_v1_dist) >= HEX4_ARM_DIST_COUNTS))
        {
            hx_state = 2U;                      /* TURN_IN */
            hx_yaw   = 0.0f;
            hx_net   = 0.0f;                    /* [HEX4-NET] 从 30° 打舵开始累计 */
            hx_net_peak = 0.0f;
            hx_t0    = h->dist;

            h->dbg_event = 5U;                  /* 5 = 进入六边形 */
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
        }
        else if ((h->dist - hx_v1_dist) > HEX4_V2_MAX_COUNTS)
        {
            hx_state = 0U;                      /* 兜底：走过头还没到 -> 解除 */
            Hex_ClearCornerWatch(corner);

            h->dbg_event = 8U;
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
        }
#endif
        break;

    case 2U:    /* TURN_IN：原地打 30° */
#if (HEX4_FLOW_VETO == 1)
        /* [HEX4-FLOW-VETO] FLOW 晚于 HEX,5 才到：在真正开始/继续 30° 打舵前
           仍然允许否决，避免已经进了 TURN_IN 就只能硬转。 */
        if ((hx_flow_veto_latch != 0U) &&
            (((hx_flow_veto_dir > 0) && (hx_dir > 0)) ||
             ((hx_flow_veto_dir < 0) && (hx_dir < 0))))
        {
            hx_flow_veto_latch = 0U;
            hx_state = 0U;
            Hex_ClearCornerWatch(corner);

            h->dbg_event = 19U;
            h->dbg_dist  = h->dist - hx_v1_dist;
            ++h->evt_seq;
            break;
        }
#endif

        if (gyro_ok)
            hx_yaw += gyro_z * 0.01f;

        if (((hx_yaw * hx_dir) >= (HEX4_TURN_DEG - HEX4_TURN_TOL)) ||
            ((h->dist - hx_t0) > HEX4_TURN_MAX_COUNTS))
        {
#if (HEX4_TURNBACK_ENABLE == 1)
            /* [HEX4-TURNBACK 2026-11-25] 先把"实际转过的角"记下来：
               没线时用它当反转目标；有线进 DRIVING 后，入口窗口内丢线也用它。 */
            {
                float done = (hx_yaw >= 0.0f) ? hx_yaw : -hx_yaw;

                /* 反转到和刚才实际转过的角差不多就停；陀螺不可用时退回 30° */
                hx_turnback_deg = (done >= HEX4_TURN_TOL) ? done : HEX4_TURN_DEG;
                if (hx_turnback_deg > HEX4_TURN_DEG)
                    hx_turnback_deg = HEX4_TURN_DEG;
            }
            hx_drive_lost = 0U;

            /* 30° 打完了，但这一拍阵列里没有线（mask 就是 control 传进来的
               line.mask，单灯/坏点已经被 Sensor_Scan 的 filt 清成 0）：
               说明这个入口的 30° 转过去是空的（误判 / 转过头）。
               不再进 DRIVING（下一拍 line.cnt=0 就会落到 HOLD，HOLD 会沿同一侧
               继续加舵，越走越远），改成反向转回，转到看到线就交回方向环。
               反向转回自己出舵（speed=0 + have_control=1），control 里的 HOLD
               因为 hex_pivot_active=1 被跳过，这条分支不走 HOLD。 */
            if (Hex_LineSeen(mask) == 0U)
            {
                hx_state    = 5U;               /* TURN_BACK */
                hx_yaw      = 0.0f;
                hx_t0       = h->dist;
                hx_net      = 0.0f;
                hx_net_peak = 0.0f;

                /* 本拍就开始反向出舵：否则这一拍 have_control=0，
                   control 里的 hex_pivot_active 还是 0，HOLD 会趁机按旧舵打一拍。 */
                out.turn         = -(float)hx_dir * HEX4_TURN_RATE;
                out.speed_cmd    = 0.0f;
                out.have_control = (HEX_MODE == 2U) ? 1U : 0U;

                h->dbg_event = 20U;             /* 20 = 转完没线，开始反向转回 */
                h->dbg_dist  = h->dist;
                ++h->evt_seq;
                break;
            }
#endif
#if (HEX4_ENTRY_ONLY == 1)
            /* [HEX4-ENTRYONLY] 只做入口 30°：完成后直接交回方向环，
               不再进 DRIVING，不再等出口 V，也不做 TURN_OUT。 */
            hx_state         = 0U;
            hx_total         = 0.0f;
            hx_yaw           = 0.0f;
            hx_lockout_dist  = h->dist + HEX4_ENTRY_ONLY_LOCKOUT_COUNTS;
            Hex_ClearCornerWatch(corner);
#else
            hx_state = 3U;                      /* DRIVING */
            hx_total = 0.0f;
#if (HEX4_TURNBACK_ENABLE == 1)
            hx_t0    = h->dist;                 /* [HEX4-TURNBACK] 入口丢线窗口从这里算 */
#endif
#endif

            h->dbg_event = 6U;                  /* 6 = 进弯转完 */
            h->dbg_dist  = h->dist;
            ++h->evt_seq;
            break;
        }

        out.turn         = (float)hx_dir * HEX4_TURN_RATE;
        out.speed_cmd    = 0.0f;                /* 原地转 */
        out.have_control = (HEX_MODE == 2U) ? 1U : 0U;
        break;

#if (HEX4_TURNBACK_ENABLE == 1)
    case 5U:    /* TURN_BACK：反向转回找线（本分支不走 HOLD） */
        /* [HEX4-TURNBACK 2026-11-25] 朝 -hx_dir 原地打舵，把刚才那 30° 收回来：
           1) 本拍看到线 -> 立刻收手，六边形标记清零，交回方向环（HEX,21）；
           2) 反转到 TURN_IN 实际转过的角还没线 -> 同样清零交回（HEX,22），
              不再硬转，后面交给正常的丢线流程（HOLD/盲穿/停车）。
           出舵期间 speed=0、have_control=1 => control 的 hex_pivot_active=1，
           HOLD 块整段被跳过，不会一边反向转一边又被 HOLD 往正方向拉。 */
        if (Hex_LineSeen(mask) != 0U)
        {
            Hex_TurnBackExit(h, corner, h->dist + HEX4_TURNBACK_LOCKOUT_COUNTS);

            h->dbg_event = 21U;                 /* 21 = 反向转回看到线，交回方向环 */
            h->dbg_dist  = h->dist;
            ++h->evt_seq;
            break;
        }

        if (gyro_ok)
            hx_yaw += gyro_z * 0.01f;

        if (((hx_yaw * (float)(-hx_dir)) >= (hx_turnback_deg - HEX4_TURN_TOL)) ||
            ((h->dist - hx_t0) > HEX4_TURN_MAX_COUNTS))
        {
            Hex_TurnBackExit(h, corner, h->dist + HEX4_TURNBACK_LOCKOUT_COUNTS);

            h->dbg_event = 22U;                 /* 22 = 反向转回没线，转满放弃 */
            h->dbg_dist  = h->dist;
            ++h->evt_seq;
            break;
        }

        out.turn         = -(float)hx_dir * HEX4_TURN_RATE;
        out.speed_cmd    = 0.0f;                /* 原地转 */
        out.have_control = (HEX_MODE == 2U) ? 1U : 0U;
        break;
#endif

    case 3U:    /* DRIVING：交回方向环，累计有方向净转角 */
#if (HEX4_TURNBACK_ENABLE == 1)
        /* [HEX4-TURNBACK 2026-11-25] 入口窗口内丢线也算"转弯完成没找到线"：
           TURN_IN 那一拍可能还剩 2 路（线正压在中心，error≈0），下一拍才变单灯，
           单灯被 filt 清成 mask=0。此时已经进 DRIVING：方向环没线、HOLD 又因为
           error≈0 没武装、六边形区间还挡着盲穿 -> 目标清零、车停住
           （现场就是"只亮一个灯停在 turnback 点"）。这里改成就地反向转回。
           丢线要连续 HEX4_TURNBACK_LOST_TICKS 拍才算，避免断线缺口毛刺误触发。 */
        if ((hx_turnback_deg > 0.0f) &&
            ((h->dist - hx_t0) <= HEX4_TURNBACK_GRACE_COUNTS))
        {
            if (Hex_LineSeen(mask) == 0U)
            {
                if (hx_drive_lost < 250U)
                    ++hx_drive_lost;

                if (hx_drive_lost >= HEX4_TURNBACK_LOST_TICKS)
                {
                    hx_state      = 5U;         /* TURN_BACK */
                    hx_yaw        = 0.0f;
                    hx_t0         = h->dist;
                    hx_drive_lost = 0U;
                    hx_net        = 0.0f;
                    hx_net_peak   = 0.0f;

                    out.turn         = -(float)hx_dir * HEX4_TURN_RATE;
                    out.speed_cmd    = 0.0f;
                    out.have_control = (HEX_MODE == 2U) ? 1U : 0U;

                    h->dbg_event = 20U;         /* 20 = 转完没线，开始反向转回 */
                    h->dbg_dist  = h->dist;
                    ++h->evt_seq;
                    break;
                }
            }
            else
            {
                hx_drive_lost = 0U;
            }
        }
        else
        {
            hx_turnback_deg = 0.0f;             /* 出了窗口就不再算入口没线 */
        }
#endif
        if (rising)
        {
            h->dbg_event = 12U;
            h->dbg_dist  = (uint32_t)hx_net;    /* 当前有向净角 */
            ++h->evt_seq;
        }

        /* [HEX4-NET 2026-11-23] 出口不再靠固定绝对角窗口：
           1) 有向净角到 320~340° -> 强制出栈打舵；
           2) 先涨后反向回落 >120°，且净角回到 ±20° -> 双六边形返回，强制出栈打舵。 */
        if (hx_net >= HEX4_NET_EXIT_DEG)
        {
            hx_state = 4U;
            hx_yaw   = 0.0f;
            hx_t0    = h->dist;
            hx_err1  = (int8_t)(hx_dir);   /* 强制出栈先按入口同方向打舵 */

            h->dbg_event = 17U;             /* 17 = 有向积分到 320°，强制出栈 */
            h->dbg_dist  = (uint32_t)hx_net;
            ++h->evt_seq;
        }
        else
        {
            float net_abs = (hx_net >= 0.0f) ? hx_net : -hx_net;

            if (((hx_net_peak - hx_net) >= HEX4_NET_REV_DROP_DEG) &&
                (net_abs <= HEX4_NET_REV_END_DEG))
            {
                hx_state = 4U;
                hx_yaw   = 0.0f;
                hx_t0    = h->dist;
                hx_err1  = (int8_t)(hx_dir); /* 反向回落也按入口同方向出栈 */

                h->dbg_event = 18U;           /* 18 = 反向回落，强制出栈 */
                h->dbg_dist  = (uint32_t)(hx_net_peak * 10.0f);
                ++h->evt_seq;
            }
            else if ((h->dist - hx_t0) > HEX4_DRIVE_MAX_COUNTS)
            {
                /* 最后的卡死保护：方向积分既不涨也不回落，走满 300 cm 才解除 */
                hx_state = 0U;
                Hex_ClearCornerWatch(corner);   /* 出栈后清旧候选，放开直角弯检测 */
                hx_exit_lock_dist = h->dist + HEX4_EXIT_LOCKOUT_COUNTS;

                h->dbg_event = 8U;
                h->dbg_dist  = (uint32_t)hx_total;
                ++h->evt_seq;
            }
        }
        break;

    case 4U:    /* TURN_OUT：按反向打 30°，打完交回方向环找线 */
#if (HEX4_TURN_OUT_ENABLE == 0)
        /* [2026-11-24] 暂时禁用出栈 30° 打舵：
           直接清状态、交回方向环，不原地转。仍然发 HEX,10，
           日志里仍能看到“出栈完成”。 */
        hx_state = 0U;
        Hex_ClearCornerWatch(corner);
        hx_exit_lock_dist = h->dist + HEX4_EXIT_LOCKOUT_COUNTS;

        h->dbg_event = 10U;                     /* 10 = 出栈完成（本次不打舵） */
        h->dbg_dist  = h->dist;
        ++h->evt_seq;
        break;
#else
        if (gyro_ok)
            hx_yaw += gyro_z * 0.01f;

        if (((hx_yaw * (float)hx_err1) >= (HEX4_TURN_DEG - HEX4_TURN_TOL)) ||
            ((h->dist - hx_t0) > HEX4_TURN_MAX_COUNTS))
        {
            hx_state = 0U;
            Hex_ClearCornerWatch(corner);       /* 出栈后清旧候选，放开直角弯检测 */
            hx_exit_lock_dist = h->dist + HEX4_EXIT_LOCKOUT_COUNTS;

            h->dbg_event = 10U;                 /* 10 = 出弯转完 */
            h->dbg_dist  = h->dist;
            ++h->evt_seq;
            break;
        }

        out.turn         = (float)hx_err1 * HEX4_TURN_RATE;
        out.speed_cmd    = 0.0f;
        out.have_control = (HEX_MODE == 2U) ? 1U : 0U;
        break;
#endif

    default:
        hx_state = 0U;
        Hex_ClearCornerWatch(corner);
        break;
    }

    /* [HEX4 2026-11-21] 六边形区间：**一旦武装就屏蔽直角弯检测**，不只是 DRIVING。
       原因：六边形顶点处的 mask 是 ch5~11 连成一片（实测 4064），
       对直角弯检测来说就是"右侧 6 路全亮" = 标准 L 形，必然误触发。
       而六边形内部的顶点又没有 V 的缺口，六边形状态机本身认不出来。
       所以必须"从第一次 V 开始"就把直角弯挡掉，否则每个顶点都会转 90°。 */
    hex_zone_flag = (uint8_t)((hx_state != 0U) ? 1U : 0U);

    /* [HEX4] 本拍如果产生了新事件，推进环形缓冲；主循环按序打印，不会只留最后一条 */
    if (h->evt_seq != hx_seq_last)
    {
        hx_seq_last = h->evt_seq;
        HexEvent_Push(h->dbg_event, h->dbg_dist, mask, right, left, center, h->evt_seq);

        /* [HEX4] 示波器式捕获：事件后再记 HEXLOG_AFTER 拍就冻结，
           这样"什么时候按 h"都不影响结果——缓冲里永远是事件前后那一段。 */
        hxlog_after = 20U;
        hxlog_pause = 0U;
    }

    if (hxlog_after > 0U)
    {
        if (--hxlog_after == 0U)
            hxlog_pause = 1U;       /* 冻结，主循环按 'h' 取出的就是这一段 */
    }

    return out;
}

/* ================= [FLOW 2026-11-23] 流动直角弯检测 =================
   第一阶段：只并行观察 + 串口 FLOW 日志，不接管电机。

   每拍做四件事：
     1) 把过滤后的 mask 压进 128 拍（1.28 s）环形窗口；
     2) 找“最中间的连续灯段”；
     3) 用上一拍主段中心把“刚加的通道”分成左右；
     4) 主段宽度突然增加 + 只有一侧新增 -> 该侧直角弯；
        同一拍两侧都新增 -> 十字/发车线。

   方向约定：+1 = 左转，-1 = 右转，0 = 十字/发车线。 */

#define FLOW_EVTQ_MAX 4U

typedef struct {
    uint16_t hist[FLOW_WIN_TICKS];
    uint8_t  head;         /* 下一个写入位置 */
    uint8_t  num;          /* 已有多少拍（最多 128） */
    uint8_t  prev_valid;
    uint8_t  prev_l;
    uint8_t  prev_r;
    uint16_t prev_mask;
    uint8_t  cooldown;

    /* [FLOW-SNAP] 检测到直角弯后延时抓窗口 */
    uint8_t  snap_pending;
    uint8_t  snap_delay;
    FlowEvent snap_evt;
} FlowCtx;

static FlowCtx flow;

/* [FLOW-SNAP] 两槽快照：中断只拷贝，主循环慢慢打印。
   槽 1 正在打印时，槽 2 还能接住下一个弯。 */
#define FLOW_SNAP_MAX 2U
static FlowSnapshot flow_snaps[FLOW_SNAP_MAX];
static volatile uint8_t flow_snap_head;
static volatile uint8_t flow_snap_tail;

/* [FLOW-DIAG] 记录“为什么这一拍的 prev 被清掉”，主循环打印。
   只在 prev_valid=1 时入队，数量很少。 */
#define FLOW_DIAG_MAX 8U
static FlowDiag flow_diagq[FLOW_DIAG_MAX];
static volatile uint8_t flow_diag_head;
static volatile uint8_t flow_diag_tail;

/* [FLOW-OR] 一次性 ARC 触发请求：Flow_Update 置位，Control_Direction 取走。 */
static volatile uint8_t flow_fire_pending;
static volatile int8_t  flow_fire_dir;

static FlowEvent flow_evtq[FLOW_EVTQ_MAX];
static volatile uint8_t flow_evt_head;
static volatile uint8_t flow_evt_tail;
static volatile uint8_t flow_log_pause;    /* 主循环转储时暂停写入，避免撕裂 */

static uint8_t Flow_PopCount(uint16_t m)
{
    uint8_t n = 0U;

    while (m != 0U)
    {
        n = (uint8_t)(n + (uint8_t)(m & 1U));
        m >>= 1;
    }

    return n;
}

static uint16_t Flow_RunMask(uint8_t l, uint8_t r)
{
    uint16_t m = 0U;
    uint8_t  i;

    for (i = l; (i <= r) && (i < 12U); ++i)
        m |= (uint16_t)(1U << i);

    return m;
}

/* 找中心离 5.5 最近的连续段；距离相同取更宽的一段 */
static uint8_t Flow_FindMainRun(uint16_t mask, uint8_t *out_l, uint8_t *out_r)
{
    uint8_t i = 0U;
    uint8_t best_l = 0U, best_r = 0U;
    int     best_d = 999;
    int     best_w = 0;

    while (i < 12U)
    {
        uint8_t a, b;
        int     center2, d, w;

        while ((i < 12U) && ((mask & (uint16_t)(1U << i)) == 0U))
            ++i;
        if (i >= 12U)
            break;

        a = i;
        while ((i < 12U) && ((mask & (uint16_t)(1U << i)) != 0U))
            ++i;
        b = (uint8_t)(i - 1U);

        center2 = (int)a + (int)b;       /* 2 × 段中心 */
        d = center2 - 11;                /* 离 5.5 的距离 ×2 */
        if (d < 0) d = -d;
        w = (int)b - (int)a + 1;

        if ((d < best_d) || ((d == best_d) && (w > best_w)))
        {
            best_d = d;
            best_w = w;
            best_l = a;
            best_r = b;
        }
    }

    if (best_d == 999)
        return 0U;

    *out_l = best_l;
    *out_r = best_r;
    return 1U;
}

static void Flow_LogPush(uint16_t mask)
{
    flow.hist[flow.head] = mask;
    flow.head = (uint8_t)((flow.head + 1U) % FLOW_WIN_TICKS);
    if (flow.num < FLOW_WIN_TICKS)
        ++flow.num;
}

static void Flow_PushEvent(const FlowEvent *ev)
{
    uint8_t nxt = (uint8_t)((flow_evt_head + 1U) % FLOW_EVTQ_MAX);

    if (nxt == flow_evt_tail)
        return;                         /* 队列满：丢新事件 */

    flow_evtq[flow_evt_head] = *ev;
    flow_evt_head = nxt;
}

static void Flow_PushDiag(uint8_t reason, uint8_t raw_cnt, uint16_t raw_mask,
                          uint16_t prev_mask, uint16_t cur_mask,
                          uint8_t prev_l, uint8_t prev_r)
{
    uint8_t nxt = (uint8_t)((flow_diag_head + 1U) % FLOW_DIAG_MAX);
    FlowDiag *d;

    if (nxt == flow_diag_tail)
        return;

    d = &flow_diagq[flow_diag_head];
    d->reason    = reason;
    d->raw_cnt   = raw_cnt;
    d->raw_mask  = raw_mask;
    d->prev_mask = prev_mask;
    d->cur_mask  = cur_mask;
    d->prev_l    = prev_l;
    d->prev_r    = prev_r;

    flow_diag_head = nxt;
}

/* [FLOW-SNAP] 在中断里把当前 1.28 s 窗口拷进快照槽。
   只做内存拷贝，不调用 printf，不阻塞控制拍。 */
static void Flow_TakeSnapshot(void)
{
    uint8_t nxt = (uint8_t)((flow_snap_head + 1U) % FLOW_SNAP_MAX);
    uint8_t start, k, i;
    FlowSnapshot *s;

    if (nxt == flow_snap_tail)
        return;                         /* 主循环还没打印完，丢新快照 */

    s = &flow_snaps[flow_snap_head];
    s->event = flow.snap_evt;
    s->num   = flow.num;

    start = (uint8_t)((flow.head + FLOW_WIN_TICKS - flow.num) % FLOW_WIN_TICKS);
    for (i = 0U; i < flow.num; ++i)
    {
        k = (uint8_t)((start + i) % FLOW_WIN_TICKS);
        s->mask[i] = flow.hist[k];
    }

    flow_snap_head = nxt;
}

void Flow_Reset(void)
{
    flow.head        = 0U;
    flow.num         = 0U;
    flow.prev_valid  = 0U;
    flow.prev_l      = 0U;
    flow.prev_r      = 0U;
    flow.prev_mask   = 0U;
    flow.cooldown    = 0U;
    flow.snap_pending = 0U;
    flow.snap_delay   = 0U;
    flow_evt_head    = 0U;
    flow_evt_tail    = 0U;
    flow_snap_head   = 0U;
    flow_snap_tail   = 0U;
    flow_diag_head   = 0U;
    flow_diag_tail   = 0U;
    flow_fire_pending = 0U;
    flow_fire_dir     = 0;
    flow_log_pause   = 0U;
}

void Flow_Update(uint16_t mask, uint8_t raw_cnt, uint16_t raw_mask)
{
    uint8_t  l, r;
    uint16_t run_mask;
    uint16_t new_mask;
    uint16_t new_l_mask = 0U, new_r_mask = 0U;
    uint8_t  new_l_cnt, new_r_cnt;
    int      center2;
    int      jump_i;
    int8_t   jump;
    uint8_t  i;

    if (flow_log_pause != 0U)
        return;

    Flow_LogPush(mask);

    /* [FLOW-SNAP] 事件后延时抓取：先在中断里等 10 拍，再拷窗口。
       这样窗口包含触发后的 100 ms，主循环打印时不会再动这份数据。 */
    if (flow.snap_pending != 0U)
    {
        if (flow.snap_delay > 0U)
            --flow.snap_delay;
        else
        {
            Flow_TakeSnapshot();
            flow.snap_pending = 0U;
        }
    }

    if (flow.cooldown > 0U)
        --flow.cooldown;

    if (Flow_FindMainRun(mask, &l, &r) == 0U)
    {
        /* [FLOW-DIAG] 只有 prev_valid=1 时才记，避免上电/长时间停线刷屏。 */
        if (flow.prev_valid != 0U)
            Flow_PushDiag((uint8_t)((raw_cnt == 1U) ? 1U : 0U),
                          raw_cnt, raw_mask,
                          flow.prev_mask, 0U,
                          flow.prev_l, flow.prev_r);

        flow.prev_valid = 0U;
        flow.prev_mask  = 0U;
        return;
    }

    run_mask = Flow_RunMask(l, r);

    /* 第一拍只建立基准，不判 */
    if (flow.prev_valid == 0U)
    {
        flow.prev_valid = 1U;
        flow.prev_l     = l;
        flow.prev_r     = r;
        flow.prev_mask  = run_mask;
        return;
    }

    /* 主段完全换了一段：不判，避免把“跳到另一段”当成突然加灯 */
    if ((run_mask & flow.prev_mask) == 0U)
    {
        Flow_PushDiag(2U, raw_cnt, raw_mask,
                      flow.prev_mask, run_mask,
                      flow.prev_l, flow.prev_r);

        flow.prev_l    = l;
        flow.prev_r    = r;
        flow.prev_mask = run_mask;
        return;
    }

    if (flow.cooldown > 0U)
    {
        flow.prev_l    = l;
        flow.prev_r    = r;
        flow.prev_mask = run_mask;
        return;
    }

    jump_i = ((int)r - (int)l + 1) - ((int)flow.prev_r - (int)flow.prev_l + 1);
    jump   = (int8_t)((jump_i < 0) ? 0 : jump_i);

    if ((uint8_t)jump < FLOW_MIN_WIDTH_JUMP)
    {
        flow.prev_l    = l;
        flow.prev_r    = r;
        flow.prev_mask = run_mask;
        return;
    }

    /* “刚加的通道”：当前主段里，上一拍主段没有的位 */
    new_mask = (uint16_t)(run_mask & (uint16_t)(~flow.prev_mask));

    /* 用“上一拍主段中心”分左右：这样判定是相对于主连续段的，
       不是相对于固定 5.5 中线。 */
    center2 = (int)flow.prev_l + (int)flow.prev_r;   /* 2 × 上一拍中心 */
    for (i = 0U; i < 12U; ++i)
    {
        int c2;

        if ((new_mask & (uint16_t)(1U << i)) == 0U)
            continue;

        c2 = 2 * (int)i;
        if (c2 < center2)
            new_l_mask |= (uint16_t)(1U << i);
        else if (c2 > center2)
            new_r_mask |= (uint16_t)(1U << i);
        /* c2 == center2：正好在上一拍中心的那一路，不归左也不归右 */
    }

    new_l_cnt = Flow_PopCount(new_l_mask);
    new_r_cnt = Flow_PopCount(new_r_mask);

    if ((new_l_cnt > 0U) || (new_r_cnt > 0U))
    {
        uint8_t type = FLOW_EVT_NONE;
        int8_t  dir  = 0;

        if ((new_l_cnt > 0U) && (new_r_cnt > 0U))
        {
            /* 十字/发车线：同一拍两侧都新增。
               再要求每侧至少 FLOW_CROSS_MIN_NEW_CH 路，专门压误触发。 */
            if ((new_l_cnt >= FLOW_CROSS_MIN_NEW_CH) &&
                (new_r_cnt >= FLOW_CROSS_MIN_NEW_CH))
            {
                type = FLOW_EVT_CROSS;
                dir  = 0;
            }
        }
        else if (new_l_cnt > 0U)
        {
            type = FLOW_EVT_LEFT;
            dir  = 1;
        }
        else
        {
            type = FLOW_EVT_RIGHT;
            dir  = -1;
        }

        if (type != FLOW_EVT_NONE)
        {
            FlowEvent ev;

            ev.type       = type;
            ev.dir        = dir;
            ev.new_mask   = new_mask;
            ev.run_mask   = run_mask;
            ev.run_l      = l;
            ev.run_r      = r;
            ev.prev_l     = flow.prev_l;
            ev.prev_r     = flow.prev_r;
            ev.width_jump = jump;
            ev.new_l_cnt  = new_l_cnt;
            ev.new_r_cnt  = new_r_cnt;

            Flow_PushEvent(&ev);
            flow.cooldown = FLOW_COOLDOWN_TICKS;

#if (FLOW_AUTO_DUMP == 1)
            /* 只对左/右直角弯自动抓窗口；十字只留 FLOW 事件行。 */
            if (((type == FLOW_EVT_LEFT) || (type == FLOW_EVT_RIGHT)) &&
                (flow.snap_pending == 0U))
            {
                flow.snap_evt     = ev;
                flow.snap_delay   = FLOW_SNAP_DELAY_TICKS;
                flow.snap_pending = 1U;
            }
#endif

#if (CORNER_TRIGGER_FLOW == 1)
            /* [FLOW-OR] 左/右直角弯直接请求触发 ARC；十字不触发。
               [2026-11-24] FLOW 不参与六边形否决：只有 FLOW 自己接管
               过弯时才生成这个一次性请求。 */
            if ((type == FLOW_EVT_LEFT) || (type == FLOW_EVT_RIGHT))
            {
                flow_fire_dir     = dir;
                flow_fire_pending = 1U;
            }
#endif
        }
    }

    flow.prev_l    = l;
    flow.prev_r    = r;
    flow.prev_mask = run_mask;
}

uint8_t Flow_PopEvent(FlowEvent *e)
{
    if (flow_evt_tail == flow_evt_head)
        return 0U;

    *e = flow_evtq[flow_evt_tail];
    flow_evt_tail = (uint8_t)((flow_evt_tail + 1U) % FLOW_EVTQ_MAX);
    return 1U;
}

uint8_t Flow_PopSnapshot(FlowSnapshot *s)
{
    if (flow_snap_tail == flow_snap_head)
        return 0U;

    *s = flow_snaps[flow_snap_tail];
    flow_snap_tail = (uint8_t)((flow_snap_tail + 1U) % FLOW_SNAP_MAX);
    return 1U;
}

uint8_t Flow_PopDiag(FlowDiag *d)
{
    if (flow_diag_tail == flow_diag_head)
        return 0U;

    *d = flow_diagq[flow_diag_tail];
    flow_diag_tail = (uint8_t)((flow_diag_tail + 1U) % FLOW_DIAG_MAX);
    return 1U;
}

uint8_t Flow_TakeFire(float *dir)
{
    if (flow_fire_pending == 0U)
        return 0U;

    flow_fire_pending = 0U;
    *dir = (float)flow_fire_dir;
    return 1U;
}

uint8_t Flow_LogCount(void)
{
    return flow.num;
}

void Flow_GetAt(uint8_t i, uint16_t *mask, uint8_t *l, uint8_t *r, uint8_t *width)
{
    uint8_t start, k, ll, rr;

    if (i >= flow.num)
    {
        *mask = 0U;
        *l = 0U;
        *r = 0U;
        *width = 0U;
        return;
    }

    start = (uint8_t)((flow.head + FLOW_WIN_TICKS - flow.num) % FLOW_WIN_TICKS);
    k = (uint8_t)((start + i) % FLOW_WIN_TICKS);

    *mask = flow.hist[k];
    if (Flow_FindMainRun(*mask, &ll, &rr) != 0U)
    {
        *l = ll;
        *r = rr;
        *width = (uint8_t)(rr - ll + 1U);
    }
    else
    {
        *l = 0U;
        *r = 0U;
        *width = 0U;
    }
}

void Flow_ClearLog(void)
{
    flow.head = 0U;
    flow.num  = 0U;

    /* 转储期间 Flow_Update 被暂停；恢复后不能拿转储前的 prev 做比较，
       否则会跨过 0.2 s 的空白误判成“突然加灯”。 */
    flow.prev_valid = 0U;
    flow.prev_mask  = 0U;
    flow.cooldown   = 0U;
    flow.snap_pending = 0U;     /* 手动转储时取消还没抓的自动快照 */
}

void Flow_LogPause(uint8_t en)
{
    flow_log_pause = en;
}
