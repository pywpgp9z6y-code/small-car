#ifndef SPECIAL_H
#define SPECIAL_H

#include <stdint.h>

/* 过弯四段状态（[APPROACH 2026-11-14]） */
typedef enum {
    CORNER_ST_LINE     = 0U,   /* 正常巡线 */
    CORNER_ST_APPROACH = 1U,   /* 冲弯：直行 CORNER_APPROACH_CM，只保航向 */
    CORNER_ST_BRAKE    = 2U,   /* 刹停 */
    CORNER_ST_PIVOT    = 3U,   /* 原地转（粗转） */
    CORNER_ST_SETTLE   = 4U,   /* [SETTLE 2026-11-15] 航向收尾：把粗转剩下的几度补掉 */
    CORNER_ST_RECOVER  = 5U,   /* [RECOVER 2026-11-16] 转完仍丢线：反向找 30° */
    CORNER_ST_ARC      = 6U    /* [ARC 2026-11-18] 带速度弧线过弯（偏航双环） */
} CornerPhase;

/* 通用过弯锁存状态 */
typedef struct {
    uint8_t  state;          /* CornerPhase：0=巡线，1=冲弯，2=刹车，3=pivot */
    float    dir;            /* +1 左转，-1 右转 */
    float    yaw;            /* 本次过弯陀螺积分 */
    uint16_t ticks;          /* 保持拍数，100 Hz */
    uint16_t cooldown;       /* 出弯冷却（旧：拍数，已不用） */
    uint32_t cooldown_dist;  /* [COOLDOWN 2026-11-18] 出弯冷却改为距离制：
                                dist < cooldown_dist 期间不再锁存。
                                时间制在 BASE 120 下是 11 cm，比锯齿段的角间距还长。 */
    uint16_t ramp_ticks;     /* 出弯速度斜坡 */

    /* ---- 原地 pivot（[PIVOT 2026-11-14] 新增） ---- */
    float    yaw_ref;        /* 目标转角，°（入弯时置 CORNER_TURN_DEG） */
    uint16_t pivot_ticks;    /* pivot 已执行拍数，用于前 15 拍给起转脉冲 */

    /* ---- 冲弯/刹车（[APPROACH 2026-11-14] 新增） ---- */
    uint32_t dist;           /* APPROACH 段已走 counts（从检测点算起） */
    float    speed_ref;      /* BRAKE 段速度给定 */
    uint16_t phase_ticks;    /* 当前段计时 */
    uint16_t stall_ticks;    /* [SAFE 2026-11-14] pivot 连续不动的拍数 */
    float    prev_error;     /* [GND 2026-11-14] 上一拍误差，用于算变化率（区分缓弯/直角弯） */

    /* ---- 冲弯段破静摩擦（[KICK 2026-11-14]，仅 APPROACH 使用） ---- */
    uint8_t  bk_left;        /* 本次脉冲还剩几拍 */
    uint8_t  bk_wait;        /* 已经连续几拍没动 */
    uint8_t  bk_tries;       /* 本轮已经踹了几次 */

    /* ---- 直角弯检测状态机（[CW 2026-11-15]） ---- */
    uint8_t  cw_state;       /* 0=IDLE, 1=WIDE（已看到单侧宽线，等全黑） */
    uint8_t  cw_dark;        /* 连续全黑拍数 */
    uint8_t  cw_low;         /* 宽线之后连续正常(<=2路)拍数 */
    float    cw_side;        /* +1 左侧宽线（左转），-1 右侧 */
    uint16_t cw_ticks;       /* WIDE 状态已持续拍数（会被"弯口特征"重置） */
    uint16_t cw_wide;        /* [ARC 2026-11-18] 进入 WIDE 后的总拍数（不重置） */
    uint16_t cw_mask;        /* [诊断 2026-11-15] 进入 WIDE（定为弯口）那一拍的原始掩码 */

    uint8_t  armed;          /* [ARM 2026-11-15] 1=允许锁存；出弯后要等误差稳定低于阈值才重新置 1 */
    uint16_t arm_cnt;        /* [ARM 2026-11-15] 误差连续低于阈值的拍数 */
    uint16_t settle_ticks;   /* [SETTLE 2026-11-15] 收尾段计时 */
    float    recover_dir;    /* [RECOVER 2026-11-16] 找线方向（= -dir） */
    uint16_t recover_ticks;  /* [RECOVER 2026-11-16] 找线计时 */
    uint16_t arc_ticks;      /* [ARC 2026-11-18] 弧线过弯计时 */
    uint8_t  arc_line_ticks; /* [ARC 2026-11-18] 出弯后连续看到线的拍数 */
} CornerState;

/* 六边形入口状态 */
typedef struct {
    uint8_t  v_prev;         /* [HEX-V 2026-11-15] 上一拍是否处于"严" V 型（武装边沿） */
    uint8_t  cond_seen;      /* [诊断] 本次武装内是否已记录过"条件成立但间隔不够" */
    uint8_t  arm_lost;       /* [FIX 2026-11-16] 武装期间连续丢线拍数，超阈值就取消武装 */
    uint8_t  zone;           /* [HEX-ZONE] 1 = 处于六边形区间（禁用直角弯检测） */
    uint8_t  v_stack;        /* [DBLHEX 2026-11-18] 1 = 已入栈（在六边形内） */
    float    hex_yaw;        /* [DBLHEX] 入栈后累计转角（每 180° 归零一次） */
    uint8_t  half_done;      /* [DBLHEX] 1 = 已转满第一个 180° */
    uint8_t  full_done;      /* [DBLHEX] 1 = 已转满 360°（累计），此时 V 才允许出栈 */
    uint32_t zone_dist;      /* [HEX-ZONE] 上次见到 V 时的里程 */
    uint8_t  v_active;       /* 1 = 已确认六边形入口，正在原地转 */
    float    v_yaw;          /* [原地转] 本次动作累计转角，° */

    float    dir;            /* -1 右转，+1 左转 */
    uint16_t event_cnt;      /* 0=无 1=第一次 V 2=已确认 */
    uint16_t arm_ticks;      /* 调试用（V 判据不再使用武装超时） */
    uint16_t turn_ticks;
    uint32_t dist;
    uint32_t v_dist;         /* 第一次 V 时的里程 */
    uint16_t evt_seq;

    uint8_t  dbg_event;
    uint32_t dbg_dist;
    uint8_t  dbg_right;
    uint8_t  dbg_left;
    uint8_t  dbg_center;
    uint16_t dbg_mask;
} HexState;

typedef struct {
    float    turn;
    float    speed_cmd;
    uint8_t  have_control;
} HexOutput;

typedef struct {
    uint16_t evt_seq;
    uint8_t  event;
    uint32_t event_dist;
    uint8_t  right;
    uint8_t  left;
    uint8_t  center;
    uint16_t mask;
    uint16_t event_cnt;
    uint16_t off_r;
    uint16_t off_l;
    uint32_t dist;
    uint16_t arm;
} HexDebug;

void Corner_Reset(CornerState *c);
uint8_t Corner_IsActive(const CornerState *c);
uint8_t Corner_IsCooling(const CornerState *c);   /* [COOLDOWN 2026-11-18] 1 = 还在出弯冷却期 */
void Corner_AddDistance(CornerState *c, uint32_t counts);   /* [APPROACH 2026-11-14] */

/* 返回最终 turn；speed_cmd 由函数写入。
   pid_turn 是方向环 PID + 二次项、限幅后的值。
   speed_meas 是编码器实测速度（左右平均），刹车段用它判停。 */
float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint16_t mask,      /* [CW 2026-11-15] 12 路原始掩码：bit i = 通道 i 压线 */
                    uint8_t gyro_ok,
                    float gyro_z,
                    float speed_meas,
                    float *speed_cmd);

void Hex_Reset(HexState *h);
void Hex_AddDistance(HexState *h, uint32_t counts);

/* [TURNBIAS 2026-11-18] 1 = 当前处于六边形区间（转向增益可放大） */
uint8_t Hex_ZoneActive(void);
HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask,
                     uint8_t gyro_ok,     /* [原地转] 陀螺可用 */
                     float gyro_z);       /* [原地转] 角速度 °/s */
void Hex_GetDebug(const HexState *h, HexDebug *d);

#endif /* SPECIAL_H */
