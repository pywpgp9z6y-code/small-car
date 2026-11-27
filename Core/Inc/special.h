#ifndef SPECIAL_H
#define SPECIAL_H

#include <stdint.h>
#include "car_config.h"

/* 过弯四段状态（[APPROACH 2026-11-14]） */
typedef enum {
    CORNER_ST_LINE     = 0U,   /* 正常巡线 */
    CORNER_ST_APPROACH = 1U,   /* 冲弯：直行 CORNER_APPROACH_CM，只保航向 */
    CORNER_ST_BRAKE    = 2U,   /* 刹停 */
    CORNER_ST_PIVOT    = 3U,   /* 原地转（粗转） */
    CORNER_ST_SETTLE   = 4U,   /* [SETTLE 2026-11-15] 航向收尾：把粗转剩下的几度补掉 */
    CORNER_ST_RECOVER  = 5U,   /* [RECOVER 2026-11-16] 转完仍丢线：反向找（上限 CORNER_RECOVER_DEG） */
    CORNER_ST_ARC      = 6U,   /* [ARC 2026-11-18] 带速度弧线过弯（偏航双环） */
    CORNER_ST_ARC_SETTLE = 7U, /* [ARC2 2026-11-21] 弧线断电后的航向收尾 */
    CORNER_ST_ARC_RECOVER = 8U /* [ARC2 2026-11-21] 收尾到位但还没看到线：
                                  继续朝**原方向**补转，最多 CORNER_RECOVER_DEG(90°)
                                  （原来的 RECOVER 是反方向，用于转过头） */
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
    float    arc_recover_dir;/* [ARC-RECOVER 2026-11-24] 段8补转方向：
                                由最后有效线 bias 定：线左 -> +1，线右 -> -1 */
    uint16_t last_valid_mask;/* [ARC-RECOVER 2026-11-24] 最后一条有效线的 mask */
    uint16_t arc_ticks;      /* [ARC 2026-11-18] 弧线过弯计时 */
    uint8_t  arc_line_ticks; /* [ARC 2026-11-18] 出弯后连续看到线的拍数 */
    uint8_t  arc_brake_active; /* [ARC-BRAKE 2026-11-23] 1 = 里程内环刹车中 */
    uint16_t arc_brake_ticks;  /* [ARC-BRAKE] 刹车已执行拍数 */
    uint8_t  arc_entry_brake_done;  /* [ARC-ENTRY-BRAKE 2026-11-27] 1 = 本弯入口
                                       双轮制动已结束，本弯内不再重新进入 */
    uint16_t arc_entry_brake_ticks; /* [ARC-ENTRY-BRAKE] 已执行拍数（20 拍兜底） */
} CornerState;

/* [CWDBG 2026-11-22] 直角弯判据的一次性事件快照（中断入队，主循环打印） */
typedef struct {
    uint8_t  code;       /* 1=丢线沿 2=CW fire 3=进corner 4=候选挂起 5=候选作废 6=被冷却挡 */
    uint8_t  cnt;
    uint16_t mask;
    uint16_t cw_mask;
    uint8_t  low, wide, pend, dark, armed, cool, hz, state;
    uint16_t ticks;
    int16_t  err10;
} CwdDebug;

uint8_t Corner_PopDebug(CwdDebug *d);

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
    int16_t  net10;      /* [HEX4-NET] 有向净角 ×10 */
} HexDebug;

void Corner_Reset(CornerState *c);
uint8_t Corner_IsActive(const CornerState *c);
uint8_t Corner_IsCooling(const CornerState *c);   /* [COOLDOWN 2026-11-18] 1 = 还在出弯冷却期 */

/* [LOST3 2026-11-22] 直角弯候选：CW4 已看到"一侧全亮"并挂起等丢线确认。
   丢线第一拍 Corner_IsActive 还是 0，control 靠这个标志先让 corner 接管。 */
uint8_t Corner_IsPending(void);
/* [LOST3 2026-11-22] 直角弯补转 20° 后仍无线：control 看到就直接走 LOST_STOP，
   不再落到 45/60 的 HOLD / 90° 补转。 */
uint8_t Corner_GaveUpNoLine(void);
/* [FULLDIST 2026-11-24] 本次运行第一个“全亮”（>=10 路亮）到当前的距离（counts）。
   第一个全亮之后起点固定，不会再因后续全亮而重置。 */
uint8_t Corner_GetFullDist(const CornerState *c, uint32_t *dist);
void Corner_AddDistance(CornerState *c, uint32_t counts);   /* [APPROACH 2026-11-14] */

/* [CW2 2026-11-21] 直角弯判据的逐拍记录（串口 'c' 打印）：
   中断里只存，主循环读。用于实测 CORNER_FULL_HOLD_TICKS / 丢线拍数。 */
uint8_t Cw_LogCount(void);
void    Cw_GetAt(uint8_t i, int16_t *cnt, uint16_t *mask, int16_t *err, uint8_t *flag);
void    Cw_ClearLog(void);
uint16_t Cw_FireCount(void);       /* [CW2] 判据满足次数 */
uint16_t Cw_EnterCount(void);      /* [CW2] 真正进状态机（= 真的打舵）次数 */
uint16_t Cw_BlockArm(void);        /* [CW2] 判据满足但没武装 */
uint16_t Cw_BlockCool(void);       /* [CW2] 判据满足但在冷却期 */

/* [ISR-SAFE 2026-11-21] 过弯事件：中断里只存，主循环取出来打印。
   取到返回 1；没有事件返回 0。 */
uint8_t Corner_PopEvent(uint8_t *why, int8_t *dir, uint16_t *mask, uint16_t *cw_mask,
                        int16_t *cnt, int16_t *err10);

/* 返回最终 turn；speed_cmd 由函数写入。
   pid_turn 是方向环 PID + 二次项、限幅后的值。
   speed_meas 是编码器实测速度（左右幅值平均），旧刹车段用它判停。
   speed_center 是带符号的中心速度 ((vL+vR)/2)，ARC 入口双轮制动用它判交接。 */
float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint16_t mask,      /* [CW 2026-11-15] 12 路原始掩码：bit i = 通道 i 压线 */
                    uint8_t gyro_ok,
                    float gyro_z,
                    float speed_meas,
                    float speed_center, /* [ARC-ENTRY-BRAKE] 带符号中心速度 */
                    uint8_t flow_fire,  /* [FLOW-OR] 1 = FLOW 请求触发 ARC */
                    float flow_dir,     /* [FLOW-OR] +1 左 / -1 右 */
                    float *speed_cmd);

void Hex_Reset(HexState *h);
void Hex_AddDistance(HexState *h, uint32_t counts);

/* [TURNBIAS 2026-11-18] 1 = 当前处于六边形区间（转向增益可放大） */
uint8_t Hex_ZoneActive(void);

/* [ARMED-COAST 2026-11-27] 1 = 六边形状态机处于 ARMED：
   已经看到第一次 V（HEX,4），正在走 HEX4_ARM_DIST_CM(9 cm) 等确认，
   还没有进入 TURN_IN。
   **关键语义**：这段时间车其实还在按普通巡线走，**人不在六边形内部**，
   后面既没有 60° 顶点，也没有"内轮要反转"的需求。所以 control 里所有
   "六边形内部"的假设在这里都不成立 —— 最典型的就是 HOLD：它是为
   "45°/60° 弯丢线、需要继续往弯里加舵到 TURN_MAX"设计的，
   在 ARMED 期接管只会把车朝 V 的方向越带越偏，30° 打舵又叠加在错误姿态上。
   control 用这个标志在丢线时改走"冻结最后一拍舵 + 保持车速"的滑行，
   让 9 cm 正常走完，由状态机自己判 TURN_IN 还是解除武装。
   注意它与 Hex_ZoneActive() 不是同一个东西：ARMED 期 zone 已经是 1。 */
uint8_t Hex_IsArmed(void);
HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask,
                     uint8_t gyro_ok,     /* [原地转] 陀螺可用 */
                     float gyro_z);       /* [原地转] 角速度 °/s */
void Hex_GetDebug(const HexState *h, HexDebug *d);

/* [HEX4 2026-11-21] 六边形"两次 V"状态机（按现场描述重写）。
   与老 Hex_Update 的区别见 special.c 里的注释。
   error / pid_turn 用来判"大误差方向"和"第一次 V 之后的回正"。 */
HexOutput Hex_Update4(HexState *h,
                      CornerState *corner,
                      uint8_t center,
                      uint8_t right,
                      uint8_t left,
                      uint16_t mask,
                      uint8_t gyro_ok,
                      float gyro_z,
                      float error,
                      uint8_t flow_fire,   /* [HEX4-FLOW-VETO] FLOW 本拍触发请求 */
                      float flow_dir,      /* +1 左 / -1 右 */
                      float pid_turn);

void Hex4_Reset(void);     /* 发车/停车时清状态，Hex_Reset 里会调用 */

/* [HEX4 2026-11-21] 六边形事件队列：中断里入队，主循环按序打印（不会丢事件） */
uint8_t Hex_PopEvent(uint8_t *code, uint32_t *dist, uint16_t *mask,
                     uint8_t *right, uint8_t *left, uint8_t *center, uint16_t *seq);

/* [HEX4] V 判据的逐拍记录（串口 'h' 打印），用于重新设计判定条件 */
uint8_t HexLog_Count(void);
void    HexLog_GetAt(uint8_t i, int16_t *cnt, uint16_t *mask, int16_t *err, uint8_t *flag);
void    HexLog_Clear(void);

/* [HEX4-TURNBACK 2026-11-25] TURN_IN 转完（30°）仍没看到线时改成反向转回：
   返回 1 = 本拍刚刚结束反向转回（看到线，或转满放弃）。control 用它在同一拍
   把 HOLD/补转的旧舵清掉——这一支本身不走 HOLD，HOLD 对其余丢线继续生效。 */
uint8_t Hex_TakeTurnBackDone(void);

/* ================= [FLOW 2026-11-23] 流动直角弯检测 =================
   第一阶段只并行观察，不接管电机。100 Hz 调用
   Flow_Update(line.mask, line.raw_cnt, line.raw_mask)，主循环用 Flow_PopEvent() 取事件打印 FLOW 行。
   方向约定：+1 = 左转，-1 = 右转，0 = 十字/发车线。 */
typedef enum {
    FLOW_EVT_NONE  = 0U,
    FLOW_EVT_LEFT  = 1U,
    FLOW_EVT_RIGHT = 2U,
    FLOW_EVT_CROSS = 3U
} FlowEventType;

typedef struct {
    uint8_t  type;        /* FlowEventType */
    int8_t   dir;         /* +1 左 / -1 右 / 0 十字 */
    uint16_t new_mask;    /* 刚加的通道（bit i = ch i） */
    uint16_t run_mask;    /* 当前主连续段 */
    uint8_t  run_l;       /* 当前主段左端 */
    uint8_t  run_r;       /* 当前主段右端 */
    uint8_t  prev_l;      /* 上一拍主段左端 */
    uint8_t  prev_r;      /* 上一拍主段右端 */
    int8_t   width_jump;  /* 当前主段宽度 - 上一拍主段宽度 */
    uint8_t  new_l_cnt;   /* 新通道里落在左半边的数量 */
    uint8_t  new_r_cnt;   /* 新通道里落在右半边的数量 */
} FlowEvent;

/* [FLOW-SNAP] 检测到左/右直角弯后，自动保存 1.28 s 窗口。
   中断里只拷贝 mask，不 printf；主循环 Flow_PopSnapshot() 取出来后打印。 */
typedef struct {
    FlowEvent event;
    uint8_t   num;                    /* 有效拍数，最多 FLOW_WIN_TICKS */
    uint16_t  mask[FLOW_WIN_TICKS];   /* 按时间顺序：mask[0] 最旧 */
} FlowSnapshot;

typedef struct {
    uint8_t  reason;      /* 0=真丢线 1=单路被过滤 2=主段换段 */
    uint8_t  raw_cnt;     /* 过滤前亮灯路数 */
    uint16_t raw_mask;    /* 过滤前 mask */
    uint16_t prev_mask;   /* Flow 上一拍的主段 */
    uint16_t cur_mask;    /* 当前主段；0 表示本拍没有主段 */
    uint8_t  prev_l;
    uint8_t  prev_r;
} FlowDiag;

void    Flow_Reset(void);
void    Flow_Update(uint16_t mask, uint8_t raw_cnt, uint16_t raw_mask);
uint8_t Flow_PopEvent(FlowEvent *e);
uint8_t Flow_PopSnapshot(FlowSnapshot *s);
uint8_t Flow_PopDiag(FlowDiag *d);
uint8_t Flow_TakeFire(float *dir);   /* [FLOW-OR] 取一次性 ARC 触发请求 */

/* 1.28 s 窗口转储（串口 'f'） */
uint8_t Flow_LogCount(void);
void    Flow_GetAt(uint8_t i, uint16_t *mask, uint8_t *l, uint8_t *r, uint8_t *width);
void    Flow_ClearLog(void);
void    Flow_LogPause(uint8_t en);   /* 主循环转储时暂停写入 */

#endif /* SPECIAL_H */
