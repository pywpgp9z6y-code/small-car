#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "special.h"   /* HexDebug */

/* ============================================================
 * 1 kHz 控制层（原 main.c 的 Control_Update 及其内部状态）
 *
 * 调用关系：
 *   TIM2 更新中断 -> Timebase_1kHz() -> Control_Update()
 *   主循环按键     -> Control_Start()
 *   主循环串口命令 -> Control_StartSpin() / Control_StartStraight()
 *   主循环调试打印 -> Control_GetDebug() / Control_GetHexDebug()
 * ============================================================ */

/* 1 kHz 控制入口：陀螺 200 Hz、方向环/速度环 100 Hz、电机输出。 */
void Control_Update(void);

/* 发车：清积分、复位 kick，从第一拍开始脉冲 kick。 */
void Control_Start(void);

/* 1 = 正在跑（发车后、停车前）。 */
uint8_t Control_IsRunning(void);

/* 串口命令 '2'：原地转 360°。 */
void Control_StartSpin(void);

/* 串口命令 '3'：脱线直线 1 m。 */
void Control_StartStraight(void);

/* 主循环用的调试快照，避免在中断里碰串口。 */
typedef struct {
    float    speed_cmd;
    float    speed_L, speed_R;   /* PID 实际使用的轮速，含符号 */
    int16_t  dL, dR;             /* 10 ms 编码器增量 */
    float    target_L, target_R;
    float    gyro_z;             /* 滤波后的角速度，°/s */
    float    error;              /* [DIAG400 2026-11-27] 本拍光电误差（路） */
    float    turn;               /* [DIAG400] 本拍方向环输出（counts，未含 HOLD） */
    uint8_t  near_d;             /* [DIAG400] 1 = 当前处于折角短前视模式 */
    uint8_t  run;                /* 1 = 正在跑 */
} ControlDebug;

void Control_GetDebug(ControlDebug *d);

/* 六边形入口调试快照（转发 special.c 的 Hex_GetDebug）。 */
void Control_GetHexDebug(HexDebug *d);

/* [FULLDIST 2026-11-24] 本次运行第一个全亮到当前的距离（counts）。
   返回 1 = 已经出现过全亮；0 = 还没出现过。 */
uint8_t Control_GetFullDist(uint32_t *dist);

/* [LOSTDBG 2026-11-22] 主循环取"刚丢线"快照：err10 / mask 是丢线前一拍
   （最后一条有效线）的值，spd 是当时左右轮速，corner=1 表示已在过弯状态。
   用途：手推 45°/60°/断路口，量丢线前 error，定 LOST_BEND_ERR。 */
uint8_t Control_PopLostLine(int16_t *err10, uint16_t *mask,
                            int16_t *spd_l, int16_t *spd_r,
                            uint8_t *corner);

/* [LOSS-FEAT 2026-11-22] 丢线沿的 16 拍窗口特征（只观察，不改控制）。
   slope 单位 centi-lane/拍；sign 为同号比例 0..100；edge/center 为拍数；
   side 为一侧最大连续亮路数；full=1 表示出现过一侧 6 路全亮。 */
uint8_t Control_PopLossFeat(int16_t *err10, int16_t *slope, uint8_t *sign,
                            uint8_t *edge, uint8_t *center,
                            uint8_t *side, uint8_t *full);

#endif /* CONTROL_H */
