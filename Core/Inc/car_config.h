#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ============================================================
 * 小车参数集中配置
 * 说明：这里只放“可调参数”，不放硬件句柄和函数实现。
 * 后续 main.c / motor.c / special.c / timebase.c 共用本文件。
 * ============================================================ */

/* ---- PWM / 速度环基础 ---- */
#define PWM_MAX                 3000.0f
#define SPEED_DIV               10U      /* 1 kHz / 10 = 100 Hz 速度环 */
#define INTEG_LIMIT             30000.0f

/* ---- 直道 / 方向环 ---- */
#define BASE_SPEED              120.0f    /* [SPEED 2026-11-16] 70 -> 120（≈27 cm/s）。
                                             提速试验：直道目标 27 cm/s，弯道仍由 pivot 处理。
                                             注意：120 时方向环满舵(52)的最小半径约 14 cm，
                                             六边形 60° 折角可能偏大，先看表现。 */
#define TURN_MAX                55.0f     /* [TURN 2026-11-15] 25 -> 55。
                                            turn=25 时两轮 45/95、ω 只有 41°/s、最小半径 22 cm，
                                            60° 折角根本拐不过来。55 时两轮 15/125、ω≈91°/s、半径≈10 cm，
                                            内轮还有余量、不会停转打滑。上限是 70（内轮=0），
                                            再大内轮要反转，变成打滑原地转。 */
#define CURVE_SLOW              0.0f//0.5f
#define MIN_CURVE_SPEED         0.0f//20.0f
#define DIR_QUAD                1.0f     /* [TURN 2026-11-15] 0.21 -> 1.0。
                                             原来 e=5.5 时 turn 只有 20.1，方向环永远打不满；
                                             现在 e=5.5 时 = Kp×5.5 + 1.0×30.25 ≈ 52（接近 TURN_MAX） */
#define DIR_KP                  4.0f     /* [TURN 2026-11-15] 方向环比例增益（原来写死在 motor.c，2.5） */
#define DIR_DAMP                0.25f    /* [DAMP 2026-11-16] 陀螺阻尼：turn -= DIR_DAMP × gyro_z。
                                             车转 100°/s 时收回 25，抑制中心附近的来回摆。
                                             过大会"转不动"，建议 0.15~0.30 */
#define TURN_SLEW               5.0f     /* [SLEW 2026-11-16] 转向命令每拍最大变化量。
                                             正常进弯天然约 1.7/拍，不会介入；
                                             只在误差突然翻符号时削掉阶跃（52→-52 要 21 拍） */

/* ---- 通用过弯锁存 ----
   [GND 2026-11-15] 135°(航向变45°) 也会压到 4 路宽线，和 90° 在这一刻无法区分，
   所以阈值提到“方向环接近饱和”的位置：只有真拐不过去才锁存。 */
#define CORNER_ENTER_ERR        4.5f     /* 3.0 -> 4.5：线到阵列边缘才算 */
#define CORNER_ENTER_TURN       40.0f    /* [TURN 2026-11-15] 18 -> 40：TURN_MAX 提到 55 后，
                                            阈值要同比提高，否则普通弯道稍一偏就触发过弯。
                                            turn≈40 对应 error≈8，即方向环已经打到七成以上 */
/* [GND 2026-11-14] 缓弯 vs 直角弯的区分：
   触发那一刻两者的 error/turn 完全一样，区别在“变化速度”——
   直角弯的线是几拍内跳到边缘，缓弯是慢慢漂。 */
#define CORNER_ENTER_RATE       0.15f    /* 已不参与锁存判据（derr 会被自身打舵触发），保留供参考 */
#define CORNER_ENTER_CNT        5U       /* 4 -> 5：45° 折角一般只压 3~4 路，直角弯口才到 5 路以上 */
#define CORNER_ARM_TICKS        15U      /* [FIX 2026-11-16] 30 -> 15（0.15 s）。
                                            实测 120正方形第二角失败：过完第一个弯后车一直有偏置，
                                            误差压不到 2.0 以下满 0.3 s -> 永不武装 -> 第二个弯全被挡住。
                                            缩短时间窗，但阈值同时放宽（见下）。 */
#define CORNER_ARM_ERR          3.0f     /* [FIX 2026-11-16] 2.0 -> 3.0。
                                            0.15 s 内误差低于 3.0 就算"回到线附近"，避免短直道上
                                            "永远等不到重新武装"。仍比锁存阈值 4.5 紧，不会来回连转。 */

/* ---- 直角弯专职检测（[CW 2026-11-15]） ----
   指纹：一侧连续全亮（压到弯口）→ 紧接连续全黑（线甩出阵列）。
   缓弯/直道不会出现“宽线之后全黑”，所以这个判据比 |error|>3 更适合直角弯。
   状态：IDLE -> WIDE（看到单侧宽线）-> 等全黑确认 -> 触发锁存。 */
#define CW_SIDE_RUN         4U       /* 一侧连续亮 >=4 路算“压到弯口” */
#define CW_DARK_TICKS       2U       /* 全黑连续 2 拍才确认（防单帧干扰） */
#define CW_LOW_TICKS        6U       /* 宽线后又恢复正常(<=2路) 6 拍 -> 判定不是弯 */
#define CW_WIDE_TIMEOUT     80U      /* 宽线状态最多持续 80 拍(0.8s)，超时复位 */
#define CORNER_SPEED            25.0f    /* [PIVOT 2026-11-14] 恢复 25：仍供六边形用；过弯 pivot 分支自己写 0 */
#define CORNER_EXIT_SPEED       0.0f     /* [PIVOT 2026-11-14] pivot 结束时车速为 0，出弯斜坡从这里起 */
#define CORNER_YAW_EXIT         70.0f
#define CORNER_MIN_TICKS        12U      /* 120 ms（pivot 方案下已不使用） */
#define CORNER_MAX_TICKS        200U     /* [GND 2026-11-14] 80->200：实测 ω≈57°/s，转 82° 要 144 拍；
                                            80 拍只够 46°——每次被超时掐断->再触发->反向再转 */
#define CORNER_COOLDOWN         40U      /* 200->400 ms：pivot 后给车时间回中，避免立刻重复锁存 */
#define CORNER_RAMP_TICKS       15U      /* 150 ms */

/* ---- 原地 pivot 过弯（[PIVOT 2026-11-14] 新增） ----
   speed_cmd 清零，两轮等幅反向 => 绕两轮中点自转。
   退出用陀螺转角，不再用计时。 */
#define CORNER_TURN_DEG         90.0f    /* [GND 2026-11-14] 86->90：改成“先直冲再原地转”后，
                                            检测时车是直的、没转过，少掉的 8° 就是“角度不够” */
#define CORNER_ANGLE_TOL        8.0f     /* ° 粗转提前量：4 -> 8。
                                            粗转在 86° 断电还会滑 5~8°，常常落在 92~95°；
                                            提前到 82° 断电，滑行 + 收尾轻推正好落在 90°。
                                            收尾段会把剩下的角度补上，所以不会少转。 */
#define PIVOT_TARGET            45.0f    /* [GND 2026-11-14] 32->45：ω 由 57°/s 提到 ~80°/s，82° 约 1.0 s */
#define PIVOT_KICK              55.0f    /* [GND 2026-11-14] 42->55：必须大于 PIVOT_TARGET(45)，否则起转脉冲比稳态还弱 */
#define PIVOT_KICK_TICKS        15U      /* 150 ms */

/* [SAFE 2026-11-14] pivot 堵转保护：两轮都不动就别再灌电流了 */
#define PIVOT_STALL_SPEED       5.0f     /* |实测速度| 低于此值算“不动” */
#define PIVOT_STALL_TICKS       40U      /* 连续 400 ms 不动 -> 提前结束 pivot */

/* ---- 航向收尾（[SETTLE 2026-11-15]） ----
   pivot 是“粗转”：断电后车还会滑转几度，转快转慢差别很大（4° 的固定补偿只在
   某一个转速下刚好）。收尾段用低速小差速把剩余角度补掉，保证每个弯都落在
   yaw_ref±1.5°，连续弯也不会累积误差。 */
#define CORNER_SETTLE_TOL       1.5f     /* 到位容差，° */
#define CORNER_SETTLE_RATE      20.0f    /* |角速度| 低于此值算“已停住”，°/s */
#define CORNER_SETTLE_MAX_TICKS 30U      /* 收尾最多 300 ms，超时放弃 */
#define CORNER_RECOVER_DEG      30.0f    /* [RECOVER 2026-11-16] 转完 90° 仍丢线时，
                                            反方向原地找线的最大角度，° */
#define CORNER_RECOVER_TICKS    80U      /* 找线最多 800 ms（40° 左右，兜底） */
#define CORNER_SETTLE_MAG       30.0f    /* 收尾用的差速（比 PIVOT_TARGET 小，动作更细） */
#define CORNER_SETTLE_COAST_RATE 35.0f   /* [JUDDER 2026-11-15] |角速度| 还高于此值就先断电滑行：
                                            在高速旋转时通电修正 = 反接制动，电流数倍 -> 抖动 */
#define CORNER_COAST_T          0.06f    /* 预测断电后还会滑转的时间，s（用于提前断电） */

/* [GND 2026-11-14] 转向差速前馈：
   速度前馈从 14.5 降到 3.2 后，turn=2 只产生 6.4 占空比差，
   远小于左右电机的 50% 效率差 -> 方向环的小修正被死区吞掉，
   车会一直偏到 error>3 才猛打一次。这里单独给“差速”一个增益。 */
#define DIR_DIFF_GAIN           10.0f    /* 每单位 turn 额外的占空比差 */

/* ---- 六边形入口（[HEX 2026-11-15] 整理） ----
   HEX_MODE 三档：
     0 = 关闭：只更新调试快照，不产生事件、不接管（当前默认）
     1 = 只记录：照常产生事件并打印 HEX 行，但**不接管舵量**——用来在真实赛道上
         验证"六边形顶点"和"直角弯"的波形差异，确认不误触发后再切 2
     2 = 接管：事件确认后按 HEX_V_MAX_TICKS 拍打舵覆盖
   注意：判据里的 ch11/ch0 + center 在直角弯上同样成立，所以 1 档是必要的验证步骤。 */
#define HEX_MODE                2
#define COUNTS_PER_CM           435.7f

/* ---- 六边形入口：V 型判据（[HEX-V 2026-11-15]） ----
   通道分组：a = ch0,1 / ch10,11（两端）
             b = ch2,3,4 / ch7,8,9（两中段，**三路全亮才算 b 存在**；任意一路暗即 !b）
             c = ch5 & ch6（正中）
   一次 V = 某侧 a 亮 && c 亮 && 同侧 b 不完整（中间有缺口）。
   [实测 2026-11-15 手推] 六边形**每个顶点只产生一次 V**，相邻顶点 10~16 cm，
   不存在"两次 V 相隔 2 cm"——所以按单次 V 直接触发。
   直角弯由过弯状态机优先处理（hex 覆盖本来就被 !Corner_IsActive 挡住）。 */
#define HEX_V_TURN              25.0f    /* 触发后的打舵量（旧"边走边转"方案，保留备用） */
#define HEX_V_SPEED             25.0f    /* 旧方案的速度目标（保留备用） */
#define HEX_V_PIVOT_TURN        45.0f    /* [原地转方案] 两轮等幅反向的差速 */
#define HEX_V_DEG               30.0f    /* [原地转方案] 目标转角，°，用陀螺闭环 */
#define HEX_V_DEG_TOL           4.0f     /* 角度容差，° */
#define HEX_V_MAX_TICKS         60U      /* 原地转最多 600 ms（兜底） */
#define HEX_V_MIN_TICKS         8U       /* 至少 8 拍才允许判"摸到线"（防触发那一拍立刻结束） */
#define HEX_V_GAP_MIN_CM        10.0f    /* [纯距离触发] 武装后走这么远就开始原地转。
                                            实测：手推 10.3 cm、自跑 10.2 cm，间距非常稳定。
                                            不用 c/b 判据——实测 9 cm 之后 c 必灭，判据自相矛盾 */
#define HEX_V_GAP_MAX_CM        16.0f    /* 兜底：超过这个距离还没触发就解除武装 */
#define HEX_V_GAP_MIN_COUNTS    ((uint32_t)(HEX_V_GAP_MIN_CM * COUNTS_PER_CM))
#define HEX_V_GAP_MAX_COUNTS    ((uint32_t)(HEX_V_GAP_MAX_CM * COUNTS_PER_CM))
#define HEX_V_B_MAX             1U       /* [FIX 2026-11-16] b 段最多亮几路才算"缺口"。
                                            2 -> 1：b 只有 3 路，亮 2 路意味着只暗 1 路——
                                            单路漏读就能伪造一个 V（实测 直角-六边耦合1：
                                            mask=117=ch0,2,4,5,6，ch1/ch3 漏读，被当成缺口）。
                                            改成 1 后要至少暗 2 路才算缺口。 */

/* ---- 六边形区间（[HEX-ZONE 2026-11-15]） ----
   [实测] 六边形**内部 6 个顶点不产生 V**（60° 折角线是连着的，不出现"端点亮+正中亮+中间缺口"）；
   V 只在**入口**和**出口**出现。所以区间不能靠"见到 V 续命"，必须一次覆盖整圈。
   六边形一圈约 120 cm，取 150 cm（含余量）。
   区间内**禁用直角弯检测**（CW + 通用锁存），否则六边形内部的宽线会被当成直角弯。 */
#define HEX_ZONE_CM             150.0f
#define HEX_ZONE_COUNTS         ((uint32_t)(HEX_ZONE_CM * COUNTS_PER_CM))

/* ---- 过弯四段式：冲弯 -> 刹停 -> 原地转（[APPROACH 2026-11-14] 新增） ----
   CORNER_APPROACH_CM 是“从检测到弯 -> 车停稳”的总位移。
   刹车区按 v^2/(2a) 反算，所以停点正好落在 CORNER_APPROACH_CM 上，而不是再加一段刹车距离。
   [2026-11-14] 7 -> 10 -> 8 cm：10 cm 会冲过弯口直接丢线，8 cm 是折中。 */
#define CORNER_APPROACH_CM      8.0f     /* [GND 2026-11-14] 10 -> 8.0：10 cm 会冲过弯口直接丢线 */
#define CORNER_APPROACH_COUNTS  ((uint32_t)(COUNTS_PER_CM * CORNER_APPROACH_CM))
#define CORNER_APPROACH_SPEED   (1.0f * BASE_SPEED)   /* [SPEED 2026-11-15] 0.5 -> 1.0。
                                            原因：目标 35 时前馈只有 112，越不过死区 150，
                                            速度环平衡点掉到 14，靠积分补 1.7 s——现场看到"爬 1.8 秒"。
                                            目标 70 时前馈 224 刚好越过死区，平衡点≈70。 */
#define CORNER_APPROACH_KYAW    1.0f     /* 航向保持 P，turn/° */
#define CORNER_APPROACH_KD      0.05f    /* 角速度阻尼，turn/(°/s) */
#define CORNER_APPROACH_TURNMAX 8.0f     /* 冲弯段最大修正舵，别让它拐起来 */
#define CORNER_APPROACH_MAX_TICKS 300U   /* [8cm] 1.8->3.0 s：低速冲 8 cm 要 ~230 拍，原 180 会超时 */
#define CORNER_APPROACH_RAMP_DEC 3.0f    /* [GND 2026-11-14] 冲弯段速度每拍下降量：
                                            从巡航 70 平滑降到 35，避免一刀切导致前馈跌破死区、车一顿 */

/* [KICK 2026-11-14] 冲弯段破静摩擦（只在 APPROACH 内部生效，不动 launch kick / Motor_Output）
   静止时 target=35 只给 ~200 占空比，而起转要 390+，车会卡住靠积分慢慢爬。
   做法：连续 WAIT 拍速度上不去，就把目标速度临时抬到 BREAKAWAY_SPEED 给 TICKS 拍，
   借速度环自己的前馈产生 ~300~500 的起转脉冲；最多试 TRIES 次。 */
#define CORNER_BREAKAWAY_SPEED   90.0f   /* 临时目标速度：FF≈288 + P，足够破静摩擦 */
#define CORNER_BREAKAWAY_TICKS   6U      /* 每次脉冲 60 ms */
#define CORNER_BREAKAWAY_STALL   3.0f    /* |实测速度| 低于此值算“没动” */
#define CORNER_BREAKAWAY_WAIT    6U      /* 连续 60 ms 没动就踹一次 */
#define CORNER_BREAKAWAY_TRIES   3U      /* 最多 3 次，避免一直顶 */
#define TARGET_MAX              400.0f   /* [SAFE 2026-11-14] 目标值护栏：不允许被推到几百 */

#define CORNER_BRAKE_STEP       5.0f     /* 减速度：每拍掉 5 counts/10ms */
#define CORNER_BRAKE_COUNTS     ((uint32_t)((CORNER_APPROACH_SPEED * CORNER_APPROACH_SPEED) / (2.0f * CORNER_BRAKE_STEP)))
#define CORNER_BRAKE_STOP       3.0f     /* |实测速度| < 此值算停稳 */
#define CORNER_STOP_ZONE        150U     /* [FIX 2026-11-16] 50 -> 150（≈3.4 mm）。
                                            实测 120正方形第二角失败：刹车段速度已经到 0，
                                            但 remain 还 >50，只能等 BRAKE_MAX(0.6 s) 超时。
                                            放宽到 3.4 mm 能让刹车段提前 0.4~0.5 s 结束。 */
#define CORNER_BRAKE_MAX        60U      /* 600 ms 兜底 */
#define CORNER_SETTLE_TICKS     3U       /* 停稳后再等 30 ms 才转 */

/* ---- 原地转 360°（90/180/270/360 分段） ---- */
#define SPIN_TARGET_DEG         360.0f
#define SPIN_STOP_DEG           358.0f
#define SPIN_KP                 0.6f
#define SPIN_MAX                35.0f
#define SPIN_MIN                30.0f
#define SPIN_KICK               42.0f
#define SPIN_KICK_TICKS         18U
#define SPIN_TIMEOUT            1000U

/* ---- 脱线直线 1 m ---- */
#define STRAIGHT_TARGET_COUNTS  43570U
#define STRAIGHT_SPEED          60.0f
#define STRAIGHT_MIN_SPEED      20.0f
#define STRAIGHT_DECEL_COUNTS   2178U
#define STRAIGHT_KP             1.0f
#define STRAIGHT_KD             0.1f
#define STRAIGHT_TURN_MAX       10.0f
#define STRAIGHT_TIMEOUT        1000U

/* ---- 前馈 / 死区 ----
   [FF 2026-11-14] 前馈按工况拆成两套。
   实测：直道自由滚动 k≈0.60，原地转（两轮互顶+横刮）k≈0.081，负载差 7 倍。
   一套线性前馈不可能同时对上——原来 14.5 在 pivot 正好，在直道超速 4 倍。 */
#define FF_DRIVE_L              2.27f    /* [SPEED 2026-11-16] BASE 70->120 重标定。
                                             由实测两点 (target70,CCR≈220)、(target35,CCR≈150) 拟合：
                                             CCR ≈ 134 + 1.148×speed  =>  target 120 需要 CCR≈272
                                             272/120 = 2.27。原来的 3.2 在 120 时会给 384，严重过驱 */
#define FF_DRIVE_R              2.41f    /* 右轮略弱，按原比例 3.4/3.2 折算 */
#define FF_DRIVE_L_REV          2.55f
#define FF_DRIVE_R_REV          2.71f

#define FF_PIVOT_L              14.5f    /* 原地转：沿用原标定 */
#define FF_PIVOT_R              15.4f
#define FF_PIVOT_L_REV          18.0f    /* 左轮效率只有右轮 53%，反转要多给 */
#define FF_PIVOT_R_REV          15.4f

#define MOTOR_MIN_DUTY_L        150.0f
#define MOTOR_MIN_DUTY_R        169.0f
/* [GND 2026-11-14] 目标太小时不做死区补偿：
   刹车段目标会降到 0.4、1.2，原来会被强行抬到 150 -> 窜一下->停->再窜（停车抖动）。 */
#define MOTOR_MIN_TARGET        4.0f

/* ---- 反向死区（[REV 2026-11-14] pivot 时总有一侧是反转） ---- */
#define MOTOR_MIN_DUTY_L_REV    150.0f
#define MOTOR_MIN_DUTY_R_REV    169.0f

/* ---- 速度环增益（[BAL 2026-11-14] pivot 期间加强，出弯恢复） ----
   原来 Kp=2.5 对 ~480 的前馈几乎没有权威，压不住跑飞的轮子。 */
#define SPEED_KP_DEFAULT        2.5f
#define SPEED_KI_DEFAULT        0.01f
#define PIVOT_KP                3.0f     /* [JUDDER 2026-11-15] 8 -> 3。pivot 时两轮被地面刚性耦合，
                                            两个独立速度环互相抢；Kp=8 时 CCR 在 376~508 拉锯（5~10 Hz 抖动） */
#define PIVOT_KI                0.02f    /* [JUDDER 2026-11-15] 0.05 -> 0.02，积分放慢 */

/* ---- 起步 kick ---- */
#define KICK_L                  390.0f
#define KICK_R                  428.0f
#define KICK_ON_TICKS           6U
#define KICK_OFF_TICKS          6U
#define KICK_MAX_TICKS          60U
#define KICK_EXIT_SPEED         4.0f
#define SETTLE_TICKS            15U

/* ---- 卡死恢复 ---- */
#define STALL_SPEED             2.0f
#define STALL_TICKS             30U
#define STALL_RETRIES           3U

/* ---- 丢线 / 编码器保护 ---- */
#define LOST_STOP_TICKS         50U      /* [FIX 2026-11-16] 80 -> 50（0.5 s）。
                                            实测丢线后要 1.4 s 才停（中间有线闪回把计数清了）；
                                            0.5 s 在 BASE 120 下约 13 cm，够短。 */
#define ENC_DELTA_LIMIT         500      /* [SPEED 2026-11-16] 300 -> 500：BASE 120 时正常拍
                                             增量就到 120，加速/弯道峰值可能到 200+，
                                             300 太紧会误杀真数据 */

/* ---- 低速增益调度 ---- */
#define KP_LOW_SPEED            10.0f
#define KP_BOOST                2.5f

/* ---- 陀螺 ---- */
#define GYRO_ALPHA              0.3f
#define GYRO_SAMPLE_DT          0.005f   /* [YAW 2026-11-15] 采样周期 = 5 ms（TIM2 的 5 拍） */

#endif /* CAR_CONFIG_H */
