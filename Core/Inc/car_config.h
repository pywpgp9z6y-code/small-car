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
#define INTEG_LIMIT             3000.0f  /* [FIX 2026-11-21] 30000 -> 3000：KI=0.01 时积分权限
                                            就是 ±30 占空比，够补死区和左右轮差异了。
                                            原来 ±300 会在速度波动时把 CCR 拉到 200~1100
                                            （实测「高速 1/直线微抖」），是最稳那版之后
                                            才被改坏的几个点之一，先退回 3000。 */

/* ---- 直道 / 方向环 ---- */
#define BASE_SPEED              300.0f    /* [SPEED300 2026-11-21] 150 -> 300（0.69 m/s）。
                                            注意：这一步只做"速度链"，TURN_MAX 还是 120，
                                            300 下的最小转弯半径约 20 cm，45° 弯过不去。
                                            验证请只跑直道（看 CCR 是否落在 438 附近），
                                            弯道等第二步放开转向权限。 */
#define TURN_MAX                250.0f    /* [SPEED300 2026-11-21] 120 -> 250。
                                            几何账：前视 d 处侧偏 3.3 cm 能表达的最小半径
                                            R = d²/(2×3.3)。前视 8 cm 时 R_min = 9.7 cm，
                                            对应 ω = 0.69/0.097 = 7.1 rad/s = 408°/s
                                            -> turn = 408/1.644 = 248，所以 250 才够跑 45° 弯。
                                            内轮 = 300 − 250 = 50，仍然不反转。 */
#define TURN_GAIN_BOOST         2.3f      /* [TURNBIAS 2026-11-18] 六边形区间内的增益倍率。
                                            平时 e=5.5 的 turn 上限是 52（不打扰直道）；
                                            区间内 ×2.3 -> 约 120，正好用满内轮=0 的能力。 */

#define CURVE_SLOW              0.3f     /* [CORNER45 2026-11-21] 0 -> 0.3：speed = BASE − 0.3×|turn|。
                                            45° 时线扫过阵列只有 ~9.5 拍，TURN_SLEW 20/拍 最多到 190，
                                            而弯里需要 250。降速是为了**多拿几拍**（不是减小半径——
                                            "内轮不反转"约束下 R_min 恒为 8 cm，和速度无关）。
                                            直道 |turn|≈10 时速度只降到 297，不影响直道。
                                            这是权宜之计：真正的解法是 TURN_SLEW 按误差调度。 */
#define MIN_CURVE_SPEED         0.0f//20.0f
#define DIR_QUAD                1.0f     /* [TURN 2026-11-15] 0.21 -> 1.0。
                                             原来 e=5.5 时 turn 只有 20.1，方向环永远打不满；
                                             现在 e=5.5 时 = Kp×5.5 + 1.0×30.25 ≈ 52（接近 TURN_MAX） */
#define DIR_KP                  4.0f     /* [TURN 2026-11-15] 方向环比例增益（原来写死在 motor.c，2.5） */

/* ---- 方向环模式（[KINEMA 2026-11-18]） ----
   0 = 旧方案：Kp·e + QUAD·e²（需要试凑三个增益）
   1 = 运动学映射：误差 → 曲率 → 角速度 → 差速（只有一个参数：前视距离） */
#define DIR_MODE                1
#define PURSUIT_L_CM            10.0f    /* [CENTER 2026-11-18] 12 -> 10：回中太慢导致连续弯漂移。
                                            增益 11 -> 16。直道抖动已由 DIR_DIFF_MAX 钳位解决，
                                            不再需要靠砍增益来压。 */
#define PURSUIT_L_CM_HEX        7.5f     /* 六边形区间内用更短的前视（更激进）：
                                            turn = 13.3·120·5.5/56 ≈ 157 -> 钳到 120（R=8 cm） */
#define PURSUIT_L_CM_NEAR       8.0f     /* [SPEED300 2026-11-21] 大误差（进弯）时的前视。
                                            前视 d 能表达的最小半径 R = d²/(2×3.3 cm)：
                                              d=14.1 -> R≥30 cm    d=10 -> R≥15 cm
                                              d=8    -> R≥9.7 cm
                                            45° 弯半径 8~12 cm，长前视在几何上就表达不出来，
                                            所以 |err|>2 时切到 8 cm；直道仍用长前视。 */
#define CORNER_NEAR_ERR         2.0f     /* [SPEED300 2026-11-21] 切短前视的误差门限（路）。
                                            调大 -> 更晚进"弯道模式"，直道更稳；
                                            调小 -> 更早变有力，但直道容易抽。 */
#define ERR_FILTER_ALPHA        0.30f    /* [CENTER 2026-11-18] 0.25 -> 0.30：折中，
                                            0.25 的延迟让回中更慢 */
#define OMEGA_KP                0.5f     /* 角速度环增益：turn / (°/s) */
#define OMEGA_PER_TURN          1.644f   /* 1 单位 turn 对应的角速度，°/s（纯几何） */
#define CURVE_K                 13.3f    /* turn = CURVE_K · v · e / d²（v 用 counts/10ms） */
#define DAMP_K                  0.4f     /* [ARC 2026-11-18] 巡线纯阻尼：turn -= DAMP_K·ω。
                                            不再用 K·(ω_des−ω)，那会把位置增益放大 82%。 */

/* ---- 直角弯模式（[ARC 2026-11-18]） ----
   0 = 老的停车方案：APPROACH → BRAKE → PIVOT → SETTLE → RECOVER
   1 = 带速度弧线：CW 看到宽线就触发，用陀螺双环把车转 90°，全程不停车 */
#define CORNER_MODE             1
#define ARC_WIDE_TICKS          3U       /* 宽线持续这么多拍就触发（不等全黑，提前 ~5 cm） */
#define ARC_YAW_DEG             90.0f    /* 目标偏航角，° */
#define ARC_YAW_TOL             2.0f     /* 角度容差，°（ARC2 下不再使用） */
#define ARC_K_YAW               4.0f     /* 旧偏航外环（ARC2 下不再使用） */
#define ARC_OMEGA_MAX           220.0f   /* 旧角速度上限（ARC2 下不再使用） */
#define ARC_K_OMEGA             0.3f     /* 旧偏航内环（ARC2 下不再使用） */
#define ARC_TURN_MAX            120.0f   /* 旧转向上限（ARC2 下不再使用） */

/* [ARC2 2026-11-21] 过弯方式：外轮不变速，内轮主动倒转。
   speed = (外+内)/2,  turn = (外-内)/2
   -> 外轮 = ARC_OUTER_SPEED（300，不变速）
      内轮 = ARC_INNER_SPEED（-70，倒转）
   Δv = 370 counts/10ms，ω = 304°/s，转 90° 约 0.30 s，半径约 5 cm。 */
#define ARC_OUTER_SPEED         300.0f   /* 过弯时外轮保持的速度（= BASE） */
#define ARC_INNER_SPEED        (-70.0f)  /* [ARC2 2026-11-21] 内轮目标：负 = 主动倒转。
                                            为什么不用 0：内轮是被车拖着走的，反向脉冲
                                            再大也只是和地面抢轮子（实测 -40 都没明显改善）；
                                            让电机主动倒转，直接抵消拖拽，不受电压/地面影响。
                                            -70 是以前 PIVOT-C 验证过的值。
                                            调法：转过头/切内道 -> -50；转不够 -> -90。 */
#define ARC_LEAD_DEG            7.0f     /* 提前这么多度就断电，靠惯性走完。
                                            246°/s 断电后还会滑 5~8°，不提前就会转过头。 */

/* [ARC2 2026-11-21] 弧线断电后的航向收尾。
   ARC_SETTLE_ENABLE = 0 就是回滚：断电后直接交回巡线（旧行为，一行开关）。 */
#define ARC_SETTLE_ENABLE       1        /* 1 = 断电后先闭环补角，再交回巡线 */
#define ARC_SETTLE_K            2.0f     /* 补角增益：turn = K × 剩余角度（°/°） */
#define ARC_SETTLE_KD           0.10f    /* 角速度阻尼：进收尾那一拍车还在 ~300°/s 旋转，
                                            只有 P 项会继续给它加正舵。减去 KD×ω 才刹得住。 */
#define ARC_SETTLE_TOL          2.0f     /* 角度容差：|剩余| < 2° 就算到了 */
#define ARC_SETTLE_RATE         20.0f    /* 角速度容差：|ω| < 20°/s 才算停住 */
#define ARC_SETTLE_MAX_TICKS    30U      /* 最多补 30 拍（0.3 s），超时强制交回 */
#define ARC_SETTLE_TURN_MAX     60.0f    /* 补角时的舵量上限，别为了 20° 打满舵 */
#define ARC_SETTLE_SPEED        115.0f   /* 收尾时的前进速度（= 弧线期间的中心速度） */
#define ARC_CREEP_SPEED         50.0f    /* [PIVOT-C 2026-11-18] 25 -> 50：转弯太早。
                                            25 时转弯期间只前进 2.6 cm，在弯口前就把 90° 转完了；
                                            50 时前进 5.3 cm，正好"扫"到弯口。
                                            顺带好处：内轮从 -95 变成 -70，反接电流更小。 */
#define ARC_BRAKE_ENABLE        0        /* [ARC2 2026-11-21] 关回去：内轮现在是主动倒转，
                                            不再需要反接脉冲。开着只会让电流翻倍。 */
#define DIR_DIFF_MAX            40.0f    /* [SPEED300 2026-11-21] 160 -> 40。
                                            160 和驱动前馈（≈447）是同一量级（36%），
                                            直道上 turn 摆 ±20 就被放大成 ±160 的轮速差，
                                            速度环再往回抢，实测 CCR 出现 233/594 的拉锯。
                                            现在只保留"修左右轮差异"的作用。 */
#define ARC_MAX_TICKS           100U     /* 1 s 兜底 */

/* ---- 断线区斜穿（[BLIND 2026-11-18]） ----
   不是直角弯/六边形的场合丢线时，按固定角度斜穿过去找线：
   tan = 15/65 -> 13.0°，路程 = √(15²+65²) = 66.7 cm。 */
#define BLIND_ANGLE_DEG         13.0f
#define BLIND_DIST_CM           66.7f
#define BLIND_DIST_COUNTS       ((uint32_t)(BLIND_DIST_CM * COUNTS_PER_CM))
#define BLIND_START_TICKS       5U       /* 丢线这么多拍后开始斜穿 */
#define BLIND_SPEED             60.0f    /* 斜穿时的前进速度 */
#define BLIND_TURN              45.0f    /* 转向阶段的差速（注意：不反转护栏会把 turn 钳到 ≤ speed，
                                            所以这里必须 ≤ BLIND_TURN_SPEED） */
#define BLIND_TURN_SPEED        45.0f    /* 转向阶段的前进速度（内轮 0，不反转） */
#define BLIND_K_YAW             2.0f     /* 直行阶段的航向保持增益 */
#define BLIND_MAX_ABS_ERR       1.0f     /* [BLIND-GATE 2026-11-21] 丢线前的误差上限（路）。
                                            只有"车基本正对着线"时线消失，才算断线/缺口，
                                            才启动盲穿；误差大说明是弯，不盲目直穿。
                                            调大 -> 更容易进盲穿；调小 -> 更严格。 */
#define BLIND_DIR_FIXED        -1.0f     /* [BLIND 2026-11-18] 斜穿方向固定值：
                                            +1 = 左转（turn 为正）
                                            -1 = 右转（turn 为负）
                                            不再看丢线前的误差——那个值不可靠。
                                            现在固定右转。 */
#define BLIND_SEEN_TICKS        3U       /* [FIX 2026-11-18] 连续这么多拍看到线才算"找到"。
                                            原来 1 拍就退出，断线区边缘一闪就把角度归零，
                                            导致永远转不到目标角。 */
#define BLIND_TURN_MAX_TICKS    60U      /* 转向阶段超时（0.6 s），兜底强制进直行 */
#define BLIND_RUN_MAX_TICKS     600U     /* 直行阶段超时（6 s），兜底放弃 */
#define ARC_BRAKE_TICKS         5U       /* [BRAKE 2026-11-18] 旧固定窗口（ARC2 下不再使用） */
#define ARC_BRAKE_MAX_TICKS     20U      /* [ARC2 2026-11-21] 刹车窗口上限 20 拍（200 ms）。
                                            实际会提前结束：内轮实测降到 ARC_BRAKE_SPEED_MIN
                                            以下就不再反接。这样不依赖电池电压/入弯速度。 */
#define ARC_BRAKE_TARGET        -40.0f   /* 反脉冲的目标速度（只越过反向死区，不是全速反接） */
#define ARC_BRAKE_SPEED_MIN     40.0f    /* 实测速度高于此值才反接（低速反接没意义还伤电机） */
#define ARC_TRIG_ERR            3.5f     /* [ARC 2026-11-18] 圆弧触发还要求误差已接近极限。
                                            只靠"宽线"会把缓弯、十字路口全当成直角弯
                                            （方向环太强：mask=4095 全亮也触发过）。 */
#define KINEMA_TD               0.5f     /* [CENTER 2026-11-18] 0.3 -> 0.5：增益拿回 16 后
                                            需要更多阻尼，否则又会摆。 */
#define DIR_DAMP                0.25f    /* [DAMP 2026-11-16] 陀螺阻尼：turn -= DIR_DAMP × gyro_z。
                                             车转 100°/s 时收回 25，抑制中心附近的来回摆。
                                             过大会"转不动"，建议 0.15~0.30 */
#define TURN_SLEW               20.0f    /* [SPEED300 2026-11-21] 5 -> 20。
                                            5/拍时弯口只有 ~10 拍可用，turn 最多爬到 50，
                                            而 45° 弯在 300 下要 250（实测 err 3.5 只到 −25）。
                                            20/拍 -> 10 拍能到 200。直道需求只有 ±35，
                                            斜率只在需求突变时才介入。 */

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
#define CW_SIDE_RUN         4U       /* 已停用（WIDE 判据整体 #if 0） */
#define CW_DARK_TICKS       2U       /* 已停用 */
#define CW_LOW_TICKS        6U       /* 已停用 */
#define CW_WIDE_TIMEOUT     80U      /* 已停用 */
#define CORNER_LOST_TICKS       5U       /* 线完全离开阵列这么多拍，
                                            才认直角弯。5 拍 = 50 ms，在 300 下是 3.5 cm，
                                            足够滤掉单帧漏读，又不会晚到压过弯口。 */

/* [CW3 2026-11-21] 触发方式二选一。
   0 = 旧：线丢满 CORNER_LOST_TICKS 拍 + 丢线前最后一条线居中才触发。
       实测触发点漂 3~4 拍（2~3 cm），出弯横向差就在这个量级。
   1 = 新：连续 CORNER_L_FULL_TICKS 拍拍到"一侧 6 路全亮"立刻触发。
       触发点由几何决定，每个弯一致。回滚就把它改成 0。 */
#define CORNER_TRIG_MODE        1
#define CORNER_L_FULL_TICKS     2U       /* [CW3] 连续几拍全亮才算数。
                                            [FIX 2026-11-21] 1 -> 2：1 拍时"十字路口的边缘"也会触发
                                            —— 车压上十字时先出现"半边亮"（ch0~5 或 ch6~11），
                                            下一拍才是 12 路全亮；只要 1 拍就够它误判成直角弯。
                                            2 拍时第二拍已经是全亮（两侧都满，不算 L），
                                            武装自动作废。 */
#define CORNER_SIDE_FULL        6U       /* [FIX 2026-11-21] 5 -> 6（退回）：5 路是"45° 斜穿"
                                            和"刁钻路口直角弯"的重叠区，用路数切不开 ——
                                            5 会让 45° 也被判成直角弯。
                                            退回 6：45° 安全；刁钻路口不触发，靠 HOLD/盲穿兜底。
                                            要救那个路口，得找"路数之外"的第二个判据。 */
#define CORNER_FULL_HOLD_TICKS  25U      /* [CW2 2026-11-21] 手推实测定的，不是猜的：
                                            直角弯"最后一拍全亮" -> "丢线第一拍"实测 18 拍
                                            （mask 127 持续 16 拍 -> 124 -> 60 -> 48 -> 32 -> 0）。
                                            取 18 + 余量 = 25 拍（250 ms，300 下约 17 cm）。 */
#define CORNER_SPEED            25.0f    /* [PIVOT 2026-11-14] 恢复 25：仍供六边形用；过弯 pivot 分支自己写 0 */
#define CORNER_EXIT_SPEED       150.0f   /* [ARC2 2026-11-21] 0 -> 150（= ARC_OUTER_SPEED/2）。
                                            出弯斜坡从这个速度起，150 -> 300 用 150 ms。
                                            原来是 0：那是 pivot 停车方案的起点，
                                            但 ARC2 出弯时车还有 150，从 0 起会先急刹一下。 */
#define CORNER_RAMP_TICKS       15U      /* 150 ms */
#define CORNER_YAW_EXIT         70.0f
#define CORNER_MIN_TICKS        12U      /* 120 ms（pivot 方案下已不使用） */
#define CORNER_MAX_TICKS        200U     /* [GND 2026-11-14] 80->200：实测 ω≈57°/s，转 82° 要 144 拍；
                                            80 拍只够 46°——每次被超时掐断->再触发->反向再转 */
#define CORNER_COOLDOWN         40U      /* 旧的时间制冷却（已不用，保留兼容） */
#define CORNER_COOLDOWN_CM      6.0f     /* [COOLDOWN 2026-11-18] 出弯冷却改为距离制：
                                            走过 6 cm 才允许下一次锁存。
                                            时间制 0.4 s 在 BASE 120 下 = 11 cm，
                                            比锯齿段的角间距（15~20 cm）还长，会漏弯。 */
#define CORNER_COOLDOWN_COUNTS  ((uint32_t)(CORNER_COOLDOWN_CM * COUNTS_PER_CM))
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
#define CORNER_RECOVER_DEG      90.0f    /* [ARC-EXIT 2026-11-21] 45 -> 90：直角弯后"没转够"时
                                            同向补转的上限（ARC2 的 ARC_RECOVER 用这个）。
                                            90° 一个整弯：45° 只够补"差一点"，连续两个直角弯
                                            （或 45° 接直角）时经常补满 45° 还是看不到线。
                                            补角只挂在直角弯状态机后面，不在普通丢线上做。 */
#define CORNER_RECOVER_TICKS    140U     /* [ARC-EXIT 2026-11-21] 80 -> 140：拍数必须跟着角度走。
                                            实测 pivot 档 ω≈80°/s（PIVOT_TARGET 45），90° 约 113 拍；
                                            原来 80 拍（0.8 s ≈ 64°）会先于角度判据触发，
                                            只改 DEG 等于没改。140 拍 = 1.4 s，留 25% 余量兜底。 */
#define CORNER_SETTLE_MAG       30.0f    /* 收尾用的差速（比 PIVOT_TARGET 小，动作更细） */
#define CORNER_SETTLE_COAST_RATE 35.0f   /* [JUDDER 2026-11-15] |角速度| 还高于此值就先断电滑行：
                                            在高速旋转时通电修正 = 反接制动，电流数倍 -> 抖动 */
#define CORNER_COAST_T          0.06f    /* 预测断电后还会滑转的时间，s（用于提前断电） */

/* [SPEED300 2026-11-21] 10 -> 2。
   这条的前馈原本是为了"turn 很小时前馈差压不过死区"，但现在驱动前馈带了 174 的截距，
   两个轮子本来就都在死区之上，这个理由已经不成立。留着 10 反而在直道摆动时
   把 turn 放大 10 倍成占空比差（10×20=200），变成正反馈。 */
#define DIR_DIFF_GAIN           2.0f     /* 每单位 turn 额外的占空比差 */

/* ---- 六边形入口（[HEX 2026-11-15] 整理） ----
   HEX_MODE 三档：
     0 = 关闭：只更新调试快照，不产生事件、不接管（当前默认）
     1 = 只记录：照常产生事件并打印 HEX 行，但**不接管舵量**——用来在真实赛道上
         验证"六边形顶点"和"直角弯"的波形差异，确认不误触发后再切 2
     2 = 接管：事件确认后按 HEX_V_MAX_TICKS 拍打舵覆盖
   注意：判据里的 ch11/ch0 + center 在直角弯上同样成立，所以 1 档是必要的验证步骤。 */
#define HEX_MODE                2        /* [HEX4 2026-11-21] 0 -> 1：先只记录。
                                            状态机会跑、会打 HEX 事件、会存 V 记录（按 'h'），
                                            但**不接管舵量**（接管只在 mode==2）。
                                            量完 V 的真实形态、确认事件序列对了再改 2。 */
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
#define HEX_V_MIN_TICKS         35U      /* [FIX 2026-11-18] 8 -> 35（0.35 s）。
                                            V 判据要求 c 亮，所以触发那一拍线就在中段；
                                            8 拍时 touch 立刻成立，只转了 7° 就交回
                                            （六边形问题：HEX,5 和 HEX,6 只隔一行）。
                                            按 74°/s 算，转完 26° 需要 35 拍。 */
#define HEX_V_GAP_MIN_CM        10.0f    /* [纯距离触发] 武装后走这么远就开始原地转。
                                            实测：手推 10.3 cm、自跑 10.2 cm，间距非常稳定。
                                            不用 c/b 判据——实测 9 cm 之后 c 必灭，判据自相矛盾 */
#define HEX_V_GAP_MAX_CM        16.0f    /* 兜底：超过这个距离还没触发就解除武装 */
#define HEX_V_GAP_MIN_COUNTS    ((uint32_t)(HEX_V_GAP_MIN_CM * COUNTS_PER_CM))
#define HEX_V_GAP_MAX_COUNTS    ((uint32_t)(HEX_V_GAP_MAX_CM * COUNTS_PER_CM))
#define HEX_V_B_MAX             1U       /* [FIX 2026-11-16] b 段最多亮几路才算"缺口"。                                                                                        2 -> 1：b 只有 3 路，亮 2 路意味着只暗 1 路——
                                            单路漏读就能伪造一个 V（实测 直角-六边耦合1：
                                            mask=117=ch0,2,4,5,6，ch1/ch3 漏读，被当成缺口）。
                                            改成 1 后要至少暗 2 路才算缺口。 */
#define HEX_V_DEBOUNCE          2U       /* [HEX4 2026-11-21] V 要连续这么多拍同一侧才算数。
                                            原版没有去抖，单拍快照会把"端亮 + 正中亮 + 单路漏读"
                                            直接当成 V。 */

/* ---- [HEX4 2026-11-21] 六边形的"两次 V"状态机（按现场描述重写） ----
   入口：第一次 V 武装（记录大误差方向 + 把方向环打偏的那部分回正），
         10~20 cm 内出现**同侧**第二次 V 且大误差方向一致 -> 进入，向记录方向打 30°；
   途中：交回方向环走六边形，陀螺累计转角；
   出口：累计 350~370° 且遇到下一个 V -> 记为出口，向该 V 误差方向的**反方向**打 30°，
         然后交回方向环找线。 */
#define HEX4_V2_MIN_CM          6.0f     /* [HEX4 2026-11-21] 10 -> 6：实测入口两次 V 相距 8.5 cm
                                            （第一次跑被 10 cm 下限挡掉，只打出 HEX,4 就解除了）。
                                            6 留了 2.5 cm 余量；上限 20 不动。 */
#define HEX4_NEED_V2            0        /* [HEX4 2026-11-21] 进六边形要不要"必须等到第二次 V"：
                                              1 = 等（原始设计，对轨迹敏感）
                                              0 = 不等：V1 之后走够 HEX4_ARM_DIST_CM 就进
                                           改成 0 的原因：巡线开着的时候，入口的第二个 V
                                           经常根本不形成（实测 20 cm 内一次都没有）；
                                           而 V1 的判据很硬，两次 V 间距手推实测 8.3 cm，
                                           所以"走 9 cm"和"等第二个 V"在几何上等价。 */
#define HEX4_ARM_DIST_CM        9.0f     /* [HEX4] NEED_V2=0 时，武装后走这么远就进 */
#define HEX4_ARM_DIST_COUNTS    ((uint32_t)(HEX4_ARM_DIST_CM * COUNTS_PER_CM))

/* ---- [HOLD 2026-11-21] 丢线后保持转向 ----
   45°/60° 弯的线会在 5~6 拍内滑出阵列；那一拍 have_control 变 0，
   原来的代码把两个轮子的目标瞬间清零，车直冲出去（看起来"方向环没努力"）。
   这里在丢线后继续用最后一拍的舵打 N 拍，让车往线消失的方向拐。 */
#define LINE_HOLD_TICKS         16U      /* 保持 16 拍（160 ms）。
                                            按 60° 弯算：turn 顶到 240 时 ω≈394°/s，
                                            转 60° 要 ~15 拍，所以 16 才够；8 拍时只能转 30°。 */
#define LINE_HOLD_SPEED_K       0.8f     /* 保持期间速度 = 0.8 × BASE。
                                            太慢会影响可用舵量（turn ≤ speed）。 */
#define LINE_HOLD_SLEW          20.0f    /* 保持期间**继续加舵**的斜率（每拍）。
                                            丢线那一刻 turn 往往只有 100~150（还在爬坡），
                                            冻结住就永远不够；这里让它继续往 240 顶。 */

/* ---- [RECOVER2 2026-11-21] 普通丢线的"同向补转" ----
   HOLD 用完还没看到线、并且**丢线前一拍 error 够大**（说明是弯，不是断线缺口）时，
   朝线消失的那一侧原地补转：上限 90°，一看到线立刻交回方向环。
   [REVIVE 2026-11-21] 原来 45 -> 0.5 把整套禁掉了（理由是断线区会误转）；
   现在用 error 门限把"断线缺口"挡掉，只在大误差丢线时启用，角度放到 90°。
   90° 是一个整弯：60° 时 60° 以上的缺角补不到；代价是最坏情况原地转 90° 才放弃。 */
#define RECOVER2_ERR_MIN        2.5f     /* 丢线前一拍 |error| 大于这个值才算"弯里丢线"。
                                            45° 弯丢线时实测 error 4~5.5；断线缺口一般 <2。 */
#define RECOVER2_DEG            90.0f    /* 补转上限：总转角（含 HOLD 段）到 90° 还没线就放弃 */
#define RECOVER2_TURN           45.0f    /* 补转用的差速（原地转） */
#define RECOVER2_TICKS          160U     /* 超时兜底（1.6 s）。90°@80°/s≈113 拍，
                                            160 拍留 40% 余量，保证角度判据先触发、
                                            低速时也能转过 90° 再放弃。 */
#define HEX4_V2_MAX_CM          20.0f    /* 上限（超过就解除武装） */
#define HEX4_V2_MIN_COUNTS      ((uint32_t)(HEX4_V2_MIN_CM * COUNTS_PER_CM))
#define HEX4_V2_MAX_COUNTS      ((uint32_t)(HEX4_V2_MAX_CM * COUNTS_PER_CM))
#define HEX4_BIG_ERR            2.0f     /* |error| > 这个值才算"大误差方向" */
#define HEX4_STRAIGHT_TICKS     6U       /* 第一次 V 之后回正的拍数 */
#define HEX4_STRAIGHT_K         0.6f     /* 回正舵量 = -K × V1 那一拍方向环的舵 */
#define HEX4_TURN_DEG           30.0f    /* 进/出各打 30° */
#define HEX4_TURN_TOL           4.0f     /* 角度容差 */
#define HEX4_TURN_RATE          45.0f    /* 打舵用的差速（原地转） */
#define HEX4_TURN_MAX_CM        12.0f    /* 打舵兜底：走这么多 cm 没转够就放弃 */
#define HEX4_TURN_MAX_COUNTS    ((uint32_t)(HEX4_TURN_MAX_CM * COUNTS_PER_CM))
#define HEX4_EXIT_MIN_DEG       320.0f   /* [HEX4 2026-11-21] 350 -> 320：进弯 pivot 已经转掉 30°，
                                            六边形一圈 360° 里 DRIVING 只剩 330°。
                                            窗口按 330±10 开。实测后按 HEX,9 的第二个字段再定死。 */
#define HEX4_EXIT_MAX_DEG       370.0f   /* 上限留到 370：HEX_MODE=1（不接管）时没有那 30° pivot，
                                            累计会是 ~360°，窗口必须能覆盖两种模式。
                                            实测一遍后按 HEX,9 的第二个字段再收窄。 */
#define HEX4_EXIT_GIVEUP_DEG    400.0f   /* 超过这么多还没等到 V -> 强制解除 */
#define HEX4_DRIVE_MAX_CM       150.0f   /* [HEX4 2026-11-21] DRIVING 距离兜底：从入口 pivot 起
                                            走这么多 cm 还没出栈就强制解除（一圈约 120 cm，
                                            留 30 cm 余量）。没有这条时，出口 V 没认出来
                                            （HEX,9 丢失）会让状态 3 永久锁死 —— 出弯后走直道
                                            转角不再增长，永远到不了 400°，直角弯检测一直被
                                            hex_zone_flag 屏蔽，直到停车才清。 */
#define HEX4_DRIVE_MAX_COUNTS   ((uint32_t)(HEX4_DRIVE_MAX_CM * COUNTS_PER_CM))
#define HEX_V_STRICT_END        1        /* [HEX4] 1 = 端部要求"两路都亮"（ch0&ch1 / ch10&ch11）。
                                            原版只要有任意一路就成立，单路假亮（ch10 悬空、
                                            ch1/ch3 漏读）就能伪造 V —— 这是历史误触发的主因。
                                            改 0 回到原判据。 */

/* ---- 六边形区间（[HEX-ZONE 2026-11-15]） ----
   [实测] 六边形**内部 6 个顶点不产生 V**（60° 折角线是连着的，不出现"端点亮+正中亮+中间缺口"）；
   V 只在**入口**和**出口**出现。所以区间不能靠"见到 V 续命"，必须一次覆盖整圈。
   六边形一圈约 120 cm，取 150 cm（含余量）。
   区间内**禁用直角弯检测**（CW + 通用锁存），否则六边形内部的宽线会被当成直角弯。 */
#define HEX_ZONE_CM             150.0f
#define HEX_HALF_DEG            180.0f   /* [DBLHEX 2026-11-18] 累计到 180°：不动作、归零重新累计 */
#define HEX_FULL_DEG            180.0f   /* 第二段再累计 180°（总 360°）后，V 才允许出栈 */
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
#define TARGET_MAX              1500.0f  /* [SPEED300 2026-11-21] 400 -> 1500。
                                            BASE 300 + turn 250 = 550，400 会把目标钳死；
                                            1500 只做软件错误的护栏，正常用不到。 */

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
#define FF_DRIVE_L              0.88f    /* [SPEED300 2026-11-21] 2.27 -> 0.88，并补上截距。
                                            两点拟合（150→306、1000→1054）：
                                              slope = (1054-306)/850 = 0.88
                                              截距  = 306 - 0.88×150 = 174
                                            所以 CCR = 174 + 0.88×target，全速度段都准。
                                            纯比例 2.27 在 target=300 时给 681，实际只要 438，
                                            就是这里 +55% 超驱。截距在 control.c 的 FF_Apply() 里加。 */
#define FF_DRIVE_R              0.94f    /* 右轮略弱，按原比例折算 */
#define FF_DRIVE_L_REV          0.99f
#define FF_DRIVE_R_REV          1.05f
#define FF_DRIVE_OFFSET         174.0f   /* 两点拟合的截距；只在驱动态加，pivot 不加 */

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
#define LOST_STOP_TICKS         120U     /* [RECOVER2 2026-11-21] 30 -> 120：
                                            "HOLD 16 拍 + 原地补转 45° 约 80 拍"，
                                            30 拍时车已经被 Car_Stop 掐断，
                                            RECOVER2 只跑了 13 拍就被停车（看起来"没补"）。 */
#define ENC_DELTA_LIMIT         2000     /* [SPEED300 2026-11-21] 500 -> 2000。
                                             BASE 300 时正常一拍增量就有 300，弯道峰值 450，
                                             500 只差一点就会误杀真数据（坏采样保护沿用旧值）。 */

/* ---- 低速增益调度 ---- */
#define KP_LOW_SPEED            10.0f
#define KP_BOOST                2.5f

/* ---- 陀螺 ---- */
#define GYRO_ALPHA              0.3f
#define GYRO_SAMPLE_DT          0.005f   /* [YAW 2026-11-15] 采样周期 = 5 ms（TIM2 的 5 拍） */

#endif /* CAR_CONFIG_H */
