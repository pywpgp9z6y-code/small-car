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
#define BASE_SPEED              400.0f    /* [SPEED300 2026-11-21] 150 -> 300（0.69 m/s）。
                                            注意：这一步只做"速度链"，TURN_MAX 还是 120，
                                            300 下的最小转弯半径约 20 cm，45° 弯过不去。
                                            验证请只跑直道（看 CCR 是否落在 438 附近），
                                            弯道等第二步放开转向权限。 */
#define TURN_MAX                300.0f    /* [SPEED300 2026-11-21] 120 -> 250。
                                            几何账：前视 d 处侧偏 3.3 cm 能表达的最小半径
                                            R = d²/(2×3.3)。前视 8 cm 时 R_min = 9.7 cm，
                                            对应 ω = 0.69/0.097 = 7.1 rad/s = 408°/s
                                            -> turn = 408/1.644 = 248，所以 250 才够跑 45° 弯。
                                            内轮 = 300 − 250 = 50，仍然不反转。 */
#define TURN_GAIN_BOOST         2.3f      /* [TURNBIAS 2026-11-18] 六边形区间内的增益倍率。
                                            平时 e=5.5 的 turn 上限是 52（不打扰直道）；
                                            区间内 ×2.3 -> 约 120，正好用满内轮=0 的能力。 */

#define CURVE_SLOW              0.15f     /* [CORNER45 2026-11-21] 0 -> 0.3：speed = BASE − 0.3×|turn|。
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
#define ERR_FILTER_ALPHA        0.40f    /* [CENTER 2026-11-18] 0.25 -> 0.30：折中，
                                            0.25 的延迟让回中更慢 */
#define OMEGA_KP                0.5f     /* 角速度环增益：turn / (°/s) */
#define OMEGA_PER_TURN          1.644f   /* 1 单位 turn 对应的角速度，°/s（纯几何） */
#define CURVE_K                 13.3f    /* turn = CURVE_K · v · e / d²（v 用 counts/10ms） */
#define DAMP_K                  0.4f     /* [ARC 2026-11-18] 巡线纯阻尼：turn -= DAMP_K·ω。
                                            不再用 K·(ω_des−ω)，那会把位置增益放大 82%。 */

/* ---- [HEX-FAST 2026-11-22] 六边形区间的"快响应"参数 ----
   60° 顶点要求在 ~0.1 s 内把舵打起来。直道用的 TURN_SLEW=20 要 12.5 拍才到 250，
   等打满车已经冲过顶点、线滑出阵列——出口 V 经常就是这么丢的。
   下面这套只在 Hex_ZoneActive()（第一次 V 到出弯转完）期间生效，不影响直道。
   回滚：把四个值改回和上面一致即可。 */
#define TURN_SLEW_HEX           60.0f    /* 每拍最大变化（直道 20） */
#define DAMP_K_HEX              0.20f    /* 陀螺阻尼（直道 0.4） */
#define PURSUIT_L_CM_HEX_NEAR   5.5f     /* 大误差时六边形区间的前视（直道 8.0）。
                                            [2026-11-28] 6.0 -> 5.5：
                                            日志显示 60° 顶点 turn 峰值只有 300（上限 320，
                                            没撞上限），说明瓶颈是**需求**不是上限。
                                            G = 13.3×v/d²：d=6.0 时 88.7/路，
                                            d=5.5 时 105/路 —— 同样 err_d=3.4 路，
                                            需求从 300 提到 357。
                                            几何上 d=5.5 能表达的最小半径
                                            R_min = d²/(2×3.3) = 4.6 cm，
                                            仍然足够表达 60° 需要的 5.7 cm。 */
#define TURN_MAX_HEX            280.0f   /* 转向上限（直道 250） */
#define HEX_ZONE_SPEED_MAX      240.0f   /* 六边形区间限速；设 300 等于关闭。
                                            [2026-11-28] 定为 240（= 300 档行为）。
                                            注意：320 时 60° 顶点差 11%，240 时只差 5%，
                                            所以降速本身是有利的。 */   /* 六边形区间限速；设 300 等于关闭。
                                            [2026-11-27] 400 档也曾提到 320（保持 80% 比例），
                                            但实测顶点更容易出界，所以**六边形整体按 300 档跑**，
                                            只在直道/直角弯/折角用 400。
                                            240/400 = 60%，六边形里是"主动降速通过"。 */

/* ---- [HEXBOOST 2026-11-22] 六边形顶点短暂允许内轮反转 ----
   60° 顶点需要 turn≈380；内轮 0 时 turn 上限只有 speed。
   这里在 |error| 大时开一个短窗口，turn 上限提到 speed + HEX_BOOST_INNER
   （内轮 -80、外轮 +80，平均速度不变），窗口结束强制冷却，避免连续反接。
   直角弯的 pivot 本来就允许负目标（护栏排除 corner 状态），这里只是把
   "短暂反接"也用到六边形的 DRIVING 段。 */
#define HEX_BOOST_ERR           2.5f     /* 丢线前/当前 |error| 超过这个才开窗口 */
#define HEX_BOOST_TICKS         12U      /* 每次窗口 12 拍（120 ms） */
#define HEX_BOOST_GAP_TICKS     8U       /* 窗口结束后至少隔 8 拍才允许下一次 */
#define HEX_BOOST_INNER         120.0f   /* 内轮最多 -120（turn = speed + 120）。
                                            [2026-11-28] 80 -> 120。之前试 120 无效是因为
                                            那时需求只有 ~300、上限 400 都没被碰到；
                                            现在配合把前视缩到 5.5（需求升到 357），
                                            上限 240+120=360 才第一次成为有效约束。
                                            内轮 -120 只出现在 12 拍的 boost 窗口里。 */

/* ---- [HEX-PUNCH 2026-11-27] 六边形顶点起转脉冲 ----
   **为什么加大 HEX_BOOST_INNER 没有效果（已从代码确认）**：
     boost 只把 tmax（**上限**）抬到 speed + HEX_BOOST_INNER，
     但 pid_turn 是方向环"要"出来的：
         pid_turn = G × err_d,   G = 13.3 × v / d²
     六边形区间 v=320、d=PURSUIT_L_CM_HEX_NEAR=6.0 -> G = 118 每路。
     要把 turn 做到 460，需要 err_d = 3.9 路 —— 而线滑出阵列之前，
     滤波后的 err_f 根本到不了这个值。
     **所以上限从来没被碰到过，抬高上限当然无感。**

   办法（与 ARC_PUNCH / BEND_PUNCH 同一套）：
     不等比例环把舵"长"出来，直接命令一个短时脉冲。
       speed 270 / turn 385 -> 外 655 / 内 -115
       电机实测上限 ~662，所以 655 刚好可达、基本不饱和
       R = 8×270/385 = 5.6 cm   （60° 顶点需要 ≤ 5.7）
       ω = 385×1.644 = 633°/s

   与 BEND_PUNCH 的区别：六边形是一整片区间，脉冲要在每个顶点各触发一次，
   所以用"|error| 越过门限的上升沿"触发，每个顶点只打一次、只持续 8 拍。

   回滚：HEX_PUNCH_ENABLE 改 0，行为与改动前完全一致。 */
#ifndef HEX_PUNCH_ENABLE
#define HEX_PUNCH_ENABLE        0        /* [2026-11-27] 1 -> 0：实测"剧烈拉扯"，
                                            原因是脉冲方向每拍按 turn 符号重算，
                                            顶点过零时翻转。已改为触发瞬间锁存方向
                                            （hex_punch_dir），确认稳定后再重开。 */
#endif
#define HEX_PUNCH_TICKS         8U       /* 每个顶点脉冲 8 拍 = 80 ms */
#define HEX_PUNCH_TURN        385.0f     /* 外 655 / 内 -115 */
#define HEX_PUNCH_SPEED       270.0f     /* 脉冲期间中心速度上限 */
#define HEX_PUNCH_ERR           2.5f     /* 触发门限：|error| 超过这个值 */

/* ---- [BEND 2026-11-22] 大误差（45° 弯）时的方向环参数 ----
   45° 弯时线扫过阵列只有 ~9.5 拍，需要 turn≈288；直道参数 250/20 拍 打不满。
   这里在 near_d（|err|>2.0，回差 1.2 退出）时切一套更激进的参数 +
   限速，弯里用，直道（|err|<1.2）完全不受影响。 */
#define BEND_TURN_MAX           350.0f   /* turn 上限；300 时外轮 600、内轮 0（平均不变） */
#define BEND_TURN_SLEW          60.0f    /* 每拍最大变化；45 -> 5~6 拍到 250 */
#define BEND_DAMP_K             0.30f    /* 陀螺阻尼；比直道 0.4 小，弯里不轻易收舵 */
#define BEND_SPEED_MAX          340.0f   /* 弯里限速；260 下 45° 需要 ~411°/s，300 舵够 */

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
#define ARC_OUTER_SPEED         400.0f   /* 过弯时外轮保持的速度（= BASE） */
#define ARC_INNER_SPEED        (-100.0f)  /* [ARC2 2026-11-21] 内轮目标：负 = 主动倒转。
                                            为什么不用 0：内轮是被车拖着走的，反向脉冲
                                            再大也只是和地面抢轮子（实测 -40 都没明显改善）；
                                            让电机主动倒转，直接抵消拖拽，不受电压/地面影响。
                                            -70 是以前 PIVOT-C 验证过的值。
                                            调法：转过头/切内道 -> -50；转不够 -> -90。 */

/* 由上面两个目标反算的弧线中心速度 / 差速。不要再手抄 115 / 185：
   外轮 = 中心 + 差速，内轮 = 中心 − 差速。 */
#define ARC_CENTER_SPEED        ((ARC_OUTER_SPEED + ARC_INNER_SPEED) * 0.5f)  /* 115 */
#define ARC_TURN_SPEED          ((ARC_OUTER_SPEED - ARC_INNER_SPEED) * 0.5f)  /* 185 */

#define ARC_LEAD_DEG            9.0f     /* 提前这么多度就断电，靠惯性走完。
                                            246°/s 断电后还会滑 5~8°，不提前就会转过头。 */

/* ---- [ARC-TAIL 2026-11-27] 弧线末段速率收敛（移植 car-main3） ----
   现状：ARC 段是"恒定差速 + 到 (90 − LEAD)° 断电靠惯性滑"，
        本质是撞到 90° 再刹，落点方差大。
   打开后：接近目标角度时把允许角速度压下来
        remain = ARC_YAW_DEG − dir×yaw
        ω_des  = min(turn×1.644, ARC_TAIL_K × remain)
   只往下收、不往上加，所以刚进弯仍是满舵，末段自己"滑进去停住"。
   K=40 时：剩余 4.6° 开始收敛（185×1.644/40 = 7.6°，实际从 7.6° 起）。

   **注意**：收敛后断电点必须同时提前量降下来，否则会停在
   (90−9)=81° 就断电、剩下的角度全靠惯性 —— 反而更差。
   ARC_EXIT_LEAD_DEG 自动切换，不需要手动改 ARC_LEAD_DEG。

   回滚：ARC_TAIL_ENABLE 改 0，断电判据回到 ARC_LEAD_DEG(9)，
         ARC 段与改动前逐字节一致。 */
#ifndef ARC_TAIL_ENABLE
#define ARC_TAIL_ENABLE         1
#endif
#define ARC_TAIL_K             80.0f     /* °/s 每 °：越大收敛越早越柔。
                                            40 时从剩余 10.3° 开始收（要 6 拍，比满速多 3 拍）；
                                            80 时从 5.1° 开始收（只要 2 拍）——
                                            顶满脉冲后断电 ω 更高，必须收得更早。 */
#define ARC_TAIL_MIN_DEG        0.5f     /* remain 下限，避免 ω_des 直接归零 */
#define ARC_TAIL_LEAD_DEG       2.0f     /* 收敛模式下的断电提前量 */

/* 必须写成"使用点展开"的表达式：下面的 #if 会在定义处求值，
   而 ARC_TAIL_ENABLE 可能被外部 -D 覆盖或后续改动，用三目最安全。 */
#define ARC_EXIT_LEAD_DEG       ((ARC_TAIL_ENABLE != 0) ? ARC_TAIL_LEAD_DEG : ARC_LEAD_DEG)

/* ---- [ARC-PUNCH 2026-11-27] 起转顶满脉冲 ----
   思路：直角弯只需要"转过 90°"，不需要"匀速转 90°"。
        前 N 拍把差速顶到电机能给的最大值，把 ω 从 411°/s 拉起来，
        之后回落到正常弧线，让 TAIL 收尾。

   时间账（400 档，L=18cm）：
     现在  恒定差速 250 -> ω=411°/s -> 转 90° 用 22 拍
     顶满  差速 500     -> ω 最高 ~820°/s（受电机上限，实际 600~700）
                         -> 转 90° 约 14~16 拍

   物理约束（**这一段决定了值能开多大**）：
     原地旋转的向心加速度 a = ω² × R_自身，R_自身 ≈ 轮距/2 = 9 cm
       ω=411°/s (7.2 rad/s) -> 4.6 m/s² = 0.47 g   ← 现在，无压力
       ω=600°/s (10.5 rad/s) -> 9.9 m/s² = 1.01 g
       ω=800°/s (14.0 rad/s) -> 17.6 m/s² = 1.80 g ← 这是 45° 弯都做不到的水平
     所以 ω 越高，需要的抓地力按 ω² 暴涨。顶满只是"短暂"冲一下，
     靠轮胎峰值摩擦顶住，不能长期维持 —— 所以必须限拍数。

   风险：
     1) 外轮目标 700 很可能超过电机空载转速 -> PID 打满占空比饱和。
        这是**可接受的**：饱和只会让它跑自己最快，不会烧。
        但记账的 turn 会比实际大，即"代码以为在转 500，实际只转了 350"。
     2) 内轮 -300 反接，电流大。**跑 3 个弯摸驱动温度**，烫就减 ARC_PUNCH_TURN。
     3) 断电时 ω 更高，惯性滑转会从 2° 变成 5~8°。
        所以必须同时把 ARC_TAIL_K 提到 80（40 太保守，来不及收）。

   回滚：ARC_PUNCH_ENABLE 改 0，ARC 段恢复"恒定 115 / 250"的原有行为。 */
#ifndef ARC_PUNCH_ENABLE
#define ARC_PUNCH_ENABLE        1
#endif
#define ARC_PUNCH_TICKS         8U       /* 前 8 拍（80 ms）顶满 */
#define ARC_PUNCH_CENTER      200.0f     /* 顶满期间中心速度 */
#define ARC_PUNCH_TURN        500.0f     /* 顶满期间差速 -> 外 700 / 内 -300 */

/* ---- [ARC-DAMP 2026-11-27] 末段陀螺阻尼（朋友实测方法） ----
   现象：直角弯抖动。

   根因（已确认）：300v0.2 的 ARC 段**没有任何速率反馈** ——
     `ARC_RATE_LOOP_ENABLE` 根本没定义，`#if ... == 1` 整块被编译掉。
     ARC 是"开环恒定差速 + TAIL 压指令"：
       - TAIL 只把**允许的角速度**往下压（改指令）
       - 但车实际还在以 550~650°/s 惯性旋转（顶满脉冲之后）
     指令压下去了、实际转速没压下去 -> 末段反复过冲 -> 抖动。

   办法：离目标角 ≤ ARC_DAMP_DEG(60°) 时，按**实测角速度**做阻尼：
       turn_r -= ARC_DAMP_K × dir × gyro_z
   这就是巡线里 DAMP_K 的同一个东西，只是只作用于末段 60°。
   提前开（比如 90°）会在起转阶段就把舵收掉，直角弯会变慢；
   只在后 60° 开，既不损失起转速度，又能刹住。

   符号：dir×gyro_z = 车在该弯方向上的实际角速度，减去它就等于反打。
   限幅到 0：不允许把差速减成反向（否则内轮会正转、出现反打）。

   回滚：ARC_DAMP_ENABLE 改 0。 */
#ifndef ARC_DAMP_ENABLE
#define ARC_DAMP_ENABLE         0
#endif
#define ARC_DAMP_DEG           60.0f     /* 剩余角度 ≤ 这个值才开阻尼 */
#define ARC_DAMP_K              0.20f    /* 阻尼增益（直道 DAMP_K 是 0.40） */

/* ---- [BEND-PUNCH 2026-11-27] 折角起转脉冲 ----
   ARC 顶满脉冲验证成功后（12 个直角弯，驱动冰凉），把同一思路搬到 45° 折角。

   折角的死结：
     45° 需要 R ≤ 8.0 cm，而 turn ≤ speed_cmd 保证 R ≥ 8 cm —— 正好卡在下界。
     更要命的是"CURVE_SLOW 定点"让 turn 只能到 308，R = 10.8 cm，差 26%。
     而折角从"线开始偏"到"线出阵列"只有 3 拍，根本没有时间慢慢加速。

   办法：折角一触发，直接给一个短时超额脉冲，越过 turn ≤ speed 的限制：
     speed = 260, turn = 400  ->  外轮 660 / 内轮 -140
     ω = 400×1.644 = 658°/s
     车还带着 0.92 m/s 时：R = 0.92 / (658×π/180) = 8.0 cm  ← 正好够
     抓地：v²/R = 10.6 m/s² = 1.08 g                          ← 400 档给得起

   为什么不会像 REV_ALLOW 那样游龙（三条硬门限）：
     1) **只在 near_d 的上升沿触发**，一个弯只打一次，不持续；
     2) 要求 |Δerr| ≥ BEND_PUNCH_DE(0.8 路/拍) —— 真折角是 2 路/拍，
        直道噪声只有 0.2~0.5，过不了这一关；
     3) 只持续 BEND_PUNCH_TICKS(4) 拍，之后立刻回到正常巡线。
   REV_ALLOW 那次是"near_d 一开就一直允许超额"，没有时间限制，
   所以直道噪声也能触发 -> 速度环正反馈 -> 大游龙。

   回滚：BEND_PUNCH_ENABLE 改 0，行为与改动前完全一致。 */
#ifndef BEND_PUNCH_ENABLE
#define BEND_PUNCH_ENABLE       0        /* [2026-11-28] 1 -> 0：GOOD 快照里没有这个功能。
                                            它会在直角弯判定前先猛转 4 拍，破坏"一侧 6 路全亮"
                                            的签名，导致 CORN 不触发、车冲出去。
                                            已确认这是"隔天直角弯冲出线"的首要嫌疑。
                                            45° 折角要用时再单独开。 */
#endif
#define BEND_PUNCH_TICKS        4U       /* 脉冲拍数：4 拍 = 40 ms */
#define BEND_PUNCH_TURN       400.0f     /* 脉冲期间差速（外 660 / 内 -140） */
#define BEND_PUNCH_SPEED      260.0f     /* 脉冲期间中心速度上限 */
#define BEND_PUNCH_DE           0.8f     /* 触发门限：|Δerr| 路/拍 */

/* ---- [ARC-ENTRY-BRAKE 2026-11-27] ARC 入口两轮同时制动（移植 car-main3） ----
   机制（car-main3 2026-11-26）：入口把中心速度压到 ARC_ENTRY_SPEED(0)，
   两轮目标从"外 300 / 内 −70"变成"外 +185 / 内 −185"：
     - 外轮目标从 300 降到 185：入口那几拍不再往前推（受 Motor_Output 的
       "输出不允许与目标反向"护栏限制，外轮实际是滑行，不是反向通电）；
     - 内轮反接从 −70 加深到 −185：制动力更大。
   这里用带符号中心速度（左右实测速度的平均，不是幅值平均）判交接：降到弧线
   中心速度 ARC_CENTER_SPEED(115) 时交回正常弧线。若用幅值平均，正常弧线里
   恒为 185（外 300/内 −70），115 永远到不了。done 锁存避免在阈值附近来回切。

   参数来源：car-main3 的 ARC_ENTRY_BRAKE_ENABLE=1 / ARC_ENTRY_SPEED=0。
   **注意**：car-main3 的 ARC_ENTRY_EXIT=260 是它 520 档的弧线中心速度
   （外 590 / 内 −70）；300v0.2 的弧线中心是 115，所以按同一条
   "EXIT = 弧线中心速度"规则换算成 115，不能照抄 260。

   风险：入口几拍外轮滑行 + 内轮反接，电流/温升高于纯单轮；先跑 5 个弯摸温度。
   回滚：ARC_ENTRY_BRAKE_ENABLE 改 0，ARC 段即恢复 115 / 185 的原行为。 */
#ifndef ARC_ENTRY_BRAKE_ENABLE
#define ARC_ENTRY_BRAKE_ENABLE   0      /* [NO-BRAKE 2026-11-27] 1 -> 0：带速直接转。
                                          关掉后 ARC 段直接用弧线中心速度 115，
                                          入口不再"两轮同时制动 20 拍"。
                                          实测时间账（400 档）：
                                            开 = 制动20拍 + 转22拍 + 交接5拍 ≈ 47 拍
                                            关 = 转22拍 + 交接5拍          ≈ 27 拍（快 43%）
                                          半径从 4.8 cm 变成 6.2 cm（仍 < 8 cm 下界）。
                                          回滚：改回 1 即恢复入口制动。 */
#endif
#define ARC_ENTRY_SPEED          0.0f
#define ARC_ENTRY_EXIT           ARC_CENTER_SPEED
#define ARC_ENTRY_BRAKE_MAX_TICKS 20U    /* 200 ms 兜底：速度一直降不下来就交回 */

/* [ARC2 2026-11-21] 弧线断电后的航向收尾。
   ARC_SETTLE_ENABLE = 0 就是回滚：断电后直接交回巡线（旧行为，一行开关）。 */
#define ARC_SETTLE_ENABLE       0        /* 1 = 断电后先闭环补角，再交回巡线 */
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

/* ---- [ARC-BRAKE 2026-11-23] ARC 入口的里程内环刹车 ----
   现有 -70 + 速度环，日志里内轮从 +40~+280 回到 0 大约 50~100 ms；
   0.69 m/s 下约 3.5~6.9 cm。先取 4 cm 作为刹车距离目标，
   后面按实测再调 ARC_BRAKE_DIST_CM。

   控制方式：只覆盖内轮的 CCR，不改外轮。
   ccr_mag = BASE + KP * (目标里程 - 已走里程)，限制在 [0, MAX]。
   到目标里程 / 超时后交回原速度环。 */
#define ARC_BRAKE_DIST_CM       4.0f
#define ARC_BRAKE_DIST_COUNTS   ((uint32_t)(ARC_BRAKE_DIST_CM * COUNTS_PER_CM))
#define ARC_BRAKE_KP            0.7f     /* CCR / count：4 cm 误差约 1220 CCR */
#define ARC_BRAKE_BASE_CCR      300.0f   /* 到目标时保留的基础反接量，接近原 -70 前馈 */
#define ARC_BRAKE_CCR_MAX       1500.0f  /* 里程环最大反接 CCR，不直接打满 PWM */
#define ARC_BRAKE_PID_MAX_TICKS 20U      /* 200 ms 兜底，避免一直卡在刹车态 */

/* ---- 断线区斜穿（[BLIND 2026-11-18]，[2026-11-27] 按实际断线区尺寸重算） ----
   不是直角弯/六边形的场合丢线时，按固定角度斜穿过去找线。
   当前断线区：横向 250 mm × 纵向 650 mm
     -> 转角 = atan(250/650) = 21.04°
     -> 路程 = √(250² + 650²) = 696.4 mm = 69.64 cm
   注意：这两个值必须**一起**重算，只改一个会斜着穿出赛道。 */
#define BLIND_ANGLE_DEG         21.04f   /* [BLIND 2026-11-27] atan(250/650) = 21.04°
                                            （上一版 22.88° 与注释里的 300/650 对不上，这里统一） */
#define BLIND_DIST_CM           69.64f   /* [BLIND 2026-11-27] √(250²+650²) = 696.4 mm = 69.64 cm
                                            （上一版 71.6 cm 对应的是 300/650） */
#define BLIND_DIST_COUNTS       ((uint32_t)(BLIND_DIST_CM * COUNTS_PER_CM))
#define BLIND_START_TICKS       5U       /* 丢线这么多拍后开始斜穿 */
#define BLIND_SPEED             500.0f   /* [BLIND 2026-11-22] 60 -> 500：断线区 650 mm 快速穿过 */
#define BLIND_ACCEL             50.0f    /* [BLIND 2026-11-22] 直行段加速斜率（counts/10ms 每拍）：
                                            从 BLIND_TURN_SPEED 爬到 BLIND_SPEED 约 9 拍，
                                            避免 45 -> 500 的阶跃导致"蹦一下" */
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
#define BLIND_SEEN_TICKS        2U       /* [FIX 2026-11-18] 连续这么多拍看到线才算"找到"。
                                            [BLIND 2026-11-22] 3 -> 2：500 速度下 3 拍=3.4 cm，
                                            线只露 1~2 拍就会冲过去。 */
#define BLIND_TURN_MAX_TICKS    60U      /* 转向阶段超时（0.6 s），兜底强制进直行 */
#define BLIND_RUN_MAX_TICKS     600U     /* 直行阶段超时（6 s），兜底放弃 */
#define BLIND_HEX_EXIT_LOCKOUT_TICKS 20U /* [BLIND-HEX-COOLDOWN 2026-11-23]
                                            六边形出栈后 20 拍（200 ms）内不启动盲线，
                                            先把控制权交回方向环，避免出栈后立刻盲穿。 */
#define ARC_BRAKE_TICKS         5U       /* [BRAKE 2026-11-18] 旧固定窗口（ARC2 下不再使用） */
#define ARC_BRAKE_MAX_TICKS     20U      /* [ARC2 2026-11-21] 刹车窗口上限 20 拍（200 ms）。
                                            实际会提前结束：内轮实测降到 ARC_BRAKE_SPEED_MIN
                                            以下就不再反接。这样不依赖电池电压/入弯速度。 */
#define ARC_BRAKE_TARGET        -50.0f   /* 反脉冲的目标速度（只越过反向死区，不是全速反接） */
#define ARC_BRAKE_SPEED_MIN     40.0f    /* 实测速度高于此值才反接（低速反接没意义还伤电机） */
#define ARC_TRIG_ERR            3.5f     /* [ARC 2026-11-18] 圆弧触发还要求误差已接近极限。
                                            只靠"宽线"会把缓弯、十字路口全当成直角弯
                                            （方向环太强：mask=4095 全亮也触发过）。 */
#define KINEMA_TD               0.65f     /* [CENTER 2026-11-18] 0.3 -> 0.5：增益拿回 16 后
                                            需要更多阻尼，否则又会摆。 */
#define DIR_DAMP                0.25f    /* [DAMP 2026-11-16] 陀螺阻尼：turn -= DIR_DAMP × gyro_z。
                                             车转 100°/s 时收回 25，抑制中心附近的来回摆。
                                             过大会"转不动"，建议 0.15~0.30 */
#define TURN_SLEW               27.0f    /* [SPEED300 2026-11-21] 5 -> 20。
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
#define CORNER_CW3_CONFIRM_TICKS 2U      /* [CW4 2026-11-22] CW3 看到全亮后，再要求"线完全
                                            离开阵列"连续这么多拍才真正触发。
                                            2 拍 ≈ 20 ms ≈ 1.4 cm（300 速度下）。
                                            0 = 回到"看到全亮立刻触发"。
                                            用来挡 17 号六边形那种"宽实心带"：线不会全丢，
                                            等到 CORNER_FULL_HOLD_TICKS(25) 就作废。 */
#define CORNER_SIDE_FULL        6U       /* [FIX 2026-11-21] 5 -> 6（退回）：5 路是"45° 斜穿"
                                            和"刁钻路口直角弯"的重叠区，用路数切不开 ——
                                            5 会让 45° 也被判成直角弯。
                                            退回 6：45° 安全；刁钻路口不触发，靠 HOLD/盲穿兜底。
                                            要救那个路口，得找"路数之外"的第二个判据。 */

/* ---- [CW5 2026-11-28] 不对称判据（第二条判据）----
   现场数据（2026-11-28 直角弯冲出线那次）：
       LOST,28,4000,435,414,0      mask = 4000 = 0b111110100000
       点亮 ch5,ch7,ch8,ch9,ch10,ch11（ch6 暗）
       按 6/6 分边：左 = 1 路，右 = 5 路
     而 CORNER_SIDE_FULL = 6 要求单侧 >= 6 -> **一次都没满足**，
     所以连候选（CWD,4）都没挂起，车完全不认为是直角弯。

   为什么不能直接把 CORNER_SIDE_FULL 降到 5：
      注释里写了，5 路会和"45° 斜穿"重叠，切不开。

   第二条判据用的是**不对称性**，不是路数：
       一侧 >= CORNER_SIDE_HI(5) 且 另一侧 <= CORNER_SIDE_LO(1)
     - 90° 直角弯：线从阵列一侧甩出去 -> 1 / 5 或 0 / 6，严重不对称
     - 45° 斜穿中段：线斜跨阵列中部 -> 3 / 3，基本对称
   与原来的 r>=6 判据是 **或** 关系，所以不会削弱原有行为。

   风险：45° 折角在"线贴到最边缘"的那一两拍也可能是 0/5。
   但还要同时满足 CORNER_L_FULL_TICKS(2) 连续两拍 + 之后完全丢线 2 拍，
   所以误判概率比单看 mask 低。**如果 45° 开始被当成直角弯，把这个开关改 0。**

   回滚：CORNER_CW5_ENABLE = 0（CORNER_SIDE_HI 变成 99，条件永不成立）。 */
#define CORNER_CW5_ENABLE       1
#if (CORNER_CW5_ENABLE != 0)
# define CORNER_SIDE_HI         5U       /* 单侧至少 5 路 */
# define CORNER_SIDE_LO         2U       /* 另一侧至多 2 路。
                                            1 -> 2（2026-11-28）：实测连续直角弯的最后一个
                                            给出 4016 = 左 2 / 右 5，用 <=1 还是漏掉。
                                            放宽到 2 仍然安全：真正的区分因素是"一侧 >=5"，
                                            45° 斜穿时左右是 3/3，右侧到不了 5。 */
#else
# define CORNER_SIDE_HI        99U       /* 永不满足 = 关闭 */
# define CORNER_SIDE_LO         0U
#endif

/* [2026-11-24] 全亮否决开关：
   1 = 看到 >=10 路亮（十字/地面标记）就清掉 CW4 已挂起的候选；
   0 = 不因全亮取消候选，让一侧 6 路全亮 + 后接丢线仍然可以判直角弯。
   现场问题：六边形出口 / 45° 接直角的位置，线会短时全亮，
   原来的 1 会把候选清掉，出口后的直角弯就不转了。 */
#define CORNER_FULLBRIGHT_VETO  1
#define CORNER_FULL_HOLD_TICKS  25U      /* [CW2 2026-11-21] 手推实测定的，不是猜的：
                                            直角弯"最后一拍全亮" -> "丢线第一拍"实测 18 拍
                                            （mask 127 持续 16 拍 -> 124 -> 60 -> 48 -> 32 -> 0）。
                                            取 18 + 余量 = 25 拍（250 ms，300 下约 17 cm）。 */
#define CORNER_SPEED            25.0f    /* [PIVOT 2026-11-14] 恢复 25：仍供六边形用；过弯 pivot 分支自己写 0 */
#define CORNER_EXIT_SPEED       150.0f   /* [ARC2 2026-11-21] 0 -> 150（= ARC_OUTER_SPEED/2）。
                                            出弯斜坡从这个速度起，150 -> 300 用 150 ms。
                                            原来是 0：那是 pivot 停车方案的起点，
                                            但 ARC2 出弯时车还有 150，从 0 起会先急刹一下。 */
#define CORNER_RAMP_TICKS       8U//15U      /* 150 ms */
#define CORNER_YAW_EXIT         70.0f
#define CORNER_MIN_TICKS        12U      /* 120 ms（pivot 方案下已不使用） */
#define CORNER_MAX_TICKS        200U     /* [GND 2026-11-14] 80->200：实测 ω≈57°/s，转 82° 要 144 拍；
                                            80 拍只够 46°——每次被超时掐断->再触发->反向再转 */
#define CORNER_COOLDOWN         40U      /* 旧的时间制冷却（已不用，保留兼容） */
#define CORNER_COOLDOWN_CM      2.0f     /* [COOLDOWN 2026-11-18] 出弯冷却改为距离制：
                                            走过 2 cm 才允许下一次锁存。
                                            时间制 0.4 s 在 BASE 120 下 = 11 cm，
                                            比锯齿段的角间距（15~20 cm）还长，会漏弯。 */
#define CORNER_COOLDOWN_COUNTS  ((uint32_t)(CORNER_COOLDOWN_CM * COUNTS_PER_CM))
#define CORNER_RAMP_TICKS       8U//15U      /* 150 ms */

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
#define CORNER_RECOVER_DEG      40.0f    /* [LOST3 2026-11-22] 90 -> 20：按用户逻辑，
                                            直角弯转完还丢线只补 20°（同向补转 + 反向找线
                                            共用这个上限）；补满 20° 还没线就 LOST_STOP 停车，
                                            不再落到 45/60 的 90° 补转。 */
#define CORNER_RECOVER_TICKS    140U     /* 拍数兜底；20° 会先到角度判据，140 拍只是保险 */
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
             c = ch5 || ch6（正中；[V-ONE] 两路似乎严苛，尝试只看一路）
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
                                            直接当成 V。
                                            [V-WIDE 2026-11-22] 判据输出先做 1 拍展宽
                                            （v = 本拍 || 上一拍），再做这个 2 拍去抖：
                                            实测六边形顶点 V 只持续 1 拍，直接去抖会漏。 */

/* ---- [HEX4 2026-11-21] 六边形的"两次 V"状态机（按现场描述重写） ----
   入口：第一次 V 武装（记录大误差方向 + 把方向环打偏的那部分回正），
         10~20 cm 内出现**同侧**第二次 V 且大误差方向一致 -> 进入，向记录方向打 30°；
   途中：交回方向环走六边形，陀螺累计转角；
   出口：有向净角累计 320~340° 后强制出栈，向反方向打 30°，
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
#define HEX4_ARM_DIST_CM        10.0f    /* [HEX4] NEED_V2=0 时，武装后走这么远就进。
                                            [2026-11-27] 9.0 -> 10.0：出入口多留 1 cm 余量。
                                            上限 HEX4_V2_MAX_CM = 20 cm 不变，仍有 10 cm 富余；
                                            COUNTS 宏自动跟着变（10 × 435.7 ≈ 4357）。
                                            太大会冲过顶点、太小会没转到位，一次只动 0.5~1 cm。 */
#define HEX4_ARM_DIST_COUNTS    ((uint32_t)(HEX4_ARM_DIST_CM * COUNTS_PER_CM))

/* [HEX4-ENTRYONLY 2026-11-23] 六边形逻辑只保留入口 30° 打舵：
   入口 V1 -> 9 cm -> TURN_IN 30° -> 交回方向环，不再跑 DRIVING/出口 V/TURN_OUT。
   之后靠方向环和直角弯检测走完六边形；用 lockout 防止刚打完 30° 又被 V 重新入口。
   回滚：HEX4_ENTRY_ONLY=0。 */
#define HEX4_ENTRY_ONLY         0
#define HEX4_ENTRY_ONLY_LOCKOUT_CM 120.0f  /* 入口完成后 120 cm 内不再识别六边形 V */
#define HEX4_ENTRY_ONLY_LOCKOUT_COUNTS ((uint32_t)(HEX4_ENTRY_ONLY_LOCKOUT_CM * COUNTS_PER_CM))

/* [HEX4-NET 2026-11-23] 有方向的角度积分出口：
   从入口 30° 打舵开始累计 hx_net = ∫ hx_dir·gyro_z dt。
   - 正常六边形：hx_net 连续涨到 320~340° 左右 -> 强制出栈打舵；
   - 双六边形：先涨后反向回落；峰值 - 当前 >120° 且回到 ±20° 左右
     -> 也强制出栈打舵；
   - 这两条都不是固定“总转角”，而是看有方向的净进度。 */
#define HEX4_NET_EXIT_DEG       320.0f   /* 出栈窗口 320~340°；到 320° 就强制，
                                            避免 10 ms 采样直接跨过 340° 上沿。 */
#define HEX4_NET_REV_DROP_DEG   120.0f   /* 从峰值反向回落超过这么多 */
#define HEX4_NET_REV_END_DEG    20.0f    /* 且净角回到这个范围，判双六边形返回 */

/* [V2 2026-11-22] 异侧否决：V1 之后、距离窗口内如果再出现一个**异侧** V，
   就取消这次进入（用于双六边形 / 17 号六边形——那种位置 V1 和 V2 是反的）。
   手推实测：V1 左侧 mask 227，4.6 cm 后出现右侧 V mask 3296（HEX,13,2008）。
   所以窗口下限不能沿用 V2 的 6 cm，单独开到 2~9 cm（9 cm 是进弯距离，必须在它之前）。
   1 = 启用否决（HEX,14）；0 = 只打日志。 */
#define HEX4_OPP_VETO           1
#define HEX4_OPP_MIN_CM         2.0f
#define HEX4_OPP_MAX_CM         9.0f
#define HEX4_OPP_MIN_COUNTS     ((uint32_t)(HEX4_OPP_MIN_CM * COUNTS_PER_CM))
#define HEX4_OPP_MAX_COUNTS     ((uint32_t)(HEX4_OPP_MAX_CM * COUNTS_PER_CM))

/* [HEX4-FLOW-VETO 2026-11-23 -> 2026-11-24 禁用]
   原意：武装 V 之后，如果 FLOW 判出与 V1 同侧的直角弯，就取消这次
   六边形入口（记 HEX,19），用来补 17 号节点出口漏检的 V。
   现场结论：FLOW 的直角弯和六边形 V 不是同一类几何，用它否决 V1
   会把正常入口一起挡掉，双六边形/17 号反而更乱 —— 禁用。
   0 = HEX,19 不再产生，Hex_Update4 也不看 flow_fire。
   代码保留仅作回滚参考，默认不再启用。 */
#define HEX4_FLOW_VETO          0
#define HEX4_FLOW_VETO_LATCH_TICKS 20U  /* 仅 HEX4_FLOW_VETO=1 时有效 */

/* ---- [HOLD 2026-11-21] 丢线后保持转向 ----
   45°/60° 弯的线会在 5~6 拍内滑出阵列；那一拍 have_control 变 0，
   原来的代码把两个轮子的目标瞬间清零，车直冲出去（看起来"方向环没努力"）。
   这里在丢线后继续用最后一拍的舵打 N 拍，让车往线消失的方向拐。 */
#define LINE_HOLD_TICKS         16U      /* 保持 16 拍（160 ms）。
                                            按 60° 弯算：turn 顶到 240 时 ω≈394°/s，
                                            转 60° 要 ~15 拍，所以 16 才够；8 拍时只能转 30°。 */
#define LINE_HOLD_SPEED_K       0.6f     /* 保持期间速度 = 0.8 × BASE。
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

/* [LOST3 2026-11-22] 丢线分流门限（手推实测）：
   断路 -0.5 路；45°/60° 丢线 3.5~5.5 路。
   2.0 做"弯 vs 断路"的分界，45/60 与 90 不靠 error 区分（90 靠 CW 全亮候选）。 */
#define LOST_BEND_ERR           2.0f
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
#define HEX4_TURN_DEG           27.0f    /* 进/出各打 30° */
#define HEX4_TURN_TOL           4.0f     /* 角度容差 */
#define HEX4_TURN_RATE          45.0f    /* 打舵用的差速（原地转） */
#define HEX4_TURN_MAX_CM        12.0f    /* 打舵兜底：走这么多 cm 没转够就放弃 */
#define HEX4_TURN_MAX_COUNTS    ((uint32_t)(HEX4_TURN_MAX_CM * COUNTS_PER_CM))

/* [2026-11-24] 出栈打舵开关：
   1 = 出栈后按 hx_dir 原地打 HEX4_TURN_DEG(30°)，再交回方向环；
   0 = 不打舵，直接清 hex 状态交回方向环（仍然发 HEX,10 表示出栈完成）。
   暂时置 0，先看方向环能否自然把车带出出口。 */
#define HEX4_TURN_OUT_ENABLE    1
#define HEX4_EXIT_MIN_DEG       260.0f   /* 保留 260° 下限，避免 DRIVING 早期 V 误出口。
                                            上限和角度强制解除先放开，只靠 300 cm 兜底。 */
#define HEX4_EXIT_MAX_DEG       10000.0f /* [DEBUG 2026-11-23] 先取消出口角度上限：
                                            实测出口 V 在 total=384° 被 370° 上限挡住，
                                            先让 260° 之后任意 V 都能出栈。 */
#define HEX4_EXIT_GIVEUP_DEG    10000.0f /* [DEBUG] 先取消 400° 角度强制解除；
                                            只保留 HEX4_DRIVE_MAX_CM=300 cm 的距离兜底。 */
#define HEX4_DRIVE_MAX_CM       300.0f   /* [HEX4 2026-11-23] DRIVING 距离兜底：150 -> 300 cm。
                                            双六边形/大六边形一圈可能超过 150 cm，原来的 150 cm
                                            会在出栈 V 之前强制退出（实测 total=329 就被掐断）。
                                            300 cm 只作为最后的卡死保护，不再参与正常出栈判断。 */
#define HEX4_DRIVE_MAX_COUNTS   ((uint32_t)(HEX4_DRIVE_MAX_CM * COUNTS_PER_CM))

/* [2026-11-24] 出栈锁：刚出栈（HEX,10 或 DRIVING 的 HEX,8 超时）后
   20 cm 内不允许再次入栈。只禁止 ARMED->TURN_IN，不禁止武装，
   这样还能继续检测 V；走过 20 cm 后自动解除。
   注意：不是“20 cm 内不识别 V”，是“20 cm 内不进去”。 */
#define HEX4_EXIT_LOCKOUT_CM    20.0f
#define HEX4_EXIT_LOCKOUT_COUNTS ((uint32_t)(HEX4_EXIT_LOCKOUT_CM * COUNTS_PER_CM))

/* [HEX4-TURNBACK 2026-11-25] TURN_IN 转完（30°）仍然没看到线怎么办。
   原来：进 DRIVING -> 下一拍 line.cnt=0 -> 落到 HOLD（沿同一侧继续加舵到 240），
   越打越远，看起来像"转过去是空的还在硬拐"。
   现在：改成**反向转回**——朝 -hx_dir 原地打舵，转到看到线就立刻收手，
   六边形标记全部清零、把控制权交回方向环；转满和 TURN_IN 一样的角度还没线
   也一样清零交回，不再硬转。这条分支不经过 HOLD；HOLD 本身不改，
   对它自己的丢线（45°/60° 弯等）继续生效。
   1 = 启用反向转回（默认）；0 = 回滚到原来的 DRIVING/HOLD 行为。 */
#define HEX4_TURNBACK_ENABLE    1
/* 反向转回结束后 20 cm 内不再武装/入栈：车是掉头往回走的，入口那个 V 马上
   又会出现在阵列里，不锁就会 TURN_IN -> TURN_BACK 死循环。锁定期内照常巡线，
   只是不进六边形状态机。 */
#define HEX4_TURNBACK_LOCKOUT_CM    20.0f
#define HEX4_TURNBACK_LOCKOUT_COUNTS ((uint32_t)(HEX4_TURNBACK_LOCKOUT_CM * COUNTS_PER_CM))

/* [HEX4-TURNBACK 2026-11-25] 入口"没线"窗口（第二版）：
   原来只在 TURN_IN 转完的**那一拍**判一次线。现场出现"只亮一个灯停在
   turnback 点"：那一拍恰好还剩 2 路（线压在 ch5/ch6，error≈0）-> 进 DRIVING；
   下一拍只剩单灯，被 Sensor_Scan 的 filt 清成 mask=0 / line.cnt=0，
   于是方向环没线、HOLD 又因为 error≈0 没武装、六边形区间还挡着盲穿
   -> 目标清零、车停住。其实这就是"转弯完成没找到线"，应该反向转回。
   现在：只要还在入口完成后的 HEX4_TURNBACK_GRACE_CM 内，连续
   HEX4_TURNBACK_LOST_TICKS 拍看不到（过滤后的）线，也进 TURN_BACK。
   出了这个窗口就恢复正常 DRIVING（HOLD/盲穿照旧）。 */
#define HEX4_TURNBACK_GRACE_CM      15.0f
#define HEX4_TURNBACK_GRACE_COUNTS  ((uint32_t)(HEX4_TURNBACK_GRACE_CM * COUNTS_PER_CM))
#define HEX4_TURNBACK_LOST_TICKS    3U    /* 连续丢线拍数：3 拍 = 30 ms，挡断线缺口毛刺 */

/* [2026-11-24] FULLDIST 硬编码：拆成两段短的，只覆盖两次实际失败点。
   早段失败：FULLDIST 813510~819796（LOST 前最后一拍 814975）；
   晚段失败：FULLDIST 851024~861352（LOST 前最后一拍 855615）。
   中间没有覆盖到的区间保持 HOLD，不能为了省事连成一整段——
   现场有 5 个弯必须靠 HOLD 才能过。
   注意：这两个区间必须对 state2/3 也生效（HOLD 是在 TURN_IN 丢线后
   才启动的），不能再加 Hex_ZoneActive()==0 的条件。 */
#define FULLDIST_BLOCK1_MIN 808000U
#define FULLDIST_BLOCK1_MAX 821000U
#define FULLDIST_BLOCK2_MIN 849000U
#define FULLDIST_BLOCK2_MAX 863000U

/* [2026-11-24] FULLDIST 区间内的丢线找线：
   在这个区间里一旦丢线，优先级最高：向线消失的那一侧原地转找线，最多 50°；
   看到线立刻交回方向环；50° 还没线就停手（不重启，避免反复转）。
   方向由丢线前最后一拍的 error 符号决定（error<0 线在左 -> 左转；
   error>0 线在右 -> 右转），不能固定右转，否则镜像失败会越转越远。
   区间外完全不执行，HOLD 保持可用。
   宏名里的 REC30 是历史名字，实际角度已改成 50°。 */
#define FULLDIST_REC30_TURN     45.0f    /* 原地转向差速 */
#define FULLDIST_REC30_DEG      50.0f    /* 最大转角（30 -> 50） */
#define FULLDIST_REC30_TICKS    120U     /* 兜底 1.2 s，给 50° 留够时间 */

#define HEX_V_STRICT_END        0        /* [V-ONE 2026-11-23] 两路似乎严苛，尝试只看一路：
                                              1 = 端部要求 ch0&ch1 / ch10&ch11 两路都亮；
                                              0 = 只看 ch0||ch1 / ch10||ch11 任意一路。
                                              c_s 也同步改成 ch5||ch6（见 sensor.c）。
                                              回滚：改回 1，并把 sensor.c 的 center 改回 &&。 */

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

/* ---- [HEX-GAIN 2026-11-27] 六边形打舵专用的速度环增益 ----
   背景：TURN_IN / TURN_OUT / TURN_BACK 这三段都是"两轮反向原地转"，
   但代码里从来没切过增益，一直沿用直道的 SPEED_KP_DEFAULT(2.5)/SPEED_KI_DEFAULT(0.01)；
   而直角弯 pivot 会切到 PIVOT_KP(3.0)/PIVOT_KI(0.02)。
   两者物理工况一样、增益却不同，手感自然不一致 —— 这就是"六边形打舵和
   直角弯打舵不像同一辆车"的来源之一。
   这里给六边形单独一组，默认跟随直角弯 pivot（先验证"切增益"这个方向对不对），
   之后可以只动这三个宏、完全不牵连直角弯：
     · 打舵发抖 / 一顿一顿  -> HEX4_PIVOT_KP 降到 2.0，甚至 1.5
     · 转动偏软、到不了 26° -> 往上加 0.5
     · "先憋住再一冲"       -> HEX4_PIVOT_KI 减半
   回滚：HEX4_USE_PIVOT_GAINS 改 0，行为与改动前完全一致（始终用直道增益）。 */
#define HEX4_USE_PIVOT_GAINS    1
#define HEX4_PIVOT_KP           PIVOT_KP
#define HEX4_PIVOT_KI           PIVOT_KI

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
#define ENC_DELTA_LIMIT         2300     /* [SPEED300 2026-11-21] 500 -> 2000。
                                             BASE 300 时正常一拍增量就有 300，弯道峰值 450，
                                             500 只差一点就会误杀真数据（坏采样保护沿用旧值）。 */

/* ---- 低速增益调度 ---- */
#define KP_LOW_SPEED            10.0f
#define KP_BOOST                2.5f

/* ---- [FLOW 2026-11-23] 流动直角弯检测（第一阶段只观察，不接管） ----
   1.28 s @ 100 Hz 的 mask 窗口；取最中间的连续灯段，按“上一拍主段中心”
   分左右；连续段突然变宽，且新增通道只在某一侧 -> 该侧直角弯。
   两侧同一拍都新增 -> 十字/发车线。 */
#define FLOW_WIN_TICKS          128U     /* 100 Hz × 1.28 s */
/* [FLOW-OR 2026-11-23] 两套直角弯触发源，独立开关：
   - CW4：老策略，"一侧全亮 2 拍 + 全丢线 2 拍"，兜底；
   - FLOW：新策略，"主连续段宽度跳变 + 单侧新增通道"，更早。
   两者是 OR 关系：任意一套先触发就进 ARC。
   debug 时单独置 0 即可关掉其中一套。

   [2026-11-24] FLOW 只做直角弯观察/触发，**不参与六边形入口否决**。
   HEX4_FLOW_VETO 已置 0；即使以后重开，也只应在确认 FLOW 与六边形 V
   几何等价之后再启用。 */
#define CORNER_TRIGGER_CW4      1
#define CORNER_TRIGGER_FLOW     0

#define FLOW_MIN_WIDTH_JUMP     3U       /* 主连续段至少突然变宽这么多路 */
#define FLOW_CROSS_MIN_NEW_CH   2U       /* 十字/发车线：每一侧至少新增这么多路。
                                            怕误触发就保持 2；想放宽改 1。 */
#define FLOW_COOLDOWN_TICKS     7U       /* 5 cm @ 0.69 m/s = 72.5 ms ≈ 7 拍（70 ms）。
                                            7 拍 = 4.83 cm，刚好短于最近 5 cm 的直角弯间距，
                                            不会把下一个弯挡在冷却里。 */

/* [FLOW-SNAP 2026-11-23] 检测到左/右直角弯后自动抓 1.28 s 窗口：
   中断里只做 256 字节拷贝，不 printf；主循环再按 FL 行输出。
   FLOW_AUTO_DUMP=0 可关掉自动抓取，只保留 FLOW 事件行。 */
#define FLOW_AUTO_DUMP          1
#define FLOW_SNAP_DELAY_TICKS   10U      /* 事件后再抓 10 拍（100 ms），让窗口包含触发后的状态 */

/* ---- [FINISH3 2026-11-27] 50 m 后终线识别停车 ----
   逻辑：
   - 里程不到 FINISH_ARM_MM（50 m）之前，完全不做终线识别；
   - 过了 50 m，第一次出现"12 路全亮"就当终点线：记下里程；
   - 之后保持正常速度再走 FINISH_STOP_DIST_MM（30 cm），然后 Car_Stop。
   为什么 50 m：第一圈回发车区约 24.6 m（标称）会被挡住；50 m 之后的第一根
   横条就是最后一次经过发车区。若实车两圈里程不足 50 m（跑完两圈不停），
   把 FINISH_ARM_MM 降到 47000 再试；若提前停了就往上加。
   里程用左右轮编码器累计（mm），只做"闸门"和"最后 30 cm"两个短距离判断。

   可选保险：若实车在闸门后仍被弯道全亮误触发，把 FINISH_PULSE_RATE_EN 改 1
   （认线那一拍要求 |gyro_z| < FINISH_PULSE_RATE = 60°/s，即必须正在走直线）。
   回滚：FINISH_ENABLE 改 0。 */
#define FINISH_ENABLE            1
#define FINISH_ARM_MM            50000.0f  /* 前 50 m 不做终线识别 */
#define FINISH_STOP_DIST_MM      300.0f    /* 认出终线后再走 30 cm 停车 */
#define FINISH_LIT_CNT           11U       /* 全亮判据：>=11/12 路 */
#define FINISH_LIT_TICKS         1U        /* 连续几拍算数（10 ms/拍） */
#define FINISH_PULSE_RATE_EN     0         /* 1 = 启用"走直线"附加条件 */
#define FINISH_PULSE_RATE        60.0f     /* 启用时要求 |gyro_z| < 该值（°/s） */

/* ---- [POST 2026-11-27] 终点后的收尾小任务 ----
   触发：FINISH 逻辑"认出终线后再走 FINISH_STOP_DIST_MM"走完那一刻，不再直接
   Car_Stop，而是依次执行：
     1) 原地停 POST_WAIT_TICKS 拍（150 拍 = 1.5 s）；
     2) 原地自旋 POST_SPIN_DEG（180°，陀螺闭环，左转）；
     3) 半速巡线直行（POST_SPEED），直到 12 路全亮（POST_LIT_CNT）→ 立刻停，
        让车停在发车区横条上、12 个光电管全压在线上；
        兜底：巡线超过 POST_FOLLOW_MAX_TICKS 或 POST_FOLLOW_MAX_MM 还没全亮就停。
   这段在 Control_Direction 里以 100 Hz 跑，不经过直角弯/六边形状态机。
   注意：POST_SPEED 默认 100（≈1/3 速）而不是 150，因为识别到全亮后是"滑行"
   停车（代码禁止反接制动），速度越低滑得越短，才停得住横条；如果停过头就
   往下调 80，停不到就调 120。
   回滚：POST_ENABLE 改 0（就恢复成原来的"认出终点线后直接停"）。 */
#define POST_ENABLE              1
#define POST_WAIT_TICKS          150U     /* 停 1.5 s（10 ms/拍） */
#define POST_SPIN_DEG            180.0f   /* 自旋角度，正 = 左转 */
#define POST_SPIN_TOL            3.0f
#define POST_SPIN_KICK           42.0f    /* 起步踹一下，破静摩擦 */
#define POST_SPIN_KICK_TICKS     18U
#define POST_SPIN_KP             0.6f
#define POST_SPIN_MIN            30.0f
#define POST_SPIN_MAX            42.0f
#define POST_SPIN_TIMEOUT        500U     /* 5 s 还没转完就放弃 */
#define POST_SPEED               100.0f   /* 巡线速度（BASE 的 1/3） */
#define POST_TURN_KP             45.0f    /* 每 1 路偏差给多少差速 */
#define POST_TURN_MAX            80.0f
#define POST_LIT_CNT             11U      /* [POST-LIT-FIX 2026-11-27] 12 -> 11：与 FINISH_LIT_CNT 对齐。
                                             原来要求 12/12 全亮，比终线识别的 11 更严；
                                             而边缘通道（ch0/ch11）斜压横条时经常漏读，
                                             导致收尾段永远等不到全亮、跑满 1.5 m 后超时停车。 */
#define POST_LIT_TICKS           1U
#define POST_STOP_TICKS          20U      /* 停车后再保持 0 目标 200 ms */
#define POST_FOLLOW_MAX_TICKS    800U     /* 8 s 找不到横条就停 */
#define POST_FOLLOW_MAX_MM       1500.0f  /* 或最多再走 1.5 m */

/* ---- 陀螺 ---- */
#define GYRO_ALPHA              0.3f
#define GYRO_SAMPLE_DT          0.005f   /* [YAW 2026-11-15] 采样周期 = 5 ms（TIM2 的 5 拍） */

#endif /* CAR_CONFIG_H */
