/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * 本文件是基于学长代码改造的小车赛校赛选拔草稿
 * 
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "car_config.h"
#include "motor.h"
#include "special.h"
#include "timebase.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 陀螺仪调试打印开关。置 1 会引入约 5.4 kB 的浮点 printf 代码，
   MDK-Lite 的 32 kB 链接上限会因此报 L6047U 而链接失败。
   平时保持 0；要串口看数据时改成 1（并注意镜像大小）。 */
#define GYRO_DEBUG 0
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_tim1_up;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//负压启动相关宏
//负压启动流程：900us脉宽50Hzpwm信号输出2秒后，改为持续2000us脉宽50Hzpwm信号输出
#define ESC_PULSE_SLOTS       200U       /* 20 ms at the 10 kHz TIM1 update rate 一个帧（周期）*/
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms 启动帧数*/
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms 运行帧数*/
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s 解锁时间*/

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;

/* Fill a whole 20 ms frame.  Only used once, before the DMA is started. */
//初始化帧
static void ESC_BuildDmaTable(uint32_t high_slots)
{
  uint32_t i;

  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}

/* Rewrite only the slots whose level really changes, in increasing index
   order: the circular DMA consumes the table in that same order, and slot 9
   is not transferred before ~0.9 ms after this interrupt, so the frame that
   is already running is never disturbed. */
//调帧
static void ESC_SetPulseWidth(uint32_t high_slots)
{
  uint32_t first;
  uint32_t last;
  uint32_t i;

  if (high_slots == esc_high_slots)
  {
    return;
  }

  first = (high_slots < esc_high_slots) ? high_slots : esc_high_slots;
  last  = (high_slots > esc_high_slots) ? high_slots : esc_high_slots;

  for (i = first; i < last; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}


//满血启动帧
static void ESC_fullPulseWidth(void){
  uint32_t i;
  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = GPIO_PIN_11;
  }
  esc_high_slots = ESC_PULSE_SLOTS;
}
/* One interrupt per 20 ms frame: DMA transfer complete in circular mode.
   The 50 Hz waveform itself is produced by TIM1 + DMA in hardware; this
   handler only counts frames and switches 0.9 ms -> 2.0 ms after 2 s. */
//传输完成中断
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma)
{
  if (esc_period_count < ESC_START_PERIODS)
  {
    ++esc_period_count;
    if (esc_period_count == ESC_START_PERIODS)
    {
      /* The frame that has just been completed was the last 0.9 ms frame:
         100 x 20 ms = 2 s have elapsed, 2.0 ms is now held forever. */
      if (!IF_start_Brushless_motor)
        ESC_SetPulseWidth(10U);
      else
        ESC_SetPulseWidth(ESC_RUN_PULSE_SLOTS);
      /* Nothing left to time, so stop the 50 Hz interrupt. */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; // 陀螺仪数据

/* ================= 小车赛控制代码================= */

/* 20 cm 方块直角弯的 0/40 方案：
   直道 BASE=30；
   turn=20 时 speed_cmd=30-0.5×20=20，
   targetL=0、targetR=40，平均 20，
   R=16×20/40=8 cm。
   外轮只有 40，不是 54/60；车心降到 20，是低速 pivot。 */

/* 方向环二次项修正：
   Kp=2.5 时最大误差 5.5 给 13.75，二次项补到约 20：
     2.5×5.5 + 0.21×30.25 ≈ 13.75 + 6.35 ≈ 20.1
     误差 1 时：2.5 + 0.21 ≈ 2.71（中心附近仍然软）
   最大 turn=20，配合 speed_cmd=20，正好 target=0/40。 */

/* 过弯状态机：
   线一回中，纯 PID 会松舵；这里在检测到大误差时锁存方向，
   固定 turn/speed 保持一段时间，直到陀螺转过 70° 或线稳定回来。
   方向环 100 Hz，所以 CORNER_*_TICKS 的单位是 10 ms。 */

/* 六边形入口：实测 435.7 counts/cm（推车 100 cm 标定）。
   11/12 上升沿算一次；第一次 arm，第二次才打右舵。
   两次之间要求车已走过 2 cm（≈871 counts），
   避免同一个 11/12 脉冲被数两次，又不会像 4 cm 那样等太久。 */

/* 原地转 360°：串口输入 '2' 触发 */

/* 脱线直线走 1 m：串口输入 '3' 触发 */

/* 前馈：基础占空比 = FF_GAIN × 目标速度。target = 20 时 60 × 20 = 1200（约 17% 占空比）。
   没有前馈时，Kp=0.5 在误差 100 时只给 50，车根本推不动，光靠积分要爬十几秒。
   标定方法：先把 pid_L / pid_R 的 Kp、Ki 都置 0、只留前馈，
   调 FF_GAIN 到车能匀速慢走，再加回 PID 修偏差。 */
//尤其注意FF_GAIN和速度无关，却对硬件参数高度依赖


/* 死区补偿：占空比低于某个值时电机根本转不起来（静摩擦），
   所以只要给了油，就至少给到每侧运行死区（L=150，R=169），否则 PID 只会在死区里空堆积分，
   表现为车一顿一顿、蛇行（积分饱和典型症状）
复习：一开始上不去，积分项上去后启动，却下不来，就会过加速
   标定方法：开环把 TIM1->CCR1 从 0 慢慢加，记下轮子刚开始转的值，取稍大一点。 */

/* 起步 kick：运行到 20 只需 290/308，但静止起转要 390/428。
   地面实测：连续给 390/428 很容易把车一脚踢到 35~46，然后 FF 保持
   又把它推到 60+。所以改成“短脉冲 kick”：
     390/428 给 60 ms → 0 给 60 ms → 重复，直到两轮都 > 4，
     或总时长 600 ms。
   起来后如果速度已经超过 BASE+2，先 0 占空比滑行，不继续给 FF。 */

/* 中途卡住恢复：速度接近 0 持续 300 ms，就重新脉冲 kick。
   高阻尼/接缝/地毯段，正常运行 FF 290/308 可能推不动，
   需要再给 390/428 的短脉冲。最多重试 3 次，避免一直顶住。 */
/* 现场微调：
   - 起不来：KICK_ON_TICKS 加到 8~10，或 KICK_MAX_TICKS 加到 80
   - 起来还冲：KICK_ON_TICKS 降到 4，或 KICK_EXIT_SPEED 降到 3
   - 一顿一顿：KICK_OFF_TICKS 加到 8~10，或 SETTLE_TICKS 加到 25 */

/* 丢线停车阈值：方向环跑 50 Hz，10 拍 = 200 ms 连续读不到线就停车。
   太短会被赛道接缝/贴纸误触发，太长会冲出赛道。实测再调。 */


/* ---- 调试打印：把中断里算出来的实测速度抄到全局变量，由主循环负责打印 ----
   置 1 = 串口输出调试数据（只在调车时用，跑正式成绩前改回 0）。 */
#define CAR_DEBUG 1
static volatile int16_t dbg_dL = 0, dbg_dR = 0;

/* 停车推车测试用：run_flag=0 时也每 10 ms 更新一次。
   这两个值就是 PID 实际使用的 speed_L / speed_R（包含当前符号），
   主循环把它们打印出来，不发车、用手推车就能看编码器方向和速度。 */
static volatile float dbg_speed_L = 0.0f, dbg_speed_R = 0.0f;

/* ---- 发车按键：PB14（GPIO_RUN），按下为低电平 ----
   run_flag = 0 时，TIM2 中断里把两路 CCR 强制写 0，电机不会出力，
   可以先把车摆到起跑线上再按键。中断和主循环都会访问，必须 volatile。 */
static volatile uint8_t run_flag = 0;

/* 按键锁存：1 = 这次按下已经处理过，必须松开才允许下一次。
   初值给 1，是为了处理"上电时按键已经被按着"的情况 —— 必须先松手再按，
   否则一上电就自己跑起来。 */
static uint8_t key_hold = 1;

/* 连续两次读到低电平（间隔 20 ms）才认为是一次有效按下，用来躲按键抖动。
   只有按键真的按下时才会阻塞这 20 ms，不影响主循环平时 10 Hz 的调试输出。 */
static uint8_t RUN_Key_Scan(void)
{
    if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) != GPIO_PIN_RESET)
        return 0;
    HAL_Delay(20);
    return (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}
/* 发车目标：不再 ramp，直接 BASE_SPEED；起步由固定 kick 负责。
   ramp 在“目标到 20 但车还没动”时仍然会让 P 项叠 100，所以这里取消。 */
static float speed_cmd = 0.0f;

/* 起步状态机 */
static uint8_t  launch_done = 0U;   /* 0 = 还在 kick，1 = 已起来 */
static uint8_t  kick_on = 0U;       /* 当前脉冲是 on 还是 off */
static uint16_t kick_phase = 0U;    /* 当前 on/off 剩余 10 ms 拍数 */
static uint16_t kick_total = 0U;    /* 已 kick 的总拍数 */
static uint16_t settle_timer = 0U;
static uint16_t stall_cnt = 0U;     /* 连续接近 0 速度的拍数 */
static uint8_t  stall_retry = 0U;   /* 卡住重试次数 */

static void Launch_Reset(void)
{
    launch_done = 0U;
    kick_on = 0U;
    kick_phase = 0U;
    kick_total = 0U;
    settle_timer = 0U;
    stall_cnt = 0U;
    stall_retry = 0U;
}

/* ================= 陀螺仪（BMI270 角速度 z 轴） ================= */

static float gyro_z_bias = 0.0f;     /* 静态零偏，开机静止标定得到 */
static float gyro_z_filt = 0.0f;     /* 去零偏 + 低通后的角速度，单位 °/s */
static volatile float dbg_gyro_z = 0.0f;   /* 供主循环打印 */
static volatile uint8_t gyro_ok = 0;       /* 1 = BMI270 初始化成功 */

/* 开机静止标定零偏：调用期间车必须保持不动。
   取 500 次平均（约 0.5 s），扣掉静态零漂，否则角速度环会一直往一边偏。
   必须在 TIM2 启动之前调用，否则会和中断里的 200 Hz 采样抢 SPI。 */
static void Gyro_Calibrate(void)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < 500U; i++)
    {
        dodo_BMI270_get_data();
        sum += BMI270_gyro_transition(BMI270_gyro_z);
        HAL_Delay(1);
    }
    gyro_z_bias = sum / 500.0f;
    gyro_z_filt = 0.0f;
    dbg_gyro_z  = 0.0f;
}

//速度环状态保留在 main.c，PID 结构/算法已移到 motor.c
static int16_t last_L = 0, last_R = 0;

/* 编码器增量合理性保护：
   BASE_SPEED 提到 40 后，起步/直道的真实轮速会超过 100 计数/10ms；
   原来限 100 会把正常高速也当成坏采样，速度反馈卡在 99/91，
   PID 看不到真实车速，车反而刹不住、进弯发抖。
   现在放到 300：正常高速能测到，21159 这种 32k 坏采样仍然会被拒绝。 */
static int16_t last_valid_dL = 0, last_valid_dR = 0;

/* 低速增益调度：
   车停住时目标 20、实测 0，误差 20。
   如果 Kp 一直是 1.5，P 只给 30，总输出约 320，低于起转需要的 390，
   车就会停住，只能靠 stall kick 踢一脚，表现就是一停一停。
   现在巡航 Kp=2.5，KP_BOOST=2.5：
     0 速 → Kp=6.0，P=120，总输出约 410，能自己起转；
     接近 10 → 平滑回到 1.5，巡航不放大噪声。
   只放大 P，不改 FF/Ki/最小占空比，避免影响稳态和巡航。 */

/* PID_Calc / Motor_Output 已移到 motor.c */


/* DirPID_t / dir_pid / DirPID_Calc 已移到 motor.c */

/* 丢线计数：方向环每 10 ms 累加一次，读到线就清零 */
static uint16_t lost_cnt = 0;

/* 过弯状态机（实现已移到 special.c） */
static CornerState corner;

/* 六边形入口检测状态（实现已移到 special.c） */
static HexState hex;

/* 串口命令 + 原地转 360° 状态 */
static volatile uint8_t  spin_request = 0U;
static volatile uint8_t  spin_mode = 0U;
static float    spin_yaw = 0.0f;
static uint16_t spin_ticks = 0U;
static uint8_t  uart_rx_byte = 0U;

/* 脱线直线 1 m 状态 */
static volatile uint8_t  straight_request = 0U;
static volatile uint8_t  straight_mode = 0U;
static float    straight_yaw = 0.0f;
static float    straight_speed = 0.0f;
static uint32_t straight_dist = 0U;
static uint16_t straight_ticks = 0U;

/* 停车：清运行标志 + 复位目标和积分。
   复位目标是为了避免"停车那一刻目标停在左转状态，下次发车直接窜出去"。
   中断和主循环都可能调用，必须是幂等的。 */
static void Car_Stop(void)
{
    run_flag = 0;
    pid_L.target = BASE_SPEED;
    pid_R.target = BASE_SPEED;
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    speed_cmd = 0.0f;
    Launch_Reset();
    lost_cnt = 0;
    Corner_Reset(&corner);

    Hex_Reset(&hex);

    spin_mode = 0U;
    spin_yaw = 0.0f;
    spin_ticks = 0U;

    straight_mode = 0U;
    straight_yaw = 0.0f;
    straight_speed = 0.0f;
    straight_dist = 0U;
    straight_ticks = 0U;
}

/* 启动原地转：不 kick，直接让速度环跟踪左右反向目标 */
static void Spin_Start(void)
{
    run_flag = 1U;
    launch_done = 1U;
    settle_timer = 0U;
    stall_cnt = 0U;
    stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    speed_cmd = 0.0f;

    spin_yaw = 0.0f;
    spin_ticks = 0U;
    straight_mode = 0U;
    spin_mode = 1U;
}

/* 方向环 100 Hz 调用：陀螺积分 + 角度闭环 */
static void Spin_Update(void)
{
    if (!gyro_ok) {
        Car_Stop();
        return;
    }

    spin_yaw += gyro_z_filt * 0.01f;   /* 100 Hz，dt=10ms */
    ++spin_ticks;

    float yaw_abs = (spin_yaw >= 0.0f) ? spin_yaw : -spin_yaw;

    if (yaw_abs >= SPIN_STOP_DEG || spin_ticks >= SPIN_TIMEOUT) {
        Car_Stop();
        return;
    }

    float err = SPIN_TARGET_DEG - yaw_abs;
    float cmd;

    if (spin_ticks <= SPIN_KICK_TICKS)
    {
        /* 前 120 ms 硬踹一下，打破静摩擦 */
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

/* 启动脱线直线 1 m */
static void Straight_Start(void)
{
    run_flag = 1U;
    launch_done = 1U;
    settle_timer = 0U;
    stall_cnt = 0U;
    stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    speed_cmd = 0.0f;

    spin_mode = 0U;
    straight_yaw = 0.0f;
    straight_speed = 0.0f;
    straight_dist = 0U;
    straight_ticks = 0U;
    straight_mode = 1U;
}

/* 方向环 100 Hz 调用：陀螺航向保持 + 编码器距离 */
static void Straight_Update(void)
{
    ++straight_ticks;

    if (straight_dist >= STRAIGHT_TARGET_COUNTS ||
        straight_ticks >= STRAIGHT_TIMEOUT) {
        Car_Stop();
        return;
    }

    if (gyro_ok)
        straight_yaw += gyro_z_filt * 0.01f;

    float remain = (float)STRAIGHT_TARGET_COUNTS - (float)straight_dist;
    if (remain < 0.0f) remain = 0.0f;

    float desired = STRAIGHT_SPEED;
    if (remain < (float)STRAIGHT_DECEL_COUNTS)
    {
        desired = STRAIGHT_MIN_SPEED +
                  (STRAIGHT_SPEED - STRAIGHT_MIN_SPEED) *
                  (remain / (float)STRAIGHT_DECEL_COUNTS);
    }

    if (straight_speed < desired) straight_speed += 0.5f;
    else                          straight_speed -= 0.5f;
    if (straight_speed < 0.0f) straight_speed = 0.0f;

    float corr = 0.0f;
    if (gyro_ok)
    {
        corr = STRAIGHT_KP * (0.0f - straight_yaw)
             - STRAIGHT_KD * gyro_z_filt;
        if (corr >  STRAIGHT_TURN_MAX) corr =  STRAIGHT_TURN_MAX;
        if (corr < -STRAIGHT_TURN_MAX) corr = -STRAIGHT_TURN_MAX;
    }

    pid_L.target = straight_speed - corr;
    pid_R.target = straight_speed + corr;
    if (pid_L.target < 0.0f) pid_L.target = 0.0f;
    if (pid_R.target < 0.0f) pid_R.target = 0.0f;
}

/* 串口单字节命令：'2' 启动原地转 360° */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (uart_rx_byte == '2')
            spin_request = 1U;
        else if (uart_rx_byte == '3')
            straight_request = 1U;

        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
    }
}

//时基

void Control_Update(void)
{
        /*陀螺仪测定
				dodo_BMI270_get_data();
				gyro_z = BMI270_gyro_transition(BMI270_gyro_z);
				*/

        /* ---- 陀螺仪采样：200 Hz（每 5 ms）----
           全工程只能在这里调用 get_data()：它内部是 SPI+DMA 忙等，
           主循环再调一次会把正在进行的事务打断。
           放中断里是为了周期严格；放主循环会被 printf 阻塞抖到好几毫秒。 */
        {
            static uint16_t div_gyro = 0;
            if (++div_gyro >= 5U) {
                div_gyro = 0;
                dodo_BMI270_get_data();
                float raw = BMI270_gyro_transition(BMI270_gyro_z) - gyro_z_bias;
                gyro_z_filt += GYRO_ALPHA * (raw - gyro_z_filt);
                dbg_gyro_z = gyro_z_filt;
            }
        }

        /* 方向环：100 Hz（10 ms） */
        static uint16_t div_dir = 0, div_speed = 0;
        if (++div_dir >= 10) {
            div_dir = 0;

            if (spin_mode) {
                Spin_Update();
            } else if (straight_mode) {
                Straight_Update();
            } else {

            uint16_t mux;
            MUX_get_value(&mux);

            /* 1) 12 路 → 偏差 */
            float pos = 0.0f;
            int cnt = 0;
            for (int i = 0; i <= 11; i++) {
                if (MUX_GET_CHANNEL(mux, i)) { pos += i; cnt++; }
            }

            float bias = 0.0f;
            float error = 0.0f;
            if (cnt > 0) {
                bias = pos / cnt;
                error = bias - 5.5f;
            }

            /* 六边形入口原始传感器：状态机在 special.c */
            uint8_t hex_center = (MUX_GET_CHANNEL(mux, 5) &&
                                  MUX_GET_CHANNEL(mux, 6));
            uint8_t hex_right  = MUX_GET_CHANNEL(mux, 11);
            uint8_t hex_left   = MUX_GET_CHANNEL(mux, 0);

            uint16_t hex_mask = 0U;
            for (int i = 0; i <= 11; i++) {
                if (MUX_GET_CHANNEL(mux, i)) hex_mask |= (uint16_t)(1U << i);
            }

            /* 方向环 PID 先算出来，交给 special 决定是否锁存 */
            float pid_turn = 0.0f;
            if (cnt > 0)
            {
                pid_turn = DirPID_Calc(&dir_pid, error);
                pid_turn += DIR_QUAD * (-error) * ((error >= 0.0f) ? error : -error);
                if (pid_turn >  TURN_MAX) pid_turn =  TURN_MAX;
                if (pid_turn < -TURN_MAX) pid_turn = -TURN_MAX;
            }

            /* 丢线保护：过弯锁存期间不因线暂时消失而停 */
            if (cnt == 0 && !Corner_IsActive(&corner))
            {
                if (++lost_cnt >= LOST_STOP_TICKS) {
                    Car_Stop();
                    return;
                }
            }
            else
            {
                lost_cnt = 0;
            }

            /* 六边形检测先跑，和原来一样先取消通用过弯锁存 */
            HexOutput hex_out = Hex_Update(&hex, &corner,
                                           hex_center, hex_right, hex_left, hex_mask);

            uint8_t corner_was_active = Corner_IsActive(&corner);

            float speed_local = BASE_SPEED;
            float turn = Corner_Update(&corner, pid_turn, error, cnt,
                                       gyro_ok, gyro_z_filt, &speed_local);
            uint8_t have_control = (cnt > 0 || corner_was_active ||
                                    Corner_IsActive(&corner)) ? 1U : 0U;
            speed_cmd = speed_local;

            /* 六边形满舵覆盖通用过弯 */
            if (hex_out.have_control)
            {
                turn = hex_out.turn;
                speed_cmd = hex_out.speed_cmd;
                have_control = 1U;
            }

            /* 5) 差速 → 左右目标速度（过弯/正常统一在这里写） */
            if (have_control)
            {
                pid_L.target = speed_cmd - turn;
                pid_R.target = speed_cmd + turn;
                if (pid_L.target < 0.0f) pid_L.target = 0.0f;
                if (pid_R.target < 0.0f) pid_R.target = 0.0f;
            }

            }   /* else: 非 spin_mode，走巡线/六边形 */
        }

        // ---- 速度环：100 Hz（10ms）
        if (++div_speed >= SPEED_DIV) {
            div_speed = 0;

            int16_t now_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
            int16_t now_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
            int16_t dL = (int16_t)(now_L - last_L);
            int16_t dR = (int16_t)(now_R - last_R);
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

            /* 六边形入口距离累计：左右轮平均路径，单位 counts */
            {
                int32_t dL_abs = (dL < 0) ? -(int32_t)dL : (int32_t)dL;
                int32_t dR_abs = (dR < 0) ? -(int32_t)dR : (int32_t)dR;
                Hex_AddDistance(&hex, (uint32_t)((dL_abs + dR_abs) / 2));
                straight_dist += (uint32_t)((dL_abs + dR_abs) / 2);
            }

            /* PID 实际使用的速度：保持当前符号 */
            float speed_L = -(float)dL;          // TIM3 左
            float speed_R =  (float)dR;          // TIM4 右

            /* 停车推车时也更新，供主循环打印 */
            dbg_speed_L = speed_L;
            dbg_speed_R = speed_R;

            /* 未发车只采样、不跑 PID，避免停车时积分乱积。
               电机由下面的 !run_flag 分支强制写 0。 */
            if (run_flag)
            {
                /* speed_cmd 由方向环按 turn 更新（弯道降速），这里不要再覆盖 */

                if (!launch_done && !spin_mode && !straight_mode)
                {
                    /* 阶段 1：短脉冲 kick：60 ms on / 60 ms off，
                       直到两轮都 >4，或总时长 600 ms。 */
                    if (speed_L > KICK_EXIT_SPEED && speed_R > KICK_EXIT_SPEED)
                    {
                        launch_done = 1U;
                        settle_timer = SETTLE_TICKS;
                    }
                    else if (kick_total < KICK_MAX_TICKS)
                    {
                        if (kick_phase == 0U)
                        {
                            kick_on = (kick_on == 0U) ? 1U : 0U;
                            kick_phase = (kick_on != 0U) ? KICK_ON_TICKS : KICK_OFF_TICKS;
                        }
                        --kick_phase;
                        ++kick_total;

                        if (kick_on != 0U) Motor_Output(KICK_L, KICK_R);
                        else               Motor_Output(0.0f, 0.0f);
                    }
                    else
                    {
                        launch_done = 1U;
                        settle_timer = SETTLE_TICKS;
                    }
                }

                /* 中途卡住：速度接近 0 持续 300 ms，重新脉冲 kick。
                   spin 模式不要走这个逻辑，否则会两轮正转把车往前踹。 */
                if (launch_done && !spin_mode && !straight_mode)
                {
                    if (speed_L < STALL_SPEED && speed_R < STALL_SPEED)
                    {
                        if (++stall_cnt >= STALL_TICKS)
                        {
                            stall_cnt = 0U;
                            if (stall_retry < STALL_RETRIES)
                            {
                                ++stall_retry;
                                launch_done = 0U;
                                kick_on = 0U;
                                kick_phase = 0U;
                                kick_total = 0U;
                                settle_timer = 0U;
                                Motor_Output(0.0f, 0.0f);
                            }
                            else
                            {
                                /* 连续 3 次还起不来，停车保护 */
                                Car_Stop();
                                return;
                            }
                        }
                    }
                    else
                    {
                        stall_cnt = 0U;
                        if (speed_L > 10.0f && speed_R > 10.0f)
                            stall_retry = 0U;
                    }
                }

                if (launch_done)
                {
                    /* 如果已经明显超速，跳过 FF 保持，直接交给 PID 平滑降输出。
                       不要用“超过阈值就 0 占空比”的硬刹车，否则速度在阈值附近
                       来回穿，CCR 会在 0 和 290 之间每 10 ms 跳，看起来就是“非常抖动”。 */
                    if (settle_timer > 0U &&
                        speed_L < BASE_SPEED + 5.0f &&
                        speed_R < BASE_SPEED + 5.0f)
                    {
                        --settle_timer;
                        Motor_Output(FF_GAIN_L * pid_L.target, FF_GAIN_R * pid_R.target);
                    }
                    else
                    {
                        Motor_Output(FF_GAIN_L * pid_L.target + PID_Calc(&pid_L, speed_L),
                                     FF_GAIN_R * pid_R.target + PID_Calc(&pid_R, speed_R));
                    }
                }
            }
        }
				
        /* 未发车：电机停转。 
           注意：这里不要再刷新 last_L/last_R！
           原来每 1 ms 刷新一次，会把 10 ms 的编码器增量冲掉，
           导致停车推车时 dL/dR 永远接近 0（就是“冻结”的原因）。
           现在 last_L/last_R 只在上面的 10 ms 采样里更新，
           所以不发车也能看到真实 speed_L / speed_R。 */
        if (!run_flag) {
            Motor_Output(0.0f, 0.0f);
            speed_cmd = 0.0f;
            Launch_Reset();               /* 下次发车重新 kick */
            return;
        }
}
//
//FF_GAIN * pid_R.target + PID_Calc(&pid_R, speed_R)
/* USER CODE END 0 */
//无刷电机
void Init_brushless_motor(void)
{
  /* PA11 idles low: the first high level then also lasts exactly 0.9 ms,
     because the DMA writes the first slot 0.1 ms after TIM1 is started. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_START_PULSE_SLOTS);
  esc_period_count = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
	//什么时候会做这个处理？堵塞？必要吗？
  if (IF_start_Brushless_motor)
    while (esc_period_count < ESC_START_PERIODS)
      HAL_Delay(200);//等待无刷电桥负压启动完成，这是ESC启动协议
  /* Disabled fan: return immediately; DMA completes the existing
     0.9 ms -> 1.0 ms idle-signal sequence in the background. */
}
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Init_brushless_motor(); //等待无刷电调负压启动完成
  /* USER CODE BEGIN 2 */
  /* 陀螺仪初始化 + 零偏标定。
     标定时车必须静止；此时 TIM2 还没启动，不会和中断里的 200 Hz 采样抢 SPI。 */
  if (dodo_BMI270_init() == 0) {
    gyro_ok = 1;
    Gyro_Calibrate();
  }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  Timebase_Init();      

  /* 开启 USART3 单字节接收：串口输入 '2' 触发原地转 */
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);

  if (CAR_DEBUG)
  {
    printf("CCR1,CCR2,tgtLx10,tgtRx10,dL,dR,speedL,speedR\r\n");
    printf("HEX,event,dist,right,left,center,mask,seq\r\n");
    printf("HEXSTAT,event,offR,offL,right,left,center,dist,arm\r\n");
  }

  pid_L.target = BASE_SPEED ;          // 脉冲/10ms；必须和方向环用同一个常量
  pid_R.target = BASE_SPEED ;

  //pwm示例
  //TIM1->CCR1 = 2000;
  //TIM1->CCR2 = 2000;
  // 延时5000秒，等待无刷电调初始化
  // TIM1->PSC = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*陀螺仪数据获取教学区
    dodo_BMI270_get_data();                         // 调用此函数会更新陀螺仪数据
    gyro_x = BMI270_gyro_transition(BMI270_gyro_x); // 将原始陀螺仪数据转换为物理值，单位为度每秒
    gyro_y = BMI270_gyro_transition(BMI270_gyro_y);
    gyro_z = BMI270_gyro_transition(BMI270_gyro_z);
    accel_x = BMI270_acc_transition(BMI270_accel_x); // 将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
    accel_y = BMI270_acc_transition(BMI270_accel_y);
    accel_z = BMI270_acc_transition(BMI270_accel_z);
    if (GYRO_DEBUG)
      printf("%f,%f,%f,%f,%f,%f\r\n",gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z);*/
    /* ---- 发车键PB14 ----*/
		/*对是否启动非阻塞轮询 */
    /* 边沿触发：按住不放只算一次按下（否则丢线停车后会被一直按着的键立刻重新发车）。
       key_hold 初值 1，要求上电后先松开一次。 */
    if (RUN_Key_Scan())
    {
      if (!key_hold)
      {
        key_hold = 1;
        if (!run_flag)
        {
          pid_L.integral = 0.0f;
          pid_R.integral = 0.0f;
          dir_pid.integral = 0.0f;
          speed_cmd = BASE_SPEED;
          Launch_Reset();            /* 从第一拍开始脉冲 kick */
          run_flag = 1;
        }
      }
    }
    else
    {
      key_hold = 0;          /* 松开，允许下一次按下 */
    }

    /* 串口输入 '2'：启动原地转 360° */
    if (spin_request)
    {
      spin_request = 0U;
      Spin_Start();
      printf("SPIN_START\r\n");
    }

    /* 串口输入 '3'：脱线直线走 1 m */
    if (straight_request)
    {
      straight_request = 0U;
      Straight_Start();
      printf("STRAIGHT_START\r\n");
    }

    if (CAR_DEBUG)
    {
      static uint32_t dbg_last = 0;
      if ((HAL_GetTick() - dbg_last) >= 100U)
      {
        dbg_last = HAL_GetTick();
        printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)TIM1->CCR1, (int)TIM1->CCR2,
               (int)(pid_L.target * 10.0f), (int)(pid_R.target * 10.0f),
               (int)dbg_dL, (int)dbg_dR,
               (int)(dbg_speed_L ), (int)(dbg_speed_R )
               );
      }
    }

    /* 六边形入口事件 */
    {
      static uint16_t hex_last_seq = 0;
      HexDebug hd;
      Hex_GetDebug(&hex, &hd);

      if (hd.evt_seq != hex_last_seq)
      {
        hex_last_seq = hd.evt_seq;
        printf("HEX,%d,%lu,%d,%d,%d,%d,%d\r\n",
               (int)hd.event, (unsigned long)hd.event_dist,
               (int)hd.right, (int)hd.left, (int)hd.center,
               (int)hd.mask, (int)hd.evt_seq);
      }
    }

    /* 六边形状态 */
    {
      static uint32_t hexstat_last = 0;
      if ((HAL_GetTick() - hexstat_last) >= 100U)
      {
        hexstat_last = HAL_GetTick();

        HexDebug hd;
        Hex_GetDebug(&hex, &hd);

        printf("HEXSTAT,%d,%d,%d,%d,%d,%d,%lu,%d\r\n",
               (int)hd.event_cnt, (int)hd.off_r, (int)hd.off_l,
               (int)hd.right, (int)hd.left, (int)hd.center,
               (unsigned long)hd.dist, (int)hd.arm);
      }
    }
		
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | R_DIR_Pin | GPIO_PIN_11 | MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin | MUX_1_Pin | MUX_2_Pin | MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RUN_Pin */
  GPIO_InitStruct.Pin = GPIO_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  /* readme 里原本是 NOPULL + 外部电路给空闲电平。若按键只是把 PB14 短到 GND、
     没有外部上拉，NOPULL 会让引脚悬空、可能读成低电平 → 上电就误发车。
     所以这里改成内部上拉（有外部上拉时也无害）。
     注意：用 CubeMX 重新生成会被改回 NOPULL，需要在 .ioc 里同步。 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin | MUX_2_Pin | MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
