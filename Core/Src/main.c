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
#include "control.h"   //1 kHz 控制体（方向环/速度环/特殊动作）
#include "gyro.h"      //BMI270 初始化与 200 Hz 采样
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
/* ================= 用户代码：人机 / 串口 / 调试 =================
   控制算法已按层拆分到独立模块，本文件只保留"初始化 + 人机 + 调试"：
     sensor.c   12 路光电扫描 -> LineSample
     gyro.c     BMI270 初始化、零偏标定、200 Hz 采样
     control.c  1 kHz 控制体（方向环 / 速度环 / 五个特殊动作）
     motor.c    PID 与 H 桥输出
     special.c  过弯锁存与六边形入口状态机（纯逻辑，不依赖 HAL）
   拆出去的设计说明（0/40 方案、FF 标定、死区标定、kick 现场微调）
   见 car_config.h 与 control.c 里的注释。 */

/* 调试打印开关：置 1 = 串口输出调试数据（只在调车时用，跑成绩前改回 0） */
#define CAR_DEBUG 1
#define YAW_DEBUG 1   /* [YAW 2026-11-15] 1 = 每 100 ms 打一行 YAW,<0.1°>，跑成绩前可关掉 */

/* ---- 发车键：PB14（GPIO_RUN），按下为低电平 ----
   key_hold = 1 表示这次按下已处理，必须松开才允许下一次。
   初值给 1，是为了处理"上电时按键已经被按着"的情况 —— 必须先松手再按，
   否则一上电就自己跑起来。 */
static uint8_t key_hold = 1;

/* 连续两次读到低电平（间隔 20 ms）才认为是一次有效按下，用来躲按键抖动。 */
static uint8_t RUN_Key_Scan(void)
{
    if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) != GPIO_PIN_RESET)
        return 0;
    HAL_Delay(20);
    return (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

/* ---- 串口单字节命令：'2' 原地转 360°，'3' 脱线直线 1 m ----
   中断里只置请求标志，动作由主循环启动，避免在中断里做状态迁移。 */
static volatile uint8_t spin_request = 0U;
static volatile uint8_t straight_request = 0U;
static volatile uint8_t cw_dump_request = 0U;   /* [CW2] 打印直角弯判据的逐拍记录 */
static volatile uint8_t hexlog_request = 0U;    /* [HEX4] 打印 V 判据的逐拍记录 */
static uint8_t uart_rx_byte = 0U;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (uart_rx_byte == '2')
            spin_request = 1U;
        else if (uart_rx_byte == '3')
            straight_request = 1U;
        else if (uart_rx_byte == 'c')
            cw_dump_request = 1U;     /* [CW2] 打印最近 128 拍的 cnt/mask/err/标志 */
        else if (uart_rx_byte == 'h')
            hexlog_request = 1U;      /* [HEX4] 打印最近 128 拍的 V 判据原始记录 */
        else if (uart_rx_byte == '0')
            Gyro_ResetYaw();      /* [YAW 2026-11-15] 航向清零，便于手转标定 */

        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
    }
}

/* Control_Update() 已移到 control.c；本文件只保留初始化、按键、串口与调试。 */

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
  Gyro_Init();   /* 初始化失败时 Gyro_IsOk()==0：巡线/速度环照常跑，只少了陀螺相关功能 */

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
    printf("HEXSTAT,state,total,vrun,right,left,center,dist,arm,mask\r\n");
    /* [CWDBG 2026-11-22] 直角弯判据事件 */
    printf("CWD,code,cnt,mask,cw_mask,low,wide,pend,dark,ticks,armed,cool,hz,state,err10\r\n");
    /* [LOSS-FEAT 2026-11-22] 丢线窗口特征 */
    printf("LF,err10,slope,sign,edge,center,side,full\r\n");
    /* [HEXCFG 2026-11-22] 上电打印实际编译进去的六边形参数，
       用来确认烧的是不是新代码（HEX,9 不出时先看这一行）。 */
    printf("HEXCFG,MIN=%d,MAX=%d,GIVEUP=%d,DRIVE=%d,SPEED=%d\r\n",
           (int)HEX4_EXIT_MIN_DEG, (int)HEX4_EXIT_MAX_DEG,
           (int)HEX4_EXIT_GIVEUP_DEG, (int)HEX4_DRIVE_MAX_CM,
           (int)HEX_ZONE_SPEED_MAX);
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
        if (!Control_IsRunning())
        {
          Control_Start();          /* 清积分 + 复位 kick，从第一拍开始脉冲 kick */
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
      Control_StartSpin();
      printf("SPIN_START\r\n");
    }

    /* 串口输入 '3'：脱线直线走 1 m */
    if (straight_request)
    {
      straight_request = 0U;
      Control_StartStraight();
      printf("STRAIGHT_START\r\n");
    }

    /* [ISR-SAFE 2026-11-21] 过弯事件：中断里只存，这里打印。
       原来 Corner_Log 在 1 kHz 中断里 printf（一行 2.4 ms > 1 ms 节拍），
       会吞掉控制拍 —— 现在改成主循环来打。 */
    {
      uint8_t  why;
      int8_t   dir;              /* 必须是 int8_t：Corner_PopEvent 的 dir 是 int8_t* */
      uint16_t m, cm;
      int16_t  c_, e_;

      while (Corner_PopEvent(&why, &dir, &m, &cm, &c_, &e_))
        printf("CORN,%d,%d,%d,%d,%d,%d\r\n",
               (int)why, (int)dir, (int)m, (int)cm, (int)c_, (int)e_);
    }

    /* [CW2 2026-11-21] 串口输入 'c'：打印直角弯判据的逐拍记录
       格式：CW,<序号>,<cnt>,<mask>,<err×10>,<标志>
       标志 bit0=左侧全亮 bit1=右侧全亮 bit2=丢线
       用法：跑过一个直角弯（或手推过去）立刻按 'c'，
       数一下"标志出现 1/2 的那一拍"到"标志出现 4 的第一拍"隔了几拍，
       那个数就是 CORNER_FULL_HOLD_TICKS 该取的值。 */
    if (cw_dump_request)
    {
      uint8_t n, i;

      cw_dump_request = 0U;
      n = Cw_LogCount();
      printf("CWLOG,%d,MATCH=%d,ENTER=%d,BLKARM=%d,BLKCOOL=%d\r\n",
             (int)n, (int)Cw_FireCount(), (int)Cw_EnterCount(),
             (int)Cw_BlockArm(), (int)Cw_BlockCool());

      for (i = 0U; i < n; ++i)
      {
        int16_t  c_, e_;
        uint16_t m_;
        uint8_t  f_;

        Cw_GetAt(i, &c_, &m_, &e_, &f_);
        printf("CW,%d,%d,%d,%d,%d\r\n", (int)i, (int)c_, (int)m_, (int)e_, (int)f_);
      }

      Cw_ClearLog();
    }

    if (CAR_DEBUG)
    {
      static uint32_t dbg_last = 0;
      if ((HAL_GetTick() - dbg_last) >= 100U)
      {
        ControlDebug d;
        dbg_last = HAL_GetTick();
        Control_GetDebug(&d);
       printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)TIM1->CCR1, (int)TIM1->CCR2,
               (int)(d.target_L * 10.0f), (int)(d.target_R * 10.0f),
               (int)d.dL, (int)d.dR,
               (int)d.speed_L, (int)d.speed_R);
      }
    }

    /* [YAW 2026-11-15] 航向角观察：10 Hz，单位 0.1°（串口发 '0' 清零） */
    if (YAW_DEBUG)
    {
      static uint32_t yaw_last = 0;
      if ((HAL_GetTick() - yaw_last) >= 100U)
      {
        yaw_last = HAL_GetTick();
        //printf("YAW,%d\r\n", (int)(Gyro_GetYaw() * 10.0f));
      }
    }

    /* [HEX4 2026-11-21] 六边形事件：中断里入队，这里取空为止。
       原来用"快照 + 序号"只打最新一条，主循环被 printf 阻塞时（比如按 'c'
       转储 0.22 s）中间的事件会被压掉；现在不会。 */
    {
      uint8_t  code, r, l, c;
      uint32_t hdist;
      uint16_t hmask, hseq;

      while (Hex_PopEvent(&code, &hdist, &hmask, &r, &l, &c, &hseq))
        printf("HEX,%d,%lu,%d,%d,%d,%d,%d\r\n",
               (int)code, (unsigned long)hdist,
               (int)r, (int)l, (int)c, (int)hmask, (int)hseq);
    }

    /* [LOSTDBG 2026-11-22] 丢线沿：拿丢线前一拍的 error/mask。
       手推 45°/60°/断路口各一次，看 LOST,<err×10>,... 就能定 LOST_BEND_ERR。 */
    {
      int16_t  le;
      uint16_t lm;
      int16_t  lsl, lsr;
      uint8_t  lc;

      if (Control_PopLostLine(&le, &lm, &lsl, &lsr, &lc))
        printf("LOST,%d,%d,%d,%d,%d\r\n",
               (int)le, (int)lm, (int)lsl, (int)lsr, (int)lc);
    }

    /* [LOSS-FEAT 2026-11-22] 丢线沿的 16 拍窗口特征（只观察）：
       err10, slope(centi-lane/拍), 同号%, 贴边拍数, 居中拍数, 最大侧向连续路数, 全亮 */
    {
      int16_t fe, fs;
      uint8_t fg, fedge, fcenter, fside, ffull;

      if (Control_PopLossFeat(&fe, &fs, &fg, &fedge, &fcenter, &fside, &ffull))
        printf("LF,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)fe, (int)fs, (int)fg,
               (int)fedge, (int)fcenter, (int)fside, (int)ffull);
    }

    /* [CWDBG 2026-11-22] 直角弯判据事件：丢线沿 / 候选 / fire / 进 corner / 被挡 */
    {
      CwdDebug cd;

      while (Corner_PopDebug(&cd))
        printf("CWD,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)cd.code, (int)cd.cnt, (int)cd.mask, (int)cd.cw_mask,
               (int)cd.low, (int)cd.wide, (int)cd.pend, (int)cd.dark,
               (int)cd.ticks, (int)cd.armed, (int)cd.cool, (int)cd.hz,
               (int)cd.state, (int)cd.err10);
    }

    /* [HEX4] 串口 'h'：打印最近 128 拍的 V 判据记录
       格式：HV,<i>,<cnt>,<mask>,<err×10>,<标志>
       标志 bit0=v_l bit1=v_r（[V-WIDE 2026-11-22] 已做 1 拍展宽）
            bit2=去抖后成立 bit3=正在 DRIVING
       用途：手推/慢跑一圈六边形，把顶点那几拍的 mask 打出来，
             重新设计"什么样的 mask 才算一个真 V"。 */
    if (hexlog_request)
    {
      uint8_t n, i;

      hexlog_request = 0U;
      n = HexLog_Count();
      printf("HVLOG,%d\r\n", (int)n);

      for (i = 0U; i < n; ++i)
      {
        int16_t  c_, e_;
        uint16_t m_;
        uint8_t  f_;

        HexLog_GetAt(i, &c_, &m_, &e_, &f_);
        printf("HV,%d,%d,%d,%d,%d\r\n", (int)i, (int)c_, (int)m_, (int)e_, (int)f_);
      }

      HexLog_Clear();
    }

		
    /* 六边形状态 */
    {
      static uint32_t hexstat_last = 0;
      if ((HAL_GetTick() - hexstat_last) >= 100U)
      {
        hexstat_last = HAL_GetTick();

        HexDebug hd;
        Control_GetHexDebug(&hd);

        /* [HEX-V 2026-11-15] 末尾追加 mask：12 路原始掩码，用来复盘 V 型判据。
           [HEXDBG 2026-11-22] 前三列改成在线状态：
             state = hx_state（0..4）、total = hx_total 累计角、vrun = V 去抖计数。
           末尾 mask 位置不变，旧解析脚本仍可读。 */
        printf("HEXSTAT,%d,%d,%d,%d,%d,%d,%lu,%d,%d\r\n",
               (int)hd.event_cnt, (int)hd.off_r, (int)hd.off_l,
               (int)hd.right, (int)hd.left, (int)hd.center,
               (unsigned long)hd.dist, (int)hd.arm, (int)hd.mask);
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
