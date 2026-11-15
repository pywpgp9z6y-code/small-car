#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

/* ============================================================
 * BMI270 陀螺仪（z 轴角速度）
 *
 * 注意：全工程只有 Gyro_Task() 可以调用 dodo_BMI270_get_data()。
 * 它内部是 SPI + DMA 忙等，别处再调一次会把正在进行的事务打断。
 * ============================================================ */

/* 初始化 + 静止零偏标定（500 次平均，约 0.5 s）。
   调用期间车必须保持不动，且必须在 TIM2 启动之前调用。
   返回 1 = 可用，0 = 初始化失败（此时 Gyro_IsOk() 为 0）。 */
uint8_t Gyro_Init(void);

/* 200 Hz 采样：读一次、去零偏、一阶低通。只在 1 kHz 控制中断里调用。 */
void Gyro_Task(void);

/* 滤波后的角速度，单位 °/s */
float Gyro_GetZ(void);

/* [YAW 2026-11-15] 连续积分的航向角，单位 °。只供串口观察/标定，不参与控制。
   正方向 = 车体逆时针（与 gyro_z 同号）。 */
float Gyro_GetYaw(void);

/* 把航向角清零（串口发 '0' 或手动调用） */
void Gyro_ResetYaw(void);

/* 1 = 陀螺仪可用 */
uint8_t Gyro_IsOk(void);

#endif /* GYRO_H */
