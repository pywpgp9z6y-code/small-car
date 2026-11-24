/**
 * @file    sensor.c
 * @brief   12 路光电扫描（原来内联在 main.c 的 Control_Update 里）
 *
 * 只依赖 multiplexer 驱动，不碰电机/状态机，方便以后脱机喂数据做测试。
 */
#include "sensor.h"
#include "multiplexer.h"

#define SENSOR_CENTER       5.5f   /* 12 路的中线位置：(0+11)/2 */
#define SENSOR_CH_NUM       12
#define SENSOR_MIN_VALID_CNT 2U    /* [SINGLE-FILTER] 少于 2 路视为无效线：
                                      真实赛道不会只有一路压线，单路就是坏点。 */

void Sensor_Scan(LineSample *s)
{
    uint16_t mux = 0U;
    uint16_t mask = 0U;
    float    pos = 0.0f;
    uint8_t  cnt = 0U;

    MUX_get_value(&mux);

    for (int i = 0; i < SENSOR_CH_NUM; i++)
    {
        if (MUX_GET_CHANNEL(mux, i))
        {
            pos += (float)i;
            cnt++;
            mask |= (uint16_t)(1U << i);
        }
    }

    /* [FLOW-DIAG] 保留过滤前的原始值：用来判断 FLOW 漏检是不是
       单路坏点被过滤成 mask=0 造成的。 */
    s->raw_mask = mask;
    s->raw_cnt  = cnt;

    /* [SINGLE-FILTER 2026-11-23] 单路亮视为坏点，直接清成“无线”。
       真实赛道不会只有一路压线；单路贴边（ch0~ch3 / ch8~ch11）会产生
       |error| = 2.5~5.5，足以误武装 rec2_armed 并触发 90° 补转。
       放在 Sensor_Scan 输出层，盲穿/rec2/过弯/六边形看到的是同一份过滤结果。 */
    if (cnt < SENSOR_MIN_VALID_CNT)
    {
        cnt  = 0U;
        mask = 0U;
        pos  = 0.0f;
    }

    s->mask = mask;
    s->cnt  = cnt;
    s->bias = (cnt > 0U) ? (pos / (float)cnt) : 0.0f;//不掉线为前提
    s->error = (cnt > 0U) ? (s->bias - SENSOR_CENTER) : 0.0f;//不掉线为前提，丢线后方向环就失效了？

    /* 六边形入口信号也从过滤后的 mask 生成：
       单路被滤掉后 center/right/left 必须一起失效，不能还留着原始 mux 的位。 */
    /* [V-ONE 2026-11-23] 两路似乎严苛，尝试只看一路：
       center 从“ch5 && ch6”改成“ch5 || ch6”。
       回滚时改回 && 即可。 */
    s->center = (uint8_t)(((mask & (1U << 5)) != 0U) || ((mask & (1U << 6)) != 0U));//center标记5/6路
    s->right  = (uint8_t)((mask & (1U << 11)) != 0U);//right标记12路
    s->left   = (uint8_t)((mask & (1U << 0)) != 0U);//left标记1路
}
