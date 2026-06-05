/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "AD.h"                          // 添加AD头文件
#include "Modbus_Slave.h"               // Modbus从站头文件（USART2）
#include "SPI_SlaveLink.h"              // SPI 从机链路层（SPI1 + EXTI4）

/* 峰值检测相关常量定义（与main.c保持一致） */
#define PEAK_STATE_IDLE         0        // 空闲状态
#define PEAK_STATE_SEARCHING    1        // 峰值搜索状态
#define PEAK_STATE_WAIT_FALL    2        // 等待回落状态
#define PEAK_WINDOW_SIZE        60       // 峰值锁定窗口大小
#define BASELINE_SIZE           100      // 基线估算窗口大小
#define RETURN_THRESHOLD        20       // 回落阈值（ADC单位）
#define DEAD_TIME_INIT          50       // 死区时间初始值
#define MIN_LOCAL_DELTA         6        // 峰值相对于邻近样本的最小差值（与main.c保持一致）

/* 峰值大小分类阈值（与main.c保持一致，用于动态稳定期） */
#define PEAK_SMALL_THRESHOLD    600      // 小雨滴阈值（ADC单位，约480mV）
#define PEAK_MEDIUM_THRESHOLD   1500     // 中等雨滴阈值（ADC单位，约1.2V）

/* 前部峰值检测参数：只分析前部（上升→峰值→下降到0），忽略后部（0→负向极值→0） */
#define PEAK_LOCK_DECAY_COUNT   4        // 连续下降样本数阈值，达到后锁定峰值
#define PEAK_LOCK_BASELINE_DELTA 30      // 回落到基线附近的阈值（ADC单位），用于提前锁定峰值

/* 后部噪声过滤参数：避免后部噪声误判为新峰值 */
#define IDLE_TRIGGER_MARGIN     50       // IDLE状态触发阈值余量（ADC单位），适配小信号检测
#define IDLE_TRIGGER_CONSEC     3        // IDLE状态需要连续N个样本都超过阈值才触发
#define STABLE_PERIOD_COUNT     100      // 稳定期样本数，WAIT_FALL完成后需要值在基线附近保持的样本数（约4.2ms，确保后部震荡完全结束）
#define STABLE_BASELINE_DELTA   50       // 稳定期基线附近的范围（ADC单位）
#define DEAD_TIME_SCALE_FACTOR  2        // 死区时间缩放因子，根据峰值大小动态调整
#define DEAD_TIME_MIN           50       // 最小死区时间
#define DEAD_TIME_MAX           200      // 最大死区时间

/* 差值触发配置 */
#define DIFF_TRIGGER_THRESHOLD      100   // 差分触发阈值（ADC单位，约80mV），适配小信号检测
#define DIFF_TRIGGER_CONSEC         2     // 连续满足差分阈值的样本数
#define DIFF_TRIGGER_COOLDOWN       150   // 触发后冷却样本数，避免重复触发

#define LIVE_SAT_THRESHOLD          4000

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Start_Snapshot_From_Index(uint16_t trig_idx_ch0, uint16_t trig_val_ch0);
static void Evaluate_Diff_Trigger(uint16_t ch0_value, uint16_t idx_ch0);

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t awd_trigger_pending = 0;
static uint16_t prev_ch0_value = 0;
static uint8_t have_prev_ch0 = 0;
static uint8_t diff_hit_counter = 0;
static uint16_t diff_cooldown_counter = 0;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

typedef struct
{
    uint32_t baseline_sum;
    uint16_t baseline_buffer[BASELINE_SIZE];
    uint16_t baseline_value;
    uint16_t dead_time;
    uint16_t search_count;
    uint16_t local_max;
    uint16_t local_max_index;
    uint8_t baseline_index;
    uint8_t baseline_count;
    uint8_t peak_state;
    /* 前部峰值检测：只分析前部，忽略后部 */
    uint16_t prev_value;                 // 上一次的值，用于检测下降
    uint8_t decay_count;                 // 连续下降计数
    uint8_t peak_locked;                 // 峰值锁定标志：1=已锁定，不再更新local_max
    /* 后部噪声过滤：避免后部噪声误判为新峰值 */
    uint8_t stable_count;                // 稳定期计数，WAIT_FALL完成后值在基线附近的样本数
    uint8_t idle_trigger_count;           // IDLE状态触发计数，连续超过阈值的样本数
    uint16_t last_peak_value;            // 上一次的峰值，用于动态调整死区时间
} PeakDetectorContext;

static PeakDetectorContext peak_ctx[2];

static void UpdateBaseline(PeakDetectorContext *ctx, uint16_t value)
{
    ctx->baseline_sum -= ctx->baseline_buffer[ctx->baseline_index];
    ctx->baseline_buffer[ctx->baseline_index] = value;
    ctx->baseline_sum += value;
    ctx->baseline_index = (ctx->baseline_index + 1) % BASELINE_SIZE;

    if (ctx->baseline_count < BASELINE_SIZE)
    {
        ctx->baseline_count++;
        ctx->baseline_value = (uint16_t)(ctx->baseline_sum / ctx->baseline_count);
    }
    else
    {
        ctx->baseline_value = (uint16_t)(ctx->baseline_sum / BASELINE_SIZE);
    }
}

static void Start_Watchdog_Snapshot(void);
static void Update_Live_Diagnostics(uint16_t ch0_value, uint16_t ch1_value)
{
    extern volatile uint16_t dbg_live_pa0_max;
    extern volatile uint16_t dbg_live_pa1_max;
    extern volatile uint16_t dbg_live_pa0_sat_run;
    extern volatile uint16_t dbg_live_pa0_sat_max;

    if (ch0_value > dbg_live_pa0_max)
    {
        dbg_live_pa0_max = ch0_value;
    }
    if (ch1_value > dbg_live_pa1_max)
    {
        dbg_live_pa1_max = ch1_value;
    }

    if (ch0_value >= LIVE_SAT_THRESHOLD)
    {
        if (dbg_live_pa0_sat_run < 0xFFFFU)
        {
            dbg_live_pa0_sat_run++;
        }
        if (dbg_live_pa0_sat_run > dbg_live_pa0_sat_max)
        {
            dbg_live_pa0_sat_max = dbg_live_pa0_sat_run;
        }
    }
    else
    {
        dbg_live_pa0_sat_run = 0;
    }
}

static void Process_ADC_Sample(uint8_t channel, uint16_t value, uint16_t ring_index)
{
    extern volatile uint16_t dynamic_threshold;
    extern volatile uint16_t last_peak_value_from_isr;
    extern volatile uint16_t last_peak_index_from_isr;
    extern volatile uint8_t  last_peak_ready_from_isr;
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t last_peak_window_from_isr[ISR_CAPTURE_WINDOW_SIZE];
    extern volatile uint16_t last_peak_window_len_from_isr;
    extern volatile int32_t last_peak_baseline_from_isr;

    PeakDetectorContext *ctx = &peak_ctx[channel];
    volatile uint16_t *dynamic_thr = &dynamic_threshold;   /* 当前只使用通道0的动态阈值 */

    if (ctx->dead_time > 0)
    {
        ctx->dead_time--;
        if (ctx->peak_state == PEAK_STATE_IDLE)
        {
            UpdateBaseline(ctx, value);
        }
        return;
    }

    if (ctx->peak_state == PEAK_STATE_IDLE)
    {
        UpdateBaseline(ctx, value);
    }

    switch (ctx->peak_state)
    {
        case PEAK_STATE_IDLE:
            /* 后部噪声过滤：提高IDLE状态的触发条件，避免后部小波动触发新的峰值检测 */
            
            /* 稳定期检查：如果刚从WAIT_FALL状态出来，需要值在基线附近保持一段时间 */
            if (ctx->stable_count > 0)
            {
                /* 在稳定期内，检查值是否在基线附近 */
                if (value <= (ctx->baseline_value + STABLE_BASELINE_DELTA) &&
                    value >= (ctx->baseline_value - STABLE_BASELINE_DELTA))
                {
                    /* 值在基线附近，稳定期计数递减 */
                    ctx->stable_count--;
                }
                else
                {
                    /* 值不在基线附近，重置稳定期计数 */
                    ctx->stable_count = STABLE_PERIOD_COUNT;
                }
                
                /* 稳定期未结束，不允许新的峰值检测 */
                if (ctx->stable_count > 0)
                {
                    break;
                }
            }
            
            /* 稳定期已结束，检查是否触发新的峰值检测 */
            /* 提高触发条件：值必须明显超过阈值（阈值 + 余量） */
            if (value > (*dynamic_thr + IDLE_TRIGGER_MARGIN))
            {
                /* 值超过阈值+余量，增加触发计数 */
                ctx->idle_trigger_count++;
                
                /* 需要连续N个样本都超过阈值才触发，避免单次噪声触发 */
                if (ctx->idle_trigger_count >= IDLE_TRIGGER_CONSEC)
                {
                    /* 触发新的峰值检测 */
                    ctx->peak_state = PEAK_STATE_SEARCHING;
                    ctx->search_count = 0;
                    ctx->local_max = value;
                    ctx->local_max_index = ring_index;
                    /* 初始化前部峰值检测变量 */
                    ctx->prev_value = value;
                    ctx->decay_count = 0;
                    ctx->peak_locked = 0;
                    /* 重置触发计数 */
                    ctx->idle_trigger_count = 0;
                }
            }
            else
            {
                /* 值未超过阈值+余量，重置触发计数 */
                ctx->idle_trigger_count = 0;
            }
            break;

        case PEAK_STATE_SEARCHING:
            /* 前部峰值检测：只在前部（上升→峰值→下降到0）搜索峰值，忽略后部 */
            /* 关键：限制搜索时间窗口，确保只捕获前部峰值，不捕获后部震荡 */
            
            /* 检查是否已回落到基线附近（前部结束标志） */
            if (value <= (ctx->baseline_value + PEAK_LOCK_BASELINE_DELTA))
            {
                /* 已回落到基线，前部结束，立即锁定峰值并进入WAIT_FALL状态 */
                ctx->peak_locked = 1;
                ctx->peak_state = PEAK_STATE_WAIT_FALL;
                break;
            }
            
            /* 如果峰值尚未锁定，继续搜索 */
            if (!ctx->peak_locked)
            {
                /* 检测下降趋势 */
                if (value < ctx->prev_value)
                {
                    /* 值在下降，增加下降计数 */
                    ctx->decay_count++;
                    
                    /* 如果连续下降达到阈值，说明峰值已过，锁定峰值 */
                    if (ctx->decay_count >= PEAK_LOCK_DECAY_COUNT)
                    {
                        ctx->peak_locked = 1;
                    }
                }
                else
                {
                    /* 值未下降，重置下降计数 */
                    ctx->decay_count = 0;
                    
                    /* 如果值大于当前峰值，更新峰值（只在前部上升阶段） */
                    if (value > ctx->local_max)
                    {
                        ctx->local_max = value;
                        ctx->local_max_index = ring_index;
                    }
                }
            }
            /* 如果峰值已锁定，不再更新local_max，即使后续有更大的值（可能是后部噪声） */
            
            ctx->prev_value = value;  // 更新上一次的值
            ctx->search_count++;
            
            /* 如果搜索窗口用完，强制锁定峰值并进入WAIT_FALL状态（防止搜索时间过长捕获后部震荡） */
            if (ctx->search_count >= PEAK_WINDOW_SIZE)
            {
                ctx->peak_locked = 1;  // 强制锁定峰值
                ctx->peak_state = PEAK_STATE_WAIT_FALL;
            }
            break;

        case PEAK_STATE_WAIT_FALL:
            if (value < (ctx->baseline_value + RETURN_THRESHOLD))
            {
                /* 脉冲完成：记录本次完整脉冲的峰值（仅通道0/PA0），供主循环显示使用 */
                /* 验证峰值是否明显大于基线，过滤后部噪声和ADC数字噪声（后部噪声通常不会明显大于基线） */
                /* 降低阈值到80，适配420-540mV小雨滴信号（约320-410mV相对基线） */
                if (channel == 0 && ctx->local_max > (ctx->baseline_value + 80))
                {
                    uint16_t win_start = (ctx->local_max_index >= ISR_CAPTURE_PRE_SAMPLES) ?
                        (uint16_t)(ctx->local_max_index - ISR_CAPTURE_PRE_SAMPLES) :
                        (uint16_t)(RING_BUFFER_SIZE + ctx->local_max_index - ISR_CAPTURE_PRE_SAMPLES);
                    uint16_t post_samples = (ring_index >= ctx->local_max_index) ?
                        (uint16_t)(ring_index - ctx->local_max_index) :
                        (uint16_t)(RING_BUFFER_SIZE - ctx->local_max_index + ring_index);
                    uint16_t window_len;
                    int32_t baseline_start = (int32_t)win_start - (int32_t)ISR_CAPTURE_BASELINE_SAMPLES;
                    uint32_t baseline_sum = 0;

                    if (post_samples > ISR_CAPTURE_POST_SAMPLES)
                    {
                        post_samples = ISR_CAPTURE_POST_SAMPLES;
                    }
                    window_len = (uint16_t)(ISR_CAPTURE_PRE_SAMPLES + 1U + post_samples);
                    while (baseline_start < 0)
                    {
                        baseline_start += RING_BUFFER_SIZE;
                    }
                    for (uint16_t k = 0; k < ISR_CAPTURE_BASELINE_SAMPLES; k++)
                    {
                        baseline_sum += adc_ring_buffer_ch0[(uint16_t)((baseline_start + k) % RING_BUFFER_SIZE)];
                    }
                    last_peak_baseline_from_isr = (int32_t)(baseline_sum / ISR_CAPTURE_BASELINE_SAMPLES);
                    for (uint16_t k = 0; k < window_len; k++)
                    {
                        last_peak_window_from_isr[k] = adc_ring_buffer_ch0[(uint16_t)((win_start + k) % RING_BUFFER_SIZE)];
                    }
                    last_peak_window_len_from_isr = window_len;

                    last_peak_value_from_isr = ctx->local_max;
                    last_peak_index_from_isr = ctx->local_max_index;
                    last_peak_ready_from_isr = 1;
                }

                /* 动态调整死区时间：根据峰值大小调整，大峰值后需要更长的死区时间 */
                uint16_t dynamic_dead_time = DEAD_TIME_INIT;
                /* 根据当前峰值大小计算死区时间：峰值越大，死区时间越长 */
                /* 峰值每增加1000个ADC单位（约0.8V），死区时间增加50个样本 */
                if (ctx->local_max > 1000)
                {
                    uint16_t extra_dead_time = ((ctx->local_max - 1000) / 1000) * 50;
                    dynamic_dead_time = DEAD_TIME_INIT + extra_dead_time;
                    if (dynamic_dead_time > DEAD_TIME_MAX)
                        dynamic_dead_time = DEAD_TIME_MAX;
                    if (dynamic_dead_time < DEAD_TIME_MIN)
                        dynamic_dead_time = DEAD_TIME_MIN;
                }
                
                /* 保存当前峰值，用于调试或后续分析 */
                ctx->last_peak_value = ctx->local_max;

                ctx->peak_state = PEAK_STATE_IDLE;
                ctx->search_count = 0;
                ctx->local_max = 0;
                ctx->dead_time = dynamic_dead_time;
                /* 重置前部峰值检测变量 */
                ctx->prev_value = 0;
                ctx->decay_count = 0;
                ctx->peak_locked = 0;
                /* 进入稳定期：根据峰值大小动态设置稳定期，小雨滴后使用更短的稳定期以支持快速检测 */
                if (ctx->last_peak_value < PEAK_SMALL_THRESHOLD)
                {
                    /* 小雨滴：50个样本（约2.1ms） */
                    ctx->stable_count = 50;
                }
                else if (ctx->last_peak_value < PEAK_MEDIUM_THRESHOLD)
                {
                    /* 中等雨滴：75个样本（约3.15ms） */
                    ctx->stable_count = 75;
                }
                else
                {
                    /* 大雨滴：100个样本（约4.2ms，原值） */
                    ctx->stable_count = STABLE_PERIOD_COUNT;
                }
                /* 重置触发计数 */
                ctx->idle_trigger_count = 0;
            }
            break;
    }

}

static void Start_Snapshot_From_Index(uint16_t trig_idx_ch0, uint16_t trig_val_ch0)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
    extern volatile uint16_t snapshot_buffer_high[SNAPSHOT_SIZE];
    extern volatile uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];
    extern volatile uint16_t snapshot_write_index;
    extern volatile uint8_t snapshot_collecting;
    extern volatile uint8_t snapshot_ready;

    if (snapshot_collecting || snapshot_ready)
    {
        return;
    }

    int16_t start_high = (int16_t)trig_idx_ch0 - SNAPSHOT_PRE_SAMPLES;
    if (start_high < 0) start_high += RING_BUFFER_SIZE;

    for (uint16_t k = 0; k < SNAPSHOT_PRE_SAMPLES; k++)
    {
        uint16_t idx = (uint16_t)((start_high + k) % RING_BUFFER_SIZE);
        snapshot_buffer_high[k] = adc_ring_buffer_ch0[idx];
        snapshot_buffer_low[k] = adc_ring_buffer_ch1[idx];
    }

    /* 将触发样本放在索引 SNAPSHOT_PRE_SAMPLES */
    snapshot_buffer_high[SNAPSHOT_PRE_SAMPLES] = trig_val_ch0;
    snapshot_buffer_low[SNAPSHOT_PRE_SAMPLES] = adc_ring_buffer_ch1[trig_idx_ch0];

    /* 保存触发时的环形缓冲区索引，用于后续计算PA1对应位置 */
    extern volatile uint16_t snapshot_trigger_index_ch0;
    snapshot_trigger_index_ch0 = trig_idx_ch0;

    snapshot_write_index = SNAPSHOT_PRE_SAMPLES + 1;
    snapshot_collecting = 1;
}

static void Evaluate_Diff_Trigger(uint16_t ch0_value, uint16_t idx_ch0)
{
    extern volatile uint8_t snapshot_collecting;
    extern volatile uint8_t snapshot_ready;

    if (snapshot_collecting || snapshot_ready)
    {
        /* 仍然需要更新前一个采样值以便下一次触发 */
        prev_ch0_value = ch0_value;
        have_prev_ch0 = 1;
        if (diff_cooldown_counter > 0)
            diff_cooldown_counter--;
        return;
    }

    uint8_t trigger_now = 0;

    if (awd_trigger_pending)
    {
        trigger_now = 1;
        awd_trigger_pending = 0;
    }
    else if (have_prev_ch0)
    {
        uint16_t diff = (ch0_value > prev_ch0_value) ? (ch0_value - prev_ch0_value) : (prev_ch0_value - ch0_value);
        if (diff >= DIFF_TRIGGER_THRESHOLD)
        {
            if (diff_hit_counter < 0xFF)
                diff_hit_counter++;
        }
        else if (diff_hit_counter > 0)
        {
            diff_hit_counter--;
        }

        if (diff_hit_counter >= DIFF_TRIGGER_CONSEC && diff_cooldown_counter == 0)
        {
            trigger_now = 1;
            diff_hit_counter = 0;
            diff_cooldown_counter = DIFF_TRIGGER_COOLDOWN;
        }
    }

    prev_ch0_value = ch0_value;
    have_prev_ch0 = 1;
    if (diff_cooldown_counter > 0)
        diff_cooldown_counter--;

    if (trigger_now)
    {
        Start_Snapshot_From_Index(idx_ch0, ch0_value);
    }
}

/**
  * @brief  This function handles DMA1 Channel1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t ring_write_index_ch0;
    extern volatile uint8_t snapshot_collecting;
    extern volatile uint16_t snapshot_buffer_high[SNAPSHOT_SIZE];
    extern volatile uint16_t snapshot_write_index;
    extern volatile uint8_t snapshot_ready;
    extern volatile uint16_t ADC_Visualize_Buffer[500];
    
    /* 半传输：处理前半缓冲（0~99），双通道扫描模式下包含50个PA0和50个PA1（交替存储） */
    /* AD_Value[i*2]是PA0数据，AD_Value[i*2+1]是PA1数据（i=0-49） */
    if (DMA_GetITStatus(DMA1_IT_HT1))
    {
        extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
        extern volatile uint16_t ring_write_index_ch1;
        
        uint8_t i;
        for (i = 0; i < 50; i++)  // 每次处理一对PA0和PA1数据
        {
            uint16_t ch0_value = AD_Value[i * 2];      // PA0数据
            uint16_t ch1_value = AD_Value[i * 2 + 1];  // PA1数据
            uint16_t current_index = ring_write_index_ch0;
            uint16_t next_index = (uint16_t)((current_index + 1U) % RING_BUFFER_SIZE);

            adc_ring_buffer_ch0[current_index] = ch0_value;
            adc_ring_buffer_ch1[current_index] = ch1_value;
            ring_write_index_ch0 = next_index;
            ring_write_index_ch1 = next_index;
            dbg_pair_pa0_last = ch0_value;
            dbg_pair_pa1_last = ch1_value;
            dbg_pair_write_idx = current_index;
            dbg_pair_write_count++;
            Update_Live_Diagnostics(ch0_value, ch1_value);
            Process_ADC_Sample(0, ch0_value, current_index);
            sampling_tick_counter++;

            Evaluate_Diff_Trigger(ch0_value, current_index);

            if (snapshot_collecting && snapshot_write_index < SNAPSHOT_SIZE)
            {
                snapshot_buffer_high[snapshot_write_index] = ch0_value;
                snapshot_buffer_low[snapshot_write_index] = ch1_value;
                snapshot_write_index++;
                if (snapshot_write_index >= SNAPSHOT_SIZE)
                {
                    snapshot_ready = 1;
                    snapshot_collecting = 0;
                }
            }
        }
        DMA_ClearITPendingBit(DMA1_IT_HT1);
    }

    /* 传输完成：处理后半缓冲（100~199），双通道扫描模式下包含50个PA0和50个PA1（交替存储） */
    /* AD_Value[i*2]是PA0数据，AD_Value[i*2+1]是PA1数据（i=50-99） */
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
        extern volatile uint16_t ring_write_index_ch1;
        
        uint8_t i;
        for (i = 50; i < 100; i++)  // 每次处理一对PA0和PA1数据
        {
            uint16_t ch0_value = AD_Value[i * 2];      // PA0数据
            uint16_t ch1_value = AD_Value[i * 2 + 1];  // PA1数据
            uint16_t current_index = ring_write_index_ch0;
            uint16_t next_index = (uint16_t)((current_index + 1U) % RING_BUFFER_SIZE);

            adc_ring_buffer_ch0[current_index] = ch0_value;
            adc_ring_buffer_ch1[current_index] = ch1_value;
            ring_write_index_ch0 = next_index;
            ring_write_index_ch1 = next_index;
            dbg_pair_pa0_last = ch0_value;
            dbg_pair_pa1_last = ch1_value;
            dbg_pair_write_idx = current_index;
            dbg_pair_write_count++;
            Update_Live_Diagnostics(ch0_value, ch1_value);
            Process_ADC_Sample(0, ch0_value, current_index);
            sampling_tick_counter++;

            Evaluate_Diff_Trigger(ch0_value, current_index);

            if (snapshot_collecting && snapshot_write_index < SNAPSHOT_SIZE)
            {
                snapshot_buffer_high[snapshot_write_index] = ch0_value;
                snapshot_buffer_low[snapshot_write_index] = ch1_value;
                snapshot_write_index++;
                if (snapshot_write_index >= SNAPSHOT_SIZE)
                {
                    snapshot_ready = 1;
                    snapshot_collecting = 0;
                }
            }
        }
        /* 更新Keil Array Visualization可视化数组：将最新的500个通道0数据复制到可视化缓冲区 */
        {
            int16_t start_idx = (int16_t)ring_write_index_ch0 - 500;
            uint16_t i;
            if (start_idx < 0) start_idx += RING_BUFFER_SIZE;
            for (i = 0; i < 500; i++)
            {
                ADC_Visualize_Buffer[i] = adc_ring_buffer_ch0[(start_idx + i) % RING_BUFFER_SIZE];
            }
        }
        
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

/**
  * @brief  ADC1与ADC2的中断（用于模拟看门狗触发）
  */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
    {
        /* 通道0硬件触发时标记一次触发，由DMA中断负责实际截取 */
        awd_trigger_pending = 1;
        extern volatile uint32_t watchdog_trigger_count;
        watchdog_trigger_count++;
        ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    }
}

/**
  * @brief  SPI1中断处理函数（RXNE/TXE，用于SPI从机链路层）
  */
void SPI1_IRQHandler(void)
{
    SPI_SlaveLink_SPI1_IRQHandler();
}

/**
  * @brief  EXTI4中断处理函数（NSS 边沿，用于SPI从机事务边界）
  */
void EXTI4_IRQHandler(void)
{
    SPI_SlaveLink_EXTI4_IRQHandler();
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
