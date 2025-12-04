// 包含必要的头文件
#include "stm32f10x.h"                  // STM32F10x微控制器标准外设库头文件
#include "Delay.h"                       // 延时函数头文件
#include "OLED.h"                        // OLED显示屏驱动头文件
#include "AD.h"                          // ADC模数转换器驱动头文件
#include "stm32f10x_usart.h"             // 串口通信头文件
#include "stm32f10x_gpio.h"              // GPIO口操作头文件
#include "stm32f10x_rcc.h"               // 时钟控制头文件

// ========== 系统参数定义 ==========
#define THRESHOLD 620                     // 初始阈值（ADC单位，500mV = 0.5V / 3.3V * 4095 ≈ 620）

/* 增益配置：CH0=高增益、CH1=低增益（默认≈15倍 vs 1倍，可按需调整） */
#define HIGH_GAIN_FACTOR         15.0f
#define LOW_GAIN_FACTOR          1.0f
#define HIGH_GAIN_SAT_THRESHOLD  4000     // 高增益ADC达到该值视为饱和（接近3.3V）
#define ADC_FULL_SCALE           4095.0f
#define ADC_REF_VOLTAGE          3.3f
#define ADC_SAMPLE_INTERVAL_US   42.0f    // 12MHz ADC、239.5周期采样、双通道扫描 ≈ 42us/样本

/* 自适应阈值相关 */
#define NOISE_WINDOW            200      // 噪声统计窗口长度（从环形缓冲末端向前取）
#define MIN_THRESHOLD           620       // 阈值下限，防止过低（500mV）
#define MAX_THRESHOLD           3000     // 阈值上限，防止过高
#define MAD_GAIN                3        // 平均绝对偏差放大倍数
#define HYSTERESIS_MARGIN       15       // 阈值滞回，降低抖动

/* 事件与抗干扰判定 */
#define MIN_PEAK_DELTA_OVER_THR 12       // 峰值需高出阈值的最小余量（约10mV）
#define MIN_LOCAL_DELTA         8        // 峰值相对于邻近样本的最小差值
#define MIN_RISE_SAMPLES        3        // 峰前上升最少采样点数
#define MIN_DECAY_SAMPLES       3        // 峰后下降最少采样点数
#define SHAPE_WINDOW_PRE        12       // 峰前用于形状判定的样本数
#define SHAPE_WINDOW_POST       24       // 峰后用于形状判定的样本数
#define BASELINE_SAMPLE_COUNT   80       // 基线估算样本数
#define LOCAL_REFINEMENT_RADIUS 6        // 峰值局部搜索半径
#define MIN_PEAK_AMPLITUDE      1500     // 最小峰值幅度（ADC单位，约1.2V），过滤噪声

/* 显示门限：小于该幅度的脉冲不刷新OLED（仅在主循环中使用） */
#define DISPLAY_MIN_AMPLITUDE   1200     // 显示下限约 1.0V，可按需要微调

/* 峰值检测状态机定义 */
#define PEAK_STATE_IDLE         0        // 空闲状态
#define PEAK_STATE_SEARCHING    1        // 峰值搜索状态
#define PEAK_STATE_WAIT_FALL    2        // 等待回落状态

/* 峰值检测参数 */
#define PEAK_WINDOW_SIZE        60       // 峰值锁定窗口大小（50~80推荐）
#define BASELINE_SIZE           100      // 基线估算窗口大小
#define RETURN_THRESHOLD        20       // 回落阈值（ADC单位）
#define DEAD_TIME_INIT          50       // 死区时间初始值（约5ms，对应50个采样点）
#define PEAK_SEARCH_HALFSPAN    80       // 在触发点附近±半窗口内搜索峰值
#define PEAK_SEARCH_CENTER      SNAPSHOT_PRE_SAMPLES  // 触发点索引（预触发长度）
#define TAIL_SETTLE_COUNT       5        // 识别回落到基线所需的连续样本数

/* 前/后部分割参数（仅分析前部“上升→峰值→回落到基线”） */
#define FRONT_START_DELTA       MIN_LOCAL_DELTA   // 认为“离开基线”的门限
#define FRONT_END_DELTA         MIN_LOCAL_DELTA   // 认为“回到基线”的门限
#define FRONT_END_SETTLE_COUNT  TAIL_SETTLE_COUNT// 回落连续样本计数

/* 严格前部处理：每个脉冲信号只取前部（约2ms），后部数据完全不分析，避免后部抖动导致跳变 */
#define FRONT_ANALYSIS_TIME_MS  2.0f              // 前部分析时间窗口（毫秒）
#define FRONT_ANALYSIS_SAMPLES   ((uint16_t)(FRONT_ANALYSIS_TIME_MS * 1000.0f / ADC_SAMPLE_INTERVAL_US))  // 前部分析采样点数（约48点）

/* 事件级死区：用于在快照层面避免同一滴的重复触发（单位：主循环次数，10ms/次） */
#define EVENT_DEADTIME_LOOPS    50       // 约 500ms，可按需要标定，避免同一滴的拖尾触发新的快照

/* 雨量学参数（需根据传感器标定修正） */
#define MM_PER_DROP             0.02f    // 每个有效雨滴折合降雨量（mm/滴）——占位标定值

/* 强度统计（mm/h）——用近60秒的滴数计算 */
#define SECONDS_WINDOW          60       // 统计窗口大小（秒）

// ========== 全局变量定义 ==========
// 通道0（第一路）变量
uint16_t current_peak_raw = 0;           // 当前峰值对应通道的原始ADC值
uint16_t current_peak = 0;               // 当前检测到的等效高增益峰值
float current_voltage = 0.0;             // 当前峰值对应的电压值(单位:伏特V)（通道0）
float voltage_sum = 0.0;                 // 累积电压总和(单位:伏特V)

/* 峰值保持机制：避免小噪声覆盖大峰值 */
static uint16_t last_valid_peak = 0;    // 上一次有效的峰值（用于峰值保持）
static uint32_t peak_hold_counter = 0;   // 峰值保持计数器
#define PEAK_HOLD_TIME_MS    200         // 峰值保持时间（毫秒），200ms内只显示更大的峰值，确保快速连续雨滴仍能检测
#define PEAK_HOLD_MIN_DELTA  200         // 新峰值必须比旧峰值大至少200个ADC单位才更新（约160mV）
#define PEAK_HOLD_MIN_RATIO  0.7f        // 新峰值必须大于旧峰值的70%才更新（仅在保持时间内生效，防止后部震荡误判）

/* 快速跳变过滤机制：过滤快速跳变（正常值→小值，一直显示小值），同时保留真实小雨滴 */
static uint16_t suspicious_peak = 0;              // 可疑峰值（明显小于旧峰值的新峰值）
static uint32_t suspicious_peak_counter = 0;      // 可疑峰值延迟计数器
#define RAPID_JUMP_FILTER_TIME_MS  50            // 快速跳变过滤时间（毫秒），如果小值在50ms内被更大的值覆盖，说明是跳变
#define RAPID_JUMP_RATIO  0.5f                    // 明显小于旧峰值的阈值（50%），小于此值认为是可疑峰值

/* 快速跳变时间过滤：如果新峰值明显小于旧峰值，且距离上次更新时间很短，直接忽略 */
static uint32_t last_update_counter = 0;          // 最近一次峰值更新的主循环计数
static uint32_t main_loop_counter = 0;            // 主循环计数器（每10ms递增）
#define RAPID_JUMP_TIME_THRESHOLD  3              // 快速跳变时间阈值（主循环次数，即30ms），30ms内的小峰值直接忽略

/* 中断层在线峰值检测结果（仅PA0）：每次完整脉冲结束时更新 */
volatile uint16_t last_peak_value_from_isr = 0;   // 最近一次完整脉冲的峰值ADC码
volatile uint16_t last_peak_index_from_isr = 0;   // 对应环形缓冲索引（调试用）
volatile uint8_t  last_peak_ready_from_isr = 0;   // 标志位：1表示有新峰值待处理

uint32_t display_counter = 0;            // 显示更新计数器，用于定时更新显示
uint32_t system_check_counter = 0;       // 系统状态检查计数器，用于定时检查系统状态
uint8_t system_normal = 1;               // 系统状态标志，1表示正常，0表示异常
uint32_t last_sampling_tick = 0;         // 上一次采样推进计数

/* 快照事件级死区计数器（单位：主循环次数） */
static uint16_t event_deadtime_loops = 0;

// ========== 函数声明 ==========
void Update_Display(void);               // 显示更新函数声明
void Check_System_Status(void);          // 系统状态检查函数声明
static void Process_Snapshot_IfReady(void);  // 处理触发快照（100+900）
static void Update_Adaptive_Threshold(void); // 计算噪声并自适应阈值
static uint8_t Validate_And_Count_Event(uint16_t *buf, uint16_t len, uint16_t peak_index, uint16_t peak_value, uint16_t threshold, uint16_t start_index, uint16_t end_index); // 验证并计数事件
static void Push_Second_Count(uint16_t drops_in_second); // 推送秒计数到窗口
static float Compute_Intensity_MMH(void); // 计算降雨强度（mm/h）
static int32_t Compute_Baseline(uint16_t *buf, uint16_t len);
static void Find_Peak_In_Buffer(uint16_t *buf, uint16_t len, int32_t baseline,
                                uint16_t *peak_index, uint16_t *peak_value,
                                uint16_t search_start, uint16_t search_end);
static uint16_t Scale_Value_With_Gain(uint16_t value, float gain);

/* 触发与统计变量（当前仅使用PA0单通道） */
volatile extern uint8_t snapshot_ready;  // 快照就绪标志（外部定义）
volatile extern uint16_t snapshot_buffer_high[SNAPSHOT_SIZE]; // 高增益快照缓冲区
volatile extern uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];  // 低增益快照缓冲区
volatile extern uint8_t snapshot_collecting; // 快照采集状态标志（外部定义）
volatile extern uint16_t snapshot_peak_value; // 快照峰值（外部定义）
volatile extern uint16_t snapshot_peak_index; // 快照峰值索引（外部定义）

/* 导出缓存与标志（保存最近一次完整快照，用于命令导出） */
static volatile uint16_t export_buffer[SNAPSHOT_SIZE]; // 导出数据缓冲区（PA0波形）
volatile uint8_t export_ready = 0;                // 导出就绪标志

/* 自适应阈值运行变量（当前仅针对通道0/PA0） */
volatile uint16_t dynamic_threshold = THRESHOLD;  // 动态阈值变量（通道0）- 在中断中使用，需volatile

/* 滴数与累计雨量 - 在中断中使用，需volatile或移除static */
volatile uint32_t drop_count = 0;          // 雨滴计数
volatile float total_rain_mm = 0.0f;       // 累计降雨量（毫米）

/* 近60秒滴数窗口 - 在中断中使用 */
volatile uint16_t drops_per_second[SECONDS_WINDOW] = {0}; // 每秒雨滴数数组
volatile uint8_t sec_index = 0;            // 当前秒索引
static uint16_t second_loop_counter = 0; // 10ms循环累加到100为1秒

/* OLED显示缓存 */
static float current_intensity_mmh = 0.0f; // 当前降雨强度（毫米/小时）
static char last_gain_used = 'H';          // 最近一次使用的增益通道
volatile uint32_t watchdog_trigger_count = 0; // 模拟看门狗触发次数
volatile uint32_t snapshot_valid_count = 0;   // 验证通过次数

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   程序入口点，完成系统初始化和主循环
  */
int main(void)
{
    // ========== 系统初始化 ==========
    Delay_Init();                        // 初始化延时函数，配置SysTick定时器
    OLED_Init();                         // 初始化OLED显示屏，配置I2C通信和显示参数
    AD_Init();                           // 初始化ADC和DMA，配置连续采样模式
    AD_SetThreshold(THRESHOLD);          // 设置模拟看门狗阈值
    
    // ========== 显示静态内容 ==========
	OLED_ShowString(1, 1, "Peak:");      // 合并通道峰值
	OLED_ShowString(1, 11, "G:");        // 当前增益标记
	OLED_ShowString(2, 1, "Volt:");      // 合并通道峰值电压
	OLED_ShowString(3, 1, "Rain:");      // 累计雨量
	OLED_ShowString(4, 1, "Sum:");       // 累计电压标签
    
    // ========== 主循环 ==========
    while (1)                            // 程序主要逻辑
    {
		main_loop_counter++;             // 主循环计数器递增（每10ms递增一次）
		/* 中断层在线峰值：作为备用路径，确保即使快照处理失败也能更新显示 */
		/* 注意：中断层已添加验证（明显大于基线），但仍可能捕获后部噪声 */
		/* 优先使用快照处理的结果（严格前部窗口），中断层作为备用 */
		if (last_peak_ready_from_isr)
		{
			/* 在线状态机给出的完整脉冲峰值（PA0） */
			uint16_t peak_candidate = last_peak_value_from_isr;
			last_peak_ready_from_isr = 0;   // 先清标志，避免重复处理

			/* 仅当幅度超过显示门限时才刷新OLED，避免0.5~0.8V等小波动干扰 */
			if (peak_candidate >= DISPLAY_MIN_AMPLITUDE)
			{
				/* 峰值保持机制：在保持时间内，只有更大的峰值才能更新显示 */
				/* 关键改进：仅在保持时间内忽略明显小于旧峰值的新峰值（小于70%），防止后部震荡误判 */
				uint8_t should_update = 0;
				
				if (last_valid_peak == 0)
				{
					/* 第一次有效峰值，直接更新 */
					should_update = 1;
					peak_hold_counter = PEAK_HOLD_TIME_MS / 10;  // 转换为10ms单位
				}
				else if (peak_hold_counter > 0)
				{
					/* 保持时间内：检查新峰值是否明显大于旧峰值 */
					if (peak_candidate > (last_valid_peak + PEAK_HOLD_MIN_DELTA))
					{
						/* 新峰值明显大于旧峰值，更新 */
						should_update = 1;
						peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
					}
					else if (peak_candidate >= (uint16_t)(last_valid_peak * PEAK_HOLD_MIN_RATIO))
					{
						/* 新峰值大于旧峰值的70%，允许更新（支持快速连续大雨滴） */
						should_update = 1;
						peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
					}
					/* 否则（新峰值小于旧峰值的70%），直接忽略，防止后部震荡误判 */
				}
				else
				{
					/* 保持时间已过，检查是否是可疑峰值（明显小于旧峰值） */
					if (peak_candidate < (uint16_t)(last_valid_peak * RAPID_JUMP_RATIO))
					{
						/* 新峰值明显小于旧峰值（小于50%），检查距离上次更新时间 */
						uint32_t time_since_last_update = main_loop_counter - last_update_counter;
						
						if (time_since_last_update <= RAPID_JUMP_TIME_THRESHOLD)
						{
							/* 距离上次更新时间很短（30ms内），很可能是快速跳变（后部震荡），直接忽略，不进行延迟验证 */
							/* 不更新，不设置可疑峰值 */
						}
						else
						{
							/* 距离上次更新时间较长（超过30ms），可能是真实小雨滴，进行延迟验证 */
							suspicious_peak = peak_candidate;
							suspicious_peak_counter = RAPID_JUMP_FILTER_TIME_MS / 10;  // 转换为10ms单位
							/* 不立即更新，等待延迟验证 */
						}
					}
					else
					{
						/* 新峰值不是明显小于旧峰值，直接更新（包括正常的小雨滴） */
						should_update = 1;
						peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
						/* 如果有可疑峰值，清除它（因为出现了更大的峰值，说明之前的是跳变） */
						suspicious_peak = 0;
						suspicious_peak_counter = 0;
					}
				}
				
				if (should_update)
				{
					current_peak_raw = peak_candidate;
					current_peak = peak_candidate;
					current_voltage = (float)current_peak / ADC_FULL_SCALE * ADC_REF_VOLTAGE;
					voltage_sum += current_voltage;
					last_gain_used = 'H';    // 当前仅高增益通道
					last_valid_peak = peak_candidate;  // 更新保持的峰值
					last_update_counter = main_loop_counter;  // 更新最近一次峰值更新的主循环计数
					/* 如果有可疑峰值，清除它（因为出现了更大的峰值，说明之前的是跳变） */
					suspicious_peak = 0;
					suspicious_peak_counter = 0;
				}
			}
			/* 若未达到显示门限，则认为是噪声/微小波动，不刷新显示 */
		}
		
		/* 峰值保持计数器递减 */
		if (peak_hold_counter > 0)
		{
			peak_hold_counter--;
		}
		
		/* 处理可疑峰值延迟验证：如果延迟时间到，且没有更大的峰值出现，说明是真实小雨滴，更新显示 */
		if (suspicious_peak_counter > 0)
		{
			suspicious_peak_counter--;
			if (suspicious_peak_counter == 0 && suspicious_peak > 0)
			{
				/* 延迟时间到，没有更大的峰值出现，说明是真实小雨滴，更新显示 */
				current_peak_raw = suspicious_peak;
				current_peak = suspicious_peak;
				current_voltage = (float)current_peak / ADC_FULL_SCALE * ADC_REF_VOLTAGE;
				voltage_sum += current_voltage;
				last_gain_used = 'H';
				last_valid_peak = suspicious_peak;
				last_update_counter = main_loop_counter;  // 更新最近一次峰值更新的主循环计数
				peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
				suspicious_peak = 0;  // 清除可疑峰值
			}
		}

		/* 处理快照数据（如果就绪）：用于精确的事件验证和计数 */
		Process_Snapshot_IfReady();

		/* 自适应阈值（基于最近噪声） */
		Update_Adaptive_Threshold();
        
        // 每200ms更新一次显示(20次 × 10ms = 200ms)
        if (display_counter >= 20)       // 检查显示计数器是否达到20
        {
            Update_Display();            // 调用显示更新函数
            display_counter = 0;         // 重置显示计数器
        }
        
        // 每1秒检查一次系统状态(100次 × 10ms = 1000ms = 1s)
        if (system_check_counter >= 100) // 检查系统状态计数器是否达到100
        {
            Check_System_Status();       // 调用系统状态检查函数
            system_check_counter = 0;    // 重置系统状态计数器

			/* 统计每秒滴数窗口并计算强度 */
			Push_Second_Count(0);       // 未在此秒新增则推0，真实新增在快照处理中累加
			current_intensity_mmh = Compute_Intensity_MMH(); // 计算当前降雨强度
        }
        
        Delay_ms(10);                    // 延时10毫秒，控制循环频率，降低CPU占用率
        display_counter++;               // 显示计数器加1
        system_check_counter++;          // 系统状态计数器加1
		second_loop_counter++;           // 秒循环计数器加1

		/* 事件级死区递减：在一定时间内丢弃新快照，避免同一滴重复计数 */
		if (event_deadtime_loops > 0)
		{
			event_deadtime_loops--;
		}

		if (second_loop_counter >= 100)  // 每100次循环（1秒）
		{
			second_loop_counter = 0;     // 重置秒循环计数器
			/* 每秒推进索引在 Push_Second_Count 中处理 */
		}
    }
}

/**
  * @brief  显示更新函数（合并通道）
  * @param  无
  * @retval 无
  * @note   更新OLED显示屏上的动态数据内容，显示两路信号的峰值和电压
  */
void Update_Display(void)
{
	/* 行1：合并峰值ADC值 */
	OLED_ShowNum(1, 6, current_peak, 5);
	OLED_ShowChar(1, 14, last_gain_used);

	/* 行2：合并峰值电压（V），保留2位小数 */
	{
		uint16_t v100 = (uint16_t)(current_voltage * 100.0f); // 电压值乘以100转换为整数
		OLED_ShowNum(2, 6, v100 / 100, 2); // 显示整数部分
		OLED_ShowChar(2, 8, '.');          // 显示小数点
		OLED_ShowNum(2, 9, (v100 % 100) / 10, 1); // 显示十分位
		OLED_ShowNum(2, 10, v100 % 10, 1);  // 显示百分位
	}

	/* 行3：实时雨量（降雨强度 mm/h），保留2位小数 */
	{
		uint16_t intensity100 = (uint16_t)(current_intensity_mmh * 100.0f); // 降雨强度乘以100转换为整数
		OLED_ShowNum(3, 7, intensity100 / 100, 3); // 显示整数部分（最多3位）
		OLED_ShowChar(3, 10, '.');          // 显示小数点
		OLED_ShowNum(3, 11, (intensity100 % 100) / 10, 1); // 显示十分位
		OLED_ShowNum(3, 12, intensity100 % 10, 1);  // 显示百分位
		OLED_ShowString(3, 14, "mm/h");    // 显示单位
	}

	/* 行4：累计电压总和（V），保留2位小数 */
	{
		uint16_t sum100 = (uint16_t)(voltage_sum * 100.0f); // 累计电压值乘以100转换为整数
		OLED_ShowNum(4, 6, sum100 / 100, 3); // 显示整数部分（最多3位）
		OLED_ShowChar(4, 9, '.');          // 显示小数点
		OLED_ShowNum(4, 10, (sum100 % 100) / 10, 1); // 显示十分位
		OLED_ShowNum(4, 11, sum100 % 10, 1);  // 显示百分位
		OLED_ShowString(4, 13, "V");       // 显示单位
	}

}

/**
  * @brief  系统状态检查函数
  * @param  无
  * @retval 无
  * @note   通过检查DMA传输计数来判断系统是否正常工作
  */
void Check_System_Status(void)
{
    /* 通过HT/TC推进的采样计数判断1s内是否有采样活动 */
    if (sampling_tick_counter == last_sampling_tick) // 如果采样计数没有变化
    {
        system_normal = 0;               // 系统状态异常
        /* 自恢复尝试 */
        AD_Restart();                    // 重启ADC
    }
    else                                // 如果采样计数有变化
    {
        system_normal = 1;               // 系统状态正常
    }
    last_sampling_tick = sampling_tick_counter; // 更新上一次采样计数
}

/* ========== 内部功能实现 ========== */

/**
  * @brief  处理快照数据（如果就绪）
  * @param  无
  * @retval 无
 * @note   模拟看门狗触发 → 预触发200点 + 后触发800点（双增益） → Validate_And_Count_Event
 *         1) DMA 持续向环形缓冲写入高/低增益两路数据；
 *         2) 模拟看门狗越界后立即复制200点历史样本，并继续采集800点后触发数据；
 *         3) 快照同时保留高增益和低增益波形，若高增益饱和则自动切换到低增益；
 *         4) 只分析“上升→峰值→回落至基线”这一正向半周期，忽略负半周；并记录脉冲长度；
 *         5) Validate_And_Count_Event 只使用有效段进行形状判定，配合 dead time/RETURN_THRESHOLD
 *            抑制单滴拖尾造成的重复计数。
 *         
 *         预触发处理时间计算（基于ADC_SAMPLE_INTERVAL_US = 42us）：
 *         - 预触发200点：200 × 42us = 8.4ms（从环形缓冲复制历史数据）
 *         - 后触发1300点：1300 × 42us = 54.6ms（继续采集实时数据）
 *         - 总快照窗口：1500 × 42us = 63ms（确保整个脉冲包括后部震荡都在快照内）
 *         - 触发点位置：索引200（峰值应位于此位置附近）
 *         
 *         注意：模拟看门狗触发后，DMA中断会在下一个样本处理时响应，延迟约1个采样周期（42us）
  */
static void Process_Snapshot_IfReady(void)
{
	if (!snapshot_ready)                 // 如果快照未就绪，直接返回
	{
		return;
	}

	/* 事件级死区内：直接丢弃本次快照，避免同一滴的拖尾触发 */
	if (event_deadtime_loops > 0)
	{
		snapshot_ready = 0;
		return;
	}

	uint16_t len = SNAPSHOT_SIZE;
	uint16_t high_peak_idx = 0, high_peak_val = 0;

	int32_t baseline_high = Compute_Baseline((uint16_t *)snapshot_buffer_high, len);

	/* 严格前部处理：初始峰值搜索允许在预触发区域和前部窗口内搜索，但不搜索后部数据 */
	/* 定义前部分析窗口：触发点后的前2ms（约48个采样点），完全忽略后部数据 */
	uint16_t front_window_start = PEAK_SEARCH_CENTER;  // 触发点索引（200）
	uint16_t front_window_end = PEAK_SEARCH_CENTER + FRONT_ANALYSIS_SAMPLES;  // 触发点后2ms（248）
	if (front_window_end > len)
	{
		front_window_end = len - 1;
	}

	/* 峰值搜索窗口：允许在预触发区域（触发点前80点）和前部窗口内搜索，确保能找到峰值 */
	/* 但分析时只使用前部窗口的数据，不分析后部数据 */
	uint16_t search_start = (PEAK_SEARCH_CENTER > PEAK_SEARCH_HALFSPAN) ?
		(PEAK_SEARCH_CENTER - PEAK_SEARCH_HALFSPAN) : 0;  // 允许搜索预触发区域
	uint16_t search_end = front_window_end - 1;  // 限制在前部窗口结束位置，不搜索后部

	Find_Peak_In_Buffer((uint16_t *)snapshot_buffer_high, len, baseline_high,
	                    &high_peak_idx, &high_peak_val, search_start, search_end);

	uint16_t *active_buffer = (uint16_t *)snapshot_buffer_high;
	int32_t active_baseline = baseline_high;
	uint16_t active_peak_index = high_peak_idx;
	uint16_t active_peak_value = high_peak_val;
	uint16_t threshold = dynamic_threshold;

	/* 当前仅使用PA0单通道，增益固定为高增益 */
	last_gain_used = 'H';

	/* 严格前部处理：只分析前部窗口内的数据（触发点后2ms），完全忽略后部数据 */
	/* 注意：front_window_start和front_window_end已在上面定义 */
	uint16_t start_index = front_window_start;    // front_start，从触发点开始
	while (start_index < front_window_end && active_buffer[start_index] <= (active_baseline + FRONT_START_DELTA))
	{
		start_index++;
	}
	if (start_index > active_peak_index)
	{
		start_index = (active_peak_index > SHAPE_WINDOW_PRE) ? (active_peak_index - SHAPE_WINDOW_PRE) : front_window_start;
	}
	if (start_index < front_window_start)
	{
		start_index = front_window_start;
	}

	uint16_t end_index = active_peak_index;   // front_end 初始从峰值开始向后搜索
	while (end_index < front_window_end && active_buffer[end_index] > (active_baseline + FRONT_END_DELTA))
	{
		end_index++;
	}
	/* 限制end_index不超过前部窗口结束位置 */
	if (end_index >= front_window_end)
	{
		end_index = front_window_end - 1;
	}

	/* 进一步确认回落点：需连续 TAIL_SETTLE_COUNT 个样本低于基线+delta */
	/* 但限制在前部窗口内搜索，不搜索后部数据 */
	uint16_t settle_idx = active_peak_index + 1;
	uint8_t settle_count = 0;
	uint16_t trimmed_end = end_index;
	while (settle_idx <= end_index && settle_idx < front_window_end)
	{
		if (active_buffer[settle_idx] <= (active_baseline + FRONT_END_DELTA))
		{
			settle_count++;
			if (settle_count >= FRONT_END_SETTLE_COUNT)
			{
				trimmed_end = settle_idx - (TAIL_SETTLE_COUNT - 1);
				break;
			}
		}
		else
		{
			settle_count = 0;
		}
		settle_idx++;
	}
	end_index = trimmed_end;
	if (end_index <= active_peak_index)
		end_index = (active_peak_index < len - 1) ? (active_peak_index + 1) : active_peak_index;
	/* 确保end_index不超过前部窗口结束位置 */
	if (end_index >= front_window_end)
	{
		end_index = front_window_end - 1;
	}

	/* 在前部区间 [start_index, end_index] 内重新搜索峰值，保证峰值仅来自前部 */
	/* 如果初始搜索找到的峰值不在前部窗口内，在前部窗口内重新搜索 */
	uint16_t front_peak_index = active_peak_index;
	uint16_t front_peak_value = active_peak_value;
	
	/* 验证峰值是否在前部窗口内 */
	if (active_peak_index < front_window_start || active_peak_index >= front_window_end)
	{
		/* 峰值不在前部窗口内（可能在预触发区域），在前部窗口内重新搜索峰值 */
		front_peak_index = front_window_start;
		front_peak_value = active_buffer[front_window_start];
		for (uint16_t i = front_window_start + 1; i < front_window_end; i++)
		{
			if (active_buffer[i] > front_peak_value)
			{
				front_peak_value = active_buffer[i];
				front_peak_index = i;
			}
		}
	}
	else if (end_index >= start_index)
	{
		/* 峰值在前部窗口内，在前部区间内重新搜索峰值 */
		front_peak_index = start_index;
		front_peak_value = active_buffer[start_index];
		for (uint16_t i = start_index + 1; i <= end_index; i++)
		{
			if (active_buffer[i] > front_peak_value)
			{
				front_peak_value = active_buffer[i];
				front_peak_index = i;
			}
		}
	}

	snapshot_peak_value = front_peak_value;
	snapshot_peak_index = front_peak_index;

	if (Validate_And_Count_Event(active_buffer, end_index + 1, front_peak_index, front_peak_value, threshold, start_index, end_index))
	{
		/* 峰值保持机制：在保持时间内，只有更大的峰值才能更新显示 */
		/* 关键改进：仅在保持时间内忽略明显小于旧峰值的新峰值（小于70%），防止后部震荡误判 */
		uint8_t should_update = 0;
		
		if (last_valid_peak == 0)
		{
			/* 第一次有效峰值，直接更新 */
			should_update = 1;
			peak_hold_counter = PEAK_HOLD_TIME_MS / 10;  // 转换为10ms单位
		}
		else if (peak_hold_counter > 0)
		{
			/* 保持时间内：检查新峰值是否明显大于旧峰值 */
			if (front_peak_value > (last_valid_peak + PEAK_HOLD_MIN_DELTA))
			{
				/* 新峰值明显大于旧峰值，更新 */
				should_update = 1;
				peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
			}
			else if (front_peak_value >= (uint16_t)(last_valid_peak * PEAK_HOLD_MIN_RATIO))
			{
				/* 新峰值大于旧峰值的70%，允许更新（支持快速连续大雨滴） */
				should_update = 1;
				peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
			}
			/* 否则（新峰值小于旧峰值的70%），直接忽略，防止后部震荡误判 */
		}
		else
		{
			/* 保持时间已过，检查是否是可疑峰值（明显小于旧峰值） */
			if (front_peak_value < (uint16_t)(last_valid_peak * RAPID_JUMP_RATIO))
			{
				/* 新峰值明显小于旧峰值（小于50%），检查距离上次更新时间 */
				uint32_t time_since_last_update = main_loop_counter - last_update_counter;
				
				if (time_since_last_update <= RAPID_JUMP_TIME_THRESHOLD)
				{
					/* 距离上次更新时间很短（30ms内），很可能是快速跳变（后部震荡），直接忽略，不进行延迟验证 */
					/* 不更新，不设置可疑峰值 */
				}
				else
				{
					/* 距离上次更新时间较长（超过30ms），可能是真实小雨滴，进行延迟验证 */
					suspicious_peak = front_peak_value;
					suspicious_peak_counter = RAPID_JUMP_FILTER_TIME_MS / 10;  // 转换为10ms单位
					/* 不立即更新，等待延迟验证 */
				}
			}
			else
			{
				/* 新峰值不是明显小于旧峰值，直接更新（包括正常的小雨滴） */
				should_update = 1;
				peak_hold_counter = PEAK_HOLD_TIME_MS / 10;
				/* 如果有可疑峰值，清除它（因为出现了更大的峰值，说明之前的是跳变） */
				suspicious_peak = 0;
				suspicious_peak_counter = 0;
			}
		}
		
		if (should_update)
		{
			/* 记录本次事件的原始峰值（仅来源于前部区间，单通道PA0） */
			current_peak_raw = front_peak_value;             // 实际采样通道的原始峰值

			/* 单通道模式下，显示值直接采用前部峰值 */
			current_peak = front_peak_value;
			float display_voltage = (float)current_peak / ADC_FULL_SCALE * ADC_REF_VOLTAGE;
			current_voltage = display_voltage;
			voltage_sum += current_voltage;                  // 累加电压总和
			last_valid_peak = front_peak_value;             // 更新保持的峰值
			last_update_counter = main_loop_counter;        // 更新最近一次峰值更新的主循环计数
			/* 如果有可疑峰值，清除它（因为出现了更大的峰值，说明之前的是跳变） */
			suspicious_peak = 0;
			suspicious_peak_counter = 0;
		}
		
		snapshot_valid_count++;

		drop_count++;                    // 雨滴计数加1
		total_rain_mm += MM_PER_DROP;    // 累计降雨量增加
		/* 将本秒计数+1 */
		if (sec_index < SECONDS_WINDOW)  // 如果索引在有效范围内
		{
			drops_per_second[sec_index] += 1; // 当前秒雨滴数加1
		}

		/* 在快照层面开启事件级死区，避免拖尾引起的重复快照触发 */
		event_deadtime_loops = EVENT_DEADTIME_LOOPS;
	}

	/* 备份快照用于导出 */
	for (uint16_t i_copy = 0; i_copy < len; i_copy++)
	{
		export_buffer[i_copy] = snapshot_buffer_high[i_copy];
	}
	export_ready = 1;                // 设置导出就绪标志
	snapshot_ready = 0;              // 清除快照就绪标志
}

/**
  * @brief  更新自适应阈值（双通道模式）
  * @param  无
  * @retval 无
  * @note   基于最近噪声统计分别为两路计算自适应阈值
  */
static void Update_Adaptive_Threshold(void)
{
	extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE]; // 通道0环形缓冲区
	extern volatile uint16_t ring_write_index_ch0; // 通道0写索引
	
	int32_t sum;                           // 求和变量
	uint16_t i;                           // 循环计数器
	int16_t start;                        // 起始索引
	int32_t mean_times_1;                 // 均值变量
	int32_t mad_sum;                      // 平均绝对偏差和
	int32_t mad;                          // 平均绝对偏差
	int32_t target;                       // 目标阈值

	if (NOISE_WINDOW > RING_BUFFER_SIZE) // 如果噪声窗口大于环形缓冲区大小
		return;

	/* ========== 通道0阈值计算（单通道PA0） ========== */
	sum = 0;
	start = (int16_t)ring_write_index_ch0 - NOISE_WINDOW;
	if (start < 0) start += RING_BUFFER_SIZE;
	for (i = 0; i < NOISE_WINDOW; i++)
	{
		sum += adc_ring_buffer_ch0[(start + i) % RING_BUFFER_SIZE];
	}
	mean_times_1 = sum / (int32_t)NOISE_WINDOW;

	mad_sum = 0;
	for (i = 0; i < NOISE_WINDOW; i++)
	{
		int32_t v = (int32_t)adc_ring_buffer_ch0[(start + i) % RING_BUFFER_SIZE];
		int32_t d = v - mean_times_1;
		if (d < 0) d = -d;
		mad_sum += d;
	}
	mad = mad_sum / (int32_t)NOISE_WINDOW;

	target = mean_times_1 + (int32_t)(MAD_GAIN * mad);
	if (target < MIN_THRESHOLD) target = MIN_THRESHOLD; // 限制最小值500mV
	if (target > MAX_THRESHOLD) target = MAX_THRESHOLD;

	if ((int32_t)dynamic_threshold - target > HYSTERESIS_MARGIN ||
		target - (int32_t)dynamic_threshold > HYSTERESIS_MARGIN)
	{
		dynamic_threshold = (uint16_t)target;
		AD_SetThreshold(dynamic_threshold); // 设置ADC模拟看门狗阈值（通道0/PA0）
	}
}

/**
  * @brief  验证并计数事件
  * @param  buf: 数据缓冲区指针
  * @param  len: 缓冲区长度
  * @param  peak_index: 峰值索引
  * @param  peak_value: 峰值数值
  * @param  threshold: 对应通道的动态阈值
  * @retval 1: 有效事件，0: 无效事件
  * @note   验证事件的有效性，包括幅值判定和形状判定
  */
static uint8_t Validate_And_Count_Event(uint16_t *buf, uint16_t len, uint16_t peak_index, uint16_t peak_value, uint16_t threshold, uint16_t start_index, uint16_t end_index)
{
	uint16_t pre;                        // 峰前样本数
	uint16_t post;                       // 峰后样本数
	uint16_t rise_ok;                    // 上升斜率计数
	uint16_t decay_ok;                   // 下降斜率计数
	int i;                               // 循环计数器

	if (len == 0 || start_index >= len)
		return 0;
	if (end_index >= len)
		end_index = len - 1;
	if (peak_index < start_index || peak_index > end_index)
		return 0;

	/* 1) 幅值判定 */
	if (peak_value <= threshold) return 0; // 如果峰值不超过通道阈值
	if (peak_value < (uint16_t)(threshold + MIN_PEAK_DELTA_OVER_THR)) return 0; // 如果峰值余量不足
	if (peak_value < MIN_PEAK_AMPLITUDE) return 0; // 如果峰值幅度太小，可能是噪声

	/* 2) 形状判定：峰前上升&峰后下降（避免随机振动） */
	uint16_t available_pre = peak_index - start_index;
	pre = (available_pre > SHAPE_WINDOW_PRE) ? SHAPE_WINDOW_PRE : available_pre; // 计算峰前样本数
	uint16_t available_post = (end_index > peak_index) ? (end_index - peak_index) : 0;
	post = (available_post > SHAPE_WINDOW_POST) ? SHAPE_WINDOW_POST : available_post; // 计算峰后样本数
	if (pre < MIN_RISE_SAMPLES || post < MIN_DECAY_SAMPLES) return 0; // 如果样本数不足

	/* 峰前：计算上升斜率数量 */
	rise_ok = 0;                         // 初始化上升计数
	int rise_start = (int)peak_index - (int)pre + 1;
	if (rise_start < (int)start_index + 1)
		rise_start = (int)start_index + 1;
	for (i = rise_start; i <= (int)peak_index; i++) // 遍历峰前样本
	{
		if (buf[i] > buf[i - 1]) rise_ok++; // 如果当前值大于前一个值，计数加1
	}
	/* 峰后：计算下降斜率数量 */
	decay_ok = 0;                        // 初始化下降计数
	int decay_end = (int)peak_index + (int)post;
	if (decay_end > (int)end_index)
		decay_end = (int)end_index;
	for (i = (int)peak_index + 1; i <= decay_end; i++) // 遍历峰后样本
	{
		if (buf[i] < buf[i - 1]) decay_ok++; // 如果当前值小于前一个值，计数加1
	}
	if (rise_ok < MIN_RISE_SAMPLES || decay_ok < MIN_DECAY_SAMPLES)
	{
		/* 对于窄脉冲，要求更严格的差值条件，避免噪声误判 */
		uint16_t left_now = (peak_index > start_index) ? buf[peak_index - 1] : buf[peak_index];
		uint16_t right_now = ((peak_index + 1) <= end_index) ? buf[peak_index + 1] : buf[peak_index];
		/* 要求峰值相对于邻近样本的差值至少是 MIN_LOCAL_DELTA 的2倍 */
		uint16_t min_diff_required = MIN_LOCAL_DELTA * 2;
		if ((peak_value > left_now + min_diff_required) && (peak_value > right_now + min_diff_required))
		{
			return 1;
		}
		/* 如果前后样本差值不足，检查更远的样本 */
		if ((peak_index > (start_index + 1)) && peak_value > buf[peak_index - 2] + min_diff_required)
		{
			if ((peak_index + 2) <= end_index && peak_value > buf[peak_index + 2] + min_diff_required)
			{
				return 1;
			}
		}
		return 0;
	}

	/* 3) 通过判定 */
	return 1;                            // 返回有效事件
}

/**
  * @brief  推送秒计数到窗口
  * @param  drops_in_second: 当前秒的雨滴数
  * @retval 无
  * @note   将当前秒的雨滴数推送到统计窗口
  */
static void Push_Second_Count(uint16_t drops_in_second)
{
	/* 每1秒推进一次索引，将当前秒的计数写入窗口 */
	if (second_loop_counter == 0)        // 如果秒循环计数器为0（每秒一次）
	{
		sec_index = (sec_index + 1) % SECONDS_WINDOW; // 更新秒索引（环形）
		drops_per_second[sec_index] = drops_in_second; // 设置当前秒雨滴数
	}
	else                                // 如果不是秒切换时刻
	{
		/* 在当前秒内增量叠加由Process_Snapshot_IfReady完成 */
		(void)drops_in_second;           // 避免未使用参数警告
	}
}

/**
  * @brief  计算降雨强度（mm/h）
  * @param  无
  * @retval 降雨强度值（毫米/小时）
  * @note   基于最近60秒的雨滴数计算当前降雨强度
  */
static float Compute_Intensity_MMH(void)
{
	uint32_t sum = 0;                    // 求和变量
	uint8_t i;                          // 循环计数器
	for (i = 0; i < SECONDS_WINDOW; i++) // 遍历统计窗口
		sum += drops_per_second[i];      // 累加雨滴数
	/* 60秒内滴数 -> 每小时：* (3600/60) = *60；每滴折算mm */
	return (float)sum * MM_PER_DROP * 60.0f / (float)SECONDS_WINDOW; // 计算降雨强度
}

static int32_t Compute_Baseline(uint16_t *buf, uint16_t len)
{
	uint16_t base_count = (BASELINE_SAMPLE_COUNT < len) ? BASELINE_SAMPLE_COUNT : len;
	if (base_count == 0)
		return 0;
	uint32_t base_sum = 0;
	for (uint16_t i = 0; i < base_count; i++)
	{
		base_sum += buf[i];
	}
	return (int32_t)(base_sum / (uint32_t)base_count);
}

static void Find_Peak_In_Buffer(uint16_t *buf, uint16_t len, int32_t baseline,
                                uint16_t *peak_index, uint16_t *peak_value,
                                uint16_t search_start, uint16_t search_end)
{
	if (len == 0)
	{
		*peak_index = 0;
		*peak_value = 0;
		return;
	}

	if (search_start >= len)
		search_start = 0;
	if (search_end >= len)
		search_end = len - 1;
	if (search_start > search_end)
	{
		uint16_t tmp = search_start;
		search_start = 0;
		search_end = tmp;
		if (search_end >= len)
			search_end = len - 1;
	}

	int32_t max_delta = INT32_MIN;
	uint16_t rough_idx = 0;
	for (uint16_t i = search_start; i <= search_end; i++)
	{
		int32_t acc = buf[i];
		uint8_t denom = 1;
		if (i > 0)
		{
			acc += buf[i - 1];
			denom++;
		}
		if (i + 1 < len)
		{
			acc += buf[i + 1];
			denom++;
		}
		int32_t smooth = acc / denom;
		int32_t delta = smooth - baseline;
		if (delta > max_delta)
		{
			max_delta = delta;
			rough_idx = i;
		}
	}

	uint16_t refine_start = (rough_idx > LOCAL_REFINEMENT_RADIUS) ? (rough_idx - LOCAL_REFINEMENT_RADIUS) : search_start;
	if (refine_start < search_start) refine_start = search_start;
	uint16_t refine_end = (rough_idx + LOCAL_REFINEMENT_RADIUS < len) ? (rough_idx + LOCAL_REFINEMENT_RADIUS) : (len - 1);
	if (refine_end > search_end) refine_end = search_end;
	uint16_t max_v = buf[rough_idx];
	uint16_t max_i = rough_idx;
	for (uint16_t i = refine_start; i <= refine_end; i++)
	{
		if (buf[i] > max_v)
		{
			max_v = buf[i];
			max_i = i;
		}
	}

	*peak_index = max_i;
	*peak_value = max_v;
}

static uint16_t Scale_Value_With_Gain(uint16_t value, float gain)
{
	float scaled = (float)value * gain;
	if (scaled < 0.0f)
	{
		return 0;
	}
	if (scaled > ADC_FULL_SCALE)
	{
		return (uint16_t)ADC_FULL_SCALE;
	}
	return (uint16_t)(scaled + 0.5f);
}
