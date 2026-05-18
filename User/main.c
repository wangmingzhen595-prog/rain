// 包含必要的头文件
#include "stm32f10x.h"                  // STM32F10x微控制器标准外设库头文件
#include "Delay.h"                       // 延时函数头文件
#include "OLED.h"                        // OLED显示屏驱动头文件
#include "AD.h"                          // ADC模数转换器驱动头文件
#include "stm32f10x_usart.h"             // 串口通信头文件
#include "Modbus_Slave.h"                // Modbus 从站头文件
#include "stm32f10x_gpio.h"              // GPIO口操作头文件
#include "stm32f10x_rcc.h"               // 时钟控制头文件
#include "stm32f10x_iwdg.h"              // 独立看门狗头文件（用于死机自启动）
#include "TF_Comm.h"                     // TF通信模块头文件
#include "SPI_SlaveLink.h"               // SPI 从机链路层（SPI1 + EXTI4）
#include "Wavelet.h"                     // 小波变换模块头文件
#include "raindrop.h"                    // 雨滴体积换算模块头文件

/* ===================== Modbus 从站地址配置 ===================== */
/* 修改此宏即可为本单片机设置 Modbus 从站地址（1~247） */
#define MODBUS_SLAVE_ID    1

// ========== 系统参数定义 ==========
/* 电压校准系数（方便修改） */
#define ADC_VOLTAGE_CALIBRATION_FACTOR  0.9155f  // 电压校准系数（示波器2.60V对应单片机2.84V：2.60/2.84≈0.9155）
#define ADC_VOLTAGE_CALIBRATION_NUM     9155UL   // 同一校准系数的整数分子，用于积分计算，避免主循环里用浮点累加
#define ADC_VOLTAGE_CALIBRATION_DEN     10000UL  // 同一校准系数的整数分母：9155/10000=0.9155
#define ADC_REF_MV                      3300UL   // ADC参考电压，单位mV

/* PA1通道配置：用于PA0接近饱和时切换（测量大于3.3V的雨滴） */
#define PA0_SWITCH_TO_PA1_HIGH    4095     // PA0切换到PA1的阈值（3.3V，ADC满量程，当PA0饱和时切换）
#define PA0_SWITCH_TO_PA1_LOW     3500     // PA0切回PA0的阈值（低阈值，滞回下限，约2.85V）
#define PA0_SATURATION_THRESHOLD  4095     // PA0饱和阈值（3.3V，ADC满量程，保留用于兼容）
#define PA1_AMPLIFICATION_FACTOR  1.0f     // PA1放大倍数（默认1.0，后续根据实际信号测试调整）
#define PA1_VOLTAGE_CALIBRATION_FACTOR  0.701f  // PA1电压校准系数（示波器2.84V对应程序显示4.05V：2.84/4.05≈0.701）
#define PA1_MIN_VALID_VALUE       50       // PA1最小有效值（噪声阈值，低于此值认为PA1无效）
#define PA1_VALID_MARGIN          100      // PA1有效性判断余量（pa1_raw必须大于噪声基线+余量才有效，避免固定偏置误判）
#define PA1_BASELINE_WINDOW       32       // PA1噪声基线统计窗口（事件前的采样点数，用于计算噪声基线）
#define PA1_INVALID_COUNT_THRESHOLD 3      // PA1连续无效次数阈值（连续3次无效才回退PA0，避免单点失效导致跳变）
#define PA1_MAX_SCALED_VALUE      65535    // PA1放大后的最大值限制（uint16_t最大值）

#define THRESHOLD 496                     // 初始阈值（ADC单位，400mV = 400/3.3*4095 ≈ 496）

/* 增益配置：CH0=高增益、CH1=低增益（默认≈15倍 vs 1倍，可按需调整） */
#define HIGH_GAIN_FACTOR         15.0f
#define LOW_GAIN_FACTOR          1.0f
#define HIGH_GAIN_SAT_THRESHOLD  4000     // 高增益ADC达到该值视为饱和（接近3.3V）
#define ADC_FULL_SCALE           4095     // ADC满量程值（12位ADC，明确写死为4095，防止未来换ADC分辨率出坑）
#define ADC_REF_VOLTAGE          3.3f

/* 自适应阈值相关 */
#define NOISE_WINDOW            200      // 噪声统计窗口长度（从环形缓冲末端向前取）
#define MIN_THRESHOLD           496       // 阈值下限，防止过低（400mV = 400/3.3*4095 ≈ 496）
#define MAX_THRESHOLD           3000     // 阈值上限，防止过高
#define MAD_GAIN                3        // 平均绝对偏差放大倍数
#define HYSTERESIS_MARGIN       15       // 阈值滞回，降低抖动

/* 事件与抗干扰判定 */
#define MIN_PEAK_DELTA_OVER_THR 8        // 峰值需高出阈值的最小余量（约6.5mV，适配小信号）
#define MIN_LOCAL_DELTA         6        // 峰值相对于邻近样本的最小差值（适配小信号）
#define MIN_RISE_SAMPLES        3        // 峰前上升最少采样点数
#define MIN_DECAY_SAMPLES       3        // 峰后下降最少采样点数
#define SHAPE_WINDOW_PRE        12       // 峰前用于形状判定的样本数
#define SHAPE_WINDOW_POST       24       // 峰后用于形状判定的样本数

/* 时间特征判定参数：区分真实雨滴信号和噪声干扰 */
/* 基于示波器数据：上升时间350.50μs，正脉宽1.36ms，周期4.33ms */
#define MIN_RISE_TIME_US        100      // 最小上升时间（微秒），适配140-180us小针头雨滴上升沿
#define MIN_FALL_TIME_US        300      // 最小下降时间（微秒），适配小雨滴信号（降低到300us）
#define MIN_PULSE_DURATION_US   700      // 最小脉冲持续时间（微秒），覆盖0.8-1.1ms主正脉冲
#define MAX_NOISE_PULSE_WIDTH   10       // 最大噪声脉冲宽度（采样点数），超过此宽度才可能是真实信号（约420us）

/* 波形对称性判定参数：雨滴波形相对对称 */
#define RISE_FALL_TIME_RATIO_MIN  0.3f   // 上升/下降时间比最小值（0.3，允许上升时间短于下降时间）
#define RISE_FALL_TIME_RATIO_MAX  3.0f   // 上升/下降时间比最大值（3.0，允许上升时间最长是下降时间的3倍）

/* 形状平滑度判定参数：真实信号相对平滑，干扰可能很陡峭 */
#define MIN_SMOOTH_RISE_RATIO   0.25f    // 最小平滑上升比例：适配小雨滴信号（降低到25%）
#define MIN_SMOOTH_FALL_RATIO   0.25f    // 最小平滑下降比例：适配小雨滴信号（降低到25%）
#define MAX_STEEP_SLOPE         50       // 最大陡峭斜率（ADC单位/样本），适配小雨滴信号（提高到50）
#define PEAK_STABILITY_WINDOW   5        // 峰值稳定性窗口：峰值附近±N个样本应该接近峰值
#define PEAK_STABILITY_DELTA    30       // 峰值稳定性容差：峰值附近样本与峰值的最大差值

/* 信号平滑滤波参数 */
#define SMOOTH_FILTER_SIZE      3        // 移动平均滤波窗口大小（3点或5点）
#define BASELINE_SAMPLE_COUNT   80       // 基线估算样本数
#define LOCAL_REFINEMENT_RADIUS 6        // 峰值局部搜索半径
#define MIN_PEAK_AMPLITUDE      500      // 最小峰值幅度（ADC单位，约400mV），适配420-540mV小雨滴信号

/* 显示门限：小于该幅度的脉冲不刷新OLED（仅在主循环中使用） */
#define DISPLAY_MIN_AMPLITUDE   400      // 显示下限约 320mV，适配420-540mV小雨滴信号显示

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

/* 前/后部分割参数（仅分析前部"上升→峰值→回落到基线"） */
#define FRONT_START_DELTA       MIN_LOCAL_DELTA   // 认为"离开基线"的门限
#define FRONT_END_DELTA         MIN_LOCAL_DELTA   // 认为"回到基线"的门限
#define FRONT_END_SETTLE_COUNT  TAIL_SETTLE_COUNT// 回落连续样本计数

/* 严格前部处理：每个脉冲信号只取前部（约2ms），后部数据完全不分析，避免后部抖动导致跳变 */
#define FRONT_ANALYSIS_TIME_MS  2.0f              // 前部分析时间窗口（毫秒）
#define FRONT_ANALYSIS_SAMPLES   ((uint16_t)(FRONT_ANALYSIS_TIME_MS * 1000.0f / ADC_SAMPLE_INTERVAL_US))  // 前部分析采样点数（约48点）

#define US_TO_SAMPLES_CEIL(us) \
	((uint16_t)((((uint32_t)(us) * 1000UL) + ADC_SAMPLE_INTERVAL_NS - 1UL) / ADC_SAMPLE_INTERVAL_NS))

#define MAIN_PULSE_PRE_US          350U
#define MAIN_PULSE_POST_US         1400U
#define MAIN_PULSE_PRE_SAMPLES     US_TO_SAMPLES_CEIL(MAIN_PULSE_PRE_US)
#define MAIN_PULSE_POST_SAMPLES    US_TO_SAMPLES_CEIL(MAIN_PULSE_POST_US)
#define MAIN_PULSE_BASELINE_DELTA  8U
#define MAIN_PULSE_SETTLE_COUNT    3U
#define MAIN_PULSE_MIN_WIDTH       8U

/* 峰值大小分类阈值（用于动态调整参数） */
#define PEAK_SMALL_THRESHOLD    600      // 小雨滴阈值（ADC单位，约480mV）
#define PEAK_MEDIUM_THRESHOLD   1500     // 中等雨滴阈值（ADC单位，约1.2V）

/* 事件级死区：用于在快照层面避免同一滴的重复触发（单位：主循环次数，10ms/次） */
#define EVENT_DEADTIME_LOOPS    50       // 约 500ms，可按需要标定，避免同一滴的拖尾触发新的快照（将被动态死区替代）

/* 小波分析相关参数（第2阶段，保留兼容） */
#define WAVELET_THRESHOLD_T1    300      // 初始阈值（permille，0-1000），用于触发动态死区，不用于否决
#define DEADTIME_SUSPECT_SCALE_NUM   2   // SUSPECT时死区倍数（分子）
#define DEADTIME_SUSPECT_SCALE_DEN   1   // SUSPECT时死区倍数（分母）
#define DEADTIME_SUSPECT_ADD_LOOPS   0   // 如果要加固定量，用 loops 口径写死

/* 小波分析固定阈值分段（第3阶段，不标定版，经验阈值） */
#define HF_BG_MAX        100   // <0.10  慢变背景/漂移
#define HF_RAIN_LOW      150   // 0.15
#define HF_RAIN_HIGH     300   // 0.30  真实雨滴典型上限
#define HF_SPLASH_HIGH   400   // 0.40  高速小雨滴/飞溅上限
#define HF_VIB_MIN       450   // >0.45 结构振动/风扰/机械敲击
#define HF_EMI_MIN       600   // >0.60 电噪声/EMI/ADC毛刺

/* 动态死区按类型分级（第3阶段） */
#define DT_SCALE_VIB_NUM   3   // VIB类型死区倍数
#define DT_SCALE_VIB_DEN   1   // VIB类型死区倍数（分母）
#define DT_SCALE_EMI_NUM   4   // EMI类型死区倍数
#define DT_SCALE_EMI_DEN   1   // EMI类型死区倍数（分母）

/* 雨量学参数（可配置，需根据传感器标定修正） */
float g_mm_per_drop = 0.02f;            // 每个有效雨滴折合降雨量（mm/滴），可通过外部接口修改，默认值0.02mm/滴

/* 强度统计（mm/h）——用近60秒的滴数计算 */
#define SECONDS_WINDOW          60       // 统计窗口大小（秒）

/* ========== 独立看门狗(IWDG)参数定义 ========== */
/* 看门狗配置：用于死机自启动功能
 * - 时钟源：LSI（内部低速时钟，约32kHz）
 * - 预分频：64 → 看门狗时钟 = 32kHz / 64 = 500Hz
 * - 超时时间：3秒（300次主循环，每次10ms）
 * - 重载值：1500（1500 / 500Hz = 3秒）
 * 注意：看门狗一旦使能无法软件关闭，只能通过硬件复位或断电
 */
#define IWDG_TIMEOUT_SECONDS    3        // 看门狗超时时间（秒）
#define IWDG_PRESCALER          IWDG_Prescaler_64  // 预分频器：64分频
#define IWDG_RELOAD_VALUE       1500     // 重载值：1500（对应3秒超时）

/* ========== OLED显示格式定义（防止显示标签错误） ========== */
/* 行1显示格式：Volt:xxx.xx G:x
 * 行2显示格式：Imp:xxxxxxxxxx（主正脉冲电压积分，mV*us）
 * 行3显示格式：Wxxx Rxxx Fxxx（主正脉冲宽度/上升/下降样本数）
 * 行4显示格式：OK A:xxxxxxxxx 或 NO A:xxxxxxxxx（窗口提取状态/原始正面积）
 */

// ========== 全局变量定义 ==========
// 通道0（第一路）变量
uint16_t current_peak_raw = 0;           // 当前峰值对应通道的原始ADC值
uint16_t current_peak = 0;               // 当前检测到的等效高增益峰值
float current_voltage = 0.0;             // 当前峰值对应的电压值(单位:伏特V)（通道0）
float voltage_sum = 0.0;                 // 累积电压总和(单位:伏特V)
uint32_t current_impulse_mv_us = 0;      // 主正脉冲电压积分值，单位：mV*us（未标定前是冲量代理量）
uint32_t current_pulse_area = 0;         // 主正脉冲正面积，单位：ADC码*采样点
uint16_t current_pulse_width = 0;        // 主正脉冲宽度，单位：采样点
uint16_t current_pulse_rise = 0;         // 主正脉冲上升样本数
uint16_t current_pulse_fall = 0;         // 主正脉冲下降样本数
uint8_t current_pulse_valid = 0;         // 主脉冲窗口提取状态：1=成功，0=失败

/* 峰值保持机制：避免小噪声覆盖大峰值 */
static uint16_t last_valid_peak = 0;    // 上一次有效的峰值（用于峰值保持）
static uint32_t peak_hold_counter = 0;   // 峰值保持计数器
#define PEAK_HOLD_TIME_MS    200         // 峰值保持时间（毫秒），200ms内只显示更大的峰值，确保快速连续雨滴仍能检测（将被动态时间替代）
#define PEAK_HOLD_MIN_DELTA  200         // 新峰值必须比旧峰值大至少200个ADC单位才更新（约160mV，中等以上雨滴）
#define PEAK_HOLD_MIN_DELTA_SMALL  100   // 小雨滴的峰值更新阈值（约80mV），支持快速切换显示
#define PEAK_HOLD_MIN_RATIO  0.7f        // 新峰值必须大于旧峰值的70%才更新（仅在保持时间内生效，防止后部震荡误判）

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

typedef struct
{
	uint8_t valid;
	uint16_t win_start;
	uint16_t win_end;
	uint16_t pulse_start;
	uint16_t pulse_end;
	uint16_t peak_index;
	uint16_t peak_value;
	uint16_t peak_delta;
	uint16_t rise_samples;
	uint16_t fall_samples;
	uint16_t width20_samples;
	uint16_t width50_samples;
	uint32_t area_pos;
	uint32_t impulse_mv_us;
	uint32_t energy_pos;
} MainPulseFeatures_t;

// ========== 函数声明 ==========
void Update_Display(void);               // 显示更新函数声明
void Check_System_Status(void);          // 系统状态检查函数声明
static void IWDG_Init(void);             // 独立看门狗初始化函数声明
static void Process_Snapshot_IfReady(void);  // 处理触发快照（100+900）
static void Update_Adaptive_Threshold(void); // 计算噪声并自适应阈值
static uint8_t Validate_And_Count_Event(uint16_t *buf, uint16_t len, uint16_t peak_index, uint16_t peak_value, uint16_t threshold, uint16_t start_index, uint16_t end_index); // 验证并计数事件
static void Push_Second_Count(uint16_t drops_in_second); // 推送秒计数到窗口
static float Compute_Intensity_MMH(void); // 计算降雨强度（mm/h）
static int32_t Compute_Baseline(uint16_t *buf, uint16_t len);
static int32_t Compute_Wavelet_Baseline(uint16_t trigger_index); // 计算小波baseline（从预触发环形缓冲区）
static float Compute_Voltage_From_ADC(uint16_t adc_value);  // 从ADC值计算电压（应用校准系数）
static uint16_t Compute_PA1_Noise_Baseline(uint16_t pa1_ring_index);  // 计算PA1噪声基线（事件前的窗口）
static inline uint32_t Scale_Clamp_PA1(uint16_t raw);  // PA1放大并clamp（内联函数，减少重复代码）
static void Find_Peak_In_Buffer(uint16_t *buf, uint16_t len, int32_t baseline,
                                uint16_t *peak_index, uint16_t *peak_value,
                                uint16_t search_start, uint16_t search_end);
static uint8_t Extract_MainPulse_Features(uint16_t *buf, uint16_t len,
                                          uint16_t anchor_peak_index,
                                          int32_t baseline,
                                          MainPulseFeatures_t *features);
static uint8_t Build_MainPulse_Features_From_Segment(uint16_t *buf, uint16_t len,
                                                     uint16_t start_index,
                                                     uint16_t end_index,
                                                     uint16_t peak_index,
                                                     int32_t baseline,
                                                     MainPulseFeatures_t *features);
static uint16_t Clamp_U16_From_I32(int32_t value);
static uint16_t Clamp_ADC_Delta_From_I32(int32_t value);
static uint32_t Integrate_MainPulse_mV_us(uint16_t *buf, uint16_t start_index,
                                          uint16_t end_index, int32_t baseline);
static void Update_MainPulse_Display_State(MainPulseFeatures_t *features);

/* 小波分析事件分类（第2阶段，保留兼容） */
typedef enum {
    EVENT_CLASS_GOOD = 0,      // hf_ratio <= T1
    EVENT_CLASS_SUSPECT = 1,   // hf_ratio > T1
    EVENT_CLASS_BAD = 2        // success=0（energy_total==0已在Wavelet模块内归到success=0）
} EventClass_t;

/* 小波分析事件类型（第3阶段，更细化的类型划分） */
typedef enum {
    EVT_BG = 0,           // hf_ratio < 100（背景漂移/无效）
    EVT_RAIN_CLEAN = 1,    // 150~300（单滴、干净）
    EVT_RAIN_FAST = 2,     // 250~400（高速小滴/飞溅，仍可能是"真雨滴类"，但更"尖"）
    EVT_VIB = 3,           // >450（振动/风扰/敲击类）
    EVT_EMI = 4,           // >600（电噪声毛刺类）
    EVT_BAD = 5            // wavelet.success==0（窗口/能量异常，退回时域）
} EventType_t;

static EventClass_t Determine_Event_Class(WaveletFeatures_t *features); // 根据小波特征确定事件类别（第2阶段）
static EventType_t Determine_Event_Type(WaveletFeatures_t *features);   // 根据小波特征确定事件类型（第3阶段）
static uint16_t Scale_Value_With_Gain(uint16_t value, float gain);
static void USART1_Config(void);          // 配置USART1用于VOFA+输出
static void USART1_SendByte(uint8_t b);   // 发送单字节
static void USART1_SendFloat_WithTail(float v); // 发送float并附加JustFloat尾标志
static void USART1_SendString(const char *str);  // 发送字符串（调试用）
static void USART1_SendUint32(uint32_t val);     // 发送uint32_t数字（调试用）
static void Send_Live_Stream(void);       // 连续下采样输出，提供示波数据流
static void Send_Debug_Statistics(void); // 发送调试统计信息
static uint16_t Get_Dynamic_Event_Deadtime(uint16_t peak_value); // 根据峰值大小返回动态事件级死区时间（主循环数）
static uint16_t Get_Dynamic_Peak_Hold_Time(uint16_t peak_value); // 根据峰值大小返回动态峰值保持时间（主循环数）
static uint8_t Update_Peak_Display(uint16_t peak_value); // 统一处理峰值显示更新，返回1表示已更新，0表示未更新

/* 触发与统计变量（当前仅使用PA0单通道） */
volatile extern uint8_t snapshot_ready;  // 快照就绪标志（外部定义）
volatile extern uint16_t snapshot_buffer_high[SNAPSHOT_SIZE]; // 高增益快照缓冲区
volatile extern uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];  // 低增益快照缓冲区
volatile extern uint8_t snapshot_collecting; // 快照采集状态标志（外部定义）
volatile extern uint16_t snapshot_peak_value; // 快照峰值（外部定义）
volatile extern uint16_t snapshot_peak_index; // 快照峰值索引（外部定义）
volatile extern uint32_t sampling_tick_counter; // 采样计数（外部定义，用于系统状态检查）

/* 导出缓存与标志（保存最近一次完整快照，用于命令导出） */
static volatile uint16_t export_buffer[SNAPSHOT_SIZE]; // 导出数据缓冲区（PA0波形）
volatile uint8_t export_ready = 0;                // 导出就绪标志

/* 自适应阈值运行变量（当前仅针对通道0/PA0） */
volatile uint16_t dynamic_threshold = THRESHOLD;  // 动态阈值变量（通道0）- 在中断中使用，需volatile

/* 滴数与累计雨量 - 在中断中使用，需volatile或移除static */
volatile uint32_t drop_count = 0;          // 雨滴计数
volatile float total_rain_mm = 0.0f;       // 累计降雨量（毫米）

/* 小波分析分类统计计数器（第2阶段，保留兼容） */
volatile uint32_t wavelet_good_count = 0;     // 只统计GOOD
volatile uint32_t wavelet_suspect_count = 0;  // 只统计SUSPECT
volatile uint32_t wavelet_bad_count = 0;     // 只统计BAD（排查小波窗口/基线错误很有用）

/* 小波分析事件类型统计计数器（第3阶段） */
volatile uint32_t cnt_rain_clean = 0;  // RAIN_CLEAN类型计数
volatile uint32_t cnt_rain_fast = 0;  // RAIN_FAST类型计数
volatile uint32_t cnt_vib = 0;        // VIB类型计数
volatile uint32_t cnt_emi = 0;        // EMI类型计数
volatile uint32_t cnt_bg = 0;          // BG类型计数
volatile uint32_t cnt_bad = 0;        // BAD类型计数（与wavelet_bad_count相同语义）

/* 原始事件与有效雨量计数（第4阶段） */
volatile uint32_t raw_event_count = 0;        // 通过时域判定的事件（触发链认可的"事件"）
volatile uint32_t effective_drop_count = 0;  // 最终计入雨量的事件（第4阶段才用）

/* 近60秒滴数窗口 - 在中断中使用 */
volatile uint16_t drops_per_second[SECONDS_WINDOW] = {0}; // 每秒雨滴数数组
volatile uint8_t sec_index = 0;            // 当前秒索引
static uint16_t second_loop_counter = 0; // 10ms循环累加到100为1秒

/* OLED显示缓存 */
float current_intensity_mmh = 0.0f;  // 当前降雨强度（毫米/小时），供Modbus从站读取
char  last_gain_used = 'H';         // 最近一次使用的增益通道（'H'或'L'），供Modbus从站读取
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
    USART1_Config();                     // 初始化USART1串口（PA10=TX，PA9=RX，115200 8N1）
    /* SPI 从站链路层初始化（SPI1，PA4=NSS，PA5=SCK，PA6=MISO，PA7=MOSI） */
    SPI_SlaveLink_Init(MODBUS_SLAVE_ID);
    Raindrop_Init();                     // 初始化雨滴体积换算模块
    
    // ========== 显示静态内容 ==========
	OLED_ShowString(1, 1, "Volt:000.00 G:H");
	OLED_ShowString(2, 1, "Imp:0000000000");
	OLED_ShowString(3, 1, "W000 R000 F000");
	OLED_ShowString(4, 1, "NO A:000000000");
    
    // ========== 独立看门狗初始化（在所有初始化完成后） ==========
    IWDG_Init();                         // 初始化独立看门狗，超时时间3秒，用于死机自启动
    IWDG_ReloadCounter();                // 立即喂一次狗，确保看门狗计数器从最大值开始
    
    // ========== 主循环 ==========
    while (1)                            // 程序主要逻辑
    {
		main_loop_counter++;             // 主循环计数器递增（每10ms递增一次）
		/* ISR峰值是最灵敏的每滴触发源：用环形缓冲窗口一次性提交Volt和Imp */
		if (last_peak_ready_from_isr)
		{
			static uint16_t isr_window_copy[ISR_CAPTURE_WINDOW_SIZE];
			uint16_t peak_candidate;
			uint16_t isr_window_len;
			int32_t isr_baseline;
			MainPulseFeatures_t isr_pulse_features;
			uint8_t isr_pulse_ok = 0;

			__disable_irq();
			peak_candidate = last_peak_value_from_isr;
			isr_window_len = last_peak_window_len_from_isr;
			isr_baseline = last_peak_baseline_from_isr;
			if (isr_window_len > ISR_CAPTURE_WINDOW_SIZE)
			{
				isr_window_len = ISR_CAPTURE_WINDOW_SIZE;
			}
			for (uint16_t i = 0; i < isr_window_len; i++)
			{
				isr_window_copy[i] = last_peak_window_from_isr[i];
			}
			last_peak_ready_from_isr = 0;
			__enable_irq();

			if (isr_window_len > ISR_CAPTURE_PRE_SAMPLES)
			{
				isr_pulse_ok = Extract_MainPulse_Features(isr_window_copy,
				                                           isr_window_len,
				                                           ISR_CAPTURE_PRE_SAMPLES,
				                                           isr_baseline,
				                                           &isr_pulse_features);
				if (!isr_pulse_ok)
				{
					isr_pulse_ok = Build_MainPulse_Features_From_Segment(isr_window_copy,
					                                                     isr_window_len,
					                                                     0,
					                                                     (uint16_t)(isr_window_len - 1U),
					                                                     ISR_CAPTURE_PRE_SAMPLES,
					                                                     isr_baseline,
					                                                     &isr_pulse_features);
				}
			}
			if (isr_window_len > ISR_CAPTURE_PRE_SAMPLES &&
			    peak_candidate >= DISPLAY_MIN_AMPLITUDE &&
			    isr_pulse_ok)
			{
				uint16_t dynamic_hold_time = Get_Dynamic_Peak_Hold_Time(isr_pulse_features.peak_value);

				current_peak_raw = isr_pulse_features.peak_value;
				current_peak = isr_pulse_features.peak_value;
				current_voltage = Compute_Voltage_From_ADC(current_peak);
				voltage_sum += current_voltage;
				last_gain_used = 'H';
				last_valid_peak = current_peak;
				last_update_counter = main_loop_counter;
				peak_hold_counter = dynamic_hold_time;
				USART1_SendFloat_WithTail(current_voltage);
				Update_MainPulse_Display_State(&isr_pulse_features);
			}
		}
		
		/* 峰值保持计数器递减 */
		if (peak_hold_counter > 0)
		{
			peak_hold_counter--;
		}

        /* 处理快照数据（如果就绪）：用于精确的事件验证和计数 */
        Process_Snapshot_IfReady();

		/* 自适应阈值（基于最近噪声） */
		Update_Adaptive_Threshold();
		
		/* 雨滴体积换算模块死区时间更新（每10ms调用一次） */
		Raindrop_UpdateRefractory();
        
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
			/* 注意：秒计数推进在Push_Second_Count中处理，此处仅计算强度 */
			current_intensity_mmh = Compute_Intensity_MMH(); // 计算当前降雨强度
			
			/* 每10秒输出一次调试统计信息 */
			static uint8_t debug_counter = 0;
			debug_counter++;
			if (debug_counter >= 10)  // 每10秒输出一次
			{
				debug_counter = 0;
				Send_Debug_Statistics();
			}
        }
        
        /* 连续示波输出：按固定频率发送下采样后的最新ADC值（约1000点/秒） */
        Send_Live_Stream();

        /* 喂独立看门狗：必须在3秒内喂狗，否则系统自动复位（死机自启动） */
        IWDG_ReloadCounter();

        Delay_ms(10);                    // 延时10毫秒，控制循环频率，降低CPU占用率
        display_counter++;               // 显示计数器加1
        system_check_counter++;          // 系统状态计数器加1
		second_loop_counter++;           // 秒循环计数器加1

		/* 事件级死区递减：只限制计数/累计，不再限制OLED显示更新 */
		if (event_deadtime_loops > 0)
		{
			event_deadtime_loops--;
		}

		if (second_loop_counter >= 100)  // 每100次循环（1秒）
		{
			second_loop_counter = 0;     // 重置秒循环计数器
			/* 每秒推进索引：将当前秒的计数写入窗口（如果该秒没有新增，则写入0） */
			sec_index = (sec_index + 1) % SECONDS_WINDOW; // 更新秒索引（环形）
			drops_per_second[sec_index] = 0; // 重置当前秒计数（新秒开始）
		}
    }
}

/**
  * @brief  显示更新函数（合并通道）
  * @param  无
  * @retval 无
  * @note   更新OLED显示屏上的动态数据内容，显示峰值电压、主脉冲积分和窗口调试信息
  *         显示函数只刷新屏幕，不在这里做体积换算或累计
  */
void Update_Display(void)
{
	uint32_t v100 = (uint32_t)(current_voltage * 100.0f + 0.5f);

	if (v100 > 99999UL)
	{
		v100 = 99999UL;
	}

	OLED_ShowString(1, 1, "Volt:");
	OLED_ShowNum(1, 6, v100 / 100U, 3);
	OLED_ShowChar(1, 9, '.');
	OLED_ShowNum(1, 10, (v100 % 100U) / 10U, 1);
	OLED_ShowNum(1, 11, v100 % 10U, 1);
	OLED_ShowString(1, 12, " G:");
	OLED_ShowChar(1, 15, last_gain_used);

	OLED_ShowString(2, 1, "Imp:");
	OLED_ShowNum(2, 5, current_impulse_mv_us, 10);

	OLED_ShowChar(3, 1, 'W');
	OLED_ShowNum(3, 2, current_pulse_width, 3);
	OLED_ShowChar(3, 5, ' ');
	OLED_ShowChar(3, 6, 'R');
	OLED_ShowNum(3, 7, current_pulse_rise, 3);
	OLED_ShowChar(3, 10, ' ');
	OLED_ShowChar(3, 11, 'F');
	OLED_ShowNum(3, 12, current_pulse_fall, 3);

	OLED_ShowString(4, 1, current_pulse_valid ? "OK A:" : "NO A:");
	OLED_ShowNum(4, 6, current_pulse_area, 9);
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
  * @brief  独立看门狗初始化函数
  * @param  无
  * @retval 无
  * @note   配置独立看门狗，超时时间3秒，用于死机自启动
  *         一旦使能，必须在3秒内喂狗，否则系统自动复位
  */
static void IWDG_Init(void)
{
	uint32_t timeout;
	
	/* 使能IWDG写访问（允许配置预分频器和重载值） */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* 设置预分频器：64分频（LSI 32kHz / 64 = 500Hz） */
	IWDG_SetPrescaler(IWDG_PRESCALER);
	
	/* 设置重载值：1500（1500 / 500Hz = 3秒超时） */
	IWDG_SetReload(IWDG_RELOAD_VALUE);
	
	/* 等待预分频器更新完成（添加超时保护，最多等待1000次循环） */
	timeout = 1000;
	while ((IWDG_GetFlagStatus(IWDG_FLAG_PVU) != RESET) && (timeout > 0))
	{
		timeout--;
	}
	
	/* 等待重载值更新完成（添加超时保护，最多等待1000次循环） */
	timeout = 1000;
	while ((IWDG_GetFlagStatus(IWDG_FLAG_RVU) != RESET) && (timeout > 0))
	{
		timeout--;
	}
	
	/* 使能独立看门狗（一旦使能无法软件关闭） */
	IWDG_Enable();
	
	/* 立即喂一次狗，启动看门狗计数器 */
	IWDG_ReloadCounter();
}

/**
  * @brief  根据峰值大小返回动态事件级死区时间（主循环数，10ms/循环）
  * @param  peak_value: 峰值ADC值
  * @retval 死区时间（主循环数）
  * @note   小雨滴：100ms（10循环），中等雨滴：200ms（20循环），大雨滴：300ms（30循环）
  */
static uint16_t Get_Dynamic_Event_Deadtime(uint16_t peak_value)
{
	if (peak_value < PEAK_SMALL_THRESHOLD)
	{
		/* 小雨滴：100ms死区时间 */
		return 10;
	}
	else if (peak_value < PEAK_MEDIUM_THRESHOLD)
	{
		/* 中等雨滴：200ms死区时间 */
		return 20;
	}
	else
	{
		/* 大雨滴：300ms死区时间 */
		return 30;
	}
}

/**
  * @brief  根据峰值大小返回动态峰值保持时间（主循环数，10ms/循环）
  * @param  peak_value: 峰值ADC值
  * @retval 峰值保持时间（主循环数）
  * @note   小雨滴：50ms（5循环），中等雨滴：100ms（10循环），大雨滴：200ms（20循环）
  */
static uint16_t Get_Dynamic_Peak_Hold_Time(uint16_t peak_value)
{
	if (peak_value < PEAK_SMALL_THRESHOLD)
	{
		/* 小雨滴：50ms保持时间 */
		return 5;
	}
	else if (peak_value < PEAK_MEDIUM_THRESHOLD)
	{
		/* 中等雨滴：100ms保持时间 */
		return 10;
	}
	else
	{
		/* 大雨滴：200ms保持时间 */
		return 20;
	}
}

/**
  * @brief  统一处理峰值显示更新（支持快速雨滴切换显示）
  * @param  peak_value: 新的峰值ADC值
  * @retval 1: 已更新显示，0: 未更新（30ms内的后部震荡被过滤）
  * @note   基于时间的判断：30ms内的峰值需要明显更大才更新（防止后部震荡），
  *         30ms后的峰值直接显示（所有有效雨滴都能及时显示）
  */
static uint8_t Update_Peak_Display(uint16_t peak_value)
{
	/* 计算距离上次更新的时间（主循环次数，每循环10ms） */
	uint32_t time_since_last_update = (last_valid_peak == 0) ? 
		(UINT32_MAX) : (main_loop_counter - last_update_counter);
	
	/* 距离上次更新时间很短（30ms内），可能是后部震荡，需要严格过滤 */
	if (time_since_last_update <= RAPID_JUMP_TIME_THRESHOLD)
	{
		/* 只有明显更大的峰值才更新（防止后部震荡覆盖真实峰值） */
		if (peak_value > (last_valid_peak + PEAK_HOLD_MIN_DELTA))
		{
			/* 明显更大的峰值，允许更新 */
			uint16_t dynamic_hold_time = Get_Dynamic_Peak_Hold_Time(peak_value);
			
			current_peak_raw = peak_value;
			current_peak = peak_value;
			current_voltage = Compute_Voltage_From_ADC(current_peak);
			/* Volt变化时累加voltage_sum */
			voltage_sum += current_voltage;
			last_gain_used = 'H';
			last_valid_peak = peak_value;
			last_update_counter = main_loop_counter;
			peak_hold_counter = dynamic_hold_time;
			/* 输出到VOFA+ */
			USART1_SendFloat_WithTail(current_voltage);
			return 1;
		}
		/* 否则忽略（30ms内的后部震荡） */
		return 0;
	}
	
	/* 距离上次更新时间较长（超过30ms），说明是新雨滴，直接显示（无论大小） */
	uint16_t dynamic_hold_time = Get_Dynamic_Peak_Hold_Time(peak_value);
	
	/* 更新显示相关变量 */
	current_peak_raw = peak_value;
	current_peak = peak_value;
	current_voltage = Compute_Voltage_From_ADC(current_peak);
	/* Volt变化时累加voltage_sum */
	voltage_sum += current_voltage;
	last_gain_used = 'H';
	last_valid_peak = peak_value;
	last_update_counter = main_loop_counter;
	peak_hold_counter = dynamic_hold_time;
	/* 输出到VOFA+：单通道即时值 -> float32 + JustFloat尾标志 */
	USART1_SendFloat_WithTail(current_voltage);
	
	return 1;
}

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
 *         预触发处理时间计算（当前快速档ADC_SAMPLE_INTERVAL_US = 14us）：
 *         - 预触发200点：约2.8ms（从环形缓冲复制历史数据）
 *         - 后触发300点：约4.2ms（继续采集实时数据）
 *         - 总快照窗口：约7.0ms（覆盖主脉冲和早期震荡）
 *         - 触发点位置：索引200（峰值应位于此位置附近）
 *         
 *         注意：模拟看门狗触发后，DMA中断会在下一个样本处理时响应，延迟约1个采样周期。
  */
static void Process_Snapshot_IfReady(void)
{
	if (!snapshot_ready)                 // 如果快照未就绪，直接返回
	{
		return;
	}

	uint16_t len = SNAPSHOT_SIZE;
	uint16_t high_peak_idx = 0, high_peak_val = 0;
	MainPulseFeatures_t pulse_features;
	uint8_t pulse_ok = 0;
	/* 死区只保护雨量计数，OLED仍处理每一张快照，保证每滴都有Volt/Imp结果 */
	uint8_t count_allowed = (event_deadtime_loops == 0);

	int32_t baseline_high = Compute_Baseline((uint16_t *)snapshot_buffer_high, len);

	/* 严格前部处理：初始峰值搜索允许在预触发区域和前部窗口内搜索，但不搜索后部数据 */
	/* 定义前部分析窗口：触发点后的前2ms，完全忽略后部数据 */
	uint16_t front_window_start = PEAK_SEARCH_CENTER;  // 触发点索引（200）
	uint16_t front_window_end = PEAK_SEARCH_CENTER + FRONT_ANALYSIS_SAMPLES;  // 触发点后2ms
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
	pulse_ok = Extract_MainPulse_Features((uint16_t *)snapshot_buffer_high,
	                                      len,
	                                      high_peak_idx,
	                                      baseline_high,
	                                      &pulse_features);

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

	if (pulse_ok)
	{
		start_index = pulse_features.pulse_start;
		end_index = pulse_features.pulse_end;
		front_peak_index = pulse_features.peak_index;
		front_peak_value = pulse_features.peak_value;
	}

	snapshot_peak_value = front_peak_value;
	snapshot_peak_index = front_peak_index;

	/* 为切换判定单独计算全窗口峰值（避免前部窗口截断导致大雨滴峰值丢失） */
	uint16_t full_peak_index = 0;
	uint16_t full_peak_value = 0;
	Find_Peak_In_Buffer((uint16_t *)snapshot_buffer_high, len, baseline_high,
	                    &full_peak_index, &full_peak_value, 0, (uint16_t)(len - 1));

	/* PA0/PA1切换逻辑（带滞回，避免反复切换） */
	// 切换判定使用 full_peak_value（全窗口峰值），而不是仅限前部窗口的 front/active 峰值
	uint16_t switch_peak_value = full_peak_value;  // 用于切换判断的峰值（全快照范围）
	
	// 调试输出：进入切换逻辑
	USART1_SendString("=== Switch Logic ===\r\n");
	USART1_SendString("full_peak_value=");
	USART1_SendUint32(full_peak_value);
	USART1_SendString(", front_peak_value=");
	USART1_SendUint32(front_peak_value);
	USART1_SendString(", HIGH=");
	USART1_SendUint32(PA0_SWITCH_TO_PA1_HIGH);
	USART1_SendString("\r\n");
	if (pulse_ok)
	{
		USART1_SendString("Pulse: area=");
		USART1_SendUint32(pulse_features.area_pos);
		USART1_SendString(", impulse=");
		USART1_SendUint32(pulse_features.impulse_mv_us);
		USART1_SendString(", energy=");
		USART1_SendUint32(pulse_features.energy_pos);
		USART1_SendString(", rise=");
		USART1_SendUint32(pulse_features.rise_samples);
		USART1_SendString(", fall=");
		USART1_SendUint32(pulse_features.fall_samples);
		USART1_SendString(", w20=");
		USART1_SendUint32(pulse_features.width20_samples);
		USART1_SendString(", w50=");
		USART1_SendUint32(pulse_features.width50_samples);
		USART1_SendString("\r\n");
	}
	
	uint16_t final_peak_value = front_peak_value;
	uint8_t should_use_pa1 = 0;
	uint16_t pa1_raw = 0;      // PA1原始数据
	uint32_t pa1_scaled = 0;   // PA1放大后的数据
	
	// 读取PA1数据（用于判断是否有效）
	extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
	extern volatile uint16_t ring_write_index_ch1;
	extern volatile uint16_t snapshot_trigger_index_ch0;  // 快照触发时的PA0索引
	
	/* 计算PA1数据对应的环形缓冲区索引 */
	/* 关键：使用 full_peak_index（全窗口峰值索引），确保PA1读取的位置对应真正的大峰时刻 */
	/* 快照索引full_peak_index，其中SNAPSHOT_PRE_SAMPLES(200)是触发点 */
	/* 快照索引相对于触发点的偏移：full_peak_index - SNAPSHOT_PRE_SAMPLES */
	int16_t offset_from_trigger = (int16_t)full_peak_index - (int16_t)SNAPSHOT_PRE_SAMPLES;
	
	/* 计算PA0对应的环形缓冲区索引（用于验证，实际使用active_peak_index对应的PA1数据） */
	int16_t pa0_ring_index = (int16_t)snapshot_trigger_index_ch0 + offset_from_trigger;
	if (pa0_ring_index < 0)
	{
		pa0_ring_index += RING_BUFFER_SIZE;
	}
	pa0_ring_index = pa0_ring_index % RING_BUFFER_SIZE;
	
	/* PA1和PA0同步采集，索引相同（ring_write_index_ch0和ring_write_index_ch1在同一个DMA中断中同步递增） */
	/* 因此PA1对应的环形缓冲区索引与PA0相同 */
	uint16_t pa1_ring_index = (uint16_t)pa0_ring_index;
	
	pa1_raw = adc_ring_buffer_ch1[pa1_ring_index];
	
	// 调试输出：PA1数据读取信息
	USART1_SendString("PA1 Read: full_peak_idx=");
	USART1_SendUint32(full_peak_index);
	USART1_SendString(", front_peak_idx=");
	USART1_SendUint32(front_peak_index);
	USART1_SendString(", offset=");
	USART1_SendUint32((uint32_t)offset_from_trigger);
	USART1_SendString(", pa1_ring_idx=");
	USART1_SendUint32(pa1_ring_index);
	USART1_SendString(", pa1_raw=");
	USART1_SendUint32(pa1_raw);
	USART1_SendString("\r\n");
	
	// 计算PA1噪声基线（用于有效性判断，避免固定偏置误判）
	uint16_t pa1_baseline = Compute_PA1_Noise_Baseline(pa1_ring_index);
	
	// 滞回切换逻辑（优化版：只在切换点判断PA1有效性）
	// 初始化：确保pa1_scaled有明确初始值
	pa1_scaled = 0;
	should_use_pa1 = 0;
	
	// PA1连续无效计数器（用于避免单点失效导致跳变）
	static uint8_t pa1_invalid_cnt = 0;
	
	// 当前是否使用PA1（静态变量，在事件之间保持状态）
	static uint8_t use_pa1 = 0;
	
	// 调试输出：当前状态
	USART1_SendString("Current use_pa1=");
	USART1_SendUint32(use_pa1 ? 1 : 0);
	USART1_SendString("\r\n");
	
	if (!use_pa1)
	{
		// 当前使用PA0，检查是否需要切换到PA1
		// 使用active_peak_value（原始峰值）判断，而不是front_peak_value（前部窗口峰值）
		
		// 调试输出：检查是否进入切换判断
		USART1_SendString("Check Switch: switch_peak=");
		USART1_SendUint32(switch_peak_value);
		USART1_SendString(", HIGH=");
		USART1_SendUint32(PA0_SWITCH_TO_PA1_HIGH);
		USART1_SendString(", condition=");
		USART1_SendUint32((switch_peak_value >= PA0_SWITCH_TO_PA1_HIGH) ? 1 : 0);
		USART1_SendString("\r\n");
		
		if (switch_peak_value >= PA0_SWITCH_TO_PA1_HIGH)
		{
			USART1_SendString("*** ENTER SWITCH LOGIC ***\r\n");
			
			// PA0接近饱和，检查PA1是否有效（简化判断：只要PA1有合理的ADC值就切换）
			// 简化有效性判断：只要PA1的ADC值大于最小值，就认为有效
			// 因为PA1和PA0是同步采集的，如果PA0有信号，PA1也应该有对应的信号
			uint8_t pa1_valid = (pa1_raw >= PA1_MIN_VALID_VALUE);
			
			// 调试输出：切换判断信息
			USART1_SendString("Switch: switch_peak=");
			USART1_SendUint32(switch_peak_value);
			USART1_SendString(", front_peak=");
			USART1_SendUint32(front_peak_value);
			USART1_SendString(", pa1_raw=");
			USART1_SendUint32(pa1_raw);
			USART1_SendString(", pa1_baseline=");
			USART1_SendUint32(pa1_baseline);
			USART1_SendString(", valid=");
			USART1_SendUint32(pa1_valid ? 1 : 0);
			USART1_SendString("\r\n");
			
			// 如果PA1有效，切换到PA1
			if (pa1_valid)
			{
				// 使用内联函数计算PA1放大值并clamp
				pa1_scaled = Scale_Clamp_PA1(pa1_raw);
				
				if (pa1_scaled > 0 && pa1_scaled <= PA1_MAX_SCALED_VALUE)
				{
					should_use_pa1 = 1;
					pa1_invalid_cnt = 0;  // 重置无效计数器
					USART1_SendString("Switch: ->PA1, pa1_raw=");
					USART1_SendUint32(pa1_raw);
					USART1_SendString(", scaled=");
					USART1_SendUint32(pa1_scaled);
					USART1_SendString("\r\n");
				}
				else
				{
					USART1_SendString("Switch: PA1 scaled invalid, scaled=");
					USART1_SendUint32(pa1_scaled);
					USART1_SendString("\r\n");
				}
			}
			else
			{
				USART1_SendString("Switch: PA1 invalid (pa1_raw < MIN_VALID), pa1_raw=");
				USART1_SendUint32(pa1_raw);
				USART1_SendString("\r\n");
			}
		}
		else
		{
			USART1_SendString("Switch: condition not met, switch_peak < HIGH\r\n");
		}
	}
	else
	{
		// 当前使用PA1，检查是否需要切回PA0
		// 优化：不再检查PA1有效性，只看PA0是否回落（避免单点噪声导致跳变）
		// 使用active_peak_value（原始峰值）判断，而不是front_peak_value（前部窗口峰值）
		if (switch_peak_value <= PA0_SWITCH_TO_PA1_LOW)
		{
			// PA0已回落，切回PA0
			should_use_pa1 = 0;
			pa1_invalid_cnt = 0;  // 重置无效计数器
		}
		else
		{
			// 继续使用PA1（不再检查PA1有效性，避免单点噪声导致跳变）
			// 使用内联函数重新计算PA1放大值用于显示
			pa1_scaled = Scale_Clamp_PA1(pa1_raw);
			
			// 连续失效保险：如果PA1连续多次失效，才回退PA0（避免单点失效导致跳变）
			if (pa1_scaled == 0)
			{
				pa1_invalid_cnt++;
			}
			else
			{
				pa1_invalid_cnt = 0;  // 重置无效计数器
			}
			
			if (pa1_invalid_cnt >= PA1_INVALID_COUNT_THRESHOLD)
			{
				// 连续3次无效，切回PA0
				should_use_pa1 = 0;
				pa1_invalid_cnt = 0;
			}
			else if (pa1_scaled > 0 && pa1_scaled <= PA1_MAX_SCALED_VALUE)
			{
				should_use_pa1 = 1;
			}
			else
			{
				// PA1放大后超出范围，切回PA0
				should_use_pa1 = 0;
			}
		}
	}
	
	// 保险规则：只要切到PA1，就强制本轮必须成功算出pa1_scaled，否则回退PA0
	if (should_use_pa1)
	{
		if (pa1_scaled == 0 || pa1_scaled > PA1_MAX_SCALED_VALUE)
		{
			// pa1_scaled未正确计算，强制回退PA0
			should_use_pa1 = 0;
		}
	}
	
	// 应用切换结果
	use_pa1 = should_use_pa1;
	
	// 调试输出：切换结果
	USART1_SendString("Result: should_use_pa1=");
	USART1_SendUint32(should_use_pa1 ? 1 : 0);
	USART1_SendString(", use_pa1=");
	USART1_SendUint32(use_pa1 ? 1 : 0);
	USART1_SendString(", pa1_scaled=");
	USART1_SendUint32(pa1_scaled);
	USART1_SendString("\r\n");
	
	if (use_pa1)
	{
		// 确保pa1_scaled已正确计算
		final_peak_value = (pa1_scaled > 65535) ? 65535 : (uint16_t)pa1_scaled;
		last_gain_used = 'L';  // 'L'表示PA1通道（低增益，用于大信号）
		// 调试输出：成功切换到PA1
		USART1_SendString("*** PA1 Active *** switch_peak=");
		USART1_SendUint32(switch_peak_value);
		USART1_SendString(", front_peak=");
		USART1_SendUint32(front_peak_value);
		USART1_SendString(", pa1_raw=");
		USART1_SendUint32(pa1_raw);
		USART1_SendString(", pa1_scaled=");
		USART1_SendUint32(pa1_scaled);
		USART1_SendString(", final=");
		USART1_SendUint32(final_peak_value);
		USART1_SendString("\r\n");
	}
	else
	{
		final_peak_value = front_peak_value;
		last_gain_used = 'H';  // 'H'表示PA0高增益通道
		// 调试输出：使用PA0（如果PA0值较高）
		if (front_peak_value >= 3500)
		{
			USART1_SendString("PA0 Active: peak=");
			USART1_SendUint32(front_peak_value);
			USART1_SendString(" (high, no switch)\r\n");
		}
	}
	
	USART1_SendString("==================\r\n");

	if (count_allowed &&
	    Validate_And_Count_Event(active_buffer, end_index + 1, front_peak_index, front_peak_value, threshold, start_index, end_index))
	{
		snapshot_valid_count++;

		// ========== 小波特征提取（触发后二次分析，第3阶段集成） ==========
		// 执行顺序（必须严格遵守）：
		// 1. 算 baseline
		// 2. 算 win_start（带前沿约束 + 边界）
		// 3. Wavelet_ExtractFeatures（得到 success/hf_ratio）
		// 4. Determine_Event_Type（得到 event_type，第3阶段）
		// 5. 更新类型统计计数器（cnt_rain_clean, cnt_rain_fast, cnt_vib, cnt_emi, cnt_bg, cnt_bad）
		// 6. 设置 event_deadtime_loops（按类型分级：VIB×3, EMI×4）
		// 7. 原计数/原雨量累计逻辑（第4阶段：raw_event_count + effective_drop_count）
		
		uint8_t wavelet_done = 0;
		WaveletFeatures_t wavelet_features;
		EventType_t event_type = EVT_BAD;  // 默认值，小波失败时使用
		
		// 1. 计算wavelet_baseline（从预触发环形缓冲区，避开触发前已抬升的前沿）
		// 使用snapshot_trigger_index_ch0作为触发点参考
		extern volatile uint16_t snapshot_trigger_index_ch0;
		int32_t wavelet_baseline = Compute_Wavelet_Baseline(snapshot_trigger_index_ch0);
		
		// 2. 选择win_start（推荐：围绕峰值，更稳定）
		// 统一采用：peak_index - 16 作为小波分析窗口起点
		// 主参考：peak_index - 16
		uint16_t win_start = (front_peak_index >= 16) ? (front_peak_index - 16) : 0;
		
		// 前沿约束：确保窗口贴近事件起点
		#define MAX_FRONT_SHIFT  24  // 允许窗口起点最多比start_index晚24点
		if (win_start < (start_index > 8 ? start_index - 8 : 0))
		{
			win_start = (start_index > 8 ? start_index - 8 : 0);
		}
		if (win_start > start_index + MAX_FRONT_SHIFT)
		{
			win_start = start_index + MAX_FRONT_SHIFT;
		}
		
		// 边界处理：确保能取满64点
		if (win_start + WAVELET_WINDOW_SIZE > end_index + 1)
		{
			win_start = (end_index + 1 >= WAVELET_WINDOW_SIZE) ? 
			            (end_index + 1 - WAVELET_WINDOW_SIZE) : 0;
		}
		
		// 3. 提取小波特征
		if (Wavelet_ExtractFeatures(active_buffer, end_index + 1, 
		                            win_start, wavelet_baseline, 
		                            &wavelet_features))
		{
			wavelet_done = wavelet_features.success;
			// 特征可用于VOFA输出或本地记录（当前不上传，仅用于离线建模）
			// 未来扩展协议再上传特征字段
		}
		
		// 4. 确定事件类型（第3阶段，更细化的类型划分）
		event_type = Determine_Event_Type(&wavelet_features);
		
		// 4.5. 输出调试信息：每滴雨的hf_ratio和事件类型（通过USART1）
		if (wavelet_done)
		{
			// 输出格式：Event: hf=xxx, type=xxx, peak=xxx\r\n
			USART1_SendString("Event: hf=");
			USART1_SendUint32((uint32_t)wavelet_features.hf_ratio);
			
			USART1_SendString(", type=");
			const char *type_str = "UNK";
			if (event_type == EVT_RAIN_CLEAN) type_str = "RAIN_CLEAN";
			else if (event_type == EVT_RAIN_FAST) type_str = "RAIN_FAST";
			else if (event_type == EVT_VIB) type_str = "VIB";
			else if (event_type == EVT_EMI) type_str = "EMI";
			else if (event_type == EVT_BG) type_str = "BG";
			else if (event_type == EVT_BAD) type_str = "BAD";
			USART1_SendString(type_str);
			
			USART1_SendString(", peak=");
			USART1_SendUint32((uint32_t)front_peak_value);
			USART1_SendString("\r\n");
		}
		
		// 5. 更新类型统计计数器（第3阶段）
		if (event_type == EVT_RAIN_CLEAN)
		{
			cnt_rain_clean++;
		}
		else if (event_type == EVT_RAIN_FAST)
		{
			cnt_rain_fast++;
		}
		else if (event_type == EVT_VIB)
		{
			cnt_vib++;
		}
		else if (event_type == EVT_EMI)
		{
			cnt_emi++;
		}
		else if (event_type == EVT_BG)
		{
			cnt_bg++;
		}
		else if (event_type == EVT_BAD)
		{
			cnt_bad++;
		}
		
		// 兼容第2阶段的计数器（保留）
		EventClass_t event_class = Determine_Event_Class(&wavelet_features);
		if (event_class == EVENT_CLASS_GOOD)
		{
			wavelet_good_count++;
		}
		else if (event_class == EVENT_CLASS_SUSPECT)
		{
			wavelet_suspect_count++;
		}
		else if (event_class == EVENT_CLASS_BAD)
		{
			wavelet_bad_count++;
		}
		
		// 6. 设置动态死区（按类型分级，第3阶段）
		uint16_t base_deadtime = Get_Dynamic_Event_Deadtime(front_peak_value);
		if (event_type == EVT_VIB)
		{
			// VIB：延长死区（×3，loops口径）
			event_deadtime_loops = base_deadtime * DT_SCALE_VIB_NUM / DT_SCALE_VIB_DEN;
		}
		else if (event_type == EVT_EMI)
		{
			// EMI：延长死区（×4，loops口径）
			event_deadtime_loops = base_deadtime * DT_SCALE_EMI_NUM / DT_SCALE_EMI_DEN;
		}
		else
		{
			// RAIN_CLEAN / RAIN_FAST / BG / BAD：保持原值（不动）
			event_deadtime_loops = base_deadtime;
		}
		
		// 7. 原计数/原雨量累计逻辑（第4阶段：raw_event_count + effective_drop_count）
		// raw_event_count：所有通过时域验证的事件都计数
		raw_event_count++;
		
		/* 雨滴体积换算：对所有通过Validate_And_Count_Event的事件都进行体积换算 */
		/* 根据使用的通道选择正确的ADC峰值 */
		uint16_t adc_peak_for_volume;
		if (use_pa1)
		{
			/* PA1通道：使用原始ADC值pa1_raw（未放大） */
			adc_peak_for_volume = pa1_raw;
		}
		else
		{
			/* PA0通道：使用front_peak_value */
			adc_peak_for_volume = front_peak_value;
		}
		
		/* 直接计算体积并累加（绕过死区时间，确保每次事件都累加） */
		/* 先计算电压（mV） */
		float voltage_V = Compute_Voltage_From_ADC(adc_peak_for_volume);
		uint32_t U_meas_mV = (uint32_t)(voltage_V * 1000.0f + 0.5f);
		
		/* 电压转体积 */
		uint8_t status_flag;
		uint32_t volume_0p01mm3 = Raindrop_VoltageToVolume_0p01mm3(U_meas_mV, &status_flag);
		
		/* 如果体积有效（电压>=360mV），直接累加到Sum */
		if (status_flag == RAIN_STATUS_OK && volume_0p01mm3 > 0)
		{
			Raindrop_AddVolume(volume_0p01mm3);
		}
		
		/* 也调用Raindrop_ProcessOneDrop用于死区时间管理（但不依赖它来累加） */
		Raindrop_ProcessOneDrop(adc_peak_for_volume);
		
		// effective_drop_count：根据事件类型决定是否计入雨量
		// 软否决规则（保守策略：宁可放过可疑雨滴，也不误加明显干扰）
		uint8_t should_count_rain = 0;  // 是否计入雨量
		if (event_type == EVT_RAIN_CLEAN || event_type == EVT_RAIN_FAST)
		{
			// 雨滴类：计入雨量
			should_count_rain = 1;
		}
		// EVT_VIB / EVT_EMI / EVT_BG / EVT_BAD：不计入雨量
		
		if (should_count_rain)
		{
			effective_drop_count++;
			drop_count++;                    // 雨滴计数加1（保持兼容）
			total_rain_mm += g_mm_per_drop;  // 累计降雨量增加（使用可配置参数）
			/* 将本秒计数+1 */
			if (sec_index < SECONDS_WINDOW)  // 如果索引在有效范围内
			{
				drops_per_second[sec_index] += 1; // 当前秒雨滴数加1
			}
		}
		
		// TF通信上传（非阻塞，队列入队）
		// 注意：src_id参数已删除，SRC_ID字段统一使用编译时宏TF_SRC_ID
		// ENERGY计算：使用pulse_samples明确样本数口径
		uint16_t pulse_samples = end_index - start_index + 1;  // 用于ENERGY的样本数口径
		uint32_t energy = pulse_ok ? pulse_features.area_pos :
			((uint32_t)front_peak_value * (uint32_t)pulse_samples);
		TF_Comm_SendEvent(1, front_peak_value, energy, 
		                 (use_pa1 ? 1 : 0), wavelet_done);
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
	/* 改进：排除异常峰值，避免干扰影响阈值计算 */
	/* 先计算均值，然后排除明显异常值（超过均值+3*MAD的样本） */
	sum = 0;
	start = (int16_t)ring_write_index_ch0 - NOISE_WINDOW;
	if (start < 0) start += RING_BUFFER_SIZE;
	for (i = 0; i < NOISE_WINDOW; i++)
	{
		sum += adc_ring_buffer_ch0[(start + i) % RING_BUFFER_SIZE];
	}
	mean_times_1 = sum / (int32_t)NOISE_WINDOW;
	
	/* 计算MAD用于识别异常值 */
	mad_sum = 0;
	for (i = 0; i < NOISE_WINDOW; i++)
	{
		int32_t v = (int32_t)adc_ring_buffer_ch0[(start + i) % RING_BUFFER_SIZE];
		int32_t d = v - mean_times_1;
		if (d < 0) d = -d;
		mad_sum += d;
	}
	int32_t mad_pre = mad_sum / (int32_t)NOISE_WINDOW;
	
	/* 排除异常值后重新计算均值 */
	sum = 0;
	uint16_t valid_count = 0;
	int32_t outlier_threshold = mean_times_1 + (int32_t)(3 * mad_pre);
	for (i = 0; i < NOISE_WINDOW; i++)
	{
		int32_t v = (int32_t)adc_ring_buffer_ch0[(start + i) % RING_BUFFER_SIZE];
		/* 排除明显异常值（可能是干扰峰值） */
		if (v <= outlier_threshold)
		{
			sum += v;
			valid_count++;
		}
	}
	if (valid_count > 0)
	{
		mean_times_1 = sum / (int32_t)valid_count;
	}

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
	if (target < MIN_THRESHOLD) target = MIN_THRESHOLD; // 限制最小值400mV
	if (target > MAX_THRESHOLD) target = MAX_THRESHOLD;

	if ((int32_t)dynamic_threshold - target > HYSTERESIS_MARGIN ||
		target - (int32_t)dynamic_threshold > HYSTERESIS_MARGIN)
	{
		dynamic_threshold = (uint16_t)target;
		AD_SetThreshold(dynamic_threshold); // 设置ADC模拟看门狗阈值（通道0/PA0）
	}
}

/**
  * @brief  对缓冲区进行移动平均滤波，减少ADC量化噪声
  * @param  buf: 输入缓冲区
  * @param  smoothed: 输出平滑后的缓冲区
  * @param  len: 缓冲区长度
  * @param  start_idx: 起始索引
  * @param  end_idx: 结束索引
  * @retval 无
  */
static void Smooth_Filter(uint16_t *buf, uint16_t *smoothed, uint16_t len, uint16_t start_idx, uint16_t end_idx)
{
	uint16_t i;
	uint16_t half_window = SMOOTH_FILTER_SIZE / 2;
	
	for (i = start_idx; i <= end_idx && i < len; i++)
	{
		uint32_t sum = 0;
		uint16_t count = 0;
		
		/* 计算移动平均：取当前点及其前后各half_window个点 */
		int16_t win_start = (int16_t)i - (int16_t)half_window;
		int16_t win_end = (int16_t)i + (int16_t)half_window;
		
		/* 限制窗口范围在有效区间内 */
		if (win_start < (int16_t)start_idx) win_start = (int16_t)start_idx;
		if (win_end > (int16_t)end_idx) win_end = (int16_t)end_idx;
		if (win_end >= (int16_t)len) win_end = (int16_t)len - 1;
		
		for (int16_t j = win_start; j <= win_end; j++)
		{
			if (j >= 0 && j < (int16_t)len)
			{
				sum += buf[j];
				count++;
			}
		}
		
		if (count > 0)
		{
			smoothed[i] = (uint16_t)(sum / count);
		}
		else
		{
			smoothed[i] = buf[i];
		}
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
	
	/* 0) 信号平滑滤波：减少ADC量化噪声，让信号特征更明显 */
	/* 使用静态数组避免动态分配，函数局部静态变量只分配一次 */
	static uint16_t smoothed_buf[SNAPSHOT_SIZE];
	uint16_t smooth_start = (start_index > 0) ? (start_index - 1) : 0;
	uint16_t smooth_end = (end_index < len - 1) ? (end_index + 1) : (len - 1);
	
	/* 对有效区间进行平滑滤波 */
	Smooth_Filter(buf, smoothed_buf, len, smooth_start, smooth_end);
	
	/* 使用平滑后的数据进行后续判定 */
	uint16_t *filtered_buf = smoothed_buf;
	
	/* 重新计算平滑后的峰值（在峰值附近搜索） */
	uint16_t filtered_peak_value = filtered_buf[peak_index];
	uint16_t filtered_peak_index = peak_index;
	for (i = (peak_index > 2) ? (peak_index - 2) : 0; 
	     i <= ((peak_index + 2 < len) ? (peak_index + 2) : (len - 1)); 
	     i++)
	{
		if (filtered_buf[i] > filtered_peak_value)
		{
			filtered_peak_value = filtered_buf[i];
			filtered_peak_index = i;
		}
	}
	
	/* 更新使用平滑后的峰值和缓冲区 */
	peak_value = filtered_peak_value;
	peak_index = filtered_peak_index;
	buf = filtered_buf;

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

	/* 3) 时间特征判定：区分真实雨滴信号和噪声干扰 */
	/* 真实信号：上升→峰值→下降有一定时间，噪声：快速毛刺 */
	
	/* 3.1 快速过滤明显毛刺：脉冲宽度太窄直接判定为噪声 */
	uint16_t pulse_width_samples = end_index - start_index + 1;
	if (pulse_width_samples <= MAX_NOISE_PULSE_WIDTH)
	{
		/* 脉冲宽度小于等于10个采样点（约420us），判定为噪声毛刺 */
		return 0;
	}
	
	/* 3.2 计算上升时间：从开始到峰值的时间 */
	uint16_t rise_samples = (peak_index > start_index) ? (peak_index - start_index) : 1;
	float rise_time_us = (float)rise_samples * ADC_SAMPLE_INTERVAL_US;
	
	/* 3.3 计算下降时间：从峰值到结束的时间 */
	uint16_t fall_samples = (end_index > peak_index) ? (end_index - peak_index) : 1;
	float fall_time_us = (float)fall_samples * ADC_SAMPLE_INTERVAL_US;
	
	/* 3.4 计算总脉冲持续时间 */
	float pulse_duration_us = (float)pulse_width_samples * ADC_SAMPLE_INTERVAL_US;
	
	/* 3.5 时间特征判定：真实信号必须有足够的上升时间、下降时间和总持续时间 */
	/* 最小时间限制：防止快速毛刺噪声 */
	if (rise_time_us < MIN_RISE_TIME_US)
	{
		/* 上升时间太短，可能是快速毛刺噪声 */
		return 0;
	}
	
	if (fall_time_us < MIN_FALL_TIME_US)
	{
		/* 下降时间太短，可能是快速毛刺噪声 */
		return 0;
	}
	
	if (pulse_duration_us < MIN_PULSE_DURATION_US)
	{
		/* 总持续时间太短，可能是快速毛刺噪声 */
		return 0;
	}
	
	/* 3.6 波形对称性判定：雨滴波形相对对称，机械振动可能不对称 */
	/* 计算上升/下降时间比 */
	float rise_fall_ratio = (fall_time_us > 0.0f) ? (rise_time_us / fall_time_us) : 0.0f;
	if (rise_fall_ratio < RISE_FALL_TIME_RATIO_MIN || rise_fall_ratio > RISE_FALL_TIME_RATIO_MAX)
	{
		/* 上升/下降时间比不在合理范围内，可能是机械振动 */
		return 0;
	}
	
	/* 3.7 上升下降平滑度判定：真实信号相对平滑，干扰可能很陡峭 */
	/* 计算上升过程中的平滑上升比例和波动次数 */
	uint16_t smooth_rise_count = 0;
	uint16_t continuous_rise_count = 0;  // 连续上升计数
	uint16_t max_continuous_rise = 0;    // 最大连续上升长度
	uint16_t rise_fluctuation_count = 0; // 上升过程中的波动次数（方向改变）
	uint16_t total_rise_samples = (peak_index > start_index) ? (peak_index - start_index) : 1;
	if (total_rise_samples > 1)
	{
		uint8_t prev_trend = 0;  // 0=未知，1=上升，2=下降
		for (i = start_index + 1; i <= peak_index; i++)
		{
			uint16_t diff = (buf[i] > buf[i - 1]) ? (buf[i] - buf[i - 1]) : 0;
			/* 平滑上升：差值不超过最大陡峭斜率 */
			if (diff > 0 && diff <= MAX_STEEP_SLOPE)
			{
				smooth_rise_count++;
				continuous_rise_count++;
				if (continuous_rise_count > max_continuous_rise)
				{
					max_continuous_rise = continuous_rise_count;
				}
				/* 检测趋势变化 */
				if (prev_trend == 2)  // 从下降转为上升
				{
					rise_fluctuation_count++;
				}
				prev_trend = 1;
			}
			else if (buf[i] < buf[i - 1])
			{
				continuous_rise_count = 0;  // 重置连续计数
				/* 检测趋势变化 */
				if (prev_trend == 1)  // 从上升转为下降
				{
					rise_fluctuation_count++;
				}
				prev_trend = 2;
			}
			else
			{
				continuous_rise_count = 0;  // 重置连续计数
			}
		}
	}
	float smooth_rise_ratio = (total_rise_samples > 0) ? 
		((float)smooth_rise_count / (float)total_rise_samples) : 0.0f;
	
	/* 计算下降过程中的平滑下降比例和波动次数 */
	uint16_t smooth_fall_count = 0;
	uint16_t continuous_fall_count = 0;  // 连续下降计数
	uint16_t max_continuous_fall = 0;     // 最大连续下降长度
	uint16_t fall_fluctuation_count = 0;  // 下降过程中的波动次数（方向改变）
	uint16_t total_fall_samples = (end_index > peak_index) ? (end_index - peak_index) : 1;
	if (total_fall_samples > 1)
	{
		uint8_t prev_trend = 0;  // 0=未知，1=上升，2=下降
		for (i = peak_index + 1; i <= end_index; i++)
		{
			uint16_t diff = (buf[i - 1] > buf[i]) ? (buf[i - 1] - buf[i]) : 0;
			/* 平滑下降：差值不超过最大陡峭斜率 */
			if (diff > 0 && diff <= MAX_STEEP_SLOPE)
			{
				smooth_fall_count++;
				continuous_fall_count++;
				if (continuous_fall_count > max_continuous_fall)
				{
					max_continuous_fall = continuous_fall_count;
				}
				/* 检测趋势变化 */
				if (prev_trend == 1)  // 从上升转为下降
				{
					fall_fluctuation_count++;
				}
				prev_trend = 2;
			}
			else if (buf[i] > buf[i - 1])
			{
				continuous_fall_count = 0;  // 重置连续计数
				/* 检测趋势变化 */
				if (prev_trend == 2)  // 从下降转为上升
				{
					fall_fluctuation_count++;
				}
				prev_trend = 1;
			}
			else
			{
				continuous_fall_count = 0;  // 重置连续计数
			}
		}
	}
	float smooth_fall_ratio = (total_fall_samples > 0) ? 
		((float)smooth_fall_count / (float)total_fall_samples) : 0.0f;
	
	/* 增强判定：要求有足够的连续上升/下降，确保信号有明确的趋势 */
	/* 根据信号幅度动态调整要求：小雨滴信号可能连续性稍弱 */
	uint16_t min_continuous_required;
	if (peak_value > 650)  // 大于650 ADC单位（约520mV），要求更严格
	{
		min_continuous_required = (total_rise_samples > 5) ? 3 : 2;
	}
	else  // 小雨滴信号（420-540mV），要求放宽
	{
		min_continuous_required = 1;  // 至少连续1个样本即可，降低要求
	}
	
	/* 只在样本数足够多时才检查连续性，避免误判小雨滴 */
	if (max_continuous_rise < min_continuous_required && total_rise_samples > 5)
	{
		/* 上升过程缺乏连续性，可能是噪声（但样本数要足够多才判定） */
		return 0;
	}
	if (max_continuous_fall < min_continuous_required && total_fall_samples > 5)
	{
		/* 下降过程缺乏连续性，可能是噪声（但样本数要足够多才判定） */
		return 0;
	}
	
	/* 平滑度判定：真实信号应该有足够的平滑上升和下降 */
	if (smooth_rise_ratio < MIN_SMOOTH_RISE_RATIO)
	{
		/* 上升过程不够平滑，可能是陡峭干扰 */
		return 0;
	}
	
	if (smooth_fall_ratio < MIN_SMOOTH_FALL_RATIO)
	{
		/* 下降过程不够平滑，可能是陡峭干扰 */
		return 0;
	}
	
	/* 3.7 峰值稳定性判定：真实信号峰值相对稳定，干扰峰值可能波动大 */
	uint16_t peak_stable_count = 0;
	uint16_t peak_check_start = (peak_index > PEAK_STABILITY_WINDOW) ? 
		(peak_index - PEAK_STABILITY_WINDOW) : start_index;
	uint16_t peak_check_end = (peak_index + PEAK_STABILITY_WINDOW < end_index) ? 
		(peak_index + PEAK_STABILITY_WINDOW) : end_index;
	
	for (i = peak_check_start; i <= peak_check_end; i++)
	{
		uint16_t diff = (peak_value > buf[i]) ? (peak_value - buf[i]) : (buf[i] - peak_value);
		/* 峰值附近样本应该接近峰值（在容差范围内） */
		if (diff <= PEAK_STABILITY_DELTA)
		{
			peak_stable_count++;
		}
	}
	
	/* 峰值稳定性判定：峰值附近至少应该有部分样本接近峰值 */
	uint16_t peak_check_samples = peak_check_end - peak_check_start + 1;
	float peak_stability_ratio = (peak_check_samples > 0) ? 
		((float)peak_stable_count / (float)peak_check_samples) : 0.0f;
	
	/* 根据信号幅度动态调整峰值稳定性要求：小雨滴信号可能峰值稳定性稍弱 */
	float min_stability_ratio;
	if (peak_value > 650)  // 大于650 ADC单位（约520mV），要求更严格
	{
		min_stability_ratio = 0.3f;  // 30%
	}
	else  // 小雨滴信号，要求放宽
	{
		min_stability_ratio = 0.2f;  // 20%，降低要求
	}
	
	if (peak_stability_ratio < min_stability_ratio)
	{
		/* 峰值不够稳定，可能是孤立尖峰干扰 */
		return 0;
	}

	/* 4) 通过所有判定：确认为真实雨滴信号 */
	return 1;                            // 返回有效事件
}

/**
  * @brief  推送秒计数到窗口
  * @param  drops_in_second: 当前秒的雨滴数
  * @retval 无
  * @note   将当前秒的雨滴数推送到统计窗口
  */
/* 注意：Push_Second_Count函数已废弃，秒计数推进逻辑已移至主循环 */
/* 保留函数声明以避免编译错误，但不再使用 */
static void Push_Second_Count(uint16_t drops_in_second)
{
	/* 函数已废弃：秒计数推进逻辑已移至主循环的second_loop_counter处理 */
	(void)drops_in_second;           // 避免未使用参数警告
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
	/* 60秒内滴数 -> 每小时：* (3600/60) = *60；每滴折算mm（使用可配置参数） */
	return (float)sum * g_mm_per_drop * 60.0f / (float)SECONDS_WINDOW; // 计算降雨强度
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

/**
  * @brief  计算PA1噪声基线（事件前的窗口）
  * @param  pa1_ring_index: PA1当前环形缓冲区索引
  * @retval PA1噪声基线值
  * @note   取事件前的32点均值作为噪声基线，用于有效性判断
  */
static uint16_t Compute_PA1_Noise_Baseline(uint16_t pa1_ring_index)
{
	extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
	uint32_t sum = 0;
	
	for (int k = 0; k < PA1_BASELINE_WINDOW; k++)
	{
		int idx = (int)pa1_ring_index - 1 - k;
		if (idx < 0) idx += RING_BUFFER_SIZE;
		idx = idx % RING_BUFFER_SIZE;
		sum += adc_ring_buffer_ch1[idx];
	}
	
	return (uint16_t)(sum / PA1_BASELINE_WINDOW);
}

/**
  * @brief  PA1放大并clamp（内联函数，减少重复代码）
  * @param  raw: PA1原始ADC值
  * @retval 放大后的值（已clamp到0-65535）
  * @note   统一处理放大和溢出保护，避免重复代码
  */
static inline uint32_t Scale_Clamp_PA1(uint16_t raw)
{
	float x = (float)raw * PA1_AMPLIFICATION_FACTOR;
	if (x <= 0.0f) return 0;
	if (x >= 65535.0f) return 65535;
	return (uint32_t)(x + 0.5f);  // 四舍五入
}

/**
  * @brief  从ADC值计算电压（应用校准系数）
  * @param  adc_value: ADC原始值
  * @retval 计算后的电压值（伏特）
  * @note   统一应用校准系数，便于与示波器测量值对齐
  */
static float Compute_Voltage_From_ADC(uint16_t adc_value)
{
	// 统一使用校准系数计算电压
	return ((float)adc_value / (float)ADC_FULL_SCALE * ADC_REF_VOLTAGE) * ADC_VOLTAGE_CALIBRATION_FACTOR;
}

/**
  * @brief  计算小波baseline（从预触发环形缓冲区，避开触发前已抬升的前沿）
  * @param  trigger_index: 触发点在环形缓冲区中的索引
  * @retval baseline值（int32_t）
  * @note   使用中位数方法（推荐，工程最不容易翻车）
  */
static int32_t Compute_Wavelet_Baseline(uint16_t trigger_index)
{
	extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
	const uint16_t N = 32;  // 取触发点前32点
	
	// 从环形缓冲区取触发点前N点
	uint16_t samples[N];
	int16_t start_idx = (int16_t)trigger_index - N;
	if (start_idx < 0)
	{
		start_idx += RING_BUFFER_SIZE;
	}
	
	for (uint16_t i = 0; i < N; i++)
	{
		samples[i] = adc_ring_buffer_ch0[(start_idx + i) % RING_BUFFER_SIZE];
	}
	
	// 中位数计算（简单排序后取中间值）
	// 冒泡排序
	for (uint16_t i = 0; i < N - 1; i++)
	{
		for (uint16_t j = 0; j < N - 1 - i; j++)
		{
			if (samples[j] > samples[j + 1])
			{
				uint16_t temp = samples[j];
				samples[j] = samples[j + 1];
				samples[j + 1] = temp;
			}
		}
	}
	
	// 返回中位数（N为偶数时取中间两个的平均值）
	return (int32_t)((samples[N/2 - 1] + samples[N/2]) / 2);
}

/* 根据小波特征确定事件类别（第2阶段） */
static EventClass_t Determine_Event_Class(WaveletFeatures_t *features)
{
	// energy_total==0已在Wavelet模块内归到success=0，这里只需判断success
	if (features->success == 0)
	{
		return EVENT_CLASS_BAD;
	}
	
	if (features->hf_ratio > WAVELET_THRESHOLD_T1)
	{
		return EVENT_CLASS_SUSPECT;
	}
	else
	{
		return EVENT_CLASS_GOOD;
	}
}

/* 根据小波特征确定事件类型（第3阶段，更细化的类型划分） */
static EventType_t Determine_Event_Type(WaveletFeatures_t *features)
{
	// BAD：窗口/能量异常，退回时域
	if (features->success == 0)
	{
		return EVT_BAD;
	}
	
	uint16_t hf_ratio = features->hf_ratio;
	
	// EMI：电噪声毛刺类（>600）
	if (hf_ratio >= HF_EMI_MIN)
	{
		return EVT_EMI;
	}
	
	// VIB：振动/风扰/敲击类（>450）
	if (hf_ratio >= HF_VIB_MIN)
	{
		return EVT_VIB;
	}
	
	// RAIN_FAST：高速小滴/飞溅（250~400）
	// 注意：250~300会同时落在CLEAN和FAST的交叠区，按"更保守的策略"先判CLEAN
	if (hf_ratio >= HF_RAIN_LOW && hf_ratio <= HF_SPLASH_HIGH)
	{
		// 如果>=250，优先判FAST；否则判CLEAN
		if (hf_ratio >= 250)
		{
			return EVT_RAIN_FAST;
		}
		else
		{
			return EVT_RAIN_CLEAN;
		}
	}
	
	// RAIN_CLEAN：单滴、干净（150~300）
	if (hf_ratio >= HF_RAIN_LOW && hf_ratio <= HF_RAIN_HIGH)
	{
		return EVT_RAIN_CLEAN;
	}
	
	// BG：背景漂移/无效（<100）
	if (hf_ratio < HF_BG_MAX)
	{
		return EVT_BG;
	}
	
	// 默认情况（100~150之间，很少见）：归为BG
	return EVT_BG;
}

static uint16_t Clamp_U16_From_I32(int32_t value)
{
	if (value <= 0)
	{
		return 0;
	}
	if (value > 65535)
	{
		return 65535;
	}
	return (uint16_t)value;
}

static uint16_t Clamp_ADC_Delta_From_I32(int32_t value)
{
	uint16_t delta = Clamp_U16_From_I32(value);

	if (delta > ADC_FULL_SCALE)
	{
		return (uint16_t)ADC_FULL_SCALE;
	}
	return delta;
}

static uint32_t Integrate_MainPulse_mV_us(uint16_t *buf, uint16_t start_index,
                                          uint16_t end_index, int32_t baseline)
{
	uint16_t i;
	uint16_t prev_delta;
	uint64_t pair_sum_adc = 0;
	uint64_t numerator;
	uint64_t denominator;
	uint64_t impulse_mv_us;

	if (buf == 0 || end_index <= start_index)
	{
		return 0;
	}

	prev_delta = Clamp_ADC_Delta_From_I32((int32_t)buf[start_index] - baseline);
	for (i = (uint16_t)(start_index + 1U); i <= end_index; i++)
	{
		uint16_t cur_delta = Clamp_ADC_Delta_From_I32((int32_t)buf[i] - baseline);

		pair_sum_adc += (uint64_t)prev_delta + (uint64_t)cur_delta;
		prev_delta = cur_delta;
	}

	/* 梯形积分：sum((V[i-1] + V[i]) / 2 * dt)，先累加ADC差值，最后一次性换成mV*us。 */
	numerator = pair_sum_adc * ADC_REF_MV * ADC_VOLTAGE_CALIBRATION_NUM * (uint64_t)ADC_SAMPLE_INTERVAL_NS;
	denominator = (uint64_t)ADC_FULL_SCALE * ADC_VOLTAGE_CALIBRATION_DEN * 2000ULL;
	impulse_mv_us = (numerator + (denominator / 2ULL)) / denominator;
	if (impulse_mv_us > 0xFFFFFFFFULL)
	{
		return 0xFFFFFFFFUL;
	}
	return (uint32_t)impulse_mv_us;
}

static void Update_MainPulse_Display_State(MainPulseFeatures_t *features)
{
	uint16_t width = 0;

	if (features == 0 || features->valid == 0)
	{
		return;
	}

	current_pulse_valid = 1;
	current_pulse_area = (features->area_pos > 999999999UL) ? 999999999UL : features->area_pos;
	current_impulse_mv_us = features->impulse_mv_us;
	if (features->pulse_end >= features->pulse_start)
	{
		width = (uint16_t)(features->pulse_end - features->pulse_start + 1U);
	}
	current_pulse_width = width;
	current_pulse_rise = features->rise_samples;
	current_pulse_fall = features->fall_samples;
}

static uint8_t Build_MainPulse_Features_From_Segment(uint16_t *buf, uint16_t len,
                                                     uint16_t start_index,
                                                     uint16_t end_index,
                                                     uint16_t peak_index,
                                                     int32_t baseline,
                                                     MainPulseFeatures_t *features)
{
	uint16_t i;
	uint16_t threshold20;
	uint16_t threshold50;

	if (features == 0 || buf == 0 || len == 0)
	{
		return 0;
	}
	if (start_index >= len)
	{
		return 0;
	}
	if (end_index >= len)
	{
		end_index = (uint16_t)(len - 1);
	}
	if (start_index > end_index)
	{
		return 0;
	}

	features->valid = 0;
	features->win_start = start_index;
	features->win_end = end_index;
	features->pulse_start = start_index;
	features->pulse_end = end_index;
	features->peak_index = (peak_index >= start_index && peak_index <= end_index) ? peak_index : start_index;
	features->peak_value = buf[features->peak_index];
	features->peak_delta = 0;
	features->rise_samples = 0;
	features->fall_samples = 0;
	features->width20_samples = 0;
	features->width50_samples = 0;
	features->area_pos = 0;
	features->impulse_mv_us = 0;
	features->energy_pos = 0;

	for (i = start_index; i <= end_index; i++)
	{
		if (buf[i] > features->peak_value)
		{
			features->peak_value = buf[i];
			features->peak_index = i;
		}
	}

	features->peak_delta = Clamp_U16_From_I32((int32_t)features->peak_value - baseline);
	if (features->peak_delta < MIN_PEAK_DELTA_OVER_THR)
	{
		return 0;
	}

	features->rise_samples = (features->peak_index > features->pulse_start) ?
		(uint16_t)(features->peak_index - features->pulse_start) : 1;
	features->fall_samples = (features->pulse_end > features->peak_index) ?
		(uint16_t)(features->pulse_end - features->peak_index) : 1;

	threshold20 = (uint16_t)(((uint32_t)features->peak_delta * 20U) / 100U);
	threshold50 = (uint16_t)(((uint32_t)features->peak_delta * 50U) / 100U);
	features->impulse_mv_us = Integrate_MainPulse_mV_us(buf,
	                                                    features->pulse_start,
	                                                    features->pulse_end,
	                                                    baseline);

	for (i = features->pulse_start; i <= features->pulse_end; i++)
	{
		int32_t delta_i32 = (int32_t)buf[i] - baseline;
		uint16_t delta = Clamp_U16_From_I32(delta_i32);

		features->area_pos += delta;
		features->energy_pos += (uint32_t)delta * (uint32_t)delta;
		if (delta >= threshold20)
		{
			features->width20_samples++;
		}
		if (delta >= threshold50)
		{
			features->width50_samples++;
		}
	}

	if ((uint16_t)(features->pulse_end - features->pulse_start + 1U) < MAIN_PULSE_MIN_WIDTH)
	{
		return 0;
	}

	features->valid = 1;
	return 1;
}

static uint8_t Extract_MainPulse_Features(uint16_t *buf, uint16_t len,
                                          uint16_t anchor_peak_index,
                                          int32_t baseline,
                                          MainPulseFeatures_t *features)
{
	uint16_t i;
	uint8_t settle;
	uint16_t threshold_near_base;
	uint16_t threshold20;
	uint16_t threshold50;

	if (features == 0 || buf == 0 || len == 0 || anchor_peak_index >= len)
	{
		return 0;
	}

	threshold_near_base = Clamp_U16_From_I32(baseline + MAIN_PULSE_BASELINE_DELTA);

	features->valid = 0;
	features->win_start = (anchor_peak_index > MAIN_PULSE_PRE_SAMPLES) ?
		(uint16_t)(anchor_peak_index - MAIN_PULSE_PRE_SAMPLES) : 0;
	features->win_end = (anchor_peak_index + MAIN_PULSE_POST_SAMPLES < len) ?
		(uint16_t)(anchor_peak_index + MAIN_PULSE_POST_SAMPLES) : (uint16_t)(len - 1);
	features->pulse_start = features->win_start;
	features->pulse_end = features->win_end;
	features->peak_index = anchor_peak_index;
	features->peak_value = buf[anchor_peak_index];
	features->peak_delta = 0;
	features->rise_samples = 0;
	features->fall_samples = 0;
	features->width20_samples = 0;
	features->width50_samples = 0;
	features->area_pos = 0;
	features->impulse_mv_us = 0;
	features->energy_pos = 0;

	for (i = features->win_start; i <= features->win_end; i++)
	{
		if (buf[i] > features->peak_value)
		{
			features->peak_value = buf[i];
			features->peak_index = i;
		}
	}

	features->peak_delta = Clamp_U16_From_I32((int32_t)features->peak_value - baseline);
	if (features->peak_delta < MIN_PEAK_DELTA_OVER_THR)
	{
		return 0;
	}

	settle = 0;
	i = features->peak_index;
	while (i > features->win_start)
	{
		if (buf[i] <= threshold_near_base)
		{
			if (settle < 255)
			{
				settle++;
			}
			if (settle >= MAIN_PULSE_SETTLE_COUNT)
			{
				features->pulse_start = (uint16_t)(i + MAIN_PULSE_SETTLE_COUNT);
				break;
			}
		}
		else
		{
			settle = 0;
		}
		i--;
	}
	if (features->pulse_start > features->peak_index)
	{
		features->pulse_start = features->win_start;
	}

	settle = 0;
	for (i = features->peak_index; i <= features->win_end; i++)
	{
		if (buf[i] <= threshold_near_base)
		{
			if (settle < 255)
			{
				settle++;
			}
			if (settle >= MAIN_PULSE_SETTLE_COUNT)
			{
				features->pulse_end = (i >= MAIN_PULSE_SETTLE_COUNT) ?
					(uint16_t)(i - MAIN_PULSE_SETTLE_COUNT) : features->peak_index;
				break;
			}
		}
		else
		{
			settle = 0;
		}
	}
	if (features->pulse_end <= features->peak_index)
	{
		features->pulse_end = features->peak_index;
	}

	features->rise_samples = (features->peak_index > features->pulse_start) ?
		(uint16_t)(features->peak_index - features->pulse_start) : 1;
	features->fall_samples = (features->pulse_end > features->peak_index) ?
		(uint16_t)(features->pulse_end - features->peak_index) : 1;

	threshold20 = (uint16_t)(((uint32_t)features->peak_delta * 20U) / 100U);
	threshold50 = (uint16_t)(((uint32_t)features->peak_delta * 50U) / 100U);
	features->impulse_mv_us = Integrate_MainPulse_mV_us(buf,
	                                                    features->pulse_start,
	                                                    features->pulse_end,
	                                                    baseline);

	for (i = features->pulse_start; i <= features->pulse_end; i++)
	{
		int32_t delta_i32 = (int32_t)buf[i] - baseline;
		uint16_t delta = Clamp_U16_From_I32(delta_i32);

		features->area_pos += delta;
		features->energy_pos += (uint32_t)delta * (uint32_t)delta;

		if (delta >= threshold20)
		{
			features->width20_samples++;
		}
		if (delta >= threshold50)
		{
			features->width50_samples++;
		}
	}

	if ((uint16_t)(features->pulse_end - features->pulse_start + 1) < MAIN_PULSE_MIN_WIDTH)
	{
		return 0;
	}

	features->valid = 1;
	return 1;
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

/**
  * @brief  配置USART1（PA10=TX, PA9=RX, 115200 8N1）
  * @note   仅用于向VOFA+发送单通道浮点波形数据
  */
static void USART1_Config(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    /* 开启时钟：GPIOA 与 USART1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9 -> USART1_TX: 复用推挽输出 50MHz（USART1默认映射） */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 -> USART1_RX: 上拉输入 */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  发送单字节（阻塞方式）
  */
static void USART1_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        /* 等待发送缓冲空 */
    }
    USART_SendData(USART1, b);
}

/**
  * @brief  发送字符串（调试用）
  * @param  str: 要发送的字符串（以'\0'结尾）
  */
static void USART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART1_SendByte(*str);
        str++;
    }
}

/**
  * @brief  发送uint32_t数字（调试用）
  * @param  val: 要发送的数字
  */
static void USART1_SendUint32(uint32_t val)
{
    char num_str[16];
    uint8_t idx = 0;
    
    if (val == 0)
    {
        num_str[idx++] = '0';
    }
    else
    {
        // 先计算位数
        uint32_t temp = val;
        uint8_t digits = 0;
        while (temp > 0)
        {
            temp /= 10;
            digits++;
        }
        // 从后往前填充
        idx = digits;
        temp = val;
        while (temp > 0)
        {
            num_str[--idx] = '0' + (temp % 10);
            temp /= 10;
        }
        idx = digits;
    }
    num_str[idx] = '\0';
    USART1_SendString(num_str);
}

/**
  * @brief  发送float并追加JustFloat尾标志 {00 00 80 7F}
  * @note   VOFA+ JustFloat引擎：仅需要在数据后追加尾标志即可，按小端发送
  */
static void USART1_SendFloat_WithTail(float v)
{
    const uint8_t jf_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    union
    {
        float f;
        uint8_t b[4];
    } u;
    u.f = v;

    /* 发送float小端字节 */
    USART1_SendByte(u.b[0]);
    USART1_SendByte(u.b[1]);
    USART1_SendByte(u.b[2]);
    USART1_SendByte(u.b[3]);

    /* 发送JustFloat结束标志 */
    USART1_SendByte(jf_tail[0]);
    USART1_SendByte(jf_tail[1]);
    USART1_SendByte(jf_tail[2]);
    USART1_SendByte(jf_tail[3]);
}

/**
  * @brief  连续下采样输出ADC值，提供VOFA+示波数据流
  * @note   目标速率约1000点/秒：每次主循环(10ms)最多发送10点
  */
static void Send_Live_Stream(void)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t ring_write_index_ch0;

    /* 每次调用发送的最大点数；主循环10ms调用一次 -> 10点/10ms ≈ 1000点/s */
    const uint8_t max_points = 10;

    static uint16_t last_index = 0;

    uint16_t write_idx = ring_write_index_ch0;
    uint16_t available;
    if (write_idx >= last_index)
    {
        available = write_idx - last_index;
    }
    else
    {
        available = (uint16_t)(RING_BUFFER_SIZE - last_index + write_idx);
    }

    /* 限制发送数量，避免带宽溢出 */
    uint16_t to_send = (available > max_points) ? max_points : available;
    for (uint16_t i = 0; i < to_send; i++)
    {
        uint16_t idx = (last_index + i) % RING_BUFFER_SIZE;
        uint16_t raw = adc_ring_buffer_ch0[idx];
        float v = Compute_Voltage_From_ADC(raw);
        USART1_SendFloat_WithTail(v);
    }

    last_index = (uint16_t)((last_index + to_send) % RING_BUFFER_SIZE);
}

/**
  * @brief  发送调试统计信息（每10秒调用一次）
  * @note   通过USART1输出所有计数器和统计信息，用于定位小波分析问题
  */
static void Send_Debug_Statistics(void)
{
    USART1_SendString("=== Stats ===\r\n");
    
    USART1_SendString("raw_event_count=");
    USART1_SendUint32(raw_event_count);
    USART1_SendString("\r\n");
    
    USART1_SendString("effective_drop_count=");
    USART1_SendUint32(effective_drop_count);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_rain_clean=");
    USART1_SendUint32(cnt_rain_clean);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_rain_fast=");
    USART1_SendUint32(cnt_rain_fast);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_vib=");
    USART1_SendUint32(cnt_vib);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_emi=");
    USART1_SendUint32(cnt_emi);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_bg=");
    USART1_SendUint32(cnt_bg);
    USART1_SendString("\r\n");
    
    USART1_SendString("cnt_bad=");
    USART1_SendUint32(cnt_bad);
    USART1_SendString("\r\n");
    
    // 计算漏计率和误判统计
    if (raw_event_count > 0)
    {
        uint32_t missed = raw_event_count - effective_drop_count;
        USART1_SendString("missed_count=");
        USART1_SendUint32(missed);
        USART1_SendString("\r\n");
        
        USART1_SendString("miss_rate=");
        uint32_t percent = (missed * 100) / raw_event_count;
        USART1_SendUint32(percent);
        USART1_SendString("%\r\n");
        
        // 误判统计：被误判为VIB/EMI/BG/BAD的真实雨滴
        uint32_t false_negative = cnt_vib + cnt_emi + cnt_bg + cnt_bad;
        USART1_SendString("false_negative=");
        USART1_SendUint32(false_negative);
        USART1_SendString("\r\n");
    }
    
    USART1_SendString("===========\r\n");
}
