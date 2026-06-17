// 包含必要的头文件
#include "stm32f10x.h"                  // STM32F10x微控制器标准外设库头文件
#include "Delay.h"                       // 延时函数头文件
#include "OLED.h"                        // OLED显示屏驱动头文件
#include "AD.h"                          // ADC模数转换器驱动头文件
#include "stm32f10x_usart.h"             // 串口通信头文件
#include "stm32f10x_gpio.h"              // GPIO口操作头文件
#include "stm32f10x_rcc.h"               // 时钟控制头文件
#include "stm32f10x_iwdg.h"              // 独立看门狗头文件（用于死机自启动）
#include "SPI_SlaveLink.h"               // SPI 从机链路层（SPI1 + EXTI4）
#include "RainAreaBuffer.h"
#include "Wavelet.h"                     // 小波变换模块头文件
#include "raindrop.h"                    // 雨滴体积换算模块头文件
#include "RainDisplay.h"                 // ILI9341 2.4寸TFT显示模块

/* LCD总开关：=1 启用TFT显示（与OLED并存，不影响雨滴计量逻辑）。 */
#define LCD_ENABLE                       1
/* LCD自检开关：=1 上电后画一次静态界面+假波形+假数值（仅调试接线用）；
 * 接真实数据时保持0。 */
#define LCD_SELFTEST                     0

/* ===================== SPI 从站地址配置（类Modbus帧协议） ===================== */
/* 修改此宏即可为本单片机设置从站地址（1~247），与主控配置一致 */
#define MODBUS_SLAVE_ID    1

#define RAIN_CALIB_UART_ENABLE           1
#define ENABLE_ISR_FAST_DISPLAY          1

// ========== 系统参数定义 ==========
/* 电压校准系数（方便修改） */
#define ADC_VOLTAGE_CALIBRATION_FACTOR  1.0f     // 电压不再全局缩小，按3.3V参考值换算
#define ADC_VOLTAGE_CALIBRATION_NUM     1UL      // 同一校准系数的整数分子，用于积分计算
#define ADC_VOLTAGE_CALIBRATION_DEN     1UL      // 同一校准系数的整数分母：1/1=1.0
#define ADC_REF_MV                      3300UL   // ADC参考电压，单位mV

/* PA1通道配置：用于PA0削顶时切换（等效测量大于3.3V的雨滴）
 * 切换策略为按事件（每张快照）独立判定，无需滞回。
 * 硬件放大倍数尚未确定：等效比优先用同一事件中PA0未削顶的上升段在线估计
 * （两通道同步采样，削顶前的样本对天然就是标定数据），估计失败时退回
 * 保守默认值；等效结果统一钳位在 [EVENT_EQ_MIN_MV, EVENT_EQ_MAX_MV]，
 * 保证切换后读数高于3.3V削顶值、又不会因比值误差而离谱。 */
#define PA1_VALID_MIN_MV          40UL     // PA1原始峰值下限（mV）：PA1是源头小信号(PA0=PA1×放大倍数)，削顶时实测约55mV，故下限降到40mV
#define PA1_VALID_MAX_MV          450UL    // PA1原始峰值上限（mV），高于此值置PA1_HIGH标志（仍切换，靠上限钳位兜底）
#define PA1_EQ_RATIO_DEFAULT_X100 300U     // 在线估计失败时的默认等效比（暂保守值，拿到实测放大倍数后再调）
#define PA1_EQ_RATIO_MIN_X100     150U     // 等效比下限（1.5倍）
#define PA1_EQ_RATIO_MAX_X100     10000U   // 等效比上限（100倍）：PA0=PA1放大几十倍，上限放宽以免把真实大倍数钳死
#define PA1_RATIO_PA0_MIN_DELTA   300U     // 参与比值估计的PA0样本最小幅度（ADC码，约240mV）
#define PA1_RATIO_PA1_MIN_DELTA   6U       // 参与比值估计的PA1样本最小幅度（ADC码，约5mV）：大放大倍数下PA1很小，门限太高会取不到样本对
#define EVENT_EQ_MIN_MV           3400UL   // 等效电压下限：PA0已削顶，真实峰值必高于3.3V
#define EVENT_EQ_MAX_MV           5000UL   // 等效电压上限：实测最大雨滴约4.xV，钳到5V留余量
#define EVENT_CLIP_MV             3300UL
#define PA0_CLIP_ADC              4090U
#define PA0_CLIP_COUNT_TH         3U

/* PA0过量程"平顶"检测（用于切换到PA1） ============================================
 * 真实饱和的大雨滴，PA0顶部会被压平——连续若干样本都贴在峰值附近（顶部可能停在
 * 3.2~3.3V，未必顶到旧的4090硬削顶阈值，所以旧判据几乎不触发）。
 * 判据：全快照最大值 >= PA0_OVERRANGE_GATE_ADC，且"贴近峰值（峰值-PA0_PLATEAU_BAND_ADC
 * 以内）的最长连续样本数" >= PA0_PLATEAU_COUNT_TH，即认定PA0过量程，触发PA1接管。
 * 这样顶部停在3.2V也能识别；而真正"圆顶"、未饱和的中等雨滴顶部不平，不会被误判
 * （误判会被Build_Event_Result钳到≥3.4V而读大）。三个值都可对照示波器现场整定：
 *   门限调低→更早切PA1（但中等雨滴误判风险↑）；带宽调小/点数调大→更严格只认真平顶。 */
#define PA0_OVERRANGE_GATE_MV     2800U    // 过量程电压门限(mV)，约2.8V：峰值低于此不当作过量程
#define PA0_OVERRANGE_GATE_ADC    ((uint16_t)(((uint32_t)PA0_OVERRANGE_GATE_MV * (uint32_t)ADC_FULL_SCALE) / ADC_REF_MV))  // ≈3475 ADC
#define PA0_PLATEAU_BAND_ADC      25U      // 平顶带宽(ADC码,约20mV)：样本与峰值之差≤此值算"贴顶"（放宽以容纳饱和纹波）
#define PA0_PLATEAU_COUNT_TH      5U       // 连续"贴顶"样本数下限(约70us@14us采样)，达到即判为平顶饱和

#define EVENT_RESULT_SOURCE_PA0   0U
#define EVENT_RESULT_SOURCE_PA1   1U
#define EVENT_RESULT_SOURCE_CLIP  2U
#define EVENT_RESULT_FLAG_VALID       0x0001U
#define EVENT_RESULT_FLAG_PA0_CLIPPED 0x0002U
#define EVENT_RESULT_FLAG_PA1_VALID   0x0004U
#define EVENT_RESULT_FLAG_PA1_LOW     0x0008U
#define EVENT_RESULT_FLAG_PA1_HIGH    0x0010U
#define EVENT_RESULT_FLAG_EQ_LIMIT    0x0020U

#define THRESHOLD 496                     // 初始阈值（ADC单位，400mV = 400/3.3*4095 ≈ 496）

/* 增益配置：CH0=高增益、CH1=低增益（默认≈15倍 vs 1倍，可按需调整） */
#define HIGH_GAIN_FACTOR         15.0f
#define LOW_GAIN_FACTOR          1.0f
#define HIGH_GAIN_SAT_THRESHOLD  4000     // 高增益ADC达到该值视为饱和（接近3.3V）
#define HIGH_GAIN_SAT_COUNT_TH   3        // PA0连续达到饱和阈值的样本数，达到后本事件改用PA1积分
#define ADC_FULL_SCALE           4095     // ADC满量程值（12位ADC，明确写死为4095，防止未来换ADC分辨率出坑）
#define ADC_REF_VOLTAGE          3.3f

/* 自适应阈值相关 */
#define NOISE_WINDOW            200      // 噪声统计窗口长度（从环形缓冲末端向前取）
#define MIN_THRESHOLD           496       // 阈值下限，防止过低（400mV = 400/3.3*4095 ≈ 496）
#define MAX_THRESHOLD           3000     // 阈值上限，防止过高
#define MAD_GAIN                3        // 平均绝对偏差放大倍数
#define HYSTERESIS_MARGIN       15       // 阈值滞回，降低抖动

/* ===================== 测试用：固定采集阈值（约2V） =====================
 * 临时把"采集雨滴的阈值"抬高到约2V用于测试。
 *   - RAIN_TEST_FIXED_THRESHOLD_ENABLE = 1：跳过噪声自适应，把动态阈值钉死在
 *     RAIN_TEST_FIXED_THRESHOLD_MV 对应的ADC码上（同时作用于硬件模拟看门狗触发
 *     与快照事件验证两处）。
 *   - 测试完成后把 RAIN_TEST_FIXED_THRESHOLD_ENABLE 改回 0，即可完全恢复原自适应
 *     阈值逻辑（MIN_THRESHOLD~MAX_THRESHOLD）。只需改本宏，无需改动其它代码。 */
#define RAIN_TEST_FIXED_THRESHOLD_ENABLE   1        // =1 启用固定阈值（测试）；=0 恢复自适应阈值
#define RAIN_TEST_FIXED_THRESHOLD_MV       2000U    // 固定阈值对应电压（mV），≈2V
#define RAIN_TEST_FIXED_THRESHOLD_ADC \
	((uint16_t)(((uint32_t)RAIN_TEST_FIXED_THRESHOLD_MV * (uint32_t)ADC_FULL_SCALE) / ADC_REF_MV))  // 2000mV → ≈2481 ADC码

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

/* LCD波形显示门限（测试用，可调）：只有本张快照PA0实采绝对峰值≥此电压时才刷新LCD波形。
 * 与采集阈值独立——高干扰阶段可单独调高此值来滤掉小信号波形，不影响计数与OLED。
 * 测试完按需改回（设0即不过滤、每张快照都刷）。 */
#define LCD_WAVE_MIN_MV         2000U    // LCD波形显示电压门限(mV)，约2V
#define LCD_WAVE_MIN_ADC        ((uint16_t)(((uint32_t)LCD_WAVE_MIN_MV * (uint32_t)ADC_FULL_SCALE) / ADC_REF_MV))  // 2000mV ≈ 2481 ADC码

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
#define FRONT_ANALYSIS_SAMPLES   ((uint16_t)(FRONT_ANALYSIS_TIME_MS * 1000.0f / ADC_SAMPLE_INTERVAL_US))  // 前部分析采样点数（14us采样下约142点）

#define US_TO_SAMPLES_CEIL(us) \
	((uint16_t)((((uint32_t)(us) * 1000UL) + ADC_SAMPLE_INTERVAL_NS - 1UL) / ADC_SAMPLE_INTERVAL_NS))

/* 主脉冲积分窗口：覆盖完整主脉冲（不再截断）。
 * POST放宽到~4.5ms，让"脉冲结束"由信号自身衰减决定，而非固定上限。
 * 结束判定改为绝对值包络：|信号-基线|连续 SETTLE_COUNT 点落在 ±BASELINE_DELTA 带内，
 * 这样负半周不会被误判为结束，积分覆盖整个振荡串。delta/count为现场可调经验值。 */
#define MAIN_PULSE_PRE_US          500U
#define MAIN_PULSE_POST_US         4500U
#define MAIN_PULSE_PRE_SAMPLES     US_TO_SAMPLES_CEIL(MAIN_PULSE_PRE_US)
#define MAIN_PULSE_POST_SAMPLES    US_TO_SAMPLES_CEIL(MAIN_PULSE_POST_US)
#define MAIN_PULSE_BASELINE_DELTA  25U     /* 包络settle带宽（ADC码，约20mV） */
#define MAIN_PULSE_SETTLE_COUNT    8U      /* 连续落带内点数（约112us）才算结束 */
#define MAIN_PULSE_MIN_WIDTH       8U

/* 峰值大小分类阈值（用于动态调整参数） */
#define PEAK_SMALL_THRESHOLD    600      // 小雨滴阈值（ADC单位，约480mV）
#define PEAK_MEDIUM_THRESHOLD   1500     // 中等雨滴阈值（ADC单位，约1.2V）

/* 事件级死区：在快照层面避免同一滴的重复计数（单位：主循环次数，10ms/次）
 * 实际值由 Get_Dynamic_Event_Deadtime 按峰值动态给出，上限 EVENT_DEADTIME_MAX_LOOPS */

/* 小波分析相关参数（第2阶段，保留兼容） */
#define WAVELET_THRESHOLD_T1    300      // 初始阈值（permille，0-1000），用于触发动态死区，不用于否决
#define DEADTIME_SUSPECT_SCALE_NUM   2   // SUSPECT时死区倍数（分子）
#define DEADTIME_SUSPECT_SCALE_DEN   1   // SUSPECT时死区倍数（分母）
#define DEADTIME_SUSPECT_ADD_LOOPS   0   // 如果要加固定量，用 loops 口径写死
#define EVENT_DEADTIME_MAX_LOOPS     12U

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
 * 行3显示格式：Dxxxx Vxxx.xx（滴数/最近一滴体积，单位mm³）
 * 行4显示格式：Txxxxxxx.xx（累计体积，单位mm³）
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
#if ENABLE_ISR_FAST_DISPLAY
static uint16_t last_valid_peak = 0;    // 上一次有效的峰值（用于峰值保持）
#endif
static uint32_t peak_hold_counter = 0;   // 峰值保持计数器
#define PEAK_HOLD_TIME_MS    200         // 峰值保持时间（毫秒），200ms内只显示更大的峰值，确保快速连续雨滴仍能检测（将被动态时间替代）
#define PEAK_HOLD_MIN_DELTA  200         // 新峰值必须比旧峰值大至少200个ADC单位才更新（约160mV，中等以上雨滴）
#define PEAK_HOLD_MIN_DELTA_SMALL  100   // 小雨滴的峰值更新阈值（约80mV），支持快速切换显示
#define PEAK_HOLD_MIN_RATIO  0.7f        // 新峰值必须大于旧峰值的70%才更新（仅在保持时间内生效，防止后部震荡误判）

/* 快速跳变时间过滤：如果新峰值明显小于旧峰值，且距离上次更新时间很短，直接忽略 */
#if ENABLE_ISR_FAST_DISPLAY
static uint32_t last_update_counter = 0;          // 最近一次峰值更新的主循环计数
#endif
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

typedef struct
{
	uint16_t source;
	uint8_t valid_flag;
	uint16_t flags;
	uint32_t old_pa0_peak_mv;
	uint32_t old_pa0_impulse_mv_us;
	uint32_t peak_mv;
	uint32_t impulse_mv_us;
	uint32_t pa1_peak_mv;
	uint32_t pa1_impulse_mv_us;
} RainEventResult_t;

// ========== 函数声明 ==========
void Update_Display(void);               // 显示更新函数声明
void Check_System_Status(void);          // 系统状态检查函数声明
static void IWDG_Init(void);             // 独立看门狗初始化函数声明
static void Process_Snapshot_IfReady(void);  // 处理触发快照（预触发200点+后触发300点）
static void Update_Adaptive_Threshold(void); // 计算噪声并自适应阈值
static uint8_t Validate_And_Count_Event(uint16_t *buf, uint16_t len, uint16_t peak_index, uint16_t peak_value, uint16_t threshold, uint16_t start_index, uint16_t end_index); // 验证并计数事件
static float Compute_Intensity_MMH(void); // 计算降雨强度（mm/h）
static int32_t Compute_Baseline(uint16_t *buf, uint16_t len);
static int32_t Compute_Wavelet_Baseline(void); // 计算小波baseline（用快照预触发段中位数）
static float Compute_Voltage_From_ADC(uint16_t adc_value);  // 从ADC值计算电压（应用校准系数）
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
static uint32_t Clamp_U64_To_U32(uint64_t value);
static uint32_t ADC_Count_To_mV(uint32_t adc_value);
static uint32_t ADC_us_To_mV_us(uint32_t adc_us);
static uint16_t Count_Consecutive_AtOrAbove(uint16_t *buf, uint16_t len, uint16_t threshold);
static RainEventResult_t Build_Event_Result(uint8_t pa0_clipped,
                                            uint32_t pa0_peak_mv,
                                            uint32_t pa0_impulse_mv_us,
                                            uint32_t pa1_raw_peak_mv,
                                            uint32_t pa1_eq_peak_mv,
                                            uint32_t pa1_eq_impulse_mv_us);
static uint16_t Count_Max_Consecutive_Saturation(uint16_t *buf, uint16_t len);
static uint8_t Detect_PA0_Plateau(uint16_t *buf, uint16_t len); // PA0平顶饱和检测（过量程→切PA1）
static void Integrate_Delta_Window_ADC_us(uint16_t *buf, uint16_t start_index,
                                          uint16_t end_index, int32_t baseline,
                                          uint32_t *area_samples,
                                          uint32_t *raw_integral_adc_us,
                                          uint32_t *scaled_integral_adc_us);
static uint16_t Estimate_PA1_Eq_Ratio_X100(uint16_t *high_buf, uint16_t *low_buf,
                                           uint16_t start_index, uint16_t end_index,
                                           int32_t baseline_high, int32_t baseline_low);
static uint32_t Integrate_MainPulse_mV_us(uint16_t *buf, uint16_t start_index,
                                          uint16_t end_index, int32_t baseline);
#if ENABLE_ISR_FAST_DISPLAY
static void Update_MainPulse_Display_State(MainPulseFeatures_t *features);
#endif

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
static void USART1_Config(void);          // 配置USART1用于标定事件输出
static void USART1_SendByte(uint8_t b);   // 发送单字节
static void USART1_SendString(const char *str);  // 发送字符串（标定用）
static void USART1_SendUint32(uint32_t val);     // 发送uint32_t数字（标定用）
static void Rain_CalibUart_SendEvent(uint32_t drop_id,
                                     uint32_t integral,
                                     uint32_t vol_0p01mm3,
                                     uint32_t total_0p01mm3,
                                     uint16_t peak,
                                     uint16_t width,
                                     uint16_t rise,
                                     uint16_t fall);
static uint16_t Get_Dynamic_Event_Deadtime(uint16_t peak_value); // 根据峰值大小返回动态事件级死区时间（主循环数）
static uint16_t Get_Dynamic_Peak_Hold_Time(uint16_t peak_value); // 根据峰值大小返回动态峰值保持时间（主循环数）
#if ENABLE_ISR_FAST_DISPLAY
static uint8_t Update_Peak_Display(uint16_t peak_value); // 统一处理峰值显示更新，返回1表示已更新，0表示未更新
#endif

/* 触发与统计变量（PA0为触发/计量主通道，PA1快照同步保存用于削顶诊断与接管） */
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

/* 累计雨量（按"滴数×g_mm_per_drop"折算的旧口径估计，仅调试参考；
 * 正式计量以Raindrop累计体积为准） */
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

/* 原始事件与有效滴数（SPI寄存器0x0042~0x0045对外可读） */
volatile uint32_t raw_event_count = 0;       // 原始事件数：每张被处理的触发快照计一次（未必通过验证）
volatile uint32_t effective_drop_count = 0;  // 有效滴数：通过验证并完成体积累计的事件（正式口径）

/* 近60秒滴数窗口 - 在中断中使用 */
volatile uint16_t drops_per_second[SECONDS_WINDOW] = {0}; // 每秒雨滴数数组
volatile uint8_t sec_index = 0;            // 当前秒索引
static uint16_t second_loop_counter = 0; // 10ms循环累加到100为1秒

/* OLED显示缓存 */
float current_intensity_mmh = 0.0f;  // 当前降雨强度（毫米/小时），供Modbus从站读取
char  last_gain_used = 'H';         // 最近一次使用的增益通道（'H'或'L'），供Modbus从站读取
volatile uint32_t watchdog_trigger_count = 0; // 模拟看门狗触发次数
volatile uint32_t snapshot_valid_count = 0;   // 验证通过次数

volatile uint16_t dbg_event_source_channel = 0;
volatile uint16_t dbg_event_max_sat_count = 0;
volatile uint16_t dbg_event_pa1_delta = 0;
volatile uint32_t dbg_event_raw_integral_adc_us = 0;
volatile uint32_t dbg_event_scaled_integral_adc_us = 0;
volatile uint16_t dbg_event_gain_x100 = 0;
volatile uint16_t dbg_event_flags = 0;
volatile uint32_t dbg_event_relaxed_count = 0;  // 经第二层核心闸门计入的事件数（严格形状验证未过）
volatile uint32_t dbg_event_reject_count = 0;   // 两层判定都未通过、被拒绝的触发快照数
volatile uint16_t dbg_live_pa0_max = 0;
volatile uint16_t dbg_live_pa1_max = 0;
volatile uint16_t dbg_live_pa0_sat_run = 0;
volatile uint16_t dbg_live_pa0_sat_max = 0;
/* 平顶检测诊断（每张快照更新，用于现场整定门限/带宽/点数） */
volatile uint16_t dbg_pa0_abs_max = 0;       // PA0全快照实采绝对峰值（ADC码）
volatile uint16_t dbg_pa0_plateau_run = 0;   // PA0贴顶最长连续样本数
static RainEventResult_t shadow_result = {0};

#if LCD_ENABLE
/* TFT波形显示：只在"有效滴(被计数)"时画一帧(从export_buffer, 21ms全分辨率)，画完保持
 * 在屏上直到下一滴——平时不重画，所以波形不会被随后的噪声触发覆盖、也不会一闪而过。 */
static volatile uint8_t lcd_draw_pending = 0;
static uint16_t lcd_wave_baseline = 0;
static uint16_t lcd_wave_threshold = 0;
static uint16_t lcd_wave_peak_index = 0;
static uint8_t  lcd_wave_pa1 = 0;
#endif

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
#if RAIN_TEST_FIXED_THRESHOLD_ENABLE
    /* 测试：上电即把采集阈值固定到约2V（与Update_Adaptive_Threshold保持一致） */
    dynamic_threshold = RAIN_TEST_FIXED_THRESHOLD_ADC;
    AD_SetThreshold(RAIN_TEST_FIXED_THRESHOLD_ADC);
#else
    AD_SetThreshold(THRESHOLD);          // 设置模拟看门狗阈值
#endif
    USART1_Config();                     // 初始化USART1串口（PA9=TX，PA10=RX，115200 8N1）
    RainAreaBuffer_Init();
    /* SPI 从站链路层初始化（SPI1，PA4=NSS，PA5=SCK，PA6=MISO，PA7=MOSI） */
    SPI_SlaveLink_Init(MODBUS_SLAVE_ID);
    Raindrop_Init();                     // 初始化雨滴体积换算模块

#if RAIN_CALIB_UART_ENABLE
    /* 开机横幅：上电立即从USART1输出，便于确认串口链路通、固件版本对、关键参数值。
     * 字段：BOOT,采集阈值ADC,平顶过量程门限ADC,平顶带宽,平顶点数阈值,LCD波形门限ADC */
    USART1_SendString("\r\n=== RAIN FW plateau+diag ===\r\n");
    USART1_SendString("BOOT,collectThrADC=");
    USART1_SendUint32(dynamic_threshold);
    USART1_SendString(",overGateADC=");
    USART1_SendUint32((uint32_t)PA0_OVERRANGE_GATE_ADC);
    USART1_SendString(",plateauBand=");
    USART1_SendUint32((uint32_t)PA0_PLATEAU_BAND_ADC);
    USART1_SendString(",plateauCnt=");
    USART1_SendUint32((uint32_t)PA0_PLATEAU_COUNT_TH);
    USART1_SendString(",lcdMinADC=");
    USART1_SendUint32((uint32_t)LCD_WAVE_MIN_ADC);
    USART1_SendString("\r\n");
#endif

    // ========== 显示静态内容 ==========
	OLED_ShowString(1, 1, "Volt:000.00 G:H");
	OLED_ShowString(2, 1, "Imp:0000000000");
	OLED_ShowString(3, 1, "D0000 V000.00");
	OLED_ShowString(4, 1, "T0000000.00");
    
#if LCD_ENABLE
    /* TFT初始化：清屏+画标题栏/波形框/网格 */
    RainDisplay_Init();
#if LCD_SELFTEST
    /* 自检：画一次假波形+假数值，仅用于调试接线 */
    RainDisplay_SelfTest();
#endif
#endif

    // ========== 独立看门狗初始化（在所有初始化完成后） ==========
    IWDG_Init();                         // 初始化独立看门狗，超时时间3秒，用于死机自启动
    IWDG_ReloadCounter();                // 立即喂一次狗，确保看门狗计数器从最大值开始
    
    // ========== 主循环 ==========
    while (1)                            // 程序主要逻辑
    {
		main_loop_counter++;             // 主循环计数器递增（每10ms递增一次）
		/* ISR峰值快路径：仅用于OLED快速显示（Volt/Imp）；
		 * 正式计数、体积累计、事件入队、EVT输出统一由快照路径完成 */
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
			    isr_pulse_ok &&
			    isr_pulse_features.peak_delta >= DISPLAY_MIN_AMPLITUDE)
			{
#if ENABLE_ISR_FAST_DISPLAY
				if (Update_Peak_Display(isr_pulse_features.peak_delta))
				{
					Update_MainPulse_Display_State(&isr_pulse_features);
				}
#endif
			}
		}
		
		/* 峰值保持计数器递减 */
		if (peak_hold_counter > 0)
		{
			peak_hold_counter--;
		}

        /* 处理快照数据（如果就绪）：用于精确的事件验证和计数 */
        Process_Snapshot_IfReady();

#if LCD_ENABLE
		/* 有效滴时画一帧(export_buffer, 21ms全分辨率)，画完保持在屏上直到下一滴：
		 * 平时不重画 → 不被随后的噪声触发覆盖，也不会一闪而过。 */
		if (lcd_draw_pending)
		{
			lcd_draw_pending = 0;
			RainDisplay_DrawWaveform((uint16_t *)export_buffer, SNAPSHOT_SIZE,
			                         lcd_wave_baseline, lcd_wave_threshold,
			                         lcd_wave_peak_index, lcd_wave_pa1);
		}
#endif

		/* 自适应阈值（基于最近噪声） */
		Update_Adaptive_Threshold();
        
        // 每200ms更新一次显示(20次 × 10ms = 200ms)
        if (display_counter >= 20)       // 检查显示计数器是否达到20
        {
            Update_Display();            // OLED显示更新（保留）
#if LCD_ENABLE
            /* TFT数值区：与OLED同源数据，单位 V / mV·us / 滴 / mm³ / mm³ / mm/h */
            RainDisplay_UpdateStats(current_voltage, current_impulse_mv_us,
                                    effective_drop_count,
                                    Raindrop_GetLastVolume_0p01mm3(),
                                    Raindrop_GetTotalVolume_0p01mm3(),
                                    current_intensity_mmh, last_gain_used);
#endif
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

#if RAIN_CALIB_UART_ENABLE
			/* 串口心跳：每秒一行，无雨时也有输出，用于确认链路存活+观察事件计数。
			 * 字段：HB,原始事件数,有效滴数,当前阈值ADC,最近快照PA0实采峰值ADC */
			USART1_SendString("HB,raw=");
			USART1_SendUint32(raw_event_count);
			USART1_SendString(",eff=");
			USART1_SendUint32(effective_drop_count);
			USART1_SendString(",thr=");
			USART1_SendUint32(dynamic_threshold);
			USART1_SendString(",pa0max=");
			USART1_SendUint32((uint32_t)dbg_pa0_abs_max);
			USART1_SendString("\r\n");
#endif
        }

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
  * @note   更新OLED显示屏上的动态数据内容，显示峰值电压、主脉冲积分、滴数与体积
  *         显示函数只读取和格式化，不在这里做体积换算或累计
  */
void Update_Display(void)
{
	uint32_t v100 = (uint32_t)(current_voltage * 100.0f + 0.5f);
	uint32_t drops = effective_drop_count;
	uint32_t last_vol = Raindrop_GetLastVolume_0p01mm3();
	uint32_t total_vol = Raindrop_GetTotalVolume_0p01mm3();

	if (v100 > 99999UL)
	{
		v100 = 99999UL;
	}
	/* OLED_ShowNum超长会截掉高位，先钳位到显示位数上限 */
	if (drops > 9999UL)
	{
		drops = 9999UL;
	}
	if (last_vol > 99999UL)
	{
		last_vol = 99999UL;
	}
	if (total_vol > 999999999UL)
	{
		total_vol = 999999999UL;
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

	/* 行3：Dxxxx Vxxx.xx（滴数/最近一滴体积mm³，内部单位0.01mm³） */
	OLED_ShowChar(3, 1, 'D');
	OLED_ShowNum(3, 2, drops, 4);
	OLED_ShowChar(3, 6, ' ');
	OLED_ShowChar(3, 7, 'V');
	OLED_ShowNum(3, 8, last_vol / 100U, 3);
	OLED_ShowChar(3, 11, '.');
	OLED_ShowNum(3, 12, last_vol % 100U, 2);

	/* 行4：Txxxxxxx.xx（累计体积mm³，内部单位0.01mm³） */
	OLED_ShowChar(4, 1, 'T');
	OLED_ShowNum(4, 2, total_vol / 100U, 7);
	OLED_ShowChar(4, 9, '.');
	OLED_ShowNum(4, 10, total_vol % 100U, 2);
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
		return 3;
	}
	else if (peak_value < PEAK_MEDIUM_THRESHOLD)
	{
		/* 中等雨滴：200ms死区时间 */
		return 5;
	}
	else
	{
		/* 大雨滴：300ms死区时间 */
		return 8;
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
#if ENABLE_ISR_FAST_DISPLAY
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
			/* 增益标志不在ISR快路径里设：由快照路径权威决定，避免把PA1的'L'秒刷回'H' */
			last_valid_peak = peak_value;
			last_update_counter = main_loop_counter;
			peak_hold_counter = dynamic_hold_time;
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
	/* 增益标志不在ISR快路径里设：由快照路径权威决定，避免把PA1的'L'秒刷回'H' */
	last_valid_peak = peak_value;
	last_update_counter = main_loop_counter;
	peak_hold_counter = dynamic_hold_time;

	return 1;
}

#endif

/**
  * @brief  处理快照数据（如果就绪）
  * @param  无
  * @retval 无
 * @note   模拟看门狗/差分触发 → 预触发200点 + 后触发300点（PA0/PA1双通道） → Validate_And_Count_Event
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

	/* 原始事件数：每张被触发并处理的快照计一次（是否有效由后续验证决定） */
	raw_event_count++;

	uint16_t len = SNAPSHOT_SIZE;
	uint16_t high_peak_idx = 0, high_peak_val = 0;
	MainPulseFeatures_t pulse_features;
	uint8_t pulse_ok = 0;
	/* 死区只保护雨量计数，OLED仍处理每一张快照，保证每滴都有Volt/Imp结果 */
	uint8_t count_allowed = (event_deadtime_loops == 0);

	int32_t baseline_high = Compute_Baseline((uint16_t *)snapshot_buffer_high, len);

	/* PA0削顶判定提前到窗口划分之前：削顶事件需要放开前部窗口限制 */
	uint16_t pa0_clip_count = Count_Consecutive_AtOrAbove((uint16_t *)snapshot_buffer_high, len, PA0_CLIP_ADC);
	/* 过量程 = 满量程硬削顶(连续≥4090) 或 顶部平顶饱和(顶部可能停在<4090的3.2~3.3V)。
	 * 任一成立即视为PA0过量程，沿用下方既有逻辑触发PA1接管。 */
	uint8_t pa0_plateau = Detect_PA0_Plateau((uint16_t *)snapshot_buffer_high, len);
	uint8_t pa0_clipped = (uint8_t)((pa0_clip_count >= PA0_CLIP_COUNT_TH) || pa0_plateau);

	/* 严格前部处理：初始峰值搜索允许在预触发区域和前部窗口内搜索，但不搜索后部数据 */
	/* 定义前部分析窗口：触发点后的前2ms，完全忽略后部数据 */
	uint16_t front_window_start = PEAK_SEARCH_CENTER;  // 触发点索引（200）
	uint16_t front_window_end = PEAK_SEARCH_CENTER + FRONT_ANALYSIS_SAMPLES;  // 触发点后2ms
	if (front_window_end > len)
	{
		front_window_end = len - 1;
	}
	if (pa0_clipped)
	{
		/* 削顶大雨滴的脉宽通常超过2ms前部窗口，放开到整个快照，避免PA1积分被截断 */
		front_window_end = len;
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

	/* 默认按PA0高增益口径，后续PA1削顶判定可能改写为'L' */
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

	if (pulse_ok && !pa0_clipped)
	{
		/* 削顶事件不用主脉冲特征的窗口：其峰后窗口仅1.4ms，会截断削顶长脉冲 */
		start_index = pulse_features.pulse_start;
		end_index = pulse_features.pulse_end;
		front_peak_index = pulse_features.peak_index;
		front_peak_value = pulse_features.peak_value;
	}

	snapshot_peak_value = front_peak_value;
	snapshot_peak_index = front_peak_index;

	uint16_t max_sat_count = Count_Max_Consecutive_Saturation((uint16_t *)snapshot_buffer_high, len);
	uint8_t pa0_saturated = (max_sat_count >= HIGH_GAIN_SAT_COUNT_TH);
	int32_t baseline_low = Compute_Baseline((uint16_t *)snapshot_buffer_low, len);
	uint16_t pa1_peak_index = start_index;
	uint16_t pa1_peak_raw = snapshot_buffer_low[start_index];
	uint16_t pa1_peak_delta;
	uint8_t use_pa1;
	uint16_t final_peak_value = front_peak_value;
	uint16_t event_source_channel = RAIN_AREA_SOURCE_PA0;
	uint16_t event_flags = RAIN_AREA_FLAG_RAW_VALID | RAIN_AREA_FLAG_SCALED_VALID;
	uint16_t event_gain_x100 = 100U;
	uint32_t pa0_area_samples;
	uint32_t event_area_samples;
	uint32_t raw_integral_adc_us;
	uint32_t scaled_integral_adc_us;
	uint16_t event_peak_adc = front_peak_value;
	uint16_t event_baseline_adc = Clamp_U16_From_I32(active_baseline);

	if (pulse_ok)
	{
		pa0_area_samples = pulse_features.area_pos;
	}
	else
	{
		/* 主脉冲提取失败：不得沿用上一事件的残值，
		 * 用本事件前部窗口[start,end]现算保守面积，并打PULSE_FAIL标记 */
		Integrate_Delta_Window_ADC_us((uint16_t *)snapshot_buffer_high,
		                              start_index, end_index, baseline_high,
		                              &pa0_area_samples, 0, 0);
		event_flags |= RAIN_AREA_FLAG_PULSE_FAIL;
	}
	event_area_samples = pa0_area_samples;

	if (end_index >= start_index)
	{
		for (uint16_t i = start_index; i <= end_index; i++)
		{
			if (snapshot_buffer_low[i] > pa1_peak_raw)
			{
				pa1_peak_raw = snapshot_buffer_low[i];
				pa1_peak_index = i;
			}
		}
	}
	pa1_peak_delta = Clamp_ADC_Delta_From_I32((int32_t)pa1_peak_raw - baseline_low);

	/* PA1等效比：用本事件中PA0未削顶的样本对在线估计，失败则用保守默认值 */
	uint16_t pa1_eq_ratio_x100 = Estimate_PA1_Eq_Ratio_X100((uint16_t *)snapshot_buffer_high,
	                                                        (uint16_t *)snapshot_buffer_low,
	                                                        start_index,
	                                                        end_index,
	                                                        baseline_high,
	                                                        baseline_low);
	{
		uint64_t shadow_pa0_adc_us64 = ((uint64_t)pa0_area_samples * (uint64_t)ADC_SAMPLE_INTERVAL_NS + 500ULL) / 1000ULL;
		uint32_t shadow_pa0_adc_us = Clamp_U64_To_U32(shadow_pa0_adc_us64);
		uint32_t shadow_pa0_peak_mv = pa0_clipped ? EVENT_CLIP_MV : ADC_Count_To_mV(front_peak_value);
		uint32_t shadow_pa0_impulse_mv_us = pulse_ok ? pulse_features.impulse_mv_us : ADC_us_To_mV_us(shadow_pa0_adc_us);
		uint32_t shadow_pa1_area_samples = 0;
		uint32_t shadow_pa1_raw_adc_us = 0;
		uint32_t shadow_pa1_scaled_adc_us = 0;
		uint32_t shadow_pa1_raw_peak_mv = ADC_Count_To_mV(pa1_peak_delta);
		uint32_t shadow_pa1_eq_peak_mv;
		uint32_t shadow_pa1_eq_impulse_mv_us;

		Integrate_Delta_Window_ADC_us((uint16_t *)snapshot_buffer_low,
		                              start_index,
		                              end_index,
		                              baseline_low,
		                              &shadow_pa1_area_samples,
		                              &shadow_pa1_raw_adc_us,
		                              &shadow_pa1_scaled_adc_us);
		(void)shadow_pa1_area_samples;
		(void)shadow_pa1_scaled_adc_us;
		shadow_pa1_eq_peak_mv = Clamp_U64_To_U32(((uint64_t)shadow_pa1_raw_peak_mv * pa1_eq_ratio_x100) / 100ULL);
		shadow_pa1_eq_impulse_mv_us = Clamp_U64_To_U32(((uint64_t)ADC_us_To_mV_us(shadow_pa1_raw_adc_us) * pa1_eq_ratio_x100) / 100ULL);
		shadow_result = Build_Event_Result(pa0_clipped,
		                                   shadow_pa0_peak_mv,
		                                   shadow_pa0_impulse_mv_us,
		                                   shadow_pa1_raw_peak_mv,
		                                   shadow_pa1_eq_peak_mv,
		                                   shadow_pa1_eq_impulse_mv_us);
	}
	use_pa1 = (uint8_t)(shadow_result.source == EVENT_RESULT_SOURCE_PA1);
	dbg_event_pa1_delta = pa1_peak_delta;

#if RAIN_CALIB_UART_ENABLE
	/* 切换判定诊断：每张快照一行。
	 * 字段：SNP,事件序号,PA0实采峰值ADC,贴顶最长连续点数,硬削顶连续点数,
	 *       PA1原始峰值mV,结果源(0=PA0/1=PA1/2=CLIP),use_pa1,在线等效比x100 */
	USART1_SendString("SNP,");
	USART1_SendUint32(raw_event_count);
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)dbg_pa0_abs_max);
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)dbg_pa0_plateau_run);
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)pa0_clip_count);
	USART1_SendString(",");
	USART1_SendUint32(ADC_Count_To_mV((uint32_t)pa1_peak_delta));
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)shadow_result.source);
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)use_pa1);
	USART1_SendString(",");
	USART1_SendUint32((uint32_t)pa1_eq_ratio_x100);
	USART1_SendString("\r\n");
#endif

	if (pa0_saturated)
	{
		event_flags |= RAIN_AREA_FLAG_PA0_SAT;
	}

	if (use_pa1)
	{
		Integrate_Delta_Window_ADC_us((uint16_t *)snapshot_buffer_low,
		                              start_index,
		                              end_index,
		                              baseline_low,
		                              &event_area_samples,
		                              &raw_integral_adc_us,
		                              &scaled_integral_adc_us);
		/* 积分按等效比换算到PA0口径；峰值用Build_Event_Result钳位后的等效mV
		 * 反算ADC码（可超过4095，Compute_Voltage_From_ADC不截断，显示即>3.3V） */
		scaled_integral_adc_us = Clamp_U64_To_U32(((uint64_t)raw_integral_adc_us * pa1_eq_ratio_x100) / 100ULL);
		final_peak_value = (uint16_t)(((uint64_t)shadow_result.peak_mv * ADC_FULL_SCALE + (ADC_REF_MV / 2U)) / ADC_REF_MV);
		event_peak_adc = final_peak_value;
		event_baseline_adc = Clamp_U16_From_I32(baseline_low);
		event_source_channel = RAIN_AREA_SOURCE_PA1;
		event_gain_x100 = pa1_eq_ratio_x100;
		event_flags |= RAIN_AREA_FLAG_USE_PA1;
		last_gain_used = 'L';
		{
			uint16_t pulse_width = (uint16_t)(end_index - start_index + 1U);
			uint64_t impulse_mv_us64 = ((uint64_t)scaled_integral_adc_us * ADC_REF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

			current_peak_raw = pa1_peak_raw;
			current_peak = final_peak_value;
			current_voltage = Compute_Voltage_From_ADC(current_peak);
			voltage_sum += current_voltage;
			current_pulse_valid = 1;
			current_pulse_area = (event_area_samples > 999999999UL) ? 999999999UL : event_area_samples;
			current_impulse_mv_us = (impulse_mv_us64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)impulse_mv_us64;
			current_pulse_width = pulse_width;
			current_pulse_rise = (front_peak_index > start_index) ? (uint16_t)(front_peak_index - start_index) : 1U;
			current_pulse_fall = (end_index > front_peak_index) ? (uint16_t)(end_index - front_peak_index) : 1U;
#if ENABLE_ISR_FAST_DISPLAY
			last_valid_peak = current_peak;
			last_update_counter = main_loop_counter;
#endif
			peak_hold_counter = Get_Dynamic_Peak_Hold_Time(current_peak);
			display_counter = 20U;
		}
	}
	else
	{
		uint64_t area_adc_us64 = ((uint64_t)event_area_samples * (uint64_t)ADC_SAMPLE_INTERVAL_NS + 500ULL) / 1000ULL;

		if (pa0_saturated)
		{
			final_peak_value = ADC_FULL_SCALE;
			event_peak_adc = ADC_FULL_SCALE;
		}
		raw_integral_adc_us = Clamp_U64_To_U32(area_adc_us64);
		scaled_integral_adc_us = raw_integral_adc_us;
		last_gain_used = 'H';
	}

	dbg_event_source_channel = event_source_channel;
	dbg_event_max_sat_count = max_sat_count;
	dbg_event_raw_integral_adc_us = raw_integral_adc_us;
	dbg_event_scaled_integral_adc_us = scaled_integral_adc_us;
	dbg_event_gain_x100 = event_gain_x100;
	dbg_event_flags = event_flags;
	(void)pa1_peak_index;

	uint8_t event_countable = 0;
	if (count_allowed)
	{
		uint16_t core_width = (uint16_t)(end_index - start_index + 1U);
		uint16_t core_rise = (front_peak_index > start_index) ? (uint16_t)(front_peak_index - start_index) : 0U;
		uint16_t core_fall = (end_index > front_peak_index) ? (uint16_t)(end_index - front_peak_index) : 0U;

		if (pa0_clipped)
		{
			/* 削顶平顶波形无法通过常规形状判定（峰后无下降、邻点差为0）：
			 * 削顶真实性已由连续削顶样本数（PA0_CLIP_COUNT_TH）保证，
			 * 这里只再用脉宽下限排除EMI短毛刺 */
			event_countable = (uint8_t)((core_width > MAX_NOISE_PULSE_WIDTH) &&
			                            ((uint32_t)core_width * ADC_SAMPLE_INTERVAL_NS >=
			                             (uint32_t)MIN_PULSE_DURATION_US * 1000UL));
		}
		else
		{
			/* 第一层：严格形状验证（平滑度/对称比/峰值稳定性等），通过即"干净雨滴" */
			event_countable = Validate_And_Count_Event(active_buffer, end_index + 1, front_peak_index, front_peak_value, threshold, start_index, end_index);

			/* 第二层：核心物理闸门兜底。严格链的平滑度（≤MAX_STEEP_SLOPE=50
			 * ADC/样本）、对称比、峰值稳定性等阈值按早期42us采样档的小雨滴
			 * 整定，14us档下真实雨滴上升沿平均斜率常超50（如2200ADC/35样本
			 * ≈63），多条件AND一条不过即整体拒绝，会把真滴全部漏计。
			 * 这里只看幅度+脉宽+持续时间+基本起落（与削顶分支同等信任级别）：
			 * EMI短毛刺仍被脉宽/持续时间挡掉，重复计数仍由事件死区挡掉。 */
			if (!event_countable &&
			    front_peak_value >= MIN_PEAK_AMPLITUDE &&
			    front_peak_value > (uint16_t)(threshold + MIN_PEAK_DELTA_OVER_THR) &&
			    core_width > MAX_NOISE_PULSE_WIDTH &&
			    ((uint32_t)core_width * ADC_SAMPLE_INTERVAL_NS >=
			     (uint32_t)MIN_PULSE_DURATION_US * 1000UL) &&
			    core_rise >= MIN_RISE_SAMPLES &&
			    core_fall >= MIN_DECAY_SAMPLES)
			{
				event_countable = 1;
				event_flags |= RAIN_AREA_FLAG_RELAXED;
				dbg_event_relaxed_count++;
			}
		}

		if (!event_countable)
		{
			dbg_event_reject_count++;
#if RAIN_CALIB_UART_ENABLE
			/* 被拒事件诊断行（只在触发快照被拒时输出，无雨时串口仍安静）：
			 * REJ,序号,峰值,基线,脉宽,上升,下降 —— 现场排查"滴了不计数"卡哪个闸门 */
			USART1_SendString("REJ,");
			USART1_SendUint32(dbg_event_reject_count);
			USART1_SendString(",");
			USART1_SendUint32((uint32_t)front_peak_value);
			USART1_SendString(",");
			USART1_SendUint32((uint32_t)event_baseline_adc);
			USART1_SendString(",");
			USART1_SendUint32((uint32_t)core_width);
			USART1_SendString(",");
			USART1_SendUint32((uint32_t)core_rise);
			USART1_SendString(",");
			USART1_SendUint32((uint32_t)core_fall);
			USART1_SendString("\r\n");
#endif
		}
	}

	if (event_countable)
	{
		snapshot_valid_count++;   /* 两层判定通过的总数（其中兜底层占比见dbg_event_relaxed_count） */

		/* 波形刷新已移到函数末尾：每张处理过的快照都刷一帧，不再只限被计数的有效滴 */

		current_peak_raw = use_pa1 ? pa1_peak_raw : final_peak_value;
		current_peak = final_peak_value;
		current_voltage = Compute_Voltage_From_ADC(current_peak);
		voltage_sum += current_voltage;
#if ENABLE_ISR_FAST_DISPLAY
		last_valid_peak = current_peak;
		last_update_counter = main_loop_counter;
#endif
		peak_hold_counter = Get_Dynamic_Peak_Hold_Time(current_peak);
		if (use_pa1)
		{
			uint16_t pulse_width = (uint16_t)(end_index - start_index + 1U);
			uint64_t impulse_mv_us64 = ((uint64_t)scaled_integral_adc_us * ADC_REF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

			current_pulse_valid = 1;
			current_pulse_area = (event_area_samples > 999999999UL) ? 999999999UL : event_area_samples;
			current_impulse_mv_us = (impulse_mv_us64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)impulse_mv_us64;
			current_pulse_width = pulse_width;
			current_pulse_rise = (front_peak_index > start_index) ? (uint16_t)(front_peak_index - start_index) : 1U;
			current_pulse_fall = (end_index > front_peak_index) ? (uint16_t)(end_index - front_peak_index) : 1U;
		}
		else if (pulse_ok)
		{
			Update_MainPulse_Display_State(&pulse_features);
		}
		else
		{
			uint16_t pulse_width = (uint16_t)(end_index - start_index + 1U);
			uint64_t impulse_mv_us64 = ((uint64_t)scaled_integral_adc_us * ADC_REF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

			current_pulse_valid = 0;
			current_pulse_area = (event_area_samples > 999999999UL) ? 999999999UL : event_area_samples;
			current_impulse_mv_us = (impulse_mv_us64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)impulse_mv_us64;
			current_pulse_width = pulse_width;
			current_pulse_rise = (front_peak_index > start_index) ? (uint16_t)(front_peak_index - start_index) : 1U;
			current_pulse_fall = (end_index > front_peak_index) ? (uint16_t)(end_index - front_peak_index) : 1U;
		}

		// ========== 小波特征提取（触发后二次分析，仅调试统计） ==========
		// 执行顺序：
		// 1. 算 baseline
		// 2. 算 win_start（带前沿约束 + 边界）
		// 3. Wavelet_ExtractFeatures（得到 success/hf_ratio）
		// 4. Determine_Event_Type（得到 event_type）
		// 5. 更新类型统计计数器（cnt_rain_clean, cnt_rain_fast, cnt_vib, cnt_emi, cnt_bg, cnt_bad）
		// 6. 设置 event_deadtime_loops（动态死区）
		// 7. 统一计量入口（体积/滴数/事件入队/调试串口）

		WaveletFeatures_t wavelet_features;
		EventType_t event_type = EVT_BAD;  // 默认值，小波失败时使用

		// 1. 计算wavelet_baseline（用快照自身的预触发段：处理到这里时
		//    环形缓冲可能已被新样本覆盖，不能再当作"触发前历史"使用）
		int32_t wavelet_baseline = Compute_Wavelet_Baseline();
		
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
		
		// 3. 提取小波特征（仅调试统计，不参与有效性判定与体积计算；
		//    失败时wavelet_features.success==0，由Determine_*归类BAD）
		(void)Wavelet_ExtractFeatures(active_buffer, end_index + 1,
		                              win_start, wavelet_baseline,
		                              &wavelet_features);
		
		// 4. 确定事件类型（第3阶段，更细化的类型划分）
		event_type = Determine_Event_Type(&wavelet_features);
		
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
		event_deadtime_loops = base_deadtime;
#if 0
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
#endif
		if (event_deadtime_loops > EVENT_DEADTIME_MAX_LOOPS)
		{
			event_deadtime_loops = EVENT_DEADTIME_MAX_LOOPS;
		}
		
		// 7. 统一计量入口：每个有效事件只在此处累计一次
		//    （体积换算 → 体积累计 → 有效滴数+1 → 事件入队 → 调试串口）
		//    小波分类只做统计，不参与否决。
		{
			RainAreaEvent_t area_evt;
			uint32_t event_impulse_mv_us;
			uint32_t event_volume_0p01mm3;
			uint16_t pulse_width = pulse_ok ?
				(uint16_t)(pulse_features.pulse_end - pulse_features.pulse_start + 1U) :
				(uint16_t)(end_index - start_index + 1U);

			/* 统一积分口径（mV·us）：
			 * - PA1接管：等效换算后的积分（scaled_integral_adc_us已乘等效比）
			 * - PA0正常：主脉冲梯形积分（与标定数据同源）
			 * - 主脉冲提取失败：前部窗口矩形积分兜底（事件已带PULSE_FAIL标记） */
			if (use_pa1 || !pulse_ok)
			{
				event_impulse_mv_us = ADC_us_To_mV_us(scaled_integral_adc_us);
			}
			else
			{
				event_impulse_mv_us = pulse_features.impulse_mv_us;
			}

			/* 体积换算并累计（全程序唯一累计入口），随后有效滴数+1 */
			Raindrop_CommitByIntegral(event_impulse_mv_us);
			event_volume_0p01mm3 = Raindrop_GetLastVolume_0p01mm3();
			effective_drop_count++;
			total_rain_mm += g_mm_per_drop;
			if (sec_index < SECONDS_WINDOW)
			{
				drops_per_second[sec_index] += 1;
			}

			area_evt.seq = RainAreaBuffer_NextSeq();
			area_evt.peak_adc = event_peak_adc;
			area_evt.baseline_adc = event_baseline_adc;
			area_evt.pulse_width_samples = pulse_width;
			area_evt.area_adc_samples = event_area_samples;
			area_evt.area_adc_us = scaled_integral_adc_us;
			area_evt.flags = event_flags;
			area_evt.source_channel = event_source_channel;
			area_evt.max_sat_count = max_sat_count;
			area_evt.gain_x100 = event_gain_x100;
			area_evt.raw_integral_adc_us = raw_integral_adc_us;
			area_evt.scaled_integral_adc_us = scaled_integral_adc_us;
			area_evt.impulse_mv_us = event_impulse_mv_us;
			area_evt.volume_0p01mm3 = event_volume_0p01mm3;

			RainAreaBuffer_UpdateLatest(&area_evt);
			if (!RainAreaBuffer_Push(&area_evt))
			{
				RainAreaBuffer_RecordOverflow();
			}

			/* 标定串口：与计量同一事件链路，每个有效事件一行 */
			Rain_CalibUart_SendEvent(effective_drop_count,
			                         event_impulse_mv_us,
			                         event_volume_0p01mm3,
			                         Raindrop_GetTotalVolume_0p01mm3(),
			                         final_peak_value,
			                         pulse_width,
			                         current_pulse_rise,
			                         current_pulse_fall);
		}
	}

	/* 备份快照用于导出 */
	for (uint16_t i_copy = 0; i_copy < len; i_copy++)
	{
		export_buffer[i_copy] = snapshot_buffer_high[i_copy];
	}
	export_ready = 1;                // 设置导出就绪标志

#if LCD_ENABLE
	/* 波形刷新：每张处理过的快照都画一帧（不再只限被计数的有效滴），
	 * 但仅当本快照PA0实采绝对峰值≥LCD_WAVE_MIN_ADC时才刷，过滤高干扰阶段的小信号。
	 * dbg_pa0_abs_max已由Detect_PA0_Plateau在本张快照里更新为PA0全缓冲绝对峰值。
	 * export_buffer已是本滴PA0全分辨率波形；元数据用本次快照的基线/阈值/峰值/通道。 */
	if (dbg_pa0_abs_max >= LCD_WAVE_MIN_ADC)
	{
		lcd_wave_baseline = Clamp_U16_From_I32(baseline_high);
		lcd_wave_threshold = dynamic_threshold;
		lcd_wave_peak_index = front_peak_index;
		lcd_wave_pa1 = use_pa1;
		lcd_draw_pending = 1;
	}
#endif

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
#if RAIN_TEST_FIXED_THRESHOLD_ENABLE
	/* 测试模式：把采集阈值固定在约2V，跳过噪声自适应。
	 * 测试完成后把 RAIN_TEST_FIXED_THRESHOLD_ENABLE 改回 0 即恢复下面的自适应逻辑。 */
	if (dynamic_threshold != RAIN_TEST_FIXED_THRESHOLD_ADC)
	{
		dynamic_threshold = RAIN_TEST_FIXED_THRESHOLD_ADC;
		AD_SetThreshold(dynamic_threshold);
	}
	return;
#else
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
#endif
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
	return (float)sum * g_mm_per_drop * 3600.0f / (float)SECONDS_WINDOW; // 计算降雨强度
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
  * @brief  计算小波baseline（取触发点前32点的中位数）
  * @retval baseline值（int32_t）
  * @note   数据源用快照自身的预触发段：主循环处理快照时，环形缓冲
  *         大概率已被新样本覆盖，只有快照里的预触发数据才是"触发前历史"
  */
#define WAVELET_BASELINE_N  32
static int32_t Compute_Wavelet_Baseline(void)
{
	uint16_t samples[WAVELET_BASELINE_N];

	// 取快照预触发段末尾的32点（即触发点前32点）
	for (uint16_t i = 0; i < WAVELET_BASELINE_N; i++)
	{
		samples[i] = snapshot_buffer_high[SNAPSHOT_PRE_SAMPLES - WAVELET_BASELINE_N + i];
	}

	// 中位数计算（冒泡排序后取中间值）
	for (uint16_t i = 0; i < WAVELET_BASELINE_N - 1; i++)
	{
		for (uint16_t j = 0; j < WAVELET_BASELINE_N - 1 - i; j++)
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
	return (int32_t)((samples[WAVELET_BASELINE_N / 2 - 1] + samples[WAVELET_BASELINE_N / 2]) / 2);
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

static uint32_t Clamp_U64_To_U32(uint64_t value)
{
	if (value > 0xFFFFFFFFULL)
	{
		return 0xFFFFFFFFUL;
	}
	return (uint32_t)value;
}

static uint32_t ADC_Count_To_mV(uint32_t adc_value)
{
	uint64_t mv;

	if (adc_value > ADC_FULL_SCALE)
	{
		adc_value = ADC_FULL_SCALE;
	}

	mv = ((uint64_t)adc_value * ADC_REF_MV * ADC_VOLTAGE_CALIBRATION_NUM +
	      ((uint64_t)ADC_FULL_SCALE * ADC_VOLTAGE_CALIBRATION_DEN / 2ULL)) /
	     ((uint64_t)ADC_FULL_SCALE * ADC_VOLTAGE_CALIBRATION_DEN);
	return Clamp_U64_To_U32(mv);
}

static uint32_t ADC_us_To_mV_us(uint32_t adc_us)
{
	uint64_t mv_us;

	mv_us = ((uint64_t)adc_us * ADC_REF_MV * ADC_VOLTAGE_CALIBRATION_NUM +
	         ((uint64_t)ADC_FULL_SCALE * ADC_VOLTAGE_CALIBRATION_DEN / 2ULL)) /
	        ((uint64_t)ADC_FULL_SCALE * ADC_VOLTAGE_CALIBRATION_DEN);
	return Clamp_U64_To_U32(mv_us);
}

static uint16_t Count_Consecutive_AtOrAbove(uint16_t *buf, uint16_t len, uint16_t threshold)
{
	uint16_t max_count = 0;
	uint16_t current_count = 0;

	if (buf == 0)
	{
		return 0;
	}

	for (uint16_t i = 0; i < len; i++)
	{
		if (buf[i] >= threshold)
		{
			current_count++;
			if (current_count > max_count)
			{
				max_count = current_count;
			}
		}
		else
		{
			current_count = 0;
		}
	}

	return max_count;
}

static RainEventResult_t Build_Event_Result(uint8_t pa0_clipped,
                                            uint32_t pa0_peak_mv,
                                            uint32_t pa0_impulse_mv_us,
                                            uint32_t pa1_raw_peak_mv,
                                            uint32_t pa1_eq_peak_mv,
                                            uint32_t pa1_eq_impulse_mv_us)
{
	RainEventResult_t result;

	result.source = EVENT_RESULT_SOURCE_PA0;
	result.valid_flag = 1U;
	result.flags = EVENT_RESULT_FLAG_VALID;
	result.old_pa0_peak_mv = pa0_peak_mv;
	result.old_pa0_impulse_mv_us = pa0_impulse_mv_us;
	result.peak_mv = pa0_peak_mv;
	result.impulse_mv_us = pa0_impulse_mv_us;
	result.pa1_peak_mv = pa1_raw_peak_mv;
	result.pa1_impulse_mv_us = pa1_eq_impulse_mv_us;

	if (pa0_clipped == 0U)
	{
		return result;
	}

	result.flags |= EVENT_RESULT_FLAG_PA0_CLIPPED;

	/* PA1有效性校验：原始幅值过小视为通道异常（断线/无信号/纯噪声），
	 * 回退为削顶估计（3.3V下限），不盲信PA1数据 */
	if (pa1_raw_peak_mv < PA1_VALID_MIN_MV)
	{
		result.flags |= EVENT_RESULT_FLAG_PA1_LOW;
		result.source = EVENT_RESULT_SOURCE_CLIP;
		result.peak_mv = EVENT_CLIP_MV;
		return result;
	}
	if (pa1_raw_peak_mv > PA1_VALID_MAX_MV)
	{
		/* 超出PA1预期量程：仍切换，等效值由下方上限钳位兜底 */
		result.flags |= EVENT_RESULT_FLAG_PA1_HIGH;
	}

	result.flags |= EVENT_RESULT_FLAG_PA1_VALID;
	result.source = EVENT_RESULT_SOURCE_PA1;

	/* 等效值钳位：PA0已削顶，真实峰值必高于3.3V，下限托底到3.4V；
	 * 上限钳到5V，防止等效比误差导致离谱读数（实测最大雨滴约4.xV） */
	if (pa1_eq_peak_mv < EVENT_EQ_MIN_MV)
	{
		pa1_eq_peak_mv = EVENT_EQ_MIN_MV;
		result.flags |= EVENT_RESULT_FLAG_EQ_LIMIT;
	}
	else if (pa1_eq_peak_mv > EVENT_EQ_MAX_MV)
	{
		pa1_eq_peak_mv = EVENT_EQ_MAX_MV;
		result.flags |= EVENT_RESULT_FLAG_EQ_LIMIT;
	}
	result.peak_mv = pa1_eq_peak_mv;
	result.impulse_mv_us = pa1_eq_impulse_mv_us;

	return result;
}

/**
  * @brief  检测PA0是否"平顶饱和"（过量程）——用于切换到PA1
  * @param  buf: PA0高增益快照缓冲区
  * @param  len: 缓冲区长度
  * @retval 1=检测到平顶饱和（峰值≥门限且顶部连续贴顶≥点数阈值），0=否
  * @note   两遍扫描：先取全快照最大值；若最大值<PA0_OVERRANGE_GATE_ADC直接判否；
  *         否则统计落在[峰值-PA0_PLATEAU_BAND_ADC, 峰值]带内的最长连续样本数，
  *         达到PA0_PLATEAU_COUNT_TH 即判为平顶饱和。门限/带宽/点数见宏定义，可现场整定。
  */
static uint8_t Detect_PA0_Plateau(uint16_t *buf, uint16_t len)
{
	uint16_t i;
	uint16_t peak = 0;
	uint16_t run = 0;
	uint16_t max_run = 0;
	int32_t band_low;

	dbg_pa0_abs_max = 0;
	dbg_pa0_plateau_run = 0;

	if (buf == 0)
	{
		return 0;
	}

	for (i = 0; i < len; i++)
	{
		if (buf[i] > peak)
		{
			peak = buf[i];
		}
	}
	dbg_pa0_abs_max = peak;       // 诊断：记录实采绝对峰值
	/* 峰值没到过量程门限：肯定不是饱和，直接返回（也避免把低幅平段误判成平顶） */
	if (peak < PA0_OVERRANGE_GATE_ADC)
	{
		return 0;
	}

	band_low = (int32_t)peak - (int32_t)PA0_PLATEAU_BAND_ADC;
	for (i = 0; i < len; i++)
	{
		if ((int32_t)buf[i] >= band_low)
		{
			run++;
			if (run > max_run)
			{
				max_run = run;
			}
		}
		else
		{
			run = 0;
		}
	}
	dbg_pa0_plateau_run = max_run;   // 诊断：记录贴顶最长连续点数

	return (uint8_t)(max_run >= PA0_PLATEAU_COUNT_TH);
}

static uint16_t Count_Max_Consecutive_Saturation(uint16_t *buf, uint16_t len)
{
	uint16_t max_count = 0;
	uint16_t current_count = 0;

	if (buf == 0)
	{
		return 0;
	}

	for (uint16_t i = 0; i < len; i++)
	{
		if (buf[i] >= HIGH_GAIN_SAT_THRESHOLD)
		{
			current_count++;
			if (current_count > max_count)
			{
				max_count = current_count;
			}
		}
		else
		{
			current_count = 0;
		}
	}

	return max_count;
}

static void Integrate_Delta_Window_ADC_us(uint16_t *buf, uint16_t start_index,
                                          uint16_t end_index, int32_t baseline,
                                          uint32_t *area_samples,
                                          uint32_t *raw_integral_adc_us,
                                          uint32_t *scaled_integral_adc_us)
{
	uint64_t area_sum = 0;
	uint64_t raw_ns_sum = 0;
	uint64_t scaled_ns_sum = 0;

	if (area_samples)
	{
		*area_samples = 0;
	}
	if (raw_integral_adc_us)
	{
		*raw_integral_adc_us = 0;
	}
	if (scaled_integral_adc_us)
	{
		*scaled_integral_adc_us = 0;
	}
	if (buf == 0 || end_index < start_index)
	{
		return;
	}

	for (uint16_t i = start_index; i <= end_index; i++)
	{
		uint16_t delta = Clamp_ADC_Delta_From_I32((int32_t)buf[i] - baseline);

		area_sum += delta;
		raw_ns_sum += (uint64_t)delta * (uint64_t)ADC_SAMPLE_INTERVAL_NS;
		scaled_ns_sum += (uint64_t)delta * (uint64_t)ADC_SAMPLE_INTERVAL_NS;
	}

	if (area_samples)
	{
		*area_samples = Clamp_U64_To_U32(area_sum);
	}
	if (raw_integral_adc_us)
	{
		*raw_integral_adc_us = Clamp_U64_To_U32((raw_ns_sum + 500ULL) / 1000ULL);
	}
	if (scaled_integral_adc_us)
	{
		*scaled_integral_adc_us = Clamp_U64_To_U32((scaled_ns_sum + 500ULL) / 1000ULL);
	}
}

/**
  * @brief  在线估计PA1→PA0等效比（×100）
  * @note   PA0/PA1同步采样，PA0未削顶的样本对就是天然标定数据：
  *         在事件窗口内取 PA0幅度足够大且未削顶、PA1幅度高于噪声 的样本对，
  *         对 pa0_delta/pa1_delta 取平均。硬件放大倍数确定后，若在线估计
  *         波动大，可把 PA1_EQ_RATIO_DEFAULT_X100 改为实测值并收紧上下限。
  */
static uint16_t Estimate_PA1_Eq_Ratio_X100(uint16_t *high_buf, uint16_t *low_buf,
                                           uint16_t start_index, uint16_t end_index,
                                           int32_t baseline_high, int32_t baseline_low)
{
	uint32_t ratio_sum_x100 = 0;
	uint16_t pair_count = 0;
	uint32_t ratio_x100;

	if (high_buf == 0 || low_buf == 0 || end_index < start_index)
	{
		return PA1_EQ_RATIO_DEFAULT_X100;
	}

	for (uint16_t i = start_index; i <= end_index; i++)
	{
		uint16_t pa0_delta;
		uint16_t pa1_delta;

		if (high_buf[i] >= PA0_CLIP_ADC)
		{
			continue;   /* 削顶样本不反映真实比值 */
		}
		pa0_delta = Clamp_ADC_Delta_From_I32((int32_t)high_buf[i] - baseline_high);
		pa1_delta = Clamp_ADC_Delta_From_I32((int32_t)low_buf[i] - baseline_low);
		if (pa0_delta < PA1_RATIO_PA0_MIN_DELTA || pa1_delta < PA1_RATIO_PA1_MIN_DELTA)
		{
			continue;   /* 小信号样本受噪声影响大，不参与估计 */
		}
		ratio_sum_x100 += ((uint32_t)pa0_delta * 100UL) / pa1_delta;
		pair_count++;
	}

	if (pair_count == 0)
	{
		return PA1_EQ_RATIO_DEFAULT_X100;
	}

	ratio_x100 = ratio_sum_x100 / pair_count;
	if (ratio_x100 < PA1_EQ_RATIO_MIN_X100)
	{
		ratio_x100 = PA1_EQ_RATIO_MIN_X100;
	}
	if (ratio_x100 > PA1_EQ_RATIO_MAX_X100)
	{
		ratio_x100 = PA1_EQ_RATIO_MAX_X100;
	}
	return (uint16_t)ratio_x100;
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

#if ENABLE_ISR_FAST_DISPLAY
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
#endif

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
	uint16_t band_lo;
	uint16_t band_hi;
	uint16_t threshold20;
	uint16_t threshold50;

	if (features == 0 || buf == 0 || len == 0 || anchor_peak_index >= len)
	{
		return 0;
	}

	/* 绝对值包络带：|信号-基线|<=delta 即在带内（正负半周对称判定） */
	band_lo = Clamp_U16_From_I32(baseline - (int32_t)MAIN_PULSE_BASELINE_DELTA);
	band_hi = Clamp_U16_From_I32(baseline + (int32_t)MAIN_PULSE_BASELINE_DELTA);

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
		if (buf[i] >= band_lo && buf[i] <= band_hi)
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
		if (buf[i] >= band_lo && buf[i] <= band_hi)
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

/**
  * @brief  配置USART1（PA9=TX, PA10=RX, 115200 8N1，USART1默认映射）
  * @note   仅用于发送雨滴标定事件CSV
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
  * @brief  发送字符串（标定用）
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
  * @brief  发送uint32_t数字（标定用）
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

static void Rain_CalibUart_SendEvent(uint32_t drop_id,
                                     uint32_t integral,
                                     uint32_t vol_0p01mm3,
                                     uint32_t total_0p01mm3,
                                     uint16_t peak,
                                     uint16_t width,
                                     uint16_t rise,
                                     uint16_t fall)
{
#if RAIN_CALIB_UART_ENABLE
    /* 格式：EVT,drop_id,integral,vol_0p01mm3,total_0p01mm3,peak,width,rise,fall */
    USART1_SendString("EVT,");
    USART1_SendUint32(drop_id);
    USART1_SendString(",");
    USART1_SendUint32(integral);
    USART1_SendString(",");
    USART1_SendUint32(vol_0p01mm3);
    USART1_SendString(",");
    USART1_SendUint32(total_0p01mm3);
    USART1_SendString(",");
    USART1_SendUint32((uint32_t)peak);
    USART1_SendString(",");
    USART1_SendUint32((uint32_t)width);
    USART1_SendString(",");
    USART1_SendUint32((uint32_t)rise);
    USART1_SendString(",");
    USART1_SendUint32((uint32_t)fall);
    USART1_SendString("\r\n");
#else
    (void)drop_id;
    (void)integral;
    (void)vol_0p01mm3;
    (void)total_0p01mm3;
    (void)peak;
    (void)width;
    (void)rise;
    (void)fall;
#endif
}
