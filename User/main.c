// 包含必要的头文件
#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "AD.h"
#include "stm32f10x_usart.h"
#include "Modbus_Slave.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_iwdg.h"
#include "TF_Comm.h"
#include "SPI_SlaveLink.h"
#include "WaveletEnergy.h"
#include "raindrop.h"
#include <string.h>  /* strncmp */

/* ===================== Modbus 从站地址配置 ===================== */
#define MODBUS_SLAVE_ID    1

/* ========== 独立看门狗(IWDG)参数定义 ========== */
#define IWDG_TIMEOUT_SECONDS    3
#define IWDG_PRESCALER          IWDG_Prescaler_64
#define IWDG_RELOAD_VALUE       1500

/* ========== 降雨强度统计 ========== */
#define SECONDS_WINDOW          60
volatile uint16_t drops_per_second[SECONDS_WINDOW] = {0};
volatile uint8_t sec_index = 0;
static uint32_t last_sec_index_update_ms = 0;  /* 上次更新秒窗口的 g_sys_time_ms */
static uint32_t last_display_update_ms = 0;    /* 上次 OLED 刷新的 g_sys_time_ms */
static uint32_t last_syscheck_update_ms = 0;   /* 上次系统检查的 g_sys_time_ms */
static uint32_t last_debug_counter_ms = 0;    /* 调试10秒计数器 */
float current_intensity_mmh = 0.0f;

/* ========== 全局变量 ========== */
WaveletEnergyCtx_t g_we_ctx;  /* 小波能量积分上下文（DMA中断+主循环共享） */

/* 累计体积（单位：0.01mm³） */
volatile uint32_t g_total_volume_0p01mm3 = 0;

/* 系统状态 */
volatile uint32_t watchdog_trigger_count = 0;
extern volatile uint32_t sampling_tick_counter;
static uint32_t last_sampling_tick = 0;

/* ===================== 标定相关全局变量 ===================== */
/* 滴数计数器（注意：不可声明为static，需被WaveletEnergy.c通过extern访问） */
volatile uint32_t g_cal_drop_count    = 0;  /* 用户手动标记次数（已废弃，仅保留兼容） */
volatile uint32_t g_cal_detected_count = 0;  /* 程序检测到的雨滴总数 */
volatile uint32_t g_cal_vib_count     = 0;  /* 程序检测到的振动总数 */

/* ===== 分组标定记录 ===== */
/* 每组最多 MAX_DROPS_PER_GROUP 滴，每滴记录：nr、hf、冲量、dE、时间戳 */
#define MAX_DROPS_PER_GROUP  100
#define MAX_GROUPS           2
typedef struct {
    uint8_t  valid;        /* 该记录是否有效 */
    uint8_t  group_idx;    /* 组编号（0~MAX_GROUPS-1） */
    uint32_t seq_in_group; /* 组内序号（1起） */
    uint32_t nr;           /* 归一化比值（×1000permille） */
    uint16_t hf;           /* 高频比例（permille） */
    float    impulse;      /* 冲量（ADC·次） */
    uint32_t event_samples; /* Event width in raw samples. */
    uint32_t dE;           /* 能量增量 */
    uint32_t time_ms;      /* 时间戳（ms） */
} CalDropRecord_t;

static CalDropRecord_t g_cal_records[MAX_DROPS_PER_GROUP * MAX_GROUPS];
static uint16_t g_cal_record_count = 0;  /* 已记录的总滴数 */

/* 当前分组信息 */
#define GROUP_NAME_LEN 16
static char g_cal_group_name[GROUP_NAME_LEN] = "default";
static uint16_t g_cal_group_idx = 0;       /* 当前组编号 */
static uint16_t g_cal_group_drop_count = 0; /* 当前组内已记录滴数 */

/* 串口命令接收缓冲区（接收上位机命令） */
#define USART_CMD_BUF_SIZE  32
static char g_usart_cmd_buf[USART_CMD_BUF_SIZE];
static uint8_t g_usart_cmd_len = 0;
static uint8_t g_live_stream_enable = 0; /* 默认关闭阻塞式波形输出，避免影响采样处理 */

/* 系统运行时间（毫秒） */
volatile uint32_t g_sys_time_ms = 0;

/* ========== 函数声明 ========== */
static void IWDG_Init(void);
static void USART1_Config(void);
static void USART1_SendByte(uint8_t b);
static void USART1_SendFloat_WithTail(float v);
static void USART1_SendString(const char *str);
static void USART1_SendUint32(uint32_t val);
static void USART1_SendInt32(int32_t val);
static uint32_t Float_To_U32_Clamp(float v, uint32_t max_val);
static uint32_t Samples_To_Ms(uint32_t samples);
static void Send_Live_Stream(void);
static void Send_Debug_Statistics(void);
static float Compute_Intensity_MMH(void);
void Update_Display(void);
void Check_System_Status(void);

/* 标定相关 */
static void CAL_ProcessCommand(const char *cmd, uint8_t len);
static void CAL_UpdateDisplay(void);
static void CAL_PrintADCStatus(void);
static void CAL_RecordDrop(uint32_t nr, uint16_t hf, float impulse, uint32_t dE);
void WE_Calibration_EventCallback(
    WE_EventType_t ev_type,
    uint32_t norm_ratio,
    uint16_t hf_ratio,
    float impulse,
    uint32_t energy_delta,
    uint32_t drop_seq);

/* ========== OLED 显示缓存 ========== */
typedef struct {
    uint32_t cur_energy;       /* 当前帧高频能量 */
    uint32_t norm_ratio;         /* 最近一次归一化比值（×1000permille） */
    uint16_t hf_ratio;          /* 最近一次高频比例（permille） */
    uint32_t baseline_energy;    /* 基线能量（均值） */
    uint32_t threshold_energy;  /* 检测阈值 */
    uint16_t dc_baseline;       /* 传感器直流偏置（ADC原始值） */
    uint8_t  learning_progress; /* 学习进度（0-100） */
    uint8_t  learning_done;    /* 学习完成标志 */
    uint8_t  event_type;      /* 当前事件类型 */
} OLEDBuf_t;
static OLEDBuf_t g_oled = {0};

/* ========== 主函数 ========== */
int main(void)
{
    Delay_Init();
    OLED_Init();
    AD_Init();
    USART1_Config();
    Raindrop_Init();
    g_total_volume_0p01mm3 = Raindrop_GetTotalVolume_0p01mm3();
    SPI_SlaveLink_Init(MODBUS_SLAVE_ID);

    /* 实测 ADC 采样频率（粗略估算：等待 DMA 稳定后统计 100ms 内的 tick 数）
     * sampling_tick_counter 在 DMA ISR 中每收到一个 PA0 样本递增一次
     * 注意：如果 sampling_tick_counter 在 AD_Init 后立即开始递增（DMA 已在运行），
     *   则 100ms 后的值即为这段时间内的采样数，乘以 10 即为 Hz */
    uint32_t tick_before = sampling_tick_counter;
    Delay_ms(100);
    uint32_t tick_after = sampling_tick_counter;
    uint32_t actual_sample_rate = (tick_after - tick_before) * 10U; /* ×10 因为统计了 100ms */
    if (actual_sample_rate == 0) actual_sample_rate = 23810U; /* 降级处理 */

    WaveletEnergy_InitWithSampleRate(&g_we_ctx, WE_LEARNING_PERIOD_SEC, actual_sample_rate);
    Modbus_Slave_Init(MODBUS_SLAVE_ID, 9600);
    TF_Comm_Init();

    /* OLED 静态标签（固定不变） */
    /* 行1：E:xxxxx + 类型 */
    OLED_ShowString(1, 1, "E:");
    /* 行2：D:xxxxx（直流）+ B:xxxx（基线能量） */
    OLED_ShowString(2, 1, "D:");
    OLED_ShowString(2, 9, "B:");
    /* 行3：Cnt:xxx */
    OLED_ShowString(3, 1, "Cnt:");
    /* 行4：Hf:xxx + T:xxxxx */
    OLED_ShowString(4, 1, "Hf:");
    OLED_ShowString(4, 9, "T:");

    IWDG_Init();
    IWDG_ReloadCounter();

    while (1)
    {
        /* ===== 批量处理 ADC 采样数据（小波能量积分，移出 ISR） ===== */
        WaveletEnergy_ProcessNewSamples(&g_we_ctx);

        /* ===== 串口命令接收 ===== */
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        {
            uint8_t b = (uint8_t)USART_ReceiveData(USART1);
            if (b == '\r' || b == '\n')
            {
                if (g_usart_cmd_len > 0)
                {
                    g_usart_cmd_buf[g_usart_cmd_len] = '\0';
                    CAL_ProcessCommand(g_usart_cmd_buf, g_usart_cmd_len);
                    g_usart_cmd_len = 0;
                }
            }
            else if (g_usart_cmd_len < USART_CMD_BUF_SIZE - 1)
            {
                /* 大写命令统一转为小写方便匹配 */
                if (b >= 'A' && b <= 'Z')
                    b = b - 'A' + 'a';
                g_usart_cmd_buf[g_usart_cmd_len++] = (char)b;
            }
        }
        /* ===== 小波能量积分事件处理 ===== */
        if (g_we_ctx.learning_done && g_we_ctx.rain_event_pending)
        {
            g_we_ctx.rain_event_pending = 0;

            uint32_t norm = WaveletEnergy_GetNormRatio(&g_we_ctx);
            uint16_t hf  = WaveletEnergy_GetHFRatio(&g_we_ctx);

            /* 体积换算：统一使用 raindrop.c 的冲量查表法 */
            uint32_t vol = Raindrop_ImpulseToVolume(
                g_we_ctx.impulse_rain,
                WaveletEnergy_GetBaselineEnergy(&g_we_ctx)
            );

            if (vol > 0)
            {
                Raindrop_AddVolume(vol);
                g_total_volume_0p01mm3 = Raindrop_GetTotalVolume_0p01mm3();

                if (sec_index < SECONDS_WINDOW)
                    drops_per_second[sec_index]++;

                uint32_t energy = WaveletEnergy_GetCurrentEnergy(&g_we_ctx);
                TF_Comm_SendEvent(1, (uint16_t)energy, energy, 0, 1);
            }

            /* 标定模式：雨滴检测计数递增 + 记录数据 */
            g_cal_detected_count++;
            CAL_RecordDrop(norm, hf, g_we_ctx.impulse_rain, g_we_ctx.energy_delta);

            /* OLED 刷新缓存（每次RAIN事件刷新，稳定显示） */
            g_oled.norm_ratio    = norm;
            g_oled.hf_ratio   = hf;
            g_oled.event_type = WE_EVENT_RAIN;
        }
        else if (g_we_ctx.learning_done)
        {
            /* 检测期但无事件：清除事件类型标记 */
            if (WaveletEnergy_PopVibEvent(&g_we_ctx))
            {
                g_cal_vib_count++;  /* 标定模式：振动计数递增 */
            }
            (void)WaveletEnergy_PopNoiseEvent(&g_we_ctx);
            g_oled.event_type = WE_EVENT_NONE;
        }

        /* ===== 显示更新（每200ms，由硬件定时器驱动） ===== */
        if ((g_sys_time_ms - last_display_update_ms) >= 200)
        {
            last_display_update_ms = g_sys_time_ms;
#if WE_CALIBRATION_MODE
            CAL_UpdateDisplay();
#else
            Update_Display();
#endif
        }

        /* ===== 系统状态检查（每1秒） ===== */
        if ((g_sys_time_ms - last_syscheck_update_ms) >= 1000)
        {
            last_syscheck_update_ms = g_sys_time_ms;
            Check_System_Status();
            current_intensity_mmh = Compute_Intensity_MMH();

            /* 调试统计输出（每10秒） */
            if ((g_sys_time_ms - last_debug_counter_ms) >= 10000)
            {
                last_debug_counter_ms = g_sys_time_ms;
                Send_Debug_Statistics();
            }
        }

        /* ===== 秒索引环形窗口推进（每1秒，由 g_sys_time_ms 驱动） ===== */
        if ((g_sys_time_ms - last_sec_index_update_ms) >= 1000)
        {
            last_sec_index_update_ms = g_sys_time_ms;
            sec_index = (sec_index + 1) % SECONDS_WINDOW;
            drops_per_second[sec_index] = 0;
        }

        if (g_live_stream_enable)
            Send_Live_Stream();
        IWDG_ReloadCounter();
        Modbus_Slave_Process();
        Raindrop_UpdateRefractory();
        Delay_ms(10);
    }
}

/* ========== 显示更新（200ms周期） ========== */
void Update_Display(void)
{
    uint8_t progress = WaveletEnergy_GetLearningProgress(&g_we_ctx);
    uint8_t done    = g_we_ctx.learning_done;

    /* 刷新所有缓存（每200ms更新一次） */
    g_oled.cur_energy        = WaveletEnergy_GetCurrentEnergy(&g_we_ctx);
    g_oled.norm_ratio        = WaveletEnergy_GetNormRatio(&g_we_ctx);
    g_oled.baseline_energy   = WaveletEnergy_GetBaselineEnergy(&g_we_ctx);
    g_oled.threshold_energy  = WaveletEnergy_GetThreshold(&g_we_ctx);
    g_oled.dc_baseline      = g_we_ctx.dc_baseline;
    g_oled.learning_progress = progress;
    g_oled.learning_done     = done;
    if (done && g_oled.event_type != WE_EVENT_RAIN)
        g_oled.event_type = WaveletEnergy_GetEventType(&g_we_ctx);

    /* ===== 行1：E:xxxxx ===== */
    /* 实时高频能量（每200ms刷新），学习期指示进度 */
    {
        uint32_t e = g_oled.cur_energy;
        if (e > 99999) e = 99999;
        OLED_ShowNum(1, 3, e, 5);

        /* 状态标识 */
        if (!done)
        {
            OLED_ShowChar(1, 10, 'L');
            OLED_ShowNum(1, 11, progress, 3);
            OLED_ShowChar(1, 14, '%');
            OLED_ShowChar(1, 15, ' ');
            OLED_ShowChar(1, 16, ' ');
        }
        else
        {
            WE_EventType_t ev = (WE_EventType_t)g_oled.event_type;
            if (ev == WE_EVENT_RAIN)
                OLED_ShowChar(1, 10, 'R');
            else if (ev == WE_EVENT_VIB)
                OLED_ShowChar(1, 10, 'V');
            else if (ev == WE_EVENT_NOISE)
                OLED_ShowChar(1, 10, 'N');
            else
                OLED_ShowChar(1, 10, '-');
            OLED_ShowString(1, 11, "ok  ");
        }
    }

    /* ===== 行2：D:xxxxx + B:xxxx ===== */
    /* 直流偏置（ADC原始值）+ 基线能量 */
    {
        uint16_t dc = g_oled.dc_baseline;
        if (dc > 4095) dc = 4095;
        OLED_ShowNum(2, 3, dc, 4);

        uint32_t base = g_oled.baseline_energy;
        if (base > 9999) base = 9999;
        OLED_ShowNum(2, 12, base, 4);
    }

    /* ===== 行3：Cnt:xxx ===== */
    /* 雨滴计数 */
    {
        uint32_t cnt = g_we_ctx.rain_count;
        if (cnt > 999) cnt = 999;
        OLED_ShowNum(3, 5, cnt, 3);
        /* 清空行3剩余 */
        OLED_ShowChar(3, 8, ' ');
        OLED_ShowChar(3, 9, ' ');
        OLED_ShowChar(3, 10, ' ');
        OLED_ShowChar(3, 11, ' ');
        OLED_ShowChar(3, 12, ' ');
    }

    /* ===== 行4：Hf:xxx + T:xxxxx ===== */
    /* 高频比例（permille）+ 检测阈值 */
    {
        uint16_t hf = g_oled.hf_ratio;
        if (hf > 999) hf = 999;
        OLED_ShowNum(4, 4, hf, 3);

        uint32_t thr = g_oled.threshold_energy;
        if (thr > 99999) thr = 99999;
        OLED_ShowNum(4, 12, thr, 5);
    }
}

/* ========== 系统状态检查 ========== */
void Check_System_Status(void)
{
    if (sampling_tick_counter == last_sampling_tick)
    {
        AD_Restart();
    }
    last_sampling_tick = sampling_tick_counter;
}

/* ========== 独立看门狗 ========== */
static void IWDG_Init(void)
{
    uint32_t timeout;
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD_VALUE);
    timeout = 1000;
    while ((IWDG_GetFlagStatus(IWDG_FLAG_PVU) != RESET) && (timeout > 0)) timeout--;
    timeout = 1000;
    while ((IWDG_GetFlagStatus(IWDG_FLAG_RVU) != RESET) && (timeout > 0)) timeout--;
    IWDG_Enable();
    IWDG_ReloadCounter();
}

/* ========== 降雨强度计算 ========== */
static float Compute_Intensity_MMH(void)
{
    uint32_t sum = 0;
    uint8_t i;
    for (i = 0; i < SECONDS_WINDOW; i++)
        sum += drops_per_second[i];
    return (float)sum * 0.1f;
}

/* ========== USART1（VOFA+） ========== */
static void USART1_Config(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
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

static void USART1_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, b);
}

static void USART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART1_SendByte(*str);
        str++;
    }
}

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
        uint32_t temp = val;
        uint8_t digits = 0;
        while (temp > 0) { temp /= 10; digits++; }
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

static uint32_t Float_To_U32_Clamp(float v, uint32_t max_val)
{
    if (v <= 0.0f)
        return 0;
    if (v >= (float)max_val)
        return max_val;
    return (uint32_t)(v + 0.5f);
}

static uint32_t Samples_To_Ms(uint32_t samples)
{
    uint32_t sample_rate = g_we_ctx.actual_sample_rate_hz;
    if (sample_rate == 0)
        sample_rate = 23810U;
    return (uint32_t)(((uint64_t)samples * 1000ULL + sample_rate / 2U) / sample_rate);
}

static void USART1_SendFloat_WithTail(float v)
{
    const uint8_t jf_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    union { float f; uint8_t b[4]; } u;
    u.f = v;
    USART1_SendByte(u.b[0]);
    USART1_SendByte(u.b[1]);
    USART1_SendByte(u.b[2]);
    USART1_SendByte(u.b[3]);
    USART1_SendByte(jf_tail[0]);
    USART1_SendByte(jf_tail[1]);
    USART1_SendByte(jf_tail[2]);
    USART1_SendByte(jf_tail[3]);
}

/* ========== 连续示波输出 ========== */
static void Send_Live_Stream(void)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t ring_write_index_ch0;
    const uint8_t max_points = 10;
    static uint16_t last_index = 0;
    uint16_t write_idx = ring_write_index_ch0;
    uint16_t available;
    if (write_idx >= last_index)
        available = write_idx - last_index;
    else
        available = (uint16_t)(RING_BUFFER_SIZE - last_index + write_idx);
    uint16_t to_send = (available > max_points) ? max_points : available;
    for (uint16_t i = 0; i < to_send; i++)
    {
        uint16_t idx = (last_index + i) % RING_BUFFER_SIZE;
        float v = (float)adc_ring_buffer_ch0[idx] / 4095.0f * 3.3f;
        USART1_SendFloat_WithTail(v);
    }
    last_index = (uint16_t)((last_index + to_send) % RING_BUFFER_SIZE);
}

/* ========== 调试统计输出（每秒） ========== */
static void Send_Debug_Statistics(void)
{
    USART1_SendString("=== WE Debug ===\r\n");
    /* 学习状态 */
    USART1_SendString("learn=");
    USART1_SendUint32(WaveletEnergy_GetLearningProgress(&g_we_ctx));
    USART1_SendString("% done=");
    USART1_SendString(g_we_ctx.learning_done ? "Y" : "N");
    USART1_SendString("\r\n");
    /* 直流偏置（最重要：ADC原始值，应在学习期稳定） */
    USART1_SendString("dc_baseline=");
    USART1_SendUint32(g_we_ctx.dc_baseline);
    USART1_SendString(" (ADC raw, should be stable during learning)\r\n");
    /* 能量参数 */
    USART1_SendString("baseline_energy=");
    USART1_SendUint32(WaveletEnergy_GetBaselineEnergy(&g_we_ctx));
    USART1_SendString("\r\n");
    USART1_SendString("threshold=");
    USART1_SendUint32(WaveletEnergy_GetThreshold(&g_we_ctx));
    USART1_SendString("\r\n");
    USART1_SendString("cur_energy=");
    USART1_SendUint32(WaveletEnergy_GetCurrentEnergy(&g_we_ctx));
    USART1_SendString("\r\n");
    USART1_SendString("sample_rate=");
    USART1_SendUint32(g_we_ctx.actual_sample_rate_hz);
    USART1_SendString(" Hz\r\n");
    /* 事件分类 */
    USART1_SendString("hf_ratio=");
    USART1_SendUint32(WaveletEnergy_GetHFRatio(&g_we_ctx));
    USART1_SendString(" permille (rain<400, vib>450)\r\n");
    USART1_SendString("norm_ratio=");
    USART1_SendUint32(WaveletEnergy_GetNormRatio(&g_we_ctx));
    USART1_SendString(" permille\r\n");
    USART1_SendString("last_impulse=");
    USART1_SendUint32(Float_To_U32_Clamp(g_we_ctx.impulse_rain, 0xFFFFFFFFU));
    USART1_SendString(" width_ms=");
    USART1_SendUint32(Samples_To_Ms(g_we_ctx.last_event_raw_samples));
    USART1_SendString("\r\n");
    /* 统计（未标定） */
    USART1_SendString("rain_cnt=");
    USART1_SendUint32(g_we_ctx.rain_count);
    USART1_SendString(" vib=");
    USART1_SendUint32(g_we_ctx.vib_count);
    USART1_SendString(" noise=");
    USART1_SendUint32(g_we_ctx.noise_count);
    USART1_SendString(" total=");
    USART1_SendUint32(g_we_ctx.total_events);
    USART1_SendString("\r\n");
    USART1_SendString("sampling_tick=");
    USART1_SendUint32(sampling_tick_counter);
    USART1_SendString("\r\n");
    USART1_SendString("batch_last=");
    USART1_SendUint32(g_we_ctx.last_available_samples);
    USART1_SendString(" max=");
    USART1_SendUint32(g_we_ctx.max_available_samples);
    USART1_SendString(" decim=");
    USART1_SendUint32(g_we_ctx.process_decimation);
    USART1_SendString("\r\n");
    USART1_SendString("overrun=");
    USART1_SendUint32(g_we_ctx.sample_overrun_count);
    USART1_SendString(" dropped=");
    USART1_SendUint32(g_we_ctx.dropped_sample_count);
    USART1_SendString(" windows=");
    USART1_SendUint32(g_we_ctx.processed_window_count);
    USART1_SendString("\r\n");
    USART1_SendString("==============\r\n");
}

/* ========== 发送有符号32位整数 ========== */
static void USART1_SendInt32(int32_t val)
{
    char num_str[16];
    uint8_t idx = 0;
    if (val < 0)
    {
        USART1_SendByte('-');
        val = -val;
    }
    if (val == 0)
    {
        num_str[idx++] = '0';
    }
    else
    {
        uint8_t digits = 0;
        uint32_t temp = (uint32_t)val;
        while (temp > 0) { temp /= 10; digits++; }
        idx = digits;
        temp = (uint32_t)val;
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

/* ========== 记录一滴数据 ========== */
/**
  * @brief  记录一滴检测数据（存入内存，供后续EXPORT）
  * @param  nr       归一化比值（×1000permille）
  * @param  hf       高频比例（permille）
  * @param  impulse  冲量（ADC·次）
  * @param  dE       能量增量
  */
static void CAL_RecordDrop(uint32_t nr, uint16_t hf, float impulse, uint32_t dE)
{
    if (g_cal_record_count >= MAX_DROPS_PER_GROUP * MAX_GROUPS)
        return; /* 记录满了，丢弃 */

    uint16_t idx = g_cal_record_count;
    g_cal_records[idx].valid       = 1;
    g_cal_records[idx].group_idx   = g_cal_group_idx;
    g_cal_records[idx].seq_in_group = g_cal_group_drop_count + 1;
    g_cal_records[idx].nr          = nr;
    g_cal_records[idx].hf          = hf;
    g_cal_records[idx].impulse     = impulse;
    g_cal_records[idx].event_samples = g_we_ctx.last_event_raw_samples;
    g_cal_records[idx].dE         = dE;
    g_cal_records[idx].time_ms    = g_sys_time_ms;

    g_cal_record_count++;
    g_cal_group_drop_count++;

    /* 串口实时输出该滴数据 */
    USART1_SendString("[");
    USART1_SendString(g_cal_group_name);
    USART1_SendString(":");
    char seq_str[8];
    uint32_t seq = g_cal_group_drop_count;
    seq_str[0] = (seq >= 100) ? ('0' + (seq / 100) % 10) : ' ';
    seq_str[1] = (seq >= 10)  ? ('0' + (seq / 10) % 10)  : ' ';
    seq_str[2] = '0' + seq % 10;
    seq_str[3] = '\0';
    USART1_SendString(seq_str);
    USART1_SendString("] nr=");
    USART1_SendUint32(nr);
    USART1_SendString(" hf=");
    USART1_SendUint32(hf);
    USART1_SendString(" imp=");
    USART1_SendUint32(Float_To_U32_Clamp(impulse, 0xFFFFFFFFU));
    USART1_SendString(" width_ms=");
    USART1_SendUint32(Samples_To_Ms(g_we_ctx.last_event_raw_samples));
    USART1_SendString(" dE=");
    USART1_SendUint32(dE);
    USART1_SendString("\r\n");
}

/* ========== 标定命令处理 ========== */
/**
  * @brief  解析并执行上位机发送的标定命令
  * @param  cmd  命令字符串（小写）
  * @param  len  命令长度
  * @retval 无
  *
  * 支持的命令：
  *   SETN name  - 设置当前针头名称（最多15字符），同时重置该组计数
  *   STAT       - 输出当前统计
  *   EXPORT     - 导出全部记录数据
  *   RESET      - 重置所有记录
  *   STREAM ON/OFF - 打开/关闭USART1实时波形输出
  *   HELP       - 输出命令帮助
  */
static void CAL_PrintADCStatus(void)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t ring_write_index_ch0;
    extern volatile uint32_t ring_write_total_ch0;

    uint16_t write_idx = ring_write_index_ch0;
    uint16_t latest_idx = (write_idx == 0) ? (RING_BUFFER_SIZE - 1) : (write_idx - 1);
    uint16_t latest = adc_ring_buffer_ch0[latest_idx];
    uint16_t min_v = 0xFFFFU;
    uint16_t max_v = 0;
    uint32_t sum = 0;

    for (uint16_t i = 0; i < 64; i++)
    {
        uint16_t idx = (uint16_t)((write_idx + RING_BUFFER_SIZE - 1 - i) % RING_BUFFER_SIZE);
        uint16_t v = adc_ring_buffer_ch0[idx];
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
        sum += v;
    }

    USART1_SendString("[ADC] samples=");
    USART1_SendUint32(ring_write_total_ch0);
    USART1_SendString(" latest=");
    USART1_SendUint32(latest);
    USART1_SendString(" min64=");
    USART1_SendUint32(min_v);
    USART1_SendString(" max64=");
    USART1_SendUint32(max_v);
    USART1_SendString(" avg64=");
    USART1_SendUint32(sum / 64U);
    USART1_SendString(" learn=");
    USART1_SendUint32(WaveletEnergy_GetLearningProgress(&g_we_ctx));
    USART1_SendString("% done=");
    USART1_SendString(g_we_ctx.learning_done ? "Y" : "N");
    USART1_SendString("\r\n");
}

static void CAL_ProcessCommand(const char *cmd, uint8_t len)
{
    if (len == 0) return;

    if (strncmp(cmd, "adc", len) == 0)
    {
        CAL_PrintADCStatus();
        return;
    }

    /* SETN name：设置针头名称并新建分组 */
    if (len >= 5 && strncmp(cmd, "setn ", 5) == 0)
    {
        const char *name = cmd + 5;
        uint8_t name_len = len - 5;
        if (name_len > 8) name_len = 8;

        g_cal_group_idx++;
        g_cal_group_drop_count = 0;
        for (uint8_t i = 0; i < name_len; i++)
            g_cal_group_name[i] = name[i];
        g_cal_group_name[name_len] = '\0';

        USART1_SendString("[CAL] SETN ok: group=");
        USART1_SendString(g_cal_group_name);
        USART1_SendString(" (#");
        USART1_SendUint32(g_cal_group_idx);
        USART1_SendString(")\r\n");
        return;
    }

    /* STAT 命令：输出当前统计 */
    if (strncmp(cmd, "stat", len) == 0)
    {
        USART1_SendString("[CAL] === Statistics ===\r\n");
        USART1_SendString("group=");
        USART1_SendString(g_cal_group_name);
        USART1_SendString(" (#");
        USART1_SendUint32(g_cal_group_idx);
        USART1_SendString(")\r\n");
        USART1_SendString("group_drops=");
        USART1_SendUint32(g_cal_group_drop_count);
        USART1_SendString("\r\ntotal_drops=");
        USART1_SendUint32(g_cal_record_count);
        USART1_SendString("\r\nvib_count=");
        USART1_SendUint32(g_cal_vib_count);
        USART1_SendString("\r\n");
        if (g_cal_record_count > 0)
        {
            USART1_SendString("last_nr=");
            USART1_SendUint32(g_we_ctx.norm_ratio);
            USART1_SendString("\r\nlast_imp=");
            USART1_SendUint32(Float_To_U32_Clamp(g_we_ctx.impulse_rain, 0xFFFFFFFFU));
            USART1_SendString("\r\nlast_width_ms=");
            USART1_SendUint32(Samples_To_Ms(g_we_ctx.last_event_raw_samples));
            USART1_SendString("\r\n");
        }
        USART1_SendString("sys_time=");
        USART1_SendUint32(g_sys_time_ms);
        USART1_SendString("ms\r\n");
        USART1_SendString("batch_last=");
        USART1_SendUint32(g_we_ctx.last_available_samples);
        USART1_SendString(" max=");
        USART1_SendUint32(g_we_ctx.max_available_samples);
        USART1_SendString(" decim=");
        USART1_SendUint32(g_we_ctx.process_decimation);
        USART1_SendString("\r\noverrun=");
        USART1_SendUint32(g_we_ctx.sample_overrun_count);
        USART1_SendString(" dropped=");
        USART1_SendUint32(g_we_ctx.dropped_sample_count);
        USART1_SendString("\r\n");
        USART1_SendString("======================\r\n");
        return;
    }

    /* EXPORT 命令：导出全部记录数据 */
    if (strncmp(cmd, "export", len) == 0)
    {
        USART1_SendString("[CAL] === EXPORT START ===\r\n");
        USART1_SendString("# group    seq    impulse      width_ms   nr      hf     dE       time_ms\r\n");
        for (uint16_t i = 0; i < g_cal_record_count; i++)
        {
            CalDropRecord_t *r = &g_cal_records[i];
            USART1_SendUint32(i + 1);
            USART1_SendString("  ");
            /* 组名右对齐8字符 */
            uint8_t nl = 0;
            while (g_cal_group_name[nl] != '\0' && nl < 8) nl++;
            for (uint8_t s = 0; s < 8 - nl; s++) USART1_SendByte(' ');
            USART1_SendString(g_cal_group_name);
            USART1_SendString("  ");
            /* 组内序号 */
            if (r->seq_in_group < 100) USART1_SendByte(' ');
            if (r->seq_in_group < 10)  USART1_SendByte(' ');
            USART1_SendUint32(r->seq_in_group);
            USART1_SendString("   ");
            /* impulse */
            USART1_SendUint32(Float_To_U32_Clamp(r->impulse, 0xFFFFFFFFU));
            USART1_SendString("     ");
            /* width_ms */
            USART1_SendUint32(Samples_To_Ms(r->event_samples));
            USART1_SendString("     ");
            /* nr */
            USART1_SendUint32(r->nr);
            USART1_SendString("    ");
            /* hf */
            USART1_SendUint32(r->hf);
            USART1_SendString("     ");
            /* dE */
            USART1_SendUint32(r->dE);
            USART1_SendString("     ");
            /* time_ms */
            USART1_SendUint32(r->time_ms);
            USART1_SendString("\r\n");
        }
        USART1_SendString("[CAL] === EXPORT END (");
        USART1_SendUint32(g_cal_record_count);
        USART1_SendString(" records) ===\r\n");
        return;
    }

    /* RESET 命令：重置所有记录 */
    if (strncmp(cmd, "reset", len) == 0)
    {
        for (uint16_t i = 0; i < MAX_DROPS_PER_GROUP * MAX_GROUPS; i++)
            g_cal_records[i].valid = 0;
        g_cal_record_count = 0;
        g_cal_group_idx = 0;
        g_cal_group_drop_count = 0;
        g_cal_detected_count = 0;
        g_cal_vib_count = 0;
        g_cal_drop_count = 0;
        for (uint8_t i = 0; i < 7; i++) g_cal_group_name[i] = "default"[i];
        g_cal_group_name[7] = '\0';
        USART1_SendString("[CAL] RESET ok. All records cleared.\r\n");
        return;
    }

    /* STREAM ON/OFF：控制阻塞式实时波形输出 */
    if (len == 9 && strncmp(cmd, "stream on", 9) == 0)
    {
        g_live_stream_enable = 1;
        USART1_SendString("[CAL] STREAM on\r\n");
        return;
    }
    if (len == 10 && strncmp(cmd, "stream off", 10) == 0)
    {
        g_live_stream_enable = 0;
        USART1_SendString("[CAL] STREAM off\r\n");
        return;
    }

    /* HELP 命令 */
    if (strncmp(cmd, "help", len) == 0)
    {
        USART1_SendString("[CAL] === Commands ===\r\n");
        USART1_SendString("SETN name - set needle name\r\n");
        USART1_SendString("ADC       - show raw ADC status\r\n");
        USART1_SendString("STAT      - show stats\r\n");
        USART1_SendString("EXPORT    - dump all data\r\n");
        USART1_SendString("RESET     - clear all\r\n");
        USART1_SendString("STREAM ON/OFF - waveform\r\n");
        USART1_SendString("==============\r\n");
        return;
    }

    USART1_SendString("[CAL] unknown cmd: ");
    for (uint8_t i = 0; i < len; i++)
        USART1_SendByte((uint8_t)cmd[i]);
    USART1_SendString(". Try HELP.\r\n");
}

/* ========== 标定模式 OLED 显示 ========== */
/**
  * @brief  标定模式 OLED 显示（每200ms刷新）
  *
  * OLED 布局（4行×16字符）：
  *   行1：[small]  xxx  - 当前组名 + 组内滴数
  *   行2：nr:xxxx hf:xxx - 最近一次 norm_ratio 和 hf_ratio
  *   行3：cnt:RRRR          - 总检测雨滴数
  *   行4：tot:RRRR          - 总记录滴数
  */
static void CAL_UpdateDisplay(void)
{
    /* 行1：组名 + 组内计数 */
    OLED_ShowString(1, 1, "[");
    uint8_t nl = 0;
    while (g_cal_group_name[nl] != '\0' && nl < 8) nl++;
    for (uint8_t s = 0; s < 8 - nl; s++) OLED_ShowChar(1, 2 + s, ' ');
    OLED_ShowString(1, 10 - nl, g_cal_group_name);
    OLED_ShowString(1, 10, "]");
    if (!g_we_ctx.learning_done)
    {
        uint8_t prog = WaveletEnergy_GetLearningProgress(&g_we_ctx);
        OLED_ShowChar(1, 11, 'L');
        OLED_ShowNum(1, 12, prog, 3);
        OLED_ShowChar(1, 15, '%');
        OLED_ShowChar(1, 16, ' ');
    }
    else
    {
        uint32_t gc = g_cal_group_drop_count;
        if (gc > 99) gc = 99;
        OLED_ShowChar(1, 11, ' ');
        if (gc > 9) OLED_ShowNum(1, 12, gc, 2);
        else        { OLED_ShowChar(1, 12, ' '); OLED_ShowNum(1, 13, gc, 1); }
        OLED_ShowChar(1, 14, ' ');
        OLED_ShowChar(1, 15, ' ');
        OLED_ShowChar(1, 16, ' ');
    }

    /* 行2：nr + hf */
    OLED_ShowString(2, 1, "IMP:");
    uint32_t imp = Float_To_U32_Clamp(g_we_ctx.impulse_rain, 99999999U);
    OLED_ShowNum(2, 5, imp, 8);
    OLED_ShowString(2, 13, "    ");

    /* 行3：总雨滴检测数 */
    OLED_ShowString(3, 1, "W:");
    uint32_t width_ms = Samples_To_Ms(g_we_ctx.last_event_raw_samples);
    if (width_ms > 9999) width_ms = 9999;
    OLED_ShowNum(3, 3, width_ms, 4);
    OLED_ShowString(3, 7, "ms");
    OLED_ShowString(3, 10, "N:");
    uint32_t rc = g_cal_detected_count;
    if (rc > 999) rc = 999;
    OLED_ShowNum(3, 12, rc, 3);
    OLED_ShowChar(3, 15, ' ');
    OLED_ShowChar(3, 16, ' ');

    /* 行4：总记录数 */
    OLED_ShowString(4, 1, "E:");
    uint32_t de = g_we_ctx.energy_delta;
    if (de > 99999) de = 99999;
    OLED_ShowNum(4, 3, de, 5);
    OLED_ShowString(4, 9, "T:");
    uint32_t thr = g_we_ctx.threshold_energy;
    if (thr > 99999) thr = 99999;
    OLED_ShowNum(4, 11, thr, 5);

    /* 如果正在学习，显示进度（覆盖行1右侧） */
}

