# 压电式雨量传感器从控系统

## 架构设计说明书（ADS）

| 属性 | 内容 |
|------|------|
| 版本 | v1.0 |
| 编制日期 | 2026-04-24 |
| 状态 | 评审中 |
| 基于需求文档 | SRS v1.0 |

---

## 修订历史

| 版本 | 日期 | 修订人 | 变更说明 |
|------|------|--------|----------|
| v0.1 | 2026-04-24 | Cursor Agent | 初稿：基于代码分析重构架构 |

---

## 1. 设计目标

本文档定义压电式雨量传感器从控系统的**目标架构**，解决现有代码中的以下核心问题：

| 编号 | 问题 | 严重程度 | 影响 |
|------|------|----------|------|
| A-01 | 两套 Haar DWT 实现并存（`WaveletEnergy.c` + `Wavelet.c`） | 严重 | 维护成本加倍，行为不一致风险 |
| A-02 | DMA ISR 中执行完整小波计算 | 严重 | 中断嵌套风险，实时性差 |
| A-03 | TF_Comm 与 Modbus 共享 USART2，波特率冲突 | 严重 | 两套通信互相破坏 |
| A-04 | 两套体积换算路径并存，无统一出口 | 严重 | 计量结果不确定 |
| A-05 | Modbus 与 RegisterMap 重复实现寄存器读逻辑 | 中等 | 代码冗余，维护困难 |
| A-06 | 串口调试输出（VOFA+）与业务通信共用 USART1 | 中等 | 调试时干扰业务 |
| A-07 | `rainfall.c` 的 `UpdateRefractory()` 在主循环中从未调用 | 中等 | 死区控制失效 |
| A-08 | 全局变量跨模块隐式耦合 | 中等 | 并发安全风险 |
| A-09 | `raindrop.c` 的死区机制与小波能量死区各自独立 | 中等 | 双重死区导致漏检 |
| A-10 | 采样频率硬编码 23810，无实测校准 | 中等 | 学习期时长不准确 |

---

## 2. 架构原则

1. **分层清晰**：硬件驱动 → 信号处理 → 雨量计量 → 通信上报，每层职责单一
2. **中断尽量轻量化**：DMA ISR 只做数据搬运，信号处理移至主循环
3. **单总线单协议**：每个物理端口专用于一种通信协议，不混用
4. **单一体积出口**：雨量计量模块统一对外提供体积数据，消除双路径
5. **模块自治**：各模块通过良好定义的接口交互，禁止跨模块直接读写内部状态
6. **零动态分配**：所有缓冲区在编译期确定大小，无 malloc/free

---

## 3. 分层架构

```
┌────────────────────────────────────────────────────────────────────┐
│                        应用层 (App)                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐           │
│  │ RainfallMgr  │  │ Calibration   │  │ SystemMgr    │           │
│  │ 雨量管理器   │  │ 标定框架      │  │ 系统管理器   │           │
│  └──────────────┘  └──────────────┘  └──────────────┘           │
├────────────────────────────────────────────────────────────────────┤
│                      信号处理层 (Signal)                             │
│  ┌──────────────────────┐  ┌───────────────────────┐             │
│  │ WaveletProcessor      │  │ PeakDetector          │             │
│  │ 小波能量积分处理器    │  │ 峰值检测器（备选）     │             │
│  └──────────────────────┘  └───────────────────────┘             │
├────────────────────────────────────────────────────────────────────┤
│                      数据采集层 (Acquisition)                        │
│  ┌──────────────────────┐  ┌───────────────────────┐             │
│  │ ADCDriver            │  │ RingBuffer            │             │
│  │ ADC + DMA 驱动       │  │ 环形缓冲区（双口）    │             │
│  └──────────────────────┘  └───────────────────────┘             │
├────────────────────────────────────────────────────────────────────┤
│                      通信层 (Comms)                                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ ModbusRTU│  │ TFProto  │  │ SPILink  │  │ DebugUART│        │
│  │ USART2   │  │ USART3   │  │ SPI1     │  │ USART1   │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
├────────────────────────────────────────────────────────────────────┤
│                      硬件抽象层 (HAL)                              │
│  GPIO  TIM2  IWDG  OLED  LED  KEY                                │
└────────────────────────────────────────────────────────────────────┘
```

---

## 4. 硬件资源重分配

### 4.1 通信端口分配（关键变更）

| 端口 | 用途 | 波特率 | 说明 |
|------|------|--------|------|
| **USART1** | VOFA+ 调试（TX+RX） | 115200 | 仅调试，断开后不影响业务 |
| **USART2** | Modbus RTU 从站 | 9600 | 保留原设计，增加 RS-485 DE 控制 |
| **USART3** | TF 主动上报协议 | 115200 | **从 USART2 移出，独立波特率** |
| SPI1 | SPI 从机接口（主控 V6） | ≤ 2 Mbps | 保留原设计 |

> **设计决策：USART2 专用于 Modbus RTU，TF 协议迁移至 USART3。** 原因：Modbus RTU 和 TF 协议的帧格式、波特率、超时机制完全不同，共享同一 USART 会导致无法同时正确工作。

### 4.2 RS-485 方向控制

Modbus RTU 使用半双工 RS-485 总线，需要 GPIO 控制 DE（Driver Enable）引脚：

```
TX Enable (DE): PA1（复用推挽输出）
  - Modbus_Send_Response() 发送前：GPIO_SetBits(DE)
  - Modbus_Send_Response() 发送完成后：GPIO_ResetBits(DE)
  - 接收期间：DE = 0
```

### 4.3 完整引脚分配表

| 引脚 | 复用功能 | 方向 | 用途 |
|------|----------|------|------|
| PA0 | ADC1_CH0 | 输入 | 压电传感器信号 |
| PA1 | GPIO_OUTPUT | 输出 | RS-485 DE（Modbus TX Enable） |
| PA2 | USART2_TX | 复用推挽 | Modbus RTU TX / RS-485差分+ |
| PA3 | USART2_RX | 复用上拉 | Modbus RTU RX / RS-485差分- |
| PA4 | SPI1_NSS | 输入 | SPI 片选 |
| PA5 | SPI1_SCK | 输入 | SPI 时钟 |
| PA6 | SPI1_MISO | 复用推挽 | SPI 数据输出 |
| PA7 | SPI1_MOSI | 输入上拉 | SPI 数据输入 |
| PA9 | USART1_TX | 复用推挽 | VOFA+ 调试 TX |
| PA10 | USART1_RX | 复用上拉 | VOFA+ 调试 RX |
| PB0 | USART3_TX | 复用推挽 | TF 协议上报 TX |
| PB1 | USART3_RX | 复用上拉 | TF 协议上报 RX |
| PB5 | GPIO_OUTPUT | 输出 | 工作指示灯 |
| PE4 | GPIO_INPUT | 输入上拉 | 标定按键 |

---

## 5. 核心模块设计

### 5.1 数据采集层

#### 5.1.1 ADCDriver — 采集驱动（改造）

**职责**：配置 ADC1 双通道 DMA 循环采样，提供原始数据。

**改动点**：
1. 保持双通道 DMA 循环采样（PA0 + PA1）
2. DMA ISR **仅做数据搬运**，不再调用任何信号处理函数
3. 将采样数据写入环形缓冲区后设置"数据就绪"标志，由主循环触发信号处理
4. 增加采样频率实测机制：在初始化阶段统计实际采样频率，替代硬编码 23810

```c
// 初始化时实测采样频率（单位：Hz）
void ADC_CalibrateSampleRate(void)
{
    uint32_t start_tick = SysTick->VAL;  // 或使用 DWT->CYCCNT
    delay_us(100000);  // 精确延时 100ms
    uint32_t end_tick = SysTick->VAL;
    uint32_t elapsed = (start_tick - end_tick) & 0xFFFFFF;
    uint32_t actual_hz = (elapsed * 10) / (SystemCoreClock / 1000000);
    g_adc_actual_sample_rate = actual_hz;
}
```

**接口**：
```c
void ADCDriver_Init(void);
void ADCDriver_Start(void);
void ADCDriver_Restart(void);
uint32_t ADCDriver_GetSampleRate(void);  // 返回实测采样频率
volatile uint16_t* ADCDriver_GetRingBuffer(void);
volatile uint16_t* ADCDriver_GetWriteIndex(void);
```

#### 5.1.2 RingBuffer — 双口环形缓冲区

**职责**：ADC ISR 写入，主循环读取，提供无锁数据交换。

**设计**：
- 单生产者（DMA ISR）、单消费者（主循环），无需锁
- 写索引：volatile，由 DMA ISR 更新
- 读索引：普通变量，由主循环更新
- 容量：`RING_BUFFER_SIZE = 512`（2的幂，便于位运算）

```c
typedef struct {
    volatile uint16_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t write_idx;   // ISR 写，主循环读
    uint16_t read_idx;             // 主循环读写
} RingBuffer_t;
```

**关键不变量**：主循环读取数据时，必须在 DMA ISR 更新 `write_idx` 之前完成读取，避免读到正在写入的数据（主循环比 DMA ISR 慢，但需防止字分裂）。实际方案：读取时先记录 `write_idx` 快照，再基于快照读取，写索引原子读取（16位赋值天然原子）。

---

### 5.2 信号处理层

#### 5.2.1 WaveletProcessor — 小波处理器（重构）

**职责**：对小波能量积分算法的完整重构，解决溢出、死区混乱、采样率硬编码问题。

**关键改动点**：

**改动 1：消除能量溢出**

现有代码中 `uint32_t acc` 累加 64 点平方和后被截断。新设计使用 `uint64_t` 累加，并**在返回前取对数**压缩动态范围：

```c
static uint64_t Compute_Energy_SumSquares(int16_t *coeffs, uint16_t n)
{
    uint64_t acc = 0;
    for (uint16_t i = 0; i < n; i++)
    {
        int32_t v = (int32_t)coeffs[i];
        acc += (uint64_t)((int64_t)v * (int64_t)v);
    }
    return acc;  // 保留完整 64 位，传递到事件分类逻辑
}
```

如果担心 64 位运算慢，可以进一步优化：**对数压缩**。将 `log2(acc + 1)` 作为"对数能量"使用，动态范围从 ~10¹¹ 压缩到 ~40，完美适配 uint32：

```c
static uint32_t Compute_LogEnergy(int16_t *coeffs, uint16_t n)
{
    uint64_t acc = 0;
    for (uint16_t i = 0; i < n; i++)
    {
        int32_t v = (int32_t)coeffs[i];
        acc += (uint64_t)((int64_t)v * (int64_t)v);
    }
    // 取对数压缩动态范围
    uint32_t log_e = 0;
    while (acc > 1) { acc >>= 1; log_e++; }
    return log_e;
}
```

**改动 2：将采样频率从硬编码改为实测注入**

```c
typedef struct {
    uint32_t actual_sample_rate_hz;  // 实测采样频率，由 ADCDriver 提供
    uint32_t energy_sample_count;
    float learning_sec_actual;
    uint32_t required_samples_for_learning;
    uint8_t learning_done;
} WaveletProcessorCtx_t;

// 初始化时传入实测采样频率
void WaveletProcessor_Init(WaveletProcessorCtx_t *ctx, uint32_t actual_sample_rate_hz)
{
    ctx->actual_sample_rate_hz = actual_sample_rate_hz;
    ctx->required_samples_for_learning = actual_sample_rate_hz * WE_LEARNING_PERIOD_SEC;
}
```

学习期计数判断：
```c
// 旧代码（硬编码）：
if (ctx->energy_sample_count >= 23810U * WE_LEARNING_PERIOD_SEC)

// 新代码（实测）：
if (ctx->energy_sample_count >= ctx->required_samples_for_learning)
```

**改动 3：统一死区管理（消除双死区）**

现有代码中存在两个独立的死区机制：
1. `WaveletEnergy.deadtime_counter`（小波层，约 8.4 ms）
2. `Raindrop_UpdateRefractory()`（雨量层，50 ms，但从未被调用）

**设计决策：删除 `raindrop.c` 的独立死区机制，统一使用小波层的死区**。原因：体积换算已经在小波事件触发后进行，小波层的死区已经保护了同一次事件的重复触发。雨量层再加死区只会导致漏检。

```c
// raindrop.c 的改动：
// - 删除 g_refractory_counter 及其相关逻辑
// - 直接由 WaveletProcessor 传入事件，不做二次判断
void Raindrop_OnRainEvent(uint16_t adc_peak, uint32_t volume_0p01mm3)
{
    // 直接累加，无死区
    g_V_sum_0p01mm3 += volume_0p01mm3;
}
```

#### 5.2.2 PeakDetector — 峰值检测器（新增，备选）

**职责**：识别信号中的局部峰值，用于辅助判定和快照记录。

**设计**：当检测到连续 N 点上升后出现下降拐点，记录该点为峰值。这是传统压电雨量计的经典方法，可与小波方法互补。

```c
typedef struct {
    int16_t prev_sample;
    uint8_t rising_counter;
    uint8_t peak_confirmed;
} PeakDetectorCtx_t;

uint8_t PeakDetector_Update(PeakDetectorCtx_t *ctx, int16_t sample);
```

> **设计决策：保留为备选。** 峰值检测对信号质量要求高，容易受到基线漂移影响。优先使用小波方法，峰值检测结果仅作为参考。

---

### 5.3 雨量计量层

#### 5.3.1 RainfallMgr — 雨量管理器（重构，**单一体积出口**）

**职责**：将信号处理结果转换为雨量数据，提供统一的体积查询接口。

**核心设计决策：删除 `raindrop.c` 的电压→体积 LUT 路径**。

理由：
1. `raindrop.c` 基于原始 ADC 峰值电压，物理上对应的是"最大冲击力时的电压"
2. 小波能量基于 64 点积分窗口，物理上对应的是"整个撞击事件的能量"
3. 这两个量没有简单的一一对应关系，混用会导致计量误差

**保留 `raindrop.c` 中的哪些部分**：
- `Raindrop_VoltageToVolume_0p01mm3()` — 保留函数签名，作为**辅助参考**，不参与主计量路径
- `g_V_sum_0p01mm3` — 统一累计变量，由 `RainfallMgr` 写入

**新增接口**：
```c
void RainfallMgr_Init(void);
void RainfallMgr_OnRainEvent(WaveletProcessorEvent_t *ev);
uint32_t RainfallMgr_GetTotalVolume_0p01mm3(void);
uint32_t RainfallMgr_GetRainfallIntensity_MMH(void);  // mm/h
void RainfallMgr_ResetTotal(void);
```

#### 5.3.2 体积计算模型

基于实测标定数据的体积模型。设计为可替换的插件式结构：

```c
typedef uint32_t (*VolumeModelFunc)(uint32_t norm_ratio, uint16_t hf_ratio, float impulse);

typedef struct {
    VolumeModelFunc func;
    const char *name;       // "linear", "poly2", "lut"
    uint8_t calibrated;    // 是否已完成现场标定
} VolumeModel_t;

// 默认线性模型（待标定）
static uint32_t DefaultLinearModel(uint32_t norm_ratio, uint16_t hf_ratio, float impulse)
{
    uint32_t vol = (norm_ratio * VOLUME_SCALE + 5) / 10;
    if (vol > 5000) vol = 5000;
    return vol;
}

static VolumeModel_t g_volume_model = {
    .func = DefaultLinearModel,
    .name = "linear_uncalibrated",
    .calibrated = 0,
};

// 标定完成后切换模型
void RainfallMgr_SetVolumeModel(VolumeModelFunc func, const char *name)
{
    g_volume_model.func = func;
    g_volume_model.name = name;
    g_volume_model.calibrated = 1;
}
```

#### 5.3.3 降雨强度计算

```c
#define INTENSITY_WINDOW_SEC  60

static uint32_t drops_in_window[INTENSITY_WINDOW_SEC];
static uint8_t window_head = 0;

uint32_t RainfallMgr_GetRainfallIntensity_MMH(void)
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < INTENSITY_WINDOW_SEC; i++)
        total += drops_in_window[i];
    // 假设平均雨滴体积为 K mm³（待标定）
    // 强度 = total_drops × K / 60（mm/h）
    return (total * AVG_DROP_VOLUME_0p01mm3 + 5) / (10 * INTENSITY_WINDOW_SEC);
}
```

---

### 5.4 通信层

#### 5.4.1 ModbusRTU — Modbus 从站（改造）

**职责**：响应主站的 Modbus 轮询，提供只读寄存器访问。

**改动点**：

1. **统一使用 RegisterMap 作为唯一寄存器源** — 删除 `Modbus_Slave.c` 中的 `Read_Input_Reg()` 重复实现，所有寄存器读写经由 `RegisterMap`
2. **增加 RS-485 DE 方向控制**
3. **增加 USART2 独立初始化**（从 USART3 共享方案中分离）

```c
void ModbusRTU_Init(uint8_t slave_id, uint32_t baudrate)
{
    // 1. GPIO 配置（PA2=TX, PA3=RX, PA1=DE）
    // 2. USART2 配置（9600 8N1）
    // 3. RS-485 DE 默认低（接收模式）
    // 4. NVIC 配置（优先级低于 DMA）

    // 5. TIM2 1ms tick 初始化
    // 6. RegisterMap_Init(slave_id)
}
```

4. **增加 Modbus 0x10（写多寄存器）支持**，用于配置参数

#### 5.4.2 TFProto — TF 自定义协议（迁移至 USART3）

**职责**：主动向主控上报雨滴事件。

**迁移**：
- 从 USART2 迁移到 USART3（PB0=TX, PB1=RX）
- 配置 115200 8N1

```c
void TFProto_Init(void)
{
    // 1. GPIO 配置（PB0=TX, PB1=RX）
    // 2. USART3 配置（115200 8N1）
    // 3. TXE 中断初始化（优先级最低）
    // 4. 发送队列初始化
}
```

**发送触发**：主循环检测到 `rain_event_pending` 后调用：

```c
void TFProto_SendRainEvent(WaveletProcessorEvent_t *ev)
{
    TFProto_Frame_t frame = {
        .sof1 = 0xAA,
        .sof2 = 0x55,
        .src_id = TF_SRC_ID,
        .flag = ev->event_type,
        .peak = ev->peak_adc,
        .energy = ev->log_energy,
        .seq = g_tf_seq++,
    };
    frame.crc = CRC16_CCITT_FALSE((uint8_t*)&frame.sof1 + 2, 11);
    TFProto_Enqueue(&frame);
}
```

#### 5.4.3 SPILink — SPI 从机（保留）

保留现有设计，保持与主控 V6 的 SPI 通信。关键点：
- NSS 下降沿开启接收，上升沿结束事务
- 寄存器映射共享 `RegisterMap`（与 Modbus 共用）

#### 5.4.4 DebugUART — 调试串口（改造）

**职责**：VOFA+ 实时波形 + 调试统计输出。

**设计决策**：将调试输出与业务通信分离到不同 USART：
- USART1（PA9/PA10）：专用于 VOFA+ 调试
- 调试输出不影响业务通信（Modbus/TF）

```c
void DebugUART_Init(void)
{
    // USART1 115200 8N1
    // 仅 TX + RX 中断（优先级最低）
    // JustFloat 协议输出
}
```

---

### 5.5 应用层

#### 5.5.1 Calibration — 标定框架（重构）

**职责**：记录实验数据，支持多组标定实验，支持数据导出。

**改动点**：

1. **统一数据入口** — 标定数据从 `RainfallMgr` 的事件回调中记录，不再从小波层和雨量层各自独立记录
2. **数据持久化** — 增加 Flash 存储，标定结果掉电不丢失

```c
typedef struct {
    uint8_t valid;
    uint8_t group_idx;
    uint32_t seq_in_group;
    uint32_t nr;
    uint16_t hf;
    float impulse;
    uint32_t dE;
    uint32_t time_ms;
    uint16_t volume;           // 实际标定体积（0.01mm³），0=未标定
} CalRecord_t;

#define MAX_RECORDS  2000
static CalRecord_t g_cal_records[MAX_RECORDS];

void Calibration_OnEvent(WaveletProcessorEvent_t *ev)
{
    if (!WE_CALIBRATION_MODE) return;
    CalRecord_t *r = &g_cal_records[g_cal_record_count % MAX_RECORDS];
    r->valid = 1;
    r->nr = ev->norm_ratio;
    r->hf = ev->hf_ratio;
    r->impulse = ev->impulse;
    r->dE = ev->energy_delta;
    r->time_ms = get_system_time_ms();
    r->volume = 0;  // 等待人工输入实际体积
}
```

3. **增加"输入实际体积"命令**：

```
SETN <name>     — 新建分组（原有）
RECORD <vol>    — 为最后一条记录设置实际体积（vol 单位：0.01mm³）
STAT            — 输出统计（原有）
EXPORT          — 导出数据（原有）
RESET           — 重置（原有）
```

#### 5.5.2 SystemMgr — 系统管理器

**职责**：看门狗、看门狗计数、显示刷新、系统状态监控。

```c
void SystemMgr_Init(void)
{
    IWDG_Init();              // 3 秒超时
    OLED_Init();
    SystemMgr_UpdateDisplay(); // 初始显示
}

void SystemMgr_10msTick(void)
{
    IWDG_ReloadCounter();
    SystemMgr_CheckADCHealth();  // 检测 sampling_tick 是否正常推进
}

void SystemMgr_200msTick(void)
{
    SystemMgr_UpdateDisplay();
}

void SystemMgr_1sTick(void)
{
    RainfallMgr_PushToIntensityWindow();
    SystemMgr_SendDebugStats();
}
```

---

## 6. 主循环设计

### 6.1 调度框架

采用**固定节拍调度器**，以 10ms 为基本时间单元：

```c
typedef struct {
    uint16_t counter_10ms;
    uint8_t  counter_100ms;
    uint8_t  counter_1s;
    uint8_t  counter_10s;
} Scheduler_t;

static Scheduler_t g_sched = {0};

int main(void)
{
    // === 初始化阶段 ===
    HAL_Init();
    Clock_Init();
    ADCDriver_Init();
    ADC_CalibrateSampleRate();          // 新增：实测采样频率
    RingBuffer_Init();
    uint32_t sample_rate = ADCDriver_GetSampleRate();
    WaveletProcessor_Init(&g_wavelet_ctx, sample_rate);
    RainfallMgr_Init();
    ModbusRTU_Init(MODBUS_SLAVE_ID, 9600);
    TFProto_Init();
    SPILink_Init(MODBUS_SLAVE_ID);
    SystemMgr_Init();
    Calibration_Init();

    while (1)
    {
        // === 10ms 主节拍 ===
        g_sched.counter_10ms++;

        // 每 10ms：看门狗 + ADC 健康检测 + 通信处理
        SystemMgr_FeedWatchdog();
        SystemMgr_CheckADCHealth();
        ModbusRTU_Process();     // 帧判断 + 处理
        SPILink_Process();       // SPI 事务处理
        TFProto_ProcessTX();     // TF 发送队列推进

        // === 信号处理（每 10ms，约每 200 次采样调用一次）===
        // 注：信号处理频率不需要与主循环同步
        // 更好的方案见 6.2 节

        // === 调度子节拍 ===
        if (g_sched.counter_10ms % 10 == 0)    // 100ms
        {
            g_sched.counter_100ms++;
            SystemMgr_UpdateDisplay();
            RainfallMgr_UpdateDisplay();
        }

        if (g_sched.counter_10ms % 100 == 0)    // 1s
        {
            g_sched.counter_1s++;
            RainfallMgr_PushIntensityWindow();
            SystemMgr_SendDebugStats();
        }

        if (g_sched.counter_10ms % 1000 == 0)    // 10s
        {
            g_sched.counter_10s++;
            Calibration_FlushToFlash();   // 定期将标定记录写入 Flash
        }

        // 空闲时让 CPU 进入 sleep 模式（降低功耗）
        __WFI();  // Wait For Interrupt
    }
}
```

### 6.2 信号处理的触发策略（改进）

现有方案在 DMA ISR 中每采样一次就调用 `WaveletEnergy_Update()`（约 20k 次/秒），每次做完整的 3 层 Haar DWT。

**推荐方案：轮询 + 批处理**

在主循环中检查"新数据是否就绪"，然后批量处理：

```c
// 主循环中（约每 10ms 执行一次）
void SignalProcessing_Task(void)
{
    static uint16_t last_write_idx = 0;
    uint16_t curr_write_idx = *ADCDriver_GetWriteIndex();

    if (curr_write_idx == last_write_idx)
        return;  // 无新数据

    // 计算有多少个新样本
    uint16_t new_count;
    if (curr_write_idx >= last_write_idx)
        new_count = curr_write_idx - last_write_idx;
    else
        new_count = RING_BUFFER_SIZE - last_write_idx + curr_write_idx;

    // 每次主循环最多处理 N 个新样本，防止积压
    uint16_t to_process = (new_count > MAX_SAMPLES_PER_LOOP) ? MAX_SAMPLES_PER_LOOP : new_count;

    for (uint16_t i = 0; i < to_process; i++)
    {
        uint16_t idx = (last_write_idx + i) % RING_BUFFER_SIZE;
        WaveletProcessor_Update(&g_wavelet_ctx, ADCDriver_GetRingBuffer(), idx);
    }

    last_write_idx = (last_write_idx + to_process) % RING_BUFFER_SIZE;

    // 检查事件 pending
    if (g_wavelet_ctx.rain_event_pending)
    {
        g_wavelet_ctx.rain_event_pending = 0;
        RainfallMgr_OnRainEvent(&g_wavelet_ctx.current_event);
        TFProto_SendRainEvent(&g_wavelet_ctx.current_event);
        Calibration_OnEvent(&g_wavelet_ctx.current_event);
    }
}
```

**优点**：
1. 信号处理完全在主循环中执行，不占用 ISR 时间
2. 死区计数器在主循环中递减，精度有保证
3. 避免了 DMA ISR 调用复杂函数的实时性风险
4. `Rainfall_UpdateRefractory()` 自然在主循环中被调用

**缺点**：事件检测有最多 10ms 延迟。对于雨量计场景，这个延迟完全可接受（降雨是一个连续过程，毫秒级延迟不影响计量精度）。

---

## 7. 事件与数据流

```
ADC DMA 循环采样 (~20k Hz)
         │
         ▼
  RingBuffer[512]      （生产者：DMA ISR）
         │
         │ (主循环轮询)
         ▼
  WaveletProcessor
  ├── 去直流（学习期/检测期不同策略）
  ├── Haar DWT 三层分解
  ├── 对数能量计算（溢出安全）
  ├── 事件分类（RAIN / VIB / NOISE / SUSPECT）
  └── 设置 rain_event_pending
         │
         ├──▶ RainfallMgr
         │         ├── 体积计算（可替换模型）
         │         ├── 累计体积
         │         ├── 降雨强度
         │         └── 强度窗口更新
         │
         ├──▶ TFProto ──▶ USART3 ──▶ 主控
         │
         ├──▶ Calibration ──▶ RAM/Flash ──▶ EXPORT
         │
         └──▶ RegisterMap ──▶ ModbusRTU ──▶ USART2 ──▶ 主站
                           └──▶ SPILink ──▶ SPI1 ──▶ 主控 V6
```

---

## 8. 模块依赖关系（禁止循环依赖）

```
ADCDriver        ──无依赖──▶ （最底层）
RingBuffer       ──依赖──▶ ADCDriver
WaveletProcessor ──依赖──▶ RingBuffer
RainfallMgr      ──依赖──▶ WaveletProcessor
Calibration      ──依赖──▶ WaveletProcessor
RegisterMap      ──依赖──▶ WaveletProcessor, RainfallMgr
ModbusRTU        ──依赖──▶ RegisterMap
TFProto          ──依赖──▶ WaveletProcessor
SPILink          ──依赖──▶ RegisterMap
SystemMgr        ──依赖──▶ 所有模块（仅读状态）
```

---

## 9. 接口设计（API 契约）

### 9.1 WaveletProcessor — 核心算法模块

```c
// 初始化
void WaveletProcessor_Init(WaveletProcessorCtx_t *ctx, uint32_t actual_sample_rate_hz);

// 每采样点调用一次（在主循环中，批处理模式）
void WaveletProcessor_Update(WaveletProcessorCtx_t *ctx,
                              volatile uint16_t *ring_buf,
                              uint16_t ring_idx);

// 获取当前事件（由调用方读取）
WaveletProcessorEvent_t WaveletProcessor_GetCurrentEvent(WaveletProcessorCtx_t *ctx);

// 查询
uint8_t  WaveletProcessor_GetLearningProgress(WaveletProcessorCtx_t *ctx);
uint32_t WaveletProcessor_GetBaselineEnergy(WaveletProcessorCtx_t *ctx);
uint16_t WaveletProcessor_GetHFRatio(WaveletProcessorCtx_t *ctx);

// 事件标志
uint8_t WaveletProcessor_PopRainPending(WaveletProcessorCtx_t *ctx);
uint8_t WaveletProcessor_PopVibPending(WaveletProcessorCtx_t *ctx);

// 事件结构体
typedef struct {
    WE_EventType_t type;
    uint32_t norm_ratio;
    uint16_t hf_ratio;
    float impulse;
    uint32_t energy_delta;
    uint32_t log_energy;
    uint16_t peak_adc;
    uint32_t timestamp_ms;
} WaveletProcessorEvent_t;
```

### 9.2 RainfallMgr — 雨量管理器

```c
void RainfallMgr_Init(void);
void RainfallMgr_OnRainEvent(WaveletProcessorEvent_t *ev);
uint32_t RainfallMgr_GetTotalVolume_0p01mm3(void);
uint32_t RainfallMgr_GetRainfallIntensity_MMH(void);
void RainfallMgr_ResetTotal(void);
void RainfallMgr_SetVolumeModel(VolumeModelFunc func, const char *name);
```

### 9.3 RegisterMap — 统一寄存器映射

```c
void RegisterMap_Init(uint8_t slave_id);
uint16_t RegisterMap_ReadHolding(uint16_t addr, uint8_t *exception);
uint8_t  RegisterMap_WriteHolding(uint16_t addr, uint16_t value, uint8_t *exception);
uint16_t RegisterMap_ReadInput(uint16_t addr, uint8_t *exception);
```

> **所有 Modbus 和 SPI 的寄存器读写统一经由此模块**，删除 `Modbus_Slave.c` 和 `SPI_Proto.c` 中的重复实现。

---

## 10. 文件结构重构

```
User/
├── main.c                         # 主循环 + 调度器
├── app/
│   ├── RainfallMgr.c / .h         # 雨量管理器（单一体积出口）
│   ├── Calibration.c / .h         # 标定框架
│   └── SystemMgr.c / .h           # 系统管理
├── signal/
│   ├── WaveletProcessor.c / .h   # 重构小波处理器
│   └── PeakDetector.c / .h         # 峰值检测器（备选）
├── driver/
│   ├── ADCDriver.c / .h           # ADC + DMA 驱动
│   └── RingBuffer.c / .h          # 环形缓冲区
├── comms/
│   ├── ModbusRTU.c / .h           # Modbus 从站
│   ├── ModbusRTU.h                # 头文件
│   ├── TFProto.c / .h             # TF 主动上报（USART3）
│   ├── SPILink.c / .h             # SPI 从机链路
│   ├── SPIProto.c / .h            # SPI 协议编解码
│   ├── RegisterMap.c / .h         # 统一寄存器映射
│   └── DebugUART.c / .h           # 调试串口（USART1）
├── hal/
│   ├── OLED.c / .h                # OLED 显示
│   ├── LED.c / .h                 # LED 驱动
│   ├── Key.c / .h                 # 按键驱动
│   ├── IWDG.c / .h                # 看门狗
│   └── Delay.c / .h               # 延时
└── stm32f10x_it.c                 # 中断向量表
```

**删除的文件**：
- `Hardware/Wavelet.c` — 与 `WaveletEnergy.c` 功能重复
- `Hardware/WaveletEnergy.c` — 重构为 `WaveletProcessor.c`
- `Hardware/raindrop.c` — 死区逻辑整合到 `WaveletProcessor`，体积计算整合到 `RainfallMgr`
- `Hardware/AD.c` — 重构为 `ADCDriver.c`
- `Hardware/Modbus_Slave.c` — 重构为 `ModbusRTU.c`
- `Hardware/TF_Comm.c` — 重构为 `TFProto.c`（USART3）

---

## 11. 关键设计决策汇总

| # | 问题 | 决策 | 替代方案 | 决策理由 |
|---|------|------|----------|----------|
| D-01 | TF 与 Modbus 共享 USART | **分离**：Modbus 用 USART2，TF 用 USART3 | 软件复用同一 USART（复杂且不可靠） | 物理层协议不同（9600 vs 115200），无法简单复用 |
| D-02 | DWT 在 ISR 中执行 | **移至主循环**：DMA ISR 只做数据搬运 | 保持 ISR 中执行（实时性好但风险高） | ISR 应尽量轻量；主循环 10ms 延迟对雨量计可接受 |
| D-03 | 双体积换算路径 | **删除 `raindrop.c` 主路径**，仅保留辅助函数 | 保持双路径并行（不可靠且维护困难） | 两路径物理模型不同，无法简单合并 |
| D-04 | 双重死区 | **统一**：删除 `raindrop.c` 死区，保留小波层死区 | 保持双层死区 | 重复死区导致漏检 |
| D-05 | 采样频率硬编码 | **实测注入**：ADC 初始化时实测，注入算法 | 保持硬编码（维护简单但不准确） | 消除硬编码依赖，提高适应性 |
| D-06 | 能量溢出 | **对数压缩**：返回 `log2(energy)` | 保持 64 位累加 | 对数压缩后动态范围从 ~10¹¹ 降至 ~40，完美适配固定阈值 |
| D-07 | 调试串口 | **专用 USART1**，与业务通信分离 | 复用 USART1（业务与调试互相干扰） | 调试时拔掉调试线不影响 Modbus/TF 工作 |
| D-08 | 寄存器映射 | **单源**：RegisterMap 是 Modbus 和 SPI 的唯一寄存器源 | Modbus 和 SPI 各维护独立映射（代码冗余） | 单一数据源，消除不一致风险 |
| D-09 | 标定数据存储 | **RAM + Flash**：运行时存 RAM，定期写 Flash | 仅存 RAM（掉电丢失） | 标定数据来之不易，需持久化 |
| D-10 | 峰值检测 | **备选保留**：作为小波方法的辅助参考 | 删除（实现简单） | 峰值方法对噪声敏感，不适合作为主路径 |

---

## 12. 迁移路径（实施建议）

由于重构幅度较大，建议分三个阶段实施：

### 阶段一：隔离核心问题（高风险，优先修复）
1. 修复 `WaveletEnergy.c` 中的能量溢出（改用 `uint64_t` 累加）
2. 修复 `raindrop.c` 的 `UpdateRefractory()` 未被调用的问题
3. 修复 USART2 波特率冲突（将 TF_Comm 迁移到 USART3）
4. 修复基线慢速跟踪条件逻辑

**目标**：修复后系统能稳定工作

### 阶段二：架构清理（中风险，提升可维护性）
5. 统一寄存器映射源（删除 `Modbus_Slave.c` 重复代码）
6. 删除 `Wavelet.c`（与 `WaveletEnergy.c` 合并）
7. 统一死区机制
8. 将 DWT 计算从 DMA ISR 移至主循环

**目标**：代码结构清晰，可维护

### 阶段三：功能完善（低风险，长期改进）
9. 实现采样频率实测机制
10. 实现 Flash 存储（标定数据持久化）
11. 实现可替换的体积模型插件架构
12. 对数能量压缩（可选）

**目标**：完整功能的产品级实现

---

*本架构设计文档描述目标系统结构，作为固件重构的实施蓝图。随着阶段一至阶段三的推进，文档将持续更新。*
