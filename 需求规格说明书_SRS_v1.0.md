# 压电式雨量传感器从控系统

## 软件需求规格说明书（SRS）

| 属性 | 内容 |
|------|------|
| 版本 | v1.0 |
| 编制日期 | 2026-04-24 |
| 项目名称 | 压电式雨量传感器从控系统 |
| 目标平台 | STM32F103C8T6（ARM Cortex-M3） |
| 主频 | 72 MHz |
| 存储 | 64 KB Flash，20 KB SRAM |

---

## 1. 简介

### 1.1 目的

本系统为一款基于 STM32F103C8T6 微控制器的**压电式雨量传感器从控单元**（以下简称"从控"）。从控负责完成降雨信号的采集、处理、计量，并通过工业标准 Modbus RTU 总线或 TF 自定义协议向上位主控（主机）报告雨量数据。本文档定义从控软件的完整功能需求、非功能需求及接口规范。

### 1.2 范围

本文档适用于从控固件的**设计、实现、测试和验收**。涵盖：

- 硬件抽象层（ADC采样、DMA环形缓冲）
- 信号处理核心算法（Haar小波能量积分）
- 雨量计量与标定框架
- 通信协议（Modbus RTU 从站、TF 自定义协议）
- 人机界面（OLED 本地显示、串口调试输出）
- 系统管理（看门狗、时间基准）

### 1.3 定义与缩略语

| 术语 | 定义 |
|------|------|
| **压电雨量传感器** | 利用压电材料将雨滴撞击能量转换为电压信号的传感器 |
| **Haar 小波能量积分** | 采用 Haar 离散小波变换对 64 点信号窗口进行三层分解，以高频-detail 系数能量区分雨滴与振动 |
| **hf_ratio** | 高频能量比例（permille），高频能量与总能量之比，用于区分事件类型 |
| **norm_ratio** | 归一化能量比值（permille），当前帧能量相对于基线能量的增量 |
| **死区时间** | 一次事件触发后，系统拒绝处理新事件的最小时间间隔，防止同一次雨滴被重复计量 |
| **学习期** | 开机后 30 秒的自适应阶段，用于估算环境直流偏置和基线能量 |
| **Modbus 从站** | 响应主站轮询的 Modbus RTU 从机设备，寄存器地址空间暴露给主站读取 |
| **TF 协议** | 从控向主控主动上报事件的自定义二进制协议，帧格式为 AA 55 ... 0D |
| **VOFA+** | 一款串口上位机软件，支持 JustFloat 协议，用于实时波形观测 |

---

## 2. 总体描述

### 2.1 系统架构

```
┌─────────────────────────────────────────────────────┐
│                    主循环 (10ms)                     │
│  ┌──────────────┐  ┌─────────────┐  ┌───────────┐  │
│  │ Modbus 从站   │  │ TF 协议上报  │  │ OLED 显示  │  │
│  │ 进程         │  │ 进程         │  │ 进程 (200ms)│ │
│  └──────────────┘  └─────────────┘  └───────────┘  │
│         ▲                 ▲              ▲          │
│  ┌──────────────────────────────────────────────┐  │
│  │           小波能量积分模块 (WaveletEnergy)    │  │
│  │  64 点窗口 → Haar DWT → 高频能量 → 事件分类   │  │
│  └──────────────────────────────────────────────┘  │
│         ▲                 ▲                         │
│  ┌──────────────────────────────────────────────┐  │
│  │            环形缓冲区 (500 点, 双通道)         │  │
│  └──────────────────────────────────────────────┘  │
└─────────▲─────────────────────────▲────────────────┘
          │ DMA 半满/完成中断     │
┌─────────┴─────────────────────────▼────────────────┐
│              ADC1 (双通道扫描, DMA 传输)           │
│  通道0 (PA0) → 压电雨量传感器                      │
│  通道1 (PA1) → 保留                               │
└────────────────────────────────────────────────────┘
```

### 2.2 硬件资源分配

| 硬件资源 | 用途 | 说明 |
|----------|------|------|
| ADC1 | 模拟信号采集 | 双通道扫描，DMA 循环模式，采样率 ~20 kSPS |
| DMA1 Channel1 | ADC 数据传输 | 半满/完成中断，填充环形缓冲区 |
| USART1 | VOFA+ 调试串口 | 115200 8N1，发送实时波形和调试统计 |
| USART2 | Modbus RTU + TF 上报（复用） | 9600 8N1，Modbus 从站；115200，TF 上报 |
| SPI1 | SPI 从机接口 | PA4(NSS)/PA5(SCK)/PA6(MISO)/PA7(MOSI)，接受主控 V6 的寄存器读写 |
| TIM2 | 1 ms 系统 tick | 为 Modbus 帧间隔计时 |
| IWDG | 独立看门狗 | 3 秒超时，防止程序异常死锁 |
| OLED | 本地显示 | 128×64 OLED，4 行显示 |
| GPIO | 按键、LED | PE4(Key)，PB5(Led) |

### 2.3 软件架构

```
User/
├── main.c                    # 主循环、标定框架、USART1 调试
├── raindrop.c / .h           # 电压→体积 LUT 换算（独立路径）
├── stm32f10x_it.c            # 中断向量表

Hardware/
├── AD.c / .h                 # ADC + DMA + 环形缓冲区
├── WaveletEnergy.c / .h       # 小波能量积分核心算法
├── Modbus_Slave.c / .h       # Modbus RTU 从站
├── TF_Comm.c / .h            # TF 自定义事件上报协议
├── SPI_SlaveLink.c / .h      # SPI 从机链路层
├── SPI_Proto.c / .h          # SPI 协议编解码
├── RegisterMap.c / .h        # 寄存器映射表
├── RegisterMap.c / .h        # 寄存器映射表
├── LED.c / .h                 # LED 驱动
├── Key.c / .h                 # 按键驱动
├── OLED.c / .h               # OLED 显示驱动
├── OLED_Font.h               # 字体点阵
└── TF_Comm.h / .c            # TF 协议

System/
├── Delay.c / .h              # 软件延时
└── Delay.c / .h               # 软件延时

Library/
└── STM32F10x_StdPeriph_Driver # 标准外设库
```

---

## 3. 功能需求

### 3.1 信号采集与预处理

#### FR-3.1.1 ADC 采样

- 系统采用 **ADC1 双通道扫描模式**，规则组配置 PA0（通道0）和 PA1（通道1）
- DMA1 Channel1 循环传输，每次半满（100点）或完成（200点）触发中断
- 采样频率取决于 ADC 时钟配置（APB2=72 MHz，6分频=12 MHz，采样时间 239.5 周期 ≈ 50 μs/点）
- 双通道交替存储，实际单通道采样率 ≈ 20 kSPS
- ADC 启动后持续运行，不间断地向环形缓冲区写入最新 500 点数据

#### FR-3.1.2 环形缓冲区

- 通道0 环形缓冲区：`adc_ring_buffer_ch0[RING_BUFFER_SIZE]`，`RING_BUFFER_SIZE = 500`
- 写索引 `ring_write_index_ch0` 由 DMA 半满/完成中断更新
- 主循环通过 `ring_write_index_ch0` 读取最新 64 点窗口数据

#### FR-3.1.3 直流偏置估算

- **学习期**：取最近 64 点样本的均值作为直流偏置（ADC 原始值），存入 `dc_baseline`
- **检测期**：`dc_baseline` 固定为学习期估算值，不再更新
- 信号去直流：每点 `diff = sample - dc_baseline`，截断至 [-32768, 32767] 存入 int16 窗口

### 3.2 小波能量积分算法

#### FR-3.2.1 Haar 离散小波变换

- 窗口长度：64 点（必须为 2 的幂）
- 分解层数：3 层（64→32→16→8）
- 每层分解使用 Haar 变换：
  - 近似系数：`approx[i] = (input[2i] + input[2i+1]) / 2`
  - Detail 系数：`detail[i] = (input[2i] - input[2i+1]) / 2`
- **能量计算**：高频能量 = d1(32点) + d2(16点) + d3(8点) 的平方和
- **高频比例**：`hf_ratio = high_freq_energy / total_energy × 1000`（permille）

#### FR-3.2.2 自适应学习

- 学习期时长：30 秒
- 学习期内：
  - 实时估算 `dc_baseline`
  - 累加所有帧的 `energy_high`，计算均值作为 `baseline_energy`
  - 用经验公式估算标准差：`baseline_std = baseline_energy / 10`
  - 检测阈值：`threshold = baseline_energy + 3 × baseline_std`
  - 阈值不低于：`baseline_energy + WE_MIN_ENERGY_DELTA`（50）
- 学习完成后：
  - `learning_done = 1`
  - 事件分类逻辑激活

#### FR-3.2.3 基线慢速跟踪

- 检测期每 1000 帧且能量增量低于 `WE_MIN_ENERGY_DELTA` 时，对基线进行 0.1% 的缓慢调整
- 防止长期运行中环境变化导致的基线漂移

#### FR-3.2.4 事件分类

根据高频比例 `hf_ratio` 和能量增量将每帧信号分为以下类型：

| 事件类型 | 条件 | 处理 |
|----------|------|------|
| NOISE（噪声） | `energy_delta < WE_MIN_ENERGY_DELTA` | 忽略，不计量 |
| SUSPECT（可疑） | `energy_delta >= WE_MIN` 但 `energy_high < threshold` | 忽略，计入噪声计数 |
| VIB（振动） | `energy_high >= threshold` 且 `hf_ratio >= 450 permille` | 忽略，计入振动计数 |
| RAIN（雨滴） | `energy_high >= threshold` 且 `hf_ratio <= 400 permille` | 触发雨量计量 |
| 中间区域 | `400 < hf_ratio < 450` | 保守处理为振动 |

#### FR-3.2.5 事件分类参数

| 参数 | 符号 | 值 | 说明 |
|------|------|-----|------|
| 最小有效能量增量 | `WE_MIN_ENERGY_DELTA` | 50 | 低于此值判定为噪声 |
| 标准差倍数 | `WE_STD_FACTOR` | 3 | 对应 99.7% 置信区间 |
| 雨滴高频上限 | `WE_HF_RAIN_MAX` | 400 permille | 低频为主→雨滴 |
| 振动高频下限 | `WE_HF_VIB_MIN` | 450 permille | 高频为主→振动 |
| 积分死区（检测期） | `WE_INTEGRAL_DEADTIME` | 200 次采样 | ≈ 8.4 ms |
| 积分死区（标定期） | `WE_CAL_DEADTIME` | 50 次采样 | ≈ 2.1 ms |

#### FR-3.2.6 体积计算

雨滴事件触发后，计算雨滴体积：

```
norm_ratio = (energy_high - baseline_energy) / baseline_energy × 1000
volume = norm_ratio × 5 / 10   (单位：0.01 mm³)
最大单次体积上限：5000 (即 50 mm³)
```

注：体积换算系数（`VOLUME_SCALE`）待现场标定后确定，当前默认 0（不进行体积累计，仅显示 norm_ratio）。

### 3.3 雨量计量

#### FR-3.3.1 雨滴计数

- 雨滴事件触发时，`rain_count` 递增
- 每秒环形窗口 `drops_per_second[60]` 记录各秒内雨滴数
- `current_intensity_mmh` = 最近 60 秒雨滴总数 × 0.1（mm/h，等效估算）

#### FR-3.3.2 累计体积

- 全局变量 `g_total_volume_0p01mm3` 记录自启动以来的累计雨液体积
- `VOLUME_SCALE = 0` 时：体积累计关闭，累计值不更新
- `VOLUME_SCALE > 0` 时：每次雨滴事件累加 `WaveletEnergy_ComputeVolume` 的返回值

### 3.4 标定框架

#### FR-3.4.1 标定模式开关

- 宏 `WE_CALIBRATION_MODE` 控制：0=关闭，1=开启
- 标定模式：死区时间缩短（2.1 ms），便于分辨连续雨滴
- 生产模式：死区时间恢复为 8.4 ms

#### FR-3.4.2 数据记录

- 每检测到一个雨滴事件，记录以下数据：
  - `nr`：归一化比值（×1000 permille）
  - `hf`：高频比例（permille）
  - `impulse`：冲量（ADC·次）
  - `dE`：能量增量
  - `time_ms`：系统时间戳（毫秒）
  - `group_idx`：分组编号
  - `seq_in_group`：组内序号
- 最大记录容量：20 组 × 100 滴/组 = 2000 条
- 记录满后自动丢弃，不再覆盖

#### FR-3.4.3 串口标定命令

| 命令 | 功能 | 说明 |
|------|------|------|
| `SETN <name>` | 新建分组 | 设置针头名称，重置组内计数 |
| `STAT` | 查询统计 | 输出当前分组信息和滴数 |
| `EXPORT` | 导出数据 | 通过 USART1 串口导出所有记录 |
| `RESET` | 重置记录 | 清空所有分组和记录 |
| `HELP` | 命令帮助 | 输出命令列表 |

#### FR-3.4.4 数据导出格式

```
[CAL] === EXPORT START ===
# group    seq    nr      hf     impulse      dE       time_ms
1       default  1   1203    385    125.30     50432    15234
1       default  2   1156    372    118.45     48231    15678
...
[CAL] === EXPORT END (2000 records) ===
```

### 3.5 通信协议

#### FR-3.5.1 Modbus RTU 从站

- 物理层：USART2，9600 8N1
- 功能码支持：
  - `0x03`：读保持寄存器
  - `0x04`：读输入寄存器
  - `0x06`：写单个保持寄存器
- 从站地址：通过 `MODBUS_SLAVE_ID` 宏配置（默认 1）
- 帧间隔检测：3.5 字符时间（基于 TIM2 1ms tick）
- 寄存器映射表：

**输入寄存器（0x04 读）：**

| 地址 | 寄存器名 | 类型 | 说明 |
|------|----------|------|------|
| 0x0000 | LearningProgress | RO | 学习进度 0-100（%） |
| 0x0001-02 | CurrentEnergy | RO | 当前高频能量（uint32，高低16位） |
| 0x0003-04 | BaselineEnergy | RO | 基线能量（uint32） |
| 0x0005-06 | Threshold | RO | 检测阈值（uint32） |
| 0x0007-08 | NormRatio | RO | 归一化比值（uint32） |
| 0x0009 | HFRatio | RO | 高频比例（uint16，permille） |
| 0x000A-0B | RainCount | RO | 雨滴计数（uint32） |
| 0x000C-0D | VibCount | RO | 振动计数（uint32） |
| 0x000E-0F | NoiseCount | RO | 噪声计数（uint32） |
| 0x0010-11 | TotalEvents | RO | 总事件数（uint32） |
| 0x0020-21 | TotalVolume | RO | 累计体积（uint32，0.01mm³） |
| 0x0022-23 | SamplingTick | RO | 采样 tick 计数（uint32） |
| 0x0024-25 | WatchdogCount | RO | 看门狗触发计数（uint32） |

**保持寄存器（0x03 读，0x06 写）：**

| 地址 | 寄存器名 | 类型 | 说明 |
|------|----------|------|------|
| 0x0000 | SlaveAddress | RW | 从站地址（1-247） |

#### FR-3.5.2 TF 自定义事件上报协议

- 物理层：USART2，115200 8N1
- 帧格式（16 字节定长）：

```
Offset  Size  Field       Description
  0      1    SOF1        0xAA（帧起始）
  1      1    SOF2        0x55（帧起始）
  2      1    LEN         0x0D（数据长度，不含SOF/EOF/CRC）
  3      1    SRC_ID      从控 ID（0x01）
  4      1    FLAG        bit0=触发标志, bit1=通道, bit2=小波完成, bit3=队列溢出
  5      2    PEAK        峰值 ADC 值（大端序）
  7      4    ENERGY      信号能量（大端序）
 11      2    SEQ         帧序号（大端序，自增）
 13      2    CRC16       CRC16-CCITT-FALSE（从 LEN 到 SEQ，共 11 字节）
 15      1    EOF         0x0D（帧结束）
```

- 发送队列：深度 10 帧
- 队满策略：丢弃最旧帧，`FLAG.bit3 = 1` 标记本次入队发生了丢帧
- 触发条件：雨滴事件 (`WE_EVENT_RAIN`) 且 `learning_done = 1`

#### FR-3.5.3 SPI 从机接口

- 物理层：SPI1，Mode 0，MSB First
- 片选：PA4 (NSS)，下降沿开始事务，上升沿结束
- 协议：Modbus-like 帧结构（地址+功能码+数据+CRC16）
- 功能码：
  - `0x03`：读保持寄存器
  - `0x04`：读输入寄存器
  - `0x10`：写多个寄存器
- 寄存器映射与 Modbus RTU 共享同一份 `RegisterMap`

### 3.6 人机界面

#### FR-3.6.1 OLED 本地显示（生产模式）

4 行 × 16 字符布局：

```
行1: E:xxxxx [状态]    当前高频能量 + 状态标识
行2: D:xxxx B:xxxx    直流偏置 + 基线能量
行3: Cnt:xxx          雨滴累计计数
行4: Hf:xxx T:xxxxx   高频比例 + 检测阈值
```

状态标识：
- 学习期：`L xx%`（进度百分比）
- 检测期：`Rok`（雨滴）、`Vok`（振动）、`Nok`（噪声）、`-ok`（无事件）

#### FR-3.6.2 OLED 本地显示（标定模式）

```
行1: [组名] xx        当前分组名 + 组内滴数
行2: nr:xxxx hf:xxx   最近一次 norm_ratio 和 hf_ratio
行3: cnt:RRRR         总雨滴检测数
行4: tot:RRRR         总记录滴数
```

#### FR-3.6.3 USART1 调试输出

- 波特率：115200 8N1
- **实时示波输出（VOFA+ JustFloat 协议）**：
  - 每主循环一次，发送最新 10 点 ADC 原始值（float，4 字节/点 + 4 字节尾部）
  - 格式：`byte0=ADC[0]/4095×3.3V, byte1=..., byte2=..., byte3=0x7F`

- **调试统计（每秒一次）**：
  ```
  === WE Debug ===
  learn=xx% done=Y/N
  dc_baseline=xxxx (ADC raw, should be stable during learning)
  baseline_energy=xxxx
  threshold=xxxxx
  cur_energy=xxxxx
  hf_ratio=xxx permille (rain<400, vib>450)
  norm_ratio=xxx permille
  rain_cnt=xxx vib=xxx noise=xxx total=xxx
  sampling_tick=xxxxxxx
  ==============
  ```

- **标定数据实时输出（每滴一次）**：
  ```
  [default:  1] nr=1203 hf=385 imp=12.53 dE=50432
  ```

---

## 4. 非功能需求

### 4.1 性能

| 指标 | 要求 | 说明 |
|------|------|------|
| ADC 采样率 | ≥ 15 kSPS/通道 | 确保 64 点窗口覆盖足够的信号持续时间 |
| 事件检测延迟 | < 20 ms | 从信号峰值到事件分类完成 |
| Modbus 响应时间 | < 10 ms | 从收到完整帧到发出应答 |
| 死区时间（检测期） | 8.4 ms（固定） | 防止同一次雨滴重复计量 |
| 死区时间（标定期） | 2.1 ms（固定） | 便于分辨连滴 |
| 系统主循环周期 | 10 ms（固定） | 固定节拍 |

### 4.2 可靠性

| 指标 | 要求 | 说明 |
|------|------|------|
| 看门狗超时 | 3 秒 | 独立看门狗，防止程序死锁 |
| 通信异常恢复 | 自动重连 | DMA 传输异常时自动重启 ADC |
| 环形缓冲区溢出 | 覆盖写入 | 始终保留最新 500 点数据 |
| 内存使用 | < 15 KB SRAM | 含标定记录、环形缓冲、上下文结构 |

### 4.3 可配置性

以下参数应可通过配置文件（编译期宏）或远程寄存器修改：

| 参数 | 编译期宏 | Modbus 寄存器 | 默认值 |
|------|----------|---------------|--------|
| 从站地址 | `MODBUS_SLAVE_ID` | 0x0000 | 1 |
| 标定模式开关 | `WE_CALIBRATION_MODE` | — | 0 |
| 体积换算系数 | `VOLUME_SCALE` | — | 0 |
| ADC 阈值 | `ADC_Threshold` | — | 620 |
| 学习期时长 | `WE_LEARNING_PERIOD_SEC` | — | 30 |
| 雨滴高频上限 | `WE_HF_RAIN_MAX` | — | 400 |
| 振动高频下限 | `WE_HF_VIB_MIN` | — | 450 |

### 4.4 兼容性

- 固件应兼容 **Keil MDK-ARM** 5.x 编译环境
- 固件应兼容 **STM32F103C8T6** 最小系统板
- Modbus RTU 协议应符合 **Modbus IDA 组织**发布的《Modbus Application Protocol Specification V1.1b3》

---

## 5. 数据流图

```
ADC 采样
  │
  ▼
DMA 中断 → 写入环形缓冲区 (adc_ring_buffer_ch0[500])
  │
  │ (主循环每 10ms 读取一次)
  ▼
WaveletEnergy_Update()
  │
  ├── Compute_DC_Baseline()  ──► 学习期估算直流偏置
  │
  ├── Fill_Window_Centered() ──► 取 64 点去直流信号
  │
  ├── Haar_DWT_Step()×3     ──► 三层 Haar 分解
  │
  ├── Compute_Energy_SumSquares() ──► 高频能量 d1+d2+d3
  │
  ├── 事件分类 (hf_ratio + energy_delta)
  │     │
  │     ├── NOISE → 忽略
  │     ├── SUSPECT → 忽略
  │     ├── VIB → 振动计数 + vib_event_pending
  │     └── RAIN → 雨量计量 + rain_event_pending
  │
  └── (死区计数器 = WE_INTEGRAL_DEADTIME)
        │
        ▼
主循环检测 rain_event_pending
  │
  ├── WaveletEnergy_ComputeVolume() → 体积计算
  ├── g_total_volume_0p01mm3 += volume
  ├── drops_per_second[sec_index]++
  ├── CAL_RecordDrop() → 标定记录
  ├── TF_Comm_SendEvent() → 主动上报
  └── OLED 刷新 + Modbus 寄存器更新
```

---

## 6. 接口信号定义

### 6.1 电气接口

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | ADC_CH0 | 压电雨量传感器信号输入 |
| PA1 | ADC_CH1 | 保留（通道1） |
| PA2 | USART2_TX | RS-485 TX+ / TF 上报 |
| PA3 | USART2_RX | RS-485 RX+ / Modbus RTU |
| PA4 | SPI1_NSS | SPI 片选输入 |
| PA5 | SPI1_SCK | SPI 时钟输入 |
| PA6 | SPI1_MISO | SPI 数据输出 |
| PA7 | SPI1_MOSI | SPI 数据输入 |
| PA9 | USART1_TX | VOFA+ 调试串口（TX） |
| PA10 | USART1_RX | VOFA+ 调试串口（RX） |
| PB5 | GPIO_LED | 工作指示灯 |
| PE4 | GPIO_KEY | 标定按键 |

### 6.2 压电传感器接口规格

| 参数 | 规格 |
|------|------|
| 输出电压范围 | 0 ~ 3.3 V（受 ADC 参考电压限制） |
| 信号带宽 | 0.1 Hz ~ 5 kHz |
| 建议采样率 | ≥ 10 kSPS |
| 等效内阻 | 待传感器手册确认 |

---

## 7. 已知限制与待办

| 序号 | 项目 | 优先级 | 说明 |
|------|------|--------|------|
| KN-1 | 体积换算系数待标定 | 高 | 当前 `VOLUME_SCALE = 0`，不进行体积累计 |
| KN-2 | 高频比例阈值待验证 | 高 | 400/450 permille 的边界值需现场实验数据支撑 |
| KN-3 | 电压→体积 LUT 需现场标定 | 高 | `raindrop.c` 中的 14 点 LUT 数据来源为实验室标定，需现场验证 |
| KN-4 | 双通道 PA1 未使用 | 中 | DMA 持续传输通道1数据但从不处理，浪费 50% 带宽 |
| KN-5 | USART2 波特率混用 | 高 | TF 协议假设 115200，Modbus 配置 9600，需统一或分离 |
| KN-6 | 无 Flash 存储 | 中 | 标定数据断电丢失，需增加 Flash 持久化 |
| KN-7 | 无固件升级机制 | 低 | 当前不支持 OTA 升级 |
| KN-8 | ADC 校准因子未验证 | 低 | 校准完成后未读取校准因子验证合理性 |

---

## 8. 参考标准

| 标准 | 名称 | 说明 |
|------|------|------|
| Modbus IDA | Modbus Application Protocol Specification V1.1b3 | Modbus RTU 协议规范 |
| Modbus IDA | Modbus over Serial Line Specification V1.02 | 串行链路层规范 |
| IEEE 1057 | Standard for Digitizing Waveform Recorders | 波形数字化标准 |
| ISO 5/1984 | 降雨强度分级标准 | 降雨量等级参考 |

---

*本文件为软件需求规格说明书（SRS），随着系统设计和实验数据的积累，部分参数阈值（如高频比例边界、雨滴体积模型）将根据现场标定数据持续更新。*
