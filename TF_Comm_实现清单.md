# TF通信模块实现清单（USART2方案）

## 一、硬件配置

### 1.1 外设与端口
- **外设**：USART2（固定映射：PA2=TX, PA3=RX）
- **端口配置**：
  - PA2 (TX)：复用推挽输出，50MHz
  - PA3 (RX)：上拉输入
- **端口冲突处理**：
  - PA2被LED占用，需迁移LED功能
  - **LED迁移优先级**：
    1. **优先**：PA4或PA5（通用GPIO，无特殊限制）
    2. **备选**：PC13（注意：可能反相逻辑/驱动能力弱/速度慢，需在代码中注明）
- **物理层连线**：3线UART（TX、RX、GND）
  - RX暂不使用也要预留焊盘/排针（未来ACK/参数下发/时间同步）

### 1.2 通信参数
- **波特率**：115200 bps（验证阶段），阵列后可升级至460800/921600 bps
- **数据位**：8
- **停止位**：1
- **校验位**：无
- **流控**：无

### 1.3 中断优先级
- **ADC/DMA/AWD中断**：高优先级（确保采样实时性）
- **USART2 TXE中断**：低优先级（避免影响采样）

---

## 二、通信协议规范

### 2.1 帧结构（固定16字节）

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ 帧头1 │ 帧头2│ 长度 │ 源ID │ 标志 │ 峰值 │ 能量 │ 序号 │ 校验 │ 帧尾  │
│ 0xAA  │ 0x55 │ 1B   │ 1B   │ 1B   │ 2B   │ 4B   │ 2B   │ 2B   │ 0x0D  │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
总线传输总长度：16字节（2字节SOF + 1字节LEN + 13字节payload）
payload长度：13字节（从SRC_ID到EOF：SRC_ID+FLAG+PEAK+ENERGY+SEQ+CRC+EOF）
```

### 2.2 字段定义

| 字段 | 长度 | 值 | 说明 |
|------|------|-----|------|
| SOF | 2字节 | 0xAA 0x55 | 帧同步标识（固定值，唯一） |
| LEN | 1字节 | 13 | 从SRC_ID字段到EOF的字节数（payload长度，不含LEN本身） |
| SRC_ID | 1字节 | 0 | 从控编号（单从控固定0，阵列扩展时区分通道） |
| FLAG | 1字节 | - | 标志位（见下表） |
| PEAK | 2字节 | - | 峰值ADC值（uint16_t，大端序/网络序） |
| ENERGY | 4字节 | - | 信号能量（uint32_t，大端序/网络序，近似值：peak × sample_count，单位ADC-count） |
| SEQ | 2字节 | - | 事件序号（uint16_t，自增，大端序/网络序） |
| CRC | 2字节 | - | CRC16-CCITT校验（大端序/网络序） |
| EOF | 1字节 | 0x0D | 帧结束标识（固定值，唯一） |

**LEN字段说明**：
- LEN = 13（固定值）
- 含义：从SRC_ID字段开始到EOF的字节数（payload长度）
- 计算：SRC_ID(1) + FLAG(1) + PEAK(2) + ENERGY(4) + SEQ(2) + CRC(2) + EOF(1) = 13字节
- 总帧长：SOF(2) + LEN(1) + payload(13) = 16字节
- 主控解析：SOF对齐后，读取LEN=13，再接收13字节payload

### 2.3 FLAG字段位定义

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | trigger_flag | 雨滴触发标志（1=有效雨滴） |
| 1 | source_channel | 通道来源（0=PA0正常，1=PA1饱和切换） |
| 2 | wavelet_done | 小波计算完成标志（1=已完成小波计算） |
| 3 | queue_overflow | 队列溢出标志（预留，1=当前段数据不完整） |
| 4-7 | reserved | 保留 |

### 2.4 CRC16校验规范

- **标准**：CRC-16/CCITT-FALSE
- **参数**：
  - Poly = 0x1021
  - Init = 0xFFFF
  - RefIn = false
  - RefOut = false
  - XorOut = 0x0000
- **覆盖范围**：LEN、SRC_ID、FLAG、PEAK、ENERGY、SEQ（不含SOF，不含CRC自身，不含EOF）
  - 共11字节：LEN(1) + SRC_ID(1) + FLAG(1) + PEAK(2) + ENERGY(4) + SEQ(2) = 11字节
- **计算位置**：主循环入队时完成，中断中只搬运字节
- **端序说明**：所有多字节字段使用大端序（网络序），便于跨平台解析

---

## 三、文件组织

### 3.1 新建文件
- `Hardware/TF_Comm.h` - 头文件
- `Hardware/TF_Comm.c` - 实现文件

### 3.2 头文件内容（TF_Comm.h）

```c
#ifndef __TF_COMM_H
#define __TF_COMM_H

#include "stm32f10x.h"

// 队列配置
#define TF_COMM_QUEUE_SIZE    32      // 队列容量（帧数）
#define TF_COMM_FRAME_SIZE    16      // 单帧字节数

// 协议常量
#define TF_COMM_SOF_BYTE1     0xAA
#define TF_COMM_SOF_BYTE2     0x55
#define TF_COMM_EOF           0x0D
#define TF_COMM_LEN           13      // payload长度（从SRC_ID到EOF）

// 从控编号（编译时配置，阵列扩展时按拨码/烧录配置修改）
#define TF_SRC_ID             0       // 单从控固定0，阵列时改为1..8

// 函数声明
void TF_Comm_Init(void);
uint8_t TF_Comm_SendEvent(uint8_t trigger_flag, uint16_t peak_value, 
                         uint32_t energy, uint8_t source_channel, 
                         uint8_t wavelet_done);  // 去掉src_id参数，统一使用TF_SRC_ID宏
uint32_t TF_Comm_GetDropCount(void);
void TF_Comm_TX_IRQHandler(void);  // USART2发送中断处理（在stm32f10x_it.c中调用）

#endif
```

---

## 四、核心函数实现

### 4.1 TF_Comm_Init() - 初始化函数

**功能**：
- 配置USART2时钟（RCC_APB1PeriphClockCmd）
- 配置GPIO（PA2=TX复用推挽，PA3=RX上拉输入）
- 配置USART2参数（115200, 8N1）
- 使能USART2
- 初始化发送队列
- 配置NVIC中断（USART2 TXE中断，低优先级）

**实现要点**：
```c
void TF_Comm_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    // 2. 配置GPIO
    // PA2 = TX (复用推挽)
    // PA3 = RX (上拉输入)
    
    // 3. 配置USART2
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart);
    USART_Cmd(USART2, ENABLE);
    
    // 4. 初始化队列
    // 5. 配置中断（低优先级）
}
```

### 4.2 TF_Comm_SendEvent() - 发送事件函数（非阻塞，队列入队）

**函数原型**：
```c
uint8_t TF_Comm_SendEvent(uint8_t trigger_flag, uint16_t peak_value, 
                         uint32_t energy, uint8_t source_channel, 
                         uint8_t wavelet_done)
```

**参数说明**：
- `trigger_flag`：雨滴触发标志（1=有效雨滴）
- `peak_value`：峰值ADC值（uint16_t）
- `energy`：信号能量（uint32_t，峰值×脉宽）
- `source_channel`：通道来源（0=PA0，1=PA1）
- `wavelet_done`：小波计算完成标志（0/1）
- **注意**：`src_id`参数已删除，SRC_ID字段统一使用编译时宏`TF_SRC_ID`

**返回值**：
- 1：入队成功
- 0：队列满（丢弃最旧帧后入队）

**实现流程**：
1. 构造数据帧（16字节）
   - SOF: 0xAA 0x55
   - LEN: 13（payload长度，从SRC_ID到EOF）
   - SRC_ID: TF_SRC_ID（编译时宏定义，当前固定0）
   - FLAG: (trigger_flag | (source_channel<<1) | (wavelet_done<<2) | (queue_overflow<<3))
     - **queue_overflow**：仅在本次入队触发drop_oldest时置1，否则为0
   - PEAK: peak_value（大端序，使用put_u16_be函数）
   - ENERGY: energy（大端序，使用put_u32_be函数）
     - **注意**：近似值 = peak × pulse_samples（单位ADC-count），占位指标，非物理能量
     - **pulse_samples**：用于ENERGY的样本数口径（uint16_t pulse_samples = end_index - start_index + 1）
   - SEQ: 序号（自增，大端序，使用put_u16_be函数）
   - CRC: CRC16-CCITT（覆盖LEN到SEQ，共11字节，大端序，使用put_u16_be函数）
   - EOF: 0x0D
2. 计算CRC16（在主循环中完成，覆盖LEN到SEQ共11字节）
3. 入队（非阻塞，精确临界区保护，收敛点火判定）
   - **临界区开始**：关闭TXE中断（防止ISR并发消耗队列）
   - 在临界区内完成：
     * 记录 `was_empty = (tx_queue.count == 0)`
     * `queue_overflow = 0`
     * 若队列满：drop_oldest，`drop_count++`，`queue_overflow = 1`
     * 写入封包（FLAG.bit3使用本次的queue_overflow）
     * 更新 `head`、`count`
   - **临界区结束**：退出临界区（释放对队列共享状态的保护）
   - **点火判定（只允许出现一次，且必须在临界区外）**：
     * **仅当 `was_empty==1` 时，开启TXE中断启动发送**
     * 若 `was_empty==0`（发送本来就在进行或队列本来就非空）：**不做任何TXE开关动作**

**端序处理函数**（建议实现）：
```c
// 大端序打包函数（避免手写移位散落）
static void put_u16_be(uint8_t *buf, uint16_t value)
{
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

static void put_u32_be(uint8_t *buf, uint32_t value)
{
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}
```

### 4.3 TF_Comm_TX_IRQHandler() - USART2发送中断处理函数（单中断模型）

**中断模型**：仅使用TXE中断，不依赖TC中断

**工作流程**：
1. TXE中断触发（发送数据寄存器空）
2. 检查队列是否有待发字节
3. 若有：从队列取1字节写入USART2->DR，继续等待TXE
4. 若无：关闭TXE中断，退出（避免空转中断）

**关键代码**（单一真相：只用tail，不维护tx_frame_idx）：
```c
void TF_Comm_TX_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        if (tx_queue.count > 0)
        {
            // 从tail指向的帧的当前字节位置发送（单一真相）
            uint8_t byte = tx_queue.buffer[tx_queue.tail][tx_queue.tx_byte_idx];
            USART2->DR = byte;  // 可选优化：直接寄存器访问，减少库函数开销
            // 注意：F1标准库中，写DR会自动清除TXE标志，无需手动Clear
            
            // 更新字节索引
            tx_queue.tx_byte_idx++;
            
            // 当前帧发送完成，切换到下一帧
            if (tx_queue.tx_byte_idx >= TF_COMM_FRAME_SIZE)
            {
                tx_queue.tx_byte_idx = 0;
                tx_queue.tail = (tx_queue.tail + 1) % TF_COMM_QUEUE_SIZE;  // 只更新tail
                tx_queue.count--;
            }
        }
        else
        {
            // 队列空，关闭TXE中断
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            // 重置发送索引
            tx_queue.tx_byte_idx = 0;
        }
        // 注意：不要手动Clear TXE标志，F1标准库会自动清除
    }
}
```

**优化说明**：
- **关键路径优化（可选）**：使用`USART2->DR = byte;`直接寄存器访问，减少库函数开销
- 但保持`USART_SendData(USART2, byte);`库函数写法也可，便于维护

### 4.4 队列实现

**数据结构**（单一真相：只用tail，不维护tx_frame_idx）：
```c
typedef struct {
    uint8_t buffer[TF_COMM_QUEUE_SIZE][TF_COMM_FRAME_SIZE];
    uint16_t head;          // 写指针（帧索引）
    uint16_t tail;          // 读指针（帧索引，唯一真相）
    uint16_t count;         // 当前帧数
    uint32_t drop_count;    // 丢包计数
    uint8_t tx_byte_idx;    // 当前帧发送到第几个字节（0..15）
} TF_Comm_Queue_t;

static TF_Comm_Queue_t tx_queue;
```

**并发安全**（收敛点火判定｜最终口径）：
- 入队/出队对共享指针的更新使用最小临界区保护
- **入队临界区只做两件事**：
  1. 保护共享队列状态（head/tail/count/drop_count/写buffer）
  2. 记录 `was_empty = (count==0)`，并处理满队列 drop_oldest
- **点火判定只允许出现一次，且必须在临界区外执行**：
  * 仅当 `was_empty==1` 时开启 TXE 中断启动发送
  * 若 `was_empty==0`（队列本来就非空/正在发送），不做任何 TXE 开关动作
- **备选方式**：使用__disable_irq() / __enable_irq()（更粗，但可用）
- **出队**：在中断中完成，按字节粒度发送（见4.3节）

**删掉/禁止出现的表述**：
- ❌ "临界区结束：恢复TXE中断（但不立即开启）"（矛盾）
- ❌ "临界区里 ENABLE 一次，临界区外 if(was_empty) 又 ENABLE 一次"（重复点火）

> **关键原则**：TXE 开关只在一个地方做：临界区外的 was_empty 点火。

**队列满策略**：
- 丢弃最旧帧（drop oldest）
- 新帧入队
- drop_count++
- **queue_overflow标志**：仅在本次入队触发drop_oldest时，将新入队帧的FLAG.bit3置1

**字节级发送机制**（单一真相）：
- 使用tail和tx_byte_idx跟踪当前发送位置（不再维护tx_frame_idx）
- ISR每次发送1字节：`buffer[tail][tx_byte_idx]`
- tx_byte_idx++，到16后：`tail++`、`count--`、`tx_byte_idx=0`

### 4.5 CRC16-CCITT-FALSE实现

**逐位法**（当前版本，省Flash）：
```c
uint16_t CRC16_CCITT_FALSE(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;  // Init
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;  // 无需XOR
}
```

**查表法**（可选优化，速度快但占Flash 512B）：
```c
static const uint16_t crc16_table[256] = {
    // CRC-16/CCITT-FALSE查表（256×2=512B）
};

uint16_t CRC16_CCITT_FALSE_Table(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;  // Init
    for (uint16_t i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ crc16_table[(crc >> 8) ^ data[i]];
    }
    return crc;  // 无需XOR
}
```

**选择建议**：
- 当前版本：使用逐位法（省Flash，事件触发频率不极端时完全够用）
- 后续优化：压测后如需要，再升级为查表法

---

## 五、集成点

### 5.1 在main.c中的集成

**位置**：`Process_Snapshot_IfReady()` 函数，约第839行

**集成代码**（只保留一次energy定义，避免重复定义）：
```c
if (Validate_And_Count_Event(...))
{
    // ... 现有代码（计数、死区设置等） ...
    
    // 提取小波特征（如果成功）
    uint8_t wavelet_done = 0;
    // ... 小波特征提取 ...
    if (wavelet_extract_success) {
        wavelet_done = 1;
    }
    
    // TF通信上传（非阻塞，队列入队）
    // 注意：src_id参数已删除，SRC_ID字段统一使用编译时宏TF_SRC_ID
    // ENERGY计算：使用pulse_samples明确样本数口径
    uint16_t pulse_samples = end_index - start_index + 1;  // 用于ENERGY的样本数口径
    uint32_t energy = (uint32_t)front_peak_value * (uint32_t)pulse_samples;
    TF_Comm_SendEvent(1, front_peak_value, energy, 
                     (use_pa1 ? 1 : 0), wavelet_done);
}
```

### 5.2 在stm32f10x_it.c中的集成

**添加中断处理函数调用**：
```c
void USART2_IRQHandler(void)
{
    TF_Comm_TX_IRQHandler();  // 在TF_Comm.c中实现
}
```

### 5.3 初始化调用

**在main()函数中**：
```c
int main(void)
{
    // ... 其他初始化 ...
    TF_Comm_Init();  // 初始化TF通信模块
    // ...
}
```

---

## 六、实现检查清单

### 6.1 硬件检查
- [ ] PA2/PA3端口配置正确
- [ ] LED功能已迁移（避免冲突）
- [ ] 3线UART连接（TX/RX/GND，RX预留）

### 6.2 协议检查
- [ ] SOF = 0xAA 0x55（固定值）
- [ ] EOF = 0x0D（固定值）
- [ ] LEN = 13（固定值，payload长度）
- [ ] 所有字段大端序（网络序，使用put_u16_be/put_u32_be函数）
- [ ] CRC16覆盖范围正确（LEN到SEQ，共11字节）
- [ ] CRC16参数正确（CCITT-FALSE）

### 6.3 队列检查
- [ ] 队列大小32帧（512字节）
- [ ] 队列满时丢弃最旧帧
- [ ] drop_count记录正确
- [ ] queue_overflow标志正确（仅在本次入队触发drop时置位）
- [ ] 入队临界区**只负责保护队列共享状态**（head/tail/count/drop_count/写buffer）
- [ ] **点火判定只在临界区外做一次**：仅 `was_empty==1` 才开启TXE
- [ ] **TXE_ENABLE 只允许出现一次：仅 `was_empty==1` 时在临界区外开启**（其余路径不动TXE）
- [ ] 队列非空/发送进行中时，不重复开关TXE
- [ ] 字节级发送机制正确（只用tail和tx_byte_idx，不维护tx_frame_idx）

### 6.4 中断检查
- [ ] TXE中断优先级低于ADC/DMA/AWD
- [ ] 队列空时关闭TXE中断
- [ ] 入队后若空闲则启动发送
- [ ] 中断中只搬运字节，不计算CRC
- [ ] 中断中不手动Clear TXE标志（F1标准库自动清除）

### 6.5 集成检查
- [ ] 在Process_Snapshot_IfReady()中正确调用
- [ ] main.c 中 `energy` **只定义一次**，按 `pulse_samples` 口径计算
- [ ] 能量计算使用uint32_t，pulse_samples变量明确样本数口径
- [ ] 非阻塞发送（不等待完成）
- [ ] 中断处理函数已注册
- [ ] TF_SRC_ID宏定义正确（当前0，阵列时改为1..8）
- [ ] TF_Comm_SendEvent不再传src_id参数

---

## 七、测试验证

### 7.1 功能测试
- [ ] 单帧发送测试（串口助手接收）
- [ ] 连续事件发送测试（序号连续性）
- [ ] 队列满压力测试（drop_count验证）
- [ ] CRC校验测试（模拟误码）

### 7.2 性能测试
- [ ] 发送不阻塞主循环
- [ ] 中断优先级不影响采样
- [ ] 队列满时行为正确

### 7.3 协议测试
- [ ] SOF扫描对齐正确
- [ ] LEN字段解析正确（=13）
- [ ] 接收端按动态LEN读取（SOF→LEN→payload(LEN)→EOF/CRC）
- [ ] CRC校验正确
- [ ] EOF验证正确（=0x0D）
- [ ] queue_overflow标志正确（drop时置位）

---

## 八、注意事项

1. **必须非阻塞**：主循环中只"入队"，不等待发送完成
2. **CRC在主循环计算**：避免中断拖时（当前使用逐位法，省Flash）
3. **中断优先级**：USART2 TXE中断优先级低于ADC/DMA/AWD
4. **队列满策略**：丢弃最旧帧，保持最新雨况，drop时置位queue_overflow标志
5. **发送启动**：入队后通过was_empty判断，仅队列原本为空时才启动发送（收敛成一个点火判定）
6. **并发安全**：入队时使用最小临界区（只在更新共享状态时关闭TXE中断），临界区外才点火
7. **单一真相**：发送进度只用tail和tx_byte_idx，不再维护tx_frame_idx
8. **接口收敛**：TF_Comm_SendEvent不再传src_id，统一用TF_SRC_ID宏
9. **ENERGY口径**：使用pulse_samples变量，明确样本数来源（end_index - start_index + 1）
10. **queue_overflow**：仅在本次入队触发drop_oldest时置位，避免误判为持续丢包
11. **端序处理**：使用put_u16_be/put_u32_be函数统一处理大端序，避免手写移位散落
12. **接收端实现**：必须按SOF→LEN→payload(LEN)→EOF/CRC流程，即使LEN固定也要动态读取
13. **中断标志处理**：F1标准库中，写DR会自动清除TXE标志，无需手动Clear
14. **TXE关键路径优化（可选）**：使用`USART2->DR = byte;`直接寄存器访问，减少库函数开销
15. **RX预留**：PA3(RX)已配置但未启用中断，预留接口用于未来ACK/参数下发/时间同步
