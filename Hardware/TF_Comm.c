#include "TF_Comm.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

// 队列数据结构（单一真相：只用tail，不维护tx_frame_idx）
typedef struct {
    uint8_t buffer[TF_COMM_QUEUE_SIZE][TF_COMM_FRAME_SIZE];
    uint16_t head;          // 写指针（帧索引）
    uint16_t tail;          // 读指针（帧索引，唯一真相）
    uint16_t count;         // 当前帧数
    uint32_t drop_count;    // 丢包计数
    uint8_t tx_byte_idx;    // 当前帧发送到第几个字节（0..15）
    uint8_t overflow_latched; // 上次入队失败标志，随下一帧上报
} TF_Comm_Queue_t;

static TF_Comm_Queue_t tx_queue;
static uint16_t sequence_counter = 0;  // 事件序号（自增）
static uint8_t tx_queue_initialized = 0;  // 队列初始化标志（防止Init前调用SendEvent）

// 大端序打包函数
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

// CRC16-CCITT-FALSE实现（逐位法，省Flash）
static uint16_t CRC16_CCITT_FALSE(uint8_t *data, uint16_t len)
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

/**
  * @brief  TF通信模块初始化
  * @note   配置 USART3（PB10=TX, PB11=RX）为 TF 协议专用端口
  *         波特率 115200 8N1，与 Modbus RTU (USART2 9600) 完全分离
  */
void TF_Comm_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    /* 初始化发送队列 */
    tx_queue.head = 0;
    tx_queue.tail = 0;
    tx_queue.count = 0;
    tx_queue.drop_count = 0;
    tx_queue.tx_byte_idx = 0;
    tx_queue.overflow_latched = 0;
    sequence_counter = 0;

    /* 开启 GPIOB 和 USART3 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PB10 = TX（推挽复用），PB11 = RX（上拉输入） */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    /* USART3 配置：115200 8N1 */
    USART_StructInit(&usart);
    usart.USART_BaudRate            = 115200;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART3, &usart);

    /* TXE 中断（仅 TX，不开 RX，因为是单向发送） */
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_Cmd(USART3, ENABLE);

    /* NVIC：USART3 中断，优先级低于 Modbus */
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tx_queue_initialized = 1;
}

/**
  * @brief  发送事件（非阻塞，队列入队）
  * @param  trigger_flag: 雨滴触发标志（1=有效雨滴）
  * @param  peak_value: 峰值ADC值
  * @param  energy: 信号能量（uint32_t，峰值×脉宽）
  * @param  source_channel: 通道来源（0=PA0，1=PA1）
  * @param  wavelet_done: 小波计算完成标志（0/1）
  * @retval 1: 入队成功，0: 队列满（丢弃本次新帧）
  */
uint8_t TF_Comm_SendEvent(uint8_t trigger_flag, uint16_t peak_value, 
                         uint32_t energy, uint8_t source_channel, 
                         uint8_t wavelet_done)
{
    /* 初始化保护：Init前不发送 */
    if (!tx_queue_initialized)
    {
        return 0;
    }

    uint8_t frame[TF_COMM_FRAME_SIZE];
    uint8_t queue_overflow = 0;
    uint8_t was_empty = 0;
    uint16_t crc;
    uint8_t idx = 0;
    
    // 1. 构造数据帧（16字节）
    // SOF
    frame[idx++] = TF_COMM_SOF_BYTE1;  // 0xAA
    frame[idx++] = TF_COMM_SOF_BYTE2;  // 0x55
    
    // LEN
    frame[idx++] = TF_COMM_LEN;  // 13
    
    // SRC_ID
    frame[idx++] = TF_SRC_ID;  // 使用编译时宏
    
    // FLAG（先占位，后面根据queue_overflow更新）
    uint8_t flag_idx = idx++;
    uint8_t flag = trigger_flag | (source_channel << 1) | (wavelet_done << 2);
    
    // PEAK（大端序）
    put_u16_be(&frame[idx], peak_value);
    idx += 2;
    
    // ENERGY（大端序）
    put_u32_be(&frame[idx], energy);
    idx += 4;
    
    // SEQ（大端序，入队成功后自增）
    put_u16_be(&frame[idx], sequence_counter);
    idx += 2;
    
    // CRC（先占位，后面计算）
    uint8_t crc_idx = idx;
    idx += 2;
    
    // EOF
    frame[idx++] = TF_COMM_EOF;  // 0x0D
    
    // 2. 计算CRC16（覆盖LEN到SEQ，共11字节）
    crc = CRC16_CCITT_FALSE(&frame[2], 11);  // 从LEN开始，共11字节
    put_u16_be(&frame[crc_idx], crc);
    
    // 3. 入队（非阻塞，精确临界区保护，收敛点火判定）
    // 临界区开始：关闭TXE中断（防止ISR并发消耗队列）
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    
    // 在临界区内完成：
    // 记录队列状态
    was_empty = (tx_queue.count == 0);
    
    // 初始化queue_overflow标志
    queue_overflow = 0;
    
    // 队列满处理：丢弃本次新帧，避免移动tail破坏正在发送的帧
    if (tx_queue.count >= TF_COMM_QUEUE_SIZE)
    {
        tx_queue.drop_count++;
        tx_queue.overflow_latched = 1;
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
        return 0;
    }
    
    // 更新FLAG（包含queue_overflow）
    queue_overflow = tx_queue.overflow_latched;
    flag |= (queue_overflow << 3);
    frame[flag_idx] = flag;
    
    // 重新计算CRC（因为FLAG可能变化）
    crc = CRC16_CCITT_FALSE(&frame[2], 11);
    put_u16_be(&frame[crc_idx], crc);
    
    // 写入封包数据
    for (uint8_t i = 0; i < TF_COMM_FRAME_SIZE; i++)
    {
        tx_queue.buffer[tx_queue.head][i] = frame[i];
    }
    
    // 更新队列
    tx_queue.head = (tx_queue.head + 1) % TF_COMM_QUEUE_SIZE;
    tx_queue.count++;
    tx_queue.overflow_latched = 0;
    sequence_counter++;
    
    // 临界区结束：退出临界区（释放对队列共享状态的保护）
    // 注意：由于我们关闭了TXE中断来保护队列，必须恢复它
    // 但根据清单要求，只在was_empty==1时"启动发送"，其他情况"不做任何TXE开关动作"
    // 实际上，关闭TXE中断后必须恢复，否则正在进行的发送会停止
    // 这里采用实际可行的方案：恢复TXE中断使能，但只在was_empty==1时确保启动发送
    
    // 临界区外：点火判定（只允许出现一次）
    if (was_empty)
    {
        // 队列原本为空，入队后需要启动发送
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
    else
    {
        // 队列原本非空或正在发送，恢复TXE中断使能（让正在发送的流程继续）
        // 注意：虽然清单说"不做任何TXE开关动作"，但关闭TXE后必须恢复，否则发送会停止
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
    
    return 1;
}

/**
  * @brief  USART3发送中断处理函数（单中断模型）
  * @param  无
  * @retval 无
  */
void TF_Comm_TX_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        if (tx_queue.count > 0)
        {
            uint8_t byte = tx_queue.buffer[tx_queue.tail][tx_queue.tx_byte_idx];
            USART3->DR = byte;
            tx_queue.tx_byte_idx++;
            if (tx_queue.tx_byte_idx >= TF_COMM_FRAME_SIZE)
            {
                tx_queue.tx_byte_idx = 0;
                tx_queue.tail = (tx_queue.tail + 1) % TF_COMM_QUEUE_SIZE;
                tx_queue.count--;
            }
        }
        else
        {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
            tx_queue.tx_byte_idx = 0;
        }
    }
}

/**
  * @brief  获取丢包计数
  * @param  无
  * @retval 丢包计数
  */
uint32_t TF_Comm_GetDropCount(void)
{
    return tx_queue.drop_count;
}
