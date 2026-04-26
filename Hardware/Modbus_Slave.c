#include "Modbus_Slave.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "RegisterMap.h"

/* ===================== 内部状态与配置 ===================== */

static uint8_t  modbus_slave_addr = 1;
static volatile uint8_t  modbus_rx_buf[MODBUS_RX_BUFFER_SIZE];
static volatile uint16_t modbus_rx_len = 0;
static volatile uint32_t modbus_last_rx_ms = 0;
static volatile uint8_t  modbus_frame_busy = 0;
static volatile uint32_t modbus_ms_counter = 0;

/* ===================== CRC16 实现（Modbus 标准） ===================== */

static uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ===================== 1ms Tick ===================== */

void Modbus_Slave_1msTick(void)
{
    modbus_ms_counter++;
}

/* ===================== USART2 与 TIM2 初始化 ===================== */

void Modbus_Slave_Init(uint8_t slave_id, uint32_t baudrate)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;
    TIM_TimeBaseInitTypeDef tim;

    modbus_slave_addr = slave_id;
    RegisterMap_Init(slave_id);

    /* 开启 GPIOA 与 USART2、TIM2 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* PA2 = TX, PA3 = RX */
    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* USART2 配置：8N1，全双工（当前主要用RX + 阻塞发送） */
    USART_StructInit(&usart);
    usart.USART_BaudRate            = baudrate;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    /* 使能 RXNE 中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    /* NVIC：USART2 中断 */
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* TIM2 作为 1ms Tick 定时器 */
    TIM_TimeBaseStructInit(&tim);
    /* 假设 APB1 定时器时钟 72MHz：72MHz / (7200 * 10) = 1kHz */
    tim.TIM_Prescaler         = 7200 - 1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = 10 - 1;     /* 1ms 周期 */
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    /* NVIC：TIM2 中断 */
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* ===================== RX 中断入口 ===================== */

void Modbus_Slave_RX_IRQHandler(uint8_t byte)
{
    if (modbus_frame_busy)
    {
        /* 正在处理/发送上一帧，丢弃新字节 */
        return;
    }

    if (modbus_rx_len < MODBUS_RX_BUFFER_SIZE)
    {
        modbus_rx_buf[modbus_rx_len++] = byte;
    }
    else
    {
        /* 溢出：丢弃整帧 */
        modbus_rx_len = 0;
    }
    modbus_last_rx_ms = modbus_ms_counter;
}

/* ===================== 寄存器访问辅助 ===================== */

static uint16_t Read_Holding_Reg(uint16_t addr, uint8_t *exception)
{
    return RegisterMap_ReadHolding(addr, exception);
}

static uint8_t Write_Holding_Reg(uint16_t addr, uint16_t value, uint8_t *exception)
{
    uint8_t ok = RegisterMap_WriteHolding(addr, value, exception);
    modbus_slave_addr = RegisterMap_GetSlaveAddr();
    return ok;
}

static uint16_t Read_Input_Reg(uint16_t addr, uint8_t *exception)
{
    return RegisterMap_ReadInput(addr, exception);
}

/* ===================== 应答发送（阻塞方式） ===================== */

static void Modbus_Send_Response(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(USART2, buf[i]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        ;
}

/* ===================== 帧处理 ===================== */

static void Modbus_Handle_Frame(uint8_t *frame, uint16_t len)
{
    if (len < 8)
        return;

    uint8_t addr = frame[0];
    uint8_t func = frame[1];
    uint16_t crc_rx = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    uint16_t crc_calc = Modbus_CRC16(frame, len - 2);
    if (crc_rx != crc_calc)
        return;

    /* 地址过滤：广播(0) 或 非本从站地址都丢弃且不应答 */
    modbus_slave_addr = RegisterMap_GetSlaveAddr();
    if (addr == 0 || addr != modbus_slave_addr)
        return;

    uint8_t resp[MODBUS_RX_BUFFER_SIZE];
    uint16_t resp_len = 0;
    uint8_t exception = 0;

    if (func == 0x03 || func == 0x04)
    {
        if (len < 8)
            return;
        uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t qty   = ((uint16_t)frame[4] << 8) | frame[5];
        if (qty == 0 || qty > 60)
        {
            exception = 0x03; /* Illegal Data Value */
        }
        else
        {
            resp[0] = addr;
            resp[1] = func;
            resp[2] = (uint8_t)(qty * 2);
            resp_len = 3;
            for (uint16_t i = 0; i < qty; i++)
            {
                uint16_t reg;
                if (func == 0x03)
                    reg = Read_Holding_Reg(start + i, &exception);
                else
                    reg = Read_Input_Reg(start + i, &exception);

                if (exception)
                    break;

                resp[resp_len++] = (uint8_t)((reg >> 8) & 0xFF);
                resp[resp_len++] = (uint8_t)(reg & 0xFF);
            }
        }
    }
    else if (func == 0x06)
    {
        if (len < 8)
            return;
        uint16_t reg_addr = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t value    = ((uint16_t)frame[4] << 8) | frame[5];

        if (!Write_Holding_Reg(reg_addr, value, &exception))
        {
            /* exception 已在 Write_Holding_Reg 中设置 */
        }
        else
        {
            /* 正常回显请求前 6 字节（Addr,Func,AddrHi,AddrLo,ValHi,ValLo） */
            for (uint8_t i = 0; i < 6; i++)
            {
                resp[i] = frame[i];
            }
            resp_len = 6;
        }
    }
    else if (func == 0x10)
    {
        if (len < 9)
            return;
        uint16_t start    = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t reg_cnt  = ((uint16_t)frame[4] << 8) | frame[5];
        uint8_t  byte_cnt = frame[6];

        if (reg_cnt == 0 || byte_cnt != (uint8_t)(2 * reg_cnt) ||
            len != (uint16_t)(7 + byte_cnt + 2))
        {
            exception = 0x03;
        }
        else
        {
            uint16_t data_idx = 7;
            for (uint16_t i = 0; i < reg_cnt; i++)
            {
                uint16_t value = ((uint16_t)frame[data_idx] << 8) | frame[data_idx + 1];
                data_idx += 2;
                if (!Write_Holding_Reg((uint16_t)(start + i), value, &exception))
                    break;
            }

            if (!exception)
            {
                resp[0] = addr;
                resp[1] = func;
                resp[2] = frame[2];
                resp[3] = frame[3];
                resp[4] = frame[4];
                resp[5] = frame[5];
                resp_len = 6;
            }
        }
    }
    else
    {
        exception = 0x01; /* Illegal Function */
    }

    if (exception)
    {
        resp[0] = addr;
        resp[1] = func | 0x80;
        resp[2] = exception;
        resp_len = 3;
    }

    uint16_t crc = Modbus_CRC16(resp, resp_len);
    resp[resp_len++] = (uint8_t)(crc & 0xFF);       /* CRC_L */
    resp[resp_len++] = (uint8_t)((crc >> 8) & 0xFF);/* CRC_H */

    Modbus_Send_Response(resp, resp_len);
}

/* ===================== 主循环处理 ===================== */

void Modbus_Slave_Process(void)
{
    if (modbus_frame_busy)
        return;

    uint16_t len;

    /* 判断是否达到 3.5 字符空闲时间并且有数据 */
    if (modbus_rx_len > 0 &&
        (modbus_ms_counter - modbus_last_rx_ms) >= MODBUS_T3_5_MS)
    {
        __disable_irq();
        len = modbus_rx_len;
        if (len > MODBUS_RX_BUFFER_SIZE)
            len = MODBUS_RX_BUFFER_SIZE;
        uint8_t frame[MODBUS_RX_BUFFER_SIZE];
        for (uint16_t i = 0; i < len; i++)
        {
            frame[i] = modbus_rx_buf[i];
        }
        modbus_rx_len = 0;
        __enable_irq();

        modbus_frame_busy = 1;
        if (len >= 8)
            Modbus_Handle_Frame(frame, len);
        modbus_frame_busy = 0;
    }
}

