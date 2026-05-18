#include "SPI_SlaveLink.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#include "SPI_Proto.h"
#include "RegisterMap.h"

/* ===================== 配置宏 ===================== */

/* 事务级复位：默认关闭，只有现场确实存在跨事务污染时再打开 */
#ifndef SPI_SLAVE_TXN_RESET_ENABLE
#define SPI_SLAVE_TXN_RESET_ENABLE   0
#endif

/* 接收/发送缓冲区大小（根据最大寄存器数和协议长度预留） */
#define RX_BUF_SIZE      80
#define TX_BUF_SIZE      80

/* 可写寄存器最大数量（0x10），防止请求撑爆缓冲区 */
#ifndef MAX_REG_COUNT
#define MAX_REG_COUNT    32
#endif

/* ===================== 链路层状态机 ===================== */

typedef enum
{
    STATE_IDLE = 0,
    STATE_RX_REQ,
    STATE_TX_RESP,
    STATE_DROP   /* 地址不匹配/异常时，丢弃本事务直至 NSS 上升沿 */
} SPI_SlaveState_t;

/* 请求/响应缓冲区，供 SPI_Proto 使用 */
uint8_t  rx_buf[RX_BUF_SIZE];
uint8_t  tx_resp[TX_BUF_SIZE];
uint16_t resp_len = 0;

/* 内部状态 */
static volatile SPI_SlaveState_t s_state      = STATE_IDLE;
static volatile uint16_t         s_rx_idx     = 0;
static volatile uint16_t         s_expected   = 8;   /* 默认请求长度8字节，0x10时动态调整 */
static volatile uint8_t          s_resp_ready = 0;
static volatile uint8_t          s_txe_enabled = 0;
static volatile uint16_t         s_tx_idx     = 0;   /* 发送索引，NSS上升沿重置 */

static uint8_t                   s_self_node_id = 1;

/* ===================== TXE中断管理封装（方案C：软件标志位） ===================== */

static inline void TXE_Enable(void)
{
    s_txe_enabled = 1;
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}

static inline void TXE_Disable(void)
{
    s_txe_enabled = 0;
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
}

/* ===================== 硬件初始化 ===================== */

void SPI_SlaveLink_Init(uint8_t self_id)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef  spi;
    NVIC_InitTypeDef nvic;
    EXTI_InitTypeDef exti;

    if (self_id == 0)
    {
        self_id = 1;
    }
    s_self_node_id = self_id;

    /* 寄存器映射初始化（从站地址镜像） */
    RegisterMap_Init(self_id);

    /* 开启时钟：GPIOA, AFIO, SPI1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_SPI1, ENABLE);

    /* PA4 = NSS: 上拉输入 + EXTI4 */
    gpio.GPIO_Pin  = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA5 = SCK: 浮空输入 */
    gpio.GPIO_Pin  = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* PA7 = MOSI: 上拉输入 */
    gpio.GPIO_Pin  = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* PA6 = MISO: 复用推挽输出，由 SPI 外设控制三态 */
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 配置 EXTI4 连接到 PA4 (NSS) */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    exti.EXTI_Line    = EXTI_Line4;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  /* 上升/下降沿均触发 */
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    /* NVIC：EXTI4 中断 */
    nvic.NVIC_IRQChannel = EXTI4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* SPI1 配置为从机，Mode 0，硬件NSS */
    SPI_I2S_DeInit(SPI1);
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Slave;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Hard;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; /* 从机时不重要，仅占位 */
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &spi);

    /* 默认关闭 RXNE/TXE 中断，由 NSS 下降沿开启 RXNE */
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);

    /* 使能 SPI1 */
    SPI_Cmd(SPI1, ENABLE);

    /* NVIC：SPI1 中断 */
    nvic.NVIC_IRQChannel = SPI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* 初始状态 */
    s_state      = STATE_IDLE;
    s_rx_idx     = 0;
    s_expected   = 8;
    s_resp_ready = 0;
}

/* ===================== EXTI4 中断：NSS 边沿 ===================== */

void SPI_SlaveLink_EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);

        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET)
        {
            /* NSS 下降沿 - 事务开始 */

            /* 1. 关闭TXE中断（防止CPU被轰炸） */
            TXE_Disable();  /* 用软件标志位封装，避免竞态 */

            /* 2. 清空RXNE残留 + OVR标志（关键！） */
            volatile uint8_t dummy;
            while (SPI1->SR & SPI_I2S_FLAG_RXNE)
                dummy = SPI1->DR;
            /* 清OVR：先判断再清，避免读走有效字节 */
            if (SPI1->SR & SPI_I2S_FLAG_OVR)
            {
                dummy = SPI1->SR;
                dummy = SPI1->DR;
            }
            (void)dummy;

            /* 3. DR预填0xFF（保证第一字节输出0xFF） */
            SPI1->DR = 0xFF;

            /* 4. 复位状态机（包括发送索引） */
            s_rx_idx     = 0;
            s_tx_idx     = 0;  /* 重置发送索引 */
            s_expected   = 8;  /* 默认8字节，0x10时动态调整 */
            s_state      = STATE_RX_REQ;
            s_resp_ready = 0;

            /* 5. 启用RXNE中断 */
            SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
        }
        else
        {
            /* NSS 上升沿 - 事务结束 */
            TXE_Disable();
            SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);

            /* 可选：事务级复位（宏控制，默认关闭，遇到跨事务污染再打开） */
#if SPI_SLAVE_TXN_RESET_ENABLE
            SPI_Cmd(SPI1, DISABLE);
#endif

            /* 关键：清RX残留 + OVR（防止跨事务污染）
             * 顺序不能变：先while(RXNE)读DR直到空，再判断OVR再清
             */
            volatile uint8_t dummy;
            while (SPI1->SR & SPI_I2S_FLAG_RXNE)
                dummy = SPI1->DR;
            if (SPI1->SR & SPI_I2S_FLAG_OVR)
            {
                dummy = SPI1->SR;
                dummy = SPI1->DR;
            }
            (void)dummy;

#if SPI_SLAVE_TXN_RESET_ENABLE
            SPI_Cmd(SPI1, ENABLE);
#endif

            s_state      = STATE_IDLE;
            s_rx_idx     = 0;
            s_resp_ready = 0;
        }
    }
}

/* ===================== SPI1 中断：RXNE/TXE ===================== */

void SPI_SlaveLink_SPI1_IRQHandler(void)
{
    /* RXNE：接收 MOSI 数据 */
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
    {
        uint8_t byte = (uint8_t)SPI1->DR;

        if (s_state == STATE_RX_REQ && s_rx_idx < RX_BUF_SIZE)
        {
            rx_buf[s_rx_idx++] = byte;

            /* 关键：0x10变长帧动态计算长度
             * rx_idx==7 时，rx_buf[6] 就是 ByteCount（已收到索引0..6共7字节）
             */
            if (s_rx_idx == 7 && rx_buf[1] == 0x10)
            {
                uint16_t reg_count = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
                uint8_t  byte_cnt  = rx_buf[6];
                s_expected         = (uint16_t)(9 + 2 * reg_count);  /* 7字节头 + 2*RegCnt 数据 + 2字节CRC */

                /* 双保险：ByteCount 合理性检查 & expected_len 上限防呆 */
                if (reg_count == 0 ||
                    byte_cnt != (uint8_t)(2 * reg_count) ||
                    reg_count > MAX_REG_COUNT ||
                    s_expected > RX_BUF_SIZE)
                {
                    Build_Exception_Response(rx_buf[0], rx_buf[1], 0x03); /* Illegal Data Value */
                    resp_len    = resp_len; /* 已在 Build_Exception_Response 中设置 */
                    s_resp_ready = 1;
                    s_state      = STATE_TX_RESP;
                    TXE_Enable();
                    return;
                }
            }

            /* 收满请求帧 */
            if (s_rx_idx >= s_expected)
            {
                /* 先校验 Addr 是否等于本机 node_id（主控V6也做此判断） */
                if (rx_buf[0] != s_self_node_id)
                {
                    /* 地址不匹配，进入 STATE_DROP（语义清晰：本事务丢弃直到 NSS 上升沿） */
                    s_state      = STATE_DROP;
                    s_resp_ready = 0;
                    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
                    SPI1->DR = 0xFF;  /* 喂0xFF作为静默输出（让MISO确定） */
                    /* NSS 上升沿 EXTI 会统一回到 STATE_IDLE */
                    return;
                }

                /* 解析并构造响应（正常或异常） */
                (void)Parse_And_Build_Response(s_rx_idx);
                s_resp_ready = 1;
                s_state      = STATE_TX_RESP;
                TXE_Enable();
            }
        }

        /* 在 RX 阶段也要喂 DR，保证 MISO 输出 0xFF
         * 关键：双保险防竞态（软件标志 + 硬件寄存器都检查）
         * 注意：统一用 SPI_I2S_IT_TXE，保持与其他代码一致
         */
        if (!s_resp_ready && !s_txe_enabled && !(SPI1->CR2 & SPI_I2S_IT_TXE))
        {
            SPI1->DR = 0xFF;
        }
    }

    /* TXE：发送 MISO 数据 */
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
    {
        if (s_resp_ready && resp_len > 0)
        {
            if (s_tx_idx < resp_len)
            {
                SPI1->DR = tx_resp[s_tx_idx++];
            }

            /* 发送完成，立即关闭 TXE（不要在同一次 TXE 里再写 DR=0xFF） */
            if (s_tx_idx >= resp_len)
            {
                TXE_Disable();
                s_resp_ready = 0;
                /* s_tx_idx 由 NSS 下降沿重置，这里不重置 */
            }
        }
        else
        {
            /* 响应未就绪或已发完，输出 0xFF 作为 dummy */
            SPI1->DR = 0xFF;
        }
    }
}

