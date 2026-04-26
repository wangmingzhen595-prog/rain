/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "AD.h"
#include "Modbus_Slave.h"
#include "SPI_SlaveLink.h"
#include "TF_Comm.h"

/* 由 main.c 定义的全局变量 */
extern volatile uint32_t watchdog_trigger_count;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void)  { while (1) {} }
void UsageFault_Handler(void){ while (1) {} }
void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)    {}
void SysTick_Handler(void)   {}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/**
  * @brief  DMA1 Channel1 中断处理（ADC 双通道连续采样）
  * @note   双通道扫描模式：AD_Value[i*2] = PA0(高增益), AD_Value[i*2+1] = PA1(低增益)
  *         每个半传输处理50对（共100对/次，采样周期≈42us）
  *         ISR 只做最小化操作：写入环形缓冲区、推进索引、计数。
  *         小波能量积分移至主循环批量处理（WaveletEnergy_ProcessNewSamples）。
  */
void DMA1_Channel1_IRQHandler(void)
{
    extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];
    extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];
    extern volatile uint16_t ring_write_index_ch0;
    extern volatile uint16_t ring_write_index_ch1;
    extern volatile uint32_t ring_write_total_ch0;
    extern volatile uint32_t ring_write_total_ch1;

    /* 半传输中断：处理前半缓冲（0~49，共50对） */
    if (DMA_GetITStatus(DMA1_IT_HT1))
    {
        uint8_t i;
        for (i = 0; i < 50; i++)
        {
            /* PA0：高增益通道 → 写入环形缓冲区 */
            uint16_t ch0_value = AD_Value[i * 2];
            uint16_t current_index = ring_write_index_ch0;
            adc_ring_buffer_ch0[current_index] = ch0_value;
            ring_write_index_ch0 = (current_index + 1) % RING_BUFFER_SIZE;
            ring_write_total_ch0++;

            sampling_tick_counter++;

            /* PA1：低增益通道 → 保留供后续扩展 */
            uint16_t ch1_value = AD_Value[i * 2 + 1];
            uint16_t ch1_index = ring_write_index_ch1;
            adc_ring_buffer_ch1[ch1_index] = ch1_value;
            ring_write_index_ch1 = (ch1_index + 1) % RING_BUFFER_SIZE;
            ring_write_total_ch1++;
        }
        DMA_ClearITPendingBit(DMA1_IT_HT1);
    }

    /* 传输完成中断：处理后半缓冲（50~99，共50对） */
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        uint8_t i;
        for (i = 50; i < 100; i++)
        {
            /* PA0：高增益通道 → 写入环形缓冲区 */
            uint16_t ch0_value = AD_Value[i * 2];
            uint16_t current_index = ring_write_index_ch0;
            adc_ring_buffer_ch0[current_index] = ch0_value;
            ring_write_index_ch0 = (current_index + 1) % RING_BUFFER_SIZE;
            ring_write_total_ch0++;

            sampling_tick_counter++;

            /* PA1：低增益通道 */
            uint16_t ch1_value = AD_Value[i * 2 + 1];
            uint16_t ch1_index = ring_write_index_ch1;
            adc_ring_buffer_ch1[ch1_index] = ch1_value;
            ring_write_index_ch1 = (ch1_index + 1) % RING_BUFFER_SIZE;
            ring_write_total_ch1++;
        }
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

/**
  * @brief  ADC 模拟看门狗中断（保留，用于触发计数）
  */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
    {
        watchdog_trigger_count++;
        ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    }
}

/**
  * @brief  SPI1 中断（从机链路层）
  */
void SPI1_IRQHandler(void)
{
    SPI_SlaveLink_SPI1_IRQHandler();
}

/**
  * @brief  EXTI4 中断（NSS 边沿，SPI 事务边界）
  */
void EXTI4_IRQHandler(void)
{
    SPI_SlaveLink_EXTI4_IRQHandler();
}

/**
  * @brief  USART2 中断（Modbus RTU RX）
  */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = (uint8_t)USART_ReceiveData(USART2);
        Modbus_Slave_RX_IRQHandler(byte);
    }
    /* 注意：TF_Comm 已迁移至 USART3，不再在 USART2 中处理 */
}

/**
  * @brief  USART3 中断（TF_Comm 主动上报 TX）
  */
void USART3_IRQHandler(void)
{
    TF_Comm_TX_IRQHandler();
}

/**
  * @brief  TIM2 中断（1ms Tick，Modbus 帧定时 + 系统时间）
  */
void TIM2_IRQHandler(void)
{
    extern volatile uint32_t g_sys_time_ms;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        Modbus_Slave_1msTick();
        g_sys_time_ms++;  /* 由硬件定时器驱动，精度与主循环解耦 */
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
