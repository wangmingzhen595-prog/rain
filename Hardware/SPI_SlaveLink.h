#ifndef __SPI_SLAVELINK_H
#define __SPI_SLAVELINK_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPI 从机链路层初始化
 * @param self_id 本从控节点地址（1~247），与主控V6配置一致
 */
void SPI_SlaveLink_Init(uint8_t self_id);

/**
 * @brief EXTI4 中断处理（NSS 边沿），由 stm32f10x_it.c 中的 EXTI4_IRQHandler 调用
 */
void SPI_SlaveLink_EXTI4_IRQHandler(void);

/**
 * @brief SPI1 中断处理（RXNE/TXE），由 stm32f10x_it.c 中的 SPI1_IRQHandler 调用
 */
void SPI_SlaveLink_SPI1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_SLAVELINK_H */

