#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "stm32f10x.h"

/* Modbus 接收缓冲区长度 */
#define MODBUS_RX_BUFFER_SIZE   256

/* 3.5 字符时间（ms），当前仅适配 9600bps，后续若修改波特率需同步调整 */
#define MODBUS_T3_5_MS          5

void Modbus_Slave_Init(uint8_t slave_id, uint32_t baudrate);
void Modbus_Slave_Process(void);                 /* 在主循环中周期调用 */
void Modbus_Slave_RX_IRQHandler(uint8_t byte);   /* 在 USART2 IRQ 中每收 1 字节调用 */
void Modbus_Slave_1msTick(void);                 /* 在 1ms 定时中断中调用 */

#endif

