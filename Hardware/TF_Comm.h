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
                         uint8_t wavelet_done);
uint32_t TF_Comm_GetDropCount(void);
void TF_Comm_TX_IRQHandler(void);  // USART2发送中断处理（在stm32f10x_it.c中调用）

#endif
