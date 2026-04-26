#ifndef __SPI_PROTO_H
#define __SPI_PROTO_H

#include "stm32f10x.h"
#include <stdint.h>

#include "RegisterMap.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 由链路层在 SPI_SlaveLink.c 中定义，并在此声明为外部变量 */
extern uint8_t  rx_buf[];   /* 请求缓冲区 */
extern uint8_t  tx_resp[];  /* 响应缓冲区 */
extern uint16_t resp_len;   /* 响应长度（字节） */

/**
 * @brief  解析请求并构造响应帧（含SOF 0xA5）
 * @param  rx_len 接收到的请求长度（字节）
 * @retval 1 表示已在 tx_resp[] 中构造好响应（正常或异常）
 *         0 表示不响应（仅在极端情况使用，正常情况下总是返回1）
 *
 * 注意：
 *  - 本函数负责 CRC 校验和异常响应构造（FC|0x80 + ExceptCode）
 *  - 异常码定义见文档：0x01/0x02/0x03/0x04
 */
uint8_t Parse_And_Build_Response(uint16_t rx_len);

/**
 * @brief  构造异常响应帧（含SOF 0xA5，payload长度固定5字节）
 * @param  addr        从站地址
 * @param  fc          功能码
 * @param  except_code 异常码
 *
 * 最终帧格式（8字节）：
 *  SOF + Addr + (FC|0x80) + ExceptCode + CRCL + CRCH
 */
void Build_Exception_Response(uint8_t addr, uint8_t fc, uint8_t except_code);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_PROTO_H */

