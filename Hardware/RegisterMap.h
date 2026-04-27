#ifndef __REGISTER_MAP_H
#define __REGISTER_MAP_H

#include "stm32f10x.h"

/**
 * @brief 寄存器映射模块
 *
 * 复用原 Modbus 寄存器语义，供 SPI 协议层/其他链路统一访问：
 * - Holding Register：配置类（阈值、从站地址等）
 * - Input Register  ：状态/测量值（雨滴统计、电压、电流强度等）
 *
 * 异常码语义与 Modbus 保持一致：
 *  - 0x01: Illegal Function
 *  - 0x02: Illegal Data Address
 *  - 0x03: Illegal Data Value
 *  - 0x04: Slave Device Failure
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  寄存器映射初始化
 * @param  slave_addr_init 初始从站地址（1~247）
 */
void RegisterMap_Init(uint8_t slave_addr_init);

/**
 * @brief  读取 Holding Register（功能码0x03）
 * @param  addr      寄存器地址
 * @param  exception 异常码指针，为0表示正常
 * @return 16 位寄存器值
 */
uint16_t RegisterMap_ReadHolding(uint16_t addr, uint8_t *exception);

/**
 * @brief  写 Holding Register（功能码0x06/0x10）
 * @param  addr      寄存器地址
 * @param  value     要写入的值
 * @param  exception 异常码指针，为0表示正常
 * @return 1 表示写入成功，0 表示失败（exception 中给出原因）
 */
uint8_t RegisterMap_WriteHolding(uint16_t addr, uint16_t value, uint8_t *exception);

/**
 * @brief  读取 Input Register（功能码0x04）
 * @param  addr      寄存器地址
 * @param  exception 异常码指针，为0表示正常
 * @return 16 位寄存器值
 */
uint16_t RegisterMap_ReadInput(uint16_t addr, uint8_t *exception);

/**
 * @brief  获取当前从站地址（寄存器0x0000）
 */
uint8_t RegisterMap_GetSlaveAddr(void);
void RegisterMap_SetSlaveAddr(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __REGISTER_MAP_H */

