#include "SPI_Proto.h"

/* 由 SPI_SlaveLink.c 提供实际存储 */
extern uint8_t  rx_buf[];
extern uint8_t  tx_resp[];
extern uint16_t resp_len;

/* ===================== CRC16(Modbus) 实现 ===================== */

static uint16_t CRC16_Modbus(const uint8_t *buf, uint16_t len)
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

void Build_Exception_Response(uint8_t addr, uint8_t fc, uint8_t except_code)
{
    /* SOF */
    tx_resp[0] = 0xA5;
    tx_resp[1] = addr;
    tx_resp[2] = fc | 0x80;
    tx_resp[3] = except_code;

    /* CRC 从 Addr 开始，长度3字节：Addr, FC|0x80, ExceptCode */
    uint16_t crc = CRC16_Modbus(&tx_resp[1], 3);
    tx_resp[4] = (uint8_t)(crc & 0xFF);        /* CRCL */
    tx_resp[5] = (uint8_t)((crc >> 8) & 0xFF); /* CRCH */

    resp_len = 6;  /* 含SOF在内共6字节，payload固定5字节 */
}

uint8_t Parse_And_Build_Response(uint16_t rx_len)
{
    if (rx_len < 8)
    {
        /* 长度不足，构造异常：Slave Device Failure */
        Build_Exception_Response(0, 0, 0x04);
        return 1;
    }

    uint8_t addr = rx_buf[0];
    uint8_t fc   = rx_buf[1];

    /* CRC 校验 */
    uint16_t crc_rx   = (uint16_t)rx_buf[rx_len - 2] | ((uint16_t)rx_buf[rx_len - 1] << 8);
    uint16_t crc_calc = CRC16_Modbus(rx_buf, rx_len - 2);
    if (crc_rx != crc_calc)
    {
        /* CRC 失败：统一用 0x04（Slave Device Failure） */
        Build_Exception_Response(addr, fc, 0x04);
        return 1;
    }

    /* 解析功能码 */
    if (fc == 0x03 || fc == 0x04)
    {
        /* 读保持/输入寄存器：固定8字节请求 */
        if (rx_len != 8)
        {
            Build_Exception_Response(addr, fc, 0x03); /* Illegal Data Value */
            return 1;
        }

        uint16_t start = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
        uint16_t qty   = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

        if (qty == 0 || qty > 60)
        {
            Build_Exception_Response(addr, fc, 0x03); /* Illegal Data Value */
            return 1;
        }

        tx_resp[0] = 0xA5;     /* SOF */
        tx_resp[1] = addr;
        tx_resp[2] = fc;
        tx_resp[3] = (uint8_t)(qty * 2); /* ByteCount */
        uint16_t idx = 4;

        uint8_t exception = 0;
        for (uint16_t i = 0; i < qty; i++)
        {
            uint16_t reg;
            if (fc == 0x03)
                reg = RegisterMap_ReadHolding(start + i, &exception);
            else
                reg = RegisterMap_ReadInput(start + i, &exception);

            if (exception)
            {
                Build_Exception_Response(addr, fc, exception);
                return 1;
            }

            tx_resp[idx++] = (uint8_t)((reg >> 8) & 0xFF);
            tx_resp[idx++] = (uint8_t)(reg & 0xFF);
        }

        uint16_t crc = CRC16_Modbus(&tx_resp[1], idx - 1);
        tx_resp[idx++] = (uint8_t)(crc & 0xFF);
        tx_resp[idx++] = (uint8_t)((crc >> 8) & 0xFF);
        resp_len = idx;
        return 1;
    }
    else if (fc == 0x06)
    {
        /* 写单个Holding寄存器：固定8字节请求 */
        if (rx_len != 8)
        {
            Build_Exception_Response(addr, fc, 0x03);
            return 1;
        }

        uint16_t reg_addr = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
        uint16_t value    = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

        uint8_t exception = 0;
        if (!RegisterMap_WriteHolding(reg_addr, value, &exception))
        {
            if (exception == 0)
                exception = 0x04; /* Slave Device Failure */
            Build_Exception_Response(addr, fc, exception);
            return 1;
        }

        /* 正常应答：回显 Addr,FC,RegAddr,Value */
        tx_resp[0] = 0xA5;
        tx_resp[1] = addr;
        tx_resp[2] = fc;
        tx_resp[3] = rx_buf[2];
        tx_resp[4] = rx_buf[3];
        tx_resp[5] = rx_buf[4];
        tx_resp[6] = rx_buf[5];
        uint16_t crc = CRC16_Modbus(&tx_resp[1], 6);
        tx_resp[7] = (uint8_t)(crc & 0xFF);
        tx_resp[8] = (uint8_t)((crc >> 8) & 0xFF);
        resp_len   = 9;
        return 1;
    }
    else if (fc == 0x10)
    {
        /* 写多个Holding寄存器：变长帧 */
        if (rx_len < 9)
        {
            Build_Exception_Response(addr, fc, 0x03);
            return 1;
        }

        uint16_t start    = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
        uint16_t reg_cnt  = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
        uint8_t  byte_cnt = rx_buf[6];

        if (reg_cnt == 0 || byte_cnt != (uint8_t)(2 * reg_cnt))
        {
            Build_Exception_Response(addr, fc, 0x03);
            return 1;
        }

        /* 数据区长度检查：7字节头 + ByteCount + 2字节CRC */
        if (rx_len != (uint16_t)(7 + byte_cnt + 2))
        {
            Build_Exception_Response(addr, fc, 0x03);
            return 1;
        }

        uint8_t exception = 0;
        uint16_t data_idx = 7;
        for (uint16_t i = 0; i < reg_cnt; i++)
        {
            uint16_t value = ((uint16_t)rx_buf[data_idx] << 8) | rx_buf[data_idx + 1];
            data_idx += 2;
            if (!RegisterMap_WriteHolding((uint16_t)(start + i), value, &exception))
            {
                if (exception == 0)
                    exception = 0x04;
                Build_Exception_Response(addr, fc, exception);
                return 1;
            }
        }

        /* 正常应答：Addr,FC,StartRegHi,StartRegLo,RegCntHi,RegCntLo */
        tx_resp[0] = 0xA5;
        tx_resp[1] = addr;
        tx_resp[2] = fc;
        tx_resp[3] = rx_buf[2];
        tx_resp[4] = rx_buf[3];
        tx_resp[5] = rx_buf[4];
        tx_resp[6] = rx_buf[5];
        uint16_t crc = CRC16_Modbus(&tx_resp[1], 6);
        tx_resp[7] = (uint8_t)(crc & 0xFF);
        tx_resp[8] = (uint8_t)((crc >> 8) & 0xFF);
        resp_len   = 9;
        return 1;
    }
    else
    {
        /* 不支持的功能码 */
        Build_Exception_Response(addr, fc, 0x01); /* Illegal Function */
        return 1;
    }
}

