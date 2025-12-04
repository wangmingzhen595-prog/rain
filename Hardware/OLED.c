#include "stm32f10x.h"
#include "OLED_Font.h"
#include "Delay.h"

/* 引脚配置 - 根据你的接线修改 */
#define OLED_SCL_PIN GPIO_Pin_8
#define OLED_SDA_PIN GPIO_Pin_9
#define OLED_PORT    GPIOB

#define OLED_SCL(x)  GPIO_WriteBit(OLED_PORT, OLED_SCL_PIN, (BitAction)(x))
#define OLED_SDA(x)  GPIO_WriteBit(OLED_PORT, OLED_SDA_PIN, (BitAction)(x))

/**
  * @brief  次方计算
  * @param  X 底数
  * @param  Y 指数
  * @retval X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  I2C初始化
  * @param  无
  * @retval 无
  */
void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 开启GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置SCL和SDA为开漏输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_Init(OLED_PORT, &GPIO_InitStructure);
    
    // 初始化为高电平
    OLED_SCL(1);
    OLED_SDA(1);
}

/**
  * @brief  I2C开始信号
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
    OLED_SDA(1);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SDA(0);
    Delay_us(5);
    OLED_SCL(0);
    Delay_us(5);
}

/**
  * @brief  I2C停止信号
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
    OLED_SDA(0);
    Delay_us(5);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SDA(1);
    Delay_us(5);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_SDA(Byte & (0x80 >> i));
        Delay_us(2);
        OLED_SCL(1);
        Delay_us(5);
        OLED_SCL(0);
        Delay_us(2);
    }
    
    // 等待应答
    OLED_SDA(1);
    Delay_us(2);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SCL(0);
}

/**
  * @brief  OLED写命令
  * @param  Command 命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);  // 从机地址
    OLED_I2C_SendByte(0x00);  // 写命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);  // 从机地址
    OLED_I2C_SendByte(0x40);  // 写数据
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/**
  * @brief  设置光标位置
  * @param  Y 行位置 (0-7)
  * @param  X 列位置 (0-127)
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 + Y);              // 设置页地址
    OLED_WriteCommand(0x10 + (X >> 4));       // 设置列地址高4位
    OLED_WriteCommand(0x00 + (X & 0x0F));     // 设置列地址低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置 (1-4)
  * @param  Column 列位置 (1-16)
  * @param  Char 要显示的字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    uint8_t page = (Line - 1) * 2;
    uint8_t col = (Column - 1) * 8;
    
    // 显示上半部分
    OLED_SetCursor(page, col);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    
    // 显示下半部分
    OLED_SetCursor(page + 1, col);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置 (1-4)
  * @param  Column 起始列位置 (1-16)
  * @param  String 要显示的字符串
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置 (1-4)
  * @param  Column 起始列位置 (1-16)
  * @param  Number 要显示的数字
  * @param  Length 要显示数字的长度
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示有符号数字
  * @param  Line 起始行位置 (1-4)
  * @param  Column 起始列位置 (1-16)
  * @param  Number 要显示的数字
  * @param  Length 要显示数字的长度
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示十六进制数字
  * @param  Line 起始行位置 (1-4)
  * @param  Column 起始列位置 (1-16)
  * @param  Number 要显示的数字
  * @param  Length 要显示数字的长度
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
    // 初始化I2C
    OLED_I2C_Init();
    
    // 延时等待OLED上电稳定
    Delay_ms(100);
    
    // SSD1306初始化序列
    OLED_WriteCommand(0xAE); // 关闭显示
    
    OLED_WriteCommand(0x20); // 设置内存地址模式
    OLED_WriteCommand(0x10); // 页地址模式
    
    OLED_WriteCommand(0xB0); // 设置页起始地址
    
    OLED_WriteCommand(0xC8); // 设置COM扫描方向
    OLED_WriteCommand(0x00); // 设置低列地址
    OLED_WriteCommand(0x10); // 设置高列地址
    
    OLED_WriteCommand(0x40); // 设置显示起始行
    
    OLED_WriteCommand(0x81); // 设置对比度
    OLED_WriteCommand(0xFF); // 对比度值
    
    OLED_WriteCommand(0xA1); // 设置段重映射
    OLED_WriteCommand(0xA6); // 设置正常显示
    
    OLED_WriteCommand(0xA8); // 设置多路复用比率
    OLED_WriteCommand(0x3F); // duty = 1/64
    
    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00); // 无偏移
    
    OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80); // 设置分频比
    
    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1);
    
    OLED_WriteCommand(0xDA); // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    
    OLED_WriteCommand(0xDB); // 设置VCOMH
    OLED_WriteCommand(0x40);
    
    OLED_WriteCommand(0x8D); // 设置充电泵
    OLED_WriteCommand(0x14);
    
    OLED_WriteCommand(0xAF); // 开启显示
    
    // 清屏
    OLED_Clear();
}
