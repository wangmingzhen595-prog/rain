#include "stm32f10x.h"

/**
  * @brief  延时初始化
  * @param  无
  * @retval 无
  */
void Delay_Init(void)
{
    // 使用外部晶振时系统时钟为72MHz
    // 选择HCLK/8作为SysTick时钟源
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
    uint32_t temp;
    SysTick->LOAD = 9 * xus;              // 设置定时器重装值，72MHz/8=9MHz
    SysTick->VAL = 0x00;                  // 清空当前计数值
    SysTick->CTRL = 0x01;                 // 使能，减到零是无动作，采用外部时钟源
    
    do {

        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到达
    
    SysTick->CTRL = 0x00;                 // 关闭计数器
    SysTick->VAL = 0x00;                  // 清空计数器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    while(xms--)
    {
        Delay_us(1000);
    }
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    while(xs--)
    {
        Delay_ms(1000);
    }
}
