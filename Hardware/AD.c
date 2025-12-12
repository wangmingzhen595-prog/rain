// 引入STM32F10x系列微控制器的头文件，包含所有寄存器定义和库函数
#include "stm32f10x.h"
#include "AD.h"

// 定义存储ADC转换结果的数组，大小为100个16位无符号整数
// 单通道模式下，AD_Value[n] 全部为 PA0（通道0）的连续采样结果
// 数组大小为100，表示100个单通道数据点
uint16_t AD_Value[100];

/* 扩展：环形缓冲与快照（原双通道模式，当前仅使用通道0） */
/* 双通道环形缓冲区：目前仅通道0实际参与处理，通道1保留以便后续扩展 */
volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];  // 通道0环形缓冲区
volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];  // 通道1环形缓冲区
volatile uint16_t ring_write_index_ch0 = 0;  // 通道0写索引
volatile uint16_t ring_write_index_ch1 = 0;  // 通道1写索引

/* Keil Array Visualization 可观察数组（500个元素） */
/* 用于在Keil调试器中观察ADC采样数据，确保数组在文件作用域声明为全局变量 */
/* 使用说明：在Keil调试器的Watch窗口中，添加 ADC_Visualize_Buffer 变量，
   然后右键选择 Array Visualization 即可查看波形 */
volatile uint16_t ADC_Visualize_Buffer[500];  // ADC可视化缓冲区（通道0数据）
volatile uint16_t ADC_Visualize_Index = 0;     // 可视化缓冲区写索引


volatile uint8_t snapshot_ready = 0;
volatile uint16_t snapshot_buffer_high[SNAPSHOT_SIZE];
volatile uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];
volatile uint16_t snapshot_write_index = 0;
volatile uint8_t snapshot_collecting = 0; /* 0-未采集，1-正在采集 */
volatile uint16_t snapshot_peak_value = 0;
volatile uint16_t snapshot_peak_index = 0;

volatile uint32_t sampling_tick_counter = 0; /* 由HT/TC中断推进，用于异常检测 */

volatile uint16_t ADC_Threshold = 620;  // 500mV

static void AD_ConfigAWD(uint16_t threshold);

void AD_SetThreshold(uint16_t threshold)
{
    ADC_Threshold = threshold;
    AD_ConfigAWD(ADC_Threshold);
}

void AD_Restart(void)
{
    /* 复位DMA与ADC，重新开始 */
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, 100);
    DMA_ClearITPendingBit(DMA1_IT_TC1 | DMA1_IT_HT1 | DMA1_IT_TE1 | DMA1_IT_GL1);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  AD初始化函数
  * @param  无
  * @retval 无
  */
void AD_Init(void)
{
    // 定义GPIO初始化结构体变量
    GPIO_InitTypeDef GPIO_InitStructure;
    // 定义ADC初始化结构体变量
    ADC_InitTypeDef ADC_InitStructure;
    // 定义DMA初始化结构体变量
    DMA_InitTypeDef DMA_InitStructure;
    // 定义NVIC中断控制器初始化结构体变量
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 开启GPIOA的时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 开启ADC1的时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    // 开启DMA1的时钟（AHB总线）
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    // 配置ADC时钟分频，PCLK2为72MHz，6分频后ADC时钟为12MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 配置PA0和PA1引脚为模拟输入模式
    // 当前只使用PA0接雨滴传感器，PA1保持为模拟输入以免悬空
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;           // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 初始化GPIOA
    
    // 配置ADC规则组通道：单通道模式（仅PA0）
    // 通道0（PA0）：采样顺序1、采样时间239.5个周期（最大采样时间，适用于高阻抗信号源）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    
    // ADC初始化配置（单通道模式）
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;              // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                   // 关闭扫描，仅单通道
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;              // 连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;          // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                         // 1个转换通道（PA0）
    ADC_Init(ADC1, &ADC_InitStructure);                             // 初始化ADC1
    
    // 复位DMA1通道1配置
    DMA_DeInit(DMA1_Channel1);
    // DMA初始化配置（单通道模式）
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // 外设地址：ADC数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;      // 内存地址：数据存储数组
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              // 传输方向：外设到内存
    DMA_InitStructure.DMA_BufferSize = 100;                         // 缓冲区大小：100个数据（100个PA0样本）
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 外设数据宽度16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 内存数据宽度16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                 // 循环模式
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                    // 禁用内存到内存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;             // 高优先级
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);                    // 初始化DMA1通道1
    
    // 使能DMA半传输与传输完成中断
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
    
    // 配置NVIC中断控制器
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;        // DMA1通道1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       // 抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              // 子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC

    /* 配置ADC模拟看门狗用于阈值触发，减少CPU扫描负担 */
    AD_ConfigAWD(ADC_Threshold);
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能DMA1通道1
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    // 使能ADC1的DMA传输
    ADC_DMACmd(ADC1, ENABLE);
    
    // 使能ADC1
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC校准流程
    ADC_ResetCalibration(ADC1);                             // 复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));            // 等待复位校准完成
    ADC_StartCalibration(ADC1);                            // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));                 // 等待校准完成
    
    // 使能ADC1软件触发转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

static void AD_ConfigAWD(uint16_t threshold)
{
    /* 将通道0加入模拟看门狗监控，仅规则通道（双通道模式下监控通道0） */
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_0);
    ADC_AnalogWatchdogThresholdsConfig(ADC1, threshold, 0);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
    /* 注意：双通道模式下，模拟看门狗仅监控通道0，通道1的阈值检测在软件中处理 */
}
