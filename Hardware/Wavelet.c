#include "Wavelet.h"

/**
  * @brief  Haar DWT单层分解（整数实现）
  * @param  input: 输入信号（int16_t数组）
  * @param  input_len: 输入长度（必须是偶数）
  * @param  approx: 输出近似系数
  * @param  detail: 输出细节系数
  * @retval 无
  */
static void Haar_DWT_Decompose(int16_t *input, uint16_t input_len,
                               int16_t *approx, int16_t *detail)
{
    uint16_t output_len = input_len >> 1;  // 输出长度是输入的一半
    
    for (uint16_t i = 0; i < output_len; i++)
    {
        // 近似系数：avg = (x[2i] + x[2i+1]) / 2
        approx[i] = (int16_t)((input[2*i] + input[2*i+1]) >> 1);
        
        // 细节系数：diff = (x[2i] - x[2i+1]) / 2
        detail[i] = (int16_t)((input[2*i] - input[2*i+1]) >> 1);
    }
}

/**
  * @brief  提取小波特征（收敛接口）
  * @param  signal: 信号缓冲区
  * @param  signal_len: 信号长度
  * @param  win_start: 窗口起点（64点窗口的起始索引）
  * @param  baseline: baseline值（用于去直流）
  * @param  features: 输出特征结构体
  * @retval 1: 成功，0: 失败
  */
uint8_t Wavelet_ExtractFeatures(uint16_t *signal, uint16_t signal_len,
                                uint16_t win_start, int32_t baseline,
                                WaveletFeatures_t *features)
{
    // 静态缓冲区（避免动态分配）
    static int16_t x64[WAVELET_WINDOW_SIZE];           // 64点输入（去直流后）
    static int16_t approx1[32], detail1[32];          // 第1层：32点
    static int16_t approx2[16], detail2[16];          // 第2层：16点
    static int16_t approx3[8], detail3[8];             // 第3层：8点
    
    // 初始化输出
    features->energy_high = 0;
    features->hf_ratio = 0;
    features->success = 0;
    
    // 检查窗口起点有效性
    if (win_start >= signal_len)
    {
        return 0;  // 窗口起点越界
    }
    
    // 1. 准备64点输入（去直流 + 有符号转换，只做一次）
    uint16_t valid_samples = 0;
    for (uint16_t i = 0; i < WAVELET_WINDOW_SIZE; i++)
    {
        uint16_t idx = win_start + i;
        
        if (idx < signal_len)
        {
            // 去直流与有符号转换只做一次
            int32_t diff = (int32_t)signal[idx] - baseline;
            // 限制到int16_t范围（防止溢出）
            if (diff > 32767) diff = 32767;
            if (diff < -32768) diff = -32768;
            x64[i] = (int16_t)diff;
            valid_samples = i + 1;
        }
        else
        {
            // 不足64点：使用末值延拓（重复最后一个有效样本）
            if (valid_samples > 0)
            {
                x64[i] = x64[valid_samples - 1];
            }
            else
            {
                // 如果完全没有有效样本，使用baseline延拓
                x64[i] = 0;  // baseline已减去，所以是0
            }
        }
    }
    
    // 2. 第1层DWT分解：64点 → 32点近似 + 32点细节
    Haar_DWT_Decompose(x64, 64, approx1, detail1);
    
    // 3. 第2层DWT分解：32点近似 → 16点近似 + 16点细节
    Haar_DWT_Decompose(approx1, 32, approx2, detail2);
    
    // 4. 第3层DWT分解：16点近似 → 8点近似 + 8点细节
    Haar_DWT_Decompose(approx2, 16, approx3, detail3);
    
    // 5. 计算能量（使用int32_t做乘法，累加到uint32_t）
    uint32_t energy_high = 0;
    uint32_t energy_total = 0;
    
    // 高频能量：Ed1 + Ed2 + Ed3
    for (uint16_t i = 0; i < 32; i++)
    {
        int32_t d1 = (int32_t)detail1[i];
        energy_high += (uint32_t)(d1 * d1);
    }
    for (uint16_t i = 0; i < 16; i++)
    {
        int32_t d2 = (int32_t)detail2[i];
        energy_high += (uint32_t)(d2 * d2);
    }
    for (uint16_t i = 0; i < 8; i++)
    {
        int32_t d3 = (int32_t)detail3[i];
        energy_high += (uint32_t)(d3 * d3);
    }
    
    // 总能量：energy_high + 第3层近似能量
    energy_total = energy_high;
    for (uint16_t i = 0; i < 8; i++)
    {
        int32_t a3 = (int32_t)approx3[i];
        energy_total += (uint32_t)(a3 * a3);
    }
    
    // 6. 计算高频比例（溢出保护）
    if (energy_total == 0)
    {
        // 极端情况：总能量为0
        features->hf_ratio = 0;
        features->success = 0;
        return 0;
    }
    
    // hf_ratio = (energy_high * 1000) / energy_total（permille，0-1000）
    // 使用64位中间结果避免溢出
    uint64_t ratio_temp = ((uint64_t)energy_high * 1000ULL) / energy_total;
    if (ratio_temp > 1000)
    {
        ratio_temp = 1000;  // 限制在1000以内
    }
    features->hf_ratio = (uint16_t)ratio_temp;
    
    // 7. 设置输出
    features->energy_high = energy_high;
    features->success = 1;
    
    return 1;
}

/**
  * @brief  提取小波特征（兼容接口，保留end_index参数但不使用）
  * @param  signal: 信号缓冲区
  * @param  signal_len: 信号长度
  * @param  start_index: 窗口起点（作为win_start使用）
  * @param  end_index: 结束索引（保留但不使用，为了兼容主流程）
  * @param  baseline: baseline值（用于去直流）
  * @param  features: 输出特征结构体
  * @retval 1: 成功，0: 失败
  */
uint8_t Wavelet_ExtractFeatures_Compat(uint16_t *signal, uint16_t signal_len,
                                      uint16_t start_index, uint16_t end_index,
                                      int32_t baseline, WaveletFeatures_t *features)
{
    // end_index参数保留但不使用，明确start_index就是win_start
    (void)end_index;  // 避免未使用变量警告
    
    return Wavelet_ExtractFeatures(signal, signal_len, start_index, baseline, features);
}
