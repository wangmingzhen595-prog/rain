$outPath = "c:\Users\Administrator\Desktop\从控\2026.2.4\雨滴传感器软件设计方案_算法阶段报告.docx"

$word = New-Object -ComObject Word.Application
$word.Visible = $false
$doc = $word.Documents.Add()
$sel = $word.Selection

$wdPageBreak = 7
$wdAlignCenter = 1
$wdAlignLeft = 0

function Add-Line {
    param([string]$text)
    $sel.TypeText($text)
    $sel.TypeParagraph()
}

# Page 1: Cover
$sel.ParagraphFormat.Alignment = $wdAlignCenter
$sel.Font.Name = "宋体"
$sel.Font.Size = 26
Add-Line '雨滴传感器软件设计方案'
$sel.Font.Size = 16
Add-Line '算法分析与数据处理阶段报告'
Add-Line ''
$sel.Font.Size = 14
Add-Line '项目：雨滴传感器软件系统设计'
Add-Line '角色：软件负责人（算法协同说明）'
Add-Line '对象：算法分析与数据处理工程师'
Add-Line '日期：2026-04-21'
Add-Line ''
$sel.Font.Size = 12
Add-Line '说明：本文聚焦算法链路与阶段性问题，供算法同事针对性提出优化方案；系统通信、显示与主控软件仍由我方负责。'

$sel.InsertBreak($wdPageBreak)

# Page 2
$sel.ParagraphFormat.Alignment = $wdAlignLeft
$sel.Font.Name = "宋体"
$sel.Font.Size = 16
Add-Line '1. 当前算法总体设计（已落地实现）'
$sel.Font.Size = 12
Add-Line '1.1 背景'
Add-Line '早期方案采用峰值检测法。实测发现振动、飞溅、EMI毛刺场景下，误检和漏检较明显，大信号还存在高增益通道饱和问题。'
Add-Line '当前阶段切换为 时域有效性筛选 + 小波特征分类 + 雨量累计 的组合方案，并尝试引入积分冲量作为下一步核心指标。'
Add-Line ''
Add-Line '1.2 当前处理链路'
Add-Line '（1）触发与快照：DMA环形采样 + 模拟看门狗触发，保存预触发和后触发窗口。'
Add-Line '（2）前部分析：仅分析上升-峰值-回落到基线的前半段，减少拖尾振铃干扰。'
Add-Line '（3）时域判定：幅值、脉宽、上升下降时间、平滑度、峰稳定性。'
Add-Line '（4）小波特征：64点Haar DWT三层分解，计算hf_ratio。'
Add-Line '（5）事件分类：RAIN_CLEAN / RAIN_FAST / VIB / EMI / BG / BAD。'
Add-Line '（6）雨量累计：电压到体积换算（LUT+插值）后累加。'
Add-Line ''
Add-Line '1.3 关键代码节选（注释版）'
$sel.Font.Name = "Consolas"
$sel.Font.Size = 10
Add-Line '/* 小波特征提取：64点窗口 + 三层Haar DWT */'
Add-Line 'uint16_t win_start = (front_peak_index >= 16) ? (front_peak_index - 16) : 0;'
Add-Line 'if (Wavelet_ExtractFeatures(active_buffer, end_index + 1, win_start,'
Add-Line '                            wavelet_baseline, &wavelet_features)) {'
Add-Line '    // 提取成功后，使用 hf_ratio 进行事件类型判定'
Add-Line '    event_type = Determine_Event_Type(&wavelet_features);'
Add-Line '}'
Add-Line ''
Add-Line '/* 高频比例阈值分段（经验参数，待标定） */'
Add-Line 'if (hf_ratio >= 600) return EVT_EMI;       // 电噪声毛刺'
Add-Line 'if (hf_ratio >= 450) return EVT_VIB;       // 振动或风扰'
Add-Line 'if (hf_ratio >= 250) return EVT_RAIN_FAST; // 高速小滴'
Add-Line 'if (hf_ratio >= 150) return EVT_RAIN_CLEAN;// 典型雨滴'
Add-Line 'return EVT_BG;                              // 背景漂移'

$sel.InsertBreak($wdPageBreak)

# Page 3
$sel.Font.Name = "宋体"
$sel.Font.Size = 16
Add-Line '2. 阶段性结果：优势、劣势与建议'
$sel.Font.Size = 12
Add-Line '2.1 优势'
Add-Line '- 相比纯峰值法，小波分类后对EMI与振动的可解释性明显提升。'
Add-Line '- 前部分析降低拖尾重复计数。'
Add-Line '- 动态死区按类型分级，对连发干扰更稳。'
Add-Line '- 双通道切换提升了动态范围。'
Add-Line ''
Add-Line '2.2 劣势'
Add-Line '- 小波阈值主要为经验值，跨环境泛化能力待验证。'
Add-Line '- 当前雨量累计仍偏峰值驱动，冲量积分尚未成为主判据。'
Add-Line '- 参数较多且耦合较高，维护成本偏高。'
Add-Line ''
Add-Line '2.3 给算法工程师的重点任务'
Add-Line 'P1：构建冲量主导的体积估计模型，替代单峰值主导。'
Add-Line 'P2：建立标定流程，输出hf_ratio阈值和死区策略。'
Add-Line 'P3：给出误检率、漏检率、RMSE等统一评估框架。'
Add-Line ''
Add-Line '2.4 小波 + 积分冲量建议方案（关键代码）'
$sel.Font.Name = "Consolas"
$sel.Font.Size = 10
Add-Line '/* 冲量积分：只积正脉冲，避免基线抵消 */'
Add-Line 'float Compute_Impulse(const uint16_t *buf, uint16_t s, uint16_t e,'
Add-Line '                     int32_t baseline, float Ts){'
Add-Line '    float I = 0.0f;'
Add-Line '    for(uint16_t i = s; i <= e; i++){'
Add-Line '        int32_t d = (int32_t)buf[i] - baseline;'
Add-Line '        if(d > 0) I += (float)d * Ts;'
Add-Line '    }'
Add-Line '    return I;'
Add-Line '}'
$sel.Font.Name = "宋体"
$sel.Font.Size = 12
Add-Line ''
Add-Line '结论：当前小波方案已经明显优于纯峰值方案。下一阶段建议以冲量积分为主量化指标，并完成小波阈值标定与冲量-体积映射。'

$doc.SaveAs([ref]$outPath)
$doc.Close()
$word.Quit()

[System.Runtime.InteropServices.Marshal]::ReleaseComObject($sel) | Out-Null
[System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null
[System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null

Write-Output "SAVED: $outPath"
