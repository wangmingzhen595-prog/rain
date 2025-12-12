-- 雨滴脉冲波形接收脚本 - AtopSerial接收预处理脚本
-- 数据格式：0xAA 0x55 [len_low] [len_high] [数据...]
-- 每个数据是uint16小端，共500个样本

-- 初始化状态变量（仅在第一次运行时执行）
if not raindrop_state then
    raindrop_state = {
        buffer = {},
        state = "HEADER",  -- HEADER, HEADER2, LEN_LOW, LEN_HIGH, DATA
        header_count = 0,
        data_len = 0,
        data_received = 0,
        current_packet = {},
        bytes_buffer = ""  -- 字节缓冲区
    }
end

local state = raindrop_state

-- 接收预处理脚本会接收数据作为参数（通常是字节串或表）
-- 如果函数有参数，使用参数；否则尝试从全局变量获取
local function process_received_data(data)
    if not data then
        return ""
    end
    
    -- 将数据转换为字节数组
    local bytes = {}
    if type(data) == "string" then
        -- 如果是字符串，转换为字节数组
        for i = 1, #data do
            bytes[i] = string.byte(data, i)
        end
    elseif type(data) == "table" then
        -- 如果已经是表，直接使用
        bytes = data
    else
        return ""
    end
    
    -- 处理每个字节
    for i = 1, #bytes do
        local byte = bytes[i]
        
        if state.state == "HEADER" then
            if byte == 0xAA then
                state.header_count = 1
                state.state = "HEADER2"
            end
        elseif state.state == "HEADER2" then
            if byte == 0x55 then
                state.state = "LEN_LOW"
                state.data_received = 0
                state.current_packet = {}
                state.bytes_buffer = ""
            else
                state.state = "HEADER"
                state.header_count = 0
            end
        elseif state.state == "LEN_LOW" then
            state.data_len = byte
            state.state = "LEN_HIGH"
        elseif state.state == "LEN_HIGH" then
            state.data_len = state.data_len + (byte * 256)  -- 小端格式
            if state.data_len > 0 and state.data_len <= 2000 then  -- 安全限制
                state.state = "DATA"
                state.data_received = 0
                state.current_packet = {}
            else
                -- 长度异常，重置
                state.state = "HEADER"
                state.header_count = 0
                state.data_len = 0
            end
        elseif state.state == "DATA" then
            table.insert(state.current_packet, byte)
            state.data_received = state.data_received + 1
            
            -- 检查是否接收完成（len * 2字节，因为每个样本是uint16）
            if state.data_received >= (state.data_len * 2) then
                -- 解析uint16数组（小端）
                local samples = {}
                for j = 1, state.data_len do
                    local idx = (j - 1) * 2 + 1
                    if idx + 1 <= #state.current_packet then
                        local low = state.current_packet[idx]
                        local high = state.current_packet[idx + 1]
                        local value = low + (high * 256)  -- 小端转换
                        samples[j] = value
                    end
                end
                
                -- 将数据存储到全局变量，供绘图使用
                _G.raindrop_samples = samples
                _G.raindrop_ready = true
                _G.raindrop_sample_count = state.data_len
                
                -- 计算峰值用于日志输出
                local max_val = 0
                local max_idx = 1
                for k = 1, #samples do
                    if samples[k] > max_val then
                        max_val = samples[k]
                        max_idx = k
                    end
                end
                
                print(string.format("✓ 接收到雨滴波形：%d个采样点，峰值=%d @索引%d", 
                      state.data_len, max_val, max_idx))
                
                -- 重置状态
                state.state = "HEADER"
                state.header_count = 0
                state.data_len = 0
                state.data_received = 0
                state.current_packet = {}
                state.bytes_buffer = ""
                
                -- 返回空字符串表示数据处理完成（不转发原始数据）
                return ""
            end
        end
    end
    
    -- 如果包未完成，返回空字符串（继续等待）
    return ""
end

-- 尝试多种方式获取接收数据
-- 方式1：函数参数（接收预处理脚本的标准方式）
-- 方式2：全局变量
-- 方式3：串口对象方法

local received_data = nil

-- 方式1：检查函数参数（如果脚本作为接收预处理脚本被调用）
if arg and arg[1] then
    received_data = arg[1]
end

-- 方式2：检查全局接收缓冲区
if not received_data and _G.SerialData then
    received_data = _G.SerialData
end

-- 方式3：尝试通过串口对象获取（但使用不同的方法名）
if not received_data then
    -- 尝试常见的串口对象名称
    local serial_names = {"uart", "UART", "serial", "Serial", "port", "Port"}
    for _, name in ipairs(serial_names) do
        local obj = _G[name]
        if obj and type(obj) == "table" then
            -- 尝试不同的方法名
            if obj.Read then
                received_data = obj:Read()
                break
            elseif obj.GetData then
                received_data = obj:GetData()
                break
            elseif obj.Receive then
                received_data = obj:Receive()
                break
            elseif obj.GetBuffer then
                received_data = obj:GetBuffer()
                break
            end
        end
    end
end

-- 方式4：如果没有参数，尝试从字符串参数获取
if not received_data and ... then
    local args = {...}
    if #args > 0 then
        received_data = args[1]
    end
end

-- 处理接收到的数据
if received_data then
    return process_received_data(received_data)
end

-- 如果没有数据，返回空字符串
return ""

