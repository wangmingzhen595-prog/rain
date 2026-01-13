# Git仓库初始化和上传说明

## 方法一：使用批处理文件（推荐）

直接双击 `git_push.bat` 文件，它会自动完成以下操作：
1. 初始化Git仓库（如果尚未初始化）
2. 添加远程仓库地址
3. 添加所有文件到暂存区
4. 提交更改
5. 推送到GitHub

## 方法二：手动执行命令

如果批处理文件无法正常工作，可以手动在命令行中执行以下命令：

```bash
# 1. 切换到项目目录
cd "D:\stm32\stm32 project\12.18\8-2 DMA+AD多通道"

# 2. 初始化git仓库
git init

# 3. 添加远程仓库
git remote add origin https://github.com/wangmingzhen595-prog/rain.git

# 4. 添加所有文件
git add .

# 5. 提交更改
git commit -m "初始提交：STM32雨滴传感器检测系统，DMA+ADC多通道采样，支持420-540mV小雨滴检测"

# 6. 重命名分支为main（如果需要）
git branch -M main

# 7. 推送到GitHub
git push -u origin main
```

## 注意事项

1. **首次使用Git需要配置用户信息**：
   ```bash
   git config --global user.name "你的名字"
   git config --global user.email "你的邮箱"
   ```

2. **如果远程仓库已有内容**，可能需要先拉取：
   ```bash
   git pull origin main --allow-unrelated-histories
   ```

3. **如果推送需要认证**，请使用GitHub Personal Access Token而不是密码。

4. **.gitignore文件已配置**，会自动排除编译生成的文件（Objects/、Listings/等）。

## 远程仓库地址

https://github.com/wangmingzhen595-prog/rain.git
