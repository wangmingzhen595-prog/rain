# GitHub网页上传详细步骤

由于系统未安装git命令行工具，可以通过GitHub网页界面上传代码。

## 方法1：通过GitHub网页直接上传（最简单）

### 步骤1：准备文件
确保以下文件已准备好：
- ✅ `User/` 目录下的所有源代码文件
- ✅ `Hardware/` 目录下的所有源代码文件  
- ✅ `System/` 目录下的所有源代码文件
- ✅ `Library/` 目录下的相关文件
- ✅ `Start/` 目录下的文件
- ✅ `README.md`
- ✅ `.gitignore`
- ✅ `Project.uvprojx`

### 步骤2：访问GitHub仓库
1. 打开浏览器，访问：https://github.com/wangmingzhen595-prog/-
2. 如果仓库是空的，会看到 "Quick setup" 页面
3. 如果仓库已有内容，点击右上角的 "Add file" -> "Upload files"

### 步骤3：上传文件
1. **如果仓库是空的**：
   - 点击 "uploading an existing file" 链接
   - 或者直接拖拽文件夹到页面

2. **如果仓库已有内容**：
   - 点击 "Add file" -> "Upload files"
   - 拖拽文件或点击 "choose your files" 选择文件

3. **上传文件时**：
   - 可以一次选择多个文件
   - 支持拖拽整个文件夹
   - 建议按目录结构上传：
     - 先上传 `User/` 目录下的文件
     - 再上传 `Hardware/` 目录下的文件
     - 依次上传其他目录

### 步骤4：提交更改
1. 在页面底部的 "Commit changes" 区域：
   - **Commit message**（提交信息）：
     ```
     初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题
     ```
   - **Description**（可选描述）：
     ```
     基于STM32F103C8的雨滴传感器检测系统
     - 使用DMA+ADC连续采样
     - 实现自适应阈值和峰值检测
     - 已修复电压显示跳变问题，添加峰值保持机制
     ```

2. 选择 "Commit directly to the main branch"

3. 点击 "Commit changes" 按钮

### 步骤5：验证上传
上传完成后，刷新页面，应该能看到所有文件已上传成功。

---

## 方法2：使用GitHub Desktop（推荐，功能更强大）

### 步骤1：下载安装
1. 访问：https://desktop.github.com/
2. 下载 GitHub Desktop for Windows
3. 安装并启动

### 步骤2：登录GitHub
1. 打开 GitHub Desktop
2. 点击 "Sign in to GitHub.com"
3. 使用浏览器登录你的GitHub账号
4. 授权 GitHub Desktop 访问

### 步骤3：添加本地仓库
1. 点击 "File" -> "Add Local Repository"
2. 点击 "Choose..." 按钮
3. 选择项目目录：`D:\stm32\stm32 project\12.2\8-2 DMA+AD多通道`
4. 如果提示 "This directory does not appear to be a Git repository"
   - 勾选 "Create a repository"
   - Repository name: `-`（或你想要的名称）
   - Description: `STM32雨滴传感器检测系统`
   - 勾选 "Initialize this repository with a README"（可选）

### 步骤4：提交并推送
1. 在左侧会看到所有更改的文件
2. 在左下角输入提交信息：
   ```
   初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题
   ```
3. 点击 "Commit to main" 按钮
4. 点击 "Publish repository" 按钮
5. 选择远程仓库：`https://github.com/wangmingzhen595-prog/-.git`
6. 点击 "Publish repository" 完成上传

---

## 方法3：安装Git后使用命令行

### 步骤1：安装Git
1. 访问：https://git-scm.com/download/win
2. 下载 Git for Windows
3. 安装（使用默认选项即可）

### 步骤2：配置Git
打开 PowerShell 或 CMD，执行：
```bash
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub邮箱"
```

### 步骤3：执行上传命令
在项目目录下执行：
```bash
git init
git add .
git commit -m "初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题"
git branch -M main
git remote add origin https://github.com/wangmingzhen595-prog/-.git
git push -u origin main
```

---

## 需要上传的文件清单

### 必须上传的核心文件：

**User目录：**
- `main.c`
- `stm32f10x_it.c`
- `stm32f10x_it.h`
- `stm32f10x_conf.h`

**Hardware目录：**
- `AD.c` / `AD.h`
- `OLED.c` / `OLED.h`
- `OLED_Font.h`
- `Key.c` / `Key.h`（如果使用）
- `LED.c` / `LED.h`（如果使用）

**System目录：**
- `Delay.c` / `Delay.h`

**Start目录：**
- `core_cm3.c` / `core_cm3.h`
- `system_stm32f10x.c` / `system_stm32f10x.h`
- `stm32f10x.h`
- `startup_stm32f10x_md.s`（根据你的芯片选择）

**Library目录：**
- 只上传你实际使用的库文件（通常包括：`stm32f10x_rcc.c/h`, `stm32f10x_gpio.c/h`, `stm32f10x_adc.c/h`, `stm32f10x_dma.c/h` 等）

**项目文件：**
- `Project.uvprojx`
- `README.md`
- `.gitignore`

### 不需要上传的文件：
- ❌ `Objects/` 目录（编译产物）
- ❌ `Listings/` 目录（编译产物）
- ❌ `DebugConfig/` 目录
- ❌ `*.uvguix*` 文件
- ❌ `*.uvoptx` 文件
- ❌ `*.bak` 文件
- ❌ `*.o`, `*.d`, `*.crf` 等编译中间文件

---

## 快速上传建议

**最快方式**：使用方法1（网页直接上传）
1. 访问 https://github.com/wangmingzhen595-prog/-
2. 点击 "uploading an existing file"
3. 拖拽整个项目文件夹（`.gitignore` 会自动排除不需要的文件）
4. 输入提交信息并提交

**最推荐方式**：使用方法2（GitHub Desktop）
- 功能更强大，后续更新更方便
- 可以查看文件差异
- 支持分支管理

---

## 常见问题

**Q: 上传后看不到文件？**
A: 刷新页面，或检查文件是否真的上传成功。

**Q: 文件太多，上传很慢？**
A: 可以先上传核心源代码文件，库文件可以后续添加。

**Q: 如何更新代码？**
A: 
- 网页方式：再次点击 "Upload files" 上传新文件
- GitHub Desktop：修改文件后会自动检测，点击 "Commit" 和 "Push" 即可

**Q: 如何删除已上传的文件？**
A: 在GitHub网页上，点击文件，然后点击垃圾桶图标删除。

