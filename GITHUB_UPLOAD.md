# GitHub上传指南

由于系统未检测到git命令，请按照以下步骤手动上传代码到GitHub：

## 方法1：使用GitHub Desktop（推荐）

1. 下载并安装 [GitHub Desktop](https://desktop.github.com/)
2. 打开GitHub Desktop，登录你的GitHub账号
3. 点击 "File" -> "Add Local Repository"
4. 选择项目目录：`D:\stm32\stm32 project\12.2\8-2 DMA+AD多通道`
5. 如果提示"这不是一个git仓库"，点击"create a repository"
6. 在提交信息中输入：`初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题`
7. 点击"Commit to main"
8. 点击"Publish repository"，选择远程仓库：`https://github.com/wangmingzhen595-prog/-.git`
9. 点击"Publish repository"完成上传

## 方法2：使用命令行（如果已安装git）

```bash
# 进入项目目录
cd "D:\stm32\stm32 project\12.2\8-2 DMA+AD多通道"

# 初始化git仓库（如果还没有）
git init

# 添加远程仓库
git remote add origin https://github.com/wangmingzhen595-prog/-.git

# 添加所有文件
git add .

# 提交
git commit -m "初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题"

# 推送到GitHub
git push -u origin main
```

## 方法3：使用GitHub网页上传

1. 访问 https://github.com/wangmingzhen595-prog/-
2. 点击 "uploading an existing file"
3. 拖拽项目文件夹中的所有源代码文件（排除编译产物）
4. 在提交信息中输入：`初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题`
5. 点击 "Commit changes"

## 需要上传的文件

### 必须上传：
- `User/` 目录下的所有.c和.h文件
- `Hardware/` 目录下的所有.c和.h文件
- `System/` 目录下的所有.c和.h文件
- `Library/` 目录下的所有.c和.h文件（或只上传使用的）
- `Start/` 目录下的所有文件
- `README.md`
- `.gitignore`
- `Project.uvprojx`（Keil项目文件）

### 不需要上传：
- `Objects/` 目录（编译产物）
- `Listings/` 目录（编译产物）
- `DebugConfig/` 目录
- `*.uvguix*` 文件
- `*.uvoptx` 文件
- `*.bak` 文件

