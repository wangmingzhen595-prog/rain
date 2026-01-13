@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo 初始化Git并推送到GitHub
echo ========================================

REM 检查git是否安装
git --version >nul 2>&1
if errorlevel 1 (
    echo 错误：未检测到Git，请先安装Git
    echo 下载地址：https://git-scm.com/download/win
    pause
    exit /b 1
)

REM 检查git用户配置
git config user.name >nul 2>&1
if errorlevel 1 (
    echo 警告：未配置Git用户信息
    echo 请先配置：
    echo   git config --global user.name "你的名字"
    echo   git config --global user.email "你的邮箱"
    echo.
    echo 或者为当前仓库配置：
    echo   git config user.name "你的名字"
    echo   git config user.email "你的邮箱"
    echo.
    pause
)

REM 检查是否已初始化git
if not exist .git (
    echo 初始化Git仓库...
    git init
    echo Git仓库初始化完成
) else (
    echo Git仓库已存在
)

REM 检查远程仓库
git remote -v | findstr "rain.git" >nul
if errorlevel 1 (
    echo 添加远程仓库...
    git remote add origin https://github.com/wangmingzhen595-prog/rain.git
    echo 远程仓库添加完成
) else (
    echo 远程仓库已存在，更新URL...
    git remote set-url origin https://github.com/wangmingzhen595-prog/rain.git
)

REM 添加所有文件
echo 添加文件到暂存区...
git add .

REM 检查分支名称并重命名为main
git branch --show-current >nul 2>&1
if errorlevel 1 (
    echo 创建main分支...
    git checkout -b main
) else (
    echo 重命名分支为main...
    git branch -M main 2>nul
)

REM 提交更改
echo 提交更改...
git commit -m "初始提交：STM32雨滴传感器检测系统，DMA+ADC多通道采样，支持420-540mV小雨滴检测"
if errorlevel 1 (
    echo 提示：没有新更改需要提交，或提交失败
)

REM 推送到GitHub
echo 推送到GitHub...
git push -u origin main
if errorlevel 1 (
    echo 尝试强制推送（如果远程仓库已有内容）...
    echo 如果失败，请先执行: git pull origin main --allow-unrelated-histories
)

echo ========================================
echo 完成！
echo ========================================
pause

