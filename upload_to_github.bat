@echo off
chcp 65001 >nul
echo ========================================
echo GitHub上传脚本
echo ========================================
echo.

REM 检查git是否安装
git --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未检测到git命令！
    echo 请先安装Git for Windows: https://git-scm.com/download/win
    echo 或者使用GitHub Desktop: https://desktop.github.com/
    pause
    exit /b 1
)

echo [1/7] 初始化git仓库...
git init
if errorlevel 1 (
    echo [错误] git init 失败
    pause
    exit /b 1
)

echo [2/7] 添加所有文件...
git add .
if errorlevel 1 (
    echo [错误] git add 失败
    pause
    exit /b 1
)

echo [3/7] 提交更改...
git commit -m "初始提交：STM32雨滴传感器检测系统，已修复电压显示跳变问题"
if errorlevel 1 (
    echo [警告] git commit 失败，可能是没有更改或已提交
)

echo [4/7] 重命名分支为main...
git branch -M main
if errorlevel 1 (
    echo [警告] git branch 失败，可能分支已经是main
)

echo [5/7] 添加远程仓库...
git remote remove origin 2>nul
git remote add origin https://github.com/wangmingzhen595-prog/-.git
if errorlevel 1 (
    echo [错误] git remote add 失败
    pause
    exit /b 1
)

echo [6/7] 检查远程仓库连接...
git remote -v

echo [7/7] 推送到GitHub...
echo 注意：如果这是第一次推送，可能需要输入GitHub用户名和密码（或Personal Access Token）
echo.
git push -u origin main
if errorlevel 1 (
    echo.
    echo [错误] git push 失败
    echo 可能的原因：
    echo 1. 需要配置git用户信息：
    echo    git config --global user.name "你的名字"
    echo    git config --global user.email "你的邮箱"
    echo 2. 需要GitHub认证（使用Personal Access Token而不是密码）
    echo 3. 远程仓库可能已存在内容，需要先pull
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo 上传成功！
echo ========================================
pause

