@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"
title Git自动推送脚本

echo.
echo ========================================
echo Git自动初始化和推送脚本
echo ========================================
echo.
echo 正在执行Git操作，请稍候...
echo.

REM 步骤1: 初始化Git仓库
if not exist .git (
    echo [步骤1/5] 初始化Git仓库...
    git init
    if errorlevel 1 (
        echo [错误] Git未安装或未添加到PATH
        echo 请安装Git: https://git-scm.com/download/win
        pause
        exit /b 1
    )
    echo [完成] Git仓库已初始化
) else (
    echo [步骤1/5] Git仓库已存在，跳过初始化
)
echo.

REM 步骤2: 配置远程仓库
echo [步骤2/5] 配置远程仓库...
git remote remove origin 2>nul
git remote add origin https://github.com/wangmingzhen595-prog/rain.git
if errorlevel 1 (
    echo [警告] 远程仓库配置可能失败，继续执行...
) else (
    echo [完成] 远程仓库已配置: https://github.com/wangmingzhen595-prog/rain.git
)
echo.

REM 步骤3: 添加文件
echo [步骤3/5] 添加所有文件到暂存区...
git add .
if errorlevel 1 (
    echo [错误] 添加文件失败
    pause
    exit /b 1
)
echo [完成] 文件已添加到暂存区
echo.

REM 步骤4: 提交更改
echo [步骤4/5] 提交更改...
git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力" 2>nul
if errorlevel 1 (
    echo [提示] 没有需要提交的更改，或提交失败（可能已提交过）
) else (
    echo [完成] 更改已提交
)
echo.

REM 步骤5: 推送
echo [步骤5/5] 推送到GitHub...
echo 注意：如果提示输入用户名和密码，请使用Personal Access Token
echo.
git branch -M main 2>nul
git push -u origin main
if errorlevel 1 (
    echo.
    echo ========================================
    echo [提示] 推送可能需要认证
    echo ========================================
    echo.
    echo 如果推送失败，请按以下步骤操作：
    echo.
    echo 1. 获取Personal Access Token:
    echo    - 登录GitHub
    echo    - Settings -^> Developer settings -^> Personal access tokens -^> Tokens (classic)
    echo    - Generate new token (classic)
    echo    - 勾选 "repo" 权限
    echo    - 复制生成的token
    echo.
    echo 2. 重新运行此脚本，当提示输入密码时：
    echo    - 用户名：你的GitHub用户名
    echo    - 密码：粘贴刚才复制的token（不是GitHub密码）
    echo.
    echo 或者手动执行：
    echo    git push -u origin main
    echo.
) else (
    echo.
    echo ========================================
    echo [成功] 代码已推送到GitHub！
    echo ========================================
    echo.
    echo 仓库地址: https://github.com/wangmingzhen595-prog/rain.git
    echo.
)

pause

