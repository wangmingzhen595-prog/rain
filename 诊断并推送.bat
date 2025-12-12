@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo ========================================
echo Git诊断和推送脚本
echo ========================================
echo.

REM 检查Git
echo [检查1] Git安装状态...
git --version
if errorlevel 1 (
    echo [X] Git未安装或未添加到PATH
    echo 请安装Git: https://git-scm.com/download/win
    pause
    exit /b 1
)
echo [OK] Git已安装
echo.

REM 检查Git配置
echo [检查2] Git用户配置...
git config --global user.name
if errorlevel 1 (
    echo [X] 未配置用户名
    echo 请运行: git config --global user.name "YourName"
) else (
    echo [OK] 用户名已配置
)
git config --global user.email
if errorlevel 1 (
    echo [X] 未配置邮箱
    echo 请运行: git config --global user.email "YourEmail"
) else (
    echo [OK] 邮箱已配置
)
echo.

REM 初始化仓库
echo [操作1] 初始化Git仓库...
if not exist .git (
    git init
    echo [OK] Git仓库已初始化
) else (
    echo [OK] Git仓库已存在
)
echo.

REM 配置远程
echo [操作2] 配置远程仓库...
git remote remove origin 2>nul
git remote add origin https://github.com/wangmingzhen595-prog/rain.git
echo [OK] 远程仓库已配置
echo.

REM 添加文件
echo [操作3] 添加文件...
git add .
echo [OK] 文件已添加
echo.

REM 提交
echo [操作4] 提交更改...
git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力" 2>&1
echo.

REM 推送
echo [操作5] 推送到GitHub...
echo 注意：如果提示输入用户名和密码，请使用Personal Access Token
echo.
git branch -M main 2>nul
git push -u origin main 2>&1

echo.
echo ========================================
echo 操作完成
echo ========================================
pause

