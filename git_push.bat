@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo ========================================
echo 初始化Git并推送到GitHub
echo ========================================
echo.

REM 检查Git是否安装
git --version >nul 2>&1
if errorlevel 1 (
    echo [错误] Git未安装或未添加到PATH环境变量
    echo 请先安装Git: https://git-scm.com/
    pause
    exit /b 1
)

REM 检查是否已初始化git
if not exist .git (
    echo [1/6] 初始化Git仓库...
    git init
    if errorlevel 1 (
        echo [错误] Git初始化失败
        pause
        exit /b 1
    )
    echo Git仓库初始化完成
) else (
    echo [1/6] Git仓库已存在
)

REM 检查并设置远程仓库
echo [2/6] 检查远程仓库...
git remote -v >nul 2>&1
if errorlevel 1 (
    echo 添加远程仓库...
    git remote add origin https://github.com/wangmingzhen595-prog/rain.git
) else (
    git remote get-url origin >nul 2>&1
    if errorlevel 1 (
        echo 添加远程仓库...
        git remote add origin https://github.com/wangmingzhen595-prog/rain.git
    ) else (
        echo 更新远程仓库URL...
        git remote set-url origin https://github.com/wangmingzhen595-prog/rain.git
    )
)
echo 远程仓库设置完成

REM 添加所有文件
echo [3/6] 添加文件到暂存区...
git add .
if errorlevel 1 (
    echo [错误] 添加文件失败
    pause
    exit /b 1
)
echo 文件添加完成

REM 检查是否有更改需要提交
git diff --cached --quiet
if errorlevel 1 (
    echo [4/6] 提交更改...
    git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力"
    if errorlevel 1 (
        echo [错误] 提交失败
        pause
        exit /b 1
    )
    echo 提交完成
) else (
    echo [4/6] 没有需要提交的更改
)

REM 检查当前分支
echo [5/6] 检查分支...
git branch --show-current >nul 2>&1
if errorlevel 1 (
    echo 创建main分支...
    git checkout -b main
)

REM 推送到GitHub
echo [6/6] 推送到GitHub...
echo 正在推送，请稍候...
git push -u origin main 2>&1
if errorlevel 1 (
    echo 尝试推送到master分支...
    git branch -M master
    git push -u origin master 2>&1
    if errorlevel 1 (
        echo.
        echo [错误] 推送失败！
        echo 可能的原因：
        echo 1. 网络连接问题
        echo 2. GitHub认证失败（需要配置用户名和邮箱）
        echo 3. 仓库权限问题
        echo.
        echo 请检查：
        echo - git config --global user.name "YourName"
        echo - git config --global user.email "YourEmail"
        echo - GitHub仓库权限设置
        pause
        exit /b 1
    )
)

echo.
echo ========================================
echo 推送成功！
echo ========================================
pause

