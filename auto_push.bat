@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo ========================================
echo 自动初始化Git并推送到GitHub
echo ========================================
echo.

REM 初始化Git仓库
if not exist .git (
    echo [1/5] 初始化Git仓库...
    call git init
    if errorlevel 1 (
        echo [错误] Git未安装，请先安装Git
        pause
        exit /b 1
    )
    echo [OK] Git仓库初始化完成
) else (
    echo [1/5] Git仓库已存在
)

REM 配置远程仓库
echo [2/5] 配置远程仓库...
call git remote remove origin 2>nul
call git remote add origin https://github.com/wangmingzhen595-prog/rain.git
echo [OK] 远程仓库配置完成

REM 添加所有文件
echo [3/5] 添加文件到暂存区...
call git add .
echo [OK] 文件添加完成

REM 提交更改
echo [4/5] 提交更改...
call git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力" 2>nul
if errorlevel 1 (
    echo [提示] 没有需要提交的更改，或提交失败
) else (
    echo [OK] 提交完成
)

REM 设置分支并推送
echo [5/5] 推送到GitHub...
call git branch -M main 2>nul
call git push -u origin main 2>&1
if errorlevel 1 (
    echo.
    echo [提示] 推送可能需要认证
    echo 如果提示输入用户名和密码：
    echo - 用户名：你的GitHub用户名
    echo - 密码：使用Personal Access Token（不是GitHub密码）
    echo.
    echo 获取Token：GitHub -^> Settings -^> Developer settings -^> Personal access tokens
    echo.
    call git push -u origin main
)

echo.
echo ========================================
echo 操作完成！
echo ========================================
pause

