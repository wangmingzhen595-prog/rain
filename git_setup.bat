@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo ========================================
echo Git配置向导
echo ========================================
echo.

REM 检查Git是否安装
git --version >nul 2>&1
if errorlevel 1 (
    echo [错误] Git未安装
    echo 请先安装Git: https://git-scm.com/
    pause
    exit /b 1
)

REM 配置用户名
echo 请输入Git用户名（用于提交记录）:
set /p git_name="用户名: "
if not "%git_name%"=="" (
    git config --global user.name "%git_name%"
    echo [OK] 用户名已设置: %git_name%
)

REM 配置邮箱
echo.
echo 请输入Git邮箱（用于提交记录）:
set /p git_email="邮箱: "
if not "%git_email%"=="" (
    git config --global user.email "%git_email%"
    echo [OK] 邮箱已设置: %git_email%
)

echo.
echo ========================================
echo 配置完成！
echo ========================================
echo.
echo 现在可以运行 git_push.bat 推送代码
pause

