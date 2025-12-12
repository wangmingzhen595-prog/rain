@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo ========================================
echo Git诊断工具
echo ========================================
echo.

REM 检查Git是否安装
echo [1] 检查Git安装...
git --version
if errorlevel 1 (
    echo [X] Git未安装或未添加到PATH
    echo 请安装Git: https://git-scm.com/
    goto :end
) else (
    echo [OK] Git已安装
)
echo.

REM 检查Git配置
echo [2] 检查Git配置...
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

REM 检查Git仓库
echo [3] 检查Git仓库...
if exist .git (
    echo [OK] Git仓库已初始化
    git status --short
) else (
    echo [X] Git仓库未初始化
    echo 请运行: git init
)
echo.

REM 检查远程仓库
echo [4] 检查远程仓库...
git remote -v
if errorlevel 1 (
    echo [X] 未配置远程仓库
    echo 请运行: git remote add origin https://github.com/wangmingzhen595-prog/rain.git
) else (
    echo [OK] 远程仓库已配置
)
echo.

REM 检查分支
echo [5] 检查分支...
git branch --show-current
if errorlevel 1 (
    echo [X] 当前不在任何分支上
) else (
    echo [OK] 当前分支: 
    git branch --show-current
)
echo.

REM 检查提交历史
echo [6] 检查提交历史...
git log --oneline -5
if errorlevel 1 (
    echo [X] 没有提交记录
) else (
    echo [OK] 有提交记录
)
echo.

:end
echo ========================================
pause

