@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo 推送代码到GitHub
echo ========================================
echo.

if not exist .git (
    echo 初始化Git仓库...
    git init
)

git remote remove origin 2>nul
git remote add origin https://github.com/wangmingzhen595-prog/rain.git

echo 添加文件...
git add .

echo 提交更改...
git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力"

echo 推送到GitHub...
git branch -M main 2>nul
git push -u origin main

if errorlevel 1 (
    echo.
    echo 如果推送失败，请检查：
    echo 1. GitHub用户名和密码（或Personal Access Token）
    echo 2. 网络连接
    echo 3. 仓库权限
    echo.
    echo 也可以手动执行以下命令：
    echo git push -u origin main
)

echo.
pause

