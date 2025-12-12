@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo Git初始化和推送...
echo.

if not exist .git git init
git remote remove origin 2>nul
git remote add origin https://github.com/wangmingzhen595-prog/rain.git
git add .
git commit -m "优化小雨滴检测：支持420-540mV信号检测" 2>nul
git branch -M main 2>nul
git push -u origin main

echo.
echo 完成！如果失败，请检查认证信息。
pause

