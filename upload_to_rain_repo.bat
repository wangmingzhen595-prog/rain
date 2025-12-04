@echo off
chcp 65001 >nul
echo ========================================
echo 上传到GitHub仓库: rain
echo ========================================
echo.

REM 尝试查找Git路径
set "GIT_CMD=git"
where git >nul 2>&1
if errorlevel 1 (
    REM 尝试GitHub Desktop的Git路径
    if exist "%LOCALAPPDATA%\GitHubDesktop\app-*\resources\app\git\cmd\git.exe" (
        for /f "delims=" %%i in ('dir /b /s "%LOCALAPPDATA%\GitHubDesktop\app-*\resources\app\git\cmd\git.exe" 2^>nul') do set "GIT_CMD=%%i"
    )
    REM 尝试Git for Windows的常见路径
    if exist "C:\Program Files\Git\cmd\git.exe" (
        set "GIT_CMD=C:\Program Files\Git\cmd\git.exe"
    )
    if exist "C:\Program Files (x86)\Git\cmd\git.exe" (
        set "GIT_CMD=C:\Program Files (x86)\Git\cmd\git.exe"
    )
)

REM 测试Git是否可用
"%GIT_CMD%" --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到Git命令！
    echo.
    echo 请确保已安装Git，并尝试以下方法：
    echo 1. 安装Git for Windows: https://git-scm.com/download/win
    echo 2. 安装GitHub Desktop: https://desktop.github.com/
    echo 3. 将Git添加到系统PATH环境变量
    echo.
    pause
    exit /b 1
)

echo [信息] 使用Git: "%GIT_CMD%"
echo.

REM 检查是否已配置用户信息
"%GIT_CMD%" config --global user.name >nul 2>&1
if errorlevel 1 (
    echo [提示] 未配置Git用户信息，将使用默认值
    echo 如需配置，请运行：
    echo   git config --global user.name "你的名字"
    echo   git config --global user.email "你的邮箱"
    echo.
)

echo [1/8] 初始化git仓库...
"%GIT_CMD%" init
if errorlevel 1 (
    echo [错误] git init 失败
    pause
    exit /b 1
)

echo [2/8] 添加所有文件...
"%GIT_CMD%" add .
if errorlevel 1 (
    echo [错误] git add 失败
    pause
    exit /b 1
)

echo [3/8] 检查是否有更改...
"%GIT_CMD%" diff --cached --quiet
if errorlevel 1 (
    echo [4/8] 提交更改...
    "%GIT_CMD%" commit -m "优化版本：完善峰值检测算法，添加快照处理，修复电压显示跳变问题"
    if errorlevel 1 (
        echo [警告] git commit 失败
    )
) else (
    echo [4/8] 没有更改需要提交
)

echo [5/8] 重命名分支为main...
"%GIT_CMD%" branch -M main 2>nul
if errorlevel 1 (
    echo [提示] 分支可能已经是main
)

echo [6/8] 添加远程仓库...
"%GIT_CMD%" remote remove origin 2>nul
"%GIT_CMD%" remote add origin https://github.com/wangmingzhen595-prog/rain.git
if errorlevel 1 (
    echo [错误] git remote add 失败
    pause
    exit /b 1
)

echo [7/8] 检查远程仓库连接...
"%GIT_CMD%" remote -v
echo.

echo [8/8] 推送到GitHub...
echo 注意：如果这是第一次推送，可能需要输入GitHub用户名和密码（或Personal Access Token）
echo 如果使用Personal Access Token，请访问: https://github.com/settings/tokens
echo.
"%GIT_CMD%" push -u origin main
if errorlevel 1 (
    echo.
    echo [错误] git push 失败
    echo.
    echo 可能的原因：
    echo 1. 需要配置git用户信息：
    echo    git config --global user.name "你的名字"
    echo    git config --global user.email "你的邮箱"
    echo.
    echo 2. 需要GitHub认证：
    echo    - 使用Personal Access Token而不是密码
    echo    - 创建Token: https://github.com/settings/tokens
    echo    - 选择权限: repo (全部权限)
    echo.
    echo 3. 如果远程仓库已有内容，可能需要先pull：
    echo    git pull origin main --allow-unrelated-histories
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo 上传成功！
echo ========================================
echo.
echo 仓库地址: https://github.com/wangmingzhen595-prog/rain
echo.
pause

