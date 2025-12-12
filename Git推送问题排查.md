# Git推送问题排查指南

## 快速诊断

运行 `git_diagnose.bat` 查看当前Git状态和配置。

## 常见问题及解决方案

### 1. Git未安装或未配置

**症状**：运行脚本时提示"Git未安装"

**解决方案**：
1. 下载安装Git：https://git-scm.com/download/win
2. 安装时选择"添加到PATH环境变量"
3. 重新运行脚本

### 2. 未配置用户名和邮箱

**症状**：提交时提示需要配置user.name和user.email

**解决方案**：
- 运行 `git_setup.bat` 进行配置
- 或手动执行：
  ```bash
  git config --global user.name "YourName"
  git config --global user.email "YourEmail@example.com"
  ```

### 3. GitHub认证失败

**症状**：推送时提示认证失败或权限被拒绝

**解决方案**：

#### 方法1：使用Personal Access Token（推荐）

1. 登录GitHub
2. 进入 Settings → Developer settings → Personal access tokens → Tokens (classic)
3. 点击 "Generate new token (classic)"
4. 勾选 `repo` 权限
5. 生成token并复制
6. 推送时使用token作为密码：
   ```bash
   git push -u origin main
   # 用户名：你的GitHub用户名
   # 密码：粘贴刚才复制的token
   ```

#### 方法2：使用SSH密钥

1. 生成SSH密钥：
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```
2. 复制公钥内容（`~/.ssh/id_ed25519.pub`）
3. 在GitHub上添加SSH密钥：Settings → SSH and GPG keys → New SSH key
4. 修改远程仓库URL为SSH：
   ```bash
   git remote set-url origin git@github.com:wangmingzhen595-prog/rain.git
   ```

### 4. 仓库不存在或没有权限

**症状**：推送时提示"repository not found"或"permission denied"

**解决方案**：
1. 确认仓库URL正确：https://github.com/wangmingzhen595-prog/rain.git
2. 确认你有该仓库的写入权限
3. 如果仓库不存在，先在GitHub上创建仓库

### 5. 分支名称不匹配

**症状**：推送时提示分支不存在

**解决方案**：
- 如果远程仓库是`main`分支：
  ```bash
  git branch -M main
  git push -u origin main
  ```
- 如果远程仓库是`master`分支：
  ```bash
  git branch -M master
  git push -u origin master
  ```

### 6. 网络连接问题

**症状**：推送超时或连接失败

**解决方案**：
1. 检查网络连接
2. 如果使用代理，配置Git代理：
   ```bash
   git config --global http.proxy http://proxy.example.com:8080
   git config --global https.proxy https://proxy.example.com:8080
   ```
3. 如果在中国大陆，可能需要使用镜像或VPN

## 手动推送步骤

如果脚本无法运行，可以手动执行以下命令：

```bash
# 1. 初始化仓库（如果还没有）
git init

# 2. 配置用户信息（如果还没有）
git config --global user.name "YourName"
git config --global user.email "YourEmail@example.com"

# 3. 添加远程仓库
git remote add origin https://github.com/wangmingzhen595-prog/rain.git
# 如果已存在，更新URL：
# git remote set-url origin https://github.com/wangmingzhen595-prog/rain.git

# 4. 添加文件
git add .

# 5. 提交更改
git commit -m "优化小雨滴检测：支持420-540mV信号检测，添加移动平均滤波和时间特征判定，提高噪声区分能力"

# 6. 推送到GitHub
git branch -M main
git push -u origin main
```

## 获取帮助

如果以上方法都无法解决问题，请：
1. 运行 `git_diagnose.bat` 获取诊断信息
2. 查看错误信息的具体内容
3. 在GitHub Issues中提问，附上诊断信息

