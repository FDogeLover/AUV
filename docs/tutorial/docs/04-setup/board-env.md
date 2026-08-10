# 板端环境配置

地瓜派 RDK X5 已预装系统（用户 `sunrise`），通过 SSH 连接。

## SSH 连接

```bash
# 别名已在 ~/.ssh/config 配置
ssh ubuntu-pi

# 进入工作目录（路径自行确定，以下为示例）
cd ~/<你的工作目录>/drone_control/basic

# 安装运行时依赖
pip install -r requirements.txt
```

!!! tip "工作目录自定义"
    板端仓库部署路径自行选择，不需要和教程示例完全一致。后续命令中的路径均以你实际克隆的位置为准。

!!! info "SSH 别名说明"
    `ubuntu-pi` 是动态IP别名（以本地 `~/.ssh/config` 配置为准）。板子IP变动时可通过路由器后台查找。

## 串口权限

确保用户有串口访问权限：

```bash
sudo usermod -aG dialout sunrise
# 重新登录后生效
```

## T265 权限

如果T265连接后权限不足，需要配置 udev 规则（一般已配置好）。

## 验证串口

```bash
# 确认飞控串口存在
ls -la /dev/ttyS6

# 确认T265被识别
lsusb | grep Intel
```

---

← [本地开发环境](local-env.md) | [桌面模拟飞行 →](../05-quick-start/dry-run.md)
