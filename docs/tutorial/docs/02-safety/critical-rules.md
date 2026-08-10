# 五条铁律

!!! danger "无人机是高速旋转的危险品"
    螺旋桨转速可达每分钟上万转，足以造成割伤。请认真阅读以下五条铁律，每次飞行前确认安全条件。

## 铁律一：降落必须人工目视确认电机停转

land() 存在假阳性问题（已知问题 #22）：飞控可能上报解锁状态已清零，但电机实际未停转。降落后手勿靠近，直到确认所有螺旋桨完全静止。

## 铁律二：禁止直接编辑 .c/.h 固件文件

飞控固件使用 GB2312/GBK 编码，直接用 UTF-8 编辑器修改会导致中文注释乱码、编码损坏，严重时 Keil 编译失败或飞行异常。

!!! example "人工操作"
    **人类开发者修改固件：用 Keil uVision 打开工程，在 Keil 内编辑。** Keil 会正确处理 GB2312 编码。

!!! abstract "AI Agent 专用"
    AI Agent 修改固件时使用 `edit_firmware.py` 脚本（确保编码不变）：

    ```bash
    python edit_firmware.py show <文件路径>        # 查看编码
    python edit_firmware.py replace <f> <旧> <新>  # 安全替换
    python edit_firmware.py verify <文件路径>       # 验证编码不变
    ```

## 铁律三：飞行区域必须清空

起飞前5秒红灯警示期内所有人员撤离到3米以外。设置紧急降落开关（遥控）随时准备接管。

## 铁律四：SSH 操作遵守分级规范

!!! abstract "AI Agent 专用规范"
    以下分级规范约束 AI Agent 在板子上的 SSH 操作行为。人类开发者通过 SSH 手动操作时，参考此表了解哪些命令需要谨慎。

    | 级别 | 操作 | AI Agent 规则 |
    |------|------|-------------|
    | 放开 | `ls`/`cat`/`ps`/`git log`/`df` | 随意 |
    | 限路径 | `cp`/`scp`/`mkdir` 在工作目录下 | 执行前说明 |
    | 需确认 | `rm -rf`/`git reset --hard` | 展示命令后等用户同意 |
    | **禁止** | `systemctl`/`reboot`/`apt`/`kill` 非自己进程/改网络配置 | 停下确认 |

!!! tip "人类开发者"
    人类开发者通过 SSH 登录板子后，可自行执行常用操作。涉及 `rm -rf`、系统级修改等高危操作时，务必确认路径后再执行。

## 铁律五：数据只存本地

!!! example "人工操作"
    每次飞行结束后必须执行（从项目根目录）：

    ```bash
    ./tools/pull_flight_log.sh
    ```

    把飞行日志拉取到本地 `data_archive/` 归档，清空板端数据。板子存储空间有限。

---

[高危已知问题 →](known-hazards.md)
