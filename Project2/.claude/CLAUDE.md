---
enabledSkills:
  - superpowers:brainstorming
  - superpowers:writing-plans
  - drone-tools
disabledSkills:
  - loop
  - claude-api
---

# Project2 - 无人机工程

无人机工程项目，包含飞控固件（C/Keil）、Python 上位机控制端、基本飞行测试模块。

## 项目结构

```
├── ANO_LX_FC_倾角保护版/      ← 飞控固件（C/Keil, STM32F407）
│   ├── FcSrc/Ano_Scheduler.c  50Hz 主调度循环
│   ├── Mycode/my_protocol.c   通信协议 + height_set PID
│   └── Mycode/my_protocol.h   协议头文件
│
├── drone_control/              ← Python 上位机
│   ├── original/               ← 全功能版（2026-07-07 从 drone_control/ 根目录整理进此子目录，跟ubuntu-pi上 `~/Desktop/FJJ/original/` 目录名对齐）
│   │   ├── main.py            入口
│   │   ├── Mission_GPT.py     任务状态机
│   │   ├── t265.py            T265 视觉里程计
│   │   ├── Lcode/             核心库 (Lpid, Lprotocol, global_variable, Logger, k230_client, coverage_planner)
│   │   └── 地面站通信协议.md
│   ├── basic/                  ← 精简版：仅基本飞行必需模块
│   │   ├── main.py            入口（无 K230/地面站）
│   │   ├── Mission_GPT.py     简化状态机
│   │   ├── t265.py            同全功能版
│   │   ├── router.txt         航点文件
│   │   └── Lcode/             精简核心库
│   ├── basic_radar/            ← 精简版 + N10P激光雷达（2026-07-07 新建，以 basic/ 为模板）
│   │   ├── Lcode/Lradar.py    N10P 串口协议解析（Serial_radar 类）
│   │   ├── radar_bench_test.py 雷达台架通电测试（不涉及飞控/T265）
│   │   └── （其余文件同 basic/）
│   └── tools/                  ← 跨版本共享分析工具（analyze_of_t265_correlation.py 等）+ test_data_* 归档
│
├── docs/architecture_overview.svg  运行时架构图
└── edit_firmware.py                飞控固件安全编辑工具
```

## 编码规范（关键约束！）

### 飞控固件 .c/.h 文件

**编码是混合的**——不能一概而论：

| 编码 | 典型文件 | 说明 |
|------|---------|------|
| **GBK** | `Ano_Scheduler.c`, `Ano_Math.c`, `Drv_*.c` | 大多数原始固件文件，含中文注释 |
| **UTF-8** | `my_protocol.c`, `my_protocol.h` | 稳定性重构时改写，中文在 UTF-8 下正常 |

**编辑规则（严格执行）：**

```
1. 禁止使用 Read/Edit 工具操作 .c/.h 文件！
   → 可能静默转换编码，导致中文注释永久损坏

2. 必须使用 edit_firmware.py 脚本编辑：
   python edit_firmware.py show <file>       # 查看编码
   python edit_firmware.py replace <f> <o> <n>  # 安全替换
   python edit_firmware.py verify <file>     # 验证编码

3. edit_firmware.py 自动完成：检测编码 → 原编码读取
   → 替换 → 原编码写回 → 验证编码不变

4. 每次编辑后必须 run verify 确认编码完整
```

### Python 文件
- UTF-8 编码，可正常使用 Read/Edit
- 串口路径通过环境变量配置：`DRONE_FC_PORT`

## 使用方式

```bash
# 基本飞行测试（树莓派）
cd drone_control/basic && python main.py

# 基本飞行 + N10P雷达台架测试（不涉及飞控/T265，不需要解锁飞机）
cd drone_control/basic_radar && DRONE_RADAR_PORT=/dev/ttyUSB0 python radar_bench_test.py

# 全功能飞行
cd drone_control/original && python main.py
```

## Git 约定

- 当前分支: `main`（`refactor/stability-v1`已合并/废弃，之前记录已过期）
- 主分支: `main`
- 提交格式：`模块: 改动简述`
- 2026-07-09起：本机提交后主动push到远程，不用等用户每次要求；板子(`ubuntu-pi`)`FJJ/.git`独立历史，本地commit要及时但依然不push（跟本机仓库无push/pull关联）

## 关键设计决策

### 下行帧协议（飞控 → Pi，`my_protocol.c` `pi_send()`）— 2026-07-04 已重构并真机验证通过
- 拆成两种帧，格式统一为 `AA | frame_id | len | DATA | checksum | 0xFF`：
  - **帧1（0x01，飞行关键帧，24字节数据）**：`Loop_50Hz` 调用，`mission_stage`/`roll,pitch,yaw`/`fusion_state`/`unlock_sta`(真实解锁状态)/`x_int,y_int`(光流积分)/`laser_height_cm`/`of1_dx,dy`(光流融合速度)/`of_quality,link_sta,work_sta`
  - **帧2（0x02，调试扩展帧，19字节数据，2026-07-08从18字节扩展）**：`Loop_2Hz` 触发但内部降频到约2.5秒一次，`fc_vel_xyz`(凌霄IMU速度估计)/`of_acc_xyz,of_gyr_xyz`(光流模块自带IMU原始数据)/`motor_pwm_mask`(新增，电机PWM非零位掩码bit0~3=m1~m4，诊断unlock_sta假阳性用，见问题17)
  - 两帧共用 USART2，`pi_send()` 统一驱动发送（`Send_str_by_len` 整帧阻塞发送，不再逐字节跨tick分段），避免了帧交织 bug，帧1实际到达率约 50Hz（之前分段发送方式只有 ~1.7Hz）
- Python 侧 `Lprotocol.py` `listen_fc()` 按 `frame_id` 分发解析，`Serial_fc` 串口超时从 0.05s 改成 1.0s（帧变长后需要更长超时才能收全）
- `re_fc`/`rxbuffer` 现在是 14 个字段（見 `main.py` 里 `re_fc` 注释的字段顺序）

### 上行帧（Pi → 飞控）
- `AA 01`：T265 速度帧（vx,vy,yaw），100Hz，整帧一次性 `ser.write()`，不受分段发送问题影响
- `AA 02`：指令帧（task_sta/com_x/y/z/yaw），50Hz
- `AA 03`：T265 位置帧（x,y,z，cm，s32，小端序），新增。**飞控侧 `my_protocol.c` 会解析存入 `t265_pos_x/y/z`，但目前没有任何代码消费这个数据**——原计划通过 CMD 0x32（通用位置型传感器数据）转发给凌霄IMU，已确认凌霄IMU固件不支持该通道，转发部分已回退，只保留飞控本地接收

### 飞控 ↔ 凌霄IMU（USART5，500000波特率，匿名通信协议V7，与 Pi↔飞控完全独立的第二套协议）
- 飞控本身**没有物理IMU**，姿态自稳完全由外部"凌霄IMU"模块自己完成（自己的加速度计/陀螺仪 → 姿态融合 → PID → 电机混控 → CMD 0x20 PWM 值发回飞控 → `ESC_Output()` 直接转发给电调），跟 Pi/T265 无关
- **定点悬停**（不漂移）依赖外部速度参考：`ext_sens.gen_vel`（CMD 0x33）默认用 T265 速度（`pi_ctrl_mode` 开机即强制=1，见 `User_Task.c:26`），T265完全失效时才会退回光流。纯IMU（无T265无光流）无法定点，会漂移
- "倾角保护"来自 `Mycode/angle_protect.c` 的 `Attitude_Check()`：roll/pitch 超过 30° 触发失控保护，跟凌霄IMU的融合算法无关，是这块板子自己加的安全阀
- `flex_send_t265_vel()`/`flex_send_guangliu_vel()`（`my_protocol.c`，走 0xF1/0xF2 灵活格式帧）只是调试可视化通道，转发 T265/光流速度给凌霄IMU官方上位机画波形，**不影响实际导航融合**
- **凌霄IMU官方上位机**：`D:\Competition\自组无人坤\1\匿名上位机.exe`（"匿名科创"品牌，V7.3-20240423）。**连接方式是 USB HID（需要专用"匿名控制台"设备，也支持UDP/TCP），不经过现有的 Pi↔飞控串口链路，只能有线连接，无法在无缆自由飞行时实时监控**——定位是地面台架调试工具（比如拆桨接USB测起飞时序），不是飞行中监控手段。
  - 功能模块：**飞控状态**(实时飞行模式/锁定/姿态/高度/GPS/电压)、**数据显示**(完整协议帧浏览器，Frame ID 0x01~0x51逐字段展开)、**飞控参数**(166项参数表)、**功能触发**(磁力计/陀螺仪/水平/6面校准+清空APP进固件升级模式)、**凌霄IMU**(控制参数微调+校准+遥测频率配置)、**地图显示**(需GPS)、**连接设置**(USB HID/UDP/TCP)
  - **关键参数**：`TAKEOFFHIGH`（起飞后自动尝试定高的目标高度，cm）/`TAKEOFFSPEED`（起飞爬升速度，cm/s）——确认 `OneKey_Takeoff()` 触发的是IMU自己独立的"按固定速度爬升到固定高度后自动定高"逻辑，用这两个参数，不是飞控算出来的
  - **关键遥测字段**（Frame 0x06 展开）：`FC_STA_FlyMode`（0=姿态自稳/1=自稳+定高/2=定点飞行/3=程控飞行）、`FC_STA_SFlag`（0锁定/1解锁）、`FC_STA_CID/CMD0/CMD1`（当前执行功能，对应0xE0帧）；（Frame 0x0E 展开）`FC_STA_I_GVEL`/`FC_STA_I_AALT`（外接速度/测高传感器连接状态，即T265/光流是否被IMU认为在线有效）——如果以后做**台架通电测试**（拆桨、USB直连飞控），可以实时看 `FC_STA_FlyMode` 在一键起飞前后有没有真正切换，验证已知问题6的候选③

### 飞控闭环控制
- `height_set()`（Kp=0.8, Ki=0.05, Kd=0.2，输出限幅±30）**不是位置式PID输出目标高度，而是算出一个垂直速度指令**：`User_Task.c:169` 在 `mission_step==5`（视觉控制阶段）调用 `tar_setdata(com_x, com_y, height_set(ano_of.of_alt_cm, com_z), com_yaw)`，`tar_setdata()`（`my_fun.c`）把这个输出塞进 `rt_tar.st_data.vel_z`（注释标注"垂直速度 cm/s"），通过匿名协议V7的 `0x41` 帧发给凌霄IMU——高度控制实际是**飞控外层算速度指令 + 凌霄IMU内层执行**的级联结构，不是飞控自己直接闭环到位置
- 上位机控制周期 30ms，速度帧 100Hz，指令帧 50Hz
- `Mission_GPT.py`：T265 连接成功后会等追踪置信度达标（默认8秒超时）才允许进入定点/起飞，否则跟T265完全失败一样需要人工输入 `YES` 确认
- `DRONE_DRY_RUN=1` 环境变量：桌面测试用，`se_fc[2]`(task_sta) 永远为0，飞控不会解锁

## 已知未解决问题（供下次会话参考）

> 详情全部迁移到 [`docs/known_issues.md`](../docs/known_issues.md)（完整时间线/数据表格/原始数据路径）。这里只放当前状态速查表，**新记录要写详情文件，这里只改一行摘要**。
>
> 标签：🔴未解决(安全关键/待办) 🟡观察中 ✅已解决 ⏸暂缓/已关闭 🟢工具就绪

| # | 问题 | 状态 | 现状摘要 |
|---|---|---|---|
| 1 | T265冷启动检测失败 | 🔴 | 概率性，物理拔插恢复，无法主动触发/规避 |
| 2 | 光流坐标系是否对齐机体 | ⏸ | 已关闭，六次测试确认对齐 |
| 3 | T265+光流数据融合 | ⏸ | 暂缓，无实测证据支撑必要性 |
| 4 | 凌霄IMU原始IMU(CMD 0x01) | 🔴 | 从未启用，需要振动分析时再开 |
| 5 | Keil编译警告 | 🔴 | 具体内容未记录，低优先级 |
| 6 | 高度控制异常(通信节流bug) | ✅ | 已修复(sleep缩进bug)，真机验证有效 |
| 7 | 一键降落/起飞指令丢弃/确认异常 | 🔴 | 安全关键，长期追踪；确认通道本身可能不可靠，需人工目视确认电机停转 |
| 8 | XY悬停随机漂移~15cm | 🟡 | 未阻塞到达判定，根源不明 |
| 9 | 起飞/降落盲飞水平位移 | ✅/见#7 | 起飞已改良(TAKEOFF_LIFTOFF_CM分段)；降落见#7 |
| 10 | 激光高度阈值写反(冻结bug) | ✅ | 已修复(`>50`改`>5/10`) |
| 11 | T265安装角度导致Y向定向漂移 | 🟡 | 已物理矫正，2026-07-12复核仍健康(6.0/2.9cm)，暂不追加软件补偿 |
| 12 | land()/navigate()缺速度门槛 | ✅ | 已修复(含滑动窗口平滑) |
| 13 | N10P雷达细杆测距不稳定 | ✅ | 已用PoleTracker(空间聚类+时间持续性)缓解，有效探测距离~1m |
| 14 | 雷达悬停避障接入navigate() | ✅ | 滞回+悬停位置锁定+15s超时兜底均已验证；降落不检查杆子靠"末航点回原点"约定规避 |
| 15 | 步长-扰动关系 | ✅ | 唯一稳固结论：扰动随步长单调增大(3次独立验证)；"0.2m硬上限"已被反向复测推翻，且问题21大步长大范围测试精度反而更好，无安全步长上限 |
| 16 | navigate() yaw修正回路 | ⏸ | 默认禁用；2026-07-12递进测试得边界[0.45,0.5]，但收益未证实优于不修正，改回默认关闭 |
| 17 | 电机PWM诊断字段(motor_pwm_mask) | ✅ | 已实现，用于问题7/22排查 |
| 18 | 转盘台架yaw自洽性检验 | ⏸ | 结果不稳定(2:1不支持异常)，yaw_sign标定优先级降低 |
| 19 | 到达确认优化(滑动窗口比例制+arrival_hold_s压缩) | ✅ | 已生效，arrival_hold_s压到0.3s，平均~4秒/段 |
| 20 | 多障碍物场景(PoleTracker多目标) | ✅ | 已验证可同时跟踪2个目标；POLE_DANGER_DIST_M上调到0.9(计入机身半径) |
| 21 | 大范围20点网格精度/速度基线 | ✅ | 精度4.6cm均值；顺带修复land()单帧误判(LAND_UNLOCK_CONFIRM_COUNT=5) |
| 22 | land()双条件确认(unlock_sta+motor_pwm_mask) | 🔴 | **最高优先级安全隐患**：两字段均归零但电机实际未停转，根因未定位，疑似凌霄IMU固件/硬件层面，人工目视确认电机停转不可替代 |
| 23 | xy_pid增益调优(0.7→0.82/0.06) | ✅ | 已应用(仅basic_radar)，速度提升29% |
| 24 | 续航测试基线 | ✅ | 关闭雷达后完整跑通；5分钟/74航点续航测试到达确认零超时 |
| 25 | T265原始IMU诊断接口(raw_imu) | 🟢 | 已实现并接入land()日志，尚未实战用于诊断 |
| 26 | fire_patrol HOVER_DROP后高度不恢复 | 🔴 | 软件命令正确但实测高度24秒无响应，疑似飞控/凌霄IMU状态未复位；2026-07-17真机复现(卡住51秒后串口超时)，已补HOVER_DROP飞行日志(此前从未记录)，根因待下次复现诊断 |
| 27 | fire_patrol长直线航段疑似yaw导致xy耦合 | 🟡 | 偏移先增后收敛，有补偿方案思路(不碰yaw本身)，未实现未验证 |
| 28 | fire_patrol火情检测命中率低(~1/2000帧) | 🟡 | 已证伪"伺服发散"，真根因是灯罩星芒高光圆度过低(非运动模糊)，已加闭运算修复，真机复测待验证 |
| 29 | fire_patrol巡航航点改"掠过式"+超时误判连跳bug修复 | ✅ | 2026-07-17真机验证通过(12航点全程飞完)，同批router.txt旋转90度此前已验证 |
| 30 | fire_patrol红色矩形地标被固定遮挡物切碎，碎片圆度混过阈值误触发 | 🟡 | 遮挡物已挪开(疑似脚架腿)，MIN_CIRCULARITY 0.5→0.7双重保险，真机验证待下次测试；附带修复main.py退出segfault |
| 31 | fire_patrol起飞"边爬升边平移"+曝光过曝看不清 | ✅ | PRECISION_HEAD_WAYPOINTS=1已2026-07-17第三次真机验证(先爬满1.6m再平移)；曝光2645→300已2026-07-17完整路径测试验证清晰可检测，圆度阈值一并验证有效 |
| 32 | fire_patrol近地误检测(实为自身激光反光)+main.py退出崩溃 | ✅/⏸ | 高度门槛已2026-07-17第三次真机验证生效；退出崩溃是pyrealsense2内部C++线程SIGABRT(非段错误)，任务已安全结束后才发生、影响很低，决定不再继续修 |
| 33 | fire_patrol问题28检测命中率低+问题26 HOVER_DROP高度不恢复 | ✅/🟡 | 2026-07-17完整12航点真机测试首次一次性全部走通：检测触发→APPROACH对准→HOVER_DROP全流程→全部航点→正常降落，圆度修复(问题28)已验证；HOVER_DROP这次未卡住，但问题26根因仍未定位(样本量仍小，已有日志留待下次复现) |
| 34 | fire_patrol HOVER_DROP抛投完成后"一边巡航一边爬升" | 🟢 | 新增RECOVER_HEIGHT状态，原地爬升回巡航高度再恢复PATROL(跟问题31起飞爬升顺序同类)，板载94/94单元测试通过，真机验证待下次测试 |

## 远程设备操作规范（SSH 到板载设备）

### 主机别名
- `ubuntu-pi`（root@192.168.137.125）：当前飞控载体，`~/Desktop/FJJ/` 下部署 `original/`（全功能版）+ `basic/`（精简版，从本机 `drone_control/basic/` 移植），已建独立 git 仓库用于回退
- `orangepi`（orangepi@192.168.137.126）：角色待定，目前离线

### 文件修改范围
- 默认工作范围：`~/Desktop/FJJ/**`，无额外说明时只在此操作
- `Desktop/` 下其他目录（`雷达/`、`GPIO测试/`、`T265/`、`wzh/`、`auto-boot/` 等）不是禁区，但只有用户明确指定时才动
- 系统级路径只读，不修改：`/etc/`、`/usr/`、`/boot/`、systemd unit 文件、`~/.ssh/`（sshd 配置、authorized_keys）

### 指令分级

| 级别 | 类型 | 处理方式 |
|------|------|---------|
| 放开 | 只读探查：`ls`/`cat`/`ps`/`lsusb`/`dmesg`/`ip a`/`df`/`git log`/`git status`/`git diff`/`python3 --version` 等 | 随时执行，无需确认 |
| 限定路径可做 | `cp`/`mv`/`mkdir`/`scp`/`chown`，目标路径在 `Desktop/` 下明确指定的目录内 | 执行前说明将改动的文件 |
| 需先确认 | `rm -rf`、`git reset --hard`、`git clean -f`、批量 `chown -R` | 先展示具体命令和受影响范围，等待明确同意再执行 |
| 禁止 | `systemctl`/`service`/`reboot`/`shutdown`/`poweroff`、`apt`/`pip install`（系统级）、修改 `/etc/` 下文件、`crontab -e`、修改 `wlan0`/网络配置、修改 `sshd_config`/`authorized_keys`、`kill`/`pkill` 非自己起的进程 | 不打招呼不能执行，出现相关需求必须停下确认——这些可能导致唯一联网路径（wlan0 静态 IP）断掉，断了只能物理上手 |

### 其他约定
- root 登录做的文件操作（`mkdir`/`mv`/`scp` 上传等）之后必须 `chown -R sunrise:sunrise` 改回属主
- `FJJ/.git` 是板载独立历史，不与本机仓库做 push/pull 关联，只用于板子上本地 `commit`/`checkout`/`reset` 回退
- 推送本机代码到远程用逐文件 `scp`，不用递归复制整个目录（避免带上 `__pycache__/`、`.ipynb_checkpoints/`）
- **`FJJ/.git` 里换行符约定不统一，scp 后提交前必须先核对**：`basic_radar/Lcode/Lradar.py` 是 LF，但 `basic_radar/Mission_GPT.py`/`main.py` 历史上就是 CRLF（板子仓库本身遗留的不一致，不是本机的问题）——本机 Windows 编辑器/git 存的文件基本都是 CRLF，如果不管三七二十一 scp 过去就 `git commit`，凡是原本是 LF 的文件会被换行符污染成假性全文件重写(`git diff --stat` 显示几百上千行改动，实际可能只改了几十行)。**流程**：scp 完先 `git show HEAD:<file> | file -` 看原来是不是 CRLF，跟 `file <file>` 比对当前工作区状态，不一致就用 `sed -i 's/\r$//'`(转LF) 或 `sed -i 's/$/\r/'`(转CRLF，注意不要对已有CRLF的行重复加，先转LF再统一转CRLF更保险) 按各文件原有约定改回来，再 `git diff --stat` 确认改动行数量级合理，才能 `git commit`
- **飞行测试数据要同步回本机仓库**：`ubuntu-pi` 上 `basic/test_data_*` 里归档的 `flight_data_*.jsonl.bak` 等测试数据，以后要同步一份到本机仓库对应位置（`drone_control/tools/test_data_*`），不要只留在板子上——之前 2026-07-06 的数据只在板子上，本机分析时要临时 ssh 上去现拉，之后应避免这种情况
- **飞行数据归档目录约定 — 板子和本机不是同一个位置，板子内部也不再按模块分散**：**2026-07-08 起统一改成 `FJJ/test_data/<版本>_<日期>/`**（比如 `test_data/basic_20260705/`、`test_data/basic_radar_20260708/`），版本名前缀区分是 `basic`/`original`/`basic_radar` 哪个模块测的，不再各模块自己开 `test_data_YYYYMMDD/` 子目录（此前 `basic/test_data_20260705`、`basic/test_data_20260706`、`basic_radar/test_data_20260708` 这种各模块自建子目录的旧结构，已用 `git mv` 统一迁移到新位置，保留了git历史）。这些日期子目录本身要 `git add` 提交进 `FJJ/.git`，不是只是散放的临时文件。本机仓库统一放在 `drone_control/tools/test_data_YYYYMMDD/`（不分模块，本机这边保持不变），板子和本机两边目录结构不对称，同步时不要想当然按同一路径映射
- `original/main.ipynb`、`original/tets_k230_tongxin.ipynb` 只存在于设备上，可能含本机没有的实验性改动，推送/覆盖前先看内容，避免丢失
- 任何"限定路径可做"及以上级别的操作，执行后立刻用只读命令验证结果，不假设命令一定成功
