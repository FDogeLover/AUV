# Project2 无人机开发教程

> 从认识硬件、搭建环境、桌面模拟到第一次真机起飞，一步步上手自主无人机系统。

---

## :material-rocket-launch: 这份教程能帮你做什么

本教程面向**第一次接触本项目的新开发者**。跟着教程走完，你将能够：

<div class="tutorial-benefits" markdown>

- :material-check-circle: 理解飞控固件 + Python上位机 + 视觉板的三层架构
- :material-check-circle: 在本地电脑上零风险模拟飞行
- :material-check-circle: 完成第一次真机自主航线飞行
- :material-check-circle: 知道代码在哪里、怎么改、怎么同步到板子
- :material-check-circle: 遇到问题知道去哪里查

</div>

## :material-map-marker-path: 学习路径

<div class="phase-track">
  <div class="phase-step">
    <div class="phase-num">1</div>
    <div class="phase-label">认识项目</div>
    <div class="phase-time">20分钟</div>
  </div>
  <div class="phase-step">
    <div class="phase-num">2</div>
    <div class="phase-label">搭环境</div>
    <div class="phase-time">30分钟</div>
  </div>
  <div class="phase-step">
    <div class="phase-num">3</div>
    <div class="phase-label">桌面模拟</div>
    <div class="phase-time">15分钟</div>
  </div>
  <div class="phase-step">
    <div class="phase-num">4</div>
    <div class="phase-label">真机首飞</div>
    <div class="phase-time">40分钟</div>
  </div>
  <div class="phase-step">
    <div class="phase-num">5</div>
    <div class="phase-label">理解代码</div>
    <div class="phase-time">30分钟</div>
  </div>
  <div class="phase-step">
    <div class="phase-num">6</div>
    <div class="phase-label">日常开发</div>
    <div class="phase-time">20分钟</div>
  </div>
</div>

## :material-book-open-variant: 章节导航

### :material-information: 项目概览

<div class="grid chapter-cards" markdown>

- :material-drone: **[项目是什么](01-overview/what-is-this.md)**

    自主无人机飞控系统的定位、和开源飞控的区别、面向的赛题 · `5分钟`

- :material-layers-triple: **[三层架构详解](01-overview/three-layer-arch.md)**

    飞控固件 + Python上位机 + 视觉板，每层职责与数据流 · `10分钟`

- :material-chip: **[硬件清单](01-overview/hardware.md)**

    基础飞行最小配置 + 可选进阶硬件 + SSH连接说明 · `5分钟`

</div>

### :material-shield-alert: 安全须知

!!! danger "开始之前必须阅读"
    无人机是高速旋转的危险品。请务必先阅读 **[五条铁律](02-safety/critical-rules.md)** 再进行任何操作。

<div class="grid chapter-cards" markdown>

- :material-gavel: **[五条铁律](02-safety/critical-rules.md)**

    降落确认、固件编码禁令、区域清空、SSH分级、数据归档 · `必读`

- :material-alert-octagon: **[高危已知问题](02-safety/known-hazards.md)**

    land()假阳性、T265冷启动、高度不恢复等飞行前必须了解的问题 · `必读`

</div>

### :material-wrench-cog: 环境与体验

<div class="grid chapter-cards" markdown>

- :material-school: **[前置知识](03-prerequisites/required-knowledge.md)**

    Python、Linux、串口、PID，你需要会什么，不会的给链接 · `5分钟`

- :material-laptop: **[本地开发环境](04-setup/local-env.md)**

    Python 3.10+ 安装、依赖安装、pytest验证 · `15分钟`

- :material-play-circle: **[桌面模拟飞行](05-quick-start/dry-run.md)**

    DRY_RUN 模式零硬件跑通完整状态机流程 · `5分钟`

- :material-test-tube: **[运行单元测试](05-quick-start/run-tests.md)**

    18+ 个测试覆盖核心逻辑，无需硬件 · `5分钟`

</div>

### :material-airplane: 真机飞行

<div class="grid chapter-cards" markdown>

- :material-clipboard-check: **[飞行前检查](06-first-flight/preflight-check.md)**

    6项检查清单 + 默认航线示例 · `10分钟`

- :material-airplane-takeoff: **[起飞流程](06-first-flight/takeoff-flow.md)**

    上电 → 启动程序 → 等待 → 起飞 → 监控 → 降落，6步详解 · `15分钟`

- :material-airplane-landing: **[飞后归档](06-first-flight/post-flight.md)**

    断电 → 日志拉取 → 数据分析 → 记录结果 · `10分钟`

</div>

### :material-code-braces: 代码与开发

<div class="grid chapter-cards" markdown>

- :material-state-machine: **[状态机流程](07-architecture/state-machine.md)**

    IDLE → TAKEOFF → NAVIGATE → DESCEND → LAND → END 全流程图 · `10分钟`

- :material-folder-multiple: **[目录结构](07-architecture/directory-tree.md)**

    项目顶层结构 + basic内部 + 航点文件格式 · `10分钟`

- :material-compare: **[版本选择指南](08-versions/comparison.md)**

    7个版本对比，新手必从 basic 开始 · `5分钟`

- :material-file-edit: **[固件编辑规范](09-workflow/firmware-edit.md)**

    GB2312编码禁令，人工用Keil / AI用edit_firmware.py · `5分钟`

- :material-help-circle: **[故障排查速查](10-troubleshoot/quick-reference.md)**

    9个常见症状：原因 → 处理 → 参考编号 · `参考用`

</div>

### :material-rocket: 进阶开发

<div class="grid chapter-cards" markdown>

- :material-eye: **[视觉伺服降落](11-advanced/visual-servo.md)**

    CyberCAM + IBVS 控制器，精准对准降落点

- :material-car-connected: **[空地协同（D题）](11-advanced/air-ground.md)**

    DCP协议 + 双T265 + 编队 + 投放

- :material-transit-connection-variant: **[事件总线架构](11-advanced/event-bus.md)**

    线程安全有界队列，publish永不阻塞

- :material-tune: **[PID调参指南](11-advanced/pid-tuning.md)**

    当前参数 + 调参建议 + T-016测试结果参考

</div>

---

## 项目简介

Project2 是一套完整的自主无人机飞控系统，面向全国大学生电子设计竞赛备赛。采用 **「裸机飞控 + 上位机智能决策」** 的分层架构：

| 层 | 技术栈 | 作用 |
|---|--------|------|
| 飞控固件 | C / Keil (STM32F407) + 凌霄IMU | 姿态自稳、电机控制、传感器融合 |
| 上位机 | Python 3 + T265 + 光流 | 状态机控制、视觉定位、导航 |
| 视觉板 | 核桃派 Cyber Camera | 目标检测、视觉伺服降落 |

---

*本教程持续更新中，最后更新：2026-08-10*
