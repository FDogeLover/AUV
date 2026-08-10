# 项目是什么

## :memo: 本节介绍

本节介绍 Project2 的定位、与开源飞控的区别、面向的赛题，以及新手应该从哪里开始。

## :trophy: 学习目标

1. 知道 Project2 是什么、做什么用的
2. 理解它和 PX4 / ArduPilot 等开源飞控的核心区别
3. 了解项目面向哪些赛题，各自在什么状态
4. 知道新手应该从哪个版本开始

---

## 一句话概括

Project2 是一套**自主无人机飞控系统**，让多旋翼无人机在无 GPS 环境下，依靠视觉里程计（T265）完成自主航线飞行、精准降落和空地协同任务。

## 和开源飞控的区别

| 特点 | 开源飞控 (PX4/ArduPilot) | 本项目 |
|------|--------------------------|--------|
| 飞控 | NuttX RTOS，功能全 | 裸机，只管姿态+电机 |
| 上位机 | Companion Computer（可选） | Python 状态机（核心决策层） |
| 实时性 | 飞控兼顾导航 | 飞控1kHz硬实时，Python 30Hz软实时 |
| 开发语言 | C++ | C（固件）+ Python（上位机） |
| 优势 | 生态成熟 | 极简、可控、适合竞赛快速迭代 |

## 项目面向的赛题

| 赛题 | 版本目录 | 状态 |
|------|---------|------|
| 2026电赛 D题（陆空协同） | `competition_2026_d/` | <span class="status-badge status-green">已实飞通过</span> |
| 2026电赛通用版 | `competition_2026/` | <span class="status-badge status-yellow">代码完成，待实飞</span> |
| 消防巡逻（G题） | `fire_patrol/` | <span class="status-badge status-green">已实飞通过</span> |
| 立体货架盘点 | `warehouse_inventory/` | <span class="status-badge status-green">已验收</span> |
| 圆杆环绕飞行 | `circle_pole/` | <span class="status-badge status-green">阶段1验证</span> |
| 基础自主飞行 | `basic/` | <span class="status-badge status-green">T-016验证通过</span> |

## 新手从哪里开始

!!! tip "永远从 basic/ 开始"
    `drone_control/basic/` 是所有版本的最小可飞基线，其他版本都在它基础上扩展。新手必须先在 basic 上跑通 DRY_RUN 模拟飞行，再考虑其他版本。

---

## :material-help: 常见问题

??? question "我能直接用 PX4 替换这个项目的飞控吗？"
    不建议。本项目的上位机代码（Lprotocol串口协议、Mission_GPT状态机）与飞控固件深度耦合，替换飞控意味着重写整个通信层。如果你只是想飞PX4，直接用PX4生态即可，不需要本项目。

??? question "不用 T265 可以吗？用什么替代？"
    T265 是当前唯一支持的视觉里程计。理论上可以替换为其他VIO方案，但需要修改 `t265.py` 的接口和坐标系转换。这不是新手应该做的事。

??? question "这个项目能用于实际产品开发吗？"
    本项目面向竞赛，不是生产级飞控。缺少冗余设计、故障容错、法规合规等生产级要求。请勿用于商业产品或载人飞行器。

---

← [首页](../index.md) | [三层架构详解 →](three-layer-arch.md)
