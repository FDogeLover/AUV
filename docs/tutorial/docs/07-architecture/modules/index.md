# 核心模块

`drone_control/basic/` 下的核心代码模块，每个模块对应 `Lcode/` 中的一个 Python 文件。

## 模块列表

| 模块 | 文件 | 职责 |
|------|------|------|
| [串口协议 Lprotocol](lprotocol.md) | `Lcode/Lprotocol.py` | 上位机与飞控的串口双向通信，三线程收发 |
| [PID控制器 Lpid](lpid.md) | `Lcode/Lpid.py` | 位置/速度闭环 PID 运算 |
| [航向保持 HeadingHold](heading-hold.md) | `Lcode/heading_hold.py` | 外环航向锁定，防止飞行中偏航 |
| [导航策略 NavigationProfile](nav-profile.md) | `Lcode/navigation_profile.py` | 航点到达判定策略（precision/cruise） |
| [T265接口](t265.md) | `t265.py` | Intel RealSense T265 位姿读取 |

## 模块间数据流

```
T265 ──→ Mission_GPT（状态机）
              │
              ├──→ Lpid（PID运算）
              │       │
              │       └──→ Lprotocol（串口发送控制帧）
              │
              ├──→ HeadingHold（航向修正）
              │
              └──→ NavigationProfile（航点判定）
```

---

[← 目录结构](../directory-tree.md)
