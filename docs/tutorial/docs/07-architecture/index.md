# 代码架构解析

深入理解 Project2 的状态机流程、目录结构和核心模块设计。

---

<div class="grid chapter-cards" markdown>

-   **状态机流程**

    ---

    IDLE→ARMING→TAKEOFF→MISSION→LANDING 完整状态流转

    [:octicons-arrow-right-24: 查看](state-machine.md)

-   **目录结构**

    ---

    drone_control/ 下各版本目录的组织方式和共享代码

    [:octicons-arrow-right-24: 查看](directory-tree.md)

-   **模块总览**

    ---

    Lcode/ 核心库的六大模块速查表

    [:octicons-arrow-right-24: 查看](modules/index.md)

-   **串口协议 Lprotocol**

    ---

    帧格式、校验、原子接收和常用控制帧

    [:octicons-arrow-right-24: 查看](modules/lprotocol.md)

-   **PID控制器 Lpid**

    ---

    位置环PID参数调优和抗积分饱和

    [:octicons-arrow-right-24: 查看](modules/lpid.md)

-   **航向保持 HeadingHold**

    ---

    yaw轴漂移补偿和runaway保护

    [:octicons-arrow-right-24: 查看](modules/heading-hold.md)

-   **导航策略 NavigationProfile**

    ---

    航点序列、到达判定和高度管理模式

    [:octicons-arrow-right-24: 查看](modules/nav-profile.md)

-   **T265接口**

    ---

    视觉里程计坐标定义、冷启动检测和数据读取

    [:octicons-arrow-right-24: 查看](modules/t265.md)

</div>
