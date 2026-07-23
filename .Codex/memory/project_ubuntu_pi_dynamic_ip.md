---
name: project-ubuntu-pi-dynamic-ip
description: ubuntu-pi局域网IP经常变化(DHCP)，SSH连不上时的正确处理流程
metadata: 
  node_type: memory
  type: project
  originSessionId: e76cc3f8-f781-4e57-ae5b-2eaeac2d0db0
---

`ubuntu-pi`(飞控载体板子)的局域网IP是DHCP动态分配的，经常变化——2026-07-17一天内至少变了3次(192.168.137.125→.244→.214)。`~/.ssh/config`里`ubuntu-pi`这个Host别名记录的IP会跟着过期。

**Why(根因已确认，2026-07-17)**：板子通过WiFi连的是**用户电脑的移动热点/网络共享(ICS)**，不是独立路由器——`192.168.137.x`网段+网关`.1`是Windows ICS的固定默认网段，这是判断"是不是连的电脑热点"的标志。板子自身`wlan0`的MAC地址是固定的硬件地址(不是随机化在作怪，曾怀疑过`usb0`/`usb1`接口的"random ethernet address"日志，排查后确认那两个接口是`DOWN`状态、跟联网无关的USB gadget功能，是干扰项)。真正原因是**用户电脑的热点被断开重连过几次**(笔记本休眠唤醒/热点开关切换/共享网络的设备重连)——Windows ICS的DHCP实现不太可靠地保留同一设备的IP，每次热点重启，即使是同一块板子重新连上也可能分配到不同IP，不像正经路由器那样通常记住MAC对应的IP。`.claude/CLAUDE.md`项目文档里如果写死具体IP，会比实际情况滞后(已发生过，文档写的`.125`实际已经是`.244`)。

**How to apply**：
1. SSH连接失败(`Connection timed out`)时，不要假设是网络/电源问题就反复重试同一个IP——先怀疑IP变了，直接问用户当前IP。
2. 拿到新IP后，用`Edit`工具改`C:\Users\FJJ\.ssh\config`里`Host ubuntu-pi`的`HostName`字段。
3. 换IP后大概率会遇到`Host key verification failed`——这是因为SSH的`known_hosts`按IP索引，新IP还没有记录，但这是同一台物理设备(host key指纹应该已经在`known_hosts`里因为旧IP出现过)。用`ssh-keyscan -t ed25519 <新IP> >> ~/.ssh/known_hosts`补上新IP的记录即可，不要用`StrictHostKeyChecking=no`这种全局绕过的方式。
4. 换完IP务必验证是同一块板子再继续操作(比如`ls`一下项目已知路径、看`git log`是否跟预期的历史匹配)，不要假设新IP一定是同一台设备。
5. `.claude/CLAUDE.md`里已经改成不写死具体IP，只说明"以`~/.ssh/config`当前配置为准"，避免文档本身又变成另一个过期IP的来源。
