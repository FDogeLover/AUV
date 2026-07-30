# 小车端通信增量清单（任务一联调）

本文只列出相对于小车现有“发送`CAR_START` + 10 Hz `CAR_STATE`”新增的内容。
DCP v1帧头、CRC、小端序和设备编号保持不变。

## 1. 小车新增接收消息

| type | 名称 | payload | 处理 |
|---:|---|---|---|
| `0x02` | `UAV_READY` | `<BHI>`，7字节 | 等待启动阶段约2 Hz接收 |
| `0x04` | `ACK` | `<BHB>`，4字节 | 匹配`acked_type/acked_seq/session_id` |
| `0x11` | `UAV_STATE` | `<iih>`，10字节 | 10 Hz接收并原样转发地面站 |
| `0x20` | `UAV_EVENT` | `<BI>`，5字节 | 立即ACK、去重并转发地面站 |

整帧长度为21字节固定开销加payload，因此`UAV_READY`为28字节、`ACK`为25字节、
`UAV_STATE`为31字节、`UAV_EVENT`为26字节。

## 2. 一键启动

小车在等待阶段接收约2 Hz的`UAV_READY`。按键后：

1. 生成非0的`session_id`。
2. 发送`CAR_START`，flags为`ACK_REQUIRED|EVENT = 0x05`。
3. 未收到匹配ACK时每100～120 ms重发完全相同的帧，session和seq不变。
4. 收到`ACK(acked_type=0x03, acked_seq=CAR_START.seq, result=0)`后停止重发。
5. 以10 Hz发送同一session的`CAR_STATE`。

## 3. 无人机坐标

`UAV_STATE` payload：

```c
<iih
x_mm:i32
y_mm:i32
z_mm:i16
```

坐标是无人机T265以H点为原点的世界坐标。该帧不再包含`phase`、`vx/vy`、
`vision_quality`或`flags`。小车不修改payload，直接转发。超过500 ms未收到新帧时，
地面站把坐标标为“已过期/链路异常”。

## 4. 无人机阶段事件

`UAV_EVENT` payload：

```c
<BI
phase:u8
elapsed_ms:u32
```

无人机每次阶段变化立即发送，frame flags为`ACK_REQUIRED|EVENT = 0x05`。小车先回复
ACK，再按`source + session_id + type + seq`去重并转发；重复事件仍回复ACK，但不重复
改变显示状态。

任务一会用到的主要阶段：

| phase | 地面站文字 |
|---:|---|
| `2` | 就绪 |
| `3` | 起飞 |
| `4` | H点悬停 |
| `5` | 拦截 |
| `18` | 搜索目标 |
| `6` | 伴飞 |
| `8` | 投放下降 |
| `7` | 抛投 |
| `12` | 返回H |
| `13` | 降落 |
| `14` | 完成 |

没有独立故障事件，也没有独立抛投、触地或完成消息；全部统一为`UAV_EVENT`。

## 5. 小车发送部分

- `CAR_STATE`仍可保持13字节payload、整帧34字节、10 Hz，供旧状态兼容。
- `state_flags.bit4`继续表示编码器速度有效。
- `CAR_STATE.seq`每帧递增并允许`65535→0`。
- 小车速度当前只供无人机记录，不参与飞行控制。
- 两端后续统一速度坐标系，当前不增加坐标转换。

新增`CAR_POSITION (0x13)`，从小车程序启动起以10 Hz发送：

```text
<iiHH
car_x_mm:i32
car_y_mm:i32
car_pose_age_ms:u16
flags:u16
```

任务建立前使用`session_id=0`；`CAR_START`收到ACK后切换为同一非零session并可重新从
seq 0开始。flags bit0=`CAR_POSE_VALID`、bit2=`CAR_POSE_FRESH`、bit3=`SESSION_VALID`。

小车每收到一帧新的`UAV_STATE`，向地面站发送一帧`FUSED_POSITION (0x12)`；该消息
不发给无人机。payload为`<iiiihHIHH>`，28字节，整帧49字节。

## 6. 联调验收

1. 小车看到连续`UAV_READY`后使能按键。
2. 按键发送`CAR_START`并收到匹配ACK。
3. 无人机红灯并初始化T265；桌面测试使用`DRONE_DRY_RUN=1`。
4. 小车保持10 Hz发送`CAR_STATE`和`CAR_POSITION`。
5. 无人机诊断显示收到`type=19`，且`position_rejected=0`。
6. 小车收到10 Hz `UAV_STATE`，坐标随无人机移动发生变化。
7. 小车为每帧新`UAV_STATE`生成一帧`FUSED_POSITION`发往地面站。
8. 模拟`UAV_EVENT`，确认小车逐帧ACK、去重并转发。
