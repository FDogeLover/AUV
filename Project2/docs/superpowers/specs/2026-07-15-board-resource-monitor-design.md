# 板载资源监控（CPU/内存/温度）设计

日期：2026-07-15
模块：`drone_control/circle_pole`

## 背景/目的

circle_pole阶段2在ubuntu-pi上同时跑视觉识别(pole_vision线程)、雷达串口监听(Lradar线程)、T265 SDK(内部线程)、主控制循环，都在同一个Python进程里。目前完全没有板子负载数据，任务运行中CPU/内存/温度是否吃紧、哪个飞行阶段（比如视觉密集识别 vs 环绕控制）负载更高，只能靠猜。需要一份轻量的负载记录，跟现有`flight_data.jsonl`时间轴对齐，事后分析时能直接叠加看。

## 架构

新增独立模块 `Lcode/resource_monitor.py`，提供 `ResourceMonitor` 类：

- `start(log_file, log_lock)`：启动一个daemon线程，立即返回，不阻塞调用方
- `stop()`：设置停止标志，并`join(timeout=2.0)`等待线程真正退出。`stop_all()`只在任务结束时调一次，不在实时控制路径上，阻塞至多2秒没有性能代价，但能保证`stop()`返回后线程已经不会再碰`_log_file`，避免调用方紧接着`close()`文件时跟仍在跑最后一轮采样的线程产生"写已关闭文件"的竞态（虽然有try/except兜底不会崩溃，但没必要留这个粗糙点）

`Mission_GPT.py`里跟现有`self._log_file`生命周期对称接入：
- `start()`方法（打开`flight_data.jsonl`+起`self.loop`线程处）：额外创建`ResourceMonitor`实例并调用`start(self._log_file, self._log_lock)`
- `stop_all()`方法（关闭`_log_file`处）：额外调用`resource_monitor.stop()`，且必须在关闭`_log_file`**之前**调用（避免monitor线程写入已关闭的文件句柄）

## 采样内容与频率

1秒/次，用`psutil`：

| 字段 | 含义 | 来源 |
|---|---|---|
| `cpu_percent_sys` | 系统总CPU使用率(%) | `psutil.cpu_percent(interval=None)` |
| `cpu_percent_proc` | 本Python进程CPU使用率(%) | `psutil.Process(os.getpid()).cpu_percent(interval=None)` |
| `mem_percent_sys` | 系统内存占用率(%) | `psutil.virtual_memory().percent` |
| `mem_used_mb_sys` | 系统已用内存(MB) | `psutil.virtual_memory().used / 1024 / 1024` |
| `mem_rss_mb_proc` | 本进程RSS内存(MB) | `psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024` |
| `cpu_temp_c` | CPU温度(℃) | `psutil.sensors_temperatures()['pvt'][0].current`，取不到时为`None` |

`cpu_percent`类接口首次调用需要一次"热身"调用（返回值无意义，直接丢弃），线程启动后立即调用一次再进入采样循环。`psutil.Process(os.getpid())`必须只在线程启动时创建一次并复用同一个实例（存成`self._proc`）——如果每次采样都新建`Process`对象再调`cpu_percent()`，该接口内部靠同一实例前后两次调用的时间差计算百分比，每次新建实例会导致读数一直是0。

进程级选择理由：视觉/雷达/T265都是本进程内的线程（非独立进程），本进程CPU%已经能回答"是circle_pole自己在吃CPU还是板子上其他东西"；线程级拆分需要额外读`/proc/[pid]/task/`并且C扩展线程（cv2/pyrealsense2内部）大概率无法映射到有意义的名字，复杂度明显高于收益，本次不做。

## 写入格式

复用现有的`flight_data.jsonl`，跟已有的`{"event": "task_start"}`事件行同一种模式，新增一行：

```json
{"event": "resource", "t": 1752566400.123, "cpu_percent_sys": 62.3, "cpu_percent_proc": 41.0, "mem_percent_sys": 55.2, "mem_used_mb_sys": 890.5, "mem_rss_mb_proc": 210.3, "cpu_temp_c": 46.3}
```

分析时按`event`字段区分资源行和飞行遥测行，`t`字段跟位置数据的`t`字段是同一时间基准（`time.time()`），可以直接按时间戳对齐叠加分析。

## 并发安全

主循环线程（写位置遥测）和资源监控线程都会写同一个`self._log_file`文件对象。新增一个`self._log_lock`（`threading.Lock()`，在`Mission_GPT.__init__`里创建），所有现有的`_log_file.write()+flush()`调用点都要改为在这个锁保护下执行，避免两个线程的`write()`调用交错导致行内容被打断（同一行JSON被拆成两半）。现有写入点共7处（`start()`任务启动行、`takeoff()`、`navigate()`里的悬停避让分支/T265丢失分支/主日志块共3处、`_log_approaching_telemetry()`、`land()`），加上新增的资源监控写入，一共8处都要用同一把锁。

## 错误处理

- 单次采样过程整体包一层`try/except Exception: pass`，某次`psutil`调用失败（比如`sensors_temperatures()`在某些内核/权限下抛异常）跳过这一轮，不重试、不影响下一轮和主飞行循环，风格上跟现有飞行日志写入的容错方式一致
- `stop()`只是设标志位，线程最多再多循环一次(至多1秒)才会真正退出，不需要精确同步

## 测试

新增单元测试`test_resource_monitor.py`，mock `psutil`验证：
- 采样字段完整性（各字段类型/存在性）
- 线程能正常启停（`start()`后能采样，`stop()`后不再新增采样）
- `psutil`调用抛异常时不崩溃、不影响后续采样
- 写入格式包含`"event": "resource"`

不需要真机验证具体数值（数值本身是psutil保证正确性，我们只验证集成逻辑），真机测试时用现有测试流程顺带观察`flight_data.jsonl`里`event=="resource"`的行是否按预期频率出现即可。

## 范围外（不做）

- 线程级CPU拆分（见"采样内容"里的理由）
- 磁盘IO/网络IO监控（当前不涉及高频磁盘写入或网络传输，需要时再加）
- 独立的监控进程/脚本（选择同进程集成，理由见架构部分）
- 历史数据可视化/告警（这次只做采集，不做展示层）
