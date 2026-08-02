# D题无人机统一自启动

## 开机顺序

RDK由`competition-2026-d-autostart.service`启动唯一的
`competition_2026_d.auto_start`进程。任务一和任务二不得分别配置为自启动，避免
`/dev/bt_serial`、Cyber Camera UART、飞控UART、舵机和RGB灯发生多进程争用。

统一入口依次完成：

1. 白灯：锁定投放舵机，启动蓝牙、Cyber Camera双向链路和飞控命令/监听线程。
2. 等待OLED显示`CAM>RDK:OK`与`RDK>CAM:OK`。
3. 等待人工拔插T265。这里只运行`lsusb`检查：
   - `03e7:2150`：需要拔插；
   - `8087:0b37`：USB枚举就绪；
   - 在收到CAR_START之前不构造T265对象、不启动管线、不校零。
4. 所有共享预检通过后，以`task_mask=0x03`广播`UAV_READY`。
5. 收到`CAR_START.task_mode`后：任务一亮绿灯1秒，任务二亮蓝灯1秒。
6. 再次核验所有门禁，先点亮红灯，再ACK CAR_START。
7. ACK后才构造T265并启动管线；T265校零/置信度检查与红灯5秒警示并行。
8. `task_mode=1`运行1.2m任务一联合投放；`task_mode=2`运行1.3m任务二视觉动态降落。

## 故障行为

- CAR_START之前失败：进程以失败退出，systemd两秒后恢复。
- CAR_START已经ACK后失败：执行安全停止并以正常状态退出，阻止systemd重新武装。
- 任务成功完成：正常退出，不重新进入等待启动状态。
- 等待任务期间共享预检连续失效45秒：在未ACK的前提下退出并恢复。
- 无效flags、零session或未知任务ID：回复NAK，不启动任务。
- 重复CAR_START：只幂等回复ACK，不重复起飞。

## systemd

服务文件：

```text
/etc/systemd/system/competition-2026-d-autostart.service
```

常用只读检查：

```bash
systemctl is-enabled competition-2026-d-autostart.service
systemctl status competition-2026-d-autostart.service --no-pager
journalctl -u competition-2026-d-autostart.service -b --no-pager
```

当前服务使用`Restart=on-failure`。不要改成`Restart=always`，否则成功降落或任务内故障后可能再次进入待启动状态。

## 首次验证

首次开机验证应保持不装桨或使用`DRONE_DRY_RUN=1`，依次确认：Cyber Camera双OK、
T265拔插、`UAV_READY`双任务掩码、任务颜色、红灯、CAR_START ACK和任务分派。未经新的
飞行安全确认，不应直接用systemd入口进行带桨测试。
