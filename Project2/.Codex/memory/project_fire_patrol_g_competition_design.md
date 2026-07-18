---
name: project-fire-patrol-g-competition-design
description: "2023电赛G题(空地协同智能消防系统)无人机侧实现drone_control/fire_patrol/，已合并main并同步板子，已做5次真机飞行测试，发现HOVER_DROP后高度不恢复+疑似yaw导致的xy耦合两个待查问题"
metadata: 
  node_type: memory
  type: project
  originSessionId: 9cbf2827-69dc-4fd6-95f0-f8c8119b71d6
---

2026-07-16新建`drone_control/fire_patrol/`（基于`drone_control/basic/`），实现2023电赛G题"空地协同智能消防系统"无人机侧：6列x5行格心弓字形全覆盖巡逻、下视摄像头红色火源检测（全程仅触发一次）、视觉伺服悬停对准、HOVER_DROP闭环位置锁定、警示LED/抛投占位接口、无人机→消防车UART广播协议。设计文档`docs/superpowers/specs/2026-07-16-fire-patrol-design.md`，实现计划`docs/superpowers/plans/2026-07-16-fire-patrol.md`，已走完brainstorming→writing-plans→subagent-driven-development全流程，代码审查发现的4个问题(router.txt编码/续飞误跳过/悬停高度判据/外设异常保护)+用户追加的HOVER_DROP闭环位置锁定需求均已修复，已合并进`main`并push，已同步到ubuntu-pi板子(`~/Desktop/FJJ/fire_patrol/`)。

**关键范围决策**：消防车(4轮小车)完全不在本仓库范围内，只做无人机侧；不用K230做火情识别，改成树莓派/板子本地跑OpenCV颜色阈值检测。

**坐标系约定**：本地坐标原点(0,0,0) = T265上电点 = 巡逻网格第1个格心(row0/col0)，物理上要求起降点跟第一个格心重合，不需要额外的网格偏移量标定。

**Why**：赛题要求无人机在40dm×48dm区域全覆盖巡逻+发现火情后抛投灭火包+坐标广播给消防车，消防车是完全独立硬件项目，跟这个Python上位机代码库无关联。

**How to apply**：以后改这个版本时，参数继承按`basic`保守值(不是`basic_radar`/`circle_pole`调优值)，因为HOVER_DROP阶段要稳定性优先于速度。

**2026-07-16下视摄像头硬件方案已确认并落地**：板子确认只有两颗摄像头(USB `/dev/video0`给circle_pole前视用，IMX219 CSI `/dev/video10`)，用户决定fire_patrol下视复用IMX219。`Lcode/fire_vision.py`已重写成基于板载专用封装`Lcode/rdk_imx219_jupyter_preview.VisionSystem`(从板子`Desktop/IMX219/`完整版复制进仓库纳入版本控制)，不再是标准`cv2.VideoCapture`，`detect_fire()`/`SmoothedFireDetector`纯逻辑不变。已合并main、push、同步板子，板载40/40测试通过。详情见[[project_imx219_camera_bringup]]。

**2026-07-16 APPROACH视觉伺服符号bug已修复(真机测试前发现，非试飞试出来)**：用户物理确认下视摄像头安装朝向(画面上边=+y、右边=+x，无镜像)，据此推导出`_do_approach()`原实现的vy符号是反的(dy_px直接同号会正反馈发散，同类问题这个项目yaw方向也出过)。已修复(vy改成`-dy_px`)并补充`TestApproachSignConvention`测试锁定这4个符号关系。真机测试前必须重新读这段代码确认没被误改回去。

**2026-07-16新增LED/按键GPIO基础外设**：`Lcode/gpio_led.py`(`set_rgb_led`)+`Lcode/gpio_button.py`(`GpioButton`)，参照`Desktop/GPIO测试/LED测试.ipynb`+`按键测试.ipynb`验证过的接线(`Hobot.GPIO`+BCM编号，R=23/G=25/B=24，按键=17)。`warn_led()`已接入真实LED，板载实测确认真的能点亮。**按键目前只是基础驱动，没有接入业务逻辑**——用户明确按键用途是"一键起飞"，但要等"后续需要的时候"才实现具体接线(比如把`main.py`直接调用`mission1.start()`改成等`GpioButton.was_pressed()`才触发)，当前会话不做，下次会话如果要做这个不用重新问用途。

**2026-07-16三次真机飞行测试记录**：
- **第1次**：起飞→巡逻→在起点附近误检测(小车红色矩形)触发APPROACH(6秒内成功对准，验证符号修复有效)→HOVER_DROP→续飞，但恢复PATROL后剩余10个航点全部被"6.5秒超时"卡死强制跳过(继承自`basic`的`arrival_timeout_max`对4m网格完全不够用)。发现后修复：`arrival_timeout_max`改到25s、`detect_fire()`加圆度过滤排除矩形误检测、警示LED点亮后一直不关(finish_hover_drop_and_resume里补关灯)、起飞前红灯示警2秒。
- **第2次**：航点9附近用户判断"要飞出地图边界"手动介入降落(飞行日志显示的骤降是人工按下去的物理特征，不是硬件高度失控)。根因确认是T265当时正对墙、视觉特征不足。修复：**不改代码**，只把`router.txt`整体旋转90度(新x=旧y,新y=-旧x)换个起飞朝向避开正对墙，网格大小不变(4m×3.2m)。此次也修复了`Serial_ground`打不开导致`main()`直接崩溃的问题(try/except降级)。
- **第3次**：**全流程成功**——10个正常巡逻航点全部真实到达确认(6~18秒/段，`arrival_timeout_max=25s`生效)、高度全程可控(0.16~1.66m，无失控)、姿态正常(最大roll-5.4°/pitch5.6°，远低于30°保护)、航点9(上次出问题的路段)顺利通过、火情检测触发在真实途中位置(不在起点)、HOVER_DROP/续飞/返回原点/分段降落到0.2m/最终降落上锁全部按设计完成，进程正常退出。**唯一不完美**：APPROACH这次10秒超时兜底、没能真正对准(第1次是6秒内成功对准的)。
- **第3次测试后用户提出4个待办**（已处理3、4，1、2需要下次真机测试的新日志数据才能诊断）：
  1. 火情应该在巡航中就被识别到，结果是返程才识别——怀疑弓字形车道中心线可能没真正覆盖到火源实际位置(摄像头FOV/车道宽度假设未必成立)，纯属假设，未验证
  2. APPROACH视觉伺服"越修越偏"的发散趋势——怀疑圆度过滤太严格导致检测结果时有时无、或增益/死区需要调，未验证
  3. **已实现**：巡航航点精度放宽换速度，只有最后`PRECISION_TAIL_WAYPOINTS`(=2)个航点(回原点+降到0.2m，为land()做准备)保留严格精度——新增`CRUISE_XY_THRESH`/`CRUISE_Z_THRESH`(0.4m)
  4. yaw漂移(巡航中最大到23.8°，起始3°)——**已用飞行数据证实**是`docs/known_issues.md`问题16"yaw修正回路默认禁用"的预期后果，不是新bug，navigate()本来就不主动修正yaw
- **为诊断1、2补充的日志**（下次真机测试才会真正产生数据）：`navigate()`的PATROL分支新增"PATROL_DETECT"节流日志(记录每帧`dx_px`/`dy_px`/位置)，`_do_approach()`新增"APPROACH"节流日志(记录`dx_px`/`dy_px`/`vx`/`vy`/是否已对准)，独立于主日志节流时间戳。

**2026-07-16第4/5次真机测试+用数据证伪"APPROACH发散"假设**：
- 第4次：全流程成功(12航点全部真实到达，含新的`arrival_timeout_max=25s`验证)，但**全程2413帧检测0次命中**，没摆火源或摆了但完全没识别到。
- 第5次：摆了真实火源，触发APPROACH，但**用PATROL_DETECT/APPROACH新日志证实**：全程2351帧只有1帧命中，进入APPROACH后头4帧还能检测(偏移在变大)，第5帧起直接检测丢失、`vx=vy=0`一路到10秒超时——**根本不是"伺服越修越偏"，是检测本身极脆弱（命中率约1/2000帧），触发后几乎立刻又跟丢，全程零修正**。用触发时存的快照(`fire_debug/*_fire_triggered.jpg`)反算发现该帧面积14177/圆度0.790远高于阈值(200~200000/0.5)，说明当前HSV/面积/圆度阈值本身不是瓶颈，怀疑是`exposure=2645`慢门(为台架弱光调的)在巡航移动时造成运动模糊。**已加`PATROL_MISS_SNAPSHOT_INTERVAL_S`(5s)节流对照存图**(`reason="patrol_no_detect"`)，下次测试能直接拿"检测到清晰图"vs"检测不到模糊图"做对比。
- **第5次还发现一个新的、未解决的严重问题**：HOVER_DROP结束恢复PATROL后，软件正确把`height_setpoint_cm`提回160并保持，但**实测高度卡在0.91~0.96m整整24秒完全没有响应爬升指令**，直到航点超时才继续；之后正常下降阶段高度又能正常跟踪。像是飞控/凌霄IMU在HOVER_DROP低高度流程后某种内部状态没复位，不是Python侧问题(`_ramp_z_cm`确认算对发对了)。**未解决，下次测试要重点看这段**。
- **疑似yaw导致xy轴耦合**：长直线航段(如`idx=9`目标x=3.2不变、y扫0→-4.0)时，x会随着飞行"鼓包"到0.2~0.3m偏差再收敛，不是单调漂移。假说：`set_speed()`算出的`vx/vy`是按世界系PID算的，但如果飞控按机体系执行且yaw非0(yaw修正回路本就默认关闭，见问题16)，纯y方向指令会被无意间带出正比于`sin(yaw)`的x分量，量级(yaw~10°时约7cm/s)能对上观测到的偏移。**建议修法：不碰yaw本身(避免重蹈问题16覆辙)，而是发送前用当前yaw把vx/vy反向旋转补偿**——用户还没决定要不要现在做，只是记录下来。
- **已实现并验证的改动**：巡航航点改"掠过式"到达判定(`is_precision_waypoint`分支)——进入`CRUISE_XY_THRESH`范围立刻切下一个目标，不再等滑动窗口确认+停留观察，只有最后`PRECISION_TAIL_WAYPOINTS`个精确航点保留严格流程；顺带修复了`_on_arrival()`没有同步重置`arrival_start_time`导致"掠过"当次调用被尾部超时检查误判、一次调用连跳两个航点的bug。
