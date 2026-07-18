---
name: project-imx219-camera-bringup
description: ubuntu-pi下置IMX219摄像头(CSI/cam1)硬件排查+取流已跑通，存在3套并行方案(手动v4l2-ctl/libsrcampy/rdk_imx219_jupyter_preview)，画面偏暗噪点重待调增益，同时用于circle_pole降落识别和fire_patrol火情检测
metadata: 
  node_type: memory
  type: project
  originSessionId: a330d6d5-aa3a-482f-b6e1-f30a4b335cbc
---

2026-07-13：给circle_pole阶段2以后的"视觉辅助定位"（识别地面降落标记形状，**不是**判断红绿杆塔颜色——那是前置摄像头/雷达的事）铺路，把ubuntu-pi(地瓜派RDK X5)下置IMX219摄像头从"完全识别不到"一路调通到"能稳定拍彩色照片"。测试图/视频存档在`drone_control/tools/camera_test_*`，详见同目录`camera_test_README.md`。

**硬件事实**：
- 摄像头型号imx219，接在CSI cam1槽位，对应`mipi_host:2`
- 板子上还有一颗USB摄像头(FHD, `/dev/video0`/`video1`)，跟imx219无关，两者不要搞混
- 前置的另一颗是1080p USB摄像头，具体FOV参数厂商没标注，需要自己棋盘格标定

**排查踩过的坑(按顺序)**：
1. I2C扫描(`i2cdetect`)全总线查不到设备 → 排线没插好，重插后能读到但整机检测仍失败
2. `/boot/config.txt`只写`dtoverlay=cam1_imx219`不够——正确文件名要带完整前缀`dtoverlay=dtoverlay_cam1_imx219`（源自`srpi-config`脚本里的命名规则，直接改config.txt容易漏这层）
3. overlay名字改对后，imx219驱动模块能加载但初始化报错`pin lsio_spi0_ssn already requested by soc:cam:vcon@2`——引脚复用冲突，根因是漏配了`v4l2_enable`+`v4l2_scene=22`这两行（`srpi-config`交互模式里"V4L2 sif-isp-vse"选项对应的配置，直接手改config.txt容易漏）
4. 三行配置(`v4l2_enable`/`v4l2_scene=22`/`dtoverlay=dtoverlay_cam1_imx219`)凑齐重启后，`media-ctl -d /dev/media1`能看到完整的imx219→ISP→VSE管线，原生格式3264x2464 SRGGB10
5. 官方Python demo(`/app/pydev_demo/08_mipi_camera_sample/02_mipi_camera_dump.py`)和C示例(`sample_vin/get_vin_data`，用`-s 28~32`选imx219各分辨率配置)都跑不通——根因是`/etc/board_config.json`声明了cam0/cam1两个槽位，但只有cam1真的接了硬件，官方的"自动检测"逻辑在只接一个摄像头时反而卡住（报错涉及`/dev/video2`/`video3`，那是cam0/host0对应的空槽位通道）；C示例还牵出另一套独立驱动栈`hbn_vnode`/`hbn_vflow`（`/sys/class/vps/mipi_hostN/status/cfg`不存在），跟`v4l2_enable`那套`vs-`前缀管线不是同一套，两边协调需要官方文档或支持，没有深入下去

**最终跑通的方案(绕开官方ISP自动管线，全部自己控制)**：
```bash
# 每次抓取前(或每次新开一个streaming会话前)必须重新显式设一次增益，
# 不能假设上次设过的值这次还生效——v4l2-ctl --get-ctrl看到的是"驱动记的值"，
# 不代表这次streaming真的把寄存器写回了传感器(真机验证过这个坑，同样的settings，
# 不重新set就是黑的，重新set就正常)
v4l2-ctl -d /dev/v4l-subdev1 --set-ctrl=analogue_gain=80  # 默认0，几乎全黑，这个数值要按现场光线试
v4l2-ctl -d /dev/video4 --set-fmt-video=width=3264,height=2464,pixelformat=RG10 \
  --stream-mmap --stream-count=1 --stream-to=/tmp/frame.rg10
```
然后Python(numpy+OpenCV)软件端做：
```python
raw = np.fromfile('frame.rg10', dtype='<u2').reshape((2464,3264))  # 值域已经是0~1023，不需要位移
# 关键：这里必须用BayerBG2BGR，不是BayerRG2BGR，见下面"已知限制"第一条
bgr16 = cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)  # 去马赛克
# 白平衡：按Bayer四通道(R/Gr/Gb/B)各自算黑电平(约51~64，四通道并不相同)校正后再算增益，
# 不要在debayer后的BGR图上直接算——debayer插值会让增益比例算出来失真(2026-07-13踩过)
```

**已知限制**：
- **拜耳排列不是fourcc名字暗示的顺序**：驱动/v4l2-ctl上报的pixelformat叫`RG10`，但实测`cv2.COLOR_BayerRG2BGR`解码出来R/B通道是错位的（正常暖色木纹墙拍出来发蓝发青，怎么调白平衡/黑电平/伽马都调不回来），2026-07-13用4种排列(RG/GR/BG/GB)横向对比debayer结果，确认`cv2.COLOR_BayerBG2BGR`才是这套硬件+驱动实际对应的正确解码。**这是长期误用的根源，不是白平衡算法或光照的问题**——之前"04号最佳结果"其实也是在错误拜耳排列下调出来的将就效果，肉眼看着还行是因为凑巧场景本身没有强烈色彩反差
- 终端节点(`/dev/video4`)直接设分辨率(如640x480)不会真的让imx219切换传感器工作模式，实测还是按原生3264x2464在跑，连续抓取只有约3~5fps——要真正拿到高帧率低分辨率，需要传感器本身切模式，这条路目前卡在C示例的驱动栈冲突上，未解决
- 灰世界白平衡(对全图算均值)容易被非中性色场景内容/过曝高光带偏，遇到强逆光(比如直接拍吸头灯)会算出夸张的增益导致中间调偏色——更稳的做法是黑电平校正后在raw四通道(而非debayer后的BGR)上算增益，且最好避开画面里大片过曝/深阴影区域，用去掉头尾百分位数的均值
- **严禁带电插拔MIPI摄像头**（官方FAQ原话），带电操作可能烧毁摄像头模组或主控MIPI接口

**2026-07-15发现更简单的取流方案，绕开了上面手动v4l2-ctl+Bayer解码这一整套麻烦**：用户提供参考代码`Desktop/视觉测试/设置.ipynb`，用`hobot_vio.libsrcampy.Camera()`高层API(`open_cam(index, -1, fps, width, height)`+`get_img()`)直接拿到已经过ISP处理的NV12帧，不需要`v4l2-ctl`手动设分辨率/格式、不需要自己debayer——库内部自动跑完整个`vin→isp→vse`流水线。真机验证：`CAMERA_INDEX=0`成功`Auto-selected sensor: imx219-1920x1080-30fps`，33/33帧全部取到，NV12→BGR(`cv2.COLOR_YUV2BGR_NV12`)解码正常，帧字节数与预期(1920×1080×1.5)完全吻合。**用途是circle_pole赛题"降落点视觉识别"（识别地面标记，不是判断红绿杆塔颜色）**，这是当前唯一实际推进的部分，其余(净空/单按键)详见[[project_circle_pole_vision_servo_stage2_design]]的赛题完成度核对。

**该方案已知问题**：`open_cam()`本身不设增益/曝光，实测拍出来的画面噪点很重、整体偏暗，隐约能看清物体轮廓但细节完全看不清——跟之前v4l2-ctl方案一样，需要现场根据实际光线手动调增益（那条`analogue_gain=80`经验值未必能直接套用到这套高层API，需要看`libsrcampy`有没有对应的增益设置接口，还没找）。**下次会话优先做**：调增益/曝光拍一张清晰照片，确认摄像头实际朝向/视野范围是否符合"识别地面降落标记"的需求。

**2026-07-16发现第三套并行方案，且发现用途已扩展到fire_patrol**：给[[project_fire_patrol_g_competition_design]]（G题空地协同消防系统，无人机侧`drone_control/fire_patrol/`）测试"下视摄像头能否正常启动拍摄"时，用户提供了`Desktop/IMX219/jupyter显示.ipynb`，指向另一套更完整的封装：`rdk_imx219_jupyter_preview.py`模块（`VisionSystem`/`IMX219VisionSystem`类），底层是专用C生产程序`/app/cdev_demo/v4l2/rdk_imx219_stream`(V4L2 MMAP)通过`/dev/video10`(VSE缩放节点)取流，`get_frame()`直接返回处理好的BGR numpy数组，**不需要**手动Bayer解码或`libsrcampy`——这是三套方案里封装程度最高、文档最完整的一套(`Desktop/IMX219/rdk_imx219_jupyter_preview_使用说明.md`)，还自带`enable_bright_detection`/`get_bright_spots()`/`check_bright_spot_in_zone()`亮点检测接口，专门适合火情检测场景。

**重要警告(文档原话)**：不能直接`cv2.VideoCapture("/dev/video10")`("当前 RDK 驱动在通用 OpenCV/V4L2 取流路径中可能超时")，必须走上述专用封装。

**模块有两份、不完全相同**：`/app/cdev_demo/v4l2/rdk_imx219_jupyter_preview.py`是旧的/不完整的(缺`VisionSystem`类定义，2026-07-16实测直接import会`AttributeError`)；`/home/sunrise/Desktop/IMX219/rdk_imx219_jupyter_preview.py`是完整可用的新版本。脚本应该在`Desktop/IMX219/`目录下运行(靠cwd被Python自动加入sys.path拿到正确版本)，不要手动把`/app/cdev_demo/v4l2`插到sys.path前面。

**2026-07-16两次台架测试结果**：`test_rdk_imx219_vision.py --mode bright`(10秒headless)和手动`get_frame()`都确认摄像头能正常启动拍摄，稳定~15.7fps，960x540 BGR帧正确。两次抓拍画面都偏暗/噪点重(默认曝光2645和调高到3174都一样)，跟本文件里"画面偏暗噪点重待调增益"是同一个老问题的第三次复现，不是新bug。第二次抓拍还发现台架测试时镜头实际对着的画面有异常的过曝弧形边界，怀疑是测试时镜头离某个物体/边缘太近，不是驱动问题。

**当前状态与关键待办**：`fire_patrol/Lcode/fire_vision.py`(2026-07-16实现)目前是按标准USB摄像头写的(`cv2.VideoCapture(device)`)，如果fire_patrol实际部署时下视摄像头用的是这颗IMX219(而不是另外接一颗USB摄像头)，这个模块在板子上会打不开摄像头——**需要重写成基于`rdk_imx219_jupyter_preview.VisionSystem`**。用户决定本次不深入调参，等实际飞行测试看效果，不理想再调曝光/增益。
