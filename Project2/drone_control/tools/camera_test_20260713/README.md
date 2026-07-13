# 下置IMX219摄像头(CSI)测试留档 — 2026-07-13

ubuntu-pi(地瓜派RDK X5) cam1槽位接的imx219，硬件排查+取流验证过程中留下的测试图/视频。用途是"辅助定位"(识别地面标记/形状)，不是识别红绿颜色。

## 文件说明(按拍摄顺序)

| 文件 | 说明 |
|---|---|
| `01_gain0_default_dark.png` | 排线接通后第一次成功拍到画面，`analogue_gain`默认值0，几乎全黑，只能隐约看出轮廓——证明硬件通了，不代表能用 |
| `02_gain0_exposuremax_still_dark.png` | 曝光时间调到最大(1759)但增益仍是0，亮度几乎没变——说明暗的根源是增益不是曝光时间 |
| `03_gain80_before_whitebalance.png` | 把`analogue_gain`调到80后画面正常亮了，但去马赛克后有明显绿色偏色(未做白平衡) |
| `04_gain80_after_whitebalance_BEST.png` | 在03基础上做灰世界白平衡校正——**目前最好的参考结果**，红色包装袋/绿色指示灯颜色基本正常 |
| `05_gain80_repeat_confirm.png` | 隔了几十分钟重新拍一张，验证"每次抓取前重新显式设一次增益"这个流程可重复 |
| `06_burst_video_DARK_gainreset_bug.mp4` | 连续60帧的测试视频，画面是黑的——**已知原因**：这批帧抓取前没有重新显式设置增益，`v4l2-ctl --get-ctrl`显示的"上次设置值"不代表这次streaming会话真的把寄存器写回了传感器。不是画质参考，是用来记录这个坑的。 |
| `07_front_usb_camera_test.jpg` | 前置1080p USB摄像头(`/dev/video0`)测试照片，标准UVC设备+MJPG格式，自带ISP(自动曝光/白平衡)，直接可用，不需要下面这些下置摄像头才有的坑 |
| `09_blacklevel_gamma.png`~`12_trimmed_wb.png` | 排查"发蓝"问题过程中的中间产物：黑电平校正+灰世界/ROI/去极值白平衡等多种算法都试过，画面依然发蓝发青——后来确认是拜耳排列解码错误，跟这几种白平衡算法本身无关(见13~16号) |
| `13_ground_raw_nowb.png` | 摄像头对准地面(木纹地板+置物架)实拍，未做任何白平衡/黑电平处理的debayer原图，用`BayerRG2BGR`(错误排列)，木纹发绿发蓝 |
| `14_ground_wb.png` | 同一张raw用黑电平+白平衡+伽马处理，但仍是`BayerRG2BGR`(错误排列)，木纹依然发蓝——证明问题不在白平衡算法 |
| `15_pattern_{RG,GR,BG,GB}.png` | 同一张raw分别用4种拜耳解码顺序跑debayer(不做白平衡)对比：RG/BG看起来还算自然(木纹偏冷/偏暖)，GR/GB明显错误(发紫红)；BG的木纹暖色调最接近肉眼记忆中的真实颜色 |
| `16_ground_BGpattern_wb.png` | **确认修复**：改用`cv2.COLOR_BayerBG2BGR` + 黑电平校正(raw四通道各自算，不在debayer后的BGR上算) + 伽马，木纹墙板呈现正常暖木色，效果最好 |

## 关键结论(供以后正式开发视觉定位代码时参考)

**0. （2026-07-13新增，最重要）拜耳解码顺序用错了，之前"发蓝"不是白平衡的问题**：
驱动/v4l2-ctl上报的格式叫`RG10`(fourcc)，但实测该硬件+驱动实际的物理拜耳排列要用`cv2.COLOR_BayerBG2BGR`解码才是对的颜色，用fourcc名字暗示的`BayerRG2BGR`会导致R/B通道错位，不管怎么调白平衡/黑电平/伽马，暖色调物体(木纹、皮肤)都会发蓝发青。判断方法：同一张raw数据分别跑4种`cv2.COLOR_Bayer{RG,GR,BG,GB}2BGR`，肉眼挑一个让已知暖色/中性色物体显示正常的（错误的排列里有明显偏色，尤其GR/GB两种如果搞错G通道位置会发紫红，一眼能看出来）。以后debayer代码要用`BayerBG2BGR`，不要想当然照着fourcc名字选。

1. **`analogue_gain`默认是0，必须手动设置**（本次用的是80，具体数值要按现场光线重新试）：
   ```
   v4l2-ctl -d /dev/v4l-subdev1 --set-ctrl=analogue_gain=80
   ```
2. **每次新开一个抓取/streaming会话，都要重新显式设一次增益**，不能假设上次设过的值这次还生效——正确做法是让流持续开着、只在开流那一刻设一次，而不是每次抓一帧就重新开关设备。
3. **去马赛克+白平衡在软件端做**（OpenCV），不依赖厂商ISP自动管线——官方Python demo脚本(`/app/pydev_demo/08_mipi_camera_sample/`)和C示例(`sample_vin/get_vin_data`)都因为"board_config.json声明了2个摄像头槽位但只有cam1真正接了硬件"这个自动检测逻辑卡住，没能跑通厂商这套自动ISP流程。
4. **实测原始格式(RG10)在终端节点直接设分辨率不会真的让传感器切换工作模式**——试过设640x480，实际还是按原生3264x2464在跑（帧率因此只有约3~5fps）。要真正拿到高帧率的低分辨率，需要让imx219**传感器本身**切到对应工作模式（C示例的`-s 28`~`-s 32`这几个索引就是绑定好的imx219各分辨率配置，但底层用的是另一套驱动栈`hbn_vnode`，跟现在这套`v4l2_enable`+`vs-`前缀管线不兼容，两边协调需要进一步排查或官方支持）。
5. **硬件通电配置固定命名**：`/etc/board_config.json`声明cam1对应`mipi_host:2`；`/boot/config.txt`需要同时有`v4l2_enable`+`v4l2_scene=22`+`dtoverlay=dtoverlay_cam1_imx219`三行(改完需重启)，缺任何一行都会导致要么摄像头识别不到、要么引脚冲突报错(`pin lsio_spi0_ssn already requested`)。
