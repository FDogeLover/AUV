"""
K230 动物识别推理代码 (YOLOv8n, 320×320)
==========================================
基于 K230 官方 yolov8n_obb.py 示例格式
对齐官方示例：320×320 输入、不使用 ALIGN_UP、不覆写 preprocess()

模型：YOLOv8n — 320×320 输入，5 类动物检测
类别：tiger / wolf / monkey / peacock / elephant

使用方法：
    将 animal_yolov8n_320.kmodel 放入 /sdcard/examples/kmodel/
    通过 CanMV IDE 运行本脚本
"""

from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
from media.sensor import *
from machine import UART, FPIOA


class TemporalFilter:
    """时序平滑滤波器：对每类检测数量做滑动窗口平均，消除逐帧波动"""
    def __init__(self, window=5):
        self.window = window
        self.buf = []

    def update(self, raw_counts):
        self.buf.append(raw_counts)
        if len(self.buf) > self.window:
            self.buf.pop(0)
        smoothed = {}
        for c in self.buf:
            for k, v in c.items():
                smoothed[k] = smoothed.get(k, 0) + v
        for k in smoothed:
            smoothed[k] = int(round(smoothed[k] / len(self.buf)))
        return smoothed


class AnimalDetectApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.5, nms_threshold=0.2,
                 rgb888p_size=[640, 480], display_size=[800, 480], debug_mode=0,
                 crop_w_ratio=1.0, crop_h_ratio=1.0,
                 detect_crop_enable=False, detect_crop_w_ratio=0.6, detect_crop_h_ratio=0.6):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [rgb888p_size[0], rgb888p_size[1]]
        self.display_size = [display_size[0], display_size[1]]
        self.debug_mode = debug_mode
        self.crop_w_ratio = crop_w_ratio
        self.crop_h_ratio = crop_h_ratio
        self.detect_crop_enable = detect_crop_enable
        self.detect_crop_x = 0
        self.detect_crop_y = 0
        self.detect_crop_w = rgb888p_size[0]
        self.detect_crop_h = rgb888p_size[1]
        self._dmap_ox = 0.0
        self._dmap_oy = 0.0
        self._dmap_sx = 1.0
        self._dmap_sy = 1.0
        if detect_crop_enable:
            sw, sh = rgb888p_size[0], rgb888p_size[1]
            self.detect_crop_w = int(sw * detect_crop_w_ratio)
            self.detect_crop_h = int(sh * detect_crop_h_ratio)
            self.detect_crop_x = (sw - self.detect_crop_w) // 2
            self.detect_crop_y = (sh - self.detect_crop_h) // 2
            dw, dh = display_size[0], display_size[1]
            self._dmap_ox = self.detect_crop_x * dw / sw
            self._dmap_oy = self.detect_crop_y * dh / sh
            self._dmap_sx = self.detect_crop_w / sw
            self._dmap_sy = self.detect_crop_h / sh
        self.color_four = get_colors(len(self.labels))
        self.scale = 1.0
        # Ai2d 实例，用于实现模型预处理
        self.ai2d = Ai2d(debug_mode)
        # 设置 Ai2d 的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作：letterbox pad + resize
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1, 3, self.model_input_size[1], self.model_input_size[0]])

    @staticmethod
    def _parse_dets(dets):
        """单次遍历检测结果，返回 (counts, max_conf)"""
        counts = {}
        max_conf = {}
        if dets:
            for i in range(len(dets[0])):
                label_id = dets[1][i]
                score = float(dets[2][i])
                counts[label_id] = counts.get(label_id, 0) + 1
                if label_id not in max_conf or score > max_conf[label_id]:
                    max_conf[label_id] = score
        return counts, max_conf

    @staticmethod
    def _resolve_best(counts, max_conf):
        """多种类冲突时取最高置信度的类，返回 (best_id, show_counts)"""
        if not counts:
            return None, {}
        if len(counts) > 1:
            best_id = max(counts, key=lambda k: max_conf.get(k, 0))
            return best_id, {best_id: sum(counts.values())}
        best_id = next(iter(counts))
        return best_id, dict(counts)

    # 获取检测结果统计（用于 UART 发送）
    def get_uart_data(self, dets):
        _, show_counts = self._resolve_best(*self._parse_dets(dets))
        return show_counts

    # 不覆写 preprocess()，使用 AIBase 基类默认实现

    # YOLOv8 后处理
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num
            )
            return det_res

    # 绘制检测框和标签
    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()
            if self.crop_w_ratio < 1.0 or self.crop_h_ratio < 1.0:
                _draw_crop_cover(pl.osd_img, self.crop_w_ratio, self.crop_h_ratio)
            counts, max_conf = self._parse_dets(dets)
            best_id, show_counts = self._resolve_best(counts, max_conf)
            if dets and best_id is not None:
                filtered_counts = {}
                for i in range(len(dets[0])):
                    label_id = dets[1][i]
                    if label_id != best_id:
                        continue
                    x, y, w, h = map(lambda v: int(round(v, 0)), dets[0][i])
                    if self.detect_crop_enable:
                        dw = self.display_size[0]
                        dh = self.display_size[1]
                        cx = x + w / 2.0
                        cy = y + h / 2.0
                        margin_x = (dw - dw * self._dmap_sx) / 2.0
                        margin_y = (dh - dh * self._dmap_sy) / 2.0
                        if cx < margin_x or cx > dw - margin_x or cy < margin_y or cy > dh - margin_y:
                            continue
                    filtered_counts[label_id] = filtered_counts.get(label_id, 0) + 1
                    score = dets[2][i]
                    pl.osd_img.draw_rectangle(x, y, w, h, color=self.color_four[label_id], thickness=4)
                    label_text = " " + self.labels[label_id] + " " + str(round(score, 2))
                    pl.osd_img.draw_string_advanced(x, y - 50, 24, label_text, color=self.color_four[label_id])
                text = ""
                show = filtered_counts if self.detect_crop_enable else show_counts
                for j in range(len(self.labels)):
                    if show.get(j, 0) != 0:
                        text += self.labels[j] + ": " + str(show[j]) + ";  "
                pl.osd_img.draw_string_advanced(50, 50, 24, text, color=[0, 255, 0])


def _draw_crop_cover(osd_img, w_ratio=0.6, h_ratio=0.6):
    """在OSD层绘制黑色遮罩，只保留中间区域可见"""
    if w_ratio >= 1.0 and h_ratio >= 1.0:
        return
    dw = osd_img.width()
    dh = osd_img.height()
    cw = int(dw * w_ratio)
    ch = int(dh * h_ratio)
    l = (dw - cw) // 2
    r = l + cw
    t = (dh - ch) // 2
    b = t + ch
    black = (1, 1, 1)
    osd_img.draw_rectangle(0, 0, dw, t, color=black, fill=True)         # top
    osd_img.draw_rectangle(0, b, dw, dh - b, color=black, fill=True)    # bottom
    osd_img.draw_rectangle(0, t, l, ch, color=black, fill=True)         # left
    osd_img.draw_rectangle(r, t, dw - r, ch, color=black, fill=True)    # right


if __name__ == "__main__":
    # ========== 显示模式 ==========
    display_mode = "hdmi"

    # ========== 裁剪显示：只保留中间区域 (1.0=全屏, 0.5=保留50%中心) ==========
    CROP_ENABLE = True
    CROP_W_RATIO = 0.6
    CROP_H_RATIO = 0.6

    # ========== 裁剪检测：AI只对中间区域推理 (独立于上面的视觉裁切) ==========
    DETECT_CROP_ENABLE = True
    DETECT_CROP_W_RATIO = 0.6
    DETECT_CROP_H_RATIO = 0.6

    # ========== 摄像头 AI 输入分辨率（对齐官方示例，不 ALIGN_UP） ==========
    rgb888p_size = [1920, 1080]

    # ========== 模型路径 ==========
    kmodel_path = "/sdcard/examples/mycode/animal_yolov8n_v2_best.kmodel"

    # ========== 动物类别标签（与训练一致） ==========
    labels = ["tiger", "wolf", "monkey", "peacock", "elephant"]

    # ========== 检测参数 ==========
    # 注：0.1 过低的置信度会导致大量误识别，建议 0.3~0.5
    confidence_threshold = 0.3
    nms_threshold = 0.5
    max_boxes_num = 30

    # ========== 时序平滑（减少逐帧波动） ==========
    # window 越大输出越稳，但响应延迟越大；5 帧约 0.2s 延迟
    SMOOTH_WINDOW = 5

    # ========== 模型输入尺寸 ==========
    model_input_size = [320, 320]

    # ========== UART 配置（串口通信，发送检测结果） ==========
    # 立创·庐山派 K230：UART2 有专用 GH1.25-4P 座子（免接线），最推荐
    #   UART2: TX→GPIO11, RX→GPIO12（标号"2"的座子）
    #   UART3: TX→GPIO50, RX→GPIO51（标号"3"的座子）
    #   UART1: TX→GPIO3,  RX→GPIO4（40PIN 排针）
    #   UART4: TX→GPIO36, RX→GPIO37（40PIN 排针）
    UART_ENABLE = True           # 设为 False 可关闭串口输出，不接串口线也能测
    UART_ID = UART.UART2        # 推荐 UART2（有独立座子），可选 UART1/UART3/UART4
    UART_BAUD = 115200
    UART_TX_PIN = 11            # 发送引脚（UART2→GPIO11, UART3→GPIO50, UART1→GPIO3, UART4→GPIO36）
    UART_RX_PIN = 12            # 接收引脚（UART2→GPIO12, UART3→GPIO51, UART1→GPIO4, UART4→GPIO37）
    _UART_FPIOA_MAP = {
        UART.UART1: (FPIOA.UART1_TXD, FPIOA.UART1_RXD),
        UART.UART2: (FPIOA.UART2_TXD, FPIOA.UART2_RXD),
        UART.UART3: (FPIOA.UART3_TXD, FPIOA.UART3_RXD),
        UART.UART4: (FPIOA.UART4_TXD, FPIOA.UART4_RXD),
    }
    if UART_ENABLE:
        fpioa = FPIOA()
        tx_func, rx_func = _UART_FPIOA_MAP[UART_ID]
        fpioa.set_function(UART_TX_PIN, tx_func)
        fpioa.set_function(UART_RX_PIN, rx_func)
        uart = UART(UART_ID, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    else:
        uart = None

    # ========== 初始化 PipeLine ==========
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create(sensor = Sensor(id=1,fps=30))
    display_size = pl.get_display_size()

    # ========== 初始化动物检测器 ==========
    animal_det = AnimalDetectApp(
        kmodel_path=kmodel_path,
        labels=labels,
        model_input_size=model_input_size,
        max_boxes_num=max_boxes_num,
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0,
        crop_w_ratio=CROP_W_RATIO if CROP_ENABLE else 1.0,
        crop_h_ratio=CROP_H_RATIO if CROP_ENABLE else 1.0,
        detect_crop_enable=DETECT_CROP_ENABLE,
        detect_crop_w_ratio=DETECT_CROP_W_RATIO,
        detect_crop_h_ratio=DETECT_CROP_H_RATIO,
    )
    animal_det.config_preprocess()

    # ========== 时序平滑滤波器 ==========
    smooth_filter = TemporalFilter(window=SMOOTH_WINDOW)

    print("=" * 50)
    print("  K230 Animal Detection (YOLOv8n 320x320)")
    print("  Model:", kmodel_path.split("/")[-1])
    print("  Classes:", labels)
    print("  Confidence:", confidence_threshold, " NMS:", nms_threshold)
    print("  Smooth window:", SMOOTH_WINDOW, "frames")
    print("  UART:", "UART{} @ {} baud (TX:GPIO{}, RX:GPIO{})".format(
        UART_ID, UART_BAUD, UART_TX_PIN, UART_RX_PIN) if UART_ENABLE else "Disabled")
    print("  Press Ctrl+C to stop")
    print("=" * 50)

    frame_count = 0
    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = animal_det.run(img)
                animal_det.draw_result(pl, res)
                # 获取原始检测计数，经时序平滑后通过 UART 发送
                raw_counts = animal_det.get_uart_data(res)
                smoothed = smooth_filter.update(raw_counts)
                smoothed["frame"] = frame_count
                if UART_ENABLE:
                    uart.write(ujson.dumps(smoothed) + "\n")
                pl.show_image()
                if frame_count % 30 == 0:
                    gc.collect()
                frame_count = (frame_count + 1) & 0x7FFFFFFF
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print("[ERROR]", e)
        import sys
        sys.print_exception(e)
    finally:
        animal_det.deinit()
        if UART_ENABLE:
            uart.deinit()
        pl.destroy()
        gc.collect()
        print("[INFO] Resources released.")
