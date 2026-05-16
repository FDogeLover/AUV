"""
K230 动物识别推理代码 (YOLOv8n, 320×320) — Pi双向通信版
==========================================================
基于 K230 官方 yolov8n_obb.py 示例格式
对齐官方示例：320×320 输入、不使用 ALIGN_UP、不覆写 preprocess()

模型：YOLOv8n — 320×320 输入，5 类动物检测
类别：tiger / wolf / monkey / peacock / elephant

协议：
  Pi→K230: AA 10 grid_idx FF                       — 启动检测(4B, AA头+FF尾)
  K230→Pi: AA 20 grid_idx cls cnt total conf FF    — 检测结果(8B, AA头+FF尾)
  Pi→K230: AA 11 grid_idx FF                       — ACK确认(4B, AA头+FF尾)

使用方法：
    将 animal_yolov8n_320.kmodel 放入 /sdcard/examples/mycode/
    通过 CanMV IDE 运行本脚本
"""

from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, gc
import time
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import aidemo
from media.sensor import *
from machine import UART
import _thread

# ========== 协议常量 ==========
FRAME_HEAD   = 0xAA
CMD_START    = 0x10   # Pi→K230: 启动检测
CMD_ACK      = 0x11   # Pi→K230: ACK确认
CMD_RESULT   = 0x20   # K230→Pi: 检测结果
NO_ANIMAL    = 0xFF   # 无动物
FRAMES_PER_GRID = 30  # 每格采集帧数
CMD_LEN      = 4      # START/ACK 帧长度 (AA CMD idx FF)

# ========== 共享状态（线程安全） ==========
_state_lock = _thread.allocate_lock()
_shared = {
    "active": False,
    "grid_idx": 0,
    "tally": {},        # cls_id → 累加帧数
    "conf_sum": {},     # cls_id → 置信度累加
    "frame_cnt": 0,
    "retry_count": 0,   # 内部重测次数
}


def uart_rx_thread():
    """守护线程：接收Pi的 START / ACK 指令（AA...FF 对齐）"""
    buf = b''
    while True:
        try:
            chunk = uart.read()
            time.sleep_ms(1)                # 避免独占UART硬件，让主线程有机会调用write
            if chunk:
                buf += chunk
            while True:
                p = buf.find(bytes([FRAME_HEAD]))
                if p < 0:
                    buf = buf[-1:]
                    break
                if p > 0:
                    buf = buf[p:]
                if len(buf) < CMD_LEN:
                    break
                if buf[CMD_LEN - 1] != 0xFF:
                    buf = buf[1:]
                    continue
                cmd = buf[1]
                idx = buf[2]
                buf = buf[CMD_LEN:]

                with _state_lock:
                    if cmd == CMD_START:
                        print("<< START idx=", idx)
                        _shared["active"] = True
                        _shared["grid_idx"] = idx
                        _shared["tally"] = {}
                        _shared["conf_sum"] = {}
                        _shared["frame_cnt"] = 0
                        _shared["retry_count"] = 0
                    elif cmd == CMD_ACK:
                        print("<< ACK idx=", idx)
                        _shared["active"] = False
                        _shared["tally"] = {}
                        _shared["conf_sum"] = {}
                        _shared["frame_cnt"] = 0
        except OSError:
            break


class AnimalDetectApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.5, nms_threshold=0.2,
                 rgb888p_size=[640, 480], display_size=[800, 480], debug_mode=0,
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
        self.detect_crop_enable = detect_crop_enable
        self._dmap_sx = 1.0
        self._dmap_sy = 1.0
        if detect_crop_enable:
            sw, sh = rgb888p_size[0], rgb888p_size[1]
            self._dmap_sx = int(sw * detect_crop_w_ratio) / sw
            self._dmap_sy = int(sh * detect_crop_h_ratio) / sh
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

    # 获取检测结果统计（用于 active 模式时累积）
    def get_frame_data(self, dets):
        counts = {}
        max_conf = {}
        if not dets:
            return counts, max_conf
        dw = self.display_size[0]
        dh = self.display_size[1]
        if self.detect_crop_enable:
            margin_x = (dw - dw * self._dmap_sx) / 2.0
            margin_y = (dh - dh * self._dmap_sy) / 2.0
        for i in range(len(dets[0])):
            if self.detect_crop_enable:
                x, y, w, h = dets[0][i]
                cx = x + w / 2.0
                cy = y + h / 2.0
                if cx < margin_x or cx > dw - margin_x or cy < margin_y or cy > dh - margin_y:
                    continue
            label_id = dets[1][i]
            score = float(dets[2][i])
            counts[label_id] = counts.get(label_id, 0) + 1
            if label_id not in max_conf or score > max_conf[label_id]:
                max_conf[label_id] = score
        return counts, max_conf

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


if __name__ == "__main__":
    # ========== 摄像头 AI 输入分辨率 ==========
    rgb888p_size = [640, 480]

    # ========== 模型路径 ==========
    kmodel_path = "/sdcard/examples/mycode/new_animal_v2.kmodel"

    # ========== 动物类别标签 ==========
    labels = ["tiger", "wolf", "monkey", "peacock", "elephant"]

    # ========== 检测参数 ==========
    confidence_threshold = 0.3
    nms_threshold = 0.5
    max_boxes_num = 30

    # ========== 检测区域裁剪：只统计中心区域的检测框 ==========
    DETECT_CROP_ENABLE = True
    DETECT_CROP_W_RATIO = 0.4
    DETECT_CROP_H_RATIO = 0.6

    # ========== 模型输入尺寸 ==========
    model_input_size = [320, 320]

    # ========== 显示尺寸（推理用，不实际显示） ==========
    display_size = [800, 480]

    # ========== UART 配置 ==========
    UART_ID = UART.UART3
    UART_BAUD = 115200
    uart = UART(UART_ID, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    # ========== 初始化 PipeLine (无显示模式) ==========
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=None)
    pl.create(sensor=Sensor(id=0, fps=30))

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
        detect_crop_enable=DETECT_CROP_ENABLE,
        detect_crop_w_ratio=DETECT_CROP_W_RATIO,
        detect_crop_h_ratio=DETECT_CROP_H_RATIO,
    )
    animal_det.config_preprocess()

    print("=" * 50)
    print("  K230 Animal Detection — Pi双向通信模式")
    print("  Model:", kmodel_path.split("/")[-1])
    print("  Classes:", labels)
    print("  Confidence:", confidence_threshold, " NMS:", nms_threshold)
    print("  Frames per grid:", FRAMES_PER_GRID)
    print("  UART: UART{} @ {} baud".format(UART_ID, UART_BAUD))
    print("  Waiting for Pi START command...")
    print("=" * 50)

    # ========== 启动 UART RX 守护线程 ==========
    _thread.start_new_thread(uart_rx_thread, ())

    frame_count = 0
    try:
        while True:
            img = pl.get_frame()
            res = animal_det.run(img)

            # === Active 模式: 累积检测结果 ===
            active = False
            with _state_lock:
                active = _shared["active"]

            if active:
                counts, max_conf = animal_det.get_frame_data(res)
                with _state_lock:
                    # 累加每类检测数
                    for cls_id, cnt in counts.items():
                        _shared["tally"][cls_id] = _shared["tally"].get(cls_id, 0) + cnt
                    # 累加每类最高置信度
                    for cls_id, conf in max_conf.items():
                        _shared["conf_sum"][cls_id] = _shared["conf_sum"].get(cls_id, 0) + conf
                    _shared["frame_cnt"] += 1

                    # 满帧 → 内部评估
                    if _shared["frame_cnt"] >= FRAMES_PER_GRID:
                        print("  eval #", _shared["retry_count"], "tally:", _shared["tally"])
                        tally = _shared["tally"]
                        conf_sum = _shared["conf_sum"]

                        if tally:
                            best_id = max(tally, key=tally.get)
                            best_cnt = tally[best_id]
                            total_dets = sum(tally.values())
                            avg_conf = int(round(conf_sum.get(best_id, 0) / best_cnt))
                            if avg_conf > 100:
                                avg_conf = 100
                        else:
                            best_id = NO_ANIMAL
                            best_cnt = 0
                            total_dets = 0
                            avg_conf = 0

                        # 评估: 占比>=70% 且 置信>=50%, 或无动物 → 通过
                        dominance = best_cnt / max(total_dets, 1)
                        ok = (best_id == NO_ANIMAL or best_cnt == 0
                              or (dominance >= 0.7 and avg_conf >= 50))

                        if ok or _shared["retry_count"] >= 1:
                            # 发送 RESULT
                            print(">> RESULT best=", best_id, "cnt=", best_cnt, "conf=", avg_conf)
                            frame = bytes([FRAME_HEAD, CMD_RESULT,
                                          _shared["grid_idx"],
                                          best_id, best_cnt, total_dets, avg_conf,
                                          0xFF])
                            n = uart.write(frame)
                            print("  WRITE returned n=", n, "frame len=", len(frame))
                            _shared["active"] = False
                        else:
                            # 重试: 清空累加，retry_count++
                            print("  RETRY #", _shared["retry_count"])
                            _shared["tally"] = {}
                            _shared["conf_sum"] = {}
                            _shared["frame_cnt"] = 0
                            _shared["retry_count"] += 1

            if frame_count % 60 == 0:
                gc.collect()
            frame_count = (frame_count + 1) & 0x7FFFFFFF
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print("[ERROR]", e)
        import sys
        sys.print_exception(e)
    finally:
        _shared["active"] = False
        animal_det.deinit()
        uart.deinit()
        pl.destroy()
        gc.collect()
        print("[INFO] Resources released.")
