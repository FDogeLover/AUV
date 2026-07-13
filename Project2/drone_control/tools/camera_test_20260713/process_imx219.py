"""IMX219 raw10(RGGB) -> 真实色彩。白平衡增益直接在raw四通道(R/Gr/Gb/B)上算并应用，
再debayer，避开cv2去马赛克插值后再算白平衡时出现的通道比例失真问题。
用法: python3 process_imx219.py <raw文件> <输出png> [black_mode: perchannel|fixed] [black_value(仅fixed用)]
"""
import sys
import numpy as np
import cv2

WIDTH, HEIGHT = 3264, 2464


def load_raw16(path):
    data = np.fromfile(path, dtype=np.uint16)
    expected = WIDTH * HEIGHT
    if data.size != expected:
        raise ValueError(f"raw文件像素数{data.size}与期望{expected}不符")
    return data.reshape(HEIGHT, WIDTH)


def split_rggb(raw):
    R = raw[0::2, 0::2].astype(np.float64)
    Gr = raw[0::2, 1::2].astype(np.float64)
    Gb = raw[1::2, 0::2].astype(np.float64)
    B = raw[1::2, 1::2].astype(np.float64)
    return R, Gr, Gb, B


def merge_rggb(R, Gr, Gb, B, dtype=np.uint16):
    h, w = R.shape
    out = np.zeros((h * 2, w * 2), dtype=np.float64)
    out[0::2, 0::2] = R
    out[0::2, 1::2] = Gr
    out[1::2, 0::2] = Gb
    out[1::2, 1::2] = B
    return np.clip(out, 0, 1023).astype(dtype)


def main():
    raw_path = sys.argv[1]
    out_path = sys.argv[2]
    black_mode = sys.argv[3] if len(sys.argv) > 3 else "perchannel"
    fixed_black = int(sys.argv[4]) if len(sys.argv) > 4 else 64

    raw = load_raw16(raw_path)
    R, Gr, Gb, B = split_rggb(raw)

    if black_mode == "perchannel":
        # 用各通道自己的低百分位数近似黑电平(比全图共用一个值更准，弥补通道间pedestal差异)
        bl_r, bl_gr, bl_gb, bl_b = (np.percentile(c, 0.5) for c in (R, Gr, Gb, B))
    else:
        bl_r = bl_gr = bl_gb = bl_b = fixed_black

    R = np.clip(R - bl_r, 0, None)
    Gr = np.clip(Gr - bl_gr, 0, None)
    Gb = np.clip(Gb - bl_gb, 0, None)
    B = np.clip(B - bl_b, 0, None)

    g_mean = (Gr.mean() + Gb.mean()) / 2
    gain_r = g_mean / R.mean()
    gain_b = g_mean / B.mean()
    print(f"black levels: R={bl_r:.1f} Gr={bl_gr:.1f} Gb={bl_gb:.1f} B={bl_b:.1f}")
    print(f"raw channel means(after black correction): R={R.mean():.1f} Gr={Gr.mean():.1f} "
          f"Gb={Gb.mean():.1f} B={B.mean():.1f}")
    print(f"wb gains: gain_r={gain_r:.3f} gain_b={gain_b:.3f}")

    R = R * gain_r
    B = B * gain_b

    raw_wb = merge_rggb(R, Gr, Gb, B)
    # 注意：这颗传感器实测正确的拜耳解码是BayerBG2BGR，不是fourcc名字RG10暗示的BayerRG2BGR
    # (2026-07-13用4种排列横向对比确认，见camera_test_20260713/README.md)
    bgr16 = cv2.cvtColor(raw_wb, cv2.COLOR_BayerBG2BGR)

    norm = np.clip(bgr16 / 1023.0, 0, 1)
    gamma = np.power(norm, 1.0 / 2.2)
    final = (gamma * 255).astype(np.uint8)

    cv2.imwrite(out_path, final)
    print(f"saved -> {out_path}")


if __name__ == "__main__":
    main()
