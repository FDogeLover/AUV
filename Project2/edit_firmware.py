#!/usr/bin/env python3
"""
edit_firmware.py — 飞控固件 .c/.h 文件安全编辑工具

自动检测文件编码 → 执行替换 → 验证编码不变 → 验证中文可读

用法:
  python edit_firmware.py show <filename>                 查看编码信息
  python edit_firmware.py replace <file> <old> <new>      替换文本（精确匹配）
  python edit_firmware.py verify <filename>               验证编码完整性

原理：
  飞控固件文件存在混合编码（GBK / UTF-8），直接使用 Read/Edit 工具可能
  静默转换编码，导致中文注释永久损坏。本脚本保证读/写使用同一编码。

退出码:
  0 = 成功
  1 = 文件不存在或编码未知
  2 = 替换目标未找到
  3 = 编码验证失败
"""
import sys
import os
import re


# ============================================================
# 编码检测
# ============================================================
def detect_encoding(filepath: str) -> str:
    """检测文件的实际编码: 'gbk' | 'utf-8' | None"""
    raw = open(filepath, 'rb').read()
    if not raw:
        return 'utf-8'  # 空文件默认 UTF-8

    # 检查 BOM
    if raw[:3] == b'\xef\xbb\xbf':
        return 'utf-8-sig'

    # 检查是否为纯 ASCII（无编码问题）
    if all(b < 128 for b in raw):
        return 'ascii'

    # 尝试 UTF-8 解码
    utf8_ok = False
    try:
        raw.decode('utf-8')
        utf8_ok = True
    except UnicodeDecodeError:
        pass

    # 尝试 GBK 解码
    gbk_ok = False
    try:
        raw.decode('gbk')
        gbk_ok = True
    except UnicodeDecodeError:
        pass

    # 判断
    if utf8_ok and gbk_ok:
        # 两者都能解码 → 检查中文是否正常
        utf8_text = raw.decode('utf-8')
        gbk_text = raw.decode('gbk')
        # 如果一个有明显乱码特征（如 �、\uFFFD 等），选另一个
        utf8_garbled = '\ufffd' in utf8_text or '锟' in utf8_text
        gbk_garbled = '\ufffd' in gbk_text or '锟' in gbk_text
        if not utf8_garbled and gbk_garbled:
            return 'utf-8'
        elif utf8_garbled and not gbk_garbled:
            return 'gbk'
        else:
            # 都看起来正常 → 按实际情况判断
            # 检查 GBK 特有高字节模式 (0x81-0xFE + 0x40-0xFE)
            gbk_pattern_count = 0
            for i in range(len(raw) - 1):
                if 0x81 <= raw[i] <= 0xFE and 0x40 <= raw[i+1] <= 0xFE:
                    gbk_pattern_count += 1
            return 'gbk' if gbk_pattern_count > 5 else 'utf-8'

    if utf8_ok:
        return 'utf-8'
    if gbk_ok:
        return 'gbk'
    return None


# ============================================================
# 编码验证
# ============================================================
def verify_encoding(filepath: str, expected_encoding: str = None) -> bool:
    """验证文件编码完整性"""
    actual = detect_encoding(filepath)
    if actual is None:
        print(f"[FAIL] 无法检测编码: {filepath}")
        return False

    if expected_encoding and actual != expected_encoding:
        print(f"[FAIL] 编码已改变: {expected_encoding} → {actual}")
        return False

    # 读取内容验证中文可读性
    raw = open(filepath, 'rb').read()
    try:
        text = raw.decode(actual)
    except Exception as e:
        print(f"[FAIL] 解码失败: {e}")
        return False

    # 检查是否有明显的编码损坏痕迹
    damage_markers = ['\ufffd', '\uFFFD', '�', '锟斤拷']
    for marker in damage_markers:
        if marker in text:
            # 仅在非 ASCII 文件中检查
            if any(ord(c) > 127 for c in text):
                print(f"[FAIL] 检测到编码损坏字符: {repr(marker)}")
                return False

    print(f"[OK] 编码={actual}, 大小={len(raw)}字节, 中文可读")
    return True


# ============================================================
# 安全替换
# ============================================================
def safe_replace(filepath: str, old_str: str, new_str: str) -> bool:
    """安全替换：检测编码 → 读取 → 替换 → 原编码写入 → 验证"""
    # 1. 检测编码
    encoding = detect_encoding(filepath)
    if encoding is None:
        print(f"[ERROR] 无法检测文件编码: {filepath}")
        return False
    print(f"[INFO] 检测到编码: {encoding}")

    # 2. 读取
    raw = open(filepath, 'rb').read()
    try:
        text = raw.decode(encoding)
    except Exception as e:
        print(f"[ERROR] 解码失败: {e}")
        return False

    # 3. 替换
    if old_str not in text:
        print(f"[ERROR] 未找到目标文本 (编码={encoding})")
        print(f"  目标: {repr(old_str)}")
        return False

    count = text.count(old_str)
    if count > 1:
        print(f"[WARN] 目标文本出现 {count} 次，将全部替换")

    text = text.replace(old_str, new_str)

    # 4. 写回（原编码）
    try:
        new_raw = text.encode(encoding)
        open(filepath, 'wb').write(new_raw)
    except Exception as e:
        print(f"[ERROR] 编码写回失败: {e}")
        return False

    print(f"[OK] 替换完成 ({count} 处)")

    # 5. 验证
    if not verify_encoding(filepath, encoding):
        return False

    return True


# ============================================================
# 显示信息
# ============================================================
def show_info(filepath: str):
    """显示文件编码信息"""
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return False

    raw = open(filepath, 'rb').read()
    encoding = detect_encoding(filepath)
    has_bom = raw[:3] == b'\xef\xbb\xbf'
    has_non_ascii = any(b > 127 for b in raw)

    print(f"文件:   {filepath}")
    print(f"大小:   {len(raw)} 字节")
    print(f"编码:   {encoding or '未知'}")
    print(f"BOM:    {'有 (UTF-8 BOM)' if has_bom else '无'}")
    print(f"中文:   {'有' if has_non_ascii else '无'}")
    if encoding and has_non_ascii:
        text = raw.decode(encoding)
        # 提取中文注释行
        cn_lines = [l.strip() for l in text.split('\n') if any('\u4e00' <= c <= '\u9fff' for c in l)]
        if cn_lines:
            print(f"中文行: {len(cn_lines)} 行")
            for l in cn_lines[:5]:
                print(f"  {l[:60]}")
    print(f"可编辑: {'是' if encoding else '否 (编码未知)'}")
    return True


# ============================================================
# 主入口
# ============================================================
def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    command = sys.argv[1]
    filepath = sys.argv[2]

    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        sys.exit(1)

    if command == 'show':
        if not show_info(filepath):
            sys.exit(1)

    elif command == 'verify':
        if not verify_encoding(filepath):
            sys.exit(3)

    elif command == 'replace':
        if len(sys.argv) < 5:
            print("用法: python edit_firmware.py replace <file> <old> <new>")
            sys.exit(1)
        old_str = sys.argv[3]
        new_str = sys.argv[4]
        if not safe_replace(filepath, old_str, new_str):
            sys.exit(2)

    else:
        print(f"[ERROR] 未知命令: {command}")
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
