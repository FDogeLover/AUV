"""drone_control 共享测试配置。

自动将当前测试文件所在目录加入 sys.path，使得各子模块的 Lcode/ 和 Mission_GPT.py
等本地模块可以被正确导入，无需每个测试文件手动 sys.path.insert。
"""
import os
import sys


def pytest_collect_file(parent, file_path):
    """支持从 shared_tests/ 目录收集测试。"""
    pass


def pytest_configure(config):
    """确保 drone_control 根目录在 sys.path 中，以便导入 shared_tests。"""
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root_dir not in sys.path:
        sys.path.insert(0, root_dir)


def pytest_collection_modifyitems(config, items):
    """为每个测试自动注入其所在目录的路径。"""
    for item in items:
        test_file_dir = os.path.dirname(str(item.fspath))
        if test_file_dir not in sys.path:
            sys.path.insert(0, test_file_dir)
