# 本地开发环境

本地环境用于写代码、跑单元测试、桌面模拟飞行，不需要连接任何硬件。

## 1. 安装 Python 3.10+

推荐使用 Miniconda 或系统 Python 3.10 及以上版本。

```bash
python --version  # 确认 >= 3.10
```

## 2. 克隆代码仓库

```bash
git clone https://github.com/FDogeLover/AUV.git
cd AUV
```

!!! tip "工作目录自定义"
    仓库可以克隆到任意位置，目录名也可以自行修改。后续教程中的路径均以仓库根目录为基准，不依赖特定的绝对路径。

## 3. 进入基础版本目录，安装依赖

```bash
cd drone_control/basic
pip install -r requirements-dev.txt  # 含 pytest 等开发工具
```

!!! info "T265 依赖说明"
    `pyrealsense2` 在桌面环境不需要安装。DRY_RUN 模式下 `t265.py` 会自动降级为模拟数据生成器。

## 4. 验证环境

```bash
pytest -v
```

看到 18+ 个测试全部通过即表示本地环境就绪。

---

[板端环境配置 →](board-env.md)
