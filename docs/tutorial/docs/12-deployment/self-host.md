# 本地部署教程

如果你网络环境访问 GitHub Pages 较慢，或需要对教程做二次修改，可以在本地部署。

## 前置条件

| 条目 | 要求 |
|------|------|
| 操作系统 | Windows / macOS / Linux 均可 |
| Python | 3.8 或以上 |
| Git | 用于克隆仓库 |

## 步骤

### 1. 克隆仓库

!!! example "人工操作"

    ```bash
    git clone https://github.com/FDogeLover/AUV.git
    cd AUV
    ```

    如果仅需教程部分，也可以直接下载仓库 ZIP 包，解压后进入目录。

### 2. 安装依赖

!!! example "人工操作"

    ```bash
    # 进入教程目录
    cd docs/tutorial

    # 建议先创建虚拟环境（可选）
    python -m venv venv
    # Windows
    venv\Scripts\activate
    # macOS / Linux
    source venv/bin/activate

    # 安装 MkDocs 及主题
    pip install -r requirements.txt
    ```

### 3. 启动本地服务

!!! example "人工操作"

    ```bash
    mkdocs serve
    ```

    终端会显示如下输出：

    ```
    INFO    -  Building documentation...
    INFO    -  Cleaning site directory
    INFO    -  Documentation built in 2.3 seconds
    INFO    -  Serving on http://127.0.0.1:8000
    ```

### 4. 浏览器访问

!!! example "人工操作"

    打开浏览器访问 **http://127.0.0.1:8000**

    本地服务支持热更新：修改 `docs/` 下的 Markdown 文件后，浏览器会自动刷新。

### 5. 构建静态站点（可选）

如果需要将教程部署到自己的服务器或其他平台：

!!! example "人工操作"

    ```bash
    mkdocs build
    ```

    构建产物在 `docs/tutorial/site/` 目录下，是一组纯静态 HTML 文件，可直接用 Nginx、Apache 或任何静态文件服务器托管。

## 常见问题

### `mkdocs` 命令找不到

安装完成后提示 `command not found`，通常是 Python 的 Scripts 目录未加入系统 PATH。

**Windows**：将 `Python安装路径\Scripts` 添加到系统环境变量 PATH。

**macOS / Linux**：使用 `python -m mkdocs serve` 代替 `mkdocs serve`。

### 端口 8000 被占用

指定其他端口启动：

```bash
mkdocs serve --dev-addr 127.0.0.1:8080
```

### 构建静态站点部署到自己的服务器

```bash
mkdocs build
```

构建产物在 `docs/tutorial/site/` 目录下，是一组纯静态 HTML 文件，可直接用 Nginx、Apache 或任何静态文件服务器托管。
