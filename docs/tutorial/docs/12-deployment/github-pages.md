# 在线访问教程

本教程已通过 GitHub Pages 自动部署，无需安装任何环境，直接用浏览器访问即可。

## 访问地址

!!! success "在线教程地址"

    **[https://fdogelover.github.io/AUV/](https://fdogelover.github.io/AUV/)**

    点击链接即可打开，支持桌面和移动端浏览器。

## 自动部署机制

教程使用 GitHub Actions 自动构建和部署：

1. 仓库 `main` 分支的 `docs/tutorial/` 目录有更新时，自动触发构建
2. GitHub Actions 运行 MkDocs Material 构建静态站点
3. 构建产物自动发布到 GitHub Pages

!!! info "构建状态"

    可在仓库的 **Actions** 页面查看每次构建的日志和状态。

## 在线版功能

| 功能 | 说明 |
|------|------|
| 全文搜索 | 支持中文搜索，实时高亮匹配 |
| 暗色模式 | 右上角切换图标，自动记忆偏好 |
| 代码复制 | 每段代码右上角有一键复制按钮 |
| 目录导航 | 左侧树形目录，支持展开/折叠 |
| 移动端适配 | 手机浏览器自动切换为抽屉式导航 |

---

如果你的网络环境访问 GitHub Pages 较慢，可以参考 [本地部署](self-host.md) 方式在本地运行教程。
