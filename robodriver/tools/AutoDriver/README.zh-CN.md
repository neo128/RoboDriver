# AutoDriver

[English](README.md) | [中文](README.zh-CN.md)

**机器人传感器配置代码生成工具**

AutoDriver 是一个面向机器人开发者的可视化配置工具，通过 GUI 快速配置机器人组件、生成 JSON 配置文件、构造 Episode 数据目录，并自动生成 ROS2 控制代码。

本工具旨在减少机器人系统集成中的重复劳动，让开发者更专注于算法与控制逻辑。

---

## ✨ 特性

### 🔧 图形化机器人组件配置
- GUI 添加关节、摄像头与传感器组件
- 属性面板实时编辑 topic / msg / period / joint_index 等
- 双向同步 JSON 配置（GUI ↔ JSON）

---

## 📦 项目结构

```text
AutoDriver/
├── config/                 # Robot JSON configs
├── GUI/                    # GUI application
│   ├── robot_config_window.py
│   ├── generate_ros2.py
│   ├── detect_components.py
│   ├── camera_widget.py
│   └── ...
├── templates               # Code templates
└── README.md
```

---

## 🛠 安装

```bash
conda create -n AutoDriver python=3.10
conda activate AutoDriver
pip install PyQt5 opencv-python-headless pyyaml
```

---

## ▶️ 运行 GUI

```bash
python AutoDriver/GUI/robot_config_window.py
```

---

## 📘 使用教程

### 1. 添加组件 Add Components
左栏选择组件类型并点击 `+`。

### 2. 配置属性 Configure Attributes
编辑 `topic`、`msg`、`joint_index` 等信息。

### 3. 保存配置 Save Configuration
生成 `config/<robot_name>.json`。

### 4. 生成代码 Apply Config
生成机器人配置文件包，例如 `robodriver-robot-so101-aio`。

### 5. 安装机器人包
举例：
```bash
cd robodriver-robot-so101-aio
pip install -e .
```

---

## 📄 许可证
在这里补充你的许可证（例如 MIT、Apache-2.0）。

## 🤝 贡献指南
欢迎提 Issue 和 PR。
