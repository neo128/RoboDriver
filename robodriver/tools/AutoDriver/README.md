# AutoDriver

[English](README.md) | [中文](README.zh-CN.md)

**Robot Sensor Configuration & Code Generation Tool**

AutoDriver is a visual configuration tool for robotics developers. It helps you quickly configure robot components via a GUI, generate JSON configuration files, construct episode data directories, and automatically generate ROS2 control code.

This tool aims to reduce repetitive integration work so developers can focus on algorithms and control logic.

---

## ✨ Features

### 🔧 Visual Robot Component Configuration
- Add joints, cameras, and sensor components via GUI
- Edit properties in real time (topic / msg / period / joint_index, etc.)
- Two-way sync between GUI and JSON config

---

## 📦 Project Structure

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

## 🛠 Installation

```bash
conda create -n AutoDriver python=3.10
conda activate AutoDriver
pip install PyQt5 opencv-python-headless pyyaml
```

---

## ▶️ Run GUI

```bash
python AutoDriver/GUI/robot_config_window.py
```

---

## 📘 Tutorial

### 1. Add Components
In the left panel, select a component type and click `+`.

### 2. Configure Attributes
Edit `topic`, `msg`, `joint_index`, etc.

### 3. Save Configuration
This generates `config/<robot_name>.json`.

### 4. Apply Config / Generate Code
Generate a robot config package, e.g. `robodriver-robot-so101-aio`.

### 5. Install Robot Package
Example:
```bash
cd robodriver-robot-so101-aio
pip install -e .
```

---

## 📄 License
Specify your license here (e.g., MIT, Apache-2.0).

## 🤝 Contributing
Issues and PRs are welcome.
