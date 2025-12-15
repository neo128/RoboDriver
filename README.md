
- [2025-12-01] RoboDriver项目开源

## 目录

1. [概述](#概述)
2. [主要特点](#主要特点)
3. [快速入门](#快速入门)
4. [机器人示例](#机器人示例)
5. [参与贡献](#参与贡献)
6. [帮助支持](#帮助支持)
7. [许可证与致谢](#许可证和致谢)
8. [引用](#引用)

## 主要特点

- **多种机器人接入方式**： RoboDriver 考虑了除SDK外，使用ROS、Dora的接入方式。
- **LeRobot兼容**： RoboDriver 的机器人接口直接使用了LeRobot的`Robot`类，这意味着RoboDriver与LeRobot可以互相兼容。
- **改进的LeRobot数据集格式**：在不同数据阶段采取不同数据结构。在采集端单条储存，更易编辑和传输。还扩展了LeRobot格式的内容。


## 快速入门

请参考项目文档：[RoboDriver-Doc](https://flagopen.github.io/RoboDriver-Doc)

## 机器人示例
RoboDriver 已完成多款主流机器人的适配，按接入方式示例如下（各仓库包含对应机器人的接入步骤、环境配置、指令适配等完整指南）：

以下是将代码链接转为 **可直接点击访问** 的格式（基于主仓库 `https://github.com/FlagOpen/RoboDriver/tree/dev/` 拼接子目录，符合开源项目链接规范），同时统一表格样式保持整洁：

### 🔌 ROS1 接入
| 机器人型号 | 简介 | 代码链接 | 贡献人（联系方式） |
|------------|------|--------------|------------------------|
| Realman 机械臂 | 基于Realman，6DOF+力控模块，3*RealSense相机 | [robodriver-robot-realman-aio-ros1](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-realman-aio-ros1) | yangxiang（[https://github.com/Ryu-Yang](https://github.com/Ryu-Yang)） |

### 🔌 ROS2 接入
| 机器人型号 | 简介 | 代码链接 | 贡献人（联系方式） |
|--------------|--------------------------------------------------------------|----------------------------------------------------------------------------------------------|------------------------|
| GALAXEALITE | 基于Galaxealite，双臂6DOF+末端夹爪，4*RealSense相机 | [robodriver-robot-galaxealite-aio-ros2](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-galaxealite-aio-ros2) | liuyou（[https://github.com/liuyou1103](https://github.com/liuyou1103)） |
| SO101 机械臂 | 开源轻量级机械臂，6DOF+末端夹爪，1*RealSense相机，1*RGB相机模块 | [robodriver-robot-so101-aio-ros2](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-so101-aio-ros2) | yangxiang（[https://github.com/Ryu-Yang](https://github.com/Ryu-Yang)） |

### 🔌 Dora（SDK）接入
| 机器人型号 | 简介 | 代码链接 | 贡献人（联系方式） |
|--------------|--------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|------------------------|
| Realman 机械臂 | 基于Realman，6DOF+力控模块，3*RealSense相机 | [robodriver-robot-realman1-aio-dora](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-realman1-aio-dora) | xuruntian（[https://github.com/XuRuntian](https://github.com/XuRuntian)） |
| SO101 机械臂 | 开源轻量级机械臂，6DOF+末端夹爪，1*RealSense相机，1*RGB相机模块 | [robodriver-robot-so101-aio-dora](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-so101-aio-dora) | yangxiang（[https://github.com/Ryu-Yang](https://github.com/Ryu-Yang)） |
| Franka | 工业级机械臂，6DOF+末端夹爪，1*RealSense相机 | [robodriver-robot-franka-aio-dora](https://github.com/FlagOpen/RoboDriver/tree/dev/robodriver/robots/robodriver-robot-franka-aio-dora) | xuruntian（[https://github.com/XuRuntian](https://github.com/XuRuntian)） |

> ✨ 说明：
> 1. 接入方式命名规范：`robodriver-robot-[机器人型号]-[遥操方式]-[接入类型]`（如 `aio`/`follwer`/`teleoperate`, `ros2`/`dora`）；
> 2. 每个适配仓库内包含**环境搭建、配置修改、采集/控制验证**等完整接入指南；
> 3. 持续新增适配机器人，可关注本列表或项目更新。

我们非常欢迎社区开发者贡献更多机器人的实现！可按以下方式参与：
1. 参考已适配机器人的代码结构和 README 模板，按接入类型（ROS1/ROS2/Dora）完成适配开发；
2. 将适配代码新增至主仓库的 `robodriver/robots/` 目录下（命名规范与已适配机器人保持一致）；
3. 确保代码规范、文档完整（包含环境准备、配置步骤、功能验证）；
4. 提交代码 PR 至主仓库的 `dev` 分支，我们将及时审核并合并。

期待与您一起丰富 RoboDriver 的机器人生态！ 🤝

## 参与贡献

我们真诚地欢迎来自社区的 *任何形式的贡献*。从**新功能的拉取请求**、**错误报告**，到甚至是使RoboDriver更易用的微小**建议**，我们都全心全意地感谢！

## 帮助支持

- 请使用 Github [Issues](https://github.com/FlagOpen/RoboDriver/issues) 报告错误和提出功能请求。

- 请使用 GitHub [Discussions](https://github.com/FlagOpen/RoboDriver/discussions) 讨论想法和提问。


## 许可证和致谢

RoboDriver 源代码根据 Apache 2.0 许可证授权。 没有这些令人惊叹的开源项目，RoboDriver 的开发是不可能的：

- 感谢LeRobot团队开源LeRobot🤗, [LeRobot](https://github.com/huggingface/lerobot)。本项目由LeRobot改进而来。
- 感谢TheRobotStudio团队开源的SO100和SO101机械臂🤗, [SO101](https://github.com/TheRobotStudio/SO-ARM100)。在本项目中，SO101机械臂作为部署案例。
- 感谢dora-rs团队开源的机器人框架🤗, [dora](https://github.com/dora-rs/dora)。在本项目中，该框架为机器人带来了全新的接入方式。

## 引用

```bibtex
@misc{RoboDriver,
  author = {RoboDriver Authors},
  title = {RoboDriver: A robot control and data acquisition framework},
  month = {November},
  year = {2025},
  url = {https://github.com/FlagOpen/RoboDriver}
}
```
