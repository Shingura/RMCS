---
name: 用户与协作背景
description: 用户在 RMCS（基于 ROS2 的 RoboMaster 控制系统）仓库工作，使用 Docker 双容器工作流，中文交流
type: user
---

用户在 `/workspaces/RMCS` 仓库（RMCS = RoboMaster Control System，基于 ROS2，C++）中开发。

- 交流语言为简体中文，回复应简洁直接。
- 项目中日常使用的命令是 `.script` 下的封装脚本：`build-rmcs`、`launch-rmcs`、`sync-remote`、`ssh-remote`、`attach-remote`、`set-robot`，而非直接调用 colcon/ros2。
- 工作流是「开发容器 → ssh → 部署容器」的双容器模式，代码更新靠 `sync-remote` 实时同步，服务通过 `service rmcs restart/attach` 管理。

**How to apply:** 讨论构建、部署、调试时优先使用上述脚本与双容器语境，不要默认用户在本机裸跑 ROS2。
