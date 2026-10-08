# RMCS 开发工作流

- 构建/运行/部署统一使用仓库 `.script` 下的脚本，不要直接调 colcon/ros2：
  - `build-rmcs`（`build-rmcs-cross --target-arch <arch>` 交叉编译）、`launch-rmcs`、`set-robot`
  - `sync-remote` 同步构建产物、`ssh-remote` / `attach-remote`（`-r` 先重启再附加）
- 采用「开发容器 → ssh → 部署容器」双容器模式，部署端服务用 `service rmcs restart|attach` 管理。
- 下位机的 udev 规则必须在**主机**（不是容器）上配置，只需执行一次。
- 常用组合：`build-rmcs && wait-sync && attach-remote -r`。

这些流程的完整说明见仓库 `README.md`，此处只作速查。
