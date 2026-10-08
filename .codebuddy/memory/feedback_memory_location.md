---
name: 记忆文件存放位置
description: 项目记忆写进仓库 .codebuddy/memory/ 并用软链接接到本地记忆目录，实现跨设备 git 同步
type: feedback
---

项目记忆写进仓库内的 `.codebuddy/memory/`，并把本地记忆目录软链过去；长期有效的项目约定写进 `.codebuddy/rules/*.md`。不要再往本地 `~/.codebuddy/projects/.../memory` 里直接写实体文件。

**Why:** CodeBuddy 的记忆目录名由工作区绝对路径生成（容器内 `/workspaces/RMCS` → `workspaces-RMCS`），宿主机/其他设备路径不同就是不同目录，靠 bind mount 也无法对齐；而仓库目录能被 git 同步，任何设备克隆后都能读到。用户明确要求「随便就能同步记忆」，不愿为同步折腾挂载配置。

**How to apply:** 写记忆时直接写 `.codebuddy/memory/` 下的文件并更新 `MEMORY.md` 索引；在新设备或重建容器后，先建软链 `ln -s <仓库>/.codebuddy/memory ~/.codebuddy/projects/<路径编码>/memory` 再开始工作。
