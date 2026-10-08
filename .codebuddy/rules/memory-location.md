# 记忆与规则的存放位置

- 项目记忆写在仓库内 `.codebuddy/memory/`，随 git 同步；`MEMORY.md` 为索引，一个记忆一个文件。
- **每次会话开始先自检软链**：若 `~/.codebuddy/projects/<工作区路径编码>/memory` 不是指向 `<仓库>/.codebuddy/memory` 的软链，先按下面的命令创建再继续工作（无需询问用户，这是常规初始化）。
- 新设备（或重建容器后）需要把本地记忆目录软链到仓库目录，否则记忆只会写进本地 home：

```bash
ln -s <仓库路径>/.codebuddy/memory ~/.codebuddy/projects/<工作区路径编码>/memory
# 例（devcontainer 内）：ln -s /workspaces/RMCS/.codebuddy/memory ~/.codebuddy/projects/workspaces-RMCS/memory
```

其中 `<工作区路径编码>` 是工作区绝对路径去掉首斜杠并把 `/` 换成 `-`（`/workspaces/RMCS` → `workspaces-RMCS`）。
