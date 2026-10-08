#!/usr/bin/env bash
# 把容器内的项目记忆目录软链到仓库内的 .codebuddy/memory，使记忆随 git 同步。
# 用法: bash .devcontainer/scripts/setup-agent-memory.sh [workspace-folder]
set -eu

workspace="${1:-/workspaces/RMCS}"
repo_memory="$workspace/.codebuddy/memory"

# ~/.codebuddy/projects/<工作区路径去掉首斜杠并把 / 换成 ->/memory
project_id="$(printf '%s' "$workspace" | sed 's#^/##; s#/#-#g')"
local_memory="$HOME/.codebuddy/projects/$project_id/memory"

mkdir -p "$repo_memory" "$(dirname "$local_memory")"

if [ -L "$local_memory" ]; then
    if [ "$(readlink "$local_memory")" = "$repo_memory" ]; then
        echo "[agent-memory] already linked: $local_memory -> $repo_memory"
        exit 0
    fi
    rm "$local_memory"
elif [ -d "$local_memory" ]; then
    # 已存在实体目录：把里面的记忆文件并进仓库目录
    shopt -s dotglob nullglob
    for f in "$local_memory"/*; do
        [ -e "$f" ] && mv -n "$f" "$repo_memory/" || true
    done
    rmdir "$local_memory"
elif [ -e "$local_memory" ]; then
    rm "$local_memory"
fi

ln -s "$repo_memory" "$local_memory"
echo "[agent-memory] linked: $local_memory -> $repo_memory"
