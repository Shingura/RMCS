#!/usr/bin/env bash

set -euo pipefail

# Keep apt configuration in the running container so it also applies to
# interactive apt commands issued after the container has been opened.
apt_proxy="http://127.0.0.1:7890"
ubuntu_mirror="https://mirrors.tuna.tsinghua.edu.cn/ubuntu"
ubuntu_ports_mirror="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"

sudo install -d -m 0755 /etc/apt/apt.conf.d
sudo tee /etc/apt/apt.conf.d/80proxy >/dev/null <<EOF_PROXY
Acquire::http::Proxy "${apt_proxy}";
Acquire::https::Proxy "${apt_proxy}";
EOF_PROXY

# Ubuntu 24.04 uses /etc/apt/sources.list.d/ubuntu.sources (DEB822), while
# older images may still use /etc/apt/sources.list or *.list files. Update
# only Ubuntu's official endpoints and leave third-party repositories intact.
for source_file in \
    /etc/apt/sources.list \
    /etc/apt/sources.list.d/*.list \
    /etc/apt/sources.list.d/*.sources; do
    [[ -f "${source_file}" ]] || continue

    sudo sed -i \
        -e "s#https\?://[[:alnum:].-]*archive\.ubuntu\.com/ubuntu#${ubuntu_mirror}#g" \
        -e "s#https\?://security\.ubuntu\.com/ubuntu#${ubuntu_mirror}#g" \
        -e "s#https\?://ports\.ubuntu\.com/ubuntu-ports#${ubuntu_ports_mirror}#g" \
        "${source_file}"
done

printf 'APT configured: proxy=%s, Ubuntu mirror=%s, Ubuntu ports mirror=%s\n' \
    "${apt_proxy}" "${ubuntu_mirror}" "${ubuntu_ports_mirror}"

# Configure proxy variables for interactive zsh sessions. Keep this block
# managed and replace it on every run so reopening the container does not
# append duplicate exports to ~/.zshrc.
terminal_http_proxy="http://127.0.0.1:7890"
terminal_all_proxy="socks5://127.0.0.1:7890"
zshrc_file="${HOME}/.zshrc"
proxy_block_start="# >>> RMCS terminal proxy >>>"
proxy_block_end="# <<< RMCS terminal proxy <<<"

touch "${zshrc_file}"
sed -i \
    "/^${proxy_block_start}$/,/^${proxy_block_end}$/d" \
    "${zshrc_file}"
cat >>"${zshrc_file}" <<EOF_ZSH_PROXY

${proxy_block_start}
export http_proxy="${terminal_http_proxy}"
export https_proxy="${terminal_http_proxy}"
export ALL_PROXY="${terminal_all_proxy}"
${proxy_block_end}
EOF_ZSH_PROXY

printf 'Terminal proxy configured in %s: http/https=%s, all=%s\n' \
    "${zshrc_file}" "${terminal_http_proxy}" "${terminal_all_proxy}"
