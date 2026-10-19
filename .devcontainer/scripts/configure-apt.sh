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
