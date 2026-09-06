#!/usr/bin/env bash
set -euo pipefail

if ((EUID != 0)); then
  echo "Run with sudo: sudo ./install.sh" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target_user="${SUDO_USER:-$(id -un)}"
target_home="$(getent passwd "${target_user}" | cut -d: -f6)"
workspace="${target_home}/robot_ws"

if [[ -z "${target_home}" || ! -f "${workspace}/install/setup.bash" ]]; then
  echo "Missing ${workspace}/install/setup.bash; build the workspace first." >&2
  exit 1
fi

if ! /usr/bin/python3 -c 'import importlib.util; raise SystemExit(importlib.util.find_spec("Jetson.GPIO") is None)'; then
  apt-get install -y python3-jetson-gpio
fi

getent group gpio >/dev/null || groupadd -r gpio
usermod -a -G gpio "${target_user}"

install -d -m 0755 /usr/local/lib/wro-robot-autostart
install -m 0755 "${script_dir}/robot_autostart.py" \
  /usr/local/lib/wro-robot-autostart/robot_autostart.py
install -m 0755 "${script_dir}/run_robot_autostart.sh" \
  /usr/local/lib/wro-robot-autostart/run_robot_autostart.sh
sed -e "s|@USER@|${target_user}|g" -e "s|@HOME@|${target_home}|g" \
  "${script_dir}/wro-robot-autostart.service.in" \
  > /etc/systemd/system/wro-robot-autostart.service

systemctl daemon-reload
systemctl enable wro-robot-autostart.service
systemctl restart wro-robot-autostart.service

echo "Installed wro-robot-autostart.service for ${target_user}."
echo "Logs: journalctl -u wro-robot-autostart.service -f"
