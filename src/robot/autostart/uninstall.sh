#!/usr/bin/env bash
set -euo pipefail

if ((EUID != 0)); then
  echo "Run with sudo: sudo ./uninstall.sh" >&2
  exit 1
fi

systemctl disable --now wro-robot-autostart.service 2>/dev/null || true
rm -f /etc/systemd/system/wro-robot-autostart.service
rm -rf /usr/local/lib/wro-robot-autostart
systemctl daemon-reload

echo "Removed wro-robot-autostart.service."
