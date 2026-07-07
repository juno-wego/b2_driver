#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sudo ./set_unitree_static_ip.sh <interface> [ipv4_cidr]

Examples:
  sudo ./set_unitree_static_ip.sh enp3s0
  sudo ./set_unitree_static_ip.sh enp3s0 192.168.123.99/24

This script writes a dedicated netplan file for a Unitree robot link and
applies it immediately. Default IPv4 is 192.168.123.99/24.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

iface="$1"
address="${2:-192.168.123.99/24}"
netplan_dir="/etc/netplan"
netplan_file="${netplan_dir}/99-unitree-${iface}.yaml"
backup_suffix
backup_suffix="$(date +%Y%m%d%H%M%S)"

if ! command -v netplan >/dev/null 2>&1; then
  echo "netplan is not installed on this system." >&2
  exit 1
fi

if ! ip link show "${iface}" >/dev/null 2>&1; then
  echo "Interface '${iface}' does not exist." >&2
  ip -brief link >&2 || true
  exit 1
fi

renderer="networkd"
shopt -s nullglob
for file in "${netplan_dir}"/*.yaml; do
  if grep -Eq '^[[:space:]]*renderer:[[:space:]]*NetworkManager' "${file}"; then
    renderer="NetworkManager"
    break
  fi
done
shopt -u nullglob

mkdir -p "${netplan_dir}"
if [[ -f "${netplan_file}" ]]; then
  cp "${netplan_file}" "${netplan_file}.bak.${backup_suffix}"
fi

cat > "${netplan_file}" <<EOF
network:
  version: 2
  renderer: ${renderer}
  ethernets:
    ${iface}:
      dhcp4: false
      dhcp6: false
      optional: true
      addresses:
        - ${address}
EOF

chmod 600 "${netplan_file}"

echo "Wrote ${netplan_file}"
echo "Applying netplan..."
netplan generate
netplan apply

echo
echo "Interface summary:"
ip -brief addr show "${iface}"
echo
echo "Recommended launch usage:"
echo "  ros2 launch b2_base b2_bringup.launch.py network_interface:=${iface}"
