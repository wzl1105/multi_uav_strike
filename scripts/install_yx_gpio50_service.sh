#!/usr/bin/env bash
set -euo pipefail

# Install a root-owned, boot-time GPIO initializer for the YX RS-485
# direction pin. Defaults match RK GPIO1_C2: sysfs GPIO 50.
#
# Usage:
#   sudo bash install_yx_gpio50_service.sh [gpio_number] [runtime_user] [runtime_group]
# Example:
#   sudo bash install_yx_gpio50_service.sh 50 rpdzkj rpdzkj

GPIO_NUMBER="${1:-50}"
RUNTIME_USER="${2:-rpdzkj}"
RUNTIME_GROUP="${3:-${RUNTIME_USER}}"
INIT_PROGRAM="/usr/local/sbin/tzs-yx-gpio-init"
UNIT_FILE="/etc/systemd/system/tzs-yx-gpio-init.service"

if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: run this installer with sudo/root" >&2
  exit 1
fi

if ! [[ "${GPIO_NUMBER}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: gpio_number must be a non-negative integer" >&2
  exit 1
fi

if ! id "${RUNTIME_USER}" >/dev/null 2>&1; then
  echo "ERROR: runtime user does not exist: ${RUNTIME_USER}" >&2
  exit 1
fi

if ! getent group "${RUNTIME_GROUP}" >/dev/null 2>&1; then
  echo "ERROR: runtime group does not exist: ${RUNTIME_GROUP}" >&2
  exit 1
fi

TASK_TMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "${TASK_TMP_DIR}"' EXIT

cat >"${TASK_TMP_DIR}/tzs-yx-gpio-init" <<'INIT_EOF'
#!/usr/bin/env bash
set -euo pipefail

GPIO_NUMBER="${1:-50}"
RUNTIME_USER="${2:-rpdzkj}"
RUNTIME_GROUP="${3:-${RUNTIME_USER}}"
GPIO_ROOT="/sys/class/gpio"
GPIO_DIR="${GPIO_ROOT}/gpio${GPIO_NUMBER}"

if [[ ! -e "${GPIO_ROOT}/export" ]]; then
  echo "ERROR: legacy GPIO sysfs is unavailable" >&2
  exit 1
fi

if [[ ! -d "${GPIO_DIR}" ]]; then
  # EBUSY means another process exported it between the directory check and write.
  if ! printf '%s' "${GPIO_NUMBER}" >"${GPIO_ROOT}/export" 2>/dev/null; then
    [[ -d "${GPIO_DIR}" ]] || {
      echo "ERROR: failed to export GPIO${GPIO_NUMBER}" >&2
      exit 1
    }
  fi
fi

for _ in $(seq 1 50); do
  [[ -e "${GPIO_DIR}/direction" && -e "${GPIO_DIR}/value" ]] && break
  sleep 0.02
done

if [[ ! -e "${GPIO_DIR}/direction" || ! -e "${GPIO_DIR}/value" ]]; then
  echo "ERROR: GPIO${GPIO_NUMBER} sysfs files did not appear" >&2
  exit 1
fi

# "low" changes direction and initial value atomically. Fall back for kernels
# that only accept "out".
if ! printf 'low' >"${GPIO_DIR}/direction" 2>/dev/null; then
  printf 'out' >"${GPIO_DIR}/direction"
  printf '0' >"${GPIO_DIR}/value"
fi

chown "${RUNTIME_USER}:${RUNTIME_GROUP}" \
  "${GPIO_DIR}/direction" "${GPIO_DIR}/value"
chmod 0660 "${GPIO_DIR}/direction" "${GPIO_DIR}/value"

direction="$(cat "${GPIO_DIR}/direction")"
value="$(cat "${GPIO_DIR}/value")"
if [[ "${direction}" != "out" || "${value}" != "0" ]]; then
  echo "ERROR: GPIO${GPIO_NUMBER} verification failed: direction=${direction} value=${value}" >&2
  exit 1
fi

echo "GPIO${GPIO_NUMBER} ready: direction=${direction} value=${value} owner=${RUNTIME_USER}:${RUNTIME_GROUP}"
INIT_EOF

cat >"${TASK_TMP_DIR}/tzs-yx-gpio-init.service" <<UNIT_EOF
[Unit]
Description=Initialize YX RS485 direction GPIO${GPIO_NUMBER}
Before=tzs_ws-multi.service tzs_ws-payload.service

[Service]
Type=oneshot
ExecStart=${INIT_PROGRAM} ${GPIO_NUMBER} ${RUNTIME_USER} ${RUNTIME_GROUP}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT_EOF

install -o root -g root -m 0755 \
  "${TASK_TMP_DIR}/tzs-yx-gpio-init" "${INIT_PROGRAM}"
install -o root -g root -m 0644 \
  "${TASK_TMP_DIR}/tzs-yx-gpio-init.service" "${UNIT_FILE}"

# Make either ROS service pull in and wait for GPIO initialization. Creating a
# drop-in for a service not installed on a particular board is harmless.
for ros_unit in tzs_ws-multi.service tzs_ws-payload.service; do
  dropin_dir="/etc/systemd/system/${ros_unit}.d"
  install -d -o root -g root -m 0755 "${dropin_dir}"
  cat >"${dropin_dir}/10-yx-gpio.conf" <<DROPIN_EOF
[Unit]
Requires=tzs-yx-gpio-init.service
After=tzs-yx-gpio-init.service
DROPIN_EOF
  chmod 0644 "${dropin_dir}/10-yx-gpio.conf"
done

systemctl daemon-reload
systemctl enable tzs-yx-gpio-init.service
systemctl restart tzs-yx-gpio-init.service

echo
systemctl --no-pager --full status tzs-yx-gpio-init.service || true
echo
echo "Installed successfully. Reboot once, then verify with:"
echo "  systemctl status tzs-yx-gpio-init.service"
echo "  cat /sys/class/gpio/gpio${GPIO_NUMBER}/direction"
echo "  cat /sys/class/gpio/gpio${GPIO_NUMBER}/value"
echo "  cat /sys/kernel/debug/gpio | grep 'gpio-${GPIO_NUMBER}'"
