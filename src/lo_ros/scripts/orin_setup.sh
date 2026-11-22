#!/usr/bin/env bash
# Jetson AGX Orin runtime setup for STRIDE (run on the robot, needs root). docs/TUNING.md.
#  * max power mode + max clocks
#  * CPU governor "performance" on every core
#  * prints the isolcpus/rtprio hints for runtime.cpu_affinity / runtime.sched_fifo_priority
set -euo pipefail
if [[ $EUID -ne 0 ]]; then echo "run as root (sudo $0)" >&2; exit 1; fi
if command -v nvpmodel >/dev/null; then nvpmodel -m 0; else echo "nvpmodel not found (not a Jetson?)"; fi
if command -v jetson_clocks >/dev/null; then jetson_clocks; fi
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [[ -w "$g" ]] && echo performance > "$g"
done
echo "governors: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
cat <<'HINT'
Pinning: give STRIDE two dedicated cores, e.g. runtime.cpu_affinity: [6, 7], and keep them free:
  kernel cmdline: isolcpus=6,7 nohz_full=6,7 rcu_nocbs=6,7   (then reboot)
SCHED_FIFO (runtime.sched_fifo_priority: 80) needs, for the user running the node:
  /etc/security/limits.d/stride.conf:  <user>  -  rtprio  90
HINT
