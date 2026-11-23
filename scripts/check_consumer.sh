#!/usr/bin/env bash
# Prove that stride-lo is consumable from another workspace (docs/INTEGRATION.md):
#  1. build examples/minimal_backend in a fresh workspace with only stride-lo's install/ as the
#     underlay (exports of lo_msgs, lo_gtsam_adapter, GTSAM);
#  2. with a BAG: play it live against stride_lo (request mode) and the example backend, and
#     require > 0 accepted factors, 0 rejected (non-SPD) and 0 unmatched.
#   scripts/dc run --rm dev scripts/check_consumer.sh [BAG] [DURATION_S] [ROBOT]
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag="${1:-}"
dur="${2:-30}"
robot="${3:-anymal_c}"
cws="${ws}/build_consumer"          # git-ignored (build_*/)
if [ -z "${AMENT_PREFIX_PATH:-}" ] || [ -z "${STRIDE_UNDERLAY:-}" ]; then
  source /usr/local/bin/stride_env.sh
fi
[ -f "${ws}/install/setup.bash" ] || { echo "check_consumer: build stride-lo first (scripts/build.sh)"; exit 1; }
source "${ws}/install/setup.bash"
export ROS_HOME="${ROS_HOME:-${cws}/ros_home}"

rm -rf "${cws:?}"
mkdir -p "${cws}/src"
cp -r "${ws}/examples/minimal_backend" "${cws}/src/"
echo "check_consumer: building the example against ${ws}/install"
(cd "$cws" && colcon build --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release)
source "${cws}/install/setup.bash"
echo "check_consumer: build OK"
[ -n "$bag" ] || exit 0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
pids=()
cleanup() {
  # Each job runs in its own session (setsid): signal the whole process group, so launch's
  # children (respawning lo_node, supervisor) never outlive the check. SIGTERM, not SIGINT:
  # background jobs of a non-interactive shell ignore SIGINT.
  for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null || true; done
  for _ in $(seq 20); do
    alive=0
    for p in "${pids[@]}"; do kill -0 -- "-$p" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && return
    sleep 0.5
  done
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
}
trap cleanup EXIT
setsid ros2 launch lo_ros stride_lo.launch.py robot:="$robot" use_sim_time:=true requests:=true \
  rviz:=false respawn:=false > "${cws}/launch.log" 2>&1 &
pids+=($!)
# the executable itself, not `ros2 run` (its Python wrapper does not forward signals)
setsid "$(ros2 pkg prefix stride_minimal_backend)/lib/stride_minimal_backend/minimal_backend" \
  --ros-args -p use_sim_time:=true \
  > "${cws}/backend.log" 2>&1 &
backend=$!
pids+=($backend)
sleep 4
echo "check_consumer: playing ${dur} s of $bag"
ros2 bag play "$bag" --clock 200 --playback-duration "$dur" \
  --topics /livox/lidar /livox/imu /imu/data /joint_states /estimator/odometry /tf_static \
  > "${cws}/play.log" 2>&1
sleep 2
kill -TERM "$backend" 2>/dev/null || true
wait "$backend" 2>/dev/null || true
line="$(grep -E 'factors: ' "${cws}/backend.log" | tail -1)"
echo "check_consumer: ${line}"
acc="$(sed -nE 's/.*factors: ([0-9]+) accepted.*/\1/p' <<<"$line")"
unm="$(sed -nE 's/.* ([0-9]+) unmatched.*/\1/p' <<<"$line")"
rej="$(sed -nE 's/.* ([0-9]+) rejected.*/\1/p' <<<"$line")"
if [ "${acc:-0}" -gt 0 ] && [ "${rej:-1}" -eq 0 ] && [ "${unm:-1}" -eq 0 ]; then
  echo "check_consumer: PASS"
else
  echo "check_consumer: FAIL (see ${cws}/backend.log, ${cws}/launch.log)"
  exit 1
fi
