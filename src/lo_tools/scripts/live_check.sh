#!/usr/bin/env bash
# Live end-to-end check (docs/PLAN.md phase 4): ros2 bag play --clock -> lo_node (lifecycle,
# supervisor, request mode) <-> lo_graph_demo (NodeRequests on IMU stamps, iSAM2), recorded
# and evaluated against ground truth with evaluate.py.
#   live_check.sh BAG [OUT_DIR] [DURATION_S] [ROBOT] [GT_TOPIC]
set -eo pipefail
bag="${1:?usage: live_check.sh BAG [OUT_DIR] [DURATION_S] [ROBOT] [GT_TOPIC]}"
out="${2:-live_out}"
dur="${3:-60}"
robot="${4:-anymal_c}"
gt="${5:-/sim/raw/odom}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"   # host networking: keep off other ROS graphs
tools="$(ros2 pkg prefix lo_tools)/lib/lo_tools"
rm -rf "${out:?}"
pids=()
cleanup() {
  # SIGTERM: background jobs of a non-interactive shell ignore SIGINT
  for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

ros2 launch lo_ros stride_lo.launch.py robot:="$robot" use_sim_time:=true requests:=true \
  > "${out}.launch.log" 2>&1 &
pids+=($!)
ros2 run lo_gtsam_adapter lo_graph_demo --ros-args -p use_sim_time:=true -p imu_topic:=/imu/data \
  > "${out}.demo.log" 2>&1 &
pids+=($!)
sleep 3
ros2 bag record -s mcap -o "$out" /stride_lo/factor /stride_lo/odometry /stride_lo/node_request \
  /diagnostics /lo_graph_demo/pose "$gt" /tf_static > "${out}.record.log" 2>&1 &
rec=$!
sleep 2
echo "live_check: playing ${dur} s of $bag"
ros2 bag play "$bag" --clock 200 --playback-duration "$dur" \
  --topics /livox/lidar /livox/imu /imu/data /joint_states /estimator/odometry /tf_static "$gt" \
  > "${out}.play.log" 2>&1
sleep 2
kill -TERM "$rec" 2>/dev/null || true
wait "$rec" 2>/dev/null || true
echo "live_check: node log tail"
grep -E "INIT|TRACKING|LOST|activ|configur|ERROR|WARN" "${out}.launch.log" | tail -8 || true
grep -E "graph:|chain" "${out}.demo.log" | tail -3 || true
python3 "${tools}/evaluate.py" "$out" --gt "$gt"
