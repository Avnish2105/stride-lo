#!/usr/bin/env bash
# Fault-injection suite (docs/DESIGN.md "0 false accepts").
#   run_fault_suite.sh                      the 11 synthetic fault scenarios (CI)
#   run_fault_suite.sh BAG [ROBOT] [WORK] [GT]  + every injectable fault on a recorded/sim bag
#                                              (first 100 s; GT default /ground_truth/odom)
# Exit status 0 only when every run has 0 false accepts (NEES > 40 against ground truth).
set -eo pipefail
fail=0
echo "== synthetic fault suite"
lo_synth_run suite || fail=1

if [ -n "${1:-}" ]; then
  bag="$1"
  robot="${2:-anymal_c}"
  work="${3:-${TMPDIR:-/tmp}/stride_fault_suite}"
  gt="${4:-/ground_truth/odom}"
  mkdir -p "${work:?}"
  tools="$(ros2 pkg prefix lo_tools)/lib/lo_tools"
  specs=(
    "imu_dropout:start=40,duration=1.5"
    "clock_jump:start=50,duration=3,offset=0.35"
    "ptp_drift:start=60,duration=5,rate=0.002"
    "point_dropout:fraction=0.5"
    "lens_blocked:start=70,duration=3"
    "leg_points:start=80,duration=10"
    "imu_vibration:accel_std=3.5,gyro_std=0.05"
  )
  for spec in "${specs[@]}"; do
    name="${spec%%:*}"
    echo "== bag fault ${name} (${spec})"
    rm -rf "${work:?}/${name:?}" "${work:?}/${name:?}_out"
    python3 "${tools}/fault_inject.py" "${bag}" "${work}/${name}" --fault "${spec}" --duration 100
    "${tools}/lo_replay" --bag "${work}/${name}" --robot "${robot}" --out "${work}/${name}_out" \
      --requests-from /estimator/odometry_keyframe --request-latency 0.005 --gt "${gt}" \
      | tail -2 || fail=1
    python3 "${tools}/evaluate.py" "${work}/${name}_out" --gt "${gt}" || fail=1
    rm -rf "${work:?}/${name:?}"
  done
fi
exit ${fail}
