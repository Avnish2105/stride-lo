# Environment for every shell in the STRIDE image: ROS, the pinned underlay, the workspace.
# Sourced by the entrypoint and by ~/.bashrc of the dev user. Not `set -u` safe (ROS scripts).
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /opt/stride_underlay/install/setup.bash ]; then
  source /opt/stride_underlay/install/setup.bash
fi
if [ -n "${STRIDE_WS:-}" ] && [ -f "${STRIDE_WS}/install/setup.bash" ]; then
  source "${STRIDE_WS}/install/setup.bash"
fi
