# Request a LoFactor that ends exactly at a graph node time (docs/DECISIONS.md D1, I12).
# STRIDE deskews the scan whose time span contains `stamp` to that time and emits
# LoFactor(stamp_i = previous emitted node, stamp_j = stamp). A backend publishes one request per
# keyframe as soon as it chooses the keyframe time (e.g. factor_graph_estimator: every 100 ms on
# an IMU stamp).
builtin_interfaces/Time stamp
