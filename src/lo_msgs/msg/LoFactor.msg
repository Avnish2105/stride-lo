# One IMU-free relative-pose measurement from STRIDE LiDAR odometry (docs/DESIGN.md "Interfaces").
# Fields are exactly those of the design; only constants are added (docs/DECISIONS.md I11).
#
# Backend use: gtsam::BetweenFactor<Pose3>(X(i), X(j), t_ij, Gaussian::Covariance(covariance)),
# dropped if chain_id differs between i and j (lo_gtsam_adapter does both).

# ---- health ---------------------------------------------------------------------------------
uint8 HEALTH_OK = 0
# >= 1 Hessian-degenerate direction: Sigma replaced (sigma 1 m / 0.5 rad) only along
# degenerate_directions, value there equals the seed (docs/DECISIONS.md D8).
uint8 HEALTH_DEGRADED = 1
# Whole-factor inflation: deskew fallback (x4), IMU lost, sensor blocked, factor spans rejected
# scans, or first factor after RECOVERING. If both apply, LOW_CONFIDENCE wins.
uint8 HEALTH_LOW_CONFIDENCE = 2

# ---- flags (bitfield) -----------------------------------------------------------------------
uint32 FLAG_IMU_MISSING = 1           # no usable IMU over the scan (deskew/seed from LO velocity)
uint32 FLAG_DESKEW_FALLBACK = 2       # IMU gap > 20 ms in the scan span: Sigma x4
uint32 FLAG_SEED_FALLBACK = 4         # seed not from gyro rotation + graph translation
uint32 FLAG_SEED_NO_GRAPH = 8         # translation seed from LO's last velocity (or identity)
uint32 FLAG_SEED_IDENTITY = 16        # at least one seed component had no prior at all
uint32 FLAG_SENSOR_BLOCKED = 32       # > 30% of returns within 0.5 m
uint32 FLAG_JOINTS_MISSING = 64       # self-filter used the static leg exclusion box
uint32 FLAG_DEGENERATE = 128          # see degenerate_directions
uint32 FLAG_SPANS_GAP = 256           # scans between i and j were rejected: Sigma = Sigma_i + Sigma_j
uint32 FLAG_AFTER_RECOVERY = 512      # first factor after LOST -> RECOVERING -> TRACKING
uint32 FLAG_GYRO_GATE_DISABLED = 1024 # IMU lost: gyro check off, plausibility limits tightened
uint32 FLAG_REQUESTED = 2048          # j answers a NodeRequest (else: overlap keyframe)
uint32 FLAG_BIAS_UNCALIBRATED = 4096  # gyro bias never estimated at standstill (no IMU at INIT)

# ---- payload (design doc) -------------------------------------------------------------------
builtin_interfaces/Time stamp_i      # reference time of frame i (= graph node time)
builtin_interfaces/Time stamp_j      # reference time of frame j
uint64 chain_id                      # increments on every chain break (map reset)
geometry_msgs/Pose  t_ij             # relative pose T_ij, base frame, i -> j (ROS 2 field names are lowercase)
float64[36] covariance               # row-major, order [rx ry rz tx ty tz]
uint8 health                         # 0 OK, 1 DEGRADED, 2 LOW_CONFIDENCE
float64[6] hessian_eigenvalues       # ascending, for backend diagnostics
float64[18] degenerate_directions    # up to 3 unit 6-vectors, unobservable axes
float32 inlier_ratio
float32 residual_rms
uint32  flags                        # bitfield: IMU_MISSING, DESKEW_FALLBACK, SEED_FALLBACK, ...
