// Scenario catalogue + event-driven simulator. Messages are delivered to a Sink in arrival
// order, exactly like a bag replay, so a test can drive lo::Pipeline the same way lo_tools'
// replay and the ROS node do. Everything is deterministic for a given seed.
#pragma once

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "lo_core/types/inputs.hpp"
#include "lo_core/types/points.hpp"
#include "lo_synth/pattern.hpp"
#include "lo_synth/robot.hpp"
#include "lo_synth/scene.hpp"
#include "lo_synth/trajectory.hpp"

namespace lo::synth {

struct ImuSpec
{
  double rate = 200.0;                 ///< Hz (Livox internal IMU)
  double gyro_noise = 1.5e-4;          ///< rad/s/sqrt(Hz) white noise density
  double accel_noise = 1.0e-3;         ///< m/s^2/sqrt(Hz)
  double gyro_bias_rw = 2e-6;          ///< rad/s^2/sqrt(Hz)
  Vec3 gyro_bias{0.004, -0.003, 0.002};   ///< turn-on bias [rad/s]
  Vec3 accel_bias{0.02, -0.01, 0.03};     ///< [m/s^2]
  bool accel_in_g = true;              ///< Livox driver convention
  double latency = 0.001;              ///< arrival - stamp [s]
};

enum class FaultType {
  IMU_DROPOUT,      ///< no IMU samples in [start, start + duration]
  CLOCK_JUMP,       ///< LiDAR stamps jump by `magnitude` seconds at `start` (for `duration`)
  PTP_LOSS,         ///< LiDAR clock drifts by `magnitude` s/s during the window
  POINT_DROPOUT,    ///< fraction `magnitude` of points become no-returns during the window
  LENS_BLOCKED,     ///< half of the field of view returns dust at 0.15-0.4 m during the window
  JOINTS_LOST,      ///< no joint states during the window
};

struct Fault
{
  FaultType type{FaultType::IMU_DROPOUT};
  double start{10.0};
  double duration{2.0};
  double magnitude{0.0};
};

struct Scenario
{
  std::string name;
  std::uint64_t seed{1};
  double duration{30.0};
  Scene scene;
  Trajectory trajectory{Vec3(0, 0, 0.5), 0.0};
  SE3 T_base_lidar;                    ///< default: front-top, dome up, 20 deg pitched down
  SE3 T_lidar_imu;                     ///< Livox internal IMU offset
  Mid360Pattern pattern;
  double range_noise{0.02};
  double min_range{0.1};
  double max_range{70.0};
  float min_reflectivity{5.f};
  ImuSpec imu;
  double joint_rate{500.0};
  bool publish_joints{true};
  double request_rate{10.0};           ///< graph-node requests on IMU stamps (0 = none)
  double request_phase{0.0123};        ///< request times = k / rate + phase
  double scan_latency{0.005};          ///< scan arrival - scan end [s]
  int leg_override{-1};
  std::array<double, 3> leg_override_q{0.0, -1.6, 2.2};
  LegGeometry legs;
  std::vector<Fault> faults;
  /// The pipeline is expected to degrade (corridor, open field): directions that may be
  /// unobservable; used by the fault suite to judge degeneracy handling.
  bool expect_degenerate{false};
};

/// Names: room, warehouse, corridor, open_field, stairs, crowd, turn180, imu_dropout,
/// clock_jump, ptp_loss, point_dropout, lens_blocked, leg_in_fov.
std::vector<std::string> scenarioNames();
/// The 11 fault-injection scenarios of docs/DESIGN.md (suite order).
std::vector<std::string> faultSuiteNames();
Scenario makeScenario(const std::string & name, std::uint64_t seed = 1);

class Sink
{
public:
  virtual ~Sink() = default;
  virtual void onImu(const ImuSample & s, TimeNs arrival) = 0;
  virtual void onJoints(const JointSample & s, TimeNs arrival) = 0;
  virtual void onRequest(TimeNs stamp, TimeNs arrival) = 0;
  /// `true_t0`: scan start in true time (differs from scan.t0 under clock faults).
  virtual void onScan(const RawScan & scan, TimeNs arrival, TimeNs true_t0) = 0;
};

class Simulator
{
public:
  explicit Simulator(Scenario sc);

  /// Run the whole scenario (or until t_max seconds), delivering messages in arrival order.
  void run(Sink & sink, double t_max = -1.0);

  const Scenario & scenario() const {return sc_;}
  const SyntheticRobot & robot() const {return robot_;}
  /// Ground truth base pose (true time).
  SE3 groundTruth(TimeNs t_true) const;
  /// LiDAR clock offset (stamp - true time) at true time t [s].
  double lidarClockOffset(double t_true) const;
  /// Approximate inverse: true time for a LiDAR stamp.
  TimeNs trueTimeOfLidarStamp(TimeNs stamp) const;
  Extrinsics extrinsics() const {return Extrinsics{sc_.T_base_lidar, sc_.T_lidar_imu};}

private:
  struct Event
  {
    TimeNs arrival{0};
    int type{0};          // 0 imu, 1 joints, 2 request, 3 scan
    std::size_t index{0};
  };

  void generateImu();
  void generateJoints();
  void generateRequests();
  void generateScan(std::size_t k, std::vector<RawPoint> & out, TimeNs & stamp, TimeNs & true_t0);
  bool faultActive(FaultType type, double t, const Fault ** f = nullptr) const;

  Scenario sc_;
  SyntheticRobot robot_;
  std::vector<ImuSample> imu_;
  std::vector<TimeNs> imu_arrival_;
  std::vector<JointSample> joints_;
  std::vector<TimeNs> requests_;
  std::mt19937_64 rng_;
  std::vector<RawPoint> scan_buf_;
  std::vector<Vec3> cap_a_, cap_b_;
  std::vector<double> cap_r_;
};

}  // namespace lo::synth
