#include <gtest/gtest.h>

#include "lo_core/preprocess/self_filter.hpp"

using namespace lo;

namespace {
std::vector<KinJointSpec> oneLeg()
{
  KinJointSpec hfe;
  hfe.name = "hfe";
  hfe.parent_link = "base";
  hfe.child_link = "thigh";
  hfe.origin = SE3(SO3(), Vec3(0.3, 0.1, 0.0));
  hfe.axis = Vec3::UnitY();
  hfe.type = JointType::REVOLUTE;
  KinJointSpec foot;
  foot.name = "foot_joint";
  foot.parent_link = "thigh";
  foot.child_link = "foot";
  foot.origin = SE3(SO3(), Vec3(0.0, 0.0, -0.4));
  foot.type = JointType::FIXED;
  KinJointSpec cam;   // rigid sensor link on the base: not a moving link
  cam.name = "cam_joint";
  cam.parent_link = "base";
  cam.child_link = "camera";
  cam.origin = SE3(SO3(), Vec3(0.4, 0.0, 0.1));
  return {hfe, foot, cam};
}
}  // namespace

TEST(Kinematics, CapsulesFollowJoints)
{
  KinematicModel m;
  std::string err;
  ASSERT_TRUE(KinematicModel::build("base", oneLeg(), 0.04, 0.03, m, err)) << err;
  ASSERT_EQ(m.numJoints(), 1);
  EXPECT_EQ(m.jointNames()[0], "hfe");
  ASSERT_EQ(m.numCapsules(), 2);   // foot sphere + thigh bone
  std::vector<Vec3> a(2), b(2);
  double q = 0.0;
  m.capsules(&q, a.data(), b.data());
  // thigh bone from (0.3, 0.1, 0) to (0.3, 0.1, -0.4) at q = 0
  bool found = false;
  for (int c = 0; c < 2; ++c) {
    if ((a[c] - Vec3(0.3, 0.1, 0.0)).norm() < 1e-9 && (b[c] - Vec3(0.3, 0.1, -0.4)).norm() < 1e-9) {found = true;}
  }
  EXPECT_TRUE(found);
  q = M_PI / 2;   // leg swung forward: foot at (0.3 - 0.4, ...)? rotation about +y
  m.capsules(&q, a.data(), b.data());
  bool swung = false;
  for (int c = 0; c < 2; ++c) {if ((b[c] - Vec3(0.3 - 0.4, 0.1, 0.0)).norm() < 1e-6) {swung = true;}}
  EXPECT_TRUE(swung);
}

TEST(SelfFilter, BodyBoxLegCapsulesAndJointsLostFallback)
{
  Config cfg;
  cfg.self_filter.body_box = {-0.2, -0.2, -0.1, 0.2, 0.2, 0.1};
  cfg.self_filter.leg_box = {-0.6, -0.4, -0.6, 0.6, 0.4, 0.1};
  Extrinsics ext;   // LiDAR at base origin
  auto model = std::make_shared<KinematicModel>();
  std::string err;
  ASSERT_TRUE(KinematicModel::build("base", oneLeg(), 0.04, 0.03, *model, err));
  SelfFilter sf(cfg, ext, model);
  TimeRing<JointSample> joints(100);
  for (int i = 0; i < 50; ++i) {
    JointSample s;
    s.t = fromSec(0.01 * i);
    s.n = 1;
    s.q[0] = 0.0;
    joints.push(s);
  }
  ASSERT_TRUE(sf.prepare(fromSec(0.1), fromSec(0.2), joints));
  EXPECT_TRUE(sf.isSelf(Vec3(0.0, 0.0, 0.0), fromSec(0.15)));          // body box
  EXPECT_TRUE(sf.isSelf(Vec3(0.3, 0.1, -0.2), fromSec(0.15)));         // on the thigh
  EXPECT_TRUE(sf.isSelf(Vec3(0.3, 0.1 + 0.06, -0.2), fromSec(0.15)));  // within r + inflate
  EXPECT_FALSE(sf.isSelf(Vec3(0.3, 0.1 + 0.09, -0.2), fromSec(0.15)));
  EXPECT_FALSE(sf.isSelf(Vec3(3.0, 0.0, 0.0), fromSec(0.15)));         // far away
  // joints stale -> static leg box
  ASSERT_FALSE(sf.prepare(fromSec(5.0), fromSec(5.1), joints));
  EXPECT_TRUE(sf.jointsMissing());
  EXPECT_TRUE(sf.isSelf(Vec3(0.5, 0.3, -0.5), fromSec(5.05)));
}
