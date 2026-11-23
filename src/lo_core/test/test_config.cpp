#include <gtest/gtest.h>

#include "lo_core/config/config.hpp"

using namespace lo;

TEST(Config, DefaultsValidateAndHashIsStable)
{
  Config c;
  EXPECT_TRUE(validate(c).empty());
  EXPECT_EQ(configHash(c), configHash(Config()));
  Config d = c;
  d.cov.kappa = 7.5;
  EXPECT_NE(configHash(c), configHash(d));
  EXPECT_NE(configDump(c).find("cov.kappa=20"), std::string::npos);
}

TEST(Config, RejectsBadValues)
{
  Config c;
  c.range.min = 10.0;
  c.range.max = 5.0;
  c.emit.mode = "sometimes";
  c.extrinsics.base_lidar = {0, 0, 0, 0, 0, 0, 2};   // not a unit quaternion
  c.self_filter.body_box = {1, 0, 0, -1, 0, 0};
  const auto e = validate(c);
  auto has = [&](const std::string & s) {
      for (const auto & x : e) {if (x.find(s) != std::string::npos) {return true;}}
      return false;
    };
  EXPECT_TRUE(has("range.min"));
  EXPECT_TRUE(has("emit.mode"));
  EXPECT_TRUE(has("orthonormal"));
  EXPECT_TRUE(has("body_box"));
}
