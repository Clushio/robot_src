#include <gtest/gtest.h>

#include <limits>

#include <jgl_dwa_local_planner/terminal_yaw_controller.h>

namespace
{

using jgl_dwa_local_planner::TerminalYawController;

TEST(TerminalYawControllerTest, UsesProportionalCommandAndConfiguredBounds)
{
  TerminalYawController controller;
  controller.configure(0.04, 1.2, 0.25, 0.06, 3);

  double command = 0.0;
  EXPECT_FALSE(controller.update(0.50, &command));
  EXPECT_DOUBLE_EQ(0.25, command);
  EXPECT_FALSE(controller.update(-0.50, &command));
  EXPECT_DOUBLE_EQ(-0.25, command);
  EXPECT_FALSE(controller.update(0.041, &command));
  EXPECT_DOUBLE_EQ(0.06, command);
}

TEST(TerminalYawControllerTest, RequiresConsecutiveStableCyclesAtZeroCommand)
{
  TerminalYawController controller;
  controller.configure(0.04, 1.2, 0.25, 0.06, 3);

  double command = 1.0;
  EXPECT_FALSE(controller.update(0.03, &command));
  EXPECT_DOUBLE_EQ(0.0, command);
  EXPECT_EQ(1, controller.stableCycles());
  EXPECT_FALSE(controller.update(-0.02, &command));
  EXPECT_EQ(2, controller.stableCycles());
  EXPECT_TRUE(controller.update(0.01, &command));
  EXPECT_EQ(3, controller.stableCycles());
}

TEST(TerminalYawControllerTest, ErrorOutsideToleranceResetsStableSequence)
{
  TerminalYawController controller;
  controller.configure(0.04, 1.2, 0.25, 0.06, 3);

  double command = 0.0;
  EXPECT_FALSE(controller.update(0.01, &command));
  EXPECT_FALSE(controller.update(0.02, &command));
  EXPECT_EQ(2, controller.stableCycles());
  EXPECT_FALSE(controller.update(0.05, &command));
  EXPECT_EQ(0, controller.stableCycles());
  EXPECT_GT(command, 0.0);
  EXPECT_FALSE(controller.update(0.01, &command));
  EXPECT_FALSE(controller.update(0.01, &command));
  EXPECT_TRUE(controller.update(0.01, &command));
}

TEST(TerminalYawControllerTest, NonFiniteErrorCannotCompleteGoal)
{
  TerminalYawController controller;
  controller.configure(0.04, 1.2, 0.25, 0.06, 1);

  double command = 1.0;
  EXPECT_FALSE(controller.update(
      std::numeric_limits<double>::quiet_NaN(), &command));
  EXPECT_DOUBLE_EQ(0.0, command);
  EXPECT_EQ(0, controller.stableCycles());
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
