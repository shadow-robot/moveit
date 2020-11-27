#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <boost/math/constants/constants.hpp>

class SphericalRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("robot", "base_link");
    geometry_msgs::Pose origin;
    origin.orientation.w = 1;
    builder.addChain("base_link->roll", "revolute", { origin }, urdf::Vector3(1, 0, 0));
    builder.addChain("roll->pitch", "revolute", { origin }, urdf::Vector3(0, 1, 0));
    builder.addChain("pitch->yaw", "revolute", { origin }, urdf::Vector3(0, 0, 1));
    ASSERT_TRUE(builder.isValid());
    robot_model_ = builder.build();
  }

  std::map<std::string, double> getJointValues(const double roll, const double pitch, const double yaw)
  {
    std::map<std::string, double> jvals;
    jvals["base_link-roll-joint"] = roll;
    jvals["roll-pitch-joint"] = pitch;
    jvals["pitch-yaw-joint"] = yaw;
    return jvals;
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(SphericalRobot, Test1)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.8;
  ocm.absolute_y_axis_tolerance = 0.8;
  ocm.absolute_z_axis_tolerance = 3.15;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // This's identical to a -1.57rad rotation around Z-axis
  robot_state.setVariablePositions(getJointValues(3.140208, 3.141588, 1.570821));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state, true).satisfied);
}

TEST_F(SphericalRobot, Test2)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.2;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // Singularity: roll + yaw = theta
  robot_state.setVariablePositions(getJointValues(0.15, boost::math::constants::half_pi<double>(), 0.15));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state, true).satisfied);

  // Singularity: roll - yaw = theta
  robot_state.setVariablePositions(getJointValues(0.15, -boost::math::constants::half_pi<double>(), 0.15));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state, true).satisfied);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
