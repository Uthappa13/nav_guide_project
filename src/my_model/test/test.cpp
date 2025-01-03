#include <gtest/gtest.h>

#include "robot.hpp"
#include "workspace.hpp"


TEST(BasicTest, Case1) { EXPECT_EQ(1, 1); }


TEST(BasicTest, Case2) { ASSERT_NEAR(1, 1.01, 0.1); }


TEST(BasicTest1, Case3) { EXPECT_STREQ("GoTerps", "GoTerps"); }

const int NO_OF_ROBOTS = 4;
float timestep_val = 0.1;
RVO::Vector2 robot_1_goal(5.0f, 5.0f);
RVO::Vector2 robot_2_goal(-5.0f, -5.0f);
RVO::Vector2 robot_3_goal(-5.0f, 5.0f);
RVO::Vector2 robot_4_goal(5.0f, -5.0f);
std::vector<RVO::Vector2> ROBOT_GOALS{robot_1_goal, robot_2_goal, robot_3_goal,
                                      robot_4_goal};
std::vector<double> robot_1_pose{-5.0f, -5.0f, 0.0};
std::vector<double> robot_2_pose{5.0f, 5.0f, 0.0};
std::vector<double> robot_3_pose{5.0f, -5.0f, 0.0};
std::vector<double> robot_4_pose{-5.0f, 5.0f, 0.0};
std::vector<std::vector<double>> start_positions{robot_1_pose, robot_2_pose,
                                                 robot_3_pose, robot_4_pose};
std::vector<std::vector<double>> current_vel{{0, 0}, {0, 0}, {0, 0}, {0, 0}};


class WorkspaceTests : public testing::Test {
 public:
 protected:
  void SetUp() override {
    testWorkspace = new Workspace(NO_OF_ROBOTS, timestep_val,
                                      start_positions, ROBOT_GOALS);
    std::cout << "Calling Fixture SetUp\n";
  };

  void TearDown() override {
    delete testWorkspace;
    std::cout << "Calling Fixture TearDown\n";
  };
  Workspace *testWorkspace;
};


TEST_F(WorkspaceTests, test_perform_iteration) {
  testWorkspace->perform_iteration();
  testWorkspace->update_environment(NO_OF_ROBOTS, start_positions,
                                      current_vel);
  std::vector<std::vector<double>> v1 =
      testWorkspace->getRobotDesiredVelocities();
  std::vector<std::vector<double>> v2 =
      testWorkspace->getRobotDesiredPositions();
}

std::vector<double> initial_pos{0, 0};
std::vector<double> initial_vel{0, 0};
std::vector<double> des_vel{0, 0};
std::vector<double> des_pos{0, 0};


class RobotTests : public testing::Test {
 public:
 protected:
  void SetUp() override {
    testRobot = new Robot(initial_pos, initial_vel);
    std::cout << "Calling Fixture SetUp\n";
  };

  void TearDown() override {
    delete testRobot;
    std::cout << "Calling Fixture TearDown\n";
  };
  Robot *testRobot;
};


TEST_F(RobotTests, test_perform_iteration) {
  testRobot->update_desired_vel(des_vel);
  testRobot->update_desired_pos(des_pos);
  testRobot->update_robot(initial_pos, initial_vel);
  std::vector<double> v1 = testRobot->getRobotDesiredVelocity();
  std::vector<double> v2 = testRobot->getRobotDesiredPosition();
}