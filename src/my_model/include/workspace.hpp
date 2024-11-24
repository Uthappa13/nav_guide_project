#include <RVO.h>

#include <memory>
#include <vector>

#include "robot.hpp"

#pragma once

/**
 * @brief The class that manages all the agents and runs iterations of the
 * simulation
 *
 */
class Workspace {
 private:
  std::vector<Robot> robots;
  RVO::RVOSimulator *sim;
  std::vector<RVO::Vector2> robot_goals;

 public:
  Workspace(int num_robots, float timestep,
              std::vector<std::vector<double>> start_positions,
              std::vector<RVO::Vector2> goal_positions);
  void update_environment(int num_robots,
                          std::vector<std::vector<double>> current_positions,
                          std::vector<std::vector<double>> current_velocities);
  void perform_iteration();
  std::vector<std::vector<double>> getRobotDesiredVelocities();
  std::vector<std::vector<double>> getRobotDesiredPositions();
  ~Workspace();
};
