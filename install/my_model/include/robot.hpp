/**
 * @file robot.hpp
 * @author Rishie Raj
 * @brief 
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <memory>
#include <vector>

#pragma once

/**
 * @brief 
 *
 */
class Robot {
 private:
  std::vector<double> position{0.0f, 0.0f};
  double heading;
  std::vector<double> velocity;
  std::vector<double> desired_velocity;
  std::vector<double> desired_position;

 public:
  Robot(std::vector<double> initial_pos, std::vector<double> initial_vel);
  void update_desired_vel(std::vector<double> desired_vel);
  void update_desired_pos(std::vector<double> desired_pos);
  void update_robot(std::vector<double> current_pos,
                    std::vector<double> current_vel);
  std::vector<double> getRobotDesiredVelocity();
  std::vector<double> getRobotDesiredPosition();
  ~Robot();
};