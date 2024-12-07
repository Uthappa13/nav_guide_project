/**
 * @file workspace.hpp
 * @author Rishie Raj
 * @brief Defines the Workspace class to manage agents and simulate their interactions.
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <RVO.h>
#include <memory>
#include <vector>
#include "robot.hpp"

/**
 * @brief Manages all the agents and runs iterations of the simulation.
 * 
 * This class initializes and manages a collection of robots in a 
 * simulated environment using the Reciprocal Velocity Obstacle (RVO) algorithm.
 */
class Workspace {
 private:
  /**
   * @brief A collection of robots in the workspace.
   */
  std::vector<Robot> robots;

  /**
   * @brief Pointer to the RVOSimulator instance for managing the simulation.
   */
  RVO::RVOSimulator *sim;

  /**
   * @brief Goal positions for each robot in the workspace.
   */
  std::vector<RVO::Vector2> robot_goals;

 public:
  /**
   * @brief Constructs a new Workspace object.
   * 
   * @param num_robots Number of robots in the workspace.
   * @param timestep Simulation timestep duration.
   * @param start_positions Initial positions of the robots.
   * @param goal_positions Desired goal positions for each robot.
   */
  Workspace(int num_robots, float timestep,
            std::vector<std::vector<double>> start_positions,
            std::vector<RVO::Vector2> goal_positions);

  /**
   * @brief Updates the environment with the current state of the robots.
   * 
   * @param num_robots Number of robots in the workspace.
   * @param current_positions Current positions of the robots.
   * @param current_velocities Current velocities of the robots.
   */
  void update_environment(int num_robots,
                          std::vector<std::vector<double>> current_positions,
                          std::vector<std::vector<double>> current_velocities);

  /**
   * @brief Performs one iteration of the simulation.
   * 
   * This method updates the state of the simulation by computing the new 
   * positions and velocities for the robots based on their goals.
   */
  void perform_iteration();

  /**
   * @brief Gets the desired velocities for all robots.
   * 
   * @return A vector containing the desired velocities for each robot.
   */
  std::vector<std::vector<double>> getRobotDesiredVelocities();

  /**
   * @brief Gets the desired positions for all robots.
   * 
   * @return A vector containing the desired positions for each robot.
   */
  std::vector<std::vector<double>> getRobotDesiredPositions();

  /**
   * @brief Destroys the Workspace object.
   * 
   * Cleans up any allocated resources for the simulation.
   */
  ~Workspace();
};
