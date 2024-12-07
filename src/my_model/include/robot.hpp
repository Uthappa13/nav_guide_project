/**
 * @file robot.hpp
 * @author Rishie Raj
 * @brief Defines the Robot class for managing individual robot state and behavior.
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <memory>
#include <vector>

/**
 * @brief Represents a robot with position, velocity, and desired state attributes.
 * 
 * The Robot class encapsulates the state and behavior of an individual robot,
 * including its current position, velocity, and desired states, and provides
 * methods to update and retrieve these attributes.
 */
class Robot {
 private:
  /**
   * @brief The current position of the robot.
   * 
   * Represented as a 2D vector: {x, y}.
   */
  std::vector<double> position{0.0f, 0.0f};

  /**
   * @brief The current heading direction of the robot in radians.
   */
  double heading;

  /**
   * @brief The current velocity of the robot.
   * 
   * Represented as a 2D vector: {vx, vy}.
   */
  std::vector<double> velocity;

  /**
   * @brief The desired velocity for the robot.
   * 
   * Represented as a 2D vector: {vx, vy}.
   */
  std::vector<double> desired_velocity;

  /**
   * @brief The desired position for the robot.
   * 
   * Represented as a 2D vector: {x, y}.
   */
  std::vector<double> desired_position;

 public:
  /**
   * @brief Constructs a new Robot object.
   * 
   * @param initial_pos Initial position of the robot as a 2D vector.
   * @param initial_vel Initial velocity of the robot as a 2D vector.
   */
  Robot(std::vector<double> initial_pos, std::vector<double> initial_vel);

  /**
   * @brief Updates the desired velocity of the robot.
   * 
   * @param desired_vel The new desired velocity as a 2D vector.
   */
  void update_desired_vel(std::vector<double> desired_vel);

  /**
   * @brief Updates the desired position of the robot.
   * 
   * @param desired_pos The new desired position as a 2D vector.
   */
  void update_desired_pos(std::vector<double> desired_pos);

  /**
   * @brief Updates the current state of the robot.
   * 
   * @param current_pos The current position of the robot as a 2D vector.
   * @param current_vel The current velocity of the robot as a 2D vector.
   */
  void update_robot(std::vector<double> current_pos,
                    std::vector<double> current_vel);

  /**
   * @brief Retrieves the robot's desired velocity.
   * 
   * @return A 2D vector representing the robot's desired velocity.
   */
  std::vector<double> getRobotDesiredVelocity();

  /**
   * @brief Retrieves the robot's desired position.
   * 
   * @return A 2D vector representing the robot's desired position.
   */
  std::vector<double> getRobotDesiredPosition();

  /**
   * @brief Destroys the Robot object.
   * 
   * Cleans up resources associated with the robot.
   */
  ~Robot();
};
