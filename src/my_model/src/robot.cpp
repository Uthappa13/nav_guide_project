#include <robot.hpp>

/**
 * @brief Constructs a new Robot object.
 * 
 * Initializes the robot's position, velocity, and heading based on 
 * the provided initial values.
 *
 * @param initial_pos A vector containing the initial position {x, y, heading} of the robot.
 * @param initial_vel A vector containing the initial velocity {vx, vy} of the robot.
 */
Robot::Robot(std::vector<double> initial_pos,
             std::vector<double> initial_vel) {
  position[0] = initial_pos[0];
  position[1] = initial_pos[1];
  heading = initial_pos[2];
  velocity = initial_vel;
}

/**
 * @brief Updates the robot's current position, velocity, and heading.
 * 
 * This function is used to update the robot's state based on input 
 * received from a high-level planner or sensors.
 *
 * @param current_pos A vector containing the current position {x, y, heading} of the robot.
 * @param current_vel A vector containing the current velocity {vx, vy} of the robot.
 */
void Robot::update_robot(std::vector<double> current_pos,
                         std::vector<double> current_vel) {
  position[0] = current_pos[0];
  position[1] = current_pos[1];
  heading = current_pos[2];
  velocity[0] = current_vel[0];
  velocity[1] = current_vel[1];
}

/**
 * @brief Updates the robot's desired velocity.
 * 
 * This function sets the desired velocity that the robot aims to achieve.
 *
 * @param desired_vel A vector containing the desired velocity {vx, vy} of the robot.
 */
void Robot::update_desired_vel(std::vector<double> desired_vel) {
  desired_velocity = desired_vel;
}

/**
 * @brief Updates the robot's desired position.
 * 
 * This function sets the desired position that the robot aims to reach.
 *
 * @param desired_pos A vector containing the desired position {x, y} of the robot.
 */
void Robot::update_desired_pos(std::vector<double> desired_pos) {
  desired_position = desired_pos;
}

/**
 * @brief Retrieves the robot's desired velocity.
 * 
 * This function returns the velocity vector that the robot is attempting to achieve.
 *
 * @return std::vector<double> A vector representing the robot's desired velocity {vx, vy}.
 */
std::vector<double> Robot::getRobotDesiredVelocity() {
  return desired_velocity;
}

/**
 * @brief Retrieves the robot's desired position.
 * 
 * This function returns the position vector that the robot is attempting to reach.
 *
 * @return std::vector<double> A vector representing the robot's desired position {x, y}.
 */
std::vector<double> Robot::getRobotDesiredPosition() {
  return desired_position;
}

/**
 * @brief Destroys the Robot object.
 * 
 * The destructor cleans up any resources used by the Robot instance.
 */
Robot::~Robot() {}
