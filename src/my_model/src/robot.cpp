
#include <robot.hpp>

/**
 * @brief The constructor for the 'SimAgent' class that initializes the agent's
 * position, velocity, and heading
 *
 */
Robot::Robot(std::vector<double> initial_pos,
                   std::vector<double> initial_vel) {
  position[0] = initial_pos[0];
  position[1] = initial_pos[1];
  heading = initial_pos[2];
  velocity = initial_vel;
}

/**
 * @brief The function that updates the agent's position, velocity, and heading
 * based on the input received from the high-level planner
 *
 * @param current_pos The current position of the agent
 * @param current_vel The current velocity of the agent
 * @return * void
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
 * @brief The function that updates the desired velocity of the agent
 *
 * @param desired_vel The desired velocity of the agent
 * @return * void
 */
void Robot::update_desired_vel(std::vector<double> desired_vel) {
  desired_velocity = desired_vel;
}

/**
 * @brief The function that updates the desired position of the agent
 *
 * @param desired_pos The desired position of the agent
 * @return * void
 */
void Robot::update_desired_pos(std::vector<double> desired_pos) {
  desired_position = desired_pos;
}

/**
 * @brief The function that gets the desired velocity of the agent from RVO
 *
 * @return * std::vector<double> The desired velocity of the agent
 */
std::vector<double> Robot::getRobotDesiredVelocity() {
  return desired_velocity;
}

/**
 * @brief The function that gets the desired position of the agent from RVO
 *
 * @return * std::vector<double> The desired position of the agent
 */
std::vector<double> Robot::getRobotDesiredPosition() {
  return desired_position;
}

/**
 * @brief The destructor for the 'SimAgent' class
 *
 */
Robot::~Robot() {}