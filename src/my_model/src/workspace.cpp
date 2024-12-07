#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "robot.hpp"
#include "workspace.hpp"
#include "Vector2.h"

/**
 * @brief Constructs a new Workspace object.
 * 
 * Initializes the simulation environment, agents, and their goal positions.
 * Sets up the RVO simulator and adds agents to the simulation with default parameters.
 *
 * @param num_robots The number of robots in the simulation.
 * @param timestep The time step for the simulation.
 * @param start_positions A vector of vectors containing the starting positions {x, y} for each robot.
 * @param goal_positions A vector of RVO::Vector2 representing the goal positions for each robot.
 */
Workspace::Workspace(int num_robots, float timestep,
                     std::vector<std::vector<double>> start_positions,
                     std::vector<RVO::Vector2> goal_positions) {
  sim = new RVO::RVOSimulator();
  std::srand(static_cast<unsigned int>(std::time(NULL)));
  sim->setTimeStep(timestep);
  sim->setAgentDefaults(5.0f, 5, 10.0f, 5.0f, 0.5f, 2.0f);
  for (int i = 0; i < num_robots; i++) {
    RVO::Vector2 startPosition = {static_cast<float>(start_positions[i][0]),
                                  static_cast<float>(start_positions[i][1])};
    sim->addAgent(startPosition);
    robot_goals.push_back(goal_positions[i]);
    std::vector<double> initial_pos = start_positions[i];
    std::vector<double> initial_vel{0.0f, 0.0f};
    Robot robot(initial_pos, initial_vel);
    robots.push_back(robot);
  }
}

/**
 * @brief Performs a single iteration of the simulation.
 * 
 * Updates the preferred velocities of all agents, executes a simulation step, 
 * and stores the updated positions and velocities for each robot.
 */
void Workspace::perform_iteration() {
  for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    RVO::Vector2 goalVector = robot_goals[i] - sim->getAgentPosition(i);
    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }
    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.0001f / RAND_MAX;
    sim->setAgentPrefVelocity(i, goalVector);
    sim->setAgentPrefVelocity(
        i, sim->getAgentPrefVelocity(i) +
               dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
  sim->doStep();
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    RVO::Vector2 velocity = sim->getAgentVelocity(i);
    RVO::Vector2 position = sim->getAgentPosition(i);
    std::vector<double> desired_vel{velocity.x(), velocity.y()};
    std::vector<double> desired_pos{position.x(), position.y()};
    robots[i].update_desired_vel(desired_vel);
    robots[i].update_desired_pos(desired_pos);
  }
}

/**
 * @brief Retrieves the desired velocities of all robots.
 * 
 * @return std::vector<std::vector<double>> A vector containing the desired velocities {vx, vy} for each robot.
 */
std::vector<std::vector<double>> Workspace::getRobotDesiredVelocities() {
  std::vector<std::vector<double>> robot_velocities;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_velocities.push_back(robots[i].getRobotDesiredVelocity());
  }
  return robot_velocities;
}

/**
 * @brief Retrieves the desired positions of all robots.
 * 
 * @return std::vector<std::vector<double>> A vector containing the desired positions {x, y} for each robot.
 */
std::vector<std::vector<double>> Workspace::getRobotDesiredPositions() {
  std::vector<std::vector<double>> robot_positions;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_positions.push_back(robots[i].getRobotDesiredPosition());
  }
  return robot_positions;
}

/**
 * @brief Updates the state of the environment.
 * 
 * Updates the positions and velocities of all robots in the simulation based on the provided inputs.
 *
 * @param num_robots The number of robots in the simulation.
 * @param current_positions A vector of vectors containing the current positions {x, y, heading} of each robot.
 * @param current_velocities A vector of vectors containing the current velocities {vx, vy} of each robot.
 */
void Workspace::update_environment(
    int num_robots, std::vector<std::vector<double>> current_positions,
    std::vector<std::vector<double>> current_velocities) {
  for (int i = 0; i < num_robots; i++) {
    RVO::Vector2 currentPosition = {static_cast<float>(current_positions[i][0]),
                                    static_cast<float>(current_positions[i][1])};
    RVO::Vector2 currentVelocity = {
        static_cast<float>(current_velocities[i][0]),
        static_cast<float>(current_velocities[i][1])};
    std::vector<double> current_pos{current_positions[i][0],
                                    current_positions[i][1],
                                    current_positions[i][2]};
    std::vector<double> current_vel{current_velocities[i][0],
                                    current_velocities[i][1]};
    robots[i].update_robot(current_pos, current_vel);
  }
}

/**
 * @brief Destroys the Workspace object.
 * 
 * Cleans up dynamically allocated resources.
 */
Workspace::~Workspace() { delete sim; }
