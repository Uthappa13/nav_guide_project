/**
 * @file workspace.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "robot.hpp"
#include "workspace.hpp"
#include "Vector2.h"

/**
 * @brief The constructor for the 'Environment' class that initializes the
 * simulation and agents (and their goal positions)
 *
 */
Workspace::Workspace(int num_robots, float timestep,
                         std::vector<std::vector<double>> start_positions,
                         std::vector<RVO::Vector2> goal_positions) {
  sim = new RVO::RVOSimulator();
  // Random Number Generator Seed
  std::srand(static_cast<unsigned int>(std::time(NULL)));
  // Set time step
  sim->setTimeStep(timestep);
  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(5.0f, 10, 10.0f, 5.0f, 0.5f, 2.0f);
  for (int i = 0; i < num_robots; i++) {
    RVO::Vector2 startPositon = {static_cast<float>(start_positions[i][0]),
                                 static_cast<float>(start_positions[i][1])};
    sim->addAgent(startPositon);

    robot_goals.push_back(goal_positions[i]);

    std::vector<double> initial_pos = start_positions[i];
    std::vector<double> initial_vel{0.0f, 0.0f};
    Robot robot(initial_pos, initial_vel);
    robots.push_back(robot);
  }
}

/**
 * @brief The function that calls all the sub-functions to perform a single
 * iteration/timestep of the simulation
 *
 * @return * void
 */
void Workspace::perform_iteration() {
  // Set preferred Velocity Step
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
  // Perform Sim Step
  sim->doStep();
  // Store Current State in Sim Agent
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    RVO::Vector2 velocity = sim->getAgentVelocity(i);
    RVO::Vector2 positon = sim->getAgentPosition(i);
    std::vector<double> desired_vel{velocity.x(), velocity.y()};
    std::vector<double> desired_pos{positon.x(), positon.y()};
    robots[i].update_desired_vel(desired_vel);
    robots[i].update_desired_pos(desired_pos);
  }
}

/**
 * @brief The function that gets the desired velocities of all the agents from
 * RVO
 *
 * @return * std::vector<std::vector<double>> The desired velocities of all the
 * agents
 */
std::vector<std::vector<double>> Workspace::getRobotDesiredVelocities() {
  std::vector<std::vector<double>> robot_velocities;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_velocities.push_back(robots[i].getRobotDesiredVelocity());
  }
  return robot_velocities;
}

/**
 * @brief The function that gets the desired positions of all the agents from
 * RVO
 *
 * @return * std::vector<std::vector<double>> The desired positions of all the
 * agents
 */
std::vector<std::vector<double>> Workspace::getRobotDesiredPositions() {
  std::vector<std::vector<double>> robot_positions;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_positions.push_back(robots[i].getRobotDesiredPosition());
  }
  return robot_positions;
}

/**
 * @brief The function that updates each agent in the ongoing iteration/timestep
 * of the simulation
 *
 * @param num_agents The total number of agents in the simulation
 * @param current_positions The current positions of the agents
 * @param current_velocities The curret velocities of the agents
 * @return * void
 */
void Workspace::update_environment(
    int num_robots, std::vector<std::vector<double>> current_positions,
    std::vector<std::vector<double>> current_velocities) {
  for (int i = 0; i < num_robots; i++) {
    RVO::Vector2 currentPositon = {static_cast<float>(current_positions[i][0]),
                                   static_cast<float>(current_positions[i][1])};
    RVO::Vector2 currentVelocity = {
        static_cast<float>(current_velocities[i][0]),
        static_cast<float>(current_velocities[i][1])};
    sim->setAgentVelocity(i,currentVelocity);
    sim->setAgentPosition(i,currentPositon);
    std::vector<double> current_pos{current_positions[i][0],
                                    current_positions[i][1],
                                    current_positions[i][2]};
    std::vector<double> current_vel{current_velocities[i][0],
                                    current_velocities[i][1],
                                    current_velocities[i][2]};
    robots[i].update_robot(current_pos, current_vel);
  }
}

/**
 * @brief The destructor of the Environment class that deallocates all
 * dynamically created memory
 *
 */
Workspace::~Workspace() { delete sim; }