/**
 * @file main.cpp
 * @author Rishie Raj
 * @brief 
 * @version 0.1
 * @date 2024-12-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"

#include "workspace.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotCommandPublisher : public rclcpp::Node {
public:
  RobotCommandPublisher(int num_robots)
  : Node("robot_commander"), 
    num_robots(num_robots),
    orientation_complete(false),
    iteration_complete(false)
  {
    // Create start and goal positions for all robots
    std::vector<std::vector<double>> start_positions(num_robots, std::vector<double>(3, 0.0));
    std::vector<RVO::Vector2> goal_positions;

    double radius = 5.0;
    for (int i = 0; i < num_robots; i++) {
      double angle = 2 * M_PI * i / num_robots;
      double x = radius * std::cos(angle);
      double y = radius * std::sin(angle);
      start_positions[i][0] = x;
      start_positions[i][1] = y;
      start_positions[i][2] = 0.0; // initial yaw

      double goal_x = -x;
      double goal_y = -y;
      goal_positions.push_back(RVO::Vector2(goal_x, goal_y));
    }

    // Use num_robots instead of hardcoding 4 here
    testenv = new Workspace(num_robots, 0.5, start_positions, goal_positions);

    robot_poses = start_positions;
    // Create a vector of velocities for all robots
    robot_velocities.resize(num_robots, std::vector<double>(3,0.0));

    // Create publishers and subscribers
    velocity_pubs.resize(num_robots);
    odom_subs.resize(num_robots);

    for (int i = 0; i < num_robots; i++) {
      std::string robot_name = "robot" + std::to_string(i+1);
      velocity_pubs[i] = this->create_publisher<geometry_msgs::msg::Twist>("/" + robot_name + "/cmd_vel", 10);
      odom_subs[i] = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robot_name + "/odom", 10,
        [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_callback(msg, i);
        }
      );
    }

    // Create the timer
    timer = this->create_wall_timer(
        500ms, std::bind(&RobotCommandPublisher::timer_callback, this));
  }

private:

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  std::vector<double> extract_pose(nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    return {x, y, yaw};
  }

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg, int robot_idx) {
    robot_poses[robot_idx] = extract_pose(msg);
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double yaw_rate = msg->twist.twist.angular.z;

    robot_velocities[robot_idx][0] = vx;
    robot_velocities[robot_idx][1] = vy;
    robot_velocities[robot_idx][2] = yaw_rate;
  }

  void timer_callback() {
    // Create vectors current_positions, current_velocities from robot_poses and robot_velocities
    std::vector<std::vector<double>> current_poses(num_robots, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> current_vels(num_robots, std::vector<double>(3, 0.0));

    for (int i = 0; i < num_robots; i++) {
      current_poses[i][0] = robot_poses[i][0];
      current_poses[i][1] = robot_poses[i][1];
      current_poses[i][2] = robot_poses[i][2];

      current_vels[i][0] = robot_velocities[i][0];
      current_vels[i][1] = robot_velocities[i][1];
      current_vels[i][2] = robot_velocities[i][2]; // Actual yaw rate
    }

    // Update the environment with the current state
    testenv->update_environment(num_robots, current_poses, current_vels);

    if (!iteration_complete) {
      RCLCPP_INFO_STREAM(this->get_logger(), "New iteration");
      testenv->perform_iteration();
      iteration_complete = true;
      orientation_complete = false;
    }

    std::vector<std::vector<double>> robot_desired_positions = testenv->getRobotDesiredPositions();
    std::vector<double> desired_headings(num_robots, 0.0);
    for (int i = 0; i < num_robots; i++) {
      double dx = robot_desired_positions[i][0] - robot_poses[i][0];
      double dy = robot_desired_positions[i][1] - robot_poses[i][1];
      desired_headings[i] = std::atan2(dy, dx);
    }

    bool all_oriented = true;
    for (int i = 0; i < num_robots; i++) {
      geometry_msgs::msg::Twist vel_msg;
      double yaw_error = normalize_angle(desired_headings[i] - robot_poses[i][2]);
      if (std::fabs(yaw_error) > 0.1) {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = (yaw_error > 0) ? 0.05 : -0.05;
        all_oriented = false;
      } else {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
      }
      velocity_pubs[i]->publish(vel_msg);
    }

    orientation_complete = all_oriented;

    if (orientation_complete) {
      bool all_reached = true;
      for (int i = 0; i < num_robots; i++) {
        double dist = std::sqrt(
          std::pow(robot_desired_positions[i][0] - robot_poses[i][0], 2) +
          std::pow(robot_desired_positions[i][1] - robot_poses[i][1], 2)
        );
        geometry_msgs::msg::Twist vel_msg;
        if (dist > 0.05) {
          vel_msg.linear.x = 0.1;
          vel_msg.angular.z = 0.0;
          all_reached = false;
        } else {
          vel_msg.linear.x = 0.0;
          vel_msg.angular.z = 0.0;
        }
        velocity_pubs[i]->publish(vel_msg);
      }

      if (all_reached) {
        iteration_complete = false;
      }
    }
  }

  int num_robots;
  bool orientation_complete;
  bool iteration_complete;
  Workspace* testenv;
  rclcpp::TimerBase::SharedPtr timer;

  // Now these are the main members for multiple robots:
  std::vector<std::vector<double>> robot_poses;
  std::vector<std::vector<double>> robot_velocities;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> velocity_pubs;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int num_robots = 5;
  rclcpp::spin(std::make_shared<RobotCommandPublisher>(num_robots));
  rclcpp::shutdown();
  return 0;
}
