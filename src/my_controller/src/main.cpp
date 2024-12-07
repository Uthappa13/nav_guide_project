/**
 * @file main.cpp
 * @author Rishie
 * @brief ROS 2 node for collision avoidance in Gazebo using TurtleBot 3 Waffle.
 * 
 * This program creates a simulation environment for a swarm of robots, each 
 * assigned to navigate to a goal position while avoiding collisions. It utilizes
 * ROS 2 publishers and subscribers to communicate with the robots and execute 
 * the navigation logic in a simulation environment.
 * 
 * @version 0.1
 * @date 2024-12-02
 * 
 * @copyright Copyright (c) 2024
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

/**
 * @brief RobotCommandPublisher class for managing robot navigation and control.
 */
class RobotCommandPublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructs the RobotCommandPublisher node.
     * 
     * This initializes the simulation environment, publishers for velocity 
     * commands, subscribers for odometry data, and a timer for periodic updates.
     */
    RobotCommandPublisher()
        : Node("robot_command_publisher") {
        // Initialize publishers and subscribers
        robot1_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        robot2_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/robot2/cmd_vel", 10);

        robot1_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", 10,
            std::bind(&RobotCommandPublisher::robot1_odom_callback, this, std::placeholders::_1));

        robot2_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/robot2/odom", 10,
            std::bind(&RobotCommandPublisher::robot2_odom_callback, this, std::placeholders::_1));

        // Timer for periodic velocity updates
        timer_ = create_wall_timer(100ms, std::bind(&RobotCommandPublisher::timer_callback, this));

        // Initialize goals
        robot1_goal_ = {2.0, 2.0};
        robot2_goal_ = {-2.0, -2.0};

        RCLCPP_INFO(this->get_logger(), "RobotCommandPublisher node has been initialized.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot1_publisher_; //!< Publisher for Robot 1 velocity commands.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot2_publisher_; //!< Publisher for Robot 2 velocity commands.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot1_subscription_; //!< Subscriber for Robot 1 odometry.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot2_subscription_; //!< Subscriber for Robot 2 odometry.
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer for periodic updates.

    std::vector<double> robot1_pose_; //!< Current pose of Robot 1 {x, y, yaw}.
    std::vector<double> robot2_pose_; //!< Current pose of Robot 2 {x, y, yaw}.
    std::vector<double> robot1_goal_; //!< Goal position for Robot 1 {x, y}.
    std::vector<double> robot2_goal_; //!< Goal position for Robot 2 {x, y}.

    /**
     * @brief Callback function for Robot 1's odometry subscriber.
     * 
     * Updates the robot's current pose based on incoming odometry messages.
     * 
     * @param msg Shared pointer to the odometry message.
     */
    void robot1_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot1_pose_ = extract_pose(msg);
    }

    /**
     * @brief Callback function for Robot 2's odometry subscriber.
     * 
     * Updates the robot's current pose based on incoming odometry messages.
     * 
     * @param msg Shared pointer to the odometry message.
     */
    void robot2_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot2_pose_ = extract_pose(msg);
    }

    /**
     * @brief Timer callback for sending velocity commands to the robots.
     * 
     * Executes the navigation logic, which includes heading adjustment 
     * and movement towards the goal position.
     */
    void timer_callback() {
        auto robot1_twist = compute_velocity(robot1_pose_, robot1_goal_);
        auto robot2_twist = compute_velocity(robot2_pose_, robot2_goal_);

        robot1_publisher_->publish(robot1_twist);
        robot2_publisher_->publish(robot2_twist);
    }

    /**
     * @brief Extracts the robot's pose from an odometry message.
     * 
     * @param msg Shared pointer to the odometry message.
     * @return std::vector<double> The robot's pose as {x, y, yaw}.
     */
    std::vector<double> extract_pose(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return {x, y, yaw};
    }

    /**
     * @brief Computes the velocity command to navigate the robot towards its goal.
     * 
     * @param pose Current pose of the robot {x, y, yaw}.
     * @param goal Goal position for the robot {x, y}.
     * @return geometry_msgs::msg::Twist Velocity command for the robot.
     */
    geometry_msgs::msg::Twist compute_velocity(const std::vector<double> &pose, const std::vector<double> &goal) {
        geometry_msgs::msg::Twist twist;

        if (pose.empty()) {
            return twist;
        }

        double x_diff = goal[0] - pose[0];
        double y_diff = goal[1] - pose[1];

        double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
        double angle_to_goal = atan2(y_diff, x_diff);

        double heading_error = angle_to_goal - pose[2];

        if (distance > 0.1) {
            twist.linear.x = 0.2 * distance;
            twist.angular.z = 0.5 * heading_error;
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }

        return twist;
    }
};

/**
 * @brief Main entry point for the RobotCommandPublisher node.
 * 
 * Initializes and spins the node.
 * 
 * @return int Program exit status.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
