#ifndef MOVE_ROBOT_HPP_
#define MOVE_ROBOT_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace move_robot {

class MoveRobot {
public:
  MoveRobot(std::shared_ptr<rclcpp::Node> node, double square_size,
            int square_number);

  int square_number_;  // The number of times robot executes the square movement
  double square_size_; // Determines how big the square will be
  rclcpp::WallRate pub_rate_;

  void draw_square(int &loop_counter);
  bool completed_;

private:
  std::shared_ptr<rclcpp::Node> node_;
  geometry_msgs::msg::Twist twist_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void stop(double duration = 0.1);       // seconds
  void turn_right(double duration = 2.7); // seconds
  void go_forward(double duration);
};

} // namespace move_robot

#endif // MOVE_ROBOT_HPP_