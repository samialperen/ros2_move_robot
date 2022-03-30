#include "ros2_draw_squares/move_robot.hpp"

using namespace std::chrono_literals;

namespace move_robot {
MoveRobot::MoveRobot(std::shared_ptr<rclcpp::Node> node, double square_size,
                     int square_number)
    : square_number_(square_number), square_size_(square_size), pub_rate_(20),
      completed_(false), node_(node) {
  publisher_ =
      this->node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void MoveRobot::draw_square(int &loop_counter) {
  // int &loop_counter --> make the change in loop counter available to the
  // function calling draw_square

  while (loop_counter < square_number_) {
    MoveRobot::go_forward(square_size_); // 1st edge
    MoveRobot::stop();
    MoveRobot::turn_right();
    MoveRobot::go_forward(square_size_); // 2nd edge
    MoveRobot::stop();
    MoveRobot::turn_right();
    MoveRobot::go_forward(square_size_); // 3rd edge
    MoveRobot::stop();
    MoveRobot::turn_right();
    MoveRobot::go_forward(square_size_); // 4th edge
    MoveRobot::stop();
    MoveRobot::turn_right(); // Come to initial pose
    MoveRobot::stop();
    loop_counter++;
  }

  completed_ = true;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Robot drew "
                                              << loop_counter
                                              << " squares. Stopping now!");
}

void MoveRobot::stop(double duration) {
  auto init_time = steady_clock_.now();

  while ((steady_clock_.now() - init_time).seconds() < duration &&
         rclcpp::ok()) {
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    publisher_->publish(twist_);
    pub_rate_.sleep();
  }
}

void MoveRobot::go_forward(double duration) {
  auto init_time = steady_clock_.now();

  while ((steady_clock_.now() - init_time).seconds() < duration &&
         rclcpp::ok()) {
    twist_.linear.x = 0.6;
    twist_.angular.z = 0.0;
    publisher_->publish(twist_);
    pub_rate_.sleep();
  }
}

void MoveRobot::turn_right(double duration) {
  auto init_time = steady_clock_.now();

  while ((steady_clock_.now() - init_time).seconds() < duration &&
         rclcpp::ok()) {
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.6;
    publisher_->publish(twist_);
    pub_rate_.sleep();
  }
}

} // namespace move_robot