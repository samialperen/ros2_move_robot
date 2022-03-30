#include "ros2_draw_squares/move_robot.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_robot_node");
  int square_number = 1;
  double square_size = 20.0;
  move_robot::MoveRobot MoveRobot(node, square_number, square_size);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}