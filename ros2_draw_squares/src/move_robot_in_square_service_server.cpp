#include "rclcpp/rclcpp.hpp"
#include "ros2_draw_squares/move_robot.hpp"
#include "ros2_draw_squares_custom_msgs/srv/square_service_message.hpp"

#include <inttypes.h>
#include <memory>

using SquareServiceMessage =
    ros2_draw_squares_custom_msgs::srv::SquareServiceMessage;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SquareServiceMessage::Request> request,
    const std::shared_ptr<SquareServiceMessage::Response> response) {
  (void)request_header;
  move_robot::MoveRobot MoveRobot(g_node, request->square_size,
                                  request->square_number);

  RCLCPP_INFO_STREAM(g_node->get_logger(),
                     "Request: square_size: " << request->square_size
                                              << " square_number: "
                                              << request->square_number);
  int loop_counter = 0; // Keep track of how many squares robot drew
  MoveRobot.completed_ = false;
  while (rclcpp::ok() && !MoveRobot.completed_) {
    MoveRobot.draw_square(loop_counter);
    // MoveRobot.pub_rate_.sleep();
  }

  response->success = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("move_robot_service_server");
  auto server = g_node->create_service<SquareServiceMessage>(
      "/move_robot_in_square", handle_service);
  RCLCPP_INFO(g_node->get_logger(), "move_robot_service_server is ready.");
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}