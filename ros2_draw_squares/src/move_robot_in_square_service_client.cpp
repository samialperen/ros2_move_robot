#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_draw_squares_custom_msgs/srv/square_service_message.hpp"

using SquareServiceMessage =
    ros2_draw_squares_custom_msgs::srv::SquareServiceMessage;

SquareServiceMessage::Response::SharedPtr
send_request(rclcpp::Node::SharedPtr node,
             rclcpp::Client<SquareServiceMessage>::SharedPtr client,
             SquareServiceMessage::Request::SharedPtr request) {

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Client Request: square_size: "
                           << request->square_size
                           << " square_number: " << request->square_number);
    return result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return NULL;
  }
}

int main(int argc, char **argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("move_robot_in_square_client");
  auto topic = std::string("/move_robot_in_square");
  auto client = node->create_client<SquareServiceMessage>(topic);
  auto request = std::make_shared<SquareServiceMessage::Request>();

  //  Fill In The variables of the Custom Service Message
  request->square_size = 5.0;
  request->square_number = 1;

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = send_request(node, client, request);
  if (result) {

    auto result_str = result->success ? "True" : "False";

    RCLCPP_INFO(node->get_logger(), "Result-Success : %s", result_str);
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}