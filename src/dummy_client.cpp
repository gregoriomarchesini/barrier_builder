#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "barrier_msg/srv/b_comp_srv.hpp"
#include "barrier_msg/msg/t_msg.hpp"
#include "barrier_msg/msg/b_msg.hpp"

using namespace std::chrono_literals;
using BCompSrv = barrier_msg::srv::BCompSrv;
using TMsg     = barrier_msg::msg::TMsg;
using BMsg     = barrier_msg::msg::BMsg;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("compute_barriers_client");

  auto client = node->create_client<BCompSrv>("compute_barriers");

  // Wait for the service to be available
  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  // Create a dummy request
  auto request = std::make_shared<BCompSrv::Request>();

  // Fill it with a few dummy TMsg tasks
  TMsg task;
  task.center = {1.0, 2.0, 3.0};
  task.size = 0.5;
  task.start = 0.0;
  task.type = "eventually";  // or "always"
  task.end = 10.0;
  task.edge_i = 0;
  task.edge_j = 1;

  request->messages.push_back(task);
  request->messages.push_back(task);  // add a second identical task

  // Call the service
  auto future = client->async_send_request(request);

  // Wait for the response
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Received %zu barrier messages", response->messages.size());

    for (size_t i = 0; i < response->messages.size(); ++i) {
      const BMsg &bmsg = response->messages[i];
      RCLCPP_INFO(node->get_logger(),
                  "Barrier %zu: r=%.2f, slack=%.2f, task_id=%ld, edge_i=%ld, edge_j=%ld",
                  i, bmsg.r, bmsg.slack, bmsg.task_id, bmsg.edge_i, bmsg.edge_j);
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service compute_barriers");
  }

  rclcpp::shutdown();
  return 0;
}
