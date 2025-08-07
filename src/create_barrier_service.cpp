#include <cinttypes>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "barrier_msg/srv/b_comp_srv.hpp"
#include "barrier_msg/msg/t_msg.hpp"
#include "barrier_msg/msg/b_msg.hpp"

using BCompSrv = barrier_msg::srv::BCompSrv;
using TMsg     = barrier_msg::msg::TMsg;
using BMsg     = barrier_msg::msg::BMsg;

rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<BCompSrv::Request> request,
  const std::shared_ptr<BCompSrv::Response> response)
{
  (void)request_header;
    size_t num_messages = request->messages.size();
    response->messages.resize(num_messages);

    for (size_t i = 0; i < num_messages; ++i) {
        BMsg msg;
        msg.slopes    = std::vector<double>{};
        msg.gamma0    = std::vector<double>{};
        msg.r         = 0.0;
        msg.slack     = 0.0;
        msg.b_vector  = std::vector<double>{};
        msg.time_grid = std::vector<double>{};
        msg.task_id   = 0;
        msg.edge_i    = 0;
        msg.edge_j    = 0;

        response->messages[i] = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("compute_barriers_server");
  auto server = g_node->create_service<BCompSrv>("compute_barriers", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}