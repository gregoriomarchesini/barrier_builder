#include <cinttypes>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "barrier_msg/srv/b_comp_srv.hpp"
#include "barrier_msg/msg/t_msg.hpp"
#include "barrier_msg/msg/b_msg.hpp"

#include "barrier_msg/msg/config.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "task.h"
#include "fusion.h"
#include "hypercube.h"
#include "utils.h"
#include "task_scheduler.h"
#include "bMsg.h"

using BCompSrv = barrier_msg::srv::BCompSrv;
using TMsg = barrier_msg::msg::TMsg;
using BMsg = barrier_msg::msg::BMsg;
using Config = barrier_msg::msg::Config;
using Odometry = nav_msgs::msg::Odometry;

rclcpp::Node::SharedPtr g_node = nullptr;

class Barrier_server : public rclcpp::Node
{
public:
    Barrier_server() : Node("barrier_service")
    {
        server = this->create_service<BCompSrv>(
            "/compute_barriers",
            std::bind(&Barrier_server::handle_service,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));
        
        std::vector<std::string> drones;

        for (int i = 1; i <= num_agents; ++i) {
            drones.push_back("/crazyflie" + std::to_string(i));
        }

        odom_subs_.reserve(drones.size());
        positions_.resize(drones.size()*dim);
        
        for (int i = 0; i < drones.size(); ++i)
        {
            auto topic = drones[i] + "/odom";
            // Capture i by value so each lambda gets its own copy
            auto sub = this->create_subscription<Odometry>(
                topic, 10,
                [this, i](const Odometry::SharedPtr msg) {
                    this->set_pos_callback(msg, i);
                }
            );
            odom_subs_.push_back(sub);
        }
    }

    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<BCompSrv::Request> request,
        const std::shared_ptr<BCompSrv::Response> response)
    {
        (void)request_header;

        size_t num_messages = request->messages.size();
        response->messages.resize(num_messages);

        std::vector<Task<dim>> tasks;
        std::vector<TMsg> task_messages = request->messages;

        for (TMsg task_msg : task_messages)
        {
            Task<dim> task = Task<dim>(
                side_length,
                task_msg.center,
                task_msg.start,
                task_msg.end,
                TaskType::EVENTUALLY,
                task_msg.edge_i,
                task_msg.edge_j,
                num_agents);

            tasks.push_back(task);
        }

        TaskScheduler<dim> scheduler(tasks, num_agents, arena_size, max_input, positions_);
        std::vector<bMsg> messages = scheduler.get_result_as_message();

        for (int i = 0; i < num_messages; i++)
        {
            BMsg msg;
            msg.slopes = messages[i].slopes;
            msg.gamma0 = messages[i].gamma0;
            msg.r = messages[i].r;
            msg.slack = messages[i].slack;
            msg.b_vector = messages[i].b_vector;
            msg.time_grid = messages[i].time_grid;
            msg.task_id = messages[i].task_id;
            msg.edge_i = messages[i].edge_i;
            msg.edge_j = messages[i].edge_j;

            response->messages[i] = msg;
        }
    }

    void set_pos_callback(const Odometry::SharedPtr msg, size_t idx)
    {
        positions_[idx*dim] = msg->pose.pose.position.x;
        positions_[idx*dim+1] = msg->pose.pose.position.y;
        if (dim > 2) 
            positions_[idx*dim+2] = msg->pose.pose.position.z;
    }


private:
    static constexpr int dim = Config::DIM;
    static constexpr int num_agents = Config::NUM_AGENTS;
    double side_length = Config::SIDE_LENGTH;
    double arena_size = Config::ARENA_SIZE;
    double max_input = Config::MAX_INPUT;

    std::vector<double> positions_;
    rclcpp::Service<BCompSrv>::SharedPtr server;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<Barrier_server>();
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}