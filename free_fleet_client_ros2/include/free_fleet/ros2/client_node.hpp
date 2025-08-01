/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FREE_FLEET__ROS2__CLIENTNODE_HPP
#define FREE_FLEET__ROS2__CLIENTNODE_HPP

#include <deque>
#include <shared_mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/impl/utils.h>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav2_util/robot_utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <free_fleet/Client.hpp>
#include <free_fleet/ClientConfig.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

#include <free_fleet/Client.hpp>
#include <free_fleet/messages/Location.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "free_fleet/ros2/client_node_config.hpp"
#include <opennav_docking_msgs/action/dock_robot.hpp>
#include <opennav_docking_msgs/action/undock_robot.hpp>

namespace free_fleet
{
namespace ros2
{

class ClientNode : public rclcpp::Node
{
public:
    using SharedPtr = std::shared_ptr<ClientNode>;
    using Mutex = std::shared_mutex;
    using ReadLock = std::shared_lock<Mutex>;
    using WriteLock = std::unique_lock<Mutex>;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ClientNode() override;

    struct Fields
    {
        /// Free fleet client
        Client::SharedPtr client;

        // navigation2 action client
        rclcpp_action::Client<NavigateToPose>::SharedPtr move_base_client;

        // Docker server client
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_trigger_client;
    };

    struct Goal
    {
        std::string level_name;
        NavigateToPose::Goal goal;
        double approach_speed_limit;
        bool sent = false;
        uint32_t aborted_count = 0;
        rclcpp::Time goal_end_time;
    };

    void print_config();
    std::chrono::steady_clock::time_point _start = std::chrono::steady_clock::now();

private:

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
    std::shared_ptr<rclcpp::TimerBase> update_timer;
    std::shared_ptr<rclcpp::TimerBase> publish_timer;
    sensor_msgs::msg::BatteryState current_battery_state;
    geometry_msgs::msg::PoseStamped current_robot_pose;
    geometry_msgs::msg::PoseStamped previous_robot_pose;

    std::atomic<bool> request_error;
    std::atomic<bool> emergency;
    std::atomic<bool> paused;
    
    std::string current_task_id;
    std::string next_task_id_for_idle_;

    
    ClientNodeConfig client_node_config;
    Fields fields;
    bool   idle_requested_    = false;
    double sim_battery_percentage_ = 1;
    double low_bat_threshold_ = 0.20;
    double recharge_soc_      = 1.00;
    bool   going_to_charge_   = false;
    bool   charger_active_    = false;
    double battery_discharge_rate_{0.0001};
    double battery_charge_rate_{0.01};
    double dock_x_{5}, dock_y_{-5}, dock_radius_{2};
    bool dock_server_available_{false}, undock_server_available_{false};
    messages::RobotMode get_robot_mode();
    bool read_mode_request();
    bool get_robot_pose();
    bool read_path_request();
    bool read_destination_request();
    bool read_task_request();
    bool is_valid_request(const std::string& request_fleet_name, const std::string& request_robot_name, const std::string& request_task_id);
    bool has_elapsed();
    bool is_robot_near_dock();
    
    
    std::mutex     mode_request_mutex;
    Mutex goal_path_mutex;
    std::deque<Goal> goal_path;
    Mutex battery_state_mutex;
    Mutex robot_pose_mutex;
    Mutex task_id_mutex;
    
    
    void battery_sim_tick();
    void read_requests();
    void handle_requests();
    void publish_robot_state();
    void update_fn();
    void publish_fn();
    void start(Fields fields);
    void set_goal_tolerances(double xy_tol, double yaw_tol, bool statful);
    void battery_state_callback_fn(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void check_battery_and_handle_charging();
    void instability_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr info);
    
    NavigateToPose::Goal location_to_nav_goal(const messages::Location& _location) const;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr             battery_percent_sub;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr      instability_sub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr                battery_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                           docking_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                           undocking_cli_;

  rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SharedPtr   dock_action_client_;
  rclcpp_action::Client<opennav_docking_msgs::action::UndockRobot>::SharedPtr undock_action_client_;

};

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET__ROS2__CLIENTNODE_HPP