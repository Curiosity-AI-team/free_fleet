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

#include <iostream>
#include <exception>
#include <thread>

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "free_fleet/ros2/utilities.hpp"
#include "free_fleet/ros2/client_node.hpp"
#include "free_fleet/ros2/client_node_config.hpp"
#include <shared_mutex>
#include <httplib.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


namespace free_fleet
{
namespace ros2
{
ClientNode::ClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("free_fleet_client_ros2", options)
{
    /// Starting the free fleet client
    RCLCPP_INFO(get_logger(), "Greetings from %s", get_name());

    // parameter declarations
    declare_parameter("fleet_name", client_node_config.fleet_name);
    declare_parameter("robot_name", client_node_config.robot_name);
    declare_parameter("robot_model", client_node_config.robot_model);
    declare_parameter("level_name", client_node_config.level_name);
    // defaults declared in header
    declare_parameter("battery_state_topic", client_node_config.battery_state_topic);
    declare_parameter("map_frame", client_node_config.map_frame);
    declare_parameter("robot_frame", client_node_config.robot_frame);
    declare_parameter("nav2_server_name", client_node_config.move_base_server_name);
    declare_parameter("docking_trigger_server_name", client_node_config.docking_trigger_server_name);
    declare_parameter("dds_domain", client_node_config.dds_domain);
    declare_parameter("dds_mode_request_topic", client_node_config.dds_mode_request_topic);
    declare_parameter("dds_path_request_topic", client_node_config.dds_path_request_topic);
    declare_parameter(
        "dds_destination_request_topic",
        client_node_config.dds_destination_request_topic);
    declare_parameter("wait_timeout", client_node_config.wait_timeout);
    declare_parameter("update_frequency", client_node_config.update_frequency);
    declare_parameter("publish_frequency", client_node_config.publish_frequency);
    declare_parameter("max_dist_to_first_waypoint", client_node_config.max_dist_to_first_waypoint);

    // getting new values for parameters or keep defaults
    get_parameter("fleet_name", client_node_config.fleet_name);
    get_parameter("robot_name", client_node_config.robot_name);
    get_parameter("robot_model", client_node_config.robot_model);
    get_parameter("level_name", client_node_config.level_name);
    get_parameter("battery_state_topic", client_node_config.battery_state_topic);
    get_parameter("map_frame", client_node_config.map_frame);
    get_parameter("robot_frame", client_node_config.robot_frame);
    get_parameter("nav2_server_name", client_node_config.move_base_server_name);
    get_parameter("docking_trigger_server_name", client_node_config.docking_trigger_server_name);
    get_parameter("dds_domain", client_node_config.dds_domain);
    get_parameter("dds_mode_request_topic", client_node_config.dds_mode_request_topic);
    get_parameter("dds_path_request_topic", client_node_config.dds_path_request_topic);
    get_parameter("dds_destination_request_topic", client_node_config.dds_destination_request_topic);
    get_parameter("wait_timeout", client_node_config.wait_timeout);
    get_parameter("update_frequency", client_node_config.update_frequency);
    get_parameter("publish_frequency", client_node_config.publish_frequency);
    get_parameter("max_dist_to_first_waypoint", client_node_config.max_dist_to_first_waypoint);
    print_config();

    ClientConfig client_config = client_node_config.get_client_config();
    Client::SharedPtr client = Client::make(client_config);
    if (!client) {
        throw std::runtime_error("Unable to create free_fleet Client from config.");
    }

    /// Setting up the navigation2 action client, wait for server
    RCLCPP_INFO(
        get_logger(), "waiting for connection with navigation action server: %s",
        client_node_config.move_base_server_name.c_str());
    rclcpp_action::Client<NavigateToPose>::SharedPtr move_base_client =
        rclcpp_action::create_client<NavigateToPose>(this, client_node_config.move_base_server_name);
    while (!move_base_client->wait_for_action_server(std::chrono::duration<double>(client_node_config.wait_timeout))) {
        RCLCPP_ERROR(
            get_logger(), "timed out waiting for action server: %s",
            client_node_config.move_base_server_name.c_str());
        if (!rclcpp::ok())
            throw std::runtime_error("exited rclcpp while constructing client_node");
    }
    RCLCPP_INFO(
        get_logger(), "connected with move base action server: %s",
        client_node_config.move_base_server_name.c_str());

    /// Setting up the docking server client, if required, wait for server
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_trigger_client = nullptr;
    if (client_node_config.docking_trigger_server_name != "") {
        docking_trigger_client = create_client<std_srvs::srv::Trigger>(
            client_node_config.docking_trigger_server_name);
        RCLCPP_INFO(
            get_logger(), "waiting for connection with trigger server: %s",
            client_node_config.docking_trigger_server_name.c_str());
        while (!docking_trigger_client->wait_for_service(
                std::chrono::duration<double>(client_node_config.wait_timeout)))
        {
            RCLCPP_ERROR(
                get_logger(), "timed out waiting for docking trigger server: %s",
                client_node_config.docking_trigger_server_name.c_str());
            if (!rclcpp::ok()) {
                throw std::runtime_error("exited rclcpp while constructing client_node");
            }
        }
    }

    tf2_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf2_buffer->setCreateTimerInterface(timer_interface);
    tf2_buffer->setUsingDedicatedThread(true);
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);

    start(
        Fields{
                std::move(client),
                std::move(move_base_client),
                std::move(docking_trigger_client)
            });
    
    // --- HTTP server for GET /mode ---
    // Example: curl http://localhost:8790/mode
    std::thread([this]() {
        httplib::Server srv;

        static const std::map<int,std::string> MODE_NAMES_MAP = {
                {0,          "idle"},
                {1,      "charging"},
                {2,        "moving"},
                {3,        "paused"},
                {4,       "waiting"},
                {5,     "emergency"},
                {6,    "going_home"},
                {7,       "docking"},
                {8, "request_error"}
        };
        srv.Get("/mode", [this](const httplib::Request&, httplib::Response& res) {
            auto mode_msg = this->get_robot_mode();
            int mode_code = mode_msg.mode;
            auto it = MODE_NAMES_MAP.find(mode_code);
            std::string mode_name = (it != MODE_NAMES_MAP.end()) ? it->second : "unknown";
            // 2) build JSON:
            json j;
            j["mode_code"] = mode_code;
            j["mode_name"] = mode_name;
            res.set_content(j.dump(2), "application/json");
        });
        
        RCLCPP_INFO(get_logger(), "Starting HTTP server on :8990");
        srv.listen("0.0.0.0", 8790);
    }).detach();

    docking_cli_    = this->create_client<std_srvs::srv::Trigger>("/recharge");
    undocking_cli_  = this->create_client<std_srvs::srv::Trigger>("/dis_recharge");

    dock_action_client_   = rclcpp_action::create_client<opennav_docking_msgs::action::DockRobot>(this,   "/dock_robot");
    undock_action_client_ = rclcpp_action::create_client<opennav_docking_msgs::action::UndockRobot>(this, "/undock_robot");
// try to connect, but only wait for e.g. 5 seconds
constexpr auto CONNECT_TIMEOUT = std::chrono::seconds(5);

bool dock_ready = dock_action_client_->wait_for_action_server(CONNECT_TIMEOUT);
if (!dock_ready) {
  RCLCPP_WARN(get_logger(),
    "Dock action server not available after %ld seconds; docking will be skipped",
    CONNECT_TIMEOUT.count());
}
bool undock_ready = undock_action_client_->wait_for_action_server(CONNECT_TIMEOUT);
if (!undock_ready) {
  RCLCPP_WARN(get_logger(),
    "Undock action server not available after %ld seconds; undocking will be skipped",
    CONNECT_TIMEOUT.count());
}

// store availability in member flags
dock_server_available_   = dock_ready;
undock_server_available_ = undock_ready;

}

ClientNode::~ClientNode()
{
}

void ClientNode::start(Fields _fields)
{
    fields = std::move(_fields);

    battery_percent_sub = create_subscription<sensor_msgs::msg::BatteryState>(
        client_node_config.battery_state_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ClientNode::battery_state_callback_fn, this, std::placeholders::_1));

    instability_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/instability_diagnostics", rclcpp::QoS(1).best_effort(),
                                                std::bind(&ClientNode::instability_callback, this, std::placeholders::_1));

    request_error = false;
    emergency = false;
    paused = false;

    RCLCPP_INFO(get_logger(), "starting update timer.");
    std::chrono::duration<double> update_period = std::chrono::duration<double>(1.0 / client_node_config.update_frequency);
    update_timer = create_wall_timer(update_period, std::bind(&ClientNode::update_fn, this));

    RCLCPP_INFO(get_logger(), "starting publish timer.");
    std::chrono::duration<double> publish_period = std::chrono::duration<double>(1.0 / client_node_config.publish_frequency);
    publish_timer = create_wall_timer(publish_period, std::bind(&ClientNode::publish_fn, this));
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(client_node_config.battery_state_topic, rclcpp::SystemDefaultsQoS());
}

void ClientNode::print_config()
{
    client_node_config.print_config();
}

void ClientNode::battery_state_callback_fn(const sensor_msgs::msg::BatteryState::SharedPtr _msg)
{
    WriteLock battery_state_lock(battery_state_mutex);
    current_battery_state = *_msg;
    charger_active_ = (_msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

void ClientNode::instability_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr info)
{
    for (const auto& status : info->status)
    {
        RCLCPP_INFO(get_logger(), "=== Diagnostic Status ===");
        RCLCPP_INFO(get_logger(), "Level: %d", status.level);
        RCLCPP_INFO(get_logger(), "Name: %s", status.name.c_str());
        RCLCPP_INFO(get_logger(), "Message: %s", status.message.c_str());
        RCLCPP_INFO(get_logger(), "Hardware ID: %s", status.hardware_id.c_str());

        for (const auto& kv : status.values)
        {
            RCLCPP_INFO(get_logger(), "  - %s: %s", kv.key.c_str(), kv.value.c_str());
        }

        if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK)
        {
            RCLCPP_ERROR(get_logger(), "Reports localization lost! Triggering RMF error mode.");
            request_error = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Localization regained.");
            request_error = false;
        }
    }
}

bool ClientNode::get_robot_pose()
{
    geometry_msgs::msg::PoseStamped tmp_pose_stamped;
    if (nav2_util::getCurrentPose(tmp_pose_stamped, *tf2_buffer,
                                                                client_node_config.map_frame,
                                                                client_node_config.robot_frame,
                                                                client_node_config.wait_timeout)) 
    {
        WriteLock robot_transform_lock(robot_pose_mutex);
        previous_robot_pose = current_robot_pose;
        current_robot_pose = tmp_pose_stamped;
        return true;
    } 
    else {
        RCLCPP_WARN(get_logger(), "Unable to get robot pose.");
        return false;
    }
}

messages::RobotMode ClientNode::get_robot_mode()
{
    /// Checks if robot has just received a request that causes an adapter error
    if (request_error) {
        return messages::RobotMode{messages::RobotMode::MODE_REQUEST_ERROR};
    }

    /// Checks if robot is under emergency
    if (emergency) {
        return messages::RobotMode{messages::RobotMode::MODE_EMERGENCY};
    }

    /// Checks if robot is charging
    {
        ReadLock battery_state_lock(battery_state_mutex);

        if (current_battery_state.power_supply_status == current_battery_state.POWER_SUPPLY_STATUS_CHARGING)
        {
            return messages::RobotMode{messages::RobotMode::MODE_CHARGING};
        }
    }

    /// Checks if robot is moving
    {
        ReadLock robot_transform_lock(robot_pose_mutex);

        if (!is_pose_close(current_robot_pose, previous_robot_pose))
        {
            return messages::RobotMode{messages::RobotMode::MODE_MOVING};
        }
    }

    /// Otherwise, robot is neither charging nor moving,
    /// Checks if the robot is paused
    if (paused) {
        return messages::RobotMode{messages::RobotMode::MODE_PAUSED};
    }

    /// Otherwise, robot has queued tasks, it is paused or waiting,
    /// default to use pausing for now
    return messages::RobotMode{messages::RobotMode::MODE_IDLE};
}

void ClientNode::publish_robot_state()
{
    messages::RobotState new_robot_state;
    new_robot_state.name = client_node_config.robot_name;
    new_robot_state.model = client_node_config.robot_model;

    {
        ReadLock task_id_lock(task_id_mutex);
        new_robot_state.task_id = current_task_id;
    }

    new_robot_state.mode = get_robot_mode();

    {
        ReadLock battery_state_lock(battery_state_mutex);
        /// RMF expects battery to have a percentage in the range for 0-100.
        /// sensor_msgs/BatteryInfo on the other hand returns a value in
        /// the range of 0-1
        new_robot_state.battery_percent = 100 * current_battery_state.percentage;
        // new_robot_state.battery_percent = 20;
    }

    {
        ReadLock robot_transform_lock(robot_pose_mutex);
        new_robot_state.location.sec            = current_robot_pose.header.stamp.sec;
        new_robot_state.location.nanosec        = current_robot_pose.header.stamp.nanosec;
        new_robot_state.location.x              = current_robot_pose.pose.position.x;
        new_robot_state.location.y              = current_robot_pose.pose.position.y;
        if (!goal_path.empty()) {
            // A dirty way to publish the yaw as goal therefore Openrmf to trigger next task.
            RCLCPP_ERROR(get_logger(), "Printing all yaw in goal_path:");
            for (const auto& g : goal_path)
            {
                double yaw = get_yaw_from_pose(g.goal.pose);
                RCLCPP_ERROR(get_logger(), "Yaw in goal_path: %.4f", yaw);
            }
            new_robot_state.location.yaw = get_yaw_from_pose(goal_path.front().goal.pose);
        } else {
            // Fallback: use the current robot pose's yaw
            new_robot_state.location.yaw = get_yaw_from_pose(current_robot_pose);
        }
        // new_robot_state.location.yaw = 0.0;
        new_robot_state.location.level_name = client_node_config.level_name;
    }

    new_robot_state.path.clear();
    {
        ReadLock goal_path_lock(goal_path_mutex);
        for (size_t i = 0; i < goal_path.size(); ++i)
        {
            new_robot_state.path.push_back(
                    messages::Location{
                            (int32_t)goal_path[i].goal.pose.header.stamp.sec,
                            goal_path[i].goal.pose.header.stamp.nanosec,
                            (float)goal_path[i].goal.pose.pose.position.x,
                            (float)goal_path[i].goal.pose.pose.position.y,
                            (float)get_yaw_from_pose(goal_path[i].goal.pose),
                            goal_path[i].level_name
                    });
        }
    }

    if (!fields.client->send_robot_state(new_robot_state)) {
        RCLCPP_WARN(
            get_logger(), "failed to send robot state: msg sec %u",
            new_robot_state.location.sec);
    }
}

bool ClientNode::is_valid_request(const std::string & _request_fleet_name,
                                                                    const std::string & _request_robot_name,
                                                                    const std::string & _request_task_id)
{
    ReadLock task_id_lock(task_id_mutex);
    if (current_task_id == _request_task_id ||
        client_node_config.robot_name != _request_robot_name ||
        client_node_config.fleet_name != _request_fleet_name)
    {
        return false;
    }
    return true;
}

// ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
// ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
void ClientNode::set_goal_tolerances(double xy_tol, double yaw_tol, bool stateful)
{
    using namespace std::chrono_literals;

    // ➊ one hidden node shared by all calls — NOT part of any external executor
    static auto helper_node = std::make_shared<rclcpp::Node>("_nav2_param_helper");

    // ➋ the SyncParametersClient that uses that helper node
    static auto param_client = std::make_shared<rclcpp::SyncParametersClient>(helper_node, "controller_server");

    if (!param_client->wait_for_service(1s))
    {
        RCLCPP_WARN(get_logger(), "controller_server not available – tolerances unchanged");
        return;
    }

    param_client->set_parameters({
        rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",  xy_tol),
        rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", yaw_tol),
        rclcpp::Parameter("general_goal_checker.stateful",           stateful)
    });

    RCLCPP_INFO(get_logger(), "Updated Nav2 tolerances: xy=%.3f  yaw=%.3f", xy_tol, yaw_tol);
}

nav2_msgs::action::NavigateToPose::Goal ClientNode::location_to_nav_goal(const messages::Location& _location) const
{
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = client_node_config.map_frame;
    goal.pose.header.stamp.sec = _location.sec;
    goal.pose.header.stamp.nanosec = _location.nanosec;
    goal.pose.pose.position.x = _location.x;
    goal.pose.pose.position.y = _location.y;
    goal.pose.pose.position.z = 0.0; // TODO: handle Z height with level
    RCLCPP_INFO(get_logger(), "received a _location.yaw. %f", _location.yaw);
    goal.pose.pose.orientation = get_quat_from_yaw(_location.yaw);
    return goal;
}

bool ClientNode::has_elapsed()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - _start;                               
  return elapsed >= std::chrono::seconds(60);
}

bool ClientNode::read_mode_request()
{
    messages::ModeRequest mode_request;
    if (fields.client->read_mode_request(mode_request) && is_valid_request(mode_request.fleet_name, mode_request.robot_name, mode_request.task_id))
    {
        if (mode_request.mode.mode == messages::RobotMode::MODE_PAUSED)
        {
            RCLCPP_INFO(get_logger(), "received a PAUSE command.");

            fields.move_base_client->async_cancel_all_goals();
            WriteLock goal_path_lock(goal_path_mutex);
            if (!goal_path.empty())
                goal_path[0].sent = false;

            paused = true;
            emergency = false;
            request_error = false;
        }
        else if (mode_request.mode.mode == messages::RobotMode::MODE_MOVING)
        {
            RCLCPP_INFO(get_logger(), "received an explicit RESUME command.");
            paused = false;
            emergency = false;
            request_error = false;
        }
        else if (mode_request.mode.mode == messages::RobotMode::MODE_EMERGENCY)
        {
            RCLCPP_INFO(get_logger(), "received an EMERGENCY command.");
            paused = false;
            emergency = true;
            request_error = false;
        }
        else if (mode_request.mode.mode == messages::RobotMode::MODE_DOCKING)
        {
            RCLCPP_INFO(get_logger(), "received a DOCKING command.");
            if (fields.docking_trigger_client && fields.docking_trigger_client->service_is_ready())
            {
                using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
                auto response_received_callback = [&](ServiceResponseFuture future) 
                {
                    auto response = future.get();
                    if (!response->success)
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to trigger docking sequence, message: %s.", response->message.c_str());
                        request_error = true;
                    } else {
                        request_error = false;
                        RCLCPP_ERROR(get_logger(), "Trigger docking sequence, message: %s.", response->message.c_str());
                    }
                };
                auto trigger_srv = std::make_shared<std_srvs::srv::Trigger::Request>();
                // sync call would block indefinelty as we are in a spinning node
                fields.docking_trigger_client->async_send_request(trigger_srv, response_received_callback);
            }
        }
        else {
            RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
                            mode_request.mode.mode);
            request_error = true;
        }

        WriteLock task_id_lock(task_id_mutex);
        current_task_id = mode_request.task_id;

        return true;
    }
    return false;
}

bool ClientNode::read_path_request()
{
    messages::PathRequest path_request;
    if (fields.client->read_path_request(path_request) &&
            is_valid_request(path_request.fleet_name, path_request.robot_name, path_request.task_id)
            )
    {
        RCLCPP_INFO(get_logger(), "received a Path command of size %lu.", path_request.path.size());

        if (path_request.path.size() <= 0)
            return false;

        // Sanity check: the first waypoint of the Path must be within N meters of
        // our current position. Otherwise, ignore the request.
        {
            ReadLock robot_transform_lock(robot_pose_mutex);
            const double dx = path_request.path[0].x - current_robot_pose.pose.position.x;
            const double dy = path_request.path[0].y - current_robot_pose.pose.position.y;
            const double dist_to_first_waypoint = std::hypot(dx,dy);
            RCLCPP_INFO(get_logger(), "distance to first waypoint: %.2f\n", dist_to_first_waypoint);

            if (dist_to_first_waypoint > client_node_config.max_dist_to_first_waypoint)
            {
                RCLCPP_WARN(get_logger(), "distance was over threshold of %.2f ! Rejecting path,"
                        "waiting for next valid request.\n",
                        client_node_config.max_dist_to_first_waypoint);

                fields.move_base_client->async_cancel_all_goals();
                {
                    WriteLock goal_path_lock(goal_path_mutex);
                    goal_path.clear();
                }

                request_error = true;
                emergency = false;
                paused = false;
                return false;
            }
        }

        fields.move_base_client->async_cancel_all_goals();
        // TODO(AA): Use a scoped lock for these mutexes, and rework the time
        // remaining wait logic. We are currently relying on the task to be updated
        // to indicate that the client does not need to wait anymore.
        {
                WriteLock task_id_lock(task_id_mutex);
                current_task_id = path_request.task_id;
        }
        {
                WriteLock goal_path_lock(goal_path_mutex);
                goal_path.clear();
                for (size_t i = 0; i < path_request.path.size(); ++i)
                {
                        // Set sec and nanosec to zero if timing is not important
                        path_request.path[i].sec = 0;
                        path_request.path[i].nanosec = 0;

                        // Calculate yaw if not the last point
                        if (i < path_request.path.size() - 1)
                        {
                            double dx = path_request.path[i + 1].x - path_request.path[i].x;
                            double dy = path_request.path[i + 1].y - path_request.path[i].y;
                            path_request.path[i].yaw = atan2(dy, dx); // Calculate yaw angle
                        }
                        else
                        {
                            // For the last point, use the yaw of the previous point
                            if (path_request.path.size() > 1)
                                    path_request.path[i].yaw = path_request.path[i - 1].yaw;
                            else
                                    path_request.path[i].yaw = 0.0; // Default yaw if only one point exists
                        }

                        // Debug logs
                        RCLCPP_INFO(get_logger(), "size: %lu", path_request.path.size());
                        RCLCPP_INFO(get_logger(), "sec: %u, yaw: %.2f", path_request.path[i].sec, path_request.path[i].yaw);
                        // Add the waypoint to the goal path
                        goal_path.push_back(
                                Goal {
                                        path_request.path[i].level_name,
                                        location_to_nav_goal(path_request.path[i]),
                                        path_request.path[i].approach_speed_limit,
                                        false,
                                        0,
                                        this->now()
                                });
                }
        }

        if (paused)
            paused = false;

        request_error = false;
        return true;
    }
    return false;
}

bool ClientNode::read_destination_request()
{
    messages::DestinationRequest destination_request;
    if (fields.client->read_destination_request(destination_request) &&
            is_valid_request(destination_request.fleet_name, destination_request.robot_name, destination_request.task_id))
    {
        RCLCPP_INFO(get_logger(), "received a Destination command, x: %.2f, y: %.2f, yaw: %.2f",
                destination_request.destination.x, destination_request.destination.y,
                destination_request.destination.yaw);

        fields.move_base_client->async_cancel_all_goals();
        {
            WriteLock goal_path_lock(goal_path_mutex);
            goal_path.clear();
            goal_path.push_back(
                    Goal {
                            destination_request.destination.level_name,
                            location_to_nav_goal(destination_request.destination),
                            0.0,
                            false,
                            0,
                            rclcpp::Time(
                                    destination_request.destination.sec,
                                    destination_request.destination.nanosec,
                                    RCL_ROS_TIME)
                                }
                    ); // messages use RCL_ROS_TIME instead of default RCL_SYSTEM_TIME
        }

        {
            WriteLock task_id_lock(task_id_mutex);
            current_task_id = destination_request.task_id;
        }

        if (paused)
            paused = false;

        request_error = false;
        return true;
    }
    return false;
}

void ClientNode::read_requests()
{
    if (read_mode_request() ||    read_path_request() ||    read_destination_request())
    {
        return;
    }
}

void ClientNode::handle_requests()
{

    if (emergency || request_error || paused)
        return;

    // ooooh we have goals
    ReadLock goal_path_lock(goal_path_mutex);
    if (!goal_path.empty())
    {
        const auto& current_goal = goal_path.front();

        // Calculate the distance to the goal
        double dx = current_goal.goal.pose.pose.position.x - current_robot_pose.pose.position.x;
        double dy = current_goal.goal.pose.pose.position.y - current_robot_pose.pose.position.y;
        double distance_to_goal = std::hypot(dx, dy);

        // Calculate the yaw error
        double yaw_to_goal = get_yaw_from_pose(current_goal.goal.pose);
        double current_yaw = get_yaw_from_pose(current_robot_pose);
        double yaw_error = fabs(yaw_to_goal - current_yaw);

        // Debug information
        // RCLCPP_INFO(get_logger(), "yaw_to_goal: %.5f", yaw_to_goal);
        // RCLCPP_INFO(get_logger(), "current_yaw: %.5f", current_yaw);
        // RCLCPP_INFO(get_logger(), "Distance to goal: %.3f", distance_to_goal);
        // RCLCPP_INFO(get_logger(), "Yaw error: %.5f", yaw_error);
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [&](const GoalHandleNavigateToPose::SharedPtr & goal) {
            if (!goal) {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }
        };
        send_goal_options.feedback_callback = [&](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Distance remaining: %f", feedback->distance_remaining);
        };
        send_goal_options.result_callback = [&](const GoalHandleNavigateToPose::WrappedResult & result) {
            WriteLock goal_path_lock(goal_path_mutex);
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "current goal state: SUCCEEDED.");
                    if (now() >= goal_path.front().goal_end_time)
                    {
                        if (!goal_path.empty())
                            goal_path.pop_front();
                    }
                    else
                    {
                        std::thread{
                            [&]()
                            {
                                std::string goal_task_id;
                                {
                                    ReadLock task_id_lock(task_id_mutex);
                                    goal_task_id = current_task_id;
                                }

                                {
                                    ReadLock goal_path_lock(goal_path_mutex);
                                    while (now() < goal_path.front().goal_end_time)
                                    {
                                        {
                                            ReadLock task_id_lock(task_id_mutex);
                                            if (current_task_id != goal_task_id)
                                            {
                                                RCLCPP_INFO(get_logger(),
                                                    "A new task has come in, moving on to new task.");
                                                return;
                                            }
                                        }

                                        rclcpp::Duration wait_time_remaining = goal_path.front().goal_end_time - now();
                                        RCLCPP_INFO(get_logger(),
                                                "We reached our goal early! Waiting %.2f more seconds",
                                                wait_time_remaining.seconds());
                                        std::this_thread::sleep_for(std::chrono::seconds(2));
                                    }
                                }

                                std::scoped_lock goal_lock{task_id_mutex, goal_path_mutex};
                                if (!goal_path.empty() && current_task_id == goal_task_id)
                                {
                                    goal_path.pop_front();
                                }
                            }
                        }.detach();
                    }
                    return;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    goal_path.front().aborted_count++;

                    if (goal_path.front().aborted_count < 5)
                    {
                        RCLCPP_INFO(get_logger(), "Retrying goal...");
                        goal_path.front().sent = false;
                        return;
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Aborted too many times. Clearing path.");
                        goal_path.clear();
                        return;
                    }
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    goal_path.front().sent = false;
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", (int)result.code);
                    goal_path.clear();
                    return;
            }
        };

        // Goals must have been updated since last handling, execute them now
        if (!goal_path.front().sent)
        {

            // ────────────────────────────────────────────────────────────────
            // If that waypoint carried a non‑zero approach_speed_limit,
            // tighten (or loosen) Nav2’s tolerances before we dispatch it.
            // Feel free to map speed→tolerance any way you like.
            if (goal_path.front().approach_speed_limit != 0.0) {
                set_goal_tolerances(2.0, 2.05, false);   // 5 cm / 0.05 rad
            } else {
                set_goal_tolerances(0.5, 0.5, true);   // default 5 cm / 0.5 rad
            }
            // ────────────────────────────────────────────────────────────────
            RCLCPP_INFO(get_logger(), "Sending next goal.");
            fields.move_base_client->async_send_goal(goal_path.front().goal, send_goal_options);
            goal_path.front().sent = true;
            return;
        }
    }
}

void ClientNode::check_battery_and_handle_charging()
{
    if (!dock_server_available_) {
    // RCLCPP_ERROR(get_logger(), "Skipping auto-dock: no action server");
    return;
    }
    double soc;
    soc = current_battery_state.percentage;  // 0.0–1.0

    // 1) dispatch a 1‑round “station→charger” patrol when SOC ≤ threshold
    if (!going_to_charge_ && !charger_active_ && soc <= low_bat_threshold_ && is_robot_near_dock())
    {
        RCLCPP_WARN(get_logger(), "[%s] SOC=%.0f%% low → sending auto‑charge request", client_node_config.robot_name.c_str(), soc*100.0);
        
        // action go to dock
        opennav_docking_msgs::action::DockRobot::Goal goal;
        goal.use_dock_id = true;
        goal.dock_id     = "home_dock";

        auto opts = rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SendGoalOptions();
        opts.goal_response_callback = [this](auto future){
            if (future.get()) {
                RCLCPP_INFO(get_logger(), "Auto-dock goal accepted");
                going_to_charge_ = true;
            } else {
                RCLCPP_ERROR(get_logger(), "Auto-dock goal rejected");
            }
        };
        opts.result_callback = [this](const auto & result){
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(get_logger(),
                "Auto-dock failed (%d) → will retry on next cycle",
                (int)result.code);
                going_to_charge_ = false;
            }
            else charger_active_ = true;
        };
        dock_action_client_->async_send_goal(goal, opts);

    }

    // 3) when SOC climbs back above recharge_soc_, call /dis_recharge once
    if (charger_active_ && soc >= recharge_soc_)
    {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        // docking_cli_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "DISCHARGE MODE");

        opennav_docking_msgs::action::UndockRobot::Goal goal;
        auto opts = rclcpp_action::Client<opennav_docking_msgs::action::UndockRobot>::SendGoalOptions();
        opts.goal_response_callback = [this](auto future){
        if (future.get()) {
            RCLCPP_INFO(get_logger(), "Auto-undock goal accepted");
            charger_active_  = false;
        } else {
            RCLCPP_ERROR(get_logger(), "Auto-undock goal rejected");
        }
        };
        opts.result_callback = [this](const auto & result){
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(),
            "Auto-undock failed (%d) → will retry on next cycle",
            (int)result.code);
            going_to_charge_ = true;
        } else {
            going_to_charge_ = false;
            charger_active_  = false;
        }
        };
        undock_action_client_->async_send_goal(goal, opts);

    }
}


bool ClientNode::is_robot_near_dock()
{
    ReadLock lock(robot_pose_mutex);
    const double dx = current_robot_pose.pose.position.x - dock_x_;
    const double dy = current_robot_pose.pose.position.y - dock_y_;
    return std::hypot(dx, dy) <= dock_radius_;
}

void ClientNode::battery_sim_tick()
{
    if (charger_active_)
        sim_battery_percentage_ = std::min(1.0, sim_battery_percentage_ + battery_charge_rate_);
    else
        sim_battery_percentage_ = std::max(0.0, sim_battery_percentage_ - battery_discharge_rate_);

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.percentage   = sim_battery_percentage_;
    msg.power_supply_status =
        charger_active_
        ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
        : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

    battery_pub_->publish(msg);
}

void ClientNode::update_fn()
{
    get_robot_pose();
    read_requests();
    handle_requests();
    check_battery_and_handle_charging();
    battery_sim_tick();
}

void ClientNode::publish_fn()
{
    publish_robot_state();
}

} // namespace ros2
} // namespace free_fleet
