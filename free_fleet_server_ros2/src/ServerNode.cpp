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

#include <chrono>

#include <Eigen/Geometry>

#include <free_fleet/Server.hpp>
#include <free_fleet/ServerConfig.hpp>

#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

#include "utilities.hpp"
#include "ServerNode.hpp"
#include "nlohmann/json.hpp"

namespace free_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
    // Starting the free fleet server node
    SharedPtr server_node(new ServerNode(_config, _node_options));

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() < 10)
    {
        rclcpp::spin_some(server_node);

        server_node->setup_config();
        if (server_node->is_ready())
            break;
        RCLCPP_INFO(server_node->get_logger(), "waiting for configuration parameters.");

        end_time = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if (!server_node->is_ready())
    {
        RCLCPP_ERROR(
            server_node->get_logger(), "unable to initialize parameters.");
        return nullptr;
    }
    server_node->print_config();

    // Starting the free fleet server
    ServerConfig server_config =
        server_node->server_node_config.get_server_config();
    Server::SharedPtr server = Server::make(server_config);
    if (!server)
        return nullptr;

    server_node->start(Fields{
        std::move(server)
    });

    return server_node;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(const ServerNodeConfig&  _config, const rclcpp::NodeOptions& _node_options):
    Node(_config.fleet_name + "_node", _node_options),
    server_node_config(_config)
{
    task_dispatcher_client_ = create_client<rmf_task_msgs::srv::SubmitTask>("/hamal_task_dispatcher/task_info");
}

void ServerNode::print_config()
{
    server_node_config.print_config();
}

void ServerNode::setup_config()
{
    get_parameter("fleet_name", server_node_config.fleet_name);
    get_parameter("fleet_state_topic", server_node_config.fleet_state_topic);
    get_parameter("mode_request_topic", server_node_config.mode_request_topic);
    get_parameter("path_request_topic", server_node_config.path_request_topic);
    get_parameter("destination_request_topic", server_node_config.destination_request_topic);
    get_parameter("dds_domain", server_node_config.dds_domain);
    get_parameter("dds_robot_state_topic", server_node_config.dds_robot_state_topic);
    get_parameter("dds_mode_request_topic", server_node_config.dds_mode_request_topic);
    get_parameter("dds_path_request_topic", server_node_config.dds_path_request_topic);
    get_parameter("dds_destination_request_topic", server_node_config.dds_destination_request_topic);
    get_parameter("update_state_frequency", server_node_config.update_state_frequency);
    get_parameter("publish_state_frequency", server_node_config.publish_state_frequency);

    get_parameter("translation_x", server_node_config.translation_x);
    get_parameter("translation_y", server_node_config.translation_y);
    get_parameter("rotation", server_node_config.rotation);
    get_parameter("scale", server_node_config.scale);


    // fetch a parameter "docks_json" which should be a JSON array like:
    //   [
    //     {"name":"station","x":1.0,"y":2.0,"z":0.0},
    //     {"name":"dock2","x":4.0,"y":1.5,"z":0.0}
    //   ]
    std::string docks_json;
    get_parameter("docks_json", docks_json);
    docks_json = R"JSON(
                        [{"name":"station","x":5.0,"y":-5.0,"z":0.0},
                        {"name":"dock2","x":4.0,"y":1.5,"z":0.0}]
                        )JSON";

    try
    {
        auto j = nlohmann::json::parse(docks_json);
        for (auto& d : j)
        {
            docks_.push_back({
            d.at("name").get<std::string>(),
            d.at("x").get<double>(),
            d.at("y").get<double>(),
            d.value("z", 0.0)  // default z = 0.0 if omitted
            });
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu docks", docks_.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(),
            "Failed to parse docks_json parameter: %s", e.what());
    }

}


// Euclidean in 3D
double ServerNode::compute_distance(double rx, double ry, double rz, Dock& dock)
{
    const double dx = rx - dock.x;
    const double dy = ry - dock.y;
    const double dz = rz - dock.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void ServerNode::check_and_dispatch(const std::string& robot_name,
                        double rx, double ry, double rz)
{
    // find the "station" dock
    auto it = std::find_if(docks_.begin(), docks_.end(),
        [](auto& d){ return d.name == "station"; });
    if (it == docks_.end())
        return;  // no station defined

    const double dist = compute_distance(rx, ry, rz, *it);
    RCLCPP_WARN(get_logger(),
    "Robot '%s' is %.2fm from dock '%s' → dispatching go_to_place",
    robot_name.c_str(), dist, it->name.c_str());
    if (dist > 5.0)
    {

        // fire off your CLI task
        // NOTE: this will block the thread briefly; for production
        // you may want to spin off a std::async or better integrate
        std::string cmd = 
        "ros2 run rmf_demos_tasks dispatch_go_to_place -p \"" +
        it->name + "\"";
        // std::system(cmd.c_str());
    }
}

bool ServerNode::is_ready()
{
    if (server_node_config.fleet_name == "fleet_name")
        return false;
    return true;
}

void ServerNode::start(Fields _fields)
{
    fields = std::move(_fields);

    {
        WriteLock robot_states_lock(robot_states_mutex);
        robot_states.clear();
    }

    using namespace std::chrono_literals;

    // --------------------------------------------------------------------------
    // First callback group that handles getting updates from all the clients
    // available

    update_state_callback_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    update_state_timer = create_wall_timer(
        100ms, std::bind(&ServerNode::update_state_callback, this),
        update_state_callback_group);

    // --------------------------------------------------------------------------
    // Second callback group that handles publishing fleet states to RMF, and
    // handling requests from RMF to be sent down to the clients

    fleet_state_pub_callback_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    fleet_state_pub =
        create_publisher<rmf_fleet_msgs::msg::FleetState>(
            server_node_config.fleet_state_topic, 10);
    
    robot_state_pub = this->create_publisher<rmf_fleet_msgs::msg::RobotState>("/robot_state", 10);
    
    fleet_state_pub_timer = create_wall_timer(
        std::chrono::seconds(1) / server_node_config.publish_state_frequency,
        std::bind(&ServerNode::publish_fleet_state, this),
        fleet_state_pub_callback_group);

    // --------------------------------------------------------------------------
    // Mode request handling

    auto mode_request_sub_opt = rclcpp::SubscriptionOptions();

    mode_request_sub_opt.callback_group = fleet_state_pub_callback_group;

    mode_request_sub = create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
        server_node_config.mode_request_topic, rclcpp::QoS(10),
        [&](rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
        {
            handle_mode_request(std::move(msg));
        },
        mode_request_sub_opt);

    // --------------------------------------------------------------------------
    // Path reqeust handling

    auto path_request_sub_opt = rclcpp::SubscriptionOptions();

    path_request_sub_opt.callback_group = fleet_state_pub_callback_group;

    path_request_sub = create_subscription<rmf_fleet_msgs::msg::PathRequest>(
        server_node_config.path_request_topic, rclcpp::QoS(10),
        [&](rmf_fleet_msgs::msg::PathRequest::UniquePtr msg)
        {
            handle_path_request(std::move(msg));
        },
        path_request_sub_opt);

    // --------------------------------------------------------------------------
    // Destination reqeust handling

    auto destination_request_sub_opt = rclcpp::SubscriptionOptions();

    destination_request_sub_opt.callback_group = fleet_state_pub_callback_group;

    destination_request_sub =
        create_subscription<rmf_fleet_msgs::msg::DestinationRequest>(
            server_node_config.destination_request_topic, rclcpp::QoS(10),
            [&](rmf_fleet_msgs::msg::DestinationRequest::UniquePtr msg)
            {
                handle_destination_request(std::move(msg));
            },
            destination_request_sub_opt);

    // --------------------------------------------------------------------------
    // Dispenser reqeust handling

    auto dispenser_request_sub_opt = rclcpp::SubscriptionOptions();

    dispenser_request_sub_opt.callback_group = fleet_state_pub_callback_group;

    dispenser_request_sub = create_subscription<rmf_dispenser_msgs::msg::DispenserRequest>(
        "/dispenser_requests", rclcpp::QoS(10),
        [&](rmf_dispenser_msgs::msg::DispenserRequest::UniquePtr msg)
        {
            handle_dispenser_request(std::move(msg));
        },
        dispenser_request_sub_opt);

    // --------------------------------------------------------------------------
    // Ingestor reqeust handling

    auto ingestor_request_sub_opt = rclcpp::SubscriptionOptions();

    ingestor_request_sub_opt.callback_group = fleet_state_pub_callback_group;

    ingestor_request_sub = create_subscription<rmf_ingestor_msgs::msg::IngestorRequest>(
            "/ingestor_requests", rclcpp::QoS(10),
            [&](rmf_ingestor_msgs::msg::IngestorRequest::UniquePtr msg)
            {
                handle_ingestor_request(std::move(msg));
            },
            ingestor_request_sub_opt);

    dispenser_result_pub = create_publisher<rmf_dispenser_msgs::msg::DispenserResult>(
        "/dispenser_results", 10);

    ingestor_result_pub = create_publisher<rmf_ingestor_msgs::msg::IngestorResult>(
        "/ingestor_results", 10);

    api_pub_ = create_publisher<rmf_task_msgs::msg::ApiRequest>("/task_api_requests", rclcpp::QoS(10).transient_local());
}

void ServerNode::dispatch_charge_task(const std::string& robot_name)
{
    rmf_task_msgs::msg::ApiRequest req;
    req.request_id = "auto_charge_" + robot_name + "_" + std::to_string(charge_request_counter_++);

    nlohmann::json j =
    {
        {"type", "dispatch_task_request"},
        {"request",
        {
            {"unix_millis_earliest_start_time", 0},
            {"priority", {{"type", "binary"}, {"value", 0}}},
            {"labels", {"auto_charge"}},
            {"category", "patrol"},
            {"description",
            {
                {"places", {"station", "charger"}},
                {"rounds", 1}
            }},
            {"dispatch_task", {{"robot_name", robot_name}}}
        }}
    };

    req.json_msg = j.dump();
    api_pub_->publish(req);

    RCLCPP_INFO(get_logger(), "[battery‑watch] Sent auto‑charge task for %s (req %s)", robot_name.c_str(), req.request_id.c_str());
}

bool ServerNode::is_request_valid(const std::string& _fleet_name, const std::string& _robot_name)
{
    if (_fleet_name != server_node_config.fleet_name)
        return false;

    ReadLock robot_states_lock(robot_states_mutex);
    auto it = robot_states.find(_robot_name);
    if (it == robot_states.end())
        return false;
    return true;
}

void ServerNode::transform_fleet_to_rmf(const rmf_fleet_msgs::msg::Location& _fleet_frame_location,  rmf_fleet_msgs::msg::Location& _rmf_frame_location) const
{
    // It feels easier to read if each operation is a separate statement.
    // The compiler will be super smart and elide all these operations.
    const Eigen::Vector2d translated =
        Eigen::Vector2d(_fleet_frame_location.x, _fleet_frame_location.y)
        - Eigen::Vector2d(
            server_node_config.translation_x, server_node_config.translation_y);

    // RCLCPP_INFO(
    //     get_logger(), "    fleet->rmf translated: (%.3f, %.3f)",
    //     translated[0], translated[1]);

    const Eigen::Vector2d rotated =
        Eigen::Rotation2D<double>(-server_node_config.rotation) * translated;

    // RCLCPP_INFO(
    //     get_logger(), "    fleet->rmf rotated: (%.3f, %.3f)",
    //     rotated[0], rotated[1]);

    const Eigen::Vector2d scaled = 1.0 / server_node_config.scale * rotated;

    // RCLCPP_INFO(
    //     get_logger(), "    fleet->rmf scaled: (%.3f, %.3f)",
    //     scaled[0], scaled[1]);

    _rmf_frame_location.x = scaled[0];
    _rmf_frame_location.y = scaled[1];
    _rmf_frame_location.yaw =
        _fleet_frame_location.yaw - server_node_config.rotation;

    _rmf_frame_location.t = _fleet_frame_location.t;
    _rmf_frame_location.level_name = _fleet_frame_location.level_name;
}

void ServerNode::transform_rmf_to_fleet(const rmf_fleet_msgs::msg::Location& _rmf_frame_location, rmf_fleet_msgs::msg::Location& _fleet_frame_location) const
{
    // It feels easier to read if each operation is a separate statement.
    // The compiler will be super smart and elide all these operations.
    const Eigen::Vector2d scaled =
        server_node_config.scale *
        Eigen::Vector2d(_rmf_frame_location.x, _rmf_frame_location.y);

    // RCLCPP_INFO(
    //     get_logger(), "    rmf->fleet scaled: (%.3f, %.3f)",
    //     scaled[0], scaled[1]);

    const Eigen::Vector2d rotated =
        Eigen::Rotation2D<double>(server_node_config.rotation) * scaled;

    // RCLCPP_INFO(
    //     get_logger(), "    rmf->fleet rotated: (%.3f, %.3f)",
    //     rotated[0], rotated[1]);

    const Eigen::Vector2d translated =
        rotated +
        Eigen::Vector2d(
            server_node_config.translation_x, server_node_config.translation_y);

    // RCLCPP_INFO(
    //     get_logger(), "    rmf->fleet translated: (%.3f, %.3f)",
    //     translated[0], translated[1]);

    _fleet_frame_location.x = translated[0];
    _fleet_frame_location.y = translated[1];
    _fleet_frame_location.yaw =
        _rmf_frame_location.yaw + server_node_config.rotation;

    _fleet_frame_location.t = _rmf_frame_location.t;
    _fleet_frame_location.level_name = _rmf_frame_location.level_name;
    _fleet_frame_location.approach_speed_limit = _rmf_frame_location.approach_speed_limit;
    _fleet_frame_location.obey_approach_speed_limit = _rmf_frame_location.obey_approach_speed_limit;

}

void ServerNode::handle_mode_request(rmf_fleet_msgs::msg::ModeRequest::UniquePtr _msg)
{
    messages::ModeRequest ff_msg;
    to_ff_message(*(_msg.get()), ff_msg);
        // to_ff_message(*(_msg.get()), ff_msg, task_dispatcher_client_);
    fields.server->send_mode_request(ff_msg);
}

void ServerNode::handle_path_request(rmf_fleet_msgs::msg::PathRequest::UniquePtr _msg)
{
    // Check if the path has at least two waypoints
    if (_msg->path.size() >= 2)
    {
        // Get the first and last waypoints
        const auto& first_wp = _msg->path.front();
        const auto& last_wp = _msg->path.back();
        
        // Compute the Euclidean distance between the first and last waypoints
        double dx = last_wp.x - first_wp.x;
        double dy = last_wp.y - first_wp.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double yaw_difference = std::abs(last_wp.yaw - first_wp.yaw);

        // If the distance is less than 1.0 and yaw difference is less than or equal to 0.1, ignore the request
        // if (distance < 0.05 && yaw_difference < 0.01)
        // {
        //     RCLCPP_INFO(
        //         get_logger(),
        //         "Ignoring path request: Distance (%.2f) < 1.0 and yaw difference (%.2f) <= 0.1",
        //         distance, yaw_difference);
        //     return;
        // }
    }

    // Continue with transforming the path waypoints
    for (std::size_t i = 0; i < _msg->path.size(); ++i)
    {
        rmf_fleet_msgs::msg::Location fleet_frame_waypoint;
        transform_rmf_to_fleet(_msg->path[i], fleet_frame_waypoint);
        _msg->path[i] = fleet_frame_waypoint;
    }

    messages::PathRequest ff_msg;
    to_ff_message(*(_msg.get()), ff_msg);
    fields.server->send_path_request(ff_msg);
}

void ServerNode::handle_destination_request(rmf_fleet_msgs::msg::DestinationRequest::UniquePtr _msg)
{
    rmf_fleet_msgs::msg::Location fleet_frame_destination;
    transform_rmf_to_fleet(_msg->destination, fleet_frame_destination);
    _msg->destination = fleet_frame_destination;

    messages::DestinationRequest ff_msg;
    to_ff_message(*(_msg.get()), ff_msg);
    fields.server->send_destination_request(ff_msg);
}

void ServerNode::handle_dispenser_request(rmf_dispenser_msgs::msg::DispenserRequest::UniquePtr msg)
{
    // 1) Print the incoming DispenserRequest
    RCLCPP_INFO(
        get_logger(),
        "Received DispenserRequest:\n"
        "  request_guid=%s\n"
        "  target_guid=%s\n"
        "  transporter_type=%s\n"
        "  items.size()=%zu",
        msg->request_guid.c_str(),
        msg->target_guid.c_str(),
        msg->transporter_type.c_str(),
        msg->items.size());

    // 2) Publish an ACKNOWLEDGED DispenserResult (status=0)
    rmf_dispenser_msgs::msg::DispenserResult ack_msg;
    ack_msg.time = now();                          // Current ROS time
    ack_msg.request_guid = msg->request_guid;      // Must match the incoming guid
    ack_msg.source_guid = msg->target_guid;        // E.g. the dispenser name
    ack_msg.status = rmf_dispenser_msgs::msg::DispenserResult::ACKNOWLEDGED; // 0

    dispenser_result_pub->publish(ack_msg);
    RCLCPP_INFO(
        get_logger(),
        "Published ACKNOWLEDGED DispenserResult (status=0) for request [%s]",
        ack_msg.request_guid.c_str());

    // ----------------------------------------------------------------------
    // In a real workflow, your robot or hardware might need time to do the
    // pickup. For example, you could:
    //
    //   1. Command the robot to pick up an item
    //   2. Wait for hardware confirmation
    //   3. Then publish SUCCESS
    //
    // Here, we'll just do a small sleep to simulate "pickup time" so that
    // you see two separate messages in the logs.

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s); // sleep 1 second

    // 3) Publish a SUCCESS DispenserResult (status=1)
    rmf_dispenser_msgs::msg::DispenserResult success_msg;
    success_msg.time = now();
    success_msg.request_guid = msg->request_guid;
    success_msg.source_guid = msg->target_guid;
    success_msg.status = rmf_dispenser_msgs::msg::DispenserResult::SUCCESS; // 1

    dispenser_result_pub->publish(success_msg);
    RCLCPP_INFO(
        get_logger(),
        "Published SUCCESS DispenserResult (status=1) for request [%s]",
        success_msg.request_guid.c_str());
}

void ServerNode::handle_ingestor_request(rmf_ingestor_msgs::msg::IngestorRequest::UniquePtr msg)
{
    // Log the contents of the received ingestor request
    RCLCPP_INFO(
        get_logger(),
        "Received IngestorRequest:\n"
        "  request_guid: %s\n"
        "  target_guid: %s\n"
        "  transporter_type: %s\n"
        "  items.size(): %zu",
        msg->request_guid.c_str(),
        msg->target_guid.c_str(),
        msg->transporter_type.c_str(),
        msg->items.size());

    // Publish an ACKNOWLEDGED IngestorResult (status = 0)
    rmf_ingestor_msgs::msg::IngestorResult ack_msg;
    ack_msg.time = now();                          // Current ROS time
    ack_msg.request_guid = msg->request_guid;      // Must match the incoming request
    ack_msg.source_guid = msg->target_guid;        // For example, the ingestor name
    ack_msg.status = rmf_ingestor_msgs::msg::IngestorResult::ACKNOWLEDGED; // Typically 0

    ingestor_result_pub->publish(ack_msg);
    RCLCPP_INFO(
        get_logger(),
        "Published ACKNOWLEDGED IngestorResult (status=0) for request [%s]",
        ack_msg.request_guid.c_str());

    // Simulate some processing time (pickup/drop-off delay)
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);

    // Publish a SUCCESS IngestorResult (status = 1)
    rmf_ingestor_msgs::msg::IngestorResult success_msg;
    success_msg.time = now();
    success_msg.request_guid = msg->request_guid;
    success_msg.source_guid = msg->target_guid;
    success_msg.status = rmf_ingestor_msgs::msg::IngestorResult::SUCCESS; // Typically 1

    ingestor_result_pub->publish(success_msg);
    RCLCPP_INFO(
        get_logger(),
        "Published SUCCESS IngestorResult (status=1) for request [%s]",
        success_msg.request_guid.c_str());
}

void ServerNode::update_state_callback()
{
    std::vector<messages::RobotState> new_robot_states;
    fields.server->read_robot_states(new_robot_states);

    for (const messages::RobotState& ff_rs : new_robot_states)
    {
        rmf_fleet_msgs::msg::RobotState ros_rs;
        to_ros_message(ff_rs, ros_rs);

        WriteLock robot_states_lock(robot_states_mutex);
        auto it = robot_states.find(ros_rs.name);
        if (it == robot_states.end())
        RCLCPP_INFO(
            get_logger(),
            "registered a new robot: [%s]",
            ros_rs.name.c_str());

        robot_states[ros_rs.name] = ros_rs;
    }
}

void ServerNode::publish_fleet_state()
{
    rmf_fleet_msgs::msg::FleetState fleet_state;
    fleet_state.name = server_node_config.fleet_name;
    fleet_state.robots.clear();

    ReadLock robot_states_lock(robot_states_mutex);
    for (const auto& it : robot_states)
    {
        const auto fleet_frame_rs = it.second;
        rmf_fleet_msgs::msg::RobotState rmf_frame_rs;

        transform_fleet_to_rmf(fleet_frame_rs.location, rmf_frame_rs.location);

        // RCLCPP_INFO(
        //     get_logger(),
        //     "robot location: (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f)",
        //     fleet_frame_rs.location.x,
        //     fleet_frame_rs.location.y,
        //     fleet_frame_rs.location.yaw,
        //     rmf_frame_rs.location.x,
        //     rmf_frame_rs.location.y,
        //     rmf_frame_rs.location.yaw);

        // // ------------------------------------------------------------------
        check_and_dispatch(rmf_frame_rs.name, fleet_frame_rs.location.x, fleet_frame_rs.location.y, /* use z if you care: */ fleet_frame_rs.name.empty()?0.0:0.0);

        rmf_frame_rs.name = fleet_frame_rs.name;
        rmf_frame_rs.model = fleet_frame_rs.model;
        // if we just got a trivial path request, bump the task_id by 1
        if (trivial_path_robots_.erase(fleet_frame_rs.name) > 0) {
            // parse the old task_id, increment, and re-stringify
            try {
                auto prev = std::stoull(fleet_frame_rs.task_id);
                rmf_frame_rs.task_id = std::to_string(prev + 1);
            }
            catch (const std::exception& e) {
                RCLCPP_WARN(
                get_logger(),
                "Failed to parse task_id as integer: %s;",e.what());
                return;
            }
        }
        else {
            rmf_frame_rs.task_id = fleet_frame_rs.task_id;
        }
        rmf_frame_rs.mode = fleet_frame_rs.mode;
        rmf_frame_rs.battery_percent = fleet_frame_rs.battery_percent;

        rmf_frame_rs.path.clear();
        for (const auto& fleet_frame_path_loc : fleet_frame_rs.path)
        {
            rmf_fleet_msgs::msg::Location rmf_frame_path_loc;
            transform_fleet_to_rmf(fleet_frame_path_loc, rmf_frame_path_loc);
            rmf_frame_rs.path.push_back(rmf_frame_path_loc);
        }

        // Add to fleet state
        fleet_state.robots.push_back(rmf_frame_rs);

        // Publish individual robot state to /robot_states
        robot_state_pub->publish(rmf_frame_rs);
    }
//   fleet_state_pub->publish(fleet_state); // why we need to publish fleet_state??? TPODAvia
}

} // namespace ros2
} // namespace free_fleet
