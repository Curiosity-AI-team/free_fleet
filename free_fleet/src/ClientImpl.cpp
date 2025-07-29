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

#include "ClientImpl.hpp"
#include <iostream>
#include "messages/message_utils.hpp"

namespace free_fleet {

Client::ClientImpl::ClientImpl(const ClientConfig& _config) :
  client_config(_config)
{}

Client::ClientImpl::~ClientImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Client::ClientImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Client::ClientImpl::send_robot_state(
    const messages::RobotState& _new_robot_state)
{
  FreeFleetData_RobotState* new_rs = FreeFleetData_RobotState__alloc();
  convert(_new_robot_state, *new_rs);
  bool sent = fields.state_pub->write(new_rs);
  FreeFleetData_RobotState_free(new_rs, DDS_FREE_ALL);
  return sent;
}

bool Client::ClientImpl::read_mode_request
    (messages::ModeRequest& _mode_request)
{
  auto mode_requests = fields.mode_request_sub->read();
  if (!mode_requests.empty())
  {
    convert(*(mode_requests[0]), _mode_request);
    return true;
  }
  return false;
}

bool Client::ClientImpl::read_path_request(
    messages::PathRequest& _path_request)
{
  auto path_requests = fields.path_request_sub->read();
  if (!path_requests.empty())
  {

      // --- print the *raw* DDS message before conversion ---
  // --- 1) Dump the raw DDS message contents: ---
  std::cout << "RAW DDS PathRequest waypoints:\n";
  try {
    // path_requests[0] is a shared_ptr<const FreeFleetData_PathRequest>
    auto raw = path_requests[0].get();
    for (uint32_t i = 0; i < raw->path._length; ++i) {
      auto& loc = raw->path._buffer[i];
      std::cout
        << "  [" << i << "]"
        << " x="    << loc.x
        << " y="    << loc.y
        << " speed_limit=" << loc.approach_speed_limit
        << " obey=" << std::boolalpha
        << loc.obey_approach_speed_limit
        << "\n";
    }
  } catch (const std::exception& e) {
    std::cout << "  (error printing raw DDS: " << e.what() << ")\n";
  }
    convert(*(path_requests[0]), _path_request);
    std::cout << "AAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPP \n";
  // now try to print waypoint #1
// After you’ve called convert(...,_path_request):
try {
  for (size_t i = 0; ; ++i) {
    const auto& wp = _path_request.path.at(i);  // throws when i >= size()
    std::cout
      << "waypoint[" << i << "] → "
      << "x="  << wp.x
      << ", y=" << wp.y
      << ", speed_limit=" << wp.approach_speed_limit
      << std::endl;
  }
}
catch (const std::out_of_range&) {
  std::cout
    << "Printed all waypoints (count=" << _path_request.path.size() << ")\n";
}

    return true;
  }
  return false;
}

bool Client::ClientImpl::read_destination_request(
    messages::DestinationRequest& _destination_request)
{
  auto destination_requests = fields.destination_request_sub->read();
  if (!destination_requests.empty())
  {
    convert(*(destination_requests[0]), _destination_request);
    return true;
  }
  return false;
}

bool Client::ClientImpl::read_task_request(
    messages::TaskRequest& _task_request)
{
  auto task_requests = fields.task_request_sub->read();
  if (!task_requests.empty())
  {
    convert(*(task_requests[0]), _task_request);
    return true;
  }
  return false;
}

} // namespace free_fleet
