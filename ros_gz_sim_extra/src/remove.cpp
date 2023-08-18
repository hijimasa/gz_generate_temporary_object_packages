// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gflags/gflags.h>
#include <ignition/msgs/entity.pb.h>

#include <sstream>
#include <string>
#include <stdlib.h>

#include <ignition/math/Pose3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


DEFINE_string(world, "", "World name.");
DEFINE_string(id, "", "ID for removed entity");
DEFINE_string(name, "", "Name for removed entity.");

// ROS interface for spawning entities into Gazebo.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `gz service` command line instead.
int main(int _argc, char ** _argv)
{
  rclcpp::init(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim");

  gflags::AllowCommandLineReparsing();
  gflags::SetUsageMessage(
    R"(Usage: remove -world [arg] [-name NAME])");
  gflags::ParseCommandLineFlags(&_argc, &_argv, true);

  // World
  std::string world_name = FLAGS_world;
  if (world_name.empty()) {
    // If caller doesn't provide a world name, get list of worlds from gz-sim server
    ignition::transport::Node node;

    bool executed{false};
    bool result{false};
    unsigned int timeout{5000};
    std::string service{"/gazebo/worlds"};
    ignition::msgs::StringMsg_V worlds_msg;

    // This loop is here to allow the server time to download resources.
    while (rclcpp::ok() && !executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
      executed = node.Request(service, timeout, worlds_msg, result);
    }

    if (!executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Timed out when getting world names.");
      return -1;
    }

    if (!result || worlds_msg.data().empty()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Failed to get world names.");
      return -1;
    }

    world_name = worlds_msg.data(0);
  }
  std::string service{"/world/" + world_name + "/remove"};

  // Request message
  ignition::msgs::Entity req;

  // ID
  if (!FLAGS_id.empty() && atoi(FLAGS_id.c_str()) != 0) {
    req.set_id(atoi(FLAGS_id.c_str()));
  } else {
    RCLCPP_ERROR(ros2_node->get_logger(), "Must set id");
    return -1;
  }

  // Name
  if (!FLAGS_name.empty()) {
    req.set_name(FLAGS_name);
  }

  // Request
  ignition::transport::Node node;
  ignition::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;
  bool executed = node.Request(service, req, timeout, rep, result);

  if (executed) {
    if (result && rep.data()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requested to remove entity.");
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed request to remove entity.\n %s",
        req.DebugString().c_str());
    }
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Request to remove entity from service [%s] timed out..\n %s",
      service.c_str(), req.DebugString().c_str());
  }
  RCLCPP_INFO(ros2_node->get_logger(), "OK creation of entity.");

  return 0;
}
