// Copyright 2020 ROS2-Control Development Team
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

#ifndef HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
#define HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "pluginlib/class_loader.hpp"

using hardware_interface::return_type;

namespace resource_manager
{

class ResourceManager
{
public:
  HARDWARE_INTERFACE_PUBLIC
  ResourceManager();

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~ResourceManager() = default;

  //  Non real-time safe functions
  HARDWARE_INTERFACE_PUBLIC
  return_type load_and_configure_resources_from_urdf(std::string urdf_string);

  HARDWARE_INTERFACE_PUBLIC
  return_type start_all_resources();

  HARDWARE_INTERFACE_PUBLIC
  return_type stop_all_resources();

  //  Real-time safe functions (at least the goal is to be...)
  HARDWARE_INTERFACE_PUBLIC
  return_type read_all_resources();

  HARDWARE_INTERFACE_PUBLIC
  return_type write_all_resources();

  HARDWARE_INTERFACE_PUBLIC
  return_type check_command_interfaces(
    const std::string & joint_name, const std::vector<std::string> & interfaces) const;

  HARDWARE_INTERFACE_PUBLIC
  return_type check_state_interfaces();

  HARDWARE_INTERFACE_PUBLIC
  return_type claim_command_handle(
    const std::string & joint_name, const std::vector<std::string> & interfaces,
    std::shared_ptr<hardware_interface::components::Joint> & command_handle);

private:
  // TODO(all): make this unique?
  std::vector<std::shared_ptr<hardware_interface::ActuatorHardware>> actuators_;
  std::vector<std::shared_ptr<hardware_interface::SensorHardware>> sensors_;
  std::vector<std::shared_ptr<hardware_interface::SystemHardware>> systems_;

  std::map<std::string, std::shared_ptr<
      hardware_interface::components::Joint>> joint_components_;
  std::map<std::string, std::vector<std::shared_ptr<
      hardware_interface::components::Joint>>> joint_components_for_hardware_;
  std::map<std::string, std::shared_ptr<
      hardware_interface::components::Sensor>> sensor_components_;
  std::map<std::string, std::vector<std::shared_ptr<
      hardware_interface::components::Sensor>>> sensor_components_for_hardware_;

  /**
   * Map of joints and their command interfaces
   */
  std::map<std::string, std::vector<std::string>> command_interfaces_;

  std::map<std::string, std::vector<std::string>> claimed_command_interfaces_;

  /**
   * Map of joints and their state interfaces
   */
  std::map<std::string, std::vector<std::string>> state_interfaces_;

  std::shared_ptr<pluginlib::ClassLoader<
      hardware_interface::ActuatorHardwareInterface>> actuator_loader_;
  std::shared_ptr<pluginlib::ClassLoader<
      hardware_interface::SensorHardwareInterface>> sensor_loader_;
  std::shared_ptr<pluginlib::ClassLoader<
      hardware_interface::SystemHardwareInterface>> system_loader_;

  std::shared_ptr<pluginlib::ClassLoader<hardware_interface::components::Joint>> joint_loader_;
};

}  //  namespace resource_manager

#endif  // HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_