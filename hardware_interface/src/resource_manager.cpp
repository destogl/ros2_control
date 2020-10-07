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

#include "hardware_interface/resource_manager.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const auto kLoggerName = "ros2_control_resource_manager";

//TODO: All should be move those constants in a shared header with the parser
constexpr const auto kActuatorTypeName = "actuator";
constexpr const auto kSensorTypeName = "sensor";
constexpr const auto kSystemTypeName = "system";
}  // namespace

namespace hardware_interface
{

class ResourceStorage
{
  static constexpr const auto pkg_name = "hardware_interface";

  static constexpr const auto joint_component_interface_name =
    "hardware_interface::components::Joint";
  static constexpr const auto sensor_component_interface_name =
    "hardware_interface::components::Sensor";

  static constexpr const auto actuator_interface_name =
    "hardware_interface::ActuatorHardwareInterface";
  static constexpr const auto sensor_interface_name =
    "hardware_interface::SensorHardwareInterface";
  static constexpr const auto system_interface_name =
    "hardware_interface::SystemHardwareInterface";

public:
  ResourceStorage()
  : joint_component_loader_(pkg_name, joint_component_interface_name),
  sensor_component_loader_(pkg_name, sensor_component_interface_name),
  actuator_loader_(pkg_name, actuator_interface_name),
  sensor_loader_(pkg_name, sensor_interface_name),
  system_loader_(pkg_name, system_interface_name)
  {}

  ~ResourceStorage() = default;

  void initialize_joint_component(
    const hardware_interface::components::ComponentInfo & component_info)
  {
    joint_components_.emplace_back(
      std::unique_ptr<hardware_interface::components::Joint>(
        joint_component_loader_.createUnmanagedInstance(component_info.class_type)));
    joint_components_.back()->configure(component_info);
  }

  void initialize_sensor_component(
    const hardware_interface::components::ComponentInfo & component_info)
  {
    sensor_components_.emplace_back(
      std::unique_ptr<hardware_interface::components::Sensor>(
        sensor_component_loader_.createUnmanagedInstance(component_info.class_type)));
    sensor_components_.back()->configure(component_info);
  }

  template<class HardwareT, class HardwareInterfaceT>
  void initialize_hardware(
    const hardware_interface::HardwareInfo & hardware_info,
    pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT actuator(std::move(interface));
    container.emplace_back(std::move(actuator));
    container.back().configure(hardware_info);
  }

  void initialize_actuator(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::ActuatorHardware,
    hardware_interface::ActuatorHardwareInterface>(
      hardware_info, actuator_loader_, actuators_);
  }

  void initialize_sensor(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::SensorHardware,
    hardware_interface::SensorHardwareInterface>(
      hardware_info, sensor_loader_, sensors_);
  }

  void initialize_system(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::SystemHardware,
    hardware_interface::SystemHardwareInterface>(
      hardware_info, system_loader_, systems_);
  }

  // components plugins
  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader_;
  pluginlib::ClassLoader<hardware_interface::components::Sensor> sensor_component_loader_;

  std::vector<std::unique_ptr<hardware_interface::components::Joint>> joint_components_;
  std::vector<std::unique_ptr<hardware_interface::components::Sensor>> sensor_components_;

  // hardware plugins
  pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface> system_loader_;

  std::vector<hardware_interface::ActuatorHardware> actuators_;
  std::vector<hardware_interface::SensorHardware> sensors_;
  std::vector<hardware_interface::SystemHardware> systems_;
};

ResourceManager::ResourceManager()
: resource_storage_(std::make_unique<ResourceStorage>())
{
  actuator_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface>(
      "hardware_interface", "hardware_interface::ActuatorHardwareInterface"));
  sensor_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface>(
      "hardware_interface", "hardware_interface::SensorHardwareInterface"));
  system_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface>(
      "hardware_interface", "hardware_interface::SystemHardwareInterface"));

  joint_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::components::Joint>(
      "hardware_interface", "hardware_interface::components::Joint"));
}

//  No real-time safe functions
return_type
ResourceManager::load_and_configure_resources_from_urdf(std::string urdf_string)
{
  std::vector<hardware_interface::HardwareInfo> hardware_info_list =
    hardware_interface::parse_control_resources_from_urdf(urdf_string);

  return_type ret = return_type::OK;
  for (auto hardware_info : hardware_info_list) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "Loading hardware plugin: " + hardware_info.hardware_class_type);
    if (!hardware_info.type.compare(kSystemTypeName)) {
      // TODO(anyone): this here is really not nice...
      std::unique_ptr<hardware_interface::SystemHardwareInterface> sys_hw_if;
      sys_hw_if.reset(system_loader_->createUnmanagedInstance(hardware_info.hardware_class_type));
      std::shared_ptr<hardware_interface::SystemHardware> system_hw =
        std::make_shared<hardware_interface::SystemHardware>(std::move(sys_hw_if));
      ret = system_hw->configure(hardware_info);
      if (ret != return_type::OK) {
        return ret;
      }
      systems_.push_back(system_hw);
      // TODO(anyone): do the same for loading sensors and actuators hardware
    } else {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName),
        "hardware type not recognized");
      return return_type::ERROR;
    }

    std::vector<std::shared_ptr<hardware_interface::components::Joint>> joints;
    for (auto joint_info : hardware_info.joints) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "Loading joint plugin: " + joint_info.class_type);
      std::shared_ptr<hardware_interface::components::Joint> joint =
        joint_loader_->createSharedInstance(joint_info.class_type);
      ret = joint->configure(joint_info);
      if (ret != hardware_interface::return_type::OK) {
        return ret;
      }
      joints.push_back(joint);
      command_interfaces_[joint_info.name] = joint->get_command_interfaces();
      // TODO(anyone): add checking if Joint has state interfaces at all
      state_interfaces_[joint_info.name] = joint->get_state_interfaces();
      joint_components_[joint_info.name] = joint;
      claimed_command_interfaces_[joint_info.name] = std::vector<std::string>();
    }
    joint_components_for_hardware_[hardware_info.name] = joints;

    // TODO(anyone): add implementation for sensors
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "All hardware and component plugins loaded and configured successfully.");
  return return_type::OK;
}

return_type ResourceManager::start_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->start();
    if (ret != return_type::OK) {
      return ret;
    }
    // initial read of joints
    ret = system->read_joints(joint_components_for_hardware_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return return_type::OK;
}

return_type ResourceManager::stop_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->stop();
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return return_type::OK;
}

//  Real-time safe functions (at least the goal is to be...)
return_type ResourceManager::read_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->read_joints(joint_components_for_hardware_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return ret;
}

return_type ResourceManager::write_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->write_joints(joint_components_for_hardware_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return ret;
}

return_type ResourceManager::check_command_interfaces(
  const std::string & joint_name, const std::vector<std::string> & interfaces) const
{
  // Check joint existance
  if (command_interfaces_.find(joint_name) == command_interfaces_.end()) {
    // TODO(all): Do we need to return dedicated code?
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "There is no command interface for " + joint_name);
    return return_type::INTERFACE_NOT_FOUND;
  }

  // Check interface existance
  for (const auto & interface : interfaces) {
    if (std::find(
        command_interfaces_.at(joint_name).cbegin(),
        command_interfaces_.at(joint_name).cend(),
        interface) == command_interfaces_.at(joint_name).cend())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "There is no command interface '" + interface + "' found for " + joint_name);
      return return_type::INTERFACE_NOT_PROVIDED;
    }
  }

  return return_type::OK;
}

return_type ResourceManager::claim_command_handle(
  const std::string & joint_name, const std::vector<std::string> & interfaces,
  std::shared_ptr<hardware_interface::components::Joint> & command_handle)
{
  // Check joint existance
  if (joint_components_.find(joint_name) == joint_components_.end()) {
    // TODO(all): Do we need to return dedicated code?
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "There is no command handle interface for " + joint_name);
    return return_type::INTERFACE_NOT_FOUND;
  }

  // check for each interface if already claimed
  for (const auto & interface : interfaces) {
    if (std::find(
        claimed_command_interfaces_.at(joint_name).cbegin(),
        claimed_command_interfaces_.at(joint_name).cend(),
        interface) != claimed_command_interfaces_.at(joint_name).cend())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "The interface '" + interface + "' for " + joint_name + " is already claimed");
      return return_type::ALREADY_CLAIMED;
    }
  }

  command_handle = joint_components_[joint_name];
  // TODO(anyone) this could be done with `insert`...
  for (const auto & interface : interfaces) {
    claimed_command_interfaces_[joint_name].push_back(interface);
  }
  return return_type::OK;
}


}  // namespace resource_manager
