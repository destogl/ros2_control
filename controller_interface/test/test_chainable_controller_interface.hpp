// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
#define TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_

#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr char TEST_CONTROLLER_NAME[] = "testable_chainable_controller";

class TestableChainableControllerInterface
: public controller_interface::ChainableControllerInterface
{
public:
  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  // Implementation of ChainableController virtual methods
  std::vector<hardware_interface::CommandInterface> do_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.push_back(
      hardware_interface::CommandInterface(node_->get_name(), "test_itf", &cmd_interface_value));

    return command_interfaces;
  }

  bool do_set_chained_mode(bool chained_mode) override
  {
    if (cmd_interface_value == 0.0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

public:
  double cmd_interface_value = 1989.0;
};

#endif  // TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
