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

#include "test_chainable_controller_interface.hpp"

#include <gmock/gmock.h>

template <class T, size_t N>
constexpr size_t arrlen(T (&)[N])
{
  return N;
}

TEST(TestableChainableControllerInterface, default_returns)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  EXPECT_TRUE(controller.is_chainable());
  EXPECT_FALSE(controller.is_in_chained_mode());

  rclcpp::shutdown();
}

TEST(TestableChainableControllerInterface, export_reference_interfaces)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();

  ASSERT_EQ(reference_interfaces.size(), 1u);
  EXPECT_EQ(reference_interfaces[0].get_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(reference_interfaces[0].get_interface_name(), "test_itf");

  EXPECT_EQ(reference_interfaces[0].get_value(), INTERFACE_VALUE);

  rclcpp::shutdown();
}

TEST(TestableChainableControllerInterface, setting_chained_mode)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 1u);

  EXPECT_FALSE(controller.is_in_chained_mode());

  // Fail setting chained mode
  EXPECT_EQ(reference_interfaces[0].get_value(), INTERFACE_VALUE);

  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  EXPECT_FALSE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  // Success setting chained mode
  reference_interfaces[0].set_value(0.0);

  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  controller.configure();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->activate();
  // Can not change chained mode until in "ACTIVE" state
  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->deactivate();
  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  // Can change 'chained' mode only in "UNCONFIGURED" state
  controller.get_node()->cleanup();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  rclcpp::shutdown();
}

TEST(TestableChainableControllerInterface, reference_interfaces_storage_not_correct_size)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  // expect empty return because storage is not resized
  controller.reference_interfaces_.clear();
  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces.empty());
}

TEST(TestableChainableControllerInterface, reference_interfaces_prefix_is_not_node_name)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);

  TestableChainableControllerInterface controller;

  // initialize, create node
  ASSERT_EQ(controller.init(TEST_CONTROLLER_NAME), controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  controller.set_name_prefix_of_reference_interfaces("some_not_correct_interface_prefix");

  // expect empty return because interface prefix is not equal to the node name
  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces.empty());
}
