// Copyright 2020 PAL Robotics SL
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

#include "controller_manager/controller_loader_pluginlib.hpp"

#include <memory>
#include <string>
#include <vector>

namespace controller_manager
{

ControllerLoaderPluginlib::ControllerLoaderPluginlib()
: ControllerLoaderInterface("controller_interface::ControllerInterface")
{
  reload();
}

controller_interface::ControllerInterfaceSharedPtr ControllerLoaderPluginlib::create(
  const std::string & controller_type)
{
  return loader_->createSharedInstance(controller_type);
}

std::vector<std::string> ControllerLoaderPluginlib::get_declared_classes() const
{
  return loader_->getDeclaredClasses();
}

bool ControllerLoaderPluginlib::is_available(const std::string & controller_type) const
{
  return loader_->isClassAvailable(controller_type);
}

void ControllerLoaderPluginlib::reload()
{
  loader_ = std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
    "controller_interface", "controller_interface::ControllerInterface");
}

}  // namespace controller_manager
