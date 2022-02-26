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

#include "controller_interface/chainable_controller_interface.hpp"

#include <vector>

namespace controller_interface
{
bool ChainableControllerInterface::is_chainable() const { return true; }

std::vector<hardware_interface::CommandInterface>
ChainableControllerInterface::export_reference_interfaces()
{
  return do_export_reference_interfaces();
}

bool ChainableControllerInterface::set_chained_mode(bool chained_mode)
{
  bool result = do_set_chained_mode(chained_mode);

  if (result)
  {
    in_chained_mode_ = chained_mode;
  }

  return result;
}

bool ChainableControllerInterface::is_in_chained_mode() const { return in_chained_mode_; }

}  // namespace controller_interface
