# Copyright 2020 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2cli.node.direct import add_arguments
from ros2cli.verb import VerbExtension
from ros2controlcli.api import add_controller_mgr_parsers, list_controller_interfaces


class ListInterfacesVerb(VerbExtension):
    """Output the list of loaded controllers, their type and status."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        controller_interfaces = list_controller_interfaces(args.controller_manager)
        controller_interfaces.command_interfaces.sort()
        controller_interfaces.state_interfaces.sort()
        print('command interfaces')
        for command_interface in controller_interfaces.command_interfaces:
            print('\t', command_interface)
        print('state interfaces')
        for state_interface in controller_interfaces.state_interfaces:
            print('\t', state_interface)
