# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

import xml.etree.ElementTree as ET

from scout_description import (
    generate_ros2_control_description,
    generate_urdf_description,
)


def urdf_xml(mode, model):
    return ET.fromstring(
        generate_urdf_description(
            "robot_", mode, "base", model, "controller_manager.yaml", "/robot/"
        )
    )


def ros2_control_xml(mode, model):
    return ET.fromstring(generate_ros2_control_description("robot_", mode, "base", model))


def test_footprint_link_name():
    assert urdf_xml("live", "mini").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():
    assert (
        ros2_control_xml("live", "mini").find("ros2_control/hardware/plugin").text
        == "scout_hardware/ScoutMiniHardware"
    )

    assert (
        ros2_control_xml("live", "v2").find("ros2_control/hardware/plugin").text
        == "scout_hardware/ScoutV2Hardware"
    )

    assert (
        ros2_control_xml("simulation_gazebo", "mini").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface"
    )

    assert (
        ros2_control_xml("simulation_gazebo", "v2").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface"
    )

    assert (
        ros2_control_xml("simulation_gazebo_classic", "mini").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface4WD"
    )

    assert (
        ros2_control_xml("simulation_gazebo_classic", "v2").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface4WD"
    )


def test_controller_filename_name():
    assert (
        urdf_xml("simulation", "mini").find("gazebo/plugin/parameters").text
        == "controller_manager.yaml"
    )
