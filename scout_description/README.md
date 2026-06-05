# scout_description

## 1) Overview

`scout_description` provides the robot-specific description layer for Scout mobile bases.

It extends `romea_mobile_base_description` with the concrete configurations, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate Scout robots.

The package currently describes:

| Configuration file | Model | Mobile base architecture | Command type |
|---|---|---|---|
| `config/scout_mini.yaml` | `mini` | `4WD` | `skid_steering` |
| `config/scout_v2.yaml` | `v2` | `4WD` | `skid_steering` |

Both models have four driving wheels and no steering joints. They are controlled as skid-steering mobile bases.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `scout_mini.yaml` | full Scout Mini mobile base configuration |
| `scout_v2.yaml` | full Scout V2 mobile base configuration |
| `teleop.yaml` | default skid-steering teleoperation configuration |

Each robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`4WD`);
* geometry, wheel dimensions and chassis bounding box;
* wheel speed command and feedback information;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/scout_mini.urdf.xacro` | main URDF entry point for Scout Mini |
| `urdf/scout_v2.urdf.xacro` | main URDF entry point for Scout V2 |
| `urdf/scout.xacro` | common Scout mobile base macro |
| `urdf/scout.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis and wheels |
| `meshes/` | Scout chassis and wheel meshes |

The Scout macro reuses the `base4WD.chassis.xacro` template from `romea_mobile_base_description` and specializes it with the selected Scout model geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/scout_mini.ros2_control.urdf.xacro` | main `ros2_control` entry point for Scout Mini |
| `ros2_control/scout_v2.ros2_control.urdf.xacro` | main `ros2_control` entry point for Scout V2 |
| `ros2_control/scout.ros2_control.xacro` | common Scout `ros2_control` macro |

Depending on the selected model and mode, the `ros2_control` description selects:

| Model | Mode | Hardware plugin |
|---|---|---|
| `mini` | `live` | `scout_hardware/ScoutMiniHardware` |
| `v2` | `live` | `scout_hardware/ScoutV2Hardware` |
| `mini`, `v2` | `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface4WD` |
| `mini`, `v2` | `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface4WD` |

## 4) Python API

The installed Python module provides helper functions used by `scout_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file(robot_model)` | returns the selected Scout configuration file path |
| `get_specifications_configuration(robot_model)` | loads the full selected Scout configuration |
| `get_configuration(robot_model)` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the selected Scout URDF description |
| `generate_ros2_control_description(...)` | generates the selected Scout `ros2_control` description |
| `urdf(...)` | compatibility wrapper around `generate_urdf_description(...)` |

Example:

```python
from scout_description import get_configuration

configuration = get_configuration("mini")
```

## 5) Relation with other packages

`scout_description` is the Scout specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `4WD` description and `ros2_control` templates;
* `scout_description` provides Scout Mini and Scout V2 configurations, meshes and Xacro specializations;
* `scout_hardware` provides the live `ros2_control` hardware plugins;
* `scout_bringup` uses this package to generate URDF and `ros2_control` artifacts for live and simulation modes.
