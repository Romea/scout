# scout

## Overview

`scout` groups the ROS2 packages that describe, launch and control Scout mobile bases in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `scout` | Metapackage that groups the Scout ROS2 packages. |
| `scout_description` | Robot-specific description layer for Scout variants, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `scout_bringup` | Main integration entry point for generating Scout configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `scout_hardware` | Live `ros2_control` hardware plugin for Scout mobile bases, built on the generic `4WD` hardware abstraction. |

## Usage

In most cases, start with `scout_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Scout model is selected from a mobile base meta-description.

The Scout stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `4WD` and it is commanded as a skid-steering robot; `scout_description` provides the concrete geometry and generated descriptions, `scout_hardware` provides the live hardware implementation, and `scout_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `scout` project was developed by Jean Laneurit in the context of the TIRREX ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
