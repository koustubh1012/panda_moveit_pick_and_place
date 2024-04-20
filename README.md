# ROS2 Package: Panda Moveit pick and place

## Overview
This ROS2 package provides moveit functionalities for Panda robot.

## Dependencies
- ROS2 (Robot Operating System 2)
- panda_description ROS2 packages
- moveit2 ROS2 package
- control_msgs ROS2

## Installation
1. Clone this repository into your ROS2 workspace.
2. Install dependencies using ROS2 package manager (e.g., `rosdep` or `rosdep install --from-paths src --ignore-src -r -y`).
3. Build the package using colcon build system.

## Usage
1. Launch the Panda Visualization in Rviz:
    ```
    $ ros2 launch package_120273766 demo.launch.py
    ```
2. Run the pick and place executable:
    ```
    $ ros2 run package_120273766 pick_and_place
    ```
       
## Troubleshooting
- Ensure all dependencies are correctly installed.
- Check ROS2 logs for error messages.
- Verify hardware connections and configurations if using real hardware.

## Maintainer
- FNU Koustubh <koustubh@umd.edu>
