# Third-Party Notices

This repository integrates and adapts third-party software.  
Copyright for each original project remains with its respective authors.

## Main External Projects Used

| Project | Original Repository | License (reference) | Use in this repository |
| --- | --- | --- | --- |
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | GPLv3 | Core SLAM frontend, adapted for this integration |
| COVINS | https://github.com/VIS4ROB-lab/covins | GPLv3 | CSLAM backend (ROS 1 Docker), configuration and workflow adjustments |
| orb_slam3_ros2_wrapper |   https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker | Check source repository license | ROS2 wrapper, modified for multi-agent/COVINS integration |
| ORB-SLAM3 ROS2 library base |  https://github.com/zang09/ORB-SLAM3-STEREO-FIXED | Check source repository | ORB_SLAM3 package, modified for COVINS integration |
| px4_msgs | https://github.com/PX4/px4_msgs | BSD-3-Clause | PX4 message definitions for ROS 2 |
| px4_msgs | https://github.com/PX4/px4_msgs | BSD-3-Clause | PX4 message definitions for ROS 2 |
| px4_ros_com | https://github.com/PX4/px4_ros_com | BSD-3-Clause | PX4-ROS 2 interface and offboard control |
| ros-jazzy-ros1-bridge-builder | https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder | Check source repository license | Build/support for `ros1_bridge` in a Jazzy environment |

## Note About Modifications

This project includes integration changes across multiple third-party packages (configuration, launch files, frontend-backend communication, multi-agent support, etc.).  
Attribution to original authors is preserved, and each upstream repository should be checked for complete licensing and citation requirements.

## Scope Of This Repository

The purpose of this repository is to document a **system integration** for research/development (multi-drone SLAM + CSLAM), not to claim authorship of ORB-SLAM3, COVINS, PX4, or other external projects.
