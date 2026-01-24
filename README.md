# Multi-SLAM (ORB-SLAM3) with 2 PX4 Drones in Gazebo + ROS 2

This repository documents a multi-drone setup (2 PX4 SITL instances) in Gazebo (gz) integrated with ROS 2 to:

- Simulate 2 PX4 SITL drones with sensors (Stereo or RGB-D).
- Bridge Gazebo ↔ ROS 2 using `ros_gz_bridge`.
- Run ORB-SLAM3 via the `orb_slam3_ros2_wrapper` (inside Docker) for tracking and mapping.
- Send offboard commands to the drones using `px4_ros_com` while ORB-SLAM3 runs.
- Connect with COVINS and build a combined map using information from both drones (currently working).

---

## 1) Requirements & Context

Base software components used by this project:

- PX4 Autopilot with Gazebo (gz) simulation support. The simulation relies on environment variables such as `PX4_SIM_MODEL` and other options to spawn multiple instances.
- ROS 2 (the same distro you use in your workspace).
- MicroXRCEAgent for the Micro XRCE-DDS bridge (used by PX4 for DDS/ROS 2 communication). The agent runs as a separate process.
- `ros_gz_bridge` to bridge topics between Gazebo and ROS 2.
- A fixed `ROS_DOMAIN_ID` (important).

We use a fixed ROS domain to isolate this project's DDS/ROS2 network and avoid interference with other DDS participants on the same machine or network. ROS 2 documents the use of `ROS_DOMAIN_ID` and its impact on discovery and DDS ports.

In this project:
```
export ROS_DOMAIN_ID=55
```

Recommendation: add that line to your `~/.bashrc` if you always work with this stack.

---

## 2) ROS 2 dependencies for PX4 (local workspace)

This setup assumes you already followed the standard PX4 ↔ ROS 2 integration tutorial and have a built workspace (via `colcon`) containing the typical PX4-ROS 2 packages, including:

- `px4_ros_com`
- `px4_msgs`

These packages enable running examples and nodes for offboard control in ROS 2.

---

## 3) ORB-SLAM3 Wrapper (Docker + ROS 2)

We use the following repository/container for running ORB-SLAM3 with a ROS 2 wrapper:

- `suchetanrs/ORB-SLAM3-ROS2-Docker`

Typical workflow for that wrapper:

- clone the repository (and its submodules)
- install Docker
- build the Docker image
- run the container and launch the ROS 2 `launch` files provided by the wrapper

Local changes made so far:
- YAML parameter files for the wrapper (e.g. `params/ros_params/*`, such as `euroc_ros_params.yaml`) were modified to rename camera topics and match the topics published by `ros_gz_bridge`.
- Specifically, the wrapper's `stereo.launch.py` subscribes to:
  - `/cam_0/image`
  - `/cam_1/image`

Note: those topics come from the bridge configuration file `gz_bridge_stereo.yaml`.

---

## 4) Initial build (before running)

4.1 Local ROS 2 workspace

From your workspace:
```bash
cd ~/ws_offboard_control
colcon build
source install/setup.bash
```

4.2 ORB-SLAM3 Docker wrapper

Follow the build and run instructions in the `suchetanrs/ORB-SLAM3-ROS2-Docker` repository to build the image and start the container.

---

## 5) Runtime routine (commands)

All ROS 2 commands must run with the same `ROS_DOMAIN_ID=55`.

5.0 Export the Domain ID (in every terminal)
```bash
export ROS_DOMAIN_ID=55
```

5.1 Launch PX4 SITL instance 1 (drone 1)

Terminal 1:
```bash
cd ~/PX4-Autopilot
source venv/px4/bin/activate   # if applicable for your installation
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 1
```

`PX4_SIM_MODEL` defines the model that Gazebo spawns and associates with that PX4 instance.

5.2 Launch PX4 SITL instance 2 (drone 2)

Terminal 2:
```bash
cd ~/PX4-Autopilot
source venv/px4/bin/activate   # if applicable
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 2
```

Notes:
- The PX4 docs describe the role of variables like `PX4_SIM_MODEL` and how they affect instance spawn/bind.
- `PX4_GZ_STANDALONE=1` is commonly used when running multiple instances or when a separate Gazebo server is already present (see PX4 docs/discussions for exact flows).

5.3 Micro XRCE Agent (for PX4 ↔ DDS/ROS 2)

Terminal 3:
```bash
MicroXRCEAgent udp4 -p 8888
```

5.4 Bridge Gazebo ↔ ROS 2 (camera topics, etc.)

Terminal 4:
```bash
cd ~/ws_offboard_control
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/carlos/ws_offboard_control/src/multi_slam/config/gz_bridge_stereo.yaml -p use_sim_time:=true
```

This bridge publishes (among others) the topics ORB-SLAM3 needs (for example `/cam_0/image` and `/cam_1/image`).

5.5 Launch ORB-SLAM3 wrapper (stereo tracking)

Terminal 5 (inside the environment where the wrapper runs, e.g., inside the Docker container):
```bash
ros2 launch orb_slam3_ros2_wrapper stereo.launch.py
```

This launch runs ORB-SLAM3 tracking and subscribes to:
- `/cam_0/image`
- `/cam_1/image`

5.6 Offboard control (send waypoints while SLAM runs)

Terminal 6:
```bash
cd ~/ws_offboard_control
source install/setup.bash
ros2 run px4_ros_com offboard_control
```

This node allows sending offboard/waypoint commands to the drone while SLAM is running.

---
