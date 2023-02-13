# Surgical Robotics Challenge (SuRobChal) ROS2 Scripts
This repository contains experimental ROS Humble scripts to work with the setup of Surgical Robotics Challenge in AMBF simulator, in which:
- AMBF has been [fixed to work with Python3](https://github.com/macmacal/ambf/tree/bugfix/ros1_pkgs_build),
- The AMBF container image has been [extended with ROS1-ROS2 bridge](https://github.com/macmacal/docker-ambf/blob/ros2_bridge/galactic/Dockerfile),
  - and the ROS1 msgs has been ported to [ambf_ros2_msgs](https://github.com/macmacal/ambf_ros2_msgs),
- The Surgical Robotics Challange [container image has been simplified](https://github.com/macmacal/docker_surgical_robotics_challenge/tree/ros2_bridge) to use host's X sesion ( `xhost +local:root > /dev/null 2>&1` ),

**Note**: The last compatible Ubuntu version with ROS1 Noetic and any ROS2 distro was 20.04 (Galactic). That is the reason of selecting already EOL Galactic distro.

# W.I.P.