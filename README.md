# VisionDrone

A drone positioning system for environments with bad or no GNSS reception

# Project structure

* data - Matlab scrips for result analysis
  * analyze.m - tool to analyze single results file and compare it with GT
  * analyze_multi.m - tool to analyze multiple results file and compare them with each other and with GT
* non_ros - Several libraries and helper scripts for calibration, also third-party libraries linked by ros programs
* src - ROS workspace

# Notable ROS packages

* [visiondrone](src/visiondrone) - Main ROS package that glues all together and provides launch files to use the project
* [height_estimation](src/height_estimation) - A package to perform height estimation of the drone from barometer, IMU, and optional lidar
* [velocity_integrator](src/velocity_integrator) - A package to integrate velocity data into pose estimation, considering GPS datum

# ROS localizer packages

* [dead_reckoning](src/localizers/dead_reckoning) - A dead reckoning localization with IMU integratio-based odometry
* [motion_simulation](src/localizers/motion_simulation) - Motion simulation localization that uses rotor RPM info and IMU data for localization
* [orb_slam3](src/localizers/orb_slam3) - Localization based on the ORB-SLAM3 library
* [px4_optical_flow](src/localizers/px4_optical_flow) - Localization using KLT tracker or using PX4-FLOW algorithm

# License

The project is licensed under the GPL-3.0 license if not otherwise stated in source files or other files of this project. Copyright 2022 Klemen Pevec.

The project also uses several other libraries, that may be covered by a different license than GPL-3.0. Check folder of these libraries and their project pages for more info.
