# VisionDrone

A drone positioning system for environments with bad or no GNSS reception

# Setup

First, build non_ros libraries:

1. Pangolin
2. orb_slam3

Then install dependencies needed by ROS:

`rosdep install --from-paths src --ignore-src -r -y`

Then run `catkin_make` from root.