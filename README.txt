Nav2 Costmap 2D deadlock reproducing code
https://github.com/ros-planning/navigation2/issues/3109

# How does it reproduces the bug?
- update costmap at 100Hz
- publish a map with different origins at 100Hz

# Launch
- ros2 launch deadlock_test test-launcy.py
  - rviz2
  - static transform publisher
  - lifecycle manager
    - test node

# Map
https://github.com/ros-planning/navigation2/tree/6a9508b772eb6f07cbfd350b3578aa9de2567d59/nav2_costmap_2d/test/map
