# LiDAR Navigation

### Explanation

All the important directories/files explained.

#### <u>config</u>

- burger_params.yaml --> config file for navigation2 using turtlebot model burger
- navigation.rviz --> config file for rviz2
- tb3_map.* --> map files
- turtlebot3_lds_2d.lua --> config file for ros2 cartographer

#### <u>launch</u>

- contains launch files

#### <u>lidar_navigation</u>

- contains python scripts, that create our ros2 nodes

---

For testing purposes:

`ros2 topic pub -1 /mission turtlebot3_interfaces/msg/Mission "{mission_name: 'tunnel'}"`