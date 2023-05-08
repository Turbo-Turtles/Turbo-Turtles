# LiDAR Navigation

### Explanation

All the important directories explained.

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

You need to take care of `package.xml` and `setup.py` aswell.

In `package.xml` you need to keep updating your dependencies, if you use any more libraries.

In `setup.py` you need to set your new python scripts as ros2 executables.