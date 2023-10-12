# ROS2/Autoware.universe bridge for CARLA simulator

 This ros package enables autonomous driving using Autoware in addition to the basic function of the official [ros-bridge](https://github.com/carla-simulator/ros-bridge) package (communication between ros and carla)

![control_carla_universe](https://user-images.githubusercontent.com/38074802/187683146-2b8d492b-6997-4460-af1a-66ea364c90ed.gif)

# Environment 
|ubuntu|ros|carla|autoware|
|:---:|:---:|:---:|:---:|
|22.04|humble|0.9.13|universe/master|

### example  
- 11th Gen Intel® Core™ i7-11700F @ 2.50GHz × 16
- GeForce RTX3070
- 32GB mem

# Setup
## install
* [Autoware.Universe](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) 
* [CARLA Installation](https://carla.readthedocs.io/en/latest/start_quickstart/) 
* carla-ros-bridge (this repo)
  ```bash
  mkdir -p colcon_ws/src
  cd colcon_ws/src
  git clone git@github.com:kuriatsu/carla-autoware.git --recursive
  rosdep install -i --from-paths src
  ```
* [autoware containts](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)  
  1. Download maps (y-axis inverted version) to arbitaly location
  2. Change names. (point_cloud/Town01.pcd -> Town01/pointcloud_map.pcd, vector_maps/lanelet2/Town01.osm -> Town01/lanelet2_map.osm)
  
* Change file path at `ros-bridge/carla_spawn_objects/carla_spawn_objects.py L49` as shown in first paragraph of Fix Log.

## build
```bash
cd colcon_ws
colcon build --symlink-install
```

# Run
1. Run carla, change map, spawn object if you need
```bash
/opt/carla/CarlaUE4.sh
python3 /opt/carla/PythonAPI/util/config.py -m Town10
# optional
python3 /opt/carla/PythonAPI/examples/generate_traffic.py -n 10 -w 20
```

2. Run ros nodes
```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
ros2 launch carla_autoware carla_autoware.xml 
ros2 launch autoware_bridge autoware_launch.xml map_path:=<path-to-map-dir>/Town10 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

3. Set initial pose
4. Set goal position
5. Wait for planning
6. Engage (use web interface or Rviz plugin (AutowareStatePanel))


# TODO
- Need to refine vehicle controller at [autoware_bridge/src/carla_vehicle_interface.cpp L82](https://github.com/kuriatsu/ros-bridge/blob/cdc593b26c123440e7d92fec71674b8a12f1881b/autoware_bridge/src/carla_vehicle_interface.cpp#L82)
