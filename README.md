# ROS2/Autoware.universe bridge for CARLA simulator

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

 The official [ros-bridge](https://github.com/carla-simulator/ros-bridge) is modified.   
 This ros package enables autonomous driving using Autoware in addition to the basic function of the official [ros-bridge](https://github.com/carla-simulator/ros-bridge) package (communication between ros and carla)

![control_carla_universe](https://user-images.githubusercontent.com/38074802/187683146-2b8d492b-6997-4460-af1a-66ea364c90ed.gif)

# Environment 
|ubuntu|ros|carla|autoware|
|:---:|:---:|:---:|:---:|
|20.04|galactic|0.9.12|universe/master|

### example  
- 11th Gen Intel® Core™ i7-11700F @ 2.50GHz × 16
- GeForce RTX3070
- 32GB mem

# Setup
## install
* [Autoware.Universe](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) (Checkout to `galactic` branch)
* [CARLA Installation](https://carla.readthedocs.io/en/latest/start_quickstart/) (Debian installation)
* carla-ros-bridge (this repo)
  ```bash
  mkdir -p ros2_ws/src
  cd ros2_ws/src
  git clone git@github.com:kuriatsu/ros-bridge.git --recursive
  ```
* [derived_object_msgs](https://github.com/astuff/astuff_sensor_msgs) (branch 3.3.0)
  ```bash
  cd ros2_ws/src
  git clone git@github.com:astuff/astuff_sensor_msgs.git
  cd astuff_sensor_msgs
  git checkout -b 3.3.0 refs/tags/3.3.0
  ```
* [autoware containts](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)  
  1. Download maps (y-axis inverted version) to arbitaly location
  2. Change names. (point_cloud/Town01.pcd -> Town01/pointcloud_map.pcd, vector_maps/lanelet2/Town01.osm -> Town01/lanelet2_map.osm)
  
* Change file path at `carla_spawn_objects/carla_spawn_objects.py L49` as shown in first paragraph of Fix Log.

## build
```bash
cd ros2_ws
colcon build --symlink-install
```
# Getting start

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
ros2 launch autoware_bridge autoware_bridge_launch.xml 
ros2 launch autoware_bridge autoware_launch.xml map_path:=<path-to-map-dir>/Town10 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

3. Set initial pose
4. Set goal position
5. Wait for planning
6. Engage (use web interface or Rviz plugin (AutowareStatePanel))

# Fix Log
carla_spawn_objects/carla_spawn_objects.py L49 : Fixed bag to load ego vehicle definition file
```python
self.objects_definition_file = self.get_param('objects_definition_file', '<path-to-this-pkg>/carla_spawn_objects/config/objects.json')
```

pcl_recorder/src/PclRecorderROS2.cpp L29 : fixed build error
```python
# transform = tf2::transformToEigen (tf_buffer_->lookupTransform(fixed_frame_, cloud->header.frame_id,  cloud->header.stamp, rclcpp::Duration(1)));
transform = tf2::transformToEigen (tf_buffer_->lookupTransform(fixed_frame_, cloud->header.frame_id,  cloud->header.stamp, rclcpp::Duration::from_seconds(1)));

# sub_opt.callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

carla_ros_bridge/bridge.py L430 : fixed map loading bag
```python
# if "town" in parameters and not parameters['passive']:
#     if parameters["town"].endswith(".xodr"):
#         carla_bridge.loginfo(
#             "Loading opendrive world from file '{}'".format(parameters["town"]))
#         with open(parameters["town"]) as od_file:
#             data = od_file.read()
#         carla_world = carla_client.generate_opendrive_world(str(data))
#     else:
#         if carla_world.get_map().name != parameters["town"]:
#             carla_bridge.loginfo("Loading town '{}' (previous: '{}').".format(
#                 parameters["town"], carla_world.get_map().name))
#             carla_world = carla_client.load_world(parameters["town"])
#     carla_world.tick()
```

carla_spawn_objects/config/objects.json : Remove tf sensor

ros_compatibility/src/ros_compatibility/node.py L137 : Fix error of /use_sim_time
```python
param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
#            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
super(CompatibleNode, self).__init__(
    name,
    allow_undeclared_parameters=True,
    automatically_declare_parameters_from_overrides=True,
    parameter_overrides=[param],
#                automatically_declare_parameters_from_overrides=True,
    automatically_declare_parameters_from_overrides=False,
#                parameter_overrides=[param],
    parameter_overrides=[],

```

carla_spawn_objects/launch/carla_example_ego_vehicle.launch.py L41 : Remove function that change ego_vehicle position by /initial_pose
```python
#        launch.actions.IncludeLaunchDescription(
#            launch.launch_description_sources.PythonLaunchDescriptionSource(
#                os.path.join(get_package_share_directory(
#                    'carla_spawn_objects'), 'set_initial_pose.launch.py')
#            ),
#            launch_arguments={
#                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
#                'control_id': launch.substitutions.LaunchConfiguration('control_id')
#            }.items()
#        )
```

# TODO
- Need to refine vehicle controller at [autoware_bridge/src/carla_vehicle_interface.cpp L82](https://github.com/kuriatsu/ros-bridge/blob/cdc593b26c123440e7d92fec71674b8a12f1881b/autoware_bridge/src/carla_vehicle_interface.cpp#L82)
