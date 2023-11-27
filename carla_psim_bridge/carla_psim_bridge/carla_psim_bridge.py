import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import carla
import numpy as np 
from carla_common.transforms import ros_quaternion_to_carla_rotation, ros_point_to_carla_location


class CarlaPsimBridge(Node):

    def __init__(self, world):
        super().__init__('carla_psim_bridge')

        self.target_frame = self.declare_parameter("target_frame", "base_link").get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_cb)

        # self.sub_blinker = self.create_subscription( String, 'topic', self.blinker_cb, 10)

        self.current_blinker = 0

        self.ego_vehicle = None
        for actor in world.get_actors():
            if actor.attributes.get("role_name") in ["hero", "ego_vehicle"]:
                self.ego_vehicle = actor


    def timer_cb(self):
        target_frame = self.target_frame
        source_frame = "map"

        if self.ego_vehicle is None:
            self.get_logger().info(f"cannot get ego vehicle in CARLA")
            return

        try:
            t = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f"cannot get transform {source_frame} to {target_frame}: {ex}")
            return

        location = ros_point_to_carla_location(t.transform.translation)
        rotation = ros_quaternion_to_carla_rotation(t.transform.rotation)
        self.ego_vehicle.set_transform(carla.Transform(location, rotation))


    # def listener_callback(self, msg):
    #    self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    carla_psim_bridge = CarlaPsimBridge(world)


    rclpy.spin(carla_psim_bridge)

    carla_psim_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

