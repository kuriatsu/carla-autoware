import rclpy
from rclpy.node import Node
from derived_object_msgs.msg import Object, ObjectArray
import carla
import random

import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)




class CarlaOtherVehiclePublisher(Node):
    def __init__(self):
        super().__init__('carla_other_vehicle_publisher')
        self.publisher_ = self.create_publisher(ObjectArray, '/carla/ego_vehicle/objects', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        blueprint_library = self.world.get_blueprint_library()
        car_bp = blueprint_library.filter('vehicle.*')[0]
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)
        self.other_vehicle = self.world.spawn_actor(car_bp, spawn_point)
        self.other_vehicle.set_autopilot(True)

    def timer_callback(self):
        object_array = ObjectArray()
        object_array.header.frame_id = "map"
        object_array.header.stamp = self.get_clock().now().to_msg()

        carla_location = self.other_vehicle.get_location()
        carla_rotation = self.other_vehicle.get_transform().rotation

        detected_object = Object()
        detected_object.id = self.other_vehicle.id
        detected_object.header.frame_id = "map"
        detected_object.pose.position.x = 120.0#carla_location.x
        detected_object.pose.position.y = -55.0#-carla_location.y
        detected_object.pose.position.z = 0.0#carla_location.z
        detected_object.pose.orientation = euler_to_quaternion(math.radians(carla_rotation.roll), -math.radians(carla_rotation.pitch), math.radians(carla_rotation.yaw))
        detected_object.shape.dimensions = [4.0,2.0,2.0] 
        
        object_array.objects.append(detected_object)
        print(object_array)
        self.publisher_.publish(object_array)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaOtherVehiclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


