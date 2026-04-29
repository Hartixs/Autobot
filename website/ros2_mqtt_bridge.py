# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Imu
# # import paho.mqtt.client as mqtt
# # import json

# # class ImuBridge(Node):
# #     def __init__(self):
# #         super().__init__('imu_mqtt_bridge')
# #         # Listen to ROS2
# #         self.subscription = self.create_subscription(
# #             Imu,
# #             '/imu/data',
# #             self.imu_callback,
# #             10)
        
# #         # Connect to MQTT (Mosquitto)
# #         self.mqtt_client = mqtt.Client()
# #         self.mqtt_client.connect("127.0.0.1", 1883, 60)
# #         self.get_logger().info("Bridge Active: Translating ROS2 /imu/data -> MQTT")

# #     def imu_callback(self, msg):
# #         # Translate ROS2 msg to JSON dictionary
# #         payload = {
# #             "orientation": {
# #                 "x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w
# #             },
# #             "angular_velocity": {
# #                 "x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z
# #             },
# #             "linear_acceleration": {
# #                 "x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z
# #             }
# #         }
# #         # Publish to MQTT so the dashboard can hear it
# #         self.mqtt_client.publish("/imu/data", json.dumps(payload))

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = ImuBridge()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry  # <-- Added import for Odometry
# import paho.mqtt.client as mqtt
# import json

# class ImuOdomBridge(Node):
#     def __init__(self):
#         super().__init__('imu_odom_mqtt_bridge')
        
#         # Connect to MQTT (Mosquitto)
#         self.mqtt_client = mqtt.Client()
#         self.mqtt_client.connect("127.0.0.1", 1883, 60)
        
#         # Listen to ROS2 IMU Data
#         self.imu_subscription = self.create_subscription(
#             Imu,
#             '/imu/data',
#             self.imu_callback,
#             10)
            
#         # Listen to ROS2 Odometry Data
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10)
            
#         self.get_logger().info("Bridge Active: Translating ROS2 /imu/data and /odom -> MQTT")

#     def imu_callback(self, msg):
#         # Translate ROS2 msg to JSON dictionary
#         payload = {
#             "orientation": {
#                 "x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w
#             },
#             "angular_velocity": {
#                 "x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z
#             },
#             "linear_acceleration": {
#                 "x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z
#             }
#         }
#         # Publish to MQTT
#         self.mqtt_client.publish("/imu/data", json.dumps(payload))

#     def odom_callback(self, msg):
#         # Translate ROS2 Odometry msg to JSON dictionary
#         payload = {
#             "pose": {
#                 "position": {
#                     "x": msg.pose.pose.position.x,
#                     "y": msg.pose.pose.position.y,
#                     "z": msg.pose.pose.position.z
#                 },
#                 "orientation": {
#                     "x": msg.pose.pose.orientation.x,
#                     "y": msg.pose.pose.orientation.y,
#                     "z": msg.pose.pose.orientation.z,
#                     "w": msg.pose.pose.orientation.w
#                 }
#             },
#             "twist": {
#                 "linear": {
#                     "x": msg.twist.twist.linear.x,
#                     "y": msg.twist.twist.linear.y,
#                     "z": msg.twist.twist.linear.z
#                 },
#                 "angular": {
#                     "x": msg.twist.twist.angular.x,
#                     "y": msg.twist.twist.angular.y,
#                     "z": msg.twist.twist.angular.z
#                 }
#             }
#         }
#         # Publish to MQTT under the /odom topic
#         self.mqtt_client.publish("/odom", json.dumps(payload))

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuOdomBridge()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json

class MainBridge(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')
        
        # Connect to MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("127.0.0.1", 1883, 60)
        
        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            
        self.get_logger().info("Bridge Active: Translating /imu/data and /odom -> MQTT")

    def imu_callback(self, msg):
        payload = {
            "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w},
            "angular_velocity": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
            "linear_acceleration": {"x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z}
        }
        self.mqtt_client.publish("/imu/data", json.dumps(payload))

    def odom_callback(self, msg):
        payload = {
            "pose": {
                "position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                "orientation": {"x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
            },
            "twist": {
                "linear": {"x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y, "z": msg.twist.twist.linear.z},
                "angular": {"x": msg.twist.twist.angular.x, "y": msg.twist.twist.angular.y, "z": msg.twist.twist.angular.z}
            }
        }
        self.mqtt_client.publish("/odom", json.dumps(payload))

def main(args=None):
    rclpy.init(args=args)
    node = MainBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()