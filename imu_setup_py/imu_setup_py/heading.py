# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
# import tf_transformations

# class HeadingLockNode(Node):
#     def __init__(self):
#         super().__init__('heading_lock_node')
       
#         # Subscribers
#         self.cmd_sub = self.create_subscription(Twist, 'cmd_vel_in', self.cmd_callback, 10)
#         self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
       
#         # Publisher (To your motor driver)
#         self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

#         # PID & State Variables
#         self.target_yaw = 0.0
#         self.current_yaw = 0.0
#         self.is_locked = False
       
#         # Proportional gain for the P-Controller - Tune this based on your bot's behavior!
#         self.kp = 1.5  
       
#         self.get_logger().info("Heading Lock Node Started. Routing cmd_vel_in -> cmd_vel_out")

#     def imu_callback(self, msg):
#         # Extract Yaw from the IMU quaternion
#         q = [
#             msg.orientation.x,
#             msg.orientation.y,
#             msg.orientation.z,
#             msg.orientation.w
#         ]
#         _, _, self.current_yaw = tf_transformations.euler_from_quaternion(q)

#     def cmd_callback(self, msg):
#         out_msg = Twist()
#         out_msg.linear.x = msg.linear.x
       
#         # 1. Check if Nav2/Teleop is commanding a turn
#         if abs(msg.angular.z) > 0.05:
#             # We are turning. Disable the lock and pass the turn command directly.
#             self.is_locked = False
#             out_msg.angular.z = msg.angular.z
#         else:
#             # 2. We are moving straight (or stopped). Engage heading lock.
#             if not self.is_locked:
#                 # We just finished a turn. Snap the target to our current heading.
#                 self.target_yaw = self.current_yaw
#                 self.is_locked = True
           
#             # Calculate the error between where we want to point and where we are pointing
#             error = self.target_yaw - self.current_yaw
           
#             # Normalize the angle error to be between -Pi and Pi
#             if error > 3.14159:
#                 error -= 2 * 3.14159
#             if error < -3.14159:
#                 error += 2 * 3.14159
           
#             # 3. Apply correction ONLY if the robot is actually trying to move forward
#             if abs(msg.linear.x) > 0.01:
#                 out_msg.angular.z = error * self.kp
#             else:
#                 out_msg.angular.z = 0.0

#         self.cmd_pub.publish(out_msg)

# def main():
#     rclpy.init()
#     node = HeadingLockNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()