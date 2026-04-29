# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TransformStamped
# from nav_msgs.msg import Odometry
# from tf2_ros import TransformBroadcaster
# from sensor_msgs.msg import JointState
# import serial
# import threading
# import math
# import transforms3d

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.043   
#         self.track_width = 0.15     
#         self.ticks_per_rev = 690.0  
        
#         # NEW: Separate power scaling!
#         self.linear_pwm_scale = 150.0   # Power for going straight
#         self.angular_pwm_scale = 150.0  # Power for turning 
        
#         # ADDED: Minimum PWM to overcome motor stall (Deadband compensation)
#         self.min_pwm = 100.0 
#         # ---------------------------------

#         # Serial setup
#         try:
#             self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
        
#         # Publishers and Broadcasters
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         # State Variables
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.first_reading = True
        
#         # Odometry Timer (20Hz)
#         self.last_time = self.get_clock().now()
#         self.odom_timer = self.create_timer(0.05, self.publish_odom)
        
#         self.current_left_ticks = 0
#         self.current_right_ticks = 0

#         # Start Serial Reader Thread
#         self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.read_thread.start()

#     def cmd_vel_callback(self, msg):
#         linear_x = msg.linear.x
#         angular_z = msg.angular.z

#         # Apply independent power scales for forward vs turning
#         forward_pwm = linear_x * self.linear_pwm_scale
#         turn_pwm = angular_z * self.angular_pwm_scale

#         # Combine them for the raw motor commands
#         raw_left_pwm = forward_pwm + turn_pwm
#         raw_right_pwm = forward_pwm - turn_pwm

#         # ADDED: Helper function to apply the 90 PWM deadband
#         def apply_deadband(pwm):
#             # If the command is positive and large enough to not be noise
#             if pwm > 0.5: 
#                 clamped_pwm = min(pwm, 255.0)
#                 # Map from (0 to 255) -> (min_pwm to 255)
#                 return int(self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
            
#             # If the command is negative
#             elif pwm < -0.5:
#                 clamped_pwm = max(pwm, -255.0)
#                 # Map from (0 to -255) -> (-min_pwm to -255)
#                 return int(-self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
            
#             # If the command is exactly 0 (or pure noise), stop the motors
#             else:
#                 return 0

#         # Pass the raw values through the deadband filter
#         left_pwm = apply_deadband(raw_left_pwm)
#         right_pwm = apply_deadband(raw_right_pwm)

#         command = f"M,{left_pwm},{right_pwm}\n"
#         self.serial_port.write(command.encode('utf-8'))

#     def read_serial_data(self):
#         while rclpy.ok():
#             if self.serial_port.in_waiting > 0:
#                 try:
#                     line = self.serial_port.readline().decode('utf-8').strip()
#                     if line.startswith("ENC,"):
#                         parts = line.split(',')
#                         if len(parts) == 3:
#                             self.current_left_ticks = int(parts[1])
#                             self.current_right_ticks = int(parts[2])
#                 except:
#                     pass

#     def publish_odom(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         if dt <= 0 or self.first_reading:
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             self.first_reading = False
#             return

#         # 1. Ticks to Meters
#         delta_l = self.current_left_ticks - self.last_left_ticks
#         delta_r = self.current_right_ticks - self.last_right_ticks
        
#         self.last_left_ticks = self.current_left_ticks
#         self.last_right_ticks = self.current_right_ticks

#         meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
#         dist_l = delta_l * meters_per_tick
#         dist_r = delta_r * meters_per_tick

#         # 2. Kinematics
#         dist_c = (dist_r + dist_l) / 2.0
#         delta_th = (dist_r - dist_l) / self.track_width

#         self.x += dist_c * math.cos(self.theta + (delta_th / 2.0))
#         self.y += dist_c * math.sin(self.theta + (delta_th / 2.0))
#         self.theta += delta_th

#         # 3. Create Quaternion
#         q = transforms3d.euler.euler2quat(0, 0, self.theta) 

#         # 4. Broadcast Transform: odom -> base_footprint
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint' 
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         t.transform.rotation.w = q[0]
#         t.transform.rotation.x = q[1]
#         t.transform.rotation.y = q[2]
#         t.transform.rotation.z = q[3]
#         self.tf_broadcaster.sendTransform(t)

#         # 5. Publish Odom Message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.w = q[0]
#         odom.pose.pose.orientation.x = q[1]
#         odom.pose.pose.orientation.y = q[2]
#         odom.pose.pose.orientation.z = q[3]
#         odom.twist.twist.linear.x = dist_c / dt
#         odom.twist.twist.angular.z = delta_th / dt
#         self.odom_pub.publish(odom)

#         # 6. Publish Joint States (Wheel rotation)
#         self.left_wheel_angle += (dist_l / self.wheel_radius)
#         self.right_wheel_angle += (dist_r / self.wheel_radius)
        
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)

#         self.last_time = current_time

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.serial_port.write(b"M,0,0\n")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState
# import serial
# import threading
# import math
# import transforms3d

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.043   
#         self.track_width = 0.15     
#         self.ticks_per_rev = 690.0  
        
#         self.linear_pwm_scale = 150.0   
#         self.angular_pwm_scale = 150.0  
#         self.min_pwm = 100.0 
#         # ---------------------------------

#         # Serial setup
#         try:
#             self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
        
#         # Publishers and Broadcasters
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
#         # CHANGED: Publish to odom_unfiltered instead of odom
#         self.odom_pub = self.create_publisher(Odometry, 'odom_unfiltered', 10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
#         # REMOVED: self.tf_broadcaster (The EKF will handle this now!)
        
#         # State Variables
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.first_reading = True
        
#         # Odometry Timer (20Hz)
#         self.last_time = self.get_clock().now()
#         self.odom_timer = self.create_timer(0.05, self.publish_odom)
        
#         self.current_left_ticks = 0
#         self.current_right_ticks = 0

#         self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.read_thread.start()

#     def cmd_vel_callback(self, msg):
#         linear_x = msg.linear.x
#         angular_z = msg.angular.z

#         forward_pwm = linear_x * self.linear_pwm_scale
#         turn_pwm = angular_z * self.angular_pwm_scale

#         raw_left_pwm = forward_pwm + turn_pwm
#         raw_right_pwm = forward_pwm - turn_pwm

#         def apply_deadband(pwm):
#             if pwm > 0.5: 
#                 clamped_pwm = min(pwm, 255.0)
#                 return int(self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             elif pwm < -0.5:
#                 clamped_pwm = max(pwm, -255.0)
#                 return int(-self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             else:
#                 return 0

#         left_pwm = apply_deadband(raw_left_pwm)
#         right_pwm = apply_deadband(raw_right_pwm)

#         command = f"M,{left_pwm},{right_pwm}\n"
#         self.serial_port.write(command.encode('utf-8'))

#     def read_serial_data(self):
#         while rclpy.ok():
#             if self.serial_port.in_waiting > 0:
#                 try:
#                     line = self.serial_port.readline().decode('utf-8').strip()
#                     if line.startswith("ENC,"):
#                         parts = line.split(',')
#                         if len(parts) == 3:
#                             self.current_left_ticks = int(parts[1])
#                             self.current_right_ticks = int(parts[2])
#                 except:
#                     pass

#     def publish_odom(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         if dt <= 0 or self.first_reading:
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             self.first_reading = False
#             return

#         delta_l = self.current_left_ticks - self.last_left_ticks
#         delta_r = self.current_right_ticks - self.last_right_ticks
        
#         self.last_left_ticks = self.current_left_ticks
#         self.last_right_ticks = self.current_right_ticks

#         meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
#         dist_l = delta_l * meters_per_tick
#         dist_r = delta_r * meters_per_tick

#         dist_c = (dist_r + dist_l) / 2.0
#         delta_th = (dist_r - dist_l) / self.track_width

#         self.x += dist_c * math.cos(self.theta + (delta_th / 2.0))
#         self.y += dist_c * math.sin(self.theta + (delta_th / 2.0))
#         self.theta += delta_th

#         q = transforms3d.euler.euler2quat(0, 0, self.theta) 

#         # REMOVED: TransformStamped broadcast code block.

#         # 5. Publish Odom Message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link' # CHANGED: to base_link to match your URDF
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.w = q[0]
#         odom.pose.pose.orientation.x = q[1]
#         odom.pose.pose.orientation.y = q[2]
#         odom.pose.pose.orientation.z = q[3]
#         odom.twist.twist.linear.x = dist_c / dt
#         odom.twist.twist.angular.z = delta_th / dt
        
#         # Will now publish to /odom_unfiltered based on __init__ definition
#         self.odom_pub.publish(odom) 

#         # 6. Publish Joint States (Wheel rotation)
#         self.left_wheel_angle += (dist_l / self.wheel_radius)
#         self.right_wheel_angle += (dist_r / self.wheel_radius)
        
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)

#         self.last_time = current_time

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.serial_port.write(b"M,0,0\n")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState
# import serial
# import threading
# import math
# import transforms3d

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.043   
#         self.track_width = 0.15     
#         self.ticks_per_rev = 690.0  
        
#         self.linear_pwm_scale = 150.0   
#         self.angular_pwm_scale = 150.0  
#         self.min_pwm = 100.0 
#         # ---------------------------------

#         # Serial setup
#         try:
#             self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
        
#         # Publishers and Broadcasters
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, 'odom_unfiltered', 10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
#         # State Variables
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.first_reading = True
        
#         self.current_left_ticks = 0
#         self.current_right_ticks = 0

#         # --- WATCHDOG TIMER VARIABLES ---
#         self.last_cmd_vel_time = self.get_clock().now()
        
#         # Odometry & Watchdog Timer (20Hz)
#         self.last_time = self.get_clock().now()
#         self.odom_timer = self.create_timer(0.05, self.timer_callback)

#         self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.read_thread.start()

#     def cmd_vel_callback(self, msg):
#         # Update the watchdog timer every time we receive a command
#         self.last_cmd_vel_time = self.get_clock().now()

#         # --- INVERTED CONTROLS ---
#         linear_x = -msg.linear.x    # Negated: Forward is now backward
#         angular_z = msg.angular.z  # Negated: Left is now right

#         forward_pwm = linear_x * self.linear_pwm_scale
#         turn_pwm = angular_z * self.angular_pwm_scale

#         raw_left_pwm = forward_pwm + turn_pwm
#         raw_right_pwm = forward_pwm - turn_pwm

#         def apply_deadband(pwm):
#             if pwm > 0.5: 
#                 clamped_pwm = min(pwm, 255.0)
#                 return int(self.min_pwm - (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             elif pwm < -0.5:
#                 clamped_pwm = max(pwm, -255.0)
#                 return int(-self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             else:
#                 return 0

#         left_pwm = apply_deadband(raw_left_pwm)
#         right_pwm = apply_deadband(raw_right_pwm)

#         command = f"M,{left_pwm},{right_pwm}\n"
#         self.serial_port.write(command.encode('utf-8'))

#     def read_serial_data(self):
#         while rclpy.ok():
#             if self.serial_port.in_waiting > 0:
#                 try:
#                     line = self.serial_port.readline().decode('utf-8').strip()
#                     if line.startswith("ENC,"):
#                         parts = line.split(',')
#                         if len(parts) == 3:
#                             self.current_left_ticks = int(parts[1])
#                             self.current_right_ticks = int(parts[2])
#                 except:
#                     pass

#     def timer_callback(self):
#         current_time = self.get_clock().now()
        
#         # --- WATCHDOG CHECK ---
#         # If no cmd_vel received in the last 0.5 seconds, force stop!
#         time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
#         if time_since_last_cmd > 0.5:
#             self.serial_port.write(b"M,0,0\n")

#         # --- ODOMETRY CALCULATION ---
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         if dt <= 0 or self.first_reading:
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             self.first_reading = False
#             return

#         delta_l = self.current_left_ticks - self.last_left_ticks
#         delta_r = self.current_right_ticks - self.last_right_ticks

#         if abs(delta_l) > 1000 or abs(delta_r) > 1000:
#             self.get_logger().warn("⚠️ ENCODER JUMP DETECTED! Arduino likely rebooted or wrapped around. Preventing teleportation.")
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             return
        
#         self.last_left_ticks = self.current_left_ticks
#         self.last_right_ticks = self.current_right_ticks

#         meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
#         dist_l = delta_l * meters_per_tick
#         dist_r = delta_r * meters_per_tick

#         dist_c = (dist_r + dist_l) / 2.0
#         delta_th = (dist_r - dist_l) / self.track_width

#         self.x += dist_c * math.cos(self.theta + (delta_th / 2.0))
#         self.y += dist_c * math.sin(self.theta + (delta_th / 2.0))
#         self.theta += delta_th

#         q = transforms3d.euler.euler2quat(0, 0, self.theta) 

#         # Publish Odom Message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint' # CRITICAL for SLAM
        
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.w = q[0]
#         odom.pose.pose.orientation.x = q[1]
#         odom.pose.pose.orientation.y = q[2]
#         odom.pose.pose.orientation.z = q[3]
#         odom.twist.twist.linear.x = dist_c / dt
#         odom.twist.twist.angular.z = delta_th / dt

#         # CRITICAL: Covariance so the EKF trusts the data
#         odom.pose.covariance[0] = 0.01   
#         odom.pose.covariance[7] = 0.01   
#         odom.pose.covariance[35] = 0.05  
#         odom.twist.covariance[0] = 0.01  
#         odom.twist.covariance[35] = 0.05 

#         self.odom_pub.publish(odom) 

#         # Publish Joint States
#         self.left_wheel_angle += (dist_l / self.wheel_radius)
#         self.right_wheel_angle += (dist_r / self.wheel_radius)
        
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)

#         self.last_time = current_time

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.serial_port.write(b"M,0,0\n")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState
# import serial
# import threading
# import math

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.043   
#         self.track_width = 0.15     
#         self.ticks_per_rev = 690.0  
        
#         self.linear_pwm_scale = 150.0   
#         self.angular_pwm_scale = 150.0  
#         self.min_pwm = 100.0 
#         # ---------------------------------

#         # Serial setup
#         try:
#             self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
        
#         # Publishers and Broadcasters
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_corrected', self.cmd_vel_callback, 10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
#         # State Variables
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.first_reading = True
        
#         self.current_left_ticks = 0
#         self.current_right_ticks = 0

#         # --- WATCHDOG TIMER VARIABLES ---
#         self.last_cmd_vel_time = self.get_clock().now()
        
#         # Joint State & Watchdog Timer (20Hz)
#         self.odom_timer = self.create_timer(0.05, self.timer_callback)

#         self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.read_thread.start()

#     def cmd_vel_callback(self, msg):
#         # Update the watchdog timer every time we receive a command
#         self.last_cmd_vel_time = self.get_clock().now()

#         # --- INVERTED CONTROLS ---
#         linear_x = -msg.linear.x    # Negated: Forward is now backward
#         angular_z = msg.angular.z   # Negated: Left is now right

#         forward_pwm = linear_x * self.linear_pwm_scale
#         turn_pwm = angular_z * self.angular_pwm_scale

#         raw_left_pwm = forward_pwm + turn_pwm
#         raw_right_pwm = forward_pwm - turn_pwm

#         def apply_deadband(pwm):
#             if pwm > 0.5: 
#                 clamped_pwm = min(pwm, 255.0)
#                 # FIX: Changed the minus sign to a plus sign here for proper power scaling
#                 return int(self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             elif pwm < -0.5:
#                 clamped_pwm = max(pwm, -255.0)
#                 return int(-self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             else:
#                 return 0

#         left_pwm = apply_deadband(raw_left_pwm)
#         right_pwm = apply_deadband(raw_right_pwm)

#         command = f"M,{left_pwm},{right_pwm}\n"
#         self.serial_port.write(command.encode('utf-8'))

#     def read_serial_data(self):
#         while rclpy.ok():
#             if self.serial_port.in_waiting > 0:
#                 try:
#                     line = self.serial_port.readline().decode('utf-8').strip()
#                     if line.startswith("ENC,"):
#                         parts = line.split(',')
#                         if len(parts) == 3:
#                             self.current_left_ticks = int(parts[1])
#                             self.current_right_ticks = int(parts[2])
#                 except:
#                     pass

#     def timer_callback(self):
#         current_time = self.get_clock().now()
        
#         # --- WATCHDOG CHECK ---
#         # If no cmd_vel received in the last 0.5 seconds, force stop!
#         time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
#         if time_since_last_cmd > 0.5:
#             self.serial_port.write(b"M,0,0\n")

#         # --- JOINT STATE CALCULATION ---
#         if self.first_reading:
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.first_reading = False
#             return

#         delta_l = self.current_left_ticks - self.last_left_ticks
#         delta_r = self.current_right_ticks - self.last_right_ticks

#         if abs(delta_l) > 1000 or abs(delta_r) > 1000:
#             self.get_logger().warn("⚠️ ENCODER JUMP DETECTED! Arduino likely rebooted or wrapped around.")
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             return
        
#         self.last_left_ticks = self.current_left_ticks
#         self.last_right_ticks = self.current_right_ticks

#         meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
#         dist_l = delta_l * meters_per_tick
#         dist_r = delta_r * meters_per_tick

#         # Publish Joint States
#         self.left_wheel_angle += (dist_l / self.wheel_radius)
#         self.right_wheel_angle += (dist_r / self.wheel_radius)
        
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)


# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.serial_port.write(b"M,0,0\n")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# import serial
# import threading
# import math

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.043   
#         self.track_width = 0.15     
#         self.ticks_per_rev = 690.0  
        
#         self.linear_pwm_scale = 150.0   
#         self.angular_pwm_scale = 150.0  
#         self.min_pwm = 100.0 
#         # ---------------------------------

#         # Serial setup
#         try:
#             self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
        
#         # Publishers
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_corrected', self.cmd_vel_callback, 10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
#         # --- MISSING ODOMETRY PUBLISHER ADDED BACK ---
#         self.odom_pub = self.create_publisher(Odometry, '/odom_unfiltered', 10)
        
#         # State Variables
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.first_reading = True
        
#         self.current_left_ticks = 0
#         self.current_right_ticks = 0

#         # --- ODOMETRY STATE VARIABLES ---
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.last_time = self.get_clock().now()

#         # --- WATCHDOG TIMER VARIABLES ---
#         self.last_cmd_vel_time = self.get_clock().now()
        
#         # Joint State & Watchdog Timer (20Hz)
#         self.odom_timer = self.create_timer(0.05, self.timer_callback)

#         self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
#         self.read_thread.start()

#     def cmd_vel_callback(self, msg):
#         # Update the watchdog timer every time we receive a command
#         self.last_cmd_vel_time = self.get_clock().now()

#         # --- INVERTED CONTROLS ---
#         linear_x = -msg.linear.x    # Negated: Forward is now backward
#         angular_z = msg.angular.z   # Negated: Left is now right

#         forward_pwm = linear_x * self.linear_pwm_scale
#         turn_pwm = angular_z * self.angular_pwm_scale

#         raw_left_pwm = forward_pwm + turn_pwm
#         raw_right_pwm = forward_pwm - turn_pwm

#         def apply_deadband(pwm):
#             if pwm > 0.5: 
#                 clamped_pwm = min(pwm, 255.0)
#                 return int(self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             elif pwm < -0.5:
#                 clamped_pwm = max(pwm, -255.0)
#                 return int(-self.min_pwm + (clamped_pwm / 255.0) * (255.0 - self.min_pwm))
#             else:
#                 return 0

#         left_pwm = apply_deadband(raw_left_pwm)
#         right_pwm = apply_deadband(raw_right_pwm)

#         command = f"M,{left_pwm},{right_pwm}\n"
#         self.serial_port.write(command.encode('utf-8'))

#     def read_serial_data(self):
#         while rclpy.ok():
#             if self.serial_port.in_waiting > 0:
#                 try:
#                     line = self.serial_port.readline().decode('utf-8').strip()
#                     if line.startswith("ENC,"):
#                         parts = line.split(',')
#                         if len(parts) == 3:
#                             self.current_left_ticks = int(parts[1])
#                             self.current_right_ticks = int(parts[2])
#                 except:
#                     pass

#     def timer_callback(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         # --- WATCHDOG CHECK ---
#         time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
#         if time_since_last_cmd > 0.5:
#             self.serial_port.write(b"M,0,0\n")

#         if self.first_reading:
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             self.first_reading = False
#             return

#         delta_l = self.current_left_ticks - self.last_left_ticks
#         delta_r = self.current_right_ticks - self.last_right_ticks

#         if abs(delta_l) > 1000 or abs(delta_r) > 1000:
#             self.get_logger().warn("⚠️ ENCODER JUMP DETECTED! Arduino likely rebooted or wrapped around.")
#             self.last_left_ticks = self.current_left_ticks
#             self.last_right_ticks = self.current_right_ticks
#             self.last_time = current_time
#             return
        
#         self.last_left_ticks = self.current_left_ticks
#         self.last_right_ticks = self.current_right_ticks

#         # --- DISTANCE MATH ---
#         meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
#         dist_l = delta_l * meters_per_tick
#         dist_r = delta_r * meters_per_tick
        
#         dist_c = (dist_l + dist_r) / 2.0
#         delta_th = (dist_r - dist_l) / self.track_width

#         # --- KINEMATIC INTEGRATION ---
#         if dt > 0:
#             v_x = dist_c / dt
#             v_th = delta_th / dt
#         else:
#             v_x = 0.0
#             v_th = 0.0

#         self.x += dist_c * math.cos(self.th + delta_th / 2.0)
#         self.y += dist_c * math.sin(self.th + delta_th / 2.0)
#         self.th += delta_th
#         self.last_time = current_time

#         # --- PUBLISH ODOMETRY ---
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
        
#         # Euler to Quaternion manually for Z-axis rotation
#         odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
#         odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

#         odom.twist.twist.linear.x = v_x
#         odom.twist.twist.angular.z = v_th

#         # Basic covariances so ESKF trusts the data
#         odom.pose.covariance[0] = 0.01
#         odom.pose.covariance[7] = 0.01
#         odom.pose.covariance[35] = 0.05
#         odom.twist.covariance[0] = 0.01
#         odom.twist.covariance[35] = 0.05

#         self.odom_pub.publish(odom)

#         # --- PUBLISH JOINT STATES ---
#         self.left_wheel_angle += (dist_l / self.wheel_radius)
#         self.right_wheel_angle += (dist_r / self.wheel_radius)
        
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)


# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.serial_port.write(b"M,0,0\n")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState
# import serial
# import math

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.0215  # 43mm diameter / 2
#         self.track_width = 0.15     # Distance between wheels in meters
#         self.ticks_per_rev = 670.0  
#         self.dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
#         # --- MOTOR TUNING PARAMETERS ---
#         self.max_pwm = 255.0
#         # The theoretical max speed of a 60RPM motor with 43mm wheels
#         self.max_speed_m_s = (60.0 / 60.0) * (2.0 * math.pi * self.wheel_radius) 
#         self.min_pwm = 80.0  # Deadband: min PWM needed to make the motor physically move
        
#         # --- ODOMETRY STATE VARIABLES ---
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.v_x = 0.0
#         self.v_th = 0.0
        
#         self.last_ticks_l = 0
#         self.last_ticks_r = 0
#         self.first_read = True
        
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
        
#         self.last_time = self.get_clock().now()

#         # --- HARDWARE INTERFACE ---
#         self.serial_port_name = '/dev/ttyUSB0' 
#         try:
#             self.arduino = serial.Serial(self.serial_port_name, 115200, timeout=0.1)
#             self.get_logger().info(f"Connected to Arduino on {self.serial_port_name}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             raise e

#         # --- ROS 2 INTERFACES ---
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
#         # Timer for reading Arduino data and publishing odometry (20Hz)
#         self.timer = self.create_timer(0.05, self.read_and_publish)

#     def speed_to_pwm(self, target_speed):
#         """Converts target velocity (m/s) to a PWM value with deadband compensation"""
#         if abs(target_speed) < 0.001:
#             return 0
        
#         # Proportional mapping based on max physical capability
#         pwm = (abs(target_speed) / self.max_speed_m_s) * (self.max_pwm - self.min_pwm)
#         pwm += self.min_pwm
        
#         # Cap to max PWM
#         pwm = min(max(pwm, 0), self.max_pwm)
        
#         # Re-apply sign
#         return int(pwm) if target_speed > 0 else int(-pwm)

#     def cmd_vel_callback(self, msg):
#         v = msg.linear.x
#         omega = msg.angular.z

#         # Inverse Kinematics: Calculate required wheel speeds (m/s)
#         v_l = v - (omega * self.track_width / 2.0)
#         v_r = v + (omega * self.track_width / 2.0)

#         # Convert to PWM
#         pwm_l = self.speed_to_pwm(v_l)
#         pwm_r = self.speed_to_pwm(v_r)

#         # Format and send command to Arduino: "M,LeftPWM,RightPWM\n"
#         cmd_str = f"M,{pwm_l},{pwm_r}\n"
#         self.arduino.write(cmd_str.encode('utf-8'))

#     def read_and_publish(self):
#         # 1. Read ALL available data in the serial buffer
#         while self.arduino.in_waiting:
#             try:
#                 line = self.arduino.readline().decode('utf-8').strip()
#                 if line.startswith("E,"):
#                     parts = line.split(',')
#                     if len(parts) == 3:
#                         try:
#                             current_ticks_l = int(parts[1])
#                             current_ticks_r = int(parts[2])
#                             self.calculate_odometry(current_ticks_l, current_ticks_r)
#                         except ValueError:
#                             pass # Ignore if int conversion fails
#             except UnicodeDecodeError:
#                 pass # Ignore garbled serial data

#         # 2. ALWAYS publish Odom and Joint States, even if no new data arrived
#         self.publish_data()

#     def calculate_odometry(self, ticks_l, ticks_r):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         if self.first_read:
#             self.last_ticks_l = ticks_l
#             self.last_ticks_r = ticks_r
#             self.first_read = False
#             self.last_time = current_time
#             return

#         delta_ticks_l = ticks_l - self.last_ticks_l
#         delta_ticks_r = ticks_r - self.last_ticks_r

#         self.last_ticks_l = ticks_l
#         self.last_ticks_r = ticks_r
#         self.last_time = current_time

#         # Calculate distances traveled by each wheel
#         d_l = delta_ticks_l * self.dist_per_tick
#         d_r = delta_ticks_r * self.dist_per_tick

#         # Forward Kinematics: Center distance and heading change
#         d_c = (d_l + d_r) / 2.0
#         d_th = (d_r - d_l) / self.track_width

#         # Calculate velocities
#         self.v_x = d_c / dt if dt > 0 else 0.0
#         self.v_th = d_th / dt if dt > 0 else 0.0

#         # Update pose estimation
#         self.x += d_c * math.cos(self.th + (d_th / 2.0))
#         self.y += d_c * math.sin(self.th + (d_th / 2.0))
#         self.th += d_th
        
#         # Update joint states
#         self.left_wheel_angle += (d_l / self.wheel_radius)
#         self.right_wheel_angle += (d_r / self.wheel_radius)

#     def publish_data(self):
#         current_time = self.get_clock().now()

#         # --- PUBLISH ODOMETRY ---
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
        
#         # Simple Euler to Quaternion for Z-axis rotation
#         odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
#         odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

#         odom.twist.twist.linear.x = self.v_x
#         odom.twist.twist.angular.z = self.v_th

#         # Basic covariances
#         odom.pose.covariance[0] = 0.01   
#         odom.pose.covariance[7] = 0.01   
#         odom.pose.covariance[35] = 0.05  
#         odom.twist.covariance[0] = 0.01  
#         odom.twist.covariance[35] = 0.05 

#         self.odom_pub.publish(odom)

#         # --- PUBLISH JOINT STATES (Needed for URDF / TF Tree) ---
#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down Base Controller...")
#         # Send zero velocity command on shutdown
#         node.arduino.write("M,0,0\n".encode('utf-8'))
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState
# import serial
# import math

# class BaseController(Node):
#     def __init__(self):
#         super().__init__('base_controller')
        
#         # ==========================================
#         # --- HARDWARE CALIBRATION & TWEAKS ---
#         # ==========================================
#         # 1. MOTORS: If the robot moves backward when commanded forward, change 1 to -1
#         self.motor_dir_l = -1
#         self.motor_dir_r = -1

#         # 2. MOTORS: If the robot turns right when commanded left, set to True
#         self.swap_motors = False

#         # 3. ENCODERS: If ODOM moves backward when you manually push it forward, change 1 to -1
#         self.encoder_dir_l = 1
#         self.encoder_dir_r = 1

#         # 4. ENCODERS: If ODOM turns right when you manually spin the robot left, set to True
#         self.swap_encoders = False
#         # ==========================================

#         # --- PHYSICAL ROBOT PARAMETERS ---
#         self.wheel_radius = 0.0215  # 43mm diameter / 2
#         self.track_width = 0.15     # Distance between wheels in meters
#         self.ticks_per_rev = 670.0  
#         self.dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
#         # --- MOTOR TUNING PARAMETERS ---
#         self.max_pwm = 255.0
#         self.max_speed_m_s = (60.0 / 60.0) * (2.0 * math.pi * self.wheel_radius) 
#         self.min_pwm = 80.0  
        
#         # --- ODOMETRY STATE VARIABLES ---
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.v_x = 0.0
#         self.v_th = 0.0
        
#         self.last_ticks_l = 0
#         self.last_ticks_r = 0
#         self.first_read = True
        
#         self.left_wheel_angle = 0.0
#         self.right_wheel_angle = 0.0
        
#         self.last_time = self.get_clock().now()

#         # --- HARDWARE INTERFACE ---
#         self.serial_port_name = '/dev/arduino' 
#         try:
#             self.arduino = serial.Serial(self.serial_port_name, 115200, timeout=0.1)
#             self.get_logger().info(f"Connected to Arduino on {self.serial_port_name}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             raise e

#         # --- ROS 2 INTERFACES ---
#         self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
#         self.timer = self.create_timer(0.05, self.read_and_publish)

#     def speed_to_pwm(self, target_speed):
#         if abs(target_speed) < 0.001:
#             return 0
        
#         pwm = (abs(target_speed) / self.max_speed_m_s) * (self.max_pwm - self.min_pwm)
#         pwm += self.min_pwm
#         pwm = min(max(pwm, 0), self.max_pwm)
        
#         return int(pwm) if target_speed > 0 else int(-pwm)

#     def cmd_vel_callback(self, msg):
#         v = msg.linear.x
#         omega = msg.angular.z

#         # Inverse Kinematics
#         v_l = v - (omega * self.track_width / 2.0)
#         v_r = v + (omega * self.track_width / 2.0)

#         # Handle physical motor swaps
#         if self.swap_motors:
#             v_l, v_r = v_r, v_l

#         pwm_l = self.speed_to_pwm(v_l) * self.motor_dir_l
#         pwm_r = self.speed_to_pwm(v_r) * self.motor_dir_r

#         cmd_str = f"M,{pwm_l},{pwm_r}\n"
#         self.arduino.write(cmd_str.encode('utf-8'))

#     def read_and_publish(self):
#         while self.arduino.in_waiting:
#             try:
#                 line = self.arduino.readline().decode('utf-8').strip()
#                 if line.startswith("ENC,"):
#                     parts = line.split(',')
#                     if len(parts) == 3:
#                         try:
#                             raw_ticks_l = int(parts[1])
#                             raw_ticks_r = int(parts[2])
                            
#                             # Handle physical encoder swaps
#                             if self.swap_encoders:
#                                 raw_ticks_l, raw_ticks_r = raw_ticks_r, raw_ticks_l
                                
#                             self.calculate_odometry(raw_ticks_l, raw_ticks_r)
#                         except ValueError:
#                             pass 
#             except UnicodeDecodeError:
#                 pass 

#         self.publish_data()

#     def calculate_odometry(self, ticks_l, ticks_r):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         if self.first_read:
#             self.last_ticks_l = ticks_l
#             self.last_ticks_r = ticks_r
#             self.first_read = False
#             self.last_time = current_time
#             return

#         # Calculate deltas and apply encoder direction multipliers
#         delta_ticks_l = (ticks_l - self.last_ticks_l) * self.encoder_dir_l
#         delta_ticks_r = (ticks_r - self.last_ticks_r) * self.encoder_dir_r

#         self.last_ticks_l = ticks_l
#         self.last_ticks_r = ticks_r
#         self.last_time = current_time

#         d_l = delta_ticks_l * self.dist_per_tick
#         d_r = delta_ticks_r * self.dist_per_tick

#         d_c = (d_l + d_r) / 2.0
#         d_th = (d_r - d_l) / self.track_width

#         self.v_x = d_c / dt if dt > 0 else 0.0
#         self.v_th = d_th / dt if dt > 0 else 0.0

#         self.x += d_c * math.cos(self.th + (d_th / 2.0))
#         self.y += d_c * math.sin(self.th + (d_th / 2.0))
#         self.th += d_th
        
#         self.left_wheel_angle += (d_l / self.wheel_radius)
#         self.right_wheel_angle += (d_r / self.wheel_radius)

#     def publish_data(self):
#         current_time = self.get_clock().now()

#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
        
#         # Ensure proper quaternion bounds
#         odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
#         odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

#         odom.twist.twist.linear.x = self.v_x
#         odom.twist.twist.angular.z = self.v_th

#         odom.pose.covariance[0] = 0.01   
#         odom.pose.covariance[7] = 0.01   
#         odom.pose.covariance[35] = 0.05  
#         odom.twist.covariance[0] = 0.01  
#         odom.twist.covariance[35] = 0.05 

#         self.odom_pub.publish(odom)

#         js = JointState()
#         js.header.stamp = current_time.to_msg()
#         js.name = ['wheel_left_joint', 'wheel_right_joint']
#         js.position = [self.left_wheel_angle, self.right_wheel_angle]
#         self.joint_pub.publish(js)

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaseController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down Base Controller...")
#         node.arduino.write("M,0,0\n".encode('utf-8'))
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import serial
import math

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # ==========================================
        # --- HARDWARE CALIBRATION & TWEAKS ---
        # ==========================================
        # 1. MOTORS: If the robot moves backward when commanded forward, change 1 to -1
        self.motor_dir_l = -1
        self.motor_dir_r = -1

        # 2. MOTORS: If the robot turns right when commanded left, set to True
        self.swap_motors = False

        # 3. ENCODERS: If ODOM moves backward when you manually push it forward, change 1 to -1
        self.encoder_dir_l = 1
        self.encoder_dir_r = 1

        # 4. ENCODERS: If ODOM turns right when you manually spin the robot left, set to True
        self.swap_encoders = False
        
        # ------------------------------------------
        # 5. ODOMETRY SCALING (The Fix for your Hallucination!)
        # ------------------------------------------
        # If RViz moves further/faster than the real robot, decrease this number (e.g., 0.5)
        # If RViz moves slower than the real robot, increase this number (e.g., 1.5)
        self.linear_scale_factor = 1.0  
        
        # If RViz rotates more than the real robot, INCREASE this number.
        # If RViz rotates less than the real robot, DECREASE this number.
        self.track_width_multiplier = 1.0
        # ==========================================

        # --- PHYSICAL ROBOT PARAMETERS ---
        self.wheel_radius = 0.0215  # 43mm diameter / 2
        self.track_width = 0.15     # Distance between wheels in meters
        self.ticks_per_rev = 670.0  
        self.dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # --- MOTOR TUNING PARAMETERS ---
        self.max_pwm = 255.0
        self.max_speed_m_s = (60.0 / 60.0) * (2.0 * math.pi * self.wheel_radius) 
        self.min_pwm = 80.0  
        
        # --- ODOMETRY STATE VARIABLES ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_x = 0.0
        self.v_th = 0.0
        
        self.last_ticks_l = 0
        self.last_ticks_r = 0
        self.first_read = True
        
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        self.last_time = self.get_clock().now()

        # --- HARDWARE INTERFACE ---
        self.serial_port_name = '/dev/arduino' 
        try:
            self.arduino = serial.Serial(self.serial_port_name, 115200, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # --- ROS 2 INTERFACES ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.timer = self.create_timer(0.05, self.read_and_publish)

    def speed_to_pwm(self, target_speed):
        if abs(target_speed) < 0.001:
            return 0
        
        pwm = (abs(target_speed) / self.max_speed_m_s) * (self.max_pwm - self.min_pwm)
        pwm += self.min_pwm
        pwm = min(max(pwm, 0), self.max_pwm)
        
        return int(pwm) if target_speed > 0 else int(-pwm)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        v_l = v - (omega * self.track_width / 2.0)
        v_r = v + (omega * self.track_width / 2.0)

        if self.swap_motors:
            v_l, v_r = v_r, v_l

        pwm_l = self.speed_to_pwm(v_l) * self.motor_dir_l
        pwm_r = self.speed_to_pwm(v_r) * self.motor_dir_r

        cmd_str = f"M,{pwm_l},{pwm_r}\n"
        self.arduino.write(cmd_str.encode('utf-8'))

    # def read_and_publish(self):
    #     while self.arduino.in_waiting:
    #         try:
    #             line = self.arduino.readline().decode('utf-8').strip()
    #             if line.startswith("E,") or line.startswith("ENC,"): # Handles both formats you might have
    #                 parts = line.split(',')
    #                 if len(parts) == 3:
    #                     try:
    #                         raw_ticks_l = int(parts[1])
    #                         raw_ticks_r = int(parts[2])
                            
    #                         if self.swap_encoders:
    #                             raw_ticks_l, raw_ticks_r = raw_ticks_r, raw_ticks_l
                                
    #                         self.calculate_odometry(raw_ticks_l, raw_ticks_r)
    #                     except ValueError:
    #                         pass 
    #         except UnicodeDecodeError:
    #             pass 

    #     self.publish_data()
    def read_and_publish(self):
        if self.arduino.in_waiting:
            try:
                line = self.arduino.readline().decode('utf-8').strip()
                if (line.startswith("E,") or line.startswith("ENC,")):
                    parts = line.split(',')
                    if len(parts) == 3:
                        try:
                            raw_ticks_l = int(parts[1])
                            raw_ticks_r = int(parts[2])
                            if self.swap_encoders:
                                raw_ticks_l, raw_ticks_r = raw_ticks_r, raw_ticks_l
                            self.calculate_odometry(raw_ticks_l, raw_ticks_r)
                        except ValueError:
                            pass
            except UnicodeDecodeError:
                pass
        self.publish_data()

    def calculate_odometry(self, ticks_l, ticks_r):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if self.first_read:
            self.last_ticks_l = ticks_l
            self.last_ticks_r = ticks_r
            self.first_read = False
            self.last_time = current_time
            return

        delta_ticks_l = (ticks_l - self.last_ticks_l) * self.encoder_dir_l
        delta_ticks_r = (ticks_r - self.last_ticks_r) * self.encoder_dir_r

        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r
        self.last_time = current_time

        # APPLY LINEAR SCALING FACTOR HERE
        d_l = delta_ticks_l * self.dist_per_tick * self.linear_scale_factor
        d_r = delta_ticks_r * self.dist_per_tick * self.linear_scale_factor

        d_c = (d_l + d_r) / 2.0
        
        # APPLY ANGULAR SCALING FACTOR HERE
        effective_track_width = self.track_width * self.track_width_multiplier
        d_th = (d_r - d_l) / effective_track_width

        alpha = 0.3
        self.v_x = alpha * (d_c / dt) + (1 - alpha) * self.v_x
        # self.v_x = d_c / dt if dt > 0 else 0.0
        self.v_th = d_th / dt if dt > 0 else 0.0

        self.x += d_c * math.cos(self.th + (d_th / 2.0))
        self.y += d_c * math.sin(self.th + (d_th / 2.0))
        self.th += d_th
        
        self.left_wheel_angle += (d_l / self.wheel_radius)
        self.right_wheel_angle += (d_r / self.wheel_radius)

    def publish_data(self):
        current_time = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x = self.v_x
        odom.twist.twist.angular.z = self.v_th

        odom.pose.covariance[0] = 0.01   
        odom.pose.covariance[7] = 0.01   
        odom.pose.covariance[35] = 0.05  
        odom.twist.covariance[0] = 0.01  
        odom.twist.covariance[35] = 0.05 

        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Base Controller...")
        node.arduino.write("M,0,0\n".encode('utf-8'))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()