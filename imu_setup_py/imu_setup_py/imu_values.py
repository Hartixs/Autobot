# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# import smbus2
# import math
# import time

# class MPU6050Visualizer(Node):
#     def __init__(self):
#         super().__init__('mpu6050_visualizer')
#         self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
#         self.bus = smbus2.SMBus(1)
#         self.addr = 0x68

#         try:
#             self.bus.write_byte_data(self.addr, 0x6B, 0)
#             time.sleep(0.1)
#             self.get_logger().info("MPU6050 awake. Calibrating gyro bias — keep robot still...")
#         except Exception as e:
#             self.get_logger().error(f"Failed to wake up MPU6050: {e}")

#         # ── NEW: Calibrate bias ──────────────────────────────────────────
#         self.gyro_bias_x = 0.0
#         self.gyro_bias_y = 0.0
#         self.gyro_bias_z = 0.0
#         self.accel_bias_x = 0.0
#         self.accel_bias_y = 0.0
#         self._calibrate()
#         # ─────────────────────────────────────────────────────────────────

#         self.pitch = 0.0
#         self.roll = 0.0
#         self.last_time = self.get_clock().now()
#         self.timer = self.create_timer(0.05, self.update)

#     def _calibrate(self, samples=200):
#         """Average 200 readings at rest to find sensor bias."""
#         gx_sum = gy_sum = gz_sum = 0.0
#         ax_sum = ay_sum = 0.0
#         for _ in range(samples):
#             try:
#                 gx_sum += self.read_raw(0x43) / 131.0
#                 gy_sum += self.read_raw(0x45) / 131.0
#                 gz_sum += self.read_raw(0x47) / 131.0
#                 ax_sum += self.read_raw(0x3B) / 16384.0
#                 ay_sum += self.read_raw(0x3D) / 16384.0
#             except OSError:
#                 pass
#             time.sleep(0.01)

#         self.gyro_bias_x = gx_sum / samples
#         self.gyro_bias_y = gy_sum / samples
#         self.gyro_bias_z = gz_sum / samples
#         self.accel_bias_x = ax_sum / samples
#         self.accel_bias_y = ay_sum / samples

#         self.get_logger().info(
#             f"Gyro bias — x:{self.gyro_bias_x:.4f}  y:{self.gyro_bias_y:.4f}  z:{self.gyro_bias_z:.4f} deg/s"
#         )

#     def read_raw(self, addr):
#         block = self.bus.read_i2c_block_data(self.addr, addr, 2)
#         val = (block[0] << 8) | block[1]
#         return val - 65536 if val >= 32768 else val

#     def get_quaternion_from_euler(self, roll, pitch, yaw):
#         """Convert Euler angles to Quaternion"""
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)

#         q = [0.0] * 4
#         q[0] = cy * cp * sr - sy * sp * cr # x
#         q[1] = sy * cp * sr + cy * sp * cr # y
#         q[2] = sy * cp * cr - cy * sp * sr # z
#         q[3] = cy * cp * cr + sy * sp * sr # w
#         return q

#     def update(self):
#         try:
#             ax_raw = self.read_raw(0x3B) / 16384.0
#             ay_raw = self.read_raw(0x3D) / 16384.0
#             az_raw = self.read_raw(0x3F) / 16384.0
#             gx_raw = self.read_raw(0x43) / 131.0
#             gy_raw = self.read_raw(0x45) / 131.0
#             gz_raw = self.read_raw(0x47) / 131.0
#         except OSError:
#             return

#         # ── Subtract calibrated bias ─────────────────────────────────────
#         gx = gx_raw - self.gyro_bias_x
#         gy = gy_raw - self.gyro_bias_y
#         gz = gz_raw - self.gyro_bias_z
#         ax = ax_raw - self.accel_bias_x
#         ay = ay_raw - self.accel_bias_y
#         az = az_raw  # keep Z as-is (gravity reference)

#         # ── Dead-zone: ignore sub-threshold values (lidar vibration) ─────
#         # Your still readings show ~0.04 rad/s noise — threshold at 0.05
#         GYRO_THRESHOLD = 0.05   # deg/s — tune this up if drift continues
#         ACCEL_THRESHOLD = 0.08  # g — tune if acceleration looks noisy

#         gx = 0.0 if abs(gx) < GYRO_THRESHOLD else gx
#         gy = 0.0 if abs(gy) < GYRO_THRESHOLD else gy
#         gz = 0.0 if abs(gz) < GYRO_THRESHOLD else gz
#         ax = 0.0 if abs(ax) < ACCEL_THRESHOLD else ax
#         ay = 0.0 if abs(ay) < ACCEL_THRESHOLD else ay

#         now = self.get_clock().now()
#         dt = (now - self.last_time).nanoseconds / 1e9
#         self.last_time = now

#         acc_pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
#         acc_roll  = math.atan2(-ax, az)
#         self.pitch = 0.96 * (self.pitch + math.radians(gy) * dt) + 0.04 * acc_pitch
#         self.roll  = 0.96 * (self.roll  + math.radians(gx) * dt) + 0.04 * acc_roll

#         q = self.get_quaternion_from_euler(self.roll, self.pitch, 0.0)

#         msg = Imu()
#         msg.header.stamp    = now.to_msg()
#         msg.header.frame_id = 'imu_link'

#         msg.orientation.x = -q[1]
#         msg.orientation.y =  q[0]
#         msg.orientation.z =  q[2]
#         msg.orientation.w =  q[3]

#         msg.angular_velocity.x = math.radians(gx)
#         msg.angular_velocity.y = math.radians(gy)
#         msg.angular_velocity.z = math.radians(gz)   # ← bias-corrected + dead-zoned

#         msg.linear_acceleration.x = -ax * 9.81
#         msg.linear_acceleration.y =  ay * 9.81
#         msg.linear_acceleration.z =  az * 9.81

#         msg.orientation_covariance         = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 9999.0]
#         msg.angular_velocity_covariance    = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.01  ]
#         msg.linear_acceleration_covariance = [0.1,  0, 0,  0, 0.1,  0,  0, 0, 0.1   ]

#         self.imu_pub.publish(msg)

# def main():
#     rclpy.init()
#     node = MPU6050Visualizer()
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


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# import smbus2
# import math
# import time

# class MPU6050Visualizer(Node):
#     def __init__(self):
#         super().__init__('mpu6050_visualizer')
#         self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
#         self.bus = smbus2.SMBus(1)
#         self.addr = 0x68

#         try:
#             self.bus.write_byte_data(self.addr, 0x6B, 0)
#             time.sleep(0.1)
#             self.get_logger().info("MPU6050 awake. Calibrating — keep robot still...")
#         except Exception as e:
#             self.get_logger().error(f"Failed to wake up MPU6050: {e}")

#         # Bias estimates
#         self.gyro_bias_x = 0.0
#         self.gyro_bias_y = 0.0
#         self.gyro_bias_z = 0.0
#         self.accel_bias_x = 0.0
#         self.accel_bias_y = 0.0

#         # State variables
#         self.pitch = 0.0
#         self.roll  = 0.0
#         self.yaw   = 0.0  # Added Yaw state

#         # Low-pass filter state for gz
#         self._gz_lpf = 0.0
#         self._GZ_LPF_ALPHA = 0.3

#         self._calibrate()

#         self.last_time = self.get_clock().now()
#         self.timer = self.create_timer(0.05, self.update)

#     def _calibrate(self, samples=500):
#         gx_vals, gy_vals, gz_vals = [], [], []
#         ax_vals, ay_vals = [], []

#         for _ in range(samples):
#             try:
#                 gx_vals.append(self.read_raw(0x43) / 131.0)
#                 gy_vals.append(self.read_raw(0x45) / 131.0)
#                 gz_vals.append(self.read_raw(0x47) / 131.0)
#                 ax_vals.append(self.read_raw(0x3B) / 16384.0)
#                 ay_vals.append(self.read_raw(0x3D) / 16384.0)
#             except OSError:
#                 pass
#             time.sleep(0.01)

#         def mean(lst): return sum(lst) / len(lst) if lst else 0.0

#         self.gyro_bias_x = mean(gx_vals)
#         self.gyro_bias_y = mean(gy_vals)
#         self.gyro_bias_z = mean(gz_vals)
#         self.accel_bias_x = mean(ax_vals)
#         self.accel_bias_y = mean(ay_vals)
#         self._gz_lpf = 0.0

#         self.get_logger().info(
#             f"Bias calibrated. Z-Gyro bias: {self.gyro_bias_z:.4f} deg/s"
#         )

#     def read_raw(self, addr):
#         block = self.bus.read_i2c_block_data(self.addr, addr, 2)
#         val = (block[0] << 8) | block[1]
#         return val - 65536 if val >= 32768 else val

#     def get_quaternion_from_euler(self, roll, pitch, yaw):
#         cy = math.cos(yaw   * 0.5); sy = math.sin(yaw   * 0.5)
#         cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll  * 0.5); sr = math.sin(roll  * 0.5)
#         return [
#             cy * cp * sr - sy * sp * cr,   # x
#             sy * cp * sr + cy * sp * cr,   # y
#             sy * cp * cr - cy * sp * sr,   # z
#             cy * cp * cr + sy * sp * sr,   # w
#         ]

#     def update(self):
#         try:
#             ax_raw = self.read_raw(0x3B) / 16384.0
#             ay_raw = self.read_raw(0x3D) / 16384.0
#             az_raw = self.read_raw(0x3F) / 16384.0
#             gx_raw = self.read_raw(0x43) / 131.0
#             gy_raw = self.read_raw(0x45) / 131.0
#             gz_raw = self.read_raw(0x47) / 131.0
#         except OSError:
#             return

#         # Subtract bias
#         gx = gx_raw - self.gyro_bias_x
#         gy = gy_raw - self.gyro_bias_y
#         gz = gz_raw - self.gyro_bias_z
#         ax = ax_raw - self.accel_bias_x
#         ay = ay_raw - self.accel_bias_y
#         az = az_raw

#         # Dead-zones
#         GYRO_THRESH_XY = 2.5
#         GYRO_THRESH_Z  = 1.0
#         ACCEL_THRESHOLD = 0.10

#         gx = 0.0 if abs(gx) < GYRO_THRESH_XY else gx
#         gy = 0.0 if abs(gy) < GYRO_THRESH_XY else gy
#         ax = 0.0 if abs(ax) < ACCEL_THRESHOLD else ax
#         ay = 0.0 if abs(ay) < ACCEL_THRESHOLD else ay
#         gz_dz = 0.0 if abs(gz) < GYRO_THRESH_Z else gz

#         # LPF on gz
#         self._gz_lpf = self._GZ_LPF_ALPHA * self._gz_lpf + (1.0 - self._GZ_LPF_ALPHA) * gz_dz
#         gz = self._gz_lpf

#         now = self.get_clock().now()
#         dt  = (now - self.last_time).nanoseconds / 1e9
#         self.last_time = now

#         # Calculate Roll, Pitch, and Integrate Yaw
#         acc_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
#         acc_roll  = math.atan2(ay, az)
#         self.pitch = 0.96 * (self.pitch + math.radians(gy) * dt) + 0.04 * acc_pitch
#         self.roll  = 0.96 * (self.roll  + math.radians(gx) * dt) + 0.04 * acc_roll
#         self.yaw  += math.radians(gz) * dt  # Angle = velocity * time

#         q = self.get_quaternion_from_euler(self.roll, self.pitch, self.yaw)

#         msg = Imu()
#         msg.header.stamp    = now.to_msg()
#         msg.header.frame_id = 'imu_link'

#         msg.orientation.x = q[0]
#         msg.orientation.y = q[1]
#         msg.orientation.z = q[2]
#         msg.orientation.w = q[3]

#         msg.angular_velocity.x = math.radians(gx)
#         msg.angular_velocity.y = math.radians(gy)
#         msg.angular_velocity.z = math.radians(gz)

#         msg.linear_acceleration.x = ax * 9.81
#         msg.linear_acceleration.y = ay * 9.81
#         msg.linear_acceleration.z = az * 9.81

#         msg.orientation_covariance         = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.05]
#         msg.angular_velocity_covariance    = [0.05, 0, 0,  0, 0.05, 0,  0, 0, 0.05]
#         msg.linear_acceleration_covariance = [0.1,  0, 0,  0, 0.1,  0,  0, 0, 0.1 ]

#         self.imu_pub.publish(msg)

# def main():
#     rclpy.init()
#     node = MPU6050Visualizer()
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