# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Header
# from geometry_msgs.msg import Vector3
# import math

# # Standard Python library for MPU6050
# from mpu6050 import mpu6050

# class ImuPublisher(Node):
#     def __init__(self):
#         super().__init__('imu_publisher')

#         # NOTE: Changed topic to /imu/data_raw. 
#         # In ROS, raw 6-DOF data goes to "data_raw", and filtered 9-DOF data goes to "data"
#         self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

#         self.timer = self.create_timer(0.1, self.publish_imu_data)

#         # Initialize the MPU6050 sensor on the default I2C address (0x68)
#         try:
#             self.sensor = mpu6050(0x68)
#             self.get_logger().info("MPU6050 IMU initialized on I2C address 0x68")
#         except Exception as e:
#             self.get_logger().error(f"Failed to initialize MPU6050: {e}. Check I2C wiring!")

#         # Define sensor-specific offsets (You will need to calibrate your specific MPU6050)
#         self.gyro_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
#         self.accel_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}

#     def publish_imu_data(self):
#         msg = Imu()

#         # Populate the header
#         msg.header = Header()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'imu_link' # Changed from 'imu' to match standard ROS TF conventions

#         try:
#             # Read sensor data
#             accel_data = self.sensor.get_accel_data()
#             gyro_data = self.sensor.get_gyro_data()

#             # 1. ORIENTATION (Not available on 6-DOF MPU6050)
#             # CRITICAL: Setting covariance[0] to -1.0 tells the ROS EKF to ignore orientation
#             msg.orientation_covariance[0] = -1.0

#             # 2. ANGULAR VELOCITY (Gyroscope)
#             # The mpu6050 library returns degrees/sec. ROS requires radians/sec.
#             msg.angular_velocity = Vector3(
#                 x=float(math.radians(gyro_data['x'] - self.gyro_offsets['x'])),
#                 y=float(math.radians(gyro_data['y'] - self.gyro_offsets['y'])),
#                 z=float(math.radians(gyro_data['z'] - self.gyro_offsets['z']))
#             )
#             # Provide a baseline covariance matrix (diagonal) so the EKF trusts it
#             msg.angular_velocity_covariance = [
#                 0.001, 0.0,   0.0,
#                 0.0,   0.001, 0.0,
#                 0.0,   0.0,   0.001
#             ]

#             # 3. LINEAR ACCELERATION (Accelerometer)
#             # The mpu6050 library automatically converts Gs to m/s^2, which is what ROS requires.
#             msg.linear_acceleration = Vector3(
#                 x=float(accel_data['x'] - self.accel_offsets['x']),
#                 y=float(accel_data['y'] - self.accel_offsets['y']),
#                 z=float(accel_data['z'] - self.accel_offsets['z'])
#             )
#             # Provide a baseline covariance matrix
#             msg.linear_acceleration_covariance = [
#                 0.01, 0.0,  0.0,
#                 0.0,  0.01, 0.0,
#                 0.0,  0.0,  0.01
#             ]

#             # Publish the message
#             self.publisher_.publish(msg)

#         except Exception as e:
#             self.get_logger().warn(f"Failed to read MPU6050 data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuPublisher()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("IMU publisher stopped cleanly")
#     except Exception as e:
#         node.get_logger().error(f"Error: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Publish to standard IMU topic
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # I2C Setup for MPU6050
        self.bus = smbus2.SMBus(1)
        self.addr = 0x68
        
        # Wake up MPU6050
        # try:
        #     self.bus.write_byte_data(self.addr, 0x6B, 0)
        #     time.sleep(0.1)
        #     self.get_logger().info("MPU6050 awake. Calibrating... Keep robot STILL!")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to wake up MPU6050: {e}. Check wiring!")
        #     return
        # Wake up MPU6050
        self.imu_ok = False   # ← add this flag
        try:
            self.bus.write_byte_data(self.addr, 0x6B, 0)
            time.sleep(0.1)
            self.get_logger().info("MPU6050 awake. Calibrating... Keep robot STILL!")
            self.imu_ok = True
        except Exception as e:
            self.get_logger().error(f"Failed to wake up MPU6050: {e}. Running WITHOUT IMU.")
            # Don't return — let the node stay alive

        if self.imu_ok:
            self.calibrate_gyro()

        # --- Calibration (Bias calculation) ---
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.calibrate_gyro()

        # State variables for Complementary Filter
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Read and publish at 20Hz
        self.timer = self.create_timer(0.05, self.publish_imu)
        self.get_logger().info("IMU Calibration complete. Publishing data...")

    # def read_word_2c(self, reg):
    #     try:
    #         high = self.bus.read_byte_data(self.addr, reg)
    #         low = self.bus.read_byte_data(self.addr, reg + 1)
    #         val = (high << 8) + low
    #         if val >= 0x8000:
    #             return -((65535 - val) + 1)
    #         else:
    #             return val
    #     except Exception:
    #         return 0
    def read_word_2c(self, reg):
        try:
            high = self.bus.read_byte_data(self.addr, reg)
            low  = self.bus.read_byte_data(self.addr, reg + 1)
            val  = (high << 8) + low
            return val - 65536 if val >= 0x8000 else val
        except Exception:
            raise   # let calibrate_gyro and publish_imu handle it their own way

    # def calibrate_gyro(self):
    #     samples = 100
    #     gx_sum, gy_sum, gz_sum = 0, 0, 0
    #     for _ in range(samples):
    #         gx_sum += self.read_word_2c(0x43)
    #         gy_sum += self.read_word_2c(0x45)
    #         gz_sum += self.read_word_2c(0x47)
    #         time.sleep(0.01)
            
    #     # 131.0 is the default scale factor for gyro (250 deg/s)
    #     self.gyro_bias_x = (gx_sum / samples) / 131.0
    #     self.gyro_bias_y = (gy_sum / samples) / 131.0
    #     self.gyro_bias_z = (gz_sum / samples) / 131.0

    def calibrate_gyro(self):
        samples = 100
        gx_sum, gy_sum, gz_sum = 0, 0, 0
        good = 0
        for _ in range(samples):
            try:                                    # ← wrap individual reads
                gx_sum += self.read_word_2c(0x43)
                gy_sum += self.read_word_2c(0x45)
                gz_sum += self.read_word_2c(0x47)
                good += 1
            except Exception:
                pass                                # skip bad reads, keep going
            time.sleep(0.01)

        if good == 0:
            self.get_logger().error("Calibration failed — check I2C wiring!")
            return

        self.gyro_bias_x = (gx_sum / good) / 131.0
        self.gyro_bias_y = (gy_sum / good) / 131.0
        self.gyro_bias_z = (gz_sum / good) / 131.0
        self.get_logger().info(
            f"Gyro bias — x:{self.gyro_bias_x:.4f} y:{self.gyro_bias_y:.4f} z:{self.gyro_bias_z:.4f}"
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_imu(self):
        if not self.imu_ok:
            return   # silently skip, don't crash
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            ax = self.read_word_2c(0x3B) / 16384.0

            # Read Raw Data
            # 16384.0 is the scale factor for accelerometer (2g)
            ax = self.read_word_2c(0x3B) / 16384.0
            ay = self.read_word_2c(0x3D) / 16384.0
            az = self.read_word_2c(0x3F) / 16384.0

            gx = (self.read_word_2c(0x43) / 131.0) - self.gyro_bias_x
            gy = (self.read_word_2c(0x45) / 131.0) - self.gyro_bias_y
            gz = (self.read_word_2c(0x47) / 131.0) - self.gyro_bias_z

            # Complementary Filter for Orientation
            acc_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            acc_roll = math.atan2(ay, az)
            
            self.pitch = 0.96 * (self.pitch + math.radians(gy) * dt) + 0.04 * acc_pitch
            self.roll = 0.96 * (self.roll + math.radians(gx) * dt) + 0.04 * acc_roll
            self.yaw += math.radians(gz) * dt 

            q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

            # Build ROS Message
            msg = Imu()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = 'imu_link' # This now matches the joint we added to the URDF!

            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            msg.linear_acceleration.x = ax * 9.81
            msg.linear_acceleration.y = ay * 9.81
            msg.linear_acceleration.z = az * 9.81

            # Covariances (Important for the ESKF to trust the data)
            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.05]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"IMU read failed: {e}. Skipping frame.")
            return  # ← skip this frame, don't crash

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()