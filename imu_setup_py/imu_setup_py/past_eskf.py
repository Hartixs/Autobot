# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
# from tf2_ros import TransformBroadcaster
# from rclpy.time import Time

# class ErrorStateKalmanFilter(Node):
#     def __init__(self):
#         super().__init__('eskf_localization_node')

#         # --- 1. State Initialization ---
#         # Nominal State: [x, y, theta, v, omega]
#         self.x_nom = np.zeros((5, 1))
        
#         # Error State Covariance (P)
#         self.P = np.eye(5) * 0.1 
        
#         # Process Noise (Q) - Trust in IMU kinematics
#         self.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05])
        
#         # Measurement Noise (R)
#         self.R_enc = np.array([[0.05]])                   # Encoder velocity noise
#         self.R_lidar = np.diag([0.02, 0.02, 0.01])        # Lidar x, y, theta noise

#         self.last_imu_time = self.get_clock().now()

#         # --- 2. ROS 2 Interfaces ---
#         # Publishers
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Subscribers (Using your ekf.yaml topics)
#         self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
#         self.create_subscription(Odometry, '/odom_unfiltered', self.encoder_callback, 10)
        
#         # TODO: Update this based on your LiDAR topic/type!
#         self.create_subscription(PoseWithCovarianceStamped, '/lidar_pose', self.lidar_callback, 10)

#     # ==========================================
#     # PREDICTION STEP (Driven by IMU at high Hz)
#     # ==========================================
#     def imu_callback(self, msg: Imu):
#         now = Time.from_msg(msg.header.stamp)
#         dt = (now - self.last_imu_time).nanoseconds / 1e9
#         if dt <= 0 or dt > 0.5:
#             self.last_imu_time = now
#             return
#         self.last_imu_time = now

#         omega_meas = msg.angular_velocity.z   # gyro yaw rate (bias-corrected now)
#         # ax removed — encoders handle velocity, IMU accel is too noisy

#         theta = float(self.x_nom[2])
#         v     = float(self.x_nom[3])          # velocity comes from encoder correction

#         self.x_nom[0] += v * math.cos(theta) * dt
#         self.x_nom[1] += v * math.sin(theta) * dt
#         self.x_nom[2] += omega_meas * dt
#         # self.x_nom[3] += ax * dt  ← DELETE THIS LINE
#         self.x_nom[4]  = omega_meas

#         self.x_nom[2] = math.atan2(math.sin(self.x_nom[2]), math.cos(self.x_nom[2]))

#         F = np.eye(5)
#         F[0, 2] = -v * math.sin(theta) * dt
#         F[0, 3] =  math.cos(theta) * dt
#         F[1, 2] =  v * math.cos(theta) * dt
#         F[1, 3] =  math.sin(theta) * dt

#         self.P = F @ self.P @ F.T + (self.Q * dt)
#         self.publish_odometry(msg.header.stamp)

#     # ==========================================
#     # CORRECTION STEP 1: WHEEL ENCODERS
#     # ==========================================
#     def encoder_callback(self, msg: Odometry):
#         # Observation vector (Encoders give us forward velocity v)
#         z = np.array([[msg.twist.twist.linear.x]])
        
#         # Measurement Jacobian (We are measuring state index 3: v)
#         H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        
#         self.eskf_update(z, H, self.R_enc)

#     # ==========================================
#     # CORRECTION STEP 2: LIDAR
#     # ==========================================
#     def lidar_callback(self, msg: PoseWithCovarianceStamped):
#         # Extract LiDAR x, y, and yaw (theta)
#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         yaw = math.atan2(siny_cosp, cosy_cosp)

#         # Observation vector [x, y, theta]
#         z = np.array([
#             [msg.pose.pose.position.x],
#             [msg.pose.pose.position.y],
#             [yaw]
#         ])

#         # Measurement Jacobian (We are measuring states 0, 1, 2)
#         H = np.array([
#             [1.0, 0.0, 0.0, 0.0, 0.0],
#             [0.0, 1.0, 0.0, 0.0, 0.0],
#             [0.0, 0.0, 1.0, 0.0, 0.0]
#         ])

#         # Dynamic R matrix from LiDAR covariance
#         R = np.diag([
#             msg.pose.covariance[0] + 0.01,
#             msg.pose.covariance[7] + 0.01,
#             msg.pose.covariance[35] + 0.01
#         ])

#         # Compute angular error correctly to avoid 360-degree wrapping issues
#         innovation = z - (H @ self.x_nom)
#         innovation[2, 0] = math.atan2(math.sin(innovation[2, 0]), math.cos(innovation[2, 0]))

#         self.eskf_update_with_innovation(innovation, H, R)

#     # ==========================================
#     # ESKF CORE MATH
#     # ==========================================
#     def eskf_update(self, z, H, R):
#         innovation = z - (H @ self.x_nom)
#         self.eskf_update_with_innovation(innovation, H, R)

#     def eskf_update_with_innovation(self, innovation, H, R):
#         # 1. Compute Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
#         S = H @ self.P @ H.T + R
#         K = self.P @ H.T @ np.linalg.inv(S)

#         # 2. Compute Error State: delta_x = K * innovation
#         delta_x = K @ innovation

#         # 3. Inject Error State into Nominal State (x = x + delta_x)
#         self.x_nom += delta_x
#         self.x_nom[2] = math.atan2(math.sin(self.x_nom[2]), math.cos(self.x_nom[2])) # keep yaw normalized

#         # 4. Reset Error State & Update Covariance: P = (I - K * H) * P
#         I = np.eye(5)
#         self.P = (I - K @ H) @ self.P

#     # ==========================================
#     # PUBLISHING
#     # ==========================================
#     def publish_odometry(self, stamp):
#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'

#         odom.pose.pose.position.x = float(self.x_nom[0])
#         odom.pose.pose.position.y = float(self.x_nom[1])
        
#         # Convert yaw back to quaternion
#         cy = math.cos(self.x_nom[2] * 0.5)
#         sy = math.sin(self.x_nom[2] * 0.5)
#         odom.pose.pose.orientation.w = cy
#         odom.pose.pose.orientation.z = sy

#         odom.twist.twist.linear.x = float(self.x_nom[3])
#         odom.twist.twist.angular.z = float(self.x_nom[4])

#         self.odom_pub.publish(odom)

#         # Broadcast TF
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = float(self.x_nom[0])
#         t.transform.translation.y = float(self.x_nom[1])
#         t.transform.rotation.w = cy
#         t.transform.rotation.z = sy
#         self.tf_broadcaster.sendTransform(t)

# def main():
#     rclpy.init()
#     node = ErrorStateKalmanFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.time import Time


class ErrorStateKalmanFilter(Node):
    def __init__(self):
        super().__init__('eskf_localization_node')

        # --- 1. State Initialization ---
        # Nominal State: [x, y, theta, v, omega]
        self.x_nom = np.zeros((5, 1))

        # Error State Covariance (P)
        self.P = np.eye(5) * 0.1

        # Process Noise (Q)
        # FIX 1: Raise Q for theta (index 2) and omega (index 4) so the filter
        # trusts the gyro prediction LESS and corrections pull it back faster.
        # Old values were 0.01/0.05 — bumped to reflect observed ~1 deg/s noise.
        self.Q = np.diag([0.01,   # x position
                          0.01,   # y position
                          0.05,   # theta  ← was 0.01
                          0.05,   # v
                          0.10])  # omega  ← was 0.05

        # Measurement Noise (R)
        self.R_enc   = np.array([[0.05]])
        self.R_lidar = np.diag([0.02, 0.02, 0.01])

        # FIX 2: Zero-Velocity Update (ZUPT) noise — very tight because
        # "robot is still" is a highly reliable observation.
        self.R_zupt_v     = np.array([[1e-4]])   # velocity = 0
        self.R_zupt_omega = np.array([[1e-4]])   # yaw-rate = 0

        self.last_imu_time = self.get_clock().now()

        # FIX 3: Track encoder velocity so the IMU callback can gate integration
        self._encoder_velocity = 0.0
        self._STILL_THRESH = 0.02   # m/s — below this we treat robot as stationary

        # --- 2. ROS 2 Interfaces ---
        self.odom_pub      = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu,  '/imu/data',       self.imu_callback,     10)
        self.create_subscription(Odometry, '/odom_raw', self.encoder_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/lidar_pose',
                                 self.lidar_callback, 10)

    # =========================================================================
    # PREDICTION STEP  (IMU — high Hz)
    # =========================================================================
    def imu_callback(self, msg: Imu):
        now = Time.from_msg(msg.header.stamp)
        dt  = (now - self.last_imu_time).nanoseconds / 1e9
        if dt <= 0 or dt > 0.5:
            self.last_imu_time = now
            return
        self.last_imu_time = now

        omega_meas = msg.angular_velocity.z   # already bias-corrected + LPF in imu_values.py

        robot_is_still = abs(self._encoder_velocity) < self._STILL_THRESH

        # ------------------------------------------------------------------
        # FIX 4: Velocity-gated gyro integration (ZUPT core idea)
        #
        # A differential-drive robot CANNOT rotate without its wheels turning.
        # If encoders say v ≈ 0, any non-zero omega is pure IMU noise — don't
        # integrate it.  This eliminates the dominant drift source.
        # ------------------------------------------------------------------
        if robot_is_still:
            omega_for_integration = 0.0
        else:
            omega_for_integration = omega_meas

        theta = float(self.x_nom[2])
        v     = float(self.x_nom[3])   # comes from encoder correction

        # Propagate nominal state
        self.x_nom[0] += v * math.cos(theta) * dt
        self.x_nom[1] += v * math.sin(theta) * dt
        self.x_nom[2] += omega_for_integration * dt
        self.x_nom[4]  = omega_for_integration   # omega state tracks what we integrated

        self.x_nom[2] = math.atan2(math.sin(float(self.x_nom[2])),
                                   math.cos(float(self.x_nom[2])))

        # Linearised transition matrix F
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] =  math.cos(theta) * dt
        F[1, 2] =  v * math.cos(theta) * dt
        F[1, 3] =  math.sin(theta) * dt

        self.P = F @ self.P @ F.T + (self.Q * dt)

        # ------------------------------------------------------------------
        # FIX 5: ZUPT corrections when robot is still
        #
        # Inject two "virtual" measurements: v=0 and omega=0.
        # This actively pulls the state back and shrinks P so drift cannot
        # accumulate during stationary periods.
        # ------------------------------------------------------------------
        if robot_is_still:
            # Correct velocity → 0
            z_v = np.array([[0.0]])
            H_v = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
            self.eskf_update(z_v, H_v, self.R_zupt_v)

            # Correct yaw-rate → 0
            z_w = np.array([[0.0]])
            H_w = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
            self.eskf_update(z_w, H_w, self.R_zupt_omega)

        self.publish_odometry(msg.header.stamp)

    # =========================================================================
    # CORRECTION STEP 1: WHEEL ENCODERS
    # =========================================================================
    def encoder_callback(self, msg: Odometry):
        # Cache for ZUPT gating in IMU callback
        self._encoder_velocity = msg.twist.twist.linear.x

        z = np.array([[self._encoder_velocity]])
        H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        self.eskf_update(z, H, self.R_enc)

    # =========================================================================
    # CORRECTION STEP 2: LIDAR
    # =========================================================================
    def lidar_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        z = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [yaw]
        ])
        H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0]
        ])
        R = np.diag([
            msg.pose.covariance[0]  + 0.01,
            msg.pose.covariance[7]  + 0.01,
            msg.pose.covariance[35] + 0.01
        ])

        innovation = z - (H @ self.x_nom)
        innovation[2, 0] = math.atan2(math.sin(innovation[2, 0]),
                                       math.cos(innovation[2, 0]))
        self.eskf_update_with_innovation(innovation, H, R)

    # =========================================================================
    # ESKF CORE MATH
    # =========================================================================
    def eskf_update(self, z, H, R):
        innovation = z - (H @ self.x_nom)
        self.eskf_update_with_innovation(innovation, H, R)

    def eskf_update_with_innovation(self, innovation, H, R):
        S     = H @ self.P @ H.T + R
        K     = self.P @ H.T @ np.linalg.inv(S)
        delta = K @ innovation

        self.x_nom   += delta
        self.x_nom[2] = math.atan2(math.sin(float(self.x_nom[2])),
                                    math.cos(float(self.x_nom[2])))
        I         = np.eye(5)
        self.P    = (I - K @ H) @ self.P

        # FIX 6: Joseph form for numerical stability (keeps P symmetric & PSD)
        # self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

    # =========================================================================
    # PUBLISHING
    # =========================================================================
    def publish_odometry(self, stamp):
        cy = math.cos(float(self.x_nom[2]) * 0.5)
        sy = math.sin(float(self.x_nom[2]) * 0.5)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = float(self.x_nom[0])
        odom.pose.pose.position.y    = float(self.x_nom[1])
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x  = float(self.x_nom[3])
        odom.twist.twist.angular.z = float(self.x_nom[4])

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = float(self.x_nom[0])
        t.transform.translation.y = float(self.x_nom[1])
        t.transform.rotation.w    = cy
        t.transform.rotation.z    = sy
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ErrorStateKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# """
# ESKF — Error State Kalman Filter for pure LIO (LiDAR-Inertial Odometry)

# Prediction : /imu/data_raw   (MPU6050 gyro → yaw rate)
# Correction : /odom_rf2o      (rf2o laser odometry → v, omega, x, y, theta)
# Output     : /odom            (filtered odometry)
#              TF odom → base_footprint
# """

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster


# class ErrorStateKalmanFilter(Node):
#     def __init__(self):
#         super().__init__('eskf_localization_node')

#         # ── Nominal state: [x, y, theta, v, omega] ────────────────────────────
#         self.x_nom = np.zeros((5, 1))

#         # ── Error covariance ──────────────────────────────────────────────────
#         self.P = np.eye(5) * 0.1

#         # ── Process noise per SECOND (will be scaled by dt in predict step) ──
#         # These represent how much we trust the IMU motion model per second.
#         self.Q = np.diag([
#             0.01,   # x     — position drifts slowly
#             0.01,   # y
#             0.05,   # theta — yaw drifts faster due to gyro bias
#             0.05,   # v     — velocity can change quickly
#             0.10,   # omega — angular velocity is noisy
#         ])

#         # ── Measurement noise for rf2o correction ─────────────────────────────
#         # rf2o gives us [v, omega, x, y, theta] but we only trust v and omega
#         # directly; x/y/theta from rf2o accumulate scan-matching error so we
#         # give them moderate trust.
#         self.R_v      = np.array([[0.02]])          # forward velocity
#         self.R_omega  = np.array([[0.05]])          # angular velocity

#         self.last_imu_time   = self.get_clock().now()
#         self.rf2o_initialized = False               # wait for first rf2o msg

#         # ── ROS interfaces ────────────────────────────────────────────────────
#         self.odom_pub      = self.create_publisher(Odometry, '/odom', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.create_subscription(Imu,      '/imu/data_raw', self.imu_callback,       50)
#         self.create_subscription(Odometry, '/odom_rf2o',    self.lidar_odom_callback, 50)

#         self.get_logger().info("ESKF LIO node started — waiting for /imu/data_raw and /odom_rf2o")

#     # ══════════════════════════════════════════════════════════════════════════
#     # PREDICTION STEP  (runs at IMU rate, ~10 Hz)
#     # ══════════════════════════════════════════════════════════════════════════
#     def imu_callback(self, msg: Imu):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_imu_time).nanoseconds / 1e9
#         self.last_imu_time = current_time

#         # Guard against bad dt (first call, ROS time jump, etc.)
#         if dt <= 0.0 or dt > 0.5:
#             return

#         # Don't integrate until rf2o has given us an initial velocity estimate.
#         # Without this, pure IMU integration from zero is meaningless.
#         if not self.rf2o_initialized:
#             return

#         omega_imu = msg.angular_velocity.z   # rad/s, bias-corrected by imu_publisher

#         theta = float(self.x_nom[2])
#         v     = float(self.x_nom[3])

#         # Kinematic prediction
#         self.x_nom[0] += v * math.cos(theta) * dt
#         self.x_nom[1] += v * math.sin(theta) * dt
#         self.x_nom[2] += omega_imu * dt
#         self.x_nom[2]  = math.atan2(math.sin(float(self.x_nom[2])),
#                                      math.cos(float(self.x_nom[2])))
#         self.x_nom[4]  = omega_imu

#         # Jacobian of motion model
#         F = np.eye(5)
#         F[0, 2] = -v * math.sin(theta) * dt
#         F[0, 3] =  math.cos(theta) * dt
#         F[1, 2] =  v * math.cos(theta) * dt
#         F[1, 3] =  math.sin(theta) * dt

#         # Scale Q by dt so uncertainty grows per-second, not per-callback
#         self.P = F @ self.P @ F.T + self.Q * dt

#         self.publish_odometry(msg.header.stamp)

#     # ══════════════════════════════════════════════════════════════════════════
#     # CORRECTION STEP  (runs at rf2o rate, ~10 Hz)
#     # ══════════════════════════════════════════════════════════════════════════
#     def lidar_odom_callback(self, msg: Odometry):
#         # Mark that we have received at least one valid lidar odom message
#         if not self.rf2o_initialized:
#             self.rf2o_initialized = True
#             self.get_logger().info("rf2o odometry received — ESKF correction active.")

#         # ── Correction 1: forward velocity ────────────────────────────────────
#         v_meas = msg.twist.twist.linear.x
#         H_v    = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
#         self._eskf_update(np.array([[v_meas]]), H_v, self.R_v)

#         # ── Correction 2: angular velocity (yaw rate from scan matching) ──────
#         omega_meas = msg.twist.twist.angular.z
#         H_omega    = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
#         self._eskf_update(np.array([[omega_meas]]), H_omega, self.R_omega)

#         # Publish after correction so SLAM gets the freshest estimate
#         self.publish_odometry(msg.header.stamp)

#     # ══════════════════════════════════════════════════════════════════════════
#     # ESKF CORE MATH
#     # ══════════════════════════════════════════════════════════════════════════
#     def _eskf_update(self, z, H, R):
#         """Standard Kalman update with scalar or vector measurement z."""
#         S       = H @ self.P @ H.T + R
#         K       = self.P @ H.T @ np.linalg.inv(S)
#         innov   = z - (H @ self.x_nom)
#         self.x_nom += K @ innov
#         self.x_nom[2] = math.atan2(math.sin(float(self.x_nom[2])),
#                                     math.cos(float(self.x_nom[2])))
#         I      = np.eye(5)
#         self.P = (I - K @ H) @ self.P

#     # ══════════════════════════════════════════════════════════════════════════
#     # PUBLISHING
#     # ══════════════════════════════════════════════════════════════════════════
#     def publish_odometry(self, stamp):
#         theta = float(self.x_nom[2])
#         cy    = math.cos(theta * 0.5)
#         sy    = math.sin(theta * 0.5)

#         # ── /odom message ─────────────────────────────────────────────────────
#         odom = Odometry()
#         odom.header.stamp    = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id  = 'base_footprint'

#         odom.pose.pose.position.x    = float(self.x_nom[0])
#         odom.pose.pose.position.y    = float(self.x_nom[1])
#         odom.pose.pose.position.z    = 0.0
#         odom.pose.pose.orientation.w = cy
#         odom.pose.pose.orientation.z = sy

#         odom.twist.twist.linear.x  = float(self.x_nom[3])
#         odom.twist.twist.angular.z = float(self.x_nom[4])

#         # Covariance from filter's P matrix (position and heading)
#         odom.pose.covariance[0]  = float(self.P[0, 0])   # xx
#         odom.pose.covariance[7]  = float(self.P[1, 1])   # yy
#         odom.pose.covariance[35] = float(self.P[2, 2])   # theta-theta
#         odom.twist.covariance[0] = float(self.P[3, 3])   # vv
#         odom.twist.covariance[35]= float(self.P[4, 4])   # omega-omega

#         self.odom_pub.publish(odom)

#         # ── odom → base_footprint TF ──────────────────────────────────────────
#         t = TransformStamped()
#         t.header.stamp            = self.get_clock().now().to_msg()
#         t.header.frame_id         = 'odom'
#         t.child_frame_id          = 'base_footprint'
#         t.transform.translation.x = float(self.x_nom[0])
#         t.transform.translation.y = float(self.x_nom[1])
#         t.transform.translation.z = 0.0
#         t.transform.rotation.w    = cy
#         t.transform.rotation.z    = sy

#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ErrorStateKalmanFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# """
# ESKF — Error State Kalman Filter for Wheel-Inertial Odometry

# Prediction : /imu/data_raw       (MPU6050 gyro → yaw rate)
# Correction : /odom_unfiltered    (Wheel encoders → v, omega)
# Output     : /odom               (filtered odometry)
#              TF odom → base_footprint
# """

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster


# class ErrorStateKalmanFilter(Node):
#     def __init__(self):
#         super().__init__('eskf_localization_node')

#         # ── Nominal state: [x, y, theta, v, omega] ────────────────────────────
#         self.x_nom = np.zeros((5, 1))

#         # ── Error covariance ──────────────────────────────────────────────────
#         self.P = np.eye(5) * 0.1

#         # ── Process noise per SECOND (will be scaled by dt in predict step) ──
#         # These represent how much we trust the IMU motion model per second.
#         self.Q = np.diag([
#             0.01,   # x     — position drifts slowly
#             0.01,   # y
#             0.05,   # theta — yaw drifts faster due to gyro bias
#             0.05,   # v     — velocity can change quickly
#             0.10,   # omega — angular velocity is noisy
#         ])

#         # ── Measurement noise for Wheel Encoder correction ────────────────────
#         # Wheel encoders give us reliable forward velocity, but angular velocity
#         # can slip. We trust forward velocity (v) highly.
#         self.R_v     = np.array([[0.02]])         # forward velocity from wheels
#         self.R_omega = np.array([[0.08]])         # angular velocity from wheels

#         self.last_imu_time   = self.get_clock().now()
#         self.wheels_initialized = False           # wait for first wheel msg

#         # ── ROS interfaces ────────────────────────────────────────────────────
#         self.odom_pub      = self.create_publisher(Odometry, '/odom', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Listen to IMU and the raw wheel odometry from base_controller.py
#         self.create_subscription(Imu,      '/imu/data_raw',    self.imu_callback,       50)
#         self.create_subscription(Odometry, '/odom_unfiltered', self.wheel_odom_callback, 50)

#         self.get_logger().info("ESKF node started — waiting for /imu/data_raw and /odom_unfiltered")

#     # ══════════════════════════════════════════════════════════════════════════
#     # PREDICTION STEP  (runs at IMU rate)
#     # ══════════════════════════════════════════════════════════════════════════
#     def imu_callback(self, msg: Imu):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_imu_time).nanoseconds / 1e9
#         self.last_imu_time = current_time

#         # Guard against bad dt (first call, ROS time jump, etc.)
#         if dt <= 0.0 or dt > 0.5:
#             return

#         # Don't integrate until wheels have given us an initial velocity estimate.
#         if not self.wheels_initialized:
#             return

#         omega_imu = msg.angular_velocity.z   # rad/s, bias-corrected by imu_publisher

#         theta = float(self.x_nom[2])
#         v     = float(self.x_nom[3])

#         # Kinematic prediction
#         self.x_nom[0] += v * math.cos(theta) * dt
#         self.x_nom[1] += v * math.sin(theta) * dt
#         self.x_nom[2] += omega_imu * dt
#         self.x_nom[2]  = math.atan2(math.sin(float(self.x_nom[2])),
#                                      math.cos(float(self.x_nom[2])))
#         self.x_nom[4]  = omega_imu

#         # Jacobian of motion model
#         F = np.eye(5)
#         F[0, 2] = -v * math.sin(theta) * dt
#         F[0, 3] =  math.cos(theta) * dt
#         F[1, 2] =  v * math.cos(theta) * dt
#         F[1, 3] =  math.sin(theta) * dt

#         # Scale Q by dt so uncertainty grows per-second, not per-callback
#         self.P = F @ self.P @ F.T + self.Q * dt

#         self.publish_odometry(msg.header.stamp)

#     # ══════════════════════════════════════════════════════════════════════════
#     # CORRECTION STEP  (runs at Wheel Encoder rate)
#     # ══════════════════════════════════════════════════════════════════════════
#     def wheel_odom_callback(self, msg: Odometry):
#         # Mark that we have received at least one valid wheel odom message
#         if not self.wheels_initialized:
#             self.wheels_initialized = True
#             self.get_logger().info("Wheel odometry received — ESKF correction active.")

#         # ── Correction 1: forward velocity (from encoders) ────────────────────
#         v_meas = msg.twist.twist.linear.x
#         H_v    = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
#         self._eskf_update(np.array([[v_meas]]), H_v, self.R_v)

#         # ── Correction 2: angular velocity (from differential drive) ──────────
#         omega_meas = msg.twist.twist.angular.z
#         H_omega    = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
#         self._eskf_update(np.array([[omega_meas]]), H_omega, self.R_omega)

#         # Publish after correction so SLAM gets the freshest estimate
#         self.publish_odometry(msg.header.stamp)

#     # ══════════════════════════════════════════════════════════════════════════
#     # ESKF CORE MATH
#     # ══════════════════════════════════════════════════════════════════════════
#     def _eskf_update(self, z, H, R):
#         """Standard Kalman update with scalar or vector measurement z."""
#         S       = H @ self.P @ H.T + R
#         K       = self.P @ H.T @ np.linalg.inv(S)
#         innov   = z - (H @ self.x_nom)
#         self.x_nom += K @ innov
#         self.x_nom[2] = math.atan2(math.sin(float(self.x_nom[2])),
#                                     math.cos(float(self.x_nom[2])))
#         I      = np.eye(5)
#         self.P = (I - K @ H) @ self.P

#     # ══════════════════════════════════════════════════════════════════════════
#     # PUBLISHING
#     # ══════════════════════════════════════════════════════════════════════════
#     def publish_odometry(self, stamp):
#         theta = float(self.x_nom[2])
#         cy    = math.cos(theta * 0.5)
#         sy    = math.sin(theta * 0.5)

#         # ── /odom message ─────────────────────────────────────────────────────
#         odom = Odometry()
#         odom.header.stamp    = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id  = 'base_footprint'

#         odom.pose.pose.position.x    = float(self.x_nom[0])
#         odom.pose.pose.position.y    = float(self.x_nom[1])
#         odom.pose.pose.position.z    = 0.0
#         odom.pose.pose.orientation.w = cy
#         odom.pose.pose.orientation.z = sy

#         odom.twist.twist.linear.x  = float(self.x_nom[3])
#         odom.twist.twist.angular.z = float(self.x_nom[4])

#         # Covariance from filter's P matrix (position and heading)
#         odom.pose.covariance[0]  = float(self.P[0, 0])   # xx
#         odom.pose.covariance[7]  = float(self.P[1, 1])   # yy
#         odom.pose.covariance[35] = float(self.P[2, 2])   # theta-theta
#         odom.twist.covariance[0] = float(self.P[3, 3])   # vv
#         odom.twist.covariance[35]= float(self.P[4, 4])   # omega-omega

#         self.odom_pub.publish(odom)

#         # ── odom → base_footprint TF ──────────────────────────────────────────
#         t = TransformStamped()
#         t.header.stamp            = self.get_clock().now().to_msg()
#         t.header.frame_id         = 'odom'
#         t.child_frame_id          = 'base_footprint'
#         t.transform.translation.x = float(self.x_nom[0])
#         t.transform.translation.y = float(self.x_nom[1])
#         t.transform.translation.z = 0.0
#         t.transform.rotation.w    = cy
#         t.transform.rotation.z    = sy

#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ErrorStateKalmanFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# class ErrorStateKalmanFilter(Node):
#     def __init__(self):
#         super().__init__('eskf_localization_node')

#         # --- State Initialization ---
#         # State vector: [x, y, theta, v, omega]^T
#         self.x_nom = np.zeros((5, 1))
        
#         # Error State Covariance (P)
#         self.P = np.eye(5) * 0.1 
        
#         # Process Noise (Q) - Uncertainty in our kinematic model
#         self.Q = np.diag([0.01, 0.01, 0.05, 0.1, 0.1])
        
#         # Measurement Noise (R)
#         self.R_odom = np.diag([0.05, 0.05])  # Wheel encoder v, omega noise
#         self.R_imu = np.array([[0.01]])      # IMU yaw rate noise

#         self.last_time = self.get_clock().now()

#         # --- ROS 2 Interfaces ---
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)

#         # High-frequency timer to run the prediction step
#         self.timer = self.create_timer(0.05, self.predict_and_publish)

#         # Variables to hold latest measurements
#         self.latest_v = 0.0
#         self.latest_omega_wheel = 0.0
#         self.latest_omega_imu = 0.0
        
#         self.get_logger().info("ESKF Node Initialized.")

#     def imu_callback(self, msg):
#         # We trust the IMU's Z-axis angular velocity (yaw rate) heavily
#         self.latest_omega_imu = msg.angular_velocity.z
        
#         # Update Step using IMU (1D measurement: omega)
#         H = np.array([[0, 0, 0, 0, 1]])
#         y = np.array([[self.latest_omega_imu]]) - (H @ self.x_nom)
        
#         S = H @ self.P @ H.T + self.R_imu
#         K = self.P @ H.T @ np.linalg.inv(S)
        
#         self.x_nom = self.x_nom + (K @ y)
#         self.P = (np.eye(5) - K @ H) @ self.P

#     def odom_callback(self, msg):
#         # Extract velocity commands from wheel encoders
#         self.latest_v = msg.twist.twist.linear.x
#         self.latest_omega_wheel = msg.twist.twist.angular.z
        
#         # Update Step using Wheel Odometry (2D measurement: v, omega)
#         H = np.array([
#             [0, 0, 0, 1, 0],
#             [0, 0, 0, 0, 1]
#         ])
#         z = np.array([[self.latest_v], [self.latest_omega_wheel]])
#         y = z - (H @ self.x_nom)
        
#         S = H @ self.P @ H.T + self.R_odom
#         K = self.P @ H.T @ np.linalg.inv(S)
        
#         self.x_nom = self.x_nom + (K @ y)
#         self.P = (np.eye(5) - K @ H) @ self.P

#     def predict_and_publish(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time

#         if dt <= 0: return

#         # Extract current states
#         theta = float(self.x_nom[2])
#         v = float(self.x_nom[3])
#         omega = float(self.x_nom[4])

#         # Kinematic Prediction (Non-linear)
#         self.x_nom[0] += v * math.cos(theta + (omega * dt / 2.0)) * dt
#         self.x_nom[1] += v * math.sin(theta + (omega * dt / 2.0)) * dt
#         self.x_nom[2] += omega * dt

#         # Normalize yaw to [-pi, pi]
#         self.x_nom[2] = math.atan2(math.sin(self.x_nom[2]), math.cos(self.x_nom[2]))

#         # Calculate Jacobian (F) for Covariance Prediction
#         F = np.eye(5)
#         F[0, 2] = -v * math.sin(theta) * dt
#         F[0, 3] = math.cos(theta) * dt
#         F[1, 2] = v * math.cos(theta) * dt
#         F[1, 3] = math.sin(theta) * dt
#         F[2, 4] = dt

#         # Predict Covariance
#         self.P = F @ self.P @ F.T + self.Q

#         # --- Publish Odometry ---
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'

#         odom.pose.pose.position.x = float(self.x_nom[0])
#         odom.pose.pose.position.y = float(self.x_nom[1])
#         odom.pose.pose.orientation.z = math.sin(float(self.x_nom[2]) / 2.0)
#         odom.pose.pose.orientation.w = math.cos(float(self.x_nom[2]) / 2.0)

#         odom.twist.twist.linear.x = float(self.x_nom[3])
#         odom.twist.twist.angular.z = float(self.x_nom[4])

#         self.odom_pub.publish(odom)

#         # --- Broadcast TF Tree (odom -> base_footprint) ---
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = float(self.x_nom[0])
#         t.transform.translation.y = float(self.x_nom[1])
#         t.transform.translation.z = 0.0
#         t.transform.rotation.z = math.sin(float(self.x_nom[2]) / 2.0)
#         t.transform.rotation.w = math.cos(float(self.x_nom[2]) / 2.0)

#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ErrorStateKalmanFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()