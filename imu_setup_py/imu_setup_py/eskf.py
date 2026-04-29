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
        self.create_timer(0.05, self._heartbeat_tf)
        self.last_imu_time = self.get_clock().now()
        self.create_timer(1.0, self._check_imu_health)

    def _heartbeat_tf(self):
        # Publish current state even if no new sensor data came in
        now = self.get_clock().now().to_msg()
        self.publish_odometry(now)
        
    def _check_imu_health(self):
        age = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
        if age > 2.0:
            self.get_logger().warn(f"IMU data stale for {age:.1f}s — running encoder-only mode")

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

        #self.publish_odometry(msg.header.stamp)

    # =========================================================================
    # CORRECTION STEP 1: WHEEL ENCODERS
    # =========================================================================
    # def encoder_callback(self, msg: Odometry):
    #     # Cache for ZUPT gating in IMU callback
    #     self._encoder_velocity = msg.twist.twist.linear.x

    #     z = np.array([[self._encoder_velocity]])
    #     H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
    #     self.eskf_update(z, H, self.R_enc)
    def encoder_callback(self, msg: Odometry):
        self._encoder_velocity = msg.twist.twist.linear.x
        
        # Correct velocity
        z_v = np.array([[self._encoder_velocity]])
        H_v = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        self.eskf_update(z_v, H_v, self.R_enc)
        
        # ALSO correct position directly from encoder odom
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        
        z_pos = np.array([[msg.pose.pose.position.x],
                        [msg.pose.pose.position.y],
                        [yaw]])
        H_pos = np.array([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0]])
        R_pos = np.diag([0.05, 0.05, 0.1])
        innovation = z_pos - (H_pos @ self.x_nom)
        innovation[2,0] = math.atan2(math.sin(innovation[2,0]), math.cos(innovation[2,0]))
        self.eskf_update_with_innovation(innovation, H_pos, R_pos)

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