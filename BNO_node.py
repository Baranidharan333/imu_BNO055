import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

import socket
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)
        self.yaw_pub = self.create_publisher(PoseStamped, '/imu/yaw', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5000))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.loop)

    # -------------------------
    def parse(self, text, key):
        try:
            start = text.index(f"{key}:[") + len(key) + 2
            end = text.index("]", start)
            return [float(x) for x in text[start:end].split(",")]
        except:
            return None

    def normalize(self, q):
        n = math.sqrt(sum(x*x for x in q))
        return [x/n for x in q] if n else [1,0,0,0]

    # -------------------------
    def loop(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            line = data.decode()

            q = self.parse(line, "Q")
            a = self.parse(line, "A")
            g = self.parse(line, "G")
            m = self.parse(line, "M")

            if not q:
                return

            q = self.normalize(q)

            # gyro → rad/s
            if g:
                g = [x * math.pi / 180 for x in g]

            now = self.get_clock().now().to_msg()

            # ---------------- IMU ----------------
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "imu_link"

            imu.orientation.w = q[0]
            imu.orientation.x = q[1]
            imu.orientation.y = q[2]
            imu.orientation.z = q[3]

            # Axis remap (match your earlier setup)
            if g:
                imu.angular_velocity.x = g[1]
                imu.angular_velocity.y = -g[0]
                imu.angular_velocity.z = g[2]

            if a:
                ax = a[1]
                ay = -a[0]
                az = a[2]

                imu.linear_acceleration.x = ax
                imu.linear_acceleration.y = ay
                imu.linear_acceleration.z = az

            # Covariance (important)
            imu.orientation_covariance[0] = 0.01
            imu.orientation_covariance[4] = 0.01
            imu.orientation_covariance[8] = 0.01

            imu.angular_velocity_covariance[0] = 0.02
            imu.angular_velocity_covariance[4] = 0.02
            imu.angular_velocity_covariance[8] = 0.02

            imu.linear_acceleration_covariance[0] = 0.04
            imu.linear_acceleration_covariance[4] = 0.04
            imu.linear_acceleration_covariance[8] = 0.04

            self.imu_pub.publish(imu)

            # ---------------- TF (IMU) ----------------
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "base_link"
            t.child_frame_id = "imu_link"

            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]

            self.tf_broadcaster.sendTransform(t)

            # ---------------- YAW (PLANAR, FIXED) ----------------
            if m and a:
                # Axis remap (must match IMU)
                ax = a[1]
                ay = -a[0]
                az = a[2]

                mx = m[1]
                my = -m[0]
                mz = m[2]

                # Normalize accel
                norm = math.sqrt(ax*ax + ay*ay + az*az)
                if norm == 0:
                    return
                ax /= norm
                ay /= norm
                az /= norm

                # Roll & Pitch
                roll  = math.atan2(ay, az)
                pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

                # Tilt compensation
                mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
                my2 = (mx * math.sin(roll) * math.sin(pitch) +
                       my * math.cos(roll) -
                       mz * math.sin(roll) * math.cos(pitch))

                yaw = math.atan2(-my2, mx2)

                # 🔥 PLANAR quaternion (Z only)
                qw = math.cos(yaw / 2)
                qx = 0.0
                qy = 0.0
                qz = math.sin(yaw / 2)

                pose = PoseStamped()
                pose.header.stamp = now
                pose.header.frame_id = "base_link"

                pose.pose.orientation.w = qw
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz

                self.yaw_pub.publish(pose)

                # TF for yaw
                t2 = TransformStamped()
                t2.header.stamp = now
                t2.header.frame_id = "base_link"
                t2.child_frame_id = "yaw_link"

                t2.transform.rotation.w = qw
                t2.transform.rotation.x = qx
                t2.transform.rotation.y = qy
                t2.transform.rotation.z = qz

                self.tf_broadcaster.sendTransform(t2)

        except BlockingIOError:
            pass


def main():
    rclpy.init()
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
