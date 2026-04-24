import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import socket
import re
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

        self.pub = self.create_publisher(Imu, '/imu/data', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # UDP receiver
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5000))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info("IMU Node with Axis Remap Started")

    # -----------------------------
    def parse(self, text, key):
        m = re.search(rf'{key}:\[(.*?)\]', text)
        if m:
            return [float(x) for x in m.group(1).split(',')]
        return None

    # -----------------------------
    def normalize(self, q):
        n = math.sqrt(sum(x*x for x in q))
        return [x/n for x in q] if n else [1,0,0,0]

    # -----------------------------
    def deg_to_rad(self, g):
        return [x * math.pi / 180.0 for x in g]

    # -----------------------------
    def loop(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            line = data.decode()

            q = self.parse(line, "Q")
            a = self.parse(line, "A")
            g = self.parse(line, "G")

            if not q:
                return

            # Normalize quaternion
            q = self.normalize(q)

            # Convert gyro → rad/s
            if g:
                g = self.deg_to_rad(g)

            now = self.get_clock().now().to_msg()

            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = "imu_link"

            # ---------------- ORIENTATION ----------------
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]

            # ---------------- AXIS REMAP ----------------
            # Adjust these if your physical mounting differs

            if a:
                msg.linear_acceleration.x = a[1]
                msg.linear_acceleration.y = -a[0]
                msg.linear_acceleration.z = a[2]

            if g:
                msg.angular_velocity.x = g[1]
                msg.angular_velocity.y = -g[0]
                msg.angular_velocity.z = g[2]

            # ---------------- COVARIANCE ----------------
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01

            msg.angular_velocity_covariance[0] = 0.02
            msg.angular_velocity_covariance[4] = 0.02
            msg.angular_velocity_covariance[8] = 0.02

            msg.linear_acceleration_covariance[0] = 0.04
            msg.linear_acceleration_covariance[4] = 0.04
            msg.linear_acceleration_covariance[8] = 0.04

            self.pub.publish(msg)

            # ---------------- TF ----------------
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "base_link"
            t.child_frame_id = "imu_link"

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]

            self.tf_broadcaster.sendTransform(t)

        except BlockingIOError:
            pass


# -----------------------------
def main():
    rclpy.init()
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

