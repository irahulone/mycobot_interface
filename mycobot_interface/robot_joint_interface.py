import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from pymycobot import MyCobot280
import numpy as np
import math
import time
import threading

class MyCobotController(Node):

    def __init__(self):
        super().__init__('mycobot_hard_control')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
     
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1000000)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.mc = MyCobot280(port, baud)
        
        time.sleep(0.2)
        self.mc.set_fresh_mode(1)
        time.sleep(0.1)

        self.mc_lock = threading.Lock()

        self.joint_enable = True
        self.cart_enable = False

        self.ini_mov = True
        self.home_coords = [0, 0, 0, 0, 0, 0]
        self.current_joints = [0.0] * 6

        self.last_cart_cmd_time = 0
        self.cart_cmd_interval = 0.2  # 5 Hz

        self.create_subscription(Float64MultiArray, '/joint_cmd', self.joint_cmd_callback, 10)

        self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        self.get_logger().info("MyCobot joint interface controller started")

    def safe_get_angles(self):
        for _ in range(3):
            with self.mc_lock:
                angles = self.mc.get_angles()

            if isinstance(angles, (list, tuple)) and len(angles) == 6:
                return angles

            time.sleep(0.01)

        self.get_logger().warn(f"Invalid angles received after retries: {angles}")
        return None

    def safe_send_angles(self, angles, speed):
        with self.mc_lock:
            self.mc.send_angles(angles, speed)
        time.sleep(0.02)

    def safe_send_coords(self, coords, speed):
        with self.mc_lock:
            self.mc.send_coords(coords, speed)
        time.sleep(0.02)

    def safe_get_coords(self):
        with self.mc_lock:
            return self.mc.get_coords()

    def joint_cmd_callback(self, msg: Float64MultiArray):
        if self.joint_enable and not self.cart_enable:

            if not msg.data:
                return

            angles_deg = np.degrees(msg.data).tolist()

            if len(angles_deg) != 6:
                self.get_logger().warn(f"Invalid joint command length: {angles_deg}")
                return

            self.safe_send_angles(angles_deg, 50)

            self.current_joints = angles_deg
            self.ini_mov = True

            self.get_logger().info(f"Sent joint angles: {angles_deg}")

    def cart_cmd_callback(self, msg: Twist):
        
        if self.cart_enable and not self.joint_enable:
            now = time.time()
            if now - self.last_cart_cmd_time < self.cart_cmd_interval:
                return
            self.last_cart_cmd_time = now

            if self.ini_mov:
                #coords = [0.1692*1000, -0.06462*1000, 0.1964*1000, math.degrees(math.pi), 0.0, math.degrees(-math.pi / 2)]
                #self.safe_send_coords(coords, 50)
                self.safe_send_angles([0, 0, -90, 0, 0, 0], 50)
                time.sleep(2)

                coords = self.safe_get_coords()
                if coords is None or len(coords) != 6:
                    self.get_logger().warn("Failed to get initial coordinates")
                    return

                self.home_coords = coords
                self.ini_mov = False

            cart_vals = [
                msg.linear.x * 1000 + self.home_coords[0],
                msg.linear.y * 1000 + self.home_coords[1],
                msg.linear.z * 1000 + self.home_coords[2],
                math.degrees(msg.angular.x) + self.home_coords[3],
                math.degrees(msg.angular.y) + self.home_coords[4],
                math.degrees(msg.angular.z) + self.home_coords[5]
            ]

            for i in range(3, 6):
                cart_vals[i] = ((cart_vals[i] + 180) % 360) - 180

            self.safe_send_coords(cart_vals, 50)

            self.get_logger().info(f"Sent Cartesian coords: {cart_vals}")

    def publish_joint_states(self):

        try:
            angles = self.safe_get_angles()

            if angles is None:
                return

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()

            js.name = [
                'joint2_to_joint1',
                'joint3_to_joint2',
                'joint4_to_joint3',
                'joint5_to_joint4',
                'joint6_to_joint5',
                'joint6output_to_joint6'
            ]

            js.position = [math.radians(a) for a in angles]

            self.joint_state_pub.publish(js)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish joint_states: {e}")


def main(args=None):

    rclpy.init(args=args)

    node = MyCobotController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
