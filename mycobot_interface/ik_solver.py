import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math
import time


class MyCobotIKPublisher(Node):

    def __init__(self):
        super().__init__("mycobot_iksolver_publish")

        self.ik_pub = self.create_publisher(Float64MultiArray, '/joint_cmd', 10)

        self.create_subscription(Twist, '/cart_cmd', self.cart_cmd_callback, 10)

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service")

        self.cart_home = [0.1692, -0.06462, 0.1964, math.pi, 0.0, -math.pi/2]
        self.current_cart = self.cart_home.copy()

        self.target_joints = [0.0] * 6

        self.ik_busy = False
        self.last_ik_time = 0.0
        self.ik_interval = 0.1  # 10 Hz

        self.get_logger().info("IK Solver Publisher Node Started")

    def is_significant_change(self, a, b, tol=1e-4):
        return any(abs(x - y) > tol for x, y in zip(a, b))

    def cart_cmd_callback(self, msg: Twist):

        now = time.time()
        if now - self.last_ik_time < self.ik_interval:
            return

        self.last_ik_time = now

        delta = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ]

        target_cart = [
            self.cart_home[0] + delta[0],
            self.cart_home[1] + delta[1],
            self.cart_home[2] + delta[2],
            self.cart_home[3] + delta[3],
            self.cart_home[4] + delta[4],
            self.cart_home[5] + delta[5],
        ]

        if self.is_significant_change(target_cart, self.current_cart):
            self.send_ik_request(target_cart)

    def send_ik_request(self, cart):

        if self.ik_busy:
            return

        self.ik_busy = True

        req = GetPositionIK.Request()

        pose = PoseStamped()
        pose.header.frame_id = "g_base"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = cart[0]
        pose.pose.position.y = cart[1]
        pose.pose.position.z = cart[2]

        q = quaternion_from_euler(cart[3], cart[4], cart[5])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        req.ik_request.group_name = "arm_group"
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1

        req.ik_request.robot_state.joint_state.name = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

        req.ik_request.robot_state.joint_state.position = self.target_joints

        self.get_logger().info(f"IK request sent: {cart}")

        future = self.ik_client.call_async(req)
        future.add_done_callback(
            lambda f: self.ik_response_callback(f, cart)
        )

    def ik_response_callback(self, future, requested_cart):

        try:
            res = future.result()

            if not res.solution.joint_state.position:
                self.get_logger().warn("IK failed: empty solution!!!")
                return

            joint_map = dict(zip(
                res.solution.joint_state.name,
                res.solution.joint_state.position
            ))

            ordered = [
                joint_map['joint2_to_joint1'],
                joint_map['joint3_to_joint2'],
                joint_map['joint4_to_joint3'],
                joint_map['joint5_to_joint4'],
                joint_map['joint6_to_joint5'],
                joint_map['joint6output_to_joint6']
            ]

            self.target_joints = ordered
            self.current_cart = requested_cart

            msg = Float64MultiArray()
            msg.data = self.target_joints

            self.ik_pub.publish(msg)
            self.get_logger().info(f"publishing joint angles (in rad): {msg.data}")

        except Exception as e:
            self.get_logger().error(f"IK error: {e}")

        finally:
            self.ik_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotIKPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IK solver")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
