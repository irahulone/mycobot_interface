import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped

from moveit_msgs.srv import GetPositionIK
from tf_transformations import quaternion_from_euler

import math


class MyCobotController(Node):
    def __init__(self):
        super().__init__('mycobot_controller')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(Bool, '/joint_en', self.joint_enable_callback, 10)
        self.create_subscription(Float64MultiArray, '/joint_cmd', self.joint_cmd_callback, 10)
        self.create_subscription(Bool, '/cart_en', self.cart_enable_callback, 10)
        self.create_subscription(Twist, '/cart_cmd', self.cart_cmd_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for MoveIt IK service...")

        self.joint_enable = False
        self.cart_enable = False

        self.joint_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cart_home = [0.1692, -0.06462, 0.1964, math.pi, 0.0, -math.pi / 2]

        self.current_joints = [0.0] * 8  
        self.current_cart = self.cart_home.copy()

        self.latest_joint_cmd = [0.0] * 6  
        self.latest_cart_cmd = [0.0] * 6  

        self.joint_state_ready = True
        self.ik_busy = False

        self.get_logger().info("MyCobot Controller (relative teleop) started")

    def joint_enable_callback(self, msg: Bool):
        self.joint_enable = msg.data
        if self.joint_enable:
            self.cart_enable = False
            self.get_logger().info("Joint mode enabled: moving to joint home")
            self.current_joints[2:] = self.joint_home.copy()
            self.latest_joint_cmd = [0.0] * 6
            self.joint_state_ready = True

    def joint_cmd_callback(self, msg: Float64MultiArray):
        if not self.joint_enable:
            return
        if len(msg.data) < 6:
            self.get_logger().warn("Joint command must contain 6 values")
            return
        self.latest_joint_cmd = [math.degrees(v) for v in msg.data]
        self.current_joints[2:] = [h + d for h, d in zip(self.joint_home, self.latest_joint_cmd)]
        self.joint_state_ready = True

    def cart_enable_callback(self, msg: Bool):
        self.cart_enable = msg.data
        if self.cart_enable:
            self.joint_enable = False
            self.get_logger().info("Cartesian mode enabled: moving to Cartesian home")
            self.current_cart = self.cart_home.copy()
            self.latest_cart_cmd = [0.0] * 6
            self.send_ik_request(self.current_cart)

    def cart_cmd_callback(self, msg: Twist):
        if not self.cart_enable:
            return

        delta = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ]

        self.latest_cart_cmd = delta
        absolute_cart = [h + d for h, d in zip(self.cart_home, self.latest_cart_cmd)]

        if not self.ik_busy:
            self.send_ik_request(absolute_cart)

    def send_ik_request(self, cart):
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
        req.ik_request.robot_state.joint_state.position = [
            math.radians(a) for a in self.current_joints[2:]
        ]

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            res = future.result()
            if len(res.solution.joint_state.name) == 0:
                self.get_logger().warn("IK returned empty solution")
            else:
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
                self.current_joints[2:] = [math.degrees(a) for a in ordered]
                self.joint_state_ready = True

            if self.cart_enable:
                absolute_cart = [h + d for h, d in zip(self.cart_home, self.latest_cart_cmd)]
                if absolute_cart != self.current_cart:
                    self.send_ik_request(absolute_cart)
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
        finally:
            self.ik_busy = False

    def publish_joint_states(self):
        if not self.joint_state_ready:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'g_base_to_joint1',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]
        js.position = [0.0] + [math.radians(a) for a in self.current_joints[2:]]
        self.joint_state_pub.publish(js)


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
