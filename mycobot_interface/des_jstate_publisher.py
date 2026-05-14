import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_cmd_publisher')

        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_cmd', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        #self.sent = False

    def publish_command(self):
        #if self.sent:
            #return

        # User 6 Joint Angle inputs. Enter in degrees.
        user_deg = [10, 30, -30, 20, 5, 0]

        cmd_rad = [math.radians(d) for d in user_deg]

        cmd_msg = Float64MultiArray()
        cmd_msg.data = cmd_rad
        self.joint_cmd_pub.publish(cmd_msg)

        self.get_logger().info(f"Published Desired Joint Angles: {user_deg}")

        #self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
