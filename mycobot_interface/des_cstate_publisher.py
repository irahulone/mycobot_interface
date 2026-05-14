import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CartCommandPublisher(Node):

    def __init__(self):
        super().__init__('cart_cmd_publisher')

        self.cart_pub = self.create_publisher(Twist, '/cart_cmd', 10)

        self.timer = self.create_timer(0.1, self.publish_command)
        #self.sent = False

    def publish_command(self):
        #if self.sent:
            #return
        
        # User Cartesian X,Y,Z inputs. Enter in mms.
        # Note: These inputs values represent the positions which are relative to the defined home cartesian position. 
        x = -30
        y = 20
        z = 45

        msg = Twist()

        msg.linear.x = x/1000
        msg.linear.y = y/1000
        msg.linear.z = z/1000

        # Fixed end-effector orientation
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.cart_pub.publish(msg)

        self.get_logger().info(f"Published Desired Cartesian Position: X:{x}, Y:{y}, Z:{z}")

        #self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = CartCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
