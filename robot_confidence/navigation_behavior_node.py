import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class NavigationBehavior(Node):
    def __init__(self):
        super().__init__('navigation_behavior')

        self.mode = 'SAFE'

        self.declare_parameter('v_aggressive', 0.8)
        self.declare_parameter('v_conservative', 0.4)
        self.declare_parameter('v_safe', 0.1)

        self.create_subscription(String, 'navigation_mode', self.mode_callback, 10)

        self.pub = self.create_publisher(Float32, 'cmd_vel_limit', 10)

        self.timer = self.create_timer(0.1, self.publish_speed)

        self.get_logger().info("NavigationBehavior started")

    def mode_callback(self, msg):
        self.mode = msg.data

    def publish_speed(self):
        v_aggr = self.get_parameter('v_aggressive').value
        v_cons = self.get_parameter('v_conservative').value
        v_safe = self.get_parameter('v_safe').value

        if self.mode == 'AGGRESSIVE':
            v = v_aggr
        elif self.mode == 'CONSERVATIVE':
            v = v_cons
        else:
            v = v_safe

        msg = Float32()
        msg.data = v
        self.pub.publish(msg)

        self.get_logger().info(f"Mode={self.mode} limit={v}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
