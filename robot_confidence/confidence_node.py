import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class ConfidenceEstimator(Node):
    def __init__(self):
        super().__init__('confidence_estimator')

        self.declare_parameter('w_vo', 0.3)
        self.declare_parameter('w_slam', 0.4)
        self.declare_parameter('w_drift', 0.2)
        self.declare_parameter('w_noise', 0.1)

        self.declare_parameter('th_aggressive', 0.7)
        self.declare_parameter('th_conservative', 0.4)

        self.vo_uncertainty = None
        self.slam_quality = None
        self.drift_norm = None
        self.sensor_noise = None

        self.create_subscription(Float32, 'vo_uncertainty', self.vo_callback, 10)
        self.create_subscription(Float32, 'slam_quality', self.slam_callback, 10)
        self.create_subscription(Float32, 'drift_norm', self.drift_callback, 10)
        self.create_subscription(Float32, 'sensor_noise', self.noise_callback, 10)

        self.conf_pub = self.create_publisher(Float32, 'robot_confidence', 10)
        self.mode_pub = self.create_publisher(String, 'navigation_mode', 10)

        self.timer = self.create_timer(0.1, self.update_confidence)

        self.get_logger().info('ConfidenceEstimator node started.')

    def vo_callback(self, msg):
        self.vo_uncertainty = self._clamp(msg.data)

    def slam_callback(self, msg):
        self.slam_quality = self._clamp(msg.data)

    def drift_callback(self, msg):
        self.drift_norm = self._clamp(msg.data)

    def noise_callback(self, msg):
        self.sensor_noise = self._clamp(msg.data)

    def _clamp(self, x):
        return max(0.0, min(1.0, x))

    def update_confidence(self):
        if None in (
            self.vo_uncertainty,
            self.slam_quality,
            self.drift_norm,
            self.sensor_noise,
        ):
            return

        w_vo = self.get_parameter('w_vo').value
        w_slam = self.get_parameter('w_slam').value
        w_drift = self.get_parameter('w_drift').value
        w_noise = self.get_parameter('w_noise').value

        th_aggr = self.get_parameter('th_aggressive').value
        th_cons = self.get_parameter('th_conservative').value

        w_sum = w_vo + w_slam + w_drift + w_noise
        if w_sum == 0:
            return

        w_vo /= w_sum
        w_slam /= w_sum
        w_drift /= w_sum
        w_noise /= w_sum

        c = (
            w_vo * (1 - self.vo_uncertainty)
            + w_slam * self.slam_quality
            + w_drift * (1 - self.drift_norm)
            + w_noise * (1 - self.sensor_noise)
        )
        c = self._clamp(c)

        conf_msg = Float32()
        conf_msg.data = c
        self.conf_pub.publish(conf_msg)

        mode_msg = String()
        if c >= th_aggr:
            mode_msg.data = 'AGGRESSIVE'
        elif c >= th_cons:
            mode_msg.data = 'CONSERVATIVE'
        else:
            mode_msg.data = 'SAFE'

        self.mode_pub.publish(mode_msg)

        self.get_logger().info(
            f'C={c:.3f} mode={mode_msg.data}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ConfidenceEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
