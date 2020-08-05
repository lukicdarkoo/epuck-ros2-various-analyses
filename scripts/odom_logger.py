import rclpy
import transforms3d
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Time
from webots_ros2_core.math_utils import quaternion_to_euler


SAMPLE_PERIOD = 0.5


class SimpleMapper(Node):
    def __init__(self, name):
        super().__init__(name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(SAMPLE_PERIOD, self.log_pose)
        self.time_since_start = 0
        self.file = open('odom_log.csv', 'w')

    def log_pose(self):
        rotation = None
        translation = None
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_link', Time(sec=0, nanosec=0))
            rotation = quaternion_to_euler(tf.transform.rotation)[0]
            translation = [tf.transform.translation.x, tf.transform.translation.y]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print('No required transformation found: `{}`'.format(str(e)))
            return

        if translation[0] or translation[1] or rotation or self.time_since_start:
            self.time_since_start += SAMPLE_PERIOD
            output = f'{self.time_since_start:.1f},{translation[0]:.3f},{translation[1]:.3f},{rotation:.2f}\n'
            print(output)
            self.file.write(output)


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = SimpleMapper('epuck_simple_mapper')
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
