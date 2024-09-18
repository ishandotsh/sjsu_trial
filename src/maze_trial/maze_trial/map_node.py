import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_node')
        self.publisher_ = self.create_publisher(String, '/map', 10)
        self.timer = self.create_timer(2.0, self.publish_map)

        package_share_directory = get_package_share_directory('maze_trial')
        map_file_path = os.path.join(package_share_directory, 'resource', 'map.txt')

        with open(map_file_path, 'r') as file:
            self.map_data = file.read()


    def publish_map(self):
        msg = String()
        msg.data = self.map_data
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing map:\n%s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
