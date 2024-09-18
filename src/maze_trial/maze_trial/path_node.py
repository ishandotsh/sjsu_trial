import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.map_sub = self.create_subscription(String, '/map', self.map_callback, 10)
        self.start_sub = self.create_subscription(Point, '/start', self.start_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal', self.goal_callback, 10)
        
        self.path_pub = self.create_publisher(String, '/path', 10)
        
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.map_data = msg.data
        self.get_logger().info('Received map:\n%s' % self.map_data)

    def start_callback(self, msg):
        self.start = msg
        self.get_logger().info(f'Received start: {self.start.x}, {self.start.y}')
    
    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info(f'Received goal: {self.goal.x}, {self.goal.y}')
        self.plan_and_pub()

    def plan_and_pub(self):
        if self.map_data and self.start and self.goal:
            path = ""
            self.path_pub.publish(String(data=path))
            self.get_logger().info('Published path:\n%s' % path)

    
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()