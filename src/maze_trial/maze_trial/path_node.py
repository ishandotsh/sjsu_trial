import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from collections import deque

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.map_sub = self.create_subscription(String, '/map', self.map_callback, 10)
        self.start_sub = self.create_subscription(Point, '/start', self.start_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal', self.goal_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        self.map_data = []
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.map_data = [list(map(int, row)) for row in msg.data.splitlines()]

    def start_callback(self, msg):
        self.start = msg
        self.get_logger().info(f'Received start: {self.start.x}, {self.start.y}')
    
    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info(f'Received goal: {self.goal.x}, {self.goal.y}')
        self.plan_and_pub()

    def plan_and_pub(self):
        if not (self.map_data and self.start and self.goal):
            return
        
        path = self.bfs(self.map_data, (self.start.x, self.start.y), (self.goal.x, self.goal.y))
        if path:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for (x, y) in path:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published path: {len(path_msg.poses)} points")
        else:
            self.get_logger().info(f"No Path bro")

    def bfs(self, map, start, goal):
        r = len(map)
        c = len(map[0])

        def is_valid(x, y):
            return 0 <= x < r and 0 <= y < c and map[int(x)][int(y)] == 1

        q = deque([start])
        back_link = {start: None}
        dirs = [(0,1), (1,0), (-1,0), (0,-1)]

        while q:
            cur = q.popleft()
            if cur == goal:
                path = []
                while cur:
                    path.append(cur)
                    cur = back_link[cur]
                path.reverse()
                return path
            for d in dirs:
                neighbor = (cur[0] + d[0], cur[1] + d[1])
                if is_valid(*neighbor) and neighbor not in back_link:
                    q.append(neighbor)
                    back_link[neighbor] = cur

        return []


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()