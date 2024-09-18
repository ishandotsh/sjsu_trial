import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import tkinter as tk
from tkinter import messagebox
import threading

class GuiNode(Node):
    def __init__(self):
        super().__init__("gui")
        self.start_pub = self.create_publisher(Point, '/start', 10)
        self.goal_pub = self.create_publisher(Point, '/goal', 10)
        self.map_sub = self.create_subscription(String, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(String, '/path', self.path_callback, 10)

        self.map_data = []
        self.path_data = ""

        self.init_gui()
    
    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Maze Planner Trial")

        tk.Label(self.root, text="Start X").grid(row=0)
        tk.Label(self.root, text="Start Y").grid(row=0)
        self.start_x_entry = tk.Entry(self.root)
        self.start_y_entry = tk.Entry(self.root)
        self.start_x_entry.grid(row=0, column=1)
        self.start_y_entry.grid(row=0, column=2)

        tk.Label(self.root, text="Goal X").grid(row=1)
        tk.Label(self.root, text="Goal Y").grid(row=1)
        self.goal_x_entry = tk.Entry(self.root)
        self.goal_y_entry = tk.Entry(self.root)
        self.goal_x_entry.grid(row=1, column=1)
        self.goal_y_entry.grid(row=1, column=2)

        self.publish_button = tk.Button(self.root, text="Publish", command=self.publish_positions)
        self.publish_button.grid(row=4, column=1)

        self.canvas = tk.Canvas(self.root, width=400, height=400)
        self.canvas.grid(row=4, column=1)

        self.map_label = tk.Label(self.root, text="Map")
        self.map_label.grid(row=3)

        self.root.after(100, self.update_gui)


    def publish_positions(self):
        start_x = float(self.start_x_entry.get())
        start_y = float(self.start_y_entry.get())
        goal_x =  float(self.goal_x_entry.get())
        goal_y =  float(self.goal_y_entry.get())

        if not (start_x and start_y and goal_x and goal_y):
            messagebox.showerror("Input Error", "Enter all coordinates")
            return

        start_msg = Point()
        start_msg.x, start_msg.y = start_x, start_y
        self.start_pub.publish(start_msg)
        self.get_logger().info(f"Published start: {start_x},{start_y}")

        goal_msg = Point()
        goal_msg.x, goal_msg.y = goal_x, goal_y
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published goal: {goal_x},{goal_y}")

    def map_callback(self, msg):
        self.map_data = [list(row) for row in msg.data.splitlines()]
        self.get_logger().info('Received map')
        self.draw_map()

    def path_callback(self, msg):
        self.path_data = msg.data
        self.get_logger().info('Received path')

    def update_gui(self):
        self.root.after(100, self.update_gui)

    def draw_map(self):
        self.canvas.delete("all")
        cell_size = 40
        for row_index, row in enumerate(self.map_data):
            for col_index, cell in enumerate(row):
                color = 'blue' if cell == '1' else 'black'
                
                x1 = col_index * cell_size
                y1 = row_index * cell_size
                x2 = x1 + cell_size
                y2 = y1 + cell_size

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='white')

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = GuiNode()

    ros_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,))
    ros_thread.start()

    gui_node.root.mainloop()

    gui_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()



if __name__ == '__main__':
    main()
