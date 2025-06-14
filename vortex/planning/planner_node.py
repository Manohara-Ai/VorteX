import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.get_logger().info("Planner node has started autonomous navigation")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_msg)

    def send_velocity_msg(self):
        msg = Twist()
        msg.linear.x = 2.0  
        msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received Ctrl+C, shutting down...")
    finally:
        if rclpy.ok():
            node_name = node.get_name() 
            node.destroy_node()
            rclpy.shutdown()
            print(f"{node_name} shutdown complete.")

if __name__ == '__main__':
    main()
