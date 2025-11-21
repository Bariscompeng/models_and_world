import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        
        self.error_sub = self.create_subscription(
            Float32,
            'line_error',
            self.error_callback,
            10
        )

        
        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/M100/cmd_vel',   
            10
        )

        
        self.Kp = 0.8

       
        self.forward_speed = 0.3

        
        self.current_error = 0.0
        self.last_error_time = self.get_clock().now()

        
        self.lost_line_timeout = 0.5  

        
        self.control_timer = self.create_timer(0.05, self.control_loop)  

        self.get_logger().info("Line follower node started with timeout logic.")

    def error_callback(self, msg: Float32):
        
        self.current_error = msg.data
        self.last_error_time = self.get_clock().now()

    def control_loop(self):
        
        now = self.get_clock().now()
        dt = (now - self.last_error_time).nanoseconds / 1e9  

        cmd = Twist()

        if dt < self.lost_line_timeout:
           
            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.Kp * self.current_error
        else:
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

