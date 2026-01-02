import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import random # debug



class SurfacePublisher(Node):
    def __init__(self):
        super().__init__('surface_publisher')
        self.get_logger().info('SurfacePublisher node has been started.')   
        self.publisher_ = self.create_publisher(String, 'surface_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_surface_data)

    def get_surface_data(self):
        msg = String()
        msg.data = f'M {random.randint(0,100)} {random.randint(0,100)} {random.randint(0,100)};'  # debug
        return msg
    
    def publish_surface_data(self):
        msg = self.get_surface_data()
        if msg is not None:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Received surface data: "{msg.data}"')
        

def main(args=None):
    rclpy.init(args=args)
    node = SurfacePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()