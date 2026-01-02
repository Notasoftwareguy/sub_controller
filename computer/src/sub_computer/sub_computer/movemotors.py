import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

from custom_interfaces.srv import CommandResponse

class MoveMotors(Node):
    def __init__(self):
        ### Configuration ###
        self.command_timeout_s = 100.0  # seconds

        ### Surface data listener setup ###
        super().__init__('move_motors')
        self.get_logger().info('MoveMotors node has been started.')   
        self.subscription = self.create_subscription(
            String,
            'surface_data',
            self.listener_callback,
            10)
        
        ### Microcontroller service setup ###
        self.cli = self.create_client(CommandResponse, 'talk_to_micro')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('talk_to_micro service not available, waiting again...')

        ### Command sender setup ###
        timer_period = 1/50  # seconds
        self.timer = self.create_timer(timer_period, self.send_motor_commands)
        
        self.use_autopilot = False
        self.motor_command = [0,0,0]
        self.command_updated = False
        self.request = CommandResponse.Request()

    def listener_callback(self, msg):
        commands = msg.data.split(";")
        for command in commands:
            if not command.startswith("M "):
                continue
            self.motor_command = list(map(int, command[2:].split()))
            self.command_updated = True
            self.get_logger().info(f'Received motor command: "{self.motor_command}"')
    
    def send_motor_commands(self):
        if self.command_updated:
            self.command_updated = False
            self.request.command = f'1 {self.motor_command[0]} {self.motor_command[1]} {self.motor_command[2]}'
            self.request.expect_response = False
            _ = self.cli.call_async(self.request)
            self.get_logger().info(f'Sent motor command: "{self.motor_command}"')

def main(args=None):
    rclpy.init(args=args)
    node = MoveMotors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()