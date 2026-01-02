import rclpy
from rclpy.node import Node
import serial
import time
from custom_interfaces.srv import CommandResponse


### Command specification ###
# Command must be a string of numbers between 0 and 253 separated by spaces.

class TalkToMicro(Node):
    def __init__(self):
        super().__init__('talk_to_micro')
        
        connected = False
        while not connected:
            try:
                self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
                connected = True
            except:
                self.get_logger().info('Microcontroller failed to connect, retrying in 2 seconds...')
                time.sleep(2)
        self.microcontroller_health_check()

        self.get_logger().info('TalkToMicro node has been started.')
        self.srv = self.create_service(CommandResponse, 'talk_to_micro', self.handle_command)
        
        self.timer = self.create_timer(10, self.microcontroller_health_check)

    def handle_command(self, request, response):
        command = request.command
        expect_response = request.expect_response
        self.get_logger().info(f'Received command request: "{command}"')
        if self.microcontroller_connected:
            # self.arduino.reset_input_buffer() # clear input buffer just in case
            response.response = self.communicate_with_micro(command, expect_response)
        else:
            response.response = ""

        return response

    def communicate_with_micro(self, command, expect_response):
        parsed_command = list(map(int, command.split()))
        self.get_logger().info(f'Sending command to microcontroller: "{parsed_command}"')
        self.send_packet(parsed_command)
        
        response = ""
        if expect_response:
            response = self.arduino.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received from microcontroller: "{response}"')
        
        return response

    def send_packet(self, numbers):
        # 1. Start Marker (254)
        packet = [254]

        # 2. Count (Length of the list)
        packet.append(len(numbers))
        packet.extend(numbers)

        # Send as raw bytes
        self.arduino.write(bytearray(packet))

    def microcontroller_health_check(self):
        self.get_logger().info(f'Checking on microcontroller health...')
        response = self.communicate_with_micro("0", expect_response=True)
        if response == "1":
            self.microcontroller_connected = True
        else:
            self.microcontroller_connected = False


def main(args=None):
    rclpy.init(args=args)
    node = TalkToMicro()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()