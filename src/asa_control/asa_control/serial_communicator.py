import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import atexit
import math
import time

class SerialCommunicatorNode(Node):
    def __init__(self):
        super().__init__('serial_communicator')
        
        # Initialize Serial communication at 115200 baud
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Update with your port
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.get_logger().info('Node has been started and subscribed to /cmd_vel.')

        self.sync_time()

        self.last_command_time = self.get_clock().now()
        self.timeout_period = 5.0  # seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.data = [0, 0, 0, 0]  # MOTOR ORDER: [BL, BR, FR, FL]
        atexit.register(self.cleanup)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Twist message: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                               f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')
        self.last_command_time = self.get_clock().now()  # Update the last command time
        self.mecanum_control(msg.linear.x, msg.linear.y, msg.angular.z)
        self.send_data_to_arduino()

    def apply_deadzone(self, value, deadzone):
        if abs(value) > deadzone:
            return int(value)
        else:
            return 0

    def mecanum_control(self, inputX, inputY, inputTwist):
        deadzone = 5
        x = -inputY
        y = inputX
        twist = -inputTwist

        inputPower = math.hypot(x, y)
        inputAngle = math.atan2(y, x) - math.pi / 4

        cosi = math.cos(inputAngle)
        sinu = math.sin(inputAngle)

        abs_cos = abs(cosi)
        abs_sin = abs(sinu)

        maximum = max(abs_cos, abs_sin)

        v1 = inputPower * (cosi / maximum) + twist
        v2 = inputPower * (sinu / maximum) - twist
        v3 = inputPower * (sinu / maximum) + twist
        v4 = inputPower * (cosi / maximum) - twist

        if (inputPower + abs(twist)) > 1:
            v1 /= inputPower + abs(twist)
            v2 /= inputPower + abs(twist)
            v3 /= inputPower + abs(twist)
            v4 /= inputPower + abs(twist)

        FL = v1 * 127
        FR = v2 * 127
        BL = v3 * 127
        BR = v4 * 127
        self.data = [self.apply_deadzone(BL, deadzone), self.apply_deadzone(BR, deadzone), self.apply_deadzone(FR, deadzone), self.apply_deadzone(FL, deadzone)]

    def timer_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_command_time).nanoseconds / 1e9 > self.timeout_period:
            self.data = [0, 0, 0, 0]
            self.send_data_to_arduino()

    def send_data_to_arduino(self):
        try:
            message = '<' + ','.join(map(str, self.data)) + '>\n'
            self.serial_port.write(message.encode('utf-8'))
            self.get_logger().info(f'Sending data to Arduino: {message}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')

    def cleanup(self):
        self.get_logger().info('Cleaning up: Sending [0,0,0,0] to Arduino')
        try:
            message = '<0,0,0,0>\n'
            self.serial_port.write(message.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error during cleanup: {e}')

    def sync_time(self):
        while True:
            incoming_message = self.serial_port.readline().decode().strip()
            if incoming_message == "Serial connected. Waiting for TimeSync.":
                self.send_sync()
                break
            elif incoming_message == "":
                self.get_logger().info("Missed invitation to sync, syncing anyways.")
                self.send_sync()
                break
            else:
                self.get_logger().info(f"Received unexpected message: {incoming_message}")

    def send_sync(self):
        current_time = int(time.time())
        sync_message = f"T{current_time}\n"
        self.serial_port.write(sync_message.encode())
        self.get_logger().info(f"Sent time sync message: {sync_message}")

        ack = self.serial_port.readline().decode().strip()
        if ack == "ACK":
            self.time_synced = True
            self.get_logger().info("Time sync successful")
        else:
            self.get_logger().error("Time sync failed")
    


def main(args=None):
    rclpy.init(args=args)
    serial_communicator = SerialCommunicatorNode()
    try:
        rclpy.spin(serial_communicator)
    except KeyboardInterrupt:
        pass
    finally:
        serial_communicator.cleanup()
        serial_communicator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
