import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import serial
import math
import time

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Publishers for odom and imu
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.temperature_publisher = self.create_publisher(Float32, '/temperature', 10)
        
        self.create_timer(0.1, self.read_serial_data)        

    def read_serial_data(self):
        try:
            # Read from serial
            incoming_message = self.serial_port.readline().decode().strip()

            if incoming_message:
                self.get_logger().info(f'Received message: {incoming_message}')
                
                # Validate message hash
                if self.validate_message(incoming_message):
                    # Parse the message and publish
                    self.parse_and_publish(incoming_message)
                else:
                    self.get_logger().error("Message hash validation failed.")
            else:
                self.get_logger().info("No data received.")
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')

    def validate_message(self, message):
        try:    
            data, hash_str = message.rsplit(',', 1)
            received_hash = int(hash_str)

            calculated_hash = self.calculate_hash(data)

            self.get_logger().info(f"data: {data} | received hash: {received_hash} | calculated hash: {calculated_hash}")
            
            return received_hash == calculated_hash
        except ValueError as e:
            self.get_logger().error(f"Error parsing message: {message}. Exception: {e}")
            return False

    def calculate_hash(self, data):
        hash = 0
        for char in data:
            hash = ((hash << 5) + hash) + ord(char)
            hash &= 0xFFFF  # Ensure hash is within 16-bit range
        return hash
    
    def parse_and_publish(self, message):
        try:
            # Split data fields
            parts = message.split(',')
            data_dict = {}
            
            self.get_logger().info(f"Parts after split: {parts}")

            for part in parts:
                if '=' not in part:
                    self.get_logger().warning(f'Skipping part: {part}')
                    continue
                try:
                    key, value = part.split('=')
                    data_dict[key] = float(value)
                except ValueError:
                    self.get_logger().error(f"Failed to parse: {part}")
                    continue
            self.get_logger().info(f"Parsed Data: {data_dict}")

            # Publish odometry message
            self.publish_odometry(data_dict)

            # Publish IMU message
            self.publish_imu(data_dict)

            # Publish temperature data
            self.publish_temperature(data_dict)

        except Exception as e:
            self.get_logger().error(f"Failed to parse and publish data: {e}")

    def publish_odometry(self, data):
        odom_msg = Odometry()
        
        # Assuming the timestamp is in ISO format, we'll convert it here
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        odom_msg.twist.twist.linear.x = (data["m1"] + data["m2"] + data["m3"] + data["m4"]) / 4.0
        odom_msg.twist.twist.angular.z = (data["m1"] - data["m2"] + data["m3"] - data["m4"]) / 4.0

        self.odom_publisher.publish(odom_msg)

    def publish_imu(self, data):
        # Create and fill the IMU message
        imu_msg = Imu()
        
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Populate acceleration and rotational velocity from data
        imu_msg.linear_acceleration.x = data["acc_x"]
        imu_msg.linear_acceleration.y = data["acc_y"]
        imu_msg.linear_acceleration.z = data["acc_z"]

        imu_msg.angular_velocity.x = data["rot_x"]
        imu_msg.angular_velocity.y = data["rot_y"]
        imu_msg.angular_velocity.z = data["rot_z"]

        # Assuming you have no orientation (quaternion) data, we'll leave it as 0s
        # Optionally you can fill orientation from another source or use a filter

        self.imu_publisher.publish(imu_msg)

    def publish_temperature(self, data):
        # Publish temperature data
        temp_msg = Float32()
        temp_msg.data = data["temp_c"]
        self.temperature_publisher.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)
    serial_data_parser = SerialReaderNode()
    try:
        rclpy.spin(serial_data_parser)
    except KeyboardInterrupt:
        pass
    finally:
        serial_data_parser.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()