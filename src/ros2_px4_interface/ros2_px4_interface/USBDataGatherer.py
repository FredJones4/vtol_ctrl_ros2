#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class USBDataGatherer(Node):

    def __init__(self):
        super().__init__('usb_data_gatherer')
        
        # Initialize subscriber and publisher
        self.write_sub = self.create_subscription(
            String,
            'write',
            self.write_callback,
            10)
        
        self.read_pub = self.create_publisher(String, 'airspeed_bridge', 10)

        # Serial port initialization
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM0'  # Change to your specific port
        self.ser.baudrate = 9600        # Set baudrate
        self.ser.timeout = 1            # Set timeout
        
        try:
            self.ser.open()
            self.get_logger().info("Serial Port initialized")
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open port: {str(e)}")
            self.destroy_node()
            return

        # Timer to check for data at regular intervals
        self.timer = self.create_timer(0.2, self.read_from_serial)  # 5Hz loop rate

    def write_callback(self, msg):
        self.get_logger().info(f"Writing to serial port: {msg.data}")
        self.ser.write(msg.data.encode())

    def read_from_serial(self):
        if self.ser.in_waiting > 0:
            self.get_logger().info("Reading from serial port")
            data = self.ser.read(self.ser.in_waiting).decode()
            self.get_logger().info(f"Read: {data}")
            result = String()
            result.data = data
            self.read_pub.publish(result)

def main(args=None):
    rclpy.init(args=args)
    
    serial_example_node = USBDataGatherer()
    
    try:
        rclpy.spin(serial_example_node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    serial_example_node.ser.close()
    serial_example_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
