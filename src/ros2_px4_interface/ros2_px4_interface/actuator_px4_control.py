import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (ActuatorMotors, ActuatorServos, VehicleCommand, OffboardControlMode)
import math

class ActuatorPX4Control(Node):
    def __init__(self):
        super().__init__('actuator_px4_control')

        # QoS profile for publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create publishers with the specified QoS profiles
        self.actuator_motors_pub = self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            qos_profile
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        self.actuator_servos_pub = self.create_publisher(
            ActuatorServos,
            '/fmu/in/actuator_servos',
            qos_profile
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

    def publish_actuator_motors(self, motors_msg):
        self.actuator_motors_pub.publish(motors_msg)
        self.get_logger().info('Published ActuatorMotors message')
    def publish_actuator_servos(self, servos_msg):
        self.actuator_servos_pub.publish(servos_msg)
        self.get_logger().info('Published ActuatorServos message')

    def publish_vehicle_command(self, command_msg):
        self.vehicle_command_pub.publish(command_msg)
        self.get_logger().info('Published VehicleCommand message')
        self.get_logger().info('Published ActuatorServos message')

    def publish_offboard_control_mode(self, control_mode_msg):
        self.offboard_control_mode_pub.publish(control_mode_msg)
        self.get_logger().info('Published OffboardControlMode message')



def main(args=None):
    rclpy.init(args=args)
    node = ActuatorPX4Control()

    # Example usage: publishing messages
    # Note: Actual message instances should be created with valid data
    motors_msg = ActuatorMotors()  # Populate with relevant data
    motors_msg.control = [1.0,      1.0,        1.0,        1.0,    1.0,     math.nan, 
                          math.nan, math.nan, math.nan, math.nan, math.nan, math.nan]
    # command_msg = VehicleCommand()  # Populate with relevant data
    servos_msg = ActuatorServos()   # Populate with relevant data
    servos_msg.control = [1.0,      1.0,        1.0,       math.nan,     math.nan,     math.nan, 
                          math.nan, math.nan]
    # control_mode_msg = OffboardControlMode()  # Populate with relevant data

    node.publish_actuator_motors(motors_msg)
    # node.publish_vehicle_command(command_msg)
    node.publish_actuator_servos(servos_msg)
    # node.publish_offboard_control_mode(control_mode_msg)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
