# ~/vtol_ctrl_ros2/src/ros2_px4_interface/ros2_px4_interface.py

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorServosTrim, ActuatorMotors, VehicleCommand, ActuatorServos, VehicleLocalPosition, VehicleOdometry, AirspeedWind, Airspeed, VehicleAngularVelocity, VehicleStatus
from std_msgs.msg import Float32, UInt8
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import logging

HZ_RATE = 200.0

class ROS2PX4Interface(Node):
    def __init__(self):
        super().__init__('ros2_px4_interface')
        
        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('ROS2PX4Interface')
        
        # Publishers
        # self.actuator_servos_trim_pub = self.create_publisher(ActuatorServosTrim, 'actuator_servos_trim', QoSProfile(depth=10))
        self.actuator_motors_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))
        self.actuator_servos_pub = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))

        # Subscribers
        

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))
        # self.create_subscription(AirspeedWind, 'fmu/out/airspeed_wind', self.airspeed_wind_callback, QoSProfile(depth=10)) #
        # self.create_subscription(Airspeed, 'fmu/out/airspeed', self.airspeed_callback, QoSProfile(depth=10)) #
        # self.create_subscription(VehicleAngularVelocity, 'fmu/out/vehicle_angular_velocity', self.vehicle_angular_velocity_callback, QoSProfile(depth=10)) #
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSProfile.HISTORY_KEEP_LAST,
            depth=10  # Set depth based on your requirements
        ))

        # Set up a timer for publishing messages at 1/HZ_RATE Hz
        self.timer = self.create_timer(1.0/HZ_RATE, self.publish_messages)

        # Initial command to arm and enable offboard mode
        self.arm_and_enable_offboard()

    def arm_and_enable_offboard(self):
        self.logger.info("Arming vehicle and enabling offboard mode")

        # Create and publish VehicleCommand message to arm the vehicle
        self.arm()

        # Create and publish VehicleCommand message to enable offboard mode
        self.engage_offboard_mode()

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.target_system = 1
        cmd.source_system = 1
        cmd.target_component = 0
        cmd.source_component = 0
        self.vehicle_command_pub.publish(cmd)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def vehicle_local_position_callback(self, msg):
        self.logger.info(f"Received VehicleLocalPosition: x={msg.x}, y={msg.y}, z={msg.z}, vx={msg.vx}, vy={msg.vy}, vz={msg.vz}")

    def vehicle_odometry_callback(self, msg):
        self.logger.info(f"Received VehicleOdometry: position={msg.position}, velocity={msg.velocity}, angular_velocity={msg.angular_velocity}")

    def airspeed_wind_callback(self, msg):
        self.logger.info(f"Received AirspeedWind: tas_innov={msg.tas_innov}, tas_scale_validated={msg.tas_scale_validated}, beta_innov={msg.beta_innov}")

    def airspeed_callback(self, msg):
        self.logger.info(f"Received Airspeed: true_airspeed_m_s={msg.true_airspeed_m_s}, indicated_airspeed_m_s={msg.indicated_airspeed_m_s}")

    def vehicle_angular_velocity_callback(self, msg):
        self.logger.info(f"Received VehicleAngularVelocity: xyz={msg.xyz}, xyz_derivative={msg.xyz_derivative}")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        self.get_logger().info(f"Received VehicleStatus: {self.vehicle_status}")

    def publish_messages(self):
        self.logger.info("Publishing messages")

        # # Publish ActuatorServosTrim message
        # trim_msg = ActuatorServosTrim()
        # trim_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        # trim_msg.trim = [0.0] * 8  # Example trim values
        # self.actuator_servos_trim_pub.publish(trim_msg)
        # self.logger.info("Published ActuatorServosTrim message")

        # Publish ActuatorMotors message
        motors_msg = ActuatorMotors()
        motors_msg.control = [1.0] * 12  # Example motor control values
        self.actuator_motors_pub.publish(motors_msg)
        # self.logger.info("Published ActuatorMotors message")

        # Publish ActuatorServos message
        servos_msg = ActuatorServos()
        servos_msg.control = [1.0] * 8  # Example servo control values
        self.actuator_servos_pub.publish(servos_msg)
        # self.logger.info("Published ActuatorServos message")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2PX4Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
