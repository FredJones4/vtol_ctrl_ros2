import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorServosTrim, ActuatorMotors, VehicleCommand, ActuatorServos, VehicleLocalPosition, VehicleOdometry, AirspeedWind, Airspeed, VehicleAngularVelocity, VehicleStatus, VehicleCommandAck, OffboardControlMode
from std_msgs.msg import Float32, UInt8
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import logging

HZ_RATE = 200.0
OFFBOARD_CONTROL_MODE_RATE = 2.0  # Hz

class ROS2PX4Interface(Node):
    def __init__(self):
        super().__init__('ros2_px4_interface')
        
        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('ROS2PX4Interface')
        
        # Publishers
        self.actuator_motors_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        ))
        self.actuator_servos_pub = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        ))
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        ))

        # Subscribers
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        ))
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        ))
        # self.create_subscription(AirspeedWind, '/fmu/out/airspeed_wind', self.airspeed_wind_callback, QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=10
        # ))
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        ))

        # Set up timers
        self.publish_messages_timer = self.create_timer(1.0 / HZ_RATE, self.publish_messages)
        self.offboard_control_mode_timer = self.create_timer(1.0 / OFFBOARD_CONTROL_MODE_RATE, self.publish_offboard_control_mode)

        # Initial command to arm and enable offboard mode
        self.publish_offboard_control_mode()

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

    # def airspeed_wind_callback(self, msg):
    #     """Callback function for airspeed_wind topic subscriber."""
    #     source_map = {
    #         0: 'Synthetic Sideslip Fusion',
    #         1: 'Sensor 1 (Airspeed Fusion)',
    #         2: 'Sensor 2 (Airspeed Fusion)',
    #         3: 'Sensor 3 (Airspeed Fusion)'
    #     }
    #     self.logger.info(f"Received AirspeedWind: "
    #                      f"timestamp={msg.timestamp}, "
    #                      f"timestamp_sample={msg.timestamp_sample}, "
    #                      f"windspeed_north={msg.windspeed_north}, "
    #                      f"windspeed_east={msg.windspeed_east}, "
    #                      f"variance_north={msg.variance_north}, "
    #                      f"variance_east={msg.variance_east}, "
    #                      f"tas_innov={msg.tas_innov}, "
    #                      f"tas_innov_var={msg.tas_innov_var}, "
    #                      f"tas_scale_raw={msg.tas_scale_raw}, "
    #                      f"tas_scale_raw_var={msg.tas_scale_raw_var}, "
    #                      f"tas_scale_validated={msg.tas_scale_validated}, "
    #                      f"beta_innov={msg.beta_innov}, "
    #                      f"beta_innov_var={msg.beta_innov_var}, "
    #                      f"source={source_map.get(msg.source, 'Unknown')}")

    def vehicle_command_ack_callback(self, msg):
        """Callback function for vehicle_command_ack topic subscriber."""
        result_map = {
            0: 'Accepted',
            1: 'Temporarily Rejected',
            2: 'Denied',
            3: 'Unsupported',
            4: 'Failed',
            5: 'In Progress',
            6: 'Cancelled'
        }
        self.logger.info(f"Received VehicleCommandAck: "
                         f"Command={msg.command}, "
                         f"Result={result_map.get(msg.result, 'Unknown')}, "
                         f"Result Param1={msg.result_param1}, "
                         f"Result Param2={msg.result_param2}, "
                         f"Target System={msg.target_system}, "
                         f"Target Component={msg.target_component}, "
                         f"From External={msg.from_external}")

    def publish_messages(self):
        self.logger.info("Publishing messages")

        # Publish ActuatorMotors message
        motors_msg = ActuatorMotors()
        motors_msg.control = [1.0] * 12  # Example motor control values
        self.actuator_motors_pub.publish(motors_msg)
        self.logger.info("Published ActuatorMotors message")

        # Publish ActuatorServos message
        servos_msg = ActuatorServos()
        servos_msg.control = [1.0] * 8  # Example servo control values
        self.actuator_servos_pub.publish(servos_msg)
        self.logger.info("Published ActuatorServos message")

    def publish_offboard_control_mode(self):
        self.logger.info("Publishing OffboardControlMode message")

        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        offboard_control_mode_msg.position = True
        offboard_control_mode_msg.velocity = True
        offboard_control_mode_msg.acceleration = True
        offboard_control_mode_msg.attitude = True
        offboard_control_mode_msg.body_rate = True
        offboard_control_mode_msg.thrust_and_torque = True
        offboard_control_mode_msg.direct_actuator = True

        self.offboard_control_mode_pub.publish(offboard_control_mode_msg)
        self.logger.info("Published OffboardControlMode message")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2PX4Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
