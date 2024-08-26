import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import (ActuatorMotors, ActuatorServos, VehicleCommand, OffboardControlMode)
from std_msgs.msg import Header
import time
import math



HZ_RATE = 100.0

class PX4_ROS2_TEST(Node):
    def __init__(self):
        super().__init__('px4_ros2_test')

        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.offboard_setpoint_counter = 0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # Create subscriber to data_to_control
        self.data_to_control_sub = self.create_subscription(
            String,
            'data_to_control',
            self.data_to_control_callback,
            qos_profile
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

        self.timer = self.create_timer(0.1, self.timer_callback)  # 100 ms timer, or 10 Hz clock

    def change_mode(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.timer_start).nanoseconds/ 1e9

        if elapsed_time >= self.timer_period:
            self.get_logger().info("Changing mode to fixed-wing")

            # Create and publish VehicleCommand message to change mode
            cmd = VehicleCommand()
            cmd.command = 84  # Command to change flight mode (see https://docs.px4.io/main/en/msg_docs/VehicleCommand.html, line 84)
            cmd.param1 = 1.0     # Parameter to indicate the desired mode (check specific parameter for your system)
            cmd.target_system = 1
            cmd.source_system = 1
            cmd.target_component = 0
            cmd.source_component = 0
            self.vehicle_command_pub.publish(cmd)
            self.get_logger().info("Published VehicleCommand message to change mode to fixed-wing")

            # Stop the timer after mode change
            self.timer2.destroy()
    def data_to_control_callback(self, msg):
        self.get_logger().info('Received data_to_control message')
        # Extract data from the message
        data = msg.data

        # Process the data
        # ...
        # # Publish actuator control messages
        # self.publish_actuator_items()

    def timer_callback(self):
        print("counter: ", self.offboard_setpoint_counter)
        if self.offboard_setpoint_counter == 10:
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            # Arm the vehicle
            self.arm()
        if self.offboard_setpoint_counter < 200: # 20 seconds
            # Publish offboard control mode and trajectory setpoint
            self.publish_offboard_control_mode()
            # self.publish_trajectory_setpoint()
            self.publish_actuator_items()
            if self.offboard_setpoint_counter > 10:
                self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter == 200:
            # self.timer_start = self.get_clock().now()
            # # Initialize the mode change (for example, change to fixed-wing after 5 seconds)
            # self.get_logger().info("Preparing to change mode from VTOL to fixed-wing")
            # self.timer_period = 5.0  # Change mode after 5 seconds
            # self.timer2 = self.create_timer(1.0, self.change_mode)
            self.offboard_setpoint_counter += 1

            pass
        elif self.offboard_setpoint_counter < 400: #TEMP
            self.publish_offboard_control_mode()
            self.publish_actuator_items()
            self.offboard_setpoint_counter += 1
        else:
            self.publish_offboard_control_mode()
            self.publish_actuator_items()

        # Increment the counter and stop after reaching 11
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def arm(self):
        """Send a command to arm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a command to disarm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(time.time() * 1e6)  # microseconds
        self.offboard_control_mode_publisher.publish(msg)
        self.get_logger().info('Published OffboardControlMode message')

    def publish_trajectory_setpoint(self):
        """Publish a trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -20.0]
        msg.yaw = -3.14  # [-PI:PI]
        msg.timestamp = int(time.time() * 1e6)  # microseconds
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info('Published TrajectorySetpoint message')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(time.time() * 1e6)  # microseconds
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f'Published VehicleCommand message with command {command}')

    def publish_actuator_motors(self, motors_msg):
        self.actuator_motors_pub.publish(motors_msg)
        self.get_logger().info('Published ActuatorMotors message')


    def publish_actuator_servos(self, servos_msg):
        self.actuator_servos_pub.publish(servos_msg)
        self.get_logger().info('Published ActuatorServos message')


    def publish_actuator_items(self):
        motors_msg = ActuatorMotors()  # Populate with relevant data
        motors_msg.control = [0.99] * 12

        # # command_msg = VehicleCommand()  # Populate with relevant data
        servos_msg = ActuatorServos()   # Populate with relevant data
        servos_msg.control = [0.99] * 8
        # control_mode_msg = OffboardControlMode()  # Populate with relevant data

        self.publish_actuator_motors(motors_msg)
        # node.publish_vehicle_command(command_msg)
        self.publish_actuator_servos(servos_msg)
        # node.publish_offboard_control_mode(control_mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4_ROS2_TEST()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
