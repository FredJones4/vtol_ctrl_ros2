import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import time

HZ_RATE = 200.0

class ROS2PX4Interface(Node):
    def __init__(self):
        super().__init__('ros2_px4_interface')

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Set up a timer for publishing messages at 1/HZ_RATE Hz
        self.timer = self.create_timer(1.0 / HZ_RATE, self.timer_callback)

        self.offboard_setpoint_counter = 0

        # Initial command to arm and enable offboard mode
        self.arm_and_enable_offboard()

    def arm_and_enable_offboard(self):
        self.get_logger().info("Arming vehicle and enabling offboard mode")

        # Create and publish VehicleCommand message to arm the vehicle
        self.arm()

        # Create and publish VehicleCommand message to enable offboard mode
        self.engage_offboard_mode()

    def arm(self):
        """Send a command to arm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a command to disarm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to offboard mode")

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
        msg.position = [0.0, 0.0, -5.0]  # Hover 5 meters above the ground
        msg.yaw = -3.14  # Yaw angle
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

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            # Arm the vehicle
            self.arm()

        # Publish offboard control mode and trajectory setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Increment the counter and stop after reaching 11
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ROS2PX4Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
