import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleLocalPosition, VehicleOdometry, AirspeedWind, Airspeed, VehicleAngularVelocity,
                          VehicleStatus, VehicleCommandAck)

class PX4DataGatherer(Node):
    def __init__(self):
        super().__init__('px4_data_gatherer')

        # QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribers
        # self.create_subscription(
        #     VehicleLocalPosition,
        #     '/fmu/out/vehicle_local_position',
        #     self.vehicle_local_position_callback,
        #     qos_profile
        # )

        # self.create_subscription(
        #     VehicleOdometry,
        #     '/fmu/out/vehicle_odometry',
        #     self.vehicle_odometry_callback,
        #     qos_profile
        # )

        self.create_subscription(
            AirspeedWind,
            '/fmu/out/airspeed_wind',
            self.airspeed_wind_callback,
            qos_profile
        )

        self.create_subscription(
            Airspeed,
            '/fmu/out/airspeed',
            self.airspeed_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.vehicle_angular_velocity_callback,
            qos_profile
        )

        # self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     qos_profile
        # )

        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback,
            qos_profile
        )

    def vehicle_local_position_callback(self, msg):
        self.get_logger().info('Received VehicleLocalPosition message')

    def vehicle_odometry_callback(self, msg):
        self.get_logger().info('Received VehicleOdometry message')
        self.get_logger().info(f"Position: {msg.position}")
        self.get_logger().info(f"Quaternion: {msg.q}")
        self.get_logger().info(f"Velocity Frame: {msg.velocity_frame}")
        self.get_logger().info(f"Velocity: {msg.velocity}")
        self.get_logger().info(f"Angular Velocity: {msg.angular_velocity}")
        self.get_logger().info(f"Position Variance: {msg.position_variance}")
        self.get_logger().info(f"Orientation Variance: {msg.orientation_variance}")
        self.get_logger().info(f"Velocity Variance: {msg.velocity_variance}")

    def airspeed_wind_callback(self, msg):
        self.get_logger().info('Received AirspeedWind message')
        self.get_logger().info(f"Timestamp: {msg.timestamp}")
        self.get_logger().info(f"Timestamp Sample: {msg.timestamp_sample}")
        self.get_logger().info(f"Windspeed North: {msg.windspeed_north}")
        self.get_logger().info(f"Windspeed East: {msg.windspeed_east}")
        self.get_logger().info(f"Variance North: {msg.variance_north}")
        self.get_logger().info(f"Variance East: {msg.variance_east}")
        self.get_logger().info(f"TAS Innov: {msg.tas_innov}")
        self.get_logger().info(f"TAS Innov Var: {msg.tas_innov_var}")
        self.get_logger().info(f"TAS Scale Raw: {msg.tas_scale_raw}")
        self.get_logger().info(f"TAS Scale Raw Var: {msg.tas_scale_raw_var}")
        self.get_logger().info(f"TAS Scale Validated: {msg.tas_scale_validated}")
        self.get_logger().info(f"Beta Innov: {msg.beta_innov}")
        self.get_logger().info(f"Beta Innov Var: {msg.beta_innov_var}")
        self.get_logger().info(f"Source: {msg.source}")

    def airspeed_callback(self, msg):
        self.get_logger().info('Received Airspeed message')
        self.get_logger().info(f"Timestamp: {msg.timestamp}")
        self.get_logger().info(f"Timestamp Sample: {msg.timestamp_sample}")
        self.get_logger().info(f"Indicated Airspeed: {msg.indicated_airspeed_m_s}")
        self.get_logger().info(f"True Airspeed: {msg.true_airspeed_m_s}")
        self.get_logger().info(f"Air Temperature: {msg.air_temperature_celsius}")
        self.get_logger().info(f"Confidence: {msg.confidence}")

    def vehicle_angular_velocity_callback(self, msg):
        self.get_logger().info('Received VehicleAngularVelocity message')

    # def vehicle_status_callback(self, msg):
    #     self.get_logger().info('Received VehicleStatus message')

    def vehicle_command_ack_callback(self, msg):
        self.get_logger().info('Received VehicleCommandAck message')

def main(args=None):
    rclpy.init(args=args)
    node = PX4DataGatherer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
