import sys
import os
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_dir, 'mavsim/mavsim_python'))
# TODO: add mavsim to the folder created in this directory.
from pyproj import Proj, transform
from scipy.spatial.transform import Rotation as R
import numpy as np

import rclpy
from rclpy.node import Node
# from nav_msgs.msg import Odometry

# from rosflight_msgs.msg import Airspeed
# from rosflight_msgs.msg import Barometer
# from rosflight_msgs.msg import Command
# from rosflight_msgs.msg import GNSS
# from sensor_msgs.msg import Imu
# from sensor_msgs.msg import MagneticField

import parameters.planner_parameters as PLAN
from controllers.autopilot import Autopilot
from estimators.observer import Observer
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from message_types.msg_autopilot import MsgAutopilot
from message_types.msg_sensors import MsgSensors
from message_types.msg_state import MsgState
from message_types.msg_waypoints import MsgWaypoints
from viewers.view_manager import ViewManager



from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleLocalPosition, VehicleOdometry, AirspeedWind, Airspeed, VehicleAngularVelocity,
                          VehicleStatus, VehicleCommandAck, SensorCombined, ActuatorServos, ActuatorMotors)
from std_msgs.msg import String

# for quaternion-to-euler
import math

def quaternion_to_euler(q):
    """
    Converts a quaternion into roll, pitch, and yaw angles.
    See the following link for the C++ example from which this derives:
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#:~:text=%23define%20_USE_MATH_DEFINES%20%23include,angles%3B%20}
    

    Args:
    q: list or tuple with four elements [q0, q1, q2, q3]
       where q0 is the scalar part, and q1, q2, q3 are the vector part.

    Returns:
    roll: rotation around the x-axis in radians
    pitch: rotation around the y-axis in radians
    yaw: rotation around the z-axis in radians
    """

    q0, q1, q2, q3 = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * math.sqrt(1 + 2*(q0 * q2 - q3 * q1))
    cosp = 2 * math.sqrt(1 - 2 * (q0 * q2 - q3 * q1))
    # if abs(sinp) >= 1:
    #     pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    # else:
    #     pitch = math.asin(sinp)
    pitch = 2*math.atan2(sinp, cosp) - math.pi/2

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def ecef_to_ned_matrix(ecef):
    '''
    Pulled from https://github.com/bsutherland333/mavsim_rosflight_bridge/blob/main/mavsim_bridge/mavsim_bridge.py
    (Used in lines 103 to 114 of said example code)
    This function was used in a gnss_callback as an alternate method of collecting gps position. 
    It is kept here for reference. 
    '''
    # Define the projection for ECEF: EPSG:4978
    # and WGS84 LatLong: EPSG:4326
    ecef_proj = Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    latlong_proj = Proj(proj='latlong', ellps='WGS84', datum='WGS84')

    # Convert from ECEF to Geodetic (Latitude, Longitude, Altitude)
    lon, lat, _ = transform(ecef_proj, latlong_proj, ecef[0], ecef[1], ecef[2], radians=True)

    # Calculate transformation matrix from ECEF to NED
    matrix = np.array([
        [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
        [-np.sin(lon), np.cos(lon), 0],
        [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]
    ])
    return matrix







class ROS_MAVSIM_wrapper(Node):
    def __init__(self):
        super().__init__('ros_mavsim_wrapper')
        control_timer_period = 1 / 100.0
        self.sensors = MsgSensors()
        self.initial_baro = None
        self.initial_ecef = None
        self.ecef_ned_matrix = None
        self.start_time = self.get_clock().now().to_msg()
        self.start_time = self.start_time.sec + self.start_time.nanosec / 1e9
        self.truth_msg = None

        # set up sensor data dictionary
        self.sensors = {}

        # QoS profile for subscriptions. Optional. If used, replace "1" in subscriptions with qos_profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        # initialize elements of the architecture
        self.autopilot = Autopilot(control_timer_period)
        self.observer = Observer(control_timer_period)
        self.path_follower = PathFollower()
        self.path_manager = PathManager()
        self.viewers = ViewManager(data=True)

        # waypoint definition 
        self.waypoints = MsgWaypoints() # TODO: update waypoints for current project's use.
        self.waypoints.type = 'fillet'
        Va = PLAN.Va0
        self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
        self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
        self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
        self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)


        # Create subscriptions
        self.airspeed_sub = self.create_subscription(String, '/airspeed_usb_data', self.airspeed_callback, 1) # Optional
        self.accel_sub = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.accel_callback, 1)
        self.odometry_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_odometry', self.odometry_callback, 1)
                # redundant subscription added for example
        # self.angular_velocity = self.create_subscription(
        #     VehicleAngularVelocity,
        #     '/fmu/out/vehicle_angular_velocity',
        #     self.vehicle_angular_velocity_callback,
        #    1 # can replace with the variable qos_profile
        # )

        # TODO: in the future, add subscriptions to USB-contrived airspeed sensors and calculate alpha.

        # Publishers 
        self.servos_pub = self.create_subscription(ActuatorServos, '/fmu/in/actuator_servos', 1)
        self.motors_pub = self.create_subscription(ActuatorMotors, '/fmu/in/actuator_motors', 1)





        # Create timer for control loop
        self.timer = self.create_timer(control_timer_period, self.timer_callback)

    def airspeed_callback(self, msg):
        # msg is in Pa
        # Uses max since mavsim is not prepared to handle negative values
        # self.sensors.diff_pressure = max(msg.differential_pressure, 0.0) # ROSFlight version
        self.sensors.diff_pressure = max(msg.data, 0.0)  # PX4 version
        self.sensors.windspeed_north = msg.windspeed_north
        self.sensors.beta_innov = msg.beta_innov

        # TODO: change the names of variables to appropriately reflect sensor data
        

    def accel_callback(self, msg):
        # msg is in m/s^2, and this is PX4 form (not ROSFlight) so dictionary
        self.sensors.accel_x = msg.accelerometer_m_s2[0]
        self.sensors.accel_y = msg.accelerometer_m_s2[1]
        self.sensors.accel_z' = msg.accelerometer_m_s2[2]
    
    def odometry_callback(self, msg):
        # msg is in m and m/s
        
        # Position
        self.position_curr = msg.position

        self.sensors.gps_north = self.position_curr[0]
        self.sensors.gps_east = self.position_curr[1]
        self.sensors.gps_altitude = -self.position_curr[2]
        # Attitude
        self.sensors.phi, self.sensors.theta, self.sensors.psi = quaternion_to_euler(msg.q)
        self.sensors.chi = self.sensor.phi # assume negligible windspeed  
      # Velocity
        self.sensors.vx = msg.velocity[0]
        self.sensors.vy = msg.velocity[1]
        self.sensors.vz = msg.velocity[2]
        # Angular velocity
        self.sensors.p = msg.angular_velocity[0]
        self.sensors.q = msg.angular_velocity[1]
        self.sensors.r = msg.angular_velocity[2]




    def timer_callback(self):
        # Get next set of commands
        estimated_state = self.observer.update(self.sensors) #TODO: update sensors object to reflect mavsim's usage
        path = self.path_manager.update(self.waypoints, estimated_state, PLAN.R_min)
        autopilot_commands = self.path_follower.update(path, estimated_state)
        delta, _ = self.autopilot.update(autopilot_commands, estimated_state)

        # Convert the autopilot command to a state message
        commanded_state = MsgState()
        commanded_state.Va = autopilot_commands.airspeed_command
        commanded_state.chi = autopilot_commands.course_command
        commanded_state.altitude = autopilot_commands.altitude_command
        commanded_state.theta = autopilot_commands.climb_rate_command
        commanded_state.phi = autopilot_commands.roll_command

        # Convert the truth message to a state message
        true_state = MsgState()
        if self.truth_msg is not None:
            # Position
            true_state.north = self.truth_msg.pose.pose.position.x
            true_state.east = self.truth_msg.pose.pose.position.y
            true_state.altitude = -self.truth_msg.pose.pose.position.z

            # Attitude
            truth_quat = self.truth_msg.pose.pose.orientation
            truth_quat = [truth_quat.x, truth_quat.y, truth_quat.z, truth_quat.w]
            truth_r = R.from_quat(truth_quat)
            truth_xyz = truth_r.as_euler('xyz', degrees=False) # TODO: decide which version of euler conversion to use. 
            true_state.phi = truth_xyz[0]
            true_state.theta = truth_xyz[1]
            true_state.psi = truth_xyz[2]
            true_state.chi = truth_xyz[2]  # NOTE: Assumes no wind

            # Velocity
            # Assumes no wind
            true_state.Va = np.linalg.norm([self.truth_msg.twist.twist.linear.x,
                                            self.truth_msg.twist.twist.linear.y,
                                            self.truth_msg.twist.twist.linear.z])

            # Angular rates
            true_state.p = self.truth_msg.twist.twist.angular.x
            true_state.q = self.truth_msg.twist.twist.angular.y
            true_state.r = self.truth_msg.twist.twist.angular.z

        # Plot the estimated state, true state, and commanded state
        curr_time = self.get_clock().now().to_msg()
        runtime = curr_time.sec + curr_time.nanosec / 1e9 - self.start_time
        self.viewers.update(runtime,
                            true_state=true_state,
                            estimated_state=estimated_state,
                            commanded_state=commanded_state,
                            delta=delta)

        # ROSFlight Publishing of Delta commands
        # # Publish delta commands
        # msg = Command()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.mode = Command.MODE_PASS_THROUGH
        # msg.x = delta.aileron
        # msg.y = -delta.elevator
        # msg.z = -delta.rudder
        # msg.f = delta.throttle
        # self.delta_pub.publish(msg)

        # PX4 Publishing deltas. Order and naming of actuators follows nomenclature of "Quadplane_Project_Instructions.pdf."
      
        # TODO: fill out servo and motor commands to reflect the appropriate positions outlined in QGroundControl.
        # Actuator Servo
        msg_servo = ActuatorServos()
        msg_servo.timestamp = self.get_clock().now().to_msg()
        msg_servo.control = [0.]*8 
    # Set Servos Appropriately. The assignment numbers reflect the Pinout assignment in QGroundControl. # TODO: confirm positive and negative positions. 
      ELEVATOR_SERVO_ASSIGMENT = 0
      AILERON_SERVO_ASSIGMENT = 1
      RUDDER_SERVO_ASSIGMENT = 2
      REVERSE_AILERON_SERVO_ASSIGMENT= 3

      msg_servo.control[ELEVATOR_SERVO_ASSIGMENT] = -delta.elevator
      msg_servo.control[AILERON_SERVO_ASSIGMENT] = delta.aileron
      msg_servo.control[RUDDER_SERVO_ASSIGMENT] = -delta.rudder
      msg_servo.control[REVERSE_AILERON_SERVO_ASSIGMENT] = -delta.aileron

      
      
        self.servos_pub.publish(msg_servo)
        
        # Actuator Motor
        msg_motor = ActuatorMotors()
        msg_motor.timestamp = self.get_clock().now().to_msg()
        msg_motor.control = [0.]*12 
      # Set Motors appropriately. The assignment numbers reflect the Pinout assignment in QGroundControl. # TODO: confirm positive and negative positions. 
      PORT_FRONT_MOTOR_ASSIGNMENT = 0
      PORT_REAR_MOTOR_ASSIGNMENT = 1
      STARBOARD_REAR_MOTOR_ASSIGNMENT = 2
      STARBOARD_FRONT_MOTOR_ASSIGNMENT = 3
      FRONT_PROPULSION_MOTO_ASSIGNMENT = 4

      # TODO: fix delta names, according to MAVSIM nomenclature and location of actual throttle values

      msg_motor.control[PORT_FRONT_MOTOR_ASSIGNMENT] = delta.port_front_motor
      msg_motor.control[PORT_REAR_MOTOR_ASSIGNMENT] = delta.port_rear_motor
      msg_motor.control[STARBOARD_REAR_MOTOR_ASSIGNMENT] = delta.starboard_rear_motor
      msg_motor.control[STARBOARD_FRONT_MOTOR_ASSIGNMENT] = delta.starboard_front_motor
      msg_motor.control[FRONT_PROPULSION_MOTO_ASSIGNMENT] = delta.forward_propulsion_motor
      
      
        self.motors_pub.publish(msg_motor)




def main():
    rclpy.init()
    mavsim_wrapper = ROS_MAVSIM_wrapper()
    rclpy.spin(mavsim_wrapper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
