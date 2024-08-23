#!/usr/bin/env python

import rospy
from serial import Serial, SerialException
from std_msgs.msg import String

# Initialize the serial connection
ser = Serial()

def write_callback(msg):
    rospy.loginfo("Writing to serial port: %s", msg.data)
    ser.write(msg.data.encode())

def main():
    rospy.init_node('serial_example_node', anonymous=True)
    
    # Subscriber for writing data to the serial port
    rospy.Subscriber("write", String, write_callback)
    # Publisher for reading data from the serial port
    read_pub = rospy.Publisher("read", String, queue_size=1000)

    try:
        # Set up the serial port
        ser.port = "/dev/ttyACM0"  # Change to your specific port
        ser.baudrate = 9600        # Set baudrate
        ser.timeout = 1            # Set timeout
        ser.open()
    except SerialException as e:
        rospy.logerr("Unable to open port: %s", str(e))
        return

    if ser.is_open:
        rospy.loginfo("Serial Port initialized")
    else:
        return

    loop_rate = rospy.Rate(5)  # Loop at 5Hz

    while not rospy.is_shutdown():
        # Spin ROS once for callbacks
        rospy.spinOnce()

        # Check if data is available to read from serial
        if ser.in_waiting > 0:
            rospy.loginfo("Reading from serial port")
            data = ser.read(ser.in_waiting).decode()
            rospy.loginfo("Read: %s", data)
            read_pub.publish(data)
        
        loop_rate.sleep()

    # Close the serial port before exiting
    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
