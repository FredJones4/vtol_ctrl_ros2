import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cmhales/vtol_ctrl_ros2/install/ros2_px4_interface'
