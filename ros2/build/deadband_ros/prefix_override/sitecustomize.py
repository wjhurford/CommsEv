import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/will-/Documents/repos/deadband/ros2/install/deadband_ros'
