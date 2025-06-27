import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rw/ros2_ws/src/ik_retargeting/install/telekinesis'
