import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/admin1/acc_robotics_ws/install/acc_waypoint_navigation'
