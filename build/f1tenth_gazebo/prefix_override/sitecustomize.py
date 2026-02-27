import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bhargav/ee_cs_265A_ws/install/f1tenth_gazebo'
