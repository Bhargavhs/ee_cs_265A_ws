import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bhargav/ee_cs_265A_ws/src/ee_cs_265a/install/ee_cs_265a'
