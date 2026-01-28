import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/issatay/Desktop/unitree_ws/install/g1_bringup'
