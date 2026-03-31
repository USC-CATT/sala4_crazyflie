import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/catt/Desktop/sala4_crazyflie/install/sala4_bringup'
