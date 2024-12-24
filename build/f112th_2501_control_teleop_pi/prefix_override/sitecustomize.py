import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/camel/f112th_2501_camel_rpi4/install/f112th_2501_control_teleop_pi'
