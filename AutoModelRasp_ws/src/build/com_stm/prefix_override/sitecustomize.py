import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alereyes/C-ROS-Automodel_Car/AutoModelRasp_ws/src/install/com_stm'
