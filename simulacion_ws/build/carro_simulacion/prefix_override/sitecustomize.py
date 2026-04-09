import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jason/C-ROS-Automodel_Car/simulacion_ws/install/carro_simulacion'
