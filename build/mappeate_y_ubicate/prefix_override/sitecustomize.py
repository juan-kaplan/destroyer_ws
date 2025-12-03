import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alumno1/Documents/destroyer_ws/install/mappeate_y_ubicate'
