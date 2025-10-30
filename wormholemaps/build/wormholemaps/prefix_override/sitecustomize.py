import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amal/my_ws/src/wormholemaps/install/wormholemaps'
