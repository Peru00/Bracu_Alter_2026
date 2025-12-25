import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/peru0002/ros2_ws/src/install/bracu_hydra_description'
