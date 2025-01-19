import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zqh/robot2/robot2_ws/src/install/fishbot_rgbd'
