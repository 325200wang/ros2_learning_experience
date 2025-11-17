import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/dubhe/store/learning/ros2_learning_experience/chapter2/chapt2_ws/install/demo_python_pkg'
