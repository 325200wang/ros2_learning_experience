import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/dubhe/store/learning/ros2_learning_experience/chapter5/chapt5_ws/install/demo_python_tf'
