import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/dubhe/store/learning/ros2_learning_experience/chapter4/chapt4_ws/src/install/demo_python_service'
