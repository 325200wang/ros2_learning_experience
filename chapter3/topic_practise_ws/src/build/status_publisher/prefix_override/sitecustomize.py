import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/dubhe/store/learning/ros2_learning_experience/chapter3/topic_practise_ws/src/install/status_publisher'
