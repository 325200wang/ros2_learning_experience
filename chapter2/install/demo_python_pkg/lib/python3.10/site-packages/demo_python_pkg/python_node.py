import rclpy
from rclpy.node import Node

def main():#在setup.py中console_scripts下，执行python_node相当于执行此main函数
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info("Hi python Node")
    rclpy.spin(node)
    rclpy.shutdown()