import rclpy # type: ignore
from rclpy.node import Node
def main():
    rclpy.init()
    node = Node("python_node") #node
    node.get_logger().info('hello python node')
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()