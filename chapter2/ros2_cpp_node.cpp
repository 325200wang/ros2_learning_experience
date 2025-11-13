#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);//initialize and distribute the resource, get ready for communicate
    auto node = std::make_shared<rclcpp::Node>("cpp_node");//construct node name,return smart pointer
    RCLCPP_INFO(node->get_logger(),"hello c++ node");//log recorder
    rclcpp::spin(node);//spin func continuously loops and detects
    rclcpp::shutdown();//clean up
    return 0;
}