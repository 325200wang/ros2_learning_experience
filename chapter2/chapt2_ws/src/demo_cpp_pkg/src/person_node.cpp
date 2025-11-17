// person_node.cpp
// 简单的 ROS2 C++ 节点示例：定义 PersonNode 类并在 main 中运行该节点。

#include <string>
#include "rclcpp/rclcpp.hpp"

// PersonNode 是一个继承自 rclcpp::Node 的简单类，表示一个人并提供 eat 方法。
class PersonNode : public rclcpp::Node
{
private:
    // 成员变量：姓名和年龄
    std::string name_;
    int age_;

public:
    // 构造函数：接收节点名、人的姓名和年龄
    // 使用基类构造函数初始化 ROS2 节点（Node(node_name)）
    PersonNode(const std::string &node_name,
               const std::string &name,
               const int &age)
        : Node(node_name)
    {
        // 将参数保存到成员变量
        this->name_ = name;
        this->age_ = age;
    };

    // eat 方法：输出日志，表示该人正在吃某种食物
    void eat(const std::string &food_name)
    {
        // 使用 RCLCPP_INFO 打印信息到 ROS2 日志系统
        RCLCPP_INFO(this->get_logger(), "%s is eating %s", name_.c_str(), food_name.c_str());
    };
};

// 程序入口：初始化 ROS2，创建 PersonNode 实例并运行（spin）
int main(int argc, char ** argv)
{
    // 初始化 ROS2 客户端库
    rclcpp::init(argc, argv);

    // 创建一个 PersonNode 节点实例，节点名为 "cpp_node"，姓名 "Alice"，年龄 30
    auto node = std::make_shared<PersonNode>("cpp_node", "Alice", 30);

    // 调用自定义方法，打印 "Alice is eating apple"
    node->eat("apple");

    // 让节点开始运行，处理回调（本例中没有订阅或定时器，因此会阻塞直到被外部中断）
    rclcpp::spin(node);

    // 清理并关闭 ROS2
    rclcpp::shutdown();
    return 0;
}