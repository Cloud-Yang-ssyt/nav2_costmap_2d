/*
  这个节点应该是个总节点，就是用于生成完整二位代价地图使用的
*/
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //创建一个Costmap2DROS类型的智能指针，并命名为"costmap"。这个节点将处理所有与代价地图相关的功能。
  auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
