/*
  这个文件实现了一个名为FootprintSubscriber的类，用于在导航栈中订阅和管理机器人的足迹(footprint)。
  足迹通常表示为一个多边形，定义了机器人在二维平面上占据的空间区域。该类主要用于接收和转换足迹数据，使其可用于代价地图计算和碰撞检测等功能。

  主要提供以下三个功能：
    订阅足迹话题：从指定的话题接收足迹数据。
    转换足迹坐标：将接收到的足迹转换到机器人的基座坐标系中。
    提供足迹数据：提供转换后的足迹数据给其他组件，如碰撞检测系统。
*/
#include <string>
#include <vector>
#include <memory>

//包含FootprintSubscriber类的头文件
#include "nav2_costmap_2d/footprint_subscriber.hpp"
//下面这三个命令是用来忽略特定编译警告的预处理指令
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop
//包含tf2库的工具函数，用于处理转换和角度
#include "tf2/utils.h"


namespace nav2_costmap_2d
{
//构造函数，第一个参数是父节点的弱指针(又是弱指针获取强指针)，第二个参数是足迹话题名称，第三个参数是TF变换缓存，第四个参数是机器人的基座坐标系，第五个参数是坐标变换的容忍度
/*
  这里补充一下，什么是坐标变换的容忍度(transform_tolerance)：
  坐标变换的容忍度是指在执行坐标系之间的转换时允许的最大时间偏差。
  这种偏差是必要的，因为在一个动态系统中，传感器数据、控制命令和其他信息不可能完全同步。
  转换容忍度定义了在查找过去的变换时可以接受的最大历史时刻与当前请求时间的差距。

  主要用途：
  时间戳对齐：
    在机器人系统中，各种传感器和数据源可能会以不同的频率和不同的时间点发布数据。使用变换容忍度可以在查询这些数据对应的坐标变换时，容忍一定程度的时间错位。
  确保数据有效性：
    如果请求的变换时间戳超出了指定的容忍度范围，那么系统可能会判断该变换为无效。这有助于避免使用错误或过时的数据进行关键的计算，如位置估计或路径规划。
  增强系统的鲁棒性：
    通过适当设置变换容忍度，可以提高系统对于传感器延迟或数据传输延迟的适应能力，从而在实际运行环境中增强其稳定性和可靠性。
*/
FootprintSubscriber::FootprintSubscriber(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance): tf_(tf), robot_base_frame_(robot_base_frame), transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();  //尝试从弱指针获取强指针
  //创建订阅者，订阅指定的足迹话题，使用系统默认的Qos
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

//FootprintSubscriber类的构造函数定义，第一个参数还是一个父节点的弱指针，第二个参数是足迹话题名称，第三个参数是TF变换缓冲区，用于处理坐标变换，第四个参数是机器人的基座坐标系名称，第五个参数是坐标变换的容忍度
FootprintSubscriber::FootprintSubscriber(
  const rclcpp::Node::WeakPtr & parent,  
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance): tf_(tf), robot_base_frame_(robot_base_frame), transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();  //弱指针获取强引用
  //创建一个订阅者对象，订阅指定的足迹话题
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}
/*
  这两个构造函数代码块在结构上看起来非常相似，虽然它们实际上执行相同的操作，但有一个关键的区别在于它们接受的父节点类型不同。这个不同会影响生命周期节点的管理。
  第一个构造函数接收一个类型为nav2_util::LifecycleNode::WeakPtr的父节点，使用这个父节点意味着这个构造函数是专门为那些实现了生命周期管理的节点设计的。
  第二个构造函数接收一个类型为rclcpp::Node::WeakPtr的父节点，使用这个节点指向的是普通的ros2节点，不是生命周期节点。
*/

//获取原始足迹数据，第一个参数是输出参数，用于存储足迹点的向量；第二个参数也是输出参数，用于存储足迹数据的头部信息
bool FootprintSubscriber::getFootprintRaw(std::vector<geometry_msgs::msg::Point> & footprint, std_msgs::msg::Header & footprint_header)
{
  if (!footprint_received_)  //如果没有接收到足迹数据，返回false
  {
    return false;
  }

  auto current_footprint = std::atomic_load(&footprint_);  //使用原子操作安全地加载共享的足迹数据
  footprint = toPointVector(std::make_shared<geometry_msgs::msg::Polygon>(current_footprint->polygon));  //将足迹多边形数据转换为点向量
  footprint_header = current_footprint->header;  //拷贝足迹数据的头部信息

  return true;  //返回true表示成功获取足迹数据
}

//将足迹转换到机器人基座坐标系
bool FootprintSubscriber::getFootprintInRobotFrame(std::vector<geometry_msgs::msg::Point> & footprint,std_msgs::msg::Header & footprint_header)
{
  //首先尝试获取原始足迹数据
  if (!getFootprintRaw(footprint, footprint_header)) 
  {
    return false;  //如果获取失败，返回 false
  }

  //接着尝试获取机器人当前位姿
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, tf_, footprint_header.frame_id, robot_base_frame_, transform_tolerance_, footprint_header.stamp))
  {
    return false;  //如果无法获取当前机器人姿态，返回 false
  }

  double x = current_pose.pose.position.x;  //获取机器人当前位置的x坐标
  double y = current_pose.pose.position.y;  //获取机器人当前位置的y坐标
  double theta = tf2::getYaw(current_pose.pose.orientation);  //获取机器人当前朝向的偏航角

  std::vector<geometry_msgs::msg::Point> temp;  //临时存储转换后的足迹点
  transformFootprint(-x, -y, 0, footprint, temp);  //先对足迹进行平移变换
  transformFootprint(0, 0, -theta, temp, footprint);  //再对足迹进行旋转变换

  footprint_header.frame_id = robot_base_frame_;  //更新足迹数据的坐标系为机器人基座坐标系
  footprint_header.stamp = current_pose.header.stamp;  //更新足迹数据的时间戳

  return true;  //成功转换足迹数据，返回 true
}

//足迹数据接收的回调函数
void FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::atomic_store(&footprint_, msg);  //使用原子操作安全地存储接收到的足迹消息
  //检查是否是第一次接收到足迹数据
  if (!footprint_received_) 
  {
    footprint_received_ = true;  //标记为已接收到足迹，确保后续处理
  }
}

}  // namespace nav2_costmap_2d
