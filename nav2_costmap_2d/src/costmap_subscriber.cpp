/*
  这个文件主要功能是订阅和接收Costmap数据，并将其转换成Costmap2D对象，供导航和路径规划使用。确保了Costmap数据的实时更新和有效管理，支持导航系统的精确运作。
  Costmap2D一个核心组件，它是一个二维网格结构，用来表示机器人周围环境中的障碍信息。这个对象主要存储和管理关于地形的代价信息，帮助机器人进行路径规划和避障。
*/
#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_subscriber.hpp"

//定义命名空间
namespace nav2_costmap_2d
{
//构造函数使用nav2_util::LifecycleNode的弱指针和话题名称初始化CostmapSubscriber。
CostmapSubscriber::CostmapSubscriber(const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & topic_name): topic_name_(topic_name)
{
  auto node = parent.lock();  //尝试从弱指针获取强指针(我发现nav2的源码很喜欢这么做)
  //创建一个订阅者，订阅指定的Costmap话题，使用特定的QoS设置
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
  /*
    这里详细介绍一下rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()这个Qos队列的创建是什么意思：
    1.rclcpp::KeepLast(1)：历史保持设置，指示ros2只保留最后一个发布的消息。这里的参数1表示最新的1个消息被缓存，如果新消息到来，它将替代旧的消息。
    2.transient_local()：这是一种持久化设置，使得发布的消息可以被后来连接的订阅者接收。
    换句话说，即使订阅者在消息发布后才开始订阅该话题，它仍然能接收到最新缓存的消息。
    这种创建方式非常适用于那些需要立即获取最新状态信息的场景，例如，一个新启动的节点可能需要知道最新的配置或状态信息。
    3.reliable()：传输可靠性设置，它要求底层的通讯层面确保所有发送的消息都能到达，如果消息在传输过程中丢失，系统会尝试重新发送。这种设置是对应BEST_EFFORT的，BEST_EFFORT则不保证消息的可靠传输。

    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()这样的Qos创建方式是一种高可靠性的消息传输方式设置，确保所有订阅者都能接收到最新的状态信息，并且可以不在乎连接到的时间。
    这样的设置对于更新costmap非常有用，具有很高的实时和动态性。

    总结一些，其实这句话：rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()就是只保持最新的消息队列更新。
  */
}

//构造函数，接受一个指向ROS节点的弱指针和订阅话题名称，初始化成员变量 topic_name_
CostmapSubscriber::CostmapSubscriber(const rclcpp::Node::WeakPtr & parent, const std::string & topic_name): topic_name_(topic_name)
{

  auto node = parent.lock();//尝试从弱指针获取强指针
  //创建一个订阅者，订阅指定的Costmap话题，并设置消息的质量服务参数。
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

//获取转换后的 Costmap2D 对象
std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  //如果还没有接收到任何 Costmap 数据，则抛出异常
  if (!costmap_received_) 
  {
    throw std::runtime_error("Costmap is not available");
  }
  //调用方法来转换最新接收的 Costmap 数据到 Costmap2D 格式
  toCostmap2D();
  //返回 Costmap2D 对象的共享指针
  return costmap_;
}

//将接收到的 Costmap 消息转换为 Costmap2D 对象
void CostmapSubscriber::toCostmap2D()
{
  //原子操作加载最新的Costmap消息
  /*
    那么什么是原子操作呢？这里补充一下：
    在多线程程序中，原子操作保证了在执行整个操作的过程中，不会有其他线程干扰或改变操作的数据。这是通过确保操作在一个单独的机器指令中完成来实现的，不用使用互斥锁。
    为什么这里要使用原子操作呢？
    std::atomic_load(&costmap_msg_)用于从共享的原子变量costmap_msg_安全地读取值。这个变量可能在不同的线程中被访问或修改，比如在一个线程中被赋值，而在另一个线程中被读取。
    使用原子加载函数可以确保读取的值是一致且最新的，避免了在读取过程中数据可能被另一个线程修改导致的数据不一致问题。
  */
  auto current_costmap_msg = std::atomic_load(&costmap_msg_);
  //如果内部的 Costmap2D 对象尚未初始化，则创建一个新的
  if (costmap_ == nullptr) 
  {
    costmap_ = std::make_shared<Costmap2D>(
      current_costmap_msg->metadata.size_x,  //使用消息中的x尺寸初始化
      current_costmap_msg->metadata.size_y,  //使用消息中的y尺寸初始化
      current_costmap_msg->metadata.resolution,  //使用消息中的分辨率初始化
      current_costmap_msg->metadata.origin.position.x,  //使用消息中的x原点初始化
      current_costmap_msg->metadata.origin.position.y);  //使用消息中的y原点初始化
  } 
  //检查是否需要调整Costmap2D对象的大小或属性。
  else if (costmap_->getSizeInCellsX() != current_costmap_msg->metadata.size_x ||  //检查Costmap2D的当前X尺寸是否与消息中的X尺寸相同
    costmap_->getSizeInCellsY() != current_costmap_msg->metadata.size_y ||  //检查Costmap2D的当前Y尺寸是否与消息中的Y尺寸相同
    costmap_->getResolution() != current_costmap_msg->metadata.resolution ||  //检查Costmap2D的分辨率是否与消息中的分辨率相同
    costmap_->getOriginX() != current_costmap_msg->metadata.origin.position.x || //检查Costmap2D的X原点是否与消息中的X原点相同
    costmap_->getOriginY() != current_costmap_msg->metadata.origin.position.y)  //检查Costmap2D的Y原点是否与消息中的Y原点相同
  {
    //如果 Costmap2D 的尺寸或属性与当前消息不匹配，则调整它
    costmap_->resizeMap(
      current_costmap_msg->metadata.size_x,  //将Costmap2D的X尺寸设置为消息中的X尺寸
      current_costmap_msg->metadata.size_y,  //将Costmap2D的Y尺寸设置为消息中的Y尺寸
      current_costmap_msg->metadata.resolution,  //将Costmap2D的分辨率设置为消息中的分辨率
      current_costmap_msg->metadata.origin.position.x,  //将Costmap2D的X原点设置为消息中的X原点
      current_costmap_msg->metadata.origin.position.y);  //将Costmap2D的Y原点设置为消息中的Y原点
  }
  //获取Costmap2D的数据数组。
  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  //遍历 Costmap 消息的数据，并填充到 Costmap2D 的数据数组中。
  for (unsigned int i = 0; i < current_costmap_msg->metadata.size_x; ++i) 
  {
    for (unsigned int j = 0; j < current_costmap_msg->metadata.size_y; ++j) 
    {
      master_array[index] = current_costmap_msg->data[index];  //将数据复制到 Costmap2D 中。
      ++index;
    }
  }
}

//当接收到Costmap消息时的回调函数
void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  //原子操作存储接收到的Costmap消息
  std::atomic_store(&costmap_msg_, msg);
  //如果是第一次接收到Costmap消息，设置标志为真
  if (!costmap_received_) 
  {
    costmap_received_ = true;
  }
}
}
