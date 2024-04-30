/*
  这段代码是用于清理Costmap的一部分或全部区域。Costmap是在移动机器人导航中用于存储环境障碍信息的数据结构。
  这个文件主要定义了ClearCostmapService类，该类提供了三种服务：清理除指定区域外的所有区域、清理机器人周围的区域、清理整个Costmap。
*/

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

//这两个头文件都在当前目录的include下面
#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{
//使用std命名空间中的常用类型
using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;

//使用ROS2服务消息的别名，便于代码理解和使用
/*
  ClearExceptRegion(nav2_msgs::srv::ClearCostmapExceptRegion):
  这个服务允许请求清除Costmap中除指定区域之外的所有区域。
  例如，在一个机器人正在进行长时间任务，而某个特定区域（如机器人正前方）的障碍信息已知并且仍然有效时，可以清除其他区域的旧障碍信息，以减少处理数据和提高导航的效率。
*/
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
/*
  ClearAroundRobot(nav2_msgs::srv::ClearCostmapAroundRobot):
  这个服务允许请求清除机器人周围一定半径内的Costmap区域。
  这常用于机器人在复杂或动态环境中需要快速适应周围变化的情况，例如，在人群中导航或在内部环境中避障时清理周围的障碍信息，以便机器人根据最新的环境数据进行路径规划。
*/
using ClearAroundRobot = nav2_msgs::srv::ClearCostmapAroundRobot;
/*
  ClearEntirely(nav2_msgs::srv::ClearEntireCostmap):
  这个服务允许请求完全清除整个Costmap。
  当环境发生显著变化，或者Costmap数据由于某些原因变得不可靠时，可能需要完全重新开始收集环境数据。例如，机器人从一个室内环境移动到另一个完全不同的室内环境时，清除所有旧的障碍信息，以建立基于新环境的障碍信息。
*/
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap;

//构造函数，第一个参数是一个生命周期节点的弱指针，第二个参数是costmap_2d_ros.hpp头文件下的一个类
ClearCostmapService::ClearCostmapService(const nav2_util::LifecycleNode::WeakPtr & parent, Costmap2DROS & costmap): costmap_(costmap)
{
  /*
    lock()方法是std::weak_ptr的一个成员函数，用来尝试从std::weak_ptr获取一个std::shared_ptr实例。
    如果原来的 std::shared_ptr 还存在，即所指向的对象还没有被销毁，则lock()会成功返回一个新的 std::shared_ptr 实例，
    这个实例将与其他的std::shared_ptr实例共享对象的所有权，并且对象不会被销毁，直到最后一个std::shared_ptr被销毁。
  */
  //所以我理解的这一步是为了从生命周期节点的若指针中获取一个强指针，至于为什么不直接使用强指针：如果这个生命周期节点已经被销毁了，用这种方式就调用不到具体的指针了
  auto node = parent.lock();
  logger_ = node->get_logger();  //获取日志记录器
  reset_value_ = costmap_.getCostmap()->getDefaultValue();  //获取Costmap默认值

  node->get_parameter("clearable_layers", clearable_layers_);  //读取配置参数clearable_layers_

  //创建clear_except_service_服务端，功能是清除Costmap中除指定区域外的所有区域。这允许保留关键区域的数据，同时清理其他可能已过时或不再相关的数据。
  clear_except_service_ = node->create_service<ClearExceptRegion>("clear_except_" + costmap_.getName(),std::bind(
    &ClearCostmapService::clearExceptRegionCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  //创建clear_around_service_服务端，功能是清除机器人周围一定范围内的Costmap区域。这常用于机器人需要清理周围旧障碍信息以响应环境变化时。
  clear_around_service_ = node->create_service<ClearAroundRobot>("clear_around_" + costmap.getName(),std::bind(
    &ClearCostmapService::clearAroundRobotCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  //创建服务端，功能是完全清除整个Costmap。机器人从一个环境迁移到另一个截然不同的环境时特别有用。
  clear_entire_service_ = node->create_service<ClearEntirely>("clear_entirely_" + costmap_.getName(),std::bind(
    &ClearCostmapService::clearEntireCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

//服务回调函数：处理除特定区域外清理请求
void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(logger_, "%s",("Received request to clear except a region the " + costmap_.getName()).c_str());

  //调用区域清理函数，传入不清理特定区域
  clearRegion(request->reset_distance, true);
}

//服务回调函数：处理清理机器人周围区域的请求
void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response>/*response*/)
{
  //调用区域清理函数，仅清理机器人周围
  clearRegion(request->reset_distance, false);
}

//服务回调函数：处理清理整个Costmap的请求
void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request>/*request*/,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(logger_, "%s", ("Received request to clear entirely the " + costmap_.getName()).c_str());

  //调用函数清理整个Costmap
  clearEntirely();
}

//实现区域清理逻辑
void ClearCostmapService::clearRegion(const double reset_distance, bool invert)
{
  double x, y;

  if (!getPosition(x, y))  //如果获取不到具体位置
  {
    //返回一个错误日志
    RCLCPP_ERROR(logger_, "%s", "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }

  //获取Costmap中所有层的插件。这些层可以是障碍物层、膨胀层等，每层都负责处理Costmap的一部分数据。
  auto layers = costmap_.getLayeredCostmap()->getPlugins();

  //遍历所有层，每个层是一个插件，负责处理Costmap的特定部分。
  for (auto & layer : *layers) 
  {
    //检查当前层是否可以被清除。isClearable方法用于确定某个层是否支持清除操作。
    if (layer->isClearable()) 
    {
      //如果层可以清除，则将层的智能指针从基类指针转换为CostmapLayer类的指针。
      //使用static_pointer_cast进行向下转型，因为已经确定这个层是CostmapLayer类型。
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);

      // 调用clearLayerRegion函数来清理当前层的指定区域。
      // 传递的参数包括转换后的层指针、机器人当前的x、y坐标、要重置的距离，以及是否反转清除区域。
      clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
    }
  }
}

//实现具体层的区域清理
void ClearCostmapService::clearLayerRegion(shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance, bool invert)
{
  //获取CostmapLayer对应的互斥锁并锁定，保证在修改区域时线程安全。
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  //计算要清理的区域的起始点坐标，基于机器人的当前位置和需要重置的距离。
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;

  //计算要清理区域的终点坐标。
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  //将计算出的世界坐标转换为地图的格子坐标，并确保坐标在地图边界内。
  int start_x, start_y, end_x, end_y;
  costmap->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  //清理指定的矩形区域，根据invert参数决定是清理区域内还是区域外。
  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

  //更新Costmap的额外边界，这样可以在必要时触发地图的更新。
  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

//清理整个Costmap层函数
void ClearCostmapService::clearEntirely()
{
  //获取Costmap的互斥锁并锁定，以确保在重置所有层时的线程安全。
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));

  //重置Costmap中的所有层，这意味着清除所有层上的障碍数据。
  costmap_.resetLayers();
}

//获取机器人当前位置的辅助函数
bool ClearCostmapService::getPosition(double & x, double & y) const
{
  //定义一个用于存储机器人当前位置的PoseStamped消息。
  geometry_msgs::msg::PoseStamped pose;

  //尝试从Costmap中获取机器人的当前位置。
  if (!costmap_.getRobotPose(pose)) 
  {
    //如果无法获取位置，则返回false。
    return false;
  }

  //如果成功获取位置，将机器人的x、y坐标从pose中提取出来。
  x = pose.pose.position.x;
  y = pose.pose.position.y;

  //返回true，表示成功获取到了位置信息。
  return true;
}
}  
