/*
  这个文件的主要作用是检查给定位姿数据是否与已知障碍物冲突，通过订阅代价图(costmap)和机器人足迹(footprint)数据来完成这一任务。
  主要有三个用途：
  1.判断一个给定的机器人姿态是否与环境中的障碍物冲突。
  2.计算给定位姿的代价评分，代价评分高于某个阈值表示位置可能有障碍物。
  3.获取和更新当前的代价图和机器人足迹以进行碰撞检查。
*/
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

//这是个碰撞检查器类定义
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
//这个头文件是异常处理类定义
#include "nav2_costmap_2d/exceptions.hpp"
//这个头文件是足迹处理相关类定义
#include "nav2_costmap_2d/footprint.hpp"
//这个是直线迭代器工具，用于图形算法中
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
//构造函数，第一个参数是代价图订阅器引用，第二个参数是足迹订阅器引用，第三个参数是实例名称
CostmapTopicCollisionChecker::CostmapTopicCollisionChecker(CostmapSubscriber & costmap_sub, FootprintSubscriber & footprint_sub,std::string name): name_(name),
  costmap_sub_(costmap_sub),
  footprint_sub_(footprint_sub),
  collision_checker_(nullptr)
{}

//判断给定姿态是否无碰撞，第一个参数是机器人的二维姿态，第二个参数是bool类型的判断是否重新获取最新的代价图和足迹
bool CostmapTopicCollisionChecker::isCollisionFree(const geometry_msgs::msg::Pose2D & pose, bool fetch_costmap_and_footprint)
{
  try 
  {
    //如果姿态的评分表示致命障碍，则返回冲突
    //这个scorePose()函数在costmap_topic_collision_checker.hpp里面，专门用来计算姿态评分的
    if (scorePose(pose, fetch_costmap_and_footprint) >= LETHAL_OBSTACLE) 
    {
      return false;
    }
    return true;  //否则返回无碰撞
  } 
  /*
    详细说明一下IllegalPoseException异常：
    IllegalPoseException 通常用于指示给定的姿态（位置和方向）无效或不合法。
    这可能是因为姿态超出了地图的边界、位于障碍物内部，或者是因为它以其他方式不符合期望的参数。
  */
  catch (const IllegalPoseException & e)  //这里捕获的是捕获非法姿态异常
  {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());  //输出错误日志
    return false;  //返回冲突
  } 
  /*
    详细说明一下CollisionCheckerException异常：
    在CostmapTopicCollisionChecker类中，如果一个姿态的转换(从世界坐标到代价图的网格坐标)失败，比如，因为姿态位于地图外的情况，就会抛出此异常。
    这里会提醒调用者该姿态在当前的导航context中是不可用的。
  */
  catch (const CollisionCheckerException & e)  //这里捕获的是碰撞检查异常
  {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } 
  catch (...)  //捕获其他所有异常
  {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "Failed to check pose score!");
    return false;
  }
}

//计算给定姿态的碰撞评分，第一个参数是机器人的二维姿态，第二个bool类型是否重新获取最新的代价图和足迹
double CostmapTopicCollisionChecker::scorePose(const geometry_msgs::msg::Pose2D & pose, bool fetch_costmap_and_footprint)
{
  //如果需要获取最新的代价图和足迹
  if (fetch_costmap_and_footprint) 
  {
    try 
    {
      //设置碰撞检查器的代价图
      collision_checker_.setCostmap(costmap_sub_.getCostmap());
    } 
    catch (const std::runtime_error & e)   //这里捕获运行时异常
    {
      throw CollisionCheckerException(e.what());  //这里抛出碰撞检查异常
    }
  }

  //代价图中的单元格坐标
  unsigned int cell_x, cell_y;
  if (!collision_checker_.worldToMap(pose.x, pose.y, cell_x, cell_y))  //将世界坐标转换为代价图坐标
  {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", cell_x, cell_y);  //输出调试信息
    throw IllegalPoseException(name_, "Pose Goes Off Grid.");  //抛出非法姿态异常
  }

  //返回足迹成本
  return collision_checker_.footprintCost(getFootprint(pose, fetch_costmap_and_footprint));
}

//获取机器人当前足迹，第一个参数是机器人的二维姿态，第二个参数是是否获取最新的足迹
Footprint CostmapTopicCollisionChecker::getFootprint(const geometry_msgs::msg::Pose2D & pose, bool fetch_latest_footprint)
{
  if (fetch_latest_footprint)  //检查是否需要从订阅的源获取最新的足迹数据
  {
    std_msgs::msg::Header header;  //创建一个头部信息对象，用于后续获取
    if (!footprint_sub_.getFootprintInRobotFrame(footprint_, header))   //调用足迹订阅对象的getFootprintInRobotFrame方法，尝试获取最新的足迹数据
    {
      throw CollisionCheckerException("Current footprint not available.");  //如果获取失败(返回false)，则抛出异常
    }
  }
  //创建一个Footprint对象，用于存储和返回变换后的足迹
  Footprint footprint;

  //调用transformFootprint函数，使用给定的姿态信息(pose.x, pose.y, pose.theta)和当前足迹(footprint_)来生成新的足迹(footprint)，该足迹是变换到全局坐标系中的
  transformFootprint(pose.x, pose.y, pose.theta, footprint_, footprint);

  //返回变换后的足迹
  return footprint;
}

}
