/*
  这个文件实现了一个多层代价地图的功能，这是在机器人导航中用于处理和合并多个地图层的关键组件，主要实现了对多层代价地图的管理，动态调整和更新，和安全性检查等功能。
*/
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_costmap_2d/footprint.hpp"

using std::vector;

namespace nav2_costmap_2d
{
//构造函数
LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
  : primary_costmap_(),  //初始化主代价地图
  combined_costmap_(),  //初始化组合代价地图
  global_frame_(global_frame),  //设置全局参考框架
  rolling_window_(rolling_window),   //设置是否使用滚动窗口
  current_(false),   //初始状态设置为非当前
  minx_(0.0),  //初始化最小边界值
  miny_(0.0),  
  maxx_(0.0),  //初始化最大边界值
  maxy_(0.0),
  bx0_(0),  //初始化边界索引
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),  //设置为未初始化
  size_locked_(false),  //设置大小未锁定
  circumscribed_radius_(1.0),  //设置外接圆半径
  inscribed_radius_(0.1)  //设置内切圆半径
{
  //如果跟踪未知区域
  if (track_unknown) 
  {
    //主地图默认值设置为255(未知区域)
    primary_costmap_.setDefaultValue(255);
    //组合地图默认值设置为255(未知区域)
    combined_costmap_.setDefaultValue(255);
  } 
  else  //如果不跟踪未知区域
  {
    //主地图默认值设置为0(无障碍)
    primary_costmap_.setDefaultValue(0);
    //组合地图默认值设置为0
    combined_costmap_.setDefaultValue(0);
  }
}

//析构函数
LayeredCostmap::~LayeredCostmap()
{
  //当还有插件存在时
  while (plugins_.size() > 0) 
  {
    //移除最后一个插件
    plugins_.pop_back();
  }
  //当还有过滤器存在时
  while (filters_.size() > 0) 
  {
    //移除最后一个过滤器
    filters_.pop_back();
  }
}

//向LayeredCostmap添加一个新的层插件
void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
  //使用互斥锁保护组合代价地图的更新过程，防止数据竞争
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  //将传入的插件添加到插件列表中
  plugins_.push_back(plugin);
}

//向LayeredCostmap添加一个新的过滤器层
void LayeredCostmap::addFilter(std::shared_ptr<Layer> filter)
{
  //同样使用互斥锁防止竞争
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  //将传入的过滤器添加到过滤器列表中
  filters_.push_back(filter);
}

//调整LayeredCostmap中的代价地图尺寸
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y, bool size_locked)
{
  //使用互斥锁保护组合代价地图调整尺寸的过程
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  //设置地图尺寸是否锁定的状态
  size_locked_ = size_locked;
  //调整主代价地图的尺寸和分辨率
  primary_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  //调整组合代价地图的尺寸和分辨率
  combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  //遍历所有插件
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
  {
    //对每一个插件调用matchSize()方法
    //这个方法在插件类中定义，主要用来保证插件的内部数据结构与主地图的尺寸同步
    (*plugin)->matchSize();
  }
  //遍历所有过滤器
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin(); filter != filters_.end(); ++filter)
  {
    //对每一个过滤器也调用matchSize()方法
    //过滤器也是层的一种，通常包含修改或影响最终代价地图数据的功能
    //matchSize()确保过滤器处理的数据结构与地图的当前尺寸保持一致
    (*filter)->matchSize();
  }
}

//检查给定坐标(robot_x, robot_y)是否在代价地图的边界之外
bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  //使用combined_costmap_的 worldToMap 方法转换世界坐标到地图坐标
  //如果坐标在地图范围内，返回false，表示不超出边界
  //如果坐标不在地图范围内，返回true，表示超出边界
  return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

//根据给定的机器人位置和姿态更新地图的状态，包括合并层的数据和检查边界(这个破函数又臭又长)
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  //加锁保护，确保更新操作不会与其他线程发生冲突
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));

  //如果是滚动窗口地图，需要根据机器人的当前位置更新地图的原点
  if (rolling_window_) 
  {
    //计算新的原点位置
    double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
    //更新原点
    primary_costmap_.updateOrigin(new_origin_x, new_origin_y);
    combined_costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  //检查机器人是否处于地图边界之外
  if (isOutofBounds(robot_x, robot_y)) 
  {
    //如果机器人位置超出地图边界，打印警告信息
    RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Robot is out of bounds of the costmap!");
  }

  //如果没有任何插件和过滤器，直接返回
  if (plugins_.size() == 0 && filters_.size() == 0) 
  {
    return;
  }

  //初始化最小和最大边界值
  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  //更新所有插件的边界
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
  {
    //保存当前边界值(最大最小值)
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    //调用插件的updateBounds方法更新边界
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    //检查边界值是否合法，不合法则警告
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) 
    {
      //这个日志的内容真是又臭又长，详细说一下，这个日志信息主要描述了边界变化前后的情况，继续描述新的边界情况并指出引起问题的层
      RCLCPP_WARN(
        rclcpp::get_logger("nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,  //这些参数代表更新前的边界值：左上角(top-left)和右下角(bottom-right)
        minx_, miny_, maxx_, maxy_,  //这些参数代表更新后的边界值
        (*plugin)->getName().c_str());  //获取引起问题的插件的名称，并转换为C风格字符串
    }
  }
  //开始遍历已注册的过滤器列表
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin(); filter != filters_.end(); ++filter)
  {
    //存储当前边界值(最大最小值)
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    //调用当前过滤器的updateBounds方法更新地图的边界
    (*filter)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    //检查是否有非法的边界变化
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) 
    {
      //这里的警告也是如果边界非法地变化，就弹出，不详细说明内容了，跟上面差不多
      RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending filter is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*filter)->getName().c_str());
    }
  }
  //定义四个变量来存储转换后的地图网格坐标
  int x0, xn, y0, yn;
  //将左下角的世界坐标(minx_, miny_)转换为地图的网格坐标(x0, y0)
  //worldToMapEnforceBounds确保即使坐标位于地图外部，也会被限制在地图的边界内
  combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  //将右上角的世界坐标(maxx_, maxy_)转换为地图的网格坐标(xn, yn)
  //这个函数同样确保坐标值不会超出地图的实际范围
  combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  //设置x0为0和x0中的较大值，确保x0不小于0。
  x0 = std::max(0, x0);
  //设置xn为地图宽度和xn+1中的较小值，确保xn不大于地图的最大宽度。
  xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
  //设置y0为0和y0中的较大值，确保y0不小于0。
  y0 = std::max(0, y0);
  //设置yn为地图高度和yn+1中的较小值，确保yn不大于地图的最大高度。
  yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

  //打印正在更新的区域。
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);
  //如果更新区域无效，则返回。
  if (xn < x0 || yn < y0) 
  {
    return;
  }
  //根据是否启用过滤器，选择不同的更新策略。
  if (filters_.size() == 0) 
  { 
    //如果没有启用过滤器，直接通过插件更新地图。
    combined_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
      //调用每个插件的更新成本方法，更新地图区域
      (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  } 
  else  
  {
    //如果启用了过滤器，先通过插件更新一个原始地图，然后复制到最终地图，并应用过滤器。
    primary_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(primary_costmap_, x0, y0, xn, yn);
    }
    //尝试将更新后的原始地图复制到最终地图，如果复制失败则抛出异常
    if (!combined_costmap_.copyWindow(primary_costmap_, x0, y0, xn, yn, x0, y0)) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),"Can not copy costmap (%i,%i)..(%i,%i) window",x0, y0, xn, yn);
      throw std::runtime_error{"Can not copy costmap"};
    }
    //应用过滤器更新最终地图
    for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();filter != filters_.end(); ++filter)
    {
      (*filter)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  }
  //记录更新的地图边界，用于下次更新
  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;
  //标记地图为已初始化
  initialized_ = true;
}

//检查层叠代价地图的当前状态
bool LayeredCostmap::isCurrent()
{
  //设置当前状态为真，假设所有层都是最新的。
  current_ = true;
  //遍历所有插件，检查它们是否最新或已禁用。
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
  {
    //如果插件不是最新的并且启用，设置当前状态为假。
    current_ = current_ && ((*plugin)->isCurrent() || !(*plugin)->isEnabled());
  }
  //遍历所有过滤器，检查它们是否最新或已禁用。
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin(); filter != filters_.end(); ++filter)
  {
    //如果过滤器不是最新的并且启用，设置当前状态为假。
    current_ = current_ && ((*filter)->isCurrent() || !(*filter)->isEnabled());
  }
  //返回当前状态，如果所有启用的层都是最新的，则为true，否则为false。
  return current_;
}

//设置代价地图的足迹
void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  //设置层叠代价地图的足迹。
  footprint_ = footprint_spec;
  //计算足迹的内接半径和外接半径。
  nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  //遍历所有插件，通知它们足迹已经改变。
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
  //遍历所有过滤器，通知它们足迹已经改变。
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin(); filter != filters_.end(); ++filter)
  {
    (*filter)->onFootprintChanged();
  }
}

} 
