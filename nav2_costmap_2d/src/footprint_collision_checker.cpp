/*
  这个文件提供了一个框架，用来检测一个给定的“足迹”(通常是机器人的底盘轮廓)在一个代价地图中是否会发生碰撞。
  这里的功能主要用于机器人导航中的碰撞检测，评估机器人在给定位置和姿态时的碰撞风险。
*/
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
/*
  下面这两个构造函数，一种是创建时不立即关联代价地图，适用于可能需要稍后设置或更改代价地图的情况；另一种是创建时立即关联到一个特定的代价地图，适用于已知代价地图环境的情况
*/
//这个构造函数是一个无参构造，它初始化FootprintCollisionChecker对象，其中成员变量costmap_被设置为 nullptr。这表明这个对象并没有关联到一个具体的代价地图实例。
template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(): costmap_(nullptr)
{}

//这个构造函数接受一个CostmapT类型的参数，并直接将其赋值给成员变量costmap_。这意味着创建FootprintCollisionChecker对象时，可以直接与一个已存在的代价地图实例关联。
template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(CostmapT costmap): costmap_(costmap)
{}

/*
  补充一下，什么叫足迹在代价地图上的成本：
    在机器人导航和路径规划中，“成本”是一个重要的概念，这个成本衡量的是机器人在一定形状和大小的足迹(所谓的足迹就是机器人底部的轮廓,在代价地图中，通常被视为一组构成该轮廓的点的集合)下，通过代价地图中某个特定区域的难度或风险。
    代价地图是一个二维网格，每个格子(或像素)代表空间中对应区域的通过成本。这些成本值通常根据地面条件、障碍物存在与否、地形类型等因素来设定。
  成本的计算：
    在代价地图中，每个格子的值代表通过该区域的成本。成本的具体值可以表示为：
      无信息(NO_INFORMATION)：代表该区域的成本未知。
      自由空间(FREE_SPACE)：代表该区域无障碍，通过成本低。
      致命障碍(LETHAL_OBSTACLE)：代表该区域有障碍物，不能通过。
      当评估一个给定的足迹时，通常需要计算足迹覆盖的所有点的成本，并根据特定的规则，如取最大值、平均值等综合这些成本来得出总的“足迹成本”
    成本评估的作用：
      主要用于以下三个途径：
        路径规划：在规划路径时，路径规划算法会尽量避开高成本区域，寻找成本最低的路径。
        碰撞检测：在移动前，机器人可以评估其预定路径上的足迹成本，以确定该路径是否可行。
        动态避障：在实时导航中，机器人可以根据周围环境的变化动态调整路径，避开新出现的障碍。
*/
//这个模板用来计算给定足迹在代价地图上的成本
template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCost(const Footprint footprint)
{
  //初始化变量，用于存储足迹点在地图上的坐标
  unsigned int x0, x1, y0, y1;
  //足迹成本的初始值设为0
  double footprint_cost = 0.0;

  //将足迹的第一个点转换为地图坐标，检查是否成功
  if (!worldToMap(footprint[0].x, footprint[0].y, x0, y0)) 
  {
    //如果转换失败，返回致命障碍物成本
    return static_cast<double>(LETHAL_OBSTACLE);
  }

  //保存足迹的起始坐标
  unsigned int xstart = x0;
  unsigned int ystart = y0;

  //遍历足迹中的所有点
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) 
  {
    //尝试将下一个足迹点转换为地图坐标
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) 
    {
      //如果转换失败，返回致命障碍物成本
      return static_cast<double>(LETHAL_OBSTACLE);
    }

    //计算当前线段的成本，并与已有的最大成本比较
    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

    //更新坐标，为下一个线段做准备
    x0 = x1;
    y0 = y1;

    //如果任何线段的成本达到致命障碍物成本，立即返回
    if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE)) 
    {
      return footprint_cost;
    }
  }

  //计算最后一个点回到起始点的线段成本，并返回整个足迹的最大成本
  return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
}

//这个模板用于计算从(x0, y0)到(x1, y1)，即两个点的直线路径上的最大成本
template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::lineCost(int x0, int x1, int y0, int y1) const
{
  //初始化线路成本为0.0
  double line_cost = 0.0;
  //初始化点成本为-1.0(无效值)
  double point_cost = -1.0;

  //使用LineIterator迭代器遍历从(x0, y0)到(x1, y1)的所有点
  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) 
  {
    //获取当前点的成本
    point_cost = pointCost(line.getX(), line.getY());   

    //如果点成本为致命障碍
    if (point_cost == static_cast<double>(LETHAL_OBSTACLE)) 
    {
      //直接返回这个致命成本
      return point_cost;
    }

    //如果当前线路成本小于点成本
    if (line_cost < point_cost) 
    {
      //更新线路成本为较大值
      line_cost = point_cost;
    }
  }
  //返回计算得到的线路成本
  return line_cost;
}

//这个模板用于将世界坐标(wx, wy)转换为地图坐标(mx, my)
template<typename CostmapT>
bool FootprintCollisionChecker<CostmapT>::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  //调用代价地图的worldToMap函数进行坐标转换
  return costmap_->worldToMap(wx, wy, mx, my);
}

//这个模板用于获取地图上点(x, y)的成本
template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::pointCost(int x, int y) const
{
  //调用代价地图的getCost方法获取点的成本
  return costmap_->getCost(x, y);
}

//这个模板用于设置用于碰撞检查的代价地图
template<typename CostmapT>
void FootprintCollisionChecker<CostmapT>::setCostmap(CostmapT costmap)
{
  //将成员变量costmap_设置为提供的代价地图实例
  costmap_ = costmap;
}

//这个模板用于计算在指定位置和姿态下的足迹成本
template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCostAtPose(double x, double y, double theta, const Footprint footprint)
{
  //计算给定角度的余弦值，用于旋转足迹
  double cos_th = cos(theta);
  //计算给定角度的正弦值，用于旋转足迹
  double sin_th = sin(theta);
  //创建一个新的足迹容器，用于存储旋转后的足迹
  Footprint oriented_footprint;
  //遍历原始足迹中的每个点
  for (unsigned int i = 0; i < footprint.size(); ++i) 
  {
    //创建一个新的点结构
    geometry_msgs::msg::Point new_pt;
    //通过旋转公式计算新的x坐标
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    //通过旋转公式计算新的y坐标
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    //将计算后的点添加到旋转后的足迹中
    oriented_footprint.push_back(new_pt);
  }
  //调用footprintCost函数计算旋转后足迹的成本，并返回这个成本
  return footprintCost(oriented_footprint);
}

/*
  最后这两个累模板的实例化，一次为std::shared_ptr<nav2_costmap_2d::Costmap2D>类型的代价地图。
  另一次为指针类型nav2_costmap_2d::Costmap2D*。这样做是为了确保这两种常用的代价地图引用类型都可以在编译时被正确处理。
*/
template class FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>;
template class FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;

}
