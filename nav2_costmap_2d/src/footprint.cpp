/*
  这个文件实现了对机器人足迹footprint的处理，主要包括了计算足迹成本、转换足迹形状、调整足迹尺寸和从字符串生成足迹、足迹填充等。
  为机器人在代价地图上的导航提供了基础，使其能够考虑到机器人的物理尺寸和形状，从而进行更安全和更有效的路径规划和避障。
*/
#include "nav2_costmap_2d/footprint.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_costmap_2d/array_parser.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

//这个文件里涉及到的很多数学公式，这里就不做过多说明了
namespace nav2_costmap_2d
{
//计算给定足迹中点到原点(0,0)的最小和最大距离
void calculateMinAndMaxDistances(const std::vector<geometry_msgs::msg::Point> & footprint, double & min_dist, double & max_dist)
{
  //初始化最小距离为最大可能值，以便任何实际距离都会更小
  min_dist = std::numeric_limits<double>::max();
  //初始化最大距离为0，这样任何实际距离都会更大
  max_dist = 0.0;

  //如果足迹点少于或等于2个，不进行计算，直接返回
  if (footprint.size() <= 2) 
  {
    return;
  }
  //遍历足迹中的每一对连续点
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) 
  {
    //用距离函数计算当前点到原点的直线距离
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    //计算当前点与下一点连线到原点的最短距离
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y, footprint[i + 1].x, footprint[i + 1].y);
    //更新最小距离
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    //更新最大距离
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  //也要检查最后一个点与第一个点形成的边
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y, footprint.front().x, footprint.front().y);
  //再次更新最小和最大距离
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

//将geometry_msgs::msg::Point类型转换为geometry_msgs::msg::Point32类型
geometry_msgs::msg::Point32 toPoint32(geometry_msgs::msg::Point pt)
{
  geometry_msgs::msg::Point32 point32;
  //直接赋值x, y, z坐标
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  //返回转换后的Point32类型点
  return point32;
}

//将geometry_msgs::msg::Point32类型转换为 geometry_msgs::msg::Point类型
geometry_msgs::msg::Point toPoint(geometry_msgs::msg::Point32 pt)
{
  geometry_msgs::msg::Point point;
  //赋值x, y, z坐标
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  //返回转换后的Point类型点
  return point;
}

/*
  这里补充，什么是polygon类型：
    geometry_msgs::msg::Polygon是一个用于表示多边形区域的标准消息类型。通常用于定义机器人的感知区域、碰撞检测区域或其他任何需要定义一组点围成的封闭区域的应用。
*/
//将geometry_msgs::msg::Point类型的向量转换为geometry_msgs::msg::Polygon类型
geometry_msgs::msg::Polygon toPolygon(std::vector<geometry_msgs::msg::Point> pts)
{
  //创建一个Polygon消息对象
  geometry_msgs::msg::Polygon polygon;
  //遍历提供的点的向量
  for (unsigned int i = 0; i < pts.size(); i++) 
  {
    //将每个Point转换为Point32类型并添加到Polygon的points向量中
    polygon.points.push_back(toPoint32(pts[i]));
  }
  //返回构建好的Polygon对象
  return polygon;
}

//将geometry_msgs::msg::Polygon类型转换为geometry_msgs::msg::Point类型的向量
std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
{
  //创建一个Point的向量
  std::vector<geometry_msgs::msg::Point> pts;
  //遍历Polygon中的每个点
  for (unsigned int i = 0; i < polygon->points.size(); i++) 
  {
    //将每个Point32类型转换为Point并添加到向量中
    pts.push_back(toPoint(polygon->points[i]));
  }
  //返回转换后的点的向量
  return pts;
}

//将机器人的足迹根据指定的位置和角度进行变换
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec, std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  //调整目标足迹向量的大小与原始足迹相同
  oriented_footprint.resize(footprint_spec.size());
  //计算给定角度的余弦值
  double cos_th = cos(theta);
  //计算给定角度的正弦值
  double sin_th = sin(theta);
  //遍历所有足迹点
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) 
  {
    //计算新的x, y坐标
    double new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    double new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    //获取对应的点引用
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    //设置新的x坐标
    new_pt.x = new_x;
    //设置新的y坐标
    new_pt.y = new_y;
  }
}

//变换足迹根据给定的位置(x, y)和角度(theta)
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec, geometry_msgs::msg::PolygonStamped & oriented_footprint)
{
  //清空目标多边形中的点
  oriented_footprint.polygon.points.clear();
  //计算给定角度的余弦值
  double cos_th = cos(theta);
  //计算给定角度的正弦值
  double sin_th = sin(theta);
  //遍历每一个足迹点
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) 
  {
    //创建新的Point32对象
    geometry_msgs::msg::Point32 new_pt;
    //根据旋转矩阵计算新的x坐标
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    //根据旋转矩阵计算新的y坐标
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    //将新点添加到多边形中
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

//在足迹的每个点上添加指定的填充(padding)
void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding)
{
  //遍历足迹中的每个点
  for (unsigned int i = 0; i < footprint.size(); i++) 
  {
    //获取对应点的引用
    geometry_msgs::msg::Point & pt = footprint[i];
    //根据x坐标的符号给x坐标添加或减去填充
    pt.x += sign0(pt.x) * padding;
    //根据y坐标的符号给y坐标添加或减去填充
    pt.y += sign0(pt.y) * padding;
  }
}

//根据指定半径生成一个圆形足迹
std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius)
{
  //创建点的向量
  std::vector<geometry_msgs::msg::Point> points;

  //定义将圆分割成多少个点
  int N = 16;
  //创建一个点对象
  geometry_msgs::msg::Point pt;
  //循环N次以生成圆上的点
  for (int i = 0; i < N; ++i) 
  {
    //计算当前点对应的角度
    double angle = i * 2 * M_PI / N;
    //根据角度和半径计算x坐标
    pt.x = cos(angle) * radius;
    //根据角度和半径计算y坐标
    pt.y = sin(angle) * radius;
    //将计算出的点添加到向量中
    points.push_back(pt);
  }
  //返回生成的圆形足迹点集
  return points;
}

//尝试从字符串中解析机器人足迹
bool makeFootprintFromString(const std::string & footprint_string, std::vector<geometry_msgs::msg::Point> & footprint)
{
  //用于存储解析中发生的错误消息
  std::string error;
  //调用parseVVF将字符串解析为浮点数的向量的向量，每个向量代表一个点的坐标
  std::vector<std::vector<float>> vvf = parseVVF(footprint_string, error);

  //检查解析过程中是否有错误发生
  if (error != "") 
  {
    //如果有错误，记录错误日志
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "Error parsing footprint parameter: '%s'", error.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "  Footprint string was '%s'.", footprint_string.c_str());
    //返回false表示解析失败
    return false;
  }

  //检查解析出的点的数量是否至少为3，因为足迹需要至少三个点来定义一个封闭形状
  if (vvf.size() < 3) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),"You must specify at least three points for the robot footprint, reverting to previous footprint."); //NOLINT
    //解析的点不足以构成足迹
    return false;
  }
  //预留足够的空间来存储足迹点
  footprint.reserve(vvf.size());
  //遍历解析出的每个点的数据
  for (unsigned int i = 0; i < vvf.size(); i++) 
  {
    //检查每个点的数据是否恰好包含两个数字(x 和 y 坐标)
    if (vvf[i].size() == 2) 
    {
      //创建一个点对象
      geometry_msgs::msg::Point point;
      //设置x坐标
      point.x = vvf[i][0];
      //设置y坐标
      point.y = vvf[i][1];
      //二维平面，z坐标为0
      point.z = 0;
      //将点添加到足迹向量中
      footprint.push_back(point);
    } 
    else  //如果点的数据不是两个数字，记录错误并返回 false
    {
      RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),"Points in the footprint specification must be pairs of numbers. Found a point with %d numbers.", static_cast<int>(vvf[i].size()));
      return false;
    }
  }
  //解析成功，返回true
  return true;
}

}  
