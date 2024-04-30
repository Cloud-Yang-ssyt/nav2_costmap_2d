/*
  这个文件的主要用于创建和管理代价地图，涉及到地图的初始化与重置，代价值管理，地图与世界坐标的转换以及其他功能
*/
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{
//构造函数定义，初始化列表设置成员变量
Costmap2D::Costmap2D(
  unsigned int cells_size_x,  //地图的宽度，以格子数表示
  unsigned int cells_size_y,  //地图的高度，以格子数表示
  double resolution,  //每个格子的分辨率
  double origin_x,   //地图在世界坐标中的原点x坐标
  double origin_y,   //地图在世界坐标中的原点y坐标
  unsigned char default_value)  //默认的格子值
  : size_x_(cells_size_x),  //初始化成员变量格子宽度
  size_y_(cells_size_y),   //初始化成员变量格子高度
  resolution_(resolution),  //初始化分辨率
  origin_x_(origin_x),  //初始化原点x坐标
  origin_y_(origin_y),  //初始化原点y坐标
  costmap_(NULL),   //初始化地图指针为NULL
  default_value_(default_value)  //初始化默认值
{
  //创建一个新的互斥锁对象
  access_ = new mutex_t();

  //初始化地图数组
  initMaps(size_x_, size_y_);
  //将所有格子重置为默认值
  resetMaps();
}

//初始化成本地图，将默认值设置为FREE_SPACE
Costmap2D::Costmap2D(const nav_msgs::msg::OccupancyGrid & map): default_value_(FREE_SPACE)
{
  //创建一个新的互斥锁对象，用于线程安全访问成本地图
  access_ = new mutex_t();

  //设置成本地图的宽度为地图的宽度
  size_x_ = map.info.width;
  //设置成本地图的高度为地图的高度
  size_y_ = map.info.height;
  //设置成本地图中每个格子的边长，表示现实世界中一个格子的大小
  resolution_ = map.info.resolution;
  //设置成本地图的x轴原点，通常表示成本地图左下角的x坐标
  origin_x_ = map.info.origin.position.x;
  //设置成本地图的y轴原点，通常表示成本地图左下角的y坐标
  origin_y_ = map.info.origin.position.y;

  //分配内存来存储成本值，数组大小为地图的格子总数
  costmap_ = new unsigned char[size_x_ * size_y_];

  //定义一个8位整型变量来存储地图中的数据值
  int8_t data;
  //遍历地图的所有格子
  for (unsigned int it = 0; it < size_x_ * size_y_; it++) 
  {
    //从地图数据中获取当前格子的值
    data = map.data[it];
    //检查当前格子是否为未知状态
    if (data == nav2_util::OCC_GRID_UNKNOWN) 
    {
      //如果是未知，将成本地图对应的格子设置为无信息
      costmap_[it] = NO_INFORMATION;
    } 
    else  //如果格子状态已知
    {
      //根据占用格子百分比值，转换成成本值。这个转换基于一个线性缩放，将占用概率映射到成本地图中的一个范围，这里是从FREE_SPACE到LETHAL_OBSTACLE
      costmap_[it] = std::round(static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) / (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE));
    }
  }
}

//用来删除代价地图的函数
void Costmap2D::deleteMaps()
{
  //使用mutex进行线程同步，确保在操作地图数据时不会出现数据竞争
  std::unique_lock<mutex_t> lock(*access_);
  //删除动态分配的代价地图数组
  delete[] costmap_;
  //将代价地图指针设置为NULL，避免野指针问题
  costmap_ = NULL;
}

//初始化代价地图
void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  //同样使用mutex进行线程同步，保护成本地图数据
  std::unique_lock<mutex_t> lock(*access_);
  //在创建新地图前，删除旧的成本地图数组，防止内存泄漏
  delete[] costmap_;
  //根据指定的宽度和高度，动态分配一个新的成本地图数组
  costmap_ = new unsigned char[size_x * size_y];
}

//重置代价地图的尺寸、分辨率和原点位置
void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
  //设置地图的宽度
  size_x_ = size_x;
  //设置地图的高度
  size_y_ = size_y;
  //设置地图分辨率
  resolution_ = resolution;
  //设置地图原点的x坐标
  origin_x_ = origin_x;
  //设置地图原点的y坐标
  origin_y_ = origin_y;

  //初始化地图存储空间
  initMaps(size_x, size_y);
  //重置地图中的所有代价值
  resetMaps();
}

//重置地图中的所有代价值为默认值
void Costmap2D::resetMaps()
{
  //获取对代价地图的独占访问
  std::unique_lock<mutex_t> lock(*access_);
  //使用默认值填充整个地图
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

//重置特定区域的代价地图到默认值
void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  //调用函数将指定区域重置为默认值
  resetMapToValue(x0, y0, xn, yn, default_value_);
}

//重置地图中指定矩形区域的代价值
void Costmap2D::resetMapToValue(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value)
{
  //使用互斥锁保证线程安全，锁定对地图数据的访问
  std::unique_lock<mutex_t> lock(*(access_));
  //计算需要重置的列数
  unsigned int len = xn - x0;
  //遍历每一行需要重置的区域
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) 
  {
    //使用memset函数填充选定区域，将其设置为特定的代价值
    memset(costmap_ + y, value, len * sizeof(unsigned char));
  }
}

//定义函数，用于从另一个代价地图的指定窗口区域复制代价数据到当前代价地图
bool Costmap2D::copyCostmapWindow(const Costmap2D & map, double win_origin_x, double win_origin_y, double win_size_x, double win_size_y)
{
  //检查是否是尝试将代价地图复制到自身，如果是，则直接返回false，表示操作无效
  if (this == &map) 
  {
    return false;
  }

  //删除当前代价地图中的所有数据，准备重新分配和填充数据
  deleteMaps();

  //定义变量以存储窗口边界在原始地图上的对应单元格坐标
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  //转换窗口左下角和右上角的世界坐标到原始地图的单元格坐标，转换失败则返回false
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y))
  {
    return false;
  }

  //计算新代价地图的尺寸，即窗口的宽度和高度转换成单元格数量
  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  //保留原始代价地图的分辨率设置
  resolution_ = map.resolution_;
  //设置新代价地图的原点坐标
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  //初始化新的代价地图，为其分配内存
  initMaps(size_x_, size_y_);

  //从原始代价地图复制数据到新代价地图的相应区域
  copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
  //复制成功，返回true
  return true;
}

//定义函数以从一个源代价地图的指定窗口区域复制数据到当前代价地图的指定位置
bool Costmap2D::copyWindow(const Costmap2D & source, unsigned int sx0, unsigned int sy0, unsigned int sxn, unsigned int syn, unsigned int dx0, unsigned int dy0)
{
  //计算源窗口的宽度和高度
  const unsigned int sz_x = sxn - sx0;
  const unsigned int sz_y = syn - sy0;

  //检查源窗口的上界是否超出源代价地图的范围
  if (sxn > source.getSizeInCellsX() || syn > source.getSizeInCellsY()) 
  {
    //如果超出范围，则返回false，表示复制失败
    return false;
  }

  //检查目标位置加上窗口大小是否会超出当前代价地图的范围
  if (dx0 + sz_x > size_x_ || dy0 + sz_y > size_y_) 
  {
    //如果超出范围，则返回false，表示复制失败
    return false;
  }
  
  //使用辅助函数将源代价地图的指定区域复制到当前代价地图的指定区域
  copyMapRegion(source.costmap_, sx0, sy0, source.size_x_, costmap_, dx0, dy0, size_x_, sz_x, sz_y);
  //复制成功，返回true
  return true;
}

//重载赋值运算符，以支持代价地图之间的赋值操作
Costmap2D & Costmap2D::operator=(const Costmap2D & map)
{
  //检查自赋值情况
  if (this == &map) 
  {
    //如果是自赋值，则直接返回本身
    return *this;
  }

  //删除当前代价地图的数据
  deleteMaps();

  //复制宽度
  size_x_ = map.size_x_;
  //复制高度
  size_y_ = map.size_y_;
  //复制分辨率
  resolution_ = map.resolution_;
  //复制x方向的起始坐标
  origin_x_ = map.origin_x_;
  //复制y方向的起始坐标
  origin_y_ = map.origin_y_;

  //初始化新的代价地图数组
  initMaps(size_x_, size_y_);
  //复制代价地图数据
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
  //返回当前对象的引用
  return *this;
}

//拷贝构造函数
Costmap2D::Costmap2D(const Costmap2D & map): costmap_(NULL)
{
  //初始化互斥锁
  access_ = new mutex_t();
  //使用赋值运算符完成拷贝
  *this = map;
}

//默认构造函数
Costmap2D::Costmap2D(): size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  //初始化互斥锁
  access_ = new mutex_t();
}

//析构函数
Costmap2D::~Costmap2D()
{
  //删除代价地图信息
  deleteMaps();
  //删除互斥锁信息
  delete access_;
}

//计算给定世界坐标距离在代价地图中的单元格数
unsigned int Costmap2D::cellDistance(double world_dist)
{
  //将世界距离转换为网格单元数并向上取整，确保非负
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  //返回单元格距离的整数值
  return (unsigned int)cells_dist;
}

//获取代价地图的原始数组指针
unsigned char * Costmap2D::getCharMap() const
{
  //返回指向代价地图数组的指针
  return costmap_;
}

//根据网格坐标(mx, my)获取单元格的代价值
unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  //使用getIndex转换二维坐标到一维索引并获取代价值
  return costmap_[getIndex(mx, my)];
}

//根据一维数组索引获取单元格的代价值
unsigned char Costmap2D::getCost(unsigned int undex) const
{
  //直接通过一维索引访问数组获取代价值
  return costmap_[undex];
}

//设置特定网格坐标(mx, my)的代价值
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  //使用getIndex函数将二维坐标转换为一维索引，并设置该位置的代价
  costmap_[getIndex(mx, my)] = cost;
}

//将网格坐标(mx, my)转换为世界坐标(wx, wy)
void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  //计算x方向的世界坐标，包括原点偏移和中心对齐
  wx = origin_x_ + (mx + 0.5) * resolution_;
  //计算y方向的世界坐标，同样考虑原点偏移和中心对齐
  wy = origin_y_ + (my + 0.5) * resolution_;
}

//将世界坐标(wx, wy)转换为网格坐标(mx, my)
bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  //如果世界坐标在代价地图的原点之前，则不在地图内
  if (wx < origin_x_ || wy < origin_y_) 
  {
    return false;
  }

  //计算x方向的网格坐标
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  //计算y方向的网格坐标
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  //如果计算出的网格坐标在地图大小范围内
  if (mx < size_x_ && my < size_y_) 
  {
    return true;
  }
  //如果坐标超出地图范围，返回false
  return false;
}

//将世界坐标转换为网格坐标，不考虑坐标是否在地图范围内
void Costmap2D::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  //直接计算x方向的网格坐标，不检查边界
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  //直接计算y方向的网格坐标，同样不检查边界
  my = static_cast<int>((wy - origin_y_) / resolution_);
}

//将世界坐标(wx, wy)强制转换为网格坐标(mx, my)，同时确保坐标不超出地图边界
void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
  //如果给定的世界坐标x小于地图的原点x，则设置网格坐标mx为0(地图的左边界)
  if (wx < origin_x_) 
  {
    mx = 0;
  } 
  //如果给定的世界坐标x大于地图宽度和原点x的总和，则设置网格坐标mx为地图最大宽度减一(地图的右边界)
  else if (wx > resolution_ * size_x_ + origin_x_) 
  {
    mx = size_x_ - 1;
  } 
  else  //如果给定的世界坐标x在地图边界内，则通过计算将其转换为相应的网格坐标
  {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
  }

  //如果给定的世界坐标y小于地图的原点y，则设置网格坐标my为0(地图的下边界)
  if (wy < origin_y_) 
  {
    my = 0;
  } 
  //如果给定的世界坐标y大于地图高度和原点y的总和，则设置网格坐标my为地图最大高度减一(地图的上边界)
  else if (wy > resolution_ * size_y_ + origin_y_) 
  {
    my = size_y_ - 1;
  } 
  else  //如果给定的世界坐标y在地图边界内，则通过计算将其转换为相应的网格坐标
  {
    my = static_cast<int>((wy - origin_y_) / resolution_);
  }
}

//这个函数用于更新代价地图的原点，并调整地图内容以匹配新的原点
void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  //计算新原点相对于当前原点的网格偏移量
  int cell_ox, cell_oy;
  //计算x方向的偏移量
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  //计算y方向的偏移量
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  //计算新的网格原点坐标
  double new_grid_ox, new_grid_oy;
  //计算新原点x坐标
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  //计算新原点y坐标
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  //备份当前地图尺寸
  //当前地图宽度
  int size_x = size_x_;
  //当前地图高度
  int size_y = size_y_;

  //计算移动后新原点的左下和右上网格坐标
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  //计算新原点左下角x坐标
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  //计算新原点左下角y坐标
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  //计算新原点右上角x坐标
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  //计算新原点右上角y坐标
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  //计算新地图区域的大小
  //新地图宽度
  unsigned int cell_size_x = upper_right_x - lower_left_x;
  //新地图高度
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  //创建一个临时地图来存储新的地图区域,分配临时地图的内存空间
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  //从原地图复制相关区域到临时地图
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  //重置当前地图到默认值，以便使用新的原点
  resetMaps();

  //设置新的原点坐标，更新原点x坐标
  origin_x_ = new_grid_ox;
  //更新原点y坐标
  origin_y_ = new_grid_oy;

  //计算新地图数据在旧地图中的起始位置
  //计算在旧地图中的起始x坐标
  int start_x = lower_left_x - cell_ox;
  //计算在旧地图中的起始y坐标
  int start_y = lower_left_y - cell_oy;

  //将临时地图的数据复制回主地图
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  //释放分配给临时地图的内存
  delete[] local_map;
}

//定义一个设置多边形区域代价值的函数
bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::msg::Point> & polygon, unsigned char cost_value)
{
  //定义一个存储地图坐标的向量
  std::vector<MapLocation> map_polygon;
  //遍历多边形的每个顶点
  for (unsigned int i = 0; i < polygon.size(); ++i) 
  {
    //创建一个地图位置结构
    MapLocation loc;
    //将世界坐标转换为地图坐标，如果不在地图范围内，返回false
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) 
    {
      return false;
    }
    //添加到地图坐标向量
    map_polygon.push_back(loc);
  }

  //定义一个向量存储多边形内的单元格
  std::vector<MapLocation> polygon_cells;
  //填充多边形内的单元格
  convexFillCells(map_polygon, polygon_cells);
  //遍历多边形的所有单元格
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) 
  {
    //获取单元格在代价数组中的索引
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    //设置单元格的代价值
    costmap_[index] = cost_value;
  }
  //成功完成设置，返回true
  return true;
}

//定义一个函数用于生成多边形轮廓的单元格
void Costmap2D::polygonOutlineCells(const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells)
{
  //创建一个单元格收集器
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  //遍历多边形顶点，除了最后一个
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) 
  {
    //使用光线投射算法绘制从当前顶点到下一个顶点的直线
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  //如果多边形不为空
  if (!polygon.empty()) 
  {
    //获取多边形的最后一个索引
    unsigned int last_index = polygon.size() - 1;
    //绘制从最后一个顶点到第一个顶点的直线
    raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

//定义一个填充凸多边形内部单元格的函数
void Costmap2D::convexFillCells(const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells)
{
  //如果多边形的顶点少于3个，直接返回，因为无法形成一个封闭的区域
  if (polygon.size() < 3) 
  {
    return;
  }

  //首先生成多边形的轮廓单元格
  polygonOutlineCells(polygon, polygon_cells);

  //使用冒泡排序法对多边形单元格按x坐标进行排序
  MapLocation swap;
  unsigned int i = 0;
  //继续排序直到没有元素需要交换，或者达到数组的末尾
  while (i < polygon_cells.size() - 1) 
  {
    //如果当前单元格的x坐标大于下一个单元格的x坐标，说明顺序错误，需要交换这两个单元格
    if (polygon_cells[i].x > polygon_cells[i + 1].x) 
    {
      //保存当前单元格到临时变量swap
      swap = polygon_cells[i];
      //将下一个单元格的值赋给当前单元格
      polygon_cells[i] = polygon_cells[i + 1];
      //将保存的当前单元格的值赋给下一个单元格，完成交换
      polygon_cells[i + 1] = swap;

      //如果不是数组的第一个元素，则将索引i减1，回退一步，重新检查前一个元素，确保排序的完整性
      if (i > 0) 
      {
        --i;
      }
    } 
    else  //如果当前单元格的x坐标不大于下一个单元格的x坐标，说明这两个单元格已经正确排序，继续检查下一个单元格
    {
      ++i;
    }
  }
  
  //重置索引，准备填充操作
  i = 0;
  //用于记录每个x坐标下的最小y坐标的单元格
  MapLocation min_pt;
  //用于记录每个x坐标下的最大y坐标的单元格
  MapLocation max_pt;
  //x坐标的最小值
  unsigned int min_x = polygon_cells[0].x;
  //x坐标的最大值
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  //遍历从x的最小值到最大值
  for (unsigned int x = min_x; x <= max_x; ++x) 
  {
    //如果达到了多边形单元格列表的末尾，中断循环
    if (i >= polygon_cells.size() - 1) 
    {
      break;
    }

    //检查当前的两个相邻单元格来确定哪一个的y坐标更小或更大
    if (polygon_cells[i].y < polygon_cells[i + 1].y) 
    {
      //如果当前单元格的y坐标较小，则它成为最小点
      min_pt = polygon_cells[i];
      //下一个单元格成为最大点
      max_pt = polygon_cells[i + 1];
    } 
    else 
    {
      //如果下一个单元格的y坐标较小，则它成为最小点
      min_pt = polygon_cells[i + 1];
      //当前单元格成为最大点
      max_pt = polygon_cells[i];
    }
    //移动索引i，跳过已经比较过的两个单元格
    i += 2;
    //继续检查后续单元格，只要x坐标相同
    while (i < polygon_cells.size() && polygon_cells[i].x == x) 
    {
      //更新y坐标的最小点和最大点
      if (polygon_cells[i].y < min_pt.y) 
      {
        //发现更小的y坐标
        min_pt = polygon_cells[i];
      } 
      else if (polygon_cells[i].y > max_pt.y) 
      {
        //发现更大的y坐标
        max_pt = polygon_cells[i];
      }
      //移动到下一个单元格
      ++i;
    }

    //遍历从最小y到最大y的范围
    MapLocation pt;
    for (unsigned int y = min_pt.y; y <= max_pt.y; ++y) 
    {
      //设置填充点的x坐标
      pt.x = x;
      //设置填充点的y坐标
      pt.y = y;
      //将填充点添加到多边形单元格列表中
      polygon_cells.push_back(pt);
    }
  }
}

//返回代价地图的宽度，单位为单元格
unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}
//返回代价地图的高度，单位为单元格
unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}
//返回代价地图的宽度，单位为米。这种计算方式考虑了单元格中心到边缘的距离
double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}
//返回代价地图的高度，单位为米。同样，计算方式考虑了单元格中心到边缘的距离
double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}
//返回代价地图的x轴原点位置，单位为米
double Costmap2D::getOriginX() const
{
  return origin_x_;
}
//返回代价地图的y轴原点位置，单位为米
double Costmap2D::getOriginY() const
{
  return origin_y_;
}
//返回代价地图单元格的分辨率，即每个单元格的边长，单位为米
double Costmap2D::getResolution() const
{
  return resolution_;
}

//这个函数用于将代价地图保存为PGM(Portable GrayMap)格式的文件
bool Costmap2D::saveMap(std::string file_name)
{
  //打开一个文件，用于写入，文件名由调用者指定
  FILE * fp = fopen(file_name.c_str(), "w");
  //如果文件没有成功打开，返回 false
  if (!fp) 
  {
    return false;
  }

  //写入PGM文件的头部，指定它是P2格式，随后是地图的宽度和高度以及最大灰度值(255)
  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  //遍历每一行
  for (unsigned int iy = 0; iy < size_y_; iy++) 
  {
    //遍历每一列
    for (unsigned int ix = 0; ix < size_x_; ix++) 
    {
      //获取当前单元格的代价值
      unsigned char cost = getCost(ix, iy);
      //将代价值写入文件
      fprintf(fp, "%d ", cost);
    }
    //每完成一行的数据后，写入一个换行符
    fprintf(fp, "\n");
  }
  //关闭文件
  fclose(fp);
  //文件成功写入后返回true
  return true;
}

} 
