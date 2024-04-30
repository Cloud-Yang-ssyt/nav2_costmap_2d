/*
  这个文件提供了对单个代价地图层的操作和管理功能，其中实现的CostmapLayer类充当了代价地图的基础层，提供通用的接口和方法，使得各种具体的地图层(如障碍物层、膨胀层等)能够在此基础上实现更专业化的功能
  主要包含了以下几个功能：
  边界管理：
    方法touch和addExtraBounds用于管理和更新地图层处理的边界区域。包括确定哪些区域的数据需要更新或是被清除，以及在更新期间保持数据的一致性和完整性。
  大小匹配：
    matchSize方法确保地图层的尺寸与主代价地图的尺寸匹配。
  清除区域：
    clearArea方法提供了清除地图层中特定区域数据的功能。主要用在删除过时或不再相关的障碍信息。
  更新策略：
    提供多种更新策略，允许地图层以不同的方式将其数据融合到主代价地图中。
  边界利用：
    useExtraBounds方法应用记录的额外边界到当前操作中，这有助于在动态环境中处理地图的局部更新。
*/

/*
  在这里需要补充一个知识点，什么是主代价地图(master costmap)？
    主代价地图是一个中心或基础的代价地图，它集成了如障碍物层、膨胀层等多个层的信息。通常表示为一张二维栅格地图，每个栅格(cell)包含一个代价值，这个值表示通过该栅格的难易程度。
    简单概括就是，所有的特征层都是独立存在的，障碍物、地形等地图的信息都被单独的存放在一张代价地图里，而主代价地图就是集成和控制所有代价地图的工具。
  那么为什么要使用主代价地图呢？为什么不能对每个代价地图单独进行操作？
    1.导航决策是一个复杂的问题，静态障碍物、动态障碍物、感知区域等信息都是用不同的消息类型来设定的，如果不统一，就无法构建一个综合的能够用于导航决策的视图
    2.主代价地图为路径规划算法(如A*或Dijkstra算法)提供了一个单一的、一致的输入源，简化了计算过程。路径规划算法可以直接在这个地图上运行，确定从起点到终点的最佳路径。
    3.主代价地图的层级结构使得动态更新成为可能。例如，当新的障碍物被检测到时，可以仅更新相关的层，而不必重新计算整个地图。这种方法提高了系统的响应速度和效率。
    4.使用主代价地图和多层架构，可以轻松添加、移除或修改特定的层，以适应不同的应用场景。这种模块化设计增加了系统的灵活性和可重用性。
*/
//这个头文件里有CostmapLayer类，这个类是所有特定代价地图的基类
#include <nav2_costmap_2d/costmap_layer.hpp>
//这个头文件里定义了一系列标准的异常类，这些类在程序运行时因为特定的错误抛出
#include <stdexcept>
#include <algorithm>

namespace nav2_costmap_2d
{
//这个方法用于更新给定点(x, y)在图层中的边界值。通常在添加或修改地图内容时调用，以确保边界区域包括所有修改
void CostmapLayer::touch(double x, double y, double * min_x, double * min_y, double * max_x, double * max_y)
{
  //如果x小于当前的最小x，更新最小x
  *min_x = std::min(x, *min_x);
  //如果y小于当前的最小y，更新最小y
  *min_y = std::min(y, *min_y);
  //如果x大于当前的最大x，更新最大x
  *max_x = std::max(x, *max_x);
  //如果y大于当前的最大y，更新最大y
  *max_y = std::max(y, *max_y);
}

//这个方法确保当前图层的尺寸、分辨率和原点位置与主代价地图相匹配
void CostmapLayer::matchSize()
{
  //获取主代价地图的引用
  Costmap2D * master = layered_costmap_->getCostmap();  
  //调整当前图层的尺寸和分辨率
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}

//清除指定区域内的代价信息
void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  //标记当前图层为未更新状态
  current_ = false;  
  //获取当前图层的代价地图数据指针
  unsigned char * grid = getCharMap();
  //遍历图层的所有x坐标
  /*
    这里又是从x坐标开始遍历，很那不怀疑写这个代价地图功能包的是好几个人，习惯不一样
  */
  for (int x = 0; x < static_cast<int>(getSizeInCellsX()); x++) 
  {
    //判断x是否在指定的x范围内
    bool xrange = x > start_x && x < end_x;
    //遍历图层的所有y坐标
    for (int y = 0; y < static_cast<int>(getSizeInCellsY()); y++) 
    {
      //根据invert参数判断当前单元格是否在清除区域内
      /*
        什么是invert参数？
          这个参数是costmap_layer.hpp头文件下的一个bool类型值，用于控制清除代价地图区域的行为逻辑。
          主要决定是要清除指定的矩形区域内的数据，还是清除区域外的数据。
      */
      if ((xrange && y > start_y && y < end_y) == invert) 
      {
        //如果在清除区域内且invert为false，或不在区域内且invert为true，跳过当前循环
        continue;
      }
      //获取当前坐标的索引
      int index = getIndex(x, y);
      //如果当前坐标的代价值不是无信息状态
      if (grid[index] != NO_INFORMATION) 
      {
        //设置为无信息
        grid[index] = NO_INFORMATION;
      }
    }
  }
}

//添加额外的边界信息到当前层
void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
  //更新额外边界的最小x值
  extra_min_x_ = std::min(mx0, extra_min_x_);
  //更新额外边界的最大x值
  extra_max_x_ = std::max(mx1, extra_max_x_);
  //更新额外边界的最小y值
  extra_min_y_ = std::min(my0, extra_min_y_);
  //更新额外边界的最大y值
  extra_max_y_ = std::max(my1, extra_max_y_);
  //标记为有额外边界
  has_extra_bounds_ = true;
}

//使用额外的边界信息更新当前层的边界
void CostmapLayer::useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y)
{
  //如果没有额外边界，则直接返回
  if (!has_extra_bounds_) 
  {
    return;
  }
  //使用额外边界更新最小x值
  *min_x = std::min(extra_min_x_, *min_x);
  //使用额外边界更新最小y值
  *min_y = std::min(extra_min_y_, *min_y);
  //使用额外边界更新最大x值
  *max_x = std::max(extra_max_x_, *max_x);
  //使用额外边界更新最大y值
  *max_y = std::max(extra_max_y_, *max_y);
  //重置额外边界的最小x值
  extra_min_x_ = 1e6;
  //重置额外边界的最小y值
  extra_min_y_ = 1e6;
  //重置额外边界的最大x值
  extra_max_x_ = -1e6;
  //重置额外边界的最大y值
  extra_max_y_ = -1e6;
  //标记为没有额外边界
  has_extra_bounds_ = false;
}

//更新主代价地图，使用当前层的数据，但只当当前层的值更大时
void CostmapLayer::updateWithMax(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //如果当前层未启用，直接返回
  if (!enabled_) 
  {
    return;
  }

  //获取主代价地图的数据
  unsigned char * master_array = master_grid.getCharMap();
  //获取主代价地图的宽度
  unsigned int span = master_grid.getSizeInCellsX();
  //遍历指定的行
  for (int j = min_j; j < max_j; j++) 
  {
    //计算行起始索引
    unsigned int it = j * span + min_i;
    //遍历指定的列
    for (int i = min_i; i < max_i; i++) 
    {
      //如果当前层的值为无信息，则跳过
      if (costmap_[it] == NO_INFORMATION) 
      {
        it++;
        continue;
      }

      //获取主代价地图当前位置的值
      unsigned char old_cost = master_array[it];
      //如果主代价地图的值为无信息或小于当前层的值
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) 
      {
        //更新主代价地图的值为当前层的值
        master_array[it] = costmap_[it];
      }
      //移动到下一个索引
      it++;
    }
  }
}

//使用完全覆写方式更新主代价地图
void CostmapLayer::updateWithTrueOverwrite(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //如果当前层未启用，直接返回，不执行更新
  if (!enabled_) 
  {
    return;
  }

  //检查当前层是否已初始化，未初始化则抛出异常
  if (costmap_ == nullptr) 
  {
    throw std::runtime_error("Can't update costmap layer: It has't been initialized yet!");
  }

  //获取主代价地图的数据指针
  unsigned char * master = master_grid.getCharMap();
  //获取主代价地图每行的单元格数量
  unsigned int span = master_grid.getSizeInCellsX();

  //遍历指定的行
  for (int j = min_j; j < max_j; j++) 
  {
    //计算当前行的起始索引
    unsigned int it = span * j + min_i;
    //遍历指定的列
    for (int i = min_i; i < max_i; i++) 
    {
      //直接使用当前层的值覆盖主代价地图的值
      master[it] = costmap_[it];
      //直接使用当前层的值覆盖主代价地图的值
      it++;
    }
  }
}

//使用条件覆写方式更新主代价地图
void CostmapLayer::updateWithOverwrite(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //如果当前层未启用，直接返回
  if (!enabled_) 
  {
    return;
  }

  //获取主代价地图的数据指针
  unsigned char * master = master_grid.getCharMap();
  //获取主代价地图每行的单元格数量
  unsigned int span = master_grid.getSizeInCellsX();
  //遍历指定的行
  for (int j = min_j; j < max_j; j++) 
  {
    //计算当前行的起始索引
    unsigned int it = span * j + min_i;
    //遍历指定的列
    for (int i = min_i; i < max_i; i++) 
    {
      //如果当前层的单元格含有有效信息
      if (costmap_[it] != NO_INFORMATION) 
      {
        //使用当前层的值覆盖主代价地图的值
        master[it] = costmap_[it];
      }
      //移至下一个单元格
      it++;
    }
  }
}

//使用加法方式更新主代价地图
void CostmapLayer::updateWithAddition(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //如果当前层未启用，直接返回
  if (!enabled_) 
  {
    return;
  }
  //获取主代价地图的数据指针
  unsigned char * master_array = master_grid.getCharMap();
  //获取主代价地图每行的单元格数量
  unsigned int span = master_grid.getSizeInCellsX();

  //遍历指定的行
  for (int j = min_j; j < max_j; j++) 
  {
    //计算当前行的起始索引
    unsigned int it = j * span + min_i;
    //遍历指定的列
    for (int i = min_i; i < max_i; i++) 
    {
      //如果当前层的单元格无有效信息，则跳过
      if (costmap_[it] == NO_INFORMATION) 
      {
        it++;
        continue;
      }
      //获取主代价地图当前单元格的值
      unsigned char old_cost = master_array[it];
      //如果主代价地图当前单元格无有效信息
      if (old_cost == NO_INFORMATION) 
      {
        //直接使用当前层的值
        master_array[it] = costmap_[it];
      } 
      else //如果两层都有有效信息
      {
        //将两层的值相加
        int sum = old_cost + costmap_[it];
        //如果和值超过了预设的最大障碍值
        if (sum >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) 
        {
          //设置为最大障碍值
          master_array[it] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        } 
        else 
        {
          //否则，使用和值
          master_array[it] = sum;
        }
      }
      //移至下一个单元格
      it++;
    }
  }
}
}  // namespace nav2_costmap_2d
