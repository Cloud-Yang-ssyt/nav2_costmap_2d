/*
  这个文件用于接收体素栅格数据，并将这些数据转换为可视化的标记，最后发布这些标记以供rviz2工具显示。
  这使得用户可以直观地查看和分析体素栅格数据，体素栅格通常用于机器人的三维空间感知和障碍物检测等导航相关的任务。
*/

#include <string>
#include <vector>
#include <memory>
#include <utility>

//这里的头文件，我在其他文件里都说明了，这里不做过多解释
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_util/execution_timer.hpp"

/*
  这个文件里有大部分都是costmap_2d_cloud.cpp文件里重复的，我就不详细写了
*/
//定义Cell结构体，用于存储体素的坐标和状态
struct Cell
{
  //体素的世界坐标
  double x;
  double y;
  double z;
  nav2_voxel_grid::VoxelStatus status;  //体素的状态
};

//定义一个包含多个Cell的向量类型
typedef std::vector<Cell> V_Cell;

float g_colors_r[] = {0.0f, 0.0f, 1.0f};  //红色
float g_colors_g[] = {0.0f, 0.0f, 0.0f};  //绿色
float g_colors_b[] = {0.0f, 1.0f, 0.0f};  //蓝色
float g_colors_a[] = {0.0f, 0.5f, 1.0f};  //透明度

//存储需要可视化的体素数据
V_Cell g_cells;
rclcpp::Node::SharedPtr g_node;

//全局标记发布者共享指针
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;

//回调函数
void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  //检查接收到的体素数据是否为空，如果为空则记录错误并返回
  if (grid->data.empty()) 
  {
    //输出错误日志
    RCLCPP_ERROR(g_node->get_logger(), "Received voxel grid");
    return;
  }

  //创建计时器对象，用于跟踪处理体素数据的时间
  nav2_util::ExecutionTimer timer;
  //开始计时
  timer.start();

  //记录调试信息，表示已经接收到体素栅格数据
  RCLCPP_DEBUG(g_node->get_logger(), "Received voxel grid");

  //从消息中提取体素栅格的各种属性
  const std::string frame_id = grid->header.frame_id;   //消息帧ID
  const rclcpp::Time stamp = grid->header.stamp;  //消息时间戳
  const uint32_t * data = &grid->data.front();  //指向体素数据的指针
  const double x_origin = grid->origin.x;   //栅格原点的x坐标
  const double y_origin = grid->origin.y;   //栅格原点的y坐标
  const double z_origin = grid->origin.z;   //栅格原点的z坐标
  const double x_res = grid->resolutions.x;   //栅格分辨率x
  const double y_res = grid->resolutions.y;   //栅格分辨率y
  const double z_res = grid->resolutions.z;   //栅格分辨率z
  const uint32_t x_size = grid->size_x;   //栅格在x维度的大小
  const uint32_t y_size = grid->size_y;   //栅格在y维度的大小
  const uint32_t z_size = grid->size_z;   //栅格在z维度的大小

  //空存储体素信息的容器
  g_cells.clear();
  //初始化标记的数量为0
  uint32_t num_markers = 0;

  //从y轴开始遍历体素栅格的每一个体素(这里为什么是从y轴开始遍历也在costmap_2d_cloud.cpp文件里写明了，这里不多做解释)
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) 
  {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) 
    {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) 
      {
        //获取当前体素的状态
        nav2_voxel_grid::VoxelStatus status = nav2_voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid,z_grid, x_size, y_size, z_size, data);
        //如果体素被标记为MARKED
        if (status == nav2_voxel_grid::MARKED) 
        {
          Cell c;  //创建一个新的Cell结构
          c.status = status;  //设置状态
          c.x = x_origin + (x_grid + 0.5) * x_res;  //计算体素中心的世界坐标x
          c.y = y_origin + (y_grid + 0.5) * y_res;  //计算体素中心的世界坐标y
          c.z = z_origin + (z_grid + 0.5) * z_res;  //计算体素中心的世界坐标z
          g_cells.push_back(c);  //将体素信息加入到列表中

          //增加标记的数量
          ++num_markers;
        }
      }
    }
  }

  //创建一个Marker消息，用于可视化
  auto m = std::make_unique<visualization_msgs::msg::Marker>();
  //设置帧ID
  m->header.frame_id = frame_id;
  //设置时间戳
  m->header.stamp = stamp;
  //设置命名空间
  m->ns = g_node->get_namespace();
  //设置标记的ID，这个位置的ID后续调参可能会使用
  m->id = 0;
  //设置标记类型为立方体列表
  m->type = visualization_msgs::msg::Marker::CUBE_LIST;
  //设置操作为添加
  m->action = visualization_msgs::msg::Marker::ADD;
  //设置方向的四元数
  m->pose.orientation.w = 1.0;
  //设置标记的尺寸x
  m->scale.x = x_res;
  //设置标记的尺寸y
  m->scale.y = y_res;
  //z
  m->scale.z = z_res;
  //设置颜色红色分量
  m->color.r = g_colors_r[nav2_voxel_grid::MARKED];
  //设置颜色绿色分量
  m->color.g = g_colors_g[nav2_voxel_grid::MARKED];
  //设置颜色蓝色分量
  m->color.b = g_colors_b[nav2_voxel_grid::MARKED];
  //设置颜色透明度
  m->color.a = g_colors_a[nav2_voxel_grid::MARKED];
  //调整points容器的大小以存储所有标记的位置
  m->points.resize(num_markers);

  for (uint32_t i = 0; i < num_markers; ++i) 
  {
    //获取每一个体素的信息
    Cell & c = g_cells[i];
    //设置点的位置
    geometry_msgs::msg::Point & p = m->points[i];
    p.x = c.x;
    p.y = c.y;
    p.z = c.z;
  }
  //发布Marker消息
  pub->publish(std::move(m));
  //结束计时并记录信息
  timer.end();
  RCLCPP_INFO(g_node->get_logger(), "Published %d markers in %f seconds", num_markers, timer.elapsed_time_in_seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("costmap_2d_marker");
  RCLCPP_DEBUG(g_node->get_logger(), "Starting costmap_2d_marker");

  //在节点上创建一个发布者，用于发布visualization_msgs::msg::Marker类型的消息到"visualization_marker"话题
  //第二个参数1表示发布队列的大小，用于控制消息队列的缓冲大小，这个队列长度的详细介绍也在costmap_2d_cloud.cpp里说明了，这里不介绍了
  pub = g_node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

  //在节点上创建一个订阅者，用于订阅nav2_msgs::msg::VoxelGrid类型的消息从"voxel_grid"话题
  //使用系统默认的服务质量设置（SystemDefaultsQoS）
  //voxelCallback是当消息到达时将被调用的回调函数
  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>("voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);

  rclcpp::spin(g_node->get_node_base_interface());
}
