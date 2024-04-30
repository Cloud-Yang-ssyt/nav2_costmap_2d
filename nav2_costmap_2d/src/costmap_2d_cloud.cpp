//这段代码是处理体素网格用的，主要是将体素网格转换为两种点云数据：被标记的点（marked points）和未知的点（unknown points）。

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
/*
下面这个头文件在nav2_voxel_grid/include下。主要是用来处理三维体素网格的文件，体素网格是一个三维网格系统，其中每个体素(体积元素)可以存储关于该空间点的信息，例如它是否被占据、空闲或未知。
文件内部我会做具体注释。
*/
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
/*
下面这个头文件在nav2_util/include下，是用来测量执行时长的文件，在处理体素网格点云数据时被用来测量处理接收到的体素网格并发布为点云数据所需的时间。
这有助于监控数据处理流程的效率，确保系统在预定的时间限制内完成任务。
文件内部我会具体注释。
*/
#include "nav2_util/execution_timer.hpp"

//这个函数将体素网格坐标下的(mx,my,mz)转换成世界坐标下的(wx,wy,wz)
static inline void mapToWorld3D
  (
  const unsigned int mx,  //体素在网格中的x坐标
  const unsigned int my,  //体素在网格中的y坐标
  const unsigned int mz,  //体素在网格中的z坐标
  const double origin_x,  //体素网格原点在世界坐标系统中的x坐标。
  const double origin_y,  //y
  const double origin_z,  //z
  const double x_resolution,  //每个点云体素在x方向的实际物理长度，也就是每个体素的分辨率
  const double y_resolution,  //同上，y方向
  const double z_resolution,  //z方向
  double & wx, double & wy, double & wz  //世界坐标的x，y，z值，用于输出计算结果。
  )
{
  /*
  下面用来计算点云体素的中心点，之所以要确定中心点，是因为每个体素的定义通常是从其边界开始，而中心点提供了一个准确的定位。具体方法是：
  世界坐标系下的坐标 + ((体素网格内坐标+0.5) * 分辨率半径)，这里的0.5是必须的，因为要引导向体素网格的中心
  */
  wx = origin_x + (mx + 0.5) * x_resolution;
  wy = origin_y + (my + 0.5) * y_resolution;
  wz = origin_z + (mz + 0.5) * z_resolution;
}

//定义了一个体素网格单元格的结构体
struct Cell
{
  //下面这(x,y,z)就是上面计算出的体素网格中心位置的坐标
  double x;
  double y;
  double z;
  //这个变量是voxel_grid.hpp文件下定义的枚举类型，用来表示体素的状态，包含：占用(OCCUPIED)、空闲(FREE)、未知(UNKNOWN)等状态
  nav2_voxel_grid::VoxelStatus status;
};

//用来存放多个Cell的容器
typedef std::vector<Cell> V_Cell;

//这四个定义了体素状态的颜色数组和透明度数组
float g_colors_r[] = {0.0f, 0.0f, 1.0f};  //红色通道
float g_colors_g[] = {0.0f, 0.0f, 0.0f};  //绿色通道
float g_colors_b[] = {0.0f, 1.0f, 0.0f};  //蓝色通道
float g_colors_a[] = {0.0f, 0.5f, 1.0f};  //透明度通道，0.0f, 0.5f, 1.0f分别表示完全透明，半透明，和不透明

//这两个值用来存储标记的体素和未知状态的体素，用上面的V_Cell存储为全局向量
V_Cell g_marked;
V_Cell g_unknown;

//ros2的全局共享指针
rclcpp::Node::SharedPtr g_node;

//这两个发布者发布的是ros2自带的消息格式，具体为传感器消息下的点云数据(这个是ros2源码包，这里不做详细介绍)。
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_marked;  //这个用于发布标记的点云
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unknown;  //这个用于发布未知状态的点云

//下面这个函数用来辅助填充pointcloud2的标记和未知点，主要是接收一个PointCloud2的unique_ptr引用，总点数，消息头，和点集合
void pointCloud2Helper
(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & cloud,  //需要填充的PointCloud2指针
  uint32_t num_channels,  //表示将被填充的总点数
  std_msgs::msg::Header header,  //携带需要分配给PointCloud2报头的报头信息
  V_Cell & g_cells  //包含需要添加到PointCloud2的x、y、z值
)
{
  cloud->header = header;  //将传入的header赋给点云的header，包含时间戳和坐标帧信息
  cloud->width = num_channels;  //设置点云的宽度为总的点数，点云宽度就是每一行包含点云的数量
  /*
  height这个参数表示点云的高度，但是这个值一般分为“等于1”或者“大于1”。
  当 height 大于 1 时，点云被认为是“有组织的”(organized)。意思就是将点云数据的结构看作图像，其中每个点都有明确的位置，并且相邻点在空间中也是相邻的。
  当 height 等于 1 时，点云被认为是“无组织的”(unorganized)。这种类型的点云没有固定的形状，其数据更像是一个长列表，每个点只按顺序排列。
  */
  cloud->height = 1;  //设置点云的高度为1，表示这是一个无组织的点云
  cloud->is_dense = true;  //设置点云为密集型，即没有无效点
  cloud->is_bigendian = false;  //设置数据在内存中的存储为小端模式，主要适用于大多数现代计算机

  //sensor_msgs::PointCloud2Modifier，这个信息是一个点云修改器，下面这部就是将初始化后的点云指针传入修改器，准备进行点云数据写入
  sensor_msgs::PointCloud2Modifier modifier(*cloud);

  //这个函数用来设置点云数据的结构和存储格式
  modifier.setPointCloud2Fields
    (
    6,  //设置数据数量
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,  //定义x坐标数据类型为float32
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,  //定义y坐标数据类型为float32
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,  //定义z坐标数据类型为float32
    "r", 1, sensor_msgs::msg::PointField::UINT8,  //定义红色分量数据类型为uint8
    "g", 1, sensor_msgs::msg::PointField::UINT8,  //定义绿色分量数据类型为uint8
    "b", 1, sensor_msgs::msg::PointField::UINT8  //定义蓝色分量数据类型为uint8
    );

  //sensor_msgs::PointCloud2Iterator是一个迭代器，第一个参数是需要处理的点云数据，第二个参数是具体的点云参数
  //初始化点云迭代器，用于按顺序写入点云数据的各个字段
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");

  //遍历所有要添加到点云中的单元格
  for (uint32_t i = 0; i < num_channels; ++i) 
  {
    //获取每个单元格的引用
    Cell & c = g_cells[i];
    //将单元格中的坐标和颜色信息分别赋值给点云数据
    *iter_x = c.x;  //设置x坐标
    *iter_y = c.y;  //设置y坐标
    *iter_z = c.z;  //设置z坐标
    *iter_r = g_colors_r[c.status] * 255.0;  //设置红色分量，状态对应的颜色乘以255才是具体的颜色
    *iter_g = g_colors_g[c.status] * 255.0;  //设置绿色分量
    *iter_b = g_colors_b[c.status] * 255.0;//设置绿色分量

    //递增所有迭代器，移动到下一个点的数据位置
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
}

//回调函数，用于处理接收到的体素网格消息
//这里传入的参数是ros2消息类型nav2_msgs::msg下的体素网格指针，具体内容需要看nav2_msgs源码，这里不做过多详细解释
void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  //检查接收到的网格数据是否为空
  if (grid->data.empty()) 
  {
    //如果数据为空，记录错误信息
    RCLCPP_ERROR(g_node->get_logger(), "Received empty voxel grid");
    return;  //直接返回，不做进一步处理
  }

  //这里初始化的是一个执行计时器，用于测量处理体素网格的时间
  nav2_util::ExecutionTimer timer;
  //开始计时
  timer.start();

  //记录接收到体素网格的调试信息
  RCLCPP_DEBUG(g_node->get_logger(), "Received voxel grid");
  //读取传入的消息的frame_id，即数据的参考坐标系
  const std::string frame_id = grid->header.frame_id;
  //读取消息的时间戳
  const rclcpp::Time stamp = grid->header.stamp;
  //获取传入的体素数据指针
  const uint32_t * data = &grid->data.front();

  //这里开始，下面读取体素网格的相对于世界坐标系下的坐标和体素的半径(就是分辨率)，也就是物理方向的长度
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;

  //读取传入的体素网格的尺寸
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;

  //初始化标记点的集合和未知状态点的集合
  g_marked.clear();
  g_unknown.clear();

  //定义两个变量，一个用来存放标记点云的数量，一个存放未知点云的数量
  uint32_t num_marked = 0;
  uint32_t num_unknown = 0;

  //开始遍历整个体素网格，处理每个体素
  /*
    这里补充一个小问题，下面的遍历是从y轴开始的，我读了代码之后发现没有明确的逻辑限制，必须要从y开始遍历，所以这里我认为应该不是一个强制的规则。
    但是由于我并不清楚nav2_msgs::msg::VoxelGrid下的这个ConstSharedPtr是否存在明确的存储格式限制；或者是对于体素网格来说，y轴优先遍历可能涉及到一定的性能优化或者算法设计，所以我并不敢随意修改逻辑来进行测试。
    还有一种可能会涉及到这样的写法：在处理图像数据的时候，是优先对y轴(行)遍历，然后是x(列)和z(深)。有可能是处于开发者的习惯，沿用了这样的代码风格，如果有人有不一样的看法欢迎讨论，qq：1820522365
  */
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) 
  {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) 
    {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) 
      {
        //这里是在获取当前体素的状态信息，getVoxel()函数的详细参数和逻辑我会在voxel_grid()中做详细说明
        nav2_voxel_grid::VoxelStatus status = nav2_voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, x_size, y_size, z_size, data);

        //根据每个点云体素的具体状态进行相关处理
        if (status == nav2_voxel_grid::UNKNOWN) //如果状态是未知
        {
          //创建一个新的Cell对象status
          Cell c;
          //给这个对象设置新的状态
          c.status = status;
          //这里就是依照上面的mapToWorld3D函数，将网格坐标转换为世界坐标
          mapToWorld3D(x_grid, y_grid, z_grid, x_origin, y_origin, z_origin, x_res, y_res, z_res, c.x, c.y, c.z);
          //将新的点数据添加到g_unknown未知点云集合中
          g_unknown.push_back(c);
          //增加未知点计数
          ++num_unknown;
        } 
        else if (status == nav2_voxel_grid::MARKED) //如果状态是标记
        {
          //这里往下的三步跟未知状态的三步是一样的
          Cell c;
          c.status = status;
          mapToWorld3D(x_grid, y_grid, z_grid, x_origin, y_origin, z_origin, x_res, y_res, z_res, c.x, c.y, c.z);
          //将新的点数据添加到g_marked标记点云集合中
          g_marked.push_back(c);
          //增加标记点计数
          ++num_marked;
        }
      }
    }
  }

  //这里新建了点云的头部信息
  std_msgs::msg::Header pcl_header;
  //这里的frame_id和stamp是pcl库中的数据，这个库的具体位置应该在/usr/include/pcl-1.12/pcl/PCLHeader.h的文件里面，这两个数据是在使用pcl库的时候自动加载的/
  pcl_header.frame_id = frame_id;  //这个是点云世界坐标系的名字
  pcl_header.stamp = stamp;  //这个是接受到点云信息的时间戳
  
  //创建点云对象，并使用辅助函数填充标记点的点云
  //这里和下面使用了两个{}是因为c++中，使用{}可以定义一个作用域，保证变量名不会冲突
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_marked, pcl_header, g_marked);  //这个是用来填充点云的标记点
    pub_marked->publish(std::move(cloud));  //发布标记的点云
  }

  //创建另一个点云对象，并填充未知点的点云
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_unknown, pcl_header, g_unknown);  //这个是用来填充未知点
    pub_unknown->publish(std::move(cloud));  //发布未知状态点的点云
  }

  //结束计时
  timer.end();
  //发布处理点云所花的时间，具体输出的数据num_marked + num_unknown是所有点云的总数，timer.elapsed_time_in_seconds()这个方法调用是获取定时器启动到终止使经过的总时间，单位是秒
  RCLCPP_DEBUG(g_node->get_logger(), "Published %d points in %f seconds", num_marked + num_unknown, timer.elapsed_time_in_seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //创建节点，命名costmap_2d_cloud
  g_node = rclcpp::Node::make_shared("costmap_2d_cloud");
  //发布日志
  RCLCPP_DEBUG(g_node->get_logger(), "Starting up costmap_2d_cloud");

  //创建一个发布者，用于发布标记过的点云数据，话题名为 "voxel_marked_cloud"，第二个参数 1 表示发布队列的大小
  pub_marked = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_marked_cloud", 1);
  //创建另一个发布者，用于发布未知的点云数据，话题名为 "voxel_unknown_cloud"，发布队列大小同样为 1
  pub_unknown = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_unknown_cloud", 1);
  /*
    这里补充一下，我查阅了相关资料，这个发布队列的大小，也就是create_publisher()里的Qos在这里并没有严格要求，可以设置成10是没问题的。
    但是这里设计者写了1，应该是为了保证实时性和资源利用的效率，就是让队列里始终只有一个待处理数据，可以避免数据溢出或者阻塞。
  */

  //创建一个订阅者，订阅"voxel_grid"的话题，使用默认服务质量设置（SystemDefaultsQoS）
  //当接收到消息时，调用voxelCallback函数进行回调处理
  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>("voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);
  /*
    有两点说明：
    第一点：这里我仔细找了一下“voxel_grid”话题名是从哪来的，并不是写在"nav2_voxel_grid/voxel_grid.hpp"里的，而是在这个系统默认的ros2内置体素网格头文件下发布的"nav2_msgs/msg/voxel_grid.hpp"，这个文件我后续有时间再写详细的说明吧
    第二点：rclcpp::SystemDefaultsQoS()这个方法调用是个ros2中默认使用的Qos队列设置，可以平衡每个场景的具体使用，还可以设置具体的参数。但是我没找到他特别的地方，这里也没有必须要使用这个默认调用方法，使用如果后续有别的调参需求的话，我认为可以随意调整。
  */

  //get_node_base_interface()这个方法用于获取ros2节点对象中的基本接口
  rclcpp::spin(g_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
