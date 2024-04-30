/*
  这个文件主要是用来管理和发布Costmap2D数据到ros2环境中，使其能被其他节点和系统用于导航和路径规划。文件里为代价地图的数据可视化、更新和服务请求提供了必要的接口和实现。
  具体分为以下几个功能部分：
  1.发布完整的代价地图：
    Costmap2DPublisher类能够将完整的Costmap2D数据发布为标准的消息类型nav_msgs::msg::OccupancyGrid。这可以让任何标准的ros工具或节点都可以接收和处理这些数据。
  2.发布原始代价地图和更新：
    除了发布完整的网格外，该文件还能发布原始的Costmap2D数据(nav2_msgs::msg::Costmap)，以及只包含变动部分的更新数据(map_msgs::msg::OccupancyGridUpdate)。
    这样的分层发布策略有助于优化网络带宽和处理效率，特别是在大规模地图或频繁更新的环境中。
  3.响应代价地图服务请求：
    类中还包括了一个服务端点，用于处理外部请求当前代价地图的服务请求(nav2_msgs::srv::GetCostmap)。这是一个服务端应用，允许其他节点按需查询当前的代价地图状态。
  4.代价值的转换：
    为了与ROS的nav_msgs::msg::OccupancyGrid格式兼容，该文件使用一个转换表(cost_translation_table_)将Costmap2D的代价值映射到标准的占用网格值。
    这个转换处理是必要的，因为ros规定的占用网格值范围和语义可能与内部代价地图使用的值不同。
  5.灵活的发布控制：
    发布操作可以根据配置(如 always_send_full_costmap_ 标志)灵活进行。这包括在必要时总是发送完整地图，或者仅在地图发生变化时发送更新。
    这种灵活性允许Costmap2DPublisher根据实际需求和系统资源有效地管理其输出。
*/
//发布代价地图头文件
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

#include <string>
#include <memory>
#include <utility>

/*
  这个头文件用于管理和表示代价地图中不同代价值，这些代价值用来表示地图上不同区域对机器人运动的通行性的评估，有一些主要的代价值：
    FREE_SPACE：表示该区域没有障碍物，机器人可以自由通过。
    LETHAL_OBSTACLE：表示该区域有致命的障碍物，机器人绝对不能通过。
    INSCRIBED_INFLATED_OBSTACLE：表示该区域接近致命障碍物的范围，在某些配置下，可能会被视为不可通行。
    NO_INFORMATION：表示该区域的信息未知，通常在代价地图未被完全探索的区域使用。
*/
#include "nav2_costmap_2d/cost_values.hpp"

/*
  在开始看这里的代码之前，需要补充一个点：代价地图的使用为什么需要互斥锁(mutex)？
    一句话总结：因为代价地图可能会被多个线程或进程同时访问和修改，所以需要互斥锁来确保数据的一致性和完整性。
  详细解释：
    防止数据竞争：
    当多个线程或任务尝试同时修改代价地图的数据时，如果没有适当的同步机制，可能会导致数据竞争。数据竞争会导致数据不一致，从而影响导航系统的决策和行为的正确性。
    保持数据一致性：
    代价地图在动态环境中频繁更新，如传感器数据输入和代价重新计算。使用互斥锁可以保证在读取或修改地图数据时，数据是最新和一致的，避免在数据更新过程中被读取，导致部分更新的情况。
    同步更新和访问操作：
    互斥锁确保在任一时刻只有一个线程可以执行对代价地图的写操作，或者当一个线程正在写数据时，防止其他线程读取正在写入的数据。这样可以同步不同线程对共享资源的访问和修改，保障操作的原子性。
    防止死锁和资源竞争：
    因为代价地图的操作是多线程，互斥锁策略可以帮助避免死锁和资源竞争的问题。通过互斥锁管理对代价地图的访问，可以有效地规划线程行为，确保系统稳定运行。
*/

namespace nav2_costmap_2d
{

//成员变量定义，用于存储代价值到占用格网的转换表
char * Costmap2DPublisher::cost_translation_table_ = NULL;

//构造函数，第一个参数是父节点的弱指针，第二个参数是代价地图对象指针，第三个参数是代价地图使用的全局坐标系，第四个参数是基础话题名称，第五个参数是用来判断是否总是发送完整代价地图的标志
Costmap2DPublisher::Costmap2DPublisher(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2D * costmap,
  std::string global_frame,
  std::string topic_name,
  bool always_send_full_costmap): costmap_(costmap), global_frame_(global_frame), topic_name_(topic_name), active_(false), always_send_full_costmap_(always_send_full_costmap)
{
  auto node = parent.lock();  //弱指针强引用
  clock_ = node->get_clock();  //获取节点的时钟对象
  logger_ = node->get_logger();  //获取节点的日志对象

  //定制的服务质量设置，这句话的详细解释在costmap_subscriber.cpp中有，这里不赘述
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  //创建用于发布完整代价地图的发布者
  costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, custom_qos);
  //创建用于发布原始代价地图数据的发布者
  costmap_raw_pub_ = node->create_publisher<nav2_msgs::msg::Costmap>(topic_name + "_raw", custom_qos);
  //创建用于发布代价地图更新的发布者
  costmap_update_pub_ = node->create_publisher<map_msgs::msg::OccupancyGridUpdate>(topic_name + "_updates", custom_qos);

  //创建一个服务，用于响应获取代价地图的请求
  costmap_service_ = node->create_service<nav2_msgs::srv::GetCostmap>("get_costmap", std::bind(&Costmap2DPublisher::costmap_service_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  //初始化代价值转换表
  if (cost_translation_table_ == NULL) 
  {
    cost_translation_table_ = new char[256];  //配转换表的内存
    
    //设置特殊代价值
    cost_translation_table_[0] = 0;   //无障碍
    cost_translation_table_[253] = 99;   //接近障碍(这个接近障碍和致命障碍设置的值太接近了，我觉得后续调参这是一个重点)
    cost_translation_table_[254] = 100;   //致命障碍
    cost_translation_table_[255] = -1;  //未知区域

    //填充其他代价值
    for (int i = 1; i < 253; i++) 
    {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);  //代价值映射到0-100之间
    }
  }

  //初始化代价地图更新区域的起始和终止坐标
  xn_ = yn_ = 0;  //初始化更新区域的宽度和高度为0
  x0_ = costmap_->getSizeInCellsX();  //初始化更新区域的起始X坐标
  y0_ = costmap_->getSizeInCellsY();  //初始化更新区域的起始Y坐标
}

//析构函数
Costmap2DPublisher::~Costmap2DPublisher() {}

/*
//这段源码里被注释掉的代码是一个成员函数，原本设计用于处理当有新的订阅者订阅Costmap2DPublisher发布的话题时的事件。在ros1中用于在有新的订阅者时动态地准备和发布数据，这种方法确保新的订阅者能立即接收到最新的数据，而不需要等待下一个发布周期
//为什么被注释掉了？因为ros2的订阅和发布机制与ros1不同。在ros2中，类似的功能可能需要使用不同的方法来实现，如监听订阅者的增加和使用不同的回调机制来处理。在ros2中，通常不需要直接处理单个订阅者的情况，因为ros2的发布和订阅机制提供了更高级的Qos设置，可以配置如何响应新的或暂时的订阅者。
void Costmap2DPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
{
  prepareGrid();  //准备网格数据
  pub.publish(grid_);  //发布准备好的网格数据给新的订阅者
} 
*/

//定义prepareGrid方法，用于准备发布的grid_消息
void Costmap2DPublisher::prepareGrid()
{
  //获取代价地图的互斥锁，确保线程安全
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  //获取并设置网格的分辨率
  grid_resolution = costmap_->getResolution();
  //获取并设置网格的宽度(单位为单元格数)
  grid_width = costmap_->getSizeInCellsX();
  //获取并设置网格的高度(单位为单元格数)
  grid_height = costmap_->getSizeInCellsY();

  //创建一个新的OccupancyGrid消息
  grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  //设置网格消息的frame ID
  grid_->header.frame_id = global_frame_;
  //设置网格消息的时间戳
  grid_->header.stamp = clock_->now();
  //设置网格信息中的分辨率
  grid_->info.resolution = grid_resolution;
  //设置网格信息中的宽度
  grid_->info.width = grid_width;
  //设置网格信息中的高度
  grid_->info.height = grid_height;
  //将代价地图的原点坐标转换为世界坐标系中的点
  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  //设置网格原点坐标，并对原点进行微调，以匹配网格的中心
  grid_->info.origin.position.x = wx - grid_resolution / 2;
  grid_->info.origin.position.y = wy - grid_resolution / 2;
  //Z坐标通常为0，因为代价地图是2D的
  grid_->info.origin.position.z = 0.0;  
  //设置网格原点的方向（四元数），这里表示无旋转
  grid_->info.origin.orientation.w = 1.0;
  //记录原点的X和Y坐标，用于后续可能的更新检查
  saved_origin_x_ = costmap_->getOriginX();
  saved_origin_y_ = costmap_->getOriginY();
  //调整网格数据的大小，以匹配网格的宽度和高度
  grid_->data.resize(grid_->info.width * grid_->info.height);

  //获取代价地图的数据指针
  unsigned char * data = costmap_->getCharMap();
  //将代价地图的数据转换为占用网格数据
  for (unsigned int i = 0; i < grid_->data.size(); i++) 
  {
    grid_->data[i] = cost_translation_table_[data[i]];
  }
}

//用来准备一个原始代价地图的nav2_msgs::msg::Costmap消息，并填充其内容以便于发布。
void Costmap2DPublisher::prepareCostmap()
{
  //锁定代价地图的互斥锁，确保在访问和修改代价地图数据时线程安全
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  //获取当前代价地图的分辨率
  double resolution = costmap_->getResolution();

  //创建一个新的Costmap消息
  costmap_raw_ = std::make_unique<nav2_msgs::msg::Costmap>();

  //设置消息头部的坐标系为世界坐标系
  costmap_raw_->header.frame_id = global_frame_;
  //设置消息头部的时间戳
  costmap_raw_->header.stamp = clock_->now();

  //设置元数据的层名称为 "master"
  costmap_raw_->metadata.layer = "master";
  //设置元数据的分辨率
  costmap_raw_->metadata.resolution = resolution;
  //设置元数据的宽度，即代价地图X轴的单元格数目
  costmap_raw_->metadata.size_x = costmap_->getSizeInCellsX();
  //设置元数据的高度，即代价地图Y轴的单元格数目
  costmap_raw_->metadata.size_y = costmap_->getSizeInCellsY();
  
  //将代价地图的网格原点从网格坐标转换为世界坐标
  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  //设置原点的世界坐标，微调使得原点处于网格单元的中心
  costmap_raw_->metadata.origin.position.x = wx - resolution / 2;
  costmap_raw_->metadata.origin.position.y = wy - resolution / 2;
  //Z坐标设置为0
  costmap_raw_->metadata.origin.position.z = 0.0;
  //设置原点的朝向，w=1.0意味着无旋转
  costmap_raw_->metadata.origin.orientation.w = 1.0;

  //分配足够的空间来存储代价地图的数据
  costmap_raw_->data.resize(costmap_raw_->metadata.size_x * costmap_raw_->metadata.size_y);

  //获取代价地图的原始数据指针
  unsigned char * data = costmap_->getCharMap();
  //将原始代价值直接拷贝到消息的数据区域
  for (unsigned int i = 0; i < costmap_raw_->data.size(); i++) 
  {
    costmap_raw_->data[i] = data[i];
  }
}

//这个方法用于管理和发布三种类型的代价地图消息：完整的代价地图、原始代价地图、以及代价地图的更新。这是基于订阅者的需求以及地图数据的变化来决定发布哪种类型的消息。
void Costmap2DPublisher::publishCostmap()
{
  //检查是否有订阅者订阅原始代价地图的话题
  if (costmap_raw_pub_->get_subscription_count() > 0) 
  {
    //准备原始代价地图，就是上一个方法的实现
    prepareCostmap();
    //发布原始代价地图
    costmap_raw_pub_->publish(std::move(costmap_raw_));
  }
  //获取当前代价地图的分辨率
  float resolution = costmap_->getResolution();

  //检查是否总是发送完整的代价地图或者代价地图的相关参数发生了变化
  if (always_send_full_costmap_ || grid_resolution != resolution ||
    grid_width != costmap_->getSizeInCellsX() ||
    grid_height != costmap_->getSizeInCellsY() ||
    saved_origin_x_ != costmap_->getOriginX() ||
    saved_origin_y_ != costmap_->getOriginY())
  {
    //检查是否有订阅者订阅完整代价地图的话题
    if (costmap_pub_->get_subscription_count() > 0) 
    {
      //准备完整的代价地图
      prepareGrid();
      //发布完整的代价地图
      costmap_pub_->publish(std::move(grid_));
    }
  } 
  //检查是否有局部更新需要发送
  else if (x0_ < xn_) 
  {
    //检查是否有订阅者订阅代价地图更新的话题
    if (costmap_update_pub_->get_subscription_count() > 0) 
    {
      //锁定代价地图的互斥锁
      std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
      //创建更新消息
      auto update = std::make_unique<map_msgs::msg::OccupancyGridUpdate>();
      //设置消息时间戳
      update->header.stamp = rclcpp::Time();
      //设置消息的参考坐标系为全局坐标系
      update->header.frame_id = global_frame_;
      //设置更新区域的起始X坐标
      update->x = x0_;
      //设置更新区域的起始Y坐标
      update->y = y0_;
      //设置更新区域的宽度
      update->width = xn_ - x0_;
      //设置更新区域的高度
      update->height = yn_ - y0_;
      //调整数据大小以匹配更新区域
      update->data.resize(update->width * update->height);
      //初始化索引变量i，用于跟踪更新数据数组的位置
      unsigned int i = 0;
      //遍历代价地图的指定区域，由y0_到yn_定义纵向范围
      for (unsigned int y = y0_; y < yn_; y++) 
      {
        //遍历代价地图的指定区域，由x0_到xn_定义横向范围
        for (unsigned int x = x0_; x < xn_; x++) 
        {
          //调用Costmap2D的getCost方法获取指定单元格(x, y)的代价值
          unsigned char cost = costmap_->getCost(x, y);
          //使用预先定义的转换表cost_translation_table_将原始代价值转换为标准的占用网格值
          update->data[i++] = cost_translation_table_[cost];
        }
      }
      //调用发布者对象costmap_update_pub_，发布准备好的OccupancyGridUpdate消息
      costmap_update_pub_->publish(std::move(update));

      /*
        上面那段从y开始的区域索引，这里需要解释一下(我发现nav2的开发人员特别喜欢从y开始遍历循环)：
          从y开始的遍历方式通常称为“行主序”(Row-major order)。这段代码遵循这种传统方法的原因有以下几点：
          内存访问模式：
          在很多编程环境中，包括C++和用于存储代价地图的数据结构，数据通常在内存中以行主序存储。这意味着同一行的数据在内存中是连续的。首先遍历y坐标(行)，然后在每一行内遍历x坐标(列)，可以更有效地利用内存的局部性原理，提高缓存效率。
          图像和地图处理的惯例：
          在图像处理和地图数据处理中，通常习惯于从顶部到底部处理每一行，即先固定行(y坐标)，然后遍历列(x坐标)。这种方式直观且符合大多数地图和图像数据的组织方式。
          数据结构的表示：
          如果代价地图在内部数据结构中是以二维数组形式存储，其中数组的每一行对应地图的一行，则首先遍历y坐标可以直接映射到数组的访问模式上。
          与图形库和标准接口兼容：
          很多图形处理库和地理信息系统(GIS)工具也采用这种行主序遍历方式，保持这种遍历顺序有助于与这些工具和库的接口兼容，方便数据的交换和处理。
      */
    }
  }

  //重置更新区域的坐标，为下一次更新做准备
  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

//下面这个方法是一个服务回调函数，用于响应来自其他节点的请求，提供当前代价地图的数据。
//定义服务回调函数
void Costmap2DPublisher::costmap_service_callback(
  const std::shared_ptr<rmw_request_id_t>,  //服务请求的头部信息(未使用)
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>,  //服务请求的具体请求数据(未使用)
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)  //服务响应对象，将在此函数中填充
{
  //在日志中记录接收到的服务请求
  RCLCPP_DEBUG(logger_, "Received costmap service request");

  //初始化四元数用于表示方向，这里设置为无旋转
  tf2::Quaternion quaternion;
  //Roll,Pitch,Yaw值都设为0
  quaternion.setRPY(0.0, 0.0, 0.0);

  //获取代价地图的尺寸(X方向单元格数)
  auto size_x = costmap_->getSizeInCellsX();
  //获取代价地图的尺寸(Y方向单元格数)
  auto size_y = costmap_->getSizeInCellsY();
  //计算总数据长度，即地图单元格的总数
  auto data_length = size_x * size_y;
  //获取指向代价地图数据的指针
  unsigned char * data = costmap_->getCharMap();
  //获取当前时间，用于填充响应消息的时间戳
  auto current_time = clock_->now();

  //设置响应消息的头部时间戳
  response->map.header.stamp = current_time;
  //设置响应消息的参考坐标系
  response->map.header.frame_id = global_frame_;
  //设置地图的宽度(单元格数)
  response->map.metadata.size_x = size_x;
  //设置地图的高度(单元格数)
  response->map.metadata.size_y = size_y;
  //设置地图的分辨率(米/单元格)
  response->map.metadata.resolution = costmap_->getResolution();
  //设置地图的层名称为master
  response->map.metadata.layer = "master";
  //设置地图的加载时间
  response->map.metadata.map_load_time = current_time;
  //设置地图的最后更新时间
  response->map.metadata.update_time = current_time;
  //设置地图原点的X坐标
  response->map.metadata.origin.position.x = costmap_->getOriginX();
  //设置地图原点的Y坐标
  response->map.metadata.origin.position.y = costmap_->getOriginY();
  //设置地图原点的Z坐标(通常为0，因为是2D地图)
  response->map.metadata.origin.position.z = 0.0;
  //将无旋转的四元数转换为消息格式并设置为地图原点的方向
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);
  //调整响应数据数组的大小以匹配地图数据长度
  response->map.data.resize(data_length);
  //将代价地图数据复制到响应消息的数据数组中
  response->map.data.assign(data, data + data_length);
}

}  // end namespace nav2_costmap_2d
