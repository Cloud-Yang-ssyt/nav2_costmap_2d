/*
  这个文件是导航栈中处理代价地图的核心部分。它负责创建和管理多层代价地图，并与ros2生命周期节点相结合进行管理。
  这里面涵盖了设置和更新代价地图、加载和管理各种地图层(如静态层、障碍层、膨胀层等)、处理传感器数据以及与其他ros2组件的接口等功能。
*/
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <utility>

//这个头文件是一个管理多层代价地图的类，它允许在基础地图上添加和管理多个地图层
#include "nav2_costmap_2d/layered_costmap.hpp"
//这个头文件是一个用于测量代码执行时间的工具类，常用于性能监控和优化
#include "nav2_util/execution_timer.hpp"
//这个头文件提供的工具用于ros2节点的操作，例如参数处理和节点命名空间的处理
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
//用于创建计时器的头文件
#include "tf2_ros/create_timer_ros.h"
//引入了与机器人操作相关的工具函数的头文件
#include "nav2_util/robot_utils.hpp"
//这个头文件用于设定参数服务响应的消息类型，用于响应动态参数更改请求，表明参数更改操作的成功或失败
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{
/*
  这里补充一下，name, "/", name，这是个什么写法：
    Costmap2DROS(name, "/", name)是一种构造函数的委托调用。这意味着这行代码在Costmap2DROS类的一个构造函数内部调用了另一个构造函数，将执行逻辑传递给另一种构造形式。
    这表示，仅提供一个名称来构造Costmap2DROS对象时(即使用单参数构造函数)，它将自动以 name 作为名称，"/" 作为父命名空间，并再次以name作为局部命名空间，调用另一个更具体的构造函数。
    这是利用了C++11引入的构造函数委托功能，使得构造函数能够重用其他构造函数的代码，减少重复代码，提高代码的维护性。
    这个具体的构造函数调用是这样的：
      Costmap2DROS::Costmap2DROS(
        const std::string & name,
        const std::string & parent_namespace,
        const std::string & local_namespace)
      它接受三个参数：节点的名称、父命名空间和局部命名空间。
      通过这种方式，单参数构造函数实际上是利用已有的三参数构造函数来初始化对象，只不过它为父命名空间提供了默认值"/"，并将局部命名空间设置为与节点名称相同。
*/
//默认构造函数，只需要一个名称参数，将自身的namespace设置为"/"并使用相同的名字作为局部namespace
Costmap2DROS::Costmap2DROS(const std::string & name): Costmap2DROS(name, "/", name) 
{}

Costmap2DROS::Costmap2DROS(
  const std::string & name,
  const std::string & parent_namespace,
  const std::string & local_namespace)
: nav2_util::LifecycleNode(name, "",  //创建一个生命周期节点，没有显式地设置namespace
    rclcpp::NodeOptions().arguments({  //设置节点选项，包括namespace和节点名称
    "--ros-args", "-r", std::string("__ns:=") +
    nav2_util::add_namespaces(parent_namespace, local_namespace),  //组合父和局部namespace
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name})),  //设置节点名称
  name_(name),  //初始化成员变量name_
  parent_namespace_(parent_namespace),  //初始化成员变量parent_namespace_
  default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},  //初始化默认插件列表
  default_types_{  //初始化默认插件类型
    //这个层处理静态地图数据，通常来源于已知的地图文件或者地图服务。负责将静态地图加载到代价地图中，这部分地图不会根据传感器数据动态变化。用于表示环境中永久不变的障碍物和自由空间，如墙壁、建筑物和其他固定结构。
    "nav2_costmap_2d::StaticLayer",
    //这个层实时处理来自不同传感器的障碍物信息，如激光雷达和声纳。动态地更新代价地图中的障碍物信息，以反映环境中临时或移动的障碍物。障碍物层可以根据传感器数据来增加或清除障碍物标记。
    "nav2_costmap_2d::ObstacleLayer",
    //这个层基于现有的障碍物在代价地图上添加缓冲区，这些缓冲区称为“膨胀区域”。膨胀层的目的是创建一个缓冲区域，防止机器人与障碍物的接触，即使它们之间有一定的距离。通过在障碍物周围设置不同代价值的区域，帮助路径规划算法安全地导航机器人，避免碰撞。
    "nav2_costmap_2d::InflationLayer"}
{
  //输出日志信息，表示正在创建代价地图
  RCLCPP_INFO(get_logger(), "Creating Costmap");
  //可以被清除的层列表
  std::vector<std::string> clearable_layers{"obstacle_layer", "voxel_layer", "range_layer"};

  declare_parameter("always_send_full_costmap", rclcpp::ParameterValue(false)); // 始终发送完整代价地图
  declare_parameter("footprint_padding", rclcpp::ParameterValue(0.01f)); // 为机器人的足迹设置填充，以避免轻微碰撞
  declare_parameter("footprint", rclcpp::ParameterValue(std::string("[]"))); // 设置机器人的足迹形状，默认为空
  declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map"))); // 设置全局坐标系，默认为"map"
  declare_parameter("height", rclcpp::ParameterValue(5)); // 设置代价地图的高度为5米
  declare_parameter("width", rclcpp::ParameterValue(5)); // 设置代价地图的宽度为5米
  declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100)); // 设置致命成本阈值为100
  declare_parameter("map_topic", rclcpp::ParameterValue((parent_namespace_ == "/" ? "/" : parent_namespace_ + "/") + std::string("map"))); // 设置代价地图的ROS话题名
  declare_parameter("observation_sources", rclcpp::ParameterValue(std::string(""))); // 设置观察源，默认为空
  declare_parameter("origin_x", rclcpp::ParameterValue(0.0)); // 设置代价地图在全局坐标系中的x坐标原点
  declare_parameter("origin_y", rclcpp::ParameterValue(0.0)); // 设置代价地图在全局坐标系中的y坐标原点
  declare_parameter("plugins", rclcpp::ParameterValue(default_plugins_)); // 设置使用的代价地图插件
  declare_parameter("filters", rclcpp::ParameterValue(std::vector<std::string>())); // 设置代价地图的过滤器
  declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0)); // 设置代价地图的发布频率为1Hz
  declare_parameter("resolution", rclcpp::ParameterValue(0.1)); // 设置代价地图的分辨率为0.1米每像素
  declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link"))); // 设置机器人的基座坐标系为"base_link"
  declare_parameter("robot_radius", rclcpp::ParameterValue(0.1)); // 设置机器人的半径为0.1米
  declare_parameter("rolling_window", rclcpp::ParameterValue(false)); // 设置是否使用滚动窗口模式
  declare_parameter("track_unknown_space", rclcpp::ParameterValue(false)); // 设置是否追踪未知空间
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3)); // 设置坐标变换的容错时间为0.3秒
  /*
    补充一下什么是三元代价地图：
      在三元代价地图中，地图的每个单元通常可以被分类为以下三种类型之一：
      可通过(Free)：这些区域被认为是安全的，没有障碍物，机器人可以自由通过。在数字地图中，这些区域通常表示为低成本或零成本。
      致命障碍(Lethal Obstacle)：这些区域包含不可穿越的障碍，如墙壁、大石头等。在地图上，这些区域通常表示为高成本或最大成本值，表明这里绝对不能通行。
      未知区域(Unknown)：这些区域的状态未知，可能是因为传感器没有覆盖到，或者数据不足以确定是否安全。在数字地图中，这些区域可能被赋予一个特定的默认值，通常不是最低也不是最高，以指示未知状态。
  */
  declare_parameter("trinary_costmap", rclcpp::ParameterValue(true)); // 设置是否使用三元代价地图
  declare_parameter("unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff))); // 设置未知区域的代价值
  declare_parameter("update_frequency", rclcpp::ParameterValue(5.0)); // 设置代价地图的更新频率为5Hz
  declare_parameter("use_maximum", rclcpp::ParameterValue(false)); // 设置是否在合并层时使用最大值
  declare_parameter("clearable_layers", rclcpp::ParameterValue(clearable_layers)); // 设置可以被清除的层
}

//析构函数
Costmap2DROS::~Costmap2DROS()
{
}

nav2_util::CallbackReturn Costmap2DROS::on_configure(const rclcpp_lifecycle::State & )
{
  //打印配置日志信息
  RCLCPP_INFO(get_logger(), "Configuring");
  //获取并设置参数
  getParameters();
  //创建一个互斥排他的回调组，不允许回调组中的回调并行执行
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  //初始化LayeredCostmap对象，设置全局帧、是否滚动窗口以及是否追踪未知空间
  layered_costmap_ = std::make_unique<LayeredCostmap>(global_frame_, rolling_window_, track_unknown_space_);

  //如果地图尺寸没有被锁定，则根据分辨率和其他参数调整地图大小
  if (!layered_costmap_->isSizeLocked()) 
  {
    //根据分辨率调整地图的大小，将地图的宽度和高度（以米为单位）转换为以单元格为单位的数值，然后设置地图的尺寸和原点
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),  //将地图宽度从米转换为单元格数
      (unsigned int)(map_height_meters_ / resolution_),  //将地图高度从米转换为单元格数
      resolution_,  //单元格的大小
      origin_x_,  //地图的原点x坐标
      origin_y_);  //地图的原点y坐标
  }

  //创建tf2_ros的Buffer实例，并将其与节点的时钟关联
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  //创建timer接口来管理时序问题，这个timer接口将在特定的callback group中工作
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface(), callback_group_);
  //将创建的timer接口设置给tf缓冲区
  tf_buffer_->setCreateTimerInterface(timer_interface);
  //创建一个tf监听器，用来订阅和存储tf帧数据
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //遍历所有插件名称
  for (unsigned int i = 0; i < plugin_names_.size(); ++i) 
  {
    //记录正在使用的插件信息
    RCLCPP_INFO(get_logger(), "Using plugin \"%s\"", plugin_names_[i].c_str());

    //根据插件类型和名称创建插件实例
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);

    //锁定代价地图，保证插件添加和初始化的线程安全
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));
    
    //将插件添加到LayeredCostmap中
    layered_costmap_->addPlugin(plugin);

    //初始化插件，设置所需的参数和回调组
    plugin->initialize(layered_costmap_.get(), plugin_names_[i], tf_buffer_.get(), shared_from_this(), callback_group_);

    //解锁
    lock.unlock();

    //打印插件初始化成功的信息
    RCLCPP_INFO(get_logger(), "Initialized plugin \"%s\"", plugin_names_[i].c_str());
  }
  
  //遍历所有的过滤器名称
  for (unsigned int i = 0; i < filter_names_.size(); ++i) 
  {
    //日志输出正在使用的过滤器名
    RCLCPP_INFO(get_logger(), "Using costmap filter \"%s\"", filter_names_[i].c_str());

    //根据过滤器类型创建过滤器实例
    std::shared_ptr<Layer> filter = plugin_loader_.createSharedInstance(filter_types_[i]);

    //加锁，保护多线程操作中的共享资源
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

    //向层叠代价地图中添加过滤器
    layered_costmap_->addFilter(filter);

    //初始化过滤器
    filter->initialize(layered_costmap_.get(), filter_names_[i], tf_buffer_.get(), shared_from_this(), callback_group_);

    //解锁
    lock.unlock();

    //日志输出已初始化的过滤器名
    RCLCPP_INFO(get_logger(), "Initialized costmap filter \"%s\"", filter_names_[i].c_str());
  }

  //创建一个订阅者，订阅机器人足迹信息
  footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>("footprint",rclcpp::SystemDefaultsQoS(), std::bind(&Costmap2DROS::setRobotFootprintPolygon, this, std::placeholders::_1));

  //创建一个发布者，用于发布机器人足迹
  footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("published_footprint", rclcpp::SystemDefaultsQoS());

  //创建代价地图发布器
  costmap_publisher_ = std::make_unique<Costmap2DPublisher>(shared_from_this(), layered_costmap_->getCostmap(), global_frame_, "costmap", always_send_full_costmap_);

  //检查是否使用预设的半径来定义足迹
  if (use_radius_) 
  {
    //使用预设的半径值robot_radius_生成圆形足迹，并将其设置为当前机器人的足迹
    setRobotFootprint(makeFootprintFromRadius(robot_radius_));
  } 
  else //如果不使用半径（即有具体的足迹字符串提供）
  {
    //从字符串中生成新的足迹
    std::vector<geometry_msgs::msg::Point> new_footprint;
    makeFootprintFromString(footprint_, new_footprint);
    setRobotFootprint(new_footprint);
  }

  //创建清除代价地图的服务
  clear_costmap_service_ = std::make_unique<ClearCostmapService>(shared_from_this(), *this);

  //创建单线程执行器并添加回调组
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  //创建执行器线程
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  //返回成功配置
  return nav2_util::CallbackReturn::SUCCESS;
}

//这个函数用于启动代价地图更新和发布机制
nav2_util::CallbackReturn Costmap2DROS::on_activate(const rclcpp_lifecycle::State & )
{
  //输出日志信息，标识正在激活节点
  RCLCPP_INFO(get_logger(), "Activating");

  //激活成本地图发布器
  costmap_publisher_->on_activate();
  //激活足迹发布器
  footprint_pub_->on_activate();

  //定义用于存储变换错误信息的字符串
  std::string tf_error;

  //输出日志，表示正在检查坐标变换
  RCLCPP_INFO(get_logger(), "Checking transform");
  //设置一个频率为每秒两次的定时器
  rclcpp::Rate r(2);
  //循环直到成功获取从机器人基座到全局坐标系的变换或ros2被关闭
  while (rclcpp::ok() && !tf_buffer_->canTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    //如果超时未能获取变换，则输出错误信息
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
    //清除错误信息
    tf_error.clear();
    //等待下一次循环
    r.sleep();
  }

  //设置stopped_标志为true，表示地图更新循环即将开始，但还未正式启动
  stopped_ = true; 
  //设置stop_updates_为false，允许地图更新操作进行
  stop_updates_ = false;
  //设置map_update_thread_shutdown_为false，表示地图更新线程不应该关闭
  map_update_thread_shutdown_ = false;
  //创建一个新的线程，专门用于地图更新循环，频率由map_update_frequency_指定
  map_update_thread_ = std::make_unique<std::thread>(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency_));
  //调用start()方法，正式启动地图更新操作
  start();
  //为这个节点添加一个参数设置的回调，允许动态地调整参数
  dyn_params_handler = this->add_on_set_parameters_callback(std::bind(&Costmap2DROS::dynamicParametersCallback, this, _1));
  //返回成功状态，表示节点已成功配置并准备好激活
  return nav2_util::CallbackReturn::SUCCESS;
}

//这个函数用于安全地停止地图更新和发布操作，并清理相关资源
nav2_util::CallbackReturn Costmap2DROS::on_deactivate(const rclcpp_lifecycle::State & )
{
  //输出日志信息，表明这个节点正在被停用
  RCLCPP_INFO(get_logger(), "Deactivating");

  //重置动态参数处理器，这将取消之前设置的所有参数回调
  dyn_params_handler.reset();

  //调用stop()方法来停止任何正在进行的地图更新操作
  stop();

  //设置map_update_thread_shutdown_为true，通知地图更新线程它应该关闭
  map_update_thread_shutdown_ = true;

  //检查更新线程是否可以被加入，如果可以，则等待线程结束
  if (map_update_thread_->joinable()) 
  {
    map_update_thread_->join();
  }

  //停用地图发布者，这会停止向其他ros2节点发布地图数据
  costmap_publisher_->on_deactivate();
  //停用足迹发布者
  footprint_pub_->on_deactivate();

  //返回成功状态，表明节点已成功停用
  return nav2_util::CallbackReturn::SUCCESS;
}

//这个函数清理在节点的生命周期中创建的所有资源
nav2_util::CallbackReturn Costmap2DROS::on_cleanup(const rclcpp_lifecycle::State & )
{
  //输出日志信息，表明这个节点正在进行清理操作
  RCLCPP_INFO(get_logger(), "Cleaning up");
  //重置层叠地图对象，释放与之关联的所有资源
  layered_costmap_.reset();

  //重置变换监听器和变换缓冲区，这将释放与它们关联的所有资源
  tf_listener_.reset();
  tf_buffer_.reset();

  //重置足迹订阅器和足迹发布器，这样它们就不会再接收或发送消息
  footprint_sub_.reset();
  footprint_pub_.reset();

  //重置地图发布器，停止发布地图
  costmap_publisher_.reset();
  //重置清除地图服务，停止提供该服务
  clear_costmap_service_.reset();

  //重置执行器线程，这将停止所有与之关联的异步操作
  executor_thread_.reset();
  //返回成功状态，表明节点已成功完成清理
  return nav2_util::CallbackReturn::SUCCESS;
}

//用于处理节点的关闭逻辑
nav2_util::CallbackReturn Costmap2DROS::on_shutdown(const rclcpp_lifecycle::State &)
{
  //输出日志信息，表明这个节点正在被关闭
  RCLCPP_INFO(get_logger(), "Shutting down");
  //返回成功状态，表示关闭操作已经成功执行
  return nav2_util::CallbackReturn::SUCCESS;
}

//获取并配置参数
void Costmap2DROS::getParameters()
{
  //输出调试信息，表明正在获取参数
  RCLCPP_DEBUG(get_logger(), " getParameters");

  //获取一个布尔值，决定是否总是发送完整的代价地图
  get_parameter("always_send_full_costmap", always_send_full_costmap_);
  //获取机器人足迹的字符串表示
  get_parameter("footprint", footprint_);
  //获取足迹周围的填充量(padding)，用于增加足迹外围的边界大小
  get_parameter("footprint_padding", footprint_padding_);
  //获取代价地图应该参考的全局坐标系
  get_parameter("global_frame", global_frame_);
  //获取代价地图的高度
  get_parameter("height", map_height_meters_);
  //获取代价地图在全局坐标系中的x轴原点
  get_parameter("origin_x", origin_x_);
  //获取代价地图在全局坐标系中的y轴原点
  get_parameter("origin_y", origin_y_);
  //获取代价地图的发布频率
  get_parameter("publish_frequency", map_publish_frequency_);
  //获取代价地图的分辨率
  get_parameter("resolution", resolution_);
  //获取机器人基座坐标系的名称
  get_parameter("robot_base_frame", robot_base_frame_);
  //获取用于定义机器人圆形足迹半径的值
  get_parameter("robot_radius", robot_radius_);
  //获取一个布尔值，决定代价地图是否为滚动窗口类型
  get_parameter("rolling_window", rolling_window_);
  //获取一个布尔值，决定是否追踪未知空间
  get_parameter("track_unknown_space", track_unknown_space_);
  //获取变换数据时允许的时间容差
  get_parameter("transform_tolerance", transform_tolerance_);
  //获取代价地图更新频率
  get_parameter("update_frequency", map_update_frequency_);
  //获取代价地图的宽度
  get_parameter("width", map_width_meters_);
  //获取将要加载的插件的名称列表
  get_parameter("plugins", plugin_names_);
  //获取将要加载的过滤器的名称列表
  get_parameter("filters", filter_names_);

  //获取当前节点的智能指针
  auto node = shared_from_this();

  //检查当前配置的插件名称是否与默认插件名称相同
  if (plugin_names_ == default_plugins_) 
  {
    //如果是，默认插件列表中的每个插件
    for (size_t i = 0; i < default_plugins_.size(); ++i) 
    {
      //在参数服务器上声明每个插件的参数
      //这里使用的是每个插件的默认类型作为参数的值
      nav2_util::declare_parameter_if_not_declared(node, default_plugins_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }
  //重新设置插件类型和过滤器类型的数组大小
  plugin_types_.resize(plugin_names_.size());
  filter_types_.resize(filter_names_.size());

  //遍历所有配置的插件名称
  for (size_t i = 0; i < plugin_names_.size(); ++i) 
  {
    //使用插件名称从参数服务器获取对应的插件类型，并存储到plugin_types_数组中
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, plugin_names_[i]);
  }
  //遍历所有配置的过滤器名称
  for (size_t i = 0; i < filter_names_.size(); ++i) 
  {
    //使用过滤器名称从参数服务器获取对应的过滤器类型，并存储到filter_types_数组中
    filter_types_[i] = nav2_util::get_plugin_type_param(node, filter_names_[i]);
  }

  //检查地图的发布频率是否大于0
  if (map_publish_frequency_ > 0) 
  { 
    //如果是，计算发布周期，这里是计算每次发布间隔的秒数
    publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
  } 
  else  //如果发布频率不大于0，设置发布周期为无限大，这通常意味着不按定期间隔自动发布
  {
    publish_cycle_ = rclcpp::Duration(-1s);
  }

  //默认设置使用基于半径的足迹
  use_radius_ = true;

  //检查参数服务器中的足迹参数是否已设置且不为空数组
  if (footprint_ != "" && footprint_ != "[]") 
  {
    //创建一个新的geometry_msgs::msg::Point类型的向量来存储解析后的足迹
    std::vector<geometry_msgs::msg::Point> new_footprint;
    //尝试将字符串参数'footprint_'转换成足迹点集'new_footprint'
    if (makeFootprintFromString(footprint_, new_footprint)) 
    {
      //如果转换成功，则不再使用基于半径的默认足迹，而使用转换后的足迹
      use_radius_ = false;
    } 
    else 
    {
      //如果转换失败，记录错误信息，提示足迹参数无效，并继续使用基于半径的足迹
      RCLCPP_ERROR(get_logger(), "The footprint parameter is invalid: \"%s\", using radius (%lf) instead", footprint_.c_str(), robot_radius_);
    }
  }
}

//设置机器人的足迹，包括添加填充(padding)并更新到分层代价地图中
void Costmap2DROS::setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  //将传入的点集赋值给未填充的足迹变量
  unpadded_footprint_ = points;
  //同样将传入的点集赋值给已填充的足迹变量，准备进行填充操作
  padded_footprint_ = points;
  //使用设置的填充值对足迹进行填充处理，更新padded_footprint_变量
  padFootprint(padded_footprint_, footprint_padding_);
  //将填充后的足迹设置到分层代价地图中，用于代价计算和避障等操作
  layered_costmap_->setFootprint(padded_footprint_);
}

//设置机器人的足迹
void Costmap2DROS::setRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  //将Polygon消息转换为点的向量，并设置为机器人的足迹
  setRobotFootprint(toPointVector(footprint));
}

//计算机器人的定向足迹
void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  //尝试获取机器人在全局坐标系下的位置和朝向
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) 
  {
    //如果无法获取位置，中止执行
    return;
  }

  //从全局姿态中提取机器人的朝向(偏航角)
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  //使用机器人的当前位置和朝向来转换足迹点，生成定向足迹
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw, padded_footprint_, oriented_footprint);
}

//这个函数用于定期更新和发布代价地图
void Costmap2DROS::mapUpdateLoop(double frequency)
{
  //打印当前地图更新循环的频率
  RCLCPP_DEBUG(get_logger(), "mapUpdateLoop frequency: %lf", frequency);
  //如果频率设置为0，则立即返回，不进行更新
  if (frequency == 0.0) 
  {
    return;
  }
  //开始进入更新循环
  RCLCPP_DEBUG(get_logger(), "Entering loop");
  //设置循环的频率
  rclcpp::WallRate r(frequency);   
  //当节点正常运行且没有收到停止更新线程的信号时持续循环
  while (rclcpp::ok() && !map_update_thread_shutdown_) 
  {
    //实例化一个执行计时器，用于测量更新耗时
    nav2_util::ExecutionTimer timer;

    //如果没有被暂停更新
    if (!stopped_) 
    {
      //开始计时
      timer.start();
      //更新地图
      updateMap();
      //停止计时
      timer.end();

      //打印更新地图所花费的时间
      RCLCPP_DEBUG(get_logger(), "Map update time: %.9f", timer.elapsed_time_in_seconds());
      //如果发布周期有效且层叠地图已初始化
      if (publish_cycle_ > rclcpp::Duration(0s) && layered_costmap_->isInitialized()) 
      {
        //获取更新后的边界
        unsigned int x0, y0, xn, yn;
        layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
        //将边界发送给地图发布者以准备数据
        costmap_publisher_->updateBounds(x0, xn, y0, yn);

        //获取当前时间
        auto current_time = now();
        //如果上次发布时间加上设定周期小于当前时间或当前时间小于上次发布时间
        if ((last_publish_ + publish_cycle_ < current_time) ||  (current_time < last_publish_))      
        {
          //发布地图数据
          RCLCPP_DEBUG(get_logger(), "Publish costmap at %s", name_.c_str());
          costmap_publisher_->publishCostmap();
          //更新上次发布时间
          last_publish_ = current_time;
        }
      }
    }

    //暂停至下一个循环周期
    r.sleep();

//被注释的代码块，如果循环实际耗时超过预期，则警告
#if 0
    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
    if (r.period() > tf2::durationFromSec(1 / frequency)) 
    {
      RCLCPP_WARN(
        get_logger(),
        "Costmap2DROS: Map update loop missed its desired rate of %.4fHz... "
        "the loop actually took %.4f seconds", frequency, r.period());
    }
#endif
  }
}

//这个函数用来更新代价地图并处理与机器人位姿相关的信息
void Costmap2DROS::updateMap()
{
  //记录正在更新地图的日志
  RCLCPP_DEBUG(get_logger(), "Updating map...");

  //如果没有停止更新的标志
  if (!stop_updates_) 
  {
    //定义一个用于存储机器人位姿的变量
    geometry_msgs::msg::PoseStamped pose;
    //尝试获取机器人的当前位姿
    if (getRobotPose(pose)) 
    {
      //从位姿中提取x坐标
      const double & x = pose.pose.position.x;
      //从位姿中提取y坐标
      const double & y = pose.pose.position.y;
      //从位姿中提取朝向(yaw角)
      const double yaw = tf2::getYaw(pose.pose.orientation);
      //使用当前的x, y和yaw更新地图
      layered_costmap_->updateMap(x, y, yaw);

      //创建一个新的多边形时间戳对象，用于发布当前的足迹
      auto footprint = std::make_unique<geometry_msgs::msg::PolygonStamped>();
      //将当前的时间戳和帧ID赋给足迹对象的头部
      footprint->header = pose.header;
      //调用transformFootprint函数，使用机器人的当前位置(x, y)和朝向(yaw)以及已有的padded_footprint_来生成转换后的足迹
      transformFootprint(x, y, yaw, padded_footprint_, *footprint);

      //记录正在发布足迹的调试信息
      RCLCPP_DEBUG(get_logger(), "Publishing footprint");
      //发布转换后的足迹信息
      footprint_pub_->publish(std::move(footprint));
      //设置initialized_标志为true，表示足迹信息已成功生成并发布
      initialized_ = true;
    }
  }
}

//该函数用于启动成本地图的更新和发布过程
void Costmap2DROS::start()
{
  //记录日志，表示启动成本地图更新过程
  RCLCPP_INFO(get_logger(), "start");
  //获取插件列表的指针，这些插件负责处理不同类型的地图数据
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  //获取过滤器列表的指针，这些过滤器用于在最终成本地图中应用额外的处理或规则
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

  //如果代价地图更新过程之前被停止了，则重新激活所有插件和过滤器
  if (stopped_) 
  {
    //遍历插件列表，并激活每个插件
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
    {
      (*plugin)->activate();
    }
    //遍历过滤器列表，并激活每个过滤器
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin(); filter != filters->end(); ++filter)
    {
      (*filter)->activate();
    }
    //更新停止状态标志，表示成本地图更新过程现在已经启动
    stopped_ = false;
  }
  //更新停止状态标志，表示成本地图更新过程现在已经启动
  stop_updates_ = false;

  //设置一个频率为每秒20次的定时器，用于控制循环的速率
  rclcpp::Rate r(20.0);
  //循环，直到成本地图完全初始化(即initialized_标志为真)
  while (rclcpp::ok() && !initialized_) 
  {
    //记录正在等待地图初始化的调试信息
    RCLCPP_DEBUG(get_logger(), "Sleeping, waiting for initialized_");
    //等待下一个循环周期
    r.sleep();
  }
}

//用于停止代价地图的更新和发布过程
void Costmap2DROS::stop()
{
  //设置停止更新的标志为真，以停止代价地图的更新
  stop_updates_ = true;
  //检查layered_costmap_对象是否存在
  if (layered_costmap_) 
  {
    //获取插件列表的指针
    std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
    //获取过滤器列表的指针
    std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

    //遍历插件列表，并停用每个插件
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
    {
      (*plugin)->deactivate();
    }
    //遍历过滤器列表，并停用每个过滤器
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin(); filter != filters->end(); ++filter)
    {
      (*filter)->deactivate();
    }
  }
  //设置初始化标志为假，表示成本地图需要重新初始化才能使用
  initialized_ = false;
  //设置已停止标志为真，表示成本地图更新过程已完全停止
  stopped_ = true;
}

//用于暂停的函数
void Costmap2DROS::pause()
{
  //设置stop_updates_为true，暂停成本地图的更新
  stop_updates_ = true;
  //设置initialized_为false，标记成本地图为未初始化状态，需要在恢复后重新初始化
  initialized_ = false;
}

//用于恢复代价地图的更新
void Costmap2DROS::resume()
{
  //设置stop_updates_为false，恢复成本地图的更新
  stop_updates_ = false;

  //设置一个循环速率为100Hz
  rclcpp::Rate r(100.0);
  //循环等待，直到成本地图被标记为已初始化
  while (!initialized_) 
  {
    //在循环的每次迭代中暂停一段时间，等待成本地图初始化完成
    r.sleep();
  }
}

//这个函数用于重置成本地图和它的所有层(插件和过滤器)
void Costmap2DROS::resetLayers()
{
  //获取指向成本地图顶层的指针
  Costmap2D * top = layered_costmap_->getCostmap();
  //重置整个成本地图，设置所有成本值为默认值
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

  //获取存储所有插件层的向量的指针
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  //获取存储所有过滤器层的向量的指针
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();
  //遍历所有插件层
  for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
  {
    //调用每个插件的reset函数，重置插件的内部状态
    (*plugin)->reset();
  }
  //遍历所有过滤器层
  for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin(); filter != filters->end(); ++filter)
  {
    //调用每个过滤器的reset函数，重置过滤器的内部状态
    (*filter)->reset();
  }
}

//获取机器人当前在全局坐标系下的位姿
bool Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
{
  //调用nav2_util提供的函数获取当前的机器人位姿，传入的参数包括目标位姿的引用，tf变换缓存，全局坐标系名，机器人基座坐标系名和变换容忍度
  return nav2_util::getCurrentPose(global_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_);
}

//将给定的位姿转换到全局坐标系中
bool Costmap2DROS::transformPoseToGlobalFrame(const geometry_msgs::msg::PoseStamped & input_pose, geometry_msgs::msg::PoseStamped & transformed_pose)
{
  //如果输入的位姿已经在全局坐标系(global_frame_)中
  if (input_pose.header.frame_id == global_frame_) 
  {
    //将输入的位姿直接赋值给输出的位姿
    transformed_pose = input_pose;
    //返回true，表示位姿无需转换即已在全局坐标系中
    return true;
  } 
  else  //如果输入的位姿不在全局坐标系中
  {
    //调用nav2_util库的transformPoseInTargetFrame函数尝试将输入的位姿转换到全局坐标系
    //这个函数需要输入位姿、输出位姿的引用、TF缓冲区、目标坐标系(全局坐标系)和时间容忍度
    return nav2_util::transformPoseInTargetFrame(input_pose, transformed_pose, *tf_buffer_, global_frame_, transform_tolerance_);
  }
}

//动态参数回调函数，用于在运行时处理和应用ros2参数变更
rcl_interfaces::msg::SetParametersResult Costmap2DROS::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  //定义动态参数回调函数
  auto result = rcl_interfaces::msg::SetParametersResult();
  //标志，指示是否需要调整地图尺寸
  bool resize_map = false;

  //遍历所有参数
  for (auto parameter : parameters) 
  {
    //获取参数类型
    const auto & type = parameter.get_type();
    //获取参数名
    const auto & name = parameter.get_name();

    //处理浮点型参数
    if (type == ParameterType::PARAMETER_DOUBLE) 
    {
      //检查参数名是否为“robot_radius”
      if (name == "robot_radius") 
      {
        //从参数中获取数值并设置为机器人半径
        robot_radius_ = parameter.as_double();
        //如果系统配置为使用机器人半径来设置足迹
        if (use_radius_) 
        {
          //根据半径生成足迹并设置
          setRobotFootprint(makeFootprintFromRadius(robot_radius_));
        }
      } 
      //如果参数名是“footprint_padding”
      else if (name == "footprint_padding") 
      {
        //设置足迹填充
        footprint_padding_ = parameter.as_double();
        //更新填充后的足迹
        padded_footprint_ = unpadded_footprint_;
        //应用填充
        padFootprint(padded_footprint_, footprint_padding_);
        //设置到代价地图
        layered_costmap_->setFootprint(padded_footprint_);
      } 
      //如果参数名为"transform_tolerance"
      else if (name == "transform_tolerance") 
      {
        //从参数中获取数值并设置转换容忍度，这个值用于在TF转换中处理延迟和时间戳误差
        transform_tolerance_ = parameter.as_double();
      } 
      //如果参数名为"publish_frequency"
      else if (name == "publish_frequency") 
      {
        //从参数中获取数值并设置地图发布频率
        map_publish_frequency_ = parameter.as_double();
        //如果频率大于0，说明是有效的发布频率
        if (map_publish_frequency_ > 0) 
        {
          //计算发布周期，周期是频率的倒数，单位转换为秒
          publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
        } 
        else  //如果频率小于等于0，处理为无效值
        {
          //设置一个无效的发布周期，标志为-1秒，这可能表示不自动发布
          publish_cycle_ = rclcpp::Duration(-1s);
        }
      } 
      //如果参数名是"resolution"
      else if (name == "resolution") 
      {
        //设置标志以指示地图尺寸需要根据新的分辨率调整
        resize_map = true;
        //从参数中获取新的分辨率值并更新到成员变量
        resolution_ = parameter.as_double();
      } 
      //如果参数名是"origin_x"
      else if (name == "origin_x") 
      {
        //设置标志以指示地图尺寸需要根据新的原点x坐标调整
        resize_map = true;
        //从参数中获取新的原点x坐标值并更新到成员变量
        origin_x_ = parameter.as_double();
      } 
      //如果参数名是"origin_y"
      else if (name == "origin_y") 
      {
        //设置标志以指示地图尺寸需要根据新的原点y坐标调整
        resize_map = true;
        //从参数中获取新的原点y坐标值并更新到成员变量
        origin_y_ = parameter.as_double();
      }
    } 
    //如果参数类型为整数
    else if (type == ParameterType::PARAMETER_INTEGER) 
    {
      //如果参数名称是"width"
      if (name == "width") 
      {
        //检查参数值是否大于0
        if (parameter.as_int() > 0) 
        {
          //设置标志以指示地图尺寸需要调整
          resize_map = true;
          //从参数中获取地图宽度的新值并更新到成员变量
          map_width_meters_ = parameter.as_int();
        } 
        else  //如果提供的宽度不是正数
        {
          //输出错误信息到日志
          RCLCPP_ERROR(
            get_logger(), "You try to set width of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          //设置结果为不成功
          result.successful = false;
          //返回结果并终止进一步执行
          return result;
        }
      } 
      //如果参数名称是"height"
      else if (name == "height") 
      {
        //检查参数值是否大于0
        if (parameter.as_int() > 0) 
        {
          //设置标志以指示地图尺寸需要调整
          resize_map = true;
          //从参数中获取地图高度的新值并更新到成员变量
          map_height_meters_ = parameter.as_int();
        } 
        else  //如果提供的高度不是正数
        {
          //输出错误信息到日志
          RCLCPP_ERROR(
            get_logger(), "You try to set height of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          //设置结果为不成功
          result.successful = false;
          //返回结果并终止进一步执行
          return result;
        }
      }
    } 
    //如果参数类型为字符串
    else if (type == ParameterType::PARAMETER_STRING) 
    {
      //如果参数名称是"footprint"
      if (name == "footprint") 
      {
        //从参数中获取足迹字符串并赋值给成员变量
        footprint_ = parameter.as_string();
        //定义一个点的向量来存储解析后的足迹
        std::vector<geometry_msgs::msg::Point> new_footprint;
        //尝试从字符串解析足迹，如果成功
        if (makeFootprintFromString(footprint_, new_footprint)) 
        {
          //使用解析后的足迹来设置机器人的足迹
          setRobotFootprint(new_footprint);
        }
      } 
      //如果参数名称是"robot_base_frame"
      else if (name == "robot_base_frame") 
      {
        //定义一个字符串来存储可能的转换错误信
        std::string tf_error;
        //记录正在检查坐标变换的信息
        RCLCPP_INFO(get_logger(), "Checking transform");
        //尝试查找从全局坐标系到新的基座坐标系的变换
        if (!tf_buffer_->canTransform(global_frame_, parameter.as_string(), tf2::TimePointZero, tf2::durationFromSec(1.0), &tf_error))
        {
          //如果查找失败，记录超时警告
          RCLCPP_WARN(
            get_logger(), "Timed out waiting for transform from %s to %s"
            " to become available, tf error: %s",
            parameter.as_string().c_str(), global_frame_.c_str(), tf_error.c_str());
          //拒绝更改，并保持原始值
          RCLCPP_WARN(
            get_logger(), "Rejecting robot_base_frame change to %s , leaving it to its original"
            " value of %s", parameter.as_string().c_str(), robot_base_frame_.c_str());
          //设置结果为失败
          result.successful = false;
          //返回失败的结果，并结束函数执行
          return result;
        }
        //如果变换成功找到，更新机器人基座坐标系
        robot_base_frame_ = parameter.as_string();
      }
    }
  }
  //检查是否需要调整地图大小，并确认地图尺寸未被锁定
  if (resize_map && !layered_costmap_->isSizeLocked()) 
  {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),  //计算并设置新的地图宽度
      (unsigned int)(map_height_meters_ / resolution_),  //计算并设置新的地图高度
      resolution_,  //设置地图分辨率
      origin_x_,   //设置地图的x原点
      origin_y_);  //设置地图的y原点
  }
  //设置回调结果为成功
  result.successful = true;
  //返回设置参数结果
  return result;
}

}
