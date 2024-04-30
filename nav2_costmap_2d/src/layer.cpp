/*
  这个文件里实现了一个layer类，Layer 类是代价地图(Costmap)中所有层(如障碍物层、膨胀层等)的基类，提供了基础功能和接口供各种具体的层类型扩展和实现。
  这个文件中的代码定义了层的基本属性和行为，包括初始化、参数管理等。
*/

//这个头文件里包含了很多layer类的定义
#include "nav2_costmap_2d/layer.hpp"

#include <string>
#include <vector>
//这个头文件包含了一组实用函数和工具，这些工具用来简化和支持与节点相关的操作。可能涉及到了参数的声明、处理和验证，节点的配置管理，以及节点生命周期相关的其他辅助功能。
#include "nav2_util/node_utils.hpp"

namespace nav2_costmap_2d
{
//默认构造函数，初始化layered_costmap_为nullptr，初始化name_为空字符串，初始化tf_为nullptr，初始化current_为false，表示该层的数据未更新，初始化enabled_为false，表示该层未启用
Layer::Layer(): layered_costmap_(nullptr), name_(), tf_(nullptr), current_(false), enabled_(false)
{}

//初始化函数，第一个参数是父节点，第二个参数是层的名称，第三个参数是TF缓冲区，用于坐标变换，第四个参数是节点的弱指针(看到这个就知道后面要从弱指针获取强指针了)，第五个参数是个回调组
/*
  这里补充一下，什么是回调组：
  在ros2中，CallbackGroup是一个用来管理回调函数执行方式的组件。
  CallbackGroup允许用户更细致地控制回调(如订阅者的消息回调、服务回调等)的执行，尤其是在多线程环境中,每个回调组可以与一个或多个回调关联，并可以配置为单线程或多线程处理。
  所以总结一下，回调组的本质就是可以管理多个回调执行的、支持多线程的集合。

  ros2中主要有两种类型的回调组：
  Mutually Exclusive（互斥）：
    这种类型的回调组，任何时候只有一个回调可以执行。这适用于需要避免并发执行因素的情景，比如当回调访问共享资源时。
  Reentrant（可重入）：
    可重入的回调组允许其内部的回调并行执行。这种类型适合于回调之间没有依赖或共享状态，且可以安全地并行处理的情况。
*/
void Layer::initialize(LayeredCostmap * parent, std::string name, tf2_ros::Buffer * tf, const nav2_util::LifecycleNode::WeakPtr & node, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  layered_costmap_ = parent;  //设置父Costmap
  name_ = name;  //设置层名称
  tf_ = tf;  //设置TF缓冲区
  node_ = node;  //设置节点
  callback_group_ = callback_group;  //设置回调组
  {
    auto node_shared_ptr = node_.lock();  //尝试从弱指针获取强指针
    logger_ = node_shared_ptr->get_logger();  //获取日志器
    clock_ = node_shared_ptr->get_clock();  //获取时钟
  }

  onInitialize();//调用onInitialize()，由派生类实现具体初始化逻辑
  /*
    这里补充一下onInitialize()类的使用：
    onInitialize()方法通常是一个在基类中声明但由派生类具体实现的虚拟方法。这个方法的目的是提供一个专门的初始化虚函数，允许派生类在其生命周期的特定时刻进行自定义的初始化操作。
    通过这种方式，基类可以在构造时或在某个特定时刻调用onInitialize()，而具体的行为则由各个子类根据其特定需求来定义。
    简单来说，这就是个专门用来重写的虚函数，里面就是用来初始化各个部分的数据。
  */
}

//获取足迹
const std::vector<geometry_msgs::msg::Point> &Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();  //返回父Costmap的足迹
}

//下面声明了两个重载的declareParameter方法，具体怎么调用看传入的参数
//重载版本1，这个版本的 declareParameter 方法用于声明一个新的参数，如果它还未被声明的话。
//声明参数，同时指定默认值
void Layer::declareParameter(const std::string & param_name, const rclcpp::ParameterValue & value)
{
  auto node = node_.lock();  //尝试获取与这个Layer关联的强引用
  if (!node)  //如果获取不到节点的强引用
  {
    throw std::runtime_error{"Failed to lock node"};  //抛出异常，因为无法操作一个未被成功锁定的节点
  }
  local_params_.insert(param_name);  //在本地参数集中记录这个参数名，用于跟踪这个Layer使用的所有参数
  //在节点上声明一个新参数，如果该参数未被声明过，则使用提供的默认值
  nav2_util::declare_parameter_if_not_declared(node, getFullName(param_name), value);
}

//重载版本2，这个版本的 declareParameter 方法用于仅声明参数类型，没有提供具体的默认值。
//声明参数，仅指定参数类型
void Layer::declareParameter(const std::string & param_name, const rclcpp::ParameterType & param_type)
{
  auto node = node_.lock();  //同上，获取节点的强引用
  if (!node) 
  {
    throw std::runtime_error{"Failed to lock node"};  //同上，处理无法锁定节点的情况
  }
  local_params_.insert(param_name);  //记录参数名
  //声明参数，指定参数类型，如果该参数未被声明过，则在节点上创建这样一个参数
  nav2_util::declare_parameter_if_not_declared(node, getFullName(param_name), param_type);
}

//检查节点是否已声明特定的参数
bool Layer::hasParameter(const std::string & param_name)
{
  auto node = node_.lock();  //获取节点的强引用
  if (!node)  //如果无法获取节点的强引用
  {
    throw std::runtime_error{"Failed to lock node"};  //抛出异常
  }
  //返回该参数是否已在节点上声明，使用参数的全名进行检查
  return node->has_parameter(getFullName(param_name));
}

//获取参数的全名
std::string Layer::getFullName(const std::string & param_name)
{
  //将层的名称与参数名称组合，形成一个全名
  return std::string(name_ + "." + param_name);
}

}
