/*
  这个文件的功能主要用于管理和维护从传感器(雷达或摄像头)收集的观测数据。其核心功能包括:存储观测数据，时间管理，坐标转换，高度过滤，有效性检查，观测访问
*/
#include "nav2_costmap_2d/observation_buffer.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

//这个头文件属于tf2(transform library 2)库，主要负责处理坐标变换。tf2 是ROS中用于管理不同坐标系之间转换的库。
#include "tf2/convert.h"
//这个头文件提供了用于处理sensor_msgs::PointCloud2消息的迭代器。PointCloud2是ROS中用于表示3D点云数据的标准消息格式。
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
//构造函数的定义，接受多个参数来初始化观测缓冲区的不同属性
ObservationBuffer::ObservationBuffer(
  const nav2_util::LifecycleNode::WeakPtr & parent,  //弱指针
  std::string topic_name,
  double observation_keep_time,  //观测数据应保持在缓冲区的时间
  double expected_update_rate,  //预期的观测数据更新率
  double min_obstacle_height, double max_obstacle_height,  //可识别障碍物的最小/最大高度
  double obstacle_max_range, double obstacle_min_range,  //障碍物识别的最大/最小范围
  double raytrace_max_range, double raytrace_min_range,  //射线追踪的最大/最小范围
  tf2_ros::Buffer & tf2_buffer,  //TF2库的缓冲区，用于处理坐标变换
  std::string global_frame,  //全局参考坐标系的名称
  std::string sensor_frame,  //传感器数据的坐标系名称
  tf2::Duration tf_tolerance)  //坐标变换的容忍时间间隔
: tf2_buffer_(tf2_buffer),  //初始化成员变量tf2_buffer_
  observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),  //将保持时间转换为rclcpp::Duration对象
  expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),  //将更新率转换为rclcpp::Duration对象
  global_frame_(global_frame),  //初始化全局坐标系名称
  sensor_frame_(sensor_frame),  //初始化传感器坐标系名称
  topic_name_(topic_name),  //初始化话题名称
  min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),  //初始化障碍物高度限制
  obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),  //初始化障碍物识别范围
  raytrace_max_range_(raytrace_max_range), raytrace_min_range_(raytrace_min_range),  //初始化射线追踪范围
  tf_tolerance_(tf_tolerance)  //初始化坐标变换容忍度
{
  auto node = parent.lock();  //尝试获取父节点的强引用
  clock_ = node->get_clock();  //获取与节点关联的时钟
  logger_ = node->get_logger();  //获取与节点关联的日志记录器
  last_updated_ = node->now();  //设置最后更新时间为当前时间
}

//析构函数
ObservationBuffer::~ObservationBuffer()
{}

//函数定义，接收一个PointCloud2消息作为输入
void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  //创建一个全局原点存储变换后的原点位置
  geometry_msgs::msg::PointStamped global_origin;
  //在观测列表前端插入一个新的观测对象
  observation_list_.push_front(Observation());
  //如果sensor_frame_为空字符串，则使用cloud的frame_id作为原点帧，否则使用sensor_frame_
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try 
  {
    //创建一个点的消息，用来表示传感器在其自身坐标系中的原点位置。
    geometry_msgs::msg::PointStamped local_origin;
    //设置原点的时间戳为点云的时间戳，确保时间同步。
    local_origin.header.stamp = cloud.header.stamp;
    //设置原点的坐标系ID。如果sensor_frame_为空，则使用点云消息头部的frame_id作为坐标系ID。
    local_origin.header.frame_id = origin_frame;
    //初始化原点的空间坐标，这里设置为(0,0,0)，表示传感器自身位置的原点。
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    //使用tf2库将本地原点转换到全局坐标系
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
    //将转换后的全局原点坐标存入最新的观测中
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    //设置新观测数据的最大射线追踪范围，这是处理点云数据时，从传感器原点到障碍物终点允许的最大距离。
    observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
    //设置新观测数据的最小射线追踪范围，这是处理点云数据时，从传感器原点到障碍物终点允许的最小距离。
    observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
    //设置新观测数据的最大障碍物检测范围，这定义了系统应该识别障碍物的最大有效距离。
    observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
    //设置新观测数据的最小障碍物检测范围，这定义了系统应该识别障碍物的最小有效距离。
    observation_list_.front().obstacle_min_range_ = obstacle_min_range_;

    //用于存储转换后的点云数据
    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    //将输入的点云转换到全局坐标系
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    //更新转换后点云的时间戳
    global_frame_cloud.header.stamp = cloud.header.stamp;

    //获取用于存放过滤后点云的对象引用
    sensor_msgs::msg::PointCloud2 & observation_cloud = *(observation_list_.front().cloud_);
    //设置转换后的全局帧点云的高度，即点云中点的行数，对于非组织点云通常为1。
    observation_cloud.height = global_frame_cloud.height;
    //设置转换后的全局帧点云的宽度，即每行中的点数，对于非组织点云，它是点的总数。
    observation_cloud.width = global_frame_cloud.width;
    //设置转换后的全局帧点云中点的数据结构，例如X、Y、Z坐标，可能还包括颜色、强度等。
    observation_cloud.fields = global_frame_cloud.fields;
    //设置转换后的全局帧点云数据的字节顺序，bigendian(大端)或littleendian(小端)。
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    //设置转换后的全局帧点云中每个点的字节步长，即存储每个点需要的字节数。
    observation_cloud.point_step = global_frame_cloud.point_step;
    //设置转换后的全局帧点云中每行的字节步长，即存储一行点需要的字节数。
    observation_cloud.row_step = global_frame_cloud.row_step;
    //指示转换后的全局帧点云中的数据是否为密集型（无无效点），如果所有点都有效，则为true，否则为false。
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    //计算点云中的点数量
    unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;
    //初始化修改器，用于调整点云大小
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    //初始化一个用于访问点云中z值的迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    //初始化指向全局帧点云数据开始和结束的迭代器
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    //初始化一个迭代器用于构建观测点云数据
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    //遍历全局帧点云数据，只保留符合高度限制的点
    for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step)
    {
      //检查当前点的z值是否在指定的高度范围内
      if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_)
      {
        //如果在高度范围内，则将该点数据复制到观测点云中
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        //更新观测点云的数据迭代器
        iter_obs += global_frame_cloud.point_step;
        //增加有效点计数
        ++point_count;
      }
    }

    //调整观测点云的大小以匹配有效点的数量
    modifier.resize(point_count);
    //设置观测点云的时间戳和帧ID
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  } 
  //捕获和处理任何可能发生的转换异常
  catch (tf2::TransformException & ex) 
  {
    //如果发生异常，从观测列表中移除当前的观测点云
    observation_list_.pop_front();
    //记录异常信息
    RCLCPP_ERROR(logger_, "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }
  //更新最后一次数据更新的时间
  last_updated_ = clock_->now();
  //清理任何过时的观测数据
  purgeStaleObservations();
}

//从观测缓冲区中获取所有有效的观测数据，并将它们复制到提供的向量中
void ObservationBuffer::getObservations(std::vector<Observation> & observations)
{
  //清理过时的观测数据，确保返回的数据是最新的
  purgeStaleObservations();

  //定义一个迭代器用于遍历观测列表
  std::list<Observation>::iterator obs_it;
  //遍历观测列表，将每一个观测数据复制到提供的向量中
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) 
  {
    //将观测数据添加到观测向量中
    observations.push_back(*obs_it);
  }
}

//从观测缓冲区中移除过时的观测数据
void ObservationBuffer::purgeStaleObservations()
{
  //检查观测列表是否为空，如果不为空，则开始检查每个观测数据的时间戳是否过时
  if (!observation_list_.empty()) 
  {
    //定义迭代器指向观测列表的开始位置
    std::list<Observation>::iterator obs_it = observation_list_.begin();
    //如果设置的观测保持时间为零，则移除除第一个之外的所有观测数据
    if (observation_keep_time_ == rclcpp::Duration(0.0s)) 
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    //遍历观测列表
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) 
    {
      //获取当前观测数据的引用
      Observation & obs = *obs_it;
      
      //计算当前时间与观测数据时间戳的差值，如果大于设定的保持时间，则移除这个观测数据及之后的所有观测数据
      if ((clock_->now() - obs.cloud_->header.stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

//检查缓冲区是否是当前的，即数据是否是最新的
bool ObservationBuffer::isCurrent() const
{
  //如果期望的更新频率设置为0秒，则认为缓冲区始终是最新的
  if (expected_update_rate_ == rclcpp::Duration(0.0s)) 
  {
    return true;
  }

  //计算从最后更新时间到现在是否超过了期望的更新频率
  bool current = (clock_->now() - last_updated_) <= expected_update_rate_;
  //如果不是最新的，则在日志中发出警告
  if (!current) 
  {
    //这个日志也是又臭又长
    RCLCPP_WARN(
      logger_,
      "The %s observation buffer has not been updated for %.2f seconds, "
      "and it should be updated every %.2f seconds.",
      topic_name_.c_str(),  //第一个%s对应的实际值，表示观测缓冲区的主题名
      (clock_->now() - last_updated_).seconds(),  //第一个%.2f对应的实际值，表示从最后一次更新到现在过去的秒数
      expected_update_rate_.seconds());  //第二个%.2f对应的实际值，表示期望的更新频率
  }
  //返回缓冲区的当前状态
  return current;
}

//重置最后一次更新的时间戳
void ObservationBuffer::resetLastUpdated()
{
  //更新最后一次更新的时间戳为当前时间
  last_updated_ = clock_->now();
}
}  
