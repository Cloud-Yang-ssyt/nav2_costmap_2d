/*
  这段代码是costmap中的一个数学工具，用于计算一个点到直线段的最短距离。这种计算在机器人导航和避障中是非常常见的需求，特别是在处理costmap数据时，经常需要判断机器人与环境中的障碍物的距离。
*/

//这个头文件在include/nav2_costmap_2d/costmap_math.hpp，这里面有一些莫名其妙的数学操作
#include <nav2_costmap_2d/costmap_math.hpp>

#include <vector>

//定义一个函数，计算一个点(pX, pY)到线段(x0, y0)到(x1, y1)的最短距离
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
  double A = pX - x0;  //计算点到线段起点的x方向差值
  double B = pY - y0;  //计算点到线段起点的y方向差值
  double C = x1 - x0;  //计算线段在x方向的长度
  double D = y1 - y0;  //计算线段在y方向的长度

  double dot = A * C + B * D;  //点乘，计算点到线段起点向量与线段向量的点乘
  double len_sq = C * C + D * D;  //线段长度的平方
  double param = dot / len_sq;  //点到线段向量的投影长度与线段长度比

  double xx, yy;  //声明投影点的坐标

  //判断投影点是否落在线段上
  if (param < 0) 
  {
    xx = x0;  //投影点在线段起点之外，使用线段起点作为最近点
    yy = y0;
  } 
  else if (param > 1)  //投影点在线段终点之外，使用线段终点作为最近点
  {
    xx = x1;
    yy = y1;
  } 
  else 
  {
    xx = x0 + param * C;  //投影点在线段内，计算实际的投影点坐标
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);  //调用distance函数计算点到最近点的距离
}
