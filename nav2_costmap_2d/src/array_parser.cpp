/*
  这个文件中定义的主要函数parseVVF()是用于解析一个特定格式的字符串，将其转换为一个二维浮点数向量（std::vector<std::vector<float>>）。
  这个功能在机器人导航和地图处理中非常有用，尤其是在处理代价地图(costmap)这样的数据结构时，其中可能需要从配置文件或网络消息中读取并解析表示不同区域成本的数据。
  而这项功能就可以通过这个文件完成
*/

#include <cstdio>  //这个头文件主要是为了使用EOF(end of file)这个宏定义，这个宏常用于检测输入是否结束。
#include <string>
#include <sstream>
#include <vector>

//这里定义了一个nav2_costmap_2d的命名空间，一方面是为了将此空间下的变量、函数和其他空间下区分开；另一方面是用来封装内部使用的一系列方法
namespace nav2_costmap_2d
{
//这里的两个输入参数：input为输入字符串，格式应为嵌套数组的形式，例如"[[1.0, 2.0], [3.3, 4.4, 5.5], ...]"
//error_return是返回错误信息的字符串引用
//函数返回值：一组二维浮点数向量
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
{
  //定义了最终要返回的结果，一组二维浮点数向量
  std::vector<std::vector<float>> result;

  //用输入字符串初始化字符串流，方便逐字符解析
  std::stringstream input_ss(input);

  //定义了当前解析的深度，用于处理嵌套的方括号
  int depth = 0;
  /*
    这里做详细解释：什么叫处理嵌套的方括号呢？
    答：在解析像 [[1.0, 2.0], [3.3, 4.4, 5.5], ...] 这样的字符串时，depth变量帮助跟踪当前字符解析的context：
    当遇到一个开方括号 [ 时，depth 会增加1。这表示解析器进入了一个新的数组层级。
    当遇到一个闭方括号 ] 时，depth 会减少1。这表示解析器离开了当前的数组层级。

    为什么需要这个变量？
    1. 嵌套结构管理：嵌套数组结构需要知道每个元素属于哪个层级，尤其是当需要将这些元素正确地归类到相应的数据结构中时。
    2. 错误检测：depth 变量还用于检测格式错误。例如，如果在没有相匹配的开方括号的情况下遇到闭方括号，或者解析结束时depth不为零，这些都是格式错误。
    3. 决定数据的添加时机：在本例中，当 depth 从2减少到1时，意味着一个内层数组（例如 [1.0, 2.0]）已经被完全解析，此时应将这个数组的数据添加到外层结构中。

    使用示例：
    一个嵌套字符串："[[1.0, 2.0], [3.3, 4.4]]"
    初始化：depth = 0
    读取第一个 [：depth = 1（进入第一层数组）
    读取第二个 [：depth = 2（进入第二层数组，开始收集 1.0, 2.0）
    读取第一个 ]：depth = 1（离开第二层数组，应该把 1.0, 2.0 添加到结果中）
    读取第三个 [：depth = 2（再次进入第二层数组，开始收集 3.3, 4.4）
    读取第二个 ]：depth = 1（再次离开第二层数组，应该把 3.3, 4.4 添加到结果中）
    读取最后一个 ]：depth = 0（离开第一层数组，整个解析完成）
  */

  //新建一个浮点数向量，用来存储当前解析的浮点数向量
  std::vector<float> current_vector;

  //循环读取每个字符直到字符串流结束
  while (!!input_ss && !input_ss.eof()) 
  {
    //查看下一个字符，但不从字符流中移除
    switch (input_ss.peek()) 
    {
      case EOF:  //如果是文件结束符，直接结束
        break;
      case '[':   //遇到开方括号，深度加一
        depth++;
        if (depth > 2)  //如果深度大于2，说明格式不正确
        {
          error_return = "Array depth greater than 2";  //返回错误的字符串
          return result;
        }
        input_ss.get();  //移除当前字符
        current_vector.clear();  //清空当前向量里的数字，准备存储新的一组数字
        break;
      case ']':  //遇到闭方括号，深度减一
        depth--;
        if (depth < 0)  //如果闭方括号多于开方括号，格式错误
        {
          error_return = "More close ] than open [";  //返回错误字符串
          return result;
        }
        input_ss.get();  //移除当前字符
        if (depth == 1)  //如果此时深度为1，表示一个向量结束
        {
          //将当前字符写入向量组
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        //上面这三个是表示如果是逗号或空白字符，就直接跳过，移除这个字符
        input_ss.get();
        break;  
      default:  
        if (depth != 2)  //判断是否在第二层深度
        {
          //如果数字不在第二层深度，即 depth 的值不为 2，那么格式就认为是错误的
          std::stringstream err_ss;  //创建一个字符串流，用于构建错误消息
          //构建错误消息，消息里包括当前字符
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          //将错误消息字符串赋值给error_return变量，传递给调用者
          error_return = err_ss.str();
          //返回当前已解析的结果，由于发现格式错误，可能不完整或为空(我感觉这里完全可以不返回。。。)
          return result;
        }
        //下面是判断深度确实在第二层就可以执行。
        //定义一个float类型的变量，用于存储从字符串流中解析出来的数字
        float value;
        //从字符串流中读取一个浮点数，并存储在变量 value 中
        input_ss >> value;
        if (!!input_ss)  //检查读取操作是否成功
        {
          current_vector.push_back(value);  //如果成功，将解析出的浮点数添加到当前向量中
        }
        break;  //跳出当前case，继续循环处理输入流的下一个字符
    }
  }

  //完成解析后，检查深度是否为0，即所有开闭方括号是否匹配
  if (depth != 0) 
  {
    error_return = "Unterminated vector string.";  //如果不匹配就返回错误报告
  } 
  else 
  {
    error_return = "";  //如果没有错误，不返回错误信息。
  }
  //返回解析的结果
  return result;
}
}
