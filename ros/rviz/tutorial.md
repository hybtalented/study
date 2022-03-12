这篇文章主要介绍如何使用`rviz`, 并通过 c++ 代码创建 rviz 的标记(Marker, 包括基本图形, 点和线), 并与相应的 rviz 标记进行交互; 然后我们将学习如何创建 `rviz` 的插件, 集成`rviz`的组件到我们自己的应用以及对 3D Stereo 进行配置以对 `rviz` 进行渲染.


在开始学习前, 我们先要在 `catkin_ws` 工作空间下创建一个 `rviz_tutorials` 包, 下面的相关代码将在这个包内编译和执行.
```shell
# 进入 catkin_ws 空间的根目录
cd src
catkin_create_pkg rviz_tutorials roscpp visualization_msgs
# 退回 catkin_ws 空间的根目录
cd ..
```
#  `rviz` 基本图形创建

这一小节, 我们将会通过 c++ 代码想 rviz 传送图形, 图形是通过  `visualization_msgs::Marker` 主题的消息传送到 `rviz` 的. 

首先我们需要在  `rviz_tutorials` 的 `src` 目录下创建一个 `basic_shape.cpp` 文件, 并输入如下内容
```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "basic_shape");
  ros::NodeHandle handle;
  // 一秒钟运行一次
  ros::Rate r(1);
  ros::Publisher marker_pub =
      handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker;
  // 设置标记的框架
  marker.header.frame_id = "/my_frame";
  // 设置标记的命名空间和 id, 如果多次发布同一个 命名空间和 id 的标记到 rviz,
  marker.ns = "basic_shape";
  marker.id = 0;
  // 将会覆盖掉之前发送的标记 设置标记的初始形状
  marker.type = visualization_msgs::Marker::CUBE;
  // action 表示操作, ADD 表示添加标记, DELETE 表示删除标记, DELETEALL
  // 表示删除所有标记, MODIFY 表示修改标记
  marker.action = visualization_msgs::Marker::ADD;
  // 设置标记的位置
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  // 设置标记的方向
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // 设置标记的颜色
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  // 设置标记的大小
  marker.scale.x = 5.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // lifetime 用于告诉 rviz 多长时间后自动删除该标记,  ros::Duration()
  // 表示永不删除
  marker.lifetime = ros::Duration();
  while (ros::ok()) {
    // 如果尚未有任何主题订阅在, 则进行等待
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscribe to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    r.sleep();
    switch (marker.type) {
    case visualization_msgs::Marker::CUBE:
      marker.type = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      marker.type = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      marker.type = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      marker.type = visualization_msgs::Marker::CUBE;
      break;
    }
  }
  return 0;
}
```
并在相应的包级别的 `CMakeLists.txt` 文件中添加如下代码
```cmake
add_executable(basic_shape_node src/basic_shape.cpp)
target_link_libraries(basic_shape_node
  ${catkin_LIBRARIES}
)
```
然后执行, 编译代码
```shell
#终端1 进入catkin_ws的根目录
catkin_make
```

下面我们将查看图形的效果, 首先打开 `rviz` 
```shell
# 终端 2
# 加载环境变量
. /opt/ros/melodic/setup.bash
# 启动 roscore
roscore&
# 启动 rviz
rosrun rviz rviz
```
然后在另外一个终端启动 `basic_shape_node` 节点
```shell
# 终端 1
# 加载环境变量
. ./devel/setup.bash
rosrun rviz_tutorials basic_shape_node
```
然后我们在 `rviz` 窗口中设置视图内容 `Global Options` 中的固定框架为 `/my_frame`, 然后添加一个视图内容 `Marker`, 如下图所示

![基本图形配置](./images/basic_shape.png)

完成上述步骤后, 我们可以在 `rviz` 的3D预览面板中看到一个图形, 并且每隔一秒图形会进行 *长方体* -> *椭球* -> *箭头* -> *椭圆柱体* -> *长方体* 的变换.

# `rviz` 中添加点和线

在这一小节中, 我们将会想 rviz 中添加点和线. 点和线也是通过 `visualization_msgs::Marker` 消息传送 `rivz` 中, 其中 `POINTS` 类型会添加一系列的点, `LINE_STRIP` 类型会将一系列的点连接处一条线, `LINE_LIST` 类型则会将每两个点连接成一系列的线.

在 `rviz_tutorials` 包的 `src` 目录下创建 `points_and_lines.cpp` 文件, 并输入如下内容
```cpp
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle handle;
  ros::Publisher marker_pub =
      handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate rate(30);
  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id =
      line_list.header.frame_id = "/my_frame";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp =
      ros::Time::now();
  points.action = line_strip.action = line_list.action =
      visualization_msgs::Marker::ADD;
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.pose.orientation.w = line_strip.pose.orientation.w =
      line_list.pose.orientation.w = 1.0;
  // 点通过 scale.x, scale.y 属性设置宽度和高度
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // 线和线段通过 scale.x 设置线宽
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;
  // 设置点为绿色
  points.color.g = 1.0f;
  points.color.a = 1.0f;

  // 设置线为蓝色
  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0f;

  // 设置线段为红色
  line_list.color.r = 1.0f;
  line_list.color.a = 1.0f;
  double start_angle = 0.0f;

  while (ros::ok()) {
    points.points.clear();
    line_strip.points.clear();
    line_list.points.clear();
    // 添加圆 x^2 + y^2 = 25 上的点
    for (int32_t i = 0; i < 100; ++i) {
      double a = start_angle + i / 100.0 * 2 * M_PI;
      geometry_msgs::Point p;
      p.x = i - 50;
      p.y = 5 * sin(a);
      p.z = 5 * cos(a);
      points.points.push_back(p);
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    rate.sleep();
    start_angle += 0.04;
  }
  return 0;
}
```

然后在包的 `CMakeLists.txt` 文件中添加
```cmake
add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines
  ${catkin_LIBRARIES}
)
```

最后执行 `catkin_make` 命令完成编译.

然后跟上一小节类似, 启动 `rviz` 和 `points_and_lines` 节点, 配置好固定框架并添加好 `Marker`视图内容后, 可以在 `rviz` 的图形预览面板中看到如下的内容
![基本图形配置](./images/points_and_lines.png)

# rviz 交互服务器编写

rviz 中的图形可以通过交互服务器进行控制, 用户可以通过控制改变可交互的图形的位置与旋转, 也可以点击相应的图形进行选择或者弹出菜单. 这些可交互的图形通过`visualization_msgs/InteractiveMarker` 主题的消息传送到 rviz 中, 一个可交互的图形包括若干个基本图形(`visualization_msgs/Marker`), 一个右键菜单, 以及若干个控制器(`visualization_msgs/InteractiveMarkerControl`), 并且通过一个回调函数处理控制消息. 如下图所示
![可交互图形的结构](./images/interactive_marker_structure.png)

可交互图形需要在一个节点上创建一个 `InteractiveMarkerServer` 对象, 用于建立和 rviz 的连接, 这个连接用于将改变被传输到 rviz, 并将用户与图形的交互通知到该节点.
![可交互图形的结构](./images/interactive_marker_architecture.png)

## 基本交互服务器的编写

在这一小节我们将实现一个简单的交互服务器.

首先创建一个 c++ 源文件 **simple_interactive_marker.cpp**, 并在文件中输入如下代码, 在代码中创建了一个基本的可交互图形, 并实现了图形的交互控制.

```cpp
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

/**
 * @brief  用户交互处理函数
 * @param feedback 交互的具体内容
 * 
 * 这个处理函数仅仅打印了图形的当前位置
 */
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  geometry_msgs::Point position = feedback->pose.position;
  ROS_INFO_STREAM(feedback->marker_name << "is now at (" << position.x << ","
                                        << position.y << "," << position.z
                                        << ")");
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simple_interactive_marker");
  ros::NodeHandle handle;

  // 在命名空间 simple_marker 上创建一个交互服务器
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // 创建一个可交互的图形
  visualization_msgs::InteractiveMarker int_marker;

  // 设置图形的框架和时间戳
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();

  // 设置图形的名称以及描述
  int_marker.name = "my_marker";
  int_marker.description = "simple 1-DOF Control";

  // 创建一个灰色的基本盒子
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // 创建一个不可交互的控制器, 用于放置盒子
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // 创建一个控制器用于移动可交互的图形, 这个控制器中不包含任何基本图形, 这会让
  // rviz 生成一对箭头.
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // 在可交互图形上添加各个控制器
  int_marker.controls.push_back(box_control);
  int_marker.controls.push_back(move_control);

  // 将可交互图形放到交互服务器中, 并告诉服务器在与该图形交互时调用
  // processFeedback
  server.insert(int_marker, &processFeedback);

  server.applyChanges();

  ros::spin();
  return 0;
}
```

然后修改 cmake 文件

```cmake
## 使用 visualization_msgs 和 interactive_markers 两个包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  interactive_markers
)
# 添加构建目标
add_executable(simple_interactive_marker src/simple_interactive_marker.cpp)
target_link_libraries(simple_interactive_marker
  ${catkin_LIBRARIES}
)
```

通过 `catkin_make` 完成目标的构建。 下面我们将在 rviz 中添加该图形。

首先启动 `roscore` 和 `rviz`

```shell
# 终端 2
# 加载环境变量
. /opt/ros/melodic/setup.bash
# 启动 roscore
roscore&
# 启动 rviz
rosrun rviz rviz
```

然后在启动 `simple_interactive_marker` 节点
```shell
# 终端 1
# 加载环境变量
. ./devel/setup.bash
rosrun rviz_tutorials simple_interactive_marker
```

最后在 rviz 中添加相应的可交互图形, 并按照下图所示进行配置
![基本可交互图形](./images/simple_interactive_marker.png)

然后点击两个箭头就可对图形进行移动， 并在`simple_interactive_marker` 的命令行窗口中，可以看到图形的当前位置。

# rviz 常用的交互控制方式

可以在[这里]()找到这个小节的完整代码。

## 基本的3D控制
rviz 中最基础的控制方式包括6个自由度的控制器， 以及相对与摄像头平面的 3D 旋转和平移控制。

其中 6 自由度控制器包括旋转控制盘以及平移控制轴，在交互图形上直接添加不包含任何图形的交互控制器。创建一个旋转控制盘只需要将控制器的 `control.interaction_mode`  设置为 `InteractiveMarkerControl::ROTATE_AXIS`，并通过`control.orientation` 设置圆盘的法线方向， 交互图形的旋转可以通过点击对应的圆盘并移动鼠标控制，在圆盘平面内旋转； 创建一个平移控制轴需要设置控制器 `control.interaction_mode`  设置为 `InteractiveMarkerControl::MOVE_AXIS`， 并通过 `control.orientation` 设置轴的指向， 图形的平移可以通过点击对应的轴并移动鼠标，在轴所在的直线上移动。
```cpp
InteractiveMarker int_marker;
InteractiveMarkerControl control;
control.orientation.w = 1.0;
control.orientation.x = 1.0;
control.orientation.y = 0.0;
control.orientation.z = 0.0;
control.name = "rotate_x";
control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
int_marker.controls.push_back(control);
control.name = "move_x";
control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);
```
对于6自由度的控制器， 可以设置其方向模式为固定方向模式，如下所示

```shell
control.orientation_mode = InteractiveMarkerControl::FIXED;
```
 如过方向模式为固定方向模式， `control.orientation`的方向将相对于交互图形所绑定的框架， 而默认的方向模式， `control.orientation` 所指定的方向相对于图形自身的框架。

 相对与摄像头平面的 3D 旋转和平移控制可以通过在控制器内添加一个或多个基本图形，并设置盒子控制器的 `interaction_mode` 为 `InteractiveMarkerControl::MOVE_3D` 、
   `InteractiveMarkerControl::ROTATE_3D`或者
     `InteractiveMarkerControl::MOVE_ROTATE_3D`。
 ```cpp
InteractiveMarkerControl &boxControl = makeBoxControl(int_marker);
boxControl.interaction_mode = interactive_mode;
 ```
其中 `makeBoxControl` 的实现见下面的完整代码。

上述三总3D控制模式的控制方式如下

* `MOVE_3D` - 点击对应的基本图形并移动鼠标，图形将会在摄像机平面内移动到鼠标的位置， 如果上述过程中按住 *SHIFT* 目标图形会在摄像机平面垂直方向上移动。
* `ROTATE_3D` - 按住基本图形并移动鼠标，将会绕着摄像机平面竖直和水平方向转动， 如果在上述过程中同时按住 `SHIFT` 键， 基本图形将会绕着垂直与摄像机平面的方向转动。
* `MOVE_ROTATE_3D` - `MOVE_3D` 和 `ROTATE_3D` 的结合， 如果未按下 *CTRL* 键， 运动模式和  `MOVE_3D`一致， 如果按下 *CTRL* 键， 运动模式和 `ROTATE_3D` 一致。

## View Facing 控制模式

View Facing 控制模式基于摄像头平面， 包括一个垂直于摄像头平面的选择控制器，以及可以任意在摄像头平面内运动的平移控制模式

```cpp
// 创建一个围绕着与摄像头平面垂直的轴的旋转控制器
control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
control.orientation.w = 1;
control.name = "rotate";

int_marker.controls.push_back(control);
// 创建一个盒子，拖动盒子可以在摄像头平面内运动
control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
// 如果设置为 false 盒子将会一直面向摄像机平面
control.independent_marker_orientation = true;
control.name = "move";

control.markers.push_back(makeBox(int_marker));
```

如上所示，旋转控制器可以通过设置 `control.orientation_mode`为`InteractiveMarkerControl::VIEW_FACING;`使其一直面向摄像头平面，同理二维平移控制的方向模式 `orientation_mode` 也需要设置为 `VIEW_FACING`， 使其能够平行于摄像头平面移动。

## 四角直升(Quadrocopter)控制模型

四角直升控制模型包括一个与z轴平行的旋转移动控制器，以及一个z方向的一维移动控制轴。

其中水平旋转控制器需要将方向设置为 z 方向，并将设置`control.interaction_mode` 为 `InteractiveMarkerControl::MOVE_ROTATE`；而垂直移动控制器也需要设置方向为 z 方向， 而它的 `interaction_mode` 设置为 `MOVE_AXIS`， 即一维平移控制。**这种情况下，rviz会自动生成上下两个一维控制轴。**
```cpp
control.orientation.x = 0;
control.orientation.y = 1;
control.orientation.z = 0;
control.orientation.w = 1;
// 创建一个圆环，拖动圆环可以进行在 orientation 垂直的平面内旋转和移动
control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
int_marker.controls.push_back(control);
// 创建一个一维平移移动轴， orientation 为轴的方向
control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);
```

## 棋盘对齐控制模式

棋盘对齐控制模式可以通过点击环形圈或者目标图形本身对目标图形进行移动，结束控制后，会自动将图形对齐的棋盘网格的中心。


```shell
control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
control.name = "move_plane";
int_marker.controls.push_back(control);

control.markers.push_back(makeBox(int_marker));
control.always_visible = true;
control.name = "box";
int_marker.controls.push_back(control);
```
图形的平面控制器只需要将相应的标记控制器的`interaction_mode` 设置为 `MOVE_PLANE`， 而图形的对齐则是控制控制回调的方式配置， 如下所示，对于该图形我们额外指定了一个反馈回调函数 `alignMarkerFeedback`。
```cpp
server.setCallback(int_marker.name, &processCallback);
server.setCallback(int_marker.name, alignMarkerFeedback,
                     InteractiveMarkerFeedback::POSE_UPDATE);
```
`alignMarkerFeedback` 回调函数的主要作用是在图形的位置更新时，修改图形的坐标使图形对齐到网格，并通知到 rviz. 位置更新回调函数的实现如下
```cpp
interactive_markers::InteractiveMarkerServer::FeedbackCallback alignMarkerFeedback = [&](const InteractiveMarkerFeedbackConstPtr &feedback) {
    geometry_msgs::Pose pose = feedback->pose;
    pose.position.x = round(pose.position.x - 0.5) + 0.5;
    pose.position.y = round(pose.position.y - 0.5) + 0.5;

    ROS_INFO_STREAM(
        feedback->marker_name
        << ": aligning position =" << feedback->pose.position.x << ","
        << feedback->pose.position.y << "," << feedback->pose.position.z
        << " to position " << pose.position.x << "," << pose.position.y
        << "," << pose.position.z);
    server.setPose(feedback->marker_name, pose);
    server.applyChanges();
};
```

## 二维数控转台模型
二维数控转台模型包括一个水平并且方向固定的旋转控制器，以及一个竖直的旋转控制器，可以用于可以左右和上下旋转的炮台或者雷达等。

水平的旋转控制器需要设置其旋转轴方向为 z 方向， 并且设置器方向模式 `orientation_mode` 为 `FIXED`， 竖直的旋转轴则只需要将在旋转轴设置在 X-Y 平面内。
```cpp
control.orientation.x = 0;
control.orientation.y = 0;
control.orientation.z = 1;
control.orientation.w = 1;

// 创建一个法向量为 y 方向的倾斜旋转控制器
control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
int_marker.controls.push_back(control);

control.orientation.y = 1;
control.orientation.z = 0;
// 创建一个垂直于 z 方向并且方向固定的平面旋转控制器
control.orientation_mode = InteractiveMarkerControl::FIXED;
int_marker.controls.push_back(control);
```

## 右键菜单

rviz 中可以通过 `interactive_markers::MenuHandler` 创建一个右键菜单， 下面的代码分别创建一个两个 `First Entry` 和 包含子选项的 `Submenu Entry` 菜单入口， 并在 `Submenu Entry` 菜单下添加一个包含 `First Entry` 的子菜单选项。**insert 函数会返回添加的菜单唯一标识**
```cpp
// 菜单创建
interactive_markers::MenuHandler menu_handler;
menu_handler.insert("First Entry", &processCallback);
  // 创建一个包括子选项的菜单
interactive_markers::MenuHandler::EntryHandle sub_menu_handle =
    menu_handler.insert("Submenu Entry");
  // 创建子选项
menu_handler.insert(sub_menu_handle, "First Entry", &processCallback);
```
为了对目标图形上调用该菜单，我们需要添加一个控制器，并设置其交互模式 `interaction_mode` 为 `MENU`。**注意： 如果对应的控制器下不包含基本图形，会自动在交互图形上创建一个浮动的文本图形**
```cpp
control.interaction_mode = InteractiveMarkerControl::MENU;
```
然后我们还需要将菜单绑定到图形上,
```cpp
menu_handler.apply(server, int_marker.name);
```
在菜单的相应选项选择时， 我们可以在反馈回调函数中获取到 `InteractiveMarkerFeedback::MENU_SELECT` 事件，可以通过 `feedback->menu_entry_id` 获取到菜单的标识id。

## 按钮
按钮的创建只需要设置相应图形的交互模式 `interaction_mode` 为 `BUTTON`, 
```cpp
control.interaction_mode = InteractiveMarkerControl::BUTTON;
```
然后就可以在相应图形点击的后出发 `InteractiveMarkerFeedback::BUTTON_CLICK` 事件。

## 将图形添加到移动的框架中
在 rviz 中可以存在多个框架，当时同时只能设置一个固定框架， 为了让其他框架中的图形同时能够在固定框架中展示，我们需要设置相应框架到固定框架的坐标转换规则，在 `rviz` 中的坐标转换是基于 `tf` 包， 在 `tf` 中，坐标转换规则可以通过 `tf::TransformBroadcaster` 通知到 ros 系统中， 下面的代码配置了在 z 方向上相对与 `base_link` 运动的框架 `moving_frame` 以及相对于 `base_link` 围绕者 y 轴旋转的框架 `rotating_frame`.

```cpp
void frameCallback(const ros::TimerEvent &event) {
  static u_int32_t counter;
  static tf::TransformBroadcaster br;

  tf::Transform t;
  ros::Time now = ros::Time::now();
  // 设置平移框架的坐标转换
  t.setOrigin(tf::Vector3(0.0, 0.0, 2.0 * sin(counter / 140.0)));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, now, "base_link", "moving_frame"));
  // 设置旋转的框架 rotating_frame 的相对与固定框架的坐标转换
  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, counter / 140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, now, "base_link", "rotating_frame"));

  ++counter;
}
```

为了将相应的图形添加到对应的框架中，我们需要设置其 `frame_id` 为相应的框架名称，
```cpp
int_marker.header.frame_id = "rotating_frame";
```
并且我们还需要将交互图形的 `header` 中的 `stamp` 设置为 `ros::Timer(0)`（默认值），这一 rviz 才会实时刷新图形的位置。