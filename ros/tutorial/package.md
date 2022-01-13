# 创建 catkin 工作空间

例如，我们要在用户主目录下创建一个文件夹`catkin_ws`作为 catkin 　的工作目录，执行如下命令

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_build
```

执行完 `catkin_build`　后会在 src 文件夹下生成一个 CMakeLists.txt 　文件，作为工作空间的主　 cmake 文件。此外,　执行完 `catkin_build` 命令后还会在工作空间目录下生成 build 和 devel 目录，其中 build 目录为 cmake 的二进制目录，用于生成对应的二进制文件；devel 目录存放一些工作空间的环境相关的脚本以及编译完成后的库和可执行程序，例如执行

```shell
source ~/catkin_ws/devel/setup.bash
```

脚本后，工作空间所在目录会被添加到 ROS 的包搜索路径中,如下所示

```shell
$ echo $ROS_PACKAGE_PATH
/home/[username]/catkin_ws/src:/opt/ros/melodic/share
```

# 创建一个 catkin 包

`catkin_create_pkg`命令用于创建一个 catkin 包。进入 catkin 工作空间下的 src 目录,并创建 catkin 包 `beginer_tutorials`,如下

```shell
cd　~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

其中，std_msgs rospy 　和 roscpp 　为当前报的依赖包。命令执行完成后，在 src 目录下会多出一个 `beginner_tutorials`　目录,在该目录下分别有

1. package.xml 用于提供报的元信息
2. CMakeLists.txt 包的子 cmake 　文件

# 包元信息文件 package.xml 介绍

package 用于描述 ros 包的基本信息：

1. description 标签 -- description 标签用于对包进行描述。
2. maintainer 标签 -- maintainer 标签用于告诉其他人包的维护者的相关信息，以及维护者的联系方式（邮件）。至少需要添加一个维护者，也可以添加通过多个 maintainer 标签添加多个维护者。
3. license 标签 -- 用于描述包的许可证。
4. 包依赖标签 -- 包括构建依赖(build_depend)、构建工具依赖(buildtool_depend),运行时依赖(exec_depend)、测试依赖(test_depend)、导出（即依赖于该包时，同时也应该依赖于 build_export_depend 标签内定义的包 12346）的依赖包(build_export_depend)、文档依赖(doc_depend)

# 包的构建

1. 进入 catkin 工作空间目录

```shell
cd　~/catkin_ws
```

2. 执行`catkin_make`命令

```shell
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws$ catkin_make
Base path: /home/hybtalented/rpi-tools/ros_study/catkin_ws
Source space: /home/hybtalented/rpi-tools/ros_study/catkin_ws/src
Build space: /home/hybtalented/rpi-tools/ros_study/catkin_ws/build
Devel space: /home/hybtalented/rpi-tools/ros_study/catkin_ws/devel
Install space: /home/hybtalented/rpi-tools/ros_study/catkin_ws/install
####
#### Running command: "cmake /home/hybtalented/rpi-tools/ros_study/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/hybtalented/rpi-tools/ros_study/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/hybtalented/rpi-tools/ros_study/catkin_ws/install -G Unix Makefiles" in "/home/hybtalented/rpi-tools/ros_study/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/hybtalented/rpi-tools/ros_study/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/hybtalented/rpi-tools/ros_study/catkin_ws/devel;/opt/ros/melodic
-- This workspace overlays: /home/hybtalented/rpi-tools/ros_study/catkin_ws/devel;/opt/ros/melodic
-- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
-- Using PYTHON_EXECUTABLE: /usr/bin/python2
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/hybtalented/rpi-tools/ros_study/catkin_ws/build/test_results
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.29
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'beginner_tutorials'
-- ==> add_subdirectory(beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /home/hybtalented/rpi-tools/ros_study/catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/hybtalented/rpi-tools/ros_study/catkin_ws/build"
####
```

**注意: 上述的 shell 命令中 catkin_ws 实际上并不在 home 目录下。**

从上面的输出结果可以看到，执行`catkin_make`实际上可以分为两个步骤。首先执行的是 cmake 命令，负责生成 Makefile 文件，命令输出的最开始为编译过程中的相关路径的配置，以及实际上的 cmake 的完整命令和 cmake 的输出；随后执行的是 make 命令，完成项目的编译。

# 编写 ros 包

## 编辑 ros 包代码

使用 `rosed` 命令可以快速的编辑包下的源代码，例如可以通过一下命令编辑`roscpp`包中的`Logger.msg`文件

```shell
rosed roscpp Logger.msg
```

默认情况下执行`rosed`会打开`vim`对文件进行编辑，可以修改环境变量`EDITOR`更改默认的编辑器，例如通过如下环境变量配置可以把默认编辑改为 `gedit`

```shell
export EDITOR='gedit
```

`rosed` 会自动遍历搜索对应包的源代码路径下的所有文件，在完成包名的输入后，按两下 `Tab` 键可以列出所有包里的所有文件，如下所示

```shell
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials/launch$ rosed roscpp
Empty.srv                   roscpp.cmake
genmsg_cpp.py               roscppConfig.cmake
gensrv_cpp.py               roscppConfig-version.cmake
GetLoggers.srv              roscpp-msg-extras.cmake
Logger.msg                  roscpp-msg-paths.cmake
msg_gen.py                  SetLoggerLevel.srv
package.xml
```

同时在输入部分文件名后，也可以通过 `Tab` 键自动补全。

**注意： 如果有多个文件的文件名重复，在调用 `rosed` 指定该文件名时，会弹出一个菜单让我们可以选择具体的文件。**

# ros 消息及其发布订阅的写法

## 编写 ros 消息定义文件

ros 消息定义文件是一个描述 ros 消息字段的文件，该文件可以用于生成各种语言的消息源代码。

消息定义文件放在包源代码路径下的 `msg` 目录下，他是一个文本文件，文件中的每一行用于描述消息的字段类型和字段名称。

ros 中的字段类型包括

- 数字类型: int8, int16, int32, int64 (以及指针类型 uint\*)
- 浮点数类型: float32, float64
- 字符串类型: string
- 时间类型: time, duration
- 其他消息定义文件定义的类型
- 数组类型: 不定长数组 array[] 和 定长数组 array[C]

下面代码在 `beginner_tutorials` 包中创建了一个名叫 Num.msg 的消息定义文件

```shell
roscd beginner_tutorials/
mkdir msg
echo 'int64 num' > msg/Num.msg
```

上述的例子中定义了只有一个 `int64` 类型的数字字段 `num`。

为了能够让对应的包自动生成消息源代码并在自动包含消息的运行时，
我们还在要在 `beginner_tutorials` 下的 `package.xml`中添加下述两个依赖包

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

然后修改包的`CMakeLists.txt`文件, 让 cmake 能够自动生成消息定义的源代码

1. 在 find_package 函数中添加 `message_generation` 依赖, 这个依赖让我们可以使用消息生成相关的 cmake api

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

2. 在 `message_runtime` 包添加到运行时依赖中, catkin_package 用于生成 cmake 配置文件, 其他包通过调用 find_package 依赖该包时做如下操作

- INCLUDE_DIRS: 将对应目录添加到包含目录中
- LIBRARIES: 将对应的库添加到依赖库中
- CATKIN_DEPENDS: 将对应的 ros 包添加到 ros 依赖包中
- DEPENDS: 将系统依赖性添加到依赖项中

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
```

3. 添加消息定义文件

```cmake
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
)
```

4. 最后一步, 设置自动生成消息

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

完成以上操作以后,我们可以就通过 rosmsg 命令查看消息了

```shell
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ rosmsg show beginner_tutorials/Num
int64 num
```

完成 CMake 文件的修改以后,下一步可以使用 `catkin_make`命令生成消息代码相应的源文件

```shell
# 回到 catkin 工作空间根目录
cd ../../
# 编译源代码
catkin_make
# 回到 begin_tutorials 目录
cd -
```

自动生成的消息源代码存放在 `devel` 目录下, 其中 c++ 头文件生成在 `devel/include/beginner_tutorials/`, python 脚本文件创建在 `devel/lib/python2.7/dist-packages/beginner_tutorials/msg` 目录下, lisp 文件创建在 `devel/share/common-lisp/ros/beginner_tutorials/msg/` 目录下, nodejs 文件创建在 `devel/share/gennodejs/ros/beginner_tutorials/msg` 目录下.

## ros 节点发布消息

进入 `beginner_tutorials` 目录,

```shell
roscd beginner_tutorials
```

并在 `src` 目录下创建一个 `talker.cpp` 文件, 开始编写一个节点来发布消息

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   *
   * ros 节点是通过 ros::init 函数进行初始化的,
   * 初始化参数可以这里一样通过命令行闯入, 也可以通过该函数的重载通过程序来配置.
   * 最后一个参数 "talker" 指定了节点的名字.
   */
  ros::init(argc, argv, "talker");
  /**
   * ros 节点通过 ros::NodeHandle 与 ros 系统交互, 在一个进程中可以多次构造
   * ros::NodeHandle, 但是只有第一次构造会将ros节点注册到ros系统中, 而最后一个
   * ros::NodeHandle 的销毁或导致节点的关闭.
   */
  ros::NodeHandle handle;

  /**
   * 通过 NodeHandle::advertise 可以通知 ros 的 master
   * 节点该节点将发布指定主题的消息, 而 master 节点会通知
   * 所有订阅了该主题的其他节点与该节点协商建立一个点对点的连接. advertise
   * 方法会返回一个发布者对象, 通过该对象可以 发布对应主题的消息,
   * 当该对象以及所有该对象的拷贝销毁后, 将会通知 master
   * 节点该节点不在发布对应主题的消息.
   *
   * advertise 方法的第二个参数指定了消息发布队列的大小,
   * 当待发送消息的数量大于消息发送的速度时, 未发送的消息将缓存在 消息队列中,
   * 而当消息队列中的消息到达这个大小后, 后续的消息将会被丢弃.
   */
  ros::Publisher chatter_pub =
      handle.advertise<std_msgs::String>("chatter", 1000);

  /**
   * 通过 loop_rate 可以控制循环的调用频率
   */
  ros::Rate loop_rate(10);
  int count = 0;
  /**
   * ok 方法的返回值将一直为true 知道 ros::showdown 方法被调用
   */
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "hello world " << count++;
    msg.data = ss.str();

    ROS_INFO("publishing %s", msg.data.c_str());

    /**
     * 发布一个消息, 发布的消息类型必须要与 advertise 创建是指定的模板参数一致.
     */
    chatter_pub.publish(msg);

    /**
     * 完成一次 ros 消息循环
     */
    ros::spinOnce();
    /**
     * 等待剩余的时间后再次运行循环
     */
    loop_rate.sleep();
  }
  ROS_INFO("ros chatter node is shutdown");
  return 0;
}
```

编写玩`talker.cpp`代码以后 beginner_tutorials 的 `CMakeLists.txt` 中, 配置 `talker` 节点的编译

```cmake
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
```

上述配置中 add_dependencies 告诉 cmake 需要先编译完目标 beginner_tutorials_generate_messages_cpp 后才能编译 `talker` 节点, 其中 `beginner_tutorials_generate_messages_cpp` 为 catkin 自动生成的用于消息定义文件和服务定义文件对应的相关头文件.

然后就可以通过 `catkin_make` 命令编译 `talker` 节点了

```shell
roscd beginner_tutorials
cd ../../
catkin_make
```

生成的二进制文件可以在 `devel/beginner_tutorials/lib` 目录下找到.

## ros 节点订阅消息

这一小节, 我们将会编写一个 ros 节点实现对 ros 主题的订阅, 在 `beginner_tutorials` 的 `src` 目录小创建一个 `listener.cpp` 文件, 并在文件内输入入校的代码

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * 订阅的 ROS chatter 消息处理函数
 */
void chatterCallback(const std_msgs::StringConstPtr &msg) {
    ROS_INFO("I hear: [%s]", msg->data.c_str());
}
int main(int argc, char **argv) {
  /**
   * 初始化ros节点
   */
  ros::init(argc, argv, "listener");

  /**
   * 创建一个节点句柄, 实现与ros系统的交互
   */
  ros::NodeHandle handle;

  /**
   * 订阅对应的 ros 主题
   *
   * 通知 ros master 节点该节点订阅的相应的主题, 并自动连接到对应主题的发布者上.
   * 当发布者发布该主题的消息 后, 将会调用回调函数 chatterCallback. subscribe
   * 方法返回一个订阅者实例, 节点将会一直订阅该主题,
   * 直到该订阅者实例及其所有的拷贝被销毁.
   *
   * subscribe 的第二个参数表示消息接收队列的大小.
   * 当消息的接收速度大于处理的速度时, 接收到的消息将会缓冲到 消息接收队列中,
   * 当消息接收队列满了以后, 后续的消息将会被丢弃.
   */
  ros::Subscriber sub = handle.subscribe("chatter", 1000, chatterCallback);
  /**
   * 进入ros的消息循环
   *
   * ros 消息循环将会处理各种消息回调, 当没有任何回调消息循环将会阻塞, 因此不会
   * 大量占用cpu. 消息循环会一直运行知道 ros::ok 返回 false, 也就是是说下列条件
   * 中的一个或多个达成
   * 1. 收到 SIGINT 信号(如用户按下了 Ctrl+C)
   * 2. 应用程序调用了 ros::shutdown 方法
   * 3. 所有的 NodeHandle 实例都被销毁了
   * 4. 另一个同名的节点启动了
   *
   * 需要注意的是它不会处理自定义队列中的回调.
   */
  ros::spin();
  return 0;
}
```

然后在 `beginner_tutorials` 的 `CMakelists.txt` 添加节点的生成目标.

```cmake
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

然后执行 `catkin_make` 命令后, 将会在 `devel/lib/beginner_tutorials` 目录生成 listener 的目标文件.

## 运行 talker 和 listener 节点

首先运行 talker 节点,其结果如下所示

```shell
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials talker
[ INFO] [1641906339.355230121]: publishing hello world 0
[ INFO] [1641906339.455316689]: publishing hello world 1
[ INFO] [1641906339.555276639]: publishing hello world 2
[ INFO] [1641906339.655316747]: publishing hello world 3
[ INFO] [1641906339.755277087]: publishing hello world 4
[ INFO] [1641906339.855319454]: publishing hello world 5
[ INFO] [1641906339.955314323]: publishing hello world 6
[ INFO] [1641906340.055317790]: publishing hello world 7
[ INFO] [1641906340.155296715]: publishing hello world 8
[ INFO] [1641906340.255296652]: publishing hello world 9
[ INFO] [1641906340.355292371]: publishing hello world 10
[ INFO] [1641906340.455284093]: publishing hello world 11
[ INFO] [1641906340.555281433]: publishing hello world 12
[ INFO] [1641906340.655320127]: publishing hello world 13
[ INFO] [1641906340.755274287]: publishing hello world 14
[ INFO] [1641906340.855289592]: publishing hello world 15
```

如果运行命令后报如下所示的错误

```shell
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials talker
[rospack] Error: package 'beginner_tutorials' not found
```

这时需要在终端内加载对应工作空间的环境配置脚本

```shell
source ~/rpi-tools/ros_study/catkin_ws/devel/setup.sh
```

随后运行 listener 节点

```shell
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials listene[ INFO] [1641906343.456092486]: I hear: [hello world 41]
[ INFO] [1641906343.555699447]: I hear: [hello world 42]
[ INFO] [1641906343.655894142]: I hear: [hello world 43]
[ INFO] [1641906343.755663323]: I hear: [hello world 44]
[ INFO] [1641906343.855644832]: I hear: [hello world 45]
[ INFO] [1641906343.956037170]: I hear: [hello world 46]
[ INFO] [1641906344.055899031]: I hear: [hello world 47]
[ INFO] [1641906344.155718945]: I hear: [hello world 48]
[ INFO] [1641906344.255980220]: I hear: [hello world 49]
[ INFO] [1641906344.355700490]: I hear: [hello world 50]
[ INFO] [1641906344.455738236]: I hear: [hello world 51]
[ INFO] [1641906344.555537344]: I hear: [hello world 52]
[ INFO] [1641906344.655636059]: I hear: [hello world 53]
[ INFO] [1641906344.755719077]: I hear: [hello world 54]
[ INFO] [1641906344.856080192]: I hear: [hello world 55]
[ INFO] [1641906344.955868093]: I hear: [hello world 56]
[ INFO] [1641906345.055791361]: I hear: [hello world 57]
[ INFO] [1641906345.155701891]: I hear: [hello world 58]
```

# ros 服务以及服务的调用的写法

## 编写 ros 服务定义文件

ros 服务定义文件用于描述一个服务的请求以及响应, 它的文件结构与消息定义文件类似, 只不过文件中包含了 请求和响应的两种类型定义,两种类型定义之间用 `---`分隔.

下面我们将编写并编译一个 ros 服务定义文件.

首先进入 `beginner_tutorials` 的源代码目录, 并创建 `srv` 文件夹

```shell
roscd beginner_tutorials
mkdir srv
```

我们从 `rospy_tutorials` 中拷贝 `AddTwoInts.srv` 服务定义文件到 `srv` 目录中

```shell
roscp rospy_tutorials  AddTwoInts.srv srv/
```

`AddTwoInts.srv` 中的内容如下所示. 该服务定义文件中定义了一个请求类型,该类型中包括 2 个 6 位整形数 `a`, `b`; 在分隔符`---` 后又定义了一个相应类型为整形的`sum`.

```shell
roscat beginner_tutorials AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

同 消息定义文件 一样, 服务定义文件也需要在 package.xml 中添加`message_generation` 和 `message_runtime` 两个依赖

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

然后我需要修改 `beginner_tutorials` 包的 `CMakeLists.txt` 文件中的配置

1. 添加 `message_generation` 依赖

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

2. 添加服务文件

```cmake
## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)
```

接着我们就可以通过 `rossrv show` 命令查看该服务文件的内容

```
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ rossrv show beginner_tutorials/AddTwoInts
int64 a
int64 b
---
int64 sum
```

最后, 类似于消息定义文件一样生成相应语言的源代码

```shell
cd beginner_tutorials
cd ../../
catkin_make
cd -
```

自动生成的服务源代码存放在 `devel` 目录下, 其中 c++ 头文件生成在 `devel/include/beginner_tutorials/`, python 脚本文件创建在 `devel/lib/python2.7/dist-packages/beginner_tutorials/srv` 目录下, lisp 文件创建在 `devel/share/common-lisp/ros/beginner_tutorials/srv` 目录下, nodejs 源代码文件创建在 `devel/share/gennodejs/ros/beginner_tutorials/srv` 目录下.

## ros 服务的编写

首先进入 `beginner_tutorials` 的根目录, 并去确认上一小节创建的 `AddTwoInts.srv` 服务定义文件的内容

```shell
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ roscd beginner_tutorials/
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ roscat beginner_tutorials AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

在 `src` 目录下创建 `add_two_ints_server.cpp ` 文件, 并输入如下内容

```cpp
#include <beginner_tutorials/AddTwoInts.h>
#include <ros/ros.h>
/**
 * @brief 服务处理函数, sum = a + b
 * @param req 请求参数
 * @param res 返回参数
 *
 * @returns 服务需要返回 true 通知服务调用成功
 */
bool add(beginner_tutorials::AddTwoIntsRequest &req,
         beginner_tutorials::AddTwoIntsResponse &res) {
  res.sum = req.a + req.b;
  ROS_INFO("add_two_ints: %ld + %ld, response %ld", req.a, req.b, res.sum);
  return true;
}

int main(int argc, char **argv) {
  // 初始化 ros 节点
  ros::init(argc, argv, "add_two_ints_server");
  // 创建一个 ros 节点句柄
  ros::NodeHandle handle;
  /**
   * 创建一个服务提供者对象
   *
   * 通过 NodeHandle::advertiseService 向 ros 的 master 节点 注册一个 名称为 add_two_ints 的 服务,
   * add 函数作为服务的处理函数. advertiseService 返回一个 服务提供者实例, 在所有服务提供者实例及其拷贝
   * 被销毁后, 相应的服务将会从 ros master 中注销.
   */
  ros::ServiceServer server = handle.advertiseService("add_two_ints", add);

  ROS_INFO("ready to serve add_two_ints service");
  // 进入 ros 的消息循环
  ros::spin();
  return 0;
}
```

然后在 `beginner_tutorials` 的 `CMakeLists.txt` 文件中添加如下内容, 将 `add_two_ints_server` 节点加入到生成目标中

```cmake
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_generate_messages_cpp)
```

执行 `catkin_make` 后接口完成服务提供者节点 `add_two_ints_server` 的生成.

## ros 服务的调用

在这个小节, 我们将创建一个 ros 服务调用的客户端. 首先, 在 `beginner_tutorials` 的 `src` 目录下创建一个 `add_two_ints_client.cpp` 作为 `add_two_ints_client` 节点的源文件, 在 `add_two_ints_client` 节点内将会创建一个 `add_two_ints` 服务的客户端并调用相应的服务, 在 `add_two_ints_client.cpp` 键入如下内容

```cpp
#include <beginner_tutorials/AddTwoInts.h>
#include <ros/ros.h>

#include <cstdlib>
int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   */
  ros::init(argc, argv, "add_two_ints_client");
  /**
   * 创建 ros 节点句柄
   */
  ros::NodeHandle handle;

  /**
   * 创建 ros 客户端
   *
   * 创建一个客户端用于调用 add_two_ints 服务,
   * 可以指定第二个参数以提高服务的调用效率, 以及第三个参数
   * 指定请求连接的连接握手时的请求头.
   *
   * serviceClient 方法返回一个服务调用客户端, 通过该客户端可以调用服务.
   */
  ros::ServiceClient client =
      handle.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  /**
   * 创建一个服务对象, 用于传递请求以及获取返回值
   */
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv)) {
    // 服务调用成功
    ROS_INFO("sum is %ld", srv.response.sum);
    return 0;
  } else {
    // 服务调用失败
    ROS_ERROR("Failed to call service add_two_ints");
    return -1;
  }
}
```

然后打开 `beginner_tutorials` 的 `CMakeLists.txt` 文件, 并添加如下代码

```cmake
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_generate_messages_cpp)
```

最后执行 `catkin_make` 命令即可完成 `add_two_ints_client` 节点的编译.

## ros 服务的服务端和客户端的测试

首先运行 `add_two_ints_server` 节点后, 在执行两次
`add_two_ints_client` 节点, 在客户端节点得到如下所示的输出

```shell
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials add_two_ints_client 1 2
[ INFO] [1642088485.536044577]: sum is 3
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials add_two_ints_client 20 23
[ INFO] [1642088496.718600770]: sum is 43
```

而在服务端节点的输出为

```shell
rosrun beginner_tutorials add_two_ints_server 
[ INFO] [1642088478.801954731]: ready to serve add_two_ints service
[ INFO] [1642088485.535826215]: add_two_ints: 1 + 2, response 3
[ INFO] [1642088496.718394770]: add_two_ints: 20 + 23, response 43
```

而如果在启动 `add_two_ints_client` 节点时, `add_two_ints_server` 节点还未启动, 则会得到一条报错信息.

```shell
hybtalented@hybtaletented-163-com:~/study$ rosrun beginner_tutorials add_two_ints_client 20 23
[ERROR] [1642088743.477537802]: Failed to call service add_two_ints
```