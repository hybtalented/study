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
   * 方法返回一个订阅者实例, 节点将会一直订阅该主题, 直到该订阅者实例及其所有的拷贝被销毁.
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

# ros2 动作以及动作客户端和服务端的编写
## ros2 动作定义文件
ros2 的动作定义文件用于描述行为的目标，反馈和结果，首先进入 beginner_tutorial 的包目录，创建 action 目录, 并在 action 目录下创建一个 `Fibonacci.action` 动作定义文件，并在里面输入如下的内容
```action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

动作定义文件通过 `---` 可分为3个部分。第一个部分为目标定义，其中 `order` 为创建的斐波那契序列的阶数；第二个部分为动作结果，其中 `sequence` 为对应的斐波那契序列；第三个部分为动作的反馈，其中 `partial_sequence` 为部分的斐波那契序列。

在 cmake 文件中 通过 `rosidl_generate_interfaces` 宏，从而将动作定义文件编译为对应语言的源文件
```shell
set(ros_actions action/Fibonacci.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${ros_msgs} ${ros_srvs} ${ros_actions})
```
自动生成的动作源代码存放在 `install` 目录下, 其中 c++ 头文件生成在 `install/beginner_tutorials/include/action/`, python 脚本文件创建在 `install/beginner_tutorials/local/lib/python3.10/dist-packages/beginner_tutorials/action/` 目录下.

## ros2 动作服务器的编写
在 `beginner_tutorials` 包的 `src` 目录下创建 `action_server.cpp` 文件，并输入如下内容

```cpp
/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-06
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-15
 * @FilePath /src/beginner_tutorials/src/action_server.cpp
 * @Description
 *
 * @Example
 */
#include <functional>
#include <memory>
#include <thread>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <beginner_tutorials/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class FibonacciActionServer : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using FibonacciGoalHandleSharedPtr = const std::shared_ptr<FibonacciGoalHandle>;
  BEGINNER_TUTORIALS_EXPORT explicit  FibonacciActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("fibonacci_action_server", options) {
    using namespace std::placeholders;
    /**/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-16
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }东
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);
    _action_server = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci",
        std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
        std::bind(&FibonacciActionServer::handle_cancel, this, _1),
        std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr _action_server;
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              Fibonacci::Goal::ConstSharedPtr goal) {
    Radd_library(action_server SHARED src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components)
target_compile_definitions(action_server PRIVATE ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL)
target_link_libraries(action_server ${cpp_typesupport_target})
target_include_directo/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-16
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }东
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);0) {
      // 接受并且执行目标
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 拒绝执行目标
    return rclcpp_action::GoalResponse::REJECT;
  }
  void handle_accepted(FibonacciGoalHandleSharedPtr goal_handle) {/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-16
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }东
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);行动作
    std::thread thread(std::bind(&FibonacciActionServer::execute, this, goal_handle));
    thread.detach();
  }

  rclcpp_action::CancelResponse
  handle_cancel(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    RCL_UNUSED(goal_handle);
    // 服务器以及取消了对应的动作
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void execute(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Execute goal");
    // 创建一个定时器，频率为 1Hz
    rclcpp::Rate loop_rate(1);
    const Fibonacci::Goal::ConstSharedPtr goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    // 初始化斐波那契序列
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<Fibonacci::Result>();
    /**
     * 迭代计算序列，如果对应的ros 上下文以及退出（即调用了 shutdown)， 循环也将会将会结束
     */
    for (int i = 1; i < goal->order && rclcpp::ok(); ++i) {
      if (goal_handle->i/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-16
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }东
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);
      }
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // 发布反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish feedback");
      // 等待一秒后继续迭代
      loop_rate.sleep();
    }
    if (rclcpp::ok()) {
      result->sequence = sequence;
      // 通知客户端动作以及成功完成
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeed");
    }
  }
};
// 将节点注册为一个ROS组件
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionServer);
```
然后在 `beginner_tutorials` 包的 `CMakeLists.txt` 文件中添加如下代码

```cmake
add_library(action_server SHARED src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components)
# 添加 ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL 宏控制动态库接口可见性
target_compile_definitions(action_server PRIVATE ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL)
# 添加本包内的相关接口引用
target_link_libraries(action_server ${cpp_typesupport_target})
target_include_directories(action_server PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
  $<INSTALL_INTERFACE:include>
)
# 注册组件方式的节点，PLUGIN 后指定节点的名词， 如果还指定了 EXECUTABLE，同时将会生成一个可执行程序，
# 从而节点也可以通过 ros2 run 的方式启动
rclcpp_components_register_node(action_server PLUGIN FibonacciActionServer EXECUTABLE fibonacci_action_server)
```
运行
```shell
colcon build
```
即可完成ros2 动作服务器的编译，在 `install/beginner_tutorials/lib/` 目录下将会生成 ros2 组件 `libaction_service.so`, 而在 `install/beginner_tutorials/lib/beginner_tutorials` 下将生成对于的可执行程序 `fibonacci_action_server`.

## ros2 动作服务器客户端的编写
在 `beginner_tutorials` 的 `src` 目录下创建 `action_client.cpp`文件，并输入如下内容

```cpp
/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-16
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }东
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);
```
然后在对应的 `CMakeLists.txt` 文件中添加
```cmake
add_library(action_client SHARED src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action rclcpp_components)
target_include_directories(action_client PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client PRIVATE ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL)
target_link_libraries(action_client ${cpp_typesupport_target})
```

最后，运行
```shell
colcon build
```
完成客户端的编译

## 测试动作的调用
在这一小节，我们将编写一个 `launch` 文件, 通过组件的方式启动动作客户端和服务端测试动作的调用。

首先，在 `beginner_tutorial` 的 `launch` 目录下创建 `action_launch.py`, 并输入如下内容
```python
'''
Author Youbiao He hybtalented@163.com
Date 2022-07-08
LastEditors Youbiao He
LastEditTime 2022-07-15
FilePath /src/beginner_tutorials/launch/action_launch.py
Description 

Example 
'''
from struct import pack
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 创建一个组建的宿主节点
    container = ComposableNodeContainer(
        name="fibonacci",
        namespace="action_test",
        package='rclcpp_components',
        executable="component_container",
        composable_node_descriptions=[
            # 创建组件节点实例
            ComposableNode(name="fibonacci_server",
                           package="beginner_tutorials",
                           plugin="FibonacciActionServer"),
            ComposableNode(package="beginner_tutorials",
                           name="fibonacci_client", plugin="FibonacciActionClient")
        ],
        output='screen')
    return launch.LaunchDescription([container])

```
在上面的我们启动了 `rclcpp_components` 包中的 `component_container` 节点作为组件节点的宿主，并通过 `ComposableNode` 指定宿主启动了 `FibonacciActionClient` 和 `FibonacciActionServer` 两个节点。

在对应的终端中通过 `ros2 launch` 启动动作服务端节点和动作客户端节点：
```shell
hybtalented@hybtalented-Ubuntu:~/rpi-tools/ros_study/ros2_study_ws$ ros2 launch beginner_tutorials action_launch.py 
[INFO] [launch]: All log files can be found below /home/hybtalented/.ros/log/2022-08-16-15-53-55-843439-hybtalented-Ubuntu-35699
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container-1]: process started with pid [35712]
[component_container-1] [INFO] [1660636436.214890814] [action_test.fibonacci]: Load Library: /home/hybtalented/rpi-tools/ros_study/ros2_study_ws/install/beginner_tutorials/lib/libaction_server.so
[component_container-1] [INFO] [1660636436.216307734] [action_test.fibonacci]: Found class: rclcpp_components::NodeFactoryTemplate<FibonacciActionServer>
[component_container-1] [INFO] [1660636436.216339512] [action_test.fibonacci]: Instantiate class: rclcpp_components::NodeFactoryTemplate<FibonacciActionServer>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/fibonacci_server' in container '/action_test/fibonacci'
[component_container-1] [INFO] [1660636436.226114898] [action_test.fibonacci]: Load Library: /home/hybtalented/rpi-tools/ros_study/ros2_study_ws/install/beginner_tutorials/lib/libaction_client.so
[component_container-1] [INFO] [1660636436.226493164] [action_test.fibonacci]: Found class: rclcpp_components::NodeFactoryTemplate<FibonacciActionClient>
[component_container-1] [INFO] [1660636436.226501326] [action_test.fibonacci]: Instantiate class: rclcpp_components::NodeFactoryTemplate<FibonacciActionClient>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/fibonacci_client' in container '/action_test/fibonacci'
[component_container-1] [INFO] [1660636436.734215986] [fibonacci_client]: Send goal
[component_container-1] [INFO] [1660636436.735685577] [fibonacci_server]: Received goal request with order 10
[component_container-1] [INFO] [1660636436.736859335] [fibonacci_server]: Execute goal
[component_container-1] [INFO] [1660636436.737353667] [fibonacci_client]: Goal accepted by the server, wait for result ...
[component_container-1] [INFO] [1660636436.737585311] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636436.738021163] [fibonacci_client]: Next number in sequence received: 0 1 1 
[component_container-1] [INFO] [1660636437.737461454] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636437.737724821] [fibonacci_client]: Next number in sequence received: 0 1 1 2 
[component_container-1] [INFO] [1660636438.737487090] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636438.737801076] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 
[component_container-1] [INFO] [1660636439.737281124] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636439.737443355] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 
[component_container-1] [INFO] [1660636440.737519493] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636440.737839459] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 8 
[component_container-1] [INFO] [1660636441.737485966] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636441.737782127] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 
[component_container-1] [INFO] [1660636442.737508813] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636442.737811440] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 
[component_container-1] [INFO] [1660636443.737501923] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636443.737852750] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 
[component_container-1] [INFO] [1660636444.737607705] [fibonacci_server]: Publish feedback
[component_container-1] [INFO] [1660636444.737978597] [fibonacci_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55 
[component_container-1] [INFO] [1660636445.738107232] [fibonacci_server]: Goal succeed
[component_container-1] [INFO] [1660636445.738861493] [fibonacci_client]: result received: 0 1 1 2 3 5 8 13 21 34 55 
[INFO] [component_container-1]: process has finished cleanly [pid 35712]
```
# roswtf 的使用
roswtf 可以用于检测ros系统或者ros包中的错误.

首先, 我们先关闭 `roscore`, 然后进入 `beginner_tutorials` 包的根目录, 并执行 `roswtf` 可以得到如下所示的命令行输出
```
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ roswtf
Loaded plugin tf.tfwtf
Package: beginner_tutorials
================================================================================
Static checks summary:

No errors or warnings
================================================================================

ROS Master does not appear to be running.
Online graph checks will not be run.
ROS_MASTER_URI is [http://localhost:11311]
```

然后, 我们退出 `beginer_tutorials` 并启动 `roscore` 节点, 然后执行 `roswtf` 可以得到如下所示的输出

```shell
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/src/beginner_tutorials$ roscd 
hybtalented@hybtaletented-163-com:~/rpi-tools/ros_study/catkin_ws/devel$ roswtf
Loaded plugin tf.tfwtf
No package or stack in the current directory
================================================================================
Static checks summary:

No errors or warnings
================================================================================
Beginning tests of your ROS graph. These may take a while...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules

Online checks summary:

Found 1 warning(s).
Warnings are things that may be just fine, but are sometimes at fault

WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```

最后, 我们将对 `roswtf` 的输出结果进行分析, 通过 `roscd` 命令进入 `rosmaster` 包的根目录, 然后执行｀roswtf` 得到如下结果
```shell
hybtalented@hybtaletented-163-com:~/study/ros/tutorial/temporary$ roscd rosmaster
hybtalented@hybtaletented-163-com:/opt/ros/melodic/share/rosmaster$ roswtf
Loaded plugin tf.tfwtf
Package: rosmaster
================================================================================
Static checks summary:

No errors or warnings
================================================================================
Beginning tests of your ROS graph. These may take a while...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules

Online checks summary:

Found 1 warning(s).
Warnings are things that may be just fine, but are sometimes at fault

WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```
上述的 `roswtf` 输出中
1. Package: rosmaster: 如果当前所在目录在`ROS_PACKAGE_PATH` 环境变量指定的工作空间的一个包中, 告诉我们当前所在目录对应的包的名称. 

2. Static checks summary: 这段输出告诉我们对应的包中以及 ros 系统中没有任何静态的错误. 其中静态的错误主要包括文件系统以及一些非运行时的问题.

3. Online checks summary: 告诉我们检测整个ros 系统中的运行时错误, 在这段输出中告诉我们 `rosmaster` 的 `/rosout` 主题没有任何订阅者.

