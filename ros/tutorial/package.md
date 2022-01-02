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

# 编写 ros 消息文件

ros 消息文件是一个描述 ros 消息字段的文件，该文件可以用于生成各种语言的消息源代码。

消息文件放在包源代码路径下的 `msg` 目录下，他是一个文本文件，文件中的每一行用于描述消息的字段类型和字段名称。

ros 中的字段类型包括

- 数字类型: int8, int16, int32, int64 (以及指针类型 uint\*)
- 浮点数类型: float32, float64
- 字符串类型: string
- 时间类型: time, duration
- 其他消息文件定义的类型
- 数组类型: 不定长数组 array[] 和 定长数组 array[C]

下面代码在 `beginner_tutorials` 包中创建了一个名叫 Num.msg 的消息文件

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

然后修改包的`CMakeLists.txt`文件, 让 cmake 能够自动生成消息文件源代码
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
## LIBRARIES: 将对应的库添加到依赖库中
## CATKIN_DEPENDS: 将对应的ros包添加到ros依赖包中
## DEPENDS: 将系统依赖性添加到依赖项中
```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)
```

3. 添加消息文件

```cmake
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

完成CMake文件的修改以后,下一步可以使用 `catkin_make`命令生成消息代码相应的源文件
```shell
# 回到 catkin 工作空间根目录
cd ../../
# 编译源代码
catkin_make 
# 回到 begin_tutorials 目录
cd - 
```

自动生成的消息代码文件存放在 `devel` 目录下, 其中 c++ 头文件生成在 `devel/include/beginner_tutorials/`, python 脚本文件创建在 `devel/lib/python2.7/dist-packages/beginner_tutorials/msg` 目录下, lisp 文件创建在 `devel/share/common-lisp/ros/beginner_tutorials/msg/` 目录下, nodejs 文件创建在 `devel/share/gennodejs/ros/beginner_tutorials/msg` 目录下.
