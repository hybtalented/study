# 创建catkin 工作空间
例如，我们要在用户主目录下创建一个文件夹`catkin_ws`作为 catkin　的工作目录，执行如下命令
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_build
```
执行完 `catkin_build`　后会在src文件夹下生成一个 CMakeLists.txt　文件，作为工作空间的主　cmake 文件。此外,　执行完 `catkin_build` 命令后还会在工作空间目录下生成build和devel目录，其中build目录为cmake的二进制目录，用于生成对应的二进制文件；devel目录存放一些工作空间的环境相关的脚本以及编译完成后的库和可执行程序，例如执行
```shell
source ~/catkin_ws/devel/setup.bash
```
脚本后，工作空间所在目录会被添加到 ROS的包搜索路径中,如下所示

```shell
$ echo $ROS_PACKAGE_PATH
/home/[username]/catkin_ws/src:/opt/ros/melodic/share
```

# 创建一个 catkin 包
`catkin_create_pkg`命令用于创建一个catkin包。进入catkin工作空间下的src目录,并创建catkin包 `beginer_tutorials`,如下
```shell
cd　~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
其中，std_msgs rospy　和 roscpp　为当前报的依赖包。命令执行完成后，在src目录下会多出一个 `beginner_tutorials`　目录,在该目录下分别有
1. package.xml 用于提供报的元信息
2. CMakeLists.txt 包的子cmake　文件

# 包元信息文件 package.xml 介绍

package 用于描述ros包的基本信息：
1. description 标签 -- description 标签用于对包进行描述。
2. maintainer 标签 -- maintainer 标签用于告诉其他人包的维护者的相关信息，以及维护者的联系方式（邮件）。至少需要添加一个维护者，也可以添加通过多个 maintainer 标签添加多个维护者。
3. license 标签 -- 用于描述包的许可证。
4. 包依赖标签 -- 包括构建依赖(build_depend)、构建工具依赖(buildtool_depend),运行时依赖(exec_depend)、测试依赖(test_depend)、导出（即依赖于该包时，同时也应该依赖于build_export_depend标签内定义的包12346）的依赖包(build_export_depend)、文档依赖(doc_depend)

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
**注意: 上述的shell命令中 catkin_ws 实际上并不在home目录下。**

从上面的输出结果可以看到，执行`catkin_make`实际上可以分为两个步骤。首先执行的是cmake命令，负责生成Makefile文件，命令输出的最开始为编译过程中的相关路径的配置，以及实际上的cmake的完整命令和cmake的输出；随后执行的是make命令，完成项目的编译。