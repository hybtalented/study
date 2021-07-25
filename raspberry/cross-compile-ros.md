在之前的[文章](./cross-compiler.md)中，已经介绍了如何配置树莓派的交叉编译环境，这篇文章将利用交叉编译环境交叉编译 `ROS Melodic`。

# 准备工作

1. 在树莓派系统中，安装相关依赖包

```shell
sudo apt install python-rosdep libconsole-bridge-dev libboost-all-dev libpoco-dev
```

至此，在树莓派上的操作以及全部完成了，后续操作都将在 宿主系统上进行。

2. 将树莓派的库和头文件同步到宿主系统中，对应命令已经在上一篇文字中介绍了，可以在之前同步的基础上继续同步，命令如下
```shell
IP_ADDRESS=192.168.0.110
SYSROOT_DIR=$RASP_SYSROOT_DIR/sysroot
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/lib $SYSROOT_DIR
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/usr/include $SYSROOT_DIR/usr
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/usr/lib $SYSROOT_DIR/usr
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/opt/vc $SYSROOT_DIR/opt
```
3. 宿主环境初始化 ROS 构建依赖（以Ubuntu 18.04为例，其他系统可以参考[ROS官方文档](http://wiki.ros.org/melodic/Installation/Source)）
```shell
sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential
sudo rosdep init
sudo rosdep update
```

在进行 rosdep 初始化时，可能会由于网络问题报错，这时建议使用 proxychain4 进行代理（代理服务器配置请参考 https://monkeywie.cn/2020/07/06/linux-global-proxy-tool-proxychain/）。
4. 创建并初始化 ros 交叉编译工作目录
```shell
sudo mkdir ros
cd ros
```

5. 下载并安装 ROS 源码
```shell
rosinstall_generator desktop --rosdistro melodic --deps --tar > melodic-desktop.rosinstall
vcs import src < melodic-desktop.rosinstall
```
这一步同样可能会由于网络问题失败，这种情况请使用 proxychain4 执行上述命令。
# 构建 ROS Melodic
1. 配置工具CMake 工具链配置文件，并将配置文件放置到ros 交叉编译工作目录
```cmake
# arm-cross-compile.cmake

# 目标系统环境
set(CMAKE_SYSTEM_NAME raspberrypi)
set(CMAKE_SYSTEM_VERSION 4.19.97-v7l+)
# 交叉编译器架构
set(CMAKE_LIBRARY_ARCHITECTURE $ENV{ARM_COMPILER_ARCH} CACHE STRING "交叉编译器架构")
# 系统根目录
set(ARM_SYSROOT_DIR $ENV{RASP_SYSROOT_DIR})
# 交叉编译器设置
set(CMAKE_CXX_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-g++)
set(CMAKE_C_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-gcc)
# 交叉编译器目标环境
set(CMAKE_FIND_ROOT_PATH  $ENV{ARM_COMPILER_PATH}/${CMAKE_LIBRARY_ARCHITECTURE})
# CMake 系统文件目录，用于 CMake 相关包的搜索路径，以及 gcc 头文件和库文件的默认搜索目录
set(CMAKE_SYSROOT ${ARM_SYSROOT_DIR})
# 在宿主机和目标环境内寻找应用程序
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)

# 只在目标环境内寻找包含文件和库文件和第三方CMake包
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```
2. 为ROS源码的CMake配置文件打上[补丁](https://gist.github.com/easdue/39539f50aac5b20dbb05c9008ac2365c/archive/d130bff9cafde5d70600d05caba9743cca103d03.zip)，解压补丁文件到ROS交叉编译工作目录下，然后执行下列命令
```shell
patch -p1 < ../ros_melodic_pi.patch
```
4. 交叉编译 ROS
```shell
CURRENT_DIR=$(dirname $(readlink -f $0))
./src/catkin/bin/catkin_make_isolated --install --install-space=$CURRENT_DIR/raspberrypi -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMAKE_TOOLCHAIN_FILE=$CURRENT_DIR/arm-cross-compiler.cmake
```
如果在编译过程中报了CMake缺少相关依赖的错误，请在ROS系统中用apt 安装相关的 ` <library>-dev` 包，并将相关头文件和库文件同步到宿主系统中（参考上一节第2步),然后重新执行上述命令。

完成ros构建后相关文件将被安装到`--install-space`指定的目录下。

# 安装ROS到树莓派
1. 在树莓派系统中创建`/opt/ros/melodic`目录，然后在宿主系统中执行下述命令，将 ROS Melodic 拷贝到树莓派系统中
```shell
scp -r ./raspberrypi/* pi@192.168.0.105:/opt/ros/melodic
```
2. 在ros系统中加载ROS相关环境变量
```shell
source /opt/ros/melodic/setup.bash
```

至此，已经完成了ROS的交叉编译以及安装。

# 参考文献

1. Cross-Compiling ROS for the Raspberry Pi, http://wiki.ros.org/ROS/CrossCompiling/RaspberryPi/Cross-Compile%20ROS%20for%20the%20RaspberryPi
2. Installing melodic from source, http://wiki.ros.org/melodic/Installation/Source



