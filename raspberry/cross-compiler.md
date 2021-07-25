# 树莓派交叉编译环境搭建
参考文章 
[Building GCC as a cross compiler for Raspberry Pi](https://solarianprogrammer.com/2018/05/06/building-gcc-cross-compiler-raspberry-pi/)
[]()

- 宿主系统环境 ubuntu 18.04
- 树莓派环境 Linux version 4.19.97-v7l+

## 升级宿主环境 gcc、g++ 版本到 8.x
```shell
sudo apt-get update
# 安装 gcc-8 和 g++-8
sudo apt-get install gcc-8
sudo apt-get install g++-8
cd /usr/bin
# 将 gcc、g++ 的默认版本设置为 8
sudo ln -s gcc-8 gcc
sudo ln -s g++-8
```

## 编译 8.3 版本交叉编译器 gcc

1. 首先，需要确保系统安装了开发依赖项
```shell
sudo apt update
sudo apt upgrade
sudo apt install build-essential gawk git texinfo bison file wget
```

在树莓派4B的系统中安装的编译器的是 gcc8.3.0 Binutil 2.31 和 glib 2.28,因此交叉编译器也需要安装对应的版本。下述命令可以查看对应的工具的版本
```shell
gcc --version
ld -v
ldd --version
```

2. 我们需要下载对应工具的源码
```shell
wget https://ftpmirror.gnu.org/binutils/binutils-2.31.tar.bz2 https://ftpmirror.gnu.org/glibc/glibc-2.28.tar.bz2 https://ftpmirror.gnu.org/gcc/gcc-8.3.0/gcc-8.3.0.tar.gz
```
解压缩相应的压缩吧
```shell
tar xf binutils-2.31.tar.bz2
tar xf  glibc-2.28.tar.bz2
tar xf  gcc-8.3.0.tar.gz
```
除此之外 gcc 还需要安装下载一些依赖，进入 gcc 源码目录，执行如下命令
```shell
cd gcc-8.3.0
contrib/download_prerequisites
# 删除压缩包
rm *.tar.*
```

3. 将树莓派系统上的编译环境同步到本地系统中

在将树莓派系统环境同步到本地之前，我们先配置下诉环境变量

```shell
# /etc/profile.d/arm-env.sh
# 树莓派工作目录
export RASP_WORK_DIR=/home/hybtalented/rpi-tools
# 树莓派系统库目录
export  RASP_SYSROOT_DIR=${RASP_WORK_DIR}/raspberrypi/sysroot

# 树莓派交叉编译工具工具链目录
export ARM_TOOL_DIR=${RASP_WORK_DIR}/tools
# 树莓派交叉工具链版本号
export ARM_COMPILER_VERSION=8.3-raspberrypi-4.19.97-v7l+

# 树莓派交叉工具链架构
export ARM_COMPILER_ARCH=arm-linux-gnueabihf
# 当前使用的树莓派交叉工具链
export ARM_COMPILER_PATH=${ARM_TOOL_DIR}/gcc-arm-${ARM_COMPILER_VERSION}-${ARM_COMPILER_ARCH}
# 将树莓派交叉工具链的可执行目录添加到 PATH
export PATH=$PATH:${ARM_COMPILER_PATH}/bin
```
**可以将上述脚本放到`/etc/profile.d/`目录下，从而在开机时会自动加载对应的环境变量**

其中，树莓派系统环境将被同步到`RASP_SYSROOT_DIR`所在的目录下，交叉编译器将被安装到`ARM_COMPILER_PATH`目录下。

假设树莓派的ip地址为 `192.168.0.110`，执行下述命令可以将树莓派系统中的相关文件同步到本地文件系统中
```shell
IP_ADDRESS=192.168.0.110
SYSROOT_DIR=$RASP_SYSROOT_DIR/sysroot
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/lib $SYSROOT_DIR
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/usr/include $SYSROOT_DIR/usr
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/usr/lib $SYSROOT_DIR/usr
rsync -avz --rsync-path="sudo rsync" --delete pi@$IP_ADDRESS:/opt/vc $SYSROOT_DIR/opt
```
**注意：上述命令可以执行任意次，从而在树莓派系统安装新的库之后，仍然可以与树莓派之间的文件同步。**

在编译`glibc`时，如果在`$SYSROOT_DIR/usr/include/`下没有找到`asm`文件夹，编译过程中将会报错。实际上`asm`是存在于`$SYSROOT_DIR/usr/include/$ARM_COMPILER_ARCH`目录下，这时可以进入`$SYSROOT_DIR/usr/include/`目录，然后执行下述命令即可
```shell
ln -s $ARM_COMPILER_ARCH/asm asm
```

4. 编译并安装 Binutils

```shell
mkdir binutils-build && cd binutils-build
mkdir -p  $ARM_COMPILER_PATH
../binutils-2.31/configure --prefix=$ARM_COMPILER_PATH  --target=${ARM_COMPILER_ARCH} --with-arch=armv6 --with-fpu=vfp --with-float=hard
make -j 8
make install
```

5. 编译 GCC 和 Glibc, gcc和glibc 是否相互以来，因此需要先部分编译 GCC和glibc，然后在完整的编译 GCC 和 glibc
首先先编译相应的交叉编译器二进制文件， 
```shell
mkdir gcc-build && cd gcc-build
../gcc-8.3.0/configure -v --with-pkgverion='$ARM_COMPILER_VERSION' --prefix=$ARM_COMPILER_PATH --target=$ARM_COMPILER_ARCH  --enable-languages=c,c++,fortran --with-arch=armv6 --with-fpu=vfp --with-float=hard --enable-multiarch --with-sysroot=$RASP_SYSROOT_DIR
make -j8 all-gcc
make install-gcc
```
然后部分编译 glibc
```shell
mkdir glibc-build && cd glibc-build
../glibc-2.28/configure --prefix=$ARM_COMPILER_PATH/$ARM_COMPILER_ARCH arm-linux-gnueabihf --build=$MACHTYPE --host=$ARM_COMPILER_ARCH --target=$ARM_COMPILER_ARCH --with-arch=armv6 --with-fpu=vfp --with-float=hard --with-headers=$RASP_SYSROOT_DIR/usr/include --disable-multilib libc_cv_forced_unwind=yes --enable-multi-arch
make install-bootstrap-headers=yes install-headers
```
回到gcc的构建目录，执行下述命令进行gcc的第二阶段编译

```shell
make -j8 all-target-libgcc
make install-target-libgcc
```
然后回到glibc的构建目录，完成glibc的编译和安装

```shell
make -j8
make install
```

最后回到gcc的构建目录，完成交叉编译器gcc的构建和安装
```shell
make -j8
make install
```
至此，交叉编译环境的gcc以及编译完毕。由于，之前配置的环境编译以及将交叉编译器的路径添加到`PATH`中，查看交叉gcc的版本如下
```shell
hybtalented@hybtaletented-163-com:~/study$ arm-linux-gnueabihf-gcc -v
使用内建 specs。
COLLECT_GCC=arm-linux-gnueabihf-gcc
COLLECT_LTO_WRAPPER=/home/hybtalented/rpi-tools/tools/gcc-arm-8.3-raspberrypi-4.19.97-v7l+-arm-linux-gnueabihf/libexec/gcc/arm-linux-gnueabihf/8.3.0/lto-wrapper
目标：arm-linux-gnueabihf
配置为：../gcc-8.3.0/configure -v --with-pkgverion='8.3-raspberrypi-4.19.97-v7l+' --prefix=/home/hybtalented/rpi-tools/tools/gcc-arm-8.3-raspberrypi-4.19.97-v7l+-arm-linux-gnueabihf --target=arm-linux-gnueabihf --enable-languages=c,c++,fortran --with-arch=armv6 --with-fpu=vfp --with-float=hard --enable-multiarch --with-sysroot=/home/hybtalented/rpi-tools/raspberrypi/sysroot
线程模型：posix
gcc 版本 8.3.0 (GCC) 
```
可以尝试使用交叉编译器编译下述程序
```c++
// hello_world.c
#include <stdio.h>

int main() {
    printf("hello wolrd\n");
}
```
直接执行下述命令编译 hello word，测试编译器是否能够正常运行。

```shell
arm-linux-gnueabihf-gcc hello_world.c
```

## 配置交叉编译器 CMake

在之前的小节，已经将树莓派系统中的编译环境同步到本地，并且编译并安装了交叉编译版本的 gcc， 在这个小节我们将要配置 CMake 交叉工具链。

基于我们之前配置的环境变量 交叉工具链的CMake 配置如下
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

在相应的 `CMakeLists.txt` 中添加下述语句
```cmake
include(arm-cross-compile.cmake)
```

或者执行 CMake 时添加

```shell
cmake -DCMAKE_TOOLCHAIN_FILE=${CURRENT_FOLDER}/arm-cross-compile.cmake
```

可以让 CMake 使用交叉工具链编译代码。

# 参考文献
1. Building GCC as a cross compiler for Raspberry Pi， https://solarianprogrammer.com/2018/05/06/building-gcc-cross-compiler-raspberry-pi/

2. Cross-Compiling ROS for the Raspberry Pi, http://wiki.ros.org/ROS/CrossCompiling/RaspberryPi/Cross-Compile%20ROS%20for%20the%20RaspberryPi