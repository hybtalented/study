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

在将树莓派系统环境同步到本地之前，我们先配置下述环境变量

```shell
echo "配置树莓派交叉编译"
export RASP_WORK_DIR=~/rpi-tools
export RASP_SYSROOT_DIR=${RASP_WORK_DIR}/raspberrypi/sysroot

export ARM_SYSTEM_ARCH=aarch64
export ARM_LIBRARY_ARCH=${ARM_SYSTEM_ARCH}-linux-gnu
export ARM_COMPILER_PATH=/usr
```
**将上述脚本放到`/etc/profile.d/`目录下，在开机时将会自动设置对应的环境变量**

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
**注意：如果树莓派系统更新或安装了软件包后，可以再次执行上述脚本，确保与树莓派之间保持文件同步。**

在编译`glibc`时，如果在`$SYSROOT_DIR/usr/include/`下没有找到`asm`文件夹，编译过程中将会报错。实际上`asm`是存在于`$SYSROOT_DIR/usr/include/$ARM_LIBRARY_ARCH`目录下，这时可以进入`$SYSROOT_DIR/usr/include/`目录，然后执行下述命令即可
```shell
ln -s $ARM_LIBRARY_ARCH/asm asm
```

由于树莓派系统中可能存在一些文件软链接指向了原始文件系统的中的路径，我们需要将这些链接改为相对路径，并指向同步后的系统路径下。我们将使用 [sysroot-relatiivelinks.py](./scripts/sysroot-relatiivelinks.py) 来完成这个任务。

```shell
python3 sysroot-relatiivelinks.py $RASP_SYSROOT_DIR
# 或者可以为 sysroot-relatiivelinks.py 添加可执行权限后, 通过如下方式达到同样目的
#  ./sysroot-relatiivelinks.py $RASP_SYSROOT_DIR
```

4. 编译并安装 Binutils

```shell
mkdir binutils-build && cd binutils-build
mkdir -p  $ARM_COMPILER_PATH
../binutils-2.31/configure --prefix=$ARM_COMPILER_PATH  --target=${ARM_LIBRARY_ARCH} --with-arch=armv6 --with-fpu=vfp --with-float=hard
make -j 8
make install
```

5. 编译 GCC 和 Glibc, gcc和glibc 是否相互以来，因此需要先部分编译 GCC和glibc，然后在完整的编译 GCC 和 glibc
首先先编译相应的交叉编译器二进制文件， 
```shell
mkdir gcc-build && cd gcc-build
../gcc-8.3.0/configure -v --with-pkgverion='$ARM_COMPILER_VERSION' --prefix=$ARM_COMPILER_PATH --target=$ARM_LIBRARY_ARCH  --enable-languages=c,c++,fortran --with-arch=armv6 --with-fpu=vfp --with-float=hard --enable-multiarch --with-sysroot=$RASP_SYSROOT_DIR
make -j8 all-gcc
make install-gcc
```
然后部分编译 glibc
```shell
mkdir glibc-build && cd glibc-build
../glibc-2.28/configure --prefix=$ARM_COMPILER_PATH/$ARM_LIBRARY_ARCH arm-linux-gnueabihf --build=$MACHTYPE --host=$ARM_LIBRARY_ARCH --target=$ARM_LIBRARY_ARCH --with-arch=armv6 --with-fpu=vfp --with-float=hard --with-headers=$RASP_SYSROOT_DIR/usr/include --disable-multilib libc_cv_forced_unwind=yes --enable-multi-arch
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
# 目标系统环境
set(CMAKE_SYSTEM_NAME ubuntu)
set(CMAKE_SYSTEM_VERSION 5.15.0-1015-raspi)

message(STATUS "use toolchain for ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_VERSION}")
# 交叉编译器架构
set(CMAKE_SYSTEM_PROCESSOR $ENV{ARM_SYSTEM_ARCH})
set(CMAKE_LIBRARY_ARCHITECTURE $ENV{ARM_LIBRARY_ARCH})
# 系统根目录
set(ARM_SYSROOT_DIR $ENV{RASP_SYSROOT_DIR})
# 交叉编译器设置
set(CMAKE_CXX_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-g++)
set(CMAKE_C_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-gcc)
# 交叉编译器目标环境
set(CMAKE_PREFIX_PATH $ENV{ROS2_INSTALL_PATH} ${ARM_SYSROOT_DIR} ${ARM_SYSROOT_DIR}/usr ${ARM_SYSROOT_DIR}/usr/local)
# CMake 系统文件目录
set(CMAKE_SYSROOT ${ARM_SYSROOT_DIR})

message(STATUS "SYSROOT ${CMAKE_SYSROOT}")
message(STATUS "FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "LIBRARY_ARCHITECTURE ${CMAKE_LIBRARY_ARCHITECTURE}")


set(PYTHON_SOABI cpython-36m-${CMAKE_LIBRARY_ARCHITECTURE})

# 在宿主机内寻找应用程序
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# 只在目标环境内寻找包含文件和库文件
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
```

在执行 cmake生存 Makefile 时，可以通过如下命令来指定交叉编译工具链

```shell
cmake -DCMAKE_TOOLCHAIN_FILE=${CURRENT_FOLDER}/arm-cross-compile.cmake
```

来之 CMake 使用交叉工具链编译代码。

# 参考文献
1. Building GCC as a cross compiler for Raspberry Pi， https://solarianprogrammer.com/2018/05/06/building-gcc-cross-compiler-raspberry-pi/

2. Cross-Compiling ROS for the Raspberry Pi, http://wiki.ros.org/ROS/CrossCompiling/RaspberryPi/Cross-Compile%20ROS%20for%20the%20RaspberryPi