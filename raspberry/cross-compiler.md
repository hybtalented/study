# 树莓派交叉编译环境搭建
参考文章 
[Building GCC as a cross compiler for Raspberry Pi](https://solarianprogrammer.com/2018/05/06/building-gcc-cross-compiler-raspberry-pi/)
[]()

- 宿主系统环境 ubuntu 18.04
- 树莓派环境 Linux version 4.19.97-v7l+

## 升级 gcc、g++ 版本到 8.x
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

## 编译 8.3 版本交叉编译器

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

3. 将树莓派系统上的相关环境同步到本地系统中

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