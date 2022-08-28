<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-08-17
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-28
 * @FilePath /raspberry/install-ros2.md
 * @Description 
-->
# 准备工作
1. 准备一张至少8G的无用SD卡用于安装树莓派的 ubuntu 系统

    将 SD 插入卡系统中后，可以使用 `fdisk -l` 命令查看 SD 对应的设备名称, 例如下面的 SD 卡设备名称为 `/dev/sdc`, 存储空间大小为 14.84GB
    ```shell
    Disk /dev/sdc：14.84 GiB，15931539456 字节，31116288 个扇区
    Disk model: MassStorageClass
    单元：扇区 / 1 * 512 = 512 字节
    扇区大小(逻辑/物理)：512 字节 / 512 字节
    I/O 大小(最小/最佳)：512 字节 / 512 字节
    磁盘标签类型：dos
    磁盘标识符：0xebb9b653

    设备       启动   起点     末尾     扇区  大小 Id 类型
    /dev/sdc1  *      2048   526335   524288  256M  c W95 FAT32 (LBA)
    /dev/sdc2       526336 15943903 15417568  7.4G 83 Linux
    ```

2. 从 ubuntu 官网下载树莓派版本的 [ubuntu-server 镜像](https://cn.ubuntu.com/download/raspberry-pi)(**如果树莓派配备有显示屏和鼠标键盘的话可以直接安装 ubuntu-desktop 镜像，因为 desktop 镜像开机时需要通过启动向导一步步配置网络和用户名、密码等**)
    <br/>
    从ubuntu官网下载的文件实际上是镜像文件的压缩包，我们需要通过 tar 命令镜像解压
    ```shell
    xz -d ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz
    ```
    解压后，可以得到名为 `ubuntu-22.04.1-preinstalled-server-arm64+raspi.img` 的镜像文件。
# 安装 ubuntu 系统
1. 直接使用 dd 命令将 ubuntu 镜像写入到 SD 卡中， 
    ```shell
    sudo dd bs=4M if=./ubuntu-22.04.1-preinstalled-server-arm64+raspi.img of=/dev/sdc status=progress
    ```
2. (可跳过**ubuntu 系统第一次启动会自动调整**) 使用 Parted 调整 SD 卡中 ubuntu 分区的大小。

    首先查看分区情况，如下所示我们可以看到写完镜像以后实际上只使用了 4G 左右的空间，启动启动分区大约有269M， 系统分区大约有3.6G。
    ```shell
    $ sudo parted /dev/sdc
    GNU Parted 3.4
    使用 /dev/sdc
    欢迎使用 GNU Parted！输入 'help' 来查看命令列表。
    (parted) p                                                                
    型号：Generic MassStorageClass (scsi)
    磁盘 /dev/sdc: 15.9GB
    扇区大小 (逻辑/物理)：512B/512B
    分区表：msdos
    磁盘标志：

    编号  起始点  结束点  大小    类型     文件系统  标志
    1    1049kB  269MB   268MB   primary  fat32     启动, lba
    2    269MB   3953MB  3683MB  primary  ext4

    ```
    为了完全利用 SD 的空间，我们可以使用 `parted` 对剩余空间进行分区，或者调整 ubuntu 分区的大小，在 `parted` 命令行中输入 `resizepart` 修改编号为 2 系统分区的大小

    ```shell
    (parted) resizepart
    分区编号？ 2                 
    结束点？  [3953MB]? 100% 
    ```
    重新查看分区大小可以看到
    ```shell
    (parted) p                                                                
    型号：Generic MassStorageClass (scsi)
    磁盘 /dev/sdc: 15.9GB
    扇区大小 (逻辑/物理)：512B/512B
    分区表：msdos
    磁盘标志：

    编号  起始点  结束点  大小    类型     文件系统  标志
    1    1049kB  269MB   268MB   primary  fat32     启动, lba
    2    269MB   15.9GB  15.6GB  primary  ext4
    ```
    系统分区的大小已经调整为15.6 GB。完成大小调整后，还需要通过 `e2fsck` 命令修复分区
    ```shell
    $ sudo e2fsck /dev/sdc2
    e2fsck 1.46.5 (30-Dec-2021)
    ext2fs_check_desc：组描述符损坏：块位图中有坏块
    e2fsck：组描述符似乎是错误的...正在尝试备份块...
    writable 未被彻底卸载，强制进行检查。
    第 1 遍：检查 inode、块，和大小
    第 2 遍：检查目录结构
    第 3 遍：检查目录连接性
    第 3A 遍：优化目录
    第 4 遍：检查引用计数
    第 5 遍：检查组概要信息
    块位图的差异：  +(819200--819640)
    处理<y>? 是
    Inode 位图末尾的填充值未设置。 处理<y>? 是

    writable：***** 文件系统已被修改 *****
    writable：77890/224896 文件（0.4% 为非连续的），596187/899241 块
    ```
3. 配置 ubuntu 系统的网络
    挂载安装的引导分区，然后修改 network-config 文件
    ```shell
    sudo mount /dev/sdc1 /media/system-boot
    gedit network-config
    ```
    去除 wifis 下所有字段的注释，在 access-points 字段下添加相应的 wifi 名称以及 wifi 的密码(记得去除每一行开头的 # 注释符号)
    ```yaml
    wifis:
      wlan0:
        dhcp4: true
        optional: true
        access-points:
          myhomewifi:
            password: "S3kr1t"
          myworkwifi:
            password: "correct battery horse staple"
          workssid:
            auth:
              key-management: eap
              method: peap
              identity: "me@example.com"
              password: "passw0rd"
              ca-certificate: /etc/my_ca.pem
    ```
    **注意： 在树莓派第一次启动的时候，将会尝试连接到这个网络，**

4. 将 SD 卡插入到树莓派中，并启动树莓派

5. 通过 SSH 远程到树莓派系统， 首先我们需要确定树莓派的 ip 地址， 通过 `arp` 命令让我们可以通过树莓派的 mac 地址查找的其对应的 ip 地址
    ```shell 
    arp -na | grep -i "b8:27:eb\|dc:a6:32\|e4:5f:01"
    ```
      得到终端的ip以后，使用ssh登陆到树莓派
    ```shell
    ssh ubuntu@IP地4址
    ```
      系统的初始密码为 ubuntu, 第一次成功登陆会要求修改 ubuntu 用户的密码, 修改完密码后 ssh 连接会自动退出。

      如果需要设置ssh免密登陆， 将宿主机上的 ssh 公钥考到树莓派上，并且附加到树莓派的信任密钥文件 `/home/ubuntu/.ssh/authorized_keys` 的末尾（如果文件不存在的话则创建, 下面的代码将会把宿主机上的公钥文件直接替换掉树莓派上的信任密钥文件
    ```shell
    scp /home/<host_user_name>/.ssh/id_rsa.pub ubuntu@192.168.2.100:/home/ubuntu/.ssh/authorized_keys
    ```
    之后在宿主机可以免密登陆树莓派。

6. （可选）配置apt源
    为了提高 apt 的速度，可以重新配置 apt 源， 例如在[清华大学开源软件镜像站](https://mirror.tuna.tsinghua.edu.cn/help/ubuntu-ports/)上可以查找到指定版本的 ubuntu 系统的 apt 源配置。完成 apt 源的更新后，我们需要使配置生效
    ```shell
      sudo apt update
      sudo apt upgrade
    ```
    在上诉过程中，如果树莓派无法解析域名，需要手动配置 dns 服务器, 打开 `vim /etc/netplan/50-cloud-init.yaml`, 并在对应网络下的 `nameservers->addresses` 字段下添加相应的域名解析服务器： 
    ```yaml
    wifis:
          wlan0:
              access-points:
                  hybtalented:
                      password: '13491633'
                  myworkwifi:
                      password: correct battery horse staple
                  workssid:
                      auth:
                          ca-certificate: /etc/my_ca.pem
                          identity: me@example.com
                          key-management: eap
                          method: peap
                          password: passw0rd
              dhcp4: true
              optional: true
              nameservers:
                  addresses:
                      - 8.8.8.8
                      - 114.114.114.114
    ```
    最后执行
    ```shell
    sudo netplan apply
    ```
    使配置生效。
7. （可选）安装 ubuntu 桌面
  ubuntu 上安装桌面非常简单，以下是两个非常受欢迎的 ubuntu 桌面
    ```
    # 安装 xubuntu
    sudo apt install xubuntu-desktop
    # 安装 lubuntu
    sudo apt install lubuntu-desktop
    ```
    然后 `sudo reboot` 重启树莓派。
    安装完 ubuntu 桌面后，如果树莓派未配置显示器，而又想要进入桌面，我们需要在树莓派上安装远程桌面工具，下面将介绍如何通过 vnc 协议远程桌面到树莓派。
    首先我们需要安装 vnc 服务器
    ```shell
    sudo apt-get install tightvncserver
    ```
    然后启动 vnc 服务器，并初始化密码
    ```shell
    $ tightvncserver 

    You will require a password to access your desktops.

    Password: 
    Verify:   
    Would you like to enter a view-only password (y/n)? n
    xauth:  file /home/ubuntu/.Xauthority does not exist

    New 'X' desktop is ubuntu:1

    Creating default startup script /home/ubuntu/.vnc/xstartup
    Starting applications specified in /home/ubuntu/.vnc/xstartup
    Log file is /home/ubuntu/.vnc/ubuntu:1.log
    ```
# 安装 ros2
在这一小节，我们将配置 ros2 的[清华的软件源](https://mirror.tuna.tsinghua.edu.cn/help/ros2/)，并安装和配置 ros2

首先，我们需要安装 ros2 的 apt 源
```shell
# 下载并保存 ros2 的 apt 密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
# 配置 ros2 的 apt 源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# 更新 apt 源
sudo apt update
```
之后我们可以选择需要的 ros2 [版本](https://docs.ros.org/en/humble/Releases.html)进行安装
```shell
# 桌面版，包含 ROS, RViz, demo 和 教程

sudo apt install ros-rolling-desktop
# 基础版本，不包含相应 GUI 工具
sudo apt install ros-rolling-ros-base
```

# 安装 wiringPi
由于 Ubuntu 上 `apt`仓库 的 wiringPi 包不支持树莓派  4B, 我们需要手动安装 wiringPi, 目前 wiringPi 的源代码和发布版可以在 [github](https://github.com/WiringPi/WiringPi/releases) 上找到, 下面我们以2.61.1版本为例安装 `wiringpi`
```shell
# 64位系统
wget https://github.com/WiringPi/WiringPi/releases/download/2.61-1/wiringpi-2.61-1-armhf.deb
sudo dpkg -i wiringpi-2.61-1-arm64.deb

# 32为系统
wget https://github.com/WiringPi/WiringPi/releases/download/2.61-1/wiringpi-2.61-1-armhf.deb
sudo dpkg -i wiringpi-2.61-1-armhf.deb
```
安装完成后，可以查看gpio的版本
```shell
$ gpio -v
gpio version: 2.61
Copyright (c) 2012-2018 Gordon Henderson
This is free software with ABSOLUTELY NO WARRANTY.
For details type: gpio -warranty

Raspberry Pi Details:
  Type: Pi 4B, Revision: 05, Memory: 8192MB, Maker: Sony 
  * Device tree is enabled.
  *--> Raspberry Pi 4 Model B Rev 1.5
  * This Raspberry Pi supports user-level GPIO access.
```
安装完 `wiringpi` 后，如果需要用 gpio 口进行 `i2c` 和`lirc` 通信，我们还需要安装下述的包 
```
sudo apt install libi2c-dev i2c-tools lirc liblirc-dev
```
# 配置交叉编译环境
1. 安装交叉编译器。由于宿主机和树莓派都安装了同样的 ubuntu 系统（22.04），我们可以直接在宿主机上通过 `apt` 安装交叉编译器，如果需要手动编译交叉编译器，可以参考 [树莓派交叉编译环境搭建](./cross-compiler.md)。
    ```shell
    # 32位树莓派系统
    sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
    # 64位树莓派系统
    sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
    ```
    可以尝试使用交叉编译器编译下述程序
    ```c++
    // hello_world.c
    #include <stdio.h>

    int main() {
        printf("hello wolrd\n");
    }
    ```
    命令如下
    ```shell
    # 32位树莓派系统
    arm-linux-gnueabihf-gcc hello_world.c
    # 64位树莓派系统
    aarch64-linux-gnu-gcc hello_world.c
    ```
    然后将编译出的文件考到树莓派上
    ```shell
    scp a.out ubuntu@<树莓派IP>:hello_world
    ```
    在树莓派上执行编译出的程序：
    ```shell
    ubuntu@ubuntu:~$ ./hello_world 
    hello wolrd
    ```
2. 同步树莓派的系统文件。在交叉编译的过程中会用到一些树莓派的编译环境，我们需要将树莓派中的编译环境同步到宿主机上，首先在 `home` 目录下创建 `rpi-toolsrpi-tools/raspberrypi/sysroot` 文件夹，并进入`rpi-toolsrpi-tools/raspberrypi`目录
    ```shell
    mkdir -p ~/rpi-tools/raspberrypi/sysroot
    cd ~/rpi-tools/raspberrypi
    ```
    然后在 `raspberrypi` 创建 [syncsysroot.sh](./scripts/syncsysroot.sh) 脚本，并输入如下内容
    ```shell
    SSH_ADDRESS=ubuntu@192.168.1.21
    rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/lib sysroot
    rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/usr/include sysroot/usr
    rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/usr/lib sysroot/usr
    ```
    其中 `SSH_ADDRESS` 变量为树莓派系统的 ssh 用户名和地址。然后为 `syncsysroot.sh` 添加可执行权限
    ```sehll
    chmod +x syncsysroot.sh
    ```
3. 配置环境变量。为了方便我们后续 cmake 脚本的配置，其他树莓派相关环境的配置，我们添加一个 shell 启动脚本 [arm-env.sh](./scripts/arm-env.sh) 放在 `~/.bashrc.d` 目录下，其内容如下
    ```shell
    # arm-env.sh
    echo "配置树莓派交叉编译"
    export RASP_WORK_DIR=/home/hybtalented/rpi-tools
    export RASP_SYSROOT_DIR=${RASP_WORK_DIR}/raspberrypi/sysroot

    # export ARM_TOOL_DIR=${RASP_WORK_DIR}/tools
    # export ARM_COMPILER_VERSION=8.3-raspberrypi-4.19.97-v7l+
    export ARM_COMPILER_ARCH=aarch64-linux-gnu
    # export ARM_COMPILER_PATH=${ARM_TOOL_DIR}/gcc-arm-${ARM_COMPILER_VERSION}-${ARM_COMPILER_ARCH}
    export ARM_COMPILER_PATH=/usr
    # export PATH=${ARM_COMPILER_PATH}/bin:$PATH
    ```
    可以看到我们配置了树梅派的工作目录 `RASP_WORK_DIR` 以及树梅派的系统环境目录 `RASP_SYSROOT_DIR`、编译器架构 `ARM_COMPILER_ARCH` 和交叉编译 `ARM_COMPILER_PATH` 环境目录四个环境变量，其中 cmake 配置中将用到 `RASP_SYSROOT_DIR`、`ARM_COMPILER_ARCH` 、`ARM_COMPILER_PATH` 这三个环境变量。
4. 配置 cmake 交叉编译工具链。创建交叉工具链配置文件 `arm-cross-compile.cmake` ，并将如下内容拷贝到文件中
    ```cmake
    # 目标系统环境
    set(CMAKE_SYSTEM_NAME raspberrypi)
    set(CMAKE_SYSTEM_VERSION 4.19.97-v7l+)
    # 交叉编译器架构
    set(CMAKE_LIBRARY_ARCHITECTURE $ENV{ARM_COMPILER_ARCH})
    # 系统根目录
    set(ARM_SYSROOT_DIR $ENV{RASP_SYSROOT_DIR})
    # 交叉编译器设置
    set(CMAKE_CXX_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-g++)
    set(CMAKE_C_COMPILER ${CMAKE_LIBRARY_ARCHITECTURE}-gcc)
    # 交叉编译器目标环境
    set(CMAKE_FIND_ROOT_PATH ${ARM_SYSROOT_DIR} $ENV{ARM_COMPILER_PATH}/${CMAKE_LIBRARY_ARCHITECTURE})
    # CMake 系统文件目录
    set(CMAKE_SYSROOT ${ARM_SYSROOT_DIR})

    message(STATUS "SYSROOT ${CMAKE_SYSROOT}")
    message(STATUS "FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH}")
    message(STATUS "LIBRARY_ARCHITECTURE ${CMAKE_LIBRARY_ARCHITECTURE}")

    # 在宿主机和目标环境内寻找应用程序
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)

    # 只在目标环境内寻找包含文件和库文件
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    ```
    在这个工具链中我们主要设置了目标系统架构 `CMAKE_LIBRARY_ARCHITECTURE`， 目标系统环境目录 `CMAKE_SYSROOT`， 交叉编译器 `CMAKE_CXX_COMPILER` 和 `CMAKE_C_COMPILER` 以及 cmake 搜索根目录 `CMAKE_FIND_ROOT_PATH`。

# 制作镜像
制作系统镜像主要有以下几个作用
1. 备份系统
2. 迁移系统到其他介质
3. 批量安装定制的树梅派系统
下面我们将把之前定制的树梅派系统制作成一个镜像。

1. 创建一个空的镜像文件。通过下面的代码，我们创建了一个 8GB (4MB*2000)大小的镜像文件
```shell
sudo dd if=/dev/zero of=ubuntu-rasp-server-ros2-gpio.img bs=4M count=2000
```
2. 通过 parted 为镜像文件分区
    首先进入 `parted` 的命令行
    ```shell
    sudo parted ubuntu-rasp-server-ros2-gpio.img
    ```
    在 `parted` 命令行中创建 `msdos` 类型的分区表
    ```shell
    (parted) mklabel msdos 
    ``` 
    ```
    (parted) mkpart 
    分区类型？  primary/主分区/extended/扩展? primary                         
    文件系统类型？  [ext2]? fat32                                             
    起始点？ 1049kB
    结束点？ 269MB
    (parted) mkpart
    分区类型？  primary/主分区/extended/扩展? primary
    文件系统类型？  [ext2]? ext4                                              
    起始点？ 269MB
    结束点？ 100% 
    ```
    完成分区后，可以在命令行中输入 `quit` 退出 `parted`， 然后我们可以通过 `fdisk` 命令查看分区是否成功
    ```shell
    $ sudo fdisk -l ./ubuntu-rasp-server-ros2-gpio.img 
    Disk ./ubuntu-rasp-server-ros2-gpio.img：8 GiB，8589934592 字节，16777216 个扇区
    单元：扇区 / 1 * 512 = 512 字节
    扇区大小(逻辑/物理)：512 字节 / 512 字节
    I/O 大小(最小/最佳)：512 字节 / 512 字节
    磁盘标签类型：dos
    磁盘标识符：0xf735e79a

    设备                                启动   起点     末尾     扇区  大小 Id 类型
    ./ubuntu-rasp-server-ros2-gpio.img1        2048   526335   524288  256M  c W95 FAT32 (LBA)
    ./ubuntu-rasp-server-ros2-gpio.img2      526336 16777215 16250880  7.7G 83 Linux
    ```
3. 挂载镜像文件中的分区。
    首先，将镜像文件挂载为一个循环设备
    ```shell
    sudo losetup -f --show ubuntu-rasp-server-ros2-gpio.img 
    /dev/loop15
    ```
    可以看到文件被挂载成了 `/dev/loop15`，然后我们可以使用 `kpartx` 加载设备中的所有分区。
    如果你的系统中没有安装 `kpartx`，可以使用 `apt` 进行安装
    ```shell
    sudo apt install kpartx
    ```
    加载分区
    ```shell
    $ sudo kpartx -va /dev/loop15
    add map loop15p1 (253:0): 0 524288 linear 7:15 2048
    add map loop15p2 (253:1): 0 16250880 linear 7:15 526336
    ```
    可以看到两个分区分别被加载为 `loop15p1` 和 `loop15p2`, 其中分区所在目录为 `/dev/mapper`。下面我们对分区进行格式化
    ```shell
    $ sudo mkfs.vfat -n system-boot /dev/mapper/loop15p1
    mkfs.fat 4.2 (2021-01-31)
    mkfs.fat: Warning: lowercase labels might not work properly on some systems
    $ sudo mkfs.ext4 -L writable /dev/mapper/loop15p2
    mke2fs 1.46.5 (30-Dec-2021)
    丢弃设备块： 完成                            
    创建含有 2031360 个块（每块 4k）和 507904 个 inode 的文件系统
    文件系统 UUID：fbebc897-db83-45f8-9a62-7ea3c0705c4b
    超级块的备份存储于下列块： 
      32768, 98304, 163840, 229376, 294912, 819200, 884736, 1605632

    正在分配组表： 完成                            
    正在写入 inode表： 完成                            
    创建日志（16384 个块）： 完成
    写入超级块和文件系统账户统计信息： 已完成
    ```
    挂载两个分区
    ```
    $ mkdir -p /mnt/system-boot/
    $ mkdir -p /mnt/rootfs/
    $ sudo mount --types vfat -o uid=hybtalented,gid=hybtalented,umask=0000 /dev/mapper/loop15p1 /mnt/system-boot/
    $ sudo mount --types ext4 /dev/mapper/loop15p2 /mnt/rootfs/
    ```
    其中 fat 分区加载过程中, `-o` 选项指定了分区的文件权限 并且 uid， gid 分别指定了当前用户的用户名和用户组名, 可以通过 `id`命令查看
    ```shell
    $ id
    用户id=1000(hybtalented) 组id=1000(hybtalented) 组=1000(hybtalented),4(adm),24(cdrom),27(sudo),30(dip),46(plugdev),122(lpadmin),134(lxd),135(sambashare),999(docker)
    ```
4. 备份 boot 分区
    ```shell
    sudo cp -rfp /media/hybtalented/system-boot/* /mnt/system-boot
    ```
5. 备份 root 分区

    为了备份 ubuntu 根分区，我们使用 `dump` + `restore` 方式对分区进行备份首先修改分区的权限，并清空分区
    ```shell
    sudo chmod 777 /mnt/rootfs/
    sudo chown hybtalented.hybtalented /mnt/rootfs/
    rm -rf /mnt/rootfs/* 
    cd /mnt/rootfs
    ```
    然后我们使用 `dump` 命令备份
    ```shell
    sudo dump -0uaf - /media/hybtalented/writable/ | sudo restore -rf -
    ```
    如果系统提示 `dump` 或 `restore` 未安装，可以通过 `apt` 安装
    ```shell
    sudo apt install dump
    ```
    至此已经完成了磁盘镜像的制作。

    **需要注意的是，我们之前为 boot 分区和 root 分区制作文件系统时，分别通过 -n 和 -L 指定了分区名称为 `system-boot` 和 `writable`, 这是由于 ubuntu 是根据卷标加载 root 分区和 boot 分区的, 如下所示**
    ```
    $ cat /mnt/rootfs/etc/fstab 
    LABEL=writable	/	ext4	discard,errors=remount-ro	0 1
    LABEL=system-boot       /boot/firmware  vfat    defaults        0       1
    $ cat /mnt/system-boot/cmdline.txt 
    console=serial0,115200 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc quiet splash
    ```
    如果需要根据分区 id 需要挂载的 boot 分区和 root 分区,可以将上述文件中的 `label=xxx` 改为 `UUID=xxx`, 其中分区的 uuid 可以通过 `lsblk` 命令查看
    ```shell
    $ lsblk -f
    NAME        FSTYPE   FSVER LABEL       UUID                                 FSAVAIL FSUSE% MOUNTPOINTS
    ...  
    loop15             
    ├─loop15p1  vfat     FAT16 system-boot C901-EF0E                             107.3M    58% /mnt/system-boot
    └─loop15p2  ext4     1.0   writable    fbebc897-db83-45f8-9a62-7ea3c0705c4b    3.7G    46% /mnt/rootfs
    ```
    可以看到 boot 分区的 uuid 为 `C901-EF0E` root 分区的 uuid 为 `fbebc897-db83-45f8-9a62-7ea3c0705c4b`。
6. 卸载镜像文件
   ```shell
   # 卸载 boot 和 root 文件系统
   sudo umount /mnt/system-boot /mnt/rootfs
   # 卸载分区设备
   sudo kpartx -d /dev/loop15
   # 卸载镜像设备
   sudo losetup -d /dev/loop15
   ```