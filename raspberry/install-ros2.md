<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-08-17
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-21
 * @FilePath /raspberry/install-ros2.md
 * @Description 
-->
# 准备工作
1. 准备一张至少8G的无用SD卡用于安装树莓派的 ubuntu 系统
<br/>
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
2. (可跳过**ubuntu 系统第一次启动会自动调整**) 使用 Parted 调整 SD 卡中 ubuntu 分区的大小
<br/>
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
ssh ubuntu@IP地址
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

# 配置交叉编译环境

