<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-08-17
 * @LastEditors Youbiao He
 * @LastEditTime 2022-08-18
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
2. 从 ubuntu 官网下载树莓派版本的 [ubuntu-server 镜像](https://cn.ubuntu.com/download/raspberry-pi)(**如果树梅派配备有显示屏和鼠标键盘的话可以直接安装 ubuntu-desktop 镜像，因为 desktop 镜像开机时需要通过启动向导一步步配置网络和用户名、密码等**)
<br/>
从ubuntu官网下载的文件实际上是镜像文件的压缩包，我们需要通过 tar 命令镜像解压
```shell
xz -d ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz
```
解压后，可以得到名为 `ubuntu-22.04.1-preinstalled-server-arm64+raspi.img` 的镜像文件。
# 安装 ubuntu 系统
1. 直接使用 dd 命令将 ubuntu 镜像写入到 SD 卡中， 
```shell
sudo dd if=./ubuntu-22.04.1-preinstalled-server-arm64+raspi.img of=/dev/sdc status=progress
```
2. 使用 Parted 调整 SD 卡中 ubuntu 分区的大小
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
去除 wifis 下所有字段的注释，在 access-points 字段下添加相应的 wifi 名称以及 wifi 的密码
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
4. 将 SD 卡插入到树莓派中，并启动树梅派
5. 通过 SSH 远程到树梅派系统， 首先我们需要确定树梅派的 ip 地址， 通过 `arp` 命令让我们可以通过树梅派的 mac 地址查找的其对应的 ip 地址
```shell 
arp -na | grep -i "b8:27:eb\|dc:a6:32\|e4:5f:01"
```
6. （可选）配置apt源
   为了提高 apt 的速度，可以重新配置 apt 源， 例如在[清华大学开源软件镜像站](https://mirror.tuna.tsinghua.edu.cn/help/ubuntu-ports/)上可以查找到指定版本的 ubuntu 系统的 apt 源配置。
7. （可选）安装 ubuntu 桌面
  
# 安装 ros2