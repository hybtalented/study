# 介绍
这篇文章主要介绍了如何制作一个树莓派的系统镜像。

# 制作镜像
制作系统镜像主要有以下几个作用
1. 备份系统
2. 迁移系统到其他介质
3. 批量安装定制的树莓派系统
下面我们将把之前定制的树莓派系统制作成一个镜像。

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