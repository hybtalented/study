# 创建catkin 工作空间
例如，我们要在用户主目录下创建一个文件夹`catkin_ws`作为 catkin　的工作目录，执行如下命令
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_build
```
执行完 `catkin_build`　后会在src文件夹下生成一个 CMakeLists.txt　文件，作为工作空间的主　cmake 文件。此外,　执行完 `catkin_build` 命令后还会在工作空间目录下生成build和devel目录，其中build目录为cmake的二进制目录，用于生成对应的二进制文件；d 　evel目录存放一些工作空间的环境相关的脚本，例如执行
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
