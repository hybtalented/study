# ROS 节点（Node）的定义
ROS系统由以下几个部分组成
1. 节点（Nodes） - ROS节点是一个通过ROS与其他节点进行通信的可执行程序
2. 消息（Messages） - ROS消息是发布订阅主题所使用的结构化数据
3. 主题（Topics） - ROS节点可以发布一个消息到主题，或者订阅主题以接收消息
4. 主节点 （Master) - 主节点提供命名服务，ROS节点可以通过主节点找到其它ROS节点
5. rosout - rosout 为ROS用于的输出服务的节点，等价于 stdout/stderr
6. roscore - Master 服务、 rosout 服务以及 ros 参数服务


ROS节点就是一个通过ROS包编译出来的可执行程序，ROS节点可以通过ROS客户端库与其它节点进行通信。ROS阶段可以发布或订阅一个主题，也可以使用或者提供一个服务。


# 启动 roscore 
在ROS启动前，首先要启动 roscore 服务，
```shell
roscore
```
如果服务正常启动，将会产生如下类似的消息
```
... logging to /home/hybtalented/.ros/log/af5e3a84-5a4a-11ec-b30a-e09467e33a05/roslaunch-hybtaletented-163-com-10904.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://hybtaletented-163-com:45577/
ros_comm version 1.14.12


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.12

NODES

auto-starting new master
process[master]: started with pid [10929]
ROS_MASTER_URI=http://hybtaletented-163-com:11311/

setting /run_id to af5e3a84-5a4a-11ec-b30a-e09467e33a05
process[rosout-1]: started with pid
```

# ROS节点查看
首先，通过rosnode list 命令可以查看到当前正在运行的ros节点
```shell
hybtalented@hybtaletented-163-com:~$ rosnode list
/rosout
```
上诉输出表示，目前只有一个rosout节点正在运行。再次输入 rosnode info /rosout 可以查看rosout的节点信息如下

```
--------------------------------------------------------------------------------
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [unknown type]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level


contacting node http://hybtaletented-163-com:41085/ ...
Pid: 10940
```
上诉消息表示 rosout 节点 发布了主题 /rosout_agg 其中消息类型为 rosgraph_msgs/Log；订阅了主题 /rosout；提供了两个服务，分别为 /rosout/get_loggers 和  /rosout/set_logger_level；可以通过 http://hybtaletented-163-com:41085/ 与节点进行通信，其进程id为 10940。


# 启动 ROS 节点
通过 `rosrun` 命令可以启动ROS节点
```shell
rosrun turtlesim turtlesim_node
```
上述命令表示启动 `turtlesim` 包中的  `turtlesim_node`节点，运行命令后将会看到一个包含一个海龟的窗口。
这是，在另一个终端输入 `rosnode list` 可以查看到节点列表如下所示
```
/rosout
/turtlesim
```
`rosrun` 可以通过重映射参数改变节点的名称如下
```shell
rosrun turtlesim turtlesim_node __name:=my_turtle
```
再次执行 `rosnode list`  可以看到节点名称已经改变。

```
/my_turtle
/rosout
/turtlesim
```
可以发现/turtlesim 节点并没有消失，这是因为本文中是通过 `Ctrl+C`强制结束节点进程，而不是通过关闭窗口的方式，这时可以通过 `rosnode cleanup`可以清理掉已经退出的节点的信息。

```shell
hybtalented@hybtaletented-163-com:~$ rosnode cleanup
ERROR: connection refused to [http://hybtaletented-163-com:42405/]
Unable to contact the following nodes:
 * /turtlesim
Warning: these might include alive and functioning nodes, e.g. in unstable networks.
Cleanup will purge all information about these nodes from the master.
Please type y or n to continue:
y
Unregistering /turtlesim
done
hybtalented@hybtaletented-163-com:~$ rosnode list
/my_turtle
/rosout
```


