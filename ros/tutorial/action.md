<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-25
 * @LastEditors Youbiao He
 * @LastEditTime 2022-06-25
 * @FilePath /ros/tutorial/action.md
 * @Description 
 * 
 * @Example 
-->
ros2 中引进了一种新的节点间通信的方式， 动作（Action）， ros 动作的主要作用是提供一种需要长时间处理的任务的机制。它由三个部分组成： 目标（Goal），反馈（Feedback）和结果(Result)。

动作的机制是基于服务的调用和主题的发布和订阅。动作的使用与服务类似，不同的是动作是可以取消的。下图为一个动作的工作过程， 首先，动作客户端发送一个目标服务调用请求到服务端，完成目标服务后,客户端再次发送一个结果服务调用请求，并监听反馈主题，在动作调用过程中，服务端将反馈不断传给客户端，直到动作完成后，将调用的结果通过结果服务响应给客户端。
![动作的工作机制示意图](./image/Action-SingleActionClient.gif)

# ros 动作的启动

这一小节，我们将以 `turtlesim` 包为例子，学习 ros 动作的启动。

打开命令行，启动 `turtlesim_node` 和 `turtle_teleop_key` 节点
```shell
# 在终端1中
ros2 run turtlesim turtlesim_node

# 在终端2中
ros2 run turtlesim turtle_teleop_key
```
在 `turtle_teleop_key` 所在的终端按下 `ERTDFGCVB` 9个按键可以观察到在 `turtlesim_node` 启动的窗口中乌龟将会被转到指定的方向，或者取消转动（F键）。

每一次我门按下对应按键的时候，一个目标将会被发送到 `turtlesim_node` 节点，`turtlesim_node` 开始对乌龟进行转动，转动完成后，在 `turtlesim_node`所在终端上我们可以看到如下的消息
```shell
[1656160199.167192577] [turtlesim]: Rotation goal completed successfully
```
如果在转动的过程中按下F键转动将会被取消, 终端上可以看到
```shell
[INFO] [1656160019.230415623] [turtlesim]: Rotation goal canceled
```
特别的是，不仅客户端可以取消掉一个动作，在服务端也可以对动作进行取消，例如，我们在乌龟转动的过错中按下一个不同方向的转动动作键，服务端首先会取消掉上一次的转动动作，这种情况叫做终止（`Abort`）目标。
```shell
[WARN] [1656160224.910799880] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

# ros 动作的查看
首先，查看 `/turtlesim` 节点的详情
```shell
$ ros2 node info /turtlesim 
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:界定
```
可以看到， `/turtlesim` 节点中有一个 `/turtle1/rotate_absolute` 的动作服务端，动作的类型是  `turtlesim/action/RotateAbsolute`。

对应的，在 `/teleop_turtle` 节点中有一个 `/turtle1/rotate_absolute` 的动作客户端：
```shell
$ ros2 node info /teleop_turtle 
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

通过 `ros2 action` 命令可以查看正在运行的动作, 例如通过
```shell
$ ros2 action list
/turtle1/rotate_absolute
```
可以列出所有的正在运行的动作服务; 通过 `ros2 action info` 可以查看动作服务的使用情况
```shell
$ ros2 action info /turtle1/rotate_absolute 
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
可以看到  ` /turtle1/rotate_absolute` 动作有一个客户端 `/teleop_turtle` 和一个服务端 `/turtlesim`。

此外通过在`ros2 action` 命令后加上 `-t` 选项， 我们还可以查看对应的动作的类型
```shell
$ ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
$ ros2 action info /turtle1/rotate_absolute -t
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle [turtlesim/action/RotateAbsolute]
Action servers: 1
    /turtlesim [turtlesim/action/RotateAbsolute]
```

通过 `ros2 action` 命令我们还可以手动发送一个目标给动作服务端，不过在如此做之前，我们需要先查看以下动作的类型
```shell
$ ros2 interface show turtlesim/action/RotateAbsolute 
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
一个动作的类型定义可以分为3个部分，并通过 `---` 分割，第一部分是目标的类型，第二部分是结果的类型，最后一个部分是反馈内容的类型。在上述的的动作定义文件中，目标类型为浮点数 theta 代表目标的朝向， 结果类型为浮点数 delta 代表旋转过的角度， 反馈类型为浮点数 remaining, 代表剩余旋转的角度。

下面，我们将通过 `ros2 action send_goal` 发送一个 `/turtle1/rotate_absolute` 动作, 结果如下
```shell
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
Waiting for an action server to become available...
Sending goal:
     theta: 1.57

Goal accepted with ID: 52c4221db9ea48b2871da723f2f58352

Result:
    delta: -2.2239999771118164

Goal finished with status: SUCCEEDED
```
如果我们在 `ros2 action send_goal` 命令后添加 `-f, --feedback` 选项， 可以查看动作的反馈消息
```
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.58}" --feedback 
Waiting for an action server to become available...
Sending goal:
     theta: 1.58

Feedback:
    remaining: -0.003999948501586914

Goal accepted with ID: af07f3778ac8405786ca8a330f3062fd

Result:
    delta: 0.0

Goal finished with status: SUCCEEDED
```