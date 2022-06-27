<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-25
 * @LastEditors Youbiao He
 * @LastEditTime 2022-06-27
 * @FilePath /ros/tutorial/ros2ros2.md
 * @Description 
 * 
 * @Example 
-->
本文主要用于学习ros与ros2的不同，以及记录从 ros 升级到 ros2 的过程中可能碰到的问题。

# ROS1 和 ROS2 的区别
## 相关工具命令行的区别
在ros1中相关工具都是通过独立的命令行工具调用，例如 rosnode 查看节点信息， rosrun 运行节点, rostopic 查看 ros 消息的发布和订阅情况，而在 ros2 中， 所有的工具都一一个统一的 CLI 前端 `ros2`， 例如 `ros2 node`， `ros2 run`，`ros2 topic` 等。

此外，在 ros2 中引进 动作 （Action）机制，因此在 ros2 中添加了 `ros2 action` 命令查看 动作的调用情况。 