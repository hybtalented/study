# rospack -- ros 包管理
* **rospack find [packageName]** 查找相应包的位置
* **rospack depends1 [packageName]**　查看包的直接依赖
* **rospack dependss [packageName]** 查看包的所有直接和间接依赖
# roscd -- 进入相应的包目录

ros 会从 `ROS_PACKAGE_PATH` 环境变量下的路径列表中查找相应的包，路径之间以 `:` 分隔。输入一下命令可以查看当前的ros包搜索路径
```shell
echo $ROS_PACKAGE_PATH
```

* **roscd [packageName]** 进入指定包的根目录，如 `roscd roscpp`
* **roscd [packageName]/subdirs** 进入指定包的相应子目录，如 `roscd roscpp/cmake`
* **roscd log** 进入ros的日志目录

# rosls -- 直接显示ros相应包的内容

* **rosls [packageName]** 显示指定包的根目录 `rosls roscpp_tutorials`
* **rosls [packageName]/subdirs** 显示指定包的子目录 `rosls roscpp_tutorials/cmake`

# ros 节点相关命令
1. **roscore** 启动ros核心节点
2. **rosnode list** 列出所有节点
3. **rosnode info [nodeName]** 显示ROS节点信息
4. **rosnode cleanup** 清理节点信息
5. **rosnode ping [nodeName]** 用于确定节点是否仍然存活
6. **rosrun [packageNode] [nodeName]** 运行节点

