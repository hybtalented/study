# rospack -- ros 包管理

- **rospack find [packageName]** 查找相应包的位置
- **rospack depends1 [packageName]**　查看包的直接依赖
- **rospack dependss [packageName]** 查看包的所有直接和间接依赖

**ros 会从 `ROS_PACKAGE_PATH` 环境变量下的路径列表中查找相应的包，路径之间以 `:` 分隔。输入一下命令可以查看当前的 ros 包搜索路径**
# ros 文件和目录相关操作



```shell
echo $ROS_PACKAGE_PATH
```

- **roscd [packageName][?/subdirs]** 进入指定包的相应子目录
- **roscd log** 进入 ros 的日志目录
- **roscp [packageName] [filePath] [targetPath]** 拷贝对应包下的相应文件到指定目录
- **roscat [packageName] [fileName]** 显示对应包中指定文件的内容
- **rosls [packageName][?/subdirs]** 显示指定包的子目录下的内容


# rosed -- ros 相关文件的编辑

- **rosed [packageName] [fileName]** 编辑指定包中的一个文件
# ros 节点相关命令

- **roscore** 启动 ros 核心节点
- **rosnode list** 列出所有节点
- **rosnode info [nodeName]** 显示 ROS 节点信息
- **rosnode cleanup** 清理节点信息
- **rosnode ping [nodeName]** 用于确定节点是否仍然存活
- **rosrun [packageNode] [nodeName] [?ARGS]** 运行节点
- **roslaunch [packageName] [fileName.launch]** 根据指定包下的相应的launch文件启动一个或多个ros节点

# rostopic -- ros 主题相关操作

- **rostopic bw [topicName]** 显示 ros 主题使用的带宽
- **rostopic echo [topicName]** 打印指定的主题消息到屏幕
- **rostopic hz [topicName]** 显示主题的发布率
- **rostopic list** 显示所有正在使用的主题，加上`-v`选项可以显示主题的发布和订阅节点信息
- **rostopic pub [topicName] [type] [args...]** 发布指定类型的数据到主题
- **rostopic type [topicName]** 显示主题的类型

# rosmsg -- ros 消息的相关类型展示

- **rosmsg show [msgType]** 显示消息类型的详情
- **rosmsg info [msgType]** 显示消息别名
- **rosmsg list** 列出所有消息类型
- **rosmsg md5 [msgType]** 显示消息的 md5 和
- **rosmsg package [packageName]** 列出包使用的消息
- **rosmsg packages** 列出所有包含消息的包

# rosbag -- ros 消息记录和播放

- **rosbag record [...topics]** 记录指定主题的消息到指定的消息记录文件, 可以通过 `-O` 选项指定输出的消息记录文件的文件名
- **rosbag record -a** 记录所有的ros消息
- **rosbag info [fileName]** 输出ros消息记录文件的详情
- **rosbag play [...files]** 顺序播放指定消息记录文件中的消息, 可以通过 `-r` 指定播放速率, `-s` 选项在播放消息记录文件指定时间后的消息, `-d` 选项控制延时指定时间后播放消息, `--topics` 指定播放的消息主题.

# rosservice -- ros 服务相关操作

- **rosservice list** 列出所有的 ros 服务
- **rosservice call [serviceName] [...args]** 通过指定参数 args 调用服务
- **rosservice type [serviceName]** 显示指定服务的服务类型
- **rosservice find [serviceType]** 根据服务类型查找服务名称
- **rosservice uri [serviceName]** 查看指定服务的rosrpc服务地址

# rossrv -- ros 服务类型的相关信息展示

- **rossrv show [serviceType]** 显示 ros 服务的请求和相应类型详情
- **rossrv info [serviceType]** 显示 ros 服务的请求和相应类型别名
- **rossrv list** 列出所有的服务类型
- **rossrv md5** 显示服务类型的 md5 和
- **rossrv package [packageName]** 列出包中的所有服务类型
- **rossrv packages** 列出所有定义了服务的包

# rosparam -- ros 参数相关操作

- **rosparam set [paramName] [paramValue]** 设置参数的值
- **rosparam get [paramName]** 获取参数的值，如果参数名 paramName 为 /， 将会显示整个ros参数服务器的内容
- **rosparam load [fileName] [namespace]** 从指定文件名字读取 ros 参数到 ros 参数服务器的指定命名空间下
- **rosparam dump [fileName] [namespace]** 保存指定命名空间下的 ros 参数到对应文件名中，保存的格式为 yaml 格式
- **rosparam delete [paramName]** 删除对应参数
- **rosparam list** 列出所有的参数

