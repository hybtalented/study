<!--
 * @Author hybtalented hybtalented@163.com
 * @Date 2022-09-29
 * @LastEditors hybtalented
 * @LastEditTime 2022-09-30
 * @FilePath /docker/npm_registry/README.md
 * @Description 
-->
# npm 镜像镜像搭建
1. 拉取 verdaccio docker 镜像
    ```shell
    docker pull verdaccio/verdaccio
    ```
2. 编写配置文件
    verdaccio 的默认配置文件可以参考 [配置](./verdaccio/conf/config.yaml)，下面则提供了一个简化版的配置
    ```yaml
    # npm 包的存储路径
    storage: ../storage
    auth:
      htpasswd:
        file: ./htpasswd
        # Maximum amount of users allowed to register, defaults to "+inf".
        # You can set this to -1 to disable registration.
        max_users: 1000
    uplinks:
      npmjs:
        url: https://registry.npmmirror.com/
    listen: 0.0.0.0:4873
    ```
3. 启动镜像
    将上述的 verdaccio 配置文件保存为 `config.yaml` 并放在 `verdaccio/conf` 目录下，然后新建 `docker-compose.yml` 并输入如下的内容
    ```yaml
    version: '3'
    services:
      npm_registry:
        container_name: verdaccio
        image: S249:8081/verdaccio
        volumes:
          - ./verdaccio:/verdaccio
          - ./verdaccio/storage:/verdaccio/storage/
          - ./verdaccio/storage/data:/verdaccio/storage/data
        ports:
          - 8087:4873
    ```
    然后执行如下命令启动 docker 容器
    ```shell
    VERDACCIO_ROOT=<path to verdaccio root>
    mkdir -p $VERDACCIO_ROOT/verdaccio/storage/data
    chmod -R 777 $VERDACCIO_ROOT/verdaccio
    # 通过 docker-compose 启动
    docker-compose up
    # 或者直接使用docker启动
    docker run -itd --add-host registry.npmmirror.com:124.239.227.228 --add-host cdn.npmmirror.com:60.221.72.239 -v $VERDACCIO_ROOT/verdaccio:/verdaccio/ -v $VERDACCIO_ROOT/verdaccio/storage:/verdaccio/storage/ -v $VERDACCIO_ROOT/verdaccio/storage/data:/verdaccio/storage/data -p 8087:4873 --name verdaccio S249:8081/verdaccio
    ```
4. 用户注册
    verdaccio 默认开放用户注册。  在客户机终端中输入上述 npm 命令，然后输入对应的用户名，密码和邮箱即可完成用户注册
    ```shell
    npm adduser --registry http://localhost:8087
    ```
5. npm 包的发布
  
