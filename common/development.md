<!--
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-23
 * @LastEditors Youbiao He
 * @LastEditTime 2022-06-23
 * @FilePath /common/development.md
 * @Description 
 * 
 * @Example 
-->
# Ubuntu 下使用 easy connect的方法 （基于docker)
本文参考了 [docker-easyconnect](https://github.com/Hagb/docker-easyconnect)中的方法，并使用了该仓库中的 docker 镜像。 linux 下的 docker 的安装可以参考 [docker 的官方网站](https://docs.docker.com/desktop/linux/install/)

通过如下命令启动docker 容器， 其中 `username` 为 Easy Connect 的用户名， `password` 为 Easy Connect 的密码， `vpnaddress` 为 Easy Connect 服务器的地址。
```shell
docker run --name EasyConnect -d --device /dev/net/tun --cap-add NET_ADMIN -ti -p 127.0.0.1:1080:1080 -p 127.0.0.1:8888:8888 -e EC_VER=7.6.3 -e CLI_OPTS="-d vpnaddress -u username -p password" hagb/docker-easyconnect:cli
```
也可以使用如下的 `docker compose`文件来启动容器
```yml
version: "3"
services:
    EasyConnect:
        image: "hagb/docker-easyconnect:cli"
        ports: 
            - "127.0.0.1:1080:1080"
            - "127.0.0.1:8888:8888"
        cap_add:
            - NET_ADMIN
        environment:
            CLI_OPTS: "-d vpnaddress -u username -p password"
            EC_VER: "7.6.3" 
        devices:
            - "/dev/net/tun" 

```
容器创建完成后， 1080 端口将用嗯与 socks5 代理, 8888 端口用于 http 代理。 Ubuntu 中的代理配置可以参考 [Ubuntu配置全局系统代理](https://blog.csdn.net/kan2016/article/details/90411137)
