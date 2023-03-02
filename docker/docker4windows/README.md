<!--
 * @Author hybtalented hybtalented@163.com
 * @Date 2022-09-29
 * @LastEditors hybtalented
 * @LastEditTime 2022-09-29
 * @FilePath /docker/docker4windows/README.md
 * @Description 
-->
优化的 docker for windows 配置，不会导致镜像创建失败
```json
{
  "builder": {
    "gc": {
      "defaultKeepStorage": "20GB",
      "enabled": true
    }
  },
  "debug": true,
  "experimental": true,
  "features": {
    "buildkit": true
  },
  "insecure-registries": [
    "http://172.20.33.249:8081",
    "http://S249:8081"
  ],
  "registry-mirrors": [
    "https://hub-mirror.c.163.com/"
  ]
}
```