# vue 数据双向绑定学习
## 介绍
vue的数据的双向绑定是通过发布订阅模式和数据劫持实现的，其主要实现代码在`/src/core/observer`目录下的`observer.js`(发布者 或 主题)、`dep.js`(基于依赖的调度中心)、`watcher.js`(订阅者 或 观察者)， 此外，`array.js`主要实现了被观察的数组的相关方法的劫持、`scheduler.js`用于异步观察者