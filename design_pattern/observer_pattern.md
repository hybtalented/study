# 观察者模式
观察者模式主要用于对象存在一对多关系时使用，当一个主题被触发时，会通知所有订阅这个主题的观察者。

## 设置思路

主要设计思路如下所示: 
``` mermaid
graph LR
   sub(主题) -- 主题更新通知 --> ob1(观察者1)
   sub(主题) -- 主题更新通知 --> ob2(观察者2)
   ob1 --  订阅主题 --> sub
   ob2 --  订阅主题 --> sub
```
观察者订阅(`subscribe`)一个主题（将自己添加到对应主题的观察者列表中），当主题更新后通知(`notify`)所有订阅改主题的观察者(调用观察者对应的`update`函数)

## 示例
* 主题
    ``` typescript
    class Subject {
        private observers: Set<Observer>;
        private _state: any;
        get state() {
            return this._state;
        }
        set state(new_state) {
            this._state = new_state;
            this.notify();
        }
        notify() { 
            this.observers.forEach(observer => observer.update(newState))
        }
        attach(observer: Observer) {
            if(!this.observers.has(observer)) {
               this.observers.add(observer)
            }
        }
        detach(observer: Observer) {
            if(this.observers.has(observer)) {
               this.observers.delete(observer)
            }
        }
    }
    ```
* 观察者
   ``` typescript
    class Observer {
        private subject: Subject;
        constructor(subject: Subject) {
           this.subject = subject;
           this.subscribe();
        }
        update() {
          console.log(this.subject.state);
        }
        subscribe() {
            this.subject.attach(this);
        }
        unsubscribe() {
            this.subject.detach(this);
        }
    }

<style>
.mume .node,.label {
    font-size: 13px;
}
</style>

## 使用情景
1. 插件调用
2. 中间件调用