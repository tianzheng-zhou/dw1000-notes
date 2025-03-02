### 其中有些功能不知道是干什么用的 还需要详细了解
*虽然有了搜索的结果，但是还没办法理解这些东西*

LDE，"Linear Delay Estimator"，即线性延迟估计器。
LDO
LDO tune

filter

### 有些寄存器的功能并没有完全理解 或者说还没找到描述
reg:0x36:0x00 bit:23 KHZCKEN 其中kilohertz clock是什么？ emmm 看reg:0x36:0x04 bit:31-26 KHZCKDIV 这玩意似乎只是用来点亮LED灯的没啥用

### TODO
中断

### 关于多TAG

暂时是使用RANGING_REQUEST和GRANT这两种帧来实现的
需要修改的内容：
1. expectedmsgid
2. timer中默认发送的帧类型

初步搜索，决定使用**时分多址**的方案实行多TAG
并且使用主机动态调动的具体方案

这样就需要再原有通信协议上增加调动帧的数据类型

需要增加库文件的代码

很可能需要定义一个HOST的系统角色来统一管理

还有以下几种可能的方案（from deepseek）

#### ANCHOR本地队列轮询（无主机）

##### ANCHOR本地维护队列

``` cpp
// ANCHOR设备代码片段

DW1000RangingClass anchor;
Queue<String> tagQueue = {"TAG1", "TAG2"};

void loop() {
    if (!anchor.isBusy()) { // 当前无通信

        String nextTag = tagQueue.dequeue();
        anchor.startRanging(nextTag); // 启动与下一TAG的测距

        tagQueue.enqueue(nextTag);    // 重新入队循环

    }
}
```
##### 防冲突机制
随机退避：TAG在发送请求前等待随机时间。

``` cpp
// TAG代码片段

void sendPollRequest() {
    delay(random(0, MAX_BACKOFF_MS)); // 随机退避

    DW1000.newTransmit();
    DW1000.setData("POLL");
    DW1000.startTransmit();
}
```

#### 混合方案：请求-许可协议

##### 通信流程

TAG发送请求：TAG主动发送Ranging_Request。
ANCHOR分配许可：
收到请求的ANCHOR回复Grant消息，分配临时时隙。
其他ANCHOR通过监听信道避免冲突。

##### 消息格式示例

|消息类型|内容|
|---|---|
|Ranging_Request| [TAG_ID, ANCHOR_ID] |
|Grant| [Time_Slot, Duration] |

