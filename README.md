## DW1000 NOTES

这是作者在学习DW1000时的中英混合笔记，主要是在DW1000库中添加更多注释

原作者地址：https://github.com/thotro/arduino-dw1000

原作者都好久没更新了啊喂，听说他好像某段时间突然从互联网上消失了，具体我也不知道怎么回事

不得不说，原作者水平是真的高

### 一些外部笔记

在src文件夹中，有一些笔记：

注意事项.md

TODO.md

内容顾名思义，包含了作者不少心血

### 测试平台

使用arduino框架，在PlatformIO中编译

主控芯片：ESP32-C3 supermini

测试临时接线图在主文件夹中，详见“DW1000测试临时接线图.pdf”

### 关于DW1000芯片

我拿dw1000主要是想要做一个多ANCHOR和多TAG的空间定位系统，但是如果设备数量变多，需要调整的代码的内容急剧增加

### 作者哔哔

这位作者不知天高地厚，竟试图给DW1000库添加多TAG的功能，最终不得不放弃，望周知。
