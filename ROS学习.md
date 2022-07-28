# ROS学习



# 自查

## ROS_INFO 调试时间戳

> 查看log信息： [rqt_console](https://blog.csdn.net/qq_30460905/article/details/123698106?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-123698106-blog-108450089.pc_relevant_multi_platform_whitelistv1&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-123698106-blog-108450089.pc_relevant_multi_platform_whitelistv1&utm_relevant_index=1) 
>
> 过滤log等级： [rqt_logger_level]
>
> [其他](https://zhuanlan.zhihu.com/p/536485585)
>
> ROS_INFO具有时间戳功能，其次，它总共包含五个记录级别，这样做的优点是可以确定在任何时间有选择的查看各种信息。 比如我们可以使用rqt_console检视所有INFO

**打印一次与定时打印**

ROS_INFO_STREAM_ONCE    放在回调函数中确认数据有接收到，但是又不想一直刷屏可以用这个

ROS_INFO_STREAM_THROTTLE(0.5, "Message print every 0.5s");   定时刷屏有时候回调的频率比较高，可以设置慢一点的打印

在ros程序运行时，默认是不显示debug信息的。如果要查看debug消息，需要先运行rosrun rqt_logger_level rqt_logger_level



## ros::Time::now()

> 该函数返回系统时间或仿真时间
>
> **ros:time::now()输出值为0？**
>
> ROS设置了一个模拟时钟的节点，使用模拟时钟的时候，now()返回时间0直到第一条消息在/clock已经收到，所以当客户端不知道时钟时间时ros:time::now()输出为0。
>
>  
>
> **ros:time::now()输出的值是什么？**
>
> ros:time::now()输出的值与参数use_sim_time有关。
>
> use_sim_time为true时，ros:time::now()输出系统时间；
>
> use_sim_time为false时，ros:time::now()输出输出仿真时间，如果回放bag则是bag的时间。

**怎么设置参数use_sim_time？**

launch文件设置：

```
<param name="use_sim_time" value="false" />
```

通过节点设置:

```
rosparam set use_sim_time true
```

 

**如何直接使用系统时间？**

使用 ros::WallTime::now() 

播放rosbag时，若参数/use_sim_time 为true，则此时

- ros::WallTime::now()为当前的真实时间，也就是墙上的挂钟时间，一直在走。
- ros::Time::now()为rosbag当时的时间，是由bag中/clock获取的。是仿真时间。





# 阅读团队代码

## 打印类的名字

> 我们如果想要在控制台输出某个类的名字，我们应该用什么命令？

在rm_common的serice_caller.h里我们可以找到这样一条命令

```
typeid(ServiceType).name()
```

[更多](https://www.bbsmax.com/A/kvJ3Xm7O5g/)

**t.name()**
返回类型的C-style字符串，类型名字用系统相关的方法产生。

ServiceType是通过template <class ServiceType>获取的类，如果这是个派生类，那么.name()并不会打印出基类名，而是当前类的名字

