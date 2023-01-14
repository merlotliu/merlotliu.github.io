---
layout: post
title: ROS Topic 通信简单示例
comments: true
date: 2022-07-16 15:07:50
updated: 2022-07-16 15:07:50
tags: [ROS,ROS-Topic]
categories: [ROS,beginner-tutorials]
---

# ROS Topic 通信简单示例

## 需求

正如前面所提到的，我们有时需要可视化激光扫描数据。需要将数据从激光测距仪传输到 rviz ，为了控制 Hokuyo 激光测距仪，我们启动 hokuyo_node 节点，该节点与激光对话并在扫描主题上发布 sensor_msgs/Laser Scan 消息。为了可视化激光扫描数据，我们启动 rviz 节点并订阅扫描主题。订阅后，rviz 节点开始接收激光扫描消息，并将其呈现到屏幕上。

对该示例简化后，让发布方以 10 hz的频率发布文本消息，订阅方接收消息并打印。

## 发布节点 Publisher Node

### 思路

1. 初始化 ROS 节点和句柄；
2. 告诉 master 节点，将创建一个使用 std_msgs/String 消息的话题 chatter；
3. 以 10 hz 的频率循环发布消息；

### cpp实现

在**功能包**的`src`目录下创建`talker.cpp`文件，编写以下内容：

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        
        std::stringstream ss;
        ss << "hello world " << count++;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        
        chatter_pub.publish(msg);

        ros::spinOnce();

	    loop_rate.sleep();
  }
  return 0;
}
```

### 代码解释

```cpp
#include "ros/ros.h"
```

`ros/ros.h`中包含了 ROS 最常用的头文件。

```cpp
#include "std_msgs/String.h"
```

包含 [std_msgs](http://wiki.ros.org/std_msgs) 包中的 [std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) 消息。实际上，`std_msgs/String`只包含一个类型为`std::string`的变量`data`。关于消息的更多定义，可以参见 [msg](http://wiki.ros.org/msg) 。

```cpp
ros::init(argc, argv, "talker");
```

初始化 ROS 节点。`ros::init`包含多个重载，对于当前使用的这个实现，第三个参数为节点的名字。因为传入了`argc`和`argv`，所以可以通过命令行或者`roslaunch`，对节点名称重映射（remapping）。此外：

- 节点名必须唯一；
- 名称必须是基础名字（ [base name](http://wiki.ros.org/Names#Graph)），比如，不能包含`/`；

```cpp
ros::NodeHandle n;
```

创建进程节点句柄。创建第一个节点句柄的时候，会执行节点初始化。析构最后一个节点句柄时，会清楚节点占用的资源（内存之类的）。

```cpp
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```

第一个参数是话题名称。这是告诉 `master`节点，我们将在主题 `chatter` 上发布  [std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) 类型的消息。这样做的目的是，让 `master`节点告诉任何订阅`chatter`话题的节点，我们将在这个话题上发布数据。

第二个参数是发布队列长度。他的意思是，在丢弃旧的消息之前最多存储 1000（发布队列长度）条消息。通常是可以自己设置，具体根据实际情况，因为在嵌入式编程中，任何一点小小的内存都是很珍贵的。

`ros::NodeHandle::advertise()` 返回一个 `ros::Publisher` 对象，它的作用：

- 含有一个发布消息到刚才创建的话题`chatter`的方法`publish()`；
- 当程序运行到该对象作用范围之外，

```cpp
ros::Rate loop_rate(10);
```

`ros::Rate`用于设定我们需要的频率。他会根据上次`Rate::sleep()`的时间，自动推算休要休眠多长时间。在这里我们设定了 10 Hz 的频率。

```cpp
int count = 0;
while (ros::ok()) {
```

默认情况下，roscpp 将安装一个能够接受`Ctrl-C`的 SIGINT 处理程序，`ctrl+c`将导致 `ros::ok()` 返回 false。

`ros::ok()` 将在以下情况下返回 false：

- 收到 SIGINT (Ctrl-C)；

- 我们被另一个同名节点踢出网络；

- `ros::shutdown()` 已被应用程序的另一部分调用；

- 所有 `ros::Node` 句柄已被销毁；

一旦 `ros::ok()` 返回 false，所有 ROS 调用都会失败。

```cpp
std_msgs::String msg;

std::stringstream ss;
ss << "hello world " << count;
msg.data = ss.str();
```

在ROS中广播消息，使用的是 [msg file](http://wiki.ros.org/msg) 生成的消息类。ROS 内置了许多可使用的数据类型，目前我们在这里使用的是`String`消息类型，它仅含一个成员变量`data`。

```cpp
chatter_pub.publish(msg);
```

现在，我们可以准确的将消息广播给订阅该消息的节点。

```cpp
ROS_INFO("%s", msg.data.c_str());
```

`ROS_INFO` 是 ROS 中替代 `printf`/`cout` 的命令行输出方法。具体可以参见[rosconsole documentation](http://wiki.ros.org/rosconsole) 。

```cpp
ros::spinOnce();
```

在当前这个简单的程序中，调用 `ros::spinOnce()` 可以说是多余的，因为我们不用处理任何回调。当然，如果在这个程序中，有订阅相关的内容，并且没有使用`ros::spinOnce()` ，那么回调函数就不会调用。所以，添加这一行语句没有问题的。

```cpp
loop_rate.sleep();
```

使用`ros::Rate`调用`sleep()`的目的是，休眠一段时间以保证 10hz 的发布频率。

### 完整注释版

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * 在 ROS 中发布数据：
 *	1. 初始化 ROS 节点和句柄；
 * 	2. 告诉 master 节点，将创建一个使用 std_msgs/String 消息的话题 chatter；
 *	3. 以 10 hz 的频率循环发布消息；
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
```

## 订阅节点 Subscriber Node

### 思路

1. 初始化 ROS 节点和句柄；
2. 订阅 `chatter` 话题；
3. 等待消息到达；
4. 消息到达会调用回调函数，在回调函数编写消息处理代码。

### cpp实现

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
```



### 代码解释

订阅程序代码中一些代码和发布程序是一样的，在下面就不再赘述。

```cpp
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```

当 `chatter`话题的数据到达时，回调函数`chatterCallBack()`将会被调用。消息使用  [boost shared_ptr](http://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm) 传递，意味着可以根据需要存储，不用担心在下文被删除，也不用复制基础数据。

```cpp
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```

向 `master`订阅`chatter` 话题。当新消息到达时，ROS 将会调用`chatterCallback()`。

第一个参数仍然是话题名称。

第二个参数是接收队列长度，当数据发布的很快，而订阅方处理过慢时，最多存储该队列长度的数据。在这里长度为1000，也就是缓存超过1000，前面的旧数据将会丢弃。

`NodeHandle::subscribe()`返回`ros::Subscriber`对象，在取消订阅之前，都必须保持该对象存在。当该对象析构时，将自动取消订阅`chatter`话题。

ROS 提供了多个`NodeHandle::subscribe()` 版本，可以自定义回调函数、指定类成员函数或者任何 Boost 调用。[roscpp overview](http://wiki.ros.org/roscpp/Overview) 给出了更详细的解释。

```cpp
  ros::spin();
```

`ros::spin()`进入一个循环，当消息到来时，调用回调函数。当 `ros::ok()`为 `false` ，`ros::spin()`也会退出。即，`ctrl+c`、`ros::shutdown()`等情况也适用 `ros::spin()`。

ROS 还提供了其他处理调用的的方式。在 [roscpp_tutorials](http://wiki.ros.org/roscpp_tutorials) 中有相关演示程序。更多信息可以参见 [roscpp overview](http://wiki.ros.org/roscpp/Overview) 。

### 完整注释版

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
```

## 编译节点

### CMakeLists.txt

修改 [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt) 文件，找到下面语句修改，或者直接复制到文件末尾：

```txt
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp
target_link_libraries(listener ${catkin_LIBRARIES})
```

这将创建两个可执行文件，`talker` 和 `listener`，默认情况下它们会进入你的[开发空间](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space)的包目录，默认位于 `~/<workspace_name>/devel/lib/<package_name>`。

这一步时确保在使用之前生成功能包的消息头。如果您在 catkin 工作空间中使用来自其他包的消息，您还需要将依赖项添加到它们各自的生成目标中，因为 catkin 并行构建所有项目。

```
target_link_libraries(talker ${catkin_LIBRARIES})
```

我们可以直接调用`可执行文件`或者使用`rosrun`。

它们没有放在 '/bin' 中，因为在将软件包安装到系统时会污染 PATH。
如果您希望您的可执行文件在安装时位于 PATH 上，您可以设置安装目标，请参阅：[catkin/CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)

更多描述信息可以参见 [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt) 文件：[catkin/CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)

### catkin_make

修改完后，在命令行输入：

```shell
# 切换到工作路径下
cd ~/catkin_ws
catkin_make
```

## 运行

首先，确保`roscore`的启动

```shell
$ roscore
```

在调用`rosrun`运行节点之前，确保设置了环境变量`source `工作空间下的`setup.sh`：

```shell
# In your catkin workspace
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```

### Publisher

打开第一个终端，运行`talker`：

```
$ rosrun beginner_tutorials talker 
```

```
[INFO] [WallTime: 1314931831.774057] hello world 1314931831.77
[INFO] [WallTime: 1314931832.775497] hello world 1314931832.77
[INFO] [WallTime: 1314931833.778937] hello world 1314931833.78
[INFO] [WallTime: 1314931834.782059] hello world 1314931834.78
[INFO] [WallTime: 1314931835.784853] hello world 1314931835.78
[INFO] [WallTime: 1314931836.788106] hello world 1314931836.79
```

### Subscriber

`rosrun`运行`listener`：

```
$ rosrun beginner_tutorials listener
```

```
[INFO] [WallTime: 1314931969.258941] /listener_17657_1314931968795I heard hello world 1314931969.26
[INFO] [WallTime: 1314931970.262246] /listener_17657_1314931968795I heard hello world 1314931970.26
[INFO] [WallTime: 1314931971.266348] /listener_17657_1314931968795I heard hello world 1314931971.26
[INFO] [WallTime: 1314931972.270429] /listener_17657_1314931968795I heard hello world 1314931972.27
[INFO] [WallTime: 1314931973.274382] /listener_17657_1314931968795I heard hello world 1314931973.27
[INFO] [WallTime: 1314931974.277694] /listener_17657_1314931968795I heard hello world 1314931974.28
[INFO] [WallTime: 1314931975.283708] /listener_17657_1314931968795I heard hello world 1314931975.28
```



## Reference 

1. [ROS/Tutorials/WritingPublisherSubscriber(c++) - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained)

