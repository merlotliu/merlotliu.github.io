---
title: ROS Service 通信简单示例
comments: true
date: 2022-07-18 13:49:27
updated: 2022-07-18 13:49:27
tags: [ROS,ROS-Service]
categories:
- [ROS,beginner-tutorials]
- [ROS,Service]
---

# ROS Service 通信简单示例

## 需求

在无人机任务执行的过程中，可能会遇到不可识别的单位，需要将采集到的数据，传输给数据处理节点，并获取有关信息。

在此，我们将实现client提交两个整数，server负责计算并返回给client。

## 服务端 Service Node

### 思路

1. 初始化 ROS 节点和句柄；
2. 创建服务并发布到 ROS；
3. 编写回调函数；

### cpp实现

创建 `add_two_ints_server.cpp` 文件：

```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```

### 代码解释

```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
```

`beginner_tutorials/AddTwoInts.h`是我们前面创建的`.srv`文件生成的头文件。

```cpp
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
```

这一方法为实现两个`int`相加服务。它携带两个参数，分别为`.srv`文件的 Request 和 Response 部分，并返回一个 boolean 值。

```cpp
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
```

两个整数相加并存储在 Response 部分。

打印请求响应相关信息。

最后完成服务并返回 `true`。

```cpp
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
```

这一语句将创建并发布服务信息。第一个参数是 `service`名字，第二个是回调函数名。

**Notes** :

- 转化长整型的原因：`beginner_tutorials::AddTwoIntsResponse_<std::allocator<void> >::_sum_type {aka long int}`

## 服务端 Client Node

### 思路

1. 初始化 ROS 节点、句柄；
2. 创建该服务的Client 对象；
3. 填写`srv`消息的`request`部分；
4. 调用服务；

### cpp实现

创建 `add_two_ints_client.cpp` 文件：

```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc < 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```

### 代码解释

```cpp
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
```

这将为 `add_two_ints` 服务创建一个客户端。`ros::Service Client` 对象用于稍后调用服务。

```cpp
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
```

在这里，我们实例化一个自动生成的服务类，并将值分配给它的成员变量 `request`。
一个服务类包含两个部分，请求`request`和响应`response`。它还包含两个类定义，请求`Request` 和响应 `Response`。

```cpp
  if (client.call(srv))
```

这是实际调用服务的地方。该服务调用是阻塞的，调用返程即返回。
如果服务调用成功， `call()` 将返回 true 并且 `srv.response` 中的值将是有效的。
如果调用不成功， `call()` 将返回 false 并且 `srv.response` 中的值将无效。

## 编译

### CMakeLists.txt

```
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_executable(add_two_ints_client src/add_two_ints_client.cpp)

add_dependencies(add_two_ints_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(add_two_ints_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(add_two_ints_server
  ${catkin_LIBRARIES}
)
target_link_libraries(add_two_ints_client
  ${catkin_LIBRARIES}
)
```

### catkin_make

```
# 切换到工作路径下
cd ~/catkin_ws
catkin_make
```

如果编译失败，很大的原因是因为前面创建`AddTwoInts.srv`的问题。

## 运行

运行 roscore

```
roscore
```

```
... logging to /u/takayama/.ros/logs/83871c9c-934b-11de-a451-
001d927076eb/roslaunch-ads-31831.log
... loading XML file 
[/wg/stor1a/rosbuild/shared_installation/ros/tools/roslaunch/roscore.xml]
Added core node of type [rosout/rosout] in namespace [/]
started roslaunch server http://ads:54367/

SUMMARY
======

NODES

changing ROS_MASTER_URI to [http://ads:11311/] for starting master locally
starting new master (master configured for auto start)
process[master]: started with pid [31874]
ROS_MASTER_URI=http://ads:11311/
setting /run_id to 83871c9c-934b-11de-a451-001d927076eb
+PARAM [/run_id] by /roslaunch
+PARAM [/roslaunch/uris/ads:54367] by /roslaunch
process[rosout-1]: started with pid [31889]
started core service [/rosout]
+SUB [/time] /rosout http://ads:33744/
+SERVICE [/rosout/get_loggers] /rosout http://ads:33744/
+SERVICE [/rosout/set_logger_level] /rosout http://ads:33744/
+PUB [/rosout_agg] /rosout http://ads:33744/
+SUB [/rosout] /rosout http://ads:33744/
```

### 运行 Server Node

打开新终端，设置环境变量，运行节点：

```shell
source ./devel/setup.bash
rosrun beginner_tutorials add_two_ints_server
```

```
Ready to add two ints.
```

### 运行 Client Node

```
source ./devel/setup.bash
rosrun beginner_tutorials add_two_ints_client 1 3
```

在client节点：

```
Sum: 4
```

在 server节点：

```
request: x=1, y=3
sending back response: [4]
```

## 优化

值得注意的是，在这里我们先启动了 server，后启动了client。如果先启动 client，服务会调用失败。

在这里我们可以使用`client.waitForExistence()`或`ros::service::waitForService("add_two_ints")`来进行程序的优化。即使client先启动也会一直等待服务的启动。

## launch文件启动

在 launch 文件夹下创建 server_client.launch 文件，输入：

```xml
<launch>
    <arg name="a" default="1"/>
    <arg name="b" default="3"/>
    <node pkg="beginner_tutorials" type="add_two_ints_server" name="add_two_ints_server" output="screen" />
    <node pkg="beginner_tutorials" type="add_two_ints_client" name="add_two_ints_client" args="$(arg a) $(arg b)" output="screen" />
</launch>
```

**Notes**：`args`是命令行参数，即 `add_two_ints_client a b`。事实上使用 launch 文件启动，后面还会有两个参数`__name:=xxx`、`__logs:=xxx`。

打开命令行，输入执行：

```shell
roslaunch beginner_tutorials server_client.launch 
```

## Reference 

1. [ROS/Tutorials/WritingServiceClient(c++) - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B))
