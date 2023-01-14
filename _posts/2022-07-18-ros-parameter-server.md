---
title: ROS 参数服务器
comments: true
date: 2022-07-18 14:31:54
updated: 2022-07-18 14:31:54
tags: [ROS,ROS-ParameterServer]
categories:
- [ROS,beginner-tutorials]
- [ROS,ParameterServer]
---

# ROS 参数服务器

ROS 参数服务器可以存储字符串、整数、浮点数、布尔值、列表、字典、iso8601 日期和 base64 编码的数据。字典必须有字符串键。

roscpp 的参数 API 支持以上所有数据类型，并在字符串、整数、浮点数和布尔值的操作方面非常方便。使用 [XmlRpc::XmlRpcValue class](http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html) 完成对其他选项的支持。

`roscpp` 提供了两套不同的参数 API：存在于 `ros::param` 命名空间中的“bare”版本，以及通过 `ros::NodeΗandle` 接口调用的“handle”版本。
下面将针对每个操作说明这两个版本。

## 获取参数

从参数服务器获取值。两个版本都支持字符串、整数、双精度、布尔值和 [XmlRpc::XmlRpcValue](http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html) 值。如果参数不存在或类型不正确，则返回 false。同时，`roscpp`还提供了一个返回默认值的版本。

### ros::NodeHandle::getParam()

通过 `NodeHandle` 版本检索的参数是相对于该节点句柄`NodeHandle`的命名空间进行解析的。
有关详细信息，请参阅  [roscpp NodeHandles overview](http://wiki.ros.org/roscpp/Overview/NodeHandles)。

```cpp
ros::NodeHandle nh;
std::string global_name, relative_name, default_param;
if (nh.getParam("/global_name", global_name))
{
    // do something
}

if (nh.getParam("relative_name", relative_name))
{
    // do something
}

// Default value version
nh.param<std::string>("default_param", default_param, "default_value");
```

### ros::param::get()

通过“bare”版本检索的参数相对于节点的命名空间进行解析。

```cpp
std::string global_name, relative_name, default_param;
if (ros::param::get("/global_name", global_name))
{
	// do something
}

if (ros::param::get("relative_name", relative_name))
{
	// do something
}

// Default value version
ros::param::param<std::string>("default_param", default_param, "default_value");
```

### Cached Parameters

`ros::NodeHandle::getParamCached()`和 `ros::param::getCached()`提供参数数据的本地缓存。这些API会通知[参数服务器 Parameter Server](http://wiki.ros.org/Parameter Server) 该节点希望在参数更改时得到通知，并防止节点在后续调用中必须重新查找参数服务器的值。

缓存参数会显着提高速度（在第一次调用之后），但应谨慎使用以避免主机过载。在节点和主节点之间出现间歇性连接问题的情况下，缓存参数目前也不太可靠。

## 设置参数

两个版本同样都支持字符串、整数、双精度、布尔值和 [XmlRpc::XmlRpcValue](http://docs.ros.org/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html) 值。

### ros::NodeHandle::setParam()

```cpp
ros::NodeHandle nh;
nh.setParam("/global_param", 5);
nh.setParam("relative_param", "my_string");
nh.setParam("bool_param", false);
```

### ros::param::set()

```cpp
   1 ros::param::set("/global_param", 5);
   2 ros::param::set("relative_param", "my_string");
   3 ros::param::set("bool_param", false);
```

## 检查参数是否存在

### ros::NodeHandle::hasParam()

```cpp
ros::NodeHandle nh;
if (nh.hasParam("my_param"))
{
  ...
}
```

### ros::param::has()

```cpp
if (ros::param::has("my_param"))
{
  ...
}
```

## 删除参数

### ros::NodeHandle::deleteParam()

```cpp
ros::NodeHandle nh;
nh.deleteParam("my_param");
```

### ros::param::del()

```cpp
ros::param::del("my_param");
```

## 私有参数访问

访问私有参数的方式不同，具体取决于您使用的是“handle”还是“bare”接口。

### handle : ros::NodeHandle

在“handle”版本的API中，您必须创建一个新的 `ros::NodeHandle`，并将私有命名空间作为其命名空间：

```
ros::NodeHandle nh("~");
std::string param;
nh.getParam("private_name", param);
```

### bare : ros::param

在“bare”版本的API中，您可以使用用于描述它们的相同符号访问私有参数，例如：

[切换行号显示](http://wiki.ros.org/roscpp/Overview/Parameter Server#)

```
std::string param;
ros::param::get("~private_name", param);
```

## 搜索参数的键值

有时您想从最近的命名空间中获取参数。例如，如果您有一个“robot_name”参数，您只想从您的私有命名空间向上搜索，直到找到匹配的参数。同样，如果您有一组相机节点，您可能希望在共享命名空间中设置一些参数，但通过在私有（`~name`）命名空间中设置它们来覆盖其他参数。

**Note:** 为了有效地检索内容，您应该使用相对名称而不是 `/global` 和 `~private` 名称。

### ros::NodeHandle::searchParam()

```cpp
std::string key;
if (nh.searchParam("bar", key))
{
  std::string val;
  nh.getParam(key, val);
}
```

### ros::param::search()

```cpp
std::string key;
if (ros::param::search("bar", key))
{
  std::string val;
  ros::param::get(key, val);
}
```

## 检索参数名称列表

**ROS indigo**及其以后版本。

可以将现有参数名称列表作为字符串的 `std::vector` 获取。

### ros::NodeHandle::getParamNames()

```
// Create a ROS node handle
ros::NodeHandle nh;

std::vector<std::string> keys;
nh.getParamNames(keys)
```

### ros::param::search()

```
std::vector<std::string> keys;
ros::param::search(keys)
```

## 检索列表

**ROS groovy**及其以后版本。

您可以使用以下模板值类型获取和设置原语和字符串的列表和字典作为 `std::vector` 和 `std::map` 容器：

- bool
- int
- float
- double
- string

例如，您可以使用 `ros::NodeHandle::get Param` / `ros::NodeHandle::set Param` 接口或 `ros::param::get` / `ros::param::set` 接口获取 vectors 和 maps ：

```cpp
// Create a ROS node handle
ros::NodeHandle nh;

// Construct a map of strings
std::map<std::string,std::string> map_s, map_s2;
map_s["a"] = "foo";
map_s["b"] = "bar";
map_s["c"] = "baz";

// Set and get a map of strings
nh.setParam("my_string_map", map_s);
nh.getParam("my_string_map", map_s2);

// Sum a list of doubles from the parameter server
std::vector<double> my_double_list;
double sum = 0;
nh.getParam("my_double_list", my_double_list);
for(unsigned i=0; i < my_double_list.size(); i++) {
  sum += my_double_list[i];
}
```

在 ROS Fuerte 和更早版本上，参数服务器上的列表只能通过使用 Xml Rpc::Xml Rpc Value 类来检索，该类可以表示参数服务器上的任何类型。这在以后的 ROS 版本中仍然是有效的方法。

```cpp
XmlRpc::XmlRpcValue my_list;
nh.getParam("my_list", my_list);
ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

for (int32_t i = 0; i < my_list.size(); ++i) 
{
  ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  sum += static_cast<double>(my_list[i]);
}
```

## Reference 

1. [roscpp/Overview/Parameter Server - ROS Wiki](http://wiki.ros.org/roscpp/Overview/Parameter Server)
