---
layout: post
title: Getting Start
date: 2022-12-20 21:10:00
updated: 2022-12-20 21:10:00
tags: [ROS]
categories: [ROS]
comments: true
---

# Getting Start

编写ROS的第一个程序，实现流程大致如下：

1. 创建工作空间；
2. 创建功能包；
3. 编写源文件；
4. 编辑配置文件；
5. 编译、执行；

### 1 创建工作空间并初始化

创建含有`src`目录的工作空间，切换到工作空间下，`catkin_make`编译完成初始化：

```shell
# create workspace folder
mkdir -p WORKSPACE_NAME/src
# 切换到工作空间下
cd WORKSPACE_NAME/
# 编译 可以视为初始化工作空间
catkin_make
```

### 2 创建功能包并添加依赖

切换到工作空间的`src`目录下，使用`catkin_create_pkg`创建功能包并添加依赖：

```shell
# 切换到工作空间下的src
cd WORKSPACE_NAME/src
# 创建功能包
catkin_create_pkg PKG_NAME roscpp rospy std_msgs
```

该功能包添加的依赖为`roscpp`、`rospy`和`std_msgs`，`roscpp`表示使用C++实现的库，`rospy`表示使用Python实现的库，`std_msgs`为标准消息库。功能包通常都会依赖这三个库实现。

### 3 编写源文件

在ROS中，虽然实现同一功能时，C++和Python可以互换，但是具体选择哪种语言，需要视需求而定，因为两种语言相较而言:C++运行效率高但是编码效率低，而Python则反之，基于二者互补的特点，ROS设计者分别设计了`roscpp`与`rospy`库，前者旨在成为ROS的高性能库，而后者则一般用于对性能无要求的场景，旨在提高开发效率。

#### C++实现

使用`vim`创建并打开`cpp`文件：

```shell
# 在功能包的src目录下创建C++文件并打开
vim WORKSPACE_NAME/src/PKG_NAME/src/hello_world.cpp
```

编辑相关源代码：

```shell
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	// ros node init
	// arg3 为 node 节点名
	ros::init(argc, argv, "hello_world_node");
	// print "hello world"
	ROS_INFO("hello world!");
	
	return 0;
}
```

#### Python实现

使用`vim`创建并打开`python`文件：

```shell
# 在功能包目录下创建scripts文件夹
mkdir WORKSPACE_NAME/src/PKG_NAME/scripts/
# 在功能包的scripts目录下创建C++文件并打开
vim WORKSPACE_NAME/src/PKG_NAME/scripts/hello_world.py
```

编辑相关源代码：

```python
#! /usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("hello_world_node_py")
    rospy.loginfo("Hello World! --python")
```

为`python`文件添加可执行权限：

```shell
chmod +x PYTHON_NAME.py
```

### 4 编辑功能包下的`CMakelist.txt`文件

#### C++

```shell
# hello_world 为可执行程序的名字，可随意命名，一般为源文件名去掉后缀
add_executable(hello_world_exe
	src/hello_world.cpp
)

# hello_world 为上面可执行程序的名字
target_link_libearies(hello_world_exe
	${catkin_LIBRARIES}
)
```

#### Python

```shell
catkin_install_python(PROGRAMS scripts/PYTHON_NAME.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 5 编译并执行

切换进入工作空间，使用`catkin_make`编译

```shell
# 切换进入工作空间
cd WORKSPACE_NAME/
# 编译
catkin_make
```

执行，需要打开两个命令窗口：

第一个命令窗口，输入

```shell
# 启动 ROS Master 节点
roscore
```

再开一个窗口，输入

```shell
# 切换到工作空间下
cd WORKSPACE_NAME/
# 设置环境变量
source ./devel/setup.bash

# Cpp文件
# 启动 ROS 节点
# PKG_NAME 为功能包名
# CPP_NODE_NAME 为 ROS 节点名，即可执行程序的名字（hello_world）
rosrun PKG_NAME CPP_EXE_NAME

# 或 Python 文件
rosrun PKG_NAME PYTHON_NAME.py
```

输入完，回车就能在命令行看见输出：**Hello World ！**

Tips :&#x20;

`source ~/工作空间/devel/setup.bash`可以添加进`.bashrc`文件，使用上更方便

添加方式1 : 直接使用 gedit 或 vi 编辑 .bashrc 文件，最后添加该内容

添加方式2 : `echo "source ~/工作空间/devel/setup.bash" >> ~/.bashrc`

### Reference

1. [1.3.1 HelloWorld实现简介 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/13-rosji-cheng-kai-fa-huan-jing-da-jian/131-helloworldshi-xian-jian-jie.html)
