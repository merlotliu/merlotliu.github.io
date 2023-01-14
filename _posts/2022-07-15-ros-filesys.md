---
layout: post
title: ROS 文件系统 & 工具
comments: true
date: 2022-07-15 15:45:12 
updated: 2022-07-15 15:45:12 
tags: [ROS]
categories: [ROS,beginner-tutorials]
---

# ROS 文件系统 & 命令行工具

## ROS 文件系統

ROS文件系统级指的是在硬盘上ROS源代码的组织形式，其结构大致可以如下图所示：

![img](../.gitbook/assets/ros-filesys.assets/%E6%96%87%E4%BB%B6%E7%B3%BB%E7%BB%9F.jpg)

```
WorkSpace --- 自定义的工作空间

    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。

    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。

    |--- src: 源码

        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成

            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件

            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)

            |-- scripts 存储python文件

            |-- src 存储C++源文件

            |-- include 头文件

            |-- msg 消息通信格式文件

            |-- srv 服务通信格式文件

            |-- action 动作格式文件

            |-- launch 可一次性运行多个节点 

            |-- config 配置信息

        |-- CMakeLists.txt: 编译的基本配置
        
```

### launch文件

`launch`文件實質也是`xml`文件

**需求**

> 一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。如果每次都调用 rosrun 逐一启动，显然效率低下，如何优化?

官方给出的优化策略是使用 launch 文件，可以一次性启动多个 ROS 节点。

**實現**

1.  在功能包下創建`launch`文件夾，通過在命令行輸入：

    ```shell
    mkdir launch
    ```
2.  在`launch`文件夾下，創建`launch`文件：

    ```shell
    vim LAUNCH_NAME.launch
    ```
3.  編輯`launch`文件内容

    ```xml
    <launch>
        <node pkg="turtlesim" type="turtlesim_node" name="turtle_gui" />
        <node pkg="turtlesim" type="turtle_teleop_key" name="turtle_ctl"/>
    </launch>
    ```

    node : 需要啓動的節點

    pkg : 功能包

    type : 被執行的節點文件

    name : 節點名稱

    **特别的**，使用`launch`文件会默认启动`roscore`，不需要额外启动。
4.  運行`launch`文件

    ```shell
    roslaunch PKG_NAME LAUNCH_NAME.launch
    ```
5. 運行結果： 原來需要分別啓動的roscore、烏龜GUI和鍵盤控制節點，能夠一次性全部啓動。

### package.xml

该文件定义有关软件包的属性，例如软件包名称，版本号，作者，维护者以及对其他catkin软件包的依赖性。请注意，该概念类似于旧版 rosbuild 构建系统中使用的_manifest.xml_文件。

```xml
<?xml version="1.0"?>
<!-- 格式: 以前是 1，推荐使用格式 2 -->
<package format="2">
  <!-- 包名 -->
  <name>demo01_hello_vscode</name>
  <!-- 版本 -->
  <version>0.0.0</version>
  <!-- 描述信息 -->
  <description>The demo01_hello_vscode package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <!-- 维护人员 -->
  <maintainer email="xuzuo@todo.todo">xuzuo</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <!-- 许可证信息，ROS核心组件默认 BSD -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/demo01_hello_vscode</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <!-- 依赖的构建工具，这是必须的 -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 指定构建此软件包所需的软件包 -->
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <!-- 指定根据这个包构建库所需要的包 -->
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <!-- 运行该程序包中的代码所需的程序包 -->  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

### CMakelists.txt

文件**CMakeLists.txt**是CMake构建系统的输入，用于构建软件包。任何兼容CMake的软件包都包含一个或多个CMakeLists.txt文件，这些文件描述了如何构建代码以及将代码安装到何处。

```
cmake_minimum_required(VERSION 3.0.2) #所需 cmake 版本
project(demo01_hello_vscode) #包名称，会被 ${PROJECT_NAME} 的方式调用

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
#默认添加系统依赖
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# 启动 python 模块支持
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
## 声明 ROS 消息、服务、动作... ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# 生成消息、服务时的依赖包
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
## 声明 ROS 动态参数配置 ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
## catkin 特定配置##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# 运行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo01_hello_vscode
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 声明 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo01_hello_vscode.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# 添加库的 cmake 目标依赖
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# 声明 C++ 可执行文件
add_executable(Hello_VSCode src/Hello_VSCode.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#重命名c++可执行文件
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#添加可执行文件的 cmake 目标依赖
add_dependencies(Hello_VSCode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#指定库、可执行文件的链接库
target_link_libraries(Hello_VSCode
  ${catkin_LIBRARIES}
)

#############
## Install ##
## 安装 ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#设置用于安装的可执行脚本
catkin_install_python(PROGRAMS
  scripts/Hi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo01_hello_vscode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

## ROS Tools

### 查找、安裝和刪除相關ROS功能包

**查找**

```shell
$ apt search ros-<distro>-<package>
```

**Notes**:`<distro>`為ROS版本號，`<package>`為ROS包名。此外，ROS所有包名的命名格式都是`ros-<distro>-<package>`，即`ros-版本號-包名`；

列出所有可用的`ROS package`：

```shell
$ apt search ros-kinetic-*
```

從中查找我們需要的包，如導航模塊中的`gmapping`:

```shell
$ apt search ros-kinetic-* | grep -i gmmaping
```

**安裝**

```shell
$ sudo apt install  ros-<distro>-<package>
```

**刪除**

```shell
$ sudo apt purge ros-<distro>-<package>
```

### roscore

`roscore`是節點運行和通信的必要條件:

* ros master；
* ros 参数服务器；
* rosout 日志节点；

用法:

```shell
$ roscore
```

或指定端口号（不常用）

```shell
$ roscore -p xxxx
```

![image-20220703203034222](../.gitbook/assets/ros-filesys.assets/image-20220703203034222.png)

### rosrun

用法:

```shell
$ rosrun <package> <node>
```

示例：

```shell
$ rosrun turtlesim turtlesim_node
```

### roslaunch

```shell
$ roslaunch <package> <launch>
```

### catkin\_create\_pkg

```shell
catkin_create_pkg <package> <dependencies>
```

### rospack

**作用**：獲取`packages`相關信息；

`rospack find`可以獲取`package`的路徑信息。下面將簡單演示`rospack find`的使用。

用法：

```shell
$ rospack find [package]
```

示例：

```shell
$ rospack find roscpp
```

將會返回以下信息：

```shell
ROS_INSTALL_PATH/share/roscpp
```

如果是在`Ubuntu`下安裝的`ROS Kinetic`，將返回以下信息：

```shell
/opt/ros/kinetic/share/roscpp
```

### roscd

**作用**：直接切換到`ROS package`的路徑下；

**ROS package 根目錄**

用法：

```shell
$ roscd <package-or-stack>[/subdir]
```

切換到`roscpp`包下：

```shell
$ roscd roscpp
```

爲了驗證確實切換到了`roscpp`下，輸入`pwd`打印當前路徑：

```shell
$ pwd
```

可以發現，這一路徑與`rospack find`給出的路徑是一樣的：

```shell
ROS_INSTALL_PATH/share/roscpp
```

**Notes**：與其他`ROS Tools`一樣，`roscd`只會查找`ROS_PACKAGE_PATH`中列出的目录中的 `ROS package`。輸入以下命令可查看當前`ROS_PACKAGE_PATH`内容：

```shell
$ echo $ROS_PACKAGE_PATH
```

在`Ubuntu`下安裝的`ROS Kinetic`通常能夠收到以下信息：

```shell
/opt/ros/kinetic/share
```

與其他環境變量一樣的是，可以向`ROS_PACKAGE_PATH`添加其他目錄，不同路徑用 `:` 分割。

**ROS package 子目錄**

示例：

```shell
$ roscd roscpp/cmake
$ pwd
```

返回以下信息：

```shell
ROS_INSTALL_PATH/share/roscpp/cmake
```

**roscd log**

`roscd log`將切換到ROS日志文件的存放路徑。

**Notes**：如果在此之前尚未運行過任何ROS節點，將返回錯誤信息并表示路徑不存在。

```shell
$ roscd log
```

### rosls

**作用**：通過`ROS package`名稱即可列出其子目錄，而不需要完整的路徑；

用法:

```shell
$ rosls <package-or-stack>[/subdir]
```

示例：

```shell
$ rosls roscpp_tutorials
```

收到以下信息：

```shell
cmake launch package.xml  srv
```

### Tab 補全（Tab-completion）

輸入完整的`package`名稱是一件冗長乏味的事情。在前面的例子中，`roscpp_tutorials`無疑是一個相當長的名字了。所幸的是，一些ROS工具支持Tab補全。

可以輸入以下内容：

```shell
$ roscd roscpp_tut<<< now push the TAB key >>>
```

然後按下`TAB`鍵，命令行就會補全剩下的部分：

```shell
$ roscd roscpp_tutorials/
```

當然，這是因爲在在當前`ROS package`中，僅有`roscpp_tutorials`以`roscpp_tut`開頭。

現在我們嘗試輸入以下内容：

```shell
$ roscd tur<<< now push the TAB key >>>
```

按下`TAB`鍵后，命令行内容可能將補充至以下内容：

```shell
$ roscd turtle
```

然而，當前有多個`packages`以`turtle`開頭。再次按下`TAB`鍵，將會輸出所有以`turtle`開頭的`ROS package`。

```shell
turtle_actionlib/  turtlesim/ turtle_tf/
```

然后，命令行仍然顯示如下内容：

```shell
$ roscd turtle
```

如果想要定位到`turtlesim/` ，則需要至少再繼續在`turtle`後面輸入一個`s`，然後按下`TAB`：

```shell
$ roscd turtles<<< now push the TAB key >>>
```

當僅有一個`package`以`turtles`開頭時, 將出現以下内容：

```shell
$ roscd turtlesim/
```

如果想看到當前所有的以安裝`packages`的列表，可以在`rosls`后，連續點擊兩次`TAB`：

```shell
$ rosls <<< now push the TAB key twice >>>
```

### rosed

需要安裝`vim`，用法：

```shell
$ rosed <package> <file>
```

示例：

```shell
$ rosed turtlesim Color.msg
```

### 幫助

當不明確`ROS Tools`使用規則時，可以在命令後鍵入`-h`獲得使用説明，如下：

```shell
$ rospack -h
```

命令行將顯示如下内容：

![image-20220703170334323](../.gitbook/assets/ros-filesys.assets/image-20220703170334323.png)

## Reference

1. [ROS/Tutorials/NavigatingTheFilesystem - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
2. [1.5 ROS架构 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/15-ben-zhang-xiao-jie.html)
