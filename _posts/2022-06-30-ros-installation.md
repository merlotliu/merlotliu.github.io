---
layout: post
title: ROS Installation
date: 2022-06-30 21:10:00
updated: 2022-06-30 21:10:00
tags: [ROS,Ubuntu,VMware]
categories: [ROS]
comments: true 
---

## 前言

目前，ROS 1 的最终版 Noetic 已完成发布。推荐安装的版本为 [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation) 和 [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation) 。对应 ROS 版本及OS可参考 [Distributions - ROS Wiki](http://wiki.ros.org/Distributions)。

我们使用的是 ROS 版本是 **Kinetic**，以下将给出 Kinetic 的安装教程，其他版本类似。下面内容是官方给出的对于**Kinetic**支持平台的说明，大致意思就是需要在**ubuntu16.04**上安装。

> ROS Kinetic Kame is primarily targeted at the Ubuntu 16.04 (Xenial) release, though other Linux systems as well as Mac OS X, Android, and Windows are supported to varying degrees. For more information on compatibility on other platforms, please see REP 3: Target Platforms. It will also support Ubuntu 15.10 Wily and Debian Jessie.

ubuntu安装常用方式有两种:



**方案**1：安装虚拟机（可以安装**VMware**或**VirtualBox**）

在这一方案中，我们按照先后顺序依次安装**VMware 16**、**Ubuntu 16**和，大致流程如下:

1. 安装虚拟机软件(比如：virtualbox 或 VMware)；
2. 使用虚拟机软件虚拟一台主机；
3. 在虚拟主机上安装 ubuntu 16.04；
4. 在 ubuntu 上安装 ROS Kinetic Kame；
5. 测试 ROS Kinetic Kame环境是否可以正常运行。



**方案**2：安装双系统（在基础学习阶段，双系统安装没有必要，本文暂不予介绍）；



两种方式比较，各有优缺点：

- 方案1可以方便的实现 windows 与 ubuntu 交互，不过性能稍差，且与硬件交互不便；

- 方案2可以保证性能，且不需要考虑硬件兼容性问题，但是和windows系统交互不便。

在 ROS 中，一些仿真操作是比较耗费系统资源的，且经常需要和一些硬件(雷达、摄像头、imu、STM32、arduino....)交互，因此，原则上建议采用方案2，不过如果只是出于学习目的，那么方案1也基本够用，且方案1在windows与ubuntu的交互上更为方便，对于学习者更为友好。



## VMware Workstation 16 Pro 安装

### VMware下载

点击下方链接下载

官方网址：[Download VMware Workstation Pro](https://www.vmware.com/products/workstation-pro/workstation-pro-evaluation.html)

![image-20220624161947876](../../.gitbook/assets/ros-installation.assets/image-20220624161947876.png)

### VMware安装

下载完成后，双击 **.exe**  文件运行，点击**下一步**

![image-20220624161041508](../../.gitbook/assets/ros-installation.assets/image-20220624161041508.png)

直到出现如下界面，将**两者勾选**，**增强型键盘驱动程序** ，可更好地处理国际键盘和带有额外按键的键盘。**更改安装位置**，默认为C盘，建议更改到其他盘符，且路径不含中文，点击下一步。

![image-20220624161433305](../../.gitbook/assets/ros-installation.assets/image-20220624161433305.png)

通常我都会取消更新计划，因为官方更新可能出现兼容性等问题，问题不大，可以根据个人喜好决定，然后下一步。

![image-20220624161803068](../../.gitbook/assets/ros-installation.assets/image-20220624161803068.png)

一直点击下一步，直到安装完成。

![image-20220624161830994](../../.gitbook/assets/ros-installation.assets/image-20220624161830994.png)

安装成功后点击 **许可证** 输入密钥激活软件。

密钥查询：[求vmware workstation 16激活密钥_百度知道 ](https://zhidao.baidu.com/question/432413156276109052.html)

![image-20220624162101220](../../.gitbook/assets/ros-installation.assets/image-20220624162101220.png)

重启电脑，安装完成

![image-20220624163430664](../../.gitbook/assets/ros-installation.assets/image-20220624163430664.png)

## Ubuntu 16.04 安装

### 一、镜像下载

Ubuntu镜像：[Index of /ubuntu-releases/16.04/ (163.com)](http://mirrors.163.com/ubuntu-releases/16.04/)

建议选后一个

![image-20220624172147792](../../.gitbook/assets/ros-installation.assets/image-20220624172147792.png)

### 二、Ubuntu安装教程

#### 1. 创建新虚拟机，加载iso文件

**创建新的虚拟机**，选择**自定义（高级）**，点击下一步

![image-20220624205807301](../../.gitbook/assets/ros-installation.assets/image-20220624205807301.png)

选择**Workstation 16.x**，点击下一步

![image-20220624210717494](../../.gitbook/assets/ros-installation.assets/image-20220624210717494.png)

选择**稍后安装操作系统**，点击下一步

![image-20220624212921852](../../.gitbook/assets/ros-installation.assets/image-20220624212921852.png)

客户机操作系统选择**Linux**，版本**Ubuntu 64位**（因为下载的是64位操作系统，如果是32位，选择Ubuntu），点击下一步

![image-20220624212941856](../../.gitbook/assets/ros-installation.assets/image-20220624212941856.png)

输入**虚拟机名称**和**安装位置**，点击下一步

![image-20220624213000558](../../.gitbook/assets/ros-installation.assets/image-20220624213000558.png)

**处理器配置**，按照个人电脑情况配置即可（会实际占用CPU），这里我们分配 **2** 个就可以了，点击下一步

![image-20220624213033625](../../.gitbook/assets/ros-installation.assets/image-20220624213033625.png)

**虚拟机内存分配**，同样根据个人电脑分配即可，在此我们分配**4GB**，点击下一步

![image-20220624213044824](../../.gitbook/assets/ros-installation.assets/image-20220624213044824.png)

**网络类型**的网络连接选择**NAT（网络地址转换）**，点击下一步

![image-20220624213056954](../../.gitbook/assets/ros-installation.assets/image-20220624213056954.png)

**连接I/O控制器类型**，选择默认即可，点击下一步

![image-20220624213105948](../../.gitbook/assets/ros-installation.assets/image-20220624213105948.png)

**选择磁盘类型**，选择默认即可，点击下一步

![image-20220624213118281](../../.gitbook/assets/ros-installation.assets/image-20220624213118281.png)

**选择磁盘**，选择默认即可，点击下一步

![image-20220624213352047](../../.gitbook/assets/ros-installation.assets/image-20220624213352047.png)

**指定磁盘容量**，设定最大磁盘大小，由于会占用电脑实际存储空间且ROS会占用较多空间，尽量设置**60GB以上**，同时选择**将虚拟磁盘存储为的单个文件**，点击下一步

![image-20220624213735150](../../.gitbook/assets/ros-installation.assets/image-20220624213735150.png)

**指定磁盘文件**，选择磁盘存储的位置

![image-20220625135537431](../../.gitbook/assets/ros-installation.assets/image-20220625135537431.png)

选择**自定义硬件**，选择**系统镜像路径**。

![image-20220625135841860](../../.gitbook/assets/ros-installation.assets/image-20220625135841860.png)

点击**新CD/DVD(SATA)**，选择**使用ISO映像文件**，找到文件路径，关闭，完成

![image-20220625140048855](../../.gitbook/assets/ros-installation.assets/image-20220625140048855.png)

**开启此虚拟机**

![image-20220625140358583](../../.gitbook/assets/ros-installation.assets/image-20220625140358583.png)

#### 2. Ubuntu安装

语言选择，建立选择**English**，为了方便阅读也可以选择**简体中文**，然后点击**Install Ubuntu**

![image-20220625140625359](../../.gitbook/assets/ros-installation.assets/image-20220625140625359.png)

安装准备，建议不勾选，点击**continue**

![image-20220625140845690](../../.gitbook/assets/ros-installation.assets/image-20220625140845690.png)

安装类型，选择**第一个选项即可**，点击**continue**

![image-20220625141302318](../../.gitbook/assets/ros-installation.assets/image-20220625141302318.png)

点击**continue**

![image-20220625141324509](../../.gitbook/assets/ros-installation.assets/image-20220625141324509.png)

选择**上海**，点击**continue**

![image-20220625141433237](../../.gitbook/assets/ros-installation.assets/image-20220625141433237.png)

键盘布局，默认选择即可，点击**continue**

![image-20220625141511636](../../.gitbook/assets/ros-installation.assets/image-20220625141511636.png)

输入相关内容，随便输入，密码尽量短，方便使用（因为Linux很多操作需要管理员权限，需要输入密码）

![image-20220625141818057](../../.gitbook/assets/ros-installation.assets/image-20220625141818057.png)

等待安装完成即可

![image-20220625141903126](../../.gitbook/assets/ros-installation.assets/image-20220625141903126.png)

安装完成，重启

![image-20220625143831523](../../.gitbook/assets/ros-installation.assets/image-20220625143831523.png)

重启完成

![image-20220625144704473](../../.gitbook/assets/ros-installation.assets/image-20220625144704473.png)

#### 3. 安装VMware Tools

**虚拟机** ----> **安装VMware Tools**

![image-20220625145255202](../../.gitbook/assets/ros-installation.assets/image-20220625145255202.png)

定位到压缩包位置

![image-20220625145828863](../../.gitbook/assets/ros-installation.assets/image-20220625145828863.png)

使用 ` cp 包名 ~/Downloads ` 拷贝到家目录下

![image-20220625150923223](../../.gitbook/assets/ros-installation.assets/image-20220625150923223.png)

` cd ~/Downloads ` 切换到压缩包所在路径

![image-20220625151007172](../../.gitbook/assets/ros-installation.assets/image-20220625151007172.png)

 `tar -xvf 安装包名` 解压

![image-20220625151040604](../../.gitbook/assets/ros-installation.assets/image-20220625151040604.png)

解压完成，`cd vmware-tools-distrib `切换到解压完成的目录下，`ls`查看当前目录，找到`vmware-install-pl`**(绿色表示已经有可执行权限了**，如果没有通过下一步添加)

![image-20220625151420270](../../.gitbook/assets/ros-installation.assets/image-20220625151420270.png)

给`vmware-install-pl`，添加可执行权限，使用命令`sudo chmod +x vmware-install-pl`，输入密码

![image-20220625151520092](../../.gitbook/assets/ros-installation.assets/image-20220625151520092.png)

执行`vmware-install-pl`，直接执行，权限不够，需要添加`sudo`

![image-20220625151825604](../../.gitbook/assets/ros-installation.assets/image-20220625151825604.png)

第一个问题是，是否继续安装，输入`y`表示同意，其他问题回车选择默认输入即可，直到安装完成

![image-20220625152016944](../../.gitbook/assets/ros-installation.assets/image-20220625152016944.png)

安装完成后：

- 屏幕会自适应窗口大小；
- 支持与windows复制粘贴、文件拖进拖出的操作；

![image-20220625152214865](../../.gitbook/assets/ros-installation.assets/image-20220625152214865.png)

#### 4. 软件源配置

打开设置`System Settings`----`Software & Updates` ---- `Download from` ---- `Select Best Server` ，等待测试完成，选择最佳源，然后需要更新软件源，系统可能会弹出窗口提示，或者命令窗口输入`sudo apt-get update`也可。

![image-20220625154100908](../../.gitbook/assets/ros-installation.assets/image-20220625154100908.png)

#### 5. 安装中文输入法

##### 安装fcitx

打开设置`System Settings`，点击`Language Support`，确认`install`，这里主要就是安装`fcitx`，其实也可以直接输入`sudo apt-get install fcitx`安装

![image-20220625154405190](../../.gitbook/assets/ros-installation.assets/image-20220625154405190.png)

重启电脑，`kygboard input method system`切换为`fcitx`

![image-20220625155405433](../../.gitbook/assets/ros-installation.assets/image-20220625155405433.png)

##### 安装googlepinyin

1. 输入`sudo apt install fcitx-googlepinyin -y`；

2. 修复依赖关系：`sudo apt-get install -f`；

3. 重启完成安装；

`win+a`打开所有应用，找到`Fcitx Configuration`,点击打开

![image-20220625160003361](../../.gitbook/assets/ros-installation.assets/image-20220625160003361.png)

添加新输入法

![image-20220625160124270](../../.gitbook/assets/ros-installation.assets/image-20220625160124270.png)

取消勾选

![image-20220625160223105](../../.gitbook/assets/ros-installation.assets/image-20220625160223105.png)

输入`Pinyin`，添加所需要的输入法（其实也可以不下载Google Pinyin，系统自带其他两个，只不过不太好用罢了）

![image-20220625160309679](../../.gitbook/assets/ros-installation.assets/image-20220625160309679.png)

切换输入法`ctrl+space`，修改成其他键的方法自行`google`

##### 卸载googlepinyin

```shell
sudo apt-get purge fcitx-googlepinyin
```

## ROS Kinetic 安装

[原文博客(Ubuntu16安装ROS Kinetic Kame（内含Gazebo）)]([(69条消息) Ubuntu16安装ROS Kinetic Kame（内含Gazebo）_MonroeLiu的博客-CSDN博客](https://blog.csdn.net/m0_46216098/article/details/123764190?spm=1001.2014.3001.5502))

### 一、Kinetic 安装

#### 1. 添加下载源

官方默认安装源:

~~~shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
~~~

国内清华的安装源

```shell
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

国内中科大的安装源

```shell
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

PS : 建议使用国内资源，安装速度更快。

#### 2. 设置密钥

~~~shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
~~~

#### 3. 更新源

```shell
sudo apt-get update
```

#### 4. 安装完整安装包（时间较长耐心等待…）

```shell
sudo apt-get install ros-kinetic-desktop-full -y
```

#### 错误及解决

##### 错误提示

```shell
E: Failed to fetch http://packages.ros.org/ros/ubuntu/pool/main/p/pyside2/python-pyside2.qthelp_2.0.0+dev1-1~202102130210~rev1858~pkg5~git131fdfd1~ubuntu16.04.1_amd64.deb  Connection failed [IP: 64.50.233.100 80]

E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?
```

![image-20220625174707175](../../.gitbook/assets/ros-installation.assets/image-20220625174707175.png)

##### 原因：网络原因

##### 解决方案

输入以下命令，回车（**如仍报出同样错误，重复此操作**）

```shell
# 更新源
sudo apt-get update --fix-missing
# 输入安装命令
sudo apt-get install ros-kinetic-desktop-full -y
```

#### 卸载kinetic

可以调用如下命令:

```shell
sudo apt remove ros-kinetic-*
```

### 二、环境配置、构建依赖和初始化

#### 1. 环境配置（修改~/.bashrc文件）

方便在任意终端中使用 ROS

```shell
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2. 安装构建依赖

```shell
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```

#### 3. 初始化rosdep

```shell
sudo rosdep init
rosdep update
```

![image-20220625194753062](../../.gitbook/assets/ros-installation.assets/image-20220625194753062.png)

ps：这一步报错可能性很大，两者报错解决方法一样。

#### 错误及解决

##### 错误提示

![image-20220625180501059](../../.gitbook/assets/ros-installation.assets/image-20220625180501059.png)

##### 原因：境外资源屏蔽，网络原因

##### 解决方案

需要修改**资源下载的地址**，将软件包下载到本地，修改相关源码

###### 1. 下载软件包

官方地址（github）：[https://github.com/ros/rosdistro](https://github.com/ros/rosdistro)

B站赵老师（gitee）：[https://gitee.com/zhao-xuzuo/rosdistro](https://gitee.com/zhao-xuzuo/rosdistro)

###### 2. 文件路径

下载完，将文件复制到加目录下，切换进下载的文件夹，打印当前路径`pwd`。通常下载下来的文件名为`rosdistro-master`

```shell
# 打印出来的路径大致为
# USERNAME为你当前的用户空间，一般和用户名一致
/home/USERNAME/rosdistro-master
```

###### 3. 需要修改**四个**文件：

1. /home/USERNAME/rosdistro-master/rosdep/sources.list.d/20-default.list
2. /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py
3. /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
4. /usr/lib/python2.7/dist-packages/rosdistro/\__init__.py

主要就是将

其中的

`https://raw.githubusercontent.com/ros/rosdistro/master`

替换为

`file:///home/USERNAME/rosdistro-master` **（你的文件路径）**

**注意**：python文件中url本地文件地址格式是：**file://**+**文件地址**

###### 文件1 20-default.list

```
sudo gedit /home/USERNAME/rosdistro-master/rosdep/sources.list.d/20-default.list
```

修改后样例

```
# os-specific listings first
yaml file:///home/USERNAME/rosdistro-master/rosdep/osx-homebrew.yaml osx

# generic
yaml file:///home/USERNAME/rosdistro-master/rosdep/base.yaml
yaml file:///home/USERNAME/rosdistro-master/rosdep/python.yaml
yaml file:///home/USERNAME/rosdistro-master/rosdep/ruby.yaml
gbpdistro file:///home/USERNAME/rosdistro-master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead

```

###### 文件2 sources_list.py

```shell
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py
```

修改后样例

```python
# default file to download with 'init' command in order to bootstrap
# rosdep
# DEFAULT_SOURCES_LIST_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list'
DEFAULT_SOURCES_LIST_URL = 'file:///home/USERNAME/rosdistro-master/rosdep/sources.list.d/20-default.list'
```

###### 文件3 rep3.py

```shell
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
```

修改后样例

```python
# location of targets file for processing gbpdistro files
# REP3_TARGETS_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/releases/targets.yaml'
REP3_TARGETS_URL = 'file:///home/USERNAME/rosdistro-master/releases/targets.yaml'
```

###### 文件4 \__init__.py

```shell
sudo gedit /usr/lib/python2.7/dist-packages/rosdistro/__init__.py
```

修改后样例

```python
# index information

# DEFAULT_INDEX_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml'
DEFAULT_INDEX_URL = 'file:///home/USERNAME/rosdistro-master/index-v4.yaml'
```

###### 重新运行并成功

![image-20220625194353830](../../.gitbook/assets/ros-installation.assets/image-20220625194353830.png)

### 三、测试（小海龟）

打开三个命令窗口，分别输入

#### 启动roscore

```shell
roscore
```

![image-20220625202612031](../../.gitbook/assets/ros-installation.assets/image-20220625202612031.png)

#### 启动海龟GUI

```shell
rosrun turtlesim turtlesim_node
#rosrun为ROS命令行工具： rosrun 节点包 节点名
```

![image-20220625202625251](../../.gitbook/assets/ros-installation.assets/image-20220625202625251.png)

#### 启动键盘控制节点

```shell
rosrun turtlesim turtle_teleop_key
```

![image-20220625202658273](../../.gitbook/assets/ros-installation.assets/image-20220625202658273.png)

#### 最终效果

![image-20220625202750594](../../.gitbook/assets/ros-installation.assets/image-20220625202750594.png)

## 结语

通用安装完成，如果还需要更多操作，需要下载相关依赖和扩展，比如无人机还需要大疆SDK等。

## Reference

1. [kinetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu)

