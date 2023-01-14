---
layout: post
title: RotorS with virtual joystick
comments: true
date: 2022-07-25 16:26:42 
updated: 2022-07-25 16:26:42 
tags: [ROS,RotorS]
categories: [ROS, RotorS]
---

# RotorS with virtual joystick

## 使用键盘控制无人机

使用`mav_with_keyboard.launch`可以打开`gazebo`以及相关的控制界面，但需要做一些基础配置。

```shell
roslaunch rotors_gazebo mav_with_keyboard.launch mav_name:=firefly world_name:=basic
```

## 设置虚拟键盘控制器

配置原文 : [Setup virtual keyboard joystick · ethz-asl/rotors\_simulator Wiki (github.com)](https://github.com/ethz-asl/rotors\_simulator/wiki/Setup-virtual-keyboard-joystick)

为了创建虚拟键盘控制器，需要用到`Python-uniput`(https://github.com/devbharat/python-uinput)

### 安装 Python-uniput

> 源仓库仅需要暂时复制一份以供安装使用

```shell
git clone git@github.com:devbharat/python-uinput.git
cd python-uinput
python setup.py build
sudo python setup.py install
```

需要加载内核的`uinput`模块，才能使用`Python-uniput`。使用以下命令加载：

```shell
modprobe -i uinput
```

`uinput`是一个内核模块，可以从用户空间模拟输入设备。通过写入 `/dev/uinput`（`/dev/input/uinput`）设备，进程可以创建具有特定功能的虚拟输入设备。一旦创建了这个虚拟设备，进程就可以通过它发送事件，这些事件将被传递给用户空间和内核中的使用者。

### 配置设备权限

我们需要程序能够访问`uniput`设备，执行以下操作并重启：

```
cd udev-rules
sudo cp 40-uinput.rules /etc/udev/rules.d
```

如果没有生效，则需要将用户添加到`uniput`的组中，同时重启：

```shell
sudo addgroup uniput
sudo adduser $USER uniput
```

### 安装Python-pygame

```shell
# python2
sudo python -m pip install -U pygame
```

### 测试虚拟键盘控制器

```shell
# 先切换到工作空间下
source ./devel/setup.bash
# 启动控制器
rosrun rotors_joy_interface key_joystick.py
```

如果能正常弹出虚拟键盘控制器的GUI，即说明一切正常。

### 安装ROS控制杆依赖包

根据ROS配置，可能需要安装控制杆的依赖包：

```shell
sudo apt-get install ros-kinetic-joy
```

### 使用虚拟控制杆运行RotorS

```shell
roslaunch rotors_gazebo mav_with_keyboard.launch
```

## 相关键盘操作

w - throttle up ↑ - pitch up

s - throttle down ↓ - pitch down

a - yaw left → - roll right

d - yaw right ← - roll left

## 相关问题

### Error 1 : Unable to find uri \[model://xxx]

#### 错误描述

```shell
Error [parser.cc:581] Unable to find uri[model://sun]
Error [parser.cc:581] Unable to find uri[model://ground_plane]
```

#### 解决方案

`Gazebo` 需要互联网连接才能下载模型文件（例如默认世界的太阳和地平面）。如果您在第一次运行 `Gazebo` 时没有访问 Internet，则会出现此错误。还可以从[gazebosim.org/models](../ros-tutorials/gazebosim.org/models/) 手动下载模型文件并将它们放在您的`~/.gazebo/models文件`夹中。

```
# 解压缩
tar -xzvf models.tar.gz
```

### Error 2 : gazebo闪退

#### 错误描述

```shell
[gazebo_gui-3]process has died
```

![erro2](../.gitbook/assets/rotors-simulator-keyboard-usage.assets/erro2.png)

#### 解决方案

**方法1**

使用以下命令关闭硬件加速

```shell
echo " export SVGA_VGPU10=0" >> ~/.bashrc
source ~/.bashrc
```

**方法2**

关闭虚拟机的3D图形加速

**方法3**

更新`gazebo`

## Reference

1. [(71条消息) ROS 无人机仿真系统4 —— 通过键盘控制飞行器飞行\_KongDaQing1290的博客-CSDN博客\_ros无人机](https://blog.csdn.net/KongDaQing1290/article/details/79743026)
2. [Setup virtual keyboard joystick · ethz-asl/rotors\_simulator Wiki (github.com)](https://github.com/ethz-asl/rotors\_simulator/wiki/Setup-virtual-keyboard-joystick)
