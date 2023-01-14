---
layout: post
title: ROS 集成开发环境搭建
comments: true
date: 2022-07-15 15:42:07
updated: 2022-07-15 15:42:07
tags: [ROS, ROS-IDE]
categories: [ROS]
---

# IDE & Tools Configuration

和大多数开发环境一样，理论上，在 ROS 中，只需要记事本就可以编写基本的 ROS 程序，但是工欲善其事必先利其器，为了提高开发效率，可以先安装集成开发工具和使用方便的工具 : 终端、IDE....

## 終端Terminator安裝

在 ROS 中，需要频繁的使用到终端，且可能需要同时开启多个窗口，推荐一款较为好用的终端: Terminator。效果如下:

![img](../.gitbook/assets/ros-IDE-config.assets/terminator效果.PNG)

### **安裝**

```shell
sudo apt install terminator
```

### **添加到收藏夾**

顯示應用程序 ---- 搜索 terminator ---- 右擊選擇 添加到收藏夾

### **常用快捷鍵**

**第一部分：關於在同一個標簽内的操作**

```
Alt+Up                          //移动到上面的终端
Alt+Down                        //移动到下面的终端
Alt+Left                        //移动到左边的终端
Alt+Right                       //移动到右边的终端
Ctrl+Shift+O                    //水平分割终端
Ctrl+Shift+E                    //垂直分割终端
Ctrl+Shift+Right                //在垂直分割的终端中将分割条向右移动
Ctrl+Shift+Left                 //在垂直分割的终端中将分割条向左移动
Ctrl+Shift+Up                   //在水平分割的终端中将分割条向上移动
Ctrl+Shift+Down                 //在水平分割的终端中将分割条向下移动
Ctrl+Shift+S                    //隐藏/显示滚动条
Ctrl+Shift+F                    //搜索
Ctrl+Shift+C                    //复制选中的内容到剪贴板
Ctrl+Shift+V                    //粘贴剪贴板的内容到此处
Ctrl+Shift+W                    //关闭当前终端
Ctrl+Shift+Q                    //退出当前窗口，当前窗口的所有终端都将被关闭
Ctrl+Shift+X                    //最大化显示当前终端
Ctrl+Shift+Z                    //最大化显示当前终端并使字体放大
Ctrl+Shift+N or Ctrl+Tab        //移动到下一个终端
Ctrl+Shift+P or Ctrl+Shift+Tab  //Crtl+Shift+Tab 移动到之前的一个终端
```

**第二部份：有关各个标签之间的操作**

```
F11                             //全屏开关
Ctrl+Shift+T                    //打开一个新的标签
Ctrl+PageDown                   //移动到下一个标签
Ctrl+PageUp                     //移动到上一个标签
Ctrl+Shift+PageDown             //将当前标签与其后一个标签交换位置
Ctrl+Shift+PageUp               //将当前标签与其前一个标签交换位置
Ctrl+Plus (+)                   //增大字体
Ctrl+Minus (-)                  //减小字体
Ctrl+Zero (0)                   //恢复字体到原始大小
Ctrl+Shift+R                    //重置终端状态
Ctrl+Shift+G                    //重置终端状态并clear屏幕
Super+g                         //绑定所有的终端，以便向一个输入能够输入到所有的终端
Super+Shift+G                   //解除绑定
Super+t                         //绑定当前标签的所有终端，向一个终端输入的内容会自动输入到其他终端
Super+Shift+T                   //解除绑定
Ctrl+Shift+I                    //打开一个窗口，新窗口与原来的窗口使用同一个进程
Super+i                         //打开一个新窗口，新窗口与原来的窗口使用不同的进程
```

## VScode安裝

### **安装包下载**

vscode linux64下载:https://code.visualstudio.com/docs/?dv=linux64\_deb

历史版本: https://code.visualstudio.com/updates

### **安装与卸载**

**安装**

```shell
sudo dpkg -i xxxx.deb
```

**卸载**

```shell
sudo dpkg --purge  code
```

**插件安装**

`C/C++`、`CMake Tools`、`Python`、`ROS`

![image-20220702144712035](../.gitbook/assets/ros-IDE-config.assets/image-20220702144712035.png)

### **基本配置**

个人认为除了以上插件，仅需要添加ROS头文件路径保证代码补全即可，以提升coding效率。

具体打开工作空间下`.vscode/c_cpp_properties.json`文件，在`includePath`中添加安装的ROS的路径即可。

如需更多配置，可参考：[VScode ROS 配置](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/14-ros-ji-cheng-kai-fa-huan-jing-da-jian/142-an-zhuang-vscode.html)

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                // 将 kinetic 更换为自己安装的 ROS 版本即可
                "/opt/ros/kinetic/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

![image-20220702154100502](../.gitbook/assets/ros-IDE-config.assets/image-20220702154100502.png)

## Reference

1. [1.4 ROS集成开发环境搭建 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/14-ros-ji-cheng-kai-fa-huan-jing-da-jian.html)
