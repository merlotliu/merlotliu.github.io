---
layout: post
title: Mac(M2) 安装 Ubuntu Arm
date: 2022-12-20 17:54:00 +0800
updated: 2022-12-20 17:54:00 +0800
tags: [mac, ubuntu, arm]
categories: [mac]
comments: true
---

## 虚拟机下载

这里下载的是 vmware fusion 的个人免费版：https://www.vmware.com/products/fusion.html

过程非常简单，注册，下载，安装，填写激活码（在注册完后软件下载链接上面）

## Ubuntu版本选择

目前，我发现的是，在 Arm 的处理器下，不是所有版本都能成功安装。以下是亲测可以成功的版本及链接：

- [Ubuntu 20.04.5 LTS (Focal Fossa) Daily Build](https://cdimage.ubuntu.com/focal/daily-live/current/)

其他版本：

- [https://cdimage.ubuntu.com/releases/](https://cdimage.ubuntu.com/releases/)

## 安装

安装的过程非常简单：

1. File -- New -- 将映像文件拖入到框中；
2. 除了可能修改一下当前虚拟机的名字，其他无脑一下步即可；
3. 然后就是 ubuntu 系统的安装，根据实际情况选择即可，有疑问可以自行百度；

## open-vm-tools

网上的教程，正常都可以使用 Fusion 中的 `Virtual Machine -- Install VMware tools` 安装 VMware tools。但我安装的时候会弹出 `` 警告。所以我只能另辟蹊径，最终找到了 open-vm-tools 。这是 VMware-tools 的开源版本，使用以下命令安装并重启虚拟机即可：

```shell
sudo apt-get update
sudo apt-get install open-vm-tools -y
sudo apt-get install open-vm-tools-desktop -y
reboot
```

open-vm-tools github 地址： https://github.com/vmware/open-vm-tools

## 无法上网

打开终端，依次输入并运行以下三行代码。

停止

```vbnet
sudo service network-manager stop
```

删除

```bash
sudo rm /var/lib/NetworkManager/NetworkManager.state
```

启动

```sql
sudo service network-manager start
```

## Reference 

1. [mac pro M1(ARM)安装：VMWare Fusion及linux(centos7/ubuntu)（一）](https://blog.csdn.net/qq_24950043/article/details/122517521)
2. [mac pro M1(ARM)安装：ubuntu虚拟机（四）](https://blog.csdn.net/qq_24950043/article/details/123764210)
3. [在ARM Mac 的虚拟机中安装Ubuntu遇到的问题](https://zhuanlan.zhihu.com/p/480229615)
4. https://blog.csdn.net/qq_41571459/article/details/114630738
5. https://blog.csdn.net/altabc/article/details/108713803
