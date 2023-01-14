---
layout: post
title: ROS 分布式通信
comments: true
date: 2022-07-25 12:21:36
updated: 2022-07-25 12:21:36
tags: [ROS]
categories: [ROS]
---

# ROS 分布式通信

ROS 的设计考虑了分布式计算。一个编写良好的节点不假设它在网络中运行的位置，允许在运行时重新定位计算以匹配可用资源（有例外；例如，与硬件通信的驱动程序节点必须运行在硬件物理连接的机器上）。在多台机器上部署 ROS 系统很容易。请记住以下几点：

- 你只需要一个 Master 节点。随便选择一台机器来运行它。
- 所有节点都必须通过 ROS MASTER URI 配置为使用相同的 Master 节点。
- 在所有端口上的所有机器对之间必须有完整的双向连接（请参阅 [ROS/网络设置](http://wiki.ros.org/ROS/NetworkSetup)）。
- 每台机器都必须通过一个所有其他机器都可以解析的名称来标识自己（参见  [ROS/网络设置](http://wiki.ros.org/ROS/NetworkSetup)）。

## 准备

先要保证不同计算机处于同一网络中，最好分别设置固定IP。

我们在这里使用的是虚拟机，需要将网络适配器改为**桥接模式**；

## 修改主机名称

我们准备了两台虚拟机，将其中一台命名为`shaun`，另外一台命名为`lea`。可以通过以下方式修改：

1. 直接打开`/etc/hostname`，将文本修改为`<new-hostname>`。或者使用`hostnamectl set-hostname <new-hostname>`修改主机名。

2. 修改`/etc/hosts`文件，使主机名与本地`ip`地址完成映射：

   ```
   127.0.0.1   localhost
   127.0.1.1   <new-hostname> # change to your new hostname
   ```

## 其他主机名与IP地址的映射

1. 使用`ifconfig`查看本机ip地址。以此获取所有主机的地址。
2. 使用`hostname`查看主机名。

3. 在`/etc/hosts`中继续将其他主机的ip地址和主机名添加。

```
IPAddress     		Hostname    		Alias
127.0.0.1			localhost	 	 	deep.openna.com
208.164.186.1		deep.openna.com		deep
208.164.186.2		mail.openna.com		mail
208.164.186.3		web.openna.com		web
```

## 测试连接

### ping

主机 `shaun`（其他主机的操作类似）：

```
# 远程连接主机，并输入明码
ssh shaun
# ping 自己
ping shaun
# ping 其他主机
ping lea
```

成功后，都会出现下面类似的消息。

```
PING lea (192.168.1.30) 56(84) bytes of data.
64 bytes from lea (192.168.1.30): icmp_seq=1 ttl=64 time=0.226 ms
64 bytes from lea (192.168.1.30): icmp_seq=2 ttl=64 time=1.71 ms
64 bytes from lea (192.168.1.30): icmp_seq=3 ttl=64 time=4.04 ms
```

**Notes**：ssh连接失败，通常是由于sshd未安装，使用相关命令安装即可。

### netcat

ping 只检查 ICMP 数据包是否可以在机器之间传递，这还不够。您需要确保可以通过所有端口进行通信。
这很难完全检查，因为您必须迭代大约 65 K 端口。代替完整的检查，您可以使用 netcat 尝试通过任意选择的端口进行通信。**一定要选择大于 1024 的端口**；低于 1024 的端口需要超级用户权限。请注意，netcat 可执行文件在某些发行版上可能被命名为 nc。

尝试从`lea`到`shaun`进行通信。在`shaun`使用`netcat`启动监听:

```
ssh shaun
netcat -l 1234
```

打开新终端，连接到`lea`：

```
ssh lea
netcat shaun 1234
```

连接完成后，即可在下方输入文本进行通信。反过来在测试一次即可。没有问题则说明这两台主机之间的连接时全双工且正常的。

## 分布式通信

现在，我们可以开始分别在两台主题上运行，发布订阅节点。首先选择一台主机作为 Master 运行的主节点，并在此运行 `roscore`。在`shaun`和`lea`中，我们选择`shaun`。

### 启动Master

首先，连接到`shaun`并启动`roscore`：

```
ssh shaun
rocore
```

### 启动 Listener

```
ssh lea
# master host 
export ROS_MASTER_URI=http://shaun:11311
# start listner
rosrun rospy_tutorials listener
```

### 启动 Talker

```
ssh shaun
# master host 
export ROS_MASTER_URI=http://shaun:11311
# start talker
rosrun rospy_tutorials talker
```

没有问题后，可以在`lea`运行`talker`，在`shaun`运行`listener`在测试一遍。

## ROS_MASTER_URI 设置

### 临时设置

从用户的本次设置到用户注销有效：

```shell
export ROS_MASTER_URI=http://<ip-address/hostname>:11311
```

### 用户设置

修改 `.bashrc`，用户每次打开一个终端都会执行 `.bashrc`，所以对于该用户每次都会自动执行该语句：

```shell
echo "export ROS_MASTER_URI=http://<ip-address/hostname>:11311" >> .bashrc
```

也就是我们对于环境变量的临时修改，都可以通过修改 `.bashrc` 的方式做到 “长久自动” 的修改。

## rostopic

对于测试，您可以在连接到Master的所有机器上使用 rostopic 工具。您将获得所有可用主题的列表。如果您未连接到Master，则会出现错误。

```
rostopic list
```

在无线网络中，有时需要检查是否有连接并且消息是否仍然出现。
对于简短的测试，打印出消息很方便。

```
rostopic echo /<topic_name>
```

## Troubleshooting

如果上述顺序中的某些内容不起作用，则原因可能在于您的网络配置。
有关配置要求和故障排除提示，请参阅 [ROS/NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup) 和[ROS/Troubleshooting](http://wiki.ros.org/ROS/Troubleshooting)。
一个常见的问题是在运行`talker`的机器上缺少对`ROS_IP` 的定义。
检查它： `echo $ROS IP`;
如果您没有定义 `ROS_IP`，那么 `rostopic info` 将确实显示发布者和侦听器的正确连接，但 `rostopic echo` 将为空。
您将在 LAN 上，在有通话器的机器上看到没有 TX 流量。
首先，在使用正确的 IP 地址（`export ROS_IP=machine_ip_addr`）定义 `ROS_IP` 后，您将看到 LAN 上的流量，并且 `listener` 将显示接收到的数据。

## Reference 

1. [ROS/NetworkSetup - ROS Wiki](http://wiki.ros.org/ROS/NetworkSetup)
1. [ROS/Tutorials/MultipleMachines - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
1. [4.7 ROS分布式通信 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/5/44-rosfen-bu-shi-tong-xin.html)
