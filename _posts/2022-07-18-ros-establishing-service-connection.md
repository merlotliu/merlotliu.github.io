---
layout: post
title: ROS Service 理论模型
comments: true
date: 2022-07-18 11:37:52
updated: 2022-07-18 11:37:52
tags: [ROS,ROS-Service]
categories: [ROS,beginner-tutorials]
---

# ROS Service 理论模型

## 实现流程

基于请求响应的服务通信，被视为是话题通信的简化版本。值得注意的是，话题可以有多个发布者，而提供服务的只能有一个，且以最近注册该服务的server为准。服务的client可以有多个，且不一定是 ROS 节点。服务通信要求，在client提交请求后，server能够在有限时间内实时的回复。大体实现流程如下：

1. server和 client 分别在 Master 注册，提交服务名称、自身的URI；（XMLRPC）
2. client 查看 Master 中对应的服务，并获取对应的 TCP 地址；（XMLRPC）
3. client 向 server发起 TCP/IP 连接；（TCPROS）
4. client 填写 request，向server调用服务；（TCPROS）
5. server 收到 request，处理信息，填写 response ，返回给 client，调用完成；（TCPROS）

## 示例

> 在无人机任务执行的过程中，可能会遇到不可识别的单位，需要将采集到的数据，传输给数据处理节点，并获取有关信息。在这一情况下，通常对通信有着实时性的要求，并期待获得响应。基于请求响应的服务通信具备这样的功能，且能完成一定的逻辑处理。

![img](../.gitbook/assets/ros-establishing-service-connection.assets/02_服务通信模型.jpg)

## 官方原文

### Establishing a service connection

We have not discussed services as much in this overview, but they can be viewed as a simplified version of topics. Whereas topics can have many publishers, there can only be a single service provider. The most recent node to register with the master is considered the current service provider. This allows for a much simpler setup protocol -- in fact, a service client does not have to be a ROS node.

1. Service registers with Master
2. Service client looks up service on the Master
3. Service client creates TCP/IP to the service
4. Service client and service exchange a [Connection Header](http://wiki.ros.org/ROS/Connection Header)
5. Service client sends serialized request message
6. Service replies with serialized response message.

If the last several steps look familiar, its because they are an extension of the [TCPROS](http://wiki.ros.org/ROS/TCPROS) protocol. In fact, [rospy](http://wiki.ros.org/rospy) and [roscpp](http://wiki.ros.org/roscpp) both use the same TCP/IP server socket to receive both topic and service connections.

As there is no callback from the Master when a new service is registered, many client libraries provide a "wait for service" API method, that simply polls the Master until a service registration appears.

### Persistent service connections

By default, service connections are stateless. For each call a client wishes to make, it repeats the steps of looking up the service on the Master and exchanging request/response data over a new connection.

The stateless approach is generally more robust as it allows a service node to be restarted, but this overhead can be high if frequent, repeated calls are made to the same service.

ROS allows for *persistent* connections to a service, which provide a very high-throughput connection for making repeated calls to a service. With these persistent connections, the connection between the client and service is kept open so that the service client can continue to send requests over the connection.

Greater care should be used with persistent connections. If a new service provider appears, it does not interrupt an ongoing connection. Similarly, if a persistent connection fails, there is no attempt made to reconnect.



## Reference 

1. [ROS/Technical Overview - ROS Wiki](http://wiki.ros.org/ROS/Technical Overview)
1. [2.2.1 服务通信理论模型 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/di-2-zhang-ros-jia-gou-she-ji/23-fu-wu-tong-xin/221-fu-wu-tong-xin-li-lun-mo-xing.html)
