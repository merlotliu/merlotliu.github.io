---
title: ROS Topic 理论模型
comments: true
date: 2022-07-18 11:10:28
updated: 2022-07-18 11:10:28
tags: [ROS,ROS-Topic]
categories:
- [ROS,beginner-tutorials]
- [ROS,Topic]
---

# ROS Topic 理论模型

## 实现流程

1. publisher 向 Master 注册，并提供 话题名称、消息类型和节点URI（RPC 地址）。Master会维护一个 publisher table。（XMLRPC）
2. subscriber 向 Master 注册，并提供 话题名称、消息类型和节点URI（RPC 地址）。Master会维护一个 subscriber table。Master返回给 subscriber 一个 publisher URIs list，并会在新的 publihser到来时更新。（XMLRPC）
3. subscriber 根据 publisher URIs list ，向订阅话题的 publisher 发起 Topic 连接 请求，并协商一个传输协议（TCPROS...）。（XMLRPC）
4. publishers 将选定的传输协议发送给 subscriber。（XMLRPC）
5. subscriber 使用 publishers 选定的协议建立连接。（TCPROS, etc ...）
6. publishers 使用选定的协议发送话题数据。（TCPROS, etc ...）

**Notes**：

1. publisher 和 subscriber 的注册并无严格的先后顺序；
2. 同一话题的 publisher 和 subscriber 都可以有多个；
3. 匿名通信，publisher 并不关心谁订阅消息也不会知道，subscriber 不关心谁发布消息。前者仅向对应话题发布消息，后者向对应话题读取而已；
4. 连接建立完成后，数据流并不通过 Master 节点，即 Master 的关闭不会影响通信；

## 示例

> 为了控制 Hokuyo 激光测距仪，我们启动 hokuyo_node 节点，该节点与激光对话并在扫描主题上发布 sensor_msgs/Laser Scan 消息。为了可视化激光扫描数据，我们启动 rviz 节点并订阅扫描主题。订阅后，rviz 节点开始接收激光扫描消息，并将其呈现到屏幕上。

![master-node-example.png](../.gitbook/assets/ros-esablishing-topic-connection.assets/Technical Overviewaction=AttachFile&do=get&target=master-node-example.png)

## 官方原文

### Establishing a topic connection

Putting it all together, the sequence by which two nodes begin exchanging messages is:

1. Subscriber starts. It reads its command-line remapping arguments to resolve which topic name it will use. (Remapping Arguments)
2. Publisher starts. It reads its command-line remapping arguments to resolve which topic name it will use. (Remapping Arguments)
3. Subscriber registers with the Master. (XMLRPC)
4. Publisher registers with the Master. (XMLRPC)
5. Master informs Subscriber of new Publisher. (XMLRPC)
6. Subscriber contacts Publisher to request a topic connection and negotiate the transport protocol. (XMLRPC)
7. Publisher sends Subscriber the settings for the selected transport protocol. (XMLRPC)
8. Subscriber connects to Publisher using the selected transport protocol. (TCPROS, etc...)

The XMLRPC portion of this will look like:

`/subscriber_node` → `master.registerSubscriber(/subscriber_node, /example_topic, std_msgs/String, http://hostname:1234)`

Master returns that there are no active publishers.

`/publisher_node` → `master.registerPublisher(/publisher_node, /example_topic, std_msgs/String, http://hostname:5678)`

Master notices that `/subscriber_node` is interested in `/example_topic`, so it makes a callback to the subscriber

`master` → `subscriber.publisherUpdate(/publisher_node, /example_topic, [http://hostname:5678])`

Subscriber notices that it has not connected to `http://hostname:5678` yet, so it contacts it to request a topic.

`subscriber` → `publisher.requestTopic(/subscriber_node, /example_topic, [[TCPROS]])`

Publisher returns `TCPROS` as the selected protocol, so subscriber creates a new connection to the publishers TCPROS host:port.

### Example

To control a Hokuyo laser range-finder, we start the [hokuyo_node](http://wiki.ros.org/hokuyo_node) node, which talks to the laser and publishes [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) messages on the **scan** topic. To visualize the laser scan data, we start the [rviz](http://wiki.ros.org/rviz) node and subscribe to the **scan** topic. After subscription, the rviz node begins receiving LaserScan messages, which it renders to the screen.

Note how the two sides are decoupled. All the hokuyo_node node does is publish scans, without knowledge of whether anyone is subscribed. All the rviz does is subscribe to scans, without knowledge of whether anyone is publishing them. The two nodes can be started, killed, and restarted, in any order, without inducing any error conditions.

In the example above, how do the laser_viewer and hokuyo_node nodes find each other? They use a name service that is provided by a special node called the **master.** The [Master](http://wiki.ros.org/Master) has a well-known XMLRPC URI that is accessible to all nodes. Before publishing on a topic for the first time, a node **advertises** its intent to publish on that topic. This advertisement sends to the master, via XMLRPC, information about the publication, including the message type, the topic name, and the publishing node's URI. The master maintains this information in a publisher table.

When a node subscribes to a topic, it communicates with the master, via XMLRPC, sending the same information (message type, topic name, and node URI). The master maintains this information in a subscriber table. In return, the subscriber is given the current list of publisher URIs. The subscriber will also receive updates from the master as the list of publishers changes. Given the list of publishers, the subscribing node is ready to initiate transport-specific connections.

**Note**: message data does **not** flow through the master. It only provides name service, connecting subscribers with publishers.

## Reference 

1. [ROS/Technical Overview - ROS Wiki](http://wiki.ros.org/ROS/Technical Overview)
