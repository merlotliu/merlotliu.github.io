---
title: RotorS 分布式通信综合案例
comments: true
date: 2022-07-31 21:29:36
updated: 2022-07-31 21:29:36
tags: [ROS, RotorS]
categories:
- [ROS, RotorS]
---

# RotorS 分布式通信综合案例

## 概述

该示例将使用两台虚拟机，分别在两台虚拟机运行一个节点，以完成分布式通信。

现在我们假定其中一台虚拟机为 Sakura ，将运行节点 ctl_center。另外一台为 Haruko ，运行节点 uav。

此外，我们将涉及到两个服务 execute_task 和 /ctl_center/shutdown，两个话题 pics_data 和pics_process_result。

### 服务 execute_task

该服务由 ctl_center 提供， uav 进行调用。uav 在调用前填写 request 部分，随后发起服务调用。 ctl_center 接收到服务请求后，检查 request 内容，依据 request 内容填写 reponse ，并返回给 uav 节点。

传输的消息为自定义的 **Task.srv**。request 部分为标识当前 uav 节点是否正常连接到 gazebo 的bool值，response 为飞行路径及任务开始的位置信息。

### 话题 pics_data

该话题由 uav 发布，ctl_center 订阅。

传输的消息为自定义的 **PicsData.msg** 。描述 uav 执行任务时获取的图片信息。

### 话题 pics_process_result

该话题由 ctl_center 发布，uav 订阅。

传输的消息为自定义的 **PicsProcessResult.msg** 。描述控制中心对接收到的图片数据的处理结果。

### uav 节点

1. 打开gazebo创建无人机实例；
2. 调用execute_task服务，记录获取的路径path及执行任务的位置task_pos；
3. 在飞行期间，到达执行任务位置，收集周围数据，将数据通过pics_data话题发布；
4. 订阅pics_process_result话题以获取数据处理结果；
4. 当返回起始坐标，并且接收到处理结果之后，调用 /ctl_center/shutdown 服务。

### ctl_center 节点

1.  等待 uav 调用服务 execute_task ，将任务信息作为 response 内容返回；
2.  创建  /ctl_center/shutdown 服务，等待调用；
3.  订阅 pics_data 话题以获取图片信息；
4.  在话题 pics_process_result 发布图片处理结果。

## 要求

1. 功能的完整实现；
2. uav 和 ctl_center 需运行在不同的主机；
3. 消息文件srv、msg，生成在单独的功能包 uav_msgs；

## 具体实现

### 消息创建

该实例的 srv 和 msg 消息都会被创建在两个节点之外一个单独的功能包内。

#### 创建srv文件：Task.srv

uav节点打开gazebo连接成功后将填写request部分，即ready设置为true，并调用服务。

ctl_center接收到服务请求，若ready为true，填写response，将内容返回给uav节点。

```srv
# ExecuteTask.srv

bool ready

---
Header header
geometry_msgs/Point[] path
geometry_msgs/Point task_pos
```

#### 创建msg文件：PicsData.msg、PicsProcessResult.msg

PicsData.msg定义了图片相关的内容，由 uav 发布。

```
# PicsData.msg
Header header
float64 height
float64 wide
string data
```

PicsProcessResult.msg定义了图片相关的内容，由ctl_center发布。

```
# PicsProcessResult.msg
Header header
string result
```

**Notes**：srv和msg文件创建的其他相关配置请参见：[ROS/Tutorials/CreatingMsgAndSrv - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

## 创建任务发布、数据处理的节点：ctl_center.cpp

### 需求

1.  等待 uav 调用服务 execute_task ，将任务信息作为 response 内容返回；
2. 订阅 pics_data 话题以获取图片信息；
3. 在话题 pics_process_result 发布图片处理结果；

### 分析

在主机 Sakura，在工作空间下，创建新功能包 ctl_center，切换到其 src 目录下，创建 ctl_center.cpp 文件。

接下来将讲述具体的实现内容，关于 ROS 的初始化、 spin() 等基础内容将略过。

对于 ctl_center 的功能实现，并不需要 RotorS的相关内容。

根据需求创建、订阅、发布对应的服务、话题并编写配套的处理函数即可。

### ctl_center.cpp 完整源码

```cpp
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point.h"
#include "uav_msgs/Task.h"
#include "uav_msgs/PicsData.h"
#include "uav_msgs/PicsProcessResult.h"
/*
    function:
    1. provide the path of uav.
    2. process pics data and return value to uav.
*/

// picture data subscriber
ros::Subscriber pics_data_sub;
// picture process result publisher
ros::Publisher pics_process_res_pub;
// uav name
std::string uav_name;

/*
 *Description:
 *	Generate a gemetry_msgs::point type variable
 *Parameters:
 *	x, y, z : position 3D
 *Return:
 *  a gemetry_msgs::point variable
 */
geometry_msgs::Point getPoint(float x, float y, float z) {
    geometry_msgs::Point pt;
    pt.x = x; 
    pt.y = y;
    pt.z = z;  
    return pt;
}

/*
 *Description:
 *	accept pics data & do some process
 *Parameters:
 *	pics_data : picture data
 */
void doPicsProcess(const uav_msgs::PicsData::ConstPtr &pics_data) {
    // accept pics data 
    ROS_INFO("Accepted picture data :");
    ROS_INFO("  [ Height : %.2f, Wide : %.2f, Data : %s ]", 
        pics_data->height, 
        pics_data->wide, 
        pics_data->data.c_str());
    // do process
    uav_msgs::PicsProcessResult res;
    res.header.stamp = ros::Time::now();
    res.result = "Everything is ok. Nothing should be worried.";

    ROS_INFO("Pics process result :");
    ROS_INFO("  [ %s ]", res.result.c_str());
    // publish process result
    pics_process_res_pub.publish(res);
}

/*
 *Description:
 *	give a response for service execute_task
 *Parameters:
 *	req : srv request part
 *	res : srv response part
 *Return:
 *  service call result : true or false
 */
bool doTaskResponse(uav_msgs::Task::Request &req,
                    uav_msgs::Task::Response &res) {
    // accepted request content
    bool ready = req.ready;
    // response task & path
    if(!ready) {
        ROS_INFO("No uav ready!");
    } else {
        uav_name = req.uav_name;
        ROS_INFO("Uav %s connected!", uav_name.c_str());

        ros::NodeHandle nh;
        // subscribe pics data
        pics_data_sub = nh.subscribe<uav_msgs::PicsData>(std::string("/"+uav_name+"/pics_data").c_str(), 10, doPicsProcess);

        // publish pics process result
        pics_process_res_pub = nh.advertise<uav_msgs::PicsProcessResult>(std::string("/"+uav_name+"/pics_process_result").c_str(), 10);

        // time stamp
        res.header.stamp = ros::Time::now();
        
        // set uav path
        res.path.push_back(getPoint(1.0, 2.0, 3.0));
        res.path.push_back(getPoint(3.0, 4.0, 4.0));
        res.path.push_back(getPoint(4.0, 8.0, 9.0));
        res.path.push_back(getPoint(-2.0, 5.0, 5.0));
        res.path.push_back(getPoint(7.0, 3.0, 8.0));

        // set task start pos
        res.task_pos = res.path[3];

        ROS_INFO("Already send task data to uav.");
        ROS_INFO("Task pos is : [ <%.2f, %.2f, %.2f> ]", 
            res.task_pos.x,
            res.task_pos.y,
            res.task_pos.z);

        return true;
    }
    return false;
}

bool shutdown(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    ros::shutdown();
    return true;
}

int main(int argc, char *argv[])
{
    // init ros 
    ros::init(argc, argv, "ctl_center");
    ros::NodeHandle nh;

    // create service "excute_task"
    ros::ServiceServer task_srv = nh.advertiseService("execute_task", &doTaskResponse);
    ros::ServiceServer shutdown_srv = nh.advertiseService("/ctl_center/shutdown", shutdown);
    ROS_INFO("Control center is ready.");
    ROS_INFO("Waiting for a service call ...");

    ros::spin();
    return 0;
}

```

### Notes

1. 一定要接收创建服务时的 ros::ServiceServer 对象，因为该服务的生命周期等同于该对象的周期；
2. ros::Publihser、ros::subscriber 也是如此；
3. 话题的名称以及命名空间需要注意，否则无法通信；

## 创建执行任务、数据采集、实战飞行的无人机节点：uav.cpp

### 需求

1. 打开gazebo创建无人机实例；
2. 调用execute_task服务，记录获取的路径path及执行任务的位置task_pos；
3. 在飞行期间，到达执行任务位置，收集周围数据，将数据通过pics_data话题发布；
4. 订阅pics_process_result话题以获取数据处理结果。

### 分析

基于 RotorS 简单案例，添加新增的服务、话题以及配套的回调函数即可。

RotorS 简单示例可参见：https://merlotliu.github.io/2022/07/25/rotors-simulator-usage

### uav.cpp 完整源码

```cpp
#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "uav_msgs/Task.h"
#include "uav_msgs/PicsData.h"
#include "uav_msgs/PicsProcessResult.h"

ros::Publisher trajectory_pub;
ros::Publisher pics_data_pub;
geometry_msgs::PointStamped cur_pos_msg;

float linear_smoothing_navigation_step = 2;
bool gps_init_ok = false;
bool take_off_ok = false;
int tasks_ok = 0;
geometry_msgs::Point home_pos;
std::vector<geometry_msgs::Point> path;
geometry_msgs::Point task_pos;
bool b_home = false;
bool b_process_res = false;

geometry_msgs::Point getPoint(float x, float y, float z) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}
/*
 *Description:
 *	when gps position changed, we update related parameters.
 *Parameters:
 *	msg : current position message
 */
void updateUAVPosition(const geometry_msgs::PointStamped& msg) {
	if(!gps_init_ok) {
		// prepare to take off
		gps_init_ok = true;
		// record current position
		home_pos.x = msg.point.x;
		home_pos.y = msg.point.y;
		home_pos.z = msg.point.z;
		ROS_INFO("Home Position is <%f, %f, %f>", msg.point.x, msg.point.y, msg.point.z);
	}
	cur_pos_msg = msg;
	// ROS_INFO("Current Position is <%f, %f, %f>", msg.point.x, msg.point.y, msg.point.z);
}

/*
 *Description:
 *	get pictures process result from topic pics_process_result
 *Parameters:
 *  msg : pictures process result
 */
void getPicsProcessResult(const uav_msgs::PicsProcessResult &msg) {
	ROS_INFO("Pics process result is :");
	ROS_INFO("  [ %s ]", msg.result.c_str());
	b_process_res = true;
}

/*
 *Description:
 *	when uav get to the task pos, we take photos and send them to Topic pics_data
 *Parameters:
 *  none
 */
void executeTask() {
    uav_msgs::PicsData pics_data;
    pics_data.header.stamp = ros::Time::now();
    pics_data.height = 7.0;
    pics_data.wide = 10.0;
    pics_data.data = "This is a photo.";
    pics_data_pub.publish(pics_data);

	ROS_INFO("Uav sends picture data : ");
	ROS_INFO("  [ Height : %.2f, Wide : %.2f, Data : %s ]", 
        pics_data.height, 
        pics_data.wide, 
        pics_data.data.c_str());
}

/*
 *Description:
 *	calculate the distance between current position and target position
 *Parameters:
 *	target_pos : destination position	
 *Return:
 *	double : distance 
 */
double getDistanceToDestination(const geometry_msgs::Point& target_pos) {
	double distance = 0.0;
	distance = pow((target_pos.x - cur_pos_msg.point.x), 2) + 
			   pow((target_pos.y - cur_pos_msg.point.y), 2) +
			   pow((target_pos.z - cur_pos_msg.point.z), 2);
	return sqrt(distance);
}

/*
 *Description:
 *	
 *Parameters:
 *	target_pos : destination position	
 *	max_err : max error (default = 0.0001)
 *Return:
 *	true : arrival
 *	false : not arrival
 */
bool isReachDestination(const geometry_msgs::Point& target_pos, float max_err = 0.2) {
	double cur_err = getDistanceToDestination(target_pos);
	return cur_err < max_err;
}

bool linearSmoothingNavigationTask(const geometry_msgs::Point& target_pos) {
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	if(isReachDestination(target_pos)) {
		return true;
	}
	double dist = getDistanceToDestination(target_pos);
	geometry_msgs::Point next_pos;
	if(dist < linear_smoothing_navigation_step) {
		next_pos = target_pos;
	} else {
		next_pos.x = cur_pos_msg.point.x + 
			(target_pos.x - cur_pos_msg.point.x) / dist * linear_smoothing_navigation_step;
		next_pos.y = cur_pos_msg.point.y + 
			(target_pos.y - cur_pos_msg.point.y) / dist * linear_smoothing_navigation_step;
		next_pos.z = cur_pos_msg.point.z + 
			(target_pos.z - cur_pos_msg.point.z) / dist * linear_smoothing_navigation_step;
	}
	double target_yaw = 0.0;
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(Eigen::Vector3d(target_pos.x, target_pos.y, target_pos.z), target_yaw, &trajectory_msg);
	trajectory_pub.publish(trajectory_msg);
	
	return false;	
}

/*
 *Description:
 *	UAV reach the height of taking off.
 *Parameters:
 *	altitude : the height of taking off
 */
bool takeOff(float altitude) {
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	static geometry_msgs::Point target_pos;
    target_pos.x = cur_pos_msg.point.x;
    target_pos.y = cur_pos_msg.point.y;
    target_pos.z = altitude;
	double target_yaw = 0.0;
	if(isReachDestination(target_pos)) {
		return true;
	}

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(Eigen::Vector3d(target_pos.x, target_pos.y, target_pos.z), 
														target_yaw, 
														&trajectory_msg);
	trajectory_pub.publish(trajectory_msg);
	return false;
}

/*
 *Description:
 *	Go back to the origin position. UAV move to home_pos with current altitude. Then UAV lands.
 */
void goHome() {
	static geometry_msgs::Point home_pos_bar = getPoint(home_pos.x, 
										                home_pos.y,
                										cur_pos_msg.point.z);
	static bool home_pos_ok = false;
	if(!home_pos_ok) {
		home_pos_ok = linearSmoothingNavigationTask(home_pos_bar);	
	} else {
		linearSmoothingNavigationTask(home_pos);
		ROS_INFO("UAV aready go back.");
        b_home = true;
	}
}

int main(int argc, char *argv[]) {
	// set local code
	setlocale(LC_ALL, "");
	// ros node init
	ros::init(argc, argv, "uav_with_task");
	ros::NodeHandle nh;
	// create a private node for accessing node parameters
	ros::NodeHandle nh_private("~");

	std::string uav_name = "";
	ros::param::get("~mav_name", uav_name);

	// global variable
	// publisher: publish uav's position
	trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
	pics_data_pub = nh.advertise<uav_msgs::PicsData>("pics_data", 10);
    // subcribe topic
	// uav position info : /odometry_sensor1/position
	// subscriber: subscribe uav's current position
	ros::Subscriber pos_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, updateUAVPosition);
    
    ros::Subscriber pics_process_res_sub = nh.subscribe(std::string("pics_process_result").c_str(), 10, getPicsProcessResult);

	// wait 5s for gazebo successfully startup
	ros::Duration(5.0).sleep();

	// make gazebo auto-play
	std_srvs::Empty srv;
	bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

	// try to make gazebo auto-play
	int i = 0;
	while(i <= 10 && !unpaused) {
		ROS_INFO("Wait for 1 second before trying to unpause Gazebo again!");
		std::this_thread::sleep_for(std::chrono::seconds(1));
		unpaused = ros::service::call("/gazebo/unpause_physics", srv);
		++i;
	}
	
	// if Gazebo is not auto-play, then exit
	if(!unpaused) {
		ROS_FATAL("Fail to wake up Gazebo.");
		return -1;	
	} else {
		ROS_INFO("Unpaused the Gazebo simulation.");
	}

    // if can't get task, then exit
	ROS_INFO("Uav is ready!");
    uav_msgs::Task task;
	task.request.ready = true;
	task.request.uav_name = uav_name;
    bool b_task = ros::service::call("/execute_task", task);
    if(!b_task) {
        ROS_FATAL("Fail to get task content.");
		return -1;	
    } else {
        path = task.response.path;
        task_pos = task.response.task_pos;
        ROS_INFO("We get task conent.");
		ROS_INFO("Trajectory has %d points.", path.size());
		ROS_INFO("Task pos is : [ <%.2f, %.2f, %.2f> ]", 
            task_pos.x,
            task_pos.y,
            task_pos.z);
    }

	ros::Rate loop_rate(10);
	while(ros::ok()) {
		if(gps_init_ok && !take_off_ok) {
			take_off_ok = takeOff(3);
		} else if(take_off_ok && tasks_ok < path.size()) {
			if(tasks_ok < path.size()) {
				ROS_INFO("task number : %d", tasks_ok);
				bool tmp = linearSmoothingNavigationTask(path[tasks_ok]);
				if(tmp) {
					tasks_ok++;
				}
			}
            if(isReachDestination(task_pos)) {
                executeTask();
            }
		} else if(tasks_ok >= path.size()) {
			goHome();
		}
		if(b_home && b_process_res) {
			ros::service::call("/ctl_center/shutdown", srv);
			exit(0);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
```



## 遇到错误

### 创建消息时错误

#### 错误1

```
CMake Error at /home/ml/ros_pros/ros_tutorials/build/uav_msgs/cmake/uav_msgs-genmsg.cmake:3 (message):
  Could not find messages which
  '/home/ml/ros_pros/ros_tutorials/src/uav_msgs/srv/Task.srv' depends on.
  Did you forget to specify generate_messages(DEPENDENCIES ...)?

  Cannot locate message [Point]: unknown package [geometry_msgs] on search
  path [{{'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'],
  'uav_msgs': ['/home/ml/ros_pros/ros_tutorials/src/uav_msgs/msg']}}]
```

##### 分析

自定义消息使用了 geometry_msgs 下的类型 

##### 解决

在 CMakelists.txt 中添加 find_package、generate_messages：

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

### 编写源码时错误

#### 错误1

```
fatal error: uav_msgs/Task.h: No such file or directory
```

##### 解决

CMakelists.txt 中 find_package和catkin_package添加 uav_msgs：

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ctl_center
 CATKIN_DEPENDS roscpp rospy std_msgs uav_msgs
#  DEPENDS system_lib
)
```



#### 错误2

```
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:196 (message):
  catkin_package() the catkin package 'uav_msgs' has been find_package()-ed
  but is not listed as a build dependency in the package.xml
```

##### 解决

package.xml 中添加

```
<depend>uav_msgs</depend>
```

#### 错误3

```shell
no matching function for call to ‘ros::AdvertiseServiceOptions::initBySpecType(const string&, const boost::function<bool(uav_task_execution::Task&)>&)’
```

##### 分析

AdvertiseService函数调用错误，找到正确的使用方式即可。

解决

这里将泛型去掉即可，服务的回调函数需要有bool返回值。

## Reference 

1. http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html
1. [ROS/Tutorials/CreatingMsgAndSrv - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
1. https://merlotliu.github.io/2022/07/25/rotors-simulator-usage
