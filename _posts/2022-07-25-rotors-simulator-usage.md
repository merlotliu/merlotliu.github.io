---
layout: post
title: RotorS 简单示例
comments: true
date: 2022-07-25 17:09:11
updated: 2022-07-25 17:09:11
tags: [ROS, RotorS]
categories: [ROS, RotorS]
---

# RotorS 简单示例

## 1 创建功能包

切换到工作空间的`src`目录下，创建`rotors_control_test`功能包

~~~shell
# 工作空间目录下的src
cd catkin_ws/src
# 创建功能包，包名后面的参数是相关依赖
catkin_create_pkg rotors_control_test roscpp rospy std_msgs 
# sensor_msgs geometry_msgs mav_msgs gazebo_msgs
~~~

## 2 修改相关功能包相关依赖

如果创建功能包的时候，少或错误添加相关依赖，可以打开`CMakeLists.txt`，修改相关内容:

```shell
find_package(catkin REQUIRED COMPONETS
	roscpp
	rospy
	std_msgs
	# 以上三个是常用的依赖，以下是本功能包需要用到的额外的依赖
	sensor_msgs
	geometry_msgs
	mav_msgs
	gazebo_msgs
)
```

同时需要修改`package.xml`文件，在其中添加:

~~~xml
<depend>gazebo_msgs</depend>
<depend>gazebo_plugins</depend>
<depend>geometry_msgs</depend>
<depend>joy</depend>
<depend>mav_msgs</depend>
<depend>rotors_gazebo_plugins</depend>
<depend>sensor_msgs</depend>
<depend>xacro</depend>
~~~

添加依赖XXX，在功能包中即可以使用XXX/XXX的消息类型，如`mav_msgs`，能够让功能包使用`mav_msgs/xxx`类型的消息。

## 3 创建源文件并修改`CMakeLists.txt`

在`rotors_control_test/src`下创建`rotors_control_node.cpp`文件，并修改`CMakeLists.txt`：

```shell
add_executable(
	rotors_control_node src/rotors_control_node.cpp
)

target_link_libraries(
	rotors_control_node ${catkin_LIBRARIES}
)
```

## 4 编写源文件代码

### 代码思路

1. 初始化 ROS 节点；
2. 创建`position_sub`订阅无人机位置信息话题：
   1. 话题名为`/odometry_sensor1/position`，可以通过`rqt`查询其他话题名称及对应的消息数据类型；
   2. 创建`updateUAVPosition`回调函数，位置信息类型为`geometry_msgs::PointStamped`；
   3. **注意**：当`GPS`初始化成功并能够接收到位置信息时，才开始执行任务；
3. 创建名为`trajectory_pub`的控制指令`publisher`:
   1. 控制无人机移动可使用`rotors_simulator`包中的控制节点：当前`pulisher`中发布的话题即为`rotors_simulator`中已定义好的话题；
   2. 值得注意的是，`rotors_simulator`中控制节点仅对无人机位置进行控制，即发送**指定位置**。同时，在起点与目标位置相距较远时，无人机会侧翻（对于仿真是没有什么影响，但对于将算法投入实际使用时，物理机则需要更多考量）；
   3. 为避免无人机侧翻，平滑移动轨迹仍十分重要。

### 相关API

`mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw`

```cpp
// Convenience method to quickly create a trajectory from a single waypoint.
```

### 源代码

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

ros::Publisher trajectory_pub;
geometry_msgs::PointStamped cur_pos_msg;

float linear_smoothing_navigation_step = 2;
bool gps_init_ok = false;
bool take_off_ok = false;
int tasks_ok = 0;
Eigen::Vector3d home_pos;

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
		home_pos[0] = msg.point.x;
		home_pos[1] = msg.point.y;
		home_pos[2] = msg.point.z;
		ROS_INFO("Home Position is <%f, %f, %f>", msg.point.x, msg.point.y, msg.point.z);
	}
	cur_pos_msg = msg;
	// ROS_INFO("Current Position is <%f, %f, %f>", msg.point.x, msg.point.y, msg.point.z);
}

/*
 *Description:
 *	calculate the distance between current position and target position
 *Parameters:
 *	target_pos : destination position	
 *Return:
 *	double : distance 
 */
double getDistanceToDestination(const Eigen::Vector3d& target_pos) {
	double distance = 0.0;
	distance = pow((target_pos[0] - cur_pos_msg.point.x), 2) + 
			   pow((target_pos[1] - cur_pos_msg.point.y), 2) +
			   pow((target_pos[2] - cur_pos_msg.point.z), 2);
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
bool isReachDestination(const Eigen::Vector3d& target_pos, float max_err = 0.2) {
	double cur_err = getDistanceToDestination(target_pos);
	return cur_err < max_err;
}


bool linearSmoothingNavigationTask(const Eigen::Vector3d& target_pos) {
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	if(isReachDestination(target_pos)) {
		return true;
	}
	double dist = getDistanceToDestination(target_pos);
	Eigen::Vector3d next_pos;
	if(dist < linear_smoothing_navigation_step) {
		next_pos = target_pos;
	} else {
		next_pos[0] = cur_pos_msg.point.x + 
			(target_pos[0] - cur_pos_msg.point.x) / dist * linear_smoothing_navigation_step;
		next_pos[1] = cur_pos_msg.point.y + 
			(target_pos[1] - cur_pos_msg.point.y) / dist * linear_smoothing_navigation_step;
		next_pos[2] = cur_pos_msg.point.z + 
			(target_pos[2] - cur_pos_msg.point.z) / dist * linear_smoothing_navigation_step;
	}
	double target_yaw = 0.0;
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_pos, target_yaw, &trajectory_msg);
	trajectory_pub.publish(trajectory_msg);
	
	ROS_INFO("point %d", tasks_ok);
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
	static Eigen::Vector3d target_pos(cur_pos_msg.point.x, cur_pos_msg.point.y, altitude);
	double target_yaw = 0.0;
	if(isReachDestination(target_pos)) {
		return true;
	}

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_pos, target_yaw, &trajectory_msg);
	trajectory_pub.publish(trajectory_msg);
	return false;
}

/*
 *Description:
 *	Go back to the origin position. UAV move to home_pos with current altitude. Then UAV lands.
 */
void goHome() {
	static Eigen::Vector3d home_pos_bar(home_pos[0], 
										home_pos[1],
										cur_pos_msg.point.z);
	static bool home_pos_ok = false;
	if(!home_pos_ok) {
		home_pos_ok = linearSmoothingNavigationTask(home_pos_bar);	
	} else {
		linearSmoothingNavigationTask(home_pos);
		ROS_INFO("UAV aready go back.");
	}
}

int main(int argc, char *argv[]) {
	// set local code
	setlocale(LC_ALL, "");
	// ros node init
	ros::init(argc, argv, "rotors_control_node");
	ros::NodeHandle nh;
	// create a private node for accessing node parameters
	ros::NodeHandle nh_private("~");

	std::string uav_name = "";
	ros::param::get("~mav_name", uav_name);

	// subcribe topic
	// uav position info : /odometry_sensor1/position
	ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUAVPosition);
	// global variable
	trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

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

	// trajectory
	std::vector<Eigen::Vector3d> path;
	path.push_back(Eigen::Vector3d(5.f, 5.f, 5.f));
	path.push_back(Eigen::Vector3d(-5.f, 5.f, 5.f));
	path.push_back(Eigen::Vector3d(-5.f, -5.f, 5.f));
	path.push_back(Eigen::Vector3d(5.f, -5.f, 5.f));
	path.push_back(Eigen::Vector3d(5.f, 5.f, 5.f));
	ROS_INFO("Trajectory has %d points.", path.size());

	ros::Rate loop_rate(10);
	while(ros::ok()) {
		if(gps_init_ok && !take_off_ok) {
			take_off_ok = takeOff(3);
		} else if(take_off_ok && tasks_ok < path.size()) {
			ROS_INFO("Move");
			if(tasks_ok < path.size()) {
				bool tmp = linearSmoothingNavigationTask(path[tasks_ok]);
				if(tmp) {
					tasks_ok++;
				}
			}
		} else if(tasks_ok >= path.size()) {
			goHome();
		}
		ROS_INFO("take off : %d, task number : %d", take_off_ok, tasks_ok);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
```

## 5 `launch`文件配置

在`rotors_control_test`目录下创建`launch`文件夹，在`launch`文件夹下创建`rotors_control_test.launch`文件

```xml
<launch>
    <arg name="mav_name" default="firefly"/>
    <arg name="world_name" default="basic"/>
    <arg name="enable_logging" default="false" />
    <arg name="enable_ground_truth" default="true" />
    <arg name="log_file" default="$(arg mav_name)" />
    <arg name="debug" default="false"/>
    <arg name="gui" default="true"/>
    <arg name="paused" default="false"/>
    <!-- The following line causes gzmsg and gzerr messages to be printed to the console
        (even when Gazebo is started through roslaunch) -->
    <arg name="verbose" default="false"/>

    <!--Run Gazebo-->
    <env name="GAZEBO_MODEL_PATH" value="${GAZEBO_MODEL_PATH}:$(find rotors_gazebo)/models"/>
    <env name="GAZEBO_RESOURCE_PATH" value="${GAZEBO_RESOURCE_PATH}:$(find rotors_gazebo)/models"/>
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find rotors_gazebo)/worlds/$(arg world_name).world" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="paused" value="$(arg paused)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="verbose" value="$(arg verbose)"/>
    </include>

    <!--Run UAV model and control node-->
    <group ns="$(arg mav_name)">
        <include file="$(find rotors_gazebo)/launch/spawn_mav.launch">
        <arg name="mav_name" value="$(arg mav_name)" />
        <arg name="model" value="$(find rotors_description)/urdf/mav_generic_odometry_sensor.gazebo" />
        <arg name="enable_logging" value="$(arg enable_logging)" />
        <arg name="enable_ground_truth" value="$(arg enable_ground_truth)" />
        <arg name="log_file" value="$(arg log_file)"/>
        </include>

        <node name="lee_position_controller_node" pkg="rotors_control" type="lee_position_controller_node" output="screen">
        <rosparam command="load" file="$(find rotors_gazebo)/resource/lee_controller_$(arg mav_name).yaml" />
        <rosparam command="load" file="$(find rotors_gazebo)/resource/$(arg mav_name).yaml" />
        <remap from="odometry" to="odometry_sensor1/odometry" />
        </node>

        <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
        <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

        <node name="rotors_control_node" pkg="rotors_control_test" type="rotors_control_node" output="screen">
            <param name="mav_name" type="string" value="$(arg mav_name)"/>
        </node>  
    </group>
</launch>
```

## 6 编译并运行

当前文件目录结构

```shell
| -- catkin_ws
	| -- build
	| -- devel
	| -- src
    	| -- rotors_control_test
        	| -- launch
            	| -- rotors_control_test.launch
			| -- src
            	| -- rotors_control_test.cpp
			CMakeLists.txt
		| -- mav_comm
		| -- rotors_simulator
        CMakeLists.txt
```

先切换到工作空间下，編譯，運行

```shell
# 先切换到工作空间下
cd ./catkin_ws

# 编译
catkin_make

# 运行
source ./devel/setup.bash
roslaunch rotors_control_test rotors_control_test.launch
```

## Reference 

1. 
