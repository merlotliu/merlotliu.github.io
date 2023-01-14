---
title: Ros MISC
comments: true
date: 2022-08-01 22:00:06
updated: 2022-08-01 22:00:06
tags: [ROS]
categories:
- [ROS, ROS-MISC]
---

# Ros MISC

## msg

### trajectory_msgs/MultiDOFJointTrajectory

#### File:trajectory_msgs/MultiDOFJointTrajectory.msg

##### Raw Message Definition

```xml
# The header is used to specify the coordinate frame and the reference time for the trajectory durations
Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points
```

##### Compact Message Definition

```xml
[std_msgs/Header](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html) header
string[] joint_names
[trajectory_msgs/MultiDOFJointTrajectoryPoint[\]](http://docs.ros.org/en/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html) points
```

#### Usage

```
rostopic pub -1 /firefly/command/trajectory trajectory_msgs/MultiDOFJointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- transforms:
  - translation:
      x: 5.0
      y: 5.0
      z: 5.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  velocities:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  accelerations:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  time_from_start:
    secs: 5
    nsecs: 0
- transforms:
  - translation:
      x: 5.0
      y: -5.0
      z: 5.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  velocities:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  accelerations:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  time_from_start:
    secs: 10
    nsecs: 0
- transforms:
  - translation:
      x: -5.0
      y: 5.0
      z: 5.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  velocities:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  accelerations:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  time_from_start:
    secs: 15
    nsecs: 0
- transforms:
  - translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  velocities:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  accelerations:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  time_from_start:
    secs: 20
    nsecs: 0"
```



## API





## Reference 

1. [trajectory_msgs/MultiDOFJointTrajectory Documentation (ros.org)](http://docs.ros.org/en/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html)
