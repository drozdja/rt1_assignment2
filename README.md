# Research Track - Assignment 2
=============================

Description
------------
- (node_a_1) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it.
- (node_a_2) Anode publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom
- (node_b) A service node that, when called, prints the number of goals reached and cancelled;
- (node_c) A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed. Use a parameter to set how fast the node publishes the information. 

Installation
-------

Create a catjing workspace:

```shell
$ mkdir rt1_assignment2/src
$ cd rt1_assignment2/src
$ catkin_init_workspace
```

Clone this repository to the source folder in created workspace

```shell
$ gh repo clone drozdja/rt1_assignment2
```

Build and source the project

```shell
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

Running
-------

```shell
$ roslaunch assignment_2_2022 rt1_ass2.launch 
```

Pseudocode of node_a_1
---------

```
initialize ROS node
create a service client for the goal_info service
create an action client for the reaching_goal action
while (ros is running)
    prompt user to enter 's' to set a new goal or 'q' to cancel current goal
    if user enters 's'
        get new x and y goal coordinates from user
        set the target goal's x and y position to the new coordinates
        send the goal to the action server
        set the new x and y goal coordinates as parameters for other nodes
    else if user enters 'q'
        cancel the current goal
        request goal cancellation from the goal_info service
    else
        print "Invalid key"
    spin ROS once

```

Pseudocode of node_a_2
---------

```

initialize ROS node
subscribe to the odometry topic and register a callback function to update global x, y, vel_x, vel_y
advertise a topic to publish the robot data
get the frequency parameter
while (ros is running)
    create a message to publish the robot data with x, y, vel_x, vel_y
    publish the message
    sleep for 1/frequency seconds
    spin ROS once
    
```

ROS graph of the project generated by rqt-graph
---------
![rqt_graph](https://user-images.githubusercontent.com/40939177/215017188-08a8f598-650e-478a-9d3b-b9c53228e629.png)



