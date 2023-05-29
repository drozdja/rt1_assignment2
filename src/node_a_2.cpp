// Jan Drozd (s5646665)

/**
 * @file   node_a_2.cpp
 * @author Jan Drozd
 * @date   3/05/2023
 * @brief  A node that publishes the robot position and velocity as a custom message (x, y, vel_x, vel_y),
 *         relying on the values published on the topic /odom.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2022/Message1.h>

using namespace ros;

// Global variables to hold the odometry data
double x, y, vel_x, vel_y, freq;

// Callback function to update the global variables with the odometry data
void data_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    vel_x = msg->twist.twist.linear.x;
    vel_y = msg->twist.twist.linear.y;
}

/**
 * @brief Main function for node_a_2
 *
 * This node is responsible for subscribing to the robot's odometry data (position and velocity)
 * published on the /odom topic. It then publishes the position and velocity as a custom message
 * (x, y, vel_x, vel_y) to be consumed by other nodes.
 *
 * @return 0 on successful execution
 */
int main(int argc, char **argv)
{
    init(argc, argv, "node_a_2"); // initialize node
    NodeHandle nh;

    // Subscribe to the odometry topic and register the callback function
    Subscriber sub = nh.subscribe("/odom", 1, data_callback);
    
    // Advertise a topic to publish the robot data
    Publisher pub = nh.advertise<assignment_2_2022::Message1>("robot_data", 1);

    // Get the frequency parameter
    param::get("freq", freq);
    Rate rate(freq);

    while (ok())
    {
        // Create a message to publish the robot data
        assignment_2_2022::Message1 data;
        data.x = x;
        data.y = y;
        data.vel_x = vel_x;
        data.vel_y = vel_y;
        pub.publish(data);

        rate.sleep();
        spinOnce();
    }

    return 0;
}
