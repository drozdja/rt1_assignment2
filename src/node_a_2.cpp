// Jan Drozd (s5646665)

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
