// Jan Drozd (s5646665)

/**
 * @file   node_c.cpp
 * @author Jan Drozd
 * @date   3/05/2023
 * @brief  A node that subscribes to the robot's position and velocity (using the custom message),
 *         and prints the distance of the robot from the target and the robot's average speed.
 */

#include <ros/ros.h>
#include <assignment_2_2022/Message1.h>
#include <math.h>

using namespace ros;

/// @brief Declare variables in the global scope
double p_x, p_y, v_x, v_y;
double vel = 0.0;
int count = 1;

/**
 * @brief Helper function for computing distance
 *
 * @param p_x Robot's x position
 * @param p_y Robot's y position
 * @return Distance between the robot and the target
 */
double distance(double p_x, double p_y){
    double t_x, t_y, dist;
    // Get the desired position from parameters
    param::get("des_pos_x",t_x);
    param::get("des_pos_y",t_y);
    // Compute the distance
    dist = sqrt(pow((t_x - p_x), 2.0) + pow((t_y - p_y), 2.0));
    return dist;
}

/**
 * @brief Helper function for computing average speed
 *
 * @param v_x Robot's x velocity
 * @param v_y Robot's y velocity
 * @return Average speed of the robot
 */
double average_speed(double v_x, double v_y){
    double ave_vel;
    // Accumulate the velocities
    vel += sqrt(pow(v_x,2.0) + pow(v_y, 2.0));
    ave_vel = vel/count;
    return ave_vel;
}

/**
 * @brief Callback function for processing robot data
 *
 * @param data Pointer to the received custom message
 */
void dataCallback(const assignment_2_2022::Message1::ConstPtr& data){
    p_x = data->x;
    p_y = data->y;
    v_x = data->vel_x;
    v_y = data->vel_y;
    // Print distance and average speed
    ROS_INFO("Distance to goal: %4.2f", distance(p_x, p_y));
    ROS_INFO("Average speed: %4.2f", average_speed(v_x, v_y));
    // Update count
    count += 1;
}

/**
 * @brief Main function for node_c
 *
 * This node subscribes to the robot's position and velocity and calculates the distance
 * between the robot and the target, as well as the average speed of the robot.
 *
 * @return 0 on successful execution
 */
int main(int argc, char **argv){

    init(argc, argv, "node_c"); // initialize node
    NodeHandle nh;
    // Create a subscriber and attach the callback function
    Subscriber sub = nh.subscribe("robot_data", 10, dataCallback);
    // Spin to process callbacks
    spin();
    return 0;
}
