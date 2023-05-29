//Jan Drozd (s5646665)

/**
 * @file   node_a_1.cpp
 * @author Jan Drozd
 * @date   3/05/2023
 * @brief  A node that implements an action client, allowing the user to set a target (x, y) or to cancel it.
 */

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_2_2022/Service1.h>
#include <iostream>

using namespace std;

/// @brief Variable to hold new goal position x-coordinate
double x_new;

/// @brief Variable to hold new goal position y-coordinate
double y_new;

/**
 * @brief Main function for node_a_1
 *
 * This node is responsible for implementing an action client that allows the user
 * to set a new target (x, y) goal or to cancel the current goal. The node interacts
 * with an action server to send goals or cancel them, and it communicates with
 * a service node to track the number of goals reached and canceled.
 *
 * @return 0 on successful execution
 */
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "node_a_1"); 
    ros::NodeHandle n;

    // Declare a service client for the /goal_info service
    ros::ServiceClient client = n.serviceClient<assignment_2_2022::Service1>("/goal_info");

    // Declare a goal object for the /goal_info service
    assignment_2_2022::Service1 goal;

    // Create a simple action client for interacting with the /reaching_goal action server
    actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
    ROS_INFO("Waiting for the action server");
    ac.waitForServer();
    ROS_INFO("Server started");

    // Declare a variable to hold the target goal
    assignment_2_2022::PlanningGoal target;

    // Declare a variable to hold user input for goal setting or cancellation
    string key;

    while (ros::ok())
    {
        cout << "Type 's' to set a new goal or 'q' to cancel the current goal" << endl;
        cin >>key;
        
        if (key == "s")
        {
            // Get new goal position from user
            std::cout << "Enter the coordinates of the new goal:\nx:\n";
            cin >> x_new;
            cout << "\ny:\n";
            cin >> y_new;
            
            // Set the target goal's position
            target.target_pose.pose.position.x = x_new;
            target.target_pose.pose.position.y = y_new;

            // Send the goal to the action server
            ac.sendGoal(target);
            ROS_INFO("You set new goal");

            // Set the new goal position as a parameter for other nodes
            ros::param::set("des_pos_x", x_new);
            ros::param::set("des_pos_y", y_new);
        }
        else if (key == "q")
        {
            // Cancel the current goal
            ac.cancelGoal();
            ROS_INFO("Goal cancelled");

            // Request goal cancellation from the /goal_info service
            goal.request.goal_count = 1;
            client.call(goal);
        }
        else
        {
            cout << "Invalid key" << endl;
        }

        ros::spinOnce();
    }
    return 0;
}
