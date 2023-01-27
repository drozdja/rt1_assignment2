//Jan Drozd (s5646665)

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_2_2022/Service1.h>
#include <iostream>

using namespace std;

// Declare variable to hold new goal position
double x_new, y_new;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_a_1"); // initialize node
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<assignment_2_2022::Service1>("/goal_info");
    assignment_2_2022::Service1 goal; // Declare a goal object

    // Create a simple action client for interacting with the action server
    actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
    ROS_INFO("Waiting for the action server");
    ac.waitForServer();
    ROS_INFO("Server started");

    // Declare a variable to hold the target goal
    assignment_2_2022::PlanningGoal target;
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

            // Request goal cancellation from the goal_info service
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
