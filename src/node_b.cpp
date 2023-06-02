// Jan Drozd (s5646665)

/**
 * @file   node_b.cpp
 * @author Jan Drozd
 * @date   3/05/2023
 * @brief  A service node that, when called, prints the number of goals reached and cancelled.
 */

#include <ros/ros.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_2_2022/Service1.h>
#include <unistd.h>
#include <math.h>

using namespace ros;

/// @brief Initialize global variables for counting reached and cancelled goals
int goals_reached = 0;
int goals_cancelled = 0;

/**
 * @brief Status callback function
 * 
 * @param msg Pointer to the received PlanningActionResult message
 */
void status_callback(const assignment_2_2022::PlanningActionResult::ConstPtr &msg)
{
    if (msg->status.status == 3)
    { // if the status of the message is 3 (reached goal)
        ROS_INFO("The robot has reached the goal");
        goals_reached += 1; // increment the number of reached goals
        ROS_INFO("Reached goals: %d\nCancelled goals: %d", goals_reached, goals_cancelled);
    }
}

/**
 * @brief Goal count function
 *
 * @param req Service request
 * @param res Service response
 * @return true on successful execution
 */
bool goal_count(assignment_2_2022::Service1::Request &req, assignment_2_2022::Service1::Response &res)
{
    goals_cancelled += 1; // increment the number of cancelled goals
    res.reached = goals_reached;
    res.cancelled = goals_cancelled;
    ROS_INFO("Goals:\nReached: %d,\nCancelled: %d", res.reached, res.cancelled);
    return true;
}

/**
 * @brief Main function for node_b
 *
 * This node is responsible for providing a service that prints the number of goals reached and
 * cancelled. Additionally, it subscribes to the action result topic to update the number of
 * reached goals when the robot reaches a goal.
 *
 * @return 0 on successful execution
 */
int main(int argc, char **argv)
{
    init(argc, argv, "node_b"); // initialize node
    NodeHandle n;

    ServiceServer service = n.advertiseService("/goal_info", goal_count);     // create service for goal number
    Subscriber sub = n.subscribe("/reaching_goal/result", 1, status_callback); // create subscriber for goal status

    spin(); // keep running until program is closed

    return 0;
}
