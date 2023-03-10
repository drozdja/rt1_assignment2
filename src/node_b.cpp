// Jan Drozd (s5646665)

#include <ros/ros.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_2_2022/Service1.h>
#include <unistd.h>
#include <math.h>

using namespace ros;

// initialize global variables for counting reached and cancelled goals
int goals_reached = 0;
int goals_cancelled = 0;

// status callback function
void status_callback(const assignment_2_2022::PlanningActionResult::ConstPtr &msg)
{
    if (msg->status.status == 3)
    { // if the status of the message is 3 (reached goal)
        ROS_INFO("The robot has reached the goal");
        goals_reached += 1; // increment the number of reached goals
        ROS_INFO("Reached goals: %d\nCancelled goals: %d", goals_reached, goals_cancelled);
    }
}

// goal count function
bool goal_count(assignment_2_2022::Service1::Request &req, assignment_2_2022::Service1::Response &res)
{
    goals_cancelled += 1; // increment the number of cancelled goals
    res.reached = goals_reached;
    res.cancelled = goals_cancelled;
    ROS_INFO("Goals:\nReached: %d,\nCancelled: %d", res.reached, res.cancelled);
    return true;
}

int main(int argc, char **argv)
{
    init(argc, argv, "node_b"); // initialize node
    NodeHandle n;

    ServiceServer service = n.advertiseService("/goal_info", goal_count);     // create service for goal number
    Subscriber sub = n.subscribe("/reaching_goal/result", 1, status_callback); // create subscriber for goal status

    spin(); // keep running until program is closed

    return 0;
}
