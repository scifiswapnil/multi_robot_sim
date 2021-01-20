#include <ros/ros.h>
#include <queue> 
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseArray.h>
#include <multi_robot_stage/waypointsArray.h>
#include <multi_robot_stage/stateHandler.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mission_controller_client");
    ros::NodeHandle node("~");
    std::string service_name;
    std::string robot_name;
    std::string path = ros::package::getPath("multi_robot_stage");
    if (node.getParam("robot_name", robot_name))
    {
        ROS_INFO_STREAM("Got robot name");
    }
    service_name = "/" + robot_name + "/mission_controller_digester";
    
    ros::ServiceClient client = node.serviceClient<multi_robot_stage::waypointsArray>(service_name);
    client.waitForExistence();
    path = path + "/worksheet/" + robot_name + ".txt";
    ROS_INFO_STREAM(path);
    std::string readline;
    std::ifstream worksheetfile(path);
    multi_robot_stage::waypointsArray goals;
    std_msgs::String data;
    if (worksheetfile.is_open())
    {
        while ( getline (worksheetfile,readline) )
        {
            std::cout << readline << '\n';
            data.data = readline;
            goals.request.goals.push_back(data);

        }
        worksheetfile.close();
        if (client.call(goals))
        {
            ROS_INFO_STREAM("service called with data");
        }
        else
        {
            ROS_ERROR("Failed to call service mission_controller_digester");
            return 1;
        }
    }
    else 
    {
        std::cout << "Unable to open file";
    }
}