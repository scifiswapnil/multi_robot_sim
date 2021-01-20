#include <ros/ros.h>
#include <queue> 
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <multi_robot_stage/waypointsArray.h>
#include <multi_robot_stage/stateHandler.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::map<std::string,std::string> dropids, pickids, robotids, specialids;
std::map<std::string,std::string>::iterator param_itr;
std::queue <geometry_msgs::PoseStamped> missionQueue;
bool controlThread = true;
double battery_level = 100.0;
std::string previous_control = "none";

// CANCEL
// STOP PAUSE
// START
// none

void visualizePose(std::map<std::string,std::string> idmap, ros::Publisher pub, int type)
{
    visualization_msgs::MarkerArray Markerarr;
    int i = 0;
    Markerarr.markers.resize(idmap.size());
    for (param_itr = idmap.begin(); param_itr != idmap.end(); param_itr++)
    {
        std::cout << "locations " << param_itr->first;
        std::cout << " : ";
        std::string s = param_itr->second;
        std::string delim = ",";
        auto start = 0U;
        auto end = s.find(delim);
        float xx = std::stof(s.substr(start, end - start));
        start = end + delim.length();
        end = s.find(delim, start);
        float yy = std::stof(s.substr(start, end - start));
        start = end + delim.length();
        end = s.find(delim, start);
        float yaw = std::stof(s.substr(start, end));
        std::cout << std::fixed << std::setprecision(2)<< xx << ",";
        std::cout << std::fixed << std::setprecision(2)<< yy << ",";
        tf::Quaternion q;
        tf::Matrix3x3 m;
        m.setRPY(0,0,yaw);
        m.getRotation(q);
        std::cout << std::fixed << std::setprecision(2)<< yaw << std::endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = xx;
        marker.pose.position.y = yy;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        if(type == 1)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        }else if(type == 2)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }else if(type == 3)
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        }
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        Markerarr.markers[i] = marker;
        i++;
    }
    pub.publish(Markerarr);
    std::cout << std::endl;
}

void displayPose(geometry_msgs::PoseStamped pose){
    ROS_INFO("--------- POSE ---------");
    ROS_INFO("Frame ID : %s",pose.header.frame_id.c_str());
    ROS_INFO("Seq ID : %d",pose.header.seq);
    ROS_INFO("Stamp ID : %d",pose.header.stamp);
    ROS_INFO("position x : %f",pose.pose.position.x);
    ROS_INFO("position y : %f",pose.pose.position.y);
    ROS_INFO("position z : %f",pose.pose.position.z);
    ROS_INFO("orientation x : %f",pose.pose.orientation.x);
    ROS_INFO("orientation y : %f",pose.pose.orientation.y);
    ROS_INFO("orientation z : %f",pose.pose.orientation.z);
    ROS_INFO("orientation w : %f",pose.pose.orientation.w);
}


void waypoint_executor(int x,ros::Publisher &Pulbisher)
{
    MoveBaseClient actionClient("move_base", true);
    
    while(!actionClient.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }    
    ros::Time prev = ros::Time::now();

    while(ros::ok())
    {
        if (controlThread == true)
        {
            if (ros::Time::now().toSec() - prev.toSec() >= 5.0)
            {
                battery_level -= 0.5;
                prev = ros::Time::now();
                std_msgs::Float64 bat;
                bat.data = battery_level;
                Pulbisher.publish(bat);
            }

            if (missionQueue.size()>0)
            {
                ROS_INFO("Executor : Running Mission %d",missionQueue.size());
                //debug missionqueue front
                geometry_msgs::PoseStamped tempStore;
                tempStore = missionQueue.front();
                displayPose(tempStore);
                if (tempStore.pose.orientation.z == 0.00 && tempStore.pose.orientation.w == 0.00)
                {
                    sleep(tempStore.pose.orientation.x);
                    if (missionQueue.size() > 0)
                            missionQueue.pop();
                } 
                else
                {
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = tempStore.header.frame_id;
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose.position.x = tempStore.pose.position.x;
                    goal.target_pose.pose.position.y = tempStore.pose.position.y;
                    goal.target_pose.pose.orientation.z = tempStore.pose.orientation.z;
                    goal.target_pose.pose.orientation.w = tempStore.pose.orientation.w;
                    ROS_INFO("Sending Goal");
                    actionClient.sendGoal(goal);

                    actionClient.waitForResult();

                    if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Executor: Mission %d completed",missionQueue.size());
                        if (missionQueue.size() > 0)
                            missionQueue.pop();
                    }
                    else
                    {
                        ROS_INFO("Executor: Mission %d did not complete",missionQueue.size());
                        controlThread = false;
                        previous_control = "stop";
                        ROS_INFO("Executor: Stopped",missionQueue.size());
                    }
                    
                }
            }
            else {
                usleep(500000);// no delay needed
                ROS_INFO("Executor : No missions to execute");
            }
        }
    }
}



bool controller_state_handler(multi_robot_stage::stateHandler::Request &req,
                              multi_robot_stage::stateHandler::Response &res,
                              ros::Publisher &cancelPulbisher){

        if (previous_control != req.state.data)
        {
            ROS_INFO("Current state : %s .Change  controller state to : %s",previous_control.c_str(),req.state.data.c_str());
            previous_control = req.state.data;

            if(previous_control == "clear")
            {
                controlThread = false;
                if (missionQueue.size() > 0)
                {
                    ROS_INFO("Flushing all current missions. Total : %d",missionQueue.size());
                    while(missionQueue.size() > 0)
                        missionQueue.pop();
                        ROS_INFO("Removing all missions");
                }
            }

            if(previous_control == "stop")
            {
                controlThread = false;
                ROS_INFO("Executor running : Stop");
            }

            if(previous_control == "start")
            {
                controlThread = true;
            }

            if (previous_control == "cancel")
            {
                controlThread = false;
                ROS_INFO("Executor : Cancelling current mission");
                actionlib_msgs::GoalID temp;
                cancelPulbisher.publish(temp);

            }
            //kill current mission 
            res.status.data = true;
        }
        else
        {
            res.status.data = false;   
        }
    return true;
}


geometry_msgs::PoseStamped findid(std::map<std::string,std::string> idmap, std::string query)
{
    geometry_msgs::PoseStamped result;
    param_itr = idmap.find(query);
    float timeToWait = 0.0;

    if (param_itr == idmap.end())
    {
        if (query.at(0) == 'W' || query.at(0) == 'I')
        {
            auto start = query.find(",");
            auto end = query.length();
            ROS_INFO_STREAM(query.substr(start+1, end - start));
            timeToWait = std::stof(query.substr(start+1, end - start));
            ROS_INFO_STREAM(timeToWait);
            result.pose.orientation.x = timeToWait;
            result.pose.orientation.z = 0.00;
            result.pose.orientation.w = 0.00;
            return result;
        }
        else 
        {
            ROS_INFO_STREAM("no pose");
        }
    }
    else 
    {
        if (query == "W" || query == "I" )
        {
            timeToWait = std::stof(param_itr->second); //delay
            result.pose.orientation.x = timeToWait;
            result.pose.orientation.z = 0.00;
            result.pose.orientation.w = 0.00;
            return result;
        }   
        std::string s = param_itr->second;
        std::string delim = ",";
        auto start = 0U;
        auto end = s.find(delim);
        float xx = std::stof(s.substr(start, end - start));
        start = end + delim.length();
        end = s.find(delim, start);
        float yy = std::stof(s.substr(start, end - start));
        start = end + delim.length();
        end = s.find(delim, start);
        float yaw = std::stof(s.substr(start, end));
        std::cout << std::fixed << std::setprecision(2)<< xx << ",";
        std::cout << std::fixed << std::setprecision(2)<< yy << ",";
        tf::Quaternion q;
        tf::Matrix3x3 m;
        m.setRPY(0,0,yaw);
        m.getRotation(q);
        result.header.frame_id = "map";
        result.header.stamp = ros::Time::now();
        result.pose.position.x = xx;
        result.pose.position.y = yy;
        result.pose.orientation.z = q[2];
        result.pose.orientation.w = q[3];
    }
    return result;
}


bool controller_digester (multi_robot_stage::waypointsArray::Request &req,
                          multi_robot_stage::waypointsArray::Response &res)
{ 
    ROS_INFO("Got new mission ");
    std::vector<std_msgs::String> test;
    test = req.goals; 
    ROS_INFO("Number of waypoints %d", test.size());
    for (size_t i = 0; i < test.size(); i++)
    {
        ROS_INFO_STREAM(test[i].data);
        std::string temp = test[i].data;
        if (temp.at(0) == 'P')
        {
            ROS_INFO_STREAM("got parking loc");
            displayPose(findid(pickids,temp));
            missionQueue.push(findid(pickids,temp));
        }
        else if (temp.at(0) == 'R')
        {
            ROS_INFO_STREAM("got robot loc");
            displayPose(findid(robotids,temp));
            missionQueue.push(findid(robotids,temp));
        }
        else if(temp.at(0) == 'W' || temp.at(0) == 'I')
        {
            ROS_INFO_STREAM("got wait loca");
            displayPose(findid(specialids,temp));
            missionQueue.push(findid(specialids,temp));
        }
        else 
        {
            ROS_INFO_STREAM("got drop loc");
            displayPose(findid(dropids,temp));
            missionQueue.push(findid(dropids,temp));
        }
    }
    res.response.data = true;
    return true;
}



int main(int argc, char** argv){
    
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle node;
    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    ros::Publisher marker_pub_goal = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_goal", 1);
    ros::Publisher marker_pub_robot = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_robot", 1);
    sleep(2);
    if (node.getParam("/drop_locations", dropids))
    {
        visualizePose(dropids,marker_pub,1);
    }
    if (node.getParam("/pick_locations", pickids))
    {
        visualizePose(pickids,marker_pub_goal,2);
    }
    if (node.getParam("/robot_locations", robotids))
    {
        visualizePose(robotids,marker_pub_robot,3);
    }
    if (node.getParam("/special_behaviour", specialids))
    {
        // visualizePose(robotids,marker_pub_robot);
    }
    ros::Publisher move_base_cancel_publisher = node.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::Publisher battery_publisher = node.advertise<std_msgs::Float64>("battery_state", 1);
    ros::ServiceServer waypoints_digester = node.advertiseService("mission_controller_digester", controller_digester);
    ros::ServiceServer state_handler = node.advertiseService<multi_robot_stage::stateHandler::Request,multi_robot_stage::stateHandler::Response>("mission_controller_state",boost::bind(controller_state_handler,_1,_2,boost::ref(move_base_cancel_publisher)));
    ROS_INFO("Mission controller started");
    std::thread thread1(waypoint_executor,1,boost::ref(battery_publisher));
    ros::spin();
}