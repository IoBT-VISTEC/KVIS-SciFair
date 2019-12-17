#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"
#include <iostream>

void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->name[0]);
  std::cout << msg->name[1] << std::endl; //house
  std::cout << msg->name[2] << std::endl; //turtlebot3
  std::cout << msg->pose[2].position.x << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, stateCallback);
    ros::spin();
    return 0;
}