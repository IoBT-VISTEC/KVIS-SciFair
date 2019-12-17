#include <ros/ros.h>
#include "gazebo_msgs/DeleteModel.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "despawn_obj");

    gazebo_msgs::DeleteModel del;
    del.request.model_name = argv[1];

    ros::service::call("gazebo/delete_model", del);

}