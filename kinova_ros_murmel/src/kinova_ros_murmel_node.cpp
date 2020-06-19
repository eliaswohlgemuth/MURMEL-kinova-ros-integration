#include <ros/ros.h>
#include "kinova_ros_controller/KinovaRosController.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinova_ros_murmel_node");
    ros::NodeHandle nodeHandle("~");

    kinova_ros_murmel::KinovaRosController kinovaRosController(nodeHandle);
    while(ros::ok()){
        kinovaRosController.readParameters();
        ROS_INFO_STREAM("Node running. Op_state: " << kinovaRosController.getOpstate());
        ros::spinOnce();
        kinovaRosController.kinovaMotion();
        ros::Duration(2).sleep();
    }
    
    return 0;
}