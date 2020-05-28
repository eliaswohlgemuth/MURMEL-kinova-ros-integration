#include <ros/ros.h>
#include "kinova_ros_controller/KinovaRosController.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinova_ros_murmel");
    ros::NodeHandle nodeHandle("~");

    kinova_ros_murmel::KinovaRosController kinovaRosController(nodeHandle);

    ros::spin();
    return 0;
}