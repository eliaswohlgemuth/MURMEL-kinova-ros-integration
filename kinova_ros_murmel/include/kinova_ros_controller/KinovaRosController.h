#pragma once

#include <ros/ros.h>
#include <kinova-ros/kinova_msgs/HomeArm.h>
#include <geometry_msgs/PoseStamped.h>
// #include messages


namespace kinova_ros_murmel {

class KinovaRosController {
    public:
        KinovaRosController(ros::NodeHandle &nodeHandle);
        ~KinovaRosController(){}

        bool KinovaRosController::readParameters();
        bool KinovaRosController::isHomed();
        

    private:
        ros::NodeHandle nodeHandle_;
        ros::Subscriber kinova_coordinates_subscriber_;
        ros::Subscriber camera_coordinates_subscriber_;
        const int queue_size_ = 10;


        void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
        void KinovaRosController::keyholeCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
};
}