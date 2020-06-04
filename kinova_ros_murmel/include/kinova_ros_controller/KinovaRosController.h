#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_ros_murmel/HomeArm.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinova_ros_murmel/ArmPoseAction.h>
// #include messages


namespace kinova_ros_murmel {

class KinovaRosController {
    public:
        KinovaRosController(ros::NodeHandle &nodeHandle);
        ~KinovaRosController(){}

        bool readParameters();
        bool isHomed();
        

    private:
        ros::NodeHandle nodeHandle_;
        ros::Subscriber kinova_coordinates_subscriber_;
        ros::Subscriber camera_coordinates_subscriber_;
        const int queue_size_ = 10;

        //custom home postition coordinates
        double home_x_;
        double home_y_;
        double home_z_;
        double home_quat_x_;
        double home_quat_y_;
        double home_quat_z_;
        double home_quat_w_;

        void kinovaCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
        void cameraCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
};
}