#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_ros_murmel/HomeArm.h>
#include <kinova_ros_murmel/ConnectionCheck.h>
#include <kinova_ros_murmel/CameraMode.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinova_ros_murmel/ArmPoseAction.h>
// #include messages


namespace kinova_ros_murmel {

class KinovaRosController {
    public:
        KinovaRosController(ros::NodeHandle &nodeHandle);
        ~KinovaRosController(){}
        void kinovaMotion();

        bool readParameters();
        bool isHomed();
        

    private:
        ros::NodeHandle nodeHandle_;
        // camera communication
        ros::Subscriber camera_coordinates_subscriber_;
        ros::ServiceClient camera_mode_client;
        ros::ServiceClient connection_check_client;
        // kinova communication
        ros::Subscriber kinova_coordinates_subscriber_;
        const int queue_size_ = 10;

        //custom home postition coordinates
        double home_x_;
        double home_y_;
        double home_z_;
        double home_quat_x_;
        double home_quat_y_;
        double home_quat_z_;
        double home_quat_w_;

        // coordinates from orbecc camera

        //OperationState op_state_;
        std::string op_state_;

        // offset for servoing
        const int offset_x = 15;
        const int offset_y = -55;
        const int offset_z = -150;
        // x and y control error threshold
        const int x_y_thresh = 10;
        // dz distance to trash can thresh
        const double dz_speed = 0.2;


        void kinovaCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
        void cameraCoordinatesCallback(const geometry_msgs::PoseStamped& pose);
        
};
}