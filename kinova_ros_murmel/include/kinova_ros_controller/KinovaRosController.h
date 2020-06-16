#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_ros_murmel/HomeArm.h>
#include <kinova_ros_murmel/CameraCoordinates.h>
#include <kinova_ros_murmel/ConnectionCheck.h>
#include <kinova_ros_murmel/CameraMode.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinova_ros_murmel/ArmPoseAction.h>
#include <kinova_ros_murmel/ArmJointAnglesAction.h>
// #include messages


namespace kinova_ros_murmel {

class KinovaRosController {
    public:
        KinovaRosController(ros::NodeHandle &nodeHandle);
        ~KinovaRosController(){}
        void kinovaMotion(); // contains Felix Kueblers logic to perform mulleimer opening sequence

        bool readParameters();
        bool isHomed();
        

    private:
        ros::NodeHandle nodeHandle_;
        // camera communication
        ros::ServiceClient camera_coordinates_client;     // get camera coordinates
        ros::ServiceClient camera_mode_client;              // send cameras operating mode 
        ros::ServiceClient connection_check_client;         // checks client.isConnected()
        // kinova communication
        actionlib::SimpleActionClient<kinova_ros_murmel::ArmJointAnglesAction> joint_angles_client;
        actionlib::SimpleActionClient<kinova_ros_murmel::ArmPoseAction> arm_pose_client;
        ros::Subscriber kinova_coordinates_subscriber_;


        const int queue_size_ = 10;

        //custom home postition coordinates
        const float actuator1_ = 275;
        const float actuator2_ = 200;
        const float actuator3_ = 25;
        const float actuator4_ = 180;
        const float actuator5_ = 268;
        const float actuator6_ = 180;
        const float actuator7_ = 0;

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