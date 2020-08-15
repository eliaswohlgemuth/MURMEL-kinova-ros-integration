#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <kinova_ros_murmel/PoseVelocity.h>
#include <kinova_msgs/KinovaPose.h>

#include <kinova_ros_murmel/HomeArm.h>
#include <kinova_ros_murmel/CameraData.h>
#include <kinova_ros_murmel/ConnectionCheck.h>
#include <kinova_ros_murmel/CameraMode.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinova_ros_murmel/ArmPoseAction.h>
#include <kinova_ros_murmel/ArmJointAnglesAction.h>

#include <ExponentialFilter.hpp>
#include <PIDController.hpp>
#include <KinovaTypes.h>


namespace kinova_ros_murmel {

class KinovaRosController {
    public:
        KinovaRosController(ros::NodeHandle &nodeHandle);
        ~KinovaRosController(){}
        void kinovaMotion(); // contains Felix Kueblers logic to perform mulleimer opening sequence
        void openTrashcanDemo();
        void initHome();
        void sendRetracted();

        void kinovaCoordinatesCallback(const kinova_msgs::KinovaPose &pose);

        CartesianInfo convertReferenceFrame(const CartesianInfo &target_velocity);
        PoseVelocity convert2PoseVelocity(const CartesianInfo &target);

        geometry_msgs::PoseStamped EulerXYZ2Quaternions(const CartesianInfo &target_pos); // point as type fits for describing 

        bool readParameters();
        bool isHomed();

        std::string getOpstate()
        {
            return op_state_;
        }

    private:
        ros::NodeHandle nodeHandle_;

        // camera communication
        ros::ServiceClient camera_data_client;       // get camera coordinates
        ros::ServiceClient camera_mode_client;              // send cameras operating mode 

        // kinova communication
        actionlib::SimpleActionClient<kinova_ros_murmel::ArmJointAnglesAction> joint_angles_client;
        actionlib::SimpleActionClient<kinova_ros_murmel::ArmPoseAction> arm_pose_client;

        ros::ServiceClient home_arm_client;

        ros::Subscriber kinova_coordinates_subscriber_;

        ros::Publisher cartesian_velocity_publisher_;
    

        bool is_first_init;
        const int queue_size_ = 10;

        // current kinova coordinates
        CartesianInfo kinova_coordinates;

        //custom home postition coordinates
        const float actuator1_ = 275;
        const float actuator2_ = 200;
        const float actuator3_ = 25;
        const float actuator4_ = 180;
        const float actuator5_ = 268;
        const float actuator6_ = 180;
        const float actuator7_ = 0;

        //OperationState op_state_;
        std::string op_state_;

        // dz distance to trashcan thresh
        const double dz_min = 0.04;

        // offset for servoing
        const int controller_offset_x = 0;
        const int controller_offset_y = 0;
        const int controller_offset_z = -0.08 - dz_min;
        const double controller_offset_theta_x = 0;
        const double controller_offset_theta_y = 0;

        const double correction_offset_x = -0.013;
        const double correction_offset_y = -0.09;
        const double correction_offset_z = dz_min;

        ExponentialFilter f_prob, f_x, f_y, f_z, f_theta_x, f_theta_y;
        PIDController p_x, p_y, p_z, p_theta_x, p_theta_y;

        // x and y control error threshold
        const int x_y_thresh = 0.3;

        // exponential filter factor
        const double exp_w = 0.2;

        // gain of PID controller
        const double pid_p = 4;

        // integral value of PIS controller
        const double pid_i = 0;

        // differential value of PID controller
        const double pid_d = 0;

        
};
}