#include "kinova_ros_controller/KinovaRosController.h"

namespace kinova_ros_murmel {

KinovaRosController::KinovaRosController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle), joint_angles_client("j2n6s300_driver/joints_action/joint_angles", true), arm_pose_client("tool_pose", true)
{
    //wait for launch of jaco to complete, since likely to take longer than current node to start

    //provide option to get parameters from parameter server at the start
    if (!readParameters())
    {
        ROS_INFO("Parameters not found");
        ros::requestShutdown();
    }


    // ROS communication setup for camera
    connection_check_client = nodeHandle_.serviceClient<kinova_ros_murmel::ConnectionCheck>("check_connection");
    camera_mode_client = nodeHandle_.serviceClient<kinova_ros_murmel::CameraMode>("set_camera_mode");
    camera_coordinates_client = nodeHandle_.serviceClient<kinova_ros_murmel::CameraCoordinates>("get_corrdinates");

    // ROS communication setup for kinova
    home_arm_client = nodeHandle_.serviceClient<kinova_ros_murmel::HomeArm>("in/home_arm");


    // // move to manually chosen start position, which points at the tool of mulleimer
    // kinova_ros_murmel::ArmJointAnglesGoal home_goal;
    // home_goal.angles.joint1 = actuator1_;
    // home_goal.angles.joint2 = actuator2_;
    // home_goal.angles.joint3 = actuator3_;
    // home_goal.angles.joint4 = actuator4_;
    // home_goal.angles.joint5 = actuator5_;
    // home_goal.angles.joint6 = actuator6_;
    // home_goal.angles.joint7 = actuator7_;
    // joint_angles_client.sendGoal(home_goal);

    // bool finished_before_timeout = joint_angles_client.waitForResult(ros::Duration(10.0));
    // if (finished_before_timeout)
    // {
    //     actionlib::SimpleClientGoalState state = joint_angles_client.getState();
    //     ROS_INFO("Homing arm finished: %s", state.toString().c_str());
    // }
    // else
    // {
    //     ROS_INFO("Homing arm did not finfish before time out");
    // }
}


bool KinovaRosController::readParameters(){
    if(!nodeHandle_.getParam("op_state", op_state_)) return false;
    return true;
}

void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped &pose) {

}


void KinovaRosController::kinovaMotion(){
    if (op_state_ == "ready") {
        ros::Duration(2).sleep();
        return;
    }
    else if (op_state_ == "retracted"){
        sendRetracted();
        return;
    }

    // following operation states result in movement of arm
    // kinova_ros_murmel::ConnectionCheck connect_srv;      //empty srv message, needed for call
    // if (connection_check_client.call(connect_srv)) { // callback function of connection_check_server is required to return bool
    //     if (op_state_ == "approaching") {
    //         ExponentialFilter f_x = ExponentialFilter(0.1);         // add path to declaration in Felix files
    //         ExponentialFilter f_y = ExponentialFilter(0.1);
    //         ExponentialFilter f_r = ExponentialFilter(0.1);

    //         PIDController p_x = PIDController(0.002, 0, 0);
    //         PIDController p_y = PIDController(0.002, 0, 0);
    //         PIDController p_z = PIDController(0.1, 0, 0);

    //         //send tracking command to camera
    //         kinova_ros_murmel::CameraMode mode_srv;
    //         mode_srv.request.camera_mode_request = "tracking";
    //         camera_mode_client.call(mode_srv);

    //         while(connection_check_client.call(connect_srv)) {
    //             double dx = 0;
    //             double dy = 0;
    //             double dz = 0;

    //             // receive Point coordinates from camera
    //             kinova_ros_murmel::CameraCoordinates coord_srv;
    //             camera_coordinates_client.call(coord_srv);      // add catching if call does not succeed
    //             dx = coord_srv.response.result.x;
    //             dy = coord_srv.response.result.y;
    //             dz = -1 * coord_srv.response.result.z;          // needs to be inverted

    //             //calculate filtered values
    //             dx = f_x.calculate(dx);                         // function calculate() from ExponentialFilter
    //             dy = f_y.calculate(dy);
    //             dz = f_r.calculate(dz);

    //             //calculate controll values
    //             dx = p_x.calculate(offset_x - dx);
    //             dy = p_y.calculate(offset_y - dy);
    //             dz = p_z.calculate(offset_z - dz);

    //             ROS_INFO_STREAM("dx: " << dx);
    //             ROS_INFO_STREAM("dy: " << dy);
    //             ROS_INFO_STREAM("dz: " << dz);

    //             //change op_state
    //             if (dz <= 0){
    //                 op_state_ = "insertion";
    //                 break;
    //             }

    //             // only move forward if robot is centered on keyhole
    //             if(!(abs(dx) < x_y_thresh && abs(dy) < x_y_thresh)){
    //                 dz = 0;
    //             }
    //             else if(dz < 0.2) {
    //                 dz = 0.2;
    //             }

    //             // get current coordinates
    //             kinova_ros_murmel::CameraCoordinates coord_srv;
    //             if(camera_coordinates_client.call(coord_srv)){
    //                 camera_x = coord_srv.response.result.x;
    //                 camera_y = coord_srv.response.result.y;
    //                 camera_z = coord_srv.response.result.z;
    //             }
    //             // send coordinates to tool_pose_action server

                
    //         }
    //         ROS_INFO("TCP connection lost.");           
    //     }
    //     else if(op_state_ == "insertion") {

    //     }

    // }
    // else {
    //     ROS_INFO("TCP connection with camera not established. Cannot start movement.");
    // }
        
}

void KinovaRosController::initHome() {
    kinova_ros_murmel::HomeArm srv;
    if (home_arm_client.call(srv))
    {
        ROS_INFO("Arm returned to home position.");
        ros::Duration(5).sleep();
    }
    else
    {
        ROS_ERROR("Failed to return arm to home position.");
    }
}

void KinovaRosController::sendRetracted() {
    ROS_INFO("Waiting for joint_angles_action server to start.");
    joint_angles_client.waitForServer();        // needs multithreading, see waitForServer() documentation

    ROS_INFO("joint_angles_action server reached.");

    kinova_ros_murmel::ArmJointAnglesGoal goal;
    goal.angles.joint1 = actuator1_;
    goal.angles.joint2 = actuator2_;
    goal.angles.joint3 = actuator3_;
    goal.angles.joint4 = actuator4_;
    goal.angles.joint5 = actuator5_;
    goal.angles.joint6 = actuator6_;
    goal.angles.joint7 = actuator7_;
    joint_angles_client.sendGoal(goal);

    bool finished_before_timeout = joint_angles_client.waitForResult(ros::Duration(7));
    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = joint_angles_client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before timeout.");
    }
}

geometry_msgs::Quaternion KinovaRosController::EulerXYZ2Quaternions(geometry_msgs::Point orientation) {
    
}

enum OperationState {
    ready,
    approaching,
    insertion,
    opening,
    extracting,
};
}