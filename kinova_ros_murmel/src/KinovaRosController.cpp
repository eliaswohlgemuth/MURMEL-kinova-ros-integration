#include "kinova_ros_controller/KinovaRosController.h"

namespace kinova_ros_murmel {

KinovaRosController::KinovaRosController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle), joint_angles_client("joint_angles", true), arm_pose_client("tool_pose", true)
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


    // move to manually chosen start position, which points at the tool of mulleimer
    kinova_ros_murmel::ArmJointAnglesGoal home_goal;
    home_goal.angles.joint1 = actuator1_;
    home_goal.angles.joint2 = actuator2_;
    home_goal.angles.joint3 = actuator3_;
    home_goal.angles.joint4 = actuator4_;
    home_goal.angles.joint5 = actuator5_;
    home_goal.angles.joint6 = actuator6_;
    home_goal.angles.joint7 = actuator7_;
    joint_angles_client.sendGoal(home_goal);

    bool finished_before_timeout = joint_angles_client.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = joint_angles_client.getState();
        ROS_INFO("Homing arm finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Homing arm did not finfish before time out");
    }
}


bool KinovaRosController::readParameters(){
    if(!nodeHandle_.getParam("op_state", op_state_)) return false;
    return true;
}

void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped &pose) {

}


void KinovaRosController::kinovaMotion(){
    if (op_state_ == "ready") {
        ros::Duration(2).sleep;
        return;
    }

    // following operation states result in movement of arm
    kinova_ros_murmel::ConnectionCheck connect_srv;      //empty srv message, needed for call
    if (connection_check_client.call(connect_srv)) { // callback function of connection_check_server is required to return bool
        if (op_state_ == "approaching") {
            ExponentialFilter f_x = ExponentialFilter(0.1);         // add path to declaration in Felix files
            ExponentialFilter f_y = ExponentialFilter(0.1);
            ExponentialFilter f_r = ExponentialFilter(0.1);

            PIDController p_x = PIDController(0.002, 0, 0);
            PIDController p_y = PIDController(0.002, 0, 0);
            PIDController p_z = PIDController(0.1, 0, 0);

            //send tracking command to camera
            kinova_ros_murmel::CameraMode mode_srv;
            mode_srv.request.camera_mode_request = "tracking";
            camera_mode_client.call(mode_srv);

            while(connection_check_client.call(connect_srv)) {
                double dx = 0;
                double dy = 0;
                double dz = 0;

                // receive Point coordinates from camera
                kinova_ros_murmel::CameraCoordinates coord_srv;
                camera_coordinates_client.call(coord_srv);      // add catching if call does not succeed
                dx = coord_srv.response.result.x;
                dy = coord_srv.response.result.y;
                dz = -1 * coord_srv.response.result.z;          // needs to be inverted

                //calculate filtered values
                dx = f_x.calculate(dx);                         // function calculate() from ExponentialFilter
                dy = f_y.calculate(dy);
                dz = f_r.calculate(dz);

                //calculate controll values
                dx = p_x.calculate(offset_x - dx);
                dy = p_y.calculate(offset_y - dy);
                dz = p_z.calculate(offset_z - dz);

                ROS_INFO_STREAM("dx: " << dx);
                ROS_INFO_STREAM("dy: " << dy);
                ROS_INFO_STREAM("dz: " << dz);

                //change op_state
                if (dz <= 0){
                    op_state_ = "insertion";
                    break;
                }

                // send coordinates to tool_pose_action server
                
            }
            ROS_INFO("TCP connection lost.");           
        }
        else if(op_state_ == "insertion") {

        }

    }
    else {
        ROS_INFO("TCP connection with camera not established. Cannot start movement.");
    }
        
}

enum OperationState {
    ready,
    approaching,
    insertion,
    opening,
    extracting,
};
}