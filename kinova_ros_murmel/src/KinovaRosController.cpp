#include "kinova_ros_controller/KinovaRosController.h"

namespace kinova_ros_murmel {
    
    KinovaRosController::KinovaRosController(ros::NodeHandle &nodeHandle) 
        : nodeHandle_(nodeHandle)
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
        camera_coordinates_subscriber_ = nodeHandle_.subscribe("keyhole_pose", queue_size_, &KinovaRosController::cameraCoordinatesCallback, this);

        //home arm via kinova homing_service
        ros::ServiceClient home_srv_client = nodeHandle_.serviceClient <kinova_ros_murmel::HomeArm>("home_arm");
        kinova_ros_murmel::HomeArm srv;
        if(home_srv_client.call(srv)){
            ROS_INFO_STREAM(srv.response.homearm_result);
        } else
            ROS_ERROR("Failed to call service home_arm");

        // move to manually chosen start position, which points at the tool of mulleimer
        actionlib::SimpleActionClient<kinova_ros_murmel::ArmPoseAction> home_action_client("tool_pose", true);
        kinova_ros_murmel::ArmPoseGoal goal;
        goal.pose.pose.position.x = home_x_;
        goal.pose.pose.position.y = home_y_;
        goal.pose.pose.position.z = home_z_;
        goal.pose.pose.orientation.x = home_quat_x_;
        goal.pose.pose.orientation.y = home_quat_y_;
        goal.pose.pose.orientation.z = home_quat_z_;
        goal.pose.pose.orientation.w = home_quat_w_;
        
        home_action_client.sendGoal(goal);
        bool finished_before_timeout = home_action_client.waitForResult(ros::Duration(5.0));
        if(finished_before_timeout) {
            actionlib::SimpleClientGoalState state = home_action_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else {
            ROS_INFO("Action did not finfish before time out");
        }

        kinova_coordinates_subscriber_ = nodeHandle_.subscribe("out/tool_pose", queue_size_, &KinovaRosController::kinovaCoordinatesCallback, this);
        camera_coordinates_subscriber_ = nodeHandle_.subscribe("out/keyhole_pose", queue_size_, &KinovaRosController::cameraCoordinatesCallback, this);
    }


    bool KinovaRosController::readParameters(){
        if(!nodeHandle_.getParam("home_x", home_x_)
            && !nodeHandle_.getParam("home_y", home_y_)
            && !nodeHandle_.getParam("home_z", home_z_)
            && !nodeHandle_.getParam("home_quat_x", home_quat_x_)
            && !nodeHandle_.getParam("home_quat_y", home_quat_y_)
            && !nodeHandle_.getParam("home_quat_z", home_quat_z_)
            && !nodeHandle_.getParam("home_quat_w", home_quat_w_)
            && !nodeHandle_.getParam("op_state", op_state_)) return false;
        return true;
    }

    void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped &pose) {

    }


    void KinovaRosController::kinovaMotion(){
        
    }

    enum OperationState {
        ready,
        approaching,
        insertion,
        opening,
        extracting,
    };
}