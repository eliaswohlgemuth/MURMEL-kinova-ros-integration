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

        //home arm via homing_service
        ros::ServiceClient homeClient = nodeHandle_.serviceClient <kinova_msgs::HomeArm>("home_arm");
        kinova_msgs::HomeArm srv;
        if(homeClient.call(srv)){
            ROS_INFO(srv.response.homearm_result);
        } else
            ROS_ERROR("Failed to call service home_arm");

        kinova_coordinates_subscriber_ = nodeHandle_.subscribe("out/tool_pose", queue_size_, &KinovaRosController::kinovaCoordinatesCallback, this);
        camera_coordinates_subscriber_ = nodeHandle_.subscribe("out/keyhole_pose", queue_size_, &KinovaRosController::keyholeCoordinatesCallback, this);
    }

    bool KinovaRosController::readParameters(){
        
        return true;
    }

    void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped &pose) {

    }
}