#include "kinova_ros_controller/KinovaRosController.h"

namespace kinova_ros_murmel {

KinovaRosController::KinovaRosController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle), 
    joint_angles_client("j2n6s300_driver/joints_action/joint_angles", true), 
    arm_pose_client("tool_pose", true)
{
    //wait for launch of jaco to complete, since likely to take longer than current node to start

    //provide option to get parameters from parameter server at the start
    if (!readParameters())
    {
        ROS_INFO("Parameters not found");
        ros::requestShutdown();
    }


    // ROS communication setup for camera
    camera_mode_client = nodeHandle_.serviceClient<kinova_ros_murmel::CameraMode>("set_camera_mode");
    camera_data_client = nodeHandle_.serviceClient<kinova_ros_murmel::CameraData>("get_corrdinates");

    // ROS communication setup for kinova
    home_arm_client = nodeHandle_.serviceClient<kinova_ros_murmel::HomeArm>("in/home_arm");

    // create Exp-filters and PID controllers, could make implemenation of no-args constructor necessary
    f_prob = f_x = f_y = f_z = f_theta_x = f_theta_y = ExponentialFilter(exp_w);
    p_x = p_y = p_z = PIDController(pid_p, pid_i, pid_d);
    p_theta_x = p_theta_y = PIDController(0.01, pid_i, pid_d);

    is_first_init = true;
}


bool KinovaRosController::readParameters(){
    if(!nodeHandle_.getParam("op_state", op_state_)) return false;
    return true;
}

void KinovaRosController::kinovaCoordinatesCallback(const geometry_msgs::PoseStamped &pose) {

}


void KinovaRosController::kinovaMotion(){
    if (op_state_ == "ready"){
        ros::Duration(2).sleep();
        return;
    }
    if (op_state_ == "calibrate"){
        initHome();
        is_first_init = false;
    }
    else if (op_state_ == "retracted"){ // sendRetracted (joint_angles_client) does not have self collision check
        // check if arm is homed, if not -> home for calibration
        if(is_first_init){
            initHome();
            is_first_init = false;
        }
        sendRetracted();
        return;
    }
    else if (op_state_ == "open"){
        // start from homeposition to make sure arm does not self-collide on moving to retracted position

        // move to retracted to position camera onto keyhole
        sendRetracted();
        openTrashcanDemo();
        sendRetracted();
        return;
    }

        
}

// does not produce same results after using sendRetracted() -> change sendRetraceted() to communicating trough Quaternions or implement own homing function
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
    ROS_INFO("Moving arm to retracted position.");
    ROS_INFO("Waiting for joint_angles_action server to start.");
    joint_angles_client.waitForServer();

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


void KinovaRosController::openTrashcanDemo(){
    //----------------------------------------------
    // A P P R O A C H 
    //----------------------------------------------
    //send tracking command to camera
    kinova_ros_murmel::CameraMode mode_srv;
    mode_srv.request.request = "tracking";
    camera_mode_client.call(mode_srv);

    // mystic counter 
    int counter = 0;

    while(true){
        double probability = 0;
        double dx = 0;
        double dy = 0;
        double dz = 0;

        double theta_x = 0;
        double theta_y = 0;


        // receive Point coordinates from camera
        kinova_ros_murmel::CameraData data_srv;
        if(camera_data_client.call(data_srv)){
            probability = data_srv.response.probability;
            dx = data_srv.response.coordinates.x / 1000;          // convert to meters
            dy = data_srv.response.coordinates.y / 1000;
            dz = -1 * data_srv.response.coordinates.z / 1000;     // needs to be inverted
            theta_x = data_srv.response.theta_x;
            theta_y = data_srv.response.theta_y;
        }
        else {
            ROS_INFO("Could not receive camera data.");           // !!diverge program flow accordingly!!
        }           

        // SKIPPING OUTPUT OF STRINGSTREAM
        
        if(counter++ > 10 && probability > 0){
            // calculate filtered values
            probability = f_prob.calculate(probability);
            dx = f_x.calculate(dx);
            dy = f_y.calculate(dy);
            dz = f_z.calculate(dz);
            theta_x = f_theta_x.calculate(theta_x);
            theta_y = f_theta_y.calculate(theta_y);

            // calculate control values
            // speed needs to be influenced by distance
            // values are speed values from here on
            dx = p_x.calculate(controller_offset_x - dx);
            dy = p_y.calculate(controller_offset_y - dy);
            dz = p_z.calculate(controller_offset_z - dz);
            theta_x = p_theta_x.calculate(controller_offset_theta_x - theta_x);
            theta_y = p_theta_y.calculate(controller_offset_theta_y - theta_y);

            // move forward to insertion
            if(dz <= 0)
            {
                break;
            }

            // only move forward if robot is centered on keyhole
            if(!((abs(dx) < x_y_thresh && abs(dy) < x_y_thresh)))   // what is this condition checking?
            {  
                dz = 0;
            }
            else if(dz < 0.2)      // continuing with 0.2, is the next iteration changing dz to 0? otherwise endless loop
            {
                // speed below 0.1 are not usable for servoing
                dz = 0.2;
            }

            // how does speed get translated into total distance moved? 
            
        }
    }
}

geometry_msgs::Quaternion KinovaRosController::EulerXYZ2Quaternions(geometry_msgs::Point orientation) {
    
}

enum OperationState {
    ready,
    calibrate,
    retracted,
    open,
};
}