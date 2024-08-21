#ifndef ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H



#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>


#include "rotors_control/common.h"
#include "rotors_control/attitude_controller.h"
//#include "rotors_control/crazyflie_complementary_filter.h"

namespace rotors_control {

class AttitudeControllerNode {
public:
    AttitudeControllerNode();
    ~AttitudeControllerNode();

    void InitializeParams();
    void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void UpdateController();
    void geometryCallback(const mav_msgs::RateThrustConstPtr& rate_thrust);
    ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber geometry_sub_;
    ros::Publisher motor_velocity_reference_pub_;

    mav_msgs::EigenTrajectoryPointDeque commands_;
    std::deque<ros::Duration> command_waiting_times_;


    AttitudeController attitude_controller_;

    mav_msgs::RateThrust last_rate_thrust_;
    
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H
