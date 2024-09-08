/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2020 Ria Sonecha, MIT, USA
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "attitude_controller_node_v2.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"

namespace rotors_control {

AttitudeControllerNode::AttitudeControllerNode() {

    ROS_INFO("Started attitude controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

        // Subscribe to command trajectory! Subscribe, so it will be able to get commands on where to go.
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &AttitudeControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
      &AttitudeControllerNode::OdometryCallback, this);

          // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // Subscribe to RateThrust message
    geometry_sub_ = nh.subscribe<mav_msgs::RateThrust>(mav_msgs::default_topics::COMMAND_RATE_THRUST, 1, 
                    &AttitudeControllerNode::geometryCallback, this);

}



void AttitudeControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // publishes the desired point only! 
  // Clear all pending commands

   commands_.clear();
   command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  attitude_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();


  //trajectory_msg
  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("AttitudeController got first MultiDOFJointTrajectory message.");
  }
}

void AttitudeControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO("IN InitalizePARAMS in AttitudeController");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  attitude_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  attitude_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  attitude_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  attitude_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  attitude_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  attitude_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  attitude_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  attitude_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  attitude_controller_.controller_parameters_.yaw_gain_kp_,
                  &attitude_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  attitude_controller_.controller_parameters_.yaw_gain_ki_,
                  &attitude_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  attitude_controller_.controller_parameters_.hovering_gain_kp_,
                  &attitude_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  attitude_controller_.controller_parameters_.hovering_gain_ki_,
                  &attitude_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  attitude_controller_.controller_parameters_.hovering_gain_kd_,
                  &attitude_controller_.controller_parameters_.hovering_gain_kd_);

  attitude_controller_.SetControllerGains();

  ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");


  //Reading the parameters come from the launch file

  std::string user;

  if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    attitude_controller_.user_ = user;
  }
  
}


AttitudeControllerNode::~AttitudeControllerNode() {}

void AttitudeControllerNode::geometryCallback(const mav_msgs::RateThrustConstPtr& rate_thrust) {
   last_rate_thrust_ = *rate_thrust;
}


void AttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  // This functions allows us to put the odometry message into the odometry variable--> _position,
  //_orientation,_velocit_body,_angular_velocity
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  attitude_controller_.SetCurrentStateFromOdometry(odometry);
  
}

void AttitudeControllerNode::writeToFile(double thrust, double theta_command, double phi_command) {
    std::ofstream file("/home/maxi/Crazyflie2-ros-gazebo-simulation/catkin_ws/src/CrazyS/rotors_control/src/nodes/att_contr.txt", std::ios::out | std::ios::app);
    
    time_t timestamp = std::time(nullptr);
    std::string timestamp_str = std::ctime(&timestamp);
    timestamp_str.pop_back(); // Remove the newline character

    if (file.is_open()) {
        file << "AttitudeControllerNode published rotorVelocities at " << timestamp_str << "\n";
        file << "Thrust x: " << thrust << "\n";
        file << "Theta: " << theta_command << "\n";
        file << "phi: " << phi_command << "\n";
        
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

}

void AttitudeControllerNode::UpdateController() {
  
  if(waypointHasBeenPublished_){
    EigenOdometry odometry;


    Eigen::Vector4d ref_rotor_velocities;
    
    // Compute control signals directly
    double delta_phi, delta_theta, delta_psi;
    double p_command, q_command, r_command;


    // // These two run in position_controller_node on different frequency. 
    // attitude_controller_.HoveringController(&attitude_controller_.control_t_.thrust);
    // attitude_controller_.XYController(&theta_command, &phi_command);

    
    
    
    // read in message that is published by position_controller_node
  

    double theta_command, phi_command;
    attitude_controller_.control_t_.thrust = last_rate_thrust_.thrust.x ;         
    theta_command = last_rate_thrust_.thrust.y ;
    phi_command =  last_rate_thrust_.thrust.z;

    // ROS_INFO("-----------------Received NEW--------------------------");
    // ROS_INFO("Thrust x: %f", attitude_controller_.control_t_.thrust);
    // ROS_INFO("Thrust y: %f", theta_command);
    // ROS_INFO("Thrust z: %f", phi_command);
    // ROS_INFO("-------------------------------------------");

    // Compute Attitude Controller
    attitude_controller_.AttitudeControllerFunction(&p_command, &q_command, theta_command, phi_command);

    // Compute Yaw Position Controller
    attitude_controller_.YawPositionController(&r_command);

    // Compute Rate Controller
    attitude_controller_.RateController(&delta_phi, &delta_theta, &delta_psi, p_command, q_command, r_command);

    // Compute Control Mixer
    double PWM_1, PWM_2, PWM_3, PWM_4;
    attitude_controller_.ControlMixer(attitude_controller_.control_t_.thrust, delta_phi, delta_theta, delta_psi, &PWM_1, &PWM_2, &PWM_3, &PWM_4);

    // Calculate Rotor Velocities
    attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities, PWM_1, PWM_2, PWM_3, PWM_4);



    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);

    actuator_msg->header.stamp = ros::Time::now();
    motor_velocity_reference_pub_.publish(actuator_msg);
    
    // This can be used to double check the frequency and see how many msg's are written to file per second.
    //writeToFile(attitude_controller_.control_t_.thrust, theta_command, phi_command);

  }
  
}

} // namespace rotors_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_controller_node");

    rotors_control::AttitudeControllerNode attitude_controller_node;

    ros::Rate rate(100); // 10 Hz

    while (ros::ok()) {
        attitude_controller_node.UpdateController();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
