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

#include "position_controller_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

namespace rotors_control {

PositionControllerNode::PositionControllerNode() {

    ROS_INFO("Started position controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");
    //PositionController position_controller_;

    InitializeParams();

    // Topics subscribe
    ROS_INFO("in position controller node. ");

    // Subscribe to command trajectory! Subscribe, so it will be able to get commands on where to go.
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
      &PositionControllerNode::OdometryCallback, this);

    //geometry_pub_ = nh.advertise<mav_msgs::RateThrust>(mav_msgs::default_topics::COMMAND_RATE_THRUST,1);

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    
}

PositionControllerNode::~PositionControllerNode(){}


void PositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // publishes the desired point only! 
  // Clear all pending commands.
  command_timer_.stop();
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

  ROS_INFO("Drone desired position in callback.XXX [x_d: %f, y_d: %f, z_d: %f]", eigen_reference.position_W[0],
        eigen_reference.position_W[1], eigen_reference.position_W[2]);


  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();


  //trajectory_msg
  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("PositionController got first MultiDOFJointTrajectory message.");
  }
}



void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO("IN InitalizePARAMS");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  position_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  position_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  position_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  position_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  position_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  position_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  position_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  position_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  position_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  position_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  position_controller_.controller_parameters_.yaw_gain_kp_,
                  &position_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  position_controller_.controller_parameters_.yaw_gain_ki_,
                  &position_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  position_controller_.controller_parameters_.hovering_gain_kp_,
                  &position_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  position_controller_.controller_parameters_.hovering_gain_ki_,
                  &position_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  position_controller_.controller_parameters_.hovering_gain_kd_,
                  &position_controller_.controller_parameters_.hovering_gain_kd_);

  position_controller_.SetControllerGains();

  ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");


  //Reading the parameters come from the launch file
  bool dataStoringActive;
  double dataStoringTime;
  std::string user;

  if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    position_controller_.user_ = user;
  }
  
}



void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  // This functions allows us to put the odometry message into the odometry variable--> _position,
  //_orientation,_velocit_body,_angular_velocity
  //ROS_INFO("Odometrycallback in positioncontrollernode");
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  position_controller_.SetCurrentStateFromOdometry(odometry);
}


void PositionControllerNode::UpdateController() {

   if(waypointHasBeenPublished_){
      Eigen::Vector4d ref_rotor_velocities;

      // Compute control signals directly
      double delta_phi, delta_theta, delta_psi;
      double p_command, q_command, r_command;
      double theta_command, phi_command;

      // Compute Hovering Controller
      position_controller_.HoveringController(&position_controller_.control_t_.thrust);

      // Compute XY Controller
      position_controller_.XYController(&theta_command, &phi_command);

      // Compute Attitude Controller
      position_controller_.AttitudeControllerFunction(&p_command, &q_command, &theta_command, &phi_command);

      // Compute Yaw Position Controller
      position_controller_.YawPositionController(&r_command);

      // Compute Rate Controller
      position_controller_.RateController(&delta_phi, &delta_theta, &delta_psi, &p_command, &q_command, &r_command);

    double PWM_1, PWM_2, PWM_3, PWM_4;
      position_controller_.ControlMixer(position_controller_.control_t_.thrust, delta_phi, delta_theta, delta_psi, &PWM_1, &PWM_2, &PWM_3, &PWM_4);


      position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &PWM_1, &PWM_2, &PWM_3, &PWM_4);

      // Publish actuator message
      mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
      actuator_msg->angular_velocities.clear();
      for (int i = 0; i < ref_rotor_velocities.size(); i++) {
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
      }

      actuator_msg->header.stamp = ros::Time::now();
      motor_velocity_reference_pub_.publish(actuator_msg);
   }
}




}





int main(int argc, char** argv){

  // eventually try and run a positioncontroller object and pass it to code?
  // Positioncontroller is not initialized.
    ROS_INFO("IN MAIN");
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");

    rotors_control::PositionControllerNode position_controller_node;
    // rotors_control::PositionController position_controller_example;

    double frequency=100;
    ros::Rate rate(frequency);  // 10 Hz
    //sampleTimefromHertz=

    while (ros::ok()) {
      //ROS_INFO("P");
        position_controller_node.UpdateController();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}