#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>


void publishTrajectory(const ros::Publisher& publisher, const Eigen::Vector3d& position, double yaw) {
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw, &trajectory_msg);
  publisher.publish(trajectory_msg);
  ROS_INFO("Publishing waypoint: [%f, %f, %f].", position.x(), position.y(), position.z());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "figure_eight_example");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started figure-eight example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  ros::Duration(5.0).sleep();

  // Define waypoints for the figure-eight pattern
  std::vector<Eigen::Vector3d> waypoints = {
    {1.0, 0.0, 1.0},
    {0.5, 1.0, 1.0},
    {0.0, 0.0, 1.0},
    {-0.5, -1.0, 1.0},
    {-1.0, 0.0, 1.0},
    {-0.5, 1.0, 1.0},
    {0.0, 0.0, 1.0},
    {0.5, -1.0, 1.0},
  };

  double yaw = 0.0; // Desired yaw

  for (const auto& waypoint : waypoints) {
    publishTrajectory(trajectory_pub, waypoint, yaw);
    ros::Duration(5.0).sleep(); 
  }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}