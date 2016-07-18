#ifndef __ROBOT_MANAGER_H__
#define __ROBOT_MANAGER_H__

#include <ros/ros.h>
#include <joint_mapping/LocalAgent.h>
#include <joint_mapping/Package.h>

#include <joint_mapping/asio_udp_utils.h>
#include <asio_udp_device/ASIOUDPDevice.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <joint_mapping/Utils.h>


class RobotManager {
public:
  explicit RobotManager() : n_robots(0), update_interval(1) {}
  ~RobotManager() {}

  bool initialize(const ros::NodeHandle& n);

private:
  // parameters load
  bool loadParameters(const ros::NodeHandle& n);
  
  // ROS topics callback functions
  
  boost::shared_ptr<Network> network_;

  // agent pointer
  std::string robot_name;
  comap::LocalAgent::Ptr agentPtr;

  unsigned int n_robots; // number of robot agents
  
  // settings
  unsigned int update_interval;
  unsigned int update_counter;
  
  std::string name; // rosnode name
  unsigned char robot_ID;
  
};

#endif
